#include "core/Library.hpp"

#include "core/Extrude.hpp"
#include "core/Scene.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <functional>

namespace sicdm {

namespace {

constexpr float kPressureAngle = 20.0f * kPi / 180.0f;

// ---------------------------------------------------------------------------
// Mesh building helpers (flat-shaded faces; edges come from computeFeatureEdges).

struct Builder {
    MeshData m;

    static bool same(const Vec3& p, const Vec3& q) { return length(p - q) < 1e-6f; }

    // Quad a-b-c-d; flipped if its normal does not face `hint`. A quad with two
    // coincident corners becomes a triangle.
    void quad(Vec3 a, Vec3 b, Vec3 c, Vec3 d, const Vec3& hint) {
        if (same(a, b)) return tri(a, c, d, hint);
        if (same(b, c)) return tri(a, b, d, hint);
        if (same(c, d)) return tri(a, b, c, hint);
        if (same(d, a)) return tri(a, b, c, hint);
        Vec3 n = cross(c - a, d - b);
        if (length(n) < 1e-12f) return;
        n = normalize(n);
        if (dot(n, hint) < 0.0f) {
            std::swap(b, d);
            n = n * -1.0f;
        }
        const auto base = static_cast<std::uint32_t>(m.vertices.size());
        m.vertices.insert(m.vertices.end(), {{a, n}, {b, n}, {c, n}, {d, n}});
        m.indices.insert(m.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
    }
    void tri(Vec3 a, Vec3 b, Vec3 c, const Vec3& hint) {
        Vec3 n = cross(b - a, c - a);
        if (length(n) < 1e-12f) return;
        n = normalize(n);
        if (dot(n, hint) < 0.0f) {
            std::swap(b, c);
            n = n * -1.0f;
        }
        const auto base = static_cast<std::uint32_t>(m.vertices.size());
        m.vertices.insert(m.vertices.end(), {{a, n}, {b, n}, {c, n}});
        m.indices.insert(m.indices.end(), {base, base + 1, base + 2});
    }
    MeshData finish(float edgeAngle = 25.0f) {
        m.computeFeatureEdges(edgeAngle);
        return std::move(m);
    }
};

Vec2 polar(float r, float a) { return {r * std::cos(a), r * std::sin(a)}; }
Vec3 at(const Vec2& p, float z) { return {p.x, p.y, z}; }

// Rotation and scale of a 2D outline (one layer of a helical / bevel gear).
std::vector<Vec2> transformed(const std::vector<Vec2>& pts, float angle, float scale) {
    const float c = std::cos(angle) * scale, s = std::sin(angle) * scale;
    std::vector<Vec2> out(pts.size());
    for (std::size_t i = 0; i < pts.size(); ++i) out[i] = {pts[i].x * c - pts[i].y * s, pts[i].x * s + pts[i].y * c};
    return out;
}

// Solid between an inner and an outer closed loop (matched point by point, both
// counter-clockwise around the Z axis) stacked through `layers` (z, inner, outer).
// An empty inner loop closes the faces at the centre.
struct Layer {
    float z;
    std::vector<Vec2> inner, outer;
};

MeshData loft(const std::vector<Layer>& layers) {
    Builder b;
    const std::size_t n = layers.front().outer.size();
    const bool hollow = !layers.front().inner.empty();
    auto cap = [&](const Layer& l, float dir) {
        for (std::size_t i = 0; i < n; ++i) {
            const std::size_t j = (i + 1) % n;
            if (hollow) b.quad(at(l.inner[i], l.z), at(l.inner[j], l.z), at(l.outer[j], l.z), at(l.outer[i], l.z), {0, 0, dir});
            else b.tri(at({0, 0}, l.z), at(l.outer[i], l.z), at(l.outer[j], l.z), {0, 0, dir});
        }
    };
    cap(layers.front(), -1.0f);
    cap(layers.back(), 1.0f);
    for (std::size_t k = 0; k + 1 < layers.size(); ++k) {
        const Layer& lo = layers[k];
        const Layer& hi = layers[k + 1];
        for (std::size_t i = 0; i < n; ++i) {
            const std::size_t j = (i + 1) % n;
            // Outward = away from the axis, through the middle of the face.
            const Vec2 mo{(lo.outer[i].x + lo.outer[j].x) * 0.5f, (lo.outer[i].y + lo.outer[j].y) * 0.5f};
            Vec2 edge{lo.outer[j].x - lo.outer[i].x, lo.outer[j].y - lo.outer[i].y};
            Vec3 out{edge.y, -edge.x, 0}; // right-hand normal of a counter-clockwise loop
            if (length(out) < 1e-9f) out = {mo.x, mo.y, 0};
            b.quad(at(lo.outer[i], lo.z), at(lo.outer[j], lo.z), at(hi.outer[j], hi.z), at(hi.outer[i], hi.z), out);
            if (hollow) {
                edge = {lo.inner[j].x - lo.inner[i].x, lo.inner[j].y - lo.inner[i].y};
                b.quad(at(lo.inner[i], lo.z), at(lo.inner[j], lo.z), at(hi.inner[j], hi.z), at(hi.inner[i], hi.z),
                       {-edge.y, edge.x, 0});
            }
        }
    }
    return b.finish();
}

// Loop of points at radius r, at the same polar angles as `like`.
std::vector<Vec2> circleLike(const std::vector<Vec2>& like, float r) {
    std::vector<Vec2> out(like.size());
    for (std::size_t i = 0; i < like.size(); ++i) out[i] = polar(r, std::atan2(like[i].y, like[i].x));
    return out;
}

// Straight prism of a simple counter-clockwise polygon between z0 and z1.
MeshData prismOf(const std::vector<Vec2>& poly, float z0, float z1) {
    Builder b;
    const auto tris = triangulatePolygon(poly);
    for (std::size_t t = 0; t + 2 < tris.size(); t += 3) {
        const Vec2 &p0 = poly[tris[t]], &p1 = poly[tris[t + 1]], &p2 = poly[tris[t + 2]];
        b.tri(at(p0, z0), at(p1, z0), at(p2, z0), {0, 0, -1});
        b.tri(at(p0, z1), at(p1, z1), at(p2, z1), {0, 0, 1});
    }
    for (std::size_t i = 0; i < poly.size(); ++i) {
        const Vec2 &p = poly[i], &q = poly[(i + 1) % poly.size()];
        b.quad(at(p, z0), at(q, z0), at(q, z1), at(p, z1), {q.y - p.y, -(q.x - p.x), 0});
    }
    return b.finish();
}

MeshData cylinderAt(float x, float y, float r, float z0, float z1, int seg = 48) {
    MeshData m = primitives::cylinder({x, y, z0}, r, z1 - z0, seg);
    return m;
}

MeshData boxOf(Vec3 lo, Vec3 hi) { return primitives::box(lo, hi); }

// ---------------------------------------------------------------------------
// Gears

float involute(float a) { return std::tan(a) - a; }

MeshData spurLikeGear(const std::vector<float>& p) {
    const float m = p[0];
    const int z = static_cast<int>(p[1]);
    const float w = p[2], bore = p.size() > 3 ? p[3] : 0.0f;
    const float helix = p.size() > 4 ? radians(p[4]) : 0.0f;
    const int style = p.size() > 5 ? static_cast<int>(p[5]) : 0;
    const float phase = p.size() > 6 ? radians(p[6]) : 0.0f;
    const float rp = m * static_cast<float>(z) * 0.5f;
    const std::vector<Vec2> outline = gearOutline(m, z, phase);
    const float rf = rp - 1.25f * m;
    const std::vector<Vec2> inner = bore > 0.0f && bore < rf * 0.9f ? circleLike(outline, bore) : std::vector<Vec2>{};
    const int layers = style == 0 ? 1 : 16;
    std::vector<Layer> ls;
    for (int k = 0; k <= layers; ++k) {
        const float zz = w * static_cast<float>(k) / static_cast<float>(layers);
        float twist = 0.0f;
        if (style == 1) twist = zz * std::tan(helix) / rp;
        if (style == 2) twist = std::min(zz, w - zz) * std::tan(helix) / rp;
        ls.push_back({zz, inner.empty() ? inner : transformed(inner, twist, 1.0f), transformed(outline, twist, 1.0f)});
    }
    return loft(ls);
}

MeshData ringGear(const std::vector<float>& p) {
    const float m = p[0];
    const int z = static_cast<int>(p[1]);
    const float w = p[2], rim = p.size() > 3 ? std::max(p[3], 0.5f * m) : 2.0f * m;
    const float phase = p.size() > 4 ? radians(p[4]) : 0.0f;
    const float rp = m * static_cast<float>(z) * 0.5f;
    // The tooth spaces of an internal gear have the shape of an external gear's teeth.
    const std::vector<Vec2> inner = gearOutline(m, z, phase + kPi / static_cast<float>(z), 1.25f, 1.0f);
    const std::vector<Vec2> outer = circleLike(inner, rp + 1.25f * m + rim);
    return loft({{0.0f, inner, outer}, {w, inner, outer}});
}

MeshData bevelGear(const std::vector<float>& p) {
    const float m = p[0];
    const int z = static_cast<int>(p[1]);
    const float face = p[2], cone = radians(std::clamp(p[3], 5.0f, 85.0f));
    const float bore = p.size() > 4 ? p[4] : 0.0f;
    const float phase = p.size() > 5 ? radians(p[5]) : 0.0f;
    const float rp = m * static_cast<float>(z) * 0.5f;
    const float apex = rp / std::sin(cone); // cone distance
    const float f = std::min(face, apex * 0.6f);
    const std::vector<Vec2> outline = gearOutline(m, z, phase);
    const std::vector<Vec2> inner = bore > 0.0f ? circleLike(outline, bore) : std::vector<Vec2>{};
    std::vector<Layer> ls;
    constexpr int kLayers = 8;
    for (int k = 0; k <= kLayers; ++k) {
        const float u = f * static_cast<float>(k) / kLayers; // along the cone
        const float s = 1.0f - u / apex;
        ls.push_back({u * std::cos(cone), inner, transformed(outline, 0.0f, s)});
    }
    for (Layer& l : ls) // keep the bore inside the shrinking teeth
        if (!l.inner.empty()) {
            const float maxR = (rp - 1.25f * m) * (1.0f - (l.z / std::cos(cone)) / apex) * 0.9f;
            if (bore > maxR) l.inner = circleLike(outline, maxR);
        }
    return loft(ls);
}

MeshData rack(const std::vector<float>& p) {
    const float m = p[0];
    const int n = std::max(1, static_cast<int>(p[1]));
    const float w = p[2], h = p.size() > 3 ? std::max(p[3], 0.5f * m) : 2.0f * m;
    const float pitch = kPi * m, len = pitch * static_cast<float>(n);
    const float yt = m, yr = -1.25f * m, yb = yr - h;
    const float t = std::tan(kPressureAngle);
    const float tipHW = pitch * 0.25f - m * t, rootHW = pitch * 0.25f + 1.25f * m * t;
    std::vector<Vec2> poly{{-len * 0.5f, yb}, {len * 0.5f, yb}, {len * 0.5f, yr}};
    for (int i = n - 1; i >= 0; --i) {
        const float xc = (static_cast<float>(i) + 0.5f) * pitch - len * 0.5f;
        poly.push_back({xc + rootHW, yr});
        poly.push_back({xc + tipHW, yt});
        poly.push_back({xc - tipHW, yt});
        poly.push_back({xc - rootHW, yr});
    }
    poly.push_back({-len * 0.5f, yr});
    return prismOf(poly, 0.0f, w);
}

MeshData worm(const std::vector<float>& p) {
    const float m = p[0];
    const int starts = std::max(1, static_cast<int>(p[1]));
    const float len = p[2];
    const float phase = p.size() > 3 ? p[3] / 360.0f : 0.0f;
    const float rp = 5.0f * m; // diameter quotient 10
    const float ra = rp + m, rf = rp - 1.25f * m;
    const float pitch = kPi * m, lead = pitch * static_cast<float>(starts);
    const float t = std::tan(kPressureAngle);
    const float tipHW = pitch * 0.25f - m * t, rootHW = pitch * 0.25f + 1.25f * m * t;
    auto radius = [&](float x, float a) {
        float u = (x - lead * a / (2.0f * kPi)) / pitch + phase;
        u -= std::floor(u);
        const float d = std::fabs(u - 0.5f) * pitch;
        if (d <= tipHW) return ra;
        if (d >= rootHW) return rf;
        return ra - (d - tipHW) / (rootHW - tipHW) * (ra - rf);
    };
    const int na = 72, nx = std::max(24, static_cast<int>(len / pitch * 20.0f));
    auto pt = [&](int ix, int ia) {
        const float x = -len * 0.5f + len * static_cast<float>(ix) / static_cast<float>(nx);
        const float a = 2.0f * kPi * static_cast<float>(ia) / static_cast<float>(na);
        const float r = radius(x, a);
        return Vec3{x, r * std::cos(a), r * std::sin(a)};
    };
    Builder b;
    for (int ix = 0; ix < nx; ++ix)
        for (int ia = 0; ia < na; ++ia) {
            const Vec3 p00 = pt(ix, ia), p10 = pt(ix + 1, ia), p11 = pt(ix + 1, ia + 1), p01 = pt(ix, ia + 1);
            const float a = 2.0f * kPi * (static_cast<float>(ia) + 0.5f) / static_cast<float>(na);
            b.quad(p00, p10, p11, p01, {0, std::cos(a), std::sin(a)});
        }
    for (int end = 0; end <= nx; end += nx) // end faces
        for (int ia = 0; ia < na; ++ia)
            b.tri({pt(end, 0).x, 0, 0}, pt(end, ia), pt(end, ia + 1), {end == 0 ? -1.0f : 1.0f, 0, 0});
    MeshData mesh = b.finish(35.0f);
    // Shaft through the worm, along X.
    MeshData shaft = primitives::cylinder({0, 0, -len * 0.5f - 0.6f * rp}, 0.45f * rp, len + 1.2f * rp, 32);
    shaft.transform(rotateAxis({0, 1, 0}, kPi * 0.5f));
    mesh.append(shaft);
    return mesh;
}

// ---------------------------------------------------------------------------
// Motors (housing: shaft along +Z through the origin, base at z = 0)

MeshData dShaft(float r, float z0, float z1) {
    // Round shaft with one flat, so its rotation is visible.
    std::vector<Vec2> poly;
    const float flat = r * 0.55f;
    for (int i = 0; i < 40; ++i) {
        const Vec2 q = polar(r, 2.0f * kPi * static_cast<float>(i) / 40.0f);
        poly.push_back({std::min(q.x, flat), q.y});
    }
    std::vector<Vec2> clean; // drop duplicates created by the clamp
    for (const Vec2& q : poly)
        if (clean.empty() || std::hypot(q.x - clean.back().x, q.y - clean.back().y) > 1e-4f) clean.push_back(q);
    return prismOf(clean, z0, z1);
}

// Coupler on the shaft with a set screw, the part that shows the rotation.
MeshData coupler(float r, float z0, float z1) {
    MeshData m = cylinderAt(0, 0, r, z0, z1);
    const float s = r * 0.28f;
    m.append(boxOf({r * 0.85f, -s, (z0 + z1) * 0.5f - s}, {r * 1.25f, s, (z0 + z1) * 0.5f + s}));
    return m;
}

MeshData dcMotor() {
    MeshData m = cylinderAt(0, 0, 12.0f, 0.0f, 30.0f, 64);   // can
    m.append(cylinderAt(0, 0, 10.5f, -3.0f, 0.0f, 48));      // end bell
    m.append(cylinderAt(0, 0, 4.5f, 30.0f, 32.0f, 32));      // bearing boss
    m.append(boxOf({-6.0f, -1.2f, -7.0f}, {-4.0f, 1.2f, -3.0f})); // terminals
    m.append(boxOf({4.0f, -1.2f, -7.0f}, {6.0f, 1.2f, -3.0f}));
    m.append(boxOf({-12.4f, -3.0f, 8.0f}, {-11.6f, 3.0f, 22.0f})); // flat label strip
    return m;
}

MeshData stepperMotor() {
    // NEMA 17: 42.3 mm square, chamfered corners, 40 mm long.
    const float h = 21.15f, c = 5.0f;
    const std::vector<Vec2> oct{{-h + c, -h}, {h - c, -h}, {h, -h + c}, {h, h - c},
                                {h - c, h},   {-h + c, h}, {-h, h - c}, {-h, -h + c}};
    MeshData m = prismOf(oct, 0.0f, 9.0f);                           // rear end cap
    m.append(prismOf(transformed(oct, 0.0f, 0.97f), 9.0f, 31.0f));  // stator (slightly recessed)
    m.append(prismOf(oct, 31.0f, 40.0f));                           // front end cap
    m.append(cylinderAt(0, 0, 11.0f, 40.0f, 42.0f, 48));            // pilot boss
    for (float x : {-15.5f, 15.5f})
        for (float y : {-15.5f, 15.5f}) m.append(cylinderAt(x, y, 2.2f, 40.0f, 40.6f, 20)); // screw heads
    m.append(boxOf({-8.0f, -h - 5.0f, 2.0f}, {8.0f, -h, 9.0f}));    // connector
    return m;
}

MeshData servoMotor() {
    // Standard hobby servo; output spline on the Z axis.
    MeshData m = boxOf({-30.0f, -10.0f, 0.0f}, {10.5f, 10.0f, 36.0f});
    m.append(boxOf({-37.5f, -10.0f, 26.0f}, {17.5f, 10.0f, 28.5f})); // mounting flanges
    m.append(cylinderAt(0, 0, 6.5f, 36.0f, 38.0f, 48));              // gear tower
    m.append(cylinderAt(-9.0f, 0, 4.0f, 36.0f, 37.5f, 32));
    m.append(boxOf({-34.0f, -1.5f, 6.0f}, {-30.0f, 1.5f, 9.0f}));    // cable gland
    return m;
}

// ---------------------------------------------------------------------------
// Electric car (traction) motor: horizontal axis along +X at height kEvAxis,
// standing on mounting feet at z = 0. Front end shield faces +X.

constexpr float kEvAxis = 170.0f;   // shaft centre height
constexpr float kEvFront = 132.0f;  // x of the front bearing seal face (shaft starts here)
constexpr float kEvBack = -130.0f;  // x of the rear bearing cap

// Cylinder / tube along +X from x0 to x1, centred on (y, z).
MeshData cylX(float x0, float x1, float r, float y, float z, int seg = 64) {
    MeshData m = primitives::cylinder({0, 0, 0}, r, x1 - x0, seg);
    m.transform(translate({x0, y, z}) * rotateAxis({0, 1, 0}, kPi * 0.5f));
    return m;
}
MeshData tubeX(float x0, float x1, float ro, float ri, float y, float z, int seg = 64) {
    MeshData m = primitives::tube({0, 0, 0}, ro, ri, x1 - x0, seg);
    m.transform(translate({x0, y, z}) * rotateAxis({0, 1, 0}, kPi * 0.5f));
    return m;
}
// Small part on the end shield at polar angle a, radius r (in the YZ plane).
Vec3 onShield(float x, float r, float a) { return {x, r * std::cos(a), kEvAxis + r * std::sin(a)}; }

MeshData evMotor() {
    const float h = kEvAxis;
    MeshData m = cylX(-110.0f, 90.0f, 140.0f, 0, h, 96);            // stator frame
    m.append(cylX(-122.0f, -110.0f, 132.0f, 0, h, 96));             // rear end cap
    m.append(cylX(kEvBack, -122.0f, 46.0f, 0, h, 48));              // rear bearing cap
    m.append(cylX(90.0f, 100.0f, 146.0f, 0, h, 96));                // front flange
    m.append(cylX(100.0f, 110.0f, 132.0f, 0, h, 96));               // front end shield
    m.append(tubeX(110.0f, 114.0f, 128.0f, 118.0f, 0, h, 96));      // shield rim
    for (int i = 0; i < 8; ++i) {                                   // cast ribs
        MeshData rib = boxOf({110.0f, 44.0f, -4.0f}, {119.0f, 120.0f, 4.0f});
        rib.transform(translate({0, 0, h}) * rotateAxis({1, 0, 0}, kPi * 0.25f * static_cast<float>(i) + kPi / 8.0f));
        m.append(rib);
    }
    m.append(cylX(110.0f, 124.0f, 50.0f, 0, h, 64));                // bearing hub
    for (int i = 0; i < 10; ++i) {                                  // hub bolt pattern
        const Vec3 b = onShield(124.0f, 38.0f, 2.0f * kPi * static_cast<float>(i) / 10.0f);
        m.append(cylX(b.x, b.x + 1.5f, 3.6f, b.y, b.z, 12));
    }
    m.append(cylX(124.0f, kEvFront, 30.0f, 0, h, 48));              // seal housing
    for (int i = 0; i < 18; ++i) {                                  // flange bolt circle
        const Vec3 b = onShield(100.0f, 139.0f, 2.0f * kPi * static_cast<float>(i) / 18.0f);
        m.append(cylX(b.x, b.x + 3.0f, 4.2f, b.y, b.z, 12));
    }
    for (int i = 0; i < 12; ++i) {                                  // rear cap bolts
        const Vec3 b = onShield(-126.0f, 126.0f, 2.0f * kPi * static_cast<float>(i) / 12.0f);
        m.append(cylX(b.x, b.x + 4.0f, 3.6f, b.y, b.z, 12));
    }
    // Terminal box on top with three cable glands facing forward.
    m.append(boxOf({-90.0f, -48.0f, h + 118.0f}, {10.0f, 48.0f, h + 178.0f}));
    m.append(boxOf({-84.0f, -42.0f, h + 178.0f}, {4.0f, 42.0f, h + 184.0f}));  // lid
    m.append(boxOf({-60.0f, -30.0f, h + 100.0f}, {-20.0f, 30.0f, h + 120.0f})); // plinth
    for (float y : {-28.0f, 0.0f, 28.0f}) {
        m.append(cylX(10.0f, 18.0f, 10.0f, y, h + 148.0f, 6));      // hex nut
        m.append(cylX(18.0f, 30.0f, 7.5f, y, h + 148.0f, 24));      // gland body
    }
    // Coolant stubs on the side and a lifting lug at the front.
    for (float x : {-40.0f, 30.0f}) {
        MeshData pipe = primitives::cylinder({0, 0, 0}, 8.0f, 30.0f, 24);
        pipe.transform(translate({x, -136.0f, h + 50.0f}) * rotateAxis({1, 0, 0}, kPi * 0.5f));
        m.append(pipe);
        MeshData barb = primitives::cylinder({0, 0, 0}, 9.5f, 5.0f, 24);
        barb.transform(translate({x, -160.0f, h + 50.0f}) * rotateAxis({1, 0, 0}, kPi * 0.5f));
        m.append(barb);
    }
    {
        MeshData lug = boxOf({74.0f, -6.0f, -24.0f}, {92.0f, 6.0f, 26.0f});
        lug.transform(translate({0, 0, h}) * rotateAxis({1, 0, 0}, kPi * 0.30f) * translate({0, 0, 140.0f}));
        m.append(lug);
    }
    // Mounting feet with bolt bosses.
    for (float side : {-1.0f, 1.0f}) {
        const float y0 = side * 70.0f, y1 = side * 132.0f;
        m.append(boxOf({-100.0f, std::min(y0, y1), 0.0f}, {80.0f, std::max(y0, y1), 16.0f}));
        m.append(boxOf({-90.0f, std::min(side * 60.0f, side * 112.0f), 16.0f}, {70.0f, std::max(side * 60.0f, side * 112.0f), 70.0f}));
        for (float x : {-80.0f, 60.0f}) m.append(cylinderAt(x, side * 118.0f, 7.0f, 16.0f, 19.0f, 16));
    }
    return m;
}

// Traction motor rotor: stepped output shaft with a key and a hollow (splined) end.
MeshData evRotor(float len, int shafts, float rearLen, float dia, bool coupling) {
    const float h = kEvAxis, r = dia * 0.5f;
    if (shafts == 0) { // flush output flange with a drive dog
        MeshData m = cylX(kEvFront, kEvFront + 6.0f, std::max(r * 1.1f, 20.0f), 0, h);
        m.append(boxOf({kEvFront + 6.0f, -4.0f, h + r * 0.4f}, {kEvFront + 10.0f, 4.0f, h + std::max(r, 18.0f)}));
        return m;
    }
    MeshData m = cylX(kEvFront, kEvFront + std::min(12.0f, len), r * 1.25f, 0, h); // collar
    const float tip = kEvFront + len;
    const float hollow = std::min(len * 0.3f, dia * 0.8f);
    if (len > 12.0f) {
        m.append(cylX(kEvFront + 12.0f, tip - hollow, r, 0, h));
        m.append(tubeX(tip - hollow, tip, r, r * 0.55f, 0, h)); // splined bore at the end
        const float kw = std::max(r * 0.28f, 1.0f);              // key along the shaft
        m.append(boxOf({kEvFront + 18.0f, -kw, h + r - kw * 0.6f}, {std::max(kEvFront + 20.0f, tip - hollow - 6.0f), kw, h + r + kw}));
    }
    if (coupling && len > 12.0f) { // flange coupling on the shaft end
        const float cr = r * 2.2f;
        m.append(cylX(tip - std::min(len * 0.25f, r * 1.6f), tip, cr, 0, h, 64));
        for (int i = 0; i < 6; ++i) {
            const Vec3 b = onShield(tip, cr * 0.72f, 2.0f * kPi * static_cast<float>(i) / 6.0f);
            m.append(cylX(b.x, b.x + 2.0f, std::max(r * 0.16f, 1.5f), b.y, b.z, 12));
        }
    }
    if (shafts == 2) m.append(cylX(kEvBack - rearLen, kEvBack, r * 0.8f, 0, h));
    return m;
}

// ---------------------------------------------------------------------------
// Flange-mount electric car motor: ribbed cylindrical frame, axis along +X at
// height kEv2Axis (resting on its ribs), splined output shaft.

constexpr float kEv2Axis = 120.0f;
constexpr float kEv2Front = 140.0f; // bearing seal face
constexpr float kEv2Back = -197.0f; // rear of the connector housing

MeshData eyeBolt(float x, float y, float zBase) {
    MeshData m = cylinderAt(x, y, 12.0f, zBase - 2.0f, zBase + 4.0f, 24); // boss
    m.append(cylinderAt(x, y, 4.5f, zBase + 4.0f, zBase + 12.0f, 16));    // stem
    MeshData ring = primitives::torus({0, 0, 0}, 10.0f, 3.2f, 32, 12);    // eye, standing up
    ring.transform(translate({x, y, zBase + 24.0f}) * rotateAxis({0, 1, 0}, kPi * 0.5f));
    m.append(ring);
    return m;
}

MeshData evFlangeMotor() {
    const float h = kEv2Axis;
    MeshData m = cylX(-160.0f, 120.0f, 112.0f, 0, h, 96);                     // frame
    for (int i = 0; i < 8; ++i) {                                              // longitudinal ribs
        MeshData rib = boxOf({-150.0f, -8.0f, 108.0f}, {112.0f, 8.0f, 116.0f});
        rib.transform(translate({0, 0, h}) * rotateAxis({1, 0, 0}, kPi * 0.25f * static_cast<float>(i) + kPi / 8.0f));
        m.append(rib);
    }
    m.append(cylX(112.0f, 124.0f, 116.0f, 0, h, 96));                         // front flange band
    m.append(cylX(124.0f, 134.0f, 108.0f, 0, h, 96));                         // front face
    m.append(tubeX(134.0f, 137.0f, 46.0f, 33.0f, 0, h, 64));                  // bearing seal ring
    m.append(cylX(134.0f, 136.0f, 33.0f, 0, h, 48));
    for (int i = 0; i < 8; ++i) {                                             // face bolt holes (rings)
        const Vec3 b = onShield(134.0f, 90.0f, 2.0f * kPi * static_cast<float>(i) / 8.0f + kPi / 8.0f);
        m.append(tubeX(b.x, b.x + 1.2f, 6.0f, 3.5f, b.y, b.z - kEvAxis + h, 16));
    }
    for (int i = 0; i < 4; ++i) {                                             // rim bosses with hex bolts
        const float a = kPi * 0.5f * static_cast<float>(i) + kPi * 0.25f;
        const Vec3 b{0.0f, 117.0f * std::cos(a), h + 117.0f * std::sin(a)};
        m.append(cylX(112.0f, 134.0f, 9.0f, b.y, b.z, 20));
        m.append(cylX(134.0f, 138.0f, 6.5f, b.y, b.z, 6));
    }
    m.append(cylX(-175.0f, -160.0f, 108.0f, 0, h, 96));                       // rear cap
    m.append(cylX(kEv2Back, -175.0f, 78.0f, 0, h, 64));                       // connector housing
    for (int i = 0; i < 6; ++i) {                                             // rear cap bolts
        const float a = 2.0f * kPi * static_cast<float>(i) / 6.0f;
        m.append(cylX(-179.0f, -175.0f, 5.0f, 96.0f * std::cos(a), h + 96.0f * std::sin(a), 6));
    }
    for (float y : {-30.0f, 0.0f, 30.0f}) m.append(cylX(kEv2Back - 10.0f, kEv2Back, 8.0f, y, h + 40.0f, 16)); // connectors
    m.append(eyeBolt(-120.0f, 0.0f, h + 112.0f));                             // lifting eyes
    m.append(eyeBolt(96.0f, 0.0f, h + 112.0f));
    // Coolant elbow at the rear: out of the side, then up, with a cap.
    {
        MeshData out = primitives::cylinder({0, 0, 0}, 6.0f, 34.0f, 24);
        out.transform(translate({-150.0f, -100.0f, h + 50.0f}) * rotateAxis({1, 0, 0}, kPi * 0.5f));
        m.append(out);
        m.append(primitives::sphere({-150.0f, -134.0f, h + 50.0f}, 6.0f, 16, 8));
        m.append(cylinderAt(-150.0f, -134.0f, 6.0f, h + 50.0f, h + 92.0f, 24));
        m.append(cylinderAt(-150.0f, -134.0f, 8.0f, h + 92.0f, h + 104.0f, 24)); // cap
    }
    return m;
}

// Splined output shaft (involute spline, 16 teeth) with a collar; plain is round.
MeshData evSplineRotor(float len, int shafts, float rearLen, float dia, bool plain) {
    const float h = kEv2Axis, r = dia * 0.5f;
    auto alongX = [&](MeshData m, float x0) {
        m.transform(translate({x0, 0, h}) * rotateAxis({0, 1, 0}, kPi * 0.5f));
        return m;
    };
    if (shafts == 0) {
        MeshData m = cylX(kEv2Front, kEv2Front + 5.0f, std::max(r * 1.2f, 16.0f), 0, h);
        m.append(boxOf({kEv2Front + 5.0f, -3.0f, h + r * 0.3f}, {kEv2Front + 8.0f, 3.0f, h + std::max(r, 14.0f)}));
        return m;
    }
    const float collar = std::min(10.0f, len * 0.3f);
    MeshData m = cylX(kEv2Front, kEv2Front + collar, r * 1.35f, 0, h, 48);
    const float shaftLen = len - collar;
    if (shaftLen > 0.5f) {
        if (plain) {
            m.append(cylX(kEv2Front + collar, kEv2Front + len, r, 0, h, 48));
        } else {
            constexpr int kTeeth = 16;
            const float module = dia / static_cast<float>(kTeeth + 2);
            // Spline: gear-like profile, shallow teeth (addendum 0.8 m, dedendum 0.9 m).
            const std::vector<Vec2> outline = gearOutline(module, kTeeth, 0.0f, 0.8f, 0.9f);
            const float mark = std::min(0.6f, shaftLen * 0.1f); // centre boss at the tip, flush with the length
            m.append(alongX(prismOf(outline, 0.0f, shaftLen - mark), kEv2Front + collar));
            m.append(cylX(kEv2Front + len - mark, kEv2Front + len, r * 0.25f, 0, h, 16));
        }
    }
    if (shafts == 2) m.append(cylX(kEv2Back - rearLen, kEv2Back, r * 0.8f, 0, h));
    return m;
}

MeshData servoHorn(int horn) {
    MeshData m = cylinderAt(0, 0, 2.5f, 38.0f, 41.0f, 24); // spline
    m.append(cylinderAt(0, 0, 4.5f, 41.0f, 45.0f, 32));   // hub
    auto arm = [&](float angle, float from, float to) {
        MeshData a = boxOf({from, -3.0f, 41.0f}, {to, 3.0f, 43.5f});
        a.append(cylinderAt(to, 0, 3.0f, 41.0f, 43.5f, 24));
        for (float x = to - 3.0f; x > std::max(from, 4.0f) + 1.0f; x -= 4.0f) a.append(cylinderAt(x, 0, 0.7f, 43.5f, 43.8f, 12));
        a.transform(rotateAxis({0, 0, 1}, angle));
        m.append(a);
    };
    switch (std::clamp(horn, 0, 3)) {
    case 0: arm(0.0f, -4.0f, 17.0f); break;                                    // single arm
    case 1: arm(0.0f, 0.0f, 16.0f); arm(kPi, 0.0f, 16.0f); break;              // double arm
    case 2: for (int i = 0; i < 4; ++i) arm(kPi * 0.5f * static_cast<float>(i), 0.0f, 13.0f); break; // cross
    default:                                                                  // disc with holes
        m.append(cylinderAt(0, 0, 11.0f, 41.0f, 43.0f, 64));
        for (int i = 0; i < 8; ++i) {
            const float a = 2.0f * kPi * static_cast<float>(i) / 8.0f;
            m.append(cylinderAt(8.0f * std::cos(a), 8.0f * std::sin(a), 0.8f, 43.0f, 43.3f, 12));
        }
        m.append(boxOf({8.5f, -0.8f, 43.0f}, {10.5f, 0.8f, 43.4f})); // index mark
        break;
    }
    return m;
}

// DC / stepper rotor: front shaft (optional coupler), optional rear shaft, or a
// flush output disc when the motor has no shaft.
MeshData motorRotor(const std::vector<float>& raw) {
    const std::vector<float> p = withDefaults("motor_rotor", raw);
    const int style = std::clamp(static_cast<int>(p[0]), 0, 4);
    if (style == 4)
        return evSplineRotor(std::max(p[1], 0.5f), std::clamp(static_cast<int>(std::lround(p[2])), 0, 2), std::max(p[3], 0.5f),
                             std::max(p[4], 4.0f), p[5] >= 0.5f);
    if (style == 2) return servoHorn(static_cast<int>(p[1]));
    if (style == 3)
        return evRotor(std::max(p[1], 0.5f), std::clamp(static_cast<int>(std::lround(p[2])), 0, 2), std::max(p[3], 0.5f),
                       std::max(p[4], 1.0f), p[5] >= 0.5f);
    const float front = style == 0 ? 32.0f : 42.0f; // face of the bearing boss
    const float back = style == 0 ? -3.0f : 0.0f;   // rear face of the housing
    const float len = std::max(p[1], 0.5f);
    const int shafts = std::clamp(static_cast<int>(std::lround(p[2])), 0, 2);
    const float rearLen = std::max(p[3], 0.5f);
    const float r = std::max(p[4], 0.2f) * 0.5f;
    const bool fitting = p[5] >= 0.5f;
    if (shafts == 0) {
        // No shaft: a flush output disc with an index mark shows the rotation.
        MeshData m = cylinderAt(0, 0, std::max(r * 1.8f, 2.0f), front, front + 0.8f, 32);
        const float w = std::max(r * 0.35f, 0.4f);
        m.append(boxOf({std::max(r * 0.6f, 0.6f), -w, front + 0.8f}, {std::max(r * 1.7f, 1.9f), w, front + 1.2f}));
        return m;
    }
    MeshData m = dShaft(r, front, front + len);
    if (fitting && len > 3.0f) {
        const float cr = std::max(r * 2.6f, r + 2.5f);
        const float cl = std::min(len * 0.45f, 2.0f * cr);
        const float top = front + len - std::min(0.5f, len * 0.05f);
        m.append(coupler(cr, top - cl, top));
    }
    if (shafts == 2) m.append(dShaft(r, back - rearLen, back));
    return m;
}

// ---------------------------------------------------------------------------
// Wheels and tyres (axis along +Y, standing on z = 0, centre at z = radius;
// positive rotation about +Y rolls toward +X), chains and belts (axis +Z).

// Surface of revolution about the local Y axis of a closed (r, y) profile.
MeshData latheY(std::vector<Vec2> prof, int seg) {
    if (signedArea(prof) < 0.0f) std::reverse(prof.begin(), prof.end());
    Builder b;
    const std::size_t n = prof.size();
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 a = prof[i], c = prof[(i + 1) % n];
        const Vec2 nrm{c.y - a.y, -(c.x - a.x)}; // outward in the (r, y) plane
        for (int k = 0; k < seg; ++k) {
            const float f0 = 2.0f * kPi * static_cast<float>(k) / static_cast<float>(seg);
            const float f1 = 2.0f * kPi * static_cast<float>(k + 1) / static_cast<float>(seg);
            const float fm = (f0 + f1) * 0.5f;
            auto p = [](const Vec2& q, float f) { return Vec3{q.x * std::cos(f), q.y, q.x * std::sin(f)}; };
            b.quad(p(a, f0), p(a, f1), p(c, f1), p(c, f0), {nrm.x * std::cos(fm), nrm.y, nrm.x * std::sin(fm)});
        }
    }
    return b.finish(32.0f);
}

// Matrix taking local x / y / z to the given axes at `origin`.
Mat4 frameOf(const Vec3& origin, const Vec3& ax, const Vec3& ay, const Vec3& az) {
    Mat4 m = Mat4::identity();
    const Vec3 cols[4] = {ax, ay, az, origin};
    for (int c = 0; c < 4; ++c) {
        m.m[c][0] = cols[c].x;
        m.m[c][1] = cols[c].y;
        m.m[c][2] = cols[c].z;
    }
    return m;
}

// Cylinder along +Y centred on y = 0.
MeshData cylY(float r, float y0, float y1, int seg = 48) {
    return latheY({{0.0f, y0}, {r, y0}, {r, y1}, {0.0f, y1}}, seg);
}

// Tyre: rounded carcass plus tread blocks. tread: 0 road, 1 off-road, 2 slick.
MeshData tyre(float dia, float width, float rimDia, int tread) {
    const float R = dia * 0.5f, ri = std::min(rimDia * 0.5f, R * 0.92f), hw = width * 0.5f;
    const float td = tread == 2 ? 0.0f : std::max(R * (tread == 1 ? 0.045f : 0.028f), 0.4f);
    const float R0 = R - td;
    const float wall = R0 - ri, c = std::min(hw * 0.38f, wall * 0.35f);
    std::vector<Vec2> prof{{ri, -hw * 0.82f}, {ri + wall * 0.3f, -hw * 0.98f}, {ri + wall * 0.62f, -hw}};
    for (int s = 0; s <= 6; ++s) { // lower shoulder
        const float t = -kPi * 0.5f + kPi * 0.5f * static_cast<float>(s) / 6.0f;
        prof.push_back({R0 - c + c * std::cos(t), -hw + c + c * std::sin(t)});
    }
    for (int s = 0; s <= 6; ++s) { // upper shoulder
        const float t = kPi * 0.5f * static_cast<float>(s) / 6.0f;
        prof.push_back({R0 - c + c * std::cos(t), hw - c + c * std::sin(t)});
    }
    prof.push_back({ri + wall * 0.62f, hw});
    prof.push_back({ri + wall * 0.3f, hw * 0.98f});
    prof.push_back({ri, hw * 0.82f});
    MeshData m = latheY(prof, 96);
    if (td > 0.0f) {
        const int n = std::clamp(static_cast<int>(2.0f * kPi * R / std::max(width * (tread == 1 ? 0.45f : 0.3f), 1.0f)), 12, 120);
        const float len = 2.0f * kPi * R0 / static_cast<float>(n);
        for (int k = 0; k < n; ++k) {
            const float f = 2.0f * kPi * static_cast<float>(k) / static_cast<float>(n);
            auto block = [&](float y0, float y1, float l) {
                // Flat block: its outer corners (not its middle) touch the tyre radius.
                const float outer = std::sqrt(std::max(R * R - l * l * 0.25f, R0 * R0));
                MeshData bl = boxOf({R0 - td * 0.6f, y0, -l * 0.5f}, {outer, y1, l * 0.5f});
                bl.transform(rotateAxis({0, 1, 0}, f));
                m.append(bl);
            };
            if (tread == 0) { // road: two rows of blocks either side of a centre groove
                block(-hw + c * 0.7f, -hw * 0.08f, len * 0.62f);
                block(hw * 0.08f, hw - c * 0.7f, len * 0.62f);
            } else { // off-road: big lugs, alternating sides
                const float s = k % 2 == 0 ? 1.0f : -1.0f;
                block(std::min(s * hw * 0.95f, -s * hw * 0.15f), std::max(s * hw * 0.95f, -s * hw * 0.15f), len * 0.5f);
            }
        }
    }
    m.transform(translate({0, 0, R}));
    return m;
}

// Alloy rim with spokes, hub and lug nuts (car wheel); y > 0 is the outer face.
MeshData alloyRim(float rimDia, float width, int spokes) {
    const float ri = rimDia * 0.5f, rw = width * 0.82f, h = rw * 0.5f;
    const float lip = std::max(ri * 0.045f, 1.0f), wall = std::max(ri * 0.06f, 1.0f);
    MeshData m = latheY({{ri - wall, -h}, {ri + lip, -h}, {ri + lip, -h + lip * 0.6f}, {ri, -h + lip * 0.8f},
                         {ri, h - lip * 0.8f}, {ri + lip, h - lip * 0.6f}, {ri + lip, h}, {ri - wall, h}},
                        96);
    // Outer face toward -Y, the side the Front view looks at.
    const float hub = ri * 0.22f, face = -(h - lip * 1.6f), depth = std::max(ri * 0.09f, 1.0f);
    m.append(cylY(hub, face - depth * 0.3f, face + depth * 2.0f, 48));        // hub
    m.append(cylY(hub * 0.45f, face - depth * 0.6f, face - depth * 0.3f, 32)); // centre cap
    for (int k = 0; k < std::max(spokes, 2); ++k) {
        const float f = 2.0f * kPi * static_cast<float>(k) / static_cast<float>(std::max(spokes, 2));
        MeshData sp = boxOf({hub * 0.8f, face, -ri * 0.07f}, {ri - wall * 0.5f, face + depth, ri * 0.07f});
        sp.transform(rotateAxis({0, 1, 0}, f));
        m.append(sp);
    }
    for (int k = 0; k < 5; ++k) { // lug nuts
        const float f = 2.0f * kPi * static_cast<float>(k) / 5.0f + 0.3f;
        MeshData nut = cylY(hub * 0.12f, face - depth * 0.7f, face - depth * 0.3f, 6);
        nut.transform(translate({hub * 0.62f * std::cos(f), 0, hub * 0.62f * std::sin(f)}));
        m.append(nut);
    }
    return m;
}

// Robot / caster wheel hub (plastic): rim, recessed web and a bore boss.
MeshData wheelHub(float rimDia, float width, float bore) {
    const float ri = rimDia * 0.5f, hw = width * 0.5f, br = std::clamp(bore * 0.5f, 0.3f, ri * 0.3f);
    return latheY({{br, -hw * 0.9f}, {ri, -hw * 0.9f}, {ri, hw * 0.9f}, {ri * 0.82f, hw * 0.9f}, {ri * 0.78f, hw * 0.3f},
                   {ri * 0.42f, hw * 0.3f}, {ri * 0.36f, hw * 1.15f}, {br, hw * 1.15f}},
                  64);
}

// Swivel caster fork and top plate for a wheel of diameter D and width W.
MeshData casterFork(float D, float W) {
    const float R = D * 0.5f, t = std::max(D * 0.05f, 1.0f), gap = W * 0.5f + t * 0.6f;
    const float top = D + t * 3.0f;
    MeshData m = boxOf({-D * 0.55f, -D * 0.55f, top + t * 2.2f}, {D * 0.55f, D * 0.55f, top + t * 2.9f}); // plate
    m.append(cylinderAt(0, 0, D * 0.22f, top + t, top + t * 2.2f, 32));                                     // swivel
    m.append(boxOf({-D * 0.18f, -gap - t, top}, {D * 0.18f, gap + t, top + t}));                            // crown
    for (float s : {-1.0f, 1.0f}) {
        const float y0 = s > 0 ? gap : -gap - t, y1 = s > 0 ? gap + t : -gap;
        m.append(boxOf({-D * 0.14f, y0, R - t}, {D * 0.14f, y1, top}));                                    // leg
        MeshData axle = cylY(t * 0.9f, y0 - t * 0.4f, y1 + t * 0.4f, 16);                                 // axle nut
        axle.transform(translate({0, 0, R}));
        m.append(axle);
    }
    for (float x : {-1.0f, 1.0f})
        for (float y : {-1.0f, 1.0f}) m.append(cylinderAt(x * D * 0.4f, y * D * 0.4f, t * 0.7f, top + t * 2.9f, top + t * 3.1f, 12));
    return m;
}

// Roller along local Y whose outer surface follows the wheel's rolling circle:
// its axis passes `rc` from the wheel axis, tilted so that axial offset s moves
// s * c along the wheel's circumference (c = 1 tangential, cos 45 for mecanum).
MeshData barrel(float R, float rc, float len, float c) {
    std::vector<Vec2> prof{{0.0f, -len * 0.5f}};
    const float minR = (R - rc) * 0.35f;
    for (int s = 0; s <= 10; ++s) {
        const float y = -len * 0.5f + len * static_cast<float>(s) / 10.0f;
        prof.push_back({std::max(R - std::sqrt(rc * rc + y * y * c * c), minR), y});
    }
    prof.push_back({0.0f, len * 0.5f});
    return latheY(prof, 20);
}

// Omni (style 0, two staggered rows of tangential rollers) or mecanum (style 1,
// rollers at 45 deg) rollers around a wheel of diameter D, width W.
MeshData wheelRollers(float D, float W, int count, int style) {
    const float R = D * 0.5f;
    MeshData m;
    if (style == 0) {
        const int n = std::max(count, 4);
        const float rr = R * 0.17f, rc = R - rr;
        const float len = 2.0f * kPi * rc / static_cast<float>(n) * 0.78f;
        for (int row = 0; row < 2; ++row)
            for (int k = 0; k < n; ++k) {
                const float f = 2.0f * kPi * (static_cast<float>(k) + 0.5f * static_cast<float>(row)) / static_cast<float>(n);
                const Vec3 rad{std::cos(f), 0, std::sin(f)}, tan{-std::sin(f), 0, std::cos(f)};
                MeshData r = barrel(R, rc, len, 1.0f);
                r.transform(frameOf(rad * rc + Vec3{0, (row == 0 ? -1.0f : 1.0f) * W * 0.25f, 0}, rad, tan, cross(rad, tan)));
                m.append(r);
            }
    } else {
        const int n = std::max(count, 5);
        const float rr = R * 0.19f, rc = R - rr;
        for (int k = 0; k < n; ++k) {
            const float f = 2.0f * kPi * static_cast<float>(k) / static_cast<float>(n);
            const Vec3 rad{std::cos(f), 0, std::sin(f)}, tan{-std::sin(f), 0, std::cos(f)};
            const Vec3 axis = normalize(tan + Vec3{0, 1, 0}); // 45 deg to the wheel axis
            MeshData r = barrel(R, rc, W * 1.2f, std::sqrt(0.5f));
            r.transform(frameOf(rad * rc, rad, axis, cross(rad, axis)));
            m.append(r);
        }
    }
    m.transform(translate({0, 0, R}));
    return m;
}

// Hub plates for omni / mecanum wheels.
MeshData rollerHub(float D, float W, int style) {
    const float R = D * 0.5f, t = std::max(D * 0.04f, 0.8f);
    MeshData m;
    const float plate = style == 0 ? R * 0.74f : R * 0.72f;
    for (float s : {-1.0f, 1.0f}) {
        const float y = style == 0 ? s * W * 0.04f : s * (W * 0.5f + t * 0.5f);
        m.append(cylY(plate, y - t * 0.5f, y + t * 0.5f, 48));
    }
    if (style == 0) m.append(cylY(plate * 0.92f, -W * 0.45f, W * 0.45f, 48)); // roller carrier
    m.append(cylY(R * 0.22f, -W * 0.5f - t, W * 0.5f + t, 32));                // hub boss
    m.transform(translate({0, 0, R}));
    return m;
}

// Roller-chain sprocket (axis +Z, base z = 0): seats for rollers of the chain.
std::vector<Vec2> sprocketOutline(float pitch, int teeth) {
    const float half = kPi / static_cast<float>(teeth);
    const float R = pitch / (2.0f * std::sin(half));                    // pitch radius
    const float Ra = pitch * (0.6f + 1.0f / std::tan(half)) * 0.5f;     // tip radius
    const float rs = pitch * 0.3125f * 1.01f;                           // roller seat
    std::vector<Vec2> out;
    const int per = 28;
    for (int i = 0; i < teeth * per; ++i) {
        const float th = 2.0f * kPi * static_cast<float>(i) / static_cast<float>(teeth * per);
        // Nearest gap centre (gaps sit between teeth; tooth 0 straddles +X).
        const float k = std::round((th - half) / (2.0f * half));
        const float gk = half + 2.0f * half * k;
        const Vec2 c = polar(R, gk), d{std::cos(th), std::sin(th)};
        const float b = d.x * c.x + d.y * c.y;
        const float disc = rs * rs - (R * R - b * b);
        float r = Ra;
        // Flanks: from the seat edge, widen outward at 20 deg toward the tip.
        const float off = std::fabs(th - gk) * R;                       // arc distance from the gap centre
        const float flank = R - rs * 0.2f + (off - rs) / std::tan(radians(20.0f));
        r = std::min(r, std::max(flank, R - rs));
        if (disc > 0.0f) r = std::min(r, b - std::sqrt(disc));
        out.push_back(polar(std::max(r, R - rs), th));
    }
    return out;
}

MeshData sprocket(float pitch, int teeth, float width, float bore) {
    const std::vector<Vec2> outline = sprocketOutline(pitch, teeth);
    const float R = pitch / (2.0f * std::sin(kPi / static_cast<float>(teeth)));
    const std::vector<Vec2> inner = bore > 0.0f && bore < R * 0.6f ? circleLike(outline, bore) : std::vector<Vec2>{};
    MeshData m = loft({{0.0f, inner, outline}, {width, inner, outline}});
    if (R > pitch * 2.0f && !inner.empty()) m.append(primitives::tube({0, 0, width}, std::min(bore * 2.2f, R * 0.55f), bore, width * 0.8f, 48));
    return m;
}

// Pulley (axis +Z, base z = 0). type 0 V-groove, 1 timing (GT2), 2 flat (crowned).
MeshData pulley(int type, float dia, float width, float bore) {
    const float R = dia * 0.5f, br = std::clamp(bore * 0.5f, 0.3f, R * 0.5f);
    MeshData m;
    if (type == 1) {
        const int teeth = std::max(8, static_cast<int>(std::round(2.0f * kPi * R / 2.0f)));
        const float R0 = 2.0f * static_cast<float>(teeth) / (2.0f * kPi) - 0.254f;
        std::vector<Vec2> out;
        const int per = 12;
        for (int i = 0; i < teeth * per; ++i) {
            const float th = 2.0f * kPi * static_cast<float>(i) / static_cast<float>(teeth * per);
            const float step = 2.0f * kPi / static_cast<float>(teeth);
            const float g = std::round(th / step) * step;
            const Vec2 c = polar(R0, g), d{std::cos(th), std::sin(th)};
            const float b = d.x * c.x + d.y * c.y, rs = 0.555f;
            const float disc = rs * rs - (R0 * R0 - b * b);
            out.push_back(polar(disc > 0.0f ? std::min(R0, b - std::sqrt(disc)) : R0, th));
        }
        const float fl = std::max(width * 0.08f, 0.8f);
        m = loft({{fl, circleLike(out, br), out}, {width - fl, circleLike(out, br), out}});
        m.append(primitives::tube({0, 0, 0}, R0 + 1.2f, br, fl, 64));            // flanges
        m.append(primitives::tube({0, 0, width - fl}, R0 + 1.2f, br, fl, 64));
        m.append(primitives::tube({0, 0, -width * 0.5f}, std::max(R0 * 0.7f, br + 2.0f), br, width * 0.5f, 48)); // hub
        return m;
    }
    std::vector<Vec2> prof;
    if (type == 0) { // V groove (40 deg) in a disc
        const float g = std::min(width * 0.36f, R * 0.35f);
        prof = {{br, 0}, {R, 0}, {R, width * 0.5f - g}, {R - g * 1.4f, width * 0.5f - g * 0.25f},
                {R - g * 1.4f, width * 0.5f + g * 0.25f}, {R, width * 0.5f + g}, {R, width}, {br, width}};
    } else { // flat, crowned
        prof = {{br, 0}, {R * 0.985f, 0}, {R, width * 0.5f}, {R * 0.985f, width}, {br, width}};
    }
    m = latheY(prof, 72);
    m.transform(rotateAxis({1, 0, 0}, kPi * 0.5f)); // lathe axis Y -> Z
    m.append(primitives::tube({0, 0, -width * 0.4f}, std::max(R * 0.32f, br + 2.0f), br, width * 0.4f, 48)); // hub
    return m;
}

// Closed loop around two circles: centre 0 radius r1, centre (a, 0) radius r2,
// counter-clockwise in XY, with outward normals.
struct LoopPoint {
    Vec2 p, n;
};
std::vector<LoopPoint> beltLoop(float r1, float r2, float a, int perTurn = 96) {
    const float sb = std::clamp((r1 - r2) / a, -0.95f, 0.95f);
    const float beta = std::asin(sb);
    const float up = kPi * 0.5f - beta, lo = -up;
    std::vector<LoopPoint> out;
    auto arc = [&](Vec2 c, float r, float from, float to) {
        const int n = std::max(4, static_cast<int>(static_cast<float>(perTurn) * (to - from) / (2.0f * kPi)));
        for (int i = 0; i <= n; ++i) {
            const float t = from + (to - from) * static_cast<float>(i) / static_cast<float>(n);
            out.push_back({{c.x + r * std::cos(t), c.y + r * std::sin(t)}, {std::cos(t), std::sin(t)}});
        }
    };
    arc({a, 0}, r2, lo, up);
    arc({0, 0}, r1, up, lo + 2.0f * kPi);
    return out;
}

float loopLength(const std::vector<LoopPoint>& l) {
    float s = 0.0f;
    for (std::size_t i = 0; i < l.size(); ++i) {
        const Vec2 a = l[i].p, b = l[(i + 1) % l.size()].p;
        s += std::hypot(b.x - a.x, b.y - a.y);
    }
    return s;
}

// Point at arc length s along the loop (with the direction of travel).
LoopPoint loopAt(const std::vector<LoopPoint>& l, float s, Vec2* dir) {
    for (std::size_t i = 0; i < l.size(); ++i) {
        const Vec2 a = l[i].p, b = l[(i + 1) % l.size()].p;
        const float seg = std::hypot(b.x - a.x, b.y - a.y);
        if (s <= seg || i + 1 == l.size()) {
            const float t = seg > 0.0f ? std::clamp(s / seg, 0.0f, 1.0f) : 0.0f;
            if (dir) *dir = seg > 0.0f ? Vec2{(b.x - a.x) / seg, (b.y - a.y) / seg} : Vec2{1, 0};
            return {{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t}, l[i].n};
        }
        s -= seg;
    }
    return l.front();
}

// Rubber belt swept around the loop; zc = height of the belt centre.
// type 0 flat, 1 V (trapezoid), 2 timing (flat with teeth inside, 2 mm pitch).
MeshData beltAround(const std::vector<LoopPoint>& loop, int type, float width, float thick, float zc, float phase = 0.0f) {
    std::vector<Vec2> sec; // (outward u, height v), counter-clockwise
    if (type == 1) sec = {{-thick * 0.5f, -width * 0.32f}, {thick * 0.5f, -width * 0.5f}, {thick * 0.5f, width * 0.5f}, {-thick * 0.5f, width * 0.32f}};
    else sec = {{-thick * 0.5f, -width * 0.5f}, {thick * 0.5f, -width * 0.5f}, {thick * 0.5f, width * 0.5f}, {-thick * 0.5f, width * 0.5f}};
    // Smooth normals: average the neighbouring segment directions.
    const std::size_t n = loop.size();
    std::vector<Vec2> nrm(n);
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 a = loop[(i + n - 1) % n].p, b = loop[(i + 1) % n].p;
        Vec2 t{b.x - a.x, b.y - a.y};
        const float l = std::max(std::hypot(t.x, t.y), 1e-6f);
        nrm[i] = {t.y / l, -t.x / l};
    }
    Builder b;
    auto P = [&](std::size_t i, const Vec2& s) {
        return Vec3{loop[i].p.x + nrm[i].x * s.x, loop[i].p.y + nrm[i].y * s.x, zc + s.y};
    };
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t j = (i + 1) % n;
        for (std::size_t k = 0; k < sec.size(); ++k) {
            const Vec2 s0 = sec[k], s1 = sec[(k + 1) % sec.size()];
            const Vec2 en{s1.y - s0.y, -(s1.x - s0.x)}; // outward of the section edge
            const Vec2 nm{(nrm[i].x + nrm[j].x) * 0.5f, (nrm[i].y + nrm[j].y) * 0.5f};
            b.quad(P(i, s0), P(j, s0), P(j, s1), P(i, s1), {nm.x * en.x, nm.y * en.x, en.y});
        }
    }
    MeshData m = b.finish(40.0f);
    const float L = loopLength(loop);
    auto along = [&](float s) {
        s = std::fmod(s + phase, L);
        return s < 0.0f ? s + L : s;
    };
    if (type != 2) { // printed marks across the back, so a moving smooth belt shows it
        const int marks = std::max(3, static_cast<int>(L / std::max(width * 8.0f, 20.0f)));
        for (int k = 0; k < marks; ++k) {
            Vec2 d;
            const LoopPoint q = loopAt(loop, along(L * static_cast<float>(k) / static_cast<float>(marks)), &d);
            const Vec2 o{d.y, -d.x};
            const float half = type == 1 ? width * 0.45f : width * 0.42f;
            MeshData mark = boxOf({-std::max(width * 0.08f, 0.6f), thick * 0.5f - 0.05f, -half},
                                  {std::max(width * 0.08f, 0.6f), thick * 0.5f + std::max(thick * 0.08f, 0.15f), half});
            mark.transform(frameOf({q.p.x, q.p.y, zc}, {d.x, d.y, 0}, {o.x, o.y, 0}, {0, 0, 1}));
            m.append(mark);
        }
    }
    if (type == 2) { // teeth on the inside every 2 mm
        const int teeth = std::max(4, static_cast<int>(L / 2.0f));
        for (int k = 0; k < teeth; ++k) {
            Vec2 d;
            const LoopPoint q = loopAt(loop, along(L * static_cast<float>(k) / static_cast<float>(teeth)), &d);
            const Vec2 o{d.y, -d.x}; // outward (right of travel on a counter-clockwise loop)
            MeshData tooth = boxOf({-0.5f, -thick * 0.5f - 0.75f, -width * 0.5f}, {0.5f, -thick * 0.5f + 0.1f, width * 0.5f});
            tooth.transform(frameOf({q.p.x, q.p.y, zc}, {d.x, d.y, 0}, {o.x, o.y, 0}, {0, 0, 1}));
            m.append(tooth);
        }
    }
    return m;
}

// Steel roller chain through `pins` (closed loop or open run): rollers and pins,
// alternating inner and outer figure-eight plates. Built from one template per
// part (transformed copies), fast enough to rebuild every simulation frame.
MeshData chainAlong(const std::vector<Vec2>& pins, bool closed, float pitch, float zc) {
    const float rr = pitch * 0.312f, wi = pitch * 0.625f * 0.5f, tp = pitch * 0.12f, ph = pitch * 0.47f;
    // Templates: roller + pin at the origin; plate pairs from (0, 0) to (pitch, 0).
    MeshData pinT = cylinderAt(0, 0, rr, zc - wi, zc + wi, 20);
    pinT.append(cylinderAt(0, 0, pitch * 0.156f, zc - wi - 2.0f * tp - pitch * 0.05f, zc + wi + 2.0f * tp + pitch * 0.05f, 12));
    std::vector<Vec2> plate;
    {
        const float wh = pitch * 0.34f, cut = 0.45f;
        for (int s = 0; s <= 10; ++s) plate.push_back(polar(ph, kPi * 0.5f + cut + (kPi - 2.0f * cut) * static_cast<float>(s) / 10.0f));
        plate.push_back({pitch * 0.5f, -wh});
        for (int s = 0; s <= 10; ++s) {
            const Vec2 q = polar(ph, -kPi * 0.5f + cut + (kPi - 2.0f * cut) * static_cast<float>(s) / 10.0f);
            plate.push_back({pitch + q.x, q.y});
        }
        plate.push_back({pitch * 0.5f, wh});
    }
    MeshData innerT = prismOf(plate, zc + wi, zc + wi + tp);
    innerT.append(prismOf(plate, zc - wi - tp, zc - wi));
    MeshData outerT = prismOf(plate, zc + wi + tp, zc + wi + 2.0f * tp);
    outerT.append(prismOf(plate, zc - wi - 2.0f * tp, zc - wi - tp));

    MeshData m;
    const std::size_t n = pins.size();
    m.vertices.reserve(n * (pinT.vertices.size() + innerT.vertices.size()));
    m.indices.reserve(n * (pinT.indices.size() + innerT.indices.size()));
    auto place = [&m](const MeshData& t, const Mat4& x) {
        MeshData copy = t;
        copy.transform(x);
        m.append(copy);
    };
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 c = pins[i];
        place(pinT, translate({c.x, c.y, 0}));
        if (!closed && i + 1 == n) break;
        const Vec2 d = pins[(i + 1) % n];
        const float chord = std::hypot(d.x - c.x, d.y - c.y);
        const Mat4 x = translate({c.x, c.y, 0}) * rotateAxis({0, 0, 1}, std::atan2(d.y - c.y, d.x - c.x)) *
                       scale(Vec3{chord / pitch, 1.0f, 1.0f});
        place(i % 2 == 0 ? innerT : outerT, x);
    }
    return m;
}

// Pins around the loop, moved `phase` mm along it. An even number of links keeps
// the inner / outer plates alternating all the way round (the travel wraps cleanly).
std::vector<Vec2> pinsAround(const std::vector<LoopPoint>& loop, float pitch, float phase = 0.0f) {
    const float L = loopLength(loop);
    const int n = std::max(4, 2 * static_cast<int>(std::round(L / pitch * 0.5f)));
    std::vector<Vec2> pins;
    for (int k = 0; k < n; ++k) {
        float s = std::fmod(L * static_cast<float>(k) / static_cast<float>(n) + phase, L);
        if (s < 0.0f) s += L;
        pins.push_back(loopAt(loop, s, nullptr).p);
    }
    return pins;
}

bool validGear(const std::vector<float>& p, std::size_t n) {
    return p.size() >= n && p[0] > 0.0f && p[1] >= 4.0f && p[1] <= 400.0f && p[2] > 0.0f;
}

} // namespace

std::vector<Vec2> gearOutline(float m, int z, float phase, float addendum, float dedendum) {
    const float rp = m * static_cast<float>(z) * 0.5f;
    const float ra = rp + addendum * m;
    const float rf = std::max(rp - dedendum * m, rp * 0.3f);
    const float rb = rp * std::cos(kPressureAngle);
    const float pitchAngle = 2.0f * kPi / static_cast<float>(z);
    // Half-angle of the tooth at radius r, measured from the tooth centre line.
    // Below the base circle the involute does not exist: the flank widens into a
    // small root fillet instead of running radially.
    auto half = [&](float r) {
        const float rr = std::max(r, rb);
        const float a = std::acos(std::clamp(rb / rr, -1.0f, 1.0f));
        float h = std::max(pitchAngle * 0.25f + involute(kPressureAngle) - involute(a), pitchAngle * 0.02f);
        if (r < rb && rb > rf) h += 0.06f * pitchAngle * (rb - r) / (rb - rf);
        return h;
    };
    constexpr int kFlank = 7, kArc = 3;
    std::vector<Vec2> out;
    for (int i = 0; i < z; ++i) {
        const float c = phase + pitchAngle * static_cast<float>(i);
        const float rootStart = c - pitchAngle * 0.5f;
        // Root land before the tooth (from the middle of the gap).
        const float hf = std::min(half(rf), pitchAngle * 0.49f);
        for (int s = 0; s < kArc; ++s)
            out.push_back(polar(rf, rootStart + (c - hf - rootStart) * static_cast<float>(s) / kArc));
        for (int s = 0; s <= kFlank; ++s) { // rising flank
            const float r = rf + (ra - rf) * static_cast<float>(s) / kFlank;
            out.push_back(polar(r, c - std::min(half(r), pitchAngle * 0.49f)));
        }
        const float ht = std::min(half(ra), pitchAngle * 0.49f);
        for (int s = 1; s < kArc; ++s) out.push_back(polar(ra, c - ht + 2.0f * ht * static_cast<float>(s) / kArc));
        for (int s = kFlank; s >= 0; --s) { // falling flank
            const float r = rf + (ra - rf) * static_cast<float>(s) / kFlank;
            out.push_back(polar(r, c + std::min(half(r), pitchAngle * 0.49f)));
        }
        for (int s = 1; s < kArc; ++s)
            out.push_back(polar(rf, c + hf + (pitchAngle * 0.5f - hf) * static_cast<float>(s) / kArc));
    }
    return out;
}

bool isLoopKind(const std::string& k) { return k == "chain_loop" || k == "belt"; }

MeshData loopMeshAt(const Body& b, float travel) {
    if (!isLoopKind(b.kind)) return b.mesh;
    std::vector<float> p = b.params;
    const std::size_t phaseIndex = b.kind == "chain_loop" ? 5 : 7;
    if (p.size() <= phaseIndex) p.resize(phaseIndex + 1, 0.0f);
    p[phaseIndex] += travel / std::max(b.scale, 1e-6f); // travel is in world mm, params unscaled
    MeshData m = buildLibraryPart(b.kind, p);
    if (b.scale != 1.0f) m.transform(scale(Vec3{b.scale, b.scale, b.scale}));
    return m;
}

bool isLibraryKind(const std::string& k) {
    return k == "gear" || k == "ring_gear" || k == "bevel_gear" || k == "rack" || k == "worm" || k == "dc_motor" ||
           k == "stepper_motor" || k == "servo_motor" || k == "ev_motor" || k == "ev_flange_motor" || k == "motor_rotor" ||
           k == "tyre" || k == "alloy_rim" || k == "wheel_hub" || k == "caster_fork" || k == "wheel_rollers" ||
           k == "roller_hub" || k == "sprocket" || k == "pulley" || k == "chain" || k == "chain_loop" || k == "belt" ||
           k == "o_ring";
}

MeshData buildLibraryPart(const std::string& k, const std::vector<float>& p) {
    if (k == "gear" && validGear(p, 3)) return spurLikeGear(p);
    if (k == "ring_gear" && validGear(p, 3)) return ringGear(p);
    if (k == "bevel_gear" && validGear(p, 4)) return bevelGear(p);
    if (k == "rack" && validGear(p, 3)) return rack(p);
    if (k == "worm" && p.size() >= 3 && p[0] > 0.0f && p[1] >= 1.0f && p[2] > 0.0f) return worm(p);
    auto at = [&](std::size_t i) { return i < p.size() ? p[i] : 0.0f; };
    auto ok = [&](std::size_t n) {
        if (p.size() < n) return false;
        for (std::size_t i = 0; i < n; ++i)
            if (!(p[i] > 0.0f) && !(k == "pulley" && i == 0) && !(k == "tyre" && i == 3)) return false;
        return true;
    };
    if (k == "tyre" && ok(3)) return tyre(p[0], p[1], p[2], static_cast<int>(at(3)));
    if (k == "alloy_rim" && ok(4)) {
        MeshData m = alloyRim(p[0], p[1], static_cast<int>(p[2]));
        m.transform(translate({0, 0, p[3] * 0.5f}));
        return m;
    }
    if (k == "wheel_hub" && ok(4)) {
        MeshData m = wheelHub(p[0], p[1], p[2]);
        m.transform(translate({0, 0, p[3] * 0.5f}));
        return m;
    }
    if (k == "caster_fork" && ok(2)) return casterFork(p[0], p[1]);
    if (k == "wheel_rollers" && ok(3)) return wheelRollers(p[0], p[1], static_cast<int>(p[2]), static_cast<int>(at(3)));
    if (k == "roller_hub" && ok(2)) return rollerHub(p[0], p[1], static_cast<int>(at(2)));
    if (k == "sprocket" && ok(3) && p[1] >= 6.0f) return sprocket(p[0], static_cast<int>(p[1]), p[2], at(3));
    if (k == "pulley" && p.size() >= 3 && p[1] > 0.0f && p[2] > 0.0f) return pulley(static_cast<int>(p[0]), p[1], p[2], at(3));
    if (k == "chain" && ok(2)) {
        std::vector<Vec2> pins;
        for (int i = 0; i <= std::max(1, static_cast<int>(p[1])); ++i) pins.push_back({p[0] * static_cast<float>(i), 0.0f});
        return chainAlong(pins, false, p[0], p[0] * 0.62f);
    }
    if (k == "chain_loop" && ok(4)) return chainAlong(pinsAround(beltLoop(p[1], p[2], p[3], 360), p[0], at(5)), true, p[0], at(4));
    if (k == "belt" && p.size() >= 6 && p[1] > 0.0f && p[2] > 0.0f && p[3] > 0.0f && p[4] > 0.0f && p[5] > 0.0f)
        return beltAround(beltLoop(p[1], p[2], p[3]), static_cast<int>(p[0]), p[4], p[5], at(6), at(7));
    if (k == "o_ring" && ok(2)) return primitives::torus({0, 0, p[1] * 0.5f}, p[0] * 0.5f + p[1] * 0.5f, p[1] * 0.5f, 96, 24);
    MeshData m;
    if (k == "dc_motor") m = dcMotor();
    else if (k == "stepper_motor") m = stepperMotor();
    else if (k == "servo_motor") m = servoMotor();
    else if (k == "ev_motor") m = evMotor();
    else if (k == "ev_flange_motor") m = evFlangeMotor();
    else if (k == "motor_rotor") m = motorRotor(p);
    return m;
}

// ---------------------------------------------------------------------------
// Parameters of placed parts

std::vector<LibraryParam> partParams(const std::string& k, const std::vector<float>& p) {
    auto fixed = [](const char* key, float v) { return LibraryParam{key, key, v, v, v, ""}; }; // shown nowhere
    const LibraryParam module{"module", "Module", 2.0f, 0.2f, 20.0f, "mm"};
    const LibraryParam teeth{"teeth", "Teeth", 20.0f, 4.0f, 400.0f, "", true};
    const LibraryParam width{"width", "Face width", 10.0f, 0.5f, 500.0f, "mm"};
    const LibraryParam bore{"bore", "Bore radius", 4.0f, 0.0f, 500.0f, "mm"};
    const LibraryParam phase{"phase", "Tooth phase", 0.0f, -360.0f, 360.0f, "deg"};
    if (k == "motor_rotor") {
        const int style = p.empty() ? 0 : std::clamp(static_cast<int>(p[0]), 0, 4);
        if (style == 4)
            return {fixed("style", 4.0f),
                    {"shaft_length", "Shaft length", 70.0f, 1.0f, 2000.0f, "mm"},
                    {"shafts", "Shafts", 1.0f, 0.0f, 2.0f, "", true, "None|Single|Dual"},
                    {"rear_length", "Rear shaft", 40.0f, 1.0f, 2000.0f, "mm"},
                    {"shaft_dia", "Shaft diameter", 35.0f, 8.0f, 120.0f, "mm"},
                    {"fitting", "Shaft end", 0.0f, 0.0f, 1.0f, "", true, "Splined|Plain"}};
        if (style == 3)
            return {fixed("style", 3.0f),
                    {"shaft_length", "Shaft length", 120.0f, 1.0f, 2000.0f, "mm"},
                    {"shafts", "Shafts", 1.0f, 0.0f, 2.0f, "", true, "None|Single|Dual"},
                    {"rear_length", "Rear shaft", 60.0f, 1.0f, 2000.0f, "mm"},
                    {"shaft_dia", "Shaft diameter", 50.0f, 10.0f, 120.0f, "mm"},
                    {"fitting", "Shaft end", 0.0f, 0.0f, 1.0f, "", true, "Plain|Coupling"}};
        if (style == 2)
            return {fixed("style", 2.0f), {"horn", "Horn", 1.0f, 0.0f, 3.0f, "", true, "Single arm|Double arm|Cross|Disc"}};
        const bool dc = style == 0;
        return {fixed("style", static_cast<float>(style)),
                {"shaft_length", "Shaft length", dc ? 14.0f : 24.0f, 0.5f, 500.0f, "mm"},
                {"shafts", "Shafts", 1.0f, 0.0f, 2.0f, "", true, "None|Single|Dual"},
                {"rear_length", "Rear shaft", dc ? 10.0f : 15.0f, 0.5f, 500.0f, "mm"},
                {"shaft_dia", "Shaft diameter", dc ? 3.0f : 5.0f, 0.4f, 60.0f, "mm"},
                {"fitting", "Shaft end", 1.0f, 0.0f, 1.0f, "", true, "Plain|Coupler"}};
    }
    const char* tread = "Road|Off-road|Slick";
    if (k == "tyre")
        return {{"diameter", "Diameter", 650.0f, 5.0f, 5000.0f, "mm"}, {"width", "Width", 205.0f, 1.0f, 2000.0f, "mm"},
                {"rim_dia", "Rim diameter", 432.0f, 2.0f, 4000.0f, "mm"}, {"tread", "Tread", 0.0f, 0.0f, 2.0f, "", true, tread}};
    if (k == "sprocket")
        return {{"pitch", "Chain pitch", 12.7f, 2.0f, 200.0f, "mm"}, {"teeth", "Teeth", 18.0f, 6.0f, 200.0f, "", true},
                {"width", "Width", 7.0f, 0.5f, 200.0f, "mm"}, {"bore", "Bore radius", 6.0f, 0.0f, 500.0f, "mm"}};
    if (k == "pulley")
        return {{"type", "Type", 0.0f, 0.0f, 2.0f, "", true, "V-groove|Timing (GT2)|Flat"},
                {"diameter", "Diameter", 60.0f, 4.0f, 3000.0f, "mm"}, {"width", "Width", 16.0f, 1.0f, 500.0f, "mm"},
                {"bore", "Bore", 6.0f, 0.0f, 500.0f, "mm"}};
    if (k == "chain") return {{"pitch", "Pitch", 12.7f, 2.0f, 200.0f, "mm"}, {"links", "Links", 20.0f, 1.0f, 2000.0f, "", true}};
    if (k == "o_ring") return {{"inner_dia", "Inner diameter", 40.0f, 1.0f, 5000.0f, "mm"}, {"section", "Section", 4.0f, 0.3f, 200.0f, "mm"}};
    if (k == "gear")
        return {module, teeth, width, bore, {"helix", "Helix angle", 0.0f, -45.0f, 45.0f, "deg"},
                {"style", "Tooth form", 0.0f, 0.0f, 2.0f, "", true, "Spur|Helical|Herringbone"}, phase};
    if (k == "ring_gear") return {module, {"teeth", "Teeth", 48.0f, 12.0f, 400.0f, "", true}, width,
                                  {"rim", "Rim thickness", 6.0f, 0.5f, 200.0f, "mm"}, phase};
    if (k == "bevel_gear") return {module, teeth, {"face", "Face width", 8.0f, 0.5f, 200.0f, "mm"},
                                   {"cone", "Pitch cone", 45.0f, 5.0f, 85.0f, "deg"}, bore, phase};
    if (k == "rack") return {module, {"teeth", "Teeth", 20.0f, 1.0f, 500.0f, "", true}, width,
                             {"height", "Bar height", 8.0f, 0.5f, 500.0f, "mm"}};
    if (k == "worm") return {module, {"starts", "Starts", 1.0f, 1.0f, 8.0f, "", true},
                             {"length", "Length", 40.0f, 2.0f, 2000.0f, "mm"}, phase};
    return {};
}

std::vector<float> withDefaults(const std::string& kind, std::vector<float> params) {
    const std::vector<LibraryParam> schema = partParams(kind, params);
    for (std::size_t i = params.size(); i < schema.size(); ++i) params.push_back(schema[i].value);
    return params;
}

// ---------------------------------------------------------------------------
// Catalogue

const std::vector<LibraryItem>& libraryItems() {
    static const std::vector<LibraryItem> items = [] {
        const LibraryParam size{"size", "Size", 1.0f, 0.1f, 20.0f, "x"};
        const LibraryParam module{"module", "Module", 2.0f, 0.2f, 20.0f, "mm"};
        const LibraryParam width{"width", "Face width", 10.0f, 0.5f, 500.0f, "mm"};
        const LibraryParam bore{"bore", "Bore radius", 4.0f, 0.0f, 500.0f, "mm"};
        auto teeth = [](float v, float lo = 6.0f) { return LibraryParam{"teeth", "Teeth", v, lo, 300.0f, "", true}; };
        auto rpm = [](float v) { return LibraryParam{"rpm", "Speed", v, -6000.0f, 6000.0f, "rpm"}; };
        const LibraryParam shafts{"shafts", "Shafts", 1.0f, 0.0f, 2.0f, "", true, "None|Single|Dual"};
        const LibraryParam fitting{"fitting", "Shaft end", 1.0f, 0.0f, 1.0f, "", true, "Plain|Coupler"};
        return std::vector<LibraryItem>{
            {"dc_motor", "DCMOTOR DC", "DC Motor", "Motors", "28 mm brushed motor; single or dual shaft; spins 360 deg",
             {size, rpm(120.0f), shafts, {"shaft_length", "Shaft length", 14.0f, 0.5f, 500.0f, "mm"},
              {"rear_length", "Rear shaft", 10.0f, 0.5f, 500.0f, "mm"},
              {"shaft_dia", "Shaft diameter", 3.0f, 0.4f, 60.0f, "mm"}, fitting}},
            {"stepper_motor", "STEPPER NEMA17 STEP", "Stepper Motor", "Motors",
             "NEMA 17, 1.8 deg steps; single or dual shaft; spins 360 deg",
             {size, rpm(30.0f), {"step", "Step angle", 1.8f, 0.0f, 90.0f, "deg"}, shafts,
              {"shaft_length", "Shaft length", 24.0f, 0.5f, 500.0f, "mm"},
              {"rear_length", "Rear shaft", 15.0f, 0.5f, 500.0f, "mm"},
              {"shaft_dia", "Shaft diameter", 5.0f, 0.4f, 60.0f, "mm"}, fitting}},
            {"servo_motor", "SERVO", "Servo Motor", "Motors", "Standard servo with a horn; turns 360 deg",
             {size, rpm(60.0f), {"sweep", "Sweep", 360.0f, 1.0f, 360.0f, "deg"},
              {"horn", "Horn", 1.0f, 0.0f, 3.0f, "", true, "Single arm|Double arm|Cross|Disc"}}},
            {"ev_motor", "EVMOTOR EV TRACTION TRACTIONMOTOR ELECTRICCAR CARMOTOR FOOTMOTOR", "Electric Car Motor (Foot)",
             "Motors", "Foot-mounted traction motor, terminal box, keyed shaft; spins 360 deg",
             {size, rpm(120.0f), shafts, {"shaft_length", "Shaft length", 120.0f, 1.0f, 2000.0f, "mm"},
              {"rear_length", "Rear shaft", 60.0f, 1.0f, 2000.0f, "mm"},
              {"shaft_dia", "Shaft diameter", 50.0f, 10.0f, 120.0f, "mm"},
              {"fitting", "Shaft end", 0.0f, 0.0f, 1.0f, "", true, "Plain|Coupling"}}},
            {"ev_flange_motor", "EVFLANGE EVMOTOR2 FLANGEMOTOR SPLINEMOTOR ELECTRICCAR2", "Electric Car Motor (Flange)",
             "Motors", "Ribbed flange-mount EV motor, lifting eyes, splined shaft; spins 360 deg",
             {size, rpm(120.0f), shafts, {"shaft_length", "Shaft length", 70.0f, 1.0f, 2000.0f, "mm"},
              {"rear_length", "Rear shaft", 40.0f, 1.0f, 2000.0f, "mm"},
              {"shaft_dia", "Shaft diameter", 35.0f, 8.0f, 120.0f, "mm"},
              {"fitting", "Shaft end", 0.0f, 0.0f, 1.0f, "", true, "Splined|Plain"}}},
            {"car_wheel", "CARWHEEL WHEEL ALLOYWHEEL", "Car Wheel", "Wheels & tyres",
             "Alloy rim with spokes and lug nuts, treaded tyre; rolls along +X",
             {{"diameter", "Tyre diameter", 650.0f, 100.0f, 3000.0f, "mm"}, {"width", "Tyre width", 205.0f, 30.0f, 1000.0f, "mm"},
              {"rim_dia", "Rim diameter", 432.0f, 50.0f, 2500.0f, "mm"}, {"spokes", "Spokes", 5.0f, 3.0f, 12.0f, "", true},
              {"tread", "Tread", 0.0f, 0.0f, 2.0f, "", true, "Road|Off-road|Slick"}, rpm(30.0f)}},
            {"robot_wheel", "ROBOTWHEEL ROVERWHEEL", "Robot Wheel", "Wheels & tyres", "Plastic hub, rubber tyre; for motor shafts",
             {{"diameter", "Diameter", 65.0f, 10.0f, 1000.0f, "mm"}, {"width", "Width", 26.0f, 3.0f, 500.0f, "mm"},
              {"bore", "Bore", 3.0f, 0.5f, 100.0f, "mm"}, {"tread", "Tread", 1.0f, 0.0f, 2.0f, "", true, "Road|Off-road|Slick"},
              rpm(60.0f)}},
            {"caster_wheel", "CASTER CASTOR SWIVEL", "Caster Wheel", "Wheels & tyres", "Swivel caster: top plate, fork, rubber wheel",
             {{"diameter", "Wheel diameter", 50.0f, 10.0f, 500.0f, "mm"}, {"width", "Wheel width", 20.0f, 3.0f, 200.0f, "mm"},
              rpm(30.0f)}},
            {"omni_wheel", "OMNI OMNIWHEEL", "Omni Wheel", "Wheels & tyres", "Two staggered rows of free rollers: moves sideways too",
             {{"diameter", "Diameter", 60.0f, 20.0f, 1000.0f, "mm"}, {"width", "Width", 26.0f, 5.0f, 300.0f, "mm"},
              {"rollers", "Rollers per row", 8.0f, 4.0f, 24.0f, "", true}, rpm(30.0f)}},
            {"mecanum_wheel", "MECANUM", "Mecanum Wheel", "Wheels & tyres", "Rollers at 45 deg for holonomic drives",
             {{"diameter", "Diameter", 80.0f, 20.0f, 1000.0f, "mm"}, {"width", "Width", 40.0f, 5.0f, 300.0f, "mm"},
              {"rollers", "Rollers", 9.0f, 5.0f, 24.0f, "", true}, rpm(30.0f)}},
            {"tyre", "TIRE TYRE TIER", "Tyre", "Wheels & tyres", "Tyre on its own: road, off-road or slick tread",
             {{"diameter", "Diameter", 650.0f, 5.0f, 5000.0f, "mm"}, {"width", "Width", 205.0f, 1.0f, 2000.0f, "mm"},
              {"rim_dia", "Rim diameter", 432.0f, 2.0f, 4000.0f, "mm"}, {"tread", "Tread", 0.0f, 0.0f, 2.0f, "", true, "Road|Off-road|Slick"}}},
            {"sprocket", "SPROCKET CHAINWHEEL", "Sprocket", "Chains & belts", "Roller-chain sprocket (ANSI 40 / 08B by default)",
             {{"pitch", "Chain pitch", 12.7f, 2.0f, 200.0f, "mm"}, {"teeth", "Teeth", 18.0f, 6.0f, 200.0f, "", true},
              {"width", "Width", 7.0f, 0.5f, 200.0f, "mm"}, {"bore", "Bore radius", 6.0f, 0.0f, 500.0f, "mm"}}},
            {"roller_chain", "CHAIN STEELCHAIN ROLLERCHAIN", "Steel Roller Chain", "Chains & belts",
             "Straight run of steel roller chain: plates, pins, rollers",
             {{"pitch", "Pitch", 12.7f, 2.0f, 200.0f, "mm"}, {"links", "Links", 20.0f, 1.0f, 2000.0f, "", true}}},
            {"chain_drive", "CHAINDRIVE", "Chain Drive", "Chains & belts", "Two sprockets with a steel chain loop; driven follows",
             {{"pitch", "Chain pitch", 12.7f, 2.0f, 200.0f, "mm"}, {"teeth1", "Driver teeth", 12.0f, 6.0f, 200.0f, "", true},
              {"teeth2", "Driven teeth", 24.0f, 6.0f, 200.0f, "", true}, {"center", "Centre distance", 200.0f, 10.0f, 10000.0f, "mm"},
              rpm(30.0f)}},
            {"pulley", "PULLEY SHEAVE", "Pulley", "Chains & belts", "V-groove, timing (GT2) or flat belt pulley",
             {{"type", "Type", 0.0f, 0.0f, 2.0f, "", true, "V-groove|Timing (GT2)|Flat"}, {"diameter", "Diameter", 60.0f, 4.0f, 3000.0f, "mm"},
              {"width", "Width", 16.0f, 1.0f, 500.0f, "mm"}, {"bore", "Bore", 6.0f, 0.0f, 500.0f, "mm"}}},
            {"rubber_belt", "BELT RUBBERBELT VBELT TIMINGBELT", "Rubber Belt", "Chains & belts",
             "Closed rubber belt: flat, V or timing (toothed)",
             {{"type", "Type", 0.0f, 0.0f, 2.0f, "", true, "Flat|V-belt|Timing"}, {"dia1", "Pulley 1 diameter", 60.0f, 4.0f, 3000.0f, "mm"},
              {"dia2", "Pulley 2 diameter", 60.0f, 4.0f, 3000.0f, "mm"}, {"center", "Centre distance", 150.0f, 5.0f, 10000.0f, "mm"},
              {"width", "Width", 12.0f, 1.0f, 500.0f, "mm"}}},
            {"belt_drive", "BELTDRIVE", "Belt Drive", "Chains & belts", "Two pulleys with a rubber belt; driven follows",
             {{"type", "Type", 0.0f, 0.0f, 2.0f, "", true, "V-belt|Timing|Flat"}, {"dia1", "Driver diameter", 50.0f, 6.0f, 3000.0f, "mm"},
              {"dia2", "Driven diameter", 100.0f, 6.0f, 3000.0f, "mm"}, {"center", "Centre distance", 180.0f, 10.0f, 10000.0f, "mm"},
              {"width", "Width", 16.0f, 2.0f, 500.0f, "mm"}, rpm(60.0f)}},
            {"o_ring", "ORING RUBBERBAND RUBBERRING SEAL", "O-Ring / Rubber Band", "Chains & belts", "Rubber ring of round section",
             {{"inner_dia", "Inner diameter", 40.0f, 1.0f, 5000.0f, "mm"}, {"section", "Section", 4.0f, 0.3f, 200.0f, "mm"}}},
            {"spur_gear", "SPUR GEAR", "Spur Gear", "Gears", "Straight involute teeth, 20 deg pressure angle",
             {module, teeth(20.0f), width, bore}},
            {"helical_gear", "HELICAL", "Helical Gear", "Gears", "Teeth on a helix: smoother, quieter mesh",
             {module, teeth(20.0f), width, bore, {"helix", "Helix angle", 20.0f, -45.0f, 45.0f, "deg"}}},
            {"herringbone_gear", "HERRINGBONE DOUBLEHELICAL", "Herringbone Gear", "Gears",
             "Double helical: no axial thrust",
             {module, teeth(20.0f), {"width", "Face width", 14.0f, 1.0f, 500.0f, "mm"}, bore,
              {"helix", "Helix angle", 25.0f, 5.0f, 45.0f, "deg"}}},
            {"bevel_gear", "BEVEL", "Bevel Gear", "Gears", "Conical teeth for shafts at an angle",
             {module, teeth(20.0f), {"face", "Face width", 8.0f, 0.5f, 200.0f, "mm"},
              {"cone", "Pitch cone", 45.0f, 5.0f, 85.0f, "deg"}, bore}},
            {"ring_gear", "INTERNAL RING ANNULUS", "Internal Ring Gear", "Gears",
             "Teeth on the inside (planetary ring)",
             {module, teeth(48.0f, 12.0f), width, {"rim", "Rim thickness", 6.0f, 0.5f, 200.0f, "mm"}}},
            {"worm", "WORMSCREW", "Worm", "Gears", "Threaded screw that drives a worm wheel",
             {module, {"starts", "Starts", 1.0f, 1.0f, 8.0f, "", true}, {"length", "Length", 40.0f, 2.0f, 2000.0f, "mm"}}},
            {"worm_wheel", "WORMWHEEL", "Worm Wheel", "Gears", "Helical wheel that meshes with a worm",
             {module, teeth(30.0f, 10.0f), width, bore}},
            {"rack", "RACK LINEARGEAR", "Gear Rack", "Linear gears", "Straight toothed bar: rotation to linear motion",
             {module, {"teeth", "Teeth", 20.0f, 1.0f, 500.0f, "", true}, width,
              {"height", "Bar height", 8.0f, 0.5f, 500.0f, "mm"}}},
            {"gear_pair", "GEARPAIR PAIR", "Gear Pair", "Gear trains", "Two meshing spur gears, driven gear follows",
             {module, {"teeth1", "Driver teeth", 16.0f, 6.0f, 300.0f, "", true},
              {"teeth2", "Driven teeth", 32.0f, 6.0f, 300.0f, "", true}, width, rpm(30.0f)}},
            {"rack_pinion", "RACKANDPINION RACKPINION", "Rack and Pinion", "Gear trains",
             "Pinion turns, rack slides back and forth",
             {module, {"teeth", "Pinion teeth", 16.0f, 6.0f, 300.0f, "", true},
              {"rack_teeth", "Rack teeth", 30.0f, 4.0f, 500.0f, "", true}, width, rpm(20.0f)}},
            {"planetary", "PLANETARY EPICYCLIC", "Planetary Gear Set", "Gear trains",
             "Sun, three planets on a carrier, fixed ring",
             {{"module", "Module", 1.5f, 0.2f, 20.0f, "mm"}, {"sun_teeth", "Sun teeth", 18.0f, 6.0f, 300.0f, "", true},
              {"planet_teeth", "Planet teeth", 18.0f, 6.0f, 300.0f, "", true}, width, rpm(20.0f)}},
            {"worm_drive", "WORMDRIVE WORMGEAR", "Worm Drive", "Gear trains", "Worm turning a worm wheel at 90 deg",
             {module, teeth(30.0f, 10.0f), {"starts", "Starts", 1.0f, 1.0f, 8.0f, "", true}, width, rpm(60.0f)}},
        };
    }();
    return items;
}

const LibraryItem* findLibraryItem(std::string_view key) {
    std::string k(key);
    for (char& c : k) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    std::string compact;
    for (char c : k)
        if (c != '_' && c != '-' && c != ' ') compact += c;
    for (const LibraryItem& it : libraryItems()) {
        std::string ik(it.key);
        for (char& c : ik) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        std::string ikc;
        for (char c : ik)
            if (c != '_') ikc += c;
        if (k == ik || compact == ikc) return &it;
        std::string aliases = std::string(" ") + it.aliases + " ";
        if (aliases.find(" " + compact + " ") != std::string::npos) return &it;
    }
    return nullptr;
}

// ---------------------------------------------------------------------------
// Components

namespace {

const Vec3 kSteel{0.78f, 0.80f, 0.84f};
const Vec3 kDarkSteel{0.36f, 0.38f, 0.42f};
const Vec3 kBlack{0.17f, 0.18f, 0.20f};
const Vec3 kBrass{0.80f, 0.64f, 0.30f};
const Vec3 kGearBlue{0.42f, 0.58f, 0.80f};
const Vec3 kServoBlue{0.20f, 0.36f, 0.72f};
const Vec3 kCastAluminium{0.80f, 0.81f, 0.83f};
const Vec3 kRubber{0.13f, 0.13f, 0.14f};
const Vec3 kAlloy{0.74f, 0.76f, 0.79f};
const Vec3 kYellowPlastic{0.96f, 0.76f, 0.18f};

Body part(const std::string& name, const std::string& kind, std::vector<float> params, const Vec3& at, Vec3 color,
          const std::string& component, float scale = 1.0f) {
    Body b = makeBody(name, kind, std::move(params), at, color);
    b.component = component;
    if (scale != 1.0f) b.setScale(scale);
    return b;
}

Motion spin(const std::string& name, std::vector<std::string> bodies, const Vec3& origin, float rpm, Vec3 axis = {0, 0, 1}) {
    Motion m;
    m.name = name;
    m.kind = MotionKind::Circular;
    m.bodies = std::move(bodies);
    m.axis = axis;
    m.origin = origin;
    m.speed = rpm * 6.0f; // rev/min -> deg/s
    m.range = 360.0f;     // full turns, angle shown 0..360
    m.mode = MotionMode::Loop;
    return m;
}

Motion follower(const std::string& name, std::vector<std::string> bodies, const Vec3& origin, const std::string& leader,
                float ratio, Vec3 axis = {0, 0, 1}) {
    Motion m;
    m.name = name;
    m.kind = MotionKind::Circular;
    m.bodies = std::move(bodies);
    m.axis = axis;
    m.origin = origin;
    m.follow = leader;
    m.ratio = ratio;
    m.speed = 0.0f;
    return m;
}

// A chain / belt travelling along its loop with the driver, at its pitch radius.
Motion loopTravelMotion(const std::string& name, const std::string& body, const std::string& driver, float radius) {
    Motion m;
    m.name = name;
    m.kind = MotionKind::Linear;
    m.bodies = {body};
    m.axis = {1, 0, 0};
    m.follow = driver;
    m.radius = radius;
    m.speed = 0.0f;
    m.alongLoop = true;
    return m;
}

} // namespace

LibraryComponent makeLibraryComponent(const LibraryItem& item, const std::map<std::string, float>& values,
                                      const std::string& name, const Vec3& o) {
    auto v = [&](const char* key) {
        for (const LibraryParam& p : item.params)
            if (std::string(p.key) == key) {
                const auto it = values.find(key);
                float x = it != values.end() ? it->second : p.value;
                x = std::clamp(x, p.min, p.max);
                return p.integer ? std::round(x) : x;
            }
        const auto it = values.find(key);
        return it != values.end() ? it->second : 0.0f;
    };
    LibraryComponent c;
    c.name = name;
    const std::string key = item.key;
    const std::string rotor = name + " Rotor";

    // Motors: housing + rotor, 360 deg circular motion on the rotor.
    if (key == "dc_motor" || key == "stepper_motor" || key == "servo_motor" || key == "ev_motor" || key == "ev_flange_motor") {
        const float s = v("size");
        const int style = key == "dc_motor" ? 0 : key == "stepper_motor" ? 1 : key == "servo_motor" ? 2 : key == "ev_motor" ? 3 : 4;
        const Vec3 housingColor = style == 0 ? kSteel : style == 1 ? kBlack : style == 2 ? kServoBlue : kCastAluminium;
        const float axisHeight = style == 3 ? kEvAxis : kEv2Axis; // traction motors lie along +X
        c.bodies.push_back(part(name, key, {}, o, housingColor, name, s));
        std::vector<float> rotorParams{static_cast<float>(style)};
        if (style == 2) rotorParams.push_back(v("horn"));
        else rotorParams.insert(rotorParams.end(), {v("shaft_length"), v("shafts"), v("rear_length"), v("shaft_dia"), v("fitting")});
        const Vec3 rotorColor = style == 2 ? Vec3{0.92f, 0.92f, 0.90f} : style == 3 ? kBlack : kSteel;
        c.bodies.push_back(part(rotor, "motor_rotor", rotorParams, o, rotorColor, name, s));
        // The traction motor turns about its horizontal axis (+X) at the shaft height.
        Motion m = style >= 3 ? spin(name + " Drive", {rotor}, o + Vec3{0, 0, axisHeight * s}, v("rpm"), {1, 0, 0})
                              : spin(name + " Drive", {rotor}, o, v("rpm"));
        if (style == 1) m.step = v("step");
        if (style == 2 && v("sweep") < 359.5f) { // positional servo: back and forth over the sweep
            m.range = v("sweep");
            m.mode = MotionMode::Oscillate;
        }
        c.motions.push_back(m);
        return c;
    }

    // Wheels: spin about +Y through the wheel centre (positive rolls toward +X).
    auto rolling = [&](std::vector<std::string> bodies, float dia) {
        if (std::fabs(v("rpm")) > 1e-6f)
            c.motions.push_back(spin(name + " Spin", std::move(bodies), o + Vec3{0, 0, dia * 0.5f}, v("rpm"), {0, 1, 0}));
    };
    if (key == "car_wheel") {
        const float d = v("diameter"), w = v("width"), rim = std::min(v("rim_dia"), d * 0.85f);
        c.bodies.push_back(part(name + " Rim", "alloy_rim", {rim, w, v("spokes"), d}, o, kAlloy, name));
        c.bodies.push_back(part(name + " Tyre", "tyre", {d, w, rim, v("tread")}, o, kRubber, name));
        rolling({name + " Rim", name + " Tyre"}, d);
        return c;
    }
    if (key == "robot_wheel" || key == "caster_wheel") {
        const float d = v("diameter"), w = v("width"), rim = d * 0.7f;
        if (key == "caster_wheel") c.bodies.push_back(part(name + " Fork", "caster_fork", {d, w}, o, kSteel, name));
        c.bodies.push_back(part(name + " Hub", "wheel_hub", {rim, w, key == "robot_wheel" ? v("bore") : d * 0.12f, d}, o,
                                key == "robot_wheel" ? kYellowPlastic : kAlloy, name));
        c.bodies.push_back(part(name + " Tyre", "tyre", {d, w, rim, key == "robot_wheel" ? v("tread") : 2.0f}, o, kRubber, name));
        rolling({name + " Hub", name + " Tyre"}, d);
        return c;
    }
    if (key == "omni_wheel" || key == "mecanum_wheel") {
        const float d = v("diameter"), w = v("width"), style = key == "omni_wheel" ? 0.0f : 1.0f;
        c.bodies.push_back(part(name + " Hub", "roller_hub", {d, w, style}, o, kAlloy, name));
        c.bodies.push_back(part(name + " Rollers", "wheel_rollers", {d, w, v("rollers"), style}, o, kRubber, name));
        rolling({name + " Hub", name + " Rollers"}, d);
        return c;
    }
    if (key == "tyre") {
        c.bodies.push_back(part(name, "tyre", {v("diameter"), v("width"), std::min(v("rim_dia"), v("diameter") * 0.95f), v("tread")}, o,
                                kRubber, name));
        return c;
    }
    if (key == "sprocket") {
        c.bodies.push_back(part(name, "sprocket", {v("pitch"), v("teeth"), v("width"), v("bore")}, o, kSteel, name));
        return c;
    }
    if (key == "roller_chain") {
        c.bodies.push_back(part(name, "chain", {v("pitch"), v("links")}, o, kDarkSteel, name));
        return c;
    }
    if (key == "chain_drive") {
        const float p = v("pitch"), z1 = v("teeth1"), z2 = v("teeth2");
        const float r1 = p / (2.0f * std::sin(kPi / z1)), r2 = p / (2.0f * std::sin(kPi / z2));
        const float a = std::max(v("center"), (r1 + r2) * 1.15f);
        const float w = p * 0.625f * 0.9f;
        const std::string s1 = name + " Driver", s2 = name + " Driven";
        c.bodies.push_back(part(s1, "sprocket", {p, z1, w, std::min(p * 0.5f, r1 * 0.4f)}, o, kSteel, name));
        c.bodies.push_back(part(s2, "sprocket", {p, z2, w, std::min(p * 0.5f, r2 * 0.4f)}, o + Vec3{a, 0, 0}, kSteel, name));
        c.bodies.push_back(part(name + " Chain", "chain_loop", {p, r1, r2, a, w * 0.5f}, o, kDarkSteel, name));
        c.motions.push_back(spin(s1 + " Drive", {s1}, o, v("rpm")));
        c.motions.push_back(follower(s2 + " Chain", {s2}, o + Vec3{a, 0, 0}, s1 + " Drive", z1 / z2));
        c.motions.push_back(loopTravelMotion(name + " Chain Travel", name + " Chain", s1 + " Drive", r1));
        return c;
    }
    if (key == "pulley") {
        c.bodies.push_back(part(name, "pulley", {v("type"), v("diameter"), v("width"), v("bore")}, o + Vec3{0, 0, v("width") * 0.4f},
                                kAlloy, name));
        return c;
    }
    if (key == "rubber_belt") {
        const int type = static_cast<int>(v("type"));
        const float w = v("width"), t = type == 1 ? w * 0.6f : type == 2 ? 1.4f : std::max(w * 0.18f, 1.0f);
        c.bodies.push_back(part(name, "belt", {v("type"), v("dia1") * 0.5f, v("dia2") * 0.5f, std::max(v("center"), (v("dia1") + v("dia2")) * 0.5f + t),
                                               w, t, w * 0.5f}, o, kRubber, name));
        return c;
    }
    if (key == "belt_drive") {
        // Pulley types follow the belt: V-belt -> V-groove, timing -> GT2, flat -> crowned.
        const int belt = static_cast<int>(v("type"));
        const float ptype = belt == 0 ? 0.0f : belt == 1 ? 1.0f : 2.0f;
        const float w = v("width"), d1 = v("dia1"), d2 = v("dia2");
        const float a = std::max(v("center"), (d1 + d2) * 0.55f);
        const std::string p1 = name + " Driver", p2 = name + " Driven";
        const float lift = w * 0.4f; // pulley hubs stand below the pulley
        c.bodies.push_back(part(p1, "pulley", {ptype, d1, w, std::min(6.0f, d1 * 0.2f)}, o + Vec3{0, 0, lift}, kAlloy, name));
        c.bodies.push_back(part(p2, "pulley", {ptype, d2, w, std::min(8.0f, d2 * 0.2f)}, o + Vec3{a, 0, lift}, kAlloy, name));
        float r1, r2, bw, bt;
        if (belt == 0) { // V-belt sits in the groove
            const float g1 = std::min(w * 0.36f, d1 * 0.175f), g2 = std::min(w * 0.36f, d2 * 0.175f);
            r1 = d1 * 0.5f - g1 * 0.7f;
            r2 = d2 * 0.5f - g2 * 0.7f;
            bw = std::min(g1, g2) * 1.8f;
            bt = std::min(g1, g2) * 1.1f;
        } else if (belt == 1) { // timing belt wraps the pitch line
            auto pitchR = [](float d) {
                const float teeth = std::max(8.0f, std::round(kPi * d / 2.0f));
                return 2.0f * teeth / (2.0f * kPi) - 0.254f;
            };
            bt = 1.4f;
            r1 = pitchR(d1) + bt * 0.5f;
            r2 = pitchR(d2) + bt * 0.5f;
            bw = w - 2.0f * std::max(w * 0.08f, 0.8f) - 0.4f;
        } else {
            bt = std::max(w * 0.12f, 1.0f);
            r1 = d1 * 0.5f + bt * 0.5f;
            r2 = d2 * 0.5f + bt * 0.5f;
            bw = w * 0.85f;
        }
        c.bodies.push_back(part(name + " Belt", "belt", {static_cast<float>(belt == 0 ? 1 : belt == 1 ? 2 : 0), r1, r2, a, bw, bt, lift + w * 0.5f},
                                o, kRubber, name));
        c.motions.push_back(spin(p1 + " Drive", {p1}, o, v("rpm")));
        c.motions.push_back(follower(p2 + " Belt", {p2}, o + Vec3{a, 0, 0}, p1 + " Drive", r1 / r2));
        c.motions.push_back(loopTravelMotion(name + " Belt Travel", name + " Belt", p1 + " Drive", r1));
        return c;
    }
    if (key == "o_ring") {
        c.bodies.push_back(part(name, "o_ring", {v("inner_dia"), v("section")}, o, kRubber, name));
        return c;
    }

    const float m = v("module");
    if (key == "spur_gear" || key == "helical_gear" || key == "herringbone_gear" || key == "worm_wheel") {
        const float helix = key == "spur_gear" ? 0.0f : key == "worm_wheel" ? 5.7f : v("helix");
        const float style = key == "spur_gear" ? 0.0f : key == "herringbone_gear" ? 2.0f : 1.0f;
        c.bodies.push_back(part(name, "gear", {m, v("teeth"), v("width"), v("bore"), helix, style, 0.0f}, o,
                                key == "worm_wheel" ? kBrass : kGearBlue, name));
        return c;
    }
    if (key == "bevel_gear") {
        c.bodies.push_back(part(name, "bevel_gear", {m, v("teeth"), v("face"), v("cone"), v("bore"), 0.0f}, o, kBrass, name));
        return c;
    }
    if (key == "ring_gear") {
        c.bodies.push_back(part(name, "ring_gear", {m, v("teeth"), v("width"), v("rim"), 0.0f}, o, kDarkSteel, name));
        return c;
    }
    if (key == "worm") {
        const float rp = 5.0f * m;
        c.bodies.push_back(part(name, "worm", {m, v("starts"), v("length"), 0.0f}, o + Vec3{0, 0, rp + m}, kSteel, name));
        return c;
    }
    if (key == "rack") {
        const float h = v("height") + 1.25f * m;
        c.bodies.push_back(part(name, "rack", {m, v("teeth"), v("width"), v("height")}, o + Vec3{0, h, 0}, kDarkSteel, name));
        return c;
    }

    // Gear trains.
    const float w = v("width");
    if (key == "gear_pair") {
        const float z1 = v("teeth1"), z2 = v("teeth2");
        const float a = m * (z1 + z2) * 0.5f; // centre distance
        const std::string g1 = name + " Driver", g2 = name + " Driven";
        c.bodies.push_back(part(g1, "gear", {m, z1, w, std::min(4.0f, m * z1 * 0.25f), 0, 0, 0}, o, kGearBlue, name));
        c.bodies.push_back(part(g2, "gear", {m, z2, w, std::min(6.0f, m * z2 * 0.25f), 0, 0, 180.0f + 180.0f / z2},
                                o + Vec3{a, 0, 0}, kBrass, name));
        c.motions.push_back(spin(g1 + " Drive", {g1}, o, v("rpm")));
        c.motions.push_back(follower(g2 + " Mesh", {g2}, o + Vec3{a, 0, 0}, g1 + " Drive", -z1 / z2));
        return c;
    }
    if (key == "rack_pinion") {
        const float z = v("teeth"), n = v("rack_teeth");
        const float rp = m * z * 0.5f;
        const std::string pin = name + " Pinion", bar = name + " Rack";
        c.bodies.push_back(part(pin, "gear", {m, z, w, std::min(4.0f, rp * 0.5f), 0, 0, -90.0f}, o, kGearBlue, name));
        // A tooth gap must sit under the pinion (x = 0): odd tooth counts shift half a pitch.
        const float shift = static_cast<int>(n) % 2 == 1 ? kPi * m * 0.5f : 0.0f;
        c.bodies.push_back(part(bar, "rack", {m, n, w, 2.0f * m}, o + Vec3{shift, -rp, 0}, kDarkSteel, name));
        // The pinion swings so the rack stays in mesh (40 % of the rack each way).
        Motion drive = spin(pin + " Drive", {pin}, o, v("rpm"));
        drive.range = degrees(0.4f * kPi * m * n / rp);
        drive.mode = MotionMode::Oscillate;
        c.motions.push_back(drive);
        Motion slide;
        slide.name = bar + " Slide";
        slide.kind = MotionKind::Linear;
        slide.bodies = {bar};
        slide.axis = {1, 0, 0};
        slide.follow = drive.name;
        slide.radius = rp;
        slide.speed = 0.0f;
        c.motions.push_back(slide);
        return c;
    }
    if (key == "planetary") {
        const float zs = v("sun_teeth"), zp = v("planet_teeth"), zr = zs + 2.0f * zp;
        const float a = m * (zs + zp) * 0.5f;
        const std::string sun = name + " Sun", ring = name + " Ring";
        c.bodies.push_back(part(sun, "gear", {m, zs, w, std::min(3.0f, m * zs * 0.2f), 0, 0, 0}, o, kBrass, name));
        std::vector<std::string> planets;
        for (int k = 0; k < 3; ++k) {
            const float phi = 120.0f * static_cast<float>(k);
            const std::string pl = name + " Planet " + std::to_string(k + 1);
            planets.push_back(pl);
            c.bodies.push_back(part(pl, "gear", {m, zp, w, std::min(2.0f, m * zp * 0.15f), 0, 0, phi + 180.0f + 180.0f / zp},
                                    o + Vec3{a * std::cos(radians(phi)), a * std::sin(radians(phi)), 0}, kGearBlue, name));
        }
        c.bodies.push_back(part(ring, "ring_gear", {m, zr, w, 3.0f * m, 0.0f}, o, kDarkSteel, name));
        const float carrier = zs / (zs + zr); // carrier turns per sun turn (ring fixed)
        c.motions.push_back(spin(sun + " Drive", {sun}, o, v("rpm")));
        c.motions.push_back(follower(name + " Carrier", planets, o, sun + " Drive", carrier));
        Motion own = follower(name + " Planet Spin", planets, o, sun + " Drive", -(1.0f - carrier) * zs / zp);
        own.eachCentre = true; // each planet about its own axle, carried by the carrier
        c.motions.push_back(own);
        return c;
    }
    if (key == "worm_drive") {
        const float z = v("teeth"), starts = v("starts");
        const float rw = 5.0f * m, rg = m * z * 0.5f;
        const std::string wheel = name + " Wheel", wormName = name + " Worm";
        c.bodies.push_back(part(wheel, "gear", {m, z, w, std::min(5.0f, rg * 0.3f), 5.7f, 1.0f, 0.0f}, o, kBrass, name));
        const Vec3 wormAt = o + Vec3{0, -(rw + rg), w * 0.5f};
        c.bodies.push_back(part(wormName, "worm", {m, starts, std::max(w * 3.0f, 6.0f * m * starts), 0.0f}, wormAt, kSteel, name));
        c.motions.push_back(spin(wormName + " Drive", {wormName}, wormAt, v("rpm"), {1, 0, 0}));
        c.motions.push_back(follower(wheel + " Mesh", {wheel}, o, wormName + " Drive", starts / z));
        return c;
    }
    return c;
}

} // namespace sicdm

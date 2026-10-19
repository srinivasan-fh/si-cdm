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
    const int style = std::clamp(static_cast<int>(p[0]), 0, 2);
    if (style == 2) return servoHorn(static_cast<int>(p[1]));
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

bool isLibraryKind(const std::string& k) {
    return k == "gear" || k == "ring_gear" || k == "bevel_gear" || k == "rack" || k == "worm" || k == "dc_motor" ||
           k == "stepper_motor" || k == "servo_motor" || k == "motor_rotor";
}

MeshData buildLibraryPart(const std::string& k, const std::vector<float>& p) {
    if (k == "gear" && validGear(p, 3)) return spurLikeGear(p);
    if (k == "ring_gear" && validGear(p, 3)) return ringGear(p);
    if (k == "bevel_gear" && validGear(p, 4)) return bevelGear(p);
    if (k == "rack" && validGear(p, 3)) return rack(p);
    if (k == "worm" && p.size() >= 3 && p[0] > 0.0f && p[1] >= 1.0f && p[2] > 0.0f) return worm(p);
    MeshData m;
    if (k == "dc_motor") m = dcMotor();
    else if (k == "stepper_motor") m = stepperMotor();
    else if (k == "servo_motor") m = servoMotor();
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
        const int style = p.empty() ? 0 : std::clamp(static_cast<int>(p[0]), 0, 2);
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
    if (key == "dc_motor" || key == "stepper_motor" || key == "servo_motor") {
        const float s = v("size");
        const int style = key == "dc_motor" ? 0 : key == "stepper_motor" ? 1 : 2;
        const Vec3 housingColor = style == 0 ? kSteel : style == 1 ? kBlack : kServoBlue;
        c.bodies.push_back(part(name, key, {}, o, housingColor, name, s));
        std::vector<float> rotorParams{static_cast<float>(style)};
        if (style == 2) rotorParams.push_back(v("horn"));
        else rotorParams.insert(rotorParams.end(), {v("shaft_length"), v("shafts"), v("rear_length"), v("shaft_dia"), v("fitting")});
        c.bodies.push_back(part(rotor, "motor_rotor", rotorParams, o, style == 2 ? Vec3{0.92f, 0.92f, 0.90f} : kSteel, name, s));
        Motion m = spin(name + " Drive", {rotor}, o, v("rpm"));
        if (style == 1) m.step = v("step");
        if (style == 2 && v("sweep") < 359.5f) { // positional servo: back and forth over the sweep
            m.range = v("sweep");
            m.mode = MotionMode::Oscillate;
        }
        c.motions.push_back(m);
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

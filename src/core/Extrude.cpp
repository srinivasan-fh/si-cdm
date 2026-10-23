#include "core/Extrude.hpp"
#include "core/SketchEdit.hpp"

#include <algorithm>
#include <cmath>

namespace sicdm {

namespace {

constexpr int kCircleSegments = 96;
constexpr float kFeatureAngleDeg = 20.0f; // vertical edges where the outline turns more than this

float cross2(Vec2 o, Vec2 a, Vec2 b) { return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x); }

bool samePoint(Vec2 a, Vec2 b, float eps) { return std::fabs(a.x - b.x) <= eps && std::fabs(a.y - b.y) <= eps; }

// Drops repeated and collinear points; returns the cleaned polygon.
std::vector<Vec2> cleanPolygon(std::vector<Vec2> p) {
    if (p.size() >= 2 && samePoint(p.front(), p.back(), 1e-6f)) p.pop_back();
    std::vector<Vec2> out;
    for (const Vec2& q : p)
        if (out.empty() || !samePoint(out.back(), q, 1e-6f)) out.push_back(q);
    bool changed = true;
    while (changed && out.size() >= 3) {
        changed = false;
        for (std::size_t i = 0; i < out.size() && out.size() >= 3; ++i) {
            const Vec2 a = out[(i + out.size() - 1) % out.size()], b = out[i], c = out[(i + 1) % out.size()];
            const float len = std::hypot(c.x - a.x, c.y - a.y);
            if (std::fabs(cross2(a, b, c)) <= 1e-6f * std::max(len, 1.0f)) {
                out.erase(out.begin() + static_cast<std::ptrdiff_t>(i));
                changed = true;
                break;
            }
        }
    }
    return out;
}

bool segmentsIntersect(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    const float d1 = cross2(c, d, a), d2 = cross2(c, d, b), d3 = cross2(a, b, c), d4 = cross2(a, b, d);
    if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) return true;
    auto onSeg = [](Vec2 p, Vec2 q, Vec2 r) { // r on segment pq, given collinear
        return std::min(p.x, q.x) - 1e-6f <= r.x && r.x <= std::max(p.x, q.x) + 1e-6f &&
               std::min(p.y, q.y) - 1e-6f <= r.y && r.y <= std::max(p.y, q.y) + 1e-6f;
    };
    if (std::fabs(d1) < 1e-9f && onSeg(c, d, a)) return true;
    if (std::fabs(d2) < 1e-9f && onSeg(c, d, b)) return true;
    if (std::fabs(d3) < 1e-9f && onSeg(a, b, c)) return true;
    if (std::fabs(d4) < 1e-9f && onSeg(a, b, d)) return true;
    return false;
}

bool pointInTriangle(Vec2 p, Vec2 a, Vec2 b, Vec2 c) {
    const float d1 = cross2(a, b, p), d2 = cross2(b, c, p), d3 = cross2(c, a, p);
    const bool neg = d1 < 0 || d2 < 0 || d3 < 0, pos = d1 > 0 || d2 > 0 || d3 > 0;
    return !(neg && pos);
}

const char* kindName(CurveKind k) { return curveKindName(k); }

} // namespace

float signedArea(const std::vector<Vec2>& p) {
    double a = 0.0;
    for (std::size_t i = 0; i < p.size(); ++i) {
        const Vec2 q = p[i], r = p[(i + 1) % p.size()];
        a += static_cast<double>(q.x) * r.y - static_cast<double>(r.x) * q.y;
    }
    return static_cast<float>(a * 0.5);
}

bool isSimplePolygon(const std::vector<Vec2>& p) {
    const std::size_t n = p.size();
    if (n < 3) return false;
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = i + 1; j < n; ++j) {
            if (j == i + 1 || (i == 0 && j == n - 1)) continue; // neighbours share a vertex
            if (segmentsIntersect(p[i], p[(i + 1) % n], p[j], p[(j + 1) % n])) return false;
        }
    return true;
}

std::vector<std::uint32_t> triangulatePolygon(const std::vector<Vec2>& p) {
    std::vector<std::uint32_t> tris;
    const std::size_t n = p.size();
    if (n < 3) return tris;
    std::vector<std::uint32_t> v(n);
    for (std::size_t i = 0; i < n; ++i) v[i] = static_cast<std::uint32_t>(i);
    tris.reserve((n - 2) * 3);
    std::size_t guard = n * n + 8;
    std::size_t i = 0;
    while (v.size() > 3 && guard-- > 0) {
        const std::size_t m = v.size();
        const std::uint32_t a = v[(i + m - 1) % m], b = v[i % m], c = v[(i + 1) % m];
        bool ear = cross2(p[a], p[b], p[c]) > 1e-9f;
        for (std::size_t k = 0; ear && k < m; ++k) {
            const std::uint32_t q = v[k];
            if (q == a || q == b || q == c) continue;
            if (pointInTriangle(p[q], p[a], p[b], p[c])) ear = false;
        }
        if (ear) {
            tris.insert(tris.end(), {a, b, c});
            v.erase(v.begin() + static_cast<std::ptrdiff_t>(i % m));
            i = (i + m - 2) % (m - 1); // re-test the neighbour that just changed
        } else {
            i = (i + 1) % m;
        }
    }
    if (v.size() != 3) return {}; // not simple / degenerate
    tris.insert(tris.end(), {v[0], v[1], v[2]});
    return tris;
}

std::optional<Profile> profileFromCurve(const SketchCurve& c, std::string* why) {
    auto fail = [&](std::string msg) -> std::optional<Profile> {
        if (why) *why = std::move(msg);
        return std::nullopt;
    };
    if (c.construction) return fail(c.name + " is a construction curve: it guides the sketch but is no profile");
    Profile p;
    switch (c.kind) {
    case CurveKind::Circle:
        if (c.pts.empty() || !(c.radius > 0.0f)) return fail("circle has no radius");
        p.kind = Profile::Kind::Circle;
        p.center = c.pts[0];
        p.radius = c.radius;
        return p;
    case CurveKind::Rectangle:
        if (c.pts.size() < 2) return fail("rectangle needs two corners");
        p.points = {c.pts[0], {c.pts[1].x, c.pts[0].y}, c.pts[1], {c.pts[0].x, c.pts[1].y}};
        break;
    case CurveKind::Polyline:
        if (c.pts.size() < 4 || !samePoint(c.pts.front(), c.pts.back(), 1e-4f))
            return fail(c.name + " is an open polyline: close it (click its first point) to extrude");
        p.points = c.pts;
        break;
    case CurveKind::Spline:
    case CurveKind::Ellipse:
    case CurveKind::Slot:
        if (!c.isClosed()) return fail(c.name + " is an open spline: close it to extrude");
        p.points = c.planeStrip(0.01f);
        if (!p.points.empty()) p.points.pop_back(); // no repeated closing point
        break;
    default: return fail(std::string(kindName(c.kind)) + " is not a closed profile");
    }
    p.points = cleanPolygon(p.points);
    if (p.points.size() < 3 || std::fabs(signedArea(p.points)) < 1e-6f) return fail(c.name + " encloses no area");
    if (!isSimplePolygon(p.points)) return fail(c.name + " crosses itself and cannot be extruded");
    if (signedArea(p.points) < 0) std::reverse(p.points.begin(), p.points.end()); // counter-clockwise
    return p;
}

std::optional<Profile> profileFromSketch(const Sketch& sketch, std::size_t index, std::string* why) {
    if (index >= sketch.curves().size()) {
        if (why) *why = "No such curve";
        return std::nullopt;
    }
    const SketchCurve& c = sketch.curves()[index];
    std::string first;
    if (auto p = profileFromCurve(c, &first)) return p;
    // An open curve: the loop it closes with others, end to end.
    if (!c.construction && !c.isClosed()) {
        std::vector<Vec2> loop = chainLoop(sketch, index);
        if (!loop.empty()) {
            Profile p;
            p.points = cleanPolygon(loop);
            if (p.points.size() >= 3 && std::fabs(signedArea(p.points)) > 1e-6f && isSimplePolygon(p.points)) {
                if (signedArea(p.points) < 0) std::reverse(p.points.begin(), p.points.end());
                return p;
            }
            if (why) *why = "The curves joined to " + c.name + " cross each other";
            return std::nullopt;
        }
        if (why) *why = c.name + " is open and doesn't join other curves end to end into a closed loop";
        return std::nullopt;
    }
    if (why) *why = first;
    return std::nullopt;
}

std::vector<float> extrudeParams(const Profile& p, SketchPlane plane, float distance, bool symmetric) {
    float w0 = 0.0f, w1 = distance;
    if (symmetric) {
        w0 = -std::fabs(distance) * 0.5f;
        w1 = std::fabs(distance) * 0.5f;
    } else if (distance < 0.0f) {
        w0 = distance;
        w1 = 0.0f;
    }
    std::vector<float> params = {static_cast<float>(plane), w0, w1};
    if (p.kind == Profile::Kind::Circle) {
        params.insert(params.end(), {1.0f, p.center.x, p.center.y, p.radius});
    } else {
        params.push_back(0.0f);
        for (const Vec2& q : p.points) params.insert(params.end(), {q.x, q.y});
    }
    return params;
}

MeshData buildExtrusion(const std::vector<float>& prm) {
    MeshData m;
    if (prm.size() < 7) return m;
    const int planeIndex = static_cast<int>(prm[0]);
    if (planeIndex < 0 || planeIndex > 2) return m;
    const PlaneFrame f = planeFrame(static_cast<SketchPlane>(planeIndex));
    const float w0 = prm[1], w1 = prm[2];
    if (!(w1 - w0 > 1e-6f)) return m;

    std::vector<Vec2> poly;
    const bool circle = prm[3] == 1.0f;
    if (circle) {
        const Vec2 c{prm[4], prm[5]};
        const float r = prm[6];
        if (!(r > 0.0f)) return m;
        for (int i = 0; i < kCircleSegments; ++i) {
            const float a = 2.0f * kPi * static_cast<float>(i) / kCircleSegments;
            poly.push_back({c.x + r * std::cos(a), c.y + r * std::sin(a)});
        }
    } else {
        for (std::size_t i = 4; i + 1 < prm.size(); i += 2) poly.push_back({prm[i], prm[i + 1]});
        if (poly.size() < 3) return m;
    }
    if (signedArea(poly) < 0) std::reverse(poly.begin(), poly.end());

    auto world = [&](Vec2 q, float w) { return f.toWorld(q) + f.normal * w; };
    const std::size_t n = poly.size();

    // Caps (bottom faces -normal, top faces +normal).
    const std::vector<std::uint32_t> tri = circle ? std::vector<std::uint32_t>{} : triangulatePolygon(poly);
    if (!circle && tri.empty()) return {};
    auto addCap = [&](float w, bool top) {
        const Vec3 nrm = top ? f.normal : -f.normal;
        const auto base = static_cast<std::uint32_t>(m.vertices.size());
        for (const Vec2& q : poly) m.vertices.push_back({world(q, w), nrm});
        if (circle) { // fan around the centre
            const auto centre = static_cast<std::uint32_t>(m.vertices.size());
            m.vertices.push_back({world({prm[4], prm[5]}, w), nrm});
            for (std::uint32_t i = 0; i < n; ++i) {
                const std::uint32_t a = base + i, b = base + (i + 1) % static_cast<std::uint32_t>(n);
                if (top) m.indices.insert(m.indices.end(), {centre, a, b});
                else m.indices.insert(m.indices.end(), {centre, b, a});
            }
        } else {
            for (std::size_t t = 0; t + 2 < tri.size(); t += 3) {
                if (top) m.indices.insert(m.indices.end(), {base + tri[t], base + tri[t + 1], base + tri[t + 2]});
                else m.indices.insert(m.indices.end(), {base + tri[t], base + tri[t + 2], base + tri[t + 1]});
            }
        }
        for (std::size_t i = 0; i < n; ++i) { // outline edges
            m.edgeVertices.push_back(world(poly[i], w));
            m.edgeVertices.push_back(world(poly[(i + 1) % n], w));
        }
    };
    addCap(w0, false);
    addCap(w1, true);

    // Side walls: outward normals (counter-clockwise outline => edge direction x normal).
    const float cosFeature = std::cos(radians(kFeatureAngleDeg));
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 a = poly[i], b = poly[(i + 1) % n];
        Vec2 d{b.x - a.x, b.y - a.y};
        const float len = std::hypot(d.x, d.y);
        if (len < 1e-9f) continue;
        d = {d.x / len, d.y / len};
        auto planeNormal = [&](Vec2 q) {
            const float l = std::hypot(q.x, q.y);
            return normalize(f.u * (q.x / l) + f.v * (q.y / l));
        };
        Vec3 na, nb;
        if (circle) { // smooth: radial normals
            na = planeNormal({a.x - prm[4], a.y - prm[5]});
            nb = planeNormal({b.x - prm[4], b.y - prm[5]});
        } else {
            na = nb = normalize(f.u * d.y - f.v * d.x); // right-hand normal of a CCW edge points outward
        }
        const auto v = static_cast<std::uint32_t>(m.vertices.size());
        m.vertices.push_back({world(a, w0), na});
        m.vertices.push_back({world(b, w0), nb});
        m.vertices.push_back({world(b, w1), nb});
        m.vertices.push_back({world(a, w1), na});
        m.indices.insert(m.indices.end(), {v, v + 1, v + 2, v, v + 2, v + 3});

        // Vertical edge at vertex a where the outline turns noticeably.
        if (!circle) {
            const Vec2 p0 = poly[(i + n - 1) % n];
            Vec2 dPrev{a.x - p0.x, a.y - p0.y};
            const float lp = std::hypot(dPrev.x, dPrev.y);
            if (lp > 1e-9f && (dPrev.x * d.x + dPrev.y * d.y) / lp < cosFeature) {
                m.edgeVertices.push_back(world(a, w0));
                m.edgeVertices.push_back(world(a, w1));
            }
        }
    }
    return m;
}

} // namespace sicdm

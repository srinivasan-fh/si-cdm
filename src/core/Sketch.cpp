#include "core/Sketch.hpp"
#include "core/Units.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>

namespace sicdm {

std::uint64_t nextRevision() noexcept {
    static std::atomic<std::uint64_t> counter{0};
    return ++counter;
}

PlaneFrame planeFrame(SketchPlane plane) noexcept {
    switch (plane) {
    case SketchPlane::Top: return {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    case SketchPlane::Front: return {{0, 0, 0}, {1, 0, 0}, {0, 0, 1}, {0, -1, 0}};
    case SketchPlane::Right: return {{0, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 0}};
    case SketchPlane::Custom: break;
    }
    return {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
}

bool makePlaneFrame(Vec3 origin, Vec3 normal, Vec3 uHint, PlaneFrame& out) noexcept {
    const float nl = length(normal);
    if (!(nl > 1e-9f) || !std::isfinite(nl)) return false;
    const Vec3 n = normal / nl;
    Vec3 u = uHint - n * dot(uHint, n);
    if (!(length(u) > 1e-4f * std::max(1.0f, length(uHint)))) { // along the normal: X, else Y
        const Vec3 alt = std::fabs(n.x) < 0.9f ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
        u = alt - n * dot(alt, n);
    }
    u = normalize(u);
    out = {origin, u, cross(n, u), n};
    return true;
}

bool sameFrame(const PlaneFrame& a, const PlaneFrame& b) noexcept {
    auto within = [](Vec3 p, Vec3 q, float tol) { return length(p - q) <= tol; };
    return within(a.origin, b.origin, 1e-3f) && within(a.u, b.u, 1e-5f) && within(a.v, b.v, 1e-5f) && within(a.normal, b.normal, 1e-5f);
}

const char* planeName(SketchPlane plane) noexcept {
    switch (plane) {
    case SketchPlane::Top: return "Top (XY)";
    case SketchPlane::Front: return "Front (XZ)";
    case SketchPlane::Right: return "Right (YZ)";
    case SketchPlane::Custom: return "Custom";
    }
    return "";
}

const char* curveKindName(CurveKind k) noexcept {
    switch (k) {
    case CurveKind::Point: return "Point";
    case CurveKind::Line: return "Line";
    case CurveKind::Polyline: return "Polyline";
    case CurveKind::Rectangle: return "Rectangle";
    case CurveKind::Circle: return "Circle";
    case CurveKind::Arc: return "Arc";
    case CurveKind::Spline: return "Spline";
    case CurveKind::Ellipse: return "Ellipse";
    case CurveKind::Slot: return "Slot";
    }
    return "";
}

Vec2 splinePoint(const std::vector<Vec2>& pts, bool closed, std::size_t span, float t) {
    const std::size_t n = pts.size();
    if (n == 0) return {};
    if (n == 1) return pts[0];
    auto at = [&](long i) -> Vec2 {
        if (closed) return pts[static_cast<std::size_t>(((i % static_cast<long>(n)) + static_cast<long>(n)) % static_cast<long>(n))];
        if (i < 0) return {2 * pts[0].x - pts[1].x, 2 * pts[0].y - pts[1].y}; // the ends carry on straight
        if (i >= static_cast<long>(n)) return {2 * pts[n - 1].x - pts[n - 2].x, 2 * pts[n - 1].y - pts[n - 2].y};
        return pts[static_cast<std::size_t>(i)];
    };
    const long i = static_cast<long>(span);
    const Vec2 p0 = at(i - 1), p1 = at(i), p2 = at(i + 1), p3 = at(i + 2);
    // Centripetal knots (alpha 1/2).
    auto knot = [](Vec2 a, Vec2 b) { return std::max(std::sqrt(std::hypot(b.x - a.x, b.y - a.y)), 1e-6f); };
    const float t0 = 0, t1 = t0 + knot(p0, p1), t2 = t1 + knot(p1, p2), t3 = t2 + knot(p2, p3);
    const float u = t1 + (t2 - t1) * t;
    auto lerp = [](Vec2 a, Vec2 b, float ta, float tb, float x) {
        const float wa = (tb - x) / (tb - ta), wb = (x - ta) / (tb - ta);
        return Vec2{a.x * wa + b.x * wb, a.y * wa + b.y * wb};
    };
    const Vec2 q1 = lerp(p0, p1, t0, t1, u), q2 = lerp(p1, p2, t1, t2, u), q3 = lerp(p2, p3, t2, t3, u);
    const Vec2 r1 = lerp(q1, q2, t0, t2, u), r2 = lerp(q2, q3, t1, t3, u);
    return lerp(r1, r2, t1, t2, u);
}

Vec3 splinePoint3(const std::vector<Vec3>& pts, bool closed, std::size_t span, float t) {
    const std::size_t n = pts.size();
    if (n == 0) return {};
    if (n == 1) return pts[0];
    auto at = [&](long i) -> Vec3 {
        if (closed) return pts[static_cast<std::size_t>(((i % static_cast<long>(n)) + static_cast<long>(n)) % static_cast<long>(n))];
        if (i < 0) return pts[0] * 2.0f - pts[1]; // the ends carry on straight
        if (i >= static_cast<long>(n)) return pts[n - 1] * 2.0f - pts[n - 2];
        return pts[static_cast<std::size_t>(i)];
    };
    const long i = static_cast<long>(span);
    const Vec3 p0 = at(i - 1), p1 = at(i), p2 = at(i + 1), p3 = at(i + 2);
    // Centripetal knots (alpha 1/2), as splinePoint.
    auto knot = [](Vec3 a, Vec3 b) { return std::max(std::sqrt(length(b - a)), 1e-6f); };
    const float t0 = 0, t1 = t0 + knot(p0, p1), t2 = t1 + knot(p1, p2), t3 = t2 + knot(p2, p3);
    const float u = t1 + (t2 - t1) * t;
    auto lerp = [](Vec3 a, Vec3 b, float ta, float tb, float x) { return a * ((tb - x) / (tb - ta)) + b * ((x - ta) / (tb - ta)); };
    const Vec3 q1 = lerp(p0, p1, t0, t1, u), q2 = lerp(p1, p2, t1, t2, u), q3 = lerp(p2, p3, t2, t3, u);
    const Vec3 r1 = lerp(q1, q2, t0, t2, u), r2 = lerp(q2, q3, t1, t3, u);
    return lerp(r1, r2, t1, t2, u);
}

bool pathBends(const std::vector<Vec3>& pts, bool closed, float radius, std::vector<PathBend>& bends, std::string* why) {
    bends.clear();
    auto fail = [&](const std::string& w) {
        if (why) *why = w;
        bends.clear();
        return false;
    };
    const std::size_t n = pts.size();
    if (n < 2 || (closed && n < 3)) return fail(closed ? "A closed path needs three points" : "A path needs two points");
    const std::size_t segs = closed ? n : n - 1;
    for (std::size_t k = 0; k < segs; ++k)
        if (!(length(pts[(k + 1) % n] - pts[k]) > 1e-6f)) return fail("Points " + std::to_string(k + 1) + " and " + std::to_string((k + 1) % n + 1) + " are the same");
    if (!(radius > 0)) return true;
    std::vector<float> tangent(n, 0.0f);
    for (std::size_t c = closed ? 0 : 1; c < (closed ? n : n - 1); ++c) {
        const Vec3 P = pts[c], prev = pts[(c + n - 1) % n], next = pts[(c + 1) % n];
        const Vec3 a = normalize(P - prev), b = normalize(next - P);
        const double angle = std::acos(std::clamp(static_cast<double>(dot(a, b)), -1.0, 1.0));
        if (angle < 2e-4) continue; // straight on
        if (angle > 3.14159265358979 - 1e-3) return fail("The path turns back on itself at point " + std::to_string(c + 1));
        PathBend bd;
        bd.corner = c;
        bd.angle = static_cast<float>(angle);
        bd.radius = radius;
        bd.tangent = static_cast<float>(radius * std::tan(angle / 2));
        bd.axis = normalize(cross(a, b));
        bd.centre = P + normalize(b - a) * static_cast<float>(radius / std::cos(angle / 2));
        bd.from = P - a * bd.tangent;
        bd.to = P + b * bd.tangent;
        tangent[c] = bd.tangent;
        bends.push_back(bd);
    }
    for (std::size_t k = 0; k < segs; ++k) {
        const std::size_t j = (k + 1) % n;
        const float len = length(pts[j] - pts[k]), need = tangent[k] + tangent[j];
        if (need > len * (1 + 1e-5f) + 1e-5f)
            return fail("The bends at a radius of " + lengthText(radius) + " need " + lengthText(need) + " of the segment from point " + std::to_string(k + 1) +
                        " to " + std::to_string(j + 1) + ", which is " + lengthText(len) + " long");
    }
    return true;
}

std::vector<Vec3> spacePoints(const SketchCurve& c, bool* closed) {
    std::vector<Vec3> p;
    for (std::size_t i = 0; i < c.pts.size(); ++i) p.push_back(c.point3(i));
    bool cl = false;
    if (c.kind == CurveKind::Spline) cl = c.closed && p.size() >= 3;
    else if (p.size() > 3 && length(p.front() - p.back()) < 1e-6f) cl = true, p.pop_back();
    if (closed) *closed = cl;
    return p;
}

std::vector<Vec3> spaceStrip(const SketchCurve& c, float chord) {
    bool closed = false;
    const std::vector<Vec3> p = spacePoints(c, &closed);
    std::vector<Vec3> out;
    const std::size_t n = p.size();
    if (n < 2) return p;
    if (c.kind == CurveKind::Spline) {
        const std::size_t spans = closed ? n : n - 1;
        out.push_back(p[0]);
        for (std::size_t sp = 0; sp < spans; ++sp) {
            const float len = length(p[(sp + 1) % n] - p[sp]);
            const int steps = std::clamp(static_cast<int>(std::ceil(len / std::max(std::sqrt(8.0f * std::max(chord, 1e-3f) * len), 1e-3f) * 2.0f)), 4, 64);
            for (int i = 1; i <= steps; ++i) out.push_back(splinePoint3(p, closed, sp, static_cast<float>(i) / static_cast<float>(steps)));
        }
        return out;
    }
    std::vector<PathBend> bends;
    if (!pathBends(p, closed, c.radius, bends)) bends.clear(); // drawn with its corners sharp
    std::vector<const PathBend*> at(n, nullptr);
    for (const PathBend& b : bends) at[b.corner] = &b;
    auto corner = [&](std::size_t k) {
        const PathBend* b = at[k];
        if (!b) {
            out.push_back(p[k]);
            return;
        }
        float step = 0.3f;
        if (chord > 0 && chord < b->radius) step = std::min(step, 2.0f * std::acos(1.0f - chord / b->radius));
        const int segs = std::clamp(static_cast<int>(std::ceil(b->angle / std::max(step, 1e-3f))), 1, 1024);
        const Vec3 v = b->from - b->centre;
        out.push_back(b->from);
        for (int i = 1; i < segs; ++i) {
            const float phi = b->angle * static_cast<float>(i) / static_cast<float>(segs);
            const float cs = std::cos(phi), sn = std::sin(phi);
            out.push_back(b->centre + v * cs + cross(b->axis, v) * sn + b->axis * (dot(b->axis, v) * (1 - cs)));
        }
        out.push_back(b->to);
    };
    if (closed) {
        for (std::size_t k = 0; k < n; ++k) corner(k);
        out.push_back(out.front());
    } else {
        out.push_back(p[0]);
        for (std::size_t k = 1; k + 1 < n; ++k) corner(k);
        out.push_back(p[n - 1]);
    }
    return out;
}

Vec3 SketchCurve::point3(std::size_t i) const {
    if (i >= pts.size()) return {};
    const PlaneFrame f = curveFrame(*this);
    Vec3 p = f.toWorld(pts[i]);
    if (i < rise.size()) p += f.normal * rise[i];
    return p;
}

bool arcThrough(Vec2 a, Vec2 m, Vec2 b, Vec2& centre, float& radius, bool& ccw) {
    const double ax = a.x, ay = a.y, bx = m.x, by = m.y, cx = b.x, cy = b.y;
    const double d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
    const double scale = std::max({std::fabs(ax - cx), std::fabs(ay - cy), std::fabs(bx - ax), std::fabs(by - ay), 1e-9});
    if (std::fabs(d) < 1e-9 * scale * scale) return false;
    const double a2 = ax * ax + ay * ay, b2 = bx * bx + by * by, c2 = cx * cx + cy * cy;
    centre = {static_cast<float>((a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d), static_cast<float>((a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d)};
    radius = std::hypot(a.x - centre.x, a.y - centre.y);
    // Counter-clockwise when the middle point lies to the right of the chord from a to b.
    ccw = (m.x - a.x) * (b.y - a.y) - (m.y - a.y) * (b.x - a.x) > 0;
    return radius > 0;
}

void SketchCurve::arcAngles(float& a0, float& a1) const {
    a0 = a1 = 0;
    if (pts.size() < 3) return;
    a0 = std::atan2(pts[1].y - pts[0].y, pts[1].x - pts[0].x);
    a1 = std::atan2(pts[2].y - pts[0].y, pts[2].x - pts[0].x);
    while (a1 <= a0 + 1e-7f) a1 += 2.0f * kPi;
}

float ellipseParam(const SketchCurve& c, Vec2 p) {
    if (c.pts.size() < 2) return 0.0f;
    const Vec2 m{c.pts[1].x - c.pts[0].x, c.pts[1].y - c.pts[0].y};
    const float rx = std::hypot(m.x, m.y), ry = c.radius;
    if (!(rx > 0) || !(ry > 0)) return 0.0f;
    const Vec2 u{m.x / rx, m.y / rx}, d{p.x - c.pts[0].x, p.y - c.pts[0].y};
    return std::atan2((-u.y * d.x + u.x * d.y) / ry, (u.x * d.x + u.y * d.y) / rx);
}

Vec2 ellipsePointAt(const SketchCurve& c, float t) {
    if (c.pts.size() < 2) return {};
    const Vec2 m{c.pts[1].x - c.pts[0].x, c.pts[1].y - c.pts[0].y};
    const float rx = std::hypot(m.x, m.y);
    const Vec2 u = rx > 0 ? Vec2{m.x / rx, m.y / rx} : Vec2{1, 0};
    const float x = rx * std::cos(t), y = c.radius * std::sin(t);
    return {c.pts[0].x + x * u.x - y * u.y, c.pts[0].y + x * u.y + y * u.x};
}

void ellipseArcParams(const SketchCurve& c, float& t0, float& t1) {
    t0 = 0.0f, t1 = 2.0f * kPi;
    if (c.pts.size() < 4) return;
    t0 = ellipseParam(c, c.pts[2]);
    t1 = ellipseParam(c, c.pts[3]);
    while (t1 <= t0 + 1e-6f) t1 += 2.0f * kPi;
}

bool SketchCurve::isClosed() const {
    switch (kind) {
    case CurveKind::Rectangle:
    case CurveKind::Circle:
    case CurveKind::Slot: return true;
    case CurveKind::Ellipse: return pts.size() < 4; // an elliptical arc is open
    case CurveKind::Polyline:
        if (spatial()) return pts.size() > 3 && length(point3(0) - point3(pts.size() - 1)) < 1e-6f;
        return pts.size() > 3 && std::hypot(pts.front().x - pts.back().x, pts.front().y - pts.back().y) < 1e-6f;
    case CurveKind::Spline: return closed && pts.size() >= 3;
    default: return false;
    }
}

std::vector<Vec2> SketchCurve::planeStrip(float chord) const {
    std::vector<Vec2> out;
    auto arcSegments = [&](float r, float sweep) {
        float step = 0.3f;
        if (chord > 0 && chord < r) step = std::min(step, 2.0f * std::acos(1.0f - chord / r));
        return std::clamp(static_cast<int>(std::ceil(std::fabs(sweep) / std::max(step, 1e-3f))), 2, 4096);
    };
    auto arc = [&](Vec2 c, float r, float a0, float a1, bool first) {
        const int n = arcSegments(r, a1 - a0);
        for (int i = first ? 0 : 1; i <= n; ++i) {
            const float a = a0 + (a1 - a0) * static_cast<float>(i) / static_cast<float>(n);
            out.push_back({c.x + r * std::cos(a), c.y + r * std::sin(a)});
        }
    };
    switch (kind) {
    case CurveKind::Point:
    case CurveKind::Line:
    case CurveKind::Polyline: return pts;
    case CurveKind::Rectangle:
        if (pts.size() >= 2) {
            const Vec2 a = pts[0], b = pts[1];
            out = {a, Vec2{b.x, a.y}, b, Vec2{a.x, b.y}, a};
        }
        break;
    case CurveKind::Circle:
        if (!pts.empty()) arc(pts[0], radius, 0.0f, 2.0f * kPi, true);
        break;
    case CurveKind::Arc:
        if (pts.size() >= 3) {
            float a0, a1;
            arcAngles(a0, a1);
            const float r = std::hypot(pts[1].x - pts[0].x, pts[1].y - pts[0].y);
            arc(pts[0], r, a0, a1, true);
            // The ends exactly: the start point, and the end's direction at the start's radius.
            out.front() = pts[1];
            const float re = std::hypot(pts[2].x - pts[0].x, pts[2].y - pts[0].y);
            if (re > 0) out.back() = {pts[0].x + (pts[2].x - pts[0].x) * r / re, pts[0].y + (pts[2].y - pts[0].y) * r / re};
        }
        break;
    case CurveKind::Spline: {
        const std::size_t n = pts.size();
        if (n < 2) return pts;
        const std::size_t spans = closed && n >= 3 ? n : n - 1;
        out.push_back(pts[0]);
        for (std::size_t sp = 0; sp < spans; ++sp) {
            const Vec2 a = pts[sp], b = pts[(sp + 1) % n];
            const float len = std::hypot(b.x - a.x, b.y - a.y);
            const int steps = std::clamp(static_cast<int>(std::ceil(len / std::max(std::sqrt(8.0f * std::max(chord, 1e-3f) * len), 1e-3f) * 2.0f)), 4, 64);
            for (int i = 1; i <= steps; ++i) out.push_back(splinePoint(pts, closed && n >= 3, sp, static_cast<float>(i) / static_cast<float>(steps)));
        }
        break;
    }
    case CurveKind::Ellipse:
        if (pts.size() >= 2) {
            const Vec2 c = pts[0], m = {pts[1].x - pts[0].x, pts[1].y - pts[0].y};
            const float rx = std::hypot(m.x, m.y), rot = std::atan2(m.y, m.x), ry = radius;
            float t0 = 0.0f, t1 = 2.0f * kPi;
            if (pts.size() >= 4) ellipseArcParams(*this, t0, t1);
            const int n = arcSegments(std::max(rx, ry), t1 - t0);
            for (int i = 0; i <= n; ++i) {
                const float a = t0 + (t1 - t0) * static_cast<float>(i) / static_cast<float>(n);
                const float x = rx * std::cos(a), y = ry * std::sin(a);
                out.push_back({c.x + x * std::cos(rot) - y * std::sin(rot), c.y + x * std::sin(rot) + y * std::cos(rot)});
            }
        }
        break;
    case CurveKind::Slot:
        if (pts.size() >= 2) {
            const Vec2 a = pts[0], b = pts[1];
            const float ang = std::atan2(b.y - a.y, b.x - a.x);
            arc(b, radius, ang - 0.5f * kPi, ang + 0.5f * kPi, true);
            arc(a, radius, ang + 0.5f * kPi, ang + 1.5f * kPi, true);
            out.push_back(out.front());
        }
        break;
    }
    return out;
}

std::vector<Vec3> SketchCurve::worldStrip(int circleSegments) const {
    const PlaneFrame f = curveFrame(*this);
    std::vector<Vec3> out;
    if (spatial()) { // round parts as finely as a circle of `circleSegments`
        AABB box;
        for (std::size_t i = 0; i < pts.size(); ++i) box.expand(point3(i));
        const float size = std::max({radius, box.valid() ? 0.5f * length(box.max - box.min) : 0.0f, 1e-3f});
        return spaceStrip(*this, (kind == CurveKind::Spline ? size : std::max(radius, 1e-3f)) * (1.0f - std::cos(kPi / static_cast<float>(std::max(circleSegments, 3)))));
    }
    switch (kind) {
    case CurveKind::Point:
        if (!pts.empty()) out.push_back(f.toWorld(pts[0]));
        break;
    case CurveKind::Line:
    case CurveKind::Polyline:
        for (const Vec2& p : pts) out.push_back(f.toWorld(p));
        break;
    case CurveKind::Rectangle:
        if (pts.size() >= 2) {
            const Vec2 a = pts[0], b = pts[1];
            for (const Vec2& p : {a, Vec2{b.x, a.y}, b, Vec2{a.x, b.y}, a}) out.push_back(f.toWorld(p));
        }
        break;
    case CurveKind::Circle:
        if (!pts.empty()) {
            for (int i = 0; i <= circleSegments; ++i) {
                const float a = 2.0f * kPi * static_cast<float>(i) / static_cast<float>(circleSegments);
                out.push_back(f.toWorld({pts[0].x + radius * std::cos(a), pts[0].y + radius * std::sin(a)}));
            }
        }
        break;
    default: { // round parts as finely as a circle of `circleSegments`
        float size = radius;
        for (const Vec2& p : pts) size = std::max({size, std::fabs(p.x - pts[0].x), std::fabs(p.y - pts[0].y)});
        const float chord = std::max(size, 1e-3f) * (1.0f - std::cos(kPi / static_cast<float>(std::max(circleSegments, 3))));
        for (const Vec2& p : planeStrip(chord)) out.push_back(f.toWorld(p));
        break;
    }
    }
    return out;
}

std::vector<Vec2> SketchCurve::keyPoints() const {
    if (kind == CurveKind::Arc && pts.size() >= 3) {
        const std::vector<Vec2> strip = planeStrip(1e9f); // ends exact
        float a0, a1;
        arcAngles(a0, a1);
        const float r = std::hypot(pts[1].x - pts[0].x, pts[1].y - pts[0].y), am = 0.5f * (a0 + a1);
        return {pts[0], strip.front(), strip.back(), {pts[0].x + r * std::cos(am), pts[0].y + r * std::sin(am)}};
    }
    if (kind == CurveKind::Ellipse && pts.size() >= 2) {
        const Vec2 c = pts[0], m = {pts[1].x - pts[0].x, pts[1].y - pts[0].y};
        const float rx = std::hypot(m.x, m.y);
        const Vec2 n = rx > 0 ? Vec2{-m.y / rx * radius, m.x / rx * radius} : Vec2{0, radius};
        if (pts.size() >= 4) {
            float t0, t1;
            ellipseArcParams(*this, t0, t1);
            return {c, ellipsePointAt(*this, t0), ellipsePointAt(*this, t1), ellipsePointAt(*this, 0.5f * (t0 + t1))};
        }
        return {c, {c.x + m.x, c.y + m.y}, {c.x - m.x, c.y - m.y}, {c.x + n.x, c.y + n.y}, {c.x - n.x, c.y - n.y}};
    }
    if (kind == CurveKind::Slot && pts.size() >= 2) {
        const Vec2 a = pts[0], b = pts[1];
        const float l = std::hypot(b.x - a.x, b.y - a.y);
        const Vec2 n = l > 0 ? Vec2{-(b.y - a.y) / l * radius, (b.x - a.x) / l * radius} : Vec2{0, radius};
        return {a, b, {a.x + n.x, a.y + n.y}, {a.x - n.x, a.y - n.y}, {b.x + n.x, b.y + n.y}, {b.x - n.x, b.y - n.y}};
    }
    if (kind == CurveKind::Rectangle && pts.size() >= 2)
        return {pts[0], {pts[1].x, pts[0].y}, pts[1], {pts[0].x, pts[1].y},
                {(pts[0].x + pts[1].x) * 0.5f, (pts[0].y + pts[1].y) * 0.5f}};
    if (kind == CurveKind::Circle && !pts.empty())
        return {pts[0], {pts[0].x + radius, pts[0].y}, {pts[0].x - radius, pts[0].y},
                {pts[0].x, pts[0].y + radius}, {pts[0].x, pts[0].y - radius}};
    return pts;
}

std::string SketchCurve::measure() const {
    auto len = [](Vec2 a, Vec2 b) { return static_cast<double>(std::hypot(b.x - a.x, b.y - a.y)); };
    switch (kind) {
    case CurveKind::Point:
        if (!pts.empty()) return lengthsText({pts[0].x, pts[0].y}, ", ", "(", ")");
        break;
    case CurveKind::Line:
        if (pts.size() >= 2) return "L " + lengthText(len(pts[0], pts[1]));
        break;
    case CurveKind::Polyline: {
        if (spatial()) { // exactly: the segments, less what the bends cut off, plus their arcs
            bool closedPath = false;
            const std::vector<Vec3> p = spacePoints(*this, &closedPath);
            double total = 0.0;
            for (std::size_t i = 0; i + (closedPath ? 0 : 1) < p.size(); ++i) total += length(p[(i + 1) % p.size()] - p[i]);
            std::vector<PathBend> bends;
            if (pathBends(p, closedPath, radius, bends))
                for (const PathBend& b : bends) total += static_cast<double>(b.radius) * b.angle - 2.0 * b.tangent;
            return "L " + lengthText(total) + " (" + std::to_string(pts.empty() ? 0 : pts.size() - 1) + " seg" + (radius > 0 ? ", bends R " + lengthText(radius) : "") + ")";
        }
        double total = 0.0;
        for (std::size_t i = 1; i < pts.size(); ++i) total += len(pts[i - 1], pts[i]);
        return "L " + lengthText(total) + " (" + std::to_string(pts.empty() ? 0 : pts.size() - 1) + " seg)";
    }
    case CurveKind::Rectangle:
        if (pts.size() >= 2) return lengthsText({std::fabs(pts[1].x - pts[0].x), std::fabs(pts[1].y - pts[0].y)}, " x ");
        break;
    case CurveKind::Circle: return "R " + lengthText(radius);
    case CurveKind::Arc:
        if (pts.size() >= 3) {
            float a0, a1;
            arcAngles(a0, a1);
            char b[32];
            std::snprintf(b, sizeof b, " %.1f deg", static_cast<double>((a1 - a0) * 180.0f / kPi));
            return "R " + lengthText(len(pts[0], pts[1])) + b;
        }
        break;
    case CurveKind::Spline: {
        if (spatial()) {
            const std::vector<Vec3> s = spaceStrip(*this, 0.01f);
            double total = 0.0;
            for (std::size_t i = 1; i < s.size(); ++i) total += length(s[i] - s[i - 1]);
            return "L " + lengthText(total) + " (" + std::to_string(pts.size()) + " points" + (closed ? ", closed)" : ")");
        }
        const std::vector<Vec2> s = planeStrip(0.01f);
        double total = 0.0;
        for (std::size_t i = 1; i < s.size(); ++i) total += len(s[i - 1], s[i]);
        return "L " + lengthText(total) + " (" + std::to_string(pts.size()) + " points" + (closed ? ", closed)" : ")");
    }
    case CurveKind::Ellipse:
        if (pts.size() >= 2) return lengthsText({len(pts[0], pts[1]), radius}, " x ", "R ") + (pts.size() >= 4 ? " arc" : "");
        break;
    case CurveKind::Slot:
        if (pts.size() >= 2) return lengthsText({len(pts[0], pts[1]) + 2.0 * radius, 2.0 * radius}, " x ");
        break;
    }
    return "";
}

SketchCurve& Sketch::add(SketchCurve c) {
    // Numbering continues past loaded curves so new names stay unique.
    ++m_serial;
    if (c.name.empty()) c.name = std::string(c.spatial() ? "3D " : "") + curveKindName(c.kind) + " " + std::to_string(m_serial);
    m_revision = nextRevision();
    return m_curves.emplace_back(std::move(c));
}

void Sketch::remove(std::size_t index) {
    if (index >= m_curves.size()) return;
    const std::string gone = m_curves[index].name;
    m_curves.erase(m_curves.begin() + static_cast<std::ptrdiff_t>(index));
    // Its constraints go with it.
    std::erase_if(m_constraints, [&](const SketchConstraint& c) {
        return std::any_of(c.refs.begin(), c.refs.end(), [&](const SketchRef& r) { return r.curve == gone; });
    });
    m_revision = nextRevision();
}

AABB Sketch::bounds() const {
    AABB b;
    for (const SketchCurve& c : m_curves)
        if (c.visible)
            for (const Vec3& p : c.worldStrip(32)) b.expand(p);
    return b;
}

} // namespace sicdm

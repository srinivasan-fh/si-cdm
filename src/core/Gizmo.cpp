// The move / rotate / scale gizmo: handles, picking, drags, and the transforms they end in.

#include "core/Gizmo.hpp"

#include "core/Armature.hpp"
#include "core/Scene.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace sicdm {

namespace {

const Vec3 kAxis[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

// A direction square to `a`, and to the view where it can be (so shapes face the camera).
Vec3 across(const Vec3& a, const Vec3& view) {
    Vec3 c = cross(a, view);
    if (length(c) < 1e-4f) c = cross(a, std::fabs(a.z) < 0.9f ? Vec3{0, 0, 1} : Vec3{1, 0, 0});
    return normalize(c);
}

// Two directions spanning the plane square to `n`.
void spanOf(const Vec3& n, Vec3& u, Vec3& v) {
    u = normalize(cross(n, std::fabs(n.z) < 0.9f ? Vec3{0, 0, 1} : Vec3{1, 0, 0}));
    v = cross(n, u);
}

float segmentDistance(const Vec2& p, const Vec2& a, const Vec2& b) {
    const float dx = b.x - a.x, dy = b.y - a.y, l2 = dx * dx + dy * dy;
    float t = l2 > 0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / l2 : 0;
    t = std::clamp(t, 0.0f, 1.0f);
    return std::hypot(p.x - (a.x + dx * t), p.y - (a.y + dy * t));
}

bool inside(const Vec2& p, const std::vector<Vec2>& poly) {
    bool in = false;
    for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++)
        if ((poly[i].y > p.y) != (poly[j].y > p.y) && p.x < (poly[j].x - poly[i].x) * (p.y - poly[i].y) / (poly[j].y - poly[i].y) + poly[i].x) in = !in;
    return in;
}

// Where along the line pivot + axis t the ray passes closest; nothing when they run alike.
std::optional<float> alongAxis(const Vec3& pivot, const Vec3& axis, const Vec3& o, const Vec3& d) {
    const Vec3 w0 = pivot - o;
    const float b = dot(axis, d), c = dot(d, d), dd = dot(axis, w0), ee = dot(d, w0);
    const float denom = c - b * b; // |axis| = 1
    if (std::fabs(denom) < 1e-6f * c) return std::nullopt;
    return (b * ee - c * dd) / denom;
}

std::optional<Vec3> onPlane(const Vec3& p, const Vec3& n, const Vec3& o, const Vec3& d) {
    const float den = dot(d, n);
    if (std::fabs(den) < 1e-6f) return std::nullopt;
    const float t = dot(p - o, n) / den;
    if (t < 0) return std::nullopt;
    return o + d * t;
}

float snapTo(float v, float step) { return step > 0 ? std::round(v / step) * step : v; }

std::string num(double v) {
    char b[48];
    std::snprintf(b, sizeof b, "%.6g", std::fabs(v) < 5e-10 ? 0.0 : v);
    return b;
}

std::string vec(const Vec3& v) { return num(v.x) + "," + num(v.y) + "," + num(v.z); }
// A point or a distance in mm, marked so: bare numbers are read in the document's units.
std::string mmVec(const Vec3& v) { return num(v.x) + "mm," + num(v.y) + "mm," + num(v.z) + "mm"; }

} // namespace

std::vector<GizmoStroke> gizmoStrokes(GizmoMode mode, const Vec3& P, float size, const Vec3& view, const GizmoProject& project) {
    std::vector<GizmoStroke> out;
    auto stroke = [&](GizmoHandle h, std::initializer_list<Vec3> pts, bool closed, bool fill) {
        GizmoStroke s{h, {}, closed, fill};
        for (const Vec3& p : pts) {
            const auto q = project(p);
            if (!q) return;
            s.points.push_back(*q);
        }
        out.push_back(std::move(s));
    };
    auto ring = [&](GizmoHandle h, const Vec3& n, float r) {
        Vec3 u, v;
        spanOf(n, u, v);
        GizmoStroke s{h, {}, true, false};
        for (int i = 0; i < 64; ++i) {
            const float a = 2 * kPi * static_cast<float>(i) / 64.0f;
            const auto q = project(P + (u * std::cos(a) + v * std::sin(a)) * r);
            if (!q) return;
            s.points.push_back(*q);
        }
        out.push_back(std::move(s));
    };
    Vec3 su, sv; // the view's plane, for the shapes that face the camera
    spanOf(view, su, sv);
    if (mode == GizmoMode::Move) {
        for (int i = 0; i < 3; ++i) {
            const Vec3 e = kAxis[i], side = across(e, view) * (size * 0.07f);
            stroke({GizmoHandle::Axis, i}, {P, P + e * (size * 0.8f)}, false, false);
            stroke({GizmoHandle::Axis, i}, {P + e * size, P + e * (size * 0.78f) + side, P + e * (size * 0.78f) - side}, true, true);
            const Vec3 a = kAxis[(i + 1) % 3], b = kAxis[(i + 2) % 3];
            stroke({GizmoHandle::Plane, i}, {P + (a * 0.22f + b * 0.22f) * size, P + (a * 0.4f + b * 0.22f) * size, P + (a * 0.4f + b * 0.4f) * size, P + (a * 0.22f + b * 0.4f) * size},
                   true, true);
        }
        GizmoStroke c{{GizmoHandle::Center, 0}, {}, true, true};
        for (int i = 0; i < 16; ++i) {
            const float t = 2 * kPi * static_cast<float>(i) / 16.0f;
            if (const auto q = project(P + (su * std::cos(t) + sv * std::sin(t)) * (size * 0.08f))) c.points.push_back(*q);
        }
        if (c.points.size() == 16) out.push_back(std::move(c));
    } else if (mode == GizmoMode::Rotate) {
        for (int i = 0; i < 3; ++i) ring({GizmoHandle::Ring, i}, kAxis[i], size);
        ring({GizmoHandle::ViewRing, 0}, view, size * 1.18f);
    } else {
        for (int i = 0; i < 3; ++i) {
            const Vec3 e = kAxis[i], tip = P + e * size;
            const float s = size * 0.06f;
            stroke({GizmoHandle::Axis, i}, {P, tip}, false, false);
            stroke({GizmoHandle::Axis, i}, {tip + (su + sv) * s, tip + (su - sv) * s, tip - (su + sv) * s, tip - (su - sv) * s}, true, true);
        }
        const float s = size * 0.09f;
        stroke({GizmoHandle::Center, 0}, {P + (su + sv) * s, P + (su - sv) * s, P - (su + sv) * s, P - (su - sv) * s}, true, true);
    }
    return out;
}

GizmoHandle gizmoPick(GizmoMode mode, const Vec3& pivot, float size, const Vec3& view, const GizmoProject& project, const Vec2& mouse, float reach) {
    GizmoHandle best;
    float bestD = reach;
    for (const GizmoStroke& s : gizmoStrokes(mode, pivot, size, view, project)) {
        if (s.points.empty()) continue;
        float d = 1e30f;
        const std::size_t n = s.points.size();
        for (std::size_t i = 0; i + 1 < n || (s.closed && i < n); ++i) d = std::min(d, segmentDistance(mouse, s.points[i], s.points[(i + 1) % n]));
        // Inside a filled handle beats a line that passes there (the axes all meet at the centre).
        if (s.fill && n >= 3 && inside(mouse, s.points)) d = -1;
        if (d < bestD || (d <= bestD && (best.kind == GizmoHandle::None || s.fill))) bestD = d, best = s.handle;
    }
    return best;
}

GizmoDelta gizmoDrag(GizmoMode mode, const GizmoHandle& h, const Vec3& P, const Vec3& view, const GizmoProject& project, const GizmoRay& start,
                     const GizmoRay& now, float step, const GizmoDelta* previous) {
    GizmoDelta d;
    if (h.kind == GizmoHandle::None) return d;
    const Vec3 e = kAxis[std::clamp(h.axis, 0, 2)];
    if (mode == GizmoMode::Move) {
        if (h.kind == GizmoHandle::Axis) {
            const auto t0 = alongAxis(P, e, start.origin, start.dir), t1 = alongAxis(P, e, now.origin, now.dir);
            if (t0 && t1) d.move = e * snapTo(*t1 - *t0, step);
        } else {
            const Vec3 n = h.kind == GizmoHandle::Plane ? e : view;
            const auto a = onPlane(P, n, start.origin, start.dir), b = onPlane(P, n, now.origin, now.dir);
            if (a && b) {
                Vec3 m = *b - *a;
                m = m - n * dot(m, n);
                d.move = {snapTo(m.x, step), snapTo(m.y, step), snapTo(m.z, step)};
            }
        }
        return d;
    }
    const auto p = project(P);
    if (mode == GizmoMode::Rotate) {
        if (!p) return d;
        // The turn of the mouse about the pivot on screen; it turns the ring's way when the
        // ring's axis points at the viewer.
        const float a0 = std::atan2(-(start.mouse.y - p->y), start.mouse.x - p->x), a1 = std::atan2(-(now.mouse.y - p->y), now.mouse.x - p->x);
        float da = a1 - a0;
        while (da > kPi) da -= 2 * kPi;
        while (da < -kPi) da += 2 * kPi;
        if (h.kind == GizmoHandle::ViewRing) {
            d.axis = view * -1.0f;
        } else {
            d.axis = e;
            if (dot(e, view) > 0) da = -da;
        }
        // On from the last frame: the whole turns the mouse has gone round, not just the angle
        // between where it started and where it is.
        float raw = degrees(da);
        if (previous) raw += 360.0f * std::round((previous->rawAngle - raw) / 360.0f);
        d.rawAngle = raw;
        d.angle = snapTo(raw, step);
        return d;
    }
    // Scale
    if (h.kind == GizmoHandle::Axis) {
        const auto t0 = alongAxis(P, e, start.origin, start.dir), t1 = alongAxis(P, e, now.origin, now.dir);
        if (t0 && t1 && std::fabs(*t0) > 1e-6f) {
            float k = std::max(0.01f, snapTo(*t1 / *t0, step));
            if (!(k > 0)) k = 0.01f;
            d.scale = {1, 1, 1};
            (h.axis == 0 ? d.scale.x : h.axis == 1 ? d.scale.y : d.scale.z) = k;
            d.uniform = false;
        }
    } else if (p) {
        const float r0 = std::hypot(start.mouse.x - p->x, start.mouse.y - p->y), r1 = std::hypot(now.mouse.x - p->x, now.mouse.y - p->y);
        if (r0 > 1) {
            const float k = std::max(0.01f, snapTo(r1 / r0, step));
            d.scale = {k, k, k};
        }
    }
    return d;
}

std::string gizmoCommand(GizmoMode mode, const GizmoDelta& d, const std::string& name, const Vec3& pivot) {
    return gizmoCommand(mode, d, std::vector<std::string>{name}, pivot);
}

std::string gizmoCommand(GizmoMode mode, const GizmoDelta& d, const std::vector<std::string>& names, const Vec3& pivot) {
    if (names.empty()) return {};
    std::string q;
    for (const std::string& n : names) q += (q.empty() ? "\"" : " \"") + n + "\"";
    if (mode == GizmoMode::Move) {
        if (length(d.move) < 1e-6f) return {};
        return "MOVE " + mmVec(d.move) + " " + q;
    }
    if (mode == GizmoMode::Rotate) {
        if (std::fabs(d.angle) < 1e-6f) return {};
        return "ROTATE " + q + " BY " + num(d.angle) + " AXIS " + vec(d.axis) + " ABOUT " + mmVec(pivot);
    }
    if (std::fabs(d.scale.x - 1) < 1e-6f && std::fabs(d.scale.y - 1) < 1e-6f && std::fabs(d.scale.z - 1) < 1e-6f) return {};
    if (d.uniform) return "SCALE " + q + " " + num(d.scale.x) + " ABOUT " + mmVec(pivot);
    return "SCALE " + q + " " + vec(d.scale) + " ABOUT " + mmVec(pivot);
}

bool turnBody(Body& b, const Vec3& axis, float deg, const Vec3& about, std::string* why) {
    auto fail = [&](const std::string& w) {
        if (why) *why = w;
        return false;
    };
    if (!(length(axis) > 0)) return fail("The axis has no length");
    if (b.linked()) return fail(b.name + " is linked component " + b.component + ": COMPONENT ROTATE turns it");
    if (!b.component.empty()) return fail(b.name + " is part of " + b.component + ": its pose comes from the component");
    const Vec3 u = normalize(axis);
    const Vec3 e = quatToEuler(quatMultiply(quatFromAxisAngle(u, radians(deg)), quatFromEuler(b.rotation)));
    const Vec3 offset = about + rotateAxis(u, radians(deg)).transformDir(b.offset - about);
    b.setRotation(e);
    b.offset = offset;
    return true;
}

bool stretchBody(Body& b, const Vec3& k, const Vec3& about, std::string* why) {
    auto fail = [&](const std::string& w) {
        if (why) *why = w;
        return false;
    };
    if (!(k.x > 0) || !(k.y > 0) || !(k.z > 0)) return fail("The factors must be above 0");
    if (!b.component.empty()) return fail(b.name + " is part of " + b.component + ": SCALE the component");
    const bool uniform = std::fabs(k.x - k.y) < 1e-7f * k.x && std::fabs(k.y - k.z) < 1e-7f * k.x;
    if (uniform) {
        b.setScale(b.scale * k.x);
    } else {
        b.mesh.transform(scale(k));
        b.kind = "mesh"; // no longer the primitive it was
        b.params.clear();
    }
    b.offset = about + Vec3{(b.offset.x - about.x) * k.x, (b.offset.y - about.y) * k.y, (b.offset.z - about.z) * k.z};
    return true;
}

} // namespace sicdm

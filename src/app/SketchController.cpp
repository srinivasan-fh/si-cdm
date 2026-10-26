#include "app/SketchController.hpp"

#include "core/Units.hpp"

#include <cmath>
#include <cstdio>

namespace sicdm {

namespace {

constexpr float kPointSnapPx = 10.0f;
constexpr ImU32 kPreview = IM_COL32(255, 190, 90, 255);
constexpr ImU32 kSnap = IM_COL32(120, 220, 255, 255);

float dist(Vec2 a, Vec2 b) { return std::hypot(b.x - a.x, b.y - a.y); }

} // namespace

std::optional<ImVec2> projectToScreen(const OrbitCamera& cam, const Vec3& p, ImVec2 vp) {
    const Vec4 c = cam.viewProjection() * Vec4{p.x, p.y, p.z, 1.0f};
    if (c.w <= 1e-6f) return std::nullopt;
    return ImVec2{(c.x / c.w * 0.5f + 0.5f) * vp.x, (0.5f - c.y / c.w * 0.5f) * vp.y};
}

float gridStep(const OrbitCamera& cam) {
    const float viewSize = cam.distance() * std::tan(cam.fovY() * 0.5f) * 2.0f;
    return std::pow(10.0f, std::floor(std::log10(std::max(viewSize / 20.0f, 1e-4f))));
}

bool SketchController::hover(const Scene& scene, const OrbitCamera& cam, SketchPlane plane, const PlaneFrame& frame, ImVec2 mouse,
                             ImVec2 vp, const SnapOptions& snap, bool constrain) {
    if (plane != m_plane || !sameFrame(frame, m_frame)) {
        m_points.clear(); // a curve never spans two planes
        m_plane = plane;
        m_frame = frame;
    }
    const PlaneFrame f = frame;
    const Ray ray = cam.screenRay(mouse.x, mouse.y, vp.x, vp.y);
    float t = 0.0f;
    const float facing = std::fabs(dot(ray.dir, f.normal));
    if (facing < 0.05f) { // nearly edge-on: placement would be unstable
        m_hasHover = false;
        return false;
    }
    // Planes are infinite and two-sided: accept hits behind the ray origin in ortho views.
    t = dot(f.origin - ray.origin, f.normal) / dot(ray.dir, f.normal);
    Vec2 p = f.toPlane(ray.at(t));

    // 1) Snap to existing key points (and our own clicked points) in screen space.
    m_snappedToPoint = false;
    if (snap.points) {
        float best = kPointSnapPx;
        auto consider = [&](Vec2 q) {
            const auto s = projectToScreen(cam, f.toWorld(q), vp);
            if (!s) return;
            const float d = std::hypot(s->x - mouse.x, s->y - mouse.y);
            if (d < best) {
                best = d;
                p = q;
                m_snappedToPoint = true;
            }
        };
        for (const SketchCurve& c : scene.sketch().curves())
            if (c.visible && onPlane(c, plane, frame))
                for (Vec2 q : c.keyPoints()) consider(q);
        for (Vec2 q : m_points) consider(q);
    }
    // 2) Otherwise snap to the visible grid.
    if (!m_snappedToPoint && snap.grid) {
        const float step = gridStep(cam);
        p = {std::round(p.x / step) * step, std::round(p.y / step) * step};
    }
    // 3) Shift: horizontal / vertical from the previous point.
    if (constrain && !m_points.empty()) {
        const Vec2 last = m_points.back();
        if (std::fabs(p.x - last.x) > std::fabs(p.y - last.y)) p.y = last.y;
        else p.x = last.x;
    }
    m_hover = p;
    m_hasHover = true;
    return true;
}

SketchCurve SketchController::makeCurve(CurveKind kind, std::vector<Vec2> pts, float radius) const {
    SketchCurve c;
    c.kind = kind;
    c.plane = m_plane;
    c.frame = m_frame;
    c.pts = std::move(pts);
    c.radius = radius;
    return c;
}

std::optional<SketchCurve> SketchController::pending(ToolId tool, Vec2 q) const {
    const std::size_t n = m_points.size();
    auto distToSegment = [](Vec2 p, Vec2 a, Vec2 b) {
        const float dx = b.x - a.x, dy = b.y - a.y, l2 = dx * dx + dy * dy;
        const float t = l2 > 0 ? std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / l2, 0.0f, 1.0f) : 0.0f;
        return std::hypot(p.x - (a.x + dx * t), p.y - (a.y + dy * t));
    };
    switch (tool) {
    case ToolId::Arc: // start, end, then a point on it
        if (n == 2) {
            Vec2 c;
            float r;
            bool ccw;
            if (!arcThrough(m_points[0], q, m_points[1], c, r, ccw)) return std::nullopt;
            return makeCurve(CurveKind::Arc, ccw ? std::vector<Vec2>{c, m_points[0], m_points[1]} : std::vector<Vec2>{c, m_points[1], m_points[0]}, r);
        }
        return std::nullopt;
    case ToolId::Ellipse: // centre, major axis end, then the minor radius
        if (n == 1 && dist(m_points[0], q) > 0) return makeCurve(CurveKind::Ellipse, {m_points[0], q}, dist(m_points[0], q) * 0.5f);
        if (n == 2) {
            const Vec2 c = m_points[0], m = m_points[1];
            const float l = dist(c, m);
            if (!(l > 0)) return std::nullopt;
            const float ry = std::fabs((m.x - c.x) * (q.y - c.y) - (m.y - c.y) * (q.x - c.x)) / l;
            if (!(ry > 0)) return std::nullopt;
            return makeCurve(CurveKind::Ellipse, {c, m}, ry);
        }
        return std::nullopt;
    case ToolId::Slot: // the two end centres, then the width
        if (n == 1 && dist(m_points[0], q) > 0) return makeCurve(CurveKind::Slot, {m_points[0], q}, dist(m_points[0], q) * 0.15f);
        if (n == 2 && dist(m_points[0], m_points[1]) > 0) {
            const float r = distToSegment(q, m_points[0], m_points[1]);
            if (!(r > 0)) return std::nullopt;
            return makeCurve(CurveKind::Slot, {m_points[0], m_points[1]}, r);
        }
        return std::nullopt;
    case ToolId::Polygon: // centre, then a corner
        if (n == 1 && dist(m_points[0], q) > 0) {
            const Vec2 c = m_points[0];
            const float r = dist(c, q), a0 = std::atan2(q.y - c.y, q.x - c.x);
            std::vector<Vec2> pts;
            for (int k = 0; k <= m_polygonSides; ++k) {
                const float a = a0 + 2.0f * kPi * static_cast<float>(k % m_polygonSides) / static_cast<float>(m_polygonSides);
                pts.push_back({c.x + r * std::cos(a), c.y + r * std::sin(a)});
            }
            return makeCurve(CurveKind::Polyline, pts);
        }
        return std::nullopt;
    case ToolId::Spline:
        if (n >= 1) {
            std::vector<Vec2> pts = m_points;
            if (dist(pts.back(), q) > 1e-6f) pts.push_back(q);
            if (pts.size() >= 2) return makeCurve(CurveKind::Spline, pts);
        }
        return std::nullopt;
    default: return std::nullopt;
    }
}

std::optional<SketchCurve> SketchController::click(ToolId tool, bool doubleClick) {
    if (!m_hasHover) return std::nullopt;
    const Vec2 p = m_hover;
    if (tool != m_tool) {
        m_points.clear();
        m_tool = tool;
    }
    switch (tool) {
    case ToolId::Arc:
    case ToolId::Ellipse:
    case ToolId::Slot:
        if (m_points.size() < 2) {
            if (m_points.empty() || dist(p, m_points.back()) > 1e-6f) m_points.push_back(p);
            return std::nullopt;
        }
        if (auto c = pending(tool, p)) {
            m_points.clear();
            return c;
        }
        return std::nullopt;
    case ToolId::Polygon:
        if (m_points.empty()) {
            m_points.push_back(p);
            return std::nullopt;
        }
        if (auto c = pending(tool, p)) {
            m_points.clear();
            return c;
        }
        return std::nullopt;
    case ToolId::Spline: {
        // Like a polyline: clicking the first point closes it, double-click finishes.
        const bool closes = m_points.size() >= 3 && m_snappedToPoint && dist(p, m_points[0]) < 1e-6f;
        if (closes) {
            auto c = makeCurve(CurveKind::Spline, m_points);
            c.closed = true;
            m_points.clear();
            return c;
        }
        if (m_points.empty() || dist(p, m_points.back()) > 1e-6f) m_points.push_back(p);
        if (doubleClick) return finish();
        return std::nullopt;
    }
    case ToolId::Point: return makeCurve(CurveKind::Point, {p});
    case ToolId::Line:
        if (m_points.empty()) {
            m_points.push_back(p);
            return std::nullopt;
        }
        if (dist(m_points[0], p) < 1e-6f) return std::nullopt; // zero length
        {
            auto c = makeCurve(CurveKind::Line, {m_points[0], p});
            m_points.clear();
            return c;
        }
    case ToolId::Rectangle:
        if (m_points.empty()) {
            m_points.push_back(p);
            return std::nullopt;
        }
        if (std::fabs(p.x - m_points[0].x) < 1e-6f || std::fabs(p.y - m_points[0].y) < 1e-6f) return std::nullopt;
        {
            auto c = makeCurve(CurveKind::Rectangle, {m_points[0], p});
            m_points.clear();
            return c;
        }
    case ToolId::Circle:
        if (m_points.empty()) {
            m_points.push_back(p);
            return std::nullopt;
        }
        if (dist(m_points[0], p) < 1e-6f) return std::nullopt;
        {
            auto c = makeCurve(CurveKind::Circle, {m_points[0]}, dist(m_points[0], p));
            m_points.clear();
            return c;
        }
    case ToolId::Polyline: {
        // Clicking the first point closes the loop; double-click finishes.
        const bool closes = m_points.size() >= 2 && m_snappedToPoint && dist(p, m_points[0]) < 1e-6f;
        if (m_points.empty() || dist(p, m_points.back()) > 1e-6f) m_points.push_back(p);
        if (closes || doubleClick) return finish();
        return std::nullopt;
    }
    default: return std::nullopt;
    }
}

std::optional<SketchCurve> SketchController::typedPoint(ToolId tool, SketchPlane plane, const PlaneFrame& frame, Vec2 p) {
    if (plane != m_plane || !sameFrame(frame, m_frame)) {
        m_points.clear();
        m_plane = plane;
        m_frame = frame;
    }
    m_hover = p;
    m_hasHover = true;
    // Typing the first point of a polyline or spline again closes it, like clicking it.
    m_snappedToPoint = (tool == ToolId::Polyline || tool == ToolId::Spline) && m_points.size() >= 2 && dist(p, m_points[0]) < 1e-6f;
    return click(tool, false);
}

std::optional<SketchCurve> SketchController::finish() {
    if (m_points.size() < 2 || (m_tool != ToolId::Polyline && m_tool != ToolId::Spline)) {
        m_points.clear();
        return std::nullopt;
    }
    auto c = makeCurve(m_tool == ToolId::Spline ? CurveKind::Spline : CurveKind::Polyline, m_points);
    m_points.clear();
    return c;
}

void SketchController::drawOverlay(ImDrawList* dl, const OrbitCamera& cam, ImVec2 vp, ToolId tool) const {
    if (!m_hasHover) return;
    const PlaneFrame f = m_frame;
    auto scr = [&](Vec2 q) { return projectToScreen(cam, f.toWorld(q), vp); };
    const auto h = scr(m_hover);
    if (!h) return;

    // Rubber-band preview of the curve being drawn.
    char label[64] = "";
    if (!m_points.empty()) {
        const Vec2 a = m_points[0];
        switch (tool) {
        case ToolId::Line:
            if (auto s = scr(a)) dl->AddLine(*s, *h, kPreview, 1.5f);
            std::snprintf(label, sizeof(label), "L %s", lengthText(dist(a, m_hover)).c_str());
            break;
        case ToolId::Polyline: {
            for (std::size_t i = 1; i < m_points.size(); ++i) {
                auto s0 = scr(m_points[i - 1]), s1 = scr(m_points[i]);
                if (s0 && s1) dl->AddLine(*s0, *s1, kPreview, 1.5f);
            }
            if (auto s = scr(m_points.back())) dl->AddLine(*s, *h, kPreview, 1.0f);
            std::snprintf(label, sizeof(label), "L %s", lengthText(dist(m_points.back(), m_hover)).c_str());
            break;
        }
        case ToolId::Rectangle: {
            const Vec2 b = m_hover;
            const Vec2 c[4] = {a, {b.x, a.y}, b, {a.x, b.y}};
            for (int i = 0; i < 4; ++i) {
                auto s0 = scr(c[i]), s1 = scr(c[(i + 1) % 4]);
                if (s0 && s1) dl->AddLine(*s0, *s1, kPreview, 1.5f);
            }
            std::snprintf(label, sizeof(label), "%s", lengthsText({std::fabs(b.x - a.x), std::fabs(b.y - a.y)}, " x ").c_str());
            break;
        }
        case ToolId::Circle: {
            const float r = dist(a, m_hover);
            ImVec2 prev{};
            for (int i = 0; i <= 64; ++i) {
                const float t = 2.0f * kPi * static_cast<float>(i) / 64.0f;
                auto s = scr({a.x + r * std::cos(t), a.y + r * std::sin(t)});
                if (s && i > 0) dl->AddLine(prev, *s, kPreview, 1.5f);
                if (s) prev = *s;
            }
            if (auto s = scr(a)) dl->AddLine(*s, *h, IM_COL32(255, 190, 90, 140), 1.0f);
            std::snprintf(label, sizeof(label), "R %s", lengthText(r).c_str());
            break;
        }
        default:
            if (const auto c = pending(tool, m_hover)) { // the curve as the next click would make it
                const std::vector<Vec2> strip = c->planeStrip(0.05f);
                for (std::size_t i = 1; i < strip.size(); ++i) {
                    auto s0 = scr(strip[i - 1]), s1 = scr(strip[i]);
                    if (s0 && s1) dl->AddLine(*s0, *s1, kPreview, 1.5f);
                }
                const std::string m = c->measure();
                std::snprintf(label, sizeof(label), "%s", m.c_str());
            } else if (auto s = scr(m_points.back())) dl->AddLine(*s, *h, kPreview, 1.0f);
            break;
        }
        for (Vec2 q : m_points)
            if (auto s = scr(q)) dl->AddCircleFilled(*s, 3.0f, kPreview);
    }

    // Snap marker: square on key points, small cross on the grid.
    if (m_snappedToPoint) dl->AddRect({h->x - 5, h->y - 5}, {h->x + 5, h->y + 5}, kSnap, 0.0f, 0, 1.5f);
    else dl->AddCircle(*h, 3.5f, kSnap, 12, 1.2f);

    // Coordinates / live dimension next to the cursor.
    char coords[64];
    std::snprintf(coords, sizeof(coords), "%s", lengthsText({m_hover.x, m_hover.y}, ", ").c_str());
    const char* text = label[0] ? label : coords;
    const ImVec2 ts = ImGui::CalcTextSize(text);
    const ImVec2 at{h->x + 14, h->y + 12};
    dl->AddRectFilled({at.x - 4, at.y - 2}, {at.x + ts.x + 4, at.y + ts.y + 2}, IM_COL32(20, 22, 28, 220), 3.0f);
    dl->AddText(at, IM_COL32(240, 240, 240, 255), text);
}

} // namespace sicdm

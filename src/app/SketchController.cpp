#include "app/SketchController.hpp"

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

bool SketchController::hover(const Scene& scene, const OrbitCamera& cam, SketchPlane plane, ImVec2 mouse,
                             ImVec2 vp, const SnapOptions& snap, bool constrain) {
    if (plane != m_plane) {
        m_points.clear(); // a curve never spans two planes
        m_plane = plane;
    }
    const PlaneFrame f = planeFrame(plane);
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
            if (c.visible && c.plane == plane)
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
    c.pts = std::move(pts);
    c.radius = radius;
    return c;
}

std::optional<SketchCurve> SketchController::click(ToolId tool, bool doubleClick) {
    if (!m_hasHover) return std::nullopt;
    const Vec2 p = m_hover;
    switch (tool) {
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

std::optional<SketchCurve> SketchController::typedPoint(ToolId tool, SketchPlane plane, Vec2 p) {
    if (plane != m_plane) {
        m_points.clear();
        m_plane = plane;
    }
    m_hover = p;
    m_hasHover = true;
    // Typing the first point of a polyline again closes it, like clicking it.
    m_snappedToPoint = tool == ToolId::Polyline && m_points.size() >= 2 && dist(p, m_points[0]) < 1e-6f;
    return click(tool, false);
}

std::optional<SketchCurve> SketchController::finish() {
    if (m_points.size() < 2) {
        m_points.clear();
        return std::nullopt;
    }
    auto c = makeCurve(CurveKind::Polyline, m_points);
    m_points.clear();
    return c;
}

void SketchController::drawOverlay(ImDrawList* dl, const OrbitCamera& cam, ImVec2 vp, ToolId tool) const {
    if (!m_hasHover) return;
    const PlaneFrame f = planeFrame(m_plane);
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
            std::snprintf(label, sizeof(label), "L %.2f", dist(a, m_hover));
            break;
        case ToolId::Polyline: {
            for (std::size_t i = 1; i < m_points.size(); ++i) {
                auto s0 = scr(m_points[i - 1]), s1 = scr(m_points[i]);
                if (s0 && s1) dl->AddLine(*s0, *s1, kPreview, 1.5f);
            }
            if (auto s = scr(m_points.back())) dl->AddLine(*s, *h, kPreview, 1.0f);
            std::snprintf(label, sizeof(label), "L %.2f", dist(m_points.back(), m_hover));
            break;
        }
        case ToolId::Rectangle: {
            const Vec2 b = m_hover;
            const Vec2 c[4] = {a, {b.x, a.y}, b, {a.x, b.y}};
            for (int i = 0; i < 4; ++i) {
                auto s0 = scr(c[i]), s1 = scr(c[(i + 1) % 4]);
                if (s0 && s1) dl->AddLine(*s0, *s1, kPreview, 1.5f);
            }
            std::snprintf(label, sizeof(label), "%.2f x %.2f", std::fabs(b.x - a.x), std::fabs(b.y - a.y));
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
            std::snprintf(label, sizeof(label), "R %.2f", r);
            break;
        }
        default: break;
        }
        for (Vec2 q : m_points)
            if (auto s = scr(q)) dl->AddCircleFilled(*s, 3.0f, kPreview);
    }

    // Snap marker: square on key points, small cross on the grid.
    if (m_snappedToPoint) dl->AddRect({h->x - 5, h->y - 5}, {h->x + 5, h->y + 5}, kSnap, 0.0f, 0, 1.5f);
    else dl->AddCircle(*h, 3.5f, kSnap, 12, 1.2f);

    // Coordinates / live dimension next to the cursor.
    char coords[64];
    auto clean = [](float v) { return std::fabs(v) < 0.005f ? 0.0f : v; };
    std::snprintf(coords, sizeof(coords), "%.2f, %.2f", clean(m_hover.x), clean(m_hover.y));
    const char* text = label[0] ? label : coords;
    const ImVec2 ts = ImGui::CalcTextSize(text);
    const ImVec2 at{h->x + 14, h->y + 12};
    dl->AddRectFilled({at.x - 4, at.y - 2}, {at.x + ts.x + 4, at.y + ts.y + 2}, IM_COL32(20, 22, 28, 220), 3.0f);
    dl->AddText(at, IM_COL32(240, 240, 240, 255), text);
}

} // namespace sicdm

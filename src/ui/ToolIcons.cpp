#include "ui/ToolIcons.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>

namespace sicdm {

namespace {

constexpr float kTau = 6.28318530718f;

// Icon canvas: x and y in [-1, 1], y pointing down.
class Pen {
public:
    Pen(ImDrawList* dl, ImVec2 c, float size, ImU32 col)
        : m_dl(dl), m_c(c), m_s(size * 0.5f), m_col(col), m_w(std::max(1.0f, size / 14.0f)) {}

    ImVec2 p(float x, float y) const { return {m_c.x + x * m_s, m_c.y + y * m_s}; }
    float r(float v) const { return v * m_s; }
    ImU32 fillCol() const { return (m_col & 0x00FFFFFF) | 0x40000000; }

    void line(float x0, float y0, float x1, float y1) const { m_dl->AddLine(p(x0, y0), p(x1, y1), m_col, m_w); }
    void poly(std::initializer_list<ImVec2> pts, bool closed, bool fill = false) const {
        ImVec2 buf[16];
        int n = 0;
        for (const ImVec2& q : pts) buf[n++] = p(q.x, q.y);
        if (fill) m_dl->AddConvexPolyFilled(buf, n, fillCol());
        m_dl->AddPolyline(buf, n, m_col, closed ? ImDrawFlags_Closed : ImDrawFlags_None, m_w);
    }
    void circle(float x, float y, float rad, bool fill = false) const {
        if (fill) m_dl->AddCircleFilled(p(x, y), r(rad), fillCol(), 24);
        m_dl->AddCircle(p(x, y), r(rad), m_col, 24, m_w);
    }
    void dot(float x, float y, float rad = 0.12f) const { m_dl->AddCircleFilled(p(x, y), r(rad), m_col, 12); }
    void arc(float x, float y, float rad, float a0, float a1) const {
        m_dl->PathArcTo(p(x, y), r(rad), a0, a1, 20);
        m_dl->PathStroke(m_col, ImDrawFlags_None, m_w);
    }
    void ellipse(float x, float y, float rx, float ry, bool fill = false) const {
        if (fill) m_dl->AddEllipseFilled(p(x, y), {r(rx), r(ry)}, fillCol(), 0.0f, 24);
        m_dl->AddEllipse(p(x, y), {r(rx), r(ry)}, m_col, 0.0f, 24, m_w);
    }
    void bezier(float x0, float y0, float x1, float y1, float x2, float y2, float x3, float y3) const {
        m_dl->AddBezierCubic(p(x0, y0), p(x1, y1), p(x2, y2), p(x3, y3), m_col, m_w, 20);
    }
    void dashed(float x0, float y0, float x1, float y1, int dashes = 4) const {
        for (int i = 0; i < dashes; ++i) {
            const float t0 = static_cast<float>(i) / static_cast<float>(dashes);
            const float t1 = t0 + 0.55f / static_cast<float>(dashes);
            line(x0 + (x1 - x0) * t0, y0 + (y1 - y0) * t0, x0 + (x1 - x0) * t1, y0 + (y1 - y0) * t1);
        }
    }
    void dashedRect(float x0, float y0, float x1, float y1) const {
        dashed(x0, y0, x1, y0);
        dashed(x1, y0, x1, y1);
        dashed(x1, y1, x0, y1);
        dashed(x0, y1, x0, y0);
    }

private:
    ImDrawList* m_dl;
    ImVec2 m_c;
    float m_s;
    ImU32 m_col;
    float m_w;
};

// Shared pieces --------------------------------------------------------------

void isoBox(const Pen& g, float w, float h, float d) {
    // Front face + top + right side of an isometric box centred on the icon.
    const float x0 = -w * 0.5f - d * 0.25f, y0 = -h * 0.5f + d * 0.25f;
    g.poly({{x0, y0}, {x0 + w, y0}, {x0 + w, y0 + h}, {x0, y0 + h}}, true, true);
    g.poly({{x0, y0}, {x0 + d * 0.5f, y0 - d * 0.5f}, {x0 + w + d * 0.5f, y0 - d * 0.5f}, {x0 + w, y0}}, true);
    g.poly({{x0 + w, y0}, {x0 + w + d * 0.5f, y0 - d * 0.5f}, {x0 + w + d * 0.5f, y0 + h - d * 0.5f}, {x0 + w, y0 + h}},
           true);
}

void cylinderShape(const Pen& g, float rx, float top, float bottom, float topRx) {
    const float ry = rx * 0.35f, tRy = topRx * 0.35f;
    g.ellipse(0, top, topRx, tRy, true);
    g.line(-topRx, top, -rx, bottom);
    g.line(topRx, top, rx, bottom);
    // Lower half of the base ellipse.
    const int n = 12;
    for (int i = 0; i < n; ++i) {
        const float a0 = kTau * 0.5f * static_cast<float>(i) / n, a1 = kTau * 0.5f * static_cast<float>(i + 1) / n;
        g.line(std::cos(a0) * rx, bottom + std::sin(a0) * ry, std::cos(a1) * rx, bottom + std::sin(a1) * ry);
    }
}

void arrowHead(const Pen& g, float x, float y, float dx, float dy, float len = 0.3f) {
    const float l = std::sqrt(dx * dx + dy * dy);
    dx /= l;
    dy /= l;
    const float px = -dy, py = dx;
    g.poly({{x, y}, {x - dx * len + px * len * 0.6f, y - dy * len + py * len * 0.6f},
            {x - dx * len - px * len * 0.6f, y - dy * len - py * len * 0.6f}},
           true, true);
}

} // namespace

void drawToolIcon(ImDrawList* dl, ToolId id, ImVec2 center, float size, ImU32 col) {
    const Pen g(dl, center, size, col);
    switch (id) {
    // ---- Navigate -----------------------------------------------------------
    case ToolId::Move:
        g.line(-0.85f, 0, 0.85f, 0);
        g.line(0, -0.85f, 0, 0.85f);
        arrowHead(g, 0.9f, 0, 1, 0);
        arrowHead(g, -0.9f, 0, -1, 0);
        arrowHead(g, 0, 0.9f, 0, 1);
        arrowHead(g, 0, -0.9f, 0, -1);
        break;
    case ToolId::Select:
        g.poly({{-0.5f, -0.85f}, {-0.5f, 0.6f}, {-0.15f, 0.25f}, {0.15f, 0.85f}, {0.35f, 0.75f}, {0.05f, 0.15f},
                {0.5f, 0.15f}},
               true, true);
        break;
    case ToolId::BoxSelect: g.dashedRect(-0.8f, -0.65f, 0.8f, 0.65f); break;
    case ToolId::LassoSelect:
        g.ellipse(0.05f, -0.2f, 0.75f, 0.5f);
        g.bezier(-0.45f, 0.15f, -0.6f, 0.5f, -0.2f, 0.6f, -0.35f, 0.9f);
        break;
    case ToolId::Orbit:
        g.ellipse(0, 0, 0.85f, 0.35f);
        g.circle(0, 0, 0.3f, true);
        arrowHead(g, 0.25f, 0.35f, 1, 0);
        break;
    case ToolId::Pan:
        // Open hand: palm + four fingers + thumb.
        g.poly({{-0.45f, 0.0f}, {0.5f, 0.0f}, {0.45f, 0.55f}, {0.15f, 0.85f}, {-0.25f, 0.85f}, {-0.5f, 0.5f}}, true,
               true);
        for (int i = 0; i < 4; ++i) {
            const float x = -0.36f + 0.27f * static_cast<float>(i);
            g.line(x, 0.0f, x, i == 0 || i == 3 ? -0.5f : -0.75f);
        }
        g.line(0.5f, 0.15f, 0.8f, -0.2f);
        break;
    case ToolId::Zoom:
        g.circle(-0.15f, -0.15f, 0.55f, true);
        g.line(0.25f, 0.25f, 0.8f, 0.8f);
        g.line(0.3f, 0.3f, 0.75f, 0.75f);
        break;

    // ---- 2D sketch ----------------------------------------------------------
    case ToolId::Point:
        g.dot(0, 0, 0.18f);
        g.line(-0.7f, 0, -0.35f, 0);
        g.line(0.35f, 0, 0.7f, 0);
        g.line(0, -0.7f, 0, -0.35f);
        g.line(0, 0.35f, 0, 0.7f);
        break;
    case ToolId::Line:
        g.line(-0.7f, 0.7f, 0.7f, -0.7f);
        g.dot(-0.7f, 0.7f);
        g.dot(0.7f, -0.7f);
        break;
    case ToolId::Polyline:
        g.poly({{-0.85f, 0.6f}, {-0.35f, -0.4f}, {0.15f, 0.4f}, {0.8f, -0.6f}}, false);
        g.dot(-0.85f, 0.6f);
        g.dot(-0.35f, -0.4f);
        g.dot(0.15f, 0.4f);
        g.dot(0.8f, -0.6f);
        break;
    case ToolId::Arc:
        g.arc(0, 0.35f, 0.75f, kTau * 0.5f, kTau);
        g.dot(-0.75f, 0.35f);
        g.dot(0.75f, 0.35f);
        g.dot(0, 0.35f, 0.08f);
        break;
    case ToolId::Conic: {
        // Parabola y = x^2 opening upward.
        for (int i = 0; i < 12; ++i) {
            const float x0 = -0.8f + 1.6f * static_cast<float>(i) / 12.0f;
            const float x1 = -0.8f + 1.6f * static_cast<float>(i + 1) / 12.0f;
            g.line(x0, 0.6f - 1.9f * (0.64f - x0 * x0) * 0.75f, x1, 0.6f - 1.9f * (0.64f - x1 * x1) * 0.75f);
        }
        g.dot(0, 0.45f, 0.08f);
        break;
    }
    case ToolId::Circle:
        g.circle(0, 0, 0.75f);
        g.dot(0, 0, 0.08f);
        break;
    case ToolId::Ellipse:
        g.ellipse(0, 0, 0.85f, 0.5f);
        g.dot(0, 0, 0.08f);
        break;
    case ToolId::Rectangle:
        g.poly({{-0.8f, -0.55f}, {0.8f, -0.55f}, {0.8f, 0.55f}, {-0.8f, 0.55f}}, true);
        break;
    case ToolId::Slot:
        g.arc(-0.45f, 0, 0.4f, kTau * 0.25f, kTau * 0.75f);
        g.arc(0.45f, 0, 0.4f, -kTau * 0.25f, kTau * 0.25f);
        g.line(-0.45f, -0.4f, 0.45f, -0.4f);
        g.line(-0.45f, 0.4f, 0.45f, 0.4f);
        break;
    case ToolId::Polygon:
        g.poly({{0.8f, 0}, {0.4f, -0.7f}, {-0.4f, -0.7f}, {-0.8f, 0}, {-0.4f, 0.7f}, {0.4f, 0.7f}}, true);
        break;
    case ToolId::Spline:
        g.bezier(-0.85f, 0.5f, -0.3f, -1.1f, 0.3f, 1.1f, 0.85f, -0.5f);
        g.dot(-0.85f, 0.5f);
        g.dot(0.85f, -0.5f);
        g.dashed(-0.85f, 0.5f, -0.3f, -1.1f, 3);
        break;
    case ToolId::SketchTrim: // a crossing, one arm cut away (dashed)
        g.line(-0.85f, -0.6f, 0.85f, -0.6f);
        g.line(0.0f, -0.95f, 0.0f, -0.6f);
        g.dashed(0.0f, -0.6f, 0.0f, 0.9f, 4);
        g.line(0.35f, 0.25f, -0.35f, 0.75f);
        g.line(-0.35f, 0.25f, 0.35f, 0.75f);
        break;
    case ToolId::SketchExtend: // a line carried on (dashed) to a wall
        g.line(0.8f, -0.9f, 0.8f, 0.9f);
        g.line(-0.9f, 0.0f, -0.1f, 0.0f);
        g.dashed(-0.1f, 0.0f, 0.8f, 0.0f, 3);
        g.dot(-0.9f, 0.0f);
        break;
    case ToolId::SketchOffset: // a curve and its copy beside it
        g.bezier(-0.85f, 0.2f, -0.3f, -0.7f, 0.3f, 0.7f, 0.85f, -0.2f);
        g.bezier(-0.85f, 0.65f, -0.3f, -0.25f, 0.3f, 1.15f, 0.85f, 0.25f);
        break;

    // ---- 3D solids ----------------------------------------------------------
    case ToolId::Block: isoBox(g, 1.1f, 1.0f, 0.7f); break;
    case ToolId::Wedge:
        g.poly({{-0.8f, 0.6f}, {0.5f, 0.6f}, {-0.8f, -0.5f}}, true, true);
        g.poly({{-0.8f, -0.5f}, {-0.45f, -0.85f}, {0.85f, 0.25f}, {0.5f, 0.6f}}, true);
        break;
    case ToolId::Pyramid:
        g.poly({{0, -0.85f}, {-0.8f, 0.5f}, {0.15f, 0.75f}}, true, true);
        g.poly({{0, -0.85f}, {0.15f, 0.75f}, {0.8f, 0.4f}}, true);
        break;
    case ToolId::Prism:
        g.poly({{-0.75f, -0.35f}, {-0.35f, -0.6f}, {0.35f, -0.6f}, {0.75f, -0.35f}, {0.35f, -0.1f}, {-0.35f, -0.1f}},
               true, true);
        g.line(-0.75f, -0.35f, -0.75f, 0.55f);
        g.line(-0.35f, -0.1f, -0.35f, 0.8f);
        g.line(0.35f, -0.1f, 0.35f, 0.8f);
        g.line(0.75f, -0.35f, 0.75f, 0.55f);
        g.poly({{-0.75f, 0.55f}, {-0.35f, 0.8f}, {0.35f, 0.8f}, {0.75f, 0.55f}}, false);
        break;
    case ToolId::Cylinder: cylinderShape(g, 0.6f, -0.6f, 0.6f, 0.6f); break;
    case ToolId::Cone:
        g.poly({{0, -0.85f}, {-0.65f, 0.55f}, {0.65f, 0.55f}}, false, false);
        g.ellipse(0, 0.55f, 0.65f, 0.22f, true);
        break;
    case ToolId::Sphere:
        g.circle(0, 0, 0.8f, true);
        g.ellipse(0, 0, 0.8f, 0.28f);
        g.ellipse(0, 0, 0.3f, 0.8f);
        break;
    case ToolId::Torus:
        g.ellipse(0, 0, 0.85f, 0.5f, true);
        g.ellipse(0, -0.02f, 0.35f, 0.16f);
        break;

    // ---- Features -----------------------------------------------------------
    case ToolId::Extrude:
        g.poly({{-0.75f, 0.35f}, {0.25f, 0.35f}, {0.75f, 0.0f}, {-0.25f, 0.0f}}, true, true);
        g.line(0, 0.15f, 0, -0.75f);
        arrowHead(g, 0, -0.85f, 0, -1);
        break;
    case ToolId::Revolve:
        g.dashed(0, -0.9f, 0, 0.9f, 5);
        g.poly({{0.15f, -0.6f}, {0.55f, -0.6f}, {0.55f, 0.6f}, {0.15f, 0.6f}}, true, true);
        g.arc(0, 0, 0.8f, kTau * 0.55f, kTau * 0.95f);
        arrowHead(g, 0.65f, -0.47f, 0.7f, 0.7f, 0.25f);
        break;
    case ToolId::Sweep:
        g.bezier(-0.8f, 0.7f, -0.8f, -0.3f, 0.2f, 0.3f, 0.75f, -0.6f);
        g.circle(-0.8f, 0.6f, 0.22f, true);
        g.circle(0.72f, -0.55f, 0.22f, true);
        break;
    case ToolId::Loft:
        g.poly({{-0.75f, 0.4f}, {0.0f, 0.4f}, {0.35f, 0.75f}, {-0.4f, 0.75f}}, true, true);
        g.circle(0.25f, -0.5f, 0.35f, true);
        g.line(-0.75f, 0.4f, -0.1f, -0.5f);
        g.line(0.35f, 0.75f, 0.6f, -0.5f);
        break;
    case ToolId::Helix:
        for (int i = 0; i < 4; ++i) {
            const float y = -0.75f + 0.45f * static_cast<float>(i);
            g.bezier(-0.6f, y, -0.6f, y + 0.25f, 0.6f, y + 0.05f, 0.6f, y + 0.3f);
        }
        break;
    case ToolId::Rib:
        g.poly({{-0.85f, 0.6f}, {0.85f, 0.6f}, {0.85f, 0.75f}, {-0.85f, 0.75f}}, true, true);
        g.poly({{-0.85f, -0.75f}, {-0.7f, -0.75f}, {-0.7f, 0.6f}, {-0.85f, 0.6f}}, true, true);
        g.poly({{-0.7f, -0.45f}, {0.55f, 0.6f}, {-0.7f, 0.6f}}, true, true);
        break;

    // ---- Surfaces -----------------------------------------------------------
    case ToolId::PlaneSurface:
        g.poly({{-0.85f, 0.35f}, {-0.2f, -0.5f}, {0.85f, -0.35f}, {0.2f, 0.5f}}, true, true);
        break;
    case ToolId::TabulatedCylinder:
        g.bezier(-0.8f, 0.3f, -0.4f, -0.2f, 0.0f, 0.6f, 0.4f, 0.2f);
        g.bezier(-0.4f, -0.4f, 0.0f, -0.9f, 0.4f, -0.1f, 0.8f, -0.5f);
        g.line(-0.8f, 0.3f, -0.4f, -0.4f);
        g.line(0.4f, 0.2f, 0.8f, -0.5f);
        arrowHead(g, 0.8f, -0.5f, 0.5f, -0.7f, 0.22f);
        break;
    case ToolId::RuledSurface:
        g.bezier(-0.8f, -0.6f, -0.3f, -0.9f, 0.3f, -0.3f, 0.8f, -0.6f);
        g.line(-0.8f, 0.6f, 0.8f, 0.6f);
        for (int i = 0; i <= 4; ++i) {
            const float t = static_cast<float>(i) / 4.0f;
            g.line(-0.8f + 1.6f * t, -0.6f - 0.15f * std::sin(t * kTau * 0.5f), -0.8f + 1.6f * t, 0.6f);
        }
        break;
    case ToolId::BSplinePatch:
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) g.dot(-0.7f + 0.7f * static_cast<float>(i), -0.7f + 0.7f * static_cast<float>(j), 0.09f);
        g.dashedRect(-0.7f, -0.7f, 0.7f, 0.7f);
        g.bezier(-0.7f, 0.0f, -0.3f, -0.5f, 0.3f, 0.5f, 0.7f, 0.0f);
        break;
    case ToolId::CoonsPatch:
        g.bezier(-0.75f, -0.6f, -0.3f, -0.85f, 0.3f, -0.35f, 0.75f, -0.6f);
        g.bezier(-0.75f, 0.6f, -0.3f, 0.35f, 0.3f, 0.85f, 0.75f, 0.6f);
        g.bezier(-0.75f, -0.6f, -0.95f, -0.2f, -0.55f, 0.2f, -0.75f, 0.6f);
        g.bezier(0.75f, -0.6f, 0.55f, -0.2f, 0.95f, 0.2f, 0.75f, 0.6f);
        break;
    case ToolId::GordonSurface:
        for (int i = 0; i < 3; ++i) {
            const float o = -0.55f + 0.55f * static_cast<float>(i);
            g.bezier(-0.8f, o, -0.3f, o - 0.25f, 0.3f, o + 0.25f, 0.8f, o);
            g.bezier(o, -0.8f, o + 0.25f, -0.3f, o - 0.25f, 0.3f, o, 0.8f);
        }
        break;

    // ---- Class-A ------------------------------------------------------------
    case ToolId::ClassANurbs:
        g.bezier(-0.85f, 0.4f, -0.4f, -0.8f, 0.4f, -0.8f, 0.85f, 0.4f);
        g.bezier(-0.85f, 0.7f, -0.4f, -0.4f, 0.4f, -0.4f, 0.85f, 0.7f);
        g.line(-0.85f, 0.4f, -0.85f, 0.7f);
        g.line(0.85f, 0.4f, 0.85f, 0.7f);
        g.bezier(-0.45f, -0.05f, -0.2f, -0.35f, 0.2f, -0.35f, 0.45f, -0.05f); // highlight line
        break;
    case ToolId::OffsetSurface:
        g.bezier(-0.85f, 0.2f, -0.4f, -0.6f, 0.4f, 0.6f, 0.85f, -0.2f);
        g.bezier(-0.85f, -0.25f, -0.4f, -1.05f, 0.4f, 0.15f, 0.85f, -0.65f);
        g.line(0, 0.05f, 0, -0.4f);
        arrowHead(g, 0, -0.45f, 0, -1, 0.2f);
        break;
    case ToolId::BlendedFillet:
        g.line(-0.75f, -0.85f, -0.75f, 0.1f);
        g.line(-0.1f, 0.75f, 0.85f, 0.75f);
        g.arc(-0.1f, 0.1f, 0.65f, kTau * 0.25f, kTau * 0.5f);
        g.dashed(-0.75f, 0.1f, -0.75f, 0.75f, 2);
        g.dashed(-0.75f, 0.75f, -0.1f, 0.75f, 2);
        break;
    case ToolId::TwistedAirfoil:
        g.bezier(-0.85f, 0.35f, -0.6f, -0.15f, 0.2f, -0.05f, 0.85f, 0.25f);
        g.bezier(-0.85f, 0.35f, -0.5f, 0.6f, 0.2f, 0.45f, 0.85f, 0.25f);
        g.bezier(-0.55f, -0.55f, -0.3f, -0.85f, 0.3f, -0.65f, 0.7f, -0.45f);
        g.dashed(-0.85f, 0.35f, -0.55f, -0.55f, 3);
        g.dashed(0.85f, 0.25f, 0.7f, -0.45f, 3);
        break;
    case ToolId::TubercleWing: {
        // Bumpy leading edge on a swept wing.
        g.line(-0.75f, 0.65f, 0.85f, 0.65f);
        g.line(0.85f, 0.65f, 0.85f, -0.05f);
        float x = -0.75f;
        for (int i = 0; i < 4; ++i) {
            g.arc(x + 0.2f, 0.45f - 0.17f * static_cast<float>(i), 0.2f, kTau * 0.5f, kTau);
            x += 0.4f;
        }
        g.line(-0.75f, 0.65f, -0.75f, 0.45f);
        break;
    }

    // ---- Generative ---------------------------------------------------------
    case ToolId::TopologyOptimized:
        g.circle(-0.6f, 0.55f, 0.22f, true);
        g.circle(0.6f, -0.55f, 0.22f, true);
        g.bezier(-0.45f, 0.4f, -0.1f, 0.1f, 0.1f, -0.1f, 0.45f, -0.4f);
        g.bezier(-0.4f, 0.65f, 0.3f, 0.6f, 0.4f, 0.1f, 0.5f, -0.35f);
        g.bezier(-0.55f, 0.3f, -0.5f, -0.2f, 0.0f, -0.5f, 0.4f, -0.6f);
        break;
    case ToolId::TpmsLattice:
        for (int i = 0; i < 3; ++i) {
            const float y = -0.6f + 0.6f * static_cast<float>(i);
            g.bezier(-0.85f, y, -0.45f, y - 0.35f, -0.05f, y + 0.35f, 0.35f, y);
            g.bezier(0.35f, y, 0.55f, y - 0.2f, 0.7f, y - 0.2f, 0.85f, y);
        }
        break;

    // ---- Mesh ---------------------------------------------------------------
    case ToolId::MeshImport:
        g.poly({{-0.8f, 0.6f}, {0.0f, -0.75f}, {0.8f, 0.6f}}, true, true);
        g.line(-0.4f, -0.07f, 0.4f, -0.07f);
        g.line(-0.4f, -0.07f, 0.0f, 0.6f);
        g.line(0.4f, -0.07f, 0.0f, 0.6f);
        break;
    case ToolId::PointCloud: {
        const float pts[][2] = {{-0.6f, -0.2f}, {-0.3f, -0.55f}, {0.1f, -0.65f}, {0.5f, -0.4f}, {0.7f, 0.0f},
                                {0.5f, 0.45f},  {0.1f, 0.65f},   {-0.35f, 0.5f}, {-0.7f, 0.2f}, {-0.1f, -0.1f},
                                {0.25f, 0.1f},  {-0.25f, 0.2f},  {0.3f, -0.3f}};
        for (const auto& q : pts) g.dot(q[0], q[1], 0.08f);
        break;
    }
    case ToolId::SubD:
        g.dashedRect(-0.8f, -0.75f, 0.8f, 0.75f);
        g.ellipse(0, 0, 0.6f, 0.52f, true);
        g.dot(-0.8f, -0.75f, 0.09f);
        g.dot(0.8f, -0.75f, 0.09f);
        g.dot(0.8f, 0.75f, 0.09f);
        g.dot(-0.8f, 0.75f, 0.09f);
        break;

    // ---- Digital sculpting ---------------------------------------------------
    // A clay surface (the baseline) with what each brush does to it.
    case ToolId::SculptDraw:
        g.line(-0.9f, 0.6f, -0.45f, 0.6f);
        g.bezier(-0.45f, 0.6f, -0.25f, 0.6f, -0.2f, -0.05f, 0.0f, -0.05f);
        g.bezier(0.0f, -0.05f, 0.2f, -0.05f, 0.25f, 0.6f, 0.45f, 0.6f);
        g.line(0.45f, 0.6f, 0.9f, 0.6f);
        g.line(0, -0.25f, 0, -0.8f);
        arrowHead(g, 0, -0.85f, 0, -1, 0.25f);
        break;
    case ToolId::SculptClay:
        g.line(-0.9f, 0.65f, 0.9f, 0.65f);
        g.poly({{-0.6f, 0.65f}, {-0.45f, 0.25f}, {0.45f, 0.25f}, {0.6f, 0.65f}}, false, true);
        g.poly({{-0.4f, 0.25f}, {-0.28f, -0.12f}, {0.28f, -0.12f}, {0.4f, 0.25f}}, false, true);
        g.line(0.55f, -0.75f, 0.55f, -0.35f);
        g.line(0.35f, -0.55f, 0.75f, -0.55f);
        break;
    case ToolId::SculptCarve:
        g.line(-0.9f, -0.05f, -0.4f, -0.05f);
        g.bezier(-0.4f, -0.05f, -0.25f, -0.05f, -0.15f, 0.6f, 0.0f, 0.6f);
        g.bezier(0.0f, 0.6f, 0.15f, 0.6f, 0.25f, -0.05f, 0.4f, -0.05f);
        g.line(0.4f, -0.05f, 0.9f, -0.05f);
        g.poly({{0.15f, -0.85f}, {0.45f, -0.85f}, {0.08f, -0.2f}}, true, true); // gouge
        break;
    case ToolId::SculptInflate:
        g.circle(0, 0, 0.38f, true);
        for (int i = 0; i < 4; ++i) {
            const float a = kTau * (0.125f + 0.25f * static_cast<float>(i));
            const float cx = std::cos(a), cy = std::sin(a);
            g.line(cx * 0.5f, cy * 0.5f, cx * 0.82f, cy * 0.82f);
            arrowHead(g, cx * 0.9f, cy * 0.9f, cx, cy, 0.2f);
        }
        break;
    case ToolId::SculptGrab:
        g.line(-0.9f, 0.65f, -0.5f, 0.65f);
        g.bezier(-0.5f, 0.65f, -0.2f, 0.65f, 0.0f, -0.35f, 0.3f, -0.35f);
        g.bezier(0.3f, -0.35f, 0.5f, -0.35f, 0.5f, 0.65f, 0.9f, 0.65f);
        g.circle(0.3f, -0.35f, 0.14f, true);
        g.dashed(-0.1f, 0.35f, 0.25f, -0.2f, 3);
        break;
    case ToolId::SculptPinch:
        g.line(-0.9f, 0.55f, -0.2f, 0.55f);
        g.line(-0.2f, 0.55f, 0, -0.1f);
        g.line(0, -0.1f, 0.2f, 0.55f);
        g.line(0.2f, 0.55f, 0.9f, 0.55f);
        g.line(-0.85f, -0.45f, -0.3f, -0.45f);
        arrowHead(g, -0.22f, -0.45f, 1, 0, 0.22f);
        g.line(0.85f, -0.45f, 0.3f, -0.45f);
        arrowHead(g, 0.22f, -0.45f, -1, 0, 0.22f);
        break;
    case ToolId::SculptCrease:
        g.line(-0.9f, -0.2f, -0.25f, -0.2f);
        g.line(-0.25f, -0.2f, 0, 0.75f);
        g.line(0, 0.75f, 0.25f, -0.2f);
        g.line(0.25f, -0.2f, 0.9f, -0.2f);
        g.dashed(0, -0.85f, 0, 0.45f, 4);
        break;
    case ToolId::SculptSmooth:
        g.bezier(-0.9f, -0.35f, -0.65f, -0.75f, -0.45f, 0.05f, -0.2f, -0.35f);
        g.bezier(-0.2f, -0.35f, 0.0f, -0.6f, 0.1f, -0.2f, 0.3f, -0.3f);
        g.line(0.3f, -0.3f, 0.9f, -0.3f);
        g.line(-0.9f, 0.45f, 0.9f, 0.45f);
        arrowHead(g, 0, 0.3f, 0, 1, 0.22f);
        g.line(0, -0.05f, 0, 0.1f);
        break;
    case ToolId::SculptFlatten:
        g.line(-0.9f, 0.65f, -0.5f, 0.65f);
        g.bezier(-0.5f, 0.65f, -0.35f, 0.65f, -0.35f, 0.05f, -0.15f, 0.05f);
        g.line(-0.15f, 0.05f, 0.15f, 0.05f);
        g.bezier(0.15f, 0.05f, 0.35f, 0.05f, 0.35f, 0.65f, 0.5f, 0.65f);
        g.line(0.5f, 0.65f, 0.9f, 0.65f);
        g.poly({{-0.55f, -0.15f}, {0.55f, -0.15f}, {0.55f, -0.4f}, {-0.55f, -0.4f}}, true, true);
        g.line(0, -0.4f, 0, -0.85f);
        break;
    case ToolId::SculptScrape:
        g.line(-0.9f, 0.65f, -0.55f, 0.65f);
        g.bezier(-0.55f, 0.65f, -0.4f, 0.65f, -0.4f, 0.05f, -0.25f, 0.05f);
        g.line(-0.25f, 0.05f, 0.25f, 0.05f);
        g.bezier(0.25f, 0.05f, 0.4f, 0.05f, 0.4f, 0.65f, 0.55f, 0.65f);
        g.line(0.55f, 0.65f, 0.9f, 0.65f);
        g.dashed(-0.25f, 0.05f, -0.1f, -0.35f, 2);
        g.dashed(0.1f, -0.35f, 0.25f, 0.05f, 2);
        g.poly({{-0.75f, -0.05f}, {0.85f, -0.05f}, {0.65f, -0.3f}}, true, true); // blade
        break;

    case ToolId::SculptClayStrips:
        g.line(-0.9f, 0.65f, 0.9f, 0.65f);
        g.poly({{-0.75f, 0.65f}, {-0.75f, 0.3f}, {-0.15f, 0.3f}, {-0.15f, 0.65f}}, false, true);
        g.poly({{0.15f, 0.65f}, {0.15f, 0.3f}, {0.75f, 0.3f}, {0.75f, 0.65f}}, false, true);
        g.poly({{-0.4f, 0.3f}, {-0.4f, -0.05f}, {0.4f, -0.05f}, {0.4f, 0.3f}}, false, true);
        g.line(-0.6f, -0.55f, 0.5f, -0.55f);
        arrowHead(g, 0.6f, -0.55f, 1, 0, 0.22f);
        break;
    case ToolId::SculptLayer:
        g.line(-0.9f, 0.65f, -0.6f, 0.65f);
        g.bezier(-0.6f, 0.65f, -0.5f, 0.65f, -0.5f, 0.1f, -0.35f, 0.1f);
        g.line(-0.35f, 0.1f, 0.35f, 0.1f);
        g.bezier(0.35f, 0.1f, 0.5f, 0.1f, 0.5f, 0.65f, 0.6f, 0.65f);
        g.line(0.6f, 0.65f, 0.9f, 0.65f);
        g.dashed(-0.35f, -0.25f, 0.35f, -0.25f, 4);
        g.line(0, -0.55f, 0, -0.35f);
        arrowHead(g, 0, -0.3f, 0, 1, 0.2f);
        break;
    case ToolId::SculptBlob:
        g.line(-0.9f, 0.65f, -0.55f, 0.65f);
        g.line(0.55f, 0.65f, 0.9f, 0.65f);
        g.bezier(-0.55f, 0.65f, -0.75f, -0.6f, 0.75f, -0.6f, 0.55f, 0.65f);
        g.dot(-0.15f, 0.05f, 0.08f);
        break;
    case ToolId::SculptSnakeHook:
        g.line(-0.9f, 0.65f, -0.35f, 0.65f);
        g.bezier(-0.35f, 0.65f, -0.2f, 0.1f, 0.1f, -0.2f, 0.45f, -0.6f);
        g.bezier(0.45f, -0.6f, 0.2f, -0.1f, 0.15f, 0.3f, 0.25f, 0.65f);
        g.line(0.25f, 0.65f, 0.9f, 0.65f);
        arrowHead(g, 0.55f, -0.72f, 0.7f, -0.7f, 0.22f);
        break;
    case ToolId::SculptNudge:
        g.line(-0.9f, 0.55f, -0.2f, 0.55f);
        g.bezier(-0.2f, 0.55f, 0.0f, 0.55f, 0.1f, 0.2f, 0.35f, 0.2f);
        g.bezier(0.35f, 0.2f, 0.55f, 0.2f, 0.55f, 0.55f, 0.9f, 0.55f);
        g.line(-0.6f, -0.35f, 0.45f, -0.35f);
        arrowHead(g, 0.55f, -0.35f, 1, 0, 0.24f);
        break;
    case ToolId::SculptFill:
        g.line(-0.9f, 0.0f, -0.55f, 0.0f);
        g.bezier(-0.55f, 0.0f, -0.4f, 0.0f, -0.4f, 0.6f, -0.2f, 0.6f);
        g.line(-0.2f, 0.6f, 0.2f, 0.6f);
        g.bezier(0.2f, 0.6f, 0.4f, 0.6f, 0.4f, 0.0f, 0.55f, 0.0f);
        g.line(0.55f, 0.0f, 0.9f, 0.0f);
        g.dashed(-0.45f, 0.0f, 0.45f, 0.0f, 4);
        g.line(0, -0.7f, 0, -0.2f);
        arrowHead(g, 0, -0.15f, 0, 1, 0.2f);
        break;
    case ToolId::SculptMask:
        g.circle(0, 0, 0.7f, false);
        g.poly({{-0.5f, 0.5f}, {0.5f, 0.5f}, {0.5f, -0.5f}, {-0.5f, -0.5f}}, true, false);
        g.line(-0.5f, -0.1f, 0.1f, 0.5f);
        g.line(-0.5f, -0.5f, 0.5f, 0.5f);
        g.line(-0.1f, -0.5f, 0.5f, 0.1f);
        break;

    // ---- Cuts & booleans ----------------------------------------------------
    case ToolId::CutHole:
        g.poly({{-0.85f, -0.2f}, {0.85f, -0.2f}, {0.85f, 0.75f}, {-0.85f, 0.75f}}, true, true);
        g.ellipse(0, -0.2f, 0.32f, 0.12f);
        g.dashed(-0.32f, -0.2f, -0.32f, 0.75f, 3);
        g.dashed(0.32f, -0.2f, 0.32f, 0.75f, 3);
        g.line(0, -0.95f, 0, -0.42f);
        arrowHead(g, 0, -0.35f, 0, 1, 0.2f);
        break;
    case ToolId::CutPocket:
        g.poly({{-0.85f, -0.6f}, {0.85f, -0.6f}, {0.85f, 0.7f}, {-0.85f, 0.7f}}, true, true);
        g.poly({{-0.45f, -0.25f}, {0.45f, -0.25f}, {0.45f, 0.35f}, {-0.45f, 0.35f}}, true);
        g.dashed(-0.45f, -0.25f, -0.3f, -0.1f, 1);
        g.dashed(0.45f, -0.25f, 0.3f, -0.1f, 1);
        g.dashed(-0.3f, -0.1f, 0.3f, -0.1f, 3);
        break;
    case ToolId::CutChannel:
        g.poly({{-0.85f, -0.6f}, {0.85f, -0.6f}, {0.85f, 0.7f}, {-0.85f, 0.7f}}, true, true);
        g.bezier(-0.85f, 0.3f, -0.3f, -0.4f, 0.3f, 0.55f, 0.85f, -0.2f);
        g.bezier(-0.85f, 0.05f, -0.3f, -0.65f, 0.3f, 0.3f, 0.85f, -0.45f);
        break;
    case ToolId::CutKnife:
        g.poly({{-0.85f, 0.65f}, {0.35f, -0.55f}, {0.55f, -0.35f}, {-0.6f, 0.75f}}, true, true); // blade
        g.line(0.35f, -0.55f, 0.75f, -0.9f);
        g.line(0.55f, -0.35f, 0.9f, -0.75f);
        g.line(0.75f, -0.9f, 0.9f, -0.75f);
        g.dashed(-0.9f, -0.1f, 0.9f, -0.1f, 5);
        break;
    case ToolId::AddBoss:
        g.poly({{-0.9f, 0.35f}, {0.9f, 0.35f}, {0.9f, 0.8f}, {-0.9f, 0.8f}}, true, true);
        g.ellipse(0, -0.55f, 0.35f, 0.13f, true);
        g.line(-0.35f, -0.55f, -0.35f, 0.35f);
        g.line(0.35f, -0.55f, 0.35f, 0.35f);
        g.line(0.75f, -0.85f, 0.75f, -0.45f);
        g.line(0.55f, -0.65f, 0.95f, -0.65f);
        break;
    case ToolId::BooleanBodies:
        g.circle(-0.28f, 0, 0.5f);
        g.circle(0.28f, 0, 0.5f, true);
        break;

    // ---- Armour & exoskeleton --------------------------------------------------
    case ToolId::ShellBody: // a hollow dome seen in section
        g.arc(0, 0.45f, 0.8f, kTau * 0.5f, kTau);
        g.arc(0, 0.45f, 0.55f, kTau * 0.5f, kTau);
        g.line(-0.8f, 0.45f, -0.55f, 0.45f);
        g.line(0.55f, 0.45f, 0.8f, 0.45f);
        break;
    case ToolId::ArmorBody: // a limb inside a plate with a gap
        g.ellipse(0, 0, 0.32f, 0.5f, true);
        g.arc(0, 0, 0.62f, kTau * 0.62f, kTau * 1.38f);
        g.arc(0, 0, 0.82f, kTau * 0.62f, kTau * 1.38f);
        break;
    case ToolId::JointBody: // two links on a pin with a swing arrow
        g.poly({{-0.85f, 0.15f}, {-0.05f, 0.15f}, {-0.05f, 0.45f}, {-0.85f, 0.45f}}, true, true);
        g.poly({{0.05f, 0.25f}, {0.7f, -0.4f}, {0.88f, -0.22f}, {0.23f, 0.43f}}, true, true);
        g.circle(0, 0.3f, 0.14f);
        g.arc(0, 0.3f, 0.62f, -kTau * 0.25f, -kTau * 0.08f);
        break;
    case ToolId::LightenBody: // a block with lattice holes
        g.poly({{-0.85f, -0.65f}, {0.85f, -0.65f}, {0.85f, 0.65f}, {-0.85f, 0.65f}}, true, true);
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 2; ++j) g.circle(-0.52f + 0.52f * static_cast<float>(i), -0.28f + 0.56f * static_cast<float>(j), 0.17f);
        break;
    case ToolId::InterferenceCheck: // two bodies touching with a warning star
        g.circle(-0.35f, 0.2f, 0.42f);
        g.circle(0.4f, 0.2f, 0.42f, true);
        g.line(0.02f, -0.75f, 0.02f, -0.35f);
        g.line(-0.18f, -0.62f, 0.22f, -0.48f);
        g.line(0.22f, -0.62f, -0.18f, -0.48f);
        break;

    case ToolId::Count: break;
    }
}

} // namespace sicdm

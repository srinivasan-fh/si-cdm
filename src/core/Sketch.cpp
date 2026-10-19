#include "core/Sketch.hpp"
#include "core/Units.hpp"

#include <cstdio>

#include <atomic>

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
    }
    return {};
}

const char* planeName(SketchPlane plane) noexcept {
    switch (plane) {
    case SketchPlane::Top: return "Top (XY)";
    case SketchPlane::Front: return "Front (XZ)";
    case SketchPlane::Right: return "Right (YZ)";
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
    }
    return "";
}

std::vector<Vec3> SketchCurve::worldStrip(int circleSegments) const {
    const PlaneFrame f = planeFrame(plane);
    std::vector<Vec3> out;
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
    }
    return out;
}

std::vector<Vec2> SketchCurve::keyPoints() const {
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
        double total = 0.0;
        for (std::size_t i = 1; i < pts.size(); ++i) total += len(pts[i - 1], pts[i]);
        return "L " + lengthText(total) + " (" + std::to_string(pts.empty() ? 0 : pts.size() - 1) + " seg)";
    }
    case CurveKind::Rectangle:
        if (pts.size() >= 2) return lengthsText({std::fabs(pts[1].x - pts[0].x), std::fabs(pts[1].y - pts[0].y)}, " x ");
        break;
    case CurveKind::Circle: return "R " + lengthText(radius);
    }
    return "";
}

SketchCurve& Sketch::add(SketchCurve c) {
    // Numbering continues past loaded curves so new names stay unique.
    ++m_serial;
    if (c.name.empty()) c.name = std::string(curveKindName(c.kind)) + " " + std::to_string(m_serial);
    m_revision = nextRevision();
    return m_curves.emplace_back(std::move(c));
}

void Sketch::remove(std::size_t index) {
    if (index >= m_curves.size()) return;
    m_curves.erase(m_curves.begin() + static_cast<std::ptrdiff_t>(index));
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

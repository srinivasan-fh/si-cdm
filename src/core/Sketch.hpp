#pragma once

#include "core/Math.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace sicdm {

// Process-wide, strictly increasing change stamp. Scenes, sketches and images
// take a fresh one on every change, so a stamp never repeats even when one
// document replaces another (GPU caches compare stamps).
std::uint64_t nextRevision() noexcept;

// Principal sketch planes. Each maps to the matching 2D drawing view.
enum class SketchPlane { Top, Front, Right };

// Orthonormal frame of a sketch plane: u to the right, v up as seen in its view.
struct PlaneFrame {
    Vec3 origin;
    Vec3 u, v, normal;

    Vec3 toWorld(const Vec2& p) const noexcept { return origin + u * p.x + v * p.y; }
    Vec2 toPlane(const Vec3& w) const noexcept { return {dot(w - origin, u), dot(w - origin, v)}; }
};

PlaneFrame planeFrame(SketchPlane plane) noexcept;
const char* planeName(SketchPlane plane) noexcept;

enum class CurveKind { Point, Line, Polyline, Rectangle, Circle };

// One sketch entity, stored in its plane's 2D coordinates (mm).
//   Point:     pts[0]
//   Line:      pts[0] -> pts[1]
//   Polyline:  pts[0..n], open
//   Rectangle: opposite corners pts[0], pts[1]
//   Circle:    centre pts[0], radius
struct SketchCurve {
    CurveKind kind = CurveKind::Line;
    SketchPlane plane = SketchPlane::Top;
    std::vector<Vec2> pts;
    float radius = 0.0f;
    std::string name;
    bool visible = true;

    // Line-strip vertices in world space (closed shapes repeat the first point).
    std::vector<Vec3> worldStrip(int circleSegments = 96) const;

    // Snap targets in plane coordinates: endpoints, corners, centres.
    std::vector<Vec2> keyPoints() const;

    // Short human-readable size, e.g. "L 42.0" or "R 15.0".
    std::string measure() const;
};

class Sketch {
public:
    std::vector<SketchCurve>& curves() noexcept { return m_curves; }
    const std::vector<SketchCurve>& curves() const noexcept { return m_curves; }

    SketchCurve& add(SketchCurve c);
    void remove(std::size_t index);

    // Bumped on add/remove/visibility edits that change GPU buffers.
    std::uint64_t revision() const noexcept { return m_revision; }
    void touch() noexcept { m_revision = nextRevision(); }

    AABB bounds() const;

private:
    std::vector<SketchCurve> m_curves;
    std::uint64_t m_revision = nextRevision();
    int m_serial = 0;
};

const char* curveKindName(CurveKind k) noexcept;

} // namespace sicdm

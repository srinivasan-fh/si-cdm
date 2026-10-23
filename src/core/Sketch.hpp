#pragma once

#include "core/Constraints.hpp"
#include "core/Math.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
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

enum class CurveKind { Point, Line, Polyline, Rectangle, Circle, Arc, Spline, Ellipse, Slot };

// One sketch entity, stored in its plane's 2D coordinates (mm).
//   Point:     pts[0]
//   Line:      pts[0] -> pts[1]
//   Polyline:  pts[0..n], open (closed when the last point is the first)
//   Rectangle: opposite corners pts[0], pts[1]
//   Circle:    centre pts[0], radius
//   Arc:       centre pts[0], start pts[1], end pts[2] (counter-clockwise from start to
//              end), radius
//   Spline:    the points it passes through, pts[0..n] (a smooth curve through them;
//              `closed` joins the last back to the first)
//   Ellipse:   centre pts[0], the end of its major axis pts[1], minor radius `radius`
//   Slot:      the centres of its round ends pts[0], pts[1], radius (half its width)
struct SketchCurve {
    CurveKind kind = CurveKind::Line;
    SketchPlane plane = SketchPlane::Top;
    std::vector<Vec2> pts;
    float radius = 0.0f;
    std::string name;
    bool visible = true;
    bool closed = false;       // Spline: a closed loop
    bool construction = false; // a reference curve: drawn dashed, not part of profiles

    // Line-strip vertices in world space (closed shapes repeat the first point).
    std::vector<Vec3> worldStrip(int circleSegments = 96) const;
    // The same in plane coordinates, round parts sampled to `chord` mm (closed shapes
    // repeat the first point).
    std::vector<Vec2> planeStrip(float chord = 0.05f) const;
    bool isClosed() const; // Rectangle, Circle, Ellipse, Slot, a closed Polyline or Spline
    // Arcs: the start and end angles (radians, end > start, counter-clockwise).
    void arcAngles(float& a0, float& a1) const;

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

    // Constraints and dimensions on the curves (see Constraints.hpp).
    std::vector<SketchConstraint>& constraints() noexcept { return m_constraints; }
    const std::vector<SketchConstraint>& constraints() const noexcept { return m_constraints; }
    // The next constraint name: "D1", "D2" for dimensions, "C1"... for the others.
    std::string nextConstraintName(bool dimension) { return (dimension ? "D" : "C") + std::to_string(++m_constraintSerial); }
    void noteConstraintName(const std::string& name) {
        if (name.size() > 1) m_constraintSerial = std::max(m_constraintSerial, std::atoi(name.c_str() + 1));
    }

private:
    std::vector<SketchCurve> m_curves;
    std::vector<SketchConstraint> m_constraints;
    int m_constraintSerial = 0;
    std::uint64_t m_revision = nextRevision();
    int m_serial = 0;
};

const char* curveKindName(CurveKind k) noexcept;

// A point on the smooth curve through `pts` (centripetal Catmull-Rom: no loops or cusps
// between close points), at t in [0, 1] along span `span` (pts[span] to pts[span + 1];
// a closed curve's last span runs back to pts[0]).
Vec2 splinePoint(const std::vector<Vec2>& pts, bool closed, std::size_t span, float t);
// The arc through three points (start, a point on it, end): its centre and radius, and
// whether it runs counter-clockwise from start to end. False when they are in line.
bool arcThrough(Vec2 a, Vec2 m, Vec2 b, Vec2& centre, float& radius, bool& ccw);

} // namespace sicdm

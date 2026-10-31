#pragma once

// Sketch shapes made from a few points: the rectangle by its centre or by three corners,
// the circle through three points or touching two lines, the arc carrying on a curve's
// direction, the elliptical arc, the conic and text. Each gives the curve in plane
// coordinates (its plane is set by whoever adds it), or nothing with a reason.

#include "core/Sketch.hpp"
#include "core/TextOutline.hpp"

#include <optional>
#include <string>
#include <vector>

namespace sicdm {

// A rectangle centred on `centre` with a corner at `corner` (sides along the plane's axes).
std::optional<SketchCurve> centreRectangle(Vec2 centre, Vec2 corner, std::string* why = nullptr);
// A rectangle at any angle: the first side from a to b, the opposite side through c's
// distance from it (on c's side). A closed polyline.
std::optional<SketchCurve> rectangle3Points(Vec2 a, Vec2 b, Vec2 c, std::string* why = nullptr);
// The circle through three points.
std::optional<SketchCurve> circle3Points(Vec2 a, Vec2 b, Vec2 c, std::string* why = nullptr);
// A circle of radius r touching two lines (each p0 -> p1, taken as endless); of the four,
// the one whose centre is nearest `near`.
std::optional<SketchCurve> circleTangentToLines(Vec2 a0, Vec2 a1, Vec2 b0, Vec2 b1, float r, Vec2 near, std::string* why = nullptr);
// An arc from `start`, leaving it along `direction` (the curve it carries on), to `end`.
std::optional<SketchCurve> tangentArc(Vec2 start, Vec2 direction, Vec2 end, std::string* why = nullptr);
// Where a sketch curve ends (or starts) and the way it is going there: for carrying it on.
bool curveEndDirection(const SketchCurve& c, bool atStart, Vec2& point, Vec2& direction);
// Part of an ellipse (centre, major axis end, minor radius), counter-clockwise from the
// point on it nearest `from` (by parameter angle) to that nearest `to`.
std::optional<SketchCurve> ellipticalArc(Vec2 centre, Vec2 majorEnd, float minorRadius, Vec2 from, Vec2 to, std::string* why = nullptr);
// A conic from a to b whose ends' tangents meet at `apex`: rho 0.5 a parabola, below it an
// ellipse's arc, above it a hyperbola's (0 < rho < 1). A spline through points on it.
std::optional<SketchCurve> conicCurve(Vec2 a, Vec2 b, Vec2 apex, float rho, std::string* why = nullptr);
// A point on that conic (t from 0 at a to 1 at b): the exact curve, for checking.
Vec2 conicPoint(Vec2 a, Vec2 b, Vec2 apex, float rho, float t);
// Text as closed polylines (outer outlines and the holes in letters), its baseline from
// `at` along the plane's u axis turned by `angleDeg`.
std::vector<SketchCurve> sketchText(const std::string& text, Vec2 at, float height, TextFont font, float angleDeg, std::string* why = nullptr);

} // namespace sicdm

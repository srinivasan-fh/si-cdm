#pragma once

// Editing sketch curves against each other, as a CAD sketcher does:
//
//   Trim     cut away the piece of a curve between the curves crossing it on either side
//            of a picked point (a circle becomes an arc, a line two lines...)
//   Extend   carry a line's, arc's or open polyline's end on to the next curve it meets
//   Offset   a copy of a curve a distance to one side: parallel lines, concentric arcs
//            and circles, rectangles and polylines with mitred corners, and sampled
//            offsets of ellipses and splines
//   Chains   a closed loop of curves meeting end to end (lines and arcs round a slot,
//            say) as one profile
//
// Lines, arcs and circles are handled exactly; ellipses and splines through fine
// polylines.

#include "core/Sketch.hpp"

#include <optional>
#include <string>
#include <vector>

namespace sicdm {

// Where two curves cross (plane coordinates).
std::vector<Vec2> curveIntersections(const SketchCurve& a, const SketchCurve& b);

// The curves left of curve `index` once the piece around `pick` is cut away (none when
// nothing else crosses it: the whole curve goes). False with `why` for points.
bool trimCurve(const Sketch& sketch, std::size_t index, Vec2 pick, std::vector<SketchCurve>& out, std::string* why = nullptr);

// Curve `index` with its end nearest `pick` carried on to the nearest curve it meets.
bool extendCurve(const Sketch& sketch, std::size_t index, Vec2 pick, SketchCurve& out, std::string* why = nullptr);

// A copy of `c` `distance` mm towards the side `side` is on.
bool offsetCurve(const SketchCurve& c, float distance, Vec2 side, SketchCurve& out, std::string* why = nullptr);

// The closed loop of curves through curve `index` whose ends meet (within `tol` mm), as a
// counter-clockwise polygon (round parts to `chord` mm), and the curves in it. Empty when
// there is none.
std::vector<Vec2> chainLoop(const Sketch& sketch, std::size_t index, std::vector<std::size_t>* members = nullptr, float tol = 1e-3f, float chord = 0.02f);

// How free each curve is under the constraints: -1 none on it, 0 fully constrained, more
// the ways it can still move, -2 a constraint on it can't be met.
std::vector<int> curveFreedom(const Sketch& sketch);

} // namespace sicdm

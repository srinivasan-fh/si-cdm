#pragma once

// Sketch constraints and dimensions, and the solver that moves the sketch curves to
// satisfy them (as a parametric CAD sketcher does).
//
//   Geometric   coincident, horizontal, vertical, parallel, perpendicular, tangent,
//               equal, concentric, point on (a line or circle), midpoint, fix
//   Dimensions  distance (two points, a line's length, a point to a line), horizontal
//               and vertical distance, radius, diameter, angle
//
// The solver is a damped least-squares (Levenberg-Marquardt) on the constraints'
// residuals over the parameters of the curves they name: each step is the smallest
// change that reduces them, so the sketch moves as little as it must. Afterwards the
// rank of the constraints' Jacobian gives the degrees of freedom left.

#include "core/Math.hpp"

#include <string>
#include <vector>

namespace sicdm {

class Sketch;
class Json;

enum class ConstraintKind {
    Coincident, Horizontal, Vertical, Parallel, Perpendicular, Tangent, Equal, Concentric, PointOn, Midpoint, Fix,
    Distance, HDistance, VDistance, Radius, Diameter, Angle
};
const char* constraintKey(ConstraintKind k);
bool constraintFromKey(const std::string& key, ConstraintKind* out);
bool constraintHasValue(ConstraintKind k);

// A curve, one of its points, or one of its segments.
//   point:   Line 0 start / 1 end; Polyline vertex; Rectangle corner 0..3 (counter-
//            clockwise from (x0, y0)); Circle 0 = centre; Point 0
//   segment: Polyline segment k (vertex k to k + 1); Rectangle side k (corner k to k + 1)
struct SketchRef {
    std::string curve;
    int point = -1;
    int segment = -1;
    std::string describe() const; // "Line 1 end", "Rectangle 2 side 3"
};

struct SketchConstraint {
    ConstraintKind kind = ConstraintKind::Coincident;
    std::vector<SketchRef> refs;
    double value = 0, value2 = 0; // a dimension (mm or degrees); Fix: the point; Tangent: +1 outside / -1 inside
    std::string name;             // "D1", "C3"
    int step = -1;                // the feature-history step that made it (-1: none)
    std::string describe() const;
};

struct SolveReport {
    bool ok = false;      // every constraint met
    int dof = 0;          // degrees of freedom left in the constrained curves
    int parameters = 0;
    int redundant = 0;    // constraints that repeat others
    double residual = 0;  // largest error left (mm, or mm-equivalent)
    int iterations = 0;
    std::string conflict; // the constraint that couldn't be met, when !ok
};

// Moves the curves to meet every constraint. On failure the curves are put back.
SolveReport solveSketch(Sketch& sketch);

// What a drag holds: a point of a curve (ref.point), a circle's or arc's rim (its
// radius), or else the whole curve, which then moves without changing shape.
struct SketchGrip {
    SketchRef ref;
    bool rim = false;
};

// Drags the sketch: the grip goes to `to` (plane mm) as nearly as the constraints let
// it, and the curves joined to it by constraints follow, moving as little as they must.
// For a whole curve, `from` is the point grabbed and `to` where it goes. Only the drag's
// own curves and those constrained to them move. The constraints all hold afterwards;
// when they can't (or the grip is no curve's) the sketch is left as it was and ok is
// false. `residual` is how far the grip ended from `to`.
SolveReport dragSketch(Sketch& sketch, const SketchGrip& grip, Vec2 from, Vec2 to);

// The current error of one constraint (0 when met); NaN when its curves are gone.
double constraintError(const Sketch& sketch, const SketchConstraint& c);
// The value a dimension has now (for a new DIM without a value).
double currentValue(const Sketch& sketch, const SketchConstraint& c);

Json constraintsToJson(const Sketch& sketch);
void constraintsFromJson(const Json& j, Sketch& sketch);

} // namespace sicdm

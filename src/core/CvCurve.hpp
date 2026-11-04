#pragma once

// CV curves: B-splines edited by their control vertices, as a stylist shapes a car's
// character lines. A curve is clamped (it starts at its first CV and ends at its last), of
// degree 1 to 7, with as many spans as its CVs allow (CVs - degree).
//
// Continuity where two curves meet, measured and made:
//   G0  the ends touch (gap below 0.001 mm)
//   G1  and their tangents agree (within 0.05 degrees)
//   G2  and so do their curvature vectors (within 0.5 %, or 1e-6 /mm when both are flat)
//   G3  and the rates at which curvature changes along them (within 2 %, or 1e-7 /mm^2)
// MATCH moves the CVs next to an end (one per level: G3 moves the end CV and the three
// after it) so the curve meets another with that continuity, keeping its own speed there:
// its shape away from the end does not change.

#include "core/Brep.hpp"

#include <string>
#include <vector>

namespace sicdm {

struct SketchCurve;

// Uniform knots for a clamped curve of n CVs and this degree.
std::vector<double> clampedKnots(std::size_t n, int degree);

// The B-spline of a CV curve (CurveKind::Nurbs), in world space.
BSplineCurve cvCurveSpline(const SketchCurve& c);
// Makes c a CV curve with this B-spline's CVs, degree and knots (non-rational).
void setCvCurve(SketchCurve& c, const BSplineCurve& b);
// Checks a B-spline is a usable CV curve: degree 1..7, enough CVs, a clamped knot vector
// of the right length that never decreases.
bool validCvCurve(const BSplineCurve& b, std::string* why = nullptr);

// The point and its derivatives up to `order` (at most 3) at t: d[0] the point.
void curveDerivatives(const BSplineCurve& b, double t, int order, DVec3 d[4]);

// What a curve does at t: its point, unit tangent, curvature vector K = dT/ds and its rate
// dK/ds along the curve.
struct CurveLocal {
    DVec3 p, T, K, dK;
    double speed = 0; // |dC/dt|
    double curvature() const;
};
CurveLocal curveLocal(const BSplineCurve& b, double t);

// The same curve run the other way.
BSplineCurve reverseCurve(const BSplineCurve& b);
// Inserts a knot at t (strictly inside the range): one more CV, the same shape.
BSplineCurve insertKnot(const BSplineCurve& b, double t);
// A least-squares curve of this degree and number of spans through the points (at chord
// length parameters), passing exactly through the first and last.
bool fitCurve(const std::vector<DVec3>& pts, int degree, int spans, BSplineCurve& out, std::string* why = nullptr);
// The curve refitted with a new degree and number of spans; maxDeviation is the largest
// distance between the two (sampled).
bool rebuildCurve(const BSplineCurve& in, int degree, int spans, BSplineCurve& out, double* maxDeviation = nullptr, std::string* why = nullptr);
// The largest distance from samples of a to b (closest points, sampled).
double curveDeviation(const BSplineCurve& a, const BSplineCurve& b);
// The largest distance from the points to b (closest points).
double pointsDeviation(const std::vector<DVec3>& pts, const BSplineCurve& b);
double curveLength(const BSplineCurve& b);

// Continuity where a's end meets b's start.
struct Continuity {
    double gap = 0;        // mm
    double angle = 0;      // degrees between the tangents
    double curvature = 0;  // |Ka - Kb| relative to the larger (or absolute when both are flat)
    double rate = 0;       // the same for dK/ds
    int level = -1;        // -1 apart, 0..3 for G0..G3
};
Continuity continuityBetween(const CurveLocal& endOfA, const CurveLocal& startOfB);
// "G2 (gap 0.0000 mm, 0.00 deg, curvature 0.01 %)"-style summary.
std::string continuityText(const Continuity& c);

// Moves CVs at a's start so it continues from `to` (`to` running into the join) with
// continuity G0..G3 (level 0..3). Fails when a has too few CVs or too low a degree. `speed`
// (> 0) sets how fast a leaves the join (|dC/dt|) instead of keeping its own.
bool matchStart(BSplineCurve& a, const CurveLocal& to, int level, std::string* why = nullptr, double speed = 0.0);

// A curve from the end of one curve to the start of another: `into` runs into the first join
// (its end is the blend's start), `outOf` runs into the second when run backwards (its end is
// the blend's end). Continuity levels at each join (0..3); `tension` scales how far the blend
// carries each end's direction (1: about the gap). Degree levelA + levelB + 1, at least 3.
bool blendCurves(const CurveLocal& into, const CurveLocal& outOf, int levelA, int levelB, double tension, BSplineCurve& out, std::string* why = nullptr);

} // namespace sicdm

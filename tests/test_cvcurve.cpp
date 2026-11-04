#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/CvCurve.hpp"
#include "core/Document.hpp"
#include "core/Sketch.hpp"

#include <cmath>
#include <filesystem>
#include <string>

using namespace sicdm;

namespace {

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }

const SketchCurve* curveNamed(const Document& d, const std::string& n) {
    for (const SketchCurve& c : d.scene.sketch().curves())
        if (c.name == n) return &c;
    return nullptr;
}

Continuity joint(const Document& d, const std::string& a, bool aEnd, const std::string& b, bool bEnd) {
    BSplineCurve A = cvCurveSpline(*curveNamed(d, a)), B = cvCurveSpline(*curveNamed(d, b));
    if (!aEnd) A = reverseCurve(A); // A runs into the join
    if (bEnd) B = reverseCurve(B);  // B runs away from it
    return continuityBetween(curveLocal(A, A.t1()), curveLocal(B, B.t0()));
}

} // namespace

TEST_CASE("CV curves: made by their CVs, through the end ones; knots inserted keep the shape") {
    Document d;
    auto r = run(d, "CVCURVE 0,0,0 30,40,0 70,40,10 100,0,0 NAME Sweep");
    CHECK(r.ok);
    const SketchCurve* c = curveNamed(d, "Sweep");
    CHECK(c != nullptr);
    if (!c) return;
    CHECK(c->kind == CurveKind::Nurbs);
    const BSplineCurve b = cvCurveSpline(*c);
    CHECK(b.degree == 3);
    CHECK(b.cp.size() == 4u);
    CHECK(dlength(b.eval(b.t0()) - DVec3{0, 0, 0}) < 1e-6);
    CHECK(dlength(b.eval(b.t1()) - DVec3{100, 0, 0}) < 1e-6);
    // The drawn line follows the B-spline (not a spline through the CVs).
    const auto strip = c->worldStrip();
    CHECK(strip.size() > 8);
    CHECK(dlength(DVec3(strip[strip.size() / 2]) - b.eval(0.5)) < 2.0); // near the middle
    // A knot: one more CV, the same curve.
    CHECK(run(d, "CVCURVE INSERT Sweep 0.3").ok);
    const BSplineCurve b2 = cvCurveSpline(*curveNamed(d, "Sweep"));
    CHECK(b2.cp.size() == 5u);
    CHECK(curveDeviation(b, b2) < 1e-6);
    CHECK(curveDeviation(b2, b) < 1e-6);
    // A CV moved; the ends stay where they are unless their own CV moves.
    CHECK(run(d, "CVCURVE MOVE Sweep 3 0,0,20").ok);
    const BSplineCurve b3 = cvCurveSpline(*curveNamed(d, "Sweep"));
    CHECK(dlength(b3.eval(b3.t0()) - DVec3{0, 0, 0}) < 1e-6);
    CHECK(b3.cp[2].z > b2.cp[2].z + 19.0);
    CHECK(run(d, "CVCURVE INFO Sweep").ok);
}

TEST_CASE("CV curves: MATCH makes G0, G1, G2 and G3 joins and leaves the far end alone") {
    for (int g = 0; g <= 3; ++g) {
        Document d;
        CHECK(run(d, "CVCURVE 0,0,0 40,10,0 80,30,5 120,20,15 160,0,10 NAME A").ok);
        CHECK(run(d, "CVCURVE 165,5,12 200,-20,0 240,-10,-10 270,10,0 300,0,0 NAME B DEGREE 4").ok);
        const BSplineCurve before = cvCurveSpline(*curveNamed(d, "B"));
        const auto r = run(d, "CVCURVE MATCH B START TO A END G" + std::to_string(g));
        CHECK(r.ok);
        const Continuity k = joint(d, "A", true, "B", false);
        CHECK(k.level >= g);
        CHECK(k.gap < 1e-6);
        // CVs past the matched ones haven't moved.
        const BSplineCurve after = cvCurveSpline(*curveNamed(d, "B"));
        for (std::size_t i = static_cast<std::size_t>(g) + 1; i < after.cp.size(); ++i) CHECK(dlength(after.cp[i] - before.cp[i]) < 1e-9);
    }
    // At the other ends too: B's end to A's start (B runs out of it, A into it reversed).
    Document d;
    CHECK(run(d, "CVCURVE 0,0,0 40,10,0 80,30,5 120,20,15 NAME A").ok);
    CHECK(run(d, "CVCURVE 300,0,0 200,-20,0 100,-10,-10 5,3,1 NAME B").ok);
    CHECK(run(d, "CVCURVE MATCH B END TO A START G2").ok);
    CHECK(joint(d, "B", true, "A", false).level >= 2);
}

TEST_CASE("CV curves: matched to an arc's curvature, fitted and rebuilt within tolerance") {
    Document d;
    CHECK(run(d, "ARC CENTER 0,0 50,0 0,50").ok); // centre, start, end: radius 50
    const std::string arc = d.scene.sketch().curves().back().name;
    CHECK(run(d, "CVCURVE 0,50,0 -30,60,0 -60,80,0 -90,120,0 NAME Run").ok);
    CHECK(run(d, "CVCURVE MATCH Run START TO \"" + arc + "\" END G2").ok);
    const BSplineCurve b = cvCurveSpline(*curveNamed(d, "Run"));
    const CurveLocal s = curveLocal(b, b.t0());
    CHECK_NEAR(s.curvature(), 1.0 / 50.0, 2e-4); // the arc's, from a fitted copy of it
    CHECK(std::fabs(s.T.y) < 1e-3);              // tangent to the arc at its top
    // FIT through points of a quarter circle: a few spans hold it to a hundredth.
    std::string fit = "CVCURVE FIT";
    for (int i = 0; i <= 24; ++i) {
        const double a = 3.14159265358979 / 2 * i / 24;
        fit += " " + std::to_string(100 * std::cos(a)) + "," + std::to_string(100 * std::sin(a)) + ",0";
    }
    CHECK(run(d, fit + " SPANS 4 NAME Quarter").ok);
    const BSplineCurve q = cvCurveSpline(*curveNamed(d, "Quarter"));
    double worst = 0;
    for (int i = 0; i <= 100; ++i) worst = std::max(worst, std::fabs(dlength(q.eval(q.t0() + (q.t1() - q.t0()) * i / 100)) - 100.0));
    CHECK(worst < 0.01);
    // Rebuilt to degree 5 with 2 spans: still close.
    CHECK(run(d, "CVCURVE REBUILD Quarter DEGREE 5 SPANS 2").ok);
    const BSplineCurve q5 = cvCurveSpline(*curveNamed(d, "Quarter"));
    CHECK(q5.degree == 5);
    CHECK(q5.cp.size() == 7u);
    CHECK(curveDeviation(q5, q) < 0.05);
    // FROM: any sketch curve as a CV curve.
    CHECK(run(d, "CVCURVE FROM \"" + arc + "\" NAME ArcCv").ok);
    CHECK(curveNamed(d, "ArcCv") != nullptr);
}

TEST_CASE("CV curves: saved and opened again, undone, and refused when they can't be made") {
    Document d;
    CHECK(run(d, "CVCURVE 0,0,0 10,20,0 30,20,5 40,0,0 50,-10,0 DEGREE 3 NAME K").ok);
    CHECK(run(d, "CVCURVE INSERT K 0.25").ok);
    const BSplineCurve before = cvCurveSpline(*curveNamed(d, "K"));
    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_cvcurve.sicdm").string();
    CHECK(run(d, "SAVEAS \"" + path + "\"").ok);
    Document e;
    CHECK(run(e, "OPEN \"" + path + "\"").ok);
    const SketchCurve* k = curveNamed(e, "K");
    CHECK(k != nullptr);
    if (k) {
        const BSplineCurve after = cvCurveSpline(*k);
        CHECK(after.degree == before.degree);
        CHECK(after.knots.size() == before.knots.size());
        CHECK(curveDeviation(after, before) < 1e-3);
    }
    std::filesystem::remove(path);
    // UNDO takes an edit back.
    CHECK(run(d, "CVCURVE MOVE K 2 0,0,30").ok);
    CHECK(run(d, "UNDO").ok);
    CHECK(curveDeviation(cvCurveSpline(*curveNamed(d, "K")), before) < 1e-4);
    // Refusals with reasons.
    CHECK(!run(d, "CVCURVE 0,0,0 10,0,0 DEGREE 3").ok);          // too few CVs for the degree
    CHECK(!run(d, "CVCURVE 0,0,0 0,0,0 10,0,0").ok);             // two CVs on top of each other
    CHECK(!run(d, "CVCURVE INSERT K 1.5").ok);                   // outside the curve
    CHECK(!run(d, "CVCURVE MATCH K START TO Nothing END").ok);   // no such curve
    CHECK(run(d, "CVCURVE 0,0,0 10,5,0 20,0,0 DEGREE 2 NAME Q").ok);
    const auto g3 = run(d, "CVCURVE MATCH Q START TO K END G3"); // degree 2 can't hold G3
    CHECK(!g3.ok);
    CHECK(g3.lines.back().find("degree 3") != std::string::npos);
    CHECK(!run(d, "CVCURVE MOVE K 99 1,1,1").ok);                // no CV 99
}

TEST_CASE("CV curves: a surface lofted through two of them") {
    Document d;
    CHECK(run(d, "CVCURVE 0,0,0 30,0,10 70,0,10 100,0,0 NAME C1").ok);
    CHECK(run(d, "CVCURVE 0,80,0 30,80,25 70,80,25 100,80,0 NAME C2").ok);
    const std::size_t bodies = d.scene.bodies().size();
    const auto r = run(d, "SURFACE LOFT C1 C2");
    CHECK(r.ok);
    CHECK(d.scene.bodies().size() == bodies + 1);
}

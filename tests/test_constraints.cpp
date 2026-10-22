#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Constraints.hpp"
#include "core/Document.hpp"
#include "core/Scene.hpp"

#include <cmath>
#include <filesystem>
#include <string>

using namespace sicdm;

#define REQUIRE(c)                  \
    do {                            \
        const bool required_ = (c); \
        CHECK(required_);           \
        if (!required_) return;     \
    } while (0)

namespace {

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }

std::string text(const CommandOutput& r) {
    std::string s;
    for (const std::string& l : r.lines) s += l + "\n";
    return s;
}

const SketchCurve& curve(const Document& d, const std::string& name) {
    for (const SketchCurve& c : d.scene.sketch().curves())
        if (c.name == name) return c;
    static SketchCurve none;
    return none;
}

double dist(Vec2 a, Vec2 b) { return std::hypot(a.x - b.x, a.y - b.y); }

// A rough quadrilateral made an exact 60 x 40 rectangle with its corner fixed.
void rectangle(Document& d) {
    REQUIRE(run(d, "PLINE 0,0 48,2 50,31 -1,29 CLOSE").ok);
    for (const char* l : {"CONSTRAIN HORIZONTAL \"Polyline 1\" SIDE 1", "CONSTRAIN VERTICAL \"Polyline 1\" SIDE 2", "CONSTRAIN HORIZONTAL \"Polyline 1\" SIDE 3",
                          "CONSTRAIN VERTICAL \"Polyline 1\" SIDE 4", "DIM \"Polyline 1\" SIDE 1 60", "DIM \"Polyline 1\" SIDE 2 40",
                          "CONSTRAIN FIX \"Polyline 1\" START"})
        REQUIRE(run(d, l).ok);
}

} // namespace

TEST_CASE("Sketch constraints square up a rough outline and count its degrees of freedom") {
    Document d;
    REQUIRE(run(d, "PLINE 0,0 48,2 50,31 -1,29 CLOSE").ok);
    auto r = run(d, "CONSTRAIN HORIZONTAL \"Polyline 1\" SIDE 1");
    REQUIRE(r.ok);
    CHECK(text(r).find("free in 7 ways") != std::string::npos); // 4 corners, 8 numbers, one fixed by the constraint
    Document e;
    rectangle(e);
    const SketchCurve p = curve(e, "Polyline 1");
    REQUIRE(p.pts.size() == 5);
    CHECK_NEAR(p.pts[1].y - p.pts[0].y, 0.0, 1e-4);
    CHECK_NEAR(p.pts[2].x - p.pts[1].x, 0.0, 1e-4);
    CHECK_NEAR(dist(p.pts[0], p.pts[1]), 60.0, 1e-4);
    CHECK_NEAR(dist(p.pts[1], p.pts[2]), 40.0, 1e-4);
    CHECK_NEAR(dist(p.pts[4], p.pts[0]), 0.0, 1e-6); // still closed
    r = run(e, "CONSTRAINTS");
    CHECK(text(r).find("fully constrained") != std::string::npos);
    // A conflicting constraint is refused, and nothing moves.
    const Vec2 before = p.pts[2];
    r = run(e, "CONSTRAIN PARALLEL \"Polyline 1\" SIDE 1 \"Polyline 1\" SIDE 2");
    CHECK(!r.ok);
    CHECK(text(r).find("conflicts") != std::string::npos);
    CHECK_NEAR(dist(curve(e, "Polyline 1").pts[2], before), 0.0, 1e-6);
    CHECK(e.scene.sketch().constraints().size() == 7);
}

TEST_CASE("Sketch constraints: tangent, equal, concentric, point on, midpoint, angle") {
    Document d;
    rectangle(d);
    REQUIRE(run(d, "CIRCLE 20,15 7").ok);
    REQUIRE(run(d, "CONSTRAIN TANGENT \"Polyline 1\" SIDE 1 \"Circle 2\"").ok);
    REQUIRE(run(d, "CONSTRAIN TANGENT \"Polyline 1\" SIDE 4 \"Circle 2\"").ok);
    REQUIRE(run(d, "DIM \"Circle 2\" 8").ok);
    const SketchCurve p = curve(d, "Polyline 1"); // copies: adding curves moves the list
    const SketchCurve c = curve(d, "Circle 2");
    CHECK_NEAR(c.radius, 8.0, 1e-4);
    CHECK_NEAR(c.pts[0].y - p.pts[0].y, 8.0, 1e-4); // tangent to the bottom side
    CHECK_NEAR(c.pts[0].x - p.pts[0].x, 8.0, 1e-4); // and to the left one
    REQUIRE(run(d, "CIRCLE 0,0 3").ok);
    REQUIRE(run(d, "CONSTRAIN CONCENTRIC \"Circle 2\" \"Circle 3\"").ok);
    REQUIRE(run(d, "CIRCLE 50,50 5").ok);
    REQUIRE(run(d, "CONSTRAIN EQUAL \"Circle 2\" \"Circle 4\"").ok);
    CHECK_NEAR(dist(curve(d, "Circle 3").pts[0], c.pts[0]), 0.0, 1e-4);
    CHECK_NEAR(curve(d, "Circle 4").radius, 8.0, 1e-4);
    REQUIRE(run(d, "POINT 25,35").ok);
    REQUIRE(run(d, "CONSTRAIN MIDPOINT \"Point 5\" \"Polyline 1\" SIDE 3").ok);
    const Vec2 m = curve(d, "Point 5").pts[0];
    CHECK_NEAR(dist(m, {(p.pts[2].x + p.pts[3].x) * 0.5f, (p.pts[2].y + p.pts[3].y) * 0.5f}), 0.0, 1e-4);
    REQUIRE(run(d, "LINE 0,60 30,75").ok);
    REQUIRE(run(d, "CONSTRAIN ON \"Line 6\" START \"Circle 4\"").ok);
    CHECK_NEAR(dist(curve(d, "Line 6").pts[0], curve(d, "Circle 4").pts[0]), 8.0, 1e-4);
    REQUIRE(run(d, "DIM ANGLE \"Line 6\" \"Polyline 1\" SIDE 1 30").ok);
    const SketchCurve l = curve(d, "Line 6");
    const double ang = std::atan2(std::fabs(l.pts[1].y - l.pts[0].y), std::fabs(l.pts[1].x - l.pts[0].x)) * 180 / 3.14159265358979;
    CHECK_NEAR(ang, 30.0, 1e-3);
    CHECK(!run(d, "CONSTRAIN TANGENT \"Point 5\" \"Line 6\"").ok); // no such tangency
    CHECK(!run(d, "CONSTRAIN WOBBLY \"Line 6\"").ok);
    CHECK(!run(d, "DIM \"Nothing\" 5").ok);
}

TEST_CASE("Changing a dimension rebuilds the model from the history") {
    Document d;
    rectangle(d);
    REQUIRE(run(d, "EXTRUDE \"Polyline 1\" 10").ok);
    REQUIRE(d.scene.bodies().size() == 1);
    Vec3 size = d.scene.bodies()[0].bounds().max - d.scene.bodies()[0].bounds().min;
    CHECK_NEAR(size.x, 60.0, 1e-3);
    const auto r = run(d, "CONSTRAINT EDIT D5 80");
    REQUIRE(r.ok);
    CHECK(text(r).find("rebuilt from step 6") != std::string::npos);
    REQUIRE(d.scene.bodies().size() == 1);
    size = d.scene.bodies()[0].bounds().max - d.scene.bodies()[0].bounds().min;
    CHECK_NEAR(size.x, 80.0, 1e-3);
    CHECK_NEAR(size.y, 40.0, 1e-3);
    CHECK(d.state.history[5].line == "DIM \"Polyline 1\" SIDE 1 80"); // the step itself says 80 now
    CHECK(d.state.history.size() == 9);                              // and the edit isn't a step of its own
    // Saved and opened: the constraints come back, and still solve.
    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_constraints.sicdm").string();
    REQUIRE(run(d, "SAVE " + path).ok);
    Document e;
    REQUIRE(run(e, "OPEN " + path).ok);
    CHECK(e.scene.sketch().constraints().size() == 7);
    CHECK(text(run(e, "CONSTRAINTS")).find("fully constrained") != std::string::npos);
    REQUIRE(run(e, "CONSTRAINT EDIT D6 25").ok);
    size = e.scene.bodies()[0].bounds().max - e.scene.bodies()[0].bounds().min;
    CHECK_NEAR(size.y, 25.0, 1e-3);
    REQUIRE(run(e, "CONSTRAINT DELETE C7").ok);
    CHECK(e.scene.sketch().constraints().size() == 6);
    std::filesystem::remove(path);
}

#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Features.hpp"
#include "core/Scene.hpp"
#include "core/Sculpt.hpp"

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

constexpr double kPiD = 3.14159265358979323846;

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }

std::string text(const CommandOutput& r) {
    std::string s;
    for (const std::string& l : r.lines) s += l + "\n";
    return s;
}

const Body* named(const Document& d, const std::string& name) {
    for (const Body& b : d.scene.bodies())
        if (b.name == name) return &b;
    return nullptr;
}

double volumeOf(const Body& b) { return meshVolume(worldMesh(b)); }

bool closed(const Body& b) { return openEdgeCount(b.mesh) == 0; }

// A regular n-gon's area over the circle's (the 96-gon profiles of circles).
double polygonFactor(int n) { return n / (2 * kPiD) * std::sin(2 * kPiD / n); }

} // namespace

TEST_CASE("REVOLVE turns a sketch profile about an axis: a closed tube, a capped quarter, and refusals") {
    Document d;
    REQUIRE(run(d, "PLANE FRONT").ok);
    REQUIRE(run(d, "RECT 10,0 @10,30").ok);
    REQUIRE(run(d, "REVOLVE \"Rectangle 1\"").ok);
    const Body* tube = named(d, "Revolve 1");
    REQUIRE(tube != nullptr);
    CHECK(closed(*tube));
    CHECK_NEAR(volumeOf(*tube), kPiD * (400 - 100) * 30 * polygonFactor(96), 1.0);
    const AABB b = tube->bounds();
    CHECK_NEAR(b.max.x, 20.0, 1e-3);
    CHECK_NEAR(b.max.z, 30.0, 1e-3);
    // A quarter turn is capped at both ends.
    REQUIRE(run(d, "RECT 10,40 @10,10").ok);
    REQUIRE(run(d, "REVOLVE \"Rectangle 2\" ANGLE 90").ok);
    const Body* q = named(d, "Revolve 2");
    REQUIRE(q != nullptr);
    CHECK(closed(*q));
    CHECK_NEAR(volumeOf(*q), kPiD * (400 - 100) * 10 / 4, 2.0);
    // A profile across the axis, an open curve.
    REQUIRE(run(d, "RECT -5,60 @10,10").ok);
    CHECK(!run(d, "REVOLVE \"Rectangle 3\" AXIS V").ok);
    REQUIRE(run(d, "LINE 0,0 10,10").ok);
    CHECK(!run(d, "REVOLVE \"Line 4\"").ok);
    // About a sketch line as the axis.
    REQUIRE(run(d, "LINE 50,0 50,10").ok);
    REQUIRE(run(d, "CIRCLE 60,20 3").ok);
    REQUIRE(run(d, "REVOLVE \"Circle 6\" AXIS \"Line 5\"").ok); // a torus about x = 50
    const Body* t = named(d, "Revolve 3");
    REQUIRE(t != nullptr);
    CHECK(closed(*t));
    CHECK_NEAR(volumeOf(*t), 2 * kPiD * 10 * kPiD * 9 * polygonFactor(96) * polygonFactor(96), 15.0);
}

TEST_CASE("SWEEP carries a profile along a path: mitred corners keep the section, closed paths close") {
    Document d;
    REQUIRE(run(d, "PLINE 0,0 60,0 60,40 100,40").ok); // on Top
    REQUIRE(run(d, "PLANE RIGHT").ok);
    REQUIRE(run(d, "CIRCLE 0,0 4").ok);
    REQUIRE(run(d, "SWEEP \"Circle 2\" ALONG \"Polyline 1\"").ok);
    const Body* s = named(d, "Sweep 1");
    REQUIRE(s != nullptr);
    CHECK(closed(*s));
    CHECK_NEAR(volumeOf(*s), kPiD * 16 * polygonFactor(96) * 140, 140 * 0.5);
    const AABB b = s->bounds();
    CHECK_NEAR(b.min.x, 0.0, 1e-3); // starts square at the path's start
    CHECK_NEAR(b.max.x, 100.0, 1e-3);
    CHECK_NEAR(b.max.z, 4.0, 1e-2);
    // Round a rectangle: a closed square ring.
    REQUIRE(run(d, "PLANE TOP").ok);
    REQUIRE(run(d, "RECT 200,0 @40,40").ok);
    REQUIRE(run(d, "PLANE FRONT").ok);
    REQUIRE(run(d, "RECT -2,-2 @4,4").ok);
    REQUIRE(run(d, "SWEEP \"Rectangle 4\" ALONG \"Rectangle 3\"").ok);
    const Body* ring = named(d, "Sweep 2");
    REQUIRE(ring != nullptr);
    CHECK(closed(*ring));
    CHECK_NEAR(volumeOf(*ring), 16.0 * 160, 2.0); // a 4 x 4 section round a 160 mm centre line
    CHECK(!run(d, "SWEEP \"Rectangle 4\"").ok);   // no path
}

TEST_CASE("LOFT joins sections at their offsets, ruled or smooth") {
    Document d;
    REQUIRE(run(d, "CIRCLE 0,0 15").ok);
    REQUIRE(run(d, "RECT -10,-10 @20,20").ok);
    REQUIRE(run(d, "LOFT \"Circle 1\" \"Rectangle 2\" AT 40 RULED").ok);
    const Body* l = named(d, "Loft 1");
    REQUIRE(l != nullptr);
    CHECK(closed(*l));
    const AABB b = l->bounds();
    CHECK_NEAR(b.min.z, 0.0, 1e-3);
    CHECK_NEAR(b.max.z, 40.0, 1e-3);
    const double v = volumeOf(*l);
    CHECK(v > 400.0 * 40); // between the square's and the circle's prisms
    CHECK(v < kPiD * 225 * 40);
    REQUIRE(run(d, "CIRCLE 0,0 8").ok);
    REQUIRE(run(d, "LOFT \"Circle 1\" \"Rectangle 2\" AT 40 \"Circle 3\" AT 80 KEEP").ok);
    const Body* s = named(d, "Loft 2");
    REQUIRE(s != nullptr);
    CHECK(closed(*s));
    CHECK_NEAR(s->bounds().max.z, 80.0, 1e-3);
    CHECK(!run(d, "LOFT \"Circle 1\"").ok);
}

TEST_CASE("PATTERN copies a body in rows or round an axis, or joins / cuts the copies") {
    Document d;
    REQUIRE(run(d, "CYLINDER 3 10 @0,0,0").ok);
    auto r = run(d, "PATTERN \"Cylinder 1\" LINEAR 4 SPACING 10 DIR X ROWS 2 SPACING2 15 DIR2 Y");
    REQUIRE(r.ok);
    CHECK(d.scene.bodies().size() == 8);
    const Body* last = named(d, "Cylinder 1 (8)");
    REQUIRE(last != nullptr);
    CHECK_NEAR(last->bounds().center().x, 30.0, 1e-3);
    CHECK_NEAR(last->bounds().center().y, 15.0, 1e-3);
    // Round the Z axis through (0, 50): six at 60 degrees.
    REQUIRE(run(d, "PATTERN \"Cylinder 1\" CIRCULAR 6 AXIS Z AT 0,50,0").ok);
    CHECK(d.scene.bodies().size() == 13);
    const Body* c = named(d, "Cylinder 1 (10)"); // the second around: 120 degrees
    REQUIRE(c != nullptr);
    CHECK_NEAR(c->bounds().center().x, 50 * std::sin(2 * kPiD / 3), 0.05); // counter-clockwise about +Z
    CHECK_NEAR(c->bounds().center().y, 50 - 50 * std::cos(2 * kPiD / 3), 0.05);
    // Three holes cut through a plate.
    Document e;
    REQUIRE(run(e, "BOX 60 60 5 @0,0,0").ok);
    REQUIRE(run(e, "CYLINDER 4 20 @-20,0,-5").ok);
    REQUIRE(run(e, "PATTERN \"Cylinder 2\" LINEAR 3 SPACING 20 SUBTRACT \"Block 1\"").ok);
    REQUIRE(e.scene.bodies().size() == 1);
    CHECK(closed(e.scene.bodies()[0]));
    CHECK_NEAR(volumeOf(e.scene.bodies()[0]), 60.0 * 60 * 5 - 3 * kPiD * 16 * 5 * polygonFactor(64), 25.0);
    CHECK(!run(e, "PATTERN \"Block 1\" LINEAR 1 SPACING 5").ok);
    CHECK(!run(e, "PATTERN \"Block 1\" LINEAR 3").ok); // no spacing
}

TEST_CASE("MIRROR copies, flips, or joins a body across a plane") {
    Document d;
    REQUIRE(run(d, "BOX 20 10 10 @20,0,0").ok); // x 10..30
    REQUIRE(run(d, "MIRROR \"Block 1\" PLANE YZ").ok);
    const Body* m = named(d, "Block 1 (2)");
    REQUIRE(m != nullptr);
    CHECK_NEAR(m->bounds().min.x, -30.0, 1e-4);
    CHECK_NEAR(m->bounds().max.x, -10.0, 1e-4);
    CHECK(volumeOf(*m) > 0); // turned the right way out
    REQUIRE(run(d, "MIRROR \"Block 1\" PLANE XY AT 0,0,20 MOVE").ok);
    CHECK_NEAR(named(d, "Block 1")->bounds().min.z, 30.0, 1e-4);
    // A half model made whole: mirrored across its cut face and joined.
    Document e;
    REQUIRE(run(e, "BOX 20 10 10 @10,0,0").ok); // x 0..20
    REQUIRE(run(e, "MIRROR \"Block 1\" PLANE YZ UNION").ok);
    REQUIRE(e.scene.bodies().size() == 1);
    const Body& w = e.scene.bodies()[0];
    CHECK(closed(w));
    CHECK_NEAR(volumeOf(w), 40.0 * 10 * 10, 1.0);
    CHECK_NEAR(w.bounds().min.x, -20.0, 1e-4);
}

TEST_CASE("FILLET rounds edges with a rolling ball: a box comes out as the exact rounded box") {
    for (const float r : {2.0f, 3.0f}) {
        Body b = makeBody("Block", "box", {0, 0, 0, 40, 30, 20});
        RoundOptions o;
        o.radius = r;
        std::string err;
        REQUIRE(roundBody(b, o, &err));
        CHECK(closed(b));
        const AABB box = b.bounds(); // the faces stay where they were
        CHECK_NEAR(box.min.x, 0.0, 0.01);
        CHECK_NEAR(box.max.x, 40.0, 0.01);
        CHECK_NEAR(box.max.z, 20.0, 0.01);
        // Steiner: the inner box grown by r.
        const double a = 40 - 2 * r, bb = 30 - 2 * r, c = 20 - 2 * r;
        const double exact = a * bb * c + 2 * r * (a * bb + bb * c + a * c) + kPiD * r * r * (a + bb + c) + 4.0 / 3 * kPiD * r * r * r;
        CHECK_NEAR(volumeOf(b), exact, exact * 0.008);
        // Every point of the surface on the rounded box, within a few hundredths.
        double worst = 0, sum = 0;
        for (const Vertex& v : b.mesh.vertices) {
            const Vec3 q{std::fabs(v.position.x - 20) - (20 - r), std::fabs(v.position.y - 15) - (15 - r), std::fabs(v.position.z - 10) - (10 - r)};
            const Vec3 qp{std::max(q.x, 0.f), std::max(q.y, 0.f), std::max(q.z, 0.f)};
            const double sd = length(qp) + std::min(std::max(q.x, std::max(q.y, q.z)), 0.f) - r;
            worst = std::max(worst, std::fabs(sd));
            sum += std::fabs(sd);
        }
        CHECK(sum / static_cast<double>(b.mesh.vertices.size()) < 0.02);
        CHECK(worst < r * 0.05);
    }
    // Fill the inside corner of an L; a box has none to fill.
    Document d;
    REQUIRE(run(d, "BOX 40 20 10 @0,0,0").ok);
    REQUIRE(run(d, "BOX 20 20 30 @10,0,0").ok);
    REQUIRE(run(d, "BOOLEAN \"Block 1\" UNION \"Block 2\"").ok);
    const double v0 = volumeOf(d.scene.bodies()[0]);
    REQUIRE(run(d, "FILLET \"Block 1\" RADIUS 3 CONCAVE").ok);
    const Body& l = d.scene.bodies()[0];
    CHECK(closed(l));
    // The inside edge (x = 0, z = 10) is filled with a quarter round about (-3, 13); the
    // edges left sharp soften only within a voxel (the volume moves by tens of mm^3).
    int inCorner = 0;
    double off = 0;
    for (const Vertex& v : l.mesh.vertices) {
        const Vec3& p = v.position;
        if (p.x > -3 && p.x < -0.05 && p.z > 10.05 && p.z < 13 && std::fabs(p.y) < 6) {
            ++inCorner;
            off = std::max(off, std::fabs(std::hypot(p.x + 3.0, p.z - 13.0) - 3.0));
        }
    }
    CHECK(inCorner > 20);
    CHECK(off < 0.08);
    CHECK_NEAR(volumeOf(l), v0, 80.0);
    // A plate thinner than the ball has nothing a ball can roll in.
    REQUIRE(run(d, "BOX 40 40 2 @100,0,0").ok);
    CHECK(!run(d, "FILLET \"Block 3\" RADIUS 3").ok);
    CHECK(!run(d, "FILLET \"Block 3\"").ok); // no radius
}

TEST_CASE("MASSPROPS: volume, mass, centre of mass and inertia of solids") {
    Document d;
    REQUIRE(run(d, "BOX 40 30 20 @100,0,0").ok);
    const auto r = run(d, "MASSPROPS \"Block 1\" MATERIAL steel");
    REQUIRE(r.ok);
    CHECK(text(r).find("mass 188.40 g") != std::string::npos);
    const SolidMass m = solidMass(worldMesh(d.scene.bodies()[0]), 7.85);
    CHECK_NEAR(m.volume, 24000.0, 1e-6);
    CHECK_NEAR(m.area, 5200.0, 1e-6);
    CHECK_NEAR(m.centroid.x, 100.0, 1e-4);
    CHECK_NEAR(m.centroid.z, 10.0, 1e-4);
    const double mass = 24.0 * 7.85;
    CHECK_NEAR(m.inertia[2][2], mass * (1600.0 + 900.0) / 12.0, 1e-3);
    CHECK_NEAR(m.inertia[0][0], mass * (900.0 + 400.0) / 12.0, 1e-3);
    CHECK_NEAR(m.inertia[0][1], 0.0, 1e-3);
    CHECK_NEAR(m.principal[0], mass * (900.0 + 400.0) / 12.0, 1e-3);
    // A cylinder's moment about its axis, m r^2 / 2.
    REQUIRE(run(d, "CYLINDER 10 50 @0,0,0").ok);
    const SolidMass c = solidMass(worldMesh(d.scene.bodies()[1]), 2.7);
    CHECK_NEAR(c.inertia[2][2], c.mass * 100.0 / 2.0, c.mass * 100.0 / 2.0 * 0.01);
    CHECK(!run(d, "MASSPROPS \"Block 1\" MATERIAL unobtainium").ok);
    CHECK(materialDensity("aluminium").value_or(0) == 2.70);
}

TEST_CASE("MEASURE: between points and the closest distance between bodies") {
    Document d;
    REQUIRE(run(d, "BOX 10 10 10 @0,0,0").ok);
    REQUIRE(run(d, "BOX 10 10 10 @25,0,0").ok); // 15 apart
    auto r = run(d, "MEASURE \"Block 1\" \"Block 2\"");
    REQUIRE(r.ok);
    CHECK(text(r).find(": 15 mm") != std::string::npos);
    r = run(d, "MEASURE 0,0,0 3,4,12");
    REQUIRE(r.ok);
    CHECK(text(r).find("Distance 13 mm") != std::string::npos);
    CHECK(!run(d, "MEASURE \"Block 1\" \"Nothing\"").ok);
}

TEST_CASE("Feature history: recorded, edited, inserted, deleted, regenerated, undone and saved") {
    Document d;
    REQUIRE(run(d, "BOX 60 40 15 @0,0,0").ok);
    REQUIRE(run(d, "HOLE \"Block 1\" AT -20,0,15 DIAMETER 6 THROUGH").ok);
    REQUIRE(run(d, "LIST").ok); // not a model step
    REQUIRE(d.state.history.size() == 2);
    CHECK(d.state.history[1].line == "HOLE \"Block 1\" AT -20,0,15 DIAMETER 6 THROUGH");
    const double v6 = volumeOf(d.scene.bodies()[0]);
    // A bigger hole: the step is changed and the model rebuilt.
    REQUIRE(run(d, "HISTORY EDIT 2 HOLE \"Block 1\" AT -20,0,15 DIAMETER 10 THROUGH").ok);
    REQUIRE(d.scene.bodies().size() == 1);
    CHECK_NEAR(v6 - volumeOf(d.scene.bodies()[0]), kPiD * (25 - 9) * 15, 15.0);
    REQUIRE(run(d, "HISTORY INSERT 2 HOLE \"Block 1\" AT 20,0,15 DIAMETER 10 THROUGH").ok);
    CHECK(d.state.history.size() == 3);
    const double two = volumeOf(d.scene.bodies()[0]);
    REQUIRE(run(d, "HISTORY DELETE 3").ok);
    CHECK(volumeOf(d.scene.bodies()[0]) > two + 100);
    // A step that no longer works leaves everything as it was.
    const std::size_t steps = d.state.history.size();
    const auto bad = run(d, "HISTORY EDIT 1 BOX 5 5 5 @300,0,0");
    CHECK(!bad.ok);
    CHECK(text(bad).find("stopped at step 2") != std::string::npos);
    CHECK(d.state.history.size() == steps);
    CHECK(d.state.history[0].line == "BOX 60 40 15 @0,0,0");
    // UNDO takes a step back off the history; REDO puts it back.
    REQUIRE(run(d, "UNDO").ok);
    CHECK(d.state.history.size() == steps - 1);
    REQUIRE(run(d, "REDO").ok);
    CHECK(d.state.history.size() == steps);
    // Saved with the project.
    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_history_test.sicdm").string();
    REQUIRE(run(d, "SAVE " + path).ok);
    Document e;
    REQUIRE(run(e, "OPEN " + path).ok);
    CHECK(e.state.history.size() == steps);
    REQUIRE(run(e, "REGEN").ok);
    CHECK_NEAR(volumeOf(e.scene.bodies()[0]), volumeOf(d.scene.bodies()[0]), 1e-3);
    std::filesystem::remove(path);
    // An edit outside the commands (a brush stroke, a drag) breaks the replay.
    d.scene.bodies()[0].offset = {5, 0, 0};
    d.scene.touchBodies();
    REQUIRE(run(d, "LIST").ok);
    CHECK(!d.state.historyComplete);
    CHECK(!run(d, "REGEN").ok);
    REQUIRE(run(d, "REGEN FORCE").ok);
    CHECK(d.state.historyComplete);
    CHECK_NEAR(d.scene.bodies()[0].offset.x, 0.0f, 1e-6);
    // NEW starts a fresh history (and CAM job).
    REQUIRE(run(d, "CAM SETUP \"Block 1\"").ok);
    REQUIRE(run(d, "NEW").ok);
    CHECK(d.state.history.empty());
    CHECK(d.state.cam.empty());
}

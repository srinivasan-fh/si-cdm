#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Scene.hpp"
#include "core/Sculpt.hpp"

#include <cmath>
#include <string>

using namespace sicdm;

#define REQUIRE(c)               \
    do {                         \
        const bool required_ = (c); \
        CHECK(required_);        \
        if (!required_) return;  \
    } while (0)

namespace {

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }

constexpr double kPiD = 3.14159265358979323846;

// Area of the regular n-gon a cutter of radius r is built from.
double polygonArea(double r, int n) { return 0.5 * n * r * r * std::sin(2 * kPiD / n); }

} // namespace

TEST_CASE("booleans are exact and watertight") {
    const Body box = makeBody("A", "box", {0, 0, 0, 100, 100, 100});
    Body cyl = makeBody("C", "cylinder", {30, 150});
    cyl.offset = {50, 50, -20};
    const double vBox = meshVolume(box.mesh), vCyl = meshVolume(worldMesh(cyl));
    const double vInside = vCyl * 100.0 / 150.0; // the part of the cylinder within the box
    for (const BooleanOp op : {BooleanOp::Union, BooleanOp::Subtract, BooleanOp::Intersect}) {
        Body a = box;
        std::string error;
        REQUIRE(bodyBoolean(a, cyl, op, &error));
        CHECK(openEdgeCount(a.mesh) == 0);
        CHECK(a.kind == "mesh");
        const double v = meshVolume(a.mesh);
        const double expect = op == BooleanOp::Union      ? vBox + vCyl - vInside
                              : op == BooleanOp::Subtract ? vBox - vInside
                                                          : vInside;
        CHECK_NEAR(v, expect, expect * 1e-6);
    }
}

TEST_CASE("holes, pockets, channels, bosses and the knife cut closed solids") {
    Body b = makeBody("Plate", "box", {0, 0, 0, 100, 100, 100});
    std::string error;
    double v = meshVolume(b.mesh);

    // A through hole removes exactly the 48-sided prism of the cutter.
    REQUIRE(drillHole(b, {50, 50, 100}, {0, 0, 1}, 20, 0, &error));
    CHECK_NEAR(meshVolume(b.mesh), v - polygonArea(10, 48) * 100, 1.0);
    CHECK(openEdgeCount(b.mesh) == 0);
    v = meshVolume(b.mesh);

    // A box pocket: exactly width x length x depth.
    REQUIRE(cutPocket(b, {25, 50, 100}, {0, 0, 1}, PocketShape::Box, 20, 40, 10, 0, &error));
    CHECK_NEAR(meshVolume(b.mesh), v - 20 * 40 * 10, 0.5);
    CHECK(openEdgeCount(b.mesh) == 0);
    v = meshVolume(b.mesh);

    // A curved channel carves material along its path.
    REQUIRE(cutChannel(b, {{10, 10, 100}, {50, 20, 100}, {90, 10, 100}}, {{0, 0, 1}, {0, 0, 1}, {0, 0, 1}}, 6, 3, &error));
    CHECK(meshVolume(b.mesh) < v - 500);
    CHECK(openEdgeCount(b.mesh) == 0);
    v = meshVolume(b.mesh);

    // The knife keeps 80% of the height; the cut face is capped.
    const double below = volumeBeyondPlane(worldMesh(b), {0, 0, 20}, {0, 0, -1});
    REQUIRE(trimByPlane(b, {0, 0, 20}, {0, 0, -1}, &error));
    CHECK_NEAR(meshVolume(b.mesh), v - below, 1.0);
    CHECK(b.bounds().min.z > 19.99f);
    CHECK(openEdgeCount(b.mesh) == 0);
    v = meshVolume(b.mesh);

    // A boss adds its cylinder above the surface.
    REQUIRE(addBoss(b, {80, 80, 100}, {0, 0, 1}, BossShape::Cylinder, 16, 20, &error));
    CHECK_NEAR(meshVolume(b.mesh), v + polygonArea(8, 48) * 20, 1.0);
    CHECK(openEdgeCount(b.mesh) == 0);

    // Cuts that miss say so and change nothing.
    const MeshData before = b.mesh;
    CHECK(!drillHole(b, {-300, 0, 300}, {0, 0, 1}, 10, 0, &error));
    CHECK(error.find("does not touch") != std::string::npos);
    CHECK(b.mesh.indices.size() == before.indices.size());
}

TEST_CASE("cutters on round bodies and blind holes stay closed") {
    Body s = makeBody("Ball", "sphere", {50});
    std::string error;
    REQUIRE(drillHole(s, {0, 0, 100}, {0, 0, 1}, 10, 30, &error)); // blind, with a drill point
    CHECK(openEdgeCount(s.mesh) == 0);
    REQUIRE(drillHole(s, {3, 2, 100}, {0, 0, 1}, 6, 0, &error));
    CHECK(openEdgeCount(s.mesh) == 0);
    Body c = makeBody("Rod", "cylinder", {30, 70});
    REQUIRE(cutPocket(c, {0, 30, 35}, {0, 1, 0}, PocketShape::Ball, 12, 12, 6, 0, &error));
    CHECK(openEdgeCount(c.mesh) == 0);
}

TEST_CASE("brushes add, remove, push, pinch and smooth clay") {
    for (const SculptBrush brush : sculptBrushes()) {
        Body b = makeBody("Clay", "box", {0, 0, 0, 100, 100, 100});
        const double v0 = meshVolume(b.mesh);
        SculptSettings s;
        s.brush = brush;
        s.radius = 25;
        s.strength = 0.8f;
        sculptStroke(b, {{30, 40, 100}, {70, 60, 100}}, s, {0, 0, 15});
        CHECK(openEdgeCount(b.mesh) == 0);
        CHECK(b.kind == "mesh");
        const double v = meshVolume(b.mesh);
        switch (brush) {
        case SculptBrush::Draw:
        case SculptBrush::Clay:
        case SculptBrush::Inflate: CHECK(v > v0 + 100); break;
        case SculptBrush::Carve:
        case SculptBrush::Crease: CHECK(v < v0 - 100); break;
        case SculptBrush::Grab: CHECK(b.bounds().max.z > 101.0f); break;
        default: CHECK(std::fabs(v - v0) < v0 * 0.01); break; // reshapes without much volume change
        }
        // The flat 12-triangle block was refined under the brush (except a grab of no vertices).
        if (brush != SculptBrush::Grab) CHECK(b.mesh.indices.size() / 3 > 100);
    }

    // Inverting a brush reverses it.
    Body b = makeBody("Clay", "box", {0, 0, 0, 100, 100, 100});
    SculptSettings s;
    s.brush = SculptBrush::Draw;
    s.invert = true;
    sculptStroke(b, {{50, 50, 100}}, s);
    CHECK(meshVolume(b.mesh) < 1e6);
    CHECK(b.bounds().max.z <= 100.001f);
}

TEST_CASE("symmetry mirrors a stroke across the body's centre") {
    Body b = makeBody("Clay", "box", {0, 0, 0, 100, 100, 100});
    SculptSettings s;
    s.brush = SculptBrush::Draw;
    s.radius = 15;
    s.strength = 1.0f;
    s.symmetryX = true;
    sculptStroke(b, {{20, 50, 100}}, s);
    float left = 0, right = 0;
    for (const Vertex& v : b.mesh.vertices) {
        if (std::fabs(v.position.y - 50) > 4) continue;
        if (std::fabs(v.position.x - 20) < 4) left = std::max(left, v.position.z);
        if (std::fabs(v.position.x - 80) < 4) right = std::max(right, v.position.z);
    }
    CHECK(left > 100.5f);
    CHECK_NEAR(left, right, 0.05);
}

TEST_CASE("sculpt and boolean commands, with undo") {
    Document d;
    REQUIRE(run(d, "BOX 100 100 40 @0,0,0").ok);
    const std::string name = d.scene.bodies().front().name;
    const double v0 = meshVolume(d.scene.bodies().front().mesh);

    auto r = run(d, "HOLE \"" + name + "\" AT 0,0,40 DIAMETER 10");
    REQUIRE(r.ok);
    CHECK(r.lines.back().find("closed") != std::string::npos);
    CHECK(meshVolume(d.scene.bodies().front().mesh) < v0 - 3000);

    REQUIRE(run(d, "SELECT \"" + name + "\"").ok);
    REQUIRE(run(d, "POCKET AT 25,25,40 BOX WIDTH 20 LENGTH 10 DEPTH 5").ok);
    REQUIRE(run(d, "CHANNEL PATH -40,-30,40 0,-35,40 40,-30,40 WIDTH 4").ok);
    REQUIRE(run(d, "BOSS AT -25,25,40 CYLINDER DIAMETER 12 HEIGHT 8").ok);
    REQUIRE(run(d, "SCULPT CLAY AT -20,0,40 20,0,40 RADIUS 10 STRENGTH 0.5 SYMMETRY").ok);
    REQUIRE(run(d, "TRIM PLANE 0,0,10 NORMAL 0,0,-1").ok);
    CHECK(d.scene.bodies().front().bounds().min.z > 9.99f);
    CHECK(openEdgeCount(d.scene.bodies().front().mesh) == 0);
    CHECK(!run(d, "SCULPT AT 0,0,40").ok);         // no brush
    CHECK(!run(d, "HOLE AT 900,0,40 D 5").ok);      // misses: nothing changes
    CHECK(run(d, "SCULPT LIST").lines.size() == sculptBrushes().size() + 1);

    // Each edit can be undone, back to the plain block.
    const double vTrim = meshVolume(d.scene.bodies().front().mesh);
    REQUIRE(run(d, "UNDO").ok);
    CHECK(meshVolume(d.scene.bodies().front().mesh) > vTrim);
    for (int i = 0; i < 5; ++i) REQUIRE(run(d, "UNDO").ok);
    CHECK_NEAR(meshVolume(d.scene.bodies().front().mesh), v0, 1e-3);
    CHECK(!run(d, "UNDO").ok);

    // Two bodies: the tool is used up unless KEEP.
    REQUIRE(run(d, "CYLINDER 10 80 @0,0,-20").ok);
    const std::string tool = d.scene.bodies().back().name;
    REQUIRE(run(d, "BOOLEAN \"" + name + "\" SUBTRACT \"" + tool + "\" KEEP").ok);
    CHECK(d.scene.bodies().size() == 2);
    REQUIRE(run(d, "UNDO").ok);
    REQUIRE(run(d, "BOOLEAN \"" + name + "\" SUBTRACT \"" + tool + "\"").ok);
    CHECK(d.scene.bodies().size() == 1);
    CHECK(openEdgeCount(d.scene.bodies().front().mesh) == 0);
}

TEST_CASE("a quoted name is a name even when it reads like a keyword") {
    Document d;
    REQUIRE(run(d, "BOX 40 40 40 @0,0,0").ok);
    REQUIRE(run(d, "BOX 40 40 40 @100,0,0").ok);
    d.scene.bodies()[0].name = "Ball";
    d.scene.bodies()[1].name = "Arm, \"left\"";
    REQUIRE(run(d, "SELECT \"Ball\"").ok); // the other body is not the selection
    const std::size_t before = d.scene.bodies()[1].mesh.indices.size();
    REQUIRE(run(d, R"(HOLE "Arm, \"left\"" AT 100,0,40 DIAMETER 8)").ok);
    CHECK(d.scene.bodies()[1].mesh.indices.size() != before);
    REQUIRE(run(d, "SHELL \"Ball\" THICKNESS 3").ok);
    CHECK(openEdgeCount(d.scene.bodies()[0].mesh) == 0);
    CHECK(meshVolume(d.scene.bodies()[0].mesh) < 40.0 * 40 * 40 * 0.5);
    REQUIRE(run(d, R"(BOOLEAN "Ball" UNION "Arm, \"left\"" KEEP)").ok);
    CHECK(d.scene.bodies().size() == 2);
}

#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Scene.hpp"
#include "core/Sculpt.hpp"

#include <algorithm>
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

TEST_CASE("booleans of bodies that touch face to face, and far from the origin") {
    // Side by side along x: one shared face, opposite normals.
    for (const BooleanOp op : {BooleanOp::Union, BooleanOp::Intersect}) {
        Body a = makeBody("A", "box", {0, 0, 0, 10, 10, 10});
        const Body b = makeBody("B", "box", {10, 0, 0, 20, 10, 10});
        std::string error;
        const bool ok = bodyBoolean(a, b, op, &error);
        if (op == BooleanOp::Union) {
            REQUIRE(ok);
            CHECK_NEAR(meshVolume(a.mesh), 2000.0, 1e-3);
            CHECK(openEdgeCount(a.mesh) == 0);
        } else {
            CHECK(!ok || std::fabs(meshVolume(a.mesh)) < 1e-3); // touching solids share no volume
        }
    }
    // A face shared on the same side: a slab cut off the end of a block.
    Body a = makeBody("A", "box", {0, 0, 0, 10, 10, 10});
    const Body b = makeBody("B", "box", {0, 0, 0, 4, 10, 10});
    std::string error;
    REQUIRE(bodyBoolean(a, b, BooleanOp::Subtract, &error));
    CHECK_NEAR(meshVolume(a.mesh), 600.0, 600.0 * 1e-4); // partly shared side faces: the nudged retry
    CHECK(openEdgeCount(a.mesh) == 0);

    // Far out, a boolean is as exact as at the origin (it runs in the body's frame).
    Body far = makeBody("F", "box", {0, 0, 0, 10, 10, 10});
    Body other = makeBody("G", "box", {5, 5, 5, 15, 15, 15});
    far.offset = other.offset = {1e5f, 1e5f, 0};
    REQUIRE(bodyBoolean(far, other, BooleanOp::Intersect, &error));
    CHECK_NEAR(meshVolume(far.mesh), 125.0, 1e-3);

    // A channel that doubles back within its own width is refused, not half-cut.
    Body plate = makeBody("P", "box", {0, 0, 0, 100, 100, 20});
    CHECK(!cutChannel(plate, {{20, 50, 20}, {80, 50, 20}, {80, 54, 20}, {20, 54, 20}},
                      {{0, 0, 1}, {0, 0, 1}, {0, 0, 1}, {0, 0, 1}}, 6, 3, &error));
    CHECK(plate.mesh.indices.size() == 36);
}

TEST_CASE("undo keeps later work and restores motions") {
    Document d;
    REQUIRE(run(d, "BOX 60 40 10 @0,0,0").ok);
    REQUIRE(run(d, "BOX 60 40 10 @70,0,0").ok);
    const std::size_t motions = d.scene.motions().size();
    REQUIRE(run(d, "JOINT \"Block 2\" TO \"Block 1\" AT 35,0,5 AXIS Y RANGE -30 30").ok);
    CHECK(d.scene.motions().size() == motions + 1);
    CHECK(!run(d, "JOINT \"Block 2\" TO \"Block 2\" AT 35,0,5").ok); // not on itself
    // A move after the joint survives its undo; the swing goes with the pin.
    d.scene.bodies()[0].offset.x += 5;
    REQUIRE(run(d, "UNDO").ok);
    CHECK(d.scene.motions().size() == motions);
    CHECK(d.scene.bodies().size() == 2);
    CHECK_NEAR(d.scene.bodies()[0].offset.x, 5, 1e-4);

    // A body added after an edit is not swept away by its undo.
    REQUIRE(run(d, "HOLE \"Block 1\" AT 5,0,10 DIAMETER 6").ok);
    REQUIRE(run(d, "SPHERE 10 @300,0,0").ok);
    const auto r = run(d, "UNDO");
    CHECK(!r.ok);
    CHECK(d.scene.bodies().size() == 3);
}

TEST_CASE("refining the union of round primitives keeps it closed") {
    // Booleans leave caps (a corner on the far edge) and needles (two corners a hair
    // apart); refined, they used to pinch the surface into open edges.
    Document d;
    for (const char* c : {"SPHERE 82 @0,5,253", "SPHERE 75 @0,25,270", "SPHERE 55 @0,-25,195", "CYLINDER 45 170 @0,15,100",
                          "BOOLEAN \"Sphere 1\" UNION \"Sphere 2\"", "BOOLEAN \"Sphere 1\" UNION \"Sphere 3\"",
                          "BOOLEAN \"Sphere 1\" UNION \"Cylinder 4\""})
        REQUIRE(run(d, c).ok);
    const auto r = run(d, "SCULPT \"Sphere 1\" REFINE 10");
    REQUIRE(r.ok);
    CHECK(r.lines[0].find(", closed") != std::string::npos);
    CHECK(openEdgeCount(d.scene.bodies()[0].mesh) == 0);
}

TEST_CASE("RUN sculpts the head bust example from a ball") {
    Document d;
    const auto r = run(d, std::string("RUN ") + SICDM_SOURCE_DIR + "/docs/examples/head-bust.txt");
    REQUIRE(r.ok);
    REQUIRE(d.scene.bodies().size() == 1);
    const Body& bust = d.scene.bodies()[0];
    CHECK(openEdgeCount(bust.mesh) == 0);
    const AABB box = bust.bounds();
    CHECK_NEAR(box.min.z, 40.0, 1e-3); // cut flat underneath
    CHECK_NEAR(box.max.z, 420.0, 5.0); // the crown
    CHECK(box.max.x - box.min.x > 350.0f); // shoulders

    const SculptMesh sm(bust);
    CHECK(sm.openEdges() == 0);
    // The front of the face (y, looking along +y), and the sides of the head (x).
    auto front = [&](float x, float z) {
        const auto h = sm.raycast({{x, -500.0f, z}, {0, 1, 0}});
        return h ? h->y : 1e9f;
    };
    auto side = [&](float dir, float y, float z) {
        const auto h = sm.raycast({{dir * 500.0f, y, z}, {-dir, 0, 0}});
        return h ? h->x : 1e9f;
    };
    CHECK(front(0, 290) < front(0, 260) - 12.0f); // the nose stands out over the lip
    CHECK(front(0, 254) > front(0, 260) + 0.8f);   // the mouth is a line between the lips
    CHECK(front(0, 254) > front(0, 247) + 0.8f);
    CHECK(front(30, 318) > front(30, 340) + 3.0f); // the eyes sit under the brow
    CHECK(side(1, 25, 310) > side(1, 25, 350) + 5.0f); // the ears stand out from the skull
    CHECK_NEAR(side(1, 25, 310), -side(-1, 25, 310), 1.0);
    // Mirrored strokes: the head is the same on both sides, within the remeshing.
    float worst = 0.0f;
    for (int x = 4; x <= 70; x += 6)
        for (int z = 228; z <= 410; z += 7) { // above the chin, where rays graze the jaw
            const float a = front(static_cast<float>(x), static_cast<float>(z)), b = front(static_cast<float>(-x), static_cast<float>(z));
            if (a < 1e8f && b < 1e8f) worst = std::max(worst, std::fabs(a - b));
        }
    CHECK(worst < 1.0f);

    CHECK(!run(d, "RUN /no/such/script.txt").ok);
}

TEST_CASE("REMESH fuses overlapping parts into one closed solid, filleted where they meet") {
    Document d;
    REQUIRE(run(d, "SPHERE 30 @0,0,0").ok);
    REQUIRE(run(d, "SPHERE 30 @45,0,0").ok); // overlapping: centres 45 apart
    const double v1 = meshVolume(worldMesh(d.scene.bodies()[0]));
    auto r = run(d, "REMESH \"Sphere 1\" \"Sphere 2\" VOXEL 1 BLEND 12");
    REQUIRE(r.ok);
    CHECK(r.lines[0].find(", closed") != std::string::npos);
    REQUIRE(d.scene.bodies().size() == 1); // the other ball is used up
    const Body& b = d.scene.bodies()[0];
    CHECK(openEdgeCount(b.mesh) == 0);
    const double v = meshVolume(worldMesh(b));
    CHECK(v > v1 * 1.5);
    CHECK(v < v1 * 2.0);
    // At the waist between the balls (x = 22.5) the plain union is 18.5 wide; the
    // fillet fills it out.
    const SculptMesh sm(b);
    const auto top = sm.raycast({{22.5f, 0.0f, 200.0f}, {0, 0, -1}});
    REQUIRE(top.has_value());
    CHECK(top->z > 30.0f + 19.8f + 1.0f);
    // Far from the join, each surface stays where it was.
    const auto left = sm.raycast({{-200.0f, 0.0f, 30.0f}, {1, 0, 0}});
    REQUIRE(left.has_value());
    CHECK_NEAR(left->x, -30.0, 0.6);

    // One faceted body: the facets round off, its size stays.
    REQUIRE(run(d, "BOX 40 40 40 @100,0,0").ok);
    const std::string block = d.scene.bodies().back().name;
    REQUIRE(run(d, "REMESH \"" + block + "\" VOXEL 1 SMOOTH 1").ok);
    const Body& k = d.scene.bodies().back();
    CHECK(openEdgeCount(k.mesh) == 0);
    CHECK_NEAR(meshVolume(worldMesh(k)), 64000.0, 64000.0 * 0.04);
    CHECK(!run(d, "REMESH \"" + block + "\" VOXEL 0.001").ok); // far too many voxels
    CHECK(!run(d, "REMESH \"" + block + "\" \"Nobody\"").ok);
}

TEST_CASE("SCALE stretches along x, y, z and ROTATE orients a body") {
    Document d;
    REQUIRE(run(d, "SPHERE 10 @0,0,0").ok);
    REQUIRE(run(d, "SCALE \"Sphere 1\" 2,1,0.5").ok);
    AABB box = d.scene.bodies()[0].bounds();
    CHECK_NEAR(box.max.x - box.min.x, 40.0, 0.1);
    CHECK_NEAR(box.max.y - box.min.y, 20.0, 0.1);
    CHECK_NEAR(box.max.z - box.min.z, 10.0, 0.1);
    CHECK_NEAR(box.min.z, 0.0, 1e-3); // about its base point
    REQUIRE(run(d, "UNDO").ok);
    box = d.scene.bodies()[0].bounds();
    CHECK_NEAR(box.max.x - box.min.x, 20.0, 0.1);
    REQUIRE(run(d, "SCALE \"Sphere 1\" 2,1,0.5").ok);
    REQUIRE(run(d, "ROTATE \"Sphere 1\" 0,0,90").ok);
    box = d.scene.bodies()[0].bounds();
    CHECK_NEAR(box.max.x - box.min.x, 20.0, 0.1);
    CHECK_NEAR(box.max.y - box.min.y, 40.0, 0.1);
    CHECK(!run(d, "SCALE \"Sphere 1\" 1,0,1").ok);
}

TEST_CASE("a stroke on a dense mesh only touches the surface it reaches") {
    Body b = makeBody("Ball", "sphere", {40});
    sculptRefine(b, 1.5f);
    const SculptMesh before(b);
    SculptSettings s;
    s.brush = SculptBrush::Draw;
    s.radius = 4;
    s.strength = 1;
    sculptStroke(b, {{0, -40, 40}, {0, -38, 55}}, s);
    CHECK(openEdgeCount(b.mesh) == 0);
    const SculptMesh after(b);
    CHECK(after.openEdges() == 0);
    // Raised under the stroke, untouched beyond twice its radius from it.
    const auto hit = after.raycast({{0, -200, 47}, {0, 1, 0}});
    REQUIRE(hit.has_value());
    CHECK(hit->y < -40.1f);
    float moved = 0.0f;
    for (const Vec3& p : before.positions()) {
        if (std::fabs(p.z - 47.5f) < 20.0f && p.y < -20.0f) continue; // near the stroke
        moved = std::max(moved, std::fabs(length(after.closestPoint(p) - p)));
    }
    CHECK(moved < 1e-3f);
}

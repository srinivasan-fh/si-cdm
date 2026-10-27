#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/FileIO.hpp"
#include "core/Scene.hpp"
#include "core/ImageWrite.hpp"
#include "core/Multires.hpp"
#include "core/Sculpt.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
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
        case SculptBrush::ClayStrips:
        case SculptBrush::Layer:
        case SculptBrush::Blob:
        case SculptBrush::Inflate: CHECK(v > v0 + 100); break;
        case SculptBrush::Carve:
        case SculptBrush::Crease: CHECK(v < v0 - 100); break;
        case SculptBrush::Grab: CHECK(b.bounds().max.z > 101.0f); break;
        case SculptBrush::Mask:
        case SculptBrush::Paint: CHECK_NEAR(v, v0, 1e-3); break; // paints the mask or colour, moves nothing
        default: CHECK(std::fabs(v - v0) < v0 * 0.01); break; // reshapes without much volume change
        }
        // The flat 12-triangle block was refined under the brush (except a grab of no
        // vertices, and the mask and paint, which need no new triangles).
        if (brush != SculptBrush::Grab && brush != SculptBrush::Mask && brush != SculptBrush::Paint) CHECK(b.mesh.indices.size() / 3 > 100);
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
    REQUIRE(run(d, "UNDO").ok); // the block itself
    CHECK(d.scene.bodies().empty());
    CHECK(!run(d, "UNDO").ok);
    REQUIRE(run(d, "REDO").ok);
    REQUIRE(d.scene.bodies().size() == 1);
    CHECK_NEAR(meshVolume(d.scene.bodies().front().mesh), v0, 1e-3);

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

    // A body added by a command is an undo step of its own; one added outside the
    // commands (no snapshot) is not swept away by the undo of an edit before it.
    REQUIRE(run(d, "HOLE \"Block 1\" AT 5,0,10 DIAMETER 6").ok);
    REQUIRE(run(d, "SPHERE 10 @300,0,0").ok);
    REQUIRE(run(d, "UNDO").ok);
    CHECK(d.scene.bodies().size() == 2);
    d.scene.add(d.scene.bodies()[1]);
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
    const SurfaceIndex nearest(after);
    for (const Vec3& p : before.positions()) {
        if (std::fabs(p.z - 47.5f) < 20.0f && p.y < -20.0f) continue; // near the stroke
        moved = std::max(moved, std::fabs(length(nearest.closestPoint(p) - p)));
    }
    CHECK(moved < 1e-3f);
}

namespace {

// A dense ball to sculpt on (about 1.5 mm edges).
Body denseBall(float r = 40) {
    Body b = makeBody("Ball", "sphere", {r});
    sculptRefine(b, 1.5f);
    return b;
}

// Surface height along a ray from outside towards the ball's centre (0, 0, r),
// tilted a little off the sphere's rings and meridians (a ray along an edge can slip
// between its triangles).
float radiusAt(const Body& b, const Vec3& dir, float r = 40) {
    const SculptMesh sm(b);
    const Vec3 c{0, 0, r};
    const Vec3 d = normalize(normalize(dir) + Vec3{0.0123f, 0.0071f, 0.0173f});
    const auto h = sm.raycast({c + d * 400.0f, -d});
    return h ? length(*h - c) : 0.0f;
}

} // namespace

TEST_CASE("the mask holds the surface still, and can be inverted, blurred, grown, saved") {
    Body b = denseBall();
    SculptSettings s;
    s.brush = SculptBrush::Mask;
    s.radius = 15;
    s.strength = 1;
    sculptStroke(b, {{0, -40, 40}}, s); // mask the front
    CHECK(maskedFraction(b) > 0.005f);
    s.brush = SculptBrush::Draw;
    s.radius = 30;
    sculptStroke(b, {{0, -40, 40}, {0, -40, 55}}, s); // a stroke over the mask and past it
    CHECK(radiusAt(b, {0, -1, 0}) < 40.2f);              // held
    CHECK(radiusAt(b, {0, -1, 0.7f}) > 40.5f);           // raised beyond the mask
    CHECK(openEdgeCount(b.mesh) == 0);

    const float masked = maskedFraction(b);
    maskBody(b, MaskOp::Grow);
    CHECK(maskedFraction(b) > masked);
    maskBody(b, MaskOp::Shrink);
    maskBody(b, MaskOp::Smooth);
    CHECK(maskedFraction(b) > 0.0f);
    maskBody(b, MaskOp::Invert);
    CHECK(maskedFraction(b) > 0.9f);
    maskBody(b, MaskOp::Clear);
    CHECK(maskedFraction(b) == 0.0f);

    // The mask is saved with the project.
    Document d;
    REQUIRE(run(d, "SPHERE 20 @0,0,0").ok);
    REQUIRE(run(d, "SCULPT \"Sphere 1\" REFINE 2").ok);
    REQUIRE(run(d, "SCULPT \"Sphere 1\" MASK AT 0,-20,20 RADIUS 8 STRENGTH 1").ok);
    REQUIRE(run(d, "MASK \"Sphere 1\" SMOOTH 2").ok);
    const float before = maskedFraction(d.scene.bodies()[0]);
    CHECK(before > 0.0f);
    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_mask_test.sicdm").string();
    std::string err;
    REQUIRE(saveProject(d.scene, d.state, path, &err));
    Scene loaded;
    ProjectState st;
    REQUIRE(loadProject(loaded, st, path, &err));
    CHECK_NEAR(maskedFraction(loaded.bodies()[0]), before, 1e-4);
    std::filesystem::remove(path);
    CHECK(!run(d, "MASK \"Sphere 1\" SIDEWAYS").ok);
}

TEST_CASE("symmetry on any axis, radial symmetry and falloffs") {
    SculptSettings s;
    s.brush = SculptBrush::Draw;
    s.radius = 10;
    s.strength = 1;
    // Four bumps round the ball's vertical axis from one dab.
    Body b = denseBall();
    s.radial = 4;
    sculptStroke(b, {{40, 0, 40}}, s);
    for (const Vec3& d : {Vec3{1, 0, 0}, Vec3{0, 1, 0}, Vec3{-1, 0, 0}, Vec3{0, -1, 0}}) CHECK(radiusAt(b, d) > 40.4f);
    CHECK(radiusAt(b, {1, 1, 0}) < 40.2f); // between them
    // Mirrored across y = centre and z = centre at once: four bumps.
    Body c = denseBall();
    s.radial = 0;
    s.symmetryY = s.symmetryZ = true;
    sculptStroke(c, {{0, 28.28f, 68.28f}}, s);
    CHECK(radiusAt(c, {0, 1, 1}) > 40.4f);
    CHECK(radiusAt(c, {0, -1, 1}) > 40.4f);
    CHECK(radiusAt(c, {0, 1, -1}) > 40.4f);
    CHECK(radiusAt(c, {0, -1, -1}) > 40.4f);
    // A flat falloff lifts the rim of the dab as much as its centre; a sharp one hardly at all.
    auto rimLift = [&](SculptFalloff f) {
        Body e = denseBall();
        SculptSettings t;
        t.brush = SculptBrush::Draw;
        t.radius = 12;
        t.strength = 1;
        t.refine = false;
        t.falloff = f;
        sculptStroke(e, {{0, -40, 40}}, t);
        return radiusAt(e, {std::sin(0.2f), -std::cos(0.2f), 0}) - 40.0f; // 8 mm out from the centre
    };
    CHECK(rimLift(SculptFalloff::Constant) > rimLift(SculptFalloff::Smooth));
    CHECK(rimLift(SculptFalloff::Smooth) > rimLift(SculptFalloff::Sharp));
    CHECK(sculptFalloffFromKey("sphere") == SculptFalloff::Sphere);
    Document d;
    REQUIRE(run(d, "SPHERE 40 @0,0,0").ok);
    CHECK(run(d, "SCULPT \"Sphere 1\" DRAW AT 40,0,40 RADIUS 8 RADIAL 6 Z FALLOFF SPHERE SYMMETRY XY").ok);
    CHECK(!run(d, "SCULPT \"Sphere 1\" DRAW AT 40,0,40 FALLOFF WOBBLY").ok);
    CHECK(!run(d, "SCULPT \"Sphere 1\" DRAW AT 40,0,40 RADIAL 0").ok);
}

TEST_CASE("layer, clay strips, fill, blob, snake hook and nudge") {
    SculptSettings s;
    s.radius = 12;
    s.strength = 1;
    // Layer builds to its height once a stroke, however often it passes.
    Body b = denseBall();
    s.brush = SculptBrush::Layer;
    sculptStroke(b, {{0, -40, 34}, {0, -40, 46}, {0, -40, 34}, {0, -40, 46}, {0, -40, 34}}, s);
    const float lift = radiusAt(b, {0, -1, 0}) - 40.0f;
    CHECK(lift > 1.0f);
    CHECK(lift < 0.25f * 12.0f + 0.3f);
    // Snake Hook pulls the surface out along the path (off the surface).
    Body c = denseBall();
    s.brush = SculptBrush::SnakeHook;
    s.radius = 8;
    sculptStroke(c, {{0, -40, 40}, {0, -55, 40}, {0, -70, 40}}, s);
    CHECK(c.bounds().min.y < -50.0f);
    CHECK(openEdgeCount(c.mesh) == 0);
    // Nudge slides the surface along itself: little volume change, but it moves.
    Body e = denseBall();
    const double ve = meshVolume(worldMesh(e));
    s.brush = SculptBrush::Nudge;
    s.radius = 10;
    sculptStroke(e, {{0, -40, 35}, {0, -40, 45}}, s);
    CHECK(std::fabs(meshVolume(worldMesh(e)) - ve) < ve * 0.01);
    // Clay strips and blob add; fill raises a dent back up.
    for (const SculptBrush br : {SculptBrush::ClayStrips, SculptBrush::Blob}) {
        Body f = denseBall();
        const double v0 = meshVolume(worldMesh(f));
        s.brush = br;
        sculptStroke(f, {{0, -40, 35}, {0, -40, 45}}, s);
        CHECK(meshVolume(worldMesh(f)) > v0 + 50);
    }
    // Fill raises a dent in a flat top back up towards the plane around it.
    Body g = makeBody("Slab", "box", {0, 0, 0, 60, 60, 10});
    sculptRefine(g, 1.5f);
    auto top = [&] {
        const auto h = SculptMesh(g).raycast({{30.13f, 30.07f, 100.0f}, {0, 0, -1}});
        return h ? h->z : 0.0f;
    };
    s.brush = SculptBrush::Draw;
    s.invert = true;
    s.radius = 6;
    sculptStroke(g, {{30, 30, 10}, {30.5f, 30, 10}, {30, 30, 10}}, s); // a dent
    const float dent = top();
    CHECK(dent < 9.5f);
    s.invert = false;
    s.brush = SculptBrush::Fill;
    s.radius = 14;
    sculptStroke(g, {{30, 30, 10}}, s);
    CHECK(top() > dent + 0.2f);
    CHECK(top() <= 10.01f);
}

TEST_CASE("SUBDIVIDE, SYMMETRIZE and DECIMATE keep bodies closed") {
    Document d;
    REQUIRE(run(d, "BOX 40 40 40 @0,0,0").ok);
    auto r = run(d, "SUBDIVIDE \"Block 1\" LEVELS 2");
    REQUIRE(r.ok);
    const Body& cube = d.scene.bodies()[0];
    CHECK(cube.mesh.indices.size() / 3 == 12 * 16);
    CHECK(openEdgeCount(cube.mesh) == 0);
    const double vc = meshVolume(worldMesh(cube));
    CHECK(vc < 64000.0);         // the corners round off
    CHECK(vc > 64000.0 * 0.4);

    // A bump on +x only; symmetrized it is on both sides.
    REQUIRE(run(d, "SPHERE 30 @200,0,0").ok);
    REQUIRE(run(d, "SCULPT \"Sphere 2\" REFINE 2").ok);
    REQUIRE(run(d, "SCULPT \"Sphere 2\" DRAW AT 230,0,30 RADIUS 10 STRENGTH 1").ok);
    REQUIRE(run(d, "SCULPT \"Sphere 2\" DRAW AT 230,0,30 RADIUS 10 STRENGTH 1").ok);
    AABB box = d.scene.bodies()[1].bounds();
    CHECK(box.max.x - 200.0f > 200.0f - box.min.x + 1.0f);
    REQUIRE(run(d, "SYMMETRIZE \"Sphere 2\" X").ok);
    const Body& sym = d.scene.bodies()[1];
    box = sym.bounds();
    CHECK_NEAR(box.max.x - 200.0f, 200.0f - box.min.x, 0.05);
    CHECK(box.max.x - 200.0f > 31.0f);
    CHECK(openEdgeCount(sym.mesh) == 0);

    const double v0 = meshVolume(worldMesh(sym));
    REQUIRE(run(d, "DECIMATE \"Sphere 2\" TO 3000").ok);
    const Body& dec = d.scene.bodies()[1];
    CHECK(dec.mesh.indices.size() / 3 <= 3400);
    CHECK(openEdgeCount(dec.mesh) == 0);
    CHECK_NEAR(meshVolume(worldMesh(dec)), v0, v0 * 0.03);
    CHECK(!run(d, "DECIMATE \"Sphere 2\" TO 2").ok);
    CHECK(!run(d, "SUBDIVIDE \"Nobody\"").ok);
}

TEST_CASE("REDO puts back what UNDO took, until something else changes") {
    Document d;
    REQUIRE(run(d, "SPHERE 20 @0,0,0").ok);
    REQUIRE(run(d, "SCULPT \"Sphere 1\" DRAW AT 0,-20,20 RADIUS 8 STRENGTH 1").ok);
    const double v1 = meshVolume(worldMesh(d.scene.bodies()[0]));
    REQUIRE(run(d, "SCULPT \"Sphere 1\" DRAW AT 0,20,20 RADIUS 8 STRENGTH 1").ok);
    const double v2 = meshVolume(worldMesh(d.scene.bodies()[0]));
    REQUIRE(run(d, "UNDO").ok);
    REQUIRE(run(d, "UNDO").ok);
    REQUIRE(run(d, "REDO").ok);
    CHECK_NEAR(meshVolume(worldMesh(d.scene.bodies()[0])), v1, 1e-6);
    REQUIRE(run(d, "REDO").ok);
    CHECK_NEAR(meshVolume(worldMesh(d.scene.bodies()[0])), v2, 1e-6);
    CHECK(!run(d, "REDO").ok); // nothing left
    REQUIRE(run(d, "UNDO").ok); // and undoable again
    CHECK_NEAR(meshVolume(worldMesh(d.scene.bodies()[0])), v1, 1e-6);
    // A new edit drops what was undone.
    REQUIRE(run(d, "SCULPT \"Sphere 1\" SMOOTH AT 0,-20,20 RADIUS 8").ok);
    CHECK(!run(d, "REDO").ok);
    // A body added since the undo: redo is refused, not mixed in.
    REQUIRE(run(d, "UNDO").ok);
    REQUIRE(run(d, "BOX 10 10 10 @100,0,0").ok);
    CHECK(!run(d, "REDO").ok);
}

TEST_CASE("a union of solids that only touch face to face is one shell, with no wall inside") {
    Document d;
    REQUIRE(run(d, "BOX 10 10 30 @0,0,0").ok);
    REQUIRE(run(d, "BOX 50 10 6 @0,0,30").ok); // stood on top: its underside partly on the post's top
    REQUIRE(run(d, "BOOLEAN \"Block 1\" UNION \"Block 2\"").ok);
    const Body& t = d.scene.bodies()[0];
    CHECK(openEdgeCount(t.mesh) == 0);
    CHECK_NEAR(meshVolume(worldMesh(t)), 10.0 * 10 * 30 + 50.0 * 10 * 6, 0.5);
    // No upward face left at the join (the post's top), and only the arms' undersides face down there.
    const MeshData w = worldMesh(t);
    double up = 0, down = 0;
    for (std::size_t k = 0; k + 2 < w.indices.size(); k += 3) {
        const Vec3 a = w.vertices[w.indices[k]].position, b = w.vertices[w.indices[k + 1]].position, c = w.vertices[w.indices[k + 2]].position;
        if (std::fabs(a.z - 30) > 0.01f || std::fabs(b.z - 30) > 0.01f || std::fabs(c.z - 30) > 0.01f) continue;
        const Vec3 n = cross(b - a, c - a);
        (n.z > 0 ? up : down) += 0.5 * length(n);
    }
    CHECK(up < 0.5);
    CHECK_NEAR(down, 50.0 * 10 - 10.0 * 10, 0.5);
    // Apart, they stay two.
    REQUIRE(run(d, "BOX 10 10 10 @100,0,0").ok);
    REQUIRE(run(d, "BOX 10 10 10 @100,0,20").ok);
    REQUIRE(run(d, "BOOLEAN \"Block 3\" UNION \"Block 4\"").ok);
    CHECK_NEAR(meshVolume(worldMesh(d.scene.bodies().back())), 2000.0, 1e-3);
}

TEST_CASE("undo takes back primitives, imports and moves; a command that changes nothing leaves no step") {
    Document d;
    REQUIRE(run(d, "BOX 20 20 20 @0,0,0").ok);
    REQUIRE(run(d, "CYLINDER 5 30 @40,0,0").ok);
    REQUIRE(run(d, "MOVE 10,0,0 \"Block 1\"").ok);
    const Vec3 moved = d.scene.bodies()[0].offset;
    REQUIRE(run(d, "UNDO").ok); // the move
    CHECK(d.scene.bodies().size() == 2);
    CHECK_NEAR(d.scene.bodies()[0].offset.x, moved.x - 10.0f, 1e-4);
    REQUIRE(run(d, "REDO").ok);
    CHECK_NEAR(d.scene.bodies()[0].offset.x, moved.x, 1e-4);
    REQUIRE(run(d, "UNDO").ok);
    REQUIRE(run(d, "UNDO").ok); // the cylinder
    CHECK(d.scene.bodies().size() == 1);
    REQUIRE(run(d, "REDO").ok);
    CHECK(d.scene.bodies().size() == 2);

    // A failed command, or one that changes nothing, adds no undo step and keeps the redo list.
    REQUIRE(run(d, "UNDO").ok);
    const std::size_t depth = d.scene.undoDepth();
    CHECK(!run(d, "BOX x y z").ok);
    CHECK(!run(d, "HOLE AT 900,0,40 D 5").ok); // misses: it takes its own snapshot and puts it back
    CHECK(!run(d, "MOVE 1,0,0 \"Nothing\"").ok);
    CHECK(d.scene.undoDepth() == depth);
    REQUIRE(run(d, "REDO").ok); // still there
    CHECK(d.scene.bodies().size() == 2);

    // An edit that keeps its own snapshot (HOLE) is one step, not two.
    const std::size_t before = d.scene.undoDepth();
    REQUIRE(run(d, "HOLE \"Block 1\" AT 0,0,20 DIAMETER 4").ok);
    CHECK(d.scene.undoDepth() == before + 1);

    // Imports undo too.
    const std::string stl = (std::filesystem::temp_directory_path() / "sicdm-undo-import.stl").string();
    REQUIRE(run(d, "EXPORT \"" + stl + "\"").ok);
    const std::size_t n = d.scene.bodies().size();
    REQUIRE(run(d, "IMPORT \"" + stl + "\"").ok);
    CHECK(d.scene.bodies().size() > n);
    REQUIRE(run(d, "UNDO").ok);
    CHECK(d.scene.bodies().size() == n);
    std::filesystem::remove(stl);

    // The history follows: each undone step leaves it.
    Document e;
    REQUIRE(run(e, "BOX 10 10 10").ok);
    REQUIRE(run(e, "SPHERE 5 @30,0,0").ok);
    const std::size_t steps = e.state.history.size();
    REQUIRE(run(e, "UNDO").ok);
    CHECK(e.state.history.size() == steps - 1);
    REQUIRE(run(e, "REDO").ok);
    CHECK(e.state.history.size() == steps);
}

TEST_CASE("polypaint: the brush, fills, the mask holding it off, kept through dyntopo, saved and exported") {
    Vec3 c;
    CHECK(parseColorText("red", c) && c.x > 0.8f && c.y < 0.2f);
    CHECK(parseColorText("#00FF80", c) && c.x == 0.0f && c.y == 1.0f && std::fabs(c.z - 128.0f / 255.0f) < 1e-6f);
    CHECK(parseColorText("0.5,0.25,1", c) && c.y == 0.25f);
    CHECK(parseColorText("255,0,0", c) && c.x == 1.0f);
    CHECK(!parseColorText("nonsense", c));
    CHECK(!parseColorText("1,2", c));
    CHECK(unpackColor(packColor({1, 0.5f, 0})).y > 0.49f);

    Document d;
    REQUIRE(run(d, "SPHERE 20 @0,0,0").ok);
    const std::string name = d.scene.bodies().back().name;
    REQUIRE(run(d, "SCULPT \"" + name + "\" REFINE 3").ok);
    // A stroke of red across the top: painted there, not on the far side.
    REQUIRE(run(d, "SCULPT \"" + name + "\" PAINT AT -10,0,40 10,0,40 RADIUS 8 STRENGTH 1 COLOR red").ok);
    const Body* b = &d.scene.bodies().back();
    CHECK(b->kind == "mesh");
    int red = 0, plain = 0;
    for (const Vertex& v : b->mesh.vertices) {
        const Vec3 q = unpackColor(v.color);
        if (v.position.z > 38 && std::fabs(v.position.x) < 6) red += q.x > 0.7f && q.y < 0.3f;
        if (v.position.z < 5) plain += std::fabs(q.x - b->color.x) < 0.02f && std::fabs(q.z - b->color.z) < 0.02f;
    }
    CHECK(red > 5);
    CHECK(plain > 5);
    // Dynamic topology under a clay stroke keeps the paint where it was.
    REQUIRE(run(d, "SCULPT \"" + name + "\" CLAY AT 0,0,40 RADIUS 6 STRENGTH 0.5").ok);
    b = &d.scene.bodies().back();
    int stillRed = 0;
    for (const Vertex& v : b->mesh.vertices)
        if (v.position.z > 38 && std::fabs(v.position.x) < 4) stillRed += unpackColor(v.color).x > 0.6f && unpackColor(v.color).y < 0.4f;
    CHECK(stillRed > 5);
    // A fill held off by the mask: the masked top keeps its red.
    REQUIRE(run(d, "MASK \"" + name + "\" CLEAR").ok);
    REQUIRE(run(d, "SCULPT \"" + name + "\" MASK AT 0,0,40 RADIUS 6 STRENGTH 1").ok);
    REQUIRE(run(d, "PAINT \"" + name + "\" FILL blue").ok);
    b = &d.scene.bodies().back();
    int blue = 0, kept = 0;
    for (const Vertex& v : b->mesh.vertices) {
        const Vec3 q = unpackColor(v.color);
        if (v.mask >= 0.99f) kept += q.z < 0.5f;
        if (v.mask == 0.0f) blue += q.z > 0.8f;
    }
    CHECK(kept > 0);
    CHECK(blue > 100);
    // Saved with the project and read back.
    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_paint.sicdm").string();
    std::string err;
    REQUIRE(saveProject(d.scene, d.state, path, &err));
    Scene scene;
    ProjectState state;
    REQUIRE(loadProject(scene, state, path, &err));
    REQUIRE(scene.bodies().back().mesh.vertices.size() == b->mesh.vertices.size());
    bool same = true;
    for (std::size_t i = 0; i < b->mesh.vertices.size(); ++i) same = same && scene.bodies().back().mesh.vertices[i].color == b->mesh.vertices[i].color;
    CHECK(same);
    std::filesystem::remove(path);
    // OBJ: vertex colours after the positions, read back.
    const std::string obj = (std::filesystem::temp_directory_path() / "sicdm_paint.obj").string();
    REQUIRE(run(d, "EXPORT \"" + obj + "\"").ok);
    REQUIRE(run(d, "IMPORT \"" + obj + "\"").ok);
    int blueBack = 0;
    for (const Vertex& v : d.scene.bodies().back().mesh.vertices) blueBack += painted(v.color) && unpackColor(v.color).z > 0.8f;
    CHECK(blueBack > 100);
    std::filesystem::remove(obj);
    REQUIRE(run(d, "PAINT \"" + name + "\" CLEAR").ok);
    CHECK(paintedFraction(d.scene.bodies()[0]) == 0.0f);
    CHECK(!run(d, "PAINT \"" + name + "\" FILL notacolour").ok);
}

TEST_CASE("polygroups: by angle, pieces, mask and paint; masking a group; kept through sculpting, saved and exported") {
    Document d;
    REQUIRE(run(d, "BOX 40 30 20 @0,0,0").ok);
    const std::string name = d.scene.bodies().back().name;
    REQUIRE(run(d, "POLYGROUP \"" + name + "\" AUTO").ok);
    CHECK(groupSizes(d.scene.bodies().back()).size() == 6); // a box: six faces
    CHECK(d.scene.bodies().back().exact());                 // the same shape, still exact
    // Mask a group: the top (z = 20) and only it.
    const int top = groupNear(d.scene.bodies().back(), {0, 0, 20});
    REQUIRE(top >= 0);
    REQUIRE(run(d, "MASK \"" + name + "\" GROUP AT 0,0,20").ok);
    for (const Vertex& v : d.scene.bodies().back().mesh.vertices) CHECK((v.mask > 0.5f) == (v.position.z > 19.99f));
    // Groups follow the faces through dynamic topology.
    REQUIRE(run(d, "MASK \"" + name + "\" CLEAR").ok);
    REQUIRE(run(d, "SCULPT \"" + name + "\" DRAW AT 0,0,20 RADIUS 8 STRENGTH 0.5").ok);
    const Body& b = d.scene.bodies().back();
    REQUIRE(b.mesh.hasGroups());
    CHECK(groupSizes(b).size() == 6);
    CHECK(b.mesh.indices.size() / 3 > 12);
    for (std::size_t t = 0; t < b.mesh.groups.size(); ++t) { // a face low on the sides is never in the top's group
        float z = 0;
        for (int k = 0; k < 3; ++k) z += b.mesh.vertices[b.mesh.indices[t * 3 + static_cast<std::size_t>(k)]].position.z / 3;
        if (z < 10) CHECK(b.mesh.groups[t] != static_cast<std::uint32_t>(top));
    }
    // The masked faces as one new group; paint regions; connected pieces.
    REQUIRE(run(d, "SCULPT \"" + name + "\" REFINE 3").ok);
    REQUIRE(run(d, "SCULPT \"" + name + "\" MASK AT 20,0,10 RADIUS 6 STRENGTH 1").ok);
    REQUIRE(run(d, "POLYGROUP \"" + name + "\" MASK").ok);
    CHECK(groupSizes(d.scene.bodies().back()).size() == 7);
    REQUIRE(run(d, "PAINT \"" + name + "\" FILL red GROUP AT 0,0,-0.0").ok);
    REQUIRE(run(d, "POLYGROUP \"" + name + "\" PAINT").ok);
    CHECK(groupSizes(d.scene.bodies().back()).size() >= 2);
    REQUIRE(run(d, "POLYGROUP \"" + name + "\" ISLANDS").ok);
    CHECK(groupSizes(d.scene.bodies().back()).size() == 1);
    REQUIRE(run(d, "POLYGROUP \"" + name + "\" AUTO ANGLE 30").ok);
    const std::size_t groups = groupSizes(d.scene.bodies().back()).size();
    CHECK(groups >= 6);
    // Saved and read back; OBJ groups come back as polygroups.
    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_groups.sicdm").string();
    std::string err;
    REQUIRE(saveProject(d.scene, d.state, path, &err));
    Scene scene;
    ProjectState state;
    REQUIRE(loadProject(scene, state, path, &err));
    CHECK(scene.bodies().back().mesh.groups == d.scene.bodies().back().mesh.groups);
    std::filesystem::remove(path);
    const std::string obj = (std::filesystem::temp_directory_path() / "sicdm_groups.obj").string();
    REQUIRE(run(d, "EXPORT \"" + obj + "\"").ok);
    REQUIRE(run(d, "IMPORT \"" + obj + "\"").ok);
    CHECK(groupSizes(d.scene.bodies().back()).size() == groups);
    std::filesystem::remove(obj);
    CHECK(run(d, "POLYGROUP \"" + name + "\" LIST").ok);
    REQUIRE(run(d, "POLYGROUP \"" + name + "\" CLEAR").ok);
    CHECK(!d.scene.bodies()[0].mesh.hasGroups());
    CHECK(!run(d, "MASK \"" + name + "\" GROUP 0").ok); // no groups left
    CHECK(!run(d, "POLYGROUP SHOW").ok);                // headless
}

TEST_CASE("alphas: built-in patterns and images stamped through the brush") {
    // Each built-in alpha has light and dark, and is 0 outside the brush's square.
    REQUIRE(sculptAlphaNames().size() >= 10);
    for (const std::string& n : sculptAlphaNames()) {
        const auto a = sculptAlpha(n);
        REQUIRE(a != nullptr);
        float lo = 1, hi = 0;
        for (float v : a->value) lo = std::min(lo, v), hi = std::max(hi, v);
        CHECK(lo < 0.2f);
        CHECK(hi > 0.8f);
        CHECK(a->sample(1.5f, 0) == 0.0f);
    }
    std::string why;
    CHECK(!sculptAlpha("no-such-alpha", &why));
    CHECK(!why.empty());
    // An image alpha: its brightness, black on the left, white on the right.
    const std::string png = (std::filesystem::temp_directory_path() / "sicdm_alpha.png").string();
    {
        std::vector<std::uint8_t> px(32 * 16 * 3);
        for (int y = 0; y < 16; ++y)
            for (int x = 0; x < 32; ++x)
                for (int c = 0; c < 3; ++c) px[static_cast<std::size_t>((y * 32 + x) * 3 + c)] = x < 16 ? 0 : 255;
        REQUIRE(writePng8(png, 32, 16, 3, px));
    }
    const auto img = sculptAlpha(png);
    REQUIRE(img != nullptr);
    CHECK(img->width == 32);
    CHECK(img->sample(-0.6f, 0) < 0.05f);
    CHECK(img->sample(0.6f, 0) > 0.95f);
    CHECK(img->sample(0.2f, 0.0f, 2) < 0.05f); // tiled twice: the second copy starts dark again
    std::filesystem::remove(png);

    // A stamp of dots raises bumps with low ground between them; without the alpha the
    // same dab is one smooth hump.
    auto stamp = [](const char* alpha) {
        Body b = makeBody("Clay", "box", {-30, -30, 0, 30, 30, 10});
        SculptSettings s;
        s.brush = SculptBrush::Draw;
        s.radius = 15;
        s.strength = 0.5f;
        s.falloff = SculptFalloff::Constant;
        if (alpha) s.alpha = sculptAlpha(alpha);
        sculptStroke(b, {{0, 0, 10}}, s);
        return b;
    };
    const Body dots = stamp("dots"), plain = stamp(nullptr);
    CHECK(openEdgeCount(dots.mesh) == 0);
    CHECK(dots.mesh.indices.size() > plain.mesh.indices.size() * 4); // finer under an alpha
    // Heights over the stamp: dots go up and down many times along a line; the plain dab doesn't.
    auto bumps = [](const Body& b) {
        int changes = 0;
        float prev = -1;
        bool rising = false;
        for (float x = -14; x <= 14; x += 0.25f) {
            float top = 0;
            for (const Vertex& v : b.mesh.vertices)
                if (std::fabs(v.position.x - x) < 0.2f && std::fabs(v.position.y - 1.25f) < 0.6f) top = std::max(top, v.position.z);
            if (prev >= 0 && std::fabs(top - prev) > 0.05f) {
                const bool up = top > prev;
                if (up != rising) ++changes, rising = up;
            }
            prev = top;
        }
        return changes;
    };
    CHECK(bumps(dots) >= 6);
    CHECK(bumps(plain) <= 3);
    // The command: an alpha by name, turned, tiled, spaced; refused when unknown.
    Document d;
    REQUIRE(run(d, "BOX 60 60 10 @0,0,0").ok);
    REQUIRE(run(d, "SCULPT \"Block 1\" DRAW AT -20,0,10 20,0,10 RADIUS 8 STRENGTH 0.3 ALPHA bricks ALPHAANGLE 30 TILE 2 SPACING 2").ok);
    CHECK(openEdgeCount(d.scene.bodies()[0].mesh) == 0);
    REQUIRE(run(d, "SCULPT \"Block 1\" CLAY AT 0,10,10 RADIUS 8 ALPHA noise FOLLOW").ok);
    CHECK(!run(d, "SCULPT \"Block 1\" DRAW AT 0,0,10 ALPHA nothing").ok);
    CHECK(!run(d, "SCULPT \"Block 1\" DRAW AT 0,0,10 ALPHA dots TILE 0").ok);
    CHECK(run(d, "SCULPT ALPHAS").ok);
}

TEST_CASE("sculpt layers: strokes recorded, dialled in and out, deleted, baked and saved") {
    Document d;
    REQUIRE(run(d, "SPHERE 30 @0,0,0").ok);
    REQUIRE(run(d, "SCULPT \"Sphere 1\" REFINE 3").ok);
    auto top = [&] { return d.scene.bodies()[0].bounds().max.z; };
    auto volume = [&] { return meshVolume(worldMesh(d.scene.bodies()[0])); };
    const float top0 = top();
    REQUIRE(run(d, "LAYER \"Sphere 1\" NEW Horns RECORD").ok);
    CHECK(!run(d, "LAYER \"Sphere 1\" NEW Horns").ok); // taken
    REQUIRE(run(d, "SCULPT \"Sphere 1\" DRAW AT 0,0,60 RADIUS 12 STRENGTH 1").ok);
    REQUIRE(run(d, "SCULPT \"Sphere 1\" DRAW AT 0,0,60 RADIUS 12 STRENGTH 1").ok);
    const float rise = top() - top0;
    CHECK(rise > 1.0f);
    REQUIRE(d.scene.bodies()[0].mesh.hasLayers());
    // Off: the shape beneath; twice as strong: twice the rise; on again: as sculpted.
    REQUIRE(run(d, "LAYER \"Sphere 1\" OFF Horns").ok);
    CHECK_NEAR(top(), top0, 0.1f);
    CHECK(d.scene.bodies()[0].mesh.recordLayer == -1); // an off layer stops recording
    REQUIRE(run(d, "LAYER \"Sphere 1\" STRENGTH Horns 2").ok);
    CHECK_NEAR(top() - top0, 2 * rise, 0.15f);
    REQUIRE(run(d, "LAYER \"Sphere 1\" STRENGTH Horns 1").ok);
    CHECK_NEAR(top() - top0, rise, 0.1f);
    // A second layer: a carve, then deleted: the volume as before it.
    const double v0 = volume();
    REQUIRE(run(d, "LAYER \"Sphere 1\" NEW Detail RECORD").ok);
    REQUIRE(run(d, "SCULPT \"Sphere 1\" CARVE AT 30,0,30 RADIUS 8 STRENGTH 0.8").ok);
    CHECK(volume() < v0 - 10);
    // Saved and read back with the layers.
    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_layers.sicdm").string();
    std::string err;
    REQUIRE(saveProject(d.scene, d.state, path, &err));
    Scene scene;
    ProjectState state;
    REQUIRE(loadProject(scene, state, path, &err));
    REQUIRE(scene.bodies()[0].mesh.layers.size() == 2);
    CHECK(scene.bodies()[0].mesh.hasLayers());
    CHECK(scene.bodies()[0].mesh.recordLayer == 1);
    CHECK(scene.bodies()[0].mesh.layers[0].name == "Horns");
    std::filesystem::remove(path);
    REQUIRE(run(d, "LAYER \"Sphere 1\" DELETE Detail").ok);
    CHECK(std::fabs(volume() - v0) < std::fabs(v0) * 0.002);
    CHECK(d.scene.bodies()[0].mesh.layers.size() == 1);
    // Undo brings the layer back.
    REQUIRE(run(d, "UNDO").ok);
    CHECK(d.scene.bodies()[0].mesh.layers.size() == 2);
    REQUIRE(run(d, "REDO").ok);
    // Baked: the shape stays, the layers go.
    const float before = top();
    REQUIRE(run(d, "LAYER \"Sphere 1\" BAKE ALL").ok);
    CHECK(d.scene.bodies()[0].mesh.layers.empty());
    CHECK_NEAR(top(), before, 1e-4f);
    CHECK(!run(d, "LAYER \"Sphere 1\" OFF Horns").ok);
    CHECK(run(d, "LAYER \"Sphere 1\" LIST").ok);
}

TEST_CASE("multiresolution: detail sculpted high rides along when the form is moved low") {
    // Two blocks alike; one gets a bump at the finest level. Both are pulled up at level 1.
    auto make = [](Document& d, bool bump) {
        REQUIRE(run(d, "BOX 40 40 40 @0,0,0").ok);
        REQUIRE(run(d, "MULTIRES \"Block 1\" ADD 3").ok);
        if (bump) REQUIRE(run(d, "SCULPT \"Block 1\" DRAW AT 0,0,40 RADIUS 6 STRENGTH 1").ok);
    };
    Document a, b;
    make(a, true);
    make(b, false);
    auto top = [](Document& d) { return d.scene.bodies()[0].bounds().max.z; };
    const float bump = top(a) - top(b);
    CHECK(bump > 0.5f);
    for (Document* d : {&a, &b}) {
        REQUIRE(run(*d, "MULTIRES \"Block 1\" LEVEL 1").ok);
        CHECK(d->scene.bodies()[0].mesh.indices.size() / 3 == 48);
        REQUIRE(run(*d, "SCULPT \"Block 1\" GRAB AT 0,0,40 RADIUS 30 MOVE 0,0,15").ok);
        REQUIRE(run(*d, "MULTIRES \"Block 1\" LEVEL 3").ok);
        CHECK(d->scene.bodies()[0].mesh.indices.size() / 3 == 768);
    }
    CHECK(top(b) > 45.0f);                      // the form moved
    CHECK_NEAR(top(a) - top(b), bump, 0.35f * bump); // and the bump came along on top of it
    CHECK(openEdgeCount(a.scene.bodies()[0].mesh) == 0);
    // SUBDIVIDE on top adds a level; saved and read back with its levels.
    REQUIRE(run(a, "SUBDIVIDE \"Block 1\"").ok);
    REQUIRE(a.scene.bodies()[0].multires != nullptr);
    CHECK(a.scene.bodies()[0].multires->levels.size() == 5);
    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_multires.sicdm").string();
    std::string err;
    REQUIRE(saveProject(a.scene, a.state, path, &err));
    Scene scene;
    ProjectState state;
    REQUIRE(loadProject(scene, state, path, &err));
    REQUIRE(scene.bodies()[0].multires != nullptr);
    CHECK(scene.bodies()[0].multires->levels.size() == 5);
    CHECK(scene.bodies()[0].multires->current == 4);
    CHECK(scene.bodies()[0].multires->levels[4].t == a.scene.bodies()[0].multires->levels[4].t);
    std::filesystem::remove(path);
    // Down a level and up again: nothing moves.
    const float before = top(a);
    REQUIRE(run(a, "MULTIRES \"Block 1\" DOWN").ok);
    REQUIRE(run(a, "MULTIRES \"Block 1\" UP").ok);
    CHECK_NEAR(top(a), before, 1e-3f);
    // A cut changes the triangles: the levels go, the shape stays.
    REQUIRE(run(a, "HOLE \"Block 1\" AT 20,0,20 DIAMETER 6 DEPTH 5").ok);
    CHECK(!run(a, "MULTIRES \"Block 1\" LEVEL 0").ok);
    CHECK(!a.scene.bodies()[0].multires);
    // Dropping levels below or above, and applying.
    REQUIRE(run(b, "MULTIRES \"Block 1\" LEVEL 2").ok);
    REQUIRE(run(b, "MULTIRES \"Block 1\" DELETE HIGHER").ok);
    CHECK(b.scene.bodies()[0].multires->levels.size() == 3);
    REQUIRE(run(b, "MULTIRES \"Block 1\" DELETE LOWER").ok);
    CHECK(!b.scene.bodies()[0].multires); // one level left: none
    CHECK(!run(b, "MULTIRES \"Block 1\" APPLY").ok);
    CHECK(run(b, "MULTIRES \"Block 1\" LIST").ok);
}

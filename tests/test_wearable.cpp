#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/FileIO.hpp"
#include "core/Motion.hpp"
#include "core/Physics.hpp"
#include "core/Scene.hpp"
#include "core/Sculpt.hpp"
#include "core/Wearable.hpp"

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

int indexOf(const Scene& s, const std::string& name) {
    for (std::size_t i = 0; i < s.bodies().size(); ++i)
        if (s.bodies()[i].name == name) return static_cast<int>(i);
    return -1;
}

} // namespace

TEST_CASE("shell hollows a solid to an exact wall and opens a side") {
    Body box = makeBody("Box", "box", {0, 0, 0, 100, 80, 60});
    ShellOptions o;
    o.thickness = 4;
    std::string error;
    REQUIRE(shellBody(box, o, &error));
    CHECK(openEdgeCount(box.mesh) == 0);
    CHECK_NEAR(meshVolume(worldMesh(box)), 100.0 * 80 * 60 - 92.0 * 72 * 52, 1.0);

    // Opened at z = 50: a 100 x 80 x 50 tray with 4 mm walls and floor.
    Body tray = makeBody("Tray", "box", {0, 0, 0, 100, 80, 60});
    o.open = true;
    o.openPoint = {50, 40, 50};
    o.openNormal = {0, 0, 1};
    REQUIRE(shellBody(tray, o, &error));
    CHECK(openEdgeCount(tray.mesh) == 0);
    CHECK_NEAR(meshVolume(worldMesh(tray)), 100.0 * 80 * 50 - 92.0 * 72 * 46, 1.0);
    CHECK_NEAR(tray.bounds().max.z, 50.0, 1e-3);

    // A helmet: a ball opened at the bottom.
    Body ball = makeBody("Ball", "sphere", {40});
    o.thickness = 3;
    o.openPoint = {0, 0, 30};
    o.openNormal = {0, 0, -1};
    REQUIRE(shellBody(ball, o, &error));
    CHECK(openEdgeCount(ball.mesh) == 0);
    CHECK(ball.bounds().min.z > 29.9f);
    CHECK(meshVolume(worldMesh(ball)) < 4.0 / 3.0 * 3.1416 * 40 * 40 * 40 * 0.25);
}

TEST_CASE("armour keeps its clearance from the body and moves with it") {
    Body rod = makeBody("Rod", "cylinder", {20, 100});
    rod.offset = {10, 0, 0};
    std::string error;
    const auto armour = armorFor(rod, 5, 3, &error);
    REQUIRE(armour.has_value());
    CHECK(openEdgeCount(armour->mesh) == 0);
    CHECK(armour->name == "Armor Rod");
    // 20 mm radius + 5 clearance + 3 wall = 28 mm around the axis at x = 10.
    CHECK_NEAR(armour->bounds().min.x, 10 - 28, 0.8);
    CHECK_NEAR(armour->bounds().max.x, 10 + 28, 0.8);
    // Its inner face is the clearance away: no armour vertex is within 4.5 mm of the rod's side.
    float nearest = 1e9f;
    for (const Vertex& v : armour->mesh.vertices) {
        const Vec3 p = v.position + armour->offset;
        if (p.z < 5 || p.z > 95) continue;
        nearest = std::min(nearest, std::hypot(p.x - 10, p.y) - 20);
    }
    CHECK(nearest > 4.5f);

    // With the command: attached to its part, it follows the part's motion.
    Document d;
    REQUIRE(run(d, "CYLINDER 20 100 @0,0,0").ok);
    REQUIRE(run(d, "ARMOR CLEARANCE 5 THICKNESS 3").ok);
    const int part = indexOf(d.scene, "Cylinder 1"), arm = indexOf(d.scene, "Armor Cylinder 1");
    REQUIRE(part >= 0);
    REQUIRE(arm >= 0);
    CHECK(d.scene.bodies()[static_cast<std::size_t>(arm)].attachedTo == "Cylinder 1");
    REQUIRE(run(d, "MOTION CIRCLE AXIS X AT 0,0,0 SPEED 90 BODIES \"Cylinder 1\"").ok);
    const auto tf = simulationTransforms(d.scene, 1.0f);
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            CHECK_NEAR(tf[static_cast<std::size_t>(arm)].m[c][r], tf[static_cast<std::size_t>(part)].m[c][r], 1e-6);

    // Saved and loaded with the project.
    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_armour_test.sicdm").string();
    std::string err;
    REQUIRE(saveProject(d.scene, d.state, path, &err));
    Scene loaded;
    ProjectState st;
    REQUIRE(loadProject(loaded, st, path, &err));
    CHECK(loaded.bodies()[static_cast<std::size_t>(arm)].attachedTo == "Cylinder 1");
    std::filesystem::remove(path);
}

TEST_CASE("lighten fills a solid with a closed gyroid lattice") {
    Body block = makeBody("Block", "box", {0, 0, 0, 80, 80, 40});
    double before = 0, after = 0;
    std::string error;
    LightenOptions o;
    REQUIRE(lightenBody(block, o, &error, &before, &after));
    CHECK(openEdgeCount(block.mesh) == 0);
    CHECK_NEAR(before, 80.0 * 80 * 40, 1.0);
    CHECK(after < before * 0.6);
    CHECK(after > before * 0.1);
    CHECK(block.bounds().max.z <= 40.001f); // the outer surface is untouched

    Body hollow = makeBody("Block", "box", {0, 0, 0, 80, 80, 40});
    o.hollow = true;
    REQUIRE(lightenBody(hollow, o, &error, &before, &after));
    CHECK_NEAR(after, 80.0 * 80 * 40 - 76.0 * 76 * 36, 80.0 * 80 * 40 * 0.02); // a 2 mm skin, within the voxels

    Body thin = makeBody("Plate", "box", {0, 0, 0, 80, 80, 3});
    CHECK(!lightenBody(thin, LightenOptions{}, &error));
}

TEST_CASE("joints swing a part and the interference check finds the collision") {
    Document d;
    REQUIRE(run(d, "BOX 60 40 10 @0,0,0").ok);
    REQUIRE(run(d, "BOX 60 40 10 @70,0,0").ok);
    // A hinge between the blocks: no collision while it swings up to 30 degrees.
    REQUIRE(run(d, "JOINT \"Block 2\" TO \"Block 1\" AT 35,0,5 AXIS Y RANGE -30 30 PERIOD 4").ok);
    CHECK(indexOf(d.scene, "Block 2 Hinge Pin") >= 0);
    REQUIRE(!d.scene.motions().empty());
    const Motion& m = d.scene.motions().back();
    CHECK(m.hasLimits);
    CHECK_NEAR(m.lo, -30, 1e-6);
    CHECK_NEAR(m.hi, 30, 1e-6);
    // At a quarter period the free block has swung to the upper limit.
    const auto tf = simulationTransforms(d.scene, 1.0f);
    const Vec3 tip = tf[1].transformPoint(Vec3{100, 0, 5});
    CHECK(tip.z < -20.0f || tip.z > 30.0f);

    auto r = run(d, "INTERFERENCE CLEARANCE 1 BODIES \"Block 1\" \"Block 2\"");
    REQUIRE(r.ok);
    CHECK(r.lines.size() >= 2);
    CHECK(r.lines[1].find("all clear") != std::string::npos);

    // Swung to 120 degrees the free block cuts through the fixed one.
    REQUIRE(run(d, "UNDO").ok);
    REQUIRE(run(d, "JOINT \"Block 2\" TO \"Block 1\" AT 35,0,5 AXIS Y RANGE -120 120 PERIOD 4").ok);
    r = run(d, "INTERFERENCE CLEARANCE 1 BODIES \"Block 1\" \"Block 2\"");
    REQUIRE(r.ok);
    bool collides = false;
    for (const std::string& l : r.lines) collides = collides || l.find("COLLIDES") != std::string::npos;
    CHECK(collides);

    // A part that overlaps another at rest (a link at its joint) and stays put is no
    // collision of the motion.
    REQUIRE(run(d, "BOX 20 20 20 @0,0,5").ok);
    const std::string third = d.scene.bodies().back().name;
    r = run(d, "INTERFERENCE CLEARANCE 1 BODIES \"Block 1\" \"" + third + "\"");
    REQUIRE(r.ok);
    REQUIRE(r.lines.size() >= 3);
    CHECK(r.lines[1].find("all clear") != std::string::npos);
    CHECK(r.lines[2].find("1 pairs already overlap") != std::string::npos);
    const InterferenceReport rep = checkInterference(d.scene, {0, static_cast<std::size_t>(indexOf(d.scene, third))}, 1.0f, 0.5f, 4.0f);
    CHECK(rep.pairs.empty());
    CHECK(rep.restOverlaps == 1);
}

TEST_CASE("shell and lighten take round primitives and refuse impossible walls") {
    Body ball = makeBody("Ball", "sphere", {30});
    std::string error;
    LightenOptions lo;
    double before = 0, after = 0;
    REQUIRE(lightenBody(ball, lo, &error, &before, &after));
    CHECK(after < before * 0.8);
    CHECK(SculptMesh(ball).openEdges() == 0); // the sphere's own seam welds shut

    Body rod = makeBody("Rod", "cylinder", {20, 60});
    ShellOptions so;
    so.thickness = 3;
    REQUIRE(shellBody(rod, so, &error));
    CHECK(openEdgeCount(rod.mesh) == 0);

    Body slab = makeBody("Slab", "box", {0, 0, 0, 60, 40, 10});
    so.thickness = 6; // two 6 mm walls in a 10 mm slab leave no cavity
    CHECK(!shellBody(slab, so, &error));
    CHECK(error.find("no cavity") != std::string::npos);
    so.outward = true; // outwards it is fine
    CHECK(shellBody(slab, so, &error));
}

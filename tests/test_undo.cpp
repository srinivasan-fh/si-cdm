#include "TestFramework.hpp"
#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Scene.hpp"

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

} // namespace

TEST_CASE("undo: a mate and the move it made go back together, and come back with REDO") {
    Document d;
    REQUIRE(run(d, "BOX 20 20 10 @0,0,0").ok);
    REQUIRE(run(d, "BOX 20 20 10 @-80,30,0").ok);
    const Vec3 was = d.scene.bodies()[1].offset;
    REQUIRE(run(d, "MATE COINCIDENT \"Block 1\" AT 0,0,10 \"Block 2\" AT -80,30,0").ok);
    REQUIRE(d.state.assembly.mates.size() == 1);
    const Vec3 moved = d.scene.bodies()[1].offset;
    CHECK(length(moved - was) > 1.0f);
    const std::size_t steps = d.state.history.size();
    REQUIRE(run(d, "UNDO").ok); // one UNDO: the mate and the move
    CHECK(d.state.assembly.mates.empty());
    CHECK(length(d.scene.bodies()[1].offset - was) < 1e-4f);
    CHECK(d.state.history.size() == steps - 1);
    REQUIRE(run(d, "REDO").ok);
    CHECK(d.state.assembly.mates.size() == 1);
    CHECK(length(d.scene.bodies()[1].offset - moved) < 1e-4f);
    // The box before it is still one more UNDO away.
    REQUIRE(run(d, "UNDO").ok);
    REQUIRE(run(d, "UNDO").ok);
    CHECK(d.scene.bodies().size() == 1);
}

TEST_CASE("undo: the drawing sheet, variables and datums") {
    Document d;
    REQUIRE(run(d, "BOX 40 30 20 @0,0,0").ok);
    // The sheet isn't a history step: undoing it leaves the history alone.
    const std::size_t steps = d.state.history.size();
    REQUIRE(run(d, "SHEET NEW A3").ok);
    REQUIRE(!d.state.drawing.views.empty());
    REQUIRE(run(d, "UNDO").ok);
    CHECK(d.state.drawing.views.empty());
    CHECK(d.state.history.size() == steps);
    REQUIRE(run(d, "REDO").ok);
    CHECK(!d.state.drawing.views.empty());
    // A variable is a history step: UNDO takes back both.
    REQUIRE(run(d, "VAR w = 60").ok);
    REQUIRE(d.state.variables.size() == 1);
    REQUIRE(run(d, "UNDO").ok);
    CHECK(d.state.variables.empty());
    CHECK(d.state.history.size() == steps);
    REQUIRE(run(d, "REFPOINT P1 AT 1,2,3").ok);
    REQUIRE(d.state.refPoints.size() == 1);
    REQUIRE(run(d, "UNDO").ok);
    CHECK(d.state.refPoints.empty());
    // Then the sheet (it came after the box), then the box.
    REQUIRE(run(d, "UNDO").ok);
    CHECK(d.state.drawing.views.empty());
    CHECK(d.scene.bodies().size() == 1);
    REQUIRE(run(d, "UNDO").ok);
    CHECK(d.scene.bodies().empty());
}

TEST_CASE("undo: parameters, motions, physics, terrain and robot settings") {
    Document d;
    REQUIRE(run(d, "SPHERE 10 @0,0,0").ok);
    REQUIRE(run(d, "CYLINDER 5 30 @50,0,0").ok);
    // A motion.
    REQUIRE(run(d, "MOTION CIRCLE AXIS X AT 0,0,0 SPEED 90 BODIES \"Cylinder 2\"").ok);
    REQUIRE(d.scene.motions().size() == 1);
    REQUIRE(run(d, "UNDO").ok);
    CHECK(d.scene.motions().empty());
    // A body's physics and the world's.
    REQUIRE(run(d, "PHYSICS BODY \"Sphere 1\" DYNAMIC VELOCITY 500,0,2000").ok);
    REQUIRE(d.scene.bodies()[0].physics.enabled);
    REQUIRE(run(d, "UNDO").ok);
    CHECK(!d.scene.bodies()[0].physics.enabled);
    const bool air = d.scene.physics().air;
    REQUIRE(run(d, "PHYSICS AIR ON").ok);
    REQUIRE(d.scene.physics().air != air || air);
    REQUIRE(run(d, "UNDO").ok);
    CHECK(d.scene.physics().air == air);
    // Terrain.
    REQUIRE(run(d, "TERRAIN STAIRS RISE 150 RUN 300 STEPS 4 SIZE 3000").ok);
    REQUIRE(d.scene.terrain().kind != TerrainKind::None);
    REQUIRE(run(d, "UNDO").ok);
    CHECK(d.scene.terrain().kind == TerrainKind::None);
    REQUIRE(run(d, "REDO").ok);
    CHECK(d.scene.terrain().kind != TerrainKind::None);
    // A part's parameters.
    Document p;
    REQUIRE(run(p, "PART STEPPER").ok);
    auto vertices = [&] {
        std::uint64_t n = 0;
        for (const Body& b : p.scene.bodies()) n = n * 31 + meshContentKey(b.mesh);
        return n;
    };
    const std::uint64_t verts = vertices();
    REQUIRE(run(p, "PARAM \"Stepper Motor 1\" SHAFTS DUAL SHAFT_LENGTH 30").ok);
    REQUIRE(vertices() != verts);
    REQUIRE(run(p, "UNDO").ok);
    CHECK(vertices() == verts);
    // Robot settings.
    Document r;
    REQUIRE(run(r, "BOX 40 40 200 @0,0,0").ok);
    REQUIRE(run(r, "BOX 30 30 150 @0,0,200").ok);
    REQUIRE(run(r, "JOINT \"Block 2\" TO \"Block 1\" AT 0,0,200 AXIS X RANGE -60 90").ok);
    const bool rigsBefore = r.scene.rigs().empty();
    REQUIRE(run(r, "RIG BASE FLOATING DENSITY 2700").ok);
    REQUIRE(!r.scene.rigs().empty());
    REQUIRE(run(r, "UNDO").ok);
    CHECK(r.scene.rigs().empty() == rigsBefore);
}

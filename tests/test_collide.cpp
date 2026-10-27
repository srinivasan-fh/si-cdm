#include "TestFramework.hpp"
#include "core/Collide.hpp"
#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Geometry.hpp"
#include "core/Kinematics.hpp"
#include "core/Motion.hpp"
#include "core/Multibody.hpp"
#include "core/Rig.hpp"
#include "core/Scene.hpp"

#include <cmath>
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

std::string text(const CommandOutput& o) {
    std::string s;
    for (const std::string& l : o.lines) s += l + "\n";
    return s;
}

Rigid moved(double x, double y, double z, double turnZDeg = 0) {
    Rigid r;
    const double a = turnZDeg * 3.14159265358979 / 180.0, c = std::cos(a), s = std::sin(a);
    r.R = {c, -s, 0, s, c, 0, 0, 0, 1};
    r.t = {x, y, z};
    return r;
}

} // namespace

TEST_CASE("collisions: exact shapes cross, touch, nest or miss") {
    const CollisionShape a(primitives::box({-10, -10, -10}, {10, 10, 10}));
    const CollisionShape b(primitives::box({-5, -5, -5}, {5, 5, 5}));
    CHECK(a.closed());
    CHECK(shapesCollide(a, Rigid{}, b, moved(12, 0, 0)));   // crossing
    CHECK(!shapesCollide(a, Rigid{}, b, moved(16, 0, 0)));  // a millimetre apart
    CHECK(shapesCollide(a, Rigid{}, b, moved(15, 0, 0)));   // faces touching
    CHECK(shapesCollide(a, Rigid{}, b, moved(0, 0, 0)));    // wholly inside: no faces cross
    CHECK(shapesCollide(b, Rigid{}, a, moved(0, 0, 0)));
    // Turned 45 degrees, its corner reaches 5 * sqrt 2 = 7.07 out.
    CHECK(shapesCollide(a, Rigid{}, b, moved(16.5, 0, 0, 45)));
    CHECK(!shapesCollide(a, Rigid{}, b, moved(17.5, 0, 0, 45)));
    // An L: a block in the corner its box covers isn't touching it.
    MeshData l = primitives::box({0, 0, 0}, {100, 20, 20});
    l.append(primitives::box({0, 20, 0}, {20, 100, 20}));
    const CollisionShape L(l);
    CHECK(!shapesCollide(L, Rigid{}, b, moved(60, 60, 10)));
    CHECK(shapesCollide(L, Rigid{}, b, moved(60, 22, 10)));
    std::array<double, 3> where{};
    REQUIRE(shapesCollide(L, Rigid{}, b, moved(60, 22, 10), &where));
    CHECK(std::fabs(where[0] - 60) < 15);
    // Points inside and outside a closed shape.
    CHECK(a.contains({1, 2, 3}));
    CHECK(!a.contains({11, 0, 0}));
}

TEST_CASE("collisions: an arm reaches into a bin its box would have covered") {
    Document d;
    REQUIRE(run(d, "BOX 100 100 100 @0,0,0").ok);
    REQUIRE(run(d, "BOX 80 80 60 @0,0,100").ok);
    REQUIRE(run(d, "BOX 40 40 300 @0,0,160").ok);
    REQUIRE(run(d, "BOX 40 40 300 @0,0,460").ok);
    REQUIRE(run(d, "JOINT \"Block 2\" TO \"Block 1\" AT 0,0,100 AXIS Z RANGE -180 180").ok);
    REQUIRE(run(d, "JOINT \"Block 3\" TO \"Block 2\" AT 0,0,160 AXIS Y RANGE -120 120").ok);
    REQUIRE(run(d, "JOINT \"Block 4\" TO \"Block 3\" AT 0,0,460 AXIS Y RANGE -150 150").ok);
    REQUIRE(run(d, "RIG TAG \"Block 4\" END_EFFECTOR").ok);
    for (Motion& m : d.scene.motions()) m.keys = {{0.0f, 0.0f}};
    // A bin: 20 mm walls and floor, open at the top.
    REQUIRE(run(d, "BOX 260 260 200 @450,0,0").ok);
    REQUIRE(run(d, "BOX 220 220 200 @450,0,20").ok);
    REQUIRE(run(d, "BOOLEAN \"Block 5\" SUBTRACT \"Block 6\"").ok);
    // The tool down inside it, the forearm over the wall.
    const CommandOutput o = run(d, "ARM MOVE_TO 450,0,120 POINT 0,0,150");
    REQUIRE(o.ok);
    RigSettings s;
    s.name = "";
    if (const RigSettings* r = d.scene.rigs().find("")) s = *r;
    RobotRig rig;
    REQUIRE(buildRig(d.scene, s, rig));
    const RobotKinematics kin(rig);
    const CollisionModel col(d.scene, rig, 0.0f);
    float end = 0;
    for (const Motion& m : d.scene.motions())
        if (!m.keys.empty()) end = std::max(end, m.keys.back().t);
    const std::vector<double> q = motionTargets(d.scene, rig, end).value;
    CHECK(!col.collides(kin.forward(q)));
    const V3d tip = kin.forward(q)[static_cast<std::size_t>(rig.endEffectors().front())].apply({0, 0, 0.760});
    CHECK(std::hypot(tip[0] - 0.45, tip[1], tip[2] - 0.12) < 0.003);
    // Into the wall itself it can't go.
    CHECK(!run(d, "ARM MOVE_TO 330,0,100 POINT 0,0,150").ok);
    // Played back, nothing runs into anything.
    const std::string clean = text(run(d, "SIM CHECK"));
    CHECK(clean.find("No parts run into each other") != std::string::npos);
    // A block put in the way after the move was programmed: SIM CHECK finds it.
    REQUIRE(run(d, "BOX 60 60 60 @250,0,560").ok);
    const CommandOutput hit = run(d, "SIM CHECK");
    REQUIRE(hit.ok);
    CHECK(text(hit).find("hits Block 7") != std::string::npos);
    CHECK(text(run(d, "SIM CHECK \"Block 1\"")).find("No parts run into each other") != std::string::npos); // the base stays clear
}

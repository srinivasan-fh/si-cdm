#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/FileIO.hpp"
#include "core/Rig.hpp"
#include "core/Scene.hpp"

#include <cmath>
#include <filesystem>
#include <string>

using namespace sicdm;
namespace fs = std::filesystem;

#define REQUIRE(c)                  \
    do {                            \
        const bool required_ = (c); \
        CHECK(required_);           \
        if (!required_) return;     \
    } while (0)

namespace {

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }

bool rigOf(const Document& d, const std::string& name, RobotRig& rig) {
    RigSettings s;
    if (const RigSettings* r = d.state.rigs.find(name)) s = *r;
    else s.name = name;
    return buildRig(d.scene, s, rig);
}

} // namespace

TEST_CASE("Robotics: mass and inertia of meshes match the textbook solids") {
    Document d;
    REQUIRE(run(d, "BOX 100 200 300 @0,0,0").ok); // 0.1 x 0.2 x 0.3 m, 1000 kg/m^3: 6 kg
    const MassInertia box = meshInertia(d.scene.bodies()[0], 1000.0);
    CHECK_NEAR(box.mass, 6.0, 1e-6);
    CHECK_NEAR(box.inertia[0], 6.0 * (0.04 + 0.09) / 12.0, 1e-6);
    CHECK_NEAR(box.inertia[4], 6.0 * (0.01 + 0.09) / 12.0, 1e-6);
    CHECK_NEAR(box.inertia[8], 6.0 * (0.01 + 0.04) / 12.0, 1e-6);
    CHECK_NEAR(box.inertia[1], 0.0, 1e-9);
    const AABB bb = d.scene.bodies()[0].bounds();
    CHECK_NEAR(box.com[0], bb.center().x * 1e-3, 1e-6);
    CHECK_NEAR(box.com[2], bb.center().z * 1e-3, 1e-6);

    REQUIRE(run(d, "SPHERE 50 @500,0,0").ok); // r = 0.05 m: I = 2/5 m r^2 (faceted, within 1 %)
    const MassInertia ball = meshInertia(d.scene.bodies()[1], 1000.0);
    const double m = 4.0 / 3.0 * 3.14159265358979 * 0.05 * 0.05 * 0.05 * 1000.0;
    CHECK(std::fabs(ball.mass - m) < 0.01 * m);
    CHECK(std::fabs(ball.inertia[0] - 0.4 * ball.mass * 0.0025) < 0.02 * 0.4 * ball.mass * 0.0025);
    CHECK_NEAR(ball.inertia[0], ball.inertia[8], 0.02 * ball.inertia[0]);
    // A body's own mass wins over its density.
    d.scene.bodies()[0].physics.mass = 2.0f;
    CHECK_NEAR(meshInertia(d.scene.bodies()[0], 1000.0).inertia[0], 2.0 * 0.13 / 12.0, 1e-6);
}

TEST_CASE("Robotics: the library arm and humanoid read as robots") {
    Document d;
    REQUIRE(run(d, "PART robot_arm").ok);
    RobotRig arm;
    REQUIRE(rigOf(d, "Robot Arm 1", arm));
    CHECK(arm.morphology == Morphology::Manipulator);
    CHECK(!arm.floatingBase);
    CHECK(arm.joints.size() == 22);
    // The shoulder is a chain of three revolute joints, the elbow hangs below it.
    const int flex = arm.jointNamed("Shoulder Flex"), abd = arm.jointNamed("Shoulder Abduction"), elbow = arm.jointNamed("Elbow");
    REQUIRE(flex >= 0 && abd >= 0 && elbow >= 0);
    CHECK(arm.joints[static_cast<std::size_t>(flex)].parent == 0);
    CHECK(arm.joints[static_cast<std::size_t>(abd)].parent == arm.joints[static_cast<std::size_t>(flex)].child);
    const RigJoint& e = arm.joints[static_cast<std::size_t>(elbow)];
    CHECK_NEAR(e.lo, 0.0, 1e-9);
    CHECK_NEAR(e.hi, 150.0 * 3.14159265358979 / 180.0, 1e-6);
    // Fingers: one drive moves the three joints of a finger.
    const RigJoint& pip = arm.joints[static_cast<std::size_t>(arm.jointNamed("Index PIP"))];
    CHECK(pip.mimic == "Robot Arm 1 Index MCP");
    CHECK_NEAR(pip.mimicRatio, 0.278 / 0.25, 0.01);
    // Every part is in exactly one link, and every link with parts has a sane inertia.
    std::size_t parts = 0;
    for (const RigLink& l : arm.links) {
        parts += l.bodies.size();
        CHECK(l.mass > 0);
        CHECK(l.inertia[0] > 0 && l.inertia[4] > 0 && l.inertia[8] > 0);
        CHECK(l.inertia[0] + l.inertia[4] >= l.inertia[8] * 0.999); // triangle inequality of principal moments
    }
    CHECK(parts == d.scene.bodies().size());
    CHECK(!arm.endEffectors().empty());

    Document h;
    REQUIRE(run(h, "PART humanoid").ok);
    RobotRig man;
    REQUIRE(rigOf(h, "Male Humanoid Robot 1", man));
    CHECK(man.morphology == Morphology::Humanoid);
    CHECK(man.floatingBase);
    const int knee = man.jointNamed("Right Knee");
    REQUIRE(knee >= 0);
    CHECK(man.joints[static_cast<std::size_t>(knee)].type == RigJointType::Revolute);
    CHECK(man.totalMass() > 30 && man.totalMass() < 300);
    // RIG names it without quotes too, and LIST shows it.
    CHECK(run(h, "RIG Male Humanoid Robot 1").ok);
    CHECK(run(h, "RIG LIST").lines.size() == 1);
}

TEST_CASE("Robotics: a robot made with JOINT, tags and save") {
    Document d;
    REQUIRE(run(d, "BOX 40 40 200 @0,0,0").ok);      // Block 1: the base post
    REQUIRE(run(d, "BOX 30 30 150 @0,0,200").ok);    // Block 2: the upper arm
    REQUIRE(run(d, "JOINT \"Block 2\" TO \"Block 1\" AT 0,0,200 AXIS X RANGE -60 90").ok);
    RobotRig r;
    REQUIRE(rigOf(d, "", r));
    REQUIRE(r.joints.size() == 1);
    CHECK(r.joints[0].type == RigJointType::Revolute);
    CHECK_NEAR(r.joints[0].lo, -60 * 3.14159265358979 / 180.0, 1e-6);
    CHECK_NEAR(r.joints[0].origin[2], 0.2, 1e-6);
    CHECK(r.links[1].tag == RigTag::None);
    // The hinge pin rides on the parent, so the base has two parts.
    CHECK(r.links[0].bodies.size() == 2);

    REQUIRE(run(d, "RIG TAG \"Block 2\" END_EFFECTOR").ok);
    REQUIRE(run(d, "RIG BASE FLOATING DENSITY 2700").ok);
    REQUIRE(rigOf(d, "", r));
    CHECK(r.links[1].tag == RigTag::EndEffector);
    CHECK(r.endEffectors() == std::vector<int>{1});
    CHECK(r.floatingBase);
    CHECK_NEAR(r.links[1].mass, 0.03 * 0.03 * 0.15 * 2700, 1e-3);

    const std::string path = (fs::temp_directory_path() / "sicdm-rig.sicdm").string();
    REQUIRE(run(d, "SAVE \"" + path + "\"").ok);
    Document e;
    REQUIRE(run(e, "OPEN \"" + path + "\"").ok);
    REQUIRE(e.state.rigs.robots.size() == 1);
    CHECK(e.state.rigs.robots[0].tagOf("Block 2") == RigTag::EndEffector);
    CHECK(e.state.rigs.robots[0].density == 2700.0f);
    CHECK(e.state.rigs.robots[0].floatingBase.value_or(false));
    fs::remove(path);
    CHECK(!run(d, "RIG TAG \"Nothing\" WHEEL").ok);
    CHECK(!run(d, "RIG TAG \"Block 2\" FLIPPER").ok);
}

TEST_CASE("Robotics: auto-rigging makes drones and rovers from plain parts") {
    Document d;
    REQUIRE(run(d, "BOX 200 200 40 @0,0,0").ok); // the frame
    for (const char* at : {"@-140,-140,40", "@140,-140,40", "@-140,140,40", "@140,140,40"})
        REQUIRE(run(d, std::string("CYLINDER 60 4 ") + at).ok); // flat discs: propellers
    for (int i = 2; i <= 5; ++i) REQUIRE(run(d, "RIG TAG \"Cylinder " + std::to_string(i) + "\" ROTOR").ok);
    RobotRig drone;
    REQUIRE(rigOf(d, "", drone));
    CHECK(drone.morphology == Morphology::Drone);
    CHECK(drone.floatingBase);
    REQUIRE(drone.joints.size() == 4);
    for (const RigJoint& j : drone.joints) {
        CHECK(j.type == RigJointType::Continuous);
        CHECK_NEAR(std::fabs(j.axis[2]), 1.0, 1e-9); // spins about the disc's axis
    }
    CHECK(d.scene.motions().size() == 4);
    // Tagging again does not add another spin; UNDO takes the tag's spin away.
    REQUIRE(run(d, "RIG TAG \"Cylinder 2\" ROTOR").ok);
    CHECK(d.scene.motions().size() == 4);

    Document v;
    REQUIRE(run(v, "BOX 400 250 60 @0,0,60").ok);
    int n = 2;
    for (const char* at : {"@-150,-150,50", "@150,-150,50", "@-150,150,50", "@150,150,50"}) {
        REQUIRE(run(v, std::string("CYLINDER 50 30 ") + at).ok);
        REQUIRE(run(v, "ROTATE \"Cylinder " + std::to_string(n++) + "\" 90,0,0").ok); // on its side: the axle along Y
    }
    for (int i = 2; i <= 5; ++i) REQUIRE(run(v, "RIG TAG \"Cylinder " + std::to_string(i) + "\" WHEEL").ok);
    RobotRig rover;
    REQUIRE(rigOf(v, "", rover));
    CHECK(rover.morphology == Morphology::Rover);
    CHECK(rover.joints.size() == 4);
    for (const RigJoint& j : rover.joints) CHECK_NEAR(std::fabs(j.axis[1]), 1.0, 1e-9);
    REQUIRE(run(v, "UNDO").ok);
    CHECK(v.scene.motions().size() == 3);
    REQUIRE(run(v, "RIG MORPHOLOGY CUSTOM BASE FIXED").ok);
    REQUIRE(rigOf(v, "", rover));
    CHECK(rover.morphology == Morphology::Custom);
    CHECK(!rover.floatingBase);
}

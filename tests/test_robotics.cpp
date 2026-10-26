#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/FileIO.hpp"
#include "core/ForceField.hpp"
#include "core/Kinematics.hpp"
#include "core/Mcp.hpp"
#include "core/RobotEnv.hpp"
#include "core/Motion.hpp"
#include "core/Multibody.hpp"
#include "core/Physics.hpp"
#include "core/Rig.hpp"
#include "core/Scene.hpp"
#include "core/Sensors.hpp"
#include "core/Terrain.hpp"

#include <stb_image.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
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
    if (const RigSettings* r = d.scene.rigs().find(name)) s = *r;
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
    REQUIRE(e.scene.rigs().robots.size() == 1);
    CHECK(e.scene.rigs().robots[0].tagOf("Block 2") == RigTag::EndEffector);
    CHECK(e.scene.rigs().robots[0].density == 2700.0f);
    CHECK(e.scene.rigs().robots[0].floatingBase.value_or(false));
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

TEST_CASE("Robotics: URDF, SDF and MJCF export with meshes") {
    Document d;
    REQUIRE(run(d, "PART robot_arm").ok);
    const fs::path dir = fs::temp_directory_path() / "sicdm_tests" / "robot_export";
    std::error_code ec;
    fs::remove_all(dir, ec);
    fs::create_directories(dir);
    auto slurp = [](const fs::path& p) {
        std::string s;
        if (FILE* f = std::fopen(p.string().c_str(), "rb")) {
            char buf[65536];
            for (std::size_t n; (n = std::fread(buf, 1, sizeof buf, f)) > 0;) s.append(buf, n);
            std::fclose(f);
        }
        return s;
    };
    auto count = [](const std::string& s, const std::string& what) {
        std::size_t n = 0;
        for (std::size_t p = s.find(what); p != std::string::npos; p = s.find(what, p + 1)) ++n;
        return n;
    };
    REQUIRE(run(d, "EXPORT \"" + (dir / "arm.urdf").string() + "\"").ok);
    const std::string u = slurp(dir / "arm.urdf");
    CHECK(count(u, "<link name=") == 24);      // the 23 links and "world" (a fixed base)
    CHECK(count(u, "<joint name=") == 23);     // 22 and the base's fixed joint
    CHECK(count(u, "<mimic joint=") == 9);     // finger couplings
    CHECK(count(u, "<inertia ixx=") == 23);
    // The elbow sits 300 mm along the upper arm from the shoulder chain's frame.
    CHECK(u.find("<child link=\"Robot_Arm_1_Elbow_link\"/>\n    <origin xyz=\"0 0.3 0\"") != std::string::npos);
    CHECK(fs::exists(dir / "arm_meshes" / "Robot_Arm_1_Forearm_Rotation_link.stl"));
    // A binary STL: 80 + 4 + 50 per triangle.
    const std::string stl = slurp(dir / "arm_meshes" / "Robot_Arm_1_Forearm_Rotation_link.stl");
    REQUIRE(stl.size() > 84);
    std::uint32_t tris = 0;
    std::memcpy(&tris, stl.data() + 80, 4);
    CHECK(stl.size() == 84 + 50 * static_cast<std::size_t>(tris));

    REQUIRE(run(d, "RIG EXPORT \"" + (dir / "arm.sdf").string() + "\"").ok);
    const std::string s = slurp(dir / "arm.sdf");
    CHECK(count(s, "<link name=") == 23);
    CHECK(count(s, "<joint name=") == 23);
    CHECK(s.find("<parent>world</parent>") != std::string::npos);

    REQUIRE(run(d, "EXPORT \"" + (dir / "arm.xml").string() + "\"").ok);
    const std::string m = slurp(dir / "arm.xml");
    CHECK(count(m, "<body name=") == 23);
    CHECK(count(m, "type=\"hinge\"") == 22);
    CHECK(count(m, "polycoef=") == 9);
    CHECK(count(m, "_servo\"") == 13);
    CHECK(m.find("<freejoint") == std::string::npos); // the arm is bolted down
    CHECK(!run(d, "RIG EXPORT \"" + (dir / "arm.txt").string() + "\"").ok);

    // A floating humanoid: MJCF frees its base, URDF leaves the root unattached.
    Document h;
    REQUIRE(run(h, "PART humanoid").ok);
    REQUIRE(run(h, "EXPORT \"" + (dir / "man.xml").string() + "\"").ok);
    CHECK(count(slurp(dir / "man.xml"), "<freejoint") == 1);
    REQUIRE(run(h, "EXPORT \"" + (dir / "man.urdf").string() + "\"").ok);
    CHECK(slurp(dir / "man.urdf").find("<link name=\"world\"") == std::string::npos);
    fs::remove_all(dir, ec);
}

namespace {

RobotRig rigFor(Document& d, const std::string& name = "") {
    RigSettings s;
    if (const RigSettings* r = d.scene.rigs().find(name)) s = *r;
    else s.name = name;
    RobotRig rig;
    buildRig(d.scene, s, rig);
    return rig;
}

} // namespace

TEST_CASE("Robotics engine: free fall, momentum and energy of a free body") {
    Document d;
    REQUIRE(run(d, "BOX 100 200 300 @0,0,1000").ok);
    RobotRig rig = rigFor(d);
    rig.floatingBase = true;
    MultibodyOptions o;
    o.contacts = false;
    Multibody fall(rig, o);
    const V3d c0 = fall.centreOfMass();
    fall.advance(1.0);
    CHECK(std::fabs((fall.centreOfMass()[2] - c0[2]) - (-0.5 * 9.80665)) < 0.01); // semi-implicit Euler: g dt t / 2 off

    o.gravity = {0, 0, 0};
    Multibody spin(rig, o);
    spin.setBaseVelocity({1.0, 0.2, 3.0}, {0.5, 0, 0});
    const double e0 = spin.kineticEnergy();
    const V3d s0 = spin.centreOfMass();
    spin.advance(5.0);
    CHECK(std::fabs(spin.kineticEnergy() - e0) < 0.005 * e0);            // torque-free tumbling keeps its energy
    CHECK(std::fabs(spin.centreOfMass()[0] - s0[0] - 2.5) < 1e-3);         // and its momentum
    CHECK(std::fabs(spin.centreOfMass()[1] - s0[1]) < 1e-6);
}

TEST_CASE("Robotics engine: a pendulum swings at its period, a servo holds, stops stop") {
    Document d;
    REQUIRE(run(d, "BOX 20 20 400 @0,0,100").ok);
    REQUIRE(run(d, "BOX 20 400 20 @0,200,500").ok); // a rod from the pivot out along +Y
    REQUIRE(run(d, "JOINT \"Block 2\" TO \"Block 1\" AT 0,0,510 AXIS X RANGE -180 180").ok);
    RobotRig rig = rigFor(d);
    REQUIRE(rig.joints.size() == 1);
    CHECK(!rig.floatingBase);
    rig.joints[0].type = RigJointType::Continuous;
    MultibodyOptions o;
    o.contacts = false;
    o.jointDamping = 0;
    Multibody mb(rig, o);
    JointControl passive;
    passive.mode = ActuatorMode::Passive;
    mb.setControl(0, passive);
    // Hanging straight down is q = -90 deg (the rod points +Y at rest, X turns it).
    const double hang = -3.14159265358979 / 2;
    mb.setJointState({hang + 0.05}, {0.0});
    const RigLink& L = rig.links[1];
    const double dy = L.com[1], dz = L.com[2] - 0.51;
    const double d2 = dy * dy + dz * dz;
    const double Ipivot = L.inertia[0] + L.mass * d2;
    // 1 % of armature (motor inertia) is part of every joint.
    const double expected = 2 * 3.14159265358979 * std::sqrt(Ipivot * 1.01 / (L.mass * 9.80665 * std::sqrt(d2)));
    double last = mb.q()[0] - hang, t0 = -1, t1 = -1;
    int crossings = 0;
    for (int i = 0; i < 40000 && t1 < 0; ++i) {
        mb.step();
        const double x = mb.q()[0] - hang;
        if (last > 0 && x <= 0) {
            if (++crossings == 1) t0 = mb.time();
            else if (crossings == 2) t1 = mb.time();
        }
        last = x;
    }
    REQUIRE(t1 > 0);
    CHECK(std::fabs((t1 - t0) - expected) < 0.005 * expected);
    // Energy over large swings: within 1 % of m g d over 10 s (semi-implicit Euler).
    mb.reset();
    mb.setControl(0, passive);
    const double e0 = mb.kineticEnergy() + mb.potentialEnergy();
    mb.advance(10.0);
    CHECK(std::fabs(mb.kineticEnergy() + mb.potentialEnergy() - e0) < 0.01 * L.mass * 9.80665 * std::sqrt(d2));

    // A position servo holds the rod out against gravity, on target (its integral takes the sag out).
    RobotRig held = rigFor(d);
    Multibody servo(held);
    JointControl c;
    c.target = 0.5;
    servo.setControl(0, c);
    servo.advance(3.0);
    CHECK(std::fabs(servo.q()[0] - 0.5) < 1e-3);
    CHECK(std::fabs(servo.torques()[0]) > 0.1); // it is working against the load
    // A tight joint (0..30 deg) let go at 20 deg falls onto its lower stop.
    held.joints[0].lo = 0;
    held.joints[0].hi = 30 * 3.14159265358979 / 180;
    Multibody stop(held);
    stop.setControl(0, passive);
    stop.setJointState({0.35}, {0.0});
    double lowest = 1;
    for (int i = 0; i < 6000; ++i) {
        stop.step();
        lowest = std::min(lowest, stop.q()[0]);
    }
    CHECK(lowest > -2.0 * 3.14159265358979 / 180); // through the stop by less than 2 degrees
    CHECK(std::fabs(stop.q()[0]) < 0.5 * 3.14159265358979 / 180); // and resting on it
}

TEST_CASE("Robotics engine: resting, standing and coupled fingers") {
    Document d;
    REQUIRE(run(d, "BOX 200 200 100 @0,0,0").ok);
    RobotRig box = rigFor(d);
    box.floatingBase = true;
    Multibody rest(box);
    rest.advance(2.0);
    double fz = 0;
    for (const ContactPoint& c : rest.contacts()) fz += c.force[2];
    CHECK(std::fabs(fz - box.totalMass() * 9.80665) < 0.01 * box.totalMass() * 9.80665);
    const V3d c0 = rest.centreOfMass();
    rest.advance(2.0);
    CHECK(std::fabs(rest.centreOfMass()[0] - c0[0]) + std::fabs(rest.centreOfMass()[1] - c0[1]) < 1e-5); // no creep

    // The humanoid stands on its own feet, servos holding its rest pose.
    Document h;
    REQUIRE(run(h, "PART humanoid").ok);
    const RobotRig man = rigFor(h, "Male Humanoid Robot 1");
    Multibody stand(man);
    const V3d s0 = stand.centreOfMass();
    stand.advance(5.0);
    REQUIRE(!stand.diverged());
    const V3d s1 = stand.centreOfMass();
    CHECK(std::fabs(s1[0] - s0[0]) < 0.02 && std::fabs(s1[1] - s0[1]) < 0.02 && std::fabs(s1[2] - s0[2]) < 0.01);
    double ground = 0;
    for (const ContactPoint& c : stand.contacts()) ground += c.force[2];
    CHECK(std::fabs(ground - man.totalMass() * 9.80665) < 0.05 * man.totalMass() * 9.80665);

    // One drive closes a finger: the coupled joints follow at their ratios.
    Document a;
    REQUIRE(run(a, "PART robot_arm").ok);
    const RobotRig arm = rigFor(a, "Robot Arm 1");
    Multibody grip(arm);
    const int mcp = arm.jointNamed("Index MCP"), pip = arm.jointNamed("Index PIP");
    REQUIRE(mcp >= 0 && pip >= 0);
    JointControl close;
    close.target = 0.8;
    grip.setControl(static_cast<std::size_t>(mcp), close);
    grip.advance(1.5);
    CHECK(std::fabs(grip.q()[static_cast<std::size_t>(mcp)] - 0.8) < 0.01);
    CHECK(std::fabs(grip.q()[static_cast<std::size_t>(pip)] - 0.8 * arm.joints[static_cast<std::size_t>(pip)].mimicRatio) < 0.02);
}

TEST_CASE("Robotics engine: engines by morphology, playback and state") {
    CHECK(defaultEngine(Morphology::Humanoid) == SimEngine::Articulated);
    CHECK(defaultEngine(Morphology::Drone) == SimEngine::Vehicle);
    CHECK(defaultEngine(Morphology::Rover) == SimEngine::Vehicle);
    CHECK(simEngineFromKey("mujoco") == SimEngine::Articulated);
    CHECK(!simEngineFromKey("warp"));

    Document d;
    REQUIRE(run(d, "BOX 40 40 200 @0,0,0").ok);
    REQUIRE(run(d, "BOX 30 400 30 @0,200,200").ok);
    REQUIRE(run(d, "JOINT \"Block 2\" TO \"Block 1\" AT 0,0,215 AXIS X RANGE -60 90 PERIOD 4").ok);
    CHECK(!run(d, "RIG ENGINE WARP").ok);
    REQUIRE(run(d, "RIG ENGINE ARTICULATED SIMULATE ON").ok);
    REQUIRE(d.scene.rigs().robots.size() == 1);
    CHECK(d.scene.rigs().robots[0].simulate);
    // Playback: the joint follows its keyframes (90 deg at a quarter period) under gravity.
    const std::vector<Mat4> tf = simulationTransforms(d.scene, 1.0f);
    REQUIRE(tf.size() == d.scene.bodies().size());
    const Vec3 tip = tf[1].transformPoint({0, 400, 215});
    CHECK(tip.z > 400); // swung up from the horizontal toward 90 deg
    const CommandOutput st = run(d, "RIG STATE AT 1");
    REQUIRE(st.ok);
    CHECK(st.lines[0].find("articulated engine") != std::string::npos);
    // The followed angle is close to its target.
    const RobotRig rig = rigFor(d);
    const auto runPtr = robotRun(d.scene, d.scene.rigs().robots[0], 10.0f);
    REQUIRE(runPtr != nullptr);
    const RobotRunFrame f = runPtr->at(1.0f);
    const MotionTargets tg = motionTargets(d.scene, rig, 1.0f);
    CHECK(std::fabs(f.q[0] - tg.value[0]) < 0.05);
    REQUIRE(run(d, "RIG SIMULATE OFF").ok);
    CHECK(robotRuns(d.scene).empty());
}

TEST_CASE("Robotics terrain: stairs, slopes, blocks, rough ground and DEM files") {
    Document d;
    REQUIRE(run(d, "TERRAIN STAIRS RISE 150 RUN 300 STEPS 4 SIZE 3000").ok);
    const TerrainSettings& t = d.scene.terrain();
    CHECK(t.kind == TerrainKind::Stairs);
    CHECK_NEAR(terrainHeight(t, -100, 0), 0.0, 1e-9);
    CHECK_NEAR(terrainHeight(t, 10, 0), 150.0, 1e-9);
    CHECK_NEAR(terrainHeight(t, 650, 300), 450.0, 1e-9);
    CHECK_NEAR(terrainHeight(t, 1400, 0), 600.0, 1e-9); // the landing
    CHECK_NEAR(terrainHeight(t, 1600, 0), 0.0, 1e-9);   // off the patch: the floor
    REQUIRE(d.scene.bodies().size() == 1);
    CHECK(d.scene.bodies()[0].name == "Terrain");
    CHECK(std::fabs(d.scene.bodies()[0].bounds().max.z - 600.0f) < 1e-3f);

    REQUIRE(run(d, "TERRAIN SLOPE ANGLE 20 DIRECTION 90 SIZE 2000").ok);
    CHECK_NEAR(terrainHeight(d.scene.terrain(), 0, 0), 1000.0 * std::tan(20 * 3.14159265358979 / 180), 1e-6);
    const Vec3 n = terrainNormal(d.scene.terrain(), 0, 0);
    CHECK(std::fabs(n.y + std::sin(20 * 3.14159265358979 / 180)) < 1e-3); // tilted toward -Y: it climbs toward +Y

    REQUIRE(run(d, "TERRAIN ROUGH AMPLITUDE 50 SEED 3").ok);
    double lo = 1e9, hi = -1e9;
    for (int x = -800; x <= 800; x += 40)
        for (int y = -800; y <= 800; y += 40) {
            const double h = terrainHeight(d.scene.terrain(), x, y);
            lo = std::min(lo, h), hi = std::max(hi, h);
        }
    CHECK(lo >= 0 && hi <= 50.0 && hi - lo > 10.0);
    CHECK_NEAR(terrainHeight(d.scene.terrain(), 999, 0), 0.0, 0.5); // fades to the floor at its edge (x = 1000)

    REQUIRE(run(d, "TERRAIN BLOCKS HEIGHT 80 BLOCK 200").ok);
    int raised = 0;
    for (int x = -900; x <= 900; x += 200) raised += terrainHeight(d.scene.terrain(), x + 100, 100) > 0 ? 1 : 0;
    CHECK(raised > 0 && raised < 10);

    // A DEM: an ESRI ASCII grid (heights in metres) ...
    const fs::path dir = fs::temp_directory_path() / "sicdm_tests" / "dem";
    std::error_code ec;
    fs::create_directories(dir, ec);
    {
        FILE* f = std::fopen((dir / "hill.asc").string().c_str(), "w");
        REQUIRE(f != nullptr);
        std::fprintf(f, "ncols 5\nnrows 3\nxllcorner 1000\nyllcorner 2000\ncellsize 0.5\nNODATA_value -9999\n");
        std::fprintf(f, "10.0 10.2 10.4 10.6 10.8\n10.0 10.1 10.2 10.3 10.4\n10.0 10.0 10.0 10.0 -9999\n"); // the first row is north (+Y)
        std::fclose(f);
    }
    REQUIRE(run(d, "TERRAIN DEM \"" + (dir / "hill.asc").string() + "\"").ok);
    const TerrainSettings& dem = d.scene.terrain();
    CHECK(dem.nx == 5 && dem.ny == 3);
    CHECK_NEAR(dem.cell, 500.0, 1e-3);
    CHECK_NEAR(dem.sizeX, 2000.0, 1e-3);
    CHECK_NEAR(terrainHeight(dem, 1000, 500), 800.0, 0.5); // north-east corner: 10.8 m - the lowest 10.0 m
    CHECK_NEAR(terrainHeight(dem, -1000, -500), 0.0, 0.5);
    CHECK_NEAR(terrainHeight(dem, 500, 0), 300.0, 0.5);
    // ... saved with the project, heights and all.
    const std::string path = (dir / "terrain.sicdm").string();
    REQUIRE(run(d, "SAVE \"" + path + "\"").ok);
    Document e;
    REQUIRE(run(e, "OPEN \"" + path + "\"").ok);
    CHECK(e.scene.terrain().kind == TerrainKind::Dem);
    CHECK_NEAR(terrainHeight(e.scene.terrain(), 500, 0), 300.0, 0.5);
    // ... or a greyscale image over a height range.
    {
        FILE* f = std::fopen((dir / "ramp.pgm").string().c_str(), "wb");
        REQUIRE(f != nullptr);
        std::fprintf(f, "P5\n4 2\n255\n");
        const unsigned char px[8] = {0, 85, 170, 255, 0, 85, 170, 255};
        std::fwrite(px, 1, 8, f);
        std::fclose(f);
    }
    REQUIRE(run(d, "TERRAIN DEM \"" + (dir / "ramp.pgm").string() + "\" CELL 100 HEIGHT 300").ok);
    CHECK_NEAR(terrainHeight(d.scene.terrain(), 150, 0), 300.0, 0.5);
    CHECK_NEAR(terrainHeight(d.scene.terrain(), -150, 0), 0.0, 0.5);
    CHECK(!run(d, "TERRAIN DEM \"" + (dir / "missing.asc").string() + "\"").ok);
    fs::remove_all(dir, ec);

    REQUIRE(run(d, "TERRAIN OFF").ok);
    CHECK(d.scene.bodies().empty());
    CHECK(!d.scene.terrain().active());
}

TEST_CASE("Robotics terrain: friction holds a block on a gentle slope and lets it slide on a steep one") {
    for (const double angle : {30.0, 50.0}) {
        Document d;
        REQUIRE(run(d, "TERRAIN SLOPE ANGLE " + std::to_string(angle) + " SIZE 8000").ok);
        REQUIRE(run(d, "BOX 200 200 100 @0,0,0").ok);
        RigSettings s;
        s.name = "Block 1";
        RobotRig rig;
        REQUIRE(buildRig(d.scene, s, rig));
        CHECK(rig.links[0].bodies.size() == 1); // the terrain is not part of it
        rig.floatingBase = true;
        MultibodyOptions o;
        o.ground = sceneGround(d.scene);
        Multibody mb(rig, o);
        const double th = angle * 3.14159265358979 / 180, c = std::cos(th), sn = std::sin(th);
        LinkPose p;
        p.R = {c, 0, -sn, 0, 1, 0, sn, 0, c};
        p.t = {-sn * 0.0005, 0, 4.0 * std::tan(th) + 0.0005 * c};
        mb.setBasePose(p);
        mb.advance(0.5);
        const V3d v1 = mb.linkVelocity(0);
        mb.advance(1.0);
        const V3d v2 = mb.linkVelocity(0);
        const double accel = std::hypot(v2[0], v2[2]) - std::hypot(v1[0], v1[2]);
        const double expected = std::max(0.0, 9.80665 * (sn - 0.9 * c)); // mu = 0.9
        CHECK(std::fabs(accel - expected) < 0.02 * std::max(expected, 1.0));
    }
}

namespace {

// A 2.3 kg quadcopter: a frame and four 240 mm rotors tagged ROTOR.
void makeDrone(Document& d) {
    REQUIRE(run(d, "BOX 200 200 40 @0,0,100").ok);
    for (const char* at : {"@-170,-170,150", "@170,-170,150", "@-170,170,150", "@170,170,150"}) REQUIRE(run(d, std::string("CYLINDER 120 4 ") + at).ok);
    for (int i = 2; i <= 5; ++i) REQUIRE(run(d, "RIG TAG \"Cylinder " + std::to_string(i) + "\" ROTOR").ok);
}

V3d frameVelocity(const RobotRun& r, float t) {
    const RobotRunFrame a = r.at(t - 0.05f), b = r.at(t + 0.05f);
    return {(b.com[0] - a.com[0]) / 0.1, (b.com[1] - a.com[1]) / 0.1, (b.com[2] - a.com[2]) / 0.1};
}

double frameYaw(const RobotRun& r, float t) {
    const LinkPose b = r.at(t).links[0];
    return std::atan2(b.R[3], b.R[0]);
}

} // namespace

TEST_CASE("Robotics vehicles: a quadcopter takes off, holds, flies at a velocity and yaws") {
    Document d;
    makeDrone(d);
    REQUIRE(run(d, "RIG MISSION GOTO 0,0,1000").ok);
    REQUIRE(run(d, "RIG MISSION VELOCITY 1,0,0 YAW 30 AT 6").ok);
    REQUIRE(run(d, "RIG MISSION HOVER AT 9").ok);
    REQUIRE(d.scene.rigs().robots[0].mission.size() == 3);
    const RigSettings s = d.scene.rigs().robots[0];
    RobotRig rig;
    REQUIRE(buildRig(d.scene, s, rig));
    CHECK(engineFor(rig, s) == SimEngine::Vehicle);
    const auto r = robotRun(d.scene, s, 12.0f);
    REQUIRE(r != nullptr);
    CHECK(!r->diverged);
    // Up and holding a metre (the centre of mass starts 107 mm up).
    const RobotRunFrame f5 = r->at(5.5f);
    CHECK(std::fabs(f5.com[2] - 1.0) < 0.12);
    CHECK(std::fabs(f5.com[0]) < 0.02 && std::fabs(f5.com[1]) < 0.02);
    // Forward at 1 m/s, turning at 30 deg/s.
    const V3d v = frameVelocity(*r, 8.0f);
    CHECK(std::fabs(v[0] - 1.0) < 0.1 && std::fabs(v[1]) < 0.1);
    const double turned = frameYaw(*r, 8.5f) - frameYaw(*r, 7.0f);
    CHECK(std::fabs(turned - 1.5 * 30 * 3.14159265358979 / 180) < 0.1);
    // Then it stops (braking takes a second or two) and holds.
    const V3d stopped = frameVelocity(*r, 11.8f);
    CHECK(std::hypot(stopped[0], stopped[1]) < 0.15);
    // The mission is saved, cleared and reported.
    CHECK(run(d, "RIG").lines.back().find("hover") != std::string::npos);
    REQUIRE(run(d, "RIG MISSION CLEAR").ok);
    CHECK(d.scene.rigs().robots[0].mission.empty());
    CHECK(!run(d, "RIG MISSION SOAR").ok);
}

TEST_CASE("Robotics vehicles: a drone holds its position in a crosswind") {
    Document d;
    makeDrone(d);
    REQUIRE(run(d, "RIG MISSION GOTO 0,0,800").ok);
    REQUIRE(run(d, "PHYSICS WIND 3,0,0").ok);
    const auto r = robotRun(d.scene, d.scene.rigs().robots[0], 10.0f);
    REQUIRE(r != nullptr);
    const RobotRunFrame f = r->at(10.0f);
    CHECK(std::fabs(f.com[0]) < 0.1);       // leans into the wind and stays put
    CHECK(std::fabs(f.com[2] - 0.8) < 0.1);
    // It leans into the wind: its body z axis tips toward -X (R's third column, x row).
    const LinkPose b = f.links[0];
    CHECK(b.R[2] < -0.001);
}

TEST_CASE("Robotics vehicles: a rover drives straight and turns at the rate asked") {
    Document d;
    REQUIRE(run(d, "BOX 400 250 60 @0,0,60").ok);
    int n = 2;
    for (const char* at : {"@-150,-150,60", "@150,-150,60", "@-150,150,60", "@150,150,60"}) {
        REQUIRE(run(d, std::string("CYLINDER 60 30 ") + at).ok);
        REQUIRE(run(d, "ROTATE \"Cylinder " + std::to_string(n++) + "\" 90,0,0").ok);
    }
    for (int i = 2; i <= 5; ++i) REQUIRE(run(d, "RIG TAG \"Cylinder " + std::to_string(i) + "\" WHEEL").ok);
    REQUIRE(run(d, "RIG MISSION DRIVE 1").ok);
    REQUIRE(run(d, "RIG MISSION DRIVE 0.5 TURN 30 AT 3").ok);
    const auto r = robotRun(d.scene, d.scene.rigs().robots[0], 8.0f);
    REQUIRE(r != nullptr);
    const V3d v = frameVelocity(*r, 2.0f);
    CHECK(std::fabs(v[0] - 1.0) < 0.05 && std::fabs(v[1]) < 0.05);
    const double turned = frameYaw(*r, 7.0f) - frameYaw(*r, 5.0f);
    CHECK(std::fabs(turned - 2.0 * 30 * 3.14159265358979 / 180) < 0.1);
    const V3d v2 = frameVelocity(*r, 6.0f);
    CHECK(std::fabs(std::hypot(v2[0], v2[1]) - 0.5) < 0.06);
}

TEST_CASE("Robotics sensors: an IMU and a GPS on a part that turns") {
    Document d;
    REQUIRE(run(d, "BOX 200 200 100 @0,0,0").ok);
    // The block spins at 90 deg/s about Z through the origin; the IMU sits 500 mm out on it.
    REQUIRE(run(d, "MOTION CIRCLE Spin AXIS Z AT 0,0,0 SPEED 90 BODIES \"Block 1\"").ok);
    REQUIRE(run(d, "RIG SENSOR ADD IMU imu ON \"Block 1\" AT 500,0,50 NOISE 0 BIAS 0").ok);
    REQUIRE(run(d, "RIG SENSOR ADD GPS gps ON \"Block 1\" AT 500,0,50 NOISE 0 ORIGIN 10,20,100").ok);
    const RigSettings& s = d.scene.rigs().robots[0];
    SensorReading r;
    REQUIRE(readSensor(d.scene, s, *s.sensor("imu"), 1.0f, r));
    const double w = 3.14159265358979 / 2, centripetal = 0.5 * w * w;
    // Specific force in the sensor's frame (x out along the arm): -r w^2 toward the axis, +g up.
    CHECK_NEAR(r.values[0], -centripetal, 0.01);
    CHECK_NEAR(r.values[1], 0.0, 0.01);
    CHECK_NEAR(r.values[2], 9.80665, 0.01);
    CHECK_NEAR(r.values[5], w, 1e-3);
    CHECK_NEAR(r.values[3], 0.0, 1e-6);
    // A quarter turn later the GPS has gone round to +Y: latitude grows by 0.5 m of arc.
    REQUIRE(readSensor(d.scene, s, *s.sensor("gps"), 1.0f, r));
    CHECK_NEAR(r.values[0], 0.0, 1e-3);
    CHECK_NEAR(r.values[1], 0.5, 1e-3);
    CHECK_NEAR(r.values[6], 10.0 + 0.5 / 6378137.0 * 180.0 / 3.14159265358979, 1e-9);
    CHECK_NEAR(r.values[7], 20.0, 1e-9);
    CHECK_NEAR(r.values[8], 100.05, 1e-6);
    // Noise: repeatable, and the size the model says.
    REQUIRE(run(d, "RIG SENSOR ADD IMU noisy ON \"Block 1\" BIAS 0 RATE 200").ok);
    const SensorSpec& noisy = *d.scene.rigs().robots[0].sensor("noisy");
    SensorReading a, b;
    REQUIRE(readSensor(d.scene, d.scene.rigs().robots[0], noisy, 0.5f, a));
    REQUIRE(readSensor(d.scene, d.scene.rigs().robots[0], noisy, 0.5f, b));
    CHECK(a.values == b.values);
    double sum = 0, sum2 = 0;
    const int n = 400;
    SensorReading ideal;
    SensorSpec clean = noisy;
    clean.noise = 0;
    for (int k = 0; k < n; ++k) {
        const float t = 0.005f * static_cast<float>(k);
        REQUIRE(readSensor(d.scene, d.scene.rigs().robots[0], noisy, t, a));
        REQUIRE(readSensor(d.scene, d.scene.rigs().robots[0], clean, t, ideal));
        const double e = a.values[0] - ideal.values[0];
        sum += e, sum2 += e * e;
    }
    const double sigma = std::sqrt(sum2 / n - (sum / n) * (sum / n));
    CHECK(std::fabs(sigma - typicalNoise(SensorKind::Imu, 0).white) < 0.25 * typicalNoise(SensorKind::Imu, 0).white);
    // Saved with the project.
    const std::string path = (fs::temp_directory_path() / "sicdm-sensors.sicdm").string();
    REQUIRE(run(d, "SAVE \"" + path + "\"").ok);
    Document e;
    REQUIRE(run(e, "OPEN \"" + path + "\"").ok);
    REQUIRE(e.scene.rigs().robots.size() == 1);
    CHECK(e.scene.rigs().robots[0].sensors.size() == 3);
    CHECK(e.scene.rigs().robots[0].sensor("gps")->lat == 10.0);
    fs::remove(path);
    CHECK(!run(d, "RIG SENSOR ADD IMU imu ON \"Block 1\"").ok); // the name is taken
    CHECK(!run(d, "RIG SENSOR ADD SONAR x ON \"Block 1\"").ok);
    CHECK(run(d, "RIG SENSOR REMOVE noisy").ok);
}

TEST_CASE("Robotics sensors: cameras and LiDARs see the scene, PNGs and point clouds") {
    Document d;
    REQUIRE(run(d, "BOX 100 100 100 @0,0,0").ok);         // Block 1: carries the sensors
    REQUIRE(run(d, "BOX 400 2000 2000 @2200,0,0").ok);     // Block 2: a wall, its face 2 m away at x = 2000
    REQUIRE(run(d, "RIG SENSOR ADD CAMERA cam ON \"Block 1\" AT 0,0,500 LOOK 1,0,0 SIZE 64 48 FOV 60 NOISE 0").ok);
    REQUIRE(run(d, "RIG SENSOR ADD LIDAR lid ON \"Block 1\" AT 0,0,500 CHANNELS 1 SAMPLES 36 RANGE 10 NOISE 0").ok);
    const RigSettings& s = d.scene.rigs().robots[0];
    SensorReading c;
    REQUIRE(readSensor(d.scene, s, *s.sensor("cam"), 0.0f, c));
    REQUIRE(c.width == 64 && c.height == 48);
    const std::size_t centre = static_cast<std::size_t>(24 * 64 + 32);
    CHECK_NEAR(c.depth[centre], 2.0, 0.01);
    CHECK(c.instance[centre] == 1);
    CHECK(c.semantic[centre] == 3); // the wall is not part of the robot
    CHECK(c.semantic[static_cast<std::size_t>(47 * 64 + 32)] == 1); // the floor at the bottom of the view
    // The LiDAR's level ring meets the wall straight ahead at 2 m.
    SensorReading l;
    REQUIRE(readSensor(d.scene, s, *s.sensor("lid"), 0.0f, l));
    REQUIRE(!l.ranges.empty());
    CHECK_NEAR(l.points[0][0], 2.0, 0.01); // azimuth 0 is first
    CHECK_NEAR(l.ranges[0], 2.0, 0.01);
    // Files: three PNGs (RGB, 16-bit depth in mm, segmentation) and a point cloud.
    const fs::path dir = fs::temp_directory_path() / "sicdm_tests" / "sensors";
    std::error_code ec;
    fs::create_directories(dir, ec);
    REQUIRE(run(d, "RIG SENSOR CAPTURE cam \"" + (dir / "view.png").string() + "\"").ok);
    int w = 0, h = 0, n = 0;
    unsigned char* px = stbi_load((dir / "view_rgb.png").string().c_str(), &w, &h, &n, 0);
    REQUIRE(px != nullptr);
    CHECK(w == 64 && h == 48 && n == 3);
    stbi_image_free(px);
    unsigned short* depth = stbi_load_16((dir / "view_depth.png").string().c_str(), &w, &h, &n, 0);
    REQUIRE(depth != nullptr);
    CHECK(std::abs(static_cast<int>(depth[centre]) - 2000) <= 5);
    stbi_image_free(depth);
    REQUIRE(run(d, "RIG SENSOR CAPTURE lid \"" + (dir / "scan.csv").string() + "\"").ok);
    CHECK(fs::file_size(dir / "scan.csv") > 20);
    CHECK(!run(d, "RIG SENSOR LOG cam \"" + (dir / "x.csv").string() + "\"").ok); // cameras capture, they don't log
    fs::remove_all(dir, ec);
}

TEST_CASE("Robotics sensors: a force-torque sensor feels the arm it holds") {
    Document d;
    REQUIRE(run(d, "BOX 40 40 200 @0,0,0").ok);
    REQUIRE(run(d, "BOX 30 400 30 @0,200,200").ok); // a rod out along +Y from the pivot
    REQUIRE(run(d, "JOINT \"Block 2\" TO \"Block 1\" AT 0,0,215 AXIS X RANGE -60 90").ok);
    d.scene.motions()[0].keys = {{0.0f, 0.0f}};      // held level
    REQUIRE(run(d, "RIG SIMULATE ON").ok);
    REQUIRE(run(d, "RIG SENSOR ADD FORCE ft ON \"Block 2\" AT 0,0,215 LOOK 1,0,0 NOISE 0").ok);
    const RigSettings& s = d.scene.rigs().robots[0];
    SensorReading r;
    REQUIRE(readSensor(d.scene, s, *s.sensor("ft"), 3.0f, r));
    RobotRig rig;
    REQUIRE(buildRig(d.scene, s, rig));
    const double m = rig.links[1].mass, arm = rig.links[1].com[1];
    CHECK_NEAR(r.values[2], m * 9.80665, 0.02 * m * 9.80665);   // the joint holds the rod up
    CHECK_NEAR(std::fabs(r.values[3]), m * 9.80665 * arm, 0.03 * m * 9.80665 * arm); // and its moment about X
    REQUIRE(run(d, "RIG SIMULATE OFF").ok);
    CHECK(!readSensor(d.scene, d.scene.rigs().robots[0], *d.scene.rigs().robots[0].sensor("ft"), 3.0f, r));
}

namespace {

// A three-joint arm standing at the origin: a turret (yaw about Z), a shoulder and an elbow
// (both about Y), 300 mm links; the forearm is the tool, its tip at (0, 0, 760) at rest.
// The joints hold still at rest until something keys them.
void makeArm(Document& d) {
    REQUIRE(run(d, "BOX 100 100 100 @0,0,0").ok);   // Block 1: base (a box stands on its point)
    REQUIRE(run(d, "BOX 80 80 60 @0,0,100").ok);    // Block 2: turret
    REQUIRE(run(d, "BOX 40 40 300 @0,0,160").ok);   // Block 3: upper arm
    REQUIRE(run(d, "BOX 40 40 300 @0,0,460").ok);   // Block 4: forearm
    REQUIRE(run(d, "JOINT \"Block 2\" TO \"Block 1\" AT 0,0,100 AXIS Z RANGE -180 180").ok);
    REQUIRE(run(d, "JOINT \"Block 3\" TO \"Block 2\" AT 0,0,160 AXIS Y RANGE -120 120").ok);
    REQUIRE(run(d, "JOINT \"Block 4\" TO \"Block 3\" AT 0,0,460 AXIS Y RANGE -150 150").ok);
    REQUIRE(run(d, "RIG TAG \"Block 4\" END_EFFECTOR").ok);
    for (Motion& m : d.scene.motions()) m.keys = {{0.0f, 0.0f}};
}

V3d armTip(const RobotKinematics& kin, const std::vector<double>& q) {
    const RobotRig& rig = kin.rig();
    const int ee = rig.endEffectors().front();
    return kin.forward(q)[static_cast<std::size_t>(ee)].apply({0, 0, 0.760});
}

} // namespace

TEST_CASE("Robotics arms: inverse kinematics reaches points and refuses what's out of reach") {
    Document d;
    makeArm(d);
    RobotRig rig;
    REQUIRE(rigOf(d, "", rig));
    REQUIRE(rig.joints.size() == 3);
    const RobotKinematics kin(rig);
    const int ee = rig.endEffectors().front();
    CHECK(kin.chain(ee).size() == 3);
    const V3d rest = armTip(kin, std::vector<double>(3, 0.0));
    CHECK_NEAR(rest[2], 0.760, 1e-9);
    const V3d goals[] = {{0.3, 0.2, 0.3}, {-0.25, -0.35, 0.5}, {0.45, 0.0, 0.3}, {0.0, 0.1, 0.72}};
    for (const V3d& g : goals) {
        IkTarget t;
        t.link = ee;
        t.point = {0, 0, 0.760};
        t.target = g;
        const IkResult r = solveIk(kin, std::vector<double>(3, 0.0), t);
        CHECK(r.ok);
        const V3d p = armTip(kin, r.q);
        CHECK(std::hypot(p[0] - g[0], p[1] - g[1], p[2] - g[2]) < 0.0011);
        for (std::size_t j = 0; j < 3; ++j) CHECK(r.q[j] >= kin.lower(j) - 1e-9 && r.q[j] <= kin.upper(j) + 1e-9);
    }
    // Pointing the tool: straight down onto a point in front (with the shoulder at 30 deg; a
    // 3-joint arm has just that one way to do it).
    IkTarget down;
    down.link = ee;
    down.point = {0, 0, 0.760};
    down.target = {0.15, 0, 0.16 + 0.3 * std::cos(3.14159265358979 / 6) - 0.3};
    down.approach = V3d{0, 0, -1};
    down.approachAt = {0, 0, 1};
    const IkResult dr = solveIk(kin, std::vector<double>(3, 0.0), down);
    REQUIRE(dr.ok);
    const LinkPose fp = kin.forward(dr.q)[static_cast<std::size_t>(ee)];
    CHECK_NEAR(fp.R[8], -1.0, 0.01); // its rest Z axis now points down
    // Out of reach: 600 mm of arm can't get 900 mm from the shoulder.
    IkTarget far = down;
    far.approach.reset();
    far.target = {0.9, 0, 0.16};
    const IkResult fr = solveIk(kin, std::vector<double>(3, 0.0), far);
    CHECK(!fr.ok);
    CHECK(fr.why.find("out of reach") != std::string::npos);
    CHECK(run(d, "ARM IK 300,200,300").ok);
    CHECK(!run(d, "ARM IK 900,0,160").ok);
}

TEST_CASE("Robotics arms: a planned move goes around an obstacle and is keyed into the motions") {
    Document d;
    makeArm(d);
    REQUIRE(run(d, "BOX 100 200 180 @300,0,580").ok); // in the way of the straight swing
    RobotRig rig;
    REQUIRE(rigOf(d, "", rig));
    const RobotKinematics kin(rig);
    const CollisionModel col(d.scene, rig, 0.0f);
    CHECK(!col.collides(kin.forward(std::vector<double>(3, 0.0))));
    const int ee = rig.endEffectors().front();
    IkTarget t;
    t.link = ee;
    t.point = {0, 0, 0.760};
    t.target = {0.45, 0, 0.3};
    const IkResult ik = solveIk(kin, std::vector<double>(3, 0.0), t, &col);
    REQUIRE(ik.ok);
    // The straight line in joint space hits the block...
    bool straightHits = false;
    for (int k = 1; k < 100; ++k) {
        std::vector<double> q(3);
        for (std::size_t j = 0; j < 3; ++j) q[j] = ik.q[j] * k / 100.0;
        straightHits = straightHits || col.collides(kin.forward(q));
    }
    CHECK(straightHits);
    // ...the plan doesn't.
    const PlanResult plan = planPath(kin, col, std::vector<double>(3, 0.0), ik.q, kin.chain(ee));
    REQUIRE(plan.ok);
    CHECK(plan.path.size() >= 3);
    for (std::size_t s = 0; s + 1 < plan.path.size(); ++s)
        for (int k = 0; k <= 50; ++k) {
            std::vector<double> q(3);
            for (std::size_t j = 0; j < 3; ++j) q[j] = plan.path[s][j] + (plan.path[s + 1][j] - plan.path[s][j]) * k / 50.0;
            CHECK(!col.collides(kin.forward(q)));
        }

    // The command does the same and keys the joints' motions: the tool gets there, clear all the way.
    const CommandOutput o = run(d, "ARM MOVE_TO 450,0,300 POINT 0,0,150");
    REQUIRE(o.ok);
    const Motion* shoulder = nullptr;
    for (const Motion& m : d.scene.motions())
        if (m.name == rig.joints[1].name) shoulder = &m;
    REQUIRE(shoulder != nullptr);
    REQUIRE(shoulder->keys.size() >= 2);
    const float end = shoulder->keys.back().t;
    CHECK(end > 0.2f);
    for (float tt = 0; tt <= end + 1e-3f; tt += 0.01f) CHECK(!col.collides(kin.forward(motionTargets(d.scene, rig, tt).value)));
    const V3d tip = armTip(kin, motionTargets(d.scene, rig, end).value);
    CHECK(std::hypot(tip[0] - 0.45, tip[1], tip[2] - 0.3) < 0.002);
    // A second move follows on from the first, and HOME brings it back.
    REQUIRE(run(d, "ARM MOVE_TO 0,-400,400 POINT 0,0,150").ok);
    CHECK(shoulder->keys.back().t > end);
    const float end2 = shoulder->keys.back().t;
    const V3d tip2 = armTip(kin, motionTargets(d.scene, rig, end2).value);
    CHECK(std::hypot(tip2[0], tip2[1] + 0.4, tip2[2] - 0.4) < 0.002);
    REQUIRE(run(d, "ARM HOME").ok);
    const std::vector<double> home = motionTargets(d.scene, rig, shoulder->keys.back().t + 0.1f).value;
    for (double v : home) CHECK_NEAR(v, 0.0, 1e-4);
    // UNDO takes a move back.
    const std::size_t keys = shoulder->keys.size();
    REQUIRE(run(d, "UNDO").ok);
    for (const Motion& m : d.scene.motions())
        if (m.name == rig.joints[1].name) CHECK(m.keys.size() < keys);
    // Something it can't get to without hitting the block.
    CHECK(!run(d, "ARM MOVE_TO 300,0,670 POINT 0,0,150").ok);
}

TEST_CASE("Robotics arms: inverse dynamics and the least-effort trajectory") {
    Document d;
    makeArm(d);
    RobotRig rig;
    REQUIRE(rigOf(d, "", rig));
    const RobotKinematics kin(rig);
    // Holding the arm out level: the shoulder carries both links' weight about its axis.
    std::vector<double> q{0, 3.14159265358979 / 2, 0}, zero(3, 0.0);
    const std::vector<double> tau = inverseDynamics(kin, q, zero, zero);
    const RigLink &upper = rig.links[2], &fore = rig.links[3];
    CHECK_NEAR(std::fabs(tau[1]), 9.80665 * (upper.mass * (upper.com[2] - 0.16) + fore.mass * (fore.com[2] - 0.16)), 1e-6 * (1 + std::fabs(tau[1])));
    CHECK_NEAR(tau[0], 0.0, 1e-9);
    // Accelerating the turret from rest: I * alpha about Z.
    std::vector<double> acc{1.0, 0, 0};
    const double spin = inverseDynamics(kin, zero, zero, acc, {0, 0, 0})[0];
    double Izz = 0;
    for (std::size_t l = 1; l < rig.links.size(); ++l) {
        const RigLink& L = rig.links[l];
        Izz += L.inertia[8] + L.mass * (L.com[0] * L.com[0] + L.com[1] * L.com[1]);
    }
    CHECK_NEAR(spin, Izz, 1e-6 * Izz);
    // A move through a high pose: optimising lowers the effort, keeps the ends and the time.
    const std::vector<std::vector<double>> path{zero, {1.5, 0.3, 1.2}, {2.5, 1.4, 0.4}};
    const TimedPath plain = timePath(kin, path, 0.4);
    CHECK(plain.effort > 0);
    const CollisionModel col(d.scene, rig, 0.0f);
    const TimedPath best = optimisePath(kin, col, plain, kin.chain(rig.endEffectors().front()), 20);
    CHECK(best.effort < plain.effort * 0.97);
    CHECK_NEAR(best.times.back(), plain.times.back(), 1e-9);
    for (std::size_t j = 0; j < 3; ++j) {
        CHECK_NEAR(best.q.front()[j], 0.0, 1e-12);
        CHECK_NEAR(best.q.back()[j], path.back()[j], 1e-12);
    }
    CHECK_NEAR(pathEffort(kin, best), best.effort, 1e-9 * best.effort);
    // RIG ENGINE OPTIMIZE (or OPTIMIZE) plans that way from the command.
    REQUIRE(run(d, "RIG ENGINE OPTIMIZE").ok);
    const CommandOutput o = run(d, "ARM MOVE_TO 300,300,300 POINT 0,0,150");
    REQUIRE(o.ok);
    bool said = false;
    for (const std::string& l : o.lines) said = said || l.find("optimised") != std::string::npos;
    CHECK(said);
}

TEST_CASE("Robotics learning: batched environments step in parallel, the same for the same seed") {
    Document d;
    makeArm(d);
    RigSettings s;
    EnvOptions o;
    o.count = 6;
    o.task = EnvTask::Reach;
    o.target = {0.3, 0.2, 0.3};
    o.episodeSeconds = 0.1; // five actions
    o.seed = 3;
    std::string err;
    auto env = RobotEnv::create(d.scene, s, o, &err);
    REQUIRE(env != nullptr);
    CHECK(env->count() == 6);
    CHECK(env->actionSize() == 3);
    CHECK(env->observationSize() == 2 * 3 + 10 + 3);
    CHECK(env->observationNames().size() == env->observationSize());
    const std::vector<double> obs = env->reset();
    REQUIRE(obs.size() == 6 * env->observationSize());
    // The tool (the forearm's centre) starts at (0, 0, 610) mm: its offset from the target
    // is the last three values.
    const std::size_t no = env->observationSize();
    CHECK_NEAR(obs[no - 3], -0.3, 0.03);
    CHECK_NEAR(obs[no - 1], 0.31, 0.03);
    // Copies with the same actions run the same whatever the threads; episodes restart.
    std::vector<double> act(6 * 3);
    for (std::size_t k = 0; k < act.size(); ++k) act[k] = std::sin(0.7 * static_cast<double>(k));
    EnvOptions single = o;
    single.threads = 1;
    auto env1 = RobotEnv::create(d.scene, s, single, &err);
    REQUIRE(env1 != nullptr);
    env1->reset();
    for (int k = 0; k < 4; ++k) {
        const EnvStep a = env->step(act), b = env1->step(act);
        for (std::size_t i = 0; i < a.observations.size(); ++i) CHECK(a.observations[i] == b.observations[i]);
        for (std::size_t i = 0; i < 6; ++i) CHECK(a.rewards[i] == b.rewards[i]);
        CHECK(a.episodes.empty());
    }
    const EnvStep last = env->step(act);
    CHECK(last.episodes.size() == 6);
    for (std::size_t i = 0; i < 6; ++i) CHECK(last.truncated[i] && !last.terminated[i]);
    CHECK_NEAR(last.observations[no - 1], 0.31, 0.03); // started again
    CHECK(env->simulatedSeconds() > 6 * 0.1 - 1e-9);
    // Driving the arm toward the target pays: its reward rises above holding still.
    std::vector<double> hold(6 * 3, 0.0), toward(6 * 3, 0.0);
    for (int i = 0; i < 6; ++i) {
        toward[static_cast<std::size_t>(i * 3)] = 0.19;  // turret ~34 deg toward +Y
        toward[static_cast<std::size_t>(i * 3 + 1)] = 0.4; // shoulder forward
        toward[static_cast<std::size_t>(i * 3 + 2)] = 0.5; // elbow bent
    }
    o.episodeSeconds = 10;
    auto e2 = RobotEnv::create(d.scene, s, o, &err);
    REQUIRE(e2 != nullptr);
    double still = 0, moved = 0;
    for (int k = 0; k < 40; ++k) still = e2->step(hold).rewards[0];
    e2->reset();
    for (int k = 0; k < 40; ++k) moved = e2->step(toward).rewards[0];
    CHECK(moved > still + 0.1);
}

TEST_CASE("Robotics learning: a humanoid stands on its motors and falls limp; the MCP tool") {
    Document d;
    REQUIRE(run(d, "PART humanoid").ok);
    RigSettings s;
    s.name = "Male Humanoid Robot 1";
    EnvOptions o;
    o.count = 2;
    o.noise = 0;
    std::string err;
    auto stand = RobotEnv::create(d.scene, s, o, &err);
    REQUIRE(stand != nullptr);
    const std::vector<double> zero(2 * stand->actionSize(), 0.0);
    bool fell = false;
    for (int k = 0; k < 50; ++k) {
        const EnvStep st = stand->step(zero);
        fell = fell || st.terminated[0] || st.terminated[1];
        CHECK(st.rewards[0] > 0.9);
    }
    CHECK(!fell);
    o.torque = true; // no torque at all: it collapses
    auto limp = RobotEnv::create(d.scene, s, o, &err);
    REQUIRE(limp != nullptr);
    fell = false;
    for (int k = 0; k < 150 && !fell; ++k) fell = limp->step(zero).terminated[0] != 0;
    CHECK(fell);

    // The same through MCP, as python/sicdm_env.py drives it.
    McpServer srv(d, nullptr);
    auto callTool = [&](const std::string& args) {
        const auto reply = Json::parse(srv.handle(R"({"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"robot_env","arguments":)" + args + "}}"));
        return reply ? (*reply)["result"] : Json();
    };
    const Json created = callTool(R"({"op":"create","count":3,"task":"stand","seed":5})");
    CHECK(!created["isError"].asBool());
    const auto info = Json::parse(created["content"][0]["text"].asString());
    REQUIRE(info.has_value());
    const int id = (*info)["id"].asInt();
    const std::size_t na = static_cast<std::size_t>((*info)["action_size"].asInt()), no = static_cast<std::size_t>((*info)["observation_size"].asInt());
    CHECK(na == stand->actionSize());
    CHECK((*info)["observations"].size() == 3 * no);
    std::string acts = "[";
    for (std::size_t k = 0; k < 3 * na; ++k) acts += (k ? ",0" : "0");
    acts += "]";
    const Json stepped = callTool(R"({"op":"step","id":)" + std::to_string(id) + R"(,"actions":)" + acts + "}");
    const auto st = Json::parse(stepped["content"][0]["text"].asString());
    REQUIRE(st.has_value());
    CHECK((*st)["rewards"].size() == 3);
    CHECK((*st)["observations"].size() == 3 * no);
    CHECK((*st)["rewards"][0].asNumber() > 0.9);
    CHECK(!callTool(R"({"op":"close","id":)" + std::to_string(id) + "}")["isError"].asBool());
    CHECK(callTool(R"({"op":"step","id":)" + std::to_string(id) + R"(,"actions":[]})")["isError"].asBool());
    CHECK(callTool(R"({"op":"create","task":"fly"})")["isError"].asBool());
    // And the headless batch from the command line.
    const CommandOutput bench = run(d, "RIG ENV COUNT 4 TASK STAND SECONDS 0.2");
    REQUIRE(bench.ok);
    CHECK(bench.lines[0].find("4 copies") != std::string::npos);
}

namespace {

// A 200 x 200 x 100 mm block (4 kg) on the floor with a small knob jointed on top, free to
// move (a floating base).
void makeFloater(Document& d) {
    REQUIRE(run(d, "BOX 200 200 100 @0,0,0").ok);
    REQUIRE(run(d, "BOX 20 20 20 @0,0,100").ok);
    REQUIRE(run(d, "JOINT \"Block 2\" TO \"Block 1\" AT 0,0,100 AXIS Z RANGE -10 10").ok);
    for (Motion& m : d.scene.motions()) m.keys = {{0.0f, 0.0f}};
    REQUIRE(run(d, "RIG BASE FLOATING").ok);
}

double comHeightAt(const Document& d, float t) {
    const auto r = robotRun(d.scene, d.scene.rigs().robots[0], t);
    return r ? r->at(t).com[2] : -1.0;
}

} // namespace

TEST_CASE("Robotics sandbox: force expressions") {
    ForceExpression e;
    REQUIRE(e.compile("2*sin(pi/2) + 3^2 - -1"));
    const double none[12] = {};
    CHECK_NEAR(e.eval(none), 12.0, 1e-12);
    REQUIRE(e.compile("mass*g + clamp(z, 0, 1)*max(vx, 2) / 4 + step(t - 1) * atan2(1, 1)"));
    double v[12] = {};
    v[0] = 2, v[3] = 0.5, v[4] = 1, v[10] = 3, v[11] = 9.8; // t, z, vx, mass, g
    CHECK_NEAR(e.eval(v), 3 * 9.8 + 0.5 * 2 / 4 + 3.14159265358979 / 4, 1e-9);
    CHECK_NEAR((e.compile("2^3^2") ? e.eval(none) : 0), 512.0, 1e-9); // right-associative
    std::string err;
    CHECK(!e.compile("foo(1)", &err));
    CHECK(err.find("foo") != std::string::npos);
    CHECK(!e.compile("1 +", &err));
    CHECK(!e.compile("(1", &err));
    CHECK(!e.compile("speed * 2", &err));
    CHECK(!e.compile("min(1)", &err));
    CHECK(e.compile("1/0") && e.eval(none) == 0.0); // no infinities into the engine
}

TEST_CASE("Robotics sandbox: a block floats at its waterline and a script holds it up") {
    Document d;
    makeFloater(d);
    // In a fluid twice its density it floats half under: its centre at the surface.
    REQUIRE(run(d, "RIG FORCE ADD BUOYANCY water ON \"Block 1\" DENSITY 2000 LEVEL 300 DRAG 60").ok);
    CHECK_NEAR(comHeightAt(d, 8.0f), 0.300, 0.004);
    REQUIRE(run(d, "RIG FORCE REMOVE water").ok);
    // Lighter than the block, it stays on the floor.
    REQUIRE(run(d, "RIG FORCE ADD BUOYANCY oil ON \"Block 1\" DENSITY 800 LEVEL 300").ok);
    CHECK_NEAR(comHeightAt(d, 2.0f), 0.050, 0.004);
    REQUIRE(run(d, "RIG FORCE REMOVE oil").ok);
    // A scripted lift: a PD hold at 0.5 m on top of its weight.
    REQUIRE(run(d, "RIG FORCE ADD SCRIPT lift ON \"Block 1\" FZ \"(mass + 0.008)*g + (0.5 - z)*400 - vz*60\"").ok);
    CHECK_NEAR(comHeightAt(d, 5.0f), 0.5, 0.01);
    CHECK(!run(d, "RIG FORCE ADD SCRIPT bad ON \"Block 1\" FZ \"2*\"").ok);
    CHECK(!run(d, "RIG FORCE ADD WRENCH w ON \"Nothing\" FORCE 0,0,1").ok);
    // Saved with the project.
    const std::string path = (fs::temp_directory_path() / "sicdm_forces.sicdm").string();
    REQUIRE(run(d, "SAVE \"" + path + "\"").ok);
    Document e;
    REQUIRE(run(e, "OPEN \"" + path + "\"").ok);
    REQUIRE(!e.scene.rigs().robots.empty());
    REQUIRE(e.scene.rigs().robots[0].forces.size() == 1);
    CHECK(e.scene.rigs().robots[0].forces[0].script[2] == "(mass + 0.008)*g + (0.5 - z)*400 - vz*60");
    fs::remove(path);
}

TEST_CASE("Robotics sandbox: magnetic adhesion, timed wrenches and C++ forces") {
    Document d;
    makeFloater(d);
    // 60 N up lifts the 4 kg block off the floor...
    REQUIRE(run(d, "RIG FORCE ADD WRENCH pull ON \"Block 1\" FORCE 0,0,60").ok);
    CHECK(comHeightAt(d, 1.0f) > 0.5);
    // ...unless a 50 N magnet holds it there.
    REQUIRE(run(d, "RIG FORCE ADD MAGNET grip ON \"Block 1\" STRENGTH 50 RANGE 20").ok);
    CHECK_NEAR(comHeightAt(d, 1.0f), 0.050, 0.003);
    REQUIRE(run(d, "RIG FORCE REMOVE grip").ok);
    REQUIRE(run(d, "RIG FORCE REMOVE pull").ok);
    // A wrench for a time only: up for half a second, then it falls back.
    REQUIRE(run(d, "RIG FORCE ADD WRENCH kick ON \"Block 1\" FORCE 0,0,80 UNTIL 0.5").ok);
    CHECK(comHeightAt(d, 0.5f) > 0.2);
    CHECK_NEAR(comHeightAt(d, 4.0f), 0.050, 0.003);
    REQUIRE(run(d, "RIG FORCE REMOVE kick").ok);
    // A force written in C++: its weight and a bit more, registered under a name.
    registerForceScript("test lift", [](const ForceContext& c, V3d& f, V3d&) { f = {0, 0, (c.mass + 0.008) * 9.80665 + 2.0}; });
    REQUIRE(run(d, "RIG FORCE ADD SCRIPT cpp ON \"Block 1\" FUNCTION \"test lift\"").ok);
    CHECK(comHeightAt(d, 2.0f) > 0.5);
    unregisterForceScript("test lift");
    CHECK(run(d, "RIG FORCE LIST").lines.size() == 1);

    // The learning environments get the forces too, and wrenches from the client per step.
    REQUIRE(run(d, "RIG FORCE REMOVE cpp").ok);
    EnvOptions o;
    o.task = EnvTask::None;
    std::string err;
    auto env = RobotEnv::create(d.scene, d.scene.rigs().robots[0], o, &err);
    REQUIRE(env != nullptr);
    const std::size_t hIndex = 2 * env->actionSize();
    EnvWrench up;
    up.link = 0;
    up.force = {0, 0, 80};
    up.point = {0, 0, 0.05};
    std::vector<double> obs;
    for (int k = 0; k < 25; ++k) obs = env->step({0.0}, {up}).observations;
    CHECK(obs[hIndex] > 0.3);
}

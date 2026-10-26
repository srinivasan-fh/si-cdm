#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/FileIO.hpp"
#include "core/Multibody.hpp"
#include "core/Physics.hpp"
#include "core/Rig.hpp"
#include "core/Scene.hpp"

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

#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/FileIO.hpp"
#include "core/Motion.hpp"
#include "core/Scene.hpp"

#include <filesystem>

using namespace sicdm;

#define REQUIRE(c)               \
    do {                         \
        const bool required_ = (c); \
        CHECK(required_);        \
        if (!required_) return;  \
    } while (0)

namespace {

Motion circular(float speed, float range = 0, MotionMode mode = MotionMode::Continuous) {
    Motion m;
    m.name = "Spin";
    m.bodies = {"*"};
    m.speed = speed;
    m.range = range;
    m.mode = mode;
    return m;
}

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }

} // namespace

TEST_CASE("rotation about an arbitrary axis follows the right-hand rule") {
    const Mat4 r = rotateAxis({0, 0, 1}, radians(90));
    const Vec3 p = r.transformPoint({1, 0, 0});
    CHECK_NEAR(p.x, 0, 1e-5);
    CHECK_NEAR(p.y, 1, 1e-5);
    const Vec3 q = rotateAxis({1, 1, 1}, radians(120)).transformPoint({1, 0, 0}); // cycles the axes
    CHECK_NEAR(q.y, 1, 1e-5);
    CHECK_NEAR(q.x, 0, 1e-5);
}

TEST_CASE("motion values for continuous, loop, oscillate and once modes") {
    CHECK_NEAR(motionValue(circular(90), 2.0f), 180, 1e-4);
    CHECK_NEAR(motionValue(circular(-90), 1.0f), -90, 1e-4);
    CHECK_NEAR(motionValue(circular(90, 100, MotionMode::Loop), 1.5f), 35, 1e-3);
    const Motion osc = circular(10, 20, MotionMode::Oscillate);
    CHECK_NEAR(motionValue(osc, 1.5f), 15, 1e-4);
    CHECK_NEAR(motionValue(osc, 3.0f), 10, 1e-4); // on the way back
    CHECK_NEAR(motionRate(osc, 1.0f), 10, 1e-4);
    CHECK_NEAR(motionRate(osc, 3.0f), -10, 1e-4);
    CHECK_NEAR(motionCycle(osc), 4, 1e-4);
    const Motion once = circular(10, 20, MotionMode::Once);
    CHECK_NEAR(motionValue(once, 9.0f), 20, 1e-4);
    CHECK_NEAR(motionRate(once, 9.0f), 0, 1e-6);
    Motion late = circular(10);
    late.start = 2.0f;
    CHECK_NEAR(motionValue(late, 1.0f), 0, 1e-6);
    CHECK_NEAR(motionValue(late, 3.0f), 10, 1e-4);
    CHECK_NEAR(motionValue(circular(90, 100, MotionMode::Continuous), 2.0f), 180, 1e-4); // range ignored
}

TEST_CASE("circular motion rotates about its origin; chained motions carry children") {
    Scene s;
    s.add(makeBody("Chassis", "box", {-20, -10, 0, 20, 10, 10}));
    s.add(makeBody("Wheel", "cylinder", {5, 4}, {0, 20, 5}));
    Motion drive;
    drive.name = "Drive";
    drive.kind = MotionKind::Linear;
    drive.axis = {1, 0, 0};
    drive.speed = 10;
    drive.bodies = {"*"};
    Motion spin = circular(90);
    spin.bodies = {"Wheel"};
    spin.axis = {0, 1, 0};
    spin.origin = {0, 20, 5};
    s.motions() = {drive, spin};

    const auto tf = motionTransforms(s, 1.0f);
    CHECK_NEAR(tf[0].transformPoint({0, 0, 0}).x, 10, 1e-4); // chassis only drives
    const Vec3 hub = tf[1].transformPoint({0, 20, 5});
    CHECK_NEAR(hub.x, 10, 1e-4); // the hub travels with the chassis...
    CHECK_NEAR(hub.z, 5, 1e-4);
    const Vec3 rim = tf[1].transformPoint({5, 20, 5}); // ...and the rim turns 90 deg about +Y
    CHECK_NEAR(rim.x, 10, 1e-3);
    CHECK_NEAR(rim.z, 0, 1e-3);
    CHECK(isMoving(s, 1));
    s.motions()[0].enabled = false;
    s.motions()[1].enabled = false;
    CHECK(!isMoving(s, 0));
    CHECK_NEAR(suggestedDuration(s), 10, 1e-6);
}

TEST_CASE("MOTION and SIM commands create, update and report motions") {
    Document d;
    CHECK(run(d, "CYLINDER 10 5 @0,0,0").ok);
    CHECK(run(d, "BOX 40 20 10").ok);
    const std::string wheel = d.scene.bodies()[0].name;
    CHECK(!run(d, "MOTION CIRCLE AXIS Q").ok);
    CHECK(run(d, "MOTION CIRCLE \"Wheel spin\" AXIS Y RPM 30 BODIES \"" + wheel + "\"").ok);
    REQUIRE(d.scene.motions().size() == 1);
    const Motion& m = d.scene.motions()[0];
    CHECK_NEAR(m.speed, 180, 1e-4);
    CHECK_NEAR(m.axis.y, 1, 1e-6);
    CHECK_NEAR(m.origin.z, 2.5, 1e-4); // centre of the cylinder
    CHECK(run(d, "MOTION LINEAR Drive DIR X SPEED 25 STROKE 100 BODIES ALL").ok);
    REQUIRE(d.scene.motions().size() == 2);
    CHECK(d.scene.motions()[1].mode == MotionMode::Oscillate); // a stroke implies back and forth
    CHECK(run(d, "MOTION LINEAR Drive SPEED 50 LOOP").ok); // update keeps the rest
    CHECK(d.scene.motions().size() == 2);
    CHECK_NEAR(d.scene.motions()[1].speed, 50, 1e-6);
    CHECK_NEAR(d.scene.motions()[1].range, 100, 1e-6);
    CHECK(d.scene.motions()[1].mode == MotionMode::Loop);
    CHECK(!run(d, "MOTION LINEAR BODIES Nobody").ok);

    const CommandOutput poses = run(d, "SIM AT 1");
    CHECK(poses.ok);
    CHECK(poses.lines.size() == 5); // time, two motions, two moving bodies
    CHECK(!run(d, "SIM PLAY").ok); // needs the window
    CHECK(run(d, "SIM DURATION 8").ok);
    CHECK_NEAR(d.state.simDuration, 8, 1e-6);

    CHECK(run(d, "MOTION OFF Drive").ok);
    CHECK(!d.scene.motions()[1].enabled);
    CHECK(run(d, "MOTION DELETE \"Wheel spin\"").ok);
    CHECK(d.scene.motions().size() == 1);
    CHECK(run(d, "MOTION").lines.size() == 1);
}

TEST_CASE("motions survive save and reopen") {
    Document d;
    CHECK(run(d, "BOX 10 10 10").ok);
    CHECK(run(d, "MOTION CIRCLE Turntable AXIS 0,0,1 AT 5,5,0 SPEED -45 RANGE 90 ONCE START 1.5 BODIES ALL").ok);
    CHECK(run(d, "SIM DURATION 12").ok);
    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_motion_test.sicdm").string();
    std::string err;
    REQUIRE(saveProject(d.scene, d.state, path, &err));
    Scene s;
    ProjectState st;
    REQUIRE(loadProject(s, st, path, &err, nullptr));
    REQUIRE(s.motions().size() == 1);
    const Motion& m = s.motions()[0];
    CHECK(m.name == "Turntable");
    CHECK(m.kind == MotionKind::Circular);
    CHECK(m.mode == MotionMode::Once);
    CHECK_NEAR(m.speed, -45, 1e-5);
    CHECK_NEAR(m.range, 90, 1e-5);
    CHECK_NEAR(m.start, 1.5, 1e-5);
    CHECK_NEAR(m.origin.x, 5, 1e-5);
    CHECK(m.bodies.size() == 1 && m.bodies[0] == "*");
    CHECK_NEAR(st.simDuration, 12, 1e-5);
}

TEST_CASE("FOLLOW rolls a linear drive with its wheel spin") {
    Document d;
    CHECK(run(d, "CYLINDER 50 20 @0,0,0").ok); // 100 mm across: radius 50 along X
    CHECK(run(d, "BOX 200 100 50 @0,0,100").ok);
    CHECK(run(d, "MOTION LINEAR Drive AXIS X BODIES ALL").ok);
    CHECK(!run(d, "MOTION LINEAR Drive FOLLOW Nothing").ok);
    CHECK(run(d, "MOTION CIRCLE Spin AXIS Y SPEED 90 EACH BODIES \"Cylinder 1\"").ok);
    CHECK(run(d, "MOTION LINEAR Drive FOLLOW Spin").ok);
    REQUIRE(d.scene.motions()[0].follow == "Spin");
    const Motion& drive = d.scene.motions()[0];
    CHECK_NEAR(motionValue(d.scene, drive, 2.0f), kPi * 50, 1e-2); // 180 deg of a 50 mm wheel
    CHECK_NEAR(motionRate(d.scene, drive, 2.0f), kPi / 2 * 50, 1e-2);
    CHECK_NEAR(bodyMotionTransform(d.scene, 1, 2.0f).transformPoint({0, 0, 0}).x, kPi * 50, 1e-2);
    CHECK(run(d, "MOTION LINEAR Drive RADIUS 10").ok);
    CHECK_NEAR(motionValue(d.scene, drive, 2.0f), kPi * 10, 1e-3);
    CHECK(run(d, "MOTION OFF Spin").ok); // wheels stopped: the body stays
    CHECK_NEAR(motionValue(d.scene, drive, 2.0f), 0, 1e-6);
    CHECK(run(d, "MOTION LINEAR Drive FOLLOW NONE").ok);
    CHECK(drive.follow.empty());
}

TEST_CASE("EACH turns every body about its own centre") {
    Document d;
    CHECK(run(d, "CYLINDER 5 4 @-20,0,0").ok);
    CHECK(run(d, "CYLINDER 5 4 @20,0,0").ok);
    CHECK(run(d, "MOTION CIRCLE Wheels AXIS Z SPEED 90 EACH BODIES ALL").ok);
    REQUIRE(d.scene.motions().size() == 1);
    CHECK(d.scene.motions()[0].eachCentre);
    const auto tf = motionTransforms(d.scene, 1.0f);
    for (std::size_t i = 0; i < 2; ++i) { // the hubs stay put
        const Vec3 c = d.scene.bodies()[i].bounds().center();
        const Vec3 p = tf[i].transformPoint(c);
        CHECK_NEAR(p.x, c.x, 1e-4);
        CHECK_NEAR(p.y, c.y, 1e-4);
    }
    CHECK_NEAR(bodyMotionTransform(d.scene, 1, 1.0f).transformPoint({25, 0, 2}).y, 5, 1e-4);
    CHECK(run(d, "MOTION CIRCLE Wheels AT 0,0,0").ok); // an explicit axis point shares it again
    CHECK(!d.scene.motions()[0].eachCentre);
}

TEST_CASE("keyframed motions settle exactly on every key with minimum-jerk curves") {
    Motion m;
    m.keys = {{0, 0}, {1, 90}, {1.5f, 90}, {2.5f, -30}};
    CHECK(motionValue(m, 0) == 0.0f);
    CHECK_NEAR(motionValue(m, 1), 90, 1e-4);
    CHECK_NEAR(motionValue(m, 1.25f), 90, 1e-4); // held
    CHECK_NEAR(motionValue(m, 2.5f), -30, 1e-4);
    CHECK_NEAR(motionValue(m, 9), -30, 1e-4);    // holds the last key
    CHECK_NEAR(motionValue(m, 0.5f), 45, 1e-3);  // halfway at mid-time
    for (float t : {0.0f, 1.0f, 1.5f, 2.5f}) CHECK_NEAR(motionRate(m, t), 0, 1e-3); // stops on each key
    CHECK_NEAR(motionRate(m, 0.5f), 1.875f * 90, 1e-2);                              // peak at mid-segment
    CHECK_NEAR(keyedPeakRate(m), 1.875f * 120, 1e-3);
    CHECK_NEAR(motionCycle(m), 2.5f, 1e-6);
    // Numeric derivative matches the analytic rate.
    for (float t = 0.05f; t < 2.5f; t += 0.13f)
        CHECK_NEAR((motionValue(m, t + 1e-3f) - motionValue(m, t - 1e-3f)) / 2e-3f, motionRate(m, t), 0.5);
    m.mode = MotionMode::Loop;
    m.start = 1.0f;
    CHECK_NEAR(motionValue(m, 1.0f + 2.5f + 1.0f), 90, 1e-3); // second cycle
    CHECK(motionValue(m, 0.5f) == 0.0f);                       // before its start
}

TEST_CASE("joint limits are hard stops for followers and keys") {
    Document d;
    REQUIRE(run(d, "BOX 10 10 10").ok);
    REQUIRE(run(d, "MOTION CIRCLE Motor SPEED 90 BODIES ALL").ok);
    REQUIRE(run(d, "MOTION CIRCLE Joint FOLLOW Motor RATIO 0.5 LIMITS -10 30 BODIES ALL").ok);
    const Motion* j = nullptr;
    for (const Motion& m : d.scene.motions())
        if (m.name == "Joint") j = &m;
    REQUIRE(j);
    CHECK(j->hasLimits);
    CHECK_NEAR(motionValue(d.scene, *j, 0.5f), 22.5, 1e-3);
    CHECK_NEAR(motionValue(d.scene, *j, 2.0f), 30, 1e-4); // stopped at the hard stop
    CHECK_NEAR(motionRate(d.scene, *j, 2.0f), 0, 1e-6);
    REQUIRE(run(d, "MOTION CIRCLE Joint KEYS 0:0,1:50,2:-40 MODE ONCE").ok);
    CHECK(j->keys.size() == 3);
    CHECK_NEAR(motionValue(d.scene, *j, 1.0f), 30, 1e-4);  // a key past the limit is clamped
    CHECK_NEAR(motionValue(d.scene, *j, 2.0f), -10, 1e-4);
    CHECK(!run(d, "MOTION CIRCLE Joint KEYS 0:0,x:5").ok);
    CHECK(!run(d, "MOTION CIRCLE Joint LIMITS 5 1").ok);
    REQUIRE(run(d, "MOTION CIRCLE Joint SPEED 20").ok); // a new speed replaces the keys
    CHECK(j->keys.empty());
    REQUIRE(run(d, "MOTION CIRCLE Joint LIMITS OFF").ok);
    CHECK(!j->hasLimits);
}

TEST_CASE("keys and limits are saved with the project") {
    Document d;
    REQUIRE(run(d, "BOX 10 10 10").ok);
    REQUIRE(run(d, "MOTION CIRCLE Wave KEYS 0:0,0.5:40,1:0 LOOP LIMITS -20 45 BODIES ALL").ok);
    const auto path = (std::filesystem::temp_directory_path() / "sicdm_keys_test.sicdm").string();
    std::string err;
    REQUIRE(saveProject(d.scene, d.state, path, &err));
    Scene s;
    ProjectState st;
    REQUIRE(loadProject(s, st, path, &err));
    REQUIRE(s.motions().size() == 1);
    const Motion& m = s.motions()[0];
    CHECK(m.keys.size() == 3);
    CHECK(m.mode == MotionMode::Loop);
    CHECK(m.hasLimits && m.lo == -20.0f && m.hi == 45.0f);
    CHECK_NEAR(motionValue(m, 1.5f), 40, 1e-3);
    std::filesystem::remove(path);
}

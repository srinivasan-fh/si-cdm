#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/FileIO.hpp"
#include "core/Motion.hpp"
#include "core/Pose.hpp"
#include "core/Scene.hpp"

#include <cmath>
#include <filesystem>

using namespace sicdm;

#define REQUIRE(c)               \
    do {                         \
        const bool required_ = (c); \
        CHECK(required_);        \
        if (!required_) return;  \
    } while (0)

namespace {

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }

const Motion* motion(const Scene& s, const std::string& name) {
    for (const Motion& m : s.motions())
        if (m.name == name) return &m;
    return nullptr;
}

float angle(const Scene& s, const std::string& name, float t) {
    const Motion* m = motion(s, name);
    return m ? motionValue(s, *m, t) : NAN;
}

bool said(const CommandOutput& o, const std::string& text) {
    for (const std::string& l : o.lines)
        if (l.find(text) != std::string::npos) return true;
    return false;
}

// Lowest point of the robot's soles at time t.
float soleBottom(const Scene& s, float t) {
    const auto movers = bodyMotions(s);
    float lowest = 1e30f;
    for (std::size_t i = 0; i < s.bodies().size(); ++i) {
        const Body& b = s.bodies()[i];
        if (b.name.size() < 5 || b.name.compare(b.name.size() - 5, 5, " Sole") != 0) continue;
        const Mat4 tf = bodyMotionTransform(s, i, t, static_cast<std::size_t>(-1), movers[i]);
        for (const Vertex& v : b.mesh.vertices) lowest = std::min(lowest, tf.transformPoint(v.position + b.offset).z);
    }
    return lowest;
}

float bodyZ(const Scene& s, const std::string& name, float t) {
    for (std::size_t i = 0; i < s.bodies().size(); ++i)
        if (s.bodies()[i].name == name) return bodyMotionTransform(s, i, t).transformPoint(s.bodies()[i].bounds().center()).z;
    return NAN;
}

// Highest drive speed (deg/s) against its rated speed, sampled over [0, end].
bool withinRatedSpeed(const Scene& s, const Robot& r, float end) {
    for (const RobotJoint& j : r.joints) {
        const Motion* d = motion(s, j.drive);
        if (!d) continue;
        for (float t = 0.0f; t <= end; t += 0.005f)
            if (std::fabs(motionRate(s, *d, t)) > std::fabs(d->speed) * 1.001f + 1e-3f) return false;
    }
    return true;
}

} // namespace

TEST_CASE("robot joints get standard ids and their hard stops") {
    Document d;
    REQUIRE(run(d, "PART humanoid").ok);
    const auto r = findRobot(d.scene, "Humanoid Robot 1");
    REQUIRE(r.has_value());
    auto joint = [&](const std::string& id) -> const RobotJoint* {
        for (const RobotJoint& j : r->joints)
            if (j.id == id) return &j;
        return nullptr;
    };
    for (const char* id : {"neck yaw", "neck pitch", "neck roll", "waist rotation", "trunk flex", "right shoulder flex",
                           "left shoulder abduction", "right elbow", "left forearm rotation", "right wrist flex",
                           "right index mcp", "left thumb ip", "right hip flex", "left knee", "right ankle pitch", "left toe"})
        CHECK(joint(id) != nullptr);
    REQUIRE(joint("right elbow") && joint("right knee") && joint("neck yaw"));
    CHECK(joint("right elbow")->lo == 0.0f && joint("right elbow")->hi == 150.0f);
    CHECK(joint("neck yaw")->lo == -80.0f && joint("neck yaw")->hi == 80.0f);
    CHECK(robotNames(d.scene) == std::vector<std::string>{"Humanoid Robot 1"});
    // A body of the robot finds it too.
    CHECK(findRobot(d.scene, "Humanoid Robot 1 Head").has_value());
}

TEST_CASE("POSE reaches the exact joint angles on a minimum-jerk path within the motors' speed") {
    Document d;
    REQUIRE(run(d, "PART humanoid").ok);
    const CommandOutput o = run(d, "POSE squat");
    REQUIRE(o.ok);
    const auto r = findRobot(d.scene, "Humanoid Robot 1");
    REQUIRE(r.has_value());
    const Motion* drive = motion(d.scene, "Humanoid Robot 1 Motor Right Knee Drive");
    REQUIRE(drive != nullptr && drive->keys.size() == 2);
    const float end = drive->keys.back().t;
    CHECK(std::fabs(angle(d.scene, "Humanoid Robot 1 Right Knee", end) - 110.0f) < 1e-3f);
    CHECK(std::fabs(angle(d.scene, "Humanoid Robot 1 Right Hip Flex", end) - 90.0f) < 1e-3f);
    CHECK(std::fabs(angle(d.scene, "Humanoid Robot 1 Left Ankle Pitch", end + 1.0f) + 20.0f) < 1e-3f); // holds
    CHECK(std::fabs(angle(d.scene, "Humanoid Robot 1 Right Knee", 0.0f)) < 1e-3f);
    // Minimum jerk: halfway at half time, starts and ends at rest.
    CHECK(std::fabs(angle(d.scene, "Humanoid Robot 1 Right Knee", end / 2) - 55.0f) < 0.01f);
    CHECK(std::fabs(motionRate(d.scene, *drive, 0.0f)) < 1e-3f);
    CHECK(std::fabs(motionRate(d.scene, *drive, end)) < 1e-3f);
    CHECK(withinRatedSpeed(d.scene, *r, end));
    // The feet stay on the floor; the body sinks.
    const float ground = soleBottom(d.scene, 0.0f);
    for (float t = 0.0f; t <= end + 0.5f; t += end / 10)
        CHECK(std::fabs(soleBottom(d.scene, t) - ground) < 1.0f);
    CHECK(bodyZ(d.scene, "Humanoid Robot 1 Head", end) < bodyZ(d.scene, "Humanoid Robot 1 Head", 0.0f) - 150.0f);
    // Back to standing from the squat it holds.
    REQUIRE(run(d, "POSE stand").ok);
    const float back = motion(d.scene, "Humanoid Robot 1 Motor Right Knee Drive")->keys.back().t;
    CHECK(std::fabs(angle(d.scene, "Humanoid Robot 1 Right Knee", 0.0f) - 110.0f) < 1e-3f);
    CHECK(std::fabs(angle(d.scene, "Humanoid Robot 1 Right Knee", back)) < 1e-3f);
    CHECK(std::fabs(soleBottom(d.scene, back / 3) - ground) < 1.0f);
}

TEST_CASE("POSE clamps to hard stops, respects motor speed and parses presets with joints") {
    Document d;
    REQUIRE(run(d, "PART humanoid").ok);
    CommandOutput o = run(d, "POSE right elbow 200 in 0.05");
    REQUIRE(o.ok);
    CHECK(said(o, "past its hard stop: 150"));
    CHECK(said(o, "the motors cannot go faster"));
    const Motion* drive = motion(d.scene, "Humanoid Robot 1 Right Arm Motor Elbow Drive");
    REQUIRE(drive != nullptr && !drive->keys.empty());
    const float end = drive->keys.back().t;
    CHECK(std::fabs(angle(d.scene, "Humanoid Robot 1 Right Arm Elbow", end) - 150.0f) < 1e-3f);
    CHECK(withinRatedSpeed(d.scene, *findRobot(d.scene, "Humanoid Robot 1"), end));
    // Other joints hold still.
    CHECK(std::fabs(angle(d.scene, "Humanoid Robot 1 Left Arm Elbow", end)) < 1e-3f);
    // A slower time than the motors need is kept exactly.
    REQUIRE(run(d, "POSE right elbow 30 IN 3").ok);
    CHECK(std::fabs(motion(d.scene, "Humanoid Robot 1 Right Arm Motor Elbow Drive")->keys.back().t - 3.0f) < 1e-4f);
    // Presets and joints together; later entries win.
    o = run(d, "POSE t_pose hand_fist neck yaw 30");
    REQUIRE(o.ok);
    const float t = 10.0f;
    CHECK(std::fabs(angle(d.scene, "Humanoid Robot 1 Right Shoulder Abduction", t) - 90.0f) < 1e-3f);
    CHECK(std::fabs(angle(d.scene, "Humanoid Robot 1 Neck Yaw", t) - 30.0f) < 1e-3f);
    CHECK(angle(d.scene, "Humanoid Robot 1 Right Arm Index MCP", t) > 60.0f);
    CHECK(!run(d, "POSE nonsense").ok);
    CHECK(!run(d, "POSE 30").ok);
    CHECK(run(d, "POSE LIST").ok);
    o = run(d, "POSE JOINTS");
    CHECK(o.ok && said(o, "right knee"));
}

// World pitch / roll (deg) of a sole at time t.
std::pair<float, float> soleTilt(const Scene& s, const std::string& name, float t) {
    for (std::size_t i = 0; i < s.bodies().size(); ++i)
        if (s.bodies()[i].name == name) {
            const Mat4 tf = bodyMotionTransform(s, i, t);
            const Vec3 o = tf.transformPoint({0, 0, 0}), y = tf.transformPoint({0, 1, 0}), x = tf.transformPoint({1, 0, 0});
            return {std::asin(std::clamp(y.z - o.z, -1.0f, 1.0f)) * 57.29578f, std::asin(std::clamp(x.z - o.z, -1.0f, 1.0f)) * 57.29578f};
        }
    return {NAN, NAN};
}

TEST_CASE("the ankles keep the feet level unless a posture names them") {
    for (const char* pose : {"POSE squat", "POSE sit", "POSE balance", "POSE right hip flex 30 right knee 20",
                             "POSE hip abduction 20", "POSE left hip flex 60 left knee 40 left hip abduction 10"}) {
        Document d;
        REQUIRE(run(d, "PART humanoid").ok);
        REQUIRE(run(d, pose).ok);
        for (const char* sole : {"Humanoid Robot 1 Right Foot Sole", "Humanoid Robot 1 Left Foot Sole"})
            for (float t = 0.0f; t <= 2.0f; t += 0.05f) { // all along the move, not only at its end
                const auto [pitch, roll] = soleTilt(d.scene, sole, t);
                CHECK(std::fabs(pitch) < 0.05f);
                CHECK(std::fabs(roll) < 0.05f);
            }
    }
    {
        Document d;
        REQUIRE(run(d, "PART humanoid").ok);
        REQUIRE(run(d, "GESTURE squat").ok);
        for (float t = 0.0f; t <= 4.0f; t += 0.05f) {
            const auto [pitch, roll] = soleTilt(d.scene, "Humanoid Robot 1 Left Foot Sole", t);
            CHECK(std::fabs(pitch) < 0.05f && std::fabs(roll) < 0.05f);
        }
    }
    // Named ankles are kept; a foot that cannot lie flat says so.
    Document d;
    REQUIRE(run(d, "PART humanoid").ok);
    REQUIRE(run(d, "POSE right knee 30 right ankle pitch 10").ok);
    CHECK(std::fabs(angle(d.scene, "Humanoid Robot 1 Right Ankle Pitch", 2.0f) - 10.0f) < 1e-3f);
    CHECK(said(run(d, "POSE right knee 90"), "right foot tilts"));
}

TEST_CASE("underactuated fingers: one joint exact, a full shape best fit on the shared motor") {
    Document d;
    REQUIRE(run(d, "PART robot_hand").ok);
    REQUIRE(run(d, "POSE right index mcp 40").ok);
    CHECK(std::fabs(angle(d.scene, "Robot Hand 1 Index MCP", 10.0f) - 40.0f) < 1e-3f);
    // The coupled PIP and DIP follow the same motor.
    CHECK(angle(d.scene, "Robot Hand 1 Index PIP", 10.0f) > 30.0f);
    const CommandOutput o = run(d, "POSE hand_ok");
    REQUIRE(o.ok);
    CHECK(said(o, "share a motor"));
    // Least squares over MCP 40 / PIP 70 / DIP 50: between the asked angles.
    const float pip = angle(d.scene, "Robot Hand 1 Index PIP", 10.0f);
    CHECK(pip > 45.0f && pip < 70.0f);
}

TEST_CASE("GESTURE keys every frame exactly, holds untouched joints and mirrors sides") {
    Document d;
    REQUIRE(run(d, "PART humanoid").ok);
    REQUIRE(run(d, "GESTURE nod").ok);
    const Motion* neck = motion(d.scene, "Humanoid Robot 1 Motor Neck Pitch Drive");
    REQUIRE(neck != nullptr);
    REQUIRE(neck->keys.size() == 5);
    const Gesture* nod = findGesture("nod");
    REQUIRE(nod != nullptr);
    for (std::size_t i = 0; i < nod->frames.size(); ++i) {
        CHECK(std::fabs(neck->keys[i + 1].t - nod->frames[i].first) < 1e-4f);
        CHECK(std::fabs(angle(d.scene, "Humanoid Robot 1 Neck Pitch", neck->keys[i + 1].t) - nod->frames[i].second[0].second) <
              1e-3f);
    }
    // Joints the gesture does not move hold still.
    const Motion* knee = motion(d.scene, "Humanoid Robot 1 Motor Right Knee Drive");
    REQUIRE(knee != nullptr);
    CHECK(knee->keys.size() == 1);
    // Wave with the right arm; mirrored, with the left.
    REQUIRE(run(d, "GESTURE wave").ok);
    CHECK(std::fabs(angle(d.scene, "Humanoid Robot 1 Right Shoulder Abduction", 1.0f)) > 40.0f);
    CHECK(std::fabs(angle(d.scene, "Humanoid Robot 1 Left Shoulder Abduction", 1.0f)) < 1e-3f);
    const auto r = findRobot(d.scene, "Humanoid Robot 1");
    REQUIRE(r.has_value());
    CHECK(withinRatedSpeed(d.scene, *r, 8.0f));
    REQUIRE(run(d, "GESTURE wave MIRROR").ok);
    CHECK(std::fabs(angle(d.scene, "Humanoid Robot 1 Left Shoulder Abduction", 1.0f)) > 40.0f);
    CHECK(std::fabs(angle(d.scene, "Humanoid Robot 1 Right Shoulder Abduction", 1.0f)) < 1e-3f);
    // SPEED 2 plays twice as fast.
    REQUIRE(run(d, "GESTURE nod SPEED 0.5").ok);
    CHECK(std::fabs(motion(d.scene, "Humanoid Robot 1 Motor Neck Pitch Drive")->keys.back().t - 3.2f) < 1e-3f);
    CHECK(!run(d, "GESTURE moonwalk").ok);
}

TEST_CASE("a looping march keeps one foot on the ground and a left hand plays right-hand shapes") {
    Document d;
    REQUIRE(run(d, "PART humanoid").ok);
    REQUIRE(run(d, "GESTURE march").ok);
    const Motion* hip = motion(d.scene, "Humanoid Robot 1 Motor Right Hip Flex Drive");
    REQUIRE(hip != nullptr);
    CHECK(hip->mode == MotionMode::Loop);
    const float cycle = hip->keys.back().t;
    const float ground = soleBottom(d.scene, 0.0f);
    for (float t = 0.0f; t < 2 * cycle; t += cycle / 16) CHECK(std::fabs(soleBottom(d.scene, t) - ground) < 1.0f);
    // The loop repeats.
    CHECK(std::fabs(angle(d.scene, "Humanoid Robot 1 Right Hip Flex", 0.3f) -
                    angle(d.scene, "Humanoid Robot 1 Right Hip Flex", 0.3f + cycle)) < 1e-3f);
    CHECK(angle(d.scene, "Humanoid Robot 1 Right Hip Flex", cycle / 4) > 20.0f ||
          angle(d.scene, "Humanoid Robot 1 Left Hip Flex", cycle / 4) > 20.0f);

    Document h;
    REQUIRE(run(h, "PART robot_hand HAND 1").ok);
    REQUIRE(run(h, "POSE hand_fist").ok); // hand shapes fit either hand
    CHECK(angle(h.scene, "Robot Hand 1 Index MCP", 10.0f) > 60.0f);
    const CommandOutput o = run(h, "GESTURE point"); // a right-hand gesture
    REQUIRE(o.ok);
    CHECK(said(o, "other side"));
}

TEST_CASE("a posture survives save and load") {
    Document d;
    REQUIRE(run(d, "PART humanoid").ok);
    REQUIRE(run(d, "POSE squat").ok);
    const auto path = std::filesystem::temp_directory_path() / "sicdm_pose_test.sicdm";
    REQUIRE(run(d, "SAVE " + path.string()).ok);
    Document e;
    REQUIRE(run(e, "OPEN " + path.string()).ok);
    std::filesystem::remove(path);
    for (float t : {0.0f, 0.4f, 0.9f, 5.0f}) {
        CHECK(std::fabs(angle(e.scene, "Humanoid Robot 1 Right Knee", t) - angle(d.scene, "Humanoid Robot 1 Right Knee", t)) < 1e-3f);
        CHECK(std::fabs(soleBottom(e.scene, t) - soleBottom(d.scene, t)) < 1e-2f);
    }
}

TEST_CASE("walk plays a real gait cycle and travels without the stance foot slipping") {
    const Gesture* walk = findGesture("walk");
    REQUIRE(walk != nullptr);
    CHECK(walk->loop && walk->smooth && walk->travel);
    Document d;
    REQUIRE(run(d, "PART humanoid RPM 300").ok); // fast motors: the natural 1.1 s stride
    const CommandOutput o = run(d, "GESTURE walk");
    REQUIRE(o.ok);
    CHECK(said(o, "travels"));
    const Motion* hip = motion(d.scene, "Humanoid Robot 1 Motor Right Hip Flex Drive");
    const Motion* travel = motion(d.scene, "Humanoid Robot 1 Walk");
    REQUIRE(hip != nullptr && travel != nullptr);
    const float T = hip->keys.back().t;
    CHECK(std::fabs(T - 1.1f) < 1e-3f);
    // Normative sagittal angles: hip 30 deg at heel strike, 10 deg extension at heel off;
    // the left leg half a cycle behind; knee about 58 deg in swing.
    CHECK(std::fabs(angle(d.scene, "Humanoid Robot 1 Right Hip Flex", 0.0f) - 30.0f) < 1e-3f);
    CHECK(std::fabs(angle(d.scene, "Humanoid Robot 1 Right Hip Flex", T * 0.5f) + 10.0f) < 1e-3f);
    CHECK(std::fabs(angle(d.scene, "Humanoid Robot 1 Left Hip Flex", 0.0f) + 10.0f) < 1e-3f);
    CHECK(std::fabs(angle(d.scene, "Humanoid Robot 1 Right Knee", T * 0.7f) - 58.0f) < 1e-3f);
    // Smooth through the loop: no stop at the wrap.
    CHECK(std::fabs(motionRate(d.scene, *hip, T - 1e-3f) - motionRate(d.scene, *hip, T + 1e-3f)) < 2.0f);
    CHECK(std::fabs(motionRate(d.scene, *hip, 0.3f * T)) > 20.0f);
    // It keeps advancing, a stride every cycle.
    const float stride = motionValue(d.scene, *travel, T) - motionValue(d.scene, *travel, 0.0f);
    CHECK(stride > 700.0f && stride < 1600.0f);
    CHECK(std::fabs(motionValue(d.scene, *travel, 2.3f * T) - motionValue(d.scene, *travel, 1.3f * T) - stride) < 1e-2f);
    // The lowest foot touches the floor, and the point it stands on does not slide
    // (allowing the instants the contact passes from one foot to the other).
    const float ground = soleBottom(d.scene, 0.0f);
    const auto movers = bodyMotions(d.scene);
    int samples = 0, slipping = 0;
    for (float t = 0.0f; t < 2.0f * T; t += 0.01f) {
        CHECK(std::fabs(soleBottom(d.scene, t) - ground) < 2.0f);
        float low = 1e30f;
        std::size_t body = 0;
        Vec3 local{};
        for (std::size_t i = 0; i < d.scene.bodies().size(); ++i) {
            const Body& b = d.scene.bodies()[i];
            if (b.name.find(" Sole") == std::string::npos) continue;
            const Mat4 tf = bodyMotionTransform(d.scene, i, t, static_cast<std::size_t>(-1), movers[i]);
            for (const Vertex& v : b.mesh.vertices)
                if (const Vec3 p = tf.transformPoint(v.position + b.offset); p.z < low) low = p.z, body = i, local = v.position + b.offset;
        }
        const Vec3 a = bodyMotionTransform(d.scene, body, t, static_cast<std::size_t>(-1), movers[body]).transformPoint(local);
        const Vec3 c = bodyMotionTransform(d.scene, body, t + 0.002f, static_cast<std::size_t>(-1), movers[body]).transformPoint(local);
        ++samples;
        if (std::hypot(c.x - a.x, c.y - a.y) / 0.002f > 60.0f) ++slipping; // walking pace is ~900 mm/s
    }
    CHECK(slipping * 20 < samples); // under 5 % of the cycle
    // Default motors are slower than a human knee in swing: the cycle stretches, shape kept.
    Document slow;
    REQUIRE(run(slow, "PART humanoid").ok);
    CHECK(said(run(slow, "GESTURE walk"), "slowed to"));
}

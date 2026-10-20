#pragma once

// Postures and gestures for the library robots (humanoid, robot arm, hand, finger,
// thumb, leg, head & neck). A posture is a set of joint angles; a gesture is a
// timed sequence of postures. Both are played by the robots' own DC gear motors:
// every target is clamped to the joint's hard stops, turned into motor angles
// (underactuated fingers move all their coupled joints together), and reached on
// minimum-jerk paths that never exceed the motor's rated speed. A humanoid keeps
// its lowest sole on the ground (its body rises and sinks instead of the feet
// leaving the floor).

#include "core/Math.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace sicdm {

class Scene;

// One joint of a robot, by its standard id: "neck yaw", "waist rotation",
// "right shoulder flex", "left elbow", "right index mcp", "left knee", ...
struct RobotJoint {
    std::string id;
    std::string motion; // the joint's motion (follows `drive`)
    std::string drive;  // its motor drive
    float ratio = 1.0f; // joint angle = drive angle x ratio
    float lo = 0.0f, hi = 0.0f;
    Vec3 axis{0, 0, 1};     // joint axis in the rest pose
    std::size_t order = 0;  // place in the motion chain (outermost first)
};

struct Robot {
    std::string component;
    std::vector<RobotJoint> joints;
};

// Components with robot joints, in scene order.
std::vector<std::string> robotNames(const Scene& scene);
// The robot a component (or one of its bodies) belongs to.
std::optional<Robot> findRobot(const Scene& scene, const std::string& componentOrBody);

// Joint id (or a group of joints: "elbow" = both elbows, "right index" = its three
// joints...) -> angle in degrees.
using Posture = std::vector<std::pair<std::string, float>>;

struct NamedPosture {
    const char* name;
    const char* summary;
    Posture joints;
};
struct Gesture {
    const char* name;
    const char* summary;
    std::vector<std::pair<float, Posture>> frames; // seconds, posture changes since the last frame
    bool loop = false;
    bool smooth = false; // a smooth spline through the frames (gait) instead of stopping on each
    bool travel = false; // the robot moves over the ground: the foot in contact does not slip
};

const std::vector<NamedPosture>& namedPostures();
const std::vector<Gesture>& namedGestures();
const NamedPosture* findPosture(const std::string& name);
const Gesture* findGesture(const std::string& name);

// Swaps right and left (and the direction of turns and side bends).
Posture mirrored(const Posture& p);

struct MoveResult {
    bool ok = true;
    std::string error;
    float duration = 0.0f;           // s until the robot is in place (one cycle for a loop)
    std::vector<std::string> notes;  // clamped or coupled joints, slowed timing
    int joints = 0;                  // joints the posture / gesture set
};

// Moves the robot from the posture it holds to `target` (joints not named hold
// still) in `seconds` (0: as fast as the motors allow, at least 0.6 s).
MoveResult poseRobot(Scene& scene, const Robot& robot, const Posture& target, float seconds = 0.0f);
// Plays a gesture from the robot's current posture. `timeScale` stretches it
// (> 1 slower); `loop` overrides the gesture's own looping; `mirror` plays it
// with the other side.
MoveResult playGesture(Scene& scene, const Robot& robot, const Gesture& gesture, float timeScale = 1.0f,
                       std::optional<bool> loop = std::nullopt, bool mirror = false);

// Joint angles the robot holds at time t (id, angle, lo, hi).
std::vector<std::pair<RobotJoint, float>> robotAngles(const Scene& scene, const Robot& robot, float t);

} // namespace sicdm

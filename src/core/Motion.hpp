#pragma once

#include "core/Math.hpp"

#include <algorithm>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace sicdm {

class Scene;

// Motion control for the Simulation window. A motion moves a set of bodies:
// circular = rotation about an axis, linear = travel along a direction.
// Motions are non-destructive: they only change where bodies are drawn while
// simulating; the modelled positions stay as they are.
enum class MotionKind { Circular, Linear };

// How a motion with a range behaves (a zero range always runs continuously).
enum class MotionMode {
    Continuous, // keeps going
    Loop,       // 0 -> range, then jumps back to 0
    Oscillate,  // 0 -> range -> 0 (back and forth)
    Once,       // 0 -> range, then stops
};

// A keyframe: value (deg / mm) at t seconds after the motion's start.
struct MotionKey {
    float t = 0.0f;
    float value = 0.0f;
};

struct Motion {
    std::string name;
    MotionKind kind = MotionKind::Circular;
    std::vector<std::string> bodies; // moved bodies by name; "*" moves every body
    Vec3 axis{0, 0, 1};              // rotation axis / travel direction
    Vec3 origin{};                   // point on the rotation axis (circular)
    bool eachCentre = false;         // circular: every body turns about its own centre (wheels)
    float speed = 90.0f;             // deg/s (circular) or mm/s (linear); negative reverses
    float range = 0.0f;              // sweep (deg) / stroke (mm); 0 = unlimited
    MotionMode mode = MotionMode::Continuous;
    float start = 0.0f;              // delay before it starts (s)
    bool enabled = true;
    // Driven by another circular motion (speed, range and mode then come from it):
    //  - linear: rolls with it on a wheel of `radius` (0 = half the first driven
    //    body's size along the direction): rack and pinion, wheels on the ground;
    //  - circular: turns `ratio` times its angle (gear trains: -z1/z2 for an
    //    external mesh). Leaders may follow others in turn (motor -> gear -> gear).
    std::string follow;
    float radius = 0.0f;
    float ratio = 1.0f;
    float step = 0.0f; // > 0: moves in steps of this size (stepper motors, deg or mm)
    // Linear only: instead of translating its bodies, the travel moves a chain or
    // belt along its own loop (see loopTravel()); usually it follows the driver
    // sprocket / pulley with radius = its pitch radius.
    bool alongLoop = false;
    // Keyframes (postures over time). When set they replace speed / range / follow:
    // the motion moves between them on minimum-jerk curves (zero velocity and
    // acceleration at every key, so it settles exactly on each value); Loop repeats
    // them, any other mode holds the last value.
    std::vector<MotionKey> keys;
    // false: minimum-jerk from key to key (stops on each: postures, gestures);
    // true: smooth spline through the keys (densely sampled tracks such as a stance).
    bool keySpline = false;
    // Joint limits (hard stops): the value is held within [lo, hi] whatever drives it.
    bool hasLimits = false;
    float lo = 0.0f, hi = 0.0f;

    bool moves(std::string_view bodyName) const;
    // Keeps the keys in time order (call after editing them).
    void sortKeys();
    float clampToLimits(float v) const noexcept { return hasLimits ? std::clamp(v, lo, hi) : v; }
    const char* unit() const noexcept { return kind == MotionKind::Circular ? "deg" : "mm"; }
    const char* rateUnit() const noexcept { return kind == MotionKind::Circular ? "deg/s" : "mm/s"; }
};

// Angle (deg) or travel (mm) at time t (s), and its rate of change.
float motionValue(const Motion& m, float t) noexcept;
float motionRate(const Motion& m, float t) noexcept;
// The same with a followed motion resolved in the scene (a rolling drive).
float motionValue(const Scene& scene, const Motion& m, float t);
float motionRate(const Scene& scene, const Motion& m, float t);
// World transform the motion applies at time t.
Mat4 motionMatrix(const Motion& m, float t) noexcept;
// The same with the rotation axis through `origin` instead of m.origin.
Mat4 motionMatrix(const Motion& m, float t, const Vec3& origin) noexcept;
// Value of a keyframed motion at time t and (optional) its rate: minimum-jerk
// interpolation 10s^3 - 15s^4 + 6s^5 between neighbouring keys, or a C1 cubic
// spline through them (keySpline).
float keyedValue(const Motion& m, float t, float* rate = nullptr) noexcept;
// Peak speed of the keyframed path (deg/s or mm/s): 1.875 x the steepest key-to-key slope.
float keyedPeakRate(const Motion& m) noexcept;
// Seconds for one full cycle (loop / oscillate / once, or the keys); 0 for unlimited motions.
float motionCycle(const Motion& m) noexcept;

// Per-body world transform at time t (identity for bodies nothing moves). A body
// moved by several motions gets them chained in list order: the first is the
// outermost, so "Drive" (all bodies) listed before "Wheel spin" carries the
// spinning wheels along.
std::vector<Mat4> motionTransforms(const Scene& scene, float t);
// The same for one body; `before` stops at that motion index (its outer motions only).
Mat4 bodyMotionTransform(const Scene& scene, std::size_t body, float t, std::size_t before = static_cast<std::size_t>(-1));
// Indices of the bodies each motion moves (by name, "*" = all), in scene order.
// Matching names once is much faster than Motion::moves() for every motion and body.
std::vector<std::vector<std::size_t>> motionBodies(const Scene& scene);
// Indices of the motions that move each body, in list order (outermost first).
std::vector<std::vector<std::size_t>> bodyMotions(const Scene& scene);
// bodyMotionTransform() given the body's motions from bodyMotions().
Mat4 bodyMotionTransform(const Scene& scene, std::size_t body, float t, std::size_t before,
                         const std::vector<std::size_t>& motionsOfBody);
// Distance (mm) a chain / belt body has travelled along its loop at time t:
// the sum of the enabled along-loop motions that move it.
float loopTravel(const Scene& scene, std::size_t body, float t);

// True when any enabled motion moves the body.
bool isMoving(const Scene& scene, std::size_t body);

// Suggested simulation length: the longest cycle (with delay), or 10 s.
float suggestedDuration(const Scene& scene) noexcept;

const char* motionKindKey(MotionKind k) noexcept;      // "circular" / "linear"
const char* motionModeKey(MotionMode m) noexcept;      // "continuous" / "loop" / "oscillate" / "once"
std::optional<MotionKind> motionKindFromKey(std::string_view key) noexcept; // also ROTARY, CIRCLE, ...
std::optional<MotionMode> motionModeFromKey(std::string_view key) noexcept; // also CONT, OSC, ...

// One line describing a motion, e.g. "Wheel spin  circular  90 deg/s about (0,1,0) at (...)  4 bodies".
std::string describeMotion(const Motion& m);

} // namespace sicdm

#pragma once

#include "core/Math.hpp"

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
    // Linear only: travel is driven by this circular motion rolling a wheel of
    // `radius` (0 = half the first driven body's size along the direction);
    // speed, range and mode then come from that motion.
    std::string follow;
    float radius = 0.0f;

    bool moves(std::string_view bodyName) const;
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
// Seconds for one full cycle (loop / oscillate / once); 0 for unlimited motions.
float motionCycle(const Motion& m) noexcept;

// Per-body world transform at time t (identity for bodies nothing moves). A body
// moved by several motions gets them chained in list order: the first is the
// outermost, so "Drive" (all bodies) listed before "Wheel spin" carries the
// spinning wheels along.
std::vector<Mat4> motionTransforms(const Scene& scene, float t);
// The same for one body; `before` stops at that motion index (its outer motions only).
Mat4 bodyMotionTransform(const Scene& scene, std::size_t body, float t, std::size_t before = static_cast<std::size_t>(-1));
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

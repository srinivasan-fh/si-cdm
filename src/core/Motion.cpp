#include "core/Motion.hpp"

#include "core/Scene.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>

namespace sicdm {

namespace {

bool iequals(std::string_view a, std::string_view b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i)
        if (std::toupper(static_cast<unsigned char>(a[i])) != std::toupper(static_cast<unsigned char>(b[i])))
            return false;
    return true;
}

bool limited(const Motion& m) noexcept { return m.range > 0.0f && m.mode != MotionMode::Continuous; }

} // namespace

bool Motion::moves(std::string_view bodyName) const {
    for (const std::string& b : bodies)
        if (b == "*" || b == bodyName) return true;
    return false;
}

float motionValue(const Motion& m, float t) noexcept {
    const float travel = std::max(t - m.start, 0.0f) * std::fabs(m.speed); // distance covered so far
    const float sign = m.speed < 0.0f ? -1.0f : 1.0f;
    if (!limited(m)) return sign * travel;
    const float r = m.range;
    switch (m.mode) {
    case MotionMode::Loop: return sign * std::fmod(travel, r);
    case MotionMode::Oscillate: {
        const float p = std::fmod(travel, 2.0f * r);
        return sign * (p <= r ? p : 2.0f * r - p);
    }
    case MotionMode::Once: return sign * std::min(travel, r);
    case MotionMode::Continuous: break;
    }
    return sign * travel;
}

float motionRate(const Motion& m, float t) noexcept {
    if (t < m.start || m.speed == 0.0f) return 0.0f;
    if (!limited(m)) return m.speed;
    const float travel = (t - m.start) * std::fabs(m.speed);
    switch (m.mode) {
    case MotionMode::Oscillate: return std::fmod(travel, 2.0f * m.range) < m.range ? m.speed : -m.speed;
    case MotionMode::Once: return travel < m.range ? m.speed : 0.0f;
    default: return m.speed;
    }
}

Mat4 motionMatrix(const Motion& m, float t) noexcept {
    if (!m.enabled || length(m.axis) < 1e-9f) return Mat4::identity();
    const float v = motionValue(m, t);
    if (m.kind == MotionKind::Linear) return translate(normalize(m.axis) * v);
    return translate(m.origin) * rotateAxis(m.axis, radians(v)) * translate(m.origin * -1.0f);
}

float motionCycle(const Motion& m) noexcept {
    if (!limited(m) || m.speed == 0.0f) return 0.0f;
    const float one = m.range / std::fabs(m.speed);
    return m.mode == MotionMode::Oscillate ? 2.0f * one : one;
}

std::vector<Mat4> motionTransforms(const Scene& scene, float t) {
    const auto& bodies = scene.bodies();
    std::vector<Mat4> out(bodies.size(), Mat4::identity());
    for (const Motion& m : scene.motions()) {
        if (!m.enabled) continue;
        const Mat4 mm = motionMatrix(m, t);
        for (std::size_t i = 0; i < bodies.size(); ++i)
            if (m.moves(bodies[i].name)) out[i] = out[i] * mm; // earlier motions stay outermost
    }
    return out;
}

Mat4 bodyMotionTransform(const Scene& scene, std::size_t body, float t, std::size_t before) {
    Mat4 out = Mat4::identity();
    if (body >= scene.bodies().size()) return out;
    const std::string& name = scene.bodies()[body].name;
    const auto& motions = scene.motions();
    for (std::size_t k = 0; k < motions.size() && k < before; ++k)
        if (motions[k].enabled && motions[k].moves(name)) out = out * motionMatrix(motions[k], t);
    return out;
}

bool isMoving(const Scene& scene, std::size_t body) {
    if (body >= scene.bodies().size()) return false;
    for (const Motion& m : scene.motions())
        if (m.enabled && m.speed != 0.0f && m.moves(scene.bodies()[body].name)) return true;
    return false;
}

float suggestedDuration(const Scene& scene) noexcept {
    float d = 0.0f;
    for (const Motion& m : scene.motions())
        if (m.enabled) d = std::max(d, m.start + motionCycle(m));
    return d > 0.05f ? d : 10.0f;
}

const char* motionKindKey(MotionKind k) noexcept { return k == MotionKind::Circular ? "circular" : "linear"; }

const char* motionModeKey(MotionMode m) noexcept {
    switch (m) {
    case MotionMode::Continuous: return "continuous";
    case MotionMode::Loop: return "loop";
    case MotionMode::Oscillate: return "oscillate";
    case MotionMode::Once: return "once";
    }
    return "continuous";
}

std::optional<MotionKind> motionKindFromKey(std::string_view k) noexcept {
    for (const char* s : {"circular", "circle", "rotary", "rotate", "rotation", "spin"})
        if (iequals(k, s)) return MotionKind::Circular;
    for (const char* s : {"linear", "line", "slide", "translate", "travel"})
        if (iequals(k, s)) return MotionKind::Linear;
    return std::nullopt;
}

std::optional<MotionMode> motionModeFromKey(std::string_view k) noexcept {
    for (const char* s : {"continuous", "cont", "continue"})
        if (iequals(k, s)) return MotionMode::Continuous;
    if (iequals(k, "loop") || iequals(k, "repeat")) return MotionMode::Loop;
    for (const char* s : {"oscillate", "osc", "pingpong", "reciprocate"})
        if (iequals(k, s)) return MotionMode::Oscillate;
    if (iequals(k, "once") || iequals(k, "single")) return MotionMode::Once;
    return std::nullopt;
}

std::string describeMotion(const Motion& m) {
    char buf[256];
    const Vec3 a = m.axis;
    if (m.kind == MotionKind::Circular)
        std::snprintf(buf, sizeof(buf), "%g deg/s about (%g, %g, %g) through (%g, %g, %g)", static_cast<double>(m.speed),
                      static_cast<double>(a.x), static_cast<double>(a.y), static_cast<double>(a.z),
                      static_cast<double>(m.origin.x), static_cast<double>(m.origin.y), static_cast<double>(m.origin.z));
    else
        std::snprintf(buf, sizeof(buf), "%g mm/s along (%g, %g, %g)", static_cast<double>(m.speed),
                      static_cast<double>(a.x), static_cast<double>(a.y), static_cast<double>(a.z));
    std::string s = m.name + "  " + motionKindKey(m.kind) + "  " + buf;
    if (m.range > 0.0f && m.mode != MotionMode::Continuous) {
        char r[64];
        std::snprintf(r, sizeof(r), "  %s %g %s", motionModeKey(m.mode), static_cast<double>(m.range), m.unit());
        s += r;
    }
    if (m.start > 0.0f) {
        char r[48];
        std::snprintf(r, sizeof(r), "  starts at %g s", static_cast<double>(m.start));
        s += r;
    }
    std::string targets;
    for (const std::string& b : m.bodies) targets += (targets.empty() ? "" : ", ") + (b == "*" ? std::string("all bodies") : b);
    s += "  [" + targets + "]";
    if (!m.enabled) s += "  (off)";
    return s;
}

} // namespace sicdm

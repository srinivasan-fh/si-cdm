#include "core/Motion.hpp"

#include "core/Scene.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <string_view>
#include <unordered_map>

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

void Motion::sortKeys() {
    std::stable_sort(keys.begin(), keys.end(), [](const MotionKey& a, const MotionKey& b) { return a.t < b.t; });
}

float keyedValue(const Motion& m, float t, float* rate) noexcept {
    if (rate) *rate = 0.0f;
    const auto& k = m.keys;
    if (k.empty()) return 0.0f;
    float u = std::max(t - m.start, 0.0f);
    const float end = k.back().t;
    // Loop repeats the keys; with keysAdvance each cycle carries on from where the last
    // one ended (a walk keeps advancing): value + cycles x (last - first).
    const bool cyclic = m.mode == MotionMode::Loop && end > 0.0f && k.size() > 1;
    const float carry = m.keysAdvance ? k.back().value - k.front().value : 0.0f;
    float offset = 0.0f;
    if (cyclic) {
        const float cycles = std::floor(u / end);
        u -= cycles * end;
        offset = cycles * carry;
    }
    if (u <= k.front().t) return k.front().value + offset;
    if (u >= end) return k.back().value + offset;
    std::size_t i = 0;
    while (i + 2 < k.size() && u >= k[i + 1].t) ++i;
    const float dt = k[i + 1].t - k[i].t;
    if (dt <= 0.0f) return k[i + 1].value;
    const float s = (u - k[i].t) / dt, d = k[i + 1].value - k[i].value;
    const float s2 = s * s, s3 = s2 * s;
    if (m.keySpline) { // cubic Hermite, tangents from the neighbours (zero at the ends unless cyclic)
        const std::size_t n = k.size();
        auto slope = [&](std::size_t j) {
            if (j == 0 || j + 1 >= n) {
                // A cyclic spline runs smoothly through the wrap: the neighbours across it.
                if (!cyclic || n < 3) return 0.0f;
                const float span = (k[1].t - k[0].t) + (end - k[n - 2].t);
                return span > 0.0f ? (k[1].value - (k[n - 2].value - carry)) / span : 0.0f;
            }
            const float span = k[j + 1].t - k[j - 1].t;
            return span > 0.0f ? (k[j + 1].value - k[j - 1].value) / span : 0.0f;
        };
        const float m0 = slope(i) * dt, m1 = slope(i + 1) * dt;
        const float h00 = 2 * s3 - 3 * s2 + 1, h10 = s3 - 2 * s2 + s, h01 = -2 * s3 + 3 * s2, h11 = s3 - s2;
        if (rate)
            *rate = ((6 * s2 - 6 * s) * k[i].value + (3 * s2 - 4 * s + 1) * m0 + (-6 * s2 + 6 * s) * k[i + 1].value +
                     (3 * s2 - 2 * s) * m1) /
                    dt;
        return h00 * k[i].value + h10 * m0 + h01 * k[i + 1].value + h11 * m1 + offset;
    }
    if (rate) *rate = d * (30.0f * s2 - 60.0f * s3 + 30.0f * s2 * s2) / dt;
    return k[i].value + d * (10.0f * s3 - 15.0f * s2 * s2 + 6.0f * s3 * s2) + offset;
}

float keyedPeakRate(const Motion& m) noexcept {
    float peak = 0.0f;
    for (std::size_t i = 0; i + 1 < m.keys.size(); ++i) {
        const float dt = m.keys[i + 1].t - m.keys[i].t, d = std::fabs(m.keys[i + 1].value - m.keys[i].value);
        if (d > 0.0f) peak = std::max(peak, dt > 0.0f ? 1.875f * d / dt : 1e30f);
    }
    return peak;
}

namespace {

float smoothValue(const Motion& m, float t) noexcept {
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

} // namespace

float motionValue(const Motion& m, float t) noexcept {
    if (!m.keys.empty()) return m.clampToLimits(keyedValue(m, t));
    const float v = smoothValue(m, t);
    return m.clampToLimits(m.step > 0.0f ? std::trunc(v / m.step + 1e-4f * (v < 0.0f ? -1.0f : 1.0f)) * m.step : v);
}

float motionRate(const Motion& m, float t) noexcept {
    if (!m.keys.empty()) {
        float rate = 0.0f;
        const float v = keyedValue(m, t, &rate);
        return m.hasLimits && (v < m.lo || v > m.hi) ? 0.0f : rate;
    }
    if (m.hasLimits) { // stopped against a limit
        const float v = smoothValue(m, t);
        if (v < m.lo || v > m.hi) return 0.0f;
    }
    if (t < m.start || m.speed == 0.0f) return 0.0f;
    if (!limited(m)) return m.speed;
    const float travel = (t - m.start) * std::fabs(m.speed);
    switch (m.mode) {
    case MotionMode::Oscillate: return std::fmod(travel, 2.0f * m.range) < m.range ? m.speed : -m.speed;
    case MotionMode::Once: return travel < m.range ? m.speed : 0.0f;
    default: return m.speed;
    }
}

Mat4 motionMatrix(const Motion& m, float t) noexcept { return motionMatrix(m, t, m.origin); }

namespace {

Mat4 matrixForValue(const Motion& m, float v, const Vec3& origin) noexcept {
    // Travel along a loop changes the chain / belt shape, not its placement.
    if (!m.enabled || m.alongLoop || length(m.axis) < 1e-9f) return Mat4::identity();
    if (m.kind == MotionKind::Linear) return translate(normalize(m.axis) * v);
    return translate(origin) * rotateAxis(m.axis, radians(v)) * translate(origin * -1.0f);
}

} // namespace

Mat4 motionMatrix(const Motion& m, float t, const Vec3& origin) noexcept {
    return matrixForValue(m, motionValue(m, t), origin);
}

namespace {

const Motion* driverOf(const Scene& scene, const Motion& m) {
    if (m.follow.empty()) return nullptr;
    for (const Motion& d : scene.motions())
        if (d.kind == MotionKind::Circular && d.enabled && &d != &m && iequals(d.name, m.follow)) return &d;
    return nullptr;
}

// Value / rate of m, following leaders up to `depth` levels (stops loops).
float valueAt(const Scene& scene, const Motion& m, float t, int depth);
float rateAt(const Scene& scene, const Motion& m, float t, int depth);

// mm of travel per radian of the driving wheel.
float rollRadius(const Scene& scene, const Motion& m, const Motion& driver) {
    if (m.radius > 0.0f) return m.radius;
    const Vec3 dir = normalize(m.axis);
    for (const Body& b : scene.bodies())
        if (driver.moves(b.name)) {
            const Vec3 size = b.bounds().max - b.bounds().min;
            return 0.5f * (std::fabs(dir.x) * size.x + std::fabs(dir.y) * size.y + std::fabs(dir.z) * size.z);
        }
    return 0.0f;
}

// Unclamped value of a follower (its leader's value through the ratio / roll radius).
float followedValue(const Scene& scene, const Motion& m, const Motion& d, float t, int depth) {
    const float lead = valueAt(scene, d, t, depth - 1);
    return m.kind == MotionKind::Linear ? radians(lead) * rollRadius(scene, m, d) : lead * m.ratio;
}

float valueAt(const Scene& scene, const Motion& m, float t, int depth) {
    if (m.follow.empty() || !m.keys.empty()) return motionValue(m, t);
    const Motion* d = depth > 0 ? driverOf(scene, m) : nullptr;
    if (!d) return m.clampToLimits(0.0f);
    return m.clampToLimits(followedValue(scene, m, *d, t, depth)); // a joint stops at its hard stops
}

float rateAt(const Scene& scene, const Motion& m, float t, int depth) {
    if (m.follow.empty() || !m.keys.empty()) return motionRate(m, t);
    const Motion* d = depth > 0 ? driverOf(scene, m) : nullptr;
    if (!d) return 0.0f;
    if (m.hasLimits) {
        const float v = followedValue(scene, m, *d, t, depth);
        if (v < m.lo || v > m.hi) return 0.0f;
    }
    const float lead = rateAt(scene, *d, t, depth - 1);
    return m.kind == MotionKind::Linear ? radians(lead) * rollRadius(scene, m, *d) : lead * m.ratio;
}

constexpr int kMaxFollowDepth = 16;

Mat4 sceneMatrix(const Scene& scene, const Motion& m, float t) {
    return matrixForValue(m, valueAt(scene, m, t, kMaxFollowDepth), m.origin);
}

// Matrix of motion m for one body (its own centre when the motion asks for it).
Mat4 matrixFor(const Scene& scene, const Motion& m, const Body& b, float t) {
    return m.kind == MotionKind::Circular && m.eachCentre
               ? matrixForValue(m, valueAt(scene, m, t, kMaxFollowDepth), b.bounds().center())
               : sceneMatrix(scene, m, t);
}

} // namespace

float motionValue(const Scene& scene, const Motion& m, float t) { return valueAt(scene, m, t, kMaxFollowDepth); }

float motionRate(const Scene& scene, const Motion& m, float t) { return rateAt(scene, m, t, kMaxFollowDepth); }

float motionCycle(const Motion& m) noexcept {
    if (!m.keys.empty()) return m.keys.back().t;
    if (!limited(m) || m.speed == 0.0f) return 0.0f;
    const float one = m.range / std::fabs(m.speed);
    return m.mode == MotionMode::Oscillate ? 2.0f * one : one;
}

std::vector<std::vector<std::size_t>> motionBodies(const Scene& scene) {
    const auto& bodies = scene.bodies();
    std::unordered_map<std::string_view, std::vector<std::size_t>> byName;
    for (std::size_t i = 0; i < bodies.size(); ++i) byName[bodies[i].name].push_back(i);
    std::vector<std::vector<std::size_t>> out(scene.motions().size());
    for (std::size_t k = 0; k < out.size(); ++k) {
        const Motion& m = scene.motions()[k];
        std::vector<std::size_t>& list = out[k];
        bool all = false;
        for (const std::string& n : m.bodies) all = all || n == "*";
        if (all) {
            list.resize(bodies.size());
            for (std::size_t i = 0; i < bodies.size(); ++i) list[i] = i;
            continue;
        }
        for (const std::string& n : m.bodies)
            if (const auto it = byName.find(n); it != byName.end()) list.insert(list.end(), it->second.begin(), it->second.end());
        std::sort(list.begin(), list.end());
        list.erase(std::unique(list.begin(), list.end()), list.end()); // a name listed twice moves its body once
    }
    return out;
}

std::vector<std::vector<std::size_t>> bodyMotions(const Scene& scene) {
    const auto perMotion = motionBodies(scene);
    std::vector<std::vector<std::size_t>> out(scene.bodies().size());
    for (std::size_t k = 0; k < perMotion.size(); ++k)
        for (std::size_t i : perMotion[k]) out[i].push_back(k);
    return out;
}

std::vector<Mat4> motionTransforms(const Scene& scene, float t) {
    const auto& bodies = scene.bodies();
    std::vector<Mat4> out(bodies.size(), Mat4::identity());
    const auto moved = motionBodies(scene);
    for (std::size_t k = 0; k < moved.size(); ++k) {
        const Motion& m = scene.motions()[k];
        if (!m.enabled || moved[k].empty()) continue;
        const Mat4 mm = sceneMatrix(scene, m, t);
        for (std::size_t i : moved[k])
            out[i] = out[i] * (m.eachCentre ? matrixFor(scene, m, bodies[i], t) : mm); // earlier motions stay outermost
    }
    return out;
}

Mat4 bodyMotionTransform(const Scene& scene, std::size_t body, float t, std::size_t before,
                         const std::vector<std::size_t>& motionsOfBody) {
    Mat4 out = Mat4::identity();
    if (body >= scene.bodies().size()) return out;
    for (std::size_t k : motionsOfBody) {
        if (k >= before) break;
        const Motion& m = scene.motions()[k];
        if (m.enabled) out = out * matrixFor(scene, m, scene.bodies()[body], t);
    }
    return out;
}

Mat4 bodyMotionTransform(const Scene& scene, std::size_t body, float t, std::size_t before) {
    Mat4 out = Mat4::identity();
    if (body >= scene.bodies().size()) return out;
    const std::string& name = scene.bodies()[body].name;
    const auto& motions = scene.motions();
    for (std::size_t k = 0; k < motions.size() && k < before; ++k)
        if (motions[k].enabled && motions[k].moves(name)) out = out * matrixFor(scene, motions[k], scene.bodies()[body], t);
    return out;
}

float loopTravel(const Scene& scene, std::size_t body, float t) {
    if (body >= scene.bodies().size()) return 0.0f;
    float s = 0.0f;
    for (const Motion& m : scene.motions())
        if (m.enabled && m.alongLoop && m.moves(scene.bodies()[body].name)) s += motionValue(scene, m, t);
    return s;
}

bool isMoving(const Scene& scene, std::size_t body) {
    if (body >= scene.bodies().size()) return false;
    for (const Motion& m : scene.motions())
        if (m.enabled && (m.speed != 0.0f || !m.follow.empty()) && m.moves(scene.bodies()[body].name)) return true;
    return false;
}

float suggestedDuration(const Scene& scene) noexcept {
    // Motions that end (Once) or turn back (Oscillate) set the length; looping
    // and continuous ones run on by themselves.
    float d = 0.0f;
    for (const Motion& m : scene.motions())
        if (m.enabled && m.follow.empty() && (m.mode == MotionMode::Once || m.mode == MotionMode::Oscillate))
            d = std::max(d, m.start + motionCycle(m));
    if (scene.physics().enabled) // a physics run sets the length too
        for (const Body& b : scene.bodies())
            if (b.physics.enabled) {
                d = std::max(d, scene.physics().duration);
                break;
            }
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
    if (m.kind == MotionKind::Circular && !m.follow.empty())
        std::snprintf(buf, sizeof(buf), "turns %g x %s about (%g, %g, %g)%s", static_cast<double>(m.ratio),
                      m.follow.c_str(), static_cast<double>(a.x), static_cast<double>(a.y), static_cast<double>(a.z),
                      m.eachCentre ? " through each body's centre" : "");
    else if (m.kind == MotionKind::Circular && m.eachCentre)
        std::snprintf(buf, sizeof(buf), "%g deg/s about (%g, %g, %g) through each body's centre",
                      static_cast<double>(m.speed), static_cast<double>(a.x), static_cast<double>(a.y),
                      static_cast<double>(a.z));
    else if (m.kind == MotionKind::Circular)
        std::snprintf(buf, sizeof(buf), "%g deg/s about (%g, %g, %g) through (%g, %g, %g)", static_cast<double>(m.speed),
                      static_cast<double>(a.x), static_cast<double>(a.y), static_cast<double>(a.z),
                      static_cast<double>(m.origin.x), static_cast<double>(m.origin.y), static_cast<double>(m.origin.z));
    else if (m.alongLoop && !m.follow.empty())
        std::snprintf(buf, sizeof(buf), "travels along its loop with %s (radius %g mm)", m.follow.c_str(),
                      static_cast<double>(m.radius));
    else if (m.alongLoop)
        std::snprintf(buf, sizeof(buf), "travels along its loop at %g mm/s", static_cast<double>(m.speed));
    else if (!m.follow.empty() && m.radius > 0.0f)
        std::snprintf(buf, sizeof(buf), "rolls with %s (radius %g mm) along (%g, %g, %g)", m.follow.c_str(),
                      static_cast<double>(m.radius), static_cast<double>(a.x), static_cast<double>(a.y),
                      static_cast<double>(a.z));
    else if (!m.follow.empty())
        std::snprintf(buf, sizeof(buf), "rolls with %s (wheel radius) along (%g, %g, %g)", m.follow.c_str(),
                      static_cast<double>(a.x), static_cast<double>(a.y), static_cast<double>(a.z));
    else
        std::snprintf(buf, sizeof(buf), "%g mm/s along (%g, %g, %g)", static_cast<double>(m.speed),
                      static_cast<double>(a.x), static_cast<double>(a.y), static_cast<double>(a.z));
    std::string s = m.name + "  " + motionKindKey(m.kind) + "  " + buf;
    if (m.follow.empty() && m.range > 0.0f && m.mode != MotionMode::Continuous) {
        char r[64];
        std::snprintf(r, sizeof(r), "  %s %g %s", motionModeKey(m.mode), static_cast<double>(m.range), m.unit());
        s += r;
    }
    if (m.start > 0.0f) {
        char r[48];
        std::snprintf(r, sizeof(r), "  starts at %g s", static_cast<double>(m.start));
        s += r;
    }
    if (m.step > 0.0f) {
        char r[48];
        std::snprintf(r, sizeof(r), "  steps of %g %s", static_cast<double>(m.step), m.unit());
        s += r;
    }
    if (!m.keys.empty()) {
        char r[96];
        std::snprintf(r, sizeof(r), "  keyframed: %zu keys over %g s%s", m.keys.size(), static_cast<double>(m.keys.back().t),
                      m.mode == MotionMode::Loop ? ", looping" : "");
        s += r;
    }
    if (m.hasLimits) {
        char r[64];
        std::snprintf(r, sizeof(r), "  limits %g..%g %s", static_cast<double>(m.lo), static_cast<double>(m.hi), m.unit());
        s += r;
    }
    std::string targets;
    for (const std::string& b : m.bodies) targets += (targets.empty() ? "" : ", ") + (b == "*" ? std::string("all bodies") : b);
    s += "  [" + targets + "]";
    if (!m.enabled) s += "  (off)";
    return s;
}

} // namespace sicdm

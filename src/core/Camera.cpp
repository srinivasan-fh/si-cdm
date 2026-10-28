#include "core/Camera.hpp"

namespace sicdm {

namespace {

constexpr float kMinDistance = 1e-3f;
constexpr float kMaxDistance = 1e6f;
constexpr float kPitchLimit = radians(89.9f);
constexpr float kOrbitSpeed = radians(0.4f); // radians per pixel
constexpr float kZoomFactor = 0.88f;          // per wheel step

float wrapAngle(float a) noexcept {
    while (a > kPi) a -= 2.0f * kPi;
    while (a < -kPi) a += 2.0f * kPi;
    return a;
}

float smoothstep(float t) noexcept { return t * t * (3.0f - 2.0f * t); }

} // namespace

OrbitCamera::OrbitCamera() {
    setStandardView(StandardView::Isometric, false);
}

Vec3 OrbitCamera::forward() const noexcept {
    // Direction from the eye towards the target.
    const float cp = std::cos(m_pitch);
    return -Vec3{cp * std::cos(m_yaw), cp * std::sin(m_yaw), std::sin(m_pitch)};
}

// Level (unrolled) camera axes; roll then turns both around forward().
Vec3 OrbitCamera::right() const noexcept {
    const Vec3 r0{-std::sin(m_yaw), std::cos(m_yaw), 0.0f};
    if (m_roll == 0.0f) return r0;
    const Vec3 u0 = cross(r0, forward());
    return r0 * std::cos(m_roll) - u0 * std::sin(m_roll);
}

Vec3 OrbitCamera::up() const noexcept {
    return cross(right(), forward());
}

Vec3 OrbitCamera::position() const noexcept {
    return m_target - forward() * m_distance;
}

Mat4 OrbitCamera::view() const noexcept {
    // Use the derived up vector so straight-down/up views stay well defined.
    return lookAt(position(), m_target, up());
}

float OrbitCamera::nearPlane() const noexcept {
    return std::max(m_distance * 0.001f, 1e-4f);
}

float OrbitCamera::farPlane() const noexcept {
    return m_distance * 1000.0f;
}

Mat4 OrbitCamera::projectionMatrix() const noexcept {
    if (m_projection == Projection::Perspective)
        return perspective(m_fovY, m_aspect, nearPlane(), farPlane());

    // Orthographic: match the perspective frustum height at the target.
    const float halfH = m_distance * std::tan(m_fovY * 0.5f);
    const float halfW = halfH * m_aspect;
    return orthographic(-halfW, halfW, -halfH, halfH, -farPlane(), farPlane());
}

Ray OrbitCamera::screenRay(float x, float y, float width, float height) const noexcept {
    const float nx = width > 0.0f ? 2.0f * x / width - 1.0f : 0.0f;
    const float ny = height > 0.0f ? 1.0f - 2.0f * y / height : 0.0f;
    const float halfH = std::tan(m_fovY * 0.5f);
    const float halfW = halfH * m_aspect;
    if (m_projection == Projection::Perspective)
        return {position(), normalize(forward() + right() * (nx * halfW) + up() * (ny * halfH))};

    // Orthographic: parallel rays, started well behind the eye.
    const Vec3 offset = right() * (nx * halfW * m_distance) + up() * (ny * halfH * m_distance);
    return {position() + offset - forward() * (farPlane() * 0.5f), forward()};
}

void OrbitCamera::orbit(float dx, float dy) noexcept {
    m_animT = 1.0f;
    // Express the screen-space drag in the level frame so a rolled view still
    // orbits the way the cursor moves.
    if (m_roll != 0.0f) {
        const float c = std::cos(m_roll), s = std::sin(m_roll);
        const float lx = dx * c - dy * s, ly = dy * c + dx * s;
        dx = lx;
        dy = ly;
    }
    m_yaw = wrapAngle(m_yaw - dx * kOrbitSpeed);
    m_pitch = std::clamp(m_pitch + dy * kOrbitSpeed, -kPitchLimit, kPitchLimit);
}

void OrbitCamera::pan(float dx, float dy, float viewportHeight) noexcept {
    m_animT = 1.0f;
    if (viewportHeight <= 0.0f) return;
    m_target += (right() * -dx + up() * dy) * worldPerPixel(viewportHeight);
}

void OrbitCamera::zoom(float steps) noexcept {
    if (m_animT < 1.0f) applyPose(m_to); // a view change in flight (FIT, a standard view) ends where it was going
    m_animT = 1.0f;
    m_distance = std::clamp(m_distance * std::pow(kZoomFactor, steps), kMinDistance, kMaxDistance);
}

void OrbitCamera::zoomAt(float steps, float x, float y, float w, float h) noexcept {
    // Point under the cursor on the plane through the target facing the camera.
    auto underCursor = [&](Vec3& out) {
        const Ray r = screenRay(x, y, w, h);
        float t = 0.0f;
        if (!intersectPlane(r, m_target, forward(), t)) return false;
        out = r.at(t);
        return true;
    };
    Vec3 before, after;
    const bool ok = underCursor(before);
    zoom(steps);
    if (ok && underCursor(after)) m_target += before - after;
}

float OrbitCamera::worldPerPixel(float viewportHeight) const noexcept {
    return viewportHeight > 0.0f ? 2.0f * m_distance * std::tan(m_fovY * 0.5f) / viewportHeight : 0.0f;
}

void OrbitCamera::setStandardView(StandardView v, bool animate) noexcept {
    Pose to{m_target, m_yaw, m_pitch, m_distance};
    switch (v) {
    case StandardView::Front:  to.yaw = -kPi * 0.5f; to.pitch = 0.0f; break;  // looking along +Y
    case StandardView::Back:   to.yaw = kPi * 0.5f;  to.pitch = 0.0f; break;
    case StandardView::Right:  to.yaw = 0.0f;        to.pitch = 0.0f; break;  // looking along -X
    case StandardView::Left:   to.yaw = kPi;         to.pitch = 0.0f; break;
    // Exactly plan-on: right() comes from yaw alone, so the view stays well defined.
    case StandardView::Top:    to.yaw = -kPi * 0.5f; to.pitch = kPi * 0.5f; break;
    case StandardView::Bottom: to.yaw = -kPi * 0.5f; to.pitch = -kPi * 0.5f; break;
    case StandardView::Isometric:
        to.yaw = -kPi * 0.25f;                       // from +X/-Y quadrant
        to.pitch = std::atan(1.0f / std::sqrt(2.0f)); // 35.264 deg
        break;
    }
    startTransition(to, animate);
}

void OrbitCamera::setViewDirection(const Vec3& d, bool animate) noexcept {
    const Vec3 n = normalize(d);
    if (length(n) == 0.0f) return;
    Pose to{m_target, m_yaw, m_pitch, m_distance};
    const float horizontal = std::sqrt(n.x * n.x + n.y * n.y);
    to.pitch = std::atan2(n.z, horizontal);
    // Straight up/down keeps the current heading (Top shows X right, Y up from Front).
    to.yaw = horizontal > 1e-5f ? std::atan2(n.y, n.x) : (m_pitch > kPitchLimit || m_pitch < -kPitchLimit ? m_yaw : -kPi * 0.5f);
    startTransition(to, animate);
}

void OrbitCamera::setViewFrame(const Vec3& d, const Vec3& rightDir, bool animate) noexcept {
    const Vec3 n = normalize(d);
    if (length(n) == 0.0f) return;
    Pose to{m_target, m_yaw, m_pitch, m_distance};
    const float horizontal = std::sqrt(n.x * n.x + n.y * n.y);
    to.pitch = std::atan2(n.z, horizontal);
    to.yaw = horizontal > 1e-5f ? std::atan2(n.y, n.x) : -kPi * 0.5f;
    // The level right axis and up of that pose, then the roll that turns right onto rightDir.
    const float cp = std::cos(to.pitch);
    const Vec3 fwd = -Vec3{cp * std::cos(to.yaw), cp * std::sin(to.yaw), std::sin(to.pitch)};
    const Vec3 r0{-std::sin(to.yaw), std::cos(to.yaw), 0.0f}, u0 = cross(r0, fwd);
    const Vec3 r = rightDir - n * dot(rightDir, n);
    if (length(r) > 1e-6f) to.roll = std::atan2(-dot(r, u0), dot(r, r0));
    startTransition(to, animate);
}

void OrbitCamera::rotateAroundZ(float rad, bool animate) noexcept {
    Pose to{m_target, m_yaw + rad, m_pitch, m_distance, m_roll};
    startTransition(to, animate);
}

void OrbitCamera::rollView(float rad, bool animate) noexcept {
    // Chain presses made while a roll is still animating.
    const bool animating = m_animT < 1.0f;
    const float base = animating ? m_to.roll : m_roll;
    Pose to{animating ? m_to.target : m_target, animating ? m_to.yaw : m_yaw, animating ? m_to.pitch : m_pitch,
            animating ? m_to.distance : m_distance, base + rad};
    startTransition(to, animate);
}

void OrbitCamera::resetRoll(bool animate) noexcept {
    Pose to{m_target, m_yaw, m_pitch, m_distance, 0.0f};
    startTransition(to, animate);
}

void OrbitCamera::fitToBounds(const AABB& bounds, bool animate) noexcept {
    if (!bounds.valid()) return;
    const float r = std::max(bounds.radius(), 1e-3f);
    const float halfFov = 0.5f * std::min(m_fovY, 2.0f * std::atan(std::tan(m_fovY * 0.5f) * m_aspect));
    // Keep the orientation of a view change still in flight (e.g. Front, then Fit).
    const bool animating = m_animT < 1.0f;
    Pose to{bounds.center(), animating ? m_to.yaw : m_yaw, animating ? m_to.pitch : m_pitch,
            (r / std::sin(halfFov)) * 1.05f, animating ? m_to.roll : m_roll};
    startTransition(to, animate);
}

void OrbitCamera::startTransition(const Pose& to, bool animate) noexcept {
    m_from = {m_target, m_yaw, m_pitch, m_distance, m_roll};
    m_to = to;
    // Take the short way round in yaw and roll.
    m_to.yaw = m_from.yaw + wrapAngle(to.yaw - m_from.yaw);
    m_to.roll = m_from.roll + wrapAngle(to.roll - m_from.roll);
    if (animate) {
        m_animT = 0.0f;
    } else {
        m_animT = 1.0f;
        applyPose(m_to);
    }
}

void OrbitCamera::applyPose(const Pose& p) noexcept {
    m_target = p.target;
    m_yaw = wrapAngle(p.yaw);
    m_pitch = p.pitch;
    m_distance = p.distance;
    m_roll = wrapAngle(p.roll);
    if (std::fabs(m_roll) < 1e-6f) m_roll = 0.0f;
}

bool OrbitCamera::update(float dt) noexcept {
    if (m_animT >= 1.0f) return false;
    m_animT = std::min(1.0f, m_animT + dt / m_animDuration);
    const float t = smoothstep(m_animT);
    Pose p;
    p.target = lerp(m_from.target, m_to.target, t);
    p.yaw = lerp(m_from.yaw, m_to.yaw, t);
    p.pitch = lerp(m_from.pitch, m_to.pitch, t);
    p.roll = lerp(m_from.roll, m_to.roll, t);
    // Interpolate distance in log space so zoom feels uniform.
    p.distance = std::exp(lerp(std::log(m_from.distance), std::log(m_to.distance), t));
    applyPose(p);
    return m_animT < 1.0f;
}

} // namespace sicdm

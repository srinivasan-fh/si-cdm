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

Vec3 OrbitCamera::right() const noexcept {
    return {-std::sin(m_yaw), std::cos(m_yaw), 0.0f};
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
    m_yaw = wrapAngle(m_yaw - dx * kOrbitSpeed);
    m_pitch = std::clamp(m_pitch + dy * kOrbitSpeed, -kPitchLimit, kPitchLimit);
}

void OrbitCamera::pan(float dx, float dy, float viewportHeight) noexcept {
    m_animT = 1.0f;
    if (viewportHeight <= 0.0f) return;
    // World units per pixel at the target depth.
    const float worldPerPixel = 2.0f * m_distance * std::tan(m_fovY * 0.5f) / viewportHeight;
    m_target += (right() * -dx + up() * dy) * worldPerPixel;
}

void OrbitCamera::zoom(float steps) noexcept {
    m_animT = 1.0f;
    m_distance = std::clamp(m_distance * std::pow(kZoomFactor, steps), kMinDistance, kMaxDistance);
}

void OrbitCamera::setStandardView(StandardView v, bool animate) noexcept {
    Pose to{m_target, m_yaw, m_pitch, m_distance};
    switch (v) {
    case StandardView::Front:  to.yaw = -kPi * 0.5f; to.pitch = 0.0f; break;  // looking along +Y
    case StandardView::Back:   to.yaw = kPi * 0.5f;  to.pitch = 0.0f; break;
    case StandardView::Right:  to.yaw = 0.0f;        to.pitch = 0.0f; break;  // looking along -X
    case StandardView::Left:   to.yaw = kPi;         to.pitch = 0.0f; break;
    case StandardView::Top:    to.yaw = -kPi * 0.5f; to.pitch = kPitchLimit; break;
    case StandardView::Bottom: to.yaw = -kPi * 0.5f; to.pitch = -kPitchLimit; break;
    case StandardView::Isometric:
        to.yaw = -kPi * 0.25f;                       // from +X/-Y quadrant
        to.pitch = std::atan(1.0f / std::sqrt(2.0f)); // 35.264 deg
        break;
    }
    startTransition(to, animate);
}

void OrbitCamera::fitToBounds(const AABB& bounds, bool animate) noexcept {
    if (!bounds.valid()) return;
    const float r = std::max(bounds.radius(), 1e-3f);
    const float halfFov = 0.5f * std::min(m_fovY, 2.0f * std::atan(std::tan(m_fovY * 0.5f) * m_aspect));
    Pose to{bounds.center(), m_yaw, m_pitch, (r / std::sin(halfFov)) * 1.05f};
    startTransition(to, animate);
}

void OrbitCamera::startTransition(const Pose& to, bool animate) noexcept {
    m_from = {m_target, m_yaw, m_pitch, m_distance};
    m_to = to;
    // Take the short way round in yaw.
    m_to.yaw = m_from.yaw + wrapAngle(to.yaw - m_from.yaw);
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
}

bool OrbitCamera::update(float dt) noexcept {
    if (m_animT >= 1.0f) return false;
    m_animT = std::min(1.0f, m_animT + dt / m_animDuration);
    const float t = smoothstep(m_animT);
    Pose p;
    p.target = lerp(m_from.target, m_to.target, t);
    p.yaw = lerp(m_from.yaw, m_to.yaw, t);
    p.pitch = lerp(m_from.pitch, m_to.pitch, t);
    // Interpolate distance in log space so zoom feels uniform.
    p.distance = std::exp(lerp(std::log(m_from.distance), std::log(m_to.distance), t));
    applyPose(p);
    return m_animT < 1.0f;
}

} // namespace sicdm

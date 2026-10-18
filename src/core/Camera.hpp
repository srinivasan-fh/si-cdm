#pragma once

#include "core/Math.hpp"

namespace sicdm {

enum class Projection { Perspective, Orthographic };

enum class StandardView { Front, Back, Left, Right, Top, Bottom, Isometric };

// CAD-style orbit camera: orbits around a target point, Z is world-up.
// Yaw is measured around +Z from +X, pitch is elevation above the XY plane.
class OrbitCamera {
public:
    OrbitCamera();

    // --- interaction (deltas in pixels, viewport size in pixels) ---
    void orbit(float dxPixels, float dyPixels) noexcept;
    void pan(float dxPixels, float dyPixels, float viewportHeight) noexcept;
    void zoom(float wheelSteps) noexcept;

    // --- framing ---
    void setStandardView(StandardView view, bool animate = true) noexcept;
    void fitToBounds(const AABB& bounds, bool animate = true) noexcept;

    // Advances any running view transition; returns true while animating.
    bool update(float dtSeconds) noexcept;

    // --- state ---
    void setAspect(float aspect) noexcept { m_aspect = aspect > 0.0f ? aspect : 1.0f; }
    void setProjection(Projection p) noexcept { m_projection = p; }
    Projection projection() const noexcept { return m_projection; }

    Vec3 target() const noexcept { return m_target; }
    float distance() const noexcept { return m_distance; }
    float yaw() const noexcept { return m_yaw; }
    float pitch() const noexcept { return m_pitch; }
    float fovY() const noexcept { return m_fovY; }

    Vec3 position() const noexcept;
    Vec3 forward() const noexcept;
    Vec3 right() const noexcept;
    Vec3 up() const noexcept;

    Mat4 view() const noexcept;
    Mat4 projectionMatrix() const noexcept;
    Mat4 viewProjection() const noexcept { return projectionMatrix() * view(); }

    float nearPlane() const noexcept;
    float farPlane() const noexcept;

private:
    struct Pose {
        Vec3 target;
        float yaw = 0, pitch = 0, distance = 1;
    };

    void startTransition(const Pose& to, bool animate) noexcept;
    void applyPose(const Pose& p) noexcept;

    Vec3 m_target{0, 0, 0};
    float m_yaw = 0;
    float m_pitch = 0;
    float m_distance = 10.0f;
    float m_fovY = radians(40.0f);
    float m_aspect = 16.0f / 9.0f;
    Projection m_projection = Projection::Perspective;

    // Animated transition between poses.
    Pose m_from{}, m_to{};
    float m_animT = 1.0f;
    float m_animDuration = 0.35f;
};

} // namespace sicdm

#include "core/SpaceMouse.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace sicdm {

std::optional<SpaceMouseEvent> decodeSpnav(std::span<const std::int32_t, 8> w) {
    SpaceMouseEvent e;
    switch (w[0]) {
    case 0:
        e.kind = SpaceMouseEvent::Motion;
        e.motion = {static_cast<float>(w[1]), static_cast<float>(w[2]), static_cast<float>(w[3]),
                    static_cast<float>(w[4]), static_cast<float>(w[5]), static_cast<float>(w[6])};
        return e;
    case 1:
    case 2:
        e.kind = w[0] == 1 ? SpaceMouseEvent::Press : SpaceMouseEvent::Release;
        e.button = w[1];
        if (e.button < 0 || e.button > 63) return std::nullopt;
        return e;
    default: return std::nullopt;
    }
}

namespace {

float i16(const std::uint8_t* p) {
    std::int16_t v = 0;
    std::memcpy(&v, p, 2); // little-endian on every platform built for
    return static_cast<float>(v);
}

} // namespace

bool decodeHidReport(std::span<const std::uint8_t> r, HidState& s) {
    if (r.empty()) return false;
    // HID: X right, Y toward the user, Z down. spacenavd: x right, y up, z toward the user.
    auto translation = [&](const std::uint8_t* p) {
        s.motion.tx = i16(p);
        s.motion.tz = i16(p + 2);
        s.motion.ty = -i16(p + 4);
    };
    auto rotation = [&](const std::uint8_t* p) {
        s.motion.rx = i16(p);
        s.motion.rz = i16(p + 2);
        s.motion.ry = -i16(p + 4);
    };
    switch (r[0]) {
    case 1:
        if (r.size() >= 13) {
            translation(&r[1]);
            rotation(&r[7]);
            return true;
        }
        if (r.size() >= 7) {
            translation(&r[1]);
            return true;
        }
        return false;
    case 2:
        if (r.size() < 7) return false;
        rotation(&r[1]);
        return true;
    case 3: {
        std::uint32_t b = 0;
        for (std::size_t k = 1; k < r.size() && k <= 4; ++k) b |= static_cast<std::uint32_t>(r[k]) << (8 * (k - 1));
        s.buttons = b;
        return true;
    }
    default: return false;
    }
}

void navigateSixDof(OrbitCamera& cam, const SixDof& in, float dt, float sensitivity, float viewportHeight, bool flat, bool cameraMode) {
    if (dt <= 0 || viewportHeight <= 0) return;
    dt = std::min(dt, 0.1f); // a stall doesn't fling the view
    const float k = std::clamp(sensitivity, 0.25f, 4.0f) * (cameraMode ? -1.0f : 1.0f);
    // Dead zone and a gentle curve: fine control near rest, speed at full push.
    auto shape = [&](float v) {
        const float a = std::fabs(v) / 350.0f;
        if (a < 0.04f) return 0.0f;
        const float t = std::min((a - 0.04f) / 0.96f, 1.5f);
        return std::copysign(t * t * 0.6f + t * 0.4f, v);
    };
    const float tx = shape(in.tx), ty = shape(in.ty), tz = shape(in.tz);
    const float rx = shape(in.rx), ry = shape(in.ry);
    // Pans as screen pixels a second; a mouse drag of the model the same way.
    const float panPx = 900.0f * k * dt;
    if (tx != 0 || ty != 0) cam.pan(tx * panPx, -ty * panPx, viewportHeight);
    // Toward you: the model comes closer (zoom steps a second).
    if (tz != 0) cam.zoom(tz * 6.0f * k * dt);
    if (!flat && (rx != 0 || ry != 0)) {
        // Twist about the vertical, tilt about the horizontal, as an orbit drag would.
        const float orbitPx = 700.0f * k * dt;
        cam.orbit(-ry * orbitPx, rx * orbitPx);
    }
}

} // namespace sicdm

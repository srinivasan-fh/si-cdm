#pragma once

// 3D mice (3Dconnexion SpaceMouse and its kind): six axes, pushed and twisted at once.
// The platform reads the device (app/SpaceMouse.cpp: spacenavd on Linux, Raw Input on
// Windows, IOKit on macOS); here the reports are decoded and turned into view moves, so
// they can be tested without a device.
//
// Axes as the cap moves, in the spacenavd convention: tx right, ty up, tz toward the
// user; rx tilts the cap toward the user (nose up), ry twists it clockwise seen from
// above, rz rolls it right. Full deflection is about 350.

#include "core/Camera.hpp"

#include <cstdint>
#include <optional>
#include <span>

namespace sicdm {

struct SixDof {
    float tx = 0, ty = 0, tz = 0, rx = 0, ry = 0, rz = 0;
    bool any() const noexcept { return tx != 0 || ty != 0 || tz != 0 || rx != 0 || ry != 0 || rz != 0; }
};

struct SpaceMouseEvent {
    enum Kind { Motion, Press, Release } kind = Motion;
    SixDof motion;
    int button = 0;
};

// spacenavd's socket protocol: eight 32-bit ints an event ({0, x, y, z, rx, ry, rz, period}
// for motion; {1 or 2, button, ...} for a press or release).
std::optional<SpaceMouseEvent> decodeSpnav(std::span<const std::int32_t, 8> words);

// A 3Dconnexion HID input report (Windows Raw Input): report 1 holds the translation (and
// the rotation too on newer devices: 12 bytes), report 2 the rotation, report 3 the buttons
// (a bit each). Axes become the spacenavd convention. `motion` keeps the last of each, as
// the two halves arrive in separate reports on older devices.
struct HidState {
    SixDof motion;
    std::uint32_t buttons = 0;
};
bool decodeHidReport(std::span<const std::uint8_t> report, HidState& state);

// One frame of navigation: the cap's deflection moves the view at a speed, `sensitivity`
// 0.25..4 scaling it, a small dead zone ignoring a resting hand. In object mode the model
// follows the cap (push it right: the model goes right; pull it toward you: it comes
// closer; twist it: the model turns that way); camera mode flies the camera instead, every
// move the other way. `flat` (the 2D window): no turning.
void navigateSixDof(OrbitCamera& camera, const SixDof& in, float dt, float sensitivity, float viewportHeight, bool flat,
                    bool cameraMode = false);

} // namespace sicdm

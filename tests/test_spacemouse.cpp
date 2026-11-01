#include "TestFramework.hpp"

#include "core/Preferences.hpp"
#include "core/SpaceMouse.hpp"

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

using namespace sicdm;

#define REQUIRE(c)                  \
    do {                            \
        const bool required_ = (c); \
        CHECK(required_);           \
        if (!required_) return;     \
    } while (0)

TEST_CASE("3D mouse: spacenavd events and HID reports decode to the same axes") {
    const std::int32_t motion[8] = {0, 100, -50, 25, 10, -20, 30, 16};
    const auto m = decodeSpnav(std::span<const std::int32_t, 8>(motion, 8));
    REQUIRE(m.has_value());
    CHECK(m->kind == SpaceMouseEvent::Motion && m->motion.tx == 100 && m->motion.ty == -50 && m->motion.tz == 25);
    CHECK(m->motion.rx == 10 && m->motion.ry == -20 && m->motion.rz == 30);
    const std::int32_t press[8] = {1, 1, 0, 0, 0, 0, 0, 0};
    const auto b = decodeSpnav(std::span<const std::int32_t, 8>(press, 8));
    CHECK(b && b->kind == SpaceMouseEvent::Press && b->button == 1);
    const std::int32_t junk[8] = {9, 0, 0, 0, 0, 0, 0, 0};
    CHECK(!decodeSpnav(std::span<const std::int32_t, 8>(junk, 8)));

    // HID: X right, Y toward the user, Z down (little-endian int16s).
    auto put = [](std::vector<std::uint8_t>& r, std::int16_t v) {
        r.push_back(static_cast<std::uint8_t>(v & 0xFF));
        r.push_back(static_cast<std::uint8_t>((v >> 8) & 0xFF));
    };
    HidState s;
    std::vector<std::uint8_t> t{1};
    put(t, 100), put(t, 25), put(t, 50); // X, Y, Z
    CHECK(decodeHidReport(t, s));
    CHECK(s.motion.tx == 100 && s.motion.tz == 25 && s.motion.ty == -50); // lifting the cap (Z up, negative) is +y
    std::vector<std::uint8_t> r{2};
    put(r, 10), put(r, 30), put(r, 20); // Rx, Ry, Rz
    CHECK(decodeHidReport(r, s));
    CHECK(s.motion.rx == 10 && s.motion.rz == 30 && s.motion.ry == -20);
    CHECK(s.motion.tx == 100); // the translation stays from its own report
    // Newer devices: both in report 1.
    HidState s2;
    std::vector<std::uint8_t> both{1};
    for (int v = 1; v <= 6; ++v) put(both, static_cast<std::int16_t>(v));
    CHECK(decodeHidReport(both, s2) && s2.motion.tx == 1 && s2.motion.rx == 4 && s2.motion.ry == -6);
    std::vector<std::uint8_t> buttons{3, 0x02, 0x00};
    CHECK(decodeHidReport(buttons, s2) && s2.buttons == 2u);
    CHECK(!decodeHidReport(std::vector<std::uint8_t>{1, 2}, s2)); // too short
}

TEST_CASE("3D mouse: the cap moves the view; at rest nothing moves; camera mode goes the other way") {
    OrbitCamera cam;
    cam.setAspect(1.5f);
    cam.setStandardView(StandardView::Isometric, false);
    const float d0 = cam.distance(), yaw0 = cam.yaw();
    const Vec3 t0 = cam.target();
    // A hand resting on it (inside the dead zone) doesn't drift.
    navigateSixDof(cam, {5, -6, 7, 4, -3, 2}, 0.016f, 1.0f, 800.0f, false);
    CHECK(cam.distance() == d0 && cam.yaw() == yaw0 && length(cam.target() - t0) == 0.0f);
    // Pull toward you: closer. Push away: further.
    navigateSixDof(cam, {0, 0, 300, 0, 0, 0}, 0.05f, 1.0f, 800.0f, false);
    CHECK(cam.distance() < d0);
    const float d1 = cam.distance();
    navigateSixDof(cam, {0, 0, -300, 0, 0, 0}, 0.05f, 1.0f, 800.0f, false, true); // camera mode: reversed
    CHECK(cam.distance() < d1);
    // Push right: the model goes right on screen, so the view's centre moves left.
    const Vec3 before = cam.target();
    navigateSixDof(cam, {300, 0, 0, 0, 0, 0}, 0.05f, 1.0f, 800.0f, false);
    CHECK(dot(cam.target() - before, cam.right()) < 0.0f);
    // A twist turns it; not in the 2D window.
    const float y1 = cam.yaw();
    navigateSixDof(cam, {0, 0, 0, 0, 300, 0}, 0.05f, 1.0f, 800.0f, true);
    CHECK(cam.yaw() == y1);
    navigateSixDof(cam, {0, 0, 0, 0, 300, 0}, 0.05f, 1.0f, 800.0f, false);
    CHECK(std::fabs(cam.yaw() - y1) > 0.01f);
    // Faster with the speed up; a stall (a long frame) is capped.
    OrbitCamera a, b;
    navigateSixDof(a, {0, 0, 300, 0, 0, 0}, 0.05f, 1.0f, 800.0f, false);
    navigateSixDof(b, {0, 0, 300, 0, 0, 0}, 0.05f, 2.0f, 800.0f, false);
    CHECK(b.distance() < a.distance());

    Preferences p;
    std::string err;
    CHECK(p.spaceMouse == "object" && p.spaceMouseSpeed == 1.0f);
    CHECK(setPreference(p, "spacemouse", "camera", &err) && p.spaceMouse == "camera");
    CHECK(setPreference(p, "spacemouse", "off", &err) && p.spaceMouse == "off");
    CHECK(!setPreference(p, "spacemouse", "turbo", &err));
    CHECK(setPreference(p, "spacemouse-speed", "2", &err) && p.spaceMouseSpeed == 2.0f);
    CHECK(!setPreference(p, "spacemouse-speed", "9", &err));
}

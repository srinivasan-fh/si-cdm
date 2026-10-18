#include "TestFramework.hpp"

#include "core/Camera.hpp"

using namespace sicdm;

TEST_CASE("front view looks along +Y with Z up") {
    OrbitCamera cam;
    cam.setStandardView(StandardView::Front, false);
    CHECK_NEAR(cam.forward().y, 1, 1e-5);
    CHECK_NEAR(cam.up().z, 1, 1e-5);
    CHECK_NEAR(cam.right().x, 1, 1e-5);
}

TEST_CASE("top view looks down -Z with X right") {
    OrbitCamera cam;
    cam.setStandardView(StandardView::Top, false);
    CHECK(cam.forward().z < -0.999f);
    CHECK_NEAR(cam.right().x, 1, 1e-5);
    CHECK(cam.up().y > 0.999f);
}

TEST_CASE("orbit keeps distance to target") {
    OrbitCamera cam;
    const float d = cam.distance();
    cam.orbit(120, -45);
    CHECK_NEAR(length(cam.position() - cam.target()), d, 1e-3);
}

TEST_CASE("pitch is clamped short of the poles") {
    OrbitCamera cam;
    cam.orbit(0, 100000);
    CHECK(cam.pitch() < radians(90));
    cam.orbit(0, -200000);
    CHECK(cam.pitch() > -radians(90));
}

TEST_CASE("fitToBounds centres the target and frames the sphere") {
    OrbitCamera cam;
    cam.setAspect(1.0f);
    AABB b;
    b.expand(Vec3{10, 10, 10});
    b.expand(Vec3{20, 30, 40});
    cam.fitToBounds(b, false);
    CHECK_NEAR(cam.target().x, 15, 1e-4);
    CHECK_NEAR(cam.target().y, 20, 1e-4);
    CHECK_NEAR(cam.target().z, 25, 1e-4);
    CHECK(cam.distance() > b.radius());
}

TEST_CASE("animated transition reaches the target pose") {
    OrbitCamera cam;
    cam.setStandardView(StandardView::Right);
    int steps = 0;
    while (cam.update(1.0f / 60.0f) && steps < 1000) ++steps;
    CHECK(steps < 1000);
    CHECK_NEAR(cam.forward().x, -1, 1e-4);
}

TEST_CASE("pan moves target in the view plane") {
    OrbitCamera cam;
    cam.setStandardView(StandardView::Front, false);
    const Vec3 before = cam.target();
    cam.pan(-100, 0, 1000);
    const Vec3 delta = cam.target() - before;
    CHECK(delta.x > 0.0f);
    CHECK_NEAR(delta.y, 0, 1e-5);
    CHECK_NEAR(delta.z, 0, 1e-5);
}

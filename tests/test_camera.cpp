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

TEST_CASE("view direction matches the ViewCube region clicked") {
    OrbitCamera cam;
    cam.setViewDirection({1, -1, 1}, false); // top-front-right corner
    const Vec3 toEye = normalize(cam.position() - cam.target());
    CHECK_NEAR(toEye.x, 0.57735, 1e-4);
    CHECK_NEAR(toEye.y, -0.57735, 1e-4);
    CHECK_NEAR(toEye.z, 0.57735, 1e-4);
    cam.setViewDirection({0, 0, 1}, false); // top face
    CHECK(cam.forward().z < -0.9999f);
    CHECK_NEAR(cam.right().x, 1, 1e-4);
}

TEST_CASE("rotateAroundZ turns front into right") {
    OrbitCamera cam;
    cam.setStandardView(StandardView::Front, false);
    cam.rotateAroundZ(kPi * 0.5f, false);
    CHECK_NEAR(cam.forward().x, -1, 1e-4);
}

TEST_CASE("zoomAt keeps the point under the cursor fixed") {
    OrbitCamera cam;
    cam.setAspect(2.0f);
    cam.setProjection(Projection::Orthographic);
    cam.setStandardView(StandardView::Top, false);
    const Ray before = cam.screenRay(150, 80, 800, 400);
    cam.zoomAt(3.0f, 150, 80, 800, 400);
    const Ray after = cam.screenRay(150, 80, 800, 400);
    float t0 = 0, t1 = 0;
    CHECK(intersectPlane(before, {0, 0, 0}, {0, 0, 1}, t0));
    CHECK(intersectPlane(after, {0, 0, 0}, {0, 0, 1}, t1));
    CHECK_NEAR(before.at(t0).x, after.at(t1).x, 1e-3);
    CHECK_NEAR(before.at(t0).y, after.at(t1).y, 1e-3);
}

TEST_CASE("fit during a view transition keeps the new orientation") {
    OrbitCamera cam;
    cam.setStandardView(StandardView::Top, false);
    cam.setStandardView(StandardView::Front, true);
    AABB b;
    b.expand(Vec3{-1, -1, -1});
    b.expand(Vec3{1, 1, 1});
    cam.fitToBounds(b, true);
    int steps = 0;
    while (cam.update(1.0f / 60.0f) && steps < 1000) ++steps;
    CHECK_NEAR(cam.forward().y, 1, 1e-4);
}

TEST_CASE("roll turns the camera around the view direction") {
    OrbitCamera cam;
    cam.setStandardView(StandardView::Front, false);
    const Vec3 fwd = cam.forward();
    cam.rollView(kPi * 0.5f, false); // picture turns counter-clockwise
    CHECK_NEAR(dot(cam.forward(), fwd), 1.0, 1e-5);
    CHECK_NEAR(cam.right().z, -1, 1e-4);
    CHECK_NEAR(cam.up().x, 1, 1e-4);
    CHECK_NEAR(dot(cam.right(), cam.up()), 0, 1e-5);
    // The view matrix agrees: world +X now points up on screen.
    CHECK_NEAR(cam.view().transformDir({1, 0, 0}).y, 1, 1e-4);
}

TEST_CASE("roll presses chain during the animation and standard views level out") {
    OrbitCamera cam;
    cam.setStandardView(StandardView::Front, false);
    cam.rollView(kPi * 0.5f);
    cam.rollView(kPi * 0.5f);
    int steps = 0;
    while (cam.update(1.0f / 60.0f) && steps < 1000) ++steps;
    CHECK_NEAR(std::fabs(cam.roll()), kPi, 1e-4);
    cam.setStandardView(StandardView::Isometric, false);
    CHECK_NEAR(cam.roll(), 0, 1e-6);
    cam.rollView(0.3f, false);
    cam.resetRoll(false);
    CHECK_NEAR(cam.roll(), 0, 1e-6);
}

TEST_CASE("rolled orbit follows the cursor") {
    // With the picture rolled 90 degrees, a vertical drag spins around world Z.
    OrbitCamera cam;
    cam.setStandardView(StandardView::Front, false);
    cam.rollView(kPi * 0.5f, false);
    const float yaw = cam.yaw(), pitch = cam.pitch();
    cam.orbit(0, 50);
    CHECK(std::fabs(cam.yaw() - yaw) > 0.1f);
    CHECK_NEAR(cam.pitch(), pitch, 1e-5);
}

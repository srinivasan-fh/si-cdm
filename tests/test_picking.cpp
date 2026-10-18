#include "TestFramework.hpp"

#include "core/Camera.hpp"
#include "core/Scene.hpp"

using namespace sicdm;

namespace {

Scene twoBoxes() {
    Scene s;
    s.add(meshBody("near", primitives::box({-1, -1, 0}, {1, 1, 2})));
    s.add(meshBody("far", primitives::box({-1, 9, 0}, {1, 11, 2})));
    return s;
}

} // namespace

TEST_CASE("centre-screen ray points along the view direction") {
    OrbitCamera cam;
    cam.setAspect(1.5f);
    const Ray r = cam.screenRay(300, 200, 600, 400);
    CHECK_NEAR(dot(r.dir, cam.forward()), 1.0, 1e-5);
    cam.setProjection(Projection::Orthographic);
    const Ray o = cam.screenRay(300, 200, 600, 400);
    CHECK_NEAR(dot(o.dir, cam.forward()), 1.0, 1e-5);
}

TEST_CASE("raycast returns the closest body") {
    const Scene s = twoBoxes();
    const Ray r{{0, -10, 1}, {0, 1, 0}};
    const auto hit = s.raycast(r);
    CHECK(hit.has_value());
    if (hit) {
        CHECK(hit->body == 0);
        CHECK_NEAR(hit->point.y, -1, 1e-4);
    }
}

TEST_CASE("raycast honours offsets and visibility") {
    Scene s = twoBoxes();
    s.bodies()[0].offset = {5, 0, 0}; // moved out of the ray's way
    const Ray r{{0, -10, 1}, {0, 1, 0}};
    auto hit = s.raycast(r);
    CHECK(hit && hit->body == 1);
    s.bodies()[1].visible = false;
    hit = s.raycast(r);
    CHECK(!hit.has_value());
}

TEST_CASE("clicking the screen centre picks what the camera looks at") {
    Scene s = twoBoxes();
    OrbitCamera cam;
    cam.setAspect(1.0f);
    AABB b = s.bodies()[0].bounds();
    cam.fitToBounds(b, false);
    const auto hit = s.raycast(cam.screenRay(250, 250, 500, 500));
    CHECK(hit && hit->body == 0);
}

TEST_CASE("add and remove bump the scene revision") {
    Scene s;
    const auto r0 = s.revision();
    s.add(meshBody("a", primitives::box({0, 0, 0}, {1, 1, 1})));
    CHECK(s.revision() != r0);
    const auto r1 = s.revision();
    s.remove(0);
    CHECK(s.revision() != r1);
    CHECK(s.bodies().empty());
}

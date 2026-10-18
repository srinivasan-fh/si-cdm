#include "TestFramework.hpp"

#include "core/Math.hpp"

using namespace sicdm;

TEST_CASE("cross product follows the right-hand rule") {
    const Vec3 z = cross({1, 0, 0}, {0, 1, 0});
    CHECK_NEAR(z.x, 0, 1e-6);
    CHECK_NEAR(z.y, 0, 1e-6);
    CHECK_NEAR(z.z, 1, 1e-6);
}

TEST_CASE("matrix inverse round-trips") {
    const Mat4 m = translate({3, -2, 5}) * scale({2, 4, 0.5f});
    const Mat4 r = m * inverse(m);
    for (int c = 0; c < 4; ++c)
        for (int row = 0; row < 4; ++row) CHECK_NEAR(r.m[c][row], c == row ? 1.0 : 0.0, 1e-5);
}

TEST_CASE("lookAt maps the target onto the -Z axis") {
    const Mat4 v = lookAt({10, 0, 0}, {0, 0, 0}, {0, 0, 1});
    const Vec3 p = v.transformPoint({0, 0, 0});
    CHECK_NEAR(p.x, 0, 1e-5);
    CHECK_NEAR(p.y, 0, 1e-5);
    CHECK_NEAR(p.z, -10, 1e-5);
    // World +Z is screen up.
    const Vec3 up = v.transformDir({0, 0, 1});
    CHECK_NEAR(up.y, 1, 1e-5);
}

TEST_CASE("perspective maps near/far planes to NDC -1/+1") {
    const Mat4 p = perspective(radians(60), 1.5f, 0.1f, 100.0f);
    CHECK_NEAR(p.transformPoint({0, 0, -0.1f}).z, -1.0, 1e-4);
    CHECK_NEAR(p.transformPoint({0, 0, -100.0f}).z, 1.0, 1e-4);
}

TEST_CASE("AABB expands and reports centre") {
    AABB b;
    CHECK(!b.valid());
    b.expand(Vec3{-1, -2, -3});
    b.expand(Vec3{3, 2, 1});
    CHECK(b.valid());
    CHECK_NEAR(b.center().x, 1, 1e-6);
    CHECK_NEAR(b.center().z, -1, 1e-6);
}

#include "TestFramework.hpp"

#include "core/Scene.hpp"

using namespace sicdm;

namespace {

// Every triangle's winding must agree with its vertex normals (outward faces).
bool windingMatchesNormals(const MeshData& m) {
    for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3) {
        const Vertex& a = m.vertices[m.indices[i]];
        const Vertex& b = m.vertices[m.indices[i + 1]];
        const Vertex& c = m.vertices[m.indices[i + 2]];
        const Vec3 n = cross(b.position - a.position, c.position - a.position);
        if (length(n) < 1e-9f) continue; // degenerate (disk centre)
        if (dot(n, a.normal + b.normal + c.normal) <= 0.0f) return false;
    }
    return true;
}

} // namespace

TEST_CASE("box has 12 triangles, 12 edges and outward winding") {
    const MeshData m = primitives::box({0, 0, 0}, {1, 2, 3});
    CHECK(m.indices.size() == 36);
    CHECK(m.edgeVertices.size() == 24);
    CHECK(windingMatchesNormals(m));
    const AABB b = m.bounds();
    CHECK_NEAR(b.max.z, 3, 1e-6);
}

TEST_CASE("cylinder, tube and sphere wind outward") {
    CHECK(windingMatchesNormals(primitives::cylinder({0, 0, 0}, 2, 5, 32)));
    CHECK(windingMatchesNormals(primitives::tube({0, 0, 0}, 3, 1, 4, 32)));
    CHECK(windingMatchesNormals(primitives::sphere({1, 2, 3}, 2, 24, 12)));
}

TEST_CASE("cone, torus, wedge, pyramid and prism wind outward") {
    CHECK(windingMatchesNormals(primitives::cone({0, 0, 0}, 2, 0, 4, 32)));
    CHECK(windingMatchesNormals(primitives::cone({0, 0, 0}, 3, 1, 4, 32)));
    CHECK(windingMatchesNormals(primitives::torus({0, 0, 0}, 5, 1, 32, 16)));
    CHECK(windingMatchesNormals(primitives::wedge({0, 0, 0}, {4, 2, 3})));
    CHECK(windingMatchesNormals(primitives::pyramid({0, 0, 0}, 2, 3, 4)));
    CHECK(windingMatchesNormals(primitives::prism({0, 0, 0}, 2, 3, 6)));
}

TEST_CASE("primitive sizes match their parameters") {
    const AABB w = primitives::wedge({0, 0, 0}, {4, 2, 3}).bounds();
    CHECK_NEAR(w.max.x, 4, 1e-5);
    CHECK_NEAR(w.max.z, 3, 1e-5);
    const AABB t = primitives::torus({0, 0, 0}, 5, 1).bounds();
    CHECK_NEAR(t.max.x, 6, 1e-4);
    CHECK_NEAR(t.max.z, 1, 1e-4);
    CHECK(primitives::prism({0, 0, 0}, 2, 3, 6).edgeVertices.size() > 0);
}

TEST_CASE("append offsets indices") {
    MeshData m = primitives::box({0, 0, 0}, {1, 1, 1});
    const std::size_t v = m.vertices.size();
    m.append(primitives::box({2, 0, 0}, {3, 1, 1}));
    CHECK(m.vertices.size() == v * 2);
    CHECK(m.indices.back() >= v);
}

TEST_CASE("demo scene is non-empty and sits on the XY plane") {
    const Scene s = Scene::makeDemo();
    CHECK(s.bodies().size() >= 4);
    CHECK(s.triangleCount() > 1000);
    CHECK_NEAR(s.bounds().min.z, 0, 1e-4);
}

TEST_CASE("sketch planes are right-handed and round-trip points") {
    for (SketchPlane p : {SketchPlane::Top, SketchPlane::Front, SketchPlane::Right}) {
        const PlaneFrame f = planeFrame(p);
        const Vec3 n = cross(f.u, f.v);
        CHECK_NEAR(dot(n, f.normal), 1.0, 1e-6); // normal points towards the viewer
        const Vec2 q = f.toPlane(f.toWorld({3, -4}));
        CHECK_NEAR(q.x, 3, 1e-6);
        CHECK_NEAR(q.y, -4, 1e-6);
    }
}

TEST_CASE("sketch curves tessellate and measure") {
    Sketch s;
    SketchCurve r;
    r.kind = CurveKind::Rectangle;
    r.pts = {{0, 0}, {30, 20}};
    s.add(r);
    CHECK(s.curves()[0].worldStrip().size() == 5);
    CHECK(s.curves()[0].measure() == "30.0 x 20.0");
    CHECK(s.curves()[0].name == "Rectangle 1");
    SketchCurve c;
    c.kind = CurveKind::Circle;
    c.plane = SketchPlane::Front;
    c.pts = {{0, 10}};
    c.radius = 5;
    s.add(c);
    const AABB b = s.bounds();
    CHECK_NEAR(b.max.z, 15, 1e-4); // front plane: v is world Z
    const auto rev = s.revision();
    s.remove(0);
    CHECK(s.revision() != rev);
}

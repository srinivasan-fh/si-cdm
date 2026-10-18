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

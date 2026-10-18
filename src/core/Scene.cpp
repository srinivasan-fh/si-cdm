#include "core/Scene.hpp"

namespace sicdm {

AABB Scene::bounds() const noexcept {
    AABB visible, all;
    for (const Body& b : m_bodies) {
        const AABB bb = b.mesh.bounds();
        all.expand(bb);
        if (b.visible) visible.expand(bb);
    }
    return visible.valid() ? visible : all;
}

std::size_t Scene::triangleCount() const noexcept {
    std::size_t n = 0;
    for (const Body& b : m_bodies)
        if (b.visible) n += b.mesh.indices.size() / 3;
    return n;
}

Scene Scene::makeDemo() {
    using namespace primitives;
    Scene s;

    // Base plate 120 x 80 x 12 mm, sitting on the XY plane.
    s.add({"Base Plate", box({-60, -40, 0}, {60, 40, 12}), {0.70f, 0.73f, 0.78f}});

    // Central boss with bore.
    s.add({"Bearing Boss", tube({0, 0, 12}, 22, 12, 30), {0.82f, 0.62f, 0.32f}});

    // Upright web behind the boss.
    s.add({"Support Web", box({-8, 22, 12}, {8, 40, 42}), {0.45f, 0.60f, 0.78f}});

    // Four mounting studs.
    const float sx = 46, sy = 28;
    const Vec3 studs[] = {{-sx, -sy, 12}, {sx, -sy, 12}, {sx, sy, 12}, {-sx, sy, 12}};
    MeshData studMesh;
    for (const Vec3& p : studs) studMesh.append(cylinder(p, 5, 8, 40));
    s.add({"Mounting Studs", std::move(studMesh), {0.55f, 0.57f, 0.60f}});

    // Datum sphere as a probe target.
    s.add({"Probe Datum", sphere({-40, 0, 20}, 8), {0.38f, 0.70f, 0.52f}});

    return s;
}

} // namespace sicdm

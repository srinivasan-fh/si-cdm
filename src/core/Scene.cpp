#include "core/Scene.hpp"

namespace sicdm {

AABB Body::bounds() const noexcept {
    AABB b = mesh.bounds();
    if (b.valid()) {
        b.min += offset;
        b.max += offset;
    }
    return b;
}

void Scene::remove(std::size_t index) {
    if (index >= m_bodies.size()) return;
    m_bodies.erase(m_bodies.begin() + static_cast<std::ptrdiff_t>(index));
    ++m_revision;
}

std::optional<RayHit> Scene::raycast(const Ray& ray) const noexcept {
    std::optional<RayHit> best;
    for (std::size_t i = 0; i < m_bodies.size(); ++i) {
        const Body& b = m_bodies[i];
        if (!b.visible) continue;
        float tBox = 0.0f;
        if (!intersectAABB(ray, b.bounds(), tBox)) continue;
        if (best && tBox > best->t) continue;

        // Test in the body's local frame.
        const Ray local{ray.origin - b.offset, ray.dir};
        const auto& v = b.mesh.vertices;
        const auto& idx = b.mesh.indices;
        for (std::size_t k = 0; k + 2 < idx.size(); k += 3) {
            float t = 0.0f;
            if (intersectTriangle(local, v[idx[k]].position, v[idx[k + 1]].position, v[idx[k + 2]].position, t) &&
                (!best || t < best->t))
                best = RayHit{static_cast<int>(i), t, ray.at(t)};
        }
    }
    return best;
}

AABB Scene::bounds() const noexcept {
    AABB visible, all;
    for (const Body& b : m_bodies) {
        const AABB bb = b.bounds();
        all.expand(bb);
        if (b.visible) visible.expand(bb);
    }
    visible.expand(m_sketch.bounds());
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

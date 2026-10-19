#include "core/Scene.hpp"

namespace sicdm {

Body meshBody(std::string name, MeshData mesh, Vec3 color) {
    Body b;
    b.name = std::move(name);
    b.mesh = std::move(mesh);
    b.color = color;
    return b;
}

Body makeBody(std::string name, std::string kind, std::vector<float> params, Vec3 offset, Vec3 color) {
    Body b;
    b.name = std::move(name);
    b.mesh = buildPrimitive(kind, params);
    b.kind = std::move(kind);
    b.params = std::move(params);
    b.offset = offset;
    b.color = color;
    return b;
}

void Body::rebuild() {
    if (kind == "mesh") return;
    mesh = buildPrimitive(kind, params);
    if (scale != 1.0f) mesh.transform(sicdm::scale(Vec3{scale, scale, scale}));
}

void Body::setScale(float s) {
    if (!(s > 0.0f) || s == scale) return;
    if (kind == "mesh") {
        const float k = s / scale;
        mesh.transform(sicdm::scale(Vec3{k, k, k}));
        scale = s;
    } else {
        scale = s;
        rebuild();
    }
}

std::array<Vec3, 4> ImageRef::corners() const noexcept {
    const PlaneFrame f = planeFrame(plane);
    const float hw = width * 0.5f, hh = height() * 0.5f;
    return {f.toWorld({center.x - hw, center.y - hh}), f.toWorld({center.x + hw, center.y - hh}),
            f.toWorld({center.x + hw, center.y + hh}), f.toWorld({center.x - hw, center.y + hh})};
}

void Scene::removeImage(std::size_t index) {
    if (index >= m_images.size()) return;
    m_images.erase(m_images.begin() + static_cast<std::ptrdiff_t>(index));
    m_imageRevision = nextRevision();
}

void Scene::clear() {
    m_bodies.clear();
    m_images.clear();
    m_motions.clear();
    m_sketch = Sketch{};
    m_revision = nextRevision();
    m_imageRevision = nextRevision();
    m_sketch.touch();
}

void Scene::assign(Scene&& other) {
    m_bodies = std::move(other.m_bodies);
    m_images = std::move(other.m_images);
    m_motions = std::move(other.m_motions);
    m_sketch = std::move(other.m_sketch);
    m_revision = nextRevision();
    m_imageRevision = nextRevision();
    m_sketch.touch();
}

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
    m_revision = nextRevision();
}

std::optional<RayHit> Scene::raycast(const Ray& ray, const std::vector<Mat4>* transforms) const noexcept {
    std::optional<RayHit> best;
    for (std::size_t i = 0; i < m_bodies.size(); ++i) {
        const Body& b = m_bodies[i];
        if (!b.visible) continue;
        // Moved body: bring the ray into its modelled position (rigid, so t is unchanged).
        Ray r = ray;
        if (transforms && i < transforms->size()) {
            const Mat4 inv = inverse((*transforms)[i]);
            r = Ray{inv.transformPoint(ray.origin), inv.transformDir(ray.dir)};
        }
        float tBox = 0.0f;
        if (!intersectAABB(r, b.bounds(), tBox)) continue;
        if (best && tBox > best->t) continue;

        // Test in the body's local frame.
        const Ray local{r.origin - b.offset, r.dir};
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
    for (const ImageRef& img : m_images)
        if (img.visible)
            for (const Vec3& c : img.corners()) visible.expand(c);
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
    s.add(makeBody("Base Plate", "box", {-60, -40, 0, 60, 40, 12}, {}, {0.70f, 0.73f, 0.78f}));

    // Central boss with bore.
    s.add(makeBody("Bearing Boss", "tube", {22, 12, 30}, {0, 0, 12}, {0.82f, 0.62f, 0.32f}));

    // Upright web behind the boss.
    s.add(makeBody("Support Web", "box", {-8, 22, 12, 8, 40, 42}, {}, {0.45f, 0.60f, 0.78f}));

    // Four mounting studs.
    const float sx = 46, sy = 28;
    const Vec3 studs[] = {{-sx, -sy, 12}, {sx, -sy, 12}, {sx, sy, 12}, {-sx, sy, 12}};
    MeshData studMesh;
    for (const Vec3& p : studs) studMesh.append(cylinder(p, 5, 8, 40));
    s.add(meshBody("Mounting Studs", std::move(studMesh), {0.55f, 0.57f, 0.60f}));

    // Datum sphere as a probe target.
    s.add(makeBody("Probe Datum", "sphere", {8}, {-40, 0, 12}, {0.38f, 0.70f, 0.52f}));

    return s;
}

} // namespace sicdm

#pragma once

#include "core/Geometry.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace sicdm {

struct Body {
    std::string name;
    MeshData mesh;
    Vec3 color{0.72f, 0.75f, 0.80f};
    bool visible = true;
    Vec3 offset{}; // translation applied on top of the mesh

    AABB bounds() const noexcept;
};

struct RayHit {
    int body = -1;
    float t = 0.0f;
    Vec3 point;
};

class Scene {
public:
    std::vector<Body>& bodies() noexcept { return m_bodies; }
    const std::vector<Body>& bodies() const noexcept { return m_bodies; }

    Body& add(Body body) {
        ++m_revision;
        return m_bodies.emplace_back(std::move(body));
    }
    void remove(std::size_t index);

    // Bumped whenever bodies are added or removed (GPU buffers must be rebuilt).
    std::uint64_t revision() const noexcept { return m_revision; }

    // Closest visible body hit by the ray, if any.
    std::optional<RayHit> raycast(const Ray& ray) const noexcept;

    // Bounds of visible bodies (or all bodies if none are visible).
    AABB bounds() const noexcept;

    std::size_t triangleCount() const noexcept;

    // A small machined part shown on the landing canvas.
    static Scene makeDemo();

private:
    std::vector<Body> m_bodies;
    std::uint64_t m_revision = 0;
};

} // namespace sicdm

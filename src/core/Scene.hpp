#pragma once

#include "core/Geometry.hpp"

#include <string>
#include <vector>

namespace sicdm {

struct Body {
    std::string name;
    MeshData mesh;
    Vec3 color{0.72f, 0.75f, 0.80f};
    bool visible = true;
};

class Scene {
public:
    std::vector<Body>& bodies() noexcept { return m_bodies; }
    const std::vector<Body>& bodies() const noexcept { return m_bodies; }

    Body& add(Body body) { return m_bodies.emplace_back(std::move(body)); }

    // Bounds of visible bodies (or all bodies if none are visible).
    AABB bounds() const noexcept;

    std::size_t triangleCount() const noexcept;

    // A small machined part shown on the landing canvas.
    static Scene makeDemo();

private:
    std::vector<Body> m_bodies;
};

} // namespace sicdm

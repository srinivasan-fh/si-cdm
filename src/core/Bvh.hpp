#pragma once

// A bounding volume hierarchy over a mesh's triangles, for picking: a ray finds its
// nearest triangle in about log(n) box tests instead of testing every triangle.

#include "core/Geometry.hpp"

#include <cstdint>
#include <vector>

namespace sicdm {

class MeshBvh {
public:
    MeshBvh() = default;
    explicit MeshBvh(const MeshData& m) { build(m); }

    // Splits the triangles at the middle of their centres' widest spread, down to a few
    // per leaf.
    void build(const MeshData& m);
    bool empty() const noexcept { return m_nodes.empty(); }
    std::size_t triangles() const noexcept { return m_tris.size(); }
    AABB bounds() const noexcept { return m_nodes.empty() ? AABB{} : m_nodes.front().box; }

    // The nearest hit along the ray (in the mesh's coordinates) closer than `t`; `t` and
    // `tri` (the triangle's first index / 3) are set when there is one.
    bool raycast(const Ray& r, float& t, std::uint32_t* tri = nullptr) const noexcept;

private:
    struct Node {
        AABB box;
        std::uint32_t first = 0, count = 0; // leaf: triangles [first, first + count)
        std::uint32_t left = 0, right = 0;  // inner (count == 0): children
    };
    struct Tri {
        Vec3 a, b, c;
        std::uint32_t index;
    };
    std::vector<Node> m_nodes;
    std::vector<Tri> m_tris;
};

} // namespace sicdm

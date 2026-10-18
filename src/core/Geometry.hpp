#pragma once

#include "core/Math.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace sicdm {

struct Vertex {
    Vec3 position;
    Vec3 normal;
};

// Renderable tessellation of a body: shaded triangles plus the
// feature edges drawn on top (the classic "shaded with edges" CAD look).
struct MeshData {
    std::vector<Vertex> vertices;
    std::vector<std::uint32_t> indices;  // triangle list
    std::vector<Vec3> edgeVertices;      // line list, pairs of points

    AABB bounds() const noexcept;
    void transform(const Mat4& m);
    void append(const MeshData& other);
};

namespace primitives {

// Axis-aligned box spanning [min, max].
MeshData box(const Vec3& min, const Vec3& max);

// Cylinder along +Z, base centred at `base`.
MeshData cylinder(const Vec3& base, float radius, float height, int segments = 64);

// Tube (cylinder with a coaxial hole) along +Z.
MeshData tube(const Vec3& base, float outerRadius, float innerRadius, float height, int segments = 64);

// UV sphere.
MeshData sphere(const Vec3& center, float radius, int segments = 48, int rings = 24);

} // namespace primitives

} // namespace sicdm

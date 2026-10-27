#pragma once

// Exact collisions between parts (clean-room): two meshes, each placed by a rigid motion,
// collide when a triangle of one crosses a triangle of the other, or when one lies wholly
// inside the other (a closed mesh). Each mesh keeps a bounding volume hierarchy of its
// triangles: the two trees are walked together, each pair of boxes tested as oriented
// boxes (separating axes), so only the triangles near where the parts meet are compared.
//
// Robot planning (ARM MOVE_TO) uses it behind each part's box, so an L-shaped link, a
// gripper round a shaft or an arm reaching into a bin is no longer stopped by the box; and
// SIM CHECK sweeps a programmed motion for parts that run into each other.

#include "core/Geometry.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace sicdm {

class Scene;

// p' = R p + t (R row by row; mm).
struct Rigid {
    std::array<double, 9> R{1, 0, 0, 0, 1, 0, 0, 0, 1};
    std::array<double, 3> t{0, 0, 0};
    std::array<double, 3> apply(const std::array<double, 3>& p) const;
};
Rigid rigidOf(const Mat4& m); // the rotation and translation of an affine matrix

// Separating-axis test of two oriented boxes (centres, axes the columns of R, half sizes).
bool obbOverlap(const std::array<double, 3>& ca, const std::array<double, 9>& Ra, const std::array<double, 3>& ha, const std::array<double, 3>& cb,
                const std::array<double, 9>& Rb, const std::array<double, 3>& hb);

class CollisionShape {
public:
    // The mesh moved by `offset` (a body's mesh where the body is at rest).
    explicit CollisionShape(const MeshData& mesh, const Vec3& offset = {});
    bool empty() const noexcept { return m_nodes.empty(); }
    bool closed() const noexcept { return m_closed; } // every edge between two triangles: an inside
    std::size_t triangles() const noexcept { return m_tris.size(); }
    // Bounds, in its own coordinates.
    std::array<double, 3> lo() const;
    std::array<double, 3> hi() const;
    // The lowest point once placed (z), and the lowest above (x, y) of a height field: true
    // when some point is below `ground(x, y) - tolerance`.
    template <class Ground>
    bool below(const Rigid& at, Ground ground, double tolerance) const;
    // Whether a point (in its own coordinates) is inside (closed meshes only).
    bool contains(const std::array<double, 3>& p) const;

    friend bool shapesCollide(const CollisionShape& a, const Rigid& ta, const CollisionShape& b, const Rigid& tb, std::array<double, 3>* where);

private:
    struct Node {
        std::array<double, 3> lo, hi;
        std::uint32_t first = 0, count = 0, left = 0, right = 0; // leaf: count > 0
    };
    struct Tri {
        std::array<double, 3> a, b, c;
    };
    std::vector<Node> m_nodes;
    std::vector<Tri> m_tris;
    std::vector<std::array<double, 3>> m_points; // distinct vertices
    bool m_closed = false;
};

// True when the two placed shapes touch or cross (`where`: a point of contact, world mm).
bool shapesCollide(const CollisionShape& a, const Rigid& ta, const CollisionShape& b, const Rigid& tb, std::array<double, 3>* where = nullptr);

template <class Ground>
bool CollisionShape::below(const Rigid& at, Ground ground, double tolerance) const {
    for (const auto& p : m_points) {
        const auto w = at.apply(p);
        if (w[2] < ground(w[0], w[1]) - tolerance) return true;
    }
    return false;
}

// Parts running into each other while the simulation plays (the motions, robot programs and
// physics as the Simulation shows them).
struct MotionClash {
    std::string a, b;
    float from = 0, to = 0;        // s: when they first and last touch
    std::array<double, 3> at{};    // where, when they first touch (mm)
};
struct MotionCheck {
    std::vector<MotionClash> clashes;
    int steps = 0;
    std::size_t pairsLeftOut = 0;  // touching where they start (assembled parts, joints)
};
// From t0 to t1 every dt seconds (0: about 200 steps). Pairs already touching at t0 are left
// out unless `all`; `only` limits it to those bodies (and what they meet).
MotionCheck checkMotion(const Scene& scene, float t0, float t1, float dt = 0, bool all = false, const std::vector<std::size_t>& only = {});

} // namespace sicdm

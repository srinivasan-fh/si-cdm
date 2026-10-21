#pragma once

// Planar sections of a triangle mesh and the 2D loop operations the slicer and the
// CAM hole finder share.

#include "core/Math.hpp"

#include <array>
#include <cstdint>
#include <utility>
#include <vector>

namespace sicdm::section {

struct P2 {
    double x, y;
};
using Loop = std::vector<P2>;
using Tri = std::array<std::uint32_t, 3>;

// Signed area: positive for a counter-clockwise loop.
double loopArea(const Loop& l);

// Moves each edge `d` to its left (where the material is), with mitred corners. A
// loop that turns inside out (it was thinner than that) is dropped.
std::vector<Loop> offsetLeft(const std::vector<Loop>& loops, double d);

// The section of the mesh at height z, as closed loops with the material on their left
// (outlines counter-clockwise, holes clockwise). `candidates`: the triangles to test.
std::vector<Loop> contours(const std::vector<Vec3>& p, const std::vector<Tri>& tris, const std::vector<std::uint32_t>& candidates,
                           double z);

// Scanlines across the region inside `loops` (even-odd), `spacing` apart, at `angle`
// (radians), alternating direction.
std::vector<std::pair<P2, P2>> hatch(const std::vector<Loop>& loops, double spacing, double angle);

} // namespace sicdm::section

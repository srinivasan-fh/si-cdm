#pragma once

// Retopology: an even, all-quad mesh over a body's surface, its edges flowing along the
// surface's curvature where it is clearly curved and smoothly elsewhere, ready to subdivide,
// animate or take a texture. Clean-room: isotropic remeshing, a four-way direction field
// over the faces, triangles paired into quads along it, then one split of every face into
// quads, relaxed and kept on the original surface.

#include "core/Geometry.hpp"

#include <string>

namespace sicdm {

struct Body;

struct RetopoOptions {
    int faces = 2000;       // about this many quads
    int iterations = 6;     // remeshing passes
    bool curvature = true;  // let the edges follow the principal curvature where it is strong
};

struct RetopoReport {
    std::size_t quads = 0;
    std::size_t irregular = 0; // inside vertices not on four quads
    float edge = 0;            // mean edge length
    float quality = 0;         // mean corner squareness (1: all right angles)
};

// The quad mesh (world coordinates less the body's offset, so it sits where the body
// does): triangles two per quad, with `quads` and the quad edges as feature lines. False
// (why) when the body has no closed surface to follow.
bool retopologize(const Body& source, const RetopoOptions& options, MeshData& out, RetopoReport* report = nullptr, std::string* why = nullptr);

// Moves every multiresolution level of `body` onto the surface of `source` (each point to
// the nearest point there), so a fresh quad mesh takes the original's detail.
void projectLevelsOnto(Body& body, const Body& source);

} // namespace sicdm

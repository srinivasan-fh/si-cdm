#pragma once

// Multiresolution sculpting: a body kept at several subdivision levels at once. Level 0 is
// the coarse cage; each level above is the one below Loop-subdivided once, plus its own
// detail. Sculpting at a low level moves the large forms, and the detail of every level
// above rides along on the moved surface (in the surface's own frame, so it stays on the
// side it was sculpted on); sculpting at the top adds fine detail. Dynamic topology is off
// while a body has levels: the levels' vertices have to stay the same ones.

#include "core/Math.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace sicdm {

struct Body;
struct MeshData;

struct MultiresLevel {
    std::vector<Vec3> p;                            // positions (the body's mesh coordinates)
    std::vector<std::array<std::uint32_t, 3>> t;    // triangles
};

struct MultiresData {
    std::vector<MultiresLevel> levels; // coarse to fine
    int current = 0;                   // the level the body shows (and is sculpted at)
};

// One level of Loop subdivision (the edges' new points after the old ones, in a fixed
// order), so the same coarse level always gives the same fine one.
MultiresLevel loopSubdivideLevel(const MultiresLevel& in);

// The level as a mesh to show and sculpt (smooth normals, a vertex per position).
MeshData multiresMesh(const MultiresLevel& level);

// Starts levels on a body (its mesh becomes level 0) or adds `count` above the top;
// the body then shows the new top level. False (why) past `maxTriangles` or on no mesh.
bool multiresAdd(Body& body, int count, std::string* why = nullptr, std::size_t maxTriangles = 4000000);
// Takes the edits made at the shown level into the levels (the finer ones follow).
// False (why) when the mesh's triangles are no longer the level's (dynamic topology, a
// remesh): the levels are then dropped and the body keeps its mesh.
bool multiresSync(Body& body, std::string* why = nullptr);
// Shows (and sculpts at) another level, after taking in the edits at this one.
bool multiresLevel(Body& body, int level, std::string* why = nullptr);
// Keeps the shown mesh and drops the levels; or drops the levels below / above the shown one.
bool multiresApply(Body& body, std::string* why = nullptr);
bool multiresDeleteLower(Body& body, std::string* why = nullptr);
bool multiresDeleteHigher(Body& body, std::string* why = nullptr);

} // namespace sicdm

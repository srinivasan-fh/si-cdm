#pragma once

// Texture coordinates and texture maps. Unwrapping cuts a mesh into charts (pieces that
// face roughly one way, or its polygroups), flattens each with least-squares conformal
// maps (angles kept as well as they can be; a planar projection where that would fold),
// evens their scale and packs them into the unit square. Baking samples another body
// (a sculpt's high-resolution original) at every texel of the unwrapped one: its
// normals (tangent or object space), its height above, its colour, or how much of the
// sky each point sees (ambient occlusion). Clean-room implementations.

#include "core/Geometry.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace sicdm {

struct Body;

struct UvOptions {
    float angle = 66.0f;   // a chart's faces turn at most this many degrees from its first one
    float margin = 0.008f; // gap between charts, as a fraction of the square
    bool groups = true;    // polygroups are charts' borders too
};

struct UvReport {
    std::size_t charts = 0;
    std::size_t projected = 0; // charts flattened by projection (the conformal map folded)
    float fill = 0;            // fraction of the square the charts cover
    float stretch = 0;         // mean angle distortion (0: none; 0.1: 10% longer one way)
};

// Sets `mesh.uvs` (a pair per triangle corner). False (why) on no triangles.
bool unwrapUv(MeshData& mesh, const UvOptions& options, UvReport* report = nullptr, std::string* why = nullptr);

// A picture of the layout: each chart filled by its faces' direction, edges dark (RGB,
// size x size, v up).
std::vector<std::uint8_t> uvLayoutImage(const MeshData& mesh, int size);

enum class BakeMap { Normal, ObjectNormal, Height, Occlusion, Color };

// "normal", "objectnormal", "height", "occlusion" (also "ao"), "color" (also "colour").
bool parseBakeMap(const std::string& word, BakeMap& out);
const char* bakeMapName(BakeMap map);

struct BakeOptions {
    BakeMap map = BakeMap::Normal;
    int size = 1024;       // pixels a side
    int rays = 32;         // occlusion: rays a texel
    float distance = 0;    // how far from the surface the other one is looked for (0: 3% of its size);
                           // occlusion: how far a blocker counts (0: a fifth of its size)
    int padding = 4;       // texels the charts are grown by, so filtering doesn't bleed
};

struct BakeReport {
    std::size_t texels = 0; // inside the charts
    std::size_t missed = 0; // no hit along the normal: the nearest point taken
    float low = 0, high = 0; // height: the range written (mm)
};

// Bakes `source` onto `target`'s texture coordinates (both as placed in the scene).
// `pixels` gets size x size texels, rows from the top (v = 1): RGB bytes for normals and
// colour, grey 16-bit (high byte first, as PNG has it) for height, grey bytes for
// occlusion; `channels` and `bits` say which. False (why) when the target has no
// texture coordinates or either body no surface.
bool bakeMap(const Body& target, const Body& source, const BakeOptions& options, std::vector<std::uint8_t>& pixels, int& channels, int& bits,
             BakeReport* report = nullptr, std::string* why = nullptr);

} // namespace sicdm

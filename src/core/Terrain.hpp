#pragma once

// Terrain for robots to walk and drive on: a slope, stairs, rough ground, scattered
// blocks, or a digital elevation model (DEM) read from a file. It is a body in the scene
// (named "Terrain", drawn like any part) and the ground the robot engines stand on: its
// height and slope at every point. Outside its patch the ground is the floor at z = 0.
//
// DEM files: ESRI ASCII grids (.asc: heights and cell size in metres) and greyscale images
// (.png, .jpg, .pgm..., black low, white high, over a height range you give). Large grids
// are resampled to at most 400 x 400 points.

#include "core/Geometry.hpp"

#include <optional>
#include <string>
#include <vector>

namespace sicdm {

class Json;

enum class TerrainKind { None, Slope, Stairs, Rough, Blocks, Dem };
const char* terrainKindKey(TerrainKind k) noexcept;
std::optional<TerrainKind> terrainKindFromKey(const std::string& key) noexcept;

// Lengths in mm, angles in degrees.
struct TerrainSettings {
    TerrainKind kind = TerrainKind::None;
    float sizeX = 4000, sizeY = 4000;  // the patch, centred on `centre`
    Vec2 centre{0, 0};
    float direction = 0;               // the way slopes and stairs climb: degrees from +X toward +Y
    float angle = 10;                  // Slope
    float rise = 150, run = 300;       // Stairs: each step; they start `run` past the centre
    int steps = 6;
    float amplitude = 40, wavelength = 400; // Rough: height range and feature size
    float blockHeight = 60, blockSize = 250; // Blocks
    int seed = 1;
    float friction = 0.9f;
    // Dem (and any grid): heights (mm) on an nx x ny grid of `cell` mm, row by row from -Y,
    // over the patch.
    std::string demPath;
    int nx = 0, ny = 0;
    float cell = 0;
    std::vector<float> grid;

    bool active() const { return kind != TerrainKind::None; }
};

Json terrainToJson(const TerrainSettings& t);
bool terrainFromJson(const Json& j, TerrainSettings& t);

// Height (mm) of the ground at (x, y) mm: the terrain on its patch, 0 elsewhere.
double terrainHeight(const TerrainSettings& t, double x, double y);
// Upward unit normal there.
Vec3 terrainNormal(const TerrainSettings& t, double x, double y);
// The surface as a closed slab (its top, sides and a bottom at z = -50 mm).
MeshData terrainMesh(const TerrainSettings& t);

// Reads a DEM into t's grid (kind Dem): `cellMm` and `heightMm` (the range a greyscale
// image spans) override the file's own where it has none. False (and why) when it can't.
bool loadDem(const std::string& path, TerrainSettings& t, float cellMm, float heightMm, std::string* error);

// The body the terrain is drawn as.
inline constexpr const char* kTerrainBody = "Terrain";

} // namespace sicdm

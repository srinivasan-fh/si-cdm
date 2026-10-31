#pragma once

// Point clouds (laser scans): read from XYZ / PTS / ASC / CSV text or PLY, normals
// estimated and turned one way, and a closed mesh made from them.

#include "core/Geometry.hpp"

#include <string>
#include <vector>

namespace sicdm {

// Points (and normals, when the file has them: otherwise left empty) from a point file:
// text with x y z (and nx ny nz) per line, or a PLY file's vertices (ASCII or binary).
bool readPointCloud(const std::string& path, std::vector<Vec3>& points, std::vector<Vec3>& normals, std::string* why = nullptr);

// The usual spacing: the median distance from a point to its nearest neighbour.
float pointSpacing(const std::vector<Vec3>& points);

// Normals from each point's `k` nearest neighbours (the direction they spread least), turned
// to agree with their neighbours' (from the highest point, which faces up) .
std::vector<Vec3> estimateNormals(const std::vector<Vec3>& points, int k = 12);

struct ReconstructReport {
    float voxel = 0;               // the grid's spacing used
    std::size_t triangles = 0;
    std::size_t dropped = 0;       // triangles far from every point, taken away
};

// A mesh through the points: their signed distance (to the nearest one's tangent plane)
// meshed on a voxel grid (`voxel` 0: twice the spacing). Empty with a reason on failure.
MeshData reconstructSurface(const std::vector<Vec3>& points, const std::vector<Vec3>& normals, float voxel, ReconstructReport* report = nullptr,
                            std::string* why = nullptr);

} // namespace sicdm

#pragma once

// Part features on meshes, as a parametric CAD system offers them:
//
//   Revolve   a closed profile turned about an axis in its sketch plane
//   Sweep     a closed profile carried along a path, square to it, mitred at corners
//   Loft      a solid through closed sections, ruled or smooth
//   Pattern   linear (one or two directions) and circular copies
//   Mirror    across a plane
//   Round     rolling-ball fillets of a solid's convex edges (and/or concave ones),
//             from an exact signed distance field
//   Mass      volume, area, mass, centre of mass, inertia tensor, principal moments
//   Measure   the closest distance between two bodies

#include "core/Extrude.hpp"
#include "core/Geometry.hpp"
#include "core/Sketch.hpp"

#include <optional>
#include <string>
#include <vector>

namespace sicdm {

struct Body;

// The profile as a counter-clockwise polygon in its plane (circles as 96-gons).
std::vector<Vec2> profilePolygon(const Profile& p, int circleSegments = 96);

// Turns a closed profile about the axis through `axisPoint` along `axisDir` (sketch-plane
// coordinates) by `degrees`; 360 closes it. The profile must stay on one side of the axis.
std::optional<MeshData> revolveProfile(const Profile& profile, SketchPlane plane, Vec2 axisPoint, Vec2 axisDir, float degrees,
                                       std::string* error = nullptr);

// Carries a closed profile (centred on its centroid, its plane turned square to the path)
// along a path of world points; a closed path closes the tube.
std::optional<MeshData> sweepProfile(const Profile& profile, SketchPlane profilePlane, const std::vector<Vec3>& path, bool closedPath,
                                     std::string* error = nullptr);

// A solid through closed planar sections (world points) in order, ruled or smooth.
std::optional<MeshData> loftSections(const std::vector<std::vector<Vec3>>& sections, bool smooth, std::string* error = nullptr);

// A world mesh mirrored across the plane through `point` with `normal` (winding flipped).
MeshData mirrorMesh(const MeshData& m, const Vec3& point, const Vec3& normal);

struct RoundOptions {
    float radius = 2.0f;
    bool convex = true;   // round the outside edges (a ball rolled inside)
    bool concave = false; // fill the inside corners (a ball rolled outside)
    float voxel = 0.0f;   // 0: from the radius and the size
};
// Rounds the body's edges with a rolling ball of the radius. The body becomes a mesh.
bool roundBody(Body& body, const RoundOptions& options, std::string* error = nullptr);

// The sharp edges of a solid (faces meeting at more than `angleDeg`), joined into
// chains: straight runs between corners, or loops (a cylinder's rim). Each point keeps
// the normals of the faces on its left (nL) and right (nR) going along the chain.
struct EdgeChain {
    std::vector<Vec3> pts, nL, nR;
    bool closed = false;
    bool convex = true;  // an outside edge (else an inside corner)
    float length() const;
};
std::vector<EdgeChain> edgeChains(const MeshData& world, float angleDeg = 25.0f);

// Which chains: by number (from the listing), a point near them, or a filter.
struct EdgeFilter {
    bool convex = false, concave = false, vertical = false, horizontal = false, top = false, bottom = false;
    std::vector<int> numbers;  // 1-based
    std::vector<Vec3> near;    // the chain nearest each point
};
std::vector<int> selectEdges(const std::vector<EdgeChain>& chains, const MeshData& world, const EdgeFilter& f);

// Rounds (or chamfers, the cut `size` back along each face) the chosen chains exactly:
// a cutter swept along each chain is cut away (outside edges) or joined on (inside
// corners). Where several rounded edges meet, the corner is mitred, not blended.
bool roundEdges(Body& body, const std::vector<EdgeChain>& chains, const std::vector<int>& chosen, float size, bool chamfer,
                std::string* error = nullptr);

struct SolidMass {
    double volume = 0, area = 0, mass = 0; // mm^3, mm^2, g
    Vec3 centroid;
    double inertia[3][3] = {};             // about the centroid, g mm^2
    double principal[3] = {};              // principal moments, ascending
    AABB box;
};
SolidMass solidMass(const MeshData& world, double densityGcm3);

// Closest distance between two meshes (vertex to triangle, both ways), with the points.
double meshDistance(const MeshData& a, const MeshData& b, Vec3* onA = nullptr, Vec3* onB = nullptr);

// Material densities, g/cm^3 (steel, aluminium, ...); nullopt when unknown.
std::optional<double> materialDensity(const std::string& name);

// The body built from a world mesh: welded, wound outward, with normals and edges.
Body featureBody(std::string name, MeshData world, Vec3 color);

} // namespace sicdm

#pragma once

// Assemblies: parts placed against each other by mates, in a tree of sub-assemblies.
//
//   Tree    each body sits in an assembly path ("Gearbox/Input shaft"); an empty path is
//           the top level. STEP files keep the tree both ways.
//   Mates   fix, coincident (faces against each other, or flush), concentric, distance,
//           angle, parallel and perpendicular, between planar faces, axes (cylinders, cones,
//           tori) and points (sphere centres), each picked by a point on the face
//   Solve   moves the parts the mates leave free, as little as it can: the fixed parts
//           (and, when none is fixed, the first part mated) stay put. A library component
//           and the bodies attached to a part move with it. Reports the freedom left and
//           the mates that can't be met together
//   Check   interference between parts: where and how much they overlap

#include "core/Brep.hpp"

#include <string>
#include <vector>

namespace sicdm {

struct Body;
class Scene;

enum class MateType { Fix, Coincident, Concentric, Distance, Angle, Parallel, Perpendicular };
const char* mateTypeName(MateType t);
bool mateTypeFromName(const std::string& name, MateType* out, bool* aligned);

enum class MateGeom { Plane, Axis, Point };
const char* mateGeomName(MateGeom g);

// A face's feature in its body's own frame (world = offset + scale * rotation * p): a plane
// (a point on it and its outward normal), an axis (a point on it and its direction) or a
// point.
struct MateRef {
    std::string body;
    MateGeom geom = MateGeom::Plane;
    DVec3 p{0, 0, 0}, d{0, 0, 1};
};

struct Mate {
    std::string name;         // M1, M2...
    MateType type = MateType::Coincident;
    MateRef a, b;             // Fix: only a
    double value = 0;         // Distance: mm; Angle: degrees
    bool aligned = false;     // Coincident / Distance of planes: normals the same way (flush) instead of facing
    bool suppressed = false;  // kept but not solved
};

struct AssemblyState {
    std::vector<Mate> mates;
    bool empty() const { return mates.empty(); }
    std::string nextName() const;
};

// The feature of the body's face nearest `world` (exact faces: planes, axes of cylinders,
// cones and tori, sphere centres; meshes: the plane of the nearest triangle).
bool mateRefAt(const Body& body, const DVec3& world, MateRef& out, std::string* error = nullptr);
// A feature where it is now, in world coordinates.
void mateRefWorld(const Body& body, const MateRef& r, DVec3& p, DVec3& d);

struct MateSolveReport {
    bool solved = true;                 // every mate met
    int iterations = 0;
    double residual = 0;                // the largest mate error left (mm, or mm-equivalent)
    int freedom = 0;                    // degrees of freedom the mates leave the moving parts
    std::vector<std::string> moved;     // bodies that moved
    std::vector<std::string> fixed;     // the parts that held still
    std::vector<std::string> failing;   // mates not met (in conflict with others)
    std::vector<std::string> missing;   // mates naming bodies that are gone
};

// Moves the scene's bodies to meet the mates.
MateSolveReport solveMates(Scene& scene, const AssemblyState& st);
// How far a mate is from met now (mm; directions count as mm at a 10 mm lever arm). Negative:
// a body is missing.
double mateError(const Scene& scene, const Mate& m);
std::string describeMate(const Mate& m);

struct InterferenceVolume {
    std::string a, b;
    double volume = 0; // mm^3 overlapping
    Vec3 centre{};     // the middle of the overlap's box
    Vec3 size{};       // the overlap's box
};
// Pairs of the given bodies (default: every visible body) that overlap by more than
// `minVolume` (and a ten-thousandth of the smaller part), largest first. Bodies that only
// touch don't count.
std::vector<InterferenceVolume> findInterference(const Scene& scene, const std::vector<std::size_t>& bodies = {}, double minVolume = 1e-3);

// The assembly tree, one line per node: sub-assemblies (indented) and their parts.
std::vector<std::string> assemblyTree(const Scene& scene);
// Parts list: each distinct part (name without its copy number) and how many there are.
std::vector<std::pair<std::string, int>> billOfMaterials(const Scene& scene);

class Json;
Json assemblyToJson(const AssemblyState& st);
bool assemblyFromJson(const Json& j, AssemblyState& st);

} // namespace sicdm

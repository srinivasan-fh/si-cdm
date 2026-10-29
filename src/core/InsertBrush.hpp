#pragma once

// Insert-mesh and curve brushes (like ZBrush's IMM and curve brushes): a shape put on a
// surface where it is clicked, standing along the surface's normal; a tube laid along a
// stroke on the surface; copies of a shape along a stroke. They join the body by an exact
// union, are appended to its mesh as a polygroup of their own (to fuse later with REMESH, as
// in a sculpting workflow), or become a body of their own.

#include "core/Geometry.hpp"
#include "core/Math.hpp"

#include <optional>
#include <string>
#include <vector>

namespace sicdm {

struct Body;

enum class InsertShape { Sphere, Cube, Cylinder, Cone, Torus, Mesh };
enum class InsertJoin { Union, Append, NewBody };

const char* insertShapeName(InsertShape s);
std::optional<InsertShape> insertShapeFromName(const std::string& word);

struct InsertPlace {
    Vec3 at{};             // on the surface
    Vec3 normal{0, 0, 1};  // the surface's, outward: the shape stands along it
    Vec3 along{1, 0, 0};   // which way its X faces (a stroke's direction), square to the normal
};

// A shape `size` mm across (and tall), its base on the surface and `sink` (0..1) of its
// height below it, turned `spin` degrees about the normal. `custom`: Mesh's shape (any mesh,
// fitted into the same size).
MeshData insertShapeMesh(InsertShape shape, float size, const InsertPlace& place, float spin = 0, float sink = 0.15f, const MeshData* custom = nullptr);

// The surface stroke evened out: points every `step` mm along it (smoothed a little), with
// their normals and directions.
std::vector<InsertPlace> evenStroke(const std::vector<Vec3>& points, const std::vector<Vec3>& normals, float step);

// A tube of radius r along the stroke, round at both ends, its centre line `sink` of its
// diameter below the surface (0: resting on it, 0.5: half in).
MeshData curveTube(const std::vector<InsertPlace>& stroke, float radius, float sink = 0.5f);

// Copies of a shape along the stroke every `spacing` mm, each facing along it.
MeshData curveCopies(const std::vector<InsertPlace>& stroke, InsertShape shape, float size, float spacing, float sink = 0.15f,
                     const MeshData* custom = nullptr);

// Puts `piece` (model space) into the body: exactly united, appended as a polygroup of its
// own, or (NewBody) left for the caller in `separate`. False (why) when the union fails.
bool joinInsert(Body& body, const MeshData& piece, InsertJoin join, std::string* why = nullptr);

} // namespace sicdm

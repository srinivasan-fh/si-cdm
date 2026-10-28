#pragma once

// Direct editing: changing a solid by its faces, whatever made it (a STEP import, a mesh,
// a feature of this project), without a feature history.
//
//   Offset  a face moves out (or in) along its normal: push / pull. A flat face's
//           neighbours stretch or shrink to meet it where they are planes, or cylinders and
//           cones that run the way it moves; otherwise the slab between the face and where it
//           goes is added or cut (a round face: a hole or boss larger or smaller)
//   Delete  a hole or a boss (a round face with what closes its end) is filled or cut away;
//           a fillet or a chamfer between two flat faces goes, the faces meeting again
//   Move    a hole or a boss to another place; a flat face along its normal
//
// Faces are found by a point on them. A body with exact geometry keeps it: the faces lie
// on the surfaces they lay on, or on their offsets.

#include "core/Brep.hpp"

#include <string>

namespace sicdm {

struct Body;

enum class DirectFaceKind { Plane, Cylinder, Cone, Sphere, Torus, Freeform };
const char* directFaceKindName(DirectFaceKind k);

// What a face is: its kind, outward normal (flat faces) or axis and radius (round ones),
// its area and the triangles it has.
struct DirectFaceInfo {
    DirectFaceKind kind = DirectFaceKind::Freeform;
    bool exact = false;        // on a surface of the body's exact geometry
    DVec3 normal{0, 0, 1};     // flat: outward
    DVec3 point{0, 0, 0};      // flat: a point on it; round: on the axis (world)
    DVec3 axis{0, 0, 1};       // round: the axis
    double radius = 0;         // cylinder / sphere: radius; cone: radius at `point`; torus: tube radius
    bool hole = false;         // round: the material is outside it (a hole), not inside (a boss)
    double area = 0;           // mm^2
    std::size_t triangles = 0;
};

// The face of `body` at `world` (a point on it, or near it). False with why when the body has
// no face there.
bool directFaceAt(const Body& body, const Vec3& world, DirectFaceInfo& out, std::string* why = nullptr);

// Push / pull: the face at `world` moved `distance` mm along its outward normal (negative:
// in). `how` says what was done. False (the body unchanged) with why.
bool directOffsetFace(Body& body, const Vec3& world, double distance, std::string* why = nullptr, std::string* how = nullptr);
// The hole, boss, fillet or chamfer at `world` taken away.
bool directDeleteFace(Body& body, const Vec3& world, std::string* why = nullptr, std::string* how = nullptr);
// The hole or boss at `world` moved by `by` (a flat face: along its normal by the part of
// `by` along it).
bool directMoveFace(Body& body, const Vec3& world, const DVec3& by, std::string* why = nullptr, std::string* how = nullptr);

} // namespace sicdm

#pragma once

// Armatures (like ZSpheres): a tree of balls, each joined to its parent by a tapered link, to
// sketch a creature's or a part's form quickly. SKIN turns it into one smooth closed mesh
// (the balls and links fused with rounded blends) that stays bound to it: each skin vertex
// follows the links near it (linear blend skinning), so turning a ball (POSE) bends the
// skin with it.
//
// An armature is a body of kind "armature" whose params hold its balls, 9 floats each:
//   x, y, z, radius, parent (-1: the root), and the ball's turn as a quaternion qx qy qz qw
// The turn swings everything after the ball (its children, their children...) about it.

#include "core/Geometry.hpp"
#include "core/Math.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace sicdm {

struct ArmatureBall {
    Vec3 at{};                         // where it is modelled (the rest pose)
    float radius = 10.0f;
    int parent = -1;
    std::array<float, 4> turn{0, 0, 0, 1}; // its pose: a unit quaternion (x, y, z, w)
};

struct Armature {
    std::vector<ArmatureBall> balls;
};

constexpr std::size_t kArmatureFloats = 9;

std::vector<float> armatureParams(const Armature& a);
bool armatureFromParams(const std::vector<float>& p, Armature& out);

// Each ball's transform in the pose (it turns about its own centre, after its parent's), and
// where the ball is then.
std::vector<Mat4> armaturePose(const Armature& a);
std::vector<Vec3> posedBalls(const Armature& a);

// Balls and links as a light preview mesh, posed.
MeshData armaturePreview(const Armature& a);

struct SkinOptions {
    float voxel = 0;  // mm; 0: from the smallest ball (a quarter of its radius), within 256 per side
    float blend = -1; // mm, the rounding where links meet; < 0: a quarter of the smallest ball's radius
    int smooth = 6;   // smoothing passes on the marched surface
};

// The bone weights of a skin: for each vertex up to four links (by the index of the ball at
// their far end) and how much each moves it.
struct SkinData {
    std::string armature;                         // the armature body it follows
    std::vector<Vec3> rest;                       // its vertices in the rest pose (the armature's frame)
    std::vector<Vec3> base;                       // the same unsculpted (as skinned, refitted with it): rest - base is what was sculpted
    std::vector<std::array<std::int32_t, 4>> bone; // -1: none
    std::vector<std::array<float, 4>> weight;
    std::uint64_t key = 0;                        // meshKey() of the mesh it last made
    SkinOptions options;                          // what it was skinned with (to skin it again the same)
};

// The armature's skin as one closed mesh, with its bone weights (`skin`: rest and weights).
bool armatureSkin(const Armature& a, const SkinOptions& o, MeshData& out, SkinData& skin, std::string* why = nullptr);

// The skin's vertices in the armature's pose (linear blend skinning: a link moves with the
// ball at its near end).
std::vector<Vec3> poseSkin(const SkinData& skin, const Armature& a);

// The rest pose that `posed` (the skin's vertices as they are now, edited perhaps) would come
// from in the armature's pose: so edits made to a posed skin are kept when it is posed again.
std::vector<Vec3> unposeSkin(const SkinData& skin, const Armature& a, const std::vector<Vec3>& posed);

// Fitting a skin to an armature edited since it was skinned (both armatures in their rest
// pose):
// - its balls moved or resized, the same balls and links: the rest pose that follows them,
//   each vertex going with the links it follows (so sculpted detail stays);
std::vector<Vec3> refitSkinRest(const SkinData& skin, const Armature& oldRest, const Armature& newRest);
// - balls added or taken away: a new skin, with `detail` (how far each vertex of the old skin
//   was from where a fresh skin of the old armature puts it: what was sculpted) and `colors`
//   (polypaint, may be empty) carried over from the old skin's nearest vertices.
bool reskinWithDetail(const Armature& newRest, const SkinOptions& o, const std::vector<Vec3>& oldFresh, const std::vector<Vec3>& detail,
                      const std::vector<std::uint32_t>& colors, MeshData& out, SkinData& skin, std::string* why = nullptr);

// Area-weighted vertex normals (a skin's vertices are shared between its triangles).
void smoothVertexNormals(MeshData& m);

// Quaternions.
std::array<float, 4> quatFromAxisAngle(const Vec3& axis, float radiansAngle);
std::array<float, 4> quatMultiply(const std::array<float, 4>& a, const std::array<float, 4>& b);
std::array<float, 4> quatFromEuler(const Vec3& degrees); // about X, then Y, then Z (as eulerRotation)
Mat4 quatMatrix(const std::array<float, 4>& q);
// The turn as degrees about X, then Y, then Z.
Vec3 quatToEuler(const std::array<float, 4>& q);

} // namespace sicdm

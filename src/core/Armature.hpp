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
#include <map>
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
    // How a vertex follows its links: false (the default) blends their turns as dual
    // quaternions, so a twisted joint keeps its volume; true sums their matrices (linear blend
    // skinning, as glTF viewers pose skins; it thins a joint twisted far about its link).
    bool linear = false;
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

// Bone weights for points about the armature (as it stands: a posed armature's balls where
// posed): each follows the links nearest it, by inverse distance; `soft` widens the blend.
void skinWeights(const Armature& a, float soft, const std::vector<Vec3>& pts, SkinData& skin);
// The `soft` a skin of these settings uses.
float skinSoftness(const Armature& a, const SkinOptions& o);

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
// - balls added or taken away: a new skin, with what the old one carried brought over from
//   its nearest points: what was sculpted (`detail`, its rest pose less its unsculpted shape
//   `bare`; fading out where nothing of the old is near, as round a new ball), the paint, and
//   per triangle the polygroups and UVs (taken from the old triangle nearest each new one).
struct SkinCarry {
    std::vector<Vec3> bare, detail;    // per old vertex
    std::vector<std::uint32_t> colors; // per old vertex (may be empty)
    std::vector<std::uint32_t> indices; // the old triangles
    std::vector<std::uint32_t> groups; // per old triangle (may be empty)
    std::vector<Vec2> uvs;             // per old triangle corner (may be empty)
    // The old weights (per old vertex; may be empty) and each old ball's number in the new
    // armature (-1: taken away). Where the old skin was, each new vertex keeps the weights it
    // had there, sharing with any new ball's link what the new skin's own weights give it.
    std::vector<std::array<std::int32_t, 4>> bone;
    std::vector<std::array<float, 4>> weight;
    std::vector<int> ballMap;
};
bool reskinWithDetail(const Armature& newRest, const SkinOptions& o, const SkinCarry& carry, MeshData& out, SkinData& skin, std::string* why = nullptr);

// What was sculpted round ball `from` (its link's skin: the vertices it weights most) copied
// onto ball `to`'s (a new ball's skin starts plain): the skin's rest pose with each vertex on
// `to`'s link given the detail of the place that matches it on `from`'s (as far along, turned
// and sized with the link), in proportion to how much `to`'s link weights it (so it blends
// into what is about it). `mirrorAxis` 0, 1, 2: the place is instead the vertex mirrored
// across the plane through the root ball square to X, Y or Z, and the detail mirrored with it.
// Both balls must have a link (not the root); the skin must keep its unsculpted shape (`base`).
// `rest`: the armature at rest. False, with why, when it can't.
bool copySkinDetail(const SkinData& skin, const Armature& rest, int from, int to, int mirrorAxis, std::vector<Vec3>& out, std::string* why = nullptr);

class NearestPoints;
// The weights about `p` from points `pts` (with their bones and weights): those of the points
// within twice the nearest's distance, blended by nearness (the nearest's alone where p sits
// on one), each bone b taken as `map[b]` (-1: dropped; an empty map: as it is), the four
// largest kept and summing to 1. False when none is left.
bool weightsAbout(const NearestPoints& near, const std::vector<Vec3>& pts, const std::vector<std::array<std::int32_t, 4>>& bone,
                  const std::vector<std::array<float, 4>>& weight, const Vec3& p, const std::vector<int>& map, std::array<std::int32_t, 4>& outBone,
                  std::array<float, 4>& outWeight);

// The nearest of a set of points to a place, by cells (a cube a 64th of their extent unless
// given): its index, or -1 when none is within `within`.
class NearestPoints {
public:
    explicit NearestPoints(const std::vector<Vec3>& pts, float cell = 0);
    std::int64_t find(const Vec3& p, float within, float* distance = nullptr) const;
    // Every point within `radius` of p (in no order).
    void around(const Vec3& p, float radius, std::vector<std::uint32_t>& out) const;
    float cell() const noexcept { return m_cell; }

private:
    std::array<int, 3> key(const Vec3& p) const;
    const std::vector<Vec3>& m_pts;
    AABB m_box;
    float m_cell = 1;
    std::map<std::array<int, 3>, std::vector<std::uint32_t>> m_cells;
};

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

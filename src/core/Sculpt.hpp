#pragma once

// Digital sculpting and boolean (CSG) modelling on body meshes.
//
//   Brushes     add and remove material (Clay, Carve), push and pull (Draw, Inflate,
//               Grab), pinch and smooth (Pinch, Crease, Smooth, Flatten, Scrape).
//               The mesh refines itself under the brush (dynamic topology), so a
//               12-triangle block can be sculpted as freely as a dense scan.
//   Booleans    drill holes, cut pockets and channels, trim with a plane (knife),
//               add bosses, or combine two bodies (union / subtract / intersect).
//               Exact: polygons are split by a BSP tree of the other solid and kept
//               or dropped by their (fast) generalised winding number, then the
//               seams are welded, so the result is a closed mesh ready for STL /
//               OBJ export to slicers and CAM.
//
// Everything works in world coordinates (mesh + Body::offset); results are written
// back as kind "mesh" bodies.

#include "core/Geometry.hpp"
#include "core/Math.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

namespace sicdm {

struct BSurface;

struct HoleSpec;

struct Body;
class Scene;

enum class SculptBrush { Draw, Clay, Carve, Inflate, Grab, Pinch, Crease, Smooth, Flatten, Scrape,
                         Layer, ClayStrips, Fill, Blob, SnakeHook, Nudge, Mask, Paint };

// How a brush's effect falls off from its centre to its rim.
enum class SculptFalloff { Smooth, Sphere, Sharp, Linear, Constant };
const char* sculptFalloffKey(SculptFalloff f) noexcept;
std::optional<SculptFalloff> sculptFalloffFromKey(const std::string& key) noexcept;

// An alpha: a grey stamp the brush is pressed through (white: the full effect, black:
// none), laid over the brush's square. Built in (noise, ring, square, dots, scales,
// crosshatch, bricks, pores, weave, stripes) or read from an image.
struct SculptAlpha {
    std::string name;
    int width = 0, height = 0;
    std::vector<float> value; // rows top to bottom, 0..1
    // At (u, v) across the brush (-1..1 each way, v up); outside it, 0. `tile` repeats it
    // that many times across.
    float sample(float u, float v, int tile = 1) const noexcept;
    bool empty() const noexcept { return value.empty(); }
};
// A built-in alpha by name, or an image file (its brightness). Null (why) when neither.
std::shared_ptr<const SculptAlpha> sculptAlpha(const std::string& nameOrPath, std::string* why = nullptr);
const std::vector<std::string>& sculptAlphaNames();

const char* sculptBrushKey(SculptBrush b) noexcept;  // "draw", "clay", ...
const char* sculptBrushName(SculptBrush b) noexcept; // "Draw (Push / Pull)", ...
std::optional<SculptBrush> sculptBrushFromKey(const std::string& key) noexcept;
const std::vector<SculptBrush>& sculptBrushes();

struct SculptSettings {
    SculptBrush brush = SculptBrush::Draw;
    float radius = 20.0f;    // mm
    float strength = 0.5f;   // 0..1
    bool invert = false;     // remove instead of add, push instead of pull...
    bool symmetryX = false;  // mirror across the body's centre plane (x = const)
    bool symmetryY = false;  // ... and y = const, z = const (any combination)
    bool symmetryZ = false;
    int radial = 0;          // radial symmetry: this many copies around an axis through the centre
    int radialAxis = 2;      // 0 X, 1 Y, 2 Z
    SculptFalloff falloff = SculptFalloff::Smooth;
    bool refine = true;      // dynamic topology: split long edges under the brush
    float detail = 0.0f;     // mm, longest edge left under the brush; 0 = radius / 4
    Vec3 color{0.8f, 0.2f, 0.2f}; // Paint: the colour laid on (inverted: back to the body's own)
    // An alpha the brush is pressed through, turned by `alphaAngle` degrees (and with
    // `alphaFollow`, along the stroke), repeated `alphaTile` times across. With an alpha the
    // surface is divided finer under the brush (a twentieth of the radius) to take its detail.
    std::shared_ptr<const SculptAlpha> alpha;
    float alphaAngle = 0.0f;
    bool alphaFollow = false;
    int alphaTile = 1;
    float spacing = 0.0f; // dabs along a stroke, in radii apart (0: a quarter; 2: stamps side by side)
};

enum class MaskOp { Clear, Invert, Smooth, Grow, Shrink, All };
// Whole-body mask edits (the Mask brush paints it): CLEAR, INVERT, SMOOTH (blur),
// GROW / SHRINK by one ring of vertices, ALL (mask everything).
void maskBody(Body& body, MaskOp op);
// Fraction of the surface's vertices masked (0..1, weighted by the mask).
float maskedFraction(const Body& body);

// --- polypaint and polygroups -------------------------------------------------------------
// They are kept on the body's mesh (a parametric body becomes a mesh body of the same
// shape; its exact geometry stays).

// A colour from words: a name (red, skin, steel...), #RRGGBB, or r,g,b (0..1, or 0..255
// when any is above 1).
bool parseColorText(const std::string& text, Vec3& rgb);
// Fills the paint (held off by the mask, as far as it is masked); `group` >= 0: that
// polygroup's faces only. False when there is nothing to paint.
bool paintBody(Body& body, const Vec3& rgb, int group = -1);
void clearPaint(Body& body);
// The share of the surface's vertices painted (0..1).
float paintedFraction(const Body& body);

enum class GroupBy { Normals, Islands, Mask, Paint };
// New polygroups: faces meeting at less than `angleDeg` (Normals), connected pieces
// (Islands), the masked faces as one new group (Mask), or regions of one colour (Paint).
// Returns how many groups there are.
int groupBody(Body& body, GroupBy how, float angleDeg = 20.0f);
void clearGroups(Body& body);
// The polygroup of the face nearest a world point (-1: no groups).
int groupNear(const Body& body, const Vec3& worldPoint);
// The groups and how many faces each has.
std::vector<std::pair<std::uint32_t, std::size_t>> groupSizes(const Body& body);
// Masks exactly the faces of `group` (the rest unmasked; `invert`: the other way round).
bool maskGroup(Body& body, std::uint32_t group, bool invert = false);

// --- sculpt layers -------------------------------------------------------------------------
// Each layer holds how far strokes moved each vertex while it recorded; its strength
// scales that in or out of the surface. False (why) for a name that isn't there (or is).
bool layerAdd(Body& body, const std::string& name, bool record, std::string* why = nullptr);
bool layerRecord(Body& body, const std::string& name, std::string* why = nullptr); // "": record into none
bool layerStrength(Body& body, const std::string& name, float strength, std::string* why = nullptr);
bool layerDelete(Body& body, const std::string& name, std::string* why = nullptr);  // its offsets taken out
bool layerBake(Body& body, const std::string& name, std::string* why = nullptr);    // the shape kept, the layer dropped ("": all)


// A welded, editable copy of a body's surface (world coordinates).
class SculptMesh {
public:
    SculptMesh() = default;
    explicit SculptMesh(const Body& body);

    std::size_t vertexCount() const noexcept { return m_p.size(); }
    std::size_t triangleCount() const noexcept { return m_t.size(); }
    const std::vector<Vec3>& positions() const noexcept { return m_p; }
    const std::vector<std::array<std::uint32_t, 3>>& triangles() const noexcept { return m_t; }
    // Edges with only one triangle after welding (0: a closed solid).
    std::size_t openEdges() const;
    // x of the symmetry plane (centre of the body when the copy was made).
    float mirrorX() const noexcept { return m_mirrorX; }
    bool empty() const noexcept { return m_t.empty(); }

    // Closest hit of a ray (world), with the face normal.
    std::optional<Vec3> raycast(const Ray& ray, Vec3* normal = nullptr) const;
    // Closest surface point to q, with the face normal.
    Vec3 closestPoint(const Vec3& q, Vec3* normal = nullptr) const;

    // One brush dab centred at `centre` (on the surface), heading along `dir` (the
    // stroke's step since the last dab: Snake Hook, Nudge and Clay Strips use it).
    // Not for Grab.
    void dab(const Vec3& centre, const SculptSettings& s, const Vec3& dir = {});
    // Starts a stroke: the Layer brush builds up to its height once per stroke.
    void beginStroke() {
        m_layer.clear();
        if (m_record >= 0) m_start = m_p;
    }
    // Grab: pick up the surface around `centre`, then move it by `delta` from where it was.
    void beginGrab(const Vec3& centre, const SculptSettings& s);
    void grabTo(const Vec3& delta);

    // Loop subdivision of the whole surface (false past `maxTriangles`).
    bool loopSubdivide(std::size_t maxTriangles);
    // Collapses the shortest edges until about `target` triangles are left.
    bool decimateTo(std::size_t target);

    // Masks (per vertex, 0..1; empty: none).
    const std::vector<float>& mask() const noexcept { return m_mask; }
    void maskEdit(MaskOp op);
    // Polypaint (per vertex, packed as Vertex::color; empty: none) and polygroups (per
    // triangle; empty: none). They follow the surface through dynamic topology.
    const std::vector<std::uint32_t>& colors() const noexcept { return m_color; }
    const std::vector<std::uint32_t>& groups() const noexcept { return m_group; }
    // Sculpt layers: a stroke's movement since it began is added to the recording layer
    // (scaled by its strength) when the stroke is committed.
    void commitLayers();

    // A stroke (dabs along the points, snapped to the surface) or a grab from
    // points[0] by `grabDelta`, run on the part of the surface it can reach.
    void stroke(const std::vector<Vec3>& points, const SculptSettings& s, const Vec3& grabDelta = {});

    // Splits every edge longer than `detail` (whole mesh), up to `maxTriangles`.
    void refineAll(float detail, std::size_t maxTriangles = 600000);
    // Collapses edges shorter than `minEdge` (within `radius` of `centre`, or
    // everywhere when radius <= 0), keeping the surface closed and unfolded.
    std::size_t collapseShort(float minEdge, const Vec3& centre = {}, float radius = 0.0f, bool keepShape = true);

    // Writes back: kind "mesh", normals smoothed across edges turning less than
    // `smoothAngle` degrees, feature edges if asked.
    void writeTo(Body& body, bool featureEdges = true, float smoothAngle = 40.0f) const;

private:
    void refineRegion(const Vec3& centre, float radius, float detail);
    void applyDab(const Vec3& centre, const SculptSettings& s, const Vec3& dir);
    // The brush's mirrored and radial copies: each a centre and how it maps directions.
    struct Image {
        Vec3 centre;
        Vec3 axes[3]; // the image of x, y and z
        Vec3 apply(const Vec3& d) const { return axes[0] * d.x + axes[1] * d.y + axes[2] * d.z; }
    };
    std::vector<Image> images(const Vec3& centre, const SculptSettings& s, float R) const;
    // Per-vertex extras that follow the vertices through splits, collapses and windows.
    void addVertexExtras(std::uint32_t a, std::uint32_t b);
    float held(std::uint32_t v) const noexcept { return m_mask.empty() ? 0.0f : m_mask[v]; }
    void updateNormals();
    void flipCaps();
    std::size_t collapseLocal(std::vector<std::uint32_t> local, float minEdge, const Vec3& centre, float radius, bool keepShape);
    std::vector<std::uint32_t> trianglesNear(const Vec3& centre, float reach) const;
    void runStroke(const std::vector<Vec3>& path, const SculptSettings& s, const Vec3& grabDelta);
    void dropUnused();
    // The triangles that meet any of the boxes, as a mesh of their own whose cut
    // border is pinned; stitch() puts the edited window back.
    SculptMesh window(const std::vector<AABB>& boxes, std::vector<std::uint32_t>& taken, float* longestCut) const;
    void stitch(const SculptMesh& w, const std::vector<std::uint32_t>& taken);
    bool pinned(std::uint32_t v) const noexcept { return !m_pinned.empty() && m_pinned[v]; }
    bool splitEdges(const std::vector<std::uint64_t>& edges, std::size_t maxTriangles);

    std::vector<Vec3> m_p;                         // positions
    std::vector<Vec3> m_n;                         // smooth vertex normals
    std::vector<std::array<std::uint32_t, 3>> m_t; // triangles
    float m_mirrorX = 0.0f;
    Vec3 m_centre{};     // symmetry centre (the body's, when the copy was made)
    float m_size = 1.0f; // bounding diagonal
    std::vector<float> m_mask;  // per vertex, or empty
    std::vector<float> m_layer; // Layer brush: how far each vertex has risen this stroke
    std::vector<std::uint32_t> m_color; // polypaint, per vertex, or empty
    std::vector<std::uint32_t> m_group; // polygroups, per triangle, or empty
    Vec3 m_base{0.72f, 0.75f, 0.80f};   // the body's own colour (unpainted)
    // Sculpt layers: names and strengths, each a per-vertex offset; the recording one,
    // and where each vertex was when the stroke began.
    std::vector<SculptLayer> m_layers; // offsets per welded vertex
    int m_record = -1;
    std::vector<Vec3> m_start;
    std::vector<std::uint32_t> m_ids; // per vertex (MeshData::ids), or empty; new vertices get 0xFFFFFFFF
    // A window on a larger mesh: where each vertex came from there (kFresh: made
    // here), and the vertices on the cut border, which nothing may move or split.
    std::vector<std::uint32_t> m_from;
    std::vector<char> m_pinned;
    std::unordered_set<std::uint64_t> m_cut; // edges shared with triangles outside the window
    // Grab in progress.
    struct GrabVert {
        std::uint32_t v;
        float w;
        Vec3 start;
        std::uint8_t image; // which copy of the brush took it
    };
    std::vector<GrabVert> m_grab;
    std::vector<Image> m_grabImages;
};

// Closest-point queries against a mesh that doesn't change while they run: a uniform
// grid of its triangles, searched outwards from the query until nothing nearer can
// be left. The mesh must outlive the index and stay as it was; one query at a time.
class SurfaceIndex {
public:
    explicit SurfaceIndex(const SculptMesh& mesh);
    Vec3 closestPoint(const Vec3& q, Vec3* normal = nullptr) const;

private:
    const SculptMesh& m_mesh;
    Vec3 m_min;
    float m_cell = 1.0f;
    int m_n[3] = {0, 0, 0};
    std::vector<std::uint32_t> m_start; // per cell, into m_items (CSR)
    std::vector<std::uint32_t> m_items; // triangle indices
    mutable std::vector<std::uint32_t> m_seen; // per triangle: the query that last tested it
    mutable std::uint32_t m_query = 0;
};

// Dabs every radius / 4 along the polyline through the points snapped to the surface,
// or a grab from points[0] moved by `grabDelta`. Writes the body back.
void sculptStroke(Body& body, const std::vector<Vec3>& points, const SculptSettings& s, const Vec3& grabDelta = {});
// The same on a welded copy of the body already made (it is changed too).
void sculptStroke(Body& body, SculptMesh& surface, const std::vector<Vec3>& points, const SculptSettings& s, const Vec3& grabDelta = {});

// Refines the whole body until no edge is longer than `detail` (for sculpting a coarse CAD mesh).
void sculptRefine(Body& body, float detail);
// Loop subdivision: each level splits every triangle in four and smooths the surface
// (corners and creases round off). False (unchanged) past `maxTriangles`.
bool subdivideBody(Body& body, int levels, std::size_t maxTriangles = 4000000, std::string* error = nullptr);
// Makes a body mirror-symmetric: the side of the centre plane (normal along `axis`,
// 0 X 1 Y 2 Z) that `positive` names is kept and mirrored onto the other.
bool symmetrizeBody(Body& body, int axis, bool positive, std::string* error = nullptr);
// Coarsens to about `targetTriangles` by collapsing the shortest edges (closed stays closed).
bool decimateBody(Body& body, std::size_t targetTriangles, std::string* error = nullptr);
// Coarsens a dense mesh by collapsing edges shorter than `edge` (closed stays closed).
void simplifyMesh(MeshData& mesh, float edge);

// ---------------------------------------------------------------------------
// Booleans

enum class BooleanOp { Union, Subtract, Intersect };

const char* booleanOpKey(BooleanOp op) noexcept;
std::optional<BooleanOp> booleanOpFromKey(const std::string& key) noexcept;

// a = a (op) b, both closed meshes in the same coordinates. False (a unchanged) when
// the result would be empty for Union / Subtract or the inputs have no triangles.
bool meshBoolean(MeshData& a, const MeshData& b, BooleanOp op);
// The same, cleaning the inputs first and nudging the tool a hair when faces lie on each
// other (blocks side by side), keeping the result with the fewest holes.
bool meshBooleanRobust(MeshData& a, const MeshData& b, BooleanOp op);

// Cutters and tools (closed, outward-facing meshes).
MeshData cutterCylinder(const Vec3& from, const Vec3& to, float radius, int segments = 48);
MeshData cutterCone(const Vec3& from, const Vec3& to, float r0, float r1, int segments = 48);
// Box with half sizes `half` along the orthonormal axes u, v, w, centred at `centre`.
MeshData cutterBox(const Vec3& centre, const Vec3& u, const Vec3& v, const Vec3& w, const Vec3& half);
MeshData cutterSphere(const Vec3& centre, float radius, int segments = 40, int rings = 20);
// Round tube along a polyline with hemispherical ends.
MeshData cutterTube(const std::vector<Vec3>& path, float radius, int segments = 24);

enum class PocketShape { Box, Round, Ball };
enum class BossShape { Cylinder, Box, Ball };

// Operations on a body, in world coordinates. Each returns false with a reason in
// `error` when it would not change the body.
//   `normal` is the outward surface normal at `at` (the tool goes in against it).
bool drillHole(Body& body, const Vec3& at, const Vec3& normal, float diameter, float depth /*0 = through*/,
               std::string* error = nullptr);
// A hole wizard hole (counterbored, countersunk, tapped...; see HoleWizard.hpp).
bool drillHole(Body& body, const Vec3& at, const Vec3& normal, const HoleSpec& hole, std::string* error = nullptr);
bool cutPocket(Body& body, const Vec3& at, const Vec3& normal, PocketShape shape, float width, float length,
               float depth, float angleDeg = 0.0f, std::string* error = nullptr);
// A groove of the given width following `path` (points on the surface, normals outward).
bool cutChannel(Body& body, const std::vector<Vec3>& path, const std::vector<Vec3>& normals, float width,
                float depth, std::string* error = nullptr);
// Cuts away everything on the side `normal` points to.
bool trimByPlane(Body& body, const Vec3& point, const Vec3& normal, std::string* error = nullptr);
// Volume (mm^3) of the part of the mesh (world) on the side `normal` points to.
double volumeBeyondPlane(const MeshData& worldMesh, const Vec3& point, const Vec3& normal);
bool addBoss(Body& body, const Vec3& at, const Vec3& normal, BossShape shape, float diameter, float height,
             std::string* error = nullptr);
// a = a (op) b (both bodies placed by their offsets).
bool bodyBoolean(Body& a, const Body& b, BooleanOp op, std::string* error = nullptr);
// The same with the exact surfaces of a mesh tool `b` (in b's frame), so `a` keeps exact
// geometry (see ExactMesh).
bool bodyBoolean(Body& a, const Body& b, BooleanOp op, std::string* error, const std::vector<BSurface>& toolSurfaces);

// The body's mesh in world coordinates.
MeshData worldMesh(const Body& body);
// Signed volume (mm^3) of a closed mesh.
double meshVolume(const MeshData& m);
// Edges not shared by exactly two triangles (0: closed, ready for 3D printing).
std::size_t openEdgeCount(const MeshData& m);

// Generalised winding number of a mesh (about 1 inside, 0 outside, robust for open
// and overlapping shells): exact near the query point, clustered far from it.
class WindingNumber {
public:
    explicit WindingNumber(const MeshData& mesh);
    ~WindingNumber();
    WindingNumber(const WindingNumber&) = delete;
    WindingNumber& operator=(const WindingNumber&) = delete;
    double operator()(const Vec3& p) const;

private:
    struct Impl;
    Impl* m_impl;
};

} // namespace sicdm

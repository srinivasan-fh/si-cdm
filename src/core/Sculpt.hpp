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
#include <optional>
#include <string>
#include <vector>

namespace sicdm {

struct Body;
class Scene;

enum class SculptBrush { Draw, Clay, Carve, Inflate, Grab, Pinch, Crease, Smooth, Flatten, Scrape };

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
    bool refine = true;      // dynamic topology: split long edges under the brush
    float detail = 0.0f;     // mm, longest edge left under the brush; 0 = radius / 4
};

// A welded, editable copy of a body's surface (world coordinates).
class SculptMesh {
public:
    SculptMesh() = default;
    explicit SculptMesh(const Body& body);

    std::size_t vertexCount() const noexcept { return m_p.size(); }
    std::size_t triangleCount() const noexcept { return m_t.size(); }
    const std::vector<Vec3>& positions() const noexcept { return m_p; }
    // x of the symmetry plane (centre of the body when the copy was made).
    float mirrorX() const noexcept { return m_mirrorX; }
    bool empty() const noexcept { return m_t.empty(); }

    // Closest hit of a ray (world), with the face normal.
    std::optional<Vec3> raycast(const Ray& ray, Vec3* normal = nullptr) const;
    // Closest surface point to q, with the face normal.
    Vec3 closestPoint(const Vec3& q, Vec3* normal = nullptr) const;

    // One brush dab centred at `centre` (on the surface). Not for Grab.
    void dab(const Vec3& centre, const SculptSettings& s);
    // Grab: pick up the surface around `centre`, then move it by `delta` from where it was.
    void beginGrab(const Vec3& centre, const SculptSettings& s);
    void grabTo(const Vec3& delta);

    // Splits every edge longer than `detail` (whole mesh), up to `maxTriangles`.
    void refineAll(float detail, std::size_t maxTriangles = 600000);
    // Collapses edges shorter than `minEdge` (within `radius` of `centre`, or
    // everywhere when radius <= 0), keeping the surface closed and unfolded.
    std::size_t collapseShort(float minEdge, const Vec3& centre = {}, float radius = 0.0f);

    // Writes back: kind "mesh", normals smoothed across edges turning less than
    // `smoothAngle` degrees, feature edges if asked.
    void writeTo(Body& body, bool featureEdges = true, float smoothAngle = 40.0f) const;

private:
    void refineRegion(const Vec3& centre, float radius, float detail);
    void applyDab(const Vec3& centre, const SculptSettings& s);
    void updateNormals();
    bool splitEdges(const std::vector<std::uint64_t>& edges, std::size_t maxTriangles);

    std::vector<Vec3> m_p;                         // positions
    std::vector<Vec3> m_n;                         // smooth vertex normals
    std::vector<std::array<std::uint32_t, 3>> m_t; // triangles
    float m_mirrorX = 0.0f;
    float m_size = 1.0f; // bounding diagonal
    // Grab in progress.
    struct GrabVert {
        std::uint32_t v;
        float w;
        Vec3 start;
        bool mirrored;
    };
    std::vector<GrabVert> m_grab;
};

// Dabs along a polyline every radius / 4 (each point snapped to the surface), or a
// grab from points[0] moved by `grabDelta`. Writes the body back.
void sculptStroke(Body& body, const std::vector<Vec3>& points, const SculptSettings& s, const Vec3& grabDelta = {});

// Refines the whole body until no edge is longer than `detail` (for sculpting a coarse CAD mesh).
void sculptRefine(Body& body, float detail);
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

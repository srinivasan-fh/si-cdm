#pragma once

#include "core/Math.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace sicdm {

struct Vertex {
    Vec3 position;
    Vec3 normal;
    float mask = 0.0f; // sculpt mask: 1 holds the surface still under the brushes
    // Polypaint: red, green, blue and alpha bytes, red lowest (alpha 0: unpainted, the
    // body's own colour shows).
    std::uint32_t color = 0;
};

// Packs and unpacks a polypaint colour (components 0..1).
std::uint32_t packColor(const Vec3& rgb) noexcept;
Vec3 unpackColor(std::uint32_t c) noexcept;
inline bool painted(std::uint32_t c) noexcept { return (c >> 24) != 0; }

// A sculpt layer: how far each vertex was moved while the layer recorded, applied at
// `strength` (0: off). The surface shown is the layers' sum over the shape beneath them.
struct SculptLayer {
    std::string name;
    float strength = 1.0f;
    float saved = 1.0f;          // the strength before it was turned off
    std::vector<Vec3> offset;    // per vertex
};

// Renderable tessellation of a body: shaded triangles plus the
// feature edges drawn on top (the classic "shaded with edges" CAD look).
struct MeshData {
    std::vector<Vertex> vertices;
    std::vector<std::uint32_t> indices;  // triangle list
    std::vector<Vec3> edgeVertices;      // line list, pairs of points
    // Polygroups: a group number per triangle (empty, or out of step with the
    // triangles after an edit that didn't keep them: none).
    std::vector<std::uint32_t> groups;
    bool hasGroups() const noexcept { return !groups.empty() && groups.size() * 3 == indices.size(); }
    // Sculpt layers (each a vertex's offset; out of step with the vertices after an edit
    // that didn't keep them: none), and the one strokes record into (-1: none).
    std::vector<SculptLayer> layers;
    int recordLayer = -1;
    // Who each vertex is (multiresolution levels match their vertices by it); empty: none.
    std::vector<std::uint32_t> ids;
    bool hasIds() const noexcept { return !ids.empty() && ids.size() == vertices.size(); }
    // Quad faces (a retopologised mesh): each quad's corners, its two triangles being the
    // indices 6q .. 6q+5; empty, or out of step with the triangles: none.
    std::vector<std::array<std::uint32_t, 4>> quads;
    bool hasQuads() const noexcept { return !quads.empty() && quads.size() * 6 == indices.size(); }
    // Texture coordinates, one per triangle corner (indices.size()): seams are corners of a
    // vertex with different coordinates. Empty, or out of step with the triangles: none.
    std::vector<Vec2> uvs;
    bool hasUvs() const noexcept { return !uvs.empty() && uvs.size() == indices.size(); }
    bool hasLayers() const noexcept {
        if (layers.empty()) return false;
        for (const SculptLayer& l : layers)
            if (l.offset.size() != vertices.size()) return false;
        return true;
    }

    AABB bounds() const noexcept;
    void transform(const Mat4& m);
    void append(const MeshData& other);

    // Rebuilds edgeVertices from the triangles: open boundaries plus edges
    // whose adjacent faces meet at more than `angleDeg` (for imported meshes).
    // Edges shorter than `shortLength` must turn more than `shortAngleDeg` (sculpted
    // detail stays clean while the rims of machined faces still show).
    void computeFeatureEdges(float angleDeg = 30.0f, float shortLength = 0.0f, float shortAngleDeg = 0.0f);
};

// A fast 64-bit key of a mesh's vertices and triangles (equal meshes, equal keys): tells
// a renderer or a picker whether a body's shape is the one it has a copy of.
std::uint64_t meshContentKey(const MeshData& m) noexcept;

// A coarser copy for drawing far away (level of detail): vertices within one `cell`
// (and facing roughly the same way) merge into one, and triangles that collapse go. It
// has no edge lines (draw the full shape's).
MeshData clusterMesh(const MeshData& m, float cell);

// Parametric description of a body, so projects can be saved and edited.
// Parameters per kind (all mm, body-local; Body::offset places it):
//   box, wedge: x0 y0 z0 x1 y1 z1     cylinder: r h      tube: R r h
//   cone: r0 r1 h    sphere: r (rests on z=0)    torus: R r (rests on z=0)
//   pyramid, prism: r h sides         mesh: none (triangles stored as-is)
//   extrude: see extrudeParams() (sketch plane, depth range, profile)
bool isPrimitiveKind(const std::string& kind);
// Returns an empty mesh for unknown kinds or bad parameter counts.
MeshData buildPrimitive(const std::string& kind, const std::vector<float>& params);

namespace primitives {

// Axis-aligned box spanning [min, max].
MeshData box(const Vec3& min, const Vec3& max);

// Cylinder along +Z, base centred at `base`.
MeshData cylinder(const Vec3& base, float radius, float height, int segments = 64);

// Tube (cylinder with a coaxial hole) along +Z.
MeshData tube(const Vec3& base, float outerRadius, float innerRadius, float height, int segments = 64);

// UV sphere.
MeshData sphere(const Vec3& center, float radius, int segments = 48, int rings = 24);

// Cone / frustum along +Z: `bottomRadius` at the base, `topRadius` (0 = apex) at the top.
MeshData cone(const Vec3& base, float bottomRadius, float topRadius, float height, int segments = 64);

// Torus around +Z: `majorRadius` to the tube centre, `minorRadius` of the tube.
MeshData torus(const Vec3& center, float majorRadius, float minorRadius, int segments = 64, int rings = 24);

// Right-angled wedge in [min, max]: full height at min.x, sloping to zero at max.x.
MeshData wedge(const Vec3& min, const Vec3& max);

// Regular pyramid along +Z with an n-sided base of circumradius `radius`.
MeshData pyramid(const Vec3& base, float radius, float height, int sides = 4);

// Regular n-sided prism (extruded polygon) along +Z.
MeshData prism(const Vec3& base, float radius, float height, int sides = 6);

} // namespace primitives

} // namespace sicdm

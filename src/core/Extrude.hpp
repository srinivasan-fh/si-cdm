#pragma once

// Extrude: closed 2D sketch profiles -> 3D solids along the sketch-plane normal.

#include "core/Geometry.hpp"
#include "core/Sketch.hpp"

#include <optional>
#include <string>
#include <vector>

namespace sicdm {

// A closed profile in sketch-plane coordinates.
struct Profile {
    enum class Kind { Polygon, Circle } kind = Kind::Polygon;
    std::vector<Vec2> points; // polygon: counter-clockwise, no repeated closing point
    Vec2 center{};            // circle
    float radius = 0.0f;      // circle
};

// Closed curve -> profile: rectangles, circles, closed simple polylines and splines,
// ellipses and slots (construction curves are none).
// Returns nullopt (with a reason) for open curves, points and self-intersections.
std::optional<Profile> profileFromCurve(const SketchCurve& curve, std::string* why = nullptr);
// The same for curve `index` of a sketch, and for an open curve, the closed loop it makes
// with the curves joined to it end to end (lines and arcs round a slot, say).
std::optional<Profile> profileFromSketch(const Sketch& sketch, std::size_t index, std::string* why = nullptr);

// Body parameters for an extrusion between depths w0 and w1 along the plane
// normal (w1 > w0). Layout (all mm):
//   [plane, w0, w1, 0, x0, y0, x1, y1, ...]   polygon
//   [plane, w0, w1, 1, cx, cy, r]             circle
std::vector<float> extrudeParams(const Profile& p, SketchPlane plane, float distance, bool symmetric);

// Mesh for extrudeParams(); empty if the parameters are invalid.
MeshData buildExtrusion(const std::vector<float>& params);

// Polygon helpers (exposed for tests).
float signedArea(const std::vector<Vec2>& poly);
bool isSimplePolygon(const std::vector<Vec2>& poly);
// Ear-clipping triangulation of a simple counter-clockwise polygon.
// Returns n-2 triangles as index triples, or empty on failure.
std::vector<std::uint32_t> triangulatePolygon(const std::vector<Vec2>& poly);

} // namespace sicdm

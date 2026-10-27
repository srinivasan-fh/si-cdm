#pragma once

// Surfacing: open surface bodies on exact B-spline surfaces (lofted through sections,
// filling four boundary curves, extruded or revolved from open curves, offset, blended
// between two surfaces), closed into solids by thickening or knitting, and checked with
// zebra stripes and curvature.

#include "core/Brep.hpp"

#include <memory>
#include <string>
#include <vector>

namespace sicdm {

struct Body;

// B-spline basis: the knot span holding u, and the degree+1 non-zero basis values there.
int bsplineSpan(int n, int degree, double u, const std::vector<double>& knots); // n: control points - 1
void bsplineBasis(int span, double u, int degree, const std::vector<double>& knots, std::vector<double>& N);

// A curve of the degree (lower when there are too few points) through the points, at
// chord-length parameters (or `params` when given, with `knots` to match).
BSplineCurve interpolateCurve(const std::vector<DVec3>& pts, int degree, const std::vector<double>* params = nullptr,
                              const std::vector<double>* knots = nullptr);
// The parameters and averaged knots interpolateCurve() would use for these points.
void interpolationParams(const std::vector<std::vector<DVec3>>& rows, int degree, std::vector<double>& params, std::vector<double>& knots);

// A surface through a grid of points: rows[i][j], i across the rows (u), j along them (v).
// Degrees up to 3 each way (lower with fewer rows or points).
bool interpolateSurface(const std::vector<std::vector<DVec3>>& rows, BSplineSurface& out, std::string* why = nullptr);

// The point and the unit normal (du x dv) of a B-spline surface.
DVec3 surfacePoint(const BSplineSurface& s, double u, double v);
DVec3 surfaceNormal(const BSplineSurface& s, double u, double v);

// A one-face open B-rep on the surface, bounded by its four boundary curves (two of them
// one seam where the surface closes on itself).
std::shared_ptr<Brep> surfaceBrep(const BSplineSurface& s);

// A surface body for the B-rep: its display mesh tessellated from it (exact geometry kept).
bool makeSurfaceBody(Body& body, const std::shared_ptr<Brep>& brep, std::string* why = nullptr);

// The B-spline surface of a surface body (one B-spline face), in world space. False for
// other bodies.
bool bodySurface(const Body& body, BSplineSurface& out);

// Points along a polyline at equal spacing (n of them; a closed one repeats the first at
// the end).
std::vector<DVec3> resample(const std::vector<DVec3>& line, int n, bool closed);

// Four curves meeting end to end, filled by a bilinearly blended (Coons) patch sampled on
// an n x n grid. False when they don't form a loop.
bool coonsGrid(std::vector<std::vector<DVec3>> sides, int n, std::vector<std::vector<DVec3>>& grid, std::string* why);

// A surface body made solid: offset `t` along its normals (both ways when `both`, half
// each), the edges walled. Mesh only.
bool thickenMesh(const MeshData& surface, float t, bool both, MeshData& solid, std::string* why);

// Meshes stitched along boundary edges whose ends are within `tol`: one mesh; `closed`
// says whether it encloses a volume (then turned outward).
MeshData knitMeshes(const std::vector<MeshData>& parts, float tol, bool& closed, std::size_t& openEdges);

// Curvature at each vertex of a mesh (per unit length): mean (H) and Gaussian (K).
void meshCurvature(const MeshData& m, std::vector<float>& mean, std::vector<float>& gauss);

} // namespace sicdm

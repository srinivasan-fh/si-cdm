#pragma once

// Exact geometry through mesh booleans. A cut or join works on triangles; the faces it
// leaves still lie on the exact surfaces of the bodies and tools that went in (a hole's
// cylinder, a box's planes, a fillet's torus). These rebuild a B-rep from the result:
// each triangle is matched to the surface it lies on, neighbours on the same surface
// become one face, and the edges between faces become lines, circles or (where two
// curved surfaces meet) polylines whose points lie on both surfaces.

#include "core/Brep.hpp"

#include <memory>
#include <string>
#include <vector>

namespace sicdm {

struct Body;

// The analytic surfaces (plane, cylinder, cone, sphere, torus) of a body's exact
// geometry, in the mesh's own frame; none for a body that isn't exact.
std::vector<BSurface> exactSurfaces(const Body& body);

// The surfaces of a solid of revolution: `profile` is (radius, distance along `axis`)
// pairs from `base`, as Sculpt's revolve() takes them.
std::vector<BSurface> revolveSurfaces(const Vec3& base, const Vec3& axis, const std::vector<std::pair<float, float>>& profile);

// Distance from p to an analytic surface (signed: + on the side its normal points) and
// the surface's normal there. False for a kind it can't measure.
bool surfaceDistance(const BSurface& s, const DVec3& p, double& distance, DVec3& normal);

struct ExactReport {
    std::size_t faces = 0, exactFaces = 0;  // faces, and those on a surface from the list
    std::size_t lines = 0, circles = 0, polylines = 0;
    double exactArea = 0;  // the share of the area on a surface from the list (0..1)
    std::string why; // when there is no B-rep
};

// A B-rep of a closed mesh whose faces lie on `surfaces` where they do (other triangles
// make flat faces of their own). Null when the mesh isn't closed, has more than
// `maxFaces` faces, or its B-rep doesn't tessellate back to the same solid.
std::shared_ptr<Brep> exactBrepFromMesh(const MeshData& mesh, const std::vector<BSurface>& surfaces, ExactReport* report = nullptr,
                                        std::size_t maxFaces = 4000);

// After a boolean left `body` with a new mesh: its exact geometry rebuilt from
// `surfaces` (the inputs' surfaces in the body's frame). False (the body then has none)
// when there are no surfaces or the rebuild fails.
bool keepExactGeometry(Body& body, const std::vector<BSurface>& surfaces);

} // namespace sicdm

#pragma once

// Curves onto surfaces: points carried onto a body along a direction (as a designer projects a
// character line from the side view onto the body side) or to their nearest point on it, and
// the distance from a point to it. A surface body's exact B-spline is used when it has one
// (the mesh finds the neighbourhood, Newton steps finish on the surface); other bodies use
// their triangles.

#include "core/Brep.hpp"
#include "core/Bvh.hpp"

#include <optional>
#include <string>
#include <vector>

namespace sicdm {

class SurfaceTarget {
public:
    // `mesh` in world space; `exact` the body's surface (world space) or null.
    SurfaceTarget(MeshData mesh, const BSplineSurface* exact);
    // Where the line through p along d meets it, nearest p (either way along the line).
    std::optional<DVec3> alongDirection(const DVec3& p, const DVec3& d) const;
    // Its point nearest p.
    DVec3 nearest(const DVec3& p) const;
    double distance(const DVec3& p) const { return dlength(nearest(p) - p); }
    bool exact() const noexcept { return m_hasExact; }

private:
    MeshData m_mesh;
    MeshBvh m_bvh;
    BSplineSurface m_exact;
    bool m_hasExact = false;
    DVec3 nearestOnMesh(const DVec3& p) const;
    bool refineNearest(const DVec3& p, DVec3& q) const;
    bool refineAlong(const DVec3& p, const DVec3& d, DVec3& q) const;
    void uvNear(const DVec3& q, double& u, double& v) const;
};

} // namespace sicdm

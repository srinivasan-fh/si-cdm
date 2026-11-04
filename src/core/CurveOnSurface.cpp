#include "core/CurveOnSurface.hpp"

#include "core/Surfacing.hpp"

#include <algorithm>
#include <cmath>

namespace sicdm {

namespace {

// The point of triangle abc nearest p (Ericson, Real-Time Collision Detection 5.1.5).
DVec3 nearestOnTriangle(const DVec3& p, const DVec3& a, const DVec3& b, const DVec3& c) {
    const DVec3 ab = b - a, ac = c - a, ap = p - a;
    const double d1 = ddot(ab, ap), d2 = ddot(ac, ap);
    if (d1 <= 0 && d2 <= 0) return a;
    const DVec3 bp = p - b;
    const double d3 = ddot(ab, bp), d4 = ddot(ac, bp);
    if (d3 >= 0 && d4 <= d3) return b;
    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) return a + ab * (d1 / (d1 - d3));
    const DVec3 cp = p - c;
    const double d5 = ddot(ab, cp), d6 = ddot(ac, cp);
    if (d6 >= 0 && d5 <= d6) return c;
    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) return a + ac * (d2 / (d2 - d6));
    const double va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    const double den = 1.0 / (va + vb + vc);
    return a + ab * (vb * den) + ac * (vc * den);
}

double uLo(const BSplineSurface& s) { return s.ku[static_cast<std::size_t>(s.du)]; }
double uHi(const BSplineSurface& s) { return s.ku[s.ku.size() - 1 - static_cast<std::size_t>(s.du)]; }
double vLo(const BSplineSurface& s) { return s.kv[static_cast<std::size_t>(s.dv)]; }
double vHi(const BSplineSurface& s) { return s.kv[s.kv.size() - 1 - static_cast<std::size_t>(s.dv)]; }

// Solves the 2x2 or 3x3 system A x = b (Cramer); false when it is singular.
bool solve2(double a, double b, double c, double d, double r0, double r1, double& x, double& y) {
    const double det = a * d - b * c;
    if (std::fabs(det) < 1e-300) return false;
    x = (r0 * d - b * r1) / det;
    y = (a * r1 - c * r0) / det;
    return true;
}

} // namespace

SurfaceTarget::SurfaceTarget(MeshData mesh, const BSplineSurface* exact) : m_mesh(std::move(mesh)) {
    m_bvh.build(m_mesh);
    if (exact && exact->nu > exact->du && exact->nv > exact->dv && !exact->ku.empty() && !exact->kv.empty()) {
        m_exact = *exact;
        m_hasExact = true;
    }
}

DVec3 SurfaceTarget::nearestOnMesh(const DVec3& p) const {
    DVec3 best = p;
    double bd = 1e300;
    const auto& vs = m_mesh.vertices;
    for (std::size_t i = 0; i + 2 < m_mesh.indices.size(); i += 3) {
        const DVec3 a(vs[m_mesh.indices[i]].position), b(vs[m_mesh.indices[i + 1]].position), c(vs[m_mesh.indices[i + 2]].position);
        const DVec3 q = nearestOnTriangle(p, a, b, c);
        const double d = ddot(q - p, q - p);
        if (d < bd) bd = d, best = q;
    }
    return best;
}

void SurfaceTarget::uvNear(const DVec3& q, double& u, double& v) const {
    const BSplineSurface& s = m_exact;
    double bd = 1e300;
    const int n = 32;
    for (int i = 0; i <= n; ++i)
        for (int j = 0; j <= n; ++j) {
            const double uu = uLo(s) + (uHi(s) - uLo(s)) * i / n, vv = vLo(s) + (vHi(s) - vLo(s)) * j / n;
            const DVec3 e = surfacePoint(s, uu, vv) - q;
            if (const double d = ddot(e, e); d < bd) bd = d, u = uu, v = vv;
        }
}

bool SurfaceTarget::refineNearest(const DVec3& p, DVec3& q) const {
    const BSplineSurface& s = m_exact;
    double u = 0, v = 0;
    uvNear(q, u, v);
    const double hu = (uHi(s) - uLo(s)) * 1e-5, hv = (vHi(s) - vLo(s)) * 1e-5;
    for (int it = 0; it < 30; ++it) {
        const DVec3 S = surfacePoint(s, u, v);
        const DVec3 Su = (surfacePoint(s, std::min(u + hu, uHi(s)), v) - surfacePoint(s, std::max(u - hu, uLo(s)), v)) / (std::min(u + hu, uHi(s)) - std::max(u - hu, uLo(s)));
        const DVec3 Sv = (surfacePoint(s, u, std::min(v + hv, vHi(s))) - surfacePoint(s, u, std::max(v - hv, vLo(s)))) / (std::min(v + hv, vHi(s)) - std::max(v - hv, vLo(s)));
        const DVec3 e = S - p;
        // Gauss-Newton on (S - p) . Su = 0, (S - p) . Sv = 0.
        double du = 0, dv = 0;
        if (!solve2(ddot(Su, Su), ddot(Su, Sv), ddot(Su, Sv), ddot(Sv, Sv), -ddot(e, Su), -ddot(e, Sv), du, dv)) break;
        u = std::clamp(u + du, uLo(s), uHi(s));
        v = std::clamp(v + dv, vLo(s), vHi(s));
        if (std::fabs(du) < 1e-12 * (uHi(s) - uLo(s)) && std::fabs(dv) < 1e-12 * (vHi(s) - vLo(s))) break;
    }
    const DVec3 r = surfacePoint(s, u, v);
    if (dlength(r - p) > dlength(q - p) + 1e-6) return false; // went astray: keep the mesh's
    q = r;
    return true;
}

bool SurfaceTarget::refineAlong(const DVec3& p, const DVec3& d, DVec3& q) const {
    const BSplineSurface& s = m_exact;
    double u = 0, v = 0;
    uvNear(q, u, v);
    double t = ddot(q - p, d);
    const double hu = (uHi(s) - uLo(s)) * 1e-5, hv = (vHi(s) - vLo(s)) * 1e-5;
    for (int it = 0; it < 30; ++it) {
        const DVec3 S = surfacePoint(s, u, v);
        const DVec3 Su = (surfacePoint(s, std::min(u + hu, uHi(s)), v) - surfacePoint(s, std::max(u - hu, uLo(s)), v)) / (std::min(u + hu, uHi(s)) - std::max(u - hu, uLo(s)));
        const DVec3 Sv = (surfacePoint(s, u, std::min(v + hv, vHi(s))) - surfacePoint(s, u, std::max(v - hv, vLo(s)))) / (std::min(v + hv, vHi(s)) - std::max(v - hv, vLo(s)));
        // S(u,v) - (p + t d) = 0: three equations in u, v, t.
        const DVec3 F = S - (p + d * t);
        const DVec3 c0 = Su, c1 = Sv, c2 = -d;
        const double det = ddot(c0, dcross(c1, c2));
        if (std::fabs(det) < 1e-300) return false;
        const DVec3 r = -F;
        const double du = ddot(r, dcross(c1, c2)) / det, dv = ddot(c0, dcross(r, c2)) / det, dt = ddot(c0, dcross(c1, r)) / det;
        u = std::clamp(u + du, uLo(s), uHi(s));
        v = std::clamp(v + dv, vLo(s), vHi(s));
        t += dt;
        if (dlength(F) < 1e-10) break;
    }
    const DVec3 r = surfacePoint(s, u, v);
    if (dlength(r - (p + d * t)) > 1e-6) return false; // no exact meeting point near the mesh's
    q = r;
    return true;
}

std::optional<DVec3> SurfaceTarget::alongDirection(const DVec3& p, const DVec3& dir) const {
    const DVec3 d = dnormalize(dir);
    if (dlength(d) < 0.5 || m_bvh.empty()) return std::nullopt;
    std::optional<DVec3> best;
    for (const double sgn : {1.0, -1.0}) {
        Ray r{p.f(), (d * sgn).f()};
        float t = 1e30f;
        if (m_bvh.raycast(r, t)) {
            const DVec3 q = p + d * (sgn * t);
            if (!best || dlength(q - p) < dlength(*best - p)) best = q;
        }
    }
    if (best && m_hasExact) refineAlong(p, d, *best);
    return best;
}

DVec3 SurfaceTarget::nearest(const DVec3& p) const {
    DVec3 q = nearestOnMesh(p);
    if (m_hasExact) refineNearest(p, q);
    return q;
}

} // namespace sicdm

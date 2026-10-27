#include "core/ExactMesh.hpp"

#include "core/Scene.hpp"
#include "core/Sculpt.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <set>
#include <tuple>

namespace sicdm {

namespace {

bool analytic(const BSurface& s) {
    return s.kind == BSurfaceKind::Plane || s.kind == BSurfaceKind::Cylinder || s.kind == BSurfaceKind::Cone || s.kind == BSurfaceKind::Sphere ||
           s.kind == BSurfaceKind::Torus;
}

// The point of s nearest p.
DVec3 projectOnto(const BSurface& s, const DVec3& p) {
    double d;
    DVec3 n;
    return surfaceDistance(s, p, d, n) ? p - n * d : p;
}

// How far a mesh point may sit off a surface and still be on it: tessellated round
// surfaces have their vertices on the surface, but points a boolean puts inside a facet
// are off it by up to the facet's sag.
double toleranceFor(const BSurface& s, double tight) {
    switch (s.kind) {
    case BSurfaceKind::Plane: return tight;
    case BSurfaceKind::Cylinder:
    case BSurfaceKind::Sphere: return std::max(tight * 10, 0.015 * s.r1);
    case BSurfaceKind::Cone: return std::max(tight * 10, 0.015 * std::max(s.r1, 1.0));
    case BSurfaceKind::Torus: return std::max(tight * 10, 0.015 * s.r2);
    default: return tight;
    }
}

// Closest point to p on the circle (centre c, axis z, radius r).
DVec3 onCircle(const DVec3& p, const DVec3& c, const DVec3& z, double r) {
    DVec3 q = p - c;
    q = q - z * ddot(q, z);
    const double l = dlength(q);
    if (l < 1e-15) return p;
    return c + q * (r / l);
}

// A circle through the points (least squares in their best plane): centre, axis, radius,
// and the largest distance of a point from it.
bool fitCircle(const std::vector<DVec3>& pts, DVec3& c, DVec3& z, double& r, double& worst) {
    if (pts.size() < 3) return false;
    DVec3 mean{0, 0, 0};
    for (const DVec3& p : pts) mean += p;
    mean = mean / static_cast<double>(pts.size());
    // The plane: the largest cross product of spans from the mean.
    DVec3 n{0, 0, 0};
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) n += dcross(pts[i] - mean, pts[i + 1] - mean);
    if (dlength(n) < 1e-18) return false;
    z = dnormalize(n);
    const DVec3 x = dnormalize(dlength(dcross(z, DVec3{1, 0, 0})) > 0.1 ? dcross(z, DVec3{1, 0, 0}) : dcross(z, DVec3{0, 1, 0}));
    const DVec3 y = dcross(z, x);
    // Kasa fit: x^2 + y^2 + D x + E y + F = 0.
    double s[3][4] = {};
    for (const DVec3& p : pts) {
        const double u = ddot(p - mean, x), v = ddot(p - mean, y), w = -(u * u + v * v);
        const double row[3] = {u, v, 1};
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) s[i][j] += row[i] * row[j];
            s[i][3] += row[i] * w;
        }
    }
    for (int i = 0; i < 3; ++i) { // Gauss-Jordan with partial pivoting
        int piv = i;
        for (int k = i + 1; k < 3; ++k)
            if (std::fabs(s[k][i]) > std::fabs(s[piv][i])) piv = k;
        if (std::fabs(s[piv][i]) < 1e-30) return false;
        for (int j = 0; j < 4; ++j) std::swap(s[i][j], s[piv][j]);
        for (int k = 0; k < 3; ++k) {
            if (k == i) continue;
            const double m = s[k][i] / s[i][i];
            for (int j = i; j < 4; ++j) s[k][j] -= m * s[i][j];
        }
    }
    const double D = s[0][3] / s[0][0], E = s[1][3] / s[1][1], F = s[2][3] / s[2][2];
    const double cu = -D / 2, cv = -E / 2, rr = cu * cu + cv * cv - F;
    if (!(rr > 0)) return false;
    r = std::sqrt(rr);
    c = mean + x * cu + y * cv;
    worst = 0;
    for (const DVec3& p : pts) worst = std::max(worst, dlength(p - onCircle(p, c, z, r)));
    for (const DVec3& p : pts) worst = std::max(worst, std::fabs(ddot(p - c, z)));
    return true;
}

} // namespace

bool surfaceDistance(const BSurface& s, const DVec3& p, double& d, DVec3& n) {
    const DVec3 q = p - s.f.o;
    switch (s.kind) {
    case BSurfaceKind::Plane:
        n = s.f.z;
        d = ddot(q, s.f.z);
        return true;
    case BSurfaceKind::Cylinder:
    case BSurfaceKind::Cone:
    case BSurfaceKind::Torus: {
        const double h = ddot(q, s.f.z);
        const DVec3 rad = q - s.f.z * h;
        const double rho = dlength(rad);
        if (rho < 1e-12) return false;
        const DVec3 rd = rad / rho;
        if (s.kind == BSurfaceKind::Cylinder) {
            n = rd;
            d = rho - s.r1;
        } else if (s.kind == BSurfaceKind::Cone) {
            const double c = std::cos(s.r2), sn = std::sin(s.r2);
            n = rd * c - s.f.z * sn;
            d = (rho - (s.r1 + h * std::tan(s.r2))) * c;
        } else {
            const DVec3 w = rd * (rho - s.r1) + s.f.z * h;
            const double lw = dlength(w);
            if (lw < 1e-12) return false;
            n = w / lw;
            d = lw - s.r2;
        }
        return true;
    }
    case BSurfaceKind::Sphere: {
        const double l = dlength(q);
        if (l < 1e-12) return false;
        n = q / l;
        d = l - s.r1;
        return true;
    }
    default: return false;
    }
}

std::vector<BSurface> exactSurfaces(const Body& body) {
    std::vector<BSurface> out;
    if (!body.exact()) return out;
    for (const BSurface& s : body.brep->surfaces)
        if (analytic(s)) out.push_back(s);
    return out;
}

std::vector<BSurface> revolveSurfaces(const Vec3& base, const Vec3& axis, const std::vector<std::pair<float, float>>& profile) {
    std::vector<BSurface> out;
    const DVec3 o(base), z = dnormalize(DVec3(axis));
    if (dlength(z) == 0) return out;
    const DVec3 ref = std::fabs(z.z) < 0.9 ? DVec3{0, 0, 1} : DVec3{1, 0, 0};
    for (std::size_t i = 0; i + 1 < profile.size(); ++i) {
        const double r0 = profile[i].first, a0 = profile[i].second, r1 = profile[i + 1].first, a1 = profile[i + 1].second;
        BSurface s;
        if (std::fabs(r0 - r1) < 1e-7 && r0 > 0) {
            s.kind = BSurfaceKind::Cylinder;
            s.f = Frame::make(o, z, ref);
            s.r1 = r0;
        } else if (std::fabs(a0 - a1) < 1e-7) {
            if (r0 <= 0 && r1 <= 0) continue;
            s.kind = BSurfaceKind::Plane;
            s.f = Frame::make(o + z * a0, z, ref);
        } else {
            s.kind = BSurfaceKind::Cone;
            s.f = Frame::make(o + z * a0, z, ref);
            s.r1 = r0;
            s.r2 = std::atan((r1 - r0) / (a1 - a0));
        }
        out.push_back(s);
    }
    return out;
}

std::shared_ptr<Brep> exactBrepFromMesh(const MeshData& mesh, const std::vector<BSurface>& given, ExactReport* report, std::size_t maxFaces) {
    ExactReport scratch;
    ExactReport& rep = report ? *report : scratch;
    rep = {};
    auto fail = [&](const std::string& why) {
        rep.why = why;
        return std::shared_ptr<Brep>();
    };
    // Welded vertices and triangles.
    std::map<std::tuple<float, float, float>, int> weld;
    std::vector<DVec3> P;
    std::vector<int> vid(mesh.vertices.size());
    for (std::size_t i = 0; i < mesh.vertices.size(); ++i) {
        const Vec3& p = mesh.vertices[i].position;
        const auto [it, added] = weld.try_emplace({p.x + 0.0f, p.y + 0.0f, p.z + 0.0f}, static_cast<int>(P.size()));
        if (added) P.push_back(DVec3(p));
        vid[i] = it->second;
    }
    std::vector<std::array<int, 3>> T;
    std::vector<DVec3> N; // zero for a sliver (its corners in a line): it joins a neighbour's face
    for (std::size_t k = 0; k + 2 < mesh.indices.size(); k += 3) {
        const std::array<int, 3> t{vid[mesh.indices[k]], vid[mesh.indices[k + 1]], vid[mesh.indices[k + 2]]};
        if (t[0] == t[1] || t[1] == t[2] || t[0] == t[2]) continue;
        const DVec3 n = dcross(P[static_cast<std::size_t>(t[1])] - P[static_cast<std::size_t>(t[0])], P[static_cast<std::size_t>(t[2])] - P[static_cast<std::size_t>(t[0])]);
        T.push_back(t);
        N.push_back(dlength(n) < 1e-24 ? DVec3{0, 0, 0} : dnormalize(n));
    }
    if (T.empty()) return fail("no triangles");
    std::map<std::pair<int, int>, std::vector<int>> edgeTris;
    for (int i = 0; i < static_cast<int>(T.size()); ++i)
        for (int e = 0; e < 3; ++e) {
            const int a = T[static_cast<std::size_t>(i)][static_cast<std::size_t>(e)], c = T[static_cast<std::size_t>(i)][static_cast<std::size_t>((e + 1) % 3)];
            edgeTris[{std::min(a, c), std::max(a, c)}].push_back(i);
        }
    for (const auto& [e, ts] : edgeTris)
        if (ts.size() != 2) return fail("the mesh is not closed");
    DVec3 lo = P[0], hi = P[0];
    for (const DVec3& p : P) lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)}, hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
    const double size = std::max(dlength(hi - lo), 1.0);
    const double tight = 1e-5 * size;

    // Each triangle on the surface it fits best (all three corners near it, facing along).
    std::vector<BSurface> S;
    for (const BSurface& s : given)
        if (analytic(s)) S.push_back(s);
    const std::size_t catalogue = S.size();
    if (static_cast<double>(T.size()) * static_cast<double>(catalogue) > 6e7) return fail("too large to match surfaces quickly");
    std::vector<int> surf(T.size(), -1);
    std::vector<char> sense(T.size(), 1);
    for (std::size_t t = 0; t < T.size(); ++t) {
        if (dlength(N[t]) == 0) continue;
        double best = 1e300;
        for (std::size_t k = 0; k < catalogue; ++k) {
            const double tol = toleranceFor(S[k], tight);
            double worst = 0;
            bool ok = true;
            for (int c = 0; c < 3 && ok; ++c) {
                double d;
                DVec3 n;
                if (!surfaceDistance(S[k], P[static_cast<std::size_t>(T[t][static_cast<std::size_t>(c)])], d, n) || std::fabs(d) > tol) ok = false;
                else worst = std::max(worst, std::fabs(d) / tol);
            }
            if (!ok) continue;
            const DVec3 mid = (P[static_cast<std::size_t>(T[t][0])] + P[static_cast<std::size_t>(T[t][1])] + P[static_cast<std::size_t>(T[t][2])]) / 3.0;
            double d;
            DVec3 n;
            if (!surfaceDistance(S[k], mid, d, n)) continue;
            const double along = ddot(n, N[t]);
            if (std::fabs(along) < 0.75) continue;
            const double score = worst + (1 - std::fabs(along));
            if (score < best) best = score, surf[t] = static_cast<int>(k), sense[t] = along > 0;
        }
    }
    {
        double all = 0, on = 0;
        for (std::size_t t = 0; t < T.size(); ++t) {
            const double a = 0.5 * dlength(dcross(P[static_cast<std::size_t>(T[t][1])] - P[static_cast<std::size_t>(T[t][0])],
                                                  P[static_cast<std::size_t>(T[t][2])] - P[static_cast<std::size_t>(T[t][0])]));
            all += a;
            if (surf[t] >= 0) on += a;
        }
        rep.exactArea = all > 0 ? on / all : 0;
    }
    // Facets a tessellation leaves where two surfaces touch (a small ledge where a round
    // meets a wall) lie on a neighbour's surface but face another way: they join it.
    for (bool changed = true; changed;) {
        changed = false;
        for (std::size_t t = 0; t < T.size(); ++t) {
            if (surf[t] >= 0 || dlength(N[t]) == 0) continue;
            double best = 1e300;
            for (int e = 0; e < 3; ++e) {
                const int a = T[t][static_cast<std::size_t>(e)], c = T[t][static_cast<std::size_t>((e + 1) % 3)];
                for (const int o2 : edgeTris[{std::min(a, c), std::max(a, c)}]) {
                    const int k = surf[static_cast<std::size_t>(o2)];
                    if (static_cast<std::size_t>(o2) == t || k < 0 || k >= static_cast<int>(catalogue)) continue;
                    double worst = 0;
                    for (int q = 0; q < 3 && worst <= 1; ++q) {
                        double d;
                        DVec3 n;
                        worst = surfaceDistance(S[static_cast<std::size_t>(k)], P[static_cast<std::size_t>(T[t][static_cast<std::size_t>(q)])], d, n)
                                    ? std::max(worst, std::fabs(d) / toleranceFor(S[static_cast<std::size_t>(k)], tight))
                                    : 2.0;
                    }
                    if (worst <= 1 && worst < best) best = worst, surf[t] = k, sense[t] = sense[static_cast<std::size_t>(o2)];
                }
            }
            changed = changed || surf[t] >= 0;
        }
    }
    // The rest: flat faces of their own (coplanar neighbours together).
    for (std::size_t s0 = 0; s0 < T.size(); ++s0) {
        if (surf[s0] >= 0 || dlength(N[s0]) == 0) continue;
        BSurface pl;
        pl.kind = BSurfaceKind::Plane;
        const DVec3 o = P[static_cast<std::size_t>(T[s0][0])];
        pl.f = Frame::make(o, N[s0], P[static_cast<std::size_t>(T[s0][1])] - o);
        S.push_back(pl);
        const int id = static_cast<int>(S.size()) - 1;
        std::vector<std::size_t> stack{s0};
        surf[s0] = id;
        while (!stack.empty()) {
            const std::size_t t = stack.back();
            stack.pop_back();
            for (int e = 0; e < 3; ++e) {
                const int a = T[t][static_cast<std::size_t>(e)], c = T[t][static_cast<std::size_t>((e + 1) % 3)];
                for (const int o2 : edgeTris[{std::min(a, c), std::max(a, c)}]) {
                    const auto u = static_cast<std::size_t>(o2);
                    if (surf[u] >= 0 || dlength(N[u]) == 0 || ddot(N[u], N[s0]) < 1 - 1e-9) continue;
                    bool flat = true;
                    for (int v : T[u]) flat = flat && std::fabs(ddot(N[s0], P[static_cast<std::size_t>(v)] - o)) <= tight;
                    if (!flat) continue;
                    surf[u] = id;
                    stack.push_back(u);
                }
            }
        }
    }
    // Slivers take the surface of the neighbour across their longest side.
    for (bool changed = true; changed;) {
        changed = false;
        for (std::size_t t = 0; t < T.size(); ++t) {
            if (surf[t] >= 0) continue;
            double longest = -1;
            for (int e = 0; e < 3; ++e) {
                const int a = T[t][static_cast<std::size_t>(e)], c = T[t][static_cast<std::size_t>((e + 1) % 3)];
                const double l = dlength(P[static_cast<std::size_t>(a)] - P[static_cast<std::size_t>(c)]);
                for (const int o2 : edgeTris[{std::min(a, c), std::max(a, c)}])
                    if (static_cast<std::size_t>(o2) != t && surf[static_cast<std::size_t>(o2)] >= 0 && l > longest)
                        longest = l, surf[t] = surf[static_cast<std::size_t>(o2)], sense[t] = sense[static_cast<std::size_t>(o2)];
            }
            changed = changed || surf[t] >= 0;
        }
    }
    for (std::size_t t = 0; t < T.size(); ++t)
        if (surf[t] < 0) return fail("a sliver with no face beside it");
    // Faces: neighbours on the same surface, the same way round.
    std::vector<int> face(T.size(), -1);
    int faces = 0;
    for (std::size_t s0 = 0; s0 < T.size(); ++s0) {
        if (face[s0] >= 0) continue;
        std::vector<std::size_t> stack{s0};
        face[s0] = faces;
        while (!stack.empty()) {
            const std::size_t t = stack.back();
            stack.pop_back();
            for (int e = 0; e < 3; ++e) {
                const int a = T[t][static_cast<std::size_t>(e)], c = T[t][static_cast<std::size_t>((e + 1) % 3)];
                for (const int o2 : edgeTris[{std::min(a, c), std::max(a, c)}]) {
                    const auto u = static_cast<std::size_t>(o2);
                    if (face[u] >= 0 || surf[u] != surf[t] || sense[u] != sense[t]) continue;
                    face[u] = faces;
                    stack.push_back(u);
                }
            }
        }
        if (static_cast<std::size_t>(++faces) > maxFaces) return fail("more than " + std::to_string(maxFaces) + " faces");
    }
    // Faces a boolean leaves as specks (a sliver of a triangle on its own) join the
    // neighbour they share the most edge with.
    for (bool changed = true; changed;) {
        changed = false;
        std::vector<double> area(static_cast<std::size_t>(faces), 0);
        for (std::size_t t = 0; t < T.size(); ++t)
            area[static_cast<std::size_t>(face[t])] += 0.5 * dlength(dcross(P[static_cast<std::size_t>(T[t][1])] - P[static_cast<std::size_t>(T[t][0])],
                                                                            P[static_cast<std::size_t>(T[t][2])] - P[static_cast<std::size_t>(T[t][0])]));
        const double speck = 100 * tight * tight;
        for (int f = 0; f < faces && !changed; ++f) {
            if (area[static_cast<std::size_t>(f)] >= speck) continue;
            std::map<int, double> shared;
            for (std::size_t t = 0; t < T.size(); ++t) {
                if (face[t] != f) continue;
                for (int e = 0; e < 3; ++e) {
                    const int a = T[t][static_cast<std::size_t>(e)], c = T[t][static_cast<std::size_t>((e + 1) % 3)];
                    for (const int o2 : edgeTris[{std::min(a, c), std::max(a, c)}])
                        if (face[static_cast<std::size_t>(o2)] != f)
                            shared[face[static_cast<std::size_t>(o2)]] += dlength(P[static_cast<std::size_t>(a)] - P[static_cast<std::size_t>(c)]);
                }
            }
            int into = -1;
            double most = -1;
            for (const auto& [g, l] : shared)
                if (l > most) most = l, into = g;
            if (into < 0) continue;
            std::size_t like = 0;
            for (std::size_t t = 0; t < T.size(); ++t)
                if (face[t] == into) like = t;
            for (std::size_t t = 0; t < T.size(); ++t)
                if (face[t] == f) face[t] = into, surf[t] = surf[like], sense[t] = sense[like];
            for (std::size_t t = 0; t < T.size(); ++t) // the last face takes the empty number
                if (face[t] == faces - 1) face[t] = f;
            --faces;
            changed = true;
        }
    }
    std::vector<int> faceSurf(static_cast<std::size_t>(faces));
    std::vector<char> faceSense(static_cast<std::size_t>(faces));
    for (std::size_t t = 0; t < T.size(); ++t) faceSurf[static_cast<std::size_t>(face[t])] = surf[t], faceSense[static_cast<std::size_t>(face[t])] = sense[t];

    // Boundary edges between faces, and the corners where runs of them break.
    std::map<std::pair<int, int>, std::pair<int, int>> boundary; // mesh edge -> the two faces (sorted)
    for (const auto& [e, ts] : edgeTris) {
        const int fa = face[static_cast<std::size_t>(ts[0])], fb = face[static_cast<std::size_t>(ts[1])];
        if (fa != fb) boundary[e] = {std::min(fa, fb), std::max(fa, fb)};
    }
    std::vector<std::vector<int>> adj(P.size());
    for (const auto& [e, f2] : boundary) adj[static_cast<std::size_t>(e.first)].push_back(e.second), adj[static_cast<std::size_t>(e.second)].push_back(e.first);
    std::vector<std::set<int>> facesAt(P.size());
    for (std::size_t t = 0; t < T.size(); ++t)
        for (int v : T[t]) facesAt[static_cast<std::size_t>(v)].insert(face[t]);
    std::vector<char> corner(P.size(), 0);
    for (std::size_t v = 0; v < P.size(); ++v) corner[v] = !adj[v].empty() && (adj[v].size() != 2 || facesAt[v].size() >= 3);
    auto pairOf = [&](int a, int c) { return boundary.at({std::min(a, c), std::max(a, c)}); };

    struct Run {
        std::vector<int> v;
        bool closed = false;
        std::pair<int, int> faces;
        int edge = -1;
    };
    std::vector<Run> runs;
    std::map<std::pair<int, int>, std::pair<int, bool>> runOf; // directed mesh edge -> (run, along it)
    std::set<std::pair<int, int>> used;
    for (const auto& [e0, f2] : boundary) {
        if (used.count(e0)) continue;
        Run r;
        r.faces = f2;
        r.v = {e0.first, e0.second};
        used.insert(e0);
        // Onward from the end, then back from the start, through vertices that aren't corners.
        for (int dir = 0; dir < 2 && !r.closed; ++dir) {
            for (int guard = 0; guard < 10000000; ++guard) {
                const int at = dir == 0 ? r.v.back() : r.v.front();
                const int from = dir == 0 ? r.v[r.v.size() - 2] : r.v[1];
                if (corner[static_cast<std::size_t>(at)]) break;
                int next = -1;
                for (int o2 : adj[static_cast<std::size_t>(at)])
                    if (o2 != from && !used.count({std::min(at, o2), std::max(at, o2)}) && pairOf(at, o2) == f2) next = o2;
                if (next < 0) {
                    // Back where it began: a closed run.
                    for (int o2 : adj[static_cast<std::size_t>(at)])
                        if (o2 != from && o2 == (dir == 0 ? r.v.front() : r.v.back()) && used.count({std::min(at, o2), std::max(at, o2)}) == 0 &&
                            pairOf(at, o2) == f2) {
                            used.insert({std::min(at, o2), std::max(at, o2)});
                            r.closed = true;
                        }
                    break;
                }
                used.insert({std::min(at, next), std::max(at, next)});
                if (dir == 0) r.v.push_back(next);
                else r.v.insert(r.v.begin(), next);
            }
        }
        const int id = static_cast<int>(runs.size());
        for (std::size_t i = 0; i + 1 < r.v.size() + (r.closed ? 1 : 0); ++i) {
            const int a = r.v[i], c = r.v[(i + 1) % r.v.size()];
            runOf[{a, c}] = {id, true};
            runOf[{c, a}] = {id, false};
        }
        runs.push_back(std::move(r));
    }

    auto b = std::make_shared<Brep>();
    std::map<int, int> surfIndex;
    auto brepSurface = [&](int s) {
        const auto it = surfIndex.find(s);
        if (it != surfIndex.end()) return it->second;
        b->surfaces.push_back(S[static_cast<std::size_t>(s)]);
        return surfIndex[s] = static_cast<int>(b->surfaces.size()) - 1;
    };
    // A point moved onto every surface of the faces round it (alternating projections).
    auto snap = [&](DVec3 p, const std::set<int>& fs) {
        for (int it = 0; it < 30; ++it) {
            DVec3 q = p;
            for (int f : fs) q = projectOnto(S[static_cast<std::size_t>(faceSurf[static_cast<std::size_t>(f)])], q);
            const bool still = dlength(q - p) < 1e-13 * size;
            p = q;
            if (still) break;
        }
        return p;
    };
    std::map<int, int> vertexOf;
    auto brepVertex = [&](int v, const DVec3& at) {
        const auto it = vertexOf.find(v);
        if (it != vertexOf.end()) return it->second;
        b->vertices.push_back({at});
        return vertexOf[v] = static_cast<int>(b->vertices.size()) - 1;
    };
    const double curveTol = std::max(1e-4 * size, 1e-6);
    for (Run& r : runs) {
        const BSurface& sa = S[static_cast<std::size_t>(faceSurf[static_cast<std::size_t>(r.faces.first)])];
        const BSurface& sb = S[static_cast<std::size_t>(faceSurf[static_cast<std::size_t>(r.faces.second)])];
        const std::set<int> two{r.faces.first, r.faces.second};
        std::vector<DVec3> pts; // on both surfaces
        for (int v : r.v) pts.push_back(snap(P[static_cast<std::size_t>(v)], two));
        // A circle: exactly, where a plane cuts a round surface square to its axis; else
        // fitted, when the points lie on one.
        DVec3 cc, cz;
        double cr = 0, worst = 1e300;
        bool circle = false;
        for (int pass = 0; pass < 2 && !circle; ++pass) {
            const BSurface& pl = pass ? sb : sa;
            const BSurface& rnd = pass ? sa : sb;
            if (pl.kind != BSurfaceKind::Plane) continue;
            if ((rnd.kind == BSurfaceKind::Cylinder || rnd.kind == BSurfaceKind::Cone) && std::fabs(ddot(pl.f.z, rnd.f.z)) > 1 - 1e-9) {
                const double h = ddot(pl.f.o - rnd.f.o, rnd.f.z);
                cc = rnd.f.o + rnd.f.z * h, cz = rnd.f.z;
                cr = rnd.kind == BSurfaceKind::Cylinder ? rnd.r1 : rnd.r1 + h * std::tan(rnd.r2);
            } else if (rnd.kind == BSurfaceKind::Sphere) {
                const double d = ddot(rnd.f.o - pl.f.o, pl.f.z);
                if (rnd.r1 * rnd.r1 - d * d <= 0) continue;
                cc = rnd.f.o - pl.f.z * d, cz = pl.f.z, cr = std::sqrt(rnd.r1 * rnd.r1 - d * d);
            } else
                continue;
            if (!(cr > 0)) continue;
            worst = 0;
            for (const DVec3& p : pts) worst = std::max(worst, dlength(p - onCircle(p, cc, cz, cr)));
            circle = worst <= std::max(curveTol, 0.02 * cr);
        }
        // Two surfaces of revolution on one axis meet in circles round it: worked out
        // exactly where one is a cylinder, else centred on the axis through the points.
        auto round = [](const BSurface& q) { return q.kind == BSurfaceKind::Cylinder || q.kind == BSurfaceKind::Cone || q.kind == BSurfaceKind::Sphere || q.kind == BSurfaceKind::Torus; };
        if (!circle && round(sa) && round(sb)) {
            const BSurface& ax = sa.kind == BSurfaceKind::Sphere ? sb : sa; // a sphere fits any axis through its centre
            const BSurface& other = &ax == &sa ? sb : sa;
            const DVec3 z = ax.f.z;
            const bool parallel = other.kind == BSurfaceKind::Sphere || std::fabs(ddot(z, other.f.z)) > 1 - 1e-9;
            if (ax.kind != BSurfaceKind::Sphere && parallel && dlength(dcross(other.f.o - ax.f.o, z)) <= curveTol) {
                DVec3 mean{0, 0, 0};
                for (const DVec3& p : pts) mean += p;
                mean = mean / static_cast<double>(pts.size());
                const double hm = ddot(mean - ax.f.o, z);
                double h = hm, rad = 0;
                for (const DVec3& p : pts) rad += dlength((p - ax.f.o) - z * ddot(p - ax.f.o, z));
                rad /= static_cast<double>(pts.size());
                const BSurface* cyl = ax.kind == BSurfaceKind::Cylinder ? &ax : other.kind == BSurfaceKind::Cylinder ? &other : nullptr;
                if (cyl) {
                    const BSurface& q = cyl == &ax ? other : ax;
                    const double rc = cyl->r1, base = ddot(q.f.o - ax.f.o, z); // q's centre along the axis
                    std::vector<double> hs;
                    if (q.kind == BSurfaceKind::Torus) {
                        const double k = q.r2 * q.r2 - (rc - q.r1) * (rc - q.r1);
                        if (k >= -1e-9) hs = {base + std::sqrt(std::max(k, 0.0)), base - std::sqrt(std::max(k, 0.0))};
                    } else if (q.kind == BSurfaceKind::Sphere) {
                        const double k = q.r1 * q.r1 - rc * rc;
                        if (k >= -1e-9) hs = {base + std::sqrt(std::max(k, 0.0)), base - std::sqrt(std::max(k, 0.0))};
                    } else if (q.kind == BSurfaceKind::Cone && std::fabs(std::tan(q.r2)) > 1e-12) {
                        hs = {base + (rc - q.r1) / std::tan(q.r2) * ddot(q.f.z, z)};
                    }
                    if (!hs.empty()) {
                        h = hs[0];
                        for (double c : hs)
                            if (std::fabs(c - hm) < std::fabs(h - hm)) h = c;
                        rad = rc;
                    }
                }
                cc = ax.f.o + z * h, cz = z, cr = rad;
                worst = 0;
                for (const DVec3& p : pts) worst = std::max(worst, dlength(p - onCircle(p, cc, cz, cr)));
                double allow = std::max(toleranceFor(sa, tight), toleranceFor(sb, tight));
                // Surfaces that touch (a round's tangent seam): triangles a little way along
                // either one fit both, so the mesh's seam wanders by sqrt(2 r tolerance).
                double da, db;
                DVec3 na, nb;
                const DVec3 onc = cc + dnormalize(dcross(z, std::fabs(z.z) < 0.9 ? DVec3{0, 0, 1} : DVec3{1, 0, 0})) * cr;
                if (cr > 0 && surfaceDistance(sa, onc, da, na) && surfaceDistance(sb, onc, db, nb) && std::fabs(ddot(na, nb)) > 0.999) {
                    auto bend = [](const BSurface& q) { return q.kind == BSurfaceKind::Torus ? q.r2 : q.r1; };
                    allow += std::sqrt(2 * std::min(bend(sa), bend(sb)) * allow);
                }
                circle = cr > 0 && worst <= allow;
            }
        }
        bool line = false;
        if (!circle && !r.closed) {
            const DVec3 d = dnormalize(pts.back() - pts.front());
            worst = 0;
            for (const DVec3& p : pts) worst = std::max(worst, dlength(dcross(p - pts.front(), d)));
            line = dlength(pts.back() - pts.front()) > 0 && worst <= curveTol;
        }
        if (!circle && !line && pts.size() >= 4 && fitCircle(pts, cc, cz, cr, worst) && worst <= curveTol) circle = true;
        if (std::getenv("SICDM_EXACT_DEBUG") && !circle && !line) {
            std::fprintf(stderr, "  polyline between %d and %d (%zu points, closed %d): circle fit off by %g (tolerance %g)\n", static_cast<int>(sa.kind),
                         static_cast<int>(sb.kind), pts.size(), static_cast<int>(r.closed), worst, curveTol);
        }
        // The ends: B-rep vertices on every surface round them (and on the circle).
        auto endAt = [&](int v) {
            DVec3 at = snap(P[static_cast<std::size_t>(v)], facesAt[static_cast<std::size_t>(v)]);
            if (circle) at = onCircle(at, cc, cz, cr);
            return brepVertex(v, at);
        };
        const int v0 = endAt(r.v.front());
        const int v1 = r.closed ? v0 : endAt(r.v.back());
        BrepEdge e{v0, v1, -1, true};
        if (circle) {
            // Turning the way the run goes.
            DVec3 turn{0, 0, 0};
            for (std::size_t i = 0; i + 1 < pts.size() + (r.closed ? 1 : 0); ++i) turn += dcross(pts[i] - cc, pts[(i + 1) % pts.size()] - cc);
            if (ddot(turn, cz) < 0) cz = -cz;
            BCurve c;
            c.kind = BCurveKind::Circle;
            c.f = Frame::make(cc, cz, b->vertices[static_cast<std::size_t>(v0)].p - cc);
            c.r1 = cr;
            b->curves.push_back(c);
            e.curve = static_cast<int>(b->curves.size()) - 1;
            ++rep.circles;
        } else if (line) {
            ++rep.lines;
        } else {
            // Points on both surfaces, joined straight: a B-spline of degree 1 (what other
            // systems read most readily).
            std::vector<DVec3> q = pts;
            if (r.closed) q.push_back(q.front());
            q.front() = b->vertices[static_cast<std::size_t>(v0)].p;
            q.back() = b->vertices[static_cast<std::size_t>(v1)].p;
            // Points crowding an end (where surfaces touch, the end's position is loosely
            // set) would make a stub along the next edge: dropped.
            for (std::size_t i = q.size() - 1; i-- > 1;)
                if (dlength(q[i] - q.front()) < 2 * curveTol || dlength(q[i] - q.back()) < 2 * curveTol) q.erase(q.begin() + static_cast<std::ptrdiff_t>(i));
            BCurve c;
            c.kind = BCurveKind::BSpline;
            c.bs.degree = 1;
            c.bs.cp = q;
            c.bs.knots.push_back(0);
            double along = 0;
            for (std::size_t i = 0; i < q.size(); ++i) {
                if (i) along += dlength(q[i] - q[i - 1]);
                c.bs.knots.push_back(along);
            }
            c.bs.knots.push_back(along);
            b->curves.push_back(c);
            e.curve = static_cast<int>(b->curves.size()) - 1;
            ++rep.polylines;
        }
        b->edges.push_back(e);
        r.edge = static_cast<int>(b->edges.size()) - 1;
    }

    // Faces and their loops of runs.
    std::vector<std::vector<std::pair<int, bool>>> faceRuns(static_cast<std::size_t>(faces));
    {
        std::set<std::tuple<int, int, bool>> seen;
        for (std::size_t t = 0; t < T.size(); ++t)
            for (int e = 0; e < 3; ++e) {
                const int a = T[t][static_cast<std::size_t>(e)], c = T[t][static_cast<std::size_t>((e + 1) % 3)];
                const auto it = runOf.find({a, c});
                if (it == runOf.end()) continue;
                if (seen.insert({face[t], it->second.first, it->second.second}).second) faceRuns[static_cast<std::size_t>(face[t])].push_back(it->second);
            }
    }
    BShell sh;
    sh.closed = true;
    for (int f = 0; f < faces; ++f) {
        auto& list = faceRuns[static_cast<std::size_t>(f)];
        if (list.empty()) return fail("a face with no edges (a whole closed surface)");
        BFace bf;
        bf.surface = brepSurface(faceSurf[static_cast<std::size_t>(f)]);
        bf.sameSense = faceSense[static_cast<std::size_t>(f)] != 0;
        const BSurface& fs = S[static_cast<std::size_t>(faceSurf[static_cast<std::size_t>(f)])];
        std::vector<char> done(list.size(), 0);
        double bestArea = -1e300;
        for (std::size_t s0 = 0; s0 < list.size(); ++s0) {
            if (done[s0]) continue;
            BLoop l;
            std::vector<DVec3> ring;
            std::size_t at = s0;
            for (std::size_t guard = 0; guard <= list.size(); ++guard) {
                done[at] = 1;
                const Run& r = runs[static_cast<std::size_t>(list[at].first)];
                const bool fwd = list[at].second;
                const BrepEdge& be = b->edges[static_cast<std::size_t>(r.edge)];
                l.edges.push_back({r.edge, fwd});
                for (std::size_t i = 0; i < r.v.size(); ++i) ring.push_back(P[static_cast<std::size_t>(r.v[fwd ? i : r.v.size() - 1 - i])]);
                if (r.closed) break;
                const int end = fwd ? be.v1 : be.v0;
                std::size_t nxt = list.size();
                for (std::size_t k = 0; k < list.size() && nxt == list.size(); ++k) {
                    if (done[k]) continue;
                    const Run& q = runs[static_cast<std::size_t>(list[k].first)];
                    if (q.closed) continue;
                    const BrepEdge& qe = b->edges[static_cast<std::size_t>(q.edge)];
                    if ((list[k].second ? qe.v0 : qe.v1) == end) nxt = k;
                }
                if (nxt == list.size()) break;
                at = nxt;
            }
            if (fs.kind == BSurfaceKind::Plane) { // the outer loop: the largest, counter-clockwise about the face's normal
                DVec3 ar{0, 0, 0};
                for (std::size_t i = 0; i < ring.size(); ++i) ar += dcross(ring[i], ring[(i + 1) % ring.size()]);
                const double area = ddot(ar, fs.f.z) * (bf.sameSense ? 1 : -1);
                if (area > bestArea) {
                    bestArea = area;
                    for (BLoop& q : bf.loops) q.outer = false;
                    l.outer = true;
                }
            }
            bf.loops.push_back(std::move(l));
        }
        ++rep.faces;
        // A cone's apex or a sphere's pole inside the face: a point loop there.
        if (fs.kind == BSurfaceKind::Cone || fs.kind == BSurfaceKind::Sphere) {
            std::vector<DVec3> poles;
            if (fs.kind == BSurfaceKind::Sphere) poles = {fs.f.o + fs.f.z * fs.r1, fs.f.o - fs.f.z * fs.r1};
            else if (std::fabs(std::tan(fs.r2)) > 1e-12) poles = {fs.f.o - fs.f.z * (fs.r1 / std::tan(fs.r2))};
            for (const DVec3& pole : poles) {
                bool in = false;
                for (std::size_t t = 0; t < T.size() && !in; ++t)
                    if (face[t] == f)
                        for (int v : T[t]) in = in || dlength(P[static_cast<std::size_t>(v)] - pole) <= toleranceFor(fs, tight);
                if (!in) continue;
                BLoop pl;
                b->vertices.push_back({pole});
                pl.vertex = static_cast<int>(b->vertices.size()) - 1;
                bf.loops.push_back(pl);
            }
        }
        if (faceSurf[static_cast<std::size_t>(f)] < static_cast<int>(catalogue)) ++rep.exactFaces;
        sh.faces.push_back(std::move(bf));
    }
    b->shells.push_back(std::move(sh));

    // It must tessellate back to the same solid.
    TessellateReport tr;
    TessellateOptions o;
    o.chord = std::max(1e-3, 2e-4 * size);
    const MeshData back = tessellate(*b, o, &tr);
    if (tr.failedFaces > 0) {
        if (std::getenv("SICDM_EXACT_DEBUG"))
            for (std::size_t f = 0; f < b->shells[0].faces.size(); ++f) {
                const BFace& bf = b->shells[0].faces[f];
                const BSurface& q = b->surfaces[static_cast<std::size_t>(bf.surface)];
                std::fprintf(stderr, "    face %zu kind %d sense %d tris %zu loops", f, static_cast<int>(q.kind), bf.sameSense ? 1 : 0,
                             f < tr.faceTriangles.size() ? tr.faceTriangles[f] : 0);
                for (const BLoop& l : bf.loops) {
                    std::fprintf(stderr, " [");
                    for (const OrientedEdge& oe : l.edges) {
                        const BrepEdge& e = b->edges[static_cast<std::size_t>(oe.edge)];
                        std::fprintf(stderr, "%s%s ", e.curve < 0 ? "L" : b->curves[static_cast<std::size_t>(e.curve)].kind == BCurveKind::Circle ? "C" : "P", oe.forward ? "+" : "-");
                    }
                    std::fprintf(stderr, "%s]", l.vertex >= 0 ? "pole" : "");
                    if (!l.edges.empty()) {
                        const DVec3 q0 = b->vertices[static_cast<std::size_t>(b->edges[static_cast<std::size_t>(l.edges[0].edge)].v0)].p;
                        std::fprintf(stderr, "@(%.3f %.3f %.3f)", q0.x, q0.y, q0.z);
                    }
                }
                std::fprintf(stderr, "\n");
            }
        return fail("a face would not tessellate (" + tr.firstError + ")");
    }
    const double v0 = meshVolume(mesh), v1 = meshVolume(back);
    if (!(std::fabs(v1 - v0) <= 0.01 * std::fabs(v0) + 1e-9)) return fail("the B-rep's volume differs from the mesh's");
    return b;
}

bool keepExactGeometry(Body& body, const std::vector<BSurface>& surfaces) {
    body.brep.reset();
    body.brepKey = 0;
    if (surfaces.empty()) return false;
    ExactReport rep;
    auto b = exactBrepFromMesh(body.mesh, surfaces, &rep, 600);
    if (std::getenv("SICDM_EXACT_DEBUG"))
        std::fprintf(stderr, "exact %s: %s faces %zu exact %zu area %.3f\n", body.name.c_str(), b ? "ok" : rep.why.c_str(), rep.faces, rep.exactFaces, rep.exactArea);
    // Mostly facets (a mesh body cut by an exact tool): no better than the mesh.
    if (!b || rep.exactArea < 0.5) return false;
    b->name = body.name;
    body.brep = std::move(b);
    body.brepKey = meshKey(body.mesh);
    return true;
}

} // namespace sicdm

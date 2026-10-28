// Direct editing (DirectEdit.hpp): faces found by a point, offset, deleted and moved. It
// works on the body's mesh in its own frame; exact geometry comes back through
// keepExactGeometry() with the surfaces the faces now lie on.

#include "core/DirectEdit.hpp"

#include "core/ExactMesh.hpp"
#include "core/Features.hpp"
#include "core/Scene.hpp"
#include "core/Sculpt.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <map>
#include <set>
#include <tuple>
#include <unordered_map>

namespace sicdm {

namespace {

bool fail(std::string* why, const std::string& w) {
    if (why) *why = w;
    return false;
}

// --- the mesh as faces ------------------------------------------------------------------------

// The body's triangles with their welded corners, their neighbours across each edge and the
// exact surface each lies on.
struct Topo {
    const MeshData* m = nullptr;
    std::vector<int> weld;        // vertex -> welded point
    std::vector<DVec3> p;         // welded points
    std::vector<std::array<int, 3>> tri; // welded corners
    std::vector<std::array<int, 3>> nb;  // the triangle across each edge (k: corner k to k+1), -1 none
    std::vector<DVec3> normal;    // per triangle, unit
    std::vector<double> area;
    std::vector<int> surf;        // index into surfaces, or -1
    std::vector<BSurface> surfaces;
    double tol = 1e-5;
    double diag = 1;
};

// Whether two surfaces are the same one (a B-rep can carry a surface more than once: one per
// face on it).
bool sameSurface(const BSurface& a, const BSurface& b, double tol) {
    if (a.kind != b.kind) return false;
    auto parallel = [](const DVec3& u, const DVec3& v) { return std::fabs(std::fabs(ddot(dnormalize(u), dnormalize(v))) - 1) < 1e-9; };
    auto onAxis = [&](const DVec3& p) { // p on a's axis line
        const DVec3 q = p - a.f.o;
        return dlength(q - a.f.z * ddot(q, a.f.z)) < tol;
    };
    switch (a.kind) {
    case BSurfaceKind::Plane: return parallel(a.f.z, b.f.z) && std::fabs(ddot(a.f.z, b.f.o - a.f.o)) < tol;
    case BSurfaceKind::Cylinder: return parallel(a.f.z, b.f.z) && onAxis(b.f.o) && std::fabs(a.r1 - b.r1) < tol;
    case BSurfaceKind::Sphere: return dlength(a.f.o - b.f.o) < tol && std::fabs(a.r1 - b.r1) < tol;
    case BSurfaceKind::Torus: return parallel(a.f.z, b.f.z) && dlength(a.f.o - b.f.o) < tol && std::fabs(a.r1 - b.r1) < tol && std::fabs(a.r2 - b.r2) < tol;
    case BSurfaceKind::Cone: {
        // The same axis, the same opening and the same radius where b's frame is.
        if (!parallel(a.f.z, b.f.z) || !onAxis(b.f.o)) return false;
        const double sg = ddot(a.f.z, b.f.z) > 0 ? 1.0 : -1.0;
        if (std::fabs(std::tan(a.r2) - sg * std::tan(b.r2)) > 1e-9) return false;
        return std::fabs(a.r1 + ddot(b.f.o - a.f.o, a.f.z) * std::tan(a.r2) - b.r1) < tol;
    }
    default: return false;
    }
}

Topo topology(const Body& body) {
    Topo t;
    const MeshData& m = body.mesh;
    t.m = &m;
    std::map<std::tuple<float, float, float>, int> at;
    t.weld.resize(m.vertices.size());
    for (std::size_t i = 0; i < m.vertices.size(); ++i) {
        const Vec3& q = m.vertices[i].position;
        const auto [it, added] = at.try_emplace({q.x, q.y, q.z}, static_cast<int>(t.p.size()));
        if (added) t.p.push_back(DVec3(q));
        t.weld[i] = it->second;
    }
    AABB box = m.bounds();
    t.diag = box.valid() ? std::max(1e-3, static_cast<double>(length(box.max - box.min))) : 1.0;
    t.tol = 2e-5 * t.diag + 1e-6;
    const std::size_t nt = m.indices.size() / 3;
    t.tri.resize(nt);
    t.normal.resize(nt);
    t.area.resize(nt);
    for (std::size_t k = 0; k < nt; ++k) {
        for (int c = 0; c < 3; ++c) t.tri[k][static_cast<std::size_t>(c)] = t.weld[m.indices[k * 3 + static_cast<std::size_t>(c)]];
        const DVec3 e = dcross(t.p[static_cast<std::size_t>(t.tri[k][1])] - t.p[static_cast<std::size_t>(t.tri[k][0])],
                               t.p[static_cast<std::size_t>(t.tri[k][2])] - t.p[static_cast<std::size_t>(t.tri[k][0])]);
        t.area[k] = 0.5 * dlength(e);
        t.normal[k] = dnormalize(e);
    }
    // Neighbours: the other triangle on each (undirected) edge.
    std::unordered_map<std::uint64_t, std::vector<std::pair<int, int>>> edges;
    auto key = [](int a, int b) {
        if (a > b) std::swap(a, b);
        return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(a)) << 32) | static_cast<std::uint32_t>(b);
    };
    for (std::size_t k = 0; k < nt; ++k) {
        const auto& tr = t.tri[k];
        if (tr[0] == tr[1] || tr[1] == tr[2] || tr[2] == tr[0]) continue; // a corner twice: no area, no neighbours
        for (int c = 0; c < 3; ++c) edges[key(tr[static_cast<std::size_t>(c)], tr[static_cast<std::size_t>((c + 1) % 3)])].push_back({static_cast<int>(k), c});
    }
    t.nb.assign(nt, {-1, -1, -1});
    for (const auto& [k, list] : edges)
        if (list.size() == 2) {
            t.nb[static_cast<std::size_t>(list[0].first)][static_cast<std::size_t>(list[0].second)] = list[1].first;
            t.nb[static_cast<std::size_t>(list[1].first)][static_cast<std::size_t>(list[1].second)] = list[0].first;
        }
    // The exact surface each triangle lies on: all its corners on it, and facing its way
    // (a cylinder's end disc has triangles with every corner on the cylinder).
    for (const BSurface& s : exactSurfaces(body))
        if (std::none_of(t.surfaces.begin(), t.surfaces.end(), [&](const BSurface& o) { return sameSurface(o, s, t.tol * 10); })) t.surfaces.push_back(s);
    t.surf.assign(nt, -1);
    for (std::size_t k = 0; k < nt && !t.surfaces.empty(); ++k) {
        if (!(t.area[k] > 0)) continue;
        const DVec3 mid = (t.p[static_cast<std::size_t>(t.tri[k][0])] + t.p[static_cast<std::size_t>(t.tri[k][1])] + t.p[static_cast<std::size_t>(t.tri[k][2])]) / 3.0;
        double best = 1e300;
        for (std::size_t s = 0; s < t.surfaces.size(); ++s) {
            double worst = 0, d = 0;
            DVec3 n;
            bool ok = true;
            for (int c = 0; c < 3 && ok; ++c) {
                ok = surfaceDistance(t.surfaces[s], t.p[static_cast<std::size_t>(t.tri[k][static_cast<std::size_t>(c)])], d, n);
                worst = std::max(worst, std::fabs(d));
            }
            if (!ok || worst > t.tol * 20) continue;
            if (!surfaceDistance(t.surfaces[s], mid, d, n) || std::fabs(ddot(n, t.normal[k])) < 0.8) continue;
            if (worst < best) best = worst, t.surf[k] = static_cast<int>(s);
        }
    }
    return t;
}

// A face: the triangles grown from one across edges while they stay on its surface (or, with
// none, in its plane).
struct Face {
    std::vector<int> tris;
    std::vector<char> in;   // per triangle
    int surf = -1;          // index into the topology's surfaces
    bool planar = false;
    DVec3 n{0, 0, 1};       // planar: outward normal
    double c = 0;           // planar: dot(n, p) on it
    int outward = 1;        // round: the outward normal is the surface's (1) or the other way (-1)
    double area = 0;
};

bool samePlane(const Topo& t, int k, const DVec3& n, double c) {
    if (ddot(t.normal[static_cast<std::size_t>(k)], n) < 1 - 1e-6) return false;
    for (int v : t.tri[static_cast<std::size_t>(k)])
        if (std::fabs(ddot(t.p[static_cast<std::size_t>(v)], n) - c) > t.tol * 10) return false;
    return true;
}

Face growFace(const Topo& t, int seed) {
    Face f;
    f.in.assign(t.tri.size(), 0);
    f.surf = t.surf[static_cast<std::size_t>(seed)];
    const BSurface* s = f.surf >= 0 ? &t.surfaces[static_cast<std::size_t>(f.surf)] : nullptr;
    f.planar = s ? s->kind == BSurfaceKind::Plane : true;
    f.n = t.normal[static_cast<std::size_t>(seed)];
    f.c = ddot(f.n, t.p[static_cast<std::size_t>(t.tri[static_cast<std::size_t>(seed)][0])]);
    if (s && s->kind == BSurfaceKind::Plane) {
        f.n = s->f.z * (ddot(s->f.z, f.n) >= 0 ? 1.0 : -1.0);
        f.c = ddot(f.n, s->f.o);
    }
    std::vector<int> stack{seed};
    f.in[static_cast<std::size_t>(seed)] = 1;
    while (!stack.empty()) {
        const int k = stack.back();
        stack.pop_back();
        f.tris.push_back(k);
        f.area += t.area[static_cast<std::size_t>(k)];
        for (int o : t.nb[static_cast<std::size_t>(k)]) {
            if (o < 0 || f.in[static_cast<std::size_t>(o)]) continue;
            // On the face's surface: the same one, or (a sliver, or a triangle the B-rep put on a
            // near copy of it) every corner on it and facing its way.
            bool same = f.surf >= 0 ? t.surf[static_cast<std::size_t>(o)] == f.surf && ddot(t.normal[static_cast<std::size_t>(o)], t.normal[static_cast<std::size_t>(k)]) > -0.5
                                     : t.surf[static_cast<std::size_t>(o)] < 0 && samePlane(t, o, f.n, f.c);
            if (!same) {
                // (A sliver some other surface has claimed stays with it: its corners being on this
                // one too is only where the two meet.)
                const bool sliver = t.area[static_cast<std::size_t>(o)] < 1e-6 * t.diag * t.diag && t.surf[static_cast<std::size_t>(o)] < 0;
                bool on = true, facing = sliver;
                for (int v : t.tri[static_cast<std::size_t>(o)]) {
                    const DVec3& q = t.p[static_cast<std::size_t>(v)];
                    if (s) {
                        double dd = 0;
                        DVec3 nn;
                        if (!surfaceDistance(*s, q, dd, nn) || std::fabs(dd) > t.tol * 2) on = false;
                        else if (!sliver && std::fabs(ddot(nn, t.normal[static_cast<std::size_t>(o)])) > 0.99 && ddot(t.normal[static_cast<std::size_t>(o)], t.normal[static_cast<std::size_t>(k)]) > -0.5) facing = true;
                    } else if (std::fabs(ddot(q, f.n) - f.c) > t.tol * 2) on = false;
                }
                if (!s && !sliver) facing = ddot(t.normal[static_cast<std::size_t>(o)], f.n) > 0.99;
                same = on && facing;
            }
            if (same) f.in[static_cast<std::size_t>(o)] = 1, stack.push_back(o);
        }
    }
    // Triangles with a corner twice (no area, no neighbours) go with the face their corners are on.
    {
        std::vector<char> mine(t.p.size(), 0);
        for (int k : f.tris)
            for (int v : t.tri[static_cast<std::size_t>(k)]) mine[static_cast<std::size_t>(v)] = 1;
        for (std::size_t k = 0; k < t.tri.size(); ++k) {
            const auto& tr = t.tri[k];
            if (f.in[k] || !(tr[0] == tr[1] || tr[1] == tr[2] || tr[2] == tr[0])) continue;
            if (mine[static_cast<std::size_t>(tr[0])] && mine[static_cast<std::size_t>(tr[1])] && mine[static_cast<std::size_t>(tr[2])]) f.in[k] = 1, f.tris.push_back(static_cast<int>(k));
        }
    }
    if (s && s->kind != BSurfaceKind::Plane) {
        double d = 0;
        DVec3 sn;
        const int k = f.tris.front();
        const DVec3 mid = (t.p[static_cast<std::size_t>(t.tri[static_cast<std::size_t>(k)][0])] + t.p[static_cast<std::size_t>(t.tri[static_cast<std::size_t>(k)][1])] +
                           t.p[static_cast<std::size_t>(t.tri[static_cast<std::size_t>(k)][2])]) / 3.0;
        surfaceDistance(*s, mid, d, sn);
        f.outward = ddot(sn, t.normal[static_cast<std::size_t>(k)]) >= 0 ? 1 : -1;
    }
    return f;
}

// The triangle nearest a point (local), or -1.
int nearestTriangle(const Topo& t, const DVec3& q, double* dist = nullptr) {
    int best = -1;
    double bd = 1e300;
    const Vec3 qf = q.f();
    for (std::size_t k = 0; k < t.tri.size(); ++k) {
        if (!(t.area[k] > 0)) continue;
        const Vec3 c = closestPointOnTriangle(qf, t.p[static_cast<std::size_t>(t.tri[k][0])].f(), t.p[static_cast<std::size_t>(t.tri[k][1])].f(),
                                              t.p[static_cast<std::size_t>(t.tri[k][2])].f());
        const double d = static_cast<double>(length(c - qf));
        if (d < bd) bd = d, best = static_cast<int>(k);
    }
    if (dist) *dist = bd;
    return best;
}

DirectFaceKind kindOf(const Topo& t, const Face& f) {
    if (f.planar) return DirectFaceKind::Plane;
    if (f.surf < 0) return DirectFaceKind::Freeform;
    switch (t.surfaces[static_cast<std::size_t>(f.surf)].kind) {
    case BSurfaceKind::Cylinder: return DirectFaceKind::Cylinder;
    case BSurfaceKind::Cone: return DirectFaceKind::Cone;
    case BSurfaceKind::Sphere: return DirectFaceKind::Sphere;
    case BSurfaceKind::Torus: return DirectFaceKind::Torus;
    default: return DirectFaceKind::Freeform;
    }
}

std::string num(double v) {
    char b[32];
    std::snprintf(b, sizeof b, "%.4g", v);
    return b;
}

// Boundary edges of a set of triangles: (a, b) as the set's triangle runs them, and the
// triangle across.
struct BEdge {
    int a, b, across;
};
std::vector<BEdge> boundary(const Topo& t, const std::vector<char>& in) {
    std::vector<BEdge> out;
    for (std::size_t k = 0; k < t.tri.size(); ++k) {
        if (!in[k]) continue;
        const auto& tr = t.tri[k];
        if (tr[0] == tr[1] || tr[1] == tr[2] || tr[2] == tr[0]) continue; // no area: no rim of its own
        for (int c = 0; c < 3; ++c) {
            const int o = t.nb[k][static_cast<std::size_t>(c)];
            if (o >= 0 && in[static_cast<std::size_t>(o)]) continue;
            out.push_back({t.tri[k][static_cast<std::size_t>(c)], t.tri[k][static_cast<std::size_t>((c + 1) % 3)], o});
        }
    }
    return out;
}

// The face of `body` at the world point, with the body's topology.
bool faceAt(const Body& body, const Vec3& world, Topo& t, Face& f, std::string* why) {
    if (body.mesh.indices.empty()) return fail(why, body.name + " has no faces");
    t = topology(body);
    double d = 0;
    const int k = nearestTriangle(t, DVec3(world - body.offset), &d);
    if (k < 0 || d > std::max(0.5, 0.02 * t.diag)) {
        if (why) *why = "No face of " + body.name + " there (the point is " + num(d) + " mm from it)";
        return false;
    }
    f = growFace(t, k);
    // A mesh's flat facet that its neighbours carry on smoothly is a piece of a curved
    // surface, not a face of its own.
    if (f.surf < 0) {
        double smooth = 0, all = 0;
        for (const BEdge& e : boundary(t, f.in)) {
            const double l = dlength(t.p[static_cast<std::size_t>(e.a)] - t.p[static_cast<std::size_t>(e.b)]);
            all += l;
            if (e.across >= 0 && ddot(t.normal[static_cast<std::size_t>(e.across)], f.n) > std::cos(20.0 * 3.14159265358979 / 180)) smooth += l;
        }
        if (all > 0 && smooth > 0.5 * all) f.planar = false;
    }
    return true;
}

// --- results --------------------------------------------------------------------------------

// Writes moved welded points back into the body's mesh (every copy of each), with normals
// from the triangles; the triangles stay as they were.
MeshData withPoints(const Topo& t, const std::vector<DVec3>& p) {
    MeshData m = *t.m;
    for (std::size_t i = 0; i < m.vertices.size(); ++i) m.vertices[i].position = p[static_cast<std::size_t>(t.weld[i])].f();
    std::vector<Vec3> acc(m.vertices.size(), Vec3{});
    for (std::size_t k = 0; k + 2 < m.indices.size(); k += 3) {
        const Vec3 a = m.vertices[m.indices[k]].position, b = m.vertices[m.indices[k + 1]].position, c = m.vertices[m.indices[k + 2]].position;
        const Vec3 n = cross(b - a, c - a);
        for (int j = 0; j < 3; ++j) acc[m.indices[k + static_cast<std::size_t>(j)]] += n;
    }
    for (std::size_t i = 0; i < m.vertices.size(); ++i)
        if (length(acc[i]) > 0) m.vertices[i].normal = normalize(acc[i]);
    m.computeFeatureEdges();
    return m;
}

// The body's new mesh (local) and exact geometry, from these surfaces.
void finish(Body& body, MeshData mesh, const std::vector<BSurface>& surfaces) {
    const bool wasExact = body.exact();
    // Points a hair apart (a seam's two ends) become one, so the mesh closes up.
    {
        const AABB box = mesh.bounds();
        const double eps = 1e-6 * (box.valid() ? std::max(1e-3, static_cast<double>(length(box.max - box.min))) : 1.0);
        std::map<std::tuple<long long, long long, long long>, Vec3> cells;
        for (Vertex& v : mesh.vertices) {
            const long long ix = std::llround(v.position.x / eps), iy = std::llround(v.position.y / eps), iz = std::llround(v.position.z / eps);
            bool snapped = false;
            for (long long dx = -1; dx <= 1 && !snapped; ++dx)
                for (long long dy = -1; dy <= 1 && !snapped; ++dy)
                    for (long long dz = -1; dz <= 1 && !snapped; ++dz) {
                        const auto it = cells.find({ix + dx, iy + dy, iz + dz});
                        if (it != cells.end()) v.position = it->second, snapped = true;
                    }
            if (!snapped) cells[{ix, iy, iz}] = v.position;
        }
    }
    // Triangles with a corner twice carry nothing but make the mesh look open: they go.
    {
        std::vector<std::uint32_t> kept;
        kept.reserve(mesh.indices.size());
        for (std::size_t k = 0; k + 2 < mesh.indices.size(); k += 3) {
            const Vec3 &a = mesh.vertices[mesh.indices[k]].position, &b = mesh.vertices[mesh.indices[k + 1]].position, &c = mesh.vertices[mesh.indices[k + 2]].position;
            auto same = [](const Vec3& p, const Vec3& q) { return p.x == q.x && p.y == q.y && p.z == q.z; };
            if (same(a, b) || same(b, c) || same(c, a)) continue;
            kept.insert(kept.end(), {mesh.indices[k], mesh.indices[k + 1], mesh.indices[k + 2]});
        }
        if (kept.size() != mesh.indices.size()) {
            mesh.indices = std::move(kept);
            mesh.groups.clear(), mesh.quads.clear(), mesh.uvs.clear();
        }
    }
    body.mesh = std::move(mesh); // a mesh now: its parameters no longer make it
    body.kind = "mesh";
    body.params.clear();
    if (wasExact && !surfaces.empty()) keepExactGeometry(body, surfaces);
    else {
        body.brep.reset();
        body.brepKey = 0;
    }
}

// Adds or cuts a closed tool (local) on the body; checks the volume went the right way.
bool applyTool(Body& body, MeshData tool, bool add, const std::vector<BSurface>& surfaces, std::string* why) {
    // Outward: a tool built inside out is turned round.
    if (meshVolume(tool) < 0)
        for (std::size_t k = 0; k + 2 < tool.indices.size(); k += 3) std::swap(tool.indices[k + 1], tool.indices[k + 2]);
    MeshData a = body.mesh;
    const double before = meshVolume(a);
    if (!meshBooleanRobust(a, tool, add ? BooleanOp::Union : BooleanOp::Subtract)) {
        if (why) *why = add ? "The material could not be added" : "That would remove the whole body";
        return false;
    }
    if (openEdgeCount(a) > openEdgeCount(body.mesh)) {
        if (why) *why = "That would leave the solid open; " + body.name + " is unchanged";
        return false;
    }
    const double after = meshVolume(a), slack = 1e-6 * std::max(std::fabs(before), 1.0);
    if ((add && after < before - slack) || (!add && after > before + slack) || !(after > 0)) {
        if (why) *why = "That did not give a clean solid; " + body.name + " is unchanged";
        return false;
    }
    if (std::fabs(after - before) <= slack) {
        if (why) *why = "Nothing changed";
        return false;
    }
    finish(body, std::move(a), surfaces);
    return true;
}

// A closed mesh of the slab between a face and the face moved along its outward normal by d
// (each point by its own normal: round faces stay round).
MeshData slab(const Topo& t, const Face& f, double d) {
    const BSurface* s = f.surf >= 0 ? &t.surfaces[static_cast<std::size_t>(f.surf)] : nullptr;
    std::map<int, std::pair<std::uint32_t, std::uint32_t>> idx; // welded -> (near, far) vertex
    MeshData m;
    auto vertex = [&](int w) {
        const auto it = idx.find(w);
        if (it != idx.end()) return it->second;
        const DVec3 q = t.p[static_cast<std::size_t>(w)];
        DVec3 n = f.n;
        if (s && s->kind != BSurfaceKind::Plane) {
            double dd = 0;
            surfaceDistance(*s, q, dd, n);
            n = n * static_cast<double>(f.outward);
        }
        const auto i0 = static_cast<std::uint32_t>(m.vertices.size());
        m.vertices.push_back({q.f(), Vec3{}});
        m.vertices.push_back({(q + n * d).f(), Vec3{}});
        return idx[w] = {i0, i0 + 1};
    };
    for (int k : f.tris) {
        const auto& tr = t.tri[static_cast<std::size_t>(k)];
        const auto a = vertex(tr[0]), b = vertex(tr[1]), c = vertex(tr[2]);
        m.indices.insert(m.indices.end(), {a.second, b.second, c.second}); // the face where it goes
        m.indices.insert(m.indices.end(), {a.first, c.first, b.first});    // where it was, turned round
    }
    for (const BEdge& e : boundary(t, f.in)) {
        const auto a = vertex(e.a), b = vertex(e.b);
        m.indices.insert(m.indices.end(), {b.second, a.second, a.first});
        m.indices.insert(m.indices.end(), {b.second, a.first, b.first});
    }
    return m;
}

// The surface a face lies on, moved `d` along the face's outward normal; false when there is
// none (the radius would go).
bool offsetSurface(const Topo& t, const Face& f, double d, BSurface& out) {
    if (f.surf < 0) {
        if (!f.planar) return false;
        out = BSurface{};
        out.kind = BSurfaceKind::Plane;
        const DVec3 ref = std::fabs(f.n.z) < 0.9 ? DVec3{0, 0, 1} : DVec3{1, 0, 0};
        out.f = Frame::make(f.n * (f.c + d), f.n, ref);
        return true;
    }
    out = t.surfaces[static_cast<std::size_t>(f.surf)];
    const double delta = d * static_cast<double>(f.outward);
    switch (out.kind) {
    case BSurfaceKind::Plane: out.f.o = out.f.o + f.n * d; return true;
    case BSurfaceKind::Cylinder:
    case BSurfaceKind::Sphere: out.r1 += delta; return out.r1 > 1e-9;
    case BSurfaceKind::Cone: out.r1 += delta / std::cos(out.r2); return true;
    case BSurfaceKind::Torus: out.r2 += delta; return out.r2 > 1e-9;
    default: return false;
    }
}

// --- fillets that go with a flat face ---------------------------------------------------------

// The fillets along a flat face's edges: round faces (cylinders square to its normal) that meet
// it tangentially and otherwise only meet faces its move runs along. They move with the face,
// staying fillets. Their triangles in `carried`, their surfaces moved by d in `moved`.
void carriedFillets(const Topo& t, const Face& f, double d, std::vector<char>& carried, std::vector<BSurface>& moved) {
    carried.assign(t.tri.size(), 0);
    std::vector<char> seen(t.tri.size(), 0);
    for (const BEdge& e : boundary(t, f.in)) {
        if (e.across < 0 || seen[static_cast<std::size_t>(e.across)]) continue;
        const int s = t.surf[static_cast<std::size_t>(e.across)];
        if (s < 0) continue;
        const BSurface& S = t.surfaces[static_cast<std::size_t>(s)];
        if (S.kind != BSurfaceKind::Cylinder || std::fabs(ddot(S.f.z, f.n)) > 1e-6) continue;
        const Face g = growFace(t, e.across);
        for (int k : g.tris) seen[static_cast<std::size_t>(k)] = 1;
        // Tangent to the face where they meet.
        double dd = 0;
        DVec3 sn;
        const DVec3 mid = (t.p[static_cast<std::size_t>(e.a)] + t.p[static_cast<std::size_t>(e.b)]) * 0.5;
        if (!surfaceDistance(S, mid, dd, sn) || std::fabs(ddot(sn, f.n)) < 0.999) continue;
        // Its other neighbours: flat faces the move runs along.
        bool ok = true;
        for (const BEdge& o : boundary(t, g.in)) {
            if (o.across < 0) continue;
            const std::size_t k = static_cast<std::size_t>(o.across);
            if (f.in[k] || t.area[k] < 1e-9 * t.diag * t.diag) continue;
            const int os = t.surf[k];
            const DVec3 n = os >= 0 ? t.surfaces[static_cast<std::size_t>(os)].f.z : t.normal[k];
            if ((os >= 0 && t.surfaces[static_cast<std::size_t>(os)].kind != BSurfaceKind::Plane) || std::fabs(ddot(n, f.n)) > 1e-6) {
                ok = false;
                break;
            }
        }
        if (!ok) continue;
        for (int k : g.tris) carried[static_cast<std::size_t>(k)] = 1;
        BSurface m = S;
        m.f.o = m.f.o + f.n * d;
        moved.push_back(m);
    }
}

// --- push / pull of a flat face by moving its points ------------------------------------------

// A flat face moved by d with its neighbours meeting it: each corner on its rim slides along
// the neighbours it is on (planes; cylinders and cones along their lines). `carried`
// triangles (fillets along its edges) move with it. False when a neighbour isn't one of those
// (`unsupported`) or it would turn a face inside out.
bool tweakPlane(const Topo& t, const Face& f, double d, const std::vector<char>& carried, std::vector<DVec3>& p, bool& unsupported, std::string* why) {
    unsupported = false;
    p = t.p;
    std::vector<char> onFace(t.p.size(), 0), rim(t.p.size(), 0);
    const auto moving = [&](std::size_t k) { return f.in[k] || (!carried.empty() && carried[k]); };
    for (std::size_t k = 0; k < t.tri.size(); ++k)
        if (moving(k))
            for (int v : t.tri[k]) onFace[static_cast<std::size_t>(v)] = 1;
    // The neighbours each rim point is on: the triangles round it not in the face.
    std::vector<std::vector<int>> around(t.p.size());
    for (std::size_t k = 0; k < t.tri.size(); ++k) {
        if (moving(k)) continue;
        if (t.area[k] < 1e-9 * t.diag * t.diag) continue; // a sliver: no direction of its own
        for (int v : t.tri[k])
            if (onFace[static_cast<std::size_t>(v)]) rim[static_cast<std::size_t>(v)] = 1, around[static_cast<std::size_t>(v)].push_back(static_cast<int>(k));
    }
    for (std::size_t v = 0; v < t.p.size(); ++v) {
        if (!onFace[v]) continue;
        const DVec3 q = t.p[v];
        const double target = ddot(f.n, q) + d; // the face's plane moved (a fillet carried with it: its own level)
        if (!rim[v]) {
            p[v] = q + f.n * d;
            continue;
        }
        // Its neighbours: planes (n, c), and lines it may slide along (round neighbours).
        std::vector<std::pair<DVec3, double>> planes;
        std::vector<DVec3> lines;
        for (int k : around[v]) {
            const int s = t.surf[static_cast<std::size_t>(k)];
            if (s >= 0 && t.surfaces[static_cast<std::size_t>(s)].kind != BSurfaceKind::Plane) {
                const BSurface& S = t.surfaces[static_cast<std::size_t>(s)];
                DVec3 w;
                if (S.kind == BSurfaceKind::Cylinder) w = S.f.z;
                else if (S.kind == BSurfaceKind::Cone && std::fabs(std::tan(S.r2)) > 1e-9) w = dnormalize(q - (S.f.o - S.f.z * (S.r1 / std::tan(S.r2))));
                else {
                    unsupported = true;
                    return false;
                }
                if (std::none_of(lines.begin(), lines.end(), [&](const DVec3& l) { return std::fabs(ddot(l, w)) > 1 - 1e-9; })) lines.push_back(w);
                continue;
            }
            const DVec3 n = s >= 0 ? t.surfaces[static_cast<std::size_t>(s)].f.z : t.normal[static_cast<std::size_t>(k)];
            const double c = s >= 0 ? ddot(n, t.surfaces[static_cast<std::size_t>(s)].f.o) : ddot(n, t.p[static_cast<std::size_t>(t.tri[static_cast<std::size_t>(k)][0])]);
            if (std::none_of(planes.begin(), planes.end(), [&](const auto& pl) {
                    return std::fabs(std::fabs(ddot(pl.first, n)) - 1) < 1e-9 && std::fabs(ddot(pl.first, n) * c - pl.second) < t.tol * 10;
                }))
                planes.push_back({n, c});
        }
        // Where it goes: on the moved face and on every neighbour.
        if (!lines.empty()) {
            if (lines.size() > 1 || !planes.empty()) {
                unsupported = true; // a round neighbour meeting another: no straight way to slide
                return false;
            }
            const double along = ddot(f.n, lines[0]);
            if (std::fabs(along) < 0.05) {
                unsupported = true;
                return false;
            }
            p[v] = q + lines[0] * (d / along);
        } else if (planes.size() == 1) {
            const DVec3 nA = planes[0].first;
            const DVec3 w = dnormalize(dcross(nA, dcross(f.n, nA))); // in A, square to the edge
            const double along = ddot(f.n, w);
            if (std::fabs(along) < 0.05) {
                if (why) *why = "A neighbouring face is parallel to it";
                return false;
            }
            p[v] = q + w * (d / along);
        } else {
            // On the moved face and the neighbour planes: least squares, which must fit.
            std::vector<std::pair<DVec3, double>> all = planes;
            all.push_back({f.n, target});
            double M[3][3] = {}, b[3] = {};
            for (const auto& [n, c] : all) {
                const double nn[3] = {n.x, n.y, n.z};
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) M[i][j] += nn[i] * nn[j];
                    b[i] += nn[i] * c;
                }
            }
            const double det = M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
                               M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
            if (std::fabs(det) < 1e-12) {
                unsupported = true;
                return false;
            }
            auto solve = [&](int col) {
                double A[3][3];
                for (int i = 0; i < 3; ++i)
                    for (int j = 0; j < 3; ++j) A[i][j] = j == col ? b[i] : M[i][j];
                return (A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                        A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0])) / det;
            };
            const DVec3 x{solve(0), solve(1), solve(2)};
            for (const auto& [n, c] : all)
                if (std::fabs(ddot(n, x) - c) > t.tol * 50) {
                    unsupported = true; // more faces meet here than one point can stay on
                    return false;
                }
            p[v] = x;
        }
    }
    // No triangle turned over or squashed flat.
    for (std::size_t k = 0; k < t.tri.size(); ++k) {
        const auto& tr = t.tri[k];
        if (!onFace[static_cast<std::size_t>(tr[0])] && !onFace[static_cast<std::size_t>(tr[1])] && !onFace[static_cast<std::size_t>(tr[2])]) continue;
        const DVec3 e = dcross(p[static_cast<std::size_t>(tr[1])] - p[static_cast<std::size_t>(tr[0])], p[static_cast<std::size_t>(tr[2])] - p[static_cast<std::size_t>(tr[0])]);
        if (t.area[k] > 0 && (dlength(e) < 1e-12 * t.diag * t.diag || ddot(dnormalize(e), t.normal[k]) < 0.5)) {
            if (why) *why = "The face can't go that far: a neighbouring face would turn inside out";
            return false;
        }
    }
    return true;
}

// A round face made larger or smaller by moving its points along the surface's normal: where
// that keeps each point on its rim on the faces round it (flat faces square to a hole's or
// boss's axis), nothing else changes. False with `unsupported` when it doesn't.
bool tweakRound(const Topo& t, const Face& f, double d, std::vector<DVec3>& p, bool& unsupported, std::string* why) {
    unsupported = false;
    const BSurface& S = t.surfaces[static_cast<std::size_t>(f.surf)];
    p = t.p;
    std::vector<char> onFace(t.p.size(), 0);
    for (int k : f.tris)
        for (int v : t.tri[static_cast<std::size_t>(k)]) onFace[static_cast<std::size_t>(v)] = 1;
    for (std::size_t v = 0; v < t.p.size(); ++v) {
        if (!onFace[v]) continue;
        double dd = 0;
        DVec3 n;
        if (!surfaceDistance(S, t.p[v], dd, n)) {
            unsupported = true;
            return false;
        }
        p[v] = t.p[v] + n * (d * static_cast<double>(f.outward));
    }
    // Every face round it must still hold the points it shares with this one.
    for (std::size_t k = 0; k < t.tri.size(); ++k) {
        if (f.in[k] || t.area[k] < 1e-9 * t.diag * t.diag) continue;
        for (int v : t.tri[k]) {
            if (!onFace[static_cast<std::size_t>(v)]) continue;
            const int s = t.surf[k];
            double dd = 0;
            DVec3 n;
            const bool stays = s >= 0 ? surfaceDistance(t.surfaces[static_cast<std::size_t>(s)], p[static_cast<std::size_t>(v)], dd, n) && std::fabs(dd) < t.tol * 10
                                      : std::fabs(ddot(t.normal[k], p[static_cast<std::size_t>(v)] - t.p[static_cast<std::size_t>(v)])) < t.tol * 10;
            if (!stays) {
                unsupported = true;
                return false;
            }
        }
    }
    for (std::size_t k = 0; k < t.tri.size(); ++k) {
        const auto& tr = t.tri[k];
        if (!onFace[static_cast<std::size_t>(tr[0])] && !onFace[static_cast<std::size_t>(tr[1])] && !onFace[static_cast<std::size_t>(tr[2])]) continue;
        const DVec3 e = dcross(p[static_cast<std::size_t>(tr[1])] - p[static_cast<std::size_t>(tr[0])], p[static_cast<std::size_t>(tr[2])] - p[static_cast<std::size_t>(tr[0])]);
        if (t.area[k] > 1e-9 * t.diag * t.diag && (dlength(e) < 1e-12 * t.diag * t.diag || ddot(dnormalize(e), t.normal[k]) < 0.5)) {
            unsupported = true; // the faces round it are cut up in a way that doesn't stretch: another way
            return fail(why, "The face can't go that far: a face round it would turn inside out");
        }
    }
    return true;
}

// --- features: holes, bosses, fillets, chamfers -----------------------------------------------

// A round feature: a cylinder or cone face all the way round its axis, with the faces that
// close its end (a hole's bottom, a drill point, a boss's top).
struct Feature {
    std::vector<char> in;
    std::vector<int> faces; // the surfaces (indices) of its faces
    bool hole = false;
    DVec3 axisPoint, axis;
};

bool roundFeature(const Topo& t, const Face& f, Feature& out) {
    if (f.surf < 0) return false;
    const BSurface& S = t.surfaces[static_cast<std::size_t>(f.surf)];
    if (S.kind != BSurfaceKind::Cylinder && S.kind != BSurfaceKind::Cone) return false;
    // All the way round its axis (a fillet is only part of a cylinder).
    bool quarter[4] = {false, false, false, false};
    for (int k : f.tris)
        for (int v : t.tri[static_cast<std::size_t>(k)]) {
            const DVec3 q = t.p[static_cast<std::size_t>(v)] - S.f.o;
            const double ang = std::atan2(ddot(q, S.f.y), ddot(q, S.f.x));
            quarter[static_cast<int>(std::floor((ang + 3.14159265358979) / (3.14159265358979 / 2))) & 3] = true;
        }
    if (!(quarter[0] && quarter[1] && quarter[2] && quarter[3])) return false;
    out.in = f.in;
    out.faces = {f.surf};
    out.hole = f.outward < 0; // a hole: the face looks in at its axis
    out.axisPoint = S.f.o, out.axis = S.f.z;
    // Caps: whole faces all of whose rim borders the feature (grown faces, repeated).
    for (bool grew = true; grew;) {
        grew = false;
        std::vector<char> tried(t.tri.size(), 0);
        for (const BEdge& e : boundary(t, out.in)) {
            if (e.across < 0 || tried[static_cast<std::size_t>(e.across)]) continue;
            const Face cap = growFace(t, e.across);
            for (int k : cap.tris) tried[static_cast<std::size_t>(k)] = 1;
            bool closesIt = true;
            for (const BEdge& ce : boundary(t, cap.in))
                if (ce.across < 0 || !out.in[static_cast<std::size_t>(ce.across)]) closesIt = false;
            if (!closesIt || cap.area > 4 * f.area + 1e-9) continue;
            for (int k : cap.tris) out.in[static_cast<std::size_t>(k)] = 1;
            if (cap.surf >= 0) out.faces.push_back(cap.surf);
            grew = true;
        }
    }
    return true;
}

std::vector<DVec3> pointsOf(const Topo& t, const std::vector<char>& in) {
    std::set<int> v;
    for (std::size_t k = 0; k < t.tri.size(); ++k)
        if (in[k])
            for (int w : t.tri[k]) v.insert(w);
    std::vector<DVec3> out;
    for (int w : v) out.push_back(t.p[static_cast<std::size_t>(w)]);
    return out;
}

// Triangulates a simple flat polygon (counter-clockwise about n) by cutting ears: indices into it.
bool earClip(const std::vector<DVec3>& poly, const DVec3& n, std::vector<std::array<std::size_t, 3>>& out) {
    const DVec3 u = dnormalize(std::fabs(n.x) < 0.9 ? dcross(n, {1, 0, 0}) : dcross(n, {0, 1, 0})), v = dcross(n, u);
    std::vector<std::array<double, 2>> q;
    for (const DVec3& p : poly) q.push_back({ddot(p, u), ddot(p, v)});
    std::vector<std::size_t> idx(poly.size());
    for (std::size_t i = 0; i < idx.size(); ++i) idx[i] = i;
    auto cross2 = [&](std::size_t a, std::size_t b, std::size_t c) {
        return (q[b][0] - q[a][0]) * (q[c][1] - q[a][1]) - (q[c][0] - q[a][0]) * (q[b][1] - q[a][1]);
    };
    double scale = 0;
    for (const auto& r : q) scale = std::max({scale, std::fabs(r[0]), std::fabs(r[1])});
    const double eps = 1e-12 * std::max(scale * scale, 1.0);
    std::size_t guard = 0;
    while (idx.size() > 3 && guard++ < poly.size() * poly.size() + 10) {
        bool cut = false;
        // An ear: a convex corner with no other point inside it.
        for (std::size_t i = 0; i < idx.size() && !cut; ++i) {
            const std::size_t a = idx[(i + idx.size() - 1) % idx.size()], b = idx[i], c = idx[(i + 1) % idx.size()];
            if (cross2(a, b, c) <= eps) continue; // reflex, or in a straight line
            bool empty = true;
            for (std::size_t j : idx) {
                if (j == a || j == b || j == c) continue;
                if (cross2(a, b, j) > -eps && cross2(b, c, j) > -eps && cross2(c, a, j) > -eps) empty = false;
                if (!empty) break;
            }
            if (!empty) continue;
            out.push_back({a, b, c});
            idx.erase(idx.begin() + static_cast<std::ptrdiff_t>(i));
            cut = true;
        }
        // None: a point in a straight line between its neighbours goes, in a triangle of no area
        // (so the edges either side stay covered).
        for (std::size_t i = 0; i < idx.size() && !cut; ++i) {
            const std::size_t a = idx[(i + idx.size() - 1) % idx.size()], b = idx[i], c = idx[(i + 1) % idx.size()];
            if (std::fabs(cross2(a, b, c)) > eps) continue;
            out.push_back({a, b, c});
            idx.erase(idx.begin() + static_cast<std::ptrdiff_t>(i));
            cut = true;
        }
        if (!cut) return false;
    }
    if (idx.size() == 3) out.push_back({idx[0], idx[1], idx[2]});
    return true;
}

// The body's mesh without the triangles in `in`, its points moved to `p` (welded points), and
// every hole that leaves closed with a flat cap. Each cap's points and outward normal go to
// `caps`. False when a hole isn't flat (or isn't a simple loop).
struct Cap {
    std::vector<DVec3> pts;
    DVec3 n;
};
bool removeAndHeal(const Topo& t, const std::vector<char>& in, const std::vector<DVec3>& p, MeshData& out, std::vector<Cap>& caps, std::string* why) {
    // Points that met (a corner both faces now reach) become one.
    std::map<std::tuple<float, float, float>, int> at;
    std::vector<int> id(p.size());
    std::vector<DVec3> pos;
    for (std::size_t i = 0; i < p.size(); ++i) {
        const Vec3 f = p[i].f();
        const auto [it, added] = at.try_emplace({f.x, f.y, f.z}, static_cast<int>(pos.size()));
        if (added) pos.push_back(p[i]);
        id[i] = it->second;
    }
    const MeshData& m = *t.m;
    out = MeshData{};
    out.vertices = m.vertices;
    for (std::size_t i = 0; i < out.vertices.size(); ++i) out.vertices[i].position = p[static_cast<std::size_t>(t.weld[i])].f();
    std::map<std::pair<int, int>, int> directed;
    for (std::size_t k = 0; k < t.tri.size(); ++k) {
        if (in[k]) continue;
        const int a = id[static_cast<std::size_t>(t.tri[k][0])], b = id[static_cast<std::size_t>(t.tri[k][1])], c = id[static_cast<std::size_t>(t.tri[k][2])];
        if (a == b || b == c || c == a) continue; // squashed to nothing where faces met
        out.indices.insert(out.indices.end(), {m.indices[k * 3], m.indices[k * 3 + 1], m.indices[k * 3 + 2]});
        ++directed[{a, b}], ++directed[{b, c}], ++directed[{c, a}];
    }
    // The holes: edges with nothing on their other side, run the other way round.
    std::map<int, int> next;
    for (const auto& [e, n] : directed) {
        const auto back = directed.find({e.second, e.first});
        const int open = n - (back == directed.end() ? 0 : back->second);
        if (open <= 0) continue;
        if (open > 1 || next.count(e.second)) return fail(why, "The faces round it don't close up simply");
        next[e.second] = e.first;
    }
    std::set<int> used;
    for (const auto& [start, unused] : next) {
        if (used.count(start)) continue;
        std::vector<int> loop;
        for (int v = start; !used.count(v);) {
            used.insert(v);
            loop.push_back(v);
            const auto it = next.find(v);
            if (it == next.end()) return fail(why, "The faces round it don't close up simply");
            v = it->second;
        }
        if (loop.size() < 3) continue;
        Cap cap;
        for (int v : loop) cap.pts.push_back(pos[static_cast<std::size_t>(v)]);
        // Its plane (Newell), which every point must be on.
        DVec3 n{0, 0, 0}, c{0, 0, 0};
        for (std::size_t i = 0; i < cap.pts.size(); ++i) {
            const DVec3& a = cap.pts[i];
            const DVec3& b = cap.pts[(i + 1) % cap.pts.size()];
            n = n + DVec3{(a.y - b.y) * (a.z + b.z), (a.z - b.z) * (a.x + b.x), (a.x - b.x) * (a.y + b.y)};
            c = c + a;
        }
        cap.n = dnormalize(n);
        c = c / static_cast<double>(cap.pts.size());
        for (const DVec3& q : cap.pts)
            if (std::fabs(ddot(q - c, cap.n)) > t.tol * 200 + t.diag * 1e-4)
                return fail(why, "It meets a curved face: taking it away would leave a hole that no flat face closes");
        std::vector<std::array<std::size_t, 3>> tris;
        if (!earClip(cap.pts, cap.n, tris)) return fail(why, "The opening it leaves can't be closed with a flat face");
        const auto base = static_cast<std::uint32_t>(out.vertices.size());
        for (const DVec3& q : cap.pts) out.vertices.push_back({q.f(), cap.n.f()});
        for (const auto& tr : tris)
            out.indices.insert(out.indices.end(), {base + static_cast<std::uint32_t>(tr[0]), base + static_cast<std::uint32_t>(tr[1]), base + static_cast<std::uint32_t>(tr[2])});
        caps.push_back(std::move(cap));
    }
    // What other edits kept with the triangles no longer matches them.
    out.groups.clear(), out.layers.clear(), out.ids.clear(), out.quads.clear(), out.uvs.clear();
    out.recordLayer = -1;
    if (openEdgeCount(out) != 0) return fail(why, "Taking it away leaves the body open");
    out.computeFeatureEdges();
    return true;
}

// The body's mesh without the triangles in `in`, with flat polygons added (each running the
// way the removed triangles ran their shared edges); false unless that closes it.
bool replaceWith(const Topo& t, const std::vector<char>& in, const std::vector<std::vector<DVec3>>& polys, MeshData& out, std::string* why) {
    const MeshData& m = *t.m;
    out = MeshData{};
    out.vertices = m.vertices;
    for (std::size_t k = 0; k < t.tri.size(); ++k)
        if (!in[k]) out.indices.insert(out.indices.end(), {m.indices[k * 3], m.indices[k * 3 + 1], m.indices[k * 3 + 2]});
    for (const std::vector<DVec3>& poly : polys) {
        DVec3 n{0, 0, 0}, c{0, 0, 0};
        for (std::size_t i = 0; i < poly.size(); ++i) {
            const DVec3& a = poly[i];
            const DVec3& b = poly[(i + 1) % poly.size()];
            n = n + DVec3{(a.y - b.y) * (a.z + b.z), (a.z - b.z) * (a.x + b.x), (a.x - b.x) * (a.y + b.y)};
            c = c + a;
        }
        if (!(dlength(n) > 0)) continue; // no area: nothing to close
        n = dnormalize(n);
        c = c / static_cast<double>(poly.size());
        for (const DVec3& q : poly)
            if (std::fabs(ddot(q - c, n)) > t.tol * 200 + t.diag * 1e-4) return fail(why, "The faces round it don't meet in flat faces");
        std::vector<std::array<std::size_t, 3>> tris;
        if (!earClip(poly, n, tris)) return fail(why, "The opening it leaves can't be closed with flat faces");
        const auto base = static_cast<std::uint32_t>(out.vertices.size());
        for (const DVec3& q : poly) out.vertices.push_back({q.f(), n.f()});
        for (const auto& tr : tris)
            out.indices.insert(out.indices.end(), {base + static_cast<std::uint32_t>(tr[0]), base + static_cast<std::uint32_t>(tr[1]), base + static_cast<std::uint32_t>(tr[2])});
    }
    out.groups.clear(), out.layers.clear(), out.ids.clear(), out.quads.clear(), out.uvs.clear();
    out.recordLayer = -1;
    if (openEdgeCount(out) != 0) return fail(why, "Taking it away leaves the body open");
    out.computeFeatureEdges();
    return true;
}

// A fillet or chamfer taken off: the face goes, the two flat faces it ran between reach to
// the line where they meet, and its ends are closed in their end faces.
bool unblend(const Topo& t, const Face& f, MeshData& out, bool& added, std::string* why) {
    const char* notBlend = "It isn't a fillet or chamfer between two flat faces";
    // Its rim as one loop, the way its triangles run it.
    std::vector<BEdge> edges = boundary(t, f.in);
    std::map<int, std::size_t> from;
    for (std::size_t i = 0; i < edges.size(); ++i) {
        if (edges[i].across < 0 || from.count(edges[i].a)) return fail(why, notBlend);
        from[edges[i].a] = i;
    }
    std::vector<BEdge> loop;
    for (std::size_t i = 0, guard = 0; guard <= edges.size(); ++guard) {
        loop.push_back(edges[i]);
        const auto it = from.find(edges[i].b);
        if (it == from.end()) return fail(why, notBlend);
        i = it->second;
        if (i == 0) break;
    }
    if (loop.size() != edges.size()) return fail(why, notBlend); // more than one loop
    // The two faces it runs along: those with the most rim.
    std::vector<Face> nbFaces;
    std::vector<double> lengths;
    std::vector<int> whose(loop.size());
    for (std::size_t i = 0; i < loop.size(); ++i) {
        std::size_t which = nbFaces.size();
        for (std::size_t j = 0; j < nbFaces.size(); ++j)
            if (nbFaces[j].in[static_cast<std::size_t>(loop[i].across)]) which = j;
        if (which == nbFaces.size()) {
            nbFaces.push_back(growFace(t, loop[i].across));
            lengths.push_back(0);
        }
        lengths[which] += dlength(t.p[static_cast<std::size_t>(loop[i].a)] - t.p[static_cast<std::size_t>(loop[i].b)]);
        whose[i] = static_cast<int>(which);
    }
    if (nbFaces.size() < 3) return fail(why, notBlend);
    std::vector<std::size_t> order(nbFaces.size());
    for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](std::size_t x, std::size_t y) { return lengths[x] > lengths[y]; });
    const int ia = static_cast<int>(order[0]), ib = static_cast<int>(order[1]);
    const Face& A = nbFaces[static_cast<std::size_t>(ia)];
    const Face& B = nbFaces[static_cast<std::size_t>(ib)];
    if (!A.planar || !B.planar || std::fabs(ddot(A.n, B.n)) > 0.999) return fail(why, notBlend);
    // The rim in runs: along A, an end, along B, the other end.
    auto tag = [&](std::size_t i) { return whose[i] == ia ? 0 : whose[i] == ib ? 1 : 2; };
    std::size_t start = 0;
    while (start < loop.size() && !(tag(start) == 0 && tag((start + loop.size() - 1) % loop.size()) != 0)) ++start;
    if (start == loop.size()) return fail(why, notBlend);
    struct Run {
        int tag;
        std::vector<int> pts; // its points in order (first to last)
        int across;
    };
    std::vector<Run> runs;
    for (std::size_t k = 0; k < loop.size(); ++k) {
        const std::size_t i = (start + k) % loop.size();
        if (runs.empty() || runs.back().tag != tag(i)) runs.push_back({tag(i), {loop[i].a}, loop[i].across});
        runs.back().pts.push_back(loop[i].b);
    }
    if (runs.size() != 4 || runs[0].tag != 0 || runs[1].tag != 2 || runs[2].tag != 1 || runs[3].tag != 2) return fail(why, notBlend);
    // The line the two faces meet on, and where it goes through each end face.
    const DVec3 dir = dnormalize(dcross(A.n, B.n));
    const std::vector<DVec3> fp = pointsOf(t, f.in);
    DVec3 mid{0, 0, 0};
    for (const DVec3& q : fp) mid = mid + q;
    mid = mid / static_cast<double>(fp.size());
    const double det = ddot(A.n, dcross(B.n, dir));
    const DVec3 x = (dcross(B.n, dir) * A.c + dcross(dir, A.n) * B.c + dcross(A.n, B.n) * ddot(dir, mid)) / det;
    auto corner = [&](const Run& r) {
        const DVec3 nE = t.normal[static_cast<std::size_t>(r.across)];
        const DVec3 q = t.p[static_cast<std::size_t>(r.pts.front())];
        const double along = ddot(nE, dir);
        return std::fabs(along) < 1e-9 ? x + dir * ddot(q - x, dir) : x + dir * (ddot(nE, q - x) / along);
    };
    const DVec3 c1 = corner(runs[1]), c3 = corner(runs[3]);
    auto pts = [&](const Run& r) {
        std::vector<DVec3> v;
        for (int w : r.pts) v.push_back(t.p[static_cast<std::size_t>(w)]);
        return v;
    };
    std::vector<DVec3> polyA = pts(runs[0]);
    polyA.push_back(c1), polyA.push_back(c3);
    std::vector<DVec3> polyB = pts(runs[2]);
    polyB.push_back(c3), polyB.push_back(c1);
    std::vector<DVec3> end1 = pts(runs[1]);
    end1.push_back(c1);
    std::vector<DVec3> end3 = pts(runs[3]);
    end3.push_back(c3);
    if (!replaceWith(t, f.in, {polyA, polyB, end1, end3}, out, why)) return false;
    // Material added where the corner is outside the body (a rounded outside edge).
    DVec3 nOut = f.n;
    if (!f.planar && f.surf >= 0) {
        double d = 0;
        surfaceDistance(t.surfaces[static_cast<std::size_t>(f.surf)], mid, d, nOut);
        nOut = nOut * static_cast<double>(f.outward);
    }
    added = ddot(x + dir * ddot(mid - x, dir) - mid, nOut) > 0;
    return true;
}

// A round feature as a solid: turned about its axis from its widest radius at each height,
// its open ends (where `caps` closed it) carried `reach` further out.
MeshData featureSolid(const Topo& t, const Feature& feat, const std::vector<Cap>& caps, double reach, double grow = 0) {
    const DVec3 o = feat.axisPoint, z = dnormalize(feat.axis);
    const DVec3 x = dnormalize(std::fabs(z.x) < 0.9 ? dcross(z, {1, 0, 0}) : dcross(z, {0, 1, 0})), y = dcross(z, x);
    const double q = std::max(t.tol * 10, 1e-9);
    std::map<long long, double> widest; // height (in steps of q) -> radius
    for (const DVec3& p : pointsOf(t, feat.in)) {
        const double h = ddot(p - o, z);
        const double r = dlength(p - o - z * h);
        double& w = widest[std::llround(h / q)];
        w = std::max(w, r);
    }
    std::vector<std::pair<double, double>> prof; // (height, radius), bottom to top
    for (const auto& [k, r] : widest) prof.push_back({static_cast<double>(k) * q, r > q ? std::max(r + grow, 0.0) : r});
    if (prof.size() < 2) return {};
    // Open ends go further out: the end a cap is at.
    for (const Cap& c : caps) {
        DVec3 m{0, 0, 0};
        for (const DVec3& p : c.pts) m = m + p;
        m = m / static_cast<double>(c.pts.size());
        const double h = ddot(m - o, z);
        if (std::fabs(h - prof.front().first) < std::fabs(h - prof.back().first)) prof.insert(prof.begin(), {prof.front().first - reach, prof.front().second});
        else prof.push_back({prof.back().first + reach, prof.back().second});
    }
    const int seg = 64;
    MeshData m;
    std::vector<std::uint32_t> ring(prof.size());
    for (std::size_t i = 0; i < prof.size(); ++i) {
        ring[i] = static_cast<std::uint32_t>(m.vertices.size());
        const double r = prof[i].second;
        if (r < q) {
            m.vertices.push_back({(o + z * prof[i].first).f(), Vec3{}});
            continue;
        }
        for (int k = 0; k < seg; ++k) {
            const double a = 2 * 3.14159265358979 * k / seg;
            m.vertices.push_back({(o + z * prof[i].first + x * (r * std::cos(a)) + y * (r * std::sin(a))).f(), Vec3{}});
        }
    }
    auto at = [&](std::size_t i, int k) { return prof[i].second < q ? ring[i] : ring[i] + static_cast<std::uint32_t>(k % seg); };
    for (std::size_t i = 0; i + 1 < prof.size(); ++i)
        for (int k = 0; k < seg; ++k) {
            const std::uint32_t a = at(i, k), b = at(i, k + 1), c = at(i + 1, k + 1), d = at(i + 1, k);
            if (a != b) m.indices.insert(m.indices.end(), {a, b, c});
            if (c != d) m.indices.insert(m.indices.end(), {a, c, d});
        }
    // The ends, where they are discs.
    for (std::size_t i : {std::size_t{0}, prof.size() - 1}) {
        if (prof[i].second < q) continue;
        const auto centre = static_cast<std::uint32_t>(m.vertices.size());
        m.vertices.push_back({(o + z * prof[i].first).f(), Vec3{}});
        for (int k = 0; k < seg; ++k) {
            if (i == 0) m.indices.insert(m.indices.end(), {centre, at(i, k + 1), at(i, k)});
            else m.indices.insert(m.indices.end(), {centre, at(i, k), at(i, k + 1)});
        }
    }
    return m;
}

} // namespace

const char* directFaceKindName(DirectFaceKind k) {
    switch (k) {
    case DirectFaceKind::Plane: return "flat";
    case DirectFaceKind::Cylinder: return "cylindrical";
    case DirectFaceKind::Cone: return "conical";
    case DirectFaceKind::Sphere: return "spherical";
    case DirectFaceKind::Torus: return "toroidal";
    case DirectFaceKind::Freeform: return "freeform";
    }
    return "";
}

bool directFaceAt(const Body& body, const Vec3& world, DirectFaceInfo& out, std::string* why) {
    Topo t;
    Face f;
    if (!faceAt(body, world, t, f, why)) return false;
    out = DirectFaceInfo{};
    out.kind = kindOf(t, f);
    out.exact = f.surf >= 0;
    out.area = f.area;
    out.triangles = f.tris.size();
    const DVec3 off(body.offset);
    if (f.planar) {
        out.normal = f.n;
        out.point = f.n * f.c + off;
    } else if (f.surf >= 0) {
        const BSurface& S = t.surfaces[static_cast<std::size_t>(f.surf)];
        out.point = S.f.o + off;
        out.axis = S.f.z;
        out.radius = S.kind == BSurfaceKind::Torus ? S.r2 : S.r1;
        out.hole = f.outward < 0;
    }
    return true;
}

bool directOffsetFace(Body& body, const Vec3& world, double distance, std::string* why, std::string* how) {
    if (!std::isfinite(distance) || std::fabs(distance) < 1e-9) return fail(why, "Give a distance");
    Topo t;
    Face f;
    if (!faceAt(body, world, t, f, why)) return false;
    const DirectFaceKind kind = kindOf(t, f);
    if (kind == DirectFaceKind::Freeform) return fail(why, "Only flat faces, and round faces of exact solids (cylinders, cones, spheres, tori), can be offset");
    std::vector<BSurface> surfaces = t.surfaces;
    BSurface moved;
    if (offsetSurface(t, f, distance, moved)) surfaces.push_back(moved);
    else if (f.surf >= 0) return fail(why, "The face would shrink to nothing");
    // A face that is the whole body (a sphere, a torus): every point moves along its normal.
    const std::vector<BEdge> rim = boundary(t, f.in);
    if (!f.planar && std::none_of(rim.begin(), rim.end(), [](const BEdge& e) { return e.across >= 0; })) {
        const BSurface& S = t.surfaces[static_cast<std::size_t>(f.surf)];
        std::vector<DVec3> p = t.p;
        std::vector<char> done(p.size(), 0);
        for (int k : f.tris)
            for (int v : t.tri[static_cast<std::size_t>(k)]) {
                if (done[static_cast<std::size_t>(v)]) continue;
                done[static_cast<std::size_t>(v)] = 1;
                double dd = 0;
                DVec3 n;
                if (!surfaceDistance(S, t.p[static_cast<std::size_t>(v)], dd, n)) continue; // on the axis: a pole stays put along it
                p[static_cast<std::size_t>(v)] = t.p[static_cast<std::size_t>(v)] + n * (distance * static_cast<double>(f.outward));
            }
        finish(body, withPoints(t, p), {moved});
        // A whole sphere or torus (no edges to rebuild it from): its exact solid made afresh.
        if (!body.exact()) {
            std::shared_ptr<Brep> exact;
            if (moved.kind == BSurfaceKind::Sphere) exact = brepSphere(moved.f.o, moved.r1);
            else if (moved.kind == BSurfaceKind::Torus && std::fabs(std::fabs(moved.f.z.z) - 1) < 1e-12) exact = brepTorus(moved.f.o, moved.r1, moved.r2);
            if (exact) body.brep = exact, body.brepKey = meshKey(body.mesh);
        }
        if (how) *how = std::string("round face ") + (S.kind == BSurfaceKind::Torus ? "tube radius " + num(S.r2) + " -> " + num(moved.r2) : "radius " + num(S.r1) + " -> " + num(moved.r1)) + " mm";
        return true;
    }
    if (f.planar) {
        std::vector<DVec3> p;
        bool unsupported = false;
        std::string whyNot;
        std::vector<char> carried;
        std::vector<BSurface> withFillets = surfaces;
        carriedFillets(t, f, distance, carried, withFillets);
        const bool fillets = std::find(carried.begin(), carried.end(), 1) != carried.end();
        if (fillets && tweakPlane(t, f, distance, carried, p, unsupported, &whyNot)) {
            finish(body, withPoints(t, p), withFillets);
            if (how) *how = "moved the flat face " + num(distance) + " mm with the fillets along its edges; its neighbours follow it";
            return true;
        }
        if (tweakPlane(t, f, distance, {}, p, unsupported, &whyNot)) {
            finish(body, withPoints(t, p), surfaces);
            if (how) *how = "moved the flat face " + num(distance) + " mm; its neighbours follow it";
            return true;
        }
        if (!unsupported) return fail(why, whyNot);
        // Neighbours it can't slide along: the slab between where it is and where it goes.
    } else {
        std::vector<DVec3> p;
        bool unsupported = false;
        std::string whyNot;
        if (tweakRound(t, f, distance, p, unsupported, &whyNot)) {
            finish(body, withPoints(t, p), surfaces);
            if (how) {
                const BSurface& S = t.surfaces[static_cast<std::size_t>(f.surf)];
                *how = std::string(f.outward < 0 ? "hole" : "round face") + " " +
                       (S.kind == BSurfaceKind::Torus ? "tube radius " + num(S.r2) + " -> " + num(moved.r2) : "radius " + num(S.r1) + " -> " + num(moved.r1)) + " mm";
            }
            return true;
        }
        if (!unsupported) return fail(why, whyNot);
        // A whole hole or boss with a straight wall: made again at its new size.
        Feature feat;
        if (t.surfaces[static_cast<std::size_t>(f.surf)].kind == BSurfaceKind::Cylinder && roundFeature(t, f, feat)) {
            const double grow = moved.r1 - t.surfaces[static_cast<std::size_t>(f.surf)].r1;
            MeshData healed;
            std::vector<Cap> caps;
            if (!removeAndHeal(t, feat.in, t.p, healed, caps, why)) return false;
            MeshData tool = featureSolid(t, feat, caps, std::max(0.5, 0.05 * t.diag), grow);
            if (tool.indices.empty()) return fail(why, "The face would shrink to nothing");
            Body work = body;
            finish(work, std::move(healed), t.surfaces);
            if (!applyTool(work, std::move(tool), !feat.hole, surfaces, why)) return false;
            body = std::move(work);
            if (how) *how = std::string(feat.hole ? "hole" : "boss") + " radius " + num(t.surfaces[static_cast<std::size_t>(f.surf)].r1) + " -> " + num(moved.r1) + " mm";
            return true;
        }
    }
    if (!applyTool(body, slab(t, f, distance), distance > 0, surfaces, why)) return false;
    if (how) {
        if (f.planar) *how = std::string(distance > 0 ? "added" : "cut") + " the " + num(std::fabs(distance)) + " mm slab under the flat face";
        else {
            const BSurface& S = t.surfaces[static_cast<std::size_t>(f.surf)];
            *how = std::string(f.outward < 0 ? "hole" : "round face") + " " +
                   (S.kind == BSurfaceKind::Torus ? "tube radius " + num(S.r2) + " -> " + num(moved.r2) : "radius " + num(S.r1) + " -> " + num(moved.r1)) + " mm";
        }
    }
    return true;
}

bool directDeleteFace(Body& body, const Vec3& world, std::string* why, std::string* how) {
    Topo t;
    Face f;
    if (!faceAt(body, world, t, f, why)) return false;
    const double before = meshVolume(body.mesh);
    MeshData healed;
    Feature feat;
    if (roundFeature(t, f, feat)) {
        std::vector<Cap> caps;
        if (!removeAndHeal(t, feat.in, t.p, healed, caps, why)) return false;
        if (how) *how = feat.hole ? "filled the hole" : "took away the boss";
    } else {
        bool added = false;
        std::string whyNot;
        if (!unblend(t, f, healed, added, &whyNot))
            return fail(why, f.planar && whyNot.find("isn't a fillet") != std::string::npos
                                 ? "That face can't be deleted: holes, bosses, fillets and chamfers can (a flat face can be offset instead)"
                                 : whyNot);
        if (how) *how = std::string(f.planar ? "took off the chamfer" : "took off the fillet") + (added ? "" : " (an inside corner)") + ": the faces meet again";
    }
    const double after = meshVolume(healed);
    if (!(after > 0) || std::fabs(after - before) < 1e-9 * std::max(before, 1.0)) return fail(why, "That did not give a clean solid; " + body.name + " is unchanged");
    finish(body, std::move(healed), t.surfaces);
    return true;
}

bool directMoveFace(Body& body, const Vec3& world, const DVec3& by, std::string* why, std::string* how) {
    if (!(dlength(by) > 1e-9)) return fail(why, "Give how far: dx,dy,dz");
    Topo t;
    Face f;
    if (!faceAt(body, world, t, f, why)) return false;
    if (f.planar) {
        const double d = ddot(by, f.n);
        if (std::fabs(d) < 1e-9) return fail(why, "A flat face moves along its normal: that move is along the face");
        if (!directOffsetFace(body, world, d, why, how)) return false;
        if (how && dlength(by - f.n * d) > 1e-6) *how += " (the part of the move along the face can't apply)";
        return true;
    }
    Feature feat;
    if (!roundFeature(t, f, feat)) return fail(why, "Only holes, bosses and flat faces can be moved");
    MeshData healed;
    std::vector<Cap> caps;
    if (!removeAndHeal(t, feat.in, t.p, healed, caps, why)) return false;
    // The feature where it goes, reaching past its open ends (out of the body for a hole,
    // into it for a boss) so the cut or join meets no face flush.
    MeshData tool = featureSolid(t, feat, caps, std::max(0.5, 0.05 * t.diag));
    if (tool.indices.empty()) return fail(why, "The feature has no volume");
    for (Vertex& v : tool.vertices) v.position = (DVec3(v.position) + by).f();
    // Its surfaces where it goes, so the body stays exact.
    std::vector<BSurface> surfaces = t.surfaces;
    for (int s : feat.faces) {
        BSurface m = t.surfaces[static_cast<std::size_t>(s)];
        m.f.o = m.f.o + by;
        surfaces.push_back(m);
    }
    Body work = body;
    finish(work, std::move(healed), t.surfaces);
    if (!applyTool(work, tool, !feat.hole, surfaces, why)) return false;
    body = std::move(work);
    if (how) *how = std::string("moved the ") + (feat.hole ? "hole" : "boss") + " by (" + num(by.x) + ", " + num(by.y) + ", " + num(by.z) + ") mm";
    return true;
}

} // namespace sicdm

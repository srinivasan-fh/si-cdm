#include "core/Wearable.hpp"

#include "core/Motion.hpp"
#include "core/Physics.hpp"
#include "core/Scene.hpp"
#include "core/Sculpt.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <unordered_map>

namespace sicdm {

namespace {

// A triangle mesh with shared vertices.
struct WMesh {
    std::vector<Vec3> p;
    std::vector<std::array<std::uint32_t, 3>> t;
};

std::uint64_t ekey(std::uint32_t a, std::uint32_t b) {
    if (a > b) std::swap(a, b);
    return (static_cast<std::uint64_t>(a) << 32) | b;
}

// Angle-weighted vertex normals and, per vertex, how far to move so every face
// around it moves by one unit (1 / cos of the widest angle to a face, capped).
void offsetDirections(const WMesh& w, std::vector<Vec3>& dir) {
    std::vector<Vec3> n(w.p.size(), Vec3{});
    std::vector<Vec3> fn(w.t.size());
    for (std::size_t i = 0; i < w.t.size(); ++i) {
        const auto& t = w.t[i];
        fn[i] = normalize(cross(w.p[t[1]] - w.p[t[0]], w.p[t[2]] - w.p[t[0]]));
        for (int k = 0; k < 3; ++k) {
            const Vec3 e1 = normalize(w.p[t[static_cast<std::size_t>((k + 1) % 3)]] - w.p[t[static_cast<std::size_t>(k)]]);
            const Vec3 e2 = normalize(w.p[t[static_cast<std::size_t>((k + 2) % 3)]] - w.p[t[static_cast<std::size_t>(k)]]);
            const float angle = std::acos(std::clamp(dot(e1, e2), -1.0f, 1.0f));
            n[t[static_cast<std::size_t>(k)]] += fn[i] * angle;
        }
    }
    for (Vec3& v : n) v = normalize(v);
    std::vector<float> minCos(w.p.size(), 1.0f);
    for (std::size_t i = 0; i < w.t.size(); ++i)
        for (std::uint32_t v : w.t[i]) minCos[v] = std::min(minCos[v], dot(n[v], fn[i]));
    dir.resize(w.p.size());
    for (std::size_t v = 0; v < w.p.size(); ++v) dir[v] = n[v] * (1.0f / std::max(minCos[v], 0.5f));
}

// A closed solid between the surfaces `outer` and `inner` (same topology as w):
// outer faces as they are, inner faces reversed, open borders bridged.
MeshData shellBetween(const WMesh& w, const std::vector<Vec3>& outer, const std::vector<Vec3>& inner) {
    MeshData m;
    const auto n = static_cast<std::uint32_t>(w.p.size());
    auto put = [&](const Vec3& a, const Vec3& b, const Vec3& c) {
        const Vec3 nn = normalize(cross(b - a, c - a));
        const auto at = static_cast<std::uint32_t>(m.vertices.size());
        m.vertices.push_back({a, nn});
        m.vertices.push_back({b, nn});
        m.vertices.push_back({c, nn});
        m.indices.insert(m.indices.end(), {at, at + 1, at + 2});
    };
    (void)n;
    std::unordered_map<std::uint64_t, int> count;
    for (const auto& t : w.t)
        for (int k = 0; k < 3; ++k) ++count[ekey(t[static_cast<std::size_t>(k)], t[static_cast<std::size_t>((k + 1) % 3)])];
    for (const auto& t : w.t) {
        put(outer[t[0]], outer[t[1]], outer[t[2]]);
        put(inner[t[0]], inner[t[2]], inner[t[1]]);
        for (int k = 0; k < 3; ++k) { // a border edge a -> b: a rim strip down to the inner surface
            const std::uint32_t a = t[static_cast<std::size_t>(k)], b = t[static_cast<std::size_t>((k + 1) % 3)];
            if (count[ekey(a, b)] != 1) continue;
            put(outer[b], outer[a], inner[a]);
            put(outer[b], inner[a], inner[b]);
        }
    }
    return m;
}

// Triangles of one body (its own frame) in a uniform grid for nearby queries.
struct TriGrid {
    std::vector<std::array<Vec3, 3>> tris;
    std::vector<Vec3> normals;
    std::vector<AABB> boxes; // per triangle, for a cheap rejection
    AABB box;
    float cell = 1.0f;
    int nx = 1, ny = 1, nz = 1;
    std::vector<std::vector<std::uint32_t>> cells;

    void build(const MeshData& m) {
        for (std::size_t k = 0; k + 2 < m.indices.size(); k += 3) {
            const std::array<Vec3, 3> t{m.vertices[m.indices[k]].position, m.vertices[m.indices[k + 1]].position,
                                        m.vertices[m.indices[k + 2]].position};
            tris.push_back(t);
            normals.push_back(normalize(cross(t[1] - t[0], t[2] - t[0])));
            AABB tb;
            for (const Vec3& v : t) tb.expand(v);
            boxes.push_back(tb);
            box.expand(tb.min);
            box.expand(tb.max);
        }
        if (tris.empty()) return;
        const Vec3 ext = box.max - box.min;
        const float vol = std::max(ext.x, 1e-3f) * std::max(ext.y, 1e-3f) * std::max(ext.z, 1e-3f);
        cell = std::max(std::cbrt(vol / static_cast<float>(tris.size()) * 4.0f), 1e-3f);
        nx = std::clamp(static_cast<int>(ext.x / cell) + 1, 1, 128);
        ny = std::clamp(static_cast<int>(ext.y / cell) + 1, 1, 128);
        nz = std::clamp(static_cast<int>(ext.z / cell) + 1, 1, 128);
        cells.assign(static_cast<std::size_t>(nx * ny * nz), {});
        for (std::size_t i = 0; i < tris.size(); ++i) {
            const AABB& tb = boxes[i];
            int lo[3], hi[3];
            index(tb.min, lo);
            index(tb.max, hi);
            for (int x = lo[0]; x <= hi[0]; ++x)
                for (int y = lo[1]; y <= hi[1]; ++y)
                    for (int z = lo[2]; z <= hi[2]; ++z) cells[static_cast<std::size_t>((x * ny + y) * nz + z)].push_back(static_cast<std::uint32_t>(i));
        }
    }
    void index(const Vec3& p, int out[3]) const {
        const Vec3 ext = box.max - box.min;
        const float f[3] = {ext.x > 0 ? (p.x - box.min.x) / ext.x * static_cast<float>(nx) : 0.0f,
                            ext.y > 0 ? (p.y - box.min.y) / ext.y * static_cast<float>(ny) : 0.0f,
                            ext.z > 0 ? (p.z - box.min.z) / ext.z * static_cast<float>(nz) : 0.0f};
        const int n[3] = {nx, ny, nz};
        for (int i = 0; i < 3; ++i) out[i] = std::clamp(static_cast<int>(std::floor(f[i])), 0, n[i] - 1);
    }
    // Distance to the surface if within `reach`, else nullopt (and the side of the
    // nearest face the point is on: false behind it).
    std::optional<float> nearest(const Vec3& q, float reach, bool* front = nullptr) const;
};

Vec3 closestOnTri(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c) {
    const Vec3 ab = b - a, ac = c - a, ap = p - a;
    const float d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0 && d2 <= 0) return a;
    const Vec3 bp = p - b;
    const float d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0 && d4 <= d3) return b;
    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) return a + ab * (d1 / (d1 - d3));
    const Vec3 cp = p - c;
    const float d5 = dot(ab, cp), d6 = dot(ac, cp);
    if (d6 >= 0 && d5 <= d6) return c;
    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) return a + ac * (d2 / (d2 - d6));
    const float va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    const float denom = 1.0f / (va + vb + vc);
    return a + ab * (vb * denom) + ac * (vc * denom);
}

std::optional<float> TriGrid::nearest(const Vec3& q, float reach, bool* front) const {
    if (tris.empty()) return std::nullopt;
    if (q.x < box.min.x - reach || q.y < box.min.y - reach || q.z < box.min.z - reach || q.x > box.max.x + reach ||
        q.y > box.max.y + reach || q.z > box.max.z + reach)
        return std::nullopt;
    float best = reach * reach;
    bool found = false;
    Vec3 bestD;
    std::uint32_t bestI = 0;
    // Outwards from the query: a box of half-size r holds every triangle closer
    // than r, so a hit that close is the nearest; else the box doubles.
    for (float r = std::min(cell, reach);; r = std::min(r * 2.0f, reach)) {
        int lo[3], hi[3];
        index(q - Vec3{r, r, r}, lo);
        index(q + Vec3{r, r, r}, hi);
        for (int x = lo[0]; x <= hi[0]; ++x)
            for (int y = lo[1]; y <= hi[1]; ++y)
                for (int z = lo[2]; z <= hi[2]; ++z)
                    for (std::uint32_t i : cells[static_cast<std::size_t>((x * ny + y) * nz + z)]) {
                        const AABB& b = boxes[i];
                        const float ex = std::max({b.min.x - q.x, 0.0f, q.x - b.max.x}), ey = std::max({b.min.y - q.y, 0.0f, q.y - b.max.y}),
                                    ez = std::max({b.min.z - q.z, 0.0f, q.z - b.max.z});
                        if (ex * ex + ey * ey + ez * ez >= best) continue;
                        const Vec3 d = q - closestOnTri(q, tris[i][0], tris[i][1], tris[i][2]);
                        const float d2 = dot(d, d);
                        if (d2 < best) {
                            best = d2;
                            found = true;
                            bestD = d;
                            bestI = i;
                        }
                    }
        if ((found && best <= r * r) || r >= reach) break;
    }
    if (!found) return std::nullopt;
    if (front) *front = dot(bestD, normals[bestI]) >= 0;
    return std::sqrt(best);
}

} // namespace

// The surface f = 0 of a field sampled at the centres of a voxel grid (marching
// tetrahedra, closed wherever the field is), triangles facing the side where
// f > 0 (`facePositive`) or f < 0.
MeshData marchTets(const std::vector<float>& f, int nx, int ny, int nz, const Vec3& origin, float h, bool facePositive) {
    auto idx = [&](int x, int y, int z) {
        return (static_cast<std::size_t>(x) * static_cast<std::size_t>(ny) + static_cast<std::size_t>(y)) * static_cast<std::size_t>(nz) +
               static_cast<std::size_t>(z);
    };
    auto at = [&](int x, int y, int z) {
        return origin + Vec3{(static_cast<float>(x) + 0.5f) * h, (static_cast<float>(y) + 0.5f) * h, (static_cast<float>(z) + 0.5f) * h};
    };
    MeshData out;
    std::unordered_map<std::uint64_t, std::uint32_t> edgeVertex;
    auto vertexOn = [&](std::size_t ia, std::size_t ib, const Vec3& pa, const Vec3& pb, float fa, float fb) {
        const std::uint64_t key = ia < ib ? (static_cast<std::uint64_t>(ia) << 32 | ib) : (static_cast<std::uint64_t>(ib) << 32 | ia);
        const auto [it, fresh] = edgeVertex.try_emplace(key, static_cast<std::uint32_t>(out.vertices.size()));
        if (fresh) out.vertices.push_back({pa + (pb - pa) * (fa / (fa - fb)), {}});
        return it->second;
    };
    static const int kCorner[8][3] = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}};
    static const int kTets[6][4] = {{0, 5, 1, 6}, {0, 1, 2, 6}, {0, 2, 3, 6}, {0, 3, 7, 6}, {0, 7, 4, 6}, {0, 4, 5, 6}};
    for (int x = 0; x + 1 < nx; ++x)
        for (int y = 0; y + 1 < ny; ++y)
            for (int z = 0; z + 1 < nz; ++z) {
                std::size_t ci[8];
                float cv[8];
                Vec3 cp[8];
                bool anyIn = false, anyOut = false;
                for (int c = 0; c < 8; ++c) {
                    ci[c] = idx(x + kCorner[c][0], y + kCorner[c][1], z + kCorner[c][2]);
                    cv[c] = f[ci[c]];
                    cp[c] = at(x + kCorner[c][0], y + kCorner[c][1], z + kCorner[c][2]);
                    (cv[c] > 0 ? anyIn : anyOut) = true;
                }
                if (!anyIn || !anyOut) continue;
                for (const auto& tet : kTets) {
                    int pos[4], neg[4], np = 0, nn = 0;
                    for (int q : tet) (cv[q] > 0 ? pos[np++] : neg[nn++]) = q;
                    if (np == 0 || nn == 0) continue;
                    auto vtx = [&](int a, int b) { return vertexOn(ci[a], ci[b], cp[a], cp[b], cv[a], cv[b]); };
                    Vec3 cIn{}, cOut{};
                    for (int q = 0; q < np; ++q) cIn += cp[pos[q]];
                    for (int q = 0; q < nn; ++q) cOut += cp[neg[q]];
                    Vec3 facing = cIn / static_cast<float>(np) - cOut / static_cast<float>(nn);
                    if (!facePositive) facing = -facing;
                    auto tri = [&](std::uint32_t a, std::uint32_t b, std::uint32_t c) {
                        const Vec3 n = cross(out.vertices[b].position - out.vertices[a].position,
                                             out.vertices[c].position - out.vertices[a].position);
                        if (dot(n, facing) < 0) std::swap(b, c);
                        out.indices.insert(out.indices.end(), {a, b, c});
                    };
                    if (np == 1 || nn == 1) {
                        const int lone = np == 1 ? pos[0] : neg[0];
                        const int* others = np == 1 ? neg : pos;
                        tri(vtx(lone, others[0]), vtx(lone, others[1]), vtx(lone, others[2]));
                    } else {
                        const std::uint32_t a = vtx(pos[0], neg[0]), b = vtx(pos[0], neg[1]), c = vtx(pos[1], neg[1]), d = vtx(pos[1], neg[0]);
                        tri(a, b, c);
                        tri(a, c, d);
                    }
                }
            }
    return out;
}

namespace {

// Taubin smoothing (alternating shrink / inflate steps): removes voxel terraces
// without shrinking the surface.
void taubin(const WMesh& w, std::vector<Vec3>& q, int iterations) {
    std::vector<Vec3> sum(q.size());
    std::vector<float> cnt(q.size());
    for (int it = 0; it < iterations * 2; ++it) {
        const float k = it % 2 == 0 ? 0.5f : -0.53f;
        std::fill(sum.begin(), sum.end(), Vec3{});
        std::fill(cnt.begin(), cnt.end(), 0.0f);
        for (const auto& t : w.t)
            for (int c = 0; c < 3; ++c) {
                sum[t[static_cast<std::size_t>(c)]] += q[t[static_cast<std::size_t>((c + 1) % 3)]] + q[t[static_cast<std::size_t>((c + 2) % 3)]];
                cnt[t[static_cast<std::size_t>(c)]] += 2.0f;
            }
        for (std::size_t v = 0; v < q.size(); ++v)
            if (cnt[v] > 0) q[v] += (sum[v] / cnt[v] - q[v]) * k;
    }
}

} // namespace

// Smooths a marched surface and gives it vertex normals.
void finishMarched(MeshData& m, int iterations) {
    WMesh w;
    w.p.resize(m.vertices.size());
    for (std::size_t i = 0; i < w.p.size(); ++i) w.p[i] = m.vertices[i].position;
    for (std::size_t q = 0; q + 2 < m.indices.size(); q += 3) w.t.push_back({m.indices[q], m.indices[q + 1], m.indices[q + 2]});
    std::vector<Vec3> p = w.p;
    taubin(w, p, iterations);
    for (std::size_t i = 0; i < p.size(); ++i) m.vertices[i] = {p[i], {}};
    for (std::size_t q = 0; q + 2 < m.indices.size(); q += 3) {
        const Vec3 n = cross(p[m.indices[q + 1]] - p[m.indices[q]], p[m.indices[q + 2]] - p[m.indices[q]]);
        for (int c = 0; c < 3; ++c) m.vertices[m.indices[q + static_cast<std::size_t>(c)]].normal += n;
    }
    for (Vertex& v : m.vertices) v.normal = normalize(v.normal);
}

namespace {

// Writes a world-space mesh into the body, with auto-smoothed normals and the edges
// sharper than `edgeAngle` (marched surfaces only show their real creases).
void store(Body& body, MeshData world, float edgeAngle = 40.0f) {
    body.offset = Vec3{};
    body.mesh = std::move(world);
    body.kind = "mesh";
    body.params.clear();
    SculptMesh(body).writeTo(body, false, std::max(edgeAngle, 40.0f) + 10.0f);
    body.mesh.computeFeatureEdges(edgeAngle);
}

} // namespace

// ---------------------------------------------------------------------------
// Shell

bool shellBody(Body& body, const ShellOptions& o, std::string* error) {
    if (!(o.thickness > 0)) {
        if (error) *error = "The shell needs a thickness";
        return false;
    }
    const Vec3 keepOffset = body.offset;
    MeshData world = worldMesh(body);
    if (o.open) { // cut the opening, then drop its cap so the surface is open there
        Body cut = body;
        if (!trimByPlane(cut, o.openPoint, o.openNormal, error)) return false;
        world = worldMesh(cut);
        const Vec3 n = normalize(o.openNormal);
        const AABB bb = world.bounds();
        const float eps = std::max(length(bb.max - bb.min), 1.0f) * 1e-5f;
        MeshData open;
        open.vertices = world.vertices;
        for (std::size_t k = 0; k + 2 < world.indices.size(); k += 3) {
            bool onPlane = true;
            for (int i = 0; i < 3; ++i)
                onPlane = onPlane && std::fabs(dot(world.vertices[world.indices[k + static_cast<std::size_t>(i)]].position - o.openPoint, n)) < eps;
            if (!onPlane) open.indices.insert(open.indices.end(), {world.indices[k], world.indices[k + 1], world.indices[k + 2]});
        }
        world = std::move(open);
    }
    // Welded with a tolerance: the seams of round primitives close up.
    Body surface;
    surface.mesh = std::move(world);
    const SculptMesh sm(surface);
    WMesh w;
    w.p = sm.positions();
    w.t = sm.triangles();
    if (w.t.empty()) {
        if (error) *error = body.name + " has no surface to shell";
        return false;
    }
    // Hollowing a closed solid inwards: the wall must leave a cavity.
    if (!o.outward && sm.openEdges() == 0) {
        AABB bb;
        for (const Vec3& p : w.p) bb.expand(p);
        const Vec3 ext = bb.max - bb.min;
        const float minDim = std::min({ext.x, ext.y, ext.z});
        if (o.thickness * 2.0f >= minDim) {
            if (error) {
                char b[160];
                std::snprintf(b, sizeof(b), "A %g mm wall leaves no cavity in %s (%g mm thin): use less, or OUTWARD",
                              static_cast<double>(o.thickness), body.name.c_str(), static_cast<double>(minDim));
                *error = b;
            }
            return false;
        }
    }
    std::vector<Vec3> dir;
    offsetDirections(w, dir);
    std::vector<Vec3> outer = w.p, inner = w.p;
    for (std::size_t v = 0; v < w.p.size(); ++v) {
        if (o.outward) outer[v] = w.p[v] + dir[v] * o.thickness;
        else inner[v] = w.p[v] - dir[v] * o.thickness;
    }
    for (std::size_t v = 0; v < w.p.size(); ++v)
        if (!std::isfinite(outer[v].x + outer[v].y + outer[v].z + inner[v].x + inner[v].y + inner[v].z)) {
            if (error) *error = "That thickness is out of range";
            return false;
        }
    MeshData m = shellBetween(w, outer, inner);
    if (m.indices.empty()) {
        if (error) *error = body.name + ": the shell came out empty";
        return false;
    }
    store(body, std::move(m));
    // Keep the body where it was placed (mesh relative to its offset).
    if (keepOffset.x != 0 || keepOffset.y != 0 || keepOffset.z != 0) {
        body.mesh.transform(translate(-keepOffset));
        body.offset = keepOffset;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Armour

std::optional<Body> armorFor(const Body& source, float clearance, float thickness, std::string* error) {
    if (clearance < 0 || !(thickness > 0)) {
        if (error) *error = "Armour needs a clearance (0 or more) and a thickness";
        return std::nullopt;
    }
    const MeshData world = worldMesh(source);
    if (world.indices.empty()) {
        if (error) *error = source.name + " has no surface";
        return std::nullopt;
    }
    // A volume around the body: the armour is where the signed distance to it lies
    // between the clearance and the clearance plus the wall. Signs come from the
    // winding number, so overlapping and open shells (most robot parts) work.
    const float band = clearance + thickness;
    const AABB bb = world.bounds();
    float h = std::min(thickness * 0.6f, band * 0.25f);
    const float pad = band + h * 3.0f;
    const Vec3 ext = bb.max - bb.min + Vec3{pad, pad, pad} * 2.0f;
    h = std::max(h, std::max({ext.x, ext.y, ext.z}) / 160.0f);
    if (thickness < h * 0.5f) {
        char b[160];
        std::snprintf(b, sizeof(b), "A wall of %g mm is too thin for a part this size: at least %.2g mm", static_cast<double>(thickness),
                      static_cast<double>(h * 0.5f));
        if (error) *error = b;
        return std::nullopt;
    }
    const int nx = static_cast<int>(ext.x / h) + 2, ny = static_cast<int>(ext.y / h) + 2, nz = static_cast<int>(ext.z / h) + 2;
    const Vec3 origin = bb.min - Vec3{pad, pad, pad};
    TriGrid grid;
    grid.build(world);
    const float reach = band + h * 2.0f;
    // The signed distance to the body (far voxels: `reach`; they are never armour).
    std::vector<float> f(static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny) * static_cast<std::size_t>(nz), reach);
    // Exact distances only where they matter: near the surface (the flood's barrier)
    // and around the two faces, within the blur's reach. A coarse pass (every 4th
    // voxel) bounds the rest: the distance changes by at most the step to the nearest
    // coarse sample, so a voxel well inside the gap or well outside keeps an estimate.
    const auto sx = static_cast<std::size_t>(ny) * static_cast<std::size_t>(nz), sy = static_cast<std::size_t>(nz);
    auto centreOf = [&](int x, int y, int z) {
        return origin + Vec3{(static_cast<float>(x) + 0.5f) * h, (static_cast<float>(y) + 0.5f) * h, (static_cast<float>(z) + 0.5f) * h};
    };
    constexpr int kStep = 4;
    const float slack = 3.0f * h * 1.7321f; // up to 3 voxels from a coarse sample on each axis
    const float far = band + 4.0f * h;      // beyond this: never armour, never blurred into it
    const int cx = (nx - 1) / kStep + 1, cy = (ny - 1) / kStep + 1, cz = (nz - 1) / kStep + 1;
    std::vector<float> coarse(static_cast<std::size_t>(cx) * static_cast<std::size_t>(cy) * static_cast<std::size_t>(cz));
    for (int x = 0; x < cx; ++x)
        for (int y = 0; y < cy; ++y)
            for (int z = 0; z < cz; ++z) {
                const auto d = grid.nearest(centreOf(x * kStep, y * kStep, z * kStep), far + slack);
                coarse[(static_cast<std::size_t>(x) * static_cast<std::size_t>(cy) + static_cast<std::size_t>(y)) * static_cast<std::size_t>(cz) +
                       static_cast<std::size_t>(z)] = d ? *d : far + slack;
            }
    for (int x = 0; x < nx; ++x)
        for (int y = 0; y < ny; ++y)
            for (int z = 0; z < nz; ++z) {
                const int kx = std::min((x + kStep / 2) / kStep, cx - 1), ky = std::min((y + kStep / 2) / kStep, cy - 1),
                          kz = std::min((z + kStep / 2) / kStep, cz - 1);
                const float dc = coarse[(static_cast<std::size_t>(kx) * static_cast<std::size_t>(cy) + static_cast<std::size_t>(ky)) *
                                            static_cast<std::size_t>(cz) +
                                        static_cast<std::size_t>(kz)];
                const float lo = dc - slack, hi = dc + slack;
                const std::size_t i = static_cast<std::size_t>(x) * sx + static_cast<std::size_t>(y) * sy + static_cast<std::size_t>(z);
                if (lo > far) continue; // stays `reach`
                const bool needed = lo <= 2.0f * h || (hi >= clearance - 4.0f * h && lo <= far);
                if (!needed) {
                    f[i] = std::min(dc, reach); // in the gap between the body and the inner face
                    continue;
                }
                if (const auto d = grid.nearest(centreOf(x, y, z), reach)) f[i] = *d;
            }
    // Outside: what a flood from the grid's border reaches without crossing the layer
    // of voxels within one cell of the surface. Topological, so the noisy inside /
    // outside of open and overlapping shells does not matter (an opening wider than
    // a few cells lets the outside in: an open surface has no inside).
    {
        std::vector<char> outside(f.size(), 0);
        std::vector<std::size_t> stack;
        auto open = [&](int x, int y, int z) {
            const std::size_t i = static_cast<std::size_t>(x) * sx + static_cast<std::size_t>(y) * sy + static_cast<std::size_t>(z);
            if (outside[i] || f[i] <= h) return;
            outside[i] = 1;
            stack.push_back(i);
        };
        for (int x = 0; x < nx; ++x)
            for (int y = 0; y < ny; ++y) {
                open(x, y, 0);
                open(x, y, nz - 1);
            }
        for (int x = 0; x < nx; ++x)
            for (int z = 0; z < nz; ++z) {
                open(x, 0, z);
                open(x, ny - 1, z);
            }
        for (int y = 0; y < ny; ++y)
            for (int z = 0; z < nz; ++z) {
                open(0, y, z);
                open(nx - 1, y, z);
            }
        while (!stack.empty()) {
            const std::size_t i = stack.back();
            stack.pop_back();
            const int x = static_cast<int>(i / sx), y = static_cast<int>((i % sx) / sy), z = static_cast<int>(i % sy);
            if (x > 0) open(x - 1, y, z);
            if (x + 1 < nx) open(x + 1, y, z);
            if (y > 0) open(x, y - 1, z);
            if (y + 1 < ny) open(x, y + 1, z);
            if (z > 0) open(x, y, z - 1);
            if (z + 1 < nz) open(x, y, z + 1);
        }
        for (std::size_t i = 0; i < f.size(); ++i)
            if (!outside[i] && f[i] > h) f[i] = -f[i];
    }
    // Rounds over surface detail finer than the wall: an exact offset of ribs and
    // panel lines has cusps between them: two passes of a [1 2 1] blur of the signed
    // distance.
    {
        std::vector<float> tmp(f.size());
        for (int pass = 0; pass < 2; ++pass)
            for (int axis = 0; axis < 3; ++axis) {
                const int n = axis == 0 ? nx : axis == 1 ? ny : nz;
                const std::size_t stride = axis == 0 ? sx : axis == 1 ? sy : 1;
                for (int x = 0; x < nx; ++x)
                    for (int y = 0; y < ny; ++y)
                        for (int z = 0; z < nz; ++z) {
                            const std::size_t i = static_cast<std::size_t>(x) * sx + static_cast<std::size_t>(y) * sy + static_cast<std::size_t>(z);
                            const int c = axis == 0 ? x : axis == 1 ? y : z;
                            const float lo = c > 0 ? f[i - stride] : f[i], hi = c + 1 < n ? f[i + stride] : f[i];
                            tmp[i] = (lo + f[i] * 2.0f + hi) * 0.25f;
                        }
                f.swap(tmp);
            }
    }
    // The inner and outer faces are two level sets of the distance, marched one at a
    // time: a wall only a voxel or two thick would otherwise alias into a stipple.
    // Level sets of one field never cross, so the shell is closed and clean.
    MeshData shell;
    std::vector<float> level(f.size());
    for (const bool outerFace : {false, true}) {
        for (std::size_t i = 0; i < f.size(); ++i) level[i] = outerFace ? band - f[i] : f[i] - clearance;
        MeshData face = marchTets(level, nx, ny, nz, origin, h, false);
        if (face.indices.empty()) {
            if (error) *error = "No room for armour around " + source.name;
            return std::nullopt;
        }
        finishMarched(face, 10);
        const auto base = static_cast<std::uint32_t>(shell.vertices.size());
        shell.vertices.insert(shell.vertices.end(), face.vertices.begin(), face.vertices.end());
        for (const std::uint32_t i : face.indices) shell.indices.push_back(base + i);
    }
    simplifyMesh(shell, h * 1.2f);
    Body armour;
    armour.name = "Armor " + source.name;
    armour.color = {0.34f, 0.37f, 0.42f};
    store(armour, std::move(shell), 70.0f);
    // Placed at the source's origin, so it turns with it about the same points.
    armour.mesh.transform(translate(-source.offset));
    armour.offset = source.offset;
    return armour;
}

// ---------------------------------------------------------------------------
// Interference

InterferenceReport checkInterference(const Scene& scene, const std::vector<std::size_t>& chosen, float clearance,
                                     float step, float duration) {
    InterferenceReport report;
    const auto& bodies = scene.bodies();
    std::vector<std::size_t> set = chosen;
    if (set.empty())
        for (std::size_t i = 0; i < bodies.size(); ++i)
            if (bodies[i].visible) set.push_back(i);
    bool anyArmour = false;
    for (std::size_t i : set) anyArmour = anyArmour || bodies[i].name.rfind("Armor ", 0) == 0;

    std::vector<std::pair<std::size_t, std::size_t>> pairs;
    for (std::size_t x = 0; x < set.size(); ++x)
        for (std::size_t y = x + 1; y < set.size(); ++y) {
            const Body& a = bodies[set[x]];
            const Body& b = bodies[set[y]];
            const bool aArm = a.name.rfind("Armor ", 0) == 0, bArm = b.name.rfind("Armor ", 0) == 0;
            // With armour, check the armour (against the body and itself); a part
            // is never checked against its own armour's skin pairs twice.
            if (chosen.empty() && anyArmour && !aArm && !bArm) continue;
            pairs.emplace_back(set[x], set[y]);
        }
    report.checkedPairs = pairs.size();
    if (duration <= 0) duration = std::max(suggestedDuration(scene), 0.0f);
    if (step <= 0) step = duration > 0 ? duration / 40.0f : 1.0f;
    step = std::max(step, duration / 2000.0f); // at most ~2000 poses
    report.duration = duration;

    const float reach = std::max(clearance * 2.0f, 1.0f);
    // Built on first use: most pairs never come near each other.
    std::unordered_map<std::size_t, TriGrid> grids;
    std::unordered_map<std::size_t, std::unique_ptr<WindingNumber>> winds;
    std::unordered_map<std::size_t, std::vector<Vec3>> samples;
    auto gridOf = [&](std::size_t k) -> const TriGrid& {
        auto [it, fresh] = grids.try_emplace(k);
        if (fresh) it->second.build(bodies[k].mesh);
        return it->second;
    };
    auto windOf = [&](std::size_t k) -> const WindingNumber& {
        auto& w = winds[k];
        if (!w) w = std::make_unique<WindingNumber>(bodies[k].mesh);
        return *w;
    };
    auto samplesOf = [&](std::size_t k) -> const std::vector<Vec3>& {
        auto [it, fresh] = samples.try_emplace(k);
        if (fresh) {
            const auto& v = bodies[k].mesh.vertices;
            const std::size_t stride = v.size() / 1500 + 1;
            for (std::size_t q = 0; q < v.size(); q += stride) it->second.push_back(v[q].position);
        }
        return it->second;
    };
    std::vector<AABB> localBox(bodies.size());
    for (const auto& [i, j] : pairs)
        for (std::size_t k : {i, j})
            if (localBox[k].min.x > localBox[k].max.x) localBox[k] = bodies[k].mesh.bounds();

    // How far a point is from the surface: the search widens until it meets a face
    // (`limit` when nothing is nearer). `front`: the side of that face the point is on.
    auto distanceOf = [&](const TriGrid& g, const Vec3& q, float from, float limit, bool* front = nullptr) {
        for (float r = from * 2.0f;; r *= 2.0f) {
            if (const auto d = g.nearest(q, std::min(r, limit), front)) return *d;
            if (r >= limit) return limit;
        }
    };
    // Armour (one clean closed shell, inner and outer face) is inside exactly where
    // the nearest face looks away; robot parts can be open or overlapping shells,
    // where only the winding number is reliable.
    auto cleanShell = [&](std::size_t k) { return bodies[k].name.rfind("Armor ", 0) == 0; };
    // Inside or outside, for a point with no surface within reach. A point D from the
    // surface shares its side with every point within D of it, so the answer is
    // cached for the largest grid cell around it that fits in that ball: the cells
    // of an octree-like set of levels, from reach/2 up.
    std::unordered_map<std::size_t, std::unordered_map<std::uint64_t, bool>> sideCache;
    auto deepInside = [&](std::size_t body, const Vec3& q, const WindingNumber& wn) {
        const TriGrid& g = gridOf(body);
        if (q.x < g.box.min.x || q.y < g.box.min.y || q.z < g.box.min.z || q.x > g.box.max.x || q.y > g.box.max.y || q.z > g.box.max.z)
            return false;
        const float base = std::max(clearance, 0.5f);
        auto& cache = sideCache[body];
        auto key = [&](int level) {
            const float c = base * static_cast<float>(1 << level);
            const auto cx = static_cast<std::uint64_t>((q.x - g.box.min.x) / c), cy = static_cast<std::uint64_t>((q.y - g.box.min.y) / c),
                       cz = static_cast<std::uint64_t>((q.z - g.box.min.z) / c);
            return static_cast<std::uint64_t>(level) << 60 | (cx & 0xfffff) << 40 | (cy & 0xfffff) << 20 | (cz & 0xfffff);
        };
        constexpr int kLevels = 10;
        for (int level = kLevels - 1; level >= 0; --level)
            if (const auto it = cache.find(key(level)); it != cache.end()) return it->second;
        bool front = true;
        const float d = distanceOf(g, q, reach, base * static_cast<float>(1 << kLevels) * 1.8f, &front);
        int level = 0;
        while (level + 1 < kLevels && base * static_cast<float>(1 << (level + 1)) * 1.7321f <= d) ++level;
        const bool inside = cleanShell(body) ? !front : wn(q) > 0.5;
        cache.emplace(key(level), inside);
        return inside;
    };
    std::vector<InterferencePair> worst(pairs.size());
    for (std::size_t p = 0; p < pairs.size(); ++p) {
        worst[p].a = bodies[pairs[p].first].name;
        worst[p].b = bodies[pairs[p].second].name;
        worst[p].distance = 1e30f;
    }
    std::vector<AABB> worldBox(bodies.size());
    // One pose: the worst distance of each pair into `into` (the pose's time recorded).
    auto measure = [&](const std::vector<Mat4>& tr, float t, std::vector<InterferencePair>& into) {
        auto worldOf = [&](std::size_t i) {
            const Mat4 at = translate(bodies[i].offset);
            return i < tr.size() ? tr[i] * at : at;
        };
        // Broad phase: the parts' boxes in this pose.
        for (std::size_t k = 0; k < bodies.size(); ++k) {
            if (localBox[k].min.x > localBox[k].max.x) continue;
            const Mat4 w = worldOf(k);
            AABB b;
            for (int c = 0; c < 8; ++c)
                b.expand(w.transformPoint({c & 1 ? localBox[k].max.x : localBox[k].min.x, c & 2 ? localBox[k].max.y : localBox[k].min.y,
                                           c & 4 ? localBox[k].max.z : localBox[k].min.z}));
            worldBox[k] = b;
        }
        for (std::size_t p = 0; p < pairs.size(); ++p) {
            const AABB& ba = worldBox[pairs[p].first];
            const AABB& bb = worldBox[pairs[p].second];
            if (ba.min.x > bb.max.x + reach || bb.min.x > ba.max.x + reach || ba.min.y > bb.max.y + reach || bb.min.y > ba.max.y + reach ||
                ba.min.z > bb.max.z + reach || bb.min.z > ba.max.z + reach)
                continue;
            for (int dirn = 0; dirn < 2; ++dirn) {
                const std::size_t from = dirn == 0 ? pairs[p].first : pairs[p].second;
                const std::size_t to = dirn == 0 ? pairs[p].second : pairs[p].first;
                const Mat4 fromW = worldOf(from), toW = worldOf(to);
                const Mat4 rel = inverse(toW) * fromW;
                const TriGrid& g = gridOf(to);
                const WindingNumber& wn = windOf(to);
                const AABB& lb = g.box;
                for (const Vec3& s : samplesOf(from)) {
                    const Vec3 q = rel.transformPoint(s);
                    if (q.x < lb.min.x - reach || q.y < lb.min.y - reach || q.z < lb.min.z - reach || q.x > lb.max.x + reach ||
                        q.y > lb.max.y + reach || q.z > lb.max.z + reach)
                        continue;
                    // Within reach of the surface: its distance, signed by the winding
                    // number; deeper inside: the distance to the nearest face.
                    bool front = true;
                    const auto near = g.nearest(q, reach, &front);
                    if (!near && !deepInside(to, q, wn)) continue;
                    // Behind the nearest face can be a concave corner: the winding number decides.
                    const bool inside = near && !front ? (cleanShell(to) || wn(q) > 0.5) : !near;
                    const float d = near ? (inside ? -*near : *near) : -distanceOf(g, q, reach, length(g.box.max - g.box.min) + reach);
                    if (d < into[p].distance) {
                        into[p].distance = d;
                        into[p].time = t;
                        into[p].at = fromW.transformPoint(s);
                    }
                }
            }
        }
    };
    // The rest pose first: links of a robot often overlap at their joints by design,
    // which is no collision of the motion.
    std::vector<InterferencePair> rest = worst;
    measure({}, 0.0f, rest);
    const auto poses = static_cast<long>(std::floor(duration / step + 1e-4f));
    for (long k = 0; k <= poses; ++k) {
        const float t = static_cast<float>(k) * step;
        ++report.samples;
        measure(simulationTransforms(scene, t), t, worst);
    }
    // Pairs that already touch at rest count only when the motion pushes them deeper
    // by more than the clearance.
    for (std::size_t p = 0; p < pairs.size(); ++p) {
        worst[p].restDistance = rest[p].distance;
        if (rest[p].distance >= clearance) continue;
        if (worst[p].distance < rest[p].distance - std::max(clearance, 0.5f))
            continue;
        ++report.restOverlaps;
        worst[p].distance = 1e30f;
    }
    for (const InterferencePair& w : worst)
        if (w.distance < clearance) report.pairs.push_back(w);
    std::sort(report.pairs.begin(), report.pairs.end(),
              [&](const InterferencePair& x, const InterferencePair& y) {
                  // The motion's own collisions first, then overlaps made worse.
                  const bool xr = x.restDistance < clearance, yr = y.restDistance < clearance;
                  if (xr != yr) return yr;
                  if (x.distance != y.distance) return x.distance < y.distance;
                  return x.a != y.a ? x.a < y.a : x.b < y.b;
              });
    return report;
}

// ---------------------------------------------------------------------------
// Lattice lightweighting

bool lightenBody(Body& body, const LightenOptions& o, std::string* error, double* volumeBefore, double* volumeAfter) {
    const MeshData world = worldMesh(body);
    if (SculptMesh(body).openEdges() != 0) { // welded with a tolerance: round primitives have seams
        if (error) *error = body.name + " is not closed: lightening needs a solid";
        return false;
    }
    const AABB bb = world.bounds();
    const Vec3 ext = bb.max - bb.min;
    const float minDim = std::min({ext.x, ext.y, ext.z});
    if (!(o.skin > 0) || minDim < o.skin * 2.5f) {
        if (error) *error = body.name + " is too thin for a skin of " + std::to_string(o.skin) + " mm";
        return false;
    }
    // Voxel size: fine enough for the skin and the lattice, at most ~100 cells a side.
    float h = std::min(o.skin, o.hollow ? 1e9f : o.cell / 5.0f);
    h = std::max(h, std::max({ext.x, ext.y, ext.z}) / 100.0f);
    // What the voxels can resolve: a lattice finer than that would only be noise
    // (and millions of triangles).
    auto mm = [](float v) {
        char b[32];
        std::snprintf(b, sizeof(b), "%.2g mm", static_cast<double>(v));
        return std::string(b);
    };
    if (o.skin < h * 0.5f) {
        if (error) *error = "SKIN must be at least " + mm(h * 0.5f) + " for a part this size";
        return false;
    }
    if (!o.hollow) {
        if (!(o.cell >= h * 4.0f)) {
            if (error) *error = "CELL must be at least " + mm(h * 4.0f) + " for a part this size";
            return false;
        }
        if (!(o.wall >= h * 0.5f) || o.wall > o.cell * 0.45f) {
            if (error) *error = "WALL must be between " + mm(h * 0.5f) + " and " + mm(o.cell * 0.45f) + " (less than half the cell)";
            return false;
        }
    }
    const int nx = static_cast<int>(ext.x / h) + 3, ny = static_cast<int>(ext.y / h) + 3, nz = static_cast<int>(ext.z / h) + 3;
    const Vec3 origin = bb.min - Vec3{h, h, h};
    auto at = [&](int x, int y, int z) {
        return origin + Vec3{(static_cast<float>(x) + 0.5f) * h, (static_cast<float>(y) + 0.5f) * h, (static_cast<float>(z) + 0.5f) * h};
    };
    auto idx = [&](int x, int y, int z) { return (static_cast<std::size_t>(x) * static_cast<std::size_t>(ny) + static_cast<std::size_t>(y)) * static_cast<std::size_t>(nz) + static_cast<std::size_t>(z); };

    // Inside: crossings of a vertical line per column (triangles binned in x, y).
    std::vector<char> inside(static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny) * static_cast<std::size_t>(nz), 0);
    {
        std::vector<std::vector<std::uint32_t>> bins(static_cast<std::size_t>(nx * ny));
        const std::size_t T = world.indices.size() / 3;
        for (std::size_t t = 0; t < T; ++t) {
            AABB tb;
            for (int k = 0; k < 3; ++k) tb.expand(world.vertices[world.indices[t * 3 + static_cast<std::size_t>(k)]].position);
            const int x0 = std::max(0, static_cast<int>((tb.min.x - origin.x) / h) - 1), x1 = std::min(nx - 1, static_cast<int>((tb.max.x - origin.x) / h) + 1);
            const int y0 = std::max(0, static_cast<int>((tb.min.y - origin.y) / h) - 1), y1 = std::min(ny - 1, static_cast<int>((tb.max.y - origin.y) / h) + 1);
            for (int x = x0; x <= x1; ++x)
                for (int y = y0; y <= y1; ++y) bins[static_cast<std::size_t>(x * ny + y)].push_back(static_cast<std::uint32_t>(t));
        }
        std::vector<float> zs;
        for (int x = 0; x < nx; ++x)
            for (int y = 0; y < ny; ++y) {
                // Slightly off the voxel centre, so the line misses mesh edges.
                const float px = at(x, y, 0).x + h * 0.0137f, py = at(x, y, 0).y + h * 0.0071f;
                zs.clear();
                for (std::uint32_t t : bins[static_cast<std::size_t>(x * ny + y)]) {
                    const Vec3 a = world.vertices[world.indices[t * 3]].position, b = world.vertices[world.indices[t * 3 + 1]].position,
                               c = world.vertices[world.indices[t * 3 + 2]].position;
                    const float d = (b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y);
                    if (std::fabs(d) < 1e-12f) continue;
                    const float l0 = ((b.y - c.y) * (px - c.x) + (c.x - b.x) * (py - c.y)) / d;
                    const float l1 = ((c.y - a.y) * (px - c.x) + (a.x - c.x) * (py - c.y)) / d;
                    const float l2 = 1.0f - l0 - l1;
                    if (l0 < 0 || l1 < 0 || l2 < 0) continue;
                    zs.push_back(l0 * a.z + l1 * b.z + l2 * c.z);
                }
                std::sort(zs.begin(), zs.end());
                for (std::size_t k = 0; k + 1 < zs.size(); k += 2)
                    for (int z = 0; z < nz; ++z) {
                        const float pz = at(x, y, z).z;
                        if (pz > zs[k] && pz < zs[k + 1]) inside[idx(x, y, z)] = 1;
                    }
            }
    }
    // Depth below the surface: chamfer distance (3-4-5) in two passes.
    std::vector<float> depth(inside.size());
    for (std::size_t i = 0; i < inside.size(); ++i) depth[i] = inside[i] ? 1e9f : 0.0f;
    const float c1 = h, c2 = h * 1.4142f, c3 = h * 1.7321f;
    for (int pass = 0; pass < 2; ++pass) {
        const int s = pass == 0 ? 1 : -1;
        for (int xi = 0; xi < nx; ++xi)
            for (int yi = 0; yi < ny; ++yi)
                for (int zi = 0; zi < nz; ++zi) {
                    const int x = pass == 0 ? xi : nx - 1 - xi, y = pass == 0 ? yi : ny - 1 - yi, z = pass == 0 ? zi : nz - 1 - zi;
                    float& d = depth[idx(x, y, z)];
                    if (d == 0) continue;
                    for (int dx = -1; dx <= 0; ++dx)
                        for (int dy = -1; dy <= 1; ++dy)
                            for (int dz = -1; dz <= 1; ++dz) {
                                if (dx == 0 && (dy > 0 || (dy == 0 && dz >= 0))) continue; // the half already visited
                                const int X = x + dx * s, Y = y + dy * s, Z = z + dz * s;
                                if (X < 0 || Y < 0 || Z < 0 || X >= nx || Y >= ny || Z >= nz) continue;
                                const int m = std::abs(dx) + std::abs(dy) + std::abs(dz);
                                d = std::min(d, depth[idx(X, Y, Z)] + (m == 1 ? c1 : m == 2 ? c2 : c3));
                            }
                }
    }
    // The void: below the skin and (with the lattice) away from the gyroid sheet.
    const float k = 6.2831853f / std::max(o.cell, 1e-3f);
    const float g0 = std::clamp(o.wall * k * 0.6f, 0.05f, 1.2f);
    std::vector<float> f(inside.size());
    for (int x = 0; x < nx; ++x)
        for (int y = 0; y < ny; ++y)
            for (int z = 0; z < nz; ++z) {
                const std::size_t i = idx(x, y, z);
                float v = depth[i] - o.skin - h * 0.5f;
                if (!o.hollow) {
                    const Vec3 p = at(x, y, z);
                    const float gy = std::sin(k * p.x) * std::cos(k * p.y) + std::sin(k * p.y) * std::cos(k * p.z) +
                                     std::sin(k * p.z) * std::cos(k * p.x);
                    v = std::min(v, (std::fabs(gy) - g0) / (k * 1.2f));
                }
                f[i] = v;
            }
    // The void's surface, facing into the void (the part's inner walls).
    MeshData cavity = marchTets(f, nx, ny, nz, origin, h, true);
    if (cavity.indices.empty()) {
        if (error) *error = body.name + " has no room inside for a skin of " + std::to_string(o.skin) + " mm";
        return false;
    }
    finishMarched(cavity, 4);
    simplifyMesh(cavity, h * 1.0f);
    if (volumeBefore) *volumeBefore = meshVolume(world);
    MeshData result = world;
    result.edgeVertices.clear();
    result.append(cavity);
    if (volumeAfter) *volumeAfter = meshVolume(result);
    const Vec3 keep = body.offset;
    result.transform(translate(-keep));
    // Only the outer surface's edges: the lattice would be a thicket of lines.
    result.edgeVertices = body.mesh.edgeVertices;
    body.mesh = std::move(result);
    body.kind = "mesh";
    body.params.clear();
    return true;
}

// ---------------------------------------------------------------------------
// Voxel remesh

bool remeshBodies(Body& target, const std::vector<const Body*>& others, const RemeshOptions& o, std::string* error) {
    std::vector<MeshData> parts{worldMesh(target)};
    for (const Body* b : others) parts.push_back(worldMesh(*b));
    AABB bb;
    for (std::size_t k = 0; k < parts.size(); ++k) {
        if (parts[k].indices.empty()) {
            if (error) *error = (k == 0 ? target.name : others[k - 1]->name) + " has no surface";
            return false;
        }
        bb.expand(parts[k].bounds());
    }
    const Vec3 size = bb.max - bb.min;
    const float h = o.voxel > 0 ? o.voxel : std::max({size.x, size.y, size.z}) / 160.0f;
    const float blend = std::isfinite(o.blend) ? std::max(o.blend, 0.0f) : 0.0f;
    const float sigma = !std::isfinite(o.smooth) ? 0.0f : o.smooth < 0 ? h : o.smooth;
    const float reach = blend + 2.0f * sigma + 3.0f * h; // how far out the distances matter
    const float pad = reach + 2.0f * h;
    const double cells = (static_cast<double>(size.x + 2 * pad) / h + 2) * (static_cast<double>(size.y + 2 * pad) / h + 2) *
                         (static_cast<double>(size.z + 2 * pad) / h + 2);
    if (!(h > 0) || !std::isfinite(h) || cells > 6.4e7) {
        char b[160];
        std::snprintf(b, sizeof(b), "A %g mm voxel is too fine for parts this size: at least %.2g mm", static_cast<double>(h),
                      static_cast<double>(h * std::cbrt(cells / 6.4e7) * 1.01));
        if (error) *error = b;
        return false;
    }
    const int nx = static_cast<int>((size.x + 2 * pad) / h) + 2, ny = static_cast<int>((size.y + 2 * pad) / h) + 2,
              nz = static_cast<int>((size.z + 2 * pad) / h) + 2;
    const Vec3 origin = bb.min - Vec3{pad, pad, pad};
    const auto sx = static_cast<std::size_t>(ny) * static_cast<std::size_t>(nz), sy = static_cast<std::size_t>(nz);
    auto centreOf = [&](int x, int y, int z) {
        return origin + Vec3{(static_cast<float>(x) + 0.5f) * h, (static_cast<float>(y) + 0.5f) * h, (static_cast<float>(z) + 0.5f) * h};
    };
    // The union's signed distance (positive outside), each part's added with a
    // smooth minimum: where two parts meet, material fills in to a fillet of about
    // the blend radius; elsewhere each surface stays where it was.
    std::vector<float> field(static_cast<std::size_t>(nx) * sx, reach);
    for (const MeshData& m : parts) {
        // The part's box of voxels, with room for the distances that matter.
        const AABB pb = m.bounds();
        const float mn[3] = {pb.min.x, pb.min.y, pb.min.z}, mx[3] = {pb.max.x, pb.max.y, pb.max.z}, o0[3] = {origin.x, origin.y, origin.z};
        const int n[3] = {nx, ny, nz};
        int lo[3], cnt[3];
        for (int k = 0; k < 3; ++k) {
            lo[k] = std::clamp(static_cast<int>(std::floor((mn[k] - reach - h - o0[k]) / h)), 0, n[k] - 1);
            const int hi = std::clamp(static_cast<int>(std::ceil((mx[k] + reach + h - o0[k]) / h)), 0, n[k] - 1);
            cnt[k] = hi - lo[k] + 1;
        }
        const auto tx = static_cast<std::size_t>(cnt[1]) * static_cast<std::size_t>(cnt[2]), ty = static_cast<std::size_t>(cnt[2]);
        const std::size_t total = static_cast<std::size_t>(cnt[0]) * tx;
        auto at = [&](int x, int y, int z) { return static_cast<std::size_t>(x) * tx + static_cast<std::size_t>(y) * ty + static_cast<std::size_t>(z); };
        auto local = [&](int x, int y, int z) { return centreOf(lo[0] + x, lo[1] + y, lo[2] + z); };
        std::vector<float> d(total, 1e30f);
        std::vector<Vec3> cp(total); // the nearest surface point found so far
        std::vector<char> side(total, 1);
        // Exact distances next to the surface: each triangle against the voxels around it.
        const float band = 1.8f * h;
        for (std::size_t k = 0; k + 2 < m.indices.size(); k += 3) {
            const Vec3 a = m.vertices[m.indices[k]].position, b = m.vertices[m.indices[k + 1]].position, c = m.vertices[m.indices[k + 2]].position;
            const Vec3 nrm = cross(b - a, c - a);
            const Vec3 tl = vmin(a, vmin(b, c)) - Vec3{band, band, band}, th = vmax(a, vmax(b, c)) + Vec3{band, band, band};
            const float tlo[3] = {tl.x, tl.y, tl.z}, thi[3] = {th.x, th.y, th.z};
            int r0[3], r1[3];
            for (int q = 0; q < 3; ++q) {
                r0[q] = std::max(static_cast<int>(std::floor((tlo[q] - o0[q]) / h - 0.5f)) - lo[q], 0);
                r1[q] = std::min(static_cast<int>(std::ceil((thi[q] - o0[q]) / h - 0.5f)) - lo[q], cnt[q] - 1);
            }
            for (int x = r0[0]; x <= r1[0]; ++x)
                for (int y = r0[1]; y <= r1[1]; ++y)
                    for (int z = r0[2]; z <= r1[2]; ++z) {
                        const Vec3 p = local(x, y, z);
                        const Vec3 on = closestOnTri(p, a, b, c);
                        const float dd = length(p - on);
                        const std::size_t i = at(x, y, z);
                        if (dd < d[i] && dd <= band) {
                            d[i] = dd;
                            cp[i] = on;
                            side[i] = dot(p - on, nrm) >= 0 ? 1 : 0;
                        }
                    }
        }
        // Out to the reach: each voxel takes the nearest point its neighbours know
        // (two raster sweeps, forwards and back, over the 26 neighbours).
        for (const int dir : {1, -1}) {
            const int x0 = dir > 0 ? 0 : cnt[0] - 1, y0 = dir > 0 ? 0 : cnt[1] - 1, z0 = dir > 0 ? 0 : cnt[2] - 1;
            for (int x = x0; x >= 0 && x < cnt[0]; x += dir)
                for (int y = y0; y >= 0 && y < cnt[1]; y += dir)
                    for (int z = z0; z >= 0 && z < cnt[2]; z += dir) {
                        const std::size_t i = at(x, y, z);
                        const Vec3 p = local(x, y, z);
                        for (int ex = -1; ex <= 1; ++ex)
                            for (int ey = -1; ey <= 1; ++ey)
                                for (int ez = -1; ez <= 1; ++ez) {
                                    // Only neighbours the sweep has already passed.
                                    const int ord = ex != 0 ? ex : ey != 0 ? ey : ez;
                                    if (ord != -dir) continue;
                                    const int qx = x + ex, qy = y + ey, qz = z + ez;
                                    if (qx < 0 || qy < 0 || qz < 0 || qx >= cnt[0] || qy >= cnt[1] || qz >= cnt[2]) continue;
                                    const std::size_t j = at(qx, qy, qz);
                                    if (d[j] >= 1e29f) continue;
                                    const float dd = length(p - cp[j]);
                                    if (dd < d[i] && dd <= reach) {
                                        d[i] = dd;
                                        cp[i] = cp[j];
                                    }
                                }
                    }
        }
        // Inside and out away from the surface: a flood from the box's border that
        // does not cross the voxels next to it. Next to it, the nearest face decides.
        std::vector<char> outside(total, 0);
        std::vector<std::size_t> stack;
        auto open = [&](int x, int y, int z) {
            const std::size_t i = at(x, y, z);
            if (outside[i] || d[i] <= h) return;
            outside[i] = 1;
            stack.push_back(i);
        };
        for (int x = 0; x < cnt[0]; ++x)
            for (int y = 0; y < cnt[1]; ++y)
                for (int z = 0; z < cnt[2]; ++z)
                    if (x == 0 || y == 0 || z == 0 || x == cnt[0] - 1 || y == cnt[1] - 1 || z == cnt[2] - 1) open(x, y, z);
        while (!stack.empty()) {
            const std::size_t i = stack.back();
            stack.pop_back();
            const int x = static_cast<int>(i / tx), y = static_cast<int>((i % tx) / ty), z = static_cast<int>(i % ty);
            if (x > 0) open(x - 1, y, z);
            if (x + 1 < cnt[0]) open(x + 1, y, z);
            if (y > 0) open(x, y - 1, z);
            if (y + 1 < cnt[1]) open(x, y + 1, z);
            if (z > 0) open(x, y, z - 1);
            if (z + 1 < cnt[2]) open(x, y, z + 1);
        }
        for (int x = 0; x < cnt[0]; ++x)
            for (int y = 0; y < cnt[1]; ++y)
                for (int z = 0; z < cnt[2]; ++z) {
                    const std::size_t i = at(x, y, z);
                    const float dist = std::min(d[i], reach);
                    const bool inside = d[i] <= band ? !side[i] : !outside[i];
                    const float sd = inside ? -dist : dist;
                    float& f = field[static_cast<std::size_t>(lo[0] + x) * sx + static_cast<std::size_t>(lo[1] + y) * sy +
                                     static_cast<std::size_t>(lo[2] + z)];
                    if (blend > 0) {
                        const float k = std::max(blend - std::fabs(f - sd), 0.0f) / blend;
                        f = std::min(f, sd) - k * k * blend * 0.25f;
                    } else {
                        f = std::min(f, sd);
                    }
                }
    }
    // Rounds over what is finer than `sigma` (the flats of a faceted primitive, small
    // dents): a Gaussian of the field, as [1 2 1] passes (each adds h^2 / 2 of variance).
    if (sigma > 0) {
        const int passes = std::clamp(static_cast<int>(std::lround(2.0f * sigma * sigma / (h * h))), 1, 64);
        std::vector<float> tmp(field.size());
        for (int pass = 0; pass < passes; ++pass)
            for (int axis = 0; axis < 3; ++axis) {
                const int len = axis == 0 ? nx : axis == 1 ? ny : nz;
                const std::size_t stride = axis == 0 ? sx : axis == 1 ? sy : 1;
                for (int x = 0; x < nx; ++x)
                    for (int y = 0; y < ny; ++y)
                        for (int z = 0; z < nz; ++z) {
                            const std::size_t i = static_cast<std::size_t>(x) * sx + static_cast<std::size_t>(y) * sy + static_cast<std::size_t>(z);
                            const int c = axis == 0 ? x : axis == 1 ? y : z;
                            const float lo = c > 0 ? field[i - stride] : field[i], hi = c + 1 < len ? field[i + stride] : field[i];
                            tmp[i] = (lo + field[i] * 2.0f + hi) * 0.25f;
                        }
                field.swap(tmp);
            }
    }
    MeshData mesh = marchTets(field, nx, ny, nz, origin, h, true);
    if (mesh.indices.empty()) {
        if (error) *error = "Nothing solid to remesh in " + target.name;
        return false;
    }
    finishMarched(mesh, 2);
    simplifyMesh(mesh, h * 0.9f); // marching leaves slivers, and more triangles than the voxels hold
    const Vec3 at = target.offset;
    store(target, std::move(mesh));
    target.mesh.transform(translate(-at));
    target.offset = at;
    return true;
}

} // namespace sicdm

#include "core/Multires.hpp"

#include "core/Geometry.hpp"
#include "core/Scene.hpp"

#include <algorithm>
#include <map>
#include <tuple>
#include <unordered_map>

namespace sicdm {

namespace {

std::uint64_t edgeKeyOf(std::uint32_t a, std::uint32_t b) {
    if (a > b) std::swap(a, b);
    return (static_cast<std::uint64_t>(a) << 32) | b;
}

// The vertices numbered in the order the triangles first use them: a mesh and the same
// mesh written out again (by the sculpt tools) then number them alike.
MultiresLevel canonical(const std::vector<Vec3>& p, const std::vector<std::array<std::uint32_t, 3>>& t) {
    MultiresLevel out;
    std::vector<std::uint32_t> id(p.size(), 0xFFFFFFFFu);
    out.t.reserve(t.size());
    for (const auto& tri : t) {
        std::array<std::uint32_t, 3> n{};
        for (int c = 0; c < 3; ++c) {
            std::uint32_t& k = id[tri[static_cast<std::size_t>(c)]];
            if (k == 0xFFFFFFFFu) {
                k = static_cast<std::uint32_t>(out.p.size());
                out.p.push_back(p[tri[static_cast<std::size_t>(c)]]);
            }
            n[static_cast<std::size_t>(c)] = k;
        }
        out.t.push_back(n);
    }
    return out;
}

// A mesh as a level: corners at the same position are one vertex.
MultiresLevel levelOf(const MeshData& m) {
    std::map<std::tuple<float, float, float>, std::uint32_t> at;
    std::vector<Vec3> p;
    std::vector<std::uint32_t> id(m.vertices.size());
    for (std::size_t i = 0; i < m.vertices.size(); ++i) {
        const Vec3& q = m.vertices[i].position;
        const auto [it, fresh] = at.try_emplace({q.x, q.y, q.z}, static_cast<std::uint32_t>(p.size()));
        if (fresh) p.push_back(q);
        id[i] = it->second;
    }
    std::vector<std::array<std::uint32_t, 3>> t;
    for (std::size_t k = 0; k + 2 < m.indices.size(); k += 3) {
        const std::array<std::uint32_t, 3> tri{id[m.indices[k]], id[m.indices[k + 1]], id[m.indices[k + 2]]};
        if (tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0]) continue;
        t.push_back(tri);
    }
    return canonical(p, t);
}

std::vector<Vec3> vertexNormals(const MultiresLevel& l) {
    std::vector<Vec3> n(l.p.size(), Vec3{});
    for (const auto& t : l.t) {
        const Vec3 f = cross(l.p[t[1]] - l.p[t[0]], l.p[t[2]] - l.p[t[0]]); // area weighted
        for (std::uint32_t v : t) n[v] += f;
    }
    for (Vec3& x : n) x = length(x) > 0 ? normalize(x) : Vec3{0, 0, 1};
    return n;
}

std::shared_ptr<MultiresData> copyOf(const Body& body) {
    return body.multires ? std::make_shared<MultiresData>(*body.multires) : std::make_shared<MultiresData>();
}

bool fail(std::string* why, const std::string& text) {
    if (why) *why = text;
    return false;
}

} // namespace

MultiresLevel loopSubdivideLevel(const MultiresLevel& in) {
    // Loop's scheme: a new point on each edge, 3/8 of its ends and 1/8 of the two corners
    // across it (on a border, the middle); each old point moves towards its neighbours by
    // Warren's weight (on a border: 3/4 itself, 1/8 each border neighbour).
    struct EdgeRec {
        std::uint32_t opp[2] = {0xFFFFFFFFu, 0xFFFFFFFFu};
        int count = 0;
        std::uint32_t mid = 0;
    };
    std::map<std::uint64_t, EdgeRec> edges; // ordered: the new points come in a fixed order
    for (const auto& t : in.t)
        for (int e = 0; e < 3; ++e) {
            EdgeRec& r = edges[edgeKeyOf(t[static_cast<std::size_t>(e)], t[static_cast<std::size_t>((e + 1) % 3)])];
            if (r.count < 2) r.opp[r.count] = t[static_cast<std::size_t>((e + 2) % 3)];
            ++r.count;
        }
    const std::size_t V = in.p.size();
    std::vector<Vec3> ring(V, Vec3{}), border(V, Vec3{});
    std::vector<int> valence(V, 0), borders(V, 0);
    for (const auto& [k, r] : edges) {
        const auto a = static_cast<std::uint32_t>(k >> 32), b = static_cast<std::uint32_t>(k & 0xFFFFFFFFu);
        ring[a] += in.p[b], ring[b] += in.p[a];
        ++valence[a], ++valence[b];
        if (r.count != 2) {
            border[a] += in.p[b], border[b] += in.p[a];
            ++borders[a], ++borders[b];
        }
    }
    std::vector<Vec3> p(V);
    for (std::size_t v = 0; v < V; ++v) {
        if (borders[v] >= 2) p[v] = in.p[v] * 0.75f + border[v] * (0.25f / static_cast<float>(borders[v]));
        else if (valence[v] > 0) {
            const float n = static_cast<float>(valence[v]);
            const float beta = valence[v] == 3 ? 3.0f / 16.0f : 3.0f / (8.0f * n);
            p[v] = in.p[v] * (1.0f - n * beta) + ring[v] * beta;
        } else p[v] = in.p[v];
    }
    for (auto& [k, r] : edges) {
        const auto a = static_cast<std::uint32_t>(k >> 32), b = static_cast<std::uint32_t>(k & 0xFFFFFFFFu);
        r.mid = static_cast<std::uint32_t>(p.size());
        p.push_back(r.count == 2 ? (in.p[a] + in.p[b]) * 0.375f + (in.p[r.opp[0]] + in.p[r.opp[1]]) * 0.125f : (in.p[a] + in.p[b]) * 0.5f);
    }
    std::vector<std::array<std::uint32_t, 3>> t;
    t.reserve(in.t.size() * 4);
    for (const auto& tri : in.t) {
        const std::uint32_t ab = edges[edgeKeyOf(tri[0], tri[1])].mid, bc = edges[edgeKeyOf(tri[1], tri[2])].mid, ca = edges[edgeKeyOf(tri[2], tri[0])].mid;
        t.push_back({tri[0], ab, ca});
        t.push_back({ab, tri[1], bc});
        t.push_back({ca, bc, tri[2]});
        t.push_back({ab, bc, ca});
    }
    return canonical(p, t);
}

MeshData multiresMesh(const MultiresLevel& level) {
    MeshData m;
    const std::vector<Vec3> n = vertexNormals(level);
    m.vertices.resize(level.p.size());
    for (std::size_t v = 0; v < level.p.size(); ++v) m.vertices[v].position = level.p[v], m.vertices[v].normal = n[v];
    m.indices.reserve(level.t.size() * 3);
    for (const auto& t : level.t) m.indices.insert(m.indices.end(), {t[0], t[1], t[2]});
    m.ids.resize(level.p.size());
    for (std::size_t v = 0; v < m.ids.size(); ++v) m.ids[v] = static_cast<std::uint32_t>(v); // the sculpt tools carry these
    AABB box;
    for (const Vec3& q : level.p) box.expand(q);
    m.computeFeatureEdges(40.0f, box.valid() ? length(box.max - box.min) * 0.006f : 0.0f, 75.0f);
    return m;
}

bool multiresAdd(Body& body, int count, std::string* why, std::size_t maxTriangles) {
    if (body.mesh.indices.empty()) return fail(why, body.name + " has no surface");
    if (count < 1) return fail(why, "Add at least one level");
    if (body.multires && !multiresSync(body, why)) return false;
    auto md = copyOf(body);
    if (md->levels.empty()) {
        md->levels.push_back(levelOf(body.mesh));
        if (md->levels[0].t.empty()) return fail(why, body.name + " has no triangles to subdivide");
    } else if (md->current != static_cast<int>(md->levels.size()) - 1)
        return fail(why, "Levels are added above the top: show the top level first (MULTIRES LEVEL " + std::to_string(md->levels.size() - 1) + ")");
    for (int k = 0; k < count; ++k) {
        if (md->levels.back().t.size() * 4 > maxTriangles) return fail(why, "That many levels would pass " + std::to_string(maxTriangles) + " triangles");
        md->levels.push_back(loopSubdivideLevel(md->levels.back()));
    }
    md->current = static_cast<int>(md->levels.size()) - 1;
    body.mesh = multiresMesh(md->levels.back());
    body.kind = "mesh";
    body.params.clear();
    body.multires = std::move(md);
    return true;
}

bool multiresSync(Body& body, std::string* why) {
    if (!body.multires || body.multires->levels.empty()) return true;
    const int k = std::clamp(body.multires->current, 0, static_cast<int>(body.multires->levels.size()) - 1);
    // The level's vertices where the mesh has them now, found by their ids; the mesh's
    // triangles must still be the level's.
    const MultiresLevel& was = body.multires->levels[static_cast<std::size_t>(k)];
    const MeshData& m = body.mesh;
    MultiresLevel now;
    bool same = m.hasIds() && m.indices.size() == was.t.size() * 3;
    if (same) {
        now.p.assign(was.p.size(), Vec3{});
        std::vector<char> seen(was.p.size(), 0);
        for (std::size_t i = 0; i < m.vertices.size() && same; ++i) {
            const std::uint32_t id = m.ids[i];
            if (id >= was.p.size()) same = false;
            else now.p[id] = m.vertices[i].position, seen[id] = 1;
        }
        same = same && std::all_of(seen.begin(), seen.end(), [](char c) { return c != 0; });
        if (same) {
            auto sorted = [](std::array<std::uint32_t, 3> t) {
                std::sort(t.begin(), t.end());
                return t;
            };
            std::vector<std::array<std::uint32_t, 3>> a, b;
            for (const auto& t : was.t) a.push_back(sorted(t));
            for (std::size_t q = 0; q + 2 < m.indices.size(); q += 3) b.push_back(sorted({m.ids[m.indices[q]], m.ids[m.indices[q + 1]], m.ids[m.indices[q + 2]]}));
            std::sort(a.begin(), a.end());
            std::sort(b.begin(), b.end());
            same = a == b;
        }
    }
    if (!same) {
        body.multires.reset();
        return fail(why, body.name + "'s triangles changed (dynamic topology, a remesh or a cut), so its levels were dropped; it keeps the shape it has");
    }
    auto md = copyOf(body);
    std::vector<Vec3> oldBelow = md->levels[static_cast<std::size_t>(k)].p; // level j-1 before the edit
    md->levels[static_cast<std::size_t>(k)].p = std::move(now.p);
    for (std::size_t j = static_cast<std::size_t>(k) + 1; j < md->levels.size(); ++j) {
        // Each finer level: its detail over the subdivided level below, carried from the old
        // surface to the new one in the normal's direction (and as it was across it).
        MultiresLevel below = md->levels[j - 1];
        const MultiresLevel sNew = loopSubdivideLevel(below);
        below.p = oldBelow;
        const MultiresLevel sOld = loopSubdivideLevel(below);
        MultiresLevel& L = md->levels[j];
        if (sNew.p.size() != L.p.size() || sOld.p.size() != L.p.size()) {
            body.multires.reset();
            return fail(why, body.name + "'s levels no longer match one another; they were dropped");
        }
        const std::vector<Vec3> nOld = vertexNormals(sOld), nNew = vertexNormals(sNew);
        oldBelow = L.p;
        for (std::size_t v = 0; v < L.p.size(); ++v) {
            const Vec3 d = L.p[v] - sOld.p[v];
            const float along = dot(d, nOld[v]);
            L.p[v] = sNew.p[v] + nNew[v] * along + (d - nOld[v] * along);
        }
    }
    body.multires = std::move(md);
    return true;
}

bool multiresLevel(Body& body, int level, std::string* why) {
    if (!body.multires || body.multires->levels.empty()) return fail(why, body.name + " has no levels (MULTIRES ADD makes them)");
    if (!multiresSync(body, why)) return false;
    if (level < 0 || level >= static_cast<int>(body.multires->levels.size()))
        return fail(why, "Levels are 0 to " + std::to_string(body.multires->levels.size() - 1));
    auto md = copyOf(body);
    md->current = level;
    body.mesh = multiresMesh(md->levels[static_cast<std::size_t>(level)]);
    body.multires = std::move(md);
    return true;
}

bool multiresApply(Body& body, std::string* why) {
    if (!body.multires) return fail(why, body.name + " has no levels");
    if (!multiresSync(body, why)) return false;
    body.multires.reset();
    return true;
}

bool multiresDeleteLower(Body& body, std::string* why) {
    if (!body.multires) return fail(why, body.name + " has no levels");
    if (!multiresSync(body, why)) return false;
    auto md = copyOf(body);
    md->levels.erase(md->levels.begin(), md->levels.begin() + md->current);
    md->current = 0;
    if (md->levels.size() <= 1) body.multires.reset();
    else body.multires = std::move(md);
    return true;
}

bool multiresDeleteHigher(Body& body, std::string* why) {
    if (!body.multires) return fail(why, body.name + " has no levels");
    if (!multiresSync(body, why)) return false;
    auto md = copyOf(body);
    md->levels.resize(static_cast<std::size_t>(md->current) + 1);
    if (md->levels.size() <= 1) body.multires.reset();
    else body.multires = std::move(md);
    return true;
}

} // namespace sicdm

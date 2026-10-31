#include "core/Geometry.hpp"
#include "core/Airfoil.hpp"
#include "core/Armature.hpp"
#include "core/SheetMetal.hpp"

#include "core/Extrude.hpp"
#include "core/Library.hpp"

#include <cmath>
#include <unordered_set>
#include <cstring>
#include <algorithm>
#include <cstdint>
#include <unordered_map>

namespace sicdm {

AABB MeshData::bounds() const noexcept {
    AABB b;
    for (const Vertex& v : vertices) b.expand(v.position);
    for (const Vec3& p : edgeVertices) b.expand(p);
    return b;
}

void MeshData::transform(const Mat4& m) {
    // Normal matrix = inverse-transpose of the upper 3x3.
    const Mat4 inv = inverse(m);
    auto xformNormal = [&inv](const Vec3& n) {
        return normalize(Vec3{
            inv.m[0][0] * n.x + inv.m[0][1] * n.y + inv.m[0][2] * n.z,
            inv.m[1][0] * n.x + inv.m[1][1] * n.y + inv.m[1][2] * n.z,
            inv.m[2][0] * n.x + inv.m[2][1] * n.y + inv.m[2][2] * n.z,
        });
    };
    for (Vertex& v : vertices) {
        v.position = m.transformPoint(v.position);
        v.normal = xformNormal(v.normal);
    }
    for (Vec3& p : edgeVertices) p = m.transformPoint(p);
}

std::uint32_t packColor(const Vec3& rgb) noexcept {
    auto byte = [](float v) { return static_cast<std::uint32_t>(std::clamp(v, 0.0f, 1.0f) * 255.0f + 0.5f); };
    return byte(rgb.x) | (byte(rgb.y) << 8) | (byte(rgb.z) << 16) | 0xFF000000u;
}

Vec3 unpackColor(std::uint32_t c) noexcept {
    return {static_cast<float>(c & 0xFF) / 255.0f, static_cast<float>((c >> 8) & 0xFF) / 255.0f, static_cast<float>((c >> 16) & 0xFF) / 255.0f};
}

void MeshData::append(const MeshData& o) {
    const auto base = static_cast<std::uint32_t>(vertices.size());
    if (hasGroups() || o.hasGroups()) { // groups kept, the other's numbered after ours
        std::uint32_t top = 0;
        if (hasGroups())
            for (std::uint32_t g : groups) top = std::max(top, g + 1);
        else groups.assign(indices.size() / 3, 0), top = indices.empty() ? 0 : 1;
        if (o.hasGroups())
            for (std::uint32_t g : o.groups) groups.push_back(g + top);
        else groups.insert(groups.end(), o.indices.size() / 3, top);
    }
    if (indices.empty() && o.hasUvs()) uvs = o.uvs;
    else if (hasUvs() && o.hasUvs()) uvs.insert(uvs.end(), o.uvs.begin(), o.uvs.end());
    else uvs.clear(); // only when both have them
    vertices.insert(vertices.end(), o.vertices.begin(), o.vertices.end());
    indices.reserve(indices.size() + o.indices.size());
    for (std::uint32_t i : o.indices) indices.push_back(base + i);
    edgeVertices.insert(edgeVertices.end(), o.edgeVertices.begin(), o.edgeVertices.end());
}

std::uint64_t meshContentKey(const MeshData& m) noexcept {
    // Word at a time (a few GB/s): multiply-rotate mixing, as in the common 64-bit hashes.
    constexpr std::uint64_t P1 = 0x9E3779B185EBCA87ull, P2 = 0xC2B2AE3D27D4EB4Full;
    std::uint64_t h = 0x27D4EB2F165667C5ull ^ (m.vertices.size() * P1) ^ (m.indices.size() * P2);
    auto mix = [&](const void* data, std::size_t bytes) {
        const auto* c = static_cast<const unsigned char*>(data);
        std::size_t i = 0;
        for (; i + 8 <= bytes; i += 8) {
            std::uint64_t w;
            std::memcpy(&w, c + i, 8);
            w *= P2;
            w = (w << 31) | (w >> 33);
            h ^= w * P1;
            h = ((h << 27) | (h >> 37)) * P1 + 0x85EBCA77C2B2AE63ull;
        }
        std::uint64_t tail = 0;
        if (i < bytes) std::memcpy(&tail, c + i, bytes - i);
        h ^= tail * P2;
    };
    if (!m.vertices.empty()) mix(m.vertices.data(), m.vertices.size() * sizeof(Vertex));
    if (!m.indices.empty()) mix(m.indices.data(), m.indices.size() * sizeof(std::uint32_t));
    if (!m.groups.empty()) mix(m.groups.data(), m.groups.size() * sizeof(std::uint32_t));
    if (!m.uvs.empty()) mix(m.uvs.data(), m.uvs.size() * sizeof(Vec2));
    h ^= h >> 33;
    h *= P2;
    h ^= h >> 29;
    return h;
}

MeshData clusterMesh(const MeshData& m, float cell) {
    MeshData out;
    if (m.indices.empty() || !(cell > 0)) return m;
    const AABB b = m.bounds();
    const Vec3 ext = b.max - b.min;
    cell = std::max(cell, std::max({ext.x, ext.y, ext.z}) / 200000.0f); // 18 bits of cells a side
    // The cell, and which way the vertex faces (so the two sides of a thin wall don't merge).
    auto key = [&](const Vertex& v) {
        const auto q = [&](float x, float lo) { return std::min<std::uint64_t>(static_cast<std::uint64_t>(std::max(0.0f, std::floor((x - lo) / cell))), 0x3FFFF); };
        // The normal rounded to one of 5 x 5 x 5 directions: faces meeting at more than about
        // 35 degrees keep their own vertices (and their shading).
        const auto d = [](float c) { return static_cast<std::uint64_t>(std::lround(std::clamp(c, -1.0f, 1.0f) * 2.0f) + 2); };
        const std::uint64_t side = d(v.normal.x) * 25 + d(v.normal.y) * 5 + d(v.normal.z); // 0..124
        return (side << 54) | (q(v.position.x, b.min.x) << 36) | (q(v.position.y, b.min.y) << 18) | q(v.position.z, b.min.z);
    };
    std::unordered_map<std::uint64_t, std::uint32_t> cluster;
    cluster.reserve(m.vertices.size() / 4 + 16);
    std::vector<std::uint32_t> id(m.vertices.size());
    std::vector<Vec3> sumP, sumN;
    std::vector<float> count;
    for (std::size_t i = 0; i < m.vertices.size(); ++i) {
        const auto [it, fresh] = cluster.try_emplace(key(m.vertices[i]), static_cast<std::uint32_t>(sumP.size()));
        if (fresh) sumP.push_back({}), sumN.push_back({}), count.push_back(0);
        const std::uint32_t c = it->second;
        sumP[c] += m.vertices[i].position;
        sumN[c] += m.vertices[i].normal;
        count[c] += 1;
        id[i] = c;
    }
    out.vertices.resize(sumP.size());
    for (std::size_t c = 0; c < sumP.size(); ++c) {
        out.vertices[c].position = sumP[c] / count[c];
        out.vertices[c].normal = normalize(sumN[c]);
    }
    for (std::size_t i = 0; i < m.vertices.size(); ++i) // the paint: a painted corner's colour
        if (painted(m.vertices[i].color)) out.vertices[id[i]].color = m.vertices[i].color;
    struct Tri3 {
        std::uint32_t a, b, c;
        bool operator==(const Tri3& o) const { return a == o.a && b == o.b && c == o.c; }
    };
    struct TriHash {
        std::size_t operator()(const Tri3& t) const { return (static_cast<std::size_t>(t.a) * 73856093u) ^ (static_cast<std::size_t>(t.b) * 19349663u) ^ (static_cast<std::size_t>(t.c) * 83492791u); }
    };
    std::unordered_set<Tri3, TriHash> seen;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        std::uint32_t a = id[m.indices[t]], c1 = id[m.indices[t + 1]], c2 = id[m.indices[t + 2]];
        if (a == c1 || c1 == c2 || c2 == a) continue;
        // The same triangle turned round to start at its least index (keeps its winding).
        while (a > c1 || a > c2) {
            const std::uint32_t x = a;
            a = c1, c1 = c2, c2 = x;
        }
        if (!seen.insert({a, c1, c2}).second) continue;
        out.indices.insert(out.indices.end(), {a, c1, c2});
    }
    return out; // no edge lines: a drawing keeps the full shape's
}

void MeshData::computeFeatureEdges(float angleDeg, float shortLength, float shortAngleDeg) {
    // Weld coincident positions so faces that share an edge find each other (ids in the
    // order positions first appear, so the result doesn't depend on hashing).
    struct Key {
        long x, y, z;
        bool operator==(const Key& o) const { return x == o.x && y == o.y && z == o.z; }
    };
    struct KeyHash {
        std::size_t operator()(const Key& k) const {
            std::uint64_t h = static_cast<std::uint64_t>(k.x) * 0x9E3779B97F4A7C15ull;
            h ^= static_cast<std::uint64_t>(k.y) + 0x7F4A7C159E3779B9ull + (h << 6) + (h >> 2);
            h ^= static_cast<std::uint64_t>(k.z) + 0x94D049BB133111EBull + (h << 6) + (h >> 2);
            return static_cast<std::size_t>(h);
        }
    };
    std::unordered_map<Key, std::uint32_t, KeyHash> weld;
    weld.reserve(vertices.size());
    std::vector<Vec3> welded;
    welded.reserve(vertices.size());
    std::vector<std::uint32_t> remap(vertices.size());
    for (std::size_t i = 0; i < vertices.size(); ++i) {
        constexpr float q = 1e4f; // 0.1 micron
        const Vec3& p = vertices[i].position;
        const auto [it, added] = weld.try_emplace(Key{std::lround(p.x * q), std::lround(p.y * q), std::lround(p.z * q)}, static_cast<std::uint32_t>(welded.size()));
        if (added) welded.push_back(p);
        remap[i] = it->second;
    }
    // Each triangle's edges with its normal, sorted by edge (stable: the first face of an
    // edge is still the first in the mesh).
    struct EdgeFace {
        std::uint64_t key;
        Vec3 n;
    };
    std::vector<EdgeFace> ef;
    ef.reserve(indices.size());
    for (std::size_t t = 0; t + 2 < indices.size(); t += 3) {
        const std::uint32_t w[3] = {remap[indices[t]], remap[indices[t + 1]], remap[indices[t + 2]]};
        const Vec3 n = normalize(cross(welded[w[1]] - welded[w[0]], welded[w[2]] - welded[w[0]]));
        if (length(n) == 0.0f) continue;
        for (int e = 0; e < 3; ++e) {
            auto a = w[e], b = w[(e + 1) % 3];
            if (a > b) std::swap(a, b);
            ef.push_back({(static_cast<std::uint64_t>(a) << 32) | b, n});
        }
    }
    std::stable_sort(ef.begin(), ef.end(), [](const EdgeFace& x, const EdgeFace& y) { return x.key < y.key; });
    const float cosLimit = std::cos(radians(angleDeg)), cosShort = std::cos(radians(std::max(shortAngleDeg, angleDeg)));
    edgeVertices.clear();
    for (std::size_t i = 0; i < ef.size();) {
        std::size_t j = i + 1;
        bool sharp = false;
        const bool shortEdge = shortLength > 0 &&
                               length(welded[static_cast<std::uint32_t>(ef[i].key >> 32)] - welded[static_cast<std::uint32_t>(ef[i].key & 0xFFFFFFFFu)]) < shortLength;
        for (; j < ef.size() && ef[j].key == ef[i].key; ++j)
            if (dot(ef[i].n, ef[j].n) < (shortEdge ? cosShort : cosLimit)) sharp = true;
        const std::size_t count = j - i;
        if (count == 1 || sharp || count > 2) {
            edgeVertices.push_back(welded[static_cast<std::uint32_t>(ef[i].key >> 32)]);
            edgeVertices.push_back(welded[static_cast<std::uint32_t>(ef[i].key & 0xFFFFFFFFu)]);
        }
        i = j;
    }
}

bool isPrimitiveKind(const std::string& k) {
    return k == "box" || k == "wedge" || k == "cylinder" || k == "tube" || k == "cone" || k == "sphere" ||
           k == "torus" || k == "pyramid" || k == "prism" || k == "ellipsoid" || k == "capsule" || k == "hemisphere" || k == "spring" || k == "blade" || k == "extrude" || k == "sheet" || k == "armature" || isLibraryKind(k);
}

MeshData buildPrimitive(const std::string& k, const std::vector<float>& p) {
    using namespace primitives;
    if (k == "extrude") return buildExtrusion(p);
    if (k == "sheet") return buildSheetParams(p);
    if (k == "armature") {
        Armature a;
        return armatureFromParams(p, a) ? armaturePreview(a) : MeshData{};
    }
    if (isLibraryKind(k)) return buildLibraryPart(k, p);
    auto need = [&](std::size_t n) { return p.size() >= n; };
    auto positive = [&](std::size_t n) {
        for (std::size_t i = 0; i < n; ++i)
            if (!(p[i] > 0.0f)) return false;
        return true;
    };
    if ((k == "box" || k == "wedge") && need(6)) {
        const Vec3 lo{std::min(p[0], p[3]), std::min(p[1], p[4]), std::min(p[2], p[5])};
        const Vec3 hi{std::max(p[0], p[3]), std::max(p[1], p[4]), std::max(p[2], p[5])};
        if (hi.x - lo.x <= 0 || hi.y - lo.y <= 0 || hi.z - lo.z <= 0) return {};
        return k == "box" ? box(lo, hi) : wedge(lo, hi);
    }
    if (k == "cylinder" && need(2) && positive(2)) return cylinder({0, 0, 0}, p[0], p[1]);
    if (k == "tube" && need(3) && positive(3) && p[1] < p[0]) // turned, so its seams meet exactly
        return lathe({{{p[0], 0, 1, 0}, {p[0], p[2], 1, 0}},
                      {{p[0], p[2], 0, 1}, {p[1], p[2], 0, 1}},
                      {{p[1], p[2], -1, 0}, {p[1], 0, -1, 0}},
                      {{p[1], 0, 0, -1}, {p[0], 0, 0, -1}}});
    if (k == "cone" && need(3) && p[0] > 0 && p[1] >= 0 && p[2] > 0) return cone({0, 0, 0}, p[0], p[1], p[2]);
    if (k == "sphere" && need(1) && positive(1)) return sphere({0, 0, p[0]}, p[0]);
    if (k == "torus" && need(2) && positive(2) && p[1] < p[0]) return torus({0, 0, p[1]}, p[0], p[1]);
    if (k == "ellipsoid" && need(3) && positive(3)) return ellipsoid(p[0], p[1], p[2]);
    if (k == "blade") return buildBlade(p);
    if (k == "capsule" && need(2) && positive(2) && p[1] >= 2.0f * p[0]) return capsule(p[0], p[1]);
    if (k == "hemisphere" && need(1) && positive(1)) return hemisphere(p[0]);
    if (k == "spring" && need(4) && positive(4) && p[1] < p[0] && p[2] > 2.0f * p[1] && p[3] <= 500.0f)
        return spring(p[0], p[1], p[2], p[3], p.size() > 4 && p[4] > 0.5f);
    if ((k == "pyramid" || k == "prism") && need(2) && positive(2)) {
        const int sides = p.size() >= 3 ? std::clamp(static_cast<int>(p[2]), 3, 64) : (k == "prism" ? 6 : 4);
        return k == "pyramid" ? pyramid({0, 0, 0}, p[0], p[1], sides) : prism({0, 0, 0}, p[0], p[1], sides);
    }
    return {};
}

namespace primitives {

namespace {

void addQuad(MeshData& m, const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d, const Vec3& n) {
    const auto base = static_cast<std::uint32_t>(m.vertices.size());
    m.vertices.push_back({a, n});
    m.vertices.push_back({b, n});
    m.vertices.push_back({c, n});
    m.vertices.push_back({d, n});
    m.indices.insert(m.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
}

void addEdge(MeshData& m, const Vec3& a, const Vec3& b) {
    m.edgeVertices.push_back(a);
    m.edgeVertices.push_back(b);
}

Vec3 ring(const Vec3& c, float r, float angle, float z) {
    return {c.x + r * std::cos(angle), c.y + r * std::sin(angle), c.z + z};
}

// Flat annulus / disk cap at height z. inner == 0 makes a disk.
void addCap(MeshData& m, const Vec3& base, float outer, float inner, float z, int seg, bool facingUp) {
    const Vec3 n{0, 0, facingUp ? 1.0f : -1.0f};
    const float step = 2.0f * kPi / static_cast<float>(seg);
    for (int i = 0; i < seg; ++i) {
        const float a0 = step * static_cast<float>(i);
        const float a1 = step * static_cast<float>(i + 1);
        const Vec3 o0 = ring(base, outer, a0, z), o1 = ring(base, outer, a1, z);
        const Vec3 i0 = ring(base, inner, a0, z), i1 = ring(base, inner, a1, z);
        if (facingUp) addQuad(m, i0, o0, o1, i1, n);
        else addQuad(m, i0, i1, o1, o0, n);
    }
}

// Side wall; outward == false for an inner (hole) wall.
void addWall(MeshData& m, const Vec3& base, float r, float h, int seg, bool outward) {
    const float step = 2.0f * kPi / static_cast<float>(seg);
    for (int i = 0; i < seg; ++i) {
        const float a0 = step * static_cast<float>(i);
        const float a1 = step * static_cast<float>(i + 1);
        const Vec3 b0 = ring(base, r, a0, 0), b1 = ring(base, r, a1, 0);
        const Vec3 t0 = ring(base, r, a0, h), t1 = ring(base, r, a1, h);
        const float s = outward ? 1.0f : -1.0f;
        const Vec3 n0{s * std::cos(a0), s * std::sin(a0), 0};
        const Vec3 n1{s * std::cos(a1), s * std::sin(a1), 0};

        const auto v = static_cast<std::uint32_t>(m.vertices.size());
        m.vertices.push_back({b0, n0});
        m.vertices.push_back({b1, n1});
        m.vertices.push_back({t1, n1});
        m.vertices.push_back({t0, n0});
        if (outward) m.indices.insert(m.indices.end(), {v, v + 1, v + 2, v, v + 2, v + 3});
        else m.indices.insert(m.indices.end(), {v, v + 2, v + 1, v, v + 3, v + 2});

        addEdge(m, b0, b1); // circular edges at bottom and top
        addEdge(m, t0, t1);
    }
}

// Planar convex face. Winding and normal are oriented away from `inside`,
// a point in the interior of the (convex) solid; boundary edges are emitted.
void addConvexFace(MeshData& m, std::vector<Vec3> pts, const Vec3& inside) {
    if (pts.size() < 3) return;
    Vec3 c{};
    for (const Vec3& p : pts) c += p;
    c = c / static_cast<float>(pts.size());
    Vec3 n = normalize(cross(pts[1] - pts[0], pts[2] - pts[0]));
    if (dot(n, c - inside) < 0.0f) {
        std::reverse(pts.begin(), pts.end());
        n = -n;
    }
    const auto base = static_cast<std::uint32_t>(m.vertices.size());
    for (const Vec3& p : pts) m.vertices.push_back({p, n});
    for (std::uint32_t i = 1; i + 1 < pts.size(); ++i) m.indices.insert(m.indices.end(), {base, base + i, base + i + 1});
    for (std::size_t i = 0; i < pts.size(); ++i) addEdge(m, pts[i], pts[(i + 1) % pts.size()]);
}

std::vector<Vec3> regularPolygon(const Vec3& c, float r, int sides, float z) {
    std::vector<Vec3> pts;
    pts.reserve(static_cast<std::size_t>(sides));
    const float step = 2.0f * kPi / static_cast<float>(sides);
    const float start = kPi * 0.5f - step * 0.5f; // flat edge facing -Y
    for (int i = 0; i < sides; ++i) pts.push_back(ring(c, r, start - step * static_cast<float>(i), z));
    return pts;
}

} // namespace

MeshData box(const Vec3& lo, const Vec3& hi) {
    MeshData m;
    const Vec3 p[8] = {
        {lo.x, lo.y, lo.z}, {hi.x, lo.y, lo.z}, {hi.x, hi.y, lo.z}, {lo.x, hi.y, lo.z},
        {lo.x, lo.y, hi.z}, {hi.x, lo.y, hi.z}, {hi.x, hi.y, hi.z}, {lo.x, hi.y, hi.z},
    };
    addQuad(m, p[0], p[3], p[2], p[1], {0, 0, -1}); // bottom
    addQuad(m, p[4], p[5], p[6], p[7], {0, 0, 1});  // top
    addQuad(m, p[0], p[1], p[5], p[4], {0, -1, 0}); // front
    addQuad(m, p[2], p[3], p[7], p[6], {0, 1, 0});  // back
    addQuad(m, p[1], p[2], p[6], p[5], {1, 0, 0});  // right
    addQuad(m, p[3], p[0], p[4], p[7], {-1, 0, 0}); // left

    constexpr int e[12][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6},
                              {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    for (const auto& ed : e) addEdge(m, p[ed[0]], p[ed[1]]);
    return m;
}

MeshData cylinder(const Vec3& base, float radius, float height, int segments) {
    MeshData m;
    addWall(m, base, radius, height, segments, true);
    addCap(m, base, radius, 0.0f, 0.0f, segments, false);
    addCap(m, base, radius, 0.0f, height, segments, true);
    return m;
}

MeshData tube(const Vec3& base, float outerR, float innerR, float height, int segments) {
    MeshData m;
    addWall(m, base, outerR, height, segments, true);
    addWall(m, base, innerR, height, segments, false);
    addCap(m, base, outerR, innerR, 0.0f, segments, false);
    addCap(m, base, outerR, innerR, height, segments, true);
    return m;
}

MeshData sphere(const Vec3& c, float r, int segments, int rings) {
    MeshData m;
    for (int j = 0; j <= rings; ++j) {
        const float v = static_cast<float>(j) / static_cast<float>(rings);
        const float phi = v * kPi; // 0 at +Z pole
        for (int i = 0; i <= segments; ++i) {
            const float u = static_cast<float>(i) / static_cast<float>(segments);
            const float theta = u * 2.0f * kPi;
            const Vec3 n{std::sin(phi) * std::cos(theta), std::sin(phi) * std::sin(theta), std::cos(phi)};
            m.vertices.push_back({c + n * r, n});
        }
    }
    const auto row = static_cast<std::uint32_t>(segments + 1);
    for (int j = 0; j < rings; ++j)
        for (int i = 0; i < segments; ++i) {
            const std::uint32_t a = static_cast<std::uint32_t>(j) * row + static_cast<std::uint32_t>(i);
            const std::uint32_t b = a + row;
            // Skip the zero-area triangles that would touch the poles.
            if (j != 0) m.indices.insert(m.indices.end(), {a, b, a + 1});
            if (j != rings - 1) m.indices.insert(m.indices.end(), {a + 1, b, b + 1});
        }
    return m;
}

} // namespace primitives

} // namespace sicdm

namespace sicdm::primitives {

MeshData cone(const Vec3& base, float r0, float r1, float h, int seg) {
    MeshData m;
    const float step = 2.0f * kPi / static_cast<float>(seg);
    for (int i = 0; i < seg; ++i) {
        const float a0 = step * static_cast<float>(i);
        const float a1 = step * static_cast<float>(i + 1);
        const Vec3 b0 = ring(base, r0, a0, 0), b1 = ring(base, r0, a1, 0);
        const Vec3 t0 = ring(base, r1, a0, h), t1 = ring(base, r1, a1, h);
        // Side normal tilts up by the slope of the wall.
        const Vec3 n0 = normalize({std::cos(a0) * h, std::sin(a0) * h, r0 - r1});
        const Vec3 n1 = normalize({std::cos(a1) * h, std::sin(a1) * h, r0 - r1});
        const auto v = static_cast<std::uint32_t>(m.vertices.size());
        m.vertices.push_back({b0, n0});
        m.vertices.push_back({b1, n1});
        m.vertices.push_back({t1, n1});
        m.vertices.push_back({t0, n0});
        m.indices.insert(m.indices.end(), {v, v + 1, v + 2});
        if (r1 > 0.0f) m.indices.insert(m.indices.end(), {v, v + 2, v + 3});
        addEdge(m, b0, b1);
        if (r1 > 0.0f) addEdge(m, t0, t1);
    }
    addCap(m, base, r0, 0.0f, 0.0f, seg, false);
    if (r1 > 0.0f) addCap(m, base, r1, 0.0f, h, seg, true);
    return m;
}

MeshData torus(const Vec3& c, float R, float r, int seg, int rings) {
    MeshData m;
    for (int i = 0; i <= seg; ++i) {
        const float u = 2.0f * kPi * static_cast<float>(i) / static_cast<float>(seg);
        for (int j = 0; j <= rings; ++j) {
            const float v = 2.0f * kPi * static_cast<float>(j) / static_cast<float>(rings);
            const Vec3 n{std::cos(v) * std::cos(u), std::cos(v) * std::sin(u), std::sin(v)};
            const Vec3 p{(R + r * std::cos(v)) * std::cos(u), (R + r * std::cos(v)) * std::sin(u), r * std::sin(v)};
            m.vertices.push_back({c + p, n});
        }
    }
    const auto row = static_cast<std::uint32_t>(rings + 1);
    for (int i = 0; i < seg; ++i)
        for (int j = 0; j < rings; ++j) {
            const std::uint32_t a = static_cast<std::uint32_t>(i) * row + static_cast<std::uint32_t>(j);
            const std::uint32_t b = a + row; // next step around the major circle
            m.indices.insert(m.indices.end(), {a, b, a + 1, a + 1, b, b + 1});
        }
    return m;
}

MeshData wedge(const Vec3& lo, const Vec3& hi) {
    MeshData m;
    const Vec3 inside{lo.x + (hi.x - lo.x) * 0.25f, (lo.y + hi.y) * 0.5f, lo.z + (hi.z - lo.z) * 0.25f};
    const Vec3 a{lo.x, lo.y, lo.z}, b{hi.x, lo.y, lo.z}, c{lo.x, lo.y, hi.z};
    const Vec3 d{lo.x, hi.y, lo.z}, e{hi.x, hi.y, lo.z}, f{lo.x, hi.y, hi.z};
    addConvexFace(m, {a, b, c}, inside);       // front triangle
    addConvexFace(m, {d, e, f}, inside);       // back triangle
    addConvexFace(m, {a, b, e, d}, inside);    // bottom
    addConvexFace(m, {a, c, f, d}, inside);    // vertical back wall
    addConvexFace(m, {b, e, f, c}, inside);    // slope
    return m;
}

MeshData pyramid(const Vec3& base, float radius, float h, int sides) {
    MeshData m;
    const Vec3 inside = base + Vec3{0, 0, h * 0.25f};
    const Vec3 apex = base + Vec3{0, 0, h};
    const std::vector<Vec3> pts = regularPolygon(base, radius, sides, 0.0f);
    addConvexFace(m, pts, inside);
    for (std::size_t i = 0; i < pts.size(); ++i) addConvexFace(m, {pts[i], pts[(i + 1) % pts.size()], apex}, inside);
    return m;
}

MeshData prism(const Vec3& base, float radius, float h, int sides) {
    MeshData m;
    const Vec3 inside = base + Vec3{0, 0, h * 0.5f};
    const std::vector<Vec3> bottom = regularPolygon(base, radius, sides, 0.0f);
    const std::vector<Vec3> top = regularPolygon(base, radius, sides, h);
    addConvexFace(m, bottom, inside);
    addConvexFace(m, top, inside);
    for (std::size_t i = 0; i < bottom.size(); ++i) {
        const std::size_t k = (i + 1) % bottom.size();
        addConvexFace(m, {bottom[i], bottom[k], top[k], top[i]}, inside);
    }
    return m;
}

} // namespace sicdm::primitives

namespace sicdm::primitives {

MeshData lathe(const std::vector<LatheSpan>& spans, int seg) {
    MeshData m;
    seg = std::max(seg, 3);
    for (const LatheSpan& span : spans) {
        if (span.size() < 2) continue;
        const auto base = static_cast<std::uint32_t>(m.vertices.size());
        const auto row = static_cast<std::uint32_t>(seg + 1);
        for (const auto& q : span)
            for (int i = 0; i <= seg; ++i) {
                const float a = 2.0f * kPi * static_cast<float>(i % seg) / static_cast<float>(seg);
                const float c = std::cos(a), s = std::sin(a);
                m.vertices.push_back({{q[0] * c, q[0] * s, q[1]}, normalize(Vec3{q[2] * c, q[2] * s, q[3]})});
            }
        for (std::size_t j = 0; j + 1 < span.size(); ++j) {
            // A face's normal is (dz, -dr) along the profile: an outline counter-clockwise in
            // (radius, z) faces out.
            const bool lowPole = span[j][0] <= 0.0f, highPole = span[j + 1][0] <= 0.0f;
            for (int i = 0; i < seg; ++i) {
                const std::uint32_t a = base + static_cast<std::uint32_t>(j) * row + static_cast<std::uint32_t>(i);
                const std::uint32_t b = a + row;
                if (!lowPole) m.indices.insert(m.indices.end(), {a, a + 1, b + 1});
                if (!highPole) m.indices.insert(m.indices.end(), {a, b + 1, b});
            }
        }
    }
    return m;
}

MeshData ellipsoid(float a, float b, float c, int segments, int rings) {
    MeshData m = sphere({0, 0, 0}, 1.0f, segments, rings);
    for (Vertex& v : m.vertices) {
        const Vec3 p = v.position;
        v.position = {p.x * a, p.y * b, p.z * c + c};
        v.normal = normalize(Vec3{p.x / a, p.y / b, p.z / c});
    }
    return m;
}

MeshData capsule(float r, float length, int seg) {
    // One smooth span: the bottom half sphere, the wall, the top half sphere.
    LatheSpan span;
    const int quarter = std::max(4, seg / 4);
    for (int k = 0; k <= quarter; ++k) { // from the bottom pole out to the wall
        const float t = -0.5f * kPi + 0.5f * kPi * static_cast<float>(k) / static_cast<float>(quarter);
        span.push_back({r * std::cos(t), r + r * std::sin(t), std::cos(t), std::sin(t)});
    }
    if (length > 2.0f * r) span.push_back({r, length - r, 1, 0});
    for (int k = length > 2.0f * r ? 0 : 1; k <= quarter; ++k) { // the top
        const float t = 0.5f * kPi * static_cast<float>(k) / static_cast<float>(quarter);
        span.push_back({r * std::cos(t), length - r + r * std::sin(t), std::cos(t), std::sin(t)});
    }
    span.front()[0] = span.back()[0] = 0.0f;
    return lathe({span}, seg);
}

MeshData hemisphere(float r, int seg) {
    const LatheSpan base{{0, 0, 0, -1}, {r, 0, 0, -1}}; // out from the axis: facing down
    LatheSpan dome;
    const int quarter = std::max(4, seg / 4);
    for (int k = 0; k <= quarter; ++k) {
        const float t = 0.5f * kPi * static_cast<float>(k) / static_cast<float>(quarter);
        dome.push_back({r * std::cos(t), r * std::sin(t), std::cos(t), std::sin(t)});
    }
    dome.back()[0] = 0.0f;
    return lathe({base, dome}, seg);
}

MeshData spring(float R, float r, float pitch, float turns, bool left) {
    MeshData m;
    const int perTurn = 64, around = 24;
    const int steps = std::max(8, static_cast<int>(std::ceil(turns * static_cast<float>(perTurn))));
    const float hand = left ? -1.0f : 1.0f;
    // The wire's centre, its tangent, and a frame across it (out from the axis, and the binormal).
    auto at = [&](float t, Vec3& c, Vec3& tan, Vec3& out, Vec3& bin) {
        const float th = hand * 2.0f * kPi * t;
        out = {std::cos(th), std::sin(th), 0};
        c = Vec3{R * out.x, R * out.y, r + pitch * t};
        tan = normalize(Vec3{-R * std::sin(th) * 2.0f * kPi * hand, R * std::cos(th) * 2.0f * kPi * hand, pitch});
        bin = normalize(cross(tan, out));
    };
    const auto row = static_cast<std::uint32_t>(around + 1);
    for (int k = 0; k <= steps; ++k) {
        Vec3 c, tan, out, bin;
        at(turns * static_cast<float>(k) / static_cast<float>(steps), c, tan, out, bin);
        for (int i = 0; i <= around; ++i) {
            const float ph = 2.0f * kPi * static_cast<float>(i % around) / static_cast<float>(around);
            const Vec3 n = out * std::cos(ph) + bin * std::sin(ph);
            m.vertices.push_back({c + n * r, n});
        }
    }
    for (int k = 0; k < steps; ++k)
        for (int i = 0; i < around; ++i) {
            const std::uint32_t a = static_cast<std::uint32_t>(k) * row + static_cast<std::uint32_t>(i), b = a + row;
            m.indices.insert(m.indices.end(), {a, b, b + 1, a, b + 1, a + 1});
        }
    // Flat ends across the wire.
    for (int end = 0; end < 2; ++end) {
        Vec3 c, tan, out, bin;
        at(end == 0 ? 0.0f : turns, c, tan, out, bin);
        const Vec3 n = end == 0 ? tan * -1.0f : tan;
        const auto centre = static_cast<std::uint32_t>(m.vertices.size());
        m.vertices.push_back({c, n});
        for (int i = 0; i <= around; ++i) {
            const float ph = 2.0f * kPi * static_cast<float>(i % around) / static_cast<float>(around);
            m.vertices.push_back({c + (out * std::cos(ph) + bin * std::sin(ph)) * r, n});
        }
        for (int i = 0; i < around; ++i) {
            const std::uint32_t a = centre + 1 + static_cast<std::uint32_t>(i);
            if (end == 0) m.indices.insert(m.indices.end(), {centre, a + 1, a});
            else m.indices.insert(m.indices.end(), {centre, a, a + 1});
        }
    }
    // Wound above with the ring's angle against the helix's way: turn every triangle to face out.
    for (std::size_t i = 0; i < m.indices.size(); i += 3) std::swap(m.indices[i + 1], m.indices[i + 2]);
    return m;
}

} // namespace sicdm::primitives


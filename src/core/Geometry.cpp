#include "core/Geometry.hpp"

#include <algorithm>
#include <map>
#include <tuple>

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

void MeshData::append(const MeshData& o) {
    const auto base = static_cast<std::uint32_t>(vertices.size());
    vertices.insert(vertices.end(), o.vertices.begin(), o.vertices.end());
    indices.reserve(indices.size() + o.indices.size());
    for (std::uint32_t i : o.indices) indices.push_back(base + i);
    edgeVertices.insert(edgeVertices.end(), o.edgeVertices.begin(), o.edgeVertices.end());
}

void MeshData::computeFeatureEdges(float angleDeg) {
    // Weld coincident positions so faces that share an edge find each other.
    std::map<std::tuple<long, long, long>, std::uint32_t> weld;
    std::vector<Vec3> welded;
    auto key = [](const Vec3& p) {
        constexpr float q = 1e4f; // 0.1 micron
        return std::make_tuple(std::lround(p.x * q), std::lround(p.y * q), std::lround(p.z * q));
    };
    std::vector<std::uint32_t> remap(vertices.size());
    for (std::size_t i = 0; i < vertices.size(); ++i) {
        const auto k = key(vertices[i].position);
        auto it = weld.find(k);
        if (it == weld.end()) {
            it = weld.emplace(k, static_cast<std::uint32_t>(welded.size())).first;
            welded.push_back(vertices[i].position);
        }
        remap[i] = it->second;
    }
    struct EdgeFaces {
        Vec3 n0{};
        int count = 0;
        bool sharp = false;
    };
    std::map<std::pair<std::uint32_t, std::uint32_t>, EdgeFaces> edges;
    const float cosLimit = std::cos(radians(angleDeg));
    for (std::size_t t = 0; t + 2 < indices.size(); t += 3) {
        const std::uint32_t w[3] = {remap[indices[t]], remap[indices[t + 1]], remap[indices[t + 2]]};
        const Vec3 n = normalize(cross(welded[w[1]] - welded[w[0]], welded[w[2]] - welded[w[0]]));
        if (length(n) == 0.0f) continue;
        for (int e = 0; e < 3; ++e) {
            auto a = w[e], b = w[(e + 1) % 3];
            if (a > b) std::swap(a, b);
            EdgeFaces& ef = edges[{a, b}];
            if (ef.count == 0) ef.n0 = n;
            else if (dot(ef.n0, n) < cosLimit) ef.sharp = true;
            ++ef.count;
        }
    }
    edgeVertices.clear();
    for (const auto& [e, ef] : edges)
        if (ef.count == 1 || ef.sharp || ef.count > 2) {
            edgeVertices.push_back(welded[e.first]);
            edgeVertices.push_back(welded[e.second]);
        }
}

bool isPrimitiveKind(const std::string& k) {
    return k == "box" || k == "wedge" || k == "cylinder" || k == "tube" || k == "cone" || k == "sphere" ||
           k == "torus" || k == "pyramid" || k == "prism";
}

MeshData buildPrimitive(const std::string& k, const std::vector<float>& p) {
    using namespace primitives;
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
    if (k == "tube" && need(3) && positive(3) && p[1] < p[0]) return tube({0, 0, 0}, p[0], p[1], p[2]);
    if (k == "cone" && need(3) && p[0] > 0 && p[1] >= 0 && p[2] > 0) return cone({0, 0, 0}, p[0], p[1], p[2]);
    if (k == "sphere" && need(1) && positive(1)) return sphere({0, 0, p[0]}, p[0]);
    if (k == "torus" && need(2) && positive(2) && p[1] < p[0]) return torus({0, 0, p[1]}, p[0], p[1]);
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

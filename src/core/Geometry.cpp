#include "core/Geometry.hpp"

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

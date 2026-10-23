#include "core/Bvh.hpp"

#include <algorithm>
#include <cmath>

namespace sicdm {

void MeshBvh::build(const MeshData& m) {
    m_nodes.clear();
    m_tris.clear();
    const std::size_t n = m.indices.size() / 3;
    if (n == 0) return;
    m_tris.reserve(n);
    for (std::size_t t = 0; t < n; ++t)
        m_tris.push_back({m.vertices[m.indices[3 * t]].position, m.vertices[m.indices[3 * t + 1]].position, m.vertices[m.indices[3 * t + 2]].position,
                          static_cast<std::uint32_t>(t)});
    std::vector<Vec3> centre(n);
    for (std::size_t t = 0; t < n; ++t) centre[t] = (m_tris[t].a + m_tris[t].b + m_tris[t].c) * (1.0f / 3.0f);
    std::vector<std::uint32_t> order(n);
    for (std::size_t t = 0; t < n; ++t) order[t] = static_cast<std::uint32_t>(t);
    m_nodes.reserve(2 * n / 2 + 1);
    // Iterative: (node, first, count) to split.
    struct Job {
        std::uint32_t node, first, count;
    };
    m_nodes.push_back({});
    std::vector<Job> stack{{0, 0, static_cast<std::uint32_t>(n)}};
    while (!stack.empty()) {
        const Job j = stack.back();
        stack.pop_back();
        AABB box, cbox;
        for (std::uint32_t i = j.first; i < j.first + j.count; ++i) {
            const Tri& tr = m_tris[order[i]];
            box.expand(tr.a), box.expand(tr.b), box.expand(tr.c);
            cbox.expand(centre[order[i]]);
        }
        m_nodes[j.node].box = box;
        const Vec3 ext = cbox.max - cbox.min;
        const int axis = ext.x >= ext.y && ext.x >= ext.z ? 0 : ext.y >= ext.z ? 1 : 2;
        const float spread = axis == 0 ? ext.x : axis == 1 ? ext.y : ext.z;
        if (j.count <= 4 || !(spread > 0)) {
            m_nodes[j.node].first = j.first, m_nodes[j.node].count = j.count;
            continue;
        }
        // Median split along the widest axis: balanced whatever the triangles' sizes.
        const std::uint32_t mid = j.first + j.count / 2;
        auto coord = [&](std::uint32_t t) { return axis == 0 ? centre[t].x : axis == 1 ? centre[t].y : centre[t].z; };
        std::nth_element(order.begin() + j.first, order.begin() + mid, order.begin() + j.first + j.count,
                         [&](std::uint32_t p, std::uint32_t q) { return coord(p) < coord(q); });
        const auto left = static_cast<std::uint32_t>(m_nodes.size());
        m_nodes.push_back({});
        m_nodes.push_back({});
        m_nodes[j.node].left = left, m_nodes[j.node].right = left + 1, m_nodes[j.node].count = 0;
        stack.push_back({left, j.first, mid - j.first});
        stack.push_back({left + 1, mid, j.first + j.count - mid});
    }
    // The triangles in leaf order.
    std::vector<Tri> sorted(n);
    for (std::size_t i = 0; i < n; ++i) sorted[i] = m_tris[order[i]];
    m_tris = std::move(sorted);
}

namespace {

// Entry distance of the ray into the box, or false (inv: 1 / direction).
bool slab(const Vec3& o, const Vec3& inv, const AABB& b, float tmax, float& tnear) noexcept {
    float t0 = 0.0f, t1 = tmax;
    const float os[3] = {o.x, o.y, o.z}, is[3] = {inv.x, inv.y, inv.z};
    const float lo[3] = {b.min.x, b.min.y, b.min.z}, hi[3] = {b.max.x, b.max.y, b.max.z};
    for (int a = 0; a < 3; ++a) {
        float tn = (lo[a] - os[a]) * is[a], tf = (hi[a] - os[a]) * is[a];
        if (tn > tf) std::swap(tn, tf);
        t0 = std::max(t0, tn);
        t1 = std::min(t1, tf);
        if (t0 > t1) return false;
    }
    tnear = t0;
    return true;
}

} // namespace

bool MeshBvh::raycast(const Ray& r, float& t, std::uint32_t* tri) const noexcept {
    if (m_nodes.empty()) return false;
    auto inv = [](float d) { return std::fabs(d) > 1e-30f ? 1.0f / d : (d < 0 ? -1e30f : 1e30f); };
    const Vec3 id{inv(r.dir.x), inv(r.dir.y), inv(r.dir.z)};
    bool hit = false;
    float best = t;
    std::uint32_t stack[64];
    int sp = 0;
    stack[sp++] = 0;
    while (sp > 0) {
        const Node& nd = m_nodes[stack[--sp]];
        float tn;
        if (!slab(r.origin, id, nd.box, best, tn)) continue;
        if (nd.count > 0) {
            for (std::uint32_t i = nd.first; i < nd.first + nd.count; ++i) {
                float th = 0.0f;
                if (intersectTriangle(r, m_tris[i].a, m_tris[i].b, m_tris[i].c, th) && th < best) {
                    best = th;
                    hit = true;
                    if (tri) *tri = m_tris[i].index;
                }
            }
            continue;
        }
        // Nearer child last, so it is taken first. (Median splits keep the tree about
        // log2(n) deep, well inside the stack.)
        float tl = 0, tr = 0;
        const bool hl = slab(r.origin, id, m_nodes[nd.left].box, best, tl), hr = slab(r.origin, id, m_nodes[nd.right].box, best, tr);
        if (hl && hr) {
            if (tl < tr) stack[sp++] = nd.right, stack[sp++] = nd.left;
            else stack[sp++] = nd.left, stack[sp++] = nd.right;
        } else if (hl) stack[sp++] = nd.left;
        else if (hr) stack[sp++] = nd.right;
    }
    if (hit) t = best;
    return hit;
}

} // namespace sicdm

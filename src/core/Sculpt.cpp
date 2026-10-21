#include "core/Sculpt.hpp"

#include "core/Scene.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <tuple>
#include <unordered_map>
#include <utility>

namespace sicdm {

// ---------------------------------------------------------------------------
// Brush names

namespace {

struct BrushInfo {
    SculptBrush brush;
    const char* key;
    const char* name;
};

constexpr BrushInfo kBrushes[] = {
    {SculptBrush::Draw, "draw", "Draw (pull / push)"},
    {SculptBrush::Clay, "clay", "Clay (add material)"},
    {SculptBrush::Carve, "carve", "Carve (remove material)"},
    {SculptBrush::Inflate, "inflate", "Inflate"},
    {SculptBrush::Grab, "grab", "Grab"},
    {SculptBrush::Pinch, "pinch", "Pinch"},
    {SculptBrush::Crease, "crease", "Crease"},
    {SculptBrush::Smooth, "smooth", "Smooth"},
    {SculptBrush::Flatten, "flatten", "Flatten"},
    {SculptBrush::Scrape, "scrape", "Scrape (trim)"},
};

std::string lower(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

} // namespace

const char* sculptBrushKey(SculptBrush b) noexcept {
    for (const BrushInfo& i : kBrushes)
        if (i.brush == b) return i.key;
    return "draw";
}

const char* sculptBrushName(SculptBrush b) noexcept {
    for (const BrushInfo& i : kBrushes)
        if (i.brush == b) return i.name;
    return "";
}

std::optional<SculptBrush> sculptBrushFromKey(const std::string& key) noexcept {
    const std::string k = lower(key);
    for (const BrushInfo& i : kBrushes)
        if (k == i.key) return i.brush;
    if (k == "pull" || k == "push") return SculptBrush::Draw;
    if (k == "add") return SculptBrush::Clay;
    if (k == "remove") return SculptBrush::Carve;
    if (k == "trim") return SculptBrush::Scrape;
    return std::nullopt;
}

const std::vector<SculptBrush>& sculptBrushes() {
    static const std::vector<SculptBrush> all = [] {
        std::vector<SculptBrush> v;
        for (const BrushInfo& i : kBrushes) v.push_back(i.brush);
        return v;
    }();
    return all;
}

const char* booleanOpKey(BooleanOp op) noexcept {
    switch (op) {
    case BooleanOp::Union: return "union";
    case BooleanOp::Subtract: return "subtract";
    case BooleanOp::Intersect: return "intersect";
    }
    return "union";
}

std::optional<BooleanOp> booleanOpFromKey(const std::string& key) noexcept {
    const std::string k = lower(key);
    if (k == "union" || k == "add" || k == "join") return BooleanOp::Union;
    if (k == "subtract" || k == "cut" || k == "difference" || k == "minus") return BooleanOp::Subtract;
    if (k == "intersect" || k == "common" || k == "intersection") return BooleanOp::Intersect;
    return std::nullopt;
}

// ---------------------------------------------------------------------------
// Shared helpers

namespace {

struct D3 {
    double x = 0, y = 0, z = 0;
    D3() = default;
    D3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    explicit D3(const Vec3& v) : x(v.x), y(v.y), z(v.z) {}
    D3 operator+(const D3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    D3 operator-(const D3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    D3 operator*(double s) const { return {x * s, y * s, z * s}; }
    D3 operator-() const { return {-x, -y, -z}; }
    D3& operator+=(const D3& o) {
        x += o.x;
        y += o.y;
        z += o.z;
        return *this;
    }
    Vec3 f() const { return {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)}; }
};
double ddot(const D3& a, const D3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
D3 dcross(const D3& a, const D3& b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
double dlen(const D3& a) { return std::sqrt(ddot(a, a)); }
D3 dnorm(const D3& a) {
    const double l = dlen(a);
    return l > 0 ? a * (1.0 / l) : D3{};
}

std::uint64_t edgeKey(std::uint32_t a, std::uint32_t b) {
    if (a > b) std::swap(a, b);
    return (static_cast<std::uint64_t>(a) << 32) | b;
}

// Merges points closer than `tol` (hash grid, neighbouring cells checked).
class Welder {
public:
    explicit Welder(double tol) : m_tol(tol), m_cell(std::max(tol * 2.0, 1e-12)) {}
    std::uint32_t add(const D3& p) {
        const long long ix = cellOf(p.x), iy = cellOf(p.y), iz = cellOf(p.z);
        for (long long dx = -1; dx <= 1; ++dx)
            for (long long dy = -1; dy <= 1; ++dy)
                for (long long dz = -1; dz <= 1; ++dz) {
                    const auto it = m_head.find(hash(ix + dx, iy + dy, iz + dz));
                    if (it == m_head.end()) continue;
                    for (std::uint32_t i = it->second; i != kNone; i = m_next[i]) {
                        const D3 d = pts[i] - p;
                        if (ddot(d, d) <= m_tol * m_tol) return i;
                    }
                }
        const auto id = static_cast<std::uint32_t>(pts.size());
        pts.push_back(p);
        const auto [it, fresh] = m_head.try_emplace(hash(ix, iy, iz), id);
        m_next.push_back(fresh ? kNone : it->second);
        it->second = id;
        return id;
    }
    std::vector<D3> pts;

private:
    static constexpr std::uint32_t kNone = 0xFFFFFFFFu;
    long long cellOf(double v) const { return static_cast<long long>(std::floor(v / m_cell)); }
    static std::uint64_t hash(long long x, long long y, long long z) {
        return (static_cast<std::uint64_t>(x) * 73856093ull) ^ (static_cast<std::uint64_t>(y) * 19349663ull) ^
               (static_cast<std::uint64_t>(z) * 83492791ull);
    }
    double m_tol, m_cell;
    std::unordered_map<std::uint64_t, std::uint32_t> m_head;
    std::vector<std::uint32_t> m_next;
};

// Shared vertices for a triangle list: identical positions become one vertex, then
// open-boundary vertices a hair apart (seams that differ by float rounding, like a
// sphere's 0 / 360 degree meridian) are snapped together. Thin but valid slivers
// elsewhere survive. Returns the vertex of every input corner.
std::vector<std::uint32_t> weldCorners(const std::vector<D3>& corners, double size, std::vector<D3>& out) {
    Welder exact(size * 1e-12);
    std::vector<std::uint32_t> id(corners.size());
    for (std::size_t i = 0; i < corners.size(); ++i) id[i] = exact.add(corners[i]);
    out = std::move(exact.pts);
    std::unordered_map<std::uint64_t, int> count;
    for (std::size_t k = 0; k + 2 < id.size(); k += 3)
        for (std::size_t e = 0; e < 3; ++e) {
            const std::uint32_t a = id[k + e], b = id[k + (e + 1) % 3];
            if (a != b) ++count[edgeKey(a, b)];
        }
    std::vector<std::uint32_t> open;
    for (const auto& [k, c] : count)
        if (c == 1) {
            open.push_back(static_cast<std::uint32_t>(k >> 32));
            open.push_back(static_cast<std::uint32_t>(k & 0xFFFFFFFFu));
        }
    if (open.empty()) return id;
    std::sort(open.begin(), open.end());
    open.erase(std::unique(open.begin(), open.end()), open.end());
    Welder snap(size * 2e-6);
    std::vector<std::uint32_t> rep(out.size());
    for (std::size_t i = 0; i < rep.size(); ++i) rep[i] = static_cast<std::uint32_t>(i);
    std::vector<std::uint32_t> first;
    for (std::uint32_t v : open) {
        const std::uint32_t w = snap.add(out[v]);
        if (w == first.size()) first.push_back(v);
        else rep[v] = first[w];
    }
    for (std::uint32_t& v : id) v = rep[v];
    return id;
}

// Closest point on triangle abc to p (Ericson, Real-Time Collision Detection 5.1.5).
Vec3 closestOnTriangle(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c) {
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

float smoothFalloff(float d, float r) {
    const float t = std::clamp(1.0f - d / r, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

Vec3 mirroredX(const Vec3& p, float mx) { return {2.0f * mx - p.x, p.y, p.z}; }

} // namespace

// ---------------------------------------------------------------------------
// SculptMesh

SculptMesh::SculptMesh(const Body& body) {
    const AABB b = body.bounds();
    if (!b.valid()) return;
    m_size = std::max(length(b.max - b.min), 1e-3f);
    m_mirrorX = b.center().x;
    const auto& v = body.mesh.vertices;
    const auto& idx = body.mesh.indices;
    std::vector<D3> corners;
    corners.reserve(idx.size());
    for (std::size_t k = 0; k + 2 < idx.size(); k += 3)
        for (std::size_t e = 0; e < 3; ++e) corners.push_back(D3(v[idx[k + e]].position + body.offset));
    std::vector<D3> pts;
    const std::vector<std::uint32_t> id = weldCorners(corners, m_size, pts);
    m_t.reserve(idx.size() / 3);
    for (std::size_t k = 0; k + 2 < id.size(); k += 3) {
        const std::array<std::uint32_t, 3> t{id[k], id[k + 1], id[k + 2]};
        if (t[0] == t[1] || t[1] == t[2] || t[2] == t[0]) continue;
        m_t.push_back(t);
    }
    m_p.reserve(pts.size());
    for (const D3& p : pts) m_p.push_back(p.f());
    // Booleans can also leave needles: two corners a hair apart. Refining splits the
    // edges from both ends alike until their new points round to the same floats.
    const float hair = m_size * 1e-6f;
    std::vector<char> hairy(m_p.size(), 0);
    bool needles = false;
    for (const auto& t : m_t)
        for (int e = 0; e < 3; ++e) {
            const Vec3 d = m_p[t[(e + 1) % 3]] - m_p[t[e]];
            if (dot(d, d) < hair * hair) {
                hairy[t[e]] = hairy[t[(e + 1) % 3]] = 1;
                needles = true;
            }
        }
    if (needles) {
        std::vector<std::uint32_t> around;
        for (std::size_t i = 0; i < m_t.size(); ++i)
            if (hairy[m_t[i][0]] || hairy[m_t[i][1]] || hairy[m_t[i][2]]) around.push_back(static_cast<std::uint32_t>(i));
        collapseLocal(std::move(around), hair, {}, 0.0f, false);
    }
    flipCaps();
    updateNormals();
}

// Booleans can leave caps: flat triangles whose third corner lies on their longest
// edge. Refining one splits it again and again until two new points round to the
// same float, which pinches the surface. Each cap is flipped away with its neighbour
// across that edge (the surface hardly moves: the cap has no area).
void SculptMesh::flipCaps() {
    auto isCap = [&](const std::array<std::uint32_t, 3>& t, int* longest) {
        float l2 = 0.0f;
        for (int e = 0; e < 3; ++e) {
            const Vec3 d = m_p[t[(e + 1) % 3]] - m_p[t[e]];
            if (dot(d, d) > l2) {
                l2 = dot(d, d);
                *longest = e;
            }
        }
        if (!(l2 > 0)) return false;
        const Vec3 cr = cross(m_p[t[1]] - m_p[t[0]], m_p[t[2]] - m_p[t[0]]);
        return dot(cr, cr) <= 4e-8f * l2 * l2; // height under 2e-4 of the edge
    };
    for (int pass = 0; pass < 8; ++pass) {
        std::vector<std::size_t> caps;
        int e = 0;
        for (std::size_t i = 0; i < m_t.size(); ++i)
            if (isCap(m_t[i], &e)) caps.push_back(i);
        if (caps.empty()) return;
        // The edges of every triangle that meets a cap's corner: the cap's long edge
        // with the neighbour across it, and any edge already joining the corner.
        std::vector<char> near(m_p.size(), 0);
        for (const std::size_t i : caps)
            for (std::uint32_t v : m_t[i]) near[v] = 1;
        std::unordered_map<std::uint64_t, std::vector<std::uint32_t>> byEdge;
        for (std::size_t i = 0; i < m_t.size(); ++i)
            if (near[m_t[i][0]] || near[m_t[i][1]] || near[m_t[i][2]])
                for (int k = 0; k < 3; ++k) byEdge[edgeKey(m_t[i][k], m_t[i][(k + 1) % 3])].push_back(static_cast<std::uint32_t>(i));
        std::vector<char> changed(m_t.size(), 0);
        std::size_t flips = 0;
        for (const std::size_t i : caps) {
            if (changed[i] || !isCap(m_t[i], &e)) continue;
            const std::uint32_t a = m_t[i][e], b = m_t[i][(e + 1) % 3], c = m_t[i][(e + 2) % 3];
            const auto& across = byEdge[edgeKey(a, b)];
            if (across.size() != 2) continue;
            const std::uint32_t j = across[0] == i ? across[1] : across[0];
            if (changed[j]) continue;
            int k = 0;
            while (k < 3 && !(m_t[j][k] == b && m_t[j][(k + 1) % 3] == a)) ++k;
            if (k == 3) continue; // not consistently oriented
            const std::uint32_t d = m_t[j][(k + 2) % 3];
            if (d == c || byEdge.count(edgeKey(c, d))) continue;
            // The corner must lie inside the edge, and the new triangles face the same way.
            const Vec3 ab = m_p[b] - m_p[a];
            const float u = dot(m_p[c] - m_p[a], ab) / dot(ab, ab);
            if (!(u > 1e-3f && u < 1.0f - 1e-3f)) continue;
            const Vec3 nj = cross(m_p[a] - m_p[b], m_p[d] - m_p[b]);
            const Vec3 n1 = cross(m_p[d] - m_p[a], m_p[c] - m_p[a]), n2 = cross(m_p[b] - m_p[d], m_p[c] - m_p[d]);
            if (!(dot(n1, nj) > 0) || !(dot(n2, nj) > 0)) continue;
            m_t[i] = {a, d, c};
            m_t[j] = {d, b, c};
            changed[i] = changed[j] = 1;
            byEdge[edgeKey(c, d)] = {static_cast<std::uint32_t>(i), j};
            ++flips;
        }
        if (flips == 0) return;
    }
}

void SculptMesh::updateNormals() {
    m_n.assign(m_p.size(), Vec3{});
    for (const auto& t : m_t) {
        const Vec3 n = cross(m_p[t[1]] - m_p[t[0]], m_p[t[2]] - m_p[t[0]]);
        m_n[t[0]] += n;
        m_n[t[1]] += n;
        m_n[t[2]] += n;
    }
    for (Vec3& n : m_n) n = normalize(n);
}

std::optional<Vec3> SculptMesh::raycast(const Ray& ray, Vec3* normal) const {
    float best = 1e30f;
    int hit = -1;
    for (std::size_t i = 0; i < m_t.size(); ++i) {
        const auto& t = m_t[i];
        float d = 0;
        if (intersectTriangle(ray, m_p[t[0]], m_p[t[1]], m_p[t[2]], d) && d < best) {
            best = d;
            hit = static_cast<int>(i);
        }
    }
    if (hit < 0) return std::nullopt;
    if (normal) {
        const auto& t = m_t[static_cast<std::size_t>(hit)];
        Vec3 n = normalize(cross(m_p[t[1]] - m_p[t[0]], m_p[t[2]] - m_p[t[0]]));
        if (dot(n, ray.dir) > 0) n = -n; // the side facing the viewer
        *normal = n;
    }
    return ray.at(best);
}

Vec3 SculptMesh::closestPoint(const Vec3& q, Vec3* normal) const {
    float best = 1e30f;
    Vec3 bestP = q;
    int hit = -1;
    for (std::size_t i = 0; i < m_t.size(); ++i) {
        const auto& t = m_t[i];
        const Vec3 c = closestOnTriangle(q, m_p[t[0]], m_p[t[1]], m_p[t[2]]);
        const Vec3 d = c - q;
        const float d2 = dot(d, d);
        if (d2 < best) {
            best = d2;
            bestP = c;
            hit = static_cast<int>(i);
        }
    }
    if (normal && hit >= 0) {
        const auto& t = m_t[static_cast<std::size_t>(hit)];
        *normal = normalize(cross(m_p[t[1]] - m_p[t[0]], m_p[t[2]] - m_p[t[0]]));
    }
    return bestP;
}

bool SculptMesh::splitEdges(const std::vector<std::uint64_t>& edges, std::size_t maxTriangles) {
    if (edges.empty() || m_t.size() + edges.size() * 2 > maxTriangles) return false;
    std::unordered_map<std::uint64_t, std::uint32_t> mid;
    mid.reserve(edges.size() * 2);
    std::vector<char> touched(m_p.size(), 0);
    for (const std::uint64_t k : edges) {
        const auto a = static_cast<std::uint32_t>(k >> 32), b = static_cast<std::uint32_t>(k & 0xFFFFFFFFu);
        mid.emplace(k, static_cast<std::uint32_t>(m_p.size()));
        // Not exactly half way: crossing edges (the overlapping slivers a boolean can
        // leave) would otherwise get the same midpoint. At most 0.05% of the edge.
        const float t = 0.5f + static_cast<float>(static_cast<int>((k * 2654435761ull >> 20) % 1001) - 500) * 1e-6f;
        m_p.push_back(m_p[a] + (m_p[b] - m_p[a]) * t);
        m_n.push_back(normalize(m_n[a] + m_n[b]));
        touched[a] = touched[b] = 1;
    }
    const std::size_t count = m_t.size();
    constexpr std::uint32_t kNone = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < count; ++i) {
        const auto tri = m_t[i];
        if (!touched[tri[0]] && !touched[tri[1]] && !touched[tri[2]]) continue;
        std::uint32_t m[3];
        int marked = 0;
        for (int e = 0; e < 3; ++e) {
            const auto it = mid.find(edgeKey(tri[e], tri[(e + 1) % 3]));
            m[e] = it == mid.end() ? kNone : it->second;
            if (m[e] != kNone) ++marked;
        }
        if (marked == 0) continue;
        if (marked == 3) {
            m_t[i] = {tri[0], m[0], m[2]};
            m_t.push_back({m[0], tri[1], m[1]});
            m_t.push_back({m[2], m[1], tri[2]});
            m_t.push_back({m[0], m[1], m[2]});
        } else if (marked == 1) {
            const int e = m[0] != kNone ? 0 : m[1] != kNone ? 1 : 2;
            const std::uint32_t a = tri[e], b = tri[(e + 1) % 3], c = tri[(e + 2) % 3], ab = m[e];
            m_t[i] = {a, ab, c};
            m_t.push_back({ab, b, c});
        } else { // two edges: a corner triangle plus a quad split along its shorter diagonal
            const int e = m[0] == kNone ? 0 : m[1] == kNone ? 1 : 2; // the unsplit edge a-b
            const std::uint32_t a = tri[e], b = tri[(e + 1) % 3], c = tri[(e + 2) % 3];
            const std::uint32_t bc = m[(e + 1) % 3], ca = m[(e + 2) % 3];
            m_t[i] = {ca, bc, c};
            if (length(m_p[a] - m_p[bc]) < length(m_p[b] - m_p[ca])) {
                m_t.push_back({a, b, bc});
                m_t.push_back({a, bc, ca});
            } else {
                m_t.push_back({a, b, ca});
                m_t.push_back({b, bc, ca});
            }
        }
    }
    return true;
}

void SculptMesh::refineRegion(const Vec3& centre, float radius, float detail) {
    const float r2 = radius * radius, d2 = detail * detail;
    // What one dab may add: about the brush's disc at the detail size, with room.
    const float cells = radius / std::max(detail, 1e-6f);
    const std::size_t budget = std::min<std::size_t>(m_t.size() + static_cast<std::size_t>(std::min(64.0f * cells * cells, 2e6f)) + 2000, 4000000);
    for (int pass = 0; pass < 16; ++pass) {
        std::vector<std::uint64_t> edges;
        for (const auto& t : m_t) {
            const Vec3 &a = m_p[t[0]], &b = m_p[t[1]], &c = m_p[t[2]];
            const Vec3 lo = vmin(a, vmin(b, c)), hi = vmax(a, vmax(b, c));
            if (centre.x < lo.x - radius || centre.x > hi.x + radius || centre.y < lo.y - radius ||
                centre.y > hi.y + radius || centre.z < lo.z - radius || centre.z > hi.z + radius)
                continue;
            const Vec3 q = closestOnTriangle(centre, a, b, c) - centre;
            if (dot(q, q) > r2) continue;
            // Slivers left by booleans are not split: a midpoint could land on another vertex.
            const Vec3 cr = cross(b - a, c - a);
            if (dot(cr, cr) <= d2 * d2 * 1e-8) continue;
            // Edges that pass under the brush, and the longest edge of each triangle the
            // brush touches (longest-edge bisection: local, and the triangles stay
            // well shaped). Splitting every long edge of a touched triangle would fan
            // refinement far out along a coarse mesh.
            int longest = 0;
            float longestL2 = 0.0f;
            for (int e = 0; e < 3; ++e) {
                const Vec3 d = m_p[t[(e + 1) % 3]] - m_p[t[e]];
                if (dot(d, d) > longestL2) {
                    longestL2 = dot(d, d);
                    longest = e;
                }
            }
            if (longestL2 > d2) edges.push_back(edgeKey(t[longest], t[(longest + 1) % 3]));
            for (int e = 0; e < 3; ++e) {
                const Vec3 &p0 = m_p[t[e]], &p1 = m_p[t[(e + 1) % 3]];
                const Vec3 d = p1 - p0;
                const float l2 = dot(d, d);
                if (l2 <= d2) continue;
                const float u = std::clamp(dot(centre - p0, d) / l2, 0.0f, 1.0f);
                const Vec3 w = p0 + d * u - centre;
                if (dot(w, w) <= r2) edges.push_back(edgeKey(t[e], t[(e + 1) % 3]));
            }
        }
        if (edges.empty()) break;
        std::sort(edges.begin(), edges.end());
        edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
        if (!splitEdges(edges, budget)) break;
    }
}

void SculptMesh::refineAll(float detail, std::size_t maxTriangles) {
    const float d2 = detail * detail;
    for (int pass = 0; pass < 16; ++pass) {
        std::vector<std::uint64_t> edges;
        for (const auto& t : m_t)
            for (int e = 0; e < 3; ++e) {
                const Vec3 d = m_p[t[(e + 1) % 3]] - m_p[t[e]];
                if (dot(d, d) > d2) edges.push_back(edgeKey(t[e], t[(e + 1) % 3]));
            }
        if (edges.empty()) break;
        std::sort(edges.begin(), edges.end());
        edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
        if (!splitEdges(edges, maxTriangles)) break;
    }
    updateNormals();
}

std::size_t SculptMesh::collapseShort(float minEdge, const Vec3& centre, float radius, bool keepShape) {
    // The triangles that can take part: under the brush, every triangle on an edge
    // short enough to go has a corner within radius + minEdge of the centre (so the
    // rings of the edge's ends are complete); else all of them.
    std::vector<std::uint32_t> local;
    if (radius > 0) {
        local = trianglesNear(centre, radius + minEdge);
    } else {
        local.resize(m_t.size());
        for (std::size_t i = 0; i < local.size(); ++i) local[i] = static_cast<std::uint32_t>(i);
    }
    const std::size_t n = collapseLocal(std::move(local), minEdge, centre, radius, keepShape);
    if (n > 0) updateNormals();
    return n;
}

std::vector<std::uint32_t> SculptMesh::trianglesNear(const Vec3& centre, float reach) const {
    std::vector<std::uint32_t> out;
    const float reach2 = reach * reach;
    for (std::size_t i = 0; i < m_t.size(); ++i)
        for (std::uint32_t v : m_t[i]) {
            const Vec3 q = m_p[v] - centre;
            if (dot(q, q) <= reach2) {
                out.push_back(static_cast<std::uint32_t>(i));
                break;
            }
        }
    return out;
}

// `local` holds every triangle around the ends of each edge that may go. A collapse
// moves corners only within that set, so it holds for every pass. Collapsed
// triangles are marked dead and dropped at the end.
std::size_t SculptMesh::collapseLocal(std::vector<std::uint32_t> local, float minEdge, const Vec3& centre, float radius, bool keepShape) {
    const float min2 = minEdge * minEdge, r2 = radius * radius;
    std::size_t total = 0;
    std::vector<char> dead(m_t.size(), 0);
    for (int pass = 0; pass < 12; ++pass) {
        local.erase(std::remove_if(local.begin(), local.end(), [&](std::uint32_t i) { return dead[i] != 0; }), local.end());
        if (local.empty()) break;
        // Edge use counts by sorting the edge keys (an edge at the rim of the local set
        // counts once: its ends are out of reach and so never collapse), and the
        // open-border vertices (left alone).
        std::vector<std::uint64_t> keys;
        keys.reserve(local.size() * 3);
        for (std::uint32_t i : local) {
            const auto& t = m_t[i];
            for (int e = 0; e < 3; ++e) keys.push_back(edgeKey(t[static_cast<std::size_t>(e)], t[static_cast<std::size_t>((e + 1) % 3)]));
        }
        std::sort(keys.begin(), keys.end());
        std::vector<std::pair<std::uint64_t, int>> count;
        for (std::size_t i = 0; i < keys.size();) {
            std::size_t j = i;
            while (j < keys.size() && keys[j] == keys[i]) ++j;
            count.emplace_back(keys[i], static_cast<int>(j - i));
            i = j;
        }
        std::vector<char> border(m_p.size(), 0);
        for (const auto& [k, c] : count)
            if (c != 2) border[k >> 32] = border[k & 0xFFFFFFFFu] = 1;
        std::vector<std::pair<float, std::uint64_t>> edges;
        for (const auto& [k, c] : count) {
            if (c != 2) continue;
            const auto a = static_cast<std::uint32_t>(k >> 32), b = static_cast<std::uint32_t>(k & 0xFFFFFFFFu);
            const Vec3 d = m_p[a] - m_p[b];
            const float l2 = dot(d, d);
            if (l2 >= min2) continue;
            if (radius > 0) {
                const Vec3 q = (m_p[a] + m_p[b]) * 0.5f - centre;
                if (dot(q, q) > r2) continue;
            }
            edges.emplace_back(l2, k);
        }
        if (edges.empty()) break;
        std::sort(edges.begin(), edges.end());
        // Vertex -> triangles (only once there is something to collapse).
        std::vector<std::vector<std::uint32_t>> around(m_p.size());
        for (std::uint32_t i : local)
            for (std::uint32_t v : m_t[i]) around[v].push_back(i);
        std::vector<char> locked(m_p.size(), 0);
        std::size_t collapsed = 0;
        std::vector<std::uint32_t> ringA, ringB;
        auto ring = [&](std::uint32_t v, std::vector<std::uint32_t>& out) {
            out.clear();
            for (std::uint32_t ti : around[v])
                if (!dead[ti])
                    for (std::uint32_t w : m_t[ti])
                        if (w != v) out.push_back(w);
            std::sort(out.begin(), out.end());
            out.erase(std::unique(out.begin(), out.end()), out.end());
        };
        for (const auto& [l2, k] : edges) {
            const auto a = static_cast<std::uint32_t>(k >> 32), b = static_cast<std::uint32_t>(k & 0xFFFFFFFFu);
            if (locked[a] || locked[b] || border[a] || border[b]) continue;
            // Link condition: the edge's two opposite vertices are the only shared neighbours.
            ring(a, ringA);
            ring(b, ringB);
            int shared = 0;
            for (std::uint32_t w : ringA)
                if (w != b && std::binary_search(ringB.begin(), ringB.end(), w)) ++shared;
            if (shared != 2) continue;
            const Vec3 m = (m_p[a] + m_p[b]) * 0.5f;
            // No triangle may flip or collapse, or turn away from the surface around
            // the edge (its area-weighted normal; slivers' own normals mean little).
            Vec3 regionN{};
            for (std::uint32_t v : {a, b})
                for (std::uint32_t ti : around[v])
                    if (!dead[ti]) regionN += cross(m_p[m_t[ti][1]] - m_p[m_t[ti][0]], m_p[m_t[ti][2]] - m_p[m_t[ti][0]]);
            regionN = normalize(regionN);
            bool ok = !keepShape || length(regionN) > 0.5f;
            for (std::uint32_t v : {a, b}) {
                if (!ok) break;
                for (std::uint32_t ti : around[v]) {
                    if (dead[ti]) continue;
                    const auto& t = m_t[ti];
                    const bool hasA = t[0] == a || t[1] == a || t[2] == a, hasB = t[0] == b || t[1] == b || t[2] == b;
                    if (hasA && hasB) continue;
                    Vec3 q[3];
                    for (int c = 0; c < 3; ++c) {
                        const std::uint32_t w = t[static_cast<std::size_t>(c)];
                        q[c] = (w == a || w == b) ? m : m_p[w];
                    }
                    const Vec3 before = cross(m_p[t[1]] - m_p[t[0]], m_p[t[2]] - m_p[t[0]]);
                    const Vec3 after = cross(q[1] - q[0], q[2] - q[0]);
                    const float la = length(after);
                    if (la <= 1e-12f || dot(before, after) <= 0.0f || (keepShape && dot(after, regionN) <= 0.85f * la)) {
                        ok = false;
                        break;
                    }
                }
                if (!ok) break;
            }
            if (!ok) continue;
            for (std::uint32_t ti : around[b]) {
                if (dead[ti]) continue;
                auto& t = m_t[ti];
                if (t[0] == a || t[1] == a || t[2] == a) {
                    dead[ti] = 1; // the two triangles on the edge
                    continue;
                }
                for (auto& v : t)
                    if (v == b) v = a;
                around[a].push_back(ti);
            }
            m_p[a] = m;
            locked[a] = locked[b] = 1;
            for (std::uint32_t w : ringA) locked[w] = 1;
            for (std::uint32_t w : ringB) locked[w] = 1;
            ++collapsed;
        }
        if (collapsed == 0) break;
        total += collapsed;
    }
    if (total > 0) {
        std::size_t n = 0;
        for (std::size_t i = 0; i < m_t.size(); ++i)
            if (!dead[i]) m_t[n++] = m_t[i];
        m_t.resize(n);
        // Drop vertices no triangle uses any more.
        std::vector<std::uint32_t> remap(m_p.size(), 0xFFFFFFFFu);
        std::vector<Vec3> p;
        for (auto& t : m_t)
            for (auto& v : t) {
                if (remap[v] == 0xFFFFFFFFu) {
                    remap[v] = static_cast<std::uint32_t>(p.size());
                    p.push_back(m_p[v]);
                }
                v = remap[v];
            }
        m_p = std::move(p);
        m_n.resize(m_p.size()); // stale: the caller updates the normals
    }
    return total;
}

void SculptMesh::applyDab(const Vec3& centre, const SculptSettings& s) {
    const float R = std::max({s.radius, m_size * 2e-3f, 1e-3f}), R2 = R * R;
    std::vector<std::pair<std::uint32_t, float>> region;
    for (std::size_t i = 0; i < m_p.size(); ++i) {
        const Vec3 d = m_p[i] - centre;
        const float dd = dot(d, d);
        if (dd < R2) region.emplace_back(static_cast<std::uint32_t>(i), smoothFalloff(std::sqrt(dd), R));
    }
    if (region.empty()) return;

    // Brush plane: weighted average position and normal under the brush.
    Vec3 N{}, C{};
    float W = 0;
    for (const auto& [i, f] : region) {
        N += m_n[i] * f;
        C += m_p[i] * f;
        W += f;
    }
    if (W <= 0) return;
    C = C / W;
    if (length(N) < 1e-6f) closestPoint(centre, &N);
    N = normalize(N);

    const float sign = s.invert ? -1.0f : 1.0f;
    const float st = std::isfinite(s.strength) ? std::clamp(s.strength, 0.0f, 1.0f) : 0.0f;
    const float k = st * 0.1f * R; // displacement per dab at the centre

    std::vector<Vec3> avg;
    if (s.brush == SculptBrush::Smooth) { // neighbour averages (Laplacian)
        std::vector<int> slot(m_p.size(), -1);
        for (std::size_t r = 0; r < region.size(); ++r) slot[region[r].first] = static_cast<int>(r);
        avg.assign(region.size(), Vec3{});
        std::vector<float> cnt(region.size(), 0.0f);
        for (const auto& t : m_t)
            for (int e = 0; e < 3; ++e) {
                const int r = slot[t[e]];
                if (r < 0) continue;
                avg[static_cast<std::size_t>(r)] += m_p[t[(e + 1) % 3]] + m_p[t[(e + 2) % 3]];
                cnt[static_cast<std::size_t>(r)] += 2.0f;
            }
        for (std::size_t r = 0; r < region.size(); ++r)
            avg[r] = cnt[r] > 0 ? avg[r] / cnt[r] : m_p[region[r].first];
    }

    const Vec3 clayPlane = C + N * (sign * 0.12f * R * st);
    for (std::size_t r = 0; r < region.size(); ++r) {
        const auto [i, f] = region[r];
        Vec3& p = m_p[i];
        switch (s.brush) {
        case SculptBrush::Draw: p += N * (sign * k * f); break;
        case SculptBrush::Carve: p -= N * (sign * k * 1.3f * f * f); break;
        case SculptBrush::Clay: {
            const float gap = dot(clayPlane - p, N) * sign; // below the plateau: fill up towards it
            if (gap > 0) p += N * (sign * gap * f * 0.5f);
            break;
        }
        case SculptBrush::Inflate: p += m_n[i] * (sign * k * f); break;
        case SculptBrush::Pinch: {
            Vec3 v = C - p;
            v -= N * dot(v, N);
            p += v * std::min(sign * 0.3f * st * f, 0.9f);
            break;
        }
        case SculptBrush::Crease: {
            Vec3 v = C - p;
            v -= N * dot(v, N);
            p += v * std::min(sign * 0.25f * st * f, 0.9f);
            p -= N * (sign * k * 0.8f * f);
            break;
        }
        case SculptBrush::Smooth: p += (avg[r] - p) * std::min(f * st, 1.0f); break;
        case SculptBrush::Flatten: p += N * (dot(C - p, N) * f * st * sign); break;
        case SculptBrush::Scrape: {
            const float h = dot(p - C, N); // above the plane (or below when inverted): cut down to it
            if (h * sign > 0) p -= N * (h * f * st);
            break;
        }
        case SculptBrush::Grab: break;
        }
    }
}

void SculptMesh::dab(const Vec3& centre, const SculptSettings& s) {
    if (s.brush == SculptBrush::Grab || m_t.empty()) return;
    // A brush no smaller than 0.2% of the body: below that, dynamic topology
    // would refine without end.
    const float R = std::max({s.radius, m_size * 2e-3f, 1e-3f});
    const float detail = std::max(s.detail > 0 ? s.detail : R * 0.25f, m_size * 2e-4f);
    // Mirror only when the two dabs do not overlap much (no double strength on the plane).
    const bool mirror = s.symmetryX && std::fabs(centre.x - m_mirrorX) > R * 0.25f;
    const Vec3 mc = mirroredX(centre, m_mirrorX);
    if (s.refine) {
        refineRegion(centre, R, detail);
        if (mirror) refineRegion(mc, R, detail);
        updateNormals();
    }
    applyDab(centre, s);
    if (mirror) applyDab(mc, s);
    if (s.refine) { // and coarsen where the clay was squashed (edges far below the detail size)
        collapseLocal(trianglesNear(centre, R + detail * 0.3f), detail * 0.3f, centre, R, true);
        if (mirror) collapseLocal(trianglesNear(mc, R + detail * 0.3f), detail * 0.3f, mc, R, true);
    }
    updateNormals();
}

void SculptMesh::beginGrab(const Vec3& centre, const SculptSettings& s) {
    m_grab.clear();
    const float R = std::max({s.radius, m_size * 2e-3f, 1e-3f});
    if (s.refine) { // something to hold on to, even on a coarse block
        const float detail = std::max(s.detail > 0 ? s.detail : R * 0.25f, m_size * 2e-4f);
        refineRegion(centre, R, detail);
        if (s.symmetryX && std::fabs(centre.x - m_mirrorX) > R * 0.25f) refineRegion(mirroredX(centre, m_mirrorX), R, detail);
        updateNormals();
    }
    const float st = std::clamp(s.strength * 2.0f, 0.0f, 1.0f); // strength 0.5 follows the mouse
    std::vector<char> taken(m_p.size(), 0);
    auto collect = [&](const Vec3& c, bool mirrored) {
        for (std::size_t i = 0; i < m_p.size(); ++i) {
            if (taken[i]) continue;
            const float d = length(m_p[i] - c);
            if (d >= R) continue;
            taken[i] = 1;
            m_grab.push_back({static_cast<std::uint32_t>(i), smoothFalloff(d, R) * st, m_p[i], mirrored});
        }
    };
    collect(centre, false);
    if (s.symmetryX && std::fabs(centre.x - m_mirrorX) > R * 0.25f) collect(mirroredX(centre, m_mirrorX), true);
}

void SculptMesh::grabTo(const Vec3& delta) {
    if (m_grab.empty()) return;
    for (const GrabVert& g : m_grab) {
        const Vec3 d = g.mirrored ? Vec3{-delta.x, delta.y, delta.z} : delta;
        m_p[g.v] = g.start + d * g.w;
    }
    updateNormals();
}

void SculptMesh::writeTo(Body& body, bool featureEdges, float smoothAngle) const {
    // Auto-smooth: a corner averages the faces around its vertex that turn less than
    // 40 degrees from its own face, so sculpted surfaces shade smoothly while the
    // hard edges of machined faces stay crisp.
    const std::size_t T = m_t.size(), V = m_p.size();
    std::vector<Vec3> fa(T), fn(T);
    for (std::size_t i = 0; i < T; ++i) {
        const auto& t = m_t[i];
        fa[i] = cross(m_p[t[1]] - m_p[t[0]], m_p[t[2]] - m_p[t[0]]);
        fn[i] = normalize(fa[i]);
    }
    std::vector<std::uint32_t> start(V + 1, 0), inc(T * 3);
    for (const auto& t : m_t)
        for (std::uint32_t v : t) ++start[v + 1];
    for (std::size_t v = 0; v < V; ++v) start[v + 1] += start[v];
    {
        std::vector<std::uint32_t> fill(start.begin(), start.end() - 1);
        for (std::size_t i = 0; i < T; ++i)
            for (std::uint32_t v : m_t[i]) inc[fill[v]++] = static_cast<std::uint32_t>(i);
    }
    const float cosLimit = std::cos(radians(smoothAngle));
    MeshData out;
    out.vertices.reserve(V + V / 4);
    out.indices.assign(T * 3, 0);
    std::vector<std::pair<Vec3, std::uint32_t>> clusters;
    for (std::size_t v = 0; v < V; ++v) {
        clusters.clear();
        for (std::uint32_t k = start[v]; k < start[v + 1]; ++k) {
            const std::uint32_t g = inc[k];
            Vec3 n{};
            for (std::uint32_t h = start[v]; h < start[v + 1]; ++h)
                if (dot(fn[inc[h]], fn[g]) >= cosLimit) n += fa[inc[h]];
            n = normalize(n);
            std::uint32_t out_i = 0xFFFFFFFFu;
            for (const auto& [cn, ci] : clusters)
                if (dot(cn, n) > 0.9999f) {
                    out_i = ci;
                    break;
                }
            if (out_i == 0xFFFFFFFFu) {
                out_i = static_cast<std::uint32_t>(out.vertices.size());
                out.vertices.push_back({m_p[v] - body.offset, n});
                clusters.emplace_back(n, out_i);
            }
            const auto& t = m_t[g];
            const int j = t[0] == v ? 0 : t[1] == v ? 1 : 2;
            out.indices[static_cast<std::size_t>(g) * 3 + static_cast<std::size_t>(j)] = out_i;
        }
    }
    if (featureEdges) out.computeFeatureEdges(40.0f);
    body.mesh = std::move(out);
    body.kind = "mesh";
    body.params.clear();
}

void sculptStroke(Body& body, const std::vector<Vec3>& points, const SculptSettings& s, const Vec3& grabDelta) {
    if (points.empty()) return;
    SculptMesh sm(body);
    if (sm.empty()) return;
    if (s.brush == SculptBrush::Grab) {
        const Vec3 delta = (grabDelta.x != 0 || grabDelta.y != 0 || grabDelta.z != 0 || points.size() < 2)
                               ? grabDelta
                               : points.back() - points.front();
        sm.beginGrab(sm.closestPoint(points.front()), s);
        sm.grabTo(delta);
    } else {
        // The path runs between the points snapped to the surface. Snapping the dabs
        // of a path that floats off the surface would bunch them on the nearest bumps.
        std::vector<Vec3> path;
        path.reserve(points.size());
        for (const Vec3& p : points) path.push_back(sm.closestPoint(p));
        // At most ~1000 dabs a stroke, whatever the radius.
        float total = 0.0f;
        for (std::size_t i = 1; i < path.size(); ++i) total += length(path[i] - path[i - 1]);
        const float spacing = std::max({s.radius * 0.25f, total / 1000.0f, 1e-3f});
        sm.dab(sm.closestPoint(path.front()), s);
        float carry = 0.0f;
        for (std::size_t i = 1; i < path.size(); ++i) {
            const Vec3 a = path[i - 1], b = path[i];
            const float len = length(b - a);
            float at = spacing - carry;
            while (at <= len) {
                sm.dab(sm.closestPoint(a + (b - a) * (at / len)), s);
                at += spacing;
            }
            carry = len - (at - spacing);
        }
    }
    sm.writeTo(body, true);
}

void simplifyMesh(MeshData& mesh, float edge) {
    Body b;
    b.mesh = std::move(mesh);
    SculptMesh sm(b);
    sm.collapseShort(edge);
    sm.writeTo(b, false);
    mesh = std::move(b.mesh);
}

void sculptRefine(Body& body, float detail) {
    SculptMesh sm(body);
    if (sm.empty() || !(detail > 0)) return;
    sm.refineAll(detail);
    sm.writeTo(body, true);
}

// ---------------------------------------------------------------------------
// Booleans. Every triangle-triangle intersection gives a segment whose ends are
// canonical points (an edge of one mesh crossing the plane of a triangle of the
// other), computed once and shared by every triangle that meets them, so the two
// sides of a seam match exactly. Each cut triangle is re-triangulated with its
// segments as constraints, and the regions between seams are kept or dropped by
// their generalised winding number in the other mesh.

namespace {

using DTri = std::array<D3, 3>;

// Generalised winding number of a triangle soup (Barill et al. 2018): exact near
// the query point, a dipole per cluster of triangles far from it.
class WindingField {
public:
    explicit WindingField(std::vector<DTri> tris) : m_tris(std::move(tris)) {
        const std::size_t T = m_tris.size();
        if (T == 0) return;
        D3 lo{1e300, 1e300, 1e300}, hi{-1e300, -1e300, -1e300};
        std::vector<D3> cen(T);
        for (std::size_t i = 0; i < T; ++i) {
            cen[i] = (m_tris[i][0] + m_tris[i][1] + m_tris[i][2]) * (1.0 / 3.0);
            lo = {std::min(lo.x, cen[i].x), std::min(lo.y, cen[i].y), std::min(lo.z, cen[i].z)};
            hi = {std::max(hi.x, cen[i].x), std::max(hi.y, cen[i].y), std::max(hi.z, cen[i].z)};
        }
        const int g = std::max(1, static_cast<int>(std::cbrt(static_cast<double>(T) / 24.0)));
        const D3 ext = hi - lo;
        auto cell = [&](const D3& c) {
            auto ax = [&](double v, double l, double e) {
                return e > 0 ? std::min(g - 1, static_cast<int>((v - l) / e * g)) : 0;
            };
            return (ax(c.x, lo.x, ext.x) * g + ax(c.y, lo.y, ext.y)) * g + ax(c.z, lo.z, ext.z);
        };
        std::vector<std::pair<int, std::uint32_t>> order(T);
        for (std::size_t i = 0; i < T; ++i) order[i] = {cell(cen[i]), static_cast<std::uint32_t>(i)};
        std::sort(order.begin(), order.end());
        std::vector<DTri> sorted(T);
        for (std::size_t i = 0; i < T; ++i) sorted[i] = m_tris[order[i].second];
        m_tris = std::move(sorted);
        std::size_t b = 0;
        while (b < T) {
            std::size_t e = b;
            while (e < T && order[e].first == order[b].first && e - b < 256) ++e;
            Cluster c;
            c.begin = static_cast<std::uint32_t>(b);
            c.end = static_cast<std::uint32_t>(e);
            double area = 0;
            for (std::size_t i = b; i < e; ++i) {
                const DTri& t = m_tris[i];
                const D3 an = dcross(t[1] - t[0], t[2] - t[0]) * 0.5;
                const double a = dlen(an);
                c.N += an;
                c.c += (t[0] + t[1] + t[2]) * (a / 3.0);
                area += a;
            }
            if (area > 0) c.c = c.c * (1.0 / area);
            else c.c = m_tris[b][0];
            for (std::size_t i = b; i < e; ++i)
                for (const D3& p : m_tris[i]) c.r = std::max(c.r, dlen(p - c.c));
            m_clusters.push_back(c);
            b = e;
        }
    }

    double operator()(const D3& q) const {
        double w = 0;
        for (const Cluster& c : m_clusters) {
            const D3 d = c.c - q;
            const double dist = dlen(d);
            if (dist > 2.5 * c.r && dist > 0) {
                w += ddot(d, c.N) / (dist * dist * dist);
                continue;
            }
            for (std::uint32_t i = c.begin; i < c.end; ++i) {
                const D3 a = m_tris[i][0] - q, b = m_tris[i][1] - q, cc = m_tris[i][2] - q;
                const double la = dlen(a), lb = dlen(b), lc = dlen(cc);
                const double det = ddot(a, dcross(b, cc));
                const double den = la * lb * lc + ddot(a, b) * lc + ddot(b, cc) * la + ddot(cc, a) * lb;
                w += 2.0 * std::atan2(det, den);
            }
        }
        return w / (4.0 * 3.14159265358979323846);
    }

private:
    struct Cluster {
        D3 c, N;
        double r = 0;
        std::uint32_t begin = 0, end = 0;
    };
    std::vector<DTri> m_tris;
    std::vector<Cluster> m_clusters;
};

struct OutTri {
    std::array<D3, 3> p, n;
};

// Builds the mesh from a closed triangle soup: points that the arrangement shares
// are bit-identical, so welding only merges exact copies.
MeshData assemble(const std::vector<OutTri>& soup, double tol) {
    Welder weld(tol);
    struct Tri {
        std::array<std::uint32_t, 3> v;
        std::array<D3, 3> n;
    };
    std::vector<Tri> tris;
    tris.reserve(soup.size());
    for (const OutTri& t : soup) {
        Tri r;
        for (std::size_t k = 0; k < 3; ++k) {
            r.v[k] = weld.add(t.p[k]);
            r.n[k] = t.n[k];
        }
        if (r.v[0] == r.v[1] || r.v[1] == r.v[2] || r.v[2] == r.v[0]) continue;
        tris.push_back(r);
    }
    std::vector<D3>& P = weld.pts;

    // Safety net for near-degenerate input (tangent contact, edges grazing edges):
    // cancel doubled triangles, snap open-boundary vertices a hair apart, and fill the
    // tiny holes left, so the result is always closed for export.
    const double snapTol = tol * 1e3, holeLimit = tol * 1e6;
    for (int pass = 0; pass < 3; ++pass) {
        {
            std::unordered_map<std::uint64_t, std::vector<std::size_t>> same;
            auto sortedKey = [](std::array<std::uint32_t, 3> v) {
                std::sort(v.begin(), v.end());
                return (static_cast<std::uint64_t>(v[0]) * 0x9E3779B1u) ^ (static_cast<std::uint64_t>(v[1]) << 21) ^
                       (static_cast<std::uint64_t>(v[2]) << 42);
            };
            for (std::size_t i = 0; i < tris.size(); ++i) same[sortedKey(tris[i].v)].push_back(i);
            std::vector<char> drop(tris.size(), 0);
            for (auto& [k, list] : same) {
                if (list.size() < 2) continue;
                for (std::size_t x = 0; x < list.size(); ++x)
                    for (std::size_t y = x + 1; y < list.size(); ++y) {
                        if (drop[list[x]] || drop[list[y]]) continue;
                        auto a = tris[list[x]].v, b = tris[list[y]].v;
                        auto sa = a, sb = b;
                        std::sort(sa.begin(), sa.end());
                        std::sort(sb.begin(), sb.end());
                        if (sa != sb) continue;
                        const bool sameTurn = (a[0] == b[0] && a[1] == b[1]) || (a[0] == b[1] && a[1] == b[2]) ||
                                              (a[0] == b[2] && a[1] == b[0]);
                        drop[list[y]] = 1;
                        if (!sameTurn) drop[list[x]] = 1; // a face and its back: both go
                    }
            }
            std::vector<Tri> kept;
            for (std::size_t i = 0; i < tris.size(); ++i)
                if (!drop[i]) kept.push_back(tris[i]);
            tris = std::move(kept);
        }
        auto openEdges = [&]() {
            std::unordered_map<std::uint64_t, int> count;
            for (const Tri& t : tris)
                for (int e = 0; e < 3; ++e) ++count[edgeKey(t.v[static_cast<std::size_t>(e)], t.v[static_cast<std::size_t>((e + 1) % 3)])];
            std::vector<std::pair<std::uint32_t, std::uint32_t>> open; // directed, as in their triangle
            for (const Tri& t : tris)
                for (int e = 0; e < 3; ++e) {
                    const std::uint32_t a = t.v[static_cast<std::size_t>(e)], b = t.v[static_cast<std::size_t>((e + 1) % 3)];
                    if (count[edgeKey(a, b)] == 1) open.emplace_back(a, b);
                }
            return open;
        };
        auto open = openEdges();
        if (open.empty()) break;
        { // snap open-boundary vertices
            Welder snap(snapTol);
            std::vector<std::uint32_t> first;
            std::unordered_map<std::uint32_t, std::uint32_t> rep;
            for (const auto& [a, b] : open)
                for (std::uint32_t v : {a, b}) {
                    if (rep.count(v)) continue;
                    const std::uint32_t w = snap.add(P[v]);
                    if (w == first.size()) first.push_back(v);
                    rep[v] = first[w];
                }
            std::vector<Tri> kept;
            for (Tri t : tris) {
                for (auto& v : t.v)
                    if (const auto it = rep.find(v); it != rep.end()) v = it->second;
                if (t.v[0] != t.v[1] && t.v[1] != t.v[2] && t.v[2] != t.v[0]) kept.push_back(t);
            }
            tris = std::move(kept);
        }
        open = openEdges();
        if (open.empty()) continue;
        // Fill small boundary loops with a fan from their centre.
        std::unordered_map<std::uint32_t, std::uint32_t> next; // hole side: b -> a
        for (const auto& [a, b] : open) next[b] = a;
        std::unordered_map<std::uint32_t, char> visited;
        for (const auto& [a, b] : open) {
            if (visited.count(b)) continue;
            std::vector<std::uint32_t> loop;
            std::uint32_t v = b;
            bool closed = false;
            while (loop.size() < 4096) {
                if (visited.count(v)) {
                    closed = v == b;
                    break;
                }
                visited[v] = 1;
                loop.push_back(v);
                const auto it = next.find(v);
                if (it == next.end()) break;
                v = it->second;
            }
            if (!closed || loop.size() < 3) continue;
            D3 c{}, lo{1e300, 1e300, 1e300}, hi{-1e300, -1e300, -1e300};
            for (std::uint32_t q : loop) {
                c += P[q];
                lo = {std::min(lo.x, P[q].x), std::min(lo.y, P[q].y), std::min(lo.z, P[q].z)};
                hi = {std::max(hi.x, P[q].x), std::max(hi.y, P[q].y), std::max(hi.z, P[q].z)};
            }
            if (dlen(hi - lo) > holeLimit) continue; // a real opening, not a crack
            c = c * (1.0 / static_cast<double>(loop.size()));
            const auto ci = static_cast<std::uint32_t>(P.size());
            P.push_back(c);
            for (std::size_t k = 0; k < loop.size(); ++k) {
                const std::uint32_t p0 = loop[k], p1 = loop[(k + 1) % loop.size()];
                const D3 n = dnorm(dcross(P[p1] - P[p0], c - P[p0]));
                tris.push_back({{p0, p1, ci}, {n, n, n}});
            }
        }
    }

    // Float positions, kept distinct: two points closer than float precision are moved
    // apart by one ulp rather than merged (merging can pinch the surface).
    std::vector<Vec3> fp(P.size());
    {
        struct FKey {
            float x, y, z;
            bool operator==(const FKey& o) const { return x == o.x && y == o.y && z == o.z; }
        };
        struct FHash {
            std::size_t operator()(const FKey& k) const {
                std::uint32_t a, b, c;
                std::memcpy(&a, &k.x, 4);
                std::memcpy(&b, &k.y, 4);
                std::memcpy(&c, &k.z, 4);
                return (static_cast<std::size_t>(a) * 73856093u) ^ (static_cast<std::size_t>(b) * 19349663u) ^
                       (static_cast<std::size_t>(c) * 83492791u);
            }
        };
        std::unordered_map<FKey, std::uint32_t, FHash> used;
        std::vector<char> referenced(P.size(), 0);
        for (const Tri& t : tris)
            for (std::uint32_t v : t.v) referenced[v] = 1;
        for (std::size_t i = 0; i < P.size(); ++i) {
            if (!referenced[i]) continue;
            Vec3 f = P[i].f();
            while (!used.try_emplace(FKey{f.x, f.y, f.z}, static_cast<std::uint32_t>(i)).second)
                f.x = std::nextafter(f.x, f.x >= static_cast<float>(P[i].x) ? 1e30f : -1e30f);
            fp[i] = f;
        }
    }

    // Shared vertices where position and normal agree.
    MeshData out;
    std::unordered_map<std::uint64_t, std::uint32_t> seen;
    seen.reserve(tris.size() * 2);
    for (const Tri& t : tris) {
        const D3 a = P[t.v[0]], b = P[t.v[1]], c = P[t.v[2]];
        for (int k = 0; k < 3; ++k) {
            const auto ki = static_cast<std::size_t>(k);
            Vec3 n = dnorm(t.n[ki]).f();
            if (length(n) < 0.5f) n = dnorm(dcross(b - a, c - a)).f();
            const auto q = [](float x) { return static_cast<std::uint64_t>(static_cast<std::int64_t>(std::lround(x * 500.0f)) & 0x3FF); };
            const std::uint64_t key = (static_cast<std::uint64_t>(t.v[ki]) << 30) | (q(n.x) << 20) | (q(n.y) << 10) | q(n.z);
            auto [it, fresh] = seen.try_emplace(key, static_cast<std::uint32_t>(out.vertices.size()));
            if (fresh) out.vertices.push_back({fp[t.v[ki]], n});
            out.indices.push_back(it->second);
        }
    }
    return out;
}

} // namespace

std::size_t SculptMesh::openEdges() const {
    std::unordered_map<std::uint64_t, int> count;
    for (const auto& t : m_t)
        for (int c = 0; c < 3; ++c) {
            std::uint32_t a = t[static_cast<std::size_t>(c)], b = t[static_cast<std::size_t>((c + 1) % 3)];
            if (a > b) std::swap(a, b);
            ++count[static_cast<std::uint64_t>(a) << 32 | b];
        }
    std::size_t open = 0;
    for (const auto& [e, n] : count) open += n == 1 ? 1 : 0;
    return open;
}

std::size_t openEdgeCount(const MeshData& m) {
    struct Key {
        float x, y, z;
        bool operator==(const Key& o) const { return x == o.x && y == o.y && z == o.z; }
    };
    struct KeyHash {
        std::size_t operator()(const Key& k) const {
            std::uint32_t a, b, c;
            std::memcpy(&a, &k.x, 4);
            std::memcpy(&b, &k.y, 4);
            std::memcpy(&c, &k.z, 4);
            return (static_cast<std::size_t>(a) * 73856093u) ^ (static_cast<std::size_t>(b) * 19349663u) ^ (static_cast<std::size_t>(c) * 83492791u);
        }
    };
    std::unordered_map<Key, std::uint32_t, KeyHash> id;
    std::vector<std::uint32_t> r(m.vertices.size());
    for (std::size_t i = 0; i < m.vertices.size(); ++i) {
        const Vec3& p = m.vertices[i].position;
        r[i] = id.try_emplace(Key{p.x + 0.0f, p.y + 0.0f, p.z + 0.0f}, static_cast<std::uint32_t>(id.size())).first->second;
    }
    std::unordered_map<std::uint64_t, int> count;
    for (std::size_t k = 0; k + 2 < m.indices.size(); k += 3)
        for (std::size_t e = 0; e < 3; ++e) {
            const std::uint32_t a = r[m.indices[k + e]], b = r[m.indices[k + (e + 1) % 3]];
            if (a != b) ++count[edgeKey(a, b)];
        }
    std::size_t open = 0;
    for (const auto& [k, c] : count) open += c != 2;
    return open;
}

struct WindingNumber::Impl {
    explicit Impl(std::vector<DTri> t) : field(std::move(t)) {}
    WindingField field;
};

WindingNumber::WindingNumber(const MeshData& m) {
    std::vector<DTri> tris;
    tris.reserve(m.indices.size() / 3);
    for (std::size_t k = 0; k + 2 < m.indices.size(); k += 3)
        tris.push_back({D3(m.vertices[m.indices[k]].position), D3(m.vertices[m.indices[k + 1]].position),
                        D3(m.vertices[m.indices[k + 2]].position)});
    m_impl = new Impl(std::move(tris));
}

WindingNumber::~WindingNumber() { delete m_impl; }

double WindingNumber::operator()(const Vec3& p) const { return m_impl->field(D3(p)); }

double meshVolume(const MeshData& m) {
    double v = 0;
    for (std::size_t k = 0; k + 2 < m.indices.size(); k += 3) {
        const D3 a(m.vertices[m.indices[k]].position), b(m.vertices[m.indices[k + 1]].position),
            c(m.vertices[m.indices[k + 2]].position);
        v += ddot(a, dcross(b, c));
    }
    return v / 6.0;
}

namespace {

// An indexed mesh with coincident vertices snapped together and triangle planes.
using I3 = std::array<std::int64_t, 3>;
#if defined(__SIZEOF_INT128__)
__extension__ typedef __int128 Wide; // exact for the grid's determinants
#else
typedef long double Wide;            // near-exact fallback
#endif

// Exact orientation of d against the plane through a, b, c (positive on the side
// the normal (b - a) x (c - a) points to). Coordinates are on a 2^26 grid, so the
// determinant fits in 128-bit integers.
Wide orient3d(const I3& a, const I3& b, const I3& c, const I3& d) {
    const Wide bx = static_cast<Wide>(b[0] - a[0]), by = b[1] - a[1], bz = b[2] - a[2];
    const Wide cx = c[0] - a[0], cy = c[1] - a[1], cz = c[2] - a[2];
    const Wide dx = d[0] - a[0], dy = d[1] - a[1], dz = d[2] - a[2];
    return (by * cz - bz * cy) * dx + (bz * cx - bx * cz) * dy + (bx * cy - by * cx) * dz;
}

int sign128(Wide v) { return v > 0 ? 1 : v < 0 ? -1 : 0; }

// The integer grid shared by both operands: 2^26 steps across the larger extent,
// finer than the float coordinates the meshes come in.
struct Grid {
    D3 origin;
    double unit = 1.0;
    I3 snap(const D3& p) const {
        return {std::llround((p.x - origin.x) / unit), std::llround((p.y - origin.y) / unit),
                std::llround((p.z - origin.z) / unit)};
    }
    D3 at(const I3& q) const {
        return {origin.x + static_cast<double>(q[0]) * unit, origin.y + static_cast<double>(q[1]) * unit,
                origin.z + static_cast<double>(q[2]) * unit};
    }
};

struct IMesh {
    std::vector<D3> p;
    std::vector<I3> q; // grid coordinates (exact predicates)
    std::vector<std::array<std::uint32_t, 3>> t;
    std::vector<std::array<D3, 3>> n; // corner normals
    std::vector<D3> pn;               // plane normal (unnormalised)
    std::vector<double> pw;           // plane offset
    std::vector<std::array<double, 6>> box;
};

IMesh indexedMesh(const MeshData& m, const Grid& grid) {
    IMesh r;
    const AABB mb = m.bounds();
    const double size = std::max(static_cast<double>(length(mb.max - mb.min)), 1e-6);
    std::vector<D3> corners;
    corners.reserve(m.indices.size());
    for (std::uint32_t i : m.indices) corners.push_back(D3(m.vertices[i].position));
    std::vector<std::uint32_t> id = weldCorners(corners, size, r.p);
    { // onto the grid; vertices landing on the same grid point merge
        struct I3Hash {
            std::size_t operator()(const I3& v) const {
                return std::hash<std::int64_t>()(v[0] * 73856093 ^ v[1] * 19349663 ^ v[2] * 83492791);
            }
        };
        std::unordered_map<I3, std::uint32_t, I3Hash> onGrid;
        std::vector<std::uint32_t> rep(r.p.size());
        std::vector<D3> pts;
        for (std::size_t i = 0; i < r.p.size(); ++i) {
            const I3 g = grid.snap(r.p[i]);
            const auto [it, fresh] = onGrid.try_emplace(g, static_cast<std::uint32_t>(pts.size()));
            if (fresh) {
                pts.push_back(grid.at(g));
                r.q.push_back(g);
            }
            rep[i] = it->second;
        }
        r.p = std::move(pts);
        for (std::uint32_t& v : id) v = rep[v];
    }
    for (std::size_t k = 0; k + 2 < m.indices.size(); k += 3) {
        const std::array<std::uint32_t, 3> t{id[k], id[k + 1], id[k + 2]};
        if (t[0] == t[1] || t[1] == t[2] || t[2] == t[0]) continue;
        // Zero-area triangles stay (they close the surface); with no plane they never
        // intersect anything.
        const D3 pn = dcross(r.p[t[1]] - r.p[t[0]], r.p[t[2]] - r.p[t[0]]);
        r.t.push_back(t);
        r.n.push_back({D3(m.vertices[m.indices[k]].normal), D3(m.vertices[m.indices[k + 1]].normal),
                       D3(m.vertices[m.indices[k + 2]].normal)});
        r.pn.push_back(pn);
        r.pw.push_back(ddot(pn, r.p[t[0]]));
        std::array<double, 6> bx{1e300, 1e300, 1e300, -1e300, -1e300, -1e300};
        for (std::uint32_t v : t) {
            const D3& q = r.p[v];
            bx[0] = std::min(bx[0], q.x);
            bx[1] = std::min(bx[1], q.y);
            bx[2] = std::min(bx[2], q.z);
            bx[3] = std::max(bx[3], q.x);
            bx[4] = std::max(bx[4], q.y);
            bx[5] = std::max(bx[5], q.z);
        }
        r.box.push_back(bx);
    }
    return r;
}

struct PointKey {
    std::uint64_t edge;
    std::uint32_t tri;
    std::uint32_t mesh;
    bool operator==(const PointKey& o) const { return edge == o.edge && tri == o.tri && mesh == o.mesh; }
};
struct PointKeyHash {
    std::size_t operator()(const PointKey& k) const {
        return std::hash<std::uint64_t>()(k.edge * 0x9E3779B97F4A7C15ull ^ (static_cast<std::uint64_t>(k.tri) << 1 | k.mesh));
    }
};

class Arrangement {
public:
    Arrangement(const IMesh& a, const IMesh& b, double tol) : m_m{&a, &b}, m_weld(tol) {
        m_base[0] = 0;
        m_base[1] = static_cast<std::uint32_t>(a.p.size());
        P = a.p;
        P.insert(P.end(), b.p.begin(), b.p.end());
        m_ptMesh.assign(P.size(), -1);
        m_ptEdge.assign(P.size(), 0);
        seg[0].resize(a.t.size());
        seg[1].resize(b.t.size());
        coplanar[0].assign(a.t.size(), 0);
        coplanar[1].assign(b.t.size(), 0);
    }

    // Intersects every pair of triangles whose boxes overlap.
    void intersectAll() {
        const IMesh& A = *m_m[0];
        const IMesh& B = *m_m[1];
        std::vector<std::uint32_t> ob(B.t.size());
        double maxW = 0;
        for (std::size_t i = 0; i < ob.size(); ++i) {
            ob[i] = static_cast<std::uint32_t>(i);
            maxW = std::max(maxW, B.box[i][3] - B.box[i][0]);
        }
        std::sort(ob.begin(), ob.end(), [&](std::uint32_t x, std::uint32_t y) { return B.box[x][0] < B.box[y][0]; });
        for (std::uint32_t ta = 0; ta < A.t.size(); ++ta) {
            const auto& pa = A.box[ta];
            auto it = std::lower_bound(ob.begin(), ob.end(), pa[0] - maxW,
                                       [&](std::uint32_t i, double x) { return B.box[i][0] < x; });
            for (; it != ob.end() && B.box[*it][0] <= pa[3]; ++it) {
                const auto& pb = B.box[*it];
                if (pb[3] < pa[0] || pb[4] < pa[1] || pb[1] > pa[4] || pb[5] < pa[2] || pb[2] > pa[5]) continue;
                intersect(ta, *it);
            }
        }
    }

    // Re-triangulates every cut triangle of mesh `side`; returns all its triangles
    // (global point ids) with corner normals, and adds the seam edges to `seam`.
    void triangulate(int side, std::vector<std::array<std::uint32_t, 3>>& outT, std::vector<std::array<D3, 3>>& outN,
                     std::vector<char>& outCut, std::vector<char>& outFlat);

    bool isSeam(std::uint32_t a, std::uint32_t b) const { return m_seam.count(edgeKey(a, b)) != 0; }

    std::vector<D3> P;
    std::vector<std::vector<std::pair<std::uint32_t, std::uint32_t>>> seg[2];
    // Per triangle: it lies in the plane of a triangle of the other mesh that it
    // overlaps (faces in contact), so it may lie on the other's surface.
    std::vector<char> coplanar[2];

private:
    std::uint32_t gid(int mesh, std::uint32_t v) const { return m_base[mesh] + v; }
    // Exact side of grid point v of mesh `of` against triangle `tri` of mesh `mesh`.
    Wide side(int mesh, std::uint32_t tri, int of, std::uint32_t v) const {
        const IMesh& M = *m_m[mesh];
        const auto& t = M.t[tri];
        return orient3d(M.q[t[0]], M.q[t[1]], M.q[t[2]], m_m[of]->q[v]);
    }
    // Where edge (a, b) of mesh `x` crosses the plane of triangle `tri` of the other
    // mesh. On-plane counts as positive, consistently everywhere.
    std::uint32_t crossing(int x, std::uint32_t a, std::uint32_t b, std::uint32_t tri) {
        if (a > b) std::swap(a, b);
        const IMesh& X = *m_m[x];
        const Wide da = side(1 - x, tri, x, a), db = side(1 - x, tri, x, b);
        if (da == 0) return gid(x, a);
        if (db == 0) return gid(x, b);
        const PointKey key{edgeKey(a, b), tri, static_cast<std::uint32_t>(x)};
        const auto [it, fresh] = m_points.try_emplace(key, static_cast<std::uint32_t>(P.size()));
        if (fresh) {
            const double fa = static_cast<double>(da), fb = static_cast<double>(db);
            const double t = fa / (fa - fb);
            const D3 q = X.p[a] + (X.p[b] - X.p[a]) * t;
            // Where edges of both meshes cross at one spot (an edge running exactly
            // through the other's edge), the same point arrives under several keys.
            const std::uint32_t w = m_weld.add(q);
            if (w < m_weldId.size()) {
                it->second = m_weldId[w];
                m_ptMore.emplace_back(it->second, (static_cast<std::uint64_t>(x) << 63) | edgeKey(a, b));
            } else {
                m_weldId.push_back(static_cast<std::uint32_t>(P.size()));
                P.push_back(q);
                m_ptMesh.push_back(static_cast<std::int8_t>(x));
                m_ptEdge.push_back(edgeKey(a, b));
            }
        }
        return it->second;
    }

    // Does edge (e0, e1) of mesh `x` pass through triangle `tri` of the other mesh
    // (its crossing point inside or on it)? Exact signed volumes: the triangle on the
    // other side of a shared edge gets the opposite answer, so neighbours agree.
    bool through(int x, std::uint32_t e0, std::uint32_t e1, std::uint32_t tri) const {
        if (e0 > e1) std::swap(e0, e1);
        const IMesh& X = *m_m[x];
        const IMesh& Y = *m_m[1 - x];
        const auto& t = Y.t[tri];
        bool pos = false, neg = false;
        for (int k = 0; k < 3; ++k) {
            const int sg = sign128(orient3d(X.q[e0], X.q[e1], Y.q[t[static_cast<std::size_t>(k)]],
                                            Y.q[t[static_cast<std::size_t>((k + 1) % 3)]]));
            pos |= sg > 0;
            neg |= sg < 0;
        }
        return !(pos && neg);
    }

    void intersect(std::uint32_t ta, std::uint32_t tb) {
        const IMesh& A = *m_m[0];
        const IMesh& B = *m_m[1];
        bool sa[3], sb[3];
        bool flat = true;
        for (int i = 0; i < 3; ++i) {
            const Wide d = side(1, tb, 0, A.t[ta][static_cast<std::size_t>(i)]);
            sa[i] = d >= 0;
            flat = flat && d == 0;
        }
        if (flat) { // coplanar: no seam, but both faces are classified with care
            coplanar[0][ta] = coplanar[1][tb] = 1;
            return;
        }
        if (sa[0] == sa[1] && sa[1] == sa[2]) return;
        for (int i = 0; i < 3; ++i) sb[i] = side(0, ta, 1, B.t[tb][static_cast<std::size_t>(i)]) >= 0;
        if (sb[0] == sb[1] && sb[1] == sb[2]) return;
        // The segment's ends: edges of either triangle that pass through the other.
        std::uint32_t ends[6];
        int n = 0;
        auto addEnd = [&](std::uint32_t g) {
            for (int i = 0; i < n; ++i)
                if (ends[i] == g) return;
            ends[n++] = g;
        };
        for (int e = 0; e < 3; ++e) {
            const auto i = static_cast<std::size_t>(e), j = static_cast<std::size_t>((e + 1) % 3);
            if (sa[i] != sa[j] && through(0, A.t[ta][i], A.t[ta][j], tb)) addEnd(crossing(0, A.t[ta][i], A.t[ta][j], tb));
            if (sb[i] != sb[j] && through(1, B.t[tb][i], B.t[tb][j], ta)) addEnd(crossing(1, B.t[tb][i], B.t[tb][j], ta));
        }
        if (n < 2) return;
        std::uint32_t lo = ends[0], hi = ends[1];
        if (n > 2) { // degenerate ties: the two far ends along the line
            const D3 dir = dcross(A.pn[ta], B.pn[tb]);
            auto s = [&](std::uint32_t g) { return ddot(dir, P[g]); };
            for (int i = 0; i < n; ++i) {
                if (s(ends[i]) < s(lo)) lo = ends[i];
                if (s(ends[i]) > s(hi)) hi = ends[i];
            }
        }
        if (lo == hi) return;
        seg[0][ta].emplace_back(lo, hi);
        seg[1][tb].emplace_back(lo, hi);
    }

    const IMesh* m_m[2];
    std::uint32_t m_base[2];
    std::unordered_map<PointKey, std::uint32_t, PointKeyHash> m_points;
    Welder m_weld;                     // new points, by position
    std::vector<std::uint32_t> m_weldId; // welded index -> point id
    std::vector<std::int8_t> m_ptMesh;   // mesh whose edge a point lies on (-1: a vertex)
    std::vector<std::uint64_t> m_ptEdge; // that edge
    // Further edges a welded point lies on: (point, mesh << 63 | edge).
    std::vector<std::pair<std::uint32_t, std::uint64_t>> m_ptMore;
    bool liesOn(std::uint32_t pnt, int mesh, std::uint64_t edge) const {
        if (m_ptMesh[pnt] == mesh && m_ptEdge[pnt] == edge) return true;
        const std::uint64_t tag = (static_cast<std::uint64_t>(mesh) << 63) | edge;
        for (const auto& [q, e] : m_ptMore)
            if (q == pnt && e == tag) return true;
        return false;
    }
    struct SeamSet {
        std::unordered_map<std::uint64_t, char> m;
        std::size_t count(std::uint64_t k) const { return m.count(k); }
        void insert(std::uint64_t k) { m[k] = 1; }
    } m_seam;
};

// Exact sign of the 2D orientation determinant (Shewchuk): a floating-point filter,
// then an exact sum of the six products when the filter cannot decide. Consistent
// in / on / out answers keep long, nearly collinear seams from tangling.
double exactSum(const double* v, int n) {
    // Grow-expansion with two-sum; the sign is the sign of the largest component.
    double e[16];
    int m = 0;
    for (int i = 0; i < n; ++i) {
        double q = v[i];
        int k = 0;
        for (int j = 0; j < m; ++j) {
            const double sum = q + e[j];
            const double bv = sum - q;
            const double err = (q - (sum - bv)) + (e[j] - bv);
            q = sum;
            if (err != 0.0) e[k++] = err;
        }
        e[k++] = q;
        m = k;
    }
    for (int j = m - 1; j >= 0; --j)
        if (e[j] != 0.0) return e[j];
    return 0.0;
}

double orient2d(double ax, double ay, double bx, double by, double cx, double cy) {
    const double l = (ax - cx) * (by - cy), r = (ay - cy) * (bx - cx);
    const double det = l - r;
    const double bound = 3.3306690738754716e-16 * (std::fabs(l) + std::fabs(r));
    if (det > bound || -det > bound) return det;
    // ax*by - ax*cy - cx*by - ay*bx + ay*cx + cy*bx, each product as an exact pair.
    const double f[6][2] = {{ax, by}, {-ax, cy}, {-cx, by}, {-ay, bx}, {ay, cx}, {cy, bx}};
    double terms[12];
    for (int i = 0; i < 6; ++i) {
        const double pr = f[i][0] * f[i][1];
        terms[2 * i] = pr;
        terms[2 * i + 1] = std::fma(f[i][0], f[i][1], -pr);
    }
    return exactSum(terms, 12);
}

// A small 2D constrained triangulation of one triangle.
class LocalTri {
public:
    std::vector<std::uint32_t> g; // global ids
    std::vector<double> x, y;
    std::vector<std::array<int, 3>> tri;
    double tol = 0; // orientation tolerance

    double orient(int a, int b, int c) const {
        const auto A = static_cast<std::size_t>(a), B = static_cast<std::size_t>(b), C = static_cast<std::size_t>(c);
        return orient2d(x[A], y[A], x[B], y[B], x[C], y[C]);
    }
    int add(std::uint32_t gid, double px, double py) {
        g.push_back(gid);
        x.push_back(px);
        y.push_back(py);
        return static_cast<int>(g.size()) - 1;
    }
    int findEdge(int a, int b) const { // triangle with directed edge a -> b
        for (std::size_t i = 0; i < tri.size(); ++i)
            for (int e = 0; e < 3; ++e)
                if (tri[i][static_cast<std::size_t>(e)] == a && tri[i][static_cast<std::size_t>((e + 1) % 3)] == b)
                    return static_cast<int>(i);
        return -1;
    }
    // Splits the edge a -> b (and its twin) at p.
    bool splitEdge(int a, int b, int p) {
        bool split = false;
        for (int pass = 0; pass < 2; ++pass) {
            const int u = pass == 0 ? a : b, v = pass == 0 ? b : a;
            const int t = findEdge(u, v);
            if (t < 0) continue;
            auto tr = tri[static_cast<std::size_t>(t)];
            int e = 0;
            while (!(tr[static_cast<std::size_t>(e)] == u && tr[static_cast<std::size_t>((e + 1) % 3)] == v)) ++e;
            const int o = tr[static_cast<std::size_t>((e + 2) % 3)];
            tri[static_cast<std::size_t>(t)] = {u, p, o};
            tri.push_back({p, v, o});
            split = true;
        }
        return split;
    }
    // An existing vertex at (practically) the same place, or -1.
    int coincident(double px, double py) const {
        for (std::size_t i = 0; i < g.size(); ++i) {
            const double dx = x[i] - px, dy = y[i] - py;
            if (dx * dx + dy * dy <= snap2) return static_cast<int>(i);
        }
        return -1;
    }
    double snap2 = 0;
    void insertInterior(int p) {
        int best = -1;
        double bestMin = -1e300;
        for (std::size_t i = 0; i < tri.size(); ++i) {
            const auto& t = tri[i];
            const double m = std::min({orient(t[1], t[2], p), orient(t[2], t[0], p), orient(t[0], t[1], p)});
            if (m > bestMin) {
                bestMin = m;
                best = static_cast<int>(i);
            }
        }
        if (best < 0) return;
        const auto t = tri[static_cast<std::size_t>(best)];
        const double o[3] = {orient(t[1], t[2], p), orient(t[2], t[0], p), orient(t[0], t[1], p)};
        int nearEdge = -1;
        for (int e = 0; e < 3; ++e)
            if (o[e] <= tol && (nearEdge < 0 || o[e] < o[nearEdge])) nearEdge = e;
        if (nearEdge >= 0) { // on an inner edge: split it (the outline is never split here)
            const int a = t[static_cast<std::size_t>((nearEdge + 1) % 3)], b = t[static_cast<std::size_t>((nearEdge + 2) % 3)];
            if (findEdge(b, a) >= 0 && splitEdge(a, b, p)) return;
        }
        tri[static_cast<std::size_t>(best)] = {t[0], t[1], p};
        tri.push_back({t[1], t[2], p});
        tri.push_back({t[2], t[0], p});
    }
    bool hasEdge(int a, int b) const { return findEdge(a, b) >= 0 || findEdge(b, a) >= 0; }

    void earClip(std::vector<int> poly) {
        while (poly.size() > 3) {
            const std::size_t n = poly.size();
            std::size_t pick = n;
            for (std::size_t i = 0; i < n && pick == n; ++i) {
                const int a = poly[(i + n - 1) % n], b = poly[i], c = poly[(i + 1) % n];
                if (orient(a, b, c) <= tol) continue;
                bool empty = true;
                for (std::size_t k = 0; k < n && empty; ++k) {
                    const int q = poly[k];
                    if (q == a || q == b || q == c) continue;
                    // Strictly inside, or on the new diagonal c-a, blocks the ear.
                    if (orient(a, b, q) > tol && orient(b, c, q) > tol && orient(c, a, q) >= -tol) empty = false;
                }
                if (empty) pick = i;
            }
            if (pick == n) { // numerically stuck: the most convex corner
                double best = -1e300;
                for (std::size_t i = 0; i < n; ++i) {
                    const double o = orient(poly[(i + n - 1) % n], poly[i], poly[(i + 1) % n]);
                    if (o > best) {
                        best = o;
                        pick = i;
                    }
                }
            }
            tri.push_back({poly[(pick + n - 1) % n], poly[pick], poly[(pick + 1) % n]});
            poly.erase(poly.begin() + static_cast<std::ptrdiff_t>(pick));
        }
        if (poly.size() == 3) tri.push_back({poly[0], poly[1], poly[2]});
    }

    // Makes a -> b an edge by flipping the edges that cross it (Sloan 1993). With exact
    // predicates every flip is decided consistently, so this always terminates.
    bool insertSegment(int a, int b, int depth, std::vector<std::pair<int, int>>& done) {
        if (a == b) return true;
        if (hasEdge(a, b)) {
            done.emplace_back(a, b);
            fixed.emplace(edgeKey(static_cast<std::uint32_t>(a), static_cast<std::uint32_t>(b)), 1);
            return true;
        }
        if (depth > 64) return false;
        const auto A = static_cast<std::size_t>(a), B = static_cast<std::size_t>(b);
        const double dx = x[B] - x[A], dy = y[B] - y[A], len2 = dx * dx + dy * dy;
        for (std::size_t c = 0; c < g.size(); ++c) { // a vertex exactly on the segment splits it
            const int ci = static_cast<int>(c);
            if (ci == a || ci == b) continue;
            const double t = ((x[c] - x[A]) * dx + (y[c] - y[A]) * dy) / len2;
            if (t <= 0 || t >= 1 || orient(a, b, ci) != 0.0) continue;
            return insertSegment(a, ci, depth + 1, done) && insertSegment(ci, b, depth + 1, done);
        }
        auto crosses = [&](int u, int v) {
            if (u == a || u == b || v == a || v == b) return false;
            const double o1 = orient(a, b, u), o2 = orient(a, b, v);
            const double o3 = orient(u, v, a), o4 = orient(u, v, b);
            return ((o1 > 0 && o2 < 0) || (o1 < 0 && o2 > 0)) && ((o3 > 0 && o4 < 0) || (o3 < 0 && o4 > 0));
        };
        std::vector<std::pair<int, int>> queue;
        for (const auto& t : tri)
            for (int e = 0; e < 3; ++e) {
                const int u = t[static_cast<std::size_t>(e)], v = t[static_cast<std::size_t>((e + 1) % 3)];
                if (u < v && crosses(u, v)) queue.emplace_back(u, v);
            }
        std::size_t guard = 0;
        const std::size_t limit = 64 + queue.size() * queue.size() * 4;
        while (!queue.empty()) {
            if (++guard > limit) return false;
            const auto [u, v] = queue.front();
            queue.erase(queue.begin());
            const int t1 = findEdge(u, v), t2 = findEdge(v, u);
            if (t1 < 0 || t2 < 0) continue; // already flipped away
            if (fixed.count(edgeKey(static_cast<std::uint32_t>(u), static_cast<std::uint32_t>(v))))
                return false; // two constraints cross (overlapping input): keep the first
            auto third = [&](int ti, int p, int q) {
                for (int w : tri[static_cast<std::size_t>(ti)])
                    if (w != p && w != q) return w;
                return -1;
            };
            const int w1 = third(t1, u, v), w2 = third(t2, u, v);
            // Flip only a strictly convex quad.
            const double ou = orient(w2, w1, u), ov = orient(w2, w1, v);
            if (!((ou > 0 && ov < 0) || (ou < 0 && ov > 0))) {
                queue.emplace_back(u, v);
                continue;
            }
            tri[static_cast<std::size_t>(t1)] = {u, w2, w1};
            tri[static_cast<std::size_t>(t2)] = {w2, v, w1};
            if (crosses(w1, w2)) queue.emplace_back(std::min(w1, w2), std::max(w1, w2));
        }
        if (!hasEdge(a, b)) return false;
        done.emplace_back(a, b);
        fixed.emplace(edgeKey(static_cast<std::uint32_t>(a), static_cast<std::uint32_t>(b)), 1);
        return true;
    }
    std::unordered_map<std::uint64_t, char> fixed; // inserted constraint edges
};

void Arrangement::triangulate(int sideIdx, std::vector<std::array<std::uint32_t, 3>>& outT,
                              std::vector<std::array<D3, 3>>& outN, std::vector<char>& outCut, std::vector<char>& outFlat) {
    const IMesh& X = *m_m[sideIdx];
    for (std::size_t ti = 0; ti < X.t.size(); ++ti) {
        const auto& t = X.t[ti];
        const auto& segs = seg[sideIdx][ti];
        const std::size_t first = outT.size();
        struct Flag { // every piece of a coplanar triangle is flagged
            std::vector<char>& out;
            const std::vector<std::array<std::uint32_t, 3>>& tris;
            std::size_t from;
            char value;
            ~Flag() { out.resize(tris.size(), 0); std::fill(out.begin() + static_cast<std::ptrdiff_t>(from), out.end(), value); }
        } flag{outFlat, outT, first, coplanar[sideIdx][ti]};
        if (segs.empty()) {
            outT.push_back({gid(sideIdx, t[0]), gid(sideIdx, t[1]), gid(sideIdx, t[2])});
            outN.push_back(X.n[ti]);
            outCut.push_back(0);
            continue;
        }
        // Project on the plane's dominant axis, keeping the winding counter-clockwise.
        const D3& pn = X.pn[ti];
        const double ax = std::fabs(pn.x), ay = std::fabs(pn.y), az = std::fabs(pn.z);
        const int k = ax >= ay && ax >= az ? 0 : ay >= az ? 1 : 2;
        const double sgn = (k == 0 ? pn.x : k == 1 ? pn.y : pn.z) >= 0 ? 1.0 : -1.0;
        auto proj = [&](const D3& q, double& u, double& v) {
            const double c[3] = {q.x, q.y, q.z};
            u = c[(k + 1) % 3];
            v = c[(k + 2) % 3] * sgn;
        };
        LocalTri L;
        double size2 = 0;
        for (int i = 0; i < 3; ++i) {
            double u, v;
            proj(X.p[t[static_cast<std::size_t>(i)]], u, v);
            L.add(gid(sideIdx, t[static_cast<std::size_t>(i)]), u, v);
            const D3 e = X.p[t[static_cast<std::size_t>((i + 1) % 3)]] - X.p[t[static_cast<std::size_t>(i)]];
            size2 = std::max(size2, ddot(e, e));
        }
        L.tol = 0.0; // exact predicates
        L.snap2 = size2 * 1e-20;
        L.tri.push_back({0, 1, 2});
        std::unordered_map<std::uint32_t, int> local;
        for (int i = 0; i < 3; ++i) local[L.g[static_cast<std::size_t>(i)]] = i;

        // Points on the triangle's edges, in order along each edge, then interior points.
        std::array<std::vector<std::pair<double, std::uint32_t>>, 3> onEdge;
        std::vector<std::uint32_t> interior;
        auto classify = [&](std::uint32_t pnt) {
            if (local.count(pnt)) return;
            for (int e = 0; e < 3; ++e) {
                const std::uint32_t a = t[static_cast<std::size_t>(e)], b = t[static_cast<std::size_t>((e + 1) % 3)];
                if (liesOn(pnt, sideIdx, edgeKey(a, b))) {
                    const D3 ab = X.p[b] - X.p[a];
                    const double s = ddot(P[pnt] - X.p[a], ab) / ddot(ab, ab);
                    for (const auto& q : onEdge[static_cast<std::size_t>(e)])
                        if (q.second == pnt) return;
                    onEdge[static_cast<std::size_t>(e)].emplace_back(s, pnt);
                    return;
                }
            }
            if (std::find(interior.begin(), interior.end(), pnt) == interior.end()) interior.push_back(pnt);
        };
        for (const auto& [a, b] : segs) {
            classify(a);
            classify(b);
        }
        for (int e = 0; e < 3; ++e) {
            auto& list = onEdge[static_cast<std::size_t>(e)];
            std::sort(list.begin(), list.end());
            int prev = e;
            const int end = (e + 1) % 3;
            for (const auto& [s, pnt] : list) {
                // Exactly on the edge in the plane, whatever the rounding of P.
                const double sc = std::clamp(s, 0.0, 1.0);
                const double u = L.x[static_cast<std::size_t>(e)] + (L.x[static_cast<std::size_t>(end)] - L.x[static_cast<std::size_t>(e)]) * sc;
                const double v = L.y[static_cast<std::size_t>(e)] + (L.y[static_cast<std::size_t>(end)] - L.y[static_cast<std::size_t>(e)]) * sc;
                if (const int same = L.coincident(u, v); same >= 0) { // the same point under another id
                    local[pnt] = same;
                    continue;
                }
                const int li = L.add(pnt, u, v);
                local[pnt] = li;
                L.splitEdge(prev, end, li);
                prev = li;
            }
        }
        for (const std::uint32_t pnt : interior) {
            double u, v;
            proj(P[pnt], u, v);
            { // The exact tests put it inside the triangle: keep it there despite rounding.
                const double x0 = L.x[0], y0 = L.y[0], x1 = L.x[1], y1 = L.y[1], x2 = L.x[2], y2 = L.y[2];
                const double det = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0);
                if (det > 0) {
                    double l1 = ((u - x0) * (y2 - y0) - (x2 - x0) * (v - y0)) / det;
                    double l2 = ((x1 - x0) * (v - y0) - (u - x0) * (y1 - y0)) / det;
                    double l0 = 1.0 - l1 - l2;
                    constexpr double kIn = 1e-12;
                    if (l0 < kIn || l1 < kIn || l2 < kIn) {
                        l0 = std::max(l0, kIn);
                        l1 = std::max(l1, kIn);
                        l2 = std::max(l2, kIn);
                        const double sum = l0 + l1 + l2;
                        l0 /= sum;
                        l1 /= sum;
                        l2 /= sum;
                        u = x0 * l0 + x1 * l1 + x2 * l2;
                        v = y0 * l0 + y1 * l1 + y2 * l2;
                    }
                }
            }
            if (const int same = L.coincident(u, v); same >= 0) {
                local[pnt] = same;
                continue;
            }
            const int li = L.add(pnt, u, v);
            local[pnt] = li;
            L.insertInterior(li);
        }
        std::vector<std::pair<int, int>> done;
        bool failed = false;
        for (const auto& [a, b] : segs)
            if (!L.insertSegment(local[a], local[b], 0, done)) failed = true;
        for (const auto& [a, b] : done)
            m_seam.insert(edgeKey(L.g[static_cast<std::size_t>(a)], L.g[static_cast<std::size_t>(b)]));

        // Corner normals by barycentric interpolation over the original triangle.
        const D3 p0 = X.p[t[0]], p1 = X.p[t[1]], p2 = X.p[t[2]];
        const D3 nrm = dcross(p1 - p0, p2 - p0);
        const double inv = 1.0 / ddot(nrm, nrm);
        auto normalAt = [&](const D3& q) {
            const double l0 = ddot(dcross(p1 - q, p2 - q), nrm) * inv;
            const double l1 = ddot(dcross(p2 - q, p0 - q), nrm) * inv;
            const double l2 = 1.0 - l0 - l1;
            return dnorm(X.n[ti][0] * l0 + X.n[ti][1] * l1 + X.n[ti][2] * l2);
        };
        for (const auto& lt : L.tri) {
            // Zero-area triangles along a straight run of seam points stay: they keep
            // the surface closed.
            if (lt[0] == lt[1] || lt[1] == lt[2] || lt[2] == lt[0]) continue;
            const std::array<std::uint32_t, 3> gt{L.g[static_cast<std::size_t>(lt[0])], L.g[static_cast<std::size_t>(lt[1])],
                                                  L.g[static_cast<std::size_t>(lt[2])]};
            outT.push_back(gt);
            outN.push_back({normalAt(P[gt[0]]), normalAt(P[gt[1]]), normalAt(P[gt[2]])});
            outCut.push_back(failed ? 1 : 0); // classified piece by piece where a seam failed
        }
    }
}

std::vector<DTri> trianglesOf(const IMesh& m) {
    std::vector<DTri> r;
    r.reserve(m.t.size());
    for (const auto& t : m.t) r.push_back({m.p[t[0]], m.p[t[1]], m.p[t[2]]});
    return r;
}

// Labels connected regions of triangles that do not cross seam edges. Pieces of a
// triangle whose seams could not all be inserted are regions of their own.
std::vector<std::uint32_t> regions(const std::vector<std::array<std::uint32_t, 3>>& tris, const std::vector<char>& cut,
                                   const Arrangement& arr) {
    std::vector<std::uint32_t> parent(tris.size());
    for (std::size_t i = 0; i < parent.size(); ++i) parent[i] = static_cast<std::uint32_t>(i);
    auto find = [&](std::uint32_t i) {
        while (parent[i] != i) i = parent[i] = parent[parent[i]];
        return i;
    };
    std::unordered_map<std::uint64_t, std::uint32_t> first;
    first.reserve(tris.size() * 2);
    for (std::size_t i = 0; i < tris.size(); ++i)
        for (int e = 0; e < 3; ++e) {
            const std::uint32_t a = tris[i][static_cast<std::size_t>(e)], b = tris[i][static_cast<std::size_t>((e + 1) % 3)];
            if (cut[i]) break;
            if (arr.isSeam(a, b)) continue;
            const auto [it, fresh] = first.try_emplace(edgeKey(a, b), static_cast<std::uint32_t>(i));
            if (!fresh) parent[find(static_cast<std::uint32_t>(i))] = find(it->second);
        }
    for (std::size_t i = 0; i < parent.size(); ++i) parent[i] = find(static_cast<std::uint32_t>(i));
    return parent;
}

} // namespace

bool meshBoolean(MeshData& a, const MeshData& b, BooleanOp op) {
    if (a.indices.empty() || b.indices.empty()) return false;
    const AABB ba = a.bounds(), bb = b.bounds();
    AABB box;
    box.min = vmax(ba.min, bb.min);
    box.max = vmin(ba.max, bb.max);
    if (!box.valid()) { // apart
        if (op != BooleanOp::Union) return false;
        a.append(b);
        a.computeFeatureEdges(35.0f);
        return true;
    }
    const double scale = std::max(1e-3, static_cast<double>(length(box.max - box.min)));

    AABB all = ba;
    all.expand(bb);
    Grid grid;
    grid.origin = D3(all.min);
    const Vec3 ext = all.max - all.min;
    grid.unit = std::max({static_cast<double>(ext.x), static_cast<double>(ext.y), static_cast<double>(ext.z), 1e-6}) /
                static_cast<double>(1 << 26);
    const IMesh A = indexedMesh(a, grid), B = indexedMesh(b, grid);
    Arrangement arr(A, B, scale * 1e-10);
    arr.intersectAll();
    std::vector<std::array<std::uint32_t, 3>> tA, tB;
    std::vector<std::array<D3, 3>> nA, nB;
    std::vector<char> cutA, cutB, flatA, flatB;
    arr.triangulate(0, tA, nA, cutA, flatA);
    arr.triangulate(1, tB, nB, cutB, flatB);
    const WindingField windA(trianglesOf(A)), windB(trianglesOf(B));
    const double delta = scale * 1e-6;

    // One winding test per region (its largest triangle), not per triangle.
    auto keepMask = [&](const std::vector<std::array<std::uint32_t, 3>>& tris, const std::vector<char>& cut,
                        const std::vector<char>& flat, const WindingField& other, double offsetSign, bool keepInside, bool isA) {
        const std::vector<std::uint32_t> reg = regions(tris, cut, arr);
        // Each region is tested at the incentre of its fattest triangle (largest inradius),
        // the point furthest from its edges, and so from the seams.
        std::unordered_map<std::uint32_t, std::pair<double, std::uint32_t>> biggest;
        auto inradius = [&](const std::array<std::uint32_t, 3>& t) {
            const D3 &p0 = arr.P[t[0]], &p1 = arr.P[t[1]], &p2 = arr.P[t[2]];
            const double per = dlen(p1 - p0) + dlen(p2 - p1) + dlen(p0 - p2);
            return per > 0 ? dlen(dcross(p1 - p0, p2 - p0)) / per : 0.0;
        };
        auto incentre = [&](const std::array<std::uint32_t, 3>& t) {
            const D3 &p0 = arr.P[t[0]], &p1 = arr.P[t[1]], &p2 = arr.P[t[2]];
            const double l0 = dlen(p2 - p1), l1 = dlen(p0 - p2), l2 = dlen(p1 - p0), sum = l0 + l1 + l2;
            return sum > 0 ? (p0 * l0 + p1 * l1 + p2 * l2) * (1.0 / sum) : p0;
        };
        // Representatives off the contact faces where the region has any: a point on a
        // face the other mesh shares cannot tell inside from outside.
        std::unordered_map<std::uint32_t, char> hasOff;
        for (std::size_t i = 0; i < tris.size(); ++i)
            if (!flat[i]) hasOff[reg[i]] = 1;
        for (std::size_t i = 0; i < tris.size(); ++i) {
            if (flat[i] && hasOff.count(reg[i])) continue;
            const double r = inradius(tris[i]);
            auto& bst = biggest[reg[i]];
            if (r >= bst.first) bst = {r, static_cast<std::uint32_t>(i)};
        }
        std::unordered_map<std::uint32_t, bool> keep;
        // Slivers left where a seam could not be inserted are too thin to test: they
        // take their label from a neighbour across a non-seam edge.
        const double sliver = scale * scale * 1e-8;
        std::vector<std::uint32_t> unsure;
        for (const auto& [r, bst] : biggest) {
            if (cut[bst.second] && bst.first * bst.first < sliver) {
                unsure.push_back(r);
                continue;
            }
            const auto& t = tris[bst.second];
            const D3 c = incentre(t);
            const D3 n = dnorm(dcross(arr.P[t[1]] - arr.P[t[0]], arr.P[t[2]] - arr.P[t[0]]));
            const bool inside = other(c + n * (std::min(delta, bst.first * 0.01) * offsetSign)) > 0.5;
            keep[r] = inside == keepInside;
        }
        if (!unsure.empty()) {
            std::unordered_map<std::uint64_t, std::vector<std::uint32_t>> byEdge;
            for (std::size_t i = 0; i < tris.size(); ++i)
                for (int e = 0; e < 3; ++e) {
                    const std::uint32_t ea = tris[i][static_cast<std::size_t>(e)], eb = tris[i][static_cast<std::size_t>((e + 1) % 3)];
                    if (!arr.isSeam(ea, eb)) byEdge[edgeKey(ea, eb)].push_back(static_cast<std::uint32_t>(i));
                }
            for (int pass = 0; pass < 64 && !unsure.empty(); ++pass) {
                std::vector<std::uint32_t> still;
                for (const std::uint32_t r : unsure) {
                    const auto& t = tris[biggest[r].second];
                    bool found = false;
                    for (int e = 0; e < 3 && !found; ++e) {
                        const auto it = byEdge.find(edgeKey(t[static_cast<std::size_t>(e)], t[static_cast<std::size_t>((e + 1) % 3)]));
                        if (it == byEdge.end()) continue;
                        for (const std::uint32_t j : it->second)
                            if (const auto k = keep.find(reg[j]); k != keep.end()) {
                                keep[r] = k->second;
                                found = true;
                                break;
                            }
                    }
                    if (!found) still.push_back(r);
                }
                if (still.size() == unsure.size()) break;
                unsure = std::move(still);
            }
            for (const std::uint32_t r : unsure) { // isolated: test after all
                const auto& t = tris[biggest[r].second];
                const D3 c = (arr.P[t[0]] + arr.P[t[1]] + arr.P[t[2]]) * (1.0 / 3.0);
                keep[r] = (other(c) > 0.5) == keepInside;
            }
        }
        std::vector<char> mask(tris.size());
        for (std::size_t i = 0; i < tris.size(); ++i) mask[i] = keep[reg[i]];
        // Pieces lying on the other's surface (faces in contact): one copy of a face
        // both solids share on the same side stays in a union or an intersection; faces
        // where the solids meet back to back go, except A's in a subtraction.
        for (std::size_t i = 0; i < tris.size(); ++i) {
            if (!flat[i]) continue;
            const auto& t = tris[i];
            const double r = inradius(t);
            if (!(r > 0)) continue;
            const D3 c = incentre(t);
            const double w = other(c);
            if (std::fabs(w - 0.5) > 0.25) continue; // beside the other's face: as its region
            const D3 n = dnorm(dcross(arr.P[t[1]] - arr.P[t[0]], arr.P[t[2]] - arr.P[t[0]]));
            const bool same = other(c + n * std::min(delta, r * 0.01)) < 0.5; // the other solid is behind it too
            mask[i] = static_cast<char>(isA && (same ? op != BooleanOp::Subtract : op == BooleanOp::Subtract));
        }
        return mask;
    };
    const std::vector<char> keepA = keepMask(tA, cutA, flatA, windB, -1.0, op == BooleanOp::Intersect, true);
    const std::vector<char> keepB = keepMask(tB, cutB, flatB, windA, 1.0, op != BooleanOp::Union, false);

    std::vector<OutTri> soup;
    for (std::size_t i = 0; i < tA.size(); ++i)
        if (keepA[i]) soup.push_back({{arr.P[tA[i][0]], arr.P[tA[i][1]], arr.P[tA[i][2]]}, nA[i]});
    const bool flip = op == BooleanOp::Subtract;
    for (std::size_t i = 0; i < tB.size(); ++i) {
        if (!keepB[i]) continue;
        if (flip)
            soup.push_back({{arr.P[tB[i][0]], arr.P[tB[i][2]], arr.P[tB[i][1]]}, {-nB[i][0], -nB[i][2], -nB[i][1]}});
        else
            soup.push_back({{arr.P[tB[i][0]], arr.P[tB[i][1]], arr.P[tB[i][2]]}, nB[i]});
    }
    if (soup.empty()) return false;
    MeshData out = assemble(soup, scale * 1e-9);
    if (out.indices.empty()) return false;
    out.computeFeatureEdges(35.0f);
    a = std::move(out);
    return true;
}

// ---------------------------------------------------------------------------
// Cutters

namespace {

void frameOf(const Vec3& w, Vec3& u, Vec3& v) {
    const Vec3 ref = std::fabs(w.z) < 0.9f ? Vec3{0, 0, 1} : Vec3{0, 1, 0};
    u = normalize(cross(ref, w));
    v = cross(w, u);
}

// Rings of points closed by poles (radius 0): ring r is a circle of radii[r] around
// centres[r] in the plane of us[r], vs[r]. Ring vertices get radial normals, poles
// poleNormals[r].
MeshData ringMesh(const std::vector<Vec3>& centres, const std::vector<float>& radii, const std::vector<Vec3>& us,
                  const std::vector<Vec3>& vs, const std::vector<Vec3>& poleNormals, int segments) {
    MeshData m;
    const std::size_t R = centres.size();
    // Per ring, per segment: index of the vertex (poles are single vertices).
    std::vector<std::vector<std::uint32_t>> idx(R);
    for (std::size_t r = 0; r < R; ++r) {
        if (radii[r] <= 0.0f) {
            idx[r].assign(static_cast<std::size_t>(segments), static_cast<std::uint32_t>(m.vertices.size()));
            m.vertices.push_back({centres[r], poleNormals[r]});
            continue;
        }
        for (int s = 0; s < segments; ++s) {
            // Half a segment round: no ring vertex lies in the plane of the path (a flat
            // surface the channel is cut along), which would make degenerate splits.
            const float a = 6.28318530718f * (static_cast<float>(s) + 0.5f) / static_cast<float>(segments);
            const Vec3 radial = us[r] * std::cos(a) + vs[r] * std::sin(a);
            idx[r].push_back(static_cast<std::uint32_t>(m.vertices.size()));
            m.vertices.push_back({centres[r] + radial * radii[r], radial});
        }
    }
    for (std::size_t r = 0; r + 1 < R; ++r)
        for (int s = 0; s < segments; ++s) {
            const auto s0 = static_cast<std::size_t>(s), s1 = static_cast<std::size_t>((s + 1) % segments);
            const std::uint32_t a0 = idx[r][s0], a1 = idx[r][s1], b0 = idx[r + 1][s0], b1 = idx[r + 1][s1];
            if (a0 != a1) m.indices.insert(m.indices.end(), {a0, a1, b1});
            if (b0 != b1) m.indices.insert(m.indices.end(), {a0, b1, b0});
        }
    return m;
}

// Solid of revolution: profile (radius, height) from the bottom pole (r = 0) to the
// top pole (r = 0) around the axis from `base` along `w`. Faceted along the profile,
// smooth around it.
MeshData revolve(const Vec3& base, const Vec3& wIn, const std::vector<std::pair<float, float>>& profile, int segments) {
    const Vec3 w = normalize(wIn);
    Vec3 u, v;
    frameOf(w, u, v);
    MeshData m;
    for (std::size_t i = 0; i + 1 < profile.size(); ++i) {
        const auto [r0, z0] = profile[i];
        const auto [r1, z1] = profile[i + 1];
        const float dr = r1 - r0, dz = z1 - z0;
        if (std::fabs(dr) < 1e-9f && std::fabs(dz) < 1e-9f) continue;
        for (int s = 0; s < segments; ++s) {
            // An odd fraction of a segment round, so a cutter never shares its angles
            // with a body built on the same grid (exact edge-on-edge crossings).
            // (Both from the integer step, so neighbouring quads share their corners exactly.)
            const float a0 = 6.28318530718f * (static_cast<float>(s) + 0.3719f) / static_cast<float>(segments);
            const float a1 = 6.28318530718f * (static_cast<float>(s + 1) + 0.3719f) / static_cast<float>(segments);
            const Vec3 e0 = u * std::cos(a0) + v * std::sin(a0), e1 = u * std::cos(a1) + v * std::sin(a1);
            const Vec3 n0 = normalize(e0 * dz - w * dr), n1 = normalize(e1 * dz - w * dr);
            const Vec3 A0 = base + e0 * r0 + w * z0, A1 = base + e1 * r0 + w * z0;
            const Vec3 B0 = base + e0 * r1 + w * z1, B1 = base + e1 * r1 + w * z1;
            const auto at = static_cast<std::uint32_t>(m.vertices.size());
            m.vertices.insert(m.vertices.end(), {{A0, n0}, {A1, n1}, {B1, n1}, {B0, n0}});
            if (r0 > 0) m.indices.insert(m.indices.end(), {at, at + 1, at + 2});
            if (r1 > 0) m.indices.insert(m.indices.end(), {at, at + 2, at + 3});
        }
    }
    return m;
}

} // namespace

MeshData cutterCylinder(const Vec3& from, const Vec3& to, float radius, int segments) {
    const float h = length(to - from);
    return revolve(from, to - from, {{0, 0}, {radius, 0}, {radius, h}, {0, h}}, segments);
}

MeshData cutterCone(const Vec3& from, const Vec3& to, float r0, float r1, int segments) {
    const float h = length(to - from);
    std::vector<std::pair<float, float>> prof{{0, 0}};
    if (r0 > 0) prof.push_back({r0, 0});
    if (r1 > 0) prof.push_back({r1, h});
    prof.push_back({0, h});
    return revolve(from, to - from, prof, segments);
}

MeshData cutterSphere(const Vec3& centre, float radius, int segments, int rings) {
    std::vector<std::pair<float, float>> prof;
    for (int i = 0; i <= rings; ++i) {
        const float a = -kPi * 0.5f + kPi * static_cast<float>(i) / static_cast<float>(rings);
        prof.push_back({i == 0 || i == rings ? 0.0f : radius * std::cos(a), radius * std::sin(a)});
    }
    return revolve(centre, {0, 0, 1}, prof, segments);
}

MeshData cutterBox(const Vec3& c, const Vec3& u, const Vec3& v, const Vec3& w, const Vec3& h) {
    MeshData m;
    auto face = [&](const Vec3& n, const Vec3& a, const Vec3& b, float hn, float ha, float hb) {
        // Corners counter-clockwise seen from outside (a x b = n).
        const Vec3 o = c + n * hn;
        const auto at = static_cast<std::uint32_t>(m.vertices.size());
        m.vertices.push_back({o - a * ha - b * hb, n});
        m.vertices.push_back({o + a * ha - b * hb, n});
        m.vertices.push_back({o + a * ha + b * hb, n});
        m.vertices.push_back({o - a * ha + b * hb, n});
        m.indices.insert(m.indices.end(), {at, at + 1, at + 2, at, at + 2, at + 3});
    };
    face(w, u, v, h.z, h.x, h.y);
    face(-w, v, u, h.z, h.y, h.x);
    face(u, v, w, h.x, h.y, h.z);
    face(-u, w, v, h.x, h.z, h.y);
    face(v, w, u, h.y, h.z, h.x);
    face(-v, u, w, h.y, h.x, h.z);
    return m;
}

MeshData cutterTube(const std::vector<Vec3>& path, float radius, int segments) {
    if (path.size() < 2) return path.empty() ? MeshData{} : cutterSphere(path.front(), radius, segments, segments / 2);
    std::vector<Vec3> cs, us, vs, ns;
    std::vector<float> rs;
    const std::size_t n = path.size();
    std::vector<Vec3> tang(n);
    for (std::size_t i = 0; i < n; ++i)
        tang[i] = normalize(path[std::min(i + 1, n - 1)] - path[i == 0 ? 0 : i - 1]);
    Vec3 u, v;
    frameOf(tang[0], u, v);
    const int capRings = std::max(3, segments / 4);
    auto addRing = [&](const Vec3& c, float r, const Vec3& t, const Vec3& nw) {
        // Parallel transport of the frame keeps the rings from twisting.
        u = normalize(u - t * dot(u, t));
        v = cross(t, u);
        cs.push_back(c);
        rs.push_back(r);
        us.push_back(u);
        vs.push_back(v);
        ns.push_back(nw);
    };
    for (int k = capRings; k >= 1; --k) { // start cap, from the pole
        const float a = kPi * 0.5f * static_cast<float>(k) / static_cast<float>(capRings);
        addRing(path[0] - tang[0] * (radius * std::sin(a)), k == capRings ? 0.0f : radius * std::cos(a), tang[0], -tang[0]);
    }
    for (std::size_t i = 0; i < n; ++i) addRing(path[i], radius, tang[i], {});
    for (int k = 1; k <= capRings; ++k) {
        const float a = kPi * 0.5f * static_cast<float>(k) / static_cast<float>(capRings);
        addRing(path[n - 1] + tang[n - 1] * (radius * std::sin(a)), k == capRings ? 0.0f : radius * std::cos(a),
                tang[n - 1], tang[n - 1]);
    }
    MeshData m = ringMesh(cs, rs, us, vs, ns, segments);
    // Cap normals point away from the end centres.
    for (Vertex& vx : m.vertices) {
        const Vec3 d0 = vx.position - path.front(), d1 = vx.position - path.back();
        if (dot(d0, tang.front()) < 0) vx.normal = normalize(d0);
        else if (dot(d1, tang.back()) > 0) vx.normal = normalize(d1);
    }
    return m;
}

// ---------------------------------------------------------------------------
// Body operations

MeshData worldMesh(const Body& body) {
    MeshData m = body.mesh;
    if (body.offset.x != 0 || body.offset.y != 0 || body.offset.z != 0) m.transform(translate(body.offset));
    return m;
}

namespace {


// meshBoolean, retried with the tool nudged by a hundred-thousandth of its size when
// a coincidence (faces or edges exactly on each other) leaves the result unclosed.
bool robustBoolean(MeshData& a, const MeshData& tool, BooleanOp op) {
    // Closed once welded (round primitives have seams of nearly equal points); the
    // cutters close up within the boolean.
    Body input;
    input.mesh = a;
    const bool closedInput = SculptMesh(input).openEdges() == 0;
    MeshData best;
    std::size_t bestOpen = static_cast<std::size_t>(-1);
    const AABB tb = tool.bounds();
    const float nudge = std::max(length(tb.max - tb.min), 1e-3f) * 1e-5f;
    const Vec3 dirs[] = {{0, 0, 0}, {0.577f, -0.211f, 0.789f}, {-0.408f, 0.816f, 0.408f}, {0.707f, 0.5f, -0.5f}};
    for (const Vec3& d : dirs) {
        MeshData r = a;
        MeshData t = tool;
        if (d.x != 0 || d.y != 0 || d.z != 0) t.transform(translate(d * nudge));
        if (!meshBoolean(r, t, op)) {
            if (bestOpen == static_cast<std::size_t>(-1) && d.x == 0 && d.y == 0 && d.z == 0) return false;
            continue;
        }
        const std::size_t open = closedInput ? openEdgeCount(r) : 0;
        if (open < bestOpen) {
            bestOpen = open;
            best = std::move(r);
        }
        if (bestOpen == 0) break;
    }
    if (bestOpen == static_cast<std::size_t>(-1)) return false;
    a = std::move(best);
    return true;
}

// Cuts and booleans run in the body's own frame (its mesh as stored, the tool moved
// into it): far from the origin, world floats would lose the detail.
void setLocalMesh(Body& body, MeshData m) {
    body.mesh = std::move(m);
    body.kind = "mesh";
    body.params.clear();
}

bool applyCut(Body& body, const MeshData& worldTool, BooleanOp op, const char* what, std::string* error) {
    MeshData a = body.mesh;
    MeshData tool = worldTool;
    if (body.offset.x != 0 || body.offset.y != 0 || body.offset.z != 0) tool.transform(translate(-body.offset));
    const AABB ab = a.bounds(), tb = tool.bounds();
    if (ab.max.x < tb.min.x || ab.min.x > tb.max.x || ab.max.y < tb.min.y || ab.min.y > tb.max.y ||
        ab.max.z < tb.min.z || ab.min.z > tb.max.z) {
        if (error) *error = std::string("The ") + what + " does not touch " + body.name;
        return false;
    }
    const double before = meshVolume(a);
    if (!robustBoolean(a, tool, op)) {
        if (error)
            *error = op == BooleanOp::Union ? std::string("The ") + what + " could not be added"
                                            : std::string("The ") + what + " would remove the whole body";
        return false;
    }
    const double after = meshVolume(a);
    if (std::fabs(after - before) <= 1e-6 * std::max(std::fabs(before), 1.0)) {
        if (error) *error = std::string("The ") + what + " does not touch " + body.name;
        return false;
    }
    // A cut that adds material, or an addition that removes it, is no solid result
    // (a tool running through itself, a broken input): keep the body as it was.
    const double slack = 1e-6 * std::max(std::fabs(before), 1.0);
    if ((op == BooleanOp::Subtract && after > before + slack) || (op == BooleanOp::Union && after < before - slack) || !(after > 0)) {
        if (error) *error = std::string("The ") + what + " did not give a clean solid; " + body.name + " is unchanged";
        return false;
    }
    setLocalMesh(body, std::move(a));
    return true;
}

float bodySize(const Body& b) {
    const AABB bb = b.bounds();
    return bb.valid() ? length(bb.max - bb.min) : 100.0f;
}

} // namespace

bool drillHole(Body& body, const Vec3& at, const Vec3& normal, float diameter, float depth, std::string* error) {
    if (!(diameter > 0)) {
        if (error) *error = "The hole needs a diameter";
        return false;
    }
    const Vec3 n = normalize(normal);
    const float r = diameter * 0.5f;
    const float margin = std::max(0.5f, diameter * 0.25f);
    std::vector<std::pair<float, float>> prof;
    if (depth > 0) { // blind hole with a 118 degree drill point
        const float tip = r / std::tan(radians(59.0f));
        prof = {{0, -depth - tip}, {r, -depth}, {r, margin}, {0, margin}};
    } else {
        const float through = bodySize(body) * 2.0f + length(at - body.bounds().center());
        prof = {{0, -through}, {r, -through}, {r, margin}, {0, margin}};
    }
    return applyCut(body, revolve(at, n, prof, 48), BooleanOp::Subtract, "hole", error);
}

bool cutPocket(Body& body, const Vec3& at, const Vec3& normal, PocketShape shape, float width, float length_,
               float depth, float angleDeg, std::string* error) {
    if (!(width > 0) || (shape == PocketShape::Box && !(length_ > 0)) || (shape != PocketShape::Ball && !(depth > 0))) {
        if (error) *error = "The pocket needs a width, length and depth";
        return false;
    }
    const Vec3 n = normalize(normal);
    Vec3 u, v;
    frameOf(n, u, v);
    if (angleDeg != 0.0f) {
        const float a = radians(angleDeg);
        const Vec3 u2 = u * std::cos(a) + v * std::sin(a);
        v = cross(n, u2);
        u = u2;
    }
    const float margin = std::max(0.5f, width * 0.25f);
    MeshData tool;
    switch (shape) {
    case PocketShape::Box: {
        const float h = (depth + margin) * 0.5f;
        tool = cutterBox(at + n * (margin - h), u, v, n, {width * 0.5f, length_ * 0.5f, h});
        break;
    }
    case PocketShape::Round: tool = cutterCylinder(at - n * depth, at + n * margin, width * 0.5f, 48); break;
    case PocketShape::Ball: {
        const float r = width * 0.5f;
        tool = cutterSphere(at - n * ((depth > 0 ? depth : r) - r), r, 40, 20);
        break;
    }
    }
    return applyCut(body, tool, BooleanOp::Subtract, "pocket", error);
}

bool cutChannel(Body& body, const std::vector<Vec3>& path, const std::vector<Vec3>& normals, float width, float depth,
                std::string* error) {
    if (path.size() < 2 || normals.size() != path.size() || !(width > 0)) {
        if (error) *error = "The channel needs a path of two or more points and a width";
        return false;
    }
    const float r = width * 0.5f;
    const float d = depth > 0 ? depth : r;
    // Centre line below the surface, resampled and smoothed so the tube does not fold
    // on itself.
    std::vector<Vec3> line;
    for (std::size_t i = 0; i < path.size(); ++i) line.push_back(path[i] - normalize(normals[i]) * (d - r));
    // Rings a radius apart turn at most ~45 degrees each without the tube folding into itself.
    std::vector<Vec3> even{line.front()};
    const float step = std::max(r, 1e-3f);
    float carry = 0;
    for (std::size_t i = 1; i < line.size(); ++i) {
        const Vec3 a = line[i - 1], b = line[i];
        const float len = length(b - a);
        float at = step - carry;
        while (at <= len) {
            even.push_back(a + (b - a) * (at / len));
            at += step;
        }
        carry = len - (at - step);
    }
    if (length(line.back() - even.back()) > step * 0.25f) even.push_back(line.back());
    for (int it = 0; it < 6 && even.size() > 2; ++it) {
        std::vector<Vec3> s = even;
        for (std::size_t i = 1; i + 1 < even.size(); ++i) s[i] = (even[i - 1] + even[i] * 2.0f + even[i + 1]) * 0.25f;
        even = std::move(s);
    }
    if (even.size() < 2) even.push_back(line.back());
    // A path that comes back within its own width (a hairpin, a crossing) would make
    // the tube run through itself: the cut would not be a solid.
    for (std::size_t i = 0; i < even.size(); ++i)
        for (std::size_t j = i + 5; j < even.size(); ++j) {
            const Vec3 q = even[j] - even[i];
            if (dot(q, q) < width * width * 0.98f) {
                if (error) *error = "The channel path comes back within its own width: draw it as separate channels";
                return false;
            }
        }
    return applyCut(body, cutterTube(even, r, 24), BooleanOp::Subtract, "channel", error);
}

double volumeBeyondPlane(const MeshData& m, const Vec3& point, const Vec3& normal) {
    const D3 o(point), n = dnorm(D3(normal));
    double vol = 0;
    for (std::size_t k = 0; k + 2 < m.indices.size(); k += 3) {
        D3 poly[6];
        int cnt = 0;
        const D3 tri[3] = {D3(m.vertices[m.indices[k]].position), D3(m.vertices[m.indices[k + 1]].position),
                           D3(m.vertices[m.indices[k + 2]].position)};
        for (int i = 0; i < 3; ++i) { // clip to the side the normal points to
            const D3& a = tri[i];
            const D3& b = tri[(i + 1) % 3];
            const double da = ddot(a - o, n), db = ddot(b - o, n);
            if (da >= 0) poly[cnt++] = a;
            if ((da >= 0) != (db >= 0)) poly[cnt++] = a + (b - a) * (da / (da - db));
        }
        // Tetrahedra from a point on the plane: the cap they would need has no volume.
        for (int i = 1; i + 1 < cnt; ++i) vol += ddot(poly[0] - o, dcross(poly[i] - o, poly[i + 1] - o));
    }
    return vol / 6.0;
}

bool trimByPlane(Body& body, const Vec3& point, const Vec3& normal, std::string* error) {
    const Vec3 n = normalize(normal);
    if (length(n) < 0.5f) {
        if (error) *error = "The knife plane needs a normal";
        return false;
    }
    Vec3 u, v;
    frameOf(n, u, v);
    // Just big enough to swallow everything on that side (a tight box keeps the
    // shared grid of the boolean fine).
    const float L = length(point - body.bounds().center()) + bodySize(body) * 0.6f + 1.0f;
    return applyCut(body, cutterBox(point + n * (L * 0.5f), u, v, n, {L, L, L * 0.5f}), BooleanOp::Subtract, "knife",
                    error);
}

bool addBoss(Body& body, const Vec3& at, const Vec3& normal, BossShape shape, float diameter, float height,
             std::string* error) {
    if (!(diameter > 0) || (shape != BossShape::Ball && !(height > 0))) {
        if (error) *error = "The boss needs a diameter and a height";
        return false;
    }
    const Vec3 n = normalize(normal);
    const float sink = std::max(0.5f, diameter * 0.25f); // overlaps the body so the two fuse
    MeshData tool;
    switch (shape) {
    case BossShape::Cylinder: tool = cutterCylinder(at - n * sink, at + n * height, diameter * 0.5f, 48); break;
    case BossShape::Box: {
        Vec3 u, v;
        frameOf(n, u, v);
        const float h = (height + sink) * 0.5f;
        tool = cutterBox(at + n * (height - h), u, v, n, {diameter * 0.5f, diameter * 0.5f, h});
        break;
    }
    case BossShape::Ball: {
        const float r = diameter * 0.5f;
        // A dome of the given height (at most 90% of a ball, so it is sunk in and fuses).
        tool = cutterSphere(at + n * (std::min(height > 0 ? height : r, diameter * 0.9f) - r), r, 40, 20);
        break;
    }
    }
    return applyCut(body, tool, BooleanOp::Union, "boss", error);
}

bool bodyBoolean(Body& a, const Body& b, BooleanOp op, std::string* error) {
    MeshData m = a.mesh; // in a's frame (see applyCut)
    MeshData other = b.mesh;
    const Vec3 shift = b.offset - a.offset;
    if (shift.x != 0 || shift.y != 0 || shift.z != 0) other.transform(translate(shift));
    if (!robustBoolean(m, other, op)) {
        if (error)
            *error = op == BooleanOp::Subtract ? b.name + " does not overlap " + a.name + " or removes all of it"
                                               : "The bodies do not overlap";
        return false;
    }
    setLocalMesh(a, std::move(m));
    return true;
}

} // namespace sicdm

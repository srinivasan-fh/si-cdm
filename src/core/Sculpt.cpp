#include "core/Sculpt.hpp"
#include "core/HoleWizard.hpp"
#include "core/ExactMesh.hpp"

#include "core/FileIO.hpp"
#include "core/Scene.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <map>
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
    {SculptBrush::Layer, "layer", "Layer (raise to a height)"},
    {SculptBrush::ClayStrips, "claystrips", "Clay Strips"},
    {SculptBrush::Fill, "fill", "Fill (raise the low spots)"},
    {SculptBrush::Blob, "blob", "Blob"},
    {SculptBrush::SnakeHook, "snakehook", "Snake Hook (pull out along the stroke)"},
    {SculptBrush::Nudge, "nudge", "Nudge (push along the surface)"},
    {SculptBrush::Mask, "mask", "Mask (hold the surface still)"},
    {SculptBrush::Paint, "paint", "Paint (polypaint colour)"},
};

struct FalloffInfo {
    SculptFalloff falloff;
    const char* key;
};
constexpr FalloffInfo kFalloffs[] = {{SculptFalloff::Smooth, "smooth"}, {SculptFalloff::Sphere, "sphere"},
                                     {SculptFalloff::Sharp, "sharp"},   {SculptFalloff::Linear, "linear"},
                                     {SculptFalloff::Constant, "constant"}};

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
    if (k == "strips") return SculptBrush::ClayStrips;
    if (k == "snake") return SculptBrush::SnakeHook;
    return std::nullopt;
}

const char* sculptFalloffKey(SculptFalloff f) noexcept {
    for (const FalloffInfo& i : kFalloffs)
        if (i.falloff == f) return i.key;
    return "smooth";
}

std::optional<SculptFalloff> sculptFalloffFromKey(const std::string& key) noexcept {
    const std::string k = lower(key);
    for (const FalloffInfo& i : kFalloffs)
        if (k == i.key) return i.falloff;
    if (k == "flat") return SculptFalloff::Constant;
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

// Merges points closer than `tol` (hash grid). Cells are four tolerances wide, so a point
// is within `tol` of at most one side of its cell on each axis: only those neighbours
// are checked (usually just its own cell).
class Welder {
public:
    explicit Welder(double tol) : m_tol(tol), m_cell(std::max(tol * 4.0, 1e-12)) {}
    std::uint32_t add(const D3& p) {
        const long long ix = cellOf(p.x), iy = cellOf(p.y), iz = cellOf(p.z);
        auto range = [&](double v, long long i, long long& lo, long long& hi) {
            const double f = v - static_cast<double>(i) * m_cell; // 0..cell
            lo = f <= m_tol ? -1 : 0;
            hi = f >= m_cell - m_tol ? 1 : 0;
        };
        long long x0, x1, y0, y1, z0, z1;
        range(p.x, ix, x0, x1);
        range(p.y, iy, y0, y1);
        range(p.z, iz, z0, z1);
        for (long long dx = x0; dx <= x1; ++dx)
            for (long long dy = y0; dy <= y1; ++dy)
                for (long long dz = z0; dz <= z1; ++dz) {
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
    // Edges used once (sorted keys: a run of one).
    std::vector<std::uint64_t> keys;
    keys.reserve(id.size());
    for (std::size_t k = 0; k + 2 < id.size(); k += 3)
        for (std::size_t e = 0; e < 3; ++e) {
            const std::uint32_t a = id[k + e], b = id[k + (e + 1) % 3];
            if (a != b) keys.push_back(edgeKey(a, b));
        }
    std::sort(keys.begin(), keys.end());
    std::vector<std::uint32_t> open;
    for (std::size_t i = 0; i < keys.size();) {
        std::size_t j = i + 1;
        while (j < keys.size() && keys[j] == keys[i]) ++j;
        if (j - i == 1) {
            open.push_back(static_cast<std::uint32_t>(keys[i] >> 32));
            open.push_back(static_cast<std::uint32_t>(keys[i] & 0xFFFFFFFFu));
        }
        i = j;
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

// The brush's weight at distance d from its centre (1 there, 0 at the rim r).
float falloffOf(float d, float r, SculptFalloff f) {
    const float t = std::clamp(1.0f - d / r, 0.0f, 1.0f);
    switch (f) {
    case SculptFalloff::Smooth: return t * t * (3.0f - 2.0f * t);
    case SculptFalloff::Sphere: return std::sqrt(std::max(0.0f, 1.0f - (1.0f - t) * (1.0f - t)));
    case SculptFalloff::Sharp: return t * t;
    case SculptFalloff::Linear: return t;
    case SculptFalloff::Constant: return t > 0.0f ? 1.0f : 0.0f;
    }
    return t;
}


Vec2 uvMid(const Vec2& a, const Vec2& b) { return {0.5f * (a.x + b.x), 0.5f * (a.y + b.y)}; }
bool uvSame(const Vec2& a, const Vec2& b) { return std::fabs(a.x - b.x) < 1e-6f && std::fabs(a.y - b.y) < 1e-6f; }

} // namespace

// ---------------------------------------------------------------------------
// SculptMesh

SculptMesh::SculptMesh(const Body& body) {
    const AABB b = body.bounds();
    if (!b.valid()) return;
    m_size = std::max(length(b.max - b.min), 1e-3f);
    m_centre = b.center();
    m_mirrorX = m_centre.x;
    const auto& v = body.mesh.vertices;
    const auto& idx = body.mesh.indices;
    std::vector<D3> corners;
    corners.reserve(idx.size());
    for (std::size_t k = 0; k + 2 < idx.size(); k += 3)
        for (std::size_t e = 0; e < 3; ++e) corners.push_back(D3(v[idx[k + e]].position + body.offset));
    std::vector<D3> pts;
    const std::vector<std::uint32_t> id = weldCorners(corners, m_size, pts);
    m_t.reserve(idx.size() / 3);
    const bool grouped = body.mesh.hasGroups(), mapped = body.mesh.hasUvs();
    for (std::size_t k = 0; k + 2 < id.size(); k += 3) {
        const std::array<std::uint32_t, 3> t{id[k], id[k + 1], id[k + 2]};
        if (t[0] == t[1] || t[1] == t[2] || t[2] == t[0]) continue;
        m_t.push_back(t);
        if (grouped) m_group.push_back(body.mesh.groups[k / 3]);
        if (mapped) m_uv.push_back({body.mesh.uvs[k], body.mesh.uvs[k + 1], body.mesh.uvs[k + 2]});
    }
    m_p.reserve(pts.size());
    for (const D3& p : pts) m_p.push_back(p.f());
    m_base = body.color;
    if (body.mesh.hasIds()) {
        m_ids.assign(m_p.size(), 0xFFFFFFFFu);
        for (std::size_t k = 0; k < id.size() && k < idx.size(); ++k) m_ids[id[k]] = body.mesh.ids[idx[k]];
    }
    // Sculpt layers: a welded vertex's offsets are its corners' (they are the same).
    if (body.mesh.hasLayers()) {
        m_layers = body.mesh.layers;
        for (SculptLayer& l : m_layers) {
            std::vector<Vec3> off(m_p.size(), Vec3{});
            for (std::size_t k = 0; k < id.size() && k < idx.size(); ++k) off[id[k]] = l.offset[idx[k]];
            l.offset = std::move(off);
        }
        m_record = body.mesh.recordLayer >= 0 && body.mesh.recordLayer < static_cast<int>(m_layers.size()) ? body.mesh.recordLayer : -1;
        if (m_record >= 0) m_start = m_p;
    }
    // Polypaint: a vertex welded from several corners takes a painted one's colour.
    if (std::any_of(v.begin(), v.end(), [](const Vertex& c) { return painted(c.color); })) {
        m_color.assign(m_p.size(), packColor(m_base));
        for (std::size_t k = 0; k < id.size() && k < idx.size(); ++k)
            if (painted(v[idx[k]].color)) m_color[id[k]] = v[idx[k]].color;
    }
    // A vertex welded from several corners is as masked as the most masked of them.
    if (std::any_of(v.begin(), v.end(), [](const Vertex& c) { return c.mask > 0.0f; })) {
        m_mask.assign(m_p.size(), 0.0f);
        for (std::size_t k = 0; k < id.size() && k < idx.size(); ++k) m_mask[id[k]] = std::max(m_mask[id[k]], v[idx[k]].mask);
    }
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
            m_uv.clear(); // rare (booleans' caps): the texture coordinates go
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
    // The nearest vertex bounds the answer (it is on the surface); a triangle whose box
    // is farther than the best so far can't hold it, so only the near ones are tested.
    float best = 1e30f;
    for (const Vec3& p : m_p) {
        const Vec3 d = p - q;
        best = std::min(best, dot(d, d));
    }
    best *= 1.0001f;
    Vec3 bestP = q;
    int hit = -1;
    for (std::size_t i = 0; i < m_t.size(); ++i) {
        const auto& t = m_t[i];
        const Vec3 &a = m_p[t[0]], &b = m_p[t[1]], &c = m_p[t[2]];
        // Distance from q to the triangle's box, one axis at a time (most fail on the first).
        auto gap = [](float u, float v, float w, float x) {
            const float lo = u < v ? (u < w ? u : w) : (v < w ? v : w), hi = u > v ? (u > w ? u : w) : (v > w ? v : w);
            return x < lo ? lo - x : x > hi ? x - hi : 0.0f;
        };
        const float gx = gap(a.x, b.x, c.x, q.x);
        float g2 = gx * gx;
        if (g2 >= best) continue;
        const float gy = gap(a.y, b.y, c.y, q.y);
        g2 += gy * gy;
        if (g2 >= best) continue;
        const float gz = gap(a.z, b.z, c.z, q.z);
        if (g2 + gz * gz >= best) continue;
        const Vec3 p = closestOnTriangle(q, a, b, c);
        const Vec3 d = p - q;
        const float d2 = dot(d, d);
        if (d2 < best) {
            best = d2;
            bestP = p;
            hit = static_cast<int>(i);
        }
    }
    if (normal && hit >= 0) {
        const auto& t = m_t[static_cast<std::size_t>(hit)];
        *normal = normalize(cross(m_p[t[1]] - m_p[t[0]], m_p[t[2]] - m_p[t[0]]));
    }
    return bestP;
}

SurfaceIndex::SurfaceIndex(const SculptMesh& mesh) : m_mesh(mesh) {
    const auto& P = mesh.positions();
    const auto& T = mesh.triangles();
    if (T.empty()) return;
    AABB box;
    for (const Vec3& p : P) box.expand(p);
    const Vec3 size = box.max - box.min;
    // About two triangles a cell, at most 2 million cells.
    const double vol = std::max(static_cast<double>(size.x) * size.y * size.z, 1e-9);
    const double cells = std::clamp(static_cast<double>(T.size()) / 2.0, 1.0, 2.0e6);
    m_cell = static_cast<float>(std::max(std::cbrt(vol / cells), static_cast<double>(std::max({size.x, size.y, size.z})) / 256.0));
    m_cell = std::max(m_cell, 1e-6f);
    m_min = box.min;
    for (int k = 0; k < 3; ++k) m_n[k] = std::max(1, static_cast<int>(std::ceil((k == 0 ? size.x : k == 1 ? size.y : size.z) / m_cell)) + 1);
    auto cellRange = [&](const std::array<std::uint32_t, 3>& t, int lo[3], int hi[3]) {
        for (int k = 0; k < 3; ++k) {
            float mn = 1e30f, mx = -1e30f;
            for (std::uint32_t v : t) {
                const float c = k == 0 ? P[v].x : k == 1 ? P[v].y : P[v].z;
                mn = std::min(mn, c), mx = std::max(mx, c);
            }
            const float o = k == 0 ? m_min.x : k == 1 ? m_min.y : m_min.z;
            lo[k] = std::clamp(static_cast<int>((mn - o) / m_cell), 0, m_n[k] - 1);
            hi[k] = std::clamp(static_cast<int>((mx - o) / m_cell), 0, m_n[k] - 1);
        }
    };
    const std::size_t ncell = static_cast<std::size_t>(m_n[0]) * static_cast<std::size_t>(m_n[1]) * static_cast<std::size_t>(m_n[2]);
    auto cellIndex = [&](int x, int y, int z) { return (static_cast<std::size_t>(z) * static_cast<std::size_t>(m_n[1]) + static_cast<std::size_t>(y)) * static_cast<std::size_t>(m_n[0]) + static_cast<std::size_t>(x); };
    std::vector<std::uint32_t> count(ncell + 1, 0);
    int lo[3], hi[3];
    for (const auto& t : T) {
        cellRange(t, lo, hi);
        for (int z = lo[2]; z <= hi[2]; ++z)
            for (int y = lo[1]; y <= hi[1]; ++y)
                for (int x = lo[0]; x <= hi[0]; ++x) ++count[cellIndex(x, y, z) + 1];
    }
    for (std::size_t i = 1; i <= ncell; ++i) count[i] += count[i - 1];
    m_start = count;
    m_items.resize(count[ncell]);
    for (std::size_t ti = 0; ti < T.size(); ++ti) {
        cellRange(T[ti], lo, hi);
        for (int z = lo[2]; z <= hi[2]; ++z)
            for (int y = lo[1]; y <= hi[1]; ++y)
                for (int x = lo[0]; x <= hi[0]; ++x) m_items[count[cellIndex(x, y, z)]++] = static_cast<std::uint32_t>(ti);
    }
    m_seen.assign(T.size(), 0);
}

Vec3 SurfaceIndex::closestPoint(const Vec3& q, Vec3* normal) const {
    const auto& P = m_mesh.positions();
    const auto& T = m_mesh.triangles();
    if (m_items.empty()) return m_mesh.closestPoint(q, normal);
    if (++m_query == 0) { // wrapped: start the marks again
        std::fill(m_seen.begin(), m_seen.end(), 0);
        m_query = 1;
    }
    int c[3];
    for (int k = 0; k < 3; ++k) {
        const float o = k == 0 ? m_min.x : k == 1 ? m_min.y : m_min.z, v = k == 0 ? q.x : k == 1 ? q.y : q.z;
        c[k] = std::clamp(static_cast<int>(std::floor((v - o) / m_cell)), 0, m_n[k] - 1);
    }
    float best = 1e30f;
    Vec3 bestP = q;
    int hit = -1;
    const int maxR = std::max({m_n[0], m_n[1], m_n[2]});
    for (int r = 0; r <= maxR; ++r) {
        // The shell of cells r away (Chebyshev) from q's cell.
        for (int z = c[2] - r; z <= c[2] + r; ++z) {
            if (z < 0 || z >= m_n[2]) continue;
            for (int y = c[1] - r; y <= c[1] + r; ++y) {
                if (y < 0 || y >= m_n[1]) continue;
                const bool inner = std::abs(z - c[2]) < r && std::abs(y - c[1]) < r;
                for (int x = c[0] - r; x <= c[0] + r; x += (inner && r > 0) ? 2 * r : 1) {
                    if (x < 0 || x >= m_n[0]) continue;
                    const std::size_t cell = (static_cast<std::size_t>(z) * static_cast<std::size_t>(m_n[1]) + static_cast<std::size_t>(y)) * static_cast<std::size_t>(m_n[0]) +
                                             static_cast<std::size_t>(x);
                    for (std::uint32_t i = m_start[cell]; i < m_start[cell + 1]; ++i) {
                        const std::uint32_t ti = m_items[i];
                        if (m_seen[ti] == m_query) continue;
                        m_seen[ti] = m_query;
                        const auto& t = T[ti];
                        const Vec3 p = closestOnTriangle(q, P[t[0]], P[t[1]], P[t[2]]);
                        const Vec3 d = p - q;
                        const float d2 = dot(d, d);
                        if (d2 < best) {
                            best = d2;
                            bestP = p;
                            hit = static_cast<int>(ti);
                        }
                    }
                }
            }
        }
        // Every cell not yet searched is at least r cells from q's own.
        const float reach = static_cast<float>(r) * m_cell;
        if (hit >= 0 && best <= reach * reach) break;
    }
    if (normal && hit >= 0) {
        const auto& t = T[static_cast<std::size_t>(hit)];
        *normal = normalize(cross(P[t[1]] - P[t[0]], P[t[2]] - P[t[0]]));
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
        if (!m_cut.empty() && m_cut.count(k)) continue; // a window's cut border stays as the mesh around it has it
        mid.emplace(k, static_cast<std::uint32_t>(m_p.size()));
        // Not exactly half way: crossing edges (the overlapping slivers a boolean can
        // leave) would otherwise get the same midpoint. At most 0.05% of the edge.
        const float t = 0.5f + static_cast<float>(static_cast<int>((k * 2654435761ull >> 20) % 1001) - 500) * 1e-6f;
        m_p.push_back(m_p[a] + (m_p[b] - m_p[a]) * t);
        m_n.push_back(normalize(m_n[a] + m_n[b]));
        addVertexExtras(a, b);
        touched[a] = touched[b] = 1;
    }
    if (mid.empty()) return false;
    const std::size_t count = m_t.size();
    const bool mapped = m_uv.size() == count;
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
        // Texture coordinates of the pieces: the corners' own, the new points' half way.
        const std::array<Vec2, 3> tuv = mapped ? m_uv[i] : std::array<Vec2, 3>{};
        auto uvOf = [&](std::uint32_t v) {
            for (int k = 0; k < 3; ++k)
                if (tri[static_cast<std::size_t>(k)] == v) return tuv[static_cast<std::size_t>(k)];
            for (int k = 0; k < 3; ++k)
                if (m[k] == v) return uvMid(tuv[static_cast<std::size_t>(k)], tuv[static_cast<std::size_t>((k + 1) % 3)]);
            return Vec2{};
        };
        auto uvTri = [&](const std::array<std::uint32_t, 3>& t) { return std::array<Vec2, 3>{uvOf(t[0]), uvOf(t[1]), uvOf(t[2])}; };
        auto put = [&](const std::array<std::uint32_t, 3>& t) {
            m_t[i] = t;
            if (mapped) m_uv[i] = uvTri(t);
        };
        auto add = [&](const std::array<std::uint32_t, 3>& t) { // a new piece, in the split triangle's group
            m_t.push_back(t);
            if (!m_group.empty()) m_group.push_back(m_group[i]);
            if (mapped) m_uv.push_back(uvTri(t));
        };
        if (marked == 3) {
            add({m[0], tri[1], m[1]});
            add({m[2], m[1], tri[2]});
            add({m[0], m[1], m[2]});
            put({tri[0], m[0], m[2]});
        } else if (marked == 1) {
            const int e = m[0] != kNone ? 0 : m[1] != kNone ? 1 : 2;
            const std::uint32_t a = tri[e], b = tri[(e + 1) % 3], c = tri[(e + 2) % 3], ab = m[e];
            add({ab, b, c});
            put({a, ab, c});
        } else { // two edges: a corner triangle plus a quad split along its shorter diagonal
            const int e = m[0] == kNone ? 0 : m[1] == kNone ? 1 : 2; // the unsplit edge a-b
            const std::uint32_t a = tri[e], b = tri[(e + 1) % 3], c = tri[(e + 2) % 3];
            const std::uint32_t bc = m[(e + 1) % 3], ca = m[(e + 2) % 3];
            put({ca, bc, c});
            if (length(m_p[a] - m_p[bc]) < length(m_p[b] - m_p[ca])) {
                add({a, b, bc});
                add({a, bc, ca});
            } else {
                add({a, b, ca});
                add({b, bc, ca});
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

void SculptMesh::flipTowardsValence() {
    std::vector<int> valence(m_p.size(), 0);
    std::unordered_map<std::uint64_t, std::array<std::uint32_t, 2>> faces; // an edge's two triangles
    std::unordered_map<std::uint64_t, int> count;
    for (std::size_t i = 0; i < m_t.size(); ++i)
        for (int e = 0; e < 3; ++e) {
            const std::uint64_t k = edgeKey(m_t[i][static_cast<std::size_t>(e)], m_t[i][static_cast<std::size_t>((e + 1) % 3)]);
            int& c = count[k];
            if (c < 2) faces[k][static_cast<std::size_t>(c)] = static_cast<std::uint32_t>(i);
            ++c;
        }
    std::vector<char> border(m_p.size(), 0);
    for (const auto& [k, c] : count) {
        const auto a = static_cast<std::uint32_t>(k >> 32), b = static_cast<std::uint32_t>(k & 0xFFFFFFFFu);
        ++valence[a], ++valence[b];
        if (c != 2) border[a] = border[b] = 1;
    }
    auto ideal = [&](std::uint32_t v) { return border[v] ? 4 : 6; };
    std::vector<char> used(m_t.size(), 0);
    for (const auto& [k, c] : count) {
        if (c != 2) continue;
        const auto& f = faces[k];
        if (used[f[0]] || used[f[1]] || m_creaseEdges.count(k)) continue;
        const auto a = static_cast<std::uint32_t>(k >> 32), b = static_cast<std::uint32_t>(k & 0xFFFFFFFFu);
        auto opposite = [&](const std::array<std::uint32_t, 3>& t) {
            for (std::uint32_t v : t)
                if (v != a && v != b) return v;
            return a;
        };
        const std::uint32_t c0 = opposite(m_t[f[0]]), d0 = opposite(m_t[f[1]]);
        if (c0 == d0 || count.count(edgeKey(c0, d0))) continue;
        auto dev = [&](int va, int vb, int vc, int vd) {
            auto sq = [](int x) { return x * x; };
            return sq(va - ideal(a)) + sq(vb - ideal(b)) + sq(vc - ideal(c0)) + sq(vd - ideal(d0));
        };
        const int before = dev(valence[a], valence[b], valence[c0], valence[d0]);
        const int after = dev(valence[a] - 1, valence[b] - 1, valence[c0] + 1, valence[d0] + 1);
        if (after >= before || valence[a] <= 3 || valence[b] <= 3) continue;
        // The new triangles (c, d, ...) keep the old faces' side and don't fold.
        const std::array<std::uint32_t, 3>& t0 = m_t[f[0]];
        int ia = 0;
        while (t0[static_cast<std::size_t>(ia)] != a) ++ia;
        const bool abInOrder = t0[static_cast<std::size_t>((ia + 1) % 3)] == b; // t0 = (a, b, c)
        const std::array<std::uint32_t, 3> n0 = abInOrder ? std::array<std::uint32_t, 3>{c0, a, d0} : std::array<std::uint32_t, 3>{c0, d0, a};
        const std::array<std::uint32_t, 3> n1 = abInOrder ? std::array<std::uint32_t, 3>{d0, b, c0} : std::array<std::uint32_t, 3>{d0, c0, b};
        auto normalOf = [&](const std::array<std::uint32_t, 3>& t) { return cross(m_p[t[1]] - m_p[t[0]], m_p[t[2]] - m_p[t[0]]); };
        const Vec3 o0 = normalize(normalOf(m_t[f[0]])), o1 = normalize(normalOf(m_t[f[1]]));
        const Vec3 m0 = normalOf(n0), m1 = normalOf(n1);
        if (length(m0) < 1e-12f || length(m1) < 1e-12f) continue;
        if (dot(normalize(m0), o0 + o1) < 0.5f || dot(normalize(m1), o0 + o1) < 0.5f || dot(normalize(m0), normalize(m1)) < 0.5f) continue;
        if (m_uv.size() == m_t.size()) { // the texture flips with it, unless the edge is a seam
            auto uvIn = [&](std::uint32_t ti, std::uint32_t v) {
                const auto& t = m_t[ti];
                return m_uv[ti][t[0] == v ? 0 : t[1] == v ? 1 : 2];
            };
            if (!uvSame(uvIn(f[0], a), uvIn(f[1], a)) || !uvSame(uvIn(f[0], b), uvIn(f[1], b))) continue;
            auto uvOf = [&](std::uint32_t v) { return v == d0 ? uvIn(f[1], v) : uvIn(f[0], v); };
            const std::array<Vec2, 3> u0{uvOf(n0[0]), uvOf(n0[1]), uvOf(n0[2])}, u1{uvOf(n1[0]), uvOf(n1[1]), uvOf(n1[2])};
            m_uv[f[0]] = u0, m_uv[f[1]] = u1;
        }
        m_t[f[0]] = n0, m_t[f[1]] = n1;
        used[f[0]] = used[f[1]] = 1;
        --valence[a], --valence[b], ++valence[c0], ++valence[d0];
    }
}

void SculptMesh::relaxOnto(const SurfaceIndex& target, float amount) {
    // Each vertex towards the average of its neighbours, along the surface only, then back
    // onto the target surface. Border vertices stay.
    std::vector<Vec3> sum(m_p.size(), Vec3{});
    std::vector<int> n(m_p.size(), 0);
    std::unordered_map<std::uint64_t, int> count;
    for (const auto& t : m_t)
        for (int e = 0; e < 3; ++e) ++count[edgeKey(t[static_cast<std::size_t>(e)], t[static_cast<std::size_t>((e + 1) % 3)])];
    std::vector<char> border(m_p.size(), 0);
    for (const auto& [k, c] : count) {
        const auto a = static_cast<std::uint32_t>(k >> 32), b = static_cast<std::uint32_t>(k & 0xFFFFFFFFu);
        sum[a] += m_p[b], sum[b] += m_p[a];
        ++n[a], ++n[b];
        if (c != 2) border[a] = border[b] = 1;
    }
    // Kept creases: a crease vertex towards the middle of its two crease neighbours.
    const bool creases = m_crease.size() == m_p.size();
    std::vector<Vec3> along;
    if (creases) {
        along.assign(m_p.size(), Vec3{});
        for (const std::uint64_t k : m_creaseEdges) {
            const auto a = static_cast<std::uint32_t>(k >> 32), b = static_cast<std::uint32_t>(k & 0xFFFFFFFFu);
            along[a] += m_p[b] * 0.5f, along[b] += m_p[a] * 0.5f;
        }
    }
    updateNormals();
    const std::vector<Vec3> before = creases ? m_p : std::vector<Vec3>{};
    for (std::size_t v = 0; v < m_p.size(); ++v) {
        if (border[v] || n[v] == 0) continue;
        if (creases && m_crease[v] == 2) continue;
        if (creases && m_crease[v] == 1) {
            m_p[v] = before[v] + (along[v] - before[v]) * amount;
            continue;
        }
        Vec3 d = sum[v] * (1.0f / static_cast<float>(n[v])) - m_p[v];
        d -= m_n[v] * dot(d, m_n[v]);
        m_p[v] = target.closestPoint(m_p[v] + d * amount);
    }
    updateNormals();
}

void SculptMesh::remeshIsotropic(float L, int iterations, const SculptMesh& target, float creaseAngle, float creaseChain) {
    m_mask.clear(), m_layer.clear(), m_color.clear(), m_group.clear(), m_layers.clear(), m_start.clear(), m_ids.clear(), m_uv.clear();
    m_record = -1;
    const SurfaceIndex index(target);
    // Creases are kept by keeping them made of edges: the target's own edges lie on them,
    // splits halve them, and nothing else may move a vertex off one.
    auto mark = [&] {
        if (creaseAngle > 0) markCreases(creaseAngle, creaseChain);
    };
    for (int it = 0; it < iterations; ++it) {
        mark();
        refineAll(L * 4.0f / 3.0f, 8000000);
        mark();
        collapseShort(L * 0.8f, {}, 0.0f, true);
        mark();
        for (int f = 0; f < 3; ++f) flipTowardsValence();
        for (int r = 0; r < 3; ++r) relaxOnto(index, 0.8f);
    }
    m_creaseEdges.clear(), m_crease.clear();
}

void SculptMesh::markCreases(float angle, float minChain) {
    m_creaseEdges.clear();
    std::unordered_map<std::uint64_t, std::array<Vec3, 2>> normals;
    std::unordered_map<std::uint64_t, int> count;
    for (const auto& t : m_t) {
        const Vec3 n = normalize(cross(m_p[t[1]] - m_p[t[0]], m_p[t[2]] - m_p[t[0]]));
        for (int e = 0; e < 3; ++e) {
            const std::uint64_t k = edgeKey(t[static_cast<std::size_t>(e)], t[static_cast<std::size_t>((e + 1) % 3)]);
            int& c = count[k];
            if (c < 2) normals[k][static_cast<std::size_t>(c)] = n;
            ++c;
        }
    }
    const float cosLimit = std::cos(angle * 3.14159265f / 180.0f);
    std::vector<std::uint64_t> sharp;
    for (const auto& [k, c] : count)
        if (c == 2 && dot(normals[k][0], normals[k][1]) < cosLimit) sharp.push_back(k);
    // Chains (sharp edges end to end) shorter than minChain are left out.
    std::vector<std::uint32_t> parent(m_p.size());
    for (std::size_t v = 0; v < parent.size(); ++v) parent[v] = static_cast<std::uint32_t>(v);
    auto root = [&](std::uint32_t v) {
        while (parent[v] != v) v = parent[v] = parent[parent[v]];
        return v;
    };
    for (const std::uint64_t k : sharp) parent[root(static_cast<std::uint32_t>(k >> 32))] = root(static_cast<std::uint32_t>(k & 0xFFFFFFFFu));
    std::unordered_map<std::uint32_t, float> chain;
    for (const std::uint64_t k : sharp) chain[root(static_cast<std::uint32_t>(k >> 32))] += length(m_p[k >> 32] - m_p[k & 0xFFFFFFFFu]);
    std::vector<int> degree(m_p.size(), 0);
    for (const std::uint64_t k : sharp) {
        if (chain[root(static_cast<std::uint32_t>(k >> 32))] < minChain) continue;
        m_creaseEdges.insert(k);
        ++degree[static_cast<std::size_t>(k >> 32)], ++degree[static_cast<std::size_t>(k & 0xFFFFFFFFu)];
    }
    m_crease.assign(m_p.size(), 0);
    for (std::size_t v = 0; v < m_p.size(); ++v) m_crease[v] = static_cast<char>(degree[v] == 0 ? 0 : degree[v] == 2 ? 1 : 2);
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
            if (locked[a] || locked[b] || border[a] || border[b] || pinned(a) || pinned(b)) continue;
            // Kept creases: corners stay, and a crease vertex goes only along its crease.
            if (!m_crease.empty() && a < m_crease.size() && b < m_crease.size() &&
                (m_crease[a] == 2 || m_crease[b] == 2 || ((m_crease[a] || m_crease[b]) && !m_creaseEdges.count(k))))
                continue;
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
            if (m_uv.size() == m_t.size()) {
                // Corners of a and b that were the edge's ends in either dying triangle's
                // texture go half way (a seam's other sides keep theirs).
                std::array<Vec2, 2> was[2];
                Vec2 mid[2];
                int dying = 0;
                for (std::uint32_t v : {a, b})
                    for (std::uint32_t ti : around[v]) {
                        if (dead[ti] || dying == 2) continue;
                        const auto& t = m_t[ti];
                        const int ia = t[0] == a ? 0 : t[1] == a ? 1 : t[2] == a ? 2 : -1, ib = t[0] == b ? 0 : t[1] == b ? 1 : t[2] == b ? 2 : -1;
                        if (ia < 0 || ib < 0) continue;
                        const Vec2 ua = m_uv[ti][static_cast<std::size_t>(ia)], ub = m_uv[ti][static_cast<std::size_t>(ib)];
                        if (dying == 1 && uvSame(was[0][0], ua) && uvSame(was[0][1], ub)) continue;
                        was[dying] = {ua, ub}, mid[dying] = uvMid(ua, ub);
                        ++dying;
                    }
                for (std::uint32_t v : {a, b})
                    for (std::uint32_t ti : around[v]) {
                        if (dead[ti]) continue;
                        const auto& t = m_t[ti];
                        for (int c = 0; c < 3; ++c) {
                            if (t[static_cast<std::size_t>(c)] != a && t[static_cast<std::size_t>(c)] != b) continue;
                            Vec2& u = m_uv[ti][static_cast<std::size_t>(c)];
                            for (int d = 0; d < dying; ++d)
                                if (uvSame(u, was[d][0]) || uvSame(u, was[d][1])) {
                                    u = mid[d];
                                    break;
                                }
                        }
                    }
            }
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
            if (!m_mask.empty()) m_mask[a] = std::max(m_mask[a], m_mask[b]);
            if (!m_layer.empty()) m_layer[a] = 0.5f * (m_layer[a] + m_layer[b]);
            if (!m_color.empty()) m_color[a] = packColor((unpackColor(m_color[a]) + unpackColor(m_color[b])) * 0.5f);
            for (SculptLayer& l : m_layers) l.offset[a] = (l.offset[a] + l.offset[b]) * 0.5f;
            if (!m_start.empty()) m_start[a] = (m_start[a] + m_start[b]) * 0.5f;
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
            if (!dead[i]) {
                if (!m_group.empty()) m_group[n] = m_group[i];
                if (m_uv.size() == m_t.size()) m_uv[n] = m_uv[i];
                m_t[n++] = m_t[i];
            }
        if (m_uv.size() == m_t.size()) m_uv.resize(n);
        else m_uv.clear();
        m_t.resize(n);
        if (!m_group.empty()) m_group.resize(n);
        dropUnused();
        m_n.resize(m_p.size()); // stale: the caller updates the normals
    }
    return total;
}

// Drops the vertices no triangle uses any more (a window keeps track of where its
// vertices came from).
void SculptMesh::dropUnused() {
    std::vector<std::uint32_t> remap(m_p.size(), 0xFFFFFFFFu);
    std::vector<Vec3> p;
    std::vector<std::uint32_t> from;
    std::vector<char> pins;
    std::vector<float> mask, layer;
    std::vector<std::uint32_t> color;
    std::vector<Vec3> start;
    std::vector<std::uint32_t> ids;
    std::vector<std::vector<Vec3>> lay(m_layers.size());
    for (auto& t : m_t)
        for (auto& v : t) {
            if (remap[v] == 0xFFFFFFFFu) {
                remap[v] = static_cast<std::uint32_t>(p.size());
                p.push_back(m_p[v]);
                if (!m_from.empty()) {
                    from.push_back(m_from[v]);
                    pins.push_back(m_pinned[v]);
                }
                if (!m_mask.empty()) mask.push_back(m_mask[v]);
                if (!m_layer.empty()) layer.push_back(m_layer[v]);
                if (!m_color.empty()) color.push_back(m_color[v]);
                if (!m_start.empty()) start.push_back(m_start[v]);
                if (!m_ids.empty()) ids.push_back(m_ids[v]);
                for (std::size_t k = 0; k < m_layers.size(); ++k) lay[k].push_back(m_layers[k].offset[v]);
            }
            v = remap[v];
        }
    m_p = std::move(p);
    if (!m_from.empty()) {
        m_from = std::move(from);
        m_pinned = std::move(pins);
    }
    if (!m_mask.empty()) m_mask = std::move(mask);
    if (!m_layer.empty()) m_layer = std::move(layer);
    if (!m_color.empty()) m_color = std::move(color);
    if (!m_start.empty()) m_start = std::move(start);
    if (!m_ids.empty()) m_ids = std::move(ids);
    for (std::size_t k = 0; k < m_layers.size(); ++k) m_layers[k].offset = std::move(lay[k]);
}

void SculptMesh::addVertexExtras(std::uint32_t a, std::uint32_t b) {
    if (!m_from.empty()) {
        m_from.push_back(0xFFFFFFFFu);
        m_pinned.push_back(0);
    }
    if (!m_mask.empty()) m_mask.push_back(0.5f * (m_mask[a] + m_mask[b]));
    if (!m_layer.empty()) m_layer.push_back(0.5f * (m_layer[a] + m_layer[b]));
    if (!m_color.empty()) m_color.push_back(packColor((unpackColor(m_color[a]) + unpackColor(m_color[b])) * 0.5f));
    for (SculptLayer& l : m_layers) l.offset.push_back((l.offset[a] + l.offset[b]) * 0.5f);
    if (!m_start.empty()) m_start.push_back((m_start[a] + m_start[b]) * 0.5f);
    if (!m_ids.empty()) m_ids.push_back(0xFFFFFFFFu);
}

std::vector<SculptMesh::Image> SculptMesh::images(const Vec3& centre, const SculptSettings& s, float R) const {
    // Every combination of the mirrors, each turned through the radial copies, about
    // the body's centre. Copies that land almost on another (a dab on a mirror plane)
    // are dropped, so the plane gets no double strength.
    std::vector<Image> out;
    const int copies = std::max(1, s.radial);
    const int ra = std::clamp(s.radialAxis, 0, 2);
    for (int m = 0; m < 8; ++m) {
        if (((m & 1) && !s.symmetryX) || ((m & 2) && !s.symmetryY) || ((m & 4) && !s.symmetryZ)) continue;
        const Vec3 flip{(m & 1) ? -1.0f : 1.0f, (m & 2) ? -1.0f : 1.0f, (m & 4) ? -1.0f : 1.0f};
        for (int k = 0; k < copies; ++k) {
            const float ang = 2.0f * kPi * static_cast<float>(k) / static_cast<float>(copies);
            const float c = std::cos(ang), sn = std::sin(ang);
            auto turn = [&](Vec3 v) {
                const int a = (ra + 1) % 3, b = (ra + 2) % 3; // the plane of the turn
                float* f = &v.x;
                const float x = f[a], y = f[b];
                f[a] = x * c - y * sn;
                f[b] = x * sn + y * c;
                return v;
            };
            Image img{};
            img.axes[0] = turn({flip.x, 0, 0});
            img.axes[1] = turn({0, flip.y, 0});
            img.axes[2] = turn({0, 0, flip.z});
            img.centre = m_centre + img.apply(centre - m_centre);
            bool near = false;
            for (const Image& o : out) near = near || length(o.centre - img.centre) <= R * 0.25f;
            if (!near) out.push_back(img);
        }
    }
    return out;
}

void SculptMesh::applyDab(const Vec3& centre, const SculptSettings& s, const Vec3& dir) {
    const float R = std::max({s.radius, m_size * 2e-3f, 1e-3f});
    // Clay Strips works on a square under the brush, the rest on its disc; with an alpha,
    // the whole square the alpha covers (the falloff spread to its corners).
    const bool alpha = s.alpha && !s.alpha->empty();
    const float span = alpha ? R * 1.4143f : R;
    const float reach = s.brush == SculptBrush::ClayStrips ? std::max(R * 1.15f, span) : span, reach2 = reach * reach;
    std::vector<std::pair<std::uint32_t, float>> region;
    for (std::size_t i = 0; i < m_p.size(); ++i) {
        const Vec3 d = m_p[i] - centre;
        const float dd = dot(d, d);
        if (dd < reach2 && !pinned(static_cast<std::uint32_t>(i))) region.emplace_back(static_cast<std::uint32_t>(i), falloffOf(std::sqrt(dd), span, s.falloff));
    }
    if (region.empty()) return;
    // Front faces only: the surface facing the way the dab's own spot does. A brush
    // larger than a thin part would otherwise reach through to its far side (and
    // pull its plane there).
    {
        std::uint32_t nearest = region.front().first;
        float best = 1e30f;
        for (const auto& [i, f] : region) {
            const Vec3 d = m_p[i] - centre;
            if (dot(d, d) < best) {
                best = dot(d, d);
                nearest = i;
            }
        }
        const Vec3 facing = m_n[nearest];
        region.erase(std::remove_if(region.begin(), region.end(), [&](const auto& e) { return dot(m_n[e.first], facing) <= 0.0f; }),
                     region.end());
        if (region.empty()) return;
        if (alpha) { // pressed through the alpha, laid on the surface's tangent plane
            const Vec3 n = normalize(facing);
            Vec3 t = s.alphaFollow ? dir - n * dot(dir, n) : Vec3{};
            if (length(t) < 1e-9f) { // no heading: the world axis least along the normal
                const Vec3 ref = std::fabs(n.x) < 0.9f ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
                t = ref - n * dot(ref, n);
            }
            t = normalize(t);
            Vec3 b = cross(n, t);
            const float ang = radians(s.alphaAngle), ca = std::cos(ang), sa = std::sin(ang);
            const Vec3 tr = t * ca + b * sa;
            b = cross(n, tr);
            for (auto& [i, f] : region) {
                const Vec3 d = m_p[i] - centre;
                f *= s.alpha->sample(dot(d, tr) / R, dot(d, b) / R, s.alphaTile);
            }
            region.erase(std::remove_if(region.begin(), region.end(), [](const auto& e) { return !(e.second > 0.0f); }), region.end());
            if (region.empty()) return;
        }
    }

    const float sign = s.invert ? -1.0f : 1.0f;
    const float st = std::isfinite(s.strength) ? std::clamp(s.strength, 0.0f, 1.0f) : 0.0f;
    if (s.brush == SculptBrush::Mask) { // paints the mask (inverted: wipes it)
        if (m_mask.empty()) m_mask.assign(m_p.size(), 0.0f);
        for (const auto& [i, f] : region) m_mask[i] = std::clamp(m_mask[i] + sign * st * f, 0.0f, 1.0f);
        return;
    }
    if (s.brush == SculptBrush::Paint) { // lays colour on (inverted: back to the body's own); the mask holds it off
        if (m_color.empty()) m_color.assign(m_p.size(), packColor(m_base));
        const Vec3 to = s.invert ? m_base : Vec3{std::clamp(s.color.x, 0.0f, 1.0f), std::clamp(s.color.y, 0.0f, 1.0f), std::clamp(s.color.z, 0.0f, 1.0f)};
        for (const auto& [i, f] : region) {
            const float w = std::clamp(st * f * (1.0f - held(i)) * 2.0f, 0.0f, 1.0f);
            const Vec3 c = unpackColor(m_color[i]);
            m_color[i] = packColor(c + (to - c) * w);
        }
        return;
    }

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
    // The stroke's heading in the brush plane (Clay Strips, Nudge), and a frame for the strip.
    Vec3 along = dir - N * dot(dir, N);
    const bool heading = length(along) > 1e-6f;
    if (!heading) along = std::fabs(N.x) < 0.9f ? cross(N, Vec3{1, 0, 0}) : cross(N, Vec3{0, 1, 0});
    along = normalize(along);
    const Vec3 side = cross(N, along);
    if (s.brush == SculptBrush::Layer && m_layer.size() != m_p.size()) m_layer.assign(m_p.size(), 0.0f);
    const float layerHeight = 0.25f * R * st;

    const Vec3 clayPlane = C + N * (sign * 0.12f * R * st);
    for (std::size_t r = 0; r < region.size(); ++r) {
        auto [i, f] = region[r];
        f *= 1.0f - held(i); // masked: held still
        if (f <= 0.0f) continue;
        Vec3& p = m_p[i];
        switch (s.brush) {
        case SculptBrush::Draw: p += N * (sign * k * f); break;
        case SculptBrush::Carve: p -= N * (sign * k * 1.3f * f * f); break;
        case SculptBrush::Clay: {
            const float gap = dot(clayPlane - p, N) * sign; // below the plateau: fill up towards it
            if (gap > 0) p += N * (sign * gap * f * 0.5f);
            break;
        }
        case SculptBrush::ClayStrips: {
            // A flat-topped strip under a square brush, laid along the stroke.
            const Vec3 d = p - centre;
            const float q = std::max(std::fabs(dot(d, along)) / R, std::fabs(dot(d, side)) / (0.55f * R));
            if (q >= 1.0f) break;
            const float w = falloffOf(q * R, R, s.falloff) * (1.0f - held(i));
            const float gap = dot(clayPlane - p, N) * sign;
            if (gap > 0) p += N * (sign * gap * w * 0.6f);
            break;
        }
        case SculptBrush::Inflate: p += m_n[i] * (sign * k * f); break;
        case SculptBrush::Blob: { // swells out along its own normals and spreads from the centre
            p += m_n[i] * (sign * k * f);
            Vec3 v = p - C;
            v -= N * dot(v, N);
            p += v * (sign * 0.12f * st * f);
            break;
        }
        case SculptBrush::Layer: { // up to a constant height over the surface, once a stroke
            const float target = sign * layerHeight * f, done = m_layer[i];
            const float step = sign > 0 ? std::min(target - done, k * 2.0f * f) : std::max(target - done, -k * 2.0f * f);
            if (step * sign > 0) {
                p += m_n[i] * step;
                m_layer[i] += step;
            }
            break;
        }
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
        case SculptBrush::Fill: {
            const float h = dot(C - p, N); // below the plane (or above when inverted): fill up to it
            if (h * sign > 0) p += N * (h * f * st);
            break;
        }
        case SculptBrush::SnakeHook: {
            // The surface follows the brush. Fully out to 60% of the radius: what moves
            // stays under the brush for the next dab, so the strand keeps its girth.
            const float w = std::clamp((R - length(p - centre)) / (0.4f * R), 0.0f, 1.0f) * (1.0f - held(i));
            p += dir * std::min(st * 2.0f, 1.0f) * w;
            break;
        }
        case SculptBrush::Nudge: p += along * (heading ? length(dir - N * dot(dir, N)) * st * f : 0.0f); break;
        case SculptBrush::Grab:
        case SculptBrush::Mask:
        case SculptBrush::Paint: break;
        }
    }
}

void SculptMesh::dab(const Vec3& centre, const SculptSettings& s, const Vec3& dir) {
    if (s.brush == SculptBrush::Grab || m_t.empty()) return;
    // A brush no smaller than 0.2% of the body: below that, dynamic topology
    // would refine without end.
    const float R = std::max({s.radius, m_size * 2e-3f, 1e-3f});
    const bool alpha = s.alpha && !s.alpha->empty();
    const float detail = std::max(s.detail > 0 ? s.detail : R * (alpha ? 0.035f : 0.25f), m_size * 2e-4f);
    const std::vector<Image> copies = images(centre, s, R);
    // A mask or paint needs no new triangles (paint as finely as the surface is divided).
    const bool topology = s.refine && s.brush != SculptBrush::Mask && s.brush != SculptBrush::Paint;
    if (topology) {
        for (const Image& c : copies) refineRegion(c.centre, alpha ? R * 1.4143f : R, detail);
        updateNormals();
    }
    for (const Image& c : copies) applyDab(c.centre, s, c.apply(dir));
    if (topology) // and coarsen where the clay was squashed (edges far below the detail size)
        for (const Image& c : copies) collapseLocal(trianglesNear(c.centre, R + detail * 0.3f), detail * 0.3f, c.centre, R, true);
    updateNormals();
}

void SculptMesh::beginGrab(const Vec3& centre, const SculptSettings& s) {
    m_grab.clear();
    if (m_record >= 0) m_start = m_p;
    const float R = std::max({s.radius, m_size * 2e-3f, 1e-3f});
    m_grabImages = images(centre, s, R);
    if (m_grabImages.size() > 255) m_grabImages.resize(255);
    if (s.refine) { // something to hold on to, even on a coarse block
        const float detail = std::max(s.detail > 0 ? s.detail : R * 0.25f, m_size * 2e-4f);
        for (const Image& c : m_grabImages) refineRegion(c.centre, R, detail);
        updateNormals();
    }
    const float st = std::clamp(s.strength * 2.0f, 0.0f, 1.0f); // strength 0.5 follows the mouse
    std::vector<char> taken(m_p.size(), 0);
    for (std::size_t c = 0; c < m_grabImages.size(); ++c)
        for (std::size_t i = 0; i < m_p.size(); ++i) {
            if (taken[i] || pinned(static_cast<std::uint32_t>(i))) continue;
            const float d = length(m_p[i] - m_grabImages[c].centre);
            if (d >= R) continue;
            taken[i] = 1;
            const float w = falloffOf(d, R, s.falloff) * st * (1.0f - held(static_cast<std::uint32_t>(i)));
            m_grab.push_back({static_cast<std::uint32_t>(i), w, m_p[i], static_cast<std::uint8_t>(c)});
        }
}

void SculptMesh::grabTo(const Vec3& delta) {
    if (m_grab.empty()) return;
    for (const GrabVert& g : m_grab) m_p[g.v] = g.start + m_grabImages[g.image].apply(delta) * g.w;
    updateNormals();
}

void SculptMesh::maskEdit(MaskOp op) {
    if (op == MaskOp::Clear) {
        m_mask.clear();
        return;
    }
    if (op == MaskOp::All) {
        m_mask.assign(m_p.size(), 1.0f);
        return;
    }
    if (m_mask.empty()) {
        if (op == MaskOp::Invert) m_mask.assign(m_p.size(), 1.0f);
        return; // nothing to smooth, grow or shrink
    }
    if (op == MaskOp::Invert) {
        for (float& m : m_mask) m = 1.0f - m;
        return;
    }
    // Over each vertex's neighbours: the average (smooth), the most (grow) or the least (shrink).
    std::vector<float> next = m_mask, sum(m_p.size(), 0.0f), cnt(m_p.size(), 0.0f);
    for (const auto& t : m_t)
        for (int e = 0; e < 3; ++e) {
            const std::uint32_t v = t[static_cast<std::size_t>(e)];
            for (int o = 1; o <= 2; ++o) {
                const float w = m_mask[t[static_cast<std::size_t>((e + o) % 3)]];
                if (op == MaskOp::Grow) next[v] = std::max(next[v], w);
                else if (op == MaskOp::Shrink) next[v] = std::min(next[v], w);
                sum[v] += w;
                cnt[v] += 1.0f;
            }
        }
    if (op == MaskOp::Smooth)
        for (std::size_t v = 0; v < next.size(); ++v)
            if (cnt[v] > 0) next[v] = 0.5f * m_mask[v] + 0.5f * sum[v] / cnt[v];
    m_mask = std::move(next);
}

bool SculptMesh::loopSubdivide(std::size_t maxTriangles) {
    if (m_t.empty() || m_t.size() * 4 > maxTriangles) return false;
    // Loop's scheme: a new point on each edge, 3/8 of its ends and 1/8 of the two
    // corners across it (the middle, on a border); each old point moves towards its
    // neighbours by Warren's weight (on a border: 3/4 itself, 1/8 each border neighbour).
    std::unordered_map<std::uint64_t, std::array<std::uint32_t, 3>> edges; // opposite corners, count
    edges.reserve(m_t.size() * 2);
    for (const auto& t : m_t)
        for (int e = 0; e < 3; ++e) {
            auto& rec = edges.try_emplace(edgeKey(t[static_cast<std::size_t>(e)], t[static_cast<std::size_t>((e + 1) % 3)]),
                                          std::array<std::uint32_t, 3>{0xFFFFFFFFu, 0xFFFFFFFFu, 0})
                            .first->second;
            const std::uint32_t opp = t[static_cast<std::size_t>((e + 2) % 3)];
            if (rec[2] < 2) rec[rec[2]] = opp;
            ++rec[2];
        }
    const std::size_t V = m_p.size();
    std::vector<Vec3> ring(V, Vec3{}), border(V, Vec3{});
    std::vector<int> valence(V, 0), borders(V, 0);
    for (const auto& [k, rec] : edges) {
        const auto a = static_cast<std::uint32_t>(k >> 32), b = static_cast<std::uint32_t>(k & 0xFFFFFFFFu);
        ring[a] += m_p[b];
        ring[b] += m_p[a];
        ++valence[a];
        ++valence[b];
        if (rec[2] != 2) { // a border (or a seam of more than two faces): kept as a crease
            border[a] += m_p[b];
            border[b] += m_p[a];
            ++borders[a];
            ++borders[b];
        }
    }
    std::vector<Vec3> moved(V);
    for (std::size_t v = 0; v < V; ++v) {
        if (borders[v] >= 2) {
            moved[v] = m_p[v] * 0.75f + border[v] * (0.25f / static_cast<float>(borders[v]));
        } else if (valence[v] > 0) {
            const float n = static_cast<float>(valence[v]);
            const float beta = valence[v] == 3 ? 3.0f / 16.0f : 3.0f / (8.0f * n);
            moved[v] = m_p[v] * (1.0f - n * beta) + ring[v] * beta;
        } else {
            moved[v] = m_p[v];
        }
    }
    std::unordered_map<std::uint64_t, std::uint32_t> mid;
    mid.reserve(edges.size());
    for (const auto& [k, rec] : edges) {
        const auto a = static_cast<std::uint32_t>(k >> 32), b = static_cast<std::uint32_t>(k & 0xFFFFFFFFu);
        Vec3 q = (m_p[a] + m_p[b]) * 0.5f;
        if (rec[2] == 2) q = (m_p[a] + m_p[b]) * 0.375f + (m_p[rec[0]] + m_p[rec[1]]) * 0.125f;
        mid.emplace(k, static_cast<std::uint32_t>(moved.size()));
        moved.push_back(q);
        addVertexExtras(a, b);
    }
    m_p = std::move(moved);
    std::vector<std::array<std::uint32_t, 3>> tris;
    tris.reserve(m_t.size() * 4);
    for (const auto& t : m_t) {
        const std::uint32_t ab = mid[edgeKey(t[0], t[1])], bc = mid[edgeKey(t[1], t[2])], ca = mid[edgeKey(t[2], t[0])];
        tris.push_back({t[0], ab, ca});
        tris.push_back({ab, t[1], bc});
        tris.push_back({ca, bc, t[2]});
        tris.push_back({ab, bc, ca});
    }
    if (m_uv.size() == m_t.size()) { // each quarter's corners: its triangle's, or half way
        std::vector<std::array<Vec2, 3>> uv;
        uv.reserve(m_uv.size() * 4);
        for (const auto& u : m_uv) {
            const Vec2 ab = uvMid(u[0], u[1]), bc = uvMid(u[1], u[2]), ca = uvMid(u[2], u[0]);
            uv.push_back({u[0], ab, ca});
            uv.push_back({ab, u[1], bc});
            uv.push_back({ca, bc, u[2]});
            uv.push_back({ab, bc, ca});
        }
        m_uv = std::move(uv);
    } else {
        m_uv.clear();
    }
    m_t = std::move(tris);
    if (!m_group.empty()) { // each quarter in its triangle's group
        std::vector<std::uint32_t> g;
        g.reserve(m_group.size() * 4);
        for (std::uint32_t x : m_group) g.insert(g.end(), 4, x);
        m_group = std::move(g);
    }
    updateNormals();
    return true;
}

bool SculptMesh::decimateTo(std::size_t target) {
    if (m_t.size() <= target) return true;
    double sum = 0;
    std::size_t n = 0;
    for (const auto& t : m_t)
        for (int e = 0; e < 3; ++e) {
            sum += length(m_p[t[static_cast<std::size_t>((e + 1) % 3)]] - m_p[t[static_cast<std::size_t>(e)]]);
            ++n;
        }
    // Collapse ever longer edges (shortest first) until few enough are left.
    float edge = static_cast<float>(sum / static_cast<double>(std::max<std::size_t>(n, 1)));
    for (int round = 0; round < 40 && m_t.size() > target; ++round) {
        const std::size_t before = m_t.size();
        collapseShort(edge);
        if (m_t.size() <= target) break;
        if (m_t.size() > before * 97 / 100) edge *= 1.3f;
    }
    return m_t.size() <= target * 11 / 10 + 8;
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
    std::vector<std::vector<Vec3>> outLayers(m_layers.size());
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
                out.vertices.push_back({m_p[v] - body.offset, n, held(static_cast<std::uint32_t>(v)), m_color.empty() ? 0u : m_color[v]});
                clusters.emplace_back(n, out_i);
                for (std::size_t li = 0; li < m_layers.size(); ++li) outLayers[li].push_back(m_layers[li].offset[v]);
                if (!m_ids.empty()) out.ids.push_back(m_ids[v]);
            }
            const auto& t = m_t[g];
            const int j = t[0] == v ? 0 : t[1] == v ? 1 : 2;
            out.indices[static_cast<std::size_t>(g) * 3 + static_cast<std::size_t>(j)] = out_i;
        }
    }
    if (featureEdges) out.computeFeatureEdges(40.0f, m_size * 0.006f, 75.0f); // fine sculpted detail: only its sharpest creases
    if (m_group.size() == m_t.size()) out.groups = m_group;
    if (m_uv.size() == m_t.size()) {
        out.uvs.resize(m_t.size() * 3);
        for (std::size_t i = 0; i < m_t.size(); ++i)
            for (std::size_t c = 0; c < 3; ++c) out.uvs[i * 3 + c] = m_uv[i][c];
    }
    out.layers = m_layers;
    for (std::size_t k = 0; k < m_layers.size(); ++k) out.layers[k].offset = std::move(outLayers[k]);
    out.recordLayer = m_record;
    body.mesh = std::move(out);
    body.kind = "mesh";
    body.params.clear();
}

void SculptMesh::stroke(const std::vector<Vec3>& points, const SculptSettings& s, const Vec3& grabDelta) {
    if (points.empty() || m_t.empty()) return;
    // The path runs between the points snapped to the surface. Snapping the dabs
    // of a path that floats off the surface would bunch them on the nearest bumps.
    const bool grab = s.brush == SculptBrush::Grab;
    Vec3 delta = grabDelta;
    if (grab && delta.x == 0 && delta.y == 0 && delta.z == 0 && points.size() >= 2) delta = points.back() - points.front();
    std::vector<Vec3> path;
    if (grab) {
        path.push_back(closestPoint(points.front()));
    } else if (s.brush == SculptBrush::SnakeHook) { // pulled out of the surface: only the start is on it
        path = points;
        path.front() = closestPoint(points.front());
    } else {
        path.reserve(points.size());
        if (points.size() > 8) { // many points: index the surface once
            const SurfaceIndex nearest(*this);
            for (const Vec3& p : points) path.push_back(nearest.closestPoint(p));
        } else {
            for (const Vec3& p : points) path.push_back(closestPoint(p));
        }
    }
    // A dab reaches its radius, and refines up to an edge beyond it: everything else
    // can be left out while the stroke runs, a great saving on a dense mesh. Between
    // the points the dabs follow the surface, which can bow out of the points' box.
    const float R = std::max({s.radius, m_size * 2e-3f, 1e-3f});
    float longest = 0.0f;
    for (std::size_t i = 1; i < path.size(); ++i) longest = std::max(longest, length(path[i] - path[i - 1]));
    const float margin = 2.0f * R + 0.25f * longest;
    // A box round the path, and round each of its mirrored and radial copies.
    std::vector<AABB> boxes;
    for (const Vec3& q : path) {
        const std::vector<Image> copies = images(q, s, 0.0f);
        if (boxes.size() < copies.size()) boxes.resize(copies.size());
        for (std::size_t c = 0; c < copies.size(); ++c) boxes[c].expand(copies[c].centre);
    }
    for (AABB& b : boxes) {
        b.min = b.min - Vec3{margin, margin, margin};
        b.max = b.max + Vec3{margin, margin, margin};
    }
    std::vector<std::uint32_t> taken;
    float longestCut = 0.0f;
    SculptMesh w = window(boxes, taken, &longestCut);
    // Not for most of the mesh (no saving), nor where the cut runs along edges longer
    // than the margin, which the brush would refine (a coarse CAD block). Cut edges
    // are never split, so the surface stays closed either way.
    if (taken.size() * 10 > m_t.size() * 6 || longestCut > margin) {
        runStroke(path, s, delta);
        return;
    }
    w.runStroke(path, s, delta);
    stitch(w, taken);
}

void SculptMesh::runStroke(const std::vector<Vec3>& path, const SculptSettings& s, const Vec3& grabDelta) {
    if (s.brush == SculptBrush::Grab) {
        beginGrab(path.front(), s);
        grabTo(grabDelta);
        return;
    }
    beginStroke();
    // At most ~1000 dabs a stroke, whatever the radius. Each dab knows the step from
    // the last one (the stroke's heading); a Snake Hook follows the path itself.
    const bool onSurface = s.brush != SculptBrush::SnakeHook;
    float total = 0.0f;
    for (std::size_t i = 1; i < path.size(); ++i) total += length(path[i] - path[i - 1]);
    const float spacing = std::max({s.radius * (s.spacing > 0 ? s.spacing : 0.25f), total / 1000.0f, 1e-3f});
    Vec3 last = closestPoint(path.front());
    dab(last, s);
    float carry = 0.0f;
    for (std::size_t i = 1; i < path.size(); ++i) {
        const Vec3 a = path[i - 1], b = path[i];
        const float len = length(b - a);
        float at = spacing - carry;
        while (at <= len) {
            const Vec3 q = a + (b - a) * (at / len);
            const Vec3 c = onSurface ? closestPoint(q) : q;
            dab(c, s, c - last);
            last = c;
            at += spacing;
        }
        carry = len - (at - spacing);
    }
}

SculptMesh SculptMesh::window(const std::vector<AABB>& boxes, std::vector<std::uint32_t>& taken, float* longestCut) const {
    SculptMesh w;
    w.m_size = m_size;
    w.m_mirrorX = m_mirrorX;
    w.m_centre = m_centre;
    w.m_base = m_base;
    w.m_record = m_record;
    for (const SculptLayer& l : m_layers) {
        w.m_layers.push_back(l);
        w.m_layers.back().offset.clear();
    }
    constexpr std::uint32_t kNone = 0xFFFFFFFFu;
    std::vector<std::uint32_t> toWindow(m_p.size(), kNone);
    std::vector<char> in(m_t.size(), 0);
    taken.clear();
    for (std::size_t i = 0; i < m_t.size(); ++i) {
        const auto& t = m_t[i];
        const Vec3 lo = vmin(m_p[t[0]], vmin(m_p[t[1]], m_p[t[2]])), hi = vmax(m_p[t[0]], vmax(m_p[t[1]], m_p[t[2]]));
        for (const AABB& b : boxes)
            if (lo.x <= b.max.x && hi.x >= b.min.x && lo.y <= b.max.y && hi.y >= b.min.y && lo.z <= b.max.z && hi.z >= b.min.z) {
                in[i] = 1;
                break;
            }
        if (!in[i]) continue;
        taken.push_back(static_cast<std::uint32_t>(i));
        std::array<std::uint32_t, 3> wt{};
        for (int c = 0; c < 3; ++c) {
            const std::uint32_t v = t[static_cast<std::size_t>(c)];
            if (toWindow[v] == kNone) {
                toWindow[v] = static_cast<std::uint32_t>(w.m_p.size());
                w.m_p.push_back(m_p[v]);
                w.m_n.push_back(m_n[v]);
                w.m_from.push_back(v);
                if (!m_mask.empty()) w.m_mask.push_back(m_mask[v]);
                if (!m_color.empty()) w.m_color.push_back(m_color[v]);
                if (!m_start.empty()) w.m_start.push_back(m_start[v]);
                if (!m_ids.empty()) w.m_ids.push_back(m_ids[v]);
                for (std::size_t k = 0; k < m_layers.size(); ++k) w.m_layers[k].offset.push_back(m_layers[k].offset[v]);
            }
            wt[static_cast<std::size_t>(c)] = toWindow[v];
        }
        w.m_t.push_back(wt);
        if (!m_group.empty()) w.m_group.push_back(m_group[i]);
        if (m_uv.size() == m_t.size()) w.m_uv.push_back(m_uv[i]);
    }
    w.m_pinned.assign(w.m_p.size(), 0);
    *longestCut = 0.0f;
    for (std::size_t i = 0; i < m_t.size(); ++i) {
        if (in[i]) continue;
        const auto& t = m_t[i];
        for (int c = 0; c < 3; ++c) {
            const std::uint32_t a = t[static_cast<std::size_t>(c)], b = t[static_cast<std::size_t>((c + 1) % 3)];
            if (toWindow[a] != kNone) w.m_pinned[toWindow[a]] = 1;
            if (toWindow[a] == kNone || toWindow[b] == kNone) continue;
            w.m_cut.insert(edgeKey(toWindow[a], toWindow[b]));
            *longestCut = std::max(*longestCut, length(m_p[a] - m_p[b]));
        }
    }
    return w;
}

void SculptMesh::stitch(const SculptMesh& w, const std::vector<std::uint32_t>& taken) {
    std::vector<std::uint32_t> toMesh(w.m_p.size());
    if (!w.m_mask.empty() && m_mask.empty()) m_mask.assign(m_p.size(), 0.0f); // the window painted the first mask
    if (!w.m_color.empty() && m_color.empty()) m_color.assign(m_p.size(), packColor(m_base)); // ... or the first paint
    for (std::size_t i = 0; i < w.m_p.size(); ++i) {
        if (w.m_from[i] != 0xFFFFFFFFu) {
            toMesh[i] = w.m_from[i];
            m_p[w.m_from[i]] = w.m_p[i];
            if (!m_mask.empty()) m_mask[w.m_from[i]] = w.m_mask[i];
            if (!m_color.empty()) m_color[w.m_from[i]] = w.m_color[i];
            if (!m_start.empty() && i < w.m_start.size()) m_start[w.m_from[i]] = w.m_start[i];
            if (!m_ids.empty() && i < w.m_ids.size()) m_ids[w.m_from[i]] = w.m_ids[i];
            for (std::size_t k = 0; k < m_layers.size() && k < w.m_layers.size(); ++k) m_layers[k].offset[w.m_from[i]] = w.m_layers[k].offset[i];
        } else {
            toMesh[i] = static_cast<std::uint32_t>(m_p.size());
            m_p.push_back(w.m_p[i]);
            if (!m_mask.empty()) m_mask.push_back(w.m_mask[i]);
            if (!m_color.empty()) m_color.push_back(w.m_color[i]);
            if (!m_start.empty()) m_start.push_back(i < w.m_start.size() ? w.m_start[i] : w.m_p[i]);
            if (!m_ids.empty()) m_ids.push_back(i < w.m_ids.size() ? w.m_ids[i] : 0xFFFFFFFFu);
            for (std::size_t k = 0; k < m_layers.size(); ++k)
                m_layers[k].offset.push_back(k < w.m_layers.size() && i < w.m_layers[k].offset.size() ? w.m_layers[k].offset[i] : Vec3{});
        }
    }
    std::vector<char> gone(m_t.size(), 0);
    for (const std::uint32_t i : taken) gone[i] = 1;
    std::size_t n = 0;
    for (std::size_t i = 0; i < m_t.size(); ++i)
        if (!gone[i]) {
            if (!m_group.empty()) m_group[n] = m_group[i];
            if (m_uv.size() == m_t.size()) m_uv[n] = m_uv[i];
            m_t[n++] = m_t[i];
        }
    const bool mapped = m_uv.size() == m_t.size() && w.m_uv.size() == w.m_t.size();
    if (mapped) m_uv.resize(n);
    else m_uv.clear();
    m_t.resize(n);
    if (!m_group.empty()) m_group.resize(n);
    for (std::size_t k = 0; k < w.m_t.size(); ++k) {
        const auto& t = w.m_t[k];
        m_t.push_back({toMesh[t[0]], toMesh[t[1]], toMesh[t[2]]});
        if (!m_group.empty()) m_group.push_back(k < w.m_group.size() ? w.m_group[k] : 0);
        if (mapped) m_uv.push_back(w.m_uv[k]);
    }
    dropUnused(); // corners the window collapsed away
    updateNormals();
}

void SculptMesh::commitLayers() {
    if (m_record < 0 || m_record >= static_cast<int>(m_layers.size()) || m_start.size() != m_p.size()) return;
    SculptLayer& l = m_layers[static_cast<std::size_t>(m_record)];
    const float k = 1.0f / std::max(l.strength, 0.01f); // what the stroke did, at the layer's full strength
    for (std::size_t v = 0; v < m_p.size(); ++v) l.offset[v] += (m_p[v] - m_start[v]) * k;
    m_start = m_p;
}

void sculptStroke(Body& body, const std::vector<Vec3>& points, const SculptSettings& s, const Vec3& grabDelta) {
    if (points.empty()) return;
    SculptMesh sm(body);
    sculptStroke(body, sm, points, s, grabDelta);
}

void sculptStroke(Body& body, SculptMesh& sm, const std::vector<Vec3>& points, const SculptSettings& s, const Vec3& grabDelta) {
    if (points.empty() || sm.empty()) return;
    sm.stroke(points, s, grabDelta);
    sm.commitLayers();
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

void maskBody(Body& body, MaskOp op) {
    SculptMesh sm(body);
    if (sm.empty()) return;
    sm.maskEdit(op);
    sm.writeTo(body, true);
}

float maskedFraction(const Body& body) {
    const auto& v = body.mesh.vertices;
    if (v.empty()) return 0.0f;
    double sum = 0;
    for (const Vertex& c : v) sum += c.mask;
    return static_cast<float>(sum / static_cast<double>(v.size()));
}

namespace {

// A parametric body painted or grouped becomes a mesh body: its mesh is saved as it is.
void keepMesh(Body& body) {
    if (body.kind == "mesh" || !isPrimitiveKind(body.kind)) return;
    body.kind = "mesh";
    body.params.clear();
}

// The mesh's corners welded by position: an id per vertex, and the edges' triangles.
struct Welded {
    std::vector<std::uint32_t> id;                               // per vertex
    std::unordered_map<std::uint64_t, std::vector<std::uint32_t>> edgeTris;
};
Welded weldMesh(const MeshData& m) {
    Welded w;
    std::map<std::tuple<float, float, float>, std::uint32_t> at;
    w.id.resize(m.vertices.size());
    for (std::size_t i = 0; i < m.vertices.size(); ++i) {
        const Vec3& p = m.vertices[i].position;
        w.id[i] = at.try_emplace({p.x, p.y, p.z}, static_cast<std::uint32_t>(at.size())).first->second;
    }
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3)
        for (int e = 0; e < 3; ++e) {
            const std::uint32_t a = w.id[m.indices[t + static_cast<std::size_t>(e)]], b = w.id[m.indices[t + static_cast<std::size_t>((e + 1) % 3)]];
            if (a != b) w.edgeTris[edgeKey(a, b)].push_back(static_cast<std::uint32_t>(t / 3));
        }
    return w;
}

struct NamedColour {
    const char* name;
    float r, g, b;
};
constexpr NamedColour kColours[] = {
    {"red", 0.85f, 0.15f, 0.12f},   {"green", 0.20f, 0.70f, 0.25f},  {"blue", 0.15f, 0.35f, 0.85f},   {"yellow", 0.95f, 0.85f, 0.15f},
    {"orange", 0.95f, 0.50f, 0.10f}, {"purple", 0.55f, 0.25f, 0.70f}, {"pink", 0.95f, 0.55f, 0.70f},   {"cyan", 0.15f, 0.80f, 0.85f},
    {"magenta", 0.85f, 0.20f, 0.70f}, {"white", 0.95f, 0.95f, 0.95f}, {"black", 0.05f, 0.05f, 0.05f},  {"grey", 0.50f, 0.50f, 0.50f},
    {"gray", 0.50f, 0.50f, 0.50f},   {"brown", 0.45f, 0.28f, 0.15f},  {"skin", 0.90f, 0.70f, 0.58f},   {"flesh", 0.90f, 0.70f, 0.58f},
    {"bone", 0.89f, 0.85f, 0.75f},   {"gold", 0.85f, 0.68f, 0.25f},   {"silver", 0.75f, 0.76f, 0.78f}, {"steel", 0.60f, 0.63f, 0.67f},
    {"copper", 0.72f, 0.45f, 0.20f}, {"clay", 0.70f, 0.48f, 0.38f}};

} // namespace

// --- alphas -----------------------------------------------------------------------------------

float SculptAlpha::sample(float u, float v, int tile) const noexcept {
    if (value.empty() || !(std::fabs(u) <= 1.0f) || !(std::fabs(v) <= 1.0f)) return 0.0f;
    const float n = static_cast<float>(std::max(tile, 1));
    float x = (u * 0.5f + 0.5f) * n, y = (0.5f - v * 0.5f) * n; // image: rows top to bottom
    x = (x - std::floor(x)) * static_cast<float>(width) - 0.5f, y = (y - std::floor(y)) * static_cast<float>(height) - 0.5f;
    const int x0 = static_cast<int>(std::floor(x)), y0 = static_cast<int>(std::floor(y));
    const float fx = x - static_cast<float>(x0), fy = y - static_cast<float>(y0);
    auto at = [&](int i, int j) {
        i = tile > 1 ? (i % width + width) % width : std::clamp(i, 0, width - 1);
        j = tile > 1 ? (j % height + height) % height : std::clamp(j, 0, height - 1);
        return value[static_cast<std::size_t>(j) * static_cast<std::size_t>(width) + static_cast<std::size_t>(i)];
    };
    return (at(x0, y0) * (1 - fx) + at(x0 + 1, y0) * fx) * (1 - fy) + (at(x0, y0 + 1) * (1 - fx) + at(x0 + 1, y0 + 1) * fx) * fy;
}

namespace {

// A smooth hash noise in [0, 1) at integer lattice points, and value noise between them.
float lattice(int x, int y, std::uint32_t seed) {
    std::uint32_t h = static_cast<std::uint32_t>(x) * 374761393u + static_cast<std::uint32_t>(y) * 668265263u + seed * 2246822519u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return static_cast<float>((h ^ (h >> 16)) & 0xFFFFFFu) / 16777216.0f;
}
float valueNoise(float x, float y, std::uint32_t seed) {
    const int x0 = static_cast<int>(std::floor(x)), y0 = static_cast<int>(std::floor(y));
    float fx = x - static_cast<float>(x0), fy = y - static_cast<float>(y0);
    fx = fx * fx * (3 - 2 * fx), fy = fy * fy * (3 - 2 * fy);
    const float a = lattice(x0, y0, seed), b = lattice(x0 + 1, y0, seed), c = lattice(x0, y0 + 1, seed), d = lattice(x0 + 1, y0 + 1, seed);
    return (a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + d * fx) * fy;
}
float smoothStep(float e0, float e1, float x) {
    const float t = std::clamp((x - e0) / (e1 - e0), 0.0f, 1.0f);
    return t * t * (3 - 2 * t);
}

// The built-in alphas, as functions of (x, y) in 0..1 across the square.
struct AlphaRecipe {
    const char* name;
    float (*f)(float x, float y);
};
constexpr float kTau = 6.28318530718f;
const AlphaRecipe kAlphas[] = {
    {"noise", [](float x, float y) { // four octaves of value noise
         float a = 0, amp = 0.5f, f = 6;
         for (int o = 0; o < 4; ++o, amp *= 0.5f, f *= 2) a += amp * valueNoise(x * f, y * f, 17u + static_cast<std::uint32_t>(o));
         return std::clamp((a - 0.2f) / 0.65f, 0.0f, 1.0f);
     }},
    {"ring", [](float x, float y) { // a crater's rim
         const float r = std::hypot(x - 0.5f, y - 0.5f) * 2;
         return std::exp(-std::pow((r - 0.68f) / 0.14f, 2.0f));
     }},
    {"square", [](float x, float y) { // a flat stamp with soft edges
         return smoothStep(0.0f, 0.06f, 0.42f - std::max(std::fabs(x - 0.5f), std::fabs(y - 0.5f)));
     }},
    {"dots", [](float x, float y) { // a grid of round bumps
         const float gx = x * 6 - std::floor(x * 6) - 0.5f, gy = y * 6 - std::floor(y * 6) - 0.5f;
         const float r = std::hypot(gx, gy) / 0.38f;
         return r < 1 ? std::sqrt(1 - r * r) : 0.0f;
     }},
    {"scales", [](float x, float y) { // overlapping fish scales, each rising to its rim
         const float n = 5;
         const float row = std::floor(y * n * 1.25f);
         float best = 0;
         for (int k = -1; k <= 1; ++k) {
             const float r0 = row + static_cast<float>(k);
             const float cy = (r0 + 0.5f) / (n * 1.25f);
             const float shift = std::fmod(r0, 2.0f) == 0 ? 0.0f : 0.5f / n;
             const float cx = (std::floor((x - shift) * n) + 0.5f) / n + shift;
             const float d = std::hypot((x - cx) * n, (y - cy) * n * 1.25f);
             if (d < 0.7f && y >= cy - 0.02f) best = std::max(best, 0.3f + 0.7f * d / 0.7f);
         }
         return best;
     }},
    {"crosshatch", [](float x, float y) { // two sets of fine diagonal grooves
         const float a = std::pow(std::fabs(std::sin((x + y) * kTau * 4)), 4.0f), b = std::pow(std::fabs(std::sin((x - y) * kTau * 4)), 4.0f);
         return std::max(a, b);
     }},
    {"bricks", [](float x, float y) { // running bond: the bricks raised, the mortar low
         const float rows = 7, cols = 3.5f;
         const float ry = y * rows, row = std::floor(ry);
         const float rx = x * cols + (std::fmod(row, 2.0f) == 0 ? 0.0f : 0.5f);
         const float fx = rx - std::floor(rx), fy = ry - row;
         return smoothStep(0.0f, 0.06f, std::min({fx, 1 - fx, (fy) * 0.5f / 0.5f, 1 - fy}) - 0.04f);
     }},
    {"pores", [](float x, float y) { // skin: small pits (cell noise)
         const float n = 22;
         const float px = x * n, py = y * n;
         const int cx = static_cast<int>(std::floor(px)), cy = static_cast<int>(std::floor(py));
         float d1 = 9;
         for (int j = -1; j <= 1; ++j)
             for (int i = -1; i <= 1; ++i) {
                 const float fx = static_cast<float>(cx + i) + lattice(cx + i, cy + j, 5u), fy = static_cast<float>(cy + j) + lattice(cx + i, cy + j, 9u);
                 d1 = std::min(d1, std::hypot(px - fx, py - fy));
             }
         return 1.0f - smoothStep(0.05f, 0.28f, d1);
     }},
    {"weave", [](float x, float y) { // over and under
         const float n = 6;
         const float u = x * n, v = y * n;
         const int iu = static_cast<int>(std::floor(u)), iv = static_cast<int>(std::floor(v));
         const float fu = u - static_cast<float>(iu), fv = v - static_cast<float>(iv);
         const bool across = ((iu + iv) & 1) == 0;
         const float t = across ? fv : fu, along = across ? fu : fv;
         return std::sin(t * 3.14159265f) * (0.6f + 0.4f * std::sin(along * 3.14159265f));
     }},
    {"stripes", [](float x, float y) { // parallel grooves
         (void)y;
         return 0.5f + 0.5f * std::cos(x * kTau * 8);
     }},
};

} // namespace

const std::vector<std::string>& sculptAlphaNames() {
    static const std::vector<std::string> names = [] {
        std::vector<std::string> n;
        for (const AlphaRecipe& r : kAlphas) n.push_back(r.name);
        return n;
    }();
    return names;
}

std::shared_ptr<const SculptAlpha> sculptAlpha(const std::string& nameOrPath, std::string* why) {
    const std::string key = lower(nameOrPath);
    for (const AlphaRecipe& r : kAlphas)
        if (key == r.name) {
            auto a = std::make_shared<SculptAlpha>();
            a->name = r.name;
            a->width = a->height = 256;
            a->value.resize(256 * 256);
            for (int j = 0; j < 256; ++j)
                for (int i = 0; i < 256; ++i)
                    a->value[static_cast<std::size_t>(j) * 256 + static_cast<std::size_t>(i)] =
                        std::clamp(r.f((static_cast<float>(i) + 0.5f) / 256.0f, 1.0f - (static_cast<float>(j) + 0.5f) / 256.0f), 0.0f, 1.0f);
            // Softened a little (a box blur 5 pixels wide, twice), so cliffs in the pattern
            // become steep slopes a sculpted surface can follow.
            std::vector<float> tmp(a->value.size());
            for (int pass = 0; pass < 2; ++pass) {
                for (int j = 0; j < 256; ++j)
                    for (int i = 0; i < 256; ++i) {
                        float sum = 0;
                        for (int k = -2; k <= 2; ++k) sum += a->value[static_cast<std::size_t>(j) * 256 + static_cast<std::size_t>(std::clamp(i + k, 0, 255))];
                        tmp[static_cast<std::size_t>(j) * 256 + static_cast<std::size_t>(i)] = sum / 5;
                    }
                for (int j = 0; j < 256; ++j)
                    for (int i = 0; i < 256; ++i) {
                        float sum = 0;
                        for (int k = -2; k <= 2; ++k) sum += tmp[static_cast<std::size_t>(std::clamp(j + k, 0, 255)) * 256 + static_cast<std::size_t>(i)];
                        a->value[static_cast<std::size_t>(j) * 256 + static_cast<std::size_t>(i)] = sum / 5;
                    }
            }
            // ... and stretched back to the full 0..1.
            const auto [mn, mx] = std::minmax_element(a->value.begin(), a->value.end());
            const float lo = *mn, span = std::max(*mx - *mn, 1e-6f);
            for (float& v : a->value) v = (v - lo) / span;
            return a;
        }
    ImageRef img;
    img.path = nameOrPath;
    std::string err;
    if (!loadImagePixels(img, &err) || img.pixelWidth <= 0 || img.pixelHeight <= 0) {
        if (why) *why = "No alpha named " + nameOrPath + " (SCULPT ALPHAS lists them) and no image there" + (err.empty() ? "" : ": " + err);
        return nullptr;
    }
    // Its brightness, at most 1024 across (larger images averaged down).
    const int step = std::max(1, std::max(img.pixelWidth, img.pixelHeight) / 1024);
    auto a = std::make_shared<SculptAlpha>();
    a->name = nameOrPath;
    a->width = img.pixelWidth / step, a->height = img.pixelHeight / step;
    a->value.resize(static_cast<std::size_t>(a->width) * static_cast<std::size_t>(a->height));
    for (int j = 0; j < a->height; ++j)
        for (int i = 0; i < a->width; ++i) {
            float sum = 0;
            for (int q = 0; q < step; ++q)
                for (int p = 0; p < step; ++p) {
                    const std::size_t k = (static_cast<std::size_t>(j * step + q) * static_cast<std::size_t>(img.pixelWidth) + static_cast<std::size_t>(i * step + p)) * 4;
                    const float lum = (0.2126f * img.pixels[k] + 0.7152f * img.pixels[k + 1] + 0.0722f * img.pixels[k + 2]) / 255.0f;
                    sum += lum * (static_cast<float>(img.pixels[k + 3]) / 255.0f);
                }
            a->value[static_cast<std::size_t>(j) * static_cast<std::size_t>(a->width) + static_cast<std::size_t>(i)] = sum / static_cast<float>(step * step);
        }
    return a;
}

bool parseColorText(const std::string& text, Vec3& rgb) {
    const std::string t = lower(text);
    for (const NamedColour& c : kColours)
        if (t == c.name) return rgb = {c.r, c.g, c.b}, true;
    if (t.size() == 7 && t[0] == '#') {
        unsigned v = 0;
        for (std::size_t i = 1; i < 7; ++i) {
            const char c = t[i];
            if (!std::isxdigit(static_cast<unsigned char>(c))) return false;
            v = v * 16 + static_cast<unsigned>(std::isdigit(static_cast<unsigned char>(c)) ? c - '0' : c - 'a' + 10);
        }
        rgb = {static_cast<float>((v >> 16) & 0xFF) / 255.0f, static_cast<float>((v >> 8) & 0xFF) / 255.0f, static_cast<float>(v & 0xFF) / 255.0f};
        return true;
    }
    float c[3];
    std::size_t at = 0;
    for (int k = 0; k < 3; ++k) {
        const std::size_t comma = t.find(',', at);
        if ((k < 2) != (comma != std::string::npos)) return false;
        const std::string part = t.substr(at, comma == std::string::npos ? std::string::npos : comma - at);
        char* end = nullptr;
        c[k] = std::strtof(part.c_str(), &end);
        if (part.empty() || !end || *end || !std::isfinite(c[k]) || c[k] < 0) return false;
        at = comma + 1;
    }
    const float scale = c[0] > 1 || c[1] > 1 || c[2] > 1 ? 1.0f / 255.0f : 1.0f;
    rgb = {std::min(c[0] * scale, 1.0f), std::min(c[1] * scale, 1.0f), std::min(c[2] * scale, 1.0f)};
    return true;
}

bool paintBody(Body& body, const Vec3& rgb, int group) {
    MeshData& m = body.mesh;
    if (m.indices.empty() || (group >= 0 && !m.hasGroups())) return false;
    std::vector<char> in(m.vertices.size(), static_cast<char>(group < 0 ? 1 : 0));
    if (group >= 0)
        for (std::size_t t = 0; t < m.groups.size(); ++t)
            if (m.groups[t] == static_cast<std::uint32_t>(group))
                for (int c = 0; c < 3; ++c) in[m.indices[t * 3 + static_cast<std::size_t>(c)]] = 1;
    if (std::none_of(in.begin(), in.end(), [](char x) { return x != 0; })) return false;
    keepMesh(body);
    for (std::size_t i = 0; i < m.vertices.size(); ++i) {
        Vertex& v = m.vertices[i];
        const float w = 1.0f - std::clamp(v.mask, 0.0f, 1.0f);
        if (!in[i] || w <= 0.0f) continue; // left as it was (unpainted stays so)
        const Vec3 old = painted(v.color) ? unpackColor(v.color) : body.color;
        v.color = packColor(old + (rgb - old) * w);
    }
    return true;
}

void clearPaint(Body& body) {
    for (Vertex& v : body.mesh.vertices) v.color = 0;
}

float paintedFraction(const Body& body) {
    const auto& v = body.mesh.vertices;
    if (v.empty()) return 0.0f;
    std::size_t n = 0;
    for (const Vertex& c : v) n += painted(c.color) ? 1 : 0;
    return static_cast<float>(n) / static_cast<float>(v.size());
}

int groupBody(Body& body, GroupBy how, float angleDeg) {
    MeshData& m = body.mesh;
    const std::size_t T = m.indices.size() / 3;
    if (T == 0) return 0;
    keepMesh(body);
    if (how == GroupBy::Mask) { // the masked faces: one new group
        std::vector<std::uint32_t> g = m.hasGroups() ? m.groups : std::vector<std::uint32_t>(T, 0);
        std::uint32_t top = 0;
        for (std::uint32_t x : g) top = std::max(top, x + 1);
        bool any = false;
        for (std::size_t t = 0; t < T; ++t) {
            float w = 0;
            for (int c = 0; c < 3; ++c) w += m.vertices[m.indices[t * 3 + static_cast<std::size_t>(c)]].mask;
            if (w >= 1.5f) g[t] = top, any = true; // most of the face masked
        }
        if (!any) return static_cast<int>(groupSizes(body).size());
        m.groups = std::move(g);
        return static_cast<int>(groupSizes(body).size());
    }
    const Welded w = weldMesh(m);
    std::vector<Vec3> fn(T);
    std::vector<std::uint32_t> col(T, 0);
    for (std::size_t t = 0; t < T; ++t) {
        const Vec3 &a = m.vertices[m.indices[t * 3]].position, &b = m.vertices[m.indices[t * 3 + 1]].position, &c = m.vertices[m.indices[t * 3 + 2]].position;
        fn[t] = normalize(cross(b - a, c - a));
        if (how == GroupBy::Paint) { // the face's colour: its corners' most common (quantised a little)
            std::uint32_t q[3];
            for (int k = 0; k < 3; ++k) {
                const std::uint32_t cc = m.vertices[m.indices[t * 3 + static_cast<std::size_t>(k)]].color;
                q[k] = painted(cc) ? (cc & 0xF0F0F0u) | 1u : 0u;
            }
            col[t] = q[1] == q[2] ? q[1] : q[0];
        }
    }
    const float cosLimit = std::cos(radians(std::clamp(angleDeg, 0.1f, 179.0f)));
    std::vector<std::uint32_t> g(T, 0xFFFFFFFFu);
    std::uint32_t next = 0;
    std::vector<std::uint32_t> stack;
    for (std::size_t s0 = 0; s0 < T; ++s0) {
        if (g[s0] != 0xFFFFFFFFu) continue;
        g[s0] = next;
        stack.assign(1, static_cast<std::uint32_t>(s0));
        while (!stack.empty()) {
            const std::uint32_t t = stack.back();
            stack.pop_back();
            for (int e = 0; e < 3; ++e) {
                const std::uint32_t a = w.id[m.indices[t * 3 + static_cast<std::size_t>(e)]], b = w.id[m.indices[t * 3 + static_cast<std::size_t>((e + 1) % 3)]];
                const auto it = w.edgeTris.find(edgeKey(a, b));
                if (it == w.edgeTris.end()) continue;
                for (std::uint32_t o : it->second) {
                    if (g[o] != 0xFFFFFFFFu) continue;
                    if (how == GroupBy::Normals && dot(fn[o], fn[t]) < cosLimit) continue;
                    if (how == GroupBy::Paint && col[o] != col[t]) continue;
                    g[o] = next;
                    stack.push_back(o);
                }
            }
        }
        ++next;
    }
    m.groups = std::move(g);
    return static_cast<int>(next);
}

void clearGroups(Body& body) { body.mesh.groups.clear(); }

int groupNear(const Body& body, const Vec3& q) {
    const MeshData& m = body.mesh;
    if (!m.hasGroups()) return -1;
    float best = 1e30f;
    int g = -1;
    for (std::size_t t = 0; t < m.groups.size(); ++t) {
        const Vec3 a = m.vertices[m.indices[t * 3]].position + body.offset, b = m.vertices[m.indices[t * 3 + 1]].position + body.offset,
                   c = m.vertices[m.indices[t * 3 + 2]].position + body.offset;
        const Vec3 p = closestOnTriangle(q, a, b, c);
        const float d = dot(p - q, p - q);
        if (d < best) best = d, g = static_cast<int>(m.groups[t]);
    }
    return g;
}

std::vector<std::pair<std::uint32_t, std::size_t>> groupSizes(const Body& body) {
    std::map<std::uint32_t, std::size_t> n;
    if (body.mesh.hasGroups())
        for (std::uint32_t g : body.mesh.groups) ++n[g];
    return {n.begin(), n.end()};
}

bool maskGroup(Body& body, std::uint32_t group, bool invert) {
    MeshData& m = body.mesh;
    if (!m.hasGroups()) return false;
    // A vertex is in the group when any face of the group uses its position (the
    // border ring belongs to both sides, as a welded mesh has it).
    const Welded w = weldMesh(m);
    std::vector<char> in(m.vertices.size(), 0);
    std::vector<char> idIn(w.id.empty() ? 0 : *std::max_element(w.id.begin(), w.id.end()) + 1, 0);
    bool any = false;
    for (std::size_t t = 0; t < m.groups.size(); ++t)
        if (m.groups[t] == group)
            for (int c = 0; c < 3; ++c) idIn[w.id[m.indices[t * 3 + static_cast<std::size_t>(c)]]] = 1, any = true;
    if (!any) return false;
    for (std::size_t i = 0; i < m.vertices.size(); ++i) m.vertices[i].mask = (idIn[w.id[i]] != 0) != invert ? 1.0f : 0.0f;
    keepMesh(body);
    return true;
}

namespace {

// The layer named (any case), or -1; layers that no longer fit the mesh are dropped first.
int layerIndex(Body& body, const std::string& name) {
    MeshData& m = body.mesh;
    if (!m.layers.empty() && !m.hasLayers()) m.layers.clear(), m.recordLayer = -1; // an edit that couldn't keep them
    for (std::size_t k = 0; k < m.layers.size(); ++k)
        if (lower(m.layers[k].name) == lower(name)) return static_cast<int>(k);
    return -1;
}

// Moves the surface by `factor` times a layer's offsets, normals made again.
void shiftByLayer(Body& body, std::size_t k, float factor) {
    MeshData& m = body.mesh;
    for (std::size_t v = 0; v < m.vertices.size(); ++v) m.vertices[v].position += m.layers[k].offset[v] * factor;
    SculptMesh sm(body);
    if (!sm.empty()) sm.writeTo(body, true);
}

bool failWith(std::string* why, const std::string& text) {
    if (why) *why = text;
    return false;
}

bool noLayer(const std::string& name, std::string* why) {
    if (why) *why = "No layer named " + name + " (LAYER LIST shows them)";
    return false;
}

} // namespace

bool layerAdd(Body& body, const std::string& name, bool record, std::string* why) {
    if (name.empty()) return failWith(why, "A layer needs a name");
    if (layerIndex(body, name) >= 0) return failWith(why, "There is already a layer named " + name);
    keepMesh(body);
    SculptLayer l;
    l.name = name;
    l.offset.assign(body.mesh.vertices.size(), Vec3{});
    body.mesh.layers.push_back(std::move(l));
    if (record) body.mesh.recordLayer = static_cast<int>(body.mesh.layers.size()) - 1;
    return true;
}

bool layerRecord(Body& body, const std::string& name, std::string* why) {
    if (name.empty()) {
        body.mesh.recordLayer = -1;
        return true;
    }
    const int k = layerIndex(body, name);
    if (k < 0) return noLayer(name, why);
    if (!(body.mesh.layers[static_cast<std::size_t>(k)].strength > 0.01f)) return failWith(why, name + " is off: turn it on to record into it");
    body.mesh.recordLayer = k;
    return true;
}

bool layerStrength(Body& body, const std::string& name, float strength, std::string* why) {
    const int k = layerIndex(body, name);
    if (k < 0) return noLayer(name, why);
    SculptLayer& l = body.mesh.layers[static_cast<std::size_t>(k)];
    if (!std::isfinite(strength)) return failWith(why, "The strength is a number (1: as sculpted, 0: off)");
    const float old = l.strength;
    if (strength > 0.0f) l.saved = strength;
    else if (old > 0.0f) l.saved = old;
    l.strength = strength;
    if (strength <= 0.01f && body.mesh.recordLayer == k) body.mesh.recordLayer = -1; // an off layer can't record
    shiftByLayer(body, static_cast<std::size_t>(k), strength - old);
    return true;
}

bool layerDelete(Body& body, const std::string& name, std::string* why) {
    const int k = layerIndex(body, name);
    if (k < 0) return noLayer(name, why);
    const float s = body.mesh.layers[static_cast<std::size_t>(k)].strength;
    shiftByLayer(body, static_cast<std::size_t>(k), -s);
    MeshData& m = body.mesh;
    m.layers.erase(m.layers.begin() + k);
    if (m.recordLayer == k) m.recordLayer = -1;
    else if (m.recordLayer > k) --m.recordLayer;
    return true;
}

bool layerBake(Body& body, const std::string& name, std::string* why) {
    MeshData& m = body.mesh;
    if (name.empty()) {
        m.layers.clear();
        m.recordLayer = -1;
        return true;
    }
    const int k = layerIndex(body, name);
    if (k < 0) return noLayer(name, why);
    m.layers.erase(m.layers.begin() + k);
    if (m.recordLayer == k) m.recordLayer = -1;
    else if (m.recordLayer > k) --m.recordLayer;
    return true;
}

bool subdivideBody(Body& body, int levels, std::size_t maxTriangles, std::string* error) {
    SculptMesh sm(body);
    if (sm.empty()) {
        if (error) *error = body.name + " has no surface";
        return false;
    }
    for (int i = 0; i < levels; ++i)
        if (!sm.loopSubdivide(maxTriangles)) {
            if (error) {
                char b[96];
                std::snprintf(b, sizeof(b), " again would pass %zu triangles", maxTriangles);
                *error = "Subdividing " + body.name + b;
            }
            return false;
        }
    sm.writeTo(body, true);
    return true;
}

bool decimateBody(Body& body, std::size_t targetTriangles, std::string* error) {
    SculptMesh sm(body);
    if (sm.empty() || targetTriangles < 4) {
        if (error) *error = sm.empty() ? body.name + " has no surface" : "Keep at least 4 triangles";
        return false;
    }
    sm.decimateTo(targetTriangles);
    sm.writeTo(body, true);
    return true;
}

bool symmetrizeBody(Body& body, int axis, bool positive, std::string* error) {
    // Cut on the plane through the body's origin, mirror the kept half onto the other
    // side and fuse the two (their cut faces lie back to back on the plane, and go).
    axis = std::clamp(axis, 0, 2);
    Vec3 n{};
    (axis == 0 ? n.x : axis == 1 ? n.y : n.z) = positive ? -1.0f : 1.0f; // the side the cut removes
    Body half = body;
    std::string err;
    if (!trimByPlane(half, body.offset, n, &err)) {
        if (error) *error = err.empty() ? "Nothing of " + body.name + " is on that side of its origin" : err;
        return false;
    }
    // In the body's own frame the plane is at 0: the cut face is snapped onto it and
    // the mirror is an exact negation, so the two cut faces lie exactly back to back.
    half.mesh.transform(translate(half.offset - body.offset));
    half.offset = body.offset;
    const AABB hb = half.mesh.bounds();
    const float snap = std::max(length(hb.max - hb.min), 1e-3f) * 1e-6f;
    auto coord = [axis](Vec3& v) -> float& { return axis == 0 ? v.x : axis == 1 ? v.y : v.z; };
    for (Vertex& v : half.mesh.vertices)
        if (std::fabs(coord(v.position)) <= snap) coord(v.position) = 0.0f;
    MeshData m = half.mesh;
    for (Vertex& v : m.vertices) {
        coord(v.position) = -coord(v.position);
        coord(v.normal) = -coord(v.normal);
    }
    for (std::size_t k = 0; k + 2 < m.indices.size(); k += 3) std::swap(m.indices[k + 1], m.indices[k + 2]);
    Body mirror = meshBody(body.name + " mirror", std::move(m));
    mirror.offset = body.offset;
    if (!bodyBoolean(half, mirror, BooleanOp::Union, &err)) {
        if (error) *error = err;
        return false;
    }
    body.mesh = std::move(half.mesh);
    body.offset = half.offset;
    body.kind = "mesh";
    body.params.clear();
    return true;
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
    id.reserve(m.vertices.size());
    std::vector<std::uint32_t> r(m.vertices.size());
    for (std::size_t i = 0; i < m.vertices.size(); ++i) {
        const Vec3& p = m.vertices[i].position;
        r[i] = id.try_emplace(Key{p.x + 0.0f, p.y + 0.0f, p.z + 0.0f}, static_cast<std::uint32_t>(id.size())).first->second;
    }
    // Edges by sorting their keys: an edge used other than twice is open (or worse).
    std::vector<std::uint64_t> keys;
    keys.reserve(m.indices.size());
    for (std::size_t k = 0; k + 2 < m.indices.size(); k += 3)
        for (std::size_t e = 0; e < 3; ++e) {
            const std::uint32_t a = r[m.indices[k + e]], b = r[m.indices[k + (e + 1) % 3]];
            if (a != b) keys.push_back(edgeKey(a, b));
        }
    std::sort(keys.begin(), keys.end());
    std::size_t open = 0;
    for (std::size_t i = 0; i < keys.size();) {
        std::size_t j = i + 1;
        while (j < keys.size() && keys[j] == keys[i]) ++j;
        open += j - i != 2;
        i = j;
    }
    return open;
}

bool watertight(const MeshData& m) {
    if (m.indices.size() < 12) return false;
    // Points at the same place are one; edges used other than twice are open.
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
    id.reserve(m.vertices.size());
    std::vector<std::uint32_t> r(m.vertices.size());
    for (std::size_t i = 0; i < m.vertices.size(); ++i) {
        const Vec3& p = m.vertices[i].position;
        r[i] = id.try_emplace(Key{p.x + 0.0f, p.y + 0.0f, p.z + 0.0f}, static_cast<std::uint32_t>(id.size())).first->second;
    }
    // The edges each used an even number of times (a closed shape; four where two solids
    // touch along an edge)? Those used oddly, else.
    auto oddEdges = [&](std::vector<std::uint64_t>* odd) {
        std::vector<std::uint64_t> keys;
        keys.reserve(m.indices.size());
        for (std::size_t k = 0; k + 2 < m.indices.size(); k += 3) {
            const std::uint32_t t[3] = {r[m.indices[k]], r[m.indices[k + 1]], r[m.indices[k + 2]]};
            if (t[0] == t[1] || t[1] == t[2] || t[2] == t[0]) continue; // collapsed: no surface
            for (std::size_t e = 0; e < 3; ++e) keys.push_back(edgeKey(t[e], t[(e + 1) % 3]));
        }
        std::sort(keys.begin(), keys.end());
        std::size_t n = 0;
        for (std::size_t i = 0; i < keys.size();) {
            std::size_t j = i + 1;
            while (j < keys.size() && keys[j] == keys[i]) ++j;
            if ((j - i) % 2) {
                ++n;
                if (odd) odd->push_back(keys[i]);
            }
            i = j;
        }
        return keys.empty() ? std::size_t{1} : n;
    };
    std::vector<std::uint64_t> odd;
    if (oddEdges(&odd) == 0) return true;
    if (odd.size() > 20000) return false; // an open sheet, not a seam
    // A seam: the points on those edges within a hair of one another become one.
    const AABB box = m.bounds();
    const double eps = 1e-5 * std::max(1e-3, static_cast<double>(length(box.max - box.min)));
    std::vector<char> loose(id.size(), 0);
    for (std::uint64_t k : odd) loose[static_cast<std::size_t>(k >> 32)] = loose[static_cast<std::size_t>(k & 0xffffffffu)] = 1;
    std::vector<Vec3> where(id.size());
    for (std::size_t i = 0; i < m.vertices.size(); ++i) where[r[i]] = m.vertices[i].position;
    std::map<std::tuple<long long, long long, long long>, std::uint32_t> cells;
    std::vector<std::uint32_t> to(id.size());
    for (std::uint32_t v = 0; v < to.size(); ++v) {
        to[v] = v;
        if (!loose[v]) continue;
        const Vec3& p = where[v];
        const long long ix = std::llround(p.x / eps), iy = std::llround(p.y / eps), iz = std::llround(p.z / eps);
        bool found = false;
        for (long long dx = -1; dx <= 1 && !found; ++dx)
            for (long long dy = -1; dy <= 1 && !found; ++dy)
                for (long long dz = -1; dz <= 1 && !found; ++dz)
                    if (const auto it = cells.find({ix + dx, iy + dy, iz + dz}); it != cells.end()) to[v] = it->second, found = true;
        if (!found) cells[{ix, iy, iz}] = v;
    }
    for (std::uint32_t& x : r) x = to[x];
    return oddEdges(nullptr) == 0;
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
// Separate pieces of a mesh (welded as a sculpt copy is: seams of round primitives join).
std::size_t shellCount(const MeshData& m) {
    Body b;
    b.mesh = m;
    const SculptMesh sm(b);
    const auto& tris = sm.triangles();
    std::vector<std::uint32_t> parent(sm.vertexCount());
    for (std::size_t i = 0; i < parent.size(); ++i) parent[i] = static_cast<std::uint32_t>(i);
    auto find = [&](std::uint32_t v) {
        while (parent[v] != v) v = parent[v] = parent[parent[v]];
        return v;
    };
    for (const auto& t : tris) {
        const std::uint32_t r = find(t[0]);
        parent[find(t[1])] = r;
        parent[find(t[2])] = r;
    }
    std::unordered_set<std::uint32_t> roots;
    for (const auto& t : tris) roots.insert(find(t[0]));
    return roots.size();
}

namespace {

// Welded, with the needles and caps (zero-area slivers) an earlier boolean left taken
// out: fed to the next boolean, they come back as holes. The nudged retries leave
// steps a hair high where faces were coplanar: edges under 2e-5 of the size go too.
MeshData cleanForBoolean(const MeshData& m) {
    if (m.indices.empty()) return m;
    Body b;
    b.mesh = m;
    SculptMesh sm(b);
    const AABB box = m.bounds();
    sm.collapseShort(length(box.max - box.min) * 2e-5f, {}, 0.0f, false); // a corner moves a hair at most
    sm.writeTo(b, true, 40.0f);
    return std::move(b.mesh);
}

} // namespace

bool robustBoolean(MeshData& aIn, const MeshData& toolIn, BooleanOp op) {
    MeshData a = cleanForBoolean(aIn);
    const MeshData tool = cleanForBoolean(toolIn);
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
    // Solids that only touch face to face (a block stood on another) can come back
    // as two shells with a wall inside them: their faces meet without crossing. Grown
    // by a hair, the tool overlaps the face it touches, and the union merges.
    if (op == BooleanOp::Union && bestOpen == 0) {
        const AABB ab = a.bounds();
        const bool touching = ab.min.x <= tb.max.x + nudge && tb.min.x <= ab.max.x + nudge && ab.min.y <= tb.max.y + nudge &&
                              tb.min.y <= ab.max.y + nudge && ab.min.z <= tb.max.z + nudge && tb.min.z <= ab.max.z + nudge;
        const std::size_t apart = shellCount(a) + shellCount(tool);
        if (touching && shellCount(best) >= apart) {
            MeshData r = a, t = tool;
            const Vec3 c = tb.center();
            const float grow = 1.0f + 2.0f * nudge / std::max(length(tb.max - tb.min), 1e-3f);
            t.transform(translate(c) * scale({grow, grow, grow}) * translate(-c));
            if (meshBoolean(r, t, op) && (!closedInput || openEdgeCount(r) == 0) && shellCount(r) < apart) best = std::move(r);
        }
    }
    aIn = cleanForBoolean(best);
    return true;
}

// Cuts and booleans run in the body's own frame (its mesh as stored, the tool moved
// into it): far from the origin, world floats would lose the detail.
void setLocalMesh(Body& body, MeshData m) {
    body.mesh = std::move(m);
    body.kind = "mesh";
    body.params.clear();
}

// The exact surfaces of the cutters below (world frame), so a cut body keeps exact geometry.
std::vector<BSurface> boxSurfaces(const Vec3& c, const Vec3& u, const Vec3& v, const Vec3& w, const Vec3& h) {
    std::vector<BSurface> out;
    const Vec3 axes[3] = {u, v, w};
    const float half[3] = {h.x, h.y, h.z};
    for (int k = 0; k < 3; ++k)
        for (int sgn = -1; sgn <= 1; sgn += 2) {
            BSurface s;
            s.kind = BSurfaceKind::Plane;
            const Vec3 n = axes[k] * static_cast<float>(sgn);
            s.f = Frame::make(DVec3(c + n * half[k]), DVec3(n), DVec3(axes[(k + 1) % 3]));
            out.push_back(s);
        }
    return out;
}

std::vector<BSurface> sphereSurfaces(const Vec3& c, float r) {
    BSurface s;
    s.kind = BSurfaceKind::Sphere;
    s.f = Frame::make(DVec3(c), {0, 0, 1}, {1, 0, 0});
    s.r1 = r;
    return {s};
}

std::vector<BSurface> cylinderSurfaces(const Vec3& from, const Vec3& to, float r) {
    const float h = length(to - from);
    return revolveSurfaces(from, to - from, {{0, 0}, {r, 0}, {r, h}, {0, h}});
}

bool applyCut(Body& body, const MeshData& worldTool, BooleanOp op, const char* what, std::string* error,
              const std::vector<BSurface>& toolSurfaces = {}) {
    std::vector<BSurface> surfaces = exactSurfaces(body); // before the mesh changes
    for (BSurface t : toolSurfaces) {
        t.f.o = t.f.o - DVec3(body.offset);
        surfaces.push_back(t);
    }
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
    keepExactGeometry(body, surfaces);
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
    return applyCut(body, revolve(at, n, prof, 48), BooleanOp::Subtract, "hole", error, revolveSurfaces(at, n, prof));
}

bool drillHole(Body& body, const Vec3& at, const Vec3& normal, const HoleSpec& hole, std::string* error) {
    const float widest = std::max({hole.diameter, hole.cboreDia, hole.csinkDia});
    const float margin = std::max(0.5f, widest * 0.25f);
    const float through = bodySize(body) * 2.0f + length(at - body.bounds().center());
    std::vector<std::pair<float, float>> prof;
    if (!holeProfile(hole, through, margin, prof, error)) return false;
    return applyCut(body, revolve(at, normalize(normal), prof, 48), BooleanOp::Subtract, "hole", error, revolveSurfaces(at, normalize(normal), prof));
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
    std::vector<BSurface> exact;
    switch (shape) {
    case PocketShape::Box: {
        const float h = (depth + margin) * 0.5f;
        tool = cutterBox(at + n * (margin - h), u, v, n, {width * 0.5f, length_ * 0.5f, h});
        exact = boxSurfaces(at + n * (margin - h), u, v, n, {width * 0.5f, length_ * 0.5f, h});
        break;
    }
    case PocketShape::Round:
        tool = cutterCylinder(at - n * depth, at + n * margin, width * 0.5f, 48);
        exact = cylinderSurfaces(at - n * depth, at + n * margin, width * 0.5f);
        break;
    case PocketShape::Ball: {
        const float r = width * 0.5f;
        tool = cutterSphere(at - n * ((depth > 0 ? depth : r) - r), r, 40, 20);
        exact = sphereSurfaces(at - n * ((depth > 0 ? depth : r) - r), r);
        break;
    }
    }
    return applyCut(body, tool, BooleanOp::Subtract, "pocket", error, exact);
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
    return applyCut(body, cutterBox(point + n * (L * 0.5f), u, v, n, {L, L, L * 0.5f}), BooleanOp::Subtract, "knife", error,
                    boxSurfaces(point + n * (L * 0.5f), u, v, n, {L, L, L * 0.5f}));
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
    std::vector<BSurface> exact;
    switch (shape) {
    case BossShape::Cylinder:
        tool = cutterCylinder(at - n * sink, at + n * height, diameter * 0.5f, 48);
        exact = cylinderSurfaces(at - n * sink, at + n * height, diameter * 0.5f);
        break;
    case BossShape::Box: {
        Vec3 u, v;
        frameOf(n, u, v);
        const float h = (height + sink) * 0.5f;
        tool = cutterBox(at + n * (height - h), u, v, n, {diameter * 0.5f, diameter * 0.5f, h});
        exact = boxSurfaces(at + n * (height - h), u, v, n, {diameter * 0.5f, diameter * 0.5f, h});
        break;
    }
    case BossShape::Ball: {
        const float r = diameter * 0.5f;
        // A dome of the given height (at most 90% of a ball, so it is sunk in and fuses).
        tool = cutterSphere(at + n * (std::min(height > 0 ? height : r, diameter * 0.9f) - r), r, 40, 20);
        exact = sphereSurfaces(at + n * (std::min(height > 0 ? height : r, diameter * 0.9f) - r), r);
        break;
    }
    }
    return applyCut(body, tool, BooleanOp::Union, "boss", error, exact);
}

bool meshBooleanRobust(MeshData& a, const MeshData& b, BooleanOp op) { return robustBoolean(a, b, op); }

bool bodyBoolean(Body& a, const Body& b, BooleanOp op, std::string* error) { return bodyBoolean(a, b, op, error, {}); }

bool bodyBoolean(Body& a, const Body& b, BooleanOp op, std::string* error, const std::vector<BSurface>& toolSurfaces) {
    MeshData m = a.mesh; // in a's frame (see applyCut)
    MeshData other = b.mesh;
    const Vec3 shift = b.offset - a.offset;
    std::vector<BSurface> surfaces = exactSurfaces(a);
    std::vector<BSurface> fromB = exactSurfaces(b);
    fromB.insert(fromB.end(), toolSurfaces.begin(), toolSurfaces.end());
    for (BSurface s : fromB) {
        s.f.o = s.f.o + DVec3(shift);
        surfaces.push_back(s);
    }
    if (shift.x != 0 || shift.y != 0 || shift.z != 0) other.transform(translate(shift));
    if (!robustBoolean(m, other, op)) {
        if (error)
            *error = op == BooleanOp::Subtract ? b.name + " does not overlap " + a.name + " or removes all of it"
                                               : "The bodies do not overlap";
        return false;
    }
    setLocalMesh(a, std::move(m));
    keepExactGeometry(a, surfaces);
    return true;
}

SculptSettings withPressure(const SculptSettings& s, float pressure, bool size, bool strength) noexcept {
    SculptSettings out = s;
    if (pressure < 0.0f) return out;
    const float p = std::clamp(pressure, 0.05f, 1.0f);
    if (size) out.radius = s.radius * p;
    if (strength) out.strength = s.strength * p;
    return out;
}

} // namespace sicdm

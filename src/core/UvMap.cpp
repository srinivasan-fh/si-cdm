#include "core/UvMap.hpp"

#include "core/Bvh.hpp"
#include "core/Scene.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <cmath>
#include <thread>
#include <unordered_map>

namespace sicdm {

namespace {

constexpr double kPiD = 3.14159265358979323846;

std::uint64_t pairKey(std::uint32_t a, std::uint32_t b) {
    if (a > b) std::swap(a, b);
    return (static_cast<std::uint64_t>(a) << 32) | b;
}

// The mesh with corners at the same place made one vertex (meshes split for flat
// shading are still one surface for the charts).
struct Welded {
    std::vector<Vec3> p;
    std::vector<std::array<std::uint32_t, 3>> t; // one per mesh triangle, in order
};

Welded weld(const MeshData& m) {
    Welded w;
    const AABB b = m.bounds();
    const double q = std::max(static_cast<double>(length(b.max - b.min)), 1e-6) * 1e-6;
    struct Hash {
        std::size_t operator()(const std::array<long long, 3>& k) const noexcept {
            return static_cast<std::size_t>(k[0] * 73856093LL ^ k[1] * 19349663LL ^ k[2] * 83492791LL);
        }
    };
    std::unordered_map<std::array<long long, 3>, std::uint32_t, Hash> at;
    std::vector<std::uint32_t> id(m.vertices.size());
    for (std::size_t i = 0; i < m.vertices.size(); ++i) {
        const Vec3& p = m.vertices[i].position;
        const std::array<long long, 3> k{std::llround(p.x / q), std::llround(p.y / q), std::llround(p.z / q)};
        const auto [it, fresh] = at.emplace(k, static_cast<std::uint32_t>(w.p.size()));
        if (fresh) w.p.push_back(p);
        id[i] = it->second;
    }
    for (std::size_t k = 0; k + 2 < m.indices.size(); k += 3) w.t.push_back({id[m.indices[k]], id[m.indices[k + 1]], id[m.indices[k + 2]]});
    return w;
}

// Calls f(x, y, b0, b1, b2) at every pixel centre inside the triangle (pixel space).
template <typename F>
void raster(const double* x, const double* y, int size, F&& f) {
    const double area = (x[1] - x[0]) * (y[2] - y[0]) - (x[2] - x[0]) * (y[1] - y[0]);
    if (std::fabs(area) < 1e-12) return;
    const int x0 = std::max(0, static_cast<int>(std::floor(std::min({x[0], x[1], x[2]}))));
    const int x1 = std::min(size - 1, static_cast<int>(std::ceil(std::max({x[0], x[1], x[2]}))));
    const int y0 = std::max(0, static_cast<int>(std::floor(std::min({y[0], y[1], y[2]}))));
    const int y1 = std::min(size - 1, static_cast<int>(std::ceil(std::max({y[0], y[1], y[2]}))));
    for (int py = y0; py <= y1; ++py)
        for (int px = x0; px <= x1; ++px) {
            const double sx = px + 0.5, sy = py + 0.5;
            const double b0 = ((x[1] - sx) * (y[2] - sy) - (x[2] - sx) * (y[1] - sy)) / area;
            const double b1 = ((x[2] - sx) * (y[0] - sy) - (x[0] - sx) * (y[2] - sy)) / area;
            const double b2 = 1.0 - b0 - b1;
            constexpr double e = -1e-9;
            if (b0 >= e && b1 >= e && b2 >= e) f(px, py, b0, b1, b2);
        }
}

// Least-squares conformal map of one chart (local vertex numbers): `uv` (u, v per
// vertex) comes in as the starting guess, with the two pinned vertices' places, and goes
// out flattened. Conjugate gradients on the normal equations.
void conformalMap(const std::vector<Vec3>& p, const std::vector<std::array<std::uint32_t, 3>>& tris, std::vector<double>& uv, std::uint32_t pinA,
                  std::uint32_t pinB) {
    struct Row {
        std::array<std::uint32_t, 6> var;
        std::array<double, 6> c;
    };
    std::vector<Row> rows;
    rows.reserve(tris.size() * 2);
    for (const auto& t : tris) {
        const Vec3 a = p[t[0]], b = p[t[1]], c = p[t[2]];
        const Vec3 n = cross(b - a, c - a);
        const double area2 = static_cast<double>(length(n));
        if (area2 < 1e-20) continue;
        const Vec3 ex = normalize(b - a), ey = cross(normalize(n), ex);
        // The triangle in its own plane, as complex numbers z_j.
        const double zr[3] = {0.0, static_cast<double>(length(b - a)), static_cast<double>(dot(c - a, ex))};
        const double zi[3] = {0.0, 0.0, static_cast<double>(dot(c - a, ey))};
        const double s = 1.0 / std::sqrt(0.5 * area2);
        Row re{}, im{};
        for (int j = 0; j < 3; ++j) {
            const int k = (j + 2) % 3, l = (j + 1) % 3; // W_j = z_{j+2} - z_{j+1}
            const double wr = (zr[k] - zr[l]) * s, wi = (zi[k] - zi[l]) * s;
            const std::uint32_t u = 2 * t[static_cast<std::size_t>(j)], v = u + 1;
            re.var[static_cast<std::size_t>(2 * j)] = u, re.c[static_cast<std::size_t>(2 * j)] = wr;
            re.var[static_cast<std::size_t>(2 * j + 1)] = v, re.c[static_cast<std::size_t>(2 * j + 1)] = -wi;
            im.var[static_cast<std::size_t>(2 * j)] = u, im.c[static_cast<std::size_t>(2 * j)] = wi;
            im.var[static_cast<std::size_t>(2 * j + 1)] = v, im.c[static_cast<std::size_t>(2 * j + 1)] = wr;
        }
        rows.push_back(re), rows.push_back(im);
    }
    const std::size_t n = uv.size();
    std::vector<char> pinned(n, 0);
    pinned[2 * pinA] = pinned[2 * pinA + 1] = pinned[2 * pinB] = pinned[2 * pinB + 1] = 1;
    std::vector<double> r(rows.size());
    auto normal = [&](const std::vector<double>& x, std::vector<double>& y) { // y = A^T A x, pinned ones 0
        for (std::size_t i = 0; i < rows.size(); ++i) {
            double s = 0;
            for (int j = 0; j < 6; ++j) s += rows[i].c[static_cast<std::size_t>(j)] * x[rows[i].var[static_cast<std::size_t>(j)]];
            r[i] = s;
        }
        std::fill(y.begin(), y.end(), 0.0);
        for (std::size_t i = 0; i < rows.size(); ++i)
            for (int j = 0; j < 6; ++j) y[rows[i].var[static_cast<std::size_t>(j)]] += rows[i].c[static_cast<std::size_t>(j)] * r[i];
        for (std::size_t k = 0; k < n; ++k)
            if (pinned[k]) y[k] = 0;
    };
    std::vector<double> res(n), d(n), Ad(n);
    normal(uv, res);
    for (double& v : res) v = -v;
    d = res;
    double rr = 0;
    for (double v : res) rr += v * v;
    const double rr0 = rr;
    const int most = static_cast<int>(std::min<std::size_t>(4000, 200 + n));
    for (int it = 0; it < most && rr > 1e-24 * rr0 && rr > 1e-30; ++it) {
        normal(d, Ad);
        double dAd = 0;
        for (std::size_t k = 0; k < n; ++k) dAd += d[k] * Ad[k];
        if (!(dAd > 0)) break;
        const double alpha = rr / dAd;
        double rn = 0;
        for (std::size_t k = 0; k < n; ++k) {
            uv[k] += alpha * d[k];
            res[k] -= alpha * Ad[k];
            rn += res[k] * res[k];
        }
        const double beta = rn / rr;
        rr = rn;
        for (std::size_t k = 0; k < n; ++k) d[k] = res[k] + beta * d[k];
    }
}

struct Pt {
    double x, y;
};

double cross2(const Pt& o, const Pt& a, const Pt& b) { return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x); }

std::vector<Pt> hullOf(std::vector<Pt> pts) {
    std::sort(pts.begin(), pts.end(), [](const Pt& a, const Pt& b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
    if (pts.size() < 3) return pts;
    std::vector<Pt> h(2 * pts.size());
    std::size_t k = 0;
    for (const Pt& p : pts) {
        while (k >= 2 && cross2(h[k - 2], h[k - 1], p) <= 0) --k;
        h[k++] = p;
    }
    for (std::size_t i = pts.size() - 1, lower = k + 1; i-- > 0;) {
        while (k >= lower && cross2(h[k - 2], h[k - 1], pts[i]) <= 0) --k;
        h[k++] = pts[i];
    }
    h.resize(k - 1);
    return h;
}

Vec3 faceNormal(const Vec3& a, const Vec3& b, const Vec3& c) { return normalize(cross(b - a, c - a)); }

} // namespace

bool unwrapUv(MeshData& mesh, const UvOptions& opt, UvReport* report, std::string* why) {
    const std::size_t F = mesh.indices.size() / 3;
    if (F == 0) {
        if (why) *why = "There are no faces to unwrap";
        return false;
    }
    const Welded w = weld(mesh);
    std::vector<Vec3> fn(F);
    std::vector<double> fa(F);
    for (std::size_t f = 0; f < F; ++f) {
        const auto& t = w.t[f];
        const Vec3 c = cross(w.p[t[1]] - w.p[t[0]], w.p[t[2]] - w.p[t[0]]);
        fa[f] = 0.5 * static_cast<double>(length(c));
        fn[f] = normalize(c);
    }
    // Neighbours across each edge (only where exactly two faces meet).
    std::unordered_map<std::uint64_t, std::array<std::int64_t, 3>> byEdge; // two faces and a count
    for (std::size_t f = 0; f < F; ++f)
        for (int e = 0; e < 3; ++e) {
            const auto& t = w.t[f];
            const std::uint32_t a = t[static_cast<std::size_t>(e)], b = t[static_cast<std::size_t>((e + 1) % 3)];
            if (a == b) continue;
            auto [it, fresh] = byEdge.emplace(pairKey(a, b), std::array<std::int64_t, 3>{-1, -1, 0});
            auto& v = it->second;
            if (v[2] < 2) v[static_cast<std::size_t>(v[2])] = static_cast<std::int64_t>(f);
            ++v[2];
        }
    std::vector<std::array<std::int64_t, 3>> nb(F, {-1, -1, -1});
    for (std::size_t f = 0; f < F; ++f)
        for (int e = 0; e < 3; ++e) {
            const auto& t = w.t[f];
            const auto it = byEdge.find(pairKey(t[static_cast<std::size_t>(e)], t[static_cast<std::size_t>((e + 1) % 3)]));
            if (it == byEdge.end() || it->second[2] != 2) continue;
            nb[f][static_cast<std::size_t>(e)] = it->second[0] == static_cast<std::int64_t>(f) ? it->second[1] : it->second[0];
        }
    // 1. Charts grown from the largest faces: faces within `angle` of the seed's direction,
    // not across a fold of more than `angle`, nor across a polygroup's border. A quad's two
    // triangles go together.
    const bool grouped = opt.groups && mesh.hasGroups(), quads = mesh.hasQuads();
    const float cosA = std::cos(std::clamp(opt.angle, 5.0f, 89.0f) * static_cast<float>(kPiD) / 180.0f);
    std::vector<std::int64_t> chart(F, -1);
    std::vector<std::size_t> order(F);
    for (std::size_t f = 0; f < F; ++f) order[f] = f;
    std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) { return fa[a] > fa[b]; });
    std::vector<Vec3> chartN;
    std::int64_t charts = 0;
    for (const std::size_t seed : order) {
        if (chart[seed] >= 0 || !(fa[seed] > 0)) continue;
        const std::int64_t id = charts++;
        const Vec3 N = fn[seed];
        chartN.push_back(N);
        std::vector<std::size_t> queue{seed};
        chart[seed] = id;
        auto fits = [&](std::size_t f, std::size_t from) {
            return chart[f] < 0 && fa[f] > 0 && dot(fn[f], N) > cosA && dot(fn[f], fn[from]) > cosA && (!grouped || mesh.groups[f] == mesh.groups[seed]);
        };
        // A quad's triangles join together or not at all.
        auto mateOf = [&](std::size_t f) { return quads && (f ^ 1u) < F ? f ^ 1u : f; };
        if (const std::size_t m = mateOf(seed); m != seed && fits(m, seed)) chart[m] = id, queue.push_back(m);
        for (std::size_t q = 0; q < queue.size(); ++q) {
            const std::size_t f = queue[q];
            for (int e = 0; e < 3; ++e) {
                const std::int64_t gi = nb[f][static_cast<std::size_t>(e)];
                if (gi < 0) continue;
                const std::size_t g = static_cast<std::size_t>(gi), m = mateOf(g);
                if (!fits(g, f) || (m != g && chart[m] < 0 && !fits(m, g))) continue;
                chart[g] = id, queue.push_back(g);
                if (m != g && chart[m] < 0) chart[m] = id, queue.push_back(m);
            }
        }
    }
    // Ragged borders smoothed: a face (with its quad mate) mostly surrounded by another
    // chart joins it, when it nearly fits that chart's direction (and is in its polygroup).
    const float cosLoose = std::cos(std::min(std::clamp(opt.angle, 5.0f, 89.0f) * 1.25f, 85.0f) * static_cast<float>(kPiD) / 180.0f);
    for (int pass = 0; pass < 6; ++pass) {
        bool moved = false;
        for (std::size_t f = 0; f < F; ++f) {
            if (chart[f] < 0 || (quads && (f & 1u) && chart[f ^ 1u] == chart[f])) continue; // a whole quad: from its first triangle
            std::size_t unit[2] = {f, f};
            const std::size_t count = quads && (f ^ 1u) < F && chart[f ^ 1u] == chart[f] ? 2 : 1;
            if (count == 2) unit[1] = f ^ 1u;
            std::int64_t around[6];
            int n = 0;
            for (std::size_t u = 0; u < count; ++u)
                for (int e = 0; e < 3; ++e) {
                    const std::int64_t g = nb[unit[u]][static_cast<std::size_t>(e)];
                    if (g < 0 || static_cast<std::size_t>(g) == unit[0] || static_cast<std::size_t>(g) == unit[1]) continue;
                    around[n++] = chart[static_cast<std::size_t>(g)];
                }
            for (int i = 0; i < n; ++i) {
                const std::int64_t c = around[i];
                if (c < 0 || c == chart[f]) continue;
                int votes = 0, own = 0;
                for (int j = 0; j < n; ++j) votes += around[j] == c ? 1 : 0, own += around[j] == chart[f] ? 1 : 0;
                if (2 * votes <= n || votes <= own) continue;
                bool fit = true;
                for (std::size_t u = 0; u < count; ++u)
                    fit = fit && dot(fn[unit[u]], chartN[static_cast<std::size_t>(c)]) > cosLoose && (!grouped || mesh.groups[unit[u]] == mesh.groups[f]);
                if (grouped && fit) { // the chart's own group
                    for (std::size_t g = 0; g < F; ++g)
                        if (chart[g] == c) {
                            fit = mesh.groups[g] == mesh.groups[f];
                            break;
                        }
                }
                if (!fit) continue;
                for (std::size_t u = 0; u < count; ++u) chart[unit[u]] = c;
                moved = true;
                break;
            }
        }
        if (!moved) break;
    }
    // Slivers (a chart of a few faces or almost no area) go into the neighbour they share
    // the most edges with: their distortion can't show.
    {
        double total = 0;
        for (std::size_t f = 0; f < F; ++f) total += fa[f];
        for (int pass = 0; pass < 3; ++pass) {
            std::vector<std::vector<std::size_t>> members(static_cast<std::size_t>(charts));
            std::vector<double> area(static_cast<std::size_t>(charts), 0.0);
            for (std::size_t f = 0; f < F; ++f)
                if (chart[f] >= 0) members[static_cast<std::size_t>(chart[f])].push_back(f), area[static_cast<std::size_t>(chart[f])] += fa[f];
            bool merged = false;
            for (std::size_t c = 0; c < members.size(); ++c) {
                const bool sliver = area[c] < 2e-4 * total || (members[c].size() <= 4 && area[c] < 2e-3 * total);
                if (members[c].empty() || !sliver) continue;
                std::unordered_map<std::int64_t, int> shared;
                for (const std::size_t f : members[c])
                    for (int e = 0; e < 3; ++e) {
                        const std::int64_t g = nb[f][static_cast<std::size_t>(e)];
                        if (g >= 0 && chart[static_cast<std::size_t>(g)] >= 0 && chart[static_cast<std::size_t>(g)] != static_cast<std::int64_t>(c) &&
                            (!grouped || mesh.groups[static_cast<std::size_t>(g)] == mesh.groups[f]))
                            ++shared[chart[static_cast<std::size_t>(g)]];
                    }
                std::int64_t to = -1;
                int most = 0;
                for (const auto& [d, n] : shared)
                    if (n > most || (n == most && d < to)) most = n, to = d;
                if (to < 0) continue;
                for (const std::size_t f : members[c]) chart[f] = to;
                members[static_cast<std::size_t>(to)].insert(members[static_cast<std::size_t>(to)].end(), members[c].begin(), members[c].end());
                area[static_cast<std::size_t>(to)] += area[c];
                members[c].clear();
                merged = true;
            }
            if (!merged) break;
        }
    }
    // Charts left empty by that are dropped (numbers made consecutive again).
    {
        std::vector<std::int64_t> renumber(static_cast<std::size_t>(charts), -1);
        std::vector<Vec3> keptN;
        for (std::size_t f = 0; f < F; ++f)
            if (chart[f] >= 0 && renumber[static_cast<std::size_t>(chart[f])] < 0) {
                renumber[static_cast<std::size_t>(chart[f])] = static_cast<std::int64_t>(keptN.size());
                keptN.push_back(chartN[static_cast<std::size_t>(chart[f])]);
            }
        for (std::size_t f = 0; f < F; ++f)
            if (chart[f] >= 0) chart[f] = renumber[static_cast<std::size_t>(chart[f])];
        chartN = std::move(keptN);
        charts = static_cast<std::int64_t>(chartN.size());
    }
    // Faces with no area: a neighbour's chart, else one of their own.
    for (bool changed = true; changed;) {
        changed = false;
        for (std::size_t f = 0; f < F; ++f) {
            if (chart[f] >= 0) continue;
            for (int e = 0; e < 3 && chart[f] < 0; ++e) {
                const std::int64_t g = nb[f][static_cast<std::size_t>(e)];
                if (g >= 0 && chart[static_cast<std::size_t>(g)] >= 0) chart[f] = chart[static_cast<std::size_t>(g)], changed = true;
            }
        }
    }
    for (std::size_t f = 0; f < F; ++f)
        if (chart[f] < 0) chart[f] = charts++, chartN.push_back(Vec3{0, 0, 1});
    // 2. Each chart flattened.
    std::vector<std::vector<std::size_t>> faces(static_cast<std::size_t>(charts));
    for (std::size_t f = 0; f < F; ++f) faces[static_cast<std::size_t>(chart[f])].push_back(f);
    std::vector<std::uint32_t> cornerLocal(F * 3);
    std::vector<std::vector<Pt>> chartUv(static_cast<std::size_t>(charts));
    std::vector<std::uint32_t> local(w.p.size(), 0xFFFFFFFFu);
    std::size_t projected = 0;
    double stretchSum = 0, areaSum = 0;
    for (std::size_t c = 0; c < faces.size(); ++c) {
        std::vector<Vec3> p;
        std::vector<std::uint32_t> global;
        std::vector<std::array<std::uint32_t, 3>> tris;
        for (const std::size_t f : faces[c]) {
            std::array<std::uint32_t, 3> lt{};
            for (int j = 0; j < 3; ++j) {
                const std::uint32_t g = w.t[f][static_cast<std::size_t>(j)];
                if (local[g] == 0xFFFFFFFFu) local[g] = static_cast<std::uint32_t>(p.size()), p.push_back(w.p[g]), global.push_back(g);
                lt[static_cast<std::size_t>(j)] = local[g];
                cornerLocal[f * 3 + static_cast<std::size_t>(j)] = local[g];
            }
            tris.push_back(lt);
        }
        for (const std::uint32_t g : global) local[g] = 0xFFFFFFFFu;
        // The chart's mean direction; the projection on its plane starts the solve.
        Vec3 N{};
        for (const std::size_t f : faces[c]) N += fn[f] * static_cast<float>(fa[f]);
        N = length(N) > 0 ? normalize(N) : chartN[c];
        const Vec3 ref = std::fabs(N.x) < 0.9f ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
        const Vec3 e1 = normalize(cross(N, ref)), e2 = cross(N, e1);
        std::vector<double> flat(p.size() * 2);
        for (std::size_t v = 0; v < p.size(); ++v) flat[2 * v] = static_cast<double>(dot(p[v], e1)), flat[2 * v + 1] = static_cast<double>(dot(p[v], e2));
        std::vector<double> uv = flat;
        if (tris.size() > 1) {
            // Pinned: the two ends of the projection's longer extent.
            double minX = 1e300, maxX = -1e300, minY = 1e300, maxY = -1e300;
            for (std::size_t v = 0; v < p.size(); ++v)
                minX = std::min(minX, flat[2 * v]), maxX = std::max(maxX, flat[2 * v]), minY = std::min(minY, flat[2 * v + 1]), maxY = std::max(maxY, flat[2 * v + 1]);
            const std::size_t axis = maxX - minX >= maxY - minY ? 0 : 1;
            std::uint32_t lo = 0, hi = 0;
            for (std::uint32_t v = 0; v < p.size(); ++v) {
                if (flat[2 * v + axis] < flat[2 * lo + axis]) lo = v;
                if (flat[2 * v + axis] > flat[2 * hi + axis]) hi = v;
            }
            if (lo != hi) conformalMap(p, tris, uv, lo, hi);
            // Folded (or failed): the projection, which can't fold within the chart's angle.
            // Faces of next to no area (slivers) don't count.
            double meanArea2 = 0;
            for (const auto& t : tris) meanArea2 += static_cast<double>(length(cross(p[t[1]] - p[t[0]], p[t[2]] - p[t[0]])));
            meanArea2 /= static_cast<double>(tris.size());
            bool folded = false;
            for (const auto& t : tris) {
                const Vec3 n3 = cross(p[t[1]] - p[t[0]], p[t[2]] - p[t[0]]);
                if (!(static_cast<double>(length(n3)) > 0.01 * meanArea2)) continue;
                const double a = (uv[2 * t[1]] - uv[2 * t[0]]) * (uv[2 * t[2] + 1] - uv[2 * t[0] + 1]) - (uv[2 * t[2]] - uv[2 * t[0]]) * (uv[2 * t[1] + 1] - uv[2 * t[0] + 1]);
                if (!(a > 0)) folded = true;
            }
            if (folded) uv = flat, ++projected;
        }
        // The chart at the surface's own scale (every chart alike: even texel density).
        double a3 = 0, a2 = 0;
        for (const auto& t : tris) {
            a3 += 0.5 * static_cast<double>(length(cross(p[t[1]] - p[t[0]], p[t[2]] - p[t[0]])));
            a2 += 0.5 * std::fabs((uv[2 * t[1]] - uv[2 * t[0]]) * (uv[2 * t[2] + 1] - uv[2 * t[0] + 1]) - (uv[2 * t[2]] - uv[2 * t[0]]) * (uv[2 * t[1] + 1] - uv[2 * t[0] + 1]));
        }
        const double k = a2 > 0 && a3 > 0 ? std::sqrt(a3 / a2) : 1.0;
        for (double& x : uv) x *= k;
        // Angle distortion: the singular values of each face's map.
        for (const auto& t : tris) {
            const Vec3 a = p[t[0]], b = p[t[1]], cc = p[t[2]];
            const Vec3 n3 = cross(b - a, cc - a);
            const double A = 0.5 * static_cast<double>(length(n3));
            if (!(A > 0)) continue;
            const Vec3 ex = normalize(b - a), ey = cross(normalize(n3), ex);
            const double x1 = static_cast<double>(length(b - a)), x2 = static_cast<double>(dot(cc - a, ex)), y2 = static_cast<double>(dot(cc - a, ey));
            const double u1 = uv[2 * t[1]] - uv[2 * t[0]], v1 = uv[2 * t[1] + 1] - uv[2 * t[0] + 1];
            const double u2 = uv[2 * t[2]] - uv[2 * t[0]], v2 = uv[2 * t[2] + 1] - uv[2 * t[0] + 1];
            // J * [x1 x2; 0 y2] = [u1 u2; v1 v2]
            const double j00 = u1 / x1, j10 = v1 / x1, j01 = (u2 - j00 * x2) / y2, j11 = (v2 - j10 * x2) / y2;
            const double E = 0.5 * (j00 + j11), Fh = 0.5 * (j00 - j11), G = 0.5 * (j10 + j01), H = 0.5 * (j10 - j01);
            const double Q = std::sqrt(E * E + H * H), R = std::sqrt(Fh * Fh + G * G);
            const double s1 = Q + R, s2 = std::fabs(Q - R);
            stretchSum += A * (s2 > 1e-12 ? std::min(s1 / s2 - 1.0, 10.0) : 10.0);
            areaSum += A;
        }
        chartUv[c].resize(p.size());
        for (std::size_t v = 0; v < p.size(); ++v) chartUv[c][v] = {uv[2 * v], uv[2 * v + 1]};
    }
    // 3. Packed: each chart first turned to its smallest bounding box, lying flat.
    struct Box {
        double w = 0, h = 0;
    };
    std::vector<Box> box(chartUv.size());
    double boxArea = 0;
    for (std::size_t c = 0; c < chartUv.size(); ++c) {
        auto& pts = chartUv[c];
        const std::vector<Pt> h = hullOf(pts);
        double bestA = 1e300, ca = 1, sa = 0;
        for (std::size_t i = 0; i < h.size(); ++i) {
            const Pt& a = h[i];
            const Pt& b = h[(i + 1) % h.size()];
            const double len = std::hypot(b.x - a.x, b.y - a.y);
            if (!(len > 0)) continue;
            const double cx = (b.x - a.x) / len, sx = (b.y - a.y) / len;
            double x0 = 1e300, x1 = -1e300, y0 = 1e300, y1 = -1e300;
            for (const Pt& q : h) {
                const double rx = q.x * cx + q.y * sx, ry = -q.x * sx + q.y * cx;
                x0 = std::min(x0, rx), x1 = std::max(x1, rx), y0 = std::min(y0, ry), y1 = std::max(y1, ry);
            }
            if ((x1 - x0) * (y1 - y0) < bestA) bestA = (x1 - x0) * (y1 - y0), ca = cx, sa = sx;
        }
        double x0 = 1e300, x1 = -1e300, y0 = 1e300, y1 = -1e300;
        for (Pt& q : pts) {
            q = {q.x * ca + q.y * sa, -q.x * sa + q.y * ca};
            x0 = std::min(x0, q.x), x1 = std::max(x1, q.x), y0 = std::min(y0, q.y), y1 = std::max(y1, q.y);
        }
        if (pts.empty()) x0 = x1 = y0 = y1 = 0;
        const bool turn = y1 - y0 > x1 - x0;
        for (Pt& q : pts) q = turn ? Pt{y1 - q.y, q.x - x0} : Pt{q.x - x0, q.y - y0};
        box[c].w = turn ? y1 - y0 : x1 - x0;
        box[c].h = turn ? x1 - x0 : y1 - y0;
        boxArea += box[c].w * box[c].h;
    }
    // Skyline packing on a grid: charts (largest first) dropped where they come to rest
    // lowest on those already placed, in whichever of four turns rests lowest. Their
    // outlines (not their boxes) meet, so round charts nest. A few widths are tried.
    double widest = 0;
    for (const Box& bx : box) widest = std::max(widest, bx.w);
    std::vector<std::size_t> bySize(box.size());
    for (std::size_t c = 0; c < box.size(); ++c) bySize[c] = c;
    std::stable_sort(bySize.begin(), bySize.end(), [&](std::size_t x, std::size_t y) { return box[x].w * box[x].h > box[y].w * box[y].h; });
    auto turned = [&](const Pt& q, const Box& bx, int k) {
        switch (k) {
        case 1: return Pt{bx.h - q.y, q.x};
        case 2: return Pt{bx.w - q.x, bx.h - q.y};
        case 3: return Pt{q.y, bx.w - q.x};
        default: return q;
        }
    };
    struct Profile {
        int w = 0, h = 0;
        std::vector<int> bottom, top; // per column, -1: empty
    };
    constexpr int G = 256;
    const double side0 = std::sqrt(std::max(boxArea, 1e-12));
    auto profileOf = [&](std::size_t c, int k, double cell, int dil) {
        const Box& bx = box[c];
        const double W = k % 2 ? bx.h : bx.w, H = k % 2 ? bx.w : bx.h;
        Profile pr;
        pr.w = static_cast<int>(std::ceil(W / cell)) + 1 + 2 * dil;
        pr.h = static_cast<int>(std::ceil(H / cell)) + 1 + 2 * dil;
        std::vector<int> lo(static_cast<std::size_t>(pr.w), -1), hi(static_cast<std::size_t>(pr.w), -1);
        for (const std::size_t f : faces[c]) {
            Pt q[3];
            for (int j = 0; j < 3; ++j) {
                const Pt t = turned(chartUv[c][cornerLocal[f * 3 + static_cast<std::size_t>(j)]], bx, k);
                q[j] = {t.x / cell + dil, t.y / cell + dil};
            }
            const int x0 = std::max(0, static_cast<int>(std::floor(std::min({q[0].x, q[1].x, q[2].x}))) - 1);
            const int x1 = std::min(pr.w - 1, static_cast<int>(std::ceil(std::max({q[0].x, q[1].x, q[2].x}))));
            const int y0 = std::max(0, static_cast<int>(std::floor(std::min({q[0].y, q[1].y, q[2].y}))) - 1);
            const int y1 = std::min(pr.h - 1, static_cast<int>(std::ceil(std::max({q[0].y, q[1].y, q[2].y}))));
            for (int x = x0; x <= x1; ++x)
                for (int y = y0; y <= y1; ++y) {
                    // The cell is touched when its centre is within half a diagonal of the triangle.
                    const Pt m{x + 0.5, y + 0.5};
                    const double s0 = cross2(q[0], q[1], m), s1 = cross2(q[1], q[2], m), s2 = cross2(q[2], q[0], m);
                    bool in = (s0 >= 0 && s1 >= 0 && s2 >= 0) || (s0 <= 0 && s1 <= 0 && s2 <= 0);
                    for (int e = 0; e < 3 && !in; ++e) {
                        const Pt& a = q[e];
                        const Pt& bb = q[(e + 1) % 3];
                        const double dx = bb.x - a.x, dy = bb.y - a.y, l2 = dx * dx + dy * dy;
                        const double t = l2 > 0 ? std::clamp(((m.x - a.x) * dx + (m.y - a.y) * dy) / l2, 0.0, 1.0) : 0.0;
                        const double ex = a.x + dx * t - m.x, ey = a.y + dy * t - m.y;
                        in = ex * ex + ey * ey <= 0.5;
                    }
                    if (!in) continue;
                    int& l = lo[static_cast<std::size_t>(x)];
                    int& u = hi[static_cast<std::size_t>(x)];
                    l = l < 0 ? y : std::min(l, y), u = std::max(u, y);
                }
        }
        // Grown by `dil` cells every way: the gap to the next chart.
        pr.bottom.assign(static_cast<std::size_t>(pr.w), -1), pr.top.assign(static_cast<std::size_t>(pr.w), -1);
        for (int x = 0; x < pr.w; ++x)
            for (int d = -dil; d <= dil; ++d) {
                const int sx = x + d;
                if (sx < 0 || sx >= pr.w || lo[static_cast<std::size_t>(sx)] < 0) continue;
                const int l = std::max(0, lo[static_cast<std::size_t>(sx)] - dil), u = std::min(pr.h - 1, hi[static_cast<std::size_t>(sx)] + dil);
                int& b0 = pr.bottom[static_cast<std::size_t>(x)];
                b0 = b0 < 0 ? l : std::min(b0, l);
                pr.top[static_cast<std::size_t>(x)] = std::max(pr.top[static_cast<std::size_t>(x)], u);
            }
        return pr;
    };
    struct Place {
        int x = 0, y = 0, k = 0;
    };
    double bestSide = 1e300, bestCell = 1;
    int bestDil = 0;
    std::vector<Place> best;
    for (double f = 0.6; f < 1.36; f += 0.05) {
        const double width = std::max(side0 * f, widest * 1.02);
        const double cell = width / G;
        const int dil = std::max(1, static_cast<int>(std::ceil(std::clamp(static_cast<double>(opt.margin), 0.0, 0.1) * G * 0.5)));
        const int cols = G + 2 * dil + 2;
        std::vector<int> sky(static_cast<std::size_t>(cols), 0);
        std::vector<Place> place(box.size());
        int height = 0;
        for (const std::size_t c : bySize) {
            int bestScore = 1 << 30, bestY = 0;
            Place pick;
            Profile chosen;
            for (int k = 0; k < 4; ++k) {
                if (k > 0 && box[c].w <= 0) break;
                Profile pr = profileOf(c, k, cell, dil);
                if (pr.w > cols) continue;
                int tallest = 0;
                for (int t : pr.top) tallest = std::max(tallest, t);
                for (int x = 0; x + pr.w <= cols; ++x) {
                    int y = 0;
                    for (int j = 0; j < pr.w; ++j)
                        if (pr.bottom[static_cast<std::size_t>(j)] >= 0) y = std::max(y, sky[static_cast<std::size_t>(x + j)] - pr.bottom[static_cast<std::size_t>(j)]);
                    const int score = y + tallest;
                    if (score < bestScore || (score == bestScore && y < bestY)) bestScore = score, bestY = y, pick = {x, y, k}, chosen = pr;
                }
            }
            if (bestScore == (1 << 30)) { // wider than the atlas (can't happen: the widest fits); put it on top
                pick = {0, height, 0};
                chosen = profileOf(c, 0, cell, dil);
            }
            for (int j = 0; j < chosen.w && pick.x + j < cols; ++j)
                if (chosen.top[static_cast<std::size_t>(j)] >= 0)
                    sky[static_cast<std::size_t>(pick.x + j)] = std::max(sky[static_cast<std::size_t>(pick.x + j)], pick.y + chosen.top[static_cast<std::size_t>(j)] + 1);
            for (int v : sky) height = std::max(height, v);
            place[c] = pick;
        }
        int right = 0;
        for (int x = 0; x < cols; ++x)
            if (sky[static_cast<std::size_t>(x)] > 0) right = x + 1;
        const double side = std::max(right, height) * cell;
        if (side < bestSide) bestSide = side, bestCell = cell, bestDil = dil, best = std::move(place);
    }
    // The charts turned and moved to their places.
    for (std::size_t c = 0; c < chartUv.size(); ++c)
        for (Pt& q : chartUv[c]) {
            const Pt t = turned(q, box[c], best[c].k);
            q = {t.x + (best[c].x + bestDil) * bestCell, t.y + (best[c].y + bestDil) * bestCell};
        }
    const double inv = bestSide > 0 && bestSide < 1e300 ? 1.0 / bestSide : 1.0;
    mesh.uvs.assign(F * 3, Vec2{});
    double used = 0;
    for (std::size_t f = 0; f < F; ++f) {
        const std::size_t c = static_cast<std::size_t>(chart[f]);
        Vec2 q[3];
        for (int j = 0; j < 3; ++j) {
            const Pt& pt = chartUv[c][cornerLocal[f * 3 + static_cast<std::size_t>(j)]];
            q[j] = {static_cast<float>(pt.x * inv), static_cast<float>(pt.y * inv)};
            mesh.uvs[f * 3 + static_cast<std::size_t>(j)] = q[j];
        }
        used += 0.5 * std::fabs(static_cast<double>((q[1].x - q[0].x) * (q[2].y - q[0].y) - (q[2].x - q[0].x) * (q[1].y - q[0].y)));
    }
    if (report) {
        report->charts = static_cast<std::size_t>(charts);
        report->projected = projected;
        report->fill = static_cast<float>(used);
        report->stretch = areaSum > 0 ? static_cast<float>(stretchSum / areaSum) : 0.0f;
    }
    return true;
}

std::vector<std::uint8_t> uvLayoutImage(const MeshData& mesh, int size) {
    size = std::clamp(size, 16, 8192);
    const std::size_t S = static_cast<std::size_t>(size);
    std::vector<std::uint8_t> img(S * S * 3, 0);
    for (std::size_t i = 0; i < S * S; ++i) img[i * 3] = 34, img[i * 3 + 1] = 36, img[i * 3 + 2] = 42;
    if (!mesh.hasUvs()) return img;
    auto plot = [&](int x, int y, std::uint8_t r, std::uint8_t g, std::uint8_t b) {
        if (x < 0 || y < 0 || x >= size || y >= size) return;
        const std::size_t k = (static_cast<std::size_t>(y) * S + static_cast<std::size_t>(x)) * 3;
        img[k] = r, img[k + 1] = g, img[k + 2] = b;
    };
    for (std::size_t f = 0; f * 3 + 2 < mesh.indices.size(); ++f) {
        const Vec3 n = faceNormal(mesh.vertices[mesh.indices[f * 3]].position, mesh.vertices[mesh.indices[f * 3 + 1]].position,
                                  mesh.vertices[mesh.indices[f * 3 + 2]].position);
        const auto r = static_cast<std::uint8_t>(90 + 120 * std::fabs(n.x)), g = static_cast<std::uint8_t>(90 + 120 * std::fabs(n.y)),
                   b = static_cast<std::uint8_t>(90 + 120 * std::fabs(n.z));
        double x[3], y[3];
        for (int j = 0; j < 3; ++j) {
            const Vec2 u = mesh.uvs[f * 3 + static_cast<std::size_t>(j)];
            x[j] = static_cast<double>(u.x) * size, y[j] = (1.0 - static_cast<double>(u.y)) * size;
        }
        raster(x, y, size, [&](int px, int py, double, double, double) { plot(px, py, r, g, b); });
    }
    for (std::size_t f = 0; f * 3 + 2 < mesh.indices.size(); ++f)
        for (int j = 0; j < 3; ++j) {
            const Vec2 a = mesh.uvs[f * 3 + static_cast<std::size_t>(j)], b = mesh.uvs[f * 3 + static_cast<std::size_t>((j + 1) % 3)];
            const double ax = static_cast<double>(a.x) * size, ay = (1.0 - static_cast<double>(a.y)) * size;
            const double bx = static_cast<double>(b.x) * size, by = (1.0 - static_cast<double>(b.y)) * size;
            const int steps = static_cast<int>(std::ceil(std::max(std::fabs(bx - ax), std::fabs(by - ay)))) + 1;
            for (int s = 0; s <= steps; ++s) {
                const double t = static_cast<double>(s) / steps;
                plot(static_cast<int>(ax + (bx - ax) * t), static_cast<int>(ay + (by - ay) * t), 20, 22, 28);
            }
        }
    return img;
}

bool parseBakeMap(const std::string& word, BakeMap& out) {
    std::string w;
    for (char c : word) w += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (w == "normal" || w == "normals") out = BakeMap::Normal;
    else if (w == "objectnormal" || w == "object") out = BakeMap::ObjectNormal;
    else if (w == "height" || w == "displacement") out = BakeMap::Height;
    else if (w == "occlusion" || w == "ao") out = BakeMap::Occlusion;
    else if (w == "color" || w == "colour") out = BakeMap::Color;
    else return false;
    return true;
}

const char* bakeMapName(BakeMap map) {
    switch (map) {
    case BakeMap::Normal: return "normal";
    case BakeMap::ObjectNormal: return "object normal";
    case BakeMap::Height: return "height";
    case BakeMap::Occlusion: return "occlusion";
    case BakeMap::Color: return "colour";
    }
    return "?";
}

bool bakeMap(const Body& target, const Body& source, const BakeOptions& opt, std::vector<std::uint8_t>& pixels, int& channels, int& bits,
             BakeReport* report, std::string* why) {
    const MeshData& low = target.mesh;
    if (!low.hasUvs()) {
        if (why) *why = target.name + " has no texture coordinates (UV UNWRAP it first)";
        return false;
    }
    if (source.mesh.indices.empty()) {
        if (why) *why = source.name + " has no surface";
        return false;
    }
    const int size = std::clamp(opt.size, 16, 8192);
    const std::size_t S = static_cast<std::size_t>(size);
    // The source as placed, with its ray index.
    MeshData src = source.mesh;
    for (Vertex& v : src.vertices) v.position += source.offset;
    const MeshBvh bvh(src);
    const AABB sb = src.bounds();
    const float diag = std::max(length(sb.max - sb.min), 1e-3f);
    const float cage = opt.map != BakeMap::Occlusion && opt.distance > 0 ? opt.distance : 0.03f * diag;
    const float reach = opt.map == BakeMap::Occlusion && opt.distance > 0 ? opt.distance : 0.2f * diag;
    // Which face (and where on it) each texel is.
    const std::size_t F = low.indices.size() / 3;
    std::vector<std::int32_t> faceOf(S * S, -1);
    std::vector<std::array<float, 2>> at(S * S);
    for (std::size_t f = 0; f < F; ++f) {
        double x[3], y[3];
        for (int j = 0; j < 3; ++j) {
            const Vec2 u = low.uvs[f * 3 + static_cast<std::size_t>(j)];
            x[j] = static_cast<double>(u.x) * size, y[j] = (1.0 - static_cast<double>(u.y)) * size;
        }
        raster(x, y, size, [&](int px, int py, double, double b1, double b2) {
            const std::size_t k = static_cast<std::size_t>(py) * S + static_cast<std::size_t>(px);
            if (faceOf[k] >= 0) return;
            faceOf[k] = static_cast<std::int32_t>(f);
            at[k] = {static_cast<float>(b1), static_cast<float>(b2)};
        });
    }
    // Each face's tangent frame (along u, along v) from its texture coordinates.
    std::vector<Vec3> tanU(F), tanV(F);
    for (std::size_t f = 0; f < F; ++f) {
        const Vec3 p0 = low.vertices[low.indices[f * 3]].position, p1 = low.vertices[low.indices[f * 3 + 1]].position,
                   p2 = low.vertices[low.indices[f * 3 + 2]].position;
        const Vec2 u0 = low.uvs[f * 3], u1 = low.uvs[f * 3 + 1], u2 = low.uvs[f * 3 + 2];
        const Vec3 e1 = p1 - p0, e2 = p2 - p0;
        const float du1 = u1.x - u0.x, dv1 = u1.y - u0.y, du2 = u2.x - u0.x, dv2 = u2.y - u0.y;
        const float r = du1 * dv2 - du2 * dv1;
        const Vec3 n = faceNormal(p0, p1, p2);
        if (std::fabs(r) < 1e-20f) {
            const Vec3 t = normalize(cross(n, std::fabs(n.x) < 0.9f ? Vec3{1, 0, 0} : Vec3{0, 1, 0}));
            tanU[f] = t, tanV[f] = cross(n, t);
            continue;
        }
        tanU[f] = (e1 * dv2 - e2 * dv1) / r;
        tanV[f] = (e2 * du1 - e1 * du2) / r;
    }
    std::vector<Vec3> value(S * S, Vec3{});
    std::vector<char> filled(S * S, 0);
    std::atomic<std::size_t> missed{0}, texels{0};
    std::atomic<int> nextRow{0};
    auto hitInfo = [&](std::uint32_t tri, const Vec3& q, Vec3& n, Vec3& colour) {
        const Vertex& a = src.vertices[src.indices[tri * 3]];
        const Vertex& b = src.vertices[src.indices[tri * 3 + 1]];
        const Vertex& c = src.vertices[src.indices[tri * 3 + 2]];
        const Vec3 nn = cross(b.position - a.position, c.position - a.position);
        const float A = dot(nn, nn);
        float w1 = 0, w2 = 0;
        if (A > 0) {
            w1 = dot(cross(q - a.position, c.position - a.position), nn) / A;
            w2 = dot(cross(b.position - a.position, q - a.position), nn) / A;
        }
        const float w0 = 1.0f - w1 - w2;
        n = normalize(a.normal * w0 + b.normal * w1 + c.normal * w2);
        if (!(length(n) > 0)) n = normalize(nn);
        auto col = [&](const Vertex& v) { return painted(v.color) ? unpackColor(v.color) : source.color; };
        colour = col(a) * w0 + col(b) * w1 + col(c) * w2;
    };
    auto work = [&] {
        for (int py = nextRow++; py < size; py = nextRow++) {
            for (int px = 0; px < size; ++px) {
                const std::size_t k = static_cast<std::size_t>(py) * S + static_cast<std::size_t>(px);
                const std::int32_t f = faceOf[k];
                if (f < 0) continue;
                const std::size_t fi = static_cast<std::size_t>(f);
                const float b1 = at[k][0], b2 = at[k][1], b0 = 1.0f - b1 - b2;
                const Vertex& va = low.vertices[low.indices[fi * 3]];
                const Vertex& vb = low.vertices[low.indices[fi * 3 + 1]];
                const Vertex& vc = low.vertices[low.indices[fi * 3 + 2]];
                const Vec3 P = va.position * b0 + vb.position * b1 + vc.position * b2 + target.offset;
                Vec3 N = normalize(va.normal * b0 + vb.normal * b1 + vc.normal * b2);
                if (!(length(N) > 0)) N = faceNormal(va.position, vb.position, vc.position);
                // The source along the normal, from outside the cage inwards; else the nearer
                // hit either way.
                Vec3 hitP = P, ns = N, colour = source.color;
                float t = 2.0f * cage;
                std::uint32_t tri = 0;
                if (bvh.raycast(Ray{P + N * cage, -N}, t, &tri)) {
                    hitP = P + N * (cage - t);
                    hitInfo(tri, hitP, ns, colour);
                } else {
                    ++missed;
                    float tIn = 1e30f, tOut = 1e30f;
                    std::uint32_t triIn = 0, triOut = 0;
                    const bool in = bvh.raycast(Ray{P, -N}, tIn, &triIn), out = bvh.raycast(Ray{P, N}, tOut, &triOut);
                    if (in && (!out || tIn <= tOut)) hitP = P - N * tIn, hitInfo(triIn, hitP, ns, colour);
                    else if (out) hitP = P + N * tOut, hitInfo(triOut, hitP, ns, colour);
                }
                Vec3 v{};
                switch (opt.map) {
                case BakeMap::Normal: {
                    const Vec3 T = normalize(tanU[fi] - N * dot(N, tanU[fi]));
                    const float hand = dot(cross(N, T), tanV[fi]) < 0 ? -1.0f : 1.0f;
                    const Vec3 B = cross(N, T) * hand;
                    v = Vec3{dot(ns, T), dot(ns, B), dot(ns, N)} * 0.5f + Vec3{0.5f, 0.5f, 0.5f};
                    break;
                }
                case BakeMap::ObjectNormal: v = ns * 0.5f + Vec3{0.5f, 0.5f, 0.5f}; break;
                case BakeMap::Height: v = {dot(hitP - P, N), 0, 0}; break;
                case BakeMap::Color: v = colour; break;
                case BakeMap::Occlusion: {
                    // Cosine-weighted rays over the hemisphere (a Hammersley set, turned and
                    // shifted per texel), blocked within `reach`.
                    const Vec3 e1 = normalize(cross(ns, std::fabs(ns.x) < 0.9f ? Vec3{1, 0, 0} : Vec3{0, 1, 0})), e2 = cross(ns, e1);
                    const std::uint32_t h = static_cast<std::uint32_t>(k) * 2654435761u;
                    const float shift = static_cast<float>(h >> 8) / 16777216.0f, turn = static_cast<float>((h * 2246822519u) >> 8) / 16777216.0f;
                    const int rays = std::clamp(opt.rays, 1, 1024);
                    int blocked = 0;
                    for (int i = 0; i < rays; ++i) {
                        std::uint32_t bitsR = static_cast<std::uint32_t>(i);
                        bitsR = (bitsR << 16) | (bitsR >> 16);
                        bitsR = ((bitsR & 0x55555555u) << 1) | ((bitsR & 0xAAAAAAAAu) >> 1);
                        bitsR = ((bitsR & 0x33333333u) << 2) | ((bitsR & 0xCCCCCCCCu) >> 2);
                        bitsR = ((bitsR & 0x0F0F0F0Fu) << 4) | ((bitsR & 0xF0F0F0F0u) >> 4);
                        bitsR = ((bitsR & 0x00FF00FFu) << 8) | ((bitsR & 0xFF00FF00u) >> 8);
                        float u1 = (static_cast<float>(i) + shift) / static_cast<float>(rays);
                        float u2 = static_cast<float>(bitsR) * 2.3283064e-10f + turn;
                        u1 -= std::floor(u1), u2 -= std::floor(u2);
                        const float r = std::sqrt(u1), phi = 2.0f * static_cast<float>(kPiD) * u2;
                        const Vec3 d = normalize(e1 * (r * std::cos(phi)) + e2 * (r * std::sin(phi)) + ns * std::sqrt(std::max(0.0f, 1.0f - u1)));
                        float tr = reach;
                        if (bvh.raycast(Ray{hitP + ns * (diag * 1e-4f), d}, tr)) ++blocked;
                    }
                    const float open = 1.0f - static_cast<float>(blocked) / static_cast<float>(rays);
                    v = {open, open, open};
                    break;
                }
                }
                value[k] = v;
                filled[k] = 1;
                ++texels;
            }
        }
    };
    const unsigned n = std::clamp(std::thread::hardware_concurrency(), 1u, 32u);
    std::vector<std::thread> pool;
    for (unsigned i = 1; i < n; ++i) pool.emplace_back(work);
    work();
    for (std::thread& t : pool) t.join();
    // Grown past the charts' edges, so a filtered lookup at a seam finds its own chart.
    for (int pass = 0; pass < std::clamp(opt.padding, 0, 64); ++pass) {
        std::vector<char> grown = filled;
        std::vector<Vec3> next = value;
        for (int y = 0; y < size; ++y)
            for (int x = 0; x < size; ++x) {
                const std::size_t k = static_cast<std::size_t>(y) * S + static_cast<std::size_t>(x);
                if (filled[k]) continue;
                Vec3 sum{};
                int count = 0;
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dx = -1; dx <= 1; ++dx) {
                        const int qx = x + dx, qy = y + dy;
                        if (qx < 0 || qy < 0 || qx >= size || qy >= size) continue;
                        const std::size_t q = static_cast<std::size_t>(qy) * S + static_cast<std::size_t>(qx);
                        if (filled[q]) sum += value[q], ++count;
                    }
                if (count) next[k] = sum / static_cast<float>(count), grown[k] = 1;
            }
        filled = std::move(grown);
        value = std::move(next);
    }
    // Out to pixels.
    Vec3 empty{};
    switch (opt.map) {
    case BakeMap::Normal: empty = {0.5f, 0.5f, 1.0f}; break;
    case BakeMap::ObjectNormal: empty = {0.5f, 0.5f, 0.5f}; break;
    case BakeMap::Occlusion: empty = {1, 1, 1}; break;
    case BakeMap::Color: empty = source.color; break;
    case BakeMap::Height: empty = {0, 0, 0}; break;
    }
    for (std::size_t k = 0; k < S * S; ++k)
        if (!filled[k]) value[k] = empty;
    auto byte = [](float x) { return static_cast<std::uint8_t>(std::lround(std::clamp(x, 0.0f, 1.0f) * 255.0f)); };
    float lowH = 0, highH = 0;
    if (opt.map == BakeMap::Height) {
        float m = 0;
        for (const Vec3& v : value) m = std::max(m, std::fabs(v.x));
        lowH = -m, highH = m;
        channels = 1, bits = 16;
        pixels.resize(S * S * 2);
        for (std::size_t k = 0; k < S * S; ++k) {
            const float t = m > 0 ? 0.5f + 0.5f * value[k].x / m : 0.5f;
            const auto q = static_cast<std::uint16_t>(std::lround(std::clamp(t, 0.0f, 1.0f) * 65535.0f));
            pixels[k * 2] = static_cast<std::uint8_t>(q >> 8), pixels[k * 2 + 1] = static_cast<std::uint8_t>(q & 0xFF);
        }
    } else if (opt.map == BakeMap::Occlusion) {
        channels = 1, bits = 8;
        pixels.resize(S * S);
        for (std::size_t k = 0; k < S * S; ++k) pixels[k] = byte(value[k].x);
    } else {
        channels = 3, bits = 8;
        pixels.resize(S * S * 3);
        for (std::size_t k = 0; k < S * S; ++k) pixels[k * 3] = byte(value[k].x), pixels[k * 3 + 1] = byte(value[k].y), pixels[k * 3 + 2] = byte(value[k].z);
    }
    if (report) {
        report->texels = texels;
        report->missed = missed;
        report->low = lowH, report->high = highH;
    }
    return true;
}

} // namespace sicdm

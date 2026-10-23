#include "core/Cam.hpp"

#include "core/Json.hpp"
#include "core/Scene.hpp"
#include "core/Section.hpp"
#include "core/Sculpt.hpp"

#include <algorithm>
#include <atomic>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <functional>
#include <limits>
#include <thread>
#include <unordered_map>

namespace sicdm {

namespace {

constexpr double kPiD = 3.14159265358979323846;
constexpr double kInf = std::numeric_limits<double>::infinity();

using section::Loop;
using section::P2;
using Tri = std::array<std::uint32_t, 3>;

struct D3 {
    double x = 0, y = 0, z = 0;
};

Vec3 vec(double x, double y, double z) { return {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)}; }
double dist2(const P2& a, const P2& b) { return std::hypot(a.x - b.x, a.y - b.y); }

// Runs f(i) for i in [0, n) on a few threads.
void parallelFor(int n, const std::function<void(int)>& f) {
    const int threads = std::clamp(static_cast<int>(std::thread::hardware_concurrency()), 1, 8);
    if (n < 64 || threads == 1) {
        for (int i = 0; i < n; ++i) f(i);
        return;
    }
    std::vector<std::thread> pool;
    for (int t = 0; t < threads; ++t)
        pool.emplace_back([&, t] {
            for (int i = t; i < n; i += threads) f(i);
        });
    for (auto& th : pool) th.join();
}

// --- the cutter's shape ---------------------------------------------------------

// A cutter of radius R whose tip is rounded with radius rc (0: flat, R: ball), or a
// cone (drills, spot and chamfer mills).
struct Cutter {
    double R = 3, rc = 0;
    double coneSlope = 0; // > 0: height per mm of radius (a cone)
    double h(double d) const noexcept {
        if (d > R) return kInf;
        if (coneSlope > 0) return d * coneSlope;
        const double a = R - rc;
        if (rc <= 0 || d <= a) return 0;
        const double e = d - a;
        return rc - std::sqrt(std::max(0.0, rc * rc - e * e));
    }
};

Cutter cutterOf(const CamTool& t, double leave = 0) {
    Cutter c;
    c.R = t.radius() + leave;
    if (t.type == CamToolType::Drill || t.type == CamToolType::Spot || t.type == CamToolType::Chamfer) {
        c.coneSlope = 1.0 / std::tan(std::clamp(t.tipAngle, 20.0, 179.0) * 0.5 * kPiD / 180.0);
        return c;
    }
    c.rc = std::min(t.tipRadius() + leave, c.R);
    return c;
}

// --- the part ---------------------------------------------------------------------

class PartModel {
public:
    PartModel(const Body& body, double binSize) {
        const MeshData m = worldMesh(body);
        p.reserve(m.vertices.size());
        for (const Vertex& v : m.vertices) p.push_back({v.position.x, v.position.y, v.position.z});
        for (std::size_t k = 0; k + 2 < m.indices.size(); k += 3) {
            const Tri t{m.indices[k], m.indices[k + 1], m.indices[k + 2]};
            const D3 &a = p[t[0]], &b = p[t[1]], &c = p[t[2]];
            const double ux = b.x - a.x, uy = b.y - a.y, uz = b.z - a.z, vx = c.x - a.x, vy = c.y - a.y, vz = c.z - a.z;
            D3 n{uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx};
            const double len = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
            if (!(len > 1e-14)) continue;
            n = {n.x / len, n.y / len, n.z / len};
            tris.push_back(t);
            normal.push_back(n);
            up.push_back(n.z);
            maxZ.push_back(std::max({a.z, b.z, c.z}));
        }
        for (const D3& q : p) {
            x0 = std::min(x0, q.x), y0 = std::min(y0, q.y), z0 = std::min(z0, q.z);
            x1 = std::max(x1, q.x), y1 = std::max(y1, q.y), z1 = std::max(z1, q.z);
        }
        if (tris.empty()) return;
        bs = std::max(binSize, std::max(x1 - x0, y1 - y0) / 512.0);
        bnx = std::max(1, static_cast<int>(std::ceil((x1 - x0) / bs)) + 1);
        bny = std::max(1, static_cast<int>(std::ceil((y1 - y0) / bs)) + 1);
        bins.assign(static_cast<std::size_t>(bnx) * static_cast<std::size_t>(bny), {});
        for (std::uint32_t i = 0; i < tris.size(); ++i) {
            const Tri& t = tris[i];
            double lx = kInf, ly = kInf, hx = -kInf, hy = -kInf;
            for (std::uint32_t v : t) lx = std::min(lx, p[v].x), ly = std::min(ly, p[v].y), hx = std::max(hx, p[v].x), hy = std::max(hy, p[v].y);
            for (int j = binY(ly); j <= binY(hy); ++j)
                for (int k = binX(lx); k <= binX(hx); ++k) bins[static_cast<std::size_t>(j * bnx + k)].push_back(i);
        }
        // Highest first: a search stops looking at a bin once its triangles are all lower.
        for (auto& b : bins) std::sort(b.begin(), b.end(), [&](std::uint32_t a, std::uint32_t c) { return maxZ[a] > maxZ[c]; });
    }

    bool empty() const noexcept { return tris.empty(); }

    // The lowest tip height at (x, y) that keeps the cutter out of the part.
    double drop(const Cutter& c, double x, double y, double floor) const {
        double best = floor;
        if (tris.empty() || x + c.R < x0 || x - c.R > x1 || y + c.R < y0 || y - c.R > y1) return best;
        const int ka = binX(x - c.R), kb = binX(x + c.R), ja = binY(y - c.R), jb = binY(y + c.R);
        for (int j = ja; j <= jb; ++j)
            for (int k = ka; k <= kb; ++k)
                for (const std::uint32_t ti : bins[static_cast<std::size_t>(j * bnx + k)]) {
                    if (maxZ[ti] <= best) break;
                    best = std::max(best, dropTriangle(c, ti, x, y, best));
                }
        return best;
    }

    // Every height where the vertical line through (x, y) crosses the surface, sorted.
    void crossings(double x, double y, std::vector<double>& zs) const {
        zs.clear();
        if (tris.empty() || x < x0 || x > x1 || y < y0 || y > y1) return;
        for (const std::uint32_t ti : bins[static_cast<std::size_t>(binY(y) * bnx + binX(x))]) {
            double z;
            if (insideXY(ti, x, y, &z)) zs.push_back(z);
        }
        std::sort(zs.begin(), zs.end());
    }

    // The highest point of the part's surface straight above or below (x, y); -inf off the part.
    double top(double x, double y) const {
        double best = -kInf;
        if (tris.empty() || x < x0 || x > x1 || y < y0 || y > y1) return best;
        for (const std::uint32_t ti : bins[static_cast<std::size_t>(binY(y) * bnx + binX(x))]) {
            if (maxZ[ti] <= best) break;
            double z;
            if (insideXY(ti, x, y, &z)) best = std::max(best, z);
        }
        return best;
    }

    std::vector<D3> p;
    std::vector<Tri> tris;
    std::vector<D3> normal;
    std::vector<double> up, maxZ;
    double x0 = kInf, y0 = kInf, z0 = kInf, x1 = -kInf, y1 = -kInf, z1 = -kInf;

private:
    int binX(double x) const { return std::clamp(static_cast<int>(std::floor((x - x0) / bs)), 0, bnx - 1); }
    int binY(double y) const { return std::clamp(static_cast<int>(std::floor((y - y0) / bs)), 0, bny - 1); }

    // Inside the triangle seen from above; z: the triangle's height there.
    bool insideXY(std::uint32_t ti, double x, double y, double* z) const {
        const Tri& t = tris[ti];
        const D3 &a = p[t[0]], &b = p[t[1]], &c = p[t[2]];
        const double d = (b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y);
        if (std::fabs(d) < 1e-18) return false;
        const double l1 = ((b.y - c.y) * (x - c.x) + (c.x - b.x) * (y - c.y)) / d;
        const double l2 = ((c.y - a.y) * (x - c.x) + (a.x - c.x) * (y - c.y)) / d;
        const double l3 = 1.0 - l1 - l2;
        constexpr double e = -1e-12;
        if (l1 < e || l2 < e || l3 < e) return false;
        *z = l1 * a.z + l2 * b.z + l3 * c.z;
        return true;
    }

    double dropTriangle(const Cutter& c, std::uint32_t ti, double x, double y, double best) const {
        const Tri& t = tris[ti];
        // Facet: the cutter touches the plane where its surface normal is the plane's.
        if (c.coneSlope <= 0) {
            D3 n = normal[ti];
            if (n.z < 0) n = {-n.x, -n.y, -n.z};
            if (n.z > 1e-9) {
                const double nh = std::hypot(n.x, n.y);
                double cx = x, cy = y;
                if (nh > 1e-12) {
                    cx += (c.R - c.rc) * (-n.x / nh) - c.rc * n.x;
                    cy += (c.R - c.rc) * (-n.y / nh) - c.rc * n.y;
                }
                double zp;
                if (insideXY(ti, cx, cy, &zp)) best = std::max(best, zp - c.rc * (1.0 - n.z));
            }
        } else { // a cone: its point on the facet
            double zp;
            if (insideXY(ti, x, y, &zp)) best = std::max(best, zp);
        }
        for (int e = 0; e < 3; ++e) {
            const D3& a = p[t[static_cast<std::size_t>(e)]];
            const D3& b = p[t[static_cast<std::size_t>((e + 1) % 3)]];
            const double d = std::hypot(a.x - x, a.y - y);
            if (d <= c.R) best = std::max(best, a.z - c.h(d));
            if (std::max(a.z, b.z) > best) best = std::max(best, dropEdge(c, a, b, x, y, best));
        }
        return best;
    }

    // The tip height where the cutter rests on edge ab; -inf when that can't beat `best`.
    static double dropEdge(const Cutter& c, const D3& a, const D3& b, double x, double y, double best = -kInf) {
        const double ux = b.x - a.x, uy = b.y - a.y, L = std::hypot(ux, uy);
        if (L < 1e-12) return -kInf; // vertical: its top corner is the vertex test
        const double ex = ux / L, ey = uy / L;
        const double tf = (x - a.x) * ex + (y - a.y) * ey;
        const double d0 = std::fabs((x - a.x) * ey - (y - a.y) * ex);
        if (d0 > c.R) return -kInf;
        const double w = std::sqrt(std::max(0.0, c.R * c.R - d0 * d0));
        const double ta = std::max(0.0, tf - w), tb = std::min(L, tf + w);
        if (ta > tb) return -kInf;
        const double m = (b.z - a.z) / L;
        // The cutter only rises away from its axis, and no point of the edge in reach is
        // nearer the axis than d0: nothing here can sit higher than this.
        if (a.z + std::max(m * ta, m * tb) - c.h(d0) <= best) return -kInf;
        // The interval's ends are at the rim (distance R, give or take rounding).
        auto f = [&](double t) { return a.z + m * t - c.h(std::min(c.R, std::hypot(d0, t - tf))); };
        if (c.coneSlope <= 0 && c.rc <= 0) return a.z + std::max(m * ta, m * tb); // flat: z is linear across the disc
        if (c.coneSlope <= 0 && c.rc >= c.R - 1e-12) {                     // ball: the circle of the sphere's section
            const double t = std::clamp(tf + m * w / std::sqrt(1 + m * m), ta, tb);
            return f(t);
        }
        // Bull nose and cone: concave along the edge, golden-section search.
        constexpr double g = 0.6180339887498949;
        double lo = ta, hi = tb, p1 = hi - g * (hi - lo), p2 = lo + g * (hi - lo), f1 = f(p1), f2 = f(p2);
        for (int i = 0; i < 48; ++i) {
            if (f1 < f2) {
                lo = p1, p1 = p2, f1 = f2, p2 = lo + g * (hi - lo), f2 = f(p2);
            } else {
                hi = p2, p2 = p1, f2 = f1, p1 = hi - g * (hi - lo), f1 = f(p1);
            }
        }
        return std::max({f(ta), f(tb), f1, f2});
    }

    std::vector<std::vector<std::uint32_t>> bins;
    double bs = 1.0;
    int bnx = 1, bny = 1;
};

// --- a grid of cutter heights -----------------------------------------------------------

struct Grid {
    double x0 = 0, y0 = 0, c = 1;
    int nx = 0, ny = 0;
    std::vector<double> v;
    double at(int i, int j) const { return v[static_cast<std::size_t>(j) * static_cast<std::size_t>(nx) + static_cast<std::size_t>(i)]; }
    double& at(int i, int j) { return v[static_cast<std::size_t>(j) * static_cast<std::size_t>(nx) + static_cast<std::size_t>(i)]; }
    P2 pos(int i, int j) const { return {x0 + i * c, y0 + j * c}; }
    // Bilinear value at a point (clamped to the grid).
    double sample(double x, double y) const {
        const double fx = std::clamp((x - x0) / c, 0.0, nx - 1.000001), fy = std::clamp((y - y0) / c, 0.0, ny - 1.000001);
        const int i = static_cast<int>(fx), j = static_cast<int>(fy);
        const double u = fx - i, w = fy - j;
        return (at(i, j) * (1 - u) + at(i + 1, j) * u) * (1 - w) + (at(i, j + 1) * (1 - u) + at(i + 1, j + 1) * u) * w;
    }
};

Grid makeGrid(double x0, double y0, double x1, double y1, double cell, std::size_t maxCells = 2500000) {
    Grid g;
    g.c = cell;
    for (;;) {
        g.nx = static_cast<int>(std::ceil((x1 - x0) / g.c)) + 3;
        g.ny = static_cast<int>(std::ceil((y1 - y0) / g.c)) + 3;
        if (static_cast<std::size_t>(g.nx) * static_cast<std::size_t>(g.ny) <= maxCells) break;
        g.c *= 1.25;
    }
    g.x0 = (x0 + x1) * 0.5 - (g.nx - 1) * g.c * 0.5;
    g.y0 = (y0 + y1) * 0.5 - (g.ny - 1) * g.c * 0.5;
    g.v.assign(static_cast<std::size_t>(g.nx) * static_cast<std::size_t>(g.ny), 0.0);
    return g;
}

Grid clGrid(const PartModel& part, const Cutter& cut, double x0, double y0, double x1, double y1, double cell, double floor) {
    Grid g = makeGrid(x0, y0, x1, y1, cell);
    parallelFor(g.ny, [&](int j) {
        for (int i = 0; i < g.nx; ++i) {
            const P2 q = g.pos(i, j);
            g.at(i, j) = part.drop(cut, q.x, q.y, floor);
        }
    });
    return g;
}

// --- loops where a field changes ---------------------------------------------------

// Marching squares over the grid's nodes: closed loops around the nodes where in(i, j),
// each with that region on its left. `cross` places the boundary on a grid edge between
// an inside node and an outside one; `inAt` decides the cells whose corners alternate.
// The nodes on the grid's border must be outside.
std::vector<Loop> isoLoops(const Grid& g, const std::function<bool(int, int)>& in,
                           const std::function<P2(const P2& inside, const P2& outside)>& cross,
                           const std::function<bool(double, double)>& inAt) {
    const int nx = g.nx, ny = g.ny;
    std::vector<char> inside(static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny));
    for (int j = 0; j < ny; ++j)
        for (int i = 0; i < nx; ++i) inside[static_cast<std::size_t>(j * nx + i)] = in(i, j) ? 1 : 0;
    auto isIn = [&](int i, int j) { return i >= 0 && j >= 0 && i < nx && j < ny && inside[static_cast<std::size_t>(j * nx + i)]; };
    // Edge keys: horizontal (i,j)-(i+1,j) = 2 (j nx + i); vertical (i,j)-(i,j+1) = 2 (j nx + i) + 1.
    std::unordered_map<std::uint64_t, P2> points;
    auto edgePoint = [&](int ia, int ja, int ib, int jb, std::uint64_t key) {
        const auto it = points.find(key);
        if (it != points.end()) return it->second;
        const bool aIn = isIn(ia, ja);
        const P2 pa = g.pos(ia, ja), pb = g.pos(ib, jb);
        const P2 q = aIn ? cross(pa, pb) : cross(pb, pa);
        points.emplace(key, q);
        return q;
    };
    struct Seg {
        std::uint64_t from, to;
    };
    std::vector<Seg> segs;
    for (int j = 0; j + 1 < ny; ++j)
        for (int i = 0; i + 1 < nx; ++i) {
            // Corners counter-clockwise: a (i,j), b (i+1,j), c (i+1,j+1), d (i,j+1).
            const int ci[4] = {i, i + 1, i + 1, i}, cj[4] = {j, j, j + 1, j + 1};
            bool cin[4];
            int count = 0;
            for (int k = 0; k < 4; ++k) count += (cin[k] = isIn(ci[k], cj[k])) ? 1 : 0;
            if (count == 0 || count == 4) continue;
            const std::uint64_t base = 2ull * (static_cast<std::uint64_t>(j) * static_cast<std::uint64_t>(nx) + static_cast<std::uint64_t>(i));
            const std::uint64_t keys[4] = {base, 2ull * (static_cast<std::uint64_t>(j) * nx + static_cast<std::uint64_t>(i + 1)) + 1,
                                           2ull * (static_cast<std::uint64_t>(j + 1) * nx + static_cast<std::uint64_t>(i)), base + 1};
            struct Crossing {
                std::uint64_t key;
                bool leaving; // in -> out walking counter-clockwise
            };
            Crossing xs[4];
            int n = 0;
            for (int k = 0; k < 4; ++k) {
                const int k2 = (k + 1) % 4;
                if (cin[k] == cin[k2]) continue;
                edgePoint(ci[k], cj[k], ci[k2], cj[k2], keys[k]);
                xs[n++] = {keys[k], cin[k]};
            }
            bool centreIn = false;
            if (n == 4) {
                const P2 m{g.x0 + (i + 0.5) * g.c, g.y0 + (j + 0.5) * g.c};
                centreIn = inAt(m.x, m.y);
            }
            for (int k = 0; k < n; ++k) {
                if (!xs[k].leaving) continue;
                const Crossing& partner = n == 2 ? xs[(k + 1) % 2] : (centreIn ? xs[(k + 1) % 4] : xs[(k + 3) % 4]);
                segs.push_back({xs[k].key, partner.key});
            }
        }
    std::unordered_map<std::uint64_t, std::size_t> starting;
    starting.reserve(segs.size() * 2);
    for (std::size_t s = 0; s < segs.size(); ++s) starting.emplace(segs[s].from, s);
    std::vector<char> used(segs.size(), 0);
    std::vector<Loop> loops;
    for (std::size_t s = 0; s < segs.size(); ++s) {
        if (used[s]) continue;
        Loop l;
        std::size_t k = s;
        bool closed = false;
        for (std::size_t guard = 0; guard <= segs.size(); ++guard) {
            used[k] = 1;
            l.push_back(points[segs[k].from]);
            const auto it = starting.find(segs[k].to);
            if (it == starting.end()) break;
            if (it->second == s) {
                closed = true;
                break;
            }
            if (used[it->second]) break;
            k = it->second;
        }
        if (!closed) continue;
        Loop clean;
        for (const P2& q : l)
            if (clean.empty() || dist2(q, clean.back()) > 1e-7) clean.push_back(q);
        while (clean.size() > 2 && dist2(clean.front(), clean.back()) <= 1e-7) clean.pop_back();
        if (clean.size() >= 3 && std::fabs(section::loopArea(clean)) > g.c * g.c * 0.02) loops.push_back(std::move(clean));
    }
    return loops;
}

// Douglas-Peucker on a closed loop: drops points within tol of the line through their
// neighbours that are kept.
Loop simplifyLoop(const Loop& l, double tol) {
    const std::size_t n = l.size();
    if (n < 4) return l;
    // Split at the point farthest from the first, and simplify both halves as open chains.
    std::size_t far = 0;
    double fd = -1;
    for (std::size_t i = 1; i < n; ++i)
        if (const double d = dist2(l[0], l[i]); d > fd) fd = d, far = i;
    std::vector<char> keep(n, 0);
    keep[0] = keep[far] = 1;
    std::vector<std::pair<std::size_t, std::size_t>> stack{{0, far}, {far, n}};
    while (!stack.empty()) {
        const auto [a, b] = stack.back();
        stack.pop_back();
        const P2& pa = l[a];
        const P2& pb = l[b % n];
        const double dx = pb.x - pa.x, dy = pb.y - pa.y, len = std::hypot(dx, dy);
        double worst = -1;
        std::size_t wi = a;
        for (std::size_t i = a + 1; i < b; ++i) {
            const double d = len > 1e-12 ? std::fabs((l[i].x - pa.x) * dy - (l[i].y - pa.y) * dx) / len : dist2(l[i], pa);
            if (d > worst) worst = d, wi = i;
        }
        if (worst > tol) {
            keep[wi] = 1;
            stack.push_back({a, wi});
            stack.push_back({wi, b});
        }
    }
    Loop out;
    for (std::size_t i = 0; i < n; ++i)
        if (keep[i]) out.push_back(l[i]);
    return out.size() >= 3 ? out : l;
}

double loopLength(const Loop& l) {
    double s = 0;
    for (std::size_t i = 0; i < l.size(); ++i) s += dist2(l[i], l[(i + 1) % l.size()]);
    return s;
}

// Laplacian smoothing of a closed loop (clearing rings from a distance field are a
// little jagged at the grid's scale).
void smoothLoop(Loop& l, int passes) {
    const std::size_t n = l.size();
    if (n < 5) return;
    for (int p = 0; p < passes; ++p) {
        Loop o(n);
        for (std::size_t i = 0; i < n; ++i) {
            const P2 &a = l[(i + n - 1) % n], &b = l[i], &c = l[(i + 1) % n];
            o[i] = {b.x * 0.5 + (a.x + c.x) * 0.25, b.y * 0.5 + (a.y + c.y) * 0.25};
        }
        l = std::move(o);
    }
}

// Squared Euclidean distance transform along one line (Felzenszwalb and Huttenlocher):
// f = 0 at the sources, kFar elsewhere.
constexpr double kFar = 1e20;
void edt1(const std::vector<double>& f, std::vector<double>& d, std::vector<int>& v, std::vector<double>& z, int n) {
    auto at = [](auto& a, int i) -> auto& { return a[static_cast<std::size_t>(i)]; };
    int k = 0;
    at(v, 0) = 0;
    at(z, 0) = -kInf;
    at(z, 1) = kInf;
    for (int q = 1; q < n; ++q) {
        auto meet = [&](int r) { return ((at(f, q) + q * q) - (at(f, r) + r * r)) / (2.0 * (q - r)); };
        double s = meet(at(v, k));
        while (s <= at(z, k)) {
            --k;
            s = meet(at(v, k));
        }
        ++k;
        at(v, k) = q;
        at(z, k) = s;
        at(z, k + 1) = kInf;
    }
    k = 0;
    for (int q = 0; q < n; ++q) {
        while (at(z, k + 1) < q) ++k;
        const int r = at(v, k);
        at(d, q) = (q - r) * static_cast<double>(q - r) + at(f, r);
    }
}

// Distance (mm) from each node inside to the nearest node outside.
Grid distanceInside(const Grid& g, const std::vector<char>& inside) {
    Grid out = g;
    const int nx = g.nx, ny = g.ny, m = std::max(nx, ny);
    std::vector<double> f(static_cast<std::size_t>(m)), d(static_cast<std::size_t>(m)), z(static_cast<std::size_t>(m) + 1);
    std::vector<int> v(static_cast<std::size_t>(m));
    for (std::size_t k = 0; k < out.v.size(); ++k) out.v[k] = inside[k] ? kFar : 0.0;
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) f[static_cast<std::size_t>(i)] = out.at(i, j);
        edt1(f, d, v, z, nx);
        for (int i = 0; i < nx; ++i) out.at(i, j) = d[static_cast<std::size_t>(i)];
    }
    for (int i = 0; i < nx; ++i) {
        for (int j = 0; j < ny; ++j) f[static_cast<std::size_t>(j)] = out.at(i, j);
        edt1(f, d, v, z, ny);
        for (int j = 0; j < ny; ++j) out.at(i, j) = std::sqrt(std::min(d[static_cast<std::size_t>(j)], kFar)) * g.c;
    }
    return out;
}

} // namespace

// --- tools and materials ---------------------------------------------------------------

namespace {
struct ToolKey {
    CamToolType type;
    const char* key;
    const char* label;
};
constexpr ToolKey kToolKeys[] = {{CamToolType::Flat, "flat", "flat end mill"},     {CamToolType::Ball, "ball", "ball end mill"},
                                 {CamToolType::BullNose, "bull", "bull-nose end mill"}, {CamToolType::FaceMill, "face", "face mill"},
                                 {CamToolType::Drill, "drill", "drill"},             {CamToolType::Spot, "spot", "spot drill"},
                                 {CamToolType::Chamfer, "chamfer", "chamfer mill"},  {CamToolType::Tap, "tap", "tap"}};
} // namespace

const char* camToolTypeKey(CamToolType t) {
    for (const ToolKey& k : kToolKeys)
        if (k.type == t) return k.key;
    return "flat";
}

bool camToolTypeFromKey(const std::string& key, CamToolType* out) {
    for (const ToolKey& k : kToolKeys)
        if (key == k.key) {
            *out = k.type;
            return true;
        }
    if (key == "endmill" || key == "end") return *out = CamToolType::Flat, true;
    if (key == "bullnose" || key == "torus") return *out = CamToolType::BullNose, true;
    return false;
}

double CamTool::tipRadius() const noexcept {
    if (type == CamToolType::Ball) return radius();
    if (type == CamToolType::BullNose) return std::clamp(cornerRadius, 0.0, radius());
    return 0.0;
}

double CamTool::gauge() const noexcept { return stickout > 0 ? stickout : fluteLength + std::max(5.0, diameter); }

double CamTool::holderRadius() const noexcept { return 0.5 * (holderDiameter > 0 ? holderDiameter : std::max(20.0, 3.0 * diameter)); }

double CamTool::profile(double d) const noexcept { return cutterOf(*this).h(d); }

std::string CamTool::describe() const {
    const char* label = "end mill";
    for (const ToolKey& k : kToolKeys)
        if (k.type == type) label = k.label;
    char b[200];
    std::snprintf(b, sizeof(b), "T%d  %g mm %s", number, diameter, label);
    std::string s = b;
    if (type == CamToolType::BullNose) {
        std::snprintf(b, sizeof(b), " R%g", cornerRadius);
        s += b;
    }
    if (type == CamToolType::Drill || type == CamToolType::Spot || type == CamToolType::Chamfer) {
        std::snprintf(b, sizeof(b), ", %g deg point", tipAngle);
        s += b;
    }
    if (type == CamToolType::Tap) {
        std::snprintf(b, sizeof(b), ", pitch %g mm", pitch);
        s += b;
    }
    std::snprintf(b, sizeof(b), ", %d flute%s, %s", flutes, flutes == 1 ? "" : "s", hss ? "HSS" : "carbide");
    s += b;
    if (!name.empty()) s += " (" + name + ")";
    if (!vendor.empty() || !productId.empty()) s += " [" + vendor + (vendor.empty() || productId.empty() ? "" : " ") + productId + "]";
    return s;
}

const std::vector<CamMaterial>& camMaterials() {
    // Cutting speeds (m/min) and chip loads for general machining; conservative, from
    // handbook ranges for uncoated carbide and HSS.
    static const std::vector<CamMaterial> kMaterials = {
        {"aluminium", "Aluminium 6061", 300, 90, 0.008, 0.020, 0.50},
        {"steel", "Mild steel 1018", 120, 30, 0.005, 0.015, 0.25},
        {"stainless", "Stainless steel 304", 80, 18, 0.004, 0.012, 0.20},
        {"brass", "Brass C360", 180, 60, 0.007, 0.020, 0.50},
        {"castiron", "Cast iron", 100, 25, 0.005, 0.018, 0.30},
        {"titanium", "Titanium Ti-6Al-4V", 50, 12, 0.003, 0.010, 0.15},
        {"plastic", "Plastic (acrylic, POM, ABS)", 250, 120, 0.012, 0.025, 0.60},
        {"wood", "Wood and MDF", 400, 200, 0.015, 0.030, 0.75},
    };
    return kMaterials;
}

const std::vector<CamMachine>& camMachinePresets() {
    // Typical figures for each class of machine: set the real ones with CAM MACHINE.
    static const std::vector<CamMachine> kPresets = [] {
        std::vector<CamMachine> v;
        CamMachine g; // no limits checked
        v.push_back(g);
        CamMachine vmc; // a 30 x 16 x 20 inch vertical machining centre
        vmc.name = "vmc";
        vmc.maxRpm = 8100, vmc.maxFeed = 16500, vmc.rapid = 25400, vmc.toolChange = 4.2;
        vmc.travelMin[0] = 0, vmc.travelMax[0] = 762;
        vmc.travelMin[1] = 0, vmc.travelMax[1] = 406;
        vmc.travelMin[2] = -508, vmc.travelMax[2] = 0;
        vmc.toolPockets = 20;
        vmc.hasA = vmc.hasC = false;
        v.push_back(vmc);
        CamMachine four = vmc; // with a rotary table on A
        four.name = "vmc-4th";
        four.hasA = true;
        v.push_back(four);
        CamMachine tr; // a compact 5-axis trunnion: A tilts, C turns without end
        tr.name = "trunnion";
        tr.maxRpm = 12000, tr.maxFeed = 16500, tr.rapid = 30000, tr.toolChange = 3.5;
        tr.travelMin[0] = 0, tr.travelMax[0] = 610;
        tr.travelMin[1] = 0, tr.travelMax[1] = 406;
        tr.travelMin[2] = -394, tr.travelMax[2] = 0;
        tr.hasA = tr.hasC = true;
        tr.aMin = -35, tr.aMax = 120;
        tr.toolPockets = 30;
        v.push_back(tr);
        CamMachine bench; // a benchtop mill with a small changer
        bench.name = "benchtop";
        bench.maxRpm = 10000, bench.maxFeed = 2540, bench.rapid = 7620, bench.toolChange = 10;
        bench.travelMin[0] = 0, bench.travelMax[0] = 457;
        bench.travelMin[1] = 0, bench.travelMax[1] = 279;
        bench.travelMin[2] = -413, bench.travelMax[2] = 0;
        bench.toolPockets = 10;
        bench.hasA = bench.hasC = false;
        v.push_back(bench);
        CamMachine router; // a hobby router (GRBL), tools changed by hand
        router.name = "router";
        router.maxRpm = 24000, router.maxFeed = 5000, router.rapid = 5000, router.toolChange = 60;
        router.travelMin[0] = 0, router.travelMax[0] = 800;
        router.travelMin[1] = 0, router.travelMax[1] = 800;
        router.travelMin[2] = -100, router.travelMax[2] = 0;
        router.hasA = router.hasC = false;
        v.push_back(router);
        return v;
    }();
    return kPresets;
}

const CamMachine* camMachinePreset(const std::string& name) {
    std::string k;
    for (char ch : name) k += static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    for (const CamMachine& m : camMachinePresets())
        if (m.name == k) return &m;
    return nullptr;
}

std::string CamMachine::describe() const {
    char b[160];
    std::string s = name;
    std::snprintf(b, sizeof b, ": %.0f rpm, feed %.0f, rapid %.0f mm/min", maxRpm, maxFeed, rapid);
    s += b;
    const char* axes = "XYZ";
    std::string travel;
    for (int i = 0; i < 3; ++i)
        if (hasTravel(i)) {
            std::snprintf(b, sizeof b, "%s%c %g..%g", travel.empty() ? "" : ", ", axes[i], travelMin[i], travelMax[i]);
            travel += b;
        }
    s += travel.empty() ? "; travel not checked" : "; travel " + travel;
    if (hasWorkOffset) {
        std::snprintf(b, sizeof b, "; G54 at %g, %g, %g", workOffset[0], workOffset[1], workOffset[2]);
        s += b;
    }
    auto rot = [&](char axis, double lo, double hi) {
        if (lo <= -99999 && hi >= 99999) std::snprintf(b, sizeof b, "; %c continuous", axis);
        else std::snprintf(b, sizeof b, "; %c %g..%g deg", axis, lo, hi);
        s += b;
    };
    if (hasA) rot('A', aMin, aMax);
    if (hasC) rot('C', cMin, cMax);
    if (!hasA && !hasC) s += "; no rotary axes";
    if (toolPockets > 0) s += "; " + std::to_string(toolPockets) + " tool pockets";
    return s;
}

const CamMaterial* camMaterial(const std::string& key) {
    std::string k;
    for (char ch : key) k += static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    if (k == "aluminum" || k == "al" || k == "alu") k = "aluminium";
    if (k == "mild" || k == "mildsteel") k = "steel";
    if (k == "ss" || k == "inox") k = "stainless";
    if (k == "iron") k = "castiron";
    if (k == "ti") k = "titanium";
    if (k == "acrylic" || k == "pom" || k == "delrin" || k == "abs") k = "plastic";
    if (k == "mdf" || k == "plywood") k = "wood";
    for (const CamMaterial& m : camMaterials())
        if (m.key == k) return &m;
    return nullptr;
}

Json camToolToJson(const CamTool& t) {
    Json o = Json::object();
    o.set("number", t.number);
    o.set("type", camToolTypeKey(t.type));
    o.set("diameter", t.diameter);
    o.set("corner", t.cornerRadius);
    o.set("flute_length", t.fluteLength);
    if (t.stickout > 0) o.set("stickout", t.stickout);
    if (t.holderDiameter > 0) o.set("holder_diameter", t.holderDiameter);
    o.set("holder_length", t.holderLength);
    o.set("angle", t.tipAngle);
    o.set("pitch", t.pitch);
    o.set("flutes", t.flutes);
    o.set("hss", t.hss);
    o.set("name", t.name);
    if (!t.vendor.empty()) o.set("vendor", t.vendor);
    if (!t.productId.empty()) o.set("product_id", t.productId);
    if (t.vc > 0) o.set("vc", t.vc);
    if (t.fz > 0) o.set("fz", t.fz);
    return o;
}

CamTool camToolFromJson(const Json& o) {
    CamTool t;
    t.number = o["number"].asInt(1);
    if (!camToolTypeFromKey(o["type"].asString("flat"), &t.type)) t.type = CamToolType::Flat;
    t.diameter = o["diameter"].asNumber(6);
    t.cornerRadius = o["corner"].asNumber(0);
    t.fluteLength = o["flute_length"].asNumber(20);
    t.stickout = o["stickout"].asNumber(0);
    t.holderDiameter = o["holder_diameter"].asNumber(0);
    t.holderLength = o["holder_length"].asNumber(40);
    t.tipAngle = o["angle"].asNumber(118);
    t.pitch = o["pitch"].asNumber(1);
    t.flutes = o["flutes"].asInt(3);
    t.hss = o["hss"].asBool(false);
    t.name = o["name"].asString("");
    t.vendor = o["vendor"].asString("");
    t.productId = o["product_id"].asString("");
    t.vc = o["vc"].asNumber(0);
    t.fz = o["fz"].asNumber(0);
    return t;
}

CamFeeds camFeeds(const CamTool& t, const CamMaterial& m, const CamMachine& machine, double stepover) {
    CamFeeds f;
    const double d = std::max(t.diameter, 0.1);
    double vc = t.hss ? m.cuttingSpeedHss : m.cuttingSpeedCarbide;
    if (t.type == CamToolType::Drill || t.type == CamToolType::Spot) vc *= 0.7;
    if (t.type == CamToolType::Tap) vc *= 0.15;
    if (t.vc > 0) vc = t.vc; // the tool maker's own figure
    f.rpm = std::min(vc * 1000.0 / (kPiD * d), machine.maxRpm);
    if (t.type == CamToolType::Tap) f.rpm = std::min(f.rpm, 1000.0);
    f.rpm = std::max(100.0, std::floor(f.rpm / 10.0) * 10.0);
    if (t.type == CamToolType::Drill || t.type == CamToolType::Spot || t.type == CamToolType::Tap) {
        const double perRev = t.type == CamToolType::Tap ? t.pitch : t.fz > 0 ? t.fz : std::clamp(m.drillFeedPerDiameter * d, 0.01, 0.5);
        f.feed = std::min(f.rpm * perRev, machine.maxFeed);
        if (t.type == CamToolType::Tap) f.feed = f.rpm * t.pitch; // synchronised: never clamped
        f.plunge = f.feed;
        f.chipLoad = perRev;
        return f;
    }
    double fz = std::clamp(m.chipLoadPerDiameter * d, 0.005, 0.3);
    if (t.type == CamToolType::FaceMill) fz *= 1.2;
    if (t.fz > 0) fz = t.fz;
    // Radial chip thinning: below half the diameter the chip is thinner than the feed per
    // tooth, so the feed rises to keep it at the target.
    if (stepover > 0 && stepover < d * 0.5) fz *= std::min(3.0, d / (2.0 * std::sqrt(stepover * (d - stepover))));
    f.chipLoad = fz;
    f.feed = std::min(f.rpm * std::max(t.flutes, 1) * fz, machine.maxFeed);
    f.feed = std::max(1.0, std::floor(f.feed));
    f.plunge = std::max(1.0, std::floor(f.feed * 0.3));
    return f;
}

const char* camOpKey(CamOpKind k) {
    switch (k) {
    case CamOpKind::Face: return "face";
    case CamOpKind::Contour: return "contour";
    case CamOpKind::Pocket: return "pocket";
    case CamOpKind::Rough: return "rough";
    case CamOpKind::Parallel: return "parallel";
    case CamOpKind::Waterline: return "waterline";
    case CamOpKind::Drill: return "drill";
    case CamOpKind::Bore: return "bore";
    case CamOpKind::Rotary: return "rotary";
    case CamOpKind::Surface5: return "5axis";
    }
    return "rough";
}

namespace {
constexpr std::pair<DrillCycle, const char*> kCycles[] = {{DrillCycle::Spot, "spot"}, {DrillCycle::Drill, "drill"},
                                                          {DrillCycle::Peck, "peck"}, {DrillCycle::ChipBreak, "chipbreak"},
                                                          {DrillCycle::Tap, "tap"},   {DrillCycle::Bore, "bore"}};
} // namespace

const char* drillCycleKey(DrillCycle c) {
    for (const auto& [cy, key] : kCycles)
        if (cy == c) return key;
    return "drill";
}

bool drillCycleFromKey(const std::string& key, DrillCycle* out) {
    for (const auto& [cy, k] : kCycles)
        if (key == k) return *out = cy, true;
    if (key == "chip" || key == "break") return *out = DrillCycle::ChipBreak, true;
    if (key == "ream") return *out = DrillCycle::Bore, true;
    return false;
}

const CamTool* CamJob::tool(int number) const {
    for (const CamTool& t : tools)
        if (t.number == number) return &t;
    return nullptr;
}

Vec3 CamJob::origin() const {
    if (setup.hasStock && multiAxis()) return rotaryCentre();
    if (!setup.hasStock || setup.origin == CamSetup::Origin::Model) return {};
    if (setup.origin == CamSetup::Origin::StockTopCentre)
        return {(setup.stockMin.x + setup.stockMax.x) * 0.5f, (setup.stockMin.y + setup.stockMax.y) * 0.5f, setup.stockMax.z};
    return {setup.stockMin.x, setup.stockMin.y, setup.stockMax.z};
}

bool CamJob::multiAxis() const {
    return std::any_of(ops.begin(), ops.end(), [](const CamOperation& o) {
        return o.kind == CamOpKind::Rotary || o.kind == CamOpKind::Surface5 || o.indexA != 0 || o.indexC != 0;
    });
}

Vec3 CamJob::rotaryCentre() const { return (setup.stockMin + setup.stockMax) * 0.5f; }

double CamJob::swingRadius() const {
    const Vec3 c = rotaryCentre();
    return std::hypot(std::max(std::fabs(setup.stockMin.y - c.y), std::fabs(setup.stockMax.y - c.y)),
                      std::max(std::fabs(setup.stockMin.z - c.z), std::fabs(setup.stockMax.z - c.z)));
}

double CamJob::clearZ() const {
    if (!multiAxis()) return setup.stockMax.z + setup.clearance;
    return rotaryCentre().z + length(setup.stockMax - setup.stockMin) * 0.5 + setup.clearance; // any turn of the table
}

namespace {
Vec3 turnA(const Vec3& v, double deg) {
    const double r = deg * kPiD / 180.0, cs = std::cos(r), sn = std::sin(r);
    return vec(v.x, v.y * cs - v.z * sn, v.y * sn + v.z * cs);
}
Vec3 turnC(const Vec3& v, double deg) {
    const double r = deg * kPiD / 180.0, cs = std::cos(r), sn = std::sin(r);
    return vec(v.x * cs - v.y * sn, v.x * sn + v.y * cs, v.z);
}
} // namespace

Vec3 camToMachine(const CamJob& job, const Vec3& part, float a, float c) {
    const Vec3 o = job.rotaryCentre();
    return o + turnA(turnC(part - o, c), a);
}

Vec3 camToPart(const CamJob& job, const Vec3& machine, float a, float c) {
    const Vec3 o = job.rotaryCentre();
    return o + turnC(turnA(machine - o, -a), -c);
}

Vec3 camToolAxis(float a, float c) { return turnC(turnA({0, 0, 1}, -a), -c); }

double cutterHeight(const Body& part, const CamTool& tool, double x, double y, double floor) {
    const PartModel pm(part, std::max(tool.radius(), 1.0));
    return pm.drop(cutterOf(tool), x, y, floor);
}

double camPartKey(const Body& part) {
    double k = static_cast<double>(part.mesh.vertices.size()) * 7.0 + static_cast<double>(part.mesh.indices.size());
    std::size_t i = 0;
    for (const Vertex& v : part.mesh.vertices) {
        const double w = 1.0 + static_cast<double>(i++ % 97) * 0.013;
        k += (v.position.x * 1.31 + v.position.y * 1.77 + v.position.z * 2.03) * w;
    }
    k += part.offset.x * 3.1 + part.offset.y * 5.3 + part.offset.z * 7.7 + part.scale * 11.0 + part.rotation.x * 13.0 + part.rotation.y * 17.0 +
         part.rotation.z * 19.0;
    return k;
}

double movesSeconds(const std::vector<CamMove>& moves, const CamMachine& machine, const Vec3& from, const Vec3* centre) {
    double s = 0;
    Vec3 at = from;
    float atA = moves.empty() ? 0.0f : moves[0].a, atC = moves.empty() ? 0.0f : moves[0].c;
    for (const CamMove& m : moves) {
        double len = length(m.to - at);
        if (centre && m.c != atC) { // A and C both turning (5-axis): the tip's path over the part
            const Vec3 pa = *centre + turnC(turnA(at - *centre, -atA), -atC), pb = *centre + turnC(turnA(m.to - *centre, -m.a), -m.c);
            len = length(pb - pa);
        } else if (centre && m.a != atA) { // the turn, along the part
            const double r = 0.5 * (std::hypot(at.y - centre->y, at.z - centre->z) + std::hypot(m.to.y - centre->y, m.to.z - centre->z));
            len = std::hypot(len, r * (m.a - atA) * kPiD / 180.0);
        }
        atA = m.a, atC = m.c;
        const double rate = m.kind == CamMove::Rapid ? machine.rapid : std::max(m.feed, 1.0);
        s += len / rate * 60.0;
        at = m.to;
    }
    return s;
}

// --- toolpaths ----------------------------------------------------------------------------

namespace {

constexpr double kRamp = 3.0 * kPiD / 180.0; // ramp entry angle

struct Ctx {
    const CamJob& job;
    const CamTool& tool;
    const PartModel& part;
    CamOperation& op;
    Cutter cut;                // the cutter grown by the stock to leave
    double sx0, sy0, sx1, sy1; // stock outline
    double stockTop, stockBottom;
    double retractZ, clearZ;
    double tol;
    CamFeeds feeds;
    double approach = 2.0;     // finishing: rapid down to this far above the surface (roughing leaves steps)
    const CamSimulation* rest = nullptr; // rest machining: the stock the operations before left
    double D() const { return tool.diameter; }
    double R() const { return tool.radius(); }
    // The tool's tip height at (x, y) that leaves the stock to leave on the part.
    double cl(double x, double y, double floor) const { return part.drop(cut, x, y, floor - op.leave) + op.leave; }
};

class PathOut {
public:
    PathOut(std::vector<CamMove>& m, const Ctx& c) : moves(m), ctx(c) {}
    void rapid(double x, double y, double z) { push(CamMove::Rapid, x, y, z, 0); }
    void feed(double x, double y, double z) { push(CamMove::Feed, x, y, z, ctx.feeds.feed); }
    void feed(double x, double y, double z, double f) { push(CamMove::Feed, x, y, z, f); }
    void plunge(double x, double y, double z) { push(CamMove::Plunge, x, y, z, ctx.feeds.plunge); }
    void liftTo(double z) {
        if (started && at.z < z - 1e-9) rapid(at.x, at.y, z);
    }
    // Moves to above `to` and down to `downTo` (rapid), lifting only as high as the part
    // under the way (and `above`) needs.
    void travel(const P2& to, double above, double downTo, double floor) {
        if (!started) {
            rapid(to.x, to.y, ctx.clearZ);
            rapid(to.x, to.y, std::max(downTo, std::min(above, ctx.retractZ)));
            return;
        }
        double h = above;
        const double len = std::hypot(to.x - at.x, to.y - at.y);
        const int n = std::max(1, static_cast<int>(std::ceil(len / std::max(ctx.R() * 0.5, 0.25))));
        for (int k = 0; k <= n; ++k) {
            const double t = static_cast<double>(k) / n;
            h = std::max(h, ctx.cl(at.x + (to.x - at.x) * t, at.y + (to.y - at.y) * t, floor) + std::max(1.0, ctx.approach));
        }
        h = std::min(h, ctx.retractZ);
        if (len > 1e-6) {
            liftTo(h);
            rapid(to.x, to.y, std::max(at.z, h));
        }
        if (at.z > downTo) rapid(to.x, to.y, downTo);
    }
    // Feed along a straight line at z when nothing of the part is in the way.
    bool freeLine(const P2& a, const P2& b, double z, double floor) const {
        const double len = dist2(a, b);
        const int n = std::max(1, static_cast<int>(std::ceil(len / std::max(ctx.R() * 0.25, 0.1))));
        for (int k = 0; k <= n; ++k) {
            const double t = static_cast<double>(k) / n;
            if (ctx.cl(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, floor) > z + ctx.tol) return false;
        }
        return true;
    }
    void end() { liftTo(ctx.clearZ); }

    std::vector<CamMove>& moves;
    const Ctx& ctx;
    D3 at{};
    bool started = false;

private:
    void push(CamMove::Kind k, double x, double y, double z, double f) {
        if (started && std::fabs(x - at.x) < 1e-7 && std::fabs(y - at.y) < 1e-7 && std::fabs(z - at.z) < 1e-7) return;
        moves.push_back({k, vec(x, y, z), f});
        at = {x, y, z};
        started = true;
    }
};

// Rotates a closed loop to start at its point nearest q.
Loop startNear(const Loop& l, const P2& q) {
    std::size_t best = 0;
    double bd = kInf;
    for (std::size_t i = 0; i < l.size(); ++i)
        if (const double d = dist2(l[i], q); d < bd) bd = d, best = i;
    Loop o;
    o.reserve(l.size());
    for (std::size_t i = 0; i < l.size(); ++i) o.push_back(l[(best + i) % l.size()]);
    return o;
}

double nearestDistance(const Loop& l, const P2& q) {
    double bd = kInf;
    for (const P2& p : l) bd = std::min(bd, dist2(p, q));
    return bd;
}

// Cuts once around a closed loop at z, starting from its first point, ramping down to z
// from `from` first when from > z.
void cutLoop(PathOut& o, const Loop& l, double z, double from) {
    const std::size_t n = l.size();
    if (n < 2) return;
    if (from > z + 1e-9) {
        const double need = (from - z) / std::tan(kRamp), len = loopLength(l);
        if (len < o.ctx.D() * 0.5) {
            o.plunge(l[0].x, l[0].y, z); // too small to ramp in
        } else {
            double done = 0, zz = from;
            o.plunge(l[0].x, l[0].y, from);
            for (std::size_t i = 1; zz > z + 1e-9 && i < n * 64; ++i) {
                const P2& a = l[(i - 1) % n];
                const P2& b = l[i % n];
                done += dist2(a, b);
                zz = std::max(z, from - (from - z) * done / need);
                o.feed(b.x, b.y, zz, o.ctx.feeds.plunge * 2.0);
            }
            // Ramp done somewhere on the loop: go round once more from there.
            const P2 here{o.at.x, o.at.y};
            const Loop again = startNear(l, here);
            for (std::size_t i = 0; i <= again.size(); ++i) o.feed(again[i % again.size()].x, again[i % again.size()].y, z);
            return;
        }
    }
    for (std::size_t i = 0; i <= n; ++i) o.feed(l[i % n].x, l[i % n].y, z);
}

// Snaps marching-squares chamfers back to the sharp corner they cut: when the corner is
// free and the region turns blocked just past it (a rounded corner is left alone).
void sharpenCorners(Loop& l, double cell, double tol, const std::function<bool(const P2&)>& ok) {
    if (l.size() < 5) return;
    for (std::size_t i = 0; i < l.size() && l.size() >= 5; ++i) {
        const std::size_t n = l.size();
        const P2 &a = l[(i + n - 1) % n], &b = l[i], &c = l[(i + 1) % n], &d = l[(i + 2) % n];
        if (dist2(b, c) > cell * 2.0) continue;
        const double ux = b.x - a.x, uy = b.y - a.y, vx = d.x - c.x, vy = d.y - c.y;
        const double lu = std::hypot(ux, uy), lv = std::hypot(vx, vy);
        if (lu < cell * 0.5 || lv < cell * 0.5) continue;
        const double cosA = (ux * vx + uy * vy) / (lu * lv);
        if (cosA > std::cos(25.0 * kPiD / 180.0)) continue;
        const double den = ux * vy - uy * vx;
        if (std::fabs(den) < 1e-12) continue;
        const double t = ((c.x - b.x) * vy - (c.y - b.y) * vx) / den;
        const P2 x{b.x + ux * t, b.y + uy * t};
        if (dist2(x, b) > cell * 2.0 || dist2(x, c) > cell * 2.0 || !ok(x)) continue;
        const P2 m{(b.x + c.x) * 0.5, (b.y + c.y) * 0.5};
        const double out = dist2(x, m);
        if (out < 1e-9) continue;
        const double e = std::max(tol, 1e-3) * 2.0;
        if (ok({x.x + (x.x - m.x) / out * e, x.y + (x.y - m.y) / out * e})) continue; // still free past it: round
        if (!ok({(x.x + m.x) * 0.5, (x.y + m.y) * 0.5})) continue;
        l[i] = x;
        l.erase(l.begin() + static_cast<std::ptrdiff_t>((i + 1) % n));
    }
}

// Adds boundary points between loop points whose chord cuts into the blocked side
// (`blocked`), found by bisection toward the loop's right (the free side).
void refineChords(Loop& l, double cell, double tol, const std::function<bool(const P2&)>& blocked) {
    for (int pass = 0; pass < 4; ++pass) {
        Loop out;
        bool changed = false;
        for (std::size_t i = 0; i < l.size(); ++i) {
            const P2& a = l[i];
            const P2& b = l[(i + 1) % l.size()];
            out.push_back(a);
            const P2 m{(a.x + b.x) * 0.5, (a.y + b.y) * 0.5};
            const double len = dist2(a, b);
            if (len < 1e-3 || !blocked(m)) continue;
            // The free side is to the right of a -> b when the blocked region is on the left;
            // a chord inside the tolerance of it stays.
            const P2 r{(b.y - a.y) / len, -(b.x - a.x) / len};
            if (!blocked({m.x + r.x * tol, m.y + r.y * tol})) continue;
            P2 in = m, outP{m.x + r.x * cell, m.y + r.y * cell};
            if (blocked(outP)) continue;
            for (int k = 0; k < 18; ++k) {
                const P2 h{(in.x + outP.x) * 0.5, (in.y + outP.y) * 0.5};
                (blocked(h) ? in : outP) = h;
            }
            out.push_back(outP);
            changed = true;
        }
        l = std::move(out);
        if (!changed) break;
    }
}

// The boundary of the region where the cutter can reach `level`, as loops with the
// blocked side (the part) on the left, exact to the tolerance.
std::vector<Loop> levelLoops(const Ctx& c, const Grid& g, double level, double floor) {
    auto blockedAt = [&](double x, double y) { return c.cl(x, y, floor) > level + 1e-9; };
    std::vector<Loop> loops = isoLoops(
        g, [&](int i, int j) { return g.at(i, j) > level + 1e-9; },
        [&](const P2& in, const P2& out) {
            P2 a = in, b = out;
            for (int k = 0; k < 20; ++k) {
                const P2 h{(a.x + b.x) * 0.5, (a.y + b.y) * 0.5};
                (blockedAt(h.x, h.y) ? a : b) = h;
            }
            return b; // the free side
        },
        blockedAt);
    for (Loop& l : loops) {
        l = simplifyLoop(l, c.tol * 0.5);
        sharpenCorners(l, g.c, c.tol, [&](const P2& q) { return !blockedAt(q.x, q.y); });
        refineChords(l, g.c, c.tol, [&](const P2& q) { return c.cl(q.x, q.y, floor) > level + c.tol; });
    }
    return loops;
}

std::vector<double> depthLevels(double top, double bottom, double step, const std::vector<double>& extra = {}) {
    std::vector<double> z;
    if (!(top > bottom + 1e-9)) return z;
    const int n = std::max(1, static_cast<int>(std::ceil((top - bottom) / step - 1e-6)));
    for (int k = 1; k <= n; ++k) z.push_back(top - (top - bottom) * k / n);
    for (double e : extra)
        if (e < top - 1e-6 && e > bottom + 1e-6) z.push_back(e);
    std::sort(z.begin(), z.end(), std::greater<double>());
    z.erase(std::unique(z.begin(), z.end(), [](double a, double b) { return std::fabs(a - b) < 0.01; }), z.end());
    return z;
}

// Heights of the part's upward flat faces: clearing levels stop on them exactly.
std::vector<double> flatLevels(const PartModel& part, double leave) {
    std::vector<double> z;
    for (std::size_t i = 0; i < part.tris.size(); ++i)
        if (part.normal[i].z > 0.9999) z.push_back(part.p[part.tris[i][0]].z + leave);
    std::sort(z.begin(), z.end());
    z.erase(std::unique(z.begin(), z.end(), [](double a, double b) { return std::fabs(a - b) < 0.01; }), z.end());
    return z;
}

double defaultStepdown(const Ctx& c) {
    const CamMaterial* m = camMaterial(c.job.setup.material);
    return std::max(0.1, std::min(c.tool.fluteLength * 0.9, c.D() * (m ? m->stepdownFactor : 0.5)));
}

// Labels 4-connected nodes where in[k]; returns the label count.
int labelRegions(const Grid& g, const std::vector<char>& in, std::vector<int>& label) {
    label.assign(in.size(), -1);
    int count = 0;
    std::vector<int> stack;
    for (int s = 0; s < static_cast<int>(in.size()); ++s) {
        if (!in[static_cast<std::size_t>(s)] || label[static_cast<std::size_t>(s)] >= 0) continue;
        stack.push_back(s);
        label[static_cast<std::size_t>(s)] = count;
        while (!stack.empty()) {
            const int k = stack.back();
            stack.pop_back();
            const int i = k % g.nx, j = k / g.nx;
            const int nb[4][2] = {{i + 1, j}, {i - 1, j}, {i, j + 1}, {i, j - 1}};
            for (const auto& q : nb) {
                if (q[0] < 0 || q[1] < 0 || q[0] >= g.nx || q[1] >= g.ny) continue;
                const int kk = q[1] * g.nx + q[0];
                if (!in[static_cast<std::size_t>(kk)] || label[static_cast<std::size_t>(kk)] >= 0) continue;
                label[static_cast<std::size_t>(kk)] = count;
                stack.push_back(kk);
            }
        }
        ++count;
    }
    return count;
}

// The region a node near q belongs to.
int regionAt(const Grid& g, const std::vector<int>& label, const P2& q) {
    const int i0 = static_cast<int>(std::floor((q.x - g.x0) / g.c)), j0 = static_cast<int>(std::floor((q.y - g.y0) / g.c));
    int best = -1;
    double bd = kInf;
    for (int j = j0 - 1; j <= j0 + 2; ++j)
        for (int i = i0 - 1; i <= i0 + 2; ++i) {
            if (i < 0 || j < 0 || i >= g.nx || j >= g.ny) continue;
            const int l = label[static_cast<std::size_t>(j * g.nx + i)];
            if (l < 0) continue;
            if (const double d = dist2(g.pos(i, j), q); d < bd) bd = d, best = l;
        }
    return best;
}

// Pocket and 3D roughing: clears each depth, from the middle of each region outwards
// in rings a stepover apart (or zig-zag), and finishes the region's edge.
bool genClearing(Ctx& c, bool pocketsOnly, std::string* err) {
    const CamOperation& op = c.op;
    const double D = c.D(), R = c.R();
    const double s = std::clamp(op.stepover > 0 ? op.stepover : D * (op.adaptive ? 0.12 : 0.4), D * 0.05, D * 0.95);
    const double sd = op.stepdown > 0 ? op.stepdown : op.adaptive ? std::max(0.1, std::min(c.tool.fluteLength * 0.9, 2.0 * D)) : defaultStepdown(c);
    const double top = std::isnan(op.top) ? c.stockTop : op.top;
    const double bottom = std::isnan(op.bottom) ? std::max(c.part.z0, c.stockBottom) : op.bottom;
    const std::vector<double> levels = depthLevels(top, bottom, sd, flatLevels(c.part, op.leave));
    if (levels.empty()) {
        if (err) *err = "Nothing to clear between the top and the bottom";
        return false;
    }
    const double floor = bottom - 10.0;
    const double ex = R * 0.95; // the tool's centre may go this far past the stock's edge
    const double cell = std::clamp(std::min(R / 4.0, s / 3.0), 0.05, 2.0);
    const Grid g = clGrid(c.part, c.cut, c.sx0 - ex - 3 * cell, c.sy0 - ex - 3 * cell, c.sx1 + ex + 3 * cell, c.sy1 + ex + 3 * cell, cell,
                          floor - op.leave);
    Grid clg = g;
    for (double& v : clg.v) v += op.leave;
    // The steps a depth leaves over the part stand up to a stepdown above it: rapids clear them.
    c.approach = std::max(c.approach, sd + op.leave + 1.0);
    auto inRect = [&](double x, double y) { return x >= c.sx0 - ex && x <= c.sx1 + ex && y >= c.sy0 - ex && y <= c.sy1 + ex; };
    auto inStock = [&](double x, double y) { return x >= c.sx0 && x <= c.sx1 && y >= c.sy0 && y <= c.sy1; };
    PathOut o(c.op.moves, c);
    double materialTop = c.stockTop;
    // The stock the operations before left (rest machining), at a node.
    const CamSimulation* rs = c.rest;
    auto restHeight = [&](const P2& q, double* partTop) {
        const int i = static_cast<int>(std::floor((q.x - c.sx0) / rs->cell)), j = static_cast<int>(std::floor((q.y - c.sy0) / rs->cell));
        if (i < 0 || j < 0 || i >= rs->nx || j >= rs->ny) {
            *partTop = -kInf;
            return -kInf;
        }
        const std::size_t k = static_cast<std::size_t>(j) * static_cast<std::size_t>(rs->nx) + static_cast<std::size_t>(i);
        *partTop = k < rs->partTop.size() ? rs->partTop[k] : -kInf;
        return static_cast<double>(rs->height[k]);
    };
    const double restMin = std::max(0.08, 4.0 * c.tol); // thinner than this isn't worth a pass
    // Adaptive: the part's top at each node (the material at a level is the stock above it).
    Grid partTop;
    if (op.adaptive) {
        partTop = g;
        parallelFor(g.ny, [&](int j) {
            for (int i = 0; i < g.nx; ++i) {
                const P2 q = g.pos(i, j);
                partTop.at(i, j) = c.part.top(q.x, q.y);
            }
        });
    }
    for (const double z : levels) {
        std::vector<char> in(g.v.size());
        for (int j = 0; j < g.ny; ++j)
            for (int i = 0; i < g.nx; ++i) {
                const P2 q = g.pos(i, j);
                in[static_cast<std::size_t>(j * g.nx + i)] = clg.at(i, j) <= z + 1e-9 && inRect(q.x, q.y);
            }
        // Rest machining: only where the tool can reach material left in this level's slab
        // (between this depth and the one above, above the part and its stock to leave).
        std::vector<char> restMat;
        if (rs) {
            restMat.assign(g.v.size(), 0);
            std::vector<char> clear(g.v.size(), 1);
            bool any = false;
            for (int j = 0; j < g.ny; ++j)
                for (int i = 0; i < g.nx; ++i) {
                    double pt;
                    const double h = restHeight(g.pos(i, j), &pt);
                    const double lo = std::max(pt + op.leave, z), hi = std::min(h, materialTop);
                    const std::size_t k = static_cast<std::size_t>(j * g.nx + i);
                    if (hi - lo > restMin) restMat[k] = 1, clear[k] = 0, any = true;
                }
            if (any) {
                const Grid toMat = distanceInside(g, clear);
                for (std::size_t k = 0; k < in.size(); ++k)
                    if (toMat.v[k] > R + 1.5 * cell) in[k] = 0;
            } else {
                std::fill(in.begin(), in.end(), 0);
            }
        }
        std::vector<int> label;
        int regions = labelRegions(g, in, label);
        if (pocketsOnly) { // only regions closed in by the part: drop those reaching outside the stock
            std::vector<char> open(static_cast<std::size_t>(regions), 0);
            for (int j = 0; j < g.ny; ++j)
                for (int i = 0; i < g.nx; ++i) {
                    const int l = label[static_cast<std::size_t>(j * g.nx + i)];
                    const P2 q = g.pos(i, j);
                    if (l >= 0 && !inStock(q.x, q.y)) open[static_cast<std::size_t>(l)] = 1;
                }
            for (std::size_t k = 0; k < in.size(); ++k)
                if (label[k] >= 0 && open[static_cast<std::size_t>(label[k])]) in[k] = 0;
            regions = labelRegions(g, in, label);
        }
        if (regions == 0) {
            materialTop = z;
            continue;
        }
        auto inside = [&](int i, int j) { return in[static_cast<std::size_t>(j * g.nx + i)] != 0; };
        auto freeAt = [&](double x, double y) {
            if (!inRect(x, y)) return false;
            if (pocketsOnly && regionAt(g, label, {x, y}) < 0) return false;
            return c.cl(x, y, floor) <= z + 1e-9;
        };
        // The region's edge, exact against the part and the stock outline.
        std::vector<Loop> edge = isoLoops(
            g, inside,
            [&](const P2& a, const P2& b) {
                P2 lo = a, hi = b;
                for (int k = 0; k < 20; ++k) {
                    const P2 h{(lo.x + hi.x) * 0.5, (lo.y + hi.y) * 0.5};
                    (freeAt(h.x, h.y) ? lo : hi) = h;
                }
                return lo;
            },
            freeAt);
        for (Loop& l : edge) {
            l = simplifyLoop(l, c.tol * 0.5);
            sharpenCorners(l, g.c, c.tol, [&](const P2& q) { return freeAt(q.x, q.y); });
            // The free side is on the left here: refine toward the left.
            std::reverse(l.begin(), l.end());
            refineChords(l, g.c, c.tol, [&](const P2& q) { return inRect(q.x, q.y) && c.cl(q.x, q.y, floor) > z + c.tol; });
            std::reverse(l.begin(), l.end());
        }
        // Rings inside, a stepover apart, from a distance field.
        struct Ring {
            Loop loop;
            double level;
            int region;
        };
        std::vector<Ring> rings;
        for (Loop& l : edge) {
            if (l.size() < 3) continue;
            const int region = regionAt(g, label, l[0]);
            rings.push_back({std::move(l), 0.0, region});
        }
        if (!op.raster || op.adaptive) {
            const Grid dist = distanceInside(g, in);
            double dmax = 0;
            for (double v : dist.v) dmax = std::max(dmax, v);
            for (int k = 1; k * s - 0.5 * cell < dmax; ++k) {
                const double lv = k * s - 0.5 * cell;
                std::vector<Loop> ls = isoLoops(
                    dist, [&](int i, int j) { return dist.at(i, j) > lv; },
                    [&](const P2& a, const P2& b) {
                        const double va = dist.sample(a.x, a.y), vb = dist.sample(b.x, b.y);
                        const double t = std::clamp((va - lv) / std::max(va - vb, 1e-12), 0.0, 1.0);
                        return P2{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
                    },
                    [&](double x, double y) { return dist.sample(x, y) > lv; });
                for (Loop& l : ls) {
                    smoothLoop(l, 2);
                    l = simplifyLoop(l, c.tol);
                    if (l.size() < 3) continue;
                    const int region = regionAt(g, label, l[0]);
                    rings.push_back({std::move(l), lv, region});
                }
            }
        }
        if (!op.climb)
            for (Ring& r : rings) std::reverse(r.loop.begin(), r.loop.end());
        // Adaptive: the material at this level, and what the tool has cleared of it so far.
        // `straight` is what the rings alone would have cleared: where going straight on
        // would overload the tool, it loops instead, a stepover further each time.
        std::vector<char> mat, cleared, straight;
        if (op.adaptive) {
            mat.assign(g.v.size(), 0);
            cleared.assign(g.v.size(), 0);
            straight.assign(g.v.size(), 0);
            for (int j = 0; j < g.ny; ++j)
                for (int i = 0; i < g.nx; ++i) {
                    const std::size_t k = static_cast<std::size_t>(j * g.nx + i);
                    const P2 q = g.pos(i, j);
                    mat[k] = inStock(q.x, q.y) && partTop.at(i, j) < z - 1e-6 && (!rs || restMat[k]);
                }
        }
        auto stamp = [&](const P2& a, const P2& b, double r, bool alsoStraight) { // clears the nodes within r of the segment
            const int i0 = std::max(0, static_cast<int>(std::floor((std::min(a.x, b.x) - r - g.x0) / g.c)));
            const int i1 = std::min(g.nx - 1, static_cast<int>(std::ceil((std::max(a.x, b.x) + r - g.x0) / g.c)));
            const int j0 = std::max(0, static_cast<int>(std::floor((std::min(a.y, b.y) - r - g.y0) / g.c)));
            const int j1 = std::min(g.ny - 1, static_cast<int>(std::ceil((std::max(a.y, b.y) + r - g.y0) / g.c)));
            const double dx = b.x - a.x, dy = b.y - a.y, l2 = dx * dx + dy * dy;
            for (int j = j0; j <= j1; ++j)
                for (int i = i0; i <= i1; ++i) {
                    const P2 q = g.pos(i, j);
                    const double t = l2 > 0 ? std::clamp(((q.x - a.x) * dx + (q.y - a.y) * dy) / l2, 0.0, 1.0) : 0.0;
                    if (std::hypot(q.x - a.x - dx * t, q.y - a.y - dy * t) <= r) {
                        cleared[static_cast<std::size_t>(j * g.nx + i)] = 1;
                        if (alsoStraight) straight[static_cast<std::size_t>(j * g.nx + i)] = 1;
                    }
                }
        };
        // The angle of the tool's edge in uncut material ahead of it at p going along t.
        constexpr int kEdge = 48;
        auto engagement = [&](const P2& p, const P2& t, const std::vector<char>& done) {
            int n = 0;
            for (int k = 0; k < kEdge; ++k) {
                const double a = 2 * kPiD * k / kEdge, ux = std::cos(a), uy = std::sin(a);
                if (ux * t.x + uy * t.y < 0) continue;
                const int i = static_cast<int>(std::lround((p.x + ux * R - g.x0) / g.c)), j = static_cast<int>(std::lround((p.y + uy * R - g.y0) / g.c));
                if (i < 0 || j < 0 || i >= g.nx || j >= g.ny) continue;
                const std::size_t q = static_cast<std::size_t>(j * g.nx + i);
                if (mat[q] && !done[q]) ++n;
            }
            return 2 * kPiD * n / kEdge;
        };
        const double nominal = std::acos(1.0 - std::min(s, D) / R);
        const double limit = std::max(nominal * 1.5, nominal + 15.0 * kPiD / 180.0);
        P2 stamped{};
        auto feedA = [&](double x, double y, double zz, double f) { // a feed that clears as it goes
            const P2 from{o.at.x, o.at.y};
            if (f > 0) o.feed(x, y, zz, f);
            else o.feed(x, y, zz);
            stamp(from, {x, y}, R, true);
            stamped = {x, y};
        };
        // A trochoidal loop at p heading along t: a circle of radius rt ahead of the tool,
        // round and back (counter-clockwise for climb milling), shrunk until it misses the part.
        auto trochoid = [&](const P2& p, const P2& t, double maxRt) {
            const P2 n{-t.y, t.x};
            const double side = op.climb ? 1.0 : -1.0;
            for (double rt = std::min(0.5 * R, maxRt); rt >= 0.05 * R; rt *= 0.5) {
                constexpr int kSeg = 20;
                std::vector<P2> pts;
                bool fits = true;
                for (int k = 1; k <= kSeg && fits; ++k) {
                    const double a = 2 * kPiD * k / kSeg;
                    const P2 q{p.x + t.x * rt * (1 - std::cos(a)) - n.x * rt * std::sin(a) * side,
                               p.y + t.y * rt * (1 - std::cos(a)) - n.y * rt * std::sin(a) * side};
                    if (!inRect(q.x, q.y) || c.cl(q.x, q.y, floor) > z + 1e-9) fits = false;
                    pts.push_back(q);
                }
                if (!fits) continue;
                for (const P2& q : pts) o.feed(q.x, q.y, z);
                stamp({p.x + t.x * rt, p.y + t.y * rt}, {p.x + t.x * rt, p.y + t.y * rt}, R + rt, false);
                return true;
            }
            return false;
        };
        // Round a loop from its first point (where the tool is), looping trochoidally
        // wherever going straight on would take more than the limit.
        auto adaptiveRound = [&](const Loop& l) {
            const std::size_t n = l.size();
            const double adv = std::max(s * 0.5, g.c);
            P2 cur = l[0];
            stamped = cur;
            double sinceLoop = kInf;
            int run = 0; // loops in a row: the first ones are smaller, so the run eases in
            const double loopStep = s * 0.5; // each loop takes a crescent this thick
            for (std::size_t i = 1; i <= n; ++i) {
                const P2 target = l[i % n];
                const double len = dist2(cur, target);
                if (len < 1e-9) continue;
                const P2 t{(target.x - cur.x) / len, (target.y - cur.y) / len};
                for (double d = 0; d < len - 1e-9;) {
                    const double step = std::min(adv, len - d);
                    const P2 at{cur.x + t.x * d, cur.y + t.y * d}, next{cur.x + t.x * (d + step), cur.y + t.y * (d + step)};
                    // Loop where straight on would overload the tool, one stepover past the
                    // last loop, while there is material ahead of the loop.
                    const bool overloaded = engagement(next, t, straight) > limit;
                    if (!overloaded) run = 0;
                    if (overloaded && sinceLoop >= loopStep - 1e-9 && engagement({at.x + t.x * R, at.y + t.y * R}, t, cleared) > 1e-9) {
                        if (dist2(at, {o.at.x, o.at.y}) > 1e-9) feedA(at.x, at.y, z, -1);
                        if (trochoid(at, t, loopStep * (run + 1))) sinceLoop = 0, ++run;
                        o.feed(at.x, at.y, z);
                    }
                    stamp(stamped, next, R, true);
                    stamped = next;
                    sinceLoop += step;
                    d += step;
                }
                feedA(target.x, target.y, z, -1);
                cur = target;
            }
        };
        auto adaptiveLoop = [&](const Loop& l, double from) {
            const std::size_t n = l.size();
            if (n < 2) return;
            if (from > z + 1e-9) {
                const double need = (from - z) / std::tan(kRamp), len = loopLength(l);
                if (len < D * 0.5) {
                    o.plunge(l[0].x, l[0].y, z);
                } else {
                    double done = 0, zz = from;
                    o.plunge(l[0].x, l[0].y, from);
                    for (std::size_t i = 1; zz > z + 1e-9 && i < n * 64; ++i) {
                        const P2& a = l[(i - 1) % n];
                        const P2& b = l[i % n];
                        done += dist2(a, b);
                        zz = std::max(z, from - (from - z) * done / need);
                        feedA(b.x, b.y, zz, c.feeds.plunge * 2.0);
                    }
                    adaptiveRound(startNear(l, {o.at.x, o.at.y}));
                    return;
                }
            }
            adaptiveRound(l);
        };
        // Region by region (nearest first), innermost ring first.
        std::vector<char> regionDone(static_cast<std::size_t>(regions), 0);
        for (int left = regions; left > 0; --left) {
            const P2 here{o.started ? o.at.x : c.sx0, o.started ? o.at.y : c.sy0};
            int region = -1;
            double bd = kInf;
            for (const Ring& r : rings)
                if (r.region >= 0 && !regionDone[static_cast<std::size_t>(r.region)])
                    if (const double d = nearestDistance(r.loop, here); d < bd) bd = d, region = r.region;
            if (region < 0) break;
            regionDone[static_cast<std::size_t>(region)] = 1;
            std::vector<const Ring*> mine;
            for (const Ring& r : rings)
                if (r.region == region) mine.push_back(&r);
            std::stable_sort(mine.begin(), mine.end(), [](const Ring* a, const Ring* b) { return a->level > b->level; });
            bool entered = false;
            auto cutRing = [&](const Loop& ring) {
                const P2 cur{o.at.x, o.at.y};
                const Loop l = startNear(ring, cur);
                if (entered && dist2(cur, l[0]) <= 2.0 * s && o.freeLine(cur, l[0], z, floor) && std::fabs(o.at.z - z) < 1e-6) {
                    if (op.adaptive) {
                        feedA(l[0].x, l[0].y, z, -1);
                        adaptiveLoop(l, z);
                    } else {
                        o.feed(l[0].x, l[0].y, z);
                        cutLoop(o, l, z, z);
                    }
                } else {
                    // Rapid only to just above the last depth (the tool may overlap stock
                    // not yet cut at this one), then ramp in along the ring.
                    o.travel(l[0], materialTop + 1.0, materialTop + 0.5, floor);
                    if (op.adaptive) adaptiveLoop(l, materialTop + 0.5);
                    else cutLoop(o, l, z, materialTop + 0.5);
                }
                entered = true;
            };
            if (op.raster && !op.adaptive) {
                std::vector<Loop> edgeLoops;
                for (const Ring* r : mine) edgeLoops.push_back(r->loop);
                const auto lines = section::hatch(edgeLoops, s, op.angle * kPiD / 180.0);
                for (const auto& [a, b] : lines) {
                    const P2 cur{o.at.x, o.at.y};
                    if (entered && dist2(cur, a) <= 2.0 * s && o.freeLine(cur, a, z, floor) && std::fabs(o.at.z - z) < 1e-6) {
                        o.feed(a.x, a.y, z);
                    } else {
                        o.travel(a, materialTop + 1.0, materialTop + 0.5, floor);
                        o.plunge(a.x, a.y, z);
                    }
                    o.feed(b.x, b.y, z);
                    entered = true;
                }
                for (const Ring* r : mine) cutRing(r->loop); // clean the edge
            } else {
                for (const Ring* r : mine) cutRing(r->loop);
            }
        }
        materialTop = z;
    }
    o.end();
    return true;
}

// Lead arcs (a quarter circle on the free side), tabs, and the loops of each depth.
bool genContour(Ctx& c, std::string* err) {
    const CamOperation& op = c.op;
    const double R = c.R();
    const double sd = op.stepdown > 0 ? op.stepdown : defaultStepdown(c);
    const double top = std::isnan(op.top) ? c.stockTop : op.top;
    const double bottom = std::isnan(op.bottom) ? c.part.z0 : op.bottom;
    const std::vector<double> levels = depthLevels(top, bottom, sd);
    if (levels.empty()) {
        if (err) *err = "The contour's bottom must be below its top";
        return false;
    }
    const double lead = op.lead >= 0 ? op.lead : R * 0.5;
    const double floor = bottom - 10.0;
    const double pad = R + lead + 2.0;
    const double cell = std::clamp(R / 4.0, 0.05, 1.5);
    Grid g = clGrid(c.part, c.cut, c.part.x0 - pad, c.part.y0 - pad, c.part.x1 + pad, c.part.y1 + pad, cell, floor - op.leave);
    for (double& v : g.v) v += op.leave;
    const double tabTop = bottom + op.tabHeight;
    PathOut o(c.op.moves, c);
    double prev = top;
    for (const double z : levels) {
        std::vector<Loop> loops = levelLoops(c, g, z, floor);
        std::vector<std::pair<Loop, bool>> todo; // loop, outside
        for (Loop& l : loops) {
            const bool outside = section::loopArea(l) > 0;
            if ((op.side == 0 && !outside) || (op.side == 1 && outside)) continue;
            if (op.climb) std::reverse(l.begin(), l.end()); // the part on the right
            todo.push_back({std::move(l), outside});
        }
        while (!todo.empty()) {
            const P2 here{o.started ? o.at.x : c.sx0, o.started ? o.at.y : c.sy0};
            std::size_t pick = 0;
            double bd = kInf;
            for (std::size_t k = 0; k < todo.size(); ++k)
                if (const double d = nearestDistance(todo[k].first, here); d < bd) bd = d, pick = k;
            const auto [loop0, outside] = todo[pick];
            todo.erase(todo.begin() + static_cast<std::ptrdiff_t>(pick));
            const Loop l = startNear(loop0, here);
            // Lead in and out on the free side (left of the direction of travel).
            const P2 p0 = l[0], p1 = l[1 % l.size()];
            const double tl = dist2(p0, p1);
            const P2 t{(p1.x - p0.x) / tl, (p1.y - p0.y) / tl}, nl{-t.y, t.x};
            auto arc = [&](double phi) {
                const P2 cc{p0.x + nl.x * lead, p0.y + nl.y * lead};
                return P2{cc.x + lead * (std::cos(phi) * -nl.x + std::sin(phi) * t.x), cc.y + lead * (std::cos(phi) * -nl.y + std::sin(phi) * t.y)};
            };
            const P2 a = arc(-kPiD / 2), b = arc(kPiD / 2);
            const bool arcs = lead > 1e-6 && c.cl(a.x, a.y, floor) <= z + c.tol && c.cl(b.x, b.y, floor) <= z + c.tol &&
                              c.cl(arc(-kPiD / 4).x, arc(-kPiD / 4).y, floor) <= z + c.tol;
            const P2 start = arcs ? a : p0;
            o.travel(start, c.stockTop + 1.0, prev + 1.0, floor);
            o.plunge(start.x, start.y, z);
            if (arcs)
                for (int k = 1; k <= 8; ++k) {
                    const P2 q = arc(-kPiD / 2 + kPiD / 2 * k / 8.0);
                    o.feed(q.x, q.y, z);
                }
            // Round the loop, over the tabs on the outside profile's last depths.
            const bool tabbed = outside && op.tabs > 0 && z < tabTop - 1e-9;
            const double len = loopLength(l), span = op.tabWidth + c.D();
            auto zAt = [&](double s) {
                if (!tabbed) return z;
                for (int k = 0; k < op.tabs; ++k) {
                    const double mid = (k + 0.5) * len / op.tabs;
                    if (std::fabs(s - mid) <= span * 0.5) return tabTop;
                }
                return z;
            };
            std::vector<double> breaks;
            if (tabbed)
                for (int k = 0; k < op.tabs; ++k) {
                    const double mid = (k + 0.5) * len / op.tabs;
                    breaks.push_back(mid - span * 0.5);
                    breaks.push_back(mid + span * 0.5);
                }
            double s = 0;
            double zNow = zAt(0);
            o.feed(l[0].x, l[0].y, zNow);
            for (std::size_t i = 1; i <= l.size(); ++i) {
                const P2& pa = l[i - 1];
                const P2& pb = l[i % l.size()];
                const double seg = dist2(pa, pb);
                for (double br : breaks)
                    if (br > s + 1e-9 && br < s + seg - 1e-9) {
                        const double f = (br - s) / seg;
                        const P2 q{pa.x + (pb.x - pa.x) * f, pa.y + (pb.y - pa.y) * f};
                        o.feed(q.x, q.y, zNow);
                        zNow = zAt(br + 1e-6);
                        o.feed(q.x, q.y, zNow);
                    }
                s += seg;
                zNow = zAt(std::min(s, len - 1e-6));
                o.feed(pb.x, pb.y, zNow);
            }
            o.feed(l[0].x, l[0].y, z);
            if (arcs)
                for (int k = 1; k <= 8; ++k) {
                    const P2 q = arc(kPiD / 2 * k / 8.0);
                    o.feed(q.x, q.y, z);
                }
        }
        prev = z;
    }
    o.end();
    return true;
}

// Facing: zig-zag over the stock, down to the top of the part.
bool genFace(Ctx& c, std::string* err) {
    const CamOperation& op = c.op;
    const double R = c.R(), D = c.D();
    const double s = std::clamp(op.stepover > 0 ? op.stepover : D * 0.7, D * 0.05, D * 0.95);
    const double top = std::isnan(op.top) ? c.stockTop : op.top;
    const double bottom = std::isnan(op.bottom) ? c.part.z1 + op.leave : op.bottom;
    const double sd = op.stepdown > 0 ? op.stepdown : std::min(1.0, defaultStepdown(c));
    const std::vector<double> levels = depthLevels(top, bottom, sd);
    if (levels.empty()) {
        if (err) *err = "Nothing to face: the stock's top is at the part's top";
        return false;
    }
    const double a = op.angle * kPiD / 180.0, ca = std::cos(a), sa = std::sin(a);
    // The stock in the facing frame: u along the passes, v across.
    double u0 = kInf, u1 = -kInf, v0 = kInf, v1 = -kInf;
    for (const P2& q : {P2{c.sx0, c.sy0}, P2{c.sx1, c.sy0}, P2{c.sx1, c.sy1}, P2{c.sx0, c.sy1}}) {
        const double u = q.x * ca + q.y * sa, v = -q.x * sa + q.y * ca;
        u0 = std::min(u0, u), u1 = std::max(u1, u), v0 = std::min(v0, v), v1 = std::max(v1, v);
    }
    const double ua = u0 - R - 2.0, ub = u1 + R + 2.0;
    std::vector<double> vs;
    const double first = v0 + std::min(R * 0.8, (v1 - v0) * 0.5), last = v1 - std::min(R * 0.8, (v1 - v0) * 0.5);
    const int n = std::max(1, static_cast<int>(std::ceil((last - first) / s - 1e-9)));
    for (int k = 0; k <= n; ++k) vs.push_back(first + (last - first) * k / n);
    if (last - first < 1e-9) vs.assign(1, (v0 + v1) * 0.5);
    auto xy = [&](double u, double v) { return P2{u * ca - v * sa, u * sa + v * ca}; };
    PathOut o(c.op.moves, c);
    for (const double z : levels) {
        bool forward = true;
        const P2 s0 = xy(ua, vs[0]);
        o.travel(s0, c.stockTop + 1.0, c.stockTop + 1.0, z - 10);
        o.plunge(s0.x, s0.y, z);
        for (std::size_t k = 0; k < vs.size(); ++k) {
            const P2 e = xy(forward ? ub : ua, vs[k]);
            o.feed(e.x, e.y, z);
            if (k + 1 < vs.size()) {
                const P2 nx = xy(forward ? ub : ua, vs[k + 1]);
                o.feed(nx.x, nx.y, z);
            }
            forward = !forward;
        }
    }
    o.end();
    return true;
}

// The stepover for a cusp of `scallop` between passes of the tool's tip radius.
double scallopStep(const CamTool& t, double scallop) {
    const double r = t.tipRadius();
    if (r <= 0) return t.diameter * 0.1;
    const double h = std::clamp(scallop, 1e-4, r);
    return 2.0 * std::sqrt(h * (2.0 * r - h)) + (t.radius() - r) * 2.0 * 0.9;
}

// Slope (degrees) of the cutter-height field at (x, y).
double slopeAt(const Grid& g, double x, double y) {
    const double e = g.c;
    const double dx = (g.sample(x + e, y) - g.sample(x - e, y)) / (2 * e), dy = (g.sample(x, y + e) - g.sample(x, y - e)) / (2 * e);
    return std::atan(std::hypot(dx, dy)) * 180.0 / kPiD;
}

// Parallel finishing: passes across the part following its surface.
bool genParallel(Ctx& c, std::string* err) {
    const CamOperation& op = c.op;
    const double R = c.R();
    const double s = op.stepover > 0 ? op.stepover : scallopStep(c.tool, op.scallop);
    if (!(s > 1e-3)) {
        if (err) *err = "The stepover is too small";
        return false;
    }
    const double floor = c.part.z0 - 10.0;
    const double a = op.angle * kPiD / 180.0, ca = std::cos(a), sa = std::sin(a);
    const double x0 = std::max(c.part.x0 - R, c.sx0 - R), x1 = std::min(c.part.x1 + R, c.sx1 + R);
    const double y0 = std::max(c.part.y0 - R, c.sy0 - R), y1 = std::min(c.part.y1 + R, c.sy1 + R);
    const bool slopes = op.minSlope > 0 || op.maxSlope < 90;
    Grid g;
    if (slopes) {
        g = clGrid(c.part, c.cut, x0, y0, x1, y1, std::clamp(R / 3.0, 0.1, 2.0), floor - op.leave);
        for (double& v : g.v) v += op.leave;
    }
    double u0 = kInf, u1 = -kInf, v0 = kInf, v1 = -kInf;
    for (const P2& q : {P2{x0, y0}, P2{x1, y0}, P2{x1, y1}, P2{x0, y1}}) {
        const double u = q.x * ca + q.y * sa, v = -q.x * sa + q.y * ca;
        u0 = std::min(u0, u), u1 = std::max(u1, u), v0 = std::min(v0, v), v1 = std::max(v1, v);
    }
    auto xy = [&](double u, double v) { return P2{u * ca - v * sa, u * sa + v * ca}; };
    const double step = std::clamp(s * 0.5, 0.05, 0.5);
    const double onPart = c.part.z0 + 1e-3;
    PathOut o(c.op.moves, c);
    bool forward = true;
    struct Pt {
        P2 q;
        double z;
    };
    const int lines = static_cast<int>(std::floor((v1 - v0) / s)) + 1;
    for (int k = 0; k < lines; ++k) {
        const double v = v0 + k * s + std::fmod(v1 - v0, s) * 0.5;
        // Samples along the pass, split where the tool leaves the part or the slope range.
        const int n = std::max(1, static_cast<int>(std::ceil((u1 - u0) / step)));
        std::vector<std::vector<Pt>> runs(1);
        for (int i = 0; i <= n; ++i) {
            const double u = forward ? u0 + (u1 - u0) * i / n : u1 - (u1 - u0) * i / n;
            const P2 q = xy(u, v);
            const double z = c.cl(q.x, q.y, floor);
            bool keep = z > onPart;
            if (keep && slopes) {
                const double sl = slopeAt(g, q.x, q.y);
                keep = sl >= op.minSlope - 1e-9 && sl <= op.maxSlope + 1e-9;
            }
            if (keep) runs.back().push_back({q, z});
            else if (!runs.back().empty()) runs.emplace_back();
        }
        // Midpoints that a straight move would cut into: added, to depth 5.
        auto refineRun = [&](std::vector<Pt>& run) {
            for (int pass = 0; pass < 5; ++pass) {
                std::vector<Pt> out{run[0]};
                bool more = false;
                for (std::size_t i = 1; i < run.size(); ++i) {
                    const Pt& pa = run[i - 1];
                    const Pt& pb = run[i];
                    const P2 m{(pa.q.x + pb.q.x) * 0.5, (pa.q.y + pb.q.y) * 0.5};
                    if (dist2(pa.q, pb.q) > 0.005) {
                        const double zm = c.cl(m.x, m.y, floor);
                        if (zm > (pa.z + pb.z) * 0.5 + c.tol * 0.5) {
                            out.push_back({m, zm});
                            more = true;
                        }
                    }
                    out.push_back(pb);
                }
                run = std::move(out);
                if (!more) break;
            }
        };
        for (auto& run : runs) {
            if (run.size() < 2) continue;
            refineRun(run);
            // Link from the last pass along the surface when it is close, else lift.
            const P2 cur{o.at.x, o.at.y};
            if (o.started && dist2(cur, run[0].q) <= 2.0 * s && o.at.z < c.retractZ - 1e-6) {
                // Over the surface to the next pass, refined like the passes are.
                const int m = std::max(1, static_cast<int>(std::ceil(dist2(cur, run[0].q) / step)));
                std::vector<Pt> link{{cur, o.at.z}};
                for (int i = 1; i < m; ++i) {
                    const P2 q{cur.x + (run[0].q.x - cur.x) * i / m, cur.y + (run[0].q.y - cur.y) * i / m};
                    link.push_back({q, c.cl(q.x, q.y, floor)});
                }
                link.push_back(run[0]);
                refineRun(link);
                for (std::size_t i = 1; i < link.size(); ++i) o.feed(link[i].q.x, link[i].q.y, link[i].z);
            } else {
                o.travel(run[0].q, run[0].z + c.approach, run[0].z + c.approach, floor);
                o.plunge(run[0].q.x, run[0].q.y, run[0].z);
            }
            // Points in a straight line (within half the tolerance, in 3D) go.
            std::vector<char> keep(run.size(), 0);
            keep.front() = keep.back() = 1;
            std::vector<std::pair<std::size_t, std::size_t>> stack{{0, run.size() - 1}};
            while (!stack.empty()) {
                const auto [ia, ib] = stack.back();
                stack.pop_back();
                const Pt &pa = run[ia], &pb = run[ib];
                const double dx = pb.q.x - pa.q.x, dy = pb.q.y - pa.q.y, dz = pb.z - pa.z;
                const double len2 = dx * dx + dy * dy + dz * dz;
                double worst = -1;
                std::size_t wi = ia;
                for (std::size_t i = ia + 1; i < ib; ++i) {
                    const double ex = run[i].q.x - pa.q.x, ey = run[i].q.y - pa.q.y, ez = run[i].z - pa.z;
                    const double t = len2 > 0 ? std::clamp((ex * dx + ey * dy + ez * dz) / len2, 0.0, 1.0) : 0.0;
                    const double d = std::sqrt((ex - dx * t) * (ex - dx * t) + (ey - dy * t) * (ey - dy * t) + (ez - dz * t) * (ez - dz * t));
                    if (d > worst) worst = d, wi = i;
                }
                if (worst > c.tol * 0.5) {
                    keep[wi] = 1;
                    stack.push_back({ia, wi});
                    stack.push_back({wi, ib});
                }
            }
            for (std::size_t i = 1; i < run.size(); ++i)
                if (keep[i]) o.feed(run[i].q.x, run[i].q.y, run[i].z);
        }
        forward = !forward;
    }
    o.end();
    if (o.moves.empty() && err) *err = "The tool never touches the part";
    return !o.moves.empty();
}

// Waterline finishing: the loops where the tool touches the part, level by level.
bool genWaterline(Ctx& c, std::string* err) {
    const CamOperation& op = c.op;
    const double R = c.R();
    const double sd = op.stepdown > 0 ? op.stepdown : (op.stepover > 0 ? op.stepover : scallopStep(c.tool, op.scallop));
    const double top = std::isnan(op.top) ? c.part.z1 : op.top;
    const double bottom = std::isnan(op.bottom) ? c.part.z0 : op.bottom;
    std::vector<double> levels = depthLevels(top, bottom, std::max(sd, 0.01));
    if (!levels.empty() && levels.back() <= c.part.z0 + 1e-6) levels.back() = c.part.z0 + std::min(0.01, sd * 0.1);
    if (levels.empty()) {
        if (err) *err = "The waterline's bottom must be below its top";
        return false;
    }
    const double floor = c.part.z0 - 10.0;
    const double pad = R + 2.0;
    Grid g = clGrid(c.part, c.cut, std::max(c.part.x0, c.sx0) - pad, std::max(c.part.y0, c.sy0) - pad, std::min(c.part.x1, c.sx1) + pad,
                    std::min(c.part.y1, c.sy1) + pad, std::clamp(R / 4.0, 0.05, 1.5), floor - op.leave);
    for (double& v : g.v) v += op.leave;
    const bool slopes = op.minSlope > 0 || op.maxSlope < 90;
    PathOut o(c.op.moves, c);
    double prev = top;
    for (const double z : levels) {
        std::vector<Loop> loops = levelLoops(c, g, z, floor);
        std::vector<std::vector<P2>> runs;
        for (Loop& l : loops) {
            if (op.climb) std::reverse(l.begin(), l.end());
            if (!slopes) {
                Loop closed = l;
                closed.push_back(l[0]);
                runs.push_back(std::move(closed));
                continue;
            }
            std::vector<P2> run;
            for (std::size_t i = 0; i <= l.size(); ++i) {
                const P2& q = l[i % l.size()];
                const double sl = slopeAt(g, q.x, q.y);
                if (sl >= op.minSlope - 1e-9 && sl <= op.maxSlope + 1e-9) run.push_back(q);
                else if (!run.empty()) {
                    if (run.size() > 1) runs.push_back(run);
                    run.clear();
                }
            }
            if (run.size() > 1) runs.push_back(run);
        }
        while (!runs.empty()) {
            const P2 here{o.started ? o.at.x : c.sx0, o.started ? o.at.y : c.sy0};
            std::size_t pick = 0;
            double bd = kInf;
            for (std::size_t k = 0; k < runs.size(); ++k)
                if (const double d = dist2(runs[k][0], here); d < bd) bd = d, pick = k;
            std::vector<P2> run = std::move(runs[pick]);
            runs.erase(runs.begin() + static_cast<std::ptrdiff_t>(pick));
            if (run.front().x == run.back().x && run.front().y == run.back().y && run.size() > 3) { // closed: start nearest
                run.pop_back();
                run = startNear(run, here);
                run.push_back(run[0]);
            }
            const P2 cur{o.at.x, o.at.y};
            if (o.started && dist2(cur, run[0]) <= std::max(3.0 * sd, R) && o.freeLine(cur, run[0], z, floor) &&
                o.at.z < c.retractZ - 1e-6)
                o.feed(run[0].x, run[0].y, z);
            else {
                o.travel(run[0], prev + c.approach, z + c.approach, floor);
                o.plunge(run[0].x, run[0].y, z);
            }
            for (const P2& q : run) o.feed(q.x, q.y, z);
        }
        prev = z;
    }
    o.end();
    if (o.moves.empty() && err) *err = "No waterline touches the part";
    return !o.moves.empty();
}

// Drilling cycles at each hole, nearest first.
void orderHoles(std::vector<CamHole>& holes, P2 from) {
    for (std::size_t i = 0; i < holes.size(); ++i) {
        std::size_t best = i;
        double bd = kInf;
        for (std::size_t k = i; k < holes.size(); ++k)
            if (const double d = std::hypot(holes[k].x - from.x, holes[k].y - from.y); d < bd) bd = d, best = k;
        std::swap(holes[i], holes[best]);
        from = {holes[i].x, holes[i].y};
    }
}

double drillDepth(const CamTool& t, const CamOperation& op, const CamHole& h) {
    const double tipLen = t.radius() / std::tan(std::clamp(t.tipAngle, 20.0, 179.0) * 0.5 * kPiD / 180.0);
    switch (op.cycle) {
    case DrillCycle::Spot: return h.top - std::min(h.diameter * 0.5 + 0.1, t.radius()) / std::tan(std::clamp(t.tipAngle, 20.0, 179.0) * 0.5 * kPiD / 180.0);
    case DrillCycle::Tap: return h.through ? h.bottom - 2.0 * t.pitch : h.bottom + 2.0 * t.pitch;
    case DrillCycle::Bore: return h.through ? h.bottom - 0.5 : h.bottom;
    default: return h.through ? h.bottom - tipLen - 0.5 : h.bottom;
    }
}

bool genDrill(Ctx& c, std::string* err) {
    CamOperation& op = c.op;
    if (op.holes.empty()) {
        if (err) *err = "No holes for " + c.tool.describe() + (op.autoHoles ? " on the part" : "");
        return false;
    }
    orderHoles(op.holes, {c.sx0, c.sy0});
    const CamMaterial* m = camMaterial(c.job.setup.material);
    const double q = op.peck > 0 ? op.peck : c.D() * (m && m->stepdownFactor < 0.4 ? 0.5 : 1.0);
    PathOut o(op.moves, c);
    for (const CamHole& h : op.holes) {
        const double rPlane = h.top + 2.0, depth = drillDepth(c.tool, op, h);
        o.travel({h.x, h.y}, c.retractZ, rPlane, depth - 10);
        o.rapid(h.x, h.y, rPlane);
        switch (op.cycle) {
        case DrillCycle::Peck:
        case DrillCycle::ChipBreak: {
            double cur = rPlane;
            while (cur > depth + 1e-9) {
                const double next = std::max(depth, (cur == rPlane ? h.top : cur) - q);
                if (cur < rPlane) o.rapid(h.x, h.y, op.cycle == DrillCycle::Peck ? cur + 0.5 : cur);
                o.feed(h.x, h.y, next);
                if (next > depth + 1e-9) o.rapid(h.x, h.y, op.cycle == DrillCycle::Peck ? rPlane : next + 0.5);
                cur = next;
            }
            o.rapid(h.x, h.y, rPlane);
            break;
        }
        case DrillCycle::Tap:
        case DrillCycle::Bore:
            o.feed(h.x, h.y, depth);
            o.feed(h.x, h.y, rPlane); // feed out (the tap reverses)
            break;
        default:
            o.feed(h.x, h.y, depth);
            o.rapid(h.x, h.y, rPlane);
        }
        o.rapid(h.x, h.y, c.retractZ);
    }
    o.end();
    return true;
}

// Helical boring of holes wider than the end mill.
bool genBore(Ctx& c, std::string* err) {
    CamOperation& op = c.op;
    const double R = c.R();
    std::vector<CamHole> holes;
    for (const CamHole& h : op.holes)
        if (h.diameter * 0.5 - op.leave > R * 1.02) holes.push_back(h);
    if (holes.empty()) {
        if (err) *err = "No holes wider than " + c.tool.describe();
        return false;
    }
    orderHoles(holes, {c.sx0, c.sy0});
    PathOut o(op.moves, c);
    for (const CamHole& h : holes) {
        // The circle's radius, or less where the part's facets come inside it: the
        // exact cutter test around the hole at its bottom.
        const double depthZ = h.through ? h.bottom - 0.5 : h.bottom + op.leave;
        double rh = h.diameter * 0.5 - op.leave - R;
        for (int k = 0; k < 72 && rh > 0; ++k) {
            const double a = 2.0 * kPiD * k / 72.0, ux = std::cos(a), uy = std::sin(a);
            if (c.cl(h.x + rh * ux, h.y + rh * uy, depthZ - 10) <= std::max(depthZ, h.bottom) + 1e-6) continue;
            double lo = 0, hi = rh;
            for (int it = 0; it < 30; ++it) {
                const double m = (lo + hi) * 0.5;
                (c.cl(h.x + m * ux, h.y + m * uy, depthZ - 10) <= std::max(depthZ, h.bottom) + 1e-6 ? lo : hi) = m;
            }
            rh = lo;
        }
        if (rh < 1e-3) continue;
        const double pitch = std::min(op.stepdown > 0 ? op.stepdown : defaultStepdown(c) * 0.5, 2.0 * kPiD * rh * std::tan(kRamp * 1.5));
        const double depth = depthZ;
        const double z0 = h.top + 0.5;
        o.travel({h.x + rh, h.y}, c.retractZ, h.top + 2.0, depth - 10);
        o.plunge(h.x + rh, h.y, z0);
        const int perTurn = 48;
        double zz = z0;
        // Counter-clockwise inside a hole is climb milling.
        const double dir = op.climb ? 1.0 : -1.0;
        int k = 0;
        while (zz > depth + 1e-9) {
            ++k;
            zz = std::max(depth, z0 - pitch * k / perTurn);
            const double a = dir * 2.0 * kPiD * k / perTurn;
            o.feed(h.x + rh * std::cos(a), h.y + rh * std::sin(a), zz, c.feeds.feed * 0.7);
        }
        for (int i = 1; i <= perTurn; ++i) { // a flat turn at the bottom
            const double a = dir * 2.0 * kPiD * (k + i) / perTurn;
            o.feed(h.x + rh * std::cos(a), h.y + rh * std::sin(a), depth);
        }
        o.feed(h.x, h.y, depth);
        o.rapid(h.x, h.y, c.retractZ);
    }
    o.end();
    return true;
}

// 4-axis rotary wrapping: the part turns on A about X (through the rotary centre) under
// the tool, which stays over the axis. At each angle the part is turned and the tool
// dropped onto it, so the paths are exact drop-cutter paths in every turned position.
// A helix round the axis (a stepover along X each turn), or passes along the axis a
// stepover apart round the largest radius; with a stepdown, roughing levels from the
// stock's swing radius down first.
bool genRotary(Ctx& c, const Body& part, std::string* err) {
    const CamOperation& op = c.op;
    const Vec3 ctr = c.job.rotaryCentre();
    const double cy = ctr.y, cz = ctr.z, R = c.R();
    const MeshData world = worldMesh(part);
    double x0 = kInf, x1 = -kInf, rmax = 0;
    for (const Vertex& v : world.vertices) {
        x0 = std::min(x0, static_cast<double>(v.position.x));
        x1 = std::max(x1, static_cast<double>(v.position.x));
        rmax = std::max(rmax, std::hypot(v.position.y - cy, v.position.z - cz));
    }
    if (!(x1 > x0) || !(rmax > 0)) {
        if (err) *err = "The part has no length along X to turn";
        return false;
    }
    if (!std::isnan(op.top)) x1 = std::min(x1, op.top);       // TOP / BOTTOM limit the X range here
    if (!std::isnan(op.bottom)) x0 = std::max(x0, op.bottom);
    if (!(x1 > x0)) {
        if (err) *err = "Nothing between the X limits";
        return false;
    }
    const double s = std::max(op.stepover > 0 ? op.stepover : scallopStep(c.tool, op.scallop), 0.01);
    const double swing = std::max(c.job.swingRadius(), rmax);
    const double chord = std::clamp(c.tol * 50.0, 0.2, 1.0);
    const double floor = cz - 1.0; // never below the axis
    // The angles and X positions to drop the tool at.
    struct Pass {
        double deg;              // A, continuous
        std::vector<double> xs;  // along X
        std::vector<double> zs;  // the tool's tip there
    };
    std::vector<Pass> passes;
    if (op.along) {
        const int m = std::clamp(static_cast<int>(std::ceil(2 * kPiD * rmax / s)), 4, 7200);
        const double dx = std::clamp(R * 0.5, 0.05, 0.5);
        const int nx = std::max(2, static_cast<int>(std::ceil((x1 - x0) / dx)) + 1);
        for (int k = 0; k < m; ++k) {
            Pass p{360.0 * k / m, {}, {}};
            for (int i = 0; i < nx; ++i) p.xs.push_back(x0 + (x1 - x0) * i / (nx - 1));
            if (k % 2) std::reverse(p.xs.begin(), p.xs.end());
            passes.push_back(std::move(p));
        }
    } else {
        // One pass per angle of a turn, holding that angle's points of every turn of the helix.
        const int n = std::clamp(static_cast<int>(std::ceil(2 * kPiD * (rmax + R) / chord)), 72, 1440);
        const int turns = std::max(1, static_cast<int>(std::ceil((x1 - x0) / s)));
        for (int j = 0; j < n; ++j) {
            Pass p{360.0 * j / n, {}, {}};
            for (int k = 0; k <= turns; ++k) p.xs.push_back(std::min(x1, x0 + s * (k + static_cast<double>(j) / n)));
            passes.push_back(std::move(p));
        }
    }
    // Drop the tool at each angle onto the part turned to it.
    parallelFor(static_cast<int>(passes.size()), [&](int k) {
        Pass& p = passes[static_cast<std::size_t>(k)];
        MeshData m = world;
        for (Vertex& v : m.vertices) v.position = ctr + turnA(v.position - ctr, p.deg);
        const PartModel pm(meshBody("turned", std::move(m)), std::max(R, 1.0));
        p.zs.resize(p.xs.size());
        for (std::size_t i = 0; i < p.xs.size(); ++i) p.zs[i] = pm.drop(c.cut, p.xs[i], cy, floor - op.leave) + op.leave;
    });
    double lowest = kInf;
    for (const Pass& p : passes)
        for (double z : p.zs) lowest = std::min(lowest, z - cz);
    // Radial levels: roughing from the swing radius down, then the finish on the part.
    std::vector<double> levels;
    if (op.stepdown > 0)
        for (double r = swing - op.stepdown; r > lowest + 1e-6; r -= op.stepdown) levels.push_back(r);
    levels.push_back(-kInf);
    auto push = [&](CamMove::Kind k, double x, double z, double f, double a) {
        const CamMove m{k, vec(x, cy, z), f, static_cast<float>(a), 0.0f};
        if (!c.op.moves.empty()) {
            const CamMove& b = c.op.moves.back();
            if (length(b.to - m.to) < 1e-6f && std::fabs(b.a - m.a) < 1e-6f) return; // nowhere new
        }
        c.op.moves.push_back(m);
    };
    const double clear = c.clearZ, retract = cz + swing + c.job.setup.retract;
    push(CamMove::Rapid, op.along ? passes[0].xs[0] : x0, clear, 0, 0);
    for (std::size_t L = 0; L < levels.size(); ++L) {
        const double lv = levels[L];
        auto zAt = [&](double z) { return std::max(z, cz + lv); };
        // Rapid down only to just above what the level before left; feed in from there.
        const double above = cz + (L == 0 ? swing : levels[L - 1]) + 1.0;
        auto approach = [&](double z) { return std::max(z + 2.0, above); };
        if (op.along) {
            for (const Pass& p : passes) {
                push(CamMove::Rapid, p.xs[0], retract, 0, p.deg);
                push(CamMove::Rapid, p.xs[0], approach(zAt(p.zs[0])), 0, p.deg);
                push(CamMove::Plunge, p.xs[0], zAt(p.zs[0]), c.feeds.plunge, p.deg);
                for (std::size_t i = 1; i < p.xs.size(); ++i) push(CamMove::Feed, p.xs[i], zAt(p.zs[i]), c.feeds.feed, p.deg);
                push(CamMove::Rapid, p.xs.back(), retract, 0, p.deg);
            }
        } else {
            // The helix, forwards on even levels and back on odd ones (A keeps turning one way per level).
            const std::size_t n = passes.size(), turns = passes[0].xs.size();
            std::vector<std::pair<std::size_t, std::size_t>> order; // (turn, angle index)
            for (std::size_t k = 0; k < turns; ++k)
                for (std::size_t j = 0; j < n; ++j) {
                    if (k > 0 && passes[j].xs[k] <= passes[j].xs[k - 1] + 1e-12 && passes[j].xs[k] >= x1 - 1e-12 && j > 0) continue;
                    order.push_back({k, j});
                }
            if (L % 2) std::reverse(order.begin(), order.end());
            auto deg = [&](std::size_t k, std::size_t j) { return 360.0 * static_cast<double>(k) + passes[j].deg; };
            const auto [k0, j0] = order.front();
            push(CamMove::Rapid, passes[j0].xs[k0], retract, 0, deg(k0, j0));
            push(CamMove::Rapid, passes[j0].xs[k0], approach(zAt(passes[j0].zs[k0])), 0, deg(k0, j0));
            push(CamMove::Plunge, passes[j0].xs[k0], zAt(passes[j0].zs[k0]), c.feeds.plunge, deg(k0, j0));
            for (const auto& [k, j] : order) push(CamMove::Feed, passes[j].xs[k], zAt(passes[j].zs[k]), c.feeds.feed, deg(k, j));
            push(CamMove::Rapid, c.op.moves.back().to.x, retract, 0, c.op.moves.back().a);
        }
    }
    push(CamMove::Rapid, c.op.moves.back().to.x, clear, 0, c.op.moves.back().a);
    return true;
}

// --- simultaneous 5-axis --------------------------------------------------------------

// Triangles in a grid of cubes, to ask whether anything of the part is within a distance
// of a point (the shank and holder checks).
class TriGrid3 {
public:
    TriGrid3(const MeshData& m, double cell) {
        for (const Vertex& v : m.vertices) p.push_back({v.position.x, v.position.y, v.position.z});
        for (std::size_t k = 0; k + 2 < m.indices.size(); k += 3) tris.push_back({m.indices[k], m.indices[k + 1], m.indices[k + 2]});
        lo = {kInf, kInf, kInf};
        D3 hi{-kInf, -kInf, -kInf};
        for (const D3& q : p) lo = {std::min(lo.x, q.x), std::min(lo.y, q.y), std::min(lo.z, q.z)}, hi = {std::max(hi.x, q.x), std::max(hi.y, q.y), std::max(hi.z, q.z)};
        if (tris.empty()) return;
        c = std::max({cell, (hi.x - lo.x) / 160, (hi.y - lo.y) / 160, (hi.z - lo.z) / 160});
        n[0] = static_cast<int>((hi.x - lo.x) / c) + 1, n[1] = static_cast<int>((hi.y - lo.y) / c) + 1, n[2] = static_cast<int>((hi.z - lo.z) / c) + 1;
        cells.assign(static_cast<std::size_t>(n[0]) * static_cast<std::size_t>(n[1]) * static_cast<std::size_t>(n[2]), {});
        for (std::uint32_t t = 0; t < tris.size(); ++t) {
            D3 a{kInf, kInf, kInf}, b{-kInf, -kInf, -kInf};
            for (std::uint32_t v : tris[t]) a = {std::min(a.x, p[v].x), std::min(a.y, p[v].y), std::min(a.z, p[v].z)}, b = {std::max(b.x, p[v].x), std::max(b.y, p[v].y), std::max(b.z, p[v].z)};
            for (int k = cz(a.z); k <= cz(b.z); ++k)
                for (int j = cy(a.y); j <= cy(b.y); ++j)
                    for (int i = cx(a.x); i <= cx(b.x); ++i) cells[idx(i, j, k)].push_back(t);
        }
    }
    // Some triangle closer than r to q.
    bool near(const D3& q, double r) const {
        if (tris.empty()) return false;
        const double r2 = r * r;
        for (int k = cz(q.z - r); k <= cz(q.z + r); ++k)
            for (int j = cy(q.y - r); j <= cy(q.y + r); ++j)
                for (int i = cx(q.x - r); i <= cx(q.x + r); ++i)
                    for (const std::uint32_t t : cells[idx(i, j, k)])
                        if (dist2(q, p[tris[t][0]], p[tris[t][1]], p[tris[t][2]]) < r2) return true;
        return false;
    }

private:
    int cx(double v) const { return std::clamp(static_cast<int>((v - lo.x) / c), 0, n[0] - 1); }
    int cy(double v) const { return std::clamp(static_cast<int>((v - lo.y) / c), 0, n[1] - 1); }
    int cz(double v) const { return std::clamp(static_cast<int>((v - lo.z) / c), 0, n[2] - 1); }
    std::size_t idx(int i, int j, int k) const {
        return (static_cast<std::size_t>(k) * static_cast<std::size_t>(n[1]) + static_cast<std::size_t>(j)) * static_cast<std::size_t>(n[0]) + static_cast<std::size_t>(i);
    }
    // Squared distance from q to the triangle abc (the closest point, by region).
    static double dist2(const D3& q, const D3& a, const D3& b, const D3& cc) {
        auto sub = [](const D3& u, const D3& v) { return D3{u.x - v.x, u.y - v.y, u.z - v.z}; };
        auto dot = [](const D3& u, const D3& v) { return u.x * v.x + u.y * v.y + u.z * v.z; };
        auto at = [&](const D3& o, const D3& u, double s) { return D3{o.x + u.x * s, o.y + u.y * s, o.z + u.z * s}; };
        auto d2 = [&](const D3& u) { const D3 w = sub(q, u); return dot(w, w); };
        const D3 ab = sub(b, a), ac = sub(cc, a), ap = sub(q, a);
        const double d1 = dot(ab, ap), e1 = dot(ac, ap);
        if (d1 <= 0 && e1 <= 0) return d2(a);
        const D3 bp = sub(q, b);
        const double d3 = dot(ab, bp), e3 = dot(ac, bp);
        if (d3 >= 0 && e3 <= d3) return d2(b);
        const double vc = d1 * e3 - d3 * e1;
        if (vc <= 0 && d1 >= 0 && d3 <= 0) return d2(at(a, ab, d1 / (d1 - d3)));
        const D3 cp = sub(q, cc);
        const double d5 = dot(ab, cp), e5 = dot(ac, cp);
        if (e5 >= 0 && d5 <= e5) return d2(cc);
        const double vb = d5 * e1 - d1 * e5;
        if (vb <= 0 && e1 >= 0 && e5 <= 0) return d2(at(a, ac, e1 / (e1 - e5)));
        const double va = d3 * e5 - d5 * e3;
        if (va <= 0 && (e3 - d3) >= 0 && (d5 - e5) >= 0) return d2(at(b, sub(cc, b), (e3 - d3) / ((e3 - d3) + (d5 - e5))));
        const double den = 1.0 / (va + vb + vc), v = vb * den, w = vc * den;
        return d2({a.x + ab.x * v + ac.x * w, a.y + ab.y * v + ac.y * w, a.z + ab.z * v + ac.z * w});
    }
    std::vector<D3> p;
    std::vector<std::array<std::uint32_t, 3>> tris;
    std::vector<std::vector<std::uint32_t>> cells;
    D3 lo{0, 0, 0};
    double c = 1;
    int n[3] = {1, 1, 1};
};

// The table's A and C (degrees) that stand the tool axis `t` (part frame) up, the one of
// the two that keeps nearest the last C and inside the machine's A travel. False when
// neither fits the machine.
bool axisAngles(const D3& t, const CamMachine& m, double lastC, double& a, double& c) {
    const double tilt = std::acos(std::clamp(t.z, -1.0, 1.0)) * 180.0 / kPiD;
    const double s = std::hypot(t.x, t.y);
    double best = kInf;
    bool found = false;
    for (const double sg : {1.0, -1.0}) {
        const double aa = sg * tilt;
        if (aa < m.aMin - 1e-9 || aa > m.aMax + 1e-9) continue;
        // T = (sin a sin c, sin a cos c, cos a): c from the horizontal part, any c when upright.
        double cc = s < 1e-7 ? lastC : std::atan2(sg * t.x, sg * t.y) * 180.0 / kPiD;
        cc += 360.0 * std::round((lastC - cc) / 360.0);
        if (cc < m.cMin - 1e-9 || cc > m.cMax + 1e-9) {
            // Another turn of C that fits.
            const double k = std::ceil((m.cMin - cc) / 360.0);
            cc += 360.0 * k;
            if (cc > m.cMax + 1e-9) continue;
        }
        const double cost = std::fabs(cc - lastC) + 0.01 * std::fabs(aa);
        if (cost < best) best = cost, a = aa, c = cc, found = true;
    }
    return found;
}

bool genSurface5(Ctx& c, const Body& part, std::string* err) {
    const CamOperation& op = c.op;
    if (c.tool.type != CamToolType::Ball) {
        if (err) *err = "5-axis surface finishing needs a ball end mill (its tip is the same whatever the tilt)";
        return false;
    }
    const CamMachine& mach = c.job.setup.machine;
    if (!mach.hasA || !mach.hasC) {
        if (err) *err = "5-axis needs a machine with A and C axes (CAM MACHINE PRESET trunnion)";
        return false;
    }
    const PartModel& pm = c.part;
    const double Rt = c.R();                                   // the ball
    const double step = std::max(op.stepover > 0 ? op.stepover : scallopStep(c.tool, op.scallop), 0.01);
    const double ds = std::clamp(std::sqrt(8.0 * Rt * c.tol), 0.1, std::min(1.0, Rt * 0.5));
    const double floor = pm.z0 - 10.0; // off the part the tool goes down beside it, as parallel finishing does
    const double maxTilt = std::clamp(op.maxTilt, 0.0, 120.0) * kPiD / 180.0;
    const double lead = op.leadAngle * kPiD / 180.0, side = op.sideTilt * kPiD / 180.0;
    const double ang = op.angle * kPiD / 180.0;
    const D3 dir{std::cos(ang), std::sin(ang), 0}, across{-std::sin(ang), std::cos(ang), 0};
    // The part's outline, in the raster's frame.
    double u0 = kInf, u1 = -kInf, v0 = kInf, v1 = -kInf;
    for (int k = 0; k < 4; ++k) {
        const double x = k & 1 ? std::min(pm.x1 + Rt, c.sx1) : std::max(pm.x0 - Rt, c.sx0), y = k & 2 ? std::min(pm.y1 + Rt, c.sy1) : std::max(pm.y0 - Rt, c.sy0);
        const double u = x * dir.x + y * dir.y, v = x * across.x + y * across.y;
        u0 = std::min(u0, u), u1 = std::max(u1, u), v0 = std::min(v0, v), v1 = std::max(v1, v);
    }
    if (!(u1 > u0) || !(v1 > v0)) {
        if (err) *err = "Nothing of the part to finish";
        return false;
    }
    const MeshData world = worldMesh(part);
    const TriGrid3 grid(world, std::max(Rt, 1.0));
    const double gauge = c.tool.gauge(), HR = c.tool.holderRadius(), holderLen = c.tool.holderLength;
    const double margin = std::max(c.tol, 0.01);
    auto norm3 = [](D3 v) {
        const double l = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        return l > 1e-15 ? D3{v.x / l, v.y / l, v.z / l} : D3{0, 0, 1};
    };
    // Lean the axis back towards vertical to at most `limit` from it.
    auto clampTilt = [&](D3 t, double limit) {
        const double tilt = std::acos(std::clamp(t.z, -1.0, 1.0));
        if (tilt <= limit) return t;
        const double h = std::hypot(t.x, t.y);
        if (h < 1e-12) return D3{0, 0, 1};
        return D3{t.x / h * std::sin(limit), t.y / h * std::sin(limit), std::cos(limit)};
    };
    // The shank and holder above the ball, along the axis, clear of the part.
    auto clear = [&](const D3& ctr, const D3& t) {
        for (double s = Rt * 0.6; s < gauge - Rt; s += Rt * 0.5)
            if (grid.near({ctr.x + t.x * s, ctr.y + t.y * s, ctr.z + t.z * s}, Rt - margin)) return false;
        for (double s = gauge - Rt; s <= gauge - Rt + holderLen; s += std::max(HR * 0.5, 1.0))
            if (grid.near({ctr.x + t.x * s, ctr.y + t.y * s, ctr.z + t.z * s}, HR - margin)) return false;
        return true;
    };
    struct Pt {
        D3 ctr, axis;
        bool ok = false;
    };
    const int passes = static_cast<int>(std::floor((v1 - v0) / step)) + 1;
    std::vector<std::vector<Pt>> rows(static_cast<std::size_t>(passes));
    std::atomic<int> unreachable{0};
    parallelFor(passes, [&](int j) {
        const double v = v0 + j * step;
        const double sgn = j % 2 ? -1.0 : 1.0; // zig-zag
        const int n = static_cast<int>(std::ceil((u1 - u0) / ds)) + 1;
        std::vector<Pt>& row = rows[static_cast<std::size_t>(j)];
        row.resize(static_cast<std::size_t>(n));
        auto xyAt = [&](double u) { return P2{dir.x * u + across.x * v, dir.y * u + across.y * v}; };
        for (int i = 0; i < n; ++i) {
            const double u = sgn > 0 ? u0 + (u1 - u0) * i / std::max(1, n - 1) : u1 - (u1 - u0) * i / std::max(1, n - 1);
            const P2 q = xyAt(u);
            Pt& pt = row[static_cast<std::size_t>(i)];
            const double h = c.cl(q.x, q.y, floor);
            const bool on = h > floor + 1e-6;
            // The surface normal at the contact: the ball centre's surface leans the same way.
            // Off the part, upright; at its edge, from the neighbours on it.
            D3 nrm{0, 0, 1};
            if (on) {
                const double dl = std::min(ds * 0.5, 0.25);
                auto slopeOf = [&](double x1, double y1, double x0, double y0) {
                    const double h1 = c.cl(x1, y1, floor), h0 = c.cl(x0, y0, floor);
                    const bool o1 = h1 > floor + 1e-6, o0 = h0 > floor + 1e-6;
                    if (o1 && o0) return (h1 - h0) / (2 * dl);
                    if (o1) return (h1 - h) / dl;
                    if (o0) return (h - h0) / dl;
                    return 0.0;
                };
                nrm = norm3({-slopeOf(q.x + dl, q.y, q.x - dl, q.y), -slopeOf(q.x, q.y + dl, q.x, q.y - dl), 1.0});
            }
            const double slope = std::acos(std::clamp(nrm.z, -1.0, 1.0)) * 180.0 / kPiD;
            if (slope < op.minSlope - 1e-9 || slope > op.maxSlope + 1e-9) continue;
            // Lead along the cut and tilt across it, from the normal.
            D3 f{dir.x * sgn, dir.y * sgn, 0};
            const double fn = f.x * nrm.x + f.y * nrm.y;
            f = norm3({f.x - nrm.x * fn, f.y - nrm.y * fn, -nrm.z * fn});
            const D3 sd = norm3({f.y * nrm.z - f.z * nrm.y, f.z * nrm.x - f.x * nrm.z, f.x * nrm.y - f.y * nrm.x});
            D3 t{nrm.x * std::cos(lead) + f.x * std::sin(lead), nrm.y * std::cos(lead) + f.y * std::sin(lead), nrm.z * std::cos(lead) + f.z * std::sin(lead)};
            t = norm3({t.x * std::cos(side) + sd.x * std::sin(side), t.y * std::cos(side) + sd.y * std::sin(side), t.z * std::cos(side) + sd.z * std::sin(side)});
            pt.axis = clampTilt(t, maxTilt);
            pt.ctr = {q.x, q.y, h + Rt};
            pt.ok = true;
        }
        // Smooth the axis along the pass (the rotary axes move gently), within each run.
        std::vector<D3> sm(row.size());
        for (std::size_t i = 0; i < row.size(); ++i) {
            if (!row[i].ok) continue;
            D3 s{0, 0, 0};
            for (std::size_t k = i >= 3 ? i - 3 : 0; k < std::min(row.size(), i + 4); ++k) {
                bool run = true; // only points of the same run
                for (std::size_t m = std::min(i, k); m <= std::max(i, k); ++m) run = run && row[m].ok;
                if (run) s = {s.x + row[k].axis.x, s.y + row[k].axis.y, s.z + row[k].axis.z};
            }
            sm[i] = clampTilt(norm3(s), maxTilt);
        }
        // Clear of the part: tilt back towards vertical in steps until the shank and holder are.
        for (std::size_t i = 0; i < row.size(); ++i) {
            if (!row[i].ok) continue;
            D3 t = sm[i];
            bool ok = clear(row[i].ctr, t);
            for (int k = 1; !ok && k <= 12; ++k) {
                const double w = k / 12.0;
                t = norm3({sm[i].x * (1 - w), sm[i].y * (1 - w), sm[i].z * (1 - w) + w});
                ok = clear(row[i].ctr, t);
            }
            if (!ok) {
                row[i].ok = false;
                ++unreachable;
                continue;
            }
            row[i].axis = t;
        }
    });
    // The runs: each a stretch of points the table turns through steadily, the tip going
    // straight over the part between them.
    struct M5 {
        Vec3 tip; // on the part
        double a, c;
    };
    std::vector<std::vector<M5>> cuts;
    double lastC = 0;
    std::size_t refused = 0;
    for (const auto& row : rows) {
        std::size_t i = 0;
        while (i < row.size()) {
            if (!row[i].ok) {
                ++i;
                continue;
            }
            std::size_t e = i;
            while (e < row.size() && row[e].ok) ++e;
            // This run's turns. A point the machine can't turn to ends it, and so does a
            // flip of the table (the axis passing upright, or A reaching its travel): the
            // tool lifts, the table turns, and the cut goes on from there.
            std::vector<std::vector<M5>> runs(1);
            for (std::size_t k = i; k < e; ++k) {
                double a = 0, cc = 0;
                const Pt& p = row[k];
                if (!axisAngles(p.axis, mach, lastC, a, cc)) {
                    ++refused;
                    if (!runs.back().empty()) runs.emplace_back();
                    continue;
                }
                const Vec3 tip = vec(p.ctr.x - p.axis.x * Rt, p.ctr.y - p.axis.y * Rt, p.ctr.z - p.axis.z * Rt);
                if (!runs.back().empty() && (std::fabs(cc - runs.back().back().c) > 30.0 || std::fabs(a - runs.back().back().a) > 15.0)) runs.emplace_back();
                lastC = cc;
                runs.back().push_back({tip, a, cc});
            }
            i = std::max(e, i + 1);
            for (const std::vector<M5>& run : runs) {
                if (run.size() < 2) continue;
                // Drop points the straight line and steady turn between their neighbours make.
                std::vector<M5> keep{run.front()};
                for (std::size_t k = 1; k + 1 < run.size(); ++k) {
                    const M5 &a = keep.back(), &b = run[k + 1], &m = run[k];
                    const Vec3 mid = a.tip + (b.tip - a.tip) * 0.5f;
                    const bool straight = length(mid - m.tip) < c.tol && std::fabs(0.5 * (a.a + b.a) - m.a) < 0.25 &&
                                          std::fabs(0.5 * (a.c + b.c) - m.c) < 0.25 && length(b.tip - a.tip) < 4.0 * ds;
                    if (!straight) keep.push_back(m);
                }
                keep.push_back(run.back());
                // Each move turns the table at most 1.5 degrees (a big turn in one straight
                // machine move would swing the tip through the part).
                std::vector<M5> fine{keep.front()};
                for (std::size_t k = 1; k < keep.size(); ++k) {
                    const M5 &a = keep[k - 1], &b = keep[k];
                    const int n = std::max(1, static_cast<int>(std::ceil(std::max(std::fabs(b.a - a.a), std::fabs(b.c - a.c)) / 1.5)));
                    for (int q = 1; q <= n; ++q) {
                        const double t = static_cast<double>(q) / n;
                        fine.push_back({a.tip + (b.tip - a.tip) * static_cast<float>(t), a.a + (b.a - a.a) * t, a.c + (b.c - a.c) * t});
                    }
                }
                cuts.push_back(std::move(fine));
            }
        }
    }
    // The moves: down each run along the axis, to above the surface by what the roughing
    // before may have left (its steps, measured along a tilted axis). Between runs close
    // together the tool goes across lifted that far, at feed (a straight move of all five
    // axes), where the ball, shank and holder stay clear of the part all the way; else up
    // to the clearance height and over.
    const double gap = std::max({2.0, c.job.setup.retract, c.approach});
    auto push = [&](CamMove::Kind k, const Vec3& to, double f, double a, double cc) {
        c.op.moves.push_back({k, to, f, static_cast<float>(a), static_cast<float>(cc)});
    };
    auto machineAt = [&](const M5& m) { return camToMachine(c.job, m.tip, static_cast<float>(m.a), static_cast<float>(m.c)); };
    auto lift = [&](const M5& m) { return gap / std::max(std::cos(m.a * kPiD / 180.0), 0.3) + 1.0; };
    auto axisOf = [](double a, double cc) {
        const Vec3 u = camToolAxis(static_cast<float>(a), static_cast<float>(cc));
        return D3{u.x, u.y, u.z};
    };
    // The lifted stretch from the end of one run to the start of the next, or empty when
    // it isn't clear.
    auto link = [&](const M5& from, const M5& to) {
        std::vector<M5> out;
        if (length(to.tip - from.tip) > 4.0 * Rt + step || std::fabs(to.a - from.a) > 90.0 || std::fabs(to.c - from.c) > 200.0) return out;
        const D3 af = axisOf(from.a, from.c), at = axisOf(to.a, to.c);
        const double lf = lift(from), lt = lift(to);
        const Vec3 p0 = from.tip + vec(af.x * lf, af.y * lf, af.z * lf), p1 = to.tip + vec(at.x * lt, at.y * lt, at.z * lt);
        const int n = std::max({1, static_cast<int>(std::ceil(length(p1 - p0) / ds)), static_cast<int>(std::ceil(std::max(std::fabs(to.a - from.a), std::fabs(to.c - from.c)) / 1.5))});
        auto centre = [&](const M5& m) {
            const D3 u = axisOf(m.a, m.c);
            return D3{m.tip.x + u.x * Rt, m.tip.y + u.y * Rt, m.tip.z + u.z * Rt};
        };
        // The gap all the way where both ends have it (over the surface); beside a wall, where
        // lifting doesn't move the ball away, no nearer than the cut itself.
        const bool over = !grid.near(centre({p0, from.a, from.c}), Rt + gap) && !grid.near(centre({p1, to.a, to.c}), Rt + gap);
        const double need = over ? Rt + gap : Rt - margin;
        for (int q = 0; q <= n; ++q) {
            const double t = static_cast<double>(q) / n;
            const M5 m{p0 + (p1 - p0) * static_cast<float>(t), from.a + (to.a - from.a) * t, from.c + (to.c - from.c) * t};
            const D3 u = axisOf(m.a, m.c), ctr = centre(m);
            if (grid.near(ctr, need) || !clear(ctr, u)) return std::vector<M5>{};
            if (q > 0) out.push_back(m);
        }
        return out;
    };
    for (std::size_t r = 0; r < cuts.size(); ++r) {
        const std::vector<M5>& fine = cuts[r];
        const M5& s0 = fine.front();
        const Vec3 m0 = machineAt(s0);
        if (r == 0 || c.op.moves.empty() || c.op.moves.back().kind != CamMove::Feed) {
            push(CamMove::Rapid, vec(m0.x, m0.y, static_cast<float>(c.clearZ)), 0, s0.a, s0.c);
            push(CamMove::Rapid, m0 + vec(0, 0, static_cast<float>(lift(s0))), 0, s0.a, s0.c);
        }
        push(CamMove::Plunge, m0, c.feeds.plunge, s0.a, s0.c);
        for (std::size_t k = 1; k < fine.size(); ++k) push(CamMove::Feed, machineAt(fine[k]), c.feeds.feed, fine[k].a, fine[k].c);
        const M5& s1 = fine.back();
        const Vec3 m1 = machineAt(s1);
        push(CamMove::Rapid, m1 + vec(0, 0, static_cast<float>(lift(s1))), 0, s1.a, s1.c);
        const std::vector<M5> hop = r + 1 < cuts.size() ? link(s1, cuts[r + 1].front()) : std::vector<M5>{};
        if (hop.empty()) push(CamMove::Rapid, vec(m1.x, m1.y, static_cast<float>(c.clearZ)), 0, s1.a, s1.c);
        else
            for (const M5& m : hop) push(CamMove::Feed, machineAt(m), std::max(c.feeds.feed, mach.maxFeed), m.a, m.c);
    }
    if (c.op.moves.empty()) {
        if (err)
            *err = unreachable > 0 || refused > 0 ? "No point of the part can be reached: the shank or holder would touch it, or the machine can't turn that far"
                                                  : "Nothing of the part to finish (SLOPE limits?)";
        return false;
    }
    return true;
}

} // namespace

std::vector<CamHole> findHoles(const Body& body) {
    std::vector<CamHole> out;
    const MeshData m = worldMesh(body);
    std::vector<Vec3> p;
    for (const Vertex& v : m.vertices) p.push_back(v.position);
    std::vector<section::Tri> tris;
    for (std::size_t k = 0; k + 2 < m.indices.size(); k += 3) tris.push_back({m.indices[k], m.indices[k + 1], m.indices[k + 2]});
    if (tris.empty()) return out;
    const AABB box = m.bounds();
    std::vector<std::uint32_t> all(tris.size());
    for (std::uint32_t i = 0; i < all.size(); ++i) all[i] = i;
    const double h = box.max.z - box.min.z;
    const int levels = std::clamp(static_cast<int>(h / 0.5), 4, 80);
    for (int k = 0; k < levels; ++k) {
        const double z = box.min.z + h * (k + 0.5) / levels;
        for (const Loop& l : section::contours(p, tris, all, z)) {
            if (l.size() < 8 || section::loopArea(l) >= 0) continue; // holes run clockwise
            // Least-squares circle (Kasa): x^2 + y^2 + D x + E y + F = 0.
            double sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0, sxz = 0, syz = 0, sz = 0;
            const double ox = l[0].x, oy = l[0].y; // shifted for precision
            for (const P2& q : l) {
                const double x = q.x - ox, y = q.y - oy, w2 = x * x + y * y;
                sx += x, sy += y, sxx += x * x, syy += y * y, sxy += x * y, sxz += x * w2, syz += y * w2, sz += w2;
            }
            const double nn = static_cast<double>(l.size());
            // [sxx sxy sx; sxy syy sy; sx sy n] [D E F] = -[sxz syz sz]
            const double sys[3][4] = {{sxx, sxy, sx, -sxz}, {sxy, syy, sy, -syz}, {sx, sy, nn, -sz}};
            auto det3 = [](double a0, double a1, double a2, double a3, double a4, double a5, double a6, double a7, double a8) {
                return a0 * (a4 * a8 - a5 * a7) - a1 * (a3 * a8 - a5 * a6) + a2 * (a3 * a7 - a4 * a6);
            };
            const double dd = det3(sys[0][0], sys[0][1], sys[0][2], sys[1][0], sys[1][1], sys[1][2], sys[2][0], sys[2][1], sys[2][2]);
            if (std::fabs(dd) < 1e-18) continue;
            const double cD = det3(sys[0][3], sys[0][1], sys[0][2], sys[1][3], sys[1][1], sys[1][2], sys[2][3], sys[2][1], sys[2][2]) / dd;
            const double cE = det3(sys[0][0], sys[0][3], sys[0][2], sys[1][0], sys[1][3], sys[1][2], sys[2][0], sys[2][3], sys[2][2]) / dd;
            const double cF = det3(sys[0][0], sys[0][1], sys[0][3], sys[1][0], sys[1][1], sys[1][3], sys[2][0], sys[2][1], sys[2][3]) / dd;
            const double cx = ox - cD / 2, cy = oy - cE / 2, r2 = cD * cD / 4 + cE * cE / 4 - cF;
            if (!(r2 > 0)) continue;
            const double r = std::sqrt(r2);
            double rmin = kInf, rmax = 0;
            for (const P2& q : l) {
                const double rq = std::hypot(q.x - cx, q.y - cy);
                rmin = std::min(rmin, rq), rmax = std::max(rmax, rq);
            }
            if (r < 0.2 || rmax - rmin > std::max(0.03 * r, 0.03)) continue;
            bool known = false;
            for (const CamHole& o : out)
                if (std::hypot(o.x - cx, o.y - cy) < 0.05 && std::fabs(o.diameter - 2 * r) < 0.05) known = true;
            if (!known) out.push_back({cx, cy, 2 * r, z, z, false});
        }
    }
    // Each hole's wall runs from its top to its bottom: the corners on its circle, which
    // also give its true size (the section's points between corners cut across).
    for (CamHole& hole : out) {
        double top = -kInf, bot = kInf, rs = 0;
        int count = 0;
        for (const Vec3& q : p) {
            const double rq = std::hypot(q.x - hole.x, q.y - hole.y);
            if (std::fabs(rq - hole.diameter * 0.5) < std::max(0.03, hole.diameter * 0.01)) {
                top = std::max(top, static_cast<double>(q.z)), bot = std::min(bot, static_cast<double>(q.z));
                rs += rq, ++count;
            }
        }
        if (std::isfinite(top)) hole.top = top, hole.bottom = bot;
        if (count > 0) hole.diameter = std::round(2.0 * rs / count * 1000.0) / 1000.0;
        hole.through = hole.bottom <= box.min.z + 1e-3;
        if (!hole.through) { // anything under the hole's bottom?
            bool under = false;
            for (const section::Tri& t : tris) {
                const Vec3 &a = p[t[0]], &b = p[t[1]], &cc = p[t[2]];
                const double d = (b.y - cc.y) * (a.x - cc.x) + (cc.x - b.x) * (a.y - cc.y);
                if (std::fabs(d) < 1e-12) continue;
                const double l1 = ((b.y - cc.y) * (hole.x - cc.x) + (cc.x - b.x) * (hole.y - cc.y)) / d;
                const double l2 = ((cc.y - a.y) * (hole.x - cc.x) + (a.x - cc.x) * (hole.y - cc.y)) / d;
                if (l1 < 0 || l2 < 0 || l1 + l2 > 1) continue;
                if (l1 * a.z + l2 * b.z + (1 - l1 - l2) * cc.z < hole.bottom - 1e-3) under = true;
            }
            hole.through = !under;
        }
    }
    std::sort(out.begin(), out.end(), [](const CamHole& a, const CamHole& b) { return a.diameter > b.diameter; });
    return out;
}

bool generateOperation(const CamJob& job, const Body& part, CamOperation& op, std::string* error) {
    op.moves.clear();
    op.generated = false;
    // 3+2: turn the part and the stock with the table, make the 3-axis operation there.
    if ((op.indexA != 0 || op.indexC != 0) && op.kind != CamOpKind::Rotary && op.kind != CamOpKind::Surface5) {
        if (!job.setup.hasStock) {
            if (error) *error = "Set up the stock first (CAM SETUP)";
            return false;
        }
        if (op.rest) {
            if (error) *error = "REST works on 3-axis operations only";
            return false;
        }
        const float a = static_cast<float>(op.indexA), cc = static_cast<float>(op.indexC);
        MeshData m = worldMesh(part);
        for (Vertex& v : m.vertices) {
            v.position = camToMachine(job, v.position, a, cc);
            v.normal = turnA(turnC(v.normal, cc), a);
        }
        const Body turned = meshBody(part.name, std::move(m));
        CamJob tj = job;
        AABB box;
        for (int k = 0; k < 8; ++k)
            box.expand(camToMachine(job, {k & 1 ? job.setup.stockMax.x : job.setup.stockMin.x, k & 2 ? job.setup.stockMax.y : job.setup.stockMin.y,
                                          k & 4 ? job.setup.stockMax.z : job.setup.stockMin.z},
                                    a, cc));
        tj.setup.stockMin = box.min;
        tj.setup.stockMax = box.max;
        std::size_t k = 0;
        while (k < job.ops.size() && &job.ops[k] != &op) ++k;
        if (k == tj.ops.size()) tj.ops.push_back(op);
        for (CamOperation& o : tj.ops) o.indexA = o.indexC = 0, o.kind = o.kind == CamOpKind::Rotary ? CamOpKind::Rough : o.kind;
        CamOperation& t = tj.ops[k];
        if (!generateOperation(tj, turned, t, error)) return false;
        op = [&] {
            CamOperation r = t;
            r.indexA = op.indexA;
            r.indexC = op.indexC;
            r.kind = op.kind;
            return r;
        }();
        // Clear the turn: rise to the job's clearance before the first move and after the last.
        const double clear = job.clearZ();
        if (!op.moves.empty()) {
            const Vec3 f = op.moves.front().to, b = op.moves.back().to;
            op.moves.insert(op.moves.begin(), CamMove{CamMove::Rapid, Vec3{f.x, f.y, static_cast<float>(clear)}, 0.0});
            op.moves.push_back({CamMove::Rapid, Vec3{b.x, b.y, static_cast<float>(clear)}, 0.0});
        }
        for (CamMove& mv : op.moves) mv.a = a, mv.c = cc;
        return true;
    }
    const CamTool* tool = job.tool(op.tool);
    if (!tool) {
        if (error) *error = "No tool T" + std::to_string(op.tool) + " in the tool library (CAM TOOL)";
        return false;
    }
    if (!job.setup.hasStock) {
        if (error) *error = "Set up the stock first (CAM SETUP)";
        return false;
    }
    const bool drilling = op.kind == CamOpKind::Drill;
    const bool pointed = tool->type == CamToolType::Drill || tool->type == CamToolType::Spot || tool->type == CamToolType::Tap ||
                         tool->type == CamToolType::Chamfer;
    if (drilling != pointed && !(drilling && tool->type == CamToolType::Flat)) {
        if (error) *error = std::string(drilling ? "Drilling needs a drill, spot drill or tap" : "Milling needs an end mill or face mill") +
                            ", not " + tool->describe();
        return false;
    }
    const CamMaterial* mat = camMaterial(job.setup.material);
    if (!mat) mat = &camMaterials()[0];
    const PartModel pm(part, std::max(tool->radius(), 1.0));
    if (pm.empty()) {
        if (error) *error = part.name + " has no surface";
        return false;
    }
    const CamSetup& su = job.setup;
    double stepover = op.stepover;
    if (stepover <= 0) {
        if (op.kind == CamOpKind::Face) stepover = tool->diameter * 0.7;
        else if (op.kind == CamOpKind::Rough || op.kind == CamOpKind::Pocket) stepover = tool->diameter * (op.adaptive ? 0.12 : 0.4);
        else if (op.kind == CamOpKind::Parallel || op.kind == CamOpKind::Waterline || op.kind == CamOpKind::Rotary || op.kind == CamOpKind::Surface5)
            stepover = scallopStep(*tool, op.scallop);
    }
    Ctx c{job, *tool, pm, op, cutterOf(*tool, op.leave), su.stockMin.x, su.stockMin.y, su.stockMax.x, su.stockMax.y, su.stockMax.z,
          su.stockMin.z, su.stockMax.z + su.retract, su.stockMax.z + su.clearance, std::max(su.tolerance, 1e-4),
          camFeeds(*tool, *mat, su.machine, op.kind == CamOpKind::Contour || op.kind == CamOpKind::Drill ? 0.0 : stepover)};
    // Roughing before this leaves steps up to its stepdown (plus its stock to leave).
    for (const CamOperation& prev : job.ops) {
        if (&prev == &op) break;
        if (prev.kind != CamOpKind::Rough && prev.kind != CamOpKind::Pocket) continue;
        const CamTool* pt = job.tool(prev.tool);
        const CamMaterial* pm2 = camMaterial(su.material);
        const double psd = prev.stepdown > 0 ? prev.stepdown : (pt ? pt->diameter * (pm2 ? pm2->stepdownFactor : 0.5) : 3.0);
        c.approach = std::max(c.approach, psd + prev.leave + 1.0);
    }
    // Rest machining: simulate the operations before this one for the stock they leave.
    CamSimulation restSim;
    if (op.rest && (op.kind == CamOpKind::Rough || op.kind == CamOpKind::Pocket)) {
        CamJob before = job;
        std::size_t k = 0;
        while (k < job.ops.size() && &job.ops[k] != &op) ++k;
        before.ops.resize(k);
        if (before.ops.empty()) {
            if (error) *error = "REST machines what the operations before it left: there are none";
            return false;
        }
        std::string e;
        if (!simulateJob(before, part, restSim, &e)) {
            if (error) *error = "REST: " + e;
            return false;
        }
        c.rest = &restSim;
    }
    if (op.kind == CamOpKind::Rotary || op.kind == CamOpKind::Surface5) {
        c.clearZ = job.clearZ();
        c.retractZ = job.rotaryCentre().z + job.swingRadius() + su.retract;
    }
    if (op.setFeed > 0) c.feeds.feed = op.setFeed;
    if (op.setPlunge > 0) c.feeds.plunge = op.setPlunge;
    if (op.setRpm > 0) c.feeds.rpm = std::min(op.setRpm, su.machine.maxRpm);
    if (drilling || op.kind == CamOpKind::Bore) {
        if (op.autoHoles) {
            op.holes.clear();
            for (const CamHole& h : findHoles(part)) {
                const bool fits = op.kind == CamOpKind::Bore ? h.diameter > tool->diameter * 1.04
                                  : op.cycle == DrillCycle::Spot ? true
                                  : op.cycle == DrillCycle::Tap  ? std::fabs(h.diameter - (tool->diameter - tool->pitch)) < 0.15 ||
                                                                      std::fabs(h.diameter - tool->diameter) < 0.05
                                                                 : std::fabs(h.diameter - tool->diameter) < 0.06;
                if (fits) op.holes.push_back(h);
            }
        }
        if (op.cycle == DrillCycle::Tap) c.feeds.feed = c.feeds.plunge = c.feeds.rpm * tool->pitch; // one pitch a turn, always
    }
    bool ok = false;
    switch (op.kind) {
    case CamOpKind::Face: ok = genFace(c, error); break;
    case CamOpKind::Contour: ok = genContour(c, error); break;
    case CamOpKind::Pocket: ok = genClearing(c, true, error); break;
    case CamOpKind::Rough: ok = genClearing(c, false, error); break;
    case CamOpKind::Parallel: ok = genParallel(c, error); break;
    case CamOpKind::Waterline: ok = genWaterline(c, error); break;
    case CamOpKind::Drill: ok = genDrill(c, error); break;
    case CamOpKind::Bore: ok = genBore(c, error); break;
    case CamOpKind::Rotary: ok = genRotary(c, part, error); break;
    case CamOpKind::Surface5: ok = genSurface5(c, part, error); break;
    }
    if (!ok) {
        op.moves.clear();
        return false;
    }
    if (op.kind == CamOpKind::Pocket && op.moves.empty()) {
        if (error) *error = "No pockets: every region reaches outside the part";
        return false;
    }
    op.rpm = c.feeds.rpm;
    op.feed = c.feeds.feed;
    op.plunge = c.feeds.plunge;
    double cut = 0;
    Vec3 at = op.moves.empty() ? Vec3{} : op.moves[0].to;
    float atA = op.moves.empty() ? 0.0f : op.moves[0].a;
    const Vec3 rc = job.rotaryCentre();
    for (const CamMove& m : op.moves) {
        double len = length(m.to - at);
        if (m.a != atA)
            len = std::hypot(len, 0.5 * (std::hypot(at.y - rc.y, at.z - rc.z) + std::hypot(m.to.y - rc.y, m.to.z - rc.z)) * (m.a - atA) * kPiD / 180.0);
        if (m.kind != CamMove::Rapid) cut += len;
        at = m.to;
        atA = m.a;
    }
    const Vec3 centre = job.rotaryCentre();
    const double secs = movesSeconds(op.moves, su.machine, op.moves.empty() ? Vec3{} : op.moves[0].to, &centre);
    char b[160];
    std::snprintf(b, sizeof(b), "%zu moves, %.2f m cutting, %d min %02d s", op.moves.size(), cut / 1000.0, static_cast<int>(secs) / 60,
                  static_cast<int>(secs) % 60);
    op.summary = b;
    op.generated = true;
    return true;
}

// --- simulation -------------------------------------------------------------------------

namespace {

bool generateAll(CamJob& job, const Body& part, std::string* error) {
    for (CamOperation& op : job.ops)
        if (!op.generated) {
            std::string e;
            if (!generateOperation(job, part, op, &e)) {
                if (error) *error = (op.name.empty() ? std::string(camOpKey(op.kind)) : op.name) + ": " + e;
                return false;
            }
        }
    return true;
}

// Multi-axis jobs: the stock as voxels, cut by each tool (its tip shape, then a straight
// flute up to the flute length) along each move in the part's frame. The tool's axis
// turns with the table, and moves that turn the part are followed in small steps.
bool simulateVoxels(CamJob& job, const Body& part, CamSimulation& out, std::string* error) {
    const CamSetup& su = job.setup;
    const Vec3 lo = su.stockMin, size = su.stockMax - su.stockMin;
    double minR = 1e9;
    for (const CamOperation& op : job.ops)
        if (const CamTool* t = job.tool(op.tool)) minR = std::min(minR, t->radius());
    const double vol = static_cast<double>(size.x) * size.y * size.z;
    const double cell = std::clamp(std::max(minR / 5.0, std::cbrt(vol / 6.0e6)), 0.05, 2.0);
    const int nx = std::max(1, static_cast<int>(std::ceil(size.x / cell))), ny = std::max(1, static_cast<int>(std::ceil(size.y / cell))),
              nz = std::max(1, static_cast<int>(std::ceil(size.z / cell)));
    auto idx = [&](int i, int j, int k) { return (static_cast<std::size_t>(k) * static_cast<std::size_t>(ny) + static_cast<std::size_t>(j)) * static_cast<std::size_t>(nx) + static_cast<std::size_t>(i); };
    auto centre = [&](int i, int j, int k) { return D3{lo.x + (i + 0.5) * cell, lo.y + (j + 0.5) * cell, lo.z + (k + 0.5) * cell}; };
    std::vector<unsigned char> v(static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny) * static_cast<std::size_t>(nz), 1);
    // Inside the part: the crossings of a vertical line through each column.
    const PartModel pm(part, std::max(cell * 4.0, 1.0));
    parallelFor(ny, [&](int j) {
        std::vector<double> zs;
        for (int i = 0; i < nx; ++i) {
            const double x = lo.x + (i + 0.5) * cell + 1.3e-7, y = lo.y + (j + 0.5) * cell + 0.7e-7; // off the edges
            pm.crossings(x, y, zs);
            for (std::size_t q = 0; q + 1 < zs.size(); q += 2)
                for (int k = 0; k < nz; ++k) {
                    const double z = lo.z + (k + 0.5) * cell;
                    if (z > zs[q] && z < zs[q + 1]) v[idx(i, j, k)] = 2;
                }
        }
    });
    // The part's inside: every neighbour part too (cutting these is a gouge of a voxel or more).
    std::vector<unsigned char> init = v;
    for (int k = 1; k + 1 < nz; ++k)
        for (int j = 1; j + 1 < ny; ++j)
            for (int i = 1; i + 1 < nx; ++i)
                if (init[idx(i, j, k)] == 2 && init[idx(i - 1, j, k)] == 2 && init[idx(i + 1, j, k)] == 2 && init[idx(i, j - 1, k)] == 2 &&
                    init[idx(i, j + 1, k)] == 2 && init[idx(i, j, k - 1)] == 2 && init[idx(i, j, k + 1)] == 2)
                    v[idx(i, j, k)] = 3;
    std::size_t removed = 0;
    const Vec3 rc = job.rotaryCentre();
    Vec3 at = camToMachine(job, {static_cast<float>((lo.x + su.stockMax.x) * 0.5), static_cast<float>((lo.y + su.stockMax.y) * 0.5),
                                 static_cast<float>(job.clearZ())},
                           0, 0);
    float atA = 0, atC = 0;
    int lastTool = -1;
    const double hi[3] = {su.stockMax.x, su.stockMax.y, su.stockMax.z};
    for (const CamOperation& op : job.ops) {
        const CamTool* tool = job.tool(op.tool);
        if (!tool) continue;
        if (op.tool != lastTool && lastTool >= 0) out.seconds += su.machine.toolChange;
        lastTool = op.tool;
        const Cutter cut = cutterOf(*tool);
        const double R = cut.R, flute = tool->fluteLength, gauge = tool->gauge(), HR = tool->holderRadius();
        out.seconds += movesSeconds(op.moves, su.machine, at, &rc);
        if (op.kind == CamOpKind::Drill && op.dwell > 0) out.seconds += op.dwell * static_cast<double>(op.holes.size());
        for (const CamMove& m : op.moves) {
            const Vec3 a = at, b = m.to;
            const float a0 = atA, a1 = m.a, c0 = atC, c1 = m.c;
            at = b, atA = m.a, atC = m.c;
            double len = length(b - a);
            if (a0 != a1) len = std::hypot(len, 0.5 * (std::hypot(a.y - rc.y, a.z - rc.z) + std::hypot(b.y - rc.y, b.z - rc.z)) * (a1 - a0) * kPiD / 180.0);
            if (m.kind == CamMove::Rapid) out.rapidLength += len;
            else out.cutLength += len;
            if (m.kind == CamMove::Rapid && std::min(a.z, b.z) >= job.clearZ() - 1e-3) continue; // turning up above everything
            // Steps of under a voxel along the part (a turn moves points far from the axis most).
            const Vec3 pa = camToPart(job, a, a0, c0), pb = camToPart(job, b, a1, c1);
            const double turn = (std::fabs(a1 - a0) + std::fabs(c1 - c0)) * kPiD / 180.0 * (length(a - rc) + gauge);
            const int steps = std::max(1, static_cast<int>(std::ceil((length(pb - pa) + turn) / (cell * 0.7))));
            if (steps > 200000) continue; // a wild turn far from anything; rapids above the stock
            bool crashed = false, shank = false, holder = false, gouged = false;
            for (int st = (len < 1e-9 && a0 == a1 && c0 == c1) ? steps : 0; st <= steps; ++st) {
                const double t = static_cast<double>(st) / steps;
                const Vec3 pm2 = a + (b - a) * static_cast<float>(t);
                const float aa = a0 + (a1 - a0) * static_cast<float>(t), ca = c0 + (c1 - c0) * static_cast<float>(t);
                const Vec3 P = camToPart(job, pm2, aa, ca), u = camToolAxis(aa, ca);
                // The voxels the flute and the shank could reach.
                const double reach = gauge;
                double bl[3], bh[3];
                for (int d = 0; d < 3; ++d) {
                    const double p0 = d == 0 ? P.x : d == 1 ? P.y : P.z, ud = d == 0 ? u.x : d == 1 ? u.y : u.z;
                    const double rad = R * std::sqrt(std::max(0.0, 1.0 - ud * ud)) + cell;
                    bl[d] = std::min(p0, p0 + ud * reach) - rad;
                    bh[d] = std::max(p0, p0 + ud * reach) + rad;
                }
                if (bh[0] < lo.x || bh[1] < lo.y || bh[2] < lo.z || bl[0] > hi[0] || bl[1] > hi[1] || bl[2] > hi[2]) continue;
                const int i0 = std::max(0, static_cast<int>((bl[0] - lo.x) / cell)), i1 = std::min(nx - 1, static_cast<int>((bh[0] - lo.x) / cell));
                const int j0 = std::max(0, static_cast<int>((bl[1] - lo.y) / cell)), j1 = std::min(ny - 1, static_cast<int>((bh[1] - lo.y) / cell));
                const int k0 = std::max(0, static_cast<int>((bl[2] - lo.z) / cell)), k1 = std::min(nz - 1, static_cast<int>((bh[2] - lo.z) / cell));
                for (int k = k0; k <= k1; ++k)
                    for (int j = j0; j <= j1; ++j)
                        for (int i = i0; i <= i1; ++i) {
                            unsigned char& s = v[idx(i, j, k)];
                            if (s == 0) continue;
                            const D3 q = centre(i, j, k);
                            const double dx = q.x - P.x, dy = q.y - P.y, dz = q.z - P.z;
                            const double h = dx * u.x + dy * u.y + dz * u.z;
                            if (h < -cell) continue;
                            const double r2 = dx * dx + dy * dy + dz * dz - h * h;
                            if (r2 > R * R) continue;
                            if (h > flute) { // above the flutes: the shank rubs
                                if (h <= reach && s == 1) shank = true;
                                continue;
                            }
                            if (h < cut.h(std::sqrt(std::max(0.0, r2)))) continue;
                            if (s == 1) {
                                ++removed;
                                if (m.kind == CamMove::Rapid) crashed = true;
                                s = 0;
                            } else if (s == 3) {
                                ++out.gouges; // a voxel of the part's inside
                                gouged = true;
                                s = 0;
                            }
                        }
            }
            // The holder's face at the end of the move: anything within its radius just above it.
            {
                const Vec3 P = camToPart(job, b, m.a, m.c), u = camToolAxis(m.a, m.c);
                const Vec3 F = P + u * static_cast<float>(gauge);
                if (F.x > lo.x - HR && F.y > lo.y - HR && F.z > lo.z - HR && F.x < hi[0] + HR && F.y < hi[1] + HR && F.z < hi[2] + HR) {
                    const int stride = std::max(1, static_cast<int>(HR / (8 * cell)));
                    const int i0 = std::max(0, static_cast<int>((F.x - HR - lo.x) / cell)), i1 = std::min(nx - 1, static_cast<int>((F.x + HR - lo.x) / cell));
                    const int j0 = std::max(0, static_cast<int>((F.y - HR - lo.y) / cell)), j1 = std::min(ny - 1, static_cast<int>((F.y + HR - lo.y) / cell));
                    const int k0 = std::max(0, static_cast<int>((F.z - HR - lo.z) / cell)), k1 = std::min(nz - 1, static_cast<int>((F.z + HR - lo.z) / cell));
                    double worst = -kInf;
                    for (int k = k0; k <= k1; k += stride)
                        for (int j = j0; j <= j1; j += stride)
                            for (int i = i0; i <= i1; i += stride) {
                                if (v[idx(i, j, k)] == 0) continue;
                                const D3 q = centre(i, j, k);
                                const double dx = q.x - P.x, dy = q.y - P.y, dz = q.z - P.z;
                                const double h = dx * u.x + dy * u.y + dz * u.z;
                                if (h < gauge || h > gauge + 3 * cell * stride) continue;
                                if (dx * dx + dy * dy + dz * dz - h * h > HR * HR) continue;
                                holder = true;
                                worst = std::max(worst, h);
                            }
                    if (holder) {
                        // The stickout that would clear it: the farthest material up the axis within the holder's radius.
                        double far = worst;
                        for (int k = 0; k < nz; k += stride)
                            for (int j = j0; j <= j1; j += stride)
                                for (int i = i0; i <= i1; i += stride) {
                                    if (v[idx(i, j, k)] == 0) continue;
                                    const D3 q = centre(i, j, k);
                                    const double dx = q.x - P.x, dy = q.y - P.y, dz = q.z - P.z;
                                    const double h = dx * u.x + dy * u.y + dz * u.z;
                                    if (h > far && dx * dx + dy * dy + dz * dz - h * h <= HR * HR) far = h;
                                }
                        out.neededStickout = std::max(out.neededStickout, far + 1.0);
                        if (out.holderTool.empty()) out.holderTool = "T" + std::to_string(tool->number);
                    }
                }
            }
            out.rapidCrashes += crashed ? 1 : 0;
            out.shankHits += shank ? 1 : 0;
            out.holderHits += holder ? 1 : 0;
            if (gouged) out.deepestGouge = std::max(out.deepestGouge, cell); // at least a voxel
        }
    }
    const double cv = cell * cell * cell;
    out.removed = static_cast<double>(removed) * cv;
    // Stock left on the part: through the stock from the cut (or open) side to each skin voxel.
    std::vector<int> dist(v.size(), -1);
    std::vector<std::size_t> queue;
    for (int k = 0; k < nz; ++k)
        for (int j = 0; j < ny; ++j)
            for (int i = 0; i < nx; ++i) {
                const std::size_t q = idx(i, j, k);
                if (v[q] == 0 || (v[q] == 1 && (i == 0 || j == 0 || k == 0 || i == nx - 1 || j == ny - 1 || k == nz - 1))) {
                    dist[q] = v[q] == 0 ? 0 : 1;
                    queue.push_back(q);
                }
            }
    for (std::size_t h = 0; h < queue.size(); ++h) {
        const std::size_t q = queue[h];
        const int i = static_cast<int>(q % static_cast<std::size_t>(nx)), j = static_cast<int>((q / static_cast<std::size_t>(nx)) % static_cast<std::size_t>(ny)),
                  k = static_cast<int>(q / (static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny)));
        for (int dk = -1; dk <= 1; ++dk)
            for (int dj = -1; dj <= 1; ++dj)
                for (int di = -1; di <= 1; ++di) {
                    const int ii = i + di, jj = j + dj, kk = k + dk;
                    if (ii < 0 || jj < 0 || kk < 0 || ii >= nx || jj >= ny || kk >= nz) continue;
                    const std::size_t w = idx(ii, jj, kk);
                    if (v[w] != 1 || dist[w] >= 0) continue;
                    dist[w] = dist[q] + 1;
                    queue.push_back(w);
                }
    }
    double sumLeft = 0;
    std::size_t skin = 0;
    for (int k = 0; k < nz; ++k)
        for (int j = 0; j < ny; ++j)
            for (int i = 0; i < nx; ++i) {
                if (v[idx(i, j, k)] != 2) continue;
                int best = -1;
                bool open = false;
                for (const auto& [di, dj, dk] : {std::array{1, 0, 0}, std::array{-1, 0, 0}, std::array{0, 1, 0}, std::array{0, -1, 0}, std::array{0, 0, 1}, std::array{0, 0, -1}}) {
                    const int ii = i + di, jj = j + dj, kk = k + dk;
                    if (ii < 0 || jj < 0 || kk < 0 || ii >= nx || jj >= ny || kk >= nz) {
                        open = true;
                        continue;
                    }
                    const std::size_t w = idx(ii, jj, kk);
                    if (v[w] == 0) open = true;
                    else if (v[w] == 1 && dist[w] > 0) best = best < 0 ? dist[w] : std::min(best, dist[w]);
                }
                if (!open && best < 0) continue; // not on the surface
                const double left = open ? 0.0 : best * cell;
                out.maxLeft = std::max(out.maxLeft, left);
                sumLeft += left;
                ++skin;
            }
    out.meanLeft = skin ? sumLeft / static_cast<double>(skin) : 0.0;
    out.cell = cell;
    out.nx = nx;
    out.ny = ny;
    out.vz = nz;
    out.voxels = true;
    out.voxelMin = lo;
    out.voxel = std::move(v);
    (void)error;
    return true;
}

} // namespace

bool simulateJob(CamJob& job, const Body& part, CamSimulation& out, std::string* error) {
    out = CamSimulation{};
    if (!job.setup.hasStock) {
        if (error) *error = "Set up the stock first (CAM SETUP)";
        return false;
    }
    if (job.ops.empty()) {
        if (error) *error = "No operations to simulate";
        return false;
    }
    if (!generateAll(job, part, error)) return false;
    if (job.multiAxis()) return simulateVoxels(job, part, out, error);
    const CamSetup& su = job.setup;
    const double w = su.stockMax.x - su.stockMin.x, hgt = su.stockMax.y - su.stockMin.y;
    double minR = 1e9, maxR = 0;
    for (const CamOperation& op : job.ops)
        if (const CamTool* t = job.tool(op.tool)) minR = std::min(minR, t->radius()), maxR = std::max(maxR, t->radius());
    // An eighth of the smallest tool's radius; coarser when the stock is big, or when the
    // biggest tool would cover more than about 250 000 cells.
    const double cell = std::clamp(std::max({minR / 8.0, std::sqrt(w * hgt / 2.5e6), maxR * 0.0035}), 0.02, 1.0);
    const int nx = std::max(1, static_cast<int>(std::ceil(w / cell))), ny = std::max(1, static_cast<int>(std::ceil(hgt / cell)));
    out.cell = cell;
    out.nx = nx;
    out.ny = ny;
    const double top = su.stockMax.z, x0 = su.stockMin.x, y0 = su.stockMin.y;
    std::vector<double> h(static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny), top);
    const PartModel pm(part, 2.0);
    std::vector<double> partTop(h.size());
    parallelFor(ny, [&](int j) {
        for (int i = 0; i < nx; ++i) partTop[static_cast<std::size_t>(j * nx + i)] = pm.top(x0 + (i + 0.5) * cell, y0 + (j + 0.5) * cell);
    });
    double removed = 0;
    Vec3 at{static_cast<float>((su.stockMin.x + su.stockMax.x) * 0.5), static_cast<float>((su.stockMin.y + su.stockMax.y) * 0.5),
            static_cast<float>(top + su.clearance)};
    int lastTool = -1;
    for (const CamOperation& op : job.ops) {
        const CamTool* tool = job.tool(op.tool);
        if (!tool) continue;
        if (op.tool != lastTool && lastTool >= 0) out.seconds += su.machine.toolChange;
        lastTool = op.tool;
        const Cutter cut = cutterOf(*tool);
        const double R = cut.R;
        out.seconds += movesSeconds(op.moves, su.machine, at);
        if (op.kind == CamOpKind::Drill && op.dwell > 0) out.seconds += op.dwell * static_cast<double>(op.holes.size());
        for (const CamMove& m : op.moves) {
            const Vec3 a = at, b = m.to;
            at = b;
            const double len = length(b - a);
            if (m.kind == CamMove::Rapid) out.rapidLength += len;
            else out.cutLength += len;
            if (std::min(a.z, b.z) >= top - 1e-9) continue; // above the stock
            bool crashed = false, shank = false;
            // The move's sweep, cell by cell: the lowest the tool's cutting edge gets over
            // the cell's centre during the move (exact for flat moves; along a ramp the
            // lowest of the tip height plus the tool's shape, which is convex in time).
            const double ax = a.x, ay = a.y, az = a.z, dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
            const double l2 = dx * dx + dy * dy;
            const int i0 = std::max(0, static_cast<int>(std::floor((std::min(a.x, b.x) - R - x0) / cell)));
            const int i1 = std::min(nx - 1, static_cast<int>(std::floor((std::max(a.x, b.x) + R - x0) / cell)));
            const int j0 = std::max(0, static_cast<int>(std::floor((std::min(a.y, b.y) - R - y0) / cell)));
            const int j1 = std::min(ny - 1, static_cast<int>(std::floor((std::max(a.y, b.y) + R - y0) / cell)));
            const bool flatTool = cut.coneSlope <= 0 && cut.rc <= 0;
            // The holder: anything within its radius higher than its face, before this move cuts.
            if (const double zLow = std::min(a.z, b.z), gauge = tool->gauge(); zLow + gauge < top - 1e-9) {
                const double HR = tool->holderRadius();
                const int stride = std::max(1, static_cast<int>(HR / (16.0 * cell)));
                const int hi0 = std::max(0, static_cast<int>(std::floor((std::min(a.x, b.x) - HR - x0) / cell)));
                const int hi1 = std::min(nx - 1, static_cast<int>(std::floor((std::max(a.x, b.x) + HR - x0) / cell)));
                const int hj0 = std::max(0, static_cast<int>(std::floor((std::min(a.y, b.y) - HR - y0) / cell)));
                const int hj1 = std::min(ny - 1, static_cast<int>(std::floor((std::max(a.y, b.y) + HR - y0) / cell)));
                double worst = -kInf;
                for (int j = hj0; j <= hj1; j += stride)
                    for (int i = hi0; i <= hi1; i += stride) {
                        const double v = h[static_cast<std::size_t>(j * nx + i)];
                        if (v <= zLow + gauge) continue;
                        const double cx = x0 + (i + 0.5) * cell, cy = y0 + (j + 0.5) * cell;
                        const double t = l2 > 0 ? std::clamp(((cx - ax) * dx + (cy - ay) * dy) / l2, 0.0, 1.0) : 0.0;
                        if (std::hypot(cx - ax - dx * t, cy - ay - dy * t) > HR) continue;
                        worst = std::max(worst, v - zLow);
                    }
                if (worst > -kInf) {
                    ++out.holderHits;
                    out.neededStickout = std::max(out.neededStickout, worst + 1.0);
                    if (out.holderTool.empty()) out.holderTool = "T" + std::to_string(tool->number);
                }
            }
            if (l2 < 1e-18 && dz >= 0) continue; // straight up: it cuts nothing new
            for (int j = j0; j <= j1; ++j)
                for (int i = i0; i <= i1; ++i) {
                    const double cx = x0 + (i + 0.5) * cell, cy = y0 + (j + 0.5) * cell;
                    double hz;
                    if (l2 < 1e-18) { // straight up or down: the lower end cuts the most
                        const double d = std::hypot(cx - ax, cy - ay);
                        if (d > R - 1e-6) continue;
                        hz = std::min(az, static_cast<double>(b.z)) + cut.h(d);
                    } else {
                        const double tl = ((cx - ax) * dx + (cy - ay) * dy) / l2; // along the line
                        const double perp = std::fabs((cx - ax) * dy - (cy - ay) * dx) / std::sqrt(l2);
                        if (perp > R - 1e-6) continue;
                        const double half = std::sqrt(std::max(0.0, (R - 1e-6) * (R - 1e-6) - perp * perp) / l2);
                        const double lo = std::max(0.0, tl - half), hi = std::min(1.0, tl + half);
                        if (lo > hi) continue;
                        auto f = [&](double t) { return az + dz * t + cut.h(std::min(R, std::hypot(perp, (tl - t) * std::sqrt(l2)))); };
                        if (std::fabs(dz) < 1e-12) hz = az + cut.h(std::hypot(perp, std::max(0.0, std::max(lo - tl, tl - hi)) * std::sqrt(l2)));
                        else if (flatTool) hz = az + dz * (dz > 0 ? lo : hi);
                        else {
                            constexpr double g = 0.6180339887498949;
                            double u = lo, v2 = hi, p1 = v2 - g * (v2 - u), p2 = u + g * (v2 - u), f1 = f(p1), f2 = f(p2);
                            for (int it = 0; it < 32; ++it) {
                                if (f1 > f2) u = p1, p1 = p2, f1 = f2, p2 = u + g * (v2 - u), f2 = f(p2);
                                else v2 = p2, p2 = p1, f2 = f1, p1 = v2 - g * (v2 - u), f1 = f(p1);
                            }
                            hz = std::min({f(lo), f(hi), f1, f2});
                        }
                    }
                    hz = std::max(hz, static_cast<double>(su.stockMin.z)); // nothing below the stock
                    double& v = h[static_cast<std::size_t>(j * nx + i)];
                    if (v <= hz + 1e-6) continue;
                    if (m.kind == CamMove::Rapid && v > hz + 1e-3) crashed = true; // a micron of stock or more
                    if (v > hz + tool->fluteLength) shank = true;
                    removed += (v - hz) * cell * cell;
                    v = hz;
                }
            out.rapidCrashes += crashed ? 1 : 0;
            out.shankHits += shank ? 1 : 0;
        }
    }
    out.removed = removed;
    const double gougeTol = std::max(2.0 * su.tolerance, 0.02);
    double sumLeft = 0;
    std::size_t onPart = 0;
    // Gouges are measured square to the surface: on a steep wall a cut the tolerance
    // deep sideways reads many times deeper straight down.
    auto slopeCos = [&](int i, int j) {
        auto topAt = [&](int ii, int jj) {
            ii = std::clamp(ii, 0, nx - 1), jj = std::clamp(jj, 0, ny - 1);
            const double v = partTop[static_cast<std::size_t>(jj * nx + ii)];
            return std::isfinite(v) ? v : partTop[static_cast<std::size_t>(j * nx + i)];
        };
        const double dx = (topAt(i + 1, j) - topAt(i - 1, j)) / (2 * cell), dy = (topAt(i, j + 1) - topAt(i, j - 1)) / (2 * cell);
        return 1.0 / std::sqrt(1.0 + dx * dx + dy * dy);
    };
    // A path may run the tolerance sideways into a wall: against the lowest part within
    // that reach of the cell (none at the part's edge), such cells aren't gouges.
    const int reach = static_cast<int>(std::ceil(gougeTol / cell));
    auto lowestNear = [&](int i, int j) {
        double lo = kInf;
        for (int jj = j - reach; jj <= j + reach; ++jj)
            for (int ii = i - reach; ii <= i + reach; ++ii) {
                if ((ii - i) * (ii - i) + (jj - j) * (jj - j) > reach * reach) continue;
                if (ii < 0 || jj < 0 || ii >= nx || jj >= ny) return -kInf;
                lo = std::min(lo, partTop[static_cast<std::size_t>(jj * nx + ii)]);
            }
        return lo;
    };
    for (std::size_t k = 0; k < h.size(); ++k) {
        if (!std::isfinite(partTop[k])) continue;
        const double left = h[k] - partTop[k];
        const int ci = static_cast<int>(k % static_cast<std::size_t>(nx)), cj = static_cast<int>(k / static_cast<std::size_t>(nx));
        if (left < -gougeTol && h[k] < lowestNear(ci, cj) - gougeTol) {
            const double normal = -left * slopeCos(ci, cj);
            if (normal > gougeTol) {
                ++out.gouges;
                out.deepestGouge = std::max(out.deepestGouge, normal);
            }
        }
        out.maxLeft = std::max(out.maxLeft, left);
        sumLeft += std::max(0.0, left);
        ++onPart;
    }
    out.meanLeft = onPart ? sumLeft / static_cast<double>(onPart) : 0.0;
    out.height.resize(h.size());
    out.partTop.resize(h.size());
    for (std::size_t k = 0; k < h.size(); ++k) {
        out.height[k] = static_cast<float>(h[k]);
        out.partTop[k] = static_cast<float>(partTop[k]);
    }
    return true;
}

Body simulatedStock(const CamJob& job, const CamSimulation& sim) {
    MeshData m;
    const CamSetup& su = job.setup;
    if (sim.voxels) { // a face wherever material meets a cut (or the outside)
        const int nx = sim.nx, ny = sim.ny, nz = sim.vz;
        const float c = static_cast<float>(sim.cell);
        auto solid = [&](int i, int j, int k) {
            if (i < 0 || j < 0 || k < 0 || i >= nx || j >= ny || k >= nz) return false;
            return sim.voxel[(static_cast<std::size_t>(k) * static_cast<std::size_t>(ny) + static_cast<std::size_t>(j)) * static_cast<std::size_t>(nx) + static_cast<std::size_t>(i)] != 0;
        };
        for (int k = 0; k < nz; ++k)
            for (int j = 0; j < ny; ++j)
                for (int i = 0; i < nx; ++i) {
                    if (!solid(i, j, k)) continue;
                    const Vec3 p = sim.voxelMin + Vec3{static_cast<float>(i) * c, static_cast<float>(j) * c, static_cast<float>(k) * c};
                    for (int d = 0; d < 6; ++d) {
                        const int ax = d / 2, sg = d % 2 ? 1 : -1;
                        if (solid(i + (ax == 0 ? sg : 0), j + (ax == 1 ? sg : 0), k + (ax == 2 ? sg : 0))) continue;
                        Vec3 n{};
                        (ax == 0 ? n.x : ax == 1 ? n.y : n.z) = static_cast<float>(sg);
                        // The face's corners, counter-clockwise seen from outside.
                        const int u = (ax + 1) % 3, w = (ax + 2) % 3;
                        auto corner = [&](int du, int dw) {
                            float q[3] = {0, 0, 0};
                            q[ax] = sg > 0 ? c : 0;
                            q[u] = static_cast<float>(du) * c;
                            q[w] = static_cast<float>(dw) * c;
                            return p + Vec3{q[0], q[1], q[2]};
                        };
                        const auto base = static_cast<std::uint32_t>(m.vertices.size());
                        for (const auto& [du, dw] : {std::pair{0, 0}, std::pair{1, 0}, std::pair{1, 1}, std::pair{0, 1}}) m.vertices.push_back({corner(du, dw), n});
                        if (sg > 0) m.indices.insert(m.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
                        else m.indices.insert(m.indices.end(), {base, base + 2, base + 1, base, base + 3, base + 2});
                    }
                }
        return meshBody("Stock (simulated)", std::move(m), {0.55f, 0.70f, 0.86f});
    }
    if (sim.nx <= 0 || sim.ny <= 0) return meshBody("Stock (simulated)", m);
    const int k = std::max(1, static_cast<int>(std::ceil(std::max(sim.nx, sim.ny) / 260.0)));
    const int nx = (sim.nx + k - 1) / k, ny = (sim.ny + k - 1) / k;
    const float bottom = su.stockMin.z;
    // Each display node takes the lowest of its block, so narrow cuts still show.
    std::vector<float> z(static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny));
    for (int j = 0; j < ny; ++j)
        for (int i = 0; i < nx; ++i) {
            float v = 1e30f;
            for (int jj = j * k; jj < std::min(sim.ny, (j + 1) * k); ++jj)
                for (int ii = i * k; ii < std::min(sim.nx, (i + 1) * k); ++ii) v = std::min(v, sim.height[static_cast<std::size_t>(jj * sim.nx + ii)]);
            z[static_cast<std::size_t>(j * nx + i)] = std::max(v, bottom);
        }
    const double cw = (su.stockMax.x - su.stockMin.x) / static_cast<double>(nx), ch = (su.stockMax.y - su.stockMin.y) / static_cast<double>(ny);
    auto px = [&](int i) { return static_cast<float>(su.stockMin.x + (i + 0.5) * cw); };
    auto py = [&](int j) { return static_cast<float>(su.stockMin.y + (j + 0.5) * ch); };
    auto zAt = [&](int i, int j) { return z[static_cast<std::size_t>(std::clamp(j, 0, ny - 1) * nx + std::clamp(i, 0, nx - 1))]; };
    for (int j = 0; j < ny; ++j)
        for (int i = 0; i < nx; ++i) {
            const float dx = (zAt(i + 1, j) - zAt(i - 1, j)) / static_cast<float>(2 * cw), dy = (zAt(i, j + 1) - zAt(i, j - 1)) / static_cast<float>(2 * ch);
            m.vertices.push_back({{px(i), py(j), zAt(i, j)}, normalize(Vec3{-dx, -dy, 1.0f})});
        }
    for (int j = 0; j + 1 < ny; ++j)
        for (int i = 0; i + 1 < nx; ++i) {
            const auto a = static_cast<std::uint32_t>(j * nx + i), b = a + 1, c = a + static_cast<std::uint32_t>(nx), d = c + 1;
            m.indices.insert(m.indices.end(), {a, b, d, a, d, c});
        }
    // Sides and bottom.
    auto wall = [&](const std::vector<std::pair<int, int>>& edge, const Vec3& n) {
        const auto base = static_cast<std::uint32_t>(m.vertices.size());
        for (const auto& [i, j] : edge) {
            m.vertices.push_back({{px(i), py(j), zAt(i, j)}, n});
            m.vertices.push_back({{px(i), py(j), bottom}, n});
        }
        for (std::uint32_t q = 0; q + 1 < edge.size(); ++q) {
            const std::uint32_t a = base + 2 * q, b = a + 1, c = a + 2, d = a + 3;
            m.indices.insert(m.indices.end(), {a, b, d, a, d, c});
        }
    };
    std::vector<std::pair<int, int>> e;
    for (int i = nx - 1; i >= 0; --i) e.push_back({i, 0});
    wall(e, {0, -1, 0});
    e.clear();
    for (int i = 0; i < nx; ++i) e.push_back({i, ny - 1});
    wall(e, {0, 1, 0});
    e.clear();
    for (int j = 0; j < ny; ++j) e.push_back({0, j});
    wall(e, {-1, 0, 0});
    e.clear();
    for (int j = ny - 1; j >= 0; --j) e.push_back({nx - 1, j});
    wall(e, {1, 0, 0});
    const auto base = static_cast<std::uint32_t>(m.vertices.size());
    for (const auto& [i, j] : {std::pair{0, 0}, std::pair{nx - 1, 0}, std::pair{nx - 1, ny - 1}, std::pair{0, ny - 1}})
        m.vertices.push_back({{px(i), py(j), bottom}, {0, 0, -1}});
    m.indices.insert(m.indices.end(), {base, base + 2, base + 1, base, base + 3, base + 2});
    Body b = meshBody("Stock (simulated)", std::move(m), {0.55f, 0.70f, 0.86f});
    return b;
}

// --- post processors ------------------------------------------------------------------

namespace {

struct PostStyle {
    std::string key;
    bool fanuc = false;    // Fanuc / Haas number format and program frame
    bool haas = false;
    bool cycles = true;    // canned drilling cycles
    bool toolChange = true;
    bool grbl = false;
};

bool postStyle(const std::string& name, PostStyle* s) {
    std::string k;
    for (char ch : name) k += static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    if (k == "fanuc") *s = {"fanuc", true, false, true, true, false};
    else if (k == "haas") *s = {"haas", true, true, true, true, false};
    else if (k == "linuxcnc" || k == "emc" || k == "mach3" || k == "mach") *s = {"linuxcnc", false, false, true, true, false};
    else if (k == "grbl") *s = {"grbl", false, false, false, false, true};
    else return false;
    return true;
}

class Writer {
public:
    explicit Writer(const PostStyle& s) : style(s) {}
    std::string num(double v) const {
        char b[48];
        if (std::fabs(v) < 5e-5) v = 0;
        std::snprintf(b, sizeof(b), style.fanuc ? "%.4f" : "%.3f", v);
        std::string s = b;
        while (!s.empty() && s.back() == '0') s.pop_back();
        if (!s.empty() && s.back() == '.' && !style.fanuc) s.pop_back();
        if (s == "-0." || s == "-0") s = style.fanuc ? "0." : "0";
        return s;
    }
    void line(const std::string& l) {
        out += l;
        out += '\n';
        ++lines;
    }
    void comment(const std::string& c) {
        std::string t;
        for (char ch : c) t += ch == '(' || ch == ')' ? ' ' : static_cast<char>(style.fanuc ? std::toupper(static_cast<unsigned char>(ch)) : ch);
        line("(" + t + ")");
    }
    // A modal move: only the words that change.
    void move(int g, const D3& p, double f, const std::string& extra = "") {
        std::string l;
        if (g != modalG) l += (style.fanuc ? (g < 10 ? "G0" : "G") : "G") + std::to_string(g);
        auto word = [&](char w, double v, double& last) {
            if (std::isnan(last) || std::fabs(v - last) > 5e-5) {
                if (!l.empty()) l += ' ';
                l += w + num(v);
                last = v;
            }
        };
        word('X', p.x, x);
        word('Y', p.y, y);
        word('Z', p.z, z);
        if (!extra.empty()) l += " " + extra;
        if (g != 0 && (std::isnan(feed) || std::fabs(f - feed) > 0.5)) {
            l += " F" + num(std::round(f));
            feed = f;
        }
        if (l.empty()) return;
        modalG = g;
        line(l);
    }
    void forget() { x = y = z = std::numeric_limits<double>::quiet_NaN(); modalG = -1; }

    const PostStyle& style;
    std::string out;
    std::size_t lines = 0, arcs = 0, cycles = 0;
    int modalG = -1;
    double x = std::numeric_limits<double>::quiet_NaN(), y = x, z = x, feed = x;
};

// Fits arcs to runs of feed moves: from pts[i] as far as a circle (XY, with Z linear in
// the angle: a helix) holds the points within tol. Returns the last index, or i.
std::size_t fitArc(const std::vector<D3>& pts, std::size_t i, double tol, double* cx, double* cy, bool* ccw) {
    const std::size_t n = pts.size();
    std::size_t best = i;
    for (std::size_t j = i + 3; j < n && j <= i + 400; ++j) {
        const std::size_t mid = (i + j) / 2;
        const D3 &a = pts[i], &b = pts[mid], &c = pts[j];
        const double d = 2.0 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
        if (std::fabs(d) < 1e-12) break;
        const double a2 = a.x * a.x + a.y * a.y, b2 = b.x * b.x + b.y * b.y, c2 = c.x * c.x + c.y * c.y;
        const double ox = (a2 * (b.y - c.y) + b2 * (c.y - a.y) + c2 * (a.y - b.y)) / d;
        const double oy = (a2 * (c.x - b.x) + b2 * (a.x - c.x) + c2 * (b.x - a.x)) / d;
        const double r = std::hypot(a.x - ox, a.y - oy);
        if (r > 1000.0 || r < 0.05) break;
        // Same turn all along, every point on the circle, and a sweep under a full turn.
        double sweep = 0;
        bool ok = true;
        int sign = 0;
        std::vector<double> ang{0.0};
        for (std::size_t k = i + 1; k <= j && ok; ++k) {
            const double a0 = std::atan2(pts[k - 1].y - oy, pts[k - 1].x - ox), a1 = std::atan2(pts[k].y - oy, pts[k].x - ox);
            double da = a1 - a0;
            while (da > kPiD) da -= 2 * kPiD;
            while (da < -kPiD) da += 2 * kPiD;
            const int sg = da > 0 ? 1 : -1;
            if (std::fabs(da) < 1e-9 || (sign != 0 && sg != sign) || std::fabs(da) > kPiD * 0.5) ok = false;
            sign = sg;
            sweep += da;
            ang.push_back(sweep);
            if (std::fabs(std::hypot(pts[k].x - ox, pts[k].y - oy) - r) > tol) ok = false;
            // The arc between two points bulges past their chord by the sagitta.
            const double chord = std::hypot(pts[k].x - pts[k - 1].x, pts[k].y - pts[k - 1].y);
            if (chord * chord / (8 * r) > tol * 4) ok = false;
        }
        if (!ok || std::fabs(sweep) > kPiD * 2 * 0.97) break;
        for (std::size_t k = i; k <= j && ok; ++k) {
            const double zk = a.z + (c.z - a.z) * ang[k - i] / sweep;
            if (std::fabs(pts[k].z - zk) > tol) ok = false;
        }
        if (!ok) break;
        best = j;
        *cx = ox;
        *cy = oy;
        *ccw = sign > 0;
    }
    return best;
}

} // namespace

const std::vector<std::string>& camPosts() {
    static const std::vector<std::string> kPosts = {"fanuc", "haas", "grbl", "linuxcnc"};
    return kPosts;
}

std::string postToString(CamJob& job, const Body& part, const std::string& post, CamPostReport* report, std::string* error) {
    PostStyle st;
    if (!postStyle(post, &st)) {
        if (error) *error = "Unknown post '" + post + "' (fanuc, haas, grbl, linuxcnc)";
        return {};
    }
    if (job.ops.empty()) {
        if (error) *error = "No operations to post";
        return {};
    }
    if (!generateAll(job, part, error)) return {};
    const bool multi = job.multiAxis();
    if (multi && st.grbl) {
        if (error) *error = "GRBL posts 3 axes: the job turns the part (rotary or indexed operations)";
        return {};
    }
    for (const CamOperation& op : job.ops)
        if (st.grbl && op.kind == CamOpKind::Drill && op.cycle == DrillCycle::Tap) {
            if (error) *error = "GRBL can't tap (no spindle-synchronised feed): remove " + (op.name.empty() ? std::string("the tapping operation") : op.name);
            return {};
        }
    const CamSetup& su = job.setup;
    const Vec3 o = job.origin();
    auto work = [&](const Vec3& p) { return D3{p.x - o.x, p.y - o.y, p.z - o.z}; };
    const double clearW = job.clearZ() - o.z;
    Writer w(st);
    std::string title;
    for (char ch : (part.name.empty() ? std::string("PART") : part.name)) title += ch == '(' || ch == ')' ? ' ' : ch;
    if (st.fanuc || st.key == "linuxcnc") w.line("%");
    if (st.fanuc) w.line("O1000 (" + [&] {
                             std::string u;
                             for (char ch : title) u += static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
                             return u;
                         }() + ")");
    else w.comment(title);
    w.comment("SI-CDM CAM, " + st.key + " post, work zero G54 at " +
              (multi ? std::string("the rotary centre (A turns about X, C about Z, through it)")
               : su.origin == CamSetup::Origin::Model ? std::string("the model origin")
               : su.origin == CamSetup::Origin::StockTopCentre ? std::string("the stock top centre")
                                                               : std::string("the stock top front left corner")));
    {
        std::vector<int> listed;
        for (const CamOperation& op : job.ops)
            if (std::find(listed.begin(), listed.end(), op.tool) == listed.end()) {
                listed.push_back(op.tool);
                if (const CamTool* t = job.tool(op.tool)) w.comment(t->describe());
            }
    }
    char sb[64];
    std::snprintf(sb, sizeof(sb), "%.3f", std::max(su.tolerance, 0.001));
    if (st.haas) w.line("G00 G17 G40 G49 G80 G90");
    if (st.fanuc) w.line(st.haas ? "G21" : "G21 G17 G40 G49 G80 G90 G94");
    else if (st.grbl) w.line("G21 G17 G90 G94");
    else w.line(std::string("G21 G17 G40 G49 G80 G90 G94 G64 P") + sb);
    w.line("G54");
    int current = -1, n = 0;
    double rpm = -1;
    bool coolant = false;
    const CamMaterial* mat = camMaterial(su.material);
    const bool wantCoolant = !(mat && (mat->key == "wood" || mat->key == "plastic"));
    double seconds = 0;
    Vec3 at{o.x, o.y, static_cast<float>(job.clearZ())};
    double curA = 0, curC = 0; // the table's turn as posted
    for (const CamOperation& op : job.ops) {
        const CamTool* tool = job.tool(op.tool);
        if (!tool || op.moves.empty()) continue;
        if (op.tool != current) {
            if (current >= 0) {
                w.line(st.fanuc ? "M05" : "M5");
                if (coolant) w.line(st.fanuc ? "M09" : "M9");
                coolant = false;
                if (st.fanuc) {
                    w.line(st.haas ? "G00 G53 Z0." : "G91 G28 Z0.");
                    if (!st.haas) w.line("G90");
                    w.line("M01");
                } else if (st.grbl) {
                    w.line("G0 Z" + w.num(clearW));
                } else {
                    w.line("G53 G0 Z0");
                }
                w.forget();
                seconds += su.machine.toolChange;
            }
            current = op.tool;
            if (st.toolChange) {
                if (st.fanuc) w.line("N" + std::to_string(++n) + " (" + [&] {
                                        std::string u;
                                        for (char ch : tool->describe()) u += ch == '(' || ch == ')' ? ' ' : static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
                                        return u;
                                    }() + ")");
                else w.comment(tool->describe());
                w.line(st.fanuc ? "T" + std::to_string(tool->number) + " M06" : "T" + std::to_string(tool->number) + " M6");
            } else {
                if (n++ > 0) w.line("M0 (Change to " + [&] {
                                     std::string u;
                                     for (char ch : tool->describe()) u += ch == '(' || ch == ')' ? ' ' : ch;
                                     return u;
                                 }() + ", set its length, then resume)");
                else w.comment("Load " + tool->describe());
            }
            rpm = -1;
        }
        if (std::fabs(op.rpm - rpm) > 0.5) {
            rpm = op.rpm;
            w.line("S" + std::to_string(static_cast<long>(std::lround(rpm))) + (st.fanuc ? " M03" : " M3"));
            if (st.grbl) w.line("G4 P2");
        }
        w.comment((op.name.empty() ? std::string(camOpKey(op.kind)) : op.name) + " - " + op.summary);
        // Turn the table for this operation, up at the clearance height, the short way round.
        double offA = 0, offC = 0;
        if (multi) {
            const double a0 = op.moves[0].a, c0 = op.moves[0].c;
            offA = 360.0 * std::round((curA - a0) / 360.0);
            offC = 360.0 * std::round((curC - c0) / 360.0);
            if (std::fabs(a0 + offA - curA) > 1e-4 || std::fabs(c0 + offC - curC) > 1e-4) {
                if (!std::isnan(w.z) && w.z < clearW - 1e-6) w.move(0, {w.x, w.y, clearW}, 0);
                w.line((st.fanuc ? "G00 A" : "G0 A") + w.num(a0 + offA) + " C" + w.num(c0 + offC));
                w.modalG = 0;
                curA = a0 + offA, curC = c0 + offC;
            }
        }
        const D3 first = work(op.moves[0].to);
        // To the start: a fresh tool goes over in X and Y first and only then down to the
        // clearance height, with its length offset on (G43), so no Z move runs without it.
        // GRBL has no length offsets (the tool was touched off): up to clearance first.
        if (std::isnan(w.x)) {
            if (st.grbl) {
                w.line("G0 Z" + w.num(clearW));
                w.z = clearW;
                w.modalG = 0;
                w.move(0, {first.x, first.y, clearW}, 0);
            } else {
                w.line((st.fanuc ? "G00 G90 X" : "G0 G90 X") + w.num(first.x) + " Y" + w.num(first.y));
                w.x = first.x, w.y = first.y;
                w.modalG = 0;
                const std::string h = st.fanuc ? (tool->number < 10 ? "0" : "") + std::to_string(tool->number) : std::to_string(tool->number);
                w.line("G43 H" + h + " Z" + w.num(clearW) + (wantCoolant && !coolant ? (st.fanuc ? " M08" : " M8") : ""));
                coolant = coolant || wantCoolant;
                w.z = clearW;
            }
        }
        if (wantCoolant && !coolant) {
            w.line(st.fanuc ? "M08" : "M8");
            coolant = true;
        }
        {
            const Vec3 rc = job.rotaryCentre();
            seconds += movesSeconds(op.moves, su.machine, at, &rc);
        }
        if (op.kind == CamOpKind::Drill && op.dwell > 0) seconds += op.dwell * static_cast<double>(op.holes.size());
        const bool cycle = op.kind == CamOpKind::Drill && st.cycles && !op.holes.empty();
        if (cycle) {
            const double clear = op.moves.front().to.z - o.z;
            w.move(0, {w.x, w.y, clear}, 0);
            bool started = false;
            for (const CamHole& hl : op.holes) {
                const double depth = drillDepth(*tool, op, hl) - o.z, r = hl.top + 2.0 - o.z;
                const D3 p{hl.x - o.x, hl.y - o.y, depth};
                if (op.cycle == DrillCycle::Tap && st.key == "linuxcnc") {
                    w.move(0, {p.x, p.y, clear}, 0);
                    w.move(0, {p.x, p.y, r}, 0);
                    w.line("G33.1 Z" + w.num(depth) + " K" + w.num(tool->pitch));
                    w.z = r;
                    w.modalG = -1; // G33.1 stays the motion mode: the next move names its G
                    w.move(0, {p.x, p.y, clear}, 0);
                    ++w.cycles;
                    continue;
                }
                if (!started) {
                    w.move(0, {p.x, p.y, clear}, 0);
                    std::string g;
                    switch (op.cycle) {
                    case DrillCycle::Spot: g = st.fanuc ? "G82" : "G82"; break;
                    case DrillCycle::Peck: g = "G83"; break;
                    case DrillCycle::ChipBreak: g = "G73"; break;
                    case DrillCycle::Tap: g = "G84"; break;
                    case DrillCycle::Bore: g = "G85"; break;
                    default: g = op.dwell > 0 ? "G82" : "G81";
                    }
                    if (op.cycle == DrillCycle::Tap && st.fanuc && !st.haas) w.line("M29 S" + std::to_string(static_cast<long>(std::lround(op.rpm))));
                    std::string l = "G98 " + g + " X" + w.num(p.x) + " Y" + w.num(p.y) + " Z" + w.num(depth) + " R" + w.num(r);
                    if (op.cycle == DrillCycle::Peck || op.cycle == DrillCycle::ChipBreak) {
                        const CamMaterial* m = mat;
                        const double q = op.peck > 0 ? op.peck : tool->diameter * (m && m->stepdownFactor < 0.4 ? 0.5 : 1.0);
                        l += " Q" + w.num(q);
                    }
                    if (g == "G82") {
                        const double dw = op.dwell > 0 ? op.dwell : 0.3;
                        l += st.fanuc && !st.haas ? " P" + std::to_string(static_cast<long>(std::lround(dw * 1000))) : " P" + w.num(dw);
                    }
                    l += " F" + w.num(std::round(op.feed));
                    w.line(l);
                    w.x = p.x, w.y = p.y;
                    w.feed = op.feed;
                    started = true;
                } else {
                    std::string l;
                    if (std::fabs(p.x - w.x) > 5e-5) l += "X" + w.num(p.x);
                    if (std::fabs(p.y - w.y) > 5e-5) l += std::string(l.empty() ? "" : " ") + "Y" + w.num(p.y);
                    if (!l.empty()) w.line(l);
                    w.x = p.x, w.y = p.y;
                }
                ++w.cycles;
            }
            if (started) w.line("G80");
            w.modalG = 0;
            w.z = clear;
        } else if (op.kind == CamOpKind::Surface5) {
            // Simultaneous 5-axis: every move turns A and C as well. Inverse-time feed (G93):
            // each cutting move takes as long as its tip's path over the part at the feed.
            w.line("G93");
            Vec3 prevPart = camToPart(job, at, op.moves[0].a, op.moves[0].c);
            for (const CamMove& m : op.moves) {
                const D3 p = work(m.to);
                const double A = m.a + offA, C = m.c + offC;
                std::string l = std::string(m.kind == CamMove::Rapid ? (st.fanuc ? "G00" : "G0") : (st.fanuc ? "G01" : "G1")) + " X" + w.num(p.x) + " Y" +
                                w.num(p.y) + " Z" + w.num(p.z) + " A" + w.num(A) + " C" + w.num(C);
                const Vec3 onPart = camToPart(job, m.to, m.a, m.c);
                if (m.kind != CamMove::Rapid) {
                    const double len = std::max(static_cast<double>(length(onPart - prevPart)), 1e-4);
                    l += " F" + w.num(std::min(99999.0, std::max(m.feed, 1.0) / len));
                }
                w.line(l);
                prevPart = onPart;
                curA = A, curC = C;
            }
            w.line("G94");
            const D3 last = work(op.moves.back().to);
            w.x = last.x, w.y = last.y, w.z = last.z;
            w.modalG = -1;
            w.feed = std::numeric_limits<double>::quiet_NaN();
        } else if (op.kind == CamOpKind::Rotary) {
            // The part turns as the tool moves: inverse-time feed (G93), each move taking
            // its length along the part at the feed.
            const Vec3 rc = job.rotaryCentre();
            w.line("G93");
            Vec3 prev = at;
            double prevA = op.moves[0].a;
            for (const CamMove& m : op.moves) {
                const D3 p = work(m.to);
                const double A = m.a + offA;
                std::string l = std::string(m.kind == CamMove::Rapid ? (st.fanuc ? "G00" : "G0") : (st.fanuc ? "G01" : "G1")) + " X" + w.num(p.x) + " Y" +
                                w.num(p.y) + " Z" + w.num(p.z) + " A" + w.num(A);
                if (m.kind != CamMove::Rapid) {
                    double len = length(m.to - prev);
                    const double r = 0.5 * (std::hypot(prev.y - rc.y, prev.z - rc.z) + std::hypot(m.to.y - rc.y, m.to.z - rc.z));
                    len = std::hypot(len, r * (m.a - prevA) * kPiD / 180.0);
                    l += " F" + w.num(std::min(99999.0, std::max(m.feed, 1.0) / std::max(len, 1e-4)));
                }
                w.line(l);
                prev = m.to;
                prevA = m.a;
                curA = A;
            }
            w.line("G94");
            const D3 last = work(op.moves.back().to);
            w.x = last.x, w.y = last.y, w.z = last.z;
            w.modalG = -1;
            w.feed = std::numeric_limits<double>::quiet_NaN();
        } else {
            // Moves, with runs of feed moves fitted with arcs.
            std::size_t k = 0;
            const auto& mv = op.moves;
            while (k < mv.size()) {
                if (mv[k].kind != CamMove::Feed) {
                    w.move(mv[k].kind == CamMove::Rapid ? 0 : 1, work(mv[k].to), mv[k].feed);
                    ++k;
                    continue;
                }
                std::vector<D3> pts{{w.x, w.y, w.z}};
                const double f = mv[k].feed;
                std::size_t e = k;
                while (e < mv.size() && mv[e].kind == CamMove::Feed && std::fabs(mv[e].feed - f) < 0.5) pts.push_back(work(mv[e++].to));
                std::size_t i = 0;
                while (i + 1 < pts.size()) {
                    double cx = 0, cy = 0;
                    bool ccw = false;
                    const std::size_t j = std::isnan(pts[0].x) ? i : fitArc(pts, i, std::max(su.tolerance, 0.001), &cx, &cy, &ccw);
                    if (j > i) {
                        const D3& s = pts[i];
                        const std::string ij = "I" + w.num(cx - s.x) + " J" + w.num(cy - s.y);
                        w.modalG = -1; // arcs always name their G
                        w.move(ccw ? 3 : 2, pts[j], f, ij);
                        ++w.arcs;
                        i = j;
                    } else {
                        w.move(1, pts[i + 1], f);
                        ++i;
                    }
                }
                k = e;
            }
        }
        at = op.moves.back().to;
    }
    w.line(st.fanuc ? "M05" : "M5");
    if (coolant) w.line(st.fanuc ? "M09" : "M9");
    if (st.fanuc) {
        if (st.haas) {
            w.line("G00 G53 Z0.");
            w.line("G53 Y0.");
        } else {
            w.line("G91 G28 Z0.");
            w.line("G28 Y0.");
            w.line("G90");
        }
        w.line("M30");
        w.line("%");
    } else if (st.grbl) {
        w.line("G0 Z" + w.num(clearW));
        w.line("M30");
    } else {
        w.line("G53 G0 Z0");
        w.line("M2");
        w.line("%");
    }
    if (report) {
        report->lines = w.lines;
        report->arcs = w.arcs;
        report->cycles = w.cycles;
        report->seconds = seconds;
    }
    return w.out;
}

bool postJob(CamJob& job, const Body& part, const std::string& post, const std::string& path, CamPostReport* report, std::string* error) {
    const std::string g = postToString(job, part, post, report, error);
    if (g.empty()) return false;
    std::ofstream f(path, std::ios::binary);
    if (!f) {
        if (error) *error = "Can't write " + path;
        return false;
    }
    f << g;
    return static_cast<bool>(f);
}

// --- save and load ----------------------------------------------------------------------

Json camToJson(const CamJob& job) {
    Json j = Json::object();
    const CamSetup& su = job.setup;
    Json s = Json::object();
    s.set("body", su.body);
    if (su.hasStock) {
        Json a = Json::array(), b = Json::array();
        for (float v : {su.stockMin.x, su.stockMin.y, su.stockMin.z}) a.push(v);
        for (float v : {su.stockMax.x, su.stockMax.y, su.stockMax.z}) b.push(v);
        s.set("stock_min", a);
        s.set("stock_max", b);
    }
    s.set("material", su.material);
    s.set("origin", su.origin == CamSetup::Origin::Model ? "model" : su.origin == CamSetup::Origin::StockTopCentre ? "centre" : "corner");
    s.set("clearance", su.clearance);
    s.set("retract", su.retract);
    s.set("tolerance", su.tolerance);
    s.set("post", su.post);
    s.set("max_rpm", su.machine.maxRpm);
    s.set("max_feed", su.machine.maxFeed);
    s.set("rapid", su.machine.rapid);
    {
        const CamMachine& m = su.machine;
        Json mj = Json::object();
        mj.set("name", m.name).set("tool_change", m.toolChange).set("tool_pockets", m.toolPockets);
        Json lo = Json::array(), hi = Json::array();
        for (int i = 0; i < 3; ++i) lo.push(m.travelMin[i]), hi.push(m.travelMax[i]);
        mj.set("travel_min", lo).set("travel_max", hi);
        if (m.hasWorkOffset) {
            Json w = Json::array();
            for (double v : m.workOffset) w.push(v);
            mj.set("work_offset", w);
        }
        mj.set("a", m.hasA ? Json(Json::Array{Json(m.aMin), Json(m.aMax)}) : Json(false));
        mj.set("c", m.hasC ? Json(Json::Array{Json(m.cMin), Json(m.cMax)}) : Json(false));
        s.set("machine", mj);
    }
    j.set("setup", s);
    j.set("show_paths", job.showPaths);
    Json tools = Json::array();
    for (const CamTool& t : job.tools) tools.push(camToolToJson(t));
    j.set("tools", tools);
    Json ops = Json::array();
    for (const CamOperation& op : job.ops) {
        Json o = Json::object();
        o.set("kind", camOpKey(op.kind));
        o.set("name", op.name);
        o.set("tool", op.tool);
        if (!std::isnan(op.top)) o.set("top", op.top);
        if (!std::isnan(op.bottom)) o.set("bottom", op.bottom);
        o.set("stepdown", op.stepdown);
        o.set("stepover", op.stepover);
        o.set("leave", op.leave);
        o.set("angle", op.angle);
        o.set("climb", op.climb);
        o.set("raster", op.raster);
        if (op.adaptive) o.set("adaptive", true);
        if (op.indexA != 0) o.set("index_a", op.indexA);
        if (op.indexC != 0) o.set("index_c", op.indexC);
        if (op.along) o.set("along", true);
        if (op.kind == CamOpKind::Surface5) o.set("max_tilt", op.maxTilt).set("lead_angle", op.leadAngle).set("side_tilt", op.sideTilt);
        if (op.rest) o.set("rest", true);
        o.set("side", op.side);
        o.set("tabs", op.tabs);
        o.set("tab_width", op.tabWidth);
        o.set("tab_height", op.tabHeight);
        o.set("lead", op.lead);
        o.set("scallop", op.scallop);
        o.set("min_slope", op.minSlope);
        o.set("max_slope", op.maxSlope);
        o.set("cycle", drillCycleKey(op.cycle));
        o.set("peck", op.peck);
        o.set("dwell", op.dwell);
        o.set("auto_holes", op.autoHoles);
        if (!op.autoHoles) {
            Json hs = Json::array();
            for (const CamHole& h : op.holes) {
                Json hj = Json::array();
                for (double v : {h.x, h.y, h.diameter, h.top, h.bottom}) hj.push(v);
                hj.push(h.through);
                hs.push(hj);
            }
            o.set("holes", hs);
        }
        o.set("rpm", op.setRpm);
        o.set("feed", op.setFeed);
        o.set("plunge", op.setPlunge);
        ops.push(o);
    }
    j.set("operations", ops);
    return j;
}

bool camFromJson(const Json& j, CamJob& job) {
    if (!j.isObject()) return false;
    job = CamJob{};
    const Json& s = j["setup"];
    CamSetup& su = job.setup;
    su.body = s["body"].asString("");
    if (s["stock_min"].isArray() && s["stock_max"].isArray() && s["stock_min"].size() == 3 && s["stock_max"].size() == 3) {
        su.hasStock = true;
        su.stockMin = {s["stock_min"][0].asFloat(), s["stock_min"][1].asFloat(), s["stock_min"][2].asFloat()};
        su.stockMax = {s["stock_max"][0].asFloat(), s["stock_max"][1].asFloat(), s["stock_max"][2].asFloat()};
    }
    su.material = s["material"].asString("aluminium");
    const std::string origin = s["origin"].asString("corner");
    su.origin = origin == "model" ? CamSetup::Origin::Model : origin == "centre" ? CamSetup::Origin::StockTopCentre : CamSetup::Origin::StockTopCorner;
    su.clearance = s["clearance"].asNumber(15);
    su.retract = s["retract"].asNumber(3);
    su.tolerance = s["tolerance"].asNumber(0.01);
    su.post = s["post"].asString("fanuc");
    su.machine.maxRpm = s["max_rpm"].asNumber(12000);
    su.machine.maxFeed = s["max_feed"].asNumber(5000);
    su.machine.rapid = s["rapid"].asNumber(10000);
    if (const Json& mj = s["machine"]; mj.isObject()) {
        CamMachine& m = su.machine;
        m.name = mj["name"].asString("generic");
        m.toolChange = mj["tool_change"].asNumber(6);
        m.toolPockets = mj["tool_pockets"].asInt(0);
        const auto& lo = mj["travel_min"].items();
        const auto& hi = mj["travel_max"].items();
        for (std::size_t i = 0; i < 3 && i < lo.size() && i < hi.size(); ++i) m.travelMin[i] = lo[i].asNumber(0), m.travelMax[i] = hi[i].asNumber(0);
        const auto& w = mj["work_offset"].items();
        m.hasWorkOffset = w.size() == 3;
        for (std::size_t i = 0; i < 3 && m.hasWorkOffset; ++i) m.workOffset[i] = w[i].asNumber(0);
        const auto& a = mj["a"].items();
        const auto& c = mj["c"].items();
        m.hasA = a.size() == 2;
        if (m.hasA) m.aMin = a[0].asNumber(-99999), m.aMax = a[1].asNumber(99999);
        m.hasC = c.size() == 2;
        if (m.hasC) m.cMin = c[0].asNumber(-99999), m.cMax = c[1].asNumber(99999);
    }
    job.showPaths = j["show_paths"].asBool(true);
    for (const Json& o : j["tools"].items()) job.tools.push_back(camToolFromJson(o));
    for (const Json& o : j["operations"].items()) {
        CamOperation op;
        const std::string kind = o["kind"].asString("rough");
        for (CamOpKind k : {CamOpKind::Face, CamOpKind::Contour, CamOpKind::Pocket, CamOpKind::Rough, CamOpKind::Parallel, CamOpKind::Waterline,
                            CamOpKind::Drill, CamOpKind::Bore, CamOpKind::Rotary, CamOpKind::Surface5})
            if (kind == camOpKey(k)) op.kind = k;
        op.name = o["name"].asString("");
        op.tool = o["tool"].asInt(1);
        if (o["top"].isNumber()) op.top = o["top"].asNumber();
        if (o["bottom"].isNumber()) op.bottom = o["bottom"].asNumber();
        op.stepdown = o["stepdown"].asNumber(0);
        op.stepover = o["stepover"].asNumber(0);
        op.leave = o["leave"].asNumber(0);
        op.angle = o["angle"].asNumber(0);
        op.climb = o["climb"].asBool(true);
        op.raster = o["raster"].asBool(false);
        op.adaptive = o["adaptive"].asBool(false);
        op.indexA = o["index_a"].asNumber(0);
        op.indexC = o["index_c"].asNumber(0);
        op.along = o["along"].asBool(false);
        op.maxTilt = o["max_tilt"].asNumber(60), op.leadAngle = o["lead_angle"].asNumber(10), op.sideTilt = o["side_tilt"].asNumber(0);
        op.rest = o["rest"].asBool(false);
        op.side = o["side"].asInt(0);
        op.tabs = o["tabs"].asInt(0);
        op.tabWidth = o["tab_width"].asNumber(4);
        op.tabHeight = o["tab_height"].asNumber(1.5);
        op.lead = o["lead"].asNumber(-1);
        op.scallop = o["scallop"].asNumber(0.01);
        op.minSlope = o["min_slope"].asNumber(0);
        op.maxSlope = o["max_slope"].asNumber(90);
        if (!drillCycleFromKey(o["cycle"].asString("drill"), &op.cycle)) op.cycle = DrillCycle::Drill;
        op.peck = o["peck"].asNumber(0);
        op.dwell = o["dwell"].asNumber(0);
        op.autoHoles = o["auto_holes"].asBool(true);
        for (const Json& h : o["holes"].items())
            if (h.size() >= 6) op.holes.push_back({h[0].asNumber(), h[1].asNumber(), h[2].asNumber(), h[3].asNumber(), h[4].asNumber(), h[5].asBool()});
        op.setRpm = o["rpm"].asNumber(0);
        op.setFeed = o["feed"].asNumber(0);
        op.setPlunge = o["plunge"].asNumber(0);
        job.ops.push_back(op);
    }
    return true;
}

} // namespace sicdm

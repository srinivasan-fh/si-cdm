#include "core/Cam.hpp"

#include "core/Json.hpp"
#include "core/Scene.hpp"
#include "core/Section.hpp"
#include "core/Sculpt.hpp"

#include <algorithm>
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
            if (std::max(a.z, b.z) > best) best = std::max(best, dropEdge(c, a, b, x, y));
        }
        return best;
    }

    static double dropEdge(const Cutter& c, const D3& a, const D3& b, double x, double y) {
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

CamFeeds camFeeds(const CamTool& t, const CamMaterial& m, const CamMachine& machine, double stepover) {
    CamFeeds f;
    const double d = std::max(t.diameter, 0.1);
    double vc = t.hss ? m.cuttingSpeedHss : m.cuttingSpeedCarbide;
    if (t.type == CamToolType::Drill || t.type == CamToolType::Spot) vc *= 0.7;
    if (t.type == CamToolType::Tap) vc *= 0.15;
    f.rpm = std::min(vc * 1000.0 / (kPiD * d), machine.maxRpm);
    if (t.type == CamToolType::Tap) f.rpm = std::min(f.rpm, 1000.0);
    f.rpm = std::max(100.0, std::floor(f.rpm / 10.0) * 10.0);
    if (t.type == CamToolType::Drill || t.type == CamToolType::Spot || t.type == CamToolType::Tap) {
        const double perRev = t.type == CamToolType::Tap ? t.pitch : std::clamp(m.drillFeedPerDiameter * d, 0.01, 0.5);
        f.feed = std::min(f.rpm * perRev, machine.maxFeed);
        if (t.type == CamToolType::Tap) f.feed = f.rpm * t.pitch; // synchronised: never clamped
        f.plunge = f.feed;
        f.chipLoad = perRev;
        return f;
    }
    double fz = std::clamp(m.chipLoadPerDiameter * d, 0.005, 0.3);
    if (t.type == CamToolType::FaceMill) fz *= 1.2;
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
    if (!setup.hasStock || setup.origin == CamSetup::Origin::Model) return {};
    if (setup.origin == CamSetup::Origin::StockTopCentre)
        return {(setup.stockMin.x + setup.stockMax.x) * 0.5f, (setup.stockMin.y + setup.stockMax.y) * 0.5f, setup.stockMax.z};
    return {setup.stockMin.x, setup.stockMin.y, setup.stockMax.z};
}

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

double movesSeconds(const std::vector<CamMove>& moves, const CamMachine& machine, const Vec3& from) {
    double s = 0;
    Vec3 at = from;
    for (const CamMove& m : moves) {
        const double len = length(m.to - at);
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
    const double s = std::clamp(op.stepover > 0 ? op.stepover : D * 0.4, D * 0.05, D * 0.95);
    const double sd = op.stepdown > 0 ? op.stepdown : defaultStepdown(c);
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
    for (const double z : levels) {
        std::vector<char> in(g.v.size());
        for (int j = 0; j < g.ny; ++j)
            for (int i = 0; i < g.nx; ++i) {
                const P2 q = g.pos(i, j);
                in[static_cast<std::size_t>(j * g.nx + i)] = clg.at(i, j) <= z + 1e-9 && inRect(q.x, q.y);
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
        if (!op.raster) {
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
                    o.feed(l[0].x, l[0].y, z);
                    cutLoop(o, l, z, z);
                } else {
                    // Rapid only to just above the last depth (the tool may overlap stock
                    // not yet cut at this one), then ramp in along the ring.
                    o.travel(l[0], materialTop + 1.0, materialTop + 0.5, floor);
                    cutLoop(o, l, z, materialTop + 0.5);
                }
                entered = true;
            };
            if (op.raster) {
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
        else if (op.kind == CamOpKind::Rough || op.kind == CamOpKind::Pocket) stepover = tool->diameter * 0.4;
        else if (op.kind == CamOpKind::Parallel || op.kind == CamOpKind::Waterline) stepover = scallopStep(*tool, op.scallop);
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
    for (const CamMove& m : op.moves) {
        if (m.kind != CamMove::Rapid) cut += length(m.to - at);
        at = m.to;
    }
    const double secs = movesSeconds(op.moves, su.machine, op.moves.empty() ? Vec3{} : op.moves[0].to);
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
    for (std::size_t k = 0; k < h.size(); ++k) out.height[k] = static_cast<float>(h[k]);
    return true;
}

Body simulatedStock(const CamJob& job, const CamSimulation& sim) {
    MeshData m;
    const CamSetup& su = job.setup;
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
    for (const CamOperation& op : job.ops)
        if (st.grbl && op.kind == CamOpKind::Drill && op.cycle == DrillCycle::Tap) {
            if (error) *error = "GRBL can't tap (no spindle-synchronised feed): remove " + (op.name.empty() ? std::string("the tapping operation") : op.name);
            return {};
        }
    const CamSetup& su = job.setup;
    const Vec3 o = job.origin();
    auto work = [&](const Vec3& p) { return D3{p.x - o.x, p.y - o.y, p.z - o.z}; };
    const double clearW = su.stockMax.z + su.clearance - o.z;
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
              (su.origin == CamSetup::Origin::Model ? std::string("the model origin")
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
    Vec3 at{o.x, o.y, static_cast<float>(su.stockMax.z + su.clearance)};
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
        seconds += movesSeconds(op.moves, su.machine, at);
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
    j.set("setup", s);
    j.set("show_paths", job.showPaths);
    Json tools = Json::array();
    for (const CamTool& t : job.tools) {
        Json o = Json::object();
        o.set("number", t.number);
        o.set("type", camToolTypeKey(t.type));
        o.set("diameter", t.diameter);
        o.set("corner", t.cornerRadius);
        o.set("flute_length", t.fluteLength);
        o.set("angle", t.tipAngle);
        o.set("pitch", t.pitch);
        o.set("flutes", t.flutes);
        o.set("hss", t.hss);
        o.set("name", t.name);
        tools.push(o);
    }
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
    job.showPaths = j["show_paths"].asBool(true);
    for (const Json& o : j["tools"].items()) {
        CamTool t;
        t.number = o["number"].asInt(1);
        if (!camToolTypeFromKey(o["type"].asString("flat"), &t.type)) t.type = CamToolType::Flat;
        t.diameter = o["diameter"].asNumber(6);
        t.cornerRadius = o["corner"].asNumber(0);
        t.fluteLength = o["flute_length"].asNumber(20);
        t.tipAngle = o["angle"].asNumber(118);
        t.pitch = o["pitch"].asNumber(1);
        t.flutes = o["flutes"].asInt(3);
        t.hss = o["hss"].asBool(false);
        t.name = o["name"].asString("");
        job.tools.push_back(t);
    }
    for (const Json& o : j["operations"].items()) {
        CamOperation op;
        const std::string kind = o["kind"].asString("rough");
        for (CamOpKind k : {CamOpKind::Face, CamOpKind::Contour, CamOpKind::Pocket, CamOpKind::Rough, CamOpKind::Parallel, CamOpKind::Waterline,
                            CamOpKind::Drill, CamOpKind::Bore})
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

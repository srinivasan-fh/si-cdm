#include "core/Surfacing.hpp"

#include "core/Scene.hpp"
#include "core/Sculpt.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <tuple>

namespace sicdm {

int bsplineSpan(int n, int p, double u, const std::vector<double>& U) {
    if (u >= U[static_cast<std::size_t>(n + 1)]) return n;
    if (u <= U[static_cast<std::size_t>(p)]) return p;
    int lo = p, hi = n + 1;
    int mid = (lo + hi) / 2;
    while (u < U[static_cast<std::size_t>(mid)] || u >= U[static_cast<std::size_t>(mid + 1)]) {
        if (u < U[static_cast<std::size_t>(mid)]) hi = mid;
        else lo = mid;
        mid = (lo + hi) / 2;
    }
    return mid;
}

void bsplineBasis(int i, double u, int p, const std::vector<double>& U, std::vector<double>& N) {
    N.assign(static_cast<std::size_t>(p + 1), 0.0);
    std::vector<double> left(static_cast<std::size_t>(p + 1)), right(static_cast<std::size_t>(p + 1));
    N[0] = 1.0;
    for (int j = 1; j <= p; ++j) {
        left[static_cast<std::size_t>(j)] = u - U[static_cast<std::size_t>(i + 1 - j)];
        right[static_cast<std::size_t>(j)] = U[static_cast<std::size_t>(i + j)] - u;
        double saved = 0;
        for (int r = 0; r < j; ++r) {
            const double den = right[static_cast<std::size_t>(r + 1)] + left[static_cast<std::size_t>(j - r)];
            const double t = den != 0 ? N[static_cast<std::size_t>(r)] / den : 0.0;
            N[static_cast<std::size_t>(r)] = saved + right[static_cast<std::size_t>(r + 1)] * t;
            saved = left[static_cast<std::size_t>(j - r)] * t;
        }
        N[static_cast<std::size_t>(j)] = saved;
    }
}

void interpolationParams(const std::vector<std::vector<DVec3>>& rows, int degree, std::vector<double>& params, std::vector<double>& knots) {
    const std::size_t m = rows.empty() ? 0 : rows[0].size();
    params.assign(m, 0.0);
    if (m < 2) return;
    // Chord lengths along each row, averaged over the rows.
    std::size_t used = 0;
    for (const auto& row : rows) {
        double total = 0;
        std::vector<double> d(m, 0.0);
        for (std::size_t k = 1; k < m; ++k) total += dlength(row[k] - row[k - 1]), d[k] = total;
        if (!(total > 0)) continue;
        for (std::size_t k = 0; k < m; ++k) params[k] += d[k] / total;
        ++used;
    }
    if (used == 0)
        for (std::size_t k = 0; k < m; ++k) params[k] = static_cast<double>(k) / static_cast<double>(m - 1);
    else
        for (double& t : params) t /= static_cast<double>(used);
    params.front() = 0, params.back() = 1;
    const int n = static_cast<int>(m) - 1, p = std::min(degree, n);
    knots.assign(static_cast<std::size_t>(n + p + 2), 0.0);
    for (int j = 0; j <= p; ++j) knots[static_cast<std::size_t>(n + 1 + j)] = 1.0;
    for (int j = 1; j <= n - p; ++j) {
        double sum = 0;
        for (int i = j; i < j + p; ++i) sum += params[static_cast<std::size_t>(i)];
        knots[static_cast<std::size_t>(j + p)] = sum / p;
    }
}

namespace {

// Solves A X = B (A n x n, B n x 3) by Gaussian elimination with partial pivoting.
bool solveSystem(std::vector<double> A, std::vector<DVec3> B, std::size_t n, std::vector<DVec3>& X) {
    for (std::size_t c = 0; c < n; ++c) {
        std::size_t piv = c;
        for (std::size_t r = c + 1; r < n; ++r)
            if (std::fabs(A[r * n + c]) > std::fabs(A[piv * n + c])) piv = r;
        if (std::fabs(A[piv * n + c]) < 1e-14) return false;
        if (piv != c) {
            for (std::size_t k = 0; k < n; ++k) std::swap(A[c * n + k], A[piv * n + k]);
            std::swap(B[c], B[piv]);
        }
        for (std::size_t r = c + 1; r < n; ++r) {
            const double f = A[r * n + c] / A[c * n + c];
            if (f == 0) continue;
            for (std::size_t k = c; k < n; ++k) A[r * n + k] -= f * A[c * n + k];
            B[r] = B[r] - B[c] * f;
        }
    }
    X.assign(n, {0, 0, 0});
    for (std::size_t r = n; r-- > 0;) {
        DVec3 s = B[r];
        for (std::size_t k = r + 1; k < n; ++k) s = s - X[k] * A[r * n + k];
        X[r] = s / A[r * n + r];
    }
    return true;
}

} // namespace

BSplineCurve interpolateCurve(const std::vector<DVec3>& pts, int degree, const std::vector<double>* paramsIn, const std::vector<double>* knotsIn) {
    BSplineCurve c;
    const int n = static_cast<int>(pts.size()) - 1;
    if (n < 1) return c;
    const int p = std::min(degree, n);
    std::vector<double> params, knots;
    if (paramsIn && knotsIn) params = *paramsIn, knots = *knotsIn;
    else interpolationParams({pts}, p, params, knots);
    const auto m = static_cast<std::size_t>(n + 1);
    std::vector<double> A(m * m, 0.0);
    std::vector<double> N;
    for (std::size_t k = 0; k < m; ++k) {
        const int span = bsplineSpan(n, p, params[k], knots);
        bsplineBasis(span, params[k], p, knots, N);
        for (int i = 0; i <= p; ++i) A[k * m + static_cast<std::size_t>(span - p + i)] = N[static_cast<std::size_t>(i)];
    }
    std::vector<DVec3> cp;
    if (!solveSystem(A, pts, m, cp)) return c;
    c.degree = p;
    c.cp = std::move(cp);
    c.knots = std::move(knots);
    return c;
}

bool interpolateSurface(const std::vector<std::vector<DVec3>>& rows, BSplineSurface& out, std::string* why) {
    auto fail = [&](const char* w) {
        if (why) *why = w;
        return false;
    };
    const std::size_t R = rows.size();
    if (R < 2) return fail("a surface needs two or more sections");
    const std::size_t M = rows[0].size();
    for (const auto& r : rows)
        if (r.size() != M) return fail("the sections have different numbers of points");
    if (M < 2) return fail("a section needs two or more points");
    const int du = std::min<int>(3, static_cast<int>(R) - 1), dv = std::min<int>(3, static_cast<int>(M) - 1);
    std::vector<double> pv, kv, pu, ku;
    interpolationParams(rows, dv, pv, kv);
    std::vector<std::vector<DVec3>> cols(M, std::vector<DVec3>(R));
    for (std::size_t i = 0; i < R; ++i)
        for (std::size_t j = 0; j < M; ++j) cols[j][i] = rows[i][j];
    interpolationParams(cols, du, pu, ku);
    // Along each row, then across the rows' control points.
    std::vector<std::vector<DVec3>> ctrlRows(R);
    for (std::size_t i = 0; i < R; ++i) {
        const BSplineCurve c = interpolateCurve(rows[i], dv, &pv, &kv);
        if (c.cp.size() != M) return fail("a section could not be fitted");
        ctrlRows[i] = c.cp;
    }
    out = {};
    out.du = du, out.dv = dv;
    out.nu = static_cast<int>(R), out.nv = static_cast<int>(M);
    out.cp.assign(R * M, {0, 0, 0});
    for (std::size_t j = 0; j < M; ++j) {
        std::vector<DVec3> col(R);
        for (std::size_t i = 0; i < R; ++i) col[i] = ctrlRows[i][j];
        const BSplineCurve c = interpolateCurve(col, du, &pu, &ku);
        if (c.cp.size() != R) return fail("the sections could not be joined");
        for (std::size_t i = 0; i < R; ++i) out.cp[i * M + j] = c.cp[i];
    }
    out.ku = ku, out.kv = kv;
    return true;
}

DVec3 surfacePoint(const BSplineSurface& s, double u, double v) { return s.eval(u, v); }

DVec3 surfaceNormal(const BSplineSurface& s, double u, double v) {
    const double h = 1e-6;
    const double u0 = std::max(s.u0(), u - h), u1 = std::min(s.u1(), u + h), v0 = std::max(s.v0(), v - h), v1 = std::min(s.v1(), v + h);
    return dnormalize(dcross(s.eval(u1, v) - s.eval(u0, v), s.eval(u, v1) - s.eval(u, v0)));
}

std::shared_ptr<Brep> surfaceBrep(const BSplineSurface& s) {
    auto b = std::make_shared<Brep>();
    BSurface su;
    su.kind = BSurfaceKind::BSpline;
    su.bs = s;
    b->surfaces.push_back(su);
    const double u0 = s.u0(), u1 = s.u1(), v0 = s.v0(), v1 = s.v1();
    const DVec3 c00 = s.eval(u0, v0), c10 = s.eval(u1, v0), c11 = s.eval(u1, v1), c01 = s.eval(u0, v1);
    double size = 0;
    for (const DVec3& p : s.cp) size = std::max(size, dlength(p - s.cp[0]));
    const double tol = 1e-9 * std::max(size, 1.0);
    // The boundary curves: the first and last rows and columns of control points.
    auto rowCurve = [&](int i) { // u fixed: along v
        BSplineCurve c;
        c.degree = s.dv;
        c.knots = s.kv;
        for (int j = 0; j < s.nv; ++j) c.cp.push_back(s.cp[static_cast<std::size_t>(i * s.nv + j)]);
        return c;
    };
    auto colCurve = [&](int j) { // v fixed: along u
        BSplineCurve c;
        c.degree = s.du;
        c.knots = s.ku;
        for (int i = 0; i < s.nu; ++i) c.cp.push_back(s.cp[static_cast<std::size_t>(i * s.nv + j)]);
        return c;
    };
    auto vertex = [&](const DVec3& p) {
        for (std::size_t k = 0; k < b->vertices.size(); ++k)
            if (dlength(b->vertices[k].p - p) <= tol) return static_cast<int>(k);
        b->vertices.push_back({p});
        return static_cast<int>(b->vertices.size()) - 1;
    };
    const int V00 = vertex(c00), V10 = vertex(c10), V11 = vertex(c11), V01 = vertex(c01);
    auto same = [&](const BSplineCurve& a, const BSplineCurve& c) {
        if (a.cp.size() != c.cp.size()) return false;
        for (std::size_t k = 0; k < a.cp.size(); ++k)
            if (dlength(a.cp[k] - c.cp[k]) > tol) return false;
        return true;
    };
    auto collapsed = [&](const BSplineCurve& c) {
        for (const DVec3& p : c.cp)
            if (dlength(p - c.cp[0]) > tol) return false;
        return true;
    };
    struct Side {
        BSplineCurve c;
        int from, to;
    };
    const Side sides[4] = {{colCurve(0), V00, V10}, {rowCurve(s.nu - 1), V10, V11}, {colCurve(s.nv - 1), V01, V11}, {rowCurve(0), V00, V01}};
    // The loop: v0 forwards, u1 forwards, v1 backwards, u0 backwards (counter-clockwise in uv).
    const bool fwd[4] = {true, true, false, false};
    int edgeOf[4] = {-1, -1, -1, -1};
    BLoop loop;
    loop.outer = true;
    for (int k = 0; k < 4; ++k) {
        if (collapsed(sides[k].c)) continue; // a pole: the loop passes through its point
        for (int o = 0; o < k; ++o) // a seam: the same curve as an earlier side
            if (edgeOf[o] >= 0 && same(sides[o].c, sides[k].c)) edgeOf[k] = edgeOf[o];
        if (edgeOf[k] < 0) {
            BCurve bc;
            bc.kind = BCurveKind::BSpline;
            bc.bs = sides[k].c;
            b->curves.push_back(bc);
            b->edges.push_back({sides[k].from, sides[k].to, static_cast<int>(b->curves.size()) - 1, true});
            edgeOf[k] = static_cast<int>(b->edges.size()) - 1;
        }
        loop.edges.push_back({edgeOf[k], fwd[k]});
    }
    BFace f;
    f.surface = 0;
    f.loops.push_back(loop);
    BShell sh;
    sh.closed = false;
    sh.faces.push_back(f);
    sh.name = "surface";
    b->shells.push_back(sh);
    return b;
}

bool makeSurfaceBody(Body& body, const std::shared_ptr<Brep>& brep, std::string* why) {
    TessellateReport rep;
    TessellateOptions o;
    MeshData m = tessellate(*brep, o, &rep);
    if (rep.failedFaces > 0 || m.indices.empty()) {
        if (why) *why = rep.firstError.empty() ? "the surface would not tessellate" : rep.firstError;
        return false;
    }
    m.computeFeatureEdges(30.0f);
    body.mesh = std::move(m);
    body.kind = "mesh";
    body.params.clear();
    body.brep = brep;
    body.brepKey = meshKey(body.mesh);
    return true;
}

bool bodySurface(const Body& body, BSplineSurface& out) {
    if (!body.exact() || body.brep->faceCount() != 1 || body.brep->surfaces.size() != 1) return false;
    if (body.brep->surfaces[0].kind != BSurfaceKind::BSpline) return false;
    out = body.brep->surfaces[0].bs;
    for (DVec3& p : out.cp) p = p + DVec3(body.offset);
    return true;
}

std::vector<DVec3> resample(const std::vector<DVec3>& line, int n, bool closed) {
    std::vector<DVec3> pts = line;
    if (closed && pts.size() > 1 && dlength(pts.front() - pts.back()) > 1e-9) pts.push_back(pts.front());
    std::vector<double> d(pts.size(), 0.0);
    for (std::size_t k = 1; k < pts.size(); ++k) d[k] = d[k - 1] + dlength(pts[k] - pts[k - 1]);
    std::vector<DVec3> out;
    if (pts.size() < 2 || n < 2) return out;
    const double total = d.back();
    std::size_t seg = 1;
    for (int i = 0; i < n; ++i) {
        const double at = total * static_cast<double>(i) / static_cast<double>(n - 1);
        while (seg + 1 < pts.size() && d[seg] < at) ++seg;
        const double len = d[seg] - d[seg - 1];
        const double t = len > 0 ? (at - d[seg - 1]) / len : 0.0;
        out.push_back(pts[seg - 1] + (pts[seg] - pts[seg - 1]) * std::clamp(t, 0.0, 1.0));
    }
    if (closed) out.back() = out.front();
    return out;
}

bool coonsGrid(std::vector<std::vector<DVec3>> sides, int n, std::vector<std::vector<DVec3>>& grid, std::string* why) {
    if (sides.size() != 4) {
        if (why) *why = "a boundary surface takes four curves";
        return false;
    }
    double size = 0;
    for (const auto& s : sides)
        for (const DVec3& p : s) size = std::max(size, dlength(p - sides[0][0]));
    const double tol = 1e-3 * std::max(size, 1.0);
    // Chain them end to end: each one starts where the one before ends.
    std::vector<std::vector<DVec3>> chain{sides[0]};
    std::vector<bool> used{true, false, false, false};
    for (int k = 1; k < 4; ++k) {
        const DVec3 end = chain.back().back();
        bool found = false;
        for (int j = 1; j < 4 && !found; ++j) {
            if (used[static_cast<std::size_t>(j)]) continue;
            auto c = sides[static_cast<std::size_t>(j)];
            if (dlength(c.back() - end) <= tol) std::reverse(c.begin(), c.end());
            if (dlength(c.front() - end) <= tol) {
                chain.push_back(c);
                used[static_cast<std::size_t>(j)] = true;
                found = true;
            }
        }
        if (!found) {
            if (why) *why = "the four curves don't meet end to end";
            return false;
        }
    }
    if (dlength(chain[3].back() - chain[0].front()) > tol) {
        if (why) *why = "the four curves don't close into a loop";
        return false;
    }
    // C0(u) = side 0, D1(v) = side 1, C1(u) = side 2 reversed, D0(v) = side 3 reversed.
    const std::vector<DVec3> C0 = resample(chain[0], n, false), D1 = resample(chain[1], n, false);
    std::vector<DVec3> c1 = chain[2], d0 = chain[3];
    std::reverse(c1.begin(), c1.end());
    std::reverse(d0.begin(), d0.end());
    const std::vector<DVec3> C1 = resample(c1, n, false), D0 = resample(d0, n, false);
    const DVec3 P00 = C0.front(), P10 = C0.back(), P01 = C1.front(), P11 = C1.back();
    grid.assign(static_cast<std::size_t>(n), std::vector<DVec3>(static_cast<std::size_t>(n)));
    for (int i = 0; i < n; ++i) {
        const double u = static_cast<double>(i) / (n - 1);
        for (int j = 0; j < n; ++j) {
            const double v = static_cast<double>(j) / (n - 1);
            const auto I = static_cast<std::size_t>(i), J = static_cast<std::size_t>(j);
            grid[I][J] = C0[I] * (1 - v) + C1[I] * v + D0[J] * (1 - u) + D1[J] * u -
                         (P00 * ((1 - u) * (1 - v)) + P10 * (u * (1 - v)) + P01 * ((1 - u) * v) + P11 * (u * v));
        }
    }
    return true;
}

namespace {

// Welded vertex ids of a mesh (exact positions).
std::vector<int> weldIds(const MeshData& m, std::vector<Vec3>& pos) {
    std::map<std::tuple<float, float, float>, int> id;
    std::vector<int> out(m.vertices.size());
    for (std::size_t i = 0; i < m.vertices.size(); ++i) {
        const Vec3& p = m.vertices[i].position;
        const auto [it, added] = id.try_emplace({p.x + 0.0f, p.y + 0.0f, p.z + 0.0f}, static_cast<int>(pos.size()));
        if (added) pos.push_back(p);
        out[i] = it->second;
    }
    return out;
}

void addTri(MeshData& m, const Vec3& a, const Vec3& b, const Vec3& c) {
    const Vec3 n = normalize(cross(b - a, c - a));
    const auto base = static_cast<std::uint32_t>(m.vertices.size());
    m.vertices.push_back({a, n}), m.vertices.push_back({b, n}), m.vertices.push_back({c, n});
    m.indices.insert(m.indices.end(), {base, base + 1, base + 2});
}

} // namespace

bool thickenMesh(const MeshData& surface, float t, bool both, MeshData& solid, std::string* why) {
    std::vector<Vec3> pos;
    const std::vector<int> id = weldIds(surface, pos);
    std::vector<Vec3> nrm(pos.size(), Vec3{0, 0, 0});
    std::vector<std::array<int, 3>> tris;
    for (std::size_t k = 0; k + 2 < surface.indices.size(); k += 3) {
        const std::array<int, 3> tr{id[surface.indices[k]], id[surface.indices[k + 1]], id[surface.indices[k + 2]]};
        if (tr[0] == tr[1] || tr[1] == tr[2] || tr[0] == tr[2]) continue;
        const Vec3 n = cross(pos[static_cast<std::size_t>(tr[1])] - pos[static_cast<std::size_t>(tr[0])],
                             pos[static_cast<std::size_t>(tr[2])] - pos[static_cast<std::size_t>(tr[0])]);
        for (int v : tr) nrm[static_cast<std::size_t>(v)] += n;
        tris.push_back(tr);
    }
    if (tris.empty()) {
        if (why) *why = "the surface has no faces";
        return false;
    }
    const float up = both ? 0.5f * t : t, down = both ? -0.5f * t : 0.0f;
    std::vector<Vec3> top(pos.size()), bottom(pos.size());
    for (std::size_t v = 0; v < pos.size(); ++v) {
        const Vec3 n = normalize(nrm[v]);
        top[v] = pos[v] + n * up, bottom[v] = pos[v] + n * down;
    }
    std::map<std::pair<int, int>, int> uses;
    for (const auto& tr : tris)
        for (int e = 0; e < 3; ++e) {
            const int a = tr[static_cast<std::size_t>(e)], c = tr[static_cast<std::size_t>((e + 1) % 3)];
            ++uses[{std::min(a, c), std::max(a, c)}];
        }
    solid = {};
    for (const auto& tr : tris) {
        const auto a = static_cast<std::size_t>(tr[0]), b = static_cast<std::size_t>(tr[1]), c = static_cast<std::size_t>(tr[2]);
        addTri(solid, top[a], top[b], top[c]);
        addTri(solid, bottom[a], bottom[c], bottom[b]);
        for (int e = 0; e < 3; ++e) { // walls along the open edges
            const int p = tr[static_cast<std::size_t>(e)], q = tr[static_cast<std::size_t>((e + 1) % 3)];
            if (uses[{std::min(p, q), std::max(p, q)}] != 1) continue;
            const auto P = static_cast<std::size_t>(p), Q = static_cast<std::size_t>(q);
            addTri(solid, top[Q], top[P], bottom[P]);
            addTri(solid, top[Q], bottom[P], bottom[Q]);
        }
    }
    if (meshVolume(solid) < 0)
        for (std::size_t k = 0; k + 2 < solid.indices.size(); k += 3) std::swap(solid.indices[k + 1], solid.indices[k + 2]);
    return true;
}

MeshData knitMeshes(const std::vector<MeshData>& parts, float tol, bool& closed, std::size_t& openEdges) {
    // One set of vertices: points within tol are one.
    std::vector<Vec3> pos;
    std::map<std::tuple<long long, long long, long long>, std::vector<int>> grid;
    const double cell = std::max(static_cast<double>(tol) * 2, 1e-9);
    auto idOf = [&](const Vec3& p) {
        const long long cx = std::llround(p.x / cell), cy = std::llround(p.y / cell), cz = std::llround(p.z / cell);
        for (long long dx = -1; dx <= 1; ++dx)
            for (long long dy = -1; dy <= 1; ++dy)
                for (long long dz = -1; dz <= 1; ++dz) {
                    const auto it = grid.find({cx + dx, cy + dy, cz + dz});
                    if (it == grid.end()) continue;
                    for (int v : it->second)
                        if (length(pos[static_cast<std::size_t>(v)] - p) <= tol) return v;
                }
        pos.push_back(p);
        grid[{cx, cy, cz}].push_back(static_cast<int>(pos.size()) - 1);
        return static_cast<int>(pos.size()) - 1;
    };
    std::vector<std::array<int, 3>> tris;
    for (const MeshData& m : parts)
        for (std::size_t k = 0; k + 2 < m.indices.size(); k += 3) {
            const std::array<int, 3> t{idOf(m.vertices[m.indices[k]].position), idOf(m.vertices[m.indices[k + 1]].position),
                                       idOf(m.vertices[m.indices[k + 2]].position)};
            if (t[0] != t[1] && t[1] != t[2] && t[0] != t[2]) tris.push_back(t);
        }
    // Where one surface's edge has points the other's lacks (their tessellations differ),
    // the edge is split at those points, so both sides share every point.
    for (std::size_t splits = 0; splits < 200000;) {
        std::map<std::pair<int, int>, std::vector<std::size_t>> uses;
        for (std::size_t i = 0; i < tris.size(); ++i)
            for (int e = 0; e < 3; ++e) {
                const int a = tris[i][static_cast<std::size_t>(e)], c = tris[i][static_cast<std::size_t>((e + 1) % 3)];
                uses[{std::min(a, c), std::max(a, c)}].push_back(i);
            }
        std::vector<int> rimVerts;
        std::vector<std::pair<int, int>> rimEdges;
        for (const auto& [e, ts] : uses)
            if (ts.size() == 1) rimEdges.push_back(e), rimVerts.push_back(e.first), rimVerts.push_back(e.second);
        std::sort(rimVerts.begin(), rimVerts.end());
        rimVerts.erase(std::unique(rimVerts.begin(), rimVerts.end()), rimVerts.end());
        std::vector<char> touched(tris.size(), 0);
        const std::size_t before = splits;
        for (const auto& e : rimEdges) {
            const std::size_t ti = uses[e][0];
            if (touched[ti]) continue; // one split per triangle a pass
            const Vec3 A = pos[static_cast<std::size_t>(e.first)], B = pos[static_cast<std::size_t>(e.second)];
            const Vec3 d = B - A;
            const float l2 = dot(d, d);
            if (!(l2 > 0)) continue;
            int best = -1;
            float bestD = tol;
            for (int v : rimVerts) {
                if (v == e.first || v == e.second) continue;
                const Vec3 P = pos[static_cast<std::size_t>(v)];
                const float t = dot(P - A, d) / l2;
                if (t <= 1e-4f || t >= 1 - 1e-4f) continue;
                const float dist = length(A + d * t - P);
                if (dist <= bestD) bestD = dist, best = v;
            }
            if (best < 0) continue;
            const std::array<int, 3> t = tris[ti];
            int k = 0;
            while (!((t[static_cast<std::size_t>(k)] == e.first || t[static_cast<std::size_t>(k)] == e.second) &&
                     (t[static_cast<std::size_t>((k + 1) % 3)] == e.first || t[static_cast<std::size_t>((k + 1) % 3)] == e.second)))
                ++k;
            const int a = t[static_cast<std::size_t>(k)], c = t[static_cast<std::size_t>((k + 1) % 3)], o = t[static_cast<std::size_t>((k + 2) % 3)];
            tris[ti] = {a, best, o};
            tris.push_back({best, c, o});
            touched[ti] = 1;
            touched.push_back(1);
            ++splits;
        }
        if (splits == before) break;
    }
    // One way round: neighbours across an edge run it opposite ways.
    std::map<std::pair<int, int>, std::vector<std::size_t>> edgeTris;
    for (std::size_t i = 0; i < tris.size(); ++i)
        for (int e = 0; e < 3; ++e) {
            const int a = tris[i][static_cast<std::size_t>(e)], c = tris[i][static_cast<std::size_t>((e + 1) % 3)];
            edgeTris[{std::min(a, c), std::max(a, c)}].push_back(i);
        }
    std::vector<int> done(tris.size(), 0);
    auto runs = [](const std::array<int, 3>& t, int a, int c) {
        for (int e = 0; e < 3; ++e)
            if (t[static_cast<std::size_t>(e)] == a && t[static_cast<std::size_t>((e + 1) % 3)] == c) return true;
        return false;
    };
    for (std::size_t s = 0; s < tris.size(); ++s) {
        if (done[s]) continue;
        done[s] = 1;
        std::vector<std::size_t> stack{s};
        while (!stack.empty()) {
            const std::size_t t = stack.back();
            stack.pop_back();
            for (int e = 0; e < 3; ++e) {
                const int a = tris[t][static_cast<std::size_t>(e)], c = tris[t][static_cast<std::size_t>((e + 1) % 3)];
                const auto& nb = edgeTris[{std::min(a, c), std::max(a, c)}];
                if (nb.size() != 2) continue;
                const std::size_t o = nb[0] == t ? nb[1] : nb[0];
                if (done[o]) continue;
                if (runs(tris[o], a, c)) std::swap(tris[o][1], tris[o][2]);
                done[o] = 1;
                stack.push_back(o);
            }
        }
    }
    openEdges = 0;
    closed = true;
    for (const auto& [e, nb] : edgeTris) {
        if (nb.size() == 1) ++openEdges;
        if (nb.size() != 2) closed = false;
    }
    MeshData out;
    for (const auto& t : tris) addTri(out, pos[static_cast<std::size_t>(t[0])], pos[static_cast<std::size_t>(t[1])], pos[static_cast<std::size_t>(t[2])]);
    if (closed && meshVolume(out) < 0)
        for (std::size_t k = 0; k + 2 < out.indices.size(); k += 3) std::swap(out.indices[k + 1], out.indices[k + 2]);
    return out;
}

void meshCurvature(const MeshData& m, std::vector<float>& mean, std::vector<float>& gauss) {
    std::vector<Vec3> pos;
    const std::vector<int> id = weldIds(m, pos);
    const std::size_t V = pos.size();
    std::vector<Vec3> lap(V, Vec3{0, 0, 0}), nrm(V, Vec3{0, 0, 0});
    std::vector<double> area(V, 0.0), angle(V, 0.0);
    std::map<std::pair<int, int>, int> uses;
    for (std::size_t k = 0; k + 2 < m.indices.size(); k += 3) {
        const int t[3] = {id[m.indices[k]], id[m.indices[k + 1]], id[m.indices[k + 2]]};
        if (t[0] == t[1] || t[1] == t[2] || t[0] == t[2]) continue;
        const Vec3 n = cross(pos[static_cast<std::size_t>(t[1])] - pos[static_cast<std::size_t>(t[0])], pos[static_cast<std::size_t>(t[2])] - pos[static_cast<std::size_t>(t[0])]);
        const double a = 0.5 * static_cast<double>(length(n));
        for (int e = 0; e < 3; ++e) {
            const auto i = static_cast<std::size_t>(t[e]), j = static_cast<std::size_t>(t[(e + 1) % 3]), o = static_cast<std::size_t>(t[(e + 2) % 3]);
            ++uses[{std::min(t[e], t[(e + 1) % 3]), std::max(t[e], t[(e + 1) % 3])}];
            // Cotangent weight of edge i-j from the angle at o.
            const Vec3 u = pos[i] - pos[o], w = pos[j] - pos[o];
            const float s = length(cross(u, w));
            const float cot = s > 1e-12f ? dot(u, w) / s : 0.0f;
            lap[i] += (pos[j] - pos[i]) * (0.5f * cot);
            lap[j] += (pos[i] - pos[j]) * (0.5f * cot);
            // The angle at i.
            const Vec3 p = pos[j] - pos[i], q = pos[o] - pos[i];
            angle[i] += std::atan2(static_cast<double>(length(cross(p, q))), static_cast<double>(dot(p, q)));
            area[i] += a / 3;
            nrm[i] += n;
        }
    }
    std::vector<char> rim(V, 0);
    for (const auto& [e, n] : uses)
        if (n == 1) rim[static_cast<std::size_t>(e.first)] = rim[static_cast<std::size_t>(e.second)] = 1;
    std::vector<float> H(V, 0.0f), K(V, 0.0f);
    for (std::size_t v = 0; v < V; ++v) {
        if (!(area[v] > 0) || rim[v]) continue;
        const Vec3 Hn = lap[v] * static_cast<float>(1.0 / area[v]); // = -2 H n (mean curvature normal)
        H[v] = -0.5f * dot(Hn, normalize(nrm[v]));
        K[v] = static_cast<float>((2 * 3.14159265358979 - angle[v]) / area[v]);
    }
    mean.assign(m.vertices.size(), 0.0f);
    gauss.assign(m.vertices.size(), 0.0f);
    for (std::size_t i = 0; i < m.vertices.size(); ++i) mean[i] = H[static_cast<std::size_t>(id[i])], gauss[i] = K[static_cast<std::size_t>(id[i])];
}

namespace {

// A polyline with its points' share of the length (0 .. 1).
struct Poly {
    std::vector<DVec3> p;
    std::vector<double> at;
    explicit Poly(std::vector<DVec3> pts) : p(std::move(pts)) {
        at.assign(p.size(), 0.0);
        for (std::size_t i = 1; i < p.size(); ++i) at[i] = at[i - 1] + dlength(p[i] - p[i - 1]);
        const double total = at.back();
        for (double& a : at) a = total > 0 ? a / total : 0.0;
    }
    DVec3 eval(double a) const {
        a = std::clamp(a, 0.0, 1.0);
        const std::size_t k = std::min<std::size_t>(static_cast<std::size_t>(std::upper_bound(at.begin(), at.end(), a) - at.begin()), p.size() - 1);
        if (k == 0) return p.front();
        const double d = at[k] - at[k - 1], f = d > 0 ? (a - at[k - 1]) / d : 0.0;
        return p[k - 1] + (p[k] - p[k - 1]) * f;
    }
    void reverse() {
        std::reverse(p.begin(), p.end());
        for (double& a : at) a = 1.0 - a;
        std::reverse(at.begin(), at.end());
    }
};

// Where two polylines come closest: their parameters and the distance.
double closestBetween(const Poly& A, const Poly& B, double& a, double& b) {
    double best = 1e300;
    for (std::size_t i = 1; i < A.p.size(); ++i)
        for (std::size_t j = 1; j < B.p.size(); ++j) {
            const DVec3 p0 = A.p[i - 1], d1 = A.p[i] - p0, q0 = B.p[j - 1], d2 = B.p[j] - q0, r = p0 - q0;
            const double aa = ddot(d1, d1), ee = ddot(d2, d2), ff = ddot(d2, r);
            double s = 0, t = 0;
            if (aa <= 1e-24 && ee <= 1e-24) {
            } else if (aa <= 1e-24) t = std::clamp(ff / ee, 0.0, 1.0);
            else {
                const double c = ddot(d1, r);
                if (ee <= 1e-24) s = std::clamp(-c / aa, 0.0, 1.0);
                else {
                    const double bb = ddot(d1, d2), den = aa * ee - bb * bb;
                    s = den > 1e-24 ? std::clamp((bb * ff - c * ee) / den, 0.0, 1.0) : 0.0;
                    t = (bb * s + ff) / ee;
                    if (t < 0) t = 0, s = std::clamp(-c / aa, 0.0, 1.0);
                    else if (t > 1) t = 1, s = std::clamp((bb - c) / aa, 0.0, 1.0);
                }
            }
            const double d = dlength((p0 + d1 * s) - (q0 + d2 * t));
            if (d < best) {
                best = d;
                a = A.at[i - 1] + (A.at[i] - A.at[i - 1]) * s;
                b = B.at[j - 1] + (B.at[j] - B.at[j - 1]) * t;
            }
        }
    return best;
}

// A curve through points at the given (increasing) parameters, cubic where it can be.
BSplineCurve curveAt(const std::vector<DVec3>& pts, const std::vector<double>& params) {
    const int n = static_cast<int>(pts.size()) - 1, p = std::min(3, n);
    std::vector<double> knots(static_cast<std::size_t>(n + p + 2), 0.0);
    for (int i = 0; i <= p; ++i) knots[static_cast<std::size_t>(n + 1 + i)] = 1.0;
    for (int j = 1; j <= n - p; ++j) {
        double sum = 0;
        for (int i = j; i < j + p; ++i) sum += params[static_cast<std::size_t>(i)];
        knots[static_cast<std::size_t>(j + p)] = sum / p;
    }
    return interpolateCurve(pts, p, &params, &knots);
}

// A curve's position at `a` when its parameter is bent so that `from[k]` lands on `to[k]`
// (both increasing, 0 and 1 at the ends): the network's crossings line up.
double bend(double a, const std::vector<double>& from, const std::vector<double>& to) {
    std::vector<double> f{0.0}, t{0.0};
    f.insert(f.end(), from.begin(), from.end()), t.insert(t.end(), to.begin(), to.end());
    f.push_back(1.0), t.push_back(1.0);
    for (std::size_t k = 1; k < t.size(); ++k)
        if (a <= t[k] || k + 1 == t.size()) {
            const double d = t[k] - t[k - 1], u = d > 0 ? (a - t[k - 1]) / d : 0.0;
            return f[k - 1] + (f[k] - f[k - 1]) * u;
        }
    return a;
}

} // namespace

bool gordonGrid(std::vector<std::vector<DVec3>> profilePts, std::vector<std::vector<DVec3>> guidePts, int n, double tol, std::vector<std::vector<DVec3>>& grid,
                std::string* why) {
    auto fail = [&](const std::string& w) {
        if (why) *why = w;
        return false;
    };
    if (profilePts.size() < 2 || guidePts.size() < 2) return fail("a network needs two or more curves each way");
    std::vector<Poly> U, V;
    for (auto& c : profilePts) {
        if (c.size() < 2) return fail("a curve has no length");
        U.emplace_back(std::move(c));
    }
    for (auto& c : guidePts) {
        if (c.size() < 2) return fail("a curve has no length");
        V.emplace_back(std::move(c));
    }
    const std::size_t m = U.size(), k = V.size();
    // Where each profile crosses each guide.
    std::vector<std::vector<double>> a(m, std::vector<double>(k)), b(m, std::vector<double>(k));
    auto cross = [&]() {
        for (std::size_t i = 0; i < m; ++i)
            for (std::size_t j = 0; j < k; ++j)
                if (closestBetween(U[i], V[j], a[i][j], b[i][j]) > tol) return false;
        return true;
    };
    if (!cross()) return fail("every profile must cross every guide (some pair doesn't meet)");
    // Each curve running the same way: the profiles' crossings in the guides' order, and back.
    for (std::size_t i = 0; i < m; ++i)
        if (a[i].front() > a[i].back()) U[i].reverse();
    for (std::size_t j = 0; j < k; ++j)
        if (b.front()[j] > b.back()[j]) V[j].reverse();
    cross();
    // The guides in order along the profiles, the profiles in order along the guides.
    std::vector<std::size_t> gi(k), pi(m);
    for (std::size_t j = 0; j < k; ++j) gi[j] = j;
    for (std::size_t i = 0; i < m; ++i) pi[i] = i;
    std::sort(gi.begin(), gi.end(), [&](std::size_t x, std::size_t y) { return a[0][x] < a[0][y]; });
    std::sort(pi.begin(), pi.end(), [&](std::size_t x, std::size_t y) { return b[x][0] < b[y][0]; });
    std::vector<Poly> U2, V2;
    for (std::size_t i : pi) U2.push_back(U[i]);
    for (std::size_t j : gi) V2.push_back(V[j]);
    U = std::move(U2), V = std::move(V2);
    cross();
    // Common parameters: t_j along the profiles, s_i along the guides.
    std::vector<double> t(k, 0.0), s(m, 0.0);
    for (std::size_t j = 0; j < k; ++j) {
        for (std::size_t i = 0; i < m; ++i) t[j] += a[i][j] / static_cast<double>(m), s[i] += b[i][j] / static_cast<double>(k);
    }
    for (std::size_t j = 1; j < k; ++j)
        if (!(t[j] > t[j - 1])) return fail("the guides must cross the profiles in the same order");
    for (std::size_t i = 1; i < m; ++i)
        if (!(s[i] > s[i - 1])) return fail("the profiles must cross the guides in the same order");
    // Bent to them: profile i at t_j and guide j at s_i are their crossing.
    auto profileAt = [&](std::size_t i, double tt) { return U[i].eval(bend(tt, a[i], t)); };
    auto guideAt = [&](std::size_t j, double ss) {
        std::vector<double> from(m);
        for (std::size_t i = 0; i < m; ++i) from[i] = b[i][j];
        return V[j].eval(bend(ss, from, s));
    };
    // The crossings (midway between the two curves where they come closest).
    std::vector<std::vector<DVec3>> X(m, std::vector<DVec3>(k));
    for (std::size_t i = 0; i < m; ++i)
        for (std::size_t j = 0; j < k; ++j) X[i][j] = (U[i].eval(a[i][j]) + V[j].eval(b[i][j])) * 0.5;
    // Lu + Lv - T on the grid.
    n = std::max(n, 3);
    grid.assign(static_cast<std::size_t>(n), std::vector<DVec3>(static_cast<std::size_t>(n)));
    std::vector<double> ts(static_cast<std::size_t>(n)), ss(static_cast<std::size_t>(n));
    for (int q = 0; q < n; ++q) { // from the first curve each way to the last
        const double f = static_cast<double>(q) / (n - 1);
        ts[static_cast<std::size_t>(q)] = t.front() + (t.back() - t.front()) * f;
        ss[static_cast<std::size_t>(q)] = s.front() + (s.back() - s.front()) * f;
    }
    // Profiles' loft: for each t, a curve across the profiles at s_i.
    std::vector<std::vector<DVec3>> Lu(static_cast<std::size_t>(n), std::vector<DVec3>(static_cast<std::size_t>(n)));
    for (int q = 0; q < n; ++q) {
        std::vector<DVec3> pts(m);
        for (std::size_t i = 0; i < m; ++i) pts[i] = profileAt(i, ts[static_cast<std::size_t>(q)]);
        const BSplineCurve c = curveAt(pts, s);
        for (int r = 0; r < n; ++r) Lu[static_cast<std::size_t>(r)][static_cast<std::size_t>(q)] = c.eval(std::clamp(ss[static_cast<std::size_t>(r)], s.front(), s.back()));
    }
    // Guides' loft: for each s, a curve across the guides at t_j.
    std::vector<std::vector<DVec3>> Lv(static_cast<std::size_t>(n), std::vector<DVec3>(static_cast<std::size_t>(n)));
    for (int r = 0; r < n; ++r) {
        std::vector<DVec3> pts(k);
        for (std::size_t j = 0; j < k; ++j) pts[j] = guideAt(j, ss[static_cast<std::size_t>(r)]);
        const BSplineCurve c = curveAt(pts, t);
        for (int q = 0; q < n; ++q) Lv[static_cast<std::size_t>(r)][static_cast<std::size_t>(q)] = c.eval(std::clamp(ts[static_cast<std::size_t>(q)], t.front(), t.back()));
    }
    // The crossings' surface, the same way: across the profiles, then along.
    std::vector<BSplineCurve> cols(k);
    for (std::size_t j = 0; j < k; ++j) {
        std::vector<DVec3> pts(m);
        for (std::size_t i = 0; i < m; ++i) pts[i] = X[i][j];
        cols[j] = curveAt(pts, s);
    }
    for (int r = 0; r < n; ++r) {
        std::vector<DVec3> pts(k);
        for (std::size_t j = 0; j < k; ++j) pts[j] = cols[j].eval(std::clamp(ss[static_cast<std::size_t>(r)], s.front(), s.back()));
        const BSplineCurve c = curveAt(pts, t);
        for (int q = 0; q < n; ++q) {
            const DVec3 T = c.eval(std::clamp(ts[static_cast<std::size_t>(q)], t.front(), t.back()));
            grid[static_cast<std::size_t>(r)][static_cast<std::size_t>(q)] = Lu[static_cast<std::size_t>(r)][static_cast<std::size_t>(q)] +
                                                                             Lv[static_cast<std::size_t>(r)][static_cast<std::size_t>(q)] - T;
        }
    }
    return true;
}

double netBending(const BSplineSurface& s) {
    auto P = [&](int i, int j) { return s.cp[static_cast<std::size_t>(i * s.nv + j)]; };
    double e = 0;
    for (int i = 0; i < s.nu; ++i)
        for (int j = 0; j < s.nv; ++j) {
            if (i > 0 && i + 1 < s.nu) e += ddot(P(i - 1, j) - P(i, j) * 2.0 + P(i + 1, j), P(i - 1, j) - P(i, j) * 2.0 + P(i + 1, j));
            if (j > 0 && j + 1 < s.nv) e += ddot(P(i, j - 1) - P(i, j) * 2.0 + P(i, j + 1), P(i, j - 1) - P(i, j) * 2.0 + P(i, j + 1));
        }
    return e;
}

bool fairSurface(BSplineSurface& s, int iterations, bool keepTangent, double& before, double& after, double& moved, std::string* why) {
    const int hold = keepTangent ? 2 : 1; // rows held at each edge
    auto isFree = [&](int i, int j) { return i >= hold && i < s.nu - hold && j >= hold && j < s.nv - hold; };
    bool any = false;
    for (int i = 0; i < s.nu && !any; ++i)
        for (int j = 0; j < s.nv && !any; ++j) any = isFree(i, j);
    before = after = netBending(s);
    moved = 0;
    if (!any) {
        if (why) *why = "too few control points to move with its edges held (fair a denser surface, or FREE the tangents)";
        return false;
    }
    const BSplineSurface original = s;
    auto idx = [&](int i, int j) { return static_cast<std::size_t>(i * s.nv + j); };
    std::vector<DVec3> grad(s.cp.size());
    for (int it = 0; it < iterations; ++it) {
        // The bending's gradient: each second difference d pushes its three points by (1, -2, 1) d.
        std::fill(grad.begin(), grad.end(), DVec3{});
        for (int i = 0; i < s.nu; ++i)
            for (int j = 0; j < s.nv; ++j) {
                if (i > 0 && i + 1 < s.nu) {
                    const DVec3 d = s.cp[idx(i - 1, j)] - s.cp[idx(i, j)] * 2.0 + s.cp[idx(i + 1, j)];
                    grad[idx(i - 1, j)] = grad[idx(i - 1, j)] + d * 2.0, grad[idx(i, j)] = grad[idx(i, j)] - d * 4.0, grad[idx(i + 1, j)] = grad[idx(i + 1, j)] + d * 2.0;
                }
                if (j > 0 && j + 1 < s.nv) {
                    const DVec3 d = s.cp[idx(i, j - 1)] - s.cp[idx(i, j)] * 2.0 + s.cp[idx(i, j + 1)];
                    grad[idx(i, j - 1)] = grad[idx(i, j - 1)] + d * 2.0, grad[idx(i, j)] = grad[idx(i, j)] - d * 4.0, grad[idx(i, j + 1)] = grad[idx(i, j + 1)] + d * 2.0;
                }
            }
        // A step under 1 / (the Hessian's largest diagonal, 24): the bending only goes down.
        for (int i = 0; i < s.nu; ++i)
            for (int j = 0; j < s.nv; ++j)
                if (isFree(i, j)) s.cp[idx(i, j)] = s.cp[idx(i, j)] - grad[idx(i, j)] * (1.0 / 48.0);
    }
    after = netBending(s);
    constexpr int kN = 21;
    for (int a = 0; a < kN; ++a)
        for (int b = 0; b < kN; ++b) {
            const double u = s.u0() + (s.u1() - s.u0()) * a / (kN - 1), v = s.v0() + (s.v1() - s.v0()) * b / (kN - 1);
            moved = std::max(moved, dlength(s.eval(u, v) - original.eval(u, v)));
        }
    return true;
}

} // namespace sicdm


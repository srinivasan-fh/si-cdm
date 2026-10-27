#include "core/Collide.hpp"

#include "core/Physics.hpp"
#include "core/Scene.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
#include <unordered_map>

namespace sicdm {

namespace {

using V = std::array<double, 3>;

V sub(const V& a, const V& b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
V add(const V& a, const V& b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
V scale(const V& a, double s) { return {a[0] * s, a[1] * s, a[2] * s}; }
double dot(const V& a, const V& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
V cross(const V& a, const V& b) { return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]}; }
double norm(const V& a) { return std::sqrt(dot(a, a)); }

V mul(const std::array<double, 9>& R, const V& p) {
    return {R[0] * p[0] + R[1] * p[1] + R[2] * p[2], R[3] * p[0] + R[4] * p[1] + R[5] * p[2], R[6] * p[0] + R[7] * p[1] + R[8] * p[2]};
}
V mulT(const std::array<double, 9>& R, const V& p) { // by the transpose
    return {R[0] * p[0] + R[3] * p[1] + R[6] * p[2], R[1] * p[0] + R[4] * p[1] + R[7] * p[2], R[2] * p[0] + R[5] * p[1] + R[8] * p[2]};
}
std::array<double, 9> mulTM(const std::array<double, 9>& A, const std::array<double, 9>& B) { // A^T B
    std::array<double, 9> C{};
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) C[r * 3 + c] = A[0 * 3 + r] * B[0 * 3 + c] + A[1 * 3 + r] * B[1 * 3 + c] + A[2 * 3 + r] * B[2 * 3 + c];
    return C;
}

// --- triangle against triangle (Moller's interval test) -----------------------------------------

// Where the line of the two planes crosses a triangle: an interval on it. False when the
// triangle lies in the other's plane.
bool interval(double p0, double p1, double p2, double d0, double d1, double d2, double& a, double& b) {
    auto isect = [&](double v0, double v1, double v2, double e0, double e1, double e2) {
        a = v0 + (v1 - v0) * e0 / (e0 - e1);
        b = v0 + (v2 - v0) * e0 / (e0 - e2);
    };
    if (d0 * d1 > 0) isect(p2, p0, p1, d2, d0, d1);
    else if (d0 * d2 > 0) isect(p1, p0, p2, d1, d0, d2);
    else if (d1 * d2 > 0 || d0 != 0) isect(p0, p1, p2, d0, d1, d2);
    else if (d1 != 0) isect(p1, p0, p2, d1, d0, d2);
    else if (d2 != 0) isect(p2, p0, p1, d2, d0, d1);
    else return false;
    if (a > b) std::swap(a, b);
    return true;
}

bool segments2d(double ax, double ay, double bx, double by, double cx, double cy, double dx, double dy) {
    auto orient = [](double px, double py, double qx, double qy, double rx, double ry) { return (qx - px) * (ry - py) - (qy - py) * (rx - px); };
    const double o1 = orient(ax, ay, bx, by, cx, cy), o2 = orient(ax, ay, bx, by, dx, dy);
    const double o3 = orient(cx, cy, dx, dy, ax, ay), o4 = orient(cx, cy, dx, dy, bx, by);
    if (((o1 > 0 && o2 < 0) || (o1 < 0 && o2 > 0)) && ((o3 > 0 && o4 < 0) || (o3 < 0 && o4 > 0))) return true;
    auto on = [](double px, double py, double qx, double qy, double rx, double ry) { // r on pq (collinear)
        return std::min(px, qx) <= rx && rx <= std::max(px, qx) && std::min(py, qy) <= ry && ry <= std::max(py, qy);
    };
    return (o1 == 0 && on(ax, ay, bx, by, cx, cy)) || (o2 == 0 && on(ax, ay, bx, by, dx, dy)) || (o3 == 0 && on(cx, cy, dx, dy, ax, ay)) ||
           (o4 == 0 && on(cx, cy, dx, dy, bx, by));
}

bool inside2d(double px, double py, const double* x, const double* y) {
    const double d1 = (x[1] - x[0]) * (py - y[0]) - (y[1] - y[0]) * (px - x[0]);
    const double d2 = (x[2] - x[1]) * (py - y[1]) - (y[2] - y[1]) * (px - x[1]);
    const double d3 = (x[0] - x[2]) * (py - y[2]) - (y[0] - y[2]) * (px - x[2]);
    return (d1 >= 0 && d2 >= 0 && d3 >= 0) || (d1 <= 0 && d2 <= 0 && d3 <= 0);
}

// Two triangles in one plane: crossing edges, or one inside the other.
bool coplanar(const V& n, const V* v, const V* u) {
    // Onto the axis plane the normal is most along.
    const double ax = std::fabs(n[0]), ay = std::fabs(n[1]), az = std::fabs(n[2]);
    int i0 = 1, i1 = 2;
    if (ay >= ax && ay >= az) i0 = 0, i1 = 2;
    else if (az >= ax && az >= ay) i0 = 0, i1 = 1;
    double vx[3], vy[3], ux[3], uy[3];
    for (int k = 0; k < 3; ++k) vx[k] = v[k][i0], vy[k] = v[k][i1], ux[k] = u[k][i0], uy[k] = u[k][i1];
    for (int e = 0; e < 3; ++e)
        for (int f = 0; f < 3; ++f)
            if (segments2d(vx[e], vy[e], vx[(e + 1) % 3], vy[(e + 1) % 3], ux[f], uy[f], ux[(f + 1) % 3], uy[(f + 1) % 3])) return true;
    return inside2d(vx[0], vy[0], ux, uy) || inside2d(ux[0], uy[0], vx, vy);
}

bool triTri(const V* v, const V* u) {
    const double eps = 1e-9;
    V n1 = cross(sub(v[1], v[0]), sub(v[2], v[0]));
    const double l1 = norm(n1);
    if (l1 < 1e-18) return false; // a degenerate triangle has no inside to cross
    n1 = scale(n1, 1.0 / l1);
    const double e1 = -dot(n1, v[0]);
    double du[3];
    for (int k = 0; k < 3; ++k) {
        du[k] = dot(n1, u[k]) + e1;
        if (std::fabs(du[k]) < eps) du[k] = 0;
    }
    if (du[0] * du[1] > 0 && du[0] * du[2] > 0) return false; // all of u on one side
    V n2 = cross(sub(u[1], u[0]), sub(u[2], u[0]));
    const double l2 = norm(n2);
    if (l2 < 1e-18) return false;
    n2 = scale(n2, 1.0 / l2);
    const double e2 = -dot(n2, u[0]);
    double dv[3];
    for (int k = 0; k < 3; ++k) {
        dv[k] = dot(n2, v[k]) + e2;
        if (std::fabs(dv[k]) < eps) dv[k] = 0;
    }
    if (dv[0] * dv[1] > 0 && dv[0] * dv[2] > 0) return false;
    // Both cross the line where the planes meet: do their stretches of it overlap?
    const V d = cross(n1, n2);
    int idx = 0;
    if (std::fabs(d[1]) > std::fabs(d[idx])) idx = 1;
    if (std::fabs(d[2]) > std::fabs(d[idx])) idx = 2;
    double a0, a1, b0, b1;
    if (!interval(v[0][idx], v[1][idx], v[2][idx], dv[0], dv[1], dv[2], a0, a1)) return coplanar(n1, v, u);
    if (!interval(u[0][idx], u[1][idx], u[2][idx], du[0], du[1], du[2], b0, b1)) return coplanar(n1, v, u);
    return !(a1 < b0 || b1 < a0);
}

bool rayTri(const V& o, const V& d, const V& a, const V& b, const V& c) {
    const V e1 = sub(b, a), e2 = sub(c, a), p = cross(d, e2);
    const double det = dot(e1, p);
    if (std::fabs(det) < 1e-14) return false;
    const double inv = 1.0 / det;
    const V s = sub(o, a);
    const double u = dot(s, p) * inv;
    if (u < 0 || u > 1) return false;
    const V q = cross(s, e1);
    const double v = dot(d, q) * inv;
    if (v < 0 || u + v > 1) return false;
    return dot(e2, q) * inv > 0;
}

} // namespace

V Rigid::apply(const V& p) const { return add(mul(R, p), t); }

Rigid rigidOf(const Mat4& m) {
    Rigid r;
    for (int row = 0; row < 3; ++row) {
        for (int c = 0; c < 3; ++c) r.R[static_cast<std::size_t>(row * 3 + c)] = m.m[static_cast<std::size_t>(c)][static_cast<std::size_t>(row)];
        r.t[static_cast<std::size_t>(row)] = m.m[3][static_cast<std::size_t>(row)];
    }
    return r;
}

bool obbOverlap(const V& ca, const std::array<double, 9>& Ra, const V& ha, const V& cb, const std::array<double, 9>& Rb, const V& hb) {
    V A[3], B[3];
    for (std::size_t i = 0; i < 3; ++i) {
        A[i] = {Ra[i], Ra[3 + i], Ra[6 + i]};
        B[i] = {Rb[i], Rb[3 + i], Rb[6 + i]};
    }
    double R[3][3], AR[3][3];
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j) {
            R[i][j] = dot(A[i], B[j]);
            AR[i][j] = std::fabs(R[i][j]) + 1e-9;
        }
    const V d = sub(cb, ca);
    const double t[3] = {dot(d, A[0]), dot(d, A[1]), dot(d, A[2])};
    for (std::size_t i = 0; i < 3; ++i)
        if (std::fabs(t[i]) > ha[i] + hb[0] * AR[i][0] + hb[1] * AR[i][1] + hb[2] * AR[i][2]) return false;
    for (std::size_t j = 0; j < 3; ++j)
        if (std::fabs(t[0] * R[0][j] + t[1] * R[1][j] + t[2] * R[2][j]) > ha[0] * AR[0][j] + ha[1] * AR[1][j] + ha[2] * AR[2][j] + hb[j]) return false;
    for (std::size_t i = 0; i < 3; ++i) {
        const std::size_t i1 = (i + 1) % 3, i2 = (i + 2) % 3;
        for (std::size_t j = 0; j < 3; ++j) {
            const std::size_t j1 = (j + 1) % 3, j2 = (j + 2) % 3;
            const double ra = ha[i1] * AR[i2][j] + ha[i2] * AR[i1][j];
            const double rb = hb[j1] * AR[i][j2] + hb[j2] * AR[i][j1];
            if (std::fabs(t[i2] * R[i1][j] - t[i1] * R[i2][j]) > ra + rb) return false;
        }
    }
    return true;
}

CollisionShape::CollisionShape(const MeshData& mesh, const Vec3& offset) {
    const std::size_t nt = mesh.indices.size() / 3;
    if (nt == 0) return;
    auto at = [&](std::uint32_t i) -> V {
        const Vec3& p = mesh.vertices[i].position;
        return {static_cast<double>(p.x) + offset.x, static_cast<double>(p.y) + offset.y, static_cast<double>(p.z) + offset.z};
    };
    // Welded corners (a CAD mesh repeats a vertex for each smooth patch): to find the points
    // and whether every edge has a triangle on each side.
    std::map<std::array<float, 3>, std::uint32_t> weld;
    std::vector<std::uint32_t> id(mesh.vertices.size());
    for (std::size_t i = 0; i < mesh.vertices.size(); ++i) {
        const Vec3& p = mesh.vertices[i].position;
        const auto [it, fresh] = weld.emplace(std::array<float, 3>{p.x, p.y, p.z}, static_cast<std::uint32_t>(m_points.size()));
        if (fresh) m_points.push_back(at(static_cast<std::uint32_t>(i)));
        id[i] = it->second;
    }
    std::unordered_map<std::uint64_t, int> edges;
    m_tris.reserve(nt);
    for (std::size_t t = 0; t < nt; ++t) {
        const std::uint32_t i0 = mesh.indices[3 * t], i1 = mesh.indices[3 * t + 1], i2 = mesh.indices[3 * t + 2];
        if (i0 >= mesh.vertices.size() || i1 >= mesh.vertices.size() || i2 >= mesh.vertices.size()) continue;
        m_tris.push_back({at(i0), at(i1), at(i2)});
        const std::uint32_t w[3] = {id[i0], id[i1], id[i2]};
        for (int e = 0; e < 3; ++e) {
            const std::uint32_t p = w[e], q = w[(e + 1) % 3];
            if (p == q) continue;
            ++edges[(static_cast<std::uint64_t>(std::min(p, q)) << 32) | std::max(p, q)];
        }
    }
    m_closed = !edges.empty() && std::all_of(edges.begin(), edges.end(), [](const auto& e) { return e.second == 2; });
    // The tree: split at the middle of the triangles' centres along their widest spread.
    std::vector<std::uint32_t> order(m_tris.size());
    std::iota(order.begin(), order.end(), 0u);
    std::vector<V> centre(m_tris.size());
    for (std::size_t i = 0; i < m_tris.size(); ++i) centre[i] = scale(add(add(m_tris[i].a, m_tris[i].b), m_tris[i].c), 1.0 / 3.0);
    struct Job {
        std::uint32_t node, first, count;
    };
    m_nodes.push_back({});
    std::vector<Job> jobs{{0, 0, static_cast<std::uint32_t>(order.size())}};
    while (!jobs.empty()) {
        const Job j = jobs.back();
        jobs.pop_back();
        Node n;
        n.lo = {1e300, 1e300, 1e300};
        n.hi = {-1e300, -1e300, -1e300};
        V clo = n.lo, chi = n.hi;
        for (std::uint32_t k = j.first; k < j.first + j.count; ++k) {
            const Tri& t = m_tris[order[k]];
            for (const V* p : {&t.a, &t.b, &t.c})
                for (int c = 0; c < 3; ++c) n.lo[c] = std::min(n.lo[c], (*p)[c]), n.hi[c] = std::max(n.hi[c], (*p)[c]);
            for (int c = 0; c < 3; ++c) clo[c] = std::min(clo[c], centre[order[k]][c]), chi[c] = std::max(chi[c], centre[order[k]][c]);
        }
        int axis = 0;
        for (int c = 1; c < 3; ++c)
            if (chi[c] - clo[c] > chi[axis] - clo[axis]) axis = c;
        if (j.count <= 4 || chi[axis] - clo[axis] <= 0) {
            n.first = j.first, n.count = j.count;
            m_nodes[j.node] = n;
            continue;
        }
        const double mid = 0.5 * (clo[axis] + chi[axis]);
        auto split = std::partition(order.begin() + j.first, order.begin() + j.first + j.count, [&](std::uint32_t t) { return centre[t][axis] < mid; });
        std::uint32_t nl = static_cast<std::uint32_t>(split - (order.begin() + j.first));
        if (nl == 0 || nl == j.count) nl = j.count / 2;
        n.left = static_cast<std::uint32_t>(m_nodes.size());
        n.right = n.left + 1;
        m_nodes.push_back({});
        m_nodes.push_back({});
        m_nodes[j.node] = n;
        jobs.push_back({n.left, j.first, nl});
        jobs.push_back({n.right, j.first + nl, j.count - nl});
    }
    std::vector<Tri> sorted(m_tris.size());
    for (std::size_t i = 0; i < order.size(); ++i) sorted[i] = m_tris[order[i]];
    m_tris = std::move(sorted);
}

V CollisionShape::lo() const { return m_nodes.empty() ? V{} : m_nodes.front().lo; }
V CollisionShape::hi() const { return m_nodes.empty() ? V{} : m_nodes.front().hi; }

bool CollisionShape::contains(const V& p) const {
    if (!m_closed || m_nodes.empty()) return false;
    // A ray in an odd direction (it misses edges and vertices): an odd number of crossings.
    const V d{0.5773502, 0.5774502, 0.5772502};
    int crossings = 0;
    std::vector<std::uint32_t> stack{0};
    while (!stack.empty()) {
        const Node& n = m_nodes[stack.back()];
        stack.pop_back();
        double t0 = 0, t1 = 1e300;
        bool hit = true;
        for (int c = 0; c < 3 && hit; ++c) {
            const double inv = 1.0 / d[c];
            double a = (n.lo[c] - p[c]) * inv, b = (n.hi[c] - p[c]) * inv;
            if (a > b) std::swap(a, b);
            t0 = std::max(t0, a), t1 = std::min(t1, b);
            hit = t0 <= t1;
        }
        if (!hit) continue;
        if (n.count) {
            for (std::uint32_t k = n.first; k < n.first + n.count; ++k) crossings += rayTri(p, d, m_tris[k].a, m_tris[k].b, m_tris[k].c) ? 1 : 0;
        } else {
            stack.push_back(n.left);
            stack.push_back(n.right);
        }
    }
    return crossings % 2 == 1;
}

bool shapesCollide(const CollisionShape& a, const Rigid& ta, const CollisionShape& b, const Rigid& tb, V* where) {
    if (a.empty() || b.empty()) return false;
    // b in a's frame.
    const std::array<double, 9> R = mulTM(ta.R, tb.R);
    const V t = mulT(ta.R, sub(tb.t, ta.t));
    static const std::array<double, 9> I{1, 0, 0, 0, 1, 0, 0, 0, 1};
    auto boxes = [&](const CollisionShape::Node& x, const CollisionShape::Node& y) {
        const V cx = scale(add(x.lo, x.hi), 0.5), hx = scale(sub(x.hi, x.lo), 0.5);
        const V cy = add(mul(R, scale(add(y.lo, y.hi), 0.5)), t), hy = scale(sub(y.hi, y.lo), 0.5);
        return obbOverlap(cx, I, hx, cy, R, hy);
    };
    if (!boxes(a.m_nodes[0], b.m_nodes[0])) return false;
    std::vector<std::pair<std::uint32_t, std::uint32_t>> stack{{0, 0}};
    while (!stack.empty()) {
        const auto [ia, ib] = stack.back();
        stack.pop_back();
        const auto& x = a.m_nodes[ia];
        const auto& y = b.m_nodes[ib];
        if (!boxes(x, y)) continue;
        if (x.count && y.count) {
            for (std::uint32_t i = x.first; i < x.first + x.count; ++i) {
                const V v[3] = {a.m_tris[i].a, a.m_tris[i].b, a.m_tris[i].c};
                for (std::uint32_t k = y.first; k < y.first + y.count; ++k) {
                    const V u[3] = {add(mul(R, b.m_tris[k].a), t), add(mul(R, b.m_tris[k].b), t), add(mul(R, b.m_tris[k].c), t)};
                    if (triTri(v, u)) {
                        if (where) *where = ta.apply(scale(add(add(v[0], v[1]), v[2]), 1.0 / 3.0));
                        return true;
                    }
                }
            }
            continue;
        }
        // Down the bigger box (or the one that isn't a leaf).
        const V sx = sub(x.hi, x.lo), sy = sub(y.hi, y.lo);
        const bool downA = !x.count && (y.count || dot(sx, sx) >= dot(sy, sy));
        if (downA) stack.push_back({x.left, ib}), stack.push_back({x.right, ib});
        else stack.push_back({ia, y.left}), stack.push_back({ia, y.right});
    }
    // No surfaces cross: one may still be wholly inside the other.
    if (b.closed() && !a.m_points.empty()) {
        const V p = mulT(tb.R, sub(ta.apply(a.m_points.front()), tb.t)); // into b's frame
        if (b.contains(p)) {
            if (where) *where = ta.apply(a.m_points.front());
            return true;
        }
    }
    if (a.closed() && !b.m_points.empty()) {
        const V p = mulT(ta.R, sub(tb.apply(b.m_points.front()), ta.t));
        if (a.contains(p)) {
            if (where) *where = tb.apply(b.m_points.front());
            return true;
        }
    }
    return false;
}

MotionCheck checkMotion(const Scene& scene, float t0, float t1, float dt, bool all, const std::vector<std::size_t>& only) {
    MotionCheck out;
    const auto& bodies = scene.bodies();
    const std::size_t n = bodies.size();
    if (t1 < t0) std::swap(t0, t1);
    if (dt <= 0) dt = std::max((t1 - t0) / 200.0f, 1e-3f);
    std::vector<std::unique_ptr<CollisionShape>> shape(n);
    for (std::size_t i = 0; i < n; ++i)
        if (bodies[i].visible && !bodies[i].mesh.indices.empty()) shape[i] = std::make_unique<CollisionShape>(bodies[i].mesh, bodies[i].offset);
    std::vector<char> focus(n, only.empty() ? 1 : 0);
    for (std::size_t i : only)
        if (i < n) focus[i] = 1;
    // World boxes where the bodies are at time t.
    auto place = [&](float t, std::vector<Rigid>& at, std::vector<std::array<V, 2>>& box) {
        const std::vector<Mat4> tf = simulationTransforms(scene, t);
        at.assign(n, Rigid{});
        box.assign(n, {});
        for (std::size_t i = 0; i < n; ++i) {
            if (!shape[i]) continue;
            if (i < tf.size()) at[i] = rigidOf(tf[i]);
            const V lo = shape[i]->lo(), hi = shape[i]->hi();
            V wlo{1e300, 1e300, 1e300}, whi{-1e300, -1e300, -1e300};
            for (int c = 0; c < 8; ++c) {
                const V p = at[i].apply({(c & 1) ? hi[0] : lo[0], (c & 2) ? hi[1] : lo[1], (c & 4) ? hi[2] : lo[2]});
                for (int k = 0; k < 3; ++k) wlo[k] = std::min(wlo[k], p[k]), whi[k] = std::max(whi[k], p[k]);
            }
            box[i] = {wlo, whi};
        }
    };
    auto touching = [&](const std::vector<Rigid>& at, const std::vector<std::array<V, 2>>& box, std::size_t i, std::size_t j, V* where) {
        for (int k = 0; k < 3; ++k)
            if (box[i][1][k] < box[j][0][k] || box[j][1][k] < box[i][0][k]) return false;
        return shapesCollide(*shape[i], at[i], *shape[j], at[j], where);
    };
    auto samePlace = [](const Rigid& x, const Rigid& y) {
        for (int k = 0; k < 9; ++k)
            if (std::fabs(x.R[static_cast<std::size_t>(k)] - y.R[static_cast<std::size_t>(k)]) > 1e-9) return false;
        for (int k = 0; k < 3; ++k)
            if (std::fabs(x.t[static_cast<std::size_t>(k)] - y.t[static_cast<std::size_t>(k)]) > 1e-6) return false;
        return true;
    };
    std::vector<Rigid> start, at;
    std::vector<std::array<V, 2>> startBox, box;
    place(t0, start, startBox);
    // Pairs touching where they start (assembled parts, parts on a joint) are left out.
    std::vector<std::vector<char>> skip(n, std::vector<char>(n, 0));
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = i + 1; j < n; ++j) {
            if (!shape[i] || !shape[j]) continue;
            if (!all && touching(start, startBox, i, j, nullptr)) skip[i][j] = 1, ++out.pairsLeftOut;
        }
    std::map<std::pair<std::size_t, std::size_t>, std::size_t> found; // pair -> clash
    for (float t = t0;; t += dt) {
        const float now = std::min(t, t1);
        place(now, at, box);
        ++out.steps;
        // Sweep along x: only boxes that overlap there are compared.
        std::vector<std::size_t> order;
        for (std::size_t i = 0; i < n; ++i)
            if (shape[i]) order.push_back(i);
        std::sort(order.begin(), order.end(), [&](std::size_t x, std::size_t y) { return box[x][0][0] < box[y][0][0]; });
        for (std::size_t p = 0; p < order.size(); ++p)
            for (std::size_t q = p + 1; q < order.size() && box[order[q]][0][0] <= box[order[p]][1][0]; ++q) {
                const std::size_t i = std::min(order[p], order[q]), j = std::max(order[p], order[q]);
                if (skip[i][j] || (!focus[i] && !focus[j])) continue;
                // Two parts that haven't moved since the start are as they were then.
                if (samePlace(at[i], start[i]) && samePlace(at[j], start[j]) && !all) continue;
                V where{};
                if (!touching(at, box, i, j, &where)) continue;
                const auto it = found.find({i, j});
                if (it == found.end()) {
                    found[{i, j}] = out.clashes.size();
                    out.clashes.push_back({bodies[i].name, bodies[j].name, now, now, where});
                } else {
                    out.clashes[it->second].to = now;
                }
            }
        if (now >= t1) break;
    }
    return out;
}

} // namespace sicdm

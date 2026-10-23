#include "core/Assembly.hpp"

#include "core/Json.hpp"
#include "core/Scene.hpp"
#include "core/Sculpt.hpp"
#include "core/Step.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <functional>
#include <map>
#include <set>

namespace sicdm {

namespace {

constexpr double kPiA = 3.14159265358979323846;
constexpr double kLever = 10.0; // mm: a direction's error counts as this lever arm's

std::string lowerOf(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

// --- rotations (row major 3 x 3) --------------------------------------------------------

struct Rot {
    std::array<double, 9> m{1, 0, 0, 0, 1, 0, 0, 0, 1};
    DVec3 operator*(const DVec3& v) const {
        return {m[0] * v.x + m[1] * v.y + m[2] * v.z, m[3] * v.x + m[4] * v.y + m[5] * v.z, m[6] * v.x + m[7] * v.y + m[8] * v.z};
    }
    Rot operator*(const Rot& o) const {
        Rot r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                double v = 0;
                for (int k = 0; k < 3; ++k) v += m[static_cast<std::size_t>(i * 3 + k)] * o.m[static_cast<std::size_t>(k * 3 + j)];
                r.m[static_cast<std::size_t>(i * 3 + j)] = v;
            }
        return r;
    }
    Rot transposed() const {
        Rot r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) r.m[static_cast<std::size_t>(i * 3 + j)] = m[static_cast<std::size_t>(j * 3 + i)];
        return r;
    }
    bool identity(double tol) const {
        static const std::array<double, 9> I{1, 0, 0, 0, 1, 0, 0, 0, 1};
        for (std::size_t i = 0; i < 9; ++i)
            if (std::fabs(m[i] - I[i]) > tol) return false;
        return true;
    }
};

// Rotation by the vector w (axis times angle, radians).
Rot expRot(const DVec3& w) {
    const double th = dlength(w);
    Rot r;
    if (th < 1e-300) return r;
    const DVec3 k = w / th;
    const double c = std::cos(th), s = std::sin(th), t = 1 - c;
    r.m = {t * k.x * k.x + c,       t * k.x * k.y - s * k.z, t * k.x * k.z + s * k.y,
           t * k.x * k.y + s * k.z, t * k.y * k.y + c,       t * k.y * k.z - s * k.x,
           t * k.x * k.z - s * k.y, t * k.y * k.z + s * k.x, t * k.z * k.z + c};
    return r;
}

Rot bodyRot(const Body& b) {
    const Placement p = eulerPlacement(decimal(b.rotation));
    Rot r;
    for (std::size_t i = 0; i < 9; ++i) r.m[i] = p.r[i];
    return r;
}

// Degrees about x, then y, then z (R = Rz Ry Rx), as eulerRotation() takes them.
DVec3 eulerOf(const Rot& R) {
    Placement p;
    for (std::size_t i = 0; i < 9; ++i) p.r[i] = R.m[i];
    return eulerDegrees(p);
}

double bodyScale(const Body& b) { return b.scale > 0 ? decimal(b.scale) : 1.0; }

int findBodyIndex(const Scene& s, const std::string& name) {
    const auto& bs = s.bodies();
    for (std::size_t i = 0; i < bs.size(); ++i)
        if (bs[i].name == name) return static_cast<int>(i);
    for (std::size_t i = 0; i < bs.size(); ++i)
        if (lowerOf(bs[i].name) == lowerOf(name)) return static_cast<int>(i);
    return -1;
}

DVec3 closestOnTriangle(const DVec3& p, const DVec3& a, const DVec3& b, const DVec3& c) {
    const DVec3 ab = b - a, ac = c - a, ap = p - a;
    const double d1 = ddot(ab, ap), d2 = ddot(ac, ap);
    if (d1 <= 0 && d2 <= 0) return a;
    const DVec3 bp = p - b;
    const double d3 = ddot(ab, bp), d4 = ddot(ac, bp);
    if (d3 >= 0 && d4 <= d3) return b;
    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) return a + ab * (d1 / (d1 - d3));
    const DVec3 cp = p - c;
    const double d5 = ddot(ab, cp), d6 = ddot(ac, cp);
    if (d6 >= 0 && d5 <= d6) return c;
    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) return a + ac * (d2 / (d2 - d6));
    const double va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    const double den = 1 / (va + vb + vc);
    return a + ab * (vb * den) + ac * (vc * den);
}

// The mesh triangle nearest p: its index, or -1.
int nearestTriangle(const MeshData& m, const DVec3& p) {
    int best = -1;
    double bd = 1e300;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const DVec3 a(m.vertices[m.indices[t]].position), b(m.vertices[m.indices[t + 1]].position), c(m.vertices[m.indices[t + 2]].position);
        const DVec3 q = closestOnTriangle(p, a, b, c);
        const double d = ddot(q - p, q - p);
        if (d < bd) bd = d, best = static_cast<int>(t / 3);
    }
    return best;
}

// --- the world pose of a part while solving ------------------------------------------------

struct Unit {
    std::vector<std::size_t> bodies;
    bool fixed = false;
    int order = 0;       // when it was first mated: later parts move more readily
    DVec3 c{0, 0, 0};    // the centre it turns about
    Rot q;               // the turn so far
    DVec3 v{0, 0, 0};    // the shift so far
    DVec3 apply(const DVec3& x) const { return q * (x - c) + c + v; }
};

struct WorldRef {
    MateGeom geom;
    DVec3 p, d;
};

void push3(std::vector<double>& r, const DVec3& v, double k = 1) {
    r.push_back(v.x * k);
    r.push_back(v.y * k);
    r.push_back(v.z * k);
}

// The mate's errors (zero when met). Directions are weighted by the lever arm.
void mateResiduals(const Mate& m, WorldRef A, WorldRef B, std::vector<double>& r) {
    const double L = kLever;
    auto order = [&](MateGeom first) { // put the `first` kind in A
        if (A.geom != first && B.geom == first) std::swap(A, B);
    };
    switch (m.type) {
    case MateType::Fix: return;
    case MateType::Parallel: push3(r, dcross(A.d, B.d), L); return;
    case MateType::Perpendicular: r.push_back(ddot(A.d, B.d) * L); return;
    case MateType::Angle: r.push_back((ddot(A.d, B.d) - std::cos(m.value * kPiA / 180)) * L); return;
    case MateType::Coincident:
    case MateType::Distance: {
        const double off = m.type == MateType::Distance ? m.value : 0.0;
        if (A.geom == MateGeom::Plane && B.geom == MateGeom::Plane) {
            push3(r, m.aligned ? A.d - B.d : A.d + B.d, L);
            r.push_back(ddot(B.p - A.p, A.d) - off);
        } else if (A.geom == MateGeom::Plane || B.geom == MateGeom::Plane) {
            order(MateGeom::Plane);
            if (B.geom == MateGeom::Axis) r.push_back(ddot(B.d, A.d) * L);
            r.push_back(ddot(B.p - A.p, A.d) - off);
        } else if (A.geom == MateGeom::Axis && B.geom == MateGeom::Axis) {
            push3(r, dcross(A.d, B.d), L);
            if (m.type == MateType::Distance) r.push_back(dlength(dcross(B.p - A.p, A.d)) - off);
            else push3(r, dcross(B.p - A.p, A.d));
        } else if (A.geom == MateGeom::Axis || B.geom == MateGeom::Axis) {
            order(MateGeom::Axis);
            if (m.type == MateType::Distance) r.push_back(dlength(dcross(B.p - A.p, A.d)) - off);
            else push3(r, dcross(B.p - A.p, A.d));
        } else {
            if (m.type == MateType::Distance) r.push_back(dlength(B.p - A.p) - off);
            else push3(r, B.p - A.p);
        }
        return;
    }
    case MateType::Concentric: {
        if (A.geom == MateGeom::Axis && B.geom == MateGeom::Axis) {
            push3(r, dcross(A.d, B.d), L);
            push3(r, dcross(B.p - A.p, A.d));
        } else if (A.geom == MateGeom::Axis || B.geom == MateGeom::Axis) {
            order(MateGeom::Axis);
            push3(r, dcross(B.p - A.p, A.d));
        } else push3(r, B.p - A.p);
        return;
    }
    }
}

double maxAbs(const std::vector<double>& r) {
    double m = 0;
    for (double x : r) m = std::max(m, std::fabs(x));
    return m;
}

// Solves A x = b (n x n, row major) by Gaussian elimination with partial pivoting.
bool solveLinear(std::vector<double> A, std::vector<double> b, std::size_t n, std::vector<double>& x) {
    for (std::size_t k = 0; k < n; ++k) {
        std::size_t p = k;
        for (std::size_t i = k + 1; i < n; ++i)
            if (std::fabs(A[i * n + k]) > std::fabs(A[p * n + k])) p = i;
        if (std::fabs(A[p * n + k]) < 1e-300) return false;
        if (p != k) {
            for (std::size_t j = 0; j < n; ++j) std::swap(A[k * n + j], A[p * n + j]);
            std::swap(b[k], b[p]);
        }
        for (std::size_t i = k + 1; i < n; ++i) {
            const double f = A[i * n + k] / A[k * n + k];
            if (f == 0) continue;
            for (std::size_t j = k; j < n; ++j) A[i * n + j] -= f * A[k * n + j];
            b[i] -= f * b[k];
        }
    }
    x.assign(n, 0);
    for (std::size_t k = n; k-- > 0;) {
        double s = b[k];
        for (std::size_t j = k + 1; j < n; ++j) s -= A[k * n + j] * x[j];
        x[k] = s / A[k * n + k];
    }
    return true;
}

// The rank of an m x n matrix (row major).
int rankOf(std::vector<double> J, std::size_t m, std::size_t n) {
    double scale = 0;
    for (double v : J) scale = std::max(scale, std::fabs(v));
    if (scale == 0) return 0;
    const double tol = 1e-7 * scale;
    int rank = 0;
    std::size_t row = 0;
    for (std::size_t col = 0; col < n && row < m; ++col) {
        std::size_t p = row;
        for (std::size_t i = row + 1; i < m; ++i)
            if (std::fabs(J[i * n + col]) > std::fabs(J[p * n + col])) p = i;
        if (std::fabs(J[p * n + col]) <= tol) continue;
        for (std::size_t j = 0; j < n; ++j) std::swap(J[row * n + j], J[p * n + j]);
        for (std::size_t i = row + 1; i < m; ++i) {
            const double f = J[i * n + col] / J[row * n + col];
            for (std::size_t j = col; j < n; ++j) J[i * n + j] -= f * J[row * n + j];
        }
        ++row, ++rank;
    }
    return rank;
}

// Eigenvalues (descending) and unit eigenvectors of a symmetric 3 x 3 matrix (Jacobi).
void eigenSym(double a[3][3], double val[3], DVec3 vec[3]) {
    double v[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    for (int sweep = 0; sweep < 50; ++sweep) {
        const double off = a[0][1] * a[0][1] + a[0][2] * a[0][2] + a[1][2] * a[1][2];
        if (off < 1e-30) break;
        for (int p = 0; p < 2; ++p)
            for (int q = p + 1; q < 3; ++q) {
                if (std::fabs(a[p][q]) < 1e-300) continue;
                const double th = 0.5 * std::atan2(2 * a[p][q], a[q][q] - a[p][p]);
                const double c = std::cos(th), s = std::sin(th);
                for (int k = 0; k < 3; ++k) { // columns p, q
                    const double akp = a[k][p], akq = a[k][q];
                    a[k][p] = c * akp - s * akq;
                    a[k][q] = s * akp + c * akq;
                }
                for (int k = 0; k < 3; ++k) { // rows p, q
                    const double apk = a[p][k], aqk = a[q][k];
                    a[p][k] = c * apk - s * aqk;
                    a[q][k] = s * apk + c * aqk;
                }
                for (int k = 0; k < 3; ++k) {
                    const double vkp = v[k][p], vkq = v[k][q];
                    v[k][p] = c * vkp - s * vkq;
                    v[k][q] = s * vkp + c * vkq;
                }
            }
    }
    int idx[3] = {0, 1, 2};
    std::sort(idx, idx + 3, [&](int i, int j) { return a[i][i] > a[j][j]; });
    for (int k = 0; k < 3; ++k) {
        val[k] = a[idx[k]][idx[k]];
        vec[k] = dnormalize(DVec3{v[0][idx[k]], v[1][idx[k]], v[2][idx[k]]});
    }
}

// The smooth patch of a mesh around triangle `seed` (neighbours turning less than 20
// degrees): a plane, or the axis of a round face when its normals all lie square to one
// direction and its points lie on a circle about it. False: neither.
bool fitMeshPatch(const MeshData& m, int seed, const DVec3& pick, MateGeom& geom, DVec3& p, DVec3& d) {
    const std::size_t nt = m.indices.size() / 3;
    // Weld vertices by position so the patch can cross split normals.
    std::map<std::array<long long, 3>, std::uint32_t> weld;
    std::vector<std::uint32_t> id(m.vertices.size());
    for (std::size_t i = 0; i < m.vertices.size(); ++i) {
        const Vec3& q = m.vertices[i].position;
        const std::array<long long, 3> k{std::llround(q.x * 1e4), std::llround(q.y * 1e4), std::llround(q.z * 1e4)};
        id[i] = weld.try_emplace(k, static_cast<std::uint32_t>(weld.size())).first->second;
    }
    std::map<std::pair<std::uint32_t, std::uint32_t>, std::vector<std::size_t>> edges;
    std::vector<DVec3> nrm(nt);
    std::vector<double> area(nt);
    for (std::size_t t = 0; t < nt; ++t) {
        const DVec3 a(m.vertices[m.indices[3 * t]].position), b(m.vertices[m.indices[3 * t + 1]].position), c(m.vertices[m.indices[3 * t + 2]].position);
        const DVec3 cr = dcross(b - a, c - a);
        area[t] = 0.5 * dlength(cr);
        nrm[t] = dnormalize(cr);
        for (int e = 0; e < 3; ++e) {
            std::uint32_t u = id[m.indices[3 * t + static_cast<std::size_t>(e)]], w = id[m.indices[3 * t + static_cast<std::size_t>((e + 1) % 3)]];
            if (u > w) std::swap(u, w);
            edges[{u, w}].push_back(t);
        }
    }
    const double cosLimit = std::cos(20.0 * kPiA / 180);
    std::vector<char> in(nt, 0);
    std::vector<std::size_t> region{static_cast<std::size_t>(seed)}, stack{static_cast<std::size_t>(seed)};
    in[static_cast<std::size_t>(seed)] = 1;
    while (!stack.empty() && region.size() < 20000) {
        const std::size_t t = stack.back();
        stack.pop_back();
        for (int e = 0; e < 3; ++e) {
            std::uint32_t u = id[m.indices[3 * t + static_cast<std::size_t>(e)]], w = id[m.indices[3 * t + static_cast<std::size_t>((e + 1) % 3)]];
            if (u > w) std::swap(u, w);
            for (std::size_t o : edges[{u, w}])
                if (!in[o] && area[o] > 0 && ddot(nrm[o], nrm[t]) > cosLimit) {
                    in[o] = 1;
                    region.push_back(o);
                    stack.push_back(o);
                }
        }
    }
    double M[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, total = 0;
    DVec3 mean{0, 0, 0};
    for (std::size_t t : region) {
        const double w = area[t];
        const DVec3& n = nrm[t];
        const double c[3] = {n.x, n.y, n.z};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) M[i][j] += w * c[i] * c[j];
        total += w;
        mean += n * w;
    }
    if (total <= 0) return false;
    for (auto& row : M)
        for (double& x : row) x /= total;
    double val[3];
    DVec3 vec[3];
    eigenSym(M, val, vec);
    const std::size_t s0 = static_cast<std::size_t>(seed) * 3;
    const DVec3 a0(m.vertices[m.indices[s0]].position);
    if (val[1] < 1e-4) { // every normal the same way: a plane
        geom = MateGeom::Plane;
        d = ddot(vec[0], mean) >= 0 ? vec[0] : -vec[0];
        p = pick - d * ddot(pick - a0, d);
        return true;
    }
    if (val[2] > 1e-3 * val[0]) return false; // curved both ways: not a cylinder
    // A round face about vec[2]: fit a circle to its points in the plane square to it.
    const DVec3 ax = vec[2], u = dnormalize(dcross(ax, std::fabs(ax.x) < 0.9 ? DVec3{1, 0, 0} : DVec3{0, 1, 0})), v = dcross(ax, u);
    double S[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, rhs[3] = {0, 0, 0};
    std::vector<std::pair<double, double>> pts;
    for (std::size_t t : region)
        for (int k = 0; k < 3; ++k) {
            const DVec3 q(m.vertices[m.indices[3 * t + static_cast<std::size_t>(k)]].position);
            pts.push_back({ddot(q, u), ddot(q, v)});
        }
    for (const auto& [x, y] : pts) { // x^2 + y^2 + D x + E y + F = 0, least squares
        const double row[3] = {x, y, 1}, b = -(x * x + y * y);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) S[i][j] += row[i] * row[j];
            rhs[i] += row[i] * b;
        }
    }
    std::vector<double> A{S[0][0], S[0][1], S[0][2], S[1][0], S[1][1], S[1][2], S[2][0], S[2][1], S[2][2]}, bb{rhs[0], rhs[1], rhs[2]}, sol;
    if (!solveLinear(A, bb, 3, sol)) return false;
    const double cx = -sol[0] / 2, cy = -sol[1] / 2, r2 = cx * cx + cy * cy - sol[2];
    if (!(r2 > 0)) return false;
    const double r = std::sqrt(r2);
    double worst = 0;
    for (const auto& [x, y] : pts) worst = std::max(worst, std::fabs(std::hypot(x - cx, y - cy) - r));
    if (worst > 0.05 * r + 1e-6) return false;
    geom = MateGeom::Axis;
    d = ax;
    const DVec3 c = u * cx + v * cy;
    p = c + ax * ddot(pick, ax);
    return true;
}

} // namespace

// --- names --------------------------------------------------------------------------------

const char* mateTypeName(MateType t) {
    switch (t) {
    case MateType::Fix: return "fix";
    case MateType::Coincident: return "coincident";
    case MateType::Concentric: return "concentric";
    case MateType::Distance: return "distance";
    case MateType::Angle: return "angle";
    case MateType::Parallel: return "parallel";
    case MateType::Perpendicular: return "perpendicular";
    }
    return "coincident";
}

bool mateTypeFromName(const std::string& name, MateType* out, bool* aligned) {
    const std::string n = lowerOf(name);
    bool al = false;
    MateType t;
    if (n == "fix" || n == "ground" || n == "fixed") t = MateType::Fix;
    else if (n == "coincident" || n == "mate" || n == "contact") t = MateType::Coincident;
    else if (n == "flush" || n == "align" || n == "aligned") t = MateType::Coincident, al = true;
    else if (n == "concentric" || n == "coaxial" || n == "axis") t = MateType::Concentric;
    else if (n == "distance" || n == "offset" || n == "gap") t = MateType::Distance;
    else if (n == "angle") t = MateType::Angle;
    else if (n == "parallel") t = MateType::Parallel;
    else if (n == "perpendicular" || n == "square") t = MateType::Perpendicular;
    else return false;
    if (out) *out = t;
    if (aligned) *aligned = al;
    return true;
}

const char* mateGeomName(MateGeom g) {
    switch (g) {
    case MateGeom::Plane: return "plane";
    case MateGeom::Axis: return "axis";
    case MateGeom::Point: return "point";
    }
    return "plane";
}

std::string AssemblyState::nextName() const {
    int n = 1;
    for (const Mate& m : mates)
        if (m.name.size() > 1 && m.name[0] == 'M') n = std::max(n, std::atoi(m.name.c_str() + 1) + 1);
    return "M" + std::to_string(n);
}

// --- features -------------------------------------------------------------------------------

void mateRefWorld(const Body& body, const MateRef& r, DVec3& p, DVec3& d) {
    const Rot R = bodyRot(body);
    p = R * r.p * bodyScale(body) + DVec3(body.offset);
    d = dnormalize(R * r.d);
}

bool mateRefAt(const Body& body, const DVec3& world, MateRef& out, std::string* error) {
    auto fail = [&](const std::string& e) {
        if (error) *error = e;
        return false;
    };
    if (body.mesh.indices.empty()) return fail(body.name + " has no faces");
    const DVec3 m = world - DVec3(body.offset); // in the mesh's (and the B-rep's) frame
    const Rot Rt = bodyRot(body).transposed();
    const double s = bodyScale(body);
    auto setRef = [&](MateGeom g, const DVec3& pm, const DVec3& dm) {
        out.body = body.name;
        out.geom = g;
        out.p = Rt * pm / s;
        out.d = dnormalize(Rt * dm);
        return true;
    };
    auto trianglePlane = [&](const MeshData& mesh, int t) {
        const std::size_t i = static_cast<std::size_t>(t) * 3;
        const DVec3 a(mesh.vertices[mesh.indices[i]].position), b(mesh.vertices[mesh.indices[i + 1]].position), c(mesh.vertices[mesh.indices[i + 2]].position);
        const DVec3 n = dnormalize(dcross(b - a, c - a));
        return setRef(MateGeom::Plane, m - n * ddot(m - a, n), n);
    };
    if (const std::shared_ptr<const Brep> br = exactBrep(body)) {
        // Which face: the nearest triangle of a tessellation that says which face made each.
        const AABB bb = body.mesh.bounds();
        const double diag = bb.valid() ? static_cast<double>(length(bb.max - bb.min)) : 100.0;
        TessellateOptions o;
        o.chord = std::max(0.02, diag * 2e-3);
        TessellateReport rep;
        const MeshData tm = tessellate(*br, o, &rep);
        const int t = nearestTriangle(tm, m);
        if (t >= 0) {
            std::size_t face = 0, upto = 0;
            for (; face < rep.faceTriangles.size(); ++face) {
                upto += rep.faceTriangles[face];
                if (static_cast<std::size_t>(t) < upto) break;
            }
            const BFace* f = nullptr;
            std::size_t k = 0;
            for (const BShell& sh : br->shells)
                for (const BFace& fc : sh.faces) {
                    if (k++ == face) f = &fc;
                }
            if (f && f->surface >= 0 && f->surface < static_cast<int>(br->surfaces.size())) {
                const BSurface& su = br->surfaces[static_cast<std::size_t>(f->surface)];
                switch (su.kind) {
                case BSurfaceKind::Plane: {
                    const DVec3 n = su.f.z * (f->sameSense ? 1.0 : -1.0);
                    return setRef(MateGeom::Plane, m - n * ddot(m - su.f.o, n), n);
                }
                case BSurfaceKind::Cylinder:
                case BSurfaceKind::Cone:
                case BSurfaceKind::Torus:
                case BSurfaceKind::Revolution:
                    return setRef(MateGeom::Axis, su.f.o + su.f.z * ddot(m - su.f.o, su.f.z), su.f.z);
                case BSurfaceKind::Sphere: return setRef(MateGeom::Point, su.f.o, su.f.z);
                default: break;
                }
            }
            return trianglePlane(tm, t);
        }
    }
    const int t = nearestTriangle(body.mesh, m);
    if (t < 0) return fail(body.name + " has no faces");
    MateGeom g;
    DVec3 fp, fd;
    if (fitMeshPatch(body.mesh, t, m, g, fp, fd)) return setRef(g, fp, fd);
    return trianglePlane(body.mesh, t);
}

std::string describeMate(const Mate& m) {
    char buf[64] = "";
    if (m.type == MateType::Distance) std::snprintf(buf, sizeof buf, " %.3g mm", m.value);
    if (m.type == MateType::Angle) std::snprintf(buf, sizeof buf, " %.3g deg", m.value);
    std::string s = m.name + ": " + (m.type == MateType::Coincident && m.aligned ? "flush" : mateTypeName(m.type)) + buf + " ";
    if (m.type == MateType::Fix) return s + m.a.body + (m.suppressed ? " (suppressed)" : "");
    s += m.a.body + " " + mateGeomName(m.a.geom) + " to " + m.b.body + " " + mateGeomName(m.b.geom);
    if (m.type == MateType::Distance && m.aligned) s += " (same way)";
    if (m.suppressed) s += " (suppressed)";
    return s;
}

double mateError(const Scene& scene, const Mate& m) {
    if (m.type == MateType::Fix) return findBodyIndex(scene, m.a.body) < 0 ? -1 : 0;
    const int ia = findBodyIndex(scene, m.a.body), ib = findBodyIndex(scene, m.b.body);
    if (ia < 0 || ib < 0) return -1;
    WorldRef A{m.a.geom, {}, {}}, B{m.b.geom, {}, {}};
    mateRefWorld(scene.bodies()[static_cast<std::size_t>(ia)], m.a, A.p, A.d);
    mateRefWorld(scene.bodies()[static_cast<std::size_t>(ib)], m.b, B.p, B.d);
    std::vector<double> r;
    mateResiduals(m, A, B, r);
    return maxAbs(r);
}

// --- solving --------------------------------------------------------------------------------

MateSolveReport solveMates(Scene& scene, const AssemblyState& st) {
    MateSolveReport rep;
    auto& bodies = scene.bodies();
    // Rigid units: a library component's bodies together, attached bodies with their parent.
    std::map<std::string, int> unitOf;
    std::vector<Unit> units;
    std::vector<int> bodyUnit(bodies.size(), -1);
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        const Body* root = &bodies[i];
        for (int guard = 0; guard < 16 && !root->attachedTo.empty(); ++guard) {
            const int p = findBodyIndex(scene, root->attachedTo);
            if (p < 0) break;
            root = &bodies[static_cast<std::size_t>(p)];
        }
        const std::string key = root->component.empty() ? "b:" + root->name : "c:" + root->component;
        auto [it, fresh] = unitOf.try_emplace(key, static_cast<int>(units.size()));
        if (fresh) units.push_back(Unit{});
        units[static_cast<std::size_t>(it->second)].bodies.push_back(i);
        bodyUnit[i] = it->second;
    }
    // The mates to meet, and the order parts are first mated in.
    std::vector<const Mate*> active;
    std::vector<int> firstSeen(units.size(), -1);
    int seen = 0;
    auto see = [&](int u) {
        if (u >= 0 && firstSeen[static_cast<std::size_t>(u)] < 0) firstSeen[static_cast<std::size_t>(u)] = seen++;
    };
    for (const Mate& m : st.mates) {
        if (m.suppressed) continue;
        const int ia = findBodyIndex(scene, m.a.body), ib = m.type == MateType::Fix ? ia : findBodyIndex(scene, m.b.body);
        if (ia < 0 || ib < 0) {
            rep.missing.push_back(m.name);
            continue;
        }
        if (m.type == MateType::Fix) {
            units[static_cast<std::size_t>(bodyUnit[static_cast<std::size_t>(ia)])].fixed = true;
            continue;
        }
        if (bodyUnit[static_cast<std::size_t>(ia)] == bodyUnit[static_cast<std::size_t>(ib)]) continue; // within one rigid part: nothing to move
        see(bodyUnit[static_cast<std::size_t>(ia)]);
        see(bodyUnit[static_cast<std::size_t>(ib)]);
        active.push_back(&m);
    }
    if (active.empty()) return rep;
    bool anyFixed = false;
    for (std::size_t u = 0; u < units.size(); ++u)
        if (firstSeen[u] >= 0 && units[u].fixed) anyFixed = true;
    if (!anyFixed) // the first part mated holds still
        for (std::size_t u = 0; u < units.size(); ++u)
            if (firstSeen[u] == 0) units[u].fixed = true;
    // The variables: a turn and a shift for each free part that is mated.
    std::vector<int> varUnit; // unit of each 6-block
    for (std::size_t u = 0; u < units.size(); ++u) {
        Unit& un = units[u];
        un.order = std::max(0, firstSeen[u]);
        const Body& lead = bodies[un.bodies.front()];
        un.c = DVec3(lead.offset);
        if (firstSeen[u] >= 0 && un.fixed)
            for (std::size_t bi : un.bodies) rep.fixed.push_back(bodies[bi].name);
        if (firstSeen[u] >= 0 && !un.fixed) varUnit.push_back(static_cast<int>(u));
    }
    // Each mate's features where they start, in world coordinates.
    struct Start {
        WorldRef a, b;
        int ua, ub;
    };
    std::vector<Start> starts;
    for (const Mate* m : active) {
        const std::size_t ia = static_cast<std::size_t>(findBodyIndex(scene, m->a.body)), ib = static_cast<std::size_t>(findBodyIndex(scene, m->b.body));
        Start s{{m->a.geom, {}, {}}, {m->b.geom, {}, {}}, bodyUnit[ia], bodyUnit[ib]};
        mateRefWorld(bodies[ia], m->a, s.a.p, s.a.d);
        mateRefWorld(bodies[ib], m->b, s.b.p, s.b.d);
        starts.push_back(s);
    }
    auto residuals = [&](const std::vector<Unit>& us, std::vector<double>* perMate) {
        std::vector<double> r;
        for (std::size_t k = 0; k < active.size(); ++k) {
            const Start& s = starts[k];
            const Unit &A = us[static_cast<std::size_t>(s.ua)], &B = us[static_cast<std::size_t>(s.ub)];
            const WorldRef a{s.a.geom, A.apply(s.a.p), A.q * s.a.d}, b{s.b.geom, B.apply(s.b.p), B.q * s.b.d};
            const std::size_t before = r.size();
            mateResiduals(*active[k], a, b, r);
            if (perMate) {
                double e = 0;
                for (std::size_t i = before; i < r.size(); ++i) e = std::max(e, std::fabs(r[i]));
                perMate->push_back(e);
            }
        }
        return r;
    };
    auto step = [&](std::vector<Unit> us, const std::vector<double>& x) {
        for (std::size_t k = 0; k < varUnit.size(); ++k) {
            Unit& u = us[static_cast<std::size_t>(varUnit[k])];
            const DVec3 w{x[6 * k], x[6 * k + 1], x[6 * k + 2]}, dv{x[6 * k + 3], x[6 * k + 4], x[6 * k + 5]};
            u.q = expRot(w) * u.q;
            u.v += dv;
        }
        return us;
    };
    const std::size_t n = 6 * varUnit.size();
    // Later parts move more readily than earlier ones; turns count by the lever arm.
    std::vector<double> S(n, 1.0);
    for (std::size_t k = 0; k < varUnit.size(); ++k) {
        const double w = std::pow(4.0, std::min(12, units[static_cast<std::size_t>(varUnit[k])].order));
        for (int j = 0; j < 3; ++j) S[6 * k + static_cast<std::size_t>(j)] = w / kLever, S[6 * k + 3 + static_cast<std::size_t>(j)] = w;
    }
    auto jacobian = [&](const std::vector<Unit>& us, const std::vector<double>& r0) {
        const std::size_t m = r0.size();
        std::vector<double> J(m * n, 0.0);
        const double h = 1e-7;
        for (std::size_t j = 0; j < n; ++j) {
            std::vector<double> x(n, 0.0);
            x[j] = h;
            const std::vector<double> r1 = residuals(step(us, x), nullptr);
            x[j] = -h;
            const std::vector<double> r2 = residuals(step(us, x), nullptr);
            for (std::size_t i = 0; i < m; ++i) J[i * n + j] = (r1[i] - r2[i]) / (2 * h);
        }
        return J;
    };
    std::vector<Unit> cur = units;
    // An angle mate between faces now parallel gives the solver no slope to follow: turn
    // a free part of it to the angle first, about an axis square to the faces.
    for (std::size_t k = 0; k < active.size(); ++k) {
        const Mate& m = *active[k];
        if (m.type != MateType::Angle && m.type != MateType::Perpendicular) continue;
        const double want = m.type == MateType::Angle ? m.value * kPiA / 180 : kPiA / 2;
        const Start& s = starts[k];
        const DVec3 da = cur[static_cast<std::size_t>(s.ua)].q * s.a.d, db = cur[static_cast<std::size_t>(s.ub)].q * s.b.d;
        if (dlength(dcross(da, db)) > 1e-3 || std::fabs(std::sin(want)) < 1e-3) continue;
        const bool bFree = std::find(varUnit.begin(), varUnit.end(), s.ub) != varUnit.end();
        const bool aFree = std::find(varUnit.begin(), varUnit.end(), s.ua) != varUnit.end();
        if (!aFree && !bFree) continue;
        Unit& u = cur[static_cast<std::size_t>(bFree ? s.ub : s.ua)];
        DVec3 axis = dcross(da, DVec3{0, 0, 1});
        if (dlength(axis) < 0.1) axis = dcross(da, DVec3{1, 0, 0});
        u.q = expRot(dnormalize(axis) * want) * u.q;
    }
    std::vector<double> r = residuals(cur, nullptr);
    double err = maxAbs(r);
    double lambda = 1e-6;
    const double tol = 1e-10;
    int it = 0;
    if (n > 0) {
        for (; it < 200 && err > tol; ++it) {
            const std::vector<double> J = jacobian(cur, r);
            const std::size_t m = r.size();
            // Normal equations of the scaled problem: (Js^T Js + lambda) y = -Js^T r, x = S y.
            std::vector<double> A(n * n, 0.0), g(n, 0.0);
            for (std::size_t i = 0; i < m; ++i)
                for (std::size_t a = 0; a < n; ++a) {
                    const double ja = J[i * n + a] * S[a];
                    if (ja == 0) continue;
                    g[a] += ja * r[i];
                    for (std::size_t b = a; b < n; ++b) A[a * n + b] += ja * J[i * n + b] * S[b];
                }
            for (std::size_t a = 0; a < n; ++a)
                for (std::size_t b = 0; b < a; ++b) A[a * n + b] = A[b * n + a];
            double tr = 0;
            for (std::size_t a = 0; a < n; ++a) tr += A[a * n + a];
            const double base = std::max(tr / static_cast<double>(n), 1e-12);
            bool improved = false;
            for (int tries = 0; tries < 12 && !improved; ++tries) {
                std::vector<double> M = A, y;
                for (std::size_t a = 0; a < n; ++a) M[a * n + a] += lambda * base;
                std::vector<double> rhs(n);
                for (std::size_t a = 0; a < n; ++a) rhs[a] = -g[a];
                if (!solveLinear(M, rhs, n, y)) {
                    lambda *= 10;
                    continue;
                }
                for (std::size_t a = 0; a < n; ++a) y[a] *= S[a];
                std::vector<Unit> next = step(cur, y);
                const std::vector<double> rn = residuals(next, nullptr);
                double sn = 0, so = 0;
                for (double v : rn) sn += v * v;
                for (double v : r) so += v * v;
                if (sn < so) {
                    cur = std::move(next);
                    r = rn;
                    err = maxAbs(r);
                    lambda = std::max(lambda / 4, 1e-12);
                    improved = true;
                } else lambda *= 10;
            }
            if (!improved) break; // as close as it gets
        }
        // The freedom left: what the mates don't pin down.
        if (!r.empty()) {
            std::vector<double> J = jacobian(cur, r);
            for (std::size_t i = 0; i < r.size(); ++i)
                for (std::size_t j = 0; j < n; ++j) J[i * n + j] *= (j % 6) < 3 ? 1.0 / kLever : 1.0;
            rep.freedom = static_cast<int>(n) - rankOf(J, r.size(), n);
        } else rep.freedom = static_cast<int>(n);
    }
    rep.iterations = it;
    // Which mates are not met.
    std::vector<double> perMate;
    residuals(cur, &perMate);
    for (std::size_t k = 0; k < perMate.size(); ++k) {
        rep.residual = std::max(rep.residual, perMate[k]);
        if (perMate[k] > 1e-6) rep.failing.push_back(active[k]->name);
    }
    rep.solved = rep.failing.empty();
    // Move the parts.
    for (int u : varUnit) {
        const Unit& un = cur[static_cast<std::size_t>(u)];
        if (un.q.identity(1e-7) && dlength(un.v) < 1e-5) continue; // where it was, but for float rounding
        for (std::size_t bi : un.bodies) {
            Body& b = bodies[bi];
            const Rot R = un.q * bodyRot(b);
            const DVec3 t = un.apply(DVec3(b.offset));
            const DVec3 e = eulerOf(R);
            b.setRotation(e.f());
            b.offset = t.f();
            rep.moved.push_back(b.name);
        }
    }
    if (!rep.moved.empty()) scene.touchBodies();
    // What the float placements leave.
    double after = 0;
    for (const Mate* m : active) after = std::max(after, mateError(scene, *m));
    rep.residual = std::max(rep.residual, after);
    return rep;
}

// --- interference ---------------------------------------------------------------------------

std::vector<InterferenceVolume> findInterference(const Scene& scene, const std::vector<std::size_t>& which, double minVolume) {
    const auto& bodies = scene.bodies();
    std::vector<std::size_t> idx = which;
    if (idx.empty())
        for (std::size_t i = 0; i < bodies.size(); ++i)
            if (bodies[i].visible && !bodies[i].mesh.indices.empty()) idx.push_back(i);
    std::vector<InterferenceVolume> out;
    std::vector<MeshData> world(idx.size());
    std::vector<AABB> boxes(idx.size());
    for (std::size_t k = 0; k < idx.size(); ++k) {
        world[k] = bodies[idx[k]].mesh;
        world[k].transform(translate(bodies[idx[k]].offset));
        boxes[k] = world[k].bounds();
    }
    for (std::size_t i = 0; i < idx.size(); ++i)
        for (std::size_t j = i + 1; j < idx.size(); ++j) {
            const AABB &a = boxes[i], &b = boxes[j];
            if (!a.valid() || !b.valid()) continue;
            const float eps = 1e-4f;
            if (a.min.x >= b.max.x - eps || b.min.x >= a.max.x - eps || a.min.y >= b.max.y - eps || b.min.y >= a.max.y - eps || a.min.z >= b.max.z - eps ||
                b.min.z >= a.max.z - eps)
                continue;
            MeshData x = world[i];
            if (!meshBooleanRobust(x, world[j], BooleanOp::Intersect) || x.indices.empty()) continue;
            // Faces that only touch can leave a sliver: count overlaps of a ten-thousandth of
            // the smaller part or more.
            const double v = meshVolume(x);
            const double small = std::min(std::fabs(meshVolume(world[i])), std::fabs(meshVolume(world[j])));
            if (!(v > std::max(minVolume, 1e-4 * small))) continue;
            const AABB xb = x.bounds();
            out.push_back({bodies[idx[i]].name, bodies[idx[j]].name, v, (xb.min + xb.max) * 0.5f, xb.max - xb.min});
        }
    std::sort(out.begin(), out.end(), [](const InterferenceVolume& p, const InterferenceVolume& q) { return p.volume > q.volume; });
    return out;
}

// --- the tree -------------------------------------------------------------------------------

std::vector<std::string> assemblyTree(const Scene& scene) {
    // Paths in the order their first part appears; parts under them.
    std::vector<std::string> paths;
    std::map<std::string, std::vector<std::string>> parts;
    for (const Body& b : scene.bodies()) {
        std::string p = b.assembly;
        // Every ancestor path too, so the tree has its branches.
        std::vector<std::string> chain;
        for (std::string q = p; !q.empty();) {
            chain.push_back(q);
            const std::size_t slash = q.rfind('/');
            q = slash == std::string::npos ? "" : q.substr(0, slash);
        }
        for (auto it = chain.rbegin(); it != chain.rend(); ++it)
            if (std::find(paths.begin(), paths.end(), *it) == paths.end()) paths.push_back(*it);
        parts[p].push_back(b.name + (b.visible ? "" : " (hidden)"));
    }
    std::vector<std::string> out;
    std::function<void(const std::string&, int)> walk = [&](const std::string& path, int depth) {
        const std::string pad(static_cast<std::size_t>(depth) * 2, ' ');
        for (const std::string& p : parts[path]) out.push_back(pad + p);
        for (const std::string& q : paths) {
            const std::size_t slash = q.rfind('/');
            const std::string parent = slash == std::string::npos ? "" : q.substr(0, slash);
            if (parent != path) continue;
            out.push_back(pad + "[" + (slash == std::string::npos ? q : q.substr(slash + 1)) + "]");
            walk(q, depth + 1);
        }
    };
    walk("", 0);
    return out;
}

std::vector<std::pair<std::string, int>> billOfMaterials(const Scene& scene) {
    std::vector<std::pair<std::string, int>> out;
    for (const Body& b : scene.bodies()) {
        // "Bolt 3", "Bolt (2)" and "Bolt" are the same part.
        std::string n = b.name;
        if (n.size() > 3 && n.back() == ')') {
            const std::size_t open = n.rfind(" (");
            if (open != std::string::npos && open + 3 < n.size() &&
                std::all_of(n.begin() + static_cast<long>(open) + 2, n.end() - 1, [](char c) { return std::isdigit(static_cast<unsigned char>(c)); }))
                n = n.substr(0, open);
        }
        const std::size_t sp = n.find_last_of(' ');
        if (sp != std::string::npos && sp + 1 < n.size() && std::all_of(n.begin() + static_cast<long>(sp) + 1, n.end(), [](char c) { return std::isdigit(static_cast<unsigned char>(c)); }))
            n = n.substr(0, sp);
        auto it = std::find_if(out.begin(), out.end(), [&](const auto& e) { return e.first == n; });
        if (it == out.end()) out.push_back({n, 1});
        else ++it->second;
    }
    return out;
}

// --- saving ---------------------------------------------------------------------------------

namespace {

Json vecJson(const DVec3& v) { return Json(Json::Array{Json(v.x), Json(v.y), Json(v.z)}); }
DVec3 vecOf(const Json& j, const DVec3& dflt) {
    const auto& a = j.items();
    return a.size() == 3 ? DVec3{a[0].asNumber(0), a[1].asNumber(0), a[2].asNumber(0)} : dflt;
}
Json refJson(const MateRef& r) { return Json::object().set("body", r.body).set("geom", mateGeomName(r.geom)).set("p", vecJson(r.p)).set("d", vecJson(r.d)); }
MateRef refOf(const Json& j) {
    MateRef r;
    r.body = j["body"].asString("");
    const std::string g = j["geom"].asString("plane");
    r.geom = g == "axis" ? MateGeom::Axis : g == "point" ? MateGeom::Point : MateGeom::Plane;
    r.p = vecOf(j["p"], {0, 0, 0});
    r.d = vecOf(j["d"], {0, 0, 1});
    return r;
}

} // namespace

Json assemblyToJson(const AssemblyState& st) {
    Json mates = Json::array();
    for (const Mate& m : st.mates) {
        Json o = Json::object();
        o.set("name", m.name).set("type", mateTypeName(m.type)).set("a", refJson(m.a));
        if (m.type != MateType::Fix) o.set("b", refJson(m.b));
        if (m.value != 0) o.set("value", m.value);
        if (m.aligned) o.set("aligned", true);
        if (m.suppressed) o.set("suppressed", true);
        mates.push(o);
    }
    return Json::object().set("mates", mates);
}

bool assemblyFromJson(const Json& j, AssemblyState& st) {
    st = AssemblyState{};
    if (!j.isObject()) return false;
    for (const Json& o : j["mates"].items()) {
        Mate m;
        m.name = o["name"].asString(st.nextName());
        bool al = false;
        if (!mateTypeFromName(o["type"].asString("coincident"), &m.type, &al)) continue;
        m.a = refOf(o["a"]);
        if (m.type != MateType::Fix) m.b = refOf(o["b"]);
        m.value = o["value"].asNumber(0);
        m.aligned = o["aligned"].asBool(al);
        m.suppressed = o["suppressed"].asBool(false);
        st.mates.push_back(m);
    }
    return true;
}

} // namespace sicdm

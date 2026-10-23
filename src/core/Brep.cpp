#include "core/Brep.hpp"

#include <algorithm>
#include <atomic>
#include <array>
#include <charconv>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <stdexcept>
#include <thread>
#include <unordered_map>

namespace sicdm {

namespace {
constexpr double kPi2 = 6.283185307179586476925286766559;
constexpr double kHalfPi = 1.5707963267948966192313216916398;
} // namespace

// --- frames and placements ---------------------------------------------------------

Frame Frame::make(const DVec3& o, const DVec3& z, const DVec3& xRef) {
    Frame f;
    f.o = o;
    f.z = dnormalize(z);
    if (dlength(f.z) == 0) f.z = {0, 0, 1};
    DVec3 x = xRef - f.z * ddot(xRef, f.z);
    if (dlength(x) < 1e-12) { // any direction square to z
        x = std::fabs(f.z.x) < 0.9 ? DVec3{1, 0, 0} : DVec3{0, 1, 0};
        x = x - f.z * ddot(x, f.z);
    }
    f.x = dnormalize(x);
    f.y = dcross(f.z, f.x);
    return f;
}

DVec3 Placement::dir(const DVec3& d) const {
    return {r[0] * d.x + r[1] * d.y + r[2] * d.z, r[3] * d.x + r[4] * d.y + r[5] * d.z, r[6] * d.x + r[7] * d.y + r[8] * d.z};
}

DVec3 Placement::point(const DVec3& p) const { return t + dir(p) * s; }

Placement Placement::then(const Placement& outer) const {
    Placement out;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            double v = 0;
            for (int k = 0; k < 3; ++k) v += outer.r[i * 3 + k] * r[k * 3 + j];
            out.r[i * 3 + j] = v;
        }
    out.s = s * outer.s;
    out.t = outer.point(t);
    return out;
}

Placement Placement::inverse() const {
    Placement p;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) p.r[i * 3 + j] = r[j * 3 + i];
    p.s = 1.0 / s;
    p.t = -p.dir(t) * p.s;
    return p;
}

double decimal(float f) {
    if (!std::isfinite(f)) return static_cast<double>(f);
    char buf[32];
    const auto res = std::to_chars(buf, buf + sizeof(buf) - 1, f); // the shortest that reads back as f
    if (res.ec != std::errc()) return static_cast<double>(f);
    *res.ptr = '\0';
    return std::strtod(buf, nullptr);
}

DVec3 eulerDegrees(const Placement& p) {
    const double* m = p.r; // R = Rz Ry Rx
    const double ry = std::asin(std::clamp(-m[6], -1.0, 1.0));
    double rx, rz;
    if (std::fabs(std::cos(ry)) > 1e-9) {
        rx = std::atan2(m[7], m[8]);
        rz = std::atan2(m[3], m[0]);
    } else { // gimbal lock: all the turn about x
        rx = std::atan2(-m[5], m[4]);
        rz = 0;
    }
    auto deg = [](double a) {
        const double d = std::round(a * 360.0 / kPi2 * 1e7) / 1e7;
        return d == 0 ? 0.0 : d; // no -0
    };
    return {deg(rx), deg(ry), deg(rz)};
}

Placement eulerPlacement(const DVec3& degrees) {
    auto turn = [](double deg, int axis) {
        double c = std::cos(deg * kPi2 / 360.0), s = std::sin(deg * kPi2 / 360.0);
        const double q = deg / 90.0;
        if (q == std::round(q)) { // a quarter turn: exactly 0 and +-1
            const long k = ((static_cast<long>(std::round(q)) % 4) + 4) % 4;
            c = k == 0 ? 1 : k == 2 ? -1 : 0;
            s = k == 1 ? 1 : k == 3 ? -1 : 0;
        }
        Placement p;
        const int a = (axis + 1) % 3, b = (axis + 2) % 3; // rotates a towards b
        p.r[a * 3 + a] = c, p.r[a * 3 + b] = -s;
        p.r[b * 3 + a] = s, p.r[b * 3 + b] = c;
        return p;
    };
    return turn(degrees.x, 0).then(turn(degrees.y, 1)).then(turn(degrees.z, 2));
}

Placement Placement::fromFrames(const Frame& from, const Frame& to) {
    // R maps from's axes onto to's: R = [to.x to.y to.z] [from.x from.y from.z]^T.
    Placement p;
    const DVec3 a[3] = {from.x, from.y, from.z}, b[3] = {to.x, to.y, to.z};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            double v = 0;
            for (int k = 0; k < 3; ++k) {
                const double bi = i == 0 ? b[k].x : i == 1 ? b[k].y : b[k].z;
                const double aj = j == 0 ? a[k].x : j == 1 ? a[k].y : a[k].z;
                v += bi * aj;
            }
            p.r[i * 3 + j] = v;
        }
    p.t = to.o - p.dir(from.o);
    return p;
}

// --- B-splines ---------------------------------------------------------------------

namespace {

// The knot span holding t: knots[k] <= t < knots[k + 1], within the valid range.
int findSpan(const std::vector<double>& knots, int degree, int n, double t) {
    // n: number of control points
    if (t >= knots[static_cast<std::size_t>(n)]) return n - 1;
    if (t <= knots[static_cast<std::size_t>(degree)]) return degree;
    int lo = degree, hi = n;
    while (hi - lo > 1) {
        const int mid = (lo + hi) / 2;
        if (t < knots[static_cast<std::size_t>(mid)]) hi = mid;
        else lo = mid;
    }
    return lo;
}

// The degree + 1 non-zero basis functions at t (Cox-de Boor, "The NURBS Book" A2.2).
void basis(const std::vector<double>& knots, int span, int degree, double t, double* N) {
    double left[32], right[32];
    N[0] = 1.0;
    for (int j = 1; j <= degree; ++j) {
        left[j] = t - knots[static_cast<std::size_t>(span + 1 - j)];
        right[j] = knots[static_cast<std::size_t>(span + j)] - t;
        double saved = 0.0;
        for (int r = 0; r < j; ++r) {
            const double den = right[r + 1] + left[j - r];
            const double tmp = den != 0 ? N[r] / den : 0.0;
            N[r] = saved + right[r + 1] * tmp;
            saved = left[j - r] * tmp;
        }
        N[j] = saved;
    }
}

} // namespace

namespace {
// Wraps t into [a, b) for a closed direction, or clamps it.
double into(double t, double a, double b, bool closed) {
    if (closed && b > a) {
        const double P = b - a;
        t = a + std::fmod(t - a, P);
        if (t < a) t += P;
        return t;
    }
    return std::clamp(t, a, b);
}
bool near3(const DVec3& a, const DVec3& b) { return dlength(a - b) <= 1e-7 * (1 + dlength(a)); }
} // namespace

bool BSplineCurve::closed() const {
    if (m_closed < 0) m_closed = cp.size() > 2 && near3(cp.front(), cp.back()) ? 1 : 0;
    return m_closed == 1;
}

bool BSplineSurface::closedU() const {
    if (m_cu < 0) {
        bool c = nu > 2;
        for (int j = 0; c && j < nv; ++j) c = near3(cp[static_cast<std::size_t>(j)], cp[static_cast<std::size_t>((nu - 1) * nv + j)]);
        m_cu = c ? 1 : 0;
    }
    return m_cu == 1;
}

bool BSplineSurface::closedV() const {
    if (m_cv < 0) {
        bool c = nv > 2;
        for (int i = 0; c && i < nu; ++i) c = near3(cp[static_cast<std::size_t>(i * nv)], cp[static_cast<std::size_t>(i * nv + nv - 1)]);
        m_cv = c ? 1 : 0;
    }
    return m_cv == 1;
}

DVec3 BSplineCurve::eval(double t) const {
    const int n = static_cast<int>(cp.size());
    if (n == 0) return {};
    const int p = std::clamp(degree, 1, 30);
    t = into(t, t0(), t1(), closed());
    const int span = findSpan(knots, p, n, t);
    double N[32];
    basis(knots, span, p, t, N);
    DVec3 s{0, 0, 0};
    double ws = 0;
    for (int j = 0; j <= p; ++j) {
        const int i = span - p + j;
        const double wi = w.empty() ? 1.0 : w[static_cast<std::size_t>(i)];
        s += cp[static_cast<std::size_t>(i)] * (N[j] * wi);
        ws += N[j] * wi;
    }
    return ws != 0 ? s / ws : s;
}

DVec3 BSplineSurface::eval(double u, double v) const {
    if (nu <= 0 || nv <= 0) return {};
    const int pu = std::clamp(du, 1, 30), pv = std::clamp(dv, 1, 30);
    u = into(u, u0(), u1(), closedU());
    v = into(v, v0(), v1(), closedV());
    const int su = findSpan(ku, pu, nu, u), sv = findSpan(kv, pv, nv, v);
    double Nu[32], Nv[32];
    basis(ku, su, pu, u, Nu);
    basis(kv, sv, pv, v, Nv);
    DVec3 s{0, 0, 0};
    double ws = 0;
    for (int a = 0; a <= pu; ++a)
        for (int b = 0; b <= pv; ++b) {
            const std::size_t k = static_cast<std::size_t>(su - pu + a) * static_cast<std::size_t>(nv) + static_cast<std::size_t>(sv - pv + b);
            const double wk = w.empty() ? 1.0 : w[k];
            const double f = Nu[a] * Nv[b] * wk;
            s += cp[k] * f;
            ws += f;
        }
    return ws != 0 ? s / ws : s;
}

// --- curves ------------------------------------------------------------------------

DVec3 BCurve::eval(double t) const {
    switch (kind) {
    case BCurveKind::Line: return f.o + f.x * t;
    case BCurveKind::Circle: return f.o + (f.x * std::cos(t) + f.y * std::sin(t)) * r1;
    case BCurveKind::Ellipse: return f.o + f.x * (r1 * std::cos(t)) + f.y * (r2 * std::sin(t));
    case BCurveKind::BSpline: return bs.eval(t);
    case BCurveKind::Polyline: {
        if (pts.empty()) return {};
        const double c = std::clamp(t, 0.0, static_cast<double>(pts.size() - 1));
        const std::size_t i = std::min(static_cast<std::size_t>(c), pts.size() - 2 < pts.size() ? pts.size() - 2 : 0);
        if (pts.size() == 1) return pts[0];
        const double a = c - static_cast<double>(i);
        return pts[i] + (pts[i + 1] - pts[i]) * a;
    }
    }
    return {};
}

double BCurve::param(const DVec3& p) const {
    const DVec3 d = p - f.o;
    switch (kind) {
    case BCurveKind::Line: return ddot(d, f.x);
    case BCurveKind::Circle: return std::atan2(ddot(d, f.y), ddot(d, f.x));
    case BCurveKind::Ellipse: {
        // The angle of the scaled point, then Newton on the squared distance.
        double t = std::atan2(ddot(d, f.y) / std::max(r2, 1e-12), ddot(d, f.x) / std::max(r1, 1e-12));
        const double px = ddot(d, f.x), py = ddot(d, f.y);
        for (int i = 0; i < 20; ++i) {
            const double c = std::cos(t), s = std::sin(t);
            const double ex = r1 * c - px, ey = r2 * s - py;
            const double g = -ex * r1 * s + ey * r2 * c;                                   // half the derivative
            const double h = r1 * r1 * s * s + r2 * r2 * c * c - ex * r1 * c - ey * r2 * s; // half the second
            if (std::fabs(h) < 1e-15) break;
            const double step = g / h;
            t -= step;
            if (std::fabs(step) < 1e-14) break;
        }
        return t;
    }
    case BCurveKind::Polyline: {
        double best = 1e300, bt = 0;
        for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
            const DVec3 ab = pts[i + 1] - pts[i];
            const double l2 = ddot(ab, ab);
            const double a = l2 > 0 ? std::clamp(ddot(p - pts[i], ab) / l2, 0.0, 1.0) : 0.0;
            const DVec3 q = pts[i] + ab * a;
            const double dd = ddot(q - p, q - p);
            if (dd < best) best = dd, bt = static_cast<double>(i) + a;
        }
        return bt;
    }
    case BCurveKind::BSpline: {
        const double a = bs.t0(), b = bs.t1();
        const int n = std::max(64, static_cast<int>(bs.cp.size()) * 8);
        double best = 1e300, bt = a;
        for (int i = 0; i <= n; ++i) {
            const double t = a + (b - a) * i / n;
            const DVec3 q = bs.eval(t) - p;
            const double dd = ddot(q, q);
            if (dd < best) best = dd, bt = t;
        }
        // Golden section in the neighbouring steps.
        double lo = std::max(a, bt - (b - a) / n), hi = std::min(b, bt + (b - a) / n);
        constexpr double g = 0.6180339887498949;
        double x1 = hi - g * (hi - lo), x2 = lo + g * (hi - lo);
        auto f2 = [&](double t) {
            const DVec3 q = bs.eval(t) - p;
            return ddot(q, q);
        };
        double f1 = f2(x1), fx2 = f2(x2);
        for (int i = 0; i < 60; ++i) {
            if (f1 < fx2) hi = x2, x2 = x1, fx2 = f1, x1 = hi - g * (hi - lo), f1 = f2(x1);
            else lo = x1, x1 = x2, f1 = fx2, x2 = lo + g * (hi - lo), fx2 = f2(x2);
        }
        return 0.5 * (lo + hi);
    }
    }
    return 0;
}

// --- surfaces ----------------------------------------------------------------------

namespace {

// Rotation of v about the unit axis k by angle a (Rodrigues).
DVec3 rotateAbout(const DVec3& v, const DVec3& k, double a) {
    const double c = std::cos(a), s = std::sin(a);
    return v * c + dcross(k, v) * s + k * (ddot(k, v) * (1 - c));
}

const BCurve* sweptCurve(const BSurface& s, const std::vector<BCurve>* curves) {
    if (!curves || s.curve < 0 || s.curve >= static_cast<int>(curves->size())) return nullptr;
    return &(*curves)[static_cast<std::size_t>(s.curve)];
}

// The surface's swept curves live in the B-rep; evaluation reaches them through this.
thread_local const std::vector<BCurve>* tCurves = nullptr;

} // namespace

DVec3 BSurface::eval(double u, double v) const {
    switch (kind) {
    case BSurfaceKind::Plane: return f.o + f.x * u + f.y * v;
    case BSurfaceKind::Cylinder: return f.o + (f.x * std::cos(u) + f.y * std::sin(u)) * r1 + f.z * v;
    case BSurfaceKind::Cone: return f.o + (f.x * std::cos(u) + f.y * std::sin(u)) * (r1 + v * std::tan(r2)) + f.z * v;
    case BSurfaceKind::Sphere: return f.o + (f.x * std::cos(u) + f.y * std::sin(u)) * (r1 * std::cos(v)) + f.z * (r1 * std::sin(v));
    case BSurfaceKind::Torus: return f.o + (f.x * std::cos(u) + f.y * std::sin(u)) * (r1 + r2 * std::cos(v)) + f.z * (r2 * std::sin(v));
    case BSurfaceKind::BSpline: return bs.eval(u, v);
    case BSurfaceKind::Extrusion: {
        const BCurve* c = sweptCurve(*this, tCurves);
        return (c ? c->eval(u) : DVec3{}) + dir * v;
    }
    case BSurfaceKind::Revolution: { // ISO 10303-42: the curve at parameter v, turned by u
        const BCurve* c = sweptCurve(*this, tCurves);
        const DVec3 p = c ? c->eval(v) : DVec3{};
        return f.o + rotateAbout(p - f.o, f.z, u);
    }
    }
    return {};
}

DVec3 BSurface::normal(double u, double v) const {
    // Central differences, scaled to the parameter's size.
    double u0, u1, v0, v1;
    domain(u0, u1, v0, v1);
    const double hu = std::max(1e-7, 1e-6 * std::max(1.0, std::fabs(u1 - u0) < 1e9 ? std::fabs(u1 - u0) : 1.0));
    const double hv = std::max(1e-7, 1e-6 * std::max(1.0, std::fabs(v1 - v0) < 1e9 ? std::fabs(v1 - v0) : 1.0));
    DVec3 n = dcross(eval(u + hu, v) - eval(u - hu, v), eval(u, v + hv) - eval(u, v - hv));
    if (dlength(n) < 1e-18) { // a pole or apex: step inwards a little
        const double uu = u, vv = v + (v > 0 ? -1e-4 : 1e-4);
        n = dcross(eval(uu + hu, vv) - eval(uu - hu, vv), eval(uu, vv + hv) - eval(uu, vv - hv));
    }
    return dnormalize(n);
}

double BSurface::periodU() const {
    switch (kind) {
    case BSurfaceKind::Cylinder:
    case BSurfaceKind::Cone:
    case BSurfaceKind::Sphere:
    case BSurfaceKind::Torus:
    case BSurfaceKind::Revolution: return kPi2;
    case BSurfaceKind::Extrusion: {
        const BCurve* c = sweptCurve(*this, tCurves);
        if (c && c->periodic()) return kPi2;
        if (c && c->kind == BCurveKind::BSpline && c->bs.closed()) return c->bs.t1() - c->bs.t0();
        return 0;
    }
    case BSurfaceKind::BSpline: return bs.closedU() ? bs.u1() - bs.u0() : 0;
    default: return 0;
    }
}

double BSurface::periodV() const {
    switch (kind) {
    case BSurfaceKind::Torus: return kPi2;
    case BSurfaceKind::Revolution: {
        const BCurve* c = sweptCurve(*this, tCurves);
        if (c && c->periodic()) return kPi2;
        if (c && c->kind == BCurveKind::BSpline && c->bs.closed()) return c->bs.t1() - c->bs.t0();
        return 0;
    }
    case BSurfaceKind::BSpline: return bs.closedV() ? bs.v1() - bs.v0() : 0;
    default: return 0;
    }
}

void BSurface::domain(double& u0, double& u1, double& v0, double& v1) const {
    constexpr double big = 1e12;
    u0 = -big, u1 = big, v0 = -big, v1 = big;
    switch (kind) {
    case BSurfaceKind::Cylinder:
    case BSurfaceKind::Cone: u0 = 0, u1 = kPi2; break;
    case BSurfaceKind::Sphere: u0 = 0, u1 = kPi2, v0 = -kHalfPi, v1 = kHalfPi; break;
    case BSurfaceKind::Torus: u0 = 0, u1 = kPi2, v0 = 0, v1 = kPi2; break;
    case BSurfaceKind::BSpline: u0 = bs.u0(), u1 = bs.u1(), v0 = bs.v0(), v1 = bs.v1(); break;
    case BSurfaceKind::Revolution:
    case BSurfaceKind::Extrusion: {
        const BCurve* c = sweptCurve(*this, tCurves);
        double a = -big, b = big;
        if (c) {
            if (c->periodic()) a = 0, b = kPi2;
            else if (c->kind == BCurveKind::BSpline) a = c->bs.t0(), b = c->bs.t1();
            else if (c->kind == BCurveKind::Polyline) a = 0, b = static_cast<double>(std::max<std::size_t>(c->pts.size(), 1) - 1);
        }
        if (kind == BSurfaceKind::Revolution) u0 = 0, u1 = kPi2, v0 = a, v1 = b;
        else u0 = a, u1 = b;
        break;
    }
    default: break;
    }
}

// --- the B-rep ----------------------------------------------------------------------

std::size_t Brep::faceCount() const {
    std::size_t n = 0;
    for (const BShell& s : shells) n += s.faces.size();
    return n;
}

void Brep::transform(const Placement& p) {
    auto frame = [&](Frame& f) {
        f.o = p.point(f.o);
        f.x = dnormalize(p.dir(f.x));
        f.y = dnormalize(p.dir(f.y));
        f.z = dnormalize(p.dir(f.z));
    };
    for (BrepVertex& v : vertices) v.p = p.point(v.p);
    for (BCurve& c : curves) {
        frame(c.f);
        if (c.kind == BCurveKind::Line) c.f.x = dnormalize(c.f.x); // lines: unit direction, t in length units
        c.r1 *= p.s;
        c.r2 *= p.s;
        for (DVec3& q : c.bs.cp) q = p.point(q);
        for (DVec3& q : c.pts) q = p.point(q);
    }
    for (BSurface& s : surfaces) {
        frame(s.f);
        if (s.kind != BSurfaceKind::Cone) s.r2 *= p.s; // a cone's r2 is its angle
        s.r1 *= p.s;
        if (s.kind == BSurfaceKind::Plane) {
            // u, v are lengths along x, y: unchanged by a rigid move, scaled with s.
        }
        for (DVec3& q : s.bs.cp) q = p.point(q);
        s.dir = p.dir(s.dir) * p.s;
    }
    for (BShell& sh : shells)
        for (BFace& f : sh.faces)
            for (BLoop& l : f.loops)
                for (DVec3& q : l.poly) q = p.point(q);
}

// --- tessellation ------------------------------------------------------------------

namespace {

struct P2 {
    double x, y;
};

// The orientation of c relative to ab (twice the signed area).
double orient(const P2& a, const P2& b, const P2& c) { return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x); }

// Is d inside the circumcircle of the counter-clockwise triangle abc?
bool inCircle(const P2& a, const P2& b, const P2& c, const P2& d) {
    const double adx = a.x - d.x, ady = a.y - d.y, bdx = b.x - d.x, bdy = b.y - d.y, cdx = c.x - d.x, cdy = c.y - d.y;
    const double ad = adx * adx + ady * ady, bd = bdx * bdx + bdy * bdy, cd = cdx * cdx + cdy * cdy;
    const double det = adx * (bdy * cd - bd * cdy) - ady * (bdx * cd - bd * cdx) + ad * (bdx * cdy - bdy * cdx);
    const double m = ad + bd + cd;
    return det > 1e-12 * m * m; // clearly inside: co-circular points (rows of a grid) never flip back and forth
}

// A constrained Delaunay triangulation in the plane, built incrementally (Lawson
// flips), with segments recovered by flipping the edges that cross them.
class Cdt {
public:
    struct Tri {
        int v[3];
        int n[3];     // neighbour across the edge opposite v[k] (-1: none)
        bool c[3];    // that edge is a constraint
        bool dead = false;
    };
    std::vector<P2> pts;
    std::vector<Tri> tris;

    explicit Cdt(double x0, double y0, double x1, double y1) {
        const double d = std::max({x1 - x0, y1 - y0, 1e-9}) * 20;
        const double cx = (x0 + x1) * 0.5, cy = (y0 + y1) * 0.5;
        pts = {{cx - d, cy - d}, {cx + d, cy - d}, {cx, cy + d}};
        tris.push_back({{0, 1, 2}, {-1, -1, -1}, {false, false, false}, false});
        touch(0);
        m_eps = 1e-12 * d * d;
    }

    // Inserts p, returns its index (an existing one if it coincides).
    int insert(const P2& p, int hint = -1) {
        int t = locate(p, hint);
        if (t < 0) return -1;
        for (int k = 0; k < 3; ++k) {
            const P2& q = pts[static_cast<std::size_t>(tris[static_cast<std::size_t>(t)].v[k])];
            if (std::fabs(q.x - p.x) <= m_tiny && std::fabs(q.y - p.y) <= m_tiny) return tris[static_cast<std::size_t>(t)].v[k];
        }
        const int id = static_cast<int>(pts.size());
        pts.push_back(p);
        // On an edge? split the two triangles sharing it.
        const Tri& T = tris[static_cast<std::size_t>(t)];
        for (int k = 0; k < 3; ++k) {
            const P2& a = pts[static_cast<std::size_t>(T.v[(k + 1) % 3])];
            const P2& b = pts[static_cast<std::size_t>(T.v[(k + 2) % 3])];
            const double o = orient(a, b, p);
            const double len2 = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);
            if (std::fabs(o) <= 1e-10 * len2) {
                splitEdge(t, k, id);
                return id;
            }
        }
        splitTri(t, id);
        return id;
    }

    // Makes segment a-b an edge of the triangulation (flipping what crosses it).
    bool force(int a, int b) {
        if (a == b) return true;
        for (int guard = 0; guard < 100000; ++guard) {
            int t, k;
            if (findEdge(a, b, t, k)) {
                markConstraint(t, k);
                return true;
            }
            // A vertex lying exactly on the segment: split it there.
            if (const int mid = vertexOnSegment(a, b); mid >= 0) return force(a, mid) && force(mid, b);
            // Flip the first crossing edge that can be flipped.
            if (!flipOneCrossing(a, b)) return false;
        }
        return false;
    }

    void legalizeAll() {
        std::vector<std::pair<int, int>> stack;
        for (int t = 0; t < static_cast<int>(tris.size()); ++t)
            if (!tris[static_cast<std::size_t>(t)].dead)
                for (int k = 0; k < 3; ++k) stack.push_back({t, k});
        legalize(stack);
    }

    // Splits triangle t at an inside point (refinement).
    int insertIn(int t, const P2& p) {
        const int id = static_cast<int>(pts.size());
        pts.push_back(p);
        splitTri(t, id);
        return id;
    }

    double tiny() const { return m_tiny; }
    void setTiny(double v) { m_tiny = v; }

private:
    double m_eps = 0, m_tiny = 0;
    // A triangle holding each vertex. Every change rewrites triangles that between them
    // hold all the vertices the old ones did, so touching them keeps this right.
    std::vector<int> m_vt;
    void touch(int t) {
        for (int v : T(t).v) {
            if (static_cast<std::size_t>(v) >= m_vt.size()) m_vt.resize(static_cast<std::size_t>(v) + 1, -1);
            m_vt[static_cast<std::size_t>(v)] = t;
        }
    }

    Tri& T(int t) { return tris[static_cast<std::size_t>(t)]; }
    const P2& P(int v) const { return pts[static_cast<std::size_t>(v)]; }

    int locate(const P2& p, int hint) {
        int t = hint >= 0 && hint < static_cast<int>(tris.size()) && !T(hint).dead ? hint : -1;
        if (t < 0)
            for (int i = static_cast<int>(tris.size()) - 1; i >= 0; --i)
                if (!T(i).dead) {
                    t = i;
                    break;
                }
        // Walk towards p.
        for (int steps = 0; steps < 4 * static_cast<int>(tris.size()) + 10; ++steps) {
            const Tri& tr = T(t);
            int next = -1;
            for (int k = 0; k < 3; ++k) {
                const P2& a = P(tr.v[(k + 1) % 3]);
                const P2& b = P(tr.v[(k + 2) % 3]);
                if (orient(a, b, p) < -1e-14 * (std::fabs(a.x) + std::fabs(a.y) + std::fabs(b.x) + std::fabs(b.y) + 1) && tr.n[k] >= 0) {
                    next = tr.n[k];
                    break;
                }
            }
            if (next < 0) return t;
            t = next;
        }
        // Fallback: a linear search.
        for (int i = 0; i < static_cast<int>(tris.size()); ++i) {
            if (T(i).dead) continue;
            const Tri& tr = T(i);
            if (orient(P(tr.v[0]), P(tr.v[1]), p) >= -1e-12 && orient(P(tr.v[1]), P(tr.v[2]), p) >= -1e-12 && orient(P(tr.v[2]), P(tr.v[0]), p) >= -1e-12)
                return i;
        }
        return -1;
    }

    void setNeighbour(int t, int a, int b, int nt) { // in t, the edge a-b now faces nt
        if (t < 0) return;
        Tri& tr = T(t);
        for (int k = 0; k < 3; ++k) {
            const int x = tr.v[(k + 1) % 3], y = tr.v[(k + 2) % 3];
            if ((x == a && y == b) || (x == b && y == a)) {
                tr.n[k] = nt;
                return;
            }
        }
    }

    void splitTri(int t, int id) {
        const Tri old = T(t);
        const int a = old.v[0], b = old.v[1], c = old.v[2];
        const int t0 = t, t1 = static_cast<int>(tris.size()), t2 = t1 + 1;
        // t0: a b id, t1: b c id, t2: c a id
        T(t0) = {{a, b, id}, {t1, t2, old.n[2]}, {false, false, old.c[2]}, false};
        tris.push_back({{b, c, id}, {t2, t0, old.n[0]}, {false, false, old.c[0]}, false});
        tris.push_back({{c, a, id}, {t0, t1, old.n[1]}, {false, false, old.c[1]}, false});
        touch(t0), touch(t1), touch(t2);
        setNeighbour(old.n[0], b, c, t1);
        setNeighbour(old.n[1], c, a, t2);
        setNeighbour(old.n[2], a, b, t0);
        std::vector<std::pair<int, int>> stack{{t0, 2}, {t1, 2}, {t2, 2}};
        legalize(stack);
    }

    // id lies on the edge opposite v[k] of t: t and its neighbour there become four.
    void splitEdge(int t, int k, int id) {
        const Tri A = T(t);
        const int a = A.v[k], b = A.v[(k + 1) % 3], c = A.v[(k + 2) % 3]; // edge b-c, opposite a
        const int u = A.n[k];
        const bool con = A.c[k];
        const int nca = A.n[(k + 1) % 3], nab = A.n[(k + 2) % 3];
        const bool cca = A.c[(k + 1) % 3], cab = A.c[(k + 2) % 3];
        const int t0 = t, t1 = static_cast<int>(tris.size());
        tris.push_back({}); // t1
        int t2 = -1, t3 = -1;
        int d = -1, ndc = -1, nbd = -1;
        bool cdc = false, cbd = false;
        if (u >= 0) {
            const Tri B = T(u);
            int j = 0;
            while (j < 3 && !(B.v[(j + 1) % 3] == c && B.v[(j + 2) % 3] == b)) ++j;
            if (j < 3) {
                d = B.v[j];
                nbd = B.n[(j + 1) % 3], ndc = B.n[(j + 2) % 3];
                cbd = B.c[(j + 1) % 3], cdc = B.c[(j + 2) % 3];
                t2 = u;
                t3 = static_cast<int>(tris.size());
                tris.push_back({});
            }
        }
        // t0 = a b id, t1 = a id c (and t2 = d c id, t3 = d id b on the other side).
        T(t0) = {{a, b, id}, {t3, t1, nab}, {con, false, cab}, false};
        T(t1) = {{a, id, c}, {t2, nca, t0}, {con, cca, false}, false};
        touch(t0), touch(t1);
        setNeighbour(nca, c, a, t1);
        std::vector<std::pair<int, int>> stack{{t0, 2}, {t1, 1}};
        if (t2 >= 0) {
            T(t2) = {{d, c, id}, {t1, t3, ndc}, {con, false, cdc}, false};
            T(t3) = {{d, id, b}, {t0, nbd, t2}, {con, cbd, false}, false};
            touch(t2), touch(t3);
            setNeighbour(nbd, b, d, t3);
            stack.push_back({t2, 2});
            stack.push_back({t3, 1});
        }
        legalize(stack);
    }

    void markConstraint(int t, int k) {
        Tri& tr = T(t);
        tr.c[k] = true;
        const int a = tr.v[(k + 1) % 3], b = tr.v[(k + 2) % 3];
        if (tr.n[k] >= 0) {
            Tri& o = T(tr.n[k]);
            for (int j = 0; j < 3; ++j)
                if ((o.v[(j + 1) % 3] == b && o.v[(j + 2) % 3] == a) || (o.v[(j + 1) % 3] == a && o.v[(j + 2) % 3] == b)) o.c[j] = true;
        }
    }

    bool findEdge(int a, int b, int& t, int& k) {
        // Round the triangles about a, one way and then the other.
        const int start = static_cast<std::size_t>(a) < m_vt.size() ? m_vt[static_cast<std::size_t>(a)] : -1;
        bool lost = start < 0 || T(start).dead;
        for (int dir = 1; dir <= 2 && !lost; ++dir) {
            int cur = start;
            for (int steps = 0; cur >= 0 && steps < 100000; ++steps) {
                const Tri& tr = T(cur);
                int ia = 0;
                while (ia < 3 && tr.v[ia] != a) ++ia;
                if (ia == 3 || tr.dead) { // not round a after all: search everything
                    lost = true;
                    break;
                }
                for (int j = 1; j <= 2; ++j)
                    if (tr.v[(ia + j) % 3] == b) {
                        t = cur, k = (ia + 3 - j) % 3; // the edge a-b is opposite the third vertex
                        return true;
                    }
                cur = tr.n[(ia + dir) % 3];
                if (cur == start) return false; // all the way round
            }
        }
        if (!lost) return false;
        for (int i = 0; i < static_cast<int>(tris.size()); ++i) {
            if (T(i).dead) continue;
            for (int j = 0; j < 3; ++j) {
                const int x = T(i).v[(j + 1) % 3], y = T(i).v[(j + 2) % 3];
                if ((x == a && y == b) || (x == b && y == a)) {
                    t = i, k = j;
                    return true;
                }
            }
        }
        return false;
    }

    int vertexOnSegment(int a, int b) const {
        const P2 &A = P(a), &B = P(b);
        const double len2 = (B.x - A.x) * (B.x - A.x) + (B.y - A.y) * (B.y - A.y);
        for (int v = 3; v < static_cast<int>(pts.size()); ++v) {
            if (v == a || v == b) continue;
            const P2& q = P(v);
            const double t = ((q.x - A.x) * (B.x - A.x) + (q.y - A.y) * (B.y - A.y)) / len2;
            if (t <= 1e-9 || t >= 1 - 1e-9) continue;
            if (std::fabs(orient(A, B, q)) <= 1e-10 * len2) return v;
        }
        return -1;
    }

    static bool properCross(const P2& a, const P2& b, const P2& c, const P2& d) {
        const double o1 = orient(a, b, c), o2 = orient(a, b, d), o3 = orient(c, d, a), o4 = orient(c, d, b);
        return ((o1 > 0 && o2 < 0) || (o1 < 0 && o2 > 0)) && ((o3 > 0 && o4 < 0) || (o3 < 0 && o4 > 0));
    }

    bool flipOneCrossing(int a, int b) {
        const P2 &A = P(a), &B = P(b);
        for (int i = 0; i < static_cast<int>(tris.size()); ++i) {
            if (T(i).dead) continue;
            for (int k = 0; k < 3; ++k) {
                const int x = T(i).v[(k + 1) % 3], y = T(i).v[(k + 2) % 3];
                if (x == a || x == b || y == a || y == b) continue;
                if (T(i).c[k] || T(i).n[k] < 0) continue;
                if (!properCross(A, B, P(x), P(y))) continue;
                if (flip(i, k, false)) return true;
            }
        }
        return false;
    }

    // Flips the edge opposite v[k] of t (when the quad is convex). With `checkDelaunay`,
    // only when that makes it Delaunay.
    bool flip(int t, int k, bool checkDelaunay) {
        Tri& A = T(t);
        const int u = A.n[k];
        if (u < 0 || A.c[k]) return false;
        const int a = A.v[k], b = A.v[(k + 1) % 3], c = A.v[(k + 2) % 3];
        Tri& B = T(u);
        int j = 0;
        while (j < 3 && !(B.v[(j + 1) % 3] == c && B.v[(j + 2) % 3] == b)) ++j;
        if (j == 3) return false;
        const int d = B.v[j];
        if (checkDelaunay && !inCircle(P(a), P(b), P(c), P(d))) return false;
        // The quad a b d c must be convex for the flip.
        if (orient(P(a), P(b), P(d)) <= 0 || orient(P(a), P(d), P(c)) <= 0) return false;
        // In B = (d, c, b): the edge b-d is opposite c, the edge d-c opposite b.
        const int nab = A.n[(k + 2) % 3], nca = A.n[(k + 1) % 3], nbd = B.n[(j + 1) % 3], ndc = B.n[(j + 2) % 3];
        const bool cab = A.c[(k + 2) % 3], cca = A.c[(k + 1) % 3], cbd = B.c[(j + 1) % 3], cdc = B.c[(j + 2) % 3];
        // t: a b d, u: a d c
        T(t) = {{a, b, d}, {nbd, u, nab}, {cbd, false, cab}, false};
        T(u) = {{a, d, c}, {ndc, nca, t}, {cdc, cca, false}, false};
        touch(t), touch(u);
        setNeighbour(nbd, b, d, t);
        setNeighbour(ndc, d, c, u);
        setNeighbour(nca, c, a, u);
        setNeighbour(nab, a, b, t);
        return true;
    }

    // Lawson's legalisation after inserting a point: each entry is a triangle and the
    // index of the new point in it; only the edge opposite that point is checked, and a
    // flip leaves two triangles that both hold the point (at index 0).
    void legalize(std::vector<std::pair<int, int>>& stack) {
        int guard = 0;
        while (!stack.empty() && guard++ < 1000000) {
            const auto [t, k] = stack.back();
            stack.pop_back();
            if (t < 0 || T(t).dead) continue;
            const int u = T(t).n[k];
            if (u < 0 || T(t).c[k]) continue;
            if (flip(t, k, true)) {
                stack.push_back({t, 0});
                stack.push_back({u, 0});
            }
        }
    }
};

// --- edge sampling --------------------------------------------------------------------

struct TessContext {
    const Brep& b;
    const TessellateOptions& o;
    std::vector<std::vector<DVec3>> edgePts; // per edge, from v0 to v1
    std::vector<char> edgeDone;
};

double segmentsFor(double radius, double sweep, const TessellateOptions& o) {
    if (!(radius > 0)) return 1;
    double step = o.angle;
    if (o.chord > 0 && o.chord < radius) step = std::min(step, 2.0 * std::acos(1.0 - o.chord / radius));
    if (o.maxEdge > 0) step = std::min(step, o.maxEdge / radius);
    return std::max(1.0, std::ceil(std::fabs(sweep) / std::max(step, 1e-4)));
}

// Adaptive samples of a curve between parameters a and b (both ends included).
void sampleAdaptive(const BCurve& c, double a, double b, const TessellateOptions& o, std::vector<DVec3>& out, int depth = 0) {
    const DVec3 pa = c.eval(a), pb = c.eval(b);
    const double m = 0.5 * (a + b);
    const DVec3 pm = c.eval(m), q1 = c.eval(a + 0.25 * (b - a)), q3 = c.eval(a + 0.75 * (b - a));
    const DVec3 chordMid = (pa + pb) * 0.5;
    const double dev = std::max({dlength(pm - chordMid), dlength(q1 - (pa * 0.75 + pb * 0.25)), dlength(q3 - (pa * 0.25 + pb * 0.75))});
    const DVec3 t1 = pm - pa, t2 = pb - pm;
    const double l1 = dlength(t1), l2 = dlength(t2);
    const double turn = l1 > 0 && l2 > 0 ? std::acos(std::clamp(ddot(t1, t2) / (l1 * l2), -1.0, 1.0)) : 0.0;
    const bool tooLong = o.maxEdge > 0 && dlength(pb - pa) > o.maxEdge;
    if (depth < 3 || (depth < 18 && (dev > o.chord || turn > o.angle || tooLong))) {
        sampleAdaptive(c, a, m, o, out, depth + 1);
        sampleAdaptive(c, m, b, o, out, depth + 1);
        return;
    }
    if (out.empty()) out.push_back(pa);
    out.push_back(pb);
}

const std::vector<DVec3>& edgeSamples(TessContext& tc, int e) {
    auto& done = tc.edgeDone[static_cast<std::size_t>(e)];
    auto& pts = tc.edgePts[static_cast<std::size_t>(e)];
    if (done) return pts;
    done = 1;
    const BrepEdge& E = tc.b.edges[static_cast<std::size_t>(e)];
    const DVec3 p0 = tc.b.vertices[static_cast<std::size_t>(E.v0)].p, p1 = tc.b.vertices[static_cast<std::size_t>(E.v1)].p;
    const bool closed = E.v0 == E.v1;
    if (E.curve < 0 || E.curve >= static_cast<int>(tc.b.curves.size())) {
        pts = {p0, p1};
        return pts;
    }
    const BCurve& c = tc.b.curves[static_cast<std::size_t>(E.curve)];
    if (c.kind == BCurveKind::Line) {
        const int n = tc.o.maxEdge > 0 ? std::max(1, static_cast<int>(std::ceil(dlength(p1 - p0) / tc.o.maxEdge))) : 1;
        for (int i = 0; i <= n; ++i) pts.push_back(p0 + (p1 - p0) * (static_cast<double>(i) / n));
        return pts;
    }
    if (c.kind == BCurveKind::Polyline) {
        double ta = c.param(p0), tb = c.param(p1);
        if (closed) ta = 0, tb = static_cast<double>(c.pts.size() - 1);
        pts.push_back(p0);
        const int lo = static_cast<int>(std::ceil(std::min(ta, tb))), hi = static_cast<int>(std::floor(std::max(ta, tb)));
        std::vector<DVec3> mid;
        for (int i = lo; i <= hi; ++i) mid.push_back(c.pts[static_cast<std::size_t>(i)]);
        if (ta > tb) std::reverse(mid.begin(), mid.end());
        for (const DVec3& q : mid)
            if (dlength(q - pts.back()) > 1e-12 && dlength(q - p1) > 1e-12) pts.push_back(q);
        pts.push_back(p1);
        return pts;
    }
    double ta = c.param(p0), tb = c.param(p1);
    if (c.periodic()) {
        if (E.sameSense) {
            while (tb <= ta + 1e-12) tb += kPi2;
            if (closed) tb = ta + kPi2;
        } else {
            while (tb >= ta - 1e-12) tb -= kPi2;
            if (closed) tb = ta - kPi2;
        }
    } else if (c.kind == BCurveKind::BSpline && closed) {
        ta = E.sameSense ? c.bs.t0() : c.bs.t1();
        tb = E.sameSense ? c.bs.t1() : c.bs.t0();
    }
    if (c.kind == BCurveKind::Circle || c.kind == BCurveKind::Ellipse) {
        const int n = static_cast<int>(segmentsFor(std::max(c.r1, c.r2), tb - ta, tc.o));
        for (int i = 0; i <= n; ++i) pts.push_back(c.eval(ta + (tb - ta) * i / n));
    } else {
        sampleAdaptive(c, ta, tb, tc.o, pts);
    }
    // The ends are the vertices exactly (neighbouring edges meet there).
    pts.front() = p0;
    pts.back() = p1;
    return pts;
}

// --- one face ---------------------------------------------------------------------------

struct FaceMesh {
    std::vector<DVec3> p;
    std::vector<DVec3> n;
    std::vector<std::array<int, 3>> t;
};

// Shifts x by whole periods to be nearest to `near`.
double wrapNear(double x, double near, double period) {
    if (period <= 0) return x;
    return x + period * std::round((near - x) / period);
}

// Projects 3D points onto a surface's parameters, near a hint when given.
class Inverse {
public:
    explicit Inverse(const BSurface& s) : m_s(s) { m_s.domain(m_u0, m_u1, m_v0, m_v1); }

    P2 operator()(const DVec3& p, const P2* hint) {
        const BSurface& s = m_s;
        const DVec3 d = p - s.f.o;
        const double x = ddot(d, s.f.x), y = ddot(d, s.f.y), z = ddot(d, s.f.z);
        switch (s.kind) {
        case BSurfaceKind::Plane: return {x, y};
        case BSurfaceKind::Cylinder:
        case BSurfaceKind::Cone: return {std::atan2(y, x), z};
        case BSurfaceKind::Sphere: return {std::atan2(y, x), std::atan2(z, std::hypot(x, y))};
        case BSurfaceKind::Torus: return {std::atan2(y, x), std::atan2(z, std::hypot(x, y) - s.r1)};
        case BSurfaceKind::Extrusion: return extrusion(p, hint);
        case BSurfaceKind::Revolution: return revolution(p, hint);
        default: return numeric(p, hint);
        }
    }

private:
    const BSurface& m_s;
    double m_u0, m_u1, m_v0, m_v1;
    std::vector<DVec3> m_grid;
    int m_gn = 0;

    bool bounded() const { return m_u1 - m_u0 < 1e11 && m_v1 - m_v0 < 1e11; }

    // The curve parameter minimising f(t): a scan of its range (or round `around` for an
    // unbounded line), then golden-section refinement.
    template <typename F>
    double minimise(const BCurve& c, double around, double reach, F f) const {
        double a, b;
        if (c.periodic()) a = 0, b = kPi2;
        else if (c.kind == BCurveKind::BSpline) a = c.bs.t0(), b = c.bs.t1();
        else if (c.kind == BCurveKind::Polyline) a = 0, b = static_cast<double>(std::max<std::size_t>(c.pts.size(), 2) - 1);
        else a = around - reach, b = around + reach;
        const int n = c.kind == BCurveKind::Line ? 8 : 128;
        double best = 1e300, bt = a;
        for (int i = 0; i <= n; ++i) {
            const double t = a + (b - a) * i / n;
            const double v = f(t);
            if (v < best) best = v, bt = t;
        }
        double lo = bt - (b - a) / n, hi = bt + (b - a) / n;
        // A closed curve goes on past its ends (its start is its end): no clamping there.
        const bool wraps = c.periodic() || (c.kind == BCurveKind::BSpline && c.bs.closed());
        if (!wraps && c.kind != BCurveKind::Line) lo = std::max(lo, a), hi = std::min(hi, b);
        constexpr double g = 0.6180339887498949;
        double x1 = hi - g * (hi - lo), x2 = lo + g * (hi - lo), f1 = f(x1), f2 = f(x2);
        for (int i = 0; i < 80; ++i) {
            if (f1 < f2) hi = x2, x2 = x1, f2 = f1, x1 = hi - g * (hi - lo), f1 = f(x1);
            else lo = x1, x1 = x2, f1 = f2, x2 = lo + g * (hi - lo), f2 = f(x2);
        }
        return 0.5 * (lo + hi);
    }
    const BCurve* swept() const {
        return tCurves && m_s.curve >= 0 && m_s.curve < static_cast<int>(tCurves->size()) ? &(*tCurves)[static_cast<std::size_t>(m_s.curve)] : nullptr;
    }
    // Extrusion: the curve point whose line along `dir` passes nearest p, then how far along.
    P2 extrusion(const DVec3& p, const P2* hint) const {
        const BCurve* c = swept();
        if (!c) return {0, 0};
        const double L2 = ddot(m_s.dir, m_s.dir);
        const DVec3 n = m_s.dir / std::sqrt(std::max(L2, 1e-300));
        auto off = [&](double t) {
            DVec3 d = c->eval(t) - p;
            d = d - n * ddot(d, n);
            return ddot(d, d);
        };
        const double around = c->kind == BCurveKind::Line ? c->param(p) : (hint ? hint->x : 0.0);
        const double reach = c->kind == BCurveKind::Line ? dlength(p - c->f.o) * 4 + 1 : 1.0;
        double u = minimise(*c, around, reach, off);
        if (hint && c->periodic()) u = wrapNear(u, hint->x, kPi2);
        else if (c->kind == BCurveKind::BSpline && c->bs.closed()) {
            const double P = c->bs.t1() - c->bs.t0();
            u = c->bs.t0() + std::fmod(std::fmod(u - c->bs.t0(), P) + P, P); // back into the range
            if (hint) u = wrapNear(u, hint->x, P);
        }
        return {u, ddot(p - c->eval(u), m_s.dir) / std::max(L2, 1e-300)};
    }
    // Revolution: the curve point at p's distance from the axis and height along it, then
    // the angle between them.
    P2 revolution(const DVec3& p, const P2* hint) const {
        const BCurve* c = swept();
        if (!c) return {0, 0};
        const DVec3 o = m_s.f.o, z = m_s.f.z;
        auto meridian = [&](const DVec3& q, double& rho, double& h) {
            const DVec3 d = q - o;
            h = ddot(d, z);
            rho = dlength(d - z * h);
        };
        double rp, hp;
        meridian(p, rp, hp);
        auto off = [&](double t) {
            double r, h;
            meridian(c->eval(t), r, h);
            return (r - rp) * (r - rp) + (h - hp) * (h - hp);
        };
        const double around = c->kind == BCurveKind::Line ? c->param(p) : 0.0;
        const double reach = c->kind == BCurveKind::Line ? dlength(p - c->f.o) * 4 + 1 : 1.0;
        double t = minimise(*c, around, reach, off);
        if (hint && c->periodic()) t = wrapNear(t, hint->y, kPi2);
        else if (c->kind == BCurveKind::BSpline && c->bs.closed()) {
            const double P = c->bs.t1() - c->bs.t0();
            t = c->bs.t0() + std::fmod(std::fmod(t - c->bs.t0(), P) + P, P);
            if (hint) t = wrapNear(t, hint->y, P);
        }
        // The angle from the curve point to p about the axis.
        const DVec3 a = c->eval(t) - o, b2 = p - o;
        const DVec3 ra = a - z * ddot(a, z), rb = b2 - z * ddot(b2, z);
        double ang = std::atan2(ddot(dcross(ra, rb), z), ddot(ra, rb));
        if (ang < 0) ang += kPi2;
        if (hint) ang = wrapNear(ang, hint->x, kPi2);
        return {ang, t};
    }

    P2 numeric(const DVec3& p, const P2* hint) {
        P2 uv{0, 0};
        double best = 1e300;
        if (hint) {
            uv = *hint;
            const DVec3 q = m_s.eval(uv.x, uv.y) - p;
            best = ddot(q, q);
        }
        if (!bounded() && !hint) return uv;
        if (!hint || best > 1e-6) {
            if (m_grid.empty() && bounded()) {
                m_gn = 48;
                m_grid.resize(static_cast<std::size_t>((m_gn + 1) * (m_gn + 1)));
                for (int i = 0; i <= m_gn; ++i)
                    for (int j = 0; j <= m_gn; ++j)
                        m_grid[static_cast<std::size_t>(i * (m_gn + 1) + j)] =
                            m_s.eval(m_u0 + (m_u1 - m_u0) * i / m_gn, m_v0 + (m_v1 - m_v0) * j / m_gn);
            }
            for (int i = 0; i <= m_gn; ++i)
                for (int j = 0; j <= m_gn; ++j) {
                    const DVec3 q = m_grid[static_cast<std::size_t>(i * (m_gn + 1) + j)] - p;
                    const double dd = ddot(q, q);
                    if (dd < best) best = dd, uv = {m_u0 + (m_u1 - m_u0) * i / m_gn, m_v0 + (m_v1 - m_v0) * j / m_gn};
                }
        }
        // Gauss-Newton on |S(u, v) - p|^2, clamped to the box.
        const double hu = (m_u1 - m_u0 < 1e11 ? m_u1 - m_u0 : 1.0) * 1e-6, hv = (m_v1 - m_v0 < 1e11 ? m_v1 - m_v0 : 1.0) * 1e-6;
        for (int it = 0; it < 40; ++it) {
            const DVec3 S = m_s.eval(uv.x, uv.y);
            const DVec3 Su = (m_s.eval(uv.x + hu, uv.y) - m_s.eval(uv.x - hu, uv.y)) / (2 * hu);
            const DVec3 Sv = (m_s.eval(uv.x, uv.y + hv) - m_s.eval(uv.x, uv.y - hv)) / (2 * hv);
            const DVec3 r = S - p;
            const double a = ddot(Su, Su), b = ddot(Su, Sv), c = ddot(Sv, Sv);
            const double g1 = ddot(Su, r), g2 = ddot(Sv, r);
            const double det = a * c - b * b;
            if (std::fabs(det) < 1e-30) break;
            double du = -(c * g1 - b * g2) / det, dv = -(a * g2 - b * g1) / det;
            double nu = uv.x + du, nv = uv.y + dv;
            if (bounded() && m_s.periodU() == 0) nu = std::clamp(nu, m_u0, m_u1);
            if (bounded() && m_s.periodV() == 0) nv = std::clamp(nv, m_v0, m_v1);
            const DVec3 q = m_s.eval(nu, nv) - p;
            if (ddot(q, q) > ddot(r, r) && it > 0) { // halve the step if it went uphill
                du *= 0.5, dv *= 0.5;
                nu = uv.x + du, nv = uv.y + dv;
                if (bounded() && m_s.periodU() == 0) nu = std::clamp(nu, m_u0, m_u1);
                if (bounded() && m_s.periodV() == 0) nv = std::clamp(nv, m_v0, m_v1);
            }
            uv = {nu, nv};
            if (std::fabs(du) < hu * 1e-3 && std::fabs(dv) < hv * 1e-3) break;
        }
        return uv;
    }
};

bool tessellateFace(TessContext& tc, const BFace& face, FaceMesh& out, std::string& err) {
    const Brep& b = tc.b;
    if (face.surface < 0 || face.surface >= static_cast<int>(b.surfaces.size())) {
        err = "a face without a surface";
        return false;
    }
    const BSurface& S = b.surfaces[static_cast<std::size_t>(face.surface)];
    const double Pu = S.periodU(), Pv = S.periodV();
    Inverse inv(S);
    // Each loop as 3D points (closed, no repeated end) with their uv, unwrapped.
    struct L {
        std::vector<DVec3> p;
        std::vector<P2> uv;
        double du = 0, dv = 0; // net change round the loop (a whole period: it wraps)
        bool pole = false;
    };
    std::vector<L> loops;
    for (const BLoop& lp : face.loops) {
        L l;
        if (!lp.edges.empty()) {
            for (const OrientedEdge& oe : lp.edges) {
                if (oe.edge < 0 || oe.edge >= static_cast<int>(b.edges.size())) continue;
                const std::vector<DVec3>& s = edgeSamples(tc, oe.edge);
                std::vector<DVec3> seg = s;
                if (!oe.forward) std::reverse(seg.begin(), seg.end());
                // Edges meet where one ends and the next starts: points a hair apart (two
                // vertices written to nine digits) are one.
                for (std::size_t i = 0; i < seg.size(); ++i) {
                    if (!l.p.empty() && dlength(seg[i] - l.p.back()) < 1e-9 * (1 + dlength(seg[i]))) continue;
                    if (!l.p.empty() && i == 0 && dlength(seg[0] - l.p.back()) < 1e-7 * (1 + dlength(seg[0]))) continue;
                    l.p.push_back(seg[i]);
                }
            }
            if (l.p.size() > 1 && dlength(l.p.front() - l.p.back()) < 1e-7 * (1 + dlength(l.p.back()))) l.p.pop_back();
        } else if (!lp.poly.empty()) {
            l.p = lp.poly;
        } else if (lp.vertex >= 0) {
            l.p = {b.vertices[static_cast<std::size_t>(lp.vertex)].p};
            l.pole = true;
        }
        if (!lp.sense) std::reverse(l.p.begin(), l.p.end());
        if (l.p.empty()) continue;
        P2 prev{0, 0};
        for (std::size_t i = 0; i < l.p.size(); ++i) {
            P2 q = inv(l.p[i], i ? &prev : nullptr);
            if (i) {
                q.x = wrapNear(q.x, prev.x, Pu);
                q.y = wrapNear(q.y, prev.y, Pv);
            }
            l.uv.push_back(q);
            prev = q;
        }
        if (l.uv.size() > 1) {
            P2 last = inv(l.p[0], &prev);
            last.x = wrapNear(last.x, prev.x, Pu);
            last.y = wrapNear(last.y, prev.y, Pv);
            l.du = last.x - l.uv[0].x;
            l.dv = last.y - l.uv[0].y;
        }
        loops.push_back(std::move(l));
    }
    if (std::getenv("SICDM_TESS_DEBUG")) {
        std::fprintf(stderr, "face: surface kind %d, periods %g %g, %zu loops\n", static_cast<int>(S.kind), Pu, Pv, loops.size());
        for (const L& l : loops) {
            std::fprintf(stderr, "  loop %zu pts, du %g dv %g pole %d:", l.p.size(), l.du, l.dv, l.pole ? 1 : 0);
            const std::size_t stride = std::getenv("SICDM_TESS_ALL") ? 1 : std::max<std::size_t>(1, l.uv.size() / 12);
            for (std::size_t i = 0; i < l.uv.size(); i += stride) std::fprintf(stderr, " (%.3f,%.3f)", l.uv[i].x, l.uv[i].y);
            std::fprintf(stderr, "\n");
        }
    }
    // Points on the sphere's and cone's singular lines take the u of their neighbours.
    auto singular = [&](const DVec3& p) {
        if (S.kind == BSurfaceKind::Sphere) return std::fabs(std::fabs(ddot(p - S.f.o, S.f.z)) - S.r1) < 1e-9 * (1 + S.r1);
        if (S.kind == BSurfaceKind::Cone) {
            const DVec3 apex = S.f.o - S.f.z * (S.r1 / std::tan(S.r2));
            return std::fabs(std::tan(S.r2)) > 1e-12 && dlength(p - apex) < 1e-9 * (1 + S.r1);
        }
        return false;
    };
    // A pole or apex on a loop is one point in 3D but a whole line in the parameters: it
    // becomes a segment along that line, from the incoming neighbour's u to the outgoing one's.
    auto singularV = [&](const DVec3& p) {
        if (S.kind == BSurfaceKind::Sphere) return ddot(p - S.f.o, S.f.z) > 0 ? kHalfPi : -kHalfPi;
        return -S.r1 / std::tan(S.r2); // the cone's apex
    };
    for (L& l : loops) {
        const std::size_t n = l.p.size();
        if (n < 2) continue;
        // A loop that wraps round the axis and passes a pole (a seam up to the pole and
        // back down, round the equator): its wrap is taken up at the pole, so it closes.
        if (Pu > 0 && std::fabs(std::fabs(l.du) - Pu) < 1e-6 * Pu) {
            std::size_t i0 = n;
            for (std::size_t i = 0; i < n && i0 == n; ++i)
                if (singular(l.p[i])) i0 = i;
            if (i0 < n) {
                for (std::size_t i = i0 + 1; i0 > 0 && i < n; ++i) l.uv[i].x -= l.du;
                l.du = 0;
            }
        }
        std::vector<DVec3> p2;
        std::vector<P2> uv2;
        for (std::size_t i = 0; i < n; ++i) {
            if (!singular(l.p[i])) {
                p2.push_back(l.p[i]);
                uv2.push_back(l.uv[i]);
                continue;
            }
            const P2& a = l.uv[(i + n - 1) % n];
            P2 c = l.uv[(i + 1) % n];
            if (i + 1 == n) c.x += l.du; // the loop's start, shifted as the loop wraps
            const double v = singularV(l.p[i]);
            p2.push_back(l.p[i]);
            uv2.push_back({a.x, v});
            if (std::fabs(c.x - a.x) > 1e-9) {
                p2.push_back(l.p[i]);
                uv2.push_back({c.x, v});
            }
        }
        l.p = std::move(p2);
        l.uv = std::move(uv2);
    }
    // The region in uv: a polygon set. Loops wrapping a periodic direction are closed off
    // along a seam line (or the singular line at a pole).
    std::vector<std::vector<P2>> rings;
    std::vector<std::vector<DVec3>> ringPts; // their 3D points (exact on edges; NaN: evaluate)
    const double nan = std::numeric_limits<double>::quiet_NaN();
    std::vector<int> wrapping, plain;
    for (int i = 0; i < static_cast<int>(loops.size()); ++i) {
        const L& l = loops[static_cast<std::size_t>(i)];
        if (l.pole) continue;
        const bool wu = Pu > 0 && std::fabs(std::fabs(l.du) - Pu) < 1e-6 * Pu;
        const bool wv = Pv > 0 && std::fabs(std::fabs(l.dv) - Pv) < 1e-6 * Pv;
        if (wu || wv) wrapping.push_back(i);
        else plain.push_back(i);
    }
    double U0, U1, V0, V1;
    S.domain(U0, U1, V0, V1);
    auto seamLine = [&](P2 a, P2 c, std::vector<P2>& uv, std::vector<DVec3>& p) {
        // From a to c (exclusive of both), straight in uv, sampled by the surface's curvature.
        const DVec3 A = S.eval(a.x, a.y), C = S.eval(c.x, c.y);
        int n = 1;
        // Enough steps for the chord tolerance along the line: sample and measure.
        const int probe = 32;
        double len = 0;
        DVec3 prevP = A;
        for (int k = 1; k <= probe; ++k) {
            const double t = static_cast<double>(k) / probe;
            const DVec3 q = S.eval(a.x + (c.x - a.x) * t, a.y + (c.y - a.y) * t);
            len += dlength(q - prevP);
            prevP = q;
        }
        const double chordLen = dlength(C - A);
        const double sag = std::max(0.0, len - chordLen);
        n = std::max(1, static_cast<int>(std::ceil(std::sqrt(sag / std::max(tc.o.chord, 1e-6)) * 2.0)));
        n = std::max(n, static_cast<int>(std::ceil(len / std::max(tc.o.maxEdge > 0 ? tc.o.maxEdge : 1e300, 1e-9))));
        // Angle-based: how far the normal turns along it.
        const DVec3 nA = S.normal(a.x, a.y), nC = S.normal(c.x, c.y);
        const double turn = std::acos(std::clamp(ddot(nA, nC), -1.0, 1.0));
        n = std::max(n, static_cast<int>(std::ceil(turn / tc.o.angle)));
        n = std::min(n, 4096);
        for (int k = 1; k < n; ++k) {
            const double t = static_cast<double>(k) / n;
            uv.push_back({a.x + (c.x - a.x) * t, a.y + (c.y - a.y) * t});
            p.push_back({nan, nan, nan});
        }
    };
    if (!wrapping.empty()) {
        // Which parameter they wrap in.
        const L& first = loops[static_cast<std::size_t>(wrapping[0])];
        const bool alongU = Pu > 0 && std::fabs(std::fabs(first.du) - Pu) < 1e-6 * Pu;
        const double P = alongU ? Pu : Pv;
        // Each wrapping loop's level in the other parameter, and its direction.
        struct W {
            int loop;
            double level;
            double dir;
        };
        std::vector<W> ws;
        for (int i : wrapping) {
            const L& l = loops[static_cast<std::size_t>(i)];
            double lv = 0;
            for (const P2& q : l.uv) lv += alongU ? q.y : q.x;
            lv /= static_cast<double>(l.uv.size());
            ws.push_back({i, lv, alongU ? (l.du > 0 ? 1.0 : -1.0) : (l.dv > 0 ? 1.0 : -1.0)});
        }
        std::sort(ws.begin(), ws.end(), [](const W& x, const W& y) { return x.level < y.level; });
        // Cuts a wrapping loop to start exactly at `at` (in the wrapping parameter, mod P),
        // giving its points from `at` round to at + P (or at - P).
        auto cut = [&](const L& l, double at, std::vector<P2>& uv, std::vector<DVec3>& p) {
            const std::size_t n = l.uv.size();
            auto coord = [&](const P2& q) { return alongU ? q.x : q.y; };
            // The segment crossing `at` (mod P).
            for (std::size_t i = 0; i < n; ++i) {
                const P2 a = l.uv[i];
                P2 c = l.uv[(i + 1) % n];
                if (i + 1 == n) (alongU ? c.x : c.y) += alongU ? l.du : l.dv;
                const double ca = coord(a), cc = coord(c);
                const double target = wrapNear(at, 0.5 * (ca + cc), P);
                const double lo = std::min(ca, cc), hi = std::max(ca, cc);
                if (target < lo - 1e-12 || target > hi + 1e-12 || hi - lo < 1e-15) continue;
                const double t = (target - ca) / (cc - ca);
                const P2 m{a.x + (c.x - a.x) * t, a.y + (c.y - a.y) * t};
                const DVec3 pm = l.p[i] + (l.p[(i + 1) % n] - l.p[i]) * t;
                const double shift = (alongU ? m.x : m.y) - at; // to bring m onto `at`
                auto shifted = [&](P2 q) {
                    (alongU ? q.x : q.y) -= shift;
                    return q;
                };
                uv.push_back(shifted(m));
                p.push_back(t < 1e-9 ? l.p[i] : t > 1 - 1e-9 ? l.p[(i + 1) % n] : pm);
                for (std::size_t k = 1; k <= n; ++k) {
                    const std::size_t idx = (i + k) % n;
                    P2 q = l.uv[idx];
                    if (i + k >= n) (alongU ? q.x : q.y) += alongU ? l.du : l.dv;
                    const bool atEnd = k == n && t < 1e-9;
                    if (atEnd) break;
                    if (k == 1 && t > 1 - 1e-9) continue; // m is that point
                    uv.push_back(shifted(q));
                    p.push_back(l.p[idx]);
                }
                // Close at at +- P.
                P2 e = uv.front();
                (alongU ? e.x : e.y) += (alongU ? l.du : l.dv);
                uv.push_back(e);
                p.push_back(p.front());
                return true;
            }
            return false;
        };
        const double at = alongU ? loops[static_cast<std::size_t>(ws[0].loop)].uv[0].x : loops[static_cast<std::size_t>(ws[0].loop)].uv[0].y;
        // Pairs of wrapping loops bound bands; a single one is closed off at the domain's
        // edge (the pole of a sphere, the apex of a cone) on the side the face lies.
        std::vector<P2> ring;
        std::vector<DVec3> rp;
        auto addBand = [&](const W& lowW, const W* highW, double limit) {
            std::vector<P2> lo, hi;
            std::vector<DVec3> lop, hip;
            cut(loops[static_cast<std::size_t>(lowW.loop)], at, lo, lop);
            // Make the lower boundary run increasing in the wrapping parameter.
            auto c = [&](const P2& q) { return alongU ? q.x : q.y; };
            if (c(lo.back()) < c(lo.front())) {
                std::reverse(lo.begin(), lo.end());
                std::reverse(lop.begin(), lop.end());
            }
            // Shift so lo starts at `at` exactly.
            const double s0 = c(lo.front()) - at;
            for (P2& q : lo) (alongU ? q.x : q.y) -= s0;
            if (highW) {
                cut(loops[static_cast<std::size_t>(highW->loop)], at, hi, hip);
                if (c(hi.back()) < c(hi.front())) {
                    std::reverse(hi.begin(), hi.end());
                    std::reverse(hip.begin(), hip.end());
                }
                const double s1 = c(hi.front()) - at;
                for (P2& q : hi) (alongU ? q.x : q.y) -= s1;
            } else {
                // The limit line, sampled like a seam, from at + P back to at.
                P2 a = alongU ? P2{at + P, limit} : P2{limit, at + P};
                P2 e = alongU ? P2{at, limit} : P2{limit, at};
                hi.push_back(e);
                hip.push_back({nan, nan, nan});
                std::vector<P2> mid;
                std::vector<DVec3> midp;
                seamLine(e, a, mid, midp);
                for (std::size_t k = 0; k < mid.size(); ++k) hi.push_back(mid[k]), hip.push_back(midp[k]);
                hi.push_back(a);
                hip.push_back({nan, nan, nan});
            }
            // Ring: lo forwards, seam up at at + P, hi backwards, seam down at `at`.
            ring.clear();
            rp.clear();
            for (std::size_t k = 0; k < lo.size(); ++k) ring.push_back(lo[k]), rp.push_back(lop[k]);
            seamLine(lo.back(), hi.back(), ring, rp);
            for (std::size_t k = hi.size(); k-- > 0;) ring.push_back(hi[k]), rp.push_back(hip[k]);
            seamLine(hi.front(), lo.front(), ring, rp);
            rings.push_back(ring);
            ringPts.push_back(rp);
        };
        // Orientation of the parameters against the face's outward normal.
        const L& l0 = loops[static_cast<std::size_t>(ws[0].loop)];
        const P2 mid = l0.uv[l0.uv.size() / 2];
        const DVec3 outward = S.normal(mid.x, mid.y) * (face.sameSense ? 1.0 : -1.0);
        double sDu = 1, sDv = 1;
        {
            const double h = 1e-5;
            const DVec3 su = S.eval(mid.x + h, mid.y) - S.eval(mid.x - h, mid.y), sv = S.eval(mid.x, mid.y + h) - S.eval(mid.x, mid.y - h);
            const double s = ddot(dcross(su, sv), outward) >= 0 ? 1.0 : -1.0; // uv is counter-clockwise seen from outside
            sDu = s, sDv = s;
        }
        // Pole vertices tell which end of the domain a single wrapping loop is closed at.
        std::vector<double> poleLevels;
        for (const L& l : loops)
            if (l.pole) {
                const P2 q = inv(l.p[0], nullptr);
                poleLevels.push_back(alongU ? q.y : q.x);
            }
        const double lim0 = alongU ? V0 : U0, lim1 = alongU ? V1 : U1;
        if (ws.size() == 1) {
            // The face lies to the left of the loop (seen from outside): for a loop going
            // + in the wrapping parameter that is + in the other (when uv is CCW from outside).
            double side = ws[0].dir * (alongU ? sDu : -sDv);
            if (!poleLevels.empty()) side = poleLevels[0] > ws[0].level ? 1.0 : -1.0;
            double limit = side > 0 ? lim1 : lim0;
            if (std::fabs(limit) > 1e11) { // an unbounded direction (a cone opening away): can't close it
                err = "an open face round a periodic surface";
                return false;
            }
            addBand(ws[0], nullptr, limit);
        } else {
            // Bands between consecutive wrapping loops (in level order): which pairs bound the
            // face follows from the lowest loop's side.
            double side = ws[0].dir * (alongU ? sDu : -sDv);
            std::size_t k = 0;
            if (side < 0) { // the face is below the lowest loop: closed off at the domain's low end
                addBand(ws[0], nullptr, lim0);
                k = 1;
            }
            for (; k + 1 < ws.size(); k += 2) addBand(ws[k], &ws[k + 1], 0);
            if (k < ws.size()) addBand(ws[k], nullptr, lim1);
        }
        // Holes (plain loops) shift into the band's range.
        for (int i : plain) {
            const L& l = loops[static_cast<std::size_t>(i)];
            std::vector<P2> uv = l.uv;
            double c = 0;
            for (const P2& q : uv) c += alongU ? q.x : q.y;
            c /= static_cast<double>(uv.size());
            const double target = at + 0.5 * P;
            const double sh = wrapNear(c, target, P) - c;
            for (P2& q : uv) (alongU ? q.x : q.y) += sh;
            rings.push_back(uv);
            ringPts.push_back(l.p);
        }
    } else if (!plain.empty()) {
        // Holes into the outer loop's periodic range.
        int outer = plain[0];
        double bestArea = -1;
        for (int i : plain) {
            const L& l = loops[static_cast<std::size_t>(i)];
            double x0 = 1e300, x1 = -1e300, y0 = 1e300, y1 = -1e300;
            for (const P2& q : l.uv) x0 = std::min(x0, q.x), x1 = std::max(x1, q.x), y0 = std::min(y0, q.y), y1 = std::max(y1, q.y);
            const double area = (x1 - x0) * (y1 - y0);
            const BLoop& src = face.loops[static_cast<std::size_t>(i)];
            if (src.outer) {
                outer = i;
                break;
            }
            if (area > bestArea) bestArea = area, outer = i;
        }
        const L& O = loops[static_cast<std::size_t>(outer)];
        double ox = 0, oy = 0;
        for (const P2& q : O.uv) ox += q.x, oy += q.y;
        ox /= static_cast<double>(O.uv.size()), oy /= static_cast<double>(O.uv.size());
        for (int i : plain) {
            const L& l = loops[static_cast<std::size_t>(i)];
            std::vector<P2> uv = l.uv;
            if (i != outer) {
                double cx = 0, cy = 0;
                for (const P2& q : uv) cx += q.x, cy += q.y;
                cx /= static_cast<double>(uv.size()), cy /= static_cast<double>(uv.size());
                const double sx = Pu > 0 ? wrapNear(cx, ox, Pu) - cx : 0, sy = Pv > 0 ? wrapNear(cy, oy, Pv) - cy : 0;
                for (P2& q : uv) q.x += sx, q.y += sy;
            }
            rings.push_back(uv);
            ringPts.push_back(l.p);
        }
    } else {
        // No edges: the whole (closed) surface.
        if (std::fabs(U1 - U0) > 1e11 || std::fabs(V1 - V0) > 1e11) {
            err = "an unbounded face";
            return false;
        }
        std::vector<P2> uv;
        std::vector<DVec3> p;
        const P2 c0{U0, V0}, c1{U1, V0}, c2{U1, V1}, c3{U0, V1};
        for (const auto& [a, c] : {std::pair{c0, c1}, std::pair{c1, c2}, std::pair{c2, c3}, std::pair{c3, c0}}) {
            uv.push_back(a);
            p.push_back({nan, nan, nan});
            seamLine(a, c, uv, p);
        }
        rings.push_back(uv);
        ringPts.push_back(p);
    }
    if (rings.empty()) {
        err = "a face without loops";
        return false;
    }
    // Scale uv so lengths in it are roughly lengths on the surface (better triangles).
    double x0 = 1e300, x1 = -1e300, y0 = 1e300, y1 = -1e300;
    for (const auto& r : rings)
        for (const P2& q : r) x0 = std::min(x0, q.x), x1 = std::max(x1, q.x), y0 = std::min(y0, q.y), y1 = std::max(y1, q.y);
    const P2 c{0.5 * (x0 + x1), 0.5 * (y0 + y1)};
    double su = 1, sv = 1;
    {
        const double hu = std::max((x1 - x0) * 1e-3, 1e-9), hv = std::max((y1 - y0) * 1e-3, 1e-9);
        double lu = 0, lv = 0;
        int n = 0;
        for (double fx : {0.25, 0.5, 0.75})
            for (double fy : {0.25, 0.5, 0.75}) {
                const double u = x0 + (x1 - x0) * fx, v = y0 + (y1 - y0) * fy;
                lu += dlength(S.eval(u + hu, v) - S.eval(u - hu, v)) / (2 * hu);
                lv += dlength(S.eval(u, v + hv) - S.eval(u, v - hv)) / (2 * hv);
                ++n;
            }
        su = std::max(lu / n, 1e-9), sv = std::max(lv / n, 1e-9);
    }
    auto toS = [&](const P2& q) { return P2{(q.x - c.x) * su, (q.y - c.y) * sv}; };
    auto fromS = [&](const P2& q) { return P2{q.x / su + c.x, q.y / sv + c.y}; };
    double sx0 = 1e300, sx1 = -1e300, sy0 = 1e300, sy1 = -1e300;
    for (const auto& r : rings)
        for (const P2& q : r) {
            const P2 s = toS(q);
            sx0 = std::min(sx0, s.x), sx1 = std::max(sx1, s.x), sy0 = std::min(sy0, s.y), sy1 = std::max(sy1, s.y);
        }
    Cdt cdt(sx0, sy0, sx1, sy1);
    cdt.setTiny(1e-11 * std::max(sx1 - sx0, sy1 - sy0));
    std::vector<DVec3> p3{{nan, nan, nan}, {nan, nan, nan}, {nan, nan, nan}}; // super triangle
    std::vector<std::vector<int>> ids(rings.size());
    for (std::size_t r = 0; r < rings.size(); ++r) {
        int hint = -1;
        for (std::size_t k = 0; k < rings[r].size(); ++k) {
            const int id = cdt.insert(toS(rings[r][k]), hint);
            if (id < 0) {
                err = "triangulation failed";
                return false;
            }
            if (static_cast<std::size_t>(id) >= p3.size()) p3.resize(static_cast<std::size_t>(id) + 1, {nan, nan, nan});
            if (!std::isnan(ringPts[r][k].x)) p3[static_cast<std::size_t>(id)] = ringPts[r][k];
            ids[r].push_back(id);
        }
    }
    for (std::size_t r = 0; r < ids.size(); ++r)
        for (std::size_t k = 0; k < ids[r].size(); ++k)
            if (!cdt.force(ids[r][k], ids[r][(k + 1) % ids[r].size()])) {
                err = "a face's boundary crosses itself";
                return false;
            }
    // Inside: an odd number of rings round the triangle's centroid.
    std::vector<std::vector<P2>> sr(rings.size());
    for (std::size_t r = 0; r < rings.size(); ++r)
        for (const P2& q : rings[r]) sr[r].push_back(toS(q));
    // The ring edges binned by their span in y, so a crossing test looks only at the edges
    // level with the point (a face with a thousand holes would otherwise test every edge
    // for every triangle).
    struct RingEdge {
        P2 a, b;
    };
    std::vector<RingEdge> redges;
    double ey0 = 1e300, ey1 = -1e300;
    for (const auto& r : sr)
        for (std::size_t i = 0, j = r.size() - 1; i < r.size(); j = i++) {
            redges.push_back({r[i], r[j]});
            ey0 = std::min({ey0, r[i].y, r[j].y}), ey1 = std::max({ey1, r[i].y, r[j].y});
        }
    const std::size_t nbins = std::clamp<std::size_t>(redges.size() / 8, 1, 4096);
    const double binH = std::max((ey1 - ey0) / static_cast<double>(nbins), 1e-300);
    std::vector<std::vector<std::uint32_t>> bins(nbins);
    auto binOf = [&](double y) { return static_cast<std::size_t>(std::clamp((y - ey0) / binH, 0.0, static_cast<double>(nbins - 1))); };
    for (std::size_t k = 0; k < redges.size(); ++k) {
        const auto& e = redges[k];
        for (std::size_t bi = binOf(std::min(e.a.y, e.b.y)), b1 = binOf(std::max(e.a.y, e.b.y)); bi <= b1; ++bi) bins[bi].push_back(static_cast<std::uint32_t>(k));
    }
    auto inside = [&](const P2& q) {
        if (redges.empty() || q.y < ey0 || q.y > ey1) return false;
        bool in = false;
        for (std::uint32_t k : bins[binOf(q.y)]) {
            const P2 &ea = redges[k].a, &eb = redges[k].b;
            if ((ea.y > q.y) != (eb.y > q.y) && q.x < (eb.x - ea.x) * (q.y - ea.y) / (eb.y - ea.y) + ea.x) in = !in;
        }
        return in;
    };
    auto isOut = [&](const Cdt::Tri& t) {
        if (t.v[0] < 3 || t.v[1] < 3 || t.v[2] < 3) return true;
        const P2 &a = cdt.pts[static_cast<std::size_t>(t.v[0])], &b2 = cdt.pts[static_cast<std::size_t>(t.v[1])], &c2 = cdt.pts[static_cast<std::size_t>(t.v[2])];
        return !inside({(a.x + b2.x + c2.x) / 3, (a.y + b2.y + c2.y) / 3});
    };
    for (Cdt::Tri& t : cdt.tris)
        if (!t.dead && isOut(t)) t.dead = true;
    // 3D points of the triangulation's vertices (interior ones from the surface).
    auto point3 = [&](int v) {
        if (static_cast<std::size_t>(v) >= p3.size()) p3.resize(static_cast<std::size_t>(v) + 1, {nan, nan, nan});
        DVec3& q = p3[static_cast<std::size_t>(v)];
        if (std::isnan(q.x)) {
            const P2 uv = fromS(cdt.pts[static_cast<std::size_t>(v)]);
            q = S.eval(uv.x, uv.y);
        }
        return q;
    };
    // Interior points on curved surfaces: rows and columns spaced for the chord and angle
    // tolerances at the face's sharpest curvature in each direction, stepped by the
    // surface's local speed (so they don't crowd at a pole), kept clear of the boundary.
    const bool curved = S.kind != BSurfaceKind::Plane;
    if (curved || tc.o.maxEdge > 0) {
        double ku = 0, kv = 0; // largest curvature along u and v
        const double du = std::max((x1 - x0) * 1e-3, 1e-9), dvv = std::max((y1 - y0) * 1e-3, 1e-9);
        for (int i = 1; i < 8; ++i) // inside the box: not on a pole or apex at its edge
            for (int j = 1; j < 8; ++j) {
                const double u = x0 + (x1 - x0) * i / 8.0, v = y0 + (y1 - y0) * j / 8.0;
                const DVec3 n0 = S.normal(u - du, v), n1 = S.normal(u + du, v);
                const double lu = dlength(S.eval(u + du, v) - S.eval(u - du, v));
                if (lu > 1e-12) ku = std::max(ku, std::acos(std::clamp(ddot(n0, n1), -1.0, 1.0)) / lu);
                const DVec3 m0 = S.normal(u, v - dvv), m1 = S.normal(u, v + dvv);
                const double lv = dlength(S.eval(u, v + dvv) - S.eval(u, v - dvv));
                if (lv > 1e-12) kv = std::max(kv, std::acos(std::clamp(ddot(m0, m1), -1.0, 1.0)) / lv);
            }
        auto spacing = [&](double k) { // mm between points for curvature k
            double h = 1e300;
            if (k > 1e-12) {
                const double rho = 1.0 / k;
                const double sag = std::min(tc.o.chord, rho);
                h = std::min(2.0 * std::sqrt(std::max(2.0 * rho * sag - sag * sag, 0.0)), rho * tc.o.angle);
            }
            if (tc.o.maxEdge > 0) h = std::min(h, tc.o.maxEdge);
            return h;
        };
        double hu = spacing(ku), hv = spacing(kv);
        // Not too long one way and short the other: triangles across a straight direction
        // must not cut the curved one.
        if (hu < 1e299 || hv < 1e299) {
            hu = std::min(hu, 3.0 * hv);
            hv = std::min(hv, 3.0 * hu);
        }
        // Boundary points, hashed, to keep new points half a spacing away from them.
        const double hmin = std::max(std::min(hu, hv), 1e-9);
        if (hmin < 1e299) {
            std::unordered_map<std::uint64_t, std::vector<int>> near;
            const double cellB = hmin;
            auto keyOf = [&](const DVec3& q) {
                const auto cx = static_cast<std::int64_t>(std::floor(q.x / cellB)), cy = static_cast<std::int64_t>(std::floor(q.y / cellB)),
                           cz = static_cast<std::int64_t>(std::floor(q.z / cellB));
                return (static_cast<std::uint64_t>(cx) * 73856093ull) ^ (static_cast<std::uint64_t>(cy) * 19349663ull) ^ (static_cast<std::uint64_t>(cz) * 83492791ull);
            };
            std::vector<DVec3> bpts;
            for (std::size_t r = 0; r < rings.size(); ++r)
                for (std::size_t k = 0; k < rings[r].size(); ++k) {
                    const DVec3 q = std::isnan(ringPts[r][k].x) ? S.eval(rings[r][k].x, rings[r][k].y) : ringPts[r][k];
                    near[keyOf(q)].push_back(static_cast<int>(bpts.size()));
                    bpts.push_back(q);
                }
            auto clear = [&](const DVec3& q) {
                const auto cx = static_cast<std::int64_t>(std::floor(q.x / cellB)), cy = static_cast<std::int64_t>(std::floor(q.y / cellB)),
                           cz = static_cast<std::int64_t>(std::floor(q.z / cellB));
                for (std::int64_t a2 = cx - 1; a2 <= cx + 1; ++a2)
                    for (std::int64_t b3 = cy - 1; b3 <= cy + 1; ++b3)
                        for (std::int64_t c3 = cz - 1; c3 <= cz + 1; ++c3) {
                            const auto it = near.find((static_cast<std::uint64_t>(a2) * 73856093ull) ^ (static_cast<std::uint64_t>(b3) * 19349663ull) ^
                                                      (static_cast<std::uint64_t>(c3) * 83492791ull));
                            if (it == near.end()) continue;
                            for (int i : it->second)
                                if (dlength(bpts[static_cast<std::size_t>(i)] - q) < 0.5 * hmin) return false;
                        }
                return true;
            };
            // Rows along v, points along u, each step the spacing over the local speed.
            const double eu = std::max((x1 - x0) * 1e-4, 1e-9), ev = std::max((y1 - y0) * 1e-4, 1e-9);
            auto speedU = [&](double u, double v) { return std::max(dlength(S.eval(u + eu, v) - S.eval(u - eu, v)) / (2 * eu), 1e-12); };
            auto speedV = [&](double u, double v) { return std::max(dlength(S.eval(u, v + ev) - S.eval(u, v - ev)) / (2 * ev), 1e-12); };
            std::size_t added = 0;
            const std::size_t cap = 150000;
            const double um = 0.5 * (x0 + x1);
            for (double v = y0 + 0.5 * hv / speedV(um, y0 + (y1 - y0) * 1e-3); v < y1 && added < cap;) {
                const double rowSpeed = speedV(um, v);
                if (hv < 1e299 || hu < 1e299) {
                    for (double u = x0 + std::min(hu, (x1 - x0) * speedU(x0, v)) / speedU(x0, v) * 0.5; u < x1 && added < cap;) {
                        const double su2 = speedU(u, v);
                        const P2 q{u, v};
                        const P2 sq = toS(q);
                        if (inside(sq)) {
                            const DVec3 p = S.eval(u, v);
                            if (clear(p)) {
                                const int id = cdt.insert(sq);
                                if (id >= 0) {
                                    if (static_cast<std::size_t>(id) >= p3.size()) p3.resize(static_cast<std::size_t>(id) + 1, {nan, nan, nan});
                                    p3[static_cast<std::size_t>(id)] = p;
                                    ++added;
                                }
                            }
                        }
                        const double stepU = hu < 1e299 ? hu / su2 : (x1 - x0);
                        u += std::max(stepU, (x1 - x0) * 1e-6);
                    }
                }
                const double stepV = hv / rowSpeed;
                v += std::max(stepV, (y1 - y0) * 1e-6);
            }
            if (added) {
                // New points may have landed in removed (outside) triangles' neighbours: re-mark.
                for (Cdt::Tri& t : cdt.tris)
                    if (!t.dead && isOut(t)) t.dead = true;
            }
        }
    }
    // Out: 3D points and triangles facing out of the solid.
    std::unordered_map<int, int> map;
    for (const Cdt::Tri& t : cdt.tris) {
        if (t.dead) continue;
        std::array<int, 3> tri{};
        for (int k = 0; k < 3; ++k) {
            const int v = t.v[k];
            const auto [it, added] = map.try_emplace(v, static_cast<int>(out.p.size()));
            if (added) {
                out.p.push_back(point3(v));
                const P2 uv = fromS(cdt.pts[static_cast<std::size_t>(v)]);
                out.n.push_back(S.normal(uv.x, uv.y) * (face.sameSense ? 1.0 : -1.0));
            }
            tri[static_cast<std::size_t>(k)] = it->second;
        }
        const DVec3 &A = out.p[static_cast<std::size_t>(tri[0])], &B = out.p[static_cast<std::size_t>(tri[1])], &C = out.p[static_cast<std::size_t>(tri[2])];
        const DVec3 nt = dcross(B - A, C - A);
        if (dlength(nt) < 1e-24) continue; // collapsed at a pole
        const P2 cu = fromS({(cdt.pts[static_cast<std::size_t>(t.v[0])].x + cdt.pts[static_cast<std::size_t>(t.v[1])].x + cdt.pts[static_cast<std::size_t>(t.v[2])].x) / 3,
                             (cdt.pts[static_cast<std::size_t>(t.v[0])].y + cdt.pts[static_cast<std::size_t>(t.v[1])].y + cdt.pts[static_cast<std::size_t>(t.v[2])].y) / 3});
        const DVec3 want = S.normal(cu.x, cu.y) * (face.sameSense ? 1.0 : -1.0);
        if (ddot(nt, want) < 0) std::swap(tri[1], tri[2]);
        out.t.push_back(tri);
    }
    return true;
}

} // namespace

MeshData tessellate(const Brep& brep, const TessellateOptions& o, TessellateReport* report) {
    MeshData m;
    const std::vector<BCurve>* saved = tCurves;
    tCurves = &brep.curves;
    TessContext tc{brep, o, {}, {}};
    tc.edgePts.resize(brep.edges.size());
    tc.edgeDone.assign(brep.edges.size(), 0);
    TessellateReport rep;
    std::vector<const BFace*> faces;
    for (const BShell& sh : brep.shells)
        for (const BFace& f : sh.faces) faces.push_back(&f);
    // Edges first: faces share them (so neighbours meet point for point), and with every
    // edge sampled the faces read them only and can be meshed side by side.
    for (std::size_t e = 0; e < brep.edges.size(); ++e) edgeSamples(tc, static_cast<int>(e));
    for (const BCurve& c : brep.curves)
        if (c.kind == BCurveKind::BSpline && !c.bs.knots.empty()) (void)c.bs.closed(); // fill the lazy caches up front
    for (const BSurface& su : brep.surfaces)
        if (su.kind == BSurfaceKind::BSpline && !su.bs.ku.empty() && !su.bs.kv.empty()) (void)su.bs.closedU(), (void)su.bs.closedV();
    std::vector<FaceMesh> meshes(faces.size());
    std::vector<std::string> errors(faces.size());
    std::vector<char> ok(faces.size(), 0);
    auto work = [&](std::size_t i) {
        try {
            ok[i] = tessellateFace(tc, *faces[i], meshes[i], errors[i]) ? 1 : 0;
        } catch (const std::exception& ex) {
            errors[i] = ex.what();
        }
    };
    const unsigned hw = std::max(1u, std::thread::hardware_concurrency());
    const std::size_t threads = std::min<std::size_t>({hw, 16, (faces.size() + 7) / 8});
    if (threads > 1) {
        std::atomic<std::size_t> next{0};
        std::vector<std::thread> pool;
        for (std::size_t k = 0; k < threads; ++k)
            pool.emplace_back([&] {
                tCurves = &brep.curves; // per thread
                for (std::size_t i; (i = next.fetch_add(1)) < faces.size();) work(i);
            });
        for (std::thread& t : pool) t.join();
    } else {
        for (std::size_t i = 0; i < faces.size(); ++i) work(i);
    }
    for (std::size_t fi = 0; fi < faces.size(); ++fi) {
        {
            ++rep.faces;
            FaceMesh& fm = meshes[fi];
            const std::size_t before = m.indices.size();
            if (!ok[fi]) {
                ++rep.failedFaces;
                if (rep.firstError.empty()) rep.firstError = errors[fi];
                rep.faceTriangles.push_back(0);
                continue;
            }
            if (std::getenv("SICDM_TESS_DEBUG")) {
                std::map<std::pair<int, int>, int> cnt;
                for (const auto& t : fm.t)
                    for (int e = 0; e < 3; ++e) {
                        int a = t[static_cast<std::size_t>(e)], c = t[static_cast<std::size_t>((e + 1) % 3)];
                        if (a > c) std::swap(a, c);
                        ++cnt[{a, c}];
                    }
                int over = 0;
                for (const auto& [e, c] : cnt) over += c > 2;
                std::fprintf(stderr, "  -> %zu triangles, %d edges used more than twice\n", fm.t.size(), over);
            }
            // Weld inside the face (seams and poles meet themselves), keep its own normals.
            const auto base = static_cast<std::uint32_t>(m.vertices.size());
            struct Key {
                long long x, y, z;
                bool operator<(const Key& o) const { return x != o.x ? x < o.x : y != o.y ? y < o.y : z < o.z; }
            };
            std::map<Key, std::uint32_t> weld; // full coordinates: no two points share a key by accident
            std::vector<std::uint32_t> id(fm.p.size());
            double scale = 0;
            for (const DVec3& q : fm.p) scale = std::max({scale, std::fabs(q.x), std::fabs(q.y), std::fabs(q.z)});
            const double cell = std::max(scale, 1.0) * 1e-9;
            for (std::size_t i = 0; i < fm.p.size(); ++i) {
                const DVec3& q = fm.p[i];
                const Key key{std::llround(q.x / cell), std::llround(q.y / cell), std::llround(q.z / cell)};
                const auto [it, added] = weld.try_emplace(key, static_cast<std::uint32_t>(m.vertices.size()));
                if (added) {
                    Vertex v;
                    v.position = q.f();
                    v.normal = fm.n[i].f();
                    m.vertices.push_back(v);
                }
                id[i] = it->second;
            }
            (void)base;
            for (const auto& t : fm.t) {
                const std::uint32_t a = id[static_cast<std::size_t>(t[0])], b = id[static_cast<std::size_t>(t[1])], c = id[static_cast<std::size_t>(t[2])];
                if (a == b || b == c || c == a) continue;
                m.indices.insert(m.indices.end(), {a, b, c});
            }
            rep.faceTriangles.push_back((m.indices.size() - before) / 3);
        }
    }
    tCurves = saved;
    if (report) *report = rep;
    m.computeFeatureEdges();
    return m;
}

DVec3 surfaceNormalNear(const Brep& b, const BSurface& s, const DVec3& p) {
    const std::vector<BCurve>* saved = tCurves;
    tCurves = &b.curves;
    Inverse inv(s);
    const P2 uv = inv(p, nullptr);
    const DVec3 n = s.normal(uv.x, uv.y);
    tCurves = saved;
    return n;
}

void orientLoops(Brep& b) {
    const std::vector<BCurve>* saved = tCurves;
    tCurves = &b.curves;
    TessellateOptions o;
    o.chord = 0.05;
    TessContext tc{b, o, {}, {}};
    tc.edgePts.resize(b.edges.size());
    tc.edgeDone.assign(b.edges.size(), 0);
    for (BShell& sh : b.shells)
        for (BFace& face : sh.faces) {
            if (face.surface < 0 || face.surface >= static_cast<int>(b.surfaces.size())) continue;
            const BSurface& S = b.surfaces[static_cast<std::size_t>(face.surface)];
            const double Pu = S.periodU(), Pv = S.periodV();
            Inverse inv(S);
            // Each loop's area in the parameters (du x dv is the surface's normal), as it runs.
            std::vector<double> area(face.loops.size(), 0.0);
            std::vector<char> usable(face.loops.size(), 0);
            for (std::size_t li = 0; li < face.loops.size(); ++li) {
                const BLoop& lp = face.loops[li];
                if (lp.edges.empty()) continue;
                std::vector<DVec3> pts;
                for (const OrientedEdge& oe : lp.edges) {
                    if (oe.edge < 0 || oe.edge >= static_cast<int>(b.edges.size())) continue;
                    std::vector<DVec3> seg = edgeSamples(tc, oe.edge);
                    if (!oe.forward) std::reverse(seg.begin(), seg.end());
                    for (std::size_t i = pts.empty() ? 0 : 1; i < seg.size(); ++i) pts.push_back(seg[i]);
                }
                if (!lp.sense) std::reverse(pts.begin(), pts.end());
                if (pts.size() < 3) continue;
                std::vector<P2> uv;
                P2 prev{0, 0};
                for (std::size_t i = 0; i < pts.size(); ++i) {
                    P2 q = inv(pts[i], i ? &prev : nullptr);
                    if (i) q.x = wrapNear(q.x, prev.x, Pu), q.y = wrapNear(q.y, prev.y, Pv);
                    uv.push_back(q);
                    prev = q;
                }
                const double du = uv.back().x - uv.front().x, dv = uv.back().y - uv.front().y;
                if ((Pu > 0 && std::fabs(du) > Pu / 2) || (Pv > 0 && std::fabs(dv) > Pv / 2)) continue; // wraps round: no area
                double a = 0;
                for (std::size_t i = 0; i < uv.size(); ++i) {
                    const P2 &p0 = uv[i], &p1 = uv[(i + 1) % uv.size()];
                    a += p0.x * p1.y - p1.x * p0.y;
                }
                area[li] = 0.5 * a;
                usable[li] = std::fabs(area[li]) > 0;
            }
            int outer = -1;
            for (std::size_t li = 0; li < face.loops.size(); ++li)
                if (face.loops[li].outer && usable[li]) outer = static_cast<int>(li);
            if (outer < 0)
                for (std::size_t li = 0; li < face.loops.size(); ++li)
                    if (usable[li] && (outer < 0 || std::fabs(area[li]) > std::fabs(area[static_cast<std::size_t>(outer)]))) outer = static_cast<int>(li);
            // The outer loop counter-clockwise seen from outside, holes the other way.
            const double out = face.sameSense ? 1.0 : -1.0;
            for (std::size_t li = 0; li < face.loops.size(); ++li) {
                if (!usable[li]) continue;
                const double want = static_cast<int>(li) == outer ? out : -out;
                if (area[li] * want < 0) face.loops[li].sense = !face.loops[li].sense;
            }
        }
    tCurves = saved;
}

std::uint64_t meshKey(const MeshData& m) {
    std::uint64_t h = 1469598103934665603ull;
    auto mix = [&](const void* p, std::size_t n) {
        const auto* c = static_cast<const unsigned char*>(p);
        for (std::size_t i = 0; i < n; ++i) h = (h ^ c[i]) * 1099511628211ull;
    };
    const std::size_t nv = m.vertices.size(), ni = m.indices.size();
    mix(&nv, sizeof nv);
    mix(&ni, sizeof ni);
    for (const Vertex& v : m.vertices) mix(&v.position, sizeof(Vec3));
    if (!m.indices.empty()) mix(m.indices.data(), m.indices.size() * sizeof(std::uint32_t));
    return h;
}

Placement placementOf(const Mat4& m) {
    Placement p;
    const double s = std::sqrt(static_cast<double>(m.m[0][0]) * m.m[0][0] + static_cast<double>(m.m[0][1]) * m.m[0][1] + static_cast<double>(m.m[0][2]) * m.m[0][2]);
    p.s = s > 0 ? s : 1.0;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) p.r[i * 3 + j] = m.m[j][i] / p.s; // m[column][row]
    p.t = {m.m[3][0], m.m[3][1], m.m[3][2]};
    return p;
}

// --- native shapes ------------------------------------------------------------------

namespace {

int addVertex(Brep& b, const DVec3& p) {
    b.vertices.push_back({p});
    return static_cast<int>(b.vertices.size()) - 1;
}

int addLineEdge(Brep& b, int v0, int v1) {
    b.edges.push_back({v0, v1, -1, true});
    return static_cast<int>(b.edges.size()) - 1;
}

int addCurve(Brep& b, const BCurve& c) {
    b.curves.push_back(c);
    return static_cast<int>(b.curves.size()) - 1;
}

int addSurface(Brep& b, const BSurface& s) {
    b.surfaces.push_back(s);
    return static_cast<int>(b.surfaces.size()) - 1;
}

BLoop loopOf(std::initializer_list<OrientedEdge> es) {
    BLoop l;
    l.edges = es;
    l.outer = true;
    return l;
}

// A full circle edge (closed on one vertex) in plane frame f.
int circleEdge(Brep& b, const Frame& f, double r, int& vertex) {
    BCurve c;
    c.kind = BCurveKind::Circle;
    c.f = f;
    c.r1 = r;
    const int ci = addCurve(b, c);
    vertex = addVertex(b, f.o + f.x * r);
    b.edges.push_back({vertex, vertex, ci, true});
    return static_cast<int>(b.edges.size()) - 1;
}

} // namespace

std::shared_ptr<Brep> brepBox(const DVec3& lo, const DVec3& hi) {
    auto b = std::make_shared<Brep>();
    int v[8];
    for (int k = 0; k < 8; ++k) v[k] = addVertex(*b, {k & 1 ? hi.x : lo.x, k & 2 ? hi.y : lo.y, k & 4 ? hi.z : lo.z});
    // Edges along x (0-1, 2-3, 4-5, 6-7), y (0-2, 1-3, 4-6, 5-7), z (0-4, 1-5, 2-6, 3-7).
    auto E = [&](int a, int c) { return addLineEdge(*b, v[a], v[c]); };
    const int x0 = E(0, 1), x1 = E(2, 3), x2 = E(4, 5), x3 = E(6, 7);
    const int y0 = E(0, 2), y1 = E(1, 3), y2 = E(4, 6), y3 = E(5, 7);
    const int z0 = E(0, 4), z1 = E(1, 5), z2 = E(2, 6), z3 = E(3, 7);
    BShell sh;
    auto plane = [&](const DVec3& o, const DVec3& n, const DVec3& xr) {
        BSurface s;
        s.kind = BSurfaceKind::Plane;
        s.f = Frame::make(o, n, xr);
        return addSurface(*b, s);
    };
    auto face = [&](int surf, BLoop l) {
        BFace f;
        f.surface = surf;
        f.loops.push_back(std::move(l));
        sh.faces.push_back(std::move(f));
    };
    // Each loop runs counter-clockwise seen from outside.
    face(plane(lo, {0, 0, -1}, {1, 0, 0}), loopOf({{y0, true}, {x1, true}, {y1, false}, {x0, false}}));  // bottom: 0 2 3 1
    face(plane(hi, {0, 0, 1}, {1, 0, 0}), loopOf({{x2, true}, {y3, true}, {x3, false}, {y2, false}}));   // top: 4 5 7 6
    face(plane(lo, {0, -1, 0}, {1, 0, 0}), loopOf({{x0, true}, {z1, true}, {x2, false}, {z0, false}}));  // front: 0 1 5 4
    face(plane(hi, {0, 1, 0}, {1, 0, 0}), loopOf({{z2, true}, {x3, true}, {z3, false}, {x1, false}}));   // back: 2 6 7 3
    face(plane(lo, {-1, 0, 0}, {0, 1, 0}), loopOf({{z0, true}, {y2, true}, {z2, false}, {y0, false}}));  // left: 0 4 6 2
    face(plane(hi, {1, 0, 0}, {0, 1, 0}), loopOf({{y1, true}, {z3, true}, {y3, false}, {z1, false}}));   // right: 1 3 7 5
    sh.name = "box";
    b->shells.push_back(std::move(sh));
    return b;
}

std::shared_ptr<Brep> brepCone(const DVec3& base, double r0, double r1, double h) {
    auto b = std::make_shared<Brep>();
    const Frame f0 = Frame::make(base, {0, 0, 1}, {1, 0, 0});
    const Frame f1 = Frame::make(base + DVec3{0, 0, h}, {0, 0, 1}, {1, 0, 0});
    BShell sh;
    int va = -1, vb = -1;
    const int e0 = r0 > 0 ? circleEdge(*b, f0, r0, va) : -1;
    const int e1 = r1 > 0 ? circleEdge(*b, f1, r1, vb) : -1;
    if (e0 < 0) va = addVertex(*b, base);
    if (e1 < 0) vb = addVertex(*b, base + DVec3{0, 0, h});
    // The seam: a line from the bottom rim to the top rim at u = 0.
    const int seam = addLineEdge(*b, va, vb);
    BSurface side;
    if (std::fabs(r0 - r1) < 1e-12) {
        side.kind = BSurfaceKind::Cylinder;
        side.f = f0;
        side.r1 = r0;
    } else {
        side.kind = BSurfaceKind::Cone;
        side.f = f0;
        side.r1 = r0;
        side.r2 = std::atan((r1 - r0) / h);
    }
    const int ss = addSurface(*b, side);
    {
        BFace f;
        f.surface = ss;
        BLoop l;
        l.outer = true;
        // Bottom rim forwards, up the seam, top rim backwards, down the seam.
        if (e0 >= 0) l.edges.push_back({e0, true});
        l.edges.push_back({seam, true});
        if (e1 >= 0) l.edges.push_back({e1, false});
        l.edges.push_back({seam, false});
        f.loops.push_back(std::move(l));
        sh.faces.push_back(std::move(f));
    }
    auto cap = [&](const Frame& fr, int e, bool top) {
        BSurface s;
        s.kind = BSurfaceKind::Plane;
        s.f = fr;
        BFace f;
        f.surface = addSurface(*b, s);
        f.sameSense = top;
        BLoop l;
        l.outer = true;
        l.edges.push_back({e, top}); // counter-clockwise seen from outside
        f.loops.push_back(std::move(l));
        sh.faces.push_back(std::move(f));
    };
    if (e0 >= 0) cap(f0, e0, false);
    if (e1 >= 0) cap(f1, e1, true);
    sh.name = "cone";
    b->shells.push_back(std::move(sh));
    return b;
}

std::shared_ptr<Brep> brepCylinder(const DVec3& base, double r, double h) { return brepCone(base, r, r, h); }

std::shared_ptr<Brep> brepSphere(const DVec3& c, double r) {
    auto b = std::make_shared<Brep>();
    BSurface s;
    s.kind = BSurfaceKind::Sphere;
    s.f = Frame::make(c, {0, 0, 1}, {1, 0, 0});
    s.r1 = r;
    BFace f;
    f.surface = addSurface(*b, s);
    BLoop l;
    l.vertex = addVertex(*b, c + DVec3{0, 0, r});
    f.loops.push_back(l);
    BShell sh;
    sh.faces.push_back(std::move(f));
    sh.name = "sphere";
    b->shells.push_back(std::move(sh));
    return b;
}

std::shared_ptr<Brep> brepTorus(const DVec3& c, double R, double r) {
    auto b = std::make_shared<Brep>();
    BSurface s;
    s.kind = BSurfaceKind::Torus;
    s.f = Frame::make(c, {0, 0, 1}, {1, 0, 0});
    s.r1 = R;
    s.r2 = r;
    BFace f;
    f.surface = addSurface(*b, s); // no loops: the whole torus
    BShell sh;
    sh.faces.push_back(std::move(f));
    sh.name = "torus";
    b->shells.push_back(std::move(sh));
    return b;
}

// --- revolves -----------------------------------------------------------------------------

namespace {

// The common frame of a revolve: points are A + D a + dir(t) r, with dir(t) turning from
// X (the start of the sweep) towards D x X as t grows.
struct RevolveFrame {
    DVec3 A, D, X, Y;
    DVec3 dir(double t) const { return X * std::cos(t) + Y * std::sin(t); }
    DVec3 at(double a, double r, double t) const { return A + D * a + dir(t) * r; }
};

bool revolveFrame(const DVec3& axisPoint, const DVec3& axis, const DVec3& ref, double sweep, RevolveFrame& rf, double& len) {
    rf.A = axisPoint;
    rf.D = dnormalize(axis);
    const DVec3 P = dnormalize(ref - rf.D * ddot(ref, rf.D));
    if (dlength(rf.D) == 0 || dlength(P) == 0 || !(std::fabs(sweep) > 1e-9) || !std::isfinite(sweep)) return false;
    const DVec3 B = dcross(rf.D, P);
    const bool full = std::fabs(sweep) >= kPi2 - 1e-9;
    len = full ? kPi2 : std::fabs(sweep);
    const double start = !full && sweep < 0 ? sweep : 0.0; // a negative sweep runs from `sweep` up to 0
    rf.X = P * std::cos(start) + B * std::sin(start);
    rf.Y = dcross(rf.D, rf.X);
    return true;
}

} // namespace

std::shared_ptr<Brep> brepRevolve(const std::vector<std::array<double, 2>>& profile, const DVec3& axisPoint, const DVec3& axis, const DVec3& ref,
                                  double sweep) {
    RevolveFrame rf;
    double len = 0;
    if (!revolveFrame(axisPoint, axis, ref, sweep, rf, len)) return nullptr;
    const bool full = len >= kPi2;
    double size = 1;
    for (const auto& q : profile) size = std::max({size, std::fabs(q[0]), std::fabs(q[1])});
    const double tol = 1e-9 * size;
    // Drop repeated and collinear points, and run counter-clockwise in (a, r).
    std::vector<std::array<double, 2>> pr;
    for (const auto& q : profile) {
        if (!(q[1] >= 0) || !std::isfinite(q[0])) return nullptr; // crosses the axis
        if (pr.empty() || std::hypot(q[0] - pr.back()[0], q[1] - pr.back()[1]) > tol) pr.push_back(q);
    }
    while (pr.size() > 1 && std::hypot(pr[0][0] - pr.back()[0], pr[0][1] - pr.back()[1]) <= tol) pr.pop_back();
    for (bool removed = true; removed && pr.size() >= 3;) {
        removed = false;
        for (std::size_t i = 0; i < pr.size() && pr.size() >= 3; ++i) {
            const auto& p0 = pr[(i + pr.size() - 1) % pr.size()];
            const auto& p1 = pr[i];
            const auto& p2 = pr[(i + 1) % pr.size()];
            const double ux = p1[0] - p0[0], uy = p1[1] - p0[1], vx = p2[0] - p1[0], vy = p2[1] - p1[1];
            if (std::fabs(ux * vy - uy * vx) <= tol * (std::hypot(ux, uy) + std::hypot(vx, vy)) && ux * vx + uy * vy > 0) {
                pr.erase(pr.begin() + static_cast<std::ptrdiff_t>(i));
                removed = true;
            }
        }
    }
    if (pr.size() < 3) return nullptr;
    double area = 0;
    for (std::size_t i = 0; i < pr.size(); ++i) {
        const auto& p = pr[i];
        const auto& q = pr[(i + 1) % pr.size()];
        area += p[0] * q[1] - q[0] * p[1];
    }
    if (std::fabs(area) <= tol * size) return nullptr;
    if (area < 0) std::reverse(pr.begin(), pr.end());
    const std::size_t n = pr.size();

    auto b = std::make_shared<Brep>();
    // Vertices at the start and the end of the sweep (one where they coincide).
    std::vector<int> vs(n), ve(n), circle(n, -1);
    for (std::size_t i = 0; i < n; ++i) {
        vs[i] = addVertex(*b, rf.at(pr[i][0], pr[i][1], 0));
        ve[i] = full || pr[i][1] == 0 ? vs[i] : addVertex(*b, rf.at(pr[i][0], pr[i][1], len));
        if (pr[i][1] > 0) { // the circle (or arc) each point turns along, from start to end
            BCurve c;
            c.kind = BCurveKind::Circle;
            c.f = Frame::make(rf.A + rf.D * pr[i][0], rf.D, rf.X);
            c.r1 = pr[i][1];
            b->edges.push_back({vs[i], ve[i], addCurve(*b, c), true});
            circle[i] = static_cast<int>(b->edges.size()) - 1;
        }
    }
    // The profile's segments at the start and the end (one edge where they coincide: on
    // the axis, or the seam of a full turn), made when a face needs them.
    std::vector<int> ls(n, -1), le(n, -1);
    auto startLine = [&](std::size_t i) {
        if (ls[i] < 0) ls[i] = addLineEdge(*b, vs[i], vs[(i + 1) % n]);
        return ls[i];
    };
    auto endLine = [&](std::size_t i) {
        if (full || (pr[i][1] == 0 && pr[(i + 1) % n][1] == 0)) return startLine(i);
        if (le[i] < 0) le[i] = addLineEdge(*b, ve[i], ve[(i + 1) % n]);
        return le[i];
    };
    BShell sh;
    sh.name = "revolve";
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t j = (i + 1) % n;
        const double a0 = pr[i][0], r0 = pr[i][1], a1 = pr[j][0], r1 = pr[j][1];
        if (r0 == 0 && r1 == 0) continue; // on the axis: no face
        const double da = a1 - a0, dr = r1 - r0;
        BSurface s;
        BFace f;
        // The face's outward normal at the start is D dr - X da (the profile runs
        // counter-clockwise), so its sense against each surface's own normal follows.
        const bool flat = std::fabs(da) <= tol;
        if (flat) {
            s.kind = BSurfaceKind::Plane;
            s.f = Frame::make(rf.A + rf.D * a0, rf.D, rf.X);
            f.sameSense = dr > 0;
        } else if (std::fabs(dr) <= tol) {
            s.kind = BSurfaceKind::Cylinder;
            s.f = Frame::make(rf.A + rf.D * a0, rf.D, rf.X);
            s.r1 = r0;
            f.sameSense = da < 0;
        } else { // a cone opening along its axis, from the radius at a0
            const double k = dr / da;
            s.kind = BSurfaceKind::Cone;
            s.f = Frame::make(rf.A + rf.D * a0, k > 0 ? rf.D : -rf.D, rf.X);
            s.r1 = r0;
            s.r2 = std::atan(std::fabs(k));
            f.sameSense = da < 0;
        }
        f.surface = addSurface(*b, s);
        if (flat && full) { // a disc or a ring: just its circles
            if (r1 > 0) {
                BLoop l;
                l.edges.push_back({circle[j], true});
                l.outer = r1 > r0;
                f.loops.push_back(std::move(l));
            }
            if (r0 > 0) {
                BLoop l;
                l.edges.push_back({circle[i], false});
                l.outer = r0 > r1;
                f.loops.push_back(std::move(l));
            }
        } else { // along the start segment, up the far circle, back along the end segment, down
            BLoop l;
            l.outer = true;
            l.edges.push_back({startLine(i), true});
            if (r1 > 0) l.edges.push_back({circle[j], true});
            l.edges.push_back({endLine(i), false});
            if (r0 > 0) l.edges.push_back({circle[i], false});
            f.loops.push_back(std::move(l));
        }
        sh.faces.push_back(std::move(f));
    }
    if (!full) { // the profile itself closes both ends
        for (int end = 0; end < 2; ++end) {
            const double t = end ? len : 0.0;
            const DVec3 out = dcross(rf.D, rf.dir(t)) * (end ? 1.0 : -1.0);
            BSurface s;
            s.kind = BSurfaceKind::Plane;
            s.f = Frame::make(rf.A, out, rf.D);
            BFace f;
            f.surface = addSurface(*b, s);
            BLoop l;
            l.outer = true;
            for (std::size_t k = 0; k < n; ++k) {
                if (end) l.edges.push_back({endLine(k), true});
                else l.edges.push_back({startLine(n - 1 - k), false});
            }
            f.loops.push_back(std::move(l));
            sh.faces.push_back(std::move(f));
        }
    }
    b->shells.push_back(std::move(sh));
    return b;
}

std::shared_ptr<Brep> brepRevolveCircle(double a, double r, double rho, const DVec3& axisPoint, const DVec3& axis, const DVec3& ref, double sweep) {
    RevolveFrame rf;
    double len = 0;
    if (!revolveFrame(axisPoint, axis, ref, sweep, rf, len) || !(rho > 0) || !(r - rho > 1e-9 * std::max(1.0, r))) return nullptr;
    const Frame centre = Frame::make(rf.A + rf.D * a, rf.D, rf.X);
    if (len >= kPi2) {
        auto b = brepTorus({0, 0, 0}, r, rho);
        b->transform(Placement::fromFrames(Frame{}, centre));
        b->shells[0].name = "revolve";
        return b;
    }
    auto b = std::make_shared<Brep>();
    BSurface s;
    s.kind = BSurfaceKind::Torus;
    s.f = centre;
    s.r1 = r;
    s.r2 = rho;
    BShell sh;
    sh.name = "revolve";
    // The profile circle at both ends (counter-clockwise in (a, r)), each closed on its
    // point furthest along the axis, and the arc those points turn along: the seam.
    int ends[2], verts[2];
    for (int e = 0; e < 2; ++e) {
        const double t = e ? len : 0.0;
        ends[e] = circleEdge(*b, Frame::make(rf.at(a, r, t), dcross(rf.D, rf.dir(t)), rf.D), rho, verts[e]);
    }
    BCurve arc;
    arc.kind = BCurveKind::Circle;
    arc.f = Frame::make(rf.A + rf.D * (a + rho), rf.D, rf.X);
    arc.r1 = r;
    b->edges.push_back({verts[0], verts[1], addCurve(*b, arc), true});
    const int seam = static_cast<int>(b->edges.size()) - 1;
    {
        BFace f;
        f.surface = addSurface(*b, s);
        f.loops.push_back(loopOf({{ends[0], true}, {seam, true}, {ends[1], false}, {seam, false}}));
        sh.faces.push_back(std::move(f));
    }
    for (int e = 0; e < 2; ++e) {
        const double t = e ? len : 0.0;
        BSurface cap;
        cap.kind = BSurfaceKind::Plane;
        cap.f = Frame::make(rf.at(a, r, t), dcross(rf.D, rf.dir(t)) * (e ? 1.0 : -1.0), rf.D);
        BFace f;
        f.surface = addSurface(*b, cap);
        f.loops.push_back(loopOf({{ends[e], e == 1}}));
        sh.faces.push_back(std::move(f));
    }
    b->shells.push_back(std::move(sh));
    return b;
}

} // namespace sicdm

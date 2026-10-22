#include "core/Constraints.hpp"

#include "core/Json.hpp"
#include "core/Sketch.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

namespace sicdm {

namespace {

constexpr double kPiC = 3.14159265358979323846;

struct Key {
    ConstraintKind kind;
    const char* key;
    bool value;
};
constexpr Key kKeys[] = {
    {ConstraintKind::Coincident, "coincident", false}, {ConstraintKind::Horizontal, "horizontal", false},
    {ConstraintKind::Vertical, "vertical", false},     {ConstraintKind::Parallel, "parallel", false},
    {ConstraintKind::Perpendicular, "perpendicular", false}, {ConstraintKind::Tangent, "tangent", false},
    {ConstraintKind::Equal, "equal", false},           {ConstraintKind::Concentric, "concentric", false},
    {ConstraintKind::PointOn, "on", false},            {ConstraintKind::Midpoint, "midpoint", false},
    {ConstraintKind::Fix, "fix", false},               {ConstraintKind::Distance, "distance", true},
    {ConstraintKind::HDistance, "hdistance", true},    {ConstraintKind::VDistance, "vdistance", true},
    {ConstraintKind::Radius, "radius", true},          {ConstraintKind::Diameter, "diameter", true},
    {ConstraintKind::Angle, "angle", true}};

struct V2 {
    double x = 0, y = 0;
};
V2 operator-(V2 a, V2 b) { return {a.x - b.x, a.y - b.y}; }
double cross(V2 a, V2 b) { return a.x * b.y - a.y * b.x; }
double dot(V2 a, V2 b) { return a.x * b.x + a.y * b.y; }
double len(V2 a) { return std::hypot(a.x, a.y); }

// The curves' parameters, laid out in one vector.
//   Point (x, y); Line (x0 y0 x1 y1); Polyline (x y per vertex); Rectangle (x0 y0 x1 y1);
//   Circle (cx cy r)
struct Params {
    std::vector<double> x;
    std::map<std::string, std::pair<int, int>> at; // curve -> (index of the curve in the sketch, first parameter)
};

// A closed polyline repeats its first point at the end: that point is the first one.
bool closedPolyline(const SketchCurve& c) {
    return c.kind == CurveKind::Polyline && c.pts.size() > 3 && std::hypot(c.pts.front().x - c.pts.back().x, c.pts.front().y - c.pts.back().y) < 1e-6;
}

int paramCount(const SketchCurve& c) {
    switch (c.kind) {
    case CurveKind::Point: return 2;
    case CurveKind::Line: return 4;
    case CurveKind::Polyline: return static_cast<int>(c.pts.size() - (closedPolyline(c) ? 1 : 0)) * 2;
    case CurveKind::Rectangle: return 4;
    case CurveKind::Circle: return 3;
    }
    return 0;
}

int findCurve(const Sketch& s, const std::string& name) {
    for (std::size_t i = 0; i < s.curves().size(); ++i)
        if (s.curves()[i].name == name) return static_cast<int>(i);
    return -1;
}

// Reads geometry from a parameter vector.
class View {
public:
    View(const Sketch& s, const Params& p, const std::vector<double>& x) : m_s(s), m_p(p), m_x(x) {}

    bool valid(const SketchRef& r) const { return m_p.at.count(r.curve) > 0; }
    const SketchCurve& curve(const SketchRef& r) const { return m_s.curves()[static_cast<std::size_t>(m_p.at.at(r.curve).first)]; }
    double v(const SketchRef& r, int k) const { return m_x[static_cast<std::size_t>(m_p.at.at(r.curve).second + k)]; }

    bool point(const SketchRef& r, V2* out) const {
        if (!valid(r)) return false;
        const SketchCurve& c = curve(r);
        const int k = r.point;
        switch (c.kind) {
        case CurveKind::Point:
            *out = {v(r, 0), v(r, 1)};
            return true;
        case CurveKind::Line:
            if (k != 0 && k != 1) return false;
            *out = {v(r, 2 * k), v(r, 2 * k + 1)};
            return true;
        case CurveKind::Polyline: {
            if (k < 0 || k >= static_cast<int>(c.pts.size())) return false;
            const int kk = closedPolyline(c) && k == static_cast<int>(c.pts.size()) - 1 ? 0 : k;
            *out = {v(r, 2 * kk), v(r, 2 * kk + 1)};
            return true;
        }
        case CurveKind::Rectangle: {
            if (k < 0 || k > 3) return false;
            const double x0 = v(r, 0), y0 = v(r, 1), x1 = v(r, 2), y1 = v(r, 3);
            *out = k == 0 ? V2{x0, y0} : k == 1 ? V2{x1, y0} : k == 2 ? V2{x1, y1} : V2{x0, y1};
            return true;
        }
        case CurveKind::Circle:
            if (k != 0) return false;
            *out = {v(r, 0), v(r, 1)};
            return true;
        }
        return false;
    }
    bool line(const SketchRef& r, V2* a, V2* b) const {
        if (!valid(r)) return false;
        const SketchCurve& c = curve(r);
        if (c.kind == CurveKind::Line && r.segment < 0 && r.point < 0) {
            *a = {v(r, 0), v(r, 1)}, *b = {v(r, 2), v(r, 3)};
            return true;
        }
        if (r.segment < 0) return false;
        const int n = c.kind == CurveKind::Rectangle ? 4 : c.kind == CurveKind::Polyline ? static_cast<int>(c.pts.size()) : 0;
        if (n == 0 || r.segment >= (c.kind == CurveKind::Rectangle ? 4 : n - 1)) return false;
        SketchRef p0 = r, p1 = r;
        p0.segment = p1.segment = -1;
        p0.point = r.segment;
        p1.point = (r.segment + 1) % n;
        return point(p0, a) && point(p1, b);
    }
    bool circle(const SketchRef& r, V2* c, double* rad) const {
        if (!valid(r) || curve(r).kind != CurveKind::Circle || r.point >= 0) return false;
        *c = {v(r, 0), v(r, 1)};
        *rad = v(r, 2);
        return true;
    }

private:
    const Sketch& m_s;
    const Params& m_p;
    const std::vector<double>& m_x;
};

double wrapAngle(double a) {
    while (a > kPiC) a -= 2 * kPiC;
    while (a < -kPiC) a += 2 * kPiC;
    return a;
}

// The residuals of one constraint (empty when its geometry doesn't fit it). Angles are
// scaled by 10 mm so they weigh about like lengths.
std::vector<double> residuals(const View& g, const SketchConstraint& c) {
    std::vector<double> r;
    const auto& f = c.refs;
    V2 p, q, a, b, a2, b2, c1, c2;
    double r1 = 0, r2 = 0;
    auto isLine = [&](std::size_t i, V2* x, V2* y) { return i < f.size() && g.line(f[i], x, y); };
    auto isPoint = [&](std::size_t i, V2* x) { return i < f.size() && g.point(f[i], x); };
    auto isCircle = [&](std::size_t i, V2* x, double* rr) { return i < f.size() && g.circle(f[i], x, rr); };
    switch (c.kind) {
    case ConstraintKind::Coincident:
        if (isPoint(0, &p) && isPoint(1, &q)) r = {q.x - p.x, q.y - p.y};
        break;
    case ConstraintKind::Horizontal:
        if (f.size() == 1 && isLine(0, &a, &b)) r = {b.y - a.y};
        else if (isPoint(0, &p) && isPoint(1, &q)) r = {q.y - p.y};
        break;
    case ConstraintKind::Vertical:
        if (f.size() == 1 && isLine(0, &a, &b)) r = {b.x - a.x};
        else if (isPoint(0, &p) && isPoint(1, &q)) r = {q.x - p.x};
        break;
    case ConstraintKind::Parallel:
    case ConstraintKind::Perpendicular:
        if (isLine(0, &a, &b) && isLine(1, &a2, &b2)) {
            const V2 d1 = b - a, d2 = b2 - a2;
            const double l = std::max(len(d1) * len(d2), 1e-12);
            r = {10.0 * (c.kind == ConstraintKind::Parallel ? cross(d1, d2) : dot(d1, d2)) / l};
        }
        break;
    case ConstraintKind::Tangent:
        if (isLine(0, &a, &b) && isCircle(1, &c1, &r1)) {
            const V2 d = b - a;
            r = {std::fabs(cross(d, c1 - a)) / std::max(len(d), 1e-12) - r1};
        } else if (isCircle(0, &c1, &r1) && isLine(1, &a, &b)) {
            const V2 d = b - a;
            r = {std::fabs(cross(d, c1 - a)) / std::max(len(d), 1e-12) - r1};
        } else if (isCircle(0, &c1, &r1) && isCircle(1, &c2, &r2)) {
            r = {c.value >= 0 ? len(c2 - c1) - (r1 + r2) : len(c2 - c1) - std::fabs(r1 - r2)};
        }
        break;
    case ConstraintKind::Equal:
        if (isLine(0, &a, &b) && isLine(1, &a2, &b2)) r = {len(b - a) - len(b2 - a2)};
        else if (isCircle(0, &c1, &r1) && isCircle(1, &c2, &r2)) r = {r1 - r2};
        break;
    case ConstraintKind::Concentric:
        if (isCircle(0, &c1, &r1) && isCircle(1, &c2, &r2)) r = {c2.x - c1.x, c2.y - c1.y};
        break;
    case ConstraintKind::PointOn:
        if (isPoint(0, &p)) {
            if (isLine(1, &a, &b)) r = {cross(b - a, p - a) / std::max(len(b - a), 1e-12)};
            else if (isCircle(1, &c1, &r1)) r = {len(p - c1) - r1};
        }
        break;
    case ConstraintKind::Midpoint:
        if (isPoint(0, &p) && isLine(1, &a, &b)) r = {p.x - (a.x + b.x) * 0.5, p.y - (a.y + b.y) * 0.5};
        break;
    case ConstraintKind::Fix:
        if (isPoint(0, &p)) r = {p.x - c.value, p.y - c.value2};
        else if (isCircle(0, &c1, &r1)) r = {c1.x - c.value, c1.y - c.value2};
        break;
    case ConstraintKind::Distance:
        if (f.size() == 1 && isLine(0, &a, &b)) r = {len(b - a) - c.value};
        else if (isPoint(0, &p) && isPoint(1, &q)) r = {len(q - p) - c.value};
        else if (isPoint(0, &p) && isLine(1, &a, &b)) r = {std::fabs(cross(b - a, p - a)) / std::max(len(b - a), 1e-12) - c.value};
        else if (isCircle(0, &c1, &r1) && isCircle(1, &c2, &r2)) r = {len(c2 - c1) - c.value};
        break;
    case ConstraintKind::HDistance:
        if (f.size() == 1 && isLine(0, &a, &b)) r = {std::fabs(b.x - a.x) - c.value};
        else if (isPoint(0, &p) && isPoint(1, &q)) r = {std::fabs(q.x - p.x) - c.value};
        break;
    case ConstraintKind::VDistance:
        if (f.size() == 1 && isLine(0, &a, &b)) r = {std::fabs(b.y - a.y) - c.value};
        else if (isPoint(0, &p) && isPoint(1, &q)) r = {std::fabs(q.y - p.y) - c.value};
        break;
    case ConstraintKind::Radius:
        if (isCircle(0, &c1, &r1)) r = {r1 - c.value};
        break;
    case ConstraintKind::Diameter:
        if (isCircle(0, &c1, &r1)) r = {2 * r1 - c.value};
        break;
    case ConstraintKind::Angle:
        if (isLine(0, &a, &b) && isLine(1, &a2, &b2)) {
            const V2 d1 = b - a, d2 = b2 - a2;
            // The angle between the lines (0..180), whichever way they are drawn.
            double ang = std::fabs(wrapAngle(std::atan2(cross(d1, d2), dot(d1, d2))));
            const double want = c.value * kPiC / 180.0;
            if (std::fabs(ang - want) > std::fabs((kPiC - ang) - want)) ang = kPiC - ang;
            r = {10.0 * (ang - want)};
        }
        break;
    }
    return r;
}

Params gather(const Sketch& s) {
    Params p;
    for (const SketchConstraint& c : s.constraints())
        for (const SketchRef& r : c.refs) {
            if (p.at.count(r.curve)) continue;
            const int ci = findCurve(s, r.curve);
            if (ci < 0) continue;
            const SketchCurve& cv = s.curves()[static_cast<std::size_t>(ci)];
            p.at[r.curve] = {ci, static_cast<int>(p.x.size())};
            if (cv.kind == CurveKind::Rectangle) {
                p.x.insert(p.x.end(), {cv.pts[0].x, cv.pts[0].y, cv.pts[1].x, cv.pts[1].y});
            } else if (cv.kind == CurveKind::Circle) {
                p.x.insert(p.x.end(), {cv.pts[0].x, cv.pts[0].y, cv.radius});
            } else {
                for (int k = 0; k < paramCount(cv) / 2; ++k) p.x.insert(p.x.end(), {cv.pts[static_cast<std::size_t>(k)].x, cv.pts[static_cast<std::size_t>(k)].y});
            }
        }
    return p;
}

void scatter(Sketch& s, const Params& p, const std::vector<double>& x) {
    for (const auto& [name, at] : p.at) {
        SketchCurve& c = s.curves()[static_cast<std::size_t>(at.first)];
        const auto xi = [&](int k) { return static_cast<float>(x[static_cast<std::size_t>(at.second + k)]); };
        if (c.kind == CurveKind::Rectangle) {
            c.pts[0] = {xi(0), xi(1)}, c.pts[1] = {xi(2), xi(3)};
        } else if (c.kind == CurveKind::Circle) {
            c.pts[0] = {xi(0), xi(1)};
            c.radius = std::max(xi(2), 1e-4f);
        } else {
            const bool closed = closedPolyline(c);
            for (int k = 0; k < paramCount(c) / 2; ++k) c.pts[static_cast<std::size_t>(k)] = {xi(2 * k), xi(2 * k + 1)};
            if (closed) c.pts.back() = c.pts.front(); // still closed
        }
    }
    s.touch();
}

std::vector<double> allResiduals(const Sketch& s, const Params& p, const std::vector<double>& x) {
    const View g(s, p, x);
    std::vector<double> r;
    for (const SketchConstraint& c : s.constraints()) {
        const auto rc = residuals(g, c);
        r.insert(r.end(), rc.begin(), rc.end());
    }
    return r;
}

double norm2(const std::vector<double>& v) {
    double s = 0;
    for (double e : v) s += e * e;
    return s;
}

// Solves A x = b (A symmetric positive definite) by Cholesky; false if singular.
bool cholesky(std::vector<double> A, std::vector<double> b, int n, std::vector<double>& x) {
    auto at = [&](int i, int j) -> double& { return A[static_cast<std::size_t>(i * n + j)]; };
    for (int j = 0; j < n; ++j) {
        double d = at(j, j);
        for (int k = 0; k < j; ++k) d -= at(j, k) * at(j, k);
        if (!(d > 1e-300)) return false;
        at(j, j) = std::sqrt(d);
        for (int i = j + 1; i < n; ++i) {
            double s = at(i, j);
            for (int k = 0; k < j; ++k) s -= at(i, k) * at(j, k);
            at(i, j) = s / at(j, j);
        }
    }
    x.assign(static_cast<std::size_t>(n), 0.0);
    for (int i = 0; i < n; ++i) {
        double s = b[static_cast<std::size_t>(i)];
        for (int k = 0; k < i; ++k) s -= at(i, k) * x[static_cast<std::size_t>(k)];
        x[static_cast<std::size_t>(i)] = s / at(i, i);
    }
    for (int i = n - 1; i >= 0; --i) {
        double s = x[static_cast<std::size_t>(i)];
        for (int k = i + 1; k < n; ++k) s -= at(k, i) * x[static_cast<std::size_t>(k)];
        x[static_cast<std::size_t>(i)] = s / at(i, i);
    }
    return true;
}

std::vector<double> jacobian(const Sketch& s, const Params& p, const std::vector<double>& x, std::size_t m) {
    const std::size_t n = x.size();
    std::vector<double> J(m * n, 0.0);
    std::vector<double> xp = x;
    for (std::size_t j = 0; j < n; ++j) {
        const double h = 1e-7 * std::max(1.0, std::fabs(x[j]));
        xp[j] = x[j] + h;
        const auto rp = allResiduals(s, p, xp);
        xp[j] = x[j] - h;
        const auto rm = allResiduals(s, p, xp);
        xp[j] = x[j];
        for (std::size_t i = 0; i < m && i < rp.size() && i < rm.size(); ++i) J[i * n + j] = (rp[i] - rm[i]) / (2 * h);
    }
    return J;
}

// Rank by Gaussian elimination with partial pivoting.
int rankOf(std::vector<double> J, std::size_t m, std::size_t n) {
    int rank = 0;
    double scale = 0;
    for (double v : J) scale = std::max(scale, std::fabs(v));
    const double tol = std::max(scale, 1.0) * 1e-7;
    std::vector<char> used(m, 0);
    for (std::size_t col = 0; col < n; ++col) {
        std::size_t best = m;
        double bv = tol;
        for (std::size_t r = 0; r < m; ++r)
            if (!used[r] && std::fabs(J[r * n + col]) > bv) bv = std::fabs(J[r * n + col]), best = r;
        if (best == m) continue;
        used[best] = 1;
        ++rank;
        for (std::size_t r = 0; r < m; ++r) {
            if (r == best) continue;
            const double f = J[r * n + col] / J[best * n + col];
            if (f == 0) continue;
            for (std::size_t c = col; c < n; ++c) J[r * n + c] -= f * J[best * n + c];
        }
    }
    return rank;
}

} // namespace

const char* constraintKey(ConstraintKind k) {
    for (const Key& e : kKeys)
        if (e.kind == k) return e.key;
    return "coincident";
}

bool constraintFromKey(const std::string& key, ConstraintKind* out) {
    for (const Key& e : kKeys)
        if (key == e.key) return *out = e.kind, true;
    if (key == "pointon" || key == "onto") return *out = ConstraintKind::PointOn, true;
    if (key == "fixed" || key == "lock") return *out = ConstraintKind::Fix, true;
    if (key == "dist" || key == "length") return *out = ConstraintKind::Distance, true;
    if (key == "perp") return *out = ConstraintKind::Perpendicular, true;
    if (key == "coincide" || key == "join") return *out = ConstraintKind::Coincident, true;
    return false;
}

bool constraintHasValue(ConstraintKind k) {
    for (const Key& e : kKeys)
        if (e.kind == k) return e.value;
    return false;
}

std::string SketchRef::describe() const {
    std::string s = curve;
    if (segment >= 0) s += " side " + std::to_string(segment + 1);
    else if (point >= 0) s += " point " + std::to_string(point + 1);
    return s;
}

std::string SketchConstraint::describe() const {
    std::string s = name + "  " + constraintKey(kind);
    for (const SketchRef& r : refs) s += "  " + r.describe();
    if (constraintHasValue(kind)) {
        char b[48];
        std::snprintf(b, sizeof(b), kind == ConstraintKind::Angle ? "  = %g deg" : "  = %g mm", value);
        s += b;
    }
    return s;
}

double constraintError(const Sketch& sketch, const SketchConstraint& c) {
    Params p;
    for (const SketchRef& r : c.refs) {
        if (p.at.count(r.curve)) continue;
        const int ci = findCurve(sketch, r.curve);
        if (ci < 0) return std::numeric_limits<double>::quiet_NaN();
        const SketchCurve& cv = sketch.curves()[static_cast<std::size_t>(ci)];
        p.at[r.curve] = {ci, static_cast<int>(p.x.size())};
        if (cv.kind == CurveKind::Rectangle) p.x.insert(p.x.end(), {cv.pts[0].x, cv.pts[0].y, cv.pts[1].x, cv.pts[1].y});
        else if (cv.kind == CurveKind::Circle) p.x.insert(p.x.end(), {cv.pts[0].x, cv.pts[0].y, cv.radius});
        else
            for (int k = 0; k < paramCount(cv) / 2; ++k) p.x.insert(p.x.end(), {cv.pts[static_cast<std::size_t>(k)].x, cv.pts[static_cast<std::size_t>(k)].y});
    }
    const View g(sketch, p, p.x);
    const auto r = residuals(g, c);
    if (r.empty()) return std::numeric_limits<double>::quiet_NaN();
    double e = 0;
    for (double v : r) e = std::max(e, std::fabs(v));
    return e;
}

double currentValue(const Sketch& sketch, const SketchConstraint& c) {
    SketchConstraint probe = c;
    probe.value = 0;
    // A dimension's residual is (measured - value) for every kind it has, so at value
    // 0 the residual is the measured value (angles carry their 10 mm weight).
    const double e = constraintError(sketch, probe);
    if (std::isnan(e)) return e;
    return c.kind == ConstraintKind::Angle ? e / 10.0 * 180.0 / kPiC : e;
}

SolveReport solveSketch(Sketch& sketch) {
    SolveReport rep;
    // Constraints whose curves are gone are dropped.
    std::erase_if(sketch.constraints(), [&](const SketchConstraint& c) {
        return std::any_of(c.refs.begin(), c.refs.end(), [&](const SketchRef& r) { return findCurve(sketch, r.curve) < 0; });
    });
    Params p = gather(sketch);
    rep.parameters = static_cast<int>(p.x.size());
    if (p.x.empty()) {
        rep.ok = true;
        return rep;
    }
    const std::vector<double> start = p.x;
    std::vector<double> x = p.x;
    std::vector<double> r = allResiduals(sketch, p, x);
    const std::size_t n = x.size(), m = r.size();
    double lambda = 1e-3;
    double f = norm2(r);
    for (int it = 0; it < 200 && f > 1e-22; ++it) {
        rep.iterations = it + 1;
        const auto J = jacobian(sketch, p, x, m);
        // (J^T J + lambda I) dx = -J^T r: the smallest step that lowers the error.
        std::vector<double> A(n * n, 0.0), g(n, 0.0);
        for (std::size_t i = 0; i < m; ++i)
            for (std::size_t a = 0; a < n; ++a) {
                const double ja = J[i * n + a];
                if (ja == 0) continue;
                g[a] -= ja * r[i];
                for (std::size_t b = 0; b < n; ++b) A[a * n + b] += ja * J[i * n + b];
            }
        bool improved = false;
        for (int tries = 0; tries < 12 && !improved; ++tries) {
            std::vector<double> Ad = A;
            for (std::size_t a = 0; a < n; ++a) Ad[a * n + a] += lambda;
            std::vector<double> dx;
            if (!cholesky(Ad, g, static_cast<int>(n), dx)) {
                lambda *= 10;
                continue;
            }
            std::vector<double> xn = x;
            for (std::size_t a = 0; a < n; ++a) xn[a] += dx[a];
            const auto rn = allResiduals(sketch, p, xn);
            const double fn = norm2(rn);
            if (fn < f) {
                x = std::move(xn), r = rn, f = fn;
                lambda = std::max(lambda / 4, 1e-12);
                improved = true;
            } else
                lambda *= 4;
        }
        if (!improved) break;
    }
    for (double e : r) rep.residual = std::max(rep.residual, std::fabs(e));
    if (rep.residual > 1e-6) { // can't all be met: name the worst one, and change nothing
        double worst = 0;
        std::size_t k = 0;
        const View g(sketch, p, x);
        for (const SketchConstraint& c : sketch.constraints()) {
            double e = 0;
            for (double v : residuals(g, c)) e = std::max(e, std::fabs(v));
            if (e > worst) worst = e, rep.conflict = c.name, (void)k;
            ++k;
        }
        rep.ok = false;
        return rep;
    }
    scatter(sketch, p, x);
    const auto J = jacobian(sketch, p, x, m);
    const int rank = rankOf(J, m, n);
    rep.dof = static_cast<int>(n) - rank;
    rep.redundant = static_cast<int>(m) - rank;
    rep.ok = true;
    (void)start;
    return rep;
}

Json constraintsToJson(const Sketch& sketch) {
    Json a = Json::array();
    for (const SketchConstraint& c : sketch.constraints()) {
        Json o = Json::object();
        o.set("kind", constraintKey(c.kind));
        o.set("name", c.name);
        Json refs = Json::array();
        for (const SketchRef& r : c.refs) {
            Json jr = Json::object();
            jr.set("curve", r.curve);
            if (r.point >= 0) jr.set("point", r.point);
            if (r.segment >= 0) jr.set("segment", r.segment);
            refs.push(jr);
        }
        o.set("refs", refs);
        o.set("value", c.value);
        o.set("value2", c.value2);
        if (c.step >= 0) o.set("step", c.step);
        a.push(o);
    }
    return a;
}

void constraintsFromJson(const Json& j, Sketch& sketch) {
    for (const Json& o : j.items()) {
        SketchConstraint c;
        if (!constraintFromKey(o["kind"].asString(""), &c.kind)) continue;
        c.name = o["name"].asString("");
        for (const Json& jr : o["refs"].items()) c.refs.push_back({jr["curve"].asString(""), jr["point"].asInt(-1), jr["segment"].asInt(-1)});
        c.value = o["value"].asNumber(0);
        c.value2 = o["value2"].asNumber(0);
        c.step = o["step"].asInt(-1);
        if (c.name.empty()) c.name = sketch.nextConstraintName(constraintHasValue(c.kind));
        else sketch.noteConstraintName(c.name); // new names continue past the loaded ones
        sketch.constraints().push_back(std::move(c));
    }
}

} // namespace sicdm

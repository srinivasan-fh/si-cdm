#include "core/Constraints.hpp"
#include "core/SketchEdit.hpp"

#include "core/Json.hpp"
#include "core/Sketch.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <functional>
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
//   Point (x, y); Line (x0 y0 x1 y1); Polyline, Spline (x y per point); Rectangle (x0 y0
//   x1 y1); Circle (cx cy r); Arc (cx cy r a0 a1: the start and end angles); Ellipse
//   (cx cy, the major axis end x y, minor radius); Slot (x0 y0 x1 y1 r)
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
    case CurveKind::Spline: return static_cast<int>(c.pts.size()) * 2;
    case CurveKind::Rectangle: return 4;
    case CurveKind::Circle: return 3;
    case CurveKind::Arc: return 5;
    case CurveKind::Ellipse: return 5;
    case CurveKind::Slot: return 5;
    }
    return 0;
}

// A curve's parameters onto the end of x.
void appendParams(const SketchCurve& cv, std::vector<double>& x) {
    switch (cv.kind) {
    case CurveKind::Rectangle: x.insert(x.end(), {cv.pts[0].x, cv.pts[0].y, cv.pts[1].x, cv.pts[1].y}); return;
    case CurveKind::Circle: x.insert(x.end(), {cv.pts[0].x, cv.pts[0].y, cv.radius}); return;
    case CurveKind::Arc: {
        float a0, a1;
        cv.arcAngles(a0, a1);
        x.insert(x.end(), {cv.pts[0].x, cv.pts[0].y, std::hypot(cv.pts[1].x - cv.pts[0].x, cv.pts[1].y - cv.pts[0].y), a0, a1});
        return;
    }
    case CurveKind::Ellipse: x.insert(x.end(), {cv.pts[0].x, cv.pts[0].y, cv.pts[1].x, cv.pts[1].y, cv.radius}); return;
    case CurveKind::Slot: x.insert(x.end(), {cv.pts[0].x, cv.pts[0].y, cv.pts[1].x, cv.pts[1].y, cv.radius}); return;
    default:
        for (int k = 0; k < paramCount(cv) / 2; ++k) x.insert(x.end(), {cv.pts[static_cast<std::size_t>(k)].x, cv.pts[static_cast<std::size_t>(k)].y});
    }
}

// Parameters back into the curve.
void writeParams(SketchCurve& c, const double* x) {
    const auto f = [&](int k) { return static_cast<float>(x[k]); };
    switch (c.kind) {
    case CurveKind::Rectangle: c.pts[0] = {f(0), f(1)}, c.pts[1] = {f(2), f(3)}; return;
    case CurveKind::Circle:
        c.pts[0] = {f(0), f(1)};
        c.radius = std::max(f(2), 1e-4f);
        return;
    case CurveKind::Arc: {
        const double r = std::max(x[2], 1e-4);
        c.pts[0] = {f(0), f(1)};
        c.pts[1] = {static_cast<float>(x[0] + r * std::cos(x[3])), static_cast<float>(x[1] + r * std::sin(x[3]))};
        c.pts[2] = {static_cast<float>(x[0] + r * std::cos(x[4])), static_cast<float>(x[1] + r * std::sin(x[4]))};
        c.radius = static_cast<float>(r);
        return;
    }
    case CurveKind::Ellipse:
    case CurveKind::Slot:
        c.pts[0] = {f(0), f(1)}, c.pts[1] = {f(2), f(3)};
        c.radius = std::max(f(4), 1e-4f);
        return;
    default: {
        const bool closed = closedPolyline(c);
        const int count = paramCount(c) / 2; // before the points move (moving the first opens the loop)
        for (int k = 0; k < count; ++k) c.pts[static_cast<std::size_t>(k)] = {f(2 * k), f(2 * k + 1)};
        if (closed) c.pts.back() = c.pts.front(); // still closed
    }
    }
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
        case CurveKind::Arc: // 0 centre, 1 start, 2 end
            if (k == 0) *out = {v(r, 0), v(r, 1)};
            else if (k == 1 || k == 2) *out = {v(r, 0) + v(r, 2) * std::cos(v(r, 2 + k)), v(r, 1) + v(r, 2) * std::sin(v(r, 2 + k))};
            else return false;
            return true;
        case CurveKind::Spline:
            if (k < 0 || k >= static_cast<int>(c.pts.size())) return false;
            *out = {v(r, 2 * k), v(r, 2 * k + 1)};
            return true;
        case CurveKind::Ellipse: // 0 centre, 1 the major axis end
        case CurveKind::Slot:    // 0, 1 the centres of the ends
            if (k != 0 && k != 1) return false;
            *out = {v(r, 2 * k), v(r, 2 * k + 1)};
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
        if (!valid(r) || (curve(r).kind != CurveKind::Circle && curve(r).kind != CurveKind::Arc) || r.point >= 0) return false;
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
            appendParams(cv, p.x);
        }
    return p;
}

void scatter(Sketch& s, const Params& p, const std::vector<double>& x) {
    for (const auto& [name, at] : p.at) {
        SketchCurve& c = s.curves()[static_cast<std::size_t>(at.first)];
        writeParams(c, &x[static_cast<std::size_t>(at.second)]);
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

// Residual blocks, each reading a few of the parameters, so the Jacobian is built
// block by block, perturbing only the block's own parameters.
struct Block {
    std::vector<int> vars;
    std::function<void(const std::vector<double>&, std::vector<double>&)> eval;
};

// Newton's method for blocks(x) = 0 taking the smallest weighted steps: each step is
// dx = -W^-1 J^T (J W^-1 J^T)^-1 r, so a parameter with a small inverse weight (or 0)
// moves little (or not at all). Returns the largest residual left.
double minimumChangeSolve(const std::vector<Block>& blocks, const std::vector<double>& winv, std::vector<double>& x, int maxIterations) {
    const std::size_t n = x.size();
    auto residualsAt = [&](const std::vector<double>& xv) {
        std::vector<double> r;
        for (const Block& b : blocks) b.eval(xv, r);
        return r;
    };
    auto worst = [](const std::vector<double>& r) {
        double w = 0;
        for (double e : r) w = std::max(w, std::fabs(e));
        return w;
    };
    std::vector<double> r = residualsAt(x), xp = x, rp, rm;
    for (int it = 0; it < maxIterations && worst(r) > 1e-11; ++it) {
        const std::size_t m = r.size();
        std::vector<double> J(m * n, 0.0);
        std::size_t row = 0;
        for (const Block& b : blocks) {
            rp.clear();
            b.eval(x, rp);
            const std::size_t rows = rp.size();
            for (const int jv : b.vars) {
                const std::size_t j = static_cast<std::size_t>(jv);
                if (winv[j] == 0) continue;
                const double h = 1e-7 * std::max(1.0, std::fabs(x[j]));
                xp[j] = x[j] + h;
                rp.clear();
                b.eval(xp, rp);
                xp[j] = x[j] - h;
                rm.clear();
                b.eval(xp, rm);
                xp[j] = x[j];
                for (std::size_t i = 0; i < rows && i < rp.size() && i < rm.size(); ++i) J[(row + i) * n + j] = (rp[i] - rm[i]) / (2 * h);
            }
            row += rows;
        }
        // M = J W^-1 J^T, a little regularised so repeated constraints don't stall it.
        std::vector<double> M(m * m, 0.0);
        double trace = 0;
        for (std::size_t a = 0; a < m; ++a)
            for (std::size_t c = a; c < m; ++c) {
                double v = 0;
                for (std::size_t j = 0; j < n; ++j) v += J[a * n + j] * winv[j] * J[c * n + j];
                M[a * m + c] = M[c * m + a] = v;
                if (a == c) trace += v;
            }
        std::vector<double> y;
        double mu = 1e-12 * std::max(trace / static_cast<double>(std::max<std::size_t>(m, 1)), 1e-12);
        bool solved = false;
        for (int tries = 0; tries < 8 && !solved; ++tries, mu *= 100) {
            std::vector<double> Md = M;
            for (std::size_t a = 0; a < m; ++a) Md[a * m + a] += mu;
            solved = cholesky(Md, r, static_cast<int>(m), y);
        }
        if (!solved) break;
        std::vector<double> dx(n, 0.0);
        for (std::size_t j = 0; j < n; ++j) {
            if (winv[j] == 0) continue;
            double v = 0;
            for (std::size_t a = 0; a < m; ++a) v += J[a * n + j] * y[a];
            dx[j] = -winv[j] * v;
        }
        // Halve the step until it helps.
        const double f = norm2(r);
        bool improved = false;
        for (double alpha = 1; alpha > 1e-4 && !improved; alpha *= 0.5) {
            std::vector<double> xn = x;
            for (std::size_t j = 0; j < n; ++j) xn[j] += alpha * dx[j];
            auto rn = residualsAt(xn);
            if (norm2(rn) < f) x = std::move(xn), r = std::move(rn), improved = true;
        }
        xp = x;
        if (!improved) break;
    }
    return worst(r);
}

// Which of a curve's parameters are positions (pairs that move with it) and which are
// sizes (radii, angles) that a rigid move keeps.
std::vector<char> positional(const SketchCurve& c) {
    std::vector<char> pos(static_cast<std::size_t>(paramCount(c)), 1);
    switch (c.kind) {
    case CurveKind::Circle: pos[2] = 0; break;
    case CurveKind::Arc: pos[2] = pos[3] = pos[4] = 0; break;
    case CurveKind::Ellipse:
    case CurveKind::Slot: pos[4] = 0; break;
    default: break;
    }
    return pos;
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
        appendParams(cv, p.x);
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

SolveReport dragSketch(Sketch& sketch, const SketchGrip& grip, Vec2 from, Vec2 to) {
    SolveReport rep;
    const int gi = findCurve(sketch, grip.ref.curve);
    if (gi < 0) return rep;
    const SketchCurve& gc = sketch.curves()[static_cast<std::size_t>(gi)];
    if (grip.rim && gc.kind != CurveKind::Circle && gc.kind != CurveKind::Arc) return rep;
    // The curves the drag reaches: the grip's, and every curve joined to it through
    // constraints. The constraints among them are the ones to keep.
    std::vector<std::string> reach{gc.name};
    std::vector<const SketchConstraint*> keep;
    std::vector<char> taken(sketch.constraints().size(), 0);
    for (bool grew = true; grew;) {
        grew = false;
        for (std::size_t k = 0; k < sketch.constraints().size(); ++k) {
            if (taken[k]) continue;
            const SketchConstraint& c = sketch.constraints()[k];
            const bool touches = std::any_of(c.refs.begin(), c.refs.end(), [&](const SketchRef& r) {
                return std::find(reach.begin(), reach.end(), r.curve) != reach.end();
            });
            if (!touches) continue;
            taken[k] = 1;
            keep.push_back(&c);
            grew = true;
            for (const SketchRef& r : c.refs)
                if (findCurve(sketch, r.curve) >= 0 && std::find(reach.begin(), reach.end(), r.curve) == reach.end()) reach.push_back(r.curve);
        }
    }
    Params p;
    for (const std::string& name : reach) {
        const int ci = findCurve(sketch, name);
        p.at[name] = {ci, static_cast<int>(p.x.size())};
        appendParams(sketch.curves()[static_cast<std::size_t>(ci)], p.x);
    }
    rep.parameters = static_cast<int>(p.x.size());
    const std::vector<double> x0 = p.x;
    const auto [gIndex, g0] = p.at.at(gc.name);
    (void)gIndex;
    const std::vector<char> pos = positional(gc);
    const V2 delta{static_cast<double>(to.x) - from.x, static_cast<double>(to.y) - from.y};
    const V2 target{to.x, to.y};

    // How far a point or rim grip is from where the drag wants it.
    auto pull = [&](const View& g, std::vector<double>& r) {
        if (grip.rim) {
            V2 c;
            double rad = 0;
            g.circle(grip.ref, &c, &rad);
            r.push_back(len(target - c) - rad);
        } else if (grip.ref.point >= 0) {
            V2 q;
            if (g.point(grip.ref, &q)) r.insert(r.end(), {q.x - target.x, q.y - target.y});
        }
    };
    // A whole curve moves rigidly: its positions by delta, its sizes kept (positional
    // parameters come in x, y pairs ahead of the sizes, for every kind).
    std::vector<double> wantRigid;
    if (!grip.rim && grip.ref.point < 0) {
        wantRigid.resize(pos.size());
        for (std::size_t k = 0; k < pos.size(); ++k)
            wantRigid[k] = x0[static_cast<std::size_t>(g0) + k] + (pos[k] ? (k % 2 == 0 ? delta.x : delta.y) : 0.0);
    }
    auto pullRigid = [&](const std::vector<double>& x, std::vector<double>& r) {
        for (std::size_t k = 0; k < wantRigid.size(); ++k) r.push_back(x[static_cast<std::size_t>(g0) + k] - wantRigid[k]);
    };

    auto pulled = [&](const View& g, const std::vector<double>& xv, std::vector<double>& r) {
        if (wantRigid.empty()) pull(g, r);
        else pullRigid(xv, r);
    };
    // The parameters the pull drives directly: a point's coordinates, a rim's radius,
    // a whole curve's.
    std::vector<char> driven(x0.size(), 0);
    {
        std::vector<double> xv = x0;
        std::vector<double> base;
        pulled(View(sketch, p, xv), xv, base);
        for (std::size_t j = 0; j < xv.size(); ++j) {
            xv[j] = x0[j] + 1e-4 * std::max(1.0, std::fabs(x0[j]));
            std::vector<double> moved;
            pulled(View(sketch, p, xv), xv, moved);
            xv[j] = x0[j];
            for (std::size_t i = 0; i < base.size() && i < moved.size(); ++i)
                if (std::fabs(moved[i] - base[i]) > 1e-9) driven[j] = 1;
        }
        // A rim drag drives the radius, not the centre (moving the centre would meet it too).
        if (grip.rim) {
            for (std::size_t k = 0; k < pos.size(); ++k) driven[static_cast<std::size_t>(g0) + k] = 0;
            driven[static_cast<std::size_t>(g0) + 2] = 1;
        }
    }
    auto curveVars = [&](const std::string& name, std::vector<int>& vars) {
        const auto& [ci, first] = p.at.at(name);
        const int count = paramCount(sketch.curves()[static_cast<std::size_t>(ci)]);
        for (int k = 0; k < count; ++k)
            if (std::find(vars.begin(), vars.end(), first + k) == vars.end()) vars.push_back(first + k);
    };
    std::vector<Block> blocks;
    for (const SketchConstraint* c : keep) {
        Block b;
        for (const SketchRef& rf : c->refs) curveVars(rf.curve, b.vars);
        b.eval = [&, c](const std::vector<double>& xv, std::vector<double>& r) {
            const auto rc = residuals(View(sketch, p, xv), *c);
            r.insert(r.end(), rc.begin(), rc.end());
        };
        blocks.push_back(std::move(b));
    }
    Block pullBlock;
    curveVars(gc.name, pullBlock.vars);
    pullBlock.eval = [&](const std::vector<double>& xv, std::vector<double>& r) { pulled(View(sketch, p, xv), xv, r); };
    std::vector<double> pullOnly(x0.size(), 0.0), weighted(x0.size(), 1.0);
    for (std::size_t j = 0; j < x0.size(); ++j)
        if (driven[j]) pullOnly[j] = 1, weighted[j] = 1e-4;
    auto pullLeft = [&](const std::vector<double>& xv) {
        std::vector<double> r;
        pulled(View(sketch, p, xv), xv, r);
        return std::sqrt(norm2(r));
    };
    // 1. The grip goes where it is dragged: only its driven parameters move.
    // 2. The constraints, with the smallest change: the driven parameters weigh 10^4
    //    times the others, so the rest of the sketch gives way first and the grip only
    //    as far as the constraints insist.
    // Again from there until the grip settles (each round leaves about 10^-4 of the
    // last one's give).
    std::vector<double> x = x0;
    double left = 0, settled = std::numeric_limits<double>::infinity();
    for (int round = 0; round < 6; ++round) {
        std::vector<double> xr = x;
        minimumChangeSolve({pullBlock}, pullOnly, xr, 20);
        left = blocks.empty() ? 0.0 : minimumChangeSolve(blocks, weighted, xr, 50);
        if (left > 1e-6) break;
        x = std::move(xr);
        const double now = pullLeft(x);
        if (std::fabs(settled - now) < 1e-9 || now < 1e-9) break;
        settled = now;
    }
    if (left > 1e-6 && x == x0) { // the drag asked too much at once: all parameters alike
        std::vector<double> xs = x0;
        minimumChangeSolve({pullBlock}, pullOnly, xs, 20);
        left = minimumChangeSolve(blocks, std::vector<double>(x0.size(), 1.0), xs, 50);
        if (left <= 1e-6) x = std::move(xs);
    } else if (left > 1e-6)
        left = 0; // a later round failed: keep the last good one
    left *= left;
    rep.residual = std::sqrt(left);
    if (rep.residual > 1e-6) {
        double worst = 0;
        const View g(sketch, p, x);
        for (const SketchConstraint* c : keep)
            for (double v : residuals(g, *c))
                if (std::fabs(v) > worst) worst = std::fabs(v), rep.conflict = c->name;
        return rep; // nothing written
    }
    scatter(sketch, p, x);
    {
        const View g(sketch, p, x);
        std::vector<double> r;
        pulled(g, x, r);
        rep.residual = std::sqrt(norm2(r));
    }
    rep.ok = true;
    return rep;
}

std::vector<int> curveFreedom(const Sketch& sketch) {
    std::vector<int> out(sketch.curves().size(), -1);
    Params p = gather(sketch);
    if (p.x.empty()) return out;
    const std::vector<double> r = allResiduals(sketch, p, p.x);
    const std::size_t m = r.size(), n = p.x.size();
    // The null space of the constraints' Jacobian: the ways the curves can move without
    // breaking any. Reduced row echelon form; each free column gives one direction.
    std::vector<double> R = jacobian(sketch, p, p.x, m);
    double scale = 0;
    for (double v : R) scale = std::max(scale, std::fabs(v));
    const double tol = std::max(scale, 1.0) * 1e-7;
    std::vector<int> pivotCol;
    std::size_t row = 0;
    for (std::size_t col = 0; col < n && row < m; ++col) {
        std::size_t best = m;
        double bv = tol;
        for (std::size_t i = row; i < m; ++i)
            if (std::fabs(R[i * n + col]) > bv) bv = std::fabs(R[i * n + col]), best = i;
        if (best == m) continue;
        for (std::size_t c = 0; c < n; ++c) std::swap(R[row * n + c], R[best * n + c]);
        const double pv = R[row * n + col];
        for (std::size_t c = 0; c < n; ++c) R[row * n + c] /= pv;
        for (std::size_t i = 0; i < m; ++i) {
            if (i == row) continue;
            const double f = R[i * n + col];
            if (f == 0) continue;
            for (std::size_t c = 0; c < n; ++c) R[i * n + c] -= f * R[row * n + c];
        }
        pivotCol.push_back(static_cast<int>(col));
        ++row;
    }
    std::vector<char> isPivot(n, 0);
    for (int c : pivotCol) isPivot[static_cast<std::size_t>(c)] = 1;
    std::vector<std::vector<double>> basis; // one per free column
    for (std::size_t f = 0; f < n; ++f) {
        if (isPivot[f]) continue;
        std::vector<double> v(n, 0.0);
        v[f] = 1;
        for (std::size_t i = 0; i < pivotCol.size(); ++i) v[static_cast<std::size_t>(pivotCol[i])] = -R[i * n + f];
        basis.push_back(std::move(v));
    }
    // Each curve: how many independent ways those directions move its parameters.
    for (const auto& [name, at] : p.at) {
        const SketchCurve& cv = sketch.curves()[static_cast<std::size_t>(at.first)];
        const std::size_t k0 = static_cast<std::size_t>(at.second), kn = static_cast<std::size_t>(paramCount(cv));
        std::vector<double> M; // kn x basis
        const std::size_t b = basis.size();
        for (std::size_t i = 0; i < kn; ++i)
            for (std::size_t j = 0; j < b; ++j) M.push_back(basis[j][k0 + i]);
        out[static_cast<std::size_t>(at.first)] = b == 0 ? 0 : rankOf(M, kn, b);
    }
    // A constraint that isn't met marks its curves.
    for (const SketchConstraint& c : sketch.constraints()) {
        const double e = constraintError(sketch, c);
        if (std::isnan(e) || e <= 1e-4) continue;
        for (const SketchRef& rf : c.refs) {
            const int ci = findCurve(sketch, rf.curve);
            if (ci >= 0) out[static_cast<std::size_t>(ci)] = -2;
        }
    }
    return out;
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

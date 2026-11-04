#include "core/CvCurve.hpp"

#include "core/Sketch.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace sicdm {

namespace {

constexpr double kGapTol = 1e-3, kAngleTol = 0.05, kCurvTol = 0.005, kRateTol = 0.02;
constexpr double kPiC = 3.14159265358979323846;

int findSpan(const BSplineCurve& b, double t) {
    const int n = static_cast<int>(b.cp.size()) - 1, p = b.degree;
    const auto& U = b.knots;
    if (t >= U[static_cast<std::size_t>(n + 1)]) {
        // The last span with a length (the end knot repeats).
        int s = n;
        while (s > p && U[static_cast<std::size_t>(s)] >= U[static_cast<std::size_t>(n + 1)]) --s;
        return s;
    }
    if (t <= U[static_cast<std::size_t>(p)]) return p;
    int lo = p, hi = n + 1, mid = (lo + hi) / 2;
    while (t < U[static_cast<std::size_t>(mid)] || t >= U[static_cast<std::size_t>(mid + 1)]) {
        if (t < U[static_cast<std::size_t>(mid)]) hi = mid;
        else lo = mid;
        mid = (lo + hi) / 2;
    }
    return mid;
}

// ders[k][j]: the k-th derivative of the basis function of CV span - p + j at u (The NURBS
// Book, A2.3). Orders above the degree are zero.
void basisDerivatives(int span, double u, int p, int order, const std::vector<double>& U, double ders[4][8]) {
    double ndu[8][8] = {}, left[8] = {}, right[8] = {}, a[2][8] = {};
    ndu[0][0] = 1.0;
    for (int j = 1; j <= p; ++j) {
        left[j] = u - U[static_cast<std::size_t>(span + 1 - j)];
        right[j] = U[static_cast<std::size_t>(span + j)] - u;
        double saved = 0.0;
        for (int r = 0; r < j; ++r) {
            ndu[j][r] = right[r + 1] + left[j - r];
            const double temp = ndu[j][r] != 0.0 ? ndu[r][j - 1] / ndu[j][r] : 0.0;
            ndu[r][j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        ndu[j][j] = saved;
    }
    for (int k = 0; k < 4; ++k)
        for (int j = 0; j < 8; ++j) ders[k][j] = 0.0;
    for (int j = 0; j <= p; ++j) ders[0][j] = ndu[j][p];
    const int n = std::min(order, p);
    for (int r = 0; r <= p; ++r) {
        int s1 = 0, s2 = 1;
        a[0][0] = 1.0;
        for (int k = 1; k <= n; ++k) {
            double d = 0.0;
            const int rk = r - k, pk = p - k;
            if (r >= k) {
                a[s2][0] = ndu[pk + 1][rk] != 0.0 ? a[s1][0] / ndu[pk + 1][rk] : 0.0;
                d = a[s2][0] * ndu[rk][pk];
            }
            const int j1 = rk >= -1 ? 1 : -rk, j2 = r - 1 <= pk ? k - 1 : p - r;
            for (int j = j1; j <= j2; ++j) {
                a[s2][j] = ndu[pk + 1][rk + j] != 0.0 ? (a[s1][j] - a[s1][j - 1]) / ndu[pk + 1][rk + j] : 0.0;
                d += a[s2][j] * ndu[rk + j][pk];
            }
            if (r <= pk) {
                a[s2][k] = ndu[pk + 1][r] != 0.0 ? -a[s1][k - 1] / ndu[pk + 1][r] : 0.0;
                d += a[s2][k] * ndu[r][pk];
            }
            ders[k][r] = d;
            std::swap(s1, s2);
        }
    }
    double f = p;
    for (int k = 1; k <= n; ++k) {
        for (int j = 0; j <= p; ++j) ders[k][j] *= f;
        f *= p - k;
    }
}

// Solves A x = b (rows of A, three right-hand sides) by Gaussian elimination.
bool solve(std::vector<std::vector<double>>& A, std::vector<DVec3>& b) {
    const std::size_t n = A.size();
    for (std::size_t c = 0; c < n; ++c) {
        std::size_t piv = c;
        for (std::size_t r = c + 1; r < n; ++r)
            if (std::fabs(A[r][c]) > std::fabs(A[piv][c])) piv = r;
        if (std::fabs(A[piv][c]) < 1e-14) return false;
        std::swap(A[c], A[piv]);
        std::swap(b[c], b[piv]);
        for (std::size_t r = c + 1; r < n; ++r) {
            const double f = A[r][c] / A[c][c];
            if (f == 0.0) continue;
            for (std::size_t k = c; k < n; ++k) A[r][k] -= f * A[c][k];
            b[r] = b[r] - b[c] * f;
        }
    }
    for (std::size_t c = n; c-- > 0;) {
        DVec3 s = b[c];
        for (std::size_t k = c + 1; k < n; ++k) s = s - b[k] * A[c][k];
        b[c] = s / A[c][c];
    }
    return true;
}

double relDiff(const DVec3& a, const DVec3& b, double floor) { return dlength(a - b) / std::max({dlength(a), dlength(b), floor}); }

} // namespace

std::vector<double> clampedKnots(std::size_t n, int degree) {
    std::vector<double> k;
    const int p = std::max(1, degree);
    const int spans = std::max(1, static_cast<int>(n) - p);
    for (int i = 0; i <= p; ++i) k.push_back(0.0);
    for (int i = 1; i < spans; ++i) k.push_back(static_cast<double>(i) / spans);
    for (int i = 0; i <= p; ++i) k.push_back(1.0);
    return k;
}

BSplineCurve cvCurveSpline(const SketchCurve& c) {
    BSplineCurve b;
    b.degree = std::clamp(c.degree, 1, 7);
    for (std::size_t i = 0; i < c.pts.size(); ++i) b.cp.push_back(DVec3(c.point3(i)));
    if (static_cast<int>(b.cp.size()) <= b.degree) b.degree = std::max(1, static_cast<int>(b.cp.size()) - 1);
    b.knots = c.knots.size() == b.cp.size() + static_cast<std::size_t>(b.degree) + 1 ? c.knots : clampedKnots(b.cp.size(), b.degree);
    return b;
}

void setCvCurve(SketchCurve& c, const BSplineCurve& b) {
    c.kind = CurveKind::Nurbs;
    const PlaneFrame f = curveFrame(c);
    c.pts.clear(), c.rise.clear();
    for (const DVec3& q : b.cp) {
        const Vec3 v = q.f();
        c.pts.push_back(f.toPlane(v));
        c.rise.push_back(dot(v - f.origin, f.normal));
    }
    c.degree = b.degree;
    c.knots = b.knots == clampedKnots(b.cp.size(), b.degree) ? std::vector<double>{} : b.knots;
    c.closed = false;
}

bool validCvCurve(const BSplineCurve& b, std::string* why) {
    auto fail = [&](const std::string& s) {
        if (why) *why = s;
        return false;
    };
    if (b.degree < 1 || b.degree > 7) return fail("the degree must be 1 to 7");
    if (static_cast<int>(b.cp.size()) < b.degree + 1) return fail("a degree " + std::to_string(b.degree) + " curve needs " + std::to_string(b.degree + 1) + " CVs or more");
    if (b.knots.size() != b.cp.size() + static_cast<std::size_t>(b.degree) + 1) return fail("the knot vector has the wrong length");
    for (std::size_t i = 1; i < b.knots.size(); ++i)
        if (!(b.knots[i] >= b.knots[i - 1])) return fail("the knots must not decrease");
    const std::size_t p = static_cast<std::size_t>(b.degree);
    for (std::size_t i = 1; i <= p; ++i)
        if (b.knots[i] != b.knots[0] || b.knots[b.knots.size() - 1 - i] != b.knots.back()) return fail("the curve must be clamped (end knots repeated degree + 1 times)");
    if (!(b.knots.back() > b.knots.front())) return fail("the knots span nothing");
    return true;
}

void curveDerivatives(const BSplineCurve& b, double t, int order, DVec3 d[4]) {
    for (int k = 0; k < 4; ++k) d[k] = {0, 0, 0};
    if (b.cp.empty()) return;
    const int p = b.degree;
    t = std::clamp(t, b.t0(), b.t1());
    const int span = findSpan(b, t);
    double ders[4][8];
    basisDerivatives(span, t, p, std::min(order, 3), b.knots, ders);
    for (int k = 0; k <= std::min(order, 3); ++k)
        for (int j = 0; j <= p; ++j) d[k] += b.cp[static_cast<std::size_t>(span - p + j)] * ders[k][j];
}

double CurveLocal::curvature() const { return dlength(K); }

CurveLocal curveLocal(const BSplineCurve& b, double t) {
    DVec3 d[4];
    curveDerivatives(b, t, 3, d);
    CurveLocal c;
    c.p = d[0];
    const double v = dlength(d[1]);
    c.speed = v;
    if (v < 1e-12) return c;
    c.T = d[1] / v;
    const DVec3 A = d[2] - c.T * ddot(d[2], c.T);
    c.K = A / (v * v);
    const DVec3 dT = A / v;
    const DVec3 dA = d[3] - c.T * ddot(d[3], c.T) - c.T * ddot(d[2], dT) - dT * ddot(d[2], c.T);
    const double dv = ddot(d[2], c.T);
    const DVec3 dK = dA / (v * v) - A * (2.0 * dv / (v * v * v));
    c.dK = dK / v;
    return c;
}

BSplineCurve reverseCurve(const BSplineCurve& b) {
    BSplineCurve r = b;
    std::reverse(r.cp.begin(), r.cp.end());
    if (!r.w.empty()) std::reverse(r.w.begin(), r.w.end());
    const double s = b.knots.front() + b.knots.back();
    for (std::size_t i = 0; i < b.knots.size(); ++i) r.knots[i] = s - b.knots[b.knots.size() - 1 - i];
    r.m_closed = -1;
    return r;
}

BSplineCurve insertKnot(const BSplineCurve& b, double t) {
    const int p = b.degree, k = findSpan(b, t);
    BSplineCurve r;
    r.degree = p;
    const std::size_t n = b.cp.size();
    r.cp.resize(n + 1);
    for (int i = 0; i <= static_cast<int>(n); ++i) {
        if (i <= k - p) r.cp[static_cast<std::size_t>(i)] = b.cp[static_cast<std::size_t>(i)];
        else if (i >= k + 1) r.cp[static_cast<std::size_t>(i)] = b.cp[static_cast<std::size_t>(i - 1)];
        else {
            const double den = b.knots[static_cast<std::size_t>(i + p)] - b.knots[static_cast<std::size_t>(i)];
            const double a = den > 0 ? (t - b.knots[static_cast<std::size_t>(i)]) / den : 0.0;
            r.cp[static_cast<std::size_t>(i)] = b.cp[static_cast<std::size_t>(i - 1)] * (1 - a) + b.cp[static_cast<std::size_t>(i)] * a;
        }
    }
    r.knots = b.knots;
    r.knots.insert(r.knots.begin() + k + 1, t);
    return r;
}

bool fitCurve(const std::vector<DVec3>& pts, int degree, int spans, BSplineCurve& out, std::string* why) {
    auto fail = [&](const std::string& s) {
        if (why) *why = s;
        return false;
    };
    if (degree < 1 || degree > 7) return fail("the degree must be 1 to 7");
    if (spans < 1) return fail("a curve needs one span or more");
    const int n = spans + degree; // CVs
    const std::size_t m = pts.size();
    if (static_cast<int>(m) < n) return fail(std::to_string(n) + " CVs need " + std::to_string(n) + " points or more to fit");
    // Chord length parameters.
    std::vector<double> u(m, 0.0);
    double total = 0;
    for (std::size_t i = 1; i < m; ++i) total += dlength(pts[i] - pts[i - 1]);
    if (total <= 1e-12) return fail("the points are all in one place");
    for (std::size_t i = 1; i < m; ++i) u[i] = u[i - 1] + dlength(pts[i] - pts[i - 1]) / total;
    u.back() = 1.0;
    // Knots averaged from the parameters, so each span holds points (The NURBS Book 9.69).
    BSplineCurve b;
    b.degree = degree;
    b.knots.assign(static_cast<std::size_t>(degree + 1), 0.0);
    const double d = static_cast<double>(m) / static_cast<double>(n - degree);
    for (int j = 1; j < n - degree; ++j) {
        const int i = std::clamp(static_cast<int>(j * d), 1, static_cast<int>(m) - 1);
        const double a = j * d - i;
        b.knots.push_back((1 - a) * u[static_cast<std::size_t>(i - 1)] + a * u[static_cast<std::size_t>(i)]);
    }
    for (int i = 0; i <= degree; ++i) b.knots.push_back(1.0);
    for (std::size_t i = 1; i < b.knots.size(); ++i)
        if (b.knots[i] < b.knots[i - 1]) b.knots[i] = b.knots[i - 1];
    b.cp.assign(static_cast<std::size_t>(n), pts.front());
    b.cp.back() = pts.back();
    if (n == 2) {
        out = b;
        return true;
    }
    // Least squares for CVs 1..n-2 with the ends fixed.
    const std::size_t k = static_cast<std::size_t>(n - 2);
    std::vector<std::vector<double>> A(k, std::vector<double>(k, 0.0));
    std::vector<DVec3> rhs(k, {0, 0, 0});
    for (std::size_t s = 0; s < m; ++s) {
        double ders[4][8];
        const int span = findSpan(b, u[s]);
        basisDerivatives(span, u[s], degree, 0, b.knots, ders);
        DVec3 r = pts[s];
        double N[64] = {};
        for (int j = 0; j <= degree; ++j) {
            const int idx = span - degree + j;
            if (idx == 0) r = r - pts.front() * ders[0][j];
            else if (idx == n - 1) r = r - pts.back() * ders[0][j];
        }
        for (int j = 0; j <= degree; ++j) {
            const int idx = span - degree + j;
            if (idx <= 0 || idx >= n - 1) continue;
            N[j] = ders[0][j];
        }
        for (int a1 = 0; a1 <= degree; ++a1) {
            const int i1 = span - degree + a1;
            if (i1 <= 0 || i1 >= n - 1) continue;
            rhs[static_cast<std::size_t>(i1 - 1)] += r * N[a1];
            for (int a2 = 0; a2 <= degree; ++a2) {
                const int i2 = span - degree + a2;
                if (i2 <= 0 || i2 >= n - 1) continue;
                A[static_cast<std::size_t>(i1 - 1)][static_cast<std::size_t>(i2 - 1)] += N[a1] * N[a2];
            }
        }
    }
    if (!solve(A, rhs)) return fail("too few points between the knots for " + std::to_string(spans) + " spans");
    for (std::size_t i = 0; i < k; ++i) b.cp[i + 1] = rhs[i];
    out = b;
    return true;
}

double curveLength(const BSplineCurve& b) {
    if (b.cp.size() < 2) return 0;
    const int steps = 64 * static_cast<int>(b.cp.size());
    double L = 0;
    DVec3 prev = b.eval(b.t0());
    for (int i = 1; i <= steps; ++i) {
        const DVec3 q = b.eval(b.t0() + (b.t1() - b.t0()) * i / steps);
        L += dlength(q - prev);
        prev = q;
    }
    return L;
}

double curveDeviation(const BSplineCurve& a, const BSplineCurve& b) {
    const int nb = 400;
    std::vector<DVec3> sb;
    for (int i = 0; i <= nb; ++i) sb.push_back(b.eval(b.t0() + (b.t1() - b.t0()) * i / nb));
    double worst = 0;
    for (int i = 0; i <= 200; ++i) {
        const DVec3 q = a.eval(a.t0() + (a.t1() - a.t0()) * i / 200);
        int best = 0;
        double bd = 1e300;
        for (int j = 0; j <= nb; ++j)
            if (const double dd = dlength(sb[static_cast<std::size_t>(j)] - q); dd < bd) bd = dd, best = j;
        // Refine with Newton steps on (C(t) - q) . C'(t) = 0.
        double t = b.t0() + (b.t1() - b.t0()) * best / nb;
        for (int it = 0; it < 8; ++it) {
            DVec3 d[4];
            curveDerivatives(b, t, 2, d);
            const DVec3 e = d[0] - q;
            const double f = ddot(e, d[1]), df = ddot(d[1], d[1]) + ddot(e, d[2]);
            if (std::fabs(df) < 1e-18) break;
            t = std::clamp(t - f / df, b.t0(), b.t1());
        }
        bd = std::min(bd, dlength(b.eval(t) - q));
        worst = std::max(worst, bd);
    }
    return worst;
}

bool rebuildCurve(const BSplineCurve& in, int degree, int spans, BSplineCurve& out, double* maxDeviation, std::string* why) {
    const int samples = std::max(200, 30 * (spans + degree));
    std::vector<DVec3> pts;
    for (int i = 0; i <= samples; ++i) pts.push_back(in.eval(in.t0() + (in.t1() - in.t0()) * i / samples));
    if (!fitCurve(pts, degree, spans, out, why)) return false;
    if (maxDeviation) *maxDeviation = std::max(curveDeviation(in, out), curveDeviation(out, in));
    return true;
}

Continuity continuityBetween(const CurveLocal& a, const CurveLocal& b) {
    Continuity c;
    c.gap = dlength(a.p - b.p);
    c.angle = std::acos(std::clamp(ddot(a.T, b.T), -1.0, 1.0)) * 180.0 / kPiC;
    c.curvature = relDiff(a.K, b.K, 2e-4);
    c.rate = relDiff(a.dK, b.dK, 5e-6);
    if (c.gap > kGapTol) return c;
    c.level = 0;
    if (a.speed < 1e-12 || b.speed < 1e-12 || c.angle > kAngleTol) return c;
    c.level = 1;
    if (c.curvature > kCurvTol) return c;
    c.level = 2;
    if (c.rate > kRateTol) return c;
    c.level = 3;
    return c;
}

std::string continuityText(const Continuity& c) {
    char buf[160];
    std::snprintf(buf, sizeof buf, "%s (gap %.4f mm, %.3f deg, curvature %.2f %%, rate %.2f %%)", c.level < 0 ? "apart" : ("G" + std::to_string(c.level)).c_str(), c.gap,
                  c.angle, 100.0 * c.curvature, 100.0 * c.rate);
    return buf;
}

bool matchStart(BSplineCurve& a, const CurveLocal& to, int level, std::string* why) {
    auto fail = [&](const std::string& s) {
        if (why) *why = s;
        return false;
    };
    level = std::clamp(level, 0, 3);
    const int p = a.degree;
    if (level > p) return fail("G" + std::to_string(level) + " needs a curve of degree " + std::to_string(level) + " or more (this one is degree " + std::to_string(p) + ")");
    if (static_cast<int>(a.cp.size()) < level + 2)
        return fail("G" + std::to_string(level) + " moves " + std::to_string(level + 1) + " CVs at the end and keeps the far one: the curve needs " + std::to_string(level + 2) +
                    " CVs or more");
    if (level >= 1 && to.speed < 1e-12) return fail("the other curve has no tangent at its end");
    const double t0 = a.t0();
    double ders[4][8];
    basisDerivatives(p, t0, p, level, a.knots, ders); // span p at the clamped start: CV j is basis j
    auto deriv = [&](int k) {
        DVec3 s{0, 0, 0};
        for (int j = 0; j <= k; ++j) s += a.cp[static_cast<std::size_t>(j)] * ders[k][j];
        return s;
    };
    auto setCv = [&](int k, const DVec3& want) {
        DVec3 rest{0, 0, 0};
        for (int j = 0; j < k; ++j) rest += a.cp[static_cast<std::size_t>(j)] * ders[k][j];
        a.cp[static_cast<std::size_t>(k)] = (want - rest) / ders[k][k];
    };
    a.cp[0] = to.p;
    if (level < 1) return true;
    double s = dlength(deriv(1));
    if (s < 1e-9) s = std::fabs(ders[1][1]) * std::max(curveLength(a) / static_cast<double>(a.cp.size()), 1e-3);
    setCv(1, to.T * s);
    if (level < 2) return true;
    const double s1 = ddot(deriv(2), to.T);
    setCv(2, to.T * s1 + to.K * (s * s));
    if (level < 3) return true;
    const double k2 = ddot(to.K, to.K);
    const double s2 = ddot(deriv(3), to.T) + s * s * s * k2;
    setCv(3, to.T * s2 + to.K * (3.0 * s * s1) + to.dK * (s * s * s));
    return true;
}

} // namespace sicdm

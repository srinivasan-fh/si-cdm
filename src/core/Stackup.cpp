// Tolerance stack-up: worst case, RSS, modified RSS and Monte Carlo over a chain of
// dimensions, against the spec, with each one's share.

#include "core/Stackup.hpp"

#include "core/Drawing.hpp"
#include "core/Json.hpp"
#include "core/Scene.hpp"
#include "core/Sculpt.hpp"
#include "core/Tolerances.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <random>

namespace sicdm {

namespace {

constexpr double kSqrt3 = 1.7320508075688772, kSqrt6 = 2.449489742783178;

std::string num(double v, int digits = 5) {
    char b[48];
    std::snprintf(b, sizeof b, "%.*g", digits, std::fabs(v) < 5e-12 ? 0.0 : v);
    return b;
}

// Parts per million: whole numbers from 100 up (16200, not 1.62e+04).
std::string ppmText(double v) {
    char b[48];
    if (v >= 100) std::snprintf(b, sizeof b, "%.0f", v);
    else std::snprintf(b, sizeof b, "%.3g", v);
    return b;
}

// A deviation with its sign always shown: +0.1, -0.05, 0.
std::string dev(double v) {
    if (std::fabs(v) < 5e-12) return "0";
    return (v > 0 ? "+" : "") + num(v);
}

std::string esc(const std::string& t) {
    std::string o;
    for (char c : t) {
        if (c == '&') o += "&amp;";
        else if (c == '<') o += "&lt;";
        else if (c == '>') o += "&gt;";
        else if (c == '"') o += "&quot;";
        else o += c;
    }
    return o;
}

std::string csvCell(const std::string& t) {
    if (t.find_first_of(",\"\n") == std::string::npos) return t;
    std::string o = "\"";
    for (char c : t) o += c == '"' ? std::string("\"\"") : std::string(1, c);
    return o + "\"";
}

double normalCdf(double x) { return 0.5 * std::erfc(-x / std::sqrt(2.0)); }

// Uniform in [0, 1) from the generator's top 53 bits: the same numbers on every platform
// (the standard distributions are not).
double unit(std::mt19937_64& g) { return static_cast<double>(g() >> 11) * (1.0 / 9007199254740992.0); }

double gauss(std::mt19937_64& g) {
    double u = unit(g);
    while (u <= 0) u = unit(g);
    const double v = unit(g);
    return std::sqrt(-2.0 * std::log(u)) * std::cos(6.283185307179586 * v);
}

int bodyAt(const Scene& scene, const std::string& name) {
    const auto& b = scene.bodies();
    for (std::size_t i = 0; i < b.size(); ++i)
        if (b[i].name == name) return static_cast<int>(i);
    auto low = [](std::string s) {
        for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        return s;
    };
    for (std::size_t i = 0; i < b.size(); ++i)
        if (low(b[i].name) == low(name)) return static_cast<int>(i);
    return -1;
}

bool extentAlong(const Scene& scene, const std::string& body, const Vec3& along, double& lo, double& hi, std::string* why) {
    const int i = bodyAt(scene, body);
    if (i < 0) {
        if (why) *why = "No body named " + body;
        return false;
    }
    const double L = length(along);
    if (!(L > 0)) {
        if (why) *why = "The direction has no length";
        return false;
    }
    const MeshData m = worldMesh(scene.bodies()[static_cast<std::size_t>(i)]);
    if (m.vertices.empty()) {
        if (why) *why = body + " has no shape";
        return false;
    }
    lo = 1e300, hi = -1e300;
    for (const Vertex& vx : m.vertices) {
        const Vec3& p = vx.position;
        const double d = (static_cast<double>(p.x) * along.x + static_cast<double>(p.y) * along.y + static_cast<double>(p.z) * along.z) / L;
        lo = std::min(lo, d), hi = std::max(hi, d);
    }
    return true;
}

} // namespace

const char* stackDistName(StackDist d) {
    switch (d) {
    case StackDist::Uniform: return "uniform";
    case StackDist::Triangular: return "triangular";
    default: return "normal";
    }
}

bool stackItemFromFit(StackItem& item, std::string* why) {
    FitDeviations f;
    if (!isoFit(item.fit, std::fabs(item.nominal), f, why)) return false;
    item.upper = f.upper, item.lower = f.lower;
    return true;
}

bool stackItemFromGeneral(StackItem& item, std::string* why) {
    const char c = static_cast<char>(std::tolower(static_cast<unsigned char>(item.general)));
    if (c != 'f' && c != 'm' && c != 'c' && c != 'v') {
        if (why) *why = "The general tolerance class is f, m, c or v (ISO 2768-1)";
        return false;
    }
    const double t = generalTolerance(c, std::fabs(item.nominal));
    if (!(t > 0)) {
        if (why) *why = std::string("ISO 2768-") + c + " has no tolerance for " + num(std::fabs(item.nominal)) + " mm";
        return false;
    }
    item.general = c;
    item.upper = t, item.lower = -t;
    return true;
}

double stackItemSigma(const StackItem& it) {
    const double half = 0.5 * std::fabs(it.upper - it.lower);
    switch (it.dist) {
    case StackDist::Uniform: return half / kSqrt3;
    case StackDist::Triangular: return half / kSqrt6;
    default: return half / (3.0 * (it.cp > 0 ? it.cp : 1.0));
    }
}

namespace {

// Turns v about the unit axis u by a (radians).
std::array<double, 3> turned(const std::array<double, 3>& v, const std::array<double, 3>& u, double a) {
    const double c = std::cos(a), s = std::sin(a), d = u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
    const std::array<double, 3> x{u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]};
    return {v[0] * c + x[0] * s + u[0] * d * (1 - c), v[1] * c + x[1] * s + u[1] * d * (1 - c), v[2] * c + x[2] * s + u[2] * d * (1 - c)};
}

std::array<double, 3> unitOf(const Vec3& v) {
    const double l = std::sqrt(static_cast<double>(v.x) * v.x + static_cast<double>(v.y) * v.y + static_cast<double>(v.z) * v.z);
    return l > 0 ? std::array<double, 3>{v.x / l, v.y / l, v.z / l} : std::array<double, 3>{1, 0, 0};
}

} // namespace

double stackValue(const ToleranceStack& st, const std::vector<double>& x) {
    if (!st.vector) {
        double v = 0;
        for (std::size_t i = 0; i < st.items.size() && i < x.size(); ++i) v += st.items[i].sign * st.items[i].sens * x[i];
        return v;
    }
    // The loop: each length along its direction as turned so far; each angle turns the frame
    // (the three directions every later length is given in).
    std::array<double, 3> p{0, 0, 0}, ex{1, 0, 0}, ey{0, 1, 0}, ez{0, 0, 1};
    for (std::size_t i = 0; i < st.items.size() && i < x.size(); ++i) {
        const StackItem& it = st.items[i];
        const double v = it.sign * it.sens * x[i];
        if (it.angle) {
            // The axis is given in the frame as turned so far.
            const std::array<double, 3> a0 = unitOf(it.axis);
            const std::array<double, 3> a{ex[0] * a0[0] + ey[0] * a0[1] + ez[0] * a0[2], ex[1] * a0[0] + ey[1] * a0[1] + ez[1] * a0[2],
                                          ex[2] * a0[0] + ey[2] * a0[1] + ez[2] * a0[2]};
            const double r = v * 3.14159265358979323846 / 180.0;
            ex = turned(ex, a, r), ey = turned(ey, a, r), ez = turned(ez, a, r);
        } else {
            const std::array<double, 3> d = unitOf(it.dir);
            for (int k = 0; k < 3; ++k) p[static_cast<std::size_t>(k)] += v * (ex[static_cast<std::size_t>(k)] * d[0] + ey[static_cast<std::size_t>(k)] * d[1] + ez[static_cast<std::size_t>(k)] * d[2]);
        }
    }
    if (st.measure.x != 0 || st.measure.y != 0 || st.measure.z != 0) {
        const std::array<double, 3> m = unitOf(st.measure);
        return p[0] * m[0] + p[1] * m[1] + p[2] * m[2];
    }
    return std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
}

std::vector<double> stackSensitivities(const ToleranceStack& st) {
    const std::size_t n = st.items.size();
    std::vector<double> c(n, 0.0);
    if (!st.vector) {
        for (std::size_t i = 0; i < n; ++i) c[i] = st.items[i].sign * st.items[i].sens;
        return c;
    }
    std::vector<double> x(n);
    for (std::size_t i = 0; i < n; ++i) x[i] = st.items[i].nominal;
    for (std::size_t i = 0; i < n; ++i) {
        // A central difference, a small part of the band (or of the size) wide.
        const StackItem& it = st.items[i];
        const double h = std::max({1e-7, 1e-3 * std::fabs(it.upper - it.lower), 1e-7 * std::fabs(it.nominal)});
        std::vector<double> up = x, down = x;
        up[i] += h, down[i] -= h;
        c[i] = (stackValue(st, up) - stackValue(st, down)) / (2 * h);
    }
    return c;
}

namespace {

// Each item's band: least and greatest value.
void bandOf(const StackItem& it, double& lo, double& hi) {
    lo = it.nominal + std::min(it.upper, it.lower), hi = it.nominal + std::max(it.upper, it.lower);
}

struct VectorExtremes {
    double low = 0, high = 0;
};

// The least and greatest result over the tolerance box: every corner (up to 14 dimensions;
// beyond, the corners the sensitivities point to), then from the best corner a search along
// each dimension in turn (golden section, so an extreme inside a band is found too), until
// nothing improves.
VectorExtremes vectorExtremes(const ToleranceStack& st) {
    const std::size_t n = st.items.size();
    std::vector<double> lo(n), hi(n);
    for (std::size_t i = 0; i < n; ++i) bandOf(st.items[i], lo[i], hi[i]);
    auto extreme = [&](double sign) {
        std::vector<double> best(n), x(n);
        double bestV = -1e300;
        auto consider = [&](const std::vector<double>& c) {
            const double v = sign * stackValue(st, c);
            if (v > bestV) bestV = v, best = c;
        };
        if (n <= 14) {
            for (std::uint32_t m = 0; m < (1u << n); ++m) {
                for (std::size_t i = 0; i < n; ++i) x[i] = (m >> i) & 1u ? hi[i] : lo[i];
                consider(x);
            }
        } else {
            const std::vector<double> g = stackSensitivities(st);
            for (std::size_t i = 0; i < n; ++i) x[i] = sign * g[i] >= 0 ? hi[i] : lo[i];
            consider(x);
        }
        for (std::size_t i = 0; i < n; ++i) x[i] = 0.5 * (lo[i] + hi[i]);
        consider(x);
        x = best;
        for (int pass = 0; pass < 30; ++pass) {
            const double before = bestV;
            for (std::size_t i = 0; i < n; ++i) {
                if (!(hi[i] > lo[i])) continue;
                auto at = [&](double t) {
                    std::vector<double> y = x;
                    y[i] = t;
                    return sign * stackValue(st, y);
                };
                // Golden section on [lo, hi], and the ends themselves.
                const double k = 0.6180339887498949;
                double a = lo[i], b = hi[i], c = b - k * (b - a), d = a + k * (b - a), fc = at(c), fd = at(d);
                for (int it = 0; it < 60 && b - a > 1e-12 * (1 + std::fabs(a) + std::fabs(b)); ++it) {
                    if (fc > fd) b = d, d = c, fd = fc, c = b - k * (b - a), fc = at(c);
                    else a = c, c = d, fc = fd, d = a + k * (b - a), fd = at(d);
                }
                for (const double t : {lo[i], hi[i], 0.5 * (a + b), x[i]}) {
                    const double v = at(t);
                    if (v > bestV) bestV = v, x[i] = t;
                }
            }
            if (bestV - before <= 1e-13 * (1 + std::fabs(bestV))) break;
        }
        return sign * bestV;
    };
    return {extreme(-1), extreme(1)};
}

struct SecondOrder {
    double mean = 0, variance = 0;
    std::vector<double> share; // each item's part of the variance
};

// Mean and variance to second order about the bands' middles: f + (1/2) sum f_ii s_i^2, and
// sum f_i^2 s_i^2 + (1/2) sum_ij f_ij^2 s_i^2 s_j^2 (independent, symmetric spreads), the
// derivatives taken over each item's own sigma.
SecondOrder secondOrder(const ToleranceStack& st) {
    const std::size_t n = st.items.size();
    std::vector<double> m(n), sd(n);
    for (std::size_t i = 0; i < n; ++i) {
        double lo, hi;
        bandOf(st.items[i], lo, hi);
        m[i] = 0.5 * (lo + hi), sd[i] = stackItemSigma(st.items[i]);
    }
    auto f = [&](const std::vector<double>& x) { return stackValue(st, x); };
    const double f0 = f(m);
    std::vector<double> g(n, 0.0);
    std::vector<std::vector<double>> h(n, std::vector<double>(n, 0.0));
    auto step = [&](std::size_t i) { return sd[i] > 0 ? sd[i] : 1e-6 * (1 + std::fabs(m[i])); };
    for (std::size_t i = 0; i < n; ++i) {
        const double hi_ = step(i);
        std::vector<double> p = m, q = m;
        p[i] += hi_, q[i] -= hi_;
        const double fp = f(p), fq = f(q);
        g[i] = (fp - fq) / (2 * hi_);
        h[i][i] = (fp - 2 * f0 + fq) / (hi_ * hi_);
        for (std::size_t j = 0; j < i; ++j) {
            const double hj = step(j);
            std::vector<double> a = m, b = m, c = m, d = m;
            a[i] += hi_, a[j] += hj;
            b[i] += hi_, b[j] -= hj;
            c[i] -= hi_, c[j] += hj;
            d[i] -= hi_, d[j] -= hj;
            h[i][j] = h[j][i] = (f(a) - f(b) - f(c) + f(d)) / (4 * hi_ * hj);
        }
    }
    SecondOrder so;
    so.mean = f0;
    so.share.assign(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        so.mean += 0.5 * h[i][i] * sd[i] * sd[i];
        double part = g[i] * g[i] * sd[i] * sd[i];
        for (std::size_t j = 0; j < n; ++j) part += 0.5 * h[i][j] * h[i][j] * sd[i] * sd[i] * sd[j] * sd[j];
        so.share[i] = part;
        so.variance += part;
    }
    return so;
}

// The inverse of the standard normal distribution (Acklam's, about 1e-9 relative).
double normalQuantile(double p) {
    static const double a[] = {-3.969683028665376e+01, 2.209460984245205e+02, -2.759285104469687e+02, 1.383577518672690e+02, -3.066479806614716e+01, 2.506628277459239e+00};
    static const double b[] = {-5.447609879822406e+01, 1.615858368580409e+02, -1.556989798598866e+02, 6.680131188771972e+01, -1.328068155288572e+01};
    static const double c[] = {-7.784894002430293e-03, -3.223964580411365e-01, -2.400758277161838e+00, -2.549732539343734e+00, 4.374664141464968e+00, 2.938163982698783e+00};
    static const double d[] = {7.784695709041462e-03, 3.224671290700398e-01, 2.445134137142996e+00, 3.754408661907416e+00};
    p = std::clamp(p, 1e-12, 1 - 1e-12);
    if (p < 0.02425) {
        const double q = std::sqrt(-2 * std::log(p));
        return (((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) / ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1);
    }
    if (p > 1 - 0.02425) {
        const double q = std::sqrt(-2 * std::log(1 - p));
        return -(((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) / ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1);
    }
    const double q = p - 0.5, r = q * q;
    return (((((a[0] * r + a[1]) * r + a[2]) * r + a[3]) * r + a[4]) * r + a[5]) * q / (((((b[0] * r + b[1]) * r + b[2]) * r + b[3]) * r + b[4]) * r + 1);
}

// The loop's own mean and variance, integrated over every item's distribution with a fixed
// quasi-random point set (Halton, shifted once by a fixed amount): the same every time, close
// to exact with smooth loops and at kinks alike (no derivatives taken).
std::pair<double, double> vectorMoments(const ToleranceStack& st, int points = 32768) {
    const std::size_t n = st.items.size();
    std::vector<int> primes;
    for (int k = 2; primes.size() < n; ++k) {
        bool prime = true;
        for (const int q : primes) prime = prime && k % q != 0;
        if (prime) primes.push_back(k);
    }
    std::vector<double> lo(n), hi(n), sd(n), shift(n), x(n);
    for (std::size_t i = 0; i < n; ++i) {
        bandOf(st.items[i], lo[i], hi[i]);
        sd[i] = stackItemSigma(st.items[i]);
        shift[i] = std::fmod(0.5 + 0.6180339887498949 * static_cast<double>(i + 1), 1.0);
    }
    double sum = 0, sum2 = 0;
    for (int k = 1; k <= points; ++k) {
        for (std::size_t i = 0; i < n; ++i) {
            double u = 0, f = 1.0 / primes[i];
            for (int m = k; m > 0; m /= primes[i], f /= primes[i]) u += f * (m % primes[i]);
            u = std::fmod(u + shift[i], 1.0);
            const double mid = 0.5 * (lo[i] + hi[i]), w = hi[i] - lo[i];
            switch (st.items[i].dist) {
            case StackDist::Uniform: x[i] = lo[i] + w * u; break;
            case StackDist::Triangular: x[i] = u < 0.5 ? lo[i] + w * std::sqrt(u / 2) : hi[i] - w * std::sqrt((1 - u) / 2); break;
            default: x[i] = mid + sd[i] * normalQuantile(u); break;
            }
        }
        const double v = stackValue(st, x);
        sum += v, sum2 += v * v;
    }
    const double mean = sum / points;
    return {mean, std::max(0.0, sum2 / points - mean * mean)};
}

} // namespace

StackResult analyzeStack(const ToleranceStack& st, std::uint64_t seed) {
    StackResult r;
    const std::size_t n = st.items.size();
    r.variance.assign(n, 0.0);
    r.wcShare.assign(n, 0.0);
    double var = 0, band = 0;
    // About the nominal: the result there, and how each dimension moves it.
    std::vector<double> nominals(n);
    for (std::size_t i = 0; i < n; ++i) nominals[i] = st.items[i].nominal;
    const std::vector<double> sens = stackSensitivities(st);
    r.nominal = stackValue(st, nominals);
    r.mean = r.wcHigh = r.wcLow = r.nominal;
    for (std::size_t i = 0; i < n; ++i) {
        const StackItem& it = st.items[i];
        const double c = sens[i];
        const double hi = std::max(it.upper, it.lower), lo = std::min(it.upper, it.lower); // from the nominal
        r.mean += c * 0.5 * (hi + lo);
        r.wcHigh += std::max(c * hi, c * lo);
        r.wcLow += std::min(c * hi, c * lo);
        const double s = c * stackItemSigma(it);
        r.variance[i] = s * s;
        var += s * s;
        r.wcShare[i] = std::fabs(c) * 0.5 * (hi - lo);
        band += r.wcShare[i];
    }
    // A vector loop isn't a straight sum: its worst case is searched for over the tolerance box
    // (the linear one can be short or long where the loop bends a lot), and its spread taken to
    // second order (the bend moves the mean and adds to the variance).
    if (st.vector && n > 0) {
        const VectorExtremes ex = vectorExtremes(st);
        r.wcLow = ex.low, r.wcHigh = ex.high;
        // The spread: the loop's own mean and variance (integrated, not a Taylor estimate, so
        // right at a kink too); each item's share of it from the second-order breakdown.
        const SecondOrder so = secondOrder(st);
        const auto [mean, variance] = vectorMoments(st);
        r.mean = mean;
        var = variance;
        for (std::size_t i = 0; i < n; ++i) r.variance[i] = so.variance > 0 ? so.share[i] / so.variance * variance : 0.0;
    }
    for (std::size_t i = 0; i < n; ++i) {
        r.variance[i] = var > 0 ? 100.0 * r.variance[i] / var : 0.0;
        r.wcShare[i] = band > 0 ? 100.0 * r.wcShare[i] / band : 0.0;
    }
    r.sigma = std::sqrt(var);
    // (Few dimensions, uniform ones above all, can put mean -+ 3 sigma past the worst case,
    // which no assembly can reach: RSS stops there.)
    r.rssLow = std::max(r.mean - 3 * r.sigma, r.wcLow), r.rssHigh = std::min(r.mean + 3 * r.sigma, r.wcHigh);
    const bool rssCapped = r.mean - 3 * r.sigma < r.wcLow - 1e-12 || r.mean + 3 * r.sigma > r.wcHigh + 1e-12;
    r.mrssLow = std::max(r.mean - 4.5 * r.sigma, r.wcLow), r.mrssHigh = std::min(r.mean + 4.5 * r.sigma, r.wcHigh);

    // Against the spec.
    const bool spec = st.hasLow || st.hasHigh;
    if (spec) {
        r.wcPass = (!st.hasLow || r.wcLow >= st.low - 1e-12) && (!st.hasHigh || r.wcHigh <= st.high + 1e-12);
        r.rssPass = (!st.hasLow || r.rssLow >= st.low - 1e-12) && (!st.hasHigh || r.rssHigh <= st.high + 1e-12);
        double out = 0, near = 1e300;
        if (r.sigma > 0) {
            if (st.hasLow) out += normalCdf((st.low - r.mean) / r.sigma), near = std::min(near, r.mean - st.low);
            if (st.hasHigh) out += 1.0 - normalCdf((st.high - r.mean) / r.sigma), near = std::min(near, st.high - r.mean);
            r.cpk = near / (3 * r.sigma);
        } else {
            const bool inside = (!st.hasLow || r.mean >= st.low) && (!st.hasHigh || r.mean <= st.high);
            out = inside ? 0 : 1;
        }
        r.ppm = std::min(out, 1.0) * 1e6;
    }

    // Monte Carlo: each item drawn from its own distribution.
    const int N = std::clamp(st.samples, 0, 2000000);
    if (N > 0 && n > 0) {
        std::mt19937_64 g(seed);
        std::vector<double> y(static_cast<std::size_t>(N));
        double sum = 0, sum2 = 0;
        long long outside = 0;
        std::vector<double> x(n);
        for (int k = 0; k < N; ++k) {
            for (std::size_t i = 0; i < n; ++i) {
                const StackItem& it = st.items[i];
                const double lo = it.nominal + std::min(it.upper, it.lower), hi = it.nominal + std::max(it.upper, it.lower);
                const double mid = 0.5 * (lo + hi), half = 0.5 * (hi - lo);
                double xi = mid;
                switch (it.dist) {
                case StackDist::Uniform: xi = lo + (hi - lo) * unit(g); break;
                case StackDist::Triangular: xi = mid + half * (unit(g) + unit(g) - 1.0); break; // the sum of two uniforms
                default: xi = mid + stackItemSigma(it) * gauss(g); break;
                }
                x[i] = xi;
            }
            const double v = stackValue(st, x);
            y[static_cast<std::size_t>(k)] = v;
            sum += v, sum2 += v * v;
            if ((st.hasLow && v < st.low) || (st.hasHigh && v > st.high)) ++outside;
        }
        r.samples = N;
        r.mcMean = sum / N;
        r.mcSigma = std::sqrt(std::max(0.0, sum2 / N - r.mcMean * r.mcMean));
        const auto [mn, mx] = std::minmax_element(y.begin(), y.end());
        r.mcLow = *mn, r.mcHigh = *mx;
        r.mcPpm = spec ? 1e6 * static_cast<double>(outside) / N : 0.0;
        // The histogram over what was seen, the spec limits and the worst case included.
        r.histLow = std::min(r.mcLow, r.wcLow), r.histHigh = std::max(r.mcHigh, r.wcHigh);
        if (st.hasLow) r.histLow = std::min(r.histLow, st.low);
        if (st.hasHigh) r.histHigh = std::max(r.histHigh, st.high);
        const double pad = 0.04 * (r.histHigh - r.histLow);
        r.histLow -= pad, r.histHigh += pad;
        if (!(r.histHigh > r.histLow)) r.histLow -= 0.5, r.histHigh += 0.5;
        r.histogram.assign(40, 0);
        for (double v : y) {
            const int b = static_cast<int>((v - r.histLow) / (r.histHigh - r.histLow) * 40.0);
            ++r.histogram[static_cast<std::size_t>(std::clamp(b, 0, 39))];
        }
        auto pct = [&](double p) {
            const std::size_t k = std::min(y.size() - 1, static_cast<std::size_t>(p * static_cast<double>(y.size() - 1) + 0.5));
            std::nth_element(y.begin(), y.begin() + static_cast<std::ptrdiff_t>(k), y.end());
            return y[k];
        };
        r.mcP0 = pct(0.00135), r.mcP1 = pct(0.99865);
    }

    // What it means, and what to change.
    if (n == 0) {
        r.notes.push_back("The stack has no dimensions yet (STACK name ADD ...)");
        return r;
    }
    if (spec) {
        const bool meanIn = (!st.hasLow || r.mean >= st.low) && (!st.hasHigh || r.mean <= st.high);
        if (!meanIn) {
            r.notes.push_back("The result's middle, " + num(r.mean) + " mm, is outside the spec itself: change a nominal size, tolerances can't fix it");
        } else {
            // Every tolerance scaled about its middle by f moves the limits to mean -+ f x (their half).
            auto scale = [&](double halfLow, double halfHigh) {
                double f = 1e300;
                if (st.hasLow && halfLow > 0) f = std::min(f, (r.mean - st.low) / halfLow);
                if (st.hasHigh && halfHigh > 0) f = std::min(f, (st.high - r.mean) / halfHigh);
                return f;
            };
            const double fw = scale(r.mean - r.wcLow, r.wcHigh - r.mean), fr = scale(3 * r.sigma, 3 * r.sigma);
            if (r.wcPass)
                r.notes.push_back("Worst case fits the spec: every part in tolerance gives a good result");
            else
                r.notes.push_back("Worst case doesn't fit: every tolerance at " + num(100 * fw, 3) + "% of what it is would make it fit");
            if (!r.wcPass && r.rssPass)
                r.notes.push_back("Statistically it fits (RSS, Cpk " + num(r.cpk, 3) + "): about " + ppmText(r.ppm) + " in a million outside, when the parts' sizes are spread as assumed");
            if (!r.rssPass)
                r.notes.push_back("RSS doesn't fit either (Cpk " + num(r.cpk, 3) + ", about " + ppmText(r.ppm) + " in a million outside): every tolerance at " +
                                  num(100 * fr, 3) + "% would bring it to Cpk 1");
        }
    }
    if (st.vector)
        r.notes.push_back("A vector stack: the worst case is the loop's own extremes over the tolerance box, RSS the loop's own mean and spread "
                          "(integrated over every dimension's distribution), and the Monte Carlo run uses the loop itself");
    if (rssCapped)
        r.notes.push_back("With so few dimensions, 3 sigma reaches past the worst case: RSS is taken as the worst case there (the Monte Carlo run shows the real spread)");
    // The RSS figure takes the result as normal; the Monte Carlo run draws each dimension
    // from its own distribution (uniform ones make short tails).
    if (spec && r.samples > 0) {
        const double floor = 1e6 / r.samples; // one sample's worth
        const double a = std::max(r.ppm, floor), b = std::max(r.mcPpm, floor);
        if ((r.ppm >= floor || r.mcPpm > 0) && (a > 2 * b || b > 2 * a))
            r.notes.push_back("The Monte Carlo run, with each dimension's own distribution, puts " + ppmText(r.mcPpm) + " in a million outside the spec, against " +
                              ppmText(r.ppm) + " from RSS (which takes the result as normal)");
    }
    std::size_t top = 0;
    for (std::size_t i = 1; i < n; ++i)
        if (r.variance[i] > r.variance[top]) top = i;
    if (n > 1 && r.variance[top] > 0) {
        // Halving its tolerance takes three quarters of its variance away.
        const double s = r.sigma * std::sqrt(std::max(0.0, 1.0 - 0.75 * r.variance[top] / 100.0));
        r.notes.push_back(st.items[top].name + " gives " + num(r.variance[top], 3) + "% of the variance: halving its tolerance takes the result's sigma from " +
                          num(r.sigma, 4) + " to " + num(s, 4) + " mm");
    }
    return r;
}

bool bodySizeAlong(const Scene& scene, const std::string& body, const Vec3& along, double& size, std::string* why) {
    double lo = 0, hi = 0;
    if (!extentAlong(scene, body, along, lo, hi, why)) return false;
    size = hi - lo;
    return true;
}

bool bodyGapAlong(const Scene& scene, const std::string& from, const std::string& to, const Vec3& along, double& gap, std::string* why) {
    double a0 = 0, a1 = 0, b0 = 0, b1 = 0;
    if (!extentAlong(scene, from, along, a0, a1, why) || !extentAlong(scene, to, along, b0, b1, why)) return false;
    gap = b0 - a1;
    return true;
}

bool stackItemFromDrawing(StackItem& item, const Drawing& drawing, int index, std::string* why) {
    if (index < 0 || index >= static_cast<int>(drawing.dims.size())) {
        if (why) *why = "The drawing has no dimension " + std::to_string(index + 1) + " (it has " + std::to_string(drawing.dims.size()) + ")";
        return false;
    }
    const DrawingDim& d = drawing.dims[static_cast<std::size_t>(index)];
    item.dim = index;
    item.nominal = dimensionValue(drawing, d);
    double hi = 0, lo = 0;
    if (dimensionLimits(drawing, d, hi, lo)) {
        item.upper = hi - item.nominal, item.lower = lo - item.nominal;
        item.general = 0;
        item.fit = d.tol.kind == DimTolerance::Fit ? d.tol.fit : std::string();
        return true;
    }
    // None of its own: the title block's general tolerance (ISO 2768-m when it names none).
    char cls = 'm';
    const std::string& g = drawing.generalTolerance;
    const std::size_t at = g.find("2768-");
    if (at != std::string::npos && at + 5 < g.size()) cls = static_cast<char>(std::tolower(static_cast<unsigned char>(g[at + 5])));
    item.general = cls;
    return stackItemFromGeneral(item, why);
}

bool updateStack(ToleranceStack& st, const Scene& scene, const Drawing& drawing, std::vector<std::string>* report, std::string* why) {
    for (StackItem& it : st.items) {
        const double before = it.nominal, up = it.upper, low = it.lower;
        if (!it.body.empty()) {
            double s = 0;
            if (!bodySizeAlong(scene, it.body, it.along, s, why)) return false;
            it.nominal = s;
            if (!it.fit.empty() && !stackItemFromFit(it, why)) return false;
            if (it.general && !stackItemFromGeneral(it, why)) return false;
        } else if (it.dim >= 0) {
            if (!stackItemFromDrawing(it, drawing, it.dim, why)) return false;
        } else {
            continue;
        }
        if (report && (std::fabs(before - it.nominal) > 1e-9 || std::fabs(up - it.upper) > 1e-12 || std::fabs(low - it.lower) > 1e-12))
            report->push_back(it.name + ": " + num(before) + " " + dev(up) + "/" + dev(low) + " is now " + num(it.nominal) + " " + dev(it.upper) + "/" + dev(it.lower));
    }
    return true;
}

std::vector<std::string> stackReport(const ToleranceStack& st, const StackResult& r) {
    std::vector<std::string> out;
    out.push_back("Stack " + st.name + (st.note.empty() ? "" : " (" + st.note + ")") + ": " + std::to_string(st.items.size()) +
                  (st.items.size() == 1 ? " dimension" : " dimensions"));
    const std::vector<double> sens = st.vector ? stackSensitivities(st) : std::vector<double>{};
    if (st.vector)
        out.push_back(std::string("A vector loop; the result is ") +
                      (st.measure.x != 0 || st.measure.y != 0 || st.measure.z != 0 ? "its end along " + num(st.measure.x) + "," + num(st.measure.y) + "," + num(st.measure.z)
                                                                                  : std::string("the distance from its start to its end")));
    for (std::size_t i = 0; i < st.items.size(); ++i) {
        const StackItem& it = st.items[i];
        std::string l = "  " + std::to_string(i + 1) + ". " + (it.sign < 0 ? "- " : "+ ") + it.name + "  " + num(it.nominal) + " " + dev(it.upper) + "/" +
                        dev(it.lower);
        if (!it.fit.empty()) l += " (" + it.fit + ")";
        else if (it.general) l += std::string(" (ISO 2768-") + it.general + ")";
        if (st.vector) {
            l += it.angle ? " deg about " + num(it.axis.x) + "," + num(it.axis.y) + "," + num(it.axis.z) + ", moves the result " + num(sens[i], 4) + " mm per deg"
                          : " along " + num(it.dir.x) + "," + num(it.dir.y) + "," + num(it.dir.z) + ", sensitivity " + num(sens[i], 4);
        } else if (std::fabs(it.sens - 1) > 1e-12) {
            l += "  x" + num(it.sens);
        }
        l += "  " + std::string(stackDistName(it.dist));
        if (it.dist == StackDist::Normal && std::fabs(it.cp - 1) > 1e-12) l += " Cp " + num(it.cp, 3);
        if (i < r.variance.size()) l += "  [" + num(r.variance[i], 3) + "% of variance, " + num(r.wcShare[i], 3) + "% of worst case]";
        out.push_back(l);
    }
    out.push_back("Nominal " + num(r.nominal) + " mm; middle of the bands " + num(r.mean) + " mm");
    out.push_back("Worst case: " + num(r.wcLow) + " to " + num(r.wcHigh) + " mm (+-" + num(0.5 * (r.wcHigh - r.wcLow)) + ")");
    out.push_back("RSS (3 sigma): " + num(r.rssLow) + " to " + num(r.rssHigh) + " mm (sigma " + num(r.sigma, 4) + ")");
    out.push_back("Modified RSS (x1.5): " + num(r.mrssLow) + " to " + num(r.mrssHigh) + " mm");
    if (r.samples > 0)
        out.push_back("Monte Carlo, " + std::to_string(r.samples) + " samples: mean " + num(r.mcMean) + ", sigma " + num(r.mcSigma, 4) + "; 99.73% between " +
                      num(r.mcP0) + " and " + num(r.mcP1) + " mm; seen " + num(r.mcLow) + " to " + num(r.mcHigh));
    if (st.hasLow || st.hasHigh) {
        std::string sp = "Spec: ";
        sp += st.hasLow ? num(st.low) : std::string("-");
        sp += " to ";
        sp += st.hasHigh ? num(st.high) : std::string("-");
        sp += " mm. Worst case " + std::string(r.wcPass ? "fits" : "doesn't fit") + "; RSS " + (r.rssPass ? "fits" : "doesn't fit") + ", Cpk " + num(r.cpk, 3) +
              ", " + ppmText(r.ppm) + " ppm outside";
        if (r.samples > 0) sp += "; Monte Carlo " + ppmText(r.mcPpm) + " ppm outside";
        out.push_back(sp);
    }
    if (r.measured) {
        const double diff = r.modelGap - r.nominal;
        out.push_back("The model's gap from " + st.gapFrom + " to " + st.gapTo + ": " + num(r.modelGap) + " mm" +
                      (std::fabs(diff) <= 1e-3 ? std::string(", as the stack's nominal") : ", but the stack's nominal is " + num(r.nominal) + " (a dimension missing or the wrong way?)"));
    }
    for (const std::string& n : r.notes) out.push_back(n);
    return out;
}

std::string stackCsv(const ToleranceStack& st, const StackResult& r) {
    std::string s = "Item,Sign,Name,Nominal (mm),Upper (mm),Lower (mm),Sensitivity,Distribution,Cp,Sigma (mm),Variance (%),Worst case (%)\n";
    char b[320];
    const std::vector<double> sens = stackSensitivities(st); // a vector stack's: its effect at the nominal sizes (deg for an angle)
    for (std::size_t i = 0; i < st.items.size(); ++i) {
        const StackItem& it = st.items[i];
        std::snprintf(b, sizeof b, "%zu,%c,%s,%.9g,%.9g,%.9g,%.9g,%s,%.6g,%.9g,%.6g,%.6g\n", i + 1, it.sign < 0 ? '-' : '+', csvCell(it.name).c_str(), it.nominal, it.upper,
                      it.lower, st.vector ? sens[i] : it.sens, stackDistName(it.dist), it.cp, stackItemSigma(it), i < r.variance.size() ? r.variance[i] : 0.0,
                      i < r.wcShare.size() ? r.wcShare[i] : 0.0);
        s += b;
    }
    s += "\nResult,Low (mm),High (mm)\n";
    std::snprintf(b, sizeof b, "Nominal,%.9g,%.9g\nWorst case,%.9g,%.9g\nRSS,%.9g,%.9g\nModified RSS,%.9g,%.9g\n", r.nominal, r.nominal, r.wcLow, r.wcHigh, r.rssLow,
                  r.rssHigh, r.mrssLow, r.mrssHigh);
    s += b;
    if (r.samples > 0) {
        std::snprintf(b, sizeof b, "Monte Carlo 99.73%%,%.9g,%.9g\nMonte Carlo seen,%.9g,%.9g\n", r.mcP0, r.mcP1, r.mcLow, r.mcHigh);
        s += b;
    }
    if (st.hasLow || st.hasHigh) {
        s += "Spec," + (st.hasLow ? num(st.low, 9) : std::string()) + "," + (st.hasHigh ? num(st.high, 9) : std::string()) + "\n";
        std::snprintf(b, sizeof b, "\nCpk,%.6g\nRSS ppm outside,%.6g\nMonte Carlo ppm outside,%.6g\n", r.cpk, r.ppm, r.mcPpm);
        s += b;
    }
    return s;
}

std::string stackSvg(const ToleranceStack& st, const StackResult& r) {
    const double W = 900, left = 70, right = 30, top = 56, plotH = 260;
    const double barTop = top + plotH + 104, barH = 22;
    const double H = barTop + static_cast<double>(st.items.size()) * (barH + 8) + 30;
    const double pw = W - left - right;
    std::string s;
    char b[640];
    std::snprintf(b, sizeof b, "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"%.0f\" height=\"%.0f\" viewBox=\"0 0 %.0f %.0f\" font-family=\"Helvetica, Arial, sans-serif\">\n", W, H, W, H);
    s += b;
    s += "<rect width=\"100%\" height=\"100%\" fill=\"#ffffff\"/>\n";
    std::snprintf(b, sizeof b, "<text x=\"%.0f\" y=\"28\" font-size=\"18\" font-weight=\"bold\" fill=\"#222\">Tolerance stack: %s</text>\n", left, esc(st.name).c_str());
    s += b;
    std::snprintf(b, sizeof b, "<text x=\"%.0f\" y=\"46\" font-size=\"12\" fill=\"#555\">nominal %s mm; worst case %s to %s; RSS %s to %s%s</text>\n", left, num(r.nominal).c_str(),
                  num(r.wcLow).c_str(), num(r.wcHigh).c_str(), num(r.rssLow).c_str(), num(r.rssHigh).c_str(),
                  (st.hasLow || st.hasHigh) ? (std::string("; Cpk ") + num(r.cpk, 3)).c_str() : "");
    s += b;
    std::snprintf(b, sizeof b, "<rect x=\"%.1f\" y=\"%.1f\" width=\"%.1f\" height=\"%.1f\" fill=\"#fafafa\" stroke=\"#bbb\"/>\n", left, top, pw, plotH);
    s += b;
    const double lo = r.histLow, hi = r.histHigh > r.histLow ? r.histHigh : r.histLow + 1;
    auto X = [&](double v) { return left + (v - lo) / (hi - lo) * pw; };
    int most = 1;
    for (int c : r.histogram) most = std::max(most, c);
    const double bw = pw / 40.0;
    for (std::size_t i = 0; i < r.histogram.size(); ++i) {
        const double h = plotH * 0.92 * r.histogram[i] / most;
        const double c0 = lo + (hi - lo) * static_cast<double>(i) / 40.0, c1 = lo + (hi - lo) * static_cast<double>(i + 1) / 40.0;
        const bool out = (st.hasLow && c1 <= st.low) || (st.hasHigh && c0 >= st.high);
        std::snprintf(b, sizeof b, "<rect x=\"%.2f\" y=\"%.2f\" width=\"%.2f\" height=\"%.2f\" fill=\"%s\"/>\n", X(c0) + 0.5, top + plotH - h, bw - 1, h,
                      out ? "#d9534f" : "#6c9bd2");
        s += b;
    }
    auto mark = [&](double v, const char* colour, const char* dash, const std::string& label, int row) {
        if (v < lo || v > hi) return;
        std::snprintf(b, sizeof b, "<line x1=\"%.2f\" y1=\"%.1f\" x2=\"%.2f\" y2=\"%.1f\" stroke=\"%s\" stroke-width=\"2\"%s/>\n", X(v), top, X(v), top + plotH, colour,
                      dash);
        s += b;
        std::snprintf(b, sizeof b, "<text x=\"%.2f\" y=\"%.1f\" font-size=\"11\" fill=\"%s\" text-anchor=\"middle\">%s</text>\n", X(v), top + plotH + 30 + 13.0 * row,
                      colour, esc(label).c_str());
        s += b;
    };
    mark(r.wcLow, "#666666", " stroke-dasharray=\"6 4\"", "WC " + num(r.wcLow), 0);
    mark(r.wcHigh, "#666666", " stroke-dasharray=\"6 4\"", "WC " + num(r.wcHigh), 0);
    mark(r.rssLow, "#2a62b8", " stroke-dasharray=\"2 3\"", "RSS " + num(r.rssLow), 1);
    mark(r.rssHigh, "#2a62b8", " stroke-dasharray=\"2 3\"", "RSS " + num(r.rssHigh), 1);
    if (st.hasLow) mark(st.low, "#c0392b", "", "LSL " + num(st.low), 2);
    if (st.hasHigh) mark(st.high, "#c0392b", "", "USL " + num(st.high), 2);
    // Ticks at a round step: 1, 2 or 5 times a power of ten, about six of them.
    double step = std::pow(10.0, std::floor(std::log10((hi - lo) / 6.0)));
    for (double m : {2.0, 5.0, 10.0})
        if ((hi - lo) / step > 8) step = std::pow(10.0, std::floor(std::log10((hi - lo) / 6.0))) * m;
    for (double v = std::ceil(lo / step) * step; v <= hi + 1e-9 * step; v += step) {
        std::snprintf(b, sizeof b, "<text x=\"%.2f\" y=\"%.1f\" font-size=\"11\" fill=\"#555\" text-anchor=\"middle\">%s</text>\n", X(v), top + plotH + 14, num(std::fabs(v) < 1e-9 * step ? 0.0 : v, 6).c_str());
        s += b;
    }
    std::snprintf(b, sizeof b, "<text x=\"%.0f\" y=\"%.0f\" font-size=\"13\" fill=\"#333\">Share of the variance (bar) and of the worst case (outline)</text>\n", left,
                  barTop - 12);
    s += b;
    const double labelW = 220, barW = pw - labelW - 60;
    for (std::size_t i = 0; i < st.items.size(); ++i) {
        const double y = barTop + static_cast<double>(i) * (barH + 8);
        const double v = i < r.variance.size() ? r.variance[i] : 0, w = i < r.wcShare.size() ? r.wcShare[i] : 0;
        std::snprintf(b, sizeof b, "<text x=\"%.0f\" y=\"%.1f\" font-size=\"12\" fill=\"#333\">%s%s</text>\n", left, y + 15, st.items[i].sign < 0 ? "- " : "+ ",
                      esc(st.items[i].name).c_str());
        s += b;
        std::snprintf(b, sizeof b,
                      "<rect x=\"%.1f\" y=\"%.1f\" width=\"%.2f\" height=\"%.1f\" fill=\"#6c9bd2\"/><rect x=\"%.1f\" y=\"%.1f\" width=\"%.2f\" height=\"%.1f\" fill=\"none\" stroke=\"#444\"/>"
                      "<text x=\"%.1f\" y=\"%.1f\" font-size=\"11\" fill=\"#333\">%s%%</text>\n",
                      left + labelW, y + 3, barW * v / 100.0, barH - 6, left + labelW, y + 1, barW * w / 100.0, barH - 2, left + labelW + barW + 8, y + 15,
                      num(v, 3).c_str());
        s += b;
    }
    s += "</svg>\n";
    return s;
}

Json stacksToJson(const std::vector<ToleranceStack>& stacks) {
    Json a = Json::array();
    for (const ToleranceStack& st : stacks) {
        Json j = Json::object();
        j.set("name", st.name);
        if (!st.note.empty()) j.set("note", st.note);
        if (st.hasLow) j.set("low", st.low);
        if (st.hasHigh) j.set("high", st.high);
        if (!st.gapFrom.empty()) {
            Json along = Json::array();
            along.push(st.gapAlong.x).push(st.gapAlong.y).push(st.gapAlong.z);
            j.set("gap_from", st.gapFrom).set("gap_to", st.gapTo).set("gap_along", along);
        }
        if (st.samples != 100000) j.set("samples", st.samples);
        Json items = Json::array();
        for (const StackItem& it : st.items) {
            Json i = Json::object();
            i.set("name", it.name).set("nominal", it.nominal).set("upper", it.upper).set("lower", it.lower);
            if (it.sign < 0) i.set("sign", -1);
            if (it.sens != 1) i.set("sens", it.sens);
            if (it.dist != StackDist::Normal) i.set("dist", stackDistName(it.dist));
            if (it.cp != 1) i.set("cp", it.cp);
            if (!it.body.empty()) {
                Json along = Json::array();
                along.push(it.along.x).push(it.along.y).push(it.along.z);
                i.set("body", it.body).set("along", along);
            }
            if (it.dim >= 0) i.set("dim", it.dim);
            if (!it.fit.empty()) i.set("fit", it.fit);
            if (it.general) i.set("general", std::string(1, it.general));
            if (st.vector) {
                if (it.angle) i.set("angle", true).set("axis", Json::array().push(it.axis.x).push(it.axis.y).push(it.axis.z));
                else i.set("dir", Json::array().push(it.dir.x).push(it.dir.y).push(it.dir.z));
            }
            items.push(i);
        }
        j.set("items", items);
        if (st.vector) j.set("vector", true).set("measure", Json::array().push(st.measure.x).push(st.measure.y).push(st.measure.z));
        a.push(j);
    }
    return a;
}

std::vector<ToleranceStack> stacksFromJson(const Json& a) {
    std::vector<ToleranceStack> out;
    auto vec = [](const Json& v, Vec3 fallback) {
        if (!v.isArray() || v.size() < 3) return fallback;
        return Vec3{v[0].asFloat(), v[1].asFloat(), v[2].asFloat()};
    };
    for (const Json& j : a.items()) {
        ToleranceStack st;
        st.name = j["name"].asString(std::string("Stack"));
        st.note = j["note"].asString(std::string());
        if (j["low"].isNumber()) st.hasLow = true, st.low = j["low"].asNumber();
        if (j["high"].isNumber()) st.hasHigh = true, st.high = j["high"].asNumber();
        st.gapFrom = j["gap_from"].asString(std::string());
        st.gapTo = j["gap_to"].asString(std::string());
        st.gapAlong = vec(j["gap_along"], {1, 0, 0});
        st.samples = std::clamp(j["samples"].asInt(100000), 0, 2000000);
        st.vector = j["vector"].asBool(false);
        st.measure = vec(j["measure"], {0, 0, 0});
        for (const Json& i : j["items"].items()) {
            StackItem it;
            it.name = i["name"].asString(std::string("Dimension"));
            it.nominal = i["nominal"].asNumber();
            it.upper = i["upper"].asNumber();
            it.lower = i["lower"].asNumber();
            it.sign = i["sign"].asInt(1) < 0 ? -1 : 1;
            it.sens = i["sens"].asNumber(1.0);
            const std::string d = i["dist"].asString(std::string("normal"));
            it.dist = d == "uniform" ? StackDist::Uniform : d == "triangular" ? StackDist::Triangular : StackDist::Normal;
            it.cp = i["cp"].asNumber(1.0);
            if (!(it.cp > 0)) it.cp = 1;
            it.body = i["body"].asString(std::string());
            it.along = vec(i["along"], {1, 0, 0});
            it.dim = i["dim"].asInt(-1);
            it.fit = i["fit"].asString(std::string());
            const std::string g = i["general"].asString(std::string());
            it.general = g.empty() ? 0 : g[0];
            it.angle = i["angle"].asBool(false);
            it.axis = vec(i["axis"], {0, 0, 1});
            it.dir = vec(i["dir"], {1, 0, 0});
            st.items.push_back(it);
        }
        out.push_back(st);
    }
    return out;
}

} // namespace sicdm

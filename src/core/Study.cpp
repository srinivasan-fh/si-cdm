#include "core/Study.hpp"

#include "core/Motion.hpp"
#include "core/Scene.hpp"
#include "core/Sculpt.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <map>

namespace sicdm {

namespace {

constexpr double kSteel = 7.85e-6; // kg/mm^3

std::string num(double v, int digits = 4) {
    char b[48];
    std::snprintf(b, sizeof b, "%.*g", digits, std::fabs(v) < 1e-12 ? 0.0 : v);
    return b;
}

// First and second derivatives of y over t at each sample: a least-squares quadratic
// through the samples within `half` either side (fewer at the ends).
void derivatives(const std::vector<double>& t, const std::vector<double>& y, int half, std::vector<double>& d1, std::vector<double>& d2) {
    const int n = static_cast<int>(t.size());
    d1.assign(static_cast<std::size_t>(n), 0.0);
    d2.assign(static_cast<std::size_t>(n), 0.0);
    if (n < 3) return;
    for (int i = 0; i < n; ++i) {
        int lo = std::max(0, i - half), hi = std::min(n - 1, i + half);
        while (hi - lo < 2) lo > 0 ? --lo : ++hi; // at least three points
        double S[5] = {0, 0, 0, 0, 0}, R[3] = {0, 0, 0};
        for (int j = lo; j <= hi; ++j) {
            const double x = t[static_cast<std::size_t>(j)] - t[static_cast<std::size_t>(i)], v = y[static_cast<std::size_t>(j)];
            double p = 1;
            for (int k = 0; k < 5; ++k) {
                S[k] += p;
                if (k < 3) R[k] += p * v;
                p *= x;
            }
        }
        // [S0 S1 S2; S1 S2 S3; S2 S3 S4] [a b c] = R: b and c by Cramer's rule.
        const double A[3][3] = {{S[0], S[1], S[2]}, {S[1], S[2], S[3]}, {S[2], S[3], S[4]}};
        auto det3 = [](const double M[3][3]) {
            return M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
                   M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
        };
        const double D = det3(A);
        if (std::fabs(D) < 1e-300) continue;
        double B[3][3], C[3][3];
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) B[r][c] = C[r][c] = A[r][c];
        for (int r = 0; r < 3; ++r) B[r][1] = R[r], C[r][2] = R[r];
        d1[static_cast<std::size_t>(i)] = det3(B) / D;
        d2[static_cast<std::size_t>(i)] = 2.0 * det3(C) / D;
    }
}

// The same with the window chosen from the data: the narrowest whose acceleration a wider one
// barely changes (a meshed cam's facets put kinks in a follower's travel that a narrow
// window turns into spikes). `fixed` > 0: that window. The half-width used comes back.
int smoothDerivatives(const std::vector<double>& t, const std::vector<double>& y, int fixed, std::vector<double>& d1, std::vector<double>& d2) {
    const int n = static_cast<int>(t.size());
    if (fixed > 0) {
        derivatives(t, y, fixed, d1, d2);
        return fixed;
    }
    static const int kHalf[] = {2, 3, 5, 8, 12, 18, 27, 40, 60, 90};
    std::vector<double> a1, a2, b1, b2;
    derivatives(t, y, kHalf[0], a1, a2);
    int used = kHalf[0];
    for (std::size_t k = 1; k < sizeof kHalf / sizeof kHalf[0] && kHalf[k] <= std::max(2, n / 10); ++k) {
        derivatives(t, y, kHalf[k], b1, b2);
        double diff = 0, size = 0, speed = 0;
        for (int i = 0; i < n; ++i) {
            diff += (a2[static_cast<std::size_t>(i)] - b2[static_cast<std::size_t>(i)]) * (a2[static_cast<std::size_t>(i)] - b2[static_cast<std::size_t>(i)]);
            size += b2[static_cast<std::size_t>(i)] * b2[static_cast<std::size_t>(i)];
            speed += b1[static_cast<std::size_t>(i)] * b1[static_cast<std::size_t>(i)];
        }
        // Settled: within 5% (rms), or too small to matter beside its speed over the span (a
        // steady turn's acceleration is nought give or take rounding).
        const double span = t.back() - t.front();
        const double scale = span > 0 ? std::sqrt(speed / n) / span : 0;
        if (diff <= 0.0025 * size || std::sqrt(diff / n) <= 1e-2 * scale || size == 0) break;
        a1.swap(b1), a2.swap(b2);
        used = kHalf[k];
    }
    d1 = std::move(a1), d2 = std::move(a2);
    return used;
}

const Body* bodyNamed(const Scene& scene, const std::string& name, std::size_t* index = nullptr) {
    const auto& b = scene.bodies();
    for (std::size_t i = 0; i < b.size(); ++i)
        if (b[i].name == name) {
            if (index) *index = i;
            return &b[i];
        }
    for (std::size_t i = 0; i < b.size(); ++i) {
        std::string x = b[i].name, y = name;
        for (char& c : x) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        for (char& c : y) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        if (x == y) {
            if (index) *index = i;
            return &b[i];
        }
    }
    return nullptr;
}

// Greatest absolute value and when.
std::pair<double, double> peak(const std::vector<double>& t, const std::vector<double>& v) {
    double best = 0, at = t.empty() ? 0 : t[0];
    for (std::size_t i = 0; i < v.size(); ++i)
        if (std::fabs(v[i]) > std::fabs(best)) best = v[i], at = t[i];
    return {best, at};
}

} // namespace

bool runStudy(const Scene& scene, const StudyRequest& rq, StudyResult& out, std::string* why) {
    auto fail = [&](const std::string& w) {
        if (why) *why = w;
        return false;
    };
    out = StudyResult{};
    const double from = std::max(rq.from, 0.0);
    const double to = rq.to > from ? rq.to : from + suggestedDuration(scene);
    if (!(to > from)) return fail("The study needs a time span");
    double step = rq.step > 0 ? rq.step : (to - from) / 600.0;
    // (A step read as a float makes 4 / 0.01 a hair over 400.)
    const double steps = (to - from) / step;
    std::size_t n = static_cast<std::size_t>(std::ceil(steps - std::max(1e-6, steps * 1e-6))) + 1;
    if (n > 20001) return fail("Too many samples (at most 20000): give a larger STEP");
    if (n < 3) n = 3, step = (to - from) / 2.0;
    for (std::size_t i = 0; i < n; ++i) out.t.push_back(std::min(from + step * static_cast<double>(i), to));
    const int fixed = rq.smooth > 0 ? std::max(1, static_cast<int>(std::lround(rq.smooth / step))) : 0;
    auto smoothNote = [&](const std::string& what, int used) {
        if (fixed == 0 && used > 3)
            out.notes.push_back(what + ": velocity and acceleration smoothed over +-" + num(used * step, 3) +
                                " s (the facets of meshed geometry; a finer mesh needs less, SMOOTH sets it)");
    };

    // The motions asked for (by name), else every enabled one.
    std::vector<const Motion*> motions;
    for (const std::string& name : rq.motions) {
        const Motion* found = nullptr;
        for (const Motion& m : scene.motions()) {
            std::string a = m.name, b = name;
            for (char& c : a) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            for (char& c : b) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            if (a == b) found = &m;
        }
        if (!found) return fail("No motion named " + name);
        motions.push_back(found);
    }
    if (rq.motions.empty() && rq.points.empty())
        for (const Motion& m : scene.motions())
            if (m.enabled) motions.push_back(&m);
    if (motions.empty() && rq.points.empty()) return fail("Nothing to study: add a motion (MOTION), or name one, or a POINT on a body");

    for (const Motion* m : motions) {
        const bool linear = m->kind == MotionKind::Linear;
        std::vector<double> v(n), d1, d2;
        double lastSmooth = 0; // the fit's reach either side (s)
        for (std::size_t i = 0; i < n; ++i) v[i] = motionValue(scene, *m, static_cast<float>(out.t[i]));
        if (m->contact.empty()) {
            // Its own rate is known exactly; the acceleration is the rate's slope.
            std::vector<double> rate(n), unused;
            for (std::size_t i = 0; i < n; ++i) rate[i] = motionRate(scene, *m, static_cast<float>(out.t[i]));
            smoothDerivatives(out.t, rate, fixed, d2, unused);
            // Rounding about a steady rate is no acceleration.
            double top = 0;
            for (double r : rate) top = std::max(top, std::fabs(r));
            const double floor = 1e-6 * top / std::max(to - from, 1e-9);
            for (double& x : d2)
                if (std::fabs(x) < floor) x = 0;
            d1 = std::move(rate);
        } else {
            const int used = smoothDerivatives(out.t, v, fixed, d1, d2);
            smoothNote(m->name, used);
            lastSmooth = used * step;
        }
        const std::string u = linear ? "mm" : "deg";
        out.series.push_back({m->name + (linear ? " travel" : " angle"), u, linear ? "travel" : "angle", v});
        out.series.push_back({m->name + " velocity", u + "/s", "velocity", d1});
        out.series.push_back({m->name + " acceleration", u + "/s2", "acceleration", d2});
        const auto [lo, hi] = std::minmax_element(v.begin(), v.end());
        const auto pv = peak(out.t, d1), pa = peak(out.t, d2);
        out.notes.push_back(m->name + ": " + (linear ? "travel " : "angle ") + num(*lo) + " to " + num(*hi) + " " + u + "; velocity peak " +
                            num(pv.first) + " " + u + "/s at " + num(pv.second) + " s; acceleration peak " + num(pa.first) + " " + u +
                            "/s2 at " + num(pa.second) + " s");
        // A cam follower on its spring: the force at the contact, and lift-off.
        if (!m->contact.empty() && linear && (m->spring > 0 || m->preload > 0 || m->mass > 0)) {
            double mass = m->mass;
            if (!(mass > 0))
                for (const Body& b : scene.bodies())
                    if (m->moves(b.name) && b.name != m->contact)
                        mass += b.physics.mass > 0 ? b.physics.mass : std::fabs(meshVolume(b.mesh)) * kSteel;
            const double low = *lo;
            std::vector<double> f(n);
            for (std::size_t i = 0; i < n; ++i) f[i] = mass * d2[i] * 1e-3 + m->preload + m->spring * (v[i] - low);
            out.series.push_back({m->name + " contact force", "N", "force", f});
            // Lift-off: where the spring can't hold it to the cam.
            double worst = 0, needK = 0, needF = 0;
            std::vector<std::pair<double, double>> off; // the spans it is off the cam
            for (std::size_t i = 0; i < n; ++i) {
                if (f[i] < 0) {
                    if (i == 0 || f[i - 1] >= 0) off.push_back({out.t[i], out.t[i]});
                    off.back().second = out.t[i];
                    worst = std::min(worst, f[i]);
                }
                const double inertia = -mass * d2[i] * 1e-3; // what the spring must supply
                const double lift = v[i] - low;
                if (lift > 1e-6) needK = std::max(needK, (inertia - m->preload) / lift);
                else needF = std::max(needF, inertia);
            }
            // A gap shorter than the fit's reach (or 2% of the span) is the facets' noise about
            // zero, not the follower landing back on the cam.
            const double bridge = std::max({out.t.size() > 1 ? 3 * (out.t[1] - out.t[0]) : 0.0, 0.02 * (to - from), lastSmooth});
            for (std::size_t k = 1; k < off.size();)
                if (off[k].first - off[k - 1].second <= bridge) {
                    off[k - 1].second = off[k].second;
                    off.erase(off.begin() + static_cast<std::ptrdiff_t>(k));
                } else {
                    ++k;
                }
            if (off.empty()) {
                const auto [flo, fhi] = std::minmax_element(f.begin(), f.end());
                out.notes.push_back(m->name + ": contact force " + num(*flo) + " to " + num(*fhi) + " N (" + num(mass) + " kg moving); it stays on " +
                                    m->contact);
            } else {
                std::string need = "a spring of at least " + num(std::max(needK, 0.0)) + " N/mm with this preload";
                if (needF > m->preload) need += ", and a preload of at least " + num(needF) + " N";
                std::string when = "between " + num(off[0].first) + " and " + num(off[0].second) + " s";
                if (off.size() > 1) when += " and " + std::to_string(off.size() - 1) + (off.size() == 2 ? " more time" : " more times");
                out.notes.push_back(m->name + ": lifts off " + m->contact + " " + when + " (contact force down to " +
                                    num(worst) + " N, " + num(mass) + " kg moving): it needs " + need);
            }
        }
    }

    for (const StudyRequest::Point& p : rq.points) {
        std::size_t bi = 0;
        const Body* b = bodyNamed(scene, p.body, &bi);
        if (!b) return fail("No body named " + p.body);
        const Vec3 at = p.given ? p.at : b->bounds().center();
        std::vector<double> x(n), y(n), z(n);
        for (std::size_t i = 0; i < n; ++i) {
            const Vec3 w = bodyMotionTransform(scene, bi, static_cast<float>(out.t[i])).transformPoint(at);
            x[i] = w.x, y[i] = w.y, z[i] = w.z;
        }
        std::vector<double> dx, ddx, dy, ddy, dz, ddz;
        const int used = std::max({smoothDerivatives(out.t, x, fixed, dx, ddx), smoothDerivatives(out.t, y, fixed, dy, ddy),
                                   smoothDerivatives(out.t, z, fixed, dz, ddz)});
        std::vector<double> speed(n), acc(n);
        for (std::size_t i = 0; i < n; ++i) {
            speed[i] = std::sqrt(dx[i] * dx[i] + dy[i] * dy[i] + dz[i] * dz[i]);
            acc[i] = std::sqrt(ddx[i] * ddx[i] + ddy[i] * ddy[i] + ddz[i] * ddz[i]);
        }
        const std::string label = b->name + " (" + num(at.x) + ", " + num(at.y) + ", " + num(at.z) + ")";
        smoothNote(label, used);
        out.series.push_back({label + " x", "mm", "position", x});
        out.series.push_back({label + " y", "mm", "position", y});
        out.series.push_back({label + " z", "mm", "position", z});
        out.series.push_back({label + " speed", "mm/s", "velocity", speed});
        out.series.push_back({label + " acceleration", "mm/s2", "acceleration", acc});
        const auto ps = peak(out.t, speed), pa = peak(out.t, acc);
        double path = 0;
        for (std::size_t i = 1; i < n; ++i)
            path += std::sqrt((x[i] - x[i - 1]) * (x[i] - x[i - 1]) + (y[i] - y[i - 1]) * (y[i] - y[i - 1]) + (z[i] - z[i - 1]) * (z[i] - z[i - 1]));
        out.notes.push_back(label + ": path " + num(path) + " mm; speed peak " + num(ps.first) + " mm/s at " + num(ps.second) +
                            " s; acceleration peak " + num(pa.first) + " mm/s2 at " + num(pa.second) + " s");
    }
    return true;
}

std::vector<std::string> studySummary(const StudyResult& r) {
    std::vector<std::string> out;
    for (const StudySeries& s : r.series) {
        if (s.v.empty()) continue;
        std::size_t lo = 0, hi = 0;
        for (std::size_t i = 1; i < s.v.size(); ++i) {
            if (s.v[i] < s.v[lo]) lo = i;
            if (s.v[i] > s.v[hi]) hi = i;
        }
        out.push_back(s.name + " (" + s.unit + "): " + num(s.v[lo]) + " at " + num(r.t[lo]) + " s to " + num(s.v[hi]) + " at " + num(r.t[hi]) + " s");
    }
    return out;
}

std::string studyCsv(const StudyResult& r) {
    std::string s = "t (s)";
    for (const StudySeries& q : r.series) {
        std::string name = q.name + " (" + q.unit + ")";
        if (name.find_first_of(",\"") != std::string::npos) {
            std::string e = "\"";
            for (char c : name) e += c == '"' ? std::string("\"\"") : std::string(1, c);
            name = e + "\"";
        }
        s += "," + name;
    }
    s += "\n";
    char b[48];
    for (std::size_t i = 0; i < r.t.size(); ++i) {
        std::snprintf(b, sizeof b, "%.6g", r.t[i]);
        s += b;
        for (const StudySeries& q : r.series) {
            std::snprintf(b, sizeof b, ",%.9g", i < q.v.size() ? q.v[i] : 0.0);
            s += b;
        }
        s += "\n";
    }
    return s;
}

namespace {

// A tick step of 1, 2 or 5 times a power of ten giving about `count` ticks over `span`.
double niceStep(double span, int count) {
    if (!(span > 0)) return 1;
    const double raw = span / count, p = std::pow(10.0, std::floor(std::log10(raw)));
    const double f = raw / p;
    return (f < 1.5 ? 1 : f < 3.5 ? 2 : f < 7.5 ? 5 : 10) * p;
}

std::string esc(const std::string& s) {
    std::string o;
    for (char c : s) {
        if (c == '<') o += "&lt;";
        else if (c == '>') o += "&gt;";
        else if (c == '&') o += "&amp;";
        else if (c == '"') o += "&quot;";
        else o += c;
    }
    return o;
}

} // namespace

std::string studySvg(const StudyResult& r, const std::string& title) {
    // Panels by group and unit, in a fixed order.
    static const char* const kOrder[] = {"travel", "angle", "position", "velocity", "acceleration", "force"};
    std::vector<std::pair<std::string, std::vector<const StudySeries*>>> panels;
    for (const char* g : kOrder)
        for (const StudySeries& s : r.series) {
            if (s.group != g) continue;
            const std::string key = std::string(g) + " (" + s.unit + ")";
            auto it = std::find_if(panels.begin(), panels.end(), [&](const auto& p) { return p.first == key; });
            if (it == panels.end()) panels.push_back({key, {}}), it = panels.end() - 1;
            it->second.push_back(&s);
        }
    const double W = 960, left = 80, right = 24, top = 48, panelH = 200, gapH = 56;
    const double H = top + static_cast<double>(panels.size()) * (panelH + gapH) + 16;
    static const char* const kColours[] = {"#2f6fd6", "#d9622b", "#2a9d5b", "#a64fc4", "#c7392f", "#1f9bb0", "#8a6d1f", "#555555"};
    std::string s;
    char b[512];
    std::snprintf(b, sizeof b, "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"%.0f\" height=\"%.0f\" viewBox=\"0 0 %.0f %.0f\" font-family=\"Helvetica, Arial, sans-serif\">\n", W, H, W, H);
    s += b;
    s += "<rect width=\"100%\" height=\"100%\" fill=\"#ffffff\"/>\n";
    std::snprintf(b, sizeof b, "<text x=\"%.0f\" y=\"28\" font-size=\"18\" font-weight=\"bold\" fill=\"#222\">%s</text>\n", left, esc(title).c_str());
    s += b;
    const double t0 = r.t.empty() ? 0 : r.t.front(), t1 = r.t.empty() ? 1 : r.t.back();
    const double pw = W - left - right;
    for (std::size_t k = 0; k < panels.size(); ++k) {
        const double y0 = top + static_cast<double>(k) * (panelH + gapH) + 20; // the plot's top
        double lo = 1e300, hi = -1e300;
        for (const StudySeries* q : panels[k].second)
            for (double v : q->v) lo = std::min(lo, v), hi = std::max(hi, v);
        if (!(hi > lo)) lo -= 1, hi += 1;
        const double pad = 0.05 * (hi - lo);
        lo -= pad, hi += pad;
        const double ty = niceStep(hi - lo, 5), tx = niceStep(t1 - t0, 10);
        auto X = [&](double t) { return left + (t - t0) / (t1 - t0 > 0 ? t1 - t0 : 1) * pw; };
        auto Y = [&](double v) { return y0 + (hi - v) / (hi - lo) * panelH; };
        std::snprintf(b, sizeof b, "<text x=\"%.0f\" y=\"%.0f\" font-size=\"13\" fill=\"#333\">%s</text>\n", left, y0 - 6, esc(panels[k].first).c_str());
        s += b;
        std::snprintf(b, sizeof b, "<rect x=\"%.1f\" y=\"%.1f\" width=\"%.1f\" height=\"%.1f\" fill=\"#fafafa\" stroke=\"#bbb\"/>\n", left, y0, pw, panelH);
        s += b;
        // Lift-off shaded on the force panel.
        if (panels[k].first.rfind("force", 0) == 0)
            for (const StudySeries* q : panels[k].second)
                for (std::size_t i = 0; i + 1 < q->v.size(); ++i)
                    if (q->v[i] < 0 || q->v[i + 1] < 0) {
                        std::snprintf(b, sizeof b, "<rect x=\"%.2f\" y=\"%.1f\" width=\"%.2f\" height=\"%.1f\" fill=\"#f3c4c0\" opacity=\"0.6\"/>\n", X(r.t[i]), y0,
                                      X(r.t[i + 1]) - X(r.t[i]), panelH);
                        s += b;
                    }
        for (double v = std::ceil(lo / ty) * ty; v <= hi + 1e-9 * ty; v += ty) {
            std::snprintf(b, sizeof b, "<line x1=\"%.1f\" y1=\"%.2f\" x2=\"%.1f\" y2=\"%.2f\" stroke=\"%s\" stroke-width=\"1\"/>\n", left, Y(v), left + pw, Y(v),
                          std::fabs(v) < 1e-9 * ty ? "#888" : "#e3e3e3");
            s += b;
            std::snprintf(b, sizeof b, "<text x=\"%.1f\" y=\"%.2f\" font-size=\"11\" fill=\"#555\" text-anchor=\"end\">%s</text>\n", left - 6, Y(v) + 4,
                          num(std::fabs(v) < 1e-9 * ty ? 0.0 : v, 6).c_str());
            s += b;
        }
        for (double t = std::ceil(t0 / tx) * tx; t <= t1 + 1e-9 * tx; t += tx) {
            std::snprintf(b, sizeof b, "<line x1=\"%.2f\" y1=\"%.1f\" x2=\"%.2f\" y2=\"%.1f\" stroke=\"#ececec\"/>\n", X(t), y0, X(t), y0 + panelH);
            s += b;
            std::snprintf(b, sizeof b, "<text x=\"%.2f\" y=\"%.1f\" font-size=\"11\" fill=\"#555\" text-anchor=\"middle\">%s</text>\n", X(t), y0 + panelH + 14,
                          num(t, 6).c_str());
            s += b;
        }
        std::size_t c = 0;
        double legendX = left + 8;
        for (const StudySeries* q : panels[k].second) {
            const char* colour = kColours[c++ % (sizeof kColours / sizeof kColours[0])];
            s += "<polyline fill=\"none\" stroke-width=\"1.6\" stroke=\"";
            s += colour;
            s += "\" points=\"";
            for (std::size_t i = 0; i < q->v.size() && i < r.t.size(); ++i) {
                std::snprintf(b, sizeof b, "%.2f,%.2f ", X(r.t[i]), Y(q->v[i]));
                s += b;
            }
            s += "\"/>\n";
            std::snprintf(b, sizeof b, "<rect x=\"%.1f\" y=\"%.1f\" width=\"12\" height=\"3\" fill=\"%s\"/><text x=\"%.1f\" y=\"%.1f\" font-size=\"11\" fill=\"#333\">%s</text>\n",
                          legendX, y0 + panelH + 26, colour, legendX + 16, y0 + panelH + 30, esc(q->name).c_str());
            s += b;
            legendX += 24 + 6.2 * static_cast<double>(q->name.size());
        }
    }
    s += "<text x=\"";
    std::snprintf(b, sizeof b, "%.0f\" y=\"%.0f\" font-size=\"11\" fill=\"#777\" text-anchor=\"end\">time (s)</text>\n</svg>\n", W - right, H - 4);
    s += b;
    return s;
}

} // namespace sicdm

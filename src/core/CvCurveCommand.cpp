#include "core/CvCurveCommand.hpp"

#include "core/Command.hpp"
#include "core/CurveOnSurface.hpp"
#include "core/CvCurve.hpp"
#include "core/Document.hpp"
#include "core/Sculpt.hpp"
#include "core/Surfacing.hpp"
#include "core/Sketch.hpp"
#include "core/Units.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <optional>

namespace sicdm {

namespace {

std::string lowerOf(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

int curveIndexOf(const Document& d, const std::string& n) {
    const auto& c = d.scene.sketch().curves();
    for (std::size_t i = 0; i < c.size(); ++i)
        if (c[i].name == n) return static_cast<int>(i);
    for (std::size_t i = 0; i < c.size(); ++i)
        if (lowerOf(c[i].name) == lowerOf(n)) return static_cast<int>(i);
    return -1;
}

std::string pointText(const DVec3& p) { return lengthsText({p.x, p.y, p.z}, ", "); }
std::string dirText(const DVec3& d) {
    char b[96];
    std::snprintf(b, sizeof b, "%g,%g,%g", d.x, d.y, d.z);
    return b;
}

// A curve's name starting at a[i]: quoted, or the longest run of words naming a curve
// (stopping at a word `stop` accepts). Moves i past it; -1 when none.
template <class Stop>
int curveAt(const Document& doc, const std::vector<std::string>& a, const std::vector<bool>& q, std::size_t& i, Stop stop) {
    if (i >= a.size()) return -1;
    if (i < q.size() && q[i]) {
        const int c = curveIndexOf(doc, a[i]);
        if (c >= 0) ++i;
        return c;
    }
    std::string name;
    int best = -1;
    std::size_t end = i;
    for (std::size_t j = i; j < a.size() && !(j < q.size() && q[j]) && !stop(lowerOf(a[j])); ++j) {
        name += (name.empty() ? "" : " ") + a[j];
        if (const int c = curveIndexOf(doc, name); c >= 0) best = c, end = j + 1;
    }
    if (best >= 0) i = end;
    return best;
}

// Options DEGREE d, SPANS n, NAME n, CONSTRUCTION among a[from..].
struct Options {
    std::optional<int> degree, spans;
    std::string name;
    bool construction = false;
};
bool readOption(const std::vector<std::string>& a, std::size_t& i, Options& o, std::string& err) {
    const std::string w = lowerOf(a[i]);
    auto intAfter = [&](int lo, int hi) -> std::optional<int> {
        if (i + 1 >= a.size()) return std::nullopt;
        const auto v = parseNumber(a[i + 1]);
        if (!v || std::floor(*v) != *v || *v < static_cast<float>(lo) || *v > static_cast<float>(hi)) return std::nullopt;
        ++i;
        return static_cast<int>(*v);
    };
    if (w == "degree" || w == "deg") {
        o.degree = intAfter(1, 7);
        if (!o.degree) err = "DEGREE needs a whole number from 1 to 7";
        return true;
    }
    if (w == "spans") {
        o.spans = intAfter(1, 500);
        if (!o.spans) err = "SPANS needs a whole number from 1 to 500";
        return true;
    }
    if (w == "name") {
        if (i + 1 >= a.size()) err = "NAME needs a name";
        else o.name = a[++i];
        return true;
    }
    if (w == "construction" || w == "constr") {
        o.construction = true;
        return true;
    }
    return false;
}

bool isOptionWord(const std::string& w) { return w == "degree" || w == "deg" || w == "spans" || w == "name" || w == "construction" || w == "constr"; }

// Any sketch curve as a B-spline: a CV curve as it is, anything else fitted to its line
// (degree 5, finely: its ends' tangents and curvature come out close to the curve's own).
bool asSpline(const SketchCurve& c, BSplineCurve& b, std::string& why) {
    if (c.kind == CurveKind::Nurbs) {
        b = cvCurveSpline(c);
        return validCvCurve(b, &why);
    }
    std::vector<DVec3> pts;
    for (const Vec3& p : c.worldStrip(256)) pts.push_back(DVec3(p));
    pts.erase(std::unique(pts.begin(), pts.end(), [](const DVec3& x, const DVec3& y) { return dlength(x - y) < 1e-9; }), pts.end());
    if (pts.size() < 2) {
        why = c.name + " has no length";
        return false;
    }
    if (c.kind == CurveKind::Line || (c.kind == CurveKind::Polyline && pts.size() == 2)) return fitCurve(pts, 1, 1, b, &why);
    const int spans = std::clamp(static_cast<int>(pts.size()) / 8, 1, 40);
    const int degree = std::min(5, static_cast<int>(pts.size()) - spans);
    if (degree < 1) return fitCurve({pts.front(), pts.back()}, 1, 1, b, &why);
    return fitCurve(pts, degree, spans, b, &why);
}

std::string describe(const SketchCurve& c) {
    const BSplineCurve b = cvCurveSpline(c);
    std::string s = c.name + ": degree " + std::to_string(b.degree) + ", " + std::to_string(b.cp.size()) + " CVs, " +
                    std::to_string(static_cast<int>(b.cp.size()) - b.degree) + " spans, length " + lengthText(curveLength(b));
    if (!c.knots.empty()) s += ", non-uniform knots";
    double kmax = 0;
    for (int i = 0; i <= 200; ++i) kmax = std::max(kmax, curveLocal(b, b.t0() + (b.t1() - b.t0()) * i / 200).curvature());
    if (kmax > 1e-9) s += ", tightest radius " + lengthText(1.0 / kmax);
    else s += ", straight";
    return s;
}

} // namespace

CommandOutput runCvCurveCommand(Document& doc, const std::vector<std::string>& a, const std::vector<bool>& quoted) {
    CommandOutput out;
    Sketch& sketch = doc.scene.sketch();
    const std::string sub = a.empty() || (!quoted.empty() && quoted[0]) ? std::string() : lowerOf(a[0]);
    auto endWord = [](const std::string& w) { return w == "start" || w == "end" || w == "to" || w == "g0" || w == "g1" || w == "g2" || w == "g3"; };
    auto isNum = [](const std::string& w) { return parseNumber(w).has_value(); };

    auto finishNew = [&](BSplineCurve b, const Options& o, const std::string& said) {
        SketchCurve c;
        c.kind = CurveKind::Nurbs;
        c.plane = SketchPlane::Top;
        c.construction = o.construction;
        if (!o.name.empty()) {
            if (curveIndexOf(doc, o.name) >= 0) return out.fail("There is a curve named " + o.name + " already");
            c.name = o.name;
        }
        setCvCurve(c, b);
        const SketchCurve& added = sketch.add(std::move(c));
        doc.selectedCurve = static_cast<int>(sketch.curves().size()) - 1;
        doc.selectedBody = doc.selectedImage = -1;
        doc.dirty = true;
        out.say(said + " " + describe(added));
        return out;
    };

    // An existing CV curve by name (or the selected one) at a[i].
    auto target = [&](std::size_t& i, std::string& err) -> int {
        int ci = curveAt(doc, a, quoted, i, [&](const std::string& w) { return isNum(w) || endWord(w) || isOptionWord(w); });
        if (ci < 0) {
            if (i < a.size() && !isNum(a[i]) && !endWord(lowerOf(a[i])) && !isOptionWord(lowerOf(a[i]))) {
                err = "No sketch curve named " + a[i];
                return -1;
            }
            ci = doc.selectedCurve;
        }
        if (ci < 0 || ci >= static_cast<int>(sketch.curves().size())) {
            err = "Which CV curve? Name it, or select it";
            return -1;
        }
        if (sketch.curves()[static_cast<std::size_t>(ci)].kind != CurveKind::Nurbs) {
            err = sketch.curves()[static_cast<std::size_t>(ci)].name + " isn't a CV curve (CVCURVE FROM " + sketch.curves()[static_cast<std::size_t>(ci)].name + " makes one from it)";
            return -1;
        }
        return ci;
    };
    auto store = [&](int ci, const BSplineCurve& b, const std::string& said) {
        SketchCurve& c = sketch.curves()[static_cast<std::size_t>(ci)];
        setCvCurve(c, b);
        sketch.touch();
        doc.dirty = true;
        doc.selectedCurve = ci;
        out.say(said + "  " + describe(c));
        return out;
    };

    if (sub.empty() || (sub != "fit" && sub != "from" && sub != "set" && sub != "move" && sub != "insert" && sub != "rebuild" && sub != "match" &&
                        sub != "continuity" && sub != "info" && sub != "blend" && sub != "project")) {
        // CVCURVE x,y,z x,y,z ... [DEGREE d]
        std::vector<DVec3> cvs;
        Options o;
        for (std::size_t i = 0; i < a.size(); ++i) {
            std::string err;
            if (!(i < quoted.size() && quoted[i]) && readOption(a, i, o, err)) {
                if (!err.empty()) return out.fail(err);
                continue;
            }
            const bool rel = !a[i].empty() && a[i][0] == '@';
            const auto v = parseVec3(a[i]);
            if (!v) return out.fail("Not a CV: " + a[i] + "  (CVCURVE x,y,z x,y,z ... [DEGREE d]; @dx,dy,dz from the CV before)");
            if (rel && cvs.empty()) return out.fail("The first CV can't be relative: x,y,z");
            cvs.push_back(rel ? cvs.back() + DVec3(*v) : DVec3(*v));
        }
        if (cvs.size() < 2) return out.fail("CVCURVE x,y,z x,y,z ... [DEGREE d] [NAME n]: two CVs or more (a degree d curve needs d + 1)");
        for (std::size_t i = 1; i < cvs.size(); ++i)
            if (dlength(cvs[i] - cvs[i - 1]) < 1e-9) return out.fail("CVs " + std::to_string(i) + " and " + std::to_string(i + 1) + " are in the same place");
        BSplineCurve b;
        b.degree = o.degree.value_or(std::min(3, static_cast<int>(cvs.size()) - 1));
        if (static_cast<int>(cvs.size()) <= b.degree)
            return out.fail("A degree " + std::to_string(b.degree) + " curve needs " + std::to_string(b.degree + 1) + " CVs or more (" + std::to_string(cvs.size()) + " given)");
        b.cp = cvs;
        b.knots = clampedKnots(cvs.size(), b.degree);
        return finishNew(b, o, "Added");
    }

    if (sub == "fit") {
        // CVCURVE FIT p1 p2 ... [DEGREE d] [SPANS n]
        std::vector<DVec3> pts;
        Options o;
        for (std::size_t i = 1; i < a.size(); ++i) {
            std::string err;
            if (readOption(a, i, o, err)) {
                if (!err.empty()) return out.fail(err);
                continue;
            }
            const auto v = parseVec3(a[i]);
            if (!v) return out.fail("Not a point: " + a[i] + "  (CVCURVE FIT x,y,z x,y,z ... [DEGREE d] [SPANS n])");
            pts.push_back(DVec3(*v));
        }
        const int degree = o.degree.value_or(3);
        const int spans = o.spans.value_or(std::max(1, std::min(static_cast<int>(pts.size()) - degree, 8)));
        BSplineCurve b;
        std::string why;
        if (pts.size() < 2) return out.fail("CVCURVE FIT needs two points or more");
        if (!fitCurve(pts, degree, spans, b, &why)) return out.fail("Can't fit: " + why);
        double worst = 0;
        for (const DVec3& p : pts) {
            double best = 1e300;
            for (int k = 0; k <= 400; ++k) best = std::min(best, dlength(b.eval(b.t0() + (b.t1() - b.t0()) * k / 400) - p));
            worst = std::max(worst, best);
        }
        return finishNew(b, o, "Fitted (points within " + lengthText(worst) + ")");
    }

    if (sub == "from") {
        // CVCURVE FROM curve [DEGREE d] [SPANS n]
        std::size_t i = 1;
        const int ci = curveAt(doc, a, quoted, i, [&](const std::string& w) { return isOptionWord(w); });
        if (ci < 0) return out.fail(i < a.size() ? "No sketch curve named " + a[i] : "CVCURVE FROM curve [DEGREE d] [SPANS n]");
        Options o;
        for (; i < a.size(); ++i) {
            std::string err;
            if (!readOption(a, i, o, err)) return out.fail("Not an option: " + a[i] + "  (DEGREE d, SPANS n, NAME n)");
            if (!err.empty()) return out.fail(err);
        }
        const SketchCurve& src = sketch.curves()[static_cast<std::size_t>(ci)];
        BSplineCurve s, b;
        std::string why;
        if (!asSpline(src, s, why)) return out.fail("Can't make a CV curve from " + src.name + ": " + why);
        const int degree = o.degree.value_or(std::min(3, std::max(1, s.degree)));
        const int spans = o.spans.value_or(std::clamp(static_cast<int>(s.cp.size()) - s.degree, 1, 12));
        double dev = 0;
        if (!rebuildCurve(s, degree, spans, b, &dev, &why)) return out.fail("Can't make a CV curve from " + src.name + ": " + why);
        return finishNew(b, o, "From " + src.name + " (within " + lengthText(dev) + "):");
    }

    std::string err;
    std::size_t i = 1;

    // A sketch curve as a B-spline at one of its ends, run into that end.
    auto endOf = [&](std::size_t& at, const std::string& usage, int& ci, bool& atEnd, CurveLocal& into) -> bool {
        ci = curveAt(doc, a, quoted, at, [&](const std::string& w) { return endWord(w); });
        if (ci < 0) {
            err = at < a.size() ? "No sketch curve named " + a[at] : usage;
            return false;
        }
        if (at >= a.size() || (lowerOf(a[at]) != "start" && lowerOf(a[at]) != "end")) {
            err = usage;
            return false;
        }
        atEnd = lowerOf(a[at++]) == "end";
        BSplineCurve s;
        if (!asSpline(sketch.curves()[static_cast<std::size_t>(ci)], s, err)) return false;
        const BSplineCurve r = atEnd ? s : reverseCurve(s);
        into = curveLocal(r, r.t1());
        return true;
    };

    if (sub == "blend") {
        // CVCURVE BLEND A START|END TO B START|END [Gk [Gk]] [TENSION t] [NAME n]
        const std::string usage = "CVCURVE BLEND curve START|END TO other START|END [G0|G1|G2|G3 [G0|G1|G2|G3]] [TENSION t] [NAME n]";
        int ca = -1, cb = -1;
        bool ea = false, eb = false;
        CurveLocal ia, ib;
        if (!endOf(i, usage, ca, ea, ia)) return out.fail(err);
        if (i < a.size() && lowerOf(a[i]) == "to") ++i;
        if (!endOf(i, usage, cb, eb, ib)) return out.fail(err);
        if (ca == cb && ea == eb) return out.fail("A blend joins two different ends");
        std::vector<int> levels;
        double tension = 1.0;
        Options o;
        for (; i < a.size(); ++i) {
            const std::string w = lowerOf(a[i]);
            if (w.size() == 2 && w[0] == 'g' && w[1] >= '0' && w[1] <= '3') {
                if (levels.size() == 2) return out.fail("Two continuities at most: one for each end (" + usage + ")");
                levels.push_back(w[1] - '0');
            } else if (w == "tension") {
                const auto t = i + 1 < a.size() ? parseNumber(a[i + 1]) : std::nullopt;
                if (!t) return out.fail("TENSION needs a number (1: the usual; more carries the ends' directions further)");
                tension = *t;
                ++i;
            } else if (!readOption(a, i, o, err)) {
                return out.fail("Not understood: " + a[i] + "  (" + usage + ")");
            } else if (!err.empty()) {
                return out.fail(err);
            }
        }
        const int la = levels.empty() ? 2 : levels[0], lb = levels.size() < 2 ? la : levels[1];
        BSplineCurve b;
        if (!blendCurves(ia, ib, la, lb, tension, b, &err)) return out.fail("Can't blend: " + err);
        const Continuity k0 = continuityBetween(ia, curveLocal(b, b.t0()));
        const BSplineCurve rb = reverseCurve(b);
        const Continuity k1 = continuityBetween(ib, curveLocal(rb, rb.t0()));
        const std::string na = sketch.curves()[static_cast<std::size_t>(ca)].name, nb = sketch.curves()[static_cast<std::size_t>(cb)].name;
        return finishNew(b, o, "Blended " + na + (ea ? " end" : " start") + " (" + continuityText(k0) + ") to " + nb + (eb ? " end" : " start") + " (" +
                                   continuityText(k1) + "):");
    }

    if (sub == "project") {
        // CVCURVE PROJECT curve ONTO body [DIR x,y,z | NORMAL] [TOLERANCE t] [NAME n]
        const std::string usage = "CVCURVE PROJECT curve ONTO body [DIR x,y,z | NORMAL] [TOLERANCE t] [NAME n]";
        const int ci = curveAt(doc, a, quoted, i, [&](const std::string& w) { return w == "onto" || w == "on"; });
        if (ci < 0) return out.fail(i < a.size() && lowerOf(a[i]) != "onto" ? "No sketch curve named " + a[i] : usage);
        if (i >= a.size() || (lowerOf(a[i]) != "onto" && lowerOf(a[i]) != "on")) return out.fail(usage);
        ++i;
        const auto& bodies = doc.scene.bodies();
        auto bodyNamed = [&](const std::string& n) {
            for (std::size_t k = 0; k < bodies.size(); ++k)
                if (lowerOf(bodies[k].name) == lowerOf(n)) return static_cast<int>(k);
            return -1;
        };
        int bi = -1;
        {
            std::string name;
            std::size_t end = i;
            if (i < a.size() && i < quoted.size() && quoted[i]) {
                bi = bodyNamed(a[i]);
                end = i + 1;
            } else
                for (std::size_t j = i; j < a.size(); ++j) {
                    const std::string w = lowerOf(a[j]);
                    if (w == "dir" || w == "normal" || w == "tolerance" || w == "tol" || isOptionWord(w)) break;
                    name += (name.empty() ? "" : " ") + a[j];
                    if (const int k = bodyNamed(name); k >= 0) bi = k, end = j + 1;
                }
            if (bi < 0) return out.fail(i < a.size() ? "No body named " + a[i] : usage);
            i = end;
        }
        std::optional<DVec3> dir;
        double tol = 0.01;
        Options o;
        for (; i < a.size(); ++i) {
            const std::string w = lowerOf(a[i]);
            if (w == "dir" || w == "direction") {
                const auto v = i + 1 < a.size() ? parseVec3(a[i + 1]) : std::nullopt;
                if (!v || length(*v) < 1e-9f) return out.fail("DIR needs a direction x,y,z");
                dir = DVec3(*v);
                ++i;
            } else if (w == "normal") {
                dir.reset();
            } else if (w == "tolerance" || w == "tol") {
                const auto t = i + 1 < a.size() ? parseLength(a[i + 1]) : std::nullopt;
                if (!t || *t <= 0) return out.fail("TOLERANCE needs a length above 0");
                tol = *t;
                ++i;
            } else if (!readOption(a, i, o, err)) {
                return out.fail("Not understood: " + a[i] + "  (" + usage + ")");
            } else if (!err.empty()) {
                return out.fail(err);
            }
        }
        const SketchCurve& src = sketch.curves()[static_cast<std::size_t>(ci)];
        const Body& body = bodies[static_cast<std::size_t>(bi)];
        BSplineSurface exact;
        const bool hasExact = bodySurface(body, exact);
        const SurfaceTarget onto(worldMesh(body), hasExact ? &exact : nullptr);
        std::vector<DVec3> line;
        for (const Vec3& q : src.worldStrip(256)) line.push_back(DVec3(q));
        if (line.size() < 2) return out.fail(src.name + " has no length");
        const std::vector<DVec3> samples = resample(line, 120, false);
        std::vector<DVec3> on;
        for (const DVec3& p : samples) {
            if (dir) {
                const auto hit = onto.alongDirection(p, *dir);
                if (!hit) return out.fail(src.name + " runs off " + body.name + " near " + pointText(p) + " (along that direction nothing is there)");
                on.push_back(*hit);
            } else {
                on.push_back(onto.nearest(p));
            }
        }
        // Fitted with more spans until it stays on the surface and through the points.
        BSplineCurve fit;
        double devSurface = 0, devPoints = 0;
        for (int spans = 4;; spans *= 2) {
            if (!fitCurve(on, o.degree.value_or(3), std::min(spans, static_cast<int>(on.size()) - o.degree.value_or(3)), fit, &err)) return out.fail("Can't fit the projected curve: " + err);
            devSurface = 0;
            for (int k = 0; k <= 100; ++k) devSurface = std::max(devSurface, onto.distance(fit.eval(fit.t0() + (fit.t1() - fit.t0()) * k / 100)));
            devPoints = pointsDeviation(on, fit);
            if ((devSurface <= tol && devPoints <= tol) || spans >= 64) break;
        }
        std::string said = "Projected " + src.name + " onto " + body.name + (dir ? " along " + dirText(*dir) : std::string(" (nearest points)")) +
                           ": within " + lengthText(std::max(devSurface, devPoints)) + " of the " + (hasExact ? "exact surface" : "body's triangles") + ".";
        if (std::max(devSurface, devPoints) > tol) said += " That is more than the tolerance " + lengthText(tol) + ": the surface turns too sharply for 64 spans.";
        return finishNew(fit, o, said);
    }

    if (sub == "info") {
        if (a.size() == 1 && doc.selectedCurve < 0) {
            int n = 0;
            for (const SketchCurve& c : sketch.curves())
                if (c.kind == CurveKind::Nurbs) out.say(describe(c)), ++n;
            if (!n) out.say("No CV curves (CVCURVE x,y,z x,y,z ... makes one)");
            return out;
        }
        const int ci = target(i, err);
        if (ci < 0) return out.fail(err);
        const SketchCurve& c = sketch.curves()[static_cast<std::size_t>(ci)];
        out.say(describe(c));
        const BSplineCurve b = cvCurveSpline(c);
        for (std::size_t k = 0; k < b.cp.size(); ++k) out.say("  CV " + std::to_string(k + 1) + ": " + pointText(b.cp[k]));
        return out;
    }

    const int ci = target(i, err);
    if (ci < 0) return out.fail(err);
    SketchCurve& c = sketch.curves()[static_cast<std::size_t>(ci)];
    BSplineCurve b = cvCurveSpline(c);

    if (sub == "set" || sub == "move") {
        const auto n = i < a.size() ? parseNumber(a[i]) : std::nullopt;
        if (!n || std::floor(*n) != *n || *n < 1 || *n > static_cast<double>(b.cp.size()) || i + 1 >= a.size())
            return out.fail("CVCURVE " + std::string(sub == "set" ? "SET curve n x,y,z" : "MOVE curve n dx,dy,dz") + " (n from 1 to " + std::to_string(b.cp.size()) + ")");
        const auto v = parseVec3(!a[i + 1].empty() && a[i + 1][0] == '@' ? a[i + 1].substr(1) : a[i + 1]);
        if (!v) return out.fail("Not a point: " + a[i + 1]);
        const std::size_t k = static_cast<std::size_t>(*n) - 1;
        b.cp[k] = sub == "set" ? DVec3(*v) : b.cp[k] + DVec3(*v);
        for (std::size_t j = 0; j < b.cp.size(); ++j)
            if (j != k && dlength(b.cp[j] - b.cp[k]) < 1e-9 && (j + 1 == k || k + 1 == j)) return out.fail("Not moved: CV " + std::to_string(k + 1) + " would sit on its neighbour");
        return store(ci, b, "Moved CV " + std::to_string(k + 1) + " of " + c.name + " to " + pointText(b.cp[k]) + ".");
    }
    if (sub == "insert") {
        const auto t = i < a.size() ? parseNumber(a[i]) : std::nullopt;
        if (!t || *t <= 0 || *t >= 1) return out.fail("CVCURVE INSERT curve t: t between 0 and 1 (along the curve's parameter), not at its ends");
        const double u = b.t0() + (b.t1() - b.t0()) * *t;
        int mult = 0;
        for (double k : b.knots)
            if (std::fabs(k - u) < 1e-12) ++mult;
        if (mult >= b.degree) return out.fail("There are " + std::to_string(mult) + " knots at " + a[i] + " already (as many as the degree)");
        if (b.cp.size() >= 500) return out.fail(c.name + " has 500 CVs: that is the most");
        return store(ci, insertKnot(b, u), "Inserted a knot into " + c.name + " (the same shape, one more CV).");
    }
    if (sub == "rebuild") {
        Options o;
        for (; i < a.size(); ++i) {
            if (!readOption(a, i, o, err)) return out.fail("Not an option: " + a[i] + "  (CVCURVE REBUILD curve [DEGREE d] [SPANS n])");
            if (!err.empty()) return out.fail(err);
        }
        BSplineCurve r;
        double dev = 0;
        const int degree = o.degree.value_or(b.degree), spans = o.spans.value_or(static_cast<int>(b.cp.size()) - b.degree);
        if (!rebuildCurve(b, degree, spans, r, &dev, &err)) return out.fail("Can't rebuild: " + err);
        return store(ci, r, "Rebuilt " + c.name + " (within " + lengthText(dev) + " of the old shape).");
    }
    if (sub == "match" || sub == "continuity") {
        // MATCH curve START|END TO other START|END [Gk] | CONTINUITY curve START|END other START|END
        auto readEnd = [&](bool& atEnd) {
            if (i >= a.size()) return false;
            const std::string w = lowerOf(a[i]);
            if (w != "start" && w != "end") return false;
            atEnd = w == "end";
            ++i;
            return true;
        };
        const std::string usage = sub == "match" ? "CVCURVE MATCH curve START|END TO other START|END [G0|G1|G2|G3]" : "CVCURVE CONTINUITY curve START|END other START|END";
        bool aEnd = false, oEnd = false;
        if (!readEnd(aEnd)) return out.fail(usage);
        if (i < a.size() && lowerOf(a[i]) == "to") ++i;
        const int oi = curveAt(doc, a, quoted, i, [&](const std::string& w) { return endWord(w); });
        if (oi < 0) return out.fail(i < a.size() ? "No sketch curve named " + a[i] : usage);
        if (oi == ci) return out.fail("A curve can't be matched to itself");
        if (!readEnd(oEnd)) return out.fail(usage);
        int level = 2;
        if (i < a.size()) {
            const std::string g = lowerOf(a[i]);
            if (g.size() != 2 || g[0] != 'g' || g[1] < '0' || g[1] > '3') return out.fail(usage);
            level = g[1] - '0';
            ++i;
        }
        if (i < a.size()) return out.fail("Not understood: " + a[i] + "  (" + usage + ")");
        BSplineCurve other;
        if (!asSpline(sketch.curves()[static_cast<std::size_t>(oi)], other, err)) return out.fail(err);
        // Both oriented so the other runs into the join and this one away from it.
        const BSplineCurve into = oEnd ? other : reverseCurve(other);
        const CurveLocal to = curveLocal(into, into.t1());
        BSplineCurve mine = aEnd ? reverseCurve(b) : b;
        const std::string otherName = sketch.curves()[static_cast<std::size_t>(oi)].name;
        if (sub == "continuity") {
            const Continuity k = continuityBetween(to, curveLocal(mine, mine.t0()));
            out.say(c.name + (aEnd ? " end" : " start") + " to " + otherName + (oEnd ? " end" : " start") + ": " + continuityText(k));
            return out;
        }
        if (!matchStart(mine, to, level, &err)) return out.fail("Can't match: " + err);
        const Continuity k = continuityBetween(to, curveLocal(mine, mine.t0()));
        if (aEnd) mine = reverseCurve(mine);
        return store(ci, mine, "Matched " + c.name + (aEnd ? " end" : " start") + " to " + otherName + (oEnd ? " end" : " start") + ": " + continuityText(k) + ".");
    }
    return out.fail("CVCURVE: " + a[0] + "?");
}

} // namespace sicdm

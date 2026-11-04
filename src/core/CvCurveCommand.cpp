#include "core/CvCurveCommand.hpp"

#include "core/Command.hpp"
#include "core/CvCurve.hpp"
#include "core/Document.hpp"
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
                        sub != "continuity" && sub != "info")) {
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

// CONSTRAIN, DIM, CONSTRAINTS and CONSTRAINT: the sketch's constraints and dimensions.

#include "core/SketchCommand.hpp"

#include "core/Command.hpp"
#include "core/Constraints.hpp"
#include "core/Document.hpp"
#include "core/FeatureCommand.hpp"
#include "core/Units.hpp"
#include "core/Extrude.hpp"
#include "core/SketchEdit.hpp"
#include "core/FileIO.hpp"
#include "core/Features.hpp"
#include "core/Sculpt.hpp"
#include "core/Section.hpp"

#include <algorithm>
#include <map>
#include <tuple>
#include <cctype>
#include <cmath>
#include <cstdio>

namespace sicdm {

namespace {

std::string lower(const std::string& s) {
    std::string o;
    for (char c : s) o += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return o;
}

const SketchCurve* curveNamed(const Sketch& s, const std::string& name) {
    for (const SketchCurve& c : s.curves())
        if (lower(c.name) == lower(name)) return &c;
    return nullptr;
}

bool isSelector(const std::string& w) {
    static const char* const k[] = {"start", "end", "center", "centre", "point", "corner", "side", "seg", "segment", "vertex"};
    return std::find(std::begin(k), std::end(k), w) != std::end(k);
}

// Reads curve references (a name, quoted or not, then an optional START / END / CENTRE /
// POINT n / CORNER n / SIDE n) and a trailing value. False with `why` on a bad word.
bool parseRefs(const Sketch& s, const std::vector<std::string>& a, const std::vector<bool>& quoted, std::size_t from, std::vector<SketchRef>& refs,
               std::optional<double>& value, bool angle, std::string& why) {
    std::size_t i = from;
    while (i < a.size()) {
        const bool q = i < quoted.size() && quoted[i];
        // A number at the end is the value.
        if (!q && i + 1 == a.size()) {
            std::optional<double> v; // degrees for an angle, else a length in the document's units
            if (!angle) v = parseLength(a[i]);
            else if (const auto n = parseNumber(a[i])) v = *n;
            if (v && !curveNamed(s, a[i])) {
                value = *v;
                return true;
            }
        }
        // The longest run of words that names a curve.
        const SketchCurve* c = nullptr;
        std::size_t used = 0;
        if (q) {
            c = curveNamed(s, a[i]);
            used = 1;
        } else {
            std::string name;
            for (std::size_t j = i; j < a.size() && !(j < quoted.size() && quoted[j]); ++j) {
                name += (name.empty() ? "" : " ") + a[j];
                if (const SketchCurve* hit = curveNamed(s, name)) c = hit, used = j - i + 1;
            }
        }
        if (!c) {
            why = "No sketch curve named " + a[i];
            return false;
        }
        if (c->spatial()) {
            why = c->name + " is a 3D curve: it takes no constraints or dimensions (PATH3D edits its points)";
            return false;
        }
        i += used;
        SketchRef r;
        r.curve = c->name;
        if (c->kind == CurveKind::Point) r.point = 0;
        if (i < a.size() && !(i < quoted.size() && quoted[i]) && isSelector(lower(a[i]))) {
            const std::string w = lower(a[i++]);
            auto index = [&]() -> int {
                const auto n = i < a.size() ? parseNumber(a[i]) : std::nullopt;
                if (!n || *n < 1) return -1;
                ++i;
                return static_cast<int>(*n) - 1;
            };
            if (w == "start") r.point = c->kind == CurveKind::Arc ? 1 : 0;
            else if (w == "end")
                r.point = c->kind == CurveKind::Line ? 1 : (c->kind == CurveKind::Polyline || c->kind == CurveKind::Spline) ? static_cast<int>(c->pts.size()) - 1 : 2;
            else if (w == "center" || w == "centre") r.point = 0;
            else if (w == "point" || w == "corner" || w == "vertex") r.point = index();
            else r.segment = index();
            if (r.point < 0 && r.segment < 0) {
                why = "A point or side number (1, 2, ...) follows " + toUpper(w);
                return false;
            }
        }
        refs.push_back(r);
    }
    return true;
}

std::string status(const SolveReport& r) {
    if (!r.ok) return "can't all be met";
    std::string s = r.dof == 0 ? "fully constrained" : "free in " + std::to_string(r.dof) + (r.dof == 1 ? " way" : " ways");
    if (r.redundant > 0) s += ", " + std::to_string(r.redundant) + " redundant";
    return s;
}

SketchConstraint* constraintNamed(Sketch& s, const std::string& w) {
    auto& cs = s.constraints();
    for (SketchConstraint& c : cs)
        if (lower(c.name) == lower(w)) return &c;
    const auto n = parseNumber(w);
    if (n && *n >= 1 && *n <= static_cast<double>(cs.size())) return &cs[static_cast<std::size_t>(*n) - 1];
    return nullptr;
}

} // namespace

CommandOutput runSketchCommand(Document& doc, const std::string& cmd, const std::vector<std::string>& a, const std::vector<bool>& quoted) {
    CommandOutput out;
    Sketch& sketch = doc.scene.sketch();

    // The curve named by the leading words (up to a keyword), or the selected one.
    auto leadingCurve = [&](std::size_t& i, std::initializer_list<const char*> keys) -> int {
        auto isKey = [&](const std::string& w) {
            for (const char* k : keys)
                if (lower(w) == k) return true;
            return false;
        };
        std::string name;
        std::size_t j = i;
        int found = -1;
        for (; j < a.size() && !(isKey(a[j]) && !(j < quoted.size() && quoted[j])); ++j) {
            name += (name.empty() ? "" : " ") + a[j];
            for (std::size_t k = 0; k < sketch.curves().size(); ++k)
                if (lower(sketch.curves()[k].name) == lower(name)) found = static_cast<int>(k), i = j + 1;
        }
        if (name.empty() && doc.selectedCurve >= 0 && doc.selectedCurve < static_cast<int>(sketch.curves().size())) return doc.selectedCurve;
        return found;
    };
    // 3D curves (PATH3D) are edited by their points, not by the flat sketch tools.
    auto flatOnly = [&](int ci) {
        return ci >= 0 && ci < static_cast<int>(sketch.curves().size()) && sketch.curves()[static_cast<std::size_t>(ci)].spatial()
                   ? sketch.curves()[static_cast<std::size_t>(ci)].name + " is a 3D curve: " + cmd + " works on flat sketch curves (PATH3D edits its points)"
                   : std::string();
    };
    auto replaceCurve = [&](std::size_t index, const std::vector<SketchCurve>& with) {
        const std::string gone = sketch.curves()[index].name;
        sketch.remove(index);
        std::string names;
        for (const SketchCurve& c : with) names += (names.empty() ? "" : ", ") + sketch.add(c).name;
        doc.selectedCurve = with.empty() ? -1 : static_cast<int>(sketch.curves().size()) - 1;
        doc.dirty = true;
        return names;
    };

    if (cmd == "CONSTRUCTION") {
        // CONSTRUCTION [curve...] [ON|OFF]
        bool on = true;
        std::vector<std::string> words = a;
        if (!words.empty() && (lower(words.back()) == "on" || lower(words.back()) == "off")) on = lower(words.back()) == "on", words.pop_back();
        std::vector<int> which;
        if (words.empty()) {
            if (doc.selectedCurve < 0 || doc.selectedCurve >= static_cast<int>(sketch.curves().size())) return out.fail("Select a sketch curve or name it");
            which.push_back(doc.selectedCurve);
        }
        for (std::size_t i = 0; i < words.size();) {
            std::string name;
            int found = -1;
            std::size_t used = 0;
            for (std::size_t j = i; j < words.size(); ++j) {
                name += (name.empty() ? "" : " ") + words[j];
                for (std::size_t k = 0; k < sketch.curves().size(); ++k)
                    if (lower(sketch.curves()[k].name) == lower(name)) found = static_cast<int>(k), used = j - i + 1;
            }
            if (found < 0) return out.fail("No sketch curve named " + words[i]);
            which.push_back(found);
            i += used;
        }
        for (int k : which) sketch.curves()[static_cast<std::size_t>(k)].construction = on;
        sketch.touch();
        doc.dirty = true;
        out.say(std::string(on ? "Construction: " : "Not construction: ") + std::to_string(which.size()) + (which.size() == 1 ? " curve" : " curves") +
                (on ? " (dashed; guides the sketch, not part of profiles)" : ""));
        return out;
    }

    if (cmd == "TRIMCURVE") {
        // TRIM "curve" AT x,y
        std::size_t i = 0;
        const int ci = leadingCurve(i, {"at"});
        if (ci < 0) return out.fail("TRIM \"curve\" AT x,y (a point on the piece to cut away)");
        if (const std::string w = flatOnly(ci); !w.empty()) return out.fail(w);
        if (i >= a.size() || lower(a[i]) != "at" || i + 1 >= a.size()) return out.fail("TRIM \"curve\" AT x,y");
        const auto p = parsePointInput(a[i + 1]);
        if (!p) return out.fail("AT takes the point x,y on the piece to cut away");
        std::vector<SketchCurve> left;
        std::string why;
        if (!trimCurve(sketch, static_cast<std::size_t>(ci), p->resolve({}), left, &why)) return out.fail(why);
        const std::string name = sketch.curves()[static_cast<std::size_t>(ci)].name;
        const std::string names = replaceCurve(static_cast<std::size_t>(ci), left);
        out.say(left.empty() ? "Trimmed away all of " + name + " (nothing crosses it there)" : "Trimmed " + name + ": " + names + " left");
        return out;
    }

    if (cmd == "EXTEND") {
        // EXTEND "curve" [START|END | AT x,y]
        std::size_t i = 0;
        const int ci = leadingCurve(i, {"at", "start", "end"});
        if (ci < 0) return out.fail(a.empty() ? std::string("EXTEND \"curve\" [START | END | AT x,y]") : "No sketch curve named " + a[0]);
        if (const std::string w = flatOnly(ci); !w.empty()) return out.fail(w);
        const SketchCurve& c = sketch.curves()[static_cast<std::size_t>(ci)];
        if (c.pts.empty()) return out.fail(c.name + " has no ends");
        Vec2 pick = c.pts.back();
        if (c.kind == CurveKind::Arc && c.pts.size() >= 3) pick = c.pts[2];
        if (i < a.size()) {
            const std::string w = lower(a[i]);
            if (w == "start") pick = c.kind == CurveKind::Arc && c.pts.size() >= 3 ? c.pts[1] : c.pts.front();
            else if (w == "at" && i + 1 < a.size()) {
                const auto p = parsePointInput(a[i + 1]);
                if (!p) return out.fail("AT takes a point x,y near the end to extend");
                pick = p->resolve({});
            } else if (w != "end") return out.fail("EXTEND \"curve\" [START | END | AT x,y]");
        }
        SketchCurve ext;
        std::string why;
        if (!extendCurve(sketch, static_cast<std::size_t>(ci), pick, ext, &why)) return out.fail(why);
        SketchCurve& target = sketch.curves()[static_cast<std::size_t>(ci)];
        target = ext;
        sketch.touch();
        doc.dirty = true;
        out.say("Extended " + target.name + "  " + target.measure());
        return out;
    }

    // Curve names from word i on (each the longest run of words naming one), up to a keyword.
    auto curveList = [&](std::size_t& i, std::initializer_list<const char*> keys, std::vector<int>& which) -> std::string {
        auto isKey = [&](std::size_t j) {
            if (j < quoted.size() && quoted[j]) return false;
            for (const char* k : keys)
                if (lower(a[j]) == k) return true;
            return false;
        };
        while (i < a.size() && !isKey(i)) {
            int found = -1;
            std::size_t used = 0;
            std::string name;
            for (std::size_t j = i; j < a.size() && !isKey(j); ++j) {
                name += (name.empty() ? "" : " ") + a[j];
                for (std::size_t k = 0; k < sketch.curves().size(); ++k)
                    if (lower(sketch.curves()[k].name) == lower(name)) found = static_cast<int>(k), used = j - i + 1;
                if (j < quoted.size() && quoted[j]) break;
            }
            if (found < 0) return a[i];
            which.push_back(found);
            i += used;
        }
        if (which.empty() && doc.selectedCurve >= 0 && doc.selectedCurve < static_cast<int>(sketch.curves().size())) which.push_back(doc.selectedCurve);
        return {};
    };

    if (cmd == "SKETCHFILLET" || cmd == "SKETCHCHAMFER") {
        // SKETCHFILLET curve [CORNER n...] radius | SKETCHFILLET line line radius
        const bool chamfer = cmd == "SKETCHCHAMFER";
        const std::string usage = std::string("Usage: ") + cmd + " curve [CORNER n...] " + (chamfer ? "distance" : "radius") + "  |  " + cmd + " line line " +
                                  (chamfer ? "distance" : "radius");
        if (a.empty()) return out.fail(usage);
        const auto size = parseLength(a.back());
        if (!size || !(*size > 0)) return out.fail(usage);
        std::vector<std::string> words(a.begin(), a.end() - 1);
        std::vector<int> corners;
        std::vector<std::string> names;
        for (std::size_t k = 0; k < words.size(); ++k) {
            if (!(k < quoted.size() && quoted[k]) && (lower(words[k]) == "corner" || lower(words[k]) == "vertex")) {
                const auto n = k + 1 < words.size() ? parseNumber(words[k + 1]) : std::nullopt;
                if (!n || *n < 1) return out.fail("CORNER takes a corner number (1, 2, ...)");
                corners.push_back(static_cast<int>(*n) - 1);
                ++k;
            } else {
                names.push_back(words[k]);
            }
        }
        std::vector<int> which; // one curve, or two lines: each the longest run of words naming one
        for (std::size_t j = 0; j < names.size();) {
            int hit = -1;
            std::size_t used = 0;
            std::string name;
            for (std::size_t k = j; k < names.size(); ++k) {
                name += (name.empty() ? "" : " ") + names[k];
                for (std::size_t q = 0; q < sketch.curves().size(); ++q)
                    if (lower(sketch.curves()[q].name) == lower(name)) hit = static_cast<int>(q), used = k - j + 1;
            }
            if (hit < 0) return out.fail("No sketch curve named " + names[j]);
            which.push_back(hit);
            j += used;
        }
        if (which.empty() && doc.selectedCurve >= 0 && doc.selectedCurve < static_cast<int>(sketch.curves().size())) which.push_back(doc.selectedCurve);
        if (which.empty() || which.size() > 2) return out.fail(usage);
        for (const int w : which)
            if (const std::string why3 = flatOnly(w); !why3.empty()) return out.fail(why3);
        const float r = static_cast<float>(*size);
        std::string why;
        if (which.size() == 2) {
            if (!corners.empty()) return out.fail(usage);
            const SketchCurve la = sketch.curves()[static_cast<std::size_t>(which[0])], lb = sketch.curves()[static_cast<std::size_t>(which[1])];
            SketchCurve na, nb, mid;
            if (!filletLines(la, lb, r, chamfer, na, nb, mid, &why)) return out.fail(why);
            sketch.curves()[static_cast<std::size_t>(which[0])].pts = na.pts;
            sketch.curves()[static_cast<std::size_t>(which[1])].pts = nb.pts;
            const SketchCurve& added = sketch.add(mid);
            doc.selectedCurve = static_cast<int>(sketch.curves().size()) - 1;
            doc.dirty = true;
            out.say((chamfer ? "Chamfered " : "Filleted ") + la.name + " and " + lb.name + ": " + added.name + "  " + added.measure());
            return out;
        }
        const std::size_t ci = static_cast<std::size_t>(which[0]);
        const SketchCurve c = sketch.curves()[ci];
        std::vector<SketchCurve> pieces;
        if (!roundCorners(c, corners, r, chamfer, pieces, &why)) return out.fail(why);
        const std::string names2 = replaceCurve(ci, pieces);
        out.say(std::string(chamfer ? "Chamfered " : "Filleted ") + c.name + " (" + lengthText(r) + "): " + names2);
        return out;
    }

    if (cmd == "SKETCHMIRROR") {
        // SKETCHMIRROR curve... ABOUT line | ABOUT x,y x,y
        const char* usage = "Usage: SKETCHMIRROR curve... ABOUT line | ABOUT x,y x,y";
        std::size_t i = 0;
        std::vector<int> which;
        if (const std::string bad = curveList(i, {"about", "across"}, which); !bad.empty()) return out.fail("No sketch curve named " + bad);
        if (which.empty() || i >= a.size()) return out.fail(usage);
        ++i; // ABOUT
        Vec2 p0, p1;
        std::string axisName;
        std::vector<int> axisCurve;
        std::size_t j = i;
        if (const std::string bad = curveList(j, {}, axisCurve); bad.empty() && axisCurve.size() == 1 && j == a.size()) {
            const SketchCurve& l = sketch.curves()[static_cast<std::size_t>(axisCurve[0])];
            if (l.kind != CurveKind::Line || l.pts.size() < 2) return out.fail("Mirror about a line (a construction centreline, say), or two points x,y x,y");
            p0 = l.pts[0], p1 = l.pts[1];
            axisName = l.name;
            for (int w : which)
                if (!samePlane(sketch.curves()[static_cast<std::size_t>(w)], l)) return out.fail(sketch.curves()[static_cast<std::size_t>(w)].name + " isn't on " + l.name + "'s plane");
        } else {
            const auto q0 = i < a.size() ? parsePointInput(a[i]) : std::nullopt;
            const auto q1 = i + 1 < a.size() ? parsePointInput(a[i + 1]) : std::nullopt;
            if (!q0 || !q1 || i + 2 != a.size()) return out.fail(usage);
            p0 = q0->resolve({}), p1 = q1->resolve({});
            axisName = "the line given";
        }
        const float dx = p1.x - p0.x, dy = p1.y - p0.y, l2 = dx * dx + dy * dy;
        if (!(l2 > 1e-12f)) return out.fail("The mirror line needs two different points");
        // Reflection across the line: m = 2 d d^T / |d|^2 - I, through p0.
        const float m[4] = {(dx * dx - dy * dy) / l2, 2 * dx * dy / l2, 2 * dx * dy / l2, (dy * dy - dx * dx) / l2};
        const Vec2 t{p0.x - (m[0] * p0.x + m[1] * p0.y), p0.y - (m[2] * p0.x + m[3] * p0.y)};
        std::string names;
        for (int w : which) {
            const SketchCurve src = sketch.curves()[static_cast<std::size_t>(w)];
            names += (names.empty() ? "" : ", ") + sketch.add(mapCurve(src, m, t)).name;
        }
        doc.selectedCurve = static_cast<int>(sketch.curves().size()) - 1;
        doc.dirty = true;
        out.say("Mirrored across " + axisName + ": " + names);
        return out;
    }

    if (cmd == "SKETCHPATTERN") {
        // SKETCHPATTERN curve... LINEAR n dx,dy [ROWS m dx,dy] | CIRCULAR n [ABOUT x,y] [ANGLE deg]
        const char* usage = "Usage: SKETCHPATTERN curve... LINEAR n dx,dy [ROWS m dx,dy] | CIRCULAR n [ABOUT x,y] [ANGLE deg]";
        std::size_t i = 0;
        std::vector<int> which;
        if (const std::string bad = curveList(i, {"linear", "circular"}, which); !bad.empty()) return out.fail("No sketch curve named " + bad);
        if (which.empty() || i >= a.size()) return out.fail(usage);
        const bool circular = lower(a[i++]) == "circular";
        const auto n = i < a.size() ? parseNumber(a[i++]) : std::nullopt;
        if (!n || *n < 2 || *n > 1000) return out.fail("The count is 2 to 1000 (the original is the first)");
        const int count = static_cast<int>(*n);
        std::vector<std::pair<float, Vec2>> place; // turn (deg) about `about`, then shift
        Vec2 about{};
        if (!circular) {
            const auto d1 = i < a.size() ? parsePointInput(a[i++]) : std::nullopt;
            if (!d1) return out.fail(usage);
            const Vec2 s1 = d1->resolve({});
            int rows = 1;
            Vec2 s2{};
            if (i < a.size() && lower(a[i]) == "rows") {
                const auto m = i + 1 < a.size() ? parseNumber(a[i + 1]) : std::nullopt;
                const auto d2 = i + 2 < a.size() ? parsePointInput(a[i + 2]) : std::nullopt;
                if (!m || *m < 1 || *m > 1000 || !d2) return out.fail(usage);
                rows = static_cast<int>(*m), s2 = d2->resolve({});
                i += 3;
            }
            if (i != a.size()) return out.fail(usage);
            if (static_cast<long>(count) * rows > 2000) return out.fail("At most 2000 copies");
            for (int r = 0; r < rows; ++r)
                for (int k = 0; k < count; ++k)
                    if (r || k) place.push_back({0.0f, {s1.x * static_cast<float>(k) + s2.x * static_cast<float>(r), s1.y * static_cast<float>(k) + s2.y * static_cast<float>(r)}});
        } else {
            float total = 360.0f;
            while (i < a.size()) {
                const std::string k = lower(a[i]);
                if (k == "about" && i + 1 < a.size()) {
                    const auto c = parsePointInput(a[i + 1]);
                    if (!c) return out.fail("ABOUT takes the centre x,y");
                    about = c->resolve({});
                    i += 2;
                } else if (k == "angle" && i + 1 < a.size()) {
                    const auto v = parseNumber(a[i + 1]);
                    if (!v || std::fabs(*v) < 1e-3f || std::fabs(*v) > 360.0f) return out.fail("ANGLE is up to 360 degrees");
                    total = *v;
                    i += 2;
                } else return out.fail(usage);
            }
            // A full turn spaces them evenly; less spans the angle from the first to the last.
            const float step = std::fabs(std::fabs(total) - 360.0f) < 1e-3f ? total / static_cast<float>(count) : total / static_cast<float>(count - 1);
            for (int k = 1; k < count; ++k) place.push_back({step * static_cast<float>(k), {}});
        }
        std::string first, lastName;
        int made = 0;
        for (const auto& [deg, shift] : place)
            for (int w : which) {
                const SketchCurve src = sketch.curves()[static_cast<std::size_t>(w)];
                const float r = radians(deg), cs = std::cos(r), sn = std::sin(r);
                const float m[4] = {cs, -sn, sn, cs};
                const Vec2 t{about.x - (cs * about.x - sn * about.y) + shift.x, about.y - (sn * about.x + cs * about.y) + shift.y};
                const std::string nm = sketch.add(mapCurve(src, m, t)).name;
                if (first.empty()) first = nm;
                lastName = nm;
                ++made;
            }
        doc.selectedCurve = static_cast<int>(sketch.curves().size()) - 1;
        doc.dirty = true;
        out.say(std::string(circular ? "Circular" : "Linear") + " pattern: " + std::to_string(made) + " copies (" + first + (made > 1 ? " to " + lastName : "") + ")");
        return out;
    }

    if (cmd == "PROJECT") {
        // PROJECT body [EDGES | SECTION] [CONSTRUCTION]
        const char* usage = "Usage: PROJECT body [EDGES | SECTION] [CONSTRUCTION]";
        bool section = false, construction = false;
        std::vector<std::string> words;
        for (std::size_t k = 0; k < a.size(); ++k) {
            const std::string w = (k < quoted.size() && quoted[k]) ? std::string() : lower(a[k]);
            if (w == "edges") section = false;
            else if (w == "section" || w == "intersect" || w == "intersection") section = true;
            else if (w == "construction" || w == "constr") construction = true;
            else words.push_back(a[k]);
        }
        std::string name;
        for (const std::string& w : words) name += (name.empty() ? "" : " ") + w;
        int bi = -1;
        for (std::size_t k = 0; k < doc.scene.bodies().size(); ++k)
            if (lower(doc.scene.bodies()[k].name) == lower(name)) bi = static_cast<int>(k);
        if (name.empty()) bi = doc.selectedBody;
        if (bi < 0 || bi >= static_cast<int>(doc.scene.bodies().size())) return out.fail(name.empty() ? std::string(usage) : "No body named " + name);
        const Body& body = doc.scene.bodies()[static_cast<std::size_t>(bi)];
        const PlaneFrame f = activePlaneFrame(doc.state);
        const MeshData m = worldMesh(body);
        if (m.indices.empty()) return out.fail(body.name + " has no surface");
        auto add = [&](std::vector<Vec2> pts, bool closedLoop) -> std::string {
            SketchCurve c;
            c.plane = doc.state.plane;
            c.frame = f;
            c.construction = construction;
            if (closedLoop) pts.push_back(pts.front());
            c.kind = pts.size() == 2 ? CurveKind::Line : CurveKind::Polyline;
            c.pts = std::move(pts);
            return sketch.add(std::move(c)).name;
        };
        // Points along a path that turn less than a hair are dropped (facets of a flat face).
        auto simplify = [](std::vector<Vec2> pts, bool closedLoop) {
            bool changed = true;
            while (changed && pts.size() > (closedLoop ? 3u : 2u)) {
                changed = false;
                for (std::size_t k = closedLoop ? 0 : 1; k < pts.size() - (closedLoop ? 0 : 1); ++k) {
                    const std::size_t n = pts.size();
                    const Vec2 p0 = pts[(k + n - 1) % n], p1 = pts[k], p2 = pts[(k + 1) % n];
                    const float l = std::hypot(p2.x - p0.x, p2.y - p0.y);
                    const float cr = std::fabs((p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x));
                    if (cr <= 1e-5f * std::max(l, 1.0f) * std::max(l, 1.0f) || std::hypot(p1.x - p0.x, p1.y - p0.y) < 1e-6f) {
                        pts.erase(pts.begin() + static_cast<std::ptrdiff_t>(k));
                        changed = true;
                        break;
                    }
                }
            }
            return pts;
        };
        int made = 0;
        if (section) {
            for (std::vector<Vec2> pts : planeSection(m, f)) {
                pts = simplify(std::move(pts), true);
                if (pts.size() < 3) continue;
                add(std::move(pts), true);
                ++made;
            }
            if (made == 0) return out.fail("The sketch plane doesn't cut " + body.name);
        } else {
            std::vector<std::vector<Vec2>> drawn; // edges behind others land on them: once each
            auto sameAs = [](const std::vector<Vec2>& x, const std::vector<Vec2>& y) {
                if (x.size() != y.size()) return false;
                bool fwd = true, back = true;
                for (std::size_t k = 0; k < x.size(); ++k) {
                    const Vec2 p = x[k], q = y[k], r = y[y.size() - 1 - k];
                    fwd = fwd && std::hypot(p.x - q.x, p.y - q.y) < 1e-3f;
                    back = back && std::hypot(p.x - r.x, p.y - r.y) < 1e-3f;
                }
                return fwd || back;
            };
            for (const EdgeChain& ch : edgeChains(m)) {
                std::vector<Vec2> pts;
                for (const Vec3& q : ch.pts) pts.push_back(f.toPlane(q));
                pts = simplify(std::move(pts), ch.closed);
                // Seen edge-on (square to the plane): nothing to draw.
                float span = 0;
                for (const Vec2& q : pts) span = std::max(span, std::hypot(q.x - pts[0].x, q.y - pts[0].y));
                if (pts.size() < 2 || span < 1e-4f) continue;
                if (ch.closed && pts.size() < 3) continue;
                if (std::any_of(drawn.begin(), drawn.end(), [&](const std::vector<Vec2>& d) { return sameAs(d, pts); })) continue;
                drawn.push_back(pts);
                add(std::move(pts), ch.closed);
                ++made;
            }
            if (made == 0) return out.fail(body.name + " has no edges to project");
        }
        doc.selectedCurve = static_cast<int>(sketch.curves().size()) - 1;
        doc.dirty = true;
        out.say(std::string("Projected ") + body.name + (section ? "'s section" : "'s edges") + " onto " + activePlaneName(doc.state) + ": " +
                std::to_string(made) + (made == 1 ? " curve" : " curves") + (construction ? " (construction)" : ""));
        return out;
    }

    if (cmd == "OFFSET") {
        // OFFSET "curve" distance [SIDE x,y]   (default: outside a closed curve, left of an open one)
        std::size_t i = 0;
        const int ci = leadingCurve(i, {"side", "toward", "towards"});
        if (ci < 0 && !a.empty() && !parseLength(a[0])) return out.fail("No sketch curve named " + a[0]);
        if (ci < 0 || i >= a.size()) return out.fail("OFFSET \"curve\" distance [SIDE x,y]");
        if (const std::string w = flatOnly(ci); !w.empty()) return out.fail(w);
        const auto dist = parseLength(a[i]);
        if (!dist) return out.fail("OFFSET \"curve\" distance [SIDE x,y]");
        const SketchCurve c = sketch.curves()[static_cast<std::size_t>(ci)];
        // Which side: given, or outside / left (a negative distance: inside / right).
        Vec2 side{};
        bool given = false;
        if (i + 2 < a.size() + 1 && i + 1 < a.size()) {
            const std::string w = lower(a[i + 1]);
            if ((w == "side" || w == "toward" || w == "towards") && i + 2 < a.size()) {
                const auto p = parsePointInput(a[i + 2]);
                if (!p) return out.fail("SIDE takes a point x,y on the side to offset to");
                side = p->resolve({});
                given = true;
            }
        }
        if (!given) {
            const std::vector<Vec2> strip = c.planeStrip(0.1f);
            if (strip.size() < 2) return out.fail(c.name + " has no side to offset to");
            Vec2 lo = strip[0], hi = strip[0];
            for (const Vec2& q : strip) lo = {std::min(lo.x, q.x), std::min(lo.y, q.y)}, hi = {std::max(hi.x, q.x), std::max(hi.y, q.y)};
            const bool outwards = *dist > 0;
            if (c.kind == CurveKind::Arc && c.pts.size() >= 3) { // away from its centre, or towards it
                const Vec2 m{(strip[strip.size() / 2].x), (strip[strip.size() / 2].y)};
                const Vec2 d{m.x - c.pts[0].x, m.y - c.pts[0].y};
                side = outwards ? Vec2{m.x + d.x, m.y + d.y} : c.pts[0];
            } else if (c.isClosed()) {
                const Vec2 mid{(lo.x + hi.x) * 0.5f, (lo.y + hi.y) * 0.5f};
                side = outwards ? Vec2{hi.x + (hi.x - lo.x) + 1.0f, hi.y + (hi.y - lo.y) + 1.0f} : mid;
                if (!outwards && (c.kind == CurveKind::Polyline || c.kind == CurveKind::Spline)) {
                    // A loop's inside: just inside its first piece.
                    const Vec2 p0 = strip[0], p1 = strip[1];
                    const float l = std::max(std::hypot(p1.x - p0.x, p1.y - p0.y), 1e-6f);
                    const float area = signedArea(strip);
                    const float s = area >= 0 ? 1.0f : -1.0f; // inside a counter-clockwise loop is on its left
                    side = {(p0.x + p1.x) * 0.5f - (p1.y - p0.y) / l * 1e-3f * s, (p0.y + p1.y) * 0.5f + (p1.x - p0.x) / l * 1e-3f * s};
                }
            } else {
                const Vec2 p0 = strip[0], p1 = strip[1];
                const float l = std::max(std::hypot(p1.x - p0.x, p1.y - p0.y), 1e-6f), s = outwards ? 1.0f : -1.0f;
                side = {(p0.x + p1.x) * 0.5f - (p1.y - p0.y) / l * s, (p0.y + p1.y) * 0.5f + (p1.x - p0.x) / l * s};
            }
        }
        SketchCurve made;
        std::string why;
        if (!offsetCurve(c, static_cast<float>(std::fabs(*dist)), side, made, &why)) return out.fail(why);
        const SketchCurve& added = sketch.add(made);
        doc.selectedCurve = static_cast<int>(sketch.curves().size()) - 1;
        doc.dirty = true;
        out.say("Offset " + c.name + " by " + lengthText(std::fabs(*dist)) + ": " + added.name + "  " + added.measure());
        return out;
    }

    if (cmd == "DRAG") {
        // DRAG curve [START|END|CENTRE|POINT n|CORNER n|RIM] TO x,y [FROM x,y]   or   DRAG curve BY dx,dy
        std::size_t i = 0;
        const int ci = leadingCurve(i, {"start", "end", "center", "centre", "point", "corner", "vertex", "rim", "to", "by", "from"});
        const char* usage = "DRAG curve [START | END | CENTRE | POINT n | RIM] TO x,y [FROM x,y], or DRAG curve BY dx,dy";
        if (ci < 0) return out.fail(a.empty() ? std::string(usage) : "No sketch curve named " + a[0]);
        const SketchCurve& c = sketch.curves()[static_cast<std::size_t>(ci)];
        SketchGrip grip;
        grip.ref.curve = c.name;
        if (c.kind == CurveKind::Point) grip.ref.point = 0;
        std::optional<Vec2> to, from, by;
        while (i < a.size()) {
            const std::string w = lower(a[i++]);
            auto pointArg = [&](std::optional<Vec2>& dst) {
                if (i >= a.size()) return false;
                const auto pt = parsePointInput(a[i++]);
                if (!pt) return false;
                dst = pt->resolve({});
                return true;
            };
            auto index = [&]() -> int {
                const auto n = i < a.size() ? parseNumber(a[i]) : std::nullopt;
                if (!n || *n < 1) return -1;
                ++i;
                return static_cast<int>(*n) - 1;
            };
            if (w == "start") grip.ref.point = c.kind == CurveKind::Arc ? 1 : 0;
            else if (w == "end")
                grip.ref.point = c.kind == CurveKind::Line ? 1
                                 : (c.kind == CurveKind::Polyline || c.kind == CurveKind::Spline) ? static_cast<int>(c.pts.size()) - 1
                                 : c.kind == CurveKind::Arc ? 2 : 1;
            else if (w == "center" || w == "centre") grip.ref.point = 0;
            else if (w == "point" || w == "corner" || w == "vertex") {
                grip.ref.point = index();
                if (grip.ref.point < 0) return out.fail("A point number (1, 2, ...) follows " + toUpper(w));
            } else if (w == "rim") {
                if (c.kind != CurveKind::Circle && c.kind != CurveKind::Arc) return out.fail("RIM drags a circle's or arc's radius");
                grip.rim = true;
            } else if (w == "to") {
                if (!pointArg(to)) return out.fail("TO takes the point x,y to drag to");
            } else if (w == "from") {
                if (!pointArg(from)) return out.fail("FROM takes the point x,y grabbed");
            } else if (w == "by") {
                if (!pointArg(by)) return out.fail("BY takes the move dx,dy");
            } else
                return out.fail(usage);
        }
        const bool whole = !grip.rim && grip.ref.point < 0;
        if (!to && !by) return out.fail(usage);
        if (by && !whole) return out.fail("BY moves a whole curve; a point or rim goes TO x,y");
        if (!c.pts.empty() && grip.ref.point >= 0) {
            // The point must be one the curve has.
            Sketch probe = sketch;
            SketchConstraint fix;
            fix.kind = ConstraintKind::Coincident;
            fix.refs = {grip.ref, grip.ref};
            if (std::isnan(constraintError(probe, fix))) return out.fail(c.name + " has no point " + std::to_string(grip.ref.point + 1));
        }
        Vec2 f = from.value_or(c.pts.empty() ? Vec2{} : c.pts[0]);
        Vec2 t = by ? Vec2{f.x + by->x, f.y + by->y} : *to;
        Sketch moved = sketch;
        const SolveReport r = dragSketch(moved, grip, f, t);
        if (!r.ok) return out.fail("Can't drag " + c.name + ": " + (r.conflict.empty() ? std::string("the constraints can't all be met") : r.conflict + " can't be met there"));
        // (c belongs to the sketch being replaced: name what moved first.)
        const std::string what = grip.rim ? c.name + " rim" : whole ? c.name : grip.ref.describe();
        sketch = std::move(moved);
        sketch.touch();
        doc.dirty = true;
        std::string line = "Dragged " + what;
        if (r.residual > 1e-3) line += ": held " + lengthText(r.residual) + " short by its constraints";
        Sketch probe = sketch;
        const SolveReport st = solveSketch(probe);
        if (!sketch.constraints().empty()) line += " (sketch " + status(st) + ")";
        out.say(line);
        return out;
    }

    if (cmd == "CONSTRAINTS") {
        if (sketch.constraints().empty()) {
            out.say("No constraints: CONSTRAIN HORIZONTAL \"Line 1\", DIM \"Line 1\" 40, ... (HELP CONSTRAIN)");
            return out;
        }
        Sketch probe = sketch;
        const SolveReport r = solveSketch(probe);
        out.say(std::to_string(sketch.constraints().size()) + " constraints on the sketch: " + status(r));
        for (const SketchConstraint& c : sketch.constraints()) {
            const double e = constraintError(sketch, c);
            out.say("  " + c.describe() + (std::isnan(e) ? "  (its curve is gone)" : e > 1e-5 ? "  (not met)" : ""));
        }
        return out;
    }

    if (cmd == "CONSTRAINT") {
        // CONSTRAINT EDIT name value | CONSTRAINT DELETE name
        if (a.size() < 2) return out.fail("Usage: CONSTRAINT EDIT D1 value | CONSTRAINT DELETE C2");
        const std::string sub = lower(a[0]);
        SketchConstraint* c = constraintNamed(sketch, a[1]);
        if (!c) return out.fail("No constraint " + a[1] + " (CONSTRAINTS lists them)");
        if (sub == "delete" || sub == "remove") {
            const std::string name = c->name;
            sketch.constraints().erase(sketch.constraints().begin() + (c - sketch.constraints().data()));
            doc.dirty = true;
            out.say("Removed " + name);
            return out;
        }
        if (sub != "edit" && sub != "set") return out.fail("Usage: CONSTRAINT EDIT D1 value | CONSTRAINT DELETE C2");
        if (!constraintHasValue(c->kind)) return out.fail(c->name + " has no value to change");
        if (a.size() < 3) return out.fail("CONSTRAINT EDIT " + c->name + " needs the new value");
        std::optional<double> v;
        if (c->kind != ConstraintKind::Angle) v = parseLength(a[2]);
        else if (const auto n = parseNumber(a[2])) v = *n;
        if (!v || !(*v >= 0)) return out.fail("The new value must be a number, zero or more");
        const std::string name = c->name;
        // Made by a step of a complete history: that step is changed, and everything
        // after it (extrusions of the sketch...) is rebuilt with the new size.
        ProjectState& st = doc.state;
        if (!doc.replaying && st.historyComplete && c->step >= 0 && c->step < static_cast<int>(st.history.size())) {
            const ParsedCommand pc = tokenizeCommand(st.history[static_cast<std::size_t>(c->step)].line);
            if (pc.name == "DIM" || pc.name == "CONSTRAIN") {
                std::vector<std::string> args = pc.args;
                std::vector<bool> q = pc.quoted;
                char b[64];
                std::snprintf(b, sizeof(b), "%.10g", *v);
                if (!args.empty() && !(args.size() <= q.size() && q[args.size() - 1]) && parseNumber(args.back()) && !curveNamed(sketch, args.back())) args.back() = b;
                else {
                    args.push_back(b);
                    q.push_back(false);
                }
                std::vector<HistoryStep> steps = st.history;
                steps[static_cast<std::size_t>(c->step)].line = pc.name + " " + rejoinArgs(args, q, 0);
                steps[static_cast<std::size_t>(c->step)].millimetres = doc.apiMillimeters ? steps[static_cast<std::size_t>(c->step)].millimetres : false;
                steps[static_cast<std::size_t>(c->step)].unit = st.units.unit;
                const int step = c->step; // (the rebuild makes the constraints anew)
                CommandOutput rebuild;
                if (!regenerateHistory(doc, steps, rebuild)) return rebuild;
                out.say(name + " set to " + a[2] + " and the model rebuilt from step " + std::to_string(step + 1));
                doc.historyRewritten = true; // the edit is in the step itself: not a step of its own
                doc.dirty = true;
                return out;
            }
        }
        const double old = c->value;
        c->value = *v;
        const SolveReport r = solveSketch(sketch);
        if (!r.ok) {
            for (SketchConstraint& k : sketch.constraints())
                if (k.name == name) k.value = old;
            return out.fail("Can't make " + name + " " + a[2] + ": it conflicts with " + r.conflict);
        }
        doc.dirty = true;
        out.say(name + " set to " + a[2] + "; the sketch is " + status(r));
        return out;
    }

    // CONSTRAIN kind refs... [value]      DIM [HORIZONTAL|VERTICAL|DIAMETER] refs... [value]
    ConstraintKind kind = ConstraintKind::Distance;
    std::size_t from = 0;
    bool dim = cmd == "DIM";
    if (dim) {
        if (!a.empty()) {
            const std::string w = lower(a[0]);
            if (w == "horizontal" || w == "h") kind = ConstraintKind::HDistance, from = 1;
            else if (w == "vertical" || w == "v") kind = ConstraintKind::VDistance, from = 1;
            else if (w == "diameter") kind = ConstraintKind::Diameter, from = 1;
            else if (w == "radius") kind = ConstraintKind::Radius, from = 1;
            else if (w == "angle") kind = ConstraintKind::Angle, from = 1;
        }
    } else {
        if (a.empty() || !constraintFromKey(lower(a[0]), &kind))
            return out.fail("CONSTRAIN COINCIDENT | HORIZONTAL | VERTICAL | PARALLEL | PERPENDICULAR | TANGENT | EQUAL | CONCENTRIC | ON | "
                            "MIDPOINT | FIX, then the curves");
        from = 1;
        dim = constraintHasValue(kind);
    }
    std::vector<SketchRef> refs;
    std::optional<double> value;
    std::string why;
    // An angle's value is in degrees; tell the parser before it reads the last word.
    bool angle = kind == ConstraintKind::Angle;
    if (dim && kind == ConstraintKind::Distance && cmd == "DIM") { // two whole lines: an angle
        std::vector<SketchRef> probe;
        std::optional<double> pv;
        std::string pw;
        if (parseRefs(sketch, a, quoted, from, probe, pv, true, pw) && probe.size() == 2) {
            const SketchCurve* c0 = curveNamed(sketch, probe[0].curve);
            const SketchCurve* c1 = curveNamed(sketch, probe[1].curve);
            auto lineLike = [](const SketchRef& r, const SketchCurve* c) {
                return r.point < 0 && (r.segment >= 0 || (c && c->kind == CurveKind::Line));
            };
            if (lineLike(probe[0], c0) && lineLike(probe[1], c1)) kind = ConstraintKind::Angle, angle = true;
        }
    }
    if (!parseRefs(sketch, a, quoted, from, refs, value, angle, why)) return out.fail(why);
    if (refs.empty()) return out.fail("Name the curves: " + cmd + " ... \"Line 1\" ...");
    if (dim && cmd == "DIM" && kind == ConstraintKind::Distance && refs.size() == 1) {
        const SketchCurve* c = curveNamed(sketch, refs[0].curve);
        if (c && (c->kind == CurveKind::Circle || c->kind == CurveKind::Arc) && refs[0].point < 0) kind = ConstraintKind::Radius;
    }
    if (!dim && value) return out.fail(std::string(constraintKey(kind)) + " takes no value");
    SketchConstraint c;
    c.kind = kind;
    c.refs = refs;
    c.name = sketch.nextConstraintName(constraintHasValue(kind));
    if (kind == ConstraintKind::Fix || kind == ConstraintKind::Tangent) {
        // Fix keeps the point where it is now; a tangent keeps the side it is on now.
        std::vector<SketchConstraint> keep = sketch.constraints();
        sketch.constraints() = {c};
        if (kind == ConstraintKind::Fix) {
            const SketchCurve* cv = curveNamed(sketch, refs[0].curve);
            if (!cv) return out.fail("No curve " + refs[0].curve);
            const int k = refs[0].point < 0 ? 0 : refs[0].point;
            Vec2 p = cv->pts[0];
            if (cv->kind == CurveKind::Rectangle) {
                const Vec2 a0 = cv->pts[0], a1 = cv->pts[1];
                p = k == 0 ? a0 : k == 1 ? Vec2{a1.x, a0.y} : k == 2 ? a1 : Vec2{a0.x, a1.y};
            } else if (k < static_cast<int>(cv->pts.size()))
                p = cv->pts[static_cast<std::size_t>(k)];
            c.value = p.x, c.value2 = p.y;
        } else if (refs.size() == 2) {
            const SketchCurve* a0 = curveNamed(sketch, refs[0].curve);
            const SketchCurve* a1 = curveNamed(sketch, refs[1].curve);
            if (a0 && a1 && (a0->kind == CurveKind::Circle || a0->kind == CurveKind::Arc) && (a1->kind == CurveKind::Circle || a1->kind == CurveKind::Arc)) {
                const double d = std::hypot(a1->pts[0].x - a0->pts[0].x, a1->pts[0].y - a0->pts[0].y);
                c.value = std::fabs(d - (a0->radius + a1->radius)) <= std::fabs(d - std::fabs(a0->radius - a1->radius)) ? 1.0 : -1.0;
            }
        }
        sketch.constraints() = std::move(keep);
    }
    if (constraintHasValue(kind) && !value) value = currentValue(sketch, c); // the size it has now
    if (value) c.value = *value;
    if (constraintHasValue(kind) && !(c.value >= 0)) return out.fail("A dimension must be zero or more");
    // Does it make sense for these curves?
    if (std::isnan(constraintError(sketch, c))) {
        sketch.noteConstraintName(c.name);
        return out.fail(std::string(constraintKey(kind)) + " doesn't apply to " + [&] {
            std::string s;
            for (const SketchRef& r : refs) s += (s.empty() ? "" : ", ") + r.describe();
            return s;
        }() + " (lines, points or circles as the constraint needs; points as START / END / CENTRE / POINT n, sides as SIDE n)");
    }
    sketch.constraints().push_back(c);
    const SolveReport r = solveSketch(sketch);
    if (!r.ok) {
        sketch.constraints().pop_back();
        return out.fail("Can't add " + c.describe() + ": it conflicts with " + (r.conflict == c.name ? std::string("the others") : r.conflict));
    }
    doc.pendingConstraint = c.name;
    doc.dirty = true;
    out.say("Added " + c.describe() + "; the sketch is " + status(r));
    return out;
}

} // namespace sicdm

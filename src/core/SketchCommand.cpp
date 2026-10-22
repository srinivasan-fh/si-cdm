// CONSTRAIN, DIM, CONSTRAINTS and CONSTRAINT: the sketch's constraints and dimensions.

#include "core/SketchCommand.hpp"

#include "core/Command.hpp"
#include "core/Constraints.hpp"
#include "core/Document.hpp"
#include "core/FeatureCommand.hpp"
#include "core/Units.hpp"

#include <algorithm>
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
            if (w == "start") r.point = 0;
            else if (w == "end") r.point = c->kind == CurveKind::Line ? 1 : c->kind == CurveKind::Polyline ? static_cast<int>(c->pts.size()) - 1 : 2;
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
        if (c && c->kind == CurveKind::Circle && refs[0].point < 0) kind = ConstraintKind::Radius;
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
            if (a0 && a1 && a0->kind == CurveKind::Circle && a1->kind == CurveKind::Circle) {
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

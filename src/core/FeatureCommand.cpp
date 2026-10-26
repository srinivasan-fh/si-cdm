// REVOLVE, SWEEP, LOFT, PATTERN, MIRROR, FILLET, MASSPROPS, MEASURE.

#include "core/FeatureCommand.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Extrude.hpp"
#include "core/Brep.hpp"
#include "core/Features.hpp"
#include "core/Sculpt.hpp"
#include "core/SketchEdit.hpp"
#include "core/Step.hpp"
#include "core/Units.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <functional>

namespace sicdm {

namespace {

std::string lower(const std::string& s) {
    std::string o;
    for (char c : s) o += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return o;
}

int curveNamed(const Document& d, const std::string& name) {
    const auto& cs = d.scene.sketch().curves();
    for (std::size_t i = 0; i < cs.size(); ++i)
        if (lower(cs[i].name) == lower(name)) return static_cast<int>(i);
    return -1;
}

int bodyNamed(const Document& d, const std::string& name) {
    const auto& bs = d.scene.bodies();
    for (std::size_t i = 0; i < bs.size(); ++i)
        if (lower(bs[i].name) == lower(name)) return static_cast<int>(i);
    return -1;
}

std::string fmt3(const char* f, double v) {
    char b[64];
    std::snprintf(b, sizeof(b), f, v);
    return b;
}

std::string closedText(const MeshData& m) {
    const std::size_t open = openEdgeCount(m);
    return std::to_string(m.indices.size() / 3) + " triangles, " + (open == 0 ? std::string("closed") : std::to_string(open) + " open edges");
}

// Reads the leading name (a quoted word, or words up to a keyword).
std::string leadingName(const std::vector<std::string>& a, const std::vector<bool>& quoted, std::size_t& i,
                        const std::function<bool(const std::string&)>& isKey) {
    std::string name;
    if (i < a.size() && i < quoted.size() && quoted[i]) return a[i++];
    while (i < a.size() && !isKey(lower(a[i]))) name += (name.empty() ? "" : " ") + a[i++];
    return name;
}

// A sketch curve as a path of world points.
bool curvePath(const SketchCurve& c, std::vector<Vec3>& path, bool& closed) {
    const PlaneFrame f = curveFrame(c);
    path.clear();
    closed = false;
    switch (c.kind) {
    case CurveKind::Line:
    case CurveKind::Polyline:
        for (const Vec2& p : c.pts) path.push_back(f.toWorld(p));
        if (path.size() > 2 && length(path.front() - path.back()) < 1e-4f) closed = true;
        return path.size() >= 2;
    case CurveKind::Rectangle: {
        const Vec2 a = c.pts[0], b = c.pts[1];
        for (const Vec2& p : {a, Vec2{b.x, a.y}, b, Vec2{a.x, b.y}}) path.push_back(f.toWorld(p));
        closed = true;
        return true;
    }
    case CurveKind::Circle:
        for (int i = 0; i < 96; ++i) {
            const float t = static_cast<float>(6.283185307179586 * i / 96);
            path.push_back(f.toWorld({c.pts[0].x + c.radius * std::cos(t), c.pts[0].y + c.radius * std::sin(t)}));
        }
        closed = true;
        return true;
    default: return false;
    }
}

void hideCurve(Document& doc, int i) {
    doc.scene.sketch().curves()[static_cast<std::size_t>(i)].visible = false;
    doc.scene.sketch().touch();
}

void addBody(Document& doc, Body b) {
    doc.scene.add(std::move(b));
    doc.selectedBody = static_cast<int>(doc.scene.bodies().size()) - 1;
    doc.selectedCurve = doc.selectedImage = -1;
    doc.dirty = true;
}

Vec3 bodyColor(int serial) {
    static const Vec3 kColours[] = {{0.42f, 0.62f, 0.86f}, {0.86f, 0.60f, 0.38f}, {0.55f, 0.78f, 0.52f}, {0.80f, 0.52f, 0.72f}, {0.84f, 0.78f, 0.45f}};
    return kColours[static_cast<std::size_t>(serial) % std::size(kColours)];
}

// "x,y" in the sketch plane (lengths in the document's units).
std::optional<Vec2> point2(const std::string& w) {
    const auto c = w.find(',');
    if (c == std::string::npos || w.find(',', c + 1) != std::string::npos) return std::nullopt;
    const auto x = parseLength(w.substr(0, c)), y = parseLength(w.substr(c + 1));
    if (!x || !y) return std::nullopt;
    return Vec2{static_cast<float>(*x), static_cast<float>(*y)};
}

std::optional<Vec3> axisWord(const std::string& w) {
    const std::string l = lower(w);
    if (l == "x") return Vec3{1, 0, 0};
    if (l == "y") return Vec3{0, 1, 0};
    if (l == "z") return Vec3{0, 0, 1};
    const auto v = parseVec3(w);
    if (v && length(*v) > 1e-9f) return normalize(*v);
    return std::nullopt;
}

// Cuts `tool` (a closed world mesh) from the bodies it meets, or joins it with them into the
// first. `target` names the only body to use ("" for any visible one it meets). The undo
// snapshot must already be taken: on false (why in `why`) the caller undoes.
bool combineTool(Document& doc, MeshData tool, BooleanOp op, const std::string& target, std::string& result, std::string& why) {
    const AABB tb = tool.bounds();
    std::vector<int> hits;
    if (!target.empty()) {
        const int t = bodyNamed(doc, target);
        if (t < 0) return why = "No body named " + target, false;
        hits.push_back(t);
    } else {
        for (std::size_t i = 0; i < doc.scene.bodies().size(); ++i) {
            const Body& b = doc.scene.bodies()[i];
            if (!b.visible || b.mesh.indices.empty()) continue;
            const AABB bb = worldMesh(b).bounds();
            if (bb.min.x <= tb.max.x && tb.min.x <= bb.max.x && bb.min.y <= tb.max.y && tb.min.y <= bb.max.y && bb.min.z <= tb.max.z &&
                tb.min.z <= bb.max.z)
                hits.push_back(static_cast<int>(i));
        }
    }
    if (hits.empty()) return why = op == BooleanOp::Subtract ? "it meets no body to cut" : "it meets no body to join", false;
    const Body toolBody = meshBody("tool", std::move(tool));
    std::vector<std::string> done;
    if (op == BooleanOp::Subtract) {
        for (int i : hits) {
            Body& b = doc.scene.bodies()[static_cast<std::size_t>(i)];
            const bool wasClosed = openEdgeCount(b.mesh) == 0;
            Body trial = b;
            std::string e;
            if (!bodyBoolean(trial, toolBody, BooleanOp::Subtract, &e)) continue; // only its box overlaps
            if (wasClosed && openEdgeCount(trial.mesh) > 0) return why = "cutting " + b.name + " left holes in it", false;
            b = std::move(trial);
            done.push_back(b.name);
        }
        if (done.empty()) return why = "it meets no body to cut (or would cut one away entirely)", false;
    } else {
        Body& into = doc.scene.bodies()[static_cast<std::size_t>(hits[0])];
        const bool wasClosed = openEdgeCount(into.mesh) == 0;
        std::string e;
        if (!bodyBoolean(into, toolBody, BooleanOp::Union, &e)) return why = e, false;
        done.push_back(into.name);
        // Other bodies it meets join in too.
        std::vector<int> gone;
        for (std::size_t k = 1; k < hits.size(); ++k) {
            const Body other = doc.scene.bodies()[static_cast<std::size_t>(hits[k])];
            Body& in = doc.scene.bodies()[static_cast<std::size_t>(hits[0])];
            Body trial = in;
            if (!bodyBoolean(trial, other, BooleanOp::Union, &e) || openEdgeCount(trial.mesh) > openEdgeCount(in.mesh)) continue;
            in = std::move(trial);
            done.push_back(other.name);
            gone.push_back(hits[k]);
        }
        if (wasClosed && openEdgeCount(doc.scene.bodies()[static_cast<std::size_t>(hits[0])].mesh) > 0) return why = "the join left holes", false;
        std::sort(gone.rbegin(), gone.rend());
        for (int g : gone) doc.scene.bodies().erase(doc.scene.bodies().begin() + g);
        const int at = hits[0] - static_cast<int>(std::count_if(gone.begin(), gone.end(), [&](int g) { return g < hits[0]; }));
        doc.selectedBody = at;
    }
    for (const std::string& n : done) result += (result.empty() ? "" : ", ") + n;
    doc.scene.touchBodies();
    doc.dirty = true;
    return true;
}

// The first distance along `dir` from each of `from` at which a ray meets a body (`only`:
// just that one), the farthest of them: where a feature starting at those points has met a
// surface across all of it. Nullopt when a ray meets nothing.
std::optional<float> upToNext(const Document& doc, const std::vector<Vec3>& from, Vec3 dir, int only) {
    std::vector<MeshData> meshes;
    for (std::size_t i = 0; i < doc.scene.bodies().size(); ++i) {
        const Body& b = doc.scene.bodies()[i];
        if (only >= 0 ? static_cast<int>(i) == only : b.visible) meshes.push_back(worldMesh(b));
    }
    float far = 0.0f;
    for (const Vec3& p : from) {
        const Ray ray{p, dir};
        float best = 1e30f;
        for (const MeshData& m : meshes)
            for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
                float d;
                if (intersectTriangle(ray, m.vertices[m.indices[t]].position, m.vertices[m.indices[t + 1]].position, m.vertices[m.indices[t + 2]].position, d) &&
                    d > 1e-3f && d < best)
                    best = d;
            }
        if (best >= 1e30f) return std::nullopt;
        far = std::max(far, best);
    }
    return far;
}

} // namespace

CommandOutput runFeatureCommand(Document& doc, const std::string& cmd, const std::vector<std::string>& a, const std::vector<bool>& quoted) {
    CommandOutput out;
    auto quotedAt = [&](std::size_t k) { return k < quoted.size() && quoted[k]; };

    if (cmd == "EXTRUDE") {
        // EXTRUDE [curve] [distance] [REVERSE] [SYMMETRIC | TWO d2] [THROUGH] [UPTO NEXT|body|x,y,z]
        //         [DRAFT deg] [THIN t] [CUT [body] | JOIN [body]] [KEEP]
        static const char* const kKeys[] = {"reverse", "symmetric", "sym", "both", "two", "through", "upto", "draft", "thin", "cut", "join", "keep"};
        auto isKey = [](const std::string& w) {
            for (const char* k : kKeys)
                if (w == k) return true;
            return false;
        };
        const char* usage = "Usage: EXTRUDE [curve] [distance] [REVERSE] [SYMMETRIC | TWO d2] [THROUGH] [UPTO NEXT|body|x,y,z] [DRAFT deg] "
                            "[THIN t] [CUT [body] | JOIN [body]] [KEEP]";
        bool reverse = false, symmetric = false, through = false, keep = false;
        std::optional<float> second, draft, thin;
        std::optional<BooleanOp> op;
        std::string target, upTo;
        std::vector<std::string> words; // the curve's name and the distance
        std::vector<bool> wordQuoted;
        for (std::size_t i = 0; i < a.size(); ++i) {
            const std::string k = quotedAt(i) ? std::string() : lower(a[i]);
            auto value = [&](bool angle) -> std::optional<float> { // the number after the keyword
                if (i + 1 >= a.size()) return std::nullopt;
                const auto v = angle ? std::optional<double>(parseNumber(a[i + 1])) : parseLength(a[i + 1]);
                if (!v || !std::isfinite(*v)) return std::nullopt;
                ++i;
                return static_cast<float>(*v);
            };
            if (k == "reverse") reverse = true;
            else if (k == "symmetric" || k == "sym" || k == "both") symmetric = true;
            else if (k == "through") through = true;
            else if (k == "keep") keep = true;
            else if (k == "two") {
                if (!(second = value(false)) || !(*second >= 0.0f)) return out.fail("TWO needs the second direction's distance");
            } else if (k == "draft") {
                if (!(draft = value(true))) return out.fail("DRAFT needs an angle in degrees");
            } else if (k == "thin") {
                if (!(thin = value(false)) || !(*thin > 0.0f)) return out.fail("THIN needs the wall's thickness");
            } else if (k == "upto") {
                if (i + 1 >= a.size()) return out.fail("UPTO needs NEXT, a body or a point x,y,z");
                upTo = a[++i];
            } else if (k == "cut" || k == "join") {
                op = k == "cut" ? BooleanOp::Subtract : BooleanOp::Union;
                if (i + 1 < a.size() && (quotedAt(i + 1) || (!isKey(lower(a[i + 1])) && bodyNamed(doc, a[i + 1]) >= 0))) target = a[++i];
            } else {
                words.push_back(a[i]);
                wordQuoted.push_back(quotedAt(i));
            }
        }
        // The distance: the last word when it is a length (needed unless THROUGH or UPTO).
        std::optional<float> distance;
        if (!words.empty() && !wordQuoted.back())
            if (const auto d = parseLength(words.back())) distance = static_cast<float>(*d), words.pop_back(), wordQuoted.pop_back();
        if (!distance && !through && upTo.empty()) return out.fail(usage);
        if (distance && *distance == 0.0f) return out.fail("Distance must not be zero");
        if ((through || !upTo.empty()) && (second || (through && !upTo.empty()))) return out.fail("THROUGH, UPTO and TWO go one at a time");
        std::string name;
        for (std::size_t k = 0; k < words.size(); ++k) name += (k ? " " : "") + words[k];
        const int ci = name.empty() ? doc.selectedCurve : curveNamed(doc, name);
        if (ci < 0 || ci >= static_cast<int>(doc.scene.sketch().curves().size()))
            return out.fail(name.empty() ? "Select a sketch curve to extrude, or name it" : "No sketch curve named " + name);
        const SketchCurve curve = doc.scene.sketch().curves()[static_cast<std::size_t>(ci)];
        const PlaneFrame f = curveFrame(curve);
        std::string why;
        const auto profile = profileFromSketch(doc.scene.sketch(), static_cast<std::size_t>(ci), &why);
        // An open curve makes a wall (THIN); its points in the plane.
        std::vector<Vec2> outline;
        bool closed = true;
        if (profile) outline = profilePolygon(*profile);
        else if (thin && !curve.construction && !curve.isClosed() && curve.kind != CurveKind::Point) {
            outline = curve.planeStrip(0.05f);
            closed = false;
        } else return out.fail("Cannot extrude: " + why);

        // The depths along the plane's normal.
        const float sign = reverse ? -1.0f : 1.0f;
        float w0 = 0.0f, w1 = 0.0f;
        std::string how;
        if (through || !upTo.empty()) {
            const int only = !target.empty() ? bodyNamed(doc, target) : -1;
            if (!target.empty() && only < 0) return out.fail("No body named " + target);
            if (through) { // past every body (or the target) along the normal
                bool any = false;
                float lo = 1e30f, hi = -1e30f;
                for (std::size_t i = 0; i < doc.scene.bodies().size(); ++i) {
                    const Body& b = doc.scene.bodies()[i];
                    if (only >= 0 ? static_cast<int>(i) != only : !b.visible) continue;
                    for (const Vertex& v : worldMesh(b).vertices) {
                        const float d = dot(v.position - f.origin, f.normal);
                        lo = std::min(lo, d), hi = std::max(hi, d), any = true;
                    }
                }
                if (!any) return out.fail("THROUGH: no body to go through");
                const float margin = 0.01f * (hi - lo) + 0.5f;
                if (symmetric) w0 = lo - margin, w1 = hi + margin;
                else if (sign > 0) w0 = 0.0f, w1 = hi + margin;
                else w0 = lo - margin, w1 = 0.0f;
                if (!(w1 - w0 > 1e-4f)) return out.fail("THROUGH: no body lies that way (REVERSE goes the other way)");
                how = "through all";
            } else if (const auto p = parseVec3(upTo); p && upTo.find(',') != std::string::npos) {
                const float d = dot(*p - f.origin, f.normal);
                if (std::fabs(d) < 1e-4f) return out.fail("UPTO: that point is on the sketch plane");
                w0 = std::min(0.0f, d), w1 = std::max(0.0f, d);
                how = "up to " + lengthText(d);
            } else {
                const bool next = lower(upTo) == "next";
                const int body = next ? only : bodyNamed(doc, upTo);
                if (!next && body < 0) return out.fail("UPTO needs NEXT, a body or a point x,y,z");
                std::vector<Vec3> starts;
                const std::vector<Vec2> probe = closed ? outline : thickenPolyline(outline, *thin * 0.5f);
                Vec2 c{};
                for (const Vec2& q : probe) starts.push_back(f.toWorld(q)), c = {c.x + q.x, c.y + q.y};
                if (!probe.empty()) starts.push_back(f.toWorld({c.x / static_cast<float>(probe.size()), c.y / static_cast<float>(probe.size())}));
                const auto d = upToNext(doc, starts, f.normal * sign, body);
                if (!d) return out.fail(std::string("UPTO: ") + (next ? "no face" : upTo + " is not") + " ahead across the whole profile (REVERSE looks the other way)");
                if (sign > 0) w0 = 0.0f, w1 = *d;
                else w0 = -*d, w1 = 0.0f;
                how = next ? "up to the next face" : "up to " + upTo;
            }
        } else {
            const float d = *distance * sign;
            if (symmetric) w0 = -std::fabs(d) * 0.5f, w1 = std::fabs(d) * 0.5f, how = lengthText(std::fabs(d)) + " symmetric";
            else if (second) {
                const float s2 = *second * (d > 0 ? -1.0f : 1.0f);
                w0 = std::min(d, s2), w1 = std::max(d, s2);
                how = lengthText(std::fabs(d)) + " and " + lengthText(*second) + " back";
            } else w0 = std::min(0.0f, d), w1 = std::max(0.0f, d), how = lengthText(std::fabs(d));
        }
        if (!(w1 - w0 > 1e-5f)) return out.fail("The extrusion has no depth");

        // The solid: a plain extrusion stays parametric (exact circles); drafts and walls are
        // built flat-sided, exact where the profile is.
        const int serial = ++doc.primitiveSerial;
        Body b;
        const bool plain = closed && !thin && (!draft || *draft == 0.0f);
        if (plain) {
            b = defaultPrimitive("box", serial);
            b.name = "Extrude " + std::to_string(serial);
            b.kind = "extrude";
            b.params = extrudeParams(*profile, curve.plane, 1.0f, false, curve.frame);
            b.params[1] = w0, b.params[2] = w1;
            b.mesh = buildExtrusion(b.params);
            if (b.mesh.indices.empty()) return out.fail("Extrusion failed for " + curve.name);
            b.offset = {};
            attachPrimitiveBrep(b);
        } else {
            ExtrudeShape sh;
            sh.w0 = w0, sh.w1 = w1;
            sh.draftDeg = draft.value_or(0.0f);
            sh.thin = thin.value_or(0.0f);
            auto mesh = extrudeShape(outline, closed, f, sh, &why);
            if (!mesh) return out.fail("Cannot extrude " + curve.name + ": " + why);
            b = featureBody("Extrude " + std::to_string(serial), std::move(*mesh), bodyColor(serial));
            const bool flat = curve.kind == CurveKind::Rectangle || curve.kind == CurveKind::Polyline || curve.kind == CurveKind::Line;
            if (flat) { // flat faces: the facets are exact
                if (auto br = meshToBrep(b.mesh)) {
                    br->name = b.name;
                    b.brep = std::move(br);
                    b.brepKey = meshKey(b.mesh);
                }
            }
            if (sh.draftDeg != 0.0f) how += ", draft " + fmt3("%g", sh.draftDeg) + " deg";
            if (sh.thin > 0.0f) how += ", wall " + lengthText(sh.thin);
        }
        auto hideProfile = [&] {
            if (keep) return;
            std::vector<std::size_t> chain; // a loop of curves end to end: all of them
            if (closed && !curve.isClosed()) chainLoop(doc.scene.sketch(), static_cast<std::size_t>(ci), &chain);
            for (std::size_t k : chain) doc.scene.sketch().curves()[k].visible = false;
            hideCurve(doc, ci);
        };
        if (op) {
            doc.scene.pushUndo("EXTRUDE " + curve.name);
            std::string into;
            if (!combineTool(doc, worldMesh(b), *op, target, into, why)) {
                doc.scene.undo();
                return out.fail("Cannot " + std::string(*op == BooleanOp::Subtract ? "cut" : "join") + " with " + curve.name + ": " + why +
                                " (nothing changed)");
            }
            hideProfile();
            if (*op == BooleanOp::Subtract) doc.selectedBody = bodyNamed(doc, into.substr(0, into.find(", ")));
            doc.selectedCurve = doc.selectedImage = -1;
            out.say(std::string(*op == BooleanOp::Subtract ? "Cut " : "Joined ") + curve.name + " " + how + (*op == BooleanOp::Subtract ? " from " : " into ") +
                    into);
            return out;
        }
        hideProfile();
        doc.scene.pushUndo("EXTRUDE " + curve.name);
        out.say("Extruded " + curve.name + " " + how + " -> " + b.name + ": " + closedText(b.mesh));
        addBody(doc, std::move(b));
        return out;
    }

    if (cmd == "REVOLVE") {
        // REVOLVE [curve] [ANGLE deg] [AXIS U|V|"line"|x,y x,y] [KEEP]
        auto isKey = [](const std::string& w) { return w == "angle" || w == "axis" || w == "keep" || w == "cut" || w == "join"; };
        std::size_t i = 0;
        const std::string name = leadingName(a, quoted, i, isKey);
        const int ci = name.empty() ? doc.selectedCurve : curveNamed(doc, name);
        if (ci < 0 || ci >= static_cast<int>(doc.scene.sketch().curves().size()))
            return out.fail(name.empty() ? "Select a closed sketch curve to revolve, or name it" : "No sketch curve named " + name);
        float angle = 360.0f;
        bool keep = false;
        std::optional<BooleanOp> op; // CUT / JOIN [body]
        std::string target;
        std::optional<std::pair<Vec2, Vec2>> axis; // point, direction
        std::string axisName = "the plane's vertical axis";
        const SketchCurve curve = doc.scene.sketch().curves()[static_cast<std::size_t>(ci)];
        for (; i < a.size(); ++i) {
            const std::string k = lower(a[i]);
            if (k == "keep") keep = true;
            else if (k == "cut" || k == "join") {
                op = k == "cut" ? BooleanOp::Subtract : BooleanOp::Union;
                if (i + 1 < a.size() && (quotedAt(i + 1) || (!isKey(lower(a[i + 1])) && bodyNamed(doc, a[i + 1]) >= 0))) target = a[++i];
            } else if (k == "angle") {
                const auto v = i + 1 < a.size() ? parseNumber(a[++i]) : std::nullopt;
                if (!v || std::fabs(*v) < 0.01f || std::fabs(*v) > 360.0f) return out.fail("ANGLE is up to 360 degrees");
                angle = *v;
            } else if (k == "axis") {
                if (i + 1 >= a.size()) return out.fail("AXIS needs U, V, a sketch line, or two points x,y x,y");
                const std::string w = a[++i];
                const std::string lw = lower(w);
                if (!quotedAt(i) && (lw == "u" || lw == "h" || lw == "horizontal")) axis = {{Vec2{0, 0}, Vec2{1, 0}}}, axisName = "the plane's horizontal axis";
                else if (!quotedAt(i) && (lw == "v" || lw == "vertical")) axis = {{Vec2{0, 0}, Vec2{0, 1}}};
                else if (const int li = curveNamed(doc, w); li >= 0) {
                    const SketchCurve& l = doc.scene.sketch().curves()[static_cast<std::size_t>(li)];
                    if (l.kind != CurveKind::Line || !samePlane(l, curve)) return out.fail("The axis must be a sketch line on the profile's plane");
                    axis = {{l.pts[0], Vec2{l.pts[1].x - l.pts[0].x, l.pts[1].y - l.pts[0].y}}};
                    axisName = l.name;
                    hideCurve(doc, li);
                } else {
                    const auto p0 = point2(w);
                    const auto p1 = i + 1 < a.size() ? point2(a[i + 1]) : std::nullopt;
                    if (!p0 || !p1) return out.fail("AXIS needs U, V, a sketch line, or two points x,y x,y");
                    ++i;
                    axis = {{*p0, Vec2{p1->x - p0->x, p1->y - p0->y}}};
                    axisName = "the axis given";
                }
            } else return out.fail("Usage: REVOLVE [curve] [ANGLE deg] [AXIS U|V|\"line\"|x,y x,y] [CUT|JOIN [body]] [KEEP]");
        }
        std::string why;
        const auto profile = profileFromSketch(doc.scene.sketch(), static_cast<std::size_t>(ci), &why);
        if (!profile) return out.fail("Cannot revolve " + curve.name + ": " + why);
        std::optional<MeshData> mesh;
        std::shared_ptr<Brep> exact;
        if (axis) mesh = revolveProfile(*profile, curveFrame(curve), axis->first, axis->second, angle, &why, &exact);
        else { // the vertical axis, else the horizontal one, through the plane's origin
            mesh = revolveProfile(*profile, curveFrame(curve), {0, 0}, {0, 1}, angle, &why, &exact);
            if (!mesh) {
                mesh = revolveProfile(*profile, curveFrame(curve), {0, 0}, {1, 0}, angle, &why, &exact);
                axisName = "the plane's horizontal axis";
            }
        }
        if (!mesh) return out.fail("Cannot revolve " + curve.name + ": " + why);
        if (op) { // a revolved cut (a groove) or boss
            doc.scene.pushUndo("REVOLVE " + curve.name);
            std::string into;
            if (!combineTool(doc, std::move(*mesh), *op, target, into, why)) {
                doc.scene.undo();
                return out.fail("Cannot " + std::string(*op == BooleanOp::Subtract ? "cut" : "join") + " with " + curve.name + ": " + why +
                                " (nothing changed)");
            }
            if (!keep) hideCurve(doc, ci);
            if (*op == BooleanOp::Subtract) doc.selectedBody = bodyNamed(doc, into.substr(0, into.find(", ")));
            doc.selectedCurve = doc.selectedImage = -1;
            out.say(std::string(*op == BooleanOp::Subtract ? "Cut " : "Joined ") + curve.name + " revolved " + fmt3("%g", angle) + " deg about " + axisName +
                    (*op == BooleanOp::Subtract ? " from " : " into ") + into);
            return out;
        }
        const int serial = ++doc.primitiveSerial;
        Body b = featureBody("Revolve " + std::to_string(serial), std::move(*mesh), bodyColor(serial));
        if (exact) { // the exact solid goes with this mesh
            exact->name = b.name;
            b.brep = std::move(exact);
            b.brepKey = meshKey(b.mesh);
        }
        if (!keep) hideCurve(doc, ci);
        doc.scene.pushUndo("REVOLVE " + curve.name);
        out.say("Revolved " + curve.name + " " + fmt3("%g", angle) + " deg about " + axisName + " -> " + b.name + ": " + closedText(b.mesh));
        addBody(doc, std::move(b));
        return out;
    }

    if (cmd == "SWEEP") {
        // SWEEP [profile] ALONG path [KEEP]
        auto isKey = [](const std::string& w) { return w == "along" || w == "path" || w == "keep"; };
        std::size_t i = 0;
        const std::string name = leadingName(a, quoted, i, isKey);
        std::string pathName;
        bool keep = false;
        for (; i < a.size(); ++i) {
            const std::string k = lower(a[i]);
            if (k == "keep") keep = true;
            else if (k == "along" || k == "path") {
                ++i;
                pathName = leadingName(a, quoted, i, isKey);
                --i;
            } else return out.fail("Usage: SWEEP [profile] ALONG path [KEEP]");
        }
        const int ci = name.empty() ? doc.selectedCurve : curveNamed(doc, name);
        if (ci < 0 || ci >= static_cast<int>(doc.scene.sketch().curves().size())) return out.fail("Name the closed profile to sweep (or select it)");
        const int pi = curveNamed(doc, pathName);
        if (pathName.empty() || pi < 0) return out.fail("Give the path: SWEEP profile ALONG \"path curve\"");
        if (pi == ci) return out.fail("The profile and the path must be different curves");
        const SketchCurve profileCurve = doc.scene.sketch().curves()[static_cast<std::size_t>(ci)];
        const SketchCurve pathCurve = doc.scene.sketch().curves()[static_cast<std::size_t>(pi)];
        std::string why;
        const auto profile = profileFromSketch(doc.scene.sketch(), static_cast<std::size_t>(ci), &why);
        if (!profile) return out.fail("Cannot sweep " + profileCurve.name + ": " + why);
        std::vector<Vec3> path;
        bool closed = false;
        if (!curvePath(pathCurve, path, closed)) return out.fail(pathCurve.name + " can't be a path (a line, polyline, rectangle or circle)");
        auto mesh = sweepProfile(*profile, curveFrame(profileCurve), path, closed, &why);
        if (!mesh) return out.fail("Cannot sweep: " + why);
        const int serial = ++doc.primitiveSerial;
        Body b = featureBody("Sweep " + std::to_string(serial), std::move(*mesh), bodyColor(serial));
        if (!keep) {
            hideCurve(doc, ci);
            hideCurve(doc, pi);
        }
        doc.scene.pushUndo("SWEEP " + profileCurve.name);
        out.say("Swept " + profileCurve.name + " along " + pathCurve.name + " -> " + b.name + ": " + closedText(b.mesh));
        addBody(doc, std::move(b));
        return out;
    }

    if (cmd == "LOFT") {
        // LOFT "a" [AT d] "b" [AT d] ... [SMOOTH|RULED] [KEEP]
        std::vector<std::pair<int, float>> sections;
        bool smooth = true, keep = false;
        for (std::size_t i = 0; i < a.size(); ++i) {
            const std::string k = lower(a[i]);
            if (!quotedAt(i) && k == "smooth") smooth = true;
            else if (!quotedAt(i) && k == "ruled") smooth = false;
            else if (!quotedAt(i) && k == "keep") keep = true;
            else if (!quotedAt(i) && k == "at") {
                const auto v = i + 1 < a.size() ? parseLength(a[++i]) : std::nullopt;
                if (!v || sections.empty()) return out.fail("AT d follows a section: the distance along its plane's normal");
                sections.back().second = static_cast<float>(*v);
            } else {
                const int ci = curveNamed(doc, a[i]);
                if (ci < 0) return out.fail("No sketch curve named " + a[i] + " (quote names with spaces)");
                sections.push_back({ci, 0.0f});
            }
        }
        if (sections.size() < 2) return out.fail("Usage: LOFT \"section\" [AT d] \"section\" [AT d] ... [SMOOTH|RULED] [KEEP]");
        std::vector<std::vector<Vec3>> loops;
        std::string names;
        for (const auto& [ci, off] : sections) {
            const SketchCurve& c = doc.scene.sketch().curves()[static_cast<std::size_t>(ci)];
            std::string why;
            const auto profile = profileFromSketch(doc.scene.sketch(), static_cast<std::size_t>(ci), &why);
            if (!profile) return out.fail("Section " + c.name + ": " + why);
            const PlaneFrame f = curveFrame(c);
            std::vector<Vec3> loop;
            for (const Vec2& p : profilePolygon(*profile)) loop.push_back(f.toWorld(p) + f.normal * off);
            loops.push_back(std::move(loop));
            names += (names.empty() ? "" : ", ") + c.name;
        }
        std::string why;
        auto mesh = loftSections(loops, smooth, &why);
        if (!mesh) return out.fail("Cannot loft: " + why);
        const int serial = ++doc.primitiveSerial;
        Body b = featureBody("Loft " + std::to_string(serial), std::move(*mesh), bodyColor(serial));
        if (!keep)
            for (const auto& s : sections) hideCurve(doc, s.first);
        doc.scene.pushUndo("LOFT");
        out.say("Lofted " + names + (smooth ? " (smooth)" : " (ruled)") + " -> " + b.name + ": " + closedText(b.mesh));
        addBody(doc, std::move(b));
        return out;
    }

    if (cmd == "PATTERN" || cmd == "MIRROR") {
        static const char* const kKeys[] = {"linear", "circular", "spacing", "dir", "rows", "spacing2", "dir2", "angle", "axis", "at",
                                            "union", "subtract", "plane", "normal", "copy", "move", "into", "xy", "yz", "xz", "zx"};
        auto isKey = [](const std::string& w) {
            for (const char* k : kKeys)
                if (w == k) return true;
            return false;
        };
        std::size_t i = 0;
        const std::string name = leadingName(a, quoted, i, isKey);
        const int bi = name.empty() ? doc.selectedBody : bodyNamed(doc, name);
        if (bi < 0 || bi >= static_cast<int>(doc.scene.bodies().size()))
            return out.fail(name.empty() ? "Select a body, or name it" : "No body named " + name);
        const Body src = doc.scene.bodies()[static_cast<std::size_t>(bi)];
        const MeshData world = worldMesh(src);
        if (world.indices.empty()) return out.fail(src.name + " has no surface");
        std::vector<MeshData> copies;
        std::vector<Mat4> placements; // PATTERN: where each copy goes
        std::string what;
        std::string into;
        BooleanOp op = BooleanOp::Union;
        bool merge = false, moveOriginal = false;
        if (cmd == "PATTERN") {
            bool circular = false;
            int n = 0, rows = 1;
            float spacing = 0, spacing2 = 0, angle = 360;
            Vec3 dir{1, 0, 0}, dir2{0, 1, 0}, axis{0, 0, 1}, at = src.bounds().center();
            bool atGiven = false;
            for (; i < a.size(); ++i) {
                const std::string k = lower(a[i]);
                auto next = [&]() -> std::string { return i + 1 < a.size() ? a[++i] : std::string(); };
                if (k == "linear" || k == "circular") {
                    circular = k == "circular";
                    const auto v = parseNumber(next());
                    if (!v || *v < 2 || *v > 500) return out.fail("The count is 2 to 500");
                    n = static_cast<int>(*v);
                } else if (k == "rows") {
                    const auto v = parseNumber(next());
                    if (!v || *v < 1 || *v > 500) return out.fail("ROWS is 1 to 500");
                    rows = static_cast<int>(*v);
                } else if (k == "spacing" || k == "spacing2") {
                    const auto v = parseLength(next());
                    if (!v) return out.fail(toUpper(k) + " needs a length");
                    (k == "spacing" ? spacing : spacing2) = static_cast<float>(*v);
                } else if (k == "dir" || k == "dir2" || k == "axis") {
                    const auto v = axisWord(next());
                    if (!v) return out.fail(toUpper(k) + " needs X, Y, Z or x,y,z");
                    (k == "dir" ? dir : k == "dir2" ? dir2 : axis) = *v;
                } else if (k == "angle") {
                    const auto v = parseNumber(next());
                    if (!v || std::fabs(*v) < 0.01f || std::fabs(*v) > 360) return out.fail("ANGLE is up to 360 degrees");
                    angle = *v;
                } else if (k == "at") {
                    const auto v = parseVec3(next());
                    if (!v) return out.fail("AT needs x,y,z");
                    at = *v;
                    atGiven = true;
                } else if (k == "union" || k == "subtract" || k == "into") {
                    merge = true;
                    if (k == "subtract") op = BooleanOp::Subtract;
                    ++i;
                    into = leadingName(a, quoted, i, isKey);
                    --i;
                } else return out.fail("Usage: PATTERN [body] LINEAR n SPACING d [DIR x,y,z] [ROWS m SPACING2 d DIR2 x,y,z] | CIRCULAR n [ANGLE a] "
                                       "[AXIS X|Y|Z|x,y,z] [AT x,y,z] [UNION|SUBTRACT \"target\"]");
            }
            if (n < 2) return out.fail("Give LINEAR n or CIRCULAR n");
            if (!circular) {
                if (spacing == 0) return out.fail("LINEAR needs SPACING");
                if (rows > 1 && spacing2 == 0) return out.fail("ROWS needs SPACING2");
                for (int r = 0; r < rows; ++r)
                    for (int k = 0; k < n; ++k) {
                        if (r == 0 && k == 0) continue;
                        const Mat4 move = translate(dir * (spacing * static_cast<float>(k)) + dir2 * (spacing2 * static_cast<float>(r)));
                        placements.push_back(move);
                        if (!merge) continue; // copies of the body itself, below
                        MeshData m = world;
                        m.transform(move);
                        copies.push_back(std::move(m));
                    }
                what = std::to_string(n) + (rows > 1 ? " x " + std::to_string(rows) : std::string()) + " linear";
            } else {
                if (!atGiven) at = {}; // about the axis through the origin
                const bool full = std::fabs(angle) >= 359.999f;
                const float step = angle / static_cast<float>(full ? n : n - 1);
                for (int k = 1; k < n; ++k) {
                    const float t = step * static_cast<float>(k) * 3.14159265f / 180.0f;
                    const Mat4 turn = translate(at) * rotateAxis(axis, t) * translate(-at);
                    placements.push_back(turn);
                    if (!merge) continue;
                    MeshData m = world;
                    m.transform(turn);
                    copies.push_back(std::move(m));
                }
                what = std::to_string(n) + " circular";
            }
        } else { // MIRROR
            Vec3 point{}, normal{1, 0, 0};
            bool copy = true;
            for (; i < a.size(); ++i) {
                const std::string k = lower(a[i]);
                auto next = [&]() -> std::string { return i + 1 < a.size() ? a[++i] : std::string(); };
                if (k == "plane") {
                    const std::string p = lower(next());
                    if (p == "yz") normal = {1, 0, 0};
                    else if (p == "xz" || p == "zx") normal = {0, 1, 0};
                    else if (p == "xy") normal = {0, 0, 1};
                    else return out.fail("PLANE is XY, YZ or XZ");
                } else if (k == "yz") normal = {1, 0, 0};
                else if (k == "xz" || k == "zx") normal = {0, 1, 0};
                else if (k == "xy") normal = {0, 0, 1};
                else if (k == "at") {
                    const auto v = parseVec3(next());
                    if (!v) return out.fail("AT needs x,y,z");
                    point = *v;
                } else if (k == "normal") {
                    const auto v = axisWord(next());
                    if (!v) return out.fail("NORMAL needs X, Y, Z or x,y,z");
                    normal = *v;
                } else if (k == "copy") copy = true;
                else if (k == "move") copy = false;
                else if (k == "union") merge = true;
                else return out.fail("Usage: MIRROR [body] [PLANE XY|YZ|XZ] [AT x,y,z] [NORMAL x,y,z] [COPY|MOVE] [UNION]");
            }
            copies.push_back(mirrorMesh(world, point, normal));
            moveOriginal = !copy;
            what = "mirror";
            if (merge) into = src.name;
        }
        doc.scene.pushUndo(cmd + " " + src.name);
        if (moveOriginal) { // the body itself flips over
            Body& b = doc.scene.bodies()[static_cast<std::size_t>(bi)];
            const std::string keepName = b.name;
            const Vec3 keepColor = b.color;
            b = featureBody(keepName, std::move(copies[0]), keepColor);
            doc.scene.touchBodies();
            doc.dirty = true;
            out.say("Mirrored " + keepName + ": " + closedText(b.mesh));
            return out;
        }
        if (merge) {
            const int ti = into.empty() ? bi : bodyNamed(doc, into);
            if (ti < 0) return out.fail("No body named " + into);
            Body& target = doc.scene.bodies()[static_cast<std::size_t>(ti)];
            // The original joins too when it isn't the target.
            if (ti != bi && op == BooleanOp::Union) copies.insert(copies.begin(), world);
            if (ti != bi && op == BooleanOp::Subtract) copies.insert(copies.begin(), world);
            std::size_t done = 0;
            const bool wasClosed = openEdgeCount(target.mesh) == 0;
            for (MeshData& m : copies) {
                const Body tool = meshBody("tool", std::move(m));
                std::string e;
                if (!bodyBoolean(target, tool, op, &e) || (wasClosed && openEdgeCount(target.mesh) > 0)) {
                    if (e.empty()) e = "the result came out with holes";
                    doc.scene.undo();
                    return out.fail("The " + what + " copy " + std::to_string(done + 1) + " couldn't be " +
                                    (op == BooleanOp::Union ? "joined" : "cut") + ": " + e + " (nothing changed)");
                }
                ++done;
            }
            const int targetAt = ti > bi ? ti - 1 : ti; // where the target is once the original goes
            if (ti != bi) doc.scene.bodies().erase(doc.scene.bodies().begin() + bi); // the original was used up
            doc.selectedBody = targetAt;
            doc.selectedCurve = doc.selectedImage = -1;
            doc.scene.touchBodies();
            doc.dirty = true;
            const Body& t = doc.scene.bodies()[static_cast<std::size_t>(targetAt)];
            out.say(std::string(op == BooleanOp::Union ? "Joined " : "Cut ") + std::to_string(done) + " copies of " + src.name + " (" + what + ")" +
                    (op == BooleanOp::Union ? " into " : " from ") + t.name + ": " + closedText(t.mesh));
            return out;
        }
        int next = 2;
        for (MeshData& m : copies) {
            while (bodyNamed(doc, src.name + " (" + std::to_string(next) + ")") >= 0) ++next; // names stay unique
            Body b = featureBody(src.name + " (" + std::to_string(next++) + ")", std::move(m), src.color);
            b.component = src.component;
            b.assembly = src.assembly;
            doc.scene.add(std::move(b));
        }
        // Pattern copies: the body itself, moved rigidly. Its kind, exact geometry and
        // normals stay; a shifted copy keeps the very same mesh.
        for (const Mat4& w : placements) {
            if (cmd != "PATTERN") break;
            while (bodyNamed(doc, src.name + " (" + std::to_string(next) + ")") >= 0) ++next;
            Body b = src;
            b.name = src.name + " (" + std::to_string(next++) + ")";
            b.attachedTo.clear();
            const Vec3 shift = w.transformPoint(src.offset) - src.offset;
            const bool turned = std::fabs(w.m[0][0] - 1) > 1e-7f || std::fabs(w.m[1][1] - 1) > 1e-7f || std::fabs(w.m[2][2] - 1) > 1e-7f;
            if (turned) {
                // World = offset + R_body p: turning about the pattern's axis turns the body
                // (about its origin) and carries its origin round.
                Placement turn;
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c) turn.r[r * 3 + c] = w.m[c][r];
                const Placement now = eulerPlacement(decimal(src.rotation)).then(turn);
                b.setRotation(eulerDegrees(now).f());
            }
            b.offset = src.offset + shift;
            doc.scene.add(std::move(b));
        }
        doc.selectedBody = static_cast<int>(doc.scene.bodies().size()) - 1;
        doc.dirty = true;
        const std::size_t made = copies.size() + (cmd == "PATTERN" ? placements.size() : 0);
        out.say("Made " + std::to_string(made) + (made == 1 ? " copy" : " copies") + " of " + src.name + " (" + what +
                (cmd == "PATTERN" ? " pattern)" : ")"));
        return out;
    }

    if (cmd == "EDGES") {
        // EDGES [body]: the sharp edges FILLET / CHAMFER ... EDGES can pick.
        std::size_t i = 0;
        const std::string name = leadingName(a, quoted, i, [](const std::string&) { return false; });
        const int bi = name.empty() ? doc.selectedBody : bodyNamed(doc, name);
        if (bi < 0 || bi >= static_cast<int>(doc.scene.bodies().size()))
            return out.fail(name.empty() ? "Select a body, or name it" : "No body named " + name);
        const Body& b = doc.scene.bodies()[static_cast<std::size_t>(bi)];
        const auto chains = edgeChains(worldMesh(b));
        out.say(b.name + ": " + std::to_string(chains.size()) + " sharp edges");
        for (std::size_t k = 0; k < chains.size(); ++k) {
            const EdgeChain& c = chains[k];
            const Vec3 d = c.pts.back() - c.pts.front();
            const char* dir = c.closed ? "loop" : std::fabs(d.z) > 0.999f * length(d) ? "vertical" : std::fabs(d.z) < 1e-3f * length(d) ? "horizontal" : "slanted";
            out.say("  " + std::to_string(k + 1) + ". " + (c.convex ? "outside " : "inside ") + dir + ", " + lengthText(c.length()) + " from " +
                    lengthsText({c.pts.front().x, c.pts.front().y, c.pts.front().z}, ", ", "(", ")") +
                    (c.closed ? std::string() : " to " + lengthsText({c.pts.back().x, c.pts.back().y, c.pts.back().z}, ", ", "(", ")")));
        }
        return out;
    }

    if (cmd == "FILLET" || cmd == "CHAMFER") {
        // FILLET [body] RADIUS r [CONVEX|CONCAVE|BOTH] [VOXEL v] [EDGES ...]
        // CHAMFER [body] DISTANCE d [EDGES ...]
        //   EDGES ALL | CONVEX | CONCAVE | VERTICAL | HORIZONTAL | TOP | BOTTOM | n ... | NEAR x,y,z ...
        const bool chamfer = cmd == "CHAMFER";
        auto isKey = [](const std::string& w) {
            return w == "radius" || w == "r" || w == "convex" || w == "concave" || w == "both" || w == "voxel" || w == "edges" || w == "edge" ||
                   w == "distance" || w == "d" || w == "size";
        };
        std::size_t i = 0;
        const std::string name = leadingName(a, quoted, i, isKey);
        const int bi = name.empty() ? doc.selectedBody : bodyNamed(doc, name);
        if (bi < 0 || bi >= static_cast<int>(doc.scene.bodies().size()))
            return out.fail(name.empty() ? std::string("Select a body to ") + (chamfer ? "chamfer" : "round") + ", or name it" : "No body named " + name);
        RoundOptions o;
        bool radiusGiven = false, edges = chamfer;
        EdgeFilter ef;
        for (; i < a.size(); ++i) {
            const std::string k = lower(a[i]);
            if (k == "edges" || k == "edge") {
                edges = true;
                for (++i; i < a.size(); ++i) {
                    const std::string w = lower(a[i]);
                    if (w == "all") continue;
                    if (w == "convex" || w == "outside") ef.convex = true;
                    else if (w == "concave" || w == "inside") ef.concave = true;
                    else if (w == "vertical") ef.vertical = true;
                    else if (w == "horizontal") ef.horizontal = true;
                    else if (w == "top") ef.top = true;
                    else if (w == "bottom") ef.bottom = true;
                    else if (w == "near") {
                        while (i + 1 < a.size()) {
                            const auto v = parseVec3(a[i + 1]);
                            if (!v) break;
                            ef.near.push_back(*v);
                            ++i;
                        }
                        if (ef.near.empty()) return out.fail("NEAR needs x,y,z points by the edges");
                    } else if (const auto n = parseNumber(a[i]); n && *n >= 1 && std::floor(*n) == *n) ef.numbers.push_back(static_cast<int>(*n));
                    else {
                        --i; // back to the options
                        break;
                    }
                }
                continue;
            }
            if (k == "radius" || k == "r" || k == "voxel" || k == "distance" || k == "d" || k == "size") {
                const auto v = i + 1 < a.size() ? parseLength(a[++i]) : std::nullopt;
                if (!v || !(*v > 0)) return out.fail(toUpper(k) + " needs a length");
                (k == "voxel" ? o.voxel : o.radius) = static_cast<float>(*v);
                radiusGiven = radiusGiven || k != "voxel";
            } else if (k == "convex") o.convex = true, o.concave = false;
            else if (k == "concave") o.convex = false, o.concave = true;
            else if (k == "both") o.convex = o.concave = true;
            else if (const auto v = parseLength(a[i]); v && *v > 0 && !radiusGiven) {
                o.radius = static_cast<float>(*v);
                radiusGiven = true;
            } else
                return out.fail(chamfer ? "Usage: CHAMFER [body] DISTANCE d [EDGES ALL|CONVEX|CONCAVE|VERTICAL|HORIZONTAL|TOP|BOTTOM|n...|NEAR x,y,z...]"
                                        : "Usage: FILLET [body] RADIUS r [CONVEX|CONCAVE|BOTH] [VOXEL v] [EDGES ALL|CONVEX|CONCAVE|VERTICAL|HORIZONTAL|"
                                          "TOP|BOTTOM|n...|NEAR x,y,z...]");
        }
        if (!radiusGiven) return out.fail(chamfer ? "CHAMFER needs DISTANCE d" : "FILLET needs RADIUS r");
        if (edges) { // exact, on the chosen edges
            Body& body = doc.scene.bodies()[static_cast<std::size_t>(bi)];
            const MeshData world = worldMesh(body);
            const auto chains = edgeChains(world);
            if (chamfer && ef.numbers.empty() && ef.near.empty() && !ef.concave) ef.convex = true; // a chamfer defaults to the outside edges
            const auto chosen = selectEdges(chains, world, ef);
            if (chosen.empty()) return out.fail("No edges match (EDGES lists " + body.name + "'s)");
            const double before = meshVolume(world);
            doc.scene.pushUndo(cmd + " " + body.name);
            std::string e;
            if (!roundEdges(body, chains, chosen, o.radius, chamfer, &e)) {
                doc.scene.undo();
                return out.fail(std::string("Cannot ") + (chamfer ? "chamfer " : "round ") + doc.scene.bodies()[static_cast<std::size_t>(bi)].name + ": " + e);
            }
            doc.scene.touchBodies();
            doc.dirty = true;
            std::string list;
            for (int k : chosen) list += (list.empty() ? "" : ", ") + std::to_string(k + 1);
            out.say(std::string(chamfer ? "Chamfered " : "Rounded ") + std::to_string(chosen.size()) + (chosen.size() == 1 ? " edge" : " edges") + " (" + list +
                    ") of " + body.name + (chamfer ? " by " : " with R") + lengthText(o.radius) + ": " + closedText(body.mesh) + ", volume " +
                    fmt3("%.1f", before / 1000.0) + " -> " + fmt3("%.1f", meshVolume(body.mesh) / 1000.0) + " cm3");
            return out;
        }
        if (!radiusGiven) return out.fail("FILLET needs RADIUS r");
        Body& body = doc.scene.bodies()[static_cast<std::size_t>(bi)];
        const double before = meshVolume(worldMesh(body));
        doc.scene.pushUndo("FILLET " + body.name);
        std::string e;
        if (!roundBody(body, o, &e)) {
            doc.scene.undo();
            return out.fail("Cannot round " + doc.scene.bodies()[static_cast<std::size_t>(bi)].name + ": " + e);
        }
        doc.scene.touchBodies();
        doc.dirty = true;
        const double after = meshVolume(body.mesh);
        out.say("Rounded the " + std::string(o.convex && o.concave ? "convex and concave" : o.convex ? "convex" : "concave") + " edges of " + body.name +
                " with R" + lengthText(o.radius) + ": " + closedText(body.mesh) + ", volume " + fmt3("%.1f", before / 1000.0) + " -> " +
                fmt3("%.1f", after / 1000.0) + " cm3");
        return out;
    }

    if (cmd == "MASSPROPS") {
        auto isKey = [](const std::string& w) { return w == "density" || w == "material"; };
        std::size_t i = 0;
        const std::string name = leadingName(a, quoted, i, isKey);
        const int bi = name.empty() ? doc.selectedBody : bodyNamed(doc, name);
        if (bi < 0 || bi >= static_cast<int>(doc.scene.bodies().size()))
            return out.fail(name.empty() ? "Select a body, or name it" : "No body named " + name);
        double rho = 7.85;
        std::string material = "steel";
        for (; i < a.size(); ++i) {
            const std::string k = lower(a[i]);
            if (k == "density") {
                const auto v = i + 1 < a.size() ? parseNumber(a[++i]) : std::nullopt;
                if (!v || !(*v > 0)) return out.fail("DENSITY needs g/cm3");
                rho = *v;
                material = fmt3("%g g/cm3", rho);
            } else if (k == "material") {
                const std::string m = i + 1 < a.size() ? a[++i] : std::string();
                const auto d = materialDensity(m);
                if (!d) return out.fail("Unknown material " + m + " (steel, aluminium, brass, copper, titanium, abs, pla, wood, ...)");
                rho = *d;
                material = lower(m);
            } else return out.fail("Usage: MASSPROPS [body] [DENSITY g/cm3 | MATERIAL name]");
        }
        const Body& b = doc.scene.bodies()[static_cast<std::size_t>(bi)];
        const MeshData w = worldMesh(b);
        const SolidMass mp = solidMass(w, rho);
        const Vec3 s = mp.box.max - mp.box.min;
        out.say(b.name + " (" + material + ", " + fmt3("%g", rho) + " g/cm3)" + (openEdgeCount(w) ? "  - not closed: the volume is approximate" : ""));
        out.say("  volume " + fmt3("%.3f", mp.volume / 1000.0) + " cm3, surface " + fmt3("%.2f", mp.area / 100.0) + " cm2, mass " + fmt3("%.2f", mp.mass) + " g");
        out.say("  centre of mass " + lengthsText({mp.centroid.x, mp.centroid.y, mp.centroid.z}, ", ", "(", ")") + ", size " +
                lengthsText({s.x, s.y, s.z}, " x ", "", ""));
        char line[200];
        const double scale = std::max({std::fabs(mp.inertia[0][0]), std::fabs(mp.inertia[1][1]), std::fabs(mp.inertia[2][2])});
        auto tidy = [&](double v) { return std::fabs(v) < scale * 1e-7 ? 0.0 : v; }; // rounding noise reads 0, not -0
        std::snprintf(line, sizeof(line), "  inertia about the centre of mass (g mm2): Ixx %.4g  Iyy %.4g  Izz %.4g  Ixy %.4g  Iyz %.4g  Ixz %.4g",
                      tidy(mp.inertia[0][0]), tidy(mp.inertia[1][1]), tidy(mp.inertia[2][2]), tidy(mp.inertia[0][1]), tidy(mp.inertia[1][2]),
                      tidy(mp.inertia[0][2]));
        out.say(line);
        std::snprintf(line, sizeof(line), "  principal moments (g mm2): %.4g  %.4g  %.4g", mp.principal[0], mp.principal[1], mp.principal[2]);
        out.say(line);
        return out;
    }

    if (cmd == "MEASURE") {
        if (a.size() >= 2) {
            const auto p = parseVec3(a[0]), q = parseVec3(a[1]);
            if (p && q && !quotedAt(0) && !quotedAt(1)) {
                const Vec3 d = *q - *p;
                out.say("Distance " + lengthText(length(d)) + "  (dx " + lengthText(d.x) + ", dy " + lengthText(d.y) + ", dz " + lengthText(d.z) + ")");
                return out;
            }
        }
        // Two bodies: quoted names, or names split by TO.
        std::vector<std::string> names;
        std::string cur;
        for (std::size_t i = 0; i < a.size(); ++i) {
            if (quotedAt(i)) {
                if (!cur.empty()) names.push_back(cur), cur.clear();
                names.push_back(a[i]);
            } else if (lower(a[i]) == "to" || lower(a[i]) == "and") {
                if (!cur.empty()) names.push_back(cur), cur.clear();
            } else cur += (cur.empty() ? "" : " ") + a[i];
        }
        if (!cur.empty()) names.push_back(cur);
        if (names.size() == 1 && doc.selectedBody >= 0) names.insert(names.begin(), doc.scene.bodies()[static_cast<std::size_t>(doc.selectedBody)].name);
        if (names.size() != 2) return out.fail("Usage: MEASURE x,y,z x,y,z  |  MEASURE \"body\" \"body\"");
        const int ia = bodyNamed(doc, names[0]), ib = bodyNamed(doc, names[1]);
        if (ia < 0) return out.fail("No body named " + names[0]);
        if (ib < 0) return out.fail("No body named " + names[1]);
        Vec3 pa, pb;
        const MeshData ma = worldMesh(doc.scene.bodies()[static_cast<std::size_t>(ia)]), mb = worldMesh(doc.scene.bodies()[static_cast<std::size_t>(ib)]);
        const double d = meshDistance(ma, mb, &pa, &pb);
        if (d < 0) return out.fail("Both bodies need a surface");
        out.say("Closest distance between " + names[0] + " and " + names[1] + ": " + lengthText(d) + (d < 1e-3 ? " (touching or overlapping)" : ""));
        out.say("  from " + lengthsText({pa.x, pa.y, pa.z}, ", ", "(", ")") + " to " + lengthsText({pb.x, pb.y, pb.z}, ", ", "(", ")"));
        return out;
    }
    return out.fail("Unknown feature command " + cmd);
}

// The arguments from `from` on as a command line again (quoted ones re-quoted).
std::string rejoinArgs(const std::vector<std::string>& a, const std::vector<bool>& quoted, std::size_t from) {
    std::string s;
    for (std::size_t i = from; i < a.size(); ++i) {
        std::string w = a[i];
        if (i < quoted.size() && quoted[i]) {
            std::string q = "\"";
            for (char c : w) q += c == '"' ? std::string("\\\"") : std::string(1, c);
            w = q + "\"";
        }
        s += (s.empty() ? "" : " ") + w;
    }
    return s;
}

// Rebuilds the model from `steps`. On a failure the model and history are left as
// they were and the failing step is reported.
bool regenerateHistory(Document& doc, const std::vector<HistoryStep>& steps, CommandOutput& out) {
    const Scene saved = doc.scene;
    const ProjectState savedState = doc.state;
    const int sb = doc.selectedBody, sc = doc.selectedCurve, si = doc.selectedImage, serial = doc.primitiveSerial;
    const bool mm = doc.apiMillimeters;
    doc.scene.clear();
    doc.state.assembly = {}; // the MATE steps put the mates back
    doc.clearSelection();
    doc.primitiveSerial = 0;
    doc.state.plane = SketchPlane::Top;
    doc.state.customPlane.clear();
    doc.state.datums.clear(); // the PLANE steps make them again
    doc.replaying = true;
    for (std::size_t k = 0; k < steps.size(); ++k) {
        doc.apiMillimeters = steps[k].millimetres;
        doc.state.units.unit = steps[k].unit;
        doc.replayStep = static_cast<int>(k);
        // The selection the step ran with.
        doc.selectedBody = steps[k].selectedBody.empty() ? -1 : bodyNamed(doc, steps[k].selectedBody);
        doc.selectedCurve = steps[k].selectedCurve.empty() ? -1 : curveNamed(doc, steps[k].selectedCurve);
        const CommandOutput r = executeCommand(doc, steps[k].line, nullptr);
        if (!r.ok) {
            doc.replaying = false;
            doc.replayStep = -1;
            doc.apiMillimeters = mm;
            doc.scene.restore(Scene(saved));
            doc.state = savedState;
            doc.selectedBody = sb, doc.selectedCurve = sc, doc.selectedImage = si, doc.primitiveSerial = serial;
            out.fail("Regeneration stopped at step " + std::to_string(k + 1) + " (" + steps[k].line + "): " +
                     (r.lines.empty() ? std::string("failed") : r.lines.back()));
            out.say("The model and its history are as they were.");
            return false;
        }
    }
    doc.replaying = false;
    doc.replayStep = -1;
    doc.apiMillimeters = mm;
    doc.state.units = savedState.units;
    doc.state.history = steps;
    doc.state.historyRedo.clear();
    doc.state.historyComplete = true;
    doc.state.historyRevision = doc.scene.revision();
    doc.dirty = true;
    for (CamOperation& op : doc.state.cam.ops) op.generated = false; // the part was rebuilt
    for (LatheOperation& op : doc.state.lathe.ops) op.generated = false;
    return true;
}


CommandOutput runHistoryCommand(Document& doc, const std::string& cmd, const std::vector<std::string>& a, const std::vector<bool>& quoted) {
    CommandOutput out;
    ProjectState& st = doc.state;
    const std::string sub = cmd == "REGEN" ? "regen" : a.empty() ? "list" : lower(a[0]);
    if (sub == "list") {
        if (st.history.empty()) {
            out.say("The history is empty: the model commands you type are recorded here, and REGEN replays them");
            return out;
        }
        out.say("History: " + std::to_string(st.history.size()) + " steps" +
                (st.historyComplete ? std::string(" (REGEN rebuilds the model from them)")
                                    : std::string(" - the model was also changed outside the command line (brush strokes, drags); REGEN FORCE would lose that")));
        for (std::size_t k = 0; k < st.history.size(); ++k) out.say("  " + std::to_string(k + 1) + ". " + st.history[k].line);
        return out;
    }
    if (sub == "clear") {
        st.history.clear();
        st.historyRedo.clear();
        st.historyComplete = true;
        out.say("History cleared (the model stays)");
        return out;
    }
    if (sub == "regen") {
        const bool force = (cmd == "REGEN" && !a.empty() && lower(a[0]) == "force") || (cmd != "REGEN" && a.size() > 1 && lower(a[1]) == "force");
        if (st.history.empty()) return out.fail("Nothing to regenerate: the history is empty");
        if (!st.historyComplete && !force)
            return out.fail("The model was changed outside the command line since the history began (brush strokes, drags): REGEN FORCE rebuilds "
                            "from the history and loses those changes");
        const std::vector<HistoryStep> steps = st.history;
        if (!regenerateHistory(doc, steps, out)) return out;
        out.say("Regenerated " + std::to_string(steps.size()) + " steps: " + std::to_string(doc.scene.bodies().size()) + " bodies, " +
                std::to_string(doc.scene.sketch().curves().size()) + " sketch curves");
        return out;
    }
    if (sub == "edit" || sub == "insert" || sub == "delete") {
        const auto n = a.size() > 1 ? parseNumber(a[1]) : std::nullopt;
        const std::size_t max = st.history.size() + (sub == "insert" ? 1 : 0);
        if (!n || *n < 1 || *n > static_cast<double>(max)) return out.fail("HISTORY " + toUpper(sub) + " n: a step's number (HISTORY lists them)");
        const std::size_t k = static_cast<std::size_t>(*n) - 1;
        std::vector<HistoryStep> steps = st.history;
        if (sub == "delete") {
            steps.erase(steps.begin() + static_cast<std::ptrdiff_t>(k));
        } else {
            const std::string line = rejoinArgs(a, quoted, 2);
            if (line.empty()) return out.fail("HISTORY " + toUpper(sub) + " n command...: the new command");
            HistoryStep step;
            step.line = line;
            step.unit = st.units.unit;
            if (sub == "edit") {
                step.undoable = steps[k].undoable;
                step.selectedBody = steps[k].selectedBody; // the step's selection stays
                step.selectedCurve = steps[k].selectedCurve;
                step.millimetres = false;
                steps[k] = step;
            } else
                steps.insert(steps.begin() + static_cast<std::ptrdiff_t>(k), step);
        }
        if (!regenerateHistory(doc, steps, out)) return out;
        out.say((sub == "delete" ? "Deleted step " : sub == "edit" ? "Changed step " : "Inserted step ") + std::to_string(k + 1) + " and regenerated " +
                std::to_string(steps.size()) + " steps");
        return out;
    }
    return out.fail("Usage: HISTORY [LIST] | HISTORY EDIT n command | HISTORY INSERT n command | HISTORY DELETE n | HISTORY CLEAR | REGEN [FORCE]");
}

} // namespace sicdm

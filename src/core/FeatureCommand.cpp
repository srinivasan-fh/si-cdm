// REVOLVE, SWEEP, LOFT, PATTERN, MIRROR, FILLET, MASSPROPS, MEASURE.

#include "core/FeatureCommand.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Extrude.hpp"
#include "core/ExactMesh.hpp"
#include "core/Brep.hpp"
#include "core/Features.hpp"
#include "core/HoleWizard.hpp"
#include "core/SheetMetal.hpp"
#include "core/Fea.hpp"
#include "core/FeaReport.hpp"
#include "core/FileIO.hpp"
#include "core/Sculpt.hpp"
#include "core/SketchEdit.hpp"
#include "core/Step.hpp"
#include "core/Surfacing.hpp"
#include "core/TextOutline.hpp"
#include "core/Units.hpp"

#include <algorithm>
#include <unordered_set>
#include <map>
#include <tuple>
#include <array>
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
    if (c.spatial()) { // a 3D path: its bends or its spline, finely
        spacePoints(c, &closed);
        path = c.worldStrip(96);
        if (closed && path.size() > 1) path.pop_back();
        return path.size() >= 2;
    }
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

std::string pointText(const Vec3& v) { return lengthsText({v.x, v.y, v.z}, ", ", "(", ")"); }

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
bool combineTool(Document& doc, MeshData tool, BooleanOp op, const std::string& target, std::string& result, std::string& why,
                 const std::vector<BSurface>& toolSurfaces = {}) {
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
            if (!bodyBoolean(trial, toolBody, BooleanOp::Subtract, &e, toolSurfaces)) continue; // only its box overlaps
            if (wasClosed && openEdgeCount(trial.mesh) > 0) return why = "cutting " + b.name + " left holes in it", false;
            b = std::move(trial);
            done.push_back(b.name);
        }
        if (done.empty()) return why = "it meets no body to cut (or would cut one away entirely)", false;
    } else {
        Body& into = doc.scene.bodies()[static_cast<std::size_t>(hits[0])];
        const bool wasClosed = openEdgeCount(into.mesh) == 0;
        std::string e;
        if (!bodyBoolean(into, toolBody, BooleanOp::Union, &e, toolSurfaces)) return why = e, false;
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

// The point of a mesh nearest `at`, and the normal of the triangle it is on.
bool nearestFace(const MeshData& m, Vec3 at, Vec3& q, Vec3& n) {
    float best = 1e30f;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const Vec3 p0 = m.vertices[m.indices[t]].position, p1 = m.vertices[m.indices[t + 1]].position, p2 = m.vertices[m.indices[t + 2]].position;
        const Vec3 nn = cross(p1 - p0, p2 - p0);
        if (!(length(nn) > 0)) continue;
        const Vec3 c = closestPointOnTriangle(at, p0, p1, p2);
        if (length(c - at) < best) best = length(c - at), q = c, n = normalize(nn);
    }
    return best < 1e30f;
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
            std::vector<BSurface> exact = exactSurfaces(b); // the tool's, in world space
            for (BSurface& q : exact) q.f.o = q.f.o + DVec3(b.offset);
            if (!combineTool(doc, worldMesh(b), *op, target, into, why, exact)) {
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
                else if (const DatumAxis* ra = findRefAxis(doc.state, w)) { // in the profile's plane
                    const PlaneFrame f = curveFrame(curve);
                    if (std::fabs(dot(ra->dir, f.normal)) > 1e-4f || std::fabs(dot(ra->origin - f.origin, f.normal)) > 1e-3f)
                        return out.fail(ra->name + " doesn't lie in " + curve.name + "'s plane");
                    const Vec2 p0 = f.toPlane(ra->origin);
                    axis = {{p0, Vec2{dot(ra->dir, f.u), dot(ra->dir, f.v)}}};
                    axisName = ra->name;
                } else if (const int li = curveNamed(doc, w); li >= 0) {
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
            std::vector<BSurface> surfaces;
            if (exact)
                for (const BSurface& q : exact->surfaces)
                    if (q.kind == BSurfaceKind::Plane || q.kind == BSurfaceKind::Cylinder || q.kind == BSurfaceKind::Cone || q.kind == BSurfaceKind::Sphere ||
                        q.kind == BSurfaceKind::Torus)
                        surfaces.push_back(q);
            if (!combineTool(doc, std::move(*mesh), *op, target, into, why, surfaces)) {
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
        // SWEEP [profile] HELIX PITCH p TURNS n [AXIS U | V | x,y x,y] [LEFT] [KEEP]
        auto isKey = [](const std::string& w) {
            return w == "along" || w == "path" || w == "keep" || w == "helix" || w == "pitch" || w == "turns" || w == "axis" || w == "left";
        };
        const char* usage = "Usage: SWEEP [profile] ALONG path [KEEP]  |  SWEEP [profile] HELIX PITCH p TURNS n [AXIS U|V|x,y x,y] [LEFT] [KEEP]";
        std::size_t i = 0;
        const std::string name = leadingName(a, quoted, i, isKey);
        std::string pathName;
        bool keep = false, helix = false, left = false;
        float pitch = 0, turns = 0;
        std::optional<std::pair<Vec2, Vec2>> axis; // in the profile's plane: point, direction
        for (; i < a.size(); ++i) {
            const std::string k = lower(a[i]);
            if (k == "keep") keep = true;
            else if (k == "helix") helix = true;
            else if (k == "left") left = true;
            else if (k == "pitch" && i + 1 < a.size()) {
                const auto v = parseLength(a[++i]);
                if (!v || !(*v > 0)) return out.fail("PITCH needs a length: how far it rises a turn");
                pitch = static_cast<float>(*v);
            } else if (k == "turns" && i + 1 < a.size()) {
                const auto v = parseNumber(a[++i]);
                if (!v || !(*v > 0) || *v > 500) return out.fail("TURNS is more than 0, up to 500");
                turns = *v;
            } else if (k == "axis" && i + 1 < a.size()) {
                const std::string w = lower(a[++i]);
                if (w == "u") axis = {{Vec2{0, 0}, Vec2{1, 0}}};
                else if (w == "v") axis = {{Vec2{0, 0}, Vec2{0, 1}}};
                else {
                    const auto p0 = point2(a[i]);
                    const auto p1 = i + 1 < a.size() ? point2(a[i + 1]) : std::nullopt;
                    if (!p0 || !p1) return out.fail("AXIS needs U, V or two points x,y x,y in the profile's plane");
                    ++i;
                    axis = {{*p0, Vec2{p1->x - p0->x, p1->y - p0->y}}};
                }
            } else if (k == "along" || k == "path") {
                ++i;
                pathName = leadingName(a, quoted, i, isKey);
                --i;
            } else return out.fail(usage);
        }
        const int ci = name.empty() ? doc.selectedCurve : curveNamed(doc, name);
        if (ci < 0 || ci >= static_cast<int>(doc.scene.sketch().curves().size())) return out.fail("Name the closed profile to sweep (or select it)");
        const SketchCurve profileCurve = doc.scene.sketch().curves()[static_cast<std::size_t>(ci)];
        std::string why;
        const auto profile = profileFromSketch(doc.scene.sketch(), static_cast<std::size_t>(ci), &why);
        if (!profile) return out.fail("Cannot sweep " + profileCurve.name + ": " + why);
        const PlaneFrame pf = curveFrame(profileCurve);
        std::vector<Vec3> path;
        bool closed = false;
        int pi = -1;
        std::string along;
        if (helix) {
            if (!pathName.empty()) return out.fail("A sweep goes ALONG a path or round a HELIX, not both");
            if (!(pitch > 0) || !(turns > 0)) return out.fail("HELIX needs PITCH p and TURNS n");
            // Round the axis in the profile's plane (its vertical axis by default), from the
            // profile's centre, rising `pitch` a turn.
            const auto [ap, ad] = axis.value_or(std::pair<Vec2, Vec2>{Vec2{0, 0}, Vec2{0, 1}});
            const float adl = std::hypot(ad.x, ad.y);
            if (!(adl > 0)) return out.fail("The helix axis needs a direction");
            const Vec3 A = pf.toWorld(ap), D = normalize(pf.u * ad.x + pf.v * ad.y);
            const std::vector<Vec2> poly = profilePolygon(*profile);
            Vec2 c{};
            for (const Vec2& q : poly) c = {c.x + q.x, c.y + q.y};
            c = {c.x / static_cast<float>(poly.size()), c.y / static_cast<float>(poly.size())};
            const Vec3 C = pf.toWorld(c);
            const Vec3 radial = (C - A) - D * dot(C - A, D);
            const float R = length(radial);
            float reach = 0; // the profile must stay off the axis
            for (const Vec2& q : poly) reach = std::max(reach, length(pf.toWorld(q) - C));
            if (!(R > reach)) return out.fail("The profile crosses the helix axis: move it out, or choose another AXIS");
            if (pitch < 2 * reach * 0.999f) return out.fail("The pitch is less than the profile's width: the coils would run into each other");
            const Vec3 e1 = radial / R, e2 = cross(D, e1) * (left ? -1.0f : 1.0f);
            const Vec3 base = C - radial;
            const int steps = std::max(8, static_cast<int>(std::ceil(turns * 48)));
            for (int k = 0; k <= steps; ++k) {
                const float f = turns * static_cast<float>(k) / static_cast<float>(steps), th = 2 * kPi * f;
                path.push_back(base + D * (pitch * f) + (e1 * std::cos(th) + e2 * std::sin(th)) * R);
            }
            along = "a helix (" + lengthText(pitch) + " pitch, " + fmt3("%g", turns) + " turns" + (left ? ", left hand" : "") + ")";
        } else {
            pi = curveNamed(doc, pathName);
            if (pathName.empty() || pi < 0) return out.fail("Give the path: SWEEP profile ALONG \"path curve\", or HELIX PITCH p TURNS n");
            if (pi == ci) return out.fail("The profile and the path must be different curves");
            const SketchCurve pathCurve = doc.scene.sketch().curves()[static_cast<std::size_t>(pi)];
            if (!curvePath(pathCurve, path, closed)) return out.fail(pathCurve.name + " can't be a path (a line, polyline, rectangle, circle or 3D path)");
            along = pathCurve.name;
        }
        auto mesh = sweepProfile(*profile, pf, path, closed, &why);
        if (!mesh) return out.fail("Cannot sweep: " + why);
        const int serial = ++doc.primitiveSerial;
        Body b = featureBody("Sweep " + std::to_string(serial), std::move(*mesh), bodyColor(serial));
        if (!keep) {
            hideCurve(doc, ci);
            if (pi >= 0) hideCurve(doc, pi);
        }
        doc.scene.pushUndo("SWEEP " + profileCurve.name);
        out.say("Swept " + profileCurve.name + " along " + along + " -> " + b.name + ": " + closedText(b.mesh));
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
                                            "union", "subtract", "plane", "normal", "copy", "move", "into", "xy", "yz", "xz", "zx",
                                            "along", "align", "points", "fill", "stagger", "margin", "ref"};
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
        // Its exact surfaces in world space, and how each mesh copy below was moved (so
        // copies and the bodies they join keep exact geometry).
        std::vector<BSurface> exactWorld = exactSurfaces(src);
        for (BSurface& q : exactWorld) q.f.o = q.f.o + DVec3(src.offset);
        std::vector<Mat4> copyMoves;
        auto movedSurfaces = [&](const Mat4& t) {
            std::vector<BSurface> moved = exactWorld;
            auto pt = [&](const DVec3& p) {
                return DVec3{t.m[0][0] * p.x + t.m[1][0] * p.y + t.m[2][0] * p.z + t.m[3][0], t.m[0][1] * p.x + t.m[1][1] * p.y + t.m[2][1] * p.z + t.m[3][1],
                             t.m[0][2] * p.x + t.m[1][2] * p.y + t.m[2][2] * p.z + t.m[3][2]};
            };
            auto dir = [&](const DVec3& d) {
                return dnormalize(DVec3{t.m[0][0] * d.x + t.m[1][0] * d.y + t.m[2][0] * d.z, t.m[0][1] * d.x + t.m[1][1] * d.y + t.m[2][1] * d.z,
                                        t.m[0][2] * d.x + t.m[1][2] * d.y + t.m[2][2] * d.z});
            };
            for (BSurface& q : moved) q.f = Frame::make(pt(q.f.o), dir(q.f.z), dir(q.f.x));
            return moved;
        };
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
            // Along a curve, at sketch points, or filling a region.
            std::string alongName, fillName;
            bool align = false, points = false, stagger = false;
            float margin = 0;
            std::optional<Vec3> ref;
            for (; i < a.size(); ++i) {
                const std::string k = lower(a[i]);
                auto next = [&]() -> std::string { return i + 1 < a.size() ? a[++i] : std::string(); };
                if (k == "along" || k == "fill") {
                    ++i;
                    (k == "along" ? alongName : fillName) = leadingName(a, quoted, i, [&](const std::string& w) { return isKey(w) || parseNumber(w).has_value(); });
                    --i;
                    if (k == "along" && i + 1 < a.size()) {
                        if (const auto v = parseNumber(a[i + 1])) {
                            if (*v < 2 || *v > 500) return out.fail("The count is 2 to 500");
                            n = static_cast<int>(*v), ++i;
                        }
                    }
                } else if (k == "align") align = true;
                else if (k == "points") points = true;
                else if (k == "stagger") stagger = true;
                else if (k == "margin") {
                    const auto v = parseLength(next());
                    if (!v || *v < 0) return out.fail("MARGIN needs a length");
                    margin = static_cast<float>(*v);
                } else if (k == "ref") {
                    if (!(ref = parseVec3(next()))) return out.fail("REF needs the point x,y,z that goes onto each place");
                } else if (k == "linear" || k == "circular") {
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
                    const std::string w = next();
                    auto v = axisWord(w);
                    if (const DatumAxis* ra = v ? nullptr : findRefAxis(doc.state, w)) { // a reference axis: its line
                        v = ra->dir;
                        if (k == "axis" && !atGiven) at = ra->origin, atGiven = true;
                    }
                    if (!v) return out.fail(toUpper(k) + " needs X, Y, Z, x,y,z or a reference axis");
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
                                       "[AXIS X|Y|Z|x,y,z] [AT x,y,z] | ALONG curve n [ALIGN] | POINTS [REF x,y,z] | FILL curve SPACING d "
                                       "[SPACING2 d] [STAGGER] [MARGIN m] [REF x,y,z]  [UNION|SUBTRACT \"target\"]");
            }
            const bool special = !alongName.empty() || !fillName.empty() || points;
            auto place = [&](const Mat4& t) {
                placements.push_back(t);
                if (!merge) return;
                MeshData m = world;
                m.transform(t);
                copies.push_back(std::move(m));
                copyMoves.push_back(t);
            };
            if (special) {
                if ((!alongName.empty()) + (!fillName.empty()) + points > 1) return out.fail("ALONG, POINTS and FILL go one at a time");
                const Vec3 centre = src.bounds().center();
                if (!alongName.empty()) {
                    // n copies spaced evenly along a curve (the original at its start).
                    const int pc = curveNamed(doc, alongName);
                    if (pc < 0) return out.fail("No sketch curve named " + alongName);
                    if (n < 2) return out.fail("ALONG curve n: how many, the original the first");
                    std::vector<Vec3> path;
                    bool closedPath = false;
                    const SketchCurve& pcurve = doc.scene.sketch().curves()[static_cast<std::size_t>(pc)];
                    path.clear();
                    for (const Vec3& q : pcurve.worldStrip(96)) path.push_back(q);
                    closedPath = pcurve.isClosed();
                    if (closedPath && path.size() > 1 && length(path.front() - path.back()) < 1e-5f) path.pop_back();
                    if (path.size() < 2) return out.fail(alongName + " has no length");
                    std::vector<float> cum{0};
                    const std::size_t segs = closedPath ? path.size() : path.size() - 1;
                    for (std::size_t k = 0; k < segs; ++k) cum.push_back(cum.back() + length(path[(k + 1) % path.size()] - path[k]));
                    const float total = cum.back();
                    if (!(total > 1e-6f)) return out.fail(alongName + " has no length");
                    auto pointAt = [&](float sArc, Vec3& tangent) {
                        std::size_t k = static_cast<std::size_t>(std::upper_bound(cum.begin(), cum.end(), sArc) - cum.begin());
                        k = std::clamp<std::size_t>(k, 1, segs) - 1;
                        const Vec3 p0 = path[k], p1 = path[(k + 1) % path.size()];
                        const float l = std::max(cum[k + 1] - cum[k], 1e-9f);
                        tangent = normalize(p1 - p0);
                        return p0 + (p1 - p0) * ((sArc - cum[k]) / l);
                    };
                    Vec3 t0;
                    const Vec3 p0 = pointAt(0, t0);
                    for (int k = 1; k < n; ++k) {
                        const float sArc = total * static_cast<float>(k) / static_cast<float>(closedPath ? n : n - 1);
                        Vec3 tk;
                        const Vec3 pk = pointAt(std::min(sArc, total), tk);
                        Mat4 t = translate(pk - p0);
                        if (align) { // turned with the curve, about its own point on it
                            const Vec3 c = cross(t0, tk);
                            const float sn = length(c), cs = dot(t0, tk);
                            if (sn > 1e-6f) t = translate(pk) * rotateAxis(c / sn, std::atan2(sn, cs)) * translate(-p0);
                            else if (cs < 0) t = translate(pk) * rotateAxis(std::fabs(t0.z) < 0.9f ? normalize(cross(t0, Vec3{0, 0, 1})) : Vec3{1, 0, 0}, kPi) * translate(-p0);
                        }
                        place(t);
                    }
                    what = std::to_string(n) + " along " + pcurve.name + (align ? " (aligned)" : "");
                } else if (points) {
                    // A copy at each visible sketch point, its reference point (the body's centre
                    // on the points' plane, or REF) moved onto it.
                    std::vector<const SketchCurve*> pts;
                    for (const SketchCurve& c : doc.scene.sketch().curves())
                        if (c.visible && c.kind == CurveKind::Point && !c.pts.empty()) pts.push_back(&c);
                    if (pts.empty()) return out.fail("No sketch points to pattern at (POINT x,y makes them)");
                    const PlaneFrame f = curveFrame(*pts[0]);
                    const Vec3 r = ref ? *ref : centre - f.normal * dot(centre - f.origin, f.normal);
                    for (const SketchCurve* c : pts) {
                        const Vec3 off = curveFrame(*c).toWorld(c->pts[0]) - r;
                        if (length(off) < 1e-5f) continue; // where the original is
                        place(translate(off));
                    }
                    what = std::to_string(placements.size() + 1) + " at sketch points";
                } else {
                    // Filling a closed sketch profile on a grid SPACING apart (rows SPACING2,
                    // default the same; STAGGER shifts every other row), MARGIN in from its edge.
                    const int fc = curveNamed(doc, fillName);
                    if (fc < 0) return out.fail("No sketch curve named " + fillName);
                    if (!(spacing > 0)) return out.fail("FILL needs SPACING");
                    const float s2 = spacing2 > 0 ? spacing2 : spacing;
                    std::string why;
                    const auto prof = profileFromSketch(doc.scene.sketch(), static_cast<std::size_t>(fc), &why);
                    if (!prof) return out.fail("Cannot fill " + fillName + ": " + why);
                    const SketchCurve& fcurve = doc.scene.sketch().curves()[static_cast<std::size_t>(fc)];
                    const PlaneFrame f = curveFrame(fcurve);
                    const std::vector<Vec2> poly = profilePolygon(*prof);
                    const Vec3 r = ref ? *ref : centre - f.normal * dot(centre - f.origin, f.normal);
                    const Vec2 rp = f.toPlane(r);
                    auto inside = [&](Vec2 q) {
                        bool in = false;
                        for (std::size_t k = 0, j = poly.size() - 1; k < poly.size(); j = k++) {
                            const Vec2 a2 = poly[k], b2 = poly[j];
                            if ((a2.y > q.y) != (b2.y > q.y) && q.x < (b2.x - a2.x) * (q.y - a2.y) / (b2.y - a2.y) + a2.x) in = !in;
                        }
                        if (!in || margin <= 0) return in;
                        for (std::size_t k = 0; k < poly.size(); ++k) { // far enough from every side
                            const Vec2 a2 = poly[k], b2 = poly[(k + 1) % poly.size()];
                            const float dx = b2.x - a2.x, dy = b2.y - a2.y, l2 = dx * dx + dy * dy;
                            const float t = l2 > 0 ? std::clamp(((q.x - a2.x) * dx + (q.y - a2.y) * dy) / l2, 0.0f, 1.0f) : 0.0f;
                            if (std::hypot(q.x - (a2.x + dx * t), q.y - (a2.y + dy * t)) < margin) return false;
                        }
                        return true;
                    };
                    Vec2 lo = poly[0], hi = poly[0];
                    for (const Vec2& q : poly) lo = {std::min(lo.x, q.x), std::min(lo.y, q.y)}, hi = {std::max(hi.x, q.x), std::max(hi.y, q.y)};
                    // The grid runs through the reference point.
                    const long r0 = static_cast<long>(std::floor((lo.y - rp.y) / s2)) - 1, r1 = static_cast<long>(std::ceil((hi.y - rp.y) / s2)) + 1;
                    if (static_cast<float>(r1 - r0) * ((hi.x - lo.x) / spacing + 3) > 5000) return out.fail("That fills more than 5000 places: a wider SPACING");
                    int count = 0;
                    for (long row = r0; row <= r1; ++row) {
                        const float shift = stagger && (row % 2 != 0) ? spacing * 0.5f : 0.0f;
                        const long c0 = static_cast<long>(std::floor((lo.x - rp.x - shift) / spacing)) - 1;
                        const long c1 = static_cast<long>(std::ceil((hi.x - rp.x - shift) / spacing)) + 1;
                        for (long col = c0; col <= c1; ++col) {
                            const Vec2 q{rp.x + shift + spacing * static_cast<float>(col), rp.y + s2 * static_cast<float>(row)};
                            if (!inside(q)) continue;
                            ++count;
                            if (std::hypot(q.x - rp.x, q.y - rp.y) < 1e-5f) continue; // the original's place
                            place(translate(f.toWorld(q) - r));
                        }
                    }
                    if (placements.empty()) return out.fail("No place inside " + fillName + " for a copy (the reference point and SPACING set the grid)");
                    what = std::to_string(count) + " filling " + fcurve.name;
                }
            } else if (n < 2) return out.fail("Give LINEAR n, CIRCULAR n, ALONG curve n, POINTS or FILL curve");
            else if (!circular) {
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
                        copyMoves.push_back(move);
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
                    copyMoves.push_back(turn);
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
            Mat4 flip = Mat4::identity(); // p - 2 n (n . (p - point))
            const Vec3 nn = normalize(normal);
            const float nv[3] = {nn.x, nn.y, nn.z};
            for (int c = 0; c < 3; ++c)
                for (int r = 0; r < 3; ++r) flip.m[c][r] = (c == r ? 1.0f : 0.0f) - 2 * nv[r] * nv[c];
            const float k = 2 * dot(nn, point);
            flip.m[3][0] = nv[0] * k, flip.m[3][1] = nv[1] * k, flip.m[3][2] = nv[2] * k;
            copyMoves.push_back(flip);
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
            if (!exactWorld.empty()) keepExactGeometry(b, movedSurfaces(copyMoves[0]));
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
            if (ti != bi) copies.insert(copies.begin(), world), copyMoves.insert(copyMoves.begin(), Mat4::identity());
            std::size_t done = 0;
            const bool wasClosed = openEdgeCount(target.mesh) == 0;
            for (std::size_t c = 0; c < copies.size(); ++c) {
                const Body tool = meshBody("tool", std::move(copies[c]));
                std::string e;
                if (!bodyBoolean(target, tool, op, &e, c < copyMoves.size() ? movedSurfaces(copyMoves[c]) : std::vector<BSurface>{}) ||
                    (wasClosed && openEdgeCount(target.mesh) > 0)) {
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
        std::unordered_set<std::string> taken; // the names in use (a set: thousands of copies stay quick)
        for (const Body& b : doc.scene.bodies()) taken.insert(lower(b.name));
        auto freeName = [&]() {
            while (taken.count(lower(src.name + " (" + std::to_string(next) + ")"))) ++next;
            std::string n = src.name + " (" + std::to_string(next++) + ")";
            taken.insert(lower(n));
            return n;
        };
        for (std::size_t c = 0; c < copies.size(); ++c) {
            Body b = featureBody(freeName(), std::move(copies[c]), src.color);
            if (!exactWorld.empty() && c < copyMoves.size()) keepExactGeometry(b, movedSurfaces(copyMoves[c]));
            b.component = src.component;
            b.assembly = src.assembly;
            doc.scene.add(std::move(b));
        }
        // Pattern copies: the body itself, moved rigidly. Its kind, exact geometry and
        // normals stay; a shifted copy keeps the very same mesh.
        for (const Mat4& w : placements) {
            if (cmd != "PATTERN") break;
            Body b = src;
            b.name = freeName();
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
                   w == "distance" || w == "d" || w == "size" || w == "to" || w == "start" || w == "fullround";
        };
        std::size_t i = 0;
        const std::string name = leadingName(a, quoted, i, isKey);
        const int bi = name.empty() ? doc.selectedBody : bodyNamed(doc, name);
        if (bi < 0 || bi >= static_cast<int>(doc.scene.bodies().size()))
            return out.fail(name.empty() ? std::string("Select a body to ") + (chamfer ? "chamfer" : "round") + ", or name it" : "No body named " + name);
        RoundOptions o;
        bool radiusGiven = false, edges = chamfer;
        EdgeFilter ef;
        float radiusEnd = -1;              // TO r2: a variable fillet
        std::optional<Vec3> start, fullRound; // START x,y,z; FULLROUND x,y,z
        for (; i < a.size(); ++i) {
            const std::string k = lower(a[i]);
            if (k == "to") {
                const auto v = i + 1 < a.size() ? parseLength(a[++i]) : std::nullopt;
                if (!v || !(*v > 0)) return out.fail("TO needs the radius at the edge's other end");
                radiusEnd = static_cast<float>(*v);
                edges = true;
                continue;
            }
            if (k == "start" || k == "fullround") {
                const auto v = i + 1 < a.size() ? parseVec3(a[++i]) : std::nullopt;
                if (!v) return out.fail(toUpper(k) + " needs a point x,y,z");
                (k == "start" ? start : fullRound) = *v;
                edges = true;
                if (k == "fullround") radiusGiven = true; // the wall sets it
                continue;
            }
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
            std::vector<int> chosen;
            if (fullRound) {
                if (chamfer) return out.fail("FULLROUND is a fillet");
                // The two outside edges either side of the face at the point; the radius half the
                // wall between them, so the two rounds meet in one half circle.
                auto nearest = [&](const EdgeChain& c, Vec3& at) {
                    float best = 1e30f;
                    const std::size_t segs = c.closed ? c.pts.size() : c.pts.size() - 1;
                    for (std::size_t k2 = 0; k2 < segs; ++k2) {
                        const Vec3 p0 = c.pts[k2], p1 = c.pts[(k2 + 1) % c.pts.size()], d = p1 - p0;
                        const float l2 = dot(d, d);
                        const float t = l2 > 0 ? std::clamp(dot(*fullRound - p0, d) / l2, 0.0f, 1.0f) : 0.0f;
                        const Vec3 q = p0 + d * t;
                        if (length(q - *fullRound) < best) best = length(q - *fullRound), at = q;
                    }
                    return best;
                };
                int c1 = -1, c2 = -1;
                Vec3 q1{}, q2{};
                float d1 = 1e30f, d2 = 1e30f;
                for (std::size_t k2 = 0; k2 < chains.size(); ++k2) {
                    if (!chains[k2].convex || chains[k2].pts.size() < 2) continue;
                    Vec3 q;
                    const float dd = nearest(chains[k2], q);
                    if (dd < d1) c2 = c1, q2 = q1, d2 = d1, c1 = static_cast<int>(k2), q1 = q, d1 = dd;
                    else if (dd < d2) c2 = static_cast<int>(k2), q2 = q, d2 = dd;
                }
                if (c1 < 0 || c2 < 0) return out.fail("FULLROUND needs a face between two outside edges at the point");
                const Vec3 across = q2 - q1;
                // They must lie either side of the point.
                if (dot(*fullRound - q1, across) <= 0 || dot(*fullRound - q2, across) >= 0 || length(*fullRound - (q1 + q2) * 0.5f) > length(across))
                    return out.fail("FULLROUND: the point must be on the face between two edges");
                o.radius = 0.4995f * length(across);
                chosen = {c1, c2};
            } else {
                chosen = selectEdges(chains, world, ef);
            }
            if (chosen.empty()) return out.fail("No edges match (EDGES lists " + body.name + "'s)");
            const double before = meshVolume(world);
            doc.scene.pushUndo(cmd + " " + body.name);
            std::string e;
            if (!roundEdges(body, chains, chosen, o.radius, chamfer, &e, radiusEnd, start)) {
                doc.scene.undo();
                return out.fail(std::string("Cannot ") + (chamfer ? "chamfer " : "round ") + doc.scene.bodies()[static_cast<std::size_t>(bi)].name + ": " + e);
            }
            doc.scene.touchBodies();
            doc.dirty = true;
            std::string list;
            for (int k : chosen) list += (list.empty() ? "" : ", ") + std::to_string(k + 1);
            out.say(std::string(chamfer ? "Chamfered " : "Rounded ") + std::to_string(chosen.size()) + (chosen.size() == 1 ? " edge" : " edges") + " (" + list +
                    ") of " + body.name + (chamfer ? " by " : " with R") + lengthText(o.radius) +
                    (radiusEnd > 0 ? " to R" + lengthText(radiusEnd) : std::string()) + (fullRound ? " (a full round)" : "") + ": " + closedText(body.mesh) + ", volume " +
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

    if (cmd == "DRAFT") {
        // DRAFT [body] ANGLE deg [PULL x,y,z | X | Y | Z] [NEUTRAL x,y,z]
        auto isKey = [](const std::string& w) { return w == "angle" || w == "pull" || w == "neutral"; };
        const char* usage = "Usage: DRAFT [body] ANGLE deg [PULL x,y,z | X | Y | Z] [NEUTRAL x,y,z]";
        std::size_t i = 0;
        const std::string name = leadingName(a, quoted, i, isKey);
        const int bi = name.empty() ? doc.selectedBody : bodyNamed(doc, name);
        if (bi < 0 || bi >= static_cast<int>(doc.scene.bodies().size())) return out.fail(name.empty() ? "Select a body to draft, or name it" : "No body named " + name);
        std::optional<float> angle;
        Vec3 pull{0, 0, 1};
        std::optional<Vec3> neutral;
        for (; i < a.size(); ++i) {
            const std::string k = lower(a[i]);
            if (k == "angle" && i + 1 < a.size()) {
                const auto v = parseNumber(a[++i]);
                if (!v) return out.fail("ANGLE needs degrees");
                angle = *v;
            } else if (k == "pull" && i + 1 < a.size()) {
                std::string w = a[++i];
                const bool neg = !w.empty() && w[0] == '-' && w.find(',') == std::string::npos;
                const auto v = axisWord(neg ? w.substr(1) : w);
                if (!v) return out.fail("PULL needs X, Y, Z or x,y,z");
                pull = neg ? -*v : *v;
            } else if (k == "neutral" && i + 1 < a.size()) {
                if (!(neutral = parseVec3(a[++i]))) return out.fail("NEUTRAL needs a point x,y,z on the neutral plane");
            } else return out.fail(usage);
        }
        if (!angle) return out.fail(usage);
        Body& body = doc.scene.bodies()[static_cast<std::size_t>(bi)];
        const MeshData world = worldMesh(body);
        if (!neutral) { // the bottom, along the pull: the part narrows away from it
            float lo = 1e30f;
            for (const Vertex& v : world.vertices) lo = std::min(lo, dot(v.position, normalize(pull)));
            neutral = normalize(pull) * lo;
        }
        std::string why;
        int sides = 0;
        auto drafted = draftMesh(world, pull, *neutral, *angle, &why, &sides);
        if (!drafted) return out.fail("Cannot draft " + body.name + ": " + why);
        const double before = meshVolume(world);
        doc.scene.pushUndo("DRAFT " + body.name);
        Body old = body;
        Body made = featureBody(old.name, std::move(*drafted), old.color);
        made.visible = old.visible, made.component = old.component, made.assembly = old.assembly, made.attachedTo = old.attachedTo;
        made.physics = old.physics;
        for (HoleNote h : old.holes) { // the callouts keep their place in the world
            h.at = h.at + old.offset - made.offset;
            made.holes.push_back(h);
        }
        body = std::move(made);
        doc.scene.touchBodies();
        doc.dirty = true;
        out.say("Drafted " + body.name + " " + fmt3("%g", *angle) + " deg (" + std::to_string(sides) + " side triangles): " + closedText(body.mesh) +
                ", volume " + fmt3("%.1f", before / 1000.0) + " -> " + fmt3("%.1f", meshVolume(worldMesh(body)) / 1000.0) + " cm3");
        return out;
    }

    if (cmd == "THREAD") {
        // THREAD [body] AT x,y,z AXIS x,y,z SIZE M10 | DIAMETER d PITCH p  LENGTH l [INTERNAL] [LEFT]
        // THREAD [body] HOLES | HOLE n      the threads of its tapped hole wizard holes
        static const char* const kKeys[] = {"at", "axis", "size", "diameter", "d", "pitch", "length", "internal", "external", "left", "holes", "hole"};
        auto isKey = [](const std::string& w) {
            for (const char* k : kKeys)
                if (w == k) return true;
            return w.find(',') != std::string::npos;
        };
        const char* usage = "Usage: THREAD [body] AT x,y,z AXIS x,y,z SIZE M10 | DIAMETER d PITCH p  LENGTH l [INTERNAL] [LEFT]  |  THREAD [body] HOLES | HOLE n";
        std::size_t i = 0;
        const std::string name = leadingName(a, quoted, i, isKey);
        const int bi = name.empty() ? doc.selectedBody : bodyNamed(doc, name);
        if (bi < 0 || bi >= static_cast<int>(doc.scene.bodies().size())) return out.fail(name.empty() ? "Select a body to thread, or name it" : "No body named " + name);
        std::optional<Vec3> at, axis;
        float dia = 0, pitch = 0, len = 0;
        bool internal = false, left = false, holes = false;
        int holeNo = -1;
        std::string sizeName;
        for (; i < a.size(); ++i) {
            const std::string k = lower(a[i]);
            auto next = [&]() -> std::string { return i + 1 < a.size() ? a[++i] : std::string(); };
            if (k == "at") {
                if (!(at = parseVec3(next()))) return out.fail("AT needs a point x,y,z on the axis");
            } else if (k == "axis") {
                if (!(axis = axisWord(next()))) return out.fail("AXIS needs X, Y, Z or x,y,z");
            } else if (k == "size") {
                sizeName = next();
            } else if (k == "diameter" || k == "d" || k == "pitch" || k == "length") {
                const auto v = parseLength(next());
                if (!v || !(*v > 0)) return out.fail(toUpper(k) + " needs a length");
                (k == "pitch" ? pitch : k == "length" ? len : dia) = static_cast<float>(*v);
            } else if (k == "internal") internal = true;
            else if (k == "external") internal = false;
            else if (k == "left") left = true;
            else if (k == "holes") holes = true;
            else if (k == "hole") {
                const auto n = parseNumber(next());
                if (!n || *n < 1) return out.fail("HOLE takes the hole's number (HOLE LIST)");
                holeNo = static_cast<int>(*n) - 1;
            } else return out.fail(usage);
        }
        Body& body = doc.scene.bodies()[static_cast<std::size_t>(bi)];
        struct Job {
            float dia, pitch, len;
            bool internal;
            Vec3 at, axis;
            int note;
        };
        std::vector<Job> jobs;
        if (holes || holeNo >= 0) {
            const MeshData w = worldMesh(body);
            for (std::size_t k = 0; k < body.holes.size(); ++k) {
                const HoleNote& h = body.holes[k];
                if (holeNo >= 0 && static_cast<int>(k) != holeNo) continue;
                if (!(h.threadDia > 0) || h.threadModelled) {
                    if (holeNo >= 0) return out.fail("Hole " + std::to_string(holeNo + 1) + (h.threadModelled ? "'s thread is modelled already" : " isn't tapped"));
                    continue;
                }
                float depth = h.threadDepth;
                const Vec3 start = h.at + body.offset;
                if (!(depth > 0)) { // through: as far as the body goes along the hole
                    for (const Vertex& v : w.vertices) depth = std::max(depth, dot(v.position - start, normalize(h.dir)));
                }
                // From just above the surface, so the thread runs out of the hole's mouth.
                const float lead = h.pitch;
                jobs.push_back({h.threadDia, h.pitch, depth + lead, true, start - normalize(h.dir) * lead, normalize(h.dir), static_cast<int>(k)});
            }
            if (holeNo >= static_cast<int>(body.holes.size())) return out.fail(body.name + " has no hole " + std::to_string(holeNo + 1));
            if (jobs.empty()) return out.fail(body.name + " has no tapped holes left to thread (HOLE ... TAPPED makes them)");
        } else {
            if (!sizeName.empty()) {
                const HoleSize* z = findHoleSize(sizeName);
                if (!z) return out.fail("No standard size " + sizeName + " (HOLE SIZES lists them)");
                if (!(dia > 0)) dia = z->nominal;
                if (!(pitch > 0)) pitch = z->pitch;
            }
            if (!at || !axis || !(dia > 0) || !(pitch > 0) || !(len > 0)) return out.fail(usage);
            jobs.push_back({dia, pitch, len, internal, *at, *axis, -1});
        }
        doc.scene.pushUndo("THREAD " + body.name);
        const double before = meshVolume(worldMesh(body));
        std::string why;
        int done = 0;
        for (const Job& j : jobs) {
            auto groove = threadGroove(j.dia, j.pitch, j.len, j.internal, left, j.at, j.axis, &why);
            if (!groove || !bodyBoolean(body, meshBody("thread", std::move(*groove)), BooleanOp::Subtract, &why)) {
                doc.scene.undo();
                return out.fail("Cannot thread " + doc.scene.bodies()[static_cast<std::size_t>(bi)].name + ": " + why + " (nothing changed)");
            }
            if (j.note >= 0) body.holes[static_cast<std::size_t>(j.note)].threadModelled = true;
            ++done;
        }
        doc.scene.touchBodies();
        doc.dirty = true;
        out.say("Threaded " + body.name + ": " + std::to_string(done) + (done == 1 ? " thread" : " threads") + (left ? " (left hand)" : "") + ", " +
                closedText(body.mesh) + ", volume " + fmt3("%.2f", before / 1000.0) + " -> " + fmt3("%.2f", meshVolume(worldMesh(body)) / 1000.0) + " cm3");
        return out;
    }

    if (cmd == "SHEETMETAL") {
        // SHEETMETAL BASE curve THICKNESS t [RADIUS r] [K k]
        // SHEETMETAL FLANGE [body] EDGE n | NEAR x,y,z  LENGTH l [ANGLE deg] [DOWN] [RADIUS r]
        // SHEETMETAL EDGES [body] | SHEETMETAL FLAT [body] [SKETCH] | SHEETMETAL INFO [body]
        const char* usage = "Usage: SHEETMETAL BASE curve THICKNESS t [RADIUS r] [K k] | SHEETMETAL FLANGE [body] EDGE n|NEAR x,y,z LENGTH l [ANGLE deg] "
                            "[DOWN] [RADIUS r] [INSET a b] | SHEETMETAL HEM [body] EDGE n|NEAR x,y,z LENGTH l [OPEN gap|CLOSED] [DOWN] | "
                            "SHEETMETAL JOG [body] EDGE n|NEAR x,y,z OFFSET h LENGTH l [ANGLE deg] [DOWN] | SHEETMETAL MITER [body] EDGES n,m,... "
                            "LENGTH l [ANGLE deg] [GAP g] [DOWN] | SHEETMETAL RELIEF [body] [SIZE s] | SHEETMETAL CONVERT body FACE x,y,z [RADIUS r] [K k] | "
                            "SHEETMETAL EDGES|FLAT|INFO [body] [SKETCH]";
        if (a.empty()) return out.fail(usage);
        const std::string sub = lower(a[0]);
        static const char* const kKeys[] = {"thickness", "t", "radius", "r", "k", "edge", "edges", "near", "length", "angle", "down", "sketch",
                                            "inset", "open", "closed", "offset", "gap", "size", "face"};
        auto isKey = [](const std::string& w) {
            for (const char* k : kKeys)
                if (w == k) return true;
            return false;
        };
        std::size_t i = 1;
        const std::string name = leadingName(a, quoted, i, isKey);
        float thickness = 0, radius = -1, k = 0.44f, flangeLen = -1, angle = 90;
        bool down = false, sketch = false, closedHem = false;
        int edgeNo = -1;
        std::optional<Vec3> near, facePoint;
        float inset0 = 0, inset1 = 0, hemGap = -1, jogOffset = 0, miterGap = -1, relief = -1;
        std::vector<int> edgeList;
        for (; i < a.size(); ++i) {
            const std::string w = lower(a[i]);
            auto len = [&](float& v, bool positive) {
                const auto x = i + 1 < a.size() ? parseLength(a[++i]) : std::nullopt;
                if (!x || (positive ? !(*x > 0) : *x < 0)) return false;
                v = static_cast<float>(*x);
                return true;
            };
            if (w == "thickness" || w == "t") {
                if (!len(thickness, true)) return out.fail("THICKNESS needs a length");
            } else if (w == "radius" || w == "r") {
                if (!len(radius, false)) return out.fail("RADIUS needs the inside bend radius");
            } else if (w == "length") {
                if (!len(flangeLen, false)) return out.fail("LENGTH needs a length");
            } else if (w == "k") {
                const auto x = i + 1 < a.size() ? parseNumber(a[++i]) : std::nullopt;
                if (!x || *x < 0 || *x > 1) return out.fail("K is 0 to 1 (0.44 is usual for steel)");
                k = *x;
            } else if (w == "angle") {
                const auto x = i + 1 < a.size() ? parseNumber(a[++i]) : std::nullopt;
                if (!x || *x <= 0 || *x > 180) return out.fail("ANGLE is more than 0, up to 180 degrees");
                angle = *x;
            } else if (w == "edge") {
                const auto x = i + 1 < a.size() ? parseNumber(a[++i]) : std::nullopt;
                if (!x || *x < 1) return out.fail("EDGE takes an edge's number (SHEETMETAL EDGES lists them)");
                edgeNo = static_cast<int>(*x) - 1;
            } else if (w == "near") {
                if (!(near = i + 1 < a.size() ? parseVec3(a[++i]) : std::nullopt)) return out.fail("NEAR needs x,y,z");
            } else if (w == "edges") {
                const std::string list = i + 1 < a.size() ? a[++i] : std::string();
                std::size_t at = 0;
                while (at <= list.size()) {
                    const std::size_t comma = list.find(',', at);
                    const auto x = parseNumber(list.substr(at, comma == std::string::npos ? std::string::npos : comma - at));
                    if (!x || *x < 1) return out.fail("EDGES takes edge numbers: EDGES 1,2,3 (SHEETMETAL EDGES lists them)");
                    edgeList.push_back(static_cast<int>(*x) - 1);
                    if (comma == std::string::npos) break;
                    at = comma + 1;
                }
            } else if (w == "inset") {
                if (!len(inset0, false) || !len(inset1, false)) return out.fail("INSET needs two lengths: held back at the edge's start and end");
            } else if (w == "open") {
                if (!len(hemGap, true)) return out.fail("OPEN needs the hem's gap");
            } else if (w == "closed") closedHem = true;
            else if (w == "offset") {
                const auto x = i + 1 < a.size() ? parseLength(a[++i]) : std::nullopt;
                if (!x || *x == 0) return out.fail("OFFSET needs the jog's height");
                jogOffset = static_cast<float>(*x);
            } else if (w == "gap") {
                if (!len(miterGap, false)) return out.fail("GAP needs a length");
            } else if (w == "size") {
                if (!len(relief, true)) return out.fail("SIZE needs the relief's size");
            } else if (w == "face") {
                if (!(facePoint = i + 1 < a.size() ? parseVec3(a[++i]) : std::nullopt)) return out.fail("FACE needs a point x,y,z on the face");
            } else if (w == "down") down = true;
            else if (w == "sketch") sketch = true;
            else return out.fail(usage);
        }
        if (sub == "base") {
            const int ci = name.empty() ? doc.selectedCurve : curveNamed(doc, name);
            if (ci < 0 || ci >= static_cast<int>(doc.scene.sketch().curves().size())) return out.fail("SHEETMETAL BASE needs a closed sketch curve");
            if (!(thickness > 0)) return out.fail("SHEETMETAL BASE needs THICKNESS t");
            std::string why;
            const auto profile = profileFromSketch(doc.scene.sketch(), static_cast<std::size_t>(ci), &why);
            if (!profile) return out.fail("Cannot make a sheet from it: " + why);
            const SketchCurve curve = doc.scene.sketch().curves()[static_cast<std::size_t>(ci)];
            SheetPart part;
            part.thickness = thickness, part.radius = radius >= 0 ? radius : thickness, part.k = k;
            part.frame = curveFrame(curve);
            part.base = profilePolygon(*profile);
            const MeshData m = buildSheet(part, false, &why);
            if (m.indices.empty()) return out.fail("Cannot make the sheet: " + why);
            const int serial = ++doc.primitiveSerial;
            Body b = defaultPrimitive("box", serial);
            b.name = "Sheet " + std::to_string(serial);
            b.kind = "sheet";
            b.params = sheetParams(part);
            b.mesh = m;
            b.offset = {};
            b.brep.reset();
            hideCurve(doc, ci);
            doc.scene.pushUndo("SHEETMETAL BASE " + curve.name);
            out.say("Sheet metal base " + b.name + ": " + lengthText(thickness) + " thick, bend radius " + lengthText(part.radius) + ", K " +
                    fmt3("%g", part.k) + " (SHEETMETAL EDGES lists where flanges go)");
            addBody(doc, std::move(b));
            return out;
        }
        const int bi = name.empty() ? doc.selectedBody : bodyNamed(doc, name);
        if (bi < 0 || bi >= static_cast<int>(doc.scene.bodies().size())) return out.fail(name.empty() ? "Select a sheet metal body, or name it" : "No body named " + name);
        Body& body = doc.scene.bodies()[static_cast<std::size_t>(bi)];
        if (sub == "convert") {
            // A thin-walled solid (a plate, a shelled box) as sheet metal: the face picked is the
            // base's outside; walls standing on its edges become flanges.
            if (!facePoint) return out.fail("SHEETMETAL CONVERT needs FACE x,y,z: a point on the base's outside face");
            const MeshData world = worldMesh(body);
            if (openEdgeCount(world) != 0) return out.fail(body.name + " is not a closed solid");
            Vec3 q{}, n{0, 0, 1};
            if (!nearestFace(world, *facePoint, q, n)) return out.fail(body.name + " has no faces");
            auto hitAlong = [&](Vec3 from, Vec3 dir) { // the nearest crossing of the body's surface
                return upToNext(doc, {from}, dir, bi);
            };
            const AABB wb = world.bounds();
            const float tiny = 1e-4f * length(wb.max - wb.min);
            const auto tHit = hitAlong(q - n * tiny, n * -1.0f);
            if (!tHit) return out.fail("No material under that face");
            const float t = *tHit + tiny;
            const float r0 = radius >= 0 ? radius : t;
            // The footprint: the section half way through the plate.
            PlaneFrame f;
            makePlaneFrame(q, n * -1.0f, std::fabs(n.z) > 0.9f ? Vec3{1, 0, 0} : cross(Vec3{0, 0, 1}, n), f);
            PlaneFrame mid = f;
            mid.origin = q - n * (0.5f * t);
            std::vector<Vec2> outline;
            for (std::vector<Vec2>& l : planeSection(world, mid)) {
                if (signedArea(l) <= 0) continue;
                bool in = false; // around the point
                const Vec2 pq = mid.toPlane(*facePoint);
                for (std::size_t k2 = 0, j = l.size() - 1; k2 < l.size(); j = k2++)
                    if ((l[k2].y > pq.y) != (l[j].y > pq.y) && pq.x < (l[j].x - l[k2].x) * (pq.y - l[k2].y) / (l[j].y - l[k2].y) + l[k2].x) in = !in;
                if (in) outline = std::move(l);
            }
            for (bool changed = true; changed && outline.size() > 3;) { // straight runs as one edge
                changed = false;
                for (std::size_t k2 = 0; k2 < outline.size(); ++k2) {
                    const Vec2 p0 = outline[(k2 + outline.size() - 1) % outline.size()], p1 = outline[k2], p2 = outline[(k2 + 1) % outline.size()];
                    if (std::fabs((p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x)) <= 1e-4f * std::max(1.0f, std::hypot(p2.x - p0.x, p2.y - p0.y))) {
                        outline.erase(outline.begin() + static_cast<std::ptrdiff_t>(k2));
                        changed = true;
                        break;
                    }
                }
            }
            if (outline.size() < 3) return out.fail("No flat plate at that face");
            // Walls: material standing on an edge, inside its outline; their height from the outside face.
            const std::size_t nb = outline.size();
            std::vector<float> wall(nb, 0.0f);
            for (std::size_t e = 0; e < nb; ++e) {
                const Vec2 a2 = outline[e], b2 = outline[(e + 1) % nb];
                const float l = std::hypot(b2.x - a2.x, b2.y - a2.y);
                const Vec2 inward{-(b2.y - a2.y) / l, (b2.x - a2.x) / l}; // left of a counter-clockwise outline
                const Vec2 m2{(a2.x + b2.x) * 0.5f + inward.x * 0.5f * t, (a2.y + b2.y) * 0.5f + inward.y * 0.5f * t};
                const Vec3 start = f.toWorld(m2) + f.normal * (t + tiny);
                // Inside a wall: the way up leaves the solid further on, and the way in leaves it within a wall's thickness.
                const auto up = hitAlong(start, f.normal);
                const Vec3 inDir = normalize(f.toWorld(inward) - f.origin);
                const auto across = hitAlong(start, inDir);
                if (up && across && *across < 1.5f * t && *up > t) wall[e] = t + tiny + *up;
            }
            SheetPart part;
            part.thickness = t, part.radius = r0, part.k = k;
            part.frame = f;
            // The base: walled edges moved in by the bend (r + t), the others where they are.
            std::vector<Vec2> base(nb);
            for (std::size_t v = 0; v < nb; ++v) {
                const std::size_t ep = (v + nb - 1) % nb, en = v; // the edges before and after corner v
                auto lineOf = [&](std::size_t e, Vec2& p0, Vec2& d) {
                    const Vec2 a2 = outline[e], b2 = outline[(e + 1) % nb];
                    const float l = std::hypot(b2.x - a2.x, b2.y - a2.y);
                    d = {(b2.x - a2.x) / l, (b2.y - a2.y) / l};
                    const float in = wall[e] > 0 ? r0 + t : 0.0f;
                    p0 = {a2.x - d.y * in, a2.y + d.x * in};
                };
                Vec2 p1, d1, p2, d2;
                lineOf(ep, p1, d1), lineOf(en, p2, d2);
                const float den = d1.x * d2.y - d1.y * d2.x;
                if (std::fabs(den) < 1e-6f) return out.fail("The base's outline has a corner that is not a corner");
                const float tt = ((p2.x - p1.x) * d2.y - (p2.y - p1.y) * d2.x) / den;
                base[v] = {p1.x + d1.x * tt, p1.y + d1.y * tt};
            }
            part.base = base;
            const float ba = bendAllowance(90, r0, k, t);
            int walls = 0;
            for (std::size_t e = 0; e < nb; ++e) {
                if (!(wall[e] > 0)) continue;
                const float fl = wall[e] - (r0 + t);
                if (fl < 0) return out.fail("A wall is lower than the bend (" + lengthText(r0 + t) + "): a smaller RADIUS");
                SheetFlange sf{-1, static_cast<int>(e), fl, 90.0f, false, -1};
                for (int end = 0; end < 2; ++end) { // a wall beside it: the corner mitered shut (rather than ripped open)
                    const std::size_t other = end ? (e + 1) % nb : (e + nb - 1) % nb;
                    if (!(wall[other] > 0)) continue;
                    (end ? sf.inset1 : sf.inset0) = 0.05f * t;
                    (end ? sf.slope1 : sf.slope0) = ba > 0 ? -(r0 + 0.5f * t) / ba : 0.0f;
                }
                part.flanges.push_back(sf);
                ++walls;
            }
            std::string why;
            MeshData m = buildSheet(part, false, &why);
            if (m.indices.empty()) return out.fail("Cannot convert " + body.name + ": " + why);
            doc.scene.pushUndo("SHEETMETAL CONVERT " + body.name);
            const double before = meshVolume(world);
            body.kind = "sheet";
            body.params = sheetParams(part);
            body.mesh = std::move(m);
            body.offset = {}, body.rotation = {}, body.scale = 1.0f;
            body.brep.reset();
            doc.scene.touchBodies();
            doc.dirty = true;
            out.say("Converted " + body.name + " to sheet metal: " + lengthText(t) + " thick, bend radius " + lengthText(r0) + ", " + std::to_string(walls) +
                    (walls == 1 ? " wall" : " walls") + " as flanges; volume " + fmt3("%.3f", before / 1000.0) + " -> " +
                    fmt3("%.3f", meshVolume(body.mesh) / 1000.0) + " cm3 (rounded bends, mitered corners)");
            return out;
        }
        SheetPart part;
        if (body.kind != "sheet" || !sheetFromParams(body.params, part)) return out.fail(body.name + " isn't a sheet metal part (SHEETMETAL BASE makes one)");
        // Bodies move with an offset: the part's own frame is where it was made.
        auto shift = [&](Vec3 p) { return p + body.offset; };
        const std::vector<SheetEdge> edges = sheetFreeEdges(part);
        if (sub == "edges") {
            if (edges.empty()) out.say(body.name + " has no free edges left");
            for (std::size_t e = 0; e < edges.size(); ++e)
                out.say("  " + std::to_string(e + 1) + "  " + (edges[e].parent < 0 ? "base" : "flange " + std::to_string(edges[e].parent + 1)) + "  " +
                        lengthText(edges[e].length) + " long, middle at " + pointText(shift((edges[e].a3 + edges[e].b3) * 0.5f)));
            return out;
        }
        if (sub == "info") {
            std::vector<std::vector<Vec2>> outline;
            std::vector<std::pair<Vec2, Vec2>> bends;
            sheetFlatOutline(part, outline, bends);
            Vec2 lo{1e30f, 1e30f}, hi{-1e30f, -1e30f};
            for (const auto& l : outline)
                for (const Vec2& q : l) lo = {std::min(lo.x, q.x), std::min(lo.y, q.y)}, hi = {std::max(hi.x, q.x), std::max(hi.y, q.y)};
            out.say(body.name + ": sheet " + lengthText(part.thickness) + ", bend radius " + lengthText(part.radius) + ", K " + fmt3("%g", part.k) + ", " +
                    std::to_string(part.flanges.size()) + (part.flanges.size() == 1 ? " flange" : " flanges"));
            for (std::size_t f = 0; f < part.flanges.size(); ++f) {
                const SheetFlange& fl = part.flanges[f];
                const float r = fl.radius >= 0 ? fl.radius : part.radius;
                out.say("  flange " + std::to_string(f + 1) + ": " + lengthText(fl.length) + ", " + fmt3("%g", fl.angle) + " deg " + (fl.down ? "down" : "up") +
                        ", radius " + lengthText(r) + ", bend allowance " + lengthText(bendAllowance(fl.angle, r, part.k, part.thickness)));
            }
            if (!outline.empty()) out.say("  flat pattern " + lengthText(hi.x - lo.x) + " x " + lengthText(hi.y - lo.y));
            return out;
        }
        // The part changed: rebuilt, or refused with why (nothing changed).
        auto commit = [&](const SheetPart& next, const std::string& what, const std::string& said) {
            std::string why;
            MeshData m = buildSheet(next, false, &why);
            if (m.indices.empty()) return out.fail("Cannot add the " + what + ": " + why), false;
            doc.scene.pushUndo("SHEETMETAL " + toUpper(what) + " " + body.name);
            body.params = sheetParams(next);
            body.mesh = std::move(m);
            body.brep.reset();
            doc.scene.touchBodies();
            doc.dirty = true;
            out.say(said);
            return true;
        };
        auto pickEdge = [&]() -> int {
            int pick = edgeNo;
            if (near) {
                float best = 1e30f;
                for (std::size_t e = 0; e < edges.size(); ++e) {
                    const float dd = length(shift((edges[e].a3 + edges[e].b3) * 0.5f) - *near);
                    if (dd < best) best = dd, pick = static_cast<int>(e);
                }
            }
            return pick >= 0 && pick < static_cast<int>(edges.size()) ? pick : -1;
        };
        const float rr = radius >= 0 ? radius : part.radius;
        if (sub == "hem") {
            // A flange folded flat back on itself: OPEN leaves a gap, CLOSED (the default) none.
            if (!(flangeLen > 0)) return out.fail("SHEETMETAL HEM needs LENGTH l");
            const int pick = pickEdge();
            if (pick < 0) return out.fail("Give EDGE n (SHEETMETAL EDGES lists them) or NEAR x,y,z");
            const SheetEdge& e = edges[static_cast<std::size_t>(pick)];
            SheetPart next = part;
            const float hr = hemGap > 0 ? hemGap * 0.5f : 0.02f * part.thickness; // the inside radius: half the gap
            (void)closedHem;
            next.flanges.push_back({e.parent, e.edge, flangeLen, 180.0f, down, hr});
            commit(next, "hem", std::string(hemGap > 0 ? "Open" : "Closed") + " hem on " + body.name + ": " + lengthText(flangeLen) + " folded back" +
                                    (hemGap > 0 ? ", gap " + lengthText(hemGap) : std::string()) + " (edge " + std::to_string(pick + 1) + ")");
            return out;
        }
        if (sub == "jog") {
            // Two bends the opposite way: the next flat steps OFFSET away and runs on LENGTH.
            if (jogOffset == 0 || !(flangeLen > 0)) return out.fail("SHEETMETAL JOG needs OFFSET h and LENGTH l");
            const int pick = pickEdge();
            if (pick < 0) return out.fail("Give EDGE n (SHEETMETAL EDGES lists them) or NEAR x,y,z");
            const SheetEdge& e = edges[static_cast<std::size_t>(pick)];
            const bool dn = down != (jogOffset < 0);
            const float first = jogFirstLength(part, e.parent, e.edge, std::fabs(jogOffset) * (dn ? -1.0f : 1.0f), angle, dn);
            if (first < 0) return out.fail("An offset of " + lengthText(std::fabs(jogOffset)) + " is less than the two bends rise: a larger OFFSET or a smaller RADIUS");
            SheetPart next = part;
            next.flanges.push_back({e.parent, e.edge, first, angle, dn, radius});
            next.flanges.push_back({static_cast<int>(next.flanges.size()) - 1, 0, flangeLen, angle, !dn, radius});
            commit(next, "jog", "Jog on " + body.name + ": " + lengthText(std::fabs(jogOffset)) + " offset, " + fmt3("%g", angle) + " deg bends, " +
                                    lengthText(first) + " between them, " + lengthText(flangeLen) + " beyond (edge " + std::to_string(pick + 1) + ")");
            return out;
        }
        if (sub == "miter") {
            // Flanges along base edges side by side, their shared corners cut on the bisector so
            // they meet (GAP apart): across the bend zones, and along a wall that leans out.
            if (!(flangeLen > 0) || edgeList.empty()) return out.fail("SHEETMETAL MITER needs EDGES n,m,... and LENGTH l");
            const float g = miterGap >= 0 ? miterGap : 0.1f * part.thickness;
            const std::size_t nb = part.base.size();
            std::vector<int> baseEdges;
            for (int ek : edgeList) {
                if (ek >= static_cast<int>(edges.size()) || edges[static_cast<std::size_t>(ek)].parent >= 0)
                    return out.fail("MITER takes free edges of the base (SHEETMETAL EDGES lists them)");
                baseEdges.push_back(edges[static_cast<std::size_t>(ek)].edge);
            }
            auto chosen = [&](int be) { return std::find(baseEdges.begin(), baseEdges.end(), be) != baseEdges.end(); };
            const float theta = angle * kPi / 180.0f;
            const float ba = bendAllowance(angle, rr, part.k, part.thickness);
            SheetPart next = part;
            int corners = 0;
            for (int be : baseEdges) {
                SheetFlange fl{-1, be, flangeLen, angle, down, radius};
                for (int end = 0; end < 2; ++end) {
                    // The base corner at this end, and its neighbour edge there.
                    const auto v = static_cast<std::size_t>(end ? (be + 1) % static_cast<int>(nb) : be);
                    const int other = end ? (be + 1) % static_cast<int>(nb) : (be + static_cast<int>(nb) - 1) % static_cast<int>(nb);
                    if (!chosen(other)) continue;
                    const Vec2 pv = part.base[(v + nb - 1) % nb], cv = part.base[v], nv = part.base[(v + 1) % nb];
                    const float phi = std::acos(std::clamp((( pv.x - cv.x) * (nv.x - cv.x) + (pv.y - cv.y) * (nv.y - cv.y)) /
                                                               (std::hypot(pv.x - cv.x, pv.y - cv.y) * std::hypot(nv.x - cv.x, nv.y - cv.y)), -1.0f, 1.0f));
                    const float cotHalf = 1.0f / std::tan(phi * 0.5f);
                    // Reaching past the corner by the bend's outward reach (mid-sheet) at the end
                    // of the zone, and by the wall's lean beyond it.
                    const float reachOut = (rr + 0.5f * part.thickness) * std::sin(theta) * cotHalf;
                    (end ? fl.inset1 : fl.inset0) = 0.5f * g;
                    (end ? fl.slope1 : fl.slope0) = ba > 0 ? -reachOut / ba : 0.0f;
                    (end ? fl.flare1 : fl.flare0) = -std::cos(theta) * cotHalf;
                    ++corners;
                }
                next.flanges.push_back(fl);
            }
            commit(next, "miter", "Miter flange on " + body.name + ": " + std::to_string(baseEdges.size()) + " edges, " + lengthText(flangeLen) + " at " +
                                      fmt3("%g", angle) + " deg, " + std::to_string(corners / 2) + " corners mitered (gap " + lengthText(g) + ")");
            return out;
        }
        if (sub == "relief") {
            // Where two flanges on the base meet at a corner, each held back SIZE from it (a
            // rectangular relief), so their bends don't run into each other.
            const float sz = relief > 0 ? relief : rr + part.thickness;
            const std::size_t nb = part.base.size();
            SheetPart next = part;
            int made = 0;
            for (SheetFlange& fl : next.flanges) {
                if (fl.parent >= 0) continue;
                for (int end = 0; end < 2; ++end) {
                    const int other = end ? (fl.edge + 1) % static_cast<int>(nb) : (fl.edge + static_cast<int>(nb) - 1) % static_cast<int>(nb);
                    const bool shared = std::any_of(next.flanges.begin(), next.flanges.end(), [&](const SheetFlange& o) { return o.parent < 0 && o.edge == other; });
                    float& in = end ? fl.inset1 : fl.inset0;
                    if (!shared || (end ? fl.slope1 : fl.slope0) != 0 || in >= sz) continue;
                    in = sz;
                    ++made;
                }
            }
            if (!made) return out.fail(body.name + " has no corner where two flanges meet without a relief");
            commit(next, "relief", "Corner reliefs on " + body.name + ": " + std::to_string(made) + " flange ends held back " + lengthText(sz));
            return out;
        }
        if (sub == "flange") {
            if (!(flangeLen >= 0)) return out.fail("SHEETMETAL FLANGE needs LENGTH l");
            int pick = edgeNo;
            if (near) {
                float best = 1e30f;
                for (std::size_t e = 0; e < edges.size(); ++e) {
                    const float dd = length(shift((edges[e].a3 + edges[e].b3) * 0.5f) - *near);
                    if (dd < best) best = dd, pick = static_cast<int>(e);
                }
            }
            if (pick < 0 || pick >= static_cast<int>(edges.size()))
                return out.fail(edges.empty() ? body.name + " has no free edges left" : "Give EDGE n (1 to " + std::to_string(edges.size()) + ", SHEETMETAL EDGES) or NEAR x,y,z");
            const SheetEdge& e = edges[static_cast<std::size_t>(pick)];
            SheetPart next = part;
            SheetFlange fl{e.parent, e.edge, flangeLen, angle, down, radius};
            fl.inset0 = inset0, fl.inset1 = inset1;
            next.flanges.push_back(fl);
            std::string why;
            MeshData m = buildSheet(next, false, &why);
            if (m.indices.empty()) return out.fail("Cannot add the flange: " + why);
            doc.scene.pushUndo("SHEETMETAL FLANGE " + body.name);
            body.params = sheetParams(next);
            body.mesh = std::move(m);
            body.brep.reset();
            doc.scene.touchBodies();
            doc.dirty = true;
            out.say("Flange " + std::to_string(next.flanges.size()) + " on " + body.name + ": " + lengthText(flangeLen) + " at " + fmt3("%g", angle) + " deg " +
                    (down ? "down" : "up") + " (edge " + std::to_string(pick + 1) + ", " + lengthText(e.length) + ")");
            return out;
        }
        if (sub == "flat") {
            std::string why;
            MeshData m = buildSheet(part, true, &why);
            if (m.indices.empty()) return out.fail("Cannot unfold " + body.name + ": " + why);
            // Beside the folded part, along the base plane's u axis.
            const AABB fb = worldMesh(body).bounds();
            const float gap = length(fb.max - fb.min) + 10.0f;
            m.transform(translate(body.offset + part.frame.u * gap));
            const int serial = ++doc.primitiveSerial;
            (void)serial;
            Body flatBody = featureBody(body.name + " flat", std::move(m), body.color);
            if (sketch) { // the outline and bend lines as sketch curves on the base's plane (moved alongside)
                std::vector<std::vector<Vec2>> outline;
                std::vector<std::pair<Vec2, Vec2>> bends;
                sheetFlatOutline(part, outline, bends);
                const Vec2 off = part.frame.toPlane(part.frame.origin + part.frame.u * gap + body.offset);
                SketchCurve proto;
                proto.plane = SketchPlane::Custom;
                proto.frame = part.frame;
                if (sameFrame(part.frame, planeFrame(SketchPlane::Top))) proto.plane = SketchPlane::Top;
                else if (sameFrame(part.frame, planeFrame(SketchPlane::Front))) proto.plane = SketchPlane::Front;
                else if (sameFrame(part.frame, planeFrame(SketchPlane::Right))) proto.plane = SketchPlane::Right;
                for (const auto& l : outline) {
                    SketchCurve c = proto;
                    c.kind = CurveKind::Polyline;
                    for (const Vec2& q : l) c.pts.push_back({q.x + off.x, q.y + off.y});
                    c.pts.push_back(c.pts.front());
                    doc.scene.sketch().add(std::move(c));
                }
                for (const auto& [p, q] : bends) {
                    SketchCurve c = proto;
                    c.kind = CurveKind::Line;
                    c.construction = true;
                    c.pts = {{p.x + off.x, p.y + off.y}, {q.x + off.x, q.y + off.y}};
                    doc.scene.sketch().add(std::move(c));
                }
            }
            doc.scene.pushUndo("SHEETMETAL FLAT " + body.name);
            out.say("Flat pattern of " + body.name + " -> " + flatBody.name + (sketch ? " (and its outline and bend lines as sketch curves)" : ""));
            addBody(doc, std::move(flatBody));
            return out;
        }
        return out.fail(usage);
    }

    if (cmd == "FEA") {
        // FEA [body] FIX FACE x,y,z | FIX NEAR x,y,z r ...  LOAD FACE x,y,z | NEAR x,y,z r FORCE fx,fy,fz ...
        //     [MATERIAL name] [E MPa] [NU v] [YIELD MPa] [CELL size] [PLOT] [DEFORM scale]
        //     FEA MATERIALS
        static const char* const kKeys[] = {"fix", "load", "force", "material", "e", "nu", "yield", "cell", "plot", "deform", "face", "near",
                                            "pressure", "gravity", "bearing", "modes", "mode", "temp", "heat", "thermal", "ref", "report"};
        auto isKey = [](const std::string& w) {
            for (const char* k : kKeys)
                if (w == k) return true;
            return w.find(',') != std::string::npos;
        };
        const char* usage = "Usage: FEA [body] FIX FACE x,y,z | FIX NEAR x,y,z r ... [LOAD FACE x,y,z | NEAR x,y,z r FORCE fx,fy,fz] "
                            "[PRESSURE FACE x,y,z MPA] [BEARING x,y,z r FORCE fx,fy,fz] [GRAVITY [x,y,z]] [TEMP FACE x,y,z C] [HEAT FACE x,y,z W] "
                            "[THERMAL] [REF C] [MODES n] [MODE k] [MATERIAL name] [E MPa] [NU v] [YIELD MPa] [CELL size] [PLOT] [DEFORM scale] [REPORT file.html]  |  FEA MATERIALS";
        if (a.size() == 1 && lower(a[0]) == "materials") {
            for (const FeaMaterial& m : feaMaterials())
                out.say("  " + m.name + fmt3(":  E %g MPa", m.E) + fmt3(", nu %g", m.nu) + fmt3(", yield %g MPa", m.yield) + fmt3(", %g g/cm3", m.density));
            return out;
        }
        std::size_t i = 0;
        const std::string name = leadingName(a, quoted, i, isKey);
        const int bi = name.empty() ? doc.selectedBody : bodyNamed(doc, name);
        if (bi < 0 || bi >= static_cast<int>(doc.scene.bodies().size())) return out.fail(name.empty() ? "Select a body to analyse, or name it" : "No body named " + name);
        FeaSetup setup;
        setup.material = *feaMaterial("steel");
        bool plot = false;
        float deform = 0;
        int showMode = 0;
        std::string reportPath;
        auto selection = [&](FeaSelection& sel) -> bool { // FACE x,y,z | NEAR x,y,z r
            if (i + 1 >= a.size()) return false;
            const std::string k = lower(a[++i]);
            const auto p = i + 1 < a.size() ? parseVec3(a[i + 1]) : std::nullopt;
            if (!p) return false;
            ++i;
            sel.point = *p;
            if (k == "face") sel.kind = FeaSelection::Kind::Face;
            else if (k == "near") {
                const auto r = i + 1 < a.size() ? parseLength(a[i + 1]) : std::nullopt;
                if (!r || !(*r > 0)) return false;
                ++i;
                sel.kind = FeaSelection::Kind::Near, sel.radius = static_cast<float>(*r);
            } else return false;
            return true;
        };
        for (; i < a.size(); ++i) {
            const std::string k = lower(a[i]);
            auto number = [&](double& v) {
                const auto x = i + 1 < a.size() ? parseNumber(a[++i]) : std::nullopt;
                if (!x || !(*x > 0)) return false;
                v = *x;
                return true;
            };
            if (k == "fix") {
                FeaSelection sel;
                if (!selection(sel)) return out.fail("FIX takes FACE x,y,z or NEAR x,y,z r");
                setup.fixed.push_back(sel);
            } else if (k == "load") {
                FeaLoad l;
                if (!selection(l.where)) return out.fail("LOAD takes FACE x,y,z or NEAR x,y,z r, then FORCE fx,fy,fz");
                if (i + 2 >= a.size() || lower(a[i + 1]) != "force") return out.fail("LOAD ... FORCE fx,fy,fz (N)");
                i += 2;
                const auto fv = parseVec3(a[i]);
                if (!fv || !(length(*fv) > 0)) return out.fail("FORCE needs fx,fy,fz in newtons");
                l.force = *fv;
                setup.loads.push_back(l);
            } else if (k == "pressure") {
                FeaPressure pr;
                if (!selection(pr.where)) return out.fail("PRESSURE takes FACE x,y,z (or NEAR x,y,z r), then the pressure in MPa");
                const auto x = i + 1 < a.size() ? parseNumber(a[++i]) : std::nullopt;
                if (!x || *x == 0) return out.fail("PRESSURE ... needs the pressure in MPa (N/mm2; negative pulls)");
                pr.mpa = *x;
                setup.pressures.push_back(pr);
            } else if (k == "bearing") {
                FeaBearing br;
                const auto c = i + 1 < a.size() ? parseVec3(a[i + 1]) : std::nullopt;
                const auto r = i + 2 < a.size() ? parseLength(a[i + 2]) : std::nullopt;
                if (!c || !r || !(*r > 0) || i + 4 >= a.size() || lower(a[i + 3]) != "force") return out.fail("BEARING x,y,z r FORCE fx,fy,fz: the hole's centre, its radius, the pin's push");
                const auto fv = parseVec3(a[i + 4]);
                if (!fv || !(length(*fv) > 0)) return out.fail("FORCE needs fx,fy,fz in newtons");
                br.center = *c, br.radius = static_cast<float>(*r), br.force = *fv;
                setup.bearings.push_back(br);
                i += 4;
            } else if (k == "gravity") {
                setup.gravity = {0, 0, -1};
                if (i + 1 < a.size())
                    if (const auto gv = parseVec3(a[i + 1])) setup.gravity = *gv, ++i;
            } else if (k == "temp" || k == "heat") {
                FeaSelection sel;
                if (!selection(sel)) return out.fail(toUpper(k) + " takes FACE x,y,z (or NEAR x,y,z r), then " + (k == "temp" ? "degrees C" : "watts"));
                const auto x = i + 1 < a.size() ? parseNumber(a[++i]) : std::nullopt;
                if (!x) return out.fail(toUpper(k) + " ... needs " + (k == "temp" ? "the temperature in C" : "the heat in watts"));
                if (k == "temp") setup.temperatures.push_back({sel, *x});
                else setup.heats.push_back({sel, *x});
            } else if (k == "thermal") setup.thermalStress = true;
            else if (k == "ref") {
                const auto x = i + 1 < a.size() ? parseNumber(a[++i]) : std::nullopt;
                if (!x) return out.fail("REF needs the stress-free temperature in C");
                setup.reference = *x;
            } else if (k == "modes" || k == "mode") {
                const auto x = i + 1 < a.size() ? parseNumber(a[++i]) : std::nullopt;
                if (!x || *x < 1 || *x > 12) return out.fail(toUpper(k) + " is 1 to 12");
                if (k == "modes") setup.modes = static_cast<int>(*x);
                else showMode = static_cast<int>(*x), plot = true;
            } else if (k == "material") {
                const auto m = i + 1 < a.size() ? feaMaterial(a[++i]) : std::nullopt;
                if (!m) return out.fail("No such material (FEA MATERIALS lists them)");
                setup.material = *m;
            } else if (k == "e") {
                if (!number(setup.material.E)) return out.fail("E needs Young's modulus in MPa");
                setup.material.name = "custom";
            } else if (k == "nu") {
                if (!number(setup.material.nu) || setup.material.nu >= 0.5) return out.fail("NU is Poisson's ratio, below 0.5");
            } else if (k == "yield") {
                if (!number(setup.material.yield)) return out.fail("YIELD needs the yield strength in MPa");
            } else if (k == "cell") {
                const auto x = i + 1 < a.size() ? parseLength(a[++i]) : std::nullopt;
                if (!x || !(*x > 0)) return out.fail("CELL needs the cubes' size");
                setup.cell = static_cast<float>(*x);
            } else if (k == "plot") plot = true;
            else if (k == "report") {
                if (i + 1 >= a.size()) return out.fail("REPORT needs a file name (.html)");
                reportPath = a[++i];
            } else if (k == "deform") {
                const auto x = i + 1 < a.size() ? parseNumber(a[++i]) : std::nullopt;
                if (!x || *x < 0) return out.fail("DEFORM needs a scale (1: true size)");
                deform = *x, plot = true;
            } else return out.fail(usage);
        }
        const Body& body = doc.scene.bodies()[static_cast<std::size_t>(bi)];
        MeshData world = worldMesh(body);
        if (openEdgeCount(world) != 0) return out.fail(body.name + " isn't closed: REPAIR it first");
        if (plot || !reportPath.empty()) { // a plot needs vertices to show the stress between corners: finer surface triangles
            const AABB wb = world.bounds();
            const float detail = std::max({wb.max.x - wb.min.x, wb.max.y - wb.min.y, wb.max.z - wb.min.z}) / 80.0f;
            Body fine = meshBody("fine", world);
            sculptRefine(fine, setup.cell > 0 ? std::min(detail, setup.cell) : detail);
            world = worldMesh(fine);
        }
        if (showMode > 0 && setup.modes < showMode) setup.modes = showMode;
        FeaResult r;
        std::string why;
        if (!runFea(world, setup, r, &why)) return out.fail("Cannot analyse " + body.name + ": " + why);
        const bool statics = !setup.loads.empty() || !setup.pressures.empty() || !setup.bearings.empty() || length(setup.gravity) > 0 ||
                             (setup.thermalStress && !setup.temperatures.empty());
        out.say("Analysis of " + body.name + " (" + setup.material.name + ", E " + fmt3("%g", setup.material.E) + " MPa, yield " +
                fmt3("%g", setup.material.yield) + " MPa): " + std::to_string(r.cells) + " cubes of " + lengthText(r.cell) + ", " +
                fmt3("%.4g", r.mass) + " kg" + (r.fixedNodes ? ", " + std::to_string(r.fixedNodes) + " nodes fixed" : std::string()) +
                (r.loadedNodes ? ", " + std::to_string(r.loadedNodes) + " loaded" : std::string()));
        if (!setup.temperatures.empty())
            out.say("  temperatures " + fmt3("%.4g", r.minTemperature) + " C (at " + pointText(r.minTemperatureAt) + ") to " + fmt3("%.4g", r.maxTemperature) +
                    " C (at " + pointText(r.maxTemperatureAt) + ")");
        for (std::size_t m = 0; m < r.frequencies.size(); ++m)
            out.say("  mode " + std::to_string(m + 1) + ": " + fmt3("%.5g", r.frequencies[m]) + " Hz");
        if (statics) {
            out.say("  max von Mises stress " + fmt3("%.4g", r.maxStress) + " MPa at " + pointText(r.maxStressAt));
            out.say("  max displacement " + fmt3("%.4g", r.maxDisplacement) + " mm at " + pointText(r.maxDisplacementAt));
            out.say("  factor of safety " + fmt3("%.3g", r.safetyFactor) + (r.safetyFactor < 1 ? "  (yields!)" : r.safetyFactor < 1.5 ? "  (thin margin)" : ""));
        }
        if (!r.converged) out.say("  (the solver stopped after " + std::to_string(r.iterations) + " steps before converging: results are rough)");
        if (!reportPath.empty() && !doc.replaying) { // regenerating the model doesn't write files
            FeaReportData rd;
            rd.title = doc.name, rd.body = body.name, rd.setup = setup, rd.result = r, rd.mesh = world;
            std::string err;
            if (!writeFile(reportPath, feaReportHtml(rd), &err)) return out.fail("Cannot write the report: " + err);
            out.say("  report: " + reportPath);
        }
        if (plot && (showMode > 0 || !statics)) { // a mode's shape, or the temperatures
            MeshData m = world;
            std::string field;
            if (showMode > 0 && static_cast<std::size_t>(showMode) <= r.modeShapes.size()) {
                const std::vector<Vec3>& shape = r.modeShapes[static_cast<std::size_t>(showMode - 1)];
                const AABB wb = world.bounds();
                const float amp = (deform > 0 ? deform : 0.08f) * std::max({wb.max.x - wb.min.x, wb.max.y - wb.min.y, wb.max.z - wb.min.z});
                for (std::size_t v = 0; v < m.vertices.size(); ++v) {
                    m.vertices[v].mask = std::clamp(length(shape[v]), 0.0f, 1.0f);
                    m.vertices[v].position = m.vertices[v].position + shape[v] * amp;
                }
                field = "mode " + std::to_string(showMode) + " at " + fmt3("%.5g", r.frequencies[static_cast<std::size_t>(showMode - 1)]) + " Hz: motion 0 (blue) to most (red)";
            } else {
                const double lo = r.minTemperature, span = std::max(1e-9, r.maxTemperature - r.minTemperature);
                for (std::size_t v = 0; v < m.vertices.size(); ++v) m.vertices[v].mask = static_cast<float>(std::clamp((r.surfaceTemperature[v] - lo) / span, 0.0, 1.0));
                field = "temperature, C: " + fmt3("%.4g", r.minTemperature) + " (blue) to " + fmt3("%.4g", r.maxTemperature) + " (red)";
            }
            int k = 1;
            const std::string base = body.name + (showMode > 0 ? " mode" : " heat");
            while (bodyNamed(doc, base + (k > 1 ? " " + std::to_string(k) : "")) >= 0) ++k;
            Body res = meshBody(base + (k > 1 ? " " + std::to_string(k) : ""), std::move(m), body.color);
            res.field = field;
            doc.scene.pushUndo("FEA " + body.name);
            doc.scene.bodies()[static_cast<std::size_t>(bi)].visible = false;
            out.say("  plot: " + res.name + " (" + res.field + ")");
            addBody(doc, std::move(res));
        } else if (plot) {
            MeshData m = world;
            const double top = r.maxStress > 0 ? r.maxStress : 1.0;
            for (std::size_t v = 0; v < m.vertices.size(); ++v) {
                m.vertices[v].mask = static_cast<float>(std::clamp(static_cast<double>(r.surfaceStress[v]) / top, 0.0, 1.0));
                if (deform > 0) m.vertices[v].position = m.vertices[v].position + r.surfaceDisplacement[v] * deform;
            }
            int k = 1;
            while (bodyNamed(doc, body.name + " stress" + (k > 1 ? " " + std::to_string(k) : "")) >= 0) ++k;
            Body res = meshBody(body.name + " stress" + (k > 1 ? " " + std::to_string(k) : ""), std::move(m), body.color);
            res.field = "von Mises stress, MPa: 0 (blue) to " + fmt3("%.4g", r.maxStress) + " (red)";
            doc.scene.bodies()[static_cast<std::size_t>(bi)].visible = false; // the plot in its place
            doc.scene.pushUndo("FEA " + body.name);
            out.say("  plot: " + res.name + " (" + res.field + (deform > 0 ? ", shape x" + fmt3("%g", deform) : std::string()) + ")");
            addBody(doc, std::move(res));
        }
        return out;
    }

    if (cmd == "SURFACE" || cmd == "THICKEN" || cmd == "KNIT" || cmd == "ANALYZE") {
        // Names in a list: "quoted", or a word and its number (Line 3), or one word.
        auto nameList = [&](std::size_t from, std::size_t to) {
            std::vector<std::string> names;
            for (std::size_t k = from; k < to; ++k) {
                if (quotedAt(k)) names.push_back(a[k]);
                else if (k + 1 < to && !quotedAt(k + 1) && !a[k + 1].empty() && std::all_of(a[k + 1].begin(), a[k + 1].end(), ::isdigit))
                    names.push_back(a[k] + " " + a[k + 1]), ++k;
                else names.push_back(a[k]);
            }
            return names;
        };
        auto uniqueName = [&](const std::string& base) {
            for (int k = 1;; ++k) {
                const std::string n = base + " " + std::to_string(k);
                if (bodyNamed(doc, n) < 0) return n;
            }
        };
        // A curve as world points (closed ones repeat the first).
        auto curvePoints = [&](const std::string& nm, std::vector<DVec3>& pts, bool& closed, std::string& why) {
            const int ci = curveNamed(doc, nm);
            if (ci < 0) return why = "No sketch curve named " + nm, false;
            const SketchCurve& c = doc.scene.sketch().curves()[static_cast<std::size_t>(ci)];
            pts.clear();
            for (const Vec3& p : c.worldStrip(128)) pts.push_back(DVec3(p));
            closed = c.isClosed();
            if (pts.size() < 2) return why = nm + " has no length", false;
            return true;
        };
        auto addSurface = [&](const BSplineSurface& s, const std::string& what, std::vector<int> hideCurves) {
            Body b;
            b.name = uniqueName("Surface");
            b.color = {0.55f, 0.75f, 0.85f};
            std::string why;
            if (!makeSurfaceBody(b, surfaceBrep(s), &why)) return out.fail("The surface could not be made: " + why), false;
            doc.scene.pushUndo("SURFACE " + what);
            for (int ci : hideCurves) hideCurve(doc, ci);
            const std::string nm = b.name;
            const std::size_t tris = b.mesh.indices.size() / 3;
            addBody(doc, std::move(b));
            out.say("Made " + nm + " (" + what + "): a B-spline surface, " + std::to_string(s.nu) + " x " + std::to_string(s.nv) + " control points, degree " +
                    std::to_string(s.du) + " x " + std::to_string(s.dv) + ", " + std::to_string(tris) + " display triangles");
            return true;
        };
        constexpr int kSamples = 33;
        const std::string sub = cmd == "SURFACE" && !a.empty() ? lower(a[0]) : "";

        if (cmd == "SURFACE" && sub == "loft") {
            // SURFACE LOFT curve curve ... : through sections in order (closed or open alike)
            const std::vector<std::string> names = nameList(1, a.size());
            if (names.size() < 2) return out.fail("Usage: SURFACE LOFT curve curve [curve ...]");
            std::vector<std::vector<DVec3>> rows;
            std::vector<int> used;
            bool firstClosed = false;
            for (std::size_t k = 0; k < names.size(); ++k) {
                std::vector<DVec3> pts;
                bool closed = false;
                std::string why;
                if (!curvePoints(names[k], pts, closed, why)) return out.fail(why);
                if (k == 0) firstClosed = closed;
                else if (closed != firstClosed) return out.fail("The sections must be all closed or all open");
                std::vector<DVec3> r = resample(pts, kSamples, closed);
                if (!rows.empty()) { // lined up with the section before: same start, same way round
                    const std::vector<DVec3>& prev = rows.back();
                    if (closed) {
                        r.pop_back();
                        std::size_t best = 0;
                        for (std::size_t j = 0; j < r.size(); ++j)
                            if (dlength(r[j] - prev[0]) < dlength(r[best] - prev[0])) best = j;
                        std::rotate(r.begin(), r.begin() + static_cast<std::ptrdiff_t>(best), r.end());
                        if (dlength(r[1] - prev[1]) > dlength(r.back() - prev[1])) std::reverse(r.begin() + 1, r.end());
                        r.push_back(r.front());
                    } else if (dlength(r.front() - prev.front()) + dlength(r.back() - prev.back()) >
                               dlength(r.back() - prev.front()) + dlength(r.front() - prev.back()))
                        std::reverse(r.begin(), r.end());
                }
                rows.push_back(std::move(r));
                used.push_back(curveNamed(doc, names[k]));
            }
            BSplineSurface s;
            std::string why;
            if (!interpolateSurface(rows, s, &why)) return out.fail("Cannot loft: " + why);
            if (!addSurface(s, "loft through " + std::to_string(names.size()) + " sections", used)) return out;
            return out;
        }
        if (cmd == "SURFACE" && sub == "boundary") {
            // SURFACE BOUNDARY c1 c2 c3 c4: the four sides of a patch, meeting end to end
            const std::vector<std::string> names = nameList(1, a.size());
            if (names.size() != 4) return out.fail("Usage: SURFACE BOUNDARY curve curve curve curve (meeting end to end)");
            std::vector<std::vector<DVec3>> sides;
            std::vector<int> used;
            for (const std::string& nm : names) {
                std::vector<DVec3> pts;
                bool closed = false;
                std::string why;
                if (!curvePoints(nm, pts, closed, why)) return out.fail(why);
                if (closed) return out.fail(nm + " is closed: a boundary side is an open curve");
                sides.push_back(pts);
                used.push_back(curveNamed(doc, nm));
            }
            std::vector<std::vector<DVec3>> grid;
            std::string why;
            if (!coonsGrid(sides, 21, grid, &why)) return out.fail("Cannot fill: " + why);
            BSplineSurface s;
            if (!interpolateSurface(grid, s, &why)) return out.fail("Cannot fill: " + why);
            addSurface(s, "boundary of four curves", used);
            return out;
        }
        if (cmd == "SURFACE" && (sub == "extrude" || sub == "revolve")) {
            // SURFACE EXTRUDE curve DISTANCE d [DIR x,y,z] | SURFACE REVOLVE curve AXIS x,y,z [AT x,y,z] [ANGLE deg]
            auto isKey = [](const std::string& w) { return w == "distance" || w == "dir" || w == "axis" || w == "at" || w == "angle"; };
            std::size_t i = 1;
            const std::string nm = leadingName(a, quoted, i, isKey);
            std::vector<DVec3> pts;
            bool closed = false;
            std::string why;
            if (!curvePoints(nm, pts, closed, why)) return out.fail(why);
            float dist = 0, angle = 360;
            std::optional<Vec3> dir, axis;
            Vec3 at{};
            for (; i < a.size(); ++i) {
                const std::string k = lower(a[i]);
                if (k == "distance" && i + 1 < a.size()) {
                    const auto v = parseLength(a[++i]);
                    if (!v) return out.fail("DISTANCE needs a length");
                    dist = static_cast<float>(*v);
                } else if ((k == "dir" || k == "axis") && i + 1 < a.size()) {
                    const auto v = axisWord(a[++i]);
                    if (!v) return out.fail(toUpper(k) + " needs X, Y, Z or x,y,z");
                    (k == "dir" ? dir : axis) = normalize(*v);
                } else if (k == "at" && i + 1 < a.size()) {
                    const auto v = parseVec3(a[++i]);
                    if (!v) return out.fail("AT needs x,y,z");
                    at = *v;
                } else if (k == "angle" && i + 1 < a.size()) {
                    const auto v = parseNumber(a[++i]);
                    if (!v || std::fabs(*v) < 0.1f || std::fabs(*v) > 360) return out.fail("ANGLE is up to 360 degrees");
                    angle = *v;
                } else return out.fail("Usage: SURFACE EXTRUDE curve DISTANCE d [DIR x,y,z] | SURFACE REVOLVE curve AXIS x,y,z [AT x,y,z] [ANGLE deg]");
            }
            const std::vector<DVec3> base = resample(pts, kSamples, closed);
            std::vector<std::vector<DVec3>> rows;
            std::string what;
            if (sub == "extrude") {
                if (dist == 0) return out.fail("SURFACE EXTRUDE needs a DISTANCE");
                const SketchCurve& c = doc.scene.sketch().curves()[static_cast<std::size_t>(curveNamed(doc, nm))];
                const DVec3 d(dir ? *dir : curveFrame(c).normal);
                std::vector<DVec3> top;
                for (const DVec3& p : base) top.push_back(p + d * dist);
                rows = {base, top};
                what = "extruded " + lengthText(dist);
            } else {
                if (!axis) return out.fail("SURFACE REVOLVE needs an AXIS");
                const int steps = std::max(2, static_cast<int>(std::ceil(std::fabs(angle) / 15.0f)));
                const bool full = std::fabs(angle) > 359.999f;
                for (int k = 0; k <= steps; ++k) {
                    const float t = angle * static_cast<float>(k) / static_cast<float>(steps) * kPi / 180.0f;
                    std::vector<DVec3> row;
                    for (const DVec3& p : base) {
                        const Vec3 q = p.f() - at;
                        const Vec3 r = q * std::cos(t) + cross(*axis, q) * std::sin(t) + *axis * (dot(*axis, q) * (1 - std::cos(t)));
                        row.push_back(DVec3(r + at));
                    }
                    rows.push_back(full && k == steps ? rows.front() : row);
                }
                what = "revolved " + fmt3("%g", angle) + " deg";
            }
            BSplineSurface s;
            if (!interpolateSurface(rows, s, &why)) return out.fail("Cannot make the surface: " + why);
            addSurface(s, what + " from " + nm, {curveNamed(doc, nm)});
            return out;
        }
        if (cmd == "SURFACE" && sub == "planar") {
            // SURFACE PLANAR curve: a flat surface filling a closed curve
            std::size_t i = 1;
            const std::string nm = leadingName(a, quoted, i, [](const std::string&) { return false; });
            const int ci = curveNamed(doc, nm);
            if (ci < 0) return out.fail("No sketch curve named " + nm);
            const SketchCurve& c = doc.scene.sketch().curves()[static_cast<std::size_t>(ci)];
            std::string why;
            const auto prof = profileFromCurve(c, &why);
            if (!prof) return out.fail("Cannot fill " + nm + ": " + why);
            const PlaneFrame f = curveFrame(c);
            std::vector<Vec2> poly = profilePolygon(*prof);
            if (signedArea(poly) < 0) std::reverse(poly.begin(), poly.end());
            const std::vector<std::uint32_t> ids = triangulatePolygon(poly);
            if (ids.empty()) return out.fail("Cannot fill " + nm + ": its outline could not be triangulated");
            MeshData m;
            for (const Vec2& p : poly) m.vertices.push_back({f.toWorld(p), f.normal});
            m.indices = ids;
            m.computeFeatureEdges(30.0f);
            Body b = meshBody(uniqueName("Surface"), std::move(m), {0.55f, 0.75f, 0.85f});
            if (auto br = meshToBrep(b.mesh)) b.brep = br, b.brepKey = meshKey(b.mesh);
            doc.scene.pushUndo("SURFACE PLANAR");
            hideCurve(doc, ci);
            out.say("Made " + b.name + " (flat, filling " + nm + ")");
            addBody(doc, std::move(b));
            return out;
        }
        if (cmd == "SURFACE" && (sub == "offset" || sub == "blend")) {
            // SURFACE OFFSET body DISTANCE d | SURFACE BLEND bodyA bodyB [WEIGHT w]
            auto isKey = [](const std::string& w) { return w == "distance" || w == "weight"; };
            std::size_t i = 1;
            while (i < a.size() && (quotedAt(i) || !isKey(lower(a[i])))) ++i;
            const std::vector<std::string> names = nameList(1, i);
            float dist = 0, weight = 1;
            for (; i < a.size(); ++i) {
                const std::string k = lower(a[i]);
                const auto v = i + 1 < a.size() ? parseLength(a[i + 1]) : std::nullopt;
                if (!v) return out.fail(toUpper(k) + " needs a number");
                (k == "distance" ? dist : weight) = static_cast<float>(*v);
                ++i;
            }
            std::vector<BSplineSurface> ss;
            for (const std::string& nm : names) {
                const int bi = bodyNamed(doc, nm);
                if (bi < 0) return out.fail("No body named " + nm);
                BSplineSurface s;
                if (!bodySurface(doc.scene.bodies()[static_cast<std::size_t>(bi)], s)) return out.fail(nm + " is not a B-spline surface (SURFACE makes them)");
                ss.push_back(s);
            }
            constexpr int n = 25;
            std::vector<std::vector<DVec3>> rows;
            std::string what;
            if (sub == "offset") {
                if (ss.size() != 1 || dist == 0) return out.fail("Usage: SURFACE OFFSET body DISTANCE d");
                const BSplineSurface& s = ss[0];
                for (int k = 0; k < n; ++k) {
                    std::vector<DVec3> row;
                    const double u = s.u0() + (s.u1() - s.u0()) * k / (n - 1);
                    for (int j = 0; j < n; ++j) {
                        const double v = s.v0() + (s.v1() - s.v0()) * j / (n - 1);
                        row.push_back(s.eval(u, v) + surfaceNormal(s, u, v) * dist);
                    }
                    rows.push_back(row);
                }
                what = "offset " + lengthText(dist) + " from " + names[0];
            } else {
                if (ss.size() != 2) return out.fail("Usage: SURFACE BLEND bodyA bodyB [WEIGHT w]");
                // Each surface's four edges, with the slope leading off it there.
                struct Edge {
                    std::vector<DVec3> p, t;
                };
                auto edges = [&](const BSplineSurface& s) {
                    std::vector<Edge> es(4);
                    const double h = 1e-4;
                    for (int j = 0; j < n; ++j) {
                        const double fu = s.u0() + (s.u1() - s.u0()) * j / (n - 1), fv = s.v0() + (s.v1() - s.v0()) * j / (n - 1);
                        const double du = (s.u1() - s.u0()) * h, dv = (s.v1() - s.v0()) * h;
                        es[0].p.push_back(s.eval(s.u0(), fv)), es[0].t.push_back(dnormalize(s.eval(s.u0(), fv) - s.eval(s.u0() + du, fv)));
                        es[1].p.push_back(s.eval(s.u1(), fv)), es[1].t.push_back(dnormalize(s.eval(s.u1(), fv) - s.eval(s.u1() - du, fv)));
                        es[2].p.push_back(s.eval(fu, s.v0())), es[2].t.push_back(dnormalize(s.eval(fu, s.v0()) - s.eval(fu, s.v0() + dv)));
                        es[3].p.push_back(s.eval(fu, s.v1())), es[3].t.push_back(dnormalize(s.eval(fu, s.v1()) - s.eval(fu, s.v1() - dv)));
                    }
                    return es;
                };
                const std::vector<Edge> ea = edges(ss[0]), eb = edges(ss[1]);
                // The two edges facing each other (either way along).
                double best = 1e300;
                Edge A, B;
                for (const Edge& x : ea)
                    for (Edge y : eb)
                        for (int flip = 0; flip < 2; ++flip) {
                            if (flip) std::reverse(y.p.begin(), y.p.end()), std::reverse(y.t.begin(), y.t.end());
                            double d = 0;
                            for (int j = 0; j < n; ++j) d += dlength(x.p[static_cast<std::size_t>(j)] - y.p[static_cast<std::size_t>(j)]);
                            if (d < best) best = d, A = x, B = y;
                        }
                if (!(best / n > 1e-6)) return out.fail("The surfaces already meet there: nothing to blend");
                // Cubic Hermite across the gap: leaving A along its slope, arriving at B against B's.
                constexpr int across = 9;
                for (int k = 0; k < across; ++k) {
                    const double s = static_cast<double>(k) / (across - 1), s2 = s * s, s3 = s2 * s;
                    const double h00 = 2 * s3 - 3 * s2 + 1, h10 = s3 - 2 * s2 + s, h01 = -2 * s3 + 3 * s2, h11 = s3 - s2;
                    std::vector<DVec3> row;
                    for (std::size_t j = 0; j < static_cast<std::size_t>(n); ++j) {
                        const double L = dlength(B.p[j] - A.p[j]) * weight;
                        row.push_back(A.p[j] * h00 + A.t[j] * (L * h10) + B.p[j] * h01 - B.t[j] * (L * h11));
                    }
                    rows.push_back(row);
                }
                what = "blend from " + names[0] + " to " + names[1];
            }
            BSplineSurface s;
            std::string why;
            if (!interpolateSurface(rows, s, &why)) return out.fail("Cannot make the surface: " + why);
            addSurface(s, what, {});
            return out;
        }
        if (cmd == "THICKEN") {
            // THICKEN body THICKNESS t [BOTH]
            auto isKey = [](const std::string& w) { return w == "thickness" || w == "both"; };
            std::size_t i = 0;
            const std::string nm = leadingName(a, quoted, i, isKey);
            const int bi = nm.empty() ? doc.selectedBody : bodyNamed(doc, nm);
            if (bi < 0 || bi >= static_cast<int>(doc.scene.bodies().size())) return out.fail(nm.empty() ? "Select a surface, or name it" : "No body named " + nm);
            float t = 0;
            bool both = false;
            for (; i < a.size(); ++i) {
                const std::string k = lower(a[i]);
                if (k == "thickness" && i + 1 < a.size()) {
                    const auto v = parseLength(a[++i]);
                    if (!v || *v == 0) return out.fail("THICKNESS needs a length (negative: the other side)");
                    t = static_cast<float>(*v);
                } else if (k == "both") both = true;
                else return out.fail("Usage: THICKEN body THICKNESS t [BOTH]");
            }
            if (t == 0) return out.fail("Usage: THICKEN body THICKNESS t [BOTH]");
            const Body& src = doc.scene.bodies()[static_cast<std::size_t>(bi)];
            const MeshData world = worldMesh(src);
            if (openEdgeCount(world) == 0) return out.fail(src.name + " is already a closed solid: THICKEN makes solids from surfaces (SHELL hollows solids)");
            MeshData solid;
            std::string why;
            if (!thickenMesh(world, t, both, solid, &why)) return out.fail("Cannot thicken " + src.name + ": " + why);
            doc.scene.pushUndo("THICKEN " + src.name);
            Body b = featureBody(uniqueName(src.name + " solid"), std::move(solid), src.color);
            doc.scene.bodies()[static_cast<std::size_t>(bi)].visible = false;
            out.say("Thickened " + doc.scene.bodies()[static_cast<std::size_t>(bi)].name + " " + lengthText(std::fabs(t)) + (both ? " (both ways)" : "") + " into " +
                    b.name + ": " + closedText(b.mesh) + ", volume " + fmt3("%.3f", meshVolume(b.mesh) / 1000.0) + " cm3");
            addBody(doc, std::move(b));
            return out;
        }
        if (cmd == "KNIT") {
            // KNIT body body ... [TOLERANCE t]
            std::size_t end = a.size();
            float tol = -1;
            for (std::size_t k = 0; k + 1 < a.size(); ++k)
                if (!quotedAt(k) && lower(a[k]) == "tolerance") {
                    const auto v = parseLength(a[k + 1]);
                    if (!v || !(*v > 0)) return out.fail("TOLERANCE needs a length");
                    tol = static_cast<float>(*v);
                    end = k;
                }
            const std::vector<std::string> names = nameList(0, end);
            if (names.size() < 2) return out.fail("Usage: KNIT body body [body ...] [TOLERANCE t]");
            std::vector<MeshData> parts;
            std::vector<int> idx;
            for (const std::string& nm : names) {
                const int bi = bodyNamed(doc, nm);
                if (bi < 0) return out.fail("No body named " + nm);
                parts.push_back(worldMesh(doc.scene.bodies()[static_cast<std::size_t>(bi)]));
                idx.push_back(bi);
            }
            if (tol < 0) { // by default: a fiftieth of a millimetre, more on large parts
                AABB all = parts[0].bounds();
                for (const MeshData& pm : parts) all.expand(pm.bounds().min), all.expand(pm.bounds().max);
                tol = std::max(0.02f, 2e-4f * length(all.max - all.min));
            }
            bool closed = false;
            std::size_t open = 0;
            MeshData m = knitMeshes(parts, tol, closed, open);
            doc.scene.pushUndo("KNIT");
            const Vec3 colour = doc.scene.bodies()[static_cast<std::size_t>(idx[0])].color;
            std::sort(idx.rbegin(), idx.rend());
            for (int bi : idx) doc.scene.bodies().erase(doc.scene.bodies().begin() + bi);
            m.computeFeatureEdges(30.0f);
            Body b = meshBody(uniqueName(closed ? "Knit solid" : "Knit surface"), std::move(m), colour);
            out.say("Knit " + std::to_string(names.size()) + " surfaces into " + b.name + ": " +
                    (closed ? "closed, a solid of " + fmt3("%.3f", meshVolume(b.mesh) / 1000.0) + " cm3"
                            : std::to_string(open) + " open edges left (gaps wider than " + lengthText(tol) + "?)"));
            addBody(doc, std::move(b));
            return out;
        }
        if (cmd == "ANALYZE") {
            // ANALYZE ZEBRA [body] | ANALYZE CURVATURE [body] [MEAN|GAUSS] | ANALYZE OFF
            const std::string what = a.empty() ? "" : lower(a[0]);
            if (what == "off") {
                int removed = 0;
                auto& bodies = doc.scene.bodies();
                for (std::size_t k = bodies.size(); k-- > 0;) {
                    const std::string f = bodies[k].field;
                    if (f.rfind("zebra", 0) != 0 && f.find("curvature") == std::string::npos && f.rfind("draft", 0) != 0) continue;
                    const std::string src = bodies[k].attachedTo;
                    bodies.erase(bodies.begin() + static_cast<std::ptrdiff_t>(k));
                    if (const int s = bodyNamed(doc, src); s >= 0) bodies[static_cast<std::size_t>(s)].visible = true;
                    ++removed;
                }
                doc.scene.touchBodies();
                out.say(removed ? "Analysis off (" + std::to_string(removed) + " shown as before)" : "No analysis shown");
                return out;
            }
            if (what != "zebra" && what != "curvature") return out.fail("Usage: ANALYZE ZEBRA [body] | ANALYZE CURVATURE [body] [MEAN|GAUSS] | ANALYZE OFF");
            auto isKey = [](const std::string& w) { return w == "mean" || w == "gauss" || w == "gaussian"; };
            std::size_t i = 1;
            const std::string nm = leadingName(a, quoted, i, isKey);
            const int bi = nm.empty() ? doc.selectedBody : bodyNamed(doc, nm);
            if (bi < 0 || bi >= static_cast<int>(doc.scene.bodies().size())) return out.fail(nm.empty() ? "Select a body, or name it" : "No body named " + nm);
            const bool gauss = i < a.size() && lower(a[i]).rfind("gauss", 0) == 0;
            Body& src = doc.scene.bodies()[static_cast<std::size_t>(bi)];
            MeshData m = worldMesh(src);
            // A finer mesh shows the stripes and colours smoothly: from the exact surface where
            // there is one (its points and normals on it), else the mesh refined.
            if (src.exact()) {
                const AABB bb = m.bounds();
                TessellateOptions o;
                o.chord = std::max(1e-3, 5e-4 * static_cast<double>(length(bb.max - bb.min)));
                o.angle = 0.06;
                TessellateReport tr;
                MeshData fine = tessellate(*src.brep, o, &tr);
                if (tr.failedFaces == 0 && !fine.indices.empty()) {
                    fine.transform(translate(src.offset));
                    m = std::move(fine);
                }
            } else {
                Body probe = meshBody("probe", m);
                for (int pass = 0; pass < 3 && probe.mesh.indices.size() < 60000; ++pass) {
                    const AABB bb = probe.mesh.bounds();
                    sculptRefine(probe, 0.01f * length(bb.max - bb.min));
                }
                m = probe.mesh;
            }
            { // smooth normals (area-weighted over the welded vertices): stripes follow the surface, not its facets
                std::map<std::tuple<float, float, float>, Vec3> acc;
                for (std::size_t k = 0; k + 2 < m.indices.size(); k += 3) {
                    const Vec3 a3 = m.vertices[m.indices[k]].position, b3 = m.vertices[m.indices[k + 1]].position, c3 = m.vertices[m.indices[k + 2]].position;
                    const Vec3 n = cross(b3 - a3, c3 - a3);
                    for (const Vec3& p : {a3, b3, c3}) acc[{p.x, p.y, p.z}] += n;
                }
                for (Vertex& v : m.vertices) v.normal = normalize(acc[{v.position.x, v.position.y, v.position.z}]);
            }
            std::string field;
            if (what == "zebra") {
                for (Vertex& v : m.vertices) v.mask = 0;
                field = "zebra stripes (reflections: smooth stripes, smooth surface)";
            } else {
                std::vector<float> H, K;
                meshCurvature(m, H, K);
                const std::vector<float>& c = gauss ? K : H;
                std::vector<float> mag;
                for (float x : c) mag.push_back(std::fabs(x));
                std::sort(mag.begin(), mag.end());
                const float top = std::max(mag.empty() ? 0.0f : mag[mag.size() * 95 / 100], 1e-6f);
                for (std::size_t v = 0; v < m.vertices.size(); ++v) m.vertices[v].mask = std::clamp(0.5f + 0.5f * c[v] / top, 0.0f, 1.0f);
                field = std::string(gauss ? "Gaussian curvature, 1/mm2: " : "mean curvature, 1/mm: ") + fmt3("%.3g", -top) + " (blue) to " + fmt3("%.3g", top) + " (red)";
            }
            doc.scene.pushUndo("ANALYZE " + src.name);
            Body res = meshBody(uniqueName(src.name + " " + what), std::move(m), src.color);
            res.field = field;
            res.attachedTo = src.name;
            src.visible = false;
            out.say("Showing " + field + " on " + res.name + " (ANALYZE OFF puts " + src.name + " back)");
            addBody(doc, std::move(res));
            return out;
        }
        return out.fail("Usage: SURFACE LOFT|BOUNDARY|EXTRUDE|REVOLVE|PLANAR|OFFSET|BLEND ...  (HELP SURFACE)");
    }

    if (cmd == "DOME") {
        // DOME [body] FACE x,y,z HEIGHT h [CONCAVE]
        auto isKey = [](const std::string& w) { return w == "face" || w == "height" || w == "concave"; };
        const char* usage = "Usage: DOME [body] FACE x,y,z HEIGHT h [CONCAVE]";
        std::size_t i = 0;
        const std::string name = leadingName(a, quoted, i, isKey);
        const int bi = name.empty() ? doc.selectedBody : bodyNamed(doc, name);
        if (bi < 0 || bi >= static_cast<int>(doc.scene.bodies().size())) return out.fail(name.empty() ? "Select a body, or name it" : "No body named " + name);
        std::optional<Vec3> at;
        float height = 0;
        bool concave = false;
        for (; i < a.size(); ++i) {
            const std::string k = lower(a[i]);
            if (k == "face" && i + 1 < a.size()) {
                if (!(at = parseVec3(a[++i]))) return out.fail("FACE needs a point x,y,z on the face");
            } else if (k == "height" && i + 1 < a.size()) {
                const auto v = parseLength(a[++i]);
                if (!v || !(*v > 0)) return out.fail("HEIGHT needs a length");
                height = static_cast<float>(*v);
            } else if (k == "concave") concave = true;
            else return out.fail(usage);
        }
        if (!at || !(height > 0)) return out.fail(usage);
        Body& body = doc.scene.bodies()[static_cast<std::size_t>(bi)];
        const MeshData world = worldMesh(body);
        // The flat face at the point: its plane, and the body's outline just inside it.
        Vec3 q{}, n{0, 0, 1};
        if (!nearestFace(world, *at, q, n)) return out.fail(body.name + " has no faces");
        PlaneFrame f;
        makePlaneFrame(q, n, std::fabs(n.z) > 0.9f ? Vec3{1, 0, 0} : cross(Vec3{0, 0, 1}, n), f);
        const AABB wb = world.bounds();
        const float eps = 1e-4f * length(wb.max - wb.min);
        PlaneFrame inside = f;
        inside.origin = q - n * eps;
        std::vector<Vec2> outline;
        for (std::vector<Vec2>& l : planeSection(world, inside)) {
            if (signedArea(l) <= 0) continue; // a hole's loop
            bool in = false; // the loop around the point
            for (std::size_t k = 0, j = l.size() - 1; k < l.size(); j = k++)
                if ((l[k].y > 0) != (l[j].y > 0) && 0 < (l[j].x - l[k].x) * (0 - l[k].y) / (l[j].y - l[k].y) + l[k].x) in = !in;
            if (in) outline = std::move(l);
        }
        if (outline.size() < 3) return out.fail("No flat face of " + body.name + " at that point");
        // Drop points in line (facets of a flat face) so the insets stay clean.
        for (bool changed = true; changed && outline.size() > 3;) {
            changed = false;
            for (std::size_t k = 0; k < outline.size(); ++k) {
                const Vec2 p0 = outline[(k + outline.size() - 1) % outline.size()], p1 = outline[k], p2 = outline[(k + 1) % outline.size()];
                const float cr = (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x);
                if (std::fabs(cr) <= 1e-6f * std::max(1.0f, std::hypot(p2.x - p0.x, p2.y - p0.y))) {
                    outline.erase(outline.begin() + static_cast<std::ptrdiff_t>(k));
                    changed = true;
                    break;
                }
            }
        }
        // How far in the outline can shrink before it turns over: the dome's reach.
        float lo = 0, hi = 0;
        {
            Vec2 bl = outline[0], bh = outline[0];
            for (const Vec2& p : outline) bl = {std::min(bl.x, p.x), std::min(bl.y, p.y)}, bh = {std::max(bh.x, p.x), std::max(bh.y, p.y)};
            hi = 0.5f * std::min(bh.x - bl.x, bh.y - bl.y);
            for (int it = 0; it < 30; ++it) {
                const float mid = 0.5f * (lo + hi);
                (offsetPolygon(outline, mid).empty() ? hi : lo) = mid;
            }
        }
        const float reach = lo * 0.98f;
        if (!(reach > 0)) return out.fail("The face is too small or too thin for a dome");
        // Rings rising as a quarter ellipse: in by reach (1 - cos a), up by height sin a.
        const int steps = 24;
        const float sgn = concave ? -1.0f : 1.0f;
        std::vector<std::pair<float, std::vector<Vec2>>> rings;
        rings.push_back({-eps * 20 * sgn, outline}); // a hair into (or out of) the body, so they join cleanly
        for (int k = 0; k <= steps; ++k) {
            const float t = static_cast<float>(k) / static_cast<float>(steps) * kPi * 0.5f;
            std::vector<Vec2> r = offsetPolygon(outline, reach * (1 - std::cos(t)));
            if (r.empty()) break;
            rings.push_back({height * std::sin(t) * sgn, std::move(r)});
        }
        MeshData m;
        auto world3 = [&](Vec2 p, float z) { return f.toWorld(p) + f.normal * z; };
        auto tri = [&](Vec3 a3, Vec3 b3, Vec3 c3) {
            const Vec3 nr = normalize(cross(b3 - a3, c3 - a3));
            const auto base = static_cast<std::uint32_t>(m.vertices.size());
            m.vertices.push_back({a3, nr}), m.vertices.push_back({b3, nr}), m.vertices.push_back({c3, nr});
            m.indices.insert(m.indices.end(), {base, base + 1, base + 2});
        };
        const std::size_t np = outline.size();
        for (std::size_t r = 0; r + 1 < rings.size(); ++r)
            for (std::size_t k = 0; k < np; ++k) {
                const std::size_t j = (k + 1) % np;
                const Vec3 a0 = world3(rings[r].second[k], rings[r].first), b0 = world3(rings[r].second[j], rings[r].first);
                const Vec3 a1 = world3(rings[r + 1].second[k], rings[r + 1].first), b1 = world3(rings[r + 1].second[j], rings[r + 1].first);
                tri(a0, b0, b1), tri(a0, b1, a1);
            }
        for (int end = 0; end < 2; ++end) { // the base and the small top
            const auto& ring = end ? rings.back() : rings.front();
            const std::vector<std::uint32_t> ids = triangulatePolygon(ring.second);
            if (ids.empty()) return out.fail("The face's outline could not be filled");
            for (std::size_t k = 0; k + 2 < ids.size(); k += 3) {
                const Vec3 a3 = world3(ring.second[ids[k]], ring.first), b3 = world3(ring.second[ids[k + 1]], ring.first),
                           c3 = world3(ring.second[ids[k + 2]], ring.first);
                if (end) tri(a3, b3, c3);
                else tri(a3, c3, b3);
            }
        }
        if (concave) // the same shape, wound inside out, is a cutter going down into the body
            for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) std::swap(m.indices[t + 1], m.indices[t + 2]);
        if (meshVolume(m) < 0)
            for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) std::swap(m.indices[t + 1], m.indices[t + 2]);
        doc.scene.pushUndo("DOME " + body.name);
        const double before = meshVolume(world);
        std::string why;
        if (!bodyBoolean(body, meshBody("dome", std::move(m)), concave ? BooleanOp::Subtract : BooleanOp::Union, &why)) {
            doc.scene.undo();
            return out.fail("Cannot dome " + doc.scene.bodies()[static_cast<std::size_t>(bi)].name + ": " + why);
        }
        Body& done = doc.scene.bodies()[static_cast<std::size_t>(bi)];
        const bool exact = done.exact();
        SculptMesh(done).writeTo(done, true, 40.0f);
        if (exact) done.brepKey = meshKey(done.mesh); // the same surface, new normals
        doc.scene.touchBodies();
        doc.dirty = true;
        out.say((concave ? "Dished " : "Domed ") + done.name + " " + lengthText(height) + (concave ? " deep: " : " high: ") + closedText(done.mesh) + ", volume " +
                fmt3("%.2f", before / 1000.0) + " -> " + fmt3("%.2f", meshVolume(worldMesh(done)) / 1000.0) + " cm3");
        return out;
    }

    if (cmd == "RIB") {
        // RIB [curve] THICKNESS t [BODY name] [FLIP] [KEEP]
        auto isKey = [](const std::string& w) { return w == "thickness" || w == "body" || w == "flip" || w == "keep"; };
        const char* usage = "Usage: RIB [curve] THICKNESS t [BODY name] [FLIP] [KEEP]";
        std::size_t i = 0;
        const std::string name = leadingName(a, quoted, i, isKey);
        const int ci = name.empty() ? doc.selectedCurve : curveNamed(doc, name);
        if (ci < 0 || ci >= static_cast<int>(doc.scene.sketch().curves().size()))
            return out.fail(name.empty() ? "Select an open sketch curve, or name it" : "No sketch curve named " + name);
        float thick = 0;
        std::string target;
        bool flip = false, keep = false;
        for (; i < a.size(); ++i) {
            const std::string k = lower(a[i]);
            if (k == "thickness" && i + 1 < a.size()) {
                const auto v = parseLength(a[++i]);
                if (!v || !(*v > 0)) return out.fail("THICKNESS needs a length");
                thick = static_cast<float>(*v);
            } else if (k == "body" && i + 1 < a.size()) target = a[++i];
            else if (k == "flip") flip = true;
            else if (k == "keep") keep = true;
            else return out.fail(usage);
        }
        if (!(thick > 0)) return out.fail(usage);
        const SketchCurve curve = doc.scene.sketch().curves()[static_cast<std::size_t>(ci)];
        if (curve.isClosed()) return out.fail("A rib is drawn from an open curve (its free edge)");
        const PlaneFrame f = curveFrame(curve);
        std::vector<Vec2> strip = curve.planeStrip(0.5f);
        if (strip.size() < 2) return out.fail(curve.name + " is too short for a rib");
        float span = 0;
        for (std::size_t k = 1; k < strip.size(); ++k) span += std::hypot(strip[k].x - strip[k - 1].x, strip[k].y - strip[k - 1].y);
        {   // even samples along the curve, so the rib follows the surface below it
            std::vector<Vec2> even{strip.front()};
            const float step = span / 64.0f;
            for (std::size_t k = 1; k < strip.size(); ++k) {
                const Vec2 p0 = strip[k - 1], p1 = strip[k];
                const int cuts = std::max(1, static_cast<int>(std::ceil(std::hypot(p1.x - p0.x, p1.y - p0.y) / step)));
                for (int c = 1; c <= cuts; ++c) {
                    const float t = static_cast<float>(c) / static_cast<float>(cuts);
                    even.push_back({p0.x + (p1.x - p0.x) * t, p0.y + (p1.y - p0.y) * t});
                }
            }
            strip = std::move(even);
        }
        // The rib grows in the sketch plane, square to the curve's chord, toward the body.
        const Vec2 chord{strip.back().x - strip.front().x, strip.back().y - strip.front().y};
        const float cl = std::hypot(chord.x, chord.y);
        if (!(cl > 1e-4f)) return out.fail("The curve's ends meet; draw an open edge for the rib");
        Vec2 d2{-chord.y / cl, chord.x / cl};
        const Vec3 o3 = f.toWorld({0, 0});
        auto dirOf = [&](Vec2 v) { return normalize(f.toWorld(v) - o3); };
        const Vec3 mid = f.toWorld(strip[strip.size() / 2]);
        int bi = target.empty() ? -1 : bodyNamed(doc, target);
        if (!target.empty() && bi < 0) return out.fail("No body named " + target);
        if (!flip) { // the side the body is on (or FLIP the other)
            auto hit = [&](Vec2 v) {
                if (bi >= 0) return upToNext(doc, {mid}, dirOf(v), bi);
                return upToNext(doc, {mid}, dirOf(v), -1);
            };
            const auto fwd = hit(d2), back = hit({-d2.x, -d2.y});
            if (!fwd || (back && *back < *fwd)) d2 = {-d2.x, -d2.y};
        } else
            d2 = {-d2.x, -d2.y};
        const Vec3 d3 = dirOf(d2);
        if (bi < 0) { // the nearest body that way
            float best = 1e30f;
            for (std::size_t b = 0; b < doc.scene.bodies().size(); ++b)
                if (doc.scene.bodies()[b].visible)
                    if (const auto h = upToNext(doc, {mid}, d3, static_cast<int>(b)); h && *h < best) best = *h, bi = static_cast<int>(b);
            if (bi < 0) return out.fail("No body below " + curve.name + " for the rib to meet");
        }
        // How far down to the body under each sample, plus a little into it.
        std::vector<Vec2> outline = strip;
        const float back = 0.01f * span;
        for (std::size_t k = strip.size(); k-- > 0;) {
            const Vec3 p = f.toWorld(strip[k]) - d3 * back; // from a hair behind, so an end on the surface still meets it
            const auto h = upToNext(doc, {p}, d3, bi);
            if (!h) return out.fail("Part of " + curve.name + " is not over " + doc.scene.bodies()[static_cast<std::size_t>(bi)].name + " (try FLIP)");
            const auto wall = upToNext(doc, {p + d3 * (*h + 1e-3f)}, d3, bi); // the body's thickness there
            const float down = std::max(0.0f, *h - back) + std::min(thick, wall ? 0.5f * *wall : thick);
            outline.push_back({strip[k].x + d2.x * down, strip[k].y + d2.y * down});
        }
        if (signedArea(outline) < 0) std::reverse(outline.begin(), outline.end());
        if (!isSimplePolygon(outline)) return out.fail("The rib's outline crosses itself; use a flatter curve");
        const std::vector<std::uint32_t> ids = triangulatePolygon(outline);
        if (ids.empty()) return out.fail("The rib's outline could not be filled");
        MeshData m;
        auto tri = [&](Vec3 a3, Vec3 b3, Vec3 c3) {
            const Vec3 nr = normalize(cross(b3 - a3, c3 - a3));
            const auto base = static_cast<std::uint32_t>(m.vertices.size());
            m.vertices.push_back({a3, nr}), m.vertices.push_back({b3, nr}), m.vertices.push_back({c3, nr});
            m.indices.insert(m.indices.end(), {base, base + 1, base + 2});
        };
        auto at = [&](Vec2 p, float z) { return f.toWorld(p) + f.normal * z; };
        const float h2 = 0.5f * thick;
        for (std::size_t k = 0; k + 2 < ids.size(); k += 3) {
            const Vec2 p0 = outline[ids[k]], p1 = outline[ids[k + 1]], p2 = outline[ids[k + 2]];
            tri(at(p0, h2), at(p1, h2), at(p2, h2));
            tri(at(p0, -h2), at(p2, -h2), at(p1, -h2));
        }
        for (std::size_t k = 0; k < outline.size(); ++k) {
            const Vec2 p0 = outline[k], p1 = outline[(k + 1) % outline.size()];
            tri(at(p0, -h2), at(p1, -h2), at(p1, h2)), tri(at(p0, -h2), at(p1, h2), at(p0, h2));
        }
        if (meshVolume(m) < 0)
            for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) std::swap(m.indices[t + 1], m.indices[t + 2]);
        Body& body = doc.scene.bodies()[static_cast<std::size_t>(bi)];
        doc.scene.pushUndo("RIB " + body.name);
        const double before = meshVolume(worldMesh(body));
        std::string why;
        if (!bodyBoolean(body, meshBody("rib", std::move(m)), BooleanOp::Union, &why)) {
            doc.scene.undo();
            return out.fail("Cannot add the rib to " + doc.scene.bodies()[static_cast<std::size_t>(bi)].name + ": " + why);
        }
        Body& done = doc.scene.bodies()[static_cast<std::size_t>(bi)];
        if (!keep) hideCurve(doc, ci), doc.selectedCurve = -1;
        doc.scene.touchBodies();
        doc.dirty = true;
        out.say("Added a " + lengthText(thick) + " rib from " + curve.name + " to " + done.name + ": " + closedText(done.mesh) + ", volume " +
                fmt3("%.2f", before / 1000.0) + " -> " + fmt3("%.2f", meshVolume(worldMesh(done)) / 1000.0) + " cm3");
        return out;
    }

    if (cmd == "EMBOSS" || cmd == "ENGRAVE") {
        // EMBOSS|ENGRAVE [body] TEXT "words" FACE x,y,z HEIGHT h DEPTH d [ANGLE deg] [FONT sans|bold|mono]
        auto isKey = [](const std::string& w) { return w == "text" || w == "face" || w == "height" || w == "depth" || w == "angle" || w == "font"; };
        const std::string usage = "Usage: " + cmd + " [body] TEXT \"words\" FACE x,y,z HEIGHT h DEPTH d [ANGLE deg] [FONT sans|bold|mono]";
        const bool cut = cmd == "ENGRAVE";
        std::size_t i = 0;
        const std::string name = leadingName(a, quoted, i, isKey);
        const int bi = name.empty() ? doc.selectedBody : bodyNamed(doc, name);
        if (bi < 0 || bi >= static_cast<int>(doc.scene.bodies().size())) return out.fail(name.empty() ? "Select a body, or name it" : "No body named " + name);
        std::string words;
        std::optional<Vec3> at;
        float height = 0, depth = 0, angle = 0;
        TextFont font = TextFont::SansBold;
        for (; i < a.size(); ++i) {
            const std::string k = lower(a[i]);
            if (k == "text" && i + 1 < a.size()) {
                words = a[++i];
                for (std::size_t p; (p = words.find("\\n")) != std::string::npos;) words.replace(p, 2, "\n");
            } else if (k == "face" && i + 1 < a.size()) {
                if (!(at = parseVec3(a[++i]))) return out.fail("FACE needs a point x,y,z on the face");
            } else if ((k == "height" || k == "depth") && i + 1 < a.size()) {
                const auto v = parseLength(a[++i]);
                if (!v || !(*v > 0)) return out.fail(a[i - 1] + " needs a length");
                (k == "height" ? height : depth) = static_cast<float>(*v);
            } else if (k == "angle" && i + 1 < a.size()) {
                const auto v = parseNumber(a[++i]);
                if (!v) return out.fail("ANGLE needs degrees");
                angle = *v * kPi / 180.0f;
            } else if (k == "font" && i + 1 < a.size()) {
                if (!textFontNamed(a[++i], font)) return out.fail("FONT is sans, bold or mono");
            } else return out.fail(usage);
        }
        if (words.empty() || !at || !(height > 0) || !(depth > 0)) return out.fail(usage);
        std::string why;
        float width = 0;
        std::vector<std::vector<Vec2>> loops = textOutline(words, height, font, &width, &why);
        if (loops.empty()) return out.fail("Cannot write \"" + words + "\": " + why);
        Body& body = doc.scene.bodies()[static_cast<std::size_t>(bi)];
        const MeshData world = worldMesh(body);
        Vec3 q{}, n{0, 0, 1};
        if (!nearestFace(world, *at, q, n)) return out.fail(body.name + " has no faces");
        // The text's frame: on the face at the point, reading along +X (or across a side
        // face, upright), turned by ANGLE; the words centred on the point.
        PlaneFrame f;
        makePlaneFrame(q, n, std::fabs(n.z) > 0.9f ? Vec3{1, 0, 0} : cross(Vec3{0, 0, 1}, n), f);
        const auto extraLines = static_cast<float>(std::count(words.begin(), words.end(), '\n'));
        const float ca = std::cos(angle), sa = std::sin(angle);
        const float midY = 0.5f * (height - extraLines * 1.6f * height); // centre on the capitals of all the lines
        for (auto& l : loops)
            for (Vec2& p : l) {
                const Vec2 c{p.x - width * 0.5f, p.y - midY};
                p = {c.x * ca - c.y * sa, c.x * sa + c.y * ca};
            }
        // The surface under the text, as a height along the face normal at each point: the
        // hit nearest the face's plane among the triangles facing the same way.
        const AABB wb = world.bounds();
        const float reach = length(wb.max - wb.min) + 1.0f;
        std::vector<std::array<Vec3, 3>> facing;
        Vec2 lo{1e30f, 1e30f}, hi{-1e30f, -1e30f};
        for (const auto& l : loops)
            for (const Vec2& p : l) lo = {std::min(lo.x, p.x), std::min(lo.y, p.y)}, hi = {std::max(hi.x, p.x), std::max(hi.y, p.y)};
        for (std::size_t t = 0; t + 2 < world.indices.size(); t += 3) {
            const std::array<Vec3, 3> tr{world.vertices[world.indices[t]].position, world.vertices[world.indices[t + 1]].position,
                                         world.vertices[world.indices[t + 2]].position};
            if (!(dot(cross(tr[1] - tr[0], tr[2] - tr[0]), n) > 0)) continue;
            Vec2 tl{1e30f, 1e30f}, th{-1e30f, -1e30f};
            for (const Vec3& v : tr) {
                const Vec2 p = f.toPlane(v);
                tl = {std::min(tl.x, p.x), std::min(tl.y, p.y)}, th = {std::max(th.x, p.x), std::max(th.y, p.y)};
            }
            if (th.x >= lo.x && tl.x <= hi.x && th.y >= lo.y && tl.y <= hi.y) facing.push_back(tr);
        }
        auto surface = [&](Vec2 p) -> std::optional<float> {
            const Ray ray{f.toWorld(p) + n * reach, n * -1.0f};
            float best = 1e30f;
            for (const auto& tr : facing) {
                float d;
                if (intersectTriangle(ray, tr[0], tr[1], tr[2], d) && std::fabs(reach - d) < std::fabs(best)) best = reach - d;
            }
            if (best >= 1e30f) return std::nullopt;
            return best;
        };
        // Flat under the text: the letters stand straight up from the plane. Curved: their
        // outlines are cut finer and every point follows the surface.
        bool flat = true;
        for (const auto& l : loops)
            for (const Vec2& p : l) {
                const auto s = surface(p);
                if (!s) return out.fail("The text runs off the face of " + body.name + "; make it smaller or move it");
                if (std::fabs(*s) > 1e-3f * height) flat = false;
            }
        const float fine = height / 8.0f;
        MeshData solid, holes;
        auto prism = [&](std::vector<Vec2> poly, MeshData& m, float under, float over) -> bool {
            if (signedArea(poly) < 0) std::reverse(poly.begin(), poly.end());
            std::vector<std::uint32_t> ids = triangulatePolygon(poly);
            if (ids.empty()) return false;
            int level = 0; // each cap triangle cut into 4^level, each outline edge into 2^level
            if (!flat) {
                float longest = 0;
                for (std::size_t k = 0; k + 2 < ids.size(); k += 3)
                    for (int e = 0; e < 3; ++e) {
                        const Vec2 p0 = poly[ids[k + static_cast<std::size_t>(e)]], p1 = poly[ids[k + static_cast<std::size_t>((e + 1) % 3)]];
                        longest = std::max(longest, std::hypot(p1.x - p0.x, p1.y - p0.y));
                    }
                while (level < 4 && longest / static_cast<float>(1 << level) > fine) ++level;
            }
            const int cuts = 1 << level;
            auto point = [&](Vec2 p, float off) {
                const float s = flat ? 0.0f : surface(p).value_or(0.0f);
                return f.toWorld(p) + n * (s + off);
            };
            auto tri = [&](Vec3 a3, Vec3 b3, Vec3 c3) {
                const Vec3 nr = normalize(cross(b3 - a3, c3 - a3));
                const auto base = static_cast<std::uint32_t>(m.vertices.size());
                m.vertices.push_back({a3, nr}), m.vertices.push_back({b3, nr}), m.vertices.push_back({c3, nr});
                m.indices.insert(m.indices.end(), {base, base + 1, base + 2});
            };
            // A point a fraction t from p to q, worked out the same way from either end, so
            // the triangles on both sides of an edge share its points exactly.
            auto along = [&](Vec2 p, Vec2 e, int step) {
                if (step == 0) return p;
                if (step == cuts) return e;
                if (e.x < p.x || (e.x == p.x && e.y < p.y)) std::swap(p, e), step = cuts - step;
                const float t = static_cast<float>(step) / static_cast<float>(cuts);
                return Vec2{p.x + (e.x - p.x) * t, p.y + (e.y - p.y) * t};
            };
            for (std::size_t k = 0; k + 2 < ids.size(); k += 3) {
                const Vec2 A = poly[ids[k]], B = poly[ids[k + 1]], C = poly[ids[k + 2]];
                auto bary = [&](int r, int c) { // r steps toward B, c toward C
                    if (c == 0) return along(A, B, r);
                    if (r == 0) return along(A, C, c);
                    if (r + c == cuts) return along(C, B, r);
                    const float u = static_cast<float>(r) / static_cast<float>(cuts), v = static_cast<float>(c) / static_cast<float>(cuts);
                    return Vec2{A.x + (B.x - A.x) * u + (C.x - A.x) * v, A.y + (B.y - A.y) * u + (C.y - A.y) * v};
                };
                for (int r = 0; r < cuts; ++r)
                    for (int c = 0; r + c < cuts; ++c) {
                        const Vec2 p0 = bary(r, c), p1 = bary(r + 1, c), p2 = bary(r, c + 1);
                        tri(point(p0, over), point(p1, over), point(p2, over));
                        tri(point(p0, under), point(p2, under), point(p1, under));
                        if (r + c + 1 < cuts) {
                            const Vec2 p3 = bary(r + 1, c + 1);
                            tri(point(p1, over), point(p3, over), point(p2, over));
                            tri(point(p1, under), point(p2, under), point(p3, under));
                        }
                    }
            }
            for (std::size_t k = 0; k < poly.size(); ++k) {
                const Vec2 p0 = poly[k], p1 = poly[(k + 1) % poly.size()];
                for (int c = 0; c < cuts; ++c) {
                    const Vec2 a2 = along(p0, p1, c), b2 = along(p0, p1, c + 1);
                    tri(point(a2, under), point(b2, under), point(b2, over)), tri(point(a2, under), point(b2, over), point(a2, over));
                }
            }
            return true;
        };
        // Raised letters stand from a little under the surface; cut ones reach a little over.
        const float under = cut ? -depth : -0.25f * depth, over = cut ? 0.25f * depth : depth;
        for (const auto& l : loops) {
            const bool hole = signedArea(l) < 0;
            if (!prism(l, hole ? holes : solid, hole ? under - 0.25f * depth : under, hole ? over + 0.25f * depth : over))
                return out.fail("A letter's outline could not be filled");
        }
        Body tool = meshBody("text", std::move(solid));
        if (!holes.indices.empty() && !bodyBoolean(tool, meshBody("holes", std::move(holes)), BooleanOp::Subtract, &why))
            return out.fail("Cannot shape the letters: " + why);
        doc.scene.pushUndo(cmd + " " + body.name);
        const double before = meshVolume(world);
        if (!bodyBoolean(body, tool, cut ? BooleanOp::Subtract : BooleanOp::Union, &why)) {
            doc.scene.undo();
            return out.fail("Cannot " + lower(cmd) + " " + doc.scene.bodies()[static_cast<std::size_t>(bi)].name + ": " + why);
        }
        Body& done = doc.scene.bodies()[static_cast<std::size_t>(bi)];
        doc.scene.touchBodies();
        doc.dirty = true;
        out.say((cut ? "Engraved \"" : "Embossed \"") + words + "\" " + lengthText(height) + " high, " + lengthText(depth) + (cut ? " deep" : " proud") + " on " +
                done.name + (flat ? "" : " (following the curved face)") + ": " + closedText(done.mesh) + ", volume " + fmt3("%.2f", before / 1000.0) + " -> " +
                fmt3("%.2f", meshVolume(worldMesh(done)) / 1000.0) + " cm3");
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
    { // the MATE steps put the mates back; the exploded view is a way of showing them, kept
        ExplodeState explode = std::move(doc.state.assembly.explode);
        doc.state.assembly = {};
        doc.state.assembly.explode = std::move(explode);
    }
    doc.clearSelection();
    doc.primitiveSerial = 0;
    doc.state.plane = SketchPlane::Top;
    doc.state.customPlane.clear();
    doc.state.datums.clear(); // the PLANE, REFAXIS and REFPOINT steps make them again
    doc.state.refAxes.clear();
    doc.state.refPoints.clear();
    doc.replaying = true;
    // Rolled back: only the steps before the rollback point; suppressed steps are skipped.
    const std::size_t built = doc.state.rollback >= 0 ? std::min(steps.size(), static_cast<std::size_t>(doc.state.rollback)) : steps.size();
    for (std::size_t k = 0; k < built; ++k) {
        if (steps[k].suppressed) continue;
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
        for (std::size_t k = 0; k < st.history.size(); ++k) {
            if (static_cast<int>(k) == st.rollback) out.say("  -- rolled back here: the steps below are not built (HISTORY ROLLFORWARD) --");
            out.say("  " + std::to_string(k + 1) + ". " + st.history[k].line + (st.history[k].suppressed ? "   (suppressed)" : ""));
        }
        return out;
    }
    // Step numbers from a[from] on: "3", "3 5 7" or "3-5".
    auto stepList = [&](std::size_t from, std::vector<std::size_t>& ks) {
        for (std::size_t i = from; i < a.size(); ++i) {
            const std::string w = a[i];
            const std::size_t dash = w.find('-', 1);
            const auto lo = parseNumber(dash == std::string::npos ? w : w.substr(0, dash));
            const auto hi = dash == std::string::npos ? lo : parseNumber(w.substr(dash + 1));
            if (!lo || !hi || *lo < 1 || *hi < *lo || *hi > static_cast<double>(st.history.size())) return false;
            for (auto k = static_cast<std::size_t>(*lo); k <= static_cast<std::size_t>(*hi); ++k) ks.push_back(k - 1);
        }
        return !ks.empty();
    };
    if (sub == "suppress" || sub == "unsuppress") {
        std::vector<std::size_t> ks;
        if (!stepList(1, ks)) return out.fail("HISTORY " + toUpper(sub) + " n [m ...]: steps' numbers (or a range a-b; HISTORY lists them)");
        std::vector<HistoryStep> steps = st.history;
        for (const std::size_t k : ks) steps[k].suppressed = sub == "suppress";
        if (!regenerateHistory(doc, steps, out)) return out;
        out.say((sub == "suppress" ? "Suppressed " : "Unsuppressed ") + std::string(ks.size() == 1 ? "step " : "steps ") + rejoinArgs(a, quoted, 1) +
                " and regenerated: " + std::to_string(doc.scene.bodies().size()) + " bodies");
        return out;
    }
    if (sub == "rollback" || sub == "rollforward") {
        const int was = st.rollback;
        if (sub == "rollforward" || (a.size() > 1 && (lower(a[1]) == "end" || lower(a[1]) == "off"))) st.rollback = -1;
        else {
            const auto n = a.size() > 1 ? parseNumber(a[1]) : std::nullopt;
            if (!n || *n < 0 || *n > static_cast<double>(st.history.size()))
                return out.fail("HISTORY ROLLBACK n: build only steps 1 to n (0: none) and add new steps there; HISTORY ROLLFORWARD builds them all");
            st.rollback = static_cast<int>(*n) == static_cast<int>(st.history.size()) ? -1 : static_cast<int>(*n);
        }
        const std::vector<HistoryStep> steps = st.history;
        if (!regenerateHistory(doc, steps, out)) {
            st.rollback = was;
            return out;
        }
        if (st.rollback < 0) out.say("Rolled forward: all " + std::to_string(steps.size()) + " steps built");
        else
            out.say("Rolled back to step " + std::to_string(st.rollback) + " of " + std::to_string(steps.size()) +
                    ": new steps go in there; HISTORY ROLLFORWARD builds the rest");
        return out;
    }
    if (sub == "move") { // HISTORY MOVE n [TO|BEFORE] m: step n becomes step m
        std::size_t i = 1;
        const auto n = a.size() > i ? parseNumber(a[i++]) : std::nullopt;
        if (i < a.size() && (lower(a[i]) == "to" || lower(a[i]) == "before")) ++i;
        const auto m = a.size() > i ? parseNumber(a[i]) : std::nullopt;
        const auto size = static_cast<double>(st.history.size());
        if (!n || !m || *n < 1 || *m < 1 || *n > size || *m > size) return out.fail("HISTORY MOVE n TO m: step n becomes step m (HISTORY lists them)");
        const auto from = static_cast<std::size_t>(*n) - 1, to = static_cast<std::size_t>(*m) - 1;
        std::vector<HistoryStep> steps = st.history;
        HistoryStep moved = steps[from];
        steps.erase(steps.begin() + static_cast<std::ptrdiff_t>(from));
        steps.insert(steps.begin() + static_cast<std::ptrdiff_t>(to), std::move(moved));
        if (!regenerateHistory(doc, steps, out)) return out;
        out.say("Moved step " + std::to_string(from + 1) + " to " + std::to_string(to + 1) + " and regenerated " + std::to_string(steps.size()) + " steps");
        return out;
    }
    if (sub == "set") { // HISTORY SET n KEY value [KEY value ...]: change a feature's parameters
        const auto n = a.size() > 1 ? parseNumber(a[1]) : std::nullopt;
        if (!n || *n < 1 || *n > static_cast<double>(st.history.size()) || a.size() < 4 || (a.size() - 2) % 2 != 0)
            return out.fail("HISTORY SET n KEY value [KEY value ...]: change parameters of step n, e.g. HISTORY SET 2 DIAMETER 10");
        const auto k = static_cast<std::size_t>(*n) - 1;
        ParsedCommand pc = tokenizeCommand(st.history[k].line);
        for (std::size_t i = 2; i + 1 < a.size(); i += 2) {
            bool found = false;
            for (std::size_t j = 0; j + 1 < pc.args.size() && !found; ++j)
                if (!(j < pc.quoted.size() && pc.quoted[j]) && lower(pc.args[j]) == lower(a[i])) {
                    pc.args[j + 1] = a[i + 1];
                    if (j + 1 < pc.quoted.size()) pc.quoted[j + 1] = i + 1 < quoted.size() && quoted[i + 1];
                    found = true;
                }
            if (!found) return out.fail("Step " + std::to_string(k + 1) + " (" + st.history[k].line + ") has no " + toUpper(a[i]) + " to change");
        }
        std::vector<HistoryStep> steps = st.history;
        steps[k].line = pc.name + (pc.args.empty() ? "" : " " + rejoinArgs(pc.args, pc.quoted, 0));
        if (!regenerateHistory(doc, steps, out)) return out;
        out.say("Step " + std::to_string(k + 1) + " is now " + steps[k].line + "; regenerated " + std::to_string(steps.size()) + " steps");
        return out;
    }
    if (sub == "clear") {
        st.history.clear();
        st.historyRedo.clear();
        st.historyComplete = true;
        st.rollback = -1;
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
        const int was = st.rollback;
        if (sub == "delete") {
            steps.erase(steps.begin() + static_cast<std::ptrdiff_t>(k));
            if (st.rollback > static_cast<int>(k)) --st.rollback;
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
            } else {
                steps.insert(steps.begin() + static_cast<std::ptrdiff_t>(k), step);
                if (st.rollback >= static_cast<int>(k)) ++st.rollback;
            }
        }
        if (st.rollback >= static_cast<int>(steps.size())) st.rollback = -1;
        if (!regenerateHistory(doc, steps, out)) {
            st.rollback = was;
            return out;
        }
        out.say((sub == "delete" ? "Deleted step " : sub == "edit" ? "Changed step " : "Inserted step ") + std::to_string(k + 1) + " and regenerated " +
                std::to_string(steps.size()) + " steps");
        return out;
    }
    return out.fail("Usage: HISTORY [LIST] | HISTORY EDIT n command | HISTORY SET n KEY value | HISTORY INSERT n command | HISTORY DELETE n | "
                    "HISTORY MOVE n TO m | HISTORY SUPPRESS n | HISTORY UNSUPPRESS n | HISTORY ROLLBACK n | HISTORY ROLLFORWARD | HISTORY CLEAR | REGEN [FORCE]");
}

} // namespace sicdm

// REVOLVE, SWEEP, LOFT, PATTERN, MIRROR, FILLET, MASSPROPS, MEASURE.

#include "core/FeatureCommand.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Extrude.hpp"
#include "core/Features.hpp"
#include "core/Sculpt.hpp"
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
    const PlaneFrame f = planeFrame(c.plane);
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

} // namespace

CommandOutput runFeatureCommand(Document& doc, const std::string& cmd, const std::vector<std::string>& a, const std::vector<bool>& quoted) {
    CommandOutput out;
    auto quotedAt = [&](std::size_t k) { return k < quoted.size() && quoted[k]; };

    if (cmd == "REVOLVE") {
        // REVOLVE [curve] [ANGLE deg] [AXIS U|V|"line"|x,y x,y] [KEEP]
        auto isKey = [](const std::string& w) { return w == "angle" || w == "axis" || w == "keep"; };
        std::size_t i = 0;
        const std::string name = leadingName(a, quoted, i, isKey);
        const int ci = name.empty() ? doc.selectedCurve : curveNamed(doc, name);
        if (ci < 0 || ci >= static_cast<int>(doc.scene.sketch().curves().size()))
            return out.fail(name.empty() ? "Select a closed sketch curve to revolve, or name it" : "No sketch curve named " + name);
        float angle = 360.0f;
        bool keep = false;
        std::optional<std::pair<Vec2, Vec2>> axis; // point, direction
        std::string axisName = "the plane's vertical axis";
        const SketchCurve curve = doc.scene.sketch().curves()[static_cast<std::size_t>(ci)];
        for (; i < a.size(); ++i) {
            const std::string k = lower(a[i]);
            if (k == "keep") keep = true;
            else if (k == "angle") {
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
                    if (l.kind != CurveKind::Line || l.plane != curve.plane) return out.fail("The axis must be a sketch line on the profile's plane");
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
            } else return out.fail("Usage: REVOLVE [curve] [ANGLE deg] [AXIS U|V|\"line\"|x,y x,y] [KEEP]");
        }
        std::string why;
        const auto profile = profileFromCurve(curve, &why);
        if (!profile) return out.fail("Cannot revolve " + curve.name + ": " + why);
        std::optional<MeshData> mesh;
        if (axis) mesh = revolveProfile(*profile, curve.plane, axis->first, axis->second, angle, &why);
        else { // the vertical axis, else the horizontal one, through the plane's origin
            mesh = revolveProfile(*profile, curve.plane, {0, 0}, {0, 1}, angle, &why);
            if (!mesh) {
                mesh = revolveProfile(*profile, curve.plane, {0, 0}, {1, 0}, angle, &why);
                axisName = "the plane's horizontal axis";
            }
        }
        if (!mesh) return out.fail("Cannot revolve " + curve.name + ": " + why);
        const int serial = ++doc.primitiveSerial;
        Body b = featureBody("Revolve " + std::to_string(serial), std::move(*mesh), bodyColor(serial));
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
        const auto profile = profileFromCurve(profileCurve, &why);
        if (!profile) return out.fail("Cannot sweep " + profileCurve.name + ": " + why);
        std::vector<Vec3> path;
        bool closed = false;
        if (!curvePath(pathCurve, path, closed)) return out.fail(pathCurve.name + " can't be a path (a line, polyline, rectangle or circle)");
        auto mesh = sweepProfile(*profile, profileCurve.plane, path, closed, &why);
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
            const auto profile = profileFromCurve(c, &why);
            if (!profile) return out.fail("Section " + c.name + ": " + why);
            const PlaneFrame f = planeFrame(c.plane);
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
                        MeshData m = world;
                        m.transform(translate(dir * (spacing * static_cast<float>(k)) + dir2 * (spacing2 * static_cast<float>(r))));
                        copies.push_back(std::move(m));
                    }
                what = std::to_string(n) + (rows > 1 ? " x " + std::to_string(rows) : std::string()) + " linear";
            } else {
                if (!atGiven) at = {}; // about the axis through the origin
                const bool full = std::fabs(angle) >= 359.999f;
                const float step = angle / static_cast<float>(full ? n : n - 1);
                for (int k = 1; k < n; ++k) {
                    MeshData m = world;
                    const float t = step * static_cast<float>(k) * 3.14159265f / 180.0f;
                    m.transform(translate(at) * rotateAxis(axis, t) * translate(-at));
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
            doc.scene.add(std::move(b));
        }
        doc.selectedBody = static_cast<int>(doc.scene.bodies().size()) - 1;
        doc.dirty = true;
        out.say("Made " + std::to_string(copies.size()) + (copies.size() == 1 ? " copy" : " copies") + " of " + src.name + " (" + what +
                (cmd == "PATTERN" ? " pattern)" : ")"));
        return out;
    }

    if (cmd == "FILLET") {
        // FILLET [body] RADIUS r [CONVEX|CONCAVE|BOTH] [VOXEL v]
        auto isKey = [](const std::string& w) { return w == "radius" || w == "r" || w == "convex" || w == "concave" || w == "both" || w == "voxel"; };
        std::size_t i = 0;
        const std::string name = leadingName(a, quoted, i, isKey);
        const int bi = name.empty() ? doc.selectedBody : bodyNamed(doc, name);
        if (bi < 0 || bi >= static_cast<int>(doc.scene.bodies().size()))
            return out.fail(name.empty() ? "Select a body to round, or name it" : "No body named " + name);
        RoundOptions o;
        bool radiusGiven = false;
        for (; i < a.size(); ++i) {
            const std::string k = lower(a[i]);
            if (k == "radius" || k == "r" || k == "voxel") {
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
            } else return out.fail("Usage: FILLET [body] RADIUS r [CONVEX|CONCAVE|BOTH] [VOXEL v]");
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

namespace {

// The arguments from `from` on as a command line again (quoted ones re-quoted).
std::string rejoin(const std::vector<std::string>& a, const std::vector<bool>& quoted, std::size_t from) {
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
bool regenerate(Document& doc, const std::vector<HistoryStep>& steps, CommandOutput& out) {
    const Scene saved = doc.scene;
    const ProjectState savedState = doc.state;
    const int sb = doc.selectedBody, sc = doc.selectedCurve, si = doc.selectedImage, serial = doc.primitiveSerial;
    const bool mm = doc.apiMillimeters;
    doc.scene.clear();
    doc.clearSelection();
    doc.primitiveSerial = 0;
    doc.state.plane = SketchPlane::Top;
    doc.replaying = true;
    for (std::size_t k = 0; k < steps.size(); ++k) {
        doc.apiMillimeters = steps[k].millimetres;
        doc.state.units.unit = steps[k].unit;
        // The selection the step ran with.
        doc.selectedBody = steps[k].selectedBody.empty() ? -1 : bodyNamed(doc, steps[k].selectedBody);
        doc.selectedCurve = steps[k].selectedCurve.empty() ? -1 : curveNamed(doc, steps[k].selectedCurve);
        const CommandOutput r = executeCommand(doc, steps[k].line, nullptr);
        if (!r.ok) {
            doc.replaying = false;
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
    doc.apiMillimeters = mm;
    doc.state.units = savedState.units;
    doc.state.history = steps;
    doc.state.historyRedo.clear();
    doc.state.historyComplete = true;
    doc.state.historyRevision = doc.scene.revision();
    doc.dirty = true;
    for (CamOperation& op : doc.state.cam.ops) op.generated = false; // the part was rebuilt
    return true;
}

} // namespace

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
        if (!regenerate(doc, steps, out)) return out;
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
            const std::string line = rejoin(a, quoted, 2);
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
        if (!regenerate(doc, steps, out)) return out;
        out.say((sub == "delete" ? "Deleted step " : sub == "edit" ? "Changed step " : "Inserted step ") + std::to_string(k + 1) + " and regenerated " +
                std::to_string(steps.size()) + " steps");
        return out;
    }
    return out.fail("Usage: HISTORY [LIST] | HISTORY EDIT n command | HISTORY INSERT n command | HISTORY DELETE n | HISTORY CLEAR | REGEN [FORCE]");
}

} // namespace sicdm

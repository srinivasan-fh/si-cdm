#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Extrude.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <filesystem>

namespace sicdm {

namespace fs = std::filesystem;

namespace {

const Vec3 kPalette[] = {{0.72f, 0.75f, 0.80f}, {0.82f, 0.62f, 0.32f}, {0.45f, 0.60f, 0.78f},
                         {0.38f, 0.70f, 0.52f}, {0.78f, 0.45f, 0.45f}, {0.65f, 0.55f, 0.80f}};

std::string fmt(const char* f, double a, double b = 0, double c = 0) {
    char buf[128];
    std::snprintf(buf, sizeof(buf), f, a, b, c);
    return buf;
}

std::string lower(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

bool iequal(const std::string& a, const std::string& b) { return lower(a) == lower(b); }

std::string titleCase(const std::string& kind) {
    std::string s = kind;
    if (!s.empty()) s[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(s[0])));
    return s;
}

std::optional<bool> onOff(const std::vector<std::string>& args) {
    if (args.empty()) return std::nullopt;
    const std::string a = lower(args[0]);
    if (a == "on" || a == "1" || a == "true" || a == "show") return true;
    if (a == "off" || a == "0" || a == "false" || a == "hide") return false;
    return std::nullopt;
}

std::optional<StandardView> viewFromWord(const std::string& w) {
    const std::string s = lower(w);
    if (s == "top") return StandardView::Top;
    if (s == "bottom") return StandardView::Bottom;
    if (s == "front") return StandardView::Front;
    if (s == "back") return StandardView::Back;
    if (s == "left") return StandardView::Left;
    if (s == "right") return StandardView::Right;
    if (s == "iso" || s == "isometric") return StandardView::Isometric;
    return std::nullopt;
}

// Splits primitive args into numbers and an optional "@x,y,z" / "x,y,z" position.
bool splitPrimitiveArgs(const std::vector<std::string>& args, std::vector<float>& nums, std::optional<Vec3>& at) {
    for (const std::string& a : args) {
        if (a.find(',') != std::string::npos) {
            at = parseVec3(a);
            if (!at) return false;
        } else if (auto n = parseNumber(a)) {
            nums.push_back(*n);
        } else {
            return false;
        }
    }
    return true;
}

void markChanged(Document& doc) { doc.dirty = true; }

int findBody(const Document& d, const std::string& name) {
    for (std::size_t i = 0; i < d.scene.bodies().size(); ++i)
        if (iequal(d.scene.bodies()[i].name, name)) return static_cast<int>(i);
    return -1;
}
int findCurve(const Document& d, const std::string& name) {
    for (std::size_t i = 0; i < d.scene.sketch().curves().size(); ++i)
        if (iequal(d.scene.sketch().curves()[i].name, name)) return static_cast<int>(i);
    return -1;
}
int findImage(const Document& d, const std::string& name) {
    for (std::size_t i = 0; i < d.scene.images().size(); ++i)
        if (iequal(d.scene.images()[i].name, name)) return static_cast<int>(i);
    return -1;
}

std::string joinArgs(const std::vector<std::string>& args, std::size_t from = 0) {
    std::string s;
    for (std::size_t i = from; i < args.size(); ++i) s += (s.empty() ? "" : " ") + args[i];
    return s;
}

// Parses a chain of point inputs; relative points chain from the previous one.
bool parsePoints(const std::vector<std::string>& args, std::vector<Vec2>& out, std::string& bad) {
    Vec2 last{};
    for (const std::string& a : args) {
        const auto p = parsePointInput(a);
        if (!p) {
            bad = a;
            return false;
        }
        last = p->resolve(last);
        out.push_back(last);
    }
    return true;
}

CommandOutput addCurve(Document& doc, SketchCurve c) {
    CommandOutput out;
    c.plane = doc.state.plane;
    const SketchCurve& added = doc.scene.sketch().add(std::move(c));
    doc.selectedCurve = static_cast<int>(doc.scene.sketch().curves().size()) - 1;
    doc.selectedBody = doc.selectedImage = -1;
    markChanged(doc);
    out.say("Added " + added.name + " on " + planeName(added.plane) + "  " + added.measure());
    return out;
}

std::optional<Vec3> axisFromWord(const std::string& w) {
    const std::string s = lower(w);
    if (s == "x" || s == "+x") return Vec3{1, 0, 0};
    if (s == "y" || s == "+y") return Vec3{0, 1, 0};
    if (s == "z" || s == "+z") return Vec3{0, 0, 1};
    if (s == "-x") return Vec3{-1, 0, 0};
    if (s == "-y") return Vec3{0, -1, 0};
    if (s == "-z") return Vec3{0, 0, -1};
    const auto v = parseVec3(w);
    if (v && length(*v) > 1e-6f) return v;
    return std::nullopt;
}

int findMotion(const Document& d, const std::string& name) {
    for (std::size_t i = 0; i < d.scene.motions().size(); ++i)
        if (iequal(d.scene.motions()[i].name, name)) return static_cast<int>(i);
    return -1;
}

std::string vecText(const Vec3& v) { return fmt("(%.1f, %.1f, %.1f)", v.x, v.y, v.z); }

// Poses of the moving bodies and the motion values at time t.
std::vector<std::string> describePoses(const Document& doc, float t) {
    std::vector<std::string> lines;
    lines.push_back(fmt("t = %.3f s", t));
    for (const Motion& m : doc.scene.motions()) {
        if (!m.enabled) continue;
        lines.push_back("  " + m.name + "  " + fmt("%.2f ", motionValue(m, t)) + m.unit() + "  " +
                        fmt("%.2f ", motionRate(m, t)) + m.rateUnit());
    }
    const std::vector<Mat4> tf = motionTransforms(doc.scene, t);
    for (std::size_t i = 0; i < doc.scene.bodies().size(); ++i) {
        if (!isMoving(doc.scene, i)) continue;
        const Body& b = doc.scene.bodies()[i];
        const Vec3 c = tf[i].transformPoint(b.bounds().center());
        const Vec3 x = tf[i].transformDir({1, 0, 0});
        const float heading = degrees(std::atan2(x.y, x.x));
        lines.push_back("  " + b.name + "  centre " + vecText(c) + " mm" + fmt("  x-axis heading %.1f deg", heading));
    }
    return lines;
}

CommandOutput needView(ViewHost* view, const char* what) {
    CommandOutput out;
    if (!view) out.fail(std::string(what) + " needs the app window (not available headless)");
    return out;
}

} // namespace

std::string Document::displayName() const { return path.empty() ? name : fs::path(path).filename().string(); }

std::string Document::title() const { return dirty ? displayName() + " *" : displayName(); }

Body defaultPrimitive(const std::string& kind, int serial) {
    Body b;
    if (kind == "box") b = makeBody("", kind, {-20, -20, 0, 20, 20, 40});
    else if (kind == "wedge") b = makeBody("", kind, {-20, -20, 0, 20, 20, 30});
    else if (kind == "pyramid") b = makeBody("", kind, {25, 35, 4});
    else if (kind == "prism") b = makeBody("", kind, {20, 40, 6});
    else if (kind == "cylinder") b = makeBody("", kind, {15, 40});
    else if (kind == "cone") b = makeBody("", kind, {20, 0, 40});
    else if (kind == "sphere") b = makeBody("", kind, {20});
    else if (kind == "torus") b = makeBody("", kind, {25, 7});
    b.name = (kind == "box" ? std::string("Block") : titleCase(kind)) + " " + std::to_string(serial);
    b.color = kPalette[static_cast<std::size_t>(serial) % std::size(kPalette)];
    return b;
}

std::vector<std::string> describeDocument(const Document& doc) {
    std::vector<std::string> l;
    l.push_back("Project: " + doc.title() + "   sketch plane: " + planeName(doc.state.plane));
    l.push_back("Bodies (" + std::to_string(doc.scene.bodies().size()) + "):");
    for (const Body& b : doc.scene.bodies()) {
        const AABB bb = b.bounds();
        std::string params;
        for (float p : b.params) params += fmt(params.empty() ? "%g" : " %g", p);
        char buf[256];
        std::snprintf(buf, sizeof(buf), "  %s [%s%s%s]  at (%.1f, %.1f, %.1f)  size %.1f x %.1f x %.1f%s", b.name.c_str(),
                      b.kind.c_str(), params.empty() ? "" : " ", params.c_str(), static_cast<double>(b.offset.x),
                      static_cast<double>(b.offset.y), static_cast<double>(b.offset.z),
                      static_cast<double>(bb.max.x - bb.min.x), static_cast<double>(bb.max.y - bb.min.y),
                      static_cast<double>(bb.max.z - bb.min.z), b.visible ? "" : "  (hidden)");
        l.push_back(buf);
    }
    l.push_back("Sketch curves (" + std::to_string(doc.scene.sketch().curves().size()) + "):");
    for (const SketchCurve& c : doc.scene.sketch().curves())
        l.push_back("  " + c.name + "  " + planeName(c.plane) + "  " + c.measure() + (c.visible ? "" : "  (hidden)"));
    l.push_back("Images (" + std::to_string(doc.scene.images().size()) + "):");
    for (const ImageRef& i : doc.scene.images()) {
        char buf[256];
        std::snprintf(buf, sizeof(buf), "  %s  %s  centre (%.1f, %.1f)  %.1f x %.1f mm  opacity %.2f%s", i.name.c_str(),
                      planeName(i.plane), static_cast<double>(i.center.x), static_cast<double>(i.center.y),
                      static_cast<double>(i.width), static_cast<double>(i.height()), static_cast<double>(i.opacity),
                      i.pixels.empty() ? "  (file missing)" : "");
        l.push_back(buf);
    }
    return l;
}

CommandOutput executeCommand(Document& doc, std::string_view line, ViewHost* view) {
    CommandOutput out;
    const ParsedCommand pc = tokenizeCommand(line);
    if (pc.name.empty()) return out;
    const CommandSpec* spec = findCommand(pc.name);
    if (!spec) return out.fail("Unknown command: " + pc.name + "   (type HELP for the list)");
    const std::string cmd = spec->name;
    const auto& a = pc.args;
    auto usage = [&] { return out.fail(std::string("Usage: ") + spec->usage); };

    // ---- Sketch -----------------------------------------------------------
    if (cmd == "LINE" || cmd == "PLINE" || cmd == "RECT" || cmd == "POINT") {
        std::vector<std::string> pointArgs = a;
        bool close = false;
        if (cmd == "PLINE" && !pointArgs.empty() && iequal(pointArgs.back(), "close")) {
            close = true;
            pointArgs.pop_back();
        }
        std::vector<Vec2> pts;
        std::string bad;
        if (!parsePoints(pointArgs, pts, bad)) return out.fail("Not a point: " + bad + "   (x,y  @dx,dy  @len<deg)");
        SketchCurve c;
        if (cmd == "POINT") {
            if (pts.size() != 1) return usage();
            c.kind = CurveKind::Point;
            c.pts = pts;
            return addCurve(doc, c);
        }
        if (cmd == "RECT") {
            if (pts.size() != 2 || pts[0].x == pts[1].x || pts[0].y == pts[1].y) return usage();
            c.kind = CurveKind::Rectangle;
            c.pts = pts;
            return addCurve(doc, c);
        }
        if (pts.size() < 2) return usage();
        if (cmd == "PLINE") {
            if (close) pts.push_back(pts.front());
            c.kind = CurveKind::Polyline;
            c.pts = pts;
            return addCurve(doc, c);
        }
        for (std::size_t i = 1; i < pts.size(); ++i) { // LINE p1 p2 p3 ... = connected segments
            SketchCurve seg;
            seg.kind = CurveKind::Line;
            seg.pts = {pts[i - 1], pts[i]};
            for (auto& l : addCurve(doc, seg).lines) out.say(l);
        }
        return out;
    }
    if (cmd == "CIRCLE") {
        if (a.size() != 2) return usage();
        const auto c = parsePointInput(a[0]);
        if (!c) return usage();
        const Vec2 centre = c->resolve({});
        float r = 0.0f;
        if (const auto n = parseNumber(a[1])) r = *n;
        else if (const auto p = parsePointInput(a[1])) {
            const Vec2 q = p->resolve(centre);
            r = std::hypot(q.x - centre.x, q.y - centre.y);
        }
        if (!(r > 0.0f)) return out.fail("Radius must be positive");
        SketchCurve curve;
        curve.kind = CurveKind::Circle;
        curve.pts = {centre};
        curve.radius = r;
        return addCurve(doc, curve);
    }

    // ---- Solids -----------------------------------------------------------
    static const std::pair<const char*, const char*> kSolids[] = {
        {"BOX", "box"},     {"CYLINDER", "cylinder"}, {"CONE", "cone"},       {"SPHERE", "sphere"},
        {"TORUS", "torus"}, {"WEDGE", "wedge"},       {"PYRAMID", "pyramid"}, {"PRISM", "prism"}};
    for (const auto& [name, kind] : kSolids) {
        if (cmd != name) continue;
        std::vector<float> n;
        std::optional<Vec3> at;
        if (!splitPrimitiveArgs(a, n, at)) return usage();
        std::vector<float> params;
        const std::string k = kind;
        if ((k == "box" || k == "wedge") && n.size() == 3)
            params = {-n[0] * 0.5f, -n[1] * 0.5f, 0, n[0] * 0.5f, n[1] * 0.5f, n[2]};
        else if (k == "cylinder" && n.size() == 2) params = n;
        else if (k == "cone" && (n.size() == 2 || n.size() == 3)) params = {n[0], n.size() == 3 ? n[2] : 0.0f, n[1]};
        else if (k == "sphere" && n.size() == 1) params = n;
        else if (k == "torus" && n.size() == 2) params = n;
        else if ((k == "pyramid" || k == "prism") && (n.size() == 2 || n.size() == 3)) {
            params = n;
            if (params.size() == 2) params.push_back(k == "prism" ? 6.0f : 4.0f);
        } else
            return usage();
        Body b = defaultPrimitive(k, ++doc.primitiveSerial);
        b.mesh = buildPrimitive(k, params);
        if (b.mesh.indices.empty()) return out.fail("Invalid dimensions for " + std::string(name));
        b.params = params;
        b.offset = at.value_or(Vec3{});
        const Vec3 o = b.offset;
        doc.scene.add(std::move(b));
        doc.selectedBody = static_cast<int>(doc.scene.bodies().size()) - 1;
        doc.selectedCurve = doc.selectedImage = -1;
        markChanged(doc);
        out.say("Created " + doc.scene.bodies().back().name + fmt(" at (%.1f, %.1f, %.1f)", o.x, o.y, o.z));
        return out;
    }

    // ---- Features ---------------------------------------------------------
    if (cmd == "EXTRUDE") {
        bool symmetric = false, keep = false;
        std::optional<float> distance;
        std::vector<std::string> nameWords;
        for (const std::string& w : a) {
            const std::string l = lower(w);
            if (l == "symmetric" || l == "sym" || l == "both") symmetric = true;
            else if (l == "keep") keep = true;
            else nameWords.push_back(w);
        }
        // The last number is the distance; earlier words (numbers too) name the curve, e.g. "Circle 2 20".
        if (!nameWords.empty()) {
            distance = parseNumber(nameWords.back());
            if (distance) nameWords.pop_back();
        }
        if (!distance) return usage();
        if (*distance == 0.0f) return out.fail("Distance must not be zero");
        int index = doc.selectedCurve;
        if (!nameWords.empty()) {
            index = findCurve(doc, joinArgs(nameWords));
            if (index < 0) return out.fail("No sketch curve named " + joinArgs(nameWords));
        }
        auto& curves = doc.scene.sketch().curves();
        if (index < 0 || index >= static_cast<int>(curves.size()))
            return out.fail("Select a closed sketch curve (rectangle, circle or closed polyline) or give its name");
        SketchCurve& c = curves[static_cast<std::size_t>(index)];
        std::string why;
        const auto profile = profileFromCurve(c, &why);
        if (!profile) return out.fail("Cannot extrude: " + why);
        Body b = defaultPrimitive("box", ++doc.primitiveSerial); // name / colour series
        b.name = "Extrude " + std::to_string(doc.primitiveSerial);
        b.kind = "extrude";
        b.params = extrudeParams(*profile, c.plane, *distance, symmetric);
        b.mesh = buildExtrusion(b.params);
        if (b.mesh.indices.empty()) return out.fail("Extrusion failed for " + c.name);
        b.offset = {};
        const std::string profileName = c.name;
        if (!keep) { // the sketch is consumed, as in most CAD tools (show it again in the Model panel)
            c.visible = false;
            doc.scene.sketch().touch();
        }
        doc.scene.add(std::move(b));
        doc.selectedBody = static_cast<int>(doc.scene.bodies().size()) - 1;
        doc.selectedCurve = doc.selectedImage = -1;
        markChanged(doc);
        out.say("Extruded " + profileName + fmt(symmetric ? " %.2f mm symmetric" : " %.2f mm", *distance) + " -> " +
                doc.scene.bodies().back().name);
        return out;
    }

    // ---- Files ------------------------------------------------------------
    if (cmd == "NEW") {
        if (view) view->prepareDocument("");
        doc.scene.clear();
        doc.path.clear();
        doc.dirty = false;
        doc.clearSelection();
        doc.primitiveSerial = 0;
        out.documentReplaced = true;
        out.say("New empty project");
        return out;
    }
    if (cmd == "OPEN") {
        if (a.empty()) return usage();
        const std::string path = joinArgs(a);
        std::string err, warn;
        Scene loaded;
        ProjectState st;
        if (!loadProject(loaded, st, path, &err, &warn)) return out.fail(err);
        if (view && view->prepareDocument(fs::absolute(path).string())) {
            out.say(fs::path(path).filename().string() + " is already open");
            return out;
        }
        doc.scene.assign(std::move(loaded));
        doc.state = st;
        doc.path = fs::absolute(path).string();
        doc.dirty = false;
        doc.clearSelection();
        doc.primitiveSerial = static_cast<int>(doc.scene.bodies().size());
        out.documentReplaced = true;
        out.say("Opened " + doc.path);
        if (!warn.empty()) out.say(warn);
        return out;
    }
    // ---- Motion & simulation ----------------------------------------------
    if (cmd == "MOTION") {
        auto& motions = doc.scene.motions();
        if (a.empty() || iequal(a[0], "list")) {
            if (motions.empty()) out.say("No motions. Example: MOTION CIRCLE \"Spin\" AXIS Z SPEED 90 BODIES ALL");
            for (const Motion& m : motions) out.say(describeMotion(m));
            return out;
        }
        const std::string sub = lower(a[0]);
        if (sub == "delete" || sub == "erase" || sub == "on" || sub == "off" || sub == "clear") {
            if (sub == "clear") {
                motions.clear();
                markChanged(doc);
                out.say("All motions removed");
                return out;
            }
            if (a.size() < 2) return usage();
            const int i = findMotion(doc, joinArgs(a, 1));
            if (i < 0) return out.fail("No motion named " + joinArgs(a, 1));
            Motion& m = motions[static_cast<std::size_t>(i)];
            const std::string name = m.name;
            if (sub == "on" || sub == "off") m.enabled = sub == "on";
            else motions.erase(motions.begin() + i);
            markChanged(doc);
            out.say((sub == "delete" || sub == "erase" ? "Deleted " : sub == "on" ? "Enabled " : "Disabled ") + name);
            return out;
        }
        const auto kind = motionKindFromKey(a[0]);
        if (!kind) return usage();

        // Options; a motion that already has the name is updated, keeping what is not given.
        std::optional<std::string> name;
        std::optional<Vec3> axis, origin;
        std::optional<float> speed, range, start;
        std::optional<MotionMode> mode;
        std::optional<std::vector<std::string>> targets;
        std::optional<bool> each; // EACH: every body turns about its own centre
        for (std::size_t i = 1; i < a.size(); ++i) {
            const std::string k = lower(a[i]);
            const bool hasNext = i + 1 < a.size();
            auto num = [&](std::optional<float>& dst) {
                const auto v = hasNext ? parseNumber(a[i + 1]) : std::nullopt;
                if (v) dst = *v, ++i;
                return v.has_value();
            };
            if (k == "axis" || k == "dir" || k == "direction") {
                if (!hasNext || !(axis = axisFromWord(a[i + 1]))) return out.fail("AXIS needs X, Y, Z, -X... or x,y,z");
                ++i;
            } else if (k == "at" || k == "origin" || k == "center" || k == "centre") {
                if (!hasNext || !(origin = parseVec3(a[i + 1]))) return out.fail("AT needs x,y,z");
                ++i;
            } else if (k == "speed" || k == "rate" || k == "rpm") {
                if (!num(speed)) return out.fail("SPEED needs a number");
                if (k == "rpm") *speed *= 6.0f; // rev/min -> deg/s
            } else if (k == "range" || k == "stroke" || k == "sweep" || k == "travel") {
                if (!num(range) || *range < 0.0f) return out.fail("RANGE needs a positive number");
            } else if (k == "start" || k == "delay") {
                if (!num(start) || *start < 0.0f) return out.fail("START needs seconds >= 0");
            } else if (k == "mode") {
                if (!hasNext || !(mode = motionModeFromKey(a[i + 1]))) return out.fail("MODE: CONT, LOOP, OSC or ONCE");
                ++i;
            } else if (k == "each" || k == "own") {
                each = true;
            } else if (k == "shared" || k == "common") {
                each = false;
            } else if (motionModeFromKey(k)) {
                mode = motionModeFromKey(k);
            } else if (k == "bodies" || k == "body" || k == "on") {
                targets = std::vector<std::string>(a.begin() + static_cast<std::ptrdiff_t>(i + 1), a.end());
                break;
            } else if (k == "name" && hasNext) {
                name = a[++i];
            } else if (!name) {
                name = a[i];
            } else {
                return out.fail("Unexpected '" + a[i] + "'.  " + spec->usage);
            }
        }

        Motion m;
        const int existing = name ? findMotion(doc, *name) : -1;
        if (existing >= 0) m = motions[static_cast<std::size_t>(existing)];
        else {
            m.speed = *kind == MotionKind::Circular ? 90.0f : 20.0f;
            m.axis = *kind == MotionKind::Circular ? Vec3{0, 0, 1} : Vec3{1, 0, 0};
            int n = 1;
            const char* base = *kind == MotionKind::Circular ? "Circular " : "Linear ";
            while (findMotion(doc, base + std::to_string(n)) >= 0) ++n;
            m.name = name.value_or(base + std::to_string(n));
        }
        m.kind = *kind;
        if (targets) {
            m.bodies.clear();
            for (const std::string& t : *targets) {
                if (iequal(t, "all") || t == "*") {
                    m.bodies = {"*"};
                    break;
                }
                const int b = findBody(doc, t);
                if (b < 0) return out.fail("No body named " + t + " (quote names with spaces)");
                m.bodies.push_back(doc.scene.bodies()[static_cast<std::size_t>(b)].name);
            }
        }
        if (m.bodies.empty()) {
            const int sel = doc.selectedBody;
            if (sel < 0 || sel >= static_cast<int>(doc.scene.bodies().size()))
                return out.fail("Select a body first, or add BODIES name... | ALL");
            m.bodies = {doc.scene.bodies()[static_cast<std::size_t>(sel)].name};
        }
        if (axis) m.axis = normalize(*axis);
        if (speed) m.speed = *speed;
        if (range) m.range = *range;
        if (mode) m.mode = *mode;
        if (range && *range > 0.0f && !mode && m.mode == MotionMode::Continuous) m.mode = MotionMode::Oscillate;
        if (start) m.start = *start;
        if (each) m.eachCentre = *each;
        if (origin) {
            m.origin = *origin;
            if (!each) m.eachCentre = false;
        }
        else if (existing < 0 && m.kind == MotionKind::Circular) {
            // Default axis position: centre of the moved bodies (a wheel spins about its hub).
            AABB box;
            for (const Body& b : doc.scene.bodies())
                if (m.moves(b.name)) box.expand(b.bounds());
            if (box.valid()) m.origin = box.center();
        }
        if (existing >= 0) motions[static_cast<std::size_t>(existing)] = m;
        else motions.push_back(m);
        markChanged(doc);
        out.say((existing >= 0 ? "Updated " : "Added ") + describeMotion(m));
        return out;
    }
    if (cmd == "SIM") {
        const std::string sub = a.empty() ? "open" : lower(a[0]);
        auto value = [&](float fallback) -> std::optional<float> {
            if (a.size() < 2) return fallback;
            if (const auto on = onOff({a[1]})) return *on ? 1.0f : 0.0f;
            return parseNumber(a[1]);
        };
        if (sub == "at" || sub == "poses" || sub == "pose") {
            const auto t = value(0.0f);
            if (!t || *t < 0.0f) return out.fail("Usage: SIM AT seconds");
            for (std::string& l : describePoses(doc, *t)) out.say(std::move(l));
            return out;
        }
        if (sub == "duration" || sub == "length") {
            const auto d = value(0.0f);
            if (!d || *d < 0.0f) return out.fail("Usage: SIM DURATION seconds (0 = automatic)");
            doc.state.simDuration = *d;
            markChanged(doc);
            out.say(*d > 0.0f ? fmt("Simulation length %.2f s", *d)
                              : fmt("Simulation length automatic (%.2f s)", suggestedDuration(doc.scene)));
            return out;
        }
        static const char* kActions[] = {"open", "play", "pause", "toggle", "stop", "reset", "time", "seek",
                                         "speed", "axes", "paths"};
        if (std::find(std::begin(kActions), std::end(kActions), sub) == std::end(kActions)) return usage();
        const bool needsValue = sub == "time" || sub == "seek" || sub == "speed";
        const auto v = value(sub == "axes" || sub == "paths" ? -1.0f : 0.0f); // -1 toggles
        if (!v || (needsValue && a.size() < 2)) return usage();
        if (auto o = needView(view, "SIM"); !o.ok) return o;
        std::string reply;
        if (!view->simulation(sub == "seek" ? "time" : sub == "reset" ? "stop" : sub, *v, &reply))
            return out.fail(reply.empty() ? "Simulation not available" : reply);
        out.say(reply);
        return out;
    }

    if (cmd == "FILES") {
        if (a.empty()) {
            const auto tabs = view ? view->fileTabs() : std::vector<std::string>{};
            if (tabs.empty()) out.say("> 1  " + doc.title());
            for (const std::string& t : tabs) out.say(t);
            return out;
        }
        if (!view) return out.fail("Only one document is open");
        std::string err;
        if (!view->switchFile(joinArgs(a), &err)) return out.fail(err);
        out.say("Switched to " + doc.title()); // `doc` now holds the activated file
        return out;
    }
    if (cmd == "CLOSE") {
        if (!view) return out.fail("CLOSE needs the app window");
        const bool discard = !a.empty() && toUpper(a[0]) == "DISCARD";
        if (!a.empty() && !discard) return usage();
        const std::string closed = doc.displayName();
        std::string err;
        if (!view->closeFile(discard, &err)) return out.fail(err);
        out.say("Closed " + closed + "; now editing " + doc.title());
        out.documentReplaced = false;
        return out;
    }
    if (cmd == "SAVE" || cmd == "SAVEAS") {
        std::string path = a.empty() ? doc.path : joinArgs(a);
        if (path.empty() || (cmd == "SAVEAS" && a.empty())) return out.fail("Usage: SAVEAS path.sicdm");
        if (fileKindOf(path) != FileKind::Project) path += ".sicdm";
        std::string err;
        if (!saveProject(doc.scene, doc.state, path, &err)) return out.fail(err);
        doc.path = fs::absolute(path).string();
        doc.dirty = false;
        out.say("Saved " + doc.path);
        return out;
    }
    if (cmd == "EXPORT") {
        if (a.empty()) return usage();
        const std::string path = joinArgs(a);
        std::string err;
        switch (fileKindOf(path)) {
        case FileKind::Stl:
            if (!exportStl(doc.scene, path, &err)) return out.fail(err);
            out.say("Exported visible bodies to " + path);
            return out;
        case FileKind::Obj:
            if (!exportObj(doc.scene, path, &err)) return out.fail(err);
            out.say("Exported visible bodies to " + path);
            return out;
        case FileKind::Dxf:
        case FileKind::Svg: {
            const int n = fileKindOf(path) == FileKind::Dxf ? exportDxf(doc.scene, doc.state.plane, path, &err)
                                                            : exportSvg(doc.scene, doc.state.plane, path, &err);
            if (n < 0) return out.fail(err);
            out.say("Exported " + std::to_string(n) + " curves on " + planeName(doc.state.plane) + " to " + path);
            return out;
        }
        default: return out.fail("Export supports .stl .obj .dxf .svg");
        }
    }
    if (cmd == "IMPORT") {
        if (a.empty()) return usage();
        std::vector<std::string> rest = a;
        std::optional<Vec2> at;
        std::optional<float> width;
        for (std::size_t i = 0; i < rest.size();) {
            if (iequal(rest[i], "width") && i + 1 < rest.size() && parseNumber(rest[i + 1])) {
                width = parseNumber(rest[i + 1]);
                rest.erase(rest.begin() + static_cast<std::ptrdiff_t>(i), rest.begin() + static_cast<std::ptrdiff_t>(i) + 2);
            } else if (rest[i].size() > 1 && rest[i][0] == '@' && parsePointInput(rest[i])) {
                at = parsePointInput(rest[i])->resolve({});
                rest.erase(rest.begin() + static_cast<std::ptrdiff_t>(i));
            } else {
                ++i;
            }
        }
        const std::string path = joinArgs(rest);
        std::string err;
        switch (fileKindOf(path)) {
        case FileKind::Stl:
        case FileKind::Obj: {
            auto b = importMesh(path, &err);
            if (!b) return out.fail(err);
            b->color = kPalette[static_cast<std::size_t>(++doc.primitiveSerial) % std::size(kPalette)];
            const std::size_t tris = b->mesh.indices.size() / 3;
            doc.scene.add(std::move(*b));
            doc.selectedBody = static_cast<int>(doc.scene.bodies().size()) - 1;
            doc.selectedCurve = doc.selectedImage = -1;
            markChanged(doc);
            out.say("Imported " + doc.scene.bodies().back().name + " (" + std::to_string(tris) + " triangles)");
            return out;
        }
        case FileKind::Dxf: {
            const int n = importDxf(doc.scene, doc.state.plane, path, &err);
            if (n <= 0) return out.fail(err.empty() ? "Nothing imported" : err);
            markChanged(doc);
            out.say("Imported " + std::to_string(n) + " curves onto " + planeName(doc.state.plane));
            return out;
        }
        case FileKind::Image: {
            auto img = importImage(path, doc.state.plane, at.value_or(Vec2{}), &err);
            if (!img) return out.fail(err);
            if (width && *width > 0) img->width = *width;
            const std::string name = img->name;
            const float w = img->width, h = img->height();
            doc.scene.addImage(std::move(*img));
            doc.selectedImage = static_cast<int>(doc.scene.images().size()) - 1;
            doc.selectedBody = doc.selectedCurve = -1;
            markChanged(doc);
            out.say("Imported image " + name + " on " + planeName(doc.state.plane) + fmt("  %.1f x %.1f mm", w, h));
            return out;
        }
        case FileKind::Project: return out.fail("Use OPEN for project files");
        default: return out.fail("Import supports .stl .obj .dxf and images (.png .jpg .bmp .tga .gif ...)");
        }
    }

    // ---- Editing ----------------------------------------------------------
    if (cmd == "SELECT") {
        if (a.empty()) return usage();
        const std::string name = joinArgs(a);
        doc.clearSelection();
        if (iequal(name, "none")) {
            out.say("Selection cleared");
            return out;
        }
        if ((doc.selectedBody = findBody(doc, name)) >= 0 || (doc.selectedCurve = findCurve(doc, name)) >= 0 ||
            (doc.selectedImage = findImage(doc, name)) >= 0) {
            out.say("Selected " + name);
            return out;
        }
        return out.fail("Nothing named " + name);
    }
    if (cmd == "ERASE") {
        if (!a.empty()) {
            const std::string name = joinArgs(a);
            doc.clearSelection();
            doc.selectedBody = findBody(doc, name);
            if (doc.selectedBody < 0) doc.selectedCurve = findCurve(doc, name);
            if (doc.selectedBody < 0 && doc.selectedCurve < 0) doc.selectedImage = findImage(doc, name);
        }
        auto& s = doc.scene;
        std::string gone;
        if (doc.selectedCurve >= 0 && doc.selectedCurve < static_cast<int>(s.sketch().curves().size())) {
            gone = s.sketch().curves()[static_cast<std::size_t>(doc.selectedCurve)].name;
            s.sketch().remove(static_cast<std::size_t>(doc.selectedCurve));
        } else if (doc.selectedBody >= 0 && doc.selectedBody < static_cast<int>(s.bodies().size())) {
            gone = s.bodies()[static_cast<std::size_t>(doc.selectedBody)].name;
            s.remove(static_cast<std::size_t>(doc.selectedBody));
        } else if (doc.selectedImage >= 0 && doc.selectedImage < static_cast<int>(s.images().size())) {
            gone = s.images()[static_cast<std::size_t>(doc.selectedImage)].name;
            s.removeImage(static_cast<std::size_t>(doc.selectedImage));
        } else {
            return out.fail(a.empty() ? "Nothing selected" : "Nothing named " + joinArgs(a));
        }
        doc.clearSelection();
        markChanged(doc);
        out.say("Deleted " + gone);
        return out;
    }
    if (cmd == "MOVE") {
        if (a.empty()) return usage();
        const auto d = parseVec3(a[0]);
        if (!d) return usage();
        if (a.size() > 1) {
            const std::string name = joinArgs(a, 1);
            doc.clearSelection();
            doc.selectedBody = findBody(doc, name);
            if (doc.selectedBody < 0) doc.selectedImage = findImage(doc, name);
        }
        auto& s = doc.scene;
        if (doc.selectedBody >= 0 && doc.selectedBody < static_cast<int>(s.bodies().size())) {
            Body& b = s.bodies()[static_cast<std::size_t>(doc.selectedBody)];
            b.offset += *d;
            markChanged(doc);
            out.say("Moved " + b.name + fmt(" to (%.1f, %.1f, %.1f)", b.offset.x, b.offset.y, b.offset.z));
            return out;
        }
        if (doc.selectedImage >= 0 && doc.selectedImage < static_cast<int>(s.images().size())) {
            ImageRef& img = s.images()[static_cast<std::size_t>(doc.selectedImage)];
            img.center = {img.center.x + d->x, img.center.y + d->y};
            markChanged(doc);
            out.say("Moved " + img.name + fmt(" to (%.1f, %.1f)", img.center.x, img.center.y));
            return out;
        }
        return out.fail("Select a body or image first (or give its name)");
    }
    if (cmd == "IMAGE") {
        auto& imgs = doc.scene.images();
        std::size_t i = 0;
        int target = doc.selectedImage;
        if (!a.empty() && findImage(doc, a[0]) >= 0) {
            target = findImage(doc, a[0]);
            i = 1;
        }
        if (target < 0 || target >= static_cast<int>(imgs.size()))
            return out.fail(imgs.empty() ? "No images: use IMPORT picture.png" : "Select an image or give its name");
        ImageRef& img = imgs[static_cast<std::size_t>(target)];
        bool changed = false;
        for (; i + 1 < a.size(); i += 2) {
            const std::string key = lower(a[i]);
            if (key == "width") {
                const auto w = parseNumber(a[i + 1]);
                if (!w || *w <= 0) return out.fail("WIDTH needs a positive number (mm)");
                img.width = *w;
            } else if (key == "opacity") {
                const auto o = parseNumber(a[i + 1]);
                if (!o) return out.fail("OPACITY needs a number between 0 and 1");
                img.opacity = std::clamp(*o, 0.0f, 1.0f);
            } else if (key == "at") {
                const auto p = parsePointInput(a[i + 1]);
                if (!p) return out.fail("AT needs x,y");
                img.center = p->resolve(img.center);
            } else {
                return usage();
            }
            changed = true;
        }
        if (changed) markChanged(doc);
        out.say(img.name + fmt("  %.1f x %.1f mm", img.width, img.height()) +
                fmt("  centre (%.1f, %.1f)  opacity %.2f", img.center.x, img.center.y, img.opacity));
        return out;
    }
    if (cmd == "LIST") {
        for (auto& l : describeDocument(doc)) out.say(l);
        if (view) out.say(view->describe());
        return out;
    }

    // ---- Views & windows --------------------------------------------------
    if (cmd == "PLANE") {
        const auto p = a.empty() ? std::nullopt : planeFromKey(a[0]);
        if (!p) return usage();
        doc.state.plane = *p;
        if (view) view->planeChanged(*p);
        out.say(std::string("Sketch plane: ") + planeName(*p));
        return out;
    }
    if (cmd == "2D") {
        if (!a.empty()) {
            const auto p = planeFromKey(a[0]);
            if (!p) return usage();
            doc.state.plane = *p;
        }
        if (auto o = needView(view, "2D"); !o.ok) return o;
        view->planeChanged(doc.state.plane);
        view->openWindow(true);
        out.say(std::string("2D Drawing window: ") + planeName(doc.state.plane));
        return out;
    }
    if (cmd == "3D") {
        if (auto o = needView(view, "3D"); !o.ok) return o;
        view->openWindow(false);
        out.say("3D Model window");
        return out;
    }
    if (cmd == "VIEW") {
        const auto v = a.empty() ? std::nullopt : viewFromWord(a[0]);
        if (!v) return usage();
        if (auto o = needView(view, "VIEW"); !o.ok) return o;
        view->standardView(*v);
        out.say("View " + lower(a[0]));
        return out;
    }
    if (cmd == "ZOOM" || cmd == "FIT") {
        if (auto o = needView(view, "ZOOM"); !o.ok) return o;
        const std::string mode = a.empty() ? "e" : lower(a[0]);
        if (cmd == "FIT" || mode == "e" || mode == "extents" || mode == "fit") view->fit();
        else if (mode == "in") view->zoomStep(2.0f);
        else if (mode == "out") view->zoomStep(-2.0f);
        else return usage();
        out.say(cmd == "FIT" ? "Zoom extents" : "Zoom " + mode);
        return out;
    }
    if (cmd == "HOME") {
        if (auto o = needView(view, "HOME"); !o.ok) return o;
        view->home();
        out.say("Home view");
        return out;
    }
    if (cmd == "ROLL") {
        const auto deg = a.empty() ? std::nullopt : parseNumber(a[0]);
        if (!deg) return usage();
        if (auto o = needView(view, "ROLL"); !o.ok) return o;
        view->roll(*deg, *deg == 0.0f);
        out.say(*deg == 0.0f ? std::string("View levelled") : fmt("Rolled %.1f deg", *deg));
        return out;
    }
    if (cmd == "ORTHO" || cmd == "PERSP") {
        if (auto o = needView(view, cmd.c_str()); !o.ok) return o;
        view->setProjection(cmd == "ORTHO" ? Projection::Orthographic : Projection::Perspective);
        out.say(cmd == "ORTHO" ? "Orthographic" : "Perspective");
        return out;
    }
    if (cmd == "DISPLAY") {
        if (a.empty()) return usage();
        const std::string m = lower(a[0]);
        int mode = -1;
        if (m == "shaded") mode = 0;
        else if (m == "edges" || m == "shadededges") mode = 1;
        else if (m == "wire" || m == "wireframe") mode = 2;
        else if (m == "hidden" || m == "hiddenline") mode = 3;
        if (mode < 0) return usage();
        if (auto o = needView(view, "DISPLAY"); !o.ok) return o;
        view->setDisplay(mode);
        out.say("Display " + m);
        return out;
    }
    if (cmd == "GRID" || cmd == "PANEL") {
        if (auto o = needView(view, cmd.c_str()); !o.ok) return o;
        const auto on = onOff(a);
        if (cmd == "GRID") view->setGrid(on);
        else view->setPanel(on);
        out.say(cmd + (on ? (*on ? " on" : " off") : " toggled"));
        return out;
    }
    if (cmd == "SNAP") {
        const auto on = onOff(a);
        const bool v = on.value_or(!doc.snapGrid);
        doc.snapGrid = doc.snapPoints = v;
        out.say(std::string("Snap ") + (v ? "on" : "off"));
        return out;
    }

    // ---- Session ----------------------------------------------------------
    if (cmd == "HELP") {
        if (!a.empty()) {
            const CommandSpec* s = findCommand(a[0]);
            if (!s) return out.fail("Unknown command: " + a[0]);
            out.say(std::string(s->usage) + "  -  " + s->help);
            if (s->aliases[0]) out.say(std::string("Aliases: ") + s->aliases);
            return out;
        }
        for (const CommandSpec& s : commandTable()) {
            char buf[256];
            std::snprintf(buf, sizeof(buf), "%-46s %s", s.usage, s.help);
            out.say(buf);
        }
        out.say("Points: x,y  @dx,dy (relative)  @len<deg (polar).  Paths with spaces in \"quotes\".");
        return out;
    }
    if (cmd == "CLEAR") {
        out.clearHistory = true;
        return out;
    }
    return out.fail("Command not implemented: " + cmd);
}

} // namespace sicdm

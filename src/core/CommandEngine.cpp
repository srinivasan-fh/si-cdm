#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Extrude.hpp"
#include "core/Library.hpp"
#include "core/Physics.hpp"
#include "core/Pose.hpp"
#include "core/CamCommand.hpp"
#include "core/Json.hpp"
#include "core/LatheCommand.hpp"
#include "core/CrashReport.hpp"
#include "core/Licence.hpp"
#include "core/Notices.hpp"
#include "core/Plugin.hpp"
#include "core/AssemblyCommand.hpp"
#include "core/SketchEdit.hpp"
#include "core/DrawingCommand.hpp"
#include "core/FeatureCommand.hpp"
#include "core/SketchCommand.hpp"
#include "core/Iges.hpp"
#include "core/Recovery.hpp"
#include "core/Step.hpp"
#include "core/Brep.hpp"
#include "core/Print.hpp"
#include "core/Sculpt.hpp"
#include "core/Wearable.hpp"
#include "core/Units.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <sstream>

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

// Splits primitive args into lengths (in the active units, as typed in `words`) and an
// optional "@x,y,z" / "x,y,z" position.
bool splitPrimitiveArgs(const std::vector<std::string>& args, std::vector<float>& nums, std::vector<std::string>& words,
                        std::optional<Vec3>& at) {
    for (const std::string& a : args) {
        if (a.find(',') != std::string::npos) {
            at = parseVec3(a);
            if (!at) return false;
        } else if (auto n = parseLength(a)) {
            nums.push_back(static_cast<float>(*n));
            words.push_back(a);
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

std::string vecText(const Vec3& v) { return lengthsText({v.x, v.y, v.z}, ", ", "(", ")"); }

// Poses of the moving bodies and the motion values at time t.
std::vector<std::string> describePoses(const Document& doc, float t) {
    std::vector<std::string> lines;
    lines.push_back(fmt("t = %.3f s", t));
    for (const Motion& m : doc.scene.motions()) {
        if (!m.enabled) continue;
        lines.push_back("  " + m.name + "  " + fmt("%.2f ", motionValue(doc.scene, m, t)) + m.unit() + "  " +
                        fmt("%.2f ", motionRate(doc.scene, m, t)) + m.rateUnit());
    }
    const std::vector<Mat4> tf = simulationTransforms(doc.scene, t);
    const auto physics = physicsRun(doc.scene);
    for (std::size_t i = 0; i < doc.scene.bodies().size(); ++i) {
        const bool falling = physics && physics->indexOf(i) >= 0 && !doc.scene.bodies()[i].physics.fixed;
        if (!isMoving(doc.scene, i) && !falling) continue;
        const Body& b = doc.scene.bodies()[i];
        const Vec3 c = tf[i].transformPoint(b.bounds().center());
        const Vec3 x = tf[i].transformDir({1, 0, 0});
        const float heading = degrees(std::atan2(x.y, x.x));
        lines.push_back("  " + b.name + "  centre " + vecText(c) + fmt("  x-axis heading %.1f deg", heading));
    }
    return lines;
}

// A parameter value: a number, or one of the parameter's choices by name ("dual").
std::optional<float> paramValue(const LibraryParam& p, const std::string& word) {
    if (p.choices) {
        // Letters and digits only, lower case: "Timing (GT2)" -> "timinggt2", "V-belt" -> "vbelt".
        auto key = [](const std::string& t) {
            std::string k;
            for (char c : t)
                if (std::isalnum(static_cast<unsigned char>(c))) k += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            return k;
        };
        const std::string w = key(word);
        std::vector<std::string> names;
        std::string list = std::string(p.choices) + "|";
        for (std::size_t at = 0, bar; (bar = list.find('|', at)) != std::string::npos; at = bar + 1)
            names.push_back(key(list.substr(at, bar - at)));
        for (std::size_t i = 0; i < names.size(); ++i)
            if (!w.empty() && names[i] == w) return static_cast<float>(i);
        int match = -1; // a unique prefix also works: TIMING, V, OFF
        for (std::size_t i = 0; i < names.size(); ++i)
            if (!w.empty() && names[i].rfind(w, 0) == 0) match = match < 0 ? static_cast<int>(i) : -2;
        if (match >= 0) return static_cast<float>(match);
    }
    // Lengths may be typed in any unit; the parameter itself is stored in mm.
    const bool length = p.unit && std::string_view(p.unit) == "mm";
    const auto parsed = length ? parseLength(word) : std::optional<double>(parseNumber(word));
    const std::optional<float> v = parsed ? std::optional<float>(static_cast<float>(*parsed)) : std::nullopt;
    if (!v || *v < p.min || *v > p.max) return std::nullopt;
    return p.integer ? std::round(*v) : *v;
}

std::string paramRange(const LibraryParam& p) {
    if (p.choices) {
        std::string c = p.choices;
        std::replace(c.begin(), c.end(), '|', '/');
        return c;
    }
    return fmt("%g to %g", p.min, p.max) + (p.unit[0] ? std::string(" ") + p.unit : std::string());
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
        std::snprintf(buf, sizeof(buf), "  %s [%s%s%s]", b.name.c_str(), b.kind.c_str(), params.empty() ? "" : " ",
                      params.c_str());
        std::string line = std::string(buf) + "  at " + vecText(b.offset) + "  size " +
                           lengthsText({bb.max.x - bb.min.x, bb.max.y - bb.min.y, bb.max.z - bb.min.z}, " x ") +
                           (b.visible ? "" : "  (hidden)");
        if (b.scale != 1.0f) line += fmt("  scale %g", b.scale);
        if (b.rotation.x != 0.0f || b.rotation.y != 0.0f || b.rotation.z != 0.0f)
            line += fmt("  rotated (%g, %g, %g) deg", b.rotation.x, b.rotation.y, b.rotation.z);
        if (!b.component.empty() && b.component != b.name) line += "  {" + b.component + "}";
        l.push_back(line);
    }
    l.push_back("Sketch curves (" + std::to_string(doc.scene.sketch().curves().size()) + "):");
    for (const SketchCurve& c : doc.scene.sketch().curves())
        l.push_back("  " + c.name + "  " + planeName(c.plane) + "  " + c.measure() + (c.visible ? "" : "  (hidden)"));
    l.push_back("Images (" + std::to_string(doc.scene.images().size()) + "):");
    for (const ImageRef& i : doc.scene.images()) {
        l.push_back("  " + i.name + "  " + planeName(i.plane) + "  centre " +
                    lengthsText({i.center.x, i.center.y}, ", ", "(", ")") + "  " + lengthsText({i.width, i.height()}, " x ") +
                    fmt("  opacity %.2f", i.opacity) + (i.pixels.empty() ? "  (file missing)" : ""));
    }
    return l;
}

namespace {
CommandOutput runCommand(Document& doc, std::string_view line, ViewHost* view);
} // namespace

namespace {

// The commands that build the model: the feature history records them, and REGEN
// replays them. SELECT and PLANE are there because later commands default to them.
// The commands that edit only the sketch: UNDO takes them back from a copy of it.
bool sketchCommand(const std::string& cmd) {
    static const char* const kSketch[] = {"LINE",     "PLINE",        "RECT",   "CIRCLE", "POINT",     "ARC",        "SPLINE",
                                          "ELLIPSE",  "SKETCHSLOT",   "POLYGON", "TRIM",  "EXTEND",    "OFFSET",     "CONSTRUCTION",
                                          "CONSTRAIN", "DIM",         "CONSTRAINT", "DRAG",  "ERASE"};
    for (const char* k : kSketch)
        if (cmd == k) return true;
    return false;
}

bool historyCommand(const std::string& cmd) {
    static const char* const kModel[] = {"LINE",     "PLINE",   "RECT",    "CIRCLE",   "POINT",     "BOX",     "CYLINDER", "CONE",    "SPHERE",
                                         "TORUS",    "WEDGE",   "PYRAMID", "PRISM",    "EXTRUDE",   "REVOLVE", "SWEEP",    "LOFT",    "PATTERN",
                                         "MIRROR",   "FILLET",  "CHAMFER",  "SCULPT",  "REMESH",   "MASK",      "SUBDIVIDE", "SYMMETRIZE", "DECIMATE", "HOLE",
                                         "POCKET",   "CHANNEL", "TRIM",    "BOSS",     "BOOLEAN",   "SHELL",   "ARMOR",    "JOINT",   "LIGHTEN",
                                         "PART",     "PARAM",   "ROTATE",  "SCALE",    "ERASE",     "MOVE",    "SELECT",   "REPAIR",  "ORIENT",
                                         "PLANE",    "IMPORT",  "MOTION",  "POSE",     "GESTURE",   "PHYSICS", "CONSTRAIN", "DIM",
                                         "CONSTRAINT", "MATE",  "ASSEMBLY", "ARC",     "SPLINE",   "ELLIPSE",   "SKETCHSLOT", "POLYGON",
                                         "CONSTRUCTION", "EXTEND", "OFFSET", "DRAG"};
    for (const char* k : kModel)
        if (cmd == k) return true;
    return false;
}

void recordHistory(Document& doc, std::string_view line, const CommandOutput& out, bool startedEmpty, const std::string& selBody,
                   const std::string& selCurve, bool pushedUndo) {
    ProjectState& st = doc.state;
    if (!out.ok) return;
    const ParsedCommand pc = tokenizeCommand(line);
    const CommandSpec* spec = findCommand(pc.name);
    if (!spec) return;
    const std::string cmd = spec->name;
    if (cmd == "NEW") {
        st.history.clear();
        st.historyRedo.clear();
        st.historyComplete = true;
        return;
    }
    if (cmd == "UNDO") { // the last step an UNDO takes back goes to the redo list
        for (std::size_t i = st.history.size(); i-- > 0;)
            if (st.history[i].undoable) {
                st.historyRedo.push_back(st.history[i]);
                st.history.erase(st.history.begin() + static_cast<std::ptrdiff_t>(i));
                break;
            }
        return;
    }
    if (cmd == "REDO") {
        if (!st.historyRedo.empty()) {
            st.history.push_back(st.historyRedo.back());
            st.historyRedo.pop_back();
        }
        return;
    }
    if (!historyCommand(cmd)) return;
    if (st.history.empty() && !startedEmpty) st.historyComplete = false; // it began on a model REGEN can't rebuild
    if (st.history.empty() && st.plane != SketchPlane::Top && cmd != "PLANE") { // the plane the history starts on
        HistoryStep plane;
        plane.line = "PLANE " + toUpper(planeKey(st.plane));
        plane.undoable = false;
        plane.unit = st.units.unit;
        st.history.push_back(std::move(plane));
    }
    HistoryStep step;
    step.line = std::string(line);
    step.undoable = pushedUndo; // only an edit with an undo snapshot is taken back by UNDO
    step.millimetres = doc.apiMillimeters;
    step.unit = st.units.unit;
    step.selectedBody = selBody;
    step.selectedCurve = selCurve;
    st.history.push_back(std::move(step));
    st.historyRedo.clear();
}

} // namespace

CommandOutput executeCommand(Document& doc, std::string_view line, ViewHost* view) {
    setCrashContext(std::string(line)); // in the report if this command brings the program down
    // The model changed since the last command, by a brush stroke, a drag or a panel:
    // the history no longer rebuilds it exactly.
    if (!doc.replaying && doc.state.historyRevision != 0 && doc.scene.revision() != doc.state.historyRevision && !doc.state.history.empty())
        doc.state.historyComplete = false;
    const bool empty = doc.scene.bodies().empty() && doc.scene.sketch().curves().empty() && doc.scene.images().empty();
    const auto& bodies = doc.scene.bodies();
    const auto& curves = doc.scene.sketch().curves();
    const std::string selBody = doc.selectedBody >= 0 && doc.selectedBody < static_cast<int>(bodies.size())
                                    ? bodies[static_cast<std::size_t>(doc.selectedBody)].name : std::string();
    const std::string selCurve = doc.selectedCurve >= 0 && doc.selectedCurve < static_cast<int>(curves.size())
                                     ? curves[static_cast<std::size_t>(doc.selectedCurve)].name : std::string();
    const std::uint64_t pushes = doc.scene.undoPushes();
    doc.pendingConstraint.clear();
    doc.historyRewritten = false;
    const CommandSpec* spec = findCommand(tokenizeCommand(line).name);
    const bool sketchEdit = spec && sketchCommand(spec->name) && !doc.replaying;
    const std::uint64_t sketchBefore = doc.scene.sketch().revision();
    if (sketchEdit) doc.scene.pushSketchUndo(std::string(line.substr(0, std::min<std::size_t>(line.size(), 48))));
    // A CAM or LATHE command: the jobs as they were, for UNDO.
    const bool jobCommand = spec && (std::string(spec->name) == "CAM" || std::string(spec->name) == "LATHE") && !doc.replaying;
    std::string camBefore, latheBefore;
    if (jobCommand) camBefore = camToJson(doc.state.cam).dump(), latheBefore = latheToJson(doc.state.lathe).dump();
    CommandOutput out = runCommand(doc, line, view);
    if (sketchEdit) {
        if (out.ok && doc.scene.sketch().revision() != sketchBefore) doc.scene.keepSketchUndo();
        else doc.scene.dropSketchUndo();
    }
    // An edit that kept an undo snapshot and succeeded: record the state it left.
    doc.scene.sealUndo();
    if (jobCommand && out.ok) {
        std::string camAfter = camToJson(doc.state.cam).dump(), latheAfter = latheToJson(doc.state.lathe).dump();
        if (camAfter != camBefore || latheAfter != latheBefore) {
            doc.jobsUndo.push_back({std::move(camBefore), std::move(latheBefore), std::string(line.substr(0, std::min<std::size_t>(line.size(), 48))),
                                    doc.scene.undoDepth()});
            if (doc.jobsUndo.size() > 64) doc.jobsUndo.erase(doc.jobsUndo.begin());
            doc.jobsRedo.clear();
            doc.scene.clearRedo(); // one history: a new edit drops what could be redone
        }
    }
    if (out.ok && doc.scene.undoPushes() != pushes) doc.jobsRedo.clear(); // a new edit: nothing to redo
    if (!doc.replaying && !doc.historyRewritten) recordHistory(doc, line, out, empty, selBody, selCurve, doc.scene.undoPushes() != pushes);
    doc.historyRewritten = false;
    // A new constraint remembers the history step that made it (CONSTRAINT EDIT rewrites it).
    if (out.ok && !doc.pendingConstraint.empty())
        for (SketchConstraint& c : doc.scene.sketch().constraints())
            if (c.name == doc.pendingConstraint) c.step = doc.replaying ? doc.replayStep : static_cast<int>(doc.state.history.size()) - 1;
    doc.pendingConstraint.clear();
    doc.state.historyRevision = doc.scene.revision();
    return out;
}

namespace {

CommandOutput runCommand(Document& doc, std::string_view line, ViewHost* view) {
    CommandOutput out;
    const ParsedCommand pc = tokenizeCommand(line);
    if (pc.name.empty()) return out;
    const CommandSpec* spec = findCommand(pc.name);
    if (!spec) {
        const Plugin* plugin = nullptr;
        if (const PluginCommand* pcmd = findPluginCommand(pc.name, &plugin)) return runPluginCommand(doc, view, *plugin, *pcmd, pc.args);
        return out.fail("Unknown command: " + pc.name + "   (type HELP for the list)");
    }
    const std::string cmd = spec->name;
    const auto& a = pc.args;
    // A quoted argument is a name, whatever it says ("Ball", "Arm, left").
    auto quotedArg = [&](std::size_t k) { return k < pc.quoted.size() && pc.quoted[k]; };
    // Bare numbers are read, and lengths are shown, in the document's units.
    ScopedUnits unitScope(doc.apiMillimeters ? UnitSettings{} : doc.state.units);
    auto usage = [&] { return out.fail(std::string("Usage: ") + spec->usage); };

    // ---- Sketch -----------------------------------------------------------
    // A trailing CONSTRUCTION makes any new curve a construction (reference) curve.
    bool construction = false;
    std::vector<std::string> sketchArgs = a;
    if ((cmd == "LINE" || cmd == "PLINE" || cmd == "RECT" || cmd == "POINT" || cmd == "CIRCLE" || cmd == "ARC" || cmd == "SPLINE" || cmd == "ELLIPSE" ||
         cmd == "SKETCHSLOT" || cmd == "POLYGON") &&
        !sketchArgs.empty() && (iequal(sketchArgs.back(), "construction") || iequal(sketchArgs.back(), "constr"))) {
        construction = true;
        sketchArgs.pop_back();
    }
    auto addSketchCurve = [&](SketchCurve c) {
        c.construction = construction;
        return addCurve(doc, std::move(c));
    };
    if (cmd == "ARC") {
        // ARC start through end | ARC CENTER c start end (counter-clockwise from start to end)
        const bool centred = !sketchArgs.empty() && (iequal(sketchArgs[0], "center") || iequal(sketchArgs[0], "centre"));
        std::vector<std::string> pa(sketchArgs.begin() + (centred ? 1 : 0), sketchArgs.end());
        std::vector<Vec2> pts;
        std::string bad;
        if (!parsePoints(pa, pts, bad)) return out.fail("Not a point: " + bad);
        if (pts.size() != 3) return out.fail("ARC start through end  |  ARC CENTER centre start end");
        SketchCurve c;
        c.kind = CurveKind::Arc;
        if (centred) {
            const float r = std::hypot(pts[1].x - pts[0].x, pts[1].y - pts[0].y);
            if (!(r > 0)) return out.fail("The start must be away from the centre");
            c.pts = {pts[0], pts[1], pts[2]};
            c.radius = r;
        } else {
            Vec2 centre;
            float r;
            bool ccw;
            if (!arcThrough(pts[0], pts[1], pts[2], centre, r, ccw)) return out.fail("The three points are in line: no arc passes through them");
            c.pts = ccw ? std::vector<Vec2>{centre, pts[0], pts[2]} : std::vector<Vec2>{centre, pts[2], pts[0]};
            c.radius = r;
        }
        return addSketchCurve(c);
    }
    if (cmd == "SPLINE") {
        std::vector<std::string> pa = sketchArgs;
        bool close = false;
        if (!pa.empty() && (iequal(pa.back(), "close") || iequal(pa.back(), "closed"))) close = true, pa.pop_back();
        std::vector<Vec2> pts;
        std::string bad;
        if (!parsePoints(pa, pts, bad)) return out.fail("Not a point: " + bad);
        if (close && pts.size() > 2 && std::hypot(pts.front().x - pts.back().x, pts.front().y - pts.back().y) < 1e-6f) pts.pop_back();
        if (pts.size() < (close ? 3u : 2u)) return usage();
        SketchCurve c;
        c.kind = CurveKind::Spline;
        c.pts = pts;
        c.closed = close;
        return addSketchCurve(c);
    }
    if (cmd == "ELLIPSE") {
        // ELLIPSE centre major-axis-end minor-radius
        if (sketchArgs.size() != 3) return usage();
        std::vector<Vec2> pts;
        std::string bad;
        if (!parsePoints({sketchArgs[0], sketchArgs[1]}, pts, bad)) return out.fail("Not a point: " + bad);
        const auto ry = parseLength(sketchArgs[2]);
        const float rx = std::hypot(pts[1].x - pts[0].x, pts[1].y - pts[0].y);
        if (!ry || !(*ry > 0) || !(rx > 0)) return out.fail("An ellipse needs a major axis and a minor radius");
        SketchCurve c;
        c.kind = CurveKind::Ellipse;
        c.pts = pts;
        c.radius = static_cast<float>(*ry);
        return addSketchCurve(c);
    }
    if (cmd == "SKETCHSLOT") {
        // SKETCHSLOT centre1 centre2 radius
        if (sketchArgs.size() != 3) return usage();
        std::vector<Vec2> pts;
        std::string bad;
        if (!parsePoints({sketchArgs[0], sketchArgs[1]}, pts, bad)) return out.fail("Not a point: " + bad);
        const auto r = parseLength(sketchArgs[2]);
        if (!r || !(*r > 0) || std::hypot(pts[1].x - pts[0].x, pts[1].y - pts[0].y) <= 0) return out.fail("A slot needs two different end centres and a radius");
        SketchCurve c;
        c.kind = CurveKind::Slot;
        c.pts = pts;
        c.radius = static_cast<float>(*r);
        return addSketchCurve(c);
    }
    if (cmd == "POLYGON") {
        // POLYGON centre radius [sides] [CIRCUMSCRIBED] [ANGLE deg]: a closed polyline
        if (sketchArgs.size() < 2) return usage();
        const auto c0 = parsePointInput(sketchArgs[0]);
        const auto r = parseLength(sketchArgs[1]);
        if (!c0 || !r || !(*r > 0)) return usage();
        const Vec2 centre = c0->resolve({});
        int sides = 6;
        bool outside = false;
        float angle = 0;
        for (std::size_t k = 2; k < sketchArgs.size(); ++k) {
            if (iequal(sketchArgs[k], "circumscribed") || iequal(sketchArgs[k], "across-flats") || iequal(sketchArgs[k], "flats")) outside = true;
            else if (iequal(sketchArgs[k], "inscribed")) outside = false;
            else if (iequal(sketchArgs[k], "angle") && k + 1 < sketchArgs.size()) {
                const auto v = parseNumber(sketchArgs[++k]);
                if (!v) return usage();
                angle = *v;
            } else if (const auto n = parseNumber(sketchArgs[k])) sides = static_cast<int>(*n);
            else return usage();
        }
        if (sides < 3 || sides > 1000) return out.fail("A polygon has 3 to 1000 sides");
        // Inscribed: corners on the radius; circumscribed: sides touching it (a nut across flats).
        const float rc = static_cast<float>(*r) / (outside ? std::cos(kPi / static_cast<float>(sides)) : 1.0f);
        SketchCurve c;
        c.kind = CurveKind::Polyline;
        for (int k = 0; k <= sides; ++k) { // the first corner on +u (turned by ANGLE)
            const float t = angle * kPi / 180.0f + 2.0f * kPi * static_cast<float>(k % sides) / static_cast<float>(sides);
            c.pts.push_back({centre.x + rc * std::cos(t), centre.y + rc * std::sin(t)});
        }
        return addSketchCurve(c);
    }
    if (cmd == "LINE" || cmd == "PLINE" || cmd == "RECT" || cmd == "POINT") {
        std::vector<std::string> pointArgs = sketchArgs;
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
            return addSketchCurve(c);
        }
        if (cmd == "RECT") {
            if (pts.size() != 2 || pts[0].x == pts[1].x || pts[0].y == pts[1].y) return usage();
            c.kind = CurveKind::Rectangle;
            c.pts = pts;
            return addSketchCurve(c);
        }
        if (pts.size() < 2) return usage();
        if (cmd == "PLINE") {
            if (close) pts.push_back(pts.front());
            c.kind = CurveKind::Polyline;
            c.pts = pts;
            return addSketchCurve(c);
        }
        for (std::size_t i = 1; i < pts.size(); ++i) { // LINE p1 p2 p3 ... = connected segments
            SketchCurve seg;
            seg.kind = CurveKind::Line;
            seg.pts = {pts[i - 1], pts[i]};
            for (auto& l : addSketchCurve(seg).lines) out.say(l);
        }
        return out;
    }
    if (cmd == "CIRCLE") {
        if (sketchArgs.size() != 2) return usage();
        const auto c = parsePointInput(sketchArgs[0]);
        if (!c) return usage();
        const Vec2 centre = c->resolve({});
        float r = 0.0f;
        if (const auto n = parseLength(sketchArgs[1])) r = static_cast<float>(*n);
        else if (const auto p = parsePointInput(sketchArgs[1])) {
            const Vec2 q = p->resolve(centre);
            r = std::hypot(q.x - centre.x, q.y - centre.y);
        }
        if (!(r > 0.0f)) return out.fail("Radius must be positive");
        SketchCurve curve;
        curve.kind = CurveKind::Circle;
        curve.pts = {centre};
        curve.radius = r;
        return addSketchCurve(curve);
    }

    // ---- Solids -----------------------------------------------------------
    static const std::pair<const char*, const char*> kSolids[] = {
        {"BOX", "box"},     {"CYLINDER", "cylinder"}, {"CONE", "cone"},       {"SPHERE", "sphere"},
        {"TORUS", "torus"}, {"WEDGE", "wedge"},       {"PYRAMID", "pyramid"}, {"PRISM", "prism"}};
    for (const auto& [name, kind] : kSolids) {
        if (cmd != name) continue;
        std::vector<float> n;
        std::vector<std::string> words;
        std::optional<Vec3> at;
        if (!splitPrimitiveArgs(a, n, words, at)) return usage();
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
            else if (const auto sides = parseNumber(words[2])) params[2] = *sides; // a count, not a length
            else return usage();
        } else
            return usage();
        Body b = defaultPrimitive(k, ++doc.primitiveSerial);
        b.mesh = buildPrimitive(k, params);
        if (b.mesh.indices.empty()) return out.fail("Invalid dimensions for " + std::string(name));
        b.params = params;
        attachPrimitiveBrep(b);
        b.offset = at.value_or(Vec3{});
        const Vec3 o = b.offset;
        doc.scene.add(std::move(b));
        doc.selectedBody = static_cast<int>(doc.scene.bodies().size()) - 1;
        doc.selectedCurve = doc.selectedImage = -1;
        markChanged(doc);
        out.say("Created " + doc.scene.bodies().back().name + " at " + vecText(o));
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
            if (const auto d = parseLength(nameWords.back())) {
                distance = static_cast<float>(*d);
                nameWords.pop_back();
            }
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
        const auto profile = profileFromSketch(doc.scene.sketch(), static_cast<std::size_t>(index), &why);
        if (!profile) return out.fail("Cannot extrude: " + why);
        Body b = defaultPrimitive("box", ++doc.primitiveSerial); // name / colour series
        b.name = "Extrude " + std::to_string(doc.primitiveSerial);
        b.kind = "extrude";
        b.params = extrudeParams(*profile, c.plane, *distance, symmetric);
        b.mesh = buildExtrusion(b.params);
        if (b.mesh.indices.empty()) return out.fail("Extrusion failed for " + c.name);
        b.offset = {};
        attachPrimitiveBrep(b);
        const std::string profileName = c.name;
        if (!keep) { // the sketch is consumed, as in most CAD tools (show it again in the Model panel)
            std::vector<std::size_t> chain; // a loop of curves end to end: all of them
            if (!c.isClosed()) chainLoop(doc.scene.sketch(), static_cast<std::size_t>(index), &chain);
            for (std::size_t k : chain) doc.scene.sketch().curves()[k].visible = false;
            c.visible = false;
            doc.scene.sketch().touch();
        }
        doc.scene.add(std::move(b));
        doc.selectedBody = static_cast<int>(doc.scene.bodies().size()) - 1;
        doc.selectedCurve = doc.selectedImage = -1;
        markChanged(doc);
        out.say("Extruded " + profileName + " " + lengthText(*distance) + (symmetric ? " symmetric" : "") + " -> " +
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
        doc.state.cam = CamJob{}; // the jobs, the drawing and the mates belonged to the old parts
        doc.jobsUndo.clear(), doc.jobsRedo.clear();
        doc.state.lathe = LatheJob{};
        doc.state.drawing = Drawing{};
        doc.state.assembly = AssemblyState{};
        doc.state.history.clear();
        doc.state.historyRedo.clear();
        doc.state.historyComplete = true;
        doc.state.simDuration = 0.0f;
        out.documentReplaced = true;
        out.say("New empty project");
        return out;
    }
    if (cmd == "RUN") {
        // Each line as if typed, stopping at the first that fails.
        static thread_local int depth = 0;
        if (a.empty()) return usage();
        if (depth >= 8) return out.fail("RUN nests too deep");
        const std::string path = joinArgs(a);
        std::ifstream in(path);
        if (!in) return out.fail("Cannot read " + path);
        ++depth;
        struct Leave {
            ~Leave() { --depth; }
        } leave;
        std::string text;
        int lineNo = 0, ran = 0;
        while (std::getline(in, text)) {
            ++lineNo;
            if (!text.empty() && text.back() == '\r') text.pop_back();
            const std::size_t first = text.find_first_not_of(" \t");
            if (first == std::string::npos || text[first] == '#') continue;
            const CommandOutput r = executeCommand(doc, text, view);
            out.clearHistory = out.clearHistory || r.clearHistory;
            out.documentReplaced = out.documentReplaced || r.documentReplaced;
            ++ran;
            if (!r.ok)
                return out.fail(path + fmt(" line %g: ", static_cast<double>(lineNo)) + text.substr(first) + "  -> " +
                                (r.lines.empty() ? "failed" : r.lines.back()));
        }
        out.say(fmt("Ran %g commands from ", static_cast<double>(ran)) + path);
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
        doc.jobsUndo.clear(), doc.jobsRedo.clear();
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
    if (cmd == "LICENCE") {
        // LICENCE [SHOW] | INSTALL file | REMOVE
        const std::string sub = a.empty() ? "show" : lower(a[0]);
        std::string err;
        if (sub == "show") {
            const LicenceStatus st = currentLicence();
            out.say(st.describe());
            if (st.state != LicenceState::None && st.state != LicenceState::NoKey) out.say("  licence file: " + st.path);
            if (buildLicenceKey().empty()) out.say("  this build was made without a licence key (SICDM_LICENCE_PUBLIC_KEY)");
            return out;
        }
        if (sub == "install") {
            if (a.size() < 2) return out.fail("LICENCE INSTALL licence.json");
            LicenceStatus st;
            if (!installLicence(a[1], &st, &err)) return out.fail(err);
            out.say("Installed: " + st.describe());
            return out;
        }
        if (sub == "remove") {
            if (!removeLicence(&err)) return out.fail(err);
            out.say("Licence removed: running as the Community edition");
            return out;
        }
        return out.fail("LICENCE [SHOW] | INSTALL licence.json | REMOVE");
    }
    if (cmd == "CRASHES") {
        // CRASHES [LIST] | SHOW n | CLEAR | FOLDER
        const std::string sub = a.empty() ? "list" : lower(a[0]);
        const std::vector<CrashReportFile> reports = crashReports();
        if (sub == "list") {
            if (reports.empty()) {
                out.say("No crash reports (they go to " + crashDirectory() + ")");
                return out;
            }
            out.say("Crash reports, newest first (kept on this computer; nothing is sent):");
            for (std::size_t i = 0; i < reports.size(); ++i) out.say("  " + std::to_string(i + 1) + ". " + reports[i].path);
            out.say("CRASHES SHOW n prints one; CRASHES CLEAR deletes them");
            return out;
        }
        if (sub == "show") {
            const long n = a.size() > 1 ? std::strtol(a[1].c_str(), nullptr, 10) : 1;
            if (n < 1 || n > static_cast<long>(reports.size())) return out.fail(reports.empty() ? "No crash reports" : "CRASHES SHOW 1.." + std::to_string(reports.size()));
            std::ifstream f(reports[static_cast<std::size_t>(n - 1)].path, std::ios::binary);
            for (std::string text; std::getline(f, text);) out.say(text);
            return out;
        }
        if (sub == "clear") {
            clearCrashReports();
            out.say("Deleted " + std::to_string(reports.size()) + " crash reports");
            return out;
        }
        if (sub == "folder") {
            out.say(crashDirectory());
            return out;
        }
        return out.fail("CRASHES [LIST] | SHOW n | CLEAR | FOLDER");
    }
    if (cmd == "PLUGINS") {
        // PLUGINS [LIST] | RELOAD | FOLDER
        const std::string sub = a.empty() ? "list" : lower(a[0]);
        if (sub == "reload") reloadPlugins();
        else if (sub == "folder") {
            out.say(pluginsFolder());
            return out;
        } else if (sub != "list") return out.fail("PLUGINS [LIST] | RELOAD | FOLDER");
        const std::vector<Plugin>& all = plugins();
        if (all.empty()) {
            out.say("No plugins. Put a plugin's folder (with its plugin.json) in " + pluginsFolder() + ", or list folders in SICDM_PLUGIN_PATH; then PLUGINS RELOAD");
            return out;
        }
        for (const Plugin& p : all) {
            out.say(p.name + (p.version.empty() ? "" : " " + p.version) + (p.description.empty() ? "" : ": " + p.description) + "  (" + p.dir + ")");
            for (const PluginCommand& c : p.commands) out.say("  " + c.usage + (c.help.empty() ? "" : "   " + c.help));
            if (!p.error.empty()) out.say("  not loaded in full: " + p.error);
        }
        return out;
    }
    if (cmd == "NOTICES") {
        std::istringstream in(thirdPartyNotices());
        for (std::string text; std::getline(in, text);) out.say(text);
        return out;
    }
    if (cmd == "RECOVER") {
        // RECOVER [LIST] | RECOVER n | RECOVER DISCARD n|ALL
        const std::vector<RecoveryFile> files = findRecoveryFiles();
        auto when = [](std::int64_t t) {
            const std::time_t tt = static_cast<std::time_t>(t);
            std::tm tm{};
#if defined(_WIN32)
            localtime_s(&tm, &tt);
#else
            localtime_r(&tt, &tm);
#endif
            char b[32];
            std::strftime(b, sizeof b, "%Y-%m-%d %H:%M", &tm);
            return std::string(b);
        };
        auto pick = [&](const std::string& w) -> int {
            char* end = nullptr;
            const long n = std::strtol(w.c_str(), &end, 10);
            return end && *end == '\0' && n >= 1 && n <= static_cast<long>(files.size()) ? static_cast<int>(n) - 1 : -1;
        };
        if (a.empty() || iequal(a[0], "list")) {
            if (files.empty()) {
                out.say("No unsaved work to recover");
                return out;
            }
            out.say("Unsaved work from sessions that ended unexpectedly:");
            for (std::size_t i = 0; i < files.size(); ++i)
                out.say("  " + std::to_string(i + 1) + ". " + files[i].name + (files[i].original.empty() ? "" : "  (" + files[i].original + ")") +
                        "  autosaved " + when(files[i].savedAt));
            out.say("RECOVER n reopens one; RECOVER DISCARD n or ALL deletes them");
            return out;
        }
        if (iequal(a[0], "discard")) {
            if (a.size() < 2) return usage();
            if (iequal(a[1], "all")) {
                for (const RecoveryFile& f : files) discardRecoveryFile(f);
                out.say("Discarded " + std::to_string(files.size()) + " recovered file" + (files.size() == 1 ? "" : "s"));
                return out;
            }
            const int i = pick(a[1]);
            if (i < 0) return out.fail("No recovered file " + a[1] + " (RECOVER lists them)");
            discardRecoveryFile(files[static_cast<std::size_t>(i)]);
            out.say("Discarded the unsaved work in " + files[static_cast<std::size_t>(i)].name);
            return out;
        }
        const int i = pick(a[0]);
        if (i < 0) return files.empty() ? out.fail("No unsaved work to recover") : out.fail("No recovered file " + a[0] + " (RECOVER lists them)");
        const RecoveryFile& f = files[static_cast<std::size_t>(i)];
        if (!view && doc.dirty) return out.fail("Save the open project first: the recovered one replaces it");
        std::string err, warn;
        Scene loaded;
        ProjectState st;
        if (!loadProject(loaded, st, f.project, &err, &warn)) return out.fail("Cannot recover " + f.name + ": " + err);
        if (view) view->prepareDocument(""); // a file tab of its own
        doc.scene.assign(std::move(loaded));
        doc.jobsUndo.clear(), doc.jobsRedo.clear();
        doc.state = st;
        doc.path = f.original; // SAVE puts it back where it came from
        doc.name = f.name;
        doc.dirty = true; // still unsaved
        doc.clearSelection();
        doc.primitiveSerial = static_cast<int>(doc.scene.bodies().size());
        adoptRecoveryFile(f);
        out.documentReplaced = true;
        out.say("Recovered " + f.name + " as autosaved " + when(f.savedAt) + ". It isn't saved yet: SAVE" +
                (f.original.empty() ? "AS a path" : "") + " keeps it");
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
        std::optional<float> speed, range, start, radius, ratio, step;
        std::optional<std::string> follow;
        std::optional<MotionMode> mode;
        std::optional<std::vector<std::string>> targets;
        std::optional<bool> each; // EACH: every body turns about its own centre
        std::optional<bool> path; // PATH: a linear motion moves a chain / belt along its loop
        std::optional<std::vector<MotionKey>> keys;           // KEYS t:v,t:v,...
        std::optional<std::optional<std::pair<float, float>>> limits; // LIMITS lo hi | OFF
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
            } else if (k == "follow" || k == "roll" || k == "driven") {
                if (!hasNext) return out.fail("FOLLOW needs a circular motion name (or NONE)");
                follow = iequal(a[++i], "none") ? std::string() : a[i];
            } else if (k == "radius") {
                const auto r = hasNext ? parseLength(a[i + 1]) : std::nullopt;
                if (!r || *r < 0.0) return out.fail("RADIUS needs a length >= 0 (0 = wheel size)");
                radius = static_cast<float>(*r);
                ++i;
            } else if (k == "ratio") {
                if (!num(ratio)) return out.fail("RATIO needs a number (e.g. -0.5 for a 1:2 external mesh)");
            } else if (k == "step" || k == "steps") {
                if (!num(step) || *step < 0.0f) return out.fail("STEP needs a size >= 0 (0 = smooth)");
            } else if (k == "keys" || k == "keyframes") {
                // KEYS t:v,t:v,...  (seconds : deg or length);  KEYS NONE removes them
                if (!hasNext) return out.fail("KEYS needs t:value,t:value,... (or NONE)");
                keys = std::vector<MotionKey>{};
                if (!iequal(a[i + 1], "none")) {
                    std::string list = a[i + 1];
                    for (std::size_t at = 0; at <= list.size();) {
                        const std::size_t comma = std::min(list.find(',', at), list.size());
                        const std::string item = list.substr(at, comma - at);
                        const std::size_t colon = item.find(':');
                        const auto t = colon == std::string::npos ? std::nullopt : parseNumber(item.substr(0, colon));
                        std::optional<float> value;
                        if (colon != std::string::npos) {
                            if (*kind == MotionKind::Linear) {
                                if (const auto mm = parseLength(item.substr(colon + 1))) value = static_cast<float>(*mm);
                            } else {
                                value = parseNumber(item.substr(colon + 1));
                            }
                        }
                        if (!t || *t < 0.0f || !value) return out.fail("Not a key: " + item + "   (seconds:value, e.g. 0:0,1.5:90)");
                        keys->push_back({*t, *value});
                        at = comma + 1;
                    }
                }
                ++i;
            } else if (k == "limits" || k == "limit") {
                if (hasNext && iequal(a[i + 1], "off")) {
                    limits = std::optional<std::pair<float, float>>{};
                    ++i;
                    continue;
                }
                const auto lo = i + 2 < a.size() ? parseNumber(a[i + 1]) : std::nullopt;
                const auto hi = i + 2 < a.size() ? parseNumber(a[i + 2]) : std::nullopt;
                if (!lo || !hi || *lo > *hi) return out.fail("LIMITS needs lo hi (lo <= hi) or OFF");
                limits = std::optional<std::pair<float, float>>{{*lo, *hi}};
                i += 2;
            } else if (k == "path" || k == "alongloop" || k == "travel_loop") {
                path = true; // chain / belt: travel along its loop
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
        // A new speed, sweep or leader takes over from keyframes (unless new keys come too).
        if ((speed || range || (follow && !follow->empty())) && !keys) m.keys.clear();
        if (keys) {
            m.keys = *keys;
            m.sortKeys();
        }
        if (limits) {
            m.hasLimits = limits->has_value();
            if (m.hasLimits) m.lo = (*limits)->first, m.hi = (*limits)->second;
        }
        if (speed) m.speed = *speed;
        if (range) m.range = *range;
        if (mode) m.mode = *mode;
        if (range && *range > 0.0f && !mode && m.mode == MotionMode::Continuous) m.mode = MotionMode::Oscillate;
        if (start) m.start = *start;
        if (each) m.eachCentre = *each;
        if (path) {
            if (m.kind != MotionKind::Linear) return out.fail("PATH is for linear motions (chains and belts)");
            m.alongLoop = *path;
        }
        if (follow && !follow->empty()) {
            const int d = findMotion(doc, *follow);
            if (d < 0 || motions[static_cast<std::size_t>(d)].kind != MotionKind::Circular)
                return out.fail("No circular motion named " + *follow);
            if (iequal(motions[static_cast<std::size_t>(d)].name, m.name)) return out.fail("A motion cannot follow itself");
            m.follow = motions[static_cast<std::size_t>(d)].name;
        } else if (follow) {
            m.follow.clear();
        }
        if (radius) m.radius = *radius;
        if (ratio) m.ratio = *ratio;
        if (step) m.step = *step;
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
    if (cmd == "POSE" || cmd == "GESTURE") {
        const bool pose = cmd == "POSE";
        if (!a.empty() && iequal(a[0], "list")) {
            if (pose) {
                out.say("Postures (POSE [robot] name [IN s]):");
                for (const NamedPosture& p : namedPostures()) out.say(std::string("  ") + toUpper(p.name) + "  " + p.summary);
            } else {
                out.say("Gestures (GESTURE [robot] name [LOOP|ONCE] [SPEED x|DURATION s] [MIRROR]):");
                for (const Gesture& g : namedGestures())
                    out.say(std::string("  ") + toUpper(g.name) + "  " + g.summary + (g.loop ? "  (loops)" : ""));
            }
            const auto robots = robotNames(doc.scene);
            std::string rl;
            for (const std::string& r : robots) rl += (rl.empty() ? "" : ", ") + r;
            out.say("Robots: " + (rl.empty() ? std::string("none (insert one with PART humanoid, robot_arm, robot_hand...)") : rl));
            return out;
        }
        // The robot: the longest leading run of words naming a component or body, else the
        // selected body's component, else the only (or first) robot in the scene.
        std::optional<Robot> robot;
        std::size_t i = 0;
        for (std::size_t n = a.size(); n > 0 && !robot; --n) {
            std::string name;
            for (std::size_t k = 0; k < n; ++k) name += (k ? " " : "") + a[k];
            if ((robot = findRobot(doc.scene, name))) i = n;
        }
        if (!robot && doc.selectedBody >= 0 && doc.selectedBody < static_cast<int>(doc.scene.bodies().size()))
            robot = findRobot(doc.scene, doc.scene.bodies()[static_cast<std::size_t>(doc.selectedBody)].name);
        if (!robot) {
            const auto robots = robotNames(doc.scene);
            if (robots.empty()) return out.fail("No robot to move: insert one with PART humanoid (or robot_arm, robot_hand...)");
            robot = findRobot(doc.scene, robots.front());
        }
        if (i < a.size() && iequal(a[i], "joints")) {
            // Angles at AT t, else where the motions settle (the end of the latest key).
            float t = 0.0f;
            if (i + 2 < a.size() && iequal(a[i + 1], "at")) {
                const auto v = parseNumber(a[i + 2]);
                if (!v || *v < 0.0f) return out.fail("AT needs seconds");
                t = *v;
            } else {
                for (const RobotJoint& j : robot->joints)
                    for (const Motion& d : doc.scene.motions())
                        if (d.name == j.drive && !d.keys.empty()) t = std::max(t, d.start + d.keys.back().t);
            }
            out.say(robot->component + fmt(" joints at %.2f s (id  angle  range):", t));
            for (const auto& [j, v] : robotAngles(doc.scene, *robot, t))
                out.say("  " + j.id + fmt("  %.1f  %g..%g deg", v, j.lo, j.hi) + "  (" + j.motion + ")");
            return out;
        }
        if (i >= a.size()) return usage();
        MoveResult r;
        if (pose) {
            Posture target;
            float seconds = 0.0f;
            std::string words;
            for (; i < a.size(); ++i) {
                const std::string k = lower(a[i]);
                if ((k == "in" || k == "duration" || k == "time") && i + 1 < a.size()) {
                    const auto s = parseNumber(a[++i]);
                    if (!s || *s < 0.0f) return out.fail("IN needs seconds");
                    seconds = *s;
                    continue;
                }
                if (const auto v = parseNumber(a[i])) {
                    if (words.empty()) return out.fail("Angle " + a[i] + " has no joint before it");
                    target.push_back({words, *v});
                    words.clear();
                    continue;
                }
                words += (words.empty() ? "" : " ") + k;
                if (const NamedPosture* p = findPosture(words)) {
                    // A preset name, unless an angle follows (then it is a joint id).
                    if (i + 1 >= a.size() || !parseNumber(a[i + 1])) {
                        target.insert(target.end(), p->joints.begin(), p->joints.end());
                        words.clear();
                    }
                }
            }
            if (!words.empty()) return out.fail("No posture or angle for '" + words + "' (POSE LIST shows the presets)");
            if (target.empty()) return usage();
            r = poseRobot(doc.scene, *robot, target, seconds);
        } else {
            const Gesture* g = findGesture(a[i]);
            if (!g) return out.fail("No gesture " + a[i] + " (GESTURE LIST shows them)");
            std::optional<bool> loop;
            float scale = 1.0f;
            bool mirror = false;
            for (++i; i < a.size(); ++i) {
                const std::string k = lower(a[i]);
                if (k == "loop") loop = true;
                else if (k == "once") loop = false;
                else if (k == "mirror" || k == "left") mirror = true;
                else if (k == "right") mirror = false;
                else if ((k == "speed" || k == "duration") && i + 1 < a.size()) {
                    const auto v = parseNumber(a[++i]);
                    if (!v || *v <= 0.0f) return out.fail(toUpper(k) + " needs a positive number");
                    if (k == "speed") scale = 1.0f / *v;
                    else {
                        float len = 0.0f;
                        for (const auto& f : g->frames) len = std::max(len, f.first);
                        scale = len > 0.0f ? *v / len : 1.0f;
                    }
                } else return usage();
            }
            r = playGesture(doc.scene, *robot, *g, scale, loop, mirror);
        }
        if (!r.ok) return out.fail(r.error);
        markChanged(doc);
        out.say(robot->component + ": " + std::to_string(r.joints) + (r.joints == 1 ? " joint" : " joints") +
                (pose ? fmt(", in place after %.2f s", r.duration) : fmt(", gesture of %.2f s", r.duration)));
        for (const std::string& n : r.notes) out.say("  note: " + n);
        return out;
    }
    if (cmd == "PHYSICS") {
        PhysicsSettings& ps = doc.scene.physics();
        const std::string sub = a.empty() ? "" : lower(a[0]);
        // Plain numbers (forces in N, fields in V/m or T, accelerations in m/s^2).
        auto numbers = [](const std::string& text) -> std::optional<Vec3> {
            float v[3] = {0, 0, 0};
            std::size_t at = 0;
            for (int k = 0; k < 3; ++k) {
                const std::size_t comma = text.find(',', at);
                const auto x = parseNumber(text.substr(at, comma == std::string::npos ? std::string::npos : comma - at));
                if (!x) return std::nullopt;
                v[k] = *x;
                if (comma == std::string::npos) return k == 2 ? std::optional<Vec3>(Vec3{v[0], v[1], v[2]}) : std::nullopt;
                at = comma + 1;
            }
            return std::nullopt;
        };
        auto settingsLine = [&] {
            std::string l = std::string("Physics ") + (ps.enabled ? "on" : "off") + ": gravity " + gravityModeKey(ps.gravity);
            if (ps.gravity == GravityMode::Uniform || ps.gravity == GravityMode::Both)
                l += fmt(" (%g, %g, %g) m/s^2", ps.g.x, ps.g.y, ps.g.z);
            l += ps.floor ? fmt(", floor (friction %g)", ps.friction) : std::string(", no floor");
            l += ps.air ? ", air drag" : "";
            if (length(ps.electricField) > 0) l += fmt(", E (%g, %g, %g) V/m", ps.electricField.x, ps.electricField.y, ps.electricField.z);
            if (length(ps.magneticField) > 0) l += fmt(", B (%g, %g, %g) T", ps.magneticField.x, ps.magneticField.y, ps.magneticField.z);
            l += fmt(", ambient %g C", ps.ambient - 273.15f);
            if (ps.relativity) l += fmt(", relativity c = %g m/s", ps.lightSpeed);
            l += fmt(", %g s", ps.duration);
            return l;
        };
        auto changed = [&](const std::string& what) {
            markChanged(doc);
            out.say(what.empty() ? settingsLine() : what);
            return out;
        };
        if (sub.empty() || sub == "status") {
            out.say(settingsLine());
            int n = 0;
            for (const Body& b : doc.scene.bodies())
                if (b.physics.enabled) {
                    ++n;
                    out.say("  " + b.name + (b.physics.fixed ? "  fixed" : "  dynamic") +
                            fmt("  mass %g kg", massProperties(b).mass) +
                            (length(b.physics.velocity) > 0 ? "  velocity " + lengthsText({b.physics.velocity.x, b.physics.velocity.y, b.physics.velocity.z}, ", ", "(", ")") + "/s" : std::string()) +
                            (b.physics.charge != 0 ? fmt("  charge %g C", b.physics.charge) : std::string()) +
                            fmt("  %g C", b.physics.temperature - 273.15f));
                }
            if (n == 0) out.say("  No physics bodies: PHYSICS BODY name DYNAMIC (or PHYSICS DEMO)");
            return out;
        }
        if (const auto on = onOff({a[0]}); on && a.size() == 1) {
            ps.enabled = *on;
            if (*on) // bodies that take part: all visible ones, when none was chosen yet
                if (std::none_of(doc.scene.bodies().begin(), doc.scene.bodies().end(), [](const Body& b) { return b.physics.enabled; }))
                    for (Body& b : doc.scene.bodies()) b.physics.enabled = b.visible;
            return changed("");
        }
        if (sub == "demo" || sub == "demos") {
            if (a.size() < 2) {
                out.say("Physics experiments (PHYSICS DEMO name):");
                for (const PhysicsDemo& d : physicsDemos()) out.say(std::string("  ") + toUpper(d.key) + "  " + d.title + ": " + d.summary);
                return out;
            }
            if (!loadPhysicsDemo(doc.scene, lower(a[1]))) return out.fail("No physics demo " + a[1] + " (PHYSICS DEMO lists them)");
            doc.clearSelection();
            doc.state.simDuration = 0.0f;
            markChanged(doc);
            if (view) view->fit();
            for (const PhysicsDemo& d : physicsDemos())
                if (lower(a[1]) == d.key) out.say(std::string("Loaded ") + d.title + ": " + d.summary + "  (SIM PLAY to run it)");
            return out;
        }
        if (sub == "at" || sub == "report") {
            float t = 0.0f;
            std::size_t i = 1;
            if (i < a.size())
                if (const auto v = parseNumber(a[i])) t = *v, ++i;
            std::optional<std::size_t> only;
            if (i < a.size()) {
                std::string name;
                for (; i < a.size(); ++i) name += (name.empty() ? "" : " ") + a[i];
                const int b = findBody(doc, name);
                if (b < 0) return out.fail("No body " + name);
                only = static_cast<std::size_t>(b);
            }
            for (std::string& l : physicsReport(doc.scene, t, only)) out.say(std::move(l));
            return out;
        }
        if (sub == "gravity") {
            if (a.size() < 2) return usage();
            const auto mode = gravityModeFromKey(lower(a[1]));
            if (!mode) return out.fail("GRAVITY OFF, UNIFORM, NEWTON or BOTH");
            ps.gravity = *mode;
            for (std::size_t i = 2; i < a.size(); ++i) {
                if (iequal(a[i], "g") && i + 1 < a.size()) { // Newton's constant
                    const auto v = parseNumber(a[++i]);
                    if (!v || *v < 0) return out.fail("G needs a value in m^3/(kg s^2)");
                    ps.bigG = *v;
                } else if (const auto vec = numbers(a[i])) {
                    ps.g = *vec;
                } else if (const auto v = parseNumber(a[i])) {
                    ps.g = {0, 0, -std::fabs(*v)}; // a magnitude: straight down
                } else {
                    return usage();
                }
            }
            return changed("");
        }
        if (sub == "floor") {
            if (a.size() < 2) return usage();
            for (std::size_t i = 1; i < a.size(); ++i) {
                if (const auto on = onOff({a[i]})) ps.floor = *on;
                else if (iequal(a[i], "friction") && i + 1 < a.size()) {
                    const auto v = parseNumber(a[++i]);
                    if (!v || *v < 0) return out.fail("FRICTION needs a coefficient >= 0");
                    ps.friction = *v;
                } else return usage();
            }
            return changed("");
        }
        if (sub == "air") {
            const auto on = a.size() > 1 ? onOff({a[1]}) : std::nullopt;
            if (!on) return usage();
            ps.air = *on;
            return changed("");
        }
        if (sub == "field") {
            if (a.size() < 3 && !(a.size() == 2 && iequal(a[1], "off"))) return usage();
            if (iequal(a[1], "off")) {
                ps.electricField = ps.magneticField = {};
                return changed("");
            }
            const std::string which = lower(a[1]);
            const auto v = iequal(a[2], "off") ? std::optional<Vec3>(Vec3{}) : numbers(a[2]);
            if (!v || (which != "e" && which != "b")) return out.fail("FIELD E x,y,z (V/m) or FIELD B x,y,z (T)");
            (which == "e" ? ps.electricField : ps.magneticField) = *v;
            return changed("");
        }
        if (sub == "ambient") {
            const auto v = a.size() > 1 ? parseNumber(a[1]) : std::nullopt;
            if (!v || *v < -273.15f) return out.fail("AMBIENT needs a temperature in C");
            ps.ambient = *v + 273.15f;
            return changed("");
        }
        if (sub == "relativity") {
            if (a.size() < 2) return usage();
            for (std::size_t i = 1; i < a.size(); ++i) {
                if (const auto on = onOff({a[i]})) ps.relativity = *on;
                else if (iequal(a[i], "c") && i + 1 < a.size()) {
                    const auto v = parseNumber(a[++i]);
                    if (!v || *v <= 0) return out.fail("C needs a speed of light in m/s");
                    ps.lightSpeed = *v;
                } else return usage();
            }
            return changed("");
        }
        if (sub == "duration" || sub == "length") {
            const auto v = a.size() > 1 ? parseNumber(a[1]) : std::nullopt;
            if (!v || *v <= 0 || *v > 600) return out.fail("DURATION needs 0 to 600 s");
            ps.duration = *v;
            return changed("");
        }
        if (sub == "body") {
            static const char* kKeys[] = {"dynamic", "fixed", "off", "mass", "density", "velocity", "spin", "charge", "temp",
                                          "temperature", "heat", "bounce", "restitution", "force", "wave", "freq",
                                          "frequency", "wavespeed", "on"};
            auto isKey = [&](const std::string& w) {
                return std::find_if(std::begin(kKeys), std::end(kKeys), [&](const char* k) { return lower(w) == k; }) != std::end(kKeys);
            };
            std::size_t i = 1;
            std::string name;
            for (; i < a.size() && !isKey(a[i]); ++i) name += (name.empty() ? "" : " ") + a[i];
            const int bi = name.empty() ? doc.selectedBody : findBody(doc, name);
            if (bi < 0 || bi >= static_cast<int>(doc.scene.bodies().size()))
                return out.fail(name.empty() ? "Select a body or name it: PHYSICS BODY name DYNAMIC" : "No body " + name);
            Body& b = doc.scene.bodies()[static_cast<std::size_t>(bi)];
            PhysicsProps& p = b.physics;
            p.enabled = true; // a body named here takes part (unless OFF follows)
            for (; i < a.size(); ++i) {
                const std::string k = lower(a[i]);
                const bool hasNext = i + 1 < a.size();
                auto number = [&](const char* what, double lo) -> std::optional<double> {
                    const auto v = hasNext ? parseNumber(a[i + 1]) : std::nullopt;
                    if (!v || *v < lo) return std::nullopt;
                    ++i;
                    (void)what;
                    return *v;
                };
                if (k == "dynamic" || k == "on") p.enabled = true, p.fixed = false;
                else if (k == "fixed") p.enabled = true, p.fixed = true;
                else if (k == "off") p.enabled = false;
                else if (k == "mass") {
                    const auto v = number("MASS", 0);
                    if (!v) return out.fail("MASS needs kg (0 = from the volume and density)");
                    p.mass = static_cast<float>(*v);
                } else if (k == "density") {
                    const auto v = number("DENSITY", 1e-6);
                    if (!v) return out.fail("DENSITY needs kg/m^3 (steel 7850, aluminium 2700, water 1000)");
                    p.density = static_cast<float>(*v);
                } else if (k == "velocity" || k == "spin" || k == "force") {
                    const auto v = !hasNext ? std::nullopt : (k == "velocity" ? parseVec3(a[i + 1]) : numbers(a[i + 1]));
                    if (!v) return out.fail(toUpper(k) + (k == "velocity" ? " needs x,y,z per second (lengths)" : k == "spin" ? " needs x,y,z deg/s" : " needs x,y,z N"));
                    ++i;
                    (k == "velocity" ? p.velocity : k == "spin" ? p.spin : p.force) = *v;
                } else if (k == "charge") {
                    const auto v = hasNext ? parseNumber(a[i + 1]) : std::nullopt;
                    if (!v) return out.fail("CHARGE needs coulombs (e.g. 5e-6)");
                    ++i;
                    p.charge = *v;
                } else if (k == "temp" || k == "temperature") {
                    const auto v = number("TEMP", -273.15);
                    if (!v) return out.fail("TEMP needs C");
                    p.temperature = static_cast<float>(*v + 273.15);
                } else if (k == "heat") {
                    const auto v = number("HEAT", 1e-3);
                    if (!v) return out.fail("HEAT needs a specific heat in J/(kg K) (steel 490, aluminium 900, water 4186)");
                    p.specificHeat = static_cast<float>(*v);
                } else if (k == "bounce" || k == "restitution") {
                    const auto v = number("BOUNCE", 0);
                    if (!v || *v > 1) return out.fail("BOUNCE needs 0 (sticks) to 1 (elastic)");
                    p.restitution = static_cast<float>(*v);
                } else if (k == "wave") {
                    const auto w = hasNext ? waveKindFromKey(lower(a[i + 1])) : std::nullopt;
                    if (!w) return out.fail("WAVE SOUND, LIGHT, CUSTOM or OFF");
                    ++i;
                    p.wave = *w;
                } else if (k == "freq" || k == "frequency") {
                    const auto v = number("FREQ", 1e-6);
                    if (!v) return out.fail("FREQ needs Hz");
                    p.waveFrequency = static_cast<float>(*v);
                } else if (k == "wavespeed") {
                    const auto v = hasNext ? parseLength(a[i + 1]) : std::nullopt;
                    if (!v || *v <= 0) return out.fail("WAVESPEED needs a length per second");
                    ++i;
                    p.waveSpeed = static_cast<float>(*v);
                } else {
                    return usage();
                }
            }
            if (p.enabled && !ps.enabled) ps.enabled = true;
            markChanged(doc);
            const MassProperties mp = massProperties(b);
            out.say(b.name + ": " + (!p.enabled ? std::string("not in the physics") : p.fixed ? "fixed" : "dynamic") +
                    fmt(", mass %g kg", mp.mass) + fmt(" (%g kg/m^3)", p.density) +
                    (length(p.velocity) > 0 ? ", velocity " + lengthsText({p.velocity.x, p.velocity.y, p.velocity.z}, ", ", "(", ")") + "/s" : std::string()) +
                    (p.charge != 0 ? fmt(", charge %g C", p.charge) : std::string()) + fmt(", %g C", p.temperature - 273.15f) +
                    fmt(", bounce %g", p.restitution) +
                    (p.wave != WaveKind::Off ? std::string(", ") + waveKindKey(p.wave) + fmt(" waves %g Hz", p.waveFrequency) : std::string()));
            return out;
        }
        return usage();
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

    // ---- Library -----------------------------------------------------------
    if (cmd == "PART") {
        if (a.empty()) {
            out.say("Library (PART name [@x,y,z] [parameter value]...):");
            for (const LibraryItem& it : libraryItems()) {
                std::string params;
                for (const LibraryParam& p : it.params) params += " " + toUpper(p.key) + " " + fmt("%g", p.value);
                out.say(std::string("  ") + it.key + "  " + it.name + "  (" + it.category + ")  " + it.summary + "  ·" + params);
            }
            return out;
        }
        const LibraryItem* item = findLibraryItem(a[0]);
        if (!item) return out.fail("No library part " + a[0] + " (PART lists them)");
        std::map<std::string, float> values;
        std::optional<Vec3> at;
        std::string name;
        for (std::size_t i = 1; i < a.size(); ++i) {
            const std::string k = lower(a[i]);
            if (!a[i].empty() && a[i][0] == '@') {
                if (!(at = parseVec3(a[i].substr(1)))) return out.fail("Position needs @x,y,z");
                continue;
            }
            if ((k == "at" || k == "position") && i + 1 < a.size()) {
                if (!(at = parseVec3(a[++i]))) return out.fail("AT needs x,y,z");
                continue;
            }
            if (k == "name" && i + 1 < a.size()) {
                name = a[++i];
                continue;
            }
            const LibraryParam* param = nullptr;
            for (const LibraryParam& p : item->params)
                if (k == p.key) param = &p;
            if (param) {
                const auto v = i + 1 < a.size() ? paramValue(*param, a[i + 1]) : std::nullopt;
                if (!v) return out.fail(std::string(param->label) + ": " + paramRange(*param));
                values[param->key] = *v;
                ++i;
            } else if (name.empty()) {
                name = a[i];
            } else {
                std::string keys;
                for (const LibraryParam& p : item->params) keys += std::string(keys.empty() ? "" : ", ") + toUpper(p.key);
                return out.fail("Unknown parameter '" + a[i] + "' for " + item->name + " (" + keys + ")");
            }
        }
        auto taken = [&](const std::string& n) {
            for (const Body& b : doc.scene.bodies())
                if (iequal(b.name, n) || iequal(b.component, n)) return true;
            return false;
        };
        if (name.empty()) {
            int n = 1;
            while (taken(std::string(item->name) + " " + std::to_string(n))) ++n;
            name = std::string(item->name) + " " + std::to_string(n);
        } else if (taken(name)) {
            return out.fail("A body or component named " + name + " already exists");
        }
        LibraryComponent c = makeLibraryComponent(*item, values, name, at.value_or(Vec3{}));
        if (c.bodies.empty()) return out.fail("Could not build " + std::string(item->name));
        if (!at && !doc.scene.bodies().empty()) {
            // Beside the model: 30 mm to the right of everything, centred on y = 0.
            AABB scene, part;
            for (const Body& b : doc.scene.bodies()) scene.expand(b.bounds());
            for (const Body& b : c.bodies) part.expand(b.bounds());
            if (scene.valid() && part.valid()) {
                const Vec3 d{scene.max.x + 30.0f - part.min.x, -part.center().y, 0.0f};
                for (Body& b : c.bodies) b.offset += d;
                for (Motion& m : c.motions) m.origin += d;
            }
        }
        const std::size_t first = doc.scene.bodies().size();
        for (Body& b : c.bodies) doc.scene.add(std::move(b));
        for (Motion& m : c.motions) doc.scene.motions().push_back(std::move(m));
        doc.selectedBody = static_cast<int>(first);
        doc.selectedCurve = doc.selectedImage = -1;
        markChanged(doc);
        const std::size_t nb = doc.scene.bodies().size() - first;
        out.say("Inserted " + name + "  (" + item->name + ", " + std::to_string(nb) + (nb == 1 ? " body" : " bodies") +
                (c.motions.empty() ? std::string()
                                   : ", " + std::to_string(c.motions.size()) + (c.motions.size() == 1 ? " motion" : " motions")) +
                ")");
        return out;
    }
    if (cmd == "PARAM") {
        // PARAM [name] [key value]...: edit a placed library part (motor shafts, gear teeth...).
        auto& bodies = doc.scene.bodies();
        std::size_t first = 0;
        int bi = -1;
        if (!a.empty()) {
            bi = findBody(doc, a[0]);
            if (bi >= 0) first = 1;
        }
        if (bi < 0) bi = doc.selectedBody;
        if (bi < 0 || bi >= static_cast<int>(bodies.size()))
            return out.fail(a.empty() ? "Select a library part first, or give its name" : "No body named " + a[0]);
        // A motor housing edits its rotor (the shafts live there).
        if (partParams(bodies[static_cast<std::size_t>(bi)].kind, bodies[static_cast<std::size_t>(bi)].params).empty() &&
            !bodies[static_cast<std::size_t>(bi)].component.empty())
            for (std::size_t i = 0; i < bodies.size(); ++i)
                if (bodies[i].component == bodies[static_cast<std::size_t>(bi)].component && bodies[i].kind == "motor_rotor")
                    bi = static_cast<int>(i);
        Body& b = bodies[static_cast<std::size_t>(bi)];
        const std::vector<LibraryParam> schema = partParams(b.kind, b.params);
        if (schema.empty()) return out.fail(b.name + " has no editable parameters (library parts only)");
        std::vector<float> params = withDefaults(b.kind, b.params);
        auto shown = [](const LibraryParam& p) { return p.min < p.max; };
        if (first >= a.size()) {
            out.say(b.name + " (" + b.kind + "):");
            for (std::size_t i = 0; i < schema.size(); ++i) {
                if (!shown(schema[i])) continue;
                std::string value = fmt("%g", params[i]);
                if (schema[i].choices) {
                    std::string list = std::string(schema[i].choices) + "|";
                    std::size_t at = 0;
                    for (int k = 0; k < static_cast<int>(params[i]); ++k) at = list.find('|', at) + 1;
                    value = list.substr(at, list.find('|', at) - at);
                }
                out.say("  " + toUpper(schema[i].key) + "  " + value + "   (" + paramRange(schema[i]) + ")");
            }
            return out;
        }
        std::string changed;
        for (std::size_t i = first; i < a.size(); i += 2) {
            const std::string k = lower(a[i]);
            std::size_t idx = schema.size();
            for (std::size_t j = 0; j < schema.size(); ++j)
                if (k == schema[j].key && shown(schema[j])) idx = j;
            if (idx == schema.size()) {
                std::string keys;
                for (const LibraryParam& p : schema)
                    if (shown(p)) keys += std::string(keys.empty() ? "" : ", ") + toUpper(p.key);
                return out.fail("Unknown parameter '" + a[i] + "' for " + b.name + " (" + keys + ")");
            }
            const auto v = i + 1 < a.size() ? paramValue(schema[idx], a[i + 1]) : std::nullopt;
            if (!v) return out.fail(std::string(schema[idx].label) + ": " + paramRange(schema[idx]));
            params[idx] = *v;
            changed += (changed.empty() ? "" : ", ") + lower(schema[idx].key) + " " + a[i + 1];
        }
        const std::vector<float> before = b.params;
        b.params = params;
        b.rebuild();
        if (b.mesh.indices.empty()) {
            b.params = before;
            b.rebuild();
            return out.fail("Those values do not make a valid part");
        }
        doc.scene.touchBodies();
        markChanged(doc);
        out.say("Updated " + b.name + ": " + changed);
        return out;
    }
    // The jobs from a snapshot; the ones replaced go to `back` (with the scene's depth there).
    auto swapJobs = [&](std::vector<Document::JobsUndo>& from, std::vector<Document::JobsUndo>& back, std::size_t depth) {
        Document::JobsUndo j = std::move(from.back());
        from.pop_back();
        const auto cj = Json::parse(j.cam), lj = Json::parse(j.lathe);
        CamJob c;
        LatheJob l;
        if (!cj || !lj || !camFromJson(*cj, c) || !latheFromJson(*lj, l)) return std::string();
        back.push_back({camToJson(doc.state.cam).dump(), latheToJson(doc.state.lathe).dump(), j.label, depth});
        doc.state.cam = std::move(c);
        doc.state.lathe = std::move(l);
        markChanged(doc);
        return j.label;
    };
    if (cmd == "UNDO" && !doc.jobsUndo.empty() && doc.scene.undoDepth() <= doc.jobsUndo.back().sceneDepth) {
        const std::string what = swapJobs(doc.jobsUndo, doc.jobsRedo, doc.scene.redoDepth());
        if (what.empty()) return out.fail("Can't undo the CAM change");
        out.say("Undid " + what);
        return out;
    }
    if (cmd == "REDO" && !doc.jobsRedo.empty() && doc.scene.redoDepth() <= doc.jobsRedo.back().sceneDepth) {
        const std::string what = swapJobs(doc.jobsRedo, doc.jobsUndo, doc.scene.undoDepth());
        if (what.empty()) return out.fail("Can't redo the CAM change");
        out.say("Redid " + what);
        return out;
    }
    if (cmd == "UNDO") {
        const std::size_t count = doc.scene.bodies().size();
        std::string why;
        const std::string what = doc.scene.undo(&why);
        if (what.empty()) return out.fail(why.empty() ? "Nothing to undo" : why);
        if (doc.scene.bodies().size() != count || doc.selectedBody >= static_cast<int>(count)) doc.selectedBody = -1;
        if (doc.selectedCurve >= static_cast<int>(doc.scene.sketch().curves().size())) doc.selectedCurve = -1;
        markChanged(doc);
        out.say("Undid " + what);
        return out;
    }
    if (cmd == "REDO") {
        std::string why;
        const std::string what = doc.scene.redo(&why);
        if (what.empty()) return out.fail(why.empty() ? "Nothing to redo" : why);
        if (doc.selectedBody >= static_cast<int>(doc.scene.bodies().size())) doc.selectedBody = -1;
        if (doc.selectedCurve >= static_cast<int>(doc.scene.sketch().curves().size())) doc.selectedCurve = -1;
        markChanged(doc);
        out.say("Redid " + what);
        return out;
    }
    if (cmd == "CAM") return runCamCommand(doc, a, pc.quoted);
    if (cmd == "LATHE") return runLatheCommand(doc, a, pc.quoted);
    if (cmd == "SHEET") return runSheetCommand(doc, a, pc.quoted, view);
    if (cmd == "MATE" || cmd == "ASSEMBLY") return runAssemblyCommand(doc, cmd, a, pc.quoted);
    if (cmd == "CONSTRUCTION" || cmd == "EXTEND" || cmd == "OFFSET" || cmd == "DRAG") return runSketchCommand(doc, cmd, a, pc.quoted);
    if (cmd == "REVOLVE" || cmd == "SWEEP" || cmd == "LOFT" || cmd == "PATTERN" || cmd == "MIRROR" || cmd == "FILLET" || cmd == "MASSPROPS" ||
        cmd == "MEASURE" || cmd == "CHAMFER" || cmd == "EDGES")
        return runFeatureCommand(doc, cmd, a, pc.quoted);
    if (cmd == "HISTORY" || cmd == "REGEN") return runHistoryCommand(doc, cmd, a, pc.quoted);
    if (cmd == "CONSTRAIN" || cmd == "DIM" || cmd == "CONSTRAINTS" || cmd == "CONSTRAINT") return runSketchCommand(doc, cmd, a, pc.quoted);
    if (cmd == "PRINTCHECK" || cmd == "REPAIR" || cmd == "ORIENT" || cmd == "SLICE") {
        // PRINTCHECK [body] [OVERHANG deg] [WALL mm] [BUILD x,y,z]      REPAIR [body]      ORIENT [body] [OVERHANG deg]
        // SLICE [body] TO path.gcode [LAYER h] [WIDTH w] [INFILL %] [WALLS n] [TOP n] [BOTTOM n] [NOZZLE t] [BED t]
        //       [SPEED v] [FILAMENT d] [RETRACT mm]
        static const char* const kKeys[] = {"overhang", "wall", "build", "to", "layer", "width", "infill", "walls", "top",
                                            "bottom", "nozzle", "bed", "speed", "filament", "retract"};
        auto isKey = [&](std::size_t k) {
            if (quotedArg(k)) return false;
            const std::string w = lower(a[k]);
            for (const char* q : kKeys)
                if (w == q) return true;
            return false;
        };
        std::size_t i = 0;
        std::vector<std::string> nameWords;
        if (quotedArg(0)) nameWords.push_back(a[i++]);
        else
            while (i < a.size() && !isKey(i)) nameWords.push_back(a[i++]);
        const int bi = nameWords.empty() ? doc.selectedBody : findBody(doc, joinArgs(nameWords));
        if (bi < 0 || bi >= static_cast<int>(doc.scene.bodies().size()))
            return out.fail(nameWords.empty() ? "Select a body first, or give its name" : "No body named " + joinArgs(nameWords));
        Body& body = doc.scene.bodies()[static_cast<std::size_t>(bi)];
        PrintSettings ps;
        std::string path;
        for (; i < a.size(); ++i) {
            const std::string k = lower(a[i]);
            if (k == "to") {
                if (i + 1 >= a.size()) return usage();
                path = a[++i];
                continue;
            }
            if (k == "build") {
                const auto v = i + 1 < a.size() ? parseVec3(a[++i]) : std::nullopt;
                if (!v || !(v->x > 0) || !(v->y > 0) || !(v->z > 0)) return out.fail("BUILD needs x,y,z");
                ps.buildVolume = *v;
                continue;
            }
            const bool isLength = k == "wall" || k == "layer" || k == "width" || k == "filament" || k == "retract";
            std::optional<double> v;
            if (i + 1 < a.size()) {
                ++i;
                if (isLength) v = parseLength(a[i]);
                else if (const auto n = parseNumber(a[i])) v = *n;
            }
            if (!v || !std::isfinite(*v) || *v < 0) return out.fail(toUpper(k) + " needs a value");
            const auto f = static_cast<float>(*v);
            if (k == "overhang") ps.overhangAngle = std::clamp(f, 0.0f, 90.0f);
            else if (k == "wall") ps.minWall = f;
            else if (k == "layer") ps.layerHeight = f;
            else if (k == "width") ps.lineWidth = f;
            else if (k == "infill") ps.infill = std::clamp(f / 100.0f, 0.0f, 1.0f); // percent
            else if (k == "walls") ps.walls = std::clamp(static_cast<int>(f), 0, 20);
            else if (k == "top") ps.topLayers = std::clamp(static_cast<int>(f), 0, 100);
            else if (k == "bottom") ps.bottomLayers = std::clamp(static_cast<int>(f), 0, 100);
            else if (k == "nozzle") ps.nozzleTemp = f;
            else if (k == "bed") ps.bedTemp = f;
            else if (k == "speed") ps.speed = std::max(f, 1.0f);
            else if (k == "filament") ps.filament = f;
            else if (k == "retract") ps.retract = f;
            else return usage();
        }
        if (body.mesh.indices.empty()) return out.fail(body.name + " has no surface");
        if (cmd == "PRINTCHECK") {
            const PrintReport r = analyzePrint(body, ps);
            const Vec3 size = r.box.max - r.box.min;
            out.say(body.name + (r.printable() ? ": ready to print" : ": needs work before printing"));
            out.say(fmt("  %g triangles, ", static_cast<double>(r.triangles)) + fmt("%g shell(s), ", static_cast<double>(r.shells)) +
                    (r.openEdges == 0 ? "closed" : fmt("%g open edges (holes)", static_cast<double>(r.openEdges))) +
                    (r.nonManifoldEdges ? fmt(", %g non-manifold edges", static_cast<double>(r.nonManifoldEdges)) : "") +
                    (r.insideOut ? ", inside out" : ""));
            out.say("  size " + lengthsText({size.x, size.y, size.z}, " x ", "", "") + (r.fits ? ", fits the build volume" : ", too big for the build volume") +
                    fmt(", volume %.1f cm3", r.volume / 1000.0) + fmt(", area %.1f cm2", r.area / 100.0));
            out.say(fmt("  overhangs past %g deg: ", static_cast<double>(ps.overhangAngle)) +
                    (r.overhangArea < 1.0 ? std::string("none, no support needed") : fmt("%.1f cm2 need support (try ORIENT)", r.overhangArea / 100.0)));
            if (r.wallSamples > 0)
                out.say(fmt("  walls: thinnest %.2f mm", static_cast<double>(r.thinnest)) +
                        (r.thinSamples ? fmt(", %g of the sampled points", static_cast<double>(r.thinSamples)) +
                                             fmt(" are under %g mm", static_cast<double>(ps.minWall))
                                       : fmt(", none under %g mm", static_cast<double>(ps.minWall))));
            out.say(fmt("  about %.0f g, ", r.grams) + fmt("%.1f m of filament, ", r.filamentMetres) + fmt("%.0f min at ", r.minutes) +
                    fmt("%.0f%% infill", static_cast<double>(ps.infill) * 100.0));
            return out;
        }
        if (cmd == "SLICE") {
            if (path.empty()) return out.fail("SLICE needs TO path.gcode");
            SliceReport sr;
            std::string err;
            if (!sliceToGcode(body, ps, path, &sr, &err)) return out.fail(err);
            out.say(body.name + fmt(": %g layers", static_cast<double>(sr.layers)) + fmt(" of %g mm", static_cast<double>(ps.layerHeight)) +
                    fmt(", %.2f m of filament", sr.filamentMm / 1000.0) + fmt(", about %.0f min", sr.seconds / 60.0) + ", G-code in " + path);
            return out;
        }
        doc.scene.pushUndo(cmd + " " + body.name);
        if (cmd == "REPAIR") {
            RepairReport rr;
            std::string err;
            if (!repairMesh(body, &rr, &err)) {
                doc.scene.undo();
                return out.fail(err);
            }
            const std::size_t open = openEdgeCount(body.mesh);
            out.say(body.name + fmt(": repaired: %g degenerate and ", static_cast<double>(rr.degenerate)) +
                    fmt("%g duplicate faces dropped, ", static_cast<double>(rr.duplicates)) + fmt("%g turned, ", static_cast<double>(rr.flipped)) +
                    fmt("%g holes filled, ", static_cast<double>(rr.holesFilled)) + fmt("%g specks removed", static_cast<double>(rr.specks)) +
                    (open == 0 ? ", closed" : fmt(", %g open edges left", static_cast<double>(open))));
        } else { // ORIENT
            double before = 0;
            const double after = orientForPrint(body, ps, &before);
            out.say(body.name + fmt(": oriented for printing, support %.1f cm2", after / 100.0) + fmt(" (was %.1f cm2), on the bed", before / 100.0));
        }
        doc.scene.touchBodies();
        markChanged(doc);
        return out;
    }
    if (cmd == "MASK" || cmd == "SUBDIVIDE" || cmd == "SYMMETRIZE" || cmd == "DECIMATE") {
        // MASK [body] CLEAR|INVERT|SMOOTH|GROW|SHRINK|ALL [n]   SUBDIVIDE [body] [LEVELS n]
        // SYMMETRIZE [body] [X|Y|Z] [-]                          DECIMATE [body] TO n | RATIO r
        static const char* const kWords[] = {"clear", "invert", "smooth", "blur", "grow", "shrink", "all", "levels", "level",
                                             "to", "ratio", "x", "y", "z", "+x", "-x", "+y", "-y", "+z", "-z", "-", "+"};
        auto isWord = [&](std::size_t k) {
            if (quotedArg(k)) return false;
            const std::string w = lower(a[k]);
            if (parseNumber(a[k])) return true;
            for (const char* q : kWords)
                if (w == q) return true;
            return false;
        };
        std::size_t i = 0;
        std::vector<std::string> nameWords;
        if (quotedArg(0)) nameWords.push_back(a[i++]);
        else
            while (i < a.size() && !isWord(i)) nameWords.push_back(a[i++]);
        const int bi = nameWords.empty() ? doc.selectedBody : findBody(doc, joinArgs(nameWords));
        if (bi < 0 || bi >= static_cast<int>(doc.scene.bodies().size()))
            return out.fail(nameWords.empty() ? "Select a body first, or give its name" : "No body named " + joinArgs(nameWords));
        Body& body = doc.scene.bodies()[static_cast<std::size_t>(bi)];
        if (body.mesh.indices.empty()) return out.fail(body.name + " has no surface");
        std::vector<std::string> rest(a.begin() + static_cast<std::ptrdiff_t>(i), a.end());
        for (std::string& w : rest) w = lower(w);
        const std::size_t before = body.mesh.indices.size() / 3;
        std::string error, done;
        doc.scene.pushUndo(cmd + " " + body.name);
        bool ok = true;
        if (cmd == "MASK") {
            if (rest.empty()) return doc.scene.undo(), usage();
            const std::string& w = rest[0];
            const MaskOp op = w == "clear" ? MaskOp::Clear : w == "invert" ? MaskOp::Invert : w == "smooth" || w == "blur" ? MaskOp::Smooth
                              : w == "grow" ? MaskOp::Grow : w == "shrink" ? MaskOp::Shrink : MaskOp::All;
            if (op == MaskOp::All && w != "all") return doc.scene.undo(), usage();
            const auto times = rest.size() > 1 ? parseNumber(rest[1]) : std::nullopt;
            const int n = times ? std::clamp(static_cast<int>(*times), 1, 50) : 1;
            for (int k = 0; k < (op == MaskOp::Smooth || op == MaskOp::Grow || op == MaskOp::Shrink ? n : 1); ++k) maskBody(body, op);
            done = "mask " + w + fmt(", %.0f%% masked", 100.0 * maskedFraction(body));
        } else if (cmd == "SUBDIVIDE") {
            int levels = 1;
            for (std::size_t k = 0; k < rest.size(); ++k)
                if (const auto n = parseNumber(rest[k])) levels = std::clamp(static_cast<int>(*n), 1, 6);
            ok = subdivideBody(body, levels, 4000000, &error);
            done = fmt("subdivided %g times", static_cast<double>(levels));
        } else if (cmd == "SYMMETRIZE") {
            int axis = 0;
            bool positive = true;
            for (const std::string& w : rest) {
                if (w.find('x') != std::string::npos) axis = 0;
                if (w.find('y') != std::string::npos) axis = 1;
                if (w.find('z') != std::string::npos) axis = 2;
                if (!w.empty() && w[0] == '-') positive = false;
            }
            ok = symmetrizeBody(body, axis, positive, &error);
            done = std::string("symmetrized: the ") + (positive ? "+" : "-") + "xyz"[axis] + " side mirrored across";
        } else { // DECIMATE
            std::optional<double> to, ratio;
            for (std::size_t k = 0; k + 1 < rest.size(); ++k) {
                if (rest[k] == "to") to = parseNumber(rest[k + 1]);
                if (rest[k] == "ratio") ratio = parseNumber(rest[k + 1]);
            }
            if (!to && !ratio && !rest.empty()) to = parseNumber(rest[0]);
            if ((!to || *to < 4) && (!ratio || !(*ratio > 0) || *ratio >= 1)) return doc.scene.undo(), usage();
            const std::size_t target = to ? static_cast<std::size_t>(*to) : static_cast<std::size_t>(static_cast<double>(before) * *ratio);
            ok = decimateBody(body, target, &error);
            done = "decimated";
        }
        if (!ok) {
            doc.scene.undo();
            return out.fail(error.empty() ? "Nothing changed" : error);
        }
        doc.scene.touchBodies();
        markChanged(doc);
        const std::size_t open = openEdgeCount(body.mesh);
        out.say(body.name + ": " + done + fmt(", %g triangles", static_cast<double>(body.mesh.indices.size() / 3)) +
                fmt(" (was %g)", static_cast<double>(before)) + (open == 0 ? ", closed" : fmt(", %g open edges", static_cast<double>(open))));
        return out;
    }
    // TRIM "curve" AT x,y cuts a sketch curve back to the curves crossing it.
    if (cmd == "TRIM" && std::any_of(a.begin(), a.end(), [](const std::string& w) { return iequal(w, "at"); })) {
        bool curveFirst = false;
        for (std::size_t k = 0; k < a.size() && !iequal(a[k], "at"); ++k) {
            std::string nm;
            for (std::size_t q = 0; q <= k; ++q) nm += (q ? " " : "") + a[q];
            for (const SketchCurve& c : doc.scene.sketch().curves()) curveFirst = curveFirst || iequal(c.name, nm);
        }
        if (curveFirst) return runSketchCommand(doc, "TRIMCURVE", a, pc.quoted);
    }
    if (cmd == "SCULPT" || cmd == "HOLE" || cmd == "POCKET" || cmd == "CHANNEL" || cmd == "TRIM" || cmd == "BOSS") {
        if (cmd == "SCULPT" && a.size() == 1 && iequal(a[0], "list")) {
            out.say("Brushes:");
            for (SculptBrush br : sculptBrushes()) out.say(std::string("  ") + toUpper(sculptBrushKey(br)) + "  " + sculptBrushName(br));
            return out;
        }
        // Leading words name the body (default: the selection), up to the first keyword or point.
        static const char* const kKeys[] = {"at", "dir", "radius", "strength", "invert", "symmetry", "sym", "detail", "falloff", "radial",
                                            "norefine", "move", "refine", "diameter", "d", "depth", "through", "box",
                                            "round", "ball", "width", "length", "angle", "path", "plane", "normal",
                                            "smaller", "cylinder", "height"};
        auto isKey = [&](const std::string& w) {
            const std::string k = lower(w);
            if (cmd == "SCULPT" && sculptBrushFromKey(k)) return true;
            for (const char* q : kKeys)
                if (k == q) return true;
            return w.find(',') != std::string::npos;
        };
        std::size_t i = 0;
        std::vector<std::string> nameWords;
        if (quotedArg(0)) nameWords.push_back(a[i++]);
        else
            while (i < a.size() && !isKey(a[i])) nameWords.push_back(a[i++]);
        const int bi = nameWords.empty() ? doc.selectedBody : findBody(doc, joinArgs(nameWords));
        if (bi < 0 || bi >= static_cast<int>(doc.scene.bodies().size()))
            return out.fail(nameWords.empty() ? "Select a body first, or give its name" : "No body named " + joinArgs(nameWords));
        Body& body = doc.scene.bodies()[static_cast<std::size_t>(bi)];

        SculptSettings ss;
        bool haveBrush = false, refineOnly = false, through = false, smaller = false;
        float refineDetail = 0, diameter = 0, depth = 0, width = 0, length_ = 0, height = 0, angle = 0;
        std::optional<Vec3> dir, normal, move;
        std::vector<Vec3> points;
        std::string shape;
        auto lengthArg = [&](float& v, const char* what) -> bool {
            if (i + 1 >= a.size()) return false;
            const auto l = parseLength(a[++i]);
            if (!l || *l < 0) return false;
            v = static_cast<float>(*l);
            (void)what;
            return true;
        };
        for (; i < a.size(); ++i) {
            const std::string k = lower(a[i]);
            if (a[i].find(',') != std::string::npos) {
                const auto p = parseVec3(a[i]);
                if (!p) return out.fail("Point " + a[i] + " needs x,y,z");
                points.push_back(*p);
            } else if (cmd == "SCULPT" && !haveBrush && sculptBrushFromKey(k)) {
                ss.brush = *sculptBrushFromKey(k);
                haveBrush = true;
                if (k == "push") ss.invert = true;
            } else if (k == "at" || k == "path" || k == "plane") {
                continue;
            } else if (k == "refine" && cmd == "SCULPT") {
                refineOnly = true;
                if (!lengthArg(refineDetail, "REFINE") || refineDetail <= 0) return out.fail("REFINE needs an edge length");
            } else if (k == "radius") {
                if (!lengthArg(ss.radius, "RADIUS") || ss.radius <= 0) return out.fail("RADIUS needs a length");
            } else if (k == "strength") {
                const auto v = i + 1 < a.size() ? parseNumber(a[++i]) : std::nullopt;
                if (!v || *v < 0 || *v > 1) return out.fail("STRENGTH needs 0..1");
                ss.strength = *v;
            } else if (k == "invert") {
                ss.invert = !ss.invert;
            } else if (k == "symmetry" || k == "sym") {
                // SYMMETRY alone mirrors across x; SYMMETRY XZ (any of x, y, z) across those.
                const std::string axes = i + 1 < a.size() && !quotedArg(i + 1) ? lower(a[i + 1]) : "";
                if (!axes.empty() && axes.find_first_not_of("xyz") == std::string::npos) {
                    ++i;
                    ss.symmetryX = axes.find('x') != std::string::npos;
                    ss.symmetryY = axes.find('y') != std::string::npos;
                    ss.symmetryZ = axes.find('z') != std::string::npos;
                } else {
                    ss.symmetryX = true;
                }
            } else if (k == "radial") {
                // RADIAL n [X|Y|Z]: n copies around an axis through the body's centre (default Z).
                const auto n = i + 1 < a.size() ? parseNumber(a[++i]) : std::nullopt;
                if (!n || *n < 1 || *n > 64) return out.fail("RADIAL needs a count, 1 to 64");
                ss.radial = static_cast<int>(*n);
                const std::string ax = i + 1 < a.size() && !quotedArg(i + 1) ? lower(a[i + 1]) : "";
                if (ax == "x" || ax == "y" || ax == "z") {
                    ++i;
                    ss.radialAxis = ax == "x" ? 0 : ax == "y" ? 1 : 2;
                }
            } else if (k == "falloff") {
                const auto f = i + 1 < a.size() ? sculptFalloffFromKey(a[++i]) : std::nullopt;
                if (!f) return out.fail("FALLOFF is one of SMOOTH SPHERE SHARP LINEAR CONSTANT");
                ss.falloff = *f;
            } else if (k == "detail") {
                if (!lengthArg(ss.detail, "DETAIL")) return out.fail("DETAIL needs a length");
            } else if (k == "norefine") {
                ss.refine = false;
            } else if (k == "move" || k == "dir" || k == "normal") {
                const auto v = i + 1 < a.size() ? parseVec3(a[++i]) : std::nullopt;
                if (!v) return out.fail(toUpper(k) + " needs x,y,z");
                (k == "move" ? move : k == "dir" ? dir : normal) = *v;
            } else if (k == "diameter" || k == "d") {
                if (!lengthArg(diameter, "DIAMETER") || diameter <= 0) return out.fail("DIAMETER needs a length");
            } else if (k == "depth") {
                if (!lengthArg(depth, "DEPTH")) return out.fail("DEPTH needs a length");
            } else if (k == "through") {
                through = true;
            } else if (k == "width") {
                if (!lengthArg(width, "WIDTH") || width <= 0) return out.fail("WIDTH needs a length");
            } else if (k == "length") {
                if (!lengthArg(length_, "LENGTH")) return out.fail("LENGTH needs a length");
            } else if (k == "height") {
                if (!lengthArg(height, "HEIGHT")) return out.fail("HEIGHT needs a length");
            } else if (k == "angle") {
                const auto v = i + 1 < a.size() ? parseNumber(a[++i]) : std::nullopt;
                if (!v) return out.fail("ANGLE needs degrees");
                angle = *v;
            } else if (k == "box" || k == "round" || k == "ball" || k == "cylinder") {
                shape = k;
            } else if (k == "smaller") {
                smaller = true;
            } else {
                return usage();
            }
        }

        const std::size_t trisBefore = body.mesh.indices.size() / 3;
        std::string error;
        bool ok = false;
        std::string done;
        // Snap each point to the surface; the outward normal there (or -DIR).
        SculptMesh surface(body);
        std::optional<SurfaceIndex> nearest; // built on first use; the points are snapped before anything changes
        const float reach = std::max({diameter, width, ss.radius, 0.05f * length(body.bounds().max - body.bounds().min)});
        std::string offBody;
        auto onSurface = [&](const Vec3& p, Vec3* n) {
            Vec3 fn;
            if (!nearest) nearest.emplace(surface);
            const Vec3 q = nearest->closestPoint(p, &fn);
            if (n) *n = dir ? -normalize(*dir) : fn;
            if (length(q - p) > reach && offBody.empty())
                offBody = vecText(p) + fmt(" is %g mm away from ", length(q - p)) + body.name;
            return q;
        };
        if (body.mesh.indices.empty()) return out.fail(body.name + " has no surface to work on");
        doc.scene.pushUndo(toUpper(cmd) + " on " + body.name);
        if (cmd == "SCULPT") {
            if (refineOnly) {
                sculptRefine(body, refineDetail);
                ok = true;
                done = fmt("refined to %g mm edges", refineDetail);
            } else {
                if (!haveBrush) return doc.scene.undo(), out.fail("Name a brush (SCULPT LIST)");
                if (points.empty()) return doc.scene.undo(), out.fail("SCULPT needs a point on the surface: AT x,y,z");
                // A Snake Hook starts on the surface and pulls it out along the rest of the path.
                for (const Vec3& q : points) {
                    onSurface(q, nullptr);
                    if (ss.brush == SculptBrush::SnakeHook) break;
                }
                if (!offBody.empty()) return doc.scene.undo(), out.fail(offBody);
                sculptStroke(body, surface, points, ss, move.value_or(Vec3{}));
                ok = true;
                done = std::string(sculptBrushKey(ss.brush)) + (ss.invert ? " (inverted)" : "") +
                       fmt(" stroke, radius %g", ss.radius);
            }
        } else if (cmd == "HOLE") {
            if (points.size() != 1 || diameter <= 0) return doc.scene.undo(), usage();
            Vec3 n;
            const Vec3 p = onSurface(points[0], &n);
            if (!offBody.empty()) return doc.scene.undo(), out.fail(offBody);
            ok = drillHole(body, p, n, diameter, through ? 0.0f : depth, &error);
            done = fmt("hole %g mm", diameter) + (through || depth <= 0 ? " through" : fmt(", %g deep", depth));
        } else if (cmd == "POCKET") {
            if (points.size() != 1 || width <= 0 || (shape != "ball" && depth <= 0)) return doc.scene.undo(), usage();
            Vec3 n;
            const Vec3 p = onSurface(points[0], &n);
            if (!offBody.empty()) return doc.scene.undo(), out.fail(offBody);
            const PocketShape ps = shape == "round" || shape == "cylinder" ? PocketShape::Round
                                   : shape == "ball"                         ? PocketShape::Ball
                                                                             : PocketShape::Box;
            ok = cutPocket(body, p, n, ps, width, length_ > 0 ? length_ : width, depth, angle, &error);
            done = fmt("pocket %g x %g", width, length_ > 0 ? length_ : width) + fmt(", %g deep", depth);
        } else if (cmd == "CHANNEL") {
            if (points.size() < 2 || width <= 0) return doc.scene.undo(), usage();
            std::vector<Vec3> ps, ns;
            for (const Vec3& q : points) {
                Vec3 n;
                ps.push_back(onSurface(q, &n));
                ns.push_back(n);
            }
            if (!offBody.empty()) return doc.scene.undo(), out.fail(offBody);
            ok = cutChannel(body, ps, ns, width, depth, &error);
            done = fmt("channel %g wide", width);
        } else if (cmd == "TRIM") {
            if (points.size() != 1 || !normal) return doc.scene.undo(), usage();
            Vec3 n = normalize(*normal);
            if (smaller) {
                const MeshData w = worldMesh(body);
                if (volumeBeyondPlane(w, points[0], n) > volumeBeyondPlane(w, points[0], -n)) n = -n;
            }
            ok = trimByPlane(body, points[0], n, &error);
            done = "trimmed";
        } else { // BOSS
            if (points.size() != 1 || diameter <= 0) return doc.scene.undo(), usage();
            Vec3 n;
            const Vec3 p = onSurface(points[0], &n);
            if (!offBody.empty()) return doc.scene.undo(), out.fail(offBody);
            const BossShape bs = shape == "box" ? BossShape::Box : shape == "ball" ? BossShape::Ball : BossShape::Cylinder;
            ok = addBoss(body, p, n, bs, diameter, height > 0 ? height : diameter * 0.5f, &error);
            done = fmt("boss %g mm", diameter);
        }
        if (!ok) {
            doc.scene.undo();
            return out.fail(error.empty() ? "Nothing changed" : error);
        }
        doc.scene.touchBodies();
        markChanged(doc);
        const std::size_t open = openEdgeCount(body.mesh);
        out.say(body.name + ": " + done + fmt(", %g triangles", static_cast<double>(body.mesh.indices.size() / 3)) +
                fmt(" (was %g)", static_cast<double>(trisBefore)) + (open == 0 ? ", closed" : fmt(", %g open edges", static_cast<double>(open))));
        return out;
    }
    if (cmd == "SHELL" || cmd == "ARMOR" || cmd == "INTERFERENCE" || cmd == "JOINT" || cmd == "LIGHTEN") {
        static const char* const kKeys[] = {"thickness", "outward", "open", "normal", "clearance", "part", "step",
                                            "duration", "bodies", "to", "at", "axis", "hinge", "ball", "range", "merge",
                                            "period", "size", "skin", "cell", "wall", "hollow"};
        auto isKey = [&](const std::string& w) {
            const std::string k = lower(w);
            for (const char* q : kKeys)
                if (k == q) return true;
            return false;
        };
        std::size_t i = 0;
        std::vector<std::string> nameWords;
        if (quotedArg(0)) nameWords.push_back(a[i++]);
        else
            while (i < a.size() && !isKey(a[i])) nameWords.push_back(a[i++]);
        auto lengthAfter = [&](float& v) {
            if (i + 1 >= a.size()) return false;
            const auto l = parseLength(a[++i]);
            if (!l) return false;
            v = static_cast<float>(*l);
            return true;
        };
        auto numberAfter = [&](float& v) {
            if (i + 1 >= a.size()) return false;
            const auto n = parseNumber(a[++i]);
            if (!n) return false;
            v = *n;
            return true;
        };
        auto bodyNamed = [&](const std::vector<std::string>& words) {
            return words.empty() ? doc.selectedBody : findBody(doc, joinArgs(words));
        };

        if (cmd == "INTERFERENCE") {
            float clearance = 2.0f, step = 0.0f, duration = 0.0f;
            std::vector<std::size_t> set;
            for (; i < a.size(); ++i) {
                const std::string k = lower(a[i]);
                if (k == "clearance") {
                    if (!lengthAfter(clearance)) return out.fail("CLEARANCE needs a length");
                } else if (k == "step") {
                    if (!numberAfter(step) || step <= 0) return out.fail("STEP needs seconds");
                } else if (k == "duration") {
                    if (!numberAfter(duration) || duration < 0) return out.fail("DURATION needs seconds");
                } else if (k == "bodies") {
                    for (++i; i < a.size(); ++i) {
                        const int b = findBody(doc, a[i]);
                        if (b < 0) return out.fail("No body named " + a[i]);
                        if (std::find(set.begin(), set.end(), static_cast<std::size_t>(b)) == set.end()) set.push_back(static_cast<std::size_t>(b));
                    }
                } else {
                    return usage();
                }
            }
            const InterferenceReport r = checkInterference(doc.scene, set, clearance, step, duration);
            out.say(fmt("Interference over %.2f s (%g samples, ", r.duration, r.samples) +
                    std::to_string(r.checkedPairs) + " pairs), clearance " + lengthText(clearance) + ":");
            if (r.pairs.empty()) out.say("  all clear");
            if (r.restOverlaps > 0)
                out.say("  (" + std::to_string(r.restOverlaps) + " pairs already overlap in the rest pose, at joints, and get no deeper)");
            std::size_t shown = 0;
            for (const InterferencePair& p : r.pairs) {
                if (++shown > 40) {
                    out.say("  ... " + std::to_string(r.pairs.size() - 40) + " more");
                    break;
                }
                out.say(std::string("  ") + (p.distance < 0 ? "COLLIDES " : "too close ") + p.a + " / " + p.b +
                        (p.distance < 0 ? ": " + lengthText(-p.distance) + " deep" : ": " + lengthText(p.distance) + " gap") +
                        fmt(" at %.2f s", p.time) + " near " + vecText(p.at) +
                        (p.restDistance < 0 ? " (" + lengthText(-p.restDistance) + " at rest)" : ""));
            }
            return out;
        }

        if (cmd == "ARMOR") {
            float clearance = 3.0f, thickness = 3.0f;
            std::string partWord;
            bool merge = false;
            for (; i < a.size(); ++i) {
                const std::string k = lower(a[i]);
                if (k == "clearance") {
                    if (!lengthAfter(clearance) || clearance < 0) return out.fail("CLEARANCE needs a length");
                } else if (k == "thickness") {
                    if (!lengthAfter(thickness) || thickness <= 0) return out.fail("THICKNESS needs a length");
                } else if (k == "merge") {
                    merge = true;
                } else if (k == "part") {
                    std::vector<std::string> w;
                    // Words up to the next ARMOR option (so "ball" or "hinge" can be part words).
                    for (++i; i < a.size(); ++i) {
                        const std::string q = lower(a[i]);
                        if (!quotedArg(i) && (q == "clearance" || q == "thickness" || q == "merge")) break;
                        w.push_back(a[i]);
                    }
                    --i;
                    partWord = lower(joinArgs(w));
                    if (partWord.empty()) return out.fail("PART needs a word, e.g. PART Forearm");
                } else {
                    return usage();
                }
            }
            std::vector<std::size_t> sources;
            auto& bodies = doc.scene.bodies();
            if (!partWord.empty()) {
                const std::string robot = joinArgs(nameWords);
                // Several words, comma separated: "chest, heart, shoulder".
                std::vector<std::string> words;
                for (std::size_t q = 0; q <= partWord.size();) {
                    const std::size_t c = std::min(partWord.find(',', q), partWord.size());
                    std::string w = partWord.substr(q, c - q);
                    while (!w.empty() && w.front() == ' ') w.erase(w.begin());
                    while (!w.empty() && w.back() == ' ') w.pop_back();
                    if (!w.empty()) words.push_back(w);
                    q = c + 1;
                }
                for (std::size_t b = 0; b < bodies.size(); ++b) {
                    const std::string n = lower(bodies[b].name);
                    bool match = false;
                    for (const std::string& w : words) match = match || n.find(w) != std::string::npos;
                    // Motors are skipped unless asked for by name.
                    bool askedMotor = false;
                    for (const std::string& w : words) askedMotor = askedMotor || w.find("motor") != std::string::npos;
                    if (!bodies[b].visible || !match || (!askedMotor && n.find("motor") != std::string::npos) || n.rfind("armor ", 0) == 0)
                        continue;
                    if (!robot.empty() && !iequal(bodies[b].component, robot) && lower(bodies[b].name).rfind(lower(robot), 0) != 0)
                        continue;
                    sources.push_back(b);
                }
                if (sources.empty()) return out.fail("No visible part with '" + partWord + "' in its name");
            } else {
                const int b = bodyNamed(nameWords);
                if (b < 0 || b >= static_cast<int>(bodies.size()))
                    return out.fail(nameWords.empty() ? "Select a body first, or give its name" : "No body named " + joinArgs(nameWords));
                sources.push_back(static_cast<std::size_t>(b));
            }
            doc.scene.pushUndo("ARMOR");
            std::vector<Body> made;
            if (merge && sources.size() > 1) { // one shell around all of them, carried by the first
                Body all = bodies[sources.front()];
                all.mesh = worldMesh(all);
                all.offset = {};
                for (std::size_t q = 1; q < sources.size(); ++q) all.mesh.append(worldMesh(bodies[sources[q]]));
                all.mesh.transform(translate(-bodies[sources.front()].offset));
                all.offset = bodies[sources.front()].offset;
                all.name = bodies[sources.front()].name;
                sources = {sources.front()};
                std::string error;
                auto armour = armorFor(all, clearance, thickness, &error);
                if (!armour) {
                    doc.scene.undo();
                    return out.fail(error);
                }
                std::string name = armour->name;
                for (int n = 2; findBody(doc, name) >= 0; ++n) name = armour->name + " " + std::to_string(n);
                armour->name = name;
                armour->attachedTo = all.name;
                made.push_back(std::move(*armour));
                sources.clear();
            }
            for (std::size_t b : sources) {
                std::string error;
                auto armour = armorFor(bodies[b], clearance, thickness, &error);
                if (!armour) {
                    doc.scene.undo();
                    return out.fail(error);
                }
                // Unique name; moves with its part.
                std::string name = armour->name;
                for (int n = 2; findBody(doc, name) >= 0; ++n) name = armour->name + " " + std::to_string(n);
                armour->name = name;
                armour->attachedTo = bodies[b].name; // moves with its part, whatever drives it
                made.push_back(std::move(*armour));
            }
            for (Body& m : made) {
                out.say("Created " + m.name + fmt(": %g triangles, ", static_cast<double>(m.mesh.indices.size() / 3)) +
                        lengthText(clearance) + " clearance, " + lengthText(thickness) + " thick");
                doc.scene.add(std::move(m));
            }
            doc.selectedBody = static_cast<int>(doc.scene.bodies().size()) - 1;
            markChanged(doc);
            return out;
        }

        if (cmd == "JOINT") {
            // JOINT child [TO parent] AT x,y,z [AXIS ...] [HINGE|BALL] [RANGE lo hi] [PERIOD s] [SIZE d]
            std::vector<std::string> parentWords;
            std::optional<Vec3> at;
            Vec3 axis{0, 0, 1};
            bool ball = false;
            float lo = -45.0f, hi = 45.0f, period = 4.0f, size = 0.0f;
            for (; i < a.size(); ++i) {
                const std::string k = lower(a[i]);
                if (k == "to") {
                    if (quotedArg(i + 1)) {
                        parentWords.push_back(a[++i]);
                    } else {
                        for (++i; i < a.size() && !isKey(a[i]); ++i) parentWords.push_back(a[i]);
                        --i;
                    }
                } else if (k == "at") {
                    if (i + 1 >= a.size() || !(at = parseVec3(a[++i]))) return out.fail("AT needs x,y,z");
                } else if (k == "axis") {
                    if (i + 1 >= a.size()) return out.fail("AXIS needs X, Y, Z or x,y,z");
                    const std::string v = lower(a[++i]);
                    if (v == "x") axis = {1, 0, 0};
                    else if (v == "y") axis = {0, 1, 0};
                    else if (v == "z") axis = {0, 0, 1};
                    else if (const auto d = parseVec3(a[i]); d && length(*d) > 0) axis = normalize(*d);
                    else return out.fail("AXIS needs X, Y, Z or x,y,z");
                } else if (k == "hinge") {
                    ball = false;
                } else if (k == "ball") {
                    ball = true;
                } else if (k == "range") {
                    if (!numberAfter(lo) || !numberAfter(hi) || hi <= lo) return out.fail("RANGE needs lo hi degrees");
                } else if (k == "period") {
                    if (!numberAfter(period) || period <= 0) return out.fail("PERIOD needs seconds");
                } else if (k == "size") {
                    if (!lengthAfter(size) || size <= 0) return out.fail("SIZE needs a length");
                } else {
                    return usage();
                }
            }
            const int ci = bodyNamed(nameWords);
            if (ci < 0) return out.fail(nameWords.empty() ? "Name the part to articulate" : "No body named " + joinArgs(nameWords));
            if (!at) return out.fail("JOINT needs AT x,y,z (the pivot)");
            const int pi = parentWords.empty() ? -1 : findBody(doc, joinArgs(parentWords));
            if (!parentWords.empty() && pi < 0) return out.fail("No body named " + joinArgs(parentWords));
            if (pi == ci) return out.fail("A part cannot hinge on itself: TO names the part it is jointed to");
            doc.scene.pushUndo("JOINT");
            auto& bodies = doc.scene.bodies();
            const std::string child = bodies[static_cast<std::size_t>(ci)].name;
            const AABB cb = bodies[static_cast<std::size_t>(ci)].bounds();
            if (size <= 0) size = std::clamp(length(cb.max - cb.min) * 0.12f, 2.0f, 40.0f);
            // Hardware: a pin through the pivot along the axis, or a ball.
            Body pin = ball ? makeBody(child + " Ball Joint", "sphere", {size * 0.5f})
                            : meshBody(child + " Hinge Pin", cutterCylinder(*at - axis * size, *at + axis * size, size * 0.22f, 32));
            if (ball) pin.offset = *at - Vec3{0, 0, size * 0.5f};
            pin.color = {0.78f, 0.80f, 0.84f};
            // The motion: the child (and its armour) swings between the limits and back.
            Motion m;
            int n = 1;
            std::string name = (ball ? "Ball joint " : "Hinge ") + std::to_string(n);
            auto taken = [&](const std::string& q) {
                for (const Motion& x : doc.scene.motions())
                    if (iequal(x.name, q)) return true;
                return false;
            };
            while (taken(name)) name = (ball ? "Ball joint " : "Hinge ") + std::to_string(++n);
            m.name = name;
            m.kind = MotionKind::Circular;
            m.axis = axis;
            m.origin = *at;
            m.mode = MotionMode::Loop;
            m.keys = {{0.0f, 0.0f}, {period * 0.25f, hi}, {period * 0.75f, lo}, {period, 0.0f}};
            m.hasLimits = true;
            m.lo = lo;
            m.hi = hi;
            m.bodies.push_back(child);
            // The ball turns with the child; a hinge pin stays with the parent.
            if (ball) pin.attachedTo = child;
            else if (pi >= 0) pin.attachedTo = bodies[static_cast<std::size_t>(pi)].name;
            if (pi >= 0) {
                // A moving parent carries the child along.
                for (Motion& pm : doc.scene.motions())
                    if (std::find(pm.bodies.begin(), pm.bodies.end(), bodies[static_cast<std::size_t>(pi)].name) != pm.bodies.end() &&
                        std::find(pm.bodies.begin(), pm.bodies.end(), child) == pm.bodies.end())
                        pm.bodies.push_back(child);
            }
            const std::string pinName = pin.name;
            doc.scene.add(std::move(pin));
            doc.scene.motions().push_back(std::move(m));
            markChanged(doc);
            out.say(std::string(ball ? "Ball joint" : "Hinge") + " '" + name + "': " + child +
                    (pi >= 0 ? " on " + bodies[static_cast<std::size_t>(pi)].name : std::string()) + fmt(", %g", lo) +
                    fmt("..%g deg every %g s", hi, period) + " (" + pinName + ")");
            return out;
        }

        const int bi = bodyNamed(nameWords);
        if (bi < 0 || bi >= static_cast<int>(doc.scene.bodies().size()))
            return out.fail(nameWords.empty() ? "Select a body first, or give its name" : "No body named " + joinArgs(nameWords));
        Body& body = doc.scene.bodies()[static_cast<std::size_t>(bi)];
        std::string error;
        if (cmd == "SHELL") {
            ShellOptions o;
            for (; i < a.size(); ++i) {
                const std::string k = lower(a[i]);
                if (k == "thickness") {
                    if (!lengthAfter(o.thickness) || o.thickness <= 0) return out.fail("THICKNESS needs a length");
                } else if (k == "outward") {
                    o.outward = true;
                } else if (k == "open") {
                    const auto p = i + 1 < a.size() ? parseVec3(a[++i]) : std::nullopt;
                    if (!p) return out.fail("OPEN needs x,y,z");
                    o.open = true;
                    o.openPoint = *p;
                } else if (k == "normal") {
                    const auto n = i + 1 < a.size() ? parseVec3(a[++i]) : std::nullopt;
                    if (!n || length(*n) == 0) return out.fail("NORMAL needs x,y,z");
                    o.openNormal = normalize(*n);
                } else {
                    return usage();
                }
            }
            doc.scene.pushUndo("SHELL on " + body.name);
            if (!shellBody(body, o, &error)) {
                doc.scene.undo();
                return out.fail(error);
            }
            doc.scene.touchBodies();
            markChanged(doc);
            const Body& r = doc.scene.bodies()[static_cast<std::size_t>(bi)];
            out.say(r.name + ": shell " + lengthText(o.thickness) + (o.outward ? " outward" : "") + (o.open ? ", open" : "") +
                    fmt(", %g mm^3", meshVolume(worldMesh(r))) + (openEdgeCount(r.mesh) == 0 ? ", closed" : ""));
            return out;
        }
        // LIGHTEN
        LightenOptions o;
        for (; i < a.size(); ++i) {
            const std::string k = lower(a[i]);
            if (k == "skin") {
                if (!lengthAfter(o.skin) || o.skin <= 0) return out.fail("SKIN needs a length");
            } else if (k == "cell") {
                if (!lengthAfter(o.cell) || o.cell <= 0) return out.fail("CELL needs a length");
            } else if (k == "wall") {
                if (!lengthAfter(o.wall) || o.wall <= 0) return out.fail("WALL needs a length");
            } else if (k == "hollow") {
                o.hollow = true;
            } else {
                return usage();
            }
        }
        doc.scene.pushUndo("LIGHTEN on " + body.name);
        double v0 = 0, v1 = 0;
        if (!lightenBody(body, o, &error, &v0, &v1)) {
            doc.scene.undo();
            return out.fail(error);
        }
        doc.scene.touchBodies();
        markChanged(doc);
        out.say(body.name + (o.hollow ? ": hollowed" : ": gyroid lattice inside") + ", skin " + lengthText(o.skin) +
                fmt(", %g%% lighter", v0 > 0 ? std::round((1.0 - v1 / v0) * 1000.0) / 10.0 : 0.0) +
                fmt(" (%g triangles)", static_cast<double>(body.mesh.indices.size() / 3)));
        return out;
    }
    if (cmd == "BOOLEAN") {
        // BOOLEAN body UNION|SUBTRACT|INTERSECT other [KEEP]
        std::size_t opAt = a.size();
        for (std::size_t i = 0; i < a.size(); ++i)
            if (!quotedArg(i) && booleanOpFromKey(a[i])) {
                opAt = i;
                break;
            }
        if (opAt == 0 || opAt + 1 >= a.size()) return usage();
        bool keep = false;
        std::vector<std::string> other(a.begin() + static_cast<std::ptrdiff_t>(opAt) + 1, a.end());
        if (!other.empty() && iequal(other.back(), "keep") && !quotedArg(a.size() - 1)) {
            keep = true;
            other.pop_back();
        }
        const int ai = findBody(doc, joinArgs(std::vector<std::string>(a.begin(), a.begin() + static_cast<std::ptrdiff_t>(opAt))));
        const int bi = findBody(doc, joinArgs(other));
        if (ai < 0) return out.fail("No body named " + joinArgs(std::vector<std::string>(a.begin(), a.begin() + static_cast<std::ptrdiff_t>(opAt))));
        if (bi < 0) return out.fail("No body named " + joinArgs(other));
        if (ai == bi) return out.fail("Pick two different bodies");
        const BooleanOp op = *booleanOpFromKey(a[opAt]);
        doc.scene.pushUndo(std::string("BOOLEAN ") + booleanOpKey(op));
        auto& bodies = doc.scene.bodies();
        std::string error;
        if (!bodyBoolean(bodies[static_cast<std::size_t>(ai)], bodies[static_cast<std::size_t>(bi)], op, &error)) {
            doc.scene.undo();
            return out.fail(error);
        }
        const std::string an = bodies[static_cast<std::size_t>(ai)].name, bn = bodies[static_cast<std::size_t>(bi)].name;
        if (!keep) {
            doc.scene.remove(static_cast<std::size_t>(bi));
            if (doc.selectedBody == bi) doc.selectedBody = -1;
            else if (doc.selectedBody > bi) --doc.selectedBody;
        }
        doc.scene.touchBodies();
        markChanged(doc);
        const Body& r = doc.scene.bodies()[static_cast<std::size_t>(ai > bi && !keep ? ai - 1 : ai)];
        const std::size_t open = openEdgeCount(r.mesh);
        out.say(an + " " + booleanOpKey(op) + " " + bn + fmt(": %g triangles", static_cast<double>(r.mesh.indices.size() / 3)) +
                (open == 0 ? ", closed" : fmt(", %g open edges", static_cast<double>(open))) + (keep ? "" : " (" + bn + " used up)"));
        return out;
    }
    if (cmd == "REMESH") {
        // REMESH [body ["other" ...]] [VOXEL v] [BLEND b] [SMOOTH s]: the others are fused in and used up.
        RemeshOptions ro;
        std::vector<std::string> names, words;
        auto flush = [&] {
            if (!words.empty()) names.push_back(joinArgs(words));
            words.clear();
        };
        for (std::size_t i = 0; i < a.size(); ++i) {
            const std::string k = lower(a[i]);
            if (quotedArg(i)) {
                flush();
                names.push_back(a[i]);
            } else if (k == "voxel" || k == "blend" || k == "smooth") {
                flush();
                const auto v = i + 1 < a.size() ? parseLength(a[++i]) : std::nullopt;
                if (!v || *v < 0 || !std::isfinite(*v)) return out.fail(toUpper(k) + " needs a length");
                (k == "voxel" ? ro.voxel : k == "blend" ? ro.blend : ro.smooth) = static_cast<float>(*v);
            } else {
                words.push_back(a[i]);
            }
        }
        flush();
        std::vector<int> idx;
        if (names.empty()) idx.push_back(doc.selectedBody);
        for (const std::string& n : names) {
            const int b = findBody(doc, n);
            if (b < 0) return out.fail("No body named " + n);
            if (std::find(idx.begin(), idx.end(), b) == idx.end()) idx.push_back(b);
        }
        auto& bodies = doc.scene.bodies();
        if (idx[0] < 0 || idx[0] >= static_cast<int>(bodies.size())) return out.fail("Select a body first, or give its name");
        doc.scene.pushUndo("REMESH " + bodies[static_cast<std::size_t>(idx[0])].name);
        std::vector<const Body*> others;
        for (std::size_t k = 1; k < idx.size(); ++k) others.push_back(&bodies[static_cast<std::size_t>(idx[k])]);
        Body& target = bodies[static_cast<std::size_t>(idx[0])];
        const std::size_t trisBefore = target.mesh.indices.size() / 3;
        std::string error;
        if (!remeshBodies(target, others, ro, &error)) {
            doc.scene.undo();
            return out.fail(error);
        }
        const std::string name = target.name;
        const std::size_t tris = target.mesh.indices.size() / 3, open = openEdgeCount(target.mesh);
        // The fused parts go (highest index first, so the rest keep theirs).
        std::vector<int> gone(idx.begin() + 1, idx.end());
        std::sort(gone.rbegin(), gone.rend());
        for (const int g : gone) {
            doc.scene.remove(static_cast<std::size_t>(g));
            if (doc.selectedBody == g) doc.selectedBody = -1;
            else if (doc.selectedBody > g) --doc.selectedBody;
        }
        doc.scene.touchBodies();
        markChanged(doc);
        out.say(name + ": remeshed" + (gone.empty() ? "" : fmt(" with %g more", static_cast<double>(gone.size()))) +
                fmt(", %g triangles", static_cast<double>(tris)) + fmt(" (was %g)", static_cast<double>(trisBefore)) +
                (open == 0 ? ", closed" : fmt(", %g open edges", static_cast<double>(open))));
        return out;
    }
    if (cmd == "ROTATE") {
        // ROTATE [name] rx,ry,rz: the orientation, degrees about X, then Y, then Z, about the body's origin.
        if (a.empty()) return usage();
        const auto deg = parseVec3(a.back());
        if (!deg || !std::isfinite(deg->x) || !std::isfinite(deg->y) || !std::isfinite(deg->z)) return usage();
        const std::vector<std::string> words(a.begin(), a.end() - 1);
        const int bi = words.empty() ? doc.selectedBody : findBody(doc, joinArgs(words));
        if (bi < 0 || bi >= static_cast<int>(doc.scene.bodies().size()))
            return out.fail(words.empty() ? "Select a body first, or give its name" : "No body named " + joinArgs(words));
        Body& b = doc.scene.bodies()[static_cast<std::size_t>(bi)];
        if (!b.component.empty()) return out.fail(b.name + " is part of " + b.component + ": its pose comes from the component");
        b.setRotation(*deg);
        doc.scene.touchBodies();
        markChanged(doc);
        out.say("Rotated " + b.name + " to " + fmt("(%g, ", deg->x) + fmt("%g, ", deg->y) + fmt("%g) deg", deg->z));
        return out;
    }
    if (cmd == "SCALE" && !a.empty() && a.back().find(',') != std::string::npos) {
        // SCALE [name] sx,sy,sz: stretches the body along its own axes, about its origin.
        const auto k = parseVec3(a.back());
        if (!k || !(k->x > 0) || !(k->y > 0) || !(k->z > 0)) return out.fail("SCALE needs three positive factors: sx,sy,sz");
        const std::vector<std::string> words(a.begin(), a.end() - 1);
        const int bi = words.empty() ? doc.selectedBody : findBody(doc, joinArgs(words));
        if (bi < 0 || bi >= static_cast<int>(doc.scene.bodies().size()))
            return out.fail(words.empty() ? "Select a body first, or give its name" : "No body named " + joinArgs(words));
        Body& b = doc.scene.bodies()[static_cast<std::size_t>(bi)];
        if (!b.component.empty()) return out.fail(b.name + " is part of " + b.component + ": stretch a single body");
        doc.scene.pushUndo("SCALE " + b.name);
        b.mesh.transform(sicdm::scale(*k));
        b.kind = "mesh"; // no longer the primitive it was
        b.params.clear();
        doc.scene.touchBodies();
        markChanged(doc);
        const AABB box = b.bounds();
        out.say("Stretched " + b.name + " to " + fmt("%g x ", box.max.x - box.min.x) + fmt("%g x ", box.max.y - box.min.y) +
                fmt("%g mm", box.max.z - box.min.z));
        return out;
    }
    if (cmd == "SCALE") {
        // SCALE [name] factor | SCALE [name] TO size
        std::vector<std::string> words = a;
        bool absolute = false;
        std::optional<float> value;
        if (!words.empty()) value = parseNumber(words.back());
        if (!value || *value <= 0.0f) return usage();
        words.pop_back();
        if (!words.empty() && lower(words.back()) == "to") {
            absolute = true;
            words.pop_back();
        }
        const int bi = words.empty() ? doc.selectedBody : findBody(doc, joinArgs(words));
        if (bi < 0 || bi >= static_cast<int>(doc.scene.bodies().size()))
            return out.fail(words.empty() ? "Select a body first, or give its name" : "No body named " + joinArgs(words));
        auto& bodies = doc.scene.bodies();
        const Body& picked = bodies[static_cast<std::size_t>(bi)];
        const std::string comp = picked.component;
        std::vector<std::size_t> targets;
        for (std::size_t i = 0; i < bodies.size(); ++i)
            if (static_cast<int>(i) == bi || (!comp.empty() && bodies[i].component == comp)) targets.push_back(i);
        const float k = absolute ? *value / picked.scale : *value;
        // A component scales about its first body's x, y and its lowest point, so a part
        // standing on the ground (a humanoid on its feet) stays standing on it.
        Vec3 anchor = bodies[targets.front()].offset;
        if (!comp.empty()) {
            AABB box;
            for (std::size_t i : targets) box.expand(bodies[i].bounds());
            if (box.valid()) anchor.z = box.min.z;
        }
        auto inGroup = [&](const std::string& n) {
            for (std::size_t i : targets)
                if (bodies[i].name == n) return true;
            return false;
        };
        for (std::size_t i : targets) {
            Body& b = bodies[i];
            b.setScale(b.scale * k);
            b.offset = anchor + (b.offset - anchor) * k;
        }
        // Motions of the component scale with it (axis points, roll radii).
        for (Motion& m : doc.scene.motions()) {
            bool all = !m.bodies.empty();
            for (const std::string& n : m.bodies) all = all && n != "*" && inGroup(n);
            if (!all) continue;
            m.origin = anchor + (m.origin - anchor) * k;
            m.radius *= k;
        }
        doc.scene.touchBodies();
        markChanged(doc);
        out.say((comp.empty() ? picked.name : comp) + fmt(" scaled x%g (size %g)", k, bodies[targets.front()].scale));
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
        if (path.size() > 4 && lower(path.substr(path.size() - 4)) == ".3mf") {
            if (!export3mf(doc.scene, path, &err)) return out.fail(err);
            out.say("Exported visible bodies to " + path + " (3MF)");
            return out;
        }
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
        case FileKind::Step:
        case FileKind::Iges: {
            // Exact B-reps where bodies have them; other meshes as planar-faced B-reps.
            std::vector<StepPart> parts;
            std::size_t exact = 0, faceted = 0;
            for (const Body& b : doc.scene.bodies()) {
                if (!b.visible || b.mesh.indices.empty()) continue;
                StepPart p;
                p.name = b.name;
                p.assembly = b.assembly;
                if (const auto ex = exactBrep(b)) {
                    auto br = std::make_shared<Brep>(*ex);
                    Placement t;
                    t.t = decimal(b.offset);
                    br->transform(t);
                    p.brep = std::move(br);
                    ++exact;
                } else {
                    p.brep = meshToBrep(worldMesh(b));
                    ++faceted;
                }
                parts.push_back(std::move(p));
            }
            if (parts.empty()) return out.fail("No visible bodies to export");
            const bool iges = fileKindOf(path) == FileKind::Iges;
            if (!(iges ? writeIgesFile(parts, path, &err) : writeStepFile(parts, path, &err))) return out.fail(err);
            out.say("Exported " + std::to_string(parts.size()) + " bodies to " + path + (iges ? " (IGES 5.3 trimmed surfaces, mm): " : " (STEP AP214, mm): ") +
                    std::to_string(exact) + " exact, " + std::to_string(faceted) + " as planar facets");
            return out;
        }
        default: return out.fail("Export supports .stl .obj .3mf .step .igs .dxf .svg");
        }
    }
    if (cmd == "IMPORT") {
        if (a.empty()) return usage();
        std::vector<std::string> rest = a;
        std::optional<Vec2> at;
        std::optional<float> width;
        for (std::size_t i = 0; i < rest.size();) {
            if (iequal(rest[i], "width") && i + 1 < rest.size() && parseLength(rest[i + 1])) {
                width = static_cast<float>(*parseLength(rest[i + 1]));
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
            out.say("Imported image " + name + " on " + planeName(doc.state.plane) + "  " + lengthsText({w, h}, " x "));
            return out;
        }
        case FileKind::Step:
        case FileKind::Iges: {
            StepReadReport rep;
            const bool iges = fileKindOf(path) == FileKind::Iges;
            std::vector<StepPart> parts = iges ? readIgesFile(path, &rep, &err) : readStepFile(path, &rep, &err);
            if (parts.empty()) return out.fail(err.empty() ? "Nothing imported" : err);
            if (iges) { // IGES has no part names: the file's, numbered
                const std::string stem = fs::path(path).stem().string();
                for (std::size_t k = 0; k < parts.size(); ++k)
                    if (parts[k].name.rfind("Body ", 0) == 0 || parts[k].name.rfind("Solid ", 0) == 0 || parts[k].name == "Surfaces")
                        parts[k].name = parts[k].name == "Surfaces" ? stem + " surfaces" : parts.size() == 1 ? stem : stem + " " + std::to_string(k + 1);
            }
            std::size_t faces = 0, failed = 0, tris = 0;
            std::string names;
            for (StepPart& p : parts) {
                // Tessellated to a tolerance that suits the part's size.
                TessellateOptions o;
                MeshData probe = tessellate(*p.brep, TessellateOptions{1.0, 0.6, 0.0});
                const AABB box = probe.bounds();
                const double diag = box.valid() ? static_cast<double>(length(box.max - box.min)) : 100.0;
                o.chord = std::clamp(diag * 2e-4, 0.002, 0.25);
                TessellateReport tr;
                MeshData m = tessellate(*p.brep, o, &tr);
                faces += tr.faces;
                failed += tr.failedFaces;
                tris += m.indices.size() / 3;
                auto taken = [&](const std::string& n) {
                    for (const Body& other : doc.scene.bodies())
                        if (other.name == n) return true;
                    return false;
                };
                std::string name = p.name;
                for (int k = 2; taken(name); ++k) name = p.name + " " + std::to_string(k);
                Body b = meshBody(name, std::move(m), kPalette[static_cast<std::size_t>(++doc.primitiveSerial) % std::size(kPalette)]);
                p.brep->name = name;
                b.brep = p.brep;
                b.brepKey = meshKey(b.mesh);
                b.assembly = p.assembly;
                doc.scene.add(std::move(b));
                if (names.size() < 120) names += (names.empty() ? "" : ", ") + name;
            }
            doc.selectedBody = static_cast<int>(doc.scene.bodies().size()) - 1;
            doc.selectedCurve = doc.selectedImage = -1;
            markChanged(doc);
            char unit[64];
            std::snprintf(unit, sizeof(unit), "%g mm", rep.unitToMm);
            out.say("Imported " + std::to_string(parts.size()) + (parts.size() == 1 ? " part" : " parts") + " from " + path + " (" +
                    (rep.schema.empty() ? std::string("STEP") : rep.schema) + ", 1 file unit = " + unit + "): " + names);
            out.say("  " + std::to_string(faces) + " exact faces, " + std::to_string(tris) + " display triangles" +
                    (failed ? ", " + std::to_string(failed) + " faces could not be shown" : std::string()));
            for (std::size_t k = 0; k < rep.warnings.size() && k < 5; ++k) out.say("  warning: " + rep.warnings[k]);
            return out;
        }
        case FileKind::Project: return out.fail("Use OPEN for project files");
        default: return out.fail("Import supports .stl .obj .step .stp .igs .iges .dxf and images (.png .jpg .bmp .tga .gif ...)");
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
            s.pushUndo("ERASE " + gone); // a deleted body can be brought back
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
                const auto w = parseLength(a[i + 1]);
                if (!w || *w <= 0) return out.fail("WIDTH needs a positive length");
                img.width = static_cast<float>(*w);
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
        out.say(img.name + "  " + lengthsText({img.width, img.height()}, " x ") +
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
    if (cmd == "PANEL" && !a.empty()) {
        // PANEL MODEL | LIBRARY | CAM: open the docked panel on that page.
        const std::string page = toUpper(a[0]);
        const int p = page == "MODEL" ? 0 : page == "LIBRARY" ? 1 : page == "CAM" || page == "LATHE" ? 2 : page == "TOOLS" ? 3 : page == "MACHINE" ? 4 : -1;
        if (p >= 0) {
            if (auto o = needView(view, "PANEL"); !o.ok) return o;
            view->setPanelPage(p);
            out.say("Panel: " + page);
            return out;
        }
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
    if (cmd == "UNITS") {
        UnitSettings u = doc.state.units;
        bool changed = false;
        for (std::size_t i = 0; i < a.size(); ++i) {
            const std::string k = lower(a[i]);
            const bool hasNext = i + 1 < a.size();
            if (const auto unit = parseUnitName(k)) {
                u.unit = *unit;
            } else if (k == "precision" || k == "decimals" || k == "dp") {
                if (!hasNext) return out.fail("PRECISION needs 0-10 or AUTO");
                const std::string v = lower(a[++i]);
                const auto n = parseNumber(v);
                if (v == "auto" || v == "default") u.precision = -1;
                else if (n && *n >= 0 && *n <= 10 && *n == std::floor(*n)) u.precision = static_cast<int>(*n);
                else return out.fail("PRECISION needs 0-10 or AUTO");
            } else if (k == "fraction" || k == "fractional" || k == "fractions") {
                std::string v = hasNext ? lower(a[i + 1]) : "1/64";
                if (hasNext) ++i;
                if (v == "off" || v == "decimal" || v == "no") {
                    u.style = LengthStyle::Decimal;
                } else {
                    if (v == "on" || v == "yes") v = "1/64";
                    if (v.rfind("1/", 0) == 0) v = v.substr(2);
                    const auto d = parseNumber(v);
                    const int den = d ? static_cast<int>(*d) : 0;
                    if (den < 2 || den > 256 || (den & (den - 1)) != 0)
                        return out.fail("FRACTION needs 1/2, 1/4 ... 1/256, or OFF");
                    u.style = LengthStyle::Fractional;
                    u.denominator = den;
                }
            } else if (k == "decimal") {
                u.style = LengthStyle::Decimal;
            } else {
                return usage();
            }
            changed = true;
        }
        const UnitSettings& was = doc.state.units;
        const bool same = u.unit == was.unit && u.precision == was.precision && u.style == was.style && u.denominator == was.denominator;
        if (changed && !same) { // re-choosing the current units leaves the file unmodified
            doc.state.units = u;
            setActiveUnits(u);
            markChanged(doc);
        }
        out.say("Units: " + describeUnits(u) + "   resolution " +
                formatLength(u.resolutionMm(), UnitSettings{LengthUnit::Millimeter, 6}));
        if (!changed) out.say("  UNITS MM|CM|M|IN|FT|PT   PRECISION n|AUTO   FRACTION 1/n|OFF   (geometry is stored in mm)");
        return out;
    }
    if (cmd == "CONVERT") {
        // CONVERT length [TO unit]: the length may contain spaces ("5' 6 1/2\"").
        std::vector<std::string> words = a;
        std::optional<LengthUnit> to;
        if (words.size() >= 2 && (lower(words[words.size() - 2]) == "to" || lower(words[words.size() - 2]) == "in")) {
            to = parseUnitName(words.back());
            if (!to) return out.fail("Unknown unit " + words.back() + "   (MM CM M IN FT PT)");
            words.resize(words.size() - 2);
        }
        if (words.empty()) return usage();
        const auto mm = parseLength(joinArgs(words));
        if (!mm) return out.fail("Not a length: " + joinArgs(words));
        UnitSettings exact;
        exact.precision = 10;
        const std::string typed = joinArgs(words);
        if (to) {
            exact.unit = *to;
            out.say(typed + "  =  " + formatLength(*mm, exact));
            return out;
        }
        out.say(typed + "  =");
        for (const LengthUnitInfo& u : lengthUnits()) {
            exact.unit = u.unit;
            out.say("  " + formatLength(*mm, exact));
        }
        UnitSettings frac{LengthUnit::Foot, -1, LengthStyle::Fractional, 64};
        out.say("  " + formatLength(*mm, frac) + "   (to 1/64 in)");
        return out;
    }
    if (cmd == "HELP") {
        if (!a.empty()) {
            const CommandSpec* s = findCommand(a[0]);
            const Plugin* owner = nullptr;
            if (const PluginCommand* pcmd = s ? nullptr : findPluginCommand(a[0], &owner)) {
                out.say(pcmd->usage + (pcmd->help.empty() ? "" : "  -  " + pcmd->help) + "   (plugin " + owner->name + ")");
                return out;
            }
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
        for (const Plugin& p : plugins())
            for (const PluginCommand& c : p.commands) {
                char buf[512];
                std::snprintf(buf, sizeof(buf), "%-46s %s (plugin %s)", c.usage.c_str(), c.help.c_str(), p.name.c_str());
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

} // namespace

} // namespace sicdm

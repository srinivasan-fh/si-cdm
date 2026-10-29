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
#include "core/Preferences.hpp"
#include "core/RobotCommand.hpp"
#include "core/RobotExport.hpp"
#include "core/UpdateCheck.hpp"
#include "core/Notices.hpp"
#include "core/Plugin.hpp"
#include "core/AssemblyCommand.hpp"
#include "core/SketchEdit.hpp"
#include "core/Collide.hpp"
#include "core/ComponentCommand.hpp"
#include "core/FaceCommand.hpp"
#include "core/MeshImport.hpp"
#include "core/SectionView.hpp"
#include "core/WeldmentCommand.hpp"
#include "core/RouteCommand.hpp"
#include "core/FastenerCommand.hpp"
#include "core/ArmatureCommand.hpp"
#include "core/StackupCommand.hpp"
#include "core/StudyCommand.hpp"
#include "core/MoldCommand.hpp"
#include "core/Components.hpp"
#include "core/DrawingCommand.hpp"
#include "core/PdmCommand.hpp"
#include "core/FeatureCommand.hpp"
#include "core/SketchCommand.hpp"
#include "core/Iges.hpp"
#include "core/Recovery.hpp"
#include "core/Step.hpp"
#include "core/Brep.hpp"
#include "core/Print.hpp"
#include "core/Multires.hpp"
#include "core/Retopo.hpp"
#include "core/UvMap.hpp"
#include "core/ImageWrite.hpp"
#include "core/Sculpt.hpp"
#include "core/Wearable.hpp"
#include "core/Units.hpp"
#include "core/Features.hpp"
#include "core/HoleWizard.hpp"
#include "core/Variables.hpp"
#include "core/Translate.hpp"
#include "core/Render.hpp"
#include "core/Assembly.hpp"
#include "core/Drawing.hpp"
#include "core/Rig.hpp"
#include "core/Terrain.hpp"

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
    if (c.plane == SketchPlane::Custom) c.frame = doc.state.customFrame;
    const SketchCurve& added = doc.scene.sketch().add(std::move(c));
    doc.selectedCurve = static_cast<int>(doc.scene.sketch().curves().size()) - 1;
    doc.selectedBody = doc.selectedImage = -1;
    markChanged(doc);
    out.say("Added " + added.name + " on " + activePlaneName(doc.state) + "  " + added.measure());
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

namespace {

int findDatum(const ProjectState& st, const std::string& name) {
    for (std::size_t i = 0; i < st.datums.size(); ++i)
        if (iequal(st.datums[i].name, name)) return static_cast<int>(i);
    return -1;
}

// An axis across a plane with this normal: level (square to Z) unless the plane is about
// level itself, then along X, so the principal planes come out as Top, Front and Right.
Vec3 levelAxis(Vec3 n) {
    n = normalize(n);
    if (std::fabs(n.z) > 0.9f) return {1, 0, 0};
    return cross(Vec3{0, 0, 1}, n);
}

std::string frameText(const PlaneFrame& f) {
    auto dir = [](Vec3 v) {
        char b[96];
        std::snprintf(b, sizeof b, "(%.4g, %.4g, %.4g)", v.x, v.y, v.z);
        return std::string(b);
    };
    return "origin " + lengthsText({f.origin.x, f.origin.y, f.origin.z}, ", ", "(", ")") + "  normal " + dir(f.normal) + "  u " + dir(f.u);
}

} // namespace

// A point typed as x,y,z or a reference point's name.
std::optional<Vec3> pointOrRef(const ProjectState& st, const std::string& w) {
    if (w.find(',') != std::string::npos) return parseVec3(w);
    if (const DatumPoint* p = findRefPoint(st, w)) return p->at;
    return std::nullopt;
}

// A direction: X, Y, Z, x,y,z or a reference axis's name.
std::optional<Vec3> directionOrRef(const ProjectState& st, const std::string& w) {
    if (const auto v = axisFromWord(w)) return v;
    if (const DatumAxis* a = findRefAxis(st, w)) return a->dir;
    return std::nullopt;
}

// A plane by name: TOP, FRONT, RIGHT or a datum plane.
std::optional<PlaneFrame> planeNamed(const ProjectState& st, const std::string& w) {
    if (const auto p = planeFromKey(w); p && *p != SketchPlane::Custom) return planeFrame(*p);
    for (const DatumPlane& d : st.datums)
        if (iequal(d.name, w)) return d.frame;
    return std::nullopt;
}

// The eigenvector of a symmetric 3x3 matrix whose eigenvalue stands furthest from the other
// two: a round body's axis from its inertia (Jacobi rotations).
Vec3 distinctAxis(const double in[3][3]) {
    double m[3][3], v[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) m[i][j] = in[i][j];
    for (int sweep = 0; sweep < 50; ++sweep) {
        for (int p = 0; p < 2; ++p)
            for (int q = p + 1; q < 3; ++q) {
                if (std::fabs(m[p][q]) < 1e-15 * (std::fabs(m[p][p]) + std::fabs(m[q][q]) + 1e-300)) continue;
                const double th = 0.5 * std::atan2(2 * m[p][q], m[q][q] - m[p][p]), c = std::cos(th), sn = std::sin(th);
                for (int k = 0; k < 3; ++k) { // columns
                    const double a = m[k][p], b = m[k][q];
                    m[k][p] = c * a - sn * b, m[k][q] = sn * a + c * b;
                }
                for (int k = 0; k < 3; ++k) { // rows
                    const double a = m[p][k], b = m[q][k];
                    m[p][k] = c * a - sn * b, m[q][k] = sn * a + c * b;
                }
                for (int k = 0; k < 3; ++k) {
                    const double a = v[k][p], b = v[k][q];
                    v[k][p] = c * a - sn * b, v[k][q] = sn * a + c * b;
                }
            }
    }
    const double e[3] = {m[0][0], m[1][1], m[2][2]};
    int best = 0;
    double gap = -1;
    for (int i = 0; i < 3; ++i) {
        const double g = std::fabs(e[i] - 0.5 * (e[(i + 1) % 3] + e[(i + 2) % 3]));
        if (g > gap) gap = g, best = i;
    }
    return normalize(Vec3{static_cast<float>(v[0][best]), static_cast<float>(v[1][best]), static_cast<float>(v[2][best])});
}

// REFAXIS name THROUGH p p | AT p DIR d | PLANES a b | OF body; REFPOINT name AT p |
// CENTRE body; LIST, DELETE name.
CommandOutput runReferenceCommand(Document& doc, const std::string& cmd, const std::vector<std::string>& a) {
    CommandOutput out;
    ProjectState& st = doc.state;
    const bool axis = cmd == "REFAXIS";
    const char* usage = axis ? "Usage: REFAXIS name THROUGH p p | AT p DIR x,y,z | PLANES plane plane | OF body  |  REFAXIS LIST | REFAXIS DELETE name"
                             : "Usage: REFPOINT name AT x,y,z | CENTRE body  |  REFPOINT LIST | REFPOINT DELETE name";
    if (a.empty() || (a.size() == 1 && iequal(a[0], "list"))) {
        if (axis) {
            if (st.refAxes.empty()) out.say("No reference axes");
            for (const DatumAxis& x : st.refAxes) out.say("  " + x.name + "  through " + vecText(x.origin) + fmt("  along (%.4g, %.4g, %.4g)", x.dir.x, x.dir.y, x.dir.z));
        } else {
            if (st.refPoints.empty()) out.say("No reference points");
            for (const DatumPoint& x : st.refPoints) out.say("  " + x.name + "  " + vecText(x.at));
        }
        return out;
    }
    if (iequal(a[0], "delete")) {
        if (a.size() != 2) return out.fail(usage);
        const std::size_t before = axis ? st.refAxes.size() : st.refPoints.size();
        if (axis) std::erase_if(st.refAxes, [&](const DatumAxis& x) { return iequal(x.name, a[1]); });
        else std::erase_if(st.refPoints, [&](const DatumPoint& x) { return iequal(x.name, a[1]); });
        if ((axis ? st.refAxes.size() : st.refPoints.size()) == before) return out.fail(std::string("No reference ") + (axis ? "axis" : "point") + " named " + a[1]);
        markChanged(doc);
        out.say("Deleted " + a[1]);
        return out;
    }
    if (a.size() < 3) return out.fail(usage);
    const std::string name = a[0], how = lower(a[1]);
    if (findRefAxis(st, name) || findRefPoint(st, name) || planeNamed(st, name) || name.find(',') != std::string::npos)
        return out.fail("There is already a plane, axis or point named " + name);
    auto bodyAt = [&](const std::string& n) -> const Body* {
        const int bi = findBody(doc, n);
        return bi < 0 ? nullptr : &doc.scene.bodies()[static_cast<std::size_t>(bi)];
    };
    if (axis) {
        Vec3 o{}, d{};
        if (how == "through" && a.size() == 4) {
            const auto p = pointOrRef(st, a[2]), q = pointOrRef(st, a[3]);
            if (!p || !q) return out.fail("THROUGH takes two points: x,y,z or reference points");
            if (!(length(*q - *p) > 1e-6f)) return out.fail("The two points are the same");
            o = *p, d = normalize(*q - *p);
        } else if (how == "at" && a.size() == 5 && iequal(a[3], "dir")) {
            const auto p = pointOrRef(st, a[2]);
            const auto v = directionOrRef(st, a[4]);
            if (!p || !v) return out.fail(usage);
            o = *p, d = normalize(*v);
        } else if (how == "planes" && a.size() == 4) {
            const auto f1 = planeNamed(st, a[2]), f2 = planeNamed(st, a[3]);
            if (!f1 || !f2) return out.fail("PLANES takes two planes: TOP, FRONT, RIGHT or datum planes");
            const Vec3 n1 = f1->normal, n2 = f2->normal, u = cross(n1, n2);
            const float uu = dot(u, u);
            if (!(uu > 1e-8f)) return out.fail("Those planes are parallel: they don't meet in a line");
            const float d1 = dot(n1, f1->origin), d2 = dot(n2, f2->origin);
            o = (cross(n2, u) * d1 + cross(u, n1) * d2) / uu;
            d = normalize(u);
        } else if (how == "of" && a.size() == 3) {
            const Body* b = bodyAt(a[2]);
            if (!b) return out.fail("No body named " + a[2]);
            const SolidMass m = solidMass(worldMesh(*b), 1.0);
            if (!(m.volume > 0)) return out.fail(b->name + " has no volume to find an axis in");
            o = m.centroid, d = distinctAxis(m.inertia);
        } else return out.fail(usage);
        st.refAxes.push_back({name, o, d});
        markChanged(doc);
        out.say("Reference axis " + name + " through " + vecText(o) + fmt(" along (%.4g, %.4g, %.4g)", d.x, d.y, d.z));
        return out;
    }
    Vec3 p{};
    if (how == "at" && a.size() == 3) {
        const auto q = pointOrRef(st, a[2]);
        if (!q) return out.fail("AT needs x,y,z");
        p = *q;
    } else if ((how == "centre" || how == "center") && a.size() == 3) {
        const Body* b = bodyAt(a[2]);
        if (!b) return out.fail("No body named " + a[2]);
        const SolidMass m = solidMass(worldMesh(*b), 1.0);
        p = m.volume > 0 ? m.centroid : worldMesh(*b).bounds().center();
    } else return out.fail(usage);
    st.refPoints.push_back({name, p});
    markChanged(doc);
    out.say("Reference point " + name + " at " + vecText(p));
    return out;
}

// SPLIT [body] BY plane | SPLIT [body] [PLANE] x,y,z NORMAL x,y,z: the body cut in two by a
// plane, both pieces kept (the one the normal points to as "name (2)").
CommandOutput runSplitCommand(Document& doc, const std::vector<std::string>& a, const std::vector<bool>& quoted) {
    CommandOutput out;
    const char* usage = "Usage: SPLIT [body] BY plane | SPLIT [body] [PLANE] x,y,z NORMAL x,y,z";
    std::size_t i = 0;
    std::string name;
    auto isKey = [&](std::size_t k) {
        const std::string w = lower(a[k]);
        return !(k < quoted.size() && quoted[k]) && (w == "by" || w == "plane" || w == "normal" || a[k].find(',') != std::string::npos);
    };
    if (!a.empty() && !quoted.empty() && quoted[0]) name = a[i++];
    else
        while (i < a.size() && !isKey(i)) name += (name.empty() ? "" : " ") + a[i++];
    const int bi = name.empty() ? doc.selectedBody : findBody(doc, name);
    if (bi < 0 || bi >= static_cast<int>(doc.scene.bodies().size())) return out.fail(name.empty() ? "Select a body to split, or name it" : "No body named " + name);
    std::optional<Vec3> point, normal;
    for (; i < a.size(); ++i) {
        const std::string k = lower(a[i]);
        if (k == "by" && i + 1 < a.size()) {
            const auto f = planeNamed(doc.state, a[++i]);
            if (!f) return out.fail("BY takes a plane: TOP, FRONT, RIGHT or a datum plane");
            point = f->origin, normal = f->normal;
        } else if (k == "plane") continue;
        else if (k == "normal" && i + 1 < a.size()) {
            if (!(normal = directionOrRef(doc.state, a[++i]))) return out.fail("NORMAL needs X, Y, Z, x,y,z or a reference axis");
        } else if (!point && (point = pointOrRef(doc.state, a[i]))) continue;
        else return out.fail(usage);
    }
    if (!point || !normal) return out.fail(usage);
    Body& body = doc.scene.bodies()[static_cast<std::size_t>(bi)];
    Body other = body;
    std::string why;
    doc.scene.pushUndo("SPLIT " + body.name);
    const Vec3 n = normalize(*normal);
    if (!trimByPlane(body, *point, n, &why) || !trimByPlane(other, *point, -n, &why)) {
        doc.scene.undo();
        return out.fail("Cannot split " + doc.scene.bodies()[static_cast<std::size_t>(bi)].name + ": " + (why.empty() ? "the plane doesn't cut it" : why));
    }
    int k = 2;
    while (findBody(doc, body.name + " (" + std::to_string(k) + ")") >= 0) ++k;
    other.name = body.name + " (" + std::to_string(k) + ")";
    // The wizard's holes stay with the piece they're in.
    std::erase_if(body.holes, [&](const HoleNote& h) { return dot(h.at + body.offset - *point, n) > 0; });
    std::erase_if(other.holes, [&](const HoleNote& h) { return dot(h.at + other.offset - *point, n) <= 0; });
    const std::string kept = body.name, made = other.name;
    doc.scene.add(std::move(other));
    doc.scene.touchBodies();
    doc.selectedBody = static_cast<int>(doc.scene.bodies().size()) - 1;
    markChanged(doc);
    out.say("Split " + kept + " in two: " + kept + " and " + made);
    return out;
}

// PLANE TOP|FRONT|RIGHT|<datum>, PLANE OFFSET / ANGLE / THROUGH / AT / FACE (a new datum
// plane, made the sketch plane), PLANE LIST, PLANE DELETE name.
CommandOutput runPlaneCommand(Document& doc, const std::vector<std::string>& a, const std::vector<bool>& quoted) {
    CommandOutput out;
    ProjectState& st = doc.state;
    static const char* const kUsage =
        "Usage: PLANE TOP|FRONT|RIGHT|<datum>  |  PLANE OFFSET d [FROM plane]  |  PLANE ANGLE deg [ABOUT U|V] [FROM plane]  |  "
        "PLANE THROUGH p1 p2 p3  |  PLANE AT x,y,z NORMAL x,y,z [U x,y,z]  |  PLANE FACE x,y,z [ON body]  "
        "(any of them [NAME name])  |  PLANE LIST  |  PLANE DELETE name";
    if (a.empty()) return out.fail(kUsage);
    const bool isQuoted = !quoted.empty() && quoted[0];
    const std::string sub = isQuoted ? std::string() : lower(a[0]);
    auto activate = [&](SketchPlane p, const PlaneFrame& f, const std::string& name) {
        st.plane = p;
        st.customFrame = p == SketchPlane::Custom ? f : planeFrame(SketchPlane::Top);
        st.customPlane = p == SketchPlane::Custom ? name : std::string();
    };
    if (sub == "list") {
        out.say("Sketch plane: " + activePlaneName(st));
        if (st.datums.empty()) out.say("No datum planes (PLANE OFFSET / ANGLE / THROUGH / AT / FACE make them)");
        for (const DatumPlane& d : st.datums) out.say("  " + d.name + "  " + frameText(d.frame));
        return out;
    }
    if (sub == "delete") {
        if (a.size() != 2) return out.fail("Usage: PLANE DELETE name");
        const int k = findDatum(st, a[1]);
        if (k < 0) return out.fail("No datum plane named " + a[1]);
        const std::string name = st.datums[static_cast<std::size_t>(k)].name;
        st.datums.erase(st.datums.begin() + k);
        if (st.plane == SketchPlane::Custom && iequal(st.customPlane, name)) st.customPlane.clear(); // its sketches stay where they are
        markChanged(doc);
        out.say("Deleted datum plane " + name);
        return out;
    }
    const bool make = sub == "offset" || sub == "angle" || sub == "through" || sub == "at" || sub == "face";
    if (!make) {
        if (a.size() != 1) return out.fail(kUsage);
        if (!isQuoted)
            if (const auto p = planeFromKey(a[0])) {
                activate(*p, st.customFrame, st.customPlane);
                out.say("Sketch plane: " + activePlaneName(st));
                return out;
            }
        const int k = findDatum(st, a[0]);
        if (k < 0) return out.fail("No plane or datum plane named " + a[0] + "   (PLANE LIST shows them)");
        const DatumPlane& d = st.datums[static_cast<std::size_t>(k)];
        activate(SketchPlane::Custom, d.frame, d.name);
        out.say("Sketch plane: " + d.name);
        return out;
    }

    // A new datum plane. Optional words: FROM plane, ABOUT U|V, ON body, U x,y,z, NAME name.
    std::vector<std::string> v; // the values after the sub-command, options taken out
    std::string from, about = "u", onBody, name;
    std::optional<Vec3> uDir, normal;
    for (std::size_t i = 1; i < a.size(); ++i) {
        const std::string k = (i < quoted.size() && quoted[i]) ? std::string() : lower(a[i]);
        auto next = [&](const char* what) -> std::optional<std::string> {
            if (i + 1 >= a.size()) return std::nullopt;
            (void)what;
            return a[++i];
        };
        if (k == "from" || k == "about" || k == "on" || k == "name" || k == "normal" || (k == "u" && sub == "at")) {
            const auto w = next(k.c_str());
            if (!w) return out.fail(toUpper(k) + " needs a value");
            if (k == "from") from = *w;
            else if (k == "about") about = lower(*w);
            else if (k == "on") onBody = *w;
            else if (k == "name") name = *w;
            else if (k == "normal") {
                if (!(normal = directionOrRef(st, *w))) return out.fail("NORMAL needs X, Y, Z, x,y,z or a reference axis");
            } else if (!(uDir = directionOrRef(st, *w))) return out.fail("U needs X, Y, Z, x,y,z or a reference axis");
        } else {
            v.push_back(a[i]);
        }
    }
    if (!name.empty()) {
        if (planeFromKey(name) || findDatum(st, name) >= 0 || findRefAxis(st, name) || findRefPoint(st, name))
            return out.fail("There is already a plane, axis or point named " + name);
    } else {
        for (int n = static_cast<int>(st.datums.size()) + 1;; ++n)
            if (findDatum(st, name = "Plane " + std::to_string(n)) < 0) break;
    }
    // The plane OFFSET and ANGLE start from: the one named, else the sketch plane.
    PlaneFrame base = activePlaneFrame(st);
    if (!from.empty()) {
        if (const auto p = planeFromKey(from); p && *p != SketchPlane::Custom) base = planeFrame(*p);
        else if (const int k = findDatum(st, from); k >= 0) base = st.datums[static_cast<std::size_t>(k)].frame;
        else return out.fail("No plane or datum plane named " + from);
    }
    PlaneFrame f;
    if (sub == "offset") {
        const auto d = v.size() == 1 ? parseLength(v[0]) : std::nullopt;
        if (!d || !std::isfinite(*d)) return out.fail("Usage: PLANE OFFSET distance [FROM plane] [NAME name]");
        f = base;
        f.origin = base.origin + base.normal * static_cast<float>(*d);
    } else if (sub == "angle") {
        const auto deg = v.size() == 1 ? parseNumber(v[0]) : std::nullopt;
        if (!deg || !std::isfinite(*deg) || (about != "u" && about != "v"))
            return out.fail("Usage: PLANE ANGLE degrees [ABOUT U|V] [FROM plane] [NAME name]");
        const float t = radians(*deg), c = std::cos(t), sn = std::sin(t);
        f = base;
        if (about == "u") { // the plane tips about its own u axis: v rises towards the normal
            f.v = normalize(base.v * c + base.normal * sn);
            f.normal = normalize(base.normal * c - base.v * sn);
        } else {
            f.u = normalize(base.u * c - base.normal * sn);
            f.normal = normalize(base.normal * c + base.u * sn);
        }
    } else if (sub == "through") {
        if (v.size() != 3) return out.fail("Usage: PLANE THROUGH x,y,z x,y,z x,y,z [NAME name]");
        Vec3 p[3];
        for (int k = 0; k < 3; ++k) {
            const auto q = pointOrRef(st, v[static_cast<std::size_t>(k)]);
            if (!q) return out.fail("Not a point: " + v[static_cast<std::size_t>(k)]);
            p[k] = *q;
        }
        const Vec3 e1 = p[1] - p[0], e2 = p[2] - p[0];
        const Vec3 n = cross(e1, e2);
        if (!(length(n) > 1e-6f * std::max(1.0f, length(e1) * length(e2))) || !makePlaneFrame(p[0], n, e1, f))
            return out.fail("The three points are in line: they make no plane");
    } else if (sub == "at") {
        const auto o = v.size() == 1 ? pointOrRef(st, v[0]) : std::nullopt;
        if (!o || !normal) return out.fail("Usage: PLANE AT x,y,z NORMAL x,y,z [U x,y,z] [NAME name]");
        if (!makePlaneFrame(*o, *normal, uDir ? *uDir : levelAxis(*normal), f)) return out.fail("NORMAL needs a direction");
    } else { // face
        const auto at = v.size() == 1 ? pointOrRef(st, v[0]) : std::nullopt;
        if (!at) return out.fail("Usage: PLANE FACE x,y,z [ON body] [NAME name]   (the face of a body nearest the point)");
        int only = -1;
        if (!onBody.empty() && (only = findBody(doc, onBody)) < 0) return out.fail("No body named " + onBody);
        double best = 1e300;
        Vec3 bestPoint{}, bestNormal{};
        std::string bestBody;
        for (std::size_t bi = 0; bi < doc.scene.bodies().size(); ++bi) {
            const Body& b = doc.scene.bodies()[bi];
            if (only >= 0 ? static_cast<int>(bi) != only : !b.visible) continue;
            const MeshData m = worldMesh(b);
            for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
                const Vec3 p0 = m.vertices[m.indices[t]].position, p1 = m.vertices[m.indices[t + 1]].position, p2 = m.vertices[m.indices[t + 2]].position;
                const Vec3 n = cross(p1 - p0, p2 - p0);
                if (!(length(n) > 1e-12f)) continue;
                const Vec3 q = closestPointOnTriangle(*at, p0, p1, p2);
                const double d = length(q - *at);
                if (d < best) best = d, bestPoint = q, bestNormal = normalize(n), bestBody = b.name;
            }
        }
        if (bestBody.empty()) return out.fail(only >= 0 ? onBody + " has no faces" : "No visible body to put the plane on");
        if (!makePlaneFrame(bestPoint, bestNormal, levelAxis(bestNormal), f)) return out.fail("That face has no direction");
        out.say("On " + bestBody + "'s face at " + vecText(bestPoint));
    }
    if (!std::isfinite(f.origin.x + f.origin.y + f.origin.z + f.normal.x + f.normal.y + f.normal.z))
        return out.fail("That plane is out of range");
    st.datums.push_back({name, f});
    activate(SketchPlane::Custom, f, name);
    markChanged(doc);
    out.say("Datum plane " + name + ": " + frameText(f) + "   (now the sketch plane)");
    return out;
}


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
        // A choice as written first ("M1.6" is not "M16", though both squash to "m16").
        std::string list0 = std::string(p.choices) + "|";
        int index = 0;
        for (std::size_t at = 0, bar; (bar = list0.find('|', at)) != std::string::npos; at = bar + 1, ++index)
            if (iequal(list0.substr(at, bar - at), word)) return static_cast<float>(index);
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
    l.push_back("Project: " + doc.title() + "   sketch plane: " + activePlaneName(doc.state));
    for (const DatumPlane& d : doc.state.datums) l.push_back("  datum plane " + d.name);
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
                                          "CONSTRAIN", "DIM",         "CONSTRAINT", "DRAG",  "ERASE",     "SKETCHFILLET", "SKETCHCHAMFER",
                                          "SKETCHMIRROR", "SKETCHPATTERN", "PROJECT", "PATH3D"};
    for (const char* k : kSketch)
        if (cmd == k) return true;
    return false;
}

// Commands that add, remove, rebuild or move bodies. Those whose edit keeps no undo
// snapshot of its own get one around them (Scene::beginBodyEdit), so UNDO takes them back.
// Commands that may change what the scene's undo snapshots don't hold (see
// Document::JobsUndo): around them that state is compared, and kept for UNDO when changed.
bool sideCommand(const std::string& cmd) {
    static const char* const kSide[] = {"CAM",  "LATHE",  "MATE", "ASSEMBLY", "SHEET",   "VAR", "CONFIG", "PLANE", "REFAXIS",
                                        "REFPOINT", "RIG", "ARM",  "TERRAIN",  "PHYSICS", "SIM", "REGEN",  "PARAM", "PROPERTY", "EXPLODE", "STACK"};
    for (const char* k : kSide)
        if (cmd == k) return true;
    return false;
}

Document::JobsUndo sideState(const Document& doc) {
    Document::JobsUndo s;
    const ProjectState& st = doc.state;
    s.cam = camToJson(st.cam).dump(), s.lathe = latheToJson(st.lathe).dump();
    s.assembly = st.assembly, s.drawing = st.drawing;
    s.variables = st.variables, s.configs = st.configs, s.config = st.config;
    s.datums = st.datums, s.refAxes = st.refAxes, s.refPoints = st.refPoints;
    s.rigs = doc.scene.rigs(), s.physics = doc.scene.physics(), s.terrain = doc.scene.terrain();
    s.product = st.product;
    s.stacks = st.stacks;
    return s;
}

// The same state as one string, to tell whether a command changed it.
std::string sideKey(const Document& doc) {
    const ProjectState& st = doc.state;
    std::string k = camToJson(st.cam).dump() + latheToJson(st.lathe).dump() + assemblyToJson(st.assembly).dump() + drawingToJson(st.drawing).dump() +
                    rigStateToJson(doc.scene.rigs()).dump() + terrainToJson(doc.scene.terrain()).dump() + "|" + st.config;
    char buf[160];
    auto num = [&](double v) {
        std::snprintf(buf, sizeof buf, "%.9g,", v);
        k += buf;
    };
    auto vec = [&](const Vec3& v) { num(v.x), num(v.y), num(v.z); };
    for (const Variable& v : st.variables) k += v.name + "=" + v.expr + ";";
    for (const Configuration& c : st.configs) {
        k += c.name + "{";
        for (const auto& [n, e] : c.values) k += n + "=" + e + ";";
        k += "}";
    }
    for (const DatumPlane& d : st.datums) k += d.name, vec(d.frame.origin), vec(d.frame.normal), vec(d.frame.u);
    for (const DatumAxis& a : st.refAxes) k += a.name, vec(a.origin), vec(a.dir);
    for (const DatumPoint& p : st.refPoints) k += p.name, vec(p.at);
    const PhysicsSettings& ph = doc.scene.physics();
    num(ph.enabled), num(static_cast<double>(ph.gravity)), vec(ph.g), num(ph.bigG), num(ph.floor), num(ph.friction), num(ph.air), vec(ph.wind),
        vec(ph.electricField), vec(ph.magneticField), num(ph.ambient), num(ph.convection), num(ph.conductance), num(ph.relativity), num(ph.lightSpeed),
        num(ph.duration);
    const ProductData& pd = st.product;
    k += "|" + pd.revision + releaseStateName(pd.state) + pd.vaultItem + "#" + std::to_string(pd.history.size());
    for (const auto& [n, v] : pd.properties) k += n + "=" + v + ";";
    k += "|" + stacksToJson(st.stacks).dump();
    return k;
}

// Puts back a kept state; returns what was there (to go the other way), with the entry's
// label and flags and the given scene depth.
Document::JobsUndo swapSide(Document& doc, Document::JobsUndo s, std::size_t depth, bool* ok) {
    Document::JobsUndo cur = sideState(doc);
    cur.label = s.label, cur.sceneDepth = depth, cur.withScene = s.withScene, cur.history = s.history;
    const auto cj = Json::parse(s.cam), lj = Json::parse(s.lathe);
    CamJob c;
    LatheJob l;
    *ok = cj && lj && camFromJson(*cj, c) && latheFromJson(*lj, l);
    if (!*ok) return cur;
    ProjectState& st = doc.state;
    st.cam = std::move(c), st.lathe = std::move(l);
    st.assembly = std::move(s.assembly), st.drawing = std::move(s.drawing);
    st.variables = std::move(s.variables), st.configs = std::move(s.configs), st.config = std::move(s.config);
    st.datums = std::move(s.datums), st.refAxes = std::move(s.refAxes), st.refPoints = std::move(s.refPoints);
    doc.scene.rigs() = std::move(s.rigs), doc.scene.physics() = s.physics, doc.scene.terrain() = std::move(s.terrain);
    st.product = std::move(s.product);
    st.stacks = std::move(s.stacks);
    return cur;
}

bool bodyCommand(const std::string& cmd) {
    static const char* const kBody[] = {"BOX",     "CYLINDER", "CONE",   "SPHERE", "TORUS",   "WEDGE",      "PYRAMID",  "PRISM",
                                        "EXTRUDE", "REVOLVE",  "SWEEP",  "LOFT",   "PATTERN", "MIRROR",     "FILLET",   "CHAMFER",
                                        "HOLE",    "POCKET",   "CHANNEL", "TRIM",  "BOSS",    "SUBDIVIDE",  "SYMMETRIZE", "DECIMATE",
                                        "REPAIR",  "ORIENT",   "IMPORT", "PART",   "MOVE",    "ROTATE",     "SCALE",    "RIG",    "TERRAIN", "ARM",
                                        "DRAFT",   "THREAD",   "SPLIT",    "SHEETMETAL", "FEA", "DOME", "RIB", "EMBOSS", "ENGRAVE", "SURFACE", "THICKEN", "KNIT", "ANALYZE",
                                        "PARAM",   "REGEN",    "MATE",     "ASSEMBLY",   "MOTION", "POSE", "GESTURE", "PHYSICS", "VAR", "CONFIG", "MATERIAL", "PROPERTY", "COMPONENT", "FACE", "WELDMENT", "ROUTE", "MOLD", "FASTENER", "ARMATURE"};
    for (const char* k : kBody)
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
                                         "CONSTRUCTION", "EXTEND", "OFFSET", "DRAG", "SKETCHFILLET", "SKETCHCHAMFER", "SKETCHMIRROR",
                                         "SKETCHPATTERN", "PROJECT", "DRAFT", "THREAD", "REFAXIS", "REFPOINT", "SPLIT", "SHEETMETAL", "FEA", "DOME", "RIB", "EMBOSS", "ENGRAVE", "SURFACE", "THICKEN", "KNIT", "ANALYZE",
                                         "PAINT", "POLYGROUP", "LAYER", "MULTIRES", "RETOPO", "UV", "MATERIAL", "PROPERTY", "COMPONENT", "FACE", "WELDMENT", "PATH3D", "ROUTE", "MOLD", "FASTENER", "ARMATURE"};
    for (const char* k : kModel)
        if (cmd == k) return true;
    return false;
}

// Why a command can't run on a released or obsolete revision ("" when it can: it doesn't
// change the design, or it only asks). CAM, lathe and simulation runs use a released design.
std::string releasedRefusal(const Document& doc, const std::string& cmd, const ParsedCommand& pc) {
    const ProductData& pd = doc.state.product;
    if (!pd.locked()) return {};
    static const char* const kFree[] = {"REVISION", "PROPERTY", "VAULT", "COMPARE", "SELECT", "CAM", "LATHE", "SIM", "ARM"};
    for (const char* k : kFree)
        if (cmd == k) return {};
    const bool model = historyCommand(cmd) || bodyCommand(cmd) || sketchCommand(cmd) || sideCommand(cmd) || cmd == "HISTORY" || cmd == "UNDO" ||
                       cmd == "REDO" || cmd == "SHEET";
    if (!model) return {};
    // Asking: no arguments (MATERIAL, VAR, HISTORY... then show what there is), or a word that only asks.
    static const char* const kAskEmpty[] = {"MATERIAL", "VAR", "CONFIG", "MATE", "ASSEMBLY", "PHYSICS", "RIG", "TERRAIN", "SHEET", "LAYER", "POLYGROUP",
                                            "UV", "MULTIRES", "HISTORY", "PLANE", "REFAXIS", "REFPOINT", "EXPLODE", "FASTENER", "ARMATURE"};
    // (Only names, like MATERIAL "Bracket", ask about those.)
    const bool onlyNames = std::all_of(pc.quoted.begin(), pc.quoted.end(), [](bool q) { return q; }) && pc.quoted.size() == pc.args.size();
    if (pc.args.empty() || onlyNames)
        for (const char* k : kAskEmpty)
            if (cmd == k) return {};
    if (cmd == "PLANE" && pc.args.size() == 1) return {}; // only picks the sketch plane
    static const char* const kAsk[] = {"list", "info", "show", "report", "tree", "bom", "parts", "interference", "export", "check", "status", "dof", "hide", "pack", "open",
                                       "cutlist", "profiles", "bends", "sizes"};
    for (std::size_t i = 0; i < pc.args.size(); ++i) {
        if (i < pc.quoted.size() && pc.quoted[i]) continue;
        const std::string w = lower(pc.args[i]);
        for (const char* k : kAsk)
            if (w == k) return {};
    }
    return "Revision " + pd.revision + " is " + releaseStateName(pd.state) + ": it can't be changed (REVISION NEXT starts revision " + nextRevision(pd.revision) + ")";
}

void recordHistory(Document& doc, std::string_view line, const CommandOutput& out, bool startedEmpty, const std::string& selBody,
                   const std::string& selCurve, bool pushedUndo) {
    ProjectState& st = doc.state;
    if (!out.ok) return;
    const ParsedCommand pc = tokenizeCommand(line);
    const CommandSpec* spec = findCommand(pc.name);
    if (!spec) return;
    const std::string cmd = spec->name;
    // Rolled back: the model ends at the rollback point, where new steps go in.
    const bool rolledBack = st.rollback >= 0 && st.rollback < static_cast<int>(st.history.size());
    if (!rolledBack) st.rollback = -1;
    const std::size_t end = rolledBack ? static_cast<std::size_t>(st.rollback) : st.history.size();
    if (cmd == "NEW") {
        st.history.clear();
        st.historyRedo.clear();
        st.historyComplete = true;
        st.rollback = -1;
        return;
    }
    if ((cmd == "UNDO" || cmd == "REDO") && doc.undoSkipsHistory) return; // a CAM job, the sheet...: not a history step
    if (cmd == "UNDO") { // the last step an UNDO takes back goes to the redo list
        for (std::size_t i = end; i-- > 0;)
            if (st.history[i].undoable && !st.history[i].suppressed) {
                st.historyRedo.push_back(st.history[i]);
                st.history.erase(st.history.begin() + static_cast<std::ptrdiff_t>(i));
                if (rolledBack) --st.rollback;
                break;
            }
        return;
    }
    if (cmd == "REDO") {
        if (!st.historyRedo.empty()) {
            st.history.insert(st.history.begin() + static_cast<std::ptrdiff_t>(end), st.historyRedo.back());
            st.historyRedo.pop_back();
            if (rolledBack) ++st.rollback;
        }
        return;
    }
    if (!historyCommand(cmd)) return;
    if ((cmd == "PLANE" || cmd == "REFAXIS" || cmd == "REFPOINT") && !pc.args.empty() && iequal(pc.args[0], "list")) return; // only looks
    if (cmd == "SHEETMETAL" && !pc.args.empty() && (iequal(pc.args[0], "edges") || iequal(pc.args[0], "info"))) return;
    if (cmd == "WELDMENT" && !pc.args.empty() && (iequal(pc.args[0], "list") || iequal(pc.args[0], "cutlist") || iequal(pc.args[0], "profiles"))) return; // only looks
    if (cmd == "PATH3D" && !pc.args.empty() && iequal(pc.args[0], "info")) return;
    if (cmd == "ROUTE" && !pc.args.empty() && (iequal(pc.args[0], "list") || iequal(pc.args[0], "bends") || iequal(pc.args[0], "sizes"))) return;
    if (cmd == "FASTENER" && (pc.args.empty() || iequal(pc.args[0], "list") || iequal(pc.args[0], "sizes"))) return; // only looks
    if (cmd == "ARMATURE" && (pc.args.empty() || iequal(pc.args[0], "list") || (pc.args.size() == 1 && !iequal(pc.args[0], "new")) ||
                              (pc.args.size() > 1 && (iequal(pc.args[1], "list") || iequal(pc.args[1], "show") || iequal(pc.args[1], "hide")))))
        return; // only looks (or shows and hides)
    if ((cmd == "POLYGROUP" || cmd == "LAYER" || cmd == "MULTIRES") &&
        std::any_of(pc.args.begin(), pc.args.end(), [](const std::string& w) { return iequal(w, "list") || iequal(w, "show") || iequal(w, "hide"); }))
        return;
    if (cmd == "UV" && std::none_of(pc.args.begin(), pc.args.end(), [](const std::string& w) { return iequal(w, "unwrap") || iequal(w, "clear"); }))
        return; // looks, pictures and the view: not the model
    if (cmd == "FEA" && std::none_of(pc.args.begin(), pc.args.end(), [](const std::string& w) { return iequal(w, "plot") || iequal(w, "deform") || iequal(w, "mode"); }))
        return; // a report only: the history keeps analyses that leave a plot
    if (st.history.empty() && !startedEmpty) st.historyComplete = false; // it began on a model REGEN can't rebuild
    if (st.history.empty() && st.plane != SketchPlane::Top && cmd != "PLANE") { // the plane the history starts on
        HistoryStep plane;
        plane.line = "PLANE " + toUpper(planeKey(st.plane));
        plane.undoable = false;
        plane.unit = st.units.unit;
        if (st.plane == SketchPlane::Custom) { // the plane itself, in mm
            const PlaneFrame& f = st.customFrame;
            char buf[256];
            std::snprintf(buf, sizeof buf, "PLANE AT %.9g,%.9g,%.9g NORMAL %.9g,%.9g,%.9g U %.9g,%.9g,%.9g", f.origin.x, f.origin.y, f.origin.z,
                          f.normal.x, f.normal.y, f.normal.z, f.u.x, f.u.y, f.u.z);
            plane.line = buf;
            if (!st.customPlane.empty()) plane.line += " NAME \"" + st.customPlane + "\"";
            plane.millimetres = true;
        }
        st.history.push_back(std::move(plane));
    }
    HistoryStep step;
    step.line = std::string(line);
    step.undoable = pushedUndo; // only an edit with an undo snapshot is taken back by UNDO
    step.millimetres = doc.apiMillimeters;
    step.unit = st.units.unit;
    step.selectedBody = selBody;
    step.selectedCurve = selCurve;
    if (rolledBack) st.history.insert(st.history.begin() + st.rollback++, std::move(step));
    else st.history.push_back(std::move(step));
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
    const ParsedCommand parsed = tokenizeCommand(line);
    const CommandSpec* spec = findCommand(parsed.name);
    if (spec && !doc.replaying)
        if (const std::string why = releasedRefusal(doc, spec->name, parsed); !why.empty()) {
            CommandOutput refused;
            return refused.fail(why);
        }
    const bool sketchEdit = spec && sketchCommand(spec->name) && !doc.replaying;
    const std::uint64_t sketchBefore = doc.scene.sketch().revision();
    if (sketchEdit) doc.scene.pushSketchUndo(std::string(line.substr(0, std::min<std::size_t>(line.size(), 48))));
    // What the scene's snapshots don't hold (jobs, mates, sheet, variables, rigs...): as it
    // was, for UNDO.
    const bool sideEdit = spec && sideCommand(spec->name) && !doc.replaying;
    std::string sideBefore;
    Document::JobsUndo sideSnapshot;
    if (sideEdit) sideBefore = sideKey(doc), sideSnapshot = sideState(doc);
    doc.undoSkipsHistory = false;
    const bool bodyEdit = spec && bodyCommand(spec->name) && !doc.replaying;
    if (bodyEdit) doc.scene.beginBodyEdit(std::string(line.substr(0, std::min<std::size_t>(line.size(), 48))));
    CommandOutput out = runCommand(doc, line, view);
    // A member moved, turned or erased: the weldment's joints found again (in the same undo
    // step; nothing is remade where no end changed).
    if (out.ok && spec) {
        const std::string name = spec->name;
        if ((bodyCommand(name) || historyCommand(name)) && name != "WELDMENT") refreshWeldments(doc.scene);
    }
    if (bodyEdit) doc.scene.endBodyEdit();
    if (sketchEdit) {
        if (out.ok && doc.scene.sketch().revision() != sketchBefore) doc.scene.keepSketchUndo();
        else doc.scene.dropSketchUndo();
    }
    // An edit that kept an undo snapshot and succeeded: record the state it left.
    doc.scene.sealUndo();
    bool sidePushed = false;
    if (sideEdit && out.ok && sideKey(doc) != sideBefore) {
        sideSnapshot.label = std::string(line.substr(0, std::min<std::size_t>(line.size(), 48)));
        sideSnapshot.sceneDepth = doc.scene.undoDepth();
        sideSnapshot.withScene = doc.scene.undoPushes() != pushes;
        sideSnapshot.history = historyCommand(spec->name);
        doc.jobsUndo.push_back(std::move(sideSnapshot));
        if (doc.jobsUndo.size() > 64) doc.jobsUndo.erase(doc.jobsUndo.begin());
        doc.jobsRedo.clear();
        doc.scene.clearRedo(); // one history: a new edit drops what could be redone
        sidePushed = true;
    }
    if (out.ok && doc.scene.undoPushes() != pushes) doc.jobsRedo.clear(); // a new edit: nothing to redo
    if (!doc.replaying && !doc.historyRewritten)
        recordHistory(doc, line, out, empty, selBody, selCurve, doc.scene.undoPushes() != pushes || sidePushed);
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

// After the variables change: the model rebuilt from its history when the history uses them.
// False (with why in `out`) when the rebuild fails; the model is then as it was.
bool regenForVariables(Document& doc, CommandOutput& out) {
    const bool uses = std::any_of(doc.state.history.begin(), doc.state.history.end(), [](const HistoryStep& h) { return h.line.find('{') != std::string::npos; });
    if (!uses) return true;
    if (!doc.state.historyComplete) {
        out.say("The model was also changed outside commands: REGEN FORCE rebuilds it with the new values");
        return true;
    }
    CommandOutput r;
    if (!regenerateHistory(doc, doc.state.history, r)) {
        for (const std::string& l : r.lines) out.say(l);
        return false;
    }
    out.say("Rebuilt the model from its history (" + std::to_string(doc.state.history.size()) + " steps)");
    return true;
}

// VAR | VAR name = expression | VAR DELETE name;  CONFIG | CONFIG SAVE / DELETE name |
// CONFIG name | CONFIG TABLE file.csv
CommandOutput runVariableCommand(Document& doc, const std::string& cmd, const std::vector<std::string>& a) {
    CommandOutput out;
    ProjectState& st = doc.state;
    auto value = [&](const std::string& e) {
        std::string why;
        const auto v = evalExpression(e, st.variables, &why);
        return v ? fmt("%.10g", *v) : "(" + why + ")";
    };
    if (cmd == "VAR") {
        if (a.empty() || (a.size() == 1 && iequal(a[0], "list"))) {
            if (st.variables.empty()) out.say("No variables (VAR name = value makes one; commands use it as {name})");
            for (const Variable& v : st.variables) out.say("  " + v.name + " = " + v.expr + (parseNumber(v.expr) ? "" : "   = " + value(v.expr)));
            return out;
        }
        if (iequal(a[0], "delete")) {
            if (a.size() != 2) return out.fail("Usage: VAR DELETE name");
            const auto it = std::find_if(st.variables.begin(), st.variables.end(), [&](const Variable& v) { return iequal(v.name, a[1]); });
            if (it == st.variables.end()) return out.fail("No variable named " + a[1]);
            const std::vector<Variable> before = st.variables;
            const std::string gone = it->name;
            st.variables.erase(it);
            for (const HistoryStep& h : st.history) { // still needed by the history: keep it
                std::string ex, why;
                if (h.line.find('{') != std::string::npos && !expandVariables(h.line, st.variables, ex, &why)) {
                    st.variables = before;
                    return out.fail("The history still uses " + gone + ": " + h.line);
                }
            }
            for (const Variable& v : st.variables) { // and other variables
                std::string why;
                if (!evalExpression(v.expr, st.variables, &why)) {
                    st.variables = before;
                    return out.fail(v.name + " uses " + gone);
                }
            }
            doc.dirty = true;
            out.say("Deleted variable " + gone);
            return out;
        }
        // VAR name = expression (the = is optional)
        const std::string name = a[0];
        std::string expr;
        for (std::size_t k = 1; k < a.size(); ++k) {
            if (k == 1 && a[k] == "=") continue;
            expr += (expr.empty() ? "" : " ") + a[k];
        }
        if (!expr.empty() && expr[0] == '=') expr.erase(0, 1);
        if (!validVariableName(name)) return out.fail(name + " isn't a variable name: a letter, then letters, digits or _");
        if (expr.empty()) return out.fail("Usage: VAR name = expression");
        std::vector<Variable> trial = st.variables;
        auto it = std::find_if(trial.begin(), trial.end(), [&](const Variable& v) { return iequal(v.name, name); });
        const bool existed = it != trial.end();
        if (existed) it->expr = expr;
        else trial.push_back({name, expr});
        std::string why;
        // Every variable must still work out (no circles, no unknown names).
        for (const Variable& v : trial)
            if (!evalExpression(v.expr, trial, &why)) return out.fail(why);
        const std::vector<Variable> before = st.variables;
        st.variables = std::move(trial);
        doc.dirty = true;
        out.say(name + " = " + expr + (parseNumber(expr) ? "" : "   = " + value(expr)));
        if (existed && !regenForVariables(doc, out)) {
            st.variables = before;
            out.fail(name + " is as it was");
        }
        return out;
    }
    // CONFIG
    auto find = [&](const std::string& n) {
        return std::find_if(st.configs.begin(), st.configs.end(), [&](const Configuration& c) { return iequal(c.name, n); });
    };
    if (a.empty() || (a.size() == 1 && iequal(a[0], "list"))) {
        if (st.configs.empty()) out.say("No configurations (CONFIG SAVE name keeps the variables' values as one)");
        for (const Configuration& c : st.configs) {
            std::string vals;
            for (const auto& [k, e] : c.values) vals += (vals.empty() ? "" : ", ") + k + " = " + e;
            out.say(std::string(iequal(c.name, st.config) ? "* " : "  ") + c.name + ": " + vals);
        }
        return out;
    }
    const std::string sub = lower(a[0]);
    if (sub == "save" || sub == "delete") {
        if (a.size() != 2) return out.fail("Usage: CONFIG " + toUpper(sub) + " name");
        auto it = find(a[1]);
        if (sub == "delete") {
            if (it == st.configs.end()) return out.fail("No configuration named " + a[1]);
            out.say("Deleted configuration " + it->name);
            st.configs.erase(it);
            doc.dirty = true;
            return out;
        }
        if (st.variables.empty()) return out.fail("No variables to keep (VAR name = value)");
        Configuration c;
        c.name = a[1];
        for (const Variable& v : st.variables) c.values.push_back({v.name, v.expr});
        if (it != st.configs.end()) *it = c;
        else st.configs.push_back(c);
        st.config = c.name;
        doc.dirty = true;
        out.say("Configuration " + c.name + ": " + std::to_string(c.values.size()) + " values");
        return out;
    }
    if (sub == "table") {
        if (a.size() != 2) return out.fail("Usage: CONFIG TABLE file.csv  (first row: the configurations' column, then variable names)");
        std::vector<Configuration> rows;
        std::string why;
        if (!readDesignTable(a[1], rows, &why)) return out.fail(why);
        for (Configuration& r : rows) {
            auto it = find(r.name);
            if (it != st.configs.end()) *it = std::move(r);
            else st.configs.push_back(std::move(r));
        }
        doc.dirty = true;
        out.say("Design table: " + std::to_string(rows.size()) + " configurations (CONFIG name applies one)");
        return out;
    }
    if (a.size() != 1) return out.fail("Usage: CONFIG [LIST] | CONFIG name | CONFIG SAVE name | CONFIG DELETE name | CONFIG TABLE file.csv");
    auto it = find(a[0]);
    if (it == st.configs.end()) return out.fail("No configuration named " + a[0] + " (CONFIG LIST)");
    std::vector<Variable> trial = st.variables;
    for (const auto& [k, e] : it->values) {
        auto v = std::find_if(trial.begin(), trial.end(), [&](const Variable& x) { return iequal(x.name, k); });
        if (v != trial.end()) v->expr = e;
        else trial.push_back({k, e});
    }
    std::string why;
    for (const Variable& v : trial)
        if (!evalExpression(v.expr, trial, &why)) return out.fail("Configuration " + it->name + ": " + why);
    const std::vector<Variable> before = st.variables;
    const std::string beforeConfig = st.config;
    st.variables = std::move(trial);
    st.config = it->name;
    doc.dirty = true;
    out.say("Configuration " + it->name);
    if (!regenForVariables(doc, out)) {
        st.variables = before;
        st.config = beforeConfig;
        out.fail("The variables are as they were");
    }
    return out;
}

CommandOutput runCommand(Document& doc, std::string_view input, ViewHost* view) {
    CommandOutput out;
    // {expressions} of the project's variables become their values (VAR).
    std::string expanded;
    std::string_view line = input;
    if (input.find('{') != std::string_view::npos) {
        std::string why;
        if (!expandVariables(std::string(input), doc.state.variables, expanded, &why)) return out.fail(why);
        line = expanded;
    }
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
        // Cuts, joins, other depths, drafts and walls: see runFeatureCommand.
        for (std::size_t k = 0; k < a.size(); ++k) {
            if (quotedArg(k)) continue;
            const std::string l = lower(a[k]);
            if (l == "reverse" || l == "two" || l == "through" || l == "upto" || l == "draft" || l == "thin" || l == "cut" || l == "join")
                return runFeatureCommand(doc, cmd, a, pc.quoted);
        }
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
        b.params = extrudeParams(*profile, c.plane, *distance, symmetric, c.frame);
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
        doc.state.datums.clear();
        doc.state.refAxes.clear();
        doc.state.refPoints.clear();
        doc.state.customPlane.clear();
        doc.state.variables.clear(); // the old project's equations, configurations, product data and render set-up too
        doc.state.configs.clear();
        doc.state.config.clear();
        doc.state.product = ProductData{};
        doc.state.render = RenderSettings{};
        doc.state.section = SectionView{};
        if (doc.state.plane == SketchPlane::Custom) doc.state.plane = SketchPlane::Top;
        if (const auto u = parseUnitName(preferences().units)) doc.state.units.unit = *u; // Preferences: units for new projects
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
            noteRecentFile(path);
            out.say(fs::path(path).filename().string() + " is already open");
            return out;
        }
        doc.scene.assign(std::move(loaded));
        doc.jobsUndo.clear(), doc.jobsRedo.clear();
        doc.state = st;
        doc.path = fs::absolute(path).string();
        // Linked part files that changed since it was saved come in as they are now.
        std::vector<std::string> linkReport;
        reloadComponents(doc.scene, "", false, linkReport);
        doc.dirty = false;
        doc.clearSelection();
        doc.primitiveSerial = static_cast<int>(doc.scene.bodies().size());
        noteRecentFile(doc.path);
        out.documentReplaced = true;
        out.say("Opened " + doc.path);
        if (!warn.empty()) out.say(warn);
        for (const std::string& l : linkReport) out.say(l);
        return out;
    }
    if (cmd == "RECENT") {
        // RECENT [n | CLEAR]
        std::vector<std::string>& recent = preferences().recent;
        if (!a.empty() && lower(a[0]) == "clear") {
            recent.clear();
            std::string err;
            if (!savePreferences(&err)) return out.fail(err);
            out.say("Recent files cleared");
            return out;
        }
        if (!a.empty()) {
            const auto n = parseNumber(a[0]);
            if (!n || *n < 1 || *n > static_cast<double>(recent.size()) || *n != std::floor(*n))
                return out.fail(recent.empty() ? std::string("No recent files") : fmt("RECENT needs 1 to %g", static_cast<double>(recent.size())));
            const std::string path = recent[static_cast<std::size_t>(*n) - 1];
            std::error_code ec;
            if (!fs::exists(path, ec)) return out.fail(path + " is no longer there");
            return executeCommand(doc, "OPEN \"" + path + "\"", view);
        }
        if (recent.empty()) out.say("No recent files");
        for (std::size_t i = 0; i < recent.size(); ++i) {
            std::error_code ec;
            out.say(fmt("%g. ", static_cast<double>(i + 1)) + recent[i] + (fs::exists(recent[i], ec) ? "" : "  (missing)"));
        }
        return out;
    }
    if (cmd == "UPDATES") {
        // UPDATES [CHECK] [url]
        std::size_t i = !a.empty() && lower(a[0]) == "check" ? 1 : 0;
        const std::string url = i < a.size() ? a[i] : preferences().updateUrl;
        const UpdateResult r = checkForUpdate(url, SICDM_VERSION);
        if (!r.ok) return out.fail(r.describe(SICDM_VERSION));
        out.say(r.describe(SICDM_VERSION));
        if (r.newer && !r.latest.notes.empty()) out.say("  " + r.latest.notes);
        return out;
    }
    if (cmd == "PREFERENCES") {
        // PREFERENCES [SHOW] | SET name value | RESET
        const std::string sub = a.empty() ? "show" : lower(a[0]);
        std::string err;
        if (sub == "set") {
            if (a.size() < 2) return usage();
            if (!setPreference(preferences(), a[1], a.size() > 2 ? a[2] : std::string(), &err)) return out.fail(err);
            if (!savePreferences(&err)) return out.fail(err);
            if (lower(a[1]) == "language") setLanguage(preferences().language, &err); // shown at once
            for (const std::string& l : describePreferences(preferences()))
                if (lower(l).rfind(lower(a[1]) + ":", 0) == 0) out.say("Preference " + l);
            return out;
        }
        if (sub == "languages") { // the built-in and installed translations
            for (const LanguageInfo& l : languages()) out.say("  " + l.code + "  " + l.name + (l.code == language() ? "  (shown)" : ""));
            return out;
        }
        if (sub == "untranslated") { // what the language shown lacks, so far: to a language file to fill in
            const std::vector<std::string> words = untranslated();
            if (a.size() > 1) {
                Json strings = Json::object();
                for (const std::string& w : words) strings.set(w, "");
                if (!writeFile(a[1], Json::object().set("name", language()).set("strings", strings).dump(), &err)) return out.fail(err);
                out.say(fmt("%g words without a translation written to ", static_cast<double>(words.size())) + a[1]);
            } else {
                for (const std::string& w : words) out.say("  " + w);
                out.say(fmt("%g words without a translation", static_cast<double>(words.size())));
            }
            return out;
        }
        if (sub == "reset") {
            Preferences fresh;
            fresh.recent = preferences().recent; // the recent list isn't a setting
            preferences() = fresh;
            if (!savePreferences(&err)) return out.fail(err);
            setLanguage(preferences().language);
            out.say("Preferences reset to the defaults");
            return out;
        }
        if (sub != "show") return usage();
        out.say("Preferences (" + preferencesPath() + "):");
        for (const std::string& l : describePreferences(preferences())) out.say("  " + l);
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
        std::optional<std::string> contact;                  // CONTACT body | NONE
        std::optional<float> spring, preload, mass;
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
            } else if (k == "contact" || k == "rides" || k == "on_cam") {
                if (!hasNext) return out.fail("CONTACT needs the body it rides on (a cam), or NONE");
                contact = iequal(a[++i], "none") ? std::string() : a[i];
            } else if (k == "spring") {
                if (!num(spring) || *spring < 0.0f) return out.fail("SPRING needs a rate in N/mm (>= 0)");
            } else if (k == "preload") {
                if (!num(preload) || *preload < 0.0f) return out.fail("PRELOAD needs a force in N (>= 0)");
            } else if (k == "mass") {
                if (!num(mass) || *mass < 0.0f) return out.fail("MASS needs kg (>= 0; 0: from the bodies)");
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
        if (contact && !contact->empty()) {
            const int b = findBody(doc, *contact);
            if (b < 0) return out.fail("No body named " + *contact + " to ride on");
            const std::string driver = doc.scene.bodies()[static_cast<std::size_t>(b)].name;
            if (m.moves(driver)) return out.fail(driver + " can't ride on itself: CONTACT is the body it is pressed against");
            m.contact = driver;
            m.follow.clear();
            m.keys.clear();
        } else if (contact) {
            m.contact.clear();
        }
        if (spring) m.spring = *spring;
        if (preload) m.preload = *preload;
        if (mass) m.mass = *mass;
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
        if (sub == "wind") {
            // PHYSICS WIND x,y,z (m/s) | OFF: the air robots fly and drive through
            if (a.size() < 2) return usage();
            if (iequal(a[1], "off")) ps.wind = {};
            else if (const auto w = parseVec3(a[1])) ps.wind = *w;
            else return out.fail("WIND needs x,y,z in m/s, or OFF");
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
        if (sub == "check" || sub == "collisions" || sub == "clash") {
            // SIM CHECK [FROM t0] [TO t1] [STEP dt] [ALL] [body...]: parts that run into each other.
            float t0 = 0, t1 = doc.state.simDuration > 0 ? doc.state.simDuration : suggestedDuration(doc.scene), dt = 0;
            bool all = false;
            std::vector<std::size_t> only;
            for (std::size_t i = 1; i < a.size(); ++i) {
                const std::string w = lower(a[i]);
                const auto num = [&](float& v) {
                    const auto n = i + 1 < a.size() ? parseNumber(a[++i]) : std::nullopt;
                    if (!n || *n < 0) return false;
                    v = static_cast<float>(*n);
                    return true;
                };
                if (w == "from" && !pc.quoted[i]) {
                    if (!num(t0)) return out.fail("FROM needs a time (s)");
                } else if (w == "to" && !pc.quoted[i]) {
                    if (!num(t1)) return out.fail("TO needs a time (s)");
                } else if (w == "step" && !pc.quoted[i]) {
                    if (!num(dt) || dt <= 0) return out.fail("STEP needs a time (s) above 0");
                } else if (w == "all" && !pc.quoted[i]) {
                    all = true;
                } else {
                    const int bi = findBody(doc, a[i]);
                    if (bi < 0) return out.fail("No body named " + a[i]);
                    only.push_back(static_cast<std::size_t>(bi));
                }
            }
            if (t1 <= t0) return out.fail("Nothing moves: SIM CHECK FROM t0 TO t1 (or program a motion first)");
            const MotionCheck r = checkMotion(doc.scene, t0, t1, dt, all, only);
            out.say(fmt("Checked %.2f", t0) + fmt(" to %.2f s", t1) + " in " + std::to_string(r.steps) + " steps" +
                    (r.pairsLeftOut ? ", " + std::to_string(r.pairsLeftOut) + " pairs touching at the start left out (ALL keeps them)" : std::string()));
            if (r.clashes.empty()) {
                out.say("No parts run into each other");
                return out;
            }
            out.say(std::to_string(r.clashes.size()) + (r.clashes.size() == 1 ? " collision:" : " collisions:"));
            for (const MotionClash& c : r.clashes)
                out.say("  " + c.a + " hits " + c.b + fmt(" at %.2f s", c.from) + (c.to > c.from ? fmt(" (until %.2f s)", c.to) : std::string()) +
                        fmt(", near (%.1f,", c.at[0]) + fmt(" %.1f,", c.at[1]) + fmt(" %.1f) mm", c.at[2]));
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
    // A kept side state put back; what it replaces goes to `back` (with the scene's depth
    // there). Its scene step too, when the same command left one.
    auto swapJobs = [&](std::vector<Document::JobsUndo>& from, std::vector<Document::JobsUndo>& back, bool undo) {
        Document::JobsUndo j = std::move(from.back());
        from.pop_back();
        if (j.withScene && (undo ? doc.scene.undoDepth() : doc.scene.redoDepth()) == j.sceneDepth) {
            std::string why;
            if ((undo ? doc.scene.undo(&why) : doc.scene.redo(&why)).empty()) j.withScene = false; // the bodies changed since: the rest only
            if (doc.selectedBody >= static_cast<int>(doc.scene.bodies().size())) doc.selectedBody = -1;
        } else {
            j.withScene = false;
        }
        const std::string label = j.label;
        doc.undoSkipsHistory = !j.history;
        bool ok = false;
        Document::JobsUndo cur = swapSide(doc, std::move(j), undo ? doc.scene.redoDepth() : doc.scene.undoDepth(), &ok);
        if (!ok) return std::string();
        back.push_back(std::move(cur));
        markChanged(doc);
        return label;
    };
    if (cmd == "UNDO" && !doc.jobsUndo.empty() && doc.scene.undoDepth() <= doc.jobsUndo.back().sceneDepth) {
        const std::string what = swapJobs(doc.jobsUndo, doc.jobsRedo, true);
        if (what.empty()) return out.fail("Can't undo that change");
        out.say("Undid " + what);
        return out;
    }
    if (cmd == "REDO" && !doc.jobsRedo.empty() && doc.scene.redoDepth() <= doc.jobsRedo.back().sceneDepth) {
        const std::string what = swapJobs(doc.jobsRedo, doc.jobsUndo, false);
        if (what.empty()) return out.fail("Can't redo that change");
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
    if (cmd == "EXPLODE") return runExplodeCommand(doc, a, pc.quoted);
    if (cmd == "SECTION") return runSectionCommand(doc, a, pc.quoted);
    if (cmd == "WELDMENT") return runWeldmentCommand(doc, a, pc.quoted);
    if (cmd == "PATH3D") return runPath3dCommand(doc, a, pc.quoted);
    if (cmd == "ROUTE") return runRouteCommand(doc, a, pc.quoted);
    if (cmd == "MOLD") return runMoldCommand(doc, a, pc.quoted);
    if (cmd == "FASTENER") return runFastenerCommand(doc, a, pc.quoted);
    if (cmd == "ARMATURE") return runArmatureCommand(doc, a, pc.quoted);
    if (cmd == "STUDY") return runStudyCommand(doc, a, pc.quoted);
    if (cmd == "STACK") return runStackCommand(doc, a, pc.quoted);
    if (cmd == "FACE") return runFaceCommand(doc, a, pc.quoted);
    if (cmd == "PROPERTY" || cmd == "REVISION" || cmd == "COMPARE" || cmd == "VAULT") return runPdmCommand(doc, cmd, a, pc.quoted, view);
    if (cmd == "COMPONENT") {
        // Replaying the history: only what builds the model (not LIST, OPEN or PACK).
        const std::string sub = a.empty() ? std::string() : lower(a[0]);
        if (doc.replaying && (sub == "list" || sub == "open" || sub == "edit" || sub == "pack")) return out;
        return runComponentCommand(doc, a, pc.quoted, view);
    }
    if (cmd == "RIG" || cmd == "TERRAIN" || cmd == "ARM") return runRobotCommand(doc, cmd, a, pc.quoted);
    if (cmd == "CONSTRUCTION" || cmd == "EXTEND" || cmd == "OFFSET" || cmd == "DRAG" || cmd == "SKETCHFILLET" || cmd == "SKETCHCHAMFER" ||
        cmd == "SKETCHMIRROR" || cmd == "SKETCHPATTERN" || cmd == "PROJECT")
        return runSketchCommand(doc, cmd, a, pc.quoted);
    if (cmd == "REVOLVE" || cmd == "SWEEP" || cmd == "LOFT" || cmd == "PATTERN" || cmd == "MIRROR" || cmd == "FILLET" || cmd == "MASSPROPS" ||
        cmd == "MEASURE" || cmd == "CHAMFER" || cmd == "EDGES" || cmd == "DRAFT" || cmd == "THREAD" || cmd == "SHEETMETAL" || cmd == "FEA" || cmd == "DOME" || cmd == "RIB" || cmd == "EMBOSS" || cmd == "ENGRAVE" || cmd == "SURFACE" || cmd == "THICKEN" || cmd == "KNIT" || cmd == "ANALYZE")
        return runFeatureCommand(doc, cmd, a, pc.quoted);
    if (cmd == "HISTORY" || cmd == "REGEN") return runHistoryCommand(doc, cmd, a, pc.quoted);
    if (cmd == "VAR" || cmd == "CONFIG") return runVariableCommand(doc, cmd, a);
    if (cmd == "REFAXIS" || cmd == "REFPOINT") return runReferenceCommand(doc, cmd, a);
    if (cmd == "SPLIT") return runSplitCommand(doc, a, pc.quoted);
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
    if (cmd == "MATERIAL") {
        // MATERIAL [body] preset | COLOR c | METALLIC m | ROUGHNESS r | TRANSMISSION t | IOR n | EMISSION e
        //          | COLORMAP file | NORMALMAP file | DEFAULT | LIST
        if (!a.empty() && iequal(a[0], "list")) {
            for (const char* family : materialFamilies()) {
                out.say(std::string(family) + ":");
                for (const MaterialPreset& p : materialPresets())
                    if (p.family == family) out.say(std::string("  ") + p.name + "  " + p.description);
            }
            std::string finishes = "Finishes (FINISH name):";
            for (const MaterialFinish& f : materialFinishes()) finishes += std::string(" ") + f.name;
            out.say(finishes);
            return out;
        }
        static const char* const kWords[] = {"color", "colour", "metallic", "roughness", "transmission", "ior", "emission", "colormap", "normalmap", "default", "finish"};
        auto isWord = [&](std::size_t k) {
            if (quotedArg(k)) return false;
            const std::string w = lower(a[k]);
            for (const char* q : kWords)
                if (w == q) return true;
            MaterialProps probe;
            return applyMaterialPreset(w, probe, nullptr);
        };
        std::size_t i = 0;
        std::vector<std::string> nameWords;
        while (i < a.size() && !isWord(i)) nameWords.push_back(a[i++]);
        const int bi = nameWords.empty() ? doc.selectedBody : findBody(doc, joinArgs(nameWords));
        if (bi < 0 || bi >= static_cast<int>(doc.scene.bodies().size()))
            return out.fail(nameWords.empty() ? "Select a body first, or give its name" : "No body named " + joinArgs(nameWords));
        Body& b = doc.scene.bodies()[static_cast<std::size_t>(bi)];
        if (i >= a.size()) { // what it has
            const MaterialProps& m = b.material;
            out.say(b.name + ": " + (m.preset.empty() ? std::string("plastic (the default)") : m.preset) + fmt(", metallic %g", static_cast<double>(m.metallic)) +
                    fmt(", roughness %g", static_cast<double>(m.roughness)) +
                    (m.transmission > 0 ? fmt(", transmission %g", static_cast<double>(m.transmission)) + fmt(" (IOR %g)", static_cast<double>(m.ior)) : std::string()) +
                    (m.emission > 0 ? fmt(", emission %g", static_cast<double>(m.emission)) : std::string()) +
                    (m.colorMap.empty() ? std::string() : ", colour map " + m.colorMap) + (m.normalMap.empty() ? std::string() : ", normal map " + m.normalMap));
            return out;
        }
        MaterialProps m = b.material;
        Vec3 color = b.color;
        for (; i < a.size(); ++i) {
            const std::string w = lower(a[i]);
            const auto value = [&](float lo, float hi, float& v) {
                const auto n = i + 1 < a.size() ? parseNumber(a[++i]) : std::nullopt;
                if (!n || *n < lo || *n > hi) return false;
                v = *n;
                return true;
            };
            if (w == "color" || w == "colour") {
                if (i + 1 >= a.size()) return out.fail("COLOR needs a colour name, #RRGGBB or r,g,b");
                if (!parseColorText(a[++i], color)) return out.fail("Unknown colour " + a[i]);
            } else if (w == "metallic") {
                if (!value(0, 1, m.metallic)) return out.fail("METALLIC is 0 (plastic, paint) to 1 (metal)");
            } else if (w == "roughness") {
                if (!value(0, 1, m.roughness)) return out.fail("ROUGHNESS is 0 (a mirror) to 1 (matte)");
            } else if (w == "transmission") {
                if (!value(0, 1, m.transmission)) return out.fail("TRANSMISSION is 0 (opaque) to 1 (clear)");
            } else if (w == "ior") {
                if (!value(1, 4, m.ior)) return out.fail("IOR is 1 to 4 (glass 1.5, water 1.33)");
            } else if (w == "emission") {
                if (!value(0, 1000, m.emission)) return out.fail("EMISSION is 0 (none) or more (how bright it glows)");
            } else if (w == "colormap" || w == "normalmap") {
                if (i + 1 >= a.size()) return out.fail(toUpper(w) + " needs an image file, or NONE");
                const std::string f = a[++i];
                (w == "colormap" ? m.colorMap : m.normalMap) = iequal(f, "none") ? std::string() : f;
            } else if (w == "finish") {
                // FINISH polished | satin | brushed | matte: how rough the surface is.
                const std::string f = i + 1 < a.size() ? lower(a[++i]) : std::string();
                const MaterialFinish* found = nullptr;
                for (const MaterialFinish& x : materialFinishes())
                    if (f == x.name) found = &x;
                if (!found) return out.fail("FINISH is POLISHED, SATIN, BRUSHED or MATTE");
                m.roughness = found->roughness;
            } else if (w == "default") {
                m = MaterialProps{};
            } else if (!applyMaterialPreset(w, m, &color)) {
                return out.fail("Unknown material " + a[i] + " (MATERIAL LIST shows them)");
            }
            if (w == "metallic" || w == "roughness" || w == "transmission" || w == "ior" || w == "emission") m.preset = m.preset.empty() ? "custom" : m.preset;
        }
        b.material = m;
        b.color = color;
        doc.scene.touchBodies();
        out.say(b.name + ": " + (m.preset.empty() ? std::string("plastic") : m.preset) + fmt(", metallic %g", static_cast<double>(m.metallic)) +
                fmt(", roughness %g", static_cast<double>(m.roughness)) + (m.transmission > 0 ? std::string(", clear") : std::string()) +
                (m.emission > 0 ? std::string(", glowing") : std::string()));
        return out;
    }
    if (cmd == "RENDER") {
        // RENDER [file.png] [SIZE w h] [SAMPLES n] [BOUNCES b] [ENV studio|sky|white|dark] [SUN az el]
        //        [LIGHT x] [EXPOSURE ev] [FLOOR ON|OFF] [DENOISE ON|OFF] [VIEW ISO|FRONT|...]
        RenderSettings& rs = doc.state.render;
        std::string path;
        std::optional<StandardView> standard;
        bool window = false, start = false;
        for (std::size_t i = 0; i < a.size(); ++i) {
            const std::string w = lower(a[i]);
            const auto num = [&]() { return i + 1 < a.size() ? parseNumber(a[++i]) : std::nullopt; };
            auto onOff = [&](bool& v) {
                if (i + 1 >= a.size()) return false;
                const std::string x = lower(a[++i]);
                if (x == "on" || x == "yes") v = true;
                else if (x == "off" || x == "no") v = false;
                else return false;
                return true;
            };
            if (w == "size") {
                const auto x = num(), y = num();
                if (!x || !y || *x < 8 || *y < 8 || *x > 8192 || *y > 8192) return out.fail("SIZE is two numbers of pixels, 8 to 8192");
                rs.width = static_cast<int>(*x), rs.height = static_cast<int>(*y);
            } else if (w == "samples") {
                const auto n = num();
                if (!n || *n < 1 || *n > 65536) return out.fail("SAMPLES is 1 to 65536 a pixel");
                rs.samples = static_cast<int>(*n);
            } else if (w == "bounces") {
                const auto n = num();
                if (!n || *n < 1 || *n > 64) return out.fail("BOUNCES is 1 to 64");
                rs.bounces = static_cast<int>(*n);
            } else if (w == "env" || w == "environment") {
                const std::string e = i + 1 < a.size() ? lower(a[++i]) : std::string();
                if (std::find(renderEnvironments().begin(), renderEnvironments().end(), e) == renderEnvironments().end())
                    return out.fail("ENV is STUDIO, SKY, WHITE or DARK");
                rs.environment = e;
            } else if (w == "sun") {
                const auto az = num(), el = num();
                if (!az || !el || *el < 1 || *el > 90) return out.fail("SUN needs an azimuth and an elevation (1 to 90 degrees)");
                rs.sunAzimuth = *az, rs.sunElevation = *el;
            } else if (w == "light") {
                const auto n = num();
                if (!n || *n < 0 || *n > 100) return out.fail("LIGHT is 0 to 100 (1: as the environment is)");
                rs.light = *n;
            } else if (w == "exposure") {
                const auto n = num();
                if (!n || *n < -10 || *n > 10) return out.fail("EXPOSURE is -10 to 10 stops");
                rs.exposure = *n;
            } else if (w == "floor") {
                if (!onOff(rs.floor)) return out.fail("FLOOR is ON or OFF");
            } else if (w == "denoise") {
                if (!onOff(rs.denoise)) return out.fail("DENOISE is ON or OFF");
            } else if (w == "view") {
                const std::string v = i + 1 < a.size() ? lower(a[++i]) : std::string();
                if (v == "iso" || v == "isometric") standard = StandardView::Isometric;
                else if (v == "front") standard = StandardView::Front;
                else if (v == "back") standard = StandardView::Back;
                else if (v == "left") standard = StandardView::Left;
                else if (v == "right") standard = StandardView::Right;
                else if (v == "top") standard = StandardView::Top;
                else if (v == "bottom") standard = StandardView::Bottom;
                else if (v != "current") return out.fail("VIEW is ISO, FRONT, BACK, LEFT, RIGHT, TOP, BOTTOM or CURRENT");
            } else if (w == "window") {
                window = true;
            } else if (w == "start") {
                window = start = true;
            } else if (path.empty() && w.find('.') != std::string::npos) {
                path = a[i];
            } else {
                return usage();
            }
        }
        if (window) { // the render window (START: making the picture at once)
            if (!view) return out.fail("The render window needs the app window (RENDER file.png renders without it)");
            view->setPanelPage(start ? 9 : 8);
            if (path.empty()) return out;
        }
        if (path.empty()) {
            out.say("Render settings: " + rs.environment + fmt(" environment, %g x ", static_cast<double>(rs.width)) + fmt("%g", static_cast<double>(rs.height)) +
                    fmt(", %g samples", static_cast<double>(rs.samples)) + fmt(", %g bounces", static_cast<double>(rs.bounces)) +
                    (rs.floor ? ", floor" : ", no floor") + (rs.denoise ? ", denoised" : "") + "  (RENDER file.png makes the picture)");
            return out;
        }
        if (doc.replaying) return out; // regenerating the model doesn't write files
        const float aspect = static_cast<float>(rs.width) / static_cast<float>(rs.height);
        // The 3D window's view (at the picture's shape), else the one asked for (iso by default).
        RenderCamera rc;
        const OrbitCamera* vc = standard || !view ? nullptr : view->viewCamera();
        OrbitCamera cam = vc ? *vc : OrbitCamera{};
        cam.setAspect(aspect);
        if (!vc) {
            cam.setStandardView(standard.value_or(StandardView::Isometric), false);
            cam.fitToBounds(doc.scene.bounds(), false);
        }
        rc = renderCamera(cam);
        std::vector<std::uint8_t> px;
        RenderStats stats;
        std::string why;
        if (!renderImage(doc.scene, rs, rc, px, &stats, &why)) return out.fail("Cannot render: " + why);
        std::string err;
        if (!writeFile(path, encodePngCompressed(rs.width, rs.height, 3, px), &err)) return out.fail("Cannot write the picture: " + err);
        out.say(fmt("Rendered %g x ", static_cast<double>(rs.width)) + fmt("%g", static_cast<double>(rs.height)) + fmt(", %g samples a pixel", static_cast<double>(rs.samples)) +
                fmt(" in %.1f s -> ", stats.seconds) + path);
        return out;
    }
    if (cmd == "UV") {
        // UV [body] UNWRAP [ANGLE deg] [MARGIN m] [NOGROUPS] | CLEAR | INFO | LAYOUT file.png [SIZE n]
        //    | SHOW CHECKER | SHOW [COLOR file.png] [NORMAL file.png] | HIDE
        static const char* const kWords[] = {"unwrap", "clear", "info", "layout", "show", "hide"};
        auto isWord = [&](std::size_t k) {
            if (quotedArg(k)) return false;
            const std::string w = lower(a[k]);
            for (const char* q : kWords)
                if (w == q) return true;
            return false;
        };
        std::size_t i = 0;
        std::vector<std::string> nameWords;
        while (i < a.size() && !isWord(i)) nameWords.push_back(a[i++]);
        const int bi = nameWords.empty() ? doc.selectedBody : findBody(doc, joinArgs(nameWords));
        if (bi < 0 || bi >= static_cast<int>(doc.scene.bodies().size()))
            return out.fail(nameWords.empty() ? "Select a body first, or give its name" : "No body named " + joinArgs(nameWords));
        if (i >= a.size()) return usage();
        const std::string sub = lower(a[i++]);
        const std::string name = doc.scene.bodies()[static_cast<std::size_t>(bi)].name;
        if (sub == "info") {
            const MeshData& m = doc.scene.bodies()[static_cast<std::size_t>(bi)].mesh;
            if (!m.hasUvs()) {
                out.say(name + " has no texture coordinates");
                return out;
            }
            double used = 0;
            for (std::size_t k = 0; k + 2 < m.uvs.size(); k += 3) {
                const Vec2 p = m.uvs[k], q = m.uvs[k + 1], r = m.uvs[k + 2];
                used += 0.5 * std::fabs(static_cast<double>((q.x - p.x) * (r.y - p.y) - (r.x - p.x) * (q.y - p.y)));
            }
            out.say(name + fmt(": texture coordinates on %g faces", static_cast<double>(m.indices.size() / 3)) + fmt(", %.0f%% of the square used", 100.0 * used));
            return out;
        }
        if (sub == "show" || sub == "hide") { // the view only: not part of the model
            bool checker = false;
            std::string color, normal;
            for (; sub == "show" && i < a.size(); ++i) {
                const std::string w = lower(a[i]);
                if (w == "checker") checker = true;
                else if ((w == "color" || w == "colour") && i + 1 < a.size()) color = a[++i];
                else if (w == "normal" && i + 1 < a.size()) normal = a[++i];
                else return usage();
            }
            if (sub == "show" && !checker && color.empty() && normal.empty()) checker = true;
            if (sub == "show" && !doc.scene.bodies()[static_cast<std::size_t>(bi)].mesh.hasUvs())
                return out.fail(name + " has no texture coordinates (UV UNWRAP it first)");
            std::string err;
            if (!view || !view->setBodyTexture(name, checker, color, normal, &err)) return out.fail(err.empty() ? "Showing textures needs the app window" : err);
            out.say(sub == "hide" ? name + " shown plain" : name + " shown with " + (checker ? std::string("a checker") : std::string()) +
                                                               (!color.empty() ? std::string(checker ? ", " : "") + "colour " + color : std::string()) +
                                                               (!normal.empty() ? std::string(checker || !color.empty() ? ", " : "") + "normals " + normal : std::string()));
            return out;
        }
        if (sub == "layout") {
            if (i >= a.size()) return out.fail("LAYOUT needs a file name (.png)");
            const std::string path = a[i++];
            int size = 1024;
            if (i + 1 < a.size() && lower(a[i]) == "size") {
                const auto n = parseNumber(a[i + 1]);
                if (!n || *n < 16 || *n > 8192) return out.fail("SIZE is 16 to 8192 pixels");
                size = static_cast<int>(*n);
                i += 2;
            }
            if (i < a.size()) return usage();
            const MeshData& m = doc.scene.bodies()[static_cast<std::size_t>(bi)].mesh;
            if (!m.hasUvs()) return out.fail(name + " has no texture coordinates (UV UNWRAP it first)");
            if (doc.replaying) return out;
            std::string err;
            if (!writeFile(path, encodePngCompressed(size, size, 3, uvLayoutImage(m, size)), &err)) return out.fail("Cannot write the layout: " + err);
            out.say("UV layout of " + name + " written to " + path);
            return out;
        }
        if (sub == "clear") {
            if (i < a.size()) return usage();
            if (!doc.scene.bodies()[static_cast<std::size_t>(bi)].mesh.hasUvs()) return out.fail(name + " has no texture coordinates");
            doc.scene.pushUndo("UV CLEAR " + name);
            Body& b = doc.scene.bodies()[static_cast<std::size_t>(bi)];
            b.mesh.uvs.clear();
            doc.scene.touchBodies();
            out.say("Texture coordinates of " + name + " cleared");
            return out;
        }
        if (sub != "unwrap") return usage();
        UvOptions uo;
        for (; i < a.size(); ++i) {
            const std::string w = lower(a[i]);
            const auto value = [&]() { return i + 1 < a.size() ? parseNumber(a[++i]) : std::nullopt; };
            if (w == "angle") {
                const auto n = value();
                if (!n || *n < 5 || *n > 89) return out.fail("ANGLE is 5 to 89 degrees");
                uo.angle = *n;
            } else if (w == "margin") {
                const auto n = value();
                if (!n || *n < 0 || *n > 0.1f) return out.fail("MARGIN is 0 to 0.1 of the square");
                uo.margin = *n;
            } else if (w == "nogroups") uo.groups = false;
            else return usage();
        }
        MeshData m = doc.scene.bodies()[static_cast<std::size_t>(bi)].mesh;
        UvReport rep;
        std::string why;
        if (!unwrapUv(m, uo, &rep, &why)) return out.fail("Cannot unwrap " + name + ": " + why);
        doc.scene.pushUndo("UV UNWRAP " + name);
        Body& b = doc.scene.bodies()[static_cast<std::size_t>(bi)];
        b.mesh.uvs = std::move(m.uvs);
        doc.scene.touchBodies();
        out.say(name + fmt(" unwrapped: %g charts", static_cast<double>(rep.charts)) + fmt(", %.0f%% of the square used", 100.0 * static_cast<double>(rep.fill)) +
                fmt(", %.1f%% stretch", 100.0 * static_cast<double>(rep.stretch)) +
                (rep.projected ? fmt(" (%g charts projected flat)", static_cast<double>(rep.projected)) : std::string()));
        return out;
    }
    if (cmd == "BAKE") {
        // BAKE [body] NORMAL|OBJECTNORMAL|HEIGHT|OCCLUSION|COLOR [FROM "source"] [SIZE n] [RAYS n] [DISTANCE d] [PADDING px] FILE path.png
        auto isMap = [&](std::size_t k) {
            BakeMap m;
            return !quotedArg(k) && parseBakeMap(a[k], m);
        };
        std::size_t i = 0;
        std::vector<std::string> nameWords;
        while (i < a.size() && !isMap(i)) nameWords.push_back(a[i++]);
        const int bi = nameWords.empty() ? doc.selectedBody : findBody(doc, joinArgs(nameWords));
        if (bi < 0 || bi >= static_cast<int>(doc.scene.bodies().size()))
            return out.fail(nameWords.empty() ? "Select a body first, or give its name" : "No body named " + joinArgs(nameWords));
        if (i >= a.size()) return usage();
        BakeOptions bo;
        parseBakeMap(a[i++], bo.map);
        int si = bi;
        std::string path;
        for (; i < a.size(); ++i) {
            const std::string w = lower(a[i]);
            const auto value = [&]() { return i + 1 < a.size() ? parseNumber(a[++i]) : std::nullopt; };
            if (w == "from") {
                if (i + 1 >= a.size()) return out.fail("FROM needs the body to bake from");
                si = findBody(doc, a[++i]);
                if (si < 0) return out.fail("No body named " + a[i]);
            } else if (w == "size") {
                const auto n = value();
                if (!n || *n < 16 || *n > 8192) return out.fail("SIZE is 16 to 8192 pixels");
                bo.size = static_cast<int>(*n);
            } else if (w == "rays") {
                const auto n = value();
                if (!n || *n < 1 || *n > 1024) return out.fail("RAYS is 1 to 1024");
                bo.rays = static_cast<int>(*n);
            } else if (w == "distance") {
                const auto n = i + 1 < a.size() ? parseLength(a[++i]) : std::nullopt;
                if (!n || !(*n > 0)) return out.fail("DISTANCE needs a length");
                bo.distance = static_cast<float>(*n);
            } else if (w == "padding") {
                const auto n = value();
                if (!n || *n < 0 || *n > 64) return out.fail("PADDING is 0 to 64 pixels");
                bo.padding = static_cast<int>(*n);
            } else if (w == "file") {
                if (i + 1 >= a.size()) return out.fail("FILE needs a file name (.png)");
                path = a[++i];
            } else return usage();
        }
        if (path.empty()) return out.fail("BAKE needs FILE path.png");
        if (doc.replaying) return out; // regenerating the model doesn't write files
        const Body& target = doc.scene.bodies()[static_cast<std::size_t>(bi)];
        const Body& source = doc.scene.bodies()[static_cast<std::size_t>(si)];
        std::vector<std::uint8_t> px;
        int channels = 3, bits = 8;
        BakeReport rep;
        std::string why;
        if (!bakeMap(target, source, bo, px, channels, bits, &rep, &why)) return out.fail("Cannot bake: " + why);
        std::string err;
        bool written = false;
        if (bits == 16) {
            std::vector<std::uint16_t> wide(px.size() / 2);
            for (std::size_t k = 0; k < wide.size(); ++k) wide[k] = static_cast<std::uint16_t>((px[k * 2] << 8) | px[k * 2 + 1]);
            written = writePng16(path, bo.size, bo.size, 1, wide, &err);
        } else {
            written = writeFile(path, encodePngCompressed(bo.size, bo.size, channels, px), &err);
        }
        if (!written) return out.fail("Cannot write the map: " + err);
        std::string what = bakeMapName(bo.map);
        what[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(what[0])));
        out.say(what + " map of " + target.name + (si != bi ? " from " + source.name : std::string()) +
                fmt(", %g x ", static_cast<double>(bo.size)) + fmt("%g", static_cast<double>(bo.size)) + " -> " + path);
        if (bo.map == BakeMap::Height) out.say(fmt("  heights %.3g", static_cast<double>(rep.low)) + fmt(" to %.3g mm (black to white)", static_cast<double>(rep.high)));
        if (rep.missed > 0 && rep.texels > 0)
            out.say(fmt("  %.1f%% of the texels found nothing within the cage (the nearest along the normal taken; DISTANCE widens it)",
                        100.0 * static_cast<double>(rep.missed) / static_cast<double>(rep.texels)));
        return out;
    }
    if (cmd == "RETOPO") {
        // RETOPO [body] [FACES n] [ITERATIONS k] [NOCURVATURE] [DETAIL levels] [REPLACE]
        static const char* const kWords[] = {"faces", "iterations", "nocurvature", "detail", "replace"};
        auto isWord = [&](std::size_t k) {
            if (quotedArg(k)) return false;
            const std::string w = lower(a[k]);
            for (const char* q : kWords)
                if (w == q) return true;
            return false;
        };
        std::size_t i = 0;
        std::vector<std::string> nameWords;
        while (i < a.size() && !isWord(i)) nameWords.push_back(a[i++]);
        const int bi = nameWords.empty() ? doc.selectedBody : findBody(doc, joinArgs(nameWords));
        if (bi < 0 || bi >= static_cast<int>(doc.scene.bodies().size()))
            return out.fail(nameWords.empty() ? "Select a body first, or give its name" : "No body named " + joinArgs(nameWords));
        RetopoOptions ro;
        int detail = 0;
        bool replace = false;
        for (; i < a.size(); ++i) {
            const std::string w = lower(a[i]);
            auto count = [&](int lo, int hi, int& v) {
                const auto n = i + 1 < a.size() ? parseNumber(a[++i]) : std::nullopt;
                if (!n || *n < static_cast<float>(lo) || *n > static_cast<float>(hi)) return false;
                v = static_cast<int>(*n);
                return true;
            };
            if (w == "faces") {
                if (!count(50, 500000, ro.faces)) return out.fail("FACES is 50 to 500000 quads");
            } else if (w == "iterations") {
                if (!count(1, 30, ro.iterations)) return out.fail("ITERATIONS is 1 to 30");
            } else if (w == "nocurvature") ro.curvature = false;
            else if (w == "detail") {
                if (!count(1, 5, detail)) return out.fail("DETAIL is 1 to 5 levels");
            } else if (w == "replace") replace = true;
            else return usage();
        }
        const Body& src = doc.scene.bodies()[static_cast<std::size_t>(bi)];
        MeshData mesh;
        RetopoReport rep;
        std::string why;
        if (!retopologize(src, ro, mesh, &rep, &why)) return out.fail("Cannot retopologise " + src.name + ": " + why);
        doc.scene.pushUndo("RETOPO " + src.name);
        Body made = meshBody(src.name + " retopo", std::move(mesh), src.color);
        made.offset = doc.scene.bodies()[static_cast<std::size_t>(bi)].offset;
        for (int k = 2; findBody(doc, made.name) >= 0; ++k) made.name = src.name + " retopo " + std::to_string(k);
        if (detail > 0) { // levels over the quads, each laid on the original: its detail at the top
            std::string err;
            if (!multiresAdd(made, detail, &err)) return doc.scene.undo(), out.fail(err);
            projectLevelsOnto(made, doc.scene.bodies()[static_cast<std::size_t>(bi)]);
        }
        const std::string sourceName = doc.scene.bodies()[static_cast<std::size_t>(bi)].name;
        out.say(sourceName + fmt(" retopologised: %g quads", static_cast<double>(rep.quads)) + fmt(", edges about %.3g mm", static_cast<double>(rep.edge)) +
                fmt(", %.0f%% square", 100.0 * static_cast<double>(rep.quality)) + fmt(", %g vertices not on four quads", static_cast<double>(rep.irregular)) +
                (detail > 0 ? fmt(", %g levels of detail from the original", static_cast<double>(detail)) : std::string()));
        if (replace) {
            Body& b = doc.scene.bodies()[static_cast<std::size_t>(bi)];
            const std::string keep = b.name;
            b.mesh = std::move(made.mesh);
            b.multires = std::move(made.multires);
            b.kind = "mesh";
            b.params.clear();
            b.brep.reset();
            out.say("  " + keep + " now has the quads");
        } else {
            doc.scene.bodies()[static_cast<std::size_t>(bi)].visible = false; // the new mesh in its place
            const std::string nm = made.name;
            doc.scene.add(std::move(made));
            doc.selectedBody = static_cast<int>(doc.scene.bodies().size()) - 1;
            out.say("  -> " + nm + " (the original hidden)");
        }
        doc.scene.touchBodies();
        markChanged(doc);
        return out;
    }
    if (cmd == "MULTIRES") {
        // MULTIRES [body] ADD [n] | LEVEL n | UP | DOWN | APPLY | DELETE LOWER|HIGHER | LIST
        static const char* const kWords[] = {"add", "level", "up", "down", "apply", "delete", "list"};
        auto isWord = [&](std::size_t k) {
            if (quotedArg(k)) return false;
            const std::string w = lower(a[k]);
            for (const char* q : kWords)
                if (w == q) return true;
            return false;
        };
        std::size_t i = 0;
        std::vector<std::string> nameWords;
        while (i < a.size() && !isWord(i)) nameWords.push_back(a[i++]);
        const int bi = nameWords.empty() ? doc.selectedBody : findBody(doc, joinArgs(nameWords));
        if (bi < 0 || bi >= static_cast<int>(doc.scene.bodies().size()))
            return out.fail(nameWords.empty() ? "Select a body first, or give its name" : "No body named " + joinArgs(nameWords));
        if (i >= a.size()) return usage();
        const std::string op = lower(a[i++]);
        auto levelsText = [&](const Body& b) {
            if (!b.multires) return b.name + " has no levels";
            return b.name + fmt(": level %g", static_cast<double>(b.multires->current)) + fmt(" of 0 to %g", static_cast<double>(b.multires->levels.size() - 1)) +
                   fmt(", %g triangles shown", static_cast<double>(b.mesh.indices.size() / 3));
        };
        if (op == "list") {
            Body& b = doc.scene.bodies()[static_cast<std::size_t>(bi)];
            std::string why;
            if (b.multires && !multiresSync(b, &why)) out.say(why);
            out.say(levelsText(b));
            if (b.multires)
                for (std::size_t k = 0; k < b.multires->levels.size(); ++k)
                    out.say(fmt("  level %g: ", static_cast<double>(k)) + fmt("%g triangles", static_cast<double>(b.multires->levels[k].t.size())) +
                            (static_cast<int>(k) == b.multires->current ? "  (shown)" : ""));
            return out;
        }
        doc.scene.pushUndo("MULTIRES " + doc.scene.bodies()[static_cast<std::size_t>(bi)].name);
        Body& body = doc.scene.bodies()[static_cast<std::size_t>(bi)];
        const bool hadLevels = body.multires != nullptr;
        std::string why;
        bool ok = true;
        if (op == "add") {
            const auto n = i < a.size() ? parseNumber(a[i]) : std::nullopt;
            ok = multiresAdd(body, n ? std::clamp(static_cast<int>(*n), 1, 6) : 1, &why);
        } else if (op == "level") {
            const auto n = i < a.size() ? parseNumber(a[i]) : std::nullopt;
            if (!n) return doc.scene.undo(), out.fail("MULTIRES LEVEL n (0: the coarse cage)");
            ok = multiresLevel(body, static_cast<int>(*n), &why);
        } else if (op == "up" || op == "down") {
            if (!body.multires) ok = false, why = body.name + " has no levels (MULTIRES ADD makes them)";
            else ok = multiresLevel(body, body.multires->current + (op == "up" ? 1 : -1), &why);
        } else if (op == "apply") {
            ok = multiresApply(body, &why);
        } else if (op == "delete") {
            const std::string which = i < a.size() ? lower(a[i]) : "";
            if (which == "lower") ok = multiresDeleteLower(body, &why);
            else if (which == "higher") ok = multiresDeleteHigher(body, &why);
            else return doc.scene.undo(), out.fail("MULTIRES DELETE LOWER|HIGHER");
        } else return doc.scene.undo(), usage();
        if (!ok) {
            // A sync that found the triangles changed has dropped the levels: that stays.
            const bool dropped = hadLevels && !doc.scene.bodies()[static_cast<std::size_t>(bi)].multires;
            if (!dropped) doc.scene.undo();
            return out.fail(why);
        }
        doc.scene.touchBodies();
        markChanged(doc);
        out.say(op == "apply" ? body.name + ": levels dropped, the shown shape kept" : levelsText(body));
        return out;
    }
    if (cmd == "LAYER") {
        // LAYER [body] NEW name [RECORD] | RECORD name | STOP | STRENGTH name s | ON name | OFF name | DELETE name | BAKE name|ALL | LIST
        static const char* const kWords[] = {"new", "add", "record", "stop", "strength", "on", "off", "delete", "remove", "bake", "merge", "list"};
        auto isWord = [&](std::size_t k) {
            if (quotedArg(k)) return false;
            const std::string w = lower(a[k]);
            for (const char* q : kWords)
                if (w == q) return true;
            return false;
        };
        std::size_t i = 0;
        std::vector<std::string> nameWords;
        if (!a.empty() && quotedArg(0) && a.size() > 1 && isWord(1)) nameWords.push_back(a[i++]);
        else
            while (i < a.size() && !isWord(i)) nameWords.push_back(a[i++]);
        const int bi = nameWords.empty() ? doc.selectedBody : findBody(doc, joinArgs(nameWords));
        if (bi < 0 || bi >= static_cast<int>(doc.scene.bodies().size()))
            return out.fail(nameWords.empty() ? "Select a body first, or give its name" : "No body named " + joinArgs(nameWords));
        if (i >= a.size()) return usage();
        const std::string op = lower(a[i++]);
        Body* body = &doc.scene.bodies()[static_cast<std::size_t>(bi)];
        if (body->mesh.indices.empty()) return out.fail(body->name + " has no surface");
        if (op == "list") {
            if (!body->mesh.hasLayers()) out.say(body->name + " has no sculpt layers (LAYER NEW name RECORD starts one)");
            for (std::size_t k = 0; k < body->mesh.layers.size() && body->mesh.hasLayers(); ++k) {
                const SculptLayer& l = body->mesh.layers[k];
                float most = 0;
                for (const Vec3& o : l.offset) most = std::max(most, length(o));
                out.say("  " + l.name + fmt(": strength %g", static_cast<double>(l.strength)) + fmt(", moves up to %.3g mm", static_cast<double>(most)) +
                        (static_cast<int>(k) == body->mesh.recordLayer ? "  (recording)" : ""));
            }
            return out;
        }
        const std::string layer = i < a.size() ? a[i] : std::string();
        doc.scene.pushUndo("LAYER " + body->name);
        body = &doc.scene.bodies()[static_cast<std::size_t>(bi)];
        std::string why, done;
        bool ok = true;
        if (op == "new" || op == "add") {
            const bool rec = i + 1 < a.size() && iequal(a[i + 1], "record");
            ok = layerAdd(*body, layer, rec, &why);
            done = "layer " + layer + " added" + (rec ? ", recording: strokes go into it" : "");
        } else if (op == "record") {
            ok = layerRecord(*body, layer, &why);
            done = "strokes now go into layer " + layer;
        } else if (op == "stop") {
            ok = layerRecord(*body, "", &why);
            done = "no layer recording";
        } else if (op == "strength") {
            const auto v = i + 1 < a.size() ? parseNumber(a[i + 1]) : std::nullopt;
            if (!v) return doc.scene.undo(), out.fail("LAYER STRENGTH name s (1: as sculpted, 0: off, 2: twice as strong, -1: the other way)");
            ok = layerStrength(*body, layer, static_cast<float>(*v), &why);
            done = "layer " + layer + fmt(" at strength %g", *v);
        } else if (op == "on" || op == "off") {
            float to = 0.0f;
            if (op == "on") {
                for (const SculptLayer& l : body->mesh.layers)
                    if (iequal(l.name, layer)) to = l.saved > 0.0f ? l.saved : 1.0f;
            }
            ok = layerStrength(*body, layer, to, &why);
            done = "layer " + layer + (op == "on" ? " on" : " off");
        } else if (op == "delete" || op == "remove") {
            ok = layerDelete(*body, layer, &why);
            done = "layer " + layer + " deleted (its sculpting taken out)";
        } else if (op == "bake" || op == "merge") {
            ok = layerBake(*body, iequal(layer, "all") ? std::string() : layer, &why);
            done = iequal(layer, "all") || layer.empty() ? "all layers baked into the shape" : "layer " + layer + " baked into the shape";
        } else {
            doc.scene.undo();
            return usage();
        }
        if (!ok) {
            doc.scene.undo();
            return out.fail(why);
        }
        doc.scene.touchBodies();
        markChanged(doc);
        out.say(doc.scene.bodies()[static_cast<std::size_t>(bi)].name + ": " + done);
        return out;
    }
    if (cmd == "PAINT" || cmd == "POLYGROUP") {
        // PAINT [body] FILL colour [GROUP n] | CLEAR      (the PAINT brush: SCULPT [body] PAINT AT ... COLOR c)
        // POLYGROUP [body] AUTO [ANGLE deg] | ISLANDS | MASK | PAINT | CLEAR | LIST
        if (cmd == "POLYGROUP" && a.size() == 1 && (iequal(a[0], "show") || iequal(a[0], "hide"))) { // the view's group colours
            if (!view || !view->setPolygroups(iequal(a[0], "show"))) return out.fail("Showing polygroups needs the app window");
            out.say(iequal(a[0], "show") ? "Polygroups shown (a colour per group)" : "Polygroups hidden");
            return out;
        }
        static const char* const kWords[] = {"fill", "clear", "group", "auto", "normals", "angle", "islands", "mask", "masked", "paint", "list", "at"};
        auto isWord = [&](std::size_t k) {
            if (quotedArg(k)) return false;
            const std::string w = lower(a[k]);
            for (const char* q : kWords)
                if (w == q) return true;
            return false;
        };
        std::size_t i = 0;
        std::vector<std::string> nameWords;
        if (!a.empty() && quotedArg(0)) nameWords.push_back(a[i++]);
        else
            while (i < a.size() && !isWord(i)) nameWords.push_back(a[i++]);
        const int bi = nameWords.empty() ? doc.selectedBody : findBody(doc, joinArgs(nameWords));
        if (bi < 0 || bi >= static_cast<int>(doc.scene.bodies().size()))
            return out.fail(nameWords.empty() ? "Select a body first, or give its name" : "No body named " + joinArgs(nameWords));
        Body& body = doc.scene.bodies()[static_cast<std::size_t>(bi)];
        if (body.mesh.indices.empty()) return out.fail(body.name + " has no surface");
        if (i >= a.size()) return usage();
        const std::string op = lower(a[i++]);
        if (cmd == "POLYGROUP" && op == "list") {
            const auto sizes = groupSizes(body);
            if (sizes.empty()) out.say(body.name + " has no polygroups (POLYGROUP AUTO makes them)");
            else out.say(body.name + fmt(": %g polygroups", static_cast<double>(sizes.size())));
            for (const auto& [g, n] : sizes) out.say(fmt("  group %g: ", static_cast<double>(g)) + fmt("%g faces", static_cast<double>(n)));
            return out;
        }
        doc.scene.pushUndo(cmd + " " + body.name);
        std::string done;
        if (cmd == "PAINT") {
            if (op == "clear") {
                clearPaint(body);
                done = "paint cleared";
            } else if (op == "fill") {
                Vec3 rgb;
                if (i >= a.size() || !parseColorText(a[i], rgb)) return doc.scene.undo(), out.fail("FILL needs a colour: a name (red, skin...), #RRGGBB or r,g,b");
                ++i;
                int group = -1;
                if (i + 1 < a.size() && iequal(a[i], "group")) {
                    if (iequal(a[i + 1], "at") && i + 2 < a.size()) {
                        const auto p = parseVec3(a[i + 2]);
                        if (!p) return doc.scene.undo(), out.fail("GROUP AT needs x,y,z");
                        group = groupNear(body, *p);
                    } else if (const auto n = parseNumber(a[i + 1])) group = static_cast<int>(*n);
                    if (group < 0) return doc.scene.undo(), out.fail("No such polygroup");
                }
                if (!paintBody(body, rgb, group)) return doc.scene.undo(), out.fail(group >= 0 ? "No faces in that polygroup" : "Nothing to paint");
                done = "painted" + (group >= 0 ? fmt(" polygroup %g", static_cast<double>(group)) : std::string());
            } else return doc.scene.undo(), usage();
            done += fmt(", %.0f%% of the surface painted", 100.0 * paintedFraction(body));
        } else {
            int n = 0;
            if (op == "auto" || op == "normals") {
                float angle = 20.0f;
                if (i + 1 < a.size() && iequal(a[i], "angle"))
                    if (const auto v = parseNumber(a[i + 1])) angle = static_cast<float>(*v);
                n = groupBody(body, GroupBy::Normals, angle);
                done = fmt("grouped by faces turning less than %g degrees", angle);
            } else if (op == "islands") {
                n = groupBody(body, GroupBy::Islands);
                done = "grouped by connected pieces";
            } else if (op == "mask" || op == "masked") {
                n = groupBody(body, GroupBy::Mask);
                done = "the masked faces made a group";
            } else if (op == "paint") {
                n = groupBody(body, GroupBy::Paint);
                done = "grouped by paint colour";
            } else if (op == "clear") {
                clearGroups(body);
                done = "polygroups cleared";
            } else return doc.scene.undo(), usage();
            if (op != "clear") done += fmt(": %g polygroups", static_cast<double>(n));
            if (op != "clear" && view) view->setPolygroups(true); // show what was made
        }
        doc.scene.touchBodies();
        markChanged(doc);
        out.say(body.name + ": " + done);
        return out;
    }
    if (cmd == "MASK" || cmd == "SUBDIVIDE" || cmd == "SYMMETRIZE" || cmd == "DECIMATE") {
        // MASK [body] CLEAR|INVERT|SMOOTH|GROW|SHRINK|ALL [n]   SUBDIVIDE [body] [LEVELS n]
        // SYMMETRIZE [body] [X|Y|Z] [-]                          DECIMATE [body] TO n | RATIO r
        static const char* const kWords[] = {"clear", "invert", "smooth", "blur", "grow", "shrink", "all", "levels", "level", "group",
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
        if (cmd == "MASK" && !rest.empty() && rest[0] == "group") { // MASK [body] GROUP n | GROUP AT x,y,z [INVERT]
            int g = -1;
            bool inv = false;
            for (std::size_t k = 1; k < rest.size(); ++k) {
                if (rest[k] == "invert") inv = true;
                else if (rest[k] == "at" && k + 1 < rest.size()) {
                    const auto p = parseVec3(rest[++k]);
                    if (!p) return doc.scene.undo(), out.fail("AT needs x,y,z");
                    g = groupNear(body, *p);
                } else if (const auto n = parseNumber(rest[k])) g = static_cast<int>(*n);
            }
            if (g < 0 || !maskGroup(body, static_cast<std::uint32_t>(g), inv)) {
                const std::string why = body.mesh.hasGroups() ? "No such polygroup (POLYGROUP LIST)" : body.name + " has no polygroups (POLYGROUP AUTO makes them)";
                doc.scene.undo(); // (body is gone with it)
                return out.fail(why);
            }
            done = fmt("polygroup %g masked", static_cast<double>(g)) + (inv ? " (the rest)" : "") + fmt(", %.0f%% masked", 100.0 * maskedFraction(body));
        } else if (cmd == "MASK") {
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
            ok = body.multires ? multiresAdd(body, levels, &error) : subdivideBody(body, levels, 4000000, &error); // multires: a level more
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
    if (cmd == "HOLE" && !a.empty() && !quotedArg(0) && (iequal(a[0], "sizes") || iequal(a[0], "list"))) {
        if (iequal(a[0], "sizes")) {
            out.say("Hole sizes (mm): thread pitch, tap drill, clearance close / normal / loose, counterbore, countersink");
            for (const HoleSize& z : holeSizes())
                out.say(std::string("  ") + z.name + (z.inch ? "  (UNC)" : "") +
                        fmt("  pitch %.4g", z.pitch) + fmt("  tap %.4g", z.tapDrill) +
                        fmt("  clear %.4g", z.clearance[0]) + fmt(" / %.4g", z.clearance[1]) + fmt(" / %.4g", z.clearance[2]) +
                        fmt("  cbore %.4g", z.cboreDia) + fmt(" x %.4g", z.cboreDepth) + fmt("  csink %.4g", z.csinkDia) + fmt(" x %g deg", z.csinkAngle));
            return out;
        }
        const std::string name = joinArgs(std::vector<std::string>(a.begin() + 1, a.end()));
        int count = 0;
        for (const Body& b : doc.scene.bodies()) {
            if (!name.empty() && !iequal(b.name, name)) continue;
            for (const HoleNote& h : b.holes) {
                out.say(b.name + "  at " + vecText(h.at + b.offset) + "  " + h.callout);
                ++count;
            }
        }
        if (!name.empty() && findBody(doc, name) < 0) return out.fail("No body named " + name);
        if (count == 0) out.say("No hole wizard holes" + (name.empty() ? std::string() : " in " + name));
        return out;
    }
    if (cmd == "SCULPT" || cmd == "HOLE" || cmd == "POCKET" || cmd == "CHANNEL" || cmd == "TRIM" || cmd == "BOSS") {
        if (cmd == "SCULPT" && a.size() == 1 && iequal(a[0], "alphas")) {
            out.say("Alphas (SCULPT ... ALPHA name, or ALPHA \"image.png\" for your own; white presses, black holds):");
            std::string names = " ";
            for (const std::string& n : sculptAlphaNames()) names += " " + toUpper(n);
            out.say(names);
            return out;
        }
        if (cmd == "SCULPT" && a.size() == 1 && iequal(a[0], "list")) {
            out.say("Brushes:");
            for (SculptBrush br : sculptBrushes()) out.say(std::string("  ") + toUpper(sculptBrushKey(br)) + "  " + sculptBrushName(br));
            return out;
        }
        // Leading words name the body (default: the selection), up to the first keyword or point.
        static const char* const kKeys[] = {"at", "dir", "radius", "strength", "invert", "symmetry", "sym", "detail", "falloff", "radial",
                                            "norefine", "move", "refine", "diameter", "d", "depth", "through", "box",
                                            "round", "ball", "width", "length", "angle", "path", "plane", "normal",
                                            "smaller", "cylinder", "height", "size", "fit", "cbore", "counterbore", "csink",
                                            "countersink", "tapped", "tap", "thread", "flat", "color", "colour",
                                            "alpha", "alphaangle", "follow", "tile", "spacing"};
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
        // Hole wizard: a standard size, its fit, counterbore / countersink, tapping.
        std::string sizeName;
        HoleFit fit = HoleFit::Normal;
        bool cbore = false, csink = false, tapped = false, flat = false;
        float cboreDia = 0, cboreDepth = 0, csinkDia = 0, csinkAngle = 0, threadDepth = 0;
        auto optLength = [&](float& v) { // an optional length after a keyword
            if (i + 1 >= a.size() || quotedArg(i + 1)) return false;
            const auto l = parseLength(a[i + 1]);
            if (!l || !(*l > 0) || a[i + 1].find(',') != std::string::npos) return false;
            v = static_cast<float>(*l);
            ++i;
            return true;
        };
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
            } else if (k == "color" || k == "colour") {
                if (i + 1 >= a.size() || !parseColorText(a[++i], ss.color)) return out.fail("COLOR is a name (red, skin...), #RRGGBB or r,g,b");
            } else if (k == "alpha") {
                if (i + 1 >= a.size()) return out.fail("ALPHA needs a name (SCULPT ALPHAS lists them) or an image file");
                std::string why;
                ss.alpha = sculptAlpha(a[++i], &why);
                if (!ss.alpha) return out.fail(why);
            } else if (k == "alphaangle") {
                const auto v = i + 1 < a.size() ? parseNumber(a[++i]) : std::nullopt;
                if (!v) return out.fail("ALPHAANGLE needs degrees");
                ss.alphaAngle = static_cast<float>(*v);
            } else if (k == "follow") {
                ss.alphaFollow = true;
            } else if (k == "tile") {
                const auto v = i + 1 < a.size() ? parseNumber(a[++i]) : std::nullopt;
                if (!v || *v < 1 || *v > 64) return out.fail("TILE repeats the alpha 1 to 64 times across the brush");
                ss.alphaTile = static_cast<int>(*v);
            } else if (k == "spacing") {
                const auto v = i + 1 < a.size() ? parseNumber(a[++i]) : std::nullopt;
                if (!v || !(*v > 0) || *v > 10) return out.fail("SPACING is the gap between dabs in radii (0.25: the default; 2: stamps side by side)");
                ss.spacing = static_cast<float>(*v);
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
            } else if (cmd == "HOLE" && k == "size") {
                if (i + 1 >= a.size()) return out.fail("SIZE needs a standard size: M6, 1/4-20... (HOLE SIZES lists them)");
                sizeName = a[++i];
            } else if (cmd == "HOLE" && k == "fit") {
                const std::string f = i + 1 < a.size() ? lower(a[++i]) : "";
                if (f == "close") fit = HoleFit::Close;
                else if (f == "normal" || f == "medium") fit = HoleFit::Normal;
                else if (f == "loose" || f == "free") fit = HoleFit::Loose;
                else return out.fail("FIT is CLOSE, NORMAL or LOOSE");
            } else if (cmd == "HOLE" && (k == "cbore" || k == "counterbore")) {
                cbore = true;
                if (optLength(cboreDia) && !optLength(cboreDepth)) return out.fail("CBORE takes a diameter and a depth (or neither, with SIZE)");
            } else if (cmd == "HOLE" && (k == "csink" || k == "countersink")) {
                csink = true;
                if (optLength(csinkDia) && i + 1 < a.size()) {
                    if (const auto ang = parseNumber(a[i + 1]); ang && a[i + 1].find(',') == std::string::npos) csinkAngle = *ang, ++i;
                }
            } else if (cmd == "HOLE" && (k == "tapped" || k == "tap")) {
                tapped = true;
            } else if (cmd == "HOLE" && k == "thread") {
                if (!lengthArg(threadDepth, "THREAD") || threadDepth <= 0) return out.fail("THREAD needs the thread's depth");
                tapped = true;
            } else if (cmd == "HOLE" && k == "flat") {
                flat = true;
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
                if (body.multires) ss.refine = false; // the levels keep their vertices
                sculptStroke(body, surface, points, ss, move.value_or(Vec3{}));
                ok = true;
                done = std::string(sculptBrushKey(ss.brush)) + (ss.invert ? " (inverted)" : "") +
                       fmt(" stroke, radius %g", ss.radius);
            }
        } else if (cmd == "HOLE") {
            const bool wizard = !sizeName.empty() || cbore || csink || tapped || flat;
            if (points.empty() || (!wizard && (points.size() != 1 || diameter <= 0))) return doc.scene.undo(), usage();
            if (!wizard) {
                Vec3 n;
                const Vec3 p = onSurface(points[0], &n);
                if (!offBody.empty()) return doc.scene.undo(), out.fail(offBody);
                ok = drillHole(body, p, n, diameter, through ? 0.0f : depth, &error);
                done = fmt("hole %g mm", diameter) + (through || depth <= 0 ? " through" : fmt(", %g deep", depth));
                if (ok) { // its note, for callouts
                    HoleNote note;
                    note.at = p - body.offset, note.dir = -normalize(n), note.callout = fmt("dia %g", diameter) + (through || depth <= 0 ? " thru" : fmt(" x %g deep", depth));
                    note.diameter = diameter, note.depth = through ? 0.0f : std::max(depth, 0.0f);
                    body.holes.push_back(note);
                }
            } else {
                // The hole's shape from the standard size (or the sizes given).
                HoleSpec h;
                h.depth = through ? 0.0f : depth;
                h.tipped = !flat;
                const HoleSize* z = nullptr;
                if (!sizeName.empty() && !(z = findHoleSize(sizeName)))
                    return doc.scene.undo(), out.fail("No standard size " + sizeName + " (HOLE SIZES lists them)");
                if (z) {
                    h.size = z->name;
                    h.diameter = tapped ? z->tapDrill : z->clearance[static_cast<int>(fit)];
                    if (cbore) h.cboreDia = cboreDia > 0 ? cboreDia : z->cboreDia, h.cboreDepth = cboreDepth > 0 ? cboreDepth : z->cboreDepth;
                    if (csink) h.csinkDia = csinkDia > 0 ? csinkDia : z->csinkDia, h.csinkAngle = csinkAngle > 0 ? csinkAngle : z->csinkAngle;
                    if (tapped) {
                        h.threadDia = z->nominal, h.threadPitch = z->pitch;
                        // A blind tapped hole: the thread stops short of the drill's full depth.
                        h.threadDepth = threadDepth > 0 ? threadDepth : (h.depth > 0 ? std::max(h.depth - 2.0f * z->pitch, h.depth * 0.5f) : 0.0f);
                    }
                } else {
                    if (tapped) return doc.scene.undo(), out.fail("A tapped hole needs a SIZE (M6, 1/4-20...)");
                    if (cbore && !(cboreDia > 0)) return doc.scene.undo(), out.fail("CBORE needs a diameter and depth, or a SIZE");
                    if (csink && !(csinkDia > 0)) return doc.scene.undo(), out.fail("CSINK needs a diameter, or a SIZE");
                    h.cboreDia = cboreDia, h.cboreDepth = cboreDepth, h.csinkDia = csinkDia, h.csinkAngle = csinkAngle > 0 ? csinkAngle : 90.0f;
                }
                if (diameter > 0) h.diameter = diameter; // a drill size given outright
                if (!(h.diameter > 0)) return doc.scene.undo(), out.fail("The hole needs a SIZE or a DIAMETER");
                if (h.threadDepth > 0 && h.depth > 0 && h.threadDepth > h.depth) return doc.scene.undo(), out.fail("The thread can't go deeper than the hole");
                // Every point onto the surface first, then drill them one by one.
                std::vector<std::pair<Vec3, Vec3>> at;
                for (const Vec3& q : points) {
                    Vec3 n;
                    const Vec3 p = onSurface(q, &n);
                    at.push_back({p, n});
                }
                if (!offBody.empty()) return doc.scene.undo(), out.fail(offBody);
                ok = true;
                for (const auto& [p, n] : at) {
                    if (!drillHole(body, p, n, h, &error)) {
                        ok = false;
                        break;
                    }
                    HoleNote& note = body.holes.emplace_back();
                    note.at = p - body.offset, note.dir = -normalize(n), note.callout = holeCallout(h);
                    note.threadDia = h.threadDia, note.pitch = h.threadPitch, note.threadDepth = h.threadDepth;
                    note.diameter = h.diameter, note.depth = h.depth, note.size = h.size;
                    note.cboreDia = h.cboreDia, note.cboreDepth = h.cboreDepth, note.csinkDia = h.csinkDia, note.csinkAngle = h.csinkAngle;
                }
                done = (at.size() > 1 ? std::to_string(at.size()) + " holes " : std::string("hole ")) + holeCallout(h);
            }
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
                                            "period", "size", "skin", "cell", "wall", "hollow", "face"};
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
                } else if (k == "face") {
                    // The face nearest the point is taken away: the cut lies a hair inside it.
                    const auto p = i + 1 < a.size() ? parseVec3(a[++i]) : std::nullopt;
                    if (!p) return out.fail("FACE needs a point x,y,z on the face to leave open");
                    const MeshData w = worldMesh(body);
                    float best = 1e30f;
                    Vec3 q{}, n{0, 0, 1};
                    for (std::size_t t = 0; t + 2 < w.indices.size(); t += 3) {
                        const Vec3 p0 = w.vertices[w.indices[t]].position, p1 = w.vertices[w.indices[t + 1]].position, p2 = w.vertices[w.indices[t + 2]].position;
                        const Vec3 nn = cross(p1 - p0, p2 - p0);
                        if (!(length(nn) > 0)) continue;
                        const Vec3 c = closestPointOnTriangle(*p, p0, p1, p2);
                        if (length(c - *p) < best) best = length(c - *p), q = c, n = normalize(nn);
                    }
                    if (best >= 1e30f) return out.fail(body.name + " has no faces");
                    const AABB wb = w.bounds();
                    o.open = true;
                    o.openPoint = q - n * (1e-4f * length(wb.max - wb.min));
                    o.openNormal = n;
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
        if (b.linked()) return out.fail(b.name + " is linked component " + b.component + ": COMPONENT ROTATE " + b.component + " rx,ry,rz turns it");
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
        noteRecentFile(doc.path);
        out.say("Saved " + doc.path);
        return out;
    }
    if (cmd == "EXPORT") {
        if (a.empty()) return usage();
        const std::string path = joinArgs(a);
        if (RobotFormat rf; robotFormatOf(path, rf)) return runRobotCommand(doc, "RIG", {"export", path}, {false, true});
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
            const int n = fileKindOf(path) == FileKind::Dxf ? exportDxf(doc.scene, doc.state.plane, path, &err, doc.state.customFrame)
                                                            : exportSvg(doc.scene, doc.state.plane, path, &err, doc.state.customFrame);
            if (n < 0) return out.fail(err);
            out.say("Exported " + std::to_string(n) + " curves on " + activePlaneName(doc.state) + " to " + path);
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
        std::optional<double> fileUnit; // mesh models: one file unit in mm (UNIT inch...)
        for (std::size_t i = 0; i < rest.size();) {
            if (iequal(rest[i], "unit") && i + 1 < rest.size() && (parseUnitName(rest[i + 1]) || iequal(rest[i + 1], "micron"))) {
                fileUnit = iequal(rest[i + 1], "micron") ? 0.001 : toMillimeters(1.0, *parseUnitName(rest[i + 1]));
                rest.erase(rest.begin() + static_cast<std::ptrdiff_t>(i), rest.begin() + static_cast<std::ptrdiff_t>(i) + 2);
            } else if (iequal(rest[i], "width") && i + 1 < rest.size() && parseLength(rest[i + 1])) {
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
            const int n = importDxf(doc.scene, doc.state.plane, path, &err, doc.state.customFrame);
            if (n <= 0) return out.fail(err.empty() ? "Nothing imported" : err);
            markChanged(doc);
            out.say("Imported " + std::to_string(n) + " curves onto " + activePlaneName(doc.state));
            return out;
        }
        case FileKind::Image: {
            if (doc.state.plane == SketchPlane::Custom) return out.fail("Images go on the Top, Front or Right plane: PLANE TOP first");
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
            if (iges) { // unnamed IGES parts (no group naming them): the file's name, numbered
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
        case FileKind::MeshModel: {
            MeshImportReport rep;
            std::vector<ImportedPart> parts = importMeshModel(path, &rep, &err, fileUnit);
            if (parts.empty()) return out.fail(err.empty() ? "Nothing imported" : err);
            std::size_t tris = 0;
            std::string names;
            for (const ImportedPart& p : parts) {
                Body b = bodyFromPart(p);
                if (!p.color) b.color = kPalette[static_cast<std::size_t>(++doc.primitiveSerial) % std::size(kPalette)];
                const std::string base = b.name.empty() ? fs::path(path).stem().string() : b.name;
                std::string name = base;
                for (int k = 2; findBody(doc, name) >= 0; ++k) name = base + " " + std::to_string(k);
                b.name = name;
                tris += b.mesh.indices.size() / 3;
                if (names.size() < 120) names += (names.empty() ? "" : ", ") + name;
                doc.scene.add(std::move(b));
            }
            doc.selectedBody = static_cast<int>(doc.scene.bodies().size()) - 1;
            doc.selectedCurve = doc.selectedImage = -1;
            markChanged(doc);
            char u[64];
            std::snprintf(u, sizeof(u), "%g mm", rep.unitToMm);
            out.say("Imported " + std::to_string(parts.size()) + (parts.size() == 1 ? " part" : " parts") + " from " + path + " (" + rep.format + ", 1 file unit = " + u +
                    (rep.unitAssumed ? ", taken: the file doesn't say; IMPORT ... UNIT inch changes it" : "") + "): " + names);
            out.say("  " + std::to_string(tris) + " triangles");
            for (std::size_t k = 0; k < rep.warnings.size() && k < 5; ++k) out.say("  warning: " + rep.warnings[k]);
            return out;
        }
        case FileKind::Project: return out.fail("Use OPEN for project files");
        default:
            return out.fail("Import supports .step .stp .igs .iges, .stl .obj .3mf .amf .ply .off .gltf .glb .wrl, .dxf and images (.png .jpg .bmp .tga .gif ...)");
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
            if (b.linked()) { // a linked part moves as one
                const std::string comp = b.component;
                moveComponent(s, comp, *d);
                markChanged(doc);
                out.say("Moved " + comp + fmt(" by (%.1f, %.1f, %.1f)", d->x, d->y, d->z));
                return out;
            }
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
        out = runPlaneCommand(doc, a, pc.quoted);
        if (out.ok && view) view->planeChanged(doc.state.plane);
        return out;
    }
    if (cmd == "2D") {
        if (auto o = needView(view, "2D"); !o.ok) return o;
        if (!a.empty()) { // a plane or datum plane, as PLANE takes it
            if (a.size() != 1) return usage();
            if (auto r = runPlaneCommand(doc, a, pc.quoted); !r.ok) return r;
        }
        view->planeChanged(doc.state.plane);
        view->openWindow(true);
        out.say("2D Drawing window: " + activePlaneName(doc.state));
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
        // PANEL MODEL | LIBRARY | CAM | TOOLS | MACHINE | GUIDE | PREFERENCES | LICENCE | PRODUCT | APPEARANCE | SECTION | PATH3D | STUDY | STACK | ARMATURE: open that page or window.
        const std::string page = toUpper(a[0]);
        const int p = page == "MODEL" ? 0 : page == "LIBRARY" ? 1 : page == "CAM" || page == "LATHE" ? 2 : page == "TOOLS" ? 3 : page == "MACHINE" ? 4 : page == "GUIDE" ? 5
                    : page == "PREFERENCES" || page == "PREFS" ? 6 : page == "LICENCE" || page == "LICENSE" ? 7
                    : page == "PRODUCT" || page == "PDM" || page == "VAULT" ? 10
                    : page == "APPEARANCE" || page == "MATERIALS" ? 11 : page == "SECTION" ? 12 : page == "PATH3D" || page == "ROUTE" ? 13
                    : page == "STUDY" || page == "PLOT" ? 14 : page == "STACK" || page == "TOLERANCE" || page == "STACKUP" ? 15
                    : page == "ARMATURE" || page == "ZSPHERE" ? 16 : -1;
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

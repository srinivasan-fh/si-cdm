// RIG: robots read from the scene for simulation and export.

#include "core/RobotCommand.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/FileIO.hpp"
#include "core/Multibody.hpp"
#include "core/Rig.hpp"
#include "core/Scene.hpp"
#include "core/Terrain.hpp"
#include "core/Units.hpp"
#include "core/RobotExport.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <optional>

namespace sicdm {

namespace {

std::string lowerOf(const std::string& s) {
    std::string o;
    for (char c : s) o += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return o;
}

const char* const kRigWords[] = {"list", "tag", "untag", "morphology", "type", "base", "density", "engine", "apply", "show", "export", "simulate", "sim", "state", "engines"};

bool isRigWord(const std::string& w) {
    const std::string k = lowerOf(w);
    for (const char* q : kRigWords)
        if (k == q) return true;
    return false;
}

int bodyIndex(const Document& doc, const std::string& name) {
    const auto& bs = doc.scene.bodies();
    for (std::size_t i = 0; i < bs.size(); ++i)
        if (bs[i].name == name) return static_cast<int>(i);
    for (std::size_t i = 0; i < bs.size(); ++i)
        if (lowerOf(bs[i].name) == lowerOf(name)) return static_cast<int>(i);
    return -1;
}

} // namespace

bool pickRobot(const Document& doc, const std::vector<std::string>& a, const std::vector<bool>& quoted, std::size_t& i, std::string& robot,
               std::string* error) {
    const std::vector<std::string> cands = rigCandidates(doc.scene);
    auto known = [&](const std::string& n) {
        for (const std::string& c : cands)
            if (lowerOf(c) == lowerOf(n)) return c;
        for (const RigSettings& r : doc.scene.rigs().robots)
            if (lowerOf(r.name) == lowerOf(n)) return r.name;
        for (const Body& b : doc.scene.bodies()) {
            if (!b.component.empty() && lowerOf(b.component) == lowerOf(n)) return b.component;
            if (!b.assembly.empty() && lowerOf(b.assembly.substr(0, b.assembly.find('/'))) == lowerOf(n)) return b.assembly.substr(0, b.assembly.find('/'));
        }
        return std::string("\x01");
    };
    if (i < a.size() && i < quoted.size() && quoted[i]) {
        const std::string k = known(a[i]);
        if (k == "\x01") {
            if (error) *error = "No robot named " + a[i];
            return false;
        }
        robot = k;
        ++i;
        return true;
    }
    // Unquoted words up to the next keyword.
    std::size_t j = i;
    std::string words;
    while (j < a.size() && !isRigWord(a[j]) && !(j < quoted.size() && quoted[j])) words += (words.empty() ? "" : " ") + a[j++];
    if (!words.empty()) {
        const std::string k = known(words);
        if (k != "\x01") {
            robot = k;
            i = j;
            return true;
        }
    }
    if (cands.size() == 1) {
        robot = cands[0];
        return true;
    }
    if (cands.empty()) {
        // No joints at all: the whole scene as one body (tags can still make joints).
        robot.clear();
        return true;
    }
    if (error) {
        std::string list;
        for (const std::string& c : cands) list += (list.empty() ? "" : ", ") + (c.empty() ? std::string("\"\" (loose jointed parts)") : "\"" + c + "\"");
        *error = "Name the robot: " + list;
    }
    return false;
}

namespace {

std::string fmtMm(double v) {
    char b[32];
    std::snprintf(b, sizeof b, "%g", std::round(v * 10.0) / 10.0);
    return b;
}

std::string describeTerrain(const TerrainSettings& t) {
    if (!t.active()) return "No terrain: robots stand on the floor at z = 0";
    std::string s = std::string("Terrain: ") + terrainKindKey(t.kind) + ", " + fmtMm(t.sizeX) + " x " + fmtMm(t.sizeY) + " mm at (" + fmtMm(t.centre.x) + ", " +
                    fmtMm(t.centre.y) + ")";
    switch (t.kind) {
    case TerrainKind::Slope: s += ", " + fmtMm(t.angle) + " deg climbing toward " + fmtMm(t.direction) + " deg"; break;
    case TerrainKind::Stairs: s += ", " + std::to_string(t.steps) + " steps of " + fmtMm(t.rise) + " mm up, " + fmtMm(t.run) + " mm deep"; break;
    case TerrainKind::Rough: s += ", bumps up to " + fmtMm(t.amplitude) + " mm, " + fmtMm(t.wavelength) + " mm across (seed " + std::to_string(t.seed) + ")"; break;
    case TerrainKind::Blocks: s += ", blocks up to " + fmtMm(t.blockHeight) + " mm on a " + fmtMm(t.blockSize) + " mm grid"; break;
    case TerrainKind::Dem:
        s += ", " + std::to_string(t.nx) + " x " + std::to_string(t.ny) + " heights every " + fmtMm(t.cell) + " mm from " + t.demPath;
        break;
    default: break;
    }
    return s + ", friction " + fmtMm(t.friction);
}

CommandOutput runTerrain(Document& doc, const std::vector<std::string>& a) {
    CommandOutput out;
    TerrainSettings t = doc.scene.terrain();
    if (a.empty()) {
        out.say(describeTerrain(t));
        return out;
    }
    std::size_t i = 0;
    const std::string first = lowerOf(a[0]);
    std::string demFile;
    float demCell = 0, demHeight = 0;
    if (const auto k = terrainKindFromKey(first)) {
        if (*k != t.kind) {
            const TerrainSettings defaults;
            const Vec2 c = t.centre;
            const float sx = t.sizeX, sy = t.sizeY;
            t = defaults;
            t.centre = c, t.sizeX = sx, t.sizeY = sy;
        }
        t.kind = *k;
        ++i;
        if (*k == TerrainKind::Dem) {
            if (i >= a.size()) return out.fail("TERRAIN DEM needs a file: .asc (ESRI grid) or a greyscale image");
            demFile = a[i++];
        }
    }
    auto lengthArg = [&](float& v, const char* what) -> bool {
        if (i + 1 >= a.size()) return false;
        const auto l = parseLength(a[++i]);
        if (!l || *l <= 0) {
            out.fail(std::string(what) + " needs a length");
            return false;
        }
        v = static_cast<float>(*l);
        return true;
    };
    for (; i < a.size(); ++i) {
        const std::string k = lowerOf(a[i]);
        auto number = [&](float& v) -> bool {
            if (i + 1 >= a.size()) return false;
            const auto n = parseNumber(a[++i]);
            if (!n) return false;
            v = *n;
            return true;
        };
        if (k == "angle") {
            if (!number(t.angle) || t.angle < 0 || t.angle > 60) return out.fail("ANGLE needs 0 to 60 degrees");
        } else if (k == "direction" || k == "toward") {
            if (!number(t.direction)) return out.fail("DIRECTION needs degrees from +X");
        } else if (k == "rise") {
            if (!lengthArg(t.rise, "RISE")) return out.fail("RISE needs a length");
        } else if (k == "run" || k == "depth") {
            if (!lengthArg(t.run, "RUN")) return out.fail("RUN needs a length");
        } else if (k == "steps" || k == "count") {
            float n = 0;
            if (!number(n) || n < 1 || n > 200) return out.fail("STEPS needs 1 to 200");
            t.steps = static_cast<int>(n);
        } else if (k == "amplitude" || k == "bumps") {
            if (!lengthArg(t.amplitude, "AMPLITUDE")) return out.fail("AMPLITUDE needs a length");
        } else if (k == "wavelength" || k == "scale") {
            if (!lengthArg(t.wavelength, "WAVELENGTH")) return out.fail("WAVELENGTH needs a length");
        } else if (k == "height") {
            float h = 0;
            if (!lengthArg(h, "HEIGHT")) return out.fail("HEIGHT needs a length");
            t.blockHeight = h, demHeight = h;
        } else if (k == "block" || k == "blocksize") {
            if (!lengthArg(t.blockSize, "BLOCK")) return out.fail("BLOCK needs a length");
        } else if (k == "cell") {
            if (!lengthArg(demCell, "CELL")) return out.fail("CELL needs a length");
        } else if (k == "size") {
            if (!lengthArg(t.sizeX, "SIZE")) return out.fail("SIZE needs x [y]");
            t.sizeY = t.sizeX;
            if (i + 1 < a.size())
                if (const auto l = parseLength(a[i + 1]); l && *l > 0) t.sizeY = static_cast<float>(*l), ++i;
        } else if (k == "at") {
            const auto p = i + 1 < a.size() ? parseVec3(a[++i]) : std::nullopt;
            if (!p) return out.fail("AT needs x,y");
            t.centre = {p->x, p->y};
        } else if (k == "seed") {
            float s = 0;
            if (!number(s)) return out.fail("SEED needs a number");
            t.seed = static_cast<int>(s);
        } else if (k == "friction") {
            if (!number(t.friction) || t.friction < 0 || t.friction > 3) return out.fail("FRICTION needs 0 to 3");
        } else {
            return out.fail("Usage: TERRAIN [OFF | SLOPE | STAIRS | ROUGH | BLOCKS | DEM file] [ANGLE deg] [DIRECTION deg] [RISE mm] [RUN mm] [STEPS n] "
                            "[AMPLITUDE mm] [WAVELENGTH mm] [HEIGHT mm] [BLOCK mm] [CELL mm] [SIZE x [y]] [AT x,y] [SEED n] [FRICTION mu]");
        }
        if (!out.ok) return out;
    }
    if (!demFile.empty()) {
        std::string err;
        const Vec2 c = t.centre;
        if (!loadDem(demFile, t, demCell, demHeight, &err)) return out.fail(err);
        t.centre = c;
    }
    // The body that shows it.
    auto& bodies = doc.scene.bodies();
    int bi = -1;
    for (std::size_t k = 0; k < bodies.size(); ++k)
        if (bodies[k].name == kTerrainBody) bi = static_cast<int>(k);
    if (!t.active()) {
        if (bi >= 0) doc.scene.remove(static_cast<std::size_t>(bi));
    } else {
        Body body = meshBody(kTerrainBody, terrainMesh(t), {0.55f, 0.50f, 0.42f});
        body.component = kTerrainBody;
        if (bi >= 0) {
            bodies[static_cast<std::size_t>(bi)].mesh = std::move(body.mesh);
            bodies[static_cast<std::size_t>(bi)].offset = {};
            doc.scene.touchBodies();
        } else {
            doc.scene.add(std::move(body));
        }
    }
    doc.scene.terrain() = t;
    doc.dirty = true;
    out.say(describeTerrain(t));
    return out;
}

} // namespace

CommandOutput runRobotCommand(Document& doc, const std::string& cmd, const std::vector<std::string>& a, const std::vector<bool>& quoted) {
    CommandOutput out;
    if (cmd == "TERRAIN") return runTerrain(doc, a);
    if (cmd != "RIG") return out.fail("Unknown robot command " + cmd);
    if (!a.empty() && lowerOf(a[0]) == "engines") {
        for (const SimEngineInfo& e : simEngines()) out.say(std::string(e.key) + ": " + e.title + ". " + e.summary);
        return out;
    }
    if (!a.empty() && lowerOf(a[0]) == "list") {
        const auto cands = rigCandidates(doc.scene);
        if (cands.empty()) out.say("No robots: no part has a joint (motions, JOINT) or a ROTOR / WHEEL tag");
        for (const std::string& c : cands) {
            RigSettings s;
            if (const RigSettings* r = doc.scene.rigs().find(c)) s = *r;
            else s.name = c;
            RobotRig rig;
            std::string err;
            if (buildRig(doc.scene, s, rig, &err)) out.say(describeRig(rig).front());
        }
        return out;
    }
    std::size_t i = 0;
    std::string robot, err;
    if (!pickRobot(doc, a, quoted, i, robot, &err)) return out.fail(err);
    RigSettings settings;
    if (const RigSettings* r = doc.scene.rigs().find(robot)) settings = *r;
    else settings.name = robot;
    bool changed = false;
    std::vector<std::string> notes;
    std::string exportPath;
    std::optional<float> stateAt;
    for (; i < a.size(); ++i) {
        const std::string k = lowerOf(a[i]);
        auto next = [&]() -> std::string { return i + 1 < a.size() ? a[++i] : std::string(); };
        if (k == "show") continue;
        if (k == "export") {
            for (++i; i < a.size(); ++i) exportPath += (exportPath.empty() ? "" : " ") + a[i];
            if (exportPath.empty()) return out.fail("EXPORT needs a file: robot.urdf, robot.sdf or robot.xml (MJCF)");
            break;
        }
        if (k == "tag" || k == "untag") {
            // TAG body ROLE (body: quoted, or words up to the role)
            std::string body, role = k == "untag" ? "none" : "";
            if (i + 1 < a.size() && i + 1 < quoted.size() && quoted[i + 1]) {
                body = a[++i];
                if (k == "tag") role = next();
            } else {
                std::vector<std::string> w;
                for (++i; i < a.size(); ++i) w.push_back(a[i]);
                if (k == "tag" && !w.empty()) {
                    role = w.back();
                    w.pop_back();
                }
                for (const std::string& x : w) body += (body.empty() ? "" : " ") + x;
            }
            const int bi = bodyIndex(doc, body);
            if (bi < 0) return out.fail(body.empty() ? "TAG needs a part and a role" : "No body named " + body);
            const auto tag = rigTagFromKey(role);
            if (!tag) return out.fail("Roles: ROTOR, WHEEL, END_EFFECTOR, FOOT, KNEE, HINGE, BASE or NONE");
            settings.setTag(doc.scene.bodies()[static_cast<std::size_t>(bi)].name, *tag);
            changed = true;
        } else if (k == "morphology" || k == "type") {
            const auto m = morphologyFromKey(next());
            if (!m) return out.fail("MORPHOLOGY is AUTO, MANIPULATOR, HUMANOID, QUADRUPED, DRONE, ROVER or CUSTOM");
            settings.morphology = *m;
            changed = true;
        } else if (k == "base") {
            const std::string v = lowerOf(next());
            if (v == "fixed") settings.floatingBase = false;
            else if (v == "floating" || v == "free") settings.floatingBase = true;
            else if (v == "auto") settings.floatingBase.reset();
            else return out.fail("BASE is FIXED, FLOATING or AUTO");
            changed = true;
        } else if (k == "density") {
            const auto v = parseNumber(next());
            if (!v || *v < 0) return out.fail("DENSITY needs kg/m^3 (0: each part's own)");
            settings.density = static_cast<float>(*v);
            changed = true;
        } else if (k == "engine") {
            const std::string e = lowerOf(next());
            if (e == "auto") settings.engine.clear();
            else if (const auto se = simEngineFromKey(e)) settings.engine = simEngineKey(*se);
            else return out.fail("ENGINE is AUTO, ARTICULATED, VEHICLE, OPTIMIZE or PARTICLE (RIG ENGINES lists them)");
            changed = true;
        } else if (k == "simulate" || k == "sim") {
            const std::string v = lowerOf(next());
            if (v == "on" || v == "yes") settings.simulate = true;
            else if (v == "off" || v == "no") settings.simulate = false;
            else return out.fail("SIMULATE is ON or OFF");
            changed = true;
        } else if (k == "state") {
            float t = 0;
            if (i + 1 < a.size() && lowerOf(a[i + 1]) == "at") {
                ++i;
                const auto v = parseNumber(next());
                if (!v || *v < 0) return out.fail("STATE AT needs seconds");
                t = *v;
            }
            stateAt = t;
        } else {
            return out.fail("Usage: RIG [robot] [TAG part ROLE] [UNTAG part] [MORPHOLOGY m] [BASE FIXED|FLOATING|AUTO] [DENSITY kg/m3] [ENGINE e] "
                            "[SIMULATE ON|OFF] [STATE [AT t]] [EXPORT file]");
        }
    }
    if (changed) {
        doc.scene.rigs().get(robot) = settings;
        notes = applyRigTags(doc.scene, settings);
        doc.dirty = true;
    }
    RobotRig rig;
    if (!buildRig(doc.scene, settings, rig, &err)) return out.fail(err);
    for (const std::string& l : notes) out.say(l);
    if (stateAt) {
        const float duration = doc.scene.physics().duration > 0 ? doc.scene.physics().duration : 10.0f;
        const auto run = robotRun(doc.scene, settings, std::max(duration, *stateAt));
        if (!run) return out.fail("The " + std::string(simEngineKey(engineFor(rig, settings))) + " engine doesn't run robots (RIG ENGINE ARTICULATED)");
        const RobotRunFrame f = run->at(*stateAt);
        char buf[256];
        std::snprintf(buf, sizeof buf, "%s at %.3f s (%s engine): centre of mass (%.1f, %.1f, %.1f) mm, %.1f N on the ground", rig.name.c_str(), *stateAt,
                      simEngineKey(run->engine), f.com[0] * 1e3, f.com[1] * 1e3, f.com[2] * 1e3, f.contactForce);
        out.say(buf);
        if (rig.floatingBase && !f.links.empty()) {
            const LinkPose& b = f.links[0];
            const double tilt = std::acos(std::clamp(b.R[8], -1.0, 1.0)) * 180.0 / 3.14159265358979;
            std::snprintf(buf, sizeof buf, "  base moved (%.1f, %.1f, %.1f) mm, tilted %.1f deg", b.t[0] * 1e3, b.t[1] * 1e3, b.t[2] * 1e3, tilt);
            out.say(buf);
        }
        const MotionTargets tg = motionTargets(doc.scene, run->rig, *stateAt);
        for (std::size_t j = 0; j < rig.joints.size() && j < f.q.size(); ++j) {
            const RigJoint& J = rig.joints[j];
            const bool lin = J.type == RigJointType::Prismatic;
            const double k = lin ? 1e3 : 180.0 / 3.14159265358979;
            if (J.type == RigJointType::Continuous) std::snprintf(buf, sizeof buf, "  %-34s %8.1f deg", J.name.c_str(), f.q[j] * k);
            else
                std::snprintf(buf, sizeof buf, "  %-34s %8.2f %s (target %.2f)", J.name.c_str(), f.q[j] * k, lin ? "mm" : "deg", tg.value[j] * k);
            out.say(buf);
        }
        if (run->diverged) out.say("  note: " + run->note);
        return out;
    }
    if (!exportPath.empty()) {
        const RobotExportResult r = exportRobot(doc.scene, rig, exportPath);
        if (!r.ok) return out.fail(r.error);
        RobotFormat f;
        robotFormatOf(exportPath, f);
        out.say(std::string("Exported ") + robotFormatName(f) + " " + exportPath + ": " + std::to_string(rig.links.size()) + " links, " +
                std::to_string(rig.joints.size()) + " joints, " + std::to_string(r.files.size() - 1) + " meshes");
        return out;
    }
    for (const std::string& l : describeRig(rig)) out.say(l);
    return out;
}

} // namespace sicdm

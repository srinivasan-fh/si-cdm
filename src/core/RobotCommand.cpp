// RIG: robots read from the scene for simulation and export.

#include "core/RobotCommand.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/FileIO.hpp"
#include "core/Multibody.hpp"
#include "core/Rig.hpp"
#include "core/RobotExport.hpp"

#include <algorithm>
#include <cctype>
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

CommandOutput runRobotCommand(Document& doc, const std::string& cmd, const std::vector<std::string>& a, const std::vector<bool>& quoted) {
    CommandOutput out;
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

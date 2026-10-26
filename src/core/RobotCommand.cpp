// RIG: robots read from the scene for simulation and export.

#include "core/RobotCommand.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/FileIO.hpp"
#include "core/Kinematics.hpp"
#include "core/Motion.hpp"
#include "core/Multibody.hpp"
#include "core/Rig.hpp"
#include "core/Sensors.hpp"
#include "core/Scene.hpp"
#include "core/Terrain.hpp"
#include "core/Units.hpp"
#include "core/RobotEnv.hpp"
#include "core/RobotExport.hpp"

#include <algorithm>
#include <chrono>
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

const char* const kRigWords[] = {"list", "tag", "untag", "morphology", "type", "base", "density", "engine", "apply", "show", "export", "simulate", "sim", "state", "engines", "mission", "sensor", "sensors", "env",
                                  "move_to", "moveto", "ik", "home", "tool", "point", "approach", "at", "speed", "optimize", "optimise"};

bool isRigWord(const std::string& w) {
    const std::string k = lowerOf(w);
    for (const char* q : kRigWords)
        if (k == q) return true;
    return false;
}

std::string toUpper(std::string s) {
    for (char& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return s;
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

// RIG [robot] SENSOR ...: `i` is at the word after SENSOR.
CommandOutput runSensor(Document& doc, RigSettings& settings, const std::vector<std::string>& a, const std::vector<bool>& quoted, std::size_t i) {
    CommandOutput out;
    auto word = [&]() -> std::string { return i < a.size() ? a[i++] : std::string(); };
    auto isQuoted = [&](std::size_t k) { return k < quoted.size() && quoted[k]; };
    const std::string sub = lowerOf(word());
    auto findSensor = [&](const std::string& n) -> SensorSpec* {
        for (SensorSpec& s : settings.sensors)
            if (lowerOf(s.name) == lowerOf(n)) return &s;
        return nullptr;
    };
    auto timeOption = [&](float& t) -> bool {
        if (i < a.size() && lowerOf(a[i]) == "at") {
            ++i;
            const auto v = parseNumber(word());
            if (!v || *v < 0) return false;
            t = *v;
        }
        return true;
    };
    if (sub.empty() || sub == "list") {
        if (settings.sensors.empty()) out.say("No sensors (RIG SENSOR ADD IMU|GPS|FORCE|CAMERA|LIDAR name ON part)");
        for (const SensorSpec& s : settings.sensors) {
            char buf[256];
            std::snprintf(buf, sizeof buf, "%s: %s on %s at (%g, %g, %g) mm looking (%g, %g, %g), %g Hz, noise x%g", s.name.c_str(), sensorKindKey(s.kind), s.body.c_str(), s.at.x,
                          s.at.y, s.at.z, s.look.x, s.look.y, s.look.z, s.rate, s.noise);
            out.say(buf);
        }
        return out;
    }
    if (sub == "add") {
        SensorSpec s;
        const auto kind = sensorKindFromKey(word());
        if (!kind) return out.fail("SENSOR ADD IMU|GPS|FORCE|CAMERA|LIDAR name ON part [options]");
        s.kind = *kind;
        if (s.kind == SensorKind::Camera) s.rate = 30;
        if (s.kind == SensorKind::Lidar) s.rate = 10;
        if (s.kind == SensorKind::Gps) s.rate = 10;
        s.name = word();
        if (s.name.empty() || lowerOf(s.name) == "on") return out.fail("SENSOR ADD needs a name");
        if (findSensor(s.name)) return out.fail("There is a sensor named " + s.name + " already");
        bool atGiven = false;
        for (; i < a.size();) {
            const std::string k = lowerOf(word());
            if (k == "on") {
                std::string body;
                if (isQuoted(i)) body = word();
                else
                    while (i < a.size() && !isQuoted(i)) {
                        static const char* const keys[] = {"at", "look", "up", "rate", "noise", "bias", "seed", "size", "fov", "range", "channels", "samples", "vfov", "origin"};
                        bool key = false;
                        for (const char* q : keys) key = key || lowerOf(a[i]) == q;
                        if (key) break;
                        body += (body.empty() ? "" : " ") + word();
                    }
                const int bi = bodyIndex(doc, body);
                if (bi < 0) return out.fail("No part named " + body);
                s.body = doc.scene.bodies()[static_cast<std::size_t>(bi)].name;
            } else if (k == "at" || k == "look" || k == "up") {
                const auto v = parseVec3(word());
                if (!v) return out.fail(toUpper(k) + " needs x,y,z");
                if (k == "at") s.at = *v, atGiven = true;
                else if (k == "look") s.look = *v;
                else s.up = *v;
            } else if (k == "size") {
                const auto w = parseNumber(word()), h = parseNumber(word());
                if (!w || !h || *w < 8 || *h < 8) return out.fail("SIZE needs a width and height in pixels (8 or more)");
                s.width = static_cast<int>(*w), s.height = static_cast<int>(*h);
            } else if (k == "origin") {
                const auto v = parseVec3(word());
                if (!v) return out.fail("ORIGIN needs latitude,longitude[,altitude]");
                s.lat = v->x, s.lon = v->y, s.alt = v->z;
            } else {
                const auto v = parseNumber(word());
                if (!v) return out.fail(toUpper(k) + " needs a number");
                if (k == "rate" && *v > 0) s.rate = *v;
                else if (k == "noise" && *v >= 0) s.noise = *v;
                else if (k == "bias" && *v >= 0) s.bias = *v;
                else if (k == "seed") s.seed = static_cast<int>(*v);
                else if (k == "fov" && *v > 0 && *v < 180) s.fov = *v;
                else if (k == "range" && *v > 0) s.range = *v;
                else if (k == "channels" && *v >= 1) s.channels = static_cast<int>(*v);
                else if (k == "samples" && *v >= 1) s.samples = static_cast<int>(*v);
                else if (k == "vfov" && *v >= 0) s.vfov = *v;
                else return out.fail("Bad or unknown sensor option " + k);
            }
        }
        if (s.body.empty()) return out.fail("SENSOR ADD needs ON part (the part it is fixed to)");
        if (!atGiven) s.at = doc.scene.bodies()[static_cast<std::size_t>(bodyIndex(doc, s.body))].bounds().center();
        settings.sensors.push_back(s);
        doc.scene.rigs().get(settings.name) = settings;
        doc.dirty = true;
        out.say(std::string("Sensor ") + s.name + ": " + sensorKindKey(s.kind) + " on " + s.body);
        return out;
    }
    const std::string name = word();
    SensorSpec* s = findSensor(name);
    if (!s) return out.fail(name.empty() ? "Name the sensor" : "No sensor named " + name);
    std::string err;
    if (sub == "remove" || sub == "delete") {
        const std::string n = s->name;
        settings.sensors.erase(settings.sensors.begin() + (s - settings.sensors.data()));
        doc.scene.rigs().get(settings.name) = settings;
        doc.dirty = true;
        out.say("Removed the sensor " + n);
        return out;
    }
    if (sub == "read") {
        float t = 0;
        if (!timeOption(t)) return out.fail("AT needs seconds");
        SensorReading r;
        if (!readSensor(doc.scene, settings, *s, t, r, &err)) return out.fail(err);
        out.say(s->name + " " + describeReading(r));
        return out;
    }
    if (sub == "capture" || sub == "log") {
        std::string path = isQuoted(i) ? word() : std::string();
        if (path.empty()) {
            while (i < a.size() && lowerOf(a[i]) != "at" && lowerOf(a[i]) != "from" && lowerOf(a[i]) != "to") path += (path.empty() ? "" : " ") + word();
        }
        if (path.empty()) return out.fail(toUpper(sub) + " needs a file");
        if (sub == "capture") {
            float t = 0;
            if (!timeOption(t)) return out.fail("AT needs seconds");
            const auto files = captureSensor(doc.scene, settings, *s, t, path, &err);
            if (files.empty()) return out.fail(err);
            for (const std::string& f : files) out.say("Wrote " + f);
            return out;
        }
        float from = 0, to = doc.scene.physics().duration > 0 ? doc.scene.physics().duration : 10.0f;
        while (i < a.size()) {
            const std::string k = lowerOf(word());
            const auto v = parseNumber(word());
            if (!v || *v < 0) return out.fail("FROM / TO need seconds");
            if (k == "from") from = *v;
            else if (k == "to") to = *v;
            else return out.fail("LOG name file [FROM t0] [TO t1]");
        }
        if (!logSensor(doc.scene, settings, *s, from, to, path, &err)) return out.fail(err);
        out.say("Wrote " + path);
        return out;
    }
    return out.fail("SENSOR ADD | REMOVE | LIST | READ name [AT t] | CAPTURE name file [AT t] | LOG name file [FROM t0] [TO t1]");
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

// A point typed in the document's units ("x,y,z", each a length), in mm.
std::optional<Vec3> parsePointMm(const std::string& text) {
    std::vector<float> comps;
    std::size_t start = 0;
    const std::string body = !text.empty() && text[0] == '@' ? text.substr(1) : text;
    for (std::size_t c = 0; c <= body.size(); ++c)
        if (c == body.size() || body[c] == ',') {
            const auto l = parseLength(body.substr(start, c - start));
            if (!l) return std::nullopt;
            comps.push_back(static_cast<float>(*l));
            start = c + 1;
        }
    if (comps.size() != 3) return std::nullopt;
    return Vec3{comps[0], comps[1], comps[2]};
}

// ARM [robot] MOVE_TO x,y,z | IK x,y,z | HOME, with [TOOL part] [POINT dx,dy,dz]
// [APPROACH dx,dy,dz] [AT t] [SPEED s] [OPTIMIZE].
CommandOutput runArm(Document& doc, const std::vector<std::string>& a, const std::vector<bool>& quoted) {
    CommandOutput out;
    const char* usage = "Usage: ARM [robot] MOVE_TO x,y,z | IK x,y,z | HOME, with [TOOL part] [POINT dx,dy,dz] [APPROACH dx,dy,dz] [AT t] [SPEED 0-1] [OPTIMIZE]";
    std::size_t i = 0;
    std::string robot, err;
    if (!pickRobot(doc, a, quoted, i, robot, &err)) return out.fail(err);
    RigSettings settings;
    if (const RigSettings* r = doc.scene.rigs().find(robot)) settings = *r;
    else settings.name = robot;
    enum class Verb { None, MoveTo, Ik, Home } verb = Verb::None;
    Vec3 goalMm{};
    std::string toolPart;
    Vec3 pointMm{};
    std::optional<Vec3> approach;
    std::optional<float> at;
    double speed = 0.5;
    // The optimize engine plans every move for the least effort.
    bool optimise = settings.engine == simEngineKey(SimEngine::Optimize);
    for (; i < a.size(); ++i) {
        const std::string k = lowerOf(a[i]);
        auto next = [&]() -> std::string { return i + 1 < a.size() ? a[++i] : std::string(); };
        if (k == "move_to" || k == "moveto" || k == "move" || k == "ik") {
            if (k == "move" && i + 1 < a.size() && lowerOf(a[i + 1]) == "to") ++i;
            const auto p = parsePointMm(next());
            if (!p) return out.fail(std::string(k == "ik" ? "IK" : "MOVE_TO") + " needs x,y,z");
            goalMm = *p;
            verb = k == "ik" ? Verb::Ik : Verb::MoveTo;
        } else if (k == "home") {
            verb = Verb::Home;
        } else if (k == "tool") {
            toolPart = next();
            if (toolPart.empty()) return out.fail("TOOL needs a part");
        } else if (k == "point") {
            const auto p = parsePointMm(next());
            if (!p) return out.fail("POINT needs dx,dy,dz (from the tool part's centre)");
            pointMm = *p;
        } else if (k == "approach") {
            const auto p = parseVec3(next());
            if (!p || length(*p) < 1e-6f) return out.fail("APPROACH needs a direction dx,dy,dz");
            approach = *p;
        } else if (k == "at") {
            const auto v = parseNumber(next());
            if (!v || *v < 0) return out.fail("AT needs seconds");
            at = static_cast<float>(*v);
        } else if (k == "speed") {
            const auto v = parseNumber(next());
            if (!v || *v <= 0 || *v > 1) return out.fail("SPEED is a share of the joints' top speeds, 0-1");
            speed = *v;
        } else if (k == "optimize" || k == "optimise") {
            optimise = true;
        } else {
            return out.fail(usage);
        }
    }
    if (verb == Verb::None) return out.fail(usage);
    RobotRig rig;
    if (!buildRig(doc.scene, settings, rig, &err)) return out.fail(err);
    if (rig.joints.empty()) return out.fail("The robot has no joints to move");
    const RobotKinematics kin(rig);

    // The tool: the part named, else the end effector; its point, the part's centre (+ POINT).
    int link = -1;
    if (!toolPart.empty()) {
        const int b = bodyIndex(doc, toolPart);
        if (b < 0) return out.fail("No body named " + toolPart);
        link = rig.linkOf(static_cast<std::size_t>(b));
        if (link <= 0) return out.fail(toolPart + " isn't on a moving part of " + (rig.name.empty() ? std::string("the robot") : rig.name));
    } else {
        const std::vector<int> ee = rig.endEffectors();
        if (ee.empty() || ee.front() <= 0) return out.fail("Tag the tool: RIG TAG part END_EFFECTOR (or give TOOL part)");
        link = ee.front();
    }
    const RigLink& L = rig.links[static_cast<std::size_t>(link)];
    const V3d point{0.5 * (L.boxMin[0] + L.boxMax[0]) + pointMm.x * 1e-3, 0.5 * (L.boxMin[1] + L.boxMax[1]) + pointMm.y * 1e-3,
                    0.5 * (L.boxMin[2] + L.boxMax[2]) + pointMm.z * 1e-3};
    const std::vector<std::size_t> active = kin.chain(link);

    // Start where the program has the arm: after its last key unless AT says when.
    float t0 = 0;
    if (at) {
        t0 = *at;
    } else {
        const auto& motions = doc.scene.motions();
        for (std::size_t j : active) {
            const RigJoint& J = rig.joints[j];
            for (std::size_t m = 0; m < motions.size(); ++m)
                if ((static_cast<int>(m) == J.motion || (!J.drive.empty() && motions[m].name == J.drive)) && !motions[m].keys.empty())
                    t0 = std::max(t0, motions[m].start + motions[m].keys.back().t);
        }
    }
    const std::vector<double> start = kin.withMimics(motionTargets(doc.scene, rig, t0).value);
    CollisionModel collisions(doc.scene, rig, t0);
    collisions.onlyMoving(kin, active);
    const int touching = collisions.allowTouching(kin.forward(start));

    std::vector<double> goal;
    char buf[320];
    if (verb == Verb::Home) {
        goal = start;
        for (std::size_t j : active) goal[j] = 0;
        goal = kin.withMimics(goal);
    } else {
        IkTarget tg;
        tg.link = link;
        tg.point = point;
        tg.target = {goalMm.x * 1e-3, goalMm.y * 1e-3, goalMm.z * 1e-3};
        if (approach) {
            const Vec3 d = normalize(*approach);
            tg.approach = V3d{d.x, d.y, d.z};
            // The tool's own approach axis: from its joint's axis out to the tool point.
            const RigJoint& J = rig.joints[static_cast<std::size_t>(L.parentJoint)];
            V3d ax{point[0] - J.origin[0], point[1] - J.origin[1], point[2] - J.origin[2]};
            const double along = ax[0] * J.axis[0] + ax[1] * J.axis[1] + ax[2] * J.axis[2];
            if (J.type != RigJointType::Prismatic)
                for (std::size_t c = 0; c < 3; ++c) ax[c] -= along * J.axis[c];
            const double n = std::sqrt(ax[0] * ax[0] + ax[1] * ax[1] + ax[2] * ax[2]);
            tg.approachAt = n > 1e-6 ? V3d{ax[0] / n, ax[1] / n, ax[2] / n} : J.axis;
        }
        const IkResult ik = solveIk(kin, start, tg, &collisions);
        if (!ik.ok) return out.fail("Can't reach (" + fmtMm(goalMm.x) + ", " + fmtMm(goalMm.y) + ", " + fmtMm(goalMm.z) + ") mm: " + ik.why);
        goal = ik.q;
        if (verb == Verb::Ik) {
            std::snprintf(buf, sizeof buf, "Reaches (%s, %s, %s) mm within %.2f mm:", fmtMm(goalMm.x).c_str(), fmtMm(goalMm.y).c_str(), fmtMm(goalMm.z).c_str(),
                          ik.error * 1e3);
            out.say(buf);
            for (std::size_t j : active) {
                const RigJoint& J = rig.joints[j];
                const bool lin = J.type == RigJointType::Prismatic;
                std::snprintf(buf, sizeof buf, "  %-34s %8.2f %s", J.name.c_str(), goal[j] * (lin ? 1e3 : 180.0 / 3.14159265358979), lin ? "mm" : "deg");
                out.say(buf);
            }
            return out;
        }
    }
    const PlanResult plan = planPath(kin, collisions, start, goal, active);
    if (!plan.ok) return out.fail("No way there: " + plan.why);
    TimedPath timed = timePath(kin, plan.path, speed);
    const double before = timed.effort;
    if (optimise) timed = optimisePath(kin, collisions, timed, active);
    const std::vector<std::string> keyed = writeJointPath(doc.scene, rig, timed, t0, active);
    doc.dirty = true;
    std::string names;
    for (const std::string& k : keyed) names += (names.empty() ? "" : ", ") + k;
    const double dur = timed.times.empty() ? 0.0 : timed.times.back();
    if (verb == Verb::Home)
        std::snprintf(buf, sizeof buf, "Arm goes home from %.2f s to %.2f s", t0, t0 + dur);
    else
        std::snprintf(buf, sizeof buf, "Arm moves the tool to (%s, %s, %s) mm from %.2f s to %.2f s", fmtMm(goalMm.x).c_str(), fmtMm(goalMm.y).c_str(),
                      fmtMm(goalMm.z).c_str(), t0, t0 + dur);
    out.say(std::string(buf) + (timed.smooth ? std::string(" on a smooth")
                                       : timed.q.size() == 2 ? std::string(" in one straight move on a")
                                                             : " in " + std::to_string(timed.q.size() - 1) + " straight moves on a") +
            " collision-free path; keys on " + (names.empty() ? std::string("nothing (its joints have no motions)") : names));
    if (touching > 0) out.say("  note: " + std::to_string(touching) + " pair(s) of parts touching where it starts were left out of the collision checks");
    if (optimise && before > 0) {
        std::snprintf(buf, sizeof buf, "  optimised: effort %.3g -> %.3g N^2 m^2 s (%+.0f%%)", before, timed.effort, 100.0 * (timed.effort - before) / before);
        out.say(buf);
    }
    return out;
}

// RIG [robot] ENV [COUNT n] [TASK t] [TARGET x,y,z] [SECONDS s] [TORQUE] [SEED n]: a batch of
// copies run headless as fast as the machine goes, on random actions; how fast, and how
// the episodes went (a check before training from Python).
CommandOutput runEnv(Document& doc, const RigSettings& settings, const std::vector<std::string>& a, std::size_t i) {
    CommandOutput out;
    EnvOptions o;
    o.count = 16;
    double seconds = 5;
    for (; i < a.size(); ++i) {
        const std::string k = lowerOf(a[i]);
        auto next = [&]() -> std::string { return i + 1 < a.size() ? a[++i] : std::string(); };
        if (k == "count" || k == "envs") {
            const auto v = parseNumber(next());
            if (!v || *v < 1 || *v > 4096) return out.fail("COUNT is 1-4096 copies");
            o.count = static_cast<int>(*v);
        } else if (k == "task") {
            if (!envTaskFromKey(next(), o.task)) return out.fail("TASK is STAND, WALK, REACH, HOVER or NONE");
        } else if (k == "target") {
            const auto p = parsePointMm(next());
            if (!p) return out.fail("TARGET needs x,y,z");
            o.target = {p->x * 1e-3, p->y * 1e-3, p->z * 1e-3};
        } else if (k == "seconds") {
            const auto v = parseNumber(next());
            if (!v || *v <= 0) return out.fail("SECONDS needs a simulated time per copy");
            seconds = *v;
        } else if (k == "torque") {
            o.torque = true;
        } else if (k == "seed") {
            const auto v = parseNumber(next());
            if (!v || *v < 0) return out.fail("SEED needs a number");
            o.seed = static_cast<std::uint64_t>(*v);
        } else {
            return out.fail("Usage: RIG [robot] ENV [COUNT n] [TASK STAND|WALK|REACH|HOVER|NONE] [TARGET x,y,z] [SECONDS s] [TORQUE] [SEED n]");
        }
    }
    o.episodeSeconds = std::min(o.episodeSeconds, seconds);
    std::string err;
    auto env = RobotEnv::create(doc.scene, settings, o, &err);
    if (!env) return out.fail(err);
    const int steps = std::max(1, static_cast<int>(std::lround(seconds / o.controlDt)));
    std::uint64_t s = o.seed * 0x9E3779B97F4A7C15ull + 1;
    std::vector<double> actions(static_cast<std::size_t>(env->count()) * env->actionSize());
    int episodes = 0, falls = 0;
    double returns = 0;
    const auto t0 = std::chrono::steady_clock::now();
    for (int k = 0; k < steps; ++k) {
        for (double& x : actions) {
            s ^= s << 13, s ^= s >> 7, s ^= s << 17;
            x = 0.3 * (static_cast<double>(s >> 11) * (2.0 / 9007199254740992.0) - 1.0);
        }
        const EnvStep st = env->step(actions);
        for (const EnvStep::Episode& e : st.episodes) {
            ++episodes;
            returns += e.ret;
            falls += st.terminated[static_cast<std::size_t>(e.env)] ? 1 : 0;
        }
    }
    const double wall = std::max(1e-6, std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
    char buf[320];
    std::snprintf(buf, sizeof buf, "%d copies of %s (%s, %zu motors, %zu observations): %.0f s simulated in %.2f s, %.0fx real time, %.0f actions/s",
                  env->count(), env->rig().name.empty() ? "the robot" : env->rig().name.c_str(), envTaskKey(o.task), env->actionSize(),
                  env->observationSize(), env->simulatedSeconds(), wall, env->simulatedSeconds() / wall, steps * env->count() / wall);
    out.say(buf);
    if (episodes > 0) {
        std::snprintf(buf, sizeof buf, "  %d episodes ended (%d fell), mean return %.3g on random actions", episodes, falls, returns / episodes);
        out.say(buf);
    }
    out.say("  train from Python: python/sicdm_env.py (VecEnv) through sicdm-mcp");
    return out;
}

} // namespace

CommandOutput runRobotCommand(Document& doc, const std::string& cmd, const std::vector<std::string>& a, const std::vector<bool>& quoted) {
    CommandOutput out;
    if (cmd == "TERRAIN") return runTerrain(doc, a);
    if (cmd == "ARM") return runArm(doc, a, quoted);
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
        if (k == "sensor" || k == "sensors") return runSensor(doc, settings, a, quoted, i + 1);
        if (k == "env") return runEnv(doc, settings, a, i + 1);
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
        } else if (k == "mission") {
            // MISSION CLEAR | HOVER | VELOCITY vx,vy,vz [YAW deg/s] | GOTO x,y,z [YAW deg/s] | DRIVE speed [TURN deg/s], each [AT t]
            const std::string what = lowerOf(next());
            if (what == "clear") {
                settings.mission.clear();
                changed = true;
                continue;
            }
            MissionStep st;
            if (what == "hover") st.kind = MissionKind::Hover;
            else if (what == "velocity" || what == "fly") {
                st.kind = MissionKind::Velocity;
                const auto v = parseVec3(next());
                if (!v) return out.fail("VELOCITY needs vx,vy,vz in m/s");
                st.vec = *v;
            } else if (what == "goto" || what == "to") {
                st.kind = MissionKind::Goto;
                const std::string p = next();
                const auto v = parseVec3(p);
                if (!v) return out.fail("GOTO needs x,y,z");
                // Lengths in the document's units, like any typed point.
                std::vector<float> comps;
                std::size_t start = 0;
                const std::string body = p[0] == '@' ? p.substr(1) : p;
                for (std::size_t c = 0; c <= body.size(); ++c)
                    if (c == body.size() || body[c] == ',') {
                        const auto l = parseLength(body.substr(start, c - start));
                        if (!l) return out.fail("GOTO needs x,y,z");
                        comps.push_back(static_cast<float>(*l));
                        start = c + 1;
                    }
                if (comps.size() != 3) return out.fail("GOTO needs x,y,z");
                st.vec = {comps[0], comps[1], comps[2]};
            } else if (what == "drive") {
                st.kind = MissionKind::Drive;
                const auto v = parseNumber(next());
                if (!v) return out.fail("DRIVE needs a speed in m/s");
                st.vec = {*v, 0, 0};
            } else {
                return out.fail("MISSION is CLEAR, HOVER, VELOCITY vx,vy,vz, GOTO x,y,z or DRIVE speed (each [AT t])");
            }
            // Options of the step.
            while (i + 1 < a.size()) {
                const std::string o = lowerOf(a[i + 1]);
                if (o == "yaw" || o == "turn") {
                    ++i;
                    const auto v = parseNumber(next());
                    if (!v) return out.fail("YAW / TURN needs deg/s");
                    st.rate = *v;
                } else if (o == "at") {
                    ++i;
                    const auto v = parseNumber(next());
                    if (!v || *v < 0) return out.fail("AT needs seconds");
                    st.t = *v;
                } else {
                    break;
                }
            }
            settings.mission.push_back(st);
            std::stable_sort(settings.mission.begin(), settings.mission.end(), [](const MissionStep& x, const MissionStep& y) { return x.t < y.t; });
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
                            "[MISSION ...] [SIMULATE ON|OFF] [STATE [AT t]] [EXPORT file]");
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
    for (const MissionStep& st : settings.mission) {
        char buf[200];
        switch (st.kind) {
        case MissionKind::Hover: std::snprintf(buf, sizeof buf, "  mission %6.2f s: hover", st.t); break;
        case MissionKind::Velocity:
            std::snprintf(buf, sizeof buf, "  mission %6.2f s: fly at (%g, %g, %g) m/s, yaw %g deg/s", st.t, st.vec.x, st.vec.y, st.vec.z, st.rate);
            break;
        case MissionKind::Goto: std::snprintf(buf, sizeof buf, "  mission %6.2f s: go to (%g, %g, %g) mm", st.t, st.vec.x, st.vec.y, st.vec.z); break;
        case MissionKind::Drive: std::snprintf(buf, sizeof buf, "  mission %6.2f s: drive at %g m/s, turning %g deg/s", st.t, st.vec.x, st.rate); break;
        }
        out.say(buf);
    }
    return out;
}

} // namespace sicdm

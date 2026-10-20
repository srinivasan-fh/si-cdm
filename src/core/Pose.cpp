#include "core/Pose.hpp"

#include "core/Motion.hpp"
#include "core/Scene.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <map>
#include <set>
#include <sstream>

namespace sicdm {

namespace {

constexpr float kMinMove = 0.6f;    // s: the shortest posture change (motors could go faster)
constexpr float kPeak = 1.875f;     // minimum-jerk peak speed / average speed
constexpr float kStanceStep = 0.04f; // s between stance samples

std::string lower(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

std::vector<std::string> words(const std::string& s) {
    std::vector<std::string> out;
    std::istringstream in(s);
    for (std::string w; in >> w;) out.push_back(w);
    return out;
}

std::string join(const std::vector<std::string>& w, std::size_t from = 0) {
    std::string s;
    for (std::size_t i = from; i < w.size(); ++i) s += (s.empty() ? "" : " ") + w[i];
    return s;
}

// "RIGHT_ELBOW", "r-elbow", "Right Index  MCP" -> "right elbow", "right index mcp".
std::string normalizeId(std::string s) {
    s = lower(std::move(s));
    for (char& c : s)
        if (c == '_' || c == '-' || c == '.') c = ' ';
    std::vector<std::string> w = words(s);
    if (!w.empty() && (w[0] == "r" || w[0] == "rt")) w[0] = "right";
    if (!w.empty() && (w[0] == "l" || w[0] == "lt")) w[0] = "left";
    return join(w);
}

bool isSide(const std::string& w) { return w == "right" || w == "left"; }

// A posture key names this joint: the joint id, or a run of its words ("elbow",
// "right index", "index mcp"); "*" names every joint.
bool keyMatches(const std::string& key, const std::string& id) {
    if (key == "*") return true;
    if (key == id) return true;
    const std::vector<std::string> k = words(key), j = words(id);
    if (k.empty() || k.size() > j.size()) return false;
    if (isSide(k[0]) && k[0] != j[0]) return false;
    for (std::size_t at = 0; at + k.size() <= j.size(); ++at)
        if (std::equal(k.begin(), k.end(), j.begin() + static_cast<std::ptrdiff_t>(at))) return true;
    return false;
}

const Body* bodyNamed(const Scene& s, const std::string& name) {
    for (const Body& b : s.bodies())
        if (b.name == name) return &b;
    return nullptr;
}

const Motion* motionNamed(const Scene& s, const std::string& name) {
    for (const Motion& m : s.motions())
        if (m.name == name) return &m;
    return nullptr;
}

Motion* motionNamed(Scene& s, const std::string& name) {
    for (Motion& m : s.motions())
        if (m.name == name) return &m;
    return nullptr;
}

// Side of a one-sided robot (arm, hand, thumb, leg) for its joint ids.
std::string robotSide(const Scene& s, const std::string& comp) {
    if (const Body* palm = bodyNamed(s, comp + " Palm"); palm && palm->params.size() > 3)
        return palm->params[3] >= 0.5f ? "left" : "right";
    if (const Body* thumb = bodyNamed(s, comp + " Thumb Metacarpal"); thumb && thumb->params.size() > 3 && !bodyNamed(s, comp + " Index Proximal"))
        return thumb->params[3] < 0.0f ? "left" : "right";
    const Body* thigh = bodyNamed(s, comp + " Thigh");
    const Body* mount = bodyNamed(s, comp + " Hip Mount");
    if (thigh && mount) return thigh->offset.x < mount->offset.x ? "right" : "left"; // faces -Y: its right is at -X
    return "right";
}

// Standard id of a joint motion: "<comp> Right Arm Index MCP" -> "right index mcp",
// "<comp> Elbow" on a left arm -> "left elbow".
std::string jointId(const std::string& motion, const std::string& comp, const std::string& side) {
    std::vector<std::string> w = words(lower(motion.substr(comp.size())));
    if (w.size() >= 2 && isSide(w[0]) && w[1] == "arm") w.erase(w.begin() + 1);
    if (!w.empty() && !isSide(w[0]) && w[0] != "neck" && w[0] != "waist" && w[0] != "trunk") w.insert(w.begin(), side);
    return join(w);
}

// Drive angle that holds a posture: the end of its keys (0 without keys).
float heldDriveValue(const Motion& d) { return d.keys.empty() ? 0.0f : d.keys.back().value; }

float clampJoint(const RobotJoint& j, float a) { return std::clamp(a, j.lo, j.hi); }

std::string fmtNum(float v) {
    char b[32];
    std::snprintf(b, sizeof(b), "%g", static_cast<double>(std::round(v * 100.0f) / 100.0f));
    return b;
}

// ---- level feet
// The pelvis carries the legs, so a sole's orientation is the product of the hip,
// knee and ankle rotations. Ankle joints a posture does not name are solved so the
// sole stays level (its normal straight up).

// A leg's hip, knee and ankle joints, outermost first.
std::vector<const RobotJoint*> legChain(const Robot& robot, const std::string& side) {
    std::vector<const RobotJoint*> chain;
    for (const RobotJoint& j : robot.joints)
        if (j.id.rfind(side + " hip ", 0) == 0 || j.id == side + " knee" || j.id.rfind(side + " ankle ", 0) == 0)
            chain.push_back(&j);
    std::sort(chain.begin(), chain.end(), [](const RobotJoint* a, const RobotJoint* b) { return a->order < b->order; });
    return chain;
}

bool isAnkle(const RobotJoint& j) { return j.id.find(" ankle ") != std::string::npos; }

// Angles of the `free` ankle joints (clamped to their stops) that level the sole, with
// the other joints at angleOf(joint). Returns the sole's remaining tilt in degrees.
template <class AngleOf>
float levelSole(const std::vector<const RobotJoint*>& chain, std::map<const RobotJoint*, float>& free, AngleOf angleOf) {
    auto normal = [&] {
        Mat4 r = Mat4::identity();
        for (const RobotJoint* j : chain) {
            const auto f = free.find(j);
            r = r * rotateAxis(j->axis, radians(f != free.end() ? f->second : angleOf(*j)));
        }
        return normalize(r.transformDir({0, 0, 1}));
    };
    std::vector<const RobotJoint*> vars;
    for (const auto& [j, a] : free) vars.push_back(j);
    // Gauss-Newton on the normal's horizontal part, from the planar answer (exact when
    // the leg moves in one plane): pitch = hip flex - knee, roll = hip abduction.
    for (const RobotJoint* a : vars) {
        const std::string side = a->id.substr(0, a->id.find(' '));
        float hip = 0.0f, knee = 0.0f, abd = 0.0f;
        for (const RobotJoint* j : chain) {
            if (j->id == side + " hip flex") hip = angleOf(*j);
            if (j->id == side + " knee") knee = angleOf(*j);
            if (j->id == side + " hip abduction") abd = angleOf(*j);
        }
        free[a] = a->id == side + " ankle pitch" ? hip - knee : a->id == side + " ankle roll" ? abd : 0.0f;
    }
    for (int it = 0; it < 12 && !vars.empty() && vars.size() <= 2; ++it) {
        const Vec3 n0 = normal();
        if (n0.x * n0.x + n0.y * n0.y < 1e-14f) break;
        std::vector<std::pair<float, float>> col; // d(n.x, n.y) / d angle
        for (const RobotJoint* a : vars) {
            free[a] += 0.01f;
            const Vec3 n1 = normal();
            free[a] -= 0.01f;
            col.push_back({(n1.x - n0.x) / 0.01f, (n1.y - n0.y) / 0.01f});
        }
        if (vars.size() == 1) {
            const float jj = col[0].first * col[0].first + col[0].second * col[0].second;
            if (jj < 1e-12f) break;
            free[vars[0]] -= (col[0].first * n0.x + col[0].second * n0.y) / jj;
        } else {
            const float a = col[0].first, b = col[1].first, c = col[0].second, d = col[1].second;
            const float det = a * d - b * c;
            if (std::fabs(det) < 1e-12f) break;
            free[vars[0]] -= (d * n0.x - b * n0.y) / det;
            free[vars[1]] -= (-c * n0.x + a * n0.y) / det;
        }
    }
    for (const RobotJoint* a : vars) free[a] = clampJoint(*a, std::remainder(free[a], 360.0f));
    return std::acos(std::clamp(normal().z, -1.0f, 1.0f)) * 57.29578f;
}

// Drive targets for a posture applied on top of the current drive values. Records
// how far each named joint ends up from what was asked (limits, coupling).
struct Solved {
    std::map<std::string, float> drives; // drive -> angle
    std::vector<std::string> notes;
    std::set<std::string> levelled; // ankle joints solved to keep a sole level
    std::set<std::string> named;    // joints the posture names (not just by "*")
    int joints = 0;
};

Solved solvePosture(const Robot& robot, const Posture& p, std::map<std::string, float> drives) {
    Solved out;
    // Requested angle per joint: later entries override earlier ones.
    std::map<std::string, float> want;
    std::vector<std::string> order;
    std::set<std::string> named; // joints named by id or group, not just by "*"
    for (const auto& [rawKey, angle] : p) {
        const std::string key = rawKey == "*" ? "*" : normalizeId(rawKey);
        for (const RobotJoint& j : robot.joints)
            if (keyMatches(key, j.id)) {
                if (!want.count(j.id)) order.push_back(j.id);
                want[j.id] = angle;
                if (key != "*") named.insert(j.id);
            }
    }
    // Level feet: unless the posture names an ankle, it keeps the sole flat.
    auto current = [&](const RobotJoint& j) {
        if (const auto w = want.find(j.id); w != want.end()) return clampJoint(j, w->second);
        return clampJoint(j, drives[j.drive] * j.ratio);
    };
    for (const char* side : {"right", "left"}) {
        const auto chain = legChain(robot, side);
        std::map<const RobotJoint*, float> free;
        bool moved = false;
        for (const RobotJoint* j : chain) {
            if (!isAnkle(*j)) moved = moved || want.count(j->id);
            else if (!named.count(j->id)) free[j] = 0.0f;
        }
        if (!moved || free.empty()) continue;
        const float tilt = levelSole(chain, free, current);
        for (const auto& [a, angle] : free) {
            if (!want.count(a->id)) order.push_back(a->id);
            want[a->id] = angle;
            out.levelled.insert(a->id);
        }
        if (tilt > 0.5f) out.notes.push_back(std::string(side) + " foot tilts " + fmtNum(tilt) + " deg: its ankle reaches a hard stop");
    }
    out.joints = static_cast<int>(want.size());
    out.named = named;
    // Each named motor is set to the angle that best fits all its named joints (least
    // squares): exact for a single joint, the closest compromise for an underactuated
    // finger whose three joints share one motor.
    std::map<std::string, std::pair<float, float>> fit; // drive -> (sum r*angle, sum r*r)
    for (const std::string& id : order) {
        const RobotJoint& j = *std::find_if(robot.joints.begin(), robot.joints.end(), [&](const RobotJoint& r) { return r.id == id; });
        const float asked = want[id];
        const float angle = clampJoint(j, asked);
        if (std::fabs(angle - asked) > 1e-3f)
            out.notes.push_back(j.id + " " + fmtNum(asked) + " deg is past its hard stop: " + fmtNum(angle) + " deg");
        if (j.ratio == 0.0f) continue;
        fit[j.drive].first += j.ratio * angle;
        fit[j.drive].second += j.ratio * j.ratio;
    }
    for (const auto& [drive, sums] : fit) drives[drive] = sums.first / sums.second;
    // Joints that share a motor move together: report how far the best fit lands.
    int off = 0;
    float worst = 0.0f;
    std::string worstText;
    for (const std::string& id : order) {
        const RobotJoint& j = *std::find_if(robot.joints.begin(), robot.joints.end(), [&](const RobotJoint& r) { return r.id == id; });
        const float got = clampJoint(j, drives[j.drive] * j.ratio), asked = clampJoint(j, want[id]);
        if (std::fabs(got - asked) <= 0.5f) continue;
        ++off;
        if (std::fabs(got - asked) > worst) {
            worst = std::fabs(got - asked);
            worstText = j.id + " " + fmtNum(got) + " for " + fmtNum(asked);
        }
    }
    if (off > 0)
        out.notes.push_back(std::to_string(off) + (off == 1 ? " joint shares" : " joints share") +
                            " a motor with others: best fit, at most " + fmtNum(worst) + " deg off (" + worstText + ")");
    out.drives = std::move(drives);
    return out;
}

std::map<std::string, float> heldDrives(const Scene& scene, const Robot& robot) {
    std::map<std::string, float> d;
    for (const RobotJoint& j : robot.joints)
        if (const Motion* m = motionNamed(scene, j.drive)) d[j.drive] = heldDriveValue(*m);
    return d;
}

float ratedSpeed(const Scene& scene, const std::string& drive) {
    const Motion* m = motionNamed(scene, drive);
    return m ? std::max(std::fabs(m->speed), 1.0f) : 360.0f;
}

// Joints and drives of the robot: enabled, joints following, drives keyed from t = 0.
void claim(Scene& scene, const Robot& robot) {
    for (const RobotJoint& j : robot.joints) {
        if (Motion* m = motionNamed(scene, j.motion)) {
            m->enabled = true;
            m->keys.clear();
        }
        if (Motion* d = motionNamed(scene, j.drive)) {
            d->enabled = true;
            d->start = 0.0f;
            d->range = 0.0f;
            d->keySpline = false;
        }
    }
}

int countMatches(const Robot& robot, const Posture& p) {
    std::set<std::string> hit;
    for (const auto& [rawKey, angle] : p) {
        if (rawKey == "*") continue;
        const std::string key = normalizeId(rawKey);
        for (const RobotJoint& j : robot.joints)
            if (keyMatches(key, j.id)) hit.insert(j.id);
    }
    return static_cast<int>(hit.size());
}

// Keeps a humanoid's lowest sole on the ground: a vertical stance motion, outermost,
// carries the whole robot so it sinks into a squat instead of lifting its feet.
void plantFeet(Scene& scene, const std::string& comp, float duration, bool loop) {
    const std::string name = comp + " Stance";
    auto& motions = scene.motions();
    motions.erase(std::remove_if(motions.begin(), motions.end(), [&](const Motion& m) { return m.name == name; }), motions.end());
    std::vector<std::size_t> soles;
    std::vector<std::string> all;
    for (std::size_t i = 0; i < scene.bodies().size(); ++i) {
        const Body& b = scene.bodies()[i];
        if (b.component != comp) continue;
        all.push_back(b.name);
        if (b.name.size() > 5 && b.name.compare(b.name.size() - 5, 5, " Sole") == 0) soles.push_back(i);
    }
    if (soles.empty() || duration <= 0.0f) return;
    float ground = 1e30f;
    for (std::size_t i : soles)
        for (const Vertex& v : scene.bodies()[i].mesh.vertices) ground = std::min(ground, v.position.z + scene.bodies()[i].offset.z);
    const auto movers = bodyMotions(scene);
    const int n = std::max(2, static_cast<int>(std::ceil(duration / kStanceStep)));
    std::vector<MotionKey> keys;
    float largest = 0.0f;
    for (int s = 0; s <= n; ++s) {
        const float t = duration * static_cast<float>(s) / static_cast<float>(n);
        float lowest = 1e30f;
        for (std::size_t i : soles) {
            const Body& b = scene.bodies()[i];
            const Mat4 tf = bodyMotionTransform(scene, i, t, static_cast<std::size_t>(-1), movers[i]);
            for (const Vertex& v : b.mesh.vertices) lowest = std::min(lowest, tf.transformPoint(v.position + b.offset).z);
        }
        keys.push_back({t, ground - lowest});
        largest = std::max(largest, std::fabs(ground - lowest));
    }
    if (largest < 0.01f) return;
    Motion stance;
    stance.name = name;
    stance.kind = MotionKind::Linear;
    stance.axis = {0, 0, 1};
    stance.bodies = all;
    stance.keys = std::move(keys);
    stance.keySpline = true;
    stance.mode = loop ? MotionMode::Loop : MotionMode::Once;
    stance.speed = 0.0f;
    motions.insert(motions.begin(), std::move(stance));
}

// Keeps the levelled ankles level all along the move, not only at its keys: their
// drives are re-keyed every kStanceStep from the hip and knee angles at that time
// (a smooth spline through the samples).
void levelAlongPath(Scene& scene, const Robot& robot, const std::set<std::string>& ankles, float duration, bool loop) {
    if (ankles.empty() || duration <= 0.0f) return;
    for (const char* side : {"right", "left"}) {
        const auto chain = legChain(robot, side);
        std::vector<const RobotJoint*> free;
        for (const RobotJoint* j : chain)
            if (ankles.count(j->id)) free.push_back(j);
        if (free.empty()) continue;
        std::map<const RobotJoint*, const Motion*> motionOf;
        for (const RobotJoint* j : chain) motionOf[j] = motionNamed(scene, j->motion);
        const int n = std::max(2, static_cast<int>(std::ceil(duration / kStanceStep)));
        std::map<const RobotJoint*, std::vector<MotionKey>> keys;
        for (int k = 0; k <= n; ++k) {
            const float t = duration * static_cast<float>(k) / static_cast<float>(n);
            std::map<const RobotJoint*, float> angles;
            for (const RobotJoint* a : free) angles[a] = 0.0f;
            levelSole(chain, angles, [&](const RobotJoint& j) {
                const Motion* m = motionOf[&j];
                return m ? motionValue(scene, *m, t) : 0.0f;
            });
            for (const RobotJoint* a : free) keys[a].push_back({t, a->ratio != 0.0f ? angles[a] / a->ratio : 0.0f});
        }
        for (const RobotJoint* a : free) {
            Motion* d = motionNamed(scene, a->drive);
            if (!d || a->ratio == 0.0f) continue;
            d->keys = std::move(keys[a]);
            d->keySpline = true;
            d->mode = loop ? MotionMode::Loop : MotionMode::Once;
        }
    }
}

// ---------------------------------------------------------------- presets
// Sign conventions (from the standing pose): + flexion / abduction / internal
// rotation / plantarflexion / inversion / toe extension / ulnar deviation; neck
// yaw + turns left, neck roll + tilts right; forearm rotation + supinates from
// the pronated zero (palm back when hanging).

void add(Posture& p, const std::string& id, float v) { p.emplace_back(id, v); }

void fingers(Posture& p, const std::string& side, float mcp, float pip, float dip) {
    const std::string s = side.empty() ? "" : side + " ";
    for (const char* f : {"index", "middle", "ring", "little"}) {
        add(p, s + f + " mcp", mcp);
        add(p, s + f + " pip", pip);
        add(p, s + f + " dip", dip);
    }
}
void finger(Posture& p, const std::string& side, const char* f, float mcp, float pip, float dip) {
    const std::string s = side.empty() ? "" : side + " ";
    add(p, s + f + " mcp", mcp);
    add(p, s + f + " pip", pip);
    add(p, s + f + " dip", dip);
}
void thumb(Posture& p, const std::string& side, float cmc, float mcp, float ip) {
    const std::string s = side.empty() ? "" : side + " ";
    add(p, s + "thumb cmc", cmc);
    add(p, s + "thumb mcp", mcp);
    add(p, s + "thumb ip", ip);
}
void spread(Posture& p, const std::string& side, float index, float middle, float ring, float little) {
    const std::string s = side.empty() ? "" : side + " ";
    add(p, s + "index abduction", index);
    add(p, s + "middle abduction", middle);
    add(p, s + "ring abduction", ring);
    add(p, s + "little abduction", little);
}

// Hand shapes ("" = every hand).
Posture handShape(const std::string& shape, const std::string& side = "") {
    Posture p;
    spread(p, side, 0, 0, 0, 0);
    if (shape == "open") {
        fingers(p, side, 0, 0, 0);
        thumb(p, side, 0, 0, 0);
    } else if (shape == "relaxed") {
        fingers(p, side, 15, 25, 10);
        thumb(p, side, 10, 10, 10);
    } else if (shape == "fist") {
        fingers(p, side, 90, 100, 70);
        thumb(p, side, 50, 45, 50);
    } else if (shape == "point") {
        fingers(p, side, 90, 100, 70);
        finger(p, side, "index", 0, 0, 0);
        thumb(p, side, 50, 45, 50);
    } else if (shape == "thumbs_up") {
        fingers(p, side, 90, 100, 70);
        thumb(p, side, 0, 0, 0);
    } else if (shape == "peace") {
        fingers(p, side, 90, 100, 70);
        finger(p, side, "index", 0, 0, 0);
        finger(p, side, "middle", 0, 0, 0);
        spread(p, side, 15, -10, 0, 0);
        thumb(p, side, 50, 40, 40);
    } else if (shape == "ok") {
        fingers(p, side, 10, 10, 5);
        finger(p, side, "index", 40, 70, 50);
        thumb(p, side, 55, 30, 25);
    } else if (shape == "pinch") {
        fingers(p, side, 20, 25, 10);
        finger(p, side, "index", 45, 50, 30);
        thumb(p, side, 50, 25, 20);
    } else if (shape == "grasp") {
        fingers(p, side, 55, 70, 40);
        thumb(p, side, 45, 30, 30);
    } else if (shape == "spread") {
        fingers(p, side, 0, 0, 0);
        thumb(p, side, 0, 0, 0);
        spread(p, side, 20, 0, 10, 20);
    }
    return p;
}

Posture operator+(Posture a, const Posture& b) {
    a.insert(a.end(), b.begin(), b.end());
    return a;
}

Posture stand() { return Posture{{"*", 0.0f}}; }

Posture squat() {
    // Parallel squat: thighs level, shins forward 20 deg (the standard dorsiflexion limit,
    // solved by the feet levelling),
    // trunk leaning over the feet and arms forward to keep the balance.
    return {{"hip flex", 90}, {"knee", 110}, {"trunk flex", 40}, {"shoulder flex", 90}, {"elbow", 10}};
}

Posture rightArm(float flex, float abd, float rot, float elbow, float forearm, float wrist) {
    return {{"right shoulder flex", flex}, {"right shoulder abduction", abd}, {"right shoulder rotation", rot},
            {"right elbow", elbow},        {"right forearm rotation", forearm}, {"right wrist flex", wrist}};
}

Posture waveReady() { return rightArm(0, 85, -85, 95, 90, 0) + handShape("open", "right"); }

Posture marchStep(const char* leg, const char* arm, bool up) {
    const std::string l = leg, a = arm;
    const float k = up ? 1.0f : 0.0f;
    return {{l + " hip flex", 50 * k}, {l + " knee", 75 * k}, {l + " ankle pitch", 15 * k},
            {a + " shoulder flex", 30 * k}, {a + " elbow", 30 * k}};
}

} // namespace

// ---------------------------------------------------------------- catalogue

const std::vector<NamedPosture>& namedPostures() {
    static const std::vector<NamedPosture> list = [] {
        std::vector<NamedPosture> v;
        v.push_back({"stand", "Every joint at zero: standing, arms hanging, hands open", stand()});
        v.push_back({"t_pose", "Arms straight out to the sides", stand() + Posture{{"shoulder abduction", 90}}});
        v.push_back({"a_pose", "Arms 45 deg out to the sides", stand() + Posture{{"shoulder abduction", 45}}});
        v.push_back({"arms_up", "Both arms raised overhead", stand() + Posture{{"shoulder flex", 170}}});
        v.push_back({"arms_forward", "Both arms straight ahead", stand() + Posture{{"shoulder flex", 90}}});
        v.push_back({"squat", "Parallel squat, heels down, arms forward for balance", stand() + squat()});
        v.push_back({"sit", "Seated: hips and knees at 90 deg", stand() + Posture{{"hip flex", 90}, {"knee", 90}}});
        v.push_back({"bow", "Bow from the waist", stand() + Posture{{"trunk flex", 40}, {"neck pitch", 15}}});
        v.push_back({"balance", "Stands on the right leg, left knee raised",
                     stand() + Posture{{"left hip flex", 80}, {"left knee", 90}, {"shoulder abduction", 40}}});
        v.push_back({"point", "Right arm points ahead with the index finger",
                     stand() + rightArm(90, 0, 0, 0, 90, 0) + handShape("point", "right")});
        v.push_back({"wave_ready", "Right hand raised, palm forward, ready to wave", stand() + waveReady()});
        v.push_back({"salute", "Right hand to the brow",
                     stand() + rightArm(80, 20, 30, 130, 90, 0) + handShape("open", "right")});
        v.push_back({"thumbs_up", "Right forearm forward, thumb up",
                     stand() + rightArm(30, 0, 0, 90, 90, 0) + handShape("thumbs_up", "right")});
        v.push_back({"look_left", "Head turned left", Posture{{"neck yaw", 60}}});
        v.push_back({"look_right", "Head turned right", Posture{{"neck yaw", -60}}});
        v.push_back({"look_up", "Head tilted back", Posture{{"neck pitch", -40}}});
        v.push_back({"look_down", "Chin down", Posture{{"neck pitch", 40}}});
        return v;
    }();
    static const std::vector<NamedPosture> withHands = [] {
        std::vector<NamedPosture> v = list;
        static const char* shapes[][2] = {{"open", "Fingers and thumb straight"},
                                          {"relaxed", "Slightly curled, at rest"},
                                          {"fist", "Closed fist"},
                                          {"point", "Index finger pointing"},
                                          {"thumbs_up", "Fist with the thumb up"},
                                          {"peace", "Index and middle finger in a V"},
                                          {"ok", "Thumb and index finger in a ring"},
                                          {"pinch", "Precision pinch: thumb to index"},
                                          {"grasp", "Power grip around a cylinder"},
                                          {"spread", "Fingers spread (fully actuated hands)"}};
        static const char* names[] = {"hand_open", "hand_relaxed", "hand_fist", "hand_point", "hand_thumbs_up",
                                      "hand_peace", "hand_ok", "hand_pinch", "hand_grasp", "hand_spread"};
        for (int i = 0; i < 10; ++i) v.push_back({names[i], shapes[i][1], handShape(shapes[i][0])});
        return v;
    }();
    return withHands;
}

const std::vector<Gesture>& namedGestures() {
    static const std::vector<Gesture> list = [] {
        std::vector<Gesture> g;
        g.push_back({"wave", "Raises the right hand and waves", {
            {0.9f, waveReady()},
            {1.3f, {{"right shoulder rotation", -55}}}, {1.7f, {{"right shoulder rotation", -85}}},
            {2.1f, {{"right shoulder rotation", -55}}}, {2.5f, {{"right shoulder rotation", -85}}},
            {3.4f, rightArm(0, 0, 0, 0, 0, 0)}}});
        g.push_back({"nod", "Nods yes", {
            {0.4f, {{"neck pitch", 20}}}, {0.8f, {{"neck pitch", -5}}}, {1.2f, {{"neck pitch", 20}}}, {1.6f, {{"neck pitch", 0}}}}});
        g.push_back({"shake", "Shakes the head no", {
            {0.35f, {{"neck yaw", 35}}}, {0.85f, {{"neck yaw", -35}}}, {1.35f, {{"neck yaw", 35}}},
            {1.85f, {{"neck yaw", -35}}}, {2.2f, {{"neck yaw", 0}}}}});
        g.push_back({"bow", "Bows and stands up again", {
            {1.0f, {{"trunk flex", 40}, {"neck pitch", 15}}}, {2.0f, {}}, {3.0f, {{"trunk flex", 0}, {"neck pitch", 0}}}}});
        g.push_back({"point", "Points ahead with the right index finger", {{1.0f, rightArm(90, 0, 0, 0, 90, 0) + handShape("point", "right")}}});
        g.push_back({"thumbs_up", "Gives a thumbs up with the right hand",
                     {{1.0f, rightArm(30, 0, 0, 90, 90, 0) + handShape("thumbs_up", "right")}}});
        g.push_back({"salute", "Salutes with the right hand", {
            {1.0f, rightArm(80, 20, 30, 130, 90, 0) + handShape("open", "right")}, {2.5f, {}}, {3.5f, rightArm(0, 0, 0, 0, 0, 0)}}});
        g.push_back({"grasp", "Opens, closes into a fist and opens every hand", {
            {0.8f, handShape("open")}, {1.6f, handShape("fist")}, {2.4f, handShape("open")}}});
        g.push_back({"count", "Counts one to five on every hand", {
            {0.6f, handShape("fist")},
            {1.2f, {{"index mcp", 0}, {"index pip", 0}, {"index dip", 0}}},
            {1.8f, {{"middle mcp", 0}, {"middle pip", 0}, {"middle dip", 0}}},
            {2.4f, {{"ring mcp", 0}, {"ring pip", 0}, {"ring dip", 0}}},
            {3.0f, {{"little mcp", 0}, {"little pip", 0}, {"little dip", 0}}},
            {3.6f, {{"thumb cmc", 0}, {"thumb mcp", 0}, {"thumb ip", 0}}}}});
        g.push_back({"squat", "Squats down and stands up", {{1.2f, squat()}, {2.2f, {}}, {3.4f, stand()}}});
        g.push_back({"look_around", "Looks left, right and back ahead", {
            {1.0f, {{"neck yaw", 60}, {"neck pitch", -10}}}, {2.2f, {{"neck yaw", -60}}}, {3.0f, {{"neck yaw", 0}, {"neck pitch", 0}}}}});
        g.push_back({"stretch", "Stretches both arms overhead", {{1.2f, {{"shoulder flex", 170}, {"elbow", 0}}}, {2.2f, {}},
                                                                 {3.2f, {{"shoulder flex", 0}}}}});
        g.push_back({"march", "Marches in place, arms swinging", {
            {0.0f, marchStep("right", "left", false) + marchStep("left", "right", false)},
            {0.6f, marchStep("right", "left", true)}, {1.2f, marchStep("right", "left", false)},
            {1.8f, marchStep("left", "right", true)}, {2.4f, marchStep("left", "right", false)}}, true});
        return g;
    }();
    return list;
}

const NamedPosture* findPosture(const std::string& name) {
    const std::string n = normalizeId(name);
    for (const NamedPosture& p : namedPostures())
        if (normalizeId(p.name) == n) return &p;
    return nullptr;
}

const Gesture* findGesture(const std::string& name) {
    const std::string n = normalizeId(name);
    for (const Gesture& g : namedGestures())
        if (normalizeId(g.name) == n) return &g;
    return nullptr;
}

Posture mirrored(const Posture& p) {
    Posture out;
    for (auto [key, v] : p) {
        std::vector<std::string> w = words(normalizeId(key));
        if (!w.empty() && isSide(w[0])) w[0] = w[0] == "right" ? "left" : "right";
        const std::string k = key == "*" ? "*" : join(w);
        if (k == "neck yaw" || k == "neck roll" || k == "waist rotation" || k == "trunk bend") v = -v;
        out.emplace_back(k, v);
    }
    return out;
}

// ---------------------------------------------------------------- robots

std::vector<std::string> robotNames(const Scene& scene) {
    std::vector<std::string> out;
    for (const Body& b : scene.bodies())
        if (!b.component.empty() && std::find(out.begin(), out.end(), b.component) == out.end()) {
            const auto r = findRobot(scene, b.component);
            if (r && !r->joints.empty()) out.push_back(b.component);
        }
    return out;
}

std::optional<Robot> findRobot(const Scene& scene, const std::string& what) {
    std::string comp;
    for (const Body& b : scene.bodies())
        if (lower(b.component) == lower(what) || lower(b.name) == lower(what)) {
            comp = b.component;
            break;
        }
    if (comp.empty()) return std::nullopt;
    Robot r;
    r.component = comp;
    const std::string side = robotSide(scene, comp);
    const std::string prefix = comp + " ";
    for (std::size_t k = 0; k < scene.motions().size(); ++k) {
        const Motion& m = scene.motions()[k];
        if (m.follow.empty() || !m.hasLimits || m.kind != MotionKind::Circular || m.name.rfind(prefix, 0) != 0) continue;
        const Motion* d = motionNamed(scene, m.follow);
        if (!d || d->name.rfind(prefix, 0) != 0) continue;
        r.joints.push_back({jointId(m.name, comp, side), m.name, d->name, m.ratio, m.lo, m.hi, m.axis, k});
    }
    if (r.joints.empty()) return std::nullopt;
    return r;
}

std::vector<std::pair<RobotJoint, float>> robotAngles(const Scene& scene, const Robot& robot, float t) {
    std::vector<std::pair<RobotJoint, float>> out;
    for (const RobotJoint& j : robot.joints)
        if (const Motion* m = motionNamed(scene, j.motion)) out.emplace_back(j, motionValue(scene, *m, t));
    return out;
}

MoveResult poseRobot(Scene& scene, const Robot& robot, const Posture& target, float seconds) {
    MoveResult r;
    Posture p = target;
    const int direct = countMatches(robot, p), other = countMatches(robot, mirrored(p));
    if (direct == 0 && other > 0) { // e.g. a right-hand posture on a left hand
        p = mirrored(p);
        r.notes.push_back("played with the other side (this robot has no matching joints on that side)");
    }
    const auto start = heldDrives(scene, robot);
    Solved s = solvePosture(robot, p, start);
    r.joints = s.joints;
    if (s.joints == 0) {
        r.ok = false;
        r.error = "None of the named joints exist on " + robot.component + " (POSE " + robot.component + " JOINTS lists them)";
        return r;
    }
    r.notes.insert(r.notes.end(), s.notes.begin(), s.notes.end());
    float needed = 0.0f;
    for (const auto& [drive, value] : s.drives)
        needed = std::max(needed, kPeak * std::fabs(value - start.at(drive)) / ratedSpeed(scene, drive));
    float duration = std::max(needed, kMinMove);
    if (seconds > 0.0f) {
        if (seconds + 1e-4f < needed)
            r.notes.push_back("took " + fmtNum(needed) + " s instead of " + fmtNum(seconds) + " s: the motors cannot go faster");
        duration = std::max(seconds, needed);
    }
    claim(scene, robot);
    for (const auto& [drive, value] : s.drives) {
        Motion* d = motionNamed(scene, drive);
        if (!d) continue;
        const float from = start.at(drive);
        d->keys = {{0.0f, from}, {duration, value}};
        d->mode = MotionMode::Once;
    }
    r.duration = duration;
    levelAlongPath(scene, robot, s.levelled, duration, false);
    plantFeet(scene, robot.component, duration, false);
    return r;
}

MoveResult playGesture(Scene& scene, const Robot& robot, const Gesture& gesture, float timeScale, std::optional<bool> loopOverride,
                       bool mirror) {
    MoveResult r;
    std::vector<std::pair<float, Posture>> frames = gesture.frames;
    Posture all;
    for (const auto& f : frames) all = all + f.second;
    if (!mirror && countMatches(robot, all) == 0 && countMatches(robot, mirrored(all)) > 0) {
        mirror = true;
        r.notes.push_back("played with the other side (this robot has no matching joints on that side)");
    }
    if (mirror) {
        for (auto& f : frames) f.second = mirrored(f.second);
        all = mirrored(all);
    }
    if (countMatches(robot, all) == 0) {
        r.ok = false;
        r.error = std::string("The ") + gesture.name + " gesture moves no joint of " + robot.component;
        return r;
    }
    const bool loop = loopOverride.value_or(gesture.loop);
    const float scale = timeScale > 0.0f ? timeScale : 1.0f;

    // Drive angles frame by frame; a looping gesture starts from its own first frame.
    std::map<std::string, float> state = heldDrives(scene, robot);
    std::vector<std::pair<float, std::map<std::string, float>>> keyed;
    if (!loop || frames.empty() || frames.front().first > 0.0f) keyed.push_back({0.0f, state});
    std::set<std::string> touched, levelled, named;
    for (const auto& [t, posture] : frames) {
        Solved s = solvePosture(robot, posture, state);
        for (const std::string& n : s.notes)
            if (std::find(r.notes.begin(), r.notes.end(), n) == r.notes.end()) r.notes.push_back(n);
        for (const auto& [drive, value] : s.drives)
            if (std::fabs(value - state[drive]) > 1e-6f) touched.insert(drive);
        state = s.drives;
        r.joints = std::max(r.joints, s.joints);
        keyed.push_back({t * scale, state});
        levelled.insert(s.levelled.begin(), s.levelled.end());
        named.insert(s.named.begin(), s.named.end());
    }
    for (const std::string& id : named) levelled.erase(id); // an ankle the gesture sets itself
    // Lengthen any step a motor could not make at its rated speed (the others keep their timing).
    float added = 0.0f, prev = 0.0f;
    for (std::size_t i = 1; i < keyed.size(); ++i) {
        const float dt = keyed[i].first - prev;
        prev = keyed[i].first;
        float needed = 0.0f;
        for (const auto& [drive, value] : keyed[i].second)
            needed = std::max(needed, kPeak * std::fabs(value - keyed[i - 1].second.at(drive)) / ratedSpeed(scene, drive));
        keyed[i].first = keyed[i - 1].first + std::max(dt, needed);
        added += std::max(0.0f, needed - dt);
    }
    if (added > 1e-3f)
        r.notes.push_back("took " + fmtNum(added) + " s longer: some steps were too fast for the motors' rated speed");
    claim(scene, robot);
    for (const RobotJoint& j : robot.joints) {
        Motion* d = motionNamed(scene, j.drive);
        if (!d) continue;
        d->keys.clear();
        if (touched.count(j.drive))
            for (const auto& [t, values] : keyed) d->keys.push_back({t, values.at(j.drive)});
        else // joints the gesture does not move hold still
            d->keys = {{0.0f, keyed.front().second.at(j.drive)}};
        d->mode = loop && touched.count(j.drive) ? MotionMode::Loop : MotionMode::Once;
    }
    r.duration = keyed.empty() ? 0.0f : keyed.back().first;
    levelAlongPath(scene, robot, levelled, r.duration, loop);
    plantFeet(scene, robot.component, r.duration, loop);
    return r;
}

} // namespace sicdm

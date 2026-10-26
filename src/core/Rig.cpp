#include "core/Rig.hpp"

#include "core/Json.hpp"
#include "core/Motion.hpp"
#include "core/Scene.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <functional>
#include <map>
#include <set>
#include <unordered_map>

namespace sicdm {

namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;

std::string lower(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

bool contains(const std::string& haystack, const char* needle) { return lower(haystack).find(needle) != std::string::npos; }

std::array<double, 3> metres(const Vec3& v) { return {v.x * 1e-3, v.y * 1e-3, v.z * 1e-3}; }

const struct {
    Morphology m;
    const char* key;
} kMorphologies[] = {{Morphology::Auto, "auto"},         {Morphology::Manipulator, "manipulator"}, {Morphology::Humanoid, "humanoid"},
                     {Morphology::Quadruped, "quadruped"}, {Morphology::Drone, "drone"},           {Morphology::Rover, "rover"},
                     {Morphology::Custom, "custom"}};

const struct {
    RigTag t;
    const char* key;
} kTags[] = {{RigTag::None, "none"},  {RigTag::Rotor, "rotor"}, {RigTag::Wheel, "wheel"}, {RigTag::EndEffector, "end_effector"},
             {RigTag::Foot, "foot"},  {RigTag::Knee, "knee"},   {RigTag::Hinge, "hinge"}, {RigTag::Base, "base"}};

} // namespace

const char* morphologyKey(Morphology m) noexcept {
    for (const auto& e : kMorphologies)
        if (e.m == m) return e.key;
    return "custom";
}

std::optional<Morphology> morphologyFromKey(const std::string& key) noexcept {
    const std::string k = lower(key);
    for (const auto& e : kMorphologies)
        if (k == e.key) return e.m;
    if (k == "arm" || k == "manipulators") return Morphology::Manipulator;
    if (k == "biped" || k == "humanoids") return Morphology::Humanoid;
    if (k == "quadrupeds" || k == "dog") return Morphology::Quadruped;
    if (k == "drones" || k == "multirotor" || k == "quadcopter") return Morphology::Drone;
    if (k == "rovers" || k == "vehicle" || k == "car") return Morphology::Rover;
    return std::nullopt;
}

const char* rigTagKey(RigTag t) noexcept {
    for (const auto& e : kTags)
        if (e.t == t) return e.key;
    return "none";
}

std::optional<RigTag> rigTagFromKey(const std::string& key) noexcept {
    std::string k = lower(key);
    for (char& c : k)
        if (c == '-' || c == ' ') c = '_';
    for (const auto& e : kTags)
        if (k == e.key) return e.t;
    if (k == "endeffector" || k == "effector" || k == "tool" || k == "gripper") return RigTag::EndEffector;
    if (k == "propeller" || k == "prop") return RigTag::Rotor;
    if (k == "feet") return RigTag::Foot;
    return std::nullopt;
}

const char* rigJointTypeKey(RigJointType t) noexcept {
    switch (t) {
    case RigJointType::Revolute: return "revolute";
    case RigJointType::Continuous: return "continuous";
    case RigJointType::Prismatic: return "prismatic";
    }
    return "revolute";
}

RigTag RigSettings::tagOf(const std::string& body) const {
    for (const auto& [b, t] : tags)
        if (b == body) return t;
    return RigTag::None;
}

void RigSettings::setTag(const std::string& body, RigTag tag) {
    tags.erase(std::remove_if(tags.begin(), tags.end(), [&](const auto& p) { return p.first == body; }), tags.end());
    if (tag != RigTag::None) tags.emplace_back(body, tag);
}

RigSettings* RigState::find(const std::string& name) {
    for (RigSettings& r : robots)
        if (r.name == name) return &r;
    return nullptr;
}

const RigSettings* RigState::find(const std::string& name) const {
    for (const RigSettings& r : robots)
        if (r.name == name) return &r;
    return nullptr;
}

RigSettings& RigState::get(const std::string& name) {
    if (RigSettings* r = find(name)) return *r;
    robots.push_back({});
    robots.back().name = name;
    return robots.back();
}

Json rigStateToJson(const RigState& s) {
    Json arr = Json::array();
    for (const RigSettings& r : s.robots) {
        Json j = Json::object();
        j.set("name", r.name);
        j.set("morphology", morphologyKey(r.morphology));
        if (r.floatingBase) j.set("floating_base", *r.floatingBase);
        if (r.density > 0) j.set("density", r.density);
        if (!r.engine.empty()) j.set("engine", r.engine);
        Json tags = Json::object();
        for (const auto& [b, t] : r.tags) tags.set(b, rigTagKey(t));
        j.set("tags", tags);
        arr.push(j);
    }
    Json out = Json::object();
    out.set("robots", arr);
    return out;
}

void rigStateFromJson(const Json& j, RigState& s) {
    s = {};
    const Json& arr = j["robots"];
    for (std::size_t i = 0; i < arr.size(); ++i) {
        const Json& r = arr[i];
        RigSettings rs;
        rs.name = r["name"].asString(std::string());
        rs.morphology = morphologyFromKey(r["morphology"].asString(std::string("auto"))).value_or(Morphology::Auto);
        if (r["floating_base"].isBool()) rs.floatingBase = r["floating_base"].asBool();
        rs.density = r["density"].asFloat(0.0f);
        rs.engine = r["engine"].asString(std::string());
        if (const Json& tags = r["tags"]; tags.isObject())
            for (const auto& [b, v] : tags.members())
                if (const auto t = rigTagFromKey(v.asString(std::string())); t && *t != RigTag::None) rs.tags.emplace_back(b, *t);
        s.robots.push_back(std::move(rs));
    }
}

MassInertia meshInertia(const Body& body, double density) {
    MassInertia out;
    const auto& v = body.mesh.vertices;
    const auto& idx = body.mesh.indices;
    // Signed tetrahedra from the origin: volume, first moment and the covariance
    // integral of x x^T, each in mm^n, summed over the faces (divergence theorem).
    double vol = 0, c[3] = {0, 0, 0}, cov[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    for (std::size_t t = 0; t + 2 < idx.size(); t += 3) {
        const Vec3 pa = v[idx[t]].position + body.offset, pb = v[idx[t + 1]].position + body.offset, pc = v[idx[t + 2]].position + body.offset;
        const double a[3] = {pa.x, pa.y, pa.z}, b[3] = {pb.x, pb.y, pb.z}, d[3] = {pc.x, pc.y, pc.z};
        const double sv = (a[0] * (b[1] * d[2] - b[2] * d[1]) - a[1] * (b[0] * d[2] - b[2] * d[0]) + a[2] * (b[0] * d[1] - b[1] * d[0])) / 6.0;
        vol += sv;
        double s[3];
        for (int k = 0; k < 3; ++k) {
            s[k] = a[k] + b[k] + d[k];
            c[k] += sv * s[k] / 4.0;
        }
        // Over the tetrahedron (0, a, b, d): int x_i x_j = V/20 (sum of the vertices' x_i x_j + s_i s_j).
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) cov[i * 3 + j] += sv / 20.0 * (a[i] * a[j] + b[i] * b[j] + d[i] * d[j] + s[i] * s[j]);
    }
    const double mm3 = 1e-9, mm5 = 1e-15;
    if (std::fabs(vol) > 1e-6) {
        const double sign = vol < 0 ? -1.0 : 1.0; // inside-out meshes
        out.volume = std::fabs(vol) * mm3;
        const double mass = body.physics.mass > 0 ? body.physics.mass : out.volume * density;
        const double rho = mass / out.volume; // kg/m^3, the mass spread through the volume
        out.mass = mass;
        for (int k = 0; k < 3; ++k) out.com[k] = c[k] / vol * 1e-3;
        // Covariance about the centre of mass (kg m^2), then I = tr(C) 1 - C.
        double C[9];
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) C[i * 3 + j] = rho * sign * cov[i * 3 + j] * mm5 - mass * out.com[i] * out.com[j];
        const double tr = C[0] + C[4] + C[8];
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) out.inertia[i * 3 + j] = (i == j ? tr : 0.0) - C[i * 3 + j];
        return out;
    }
    // Open or flat: a solid box of its bounds.
    const AABB bb = body.bounds();
    if (!bb.valid()) return out;
    const double sx = (bb.max.x - bb.min.x) * 1e-3, sy = (bb.max.y - bb.min.y) * 1e-3, sz = (bb.max.z - bb.min.z) * 1e-3;
    out.volume = std::max(sx * sy * sz, 1e-9);
    out.mass = body.physics.mass > 0 ? body.physics.mass : out.volume * density;
    out.com = metres(bb.center());
    out.inertia = {out.mass * (sy * sy + sz * sz) / 12.0, 0, 0, 0, out.mass * (sx * sx + sz * sz) / 12.0, 0, 0, 0, out.mass * (sx * sx + sy * sy) / 12.0};
    return out;
}

namespace {

// The bodies of a robot: its component, its assembly path, or (no name) every body.
std::vector<char> robotBodies(const Scene& scene, const std::string& name) {
    const auto& bodies = scene.bodies();
    std::vector<char> in(bodies.size(), name.empty() ? 1 : 0);
    if (!name.empty())
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            const Body& b = bodies[i];
            in[i] = b.component == name || b.assembly == name || b.assembly.rfind(name + "/", 0) == 0 || b.name == name;
        }
    // Bodies attached to one of them ride along.
    for (bool grew = true; grew;) {
        grew = false;
        std::unordered_map<std::string_view, std::size_t> byName;
        for (std::size_t i = 0; i < bodies.size(); ++i)
            if (in[i]) byName[bodies[i].name] = i;
        for (std::size_t i = 0; i < bodies.size(); ++i)
            if (!in[i] && !bodies[i].attachedTo.empty() && byName.count(bodies[i].attachedTo)) in[i] = 1, grew = true;
    }
    return in;
}

// Motions other motions follow: drive motors, not joints.
std::set<std::string> driveNames(const Scene& scene) {
    std::set<std::string> drives;
    for (const Motion& m : scene.motions())
        if (!m.follow.empty()) drives.insert(m.follow);
    return drives;
}

Mat3d addInertia(const Mat3d& a, const Mat3d& b) {
    Mat3d r;
    for (int i = 0; i < 9; ++i) r[i] = a[i] + b[i];
    return r;
}

// Inertia of a point mass m at offset d from the reference point (parallel axis).
Mat3d pointInertia(double m, const std::array<double, 3>& d) {
    const double dd = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
    Mat3d r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) r[i * 3 + j] = m * ((i == j ? dd : 0.0) - d[i] * d[j]);
    return r;
}

} // namespace

std::vector<std::string> rigCandidates(const Scene& scene) {
    const auto drives = driveNames(scene);
    const auto perMotion = motionBodies(scene);
    std::vector<std::string> out;
    bool loose = false;
    for (std::size_t k = 0; k < scene.motions().size(); ++k) {
        const Motion& m = scene.motions()[k];
        if (!m.enabled || m.alongLoop || drives.count(m.name)) continue;
        for (std::size_t b : perMotion[k]) {
            const Body& body = scene.bodies()[b];
            std::string name = !body.component.empty() ? body.component : body.assembly.substr(0, body.assembly.find('/'));
            if (name.empty()) {
                loose = true;
                continue;
            }
            if (std::find(out.begin(), out.end(), name) == out.end()) out.push_back(name);
        }
    }
    if (loose) out.insert(out.begin(), std::string());
    return out;
}

int RobotRig::linkOf(std::size_t sceneBody) const {
    for (std::size_t l = 0; l < links.size(); ++l)
        if (std::find(links[l].bodies.begin(), links[l].bodies.end(), sceneBody) != links[l].bodies.end()) return static_cast<int>(l);
    return -1;
}

int RobotRig::jointNamed(const std::string& n) const {
    const std::string k = lower(n);
    for (std::size_t j = 0; j < joints.size(); ++j)
        if (lower(joints[j].name) == k) return static_cast<int>(j);
    // A unique suffix ("Elbow" for "Robot Arm 1 Elbow").
    int found = -1;
    for (std::size_t j = 0; j < joints.size(); ++j) {
        const std::string q = lower(joints[j].name);
        if (q.size() > k.size() && q.compare(q.size() - k.size(), k.size(), k) == 0 && q[q.size() - k.size() - 1] == ' ') {
            if (found >= 0) return -1;
            found = static_cast<int>(j);
        }
    }
    return found;
}

int RobotRig::linkNamed(const std::string& n) const {
    const std::string k = lower(n);
    for (std::size_t l = 0; l < links.size(); ++l)
        if (lower(links[l].name) == k) return static_cast<int>(l);
    return -1;
}

double RobotRig::totalMass() const {
    double m = 0;
    for (const RigLink& l : links) m += l.mass;
    return m;
}

std::vector<int> RobotRig::endEffectors() const {
    std::vector<int> out;
    for (std::size_t l = 0; l < links.size(); ++l)
        if (links[l].tag == RigTag::EndEffector) out.push_back(static_cast<int>(l));
    if (!out.empty() || joints.empty()) return out;
    // The deepest leaves.
    std::vector<int> depth(links.size(), 0);
    std::vector<char> parentOfSome(links.size(), 0);
    for (const RigJoint& j : joints) {
        depth[static_cast<std::size_t>(j.child)] = depth[static_cast<std::size_t>(j.parent)] + 1;
        parentOfSome[static_cast<std::size_t>(j.parent)] = 1;
    }
    int best = 0;
    for (std::size_t l = 0; l < links.size(); ++l)
        if (!parentOfSome[l]) best = std::max(best, depth[l]);
    for (std::size_t l = 0; l < links.size(); ++l)
        if (!parentOfSome[l] && depth[l] == best) out.push_back(static_cast<int>(l));
    return out;
}

bool buildRig(const Scene& scene, const RigSettings& settings, RobotRig& out, std::string* error) {
    out = {};
    out.name = settings.name.empty() ? std::string("Robot") : settings.name;
    const auto& bodies = scene.bodies();
    const auto& motions = scene.motions();
    const std::vector<char> in = robotBodies(scene, settings.name);
    std::size_t count = 0;
    for (char c : in) count += c ? 1 : 0;
    if (count == 0) {
        if (error) *error = settings.name.empty() ? "The scene has no parts" : "No component or assembly named " + settings.name;
        return false;
    }
    std::unordered_map<std::string_view, std::size_t> byName;
    for (std::size_t i = 0; i < bodies.size(); ++i) byName[bodies[i].name] = i;
    // Attachment closure: a body and everything attached to it (transitively).
    std::vector<std::vector<std::size_t>> attached(bodies.size());
    for (std::size_t i = 0; i < bodies.size(); ++i)
        if (in[i] && !bodies[i].attachedTo.empty())
            if (const auto it = byName.find(bodies[i].attachedTo); it != byName.end()) attached[it->second].push_back(i);
    auto withAttached = [&](std::vector<std::size_t> set) {
        for (std::size_t k = 0; k < set.size(); ++k)
            for (std::size_t a : attached[set[k]]) set.push_back(a);
        std::sort(set.begin(), set.end());
        set.erase(std::unique(set.begin(), set.end()), set.end());
        return set;
    };

    // Joint motions and the bodies each moves.
    const auto drives = driveNames(scene);
    const auto perMotion = motionBodies(scene);
    struct Cand {
        std::size_t motion;
        std::vector<std::size_t> set;
    };
    std::vector<Cand> cands;
    for (std::size_t k = 0; k < motions.size(); ++k) {
        const Motion& m = motions[k];
        if (!m.enabled || m.alongLoop || drives.count(m.name)) continue;
        if (std::find(m.bodies.begin(), m.bodies.end(), "*") != m.bodies.end()) continue;
        std::vector<std::size_t> set;
        for (std::size_t b : perMotion[k])
            if (in[b]) set.push_back(b);
        if (set.empty()) continue;
        set = withAttached(std::move(set));
        if (set.size() >= count) continue; // moves the whole robot: travel, not a joint
        cands.push_back({k, std::move(set)});
    }
    // Outermost first: larger sets first, list order among equal ones.
    std::stable_sort(cands.begin(), cands.end(), [](const Cand& a, const Cand& b) { return a.set.size() > b.set.size(); });
    const std::size_t nj = cands.size();
    auto superset = [](const std::vector<std::size_t>& big, const std::vector<std::size_t>& small) {
        return std::includes(big.begin(), big.end(), small.begin(), small.end());
    };
    std::vector<int> parentJoint(nj, -1);
    for (std::size_t j = 0; j < nj; ++j) {
        int best = -1;
        for (std::size_t k = 0; k < j; ++k)
            if (superset(cands[k].set, cands[j].set) && (best < 0 || cands[k].set.size() <= cands[static_cast<std::size_t>(best)].set.size()))
                best = static_cast<int>(k);
        parentJoint[j] = best;
    }
    // Each body's innermost joint.
    std::vector<int> inner(bodies.size(), -1);
    for (std::size_t j = 0; j < nj; ++j)
        for (std::size_t b : cands[j].set) {
            const int cur = inner[b];
            if (cur < 0 || cands[j].set.size() <= cands[static_cast<std::size_t>(cur)].set.size()) inner[b] = static_cast<int>(j);
        }

    // Links: the base, then one per joint.
    out.links.resize(nj + 1);
    out.links[0].name = out.name + " base";
    for (std::size_t j = 0; j < nj; ++j) {
        out.links[j + 1].name = motions[cands[j].motion].name + " link";
        out.links[j + 1].parentJoint = static_cast<int>(j);
    }
    for (std::size_t b = 0; b < bodies.size(); ++b)
        if (in[b]) out.links[static_cast<std::size_t>(inner[b] + 1)].bodies.push_back(b);

    // Mass properties and tags.
    int massless = 0;
    for (RigLink& link : out.links) {
        double m = 0;
        std::array<double, 3> c{};
        std::vector<MassInertia> parts;
        bool boxSet = false;
        for (std::size_t b : link.bodies) {
            const Body& body = bodies[b];
            if (const RigTag t = settings.tagOf(body.name); t != RigTag::None && link.tag == RigTag::None) link.tag = t;
            if (body.mesh.indices.empty()) continue;
            const double density = settings.density > 0 ? settings.density : (body.physics.density > 0 ? body.physics.density : 1000.0);
            const MassInertia mi = meshInertia(body, density);
            if (mi.mass <= 0) continue;
            parts.push_back(mi);
            m += mi.mass;
            for (int k = 0; k < 3; ++k) c[k] += mi.mass * mi.com[k];
            const AABB bb = body.bounds();
            const auto lo = metres(bb.min), hi = metres(bb.max);
            for (int k = 0; k < 3; ++k) {
                link.boxMin[k] = boxSet ? std::min(link.boxMin[k], lo[k]) : lo[k];
                link.boxMax[k] = boxSet ? std::max(link.boxMax[k], hi[k]) : hi[k];
            }
            boxSet = true;
        }
        if (m <= 0) {
            // A link of the joint chain with no parts (between the joints of a ball): next
            // to nothing, so the solvers keep it well conditioned.
            ++massless;
            link.mass = 1e-3;
            link.inertia = {1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6};
            continue;
        }
        for (double& v : c) v /= m;
        link.mass = m;
        link.com = c;
        Mat3d I{};
        for (const MassInertia& p : parts) {
            const std::array<double, 3> d{p.com[0] - c[0], p.com[1] - c[1], p.com[2] - c[2]};
            I = addInertia(I, addInertia(p.inertia, pointInertia(p.mass, d)));
        }
        link.inertia = I;
    }
    // Massless links sit on their joint's axis.
    for (std::size_t j = 0; j < nj; ++j) {
        RigLink& l = out.links[j + 1];
        if (l.bodies.empty() || l.mass <= 1e-3) {
            const Motion& m = motions[cands[j].motion];
            if (l.mass <= 1e-3) l.com = metres(m.origin);
        }
    }

    // Joints.
    std::map<std::string, int> firstOfDrive; // drive -> the first joint following it
    for (std::size_t j = 0; j < nj; ++j) {
        const Motion& m = motions[cands[j].motion];
        RigJoint jt;
        jt.name = m.name;
        jt.motion = static_cast<int>(cands[j].motion);
        jt.parent = parentJoint[j] + 1;
        jt.child = static_cast<int>(j) + 1;
        jt.origin = metres(m.origin);
        Vec3 ax = length(m.axis) > 0 ? normalize(m.axis) : Vec3{0, 0, 1};
        jt.axis = {ax.x, ax.y, ax.z};
        if (m.kind == MotionKind::Circular) {
            if (m.hasLimits) {
                jt.type = RigJointType::Revolute;
                jt.lo = m.lo * kDeg;
                jt.hi = m.hi * kDeg;
            } else if (m.range > 0 && m.mode != MotionMode::Continuous && m.keys.empty()) {
                jt.type = RigJointType::Revolute;
                jt.lo = std::min(0.0f, m.range) * kDeg;
                jt.hi = std::max(0.0f, m.range) * kDeg;
            } else {
                jt.type = RigJointType::Continuous;
            }
            jt.velocity = m.follow.empty() && m.keys.empty() ? std::fabs(m.speed) * kDeg : 0.0;
        } else {
            jt.type = RigJointType::Prismatic;
            jt.lo = (m.hasLimits ? m.lo : std::min(0.0f, m.range)) * 1e-3;
            jt.hi = (m.hasLimits ? m.hi : std::max(0.0f, m.range)) * 1e-3;
            if (jt.hi <= jt.lo) jt.lo = -1.0, jt.hi = 1.0;
            jt.velocity = m.follow.empty() && m.keys.empty() ? std::fabs(m.speed) * 1e-3 : 0.0;
        }
        // Coupling: joints following one drive move together.
        if (!m.follow.empty()) {
            if (drives.count(m.follow)) {
                bool followsJoint = false;
                for (const Cand& c : cands) followsJoint = followsJoint || motions[c.motion].name == m.follow;
                if (followsJoint) {
                    jt.mimic = m.follow;
                    jt.mimicRatio = m.ratio;
                } else {
                    jt.drive = m.follow;
                    const auto it = firstOfDrive.find(m.follow);
                    if (it == firstOfDrive.end()) {
                        firstOfDrive[m.follow] = static_cast<int>(out.joints.size());
                    } else {
                        const Motion& lead = motions[static_cast<std::size_t>(out.joints[static_cast<std::size_t>(it->second)].motion)];
                        jt.mimic = lead.name;
                        jt.mimicRatio = lead.ratio != 0 ? m.ratio / lead.ratio : 1.0;
                    }
                }
            }
        }
        out.joints.push_back(std::move(jt));
    }
    // A tag on the child link sets its joint type.
    for (RigJoint& jt : out.joints) {
        const RigTag t = out.links[static_cast<std::size_t>(jt.child)].tag;
        if (t == RigTag::Rotor || t == RigTag::Wheel) jt.type = RigJointType::Continuous;
    }

    // Morphology and base.
    int rotors = 0, wheels = 0, knees = 0, feet = 0, bases = 0;
    bool palm = false, pelvis = false;
    for (const RigLink& l : out.links) {
        rotors += l.tag == RigTag::Rotor;
        wheels += l.tag == RigTag::Wheel;
        feet += l.tag == RigTag::Foot;
        bases += l.tag == RigTag::Base;
    }
    for (std::size_t b = 0; b < bodies.size(); ++b)
        if (in[b]) {
            palm = palm || bodies[b].kind == "robot_palm";
            pelvis = pelvis || bodies[b].kind == "robot_pelvis";
            if (bodies[b].kind == "tyre" && settings.tagOf(bodies[b].name) == RigTag::None) ++wheels;
        }
    for (const RigJoint& jt : out.joints) knees += contains(jt.name, "knee") ? 1 : 0;
    Morphology mo = settings.morphology;
    if (mo == Morphology::Auto) {
        if (rotors >= 2) mo = Morphology::Drone;
        else if (wheels >= 3) mo = Morphology::Rover;
        else if (knees >= 4 || feet >= 4) mo = Morphology::Quadruped;
        else if (pelvis || knees >= 2 || feet == 2) mo = Morphology::Humanoid;
        else if (palm || !out.joints.empty()) mo = Morphology::Manipulator;
        else mo = Morphology::Custom;
    }
    out.morphology = mo;
    out.floatingBase = settings.floatingBase.value_or(bases == 0 && mo != Morphology::Manipulator);

    if (massless > 0) out.notes.push_back(std::to_string(massless) + " link(s) have no parts (joint chains such as a ball joint): given 1 g");
    if (out.joints.empty()) out.notes.push_back("No joints: the robot is one rigid body (give its parts motions, JOINT, or tag ROTOR / WHEEL)");
    return true;
}

std::vector<std::string> applyRigTags(Scene& scene, const RigSettings& settings) {
    std::vector<std::string> done;
    const auto drives = driveNames(scene);
    for (const auto& [name, tag] : settings.tags) {
        int bi = -1;
        for (std::size_t i = 0; i < scene.bodies().size(); ++i)
            if (scene.bodies()[i].name == name) bi = static_cast<int>(i);
        if (bi < 0) continue;
        const Body& body = scene.bodies()[static_cast<std::size_t>(bi)];
        // The motions that name this body, innermost last.
        int own = -1;
        for (std::size_t k = 0; k < scene.motions().size(); ++k) {
            const Motion& m = scene.motions()[k];
            if (drives.count(m.name) || m.alongLoop) continue;
            if (std::find(m.bodies.begin(), m.bodies.end(), name) != m.bodies.end()) own = static_cast<int>(k);
        }
        if (tag == RigTag::Rotor || tag == RigTag::Wheel) {
            // Its own spin: a motion that moves this body and nothing outside what rides on it.
            bool spins = false;
            if (own >= 0) {
                const Motion& m = scene.motions()[static_cast<std::size_t>(own)];
                spins = m.kind == MotionKind::Circular && static_cast<std::ptrdiff_t>(m.bodies.size()) <= 1 + std::count_if(scene.bodies().begin(), scene.bodies().end(),
                                                                                               [&](const Body& b) { return b.attachedTo == name; });
            }
            if (spins) continue;
            const AABB bb = body.bounds();
            const Vec3 size = bb.max - bb.min;
            Vec3 axis{0, 0, 1};
            if (size.x <= size.y && size.x <= size.z) axis = {1, 0, 0};
            else if (size.y <= size.x && size.y <= size.z) axis = {0, 1, 0};
            Motion m;
            m.name = name + " Spin";
            m.kind = MotionKind::Circular;
            m.bodies.push_back(name);
            for (const Body& b : scene.bodies())
                if (b.attachedTo == name) m.bodies.push_back(b.name);
            m.axis = axis;
            m.origin = bb.center();
            m.speed = tag == RigTag::Rotor ? 1800.0f : 180.0f;
            m.mode = MotionMode::Continuous;
            scene.motions().push_back(std::move(m));
            done.push_back(std::string(tag == RigTag::Rotor ? "Rotor " : "Wheel ") + name + ": spins freely about " +
                           (axis.x > 0 ? "X" : axis.y > 0 ? "Y" : "Z") + " through its centre");
        } else if ((tag == RigTag::Knee || tag == RigTag::Hinge) && own >= 0) {
            Motion& m = scene.motions()[static_cast<std::size_t>(own)];
            if (m.hasLimits || m.kind != MotionKind::Circular) continue;
            m.hasLimits = true;
            m.lo = tag == RigTag::Knee ? 0.0f : -90.0f;
            m.hi = tag == RigTag::Knee ? 140.0f : 90.0f;
            char buf[160];
            std::snprintf(buf, sizeof buf, "%s %s: limited to %g..%g deg", tag == RigTag::Knee ? "Knee" : "Hinge", m.name.c_str(), m.lo, m.hi);
            done.emplace_back(buf);
        }
    }
    return done;
}

std::vector<std::string> describeRig(const RobotRig& rig) {
    std::vector<std::string> out;
    char buf[400];
    std::snprintf(buf, sizeof buf, "%s: %s, %s base, %zu links, %zu joints, %.3g kg", rig.name.c_str(), morphologyKey(rig.morphology),
                  rig.floatingBase ? "floating" : "fixed", rig.links.size(), rig.joints.size(), rig.totalMass());
    out.emplace_back(buf);
    for (std::size_t j = 0; j < rig.joints.size(); ++j) {
        const RigJoint& jt = rig.joints[j];
        const RigLink& child = rig.links[static_cast<std::size_t>(jt.child)];
        std::string range;
        if (jt.type == RigJointType::Revolute) {
            std::snprintf(buf, sizeof buf, " %g..%g deg", std::round(jt.lo / kDeg * 10) / 10, std::round(jt.hi / kDeg * 10) / 10);
            range = buf;
        } else if (jt.type == RigJointType::Prismatic) {
            std::snprintf(buf, sizeof buf, " %g..%g mm", std::round(jt.lo * 1e4) / 10, std::round(jt.hi * 1e4) / 10);
            range = buf;
        }
        std::snprintf(buf, sizeof buf, "  %-34s %-10s%s  on %s, %zu parts, %.3g kg%s%s", jt.name.c_str(), rigJointTypeKey(jt.type), range.c_str(),
                      jt.parent == 0 ? "base" : rig.joints[static_cast<std::size_t>(jt.parent - 1)].name.c_str(), child.bodies.size(), child.mass,
                      child.tag != RigTag::None ? (std::string(", ") + rigTagKey(child.tag)).c_str() : "",
                      !jt.mimic.empty() ? (", follows " + jt.mimic).c_str() : "");
        out.emplace_back(buf);
    }
    for (const std::string& n : rig.notes) out.push_back("  note: " + n);
    return out;
}

} // namespace sicdm

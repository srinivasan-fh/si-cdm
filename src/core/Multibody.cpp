// The articulated robot engine: Featherstone's articulated-body algorithm with all spatial
// quantities in the world frame (Plucker coordinates at the world origin), a floating base
// as a free root, soft ground contacts with stick-slip friction, and implicitly integrated
// servos and joint limits.

#include "core/Multibody.hpp"

#include "core/Motion.hpp"
#include "core/Scene.hpp"
#include "core/Json.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <map>
#include <mutex>

namespace sicdm {

namespace {

constexpr double kPiD = 3.14159265358979323846;

// --- small fixed-size algebra ------------------------------------------------------------

using M3 = std::array<double, 9>; // row major

V3d add(const V3d& a, const V3d& b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
V3d sub(const V3d& a, const V3d& b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
V3d mul(const V3d& a, double s) { return {a[0] * s, a[1] * s, a[2] * s}; }
double dot(const V3d& a, const V3d& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
V3d cross(const V3d& a, const V3d& b) { return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]}; }
double norm(const V3d& a) { return std::sqrt(dot(a, a)); }

V3d mv(const M3& m, const V3d& v) {
    return {m[0] * v[0] + m[1] * v[1] + m[2] * v[2], m[3] * v[0] + m[4] * v[1] + m[5] * v[2], m[6] * v[0] + m[7] * v[1] + m[8] * v[2]};
}
M3 mm(const M3& a, const M3& b) {
    M3 r{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) r[i * 3 + j] = a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
    return r;
}
M3 transpose(const M3& a) { return {a[0], a[3], a[6], a[1], a[4], a[7], a[2], a[5], a[8]}; }
M3 identity3() { return {1, 0, 0, 0, 1, 0, 0, 0, 1}; }

// Rotation by `angle` about the unit `axis` (Rodrigues).
M3 rotation(const V3d& axis, double angle) {
    const double c = std::cos(angle), s = std::sin(angle), k = 1 - c;
    const double x = axis[0], y = axis[1], z = axis[2];
    return {c + x * x * k, x * y * k - z * s, x * z * k + y * s, y * x * k + z * s, c + y * y * k, y * z * k - x * s, z * x * k - y * s, z * y * k + x * s, c + z * z * k};
}

// Spatial motion / force vector: angular (or moment) part first.
struct SV {
    V3d w{0, 0, 0}, v{0, 0, 0};
};
SV operator+(const SV& a, const SV& b) { return {add(a.w, b.w), add(a.v, b.v)}; }
SV operator-(const SV& a, const SV& b) { return {sub(a.w, b.w), sub(a.v, b.v)}; }
SV operator*(const SV& a, double s) { return {mul(a.w, s), mul(a.v, s)}; }
double sdot(const SV& a, const SV& b) { return dot(a.w, b.w) + dot(a.v, b.v); }
// Motion cross motion: v x m u.
SV crm(const SV& v, const SV& u) { return {cross(v.w, u.w), add(cross(v.w, u.v), cross(v.v, u.w))}; }
// Motion cross force: v x* f.
SV crf(const SV& v, const SV& f) { return {add(cross(v.w, f.w), cross(v.v, f.v)), cross(v.w, f.v)}; }

// 6 x 6, row major, acting on (w, v).
struct M6 {
    double a[36] = {};
    SV operator*(const SV& x) const {
        const double in[6] = {x.w[0], x.w[1], x.w[2], x.v[0], x.v[1], x.v[2]};
        double o[6];
        for (int i = 0; i < 6; ++i) {
            double s = 0;
            for (int j = 0; j < 6; ++j) s += a[i * 6 + j] * in[j];
            o[i] = s;
        }
        return {{o[0], o[1], o[2]}, {o[3], o[4], o[5]}};
    }
    M6& operator+=(const M6& b) {
        for (int i = 0; i < 36; ++i) a[i] += b.a[i];
        return *this;
    }
};

// Spatial inertia at the world origin of mass m, centre c and inertia Ic about c.
M6 spatialInertia(double m, const V3d& c, const M3& Ic) {
    // [[Ic + m cx cx^T, m cx], [m cx^T, m 1]], cx = skew(c).
    const M3 cx = {0, -c[2], c[1], c[2], 0, -c[0], -c[1], c[0], 0};
    const M3 cxcxT = mm(cx, transpose(cx));
    M6 I;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            I.a[i * 6 + j] = Ic[i * 3 + j] + m * cxcxT[i * 3 + j];
            I.a[i * 6 + 3 + j] = m * cx[i * 3 + j];
            I.a[(3 + i) * 6 + j] = m * cx[j * 3 + i];
            I.a[(3 + i) * 6 + 3 + j] = i == j ? m : 0.0;
        }
    return I;
}

// Solves A x = b (6 x 6) by Gaussian elimination with partial pivoting.
SV solve6(const M6& A, const SV& b) {
    double m[6][7];
    const double rhs[6] = {b.w[0], b.w[1], b.w[2], b.v[0], b.v[1], b.v[2]};
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) m[i][j] = A.a[i * 6 + j];
        m[i][6] = rhs[i];
    }
    for (int c = 0; c < 6; ++c) {
        int p = c;
        for (int r = c + 1; r < 6; ++r)
            if (std::fabs(m[r][c]) > std::fabs(m[p][c])) p = r;
        if (p != c)
            for (int k = 0; k < 7; ++k) std::swap(m[p][k], m[c][k]);
        const double d = std::fabs(m[c][c]) > 1e-300 ? m[c][c] : 1e-300;
        for (int r = c + 1; r < 6; ++r) {
            const double f = m[r][c] / d;
            if (f == 0) continue;
            for (int k = c; k < 7; ++k) m[r][k] -= f * m[c][k];
        }
    }
    double x[6];
    for (int i = 5; i >= 0; --i) {
        double s = m[i][6];
        for (int k = i + 1; k < 6; ++k) s -= m[i][k] * x[k];
        x[i] = s / (std::fabs(m[i][i]) > 1e-300 ? m[i][i] : 1e-300);
    }
    return {{x[0], x[1], x[2]}, {x[3], x[4], x[5]}};
}

// Spatial force of a force F applied at point p plus a pure torque.
SV forceAt(const V3d& F, const V3d& p, const V3d& torque) { return {add(cross(p, F), torque), F}; }

struct Joint {
    RigJointType type = RigJointType::Revolute;
    int parent = 0;
    V3d axis0{0, 0, 1}, origin0{0, 0, 0};
    double lo = 0, hi = 0;
    bool limited = false;
    double effort = 1, kp = 0, kd = 0, kv = 0, armature = 1e-6, inertia = 0;
    int leader = -1; // mimic
    double ratio = 1;
    double limitK = 0, limitD = 0;
};

struct Link {
    double mass = 0;
    V3d com0{};
    M3 Ic0{};
    std::vector<V3d> points;     // contact points at rest (world, m)
    double k = 0, c = 0;         // contact stiffness and damping of the link
    double share = 1;            // each point's part of them
};

} // namespace

// --- engines -------------------------------------------------------------------------------

const std::vector<SimEngineInfo>& simEngines() {
    static const std::vector<SimEngineInfo> list = {
        {SimEngine::Articulated, "articulated", "Articulated (joint space)",
         "Generalized-coordinate dynamics of the joint tree: contacts, grasping and gaits. Manipulators, humanoids, quadrupeds, custom robots"},
        {SimEngine::Vehicle, "vehicle", "Vehicle",
         "The articulated engine with rotor thrust, wind, air drag and tyre grip; batches of copies in parallel. Drones and rovers"},
        {SimEngine::Optimize, "optimize", "Trajectory optimisation",
         "The least-effort joint motion to a goal on the articulated model, then simulated. Custom manipulators"},
        {SimEngine::Particle, "particle", "Rigid bodies", "The scene's physics: free rigid bodies (PHYSICS)"},
    };
    return list;
}

std::optional<SimEngine> simEngineFromKey(const std::string& key) {
    std::string k;
    for (char c : key) k += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    for (const SimEngineInfo& e : simEngines())
        if (k == e.key) return e.engine;
    if (k == "multibody" || k == "joint" || k == "mujoco") return SimEngine::Articulated;
    if (k == "drone" || k == "rover" || k == "physx" || k == "isaac") return SimEngine::Vehicle;
    if (k == "trajopt" || k == "optimise" || k == "drake") return SimEngine::Optimize;
    if (k == "rigid" || k == "physics") return SimEngine::Particle;
    return std::nullopt;
}

const char* simEngineKey(SimEngine e) {
    for (const SimEngineInfo& i : simEngines())
        if (i.engine == e) return i.key;
    return "articulated";
}

SimEngine defaultEngine(Morphology m) {
    return m == Morphology::Drone || m == Morphology::Rover ? SimEngine::Vehicle : SimEngine::Articulated;
}

SimEngine engineFor(const RobotRig& rig, const RigSettings& settings) {
    if (const auto e = simEngineFromKey(settings.engine)) return *e;
    return defaultEngine(rig.morphology);
}

Ground Ground::flat(double z, double friction) {
    Ground g;
    g.height = [z](double, double) { return z; };
    g.friction = friction;
    return g;
}

const char* actuatorModeKey(ActuatorMode m) {
    switch (m) {
    case ActuatorMode::Position: return "position";
    case ActuatorMode::Velocity: return "velocity";
    case ActuatorMode::Torque: return "torque";
    case ActuatorMode::Passive: return "passive";
    }
    return "position";
}

V3d LinkPose::apply(const V3d& p) const { return add(mv(R, p), t); }

// --- the engine --------------------------------------------------------------------------------

struct Multibody::Impl {
    RobotRig rig;
    MultibodyOptions opt;
    std::vector<Joint> joints;
    std::vector<Link> links;
    std::vector<JointControl> controls;
    // State.
    std::vector<double> q, qd, tau, integral; // integral: each position servo's integral of the error (rad s)
    M3 baseR = identity3();
    V3d baseT{0, 0, 0};
    SV baseV;
    double time = 0;
    bool diverged = false;
    // Per link, per step.
    std::vector<M3> R;
    std::vector<V3d> T;
    std::vector<SV> v, c, a, S, fext, pA, f;
    std::vector<M6> I, IA;
    std::vector<SV> U;
    std::vector<double> D, u, qdd;
    std::vector<SV> acc; // spatial acceleration of each link (last step)
    // Contacts: anchors per link per point.
    std::vector<std::vector<V3d>> anchor;
    std::vector<std::vector<char>> touching;
    std::vector<ContactPoint> contacts;
    // Wrenches.
    struct Wrench {
        int link;
        V3d force, torque, point;
    };
    std::vector<Wrench> once, persistent;
    std::vector<ForceHook> hooks;
    std::vector<std::pair<V3d, V3d>> jointWrench;

    void setup();
    void kinematics();
    void computeContacts();
    void dynamics();
};

void Multibody::Impl::setup() {
    const std::size_t nl = rig.links.size(), nj = rig.joints.size();
    links.assign(nl, {});
    joints.assign(nj, {});
    controls.assign(nj, {});
    q.assign(nj, 0), qd.assign(nj, 0), tau.assign(nj, 0), qdd.assign(nj, 0), integral.assign(nj, 0);
    R.assign(nl, identity3()), T.assign(nl, {0, 0, 0});
    v.assign(nl, {}), c.assign(nl, {}), a.assign(nl, {}), S.assign(nl, {}), fext.assign(nl, {}), pA.assign(nl, {}), f.assign(nl, {});
    I.assign(nl, {}), IA.assign(nl, {}), U.assign(nl, {}), acc.assign(nl, {});
    D.assign(nl, 0), u.assign(nl, 0);
    jointWrench.assign(nj, {});
    const double total = std::max(rig.totalMass(), 1e-3);
    const double g = norm(opt.gravity);
    for (std::size_t l = 0; l < nl; ++l) {
        const RigLink& L = rig.links[l];
        Link& k = links[l];
        k.mass = std::max(L.mass, 1e-4);
        k.com0 = L.com;
        k.Ic0 = L.inertia;
        // Keep the tensor physical (tiny parts, flat meshes).
        for (int d = 0; d < 3; ++d) k.Ic0[static_cast<std::size_t>(d * 4)] = std::max(k.Ic0[static_cast<std::size_t>(d * 4)], 1e-7);
    }
    for (std::size_t l = 0; l < nl; ++l) {
        double lowest = 1e300;
        for (const auto& p : rig.links[l].contactPoints) {
            links[l].points.push_back(p);
            lowest = std::min(lowest, p[2]);
        }
        int onFace = 0;
        for (const auto& p : rig.links[l].contactPoints) onFace += p[2] < lowest + 0.003 ? 1 : 0;
        links[l].share = 1.0 / std::max(onFace, 1);
    }
    for (std::size_t j = 0; j < nj; ++j) {
        const RigJoint& J = rig.joints[j];
        Joint& jt = joints[j];
        jt.type = J.type;
        jt.parent = J.parent;
        jt.axis0 = J.axis;
        jt.origin0 = J.origin;
        jt.limited = J.type != RigJointType::Continuous;
        jt.lo = J.lo, jt.hi = J.hi;
        jt.effort = rigJointEffort(rig, j);
        jt.ratio = J.mimicRatio;
        if (!J.mimic.empty())
            for (std::size_t k = 0; k < nj; ++k)
                if (rig.joints[k].name == J.mimic) jt.leader = static_cast<int>(k);
        // Inertia the joint turns (moves): the links below it about its axis.
        double inertia = 0;
        for (int l : rigSubtree(rig, J.child)) {
            const Link& L = links[static_cast<std::size_t>(l)];
            if (J.type == RigJointType::Prismatic) {
                inertia += L.mass;
                continue;
            }
            const V3d d = sub(L.com0, J.origin);
            const double along = dot(d, J.axis);
            const double r2 = std::max(0.0, dot(d, d) - along * along);
            inertia += dot(J.axis, mv(L.Ic0, J.axis)) + L.mass * r2;
        }
        jt.inertia = std::max(inertia, 1e-7);
        jt.armature = 0.01 * jt.inertia + 1e-7;
        const double ie = jt.inertia + jt.armature;
        // Position servo: full effort at 3 degrees (3 mm) of error, critically damped.
        jt.kp = opt.servoStiffness * jt.effort / (J.type == RigJointType::Prismatic ? 0.003 : 0.05);
        jt.kd = 2.0 * std::sqrt(jt.kp * ie);
        jt.kv = jt.effort / 0.5;
        if (jt.leader >= 0) jt.kp *= 4, jt.kd *= 2;
        jt.limitK = 20.0 * jt.kp + 1.0;
        jt.limitD = 2.0 * std::sqrt(jt.limitK * ie);
    }
    // Contact stiffness of a link: a quarter of the robot's weight on it sinks 0.5 mm (stiff
    // enough that a tall robot can't tip over its own feet), within what keeps the link
    // itself stable at this step (omega dt <= 1, c dt / m <= 1).
    const double kBase = opt.stiffness > 0 ? opt.stiffness : std::max(500.0, total * std::max(g, 1.0) / (4 * 0.0005));
    for (Link& L : links) {
        L.k = std::min(kBase, L.mass / (opt.dt * opt.dt));
        // Damped as if it carried a share of the robot (a foot carries it all).
        const double carried = std::clamp(total / 8.0, L.mass, 2.0 * L.mass);
        L.c = std::min(2.0 * std::sqrt(L.k * carried), L.mass / opt.dt);
    }
    anchor.assign(nl, {});
    touching.assign(nl, {});
}

void Multibody::Impl::kinematics() {
    const std::size_t nl = links.size();
    R[0] = baseR, T[0] = baseT;
    v[0] = rig.floatingBase ? baseV : SV{};
    c[0] = {};
    for (std::size_t j = 0; j < joints.size(); ++j) {
        const Joint& jt = joints[j];
        const std::size_t ch = j + 1, p = static_cast<std::size_t>(jt.parent);
        const V3d axis = mv(R[p], jt.axis0);
        const V3d origin = add(mv(R[p], jt.origin0), T[p]);
        if (jt.type == RigJointType::Prismatic) {
            R[ch] = R[p];
            T[ch] = add(T[p], mul(axis, q[j]));
            S[ch] = {{0, 0, 0}, axis};
        } else {
            const M3 rot = rotation(axis, q[j]);
            R[ch] = mm(rot, R[p]);
            T[ch] = add(mv(rot, sub(T[p], origin)), origin);
            S[ch] = {axis, cross(origin, axis)};
        }
        const SV vj = S[ch] * qd[j];
        v[ch] = v[p] + vj;
        c[ch] = crm(v[ch], vj);
    }
    for (std::size_t l = 0; l < nl; ++l) {
        const Link& L = links[l];
        const V3d com = add(mv(R[l], L.com0), T[l]);
        const M3 Ic = mm(mm(R[l], L.Ic0), transpose(R[l]));
        I[l] = spatialInertia(L.mass, com, Ic);
    }
}

void Multibody::Impl::computeContacts() {
    contacts.clear();
    if (!opt.contacts || !opt.ground.height) return;
    const double mu = opt.ground.friction;
    struct Touch {
        std::size_t k;
        V3d p, n;
        double depth;
    };
    std::vector<Touch> touch;
    for (std::size_t l = 0; l < links.size(); ++l) {
        Link& L = links[l];
        if (anchor[l].size() != L.points.size()) anchor[l].assign(L.points.size(), {}), touching[l].assign(L.points.size(), 0);
        touch.clear();
        for (std::size_t k = 0; k < L.points.size(); ++k) {
            const V3d p = add(mv(R[l], L.points[k]), T[l]);
            const double h = opt.ground.height(p[0], p[1]);
            if (p[2] >= h) {
                touching[l][k] = 0;
                continue;
            }
            V3d n{0, 0, 1};
            if (opt.ground.normal) n = opt.ground.normal(p[0], p[1]);
            touch.push_back({k, p, n, (h - p[2]) * n[2]});
        }
        if (touch.empty()) continue;
        // Each point has a fixed share of the link's stiffness and damping (a spring per
        // point, so the forces stay conservative), sized so the link resting on its lowest
        // face has the link's own.
        const double kn = L.k * L.share, cn = L.c * L.share, kt = 0.5 * kn;
        for (const Touch& t : touch) {
            const std::size_t k = t.k;
            const V3d& p = t.p;
            const V3d& n = t.n;
            const V3d vp = add(v[l].v, cross(v[l].w, p));
            const double vn = dot(vp, n);
            const double Fn = std::max(0.0, kn * t.depth - cn * vn);
            if (!touching[l][k]) anchor[l][k] = p, touching[l][k] = 1;
            // Stick: a spring to where the point first touched; slip past mu Fn.
            V3d disp = sub(p, anchor[l][k]);
            disp = sub(disp, mul(n, dot(disp, n)));
            const V3d vt = sub(vp, mul(n, vn));
            V3d Ft = sub(mul(disp, -kt), mul(vt, cn));
            const double ft = norm(Ft), limit = mu * Fn;
            bool sliding = false;
            if (ft > limit) {
                Ft = ft > 0 ? mul(Ft, limit / ft) : V3d{0, 0, 0};
                // Move the anchor so the spring alone gives the sliding friction.
                anchor[l][k] = add(p, mul(Ft, 1.0 / std::max(kt, 1e-12)));
                sliding = true;
            }
            const V3d F = add(mul(n, Fn), Ft);
            fext[l] = fext[l] + forceAt(F, p, {0, 0, 0});
            contacts.push_back({static_cast<int>(l), p, F, sliding});
        }
    }
}

void Multibody::Impl::dynamics() {
    const std::size_t nl = links.size(), nj = joints.size();
    const double dt = opt.dt;
    const SV ag{{0, 0, 0}, opt.gravity};
    // Bias forces: velocity products, gravity and the external forces.
    for (std::size_t l = 0; l < nl; ++l) {
        IA[l] = I[l];
        pA[l] = crf(v[l], I[l] * v[l]) - I[l] * ag - fext[l];
    }
    // Joint torques: servos and limits, with their stiffness and damping taken implicitly.
    std::vector<double> extra(nj, 0.0);
    for (std::size_t j = 0; j < nj; ++j) {
        const Joint& jt = joints[j];
        const JointControl& ctl = controls[j];
        double t = 0, x = 0;
        ActuatorMode mode = ctl.mode;
        double target = ctl.target;
        if (jt.leader >= 0) mode = ActuatorMode::Position, target = jt.ratio * q[static_cast<std::size_t>(jt.leader)];
        if (mode == ActuatorMode::Position) {
            // PID: the integral (time constant 0.25 s) takes out the steady error a load
            // leaves, held within the effort (no wind-up). Off for a floating base by default:
            // legs on the ground close a loop the integral would fight.
            const double ki = (opt.integral >= 0 ? opt.integral : rig.floatingBase ? 0.0 : 4.0) * jt.kp;
            integral[j] = std::clamp(integral[j] + (target - q[j]) * dt, -jt.effort / ki, jt.effort / ki);
            const double implicitTau = jt.kp * (target - q[j] - dt * qd[j]) - jt.kd * qd[j] + ki * integral[j];
            if (std::fabs(implicitTau) <= jt.effort || jt.leader >= 0) {
                t = implicitTau;
                x = jt.kp * dt * dt + jt.kd * dt;
            } else {
                t = std::clamp(jt.kp * (target - q[j]) - jt.kd * qd[j] + ki * integral[j], -jt.effort, jt.effort);
            }
        } else if (mode == ActuatorMode::Velocity) {
            const double implicitTau = jt.kv * (target - qd[j]);
            if (std::fabs(implicitTau) <= jt.effort) {
                t = implicitTau;
                x = jt.kv * dt;
            } else {
                t = std::clamp(implicitTau, -jt.effort, jt.effort);
            }
        }
        if (mode != ActuatorMode::Passive) t += ctl.torque;
        tau[j] = t;
        // Joint friction (bearings of spinning joints, wheels and rotors, far less).
        const double b = jt.type == RigJointType::Continuous ? opt.jointDamping * 0.005 : opt.jointDamping;
        t -= b * qd[j];
        x += b * dt;
        // Hard stops.
        if (jt.limited) {
            if (q[j] < jt.lo) {
                t += jt.limitK * (jt.lo - q[j] - dt * qd[j]) - jt.limitD * qd[j];
                x += jt.limitK * dt * dt + jt.limitD * dt;
            } else if (q[j] > jt.hi) {
                t += jt.limitK * (jt.hi - q[j] - dt * qd[j]) - jt.limitD * qd[j];
                x += jt.limitK * dt * dt + jt.limitD * dt;
            }
        }
        u[j + 1] = t;
        extra[j] = x + jt.armature;
    }
    // Articulated inertias, from the leaves in.
    for (std::size_t j = nj; j-- > 0;) {
        const std::size_t i = j + 1, p = static_cast<std::size_t>(joints[j].parent);
        U[i] = IA[i] * S[i];
        D[i] = sdot(S[i], U[i]) + extra[j];
        u[i] = u[i] - sdot(S[i], pA[i]);
        M6 Ia = IA[i];
        for (int r = 0; r < 6; ++r) {
            const double ur = r < 3 ? U[i].w[static_cast<std::size_t>(r)] : U[i].v[static_cast<std::size_t>(r - 3)];
            for (int col = 0; col < 6; ++col) {
                const double uc = col < 3 ? U[i].w[static_cast<std::size_t>(col)] : U[i].v[static_cast<std::size_t>(col - 3)];
                Ia.a[r * 6 + col] -= ur * uc / D[i];
            }
        }
        const SV pa = pA[i] + Ia * c[i] + U[i] * (u[i] / D[i]);
        IA[p] += Ia;
        pA[p] = pA[p] + pa;
    }
    // Accelerations, from the root out.
    a[0] = rig.floatingBase ? solve6(IA[0], pA[0] * -1.0) : SV{};
    for (std::size_t j = 0; j < nj; ++j) {
        const std::size_t i = j + 1, p = static_cast<std::size_t>(joints[j].parent);
        const SV ap = a[p] + c[i];
        qdd[j] = (u[i] - sdot(U[i], ap)) / D[i];
        a[i] = ap + S[i] * qdd[j];
    }
    // Joint wrenches (net spatial force each joint passes on), from the leaves in.
    for (std::size_t l = 0; l < nl; ++l) f[l] = I[l] * a[l] + crf(v[l], I[l] * v[l]) - I[l] * ag - fext[l];
    for (std::size_t j = nj; j-- > 0;) {
        const std::size_t i = j + 1, p = static_cast<std::size_t>(joints[j].parent);
        const V3d origin = add(mv(R[p], joints[j].origin0), T[p]);
        jointWrench[j] = {f[i].v, sub(f[i].w, cross(origin, f[i].v))};
        f[p] = f[p] + f[i];
    }
    acc = a;
}

Multibody::Multibody(const RobotRig& rig, MultibodyOptions options) : m(std::make_unique<Impl>()) {
    m->rig = rig;
    m->opt = std::move(options);
    m->setup();
    reset();
}

Multibody::~Multibody() = default;
Multibody::Multibody(const Multibody& o) : m(std::make_unique<Impl>(*o.m)) {}
Multibody& Multibody::operator=(const Multibody& o) {
    if (this != &o) m = std::make_unique<Impl>(*o.m);
    return *this;
}

const RobotRig& Multibody::rig() const { return m->rig; }
const MultibodyOptions& Multibody::options() const { return m->opt; }
MultibodyOptions& Multibody::options() { return m->opt; }
std::size_t Multibody::jointCount() const { return m->joints.size(); }
bool Multibody::floatingBase() const { return m->rig.floatingBase; }

void Multibody::reset() {
    std::fill(m->q.begin(), m->q.end(), 0.0);
    std::fill(m->qd.begin(), m->qd.end(), 0.0);
    std::fill(m->tau.begin(), m->tau.end(), 0.0);
    std::fill(m->integral.begin(), m->integral.end(), 0.0);
    m->baseR = identity3();
    m->baseT = {0, 0, 0};
    m->baseV = {};
    m->time = 0;
    m->diverged = false;
    for (auto& t : m->touching) std::fill(t.begin(), t.end(), 0);
    m->contacts.clear();
    m->kinematics();
}

void Multibody::step() {
    Impl& s = *m;
    if (s.diverged) return;
    for (auto& w : s.fext) w = {};
    if (!s.hooks.empty()) {
        s.kinematics(); // hooks see this step's poses and velocities
        for (ForceHook& h : s.hooks) h(*this);
    }
    s.kinematics();
    for (const auto* list : {&s.once, &s.persistent})
        for (const Impl::Wrench& w : *list)
            if (w.link >= 0 && static_cast<std::size_t>(w.link) < s.fext.size())
                s.fext[static_cast<std::size_t>(w.link)] = s.fext[static_cast<std::size_t>(w.link)] + forceAt(w.force, w.point, w.torque);
    s.once.clear();
    s.computeContacts();
    s.dynamics();
    const double dt = s.opt.dt;
    // Semi-implicit Euler: velocities first, positions with the new velocities.
    for (std::size_t j = 0; j < s.q.size(); ++j) {
        s.qd[j] += s.qdd[j] * dt;
        s.q[j] += s.qd[j] * dt;
    }
    if (s.rig.floatingBase) {
        // Integrate the base's centre of mass and angular velocity (classical), so a base no
        // force acts on keeps its momentum exactly; then rebuild the spatial velocity.
        const V3d c = add(mv(s.baseR, s.links[0].com0), s.baseT);
        const V3d w = s.baseV.w;
        const V3d vc = add(s.baseV.v, cross(w, c));
        const V3d ac = add(add(s.a[0].v, cross(s.a[0].w, c)), cross(w, vc));
        const V3d w1 = add(w, mul(s.a[0].w, dt));
        const V3d vc1 = add(vc, mul(ac, dt));
        const double wn = norm(w1);
        if (wn > 1e-12) s.baseR = mm(rotation(mul(w1, 1.0 / wn), wn * dt), s.baseR);
        const V3d c1 = add(c, mul(vc1, dt));
        s.baseT = sub(c1, mv(s.baseR, s.links[0].com0));
        s.baseV = {w1, sub(vc1, cross(w1, c1))};
    }
    s.time += dt;
    // A run that blew up stops (and says so) rather than drawing nonsense.
    double worst = 0;
    for (double x : s.qd) worst = std::max(worst, std::fabs(x));
    worst = std::max(worst, norm(s.baseV.w));
    if (!std::isfinite(worst) || worst > 1e5) s.diverged = true;
    for (std::size_t j = 0; j < s.q.size(); ++j)
        if (!std::isfinite(s.q[j])) s.diverged = true;
}

void Multibody::advance(double seconds) {
    const long n = std::lround(seconds / m->opt.dt);
    for (long i = 0; i < n && !m->diverged; ++i) step();
}

double Multibody::time() const { return m->time; }

void Multibody::setControl(std::size_t joint, const JointControl& c) {
    if (joint < m->controls.size()) m->controls[joint] = c;
}
const JointControl& Multibody::control(std::size_t joint) const { return m->controls[joint]; }
void Multibody::holdRest() {
    for (JointControl& c : m->controls) c = {};
}

void Multibody::applyWrench(int link, const V3d& force, const V3d& torque, const V3d& point, bool persistent) {
    (persistent ? m->persistent : m->once).push_back({link, force, torque, point});
}
void Multibody::clearWrenches() {
    m->once.clear();
    m->persistent.clear();
}
void Multibody::addForceHook(ForceHook hook) { m->hooks.push_back(std::move(hook)); }

std::vector<double> Multibody::q() const { return m->q; }
std::vector<double> Multibody::qd() const { return m->qd; }
std::vector<double> Multibody::torques() const { return m->tau; }
void Multibody::setJointState(const std::vector<double>& q, const std::vector<double>& qd) {
    if (q.size() == m->q.size()) m->q = q;
    if (qd.size() == m->qd.size()) m->qd = qd;
    m->kinematics();
}

LinkPose Multibody::linkPose(int link) const {
    LinkPose p;
    if (link < 0 || static_cast<std::size_t>(link) >= m->R.size()) return p;
    p.R = m->R[static_cast<std::size_t>(link)];
    p.t = m->T[static_cast<std::size_t>(link)];
    return p;
}
V3d Multibody::linkCom(int link) const {
    const std::size_t l = static_cast<std::size_t>(link);
    return add(mv(m->R[l], m->links[l].com0), m->T[l]);
}
V3d Multibody::linkAngularVelocity(int link) const { return m->v[static_cast<std::size_t>(link)].w; }
V3d Multibody::linkVelocity(int link) const {
    const SV& v = m->v[static_cast<std::size_t>(link)];
    return add(v.v, cross(v.w, linkCom(link)));
}
V3d Multibody::linkAcceleration(int link) const {
    const std::size_t l = static_cast<std::size_t>(link);
    const SV& a = m->acc[l];
    const SV& v = m->v[l];
    const V3d p = linkCom(link);
    // Classical acceleration of the point p from the spatial acceleration.
    return add(add(a.v, cross(a.w, p)), cross(v.w, add(v.v, cross(v.w, p))));
}
LinkPose Multibody::basePose() const {
    LinkPose p;
    p.R = m->baseR;
    p.t = m->baseT;
    return p;
}
void Multibody::setBasePose(const LinkPose& p) {
    m->baseR = p.R;
    m->baseT = p.t;
    m->kinematics();
}
void Multibody::setBaseVelocity(const V3d& angular, const V3d& linear) {
    // linear: of the base's centre of mass; stored as the velocity field at the origin.
    const V3d com = linkCom(0);
    m->baseV = {angular, sub(linear, cross(angular, com))};
    m->kinematics();
}
V3d Multibody::centreOfMass() const {
    V3d c{0, 0, 0};
    double mass = 0;
    for (std::size_t l = 0; l < m->links.size(); ++l) {
        c = add(c, mul(linkCom(static_cast<int>(l)), m->links[l].mass));
        mass += m->links[l].mass;
    }
    return mass > 0 ? mul(c, 1.0 / mass) : c;
}
double Multibody::kineticEnergy() const {
    double e = 0;
    for (std::size_t l = 0; l < m->links.size(); ++l) e += 0.5 * sdot(m->v[l], m->I[l] * m->v[l]);
    return e;
}
double Multibody::potentialEnergy() const {
    double e = 0;
    for (std::size_t l = 0; l < m->links.size(); ++l) e -= m->links[l].mass * dot(m->opt.gravity, linkCom(static_cast<int>(l)));
    return e;
}
const std::vector<ContactPoint>& Multibody::contacts() const { return m->contacts; }
std::pair<V3d, V3d> Multibody::jointWrench(std::size_t joint) const { return m->jointWrench[joint]; }
bool Multibody::diverged() const { return m->diverged; }

} // namespace sicdm

// Learning environments: batches of robot copies stepped in parallel (see RobotEnv.hpp).

#include "core/RobotEnv.hpp"

#include "core/ForceField.hpp"
#include "core/Physics.hpp"
#include "core/Rig.hpp"
#include "core/Scene.hpp"
#include "core/Vehicle.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <map>
#include <mutex>
#include <thread>

namespace sicdm {

namespace {

constexpr double kPiD = 3.14159265358979323846;

// Deterministic random numbers (splitmix64).
struct Rng {
    std::uint64_t s;
    explicit Rng(std::uint64_t seed) : s(seed * 0x9E3779B97F4A7C15ull + 0xD1B54A32D192ED03ull) {}
    double next() {
        std::uint64_t z = (s += 0x9E3779B97F4A7C15ull);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        z ^= z >> 31;
        return static_cast<double>(z >> 11) * (1.0 / 9007199254740992.0);
    }
};

// R^T v: a world vector in a pose's frame.
V3d intoFrame(const LinkPose& p, const V3d& v) {
    return {p.R[0] * v[0] + p.R[3] * v[1] + p.R[6] * v[2], p.R[1] * v[0] + p.R[4] * v[1] + p.R[7] * v[2], p.R[2] * v[0] + p.R[5] * v[1] + p.R[8] * v[2]};
}

} // namespace

const char* envTaskKey(EnvTask t) noexcept {
    switch (t) {
    case EnvTask::None: return "none";
    case EnvTask::Stand: return "stand";
    case EnvTask::Walk: return "walk";
    case EnvTask::Reach: return "reach";
    case EnvTask::Hover: return "hover";
    }
    return "none";
}

bool envTaskFromKey(const std::string& key, EnvTask& out) noexcept {
    std::string k;
    for (char c : key) k += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    for (EnvTask t : {EnvTask::None, EnvTask::Stand, EnvTask::Walk, EnvTask::Reach, EnvTask::Hover})
        if (k == envTaskKey(t)) {
            out = t;
            return true;
        }
    return false;
}

struct RobotEnv::Impl {
    EnvOptions opt;
    RobotRig rig;
    std::unique_ptr<Multibody> start;        // the robot as every episode starts
    std::vector<std::unique_ptr<Multibody>> envs;
    std::vector<std::size_t> motors;         // joints the actions drive
    std::vector<char> spins;                 // per motor: speed controlled (wheels, rotors)
    std::vector<double> q0;                  // the start's joint values
    std::vector<double> lo, hi, effort, speed; // per motor
    std::vector<std::string> obsNames, actNames;
    double z0 = 0;                           // the start's centre of mass height (m)
    V3d com0{};
    int tool = -1;                           // Reach: the end effector...
    V3d toolPoint{};                         // ...and the point on it (at rest, m)
    std::vector<double> ret;                 // per copy: this episode's return so far
    std::vector<int> length;                 // and its length in actions
    std::vector<std::uint64_t> episodes;     // episodes each copy has started
    std::uint64_t seed = 1;
    double simulated = 0;
    std::vector<std::string> notes;

    void startEpisode(std::size_t i);
    void observe(std::size_t i, double* out) const;
    double reward(std::size_t i, const double* action) const;
    bool fallen(std::size_t i) const;
    std::size_t obsSize() const { return obsNames.size(); }
};

void RobotEnv::Impl::startEpisode(std::size_t i) {
    envs[i] = std::make_unique<Multibody>(*start);
    Multibody& mb = *envs[i];
    // Spread the starting pose a little, the same way for the same seed and episode.
    Rng rng(seed ^ (static_cast<std::uint64_t>(i) << 32) ^ (episodes[i] * 0x2545F4914F6CDD1Dull));
    std::vector<double> q = mb.q();
    for (std::size_t k = 0; k < motors.size(); ++k) {
        const std::size_t j = motors[k];
        if (spins[k]) continue;
        q[j] = std::clamp(q[j] + opt.noise * (2 * rng.next() - 1), lo[k], hi[k]);
    }
    mb.setJointState(q, std::vector<double>(q.size(), 0.0));
    ret[i] = 0, length[i] = 0;
    ++episodes[i];
}

void RobotEnv::Impl::observe(std::size_t i, double* out) const {
    const Multibody& mb = *envs[i];
    const std::vector<double> q = mb.q(), qd = mb.qd();
    std::size_t k = 0;
    for (std::size_t j : motors) out[k++] = q[j];
    for (std::size_t j : motors) out[k++] = qd[j];
    const LinkPose base = mb.basePose();
    out[k++] = mb.centreOfMass()[2];
    const V3d up = intoFrame(base, {0, 0, 1});
    const V3d v = rig.floatingBase ? intoFrame(base, mb.linkVelocity(0)) : V3d{0, 0, 0};
    const V3d w = rig.floatingBase ? intoFrame(base, mb.linkAngularVelocity(0)) : V3d{0, 0, 0};
    for (double x : up) out[k++] = x;
    for (double x : v) out[k++] = x;
    for (double x : w) out[k++] = x;
    if (opt.task == EnvTask::Reach) {
        const V3d p = mb.linkPose(tool).apply(toolPoint);
        for (int c = 0; c < 3; ++c) out[k++] = p[static_cast<std::size_t>(c)] - opt.target[static_cast<std::size_t>(c)];
    } else if (opt.task == EnvTask::Hover) {
        const V3d c = mb.centreOfMass();
        for (int d = 0; d < 3; ++d) out[k++] = c[static_cast<std::size_t>(d)] - com0[static_cast<std::size_t>(d)];
    }
}

double RobotEnv::Impl::reward(std::size_t i, const double* action) const {
    const Multibody& mb = *envs[i];
    double used = 0; // mean squared action
    for (std::size_t k = 0; k < motors.size(); ++k) used += action[k] * action[k];
    used /= static_cast<double>(std::max<std::size_t>(motors.size(), 1));
    const V3d c = mb.centreOfMass();
    switch (opt.task) {
    case EnvTask::None: return 0;
    case EnvTask::Stand: return 1.0 - 2.0 * std::fabs(c[2] - z0) / std::max(z0, 1e-3) - 0.01 * used;
    case EnvTask::Walk: {
        const V3d v = comVelocity(mb);
        return v[0] + 0.5 - 0.01 * used;
    }
    case EnvTask::Reach: {
        const V3d p = mb.linkPose(tool).apply(toolPoint);
        const double d = std::hypot(p[0] - opt.target[0], p[1] - opt.target[1], p[2] - opt.target[2]);
        return -d - 0.01 * used;
    }
    case EnvTask::Hover: {
        const double d = std::hypot(c[0] - com0[0], c[1] - com0[1], c[2] - com0[2]);
        return 1.0 - d - 0.01 * used;
    }
    }
    return 0;
}

bool RobotEnv::Impl::fallen(std::size_t i) const {
    const Multibody& mb = *envs[i];
    if (mb.diverged()) return true;
    if (!rig.floatingBase || opt.task == EnvTask::None) return false;
    const LinkPose b = mb.basePose();
    const double tilt = std::acos(std::clamp(b.R[8], -1.0, 1.0)); // the base's z axis from vertical
    return tilt > kPiD / 3 || mb.centreOfMass()[2] < 0.4 * z0;
}

RobotEnv::RobotEnv() : m(std::make_unique<Impl>()) {}
RobotEnv::~RobotEnv() = default;

std::unique_ptr<RobotEnv> RobotEnv::create(const Scene& scene, const RigSettings& robot, const EnvOptions& options, std::string* error) {
    std::unique_ptr<RobotEnv> env(new RobotEnv());
    Impl& I = *env->m;
    I.opt = options;
    I.opt.count = std::clamp(options.count, 1, 4096);
    I.opt.controlDt = std::clamp(options.controlDt, 0.0005, 1.0);
    I.opt.episodeSeconds = std::max(options.episodeSeconds, I.opt.controlDt);
    I.seed = options.seed;
    std::string err;
    if (!buildRig(scene, robot, I.rig, &err)) {
        if (error) *error = err;
        return nullptr;
    }
    const SimEngine engine = engineFor(I.rig, robot);
    if (engine == SimEngine::Vehicle) {
        const Vec3 w = scene.physics().wind;
        sizeRotorMotors(I.rig, vehicleModel(I.rig, {w.x, w.y, w.z}));
    }
    for (std::size_t j = 0; j < I.rig.joints.size(); ++j)
        if (I.rig.joints[j].mimic.empty()) I.motors.push_back(j);
    if (I.motors.empty()) {
        if (error) *error = "The robot has no motors (joints) to drive";
        return nullptr;
    }
    if (I.opt.task == EnvTask::Reach) {
        const std::vector<int> ee = I.rig.endEffectors();
        if (ee.empty() || ee.front() <= 0) {
            if (error) *error = "REACH needs an end effector: RIG TAG part END_EFFECTOR";
            return nullptr;
        }
        I.tool = ee.front();
        const RigLink& L = I.rig.links[static_cast<std::size_t>(I.tool)];
        for (int c = 0; c < 3; ++c) I.toolPoint[static_cast<std::size_t>(c)] = 0.5 * (L.boxMin[static_cast<std::size_t>(c)] + L.boxMax[static_cast<std::size_t>(c)]);
    }
    MultibodyOptions mo;
    mo.ground = sceneGround(scene);
    mo.gravity = {scene.physics().g.x, scene.physics().g.y, scene.physics().g.z};
    if (scene.physics().gravity == GravityMode::Off) mo.gravity = {0, 0, 0};
    I.start = std::make_unique<Multibody>(I.rig, mo);
    Multibody& mb = *I.start;
    // The programmed pose at time 0, set down on the ground (as a simulation run starts).
    const MotionTargets t0 = motionTargets(scene, I.rig, 0.0f);
    I.q0 = t0.value;
    for (std::size_t j = 0; j < I.q0.size(); ++j)
        if (t0.velocity[j] || I.rig.joints[j].type == RigJointType::Continuous) I.q0[j] = 0;
    mb.setJointState(I.q0, std::vector<double>(I.q0.size(), 0.0));
    if (I.rig.floatingBase && mo.ground.height) {
        double lowest = 1e300;
        for (std::size_t l = 0; l < I.rig.links.size(); ++l) {
            const LinkPose p = mb.linkPose(static_cast<int>(l));
            for (const auto& c : I.rig.links[l].contactPoints) {
                const V3d w = p.apply(c);
                lowest = std::min(lowest, w[2] - mo.ground.height(w[0], w[1]));
            }
        }
        if (lowest < 0 && lowest > -1e299) {
            LinkPose b = mb.basePose();
            b.t[2] -= lowest;
            mb.setBasePose(b);
        }
    }
    if (engine == SimEngine::Vehicle) {
        const Vec3 w = scene.physics().wind;
        addVehicleForces(mb, vehicleModel(I.rig, {w.x, w.y, w.z}));
    }
    // The sandbox's forces act on every copy.
    for (const std::string& n : addRobotForces(mb, scene, robot)) I.notes.push_back(n);
    // Motors hold the start until the first action.
    for (std::size_t j = 0; j < I.rig.joints.size(); ++j) {
        JointControl c;
        c.target = I.q0[j];
        if (I.rig.joints[j].type == RigJointType::Continuous) c.mode = ActuatorMode::Velocity, c.target = 0;
        mb.setControl(j, c);
    }
    I.z0 = mb.centreOfMass()[2];
    I.com0 = mb.centreOfMass();
    for (std::size_t j : I.motors) {
        const RigJoint& J = I.rig.joints[j];
        const bool spin = J.type == RigJointType::Continuous;
        I.spins.push_back(spin ? 1 : 0);
        I.lo.push_back(spin ? -1e9 : J.lo);
        I.hi.push_back(spin ? 1e9 : J.hi);
        I.effort.push_back(rigJointEffort(I.rig, j));
        I.speed.push_back(rigJointVelocity(I.rig, j));
        I.actNames.push_back(J.name);
    }
    for (std::size_t j : I.motors) I.obsNames.push_back(I.rig.joints[j].name + " position");
    for (std::size_t j : I.motors) I.obsNames.push_back(I.rig.joints[j].name + " rate");
    for (const char* n : {"height", "up x", "up y", "up z", "velocity x", "velocity y", "velocity z", "turn rate x", "turn rate y", "turn rate z"})
        I.obsNames.push_back(n);
    if (I.opt.task == EnvTask::Reach)
        for (const char* n : {"tool to target x", "tool to target y", "tool to target z"}) I.obsNames.push_back(n);
    if (I.opt.task == EnvTask::Hover)
        for (const char* n : {"drift x", "drift y", "drift z"}) I.obsNames.push_back(n);
    const std::size_t n = static_cast<std::size_t>(I.opt.count);
    I.envs.resize(n);
    I.ret.assign(n, 0.0), I.length.assign(n, 0), I.episodes.assign(n, 0);
    for (std::size_t i = 0; i < n; ++i) I.startEpisode(i);
    return env;
}

int RobotEnv::count() const { return m->opt.count; }
std::size_t RobotEnv::observationSize() const { return m->obsSize(); }
std::size_t RobotEnv::actionSize() const { return m->motors.size(); }
const std::vector<std::string>& RobotEnv::observationNames() const { return m->obsNames; }
const std::vector<std::string>& RobotEnv::actionNames() const { return m->actNames; }
const RobotRig& RobotEnv::rig() const { return m->rig; }
const EnvOptions& RobotEnv::options() const { return m->opt; }
double RobotEnv::simulatedSeconds() const { return m->simulated; }

std::vector<double> RobotEnv::reset(std::uint64_t seed) {
    if (seed != 0) m->seed = seed;
    const std::size_t n = m->envs.size();
    std::fill(m->episodes.begin(), m->episodes.end(), 0);
    std::vector<double> obs(n * m->obsSize());
    for (std::size_t i = 0; i < n; ++i) {
        m->startEpisode(i);
        m->observe(i, obs.data() + i * m->obsSize());
    }
    return obs;
}

const std::vector<std::string>& RobotEnv::notes() const { return m->notes; }

EnvStep RobotEnv::step(const std::vector<double>& actions, const std::vector<EnvWrench>& wrenches) {
    Impl& I = *m;
    const std::size_t n = I.envs.size(), na = I.motors.size(), no = I.obsSize();
    EnvStep out;
    out.observations.assign(n * no, 0.0);
    out.rewards.assign(n, 0.0);
    out.terminated.assign(n, 0);
    out.truncated.assign(n, 0);
    const long substeps = std::max(1L, std::lround(I.opt.controlDt / I.start->options().dt));
    const int maxLength = std::max(1, static_cast<int>(std::lround(I.opt.episodeSeconds / I.opt.controlDt)));
    std::vector<char> ended(n, 0);
    auto run = [&](std::size_t from, std::size_t to) {
        std::vector<double> a(na);
        for (std::size_t i = from; i < to; ++i) {
            Multibody& mb = *I.envs[i];
            for (std::size_t k = 0; k < na; ++k) {
                const std::size_t idx = i * na + k;
                a[k] = idx < actions.size() && std::isfinite(actions[idx]) ? std::clamp(actions[idx], -1.0, 1.0) : 0.0;
                const std::size_t j = I.motors[k];
                JointControl c;
                if (I.opt.torque) {
                    c.mode = ActuatorMode::Torque;
                    c.torque = a[k] * I.effort[k];
                } else if (I.spins[k]) {
                    c.mode = ActuatorMode::Velocity;
                    c.target = a[k] * I.speed[k];
                } else {
                    c.mode = ActuatorMode::Position;
                    const double half = 0.5 * (I.hi[k] - I.lo[k]);
                    c.target = std::clamp(I.q0[j] + a[k] * half, I.lo[k], I.hi[k]);
                }
                mb.setControl(j, c);
            }
            // Wrenches sent with this step act until the next action.
            std::vector<const EnvWrench*> mine;
            for (const EnvWrench& w : wrenches)
                if ((w.env < 0 || w.env == static_cast<int>(i)) && w.link >= 0 && static_cast<std::size_t>(w.link) < I.rig.links.size()) mine.push_back(&w);
            for (long s = 0; s < substeps && !mb.diverged(); ++s) {
                for (const EnvWrench* wp : mine) {
                    const EnvWrench& w = *wp;
                    const LinkPose p = mb.linkPose(w.link);
                    V3d F = w.force, T = w.torque;
                    if (w.local) {
                        F = {p.R[0] * w.force[0] + p.R[1] * w.force[1] + p.R[2] * w.force[2], p.R[3] * w.force[0] + p.R[4] * w.force[1] + p.R[5] * w.force[2],
                             p.R[6] * w.force[0] + p.R[7] * w.force[1] + p.R[8] * w.force[2]};
                        T = {p.R[0] * w.torque[0] + p.R[1] * w.torque[1] + p.R[2] * w.torque[2], p.R[3] * w.torque[0] + p.R[4] * w.torque[1] + p.R[5] * w.torque[2],
                             p.R[6] * w.torque[0] + p.R[7] * w.torque[1] + p.R[8] * w.torque[2]};
                    }
                    mb.applyWrench(w.link, F, T, p.apply(w.point));
                }
                mb.step();
            }
            out.rewards[i] = mb.diverged() ? -1.0 : I.reward(i, a.data());
            I.ret[i] += out.rewards[i];
            ++I.length[i];
            out.terminated[i] = I.fallen(i) ? 1 : 0;
            out.truncated[i] = !out.terminated[i] && I.length[i] >= maxLength ? 1 : 0;
            ended[i] = out.terminated[i] || out.truncated[i];
            if (ended[i]) continue; // observed after its new start, below
            I.observe(i, out.observations.data() + i * no);
        }
    };
    const unsigned hw = std::max(1u, std::thread::hardware_concurrency());
    const std::size_t threads = std::min<std::size_t>(n, I.opt.threads > 0 ? static_cast<std::size_t>(I.opt.threads) : hw);
    if (threads <= 1) {
        run(0, n);
    } else {
        std::vector<std::thread> pool;
        const std::size_t per = (n + threads - 1) / threads;
        for (std::size_t t = 0; t < threads; ++t) {
            const std::size_t a = t * per, b = std::min(n, a + per);
            if (a < b) pool.emplace_back(run, a, b);
        }
        for (std::thread& th : pool) th.join();
    }
    I.simulated += static_cast<double>(n) * static_cast<double>(substeps) * I.start->options().dt;
    // Finished copies start again; their first observation is of the new episode.
    for (std::size_t i = 0; i < n; ++i) {
        if (!ended[i]) continue;
        out.episodes.push_back({static_cast<int>(i), I.ret[i], I.length[i]});
        I.startEpisode(i);
        I.observe(i, out.observations.data() + i * no);
    }
    return out;
}

// --- kept environments -------------------------------------------------------------------------

namespace {
std::mutex& envLock() {
    static std::mutex m;
    return m;
}
std::map<int, std::unique_ptr<RobotEnv>>& envTable() {
    static std::map<int, std::unique_ptr<RobotEnv>> t;
    return t;
}
} // namespace

int keepEnv(std::unique_ptr<RobotEnv> env) {
    static int next = 1;
    std::lock_guard<std::mutex> g(envLock());
    // A client that forgets to close them can't run the machine out of memory.
    while (envTable().size() >= 16) envTable().erase(envTable().begin());
    const int id = next++;
    envTable()[id] = std::move(env);
    return id;
}

RobotEnv* findEnv(int id) {
    std::lock_guard<std::mutex> g(envLock());
    const auto it = envTable().find(id);
    return it == envTable().end() ? nullptr : it->second.get();
}

bool closeEnv(int id) {
    std::lock_guard<std::mutex> g(envLock());
    return envTable().erase(id) > 0;
}

std::vector<int> envIds() {
    std::lock_guard<std::mutex> g(envLock());
    std::vector<int> ids;
    for (const auto& e : envTable()) ids.push_back(e.first);
    return ids;
}

} // namespace sicdm

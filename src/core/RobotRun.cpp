// Robot runs for the Simulation: a rig's motions as servo targets, simulated by its engine
// and recorded for playback.

#include "core/Json.hpp"
#include "core/Motion.hpp"
#include "core/Multibody.hpp"
#include "core/ForceField.hpp"
#include "core/Scene.hpp"
#include "core/Vehicle.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>

namespace sicdm {

namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

std::uint64_t fnv(std::uint64_t h, const void* data, std::size_t n) {
    const auto* p = static_cast<const unsigned char*>(data);
    for (std::size_t i = 0; i < n; ++i) h = (h ^ p[i]) * 1099511628211ull;
    return h;
}
template <class T>
std::uint64_t mix(std::uint64_t h, const T& v) {
    return fnv(h, &v, sizeof v);
}

// Everything a robot run depends on.
std::uint64_t runKey(const Scene& scene, const RigSettings& settings, float duration) {
    std::uint64_t h = 1469598103934665603ull;
    const std::string rigs = rigStateToJson(scene.rigs()).dump();
    h = fnv(h, rigs.data(), rigs.size());
    h = fnv(h, settings.name.data(), settings.name.size());
    h = mix(h, duration);
    for (const Body& b : scene.bodies()) {
        h = fnv(h, b.name.data(), b.name.size());
        h = fnv(h, b.component.data(), b.component.size());
        h = fnv(h, b.attachedTo.data(), b.attachedTo.size());
        h = mix(h, b.offset), h = mix(h, b.scale), h = mix(h, b.rotation), h = mix(h, b.mesh.vertices.size());
        h = mix(h, b.physics.mass), h = mix(h, b.physics.density);
        const AABB bb = b.mesh.bounds();
        h = mix(h, bb.min), h = mix(h, bb.max);
    }
    for (const Motion& m : scene.motions()) {
        h = fnv(h, m.name.data(), m.name.size());
        h = mix(h, m.enabled), h = mix(h, m.kind), h = mix(h, m.speed), h = mix(h, m.range), h = mix(h, m.mode);
        h = mix(h, m.start), h = mix(h, m.axis), h = mix(h, m.origin), h = mix(h, m.ratio), h = mix(h, m.keys.size());
        h = mix(h, m.hasLimits), h = mix(h, m.lo), h = mix(h, m.hi), h = mix(h, m.keySpline), h = mix(h, m.eachCentre);
        for (const MotionKey& k : m.keys) h = mix(h, k);
        h = fnv(h, m.follow.data(), m.follow.size());
        for (const std::string& n : m.bodies) h = fnv(h, n.data(), n.size());
    }
    const std::string terrain = terrainToJson(scene.terrain()).dump();
    h = fnv(h, terrain.data(), terrain.size());
    const PhysicsSettings& ps = scene.physics();
    h = mix(h, ps.g), h = mix(h, ps.floor), h = mix(h, ps.friction);
    return h;
}

Mat4 poseMatrix(const LinkPose& p) {
    // world (mm) = R * rest (mm) + t (m) * 1000
    Mat4 m = Mat4::identity();
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) m.m[c][r] = static_cast<float>(p.R[static_cast<std::size_t>(r * 3 + c)]);
    for (int r = 0; r < 3; ++r) m.m[3][r] = static_cast<float>(p.t[static_cast<std::size_t>(r)] * 1000.0);
    return m;
}

LinkPose lerpPose(const LinkPose& a, const LinkPose& b, double u) {
    LinkPose o;
    for (int i = 0; i < 3; ++i) o.t[static_cast<std::size_t>(i)] = a.t[static_cast<std::size_t>(i)] * (1 - u) + b.t[static_cast<std::size_t>(i)] * u;
    // Blend the rotations and re-orthonormalise (frames are 10 ms apart: small angles).
    std::array<double, 9> R;
    for (int i = 0; i < 9; ++i) R[static_cast<std::size_t>(i)] = a.R[static_cast<std::size_t>(i)] * (1 - u) + b.R[static_cast<std::size_t>(i)] * u;
    double c0[3] = {R[0], R[3], R[6]}, c1[3] = {R[1], R[4], R[7]};
    auto normalise = [](double* v) {
        const double n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        if (n > 0)
            for (int i = 0; i < 3; ++i) v[i] /= n;
    };
    normalise(c0);
    const double d = c0[0] * c1[0] + c0[1] * c1[1] + c0[2] * c1[2];
    for (int i = 0; i < 3; ++i) c1[i] -= d * c0[i];
    normalise(c1);
    const double c2[3] = {c0[1] * c1[2] - c0[2] * c1[1], c0[2] * c1[0] - c0[0] * c1[2], c0[0] * c1[1] - c0[1] * c1[0]};
    o.R = {c0[0], c1[0], c2[0], c0[1], c1[1], c2[1], c0[2], c1[2], c2[2]};
    return o;
}

} // namespace

Ground sceneGround(const Scene& scene) {
    const TerrainSettings& t = scene.terrain();
    bool drawn = false;
    for (const Body& b : scene.bodies()) drawn = drawn || b.name == kTerrainBody;
    if (!t.active() || !drawn) return Ground::flat(0.0, 0.9);
    Ground g;
    auto shared = std::make_shared<TerrainSettings>(t);
    g.height = [shared](double x, double y) { return terrainHeight(*shared, x * 1e3, y * 1e3) * 1e-3; };
    g.normal = [shared](double x, double y) {
        const Vec3 n = terrainNormal(*shared, x * 1e3, y * 1e3);
        return V3d{n.x, n.y, n.z};
    };
    g.friction = t.friction;
    return g;
}

MotionTargets motionTargets(const Scene& scene, const RobotRig& rig, float t) {
    MotionTargets out;
    const std::size_t n = rig.joints.size();
    out.value.assign(n, 0.0), out.rate.assign(n, 0.0), out.velocity.assign(n, 0);
    for (std::size_t j = 0; j < n; ++j) {
        const RigJoint& J = rig.joints[j];
        if (J.motion < 0 || static_cast<std::size_t>(J.motion) >= scene.motions().size()) continue;
        const Motion& m = scene.motions()[static_cast<std::size_t>(J.motion)];
        const double unit = J.type == RigJointType::Prismatic ? 1e-3 : kDegToRad;
        out.value[j] = motionValue(scene, m, t) * unit;
        out.rate[j] = motionRate(scene, m, t) * unit;
        out.velocity[j] = J.type == RigJointType::Continuous ? 1 : 0;
    }
    return out;
}

RobotRunFrame RobotRun::at(float t) const {
    if (frames.empty()) return {};
    if (t <= 0) return frames.front();
    const float f = t / frameStep;
    const std::size_t i = static_cast<std::size_t>(f);
    if (i + 1 >= frames.size()) return frames.back();
    const double u = f - static_cast<float>(i);
    RobotRunFrame o = frames[i];
    o.t = t;
    for (std::size_t l = 0; l < o.links.size(); ++l) o.links[l] = lerpPose(frames[i].links[l], frames[i + 1].links[l], u);
    for (std::size_t j = 0; j < o.q.size(); ++j) o.q[j] = static_cast<float>(frames[i].q[j] * (1 - u) + frames[i + 1].q[j] * u);
    return o;
}

std::shared_ptr<const RobotRun> robotRun(const Scene& scene, const RigSettings& settings, float duration) {
    // As long as the physics' longest run (every frame is kept for playback).
    duration = std::isfinite(duration) ? std::clamp(duration, 0.0f, kMaxRobotRun) : kMaxRobotRun;
    static std::mutex lock;
    static std::vector<std::pair<std::uint64_t, std::shared_ptr<const RobotRun>>> cache;
    const std::uint64_t key = runKey(scene, settings, duration);
    {
        std::lock_guard<std::mutex> g(lock);
        for (std::size_t i = 0; i < cache.size(); ++i)
            if (cache[i].first == key) return cache[i].second;
    }
    auto run = std::make_shared<RobotRun>();
    run->robot = settings.name;
    std::string err;
    if (!buildRig(scene, settings, run->rig, &err)) return nullptr;
    run->engine = engineFor(run->rig, settings);
    if (run->engine == SimEngine::Particle) return nullptr;
    // Vehicles: size the rotor motors before the engine is made.
    if (run->engine == SimEngine::Vehicle) {
        const Vec3 w = scene.physics().wind;
        sizeRotorMotors(run->rig, vehicleModel(run->rig, {w.x, w.y, w.z}));
    }
    MultibodyOptions opt;
    opt.ground = sceneGround(scene);
    opt.gravity = {scene.physics().g.x, scene.physics().g.y, scene.physics().g.z};
    if (scene.physics().gravity == GravityMode::Off) opt.gravity = {0, 0, 0};
    if (run->engine == SimEngine::Vehicle) {
        const Vec3 w = scene.physics().wind;
        opt.dt = vehicleStep(vehicleModel(run->rig, {w.x, w.y, w.z}), opt.dt);
    }
    Multibody mb(run->rig, opt);
    // Start in the programmed pose, the base set down so nothing is below the ground.
    MotionTargets t0 = motionTargets(scene, run->rig, 0.0f);
    std::vector<double> q0 = t0.value;
    for (std::size_t j = 0; j < q0.size(); ++j)
        if (t0.velocity[j]) q0[j] = 0;
    mb.setJointState(q0, std::vector<double>(q0.size(), 0.0));
    if (run->rig.floatingBase && opt.ground.height) {
        double lowest = 1e300;
        for (std::size_t l = 0; l < run->rig.links.size(); ++l) {
            const LinkPose p = mb.linkPose(static_cast<int>(l));
            for (const auto& c : run->rig.links[l].contactPoints) {
                const V3d w = p.apply(c);
                lowest = std::min(lowest, w[2] - opt.ground.height(w[0], w[1]));
            }
        }
        if (lowest < 0 && lowest > -1e299) {
            LinkPose b = mb.basePose();
            b.t[2] -= lowest;
            mb.setBasePose(b);
        }
    }
    // Vehicles: rotor thrust, air and wind; a mission flies or drives them.
    std::vector<char> controlled(run->rig.joints.size(), 0);
    if (run->engine == SimEngine::Vehicle) {
        const Vec3 w = scene.physics().wind;
        const VehicleModel vm = vehicleModel(run->rig, {w.x, w.y, w.z});
        addVehicleForces(mb, vm);
        if (!settings.mission.empty()) {
            auto mission = settings.mission;
            std::stable_sort(mission.begin(), mission.end(), [](const MissionStep& a, const MissionStep& b) { return a.t < b.t; });
            constexpr double kDegRad = 3.14159265358979323846 / 180.0;
            if (vm.rotors.size() >= 3) {
                auto fc = std::make_shared<FlightController>(run->rig, vm);
                for (std::size_t j : fc->joints()) controlled[j] = 1;
                auto current = std::make_shared<int>(-1);
                mb.addForceHook([fc, mission, current](Multibody& m) {
                    int k = -1;
                    for (std::size_t i = 0; i < mission.size(); ++i)
                        if (mission[i].t <= m.time() + 1e-9) k = static_cast<int>(i);
                    if (k != *current) {
                        *current = k;
                        if (k < 0) fc->hover();
                        else {
                            const MissionStep& s = mission[static_cast<std::size_t>(k)];
                            if (s.kind == MissionKind::Velocity) fc->velocity({s.vec.x, s.vec.y, s.vec.z}, s.rate * kDegRad);
                            else if (s.kind == MissionKind::Goto) fc->moveTo({s.vec.x * 1e-3, s.vec.y * 1e-3, s.vec.z * 1e-3}, s.rate * kDegRad);
                            else fc->hover();
                        }
                    }
                    fc->update(m);
                });
            } else if (!vm.wheels.empty()) {
                auto dc = std::make_shared<DriveController>(vm);
                for (std::size_t j : dc->joints()) controlled[j] = 1;
                mb.addForceHook([dc, mission](Multibody& m) {
                    int k = -1;
                    for (std::size_t i = 0; i < mission.size(); ++i)
                        if (mission[i].t <= m.time() + 1e-9) k = static_cast<int>(i);
                    if (k < 0 || mission[static_cast<std::size_t>(k)].kind != MissionKind::Drive) dc->drive(0, 0);
                    else dc->drive(mission[static_cast<std::size_t>(k)].vec.x, mission[static_cast<std::size_t>(k)].rate * kDegRad);
                    dc->update(m);
                });
            } else {
                run->note = "A mission needs three or more ROTOR parts (a multirotor) or WHEEL parts (a rover)";
            }
        }
    }
    // The sandbox's forces: wrenches, buoyancy, magnets, scripts.
    for (const std::string& n : addRobotForces(mb, scene, settings)) run->note += (run->note.empty() ? "" : "; ") + n;
    const double dt = opt.dt;
    const long steps = std::lround(duration / dt);
    const long perFrame = std::max(1L, std::lround(run->frameStep / dt));
    const long perTarget = 4;
    auto record = [&]() {
        RobotRunFrame f;
        f.t = static_cast<float>(mb.time());
        for (std::size_t l = 0; l < run->rig.links.size(); ++l) f.links.push_back(mb.linkPose(static_cast<int>(l)));
        for (double v : mb.q()) f.q.push_back(static_cast<float>(v));
        f.com = mb.centreOfMass();
        for (const ContactPoint& c : mb.contacts()) f.contactForce += c.force[2];
        for (std::size_t j = 0; j < run->rig.joints.size(); ++j) {
            const auto w = mb.jointWrench(j);
            f.wrench.push_back({w.first[0], w.first[1], w.first[2], w.second[0], w.second[1], w.second[2]});
        }
        run->frames.push_back(std::move(f));
    };
    record();
    for (long s = 0; s < steps; ++s) {
        if (s % perTarget == 0) {
            const MotionTargets tg = motionTargets(scene, run->rig, static_cast<float>(mb.time()));
            for (std::size_t j = 0; j < tg.value.size(); ++j) {
                if (controlled[j]) continue; // the flight or drive controller has it
                JointControl c;
                c.mode = tg.velocity[j] ? ActuatorMode::Velocity : ActuatorMode::Position;
                c.target = tg.velocity[j] ? tg.rate[j] : tg.value[j];
                mb.setControl(j, c);
            }
        }
        mb.step();
        if (mb.diverged()) {
            run->diverged = true;
            run->note = "The simulation became unstable at " + std::to_string(mb.time()) + " s and stopped there";
            break;
        }
        if ((s + 1) % perFrame == 0) record();
    }
    std::lock_guard<std::mutex> g(lock);
    cache.insert(cache.begin(), {key, run});
    if (cache.size() > 6) cache.pop_back();
    return run;
}

std::vector<std::shared_ptr<const RobotRun>> robotRuns(const Scene& scene) {
    std::vector<std::shared_ptr<const RobotRun>> out;
    const float duration = scene.physics().duration > 0 ? scene.physics().duration : 10.0f;
    for (const RigSettings& s : scene.rigs().robots)
        if (s.simulate)
            if (auto r = robotRun(scene, s, duration)) out.push_back(std::move(r));
    return out;
}

// Per-body transforms of the simulated robots at time t (bodies not in one keep `tf`).
void applyRobotRuns(const Scene& scene, float t, std::vector<Mat4>& tf) {
    for (const auto& run : robotRuns(scene)) {
        const RobotRunFrame f = run->at(t);
        for (std::size_t l = 0; l < run->rig.links.size() && l < f.links.size(); ++l) {
            const Mat4 m = poseMatrix(f.links[l]);
            for (std::size_t b : run->rig.links[l].bodies)
                if (b < tf.size()) tf[b] = m;
        }
    }
}

} // namespace sicdm

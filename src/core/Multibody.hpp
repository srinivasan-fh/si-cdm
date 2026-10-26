#pragma once

// Robot simulation engines, picked per robot (RIG ENGINE) or by its morphology.
//
//   articulated  Joint-space (generalized coordinate) dynamics of the robot's tree: the
//                articulated-body algorithm, a floating base for anything that walks or
//                flies, soft contacts with stick-slip friction, joint limits, and servos
//                integrated implicitly so stiff position control stays stable. The default
//                for manipulators, humanoids, quadrupeds and custom robots.
//   vehicle      The articulated engine with the vehicle models switched on: rotor thrust
//                and drag torque, wind and air drag, tyre grip on the ground. The default
//                for drones and rovers. Batches of copies run in parallel (see RobotEnv).
//   optimize     Trajectory optimisation on the articulated model: the joint motion that
//                reaches a goal with the least effort, then simulated (manipulators).
//   particle     The scene's physics engine (rigid bodies as spheres, PHYSICS).
//
// Units are SI (m, kg, s, rad, N, N m); the rig's rest pose is every joint at 0.

#include "core/Rig.hpp"

#include <array>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace sicdm {

enum class SimEngine { Articulated, Vehicle, Optimize, Particle };
struct SimEngineInfo {
    SimEngine engine;
    const char* key;
    const char* title;
    const char* summary;
};
const std::vector<SimEngineInfo>& simEngines();
std::optional<SimEngine> simEngineFromKey(const std::string& key);
const char* simEngineKey(SimEngine e);
SimEngine defaultEngine(Morphology m);
// The engine a robot runs on: its own setting, else its morphology's default.
SimEngine engineFor(const RobotRig& rig, const RigSettings& settings);

using V3d = std::array<double, 3>;

// The ground under the robot: height (m) and the upward normal at a point. Flat z = 0 by
// default; terrain replaces it (Terrain.hpp).
struct Ground {
    std::function<double(double x, double y)> height;           // null: no ground
    std::function<V3d(double x, double y)> normal;              // null: (0, 0, 1)
    double friction = 0.9;                                       // Coulomb coefficient
    static Ground flat(double z = 0.0, double friction = 0.9);
};

enum class ActuatorMode { Position, Velocity, Torque, Passive };
const char* actuatorModeKey(ActuatorMode m);

struct JointControl {
    ActuatorMode mode = ActuatorMode::Position;
    double target = 0;   // rad / m (Position) or rad/s / m/s (Velocity)
    double torque = 0;   // N m / N (Torque; added on top of the servo in the other modes)
};

struct MultibodyOptions {
    double dt = 0.0005;                  // s per step
    V3d gravity{0, 0, -9.80665};
    bool contacts = true;
    Ground ground = Ground::flat();
    double stiffness = 0;                // N/m per contact point; 0: from the robot's weight
    double servoStiffness = 1.0;         // scales the position servos' gains
    double jointDamping = 0.02;          // N m s/rad of friction in every joint (plus the servo's)
    double integral = -1;                // position servos' integral gain / kp (1/s); < 0: 4 for a fixed base, 0 floating
};

// A link's pose: world = R * rest + t (rest: where the link is in the scene, joints at 0).
struct LinkPose {
    std::array<double, 9> R{1, 0, 0, 0, 1, 0, 0, 0, 1};
    V3d t{0, 0, 0};
    V3d apply(const V3d& p) const;
};

struct ContactPoint {
    int link = 0;
    V3d point{};   // world, m
    V3d force{};   // on the link, N
    bool sliding = false;
};

// One robot's dynamics, stepped in fixed steps. Joint i is the rig's joint i.
class Multibody {
public:
    explicit Multibody(const RobotRig& rig, MultibodyOptions options = {});
    ~Multibody();
    Multibody(const Multibody&);
    Multibody& operator=(const Multibody&);

    const RobotRig& rig() const;
    const MultibodyOptions& options() const;
    MultibodyOptions& options();
    std::size_t jointCount() const;
    bool floatingBase() const;

    // Back to the rest pose, still, at time 0.
    void reset();
    void step();                 // one options().dt
    void advance(double seconds);// whole steps
    double time() const;

    // Controls (held until changed). Coupled joints (mimics) follow their leader whatever
    // their own control says.
    void setControl(std::size_t joint, const JointControl& c);
    const JointControl& control(std::size_t joint) const;
    // Every joint to hold its rest angle (the default).
    void holdRest();

    // A force (N) at a world point, and a torque (N m), on a link, for the next step only
    // (persistent = true: every step until clearWrenches()).
    void applyWrench(int link, const V3d& force, const V3d& torque, const V3d& point, bool persistent = false);
    void clearWrenches();
    // Called before every step to add forces (buoyancy, magnets, scripted thrust...).
    using ForceHook = std::function<void(Multibody&)>;
    void addForceHook(ForceHook hook);

    // State.
    std::vector<double> q() const;   // joint values (rad / m)
    std::vector<double> qd() const;  // joint rates
    std::vector<double> torques() const; // actuator torques of the last step
    void setJointState(const std::vector<double>& q, const std::vector<double>& qd);
    LinkPose linkPose(int link) const;
    // Spatial velocity of a link: angular (rad/s) and the linear velocity (m/s) of its
    // centre of mass; and its classical acceleration (of the last step).
    V3d linkAngularVelocity(int link) const;
    V3d linkVelocity(int link) const;
    V3d linkAcceleration(int link) const;
    V3d linkCom(int link) const;     // world, m
    LinkPose basePose() const;
    void setBasePose(const LinkPose& p);
    void setBaseVelocity(const V3d& angular, const V3d& linear);
    V3d centreOfMass() const;
    double kineticEnergy() const;
    double potentialEnergy() const;
    const std::vector<ContactPoint>& contacts() const; // touching points of the last step
    // Force and torque the joint carries from the parent onto the child (N, N m), last step.
    std::pair<V3d, V3d> jointWrench(std::size_t joint) const;
    bool diverged() const;           // NaN or runaway speeds: the run stopped being physical

private:
    struct Impl;
    std::unique_ptr<Impl> m;
};

// Joint targets a scene's motions ask for at time t: each joint's motion value (relative to
// rest, rad / m) and rate; continuous joints run at their motion's speed.
struct MotionTargets {
    std::vector<double> value, rate;
    std::vector<char> velocity; // 1: the joint is speed controlled (continuous)
};
MotionTargets motionTargets(const class Scene& scene, const RobotRig& rig, float t);

// A whole run of a robot over [0, duration], recorded for playback (SIM): the scene's
// motions are the servos' targets, so the robot does what it was programmed to, under
// gravity, with its contacts and its motors' limits.
struct RobotRunFrame {
    float t = 0;
    std::vector<LinkPose> links;
    std::vector<float> q;
    V3d com{};
    double contactForce = 0; // total normal force on the ground (N)
    std::vector<std::array<double, 6>> wrench; // each joint's force (N) and torque (N m) on its child, at its axis point
};
struct RobotRun {
    std::string robot;
    SimEngine engine = SimEngine::Articulated;
    RobotRig rig;
    std::vector<RobotRunFrame> frames;
    float frameStep = 0.01f;
    bool diverged = false;
    std::string note;
    RobotRunFrame at(float t) const;
};

// The longest robot run (s), as for the physics.
constexpr float kMaxRobotRun = 600.0f;

// The runs of the robots set to SIMULATE (computed once, then cached).
std::vector<std::shared_ptr<const RobotRun>> robotRuns(const Scene& scene);
std::shared_ptr<const RobotRun> robotRun(const Scene& scene, const RigSettings& settings, float duration);

// The scene's ground for robots: the floor at z = 0, or the terrain.
Ground sceneGround(const Scene& scene);

// Places the simulated robots' parts at time t in `tf` (per-body transforms, as
// simulationTransforms() makes them); other bodies keep theirs.
void applyRobotRuns(const Scene& scene, float t, std::vector<Mat4>& tf);

} // namespace sicdm

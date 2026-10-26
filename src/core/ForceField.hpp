#pragma once

// The robot sandbox: forces of any kind on any part, on top of gravity, the motors and the
// contacts (RIG FORCE; the kinds are in Rig.hpp: wrench, buoyancy, magnetic adhesion,
// scripted). They act in every engine run of the robot: the Simulation's playback, the
// sensors, and the learning environments.
//
// Scripts are written two ways:
//   expressions   in the project: FX "..." FY "..." ... of the variables below, with
//                 + - * / ^, parentheses, and sin cos tan asin acos atan atan2 exp log sqrt
//                 abs sign min max pow clamp step (0 below 0, else 1), pi
//   C++           a function registered under a name (registerForceScript), named by the
//                 force's FUNCTION; from Python, wrenches are sent with each learning step
//                 (robot_env step "wrenches", python/sicdm_env.py)
//
// Variables: t (s); x, y, z (m) and vx, vy, vz (m/s) of the force's point; wx, wy, wz
// (rad/s) of the part; mass (kg) of the part's link; g (m/s^2).

#include "core/Multibody.hpp"

#include <functional>
#include <string>
#include <vector>

namespace sicdm {

class Scene;
struct RigSettings;

// A compiled expression.
class ForceExpression {
public:
    static const std::vector<std::string>& variables(); // the order eval() takes them in
    bool compile(const std::string& text, std::string* error = nullptr);
    double eval(const double* vars) const;
    bool empty() const { return m_ops.empty(); }

private:
    struct Op {
        int code = 0;
        double value = 0;
    };
    std::vector<Op> m_ops; // reverse Polish
};

struct ForceContext {
    double t = 0;
    int link = 0;
    V3d point{}, velocity{}, angular{}; // world, m, m/s, rad/s
    double mass = 0;
    V3d gravity{0, 0, -9.80665};
};
// Writes the force (N) and torque (N m) to apply at ctx.point, in the world's axes.
using ForceFunction = std::function<void(const ForceContext& ctx, V3d& force, V3d& torque)>;
void registerForceScript(const std::string& name, ForceFunction fn);
void unregisterForceScript(const std::string& name);
bool hasForceScript(const std::string& name);

// Adds the robot's forces to `mb` (as force hooks). Returns notes on forces it couldn't add.
std::vector<std::string> addRobotForces(Multibody& mb, const Scene& scene, const RigSettings& settings);

// One line describing a force.
std::string describeForce(const struct ForceSpec& f);

} // namespace sicdm

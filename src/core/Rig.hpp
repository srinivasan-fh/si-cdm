#pragma once

// Robot rigs: the scene's parts and motions read as a robot for simulation and export.
//
// A robot is made the way SI CDM already moves parts: a joint is a motion (circular: a
// revolute joint about its axis through its origin, with its limits; linear: a prismatic
// one), and it moves its child part and everything below it. So the tree comes from the
// motions themselves: a joint's parent is the joint whose bodies are the smallest set
// holding all of its own (three motions on the same bodies, like a ball shoulder, are a
// chain of three revolute joints). The bodies a joint is the innermost one to move make
// its link; bodies that move with no joint make the base. Bodies attached to another
// (Body::attachedTo) ride with it. Drive motors (motions other joints follow) stay with
// the link they sit in; a joint that follows one keeps the coupling (its ratio).
//
// Each link gets its mass, centre of mass and inertia tensor from its meshes (the bodies'
// physics mass or density). Auto-rigging tags say what a part is for and set the joint it
// needs: a ROTOR or a WHEEL spins freely (a continuous joint about its own axis, made if
// the part has none), a KNEE or HINGE is a limited hinge, an END_EFFECTOR is the frame
// arms are steered by, a FOOT is a contact the legs stand on. The morphology (manipulator,
// humanoid, quadruped, drone, rover, custom) follows from the parts and tags unless set,
// and picks the base (fixed for an arm, floating for anything that walks, rolls or flies)
// and the simulation engine.
//
// All lengths in the rig are metres and angles radians (the units robot formats and
// solvers use); the scene stays in mm.

#include "core/Math.hpp"

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace sicdm {

class Scene;
class Json;

enum class Morphology { Auto, Manipulator, Humanoid, Quadruped, Drone, Rover, Custom };
const char* morphologyKey(Morphology m) noexcept;
std::optional<Morphology> morphologyFromKey(const std::string& key) noexcept;

enum class RigTag { None, Rotor, Wheel, EndEffector, Foot, Knee, Hinge, Base };
const char* rigTagKey(RigTag t) noexcept;
std::optional<RigTag> rigTagFromKey(const std::string& key) noexcept;

// A robot's settings, saved with the project (the tree itself is read from the scene).
struct RigSettings {
    std::string name;                  // the robot: a component, an assembly path, or "" (every jointed part)
    Morphology morphology = Morphology::Auto;
    std::optional<bool> floatingBase;  // unset: from the morphology
    std::vector<std::pair<std::string, RigTag>> tags; // body name -> tag
    float density = 0.0f;              // kg/m^3 for parts without their own; 0: each body's
    std::string engine;                // "" (from the morphology) or an engine key, see Multibody.hpp

    RigTag tagOf(const std::string& body) const;
    void setTag(const std::string& body, RigTag tag); // None removes it
};

struct RigState {
    std::vector<RigSettings> robots;
    bool empty() const { return robots.empty(); }
    RigSettings* find(const std::string& name);
    const RigSettings* find(const std::string& name) const;
    RigSettings& get(const std::string& name); // makes one if missing
};
Json rigStateToJson(const RigState& s);
void rigStateFromJson(const Json& j, RigState& s);

// 3x3 matrix, row major (inertia tensors, rotations).
using Mat3d = std::array<double, 9>;

struct RigLink {
    std::string name;
    std::vector<std::size_t> bodies; // scene indices
    double mass = 0;                 // kg
    std::array<double, 3> com{};     // centre of mass, world frame at rest (m)
    Mat3d inertia{};                 // about the centre of mass, world axes at rest (kg m^2)
    int parentJoint = -1;            // -1: the base
    RigTag tag = RigTag::None;
    std::array<double, 3> boxMin{}, boxMax{}; // its parts' bounds at rest (m)
};

enum class RigJointType { Revolute, Continuous, Prismatic };
const char* rigJointTypeKey(RigJointType t) noexcept;

struct RigJoint {
    std::string name;               // the motion's name
    RigJointType type = RigJointType::Revolute;
    int parent = 0, child = 0;      // links
    std::array<double, 3> origin{}; // a point on the axis, world frame at rest (m)
    std::array<double, 3> axis{0, 0, 1};
    double lo = 0, hi = 0;          // rad or m (Continuous: none)
    double effort = 0, velocity = 0;// N m (N) and rad/s (m/s) the drive can give; 0: unknown
    std::string mimic;              // a coupled joint: the joint it follows...
    double mimicRatio = 1;          // ...as `ratio` times its value
    std::string drive;              // the drive motor motion it follows ("" none)
    int motion = -1;                // scene motion index
    bool made = false;              // made by a tag (the scene has no motion for it)
};

struct RobotRig {
    std::string name;
    Morphology morphology = Morphology::Custom;
    bool floatingBase = false;
    std::vector<RigLink> links;    // links[0] is the base
    std::vector<RigJoint> joints;  // parents before children
    std::vector<std::string> notes;

    int linkOf(std::size_t sceneBody) const;
    int jointNamed(const std::string& name) const;
    int linkNamed(const std::string& name) const;
    double totalMass() const;
    // Links tagged END_EFFECTOR (or, with none, the tips of the longest chain).
    std::vector<int> endEffectors() const;
};

// Mass, centre of mass and inertia tensor about it of a closed mesh of uniform density
// (kg/m^3), in metres. Open meshes fall back to their bounding box.
struct MassInertia {
    double mass = 0, volume = 0;
    std::array<double, 3> com{};
    Mat3d inertia{};
};
MassInertia meshInertia(const struct Body& body, double density);

// The robots the scene has: components and assembly paths with joints, or "" when jointed
// parts belong to neither.
std::vector<std::string> rigCandidates(const Scene& scene);

// Reads a robot from the scene. Missing `settings` take the defaults. False (and why)
// when it has no parts.
bool buildRig(const Scene& scene, const RigSettings& settings, RobotRig& out, std::string* error = nullptr);

// Gives the tagged parts the joints their tags call for: a ROTOR or WHEEL without its own
// motion gets a continuous spin about its axis (the thinnest direction of its bounds,
// through its centre), a KNEE or HINGE without limits gets 0..140 / -90..90 deg.
// Returns what it changed, one line each.
std::vector<std::string> applyRigTags(Scene& scene, const RigSettings& settings);

// One line per link and joint, for the RIG command.
std::vector<std::string> describeRig(const RobotRig& rig);

} // namespace sicdm

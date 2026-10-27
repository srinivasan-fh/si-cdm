#pragma once

// Arm kinematics and motion planning on a robot rig (ARM):
//
//   forward kinematics   every link's pose for a set of joint values
//   inverse kinematics   joint values that put a tool point on a target (and, if asked, its
//                        approach axis along a direction): damped least squares within the
//                        joint limits, restarted from spread-out guesses until a solution
//                        that collides with nothing is found; coupled joints move with
//                        their leader
//   collisions           each link's parts as boxes (turning with it) against
//                        the other links and the scene around the robot (other parts, the
//                        floor and the terrain); pairs that already touch at rest (at
//                        joints) are left out
//   planning             RRT-Connect in the arm's joint space, collision checked every 1 deg
//                        along each edge, then shortcut: a collision-free path from where the
//                        arm is to the IK solution
//   optimisation         the least-effort motion through the same waypoints in the same time:
//                        each segment bent by smooth shapes to lower the integral of the
//                        squared joint torques (inverse dynamics), within the joints' ranges
//                        and speeds and clear of collisions
//
// The base stays where it is at rest (a floating base is held for the arm's sake).

#include "core/Collide.hpp"
#include "core/Multibody.hpp"
#include "core/Terrain.hpp"

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace sicdm {

class Scene;

class RobotKinematics {
public:
    explicit RobotKinematics(const RobotRig& rig);
    const RobotRig& rig() const { return m_rig; }
    // Link poses for joint values q (one per rig joint; coupled joints follow their leader).
    std::vector<LinkPose> forward(const std::vector<double>& q) const;
    // q with every coupled joint set from its leader.
    std::vector<double> withMimics(std::vector<double> q) const;
    // The joints from the base out to `link`, base first (coupled ones left out).
    std::vector<std::size_t> chain(int link) const;
    double lower(std::size_t joint) const;
    double upper(std::size_t joint) const;

private:
    RobotRig m_rig;
    std::vector<int> m_leader; // per joint: the joint it follows (-1: none)
};

class CollisionModel {
public:
    // The scene around the robot at time t (its other parts where they are then).
    CollisionModel(const Scene& scene, const RobotRig& rig, float t = 0.0f);
    bool collides(const std::vector<LinkPose>& poses, std::string* what = nullptr) const;
    // For a move of these joints only: pairs of links no moving joint separates (fingers on a
    // hand the arm carries) keep where they are to each other, so they are left out, and so
    // are the links that don't move at all.
    void onlyMoving(const RobotKinematics& kin, const std::vector<std::size_t>& active);
    // Leaves out whatever touches in this pose (where the arm starts: the boxes are a little
    // bigger than the parts, so curled fingers may overlap). Returns how many pairs.
    int allowTouching(const std::vector<LinkPose>& poses);
    std::size_t obstacles() const { return m_env.size(); }

private:
    // Each part's box finds what may touch; its exact shape (Collide.hpp) then says
    // whether it does.
    struct Box {
        V3d centre{}, half{};      // at rest, world axes (m)
        std::shared_ptr<const CollisionShape> shape; // at rest, world (mm)
    };
    struct Obstacle {
        V3d lo{}, hi{};            // world, m
        std::string name;
        std::shared_ptr<const CollisionShape> shape;
        Rigid at;                  // where it is (mm)
    };
    std::vector<std::string> m_names;          // per link
    std::vector<std::vector<Box>> m_links;     // per link: its parts' boxes
    std::vector<Obstacle> m_env;
    std::vector<std::vector<char>> m_skip;     // link pairs to leave out
    std::vector<std::vector<char>> m_skipEnv;  // per link, per obstacle
    std::vector<char> m_skipFloor;             // per link
    std::shared_ptr<const TerrainSettings> m_terrain; // null: the floor at z = 0
};

struct IkTarget {
    int link = -1;            // the tool link (-1: the rig's end effector)
    V3d point{};              // the tool point on it, where it is at rest (m)
    V3d target{};             // where it should go (m)
    std::optional<V3d> approach; // the link's approach axis (at rest) to point along...
    V3d approachAt{0, 0, 1};     // ...this axis at rest
    double tolerance = 0.001; // m
};

struct IkResult {
    bool ok = false;
    std::vector<double> q;
    double error = 0;         // m left
    bool collisionFree = true;
    std::string why;
};

IkResult solveIk(const RobotKinematics& kin, const std::vector<double>& start, const IkTarget& target, const CollisionModel* collisions = nullptr,
                 int restarts = 30);

struct PlanResult {
    bool ok = false;
    std::vector<std::vector<double>> path; // joint values, start to goal
    std::string why;
    int iterations = 0;
};

// A collision-free path between two joint states, moving only `active` joints.
PlanResult planPath(const RobotKinematics& kin, const CollisionModel& collisions, const std::vector<double>& start, const std::vector<double>& goal,
                    const std::vector<std::size_t>& active, int maxIterations = 6000, unsigned seed = 1);

// Joint torques for joint values, rates and accelerations, the base fixed (recursive
// Newton-Euler, gravity included).
std::vector<double> inverseDynamics(const RobotKinematics& kin, const std::vector<double>& q, const std::vector<double>& qd, const std::vector<double>& qdd,
                                    const V3d& gravity = {0, 0, -9.80665});

struct TimedPath {
    std::vector<double> times;              // s, from 0
    std::vector<std::vector<double>> q;     // joint values at those times
    // false: minimum-jerk moves that stop at each waypoint; true: a spline through them
    // (finely sampled curves, as keys with SPLINE on)
    bool smooth = false;
    double effort = 0;                      // integral of the squared torques (N^2 m^2 s)
};

// Times a path for the joints' speeds (each segment's slowest joint at `speedScale` x its
// top speed on a minimum-jerk profile).
TimedPath timePath(const RobotKinematics& kin, const std::vector<std::vector<double>>& path, double speedScale = 0.5);
// The least-effort motion between the same waypoints in the same times: each segment bent by
// smooth shapes (the waypoints and the stops at them stay) to lower the integral of the squared
// motor torques, within the joints' ranges and speeds and clear of collisions. Returns a smooth
// path, or the initial one when no bend pays.
TimedPath optimisePath(const RobotKinematics& kin, const CollisionModel& collisions, const TimedPath& initial, const std::vector<std::size_t>& active,
                       int iterations = 60);
// Effort of a timed path (sampled at 100 Hz, moving as its keys would).
double pathEffort(const RobotKinematics& kin, const TimedPath& path);

// Writes a timed path into the scene's motions from time t0: keyframes on each joint's
// motion, or on its drive motor's for driven joints (the drive turns ratio x slower).
// Earlier keys stay; later ones go. Returns the motions changed.
std::vector<std::string> writeJointPath(Scene& scene, const RobotRig& rig, const TimedPath& path, float t0, const std::vector<std::size_t>& active);

} // namespace sicdm

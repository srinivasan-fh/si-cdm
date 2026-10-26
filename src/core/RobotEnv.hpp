#pragma once

// Learning environments: a batch of copies of a robot, stepped together in parallel threads,
// headless and as fast as the machine runs (no drawing, no real-time pacing), for training
// controllers overnight. A policy outside the app (Python with PyTorch or JAX, through the
// sicdm-mcp server and python/sicdm_env.py) reads the observations, sends actions, and gets
// rewards back; finished episodes start again on their own.
//
//   actions       one per motor (each joint that isn't coupled to another), in [-1, 1]:
//                 POSITION  a target across the joint's range around where it starts
//                           (spinning joints, wheels and rotors: a speed up to their top speed)
//                 TORQUE    a share of the joint's effort
//   observations  per copy: the motors' joint values and rates; the base's height, the way
//                 up in its frame, and its velocity and turn rate in its frame; then what
//                 the task adds (the tool's offset from its target, the drift from home)
//   tasks         STAND (stay up at the starting height), WALK (go along +X), REACH x,y,z
//                 (the end effector to a point), HOVER (a drone holds where it starts),
//                 NONE (no reward: the policy's own)
//
// A copy's episode ends when the robot falls (a floating base tipped past 60 degrees or its
// centre of mass down to 40% of the starting height), the simulation fails, or the episode
// time is up.

#include "core/Multibody.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace sicdm {

class Scene;
struct RigSettings;

enum class EnvTask { None, Stand, Walk, Reach, Hover };
const char* envTaskKey(EnvTask t) noexcept;
bool envTaskFromKey(const std::string& key, EnvTask& out) noexcept;

struct EnvOptions {
    int count = 1;                 // copies
    EnvTask task = EnvTask::Stand;
    V3d target{0, 0, 0};           // Reach: where the tool should go (m)
    bool torque = false;           // actions are torques, not position / speed targets
    double controlDt = 0.02;       // s between actions (the engine steps 0.5 ms in between)
    double episodeSeconds = 10;
    double noise = 0.02;           // starting joint values spread by up to this (rad / m)
    std::uint64_t seed = 1;
    int threads = 0;               // 0: as many as the machine has (at most one per copy)
};

struct EnvStep {
    std::vector<double> observations; // count x observationSize, row by row
    std::vector<double> rewards;
    std::vector<char> terminated;     // fell or failed
    std::vector<char> truncated;      // time up
    struct Episode {
        int env = 0;
        double ret = 0;               // its rewards summed
        int length = 0;               // in actions
    };
    std::vector<Episode> episodes;    // episodes that ended (those copies have started again)
};

class RobotEnv {
public:
    // False (and why) when the robot can't be built or has no motors.
    static std::unique_ptr<RobotEnv> create(const Scene& scene, const RigSettings& robot, const EnvOptions& options, std::string* error = nullptr);
    ~RobotEnv();

    int count() const;
    std::size_t observationSize() const;
    std::size_t actionSize() const;
    const std::vector<std::string>& observationNames() const;
    const std::vector<std::string>& actionNames() const;
    const RobotRig& rig() const;
    const EnvOptions& options() const;
    double simulatedSeconds() const; // over all copies, since created

    // Every copy back to the start (spread by the noise), with a new seed if given.
    std::vector<double> reset(std::uint64_t seed = 0);
    // Actions for every copy (count x actionSize, row by row; missing ones are 0).
    EnvStep step(const std::vector<double>& actions);

private:
    RobotEnv();
    struct Impl;
    std::unique_ptr<Impl> m;
};

// Environments kept for clients (MCP), by id.
int keepEnv(std::unique_ptr<RobotEnv> env);
RobotEnv* findEnv(int id);
bool closeEnv(int id);
std::vector<int> envIds();

} // namespace sicdm

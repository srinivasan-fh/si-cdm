#pragma once

#include <string>
#include <vector>

namespace sicdm {

struct CommandOutput;
struct Document;
struct RigSettings;

// RIG (robots read from the scene: links, joints, tags, morphology) and the robot
// simulation commands (see their entries in the command table). `args` follow the
// command word.
CommandOutput runRobotCommand(Document& doc, const std::string& cmd, const std::vector<std::string>& args, const std::vector<bool>& quoted);

// The robot a command names: args[i] when it is one (quoted, a component or assembly with
// joints), else the only one the scene has. Advances i past the name. False (and why)
// when there is none or the choice is ambiguous.
bool pickRobot(const Document& doc, const std::vector<std::string>& args, const std::vector<bool>& quoted, std::size_t& i, std::string& robot,
               std::string* error);

} // namespace sicdm

#pragma once

#include <string>
#include <vector>

namespace sicdm {

struct CommandOutput;
struct Document;

// ARMATURE: sketch a form as a tree of balls (Armature.hpp), skin it and pose it. `args`
// follow the word ARMATURE.
//   ARMATURE                                   the armatures and their skins
//   ARMATURE NEW [name] [AT x,y,z] [RADIUS r]
//   ARMATURE name ADD [TO n] AT x,y,z [RADIUS r] [MIRROR X|Y|Z]   a ball joined to ball n (the last added)
//   ARMATURE name INSERT n                      a ball halfway along the link to ball n
//   ARMATURE name MOVE n TO x,y,z | BY dx,dy,dz [CHAIN] [MIRROR X|Y|Z]
//   ARMATURE name RADIUS n r [MIRROR X|Y|Z]
//   ARMATURE name REMOVE n                      the ball and what hangs from it
//   ARMATURE name LIST
//   ARMATURE name SKIN [NAME skin] [VOXEL v] [BLEND b] [SMOOTH k]
//   ARMATURE name POSE n TURN ax,ay,az | AXIS x,y,z ANGLE a | RESET [MIRROR X|Y|Z]  (POSE RESET: every ball)
CommandOutput runArmatureCommand(Document& doc, const std::vector<std::string>& args, const std::vector<bool>& quoted);

} // namespace sicdm

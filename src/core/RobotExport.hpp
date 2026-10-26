#pragma once

// Robot description files for other simulators and robot software, written from a rig:
//
//   URDF  (.urdf)  ROS, MoveIt, Isaac Sim, PyBullet, Pinocchio
//   SDF   (.sdf)   Gazebo, Drake
//   MJCF  (.xml, .mjcf)  MuJoCo
//
// Every link's parts go into one binary STL (metres, in the link's frame) in a folder
// beside the file, named after it ("arm.urdf" -> "arm_meshes/"), and the file refers to
// them relatively, as visual and collision geometry. Each link's frame sits on its joint's
// axis point with the world's axes (the scene's rest pose is every joint at 0), so joint
// origins are the offsets between them and axes are the scene's. Inertials (mass, centre
// of mass, full tensor) are the rig's. Couplings (one drive moving several joints) are
// URDF <mimic>, MJCF <equality joint>; SDF has no coupling and says so in a comment.
// A fixed base is fixed to the world; a floating one is left free (MJCF: <freejoint>).
// MJCF also gets actuators: position servos for limited joints, velocity motors for
// wheels and rotors.

#include <string>
#include <vector>

namespace sicdm {

class Scene;
struct RobotRig;

enum class RobotFormat { Urdf, Sdf, Mjcf };
// By the file's extension (.urdf, .sdf, .xml / .mjcf); false for others.
bool robotFormatOf(const std::string& path, RobotFormat& out);
const char* robotFormatName(RobotFormat f);

struct RobotExportResult {
    bool ok = false;
    std::string error;
    std::vector<std::string> files; // the robot file, then the meshes
};

// Text of the robot file alone (meshes named as exportRobot writes them under `meshDir`).
std::string robotDescription(const Scene& scene, const RobotRig& rig, RobotFormat format, const std::string& meshDir);
// Writes the robot file and its meshes.
RobotExportResult exportRobot(const Scene& scene, const RobotRig& rig, const std::string& path);

// An XML-safe name: letters, digits and underscores ("Robot Arm 1 Elbow" -> "Robot_Arm_1_Elbow").
std::string robotIdentifier(const std::string& name);

} // namespace sicdm

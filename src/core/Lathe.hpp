#pragma once

// Lathe CAM: turning a part from bar stock on a 2-axis lathe (X across, Z along the
// spindle), checked in a simulation of the turned profile and posted as G-code.
//
//   Setup       the part's turning axis (the model's X, Y or Z through the part's
//               centre), the end the lathe faces, the bar's diameter and what sticks
//               out in front of the part
//   Profile     the part's outline as the lathe sees it: at each station along the axis
//               the farthest the part reaches from the axis (what can't be turned off),
//               and the nearest its material comes to it (the bore, 0 where it is solid)
//   Tools       turning inserts (nose radius, insert angle: how steeply the tool can go
//               down behind a shoulder), grooving / parting blades, threading inserts,
//               drills and boring bars
//   Operations  facing; roughing and finishing outside (OD) and in the bore (ID), with
//               stock to leave; grooving the recesses a turning insert can't reach;
//               drilling on the axis; single-point threading; parting off
//   Simulation  the stock's outer and inner radius along the axis, cut by each tool's
//               shape: removed volume, stock left, gouges, rapid moves through stock and
//               cycle time at constant surface speed
//   Post        Fanuc, Haas and LinuxCNC lathe programs: diameters for X, constant
//               surface speed (G96 with a spindle limit), feed per turn, threading passes
//
// Lathe coordinates: Z along the axis, 0 at the part's front face and negative towards
// the chuck; X the radius (programs show diameters). Toolpaths are kept as the tool's
// reference: the nose's centre for turning and boring, the middle of a blade's edge for
// grooving and threading, the tip for drills. Posts move them to the programmed point.

#include "core/Math.hpp"

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace sicdm {

struct Body;
class Json;

enum class LatheToolType { Turn, Groove, Thread, Drill, Bore };
const char* latheToolTypeKey(LatheToolType t); // "turn", "groove", ...
bool latheToolTypeFromKey(const std::string& key, LatheToolType* out);

struct LatheTool {
    int number = 1;                 // T number (the post calls Tnnnn with the same offset)
    LatheToolType type = LatheToolType::Turn;
    double noseRadius = 0.4;        // turning, boring and threading inserts; a blade's corners
    double insertAngle = 55.0;      // turning and boring: the insert's included angle (80 C, 55 D, 35 V)
    double width = 3.0;             // grooving / parting blade
    double diameter = 10.0;         // drill; boring bar (the smallest bore it fits)
    double tipAngle = 118.0;        // drill
    double reach = 50.0;            // drill / boring bar / blade: how deep it can go
    bool hss = false;               // high-speed steel (else carbide)
    std::string name;
    // Cutting data, used instead of the material's when set: speed (m/min), feed (mm/turn).
    double vc = 0.0, feedPerRev = 0.0;
    // Degrees the tool can go down (towards the axis, outside; away from it, in a bore)
    // while moving towards the chuck, from the insert's shape.
    double relief() const noexcept;
    std::string describe() const;   // "T1  turning insert, 0.4 mm nose, 55 deg (D)"
};

enum class LatheOpKind { Face, Rough, Finish, Groove, Drill, Thread, PartOff };
const char* latheOpKey(LatheOpKind k);

struct LatheMove {
    enum Kind : unsigned char { Rapid, Feed, Thread } kind = Feed;
    double x = 0.0, z = 0.0; // radius, axial (lathe coordinates, the tool's reference)
    double feed = 0.0;       // mm per turn (Thread: the pitch)
};

struct LatheOperation {
    LatheOpKind kind = LatheOpKind::Rough;
    std::string name;
    int tool = 1;
    bool inside = false;            // rough / finish: the bore instead of the outside
    double stepdown = 0.0;          // radial depth of cut (mm); 0: from the material
    double leave = 0.0;             // stock to leave radially (mm)
    double leaveZ = 0.0;            // stock to leave on faces (mm)
    // Z range (lathe coordinates); NaN: the part's front to its back.
    double zFrom = std::numeric_limits<double>::quiet_NaN();
    double zTo = std::numeric_limits<double>::quiet_NaN();
    double depth = 0.0;             // drill: from the front face (0: the bore's depth, or through)
    double peck = 0.0;              // drill / part off: peck depth (0: none)
    double pitch = 0.0;             // thread
    double threadDiameter = 0.0;    // thread: the major diameter (0: the part's there)
    double speed = 0.0;             // m/min (0: from the tool and material)
    double feedPerRev = 0.0;        // mm per turn (0: from the tool and material)
    double rpm = 0.0;               // constant spindle speed instead of constant surface speed
    // Output.
    std::vector<LatheMove> moves;
    bool generated = false;
    double usedSpeed = 0.0, usedFeed = 0.0, usedRpm = 0.0; // what the moves use
};

struct LatheSetup {
    std::string body;
    int axis = 0;                   // the model's axis the part turns about: 0 X, 1 Y, 2 Z
    bool flip = false;              // face the part's -axis end instead of its +axis end
    double stockDiameter = 0.0;     // 0: the part's, plus 2 mm, to a whole mm
    double frontStock = 1.0;        // bar in front of the part's front face (mm)
    std::string material = "aluminium";
    std::string post = "fanuc";
    double maxRpm = 4000.0;
    double rapid = 10000.0;         // mm/min
    double clearance = 2.0;         // rapids this far off the stock (mm)
    double tolerance = 0.01;        // mm
    double toolChange = 2.0;        // seconds
};

// The lathe frame on the model: the axis point at the front face, the axis (towards the
// front), and a radial direction for drawing.
struct LatheFrame {
    Vec3 origin, axis{1, 0, 0}, radial{0, 0, 1};
    Vec3 toModel(double x, double z) const;
};

struct LatheJob {
    LatheSetup setup;
    std::vector<LatheTool> tools;
    std::vector<LatheOperation> ops;
    bool showPaths = true;
    LatheFrame frame; // where the paths fall on the model, from the last generation (not saved)
    bool empty() const noexcept { return setup.body.empty() && tools.empty() && ops.empty(); }
    const LatheTool* tool(int number) const;
};

// The part as the lathe sees it, on stations `dz` apart from its back (z0) to its front (0).
struct LatheProfile {
    double z0 = 0.0, dz = 0.01;
    std::vector<double> outer;      // the farthest the part reaches from the axis
    std::vector<double> inner;      // the nearest its material comes to the axis (0: solid)
    std::vector<unsigned char> has; // 0 where the part has nothing at that station
    double maxRadius = 0.0;
    double length() const noexcept { return -z0; }
    std::size_t size() const noexcept { return outer.size(); }
    double zAt(std::size_t i) const noexcept { return z0 + dz * static_cast<double>(i); }
};

bool latheFrame(const LatheJob& job, const Body& part, LatheFrame& out);
bool latheProfile(const LatheJob& job, const Body& part, LatheProfile& out, std::string* error = nullptr);
double latheStockDiameter(const LatheJob& job, const LatheProfile& p);

// Fills `op.moves`. False (and `error`) when it can't.
bool generateLatheOperation(const LatheJob& job, const Body& part, LatheOperation& op, std::string* error = nullptr);

struct LatheSimulation {
    double removed = 0.0;           // mm^3
    double maxLeft = 0.0, meanLeft = 0.0; // stock left on the part's outside and bore (mm)
    std::size_t gouges = 0;         // stations cut inside the part
    double deepestGouge = 0.0;
    std::size_t rapidCrashes = 0;
    double seconds = 0.0, cutLength = 0.0, rapidLength = 0.0;
    double z0 = 0.0, dz = 0.0;
    std::vector<double> outer, inner; // the cut stock's radii along Z (inner >= outer: nothing left)
    bool partedOff = false;
};
bool simulateLathe(LatheJob& job, const Body& part, LatheSimulation& out, std::string* error = nullptr);
// The turned stock as a body (revolved about the axis), for display.
Body turnedStock(const LatheJob& job, const Body& part, const LatheSimulation& sim);

// Seconds for an operation's moves at its feeds (constant surface speed clamped to the
// spindle's limit; rapids at the setup's rapid rate).
double latheSeconds(const LatheJob& job, const LatheOperation& op);

const std::vector<std::string>& lathePosts(); // fanuc, haas, linuxcnc
// With `cycles`, rough turning and boring are written as G71 and facing as G72 (Fanuc, Haas),
// where a type I cycle can describe the outline; otherwise, and on LinuxCNC, every pass.
std::string postLathe(LatheJob& job, const Body& part, const std::string& post, std::string* error = nullptr, bool cycles = false);

Json latheToJson(const LatheJob& job);
bool latheFromJson(const Json& j, LatheJob& job);

} // namespace sicdm

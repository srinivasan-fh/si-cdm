#pragma once

// CAM: milling and drilling toolpaths from a part, checked in a stock simulation and
// posted as G-code for a machine's controller.
//
//   Tools       flat, ball and bull-nose end mills, face mills, drills, spot / chamfer
//               mills and taps, with flutes, flute length and tool material
//   Feeds       spindle speed and feed from the material's cutting speed and chip load,
//               with radial chip thinning at light stepovers, clamped to the machine
//   Operations  facing; 2D contour with lead in / out, multiple depths and tabs; pocket
//               and 3D roughing (offset or zig-zag clearing at each depth, ramped
//               entries, stock to leave); parallel and waterline 3D finishing; drilling
//               (spot, drill, peck, chip-break, tap, bore) with holes found on the
//               part; helical boring of holes bigger than the end mill
//   Simulation  the stock as a height field cut by each tool's shape along the moves:
//               removed volume, stock left, gouges, rapid moves through stock, shank
//               collisions and cycle time; the cut stock as a body
//   Post        Fanuc, Haas, GRBL and LinuxCNC G-code, with arcs (G2 / G3) fitted to
//               the cuts and canned drilling cycles where the control has them
//
// All coordinates are millimetres in the model's frame; the post moves them to the
// setup's work origin (G54).
//
// Every milling path is built from the cutter-location (CL) height: the lowest the
// tool's tip can sit at (x, y) without cutting into the part, found exactly against
// each triangle (drop-cutter). Paths on a level come from where that height crosses
// the level, refined by bisection on the exact height, so they touch the part without
// gouging it.

#include "core/Math.hpp"

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace sicdm {

struct Body;
class Json;

enum class CamToolType { Flat, Ball, BullNose, FaceMill, Drill, Spot, Chamfer, Tap };
const char* camToolTypeKey(CamToolType t);  // "flat", "ball", ...
bool camToolTypeFromKey(const std::string& key, CamToolType* out);

struct CamTool {
    int number = 1;                  // T number
    CamToolType type = CamToolType::Flat;
    double diameter = 6.0;           // mm
    double cornerRadius = 0.0;       // bull nose; a ball's is diameter / 2
    double fluteLength = 20.0;       // mm of cutting length
    double tipAngle = 118.0;         // drills, spot and chamfer mills: included angle, degrees
    double pitch = 1.0;              // taps: mm per turn
    int flutes = 3;
    bool hss = false;                // high-speed steel (else carbide)
    std::string name;                // shown in the tool list and the G-code
    double radius() const noexcept { return diameter * 0.5; }
    double tipRadius() const noexcept; // the corner radius used for the cutting shape
    // Height of the tool's cutting edge above its tip at distance d from its axis
    // (infinity past the radius).
    double profile(double d) const noexcept;
    std::string describe() const;    // "T1  6 mm flat end mill, 3 flutes, carbide"
};

struct CamMaterial {
    std::string key, name;
    double cuttingSpeedCarbide = 0.0, cuttingSpeedHss = 0.0; // m/min
    double chipLoadPerDiameter = 0.0; // mm per tooth per mm of diameter (end mills)
    double drillFeedPerDiameter = 0.0; // mm per turn per mm of diameter
    double stepdownFactor = 0.0;      // roughing depth of cut, times the diameter
};
const std::vector<CamMaterial>& camMaterials();
const CamMaterial* camMaterial(const std::string& key);

struct CamMachine {
    double maxRpm = 12000.0;
    double maxFeed = 5000.0;   // mm/min
    double rapid = 10000.0;    // mm/min
    double toolChange = 6.0;   // seconds
};

struct CamFeeds {
    double rpm = 0.0, feed = 0.0, plunge = 0.0; // mm/min
    double chipLoad = 0.0;                      // mm per tooth after chip thinning
};
// The spindle speed and feeds for a tool in a material. `stepover`: the radial width of
// cut in mm (0: full width), which thins the chip and lets the feed rise.
CamFeeds camFeeds(const CamTool& tool, const CamMaterial& material, const CamMachine& machine, double stepover = 0.0);

enum class CamOpKind { Face, Contour, Pocket, Rough, Parallel, Waterline, Drill, Bore };
const char* camOpKey(CamOpKind k);

enum class DrillCycle { Spot, Drill, Peck, ChipBreak, Tap, Bore };
const char* drillCycleKey(DrillCycle c);
bool drillCycleFromKey(const std::string& key, DrillCycle* out);

struct CamMove {
    enum Kind : unsigned char { Rapid, Feed, Plunge } kind = Feed;
    Vec3 to;
    double feed = 0.0; // mm/min (Rapid: unused)
};

struct CamHole {
    double x = 0, y = 0;
    double diameter = 0;
    double top = 0, bottom = 0; // z
    bool through = false;
};

struct CamOperation {
    CamOpKind kind = CamOpKind::Rough;
    std::string name;
    int tool = 1;                  // T number
    // Depths (z): from the top down to the bottom. NaN: from the stock / part.
    double top = std::numeric_limits<double>::quiet_NaN();
    double bottom = std::numeric_limits<double>::quiet_NaN();
    double stepdown = 0.0;         // 0: from the material
    double stepover = 0.0;         // mm; 0: from the tool (40% roughing, 10% finishing ball)
    double leave = 0.0;            // stock to leave on walls and floors, mm
    double angle = 0.0;            // raster direction, degrees from X
    bool climb = true;
    bool raster = false;           // zig-zag clearing instead of offsets
    // Contour: 0 the outside profile, 1 the inside walls (pockets, holes), 2 both.
    int side = 0;
    int tabs = 0;
    double tabWidth = 4.0, tabHeight = 1.5;
    double lead = -1.0;            // lead-in / out arc radius; < 0: from the tool
    // Finishing.
    double scallop = 0.01;         // mm: cusp height left between passes (sets the stepover when it is 0)
    double minSlope = 0.0, maxSlope = 90.0; // degrees: parallel / waterline only cut where the surface leans in this range
    // Drilling.
    DrillCycle cycle = DrillCycle::Drill;
    double peck = 0.0;             // mm; 0: from the material
    double dwell = 0.0;            // s at the bottom
    bool autoHoles = true;         // find the part's holes that fit the tool
    std::vector<CamHole> holes;    // given or found
    // Output.
    std::vector<CamMove> moves;
    bool generated = false;
    std::string summary;           // "412 moves, 3.2 m, 1 min 40 s"
    double rpm = 0, feed = 0, plunge = 0;          // what the moves use (set when generated)
    double setRpm = 0, setFeed = 0, setPlunge = 0; // the user's values (0: from the tool and material)
};

struct CamSetup {
    std::string body;              // the part
    bool hasStock = false;
    Vec3 stockMin, stockMax;       // the raw block
    std::string material = "aluminium";
    enum class Origin { StockTopCorner, StockTopCentre, Model } origin = Origin::StockTopCorner;
    double clearance = 15.0;       // above the stock top: rapids between features
    double retract = 3.0;          // above the stock top: the feed plane
    double tolerance = 0.01;       // mm: path chord and arc fitting tolerance
    std::string post = "fanuc";
    CamMachine machine;
};

struct CamJob {
    CamSetup setup;
    std::vector<CamTool> tools;
    std::vector<CamOperation> ops;
    bool showPaths = true;  // draw the toolpaths in the 3D view
    double partKey = 0.0;   // the part's fingerprint when the paths were made (not saved)
    bool empty() const noexcept { return setup.body.empty() && tools.empty() && ops.empty(); }
    const CamTool* tool(int number) const;
    Vec3 origin() const; // the work zero in model coordinates
};

Json camToJson(const CamJob& job);
bool camFromJson(const Json& j, CamJob& job);

// Fills `op.moves` for the part. False (and `error`) when it can't, e.g. no stock.
bool generateOperation(const CamJob& job, const Body& part, CamOperation& op, std::string* error = nullptr);

// The part's vertical round holes (found from sections), largest first.
std::vector<CamHole> findHoles(const Body& part);

// Drop-cutter: the lowest z of the tool's tip at (x, y) that keeps it out of the part
// (lowest: `floor`, when nothing is under the tool).
double cutterHeight(const Body& part, const CamTool& tool, double x, double y, double floor);

struct CamSimulation {
    double removed = 0.0;          // mm^3
    double maxLeft = 0.0;          // mm of stock left above the part (where there is part)
    double meanLeft = 0.0;
    std::size_t gouges = 0;        // cells cut below the part's surface
    double deepestGouge = 0.0;
    std::size_t rapidCrashes = 0;  // rapid moves through stock
    std::size_t shankHits = 0;     // stock above a tool's flute length while cutting
    double seconds = 0.0;
    double cutLength = 0.0, rapidLength = 0.0;
    double cell = 0.0;
    int nx = 0, ny = 0;
    std::vector<float> height;     // nx * ny, the cut stock's top
};
// Cuts the stock with every operation in order (generating any not yet generated).
bool simulateJob(CamJob& job, const Body& part, CamSimulation& out, std::string* error = nullptr);
// The cut stock as a closed body, for display.
Body simulatedStock(const CamJob& job, const CamSimulation& sim);

struct CamPostReport {
    std::size_t lines = 0, arcs = 0, cycles = 0;
    double seconds = 0.0;
};
// Writes the job's operations as G-code for `post` (fanuc, haas, grbl, linuxcnc).
bool postJob(CamJob& job, const Body& part, const std::string& post, const std::string& path, CamPostReport* report = nullptr,
             std::string* error = nullptr);
std::string postToString(CamJob& job, const Body& part, const std::string& post, CamPostReport* report = nullptr,
                         std::string* error = nullptr);
const std::vector<std::string>& camPosts();

// A fingerprint of the part's shape and place: paths made for another are stale.
double camPartKey(const Body& part);

// Seconds the moves take at their feeds (rapids at the machine's rapid rate).
double movesSeconds(const std::vector<CamMove>& moves, const CamMachine& machine, const Vec3& from);

} // namespace sicdm

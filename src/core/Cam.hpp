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
//               entries, stock to leave), adaptive (light stepover, deep cuts, trochoidal
//               loops wherever the tool would take more than its share), and rest
//               machining (only the material earlier operations left); parallel and
//               waterline 3D finishing; drilling
//               (spot, drill, peck, chip-break, tap, bore) with holes found on the
//               part; helical boring of holes bigger than the end mill
//   Simulation  the stock as a height field (or, with multi-axis operations, voxels) cut
//               by each tool's shape along the moves:
//               removed volume, stock left, gouges, rapid moves through stock, shank
//               and holder collisions (and the stickout that would clear them) and cycle
//               time; the cut stock as a body
//   Multi-axis  3+2 indexed operations (the table turned to A and C, then any 3-axis
//               operation), and 4-axis rotary wrapping (the part turning on A about X
//               under the tool: helical or along-the-axis passes, roughing in levels)
//   Post        Fanuc, Haas, GRBL and LinuxCNC G-code, with arcs (G2 / G3) fitted to
//               the cuts and canned drilling cycles where the control has them; A and C
//               words, and inverse-time feed (G93) for moves that turn the part
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
    std::string vendor, productId;   // from a tool catalogue
    // The tool maker's cutting data, used instead of the material's when set: cutting speed
    // (m/min) and chip load (mm per tooth; drills and spot drills: mm per turn).
    double vc = 0.0, fz = 0.0;
    // The holder: the tool sticks out `stickout` from its face (0: the flute length plus
    // a diameter, at least 5 mm), and it is `holderDiameter` across (0: 3 diameters, at
    // least 20 mm) and `holderLength` long.
    double stickout = 0.0, holderDiameter = 0.0, holderLength = 40.0;
    double radius() const noexcept { return diameter * 0.5; }
    double gauge() const noexcept;        // the stickout in use
    double holderRadius() const noexcept; // the holder's radius in use
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

// The machine the program runs on: what posted programs are checked against.
struct CamMachine {
    std::string name = "generic";
    double maxRpm = 12000.0;
    double maxFeed = 5000.0;   // mm/min
    double rapid = 10000.0;    // mm/min
    double toolChange = 6.0;   // seconds
    // Travel of the spindle nose in machine coordinates (mm), as the control's machine
    // position shows it; an axis isn't checked while its min >= max.
    double travelMin[3] = {0, 0, 0}, travelMax[3] = {0, 0, 0};
    // Where G54 puts the work zero, in machine coordinates. Without it only the program's
    // span on each axis is checked against the travel.
    bool hasWorkOffset = false;
    double workOffset[3] = {0, 0, 0};
    // Rotary axes (degrees): A turns the table about X, C about Z. The generic machine has
    // both, without limits; the presets say what theirs have. B tilts the spindle head about
    // Y instead (a head-table machine: B and C); a machine with B tilts the tool, not the
    // part, and its 5-axis programs use the control's tool centre point mode.
    bool hasA = true, hasC = true, hasB = false;
    double aMin = -99999.0, aMax = 99999.0, cMin = -99999.0, cMax = 99999.0, bMin = -99999.0, bMax = 99999.0;
    bool headTilts() const { return hasB && !hasA; }
    int toolPockets = 0;       // highest T number the changer takes (0: any)
    bool hasTravel(int axis) const { return travelMax[axis] > travelMin[axis]; }
    std::string describe() const;
};
// Typical machine classes to start from: generic, vmc, vmc-4th, trunnion, benchtop, router.
const std::vector<CamMachine>& camMachinePresets();
const CamMachine* camMachinePreset(const std::string& name);

struct CamFeeds {
    double rpm = 0.0, feed = 0.0, plunge = 0.0; // mm/min
    double chipLoad = 0.0;                      // mm per tooth after chip thinning
};
// A tool as JSON (the project's and the tool library's form) and back.
Json camToolToJson(const CamTool& t);
CamTool camToolFromJson(const Json& o);

// The spindle speed and feeds for a tool in a material. `stepover`: the radial width of
// cut in mm (0: full width), which thins the chip and lets the feed rise.
CamFeeds camFeeds(const CamTool& tool, const CamMaterial& material, const CamMachine& machine, double stepover = 0.0);

enum class CamOpKind { Face, Contour, Pocket, Rough, Parallel, Waterline, Drill, Bore, Rotary, Surface5 };
const char* camOpKey(CamOpKind k);

enum class DrillCycle { Spot, Drill, Peck, ChipBreak, Tap, Bore };
const char* drillCycleKey(DrillCycle c);
bool drillCycleFromKey(const std::string& key, DrillCycle* out);

// A move in the machine's frame. With the table turned (A about X, then C about Z, both
// through the job's rotary centre) the part is turned with it: camToPart maps back. On a
// head-table machine `b` tilts the tool about Y instead; the tip is still where `to` says.
struct CamMove {
    enum Kind : unsigned char { Rapid, Feed, Plunge } kind = Feed;
    Vec3 to;
    double feed = 0.0; // mm/min along the part (Rapid: unused)
    float a = 0.0f, c = 0.0f; // degrees
    float b = 0.0f;
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
    bool adaptive = false;         // rough / pocket: light stepover, deep stepdown, trochoids where the engagement would rise
    bool rest = false;             // rough / pocket: only where the operations before left material
    // Multi-axis.
    double indexA = 0.0, indexC = 0.0; // 3+2: the table turned to these (degrees) for the whole operation
    bool along = false;            // rotary: passes along the axis instead of a helix round it
    // Simultaneous 5-axis surface finishing (Surface5): the tool axis follows the surface
    // normal, leaning `leadAngle` along the cut and `sideTilt` across it, never more than
    // `maxTilt` from vertical, and tilting back towards vertical where its shank or holder
    // would touch the part.
    double maxTilt = 60.0, leadAngle = 10.0, sideTilt = 0.0; // degrees
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
    Vec3 origin() const; // the work zero in model coordinates (the rotary centre for multi-axis jobs)
    bool multiAxis() const;     // any operation turns the part
    Vec3 rotaryCentre() const;  // the stock's centre: A and C turn about it
    double swingRadius() const; // the stock's farthest corner from the A axis
    double clearZ() const;      // the clearance height in the machine frame, whatever the table's turn
};

Json camToJson(const CamJob& job);
bool camFromJson(const Json& j, CamJob& job);

// Machine frame <-> part frame for a table turned to (a, c) degrees; the tool's axis
// (machine +Z) as a direction on the part.
Vec3 camToPart(const CamJob& job, const Vec3& machine, float a, float c);
Vec3 camToMachine(const CamJob& job, const Vec3& part, float a, float c);
// The tool's axis on the part (up from the tip), for the table at A and C and the head at B.
Vec3 camToolAxis(float a, float c, float b = 0.0f);

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
    std::size_t holderHits = 0;    // moves where the stock or part reaches a tool's holder
    double neededStickout = 0.0;   // the stickout that would clear every holder hit (mm)
    std::string holderTool;        // the first tool whose holder hits
    double seconds = 0.0;
    double cutLength = 0.0, rapidLength = 0.0;
    double cell = 0.0;
    int nx = 0, ny = 0;
    std::vector<float> height;     // nx * ny, the cut stock's top
    std::vector<float> partTop;    // nx * ny, the part's top (-inf off the part)
    // Multi-axis jobs: voxels instead (0 cut, 1 stock, 2 part's skin, 3 part's inside).
    bool voxels = false;
    int vz = 0;
    Vec3 voxelMin;
    std::vector<unsigned char> voxel; // nx * ny * vz
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
// Moves that turn the table count their length along the part, about `centre` (A only).
double movesSeconds(const std::vector<CamMove>& moves, const CamMachine& machine, const Vec3& from, const Vec3* centre = nullptr);

} // namespace sicdm

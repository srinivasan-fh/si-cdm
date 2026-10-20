#include "ui/Tools.hpp"

namespace sicdm {

namespace {

using C = ToolCategory;
using T = ToolId;

// Order must match ToolId.
constexpr std::array<ToolInfo, static_cast<std::size_t>(ToolId::Count)> kTools = {{
    {T::Move, C::Navigate, "Move", "V", "Drag a body to move it in the view plane.", true},
    {T::Select, C::Navigate, "Select", "S", "Click a body to select it. Click empty space to clear.", true},
    {T::BoxSelect, C::Navigate, "Box Select", "", "Drag a rectangle to select everything inside it.", false},
    {T::LassoSelect, C::Navigate, "Lasso Select", "", "Draw a freehand loop to select what it encloses.", false},
    {T::Orbit, C::Navigate, "Orbit", "O", "Left drag rotates the view around the target (3D window only).", true},
    {T::Pan, C::Navigate, "Pan", "H", "Left drag slides the view.", true},
    {T::Zoom, C::Navigate, "Zoom", "Z", "Left drag up or down to zoom.", true},

    {T::Point, C::Sketch, "Point", "", "Zero-dimensional references for hole placements, centres and weld locations. Click to place.", true},
    {T::Line, C::Sketch, "Line", "L", "Straight segments defining edges, structural frames and linear boundaries. Click start, click end.", true},
    {T::Polyline, C::Sketch, "Polyline", "", "Connected lines acting as a single continuous path. Click points; Enter, double-click or click the start to finish.", true},
    {T::Arc, C::Sketch, "Arc", "A", "Curved segments for smooth transitions, corners and circular paths.", false},
    {T::Conic, C::Sketch, "Conic Section", "", "Parabolas and hyperbolas for precise trajectory paths.", false},
    {T::Circle, C::Sketch, "Circle", "C", "Closed circles for shafts, holes, fasteners and rotational axes. Click centre, click radius.", true},
    {T::Ellipse, C::Sketch, "Ellipse", "", "Stretched circles for specialised fluid and structural profiles.", false},
    {T::Rectangle, C::Sketch, "Rectangle", "R", "Four-sided enclosures for blocks, pockets and stock footprints. Click two opposite corners.", true},
    {T::Slot, C::Sketch, "Slot", "", "Elongated tracks with rounded ends for sliding joints and fasteners.", false},
    {T::Polygon, C::Sketch, "Polygon", "", "Multi-sided shapes (hexagons) for bolts, nuts and sockets.", false},
    {T::Spline, C::Sketch, "Spline", "", "Flexible, mathematically controlled curves for organic and aerodynamic outlines.", false},

    {T::Block, C::Solid, "Block / Cuboid", "B", "Rectangular prism for housings and raw stock. Click to place.", true},
    {T::Wedge, C::Solid, "Wedge", "", "Right-angled triangular block for braces, ramps and stiffeners. Click to place.", true},
    {T::Pyramid, C::Solid, "Pyramid", "", "Tapered multi-sided shape for anchors and load distribution. Click to place.", true},
    {T::Prism, C::Solid, "Prism", "", "Extruded regular polygon for rigid beams and mechanical keys. Click to place.", true},
    {T::Cylinder, C::Solid, "Cylinder", "Y", "Round column for axles, pistons and rotary pins. Click to place.", true},
    {T::Cone, C::Solid, "Cone", "", "Tapered shape for nose cones, nozzles and countersinks. Click to place.", true},
    {T::Sphere, C::Solid, "Sphere", "", "Round shape for ball-and-socket joints. Click to place.", true},
    {T::Torus, C::Solid, "Torus", "", "Doughnut shape for O-rings, seals and circular sweeps. Click to place.", true},

    {T::Extrude, C::Feature, "Extrude", "E",
     "Push a closed 2D profile linearly into brackets, beams and blocks. Click a rectangle, circle or closed "
     "polyline, drag or type the depth, click to finish.",
     true},
    {T::Revolve, C::Feature, "Revolve", "", "Spin a profile around an axis to form wheels, gears and pulleys.", false},
    {T::Sweep, C::Feature, "Sweep", "", "Push a profile along a curved path for pipes, wiring and springs.", false},
    {T::Loft, C::Feature, "Loft", "", "Morph smoothly between 2D shapes for ducting and air intakes.", false},
    {T::Helix, C::Feature, "Helix / Spiral", "", "Corkscrew paths for threads, screws and coils.", false},
    {T::Rib, C::Feature, "Rib / Web", "", "Thin internal walls that reinforce cavities without excess weight.", false},

    {T::PlaneSurface, C::Surface, "Plane Surface", "", "Flat zero-thickness sheets for construction boundaries and mating faces.", false},
    {T::TabulatedCylinder, C::Surface, "Tabulated Cylinder", "", "Surface projected linearly from a curve for simple shields.", false},
    {T::RuledSurface, C::Surface, "Ruled Surface", "", "Straight transitions blended between two boundary curves.", false},
    {T::BSplinePatch, C::Surface, "Bezier / B-Spline Patch", "", "Surface sheets shaped by a control-point grid.", false},
    {T::CoonsPatch, C::Surface, "Coons Patch", "", "Skin interpolated between four boundary curves.", false},
    {T::GordonSurface, C::Surface, "Gordon Surface", "", "Network surface woven over a grid of profile curves.", false},

    {T::ClassANurbs, C::ClassA, "Class-A NURBS", "", "Ultra-smooth surfaces with reflection continuity for body panels.", false},
    {T::OffsetSurface, C::ClassA, "Offset Surface", "", "Parallel skins defining uniform wall thickness over freeform shapes.", false},
    {T::BlendedFillet, C::ClassA, "Blended Fillet", "", "Variable-radius transitions merging intersecting panels.", false},
    {T::TwistedAirfoil, C::ClassA, "Twisted Airfoil", "", "Blade profiles morphing along a path for propellers and wings.", false},
    {T::TubercleWing, C::ClassA, "Tubercle Wing", "", "Whale-fin-inspired leading edges that delay stall on UAVs.", false},

    {T::TopologyOptimized, C::Generative, "Topology Optimisation", "", "Skeletal shapes stripped of dead weight, ideal for robot limbs.", false},
    {T::TpmsLattice, C::Generative, "TPMS Lattice", "", "Periodic gyroid-type infill for light, crash-absorbent parts.", false},

    {T::MeshImport, C::Mesh, "STL / OBJ Mesh", "", "Triangulated surfaces for 3D printers and slicers.", false},
    {T::PointCloud, C::Mesh, "Point Cloud", "", "Millions of points captured by 3D laser scanners.", false},
    {T::SubD, C::Mesh, "SubD Surface", "", "Smooth organic shapes sculpted from a low-resolution cage.", false},

    {T::SculptDraw, C::Sculpt, "Draw (Push / Pull)", "D",
     "Pull the surface out like pressing clay from behind; Ctrl pushes it in. Drag over a body. Shift smooths.", true},
    {T::SculptClay, C::Sculpt, "Clay (Add)", "", "Build up material in flat layers, like laying on strips of clay. Ctrl removes.", true},
    {T::SculptCarve, C::Sculpt, "Carve (Remove)", "", "Dig material away with a sharp-edged tool. Ctrl raises a ridge instead.", true},
    {T::SculptInflate, C::Sculpt, "Inflate", "", "Swell the surface along its own normals (Ctrl deflates).", true},
    {T::SculptGrab, C::Sculpt, "Grab", "", "Pick up the surface under the brush and drag it, like pulling clay.", true},
    {T::SculptPinch, C::Sculpt, "Pinch", "", "Draw the surface towards the stroke for sharp creases and hard edges (Ctrl spreads).", true},
    {T::SculptCrease, C::Sculpt, "Crease", "", "Cut a sharp valley along the stroke (Ctrl: a sharp ridge).", true},
    {T::SculptSmooth, C::Sculpt, "Smooth", "", "Relax bumps and noise (Laplacian smoothing). Shift smooths with any brush.", true},
    {T::SculptFlatten, C::Sculpt, "Flatten", "", "Press the surface flat to the average plane under the brush.", true},
    {T::SculptScrape, C::Sculpt, "Scrape (Trim)", "", "Shave off the high spots down to a plane, like a trimming knife (Ctrl fills).", true},

    {T::CutHole, C::Cut, "Drill Hole", "", "Click a surface to drill a hole along its normal: through, or blind with a 118 deg tip.", true},
    {T::CutPocket, C::Cut, "Pocket", "", "Click a surface to hollow a rectangular, round or ball-end pocket.", true},
    {T::CutChannel, C::Cut, "Channel", "", "Drag over a surface to carve a round-bottomed channel along the stroke.", true},
    {T::CutKnife, C::Cut, "Knife / Trim", "", "Drag a line across a body: the plane through it slices the body and the smaller side is removed. The cut is capped.", true},
    {T::AddBoss, C::Cut, "Add Volume (Boss)", "", "Click a surface to fuse on a boss, pad or dome.", true},
    {T::BooleanBodies, C::Cut, "Boolean Bodies", "", "Click a body, then a second one: union, subtract or intersect them exactly.", true},
}};

constexpr ToolId gMove[] = {T::Move};
constexpr ToolId gSelect[] = {T::Select, T::BoxSelect, T::LassoSelect};
constexpr ToolId gOrbit[] = {T::Orbit};
constexpr ToolId gPan[] = {T::Pan};
constexpr ToolId gZoom[] = {T::Zoom};
constexpr ToolId gPoint[] = {T::Point};
constexpr ToolId gLine[] = {T::Line, T::Polyline};
constexpr ToolId gArc[] = {T::Arc, T::Conic};
constexpr ToolId gCircle[] = {T::Circle, T::Ellipse};
constexpr ToolId gRect[] = {T::Rectangle, T::Slot, T::Polygon};
constexpr ToolId gSpline[] = {T::Spline};
constexpr ToolId gBlock[] = {T::Block, T::Wedge, T::Pyramid, T::Prism};
constexpr ToolId gCylinder[] = {T::Cylinder, T::Cone};
constexpr ToolId gSphere[] = {T::Sphere, T::Torus};
constexpr ToolId gExtrude[] = {T::Extrude, T::Revolve};
constexpr ToolId gSweep[] = {T::Sweep, T::Loft, T::Helix};
constexpr ToolId gRib[] = {T::Rib};
constexpr ToolId gPlane[] = {T::PlaneSurface, T::TabulatedCylinder, T::RuledSurface};
constexpr ToolId gPatch[] = {T::BSplinePatch, T::CoonsPatch, T::GordonSurface};
constexpr ToolId gNurbs[] = {T::ClassANurbs, T::OffsetSurface, T::BlendedFillet};
constexpr ToolId gAirfoil[] = {T::TwistedAirfoil, T::TubercleWing};
constexpr ToolId gTopo[] = {T::TopologyOptimized};
constexpr ToolId gLattice[] = {T::TpmsLattice};
constexpr ToolId gMesh[] = {T::MeshImport, T::PointCloud, T::SubD};
constexpr ToolId gSculptAdd[] = {T::SculptDraw, T::SculptClay, T::SculptCarve, T::SculptInflate};
constexpr ToolId gSculptGrab[] = {T::SculptGrab};
constexpr ToolId gSculptPinch[] = {T::SculptPinch, T::SculptCrease};
constexpr ToolId gSculptSmooth[] = {T::SculptSmooth, T::SculptFlatten, T::SculptScrape};
constexpr ToolId gCutHole[] = {T::CutHole, T::CutPocket, T::CutChannel};
constexpr ToolId gCutKnife[] = {T::CutKnife};
constexpr ToolId gBoss[] = {T::AddBoss};
constexpr ToolId gBoolean[] = {T::BooleanBodies};

constexpr ToolGroup kGroups[] = {
    {C::Navigate, gMove},   {C::Navigate, gSelect}, {C::Navigate, gOrbit},   {C::Navigate, gPan},
    {C::Navigate, gZoom},   {C::Sketch, gPoint},    {C::Sketch, gLine},      {C::Sketch, gArc},
    {C::Sketch, gCircle},   {C::Sketch, gRect},     {C::Sketch, gSpline},    {C::Solid, gBlock},
    {C::Solid, gCylinder},  {C::Solid, gSphere},    {C::Feature, gExtrude},  {C::Feature, gSweep},
    {C::Feature, gRib},     {C::Surface, gPlane},   {C::Surface, gPatch},    {C::ClassA, gNurbs},
    {C::ClassA, gAirfoil},  {C::Generative, gTopo}, {C::Generative, gLattice}, {C::Mesh, gMesh},
    {C::Sculpt, gSculptAdd}, {C::Sculpt, gSculptGrab}, {C::Sculpt, gSculptPinch}, {C::Sculpt, gSculptSmooth},
    {C::Cut, gCutHole},     {C::Cut, gCutKnife},    {C::Cut, gBoss},        {C::Cut, gBoolean},
};

} // namespace

const ToolInfo& toolInfo(ToolId id) { return kTools[static_cast<std::size_t>(id)]; }

std::span<const ToolGroup> toolGroups() { return kGroups; }

const char* categoryName(ToolCategory c) {
    switch (c) {
    case C::Navigate: return "Navigate & Select";
    case C::Sketch: return "2D Sketches & Profiles";
    case C::Solid: return "3D Solid Primitives";
    case C::Feature: return "Form Features";
    case C::Surface: return "Analytical Surfaces";
    case C::ClassA: return "Class-A & Aerodynamic";
    case C::Generative: return "Generative";
    case C::Mesh: return "Mesh Data";
    case C::Sculpt: return "Digital Sculpting";
    case C::Cut: return "Cuts & Booleans";
    }
    return "";
}

bool isPrimitiveTool(ToolId id) { return toolInfo(id).category == ToolCategory::Solid; }

bool isSculptTool(ToolId id) { return toolInfo(id).category == ToolCategory::Sculpt; }

bool isCutTool(ToolId id) { return toolInfo(id).category == ToolCategory::Cut; }

bool isSketchTool(ToolId id) {
    return id == ToolId::Point || id == ToolId::Line || id == ToolId::Polyline || id == ToolId::Rectangle ||
           id == ToolId::Circle;
}

} // namespace sicdm

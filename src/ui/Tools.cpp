#include "ui/Tools.hpp"
#include "core/Translate.hpp"

namespace sicdm {

namespace {

using C = ToolCategory;
using T = ToolId;

// Order must match ToolId.
constexpr std::array<ToolInfo, static_cast<std::size_t>(ToolId::Count)> kTools = {{
    {T::Move, C::Navigate, SICDM_TR_NOOP("Move"), "V", SICDM_TR_NOOP("Drag a body to move it in the view plane."), true},
    {T::Select, C::Navigate, SICDM_TR_NOOP("Select"), "S", SICDM_TR_NOOP("Click a body to select it. Click empty space to clear."), true},
    {T::BoxSelect, C::Navigate, SICDM_TR_NOOP("Box Select"), "", SICDM_TR_NOOP("Drag a rectangle to select everything inside it."), false},
    {T::LassoSelect, C::Navigate, SICDM_TR_NOOP("Lasso Select"), "", SICDM_TR_NOOP("Draw a freehand loop to select what it encloses."), false},
    {T::Orbit, C::Navigate, SICDM_TR_NOOP("Orbit"), "O", SICDM_TR_NOOP("Left drag rotates the view around the target (3D window only)."), true},
    {T::Pan, C::Navigate, SICDM_TR_NOOP("Pan"), "H", SICDM_TR_NOOP("Left drag slides the view."), true},
    {T::Zoom, C::Navigate, SICDM_TR_NOOP("Zoom"), "Z", SICDM_TR_NOOP("Left drag up or down to zoom."), true},

    {T::Point, C::Sketch, SICDM_TR_NOOP("Point"), "", SICDM_TR_NOOP("Zero-dimensional references for hole placements, centres and weld locations. Click to place."), true},
    {T::Line, C::Sketch, SICDM_TR_NOOP("Line"), "L", SICDM_TR_NOOP("Straight segments defining edges, structural frames and linear boundaries. Click start, click end."), true},
    {T::Polyline, C::Sketch, SICDM_TR_NOOP("Polyline"), "", SICDM_TR_NOOP("Connected lines acting as a single continuous path. Click points; Enter, double-click or click the start to finish."), true},
    {T::Arc, C::Sketch, SICDM_TR_NOOP("Arc"), "A", SICDM_TR_NOOP("Curved segments for smooth transitions, corners and circular paths. Click the start, the end, then a point on the arc."), true},
    {T::Conic, C::Sketch, SICDM_TR_NOOP("Conic Section"), "", SICDM_TR_NOOP("Parabolas and hyperbolas for precise trajectory paths."), false},
    {T::Circle, C::Sketch, SICDM_TR_NOOP("Circle"), "C", SICDM_TR_NOOP("Closed circles for shafts, holes, fasteners and rotational axes. Click centre, click radius."), true},
    {T::Ellipse, C::Sketch, SICDM_TR_NOOP("Ellipse"), "", SICDM_TR_NOOP("Stretched circles for specialised fluid and structural profiles. Click the centre, the end of the major axis, then the minor radius."), true},
    {T::Rectangle, C::Sketch, SICDM_TR_NOOP("Rectangle"), "R", SICDM_TR_NOOP("Four-sided enclosures for blocks, pockets and stock footprints. Click two opposite corners."), true},
    {T::Slot, C::Sketch, SICDM_TR_NOOP("Slot"), "", SICDM_TR_NOOP("Elongated tracks with rounded ends for sliding joints and fasteners. Click the two end centres, then the width."), true},
    {T::Polygon, C::Sketch, SICDM_TR_NOOP("Polygon"), "", SICDM_TR_NOOP("Multi-sided shapes (hexagons) for bolts, nuts and sockets. Click the centre, then a corner (sides in the options bar)."), true},
    {T::Spline, C::Sketch, SICDM_TR_NOOP("Spline"), "", SICDM_TR_NOOP("Flexible, mathematically controlled curves for organic and aerodynamic outlines. Click the points it passes through; Enter or double-click finishes, clicking the first point closes it."), true},
    {T::SketchTrim, C::Sketch, SICDM_TR_NOOP("Trim"), "T", SICDM_TR_NOOP("Click a piece of a sketch curve to cut it away, back to the curves crossing it."), true},
    {T::SketchExtend, C::Sketch, SICDM_TR_NOOP("Extend"), "", SICDM_TR_NOOP("Click near the end of a line, arc or open polyline to carry it on to the next curve."), true},
    {T::SketchOffset, C::Sketch, SICDM_TR_NOOP("Offset"), "", SICDM_TR_NOOP("Click a sketch curve, then the side: a copy the distance in the options bar away."), true},

    {T::Block, C::Solid, SICDM_TR_NOOP("Block / Cuboid"), "B", SICDM_TR_NOOP("Rectangular prism for housings and raw stock. Click to place."), true},
    {T::Wedge, C::Solid, SICDM_TR_NOOP("Wedge"), "", SICDM_TR_NOOP("Right-angled triangular block for braces, ramps and stiffeners. Click to place."), true},
    {T::Pyramid, C::Solid, SICDM_TR_NOOP("Pyramid"), "", SICDM_TR_NOOP("Tapered multi-sided shape for anchors and load distribution. Click to place."), true},
    {T::Prism, C::Solid, SICDM_TR_NOOP("Prism"), "", SICDM_TR_NOOP("Extruded regular polygon for rigid beams and mechanical keys. Click to place."), true},
    {T::Cylinder, C::Solid, SICDM_TR_NOOP("Cylinder"), "Y", SICDM_TR_NOOP("Round column for axles, pistons and rotary pins. Click to place."), true},
    {T::Cone, C::Solid, SICDM_TR_NOOP("Cone"), "", SICDM_TR_NOOP("Tapered shape for nose cones, nozzles and countersinks. Click to place."), true},
    {T::Sphere, C::Solid, SICDM_TR_NOOP("Sphere"), "", SICDM_TR_NOOP("Round shape for ball-and-socket joints. Click to place."), true},
    {T::Torus, C::Solid, SICDM_TR_NOOP("Torus"), "", SICDM_TR_NOOP("Doughnut shape for O-rings, seals and circular sweeps. Click to place."), true},

    {T::Extrude, C::Feature, "Extrude", "E",
     "Push a closed 2D profile linearly into brackets, beams and blocks. Click a rectangle, circle or closed "
     "polyline, drag or type the depth, click to finish.",
     true},
    {T::Revolve, C::Feature, SICDM_TR_NOOP("Revolve"), "", SICDM_TR_NOOP("Spin a profile around an axis to form wheels, gears and pulleys."), false},
    {T::Sweep, C::Feature, SICDM_TR_NOOP("Sweep"), "", SICDM_TR_NOOP("Push a profile along a curved path for pipes, wiring and springs."), false},
    {T::Loft, C::Feature, SICDM_TR_NOOP("Loft"), "", SICDM_TR_NOOP("Morph smoothly between 2D shapes for ducting and air intakes."), false},
    {T::Helix, C::Feature, SICDM_TR_NOOP("Helix / Spiral"), "", SICDM_TR_NOOP("Corkscrew paths for threads, screws and coils."), false},
    {T::Rib, C::Feature, SICDM_TR_NOOP("Rib / Web"), "", SICDM_TR_NOOP("Thin internal walls that reinforce cavities without excess weight."), false},

    {T::PlaneSurface, C::Surface, SICDM_TR_NOOP("Plane Surface"), "", SICDM_TR_NOOP("Flat zero-thickness sheets for construction boundaries and mating faces."), false},
    {T::TabulatedCylinder, C::Surface, SICDM_TR_NOOP("Tabulated Cylinder"), "", SICDM_TR_NOOP("Surface projected linearly from a curve for simple shields."), false},
    {T::RuledSurface, C::Surface, SICDM_TR_NOOP("Ruled Surface"), "", SICDM_TR_NOOP("Straight transitions blended between two boundary curves."), false},
    {T::BSplinePatch, C::Surface, SICDM_TR_NOOP("Bezier / B-Spline Patch"), "", SICDM_TR_NOOP("Surface sheets shaped by a control-point grid."), false},
    {T::CoonsPatch, C::Surface, SICDM_TR_NOOP("Coons Patch"), "", SICDM_TR_NOOP("Skin interpolated between four boundary curves."), false},
    {T::GordonSurface, C::Surface, SICDM_TR_NOOP("Gordon Surface"), "", SICDM_TR_NOOP("Network surface woven over a grid of profile curves."), false},

    {T::ClassANurbs, C::ClassA, SICDM_TR_NOOP("Class-A NURBS"), "", SICDM_TR_NOOP("Ultra-smooth surfaces with reflection continuity for body panels."), false},
    {T::OffsetSurface, C::ClassA, SICDM_TR_NOOP("Offset Surface"), "", SICDM_TR_NOOP("Parallel skins defining uniform wall thickness over freeform shapes."), false},
    {T::BlendedFillet, C::ClassA, SICDM_TR_NOOP("Blended Fillet"), "", SICDM_TR_NOOP("Variable-radius transitions merging intersecting panels."), false},
    {T::TwistedAirfoil, C::ClassA, SICDM_TR_NOOP("Twisted Airfoil"), "", SICDM_TR_NOOP("Blade profiles morphing along a path for propellers and wings."), false},
    {T::TubercleWing, C::ClassA, SICDM_TR_NOOP("Tubercle Wing"), "", SICDM_TR_NOOP("Whale-fin-inspired leading edges that delay stall on UAVs."), false},

    {T::TopologyOptimized, C::Generative, SICDM_TR_NOOP("Topology Optimisation"), "", SICDM_TR_NOOP("Skeletal shapes stripped of dead weight, ideal for robot limbs."), false},
    {T::TpmsLattice, C::Generative, SICDM_TR_NOOP("TPMS Lattice"), "", SICDM_TR_NOOP("Periodic gyroid-type infill for light, crash-absorbent parts. Click a solid body to fill it."), true},

    {T::MeshImport, C::Mesh, SICDM_TR_NOOP("STL / OBJ Mesh"), "", SICDM_TR_NOOP("Triangulated surfaces for 3D printers and slicers."), false},
    {T::PointCloud, C::Mesh, SICDM_TR_NOOP("Point Cloud"), "", SICDM_TR_NOOP("Millions of points captured by 3D laser scanners."), false},
    {T::SubD, C::Mesh, SICDM_TR_NOOP("SubD Surface"), "", SICDM_TR_NOOP("Smooth organic shapes sculpted from a low-resolution cage."), false},

    {T::SculptDraw, C::Sculpt, "Draw (Push / Pull)", "D",
     "Pull the surface out like pressing clay from behind; Ctrl pushes it in. Drag over a body. Shift smooths.", true},
    {T::SculptClay, C::Sculpt, SICDM_TR_NOOP("Clay (Add)"), "", SICDM_TR_NOOP("Build up material in flat layers, like laying on strips of clay. Ctrl removes."), true},
    {T::SculptCarve, C::Sculpt, SICDM_TR_NOOP("Carve (Remove)"), "", SICDM_TR_NOOP("Dig material away with a sharp-edged tool. Ctrl raises a ridge instead."), true},
    {T::SculptInflate, C::Sculpt, SICDM_TR_NOOP("Inflate"), "", SICDM_TR_NOOP("Swell the surface along its own normals (Ctrl deflates)."), true},
    {T::SculptGrab, C::Sculpt, SICDM_TR_NOOP("Grab"), "", SICDM_TR_NOOP("Pick up the surface under the brush and drag it, like pulling clay."), true},
    {T::SculptPinch, C::Sculpt, SICDM_TR_NOOP("Pinch"), "", SICDM_TR_NOOP("Draw the surface towards the stroke for sharp creases and hard edges (Ctrl spreads)."), true},
    {T::SculptCrease, C::Sculpt, SICDM_TR_NOOP("Crease"), "", SICDM_TR_NOOP("Cut a sharp valley along the stroke (Ctrl: a sharp ridge)."), true},
    {T::SculptSmooth, C::Sculpt, SICDM_TR_NOOP("Smooth"), "", SICDM_TR_NOOP("Relax bumps and noise (Laplacian smoothing). Shift smooths with any brush."), true},
    {T::SculptFlatten, C::Sculpt, SICDM_TR_NOOP("Flatten"), "", SICDM_TR_NOOP("Press the surface flat to the average plane under the brush."), true},
    {T::SculptScrape, C::Sculpt, SICDM_TR_NOOP("Scrape (Trim)"), "", SICDM_TR_NOOP("Shave off the high spots down to a plane, like a trimming knife (Ctrl fills)."), true},
    {T::SculptLayer, C::Sculpt, SICDM_TR_NOOP("Layer"), "", SICDM_TR_NOOP("Raise the surface to an even height over the stroke, once per stroke (Ctrl lowers)."), true},
    {T::SculptClayStrips, C::Sculpt, SICDM_TR_NOOP("Clay Strips"), "", SICDM_TR_NOOP("Lay on flat-topped strips of clay along the stroke (square brush). Ctrl removes."), true},
    {T::SculptFill, C::Sculpt, SICDM_TR_NOOP("Fill"), "", SICDM_TR_NOOP("Raise the low spots up to the plane under the brush (Ctrl scrapes)."), true},
    {T::SculptBlob, C::Sculpt, SICDM_TR_NOOP("Blob"), "", SICDM_TR_NOOP("Swell rounded blobs that spread from the centre (Ctrl: dents)."), true},
    {T::SculptSnakeHook, C::Sculpt, SICDM_TR_NOOP("Snake Hook"), "", SICDM_TR_NOOP("Pull the surface out along the stroke, for horns, tentacles and strands."), true},
    {T::SculptNudge, C::Sculpt, SICDM_TR_NOOP("Nudge"), "", SICDM_TR_NOOP("Push the surface along itself in the stroke's direction."), true},
    {T::SculptMask, C::Sculpt, SICDM_TR_NOOP("Mask"), "", SICDM_TR_NOOP("Paint a mask: masked surface holds still under the other brushes. Ctrl unmasks."), true},
    {T::SculptPaint, C::Sculpt, SICDM_TR_NOOP("Paint"), "", SICDM_TR_NOOP("Polypaint: paint colour onto the surface (the mask holds it off). Ctrl paints the body's own colour back."), true},

    {T::CutHole, C::Cut, SICDM_TR_NOOP("Drill Hole"), "", SICDM_TR_NOOP("Click a surface to drill a hole along its normal: through, or blind with a 118 deg tip."), true},
    {T::CutPocket, C::Cut, SICDM_TR_NOOP("Pocket"), "", SICDM_TR_NOOP("Click a surface to hollow a rectangular, round or ball-end pocket."), true},
    {T::CutChannel, C::Cut, SICDM_TR_NOOP("Channel"), "", SICDM_TR_NOOP("Drag over a surface to carve a round-bottomed channel along the stroke."), true},
    {T::CutKnife, C::Cut, SICDM_TR_NOOP("Knife / Trim"), "", SICDM_TR_NOOP("Drag a line across a body: the plane through it slices the body and the smaller side is removed. The cut is capped."), true},
    {T::AddBoss, C::Cut, SICDM_TR_NOOP("Add Volume (Boss)"), "", SICDM_TR_NOOP("Click a surface to fuse on a boss, pad or dome."), true},
    {T::BooleanBodies, C::Cut, SICDM_TR_NOOP("Boolean Bodies"), "", SICDM_TR_NOOP("Click a body, then a second one: union, subtract or intersect them exactly."), true},
    {T::PushPull, C::Cut, SICDM_TR_NOOP("Push / Pull Face"), "", SICDM_TR_NOOP("Drag a face of any solid along its normal: a flat face moves and its neighbours follow; a hole or boss gets larger or smaller. Shift: whole millimetres."), true},
    {T::DeleteFace, C::Cut, SICDM_TR_NOOP("Delete Face"), "", SICDM_TR_NOOP("Click a hole, boss, fillet or chamfer to take it away: the faces round it close up."), true},

    {T::ShellBody, C::Wearable, SICDM_TR_NOOP("Shell / Hollow"), "", SICDM_TR_NOOP("Click a body to hollow it to a wall thickness (armour plating), or solidify a surface. With Open, the side you click is cut open: a helmet, a gauntlet, a chest plate."), true},
    {T::ArmorBody, C::Wearable, SICDM_TR_NOOP("Armour Offset"), "", SICDM_TR_NOOP("Click a body part (a humanoid's forearm, chest...) for an armour shell around it: a clearance gap for padding, wiring and movement, then the wall. It moves with the part."), true},
    {T::JointBody, C::Wearable, SICDM_TR_NOOP("Joint / Hinge"), "", SICDM_TR_NOOP("Click a part at its pivot: a hinge pin or ball joint there, and a motion swinging the part through its range against the part it touches."), true},
    {T::LightenBody, C::Wearable, SICDM_TR_NOOP("Lighten (Lattice)"), "", SICDM_TR_NOOP("Click a thick part: keep a skin and fill the inside with a gyroid lattice (or hollow it) to save weight."), true},
    {T::InterferenceCheck, C::Wearable, SICDM_TR_NOOP("Interference Check"), "", SICDM_TR_NOOP("Click to sweep the motions (gestures, walk, joints) and list every part that collides or comes closer than the clearance."), true},
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
constexpr ToolId gSketchEdit[] = {T::SketchTrim, T::SketchExtend, T::SketchOffset};
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
constexpr ToolId gSculptAdd[] = {T::SculptDraw, T::SculptClay, T::SculptClayStrips, T::SculptLayer, T::SculptCarve, T::SculptInflate, T::SculptBlob};
constexpr ToolId gSculptGrab[] = {T::SculptGrab, T::SculptSnakeHook, T::SculptNudge};
constexpr ToolId gSculptPinch[] = {T::SculptPinch, T::SculptCrease};
constexpr ToolId gSculptSmooth[] = {T::SculptSmooth, T::SculptFlatten, T::SculptScrape, T::SculptFill};
constexpr ToolId gSculptMask[] = {T::SculptMask, T::SculptPaint};
constexpr ToolId gCutHole[] = {T::CutHole, T::CutPocket, T::CutChannel};
constexpr ToolId gCutKnife[] = {T::CutKnife};
constexpr ToolId gBoss[] = {T::AddBoss};
constexpr ToolId gBoolean[] = {T::BooleanBodies};
constexpr ToolId gDirect[] = {T::PushPull, T::DeleteFace};
constexpr ToolId gShell[] = {T::ShellBody, T::LightenBody};
constexpr ToolId gArmor[] = {T::ArmorBody};
constexpr ToolId gJoint[] = {T::JointBody};
constexpr ToolId gInterference[] = {T::InterferenceCheck};

constexpr ToolGroup kGroups[] = {
    {C::Navigate, gMove},   {C::Navigate, gSelect}, {C::Navigate, gOrbit},   {C::Navigate, gPan},
    {C::Navigate, gZoom},   {C::Sketch, gPoint},    {C::Sketch, gLine},      {C::Sketch, gArc},
    {C::Sketch, gCircle},   {C::Sketch, gRect},     {C::Sketch, gSpline},    {C::Sketch, gSketchEdit}, {C::Solid, gBlock},
    {C::Solid, gCylinder},  {C::Solid, gSphere},    {C::Feature, gExtrude},  {C::Feature, gSweep},
    {C::Feature, gRib},     {C::Surface, gPlane},   {C::Surface, gPatch},    {C::ClassA, gNurbs},
    {C::ClassA, gAirfoil},  {C::Generative, gTopo}, {C::Generative, gLattice}, {C::Mesh, gMesh},
    {C::Sculpt, gSculptAdd}, {C::Sculpt, gSculptGrab}, {C::Sculpt, gSculptPinch}, {C::Sculpt, gSculptSmooth}, {C::Sculpt, gSculptMask},
    {C::Cut, gCutHole},     {C::Cut, gCutKnife},    {C::Cut, gBoss},        {C::Cut, gBoolean},     {C::Cut, gDirect},
    {C::Wearable, gShell},  {C::Wearable, gArmor},  {C::Wearable, gJoint},  {C::Wearable, gInterference},
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
    case C::Wearable: return "Armour & Exoskeleton";
    }
    return "";
}

bool isPrimitiveTool(ToolId id) { return toolInfo(id).category == ToolCategory::Solid; }

bool isSculptTool(ToolId id) { return toolInfo(id).category == ToolCategory::Sculpt; }

bool isCutTool(ToolId id) { return toolInfo(id).category == ToolCategory::Cut; }

bool isWearableTool(ToolId id) { return toolInfo(id).category == ToolCategory::Wearable || id == ToolId::TpmsLattice; }

bool isSketchTool(ToolId id) {
    return id == ToolId::Point || id == ToolId::Line || id == ToolId::Polyline || id == ToolId::Rectangle || id == ToolId::Circle || id == ToolId::Arc ||
           id == ToolId::Ellipse || id == ToolId::Slot || id == ToolId::Polygon || id == ToolId::Spline;
}

bool isSketchEditTool(ToolId id) { return id == ToolId::SketchTrim || id == ToolId::SketchExtend || id == ToolId::SketchOffset; }

} // namespace sicdm

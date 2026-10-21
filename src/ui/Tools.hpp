#pragma once

#include <array>
#include <cstddef>
#include <span>

namespace sicdm {

enum class ToolId {
    // Navigate & select
    Move, Select, BoxSelect, LassoSelect, Orbit, Pan, Zoom,
    // I. 2D sketches & profiles
    Point, Line, Polyline, Arc, Conic, Circle, Ellipse, Rectangle, Slot, Polygon, Spline,
    // II. 3D solid primitives
    Block, Wedge, Pyramid, Prism, Cylinder, Cone, Sphere, Torus,
    // III. Form features
    Extrude, Revolve, Sweep, Loft, Helix, Rib,
    // IV. Analytical & synthetic surfaces
    PlaneSurface, TabulatedCylinder, RuledSurface, BSplinePatch, CoonsPatch, GordonSurface,
    // V. Class-A & aerodynamic
    ClassANurbs, OffsetSurface, BlendedFillet, TwistedAirfoil, TubercleWing,
    // VI. Generative
    TopologyOptimized, TpmsLattice,
    // VII. Mesh data
    MeshImport, PointCloud, SubD,
    // VIII. Digital sculpting (clay brushes on any body)
    SculptDraw, SculptClay, SculptCarve, SculptInflate, SculptGrab, SculptPinch, SculptCrease, SculptSmooth,
    SculptFlatten, SculptScrape, SculptLayer, SculptClayStrips, SculptFill, SculptBlob, SculptSnakeHook, SculptNudge,
    SculptMask,
    // IX. Cuts & booleans (exact, closed results)
    CutHole, CutPocket, CutChannel, CutKnife, AddBoss, BooleanBodies,
    // X. Armour & exoskeletons (wearable parts over a human base)
    ShellBody, ArmorBody, JointBody, LightenBody, InterferenceCheck,
    Count
};

enum class ToolCategory { Navigate, Sketch, Solid, Feature, Surface, ClassA, Generative, Mesh, Sculpt, Cut, Wearable };

struct ToolInfo {
    ToolId id;
    ToolCategory category;
    const char* name;
    const char* shortcut;    // single key, "" for none
    const char* description; // tooltip
    bool available;          // implemented in this build
};

const ToolInfo& toolInfo(ToolId id);

// A slot in the tool rail. Related tools share a slot (like Photoshop):
// the slot shows the last-used tool and a flyout lists the rest.
struct ToolGroup {
    ToolCategory category;
    std::span<const ToolId> tools;
};

std::span<const ToolGroup> toolGroups();

const char* categoryName(ToolCategory c);

// True for tools that place a 3D primitive with a click.
bool isPrimitiveTool(ToolId id);

// Ctrl / Cmd held: ImGui's modifier flag or the physical keys (the flag can lag
// a key pressed in the same event batch).
bool commandModifierHeld();

// True for the 2D sketch tools that are implemented (draw on the active sketch plane).
bool isSketchTool(ToolId id);

// Sculpting brushes (drag over a body) and the cut / boolean tools.
bool isSculptTool(ToolId id);
bool isCutTool(ToolId id);
// Armour tools (click a body); the TPMS lattice tool works like Lighten.
bool isWearableTool(ToolId id);

} // namespace sicdm

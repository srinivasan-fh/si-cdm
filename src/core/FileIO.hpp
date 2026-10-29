#pragma once

// Project files and data exchange.
//
//   .sicdm      Project (JSON): parametric bodies, meshes, sketches, images
//   .stl .obj   3D meshes: import as a body, export visible bodies
//   .dxf        2D drawings: import onto / export from the active sketch plane
//   .svg        2D drawing export of the active sketch plane
//   images      .png .jpg .jpeg .bmp .tga .gif .psd .hdr .pnm: reference images

#include "core/Cam.hpp"
#include "core/Lathe.hpp"
#include "core/Pdm.hpp"
#include "core/Assembly.hpp"
#include "core/SectionView.hpp"
#include "core/Stackup.hpp"
#include "core/Drawing.hpp"
#include "core/Render.hpp"
#include "core/Scene.hpp"
#include "core/Units.hpp"
#include "core/Variables.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace sicdm {

enum class FileKind { Project, Stl, Obj, Dxf, Svg, Step, Iges, Image, MeshModel, Unknown }; // MeshModel: 3MF, AMF, PLY, OFF, glTF, VRML

FileKind fileKindOf(const std::string& path);
const char* fileKindName(FileKind k);

// Document-level state saved alongside the scene.
// One model-building command in the feature history (REGEN replays them in order).
struct HistoryStep {
    std::string line;
    bool undoable = true;     // an UNDO takes it back
    bool millimetres = false; // typed by an MCP tool (bare numbers in mm)
    LengthUnit unit = LengthUnit::Millimeter; // the units its lengths were typed in
    // What was selected when it ran (a click selects without a command): commands that
    // default to the selection find the same body and curve again on replay.
    std::string selectedBody, selectedCurve;
    bool suppressed = false; // kept in the history but skipped when the model is rebuilt
};

struct ProjectState {
    SketchPlane plane = SketchPlane::Top;
    PlaneFrame customFrame = planeFrame(SketchPlane::Top); // the plane when `plane` is Custom
    std::string customPlane;          // its datum's name ("" for an unnamed one)
    std::vector<DatumPlane> datums;   // named datum planes (PLANE OFFSET, THROUGH...)
    std::vector<DatumAxis> refAxes;   // reference axes (REFAXIS)
    std::vector<DatumPoint> refPoints; // reference points (REFPOINT)
    std::vector<Variable> variables;  // global variables and equations (VAR), used as {expr}
    std::vector<Configuration> configs; // named sets of their values (CONFIG)
    std::string config;               // the configuration last applied
    UnitSettings units{};     // display / input units (geometry is always stored in mm)
    float simDuration = 0.0f; // Simulation timeline length (s); 0 = suggested from the motions
    CamJob cam;               // CAM: setup, tools and operations (toolpaths are remade)
    LatheJob lathe;           // lathe CAM: setup, tools and operations
    Drawing drawing;          // the drawing sheet: views, dimensions, title block
    AssemblyState assembly;   // mates between the parts
    RenderSettings render;    // RENDER: environment, lights, size, samples
    SectionView section;      // SECTION: the live section view of the 3D window
    ProductData product;      // properties, the revision and its state, the vault item (PROPERTY, REVISION, VAULT)
    std::vector<ToleranceStack> stacks; // tolerance stack-ups (STACK)
    std::vector<HistoryStep> history;     // the feature history
    std::vector<HistoryStep> historyRedo; // steps an UNDO took back (not saved)
    bool historyComplete = true;          // false: the model was also changed outside commands
    int rollback = -1;                    // >= 0: the model is built from only that many steps, and new ones go there
    std::uint64_t historyRevision = 0;    // the scene's revision after the last command (not saved)
};

// A reference axis or point by name (case-insensitive); nullptr when there is none.
const DatumAxis* findRefAxis(const ProjectState& st, const std::string& name);
const DatumPoint* findRefPoint(const ProjectState& st, const std::string& name);

// The active sketch plane's frame, and its name ("Top (XY)", a datum's name, "Custom").
PlaneFrame activePlaneFrame(const ProjectState& st);
std::string activePlaneName(const ProjectState& st);

// The project format: files carry "version" (the format that wrote them) and
// "reader_version" (the oldest format a reader must understand to open them). A newer
// file whose reader_version this build knows still opens; what it adds is left out.
//   1  the first format
//   2  adds exact geometry ("brep_step"), "reader_version" and "saved_by"
//   3  adds the drawing sheet ("drawing")
//   4  adds assemblies: each body's "assembly" path and the mates ("assembly")
//   5  large meshes (20000 vertices or more) as base64 binary arrays ("positions_b64"...),
//      written once and referred to by later bodies of the same shape ("mesh_same_as"); a
//      file with either has reader_version 5, others still 1
//   6  adds sketch arcs, splines, ellipses and slots, and construction curves (a reader
//      before 6 leaves those curves out)
//   7  adds drawing tolerances: dimension tolerances ("tol"), feature control frames
//      ("frames"), datums, surface finish marks and the general tolerance
//   8  adds the lathe job ("lathe")
//   9  adds robot rigs ("robots": morphology, base, tags) and the terrain ("terrain")
//   10 adds datum planes ("datums", "custom_plane") and sketches on them: curves on a
//      "custom" plane carry its "frame" (origin, u, v), and so do extrusions from them (in
//      their params); a file with either has reader_version 10. Bodies keep their hole
//      wizard callouts ("holes"), and the project its variables ("variables"),
//      configurations ("configs", "config") and reference axes and points
//      ("ref_axes", "ref_points")
//   11 adds 3D sketch curves: a polyline or spline whose points "rise" off its plane (one
//      height per point); a file with one has reader_version 11 (an older reader would lay
//      it flat). Bodies keep their weldment "member" and pipe "route" data
//   12 adds tolerance stack-ups ("stacks") and motions that ride on a body ("contact",
//      "spring", "preload", "mass"); a reader before 12 leaves them out
constexpr int kProjectVersion = 12;

// Writes a whole file safely: to a temporary file beside it, flushed to the disk, then
// renamed over it. A failure part way leaves any old file as it was.
bool writeFile(const std::string& path, const std::string& data, std::string* error);

bool saveProject(const Scene& scene, const ProjectState& state, const std::string& path, std::string* error);
// Replaces `scene`. Missing images are kept (without pixels) and reported in `warning`.
bool loadProject(Scene& scene, ProjectState& state, const std::string& path, std::string* error,
                 std::string* warning = nullptr);

// Visible bodies in world coordinates (offsets applied).
bool exportStl(const Scene& scene, const std::string& path, std::string* error);
bool exportObj(const Scene& scene, const std::string& path, std::string* error);

// Sketch curves on `plane`, in that plane's 2D coordinates (mm).
// Returns the number of entities written, or -1 on error.
int exportDxf(const Scene& scene, SketchPlane plane, const std::string& path, std::string* error,
              const PlaneFrame& custom = planeFrame(SketchPlane::Top)); // custom: the plane's frame when it is Custom
int exportSvg(const Scene& scene, SketchPlane plane, const std::string& path, std::string* error,
              const PlaneFrame& custom = planeFrame(SketchPlane::Top)); // custom: the plane's frame when it is Custom

// STL (ASCII or binary) or OBJ, as a "mesh" body named after the file.
std::optional<Body> importMesh(const std::string& path, std::string* error);

// LINE, CIRCLE, ARC, POINT, LWPOLYLINE and POLYLINE entities onto `plane`.
// Returns the number of curves added, or -1 on error.
int importDxf(Scene& scene, SketchPlane plane, const std::string& path, std::string* error,
              const PlaneFrame& custom = planeFrame(SketchPlane::Top)); // custom: the plane's frame when it is Custom

// Decodes the image at img.path into img.pixels (RGBA8, top row first).
bool loadImagePixels(ImageRef& img, std::string* error);

// New reference image sized to `width` mm (default: 1 px = 0.25 mm, max 400 mm), centred at `center`.
std::optional<ImageRef> importImage(const std::string& path, SketchPlane plane, Vec2 center, std::string* error);

const char* planeKey(SketchPlane p);
std::optional<SketchPlane> planeFromKey(const std::string& key);

} // namespace sicdm

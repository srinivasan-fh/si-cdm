#pragma once

// Project files and data exchange.
//
//   .sicdm      Project (JSON): parametric bodies, meshes, sketches, images
//   .stl .obj   3D meshes: import as a body, export visible bodies
//   .dxf        2D drawings: import onto / export from the active sketch plane
//   .svg        2D drawing export of the active sketch plane
//   images      .png .jpg .jpeg .bmp .tga .gif .psd .hdr .pnm: reference images

#include "core/Cam.hpp"
#include "core/Scene.hpp"
#include "core/Units.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace sicdm {

enum class FileKind { Project, Stl, Obj, Dxf, Svg, Image, Unknown };

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
};

struct ProjectState {
    SketchPlane plane = SketchPlane::Top;
    UnitSettings units{};     // display / input units (geometry is always stored in mm)
    float simDuration = 0.0f; // Simulation timeline length (s); 0 = suggested from the motions
    CamJob cam;               // CAM: setup, tools and operations (toolpaths are remade)
    std::vector<HistoryStep> history;     // the feature history
    std::vector<HistoryStep> historyRedo; // steps an UNDO took back (not saved)
    bool historyComplete = true;          // false: the model was also changed outside commands
    std::uint64_t historyRevision = 0;    // the scene's revision after the last command (not saved)
};

bool saveProject(const Scene& scene, const ProjectState& state, const std::string& path, std::string* error);
// Replaces `scene`. Missing images are kept (without pixels) and reported in `warning`.
bool loadProject(Scene& scene, ProjectState& state, const std::string& path, std::string* error,
                 std::string* warning = nullptr);

// Visible bodies in world coordinates (offsets applied).
bool exportStl(const Scene& scene, const std::string& path, std::string* error);
bool exportObj(const Scene& scene, const std::string& path, std::string* error);

// Sketch curves on `plane`, in that plane's 2D coordinates (mm).
// Returns the number of entities written, or -1 on error.
int exportDxf(const Scene& scene, SketchPlane plane, const std::string& path, std::string* error);
int exportSvg(const Scene& scene, SketchPlane plane, const std::string& path, std::string* error);

// STL (ASCII or binary) or OBJ, as a "mesh" body named after the file.
std::optional<Body> importMesh(const std::string& path, std::string* error);

// LINE, CIRCLE, ARC, POINT, LWPOLYLINE and POLYLINE entities onto `plane`.
// Returns the number of curves added, or -1 on error.
int importDxf(Scene& scene, SketchPlane plane, const std::string& path, std::string* error);

// Decodes the image at img.path into img.pixels (RGBA8, top row first).
bool loadImagePixels(ImageRef& img, std::string* error);

// New reference image sized to `width` mm (default: 1 px = 0.25 mm, max 400 mm), centred at `center`.
std::optional<ImageRef> importImage(const std::string& path, SketchPlane plane, Vec2 center, std::string* error);

const char* planeKey(SketchPlane p);
std::optional<SketchPlane> planeFromKey(const std::string& key);

} // namespace sicdm

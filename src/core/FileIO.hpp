#pragma once

// Project files and data exchange.
//
//   .sicdm      Project (JSON): parametric bodies, meshes, sketches, images
//   .stl .obj   3D meshes: import as a body, export visible bodies
//   .dxf        2D drawings: import onto / export from the active sketch plane
//   .svg        2D drawing export of the active sketch plane
//   images      .png .jpg .jpeg .bmp .tga .gif .psd .hdr .pnm: reference images

#include "core/Scene.hpp"

#include <optional>
#include <string>

namespace sicdm {

enum class FileKind { Project, Stl, Obj, Dxf, Svg, Image, Unknown };

FileKind fileKindOf(const std::string& path);
const char* fileKindName(FileKind k);

// Document-level state saved alongside the scene.
struct ProjectState {
    SketchPlane plane = SketchPlane::Top;
    float simDuration = 0.0f; // Simulation timeline length (s); 0 = suggested from the motions
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

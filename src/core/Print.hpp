#pragma once

// 3D-print preparation: check a part, repair it, orient it, and slice it.
//
//   Check     closed or not, non-manifold edges, inside-out shells, separate
//             shells, volume, area; overhangs that need support; walls thinner
//             than the nozzle can print; build-volume fit; material and time
//   Repair    welds seams, drops degenerate and duplicate faces, makes the winding
//             consistent and outward, fills holes, drops specks
//   Orient    the rotation with the least support (ties: the lowest part), placed
//             on the bed
//   3MF       the 3D Manufacturing Format package slicers read (units, objects)
//   Slice     layers, walls, rectilinear infill, solid top and bottom layers and
//             retraction, as Marlin-style G-code with time and filament estimates

#include "core/Geometry.hpp"
#include "core/Math.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace sicdm {

struct Body;
class Scene;

struct PrintSettings {
    float overhangAngle = 45.0f;    // degrees from vertical a face may lean out before it needs support
    float minWall = 0.8f;           // mm, thinnest wall the nozzle prints well
    Vec3 buildVolume{256, 256, 256};// mm
    // Slicing (FDM).
    float layerHeight = 0.2f;       // mm
    float lineWidth = 0.45f;        // mm
    float filament = 1.75f;         // mm diameter
    float infill = 0.2f;            // 0..1
    int walls = 2;
    int topLayers = 4, bottomLayers = 4;
    float nozzleTemp = 210.0f, bedTemp = 60.0f; // deg C
    float speed = 50.0f, firstLayerSpeed = 25.0f, travelSpeed = 150.0f; // mm/s
    float retract = 0.8f;           // mm of filament
    float density = 1.24f;          // g/cm^3 (PLA)
};

struct PrintReport {
    std::size_t triangles = 0;
    std::size_t openEdges = 0;        // edges with one face: holes
    std::size_t nonManifoldEdges = 0; // edges with three or more faces
    std::size_t shells = 0;           // separate pieces
    bool insideOut = false;           // faces point inwards
    double volume = 0.0, area = 0.0;  // mm^3, mm^2
    AABB box;
    bool fits = true;                 // inside the build volume
    double overhangArea = 0.0;        // mm^2 of downward faces off the bed that need support
    std::size_t wallSamples = 0, thinSamples = 0;
    float thinnest = 0.0f;            // mm, thinnest wall found (0: none measured)
    double grams = 0.0, filamentMetres = 0.0, minutes = 0.0; // estimates
    bool printable() const noexcept { return openEdges == 0 && nonManifoldEdges == 0 && !insideOut && fits; }
};
PrintReport analyzePrint(const Body& body, const PrintSettings& settings = {});

struct RepairReport {
    std::size_t degenerate = 0, duplicates = 0, flipped = 0, holesFilled = 0, specks = 0;
};
// Makes a body's mesh a clean closed solid where it can.
bool repairMesh(Body& body, RepairReport* report = nullptr, std::string* error = nullptr);

// Turns the body (about its origin) so that the least area needs support, and sets it
// down on the bed (lowest point at z = 0). Returns the support area after, in mm^2.
double orientForPrint(Body& body, const PrintSettings& settings = {}, double* before = nullptr);

// The visible bodies as a 3MF package (millimetres).
bool export3mf(const Scene& scene, const std::string& path, std::string* error = nullptr);

struct SliceReport {
    int layers = 0;
    double filamentMm = 0.0, seconds = 0.0;
    std::size_t moves = 0;
};
// Slices the body for an FDM printer, centred on the bed, to G-code.
bool sliceToGcode(const Body& body, const PrintSettings& settings, const std::string& path, SliceReport* report = nullptr,
                  std::string* error = nullptr);

} // namespace sicdm

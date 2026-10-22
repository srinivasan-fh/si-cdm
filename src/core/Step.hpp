#pragma once

// STEP (ISO 10303-21 files, AP203 / AP214 / AP242 geometry): exact B-rep solids and
// surface models in, exact B-reps (or meshes as planar-faced B-reps) out.
//
// Read: manifold solids (with voids), faceted solids and shell-based surface models;
// planes, cylinders, cones, spheres, tori, B-spline surfaces (rational or not), linear
// extrusions and revolutions; lines, circles, ellipses, B-spline curves, polylines (and
// the 3D curve of surface / seam curves and trimmed curves); length units (SI with any
// prefix, inch, foot...) and plane-angle units; assemblies (the placement of each part
// instance through the representation relationships); product names.
//
// Written: AP214 (automotive_design), millimetres, one product per body.

#include "core/Brep.hpp"

#include <memory>
#include <string>
#include <vector>

namespace sicdm {

struct StepPart {
    std::string name;
    std::shared_ptr<Brep> brep; // millimetres, placed where the assembly puts it
};

struct StepReadReport {
    std::size_t entities = 0;
    double unitToMm = 1.0;
    std::string schema;
    std::vector<std::string> warnings;
};

// Parts in the file (an empty list and `error` when it can't be read).
std::vector<StepPart> readStep(const std::string& text, StepReadReport* report = nullptr, std::string* error = nullptr);
std::vector<StepPart> readStepFile(const std::string& path, StepReadReport* report = nullptr, std::string* error = nullptr);

// One B-rep per part (in mm).
std::string writeStep(const std::vector<StepPart>& parts, const std::string& fileName = "model.step");
bool writeStepFile(const std::vector<StepPart>& parts, const std::string& path, std::string* error = nullptr);

// A triangle mesh as a B-rep of planar faces (coplanar neighbouring triangles merged).
// Closed meshes give a closed shell.
std::shared_ptr<Brep> meshToBrep(const MeshData& mesh, double planarTolerance = 1e-6);

// The exact B-rep of a primitive body kind (see buildPrimitive), before its rotation and
// scale; null for kinds without one.
std::shared_ptr<Brep> primitiveBrep(const std::string& kind, const std::vector<float>& params);

// Attaches the exact geometry to a primitive body (after its mesh is built), or drops it.
struct Body;
void attachPrimitiveBrep(Body& body);
// The body's exact geometry when it has some that still matches its mesh (a primitive's
// is rebuilt from its parameters if need be); null otherwise.
std::shared_ptr<const Brep> exactBrep(const Body& body);

} // namespace sicdm

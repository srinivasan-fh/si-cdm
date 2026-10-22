#pragma once

// IGES (5.3) files: exact surfaces and solids in, trimmed surfaces out.
//
// Read:
//   Solids      B-rep solids (186 manifold solid, 514 shell, 510 face, 508 loop, 504 edge
//               and 502 vertex lists), with voids
//   Surfaces    trimmed surfaces (144 with 142 boundaries), sewn into shells along the
//               edges they share; closed shells become solids, the rest surface models;
//               untrimmed surfaces on their own
//   Geometry    planes (108, 190), cylinders (192), cones (194), spheres (196), tori (198),
//               B-spline surfaces (128), surfaces of revolution (120), tabulated
//               cylinders (122); lines (110), circular arcs (100), elliptical arcs (104),
//               B-spline curves (126), polylines (106), composite curves (102);
//               transformation matrices (124) at any depth
//   Units       the global section's unit (inch, mm, m, ...), to millimetres
// Written: trimmed surfaces (144) on planes, surfaces of revolution, tabulated cylinders
// and B-spline surfaces, bounded by their model-space edges, in millimetres.

#include "core/Step.hpp"

#include <string>
#include <vector>

namespace sicdm {

// Parts in the file (an empty list and `error` when it can't be read). The report's
// schema is "IGES".
std::vector<StepPart> readIges(const std::string& text, StepReadReport* report = nullptr, std::string* error = nullptr);
std::vector<StepPart> readIgesFile(const std::string& path, StepReadReport* report = nullptr, std::string* error = nullptr);

std::string writeIges(const std::vector<StepPart>& parts, const std::string& fileName = "model.igs");
bool writeIgesFile(const std::vector<StepPart>& parts, const std::string& path, std::string* error = nullptr);

// Gives every face of each shell the same sense across the edges it shares with its
// neighbours, outward for closed shells (the solid's volume positive). Faces whose
// orientation the file left open (IGES trimmed surfaces) come out right this way.
void orientShells(Brep& brep);

} // namespace sicdm

#pragma once

// Airfoils and blades: NACA four-digit sections, and a blade lofted through them along its
// span with taper, twist, sweep and (whale-fin) tubercles on its leading edge.

#include "core/Geometry.hpp"

#include <memory>
#include <string>
#include <vector>

namespace sicdm {

struct Brep;

// A NACA four-digit section ("2412": 2% camber at 40% of the chord, 12% thick) of unit
// chord, leading edge at (0, 0): from the trailing edge over the top to the leading edge and
// back under it, counter-clockwise, the closed trailing edge once. `n` points a side
// (cosine spaced). False for a code that isn't four digits or makes no thickness.
bool nacaSection(const std::string& code, int n, std::vector<Vec2>& out, std::string* why = nullptr);

// A blade's parameters (Body::params of kind "blade"):
//   0 NACA code (as a number, e.g. 2412)   1 span (along +Z)   2 root chord   3 tip chord
//   4 twist at the tip (degrees, nose down +, linear from the root)   5 sweep of the leading
//   edge at the tip (mm, along +X)   6 tubercle amplitude (mm, 0: none)   7 tubercle wavelength (mm)
// The root section sits on z = 0 with its leading edge at x = 0; each section turns about its
// quarter chord.
std::string bladeCode(const std::vector<float>& params);
// A section of the blade at height z (closed loop, as nacaSection), in place.
std::vector<Vec3> bladeSection(const std::vector<float>& params, float z, int n);
// The closed blade mesh; empty for bad parameters.
MeshData buildBlade(const std::vector<float>& params);
// Its B-rep: a cubic B-spline skin through the sections with flat ends.
std::shared_ptr<Brep> bladeBrep(const std::vector<float>& params);

} // namespace sicdm

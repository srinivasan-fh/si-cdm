#pragma once

// ISO 286 limits and fits: the deviations a fit code such as H7 or g6 gives a size, and
// ISO 2768-1 general tolerances.
//
//   Holes   (capitals) D E F G H JS        grades 5 to 11
//   Shafts  (lower case) d e f g h js k m n p
//
// Sizes over 0 and up to 500 mm. Deviations come back in mm, upper over lower: H7 on
// 20 mm is +0.021 / 0, g6 on 20 mm is -0.007 / -0.020.

#include <string>

namespace sicdm {

struct FitDeviations {
    double upper = 0, lower = 0; // mm, from the nominal size
};

// The deviations of `code` ("H7", "g6", "js9"...) on a nominal `size` (mm). False with
// `why` for an unknown letter or grade, or a size out of range.
bool isoFit(const std::string& code, double size, FitDeviations& out, std::string* why = nullptr);

// The ISO 286 standard tolerance (IT grade 5..11) for a size, mm; 0 when out of range.
double itTolerance(int grade, double size);

// ISO 2768-1 general tolerance (+/-, mm) for a linear size in class f, m, c or v; 0 when
// the class doesn't cover it (f and v below 0.5 or 3 mm...).
double generalTolerance(char tolClass, double size);

} // namespace sicdm

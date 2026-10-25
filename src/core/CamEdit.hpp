#pragma once

// The options that turn one operation into another: what the CAM panel's Apply sends as
// CAM EDIT n ... / LATHE EDIT n ... after the user changed fields in its copy.

#include "core/Cam.hpp"
#include "core/Lathe.hpp"

#include <string>

namespace sicdm {

// " LEAVE 0.5 TOP AUTO ..." (with a leading space), or "" when nothing changed.
std::string camEditArgs(const CamOperation& from, const CamOperation& to);
std::string latheEditArgs(const LatheOperation& from, const LatheOperation& to);

} // namespace sicdm

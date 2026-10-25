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

// The CAM TOOL command that makes `t` (every option given, so it replaces the tool with
// that number whatever it was).
std::string camToolCommand(const CamTool& t);

// The CAM MACHINE command that sets the job's machine to `m` (every setting given).
std::string camMachineCommand(const CamMachine& m);

} // namespace sicdm

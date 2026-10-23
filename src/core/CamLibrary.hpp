#pragma once

// Tool libraries: the cutters a shop owns, kept apart from any one job, and read from or
// written to the files tool makers and other CAM systems use.
//
//   The user's library   one file per user (SICDM_TOOL_LIBRARY overrides where), that
//                        jobs take tools from (CAM LIBRARY USE) and add to (STORE)
//   Native JSON          {"format": "sicdm-tools", "tools": [...]}, the project's own form
//   CSV                  a header row naming the columns, in any order: number, type,
//                        diameter, flutes, flute_length, corner_radius, tip_angle, pitch,
//                        material (carbide / hss), name, vendor, product_id, vc, fz,
//                        stickout, holder_diameter, holder_length
//   Fusion 360 JSON      the tool library export ({"data": [...]}): type, geometry (DC,
//                        NOF, LCF, RE, SIG, TA, TP), BMC, vendor, product-id, post-process
//                        number, and start values (v_c, f_z, f_n); inch libraries are
//                        converted
//
// The vendor's cutting speed and chip load ride along and drive the feeds and speeds.

#include "core/Cam.hpp"

#include <string>
#include <vector>

namespace sicdm {

enum class ToolFileFormat { Auto, Native, Csv, Fusion };

// Where the user's library lives.
std::string toolLibraryPath();

// Reads tools from a file in any of the formats (Auto: by the extension and content).
// `notes` says what was left out (unknown tool types...).
bool loadToolFile(const std::string& path, std::vector<CamTool>& tools, std::string* error = nullptr, std::vector<std::string>* notes = nullptr,
                  ToolFileFormat format = ToolFileFormat::Auto);
// Writes tools (Auto: .csv as CSV, else native JSON).
bool saveToolFile(const std::string& path, const std::vector<CamTool>& tools, std::string* error = nullptr, ToolFileFormat format = ToolFileFormat::Auto);

std::string toolsToCsv(const std::vector<CamTool>& tools);
bool toolsFromCsv(const std::string& text, std::vector<CamTool>& tools, std::string* error, std::vector<std::string>* notes);
std::string toolsToFusion(const std::vector<CamTool>& tools);
bool toolsFromFusion(const std::string& text, std::vector<CamTool>& tools, std::string* error, std::vector<std::string>* notes);

} // namespace sicdm

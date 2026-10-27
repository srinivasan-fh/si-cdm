#pragma once

// An analysis report: what was analysed and how (material, cubes, fixings, loads), what came
// out (stress, displacement, factor of safety, natural frequencies, temperatures) and
// pictures of the results on the part, as one self-contained HTML page (prints to PDF from
// any browser).

#include "core/Fea.hpp"

#include <string>

namespace sicdm {

struct FeaReportData {
    std::string title;   // the project's name
    std::string body;    // the body analysed
    FeaSetup setup;
    FeaResult result;
    MeshData mesh;       // the world mesh analysed: the result's surface values are per vertex of it
};

std::string feaReportHtml(const FeaReportData& r);

// The result colour scale (0 blue .. 1 red), as the 3D view draws it.
Vec3 feaFieldColour(float t);

} // namespace sicdm

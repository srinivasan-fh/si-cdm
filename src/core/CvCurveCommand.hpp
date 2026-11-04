#pragma once

#include <string>
#include <vector>

namespace sicdm {

struct CommandOutput;
struct Document;

// CVCURVE x,y,z ... [DEGREE d] | FIT p ... [DEGREE d] [SPANS n] | FROM curve [DEGREE d] [SPANS n] |
// SET curve n x,y,z | MOVE curve n dx,dy,dz | INSERT curve t | REBUILD curve [DEGREE d] [SPANS n] |
// MATCH curve START|END TO other START|END [G0|G1|G2|G3] | CONTINUITY curve START|END other START|END |
// INFO [curve]: curves by control vertices (see CvCurve.hpp). `args` follow the word CVCURVE.
CommandOutput runCvCurveCommand(Document& doc, const std::vector<std::string>& args, const std::vector<bool>& quoted);

} // namespace sicdm

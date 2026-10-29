#pragma once

#include <string>
#include <vector>

namespace sicdm {

struct CommandOutput;
struct Document;

// PATH3D p1 p2 ... [SPLINE] [CLOSE] [BEND r] | POINT | INSERT | DELETE | BEND | INFO: 3D
// sketch curves. `args` follow the word PATH3D.
CommandOutput runPath3dCommand(Document& doc, const std::vector<std::string>& args, const std::vector<bool>& quoted);

// ROUTE [path] size [BEND r] [MATERIAL m] | LIST | BENDS | UPDATE | EDIT | SIZES (Route.hpp).
CommandOutput runRouteCommand(Document& doc, const std::vector<std::string>& args, const std::vector<bool>& quoted);

} // namespace sicdm

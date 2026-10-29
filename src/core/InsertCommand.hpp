#pragma once

#include <string>
#include <vector>

namespace sicdm {

struct CommandOutput;
struct Document;

// INSERTMESH [body] SPHERE|CUBE|CYLINDER|CONE|TORUS|BODY "name" AT x,y,z [SIZE s] [DIR x,y,z] [SPIN deg] [SINK f]
//            [UNION|APPEND|NEW]
// CURVE [body] TUBE PATH x,y,z x,y,z ... [RADIUS r] [SINK f] [UNION|APPEND|NEW]
// CURVE [body] INSERT shape|BODY "name" PATH ... [SIZE s] [SPACING d] [SINK f] [UNION|APPEND|NEW]
// (InsertBrush.hpp). `cmd` is INSERTMESH or CURVE; `args` follow it.
CommandOutput runInsertCommand(Document& doc, const std::string& cmd, const std::vector<std::string>& args, const std::vector<bool>& quoted);

} // namespace sicdm

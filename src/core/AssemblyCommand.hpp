#pragma once

#include <string>
#include <vector>

namespace sicdm {

struct CommandOutput;
struct Document;

// MATE type "A" AT x,y,z "B" AT x,y,z ... | MATE FIX | DELETE | SUPPRESS | EDIT | SOLVE, and
// ASSEMBLY (tree) | GROUP | UNGROUP | INTERFERENCE | BOM (see their entries in the command
// table). `args` follow the command word.
CommandOutput runAssemblyCommand(Document& doc, const std::string& cmd, const std::vector<std::string>& args, const std::vector<bool>& quoted);
// EXPLODE [LIST] | AUTO | ADD | EDIT | DELETE | CLEAR | SHOW | HIDE | TRAILS: the exploded view.
CommandOutput runExplodeCommand(Document& doc, const std::vector<std::string>& args, const std::vector<bool>& quoted);

} // namespace sicdm

#pragma once

#include <string>
#include <vector>

namespace sicdm {

struct CommandOutput;
struct Document;

// LATHE SETUP | TOOL | FACE | ROUGH | FINISH | GROOVE | DRILL | THREAD | PARTOFF | ...
// (see the LATHE entry in the command table). `args` follow the word LATHE.
CommandOutput runLatheCommand(Document& doc, const std::vector<std::string>& args, const std::vector<bool>& quoted);

} // namespace sicdm

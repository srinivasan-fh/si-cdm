#pragma once

#include <string>
#include <vector>

namespace sicdm {

struct CommandOutput;
struct Document;

// REVOLVE, SWEEP, LOFT, PATTERN, MIRROR, FILLET, MASSPROPS and MEASURE (see the
// command table); `args` follow the command's name.
CommandOutput runFeatureCommand(Document& doc, const std::string& cmd, const std::vector<std::string>& args, const std::vector<bool>& quoted);

// HISTORY [LIST] | HISTORY EDIT n line | HISTORY INSERT n line | HISTORY DELETE n | REGEN [FORCE]
CommandOutput runHistoryCommand(Document& doc, const std::string& cmd, const std::vector<std::string>& args, const std::vector<bool>& quoted);

} // namespace sicdm

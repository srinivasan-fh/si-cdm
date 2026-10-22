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

struct HistoryStep;

// Rebuilds the model from `steps` (they become the history). On a failure everything is
// left as it was and `out` says which step failed.
bool regenerateHistory(Document& doc, const std::vector<HistoryStep>& steps, CommandOutput& out);

// Arguments from `from` on, as a command line again (quoted ones re-quoted).
std::string rejoinArgs(const std::vector<std::string>& args, const std::vector<bool>& quoted, std::size_t from);

} // namespace sicdm

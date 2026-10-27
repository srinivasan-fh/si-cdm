#pragma once

#include <string>
#include <vector>

namespace sicdm {

struct CommandOutput;
struct Document;
class ViewHost;

// PROPERTY, REVISION, COMPARE and VAULT (see their entries in the command table). `args`
// follow the command word.
CommandOutput runPdmCommand(Document& doc, const std::string& cmd, const std::vector<std::string>& args, const std::vector<bool>& quoted, ViewHost* view);

// The vault folder: the preference, else $SICDM_VAULT ("" when neither is set).
std::string vaultFolder();

} // namespace sicdm

#pragma once

#include <string>
#include <vector>

namespace sicdm {

struct CommandOutput;
struct Document;

// CAM SETUP | TOOL | FACE | ROUGH | ... (see the CAM entry in the command table).
// `args` follow the word CAM; `quoted` marks the arguments typed in quotes.
CommandOutput runCamCommand(Document& doc, const std::vector<std::string>& args, const std::vector<bool>& quoted);

} // namespace sicdm

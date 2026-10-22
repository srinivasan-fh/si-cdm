#pragma once

#include <string>
#include <vector>

namespace sicdm {

struct CommandOutput;
struct Document;

// CONSTRAIN, DIM, CONSTRAINTS and CONSTRAINT (see the command table).
CommandOutput runSketchCommand(Document& doc, const std::string& cmd, const std::vector<std::string>& args, const std::vector<bool>& quoted);

} // namespace sicdm

#pragma once

#include <string>
#include <vector>

namespace sicdm {

struct CommandOutput;
struct Document;

// MOLD DRAFT | PARTING | SPLIT | OFF (Mold.hpp). `args` follow the word MOLD.
CommandOutput runMoldCommand(Document& doc, const std::vector<std::string>& args, const std::vector<bool>& quoted);

} // namespace sicdm

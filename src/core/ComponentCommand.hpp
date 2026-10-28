#pragma once

#include <string>
#include <vector>

namespace sicdm {

struct CommandOutput;
struct Document;
class ViewHost;

// COMPONENT INSERT | LIST | RELOAD | OPEN | REPLACE | MAKE | BREAK | MOVE | ROTATE | PACK (see
// its entry in the command table). `args` follow the word COMPONENT.
CommandOutput runComponentCommand(Document& doc, const std::vector<std::string>& args, const std::vector<bool>& quoted, ViewHost* view);

} // namespace sicdm

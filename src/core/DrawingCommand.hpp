#pragma once

#include <string>
#include <vector>

namespace sicdm {

struct CommandOutput;
struct Document;
class ViewHost;

// SHEET NEW | VIEW | SECTION | DIM | AUTODIM | TITLE | NOTE | SCALE | EXPORT | SHOW ...
// (see the SHEET entry in the command table). `args` follow the word SHEET.
CommandOutput runSheetCommand(Document& doc, const std::vector<std::string>& args, const std::vector<bool>& quoted, ViewHost* view);

} // namespace sicdm

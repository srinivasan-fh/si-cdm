#pragma once

#include <string>
#include <vector>

namespace sicdm {

struct CommandOutput;
struct Document;

// FACE INFO | OFFSET | DELETE | MOVE: direct editing of a body by its faces (DirectEdit.hpp).
// `args` follow the word FACE.
CommandOutput runFaceCommand(Document& doc, const std::vector<std::string>& args, const std::vector<bool>& quoted);

} // namespace sicdm

#pragma once

#include <string>
#include <vector>

namespace sicdm {

struct CommandOutput;
struct Document;
class Scene;

// WELDMENT PROFILES | MEMBER | JOINT | EDIT | LIST | CUTLIST (Weldment.hpp). `args` follow
// the word WELDMENT.
CommandOutput runWeldmentCommand(Document& doc, const std::vector<std::string>& args, const std::vector<bool>& quoted);

// After other edits: the members' joints found again, and their descriptions and part
// numbers kept up (a no-op without members).
void refreshWeldments(Scene& scene);

} // namespace sicdm

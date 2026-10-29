#pragma once

#include <string>
#include <vector>

namespace sicdm {

struct CommandOutput;
struct Document;

// STUDY [motion...] [POINT body [x,y,z]]... [FROM t0] [TO t1] [STEP dt | SAMPLES n] [SMOOTH s] [CSV file]
// [SVG file] [TITLE text] (Study.hpp). `args` follow the word STUDY.
CommandOutput runStudyCommand(Document& doc, const std::vector<std::string>& args, const std::vector<bool>& quoted);

} // namespace sicdm

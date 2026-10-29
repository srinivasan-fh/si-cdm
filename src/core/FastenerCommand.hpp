#pragma once

#include <string>
#include <vector>

namespace sicdm {

struct CommandOutput;
struct Document;

// FASTENER [LIST] | SIZES type | type size[xlength] [LENGTH l] [AT x,y,z] [DIR x,y,z] [NAME n] |
// INTO [body] [HOLE n,...] [type] [SIZE s] [LENGTH l] [GRIP g] [WASHER] [NUT] (Fastener.hpp).
// `args` follow the word FASTENER.
CommandOutput runFastenerCommand(Document& doc, const std::vector<std::string>& args, const std::vector<bool>& quoted);

} // namespace sicdm

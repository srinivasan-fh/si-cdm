#pragma once

#include <string>
#include <vector>

namespace sicdm {

struct CommandOutput;
struct Document;

// STACK: tolerance stack-ups (Stackup.hpp), kept with the project. `args` follow the word STACK.
//   STACK                                       the stacks, one line each
//   STACK NEW name [NOTE text] [SPEC lo hi | MIN lo | MAX hi] [GAP from to ALONG dir] [SAMPLES n]
//   STACK name ADD [label] nominal [tolerance] [options]
//   STACK name ADD BODY body ALONG dir [label] [tolerance] [options]     its size, measured
//   STACK name ADD DIM n [label] [options]                               a drawing dimension
//       tolerance: TOL t | TOL upper lower | LIMITS high low | FIT H7 | GENERAL f|m|c|v
//       options:   MINUS | PLUS, SENS k, NORMAL [CP c] | UNIFORM | TRIANGULAR
//   STACK name EDIT n [NAME label] [NOMINAL v] [tolerance] [options]
//   STACK name REMOVE n | SPEC ... | SPEC NONE | GAP ... | GAP NONE | NOTE text | UPDATE | RENAME new | DELETE
//   STACK name [REPORT] [SAMPLES n] [CSV file] [SVG file]
CommandOutput runStackCommand(Document& doc, const std::vector<std::string>& args, const std::vector<bool>& quoted);

} // namespace sicdm

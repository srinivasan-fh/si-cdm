#pragma once

#include "core/Math.hpp"

#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace sicdm {

// One command the command line understands.
struct CommandSpec {
    const char* name;    // canonical upper-case name, e.g. "CIRCLE"
    const char* aliases; // space-separated short forms, e.g. "C CIR"
    const char* usage;   // e.g. "CIRCLE cx,cy r"
    const char* help;
};

std::span<const CommandSpec> commandTable();

// Resolves a name or alias (case-insensitive) to its spec.
const CommandSpec* findCommand(std::string_view word);

// Commands whose name or alias starts with `prefix` (case-insensitive).
std::vector<const CommandSpec*> completeCommand(std::string_view prefix);

struct ParsedCommand {
    std::string name;              // upper-cased first word
    std::vector<std::string> args; // remaining words, as typed
    std::vector<bool> quoted;      // per argument: it was in quotes (a name, never a keyword)
};

// Splits a command line on whitespace. Empty name for a blank line. "Quoted words" are
// one argument; \" inside quotes is a literal quote.
ParsedCommand tokenizeCommand(std::string_view line);

// Coordinate entry, AutoCAD style:
//   "x,y"        absolute
//   "@dx,dy"     relative to the last point
//   "@len<deg"   polar, relative to the last point
//   "len<deg"    polar from the origin
struct PointInput {
    bool relative = false;
    bool polar = false;
    float a = 0, b = 0; // x,y or length,angle(deg)

    // Resolves against the previous point (used for relative input).
    Vec2 resolve(const Vec2& last) const noexcept;
};

std::optional<PointInput> parsePointInput(std::string_view text);

// "x,y,z" (or "x,y" with z = 0). Leading '@' is accepted and ignored.
std::optional<Vec3> parseVec3(std::string_view text);

std::optional<float> parseNumber(std::string_view text);

std::string toUpper(std::string_view s);

} // namespace sicdm

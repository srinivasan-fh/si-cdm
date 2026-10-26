#pragma once

// Global variables and equations: named values (VAR w = 60, VAR d = w / 2) that any command
// uses in braces: BOX {w} {d} 10, RECT 0,0 {w},{d / 2}. The feature history keeps the
// braces, so changing a variable and regenerating rebuilds the model from the new values.
// Configurations are named sets of values (CONFIG SAVE / CONFIG name); a design table is a
// CSV file of them.
//
// Expressions: numbers, variables, + - * / ^, parentheses, and the functions sin cos tan
// (degrees) asin acos atan (to degrees) sqrt abs floor ceil round min max pow, and pi.

#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace sicdm {

struct Variable {
    std::string name;
    std::string expr; // as typed: "60", "w / 2"
};

struct Configuration {
    std::string name;
    std::vector<std::pair<std::string, std::string>> values; // variable, expression
};

// A variable name: a letter or _, then letters, digits and _; not a function's name.
bool validVariableName(const std::string& name);
const Variable* findVariable(const std::vector<Variable>& vars, const std::string& name); // case-insensitive

// The value of an expression. Nullopt (and why) for bad syntax, unknown names, cycles
// (a = b + 1, b = a), division by zero and results that aren't finite.
std::optional<double> evalExpression(const std::string& text, const std::vector<Variable>& vars, std::string* error = nullptr);

// `line` with each {expression} outside quotes replaced by its value. False (and why) when
// one can't be worked out or a brace isn't closed. `used` (if given) gets the names read.
bool expandVariables(const std::string& line, const std::vector<Variable>& vars, std::string& out, std::string* error = nullptr,
                     std::vector<std::string>* used = nullptr);

// Reads a design table: a CSV file whose first row names the variables (its first cell is
// the configurations' column) and each further row is a configuration.
bool readDesignTable(const std::string& path, std::vector<Configuration>& out, std::string* error = nullptr);

} // namespace sicdm

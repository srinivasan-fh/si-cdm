#include "core/Variables.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <utility>

namespace sicdm {

namespace {

std::string lowerOf(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

const char* const kFunctions[] = {"sin", "cos", "tan", "asin", "acos", "atan", "sqrt", "abs", "floor", "ceil", "round", "min", "max", "pow", "pi"};

constexpr double kDeg = 3.14159265358979323846 / 180.0;

// Recursive descent over one expression.
class Parser {
public:
    Parser(std::string text, const std::vector<Variable>& vars, std::vector<std::string>& stack, std::vector<std::string>* used)
        : m_s(std::move(text)), m_vars(vars), m_stack(stack), m_used(used) {}

    std::optional<double> run(std::string& error) {
        auto v = expr();
        skip();
        if (v && m_i != m_s.size()) fail("Unexpected '" + m_s.substr(m_i, 1) + "' in " + m_s);
        if (!m_error.empty()) {
            error = m_error;
            return std::nullopt;
        }
        if (!v || !std::isfinite(*v)) {
            error = "The value of " + m_s + " isn't a finite number";
            return std::nullopt;
        }
        return v;
    }

private:
    void fail(std::string why) {
        if (m_error.empty()) m_error = std::move(why);
    }
    void skip() {
        while (m_i < m_s.size() && std::isspace(static_cast<unsigned char>(m_s[m_i]))) ++m_i;
    }
    bool eat(char c) {
        skip();
        if (m_i < m_s.size() && m_s[m_i] == c) return ++m_i, true;
        return false;
    }
    std::optional<double> expr() {
        auto v = term();
        while (v) {
            if (eat('+')) {
                auto r = term();
                if (!r) return std::nullopt;
                *v += *r;
            } else if (eat('-')) {
                auto r = term();
                if (!r) return std::nullopt;
                *v -= *r;
            } else break;
        }
        return v;
    }
    std::optional<double> term() {
        auto v = power();
        while (v) {
            if (eat('*')) {
                auto r = power();
                if (!r) return std::nullopt;
                *v *= *r;
            } else if (eat('/')) {
                auto r = power();
                if (!r) return std::nullopt;
                if (*r == 0.0) return fail("Division by zero in " + m_s), std::nullopt;
                *v /= *r;
            } else break;
        }
        return v;
    }
    std::optional<double> power() {
        auto v = unary();
        if (v && eat('^')) {
            auto r = power(); // right to left
            if (!r) return std::nullopt;
            return std::pow(*v, *r);
        }
        return v;
    }
    std::optional<double> unary() {
        if (eat('-')) {
            auto v = unary();
            return v ? std::optional<double>(-*v) : std::nullopt;
        }
        if (eat('+')) return unary();
        return primary();
    }
    std::optional<double> primary() {
        skip();
        if (m_i >= m_s.size()) return fail("Missing a value at the end of " + m_s), std::nullopt;
        const char c = m_s[m_i];
        if (c == '(') {
            ++m_i;
            auto v = expr();
            if (!eat(')')) return fail("Missing ')' in " + m_s), std::nullopt;
            return v;
        }
        if (std::isdigit(static_cast<unsigned char>(c)) || c == '.') {
            const char* start = m_s.c_str() + m_i;
            char* end = nullptr;
            const double v = std::strtod(start, &end);
            if (end == start) return fail("Not a number in " + m_s), std::nullopt;
            m_i += static_cast<std::size_t>(end - start);
            return v;
        }
        if (std::isalpha(static_cast<unsigned char>(c)) || c == '_') {
            std::size_t j = m_i;
            while (j < m_s.size() && (std::isalnum(static_cast<unsigned char>(m_s[j])) || m_s[j] == '_')) ++j;
            const std::string name = m_s.substr(m_i, j - m_i), key = lowerOf(name);
            m_i = j;
            if (key == "pi") return 3.14159265358979323846;
            if (std::find(std::begin(kFunctions), std::end(kFunctions), key) != std::end(kFunctions)) return call(key);
            return variable(name);
        }
        return fail("Unexpected '" + std::string(1, c) + "' in " + m_s), std::nullopt;
    }
    std::optional<double> call(const std::string& f) {
        if (!eat('(')) return fail(f + " needs ( ) round its value"), std::nullopt;
        std::vector<double> args;
        if (!eat(')')) {
            do {
                auto v = expr();
                if (!v) return std::nullopt;
                args.push_back(*v);
            } while (eat(','));
            if (!eat(')')) return fail("Missing ')' after " + f), std::nullopt;
        }
        const bool two = f == "min" || f == "max" || f == "pow";
        if (args.size() != (two ? 2u : 1u)) return fail(f + " takes " + (two ? "two values" : "one value")), std::nullopt;
        const double x = args[0];
        if (f == "sin") return std::sin(x * kDeg);
        if (f == "cos") return std::cos(x * kDeg);
        if (f == "tan") return std::tan(x * kDeg);
        if (f == "asin" || f == "acos") {
            if (x < -1 || x > 1) return fail(f + " takes -1 to 1"), std::nullopt;
            return (f == "asin" ? std::asin(x) : std::acos(x)) / kDeg;
        }
        if (f == "atan") return std::atan(x) / kDeg;
        if (f == "sqrt") {
            if (x < 0) return fail("sqrt of a negative number"), std::nullopt;
            return std::sqrt(x);
        }
        if (f == "abs") return std::fabs(x);
        if (f == "floor") return std::floor(x);
        if (f == "ceil") return std::ceil(x);
        if (f == "round") return std::round(x);
        if (f == "min") return std::min(x, args[1]);
        if (f == "max") return std::max(x, args[1]);
        return std::pow(x, args[1]);
    }
    std::optional<double> variable(const std::string& name) {
        const Variable* v = findVariable(m_vars, name);
        if (!v) return fail("No variable named " + name + " (VAR " + name + " = ... makes it)"), std::nullopt;
        if (m_used && std::find(m_used->begin(), m_used->end(), v->name) == m_used->end()) m_used->push_back(v->name);
        for (const std::string& s : m_stack)
            if (lowerOf(s) == lowerOf(v->name)) return fail("The variables go round in a circle through " + v->name), std::nullopt;
        if (m_stack.size() > 64) return fail("The variables refer to each other too deeply"), std::nullopt;
        m_stack.push_back(v->name);
        std::string e;
        Parser inner(v->expr, m_vars, m_stack, m_used);
        auto r = inner.run(e);
        m_stack.pop_back();
        if (!r) return fail(e), std::nullopt;
        return r;
    }

    std::string m_s;
    const std::vector<Variable>& m_vars;
    std::vector<std::string>& m_stack;
    std::vector<std::string>* m_used;
    std::size_t m_i = 0;
    std::string m_error;
};

std::string trim(const std::string& s) {
    std::size_t a = 0, b = s.size();
    while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
    return s.substr(a, b - a);
}

std::vector<std::string> csvRow(const std::string& line) {
    std::vector<std::string> cells;
    std::string cur;
    bool inQuotes = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];
        if (inQuotes) {
            if (c == '"' && i + 1 < line.size() && line[i + 1] == '"') cur += '"', ++i;
            else if (c == '"') inQuotes = false;
            else cur += c;
        } else if (c == '"') inQuotes = true;
        else if (c == ',' || c == ';' || c == '\t') cells.push_back(trim(cur)), cur.clear();
        else if (c != '\r') cur += c;
    }
    cells.push_back(trim(cur));
    return cells;
}

} // namespace

bool validVariableName(const std::string& name) {
    if (name.empty() || name.size() > 64) return false;
    if (!std::isalpha(static_cast<unsigned char>(name[0])) && name[0] != '_') return false;
    for (char c : name)
        if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_') return false;
    return std::find(std::begin(kFunctions), std::end(kFunctions), lowerOf(name)) == std::end(kFunctions);
}

const Variable* findVariable(const std::vector<Variable>& vars, const std::string& name) {
    const std::string k = lowerOf(name);
    for (const Variable& v : vars)
        if (lowerOf(v.name) == k) return &v;
    return nullptr;
}

std::optional<double> evalExpression(const std::string& text, const std::vector<Variable>& vars, std::string* error) {
    std::vector<std::string> stack;
    std::string e;
    Parser p(text, vars, stack, nullptr);
    auto v = p.run(e);
    if (!v && error) *error = e;
    return v;
}

bool expandVariables(const std::string& line, const std::vector<Variable>& vars, std::string& out, std::string* error, std::vector<std::string>* used) {
    out.clear();
    bool inQuotes = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];
        if (c == '"') inQuotes = !inQuotes;
        if (c != '{' || inQuotes) {
            out += c;
            continue;
        }
        const std::size_t close = line.find('}', i + 1);
        if (close == std::string::npos) {
            if (error) *error = "A '{' isn't closed with '}'";
            return false;
        }
        std::vector<std::string> stack;
        std::string e;
        Parser p(line.substr(i + 1, close - i - 1), vars, stack, used);
        const auto v = p.run(e);
        if (!v) {
            if (error) *error = e;
            return false;
        }
        char b[40];
        std::snprintf(b, sizeof b, "%.10g", *v);
        out += b;
        i = close;
    }
    return true;
}

bool readDesignTable(const std::string& path, std::vector<Configuration>& out, std::string* error) {
    auto fail = [&](std::string w) {
        if (error) *error = std::move(w);
        return false;
    };
    std::ifstream f(path);
    if (!f) return fail("Can't read " + path);
    std::string line;
    std::vector<std::string> head;
    out.clear();
    int row = 0;
    while (std::getline(f, line)) {
        ++row;
        if (trim(line).empty()) continue;
        std::vector<std::string> cells = csvRow(line);
        if (head.empty()) {
            head = cells;
            if (head.size() < 2) return fail("The first row names the configurations' column, then the variables");
            for (std::size_t k = 1; k < head.size(); ++k)
                if (!validVariableName(head[k])) return fail("\"" + head[k] + "\" in the first row isn't a variable name");
            continue;
        }
        if (cells[0].empty()) return fail("Row " + std::to_string(row) + " has no configuration name");
        Configuration c;
        c.name = cells[0];
        for (std::size_t k = 1; k < head.size() && k < cells.size(); ++k)
            if (!cells[k].empty()) c.values.push_back({head[k], cells[k]});
        out.push_back(std::move(c));
        if (out.size() > 1000) return fail("At most 1000 configurations");
    }
    if (head.empty() || out.empty()) return fail(path + " has no configurations");
    return true;
}

} // namespace sicdm

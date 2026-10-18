#include "core/Command.hpp"

#include <cctype>
#include <locale>
#include <sstream>
#include <cmath>

namespace sicdm {

namespace {

constexpr CommandSpec kCommands[] = {
    // Sketch
    {"LINE", "L", "LINE x1,y1 x2,y2", "Line on the active sketch plane (or pick the Line tool)"},
    {"PLINE", "PL", "PLINE x,y x,y ... [CLOSE]", "Polyline through the given points"},
    {"RECT", "REC RECTANG", "RECT x1,y1 x2,y2", "Rectangle from two opposite corners"},
    {"CIRCLE", "C", "CIRCLE cx,cy r", "Circle from centre and radius"},
    {"POINT", "PO", "POINT x,y", "Reference point"},
    // Solids
    {"BOX", "BLOCK", "BOX w d h [@x,y,z]", "Block centred on x,y with its base at z"},
    {"CYLINDER", "CYL", "CYLINDER r h [@x,y,z]", "Cylinder along +Z"},
    {"CONE", "", "CONE r h [@x,y,z]", "Cone along +Z"},
    {"SPHERE", "SPH", "SPHERE r [@x,y,z]", "Sphere resting on z"},
    {"TORUS", "TOR", "TORUS R r [@x,y,z]", "Torus around +Z"},
    {"WEDGE", "WE", "WEDGE w d h [@x,y,z]", "Right-angled wedge"},
    {"PYRAMID", "PYR", "PYRAMID r h [sides] [@x,y,z]", "Regular pyramid"},
    {"PRISM", "", "PRISM r h [sides] [@x,y,z]", "Regular prism (default hexagon)"},
    // Features
    {"EXTRUDE", "EXT", "EXTRUDE [curve] distance [SYMMETRIC] [KEEP]",
     "Extrude a closed sketch curve (selected or named) into a solid along its plane normal"},
    // Files
    {"NEW", "", "NEW", "Start an empty project"},
    {"OPEN", "", "OPEN path.sicdm", "Open a project"},
    {"SAVE", "", "SAVE [path.sicdm]", "Save the project (to a new path if given)"},
    {"SAVEAS", "", "SAVEAS path.sicdm", "Save the project under a new name"},
    {"IMPORT", "IMP", "IMPORT path [@x,y] [WIDTH mm]",
     "STL/OBJ as a body, DXF onto the sketch plane, PNG/JPG/... as a reference image"},
    {"EXPORT", "EXP", "EXPORT path.stl|.obj|.dxf|.svg", "STL/OBJ of visible bodies; DXF/SVG of the sketch plane"},
    // Editing
    {"SELECT", "SEL", "SELECT name | NONE", "Select a body, curve or image by name"},
    {"ERASE", "E DEL DELETE", "ERASE [name]", "Delete the selection (or the named item)"},
    {"MOVE", "M", "MOVE dx,dy[,dz] [name]", "Move the selected body or image"},
    {"IMAGE", "IMG", "IMAGE [name] WIDTH mm | OPACITY 0-1 | AT x,y", "Edit a reference image"},
    {"LIST", "LS", "LIST", "List bodies, sketch curves and images"},
    // Views & windows
    {"VIEW", "V", "VIEW TOP|FRONT|RIGHT|BACK|LEFT|BOTTOM|ISO", "Standard view (3D) or drawing plane (2D)"},
    {"2D", "DRAWING", "2D [TOP|FRONT|RIGHT]", "Open the 2D Drawing window"},
    {"3D", "MODEL", "3D", "Open the 3D Model window"},
    {"PLANE", "", "PLANE TOP|FRONT|RIGHT", "Active sketch plane"},
    {"ZOOM", "Z", "ZOOM E|IN|OUT", "Zoom extents (fit) or step in / out"},
    {"FIT", "ZE", "FIT", "Zoom to fit everything"},
    {"HOME", "", "HOME", "Home view: 3D isometric, fit all"},
    {"ROLL", "", "ROLL deg | ROLL 0", "Roll the 3D view; 0 levels it"},
    {"ORTHO", "", "ORTHO", "Orthographic projection (3D)"},
    {"PERSP", "PERSPECTIVE", "PERSP", "Perspective projection (3D)"},
    {"DISPLAY", "SHADE", "DISPLAY SHADED|EDGES|WIRE|HIDDEN", "Display mode of the 3D window"},
    {"GRID", "", "GRID [ON|OFF]", "Show or hide the grid"},
    {"SNAP", "", "SNAP [ON|OFF]", "Grid and point snapping for sketches"},
    {"PANEL", "", "PANEL [ON|OFF]", "Show or hide the Model panel"},
    // Session
    {"HELP", "? COMMANDS", "HELP [command]", "List commands or show one command's usage"},
    {"CLEAR", "CLS", "CLEAR", "Clear the command history"},
};

std::string_view trim(std::string_view s) {
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) s.remove_prefix(1);
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.remove_suffix(1);
    return s;
}

bool iequalsPrefix(std::string_view word, std::string_view prefix) {
    if (prefix.size() > word.size()) return false;
    for (std::size_t i = 0; i < prefix.size(); ++i)
        if (std::toupper(static_cast<unsigned char>(word[i])) != std::toupper(static_cast<unsigned char>(prefix[i])))
            return false;
    return true;
}

bool iequals(std::string_view a, std::string_view b) { return a.size() == b.size() && iequalsPrefix(a, b); }

// Calls f(alias) for each space-separated alias.
template <typename F>
void forEachAlias(const char* aliases, F&& f) {
    std::string_view s = aliases;
    while (!s.empty()) {
        const std::size_t sp = s.find(' ');
        f(s.substr(0, sp));
        if (sp == std::string_view::npos) break;
        s.remove_prefix(sp + 1);
    }
}

} // namespace

std::span<const CommandSpec> commandTable() { return kCommands; }

std::string toUpper(std::string_view s) {
    std::string out(s);
    for (char& c : out) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return out;
}

const CommandSpec* findCommand(std::string_view word) {
    word = trim(word);
    if (word.empty()) return nullptr;
    for (const CommandSpec& c : kCommands) {
        if (iequals(word, c.name)) return &c;
        bool hit = false;
        forEachAlias(c.aliases, [&](std::string_view a) { hit = hit || (!a.empty() && iequals(word, a)); });
        if (hit) return &c;
    }
    return nullptr;
}

std::vector<const CommandSpec*> completeCommand(std::string_view prefix) {
    std::vector<const CommandSpec*> out;
    prefix = trim(prefix);
    if (prefix.empty()) return out;
    for (const CommandSpec& c : kCommands) {
        bool hit = iequalsPrefix(c.name, prefix);
        forEachAlias(c.aliases, [&](std::string_view a) { hit = hit || (!a.empty() && iequalsPrefix(a, prefix)); });
        if (hit) out.push_back(&c);
    }
    return out;
}

ParsedCommand tokenizeCommand(std::string_view line) {
    ParsedCommand pc;
    line = trim(line);
    bool first = true;
    while (!line.empty()) {
        std::string word;
        std::size_t end = 0;
        if (line.front() == '"') { // quoted argument (paths with spaces)
            end = 1;
            while (end < line.size() && line[end] != '"') word += line[end++];
            if (end < line.size()) ++end;
        } else {
            while (end < line.size() && !std::isspace(static_cast<unsigned char>(line[end]))) ++end;
            word = std::string(line.substr(0, end));
        }
        if (first) pc.name = toUpper(word);
        else pc.args.push_back(std::move(word));
        first = false;
        line = trim(line.substr(end));
    }
    return pc;
}

std::optional<float> parseNumber(std::string_view text) {
    text = trim(text);
    if (text.empty()) return std::nullopt;
    if (text.front() == '+') text.remove_prefix(1);
    if (text.empty() || text.front() == '+' || text.front() == ' ') return std::nullopt;
    // Floating-point std::from_chars needs macOS 26; parse with the "C" locale instead.
    std::istringstream in{std::string(text)};
    in.imbue(std::locale::classic());
    float v = 0.0f;
    in >> v;
    if (in.fail() || in.peek() != std::char_traits<char>::eof() || !std::isfinite(v)) return std::nullopt;
    return v;
}

std::optional<PointInput> parsePointInput(std::string_view text) {
    text = trim(text);
    PointInput p;
    if (!text.empty() && text.front() == '@') {
        p.relative = true;
        text.remove_prefix(1);
    }
    const std::size_t lt = text.find('<');
    const std::size_t comma = text.find(',');
    if (lt != std::string_view::npos && comma == std::string_view::npos) {
        p.polar = true;
        const auto len = parseNumber(text.substr(0, lt));
        const auto ang = parseNumber(text.substr(lt + 1));
        if (!len || !ang) return std::nullopt;
        p.a = *len;
        p.b = *ang;
        return p;
    }
    if (comma == std::string_view::npos || text.find(',', comma + 1) != std::string_view::npos) return std::nullopt;
    const auto x = parseNumber(text.substr(0, comma));
    const auto y = parseNumber(text.substr(comma + 1));
    if (!x || !y) return std::nullopt;
    p.a = *x;
    p.b = *y;
    return p;
}

Vec2 PointInput::resolve(const Vec2& last) const noexcept {
    Vec2 d{a, b};
    if (polar) {
        const float r = radians(b);
        d = {a * std::cos(r), a * std::sin(r)};
    }
    return relative ? Vec2{last.x + d.x, last.y + d.y} : d;
}

std::optional<Vec3> parseVec3(std::string_view text) {
    text = trim(text);
    if (!text.empty() && text.front() == '@') text.remove_prefix(1);
    float v[3] = {0, 0, 0};
    int n = 0;
    while (true) {
        const std::size_t comma = text.find(',');
        const auto num = parseNumber(text.substr(0, comma));
        if (!num || n == 3) return std::nullopt;
        v[n++] = *num;
        if (comma == std::string_view::npos) break;
        text.remove_prefix(comma + 1);
    }
    if (n < 2) return std::nullopt;
    return Vec3{v[0], v[1], v[2]};
}

} // namespace sicdm

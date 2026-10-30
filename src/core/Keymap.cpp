// Keyboard shortcuts that can be changed (see Keymap.hpp).

#include "core/Keymap.hpp"

#include "core/Preferences.hpp"
#include "core/Translate.hpp"

#include <algorithm>
#include <cctype>

namespace sicdm {

namespace {

std::string upper(std::string s) {
    for (char& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

std::string trimmed(const std::string& s) {
    const std::size_t a = s.find_first_not_of(" \t"), b = s.find_last_not_of(" \t");
    return a == std::string::npos ? std::string() : s.substr(a, b - a + 1);
}

std::vector<std::pair<std::string, std::string>>& toolDefaults() {
    static std::vector<std::pair<std::string, std::string>> d;
    return d;
}

// The default for an id: from the table, else a tool's own.
std::string defaultChords(const std::string& id) {
    if (const KeyAction* a = findKeyAction(id)) return a->chords;
    for (const auto& [t, c] : toolDefaults())
        if (t == id) return c;
    return {};
}

} // namespace

const std::vector<std::string>& keyNames() {
    static const std::vector<std::string> names = [] {
        std::vector<std::string> n;
        for (char c = 'A'; c <= 'Z'; ++c) n.push_back(std::string(1, c));
        for (char c = '0'; c <= '9'; ++c) n.push_back(std::string(1, c));
        for (int k = 1; k <= 12; ++k) n.push_back("F" + std::to_string(k));
        for (char c = '0'; c <= '9'; ++c) n.push_back(std::string("Num") + c);
        for (const char* s : {"Escape", "Enter", "NumEnter", "Delete", "Backspace", "Insert", "Home", "End", "PageUp", "PageDown", "Left", "Right", "Up", "Down", "Tab", "Space",
                              ",", ".", "/", ";", "'", "-", "=", "[", "]", "\\", "`"})
            n.push_back(s);
        return n;
    }();
    return names;
}

std::optional<std::vector<KeyChord>> parseChords(const std::string& text) {
    std::vector<KeyChord> out;
    const std::string all = trimmed(text);
    if (all.empty() || upper(all) == "NONE") return out;
    // Chords are split at ", " (a lone "," is the comma key: "Ctrl+,").
    std::vector<std::string> parts;
    std::size_t from = 0;
    while (true) {
        const std::size_t at = all.find(", ", from);
        parts.push_back(trimmed(all.substr(from, at == std::string::npos ? std::string::npos : at - from)));
        if (at == std::string::npos) break;
        from = at + 2;
    }
    for (const std::string& part : parts) {
        if (part.empty()) return std::nullopt;
        KeyChord c;
        std::size_t start = 0;
        std::string key;
        while (true) {
            // The last '+' that isn't the key itself ("Ctrl++" would be "+": not a key we know).
            const std::size_t plus = part.find('+', start);
            if (plus == std::string::npos || plus + 1 >= part.size()) {
                key = part.substr(start);
                break;
            }
            const std::string mod = upper(trimmed(part.substr(start, plus - start)));
            if (mod == "CTRL" || mod == "CONTROL" || mod == "CMD" || mod == "COMMAND" || mod == "\xE2\x8C\x98") c.ctrl = true;
            else if (mod == "SHIFT") c.shift = true;
            else if (mod == "ALT" || mod == "OPTION" || mod == "OPT") c.alt = true;
            else return std::nullopt;
            start = plus + 1;
        }
        key = trimmed(key);
        // The name as keyNames() spells it.
        const std::string want = upper(key);
        const auto& names = keyNames();
        const auto it = std::find_if(names.begin(), names.end(), [&](const std::string& n) { return upper(n) == want; });
        if (it == names.end()) {
            if (want == "DEL") key = "Delete";
            else if (want == "ESC") key = "Escape";
            else if (want == "RETURN") key = "Enter";
            else if (want == "COMMA") key = ",";
            else if (want == "PERIOD") key = ".";
            else return std::nullopt;
        } else {
            key = *it;
        }
        c.key = key;
        out.push_back(c);
    }
    return out;
}

std::string chordText(const KeyChord& c) {
    std::string s;
    if (c.ctrl) s += "Ctrl+";
    if (c.alt) s += "Alt+";
    if (c.shift) s += "Shift+";
    return s + c.key;
}

std::string chordsText(const std::vector<KeyChord>& chords) {
    std::string s;
    for (const KeyChord& c : chords) s += (s.empty() ? "" : ", ") + chordText(c);
    return s.empty() ? "none" : s;
}

const std::vector<KeyAction>& keyActions() {
    static const std::vector<KeyAction> actions = {
        {"file.new", "Ctrl+N", SICDM_TR_NOOP("New file"), SICDM_TR_NOOP("File")},
        {"file.open", "Ctrl+O", SICDM_TR_NOOP("Open..."), SICDM_TR_NOOP("File")},
        {"file.import", "Ctrl+I", SICDM_TR_NOOP("Import..."), SICDM_TR_NOOP("File")},
        {"file.export", "Ctrl+E", SICDM_TR_NOOP("Export..."), SICDM_TR_NOOP("File")},
        {"file.save", "Ctrl+S", SICDM_TR_NOOP("Save"), SICDM_TR_NOOP("File")},
        {"file.save-as", "Ctrl+Shift+S", SICDM_TR_NOOP("Save As..."), SICDM_TR_NOOP("File")},
        {"file.close", "Ctrl+W", SICDM_TR_NOOP("Close the file tab"), SICDM_TR_NOOP("File")},
        {"file.next", "Ctrl+Tab", SICDM_TR_NOOP("Next file tab"), SICDM_TR_NOOP("File")},
        {"file.previous", "Ctrl+Shift+Tab", SICDM_TR_NOOP("Previous file tab"), SICDM_TR_NOOP("File")},
        {"file.preferences", "Ctrl+,", SICDM_TR_NOOP("Preferences"), SICDM_TR_NOOP("File")},
        {"file.quit", "Ctrl+Q", SICDM_TR_NOOP("Quit"), SICDM_TR_NOOP("File")},
        {"edit.undo", "Ctrl+Z", SICDM_TR_NOOP("Undo"), SICDM_TR_NOOP("Edit")},
        {"edit.redo", "Ctrl+Shift+Z, Ctrl+Y", SICDM_TR_NOOP("Redo"), SICDM_TR_NOOP("Edit")},
        {"edit.copy", "Ctrl+C", SICDM_TR_NOOP("Copy"), SICDM_TR_NOOP("Edit")},
        {"edit.cut", "Ctrl+X", SICDM_TR_NOOP("Cut"), SICDM_TR_NOOP("Edit")},
        {"edit.paste", "Ctrl+V", SICDM_TR_NOOP("Paste"), SICDM_TR_NOOP("Edit")},
        {"edit.duplicate", "Ctrl+D", SICDM_TR_NOOP("Duplicate"), SICDM_TR_NOOP("Edit")},
        {"edit.delete", "Delete, Backspace", SICDM_TR_NOOP("Delete"), SICDM_TR_NOOP("Edit")},
        {"edit.rename", "F2", SICDM_TR_NOOP("Rename"), SICDM_TR_NOOP("Edit")},
        {"edit.select-all", "Ctrl+A", SICDM_TR_NOOP("Select all"), SICDM_TR_NOOP("Edit")},
        {"edit.hide", "Alt+H", SICDM_TR_NOOP("Hide the selection"), SICDM_TR_NOOP("Edit")},
        {"edit.isolate", "Alt+I", SICDM_TR_NOOP("Isolate the selection"), SICDM_TR_NOOP("Edit")},
        {"edit.show-all", "Alt+Shift+H", SICDM_TR_NOOP("Show every body"), SICDM_TR_NOOP("Edit")},
        {"edit.cancel", "Escape", SICDM_TR_NOOP("Cancel, or clear the selection"), SICDM_TR_NOOP("Edit")},
        {"edit.finish", "Enter, NumEnter", SICDM_TR_NOOP("Finish (a polyline, an extrusion's depth, a brush size)"), SICDM_TR_NOOP("Edit")},
        {"commandline.focus", "Space", SICDM_TR_NOOP("Type a command"), SICDM_TR_NOOP("Edit"), "model"},
        {"view.fit", "F", SICDM_TR_NOOP("Zoom to fit"), SICDM_TR_NOOP("View")},
        {"view.front", "1, Num1", SICDM_TR_NOOP("Front view"), SICDM_TR_NOOP("View")},
        {"view.right", "3, Num3", SICDM_TR_NOOP("Right view"), SICDM_TR_NOOP("View")},
        {"view.top", "7, Num7", SICDM_TR_NOOP("Top view"), SICDM_TR_NOOP("View")},
        {"view.back", "Ctrl+1, Ctrl+Num1", SICDM_TR_NOOP("Back view"), SICDM_TR_NOOP("View")},
        {"view.left", "Ctrl+3, Ctrl+Num3", SICDM_TR_NOOP("Left view"), SICDM_TR_NOOP("View")},
        {"view.bottom", "Ctrl+7, Ctrl+Num7", SICDM_TR_NOOP("Bottom view"), SICDM_TR_NOOP("View")},
        {"view.iso", "0, Num0", SICDM_TR_NOOP("Isometric view"), SICDM_TR_NOOP("View")},
        {"view.projection", "P", SICDM_TR_NOOP("Perspective / orthographic"), SICDM_TR_NOOP("View")},
        {"view.grid", "G", SICDM_TR_NOOP("Grid on / off"), SICDM_TR_NOOP("View")},
        {"view.display", "W", SICDM_TR_NOOP("Next display mode"), SICDM_TR_NOOP("View")},
        {"view.windows", "Tab", SICDM_TR_NOOP("2D Drawing / 3D Model window"), SICDM_TR_NOOP("View")},
        {"view.simulation", "Shift+Tab", SICDM_TR_NOOP("Simulation window"), SICDM_TR_NOOP("View")},
        {"panel.model", "M", SICDM_TR_NOOP("Model panel"), SICDM_TR_NOOP("Panels")},
        {"panel.library", "K", SICDM_TR_NOOP("Library panel"), SICDM_TR_NOOP("Panels")},
        {"panel.cam", "J", SICDM_TR_NOOP("CAM panel"), SICDM_TR_NOOP("Panels")},
        {"help.help", "F1", SICDM_TR_NOOP("Help"), SICDM_TR_NOOP("Help")},
        {"sim.play", "Space", SICDM_TR_NOOP("Play / pause"), SICDM_TR_NOOP("Simulation"), "sim"},
        {"sim.rewind", "Home", SICDM_TR_NOOP("Back to the start"), SICDM_TR_NOOP("Simulation"), "sim"},
        {"sim.back", "Left", SICDM_TR_NOOP("A frame back"), SICDM_TR_NOOP("Simulation"), "sim"},
        {"sim.forward", "Right", SICDM_TR_NOOP("A frame on"), SICDM_TR_NOOP("Simulation"), "sim"},
        {"brush.smaller", "[", SICDM_TR_NOOP("Smaller brush"), SICDM_TR_NOOP("Sculpting"), "sculpt"},
        {"brush.larger", "]", SICDM_TR_NOOP("Larger brush"), SICDM_TR_NOOP("Sculpting"), "sculpt"},
        {"brush.weaker", "Shift+[", SICDM_TR_NOOP("Weaker brush"), SICDM_TR_NOOP("Sculpting"), "sculpt"},
        {"brush.stronger", "Shift+]", SICDM_TR_NOOP("Stronger brush"), SICDM_TR_NOOP("Sculpting"), "sculpt"},
    };
    return actions;
}

const KeyAction* findKeyAction(const std::string& id) {
    for (const KeyAction& a : keyActions())
        if (id == a.id) return &a;
    return nullptr;
}

bool isToolKeyId(const std::string& id) { return id.rfind("tool.", 0) == 0 && id.size() > 5; }

std::string toolKeyId(const std::string& name) {
    std::string s = "tool.";
    bool dash = false;
    for (const char c : name) {
        if (std::isalnum(static_cast<unsigned char>(c))) {
            if (dash && s.size() > 5) s += '-';
            s += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            dash = false;
        } else {
            dash = true;
        }
    }
    return s;
}

void setToolKeyDefaults(std::vector<std::pair<std::string, std::string>> d) { toolDefaults() = std::move(d); }
const std::vector<std::pair<std::string, std::string>>& toolKeyDefaults() { return toolDefaults(); }

std::vector<KeyChord> boundChords(const Preferences& p, const std::string& id) {
    const auto it = p.keys.find(id);
    const auto chords = parseChords(it != p.keys.end() ? it->second : defaultChords(id));
    return chords.value_or(std::vector<KeyChord>{});
}

bool keyContextsApart(const std::string& a, const std::string& b) {
    auto ctx = [](const std::string& id) {
        const KeyAction* k = findKeyAction(id);
        return std::string(k ? k->context : "");
    };
    const std::string x = ctx(a), y = ctx(b);
    // "model" and "sim" never overlap; "sculpt" is outside the Simulation window, so it
    // overlaps "model" but not "sim".
    auto apart = [](const std::string& p, const std::string& q) { return (p == "sim" && (q == "model" || q == "sculpt")); };
    return !x.empty() && !y.empty() && x != y && (apart(x, y) || apart(y, x));
}

std::vector<std::string> actionsOnChord(const Preferences& p, const KeyChord& chord, const std::string& except) {
    std::vector<std::string> ids;
    for (const KeyAction& a : keyActions()) ids.push_back(a.id);
    for (const auto& [t, c] : toolDefaults()) ids.push_back(t);
    for (const auto& [id, c] : p.keys)
        if (std::find(ids.begin(), ids.end(), id) == ids.end()) ids.push_back(id);
    std::vector<std::string> out;
    for (const std::string& id : ids) {
        if (id == except || (!except.empty() && keyContextsApart(id, except))) continue;
        for (const KeyChord& c : boundChords(p, id))
            if (c == chord) {
                out.push_back(id);
                break;
            }
    }
    return out;
}

bool setKeyBinding(Preferences& p, const std::string& id, const std::string& text, bool force, std::string* why) {
    auto fail = [&](const std::string& w) {
        if (why) *why = w;
        return false;
    };
    if (!findKeyAction(id) && !isToolKeyId(id)) return fail("No action " + id + " (KEYS lists them)");
    const auto chords = parseChords(text);
    if (!chords)
        return fail("Not keys: " + text + " (a key with Ctrl, Shift, Alt: F2, Ctrl+Shift+S, Alt+H, \"Delete, Backspace\"; Esc and Enter stay as they are)");
    if (!force)
        for (const KeyChord& c : *chords)
            if (const auto others = actionsOnChord(p, c, id); !others.empty())
                return fail(chordText(c) + " is " + others[0] + "'s already (KEYS SET " + id + " ... FORCE to share it, or give " + others[0] + " another)");
    const std::string now = chordsText(*chords);
    if (now == chordsText(parseChords(defaultChords(id)).value_or(std::vector<KeyChord>{}))) p.keys.erase(id);
    else p.keys[id] = now;
    return true;
}

void resetKeyBinding(Preferences& p, const std::string& id) {
    if (id.empty()) p.keys.clear();
    else p.keys.erase(id);
}

std::vector<std::string> describeKeyBindings(const Preferences& p) {
    std::vector<std::string> out;
    std::string group;
    for (const KeyAction& a : keyActions()) {
        if (group != a.group) out.push_back(std::string(a.group) + ":"), group = a.group;
        const bool own = p.keys.count(a.id) > 0;
        out.push_back("  " + std::string(a.id) + "  " + chordsText(boundChords(p, a.id)) + "  (" + a.label + ")" + (own ? "  [changed]" : ""));
    }
    bool tools = false;
    std::vector<std::string> ids;
    for (const auto& [t, c] : toolDefaults()) ids.push_back(t);
    for (const auto& [id, c] : p.keys)
        if (isToolKeyId(id) && std::find(ids.begin(), ids.end(), id) == ids.end()) ids.push_back(id);
    for (const std::string& id : ids) {
        const std::vector<KeyChord> c = boundChords(p, id);
        if (c.empty() && !p.keys.count(id)) continue;
        if (!tools) out.push_back("Tools:"), tools = true;
        out.push_back("  " + id + "  " + chordsText(c) + (p.keys.count(id) ? "  [changed]" : ""));
    }
    return out;
}

} // namespace sicdm

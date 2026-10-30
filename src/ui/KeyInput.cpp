// Keyboard shortcuts as key presses (see KeyInput.hpp).

#include "ui/KeyInput.hpp"

#include "core/Preferences.hpp"
#include "ui/Tools.hpp"

#include <imgui.h>

#include <cstdlib>
#include <utility>
#include <vector>

namespace sicdm {

namespace {

ImGuiKey imguiKey(const std::string& name) {
    if (name.size() == 1) {
        const char c = name[0];
        if (c >= 'A' && c <= 'Z') return static_cast<ImGuiKey>(ImGuiKey_A + (c - 'A'));
        if (c >= '0' && c <= '9') return static_cast<ImGuiKey>(ImGuiKey_0 + (c - '0'));
        switch (c) {
        case ',': return ImGuiKey_Comma;
        case '.': return ImGuiKey_Period;
        case '/': return ImGuiKey_Slash;
        case ';': return ImGuiKey_Semicolon;
        case '\'': return ImGuiKey_Apostrophe;
        case '-': return ImGuiKey_Minus;
        case '=': return ImGuiKey_Equal;
        case '[': return ImGuiKey_LeftBracket;
        case ']': return ImGuiKey_RightBracket;
        case '\\': return ImGuiKey_Backslash;
        case '`': return ImGuiKey_GraveAccent;
        default: return ImGuiKey_None;
        }
    }
    if (name.size() >= 2 && name[0] == 'F' && name[1] >= '1' && name[1] <= '9') {
        const int n = std::atoi(name.c_str() + 1);
        if (n >= 1 && n <= 12) return static_cast<ImGuiKey>(ImGuiKey_F1 + (n - 1));
    }
    if (name.size() == 4 && name.rfind("Num", 0) == 0 && name[3] >= '0' && name[3] <= '9') return static_cast<ImGuiKey>(ImGuiKey_Keypad0 + (name[3] - '0'));
    static const std::pair<const char*, ImGuiKey> kNamed[] = {
        {"Escape", ImGuiKey_Escape}, {"Enter", ImGuiKey_Enter}, {"NumEnter", ImGuiKey_KeypadEnter},
        {"Delete", ImGuiKey_Delete}, {"Backspace", ImGuiKey_Backspace}, {"Insert", ImGuiKey_Insert}, {"Home", ImGuiKey_Home},
        {"End", ImGuiKey_End},       {"PageUp", ImGuiKey_PageUp},       {"PageDown", ImGuiKey_PageDown}, {"Left", ImGuiKey_LeftArrow},
        {"Right", ImGuiKey_RightArrow}, {"Up", ImGuiKey_UpArrow},       {"Down", ImGuiKey_DownArrow},    {"Tab", ImGuiKey_Tab},
        {"Space", ImGuiKey_Space}};
    for (const auto& [n, k] : kNamed)
        if (name == n) return k;
    return ImGuiKey_None;
}

bool g_capture = false;

} // namespace

void setKeyCapture(bool on) { g_capture = on; }
bool keyCapture() { return g_capture; }

bool chordPressed(const KeyChord& c, bool repeat) {
    if (g_capture) return false;
    const ImGuiKey k = imguiKey(c.key);
    if (k == ImGuiKey_None || !ImGui::IsKeyPressed(k, repeat)) return false;
    const ImGuiIO& io = ImGui::GetIO();
    return commandModifierHeld() == c.ctrl && io.KeyShift == c.shift && io.KeyAlt == c.alt;
}

bool keyActionPressed(const std::string& id, bool repeat, bool whileTyping) {
    const bool typing = ImGui::GetIO().WantTextInput && !whileTyping;
    for (const KeyChord& c : boundChords(preferences(), id)) {
        if (typing && !c.ctrl && !c.alt) continue; // a plain key is typed into the field
        if (chordPressed(c, repeat)) return true;
    }
    return false;
}

void registerToolKeys() {
    std::vector<std::pair<std::string, std::string>> d;
    for (int i = 0; i < static_cast<int>(ToolId::Count); ++i) {
        const ToolInfo& t = toolInfo(static_cast<ToolId>(i));
        d.emplace_back(toolKeyId(t.name), t.shortcut ? t.shortcut : "");
    }
    setToolKeyDefaults(std::move(d));
}

std::optional<KeyChord> pressedChord() {
    const ImGuiIO& io = ImGui::GetIO();
    for (const std::string& name : keyNames()) {
        const ImGuiKey k = imguiKey(name);
        if (k != ImGuiKey_None && ImGui::IsKeyPressed(k, false)) return KeyChord{name, commandModifierHeld(), io.KeyShift, io.KeyAlt};
    }
    return std::nullopt;
}

} // namespace sicdm

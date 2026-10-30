#pragma once

// Keyboard shortcuts that can be changed: each action (a menu item, a view key, a tool) has
// default keys and may have the user's own instead, kept with the preferences. A binding is
// one or two key chords, "Ctrl+Shift+Z, Ctrl+Y": a key with the modifiers that must be held
// with it, exactly (Ctrl is Cmd on a Mac). The interface turns them into key presses; this
// holds the names, the defaults and what the user changed (KEYS).

#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace sicdm {

struct Preferences;

struct KeyChord {
    std::string key; // a key name as keyNames() has it: "A", "F2", "Delete", "Num1", "[" ...
    bool ctrl = false, shift = false, alt = false;
    bool operator==(const KeyChord& o) const { return key == o.key && ctrl == o.ctrl && shift == o.shift && alt == o.alt; }
};

// Every key name a chord can use.
const std::vector<std::string>& keyNames();
// "Ctrl+Shift+Z, Ctrl+Y" (any case; Cmd for Ctrl, Option for Alt; "none" or "": no keys).
// Nothing for a key name it doesn't know.
std::optional<std::vector<KeyChord>> parseChords(const std::string& text);
std::string chordText(const KeyChord& c);
std::string chordsText(const std::vector<KeyChord>& chords);

struct KeyAction {
    const char* id;       // "edit.copy"
    const char* chords;   // its default keys
    const char* label;    // what it does
    const char* group;    // File, Edit, View, Panels ...
};
// The actions other than the tools (whose ids are "tool." and the tool's name, lower case,
// words joined by '-', with the tool's own key as the default).
const std::vector<KeyAction>& keyActions();
const KeyAction* findKeyAction(const std::string& id);
bool isToolKeyId(const std::string& id);
std::string toolKeyId(const std::string& toolName);

// The tools' default keys, from the interface (so conflicts with them can be found).
void setToolKeyDefaults(std::vector<std::pair<std::string, std::string>> idAndChords);
const std::vector<std::pair<std::string, std::string>>& toolKeyDefaults();

// What an action is bound to: the user's keys, else its default (`fallback` for a tool).
std::vector<KeyChord> boundChords(const Preferences& p, const std::string& id);
// The other actions bound to a chord (`except`: not this one).
std::vector<std::string> actionsOnChord(const Preferences& p, const KeyChord& chord, const std::string& except = {});

// KEYS: list, set, reset (and saves the preferences). Lines for the output; false and why.
bool setKeyBinding(Preferences& p, const std::string& id, const std::string& chords, bool force, std::string* why);
void resetKeyBinding(Preferences& p, const std::string& id); // "" for all
std::vector<std::string> describeKeyBindings(const Preferences& p);

} // namespace sicdm

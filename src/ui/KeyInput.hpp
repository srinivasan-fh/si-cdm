#pragma once

// Keyboard shortcuts as key presses: whether an action's keys (Keymap.hpp, as the user set
// them) were pressed this frame, with exactly the modifiers they name held.

#include "core/Keymap.hpp"

#include <optional>
#include <string>

namespace sicdm {

// Any of the action's chords pressed now. `repeat`: also while the key is held down.
bool keyActionPressed(const std::string& id, bool repeat = false);
bool chordPressed(const KeyChord& chord, bool repeat = false);
// The tools' own keys, for the keymap's defaults (once, at start).
void registerToolKeys();
// A chord pressed this frame (not a modifier, Esc or Enter alone): for setting a shortcut.
std::optional<KeyChord> pressedChord();
// While a shortcut is being set, no shortcut acts (the keys pressed are the new one).
void setKeyCapture(bool on);
bool keyCapture();

} // namespace sicdm

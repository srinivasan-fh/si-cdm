#pragma once

// Which mouse button (with Shift or Ctrl) orbits, pans or zooms the view: the MOUSE
// preference. Presets follow the programs people come from, so their hands don't have to
// learn again. The left button always belongs to the tool; the wheel always zooms.

#include <string>
#include <vector>

namespace sicdm {

enum class MouseButton { Left, Right, Middle };
enum class ViewDrag { None, Orbit, Pan, Zoom };

struct MouseSchemeInfo {
    std::string key;         // as typed: PREFERENCES SET mouse solidworks
    std::string name;        // as shown
    std::string description; // the buttons, in words
};

const std::vector<MouseSchemeInfo>& mouseSchemes();
bool isMouseScheme(const std::string& key);

// What dragging `button` (Shift, Ctrl held as pressed) does in `scheme`; in the 2D window
// (`flat`) an orbit pans, as the view never tilts there.
ViewDrag mouseViewDrag(const std::string& scheme, MouseButton button, bool shift, bool ctrl, bool flat);

} // namespace sicdm

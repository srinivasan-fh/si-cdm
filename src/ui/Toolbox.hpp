#pragma once

#include "ui/Tools.hpp"

#include <vector>

namespace sicdm {

// Photoshop-style tool rail docked to the left edge of the canvas.
// Shows one or two columns of slots; a slot with several tools has a corner
// triangle and opens a flyout on right-click or long press.
class Toolbox {
public:
    Toolbox();

    // Draws the rail between `top` and `bottom` (screen y). Returns its width.
    float draw(float top, float bottom);

    ToolId active() const noexcept { return m_active; }
    void setActive(ToolId id);

    // Keyboard shortcut handling; Shift+key cycles through a slot's tools.
    void handleShortcuts();

private:
    void drawSlot(int groupIndex, float slotSize);
    void drawFlyout(int groupIndex);
    int groupOf(ToolId id) const;

    ToolId m_active = ToolId::Orbit;
    std::vector<int> m_current; // per group: index of the tool shown in the slot
    bool m_twoColumns = true;
    int m_pressedGroup = -1;
    double m_pressTime = 0.0;
};

} // namespace sicdm

#pragma once

#include "ui/Tools.hpp"

#include <imgui.h>

namespace sicdm {

// Draws a vector icon for `id` centred at `center`, fitting a `size` square.
void drawToolIcon(ImDrawList* dl, ToolId id, ImVec2 center, float size, ImU32 color);

} // namespace sicdm

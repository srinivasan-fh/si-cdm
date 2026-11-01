#pragma once

// Reads a 3D mouse (SpaceMouse) where the system offers it: Linux through spacenavd (its
// socket, $SICDM_SPNAV_SOCKET or /var/run/spnav.sock), Windows through Raw Input (the HID
// multi-axis controller), macOS through IOKit's HID manager. No 3Dconnexion driver or SDK
// is needed by the app itself. core/SpaceMouse.hpp turns what comes in into view moves.

#include "core/SpaceMouse.hpp"

#include <vector>

struct GLFWwindow;

namespace sicdm {

struct SpaceMouseInput {
    SixDof motion;            // the cap's deflection now (all zero at rest)
    std::vector<int> pressed; // buttons pressed since the last poll (0 the first)
    bool connected = false;   // a device (or spacenavd) is there
};

void spaceMouseAttach(GLFWwindow* window);
void spaceMouseDetach();
SpaceMouseInput spaceMousePoll();

} // namespace sicdm

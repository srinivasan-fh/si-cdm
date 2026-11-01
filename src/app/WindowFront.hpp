#pragma once

// Brings the main window to the front with the keyboard focus once the splash closes.
// glfwFocusWindow alone isn't always enough: Windows hands the foreground to another
// program when the splash (the window that had it) goes, and only flashes the taskbar
// button; macOS may leave the app behind the one it was started from.

struct GLFWwindow;

namespace sicdm {

void bringWindowToFront(GLFWwindow* window);

} // namespace sicdm

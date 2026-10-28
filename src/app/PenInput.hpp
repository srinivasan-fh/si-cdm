#pragma once

// A drawing tablet's pen pressure, which GLFW doesn't report. Windows: the pointer messages
// (WM_POINTER*, Windows 8 and later). macOS: the tablet data on mouse events (PenInput_mac.mm).
// Linux (X11): the "Abs Pressure" valuator of the tablet's XInput2 devices, read each frame.
// Elsewhere (no tablet, Wayland), there is none and brushes ignore pressure.

struct GLFWwindow;

namespace sicdm {

void penInputAttach(GLFWwindow* window);
void penInputDetach();
// Once a frame: reads what needs reading (X11).
void penInputPoll();
// 0..1 while a pen is down (0 just lifted); negative when the pointer isn't a pen (a mouse,
// a touchpad) or there's no tablet.
float penPressure();

} // namespace sicdm

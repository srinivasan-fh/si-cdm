// macOS: a tablet's pen moves the mouse, and its mouse events carry the pressure (subtype
// tablet point); a local event monitor reads it before GLFW sees the event.

#include "app/PenInput.hpp"

#import <Cocoa/Cocoa.h>

namespace sicdm {

namespace {
id g_monitor = nil;
float g_pressure = -1.0f;
} // namespace

void penInputAttach(GLFWwindow*) {
    const NSEventMask mask = NSEventMaskLeftMouseDown | NSEventMaskLeftMouseDragged | NSEventMaskLeftMouseUp | NSEventMaskMouseMoved | NSEventMaskTabletPoint;
    g_monitor = [NSEvent addLocalMonitorForEventsMatchingMask:mask
                                                      handler:^NSEvent*(NSEvent* e) {
                                                          const bool tablet = e.type == NSEventTypeTabletPoint ||
                                                                              ((e.type == NSEventTypeLeftMouseDown || e.type == NSEventTypeLeftMouseDragged ||
                                                                                e.type == NSEventTypeLeftMouseUp || e.type == NSEventTypeMouseMoved) &&
                                                                               e.subtype == NSEventSubtypeTabletPoint);
                                                          if (!tablet) g_pressure = -1.0f;
                                                          else g_pressure = e.type == NSEventTypeLeftMouseUp ? 0.0f : static_cast<float>(e.pressure);
                                                          return e;
                                                      }];
}

void penInputDetach() {
    if (g_monitor) [NSEvent removeMonitor:g_monitor];
    g_monitor = nil;
}

void penInputPoll() {}
float penPressure() { return g_pressure; }

} // namespace sicdm

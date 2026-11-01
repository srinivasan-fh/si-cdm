#include "app/WindowFront.hpp"

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#define GLFW_EXPOSE_NATIVE_WIN32
#elif !defined(__APPLE__)
#include <dlfcn.h>
#include <X11/Xlib.h>
#define GLFW_EXPOSE_NATIVE_X11
#endif

#include <GLFW/glfw3.h>
#if !defined(__APPLE__)
#include <GLFW/glfw3native.h>
#endif

#include <cstring>

namespace sicdm {

#if defined(__APPLE__)
void bringAppToFrontMac(); // WindowFront_mac.mm
#endif

void bringWindowToFront(GLFWwindow* window) {
    if (!window) return;
    if (glfwGetWindowAttrib(window, GLFW_ICONIFIED)) glfwRestoreWindow(window);
    glfwShowWindow(window);
#if defined(_WIN32)
    if (HWND hwnd = glfwGetWin32Window(window)) {
        // The foreground lock only lets the program the user is working with take the
        // foreground: borrow that thread's input state for the moment of the switch.
        const HWND front = GetForegroundWindow();
        const DWORD frontThread = front ? GetWindowThreadProcessId(front, nullptr) : 0;
        const DWORD ours = GetCurrentThreadId();
        const bool attached = frontThread && frontThread != ours && AttachThreadInput(frontThread, ours, TRUE);
        // Top-most for a moment puts it over everything, then back to an ordinary window.
        SetWindowPos(hwnd, HWND_TOPMOST, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE | SWP_SHOWWINDOW);
        SetWindowPos(hwnd, HWND_NOTOPMOST, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE | SWP_SHOWWINDOW);
        BringWindowToTop(hwnd);
        SetForegroundWindow(hwnd);
        SetActiveWindow(hwnd);
        SetFocus(hwnd);
        if (attached) AttachThreadInput(frontThread, ours, FALSE);
        FlashWindow(hwnd, FALSE); // stops a taskbar flash left from an earlier refusal
    }
#elif defined(__APPLE__)
    bringAppToFrontMac();
#else
    if (glfwGetPlatform() == GLFW_PLATFORM_X11) {
        // Xlib is looked up at run time, as GLFW itself does, so the app doesn't link it.
        using InternAtom = Atom (*)(Display*, const char*, Bool);
        using SendEvent = Status (*)(Display*, Window, Bool, long, XEvent*);
        using WindowFn = int (*)(Display*, Window);
        using FlushFn = int (*)(Display*);
        void* x11 = dlopen("libX11.so.6", RTLD_LAZY | RTLD_LOCAL);
        const auto internAtom = x11 ? reinterpret_cast<InternAtom>(dlsym(x11, "XInternAtom")) : nullptr;
        const auto sendEvent = x11 ? reinterpret_cast<SendEvent>(dlsym(x11, "XSendEvent")) : nullptr;
        const auto raise = x11 ? reinterpret_cast<WindowFn>(dlsym(x11, "XRaiseWindow")) : nullptr;
        const auto flush = x11 ? reinterpret_cast<FlushFn>(dlsym(x11, "XFlush")) : nullptr;
        Display* dpy = glfwGetX11Display();
        const Window w = glfwGetX11Window(window);
        if (dpy && w && internAtom && sendEvent && raise && flush) {
            // _NET_ACTIVE_WINDOW from a "pager" (source 2): window managers that hold back
            // focus requests from programs still honour it.
            XEvent e;
            std::memset(&e, 0, sizeof e);
            e.xclient.type = ClientMessage;
            e.xclient.window = w;
            e.xclient.message_type = internAtom(dpy, "_NET_ACTIVE_WINDOW", False);
            e.xclient.format = 32;
            e.xclient.data.l[0] = 2;
            e.xclient.data.l[1] = CurrentTime;
            sendEvent(dpy, DefaultRootWindow(dpy), False, SubstructureRedirectMask | SubstructureNotifyMask, &e);
            raise(dpy, w);
            flush(dpy);
        }
        if (x11) dlclose(x11);
    }
#endif
    glfwFocusWindow(window);
}

} // namespace sicdm

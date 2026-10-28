#include "app/PenInput.hpp"

#if defined(_WIN32)
#ifndef WINVER
#define WINVER 0x0602
#endif
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0602
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#define GLFW_EXPOSE_NATIVE_WIN32
#elif !defined(__APPLE__)
#include <dlfcn.h>
#include <X11/Xlib.h>
#include <X11/extensions/XInput2.h>
#define GLFW_EXPOSE_NATIVE_X11
#endif

#if !defined(__APPLE__)
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>
#endif

#include <algorithm>
#include <cstring>
#include <vector>

namespace sicdm {

#if defined(_WIN32)

namespace {

WNDPROC g_previous = nullptr;
HWND g_window = nullptr;
float g_pressure = -1.0f;

// Mouse messages Windows makes up for a pen or touch carry this signature.
bool fromPen() { return (static_cast<DWORD>(GetMessageExtraInfo()) & 0xFFFFFF00u) == 0xFF515700u; }

LRESULT CALLBACK penProc(HWND h, UINT m, WPARAM w, LPARAM l) {
    switch (m) {
    case WM_POINTERDOWN:
    case WM_POINTERUPDATE:
    case WM_POINTERUP: {
        const UINT32 id = GET_POINTERID_WPARAM(w);
        POINTER_INPUT_TYPE type = PT_POINTER;
        if (GetPointerType(id, &type) && type == PT_PEN) {
            POINTER_PEN_INFO info{};
            if (m == WM_POINTERUP) g_pressure = 0.0f;
            else if (GetPointerPenInfo(id, &info)) g_pressure = (info.penMask & PEN_MASK_PRESSURE) ? static_cast<float>(info.pressure) / 1024.0f : 1.0f;
        } else {
            g_pressure = -1.0f;
        }
        break;
    }
    case WM_LBUTTONDOWN:
    case WM_MOUSEMOVE:
        if (!fromPen()) g_pressure = -1.0f; // a real mouse
        break;
    default: break;
    }
    return CallWindowProcW(g_previous, h, m, w, l);
}

} // namespace

void penInputAttach(GLFWwindow* window) {
    g_window = glfwGetWin32Window(window);
    if (!g_window) return;
    g_previous = reinterpret_cast<WNDPROC>(SetWindowLongPtrW(g_window, GWLP_WNDPROC, reinterpret_cast<LONG_PTR>(&penProc)));
}

void penInputDetach() {
    if (g_window && g_previous) SetWindowLongPtrW(g_window, GWLP_WNDPROC, reinterpret_cast<LONG_PTR>(g_previous));
    g_window = nullptr, g_previous = nullptr;
}

void penInputPoll() {}
float penPressure() { return g_pressure; }

#elif !defined(__APPLE__)

namespace {

// libXi and libX11 are opened at run time (as GLFW does): no new link dependency, and
// nothing breaks where they're missing.
using QueryDevice = XIDeviceInfo* (*)(Display*, int, int*);
using FreeDeviceInfo = void (*)(XIDeviceInfo*);
using GetAtomName = char* (*)(Display*, Atom);
using XFreeFn = int (*)(void*);

struct Pen {
    int device = 0, valuator = 0;
    double lo = 0, hi = 1;
};

struct State {
    void* xi = nullptr;
    void* x11 = nullptr;
    QueryDevice query = nullptr;
    FreeDeviceInfo freeInfo = nullptr;
    Display* display = nullptr;
    std::vector<Pen> pens;
    float pressure = -1.0f;
} g;

} // namespace

void penInputAttach(GLFWwindow*) {
    if (glfwGetPlatform() != GLFW_PLATFORM_X11) return;
    g.display = glfwGetX11Display();
    g.xi = dlopen("libXi.so.6", RTLD_LAZY | RTLD_LOCAL);
    g.x11 = dlopen("libX11.so.6", RTLD_LAZY | RTLD_LOCAL);
    if (!g.display || !g.xi || !g.x11) return;
    g.query = reinterpret_cast<QueryDevice>(dlsym(g.xi, "XIQueryDevice"));
    g.freeInfo = reinterpret_cast<FreeDeviceInfo>(dlsym(g.xi, "XIFreeDeviceInfo"));
    const auto atomName = reinterpret_cast<GetAtomName>(dlsym(g.x11, "XGetAtomName"));
    const auto xfree = reinterpret_cast<XFreeFn>(dlsym(g.x11, "XFree"));
    if (!g.query || !g.freeInfo || !atomName || !xfree) return;
    // The pointers with a pressure axis: the tablet's pens and erasers.
    int n = 0;
    XIDeviceInfo* devices = g.query(g.display, XIAllDevices, &n);
    for (int i = 0; devices && i < n; ++i) {
        const XIDeviceInfo& d = devices[i];
        if (d.use != XISlavePointer) continue;
        for (int c = 0; c < d.num_classes; ++c) {
            if (d.classes[c]->type != XIValuatorClass) continue;
            const auto* v = reinterpret_cast<const XIValuatorClassInfo*>(d.classes[c]);
            if (!v->label) continue;
            char* name = atomName(g.display, v->label);
            if (name && std::strcmp(name, "Abs Pressure") == 0 && v->max > v->min) g.pens.push_back({d.deviceid, v->number, v->min, v->max});
            if (name) xfree(name);
        }
    }
    if (devices) g.freeInfo(devices);
}

void penInputDetach() {
    g.pens.clear();
    if (g.xi) dlclose(g.xi);
    if (g.x11) dlclose(g.x11);
    g = State{};
}

void penInputPoll() {
    if (g.pens.empty()) return;
    // The valuators' current values: the pen pressing hardest is the one in use. One that
    // isn't touching reads 0, so a mouse stroke (no pen down) gets no pressure.
    float best = -1.0f;
    for (const Pen& p : g.pens) {
        int n = 0;
        XIDeviceInfo* d = g.query(g.display, p.device, &n);
        if (!d) continue;
        for (int c = 0; c < d->num_classes; ++c) {
            if (d->classes[c]->type != XIValuatorClass) continue;
            const auto* v = reinterpret_cast<const XIValuatorClassInfo*>(d->classes[c]);
            if (v->number == p.valuator) best = std::max(best, static_cast<float>(std::clamp((v->value - p.lo) / (p.hi - p.lo), 0.0, 1.0)));
        }
        g.freeInfo(d);
    }
    g.pressure = best > 0.0f ? best : -1.0f;
}

float penPressure() { return g.pressure; }

#endif

} // namespace sicdm

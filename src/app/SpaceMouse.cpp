#include "app/SpaceMouse.hpp"

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>
#elif defined(__APPLE__)
#include <GLFW/glfw3.h>
#include <IOKit/hid/IOHIDManager.h>
#else
#include <GLFW/glfw3.h>
#include <cerrno>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#endif

#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace sicdm {

namespace {

// What has come in, shared with the reader (a thread on Linux, the window on Windows, the
// run loop on macOS).
std::mutex g_lock;
SixDof g_motion;
std::vector<int> g_pressed;
std::atomic<bool> g_connected{false};

void pressed(int button) {
    std::lock_guard<std::mutex> l(g_lock);
    g_pressed.push_back(button);
}

} // namespace

#if defined(_WIN32)

namespace {

HWND g_hwnd = nullptr;
WNDPROC g_previous = nullptr;
HidState g_hid;

LRESULT CALLBACK spaceProc(HWND h, UINT m, WPARAM w, LPARAM l) {
    if (m == WM_INPUT) {
        UINT size = 0;
        GetRawInputData(reinterpret_cast<HRAWINPUT>(l), RID_INPUT, nullptr, &size, sizeof(RAWINPUTHEADER));
        std::vector<std::uint8_t> buf(size);
        if (size && GetRawInputData(reinterpret_cast<HRAWINPUT>(l), RID_INPUT, buf.data(), &size, sizeof(RAWINPUTHEADER)) == size) {
            const auto* ri = reinterpret_cast<const RAWINPUT*>(buf.data());
            if (ri->header.dwType == RIM_TYPEHID) {
                const DWORD n = ri->data.hid.dwSizeHid;
                for (DWORD k = 0; k < ri->data.hid.dwCount; ++k) {
                    const std::uint8_t* report = ri->data.hid.bRawData + static_cast<std::size_t>(k) * n;
                    const std::uint32_t before = g_hid.buttons;
                    if (!decodeHidReport({report, n}, g_hid)) continue;
                    g_connected = true;
                    for (int b = 0; b < 32; ++b)
                        if ((g_hid.buttons & (1u << b)) && !(before & (1u << b))) pressed(b);
                    std::lock_guard<std::mutex> lk(g_lock);
                    g_motion = g_hid.motion;
                }
            }
        }
    }
    return CallWindowProcW(g_previous, h, m, w, l);
}

} // namespace

void spaceMouseAttach(GLFWwindow* window) {
    g_hwnd = glfwGetWin32Window(window);
    if (!g_hwnd) return;
    RAWINPUTDEVICE dev{};
    dev.usUsagePage = 0x01; // generic desktop
    dev.usUsage = 0x08;     // multi-axis controller
    dev.dwFlags = 0;        // while the window has the focus
    dev.hwndTarget = g_hwnd;
    if (!RegisterRawInputDevices(&dev, 1, sizeof(dev))) return;
    g_previous = reinterpret_cast<WNDPROC>(SetWindowLongPtrW(g_hwnd, GWLP_WNDPROC, reinterpret_cast<LONG_PTR>(&spaceProc)));
    // Is one plugged in? (Reports only come while it is used.)
    UINT count = 0;
    GetRawInputDeviceList(nullptr, &count, sizeof(RAWINPUTDEVICELIST));
    std::vector<RAWINPUTDEVICELIST> list(count);
    if (count && GetRawInputDeviceList(list.data(), &count, sizeof(RAWINPUTDEVICELIST)) != static_cast<UINT>(-1))
        for (const RAWINPUTDEVICELIST& d : list) {
            if (d.dwType != RIM_TYPEHID) continue;
            RID_DEVICE_INFO info{};
            info.cbSize = sizeof(info);
            UINT sz = sizeof(info);
            if (GetRawInputDeviceInfoW(d.hDevice, RIDI_DEVICEINFO, &info, &sz) != static_cast<UINT>(-1) && info.hid.usUsagePage == 1 &&
                info.hid.usUsage == 8)
                g_connected = true;
        }
}

void spaceMouseDetach() {
    if (g_hwnd && g_previous) SetWindowLongPtrW(g_hwnd, GWLP_WNDPROC, reinterpret_cast<LONG_PTR>(g_previous));
    g_hwnd = nullptr, g_previous = nullptr;
}

#elif defined(__APPLE__)

namespace {

IOHIDManagerRef g_manager = nullptr;
HidState g_hid;

void onValue(void*, IOReturn, void*, IOHIDValueRef value) {
    IOHIDElementRef el = IOHIDValueGetElement(value);
    const std::uint32_t page = IOHIDElementGetUsagePage(el), usage = IOHIDElementGetUsage(el);
    const float v = static_cast<float>(IOHIDValueGetIntegerValue(value));
    g_connected = true;
    if (page == 0x01) {
        // HID: X right, Y toward the user, Z down; spacenavd: x right, y up, z toward the user.
        std::lock_guard<std::mutex> l(g_lock);
        switch (usage) {
        case 0x30: g_hid.motion.tx = v; break;
        case 0x31: g_hid.motion.tz = v; break;
        case 0x32: g_hid.motion.ty = -v; break;
        case 0x33: g_hid.motion.rx = v; break;
        case 0x34: g_hid.motion.rz = v; break;
        case 0x35: g_hid.motion.ry = -v; break;
        default: return;
        }
        g_motion = g_hid.motion;
    } else if (page == 0x09 && usage >= 1 && usage <= 32) {
        if (v != 0) pressed(static_cast<int>(usage) - 1);
    }
    glfwPostEmptyEvent(); // wake the idle loop
}

CFDictionaryRef matching(int page, int usage) {
    CFMutableDictionaryRef d = CFDictionaryCreateMutable(kCFAllocatorDefault, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
    CFNumberRef p = CFNumberCreate(kCFAllocatorDefault, kCFNumberIntType, &page);
    CFNumberRef u = CFNumberCreate(kCFAllocatorDefault, kCFNumberIntType, &usage);
    CFDictionarySetValue(d, CFSTR(kIOHIDDeviceUsagePageKey), p);
    CFDictionarySetValue(d, CFSTR(kIOHIDDeviceUsageKey), u);
    CFRelease(p);
    CFRelease(u);
    return d;
}

void onDevice(void*, IOReturn, void*, IOHIDDeviceRef) { g_connected = true; }

} // namespace

void spaceMouseAttach(GLFWwindow*) {
    g_manager = IOHIDManagerCreate(kCFAllocatorDefault, kIOHIDOptionsTypeNone);
    if (!g_manager) return;
    CFDictionaryRef m = matching(0x01, 0x08); // multi-axis controller
    IOHIDManagerSetDeviceMatching(g_manager, m);
    CFRelease(m);
    IOHIDManagerRegisterInputValueCallback(g_manager, onValue, nullptr);
    IOHIDManagerRegisterDeviceMatchingCallback(g_manager, onDevice, nullptr);
    IOHIDManagerScheduleWithRunLoop(g_manager, CFRunLoopGetMain(), kCFRunLoopDefaultMode);
    if (IOHIDManagerOpen(g_manager, kIOHIDOptionsTypeNone) != kIOReturnSuccess) {
        CFRelease(g_manager);
        g_manager = nullptr;
    }
}

void spaceMouseDetach() {
    if (!g_manager) return;
    IOHIDManagerUnscheduleFromRunLoop(g_manager, CFRunLoopGetMain(), kCFRunLoopDefaultMode);
    IOHIDManagerClose(g_manager, kIOHIDOptionsTypeNone);
    CFRelease(g_manager);
    g_manager = nullptr;
}

#else

namespace {

std::thread g_reader;
std::atomic<bool> g_stop{false};
int g_wake[2] = {-1, -1}; // a pipe that stops the reader

std::string socketPath() {
    if (const char* e = std::getenv("SICDM_SPNAV_SOCKET"); e && *e) return e;
    return "/var/run/spnav.sock";
}

// Connects, reads events until the daemon goes, then tries again every few seconds.
void readLoop() {
    while (!g_stop) {
        const int fd = socket(AF_UNIX, SOCK_STREAM, 0);
        if (fd < 0) return;
        sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        const std::string path = socketPath();
        std::strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);
        if (connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0) {
            g_connected = true;
            std::int32_t words[8];
            std::size_t have = 0;
            while (!g_stop) {
                pollfd p[2] = {{fd, POLLIN, 0}, {g_wake[0], POLLIN, 0}};
                if (poll(p, 2, -1) < 0 && errno != EINTR) break;
                if (p[1].revents) break;
                if (!(p[0].revents & (POLLIN | POLLHUP | POLLERR))) continue;
                const ssize_t n = read(fd, reinterpret_cast<char*>(words) + have, sizeof(words) - have);
                if (n <= 0) break;
                have += static_cast<std::size_t>(n);
                if (have < sizeof(words)) continue;
                have = 0;
                if (const auto e = decodeSpnav(std::span<const std::int32_t, 8>(words, 8))) {
                    if (e->kind == SpaceMouseEvent::Motion) {
                        std::lock_guard<std::mutex> l(g_lock);
                        g_motion = e->motion;
                    } else if (e->kind == SpaceMouseEvent::Press) {
                        pressed(e->button);
                    }
                    glfwPostEmptyEvent(); // wake the idle loop
                }
            }
            g_connected = false;
            std::lock_guard<std::mutex> l(g_lock);
            g_motion = {};
        }
        close(fd);
        // Not running (or gone): look again in a while.
        pollfd p{g_wake[0], POLLIN, 0};
        if (poll(&p, 1, 3000) > 0) break;
    }
}

} // namespace

void spaceMouseAttach(GLFWwindow*) {
    if (g_reader.joinable() || pipe(g_wake) != 0) return;
    g_stop = false;
    g_reader = std::thread(readLoop);
}

void spaceMouseDetach() {
    if (!g_reader.joinable()) return;
    g_stop = true;
    const char c = 1;
    [[maybe_unused]] const ssize_t w = write(g_wake[1], &c, 1);
    g_reader.join();
    close(g_wake[0]);
    close(g_wake[1]);
    g_wake[0] = g_wake[1] = -1;
}

#endif

SpaceMouseInput spaceMousePoll() {
    SpaceMouseInput in;
    std::lock_guard<std::mutex> l(g_lock);
    in.motion = g_motion;
    in.pressed.swap(g_pressed);
    in.connected = g_connected;
    return in;
}

} // namespace sicdm

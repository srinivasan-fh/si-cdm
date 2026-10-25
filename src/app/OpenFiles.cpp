#include "app/OpenFiles.hpp"

#include <mutex>

#if defined(__APPLE__)
#include <objc/message.h>
#include <objc/runtime.h>
#endif

namespace sicdm {

namespace {

std::mutex g_mutex;
std::vector<std::string> g_requests;

#if defined(__APPLE__)
// -[NSApplicationDelegate application:openURLs:], added to GLFW's delegate class through the
// Objective-C runtime (so no Objective-C source is needed). `urls` is an NSArray of NSURL.
void openURLs(id, SEL, id, id urls) {
    using Count = unsigned long (*)(id, SEL);
    using At = id (*)(id, SEL, unsigned long);
    using Get = id (*)(id, SEL);
    using Utf8 = const char* (*)(id, SEL);
    const unsigned long n = reinterpret_cast<Count>(objc_msgSend)(urls, sel_registerName("count"));
    std::lock_guard<std::mutex> lock(g_mutex);
    for (unsigned long i = 0; i < n; ++i) {
        id url = reinterpret_cast<At>(objc_msgSend)(urls, sel_registerName("objectAtIndex:"), i);
        id path = url ? reinterpret_cast<Get>(objc_msgSend)(url, sel_registerName("path")) : nullptr;
        const char* s = path ? reinterpret_cast<Utf8>(objc_msgSend)(path, sel_registerName("UTF8String")) : nullptr;
        if (s && *s) g_requests.emplace_back(s);
    }
}
#endif

} // namespace

void installOpenFileHandler() {
#if defined(__APPLE__)
    if (Class delegate = objc_getClass("GLFWApplicationDelegate"))
        class_addMethod(delegate, sel_registerName("application:openURLs:"), reinterpret_cast<IMP>(openURLs), "v@:@@");
#endif
}

std::vector<std::string> takeOpenFileRequests() {
    std::lock_guard<std::mutex> lock(g_mutex);
    std::vector<std::string> out;
    out.swap(g_requests);
    return out;
}

} // namespace sicdm

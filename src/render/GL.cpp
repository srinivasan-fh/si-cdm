#if defined(_WIN32)
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <windows.h>
#endif

#include "render/GL.hpp"

#include <GLFW/glfw3.h>

#include <cstdio>

namespace sicdm {

#if defined(_WIN32)
namespace gl {
#  define SICDM_GL_DEFINE(type, name) type p_##name = nullptr;
SICDM_GL_FUNCTIONS(SICDM_GL_DEFINE)
#  undef SICDM_GL_DEFINE
} // namespace gl

bool loadGLFunctions(const char** missing) {
    bool ok = true;
#  define SICDM_GL_LOAD(type, name)                                                 \
    gl::p_##name = reinterpret_cast<type>(glfwGetProcAddress(#name));                  \
    if (!gl::p_##name && ok) {                                                          \
        ok = false;                                                                 \
        if (missing) *missing = #name;                                              \
    }
    SICDM_GL_FUNCTIONS(SICDM_GL_LOAD)
#  undef SICDM_GL_LOAD
    return ok;
}
#else
bool loadGLFunctions(const char** missing) {
    (void)missing;
    return true; // linked directly
}
#endif

void reportStartupError(const char* message) {
    std::fprintf(stderr, "%s\n", message);
#if defined(_WIN32)
    MessageBoxA(nullptr, message, "SI CDM", MB_OK | MB_ICONERROR);
#endif
}

} // namespace sicdm

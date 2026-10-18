#pragma once

// Single include point for OpenGL. The viewport targets OpenGL 4.1 core,
// the highest version available on macOS, so the same code runs on macOS,
// Linux and (with a loader) Windows. A Metal backend can be slotted in
// behind render/Renderer.hpp later without touching core/ or app/.

#if defined(__APPLE__)
#  ifndef GL_SILENCE_DEPRECATION
#    define GL_SILENCE_DEPRECATION
#  endif
#  include <OpenGL/gl3.h>
#else
#  ifndef GL_GLEXT_PROTOTYPES
#    define GL_GLEXT_PROTOTYPES 1
#  endif
#  include <GL/glcorearb.h>
#endif

#define GLFW_INCLUDE_NONE

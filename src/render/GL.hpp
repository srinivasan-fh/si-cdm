#pragma once

// Single include point for OpenGL. The viewport targets OpenGL 4.1 core,
// the highest version available on macOS, so the same code runs on macOS,
// Linux and Windows. A Metal backend can be slotted in behind
// render/Renderer.hpp later without touching core/ or app/.
//
// Windows' opengl32 has only OpenGL 1.1: the later functions the renderer uses are
// pointers, loaded by loadGLFunctions() once a context is current.

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

#if defined(_WIN32)
// The functions past OpenGL 1.1 that the renderer calls.
#  define SICDM_GL_FUNCTIONS(X) \
    X(PFNGLACTIVETEXTUREPROC, glActiveTexture) \
    X(PFNGLATTACHSHADERPROC, glAttachShader) \
    X(PFNGLBINDBUFFERPROC, glBindBuffer) \
    X(PFNGLBINDFRAMEBUFFERPROC, glBindFramebuffer) \
    X(PFNGLBINDVERTEXARRAYPROC, glBindVertexArray) \
    X(PFNGLBUFFERDATAPROC, glBufferData) \
    X(PFNGLBUFFERSUBDATAPROC, glBufferSubData) \
    X(PFNGLCOMPILESHADERPROC, glCompileShader) \
    X(PFNGLCREATEPROGRAMPROC, glCreateProgram) \
    X(PFNGLCREATESHADERPROC, glCreateShader) \
    X(PFNGLDELETEBUFFERSPROC, glDeleteBuffers) \
    X(PFNGLDELETEFRAMEBUFFERSPROC, glDeleteFramebuffers) \
    X(PFNGLDELETEPROGRAMPROC, glDeleteProgram) \
    X(PFNGLDELETESHADERPROC, glDeleteShader) \
    X(PFNGLDELETEVERTEXARRAYSPROC, glDeleteVertexArrays) \
    X(PFNGLENABLEVERTEXATTRIBARRAYPROC, glEnableVertexAttribArray) \
    X(PFNGLDISABLEVERTEXATTRIBARRAYPROC, glDisableVertexAttribArray) \
    X(PFNGLFRAMEBUFFERTEXTURE2DPROC, glFramebufferTexture2D) \
    X(PFNGLGENBUFFERSPROC, glGenBuffers) \
    X(PFNGLGENFRAMEBUFFERSPROC, glGenFramebuffers) \
    X(PFNGLGENVERTEXARRAYSPROC, glGenVertexArrays) \
    X(PFNGLGENERATEMIPMAPPROC, glGenerateMipmap) \
    X(PFNGLGETPROGRAMINFOLOGPROC, glGetProgramInfoLog) \
    X(PFNGLGETPROGRAMIVPROC, glGetProgramiv) \
    X(PFNGLGETSHADERINFOLOGPROC, glGetShaderInfoLog) \
    X(PFNGLGETSHADERIVPROC, glGetShaderiv) \
    X(PFNGLGETUNIFORMLOCATIONPROC, glGetUniformLocation) \
    X(PFNGLLINKPROGRAMPROC, glLinkProgram) \
    X(PFNGLSHADERSOURCEPROC, glShaderSource) \
    X(PFNGLTEXBUFFERPROC, glTexBuffer) \
    X(PFNGLUNIFORM1FPROC, glUniform1f) \
    X(PFNGLUNIFORM1IPROC, glUniform1i) \
    X(PFNGLUNIFORM2FPROC, glUniform2f) \
    X(PFNGLUNIFORM3FPROC, glUniform3f) \
    X(PFNGLUNIFORM3FVPROC, glUniform3fv) \
    X(PFNGLUNIFORM4FPROC, glUniform4f) \
    X(PFNGLUNIFORMMATRIX4FVPROC, glUniformMatrix4fv) \
    X(PFNGLUSEPROGRAMPROC, glUseProgram) \
    X(PFNGLVERTEXATTRIBPOINTERPROC, glVertexAttribPointer) \
    X(PFNGLVERTEXATTRIBDIVISORPROC, glVertexAttribDivisor) \
    X(PFNGLDRAWELEMENTSINSTANCEDPROC, glDrawElementsInstanced) \
    X(PFNGLDRAWARRAYSINSTANCEDPROC, glDrawArraysInstanced)
// Pointers are named p_glX (token pasting keeps the glX macros below out of the X-macros).
namespace sicdm::gl {
#  define SICDM_GL_DECLARE(type, name) extern type p_##name;
SICDM_GL_FUNCTIONS(SICDM_GL_DECLARE)
#  undef SICDM_GL_DECLARE
} // namespace sicdm::gl
#  define glActiveTexture sicdm::gl::p_glActiveTexture
#  define glAttachShader sicdm::gl::p_glAttachShader
#  define glBindBuffer sicdm::gl::p_glBindBuffer
#  define glBindFramebuffer sicdm::gl::p_glBindFramebuffer
#  define glBindVertexArray sicdm::gl::p_glBindVertexArray
#  define glBufferData sicdm::gl::p_glBufferData
#  define glBufferSubData sicdm::gl::p_glBufferSubData
#  define glTexBuffer sicdm::gl::p_glTexBuffer
#  define glCompileShader sicdm::gl::p_glCompileShader
#  define glCreateProgram sicdm::gl::p_glCreateProgram
#  define glCreateShader sicdm::gl::p_glCreateShader
#  define glDeleteBuffers sicdm::gl::p_glDeleteBuffers
#  define glDeleteFramebuffers sicdm::gl::p_glDeleteFramebuffers
#  define glDeleteProgram sicdm::gl::p_glDeleteProgram
#  define glDeleteShader sicdm::gl::p_glDeleteShader
#  define glDeleteVertexArrays sicdm::gl::p_glDeleteVertexArrays
#  define glEnableVertexAttribArray sicdm::gl::p_glEnableVertexAttribArray
#  define glDisableVertexAttribArray sicdm::gl::p_glDisableVertexAttribArray
#  define glFramebufferTexture2D sicdm::gl::p_glFramebufferTexture2D
#  define glGenBuffers sicdm::gl::p_glGenBuffers
#  define glGenFramebuffers sicdm::gl::p_glGenFramebuffers
#  define glGenVertexArrays sicdm::gl::p_glGenVertexArrays
#  define glGenerateMipmap sicdm::gl::p_glGenerateMipmap
#  define glGetProgramInfoLog sicdm::gl::p_glGetProgramInfoLog
#  define glGetProgramiv sicdm::gl::p_glGetProgramiv
#  define glGetShaderInfoLog sicdm::gl::p_glGetShaderInfoLog
#  define glGetShaderiv sicdm::gl::p_glGetShaderiv
#  define glGetUniformLocation sicdm::gl::p_glGetUniformLocation
#  define glLinkProgram sicdm::gl::p_glLinkProgram
#  define glShaderSource sicdm::gl::p_glShaderSource
#  define glUniform1f sicdm::gl::p_glUniform1f
#  define glUniform1i sicdm::gl::p_glUniform1i
#  define glUniform2f sicdm::gl::p_glUniform2f
#  define glUniform3f sicdm::gl::p_glUniform3f
#  define glUniform3fv sicdm::gl::p_glUniform3fv
#  define glUniform4f sicdm::gl::p_glUniform4f
#  define glUniformMatrix4fv sicdm::gl::p_glUniformMatrix4fv
#  define glUseProgram sicdm::gl::p_glUseProgram
#  define glVertexAttribPointer sicdm::gl::p_glVertexAttribPointer
#  define glVertexAttribDivisor sicdm::gl::p_glVertexAttribDivisor
#  define glDrawElementsInstanced sicdm::gl::p_glDrawElementsInstanced
#  define glDrawArraysInstanced sicdm::gl::p_glDrawArraysInstanced
#endif

namespace sicdm {
// Loads the OpenGL functions a platform doesn't link directly (Windows); call with the
// context current. False (with the first one missing in `missing`) when the driver lacks one.
bool loadGLFunctions(const char** missing = nullptr);

// Tells the user why the app can't start: stderr, and a message box on Windows, where a
// GUI program has no console.
void reportStartupError(const char* message);
} // namespace sicdm

#define GLFW_INCLUDE_NONE

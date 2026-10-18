#pragma once

#include "core/Math.hpp"
#include "render/GL.hpp"

#include <string_view>

namespace sicdm {

// RAII wrapper for a linked GLSL program.
class Shader {
public:
    Shader() = default;
    Shader(std::string_view vertexSrc, std::string_view fragmentSrc);
    ~Shader();

    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;
    Shader(Shader&& o) noexcept;
    Shader& operator=(Shader&& o) noexcept;

    bool valid() const noexcept { return m_program != 0; }
    void bind() const noexcept { glUseProgram(m_program); }

    void set(const char* name, float v) const noexcept;
    void set(const char* name, const Vec2& v) const noexcept;
    void set(const char* name, const Vec3& v) const noexcept;
    void set(const char* name, const Vec4& v) const noexcept;
    void set(const char* name, const Mat4& m) const noexcept;

private:
    GLint location(const char* name) const noexcept { return glGetUniformLocation(m_program, name); }

    GLuint m_program = 0;
};

} // namespace sicdm

#include "render/Shader.hpp"

#include <cstdio>
#include <string>
#include <utility>

namespace sicdm {

namespace {

GLuint compile(GLenum type, std::string_view src) {
    const GLuint s = glCreateShader(type);
    const char* ptr = src.data();
    const auto len = static_cast<GLint>(src.size());
    glShaderSource(s, 1, &ptr, &len);
    glCompileShader(s);

    GLint ok = GL_FALSE;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        GLint logLen = 0;
        glGetShaderiv(s, GL_INFO_LOG_LENGTH, &logLen);
        std::string log(static_cast<std::size_t>(std::max(logLen, 1)), '\0');
        glGetShaderInfoLog(s, logLen, nullptr, log.data());
        std::fprintf(stderr, "[shader] %s compile error:\n%s\n",
                     type == GL_VERTEX_SHADER ? "vertex" : "fragment", log.c_str());
        glDeleteShader(s);
        return 0;
    }
    return s;
}

} // namespace

Shader::Shader(std::string_view vs, std::string_view fs) {
    const GLuint v = compile(GL_VERTEX_SHADER, vs);
    const GLuint f = compile(GL_FRAGMENT_SHADER, fs);
    if (!v || !f) {
        if (v) glDeleteShader(v);
        if (f) glDeleteShader(f);
        return;
    }

    m_program = glCreateProgram();
    glAttachShader(m_program, v);
    glAttachShader(m_program, f);
    glLinkProgram(m_program);
    glDeleteShader(v);
    glDeleteShader(f);

    GLint ok = GL_FALSE;
    glGetProgramiv(m_program, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[2048];
        glGetProgramInfoLog(m_program, sizeof(log), nullptr, log);
        std::fprintf(stderr, "[shader] link error:\n%s\n", log);
        glDeleteProgram(m_program);
        m_program = 0;
    }
}

Shader::~Shader() {
    if (m_program) glDeleteProgram(m_program);
}

Shader::Shader(Shader&& o) noexcept : m_program(std::exchange(o.m_program, 0)) {}

Shader& Shader::operator=(Shader&& o) noexcept {
    if (this != &o) {
        if (m_program) glDeleteProgram(m_program);
        m_program = std::exchange(o.m_program, 0);
    }
    return *this;
}

void Shader::set(const char* n, float v) const noexcept { glUniform1f(location(n), v); }
void Shader::set(const char* n, const Vec2& v) const noexcept { glUniform2f(location(n), v.x, v.y); }
void Shader::set(const char* n, const Vec3& v) const noexcept { glUniform3f(location(n), v.x, v.y, v.z); }
void Shader::set(const char* n, const Vec4& v) const noexcept { glUniform4f(location(n), v.x, v.y, v.z, v.w); }
void Shader::set(const char* n, const Mat4& m) const noexcept { glUniformMatrix4fv(location(n), 1, GL_FALSE, m.data()); }

} // namespace sicdm

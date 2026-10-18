#include "render/Renderer.hpp"

#include <cmath>
#include <cstddef>

namespace sicdm {

namespace {

// ---------------------------------------------------------------------------
// Shaders (GLSL 410 core: the OpenGL ceiling on macOS).

constexpr const char* kBackgroundVS = R"(#version 410 core
out vec2 vUV;
void main() {
    // Full-screen triangle from gl_VertexID, no vertex buffer needed.
    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
    vUV = p;
    gl_Position = vec4(p * 2.0 - 1.0, 1.0, 1.0);
}
)";

constexpr const char* kBackgroundFS = R"(#version 410 core
in vec2 vUV;
uniform vec3 uTop;
uniform vec3 uBottom;
out vec4 fragColor;
void main() {
    float t = smoothstep(0.0, 1.0, vUV.y);
    fragColor = vec4(mix(uBottom, uTop, t), 1.0);
}
)";

constexpr const char* kMeshVS = R"(#version 410 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
uniform mat4 uViewProj;
uniform vec3 uOffset;
out vec3 vWorldPos;
out vec3 vNormal;
void main() {
    vWorldPos = aPos + uOffset;
    vNormal = aNormal;
    gl_Position = uViewProj * vec4(vWorldPos, 1.0);
}
)";

constexpr const char* kMeshFS = R"(#version 410 core
in vec3 vWorldPos;
in vec3 vNormal;
uniform vec3 uColor;
uniform vec3 uCamPos;
uniform vec3 uCamRight;
uniform vec3 uCamUp;
uniform float uHighlight;
out vec4 fragColor;
void main() {
    vec3 N = normalize(vNormal);
    vec3 V = normalize(uCamPos - vWorldPos);
    if (!gl_FrontFacing) N = -N;

    // Headlight slightly above-left of the eye plus a cool fill light.
    vec3 L1 = normalize(V + 0.35 * uCamUp - 0.25 * uCamRight);
    vec3 L2 = normalize(-V + 0.8 * uCamUp + 0.6 * uCamRight);

    float diff = max(dot(N, L1), 0.0) * 0.80 + max(dot(N, L2), 0.0) * 0.18;
    float spec = pow(max(dot(N, normalize(L1 + V)), 0.0), 64.0) * 0.35;

    // Hemisphere ambient around world +Z.
    float hemi = 0.5 + 0.5 * N.z;
    vec3 ambient = mix(vec3(0.16, 0.15, 0.14), vec3(0.24, 0.26, 0.30), hemi);

    vec3 base = mix(uColor, vec3(0.30, 0.65, 1.00), uHighlight * 0.55);
    vec3 c = base * (ambient + diff) + vec3(spec);
    // Light rim to separate silhouettes from the background.
    c += pow(1.0 - max(dot(N, V), 0.0), 3.0) * 0.08;
    fragColor = vec4(pow(c, vec3(1.0 / 1.1)), 1.0);
}
)";

constexpr const char* kLinesVS = R"(#version 410 core
layout(location = 0) in vec3 aPos;
uniform mat4 uViewProj;
uniform vec3 uOffset;
void main() { gl_Position = uViewProj * vec4(aPos + uOffset, 1.0); }
)";

constexpr const char* kLinesFS = R"(#version 410 core
uniform vec4 uColor;
out vec4 fragColor;
void main() { fragColor = uColor; }
)";

constexpr const char* kGridVS = R"(#version 410 core
uniform mat4 uViewProj;
uniform vec3 uCenter;
uniform float uExtent;
out vec3 vWorldPos;
void main() {
    const vec2 corners[6] = vec2[](vec2(-1,-1), vec2(1,-1), vec2(1,1),
                                   vec2(-1,-1), vec2(1,1), vec2(-1,1));
    vec3 p = vec3(uCenter.xy + corners[gl_VertexID] * uExtent, 0.0);
    vWorldPos = p;
    gl_Position = uViewProj * vec4(p, 1.0);
}
)";

constexpr const char* kGridFS = R"(#version 410 core
in vec3 vWorldPos;
uniform vec3 uCenter;
uniform float uExtent;
uniform float uLevel;   // log10 of the minor spacing (fractional)
out vec4 fragColor;

float lineMask(vec2 p, float spacing, float widthPx) {
    vec2 c = p / spacing;
    vec2 d = fwidth(c);
    vec2 g = abs(fract(c - 0.5) - 0.5) / (d * widthPx);
    return 1.0 - min(min(g.x, g.y), 1.0);
}

void main() {
    vec2 p = vWorldPos.xy;
    float lvl = floor(uLevel);
    float blend = fract(uLevel);
    float minor = pow(10.0, lvl);
    float major = minor * 10.0;

    float a = lineMask(p, minor, 1.0) * (1.0 - blend) * 0.22;
    a = max(a, lineMask(p, major, 1.2) * 0.45);
    a = max(a, lineMask(p, major * 10.0, 1.4) * 0.55);
    vec3 col = vec3(0.62, 0.66, 0.72);

    // World axes on the plane: X red (y == 0), Y green (x == 0).
    vec2 fw = fwidth(p);
    float ax = 1.0 - min(abs(p.y) / (fw.y * 1.6), 1.0);
    float ay = 1.0 - min(abs(p.x) / (fw.x * 1.6), 1.0);
    col = mix(col, vec3(0.90, 0.30, 0.30), ax);
    col = mix(col, vec3(0.40, 0.80, 0.35), ay);
    a = max(a, max(ax, ay) * 0.9);

    // Fade towards the edge of the patch so it reads as infinite.
    float r = length(p - uCenter.xy) / uExtent;
    a *= 1.0 - smoothstep(0.35, 1.0, r);
    if (a <= 0.002) discard;
    fragColor = vec4(col, a);
}
)";

GpuMesh uploadMesh(const MeshData& m) {
    GpuMesh g;

    glGenVertexArrays(1, &g.vao);
    glBindVertexArray(g.vao);
    glGenBuffers(1, &g.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, g.vbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(m.vertices.size() * sizeof(Vertex)),
                 m.vertices.data(), GL_STATIC_DRAW);
    glGenBuffers(1, &g.ibo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, g.ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(m.indices.size() * sizeof(std::uint32_t)),
                 m.indices.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          reinterpret_cast<const void*>(offsetof(Vertex, position)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          reinterpret_cast<const void*>(offsetof(Vertex, normal)));
    g.indexCount = static_cast<GLsizei>(m.indices.size());

    if (!m.edgeVertices.empty()) {
        glGenVertexArrays(1, &g.edgeVao);
        glBindVertexArray(g.edgeVao);
        glGenBuffers(1, &g.edgeVbo);
        glBindBuffer(GL_ARRAY_BUFFER, g.edgeVbo);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(m.edgeVertices.size() * sizeof(Vec3)),
                     m.edgeVertices.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vec3), nullptr);
        g.edgeVertexCount = static_cast<GLsizei>(m.edgeVertices.size());
    }

    glBindVertexArray(0);
    return g;
}

} // namespace

Renderer::Renderer()
    : m_background(kBackgroundVS, kBackgroundFS),
      m_mesh(kMeshVS, kMeshFS),
      m_lines(kLinesVS, kLinesFS),
      m_grid(kGridVS, kGridFS) {
    glGenVertexArrays(1, &m_emptyVao);
}

Renderer::~Renderer() {
    releaseMeshes();
    if (m_emptyVao) glDeleteVertexArrays(1, &m_emptyVao);
}

bool Renderer::valid() const noexcept {
    return m_background.valid() && m_mesh.valid() && m_lines.valid() && m_grid.valid();
}

void Renderer::releaseMeshes() {
    for (GpuMesh& g : m_meshes) {
        glDeleteBuffers(1, &g.vbo);
        glDeleteBuffers(1, &g.ibo);
        glDeleteVertexArrays(1, &g.vao);
        if (g.edgeVbo) glDeleteBuffers(1, &g.edgeVbo);
        if (g.edgeVao) glDeleteVertexArrays(1, &g.edgeVao);
    }
    m_meshes.clear();
}

void Renderer::upload(const Scene& scene) {
    releaseMeshes();
    m_meshes.reserve(scene.bodies().size());
    for (const Body& b : scene.bodies()) m_meshes.push_back(uploadMesh(b.mesh));
    m_uploadedRevision = scene.revision();
}

void Renderer::drawBackground(const ViewportSettings& s) {
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    m_background.bind();
    m_background.set("uTop", s.backgroundTop);
    m_background.set("uBottom", s.backgroundBottom);
    glBindVertexArray(m_emptyVao);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
}

void Renderer::drawGrid(const OrbitCamera& cam) {
    // Pick a spacing that keeps ~10-100 minor cells across the view.
    const float viewSize = cam.distance() * std::tan(cam.fovY() * 0.5f) * 2.0f;
    const float level = std::log10(std::max(viewSize / 20.0f, 1e-4f));
    const float extent = std::max(cam.distance() * 8.0f, 10.0f);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);

    m_grid.bind();
    m_grid.set("uViewProj", cam.viewProjection());
    m_grid.set("uCenter", Vec3{cam.target().x, cam.target().y, 0.0f});
    m_grid.set("uExtent", extent);
    m_grid.set("uLevel", level);
    glBindVertexArray(m_emptyVao);
    glDrawArrays(GL_TRIANGLES, 0, 6);

    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}

void Renderer::render(const Scene& scene, const OrbitCamera& cam, const ViewportSettings& s,
                      int fbWidth, int fbHeight, int selectedBody) {
    glViewport(0, 0, fbWidth, fbHeight);
    glClearColor(s.backgroundBottom.x, s.backgroundBottom.y, s.backgroundBottom.z, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_MULTISAMPLE);

    drawBackground(s);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDisable(GL_CULL_FACE); // CAD bodies may be open; light both sides.

    const Mat4 vp = cam.viewProjection();
    const auto& bodies = scene.bodies();
    const std::size_t count = std::min(bodies.size(), m_meshes.size());

    // Solid pass, pushed back slightly so edges win the depth test.
    if (s.displayMode != DisplayMode::Wireframe) {
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(1.0f, 1.0f);
        m_mesh.bind();
        m_mesh.set("uViewProj", vp);
        m_mesh.set("uCamPos", cam.position());
        m_mesh.set("uCamRight", cam.right());
        m_mesh.set("uCamUp", cam.up());
        for (std::size_t i = 0; i < count; ++i) {
            if (!bodies[i].visible) continue;
            m_mesh.set("uColor", bodies[i].color);
            m_mesh.set("uOffset", bodies[i].offset);
            m_mesh.set("uHighlight", static_cast<int>(i) == selectedBody ? 1.0f : 0.0f);
            glBindVertexArray(m_meshes[i].vao);
            glDrawElements(GL_TRIANGLES, m_meshes[i].indexCount, GL_UNSIGNED_INT, nullptr);
        }
        glDisable(GL_POLYGON_OFFSET_FILL);
    }

    // Edge / wireframe pass.
    if (s.displayMode != DisplayMode::Shaded) {
        m_lines.bind();
        m_lines.set("uViewProj", vp);
        const bool wire = s.displayMode == DisplayMode::Wireframe;
        for (std::size_t i = 0; i < count; ++i) {
            if (!bodies[i].visible) continue;
            const bool sel = static_cast<int>(i) == selectedBody;
            m_lines.set("uOffset", bodies[i].offset);
            if (wire) {
                const Vec3 c = sel ? Vec3{0.35f, 0.70f, 1.0f} : bodies[i].color;
                m_lines.set("uColor", Vec4{c.x, c.y, c.z, 1.0f});
                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                glBindVertexArray(m_meshes[i].vao);
                glDrawElements(GL_TRIANGLES, m_meshes[i].indexCount, GL_UNSIGNED_INT, nullptr);
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            } else if (m_meshes[i].edgeVao) {
                const Vec3 c = sel ? Vec3{0.10f, 0.45f, 0.95f} : s.edgeColor;
                m_lines.set("uColor", Vec4{c.x, c.y, c.z, 1.0f});
                glBindVertexArray(m_meshes[i].edgeVao);
                glDrawArrays(GL_LINES, 0, m_meshes[i].edgeVertexCount);
            }
        }
    }

    if (s.showGrid) drawGrid(cam);

    glBindVertexArray(0);
    glUseProgram(0);
}

} // namespace sicdm

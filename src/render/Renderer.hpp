#pragma once

#include "core/Camera.hpp"
#include "core/Scene.hpp"
#include "render/Shader.hpp"

#include <vector>

namespace sicdm {

enum class DisplayMode { Shaded, ShadedWithEdges, Wireframe };

struct ViewportSettings {
    DisplayMode displayMode = DisplayMode::ShadedWithEdges;
    bool showGrid = true;
    Vec3 backgroundTop{0.20f, 0.23f, 0.29f};
    Vec3 backgroundBottom{0.06f, 0.07f, 0.09f};
    Vec3 edgeColor{0.08f, 0.09f, 0.11f};
};

// GPU-side copy of one body.
struct GpuMesh {
    GLuint vao = 0, vbo = 0, ibo = 0;
    GLuint edgeVao = 0, edgeVbo = 0;
    GLsizei indexCount = 0;
    GLsizei edgeVertexCount = 0;
};

class Renderer {
public:
    Renderer();
    ~Renderer();

    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    bool valid() const noexcept;

    // Uploads all scene bodies. Call again if geometry changes.
    void upload(const Scene& scene);

    void render(const Scene& scene, const OrbitCamera& camera, const ViewportSettings& settings,
                int framebufferWidth, int framebufferHeight, int selectedBody);

private:
    void drawBackground(const ViewportSettings& s);
    void drawGrid(const OrbitCamera& cam);
    void releaseMeshes();

    Shader m_background;
    Shader m_mesh;
    Shader m_lines;
    Shader m_grid;

    GLuint m_emptyVao = 0;
    std::vector<GpuMesh> m_meshes;
};

} // namespace sicdm

#pragma once

#include "core/Camera.hpp"
#include "core/Scene.hpp"
#include "render/Shader.hpp"

#include <chrono>
#include <future>
#include <memory>
#include <vector>

namespace sicdm {

enum class DisplayMode { Shaded, ShadedWithEdges, Wireframe, HiddenLine };

struct ViewportSettings {
    DisplayMode displayMode = DisplayMode::ShadedWithEdges;
    bool showGrid = true;
    Vec3 backgroundTop{0.20f, 0.23f, 0.29f};
    Vec3 backgroundBottom{0.06f, 0.07f, 0.09f};
    Vec3 edgeColor{0.08f, 0.09f, 0.11f};
    Vec3 sketchColor{1.0f, 0.66f, 0.28f};
    SketchPlane gridPlane = SketchPlane::Top;
    PlaneFrame gridFrame = planeFrame(SketchPlane::Top); // the grid's plane when gridPlane is Custom
    float gridStrength = 1.0f;
    bool sketchOnTop = false; // draw sketch curves over bodies (2D drawing view)
    std::vector<Vec3> sketchColors; // per curve, in place of sketchColor (constraint status); empty: all sketchColor
    bool showPolygroups = false; // bodies with polygroups drawn a colour per group
};

// GPU-side copy of one body.
struct GpuMesh {
    GLuint vao = 0, vbo = 0, ibo = 0;
    GLuint edgeVao = 0, edgeVbo = 0;
    GLsizei indexCount = 0;
    GLsizei edgeVertexCount = 0;
    GLuint groupTbo = 0, groupTex = 0; // polygroups, per triangle (a texture buffer)
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

    // Re-uploads only when bodies were added or removed since the last upload.
    void sync(const Scene& scene) {
        if (scene.revision() != m_uploadedRevision) upload(scene);
    }

    void render(const Scene& scene, const OrbitCamera& camera, const ViewportSettings& settings,
                int framebufferWidth, int framebufferHeight, int selectedBody, int selectedCurve,
                int selectedImage = -1, const std::vector<Mat4>* motion = nullptr, // motion: per-body transforms
                const std::vector<const MeshData*>* overrides = nullptr);           // per-body shapes for this frame

private:
    void drawBackground(const ViewportSettings& s);
    void drawGrid(const OrbitCamera& cam, const ViewportSettings& s);
    void uploadSketch(const Sketch& sketch);
    void drawSketch(const Scene& scene, const OrbitCamera& cam, const ViewportSettings& s, int selected);
    void releaseMeshes();
    void releaseImages();
    void uploadImages(const Scene& scene);
    void drawImages(const Scene& scene, const OrbitCamera& cam, const ViewportSettings& s, int selected);
    GLuint m_imageShaderProgram() const noexcept { return m_imageShader.program(); }

    Shader m_background;
    Shader m_mesh;
    Shader m_lines;
    Shader m_grid;
    Shader m_sketchShader;
    Shader m_imageShader;
    std::vector<GLuint> m_imageTextures; // parallel to scene.images(); 0 = no pixels
    std::uint64_t m_imageRevision = ~std::uint64_t{0};
    GLuint m_outlineVao = 0, m_outlineVbo = 0;

    struct SketchRange {
        GLint first;
        GLsizei count;
        bool point;
        bool dashed; // construction: dash pairs (GL_LINES)
    };
    GLuint m_sketchVao = 0, m_sketchVbo = 0;
    std::vector<SketchRange> m_sketchRanges;
    std::uint64_t m_sketchRevision = ~std::uint64_t{0};

    GLuint m_emptyVao = 0;
    // A body's shape on the GPU, shared by bodies with the same shape and kept across
    // uploads while the shape stays the same (only changed bodies are uploaded again).
    // Large ones get coarser copies for drawing small on screen, made in the background
    // once the shape has stopped changing.
    struct MeshEntry {
        std::uint64_t key = 0;
        GpuMesh full;
        std::size_t triangles = 0;
        AABB box; // mesh coordinates
        std::chrono::steady_clock::time_point born;
        bool lodStarted = false;
        std::future<std::vector<std::pair<float, MeshData>>> job; // cell size, mesh: finer first
        std::vector<std::pair<float, GpuMesh>> lods;
    };
    std::vector<std::shared_ptr<MeshEntry>> m_entries; // per body
    std::vector<std::future<std::vector<std::pair<float, MeshData>>>> m_orphanJobs; // of shapes since replaced
    std::uint64_t m_frame = 0;
    std::vector<GpuMesh> m_streamed; // per-frame overrides (simulated chains / belts)
    std::uint64_t m_uploadedRevision = ~std::uint64_t{0};

public:
    // What the last frame drew (for the status bar and tests).
    struct FrameStats {
        std::size_t triangles = 0;     // drawn
        std::size_t fullTriangles = 0; // the bodies' own
        int coarseBodies = 0;          // drawn from a coarser copy
        int uploads = 0;               // shapes uploaded by the last upload()
    };
    const FrameStats& stats() const noexcept { return m_stats; }
    // The last frame drew some shape coarser than a still view allows (the view was
    // moving): draw another once it stops.
    bool wantsRefine() const noexcept { return m_refine; }
    // Coarser copies are waiting to be made or uploaded: draw a frame now and then.
    bool pendingWork() const noexcept;

private:
    FrameStats m_stats;
    Mat4 m_lastViewProj{};
    bool m_refine = false;
};

} // namespace sicdm

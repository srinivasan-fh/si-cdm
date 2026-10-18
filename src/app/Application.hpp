#pragma once

#include "core/Camera.hpp"
#include "core/Document.hpp"
#include "core/Mcp.hpp"
#include "core/Scene.hpp"
#include "app/McpHttpServer.hpp"
#include "app/SketchController.hpp"
#include "render/Renderer.hpp"
#include "ui/LandingUI.hpp"

#include <memory>
#include <optional>
#include <string>
#include <vector>

struct GLFWwindow;

namespace sicdm {

struct AppOptions {
    int width = 1600;
    int height = 960;
    std::string screenshotPath; // when set: render a few frames, save PPM, exit
    bool start2D = false;       // open in the 2D drawing window
    int mcpPort = 0;            // > 0: start the MCP HTTP server for AI agents
    std::string openPath;       // project or file to open at startup
    std::string executablePath; // argv[0], for MCP setup hints
};

class Application final : public ViewHost {
public:
    explicit Application(AppOptions options);
    ~Application();

    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    int run();

    // ViewHost: window / camera control for commands and MCP.
    void openWindow(bool drawing2D) override;
    void standardView(StandardView v) override;
    void planeChanged(SketchPlane p) override;
    void fit() override;
    void zoomStep(float steps) override;
    void home() override;
    void roll(float degrees, bool absolute) override;
    void setProjection(Projection p) override;
    void setDisplay(int mode) override;
    void setGrid(std::optional<bool> on) override;
    void setPanel(std::optional<bool> on) override;
    std::string describe() const override;
    bool simulation(const std::string& action, float value, std::string* reply) override;

    // ViewHost: file tabs.
    bool prepareDocument(const std::string& path) override;
    std::vector<std::string> fileTabs() const override;
    bool switchFile(const std::string& which, std::string* error) override;
    bool closeFile(bool discard, std::string* error) override;

private:
    bool init();
    void shutdown();
    void frame(float dt);
    void handleViewportInput();
    void onCanvasClick(float x, float y, bool doubleClick);
    std::optional<RayHit> pick(float x, float y) const;
    int pickCurve(float x, float y) const;
    void addCurve(SketchCurve curve);
    void setMode(ViewMode mode);
    void setPlane(SketchPlane plane);
    int pickImage(float x, float y, Vec2* planePoint = nullptr) const;

    // Extrude tool.
    int pickProfile(float x, float y) const; // closed curve under or around the cursor
    void beginExtrude(int curve);
    void updateExtrudeFromMouse();
    void applyExtrude();
    void cancelExtrude();
    void drawExtrudePreview() const;

    // Simulation window.
    bool simulating() const noexcept { return m_uiState.mode == ViewMode::Simulation; }
    void advanceSimulation(float dt);
    void drawSimulationOverlay() const;
    std::string simStatus() const;

    // Command line / MCP plumbing.
    void runCommandLine(const std::string& text);
    void log(const std::string& text, int kind);
    void syncToDoc();
    void syncFromDoc();
    void afterDocumentReplaced();
    void updatePrompt();
    void toggleMcp();
    std::string handleMcp(const std::string& body);
    static std::uint64_t signatureOf(const Document& doc); // changes whenever the saved content would
    std::uint64_t contentSignature() const { return signatureOf(m_doc); }
    void updateTitle();
    void init2DCamera();

    // File tabs. Each open file keeps its own document, cameras, windows and selection.
    // The active file lives in the members below (m_doc, m_cam3D, ...); the others are
    // parked in m_files, and m_files[m_activeFile] is an empty placeholder.
    struct FileTab {
        int id = 0;
        Document doc;
        OrbitCamera cam3D, cam2D;
        ViewportSettings vp3D, vp2D;
        bool ready2D = false;
        std::uint64_t savedSignature = 0;
        ViewMode mode = ViewMode::Model3D;
        float simTime = 0.0f;
        bool demo = false; // the start-up demo part: Open replaces it while it is untouched
    };
    void parkActiveFile();
    void loadFile(int index);
    void activateFile(int index);
    void newFileTab();
    bool closeFileTab(int index, bool discard);
    int fileIndex(int id) const;
    const Document& fileDoc(int index) const;
    bool anyFileDirty() const;
    void refreshFileTabs();
    void saveAll();

    // Each window keeps its own camera and display settings.
    OrbitCamera& camera() noexcept { return m_uiState.mode == ViewMode::Drawing2D ? m_cam2D : m_cam3D; }
    const OrbitCamera& camera() const noexcept { return m_uiState.mode == ViewMode::Drawing2D ? m_cam2D : m_cam3D; }
    ViewportSettings& viewport() noexcept { return m_uiState.mode == ViewMode::Drawing2D ? m_vp2D : m_vp3D; }
    const ViewportSettings& viewport() const noexcept { return m_uiState.mode == ViewMode::Drawing2D ? m_vp2D : m_vp3D; }
    void setStatus(std::string message);
    void handleCommands(const UiCommands& cmd);
    bool saveScreenshot(const std::string& path) const;

    static void onScroll(GLFWwindow* w, double dx, double dy);
    static void onEvent(GLFWwindow* w);

    AppOptions m_options;
    GLFWwindow* m_window = nullptr;
    bool m_imguiReady = false;

    std::vector<FileTab> m_files;
    int m_activeFile = 0;
    int m_nextFileId = 1;
    int m_untitledCount = 0;

    Document m_doc;
    OrbitCamera m_cam3D, m_cam2D;
    ViewportSettings m_vp3D, m_vp2D;
    bool m_2DReady = false;
    float m_aspect = 1.0f;
    SketchController m_sketcher;
    std::unique_ptr<Renderer> m_renderer;
    LandingUI m_ui;
    UiState m_uiState;

    // Input state
    enum class DragMode { None, Pending, Orbit, Pan, Zoom, MoveBody, MoveImage };
    DragMode m_leftDrag = DragMode::None;
    DragMode m_otherDrag = DragMode::None; // right / middle button
    float m_pressX = 0, m_pressY = 0;
    float m_pendingScroll = 0;
    Vec3 m_movePlanePoint;
    Vec3 m_moveStartOffset;
    Vec2 m_imageGrab{}, m_imageStart{};

    std::vector<Mat4> m_simTransforms; // per body, while the Simulation window shows motion

    int m_extrudeCurve = -1;   // profile being extruded
    float m_extrudeAnchorDist = 0.0f;
    float m_extrudeAnchorX = 0.0f, m_extrudeAnchorY = 0.0f;

    // MCP server (shared by the HTTP transport) and unsaved-change tracking.
    std::unique_ptr<McpServer> m_mcp;
    McpHttpServer m_mcpHttp;
    std::uint64_t m_savedSignature = 0;
    std::string m_shownTitle;
    bool m_closeRequested = false;

    double m_statusUntil = 0.0;

    // On-demand rendering: CAD viewports only redraw when something changes.
    int m_framesToRender = 3;
};

} // namespace sicdm

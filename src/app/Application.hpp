#pragma once

#include "core/Camera.hpp"
#include "core/Scene.hpp"
#include "app/SketchController.hpp"
#include "render/Renderer.hpp"
#include "ui/LandingUI.hpp"

#include <memory>
#include <optional>
#include <string>

struct GLFWwindow;

namespace sicdm {

struct AppOptions {
    int width = 1600;
    int height = 960;
    std::string screenshotPath; // when set: render a few frames, save PPM, exit
    bool start2D = false;       // open in the 2D drawing window
};

class Application {
public:
    explicit Application(AppOptions options);
    ~Application();

    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    int run();

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

    // Each window keeps its own camera and display settings.
    OrbitCamera& camera() noexcept { return m_uiState.mode == ViewMode::Drawing2D ? m_cam2D : m_cam3D; }
    const OrbitCamera& camera() const noexcept { return m_uiState.mode == ViewMode::Drawing2D ? m_cam2D : m_cam3D; }
    ViewportSettings& viewport() noexcept { return m_uiState.mode == ViewMode::Drawing2D ? m_vp2D : m_vp3D; }
    void setStatus(std::string message);
    void handleCommands(const UiCommands& cmd);
    bool saveScreenshot(const std::string& path) const;

    static void onScroll(GLFWwindow* w, double dx, double dy);
    static void onEvent(GLFWwindow* w);

    AppOptions m_options;
    GLFWwindow* m_window = nullptr;
    bool m_imguiReady = false;

    Scene m_scene;
    OrbitCamera m_cam3D, m_cam2D;
    ViewportSettings m_vp3D, m_vp2D;
    bool m_2DReady = false;
    float m_aspect = 1.0f;
    SketchController m_sketcher;
    std::unique_ptr<Renderer> m_renderer;
    LandingUI m_ui;
    UiState m_uiState;

    // Input state
    enum class DragMode { None, Pending, Orbit, Pan, Zoom, MoveBody };
    DragMode m_leftDrag = DragMode::None;
    DragMode m_otherDrag = DragMode::None; // right / middle button
    float m_pressX = 0, m_pressY = 0;
    float m_pendingScroll = 0;
    Vec3 m_movePlanePoint;
    Vec3 m_moveStartOffset;

    int m_primitiveSerial = 0;
    double m_statusUntil = 0.0;

    // On-demand rendering: CAD viewports only redraw when something changes.
    int m_framesToRender = 3;
};

} // namespace sicdm

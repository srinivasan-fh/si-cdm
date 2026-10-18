#pragma once

#include "core/Camera.hpp"
#include "core/Scene.hpp"
#include "render/Renderer.hpp"
#include "ui/LandingUI.hpp"

#include <memory>
#include <string>

struct GLFWwindow;

namespace sicdm {

struct AppOptions {
    int width = 1600;
    int height = 960;
    std::string screenshotPath; // when set: render a few frames, save PPM, exit
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
    void handleCommands(const UiCommands& cmd);
    bool saveScreenshot(const std::string& path) const;

    static void onScroll(GLFWwindow* w, double dx, double dy);
    static void onEvent(GLFWwindow* w);

    AppOptions m_options;
    GLFWwindow* m_window = nullptr;
    bool m_imguiReady = false;

    Scene m_scene;
    OrbitCamera m_camera;
    ViewportSettings m_viewport;
    std::unique_ptr<Renderer> m_renderer;
    LandingUI m_ui;
    UiState m_uiState;

    // Input state
    double m_lastX = 0, m_lastY = 0;
    float m_pendingScroll = 0;
    bool m_dragging = false;

    // On-demand rendering: CAD viewports only redraw when something changes.
    int m_framesToRender = 3;
};

} // namespace sicdm

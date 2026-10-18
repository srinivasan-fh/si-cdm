#pragma once

#include "core/Camera.hpp"
#include "core/Scene.hpp"
#include "render/Renderer.hpp"
#include "ui/Toolbox.hpp"

#include <optional>
#include <string>

namespace sicdm {

// Per-frame state the UI displays.
struct UiState {
    bool showWelcome = true;
    bool showModelTree = true;
    bool showControls = false;
    bool showAbout = false;
    int selectedBody = -1;
    float frameMs = 0.0f;
    std::string statusMessage; // transient feedback (e.g. "Placed Block 1")
    std::string gpuName;
    std::string glVersion;
};

// Actions requested by the UI, applied by the application after drawing.
struct UiCommands {
    std::optional<StandardView> view;
    bool fit = false;
    bool toggleProjection = false;
    bool quit = false;
};

// Landing page of the workbench: menu bar, view toolbar, model tree,
// welcome card, navigation triad and status bar around the 3D canvas.
class LandingUI {
public:
    UiCommands draw(Scene& scene, const OrbitCamera& camera, ViewportSettings& viewport, UiState& state);

    Toolbox& toolbox() noexcept { return m_toolbox; }

private:
    void drawMenuBar(UiCommands& cmd, const OrbitCamera& cam, ViewportSettings& vp, UiState& st);
    void drawToolbar(UiCommands& cmd, const OrbitCamera& cam, ViewportSettings& vp);
    void drawModelTree(Scene& scene, UiState& st);
    void drawWelcome(UiState& st);
    void drawStatusBar(const Scene& scene, const OrbitCamera& cam, const UiState& st);
    void drawTriad(UiCommands& cmd, const OrbitCamera& cam);
    void drawControlsWindow(UiState& st);
    void drawAboutWindow(UiState& st);

    Toolbox m_toolbox;
    float m_toolboxWidth = 0.0f;
    float m_menuHeight = 0.0f;
    float m_toolbarHeight = 0.0f;
    float m_statusHeight = 0.0f;
};

} // namespace sicdm

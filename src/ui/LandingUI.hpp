#pragma once

#include "core/Camera.hpp"
#include "core/Scene.hpp"
#include "render/Renderer.hpp"
#include "ui/FileDialog.hpp"
#include "ui/Toolbox.hpp"
#include "ui/ViewCube.hpp"

#include <optional>
#include <string>

namespace sicdm {

// The two view windows of a document.
enum class ViewMode { Model3D, Drawing2D };

// Per-frame state the UI displays.
struct UiState {
    ViewMode mode = ViewMode::Model3D;
    SketchPlane plane = SketchPlane::Top; // active sketch / drawing plane
    bool showModelTree = false; // docked Model panel beside the tool rail
    bool showControls = false;
    bool showAbout = false;
    bool snapGrid = true;
    bool snapPoints = true;
    int selectedBody = -1;
    int selectedCurve = -1;
    int selectedImage = -1;

    // Command line: prompt for the current step, and the output log.
    struct LogLine {
        std::string text;
        enum Kind { Input, Output, Error } kind = Output;
    };
    std::string commandPrompt = "Command:";
    bool commandKeepsFocus = false; // a sketch tool is waiting for typed points

    // Extrude tool options.
    float extrudeDistance = 20.0f;
    bool extrudeSymmetric = false;
    bool extrudeKeepSketch = false;
    bool extrudeLive = true;          // mouse drag sets the distance (off once typed)
    std::string extrudeProfileName;   // empty until a profile is picked
    std::vector<LogLine> commandLog;

    // Project / file.
    std::string projectTitle = "Untitled";
    std::string projectPath;
    bool dirty = false;

    // MCP server for AI agents (live app over HTTP).
    bool mcpRunning = false;
    int mcpPort = 8765;
    std::string mcpError;
    std::string mcpExecutable; // path of sicdm-mcp next to the app, for setup hints
    std::optional<Vec2> cursorPlane; // cursor on the sketch plane (mm), when known
    float frameMs = 0.0f;
    std::string statusMessage; // transient feedback (e.g. "Placed Block 1")
    std::string gpuName;
    std::string glVersion;
};

// Actions requested by the UI, applied by the application after drawing.
struct UiCommands {
    std::optional<StandardView> view;
    std::optional<Vec3> viewDirection;
    std::optional<ViewMode> mode;
    std::optional<SketchPlane> plane;
    float rotateZ = 0.0f;
    float roll = 0.0f;
    bool resetRoll = false;
    Vec2 orbitDelta{};
    bool home = false;
    bool fit = false;
    bool toggleProjection = false;
    bool quit = false;
    std::vector<std::string> commands; // command lines to run (command bar, File menu)
    bool toggleMcp = false;
    bool applyExtrude = false;
    bool cancelExtrude = false;
};

// Workbench chrome around the canvas: menu bar, options bar (tool options and
// the 2D / 3D window switch), document tabs, rulers, tool rail, model tree,
// ViewCube and status bar.
class LandingUI {
public:
    UiCommands draw(Scene& scene, const OrbitCamera& camera, ViewportSettings& viewport, UiState& state);

    Toolbox& toolbox() noexcept { return m_toolbox; }

    // Canvas area (inside the chrome) in window coordinates, valid after draw().
    ImVec2 canvasMin() const noexcept { return m_canvasMin; }
    ImVec2 canvasMax() const noexcept { return m_canvasMax; }

private:
    void drawMenuBar(UiCommands& cmd, const OrbitCamera& cam, ViewportSettings& vp, UiState& st);
    void drawOptionsBar(UiCommands& cmd, const OrbitCamera& cam, ViewportSettings& vp, UiState& st);
    void drawDocumentTabs(UiCommands& cmd, const UiState& st);
    void drawRulers(const OrbitCamera& cam, const UiState& st);
    float drawModelTree(Scene& scene, UiState& st, float x, float top, float bottom); // returns width
    void drawCommandBar(UiCommands& cmd, UiState& st);
    void drawFileDialogs(UiCommands& cmd, UiState& st);
    void drawMcpWindow(UiState& st);
    void requestFile(int action, const UiState& st); // see FileAction
    static int commandInputCallback(struct ImGuiInputTextCallbackData* data);
    void drawStatusBar(const Scene& scene, const OrbitCamera& cam, const UiState& st);
    void drawControlsWindow(UiState& st);
    void drawAboutWindow(UiState& st);

    Toolbox m_toolbox;
    ViewCube m_viewCube;
    std::optional<ViewMode> m_tabMode; // tab the tab bar currently shows
    float m_toolboxWidth = 0.0f;
    float m_leftDockWidth = 0.0f; // rail + docked Model panel
    float m_menuHeight = 0.0f;
    float m_optionsHeight = 0.0f;
    float m_rightGroupWidth = 560.0f;
    float m_tabsBottom = 0.0f;
    float m_statusHeight = 0.0f;
    ImVec2 m_canvasMin{}, m_canvasMax{};
    float m_commandHeight = 0.0f;

    // Command bar state.
    char m_commandBuf[512] = "";
    bool m_focusCommand = false;
    bool m_showHistory = false;
    double m_showRecentUntil = 0.0;
    std::size_t m_seenLog = 0;
    std::vector<std::string> m_recall; // previous inputs for Up / Down
    int m_recallPos = -1;

    // File menu.
    enum FileAction { FileNone, FileOpen, FileSaveAs, FileImport, FileImportImage, FileExport };
    FileDialog m_fileDialog;
    int m_fileAction = FileNone;
    int m_confirmAction = FileNone; // New (-1) / Open waiting for "discard changes?"
    bool m_confirmOpen = false;
    bool m_showMcp = false;
};

} // namespace sicdm

#pragma once

#include "core/Camera.hpp"
#include "core/Scene.hpp"
#include "render/Renderer.hpp"
#include "ui/FileDialog.hpp"
#include "ui/Toolbox.hpp"
#include "ui/ViewCube.hpp"

#include <map>
#include <optional>
#include <string>

namespace sicdm {

// The view windows of a document.
enum class ViewMode { Model3D, Drawing2D, Simulation };

// Colour of the i-th motion (panel, axes and paths share it).
ImVec4 motionColor(std::size_t i);

// Per-frame state the UI displays.
struct UiState {
    ViewMode mode = ViewMode::Model3D;
    SketchPlane plane = SketchPlane::Top; // active sketch / drawing plane
    bool showModelTree = false; // docked panel beside the tool rail
    int dockTab = 0;            // its page: 0 Model, 1 Library
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

    // Simulation window: playback and what to show while bodies move.
    float simTime = 0.0f;      // s
    bool simPlaying = false;
    float simSpeed = 1.0f;     // playback factor
    bool simLoop = true;       // restart at the end of the timeline
    float simDuration = 10.0f; // effective timeline length (s)
    float simDurationSetting = 0.0f; // saved length; 0 = automatic
    bool simAxes = true;       // real-time axis triads on moving bodies
    bool simMotionAxes = true; // rotation axes / travel arrows
    bool simPaths = false;     // trails of the moving bodies
    bool simReadout = true;    // live axis readout on the canvas

    // Project / file.
    std::string projectTitle = "Untitled";
    std::string projectName = "Untitled"; // file stem, or the name of an unsaved file
    std::string projectPath;
    bool dirty = false;

    // Open files, one tab each, shown at the top of the canvas.
    struct FileTabInfo {
        int id = 0;
        std::string name; // "part.sicdm" or "Untitled 2"
        std::string path;
        bool dirty = false;
        ViewMode mode = ViewMode::Model3D;
    };
    std::vector<FileTabInfo> files;
    int activeFile = 0; // index into files

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
    std::optional<int> switchFile; // file tab id; applied first
    std::optional<int> closeFile;  // file tab id; applied last, only if saved unless closeDiscard
    bool closeDiscard = false;
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
    void drawFileTabs(UiCommands& cmd, const UiState& st);
    void requestClose(UiCommands& cmd, const UiState& st, int index);
    void drawRulers(const OrbitCamera& cam, const UiState& st);
    float drawModelTree(UiCommands& cmd, Scene& scene, UiState& st, float x, float top, float bottom); // returns width
    void drawLibrary(UiCommands& cmd, const Scene& scene, UiState& st);
    float drawMotionPanel(Scene& scene, UiCommands& cmd, UiState& st, float top, float bottom); // returns width
    void drawSimTransport(UiState& st);
    void drawCommandBar(UiCommands& cmd, UiState& st);
    void drawFileDialogs(UiCommands& cmd, UiState& st);
    void drawMcpWindow(UiState& st);
    void requestFile(int action, const UiState& st); // see FileAction
    static int commandInputCallback(struct ImGuiInputTextCallbackData* data);
    void drawStatusBar(UiCommands& cmd, const Scene& scene, const OrbitCamera& cam, const UiState& st);
    void drawControlsWindow(UiState& st);
    void drawAboutWindow(UiState& st);

    Toolbox m_toolbox;
    ViewCube m_viewCube;
    int m_shownFile = -1; // file tab id the tab bar currently shows
    int m_shownDockTab = -1; // dock page the tab bar currently shows
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
    int m_closeAsk = -1;          // file tab id waiting for "save changes?"
    bool m_closeAskOpen = false;
    int m_closeAfterSave = -1;    // file tab id to close once Save As succeeds
    bool m_showMcp = false;

    // Library page: selected item and its edited parameters.
    std::string m_libSelected = "dc_motor";
    bool m_libParamsOpen = true;
    std::map<std::string, std::map<std::string, float>> m_libValues;
    char m_libSearch[64] = "";
};

} // namespace sicdm

#pragma once

#include "core/Keymap.hpp"

#include "core/Assembly.hpp"
#include "core/Cam.hpp"
#include "core/Camera.hpp"
#include "core/Drawing.hpp"
#include "core/FileIO.hpp"
#include "core/Lathe.hpp"
#include "core/Pdm.hpp"
#include "core/PickFeature.hpp"
#include "core/Scene.hpp"
#include "core/Selection.hpp"
#include "core/SectionView.hpp"
#include "core/Stackup.hpp"
#include "core/Study.hpp"
#include "render/Renderer.hpp"
#include "ui/FileDialog.hpp"
#include "ui/Toolbox.hpp"
#include "ui/ViewCube.hpp"

#include <map>
#include <memory>
#include <optional>
#include <set>
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
    PlaneFrame activeFrame = planeFrame(SketchPlane::Top); // its frame (a datum plane's when Custom)
    std::string planeLabel = "Top (XY)";  // its name
    std::vector<std::string> datums;      // the project's datum planes
    bool showModelTree = false; // docked panel beside the tool rail
    int dockTab = 0;            // its page: 0 Model, 1 Library, 2 CAM
    bool showControls = false;
    bool showAbout = false;
    // The drawing sheet window; the app keeps `sheet` drawn while it is open.
    bool showSheet = false;
    const SheetGraphics* sheet = nullptr;
    std::string sketchConstraintStatus; // "3 fully constrained, 2 free" ("" with no constraints)
    // Sketch tool options.
    bool sketchConstruction = false; // new curves are construction curves
    int polygonSides = 6;
    float sketchOffset = 5.0f;       // mm, the offset tool's distance
    // Assembly: the mates (the app points this at the document's), and the mate tool:
    // its type (COINCIDENT, FLUSH... ; empty: off), value, and the face picked first.
    const AssemblyState* assembly = nullptr;
    const ProjectState* project = nullptr; // the feature history for the model tree
    // Manufacturing: the jobs (the app points these at the document's), the operation picked
    // in the CAM panel (-1: none), and the tool library and machine windows.
    const CamJob* cam = nullptr;
    const LatheJob* lathe = nullptr;
    int camSelected = -1, latheSelected = -1;
    bool showCamTools = false, showCamMachine = false;
    // The user guide window and the page it shows ("cam.md", "cam.md#the-cam-panel").
    bool showHelp = false;
    bool showPreferences = false, showLicence = false;
    // The render window. The app renders the 3D view in the background when renderStart is
    // set (and stops when renderCancel is), and shows the picture (renderTexture, an OpenGL
    // texture); renderSavePath set: the app writes the picture there.
    bool showRender = false;
    bool showProduct = false; // the Product data window: properties, revision, vault
    bool showAppearance = false; // the Appearance window: the selected body's material from swatches
    bool showSection = false;    // the Section window: the live section view's plane
    // The Motion Study window: the last study (the app points this at the document's), the
    // line that made it, and the span asked for (to 0: the scene's own length).
    bool showStudy = false;
    std::shared_ptr<const StudyResult> study;
    std::string studyCommand;
    float studyFrom = 0.0f, studyTo = 0.0f;
    int studySamples = 600;
    // The Tolerance Stack-up window (its stacks are the project's).
    bool showStack = false;
    // The Armature window: the armature edited, its ball chosen (0...), the plane balls are
    // mirrored across (-1 none, 0 X, 1 Y, 2 Z), and clicks on it add balls while armaturePick is on.
    bool showArmature = false, armaturePick = false;
    // The Move tool's gizmo: 0 move, 1 rotate, 2 scale; snapping (also while Ctrl is held) to
    // these steps (mm, degrees, a factor).
    int gizmoMode = 0;
    bool gizmoSnap = false;
    bool snapMoveToPoints = true; // moves land on other bodies' corners, edge and face middles near the mouse
    float gizmoMoveStep = 1.0f, gizmoAngleStep = 15.0f, gizmoScaleStep = 0.1f;
    // Typed in the Move tool's bar and applied to the selection: a move (mm), a turn, a scale.
    Vec3 typedMove{0, 0, 0};
    float typedAngle = 90.0f;
    int typedAxis = 2; // X, Y, Z
    float typedScale = 2.0f;
    std::string armatureName;
    int armatureBall = 0, armatureMirror = 0;
    // The 3D Path window: points for a new 3D path (clicks on the model add them while
    // pathPick is on), and a route along the selected path.
    bool showPath3d = false, pathPick = false, pathSpline = false, pathClosed = false;
    std::vector<Vec3> pathPoints;
    float pathBend = 0.0f;
    bool renderStart = false, renderCancel = false, renderRunning = false;
    float renderProgress = 0;
    unsigned renderTexture = 0;
    int renderWidth = 0, renderHeight = 0;
    std::string renderInfo, renderSavePath;
    // Videos (in the render window): what is filmed (0 turntable, 1 the motions, 2 the motions
    // with the camera going round), its length, frame rate, turns, size and samples, and the
    // format (0 AVI, 1 animated PNG, 2 GIF, 3 PNG frames). videoPath set: the app makes it
    // in the background (videoCancel stops it).
    int videoKind = 0, videoFps = 24, videoSize = 1, videoSamples = 16, videoFormat = 0;
    float videoSeconds = 6.0f, videoTurns = 1.0f;
    std::string videoPath, videoInfo;
    bool videoRunning = false, videoCancel = false;
    float videoProgress = 0;
    std::string helpPage = "getting-started.md";
    std::string mateTool;
    float mateValue = 0.0f;
    std::string mateFirst;    // the first part's name ("" until picked)
    Vec3 mateFirstPoint{};
    bool snapGrid = true;
    bool snapPoints = true;
    int selectedBody = -1;
    BodySelection bodySelection; // other bodies selected with selectedBody (Selection.hpp)
    BodySelection curveSelection; // other sketch curves selected with selectedCurve
    int renameBody = -1;         // asks for a new name for this body (F2, double-click in the tree)
    bool openBodyMenu = false;   // a right click on a body: open the body menu (for the selection)
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

    // Pick tools (revolve, sweep, loft, rib, surfaces): options, and the curves picked so far.
    PickFeatureOptions pick;
    std::vector<std::string> pickedNames;
    bool pickCanApply = false; // enough picked for Apply (loft, B-spline patch)
    std::vector<LogLine> commandLog;

    // Sculpting brushes (options bar). Ctrl inverts a brush, Shift smooths.
    float sculptRadius = 20.0f;   // mm
    float sculptStrength = 0.5f;  // 0..1
    bool sculptPressureSize = false;     // a tablet pen's pressure sets the brush size...
    bool sculptPressureStrength = true;  // ...and its strength
    bool sculptSymmetry = false;  // mirror across the body's centre (X)
    bool sculptSymmetryY = false, sculptSymmetryZ = false;
    int sculptRadial = 1;         // radial copies around the body's Z axis (1: none)
    int sculptFalloff = 0;        // SculptFalloff
    bool sculptRefine = true;     // dynamic topology under the brush
    float sculptDetail = 0.0f;    // longest edge under the brush (mm); 0 = radius / 4
    float sculptColor[3] = {0.80f, 0.20f, 0.20f}; // the Paint brush's colour
    int sculptAlpha = 0;          // 0: none, else 1 + index into sculptAlphaNames()
    float sculptAlphaAngle = 0.0f; // degrees
    bool sculptAlphaFollow = false; // turn the alpha along the stroke
    bool sculptStamp = false;       // drag out one stamp (its size and turn) instead of a stroke
    std::string sculptInfo;       // triangles / closed state of the body being edited
    // Cut tools.
    float holeDiameter = 6.0f, holeDepth = 10.0f;
    bool holeThrough = true;
    int pocketShape = 0; // 0 box, 1 round, 2 ball
    float pocketWidth = 20.0f, pocketLength = 20.0f, pocketDepth = 5.0f, pocketAngle = 0.0f;
    float channelWidth = 4.0f, channelDepth = 0.0f; // 0 = half the width (a round groove)
    int bossShape = 0; // 0 cylinder, 1 box, 2 dome
    float bossDiameter = 10.0f, bossHeight = 8.0f;
    // Insert Mesh and Curve Brush: the shape (0 sphere, 1 cube, 2 cylinder, 3 cone, 4 torus,
    // 5 the body insertBody), its size, how far it sinks in (0..1), how it joins (0 union, 1
    // append as a polygroup, 2 a new body); the curve's tube or row, radius and spacing (0: the size).
    int insertShape = 0, insertJoin = 0;
    std::string insertBody;
    float insertSize = 8.0f, insertSink = 0.15f;
    bool curveTube = true;
    float curveRadius = 1.5f, curveSpacing = 0.0f;
    int curveProfile = 0;                       // 0 round, 1 square, 2 flat, 3 triangle, 4 star
    float curveTaperStart = 1.0f, curveTaperEnd = 1.0f; // the tube's size at the stroke's start and end (x radius)
    int booleanOp = 1; // 0 union, 1 subtract, 2 intersect
    bool booleanKeep = false;
    std::string booleanFirst; // the body picked first
    // Armour & exoskeleton tools.
    float shellThickness = 3.0f;
    bool shellOpen = false;    // cut the clicked side open (helmets, gauntlets)
    float shellOpenDepth = 2.0f;
    bool shellOutward = false;
    float armorClearance = 4.0f, armorThickness = 3.0f;
    int jointAxis = 0; // 0 X, 1 Y, 2 Z, 3 the view direction
    bool jointBall = false;
    float jointLo = -45.0f, jointHi = 45.0f, jointPeriod = 4.0f;
    float latticeSkin = 2.0f, latticeCell = 12.0f, latticeWall = 1.5f;
    bool latticeHollow = false;
    float interferenceClearance = 2.0f;

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
    bool applyPick = false;
    bool clearPick = false;
    std::optional<int> switchFile; // file tab id; applied first
    std::optional<int> closeFile;  // file tab id; applied last, only if saved unless closeDiscard
    bool closeDiscard = false;
    std::optional<SectionView> section; // the section plane dragged in the Section window (set as it moves)
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
    void drawRenderWindow(UiCommands& cmd, UiState& st);
    void drawSectionWindow(UiCommands& cmd, UiState& st, const Scene& scene);
    void drawRenamePopup(UiCommands& cmd, UiState& st, const Scene& scene);
    void drawKeyboardShortcuts(UiCommands& cmd);
    void drawBodyMenuItems(UiCommands& cmd, UiState& st, const Scene& scene); // the right-click menu of the selected bodies
    void drawStudyWindow(UiCommands& cmd, UiState& st, const Scene& scene);
    void drawStackWindow(UiCommands& cmd, UiState& st, const Scene& scene);
    void drawArmatureWindow(UiCommands& cmd, UiState& st, const Scene& scene);
    void drawPath3dWindow(UiCommands& cmd, UiState& st, const Scene& scene);
    void drawProductWindow(UiCommands& cmd, UiState& st); // ProductWindow.cpp
    void drawAppearanceWindow(UiCommands& cmd, UiState& st); // AppearanceWindow.cpp
    void drawOptionsBar(UiCommands& cmd, const OrbitCamera& cam, ViewportSettings& vp, UiState& st);
    void drawFileTabs(UiCommands& cmd, const UiState& st);
    void requestClose(UiCommands& cmd, const UiState& st, int index);
    void drawRulers(const OrbitCamera& cam, const UiState& st);
    void drawFeatureHistory(UiCommands& cmd, const UiState& st);
    float drawModelTree(UiCommands& cmd, Scene& scene, UiState& st, float x, float top, float bottom); // returns width
    void drawLibrary(UiCommands& cmd, const Scene& scene, UiState& st);
    void drawCamPanel(UiCommands& cmd, const Scene& scene, UiState& st);   // the dock's CAM page (CamPanel.cpp)
    void drawCamOperations(UiCommands& cmd, UiState& st);
    void drawLatheOperations(UiCommands& cmd, UiState& st);
    void drawCamToolsWindow(UiCommands& cmd, UiState& st);
    void drawCamMachineWindow(UiCommands& cmd, UiState& st);
    void drawHelpWindow(UiCommands& cmd, UiState& st); // HelpWindow.cpp
    void runSample(UiCommands& cmd, const std::string& file);
    void drawPreferencesWindow(UiCommands& cmd, UiState& st); // PreferencesWindow.cpp
    void drawLicenceWindow(UiCommands& cmd, UiState& st);
    void drawViewBar(UiCommands& cmd, const OrbitCamera& cam, ViewportSettings& vp, UiState& st, float y);
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
    void drawMateWindow(UiCommands& cmd, UiState& st);
    const Scene* m_menuScene = nullptr; // the scene the menus name parts from (set each frame)
    std::vector<VaultItem> m_vaultItems; // the vault as the Product data window last read it
    std::string m_vaultWhy;
    bool m_vaultDirty = false;
    void drawSheetWindow(UiCommands& cmd, UiState& st);
    // Drawing sheet window: zoom (pixels per mm), pan (mm at the canvas's top left), and
    // the dimension tool's first point.
    float m_sheetZoom = 0.0f;
    float m_sheetPanX = 0.0f, m_sheetPanY = 0.0f;
    bool m_sheetDimTool = false; // any annotation tool (below) is active
    int m_sheetDimFirst = -1;
    // Annotation tools: 0 pan, 1 dimension, 2 feature control frame, 3 datum, 4 surface finish.
    int m_sheetTool = 0;
    char m_sheetTol[48] = "";        // the dimension tool's tolerance: 0.1, +0.1 -0.05, FIT H7 ...
    int m_gdtSymbol = 9;             // position
    float m_gdtValue = 0.05f;
    bool m_gdtDiameter = true;
    int m_gdtMaterial = 0;           // none, MMC, LMC
    char m_gdtDatums[24] = "A B";
    char m_datumLetter[4] = "A";
    float m_finishRa = 1.6f;
    int m_finishProcess = 1;

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
    float m_rightRailWidth = 0.0f;
    static constexpr float kViewBarHeight = 34.0f;
    float m_viewBarWidth = 600.0f; // the view bar's buttons last frame (to right-align them)
    float m_statusHeight = 0.0f;
    ImVec2 m_canvasMin{}, m_canvasMax{};
    float m_commandHeight = 0.0f;

    // Command bar state.
    char m_commandBuf[512] = "";
    bool m_focusCommand = false;
    bool m_showHistory = false;
    double m_showRecentUntil = 0.0;
    std::size_t m_seenLog = 0;
    char m_renameText[128] = {};
    // Preferences > Keyboard Shortcuts: the action whose keys are being set, a key it would
    // share with another (asked first), and the list's filter.
    std::string m_keyCapture;
    std::optional<KeyChord> m_keyShared;
    char m_keyFilter[64] = {};
    int m_renameShown = -1; // the body the rename box was filled for
    std::vector<std::string> m_recall; // previous inputs for Up / Down
    int m_recallPos = -1;

    // File menu.
    enum FileAction {
        FileNone, FileOpen, FileSaveAs, FileImport, FileImportImage, FileExport, FileSlice, File3mf, FilePost, FileImportStep, FileExportStep, FileSheetExport,
        FileLathePost, FileToolsImport, FileToolsExport, FileLicence, FileInsertComponent, FileMakeComponent, FilePackComponents,
        FileStudyExport, FileStackExport, FileExportModel, FileVideo
    };
    FileDialog m_fileDialog;
    int m_fileAction = FileNone;
    std::string m_makeComponentBody; // the body FileMakeComponent saves as a part file
    // Design menu inputs.
    float m_revolveAngle = 360.0f, m_filletRadius = 2.0f, m_patternSpacing = 20.0f, m_patternAngle = 360.0f;
    int m_weldFamily = 0, m_weldSize = 6, m_weldCorner = 0; // the Weldment menu's profile (family, size) and corners
    int m_routeKind = 0, m_routeSize = 4;                    // the 3D Path window's route: kind and size
    float m_routeBend = 0.0f;                                // its bends (0: the path's own, else the size's default)
    int m_moldPull = 0;                                      // the Mold Tools menu: the pull (+Z, -Z, +X, -X, +Y, -Y),
    float m_moldMin = 1.0f, m_moldShrink = 0.0f;             // the least draft (degrees) and the shrinkage (%)
    int m_fastType = 1, m_fastSize = 6;                      // the Fasteners menu: the type (FastenerType) and its size,
    float m_fastLength = 0.0f;                               // the length (mm, 0: picked from the size)
    bool m_fastWasher = false, m_fastNut = false;            // and what goes with them into holes
    std::set<std::string> m_studyPick;                       // the Motion Study window: the motions ticked
    std::string m_stackName;                                 // the Tolerance Stack-up window: the stack shown,
    std::string m_stackKey;                                  // what its last analysis was of (stacksToJson),
    StackResult m_stackResult;                               // and that analysis
    char m_stackNew[64] = "";                                // a new stack's name as it is typed
    int m_stackAxis = 0;                                     // the size of the selected body along X, Y or Z
    int m_stackGdt = 0;                                      // Add a Geometric Tolerance: the characteristic
    float m_stackZone = 0.1f, m_stackBonus = 0.0f;           // ... its zone, and the bonus with MMC/LMC
    int m_stackMaterial = 0;                                 // ... 0 RFS, 1 MMC, 2 LMC
    int m_armSide = 0;                                       // the Armature window: the side a new ball goes on (+X ... -Z)
    float m_armVoxel = 0.0f, m_armBlend = -1.0f;             // the skin's voxel and blend (0 / -1: from the balls)
    int m_armSmooth = 6;                                     // and its smoothing passes
    int m_armDetailFrom = 2;                                 // the ball whose sculpted detail Copy Detail takes
    bool m_armLinear = false;                                // new skins posed linearly (as glTF viewers) rather than with dual quaternions
    int m_patternCount = 3, m_patternAxis = 0;
    int m_closeAsk = -1;          // file tab id waiting for "save changes?"
    bool m_closeAskOpen = false;
    int m_closeAfterSave = -1;    // file tab id to close once Save As succeeds
    bool m_showMcp = false;
    float m_scaleEdit = 1.0f;     // Model panel: the selected body's Scale field while it is edited
    bool m_scaleEditing = false;
    std::string m_scaleEditBody;

    // CAM panel: a copy of the picked operation that the fields edit; Apply sends the
    // differences (CAM EDIT). It follows the job while nothing is changed.
    CamOperation m_camEdit;
    int m_camEditIndex = -1;
    bool m_camEditDirty = false;
    std::size_t m_camOpsSeen = 0; // a new operation gets picked
    LatheOperation m_latheEdit;
    int m_latheEditIndex = -1;
    int m_historyEdit = -1;        // the feature history step being edited (its popup)
    char m_historyEditLine[1024] = {}; // the edited command line
    bool m_latheEditDirty = false;
    std::size_t m_latheOpsSeen = 0;
    bool m_latheCycles = false;   // post rough turning and facing as G71 / G72
    // Tool library window: a copy of the picked tool the fields edit (Apply sends it as
    // CAM TOOL), and the user's tool library as last read.
    CamTool m_toolEdit;
    int m_toolEditNumber = -1;
    bool m_toolEditDirty = false;
    std::vector<CamTool> m_userLibrary;
    bool m_userLibraryRead = false;
    std::string m_userLibraryError;
    // Machine window: edits before Apply (CAM MACHINE).
    CamMachine m_machineEdit;
    bool m_machineEditDirty = false;

    // User guide: the page shown (its file and the heading to scroll to), the pages before it
    // (Back), and the search text.
    std::string m_helpShown;
    std::string m_helpAnchor;
    std::vector<std::string> m_helpBack;
    char m_helpSearch[64] = "";

    // Library page: selected item and its edited parameters.
    std::string m_libSelected = "dc_motor";
    bool m_libParamsOpen = true;
    std::map<std::string, std::map<std::string, float>> m_libValues;
    char m_libSearch[64] = "";
};

} // namespace sicdm

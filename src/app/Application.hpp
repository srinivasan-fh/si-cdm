#pragma once

#include "core/Render.hpp"
#include "core/Video.hpp"
#include "core/Camera.hpp"
#include "core/Constraints.hpp"
#include "core/Document.hpp"
#include "core/Gizmo.hpp"
#include "core/Sculpt.hpp"
#include "core/Mcp.hpp"
#include "core/Recovery.hpp"
#include "core/CrashReport.hpp"
#include "core/Scene.hpp"
#include "app/McpHttpServer.hpp"
#include "app/SketchController.hpp"
#include "app/Splash.hpp"
#include "render/Renderer.hpp"
#include "ui/LandingUI.hpp"
#include "core/UpdateCheck.hpp"

#include <atomic>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <functional>
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
    std::vector<std::string> commands; // command lines to run after it (--command), e.g. for tests and screenshots
    std::string executablePath; // argv[0], for MCP setup hints
    bool splash = true;         // cinematic launch splash with loading progress
    float splashSeconds = 3.2f; // shortest time the splash stays up
    std::string language;       // the interface's language for this run ("": the preference)
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
    void orient2D(bool animate); // the 2D view square to the sketch plane
    void fit() override;
    void zoomStep(float steps) override;
    void home() override;
    void roll(float degrees, bool absolute) override;
    void setProjection(Projection p) override;
    void setDisplay(int mode) override;
    void setGrid(std::optional<bool> on) override;
    bool setPolygroups(std::optional<bool> on) override;
    const OrbitCamera* viewCamera() const override { return &m_cam3D; }
    bool setBodyTexture(const std::string& body, bool checker, const std::string& colorPath, const std::string& normalPath, std::string* error) override;
    void setPanel(std::optional<bool> on) override;
    void setPanelPage(int page) override;
    bool setGizmo(int mode) override;
    std::string describe() const override;
    bool simulation(const std::string& action, float value, std::string* reply) override;
    bool showSheet(bool on) override;

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
    // Sketch geometry under the cursor that the Select tool can drag: a point of a curve
    // (an end, corner, vertex or centre), a circle's or arc's rim, or a whole curve.
    struct SketchGripHit {
        SketchGrip grip;
        Vec2 from; // the plane point grabbed (mm)
        int curve = -1;
    };
    std::optional<SketchGripHit> pickSketchGrip(float x, float y) const;
    void updateSketchDrag(float x, float y);
    void finishSketchDrag();
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
    void pickFeatureClick(ToolId tool, float x, float y);
    void applyPickFeature(const std::string& body = {});
    void clearPicks();
    void drawExtrudePreview() const;

    // Simulation window.
    bool simulating() const noexcept { return m_uiState.mode == ViewMode::Simulation; }
    void advanceSimulation(float dt);
    // The exploded view (EXPLODE SHOW / HIDE): eased open and shut, drawn by moving the bodies.
    void advanceExplode(float dt);
    void drawExplodeOverlay() const;
    // How each body is drawn moved: the simulation's motion, or the exploded view (nullptr: as they are).
    const std::vector<Mat4>* bodyTransforms() const;
    void drawSimulationOverlay() const;
    void drawToolpathOverlay() const;
    void drawDatumOverlay() const;
    void drawFieldLegend() const;
    void drawConstraintOverlay() const;
    // What Fit / Home frame: the model, and in the Simulation where physics bodies go.
    AABB viewBounds() const;
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
    void autosave(); // keeps recovery copies of files with unsaved changes
    void watchComponents(); // reloads linked part files that changed on disk

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
    std::unique_ptr<Splash> m_splash; // only while launching
    LandingUI m_ui;
    UiState m_uiState;
    // The drawing sheet's graphics, redrawn when the drawing or the model changes.
    SheetGraphics m_sheet;
    std::uint64_t m_sheetKey = 0;
    void updateSheet();
    // Sketch curves coloured by how the constraints hold them.
    std::uint64_t m_sketchColorKey = 0;
    // Dragging sketch geometry: the press's grip, the sketch as it was, and where the
    // grip was last asked to go (each frame solves afresh from the start, as the DRAG
    // command recorded on release does).
    std::optional<SketchGripHit> m_gripCandidate;
    SketchGripHit m_sketchDrag;
    Sketch m_sketchDragStart;
    Vec2 m_sketchDragTo{};
    bool m_sketchDragMoved = false;
    void updateSketchColors();

    // Input state
    enum class DragMode { None, Pending, Orbit, Pan, Zoom, MoveBody, MoveImage, Sculpt, Grab, CutPath, Knife, SketchGrip, PushPull, Gizmo, BoxSelect, Lasso };
    DragMode m_leftDrag = DragMode::None;
    bool m_escapedDrag = false; // Esc cancelled the drag; the button is still held
    DragMode m_otherDrag = DragMode::None; // right / middle button
    float m_pressX = 0, m_pressY = 0;
    float m_pendingScroll = 0;
    Vec3 m_movePlanePoint;
    bool m_rightClickOnCanvas = false; // the right button went down on the 3D view (a click there opens the body menu)
    std::vector<int> m_moveBodies;    // the Move tool drags these (the selection, when the body grabbed is in it)
    std::vector<Vec3> m_moveStarts;   // where they were
    Vec2 m_imageGrab{}, m_imageStart{};

    std::vector<Mat4> m_simTransforms; // per body, while the Simulation window shows motion
    // Push / pull: the face pressed on, the way it moves and how far the drag has taken it.
    struct PushPullDrag {
        int body = -1;
        Vec3 at{};          // where it was pressed (the body as it is)
        Vec3 dir{0, 0, 1};  // the face's outward normal there
        ImVec2 start{};     // the press on screen
        ImVec2 screenDir{}; // the normal on screen, unit
        float pxPerMm = 1;  // screen pixels a millimetre along it
        float distance = 0;
        std::string what;   // "flat face", "hole"...
    };
    PushPullDrag m_push;
    bool beginPushPull(float x, float y);
    void updatePushPull(float x, float y);
    void finishPushPull();
    void drawPushPullOverlay() const;
    std::vector<Mat4> m_explodeTransforms; // per body, while the exploded view shows (or opens or shuts)
    std::vector<DVec3> m_explodeShifts;    // the same as moves
    float m_explodeT = 0.0f;               // 0 together .. 1 exploded
    bool m_explodeMoving = false;          // opening or shutting
    bool m_viewMoving = false;             // a view change (Fit, a standard view) still under way
    // Chains / belts travelling along their loop: shape per body for this frame.
    struct LoopShape {
        float travel = 0.0f;
        std::uint64_t revision = 0;
        MeshData mesh;
        bool valid = false;
    };
    std::vector<LoopShape> m_loopShapes;
    std::vector<const MeshData*> m_loopOverrides;
    void updateLoopShapes();

    // Sculpting stroke in progress (a welded copy of the body, written back each frame).
    int m_sculptBody = -1;
    SculptMesh m_sculptMesh;
    SculptMesh m_stampBase;   // Stamp: the surface before the stamp being dragged out
    bool m_stamping = false;
    // The radial control (F: brush size, Shift+F: strength, as in Blender): the value follows
    // how far the pointer is from a circle's centre; a click or Enter keeps it, Esc or a right
    // click puts it back.
    enum class Radial { None, Size, Strength };
    Radial m_radial = Radial::None;
    ImVec2 m_radialCentre{};
    float m_radialWas = 0.0f;      // the value before, for Esc
    float m_radialPerPixel = 1.0f; // mm a pixel at the brush (Size)
    static constexpr float kRadialStrengthSpan = 200.0f; // pixels from 0 to full strength
    void startRadial(Radial mode);
    bool updateRadial(); // true while it has the mouse
    SculptSettings m_sculptSettings;
    Vec3 m_lastDab{};
    int m_dabs = 0;
    Vec3 m_grabPoint{};
    std::vector<const MeshData*> m_sculptOverrides; // streams the edited body while dragging
    // The stroke's body as it was picked: a stroke whose body was renamed, removed or
    // changed underneath it (an AI agent's command, an undo) is dropped, not misapplied.
    std::string m_sculptName;
    std::uint64_t m_sculptRevision = 0;
    std::optional<Vec3> m_brushHit;  // hover point of a sculpt / cut tool
    Vec3 m_brushNormal{};
    bool m_brushNormalValid = false;
    // Channel stroke and knife line.
    int m_cutBody = -1;
    ToolId m_cutTool = ToolId::CutChannel; // the tool whose path m_cutPath is (Channel or Curve Brush)
    std::vector<Vec3> m_cutPath;
    ImVec2 m_knifeStart{};
    ImVec2 m_boxStart{};    // a Ctrl+drag box selecting bodies: where it began
    bool beginSculpt(float x, float y);
    void continueSculpt(float x, float y);
    void endSculpt();
    bool sculptTargetValid() const;
    void abortSculpt(); // drops the stroke and restores the body
    // Ends whatever the left button is doing: a stroke is kept (`commit`) or
    // dropped; channel paths, knife lines and a half-picked boolean are forgotten.
    void finishMoveDrag(bool commit); // the move tool let go (commit) or cancelled
    void cancelInteraction(bool commit);
    // The move / rotate / scale gizmo on the selected body while the Move tool is on: where it
    // is and how long (mm, a fixed size on screen); false when none shows.
    bool gizmoShown(Vec3& pivot, float& size) const;
    GizmoProject gizmoProjector() const;
    void drawGizmo();
    void finishGizmoDrag(bool commit); // let go: the drag becomes its command (MOVE, ROTATE, SCALE)
    GizmoHandle m_gizmoHover, m_gizmoHandle;
    std::vector<int> m_gizmoBodies;   // what it drags: every selected body, the active one first
    std::vector<Body> m_gizmoStarts;  // them as the drag began (the preview starts from them each frame)
    int m_gizmoBody = -1;
    Vec3 m_gizmoPivot{};
    float m_gizmoSize = 0;
    GizmoRay m_gizmoFrom{};
    GizmoDelta m_gizmoDelta{};
    // Snapping a move to other bodies' points: what moves and what it can land on (taken as
    // the drag starts), and the pair it snapped to this frame.
    std::vector<Vec3> m_snapMoving, m_snapTargets;
    std::vector<std::int8_t> m_snapSeen; // per target: -1 not asked yet, 1 in sight, 0 hidden behind something
    std::vector<int> m_snapMovingBodies;
    bool m_snapXray = false; // the last snap was seen through (Alt held)
    // A move with no line or plane to keep to: onto a point in sight, in full 3D; else onto a
    // hidden one, within the view.
    // A free drag's snap: onto a point in sight in full 3D, else a hidden one within the view;
    // `xray` (Alt held): onto any point in full 3D, hidden or not.
    std::optional<GizmoSnapHit> freeSnap(const Vec3& move, bool xray = false);
    std::optional<GizmoSnapHit> m_snapHit;
    void startSnap(const std::vector<int>& moving);
    void drawSnapMarker();
    void applyCutTool(ToolId tool, float x, float y);
    void applyWearableTool(ToolId tool, float x, float y);
    void finishCutDrag(ToolId tool, float x, float y);
    void drawSculptOverlay();

    int m_extrudeCurve = -1;   // profile being extruded
    // Pick tools: the tool and the curves picked for it so far.
    ToolId m_pickTool = ToolId::Count;
    std::vector<int> m_picks;
    float m_extrudeAnchorDist = 0.0f;
    float m_extrudeAnchorX = 0.0f, m_extrudeAnchorY = 0.0f;

    // MCP server (shared by the HTTP transport) and unsaved-change tracking.
    std::unique_ptr<McpServer> m_mcp;
    McpHttpServer m_mcpHttp;
    std::uint64_t m_savedSignature = 0;
    std::string m_shownTitle;
    bool m_closeRequested = false;
    // Autosave of unsaved work, and what an earlier session that ended badly left.
    Autosaver m_autosaver;
    double m_autosaveCheck = 0.0;
    double m_componentCheck = 0.0;
    std::string m_componentTried; // the changes last reloaded (or refused), so each is tried once
    std::future<UpdateResult> m_updateCheck; // the check at start (Preferences), while it runs
    // The render window's picture: made in the background from a copy of the scene.
    void pollRender();
    void pollVideo();
    void finishBoxSelect(float x, float y);
    void finishLassoSelect();
    // Selects what lies in a region of the screen (`inside`), wholly (`window`) or touching it;
    // added to the selection (`add`) or in its place.
    void selectInRegion(const std::function<bool(const ImVec2&)>& inside, bool window, bool add, const ImVec2& middle);
    std::vector<ImVec2> m_lasso;   // the lasso's points as drawn
    bool m_regionAdd = true;       // the box or lasso adds to the selection (Ctrl / Shift, or Ctrl+drag)
    void selectionChanged(); // says what is selected now
    struct RenderJob {
        std::vector<std::uint8_t> pixels;
        int width = 0, height = 0, samples = 0;
        RenderStats stats;
        std::string why;
        std::atomic<float> progress{0};
        std::atomic<bool> stop{false};
    };
    std::shared_ptr<RenderJob> m_renderJob;
    // A video made in the background (the render window's Make Video).
    struct VideoJob {
        VideoReport report;
        std::string why, path;
        std::atomic<float> progress{0};
        std::atomic<bool> stop{false};
    };
    std::shared_ptr<VideoJob> m_videoJob;
    std::future<bool> m_videoDone;
    std::future<bool> m_renderDone;
    std::vector<std::uint8_t> m_renderPixels; // the last picture (to save)
    int m_renderWidth = 0, m_renderHeight = 0;
    unsigned m_renderTexture = 0;
    bool m_autosaveOn = true;
    int m_commandFailures = 0; // commands that failed (a scripted --screenshot run exits with 3)
    std::vector<RecoveryFile> m_recoverable;
    bool m_offerRecovery = false;
    std::vector<CrashReportFile> m_newCrashes; // reports from earlier sessions not yet shown
    bool m_offerCrashes = false;

    double m_statusUntil = 0.0;

    // On-demand rendering: CAD viewports only redraw when something changes.
    int m_framesToRender = 3;
};

} // namespace sicdm

#include "app/Application.hpp"
#include "app/PenInput.hpp"
#include "core/Armature.hpp"
#include "core/Components.hpp"
#include "core/DirectEdit.hpp"
#include "core/Mold.hpp"
#include "core/SectionView.hpp"
#include "core/Translate.hpp"
#include "app/OpenFiles.hpp"

#include "core/CrashReport.hpp"

#include "render/GL.hpp"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include "core/Command.hpp"
#include "core/ImageWrite.hpp"
#include "core/Render.hpp"
#include "core/FileIO.hpp"
#include "core/Library.hpp"
#include "core/Physics.hpp"
#include "core/Preferences.hpp"
#include "core/Units.hpp"
#include "ui/KeyInput.hpp"
#include "ui/Theme.hpp"
#include "core/Extrude.hpp"
#include "core/Json.hpp"
#include "core/SketchEdit.hpp"

#include <algorithm>
#include <filesystem>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iterator>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <vector>

namespace sicdm {

namespace {

void glfwErrorCallback(int code, const char* msg) {
    std::fprintf(stderr, "[glfw] error %d: %s\n", code, msg);
}


} // namespace

Application::Application(AppOptions options) : m_options(std::move(options)) {}

Application::~Application() { shutdown(); }

bool Application::init() {
    glfwSetErrorCallback(glfwErrorCallback);
    installOpenFileHandler(); // before GLFW sets up the application
    if (!glfwInit()) return false;

    // Launch splash while the workbench loads (not for scripted screenshots).
    if (m_options.splash && m_options.screenshotPath.empty()) {
        m_splash = std::make_unique<Splash>(m_options.splashSeconds);
        if (!m_splash->ok()) m_splash.reset();
    }
    auto progress = [this](float p, const std::string& status, const char* subsystem) {
        if (m_splash) m_splash->step(p, status.c_str(), subsystem);
    };
    progress(0.06f, "Initialising graphics", "GPU context");

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE); // required on macOS
    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_STENCIL_BITS, 8); // the section view's filled cuts
#if defined(__APPLE__)
    glfwWindowHint(GLFW_COCOA_RETINA_FRAMEBUFFER, GLFW_TRUE);
#endif
    if (m_splash) glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE); // shown when the splash closes

    m_window = glfwCreateWindow(m_options.width, m_options.height, "SI CDM — CAD / CAM Workbench", nullptr, nullptr);
    glfwDefaultWindowHints();
    if (!m_window) {
        reportStartupError("SI CDM needs OpenGL 4.1, which this graphics driver doesn't offer. Update the graphics driver.");
        return false;
    }
    applyWindowIcon(m_window);
    glfwMakeContextCurrent(m_window);
    glfwSwapInterval(1);
    if (const char* missing = nullptr; !loadGLFunctions(&missing)) {
        const std::string why = std::string("SI CDM needs OpenGL 4.1: the graphics driver has no ") + (missing ? missing : "?") +
                                ". Update the graphics driver.";
        reportStartupError(why.c_str());
        return false;
    }

    glfwSetWindowUserPointer(m_window, this);
    glfwSetScrollCallback(m_window, onScroll);
    // Any input or resize schedules redraws (on-demand rendering).
    glfwSetCursorPosCallback(m_window, [](GLFWwindow* w, double x, double y) {
        ImGui_ImplGlfw_CursorPosCallback(w, x, y);
        onEvent(w);
    });
    glfwSetMouseButtonCallback(m_window, [](GLFWwindow* w, int b, int a, int m) {
        ImGui_ImplGlfw_MouseButtonCallback(w, b, a, m);
        onEvent(w);
    });
    glfwSetKeyCallback(m_window, [](GLFWwindow* w, int k, int s, int a, int m) {
        ImGui_ImplGlfw_KeyCallback(w, k, s, a, m);
        onEvent(w);
    });
    glfwSetFramebufferSizeCallback(m_window, [](GLFWwindow* w, int, int) { onEvent(w); });
    glfwSetWindowRefreshCallback(m_window, [](GLFWwindow* w) { onEvent(w); });

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::GetIO().IniFilename = nullptr; // landing layout is fixed for now
    {
        // Sharp text on Retina (framebuffer > window size) and larger UI on HiDPI elsewhere.
        int ww = 0, wh = 0, fw = 0, fh = 0;
        float xs = 1.0f, ys = 1.0f;
        glfwGetWindowSize(m_window, &ww, &wh);
        glfwGetFramebufferSize(m_window, &fw, &fh);
        glfwGetWindowContentScale(m_window, &xs, &ys);
        const float fbScale = ww > 0 ? static_cast<float>(fw) / static_cast<float>(ww) : 1.0f;
        theme::apply(xs / std::max(fbScale, 1.0f), fbScale);
    }
    // Callbacks are chained manually above; let the backend install the rest.
    ImGui_ImplGlfw_InitForOpenGL(m_window, false);
    penInputAttach(m_window); // a tablet pen's pressure for the brushes
    glfwSetCharCallback(m_window, ImGui_ImplGlfw_CharCallback);
    glfwSetWindowFocusCallback(m_window, ImGui_ImplGlfw_WindowFocusCallback);
    glfwSetCursorEnterCallback(m_window, ImGui_ImplGlfw_CursorEnterCallback);
    ImGui_ImplOpenGL3_Init("#version 410 core");
    m_imguiReady = true;
    progress(0.22f, "Loading fonts and interface theme", "Interface");

    m_renderer = std::make_unique<Renderer>();
    if (!m_renderer->valid()) {
        std::fprintf(stderr, "Failed to build viewport shaders\n");
        return false;
    }

    m_uiState.gpuName = reinterpret_cast<const char*>(glGetString(GL_RENDERER));
    m_uiState.glVersion = std::string("OpenGL ") + reinterpret_cast<const char*>(glGetString(GL_VERSION));
    progress(0.4f, "Compiling viewport shaders", "Render pipeline");

    progress(0.55f, "Indexing part library", (std::to_string(libraryItems().size()) + " components").c_str());

    m_files.push_back(FileTab{});
    m_files.back().id = m_nextFileId++;
    m_files.back().demo = true;
    m_doc.scene = Scene::makeDemo();
    m_doc.name = "Demo Bracket";
    m_renderer->upload(m_doc.scene);

    int fbw = 0, fbh = 0;
    glfwGetFramebufferSize(m_window, &fbw, &fbh);
    m_aspect = fbh > 0 ? static_cast<float>(fbw) / static_cast<float>(fbh) : 1.0f;
    m_cam3D.setAspect(m_aspect);
    m_cam3D.setStandardView(StandardView::Isometric, false);
    m_cam3D.fitToBounds(m_doc.scene.bounds(), false);
    m_cam3D.zoom(-1.5f); // leave some breathing room on the landing page
    progress(0.7f, "Preparing workspace", "Digital twin");

    // 2D drawing window: flat drafting background, hidden-line bodies,
    // sketches drawn over everything on the active plane.
    m_vp2D.displayMode = DisplayMode::HiddenLine;
    m_vp2D.backgroundTop = m_vp2D.backgroundBottom = {0.105f, 0.115f, 0.135f};
    m_vp2D.edgeColor = {0.78f, 0.81f, 0.86f};
    m_vp2D.gridStrength = 0.8f;
    m_vp2D.sketchOnTop = true;

    if (m_options.start2D) {
        setMode(ViewMode::Drawing2D);
    }

    m_mcp = std::make_unique<McpServer>(m_doc, this);
    std::error_code ec;
    const std::filesystem::path exe = std::filesystem::absolute(m_options.executablePath, ec);
    for (const char* name : {"sicdm-mcp", "sicdm-mcp.exe"}) {
        const auto candidate = exe.parent_path() / name;
        if (std::filesystem::exists(candidate, ec)) m_uiState.mcpExecutable = candidate.string();
    }
    if (m_options.mcpPort > 0) {
        progress(0.8f, "Starting MCP server", "AI agent bridge");
        m_uiState.mcpPort = m_options.mcpPort;
        toggleMcp();
    }
    glfwSetWindowCloseCallback(m_window, [](GLFWwindow* w) {
        // Ask before discarding unsaved work; frame() shows the dialog.
        auto* app = static_cast<Application*>(glfwGetWindowUserPointer(w));
        if (app->anyFileDirty()) {
            glfwSetWindowShouldClose(w, GLFW_FALSE);
            app->m_closeRequested = true;
            onEvent(w);
        }
    });

    if (const auto u = parseUnitName(preferences().units)) m_doc.state.units.unit = *u; // Preferences: units for new projects
    if (!setLanguage(m_options.language.empty() ? preferences().language : m_options.language)) setLanguage(preferences().language);
    log("SI CDM ready. Type HELP for commands, or pick a tool on the left.", UiState::LogLine::Output);
    m_savedSignature = contentSignature(); // the demo part counts as unmodified
    if (!m_options.openPath.empty()) {
        progress(0.88f, "Opening " + std::filesystem::path(m_options.openPath).filename().string(), "Project");
        // A project opens in its own tab next to the demo; other files import into the demo.
        const FileKind kind = fileKindOf(m_options.openPath);
        runCommandLine((kind == FileKind::Project ? "OPEN \"" : "IMPORT \"") + m_options.openPath + "\"");
        if (kind != FileKind::Project) m_savedSignature = contentSignature();
    }
    for (const std::string& line : m_options.commands) runCommandLine(line);
    if (preferences().checkUpdates && !preferences().updateUrl.empty() && m_options.screenshotPath.empty())
        m_updateCheck = std::async(std::launch::async, [url = preferences().updateUrl] {
            UpdateResult r = checkForUpdate(url, SICDM_VERSION);
            glfwPostEmptyEvent(); // wake the idle loop to show it
            return r;
        });
    refreshFileTabs();
    updateTitle();
    // Unsaved work from a session that ended without closing: offered back in frame().
    m_recoverable = findRecoveryFiles();
    m_offerRecovery = !m_recoverable.empty() && m_options.screenshotPath.empty();
    // Crash reports this computer hasn't shown yet (written by the handler main() installs).
    for (const CrashReportFile& f : crashReports())
        if (!f.seen) m_newCrashes.push_back(f);
    m_offerCrashes = !m_newCrashes.empty() && m_options.screenshotPath.empty();
    progress(0.96f, "Restoring interface", "Workspace");
    return true;
}

void Application::shutdown() {
    m_autosaver.clear(); // a clean exit: the recovery copies aren't needed
    m_splash.reset();
    m_mcpHttp.stop();
    if (m_renderJob) m_renderJob->stop = true; // a picture being made: stopped, not waited for to the end
    if (m_videoJob) m_videoJob->stop = true; // a video being made: stopped after the frame it is on
    if (m_renderDone.valid()) m_renderDone.wait();
    if (m_videoDone.valid()) m_videoDone.wait();
    if (m_renderTexture) glDeleteTextures(1, &m_renderTexture), m_renderTexture = 0;
    m_renderer.reset();
    if (m_imguiReady) {
        ImGui_ImplOpenGL3_Shutdown();
        penInputDetach();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        m_imguiReady = false;
    }
    if (m_window) {
        glfwDestroyWindow(m_window);
        m_window = nullptr;
    }
    glfwTerminate();
}

void Application::onScroll(GLFWwindow* w, double dx, double dy) {
    ImGui_ImplGlfw_ScrollCallback(w, dx, dy);
    auto* app = static_cast<Application*>(glfwGetWindowUserPointer(w));
    if (!ImGui::GetIO().WantCaptureMouse) app->m_pendingScroll += static_cast<float>(dy);
    onEvent(w);
}

void Application::onEvent(GLFWwindow* w) {
    auto* app = static_cast<Application*>(glfwGetWindowUserPointer(w));
    app->m_framesToRender = std::max(app->m_framesToRender, 3);
}

namespace {

constexpr float kDragThreshold = 4.0f; // pixels before a press becomes a drag
constexpr float kPlaceSnap = 5.0f;     // mm grid snap when placing solids
constexpr float kCurvePickPx = 6.0f;   // screen distance for picking sketch curves
constexpr float kGripPickPx = 8.0f;    // ... and the points they are dragged by

// The points a sketch curve can be dragged by, numbered as constraint references number
// them: ends, corners, vertices, centres.
std::vector<std::pair<int, Vec2>> gripPoints(const SketchCurve& c) {
    std::vector<std::pair<int, Vec2>> pts;
    if (c.pts.empty()) return pts;
    switch (c.kind) {
    case CurveKind::Rectangle: {
        if (c.pts.size() < 2) break;
        const Vec2 a = c.pts[0], b = c.pts[1];
        pts = {{0, a}, {1, {b.x, a.y}}, {2, b}, {3, {a.x, b.y}}};
        break;
    }
    case CurveKind::Polyline:
    case CurveKind::Spline: {
        const std::size_t n = c.pts.size() - (c.isClosed() && c.kind == CurveKind::Polyline && c.pts.size() > 3 ? 1 : 0);
        for (std::size_t k = 0; k < n; ++k) pts.push_back({static_cast<int>(k), c.pts[k]});
        break;
    }
    case CurveKind::Arc:
        for (std::size_t k = 0; k < c.pts.size() && k < 3; ++k) pts.push_back({static_cast<int>(k), c.pts[k]});
        break;
    case CurveKind::Circle:
    case CurveKind::Point: pts = {{0, c.pts[0]}}; break;
    default:
        for (std::size_t k = 0; k < c.pts.size() && k < 2; ++k) pts.push_back({static_cast<int>(k), c.pts[k]});
    }
    return pts;
}

float snap(float v, float step) { return std::round(v / step) * step; }

StandardView planeView(SketchPlane p) {
    switch (p) {
    case SketchPlane::Top: return StandardView::Top;
    case SketchPlane::Front: return StandardView::Front;
    case SketchPlane::Right: return StandardView::Right;
    case SketchPlane::Custom: break; // see orient2D()
    }
    return StandardView::Top;
}

float segmentDistance(ImVec2 p, ImVec2 a, ImVec2 b) {
    const float dx = b.x - a.x, dy = b.y - a.y;
    const float len2 = dx * dx + dy * dy;
    float t = len2 > 0.0f ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2 : 0.0f;
    t = std::clamp(t, 0.0f, 1.0f);
    return std::hypot(p.x - (a.x + dx * t), p.y - (a.y + dy * t));
}

// Tool rail solid -> parametric kind (see buildPrimitive()).
const char* primitiveKind(ToolId tool) {
    switch (tool) {
    case ToolId::Block: return "box";
    case ToolId::Wedge: return "wedge";
    case ToolId::Pyramid: return "pyramid";
    case ToolId::Prism: return "prism";
    case ToolId::Cylinder: return "cylinder";
    case ToolId::Cone: return "cone";
    case ToolId::Sphere: return "sphere";
    case ToolId::Torus: return "torus";
    default: return "box";
    }
}

} // namespace

void Application::setStatus(std::string message) {
    m_uiState.statusMessage = std::move(message);
    m_statusUntil = glfwGetTime() + 4.0;
}

void Application::setMode(ViewMode mode) {
    if (mode == m_uiState.mode) return;
    m_sketcher.cancel();
    cancelInteraction(true);
    m_uiState.mode = mode;
    if (mode == ViewMode::Drawing2D && !m_2DReady) init2DCamera();
    if (mode != ViewMode::Simulation) m_uiState.simPlaying = false; // leaving the simulation pauses it
    if (mode == ViewMode::Simulation)
        setStatus(m_doc.scene.motions().empty() ? "Simulation window: add a Circular or Linear motion on the right"
                                                : "Simulation window: Space plays / pauses");
    else
        setStatus(mode == ViewMode::Drawing2D ? std::string("2D Drawing window: ") + activePlaneName(m_doc.state)
                                              : std::string("3D Model window"));
}

void Application::init2DCamera() {
    // First visit: plan view of the active plane, framed on the model.
    m_cam2D.setAspect(m_aspect);
    m_cam2D.setProjection(Projection::Orthographic);
    orient2D(false);
    if (m_doc.scene.bounds().valid()) m_cam2D.fitToBounds(m_doc.scene.bounds(), false);
    m_2DReady = true;
}

void Application::orient2D(bool animate) {
    if (m_uiState.plane != SketchPlane::Custom) {
        m_cam2D.setStandardView(planeView(m_uiState.plane), animate);
        return;
    }
    const PlaneFrame& f = m_doc.state.customFrame; // plan-on, u to the right
    m_cam2D.setViewFrame(f.normal, f.u, animate);
}

void Application::setPlane(SketchPlane plane) {
    m_sketcher.cancel();
    const bool changed = plane != m_uiState.plane || (plane == SketchPlane::Custom && !sameFrame(m_vp2D.gridFrame, m_doc.state.customFrame));
    m_uiState.plane = plane;
    m_doc.state.plane = plane;
    m_vp2D.gridPlane = plane;
    m_vp2D.gridFrame = m_doc.state.customFrame;
    if (m_2DReady && changed) {
        orient2D(true);
        m_cam2D.fitToBounds(m_doc.scene.bounds(), true);
    }
    if (changed) setStatus("Sketch plane: " + activePlaneName(m_doc.state));
}

std::optional<RayHit> Application::pick(float x, float y) const {
    const ImGuiIO& io = ImGui::GetIO();
    const Ray ray = camera().screenRay(x, y, io.DisplaySize.x, io.DisplaySize.y);
    // Sectioned: what is taken away can't be picked, and a cut face can.
    auto hit = m_uiState.mode == ViewMode::Drawing2D ? m_doc.scene.raycast(ray, bodyTransforms())
                                                     : sectionRaycast(m_doc.scene, m_doc.state.section, ray, bodyTransforms());
    // Exploded: the point where the part really is (tools work on the assembly as it stands).
    if (hit && !simulating() && hit->body >= 0 && static_cast<std::size_t>(hit->body) < m_explodeShifts.size())
        hit->point = hit->point - m_explodeShifts[static_cast<std::size_t>(hit->body)].f();
    return hit;
}

int Application::pickCurve(float x, float y) const {
    const ImGuiIO& io = ImGui::GetIO();
    const auto& curves = m_doc.scene.sketch().curves();
    int best = -1;
    float bestD = kCurvePickPx;
    for (std::size_t i = 0; i < curves.size(); ++i) {
        if (!curves[i].visible) continue;
        const std::vector<Vec3> strip = curves[i].worldStrip(64);
        std::optional<ImVec2> prev;
        for (const Vec3& p : strip) {
            const auto s = projectToScreen(camera(), p, io.DisplaySize);
            if (s) {
                const float d = prev ? segmentDistance({x, y}, *prev, *s) : std::hypot(x - s->x, y - s->y);
                if (d < bestD) {
                    bestD = d;
                    best = static_cast<int>(i);
                }
            }
            prev = s;
        }
    }
    return best;
}

std::optional<Application::SketchGripHit> Application::pickSketchGrip(float x, float y) const {
    const ImGuiIO& io = ImGui::GetIO();
    const auto& curves = m_doc.scene.sketch().curves();
    std::optional<SketchGripHit> best;
    float bestD = kGripPickPx;
    for (std::size_t i = 0; i < curves.size(); ++i) {
        const SketchCurve& c = curves[i];
        if (!c.visible || c.pts.empty() || c.spatial()) continue; // a 3D curve's points move with PATH3D POINT
        const std::vector<std::pair<int, Vec2>> pts = gripPoints(c);
        const PlaneFrame f = curveFrame(c);
        for (const auto& [k, q] : pts) {
            const auto sp = projectToScreen(camera(), f.toWorld(q), io.DisplaySize);
            if (!sp) continue;
            const float d = std::hypot(x - sp->x, y - sp->y);
            if (d < bestD) {
                bestD = d;
                SketchGripHit h;
                h.grip.ref.curve = c.name;
                h.grip.ref.point = k;
                h.from = q;
                h.curve = static_cast<int>(i);
                best = h;
            }
        }
    }
    if (best) return best;
    // Else the curve itself: a circle or arc by its rim, anything else whole.
    const int ci = pickCurve(x, y);
    if (ci < 0) return std::nullopt;
    const SketchCurve& c = curves[static_cast<std::size_t>(ci)];
    const PlaneFrame f = curveFrame(c);
    const Ray ray = camera().screenRay(x, y, io.DisplaySize.x, io.DisplaySize.y);
    const float dn = dot(ray.dir, f.normal);
    if (std::fabs(dn) < 1e-6f) return std::nullopt;
    SketchGripHit h;
    h.grip.ref.curve = c.name;
    h.grip.rim = c.kind == CurveKind::Circle || c.kind == CurveKind::Arc;
    if (c.kind == CurveKind::Point) h.grip.ref.point = 0;
    h.from = f.toPlane(ray.at(dot(f.origin - ray.origin, f.normal) / dn));
    h.curve = ci;
    return h;
}

void Application::updateSketchDrag(float x, float y) {
    const auto& curves = m_sketchDragStart.curves();
    if (m_sketchDrag.curve < 0 || m_sketchDrag.curve >= static_cast<int>(curves.size())) return;
    const SketchCurve& c = curves[static_cast<std::size_t>(m_sketchDrag.curve)];
    const ImGuiIO& io = ImGui::GetIO();
    const PlaneFrame f = curveFrame(c);
    const Ray ray = camera().screenRay(x, y, io.DisplaySize.x, io.DisplaySize.y);
    const float dn = dot(ray.dir, f.normal);
    if (std::fabs(dn) < 1e-6f) return;
    Vec2 to = f.toPlane(ray.at(dot(f.origin - ray.origin, f.normal) / dn));
    // The command records the point to 6 significant figures: drag to that same point.
    char b[64];
    std::snprintf(b, sizeof b, "%.6g %.6g", static_cast<double>(to.x), static_cast<double>(to.y));
    std::sscanf(b, "%f %f", &to.x, &to.y);
    if (m_sketchDragMoved && to.x == m_sketchDragTo.x && to.y == m_sketchDragTo.y) return;
    Sketch s = m_sketchDragStart;
    const SolveReport r = dragSketch(s, m_sketchDrag.grip, m_sketchDrag.from, to);
    if (!r.ok) {
        setStatus("Can't drag there: " + (r.conflict.empty() ? std::string("the constraints can't all be met") : r.conflict + " can't be met"));
        return;
    }
    m_doc.scene.sketch() = std::move(s);
    m_doc.scene.sketch().touch();
    m_sketchDragTo = to;
    m_sketchDragMoved = true;
    const std::string what = m_sketchDrag.grip.rim ? c.name + " rim" : m_sketchDrag.grip.ref.point >= 0 ? m_sketchDrag.grip.ref.describe() : c.name;
    std::string line = "Dragging " + what + " to " + lengthText(to.x) + ", " + lengthText(to.y);
    if (r.residual > 1e-3) line += "  (held " + lengthText(r.residual) + " short by its constraints)";
    setStatus(line);
    m_framesToRender = std::max(m_framesToRender, 2);
}

void Application::finishSketchDrag() {
    if (!m_sketchDragMoved) return;
    m_sketchDragMoved = false;
    // Put the sketch back and drag it by the command, so it is in the history and undoes.
    // (The copy keeps its revision, so the edit before the drag can still be undone.)
    m_doc.scene.sketch() = m_sketchDragStart;
    const SketchGrip& g = m_sketchDrag.grip;
    char to[96], from[96];
    std::snprintf(to, sizeof to, "%.6gmm,%.6gmm", static_cast<double>(m_sketchDragTo.x), static_cast<double>(m_sketchDragTo.y));
    std::snprintf(from, sizeof from, "%.6gmm,%.6gmm", static_cast<double>(m_sketchDrag.from.x), static_cast<double>(m_sketchDrag.from.y));
    std::string line = "DRAG \"" + g.ref.curve + "\"";
    if (g.rim) line += " RIM";
    else if (g.ref.point >= 0 && m_sketchDragStart.curves()[static_cast<std::size_t>(m_sketchDrag.curve)].kind != CurveKind::Point)
        line += " POINT " + std::to_string(g.ref.point + 1);
    line += std::string(" TO ") + to;
    if (!g.rim && g.ref.point < 0) line += std::string(" FROM ") + from;
    runCommandLine(line);
}

void Application::addCurve(SketchCurve curve) {
    curve.construction = m_uiState.sketchConstruction;
    const SketchCurve& c = m_doc.scene.sketch().add(std::move(curve));
    m_uiState.selectedCurve = static_cast<int>(m_doc.scene.sketch().curves().size()) - 1;
    m_uiState.selectedBody = m_uiState.selectedImage = -1;
    setStatus("Added " + c.name + "  " + c.measure());
    log("Added " + c.name + " on " + activePlaneName(m_doc.state) + "  " + c.measure(), UiState::LogLine::Output);
}

void Application::onCanvasClick(float x, float y, bool doubleClick) {
    ToolId tool = m_ui.toolbox().active();
    if (simulating()) { // the simulation shows motion; modelling tools work in the 2D / 3D windows
        if (tool != ToolId::Select && tool != ToolId::Move && tool != ToolId::Orbit && tool != ToolId::Pan &&
            tool != ToolId::Zoom)
            setStatus(std::string(toolInfo(tool).name) + " works in the 2D / 3D windows; clicks select here");
        tool = ToolId::Select;
        const auto hit = pick(x, y);
        m_uiState.selectedBody = hit ? hit->body : -1;
        m_uiState.selectedCurve = m_uiState.selectedImage = -1;
        if (hit) setStatus("Selected " + m_doc.scene.bodies()[static_cast<std::size_t>(hit->body)].name);
        (void)doubleClick;
        return;
    }
    const ToolInfo& info = toolInfo(tool);
    const bool is2D = m_uiState.mode == ViewMode::Drawing2D;

    // The mate tool: a face on the part that stays, then one on the part that moves.
    if (!m_uiState.mateTool.empty() && !is2D) {
        const auto hit = pick(x, y);
        if (!hit) {
            setStatus("Click a face of a part");
            return;
        }
        const Body& b = m_doc.scene.bodies()[static_cast<std::size_t>(hit->body)];
        // Millimetres stated, whatever units the document shows.
        char at[160];
        std::snprintf(at, sizeof at, "%.6gmm,%.6gmm,%.6gmm", static_cast<double>(hit->point.x), static_cast<double>(hit->point.y), static_cast<double>(hit->point.z));
        if (m_uiState.mateFirst.empty()) {
            m_uiState.mateFirst = b.name;
            m_uiState.mateFirstPoint = hit->point;
            setStatus(b.name + " picked: now a face on the part to move");
            return;
        }
        if (b.name == m_uiState.mateFirst) {
            setStatus("Pick a face on another part");
            return;
        }
        char first[160];
        std::snprintf(first, sizeof first, "%.6gmm,%.6gmm,%.6gmm", static_cast<double>(m_uiState.mateFirstPoint.x), static_cast<double>(m_uiState.mateFirstPoint.y),
                      static_cast<double>(m_uiState.mateFirstPoint.z));
        std::string line = "MATE " + m_uiState.mateTool;
        if (m_uiState.mateTool == "DISTANCE" || m_uiState.mateTool == "ANGLE") {
            char v[48];
            std::snprintf(v, sizeof v, m_uiState.mateTool == "DISTANCE" ? " %gmm" : " %g", static_cast<double>(m_uiState.mateValue));
            line += v;
        }
        line += " \"" + m_uiState.mateFirst + "\" AT " + first + " \"" + b.name + "\" AT " + at;
        m_uiState.mateFirst.clear(); // ready for the next pair
        runCommandLine(line);
        return;
    }

    // The Armature window adding balls: a click on a ball adds one on its surface there,
    // joined to it (in its rest frame, so a posed armature takes it where it was clicked).
    if (m_uiState.showArmature && m_uiState.armaturePick && !is2D) {
        const auto hit = pick(x, y);
        const auto& bodies = m_doc.scene.bodies();
        if (!hit || hit->body < 0 || hit->body >= static_cast<int>(bodies.size()) || bodies[static_cast<std::size_t>(hit->body)].kind != "armature") {
            setStatus("Click a ball of the armature to add one there");
            return;
        }
        const Body& ab = bodies[static_cast<std::size_t>(hit->body)];
        Armature arm;
        if (!armatureFromParams(ab.params, arm)) return;
        const Vec3 p = hit->point - ab.offset;
        const std::vector<Vec3> posed = posedBalls(arm);
        std::size_t nearest = 0;
        float best = 1e30f;
        for (std::size_t i = 0; i < posed.size(); ++i) {
            const float d = std::fabs(length(p - posed[i]) - arm.balls[i].radius);
            if (d < best) best = d, nearest = i;
        }
        const Vec3 out = length(p - posed[nearest]) > 1e-6f ? normalize(p - posed[nearest]) : Vec3{0, 0, 1};
        const Vec3 posedAt = posed[nearest] + out * arm.balls[nearest].radius;
        const std::vector<Mat4> pose = armaturePose(arm);
        const Vec3 restAt = inverse(pose[nearest]).transformPoint(posedAt);
        char line[256];
        std::snprintf(line, sizeof line, "ARMATURE \"%s\" ADD TO %zu AT %.4g,%.4g,%.4g RADIUS %.4g", ab.name.c_str(), nearest + 1, static_cast<double>(restAt.x),
                      static_cast<double>(restAt.y), static_cast<double>(restAt.z), static_cast<double>(arm.balls[nearest].radius * 0.8f));
        std::string cmdLine = line;
        static const char* const kMirror[] = {"", " MIRROR X", " MIRROR Y", " MIRROR Z"};
        cmdLine += kMirror[std::clamp(m_uiState.armatureMirror, 0, 3)];
        m_uiState.armatureName = ab.name;
        m_uiState.armatureBall = static_cast<int>(arm.balls.size());
        runCommandLine(cmdLine);
        return;
    }

    // The 3D Path window picking points: a 3D path's or reference point under the cursor, a
    // point on a body, else level with the last point (or the ground).
    if (m_uiState.showPath3d && m_uiState.pathPick && !is2D) {
        const ImGuiIO& io = ImGui::GetIO();
        std::optional<Vec3> at;
        float best = 10.0f;
        auto consider = [&](const Vec3& p) {
            if (const auto sp = projectToScreen(camera(), p, io.DisplaySize)) {
                const float d = std::hypot(sp->x - x, sp->y - y);
                if (d < best) best = d, at = p;
            }
        };
        for (const SketchCurve& c : m_doc.scene.sketch().curves())
            if (c.visible && c.spatial())
                for (const Vec3& p : spacePoints(c)) consider(p);
        for (const DatumPoint& r : m_doc.state.refPoints) consider(r.at);
        if (!at)
            if (const auto hit = pick(x, y)) at = hit->point;
        if (!at) {
            const Ray ray = camera().screenRay(x, y, io.DisplaySize.x, io.DisplaySize.y);
            const float level = m_uiState.pathPoints.empty() ? 0.0f : m_uiState.pathPoints.back().z;
            if (std::fabs(ray.dir.z) > 1e-6f) {
                const float t = (level - ray.origin.z) / ray.dir.z;
                if (t > 0) at = ray.at(t);
            }
        }
        if (!at) {
            setStatus("Click a body, a 3D path's point, or the space level with the last point");
            return;
        }
        m_uiState.pathPoints.push_back(*at);
        char b[160];
        std::snprintf(b, sizeof b, "Point %zu of the new path at (%.2f, %.2f, %.2f) mm", m_uiState.pathPoints.size(), static_cast<double>(at->x), static_cast<double>(at->y),
                      static_cast<double>(at->z));
        setStatus(b);
        return;
    }

    if (isSketchTool(tool)) {
        m_sketcher.setPolygonSides(m_uiState.polygonSides);
        if (auto c = m_sketcher.click(tool, doubleClick)) addCurve(std::move(*c));
        return;
    }
    if (isSketchEditTool(tool)) { // trim, extend, offset: the curve under the cursor
        const int ci = pickCurve(x, y);
        if (ci < 0) {
            setStatus("Click a sketch curve");
            return;
        }
        const SketchCurve& c = m_doc.scene.sketch().curves()[static_cast<std::size_t>(ci)];
        const ImGuiIO& io = ImGui::GetIO();
        const Ray ray = camera().screenRay(x, y, io.DisplaySize.x, io.DisplaySize.y);
        const PlaneFrame f = curveFrame(c);
        const float dn = dot(ray.dir, f.normal);
        if (std::fabs(dn) < 1e-6f) {
            setStatus("Look at the sketch plane to edit its curves");
            return;
        }
        const Vec2 q = f.toPlane(ray.at(dot(f.origin - ray.origin, f.normal) / dn));
        char at[96];
        std::snprintf(at, sizeof at, "%.6gmm,%.6gmm", static_cast<double>(q.x), static_cast<double>(q.y));
        std::string name = "\"" + c.name + "\"";
        if (tool == ToolId::SketchTrim) runCommandLine("TRIM " + name + " AT " + at);
        else if (tool == ToolId::SketchExtend) runCommandLine("EXTEND " + name + " AT " + at);
        else {
            char d[48];
            std::snprintf(d, sizeof d, "%.6gmm", static_cast<double>(m_uiState.sketchOffset));
            runCommandLine("OFFSET " + name + " " + d + " SIDE " + at);
        }
        return;
    }

    if (isCutTool(tool)) {
        if (tool != ToolId::CutChannel && tool != ToolId::CurveBrush && tool != ToolId::CutKnife && tool != ToolId::PushPull) applyCutTool(tool, x, y);
        return;
    }
    if (isWearableTool(tool)) {
        applyWearableTool(tool, x, y);
        return;
    }
    if (isSculptTool(tool)) {
        if (!pick(x, y)) setStatus("Drag over a body to sculpt it");
        return;
    }

    if (tool == ToolId::Extrude) {
        if (m_extrudeCurve >= 0) {
            applyExtrude(); // second click confirms the depth
            return;
        }
        const int curve = pickProfile(x, y);
        if (curve < 0) setStatus("Click a closed sketch curve: rectangle, circle or closed polyline");
        else beginExtrude(curve);
        return;
    }

    if ((tool == ToolId::Select || tool == ToolId::Move) && commandModifierHeld()) {
        // Ctrl+click (Cmd on a Mac): the curve or body goes into the selection, or comes out of it.
        if (const int curve = tool == ToolId::Select ? pickCurve(x, y) : -1; curve >= 0) {
            m_uiState.selectedBody = m_uiState.selectedImage = -1;
            m_uiState.bodySelection = {};
            toggleCurve(m_doc.scene.sketch(), m_uiState.selectedCurve, m_uiState.curveSelection, curve);
            selectionChanged();
        } else if (const auto hit = pick(x, y)) {
            m_uiState.curveSelection = {};
            m_uiState.selectedCurve = m_uiState.selectedImage = -1;
            toggleBody(m_doc.scene, m_uiState.selectedBody, m_uiState.bodySelection, hit->body);
            selectionChanged();
        }
        return;
    }
    if (tool == ToolId::Select || tool == ToolId::Move) {
        // Sketch curves are thin, so they win over the bodies behind them.
        const int curve = tool == ToolId::Select ? pickCurve(x, y) : -1;
        if (curve >= 0) {
            m_uiState.selectedCurve = curve;
            m_uiState.curveSelection = {};
            m_uiState.selectedBody = m_uiState.selectedImage = -1;
            setStatus("Selected " + m_doc.scene.sketch().curves()[static_cast<std::size_t>(curve)].name);
            return;
        }
        const auto hit = pick(x, y);
        m_uiState.selectedBody = hit ? hit->body : -1;
        m_uiState.selectedCurve = m_uiState.selectedImage = -1;
        if (hit) {
            setStatus("Selected " + m_doc.scene.bodies()[static_cast<std::size_t>(hit->body)].name);
        } else if (const int img = pickImage(x, y); img >= 0) {
            m_uiState.selectedImage = img;
            setStatus("Selected " + m_doc.scene.images()[static_cast<std::size_t>(img)].name);
        }
        return;
    }

    if (isPrimitiveTool(tool)) {
        const ImGuiIO& io = ImGui::GetIO();
        const Ray ray = camera().screenRay(x, y, io.DisplaySize.x, io.DisplaySize.y);
        Vec3 at;
        const auto hit = pick(x, y);
        if (hit && !is2D) {
            at = hit->point; // stack on the surface that was clicked
        } else {
            // 3D: the ground plane; 2D: the drawing plane (two-sided).
            const PlaneFrame f = is2D ? activePlaneFrame(m_doc.state) : planeFrame(SketchPlane::Top);
            const float d = dot(ray.dir, f.normal);
            if (std::fabs(d) < 1e-4f) {
                setStatus("Click on the grid or a body to place it");
                return;
            }
            const float t = dot(f.origin - ray.origin, f.normal) / d;
            if (t < 0.0f && !is2D) {
                setStatus("Click on the grid or a body to place it");
                return;
            }
            const Vec2 q = f.toPlane(ray.at(t));
            at = f.toWorld({snap(q.x, kPlaceSnap), snap(q.y, kPlaceSnap)});
        }
        Body body = defaultPrimitive(primitiveKind(tool), ++m_doc.primitiveSerial);
        body.offset = at;
        char msg[160];
        std::snprintf(msg, sizeof(msg), "Placed %s at %s", body.name.c_str(),
                      lengthsText({at.x, at.y, at.z}, ", ", "(", ")").c_str());
        m_doc.scene.beginBodyEdit("Place " + body.name); // UNDO takes it away
        m_doc.scene.add(std::move(body));
        m_doc.scene.endBodyEdit();
        m_doc.scene.sealUndo();
        m_uiState.selectedBody = static_cast<int>(m_doc.scene.bodies().size()) - 1;
        m_uiState.selectedCurve = m_uiState.selectedImage = -1;
        setStatus(msg);
        log(msg, UiState::LogLine::Output);
        return;
    }

    if (tool == ToolId::Orbit && is2D) setStatus("Orbit works in the 3D window; switch with Tab");
    else if (!info.available) setStatus(std::string(info.name) + " is planned for a later milestone");
}

// ---------------------------------------------------------------------------
// Sculpting and cut tools

namespace {

SculptBrush brushOf(ToolId t) {
    switch (t) {
    case ToolId::SculptClay: return SculptBrush::Clay;
    case ToolId::SculptCarve: return SculptBrush::Carve;
    case ToolId::SculptInflate: return SculptBrush::Inflate;
    case ToolId::SculptGrab: return SculptBrush::Grab;
    case ToolId::SculptPinch: return SculptBrush::Pinch;
    case ToolId::SculptCrease: return SculptBrush::Crease;
    case ToolId::SculptSmooth: return SculptBrush::Smooth;
    case ToolId::SculptFlatten: return SculptBrush::Flatten;
    case ToolId::SculptScrape: return SculptBrush::Scrape;
    case ToolId::SculptLayer: return SculptBrush::Layer;
    case ToolId::SculptClayStrips: return SculptBrush::ClayStrips;
    case ToolId::SculptFill: return SculptBrush::Fill;
    case ToolId::SculptBlob: return SculptBrush::Blob;
    case ToolId::SculptSnakeHook: return SculptBrush::SnakeHook;
    case ToolId::SculptNudge: return SculptBrush::Nudge;
    case ToolId::SculptMask: return SculptBrush::Mask;
    case ToolId::SculptPaint: return SculptBrush::Paint;
    default: return SculptBrush::Draw;
    }
}

// Exact millimetres for command lines, whatever the display units.
std::string mmText(float v) {
    char b[48];
    std::snprintf(b, sizeof(b), "%.9gmm", static_cast<double>(v));
    return b;
}
std::string mmVec(const Vec3& v) { return mmText(v.x) + "," + mmText(v.y) + "," + mmText(v.z); }
std::string dirVec(const Vec3& v) {
    char b[96];
    std::snprintf(b, sizeof(b), "%.6g,%.6g,%.6g", static_cast<double>(v.x), static_cast<double>(v.y), static_cast<double>(v.z));
    return b;
}
// A quoted name: always read as a name, quotes inside written \".
std::string quotedName(const std::string& s) {
    std::string q = "\"";
    for (const char c : s) {
        if (c == '"') q += '\\';
        q += c;
    }
    return q + "\"";
}

} // namespace

bool Application::beginSculpt(float x, float y) {
    const auto hit = pick(x, y);
    if (!hit) return false;
    const auto bi = static_cast<std::size_t>(hit->body);
    SculptMesh mesh(m_doc.scene.bodies()[bi]);
    if (mesh.empty()) return false;
    m_doc.scene.pushUndo("SCULPT on " + m_doc.scene.bodies()[bi].name);
    Body& body = m_doc.scene.bodies()[bi];
    m_sculptMesh = std::move(mesh);
    SculptSettings s;
    s.brush = ImGui::GetIO().KeyShift ? SculptBrush::Smooth : brushOf(m_ui.toolbox().active());
    s.invert = commandModifierHeld();
    s.radius = std::max(m_uiState.sculptRadius, 0.01f);
    s.strength = std::clamp(m_uiState.sculptStrength, 0.0f, 1.0f);
    s.symmetryX = m_uiState.sculptSymmetry;
    s.symmetryY = m_uiState.sculptSymmetryY;
    s.symmetryZ = m_uiState.sculptSymmetryZ;
    s.radial = m_uiState.sculptRadial > 1 ? m_uiState.sculptRadial : 0;
    s.falloff = static_cast<SculptFalloff>(std::clamp(m_uiState.sculptFalloff, 0, 4));
    s.refine = m_uiState.sculptRefine && !body.multires; // multiresolution levels keep their vertices
    s.detail = std::max(m_uiState.sculptDetail, 0.0f);
    s.color = {m_uiState.sculptColor[0], m_uiState.sculptColor[1], m_uiState.sculptColor[2]};
    if (m_uiState.sculptAlpha > 0 && static_cast<std::size_t>(m_uiState.sculptAlpha) <= sculptAlphaNames().size()) {
        static std::vector<std::shared_ptr<const SculptAlpha>> made(sculptAlphaNames().size()); // built once each
        auto& a = made[static_cast<std::size_t>(m_uiState.sculptAlpha - 1)];
        if (!a) a = sculptAlpha(sculptAlphaNames()[static_cast<std::size_t>(m_uiState.sculptAlpha - 1)]);
        s.alpha = a;
        s.alphaAngle = m_uiState.sculptAlphaAngle;
        s.alphaFollow = m_uiState.sculptAlphaFollow;
    }
    m_sculptSettings = s;
    m_sculptBody = hit->body;
    m_sculptName = body.name;
    m_dabs = 0;
    const ImGuiIO& io = ImGui::GetIO();
    const Ray ray = camera().screenRay(x, y, io.DisplaySize.x, io.DisplaySize.y);
    const Vec3 p = m_sculptMesh.raycast(ray).value_or(hit->point);
    m_stamping = s.alpha && m_uiState.sculptStamp && s.brush != SculptBrush::Grab;
    if (s.brush == SculptBrush::Grab) {
        m_sculptMesh.beginGrab(p, s);
        m_grabPoint = p;
    } else if (m_stamping) { // nothing yet: the drag sizes and turns the stamp
        m_stampBase = m_sculptMesh;
    } else {
        m_sculptMesh.beginStroke();
        m_sculptMesh.dab(p, withPressure(s, penPressure(), m_uiState.sculptPressureSize, m_uiState.sculptPressureStrength));
        ++m_dabs;
    }
    m_lastDab = p;
    m_sculptMesh.writeTo(body, false);
    m_sculptOverrides.assign(m_doc.scene.bodies().size(), nullptr);
    m_sculptOverrides[bi] = &body.mesh;
    m_sculptRevision = m_doc.scene.revision();
    m_uiState.selectedBody = hit->body;
    m_uiState.selectedCurve = m_uiState.selectedImage = -1;
    return true;
}

bool Application::sculptTargetValid() const {
    const auto& bodies = m_doc.scene.bodies();
    return m_sculptBody >= 0 && m_sculptBody < static_cast<int>(bodies.size()) &&
           m_doc.scene.revision() == m_sculptRevision && bodies[static_cast<std::size_t>(m_sculptBody)].name == m_sculptName &&
           m_sculptOverrides.size() == bodies.size();
}

void Application::continueSculpt(float x, float y) {
    if (!sculptTargetValid()) {
        abortSculpt();
        return;
    }
    Body& body = m_doc.scene.bodies()[static_cast<std::size_t>(m_sculptBody)];
    const ImGuiIO& io = ImGui::GetIO();
    const Ray ray = camera().screenRay(x, y, io.DisplaySize.x, io.DisplaySize.y);
    if (m_sculptSettings.brush == SculptBrush::Grab) {
        float t = 0;
        if (!intersectPlane(ray, m_grabPoint, camera().forward(), t)) return;
        m_sculptMesh.grabTo(ray.at(t) - m_grabPoint);
    } else if (m_stamping) { // the stamp from where the press was, as large as the drag, turned along it
        float t = 0;
        if (!intersectPlane(ray, m_lastDab, camera().forward(), t)) return;
        const Vec3 to = ray.at(t) - m_lastDab;
        if (length(to) < 1e-3f) return;
        SculptSettings s = m_sculptSettings;
        s.radius = length(to);
        s.alphaFollow = true;
        m_sculptMesh = m_stampBase;
        m_sculptMesh.beginStroke();
        m_sculptMesh.dab(m_lastDab, s, to);
        m_dabs = 1;
    } else {
        const auto hit = m_sculptMesh.raycast(ray);
        if (!hit) return;
        // Dabs every quarter radius (or the spacing set) along the stroke, each on the surface;
        // a pen's pressure sizes and weights them.
        const SculptSettings dabSettings = withPressure(m_sculptSettings, penPressure(), m_uiState.sculptPressureSize, m_uiState.sculptPressureStrength);
        const float spacing = std::max(dabSettings.radius * (dabSettings.spacing > 0 ? dabSettings.spacing : 0.25f), 1e-3f);
        const Vec3 from = m_lastDab;
        const float len = length(*hit - from);
        if (len < spacing) return;
        const int steps = std::min(static_cast<int>(len / spacing), 64);
        for (int i = 1; i <= steps; ++i) {
            const Vec3 q = m_sculptMesh.closestPoint(from + (*hit - from) * (static_cast<float>(i) * spacing / len));
            m_sculptMesh.dab(q, dabSettings, q - m_lastDab);
            ++m_dabs;
            m_lastDab = q;
        }
    }
    m_sculptMesh.writeTo(body, false);
    m_sculptOverrides[static_cast<std::size_t>(m_sculptBody)] = &body.mesh;
}

void Application::endSculpt() {
    if (!sculptTargetValid()) {
        abortSculpt();
        return;
    }
    Body& body = m_doc.scene.bodies()[static_cast<std::size_t>(m_sculptBody)];
    m_sculptMesh.commitLayers(); // the stroke goes into the recording layer, if any
    m_sculptMesh.writeTo(body, true);
    m_doc.scene.touchBodies();
    m_doc.scene.sealUndo();
    const std::size_t open = openEdgeCount(body.mesh);
    char info[160];
    std::snprintf(info, sizeof(info), "%s: %zu triangles%s", body.name.c_str(), body.mesh.indices.size() / 3,
                  open == 0 ? ", closed" : "");
    m_uiState.sculptInfo = info;
    setStatus(info);
    log(std::string("Sculpted ") + body.name + " (" + sculptBrushKey(m_sculptSettings.brush) +
            (m_sculptSettings.invert ? ", inverted" : "") + ", " + std::to_string(std::max(m_dabs, 1)) + " dabs)",
        UiState::LogLine::Output);
    m_sculptBody = -1;
    m_sculptOverrides.clear();
    m_sculptMesh = SculptMesh();
    m_stampBase = SculptMesh();
    m_stamping = false;
}

void Application::abortSculpt() {
    if (m_sculptBody < 0) return;
    // Untouched since the stroke began: its snapshot is the newest one, restore it.
    // Otherwise something else changed the bodies; leave them as they are now.
    if (m_doc.scene.revision() == m_sculptRevision && m_doc.scene.undoDepth() > 0) {
        m_doc.scene.undo();
        setStatus("Stroke cancelled");
    } else {
        setStatus("Stroke dropped: the body changed while sculpting");
    }
    m_sculptBody = -1;
    m_sculptOverrides.clear();
    m_sculptMesh = SculptMesh();
    m_stampBase = SculptMesh();
    m_stamping = false;
}

void Application::finishMoveDrag(bool commit) {
    // Put them back, then (let go) move them again by MOVE: history and undo have it.
    auto& bodies = m_doc.scene.bodies();
    Vec3 by{0, 0, 0};
    std::vector<std::string> names;
    for (std::size_t k = 0; k < m_moveBodies.size(); ++k) {
        const int b = m_moveBodies[k];
        if (b < 0 || b >= static_cast<int>(bodies.size())) continue;
        Body& body = bodies[static_cast<std::size_t>(b)];
        if (names.empty()) by = body.offset - m_moveStarts[k];
        body.offset = m_moveStarts[k];
        names.push_back(body.name);
    }
    m_snapHit.reset();
    const int grabbed = m_moveBodies.empty() ? -1 : m_moveBodies.front();
    m_moveBodies.clear();
    m_moveStarts.clear();
    m_doc.scene.touchBodies();
    if (commit && names.size() > 1 && length(by) < 1e-6f && grabbed >= 0) { // a click, not a drag: just that one
        m_uiState.selectedBody = grabbed;
        m_uiState.bodySelection = {};
        selectionChanged();
        return;
    }
    if (!commit || names.empty() || length(by) < 1e-6f) return;
    GizmoDelta d;
    d.move = by;
    runCommandLine(gizmoCommand(GizmoMode::Move, d, names, Vec3{0, 0, 0}));
}

void Application::finishGizmoDrag(bool commit) {
    if (m_gizmoBody < 0 || m_gizmoBody >= static_cast<int>(m_doc.scene.bodies().size())) return;
    // Put them back as they were, then (let go) do it again by its command: history and undo have it.
    std::vector<std::string> names;
    for (std::size_t k = 0; k < m_gizmoBodies.size(); ++k) {
        m_doc.scene.bodies()[static_cast<std::size_t>(m_gizmoBodies[k])] = m_gizmoStarts[k];
        names.push_back(m_gizmoStarts[k].name);
    }
    m_doc.scene.touchBodies();
    m_gizmoBody = -1;
    m_snapHit.reset();
    m_gizmoBodies.clear();
    m_gizmoStarts.clear();
    if (!commit) {
        setStatus("Cancelled");
        return;
    }
    const std::string line = gizmoCommand(static_cast<GizmoMode>(std::clamp(m_uiState.gizmoMode, 0, 2)), m_gizmoDelta, names, m_gizmoPivot);
    if (!line.empty()) runCommandLine(line);
}

void Application::cancelInteraction(bool commit) {
    if (m_leftDrag == DragMode::Gizmo) finishGizmoDrag(commit);
    if (m_leftDrag == DragMode::MoveBody) finishMoveDrag(commit);
    if (m_leftDrag == DragMode::SketchGrip) { // Esc: the sketch as it was
        if (commit) finishSketchDrag();
        else if (m_sketchDragMoved) {
            m_doc.scene.sketch() = m_sketchDragStart; // its own revision: as if never dragged
            m_sketchDragMoved = false;
            setStatus("Drag cancelled");
        }
    }
    m_gripCandidate.reset();
    if (m_sculptBody >= 0) {
        if (commit) endSculpt();
        else abortSculpt();
    }
    m_cutPath.clear();
    m_cutBody = -1;
    m_uiState.booleanFirst.clear();
    m_leftDrag = m_otherDrag = DragMode::None;
}

// The shape and join words of an INSERTMESH / CURVE INSERT line from the options bar.
static std::string insertShapeWords(const UiState& u) {
    static const char* const kShapes[] = {"SPHERE", "CUBE", "CYLINDER", "CONE", "TORUS"};
    return u.insertShape >= 0 && u.insertShape < 5 ? std::string(kShapes[u.insertShape]) : "BODY " + quotedName(u.insertBody);
}

static std::string insertTailWords(const UiState& u) {
    char sink[32];
    std::snprintf(sink, sizeof(sink), " SINK %.3g", static_cast<double>(std::clamp(u.insertSink, 0.0f, 1.0f)));
    return sink + std::string(u.insertJoin == 1 ? " APPEND" : u.insertJoin == 2 ? " NEW" : "");
}

void Application::applyCutTool(ToolId tool, float x, float y) {
    const auto hit = pick(x, y);
    const UiState& u = m_uiState;
    if (tool == ToolId::BooleanBodies) {
        if (!hit) {
            m_uiState.booleanFirst.clear();
            setStatus("Click the body to keep, then the tool body");
            return;
        }
        const std::string name = m_doc.scene.bodies()[static_cast<std::size_t>(hit->body)].name;
        bool firstExists = false;
        for (const Body& b : m_doc.scene.bodies()) firstExists = firstExists || b.name == u.booleanFirst;
        if (u.booleanFirst.empty() || !firstExists || name == u.booleanFirst) {
            m_uiState.booleanFirst = name;
            m_uiState.selectedBody = hit->body;
            setStatus(name + " picked: now click the body to " + (u.booleanOp == 0 ? "join to it" : u.booleanOp == 1 ? "cut from it" : "intersect it with"));
            return;
        }
        static const char* const kOps[] = {"UNION", "SUBTRACT", "INTERSECT"};
        runCommandLine("BOOLEAN " + quotedName(u.booleanFirst) + " " + kOps[std::clamp(u.booleanOp, 0, 2)] + " " + quotedName(name) +
                       (u.booleanKeep ? " KEEP" : ""));
        m_uiState.booleanFirst.clear();
        return;
    }
    if (!hit) {
        setStatus("Click a surface of a body");
        return;
    }
    const std::string body = quotedName(m_doc.scene.bodies()[static_cast<std::size_t>(hit->body)].name);
    const std::string at = " AT " + mmVec(hit->point);
    std::string line;
    switch (tool) {
    case ToolId::CutHole:
        line = "HOLE " + body + at + " DIAMETER " + mmText(u.holeDiameter) + (u.holeThrough ? "" : " DEPTH " + mmText(u.holeDepth));
        break;
    case ToolId::CutPocket: {
        static const char* const kShapes[] = {"BOX", "ROUND", "BALL"};
        line = "POCKET " + body + at + " " + kShapes[std::clamp(u.pocketShape, 0, 2)] + " WIDTH " + mmText(u.pocketWidth) +
               " LENGTH " + mmText(u.pocketShape == 0 ? u.pocketLength : u.pocketWidth) + " DEPTH " + mmText(u.pocketDepth);
        if (u.pocketShape == 0 && u.pocketAngle != 0.0f) {
            char angle[48];
            std::snprintf(angle, sizeof(angle), " ANGLE %.6g", static_cast<double>(u.pocketAngle));
            line += angle;
        }
        break;
    }
    case ToolId::DeleteFace:
        line = "FACE DELETE " + body + at;
        break;
    case ToolId::AddBoss: {
        static const char* const kShapes[] = {"CYLINDER", "BOX", "BALL"};
        line = "BOSS " + body + at + " " + kShapes[std::clamp(u.bossShape, 0, 2)] + " DIAMETER " + mmText(u.bossDiameter) +
               " HEIGHT " + mmText(u.bossHeight);
        break;
    }
    case ToolId::InsertMesh:
        line = "INSERTMESH " + body + " " + insertShapeWords(u) + at + " SIZE " + mmText(u.insertSize) + insertTailWords(u);
        break;
    default: return;
    }
    runCommandLine(line);
}

void Application::applyWearableTool(ToolId tool, float x, float y) {
    const UiState& u = m_uiState;
    if (tool == ToolId::InterferenceCheck) {
        runCommandLine("INTERFERENCE CLEARANCE " + mmText(u.interferenceClearance));
        return;
    }
    const auto hit = pick(x, y);
    if (!hit) {
        setStatus("Click a body");
        return;
    }
    const Body& body = m_doc.scene.bodies()[static_cast<std::size_t>(hit->body)];
    const std::string name = quotedName(body.name);
    const Vec3 n = normalize(m_brushNormal);
    switch (tool) {
    case ToolId::ShellBody: {
        std::string line = "SHELL " + name + " THICKNESS " + mmText(u.shellThickness) + (u.shellOutward ? " OUTWARD" : "");
        if (u.shellOpen && m_brushNormalValid && length(n) > 0.5f)
            line += " OPEN " + mmVec(hit->point - n * u.shellOpenDepth) + " NORMAL " + dirVec(n);
        runCommandLine(line);
        break;
    }
    case ToolId::ArmorBody:
        runCommandLine("ARMOR " + name + " CLEARANCE " + mmText(u.armorClearance) + " THICKNESS " + mmText(u.armorThickness));
        break;
    case ToolId::JointBody: {
        // The parent: the nearest other part (by box) that is not this part's own armour or hardware.
        int parent = -1;
        float best = 1e30f;
        const auto& bodies = m_doc.scene.bodies();
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            if (static_cast<int>(i) == hit->body || !bodies[i].visible || bodies[i].attachedTo == body.name) continue;
            // Joint hardware (pins, balls) sits on the pivot but is never the parent.
            const std::string& nm = bodies[i].name;
            auto endsWith = [&](const char* tail) {
                const std::size_t len = std::strlen(tail);
                return nm.size() >= len && nm.compare(nm.size() - len, len, tail) == 0;
            };
            if (endsWith(" Hinge Pin") || endsWith(" Ball Joint")) continue;
            const AABB b = bodies[i].bounds();
            if (!b.valid()) continue;
            const Vec3 q = vmax(b.min, vmin(hit->point, b.max));
            const float d = length(q - hit->point);
            if (d < best) {
                best = d;
                parent = static_cast<int>(i);
            }
        }
        const Vec3 f = camera().forward();
        const std::string axis = u.jointAxis == 0 ? "X" : u.jointAxis == 1 ? "Y" : u.jointAxis == 2 ? "Z" : dirVec(f);
        std::string line = "JOINT " + name;
        if (parent >= 0 && best < 50.0f) line += " TO " + quotedName(bodies[static_cast<std::size_t>(parent)].name);
        char range[96];
        const float lo = std::min(u.jointLo, u.jointHi), hi = std::max(std::max(u.jointLo, u.jointHi), lo + 1.0f);
        std::snprintf(range, sizeof(range), " RANGE %g %g PERIOD %g", static_cast<double>(lo), static_cast<double>(hi),
                      static_cast<double>(std::max(u.jointPeriod, 0.2f)));
        line += " AT " + mmVec(hit->point) + " AXIS " + axis + (u.jointBall ? " BALL" : " HINGE") + range;
        runCommandLine(line);
        break;
    }
    default: // Lighten / TPMS lattice
        runCommandLine("LIGHTEN " + name + " SKIN " + mmText(u.latticeSkin) +
                       (u.latticeHollow ? std::string(" HOLLOW")
                                        : " CELL " + mmText(u.latticeCell) + " WALL " + mmText(u.latticeWall)));
        break;
    }
}

void Application::finishCutDrag(ToolId tool, float x, float y) {
    const ImGuiIO& io = ImGui::GetIO();
    if (tool == ToolId::CutChannel) {
        if (m_cutPath.size() < 2 || m_cutBody < 0 || m_cutBody >= static_cast<int>(m_doc.scene.bodies().size())) {
            setStatus("Drag along a surface to carve a channel");
            m_cutPath.clear();
            return;
        }
        // At most ~100 path points (the command resamples and smooths them anyway).
        const std::size_t every = m_cutPath.size() / 100 + 1;
        std::string line = "CHANNEL " + quotedName(m_doc.scene.bodies()[static_cast<std::size_t>(m_cutBody)].name) + " PATH";
        for (std::size_t i = 0; i < m_cutPath.size(); i += every) line += " " + mmVec(m_cutPath[i]);
        if ((m_cutPath.size() - 1) % every != 0) line += " " + mmVec(m_cutPath.back());
        line += " WIDTH " + mmText(m_uiState.channelWidth);
        if (m_uiState.channelDepth > 0) line += " DEPTH " + mmText(m_uiState.channelDepth);
        m_cutPath.clear();
        runCommandLine(line);
        return;
    }
    if (tool == ToolId::CurveBrush) {
        if (m_cutPath.size() < 2 || m_cutBody < 0 || m_cutBody >= static_cast<int>(m_doc.scene.bodies().size())) {
            setStatus("Drag along a surface to lay the curve");
            m_cutPath.clear();
            return;
        }
        const UiState& u = m_uiState;
        const std::size_t every = m_cutPath.size() / 100 + 1;
        std::string line = "CURVE " + quotedName(m_doc.scene.bodies()[static_cast<std::size_t>(m_cutBody)].name) +
                           (u.curveTube ? std::string(" TUBE") : " INSERT " + insertShapeWords(u)) + " PATH";
        for (std::size_t i = 0; i < m_cutPath.size(); i += every) line += " " + mmVec(m_cutPath[i]);
        if ((m_cutPath.size() - 1) % every != 0) line += " " + mmVec(m_cutPath.back());
        if (u.curveTube) line += " RADIUS " + mmText(u.curveRadius);
        else line += " SIZE " + mmText(u.insertSize) + (u.curveSpacing > 0 ? " SPACING " + mmText(u.curveSpacing) : std::string());
        line += insertTailWords(u);
        m_cutPath.clear();
        runCommandLine(line);
        return;
    }
    if (tool != ToolId::CutKnife) return;
    if (std::hypot(x - m_knifeStart.x, y - m_knifeStart.y) < 6.0f) {
        setStatus("Drag a line across a body to slice it");
        return;
    }
    // The body: the selection if the line crosses it, else the first body under the line.
    int bi = -1;
    const int sel = m_uiState.selectedBody;
    for (int k = 0; k <= 32; ++k) {
        const float t = static_cast<float>(k) / 32.0f;
        if (const auto hit = pick(m_knifeStart.x + (x - m_knifeStart.x) * t, m_knifeStart.y + (y - m_knifeStart.y) * t)) {
            if (bi < 0) bi = hit->body;
            if (hit->body == sel) {
                bi = sel;
                break;
            }
        }
    }
    if (bi < 0 && sel >= 0 && sel < static_cast<int>(m_doc.scene.bodies().size())) bi = sel; // a line beside it
    if (bi < 0) {
        setStatus("Drag the knife across a body (or select one first)");
        return;
    }
    const Body& body = m_doc.scene.bodies()[static_cast<std::size_t>(bi)];
    const Vec3 centre = body.bounds().center();
    const Ray r1 = camera().screenRay(m_knifeStart.x, m_knifeStart.y, io.DisplaySize.x, io.DisplaySize.y);
    const Ray r2 = camera().screenRay(x, y, io.DisplaySize.x, io.DisplaySize.y);
    const Vec3 a = r1.at(dot(centre - r1.origin, r1.dir)), b = r2.at(dot(centre - r2.origin, r2.dir));
    const Vec3 n = normalize(cross(b - a, normalize(r1.dir + r2.dir)));
    if (length(n) < 0.5f) return;
    runCommandLine("TRIM " + quotedName(body.name) + " PLANE " + mmVec(a) + " NORMAL " + dirVec(n) + " SMALLER");
}

void Application::startRadial(Radial mode) {
    const ImGuiIO& io = ImGui::GetIO();
    if (io.MousePos.x < -1e30f) return;
    m_radial = mode;
    if (mode == Radial::Size) {
        // mm a pixel where the brush is (at the surface under the cursor, else the view's centre).
        const OrbitCamera& cam = camera();
        float perPixel = cam.worldPerPixel(io.DisplaySize.y);
        if (m_brushHit && cam.projection() == Projection::Perspective && cam.distance() > 0)
            perPixel *= std::max(dot(*m_brushHit - cam.position(), cam.forward()), 1e-3f) / cam.distance();
        m_radialPerPixel = std::max(perPixel, 1e-6f);
        m_radialWas = m_uiState.sculptRadius;
        m_radialCentre = {io.MousePos.x - m_uiState.sculptRadius / m_radialPerPixel, io.MousePos.y};
    } else {
        m_radialWas = m_uiState.sculptStrength;
        m_radialCentre = {io.MousePos.x - m_uiState.sculptStrength * kRadialStrengthSpan, io.MousePos.y};
    }
}

bool Application::updateRadial() {
    if (m_radial == Radial::None) return false;
    const ImGuiIO& io = ImGui::GetIO();
    const float d = std::hypot(io.MousePos.x - m_radialCentre.x, io.MousePos.y - m_radialCentre.y);
    if (m_radial == Radial::Size) m_uiState.sculptRadius = std::clamp(d * m_radialPerPixel, 0.1f, 1e5f);
    else m_uiState.sculptStrength = std::clamp(d / kRadialStrengthSpan, 0.0f, 1.0f);
    const bool cancel = keyActionPressed("edit.cancel") || ImGui::IsMouseClicked(ImGuiMouseButton_Right);
    const bool keep = ImGui::IsMouseClicked(ImGuiMouseButton_Left) || keyActionPressed("edit.finish");
    if (cancel) (m_radial == Radial::Size ? m_uiState.sculptRadius : m_uiState.sculptStrength) = m_radialWas;
    if (cancel || keep) {
        char status[96];
        if (m_radial == Radial::Size) std::snprintf(status, sizeof status, "%s %s", tr("Brush radius"), lengthText(m_uiState.sculptRadius).c_str());
        else std::snprintf(status, sizeof status, "%s %.2f", tr("Brush strength"), static_cast<double>(m_uiState.sculptStrength));
        setStatus(status);
        m_radial = Radial::None;
    }
    return true;
}

void Application::drawSculptOverlay() {
    const ImGuiIO& io = ImGui::GetIO();
    ImDrawList* fg = ImGui::GetForegroundDrawList();
    const ToolId tool = m_ui.toolbox().active();
    if (m_radial != Radial::None) { // the radial control: its circle and value
        const float d = std::hypot(io.MousePos.x - m_radialCentre.x, io.MousePos.y - m_radialCentre.y);
        const ImU32 ring = IM_COL32(235, 238, 245, 240), fill = IM_COL32(90, 160, 255, 70);
        char label[64];
        if (m_radial == Radial::Size) {
            fg->AddCircle(m_radialCentre, d, ring, 96, 2.0f);
            std::snprintf(label, sizeof label, "%s", lengthText(m_uiState.sculptRadius).c_str());
        } else {
            fg->AddCircle(m_radialCentre, kRadialStrengthSpan, IM_COL32(235, 238, 245, 110), 96, 1.2f);
            fg->AddCircleFilled(m_radialCentre, d, fill, 96);
            fg->AddCircle(m_radialCentre, d, ring, 96, 2.0f);
            std::snprintf(label, sizeof label, "%.2f", static_cast<double>(m_uiState.sculptStrength));
        }
        fg->AddLine(m_radialCentre, io.MousePos, IM_COL32(235, 238, 245, 120), 1.0f);
        const ImVec2 ts = ImGui::CalcTextSize(label);
        fg->AddRectFilled({m_radialCentre.x - ts.x * 0.5f - 6, m_radialCentre.y - ts.y * 0.5f - 3}, {m_radialCentre.x + ts.x * 0.5f + 6, m_radialCentre.y + ts.y * 0.5f + 3},
                          IM_COL32(20, 22, 28, 210), 4.0f);
        fg->AddText({m_radialCentre.x - ts.x * 0.5f, m_radialCentre.y - ts.y * 0.5f}, IM_COL32(240, 242, 248, 255), label);
        return;
    }
    if (m_leftDrag == DragMode::BoxSelect) {
        // Left to right: bodies wholly inside (solid); right to left: bodies it touches (dashed).
        const ImVec2 a = m_boxStart, b = io.MousePos;
        const ImVec2 lo{std::min(a.x, b.x), std::min(a.y, b.y)}, hi{std::max(a.x, b.x), std::max(a.y, b.y)};
        const bool window = b.x >= a.x;
        const ImU32 edge = window ? IM_COL32(90, 160, 255, 255) : IM_COL32(110, 220, 140, 255);
        fg->AddRectFilled(lo, hi, window ? IM_COL32(90, 160, 255, 40) : IM_COL32(110, 220, 140, 40));
        if (window) {
            fg->AddRect(lo, hi, edge, 0, 0, 1.5f);
        } else {
            auto dashed = [&](ImVec2 p, ImVec2 q) {
                const float len = std::hypot(q.x - p.x, q.y - p.y);
                for (float t = 0; t < len; t += 10) {
                    const float u = t / len, v = std::min(t + 6, len) / len;
                    fg->AddLine({p.x + (q.x - p.x) * u, p.y + (q.y - p.y) * u}, {p.x + (q.x - p.x) * v, p.y + (q.y - p.y) * v}, edge, 1.5f);
                }
            };
            dashed(lo, {hi.x, lo.y}), dashed({hi.x, lo.y}, hi), dashed(hi, {lo.x, hi.y}), dashed({lo.x, hi.y}, lo);
        }
        return;
    }
    if (m_leftDrag == DragMode::Knife) {
        fg->AddLine(m_knifeStart, io.MousePos, IM_COL32(255, 120, 90, 255), 2.0f);
        fg->AddCircleFilled(m_knifeStart, 3.5f, IM_COL32(255, 120, 90, 255));
        return;
    }
    if (m_leftDrag == DragMode::CutPath && m_cutPath.size() > 1) {
        std::optional<ImVec2> prev;
        for (const Vec3& p : m_cutPath) {
            const auto s = projectToScreen(camera(), p, io.DisplaySize);
            if (s && prev) fg->AddLine(*prev, *s, IM_COL32(255, 170, 60, 255), 2.5f);
            prev = s;
        }
    }
    if (!m_brushHit || tool == ToolId::CutKnife || tool == ToolId::BooleanBodies || tool == ToolId::PushPull || tool == ToolId::DeleteFace) return;
    if (isWearableTool(tool)) { // the clicked point and its normal
        const auto c = projectToScreen(camera(), *m_brushHit, io.DisplaySize);
        const AABB vb = m_doc.scene.bounds();
        const float len = vb.valid() ? length(vb.max - vb.min) * 0.04f : 10.0f;
        const auto tip = projectToScreen(camera(), *m_brushHit + normalize(m_brushNormal) * len, io.DisplaySize);
        const ImU32 col = IM_COL32(120, 220, 160, 240);
        if (c) fg->AddCircle(*c, 6.0f, col, 16, 1.6f);
        if (c && tip) fg->AddLine(*c, *tip, col, 1.4f);
        return;
    }
    const UiState& u = m_uiState;
    float r = u.sculptRadius;
    ImU32 col = IM_COL32(235, 238, 245, 230);
    if (isSculptTool(tool)) {
        if (io.KeyShift) col = IM_COL32(120, 190, 255, 240);         // smoothing
        else if (commandModifierHeld()) col = IM_COL32(255, 120, 110, 240); // inverted
    } else {
        col = IM_COL32(255, 170, 60, 240);
        r = tool == ToolId::CutHole     ? u.holeDiameter * 0.5f
            : tool == ToolId::CutPocket ? u.pocketWidth * 0.5f
            : tool == ToolId::AddBoss   ? u.bossDiameter * 0.5f
            : tool == ToolId::InsertMesh ? u.insertSize * 0.5f
            : tool == ToolId::CurveBrush ? (u.curveTube ? u.curveRadius : u.insertSize * 0.5f)
                                        : u.channelWidth * 0.5f;
    }
    // The brush footprint drawn on the surface plane, and the normal.
    const Vec3 n = normalize(m_brushNormal);
    const Vec3 ref = std::fabs(n.z) < 0.9f ? Vec3{0, 0, 1} : Vec3{1, 0, 0};
    const Vec3 e1 = normalize(cross(n, ref)), e2 = cross(n, e1);
    std::optional<ImVec2> first, prev;
    for (int i = 0; i <= 48; ++i) {
        const float a = 6.2831853f * static_cast<float>(i) / 48.0f;
        const auto s = projectToScreen(camera(), *m_brushHit + (e1 * std::cos(a) + e2 * std::sin(a)) * r, io.DisplaySize);
        if (s && prev) fg->AddLine(*prev, *s, col, 1.6f);
        prev = s;
    }
    const auto c = projectToScreen(camera(), *m_brushHit, io.DisplaySize);
    const auto tip = projectToScreen(camera(), *m_brushHit + n * (r * 0.6f), io.DisplaySize);
    if (c && tip) fg->AddLine(*c, *tip, col, 1.4f);
    if (c) fg->AddCircleFilled(*c, 2.0f, col);
    if (m_uiState.sculptSymmetry && isSculptTool(tool) && m_sculptBody >= 0) {
        const Vec3 m{2.0f * m_sculptMesh.mirrorX() - m_brushHit->x, m_brushHit->y, m_brushHit->z};
        if (const auto ms = projectToScreen(camera(), m, io.DisplaySize)) fg->AddCircle(*ms, 4.0f, col, 12, 1.2f);
    }
}

void Application::handleViewportInput() {
    ImGuiIO& io = ImGui::GetIO();
    const float mx = io.MousePos.x, my = io.MousePos.y;
    const float dx = io.MouseDelta.x, dy = io.MouseDelta.y;
    const float wh = io.DisplaySize.y;
    const bool onCanvas = !io.WantCaptureMouse;
    const bool is2D = m_uiState.mode == ViewMode::Drawing2D;
    ToolId tool = m_ui.toolbox().active();
    // Simulation: only navigation; every other tool selects.
    if (simulating() && tool != ToolId::Orbit && tool != ToolId::Pan && tool != ToolId::Zoom) tool = ToolId::Select;
    OrbitCamera& cam = camera();
    // In the 2D window the view never tilts: every "orbit" gesture pans.
    const DragMode orbitOrPan = is2D ? DragMode::Pan : DragMode::Orbit;
    if (updateRadial()) return; // the radial control (F) has the mouse

    // Sketch cursor (also feeds the rulers and status bar).
    m_uiState.cursorPlane.reset();
    if (onCanvas && io.MousePos.x > -1e30f) {
        const SnapOptions so{m_uiState.snapGrid, m_uiState.snapPoints};
        if (isSketchTool(tool)) {
            if (m_sketcher.hover(m_doc.scene, cam, m_uiState.plane, activePlaneFrame(m_doc.state), io.MousePos, io.DisplaySize, so, io.KeyShift))
                m_uiState.cursorPlane = m_sketcher.cursor();
        } else {
            const PlaneFrame f = activePlaneFrame(m_doc.state);
            const Ray r = cam.screenRay(mx, my, io.DisplaySize.x, wh);
            const float d = dot(r.dir, f.normal);
            if (std::fabs(d) > 0.05f) m_uiState.cursorPlane = f.toPlane(r.at(dot(f.origin - r.origin, f.normal) / d));
        }
    }

    if (tool != ToolId::BooleanBodies) m_uiState.booleanFirst.clear();

    // Extrude: the profile must still exist and the tool still be active.
    if (m_extrudeCurve >= 0 &&
        (tool != ToolId::Extrude || m_extrudeCurve >= static_cast<int>(m_doc.scene.sketch().curves().size())))
        cancelExtrude();
    if (m_extrudeCurve >= 0 && onCanvas && m_otherDrag == DragMode::None && m_leftDrag == DragMode::None)
        updateExtrudeFromMouse();

    // Right drag orbits (3D) / pans (2D); middle drag pans; any tool. A right click (no drag)
    // on a body in 3D opens its menu, for it or for the selection it is in.
    if (onCanvas && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) m_rightClickOnCanvas = true;
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Right)) {
        const bool click = m_rightClickOnCanvas && std::sqrt(io.MouseDragMaxDistanceSqr[ImGuiMouseButton_Right]) < kDragThreshold;
        m_rightClickOnCanvas = false;
        if (click && onCanvas && !is2D && !simulating() && m_leftDrag == DragMode::None && !m_sketcher.busy() && m_extrudeCurve < 0 &&
            m_uiState.booleanFirst.empty()) {
            if (const auto hit = pick(mx, my)) {
                const std::vector<int> sel = selectedBodies(m_doc.scene, m_uiState.selectedBody, m_uiState.bodySelection);
                if (std::find(sel.begin(), sel.end(), hit->body) == sel.end()) {
                    m_uiState.selectedBody = hit->body;
                    m_uiState.bodySelection = {};
                    m_uiState.selectedCurve = m_uiState.selectedImage = -1;
                    selectionChanged();
                }
                m_uiState.openBodyMenu = true;
            }
        }
    }
    if (onCanvas && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) m_otherDrag = orbitOrPan;
    if (onCanvas && ImGui::IsMouseClicked(ImGuiMouseButton_Middle)) m_otherDrag = DragMode::Pan;
    if (!ImGui::IsMouseDown(ImGuiMouseButton_Right) && !ImGui::IsMouseDown(ImGuiMouseButton_Middle))
        m_otherDrag = DragMode::None;
    if (m_otherDrag == DragMode::Orbit) cam.orbit(dx, dy);
    if (m_otherDrag == DragMode::Pan) cam.pan(dx, dy, wh);

    // The gizmo on the selected body (Move tool): the handle under the mouse.
    Vec3 gizmoPivot{};
    float gizmoSize = 0;
    const bool gizmo = tool == ToolId::Move && !is2D && !simulating() && gizmoShown(gizmoPivot, gizmoSize);
    const GizmoMode gizmoMode = static_cast<GizmoMode>(std::clamp(m_uiState.gizmoMode, 0, 2));
    if (m_leftDrag != DragMode::Gizmo)
        m_gizmoHover = gizmo && onCanvas && m_leftDrag == DragMode::None
                           ? gizmoPick(gizmoMode, gizmoPivot, gizmoSize, cam.forward(), gizmoProjector(), {mx, my})
                           : GizmoHandle{};

    // Left button behaviour depends on the active tool.
    if (onCanvas && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        m_pressX = mx;
        m_pressY = my;
        m_leftDrag = DragMode::Pending;
        m_gripCandidate.reset();
        if (tool == ToolId::Select && !simulating() && !io.KeyShift && !commandModifierHeld()) m_gripCandidate = pickSketchGrip(mx, my);
        if (isSculptTool(tool) && beginSculpt(mx, my)) {
            m_leftDrag = m_sculptSettings.brush == SculptBrush::Grab ? DragMode::Grab : DragMode::Sculpt;
        } else if ((tool == ToolId::CutChannel || tool == ToolId::CurveBrush) && pick(mx, my)) {
            const auto hit = pick(mx, my);
            m_leftDrag = DragMode::CutPath;
            m_cutTool = tool;
            m_cutBody = hit->body;
            m_cutPath = {hit->point};
        } else if (tool == ToolId::CutKnife) {
            m_leftDrag = DragMode::Knife;
            m_knifeStart = {mx, my};
        } else if (tool == ToolId::PushPull) {
            if (beginPushPull(mx, my)) m_leftDrag = DragMode::PushPull;
        } else if (io.KeyShift && !isSketchTool(tool)) {
            m_leftDrag = DragMode::Pan;
        } else if (tool == ToolId::Orbit) {
            m_leftDrag = orbitOrPan;
        } else if (tool == ToolId::Pan) {
            m_leftDrag = DragMode::Pan;
        } else if (tool == ToolId::Zoom) {
            m_leftDrag = DragMode::Zoom;
        } else if (tool == ToolId::Move && gizmo && m_gizmoHover.kind != GizmoHandle::None) {
            // On a gizmo handle: it drags the selected body that way.
            m_leftDrag = DragMode::Gizmo;
            m_gizmoHandle = m_gizmoHover;
            m_gizmoBody = m_uiState.selectedBody;
            m_gizmoBodies = selectedBodies(m_doc.scene, m_uiState.selectedBody, m_uiState.bodySelection);
            m_gizmoStarts.clear();
            for (int b : m_gizmoBodies) m_gizmoStarts.push_back(m_doc.scene.bodies()[static_cast<std::size_t>(b)]);
            if (gizmoMode == GizmoMode::Move) startSnap(m_gizmoBodies);
            m_gizmoPivot = gizmoPivot;
            m_gizmoSize = gizmoSize;
            const Ray r = cam.screenRay(mx, my, io.DisplaySize.x, wh);
            m_gizmoFrom = {r.origin, r.dir, {mx, my}};
            m_gizmoDelta = {};
        } else if (tool == ToolId::Move && !commandModifierHeld()) { // (Ctrl+click: adds to the selection)
            Vec2 grab{};
            if (const auto hit = pick(mx, my)) {
                // On a selected body: all the selected ones move with it; else it alone (selected).
                m_leftDrag = DragMode::MoveBody;
                m_moveBodies = selectedBodies(m_doc.scene, m_uiState.selectedBody, m_uiState.bodySelection);
                if (const auto it = std::find(m_moveBodies.begin(), m_moveBodies.end(), hit->body); it != m_moveBodies.end()) {
                    std::rotate(m_moveBodies.begin(), it, it + 1); // the one grabbed first
                } else {
                    m_uiState.selectedBody = hit->body;
                    m_uiState.bodySelection = {};
                    m_moveBodies = {hit->body};
                }
                m_uiState.selectedCurve = m_uiState.selectedImage = -1;
                m_movePlanePoint = hit->point;
                m_moveStarts.clear();
                for (int b : m_moveBodies) m_moveStarts.push_back(m_doc.scene.bodies()[static_cast<std::size_t>(b)].offset);
                startSnap(m_moveBodies);
            } else if (const int img = pickImage(mx, my, &grab); img >= 0) {
                // Images slide within their own plane.
                m_leftDrag = DragMode::MoveImage;
                m_uiState.selectedImage = img;
                m_uiState.selectedBody = m_uiState.selectedCurve = -1;
                m_imageGrab = grab;
                m_imageStart = m_doc.scene.images()[static_cast<std::size_t>(img)].center;
            }
        }
    }

    if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && !m_escapedDrag) {
        if (m_leftDrag == DragMode::Pending && std::hypot(mx - m_pressX, my - m_pressY) > kDragThreshold) {
            if (m_gripCandidate) { // pressed on sketch geometry: drag it
                m_leftDrag = DragMode::SketchGrip;
                m_sketchDrag = *m_gripCandidate;
                m_sketchDragStart = m_doc.scene.sketch();
                m_sketchDragMoved = false;
                m_uiState.selectedCurve = m_sketchDrag.curve;
                m_uiState.selectedBody = m_uiState.selectedImage = -1;
            } else if (commandModifierHeld() && (tool == ToolId::Select || tool == ToolId::Move) && !simulating()) {
                m_leftDrag = DragMode::BoxSelect; // Ctrl+drag: a box adding the bodies in it
                m_boxStart = {m_pressX, m_pressY};
            } else
                m_leftDrag = isSketchTool(tool) ? DragMode::None : orbitOrPan; // dragging empty space navigates
        }
        switch (m_leftDrag) {
        case DragMode::Orbit: cam.orbit(dx, dy); break;
        case DragMode::Pan: cam.pan(dx, dy, wh); break;
        case DragMode::Zoom: cam.zoomAt(-dy * 0.02f, m_pressX, m_pressY, io.DisplaySize.x, wh); break;
        case DragMode::MoveBody: {
            // Slide in the plane through the grab point, facing the camera.
            const Ray ray = cam.screenRay(mx, my, io.DisplaySize.x, wh);
            float t = 0.0f;
            if (!intersectPlane(ray, m_movePlanePoint, cam.forward(), t)) break;
            Vec3 by = ray.at(t) - m_movePlanePoint;
            m_snapHit.reset();
            // Onto a point in sight, in full 3D; else a hidden one, within the view.
            if (m_uiState.snapMoveToPoints && (m_snapHit = freeSnap(by))) by = m_snapHit->move;
            for (std::size_t k = 0; k < m_moveBodies.size(); ++k)
                if (m_moveBodies[k] >= 0 && m_moveBodies[k] < static_cast<int>(m_doc.scene.bodies().size()))
                    m_doc.scene.bodies()[static_cast<std::size_t>(m_moveBodies[k])].offset = m_moveStarts[k] + by;
            m_doc.scene.touchBodies();
            break;
        }
        case DragMode::Gizmo: {
            if (m_gizmoBody < 0 || m_gizmoBody >= static_cast<int>(m_doc.scene.bodies().size())) break;
            const Ray r = cam.screenRay(mx, my, io.DisplaySize.x, wh);
            const bool snap = m_uiState.gizmoSnap || commandModifierHeld();
            const float step = !snap ? 0.0f : gizmoMode == GizmoMode::Move ? m_uiState.gizmoMoveStep : gizmoMode == GizmoMode::Rotate ? m_uiState.gizmoAngleStep : m_uiState.gizmoScaleStep;
            m_gizmoDelta = gizmoDrag(gizmoMode, m_gizmoHandle, m_gizmoPivot, cam.forward(), gizmoProjector(), m_gizmoFrom, {r.origin, r.dir, {mx, my}}, step, &m_gizmoDelta);
            m_snapHit.reset();
            if (gizmoMode == GizmoMode::Move && m_uiState.snapMoveToPoints) { // onto another body's corner, edge or face middle
                const Vec3 e = m_gizmoHandle.axis == 0 ? Vec3{1, 0, 0} : m_gizmoHandle.axis == 1 ? Vec3{0, 1, 0} : Vec3{0, 0, 1};
                const bool along = m_gizmoHandle.kind == GizmoHandle::Axis, inPlane = m_gizmoHandle.kind == GizmoHandle::Plane;
                // The dot: onto a point in sight in full 3D, else a hidden one within the view.
                if (along || inPlane) m_snapHit = gizmoSnap(m_snapMoving, m_snapTargets, m_gizmoDelta.move, gizmoProjector(), 12.0f, along ? e : Vec3{}, inPlane ? e : Vec3{});
                else m_snapHit = freeSnap(m_gizmoDelta.move);
                if (m_snapHit) m_gizmoDelta.move = m_snapHit->move;
            }
            // The preview: the body as it began, done to as the drag says so far.
            for (std::size_t k = 0; k < m_gizmoBodies.size(); ++k) {
                Body& b = m_doc.scene.bodies()[static_cast<std::size_t>(m_gizmoBodies[k])];
                b = m_gizmoStarts[k];
                if (gizmoMode == GizmoMode::Move) b.offset += m_gizmoDelta.move;
                else if (gizmoMode == GizmoMode::Rotate) turnBody(b, m_gizmoDelta.axis, m_gizmoDelta.angle, m_gizmoPivot);
                else stretchBody(b, m_gizmoDelta.scale, m_gizmoPivot);
            }
            m_doc.scene.touchBodies();
            break;
        }
        case DragMode::MoveImage: {
            const int sel = m_uiState.selectedImage;
            if (sel < 0 || sel >= static_cast<int>(m_doc.scene.images().size())) break;
            ImageRef& img = m_doc.scene.images()[static_cast<std::size_t>(sel)];
            const PlaneFrame f = planeFrame(img.plane);
            const Ray ray = cam.screenRay(mx, my, io.DisplaySize.x, wh);
            const float d = dot(ray.dir, f.normal);
            if (std::fabs(d) < 1e-4f) break;
            const Vec2 q = f.toPlane(ray.at(dot(f.origin - ray.origin, f.normal) / d));
            img.center = {m_imageStart.x + q.x - m_imageGrab.x, m_imageStart.y + q.y - m_imageGrab.y};
            break;
        }
        case DragMode::Sculpt:
        case DragMode::Grab: continueSculpt(mx, my); break;
        case DragMode::SketchGrip: updateSketchDrag(mx, my); break;
        case DragMode::PushPull: updatePushPull(mx, my); break;
        case DragMode::CutPath: {
            const auto hit = pick(mx, my);
            const float step = std::max((m_cutTool == ToolId::CurveBrush ? (m_uiState.curveTube ? m_uiState.curveRadius : m_uiState.insertSize * 0.25f)
                                                                          : m_uiState.channelWidth * 0.5f),
                                        0.5f);
            if (hit && hit->body == m_cutBody && length(hit->point - m_cutPath.back()) >= step) m_cutPath.push_back(hit->point);
            break;
        }
        default: break;
        }
    }

    if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        if (m_leftDrag == DragMode::Pending) onCanvasClick(mx, my, false);
        else if (m_leftDrag == DragMode::Sculpt || m_leftDrag == DragMode::Grab) endSculpt();
        else if (m_leftDrag == DragMode::CutPath) finishCutDrag(m_cutTool, mx, my);
        else if (m_leftDrag == DragMode::Knife) finishCutDrag(ToolId::CutKnife, mx, my);
        else if (m_leftDrag == DragMode::PushPull) finishPushPull();
        else if (m_leftDrag == DragMode::SketchGrip) finishSketchDrag();
        else if (m_leftDrag == DragMode::MoveBody) finishMoveDrag(true);
        else if (m_leftDrag == DragMode::Gizmo) finishGizmoDrag(true);
        else if (m_leftDrag == DragMode::BoxSelect) finishBoxSelect(mx, my);
        m_leftDrag = DragMode::None;
        m_gripCandidate.reset();
        m_escapedDrag = false;
    }
    // Brush / tool cursor on the surface under the mouse.
    m_brushHit.reset();
    if (onCanvas && !simulating() && (isSculptTool(tool) || isCutTool(tool) || isWearableTool(tool)) && m_leftDrag != DragMode::Orbit &&
        m_leftDrag != DragMode::Pan) {
        m_brushNormalValid = false;
        if (m_leftDrag == DragMode::Sculpt || m_leftDrag == DragMode::Grab) {
            Vec3 n;
            if (const auto p = m_sculptMesh.raycast(cam.screenRay(mx, my, io.DisplaySize.x, wh), &n)) {
                m_brushHit = *p;
                m_brushNormal = n;
                m_brushNormalValid = true;
            }
        } else if (const auto hit = pick(mx, my)) {
            m_brushHit = hit->point;
            const Body& b = m_doc.scene.bodies()[static_cast<std::size_t>(hit->body)];
            const Ray r = cam.screenRay(mx, my, io.DisplaySize.x, wh);
            const Ray local{r.origin - b.offset, r.dir};
            float best = 1e30f;
            for (std::size_t k = 0; k + 2 < b.mesh.indices.size(); k += 3) {
                const Vec3& a = b.mesh.vertices[b.mesh.indices[k]].position;
                const Vec3& c1 = b.mesh.vertices[b.mesh.indices[k + 1]].position;
                const Vec3& c2 = b.mesh.vertices[b.mesh.indices[k + 2]].position;
                float t = 0;
                if (intersectTriangle(local, a, c1, c2, t) && t < best) {
                    best = t;
                    m_brushNormal = normalize(cross(c1 - a, c2 - a));
                    m_brushNormalValid = true;
                }
            }
            if (dot(m_brushNormal, r.dir) > 0) m_brushNormal = -m_brushNormal;
        }
    }
    if (onCanvas && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left) && tool == ToolId::Polyline)
        if (auto c = m_sketcher.finish()) addCurve(std::move(*c));

    if (m_pendingScroll != 0.0f) {
        cam.zoomAt(m_pendingScroll, mx, my, io.DisplaySize.x, wh);
        m_pendingScroll = 0.0f;
    }

    if (io.WantCaptureKeyboard) return;
    if (!isSketchTool(m_ui.toolbox().active())) m_sketcher.cancel();
    const ToolId before = m_ui.toolbox().active();
    // V again with the Move tool on: the gizmo moves, then rotates, then scales.
    if (m_leftDrag == DragMode::None && before == ToolId::Move && keyActionPressed(toolKeyId(toolInfo(ToolId::Move).name)))
        m_uiState.gizmoMode = (m_uiState.gizmoMode + 1) % 3;
    if (m_leftDrag == DragMode::None) m_ui.toolbox().handleShortcuts(); // not mid-stroke
    if (m_ui.toolbox().active() != before) m_sketcher.cancel();

    UiCommands cmd;
    // The keys come from the keymap (Keymap.hpp): the user's, else the defaults.
    if (keyActionPressed("view.simulation")) cmd.mode = simulating() ? ViewMode::Model3D : ViewMode::Simulation;
    else if (keyActionPressed("view.windows")) cmd.mode = is2D ? ViewMode::Model3D : simulating() ? ViewMode::Model3D : ViewMode::Drawing2D;
    if (simulating()) { // playback keys
        if (keyActionPressed("sim.play")) {
            if (!m_uiState.simPlaying && m_uiState.simTime >= m_uiState.simDuration - 1e-4f) m_uiState.simTime = 0.0f;
            m_uiState.simPlaying = !m_uiState.simPlaying;
        }
        if (keyActionPressed("sim.rewind")) m_uiState.simTime = 0.0f;
        if (keyActionPressed("sim.back", true)) m_uiState.simTime = std::max(m_uiState.simTime - 1.0f / 30.0f, 0.0f);
        if (keyActionPressed("sim.forward", true)) m_uiState.simTime += 1.0f / 30.0f;
    }
    static const std::pair<const char*, StandardView> kViews[] = {{"view.front", StandardView::Front}, {"view.right", StandardView::Right},
                                                                   {"view.top", StandardView::Top},     {"view.back", StandardView::Back},
                                                                   {"view.left", StandardView::Left},   {"view.bottom", StandardView::Bottom},
                                                                   {"view.iso", StandardView::Isometric}};
    for (const auto& [id, v] : kViews)
        if (keyActionPressed(id)) cmd.view = v;
    // Fit fits the view; with a sculpt brush it sizes the brush (with Shift: its strength), as in Blender.
    bool fitShift = false;
    for (KeyChord c : boundChords(preferences(), "view.fit"))
        if (!c.shift && (c.shift = true, chordPressed(c))) fitShift = true;
    if (const bool fit = keyActionPressed("view.fit"); fit || fitShift) {
        if (isSculptTool(m_ui.toolbox().active()) && !is2D && m_leftDrag == DragMode::None)
            startRadial(fitShift ? Radial::Strength : Radial::Size);
        else if (fit)
            cmd.fit = true;
    }
    auto togglePage = [&](int page) { // M: Model page, K: Library page, J: CAM page of the docked panel
        m_uiState.showModelTree = !(m_uiState.showModelTree && m_uiState.dockTab == page);
        m_uiState.dockTab = page;
    };
    if (keyActionPressed("panel.model")) togglePage(0);
    if (keyActionPressed("panel.library")) togglePage(1);
    if (keyActionPressed("panel.cam")) togglePage(2);
    if (keyActionPressed("help.help")) m_uiState.showHelp = !m_uiState.showHelp;
    if (keyActionPressed("file.quit")) cmd.quit = true;
    if (isSculptTool(m_ui.toolbox().active())) { // brush size, like Photoshop and ZBrush; with Shift: its strength
        char said[64] = "";
        bool shift = false;
        if (keyActionPressed("brush.smaller", true)) m_uiState.sculptRadius = std::max(m_uiState.sculptRadius / 1.2f, 0.1f), std::snprintf(said, sizeof said, "-");
        if (keyActionPressed("brush.larger", true)) m_uiState.sculptRadius = std::min(m_uiState.sculptRadius * 1.2f, 1e5f), std::snprintf(said, sizeof said, "+");
        if (keyActionPressed("brush.weaker", true))
            m_uiState.sculptStrength = std::max(m_uiState.sculptStrength - 0.05f, 0.0f), shift = true, std::snprintf(said, sizeof said, "-");
        if (keyActionPressed("brush.stronger", true))
            m_uiState.sculptStrength = std::min(m_uiState.sculptStrength + 0.05f, 1.0f), shift = true, std::snprintf(said, sizeof said, "+");
        if (said[0]) {
            char status[96];
            if (shift) std::snprintf(status, sizeof status, "%s %.2f", tr("Brush strength"), static_cast<double>(m_uiState.sculptStrength));
            else std::snprintf(status, sizeof status, "%s %s", tr("Brush radius"), lengthText(m_uiState.sculptRadius).c_str());
            setStatus(status);
        }
    }
    if (keyActionPressed("view.projection") && !is2D) cmd.toggleProjection = true;
    if (keyActionPressed("view.grid")) viewport().showGrid = !viewport().showGrid;
    if (keyActionPressed("view.display") && !is2D)
        m_vp3D.displayMode = static_cast<DisplayMode>((static_cast<int>(m_vp3D.displayMode) + 1) % 4);
    if (keyActionPressed("edit.finish")) {
        if (m_extrudeCurve >= 0) applyExtrude();
        else if (auto c = m_sketcher.finish()) addCurve(std::move(*c));
    }
    if (keyActionPressed("edit.cancel")) {
        if (m_leftDrag != DragMode::None && m_leftDrag != DragMode::Pending) {
            cancelInteraction(false);
            m_escapedDrag = true; // the button is still down: ignore it until released
        } else if (!m_uiState.booleanFirst.empty()) {
            m_uiState.booleanFirst.clear();
            setStatus("Boolean cancelled");
        } else if (m_extrudeCurve >= 0) {
            cancelExtrude();
            setStatus("Extrude cancelled");
        } else if (m_sketcher.busy()) {
            m_sketcher.cancel();
            setStatus("Drawing cancelled");
        } else {
            m_uiState.selectedBody = m_uiState.selectedCurve = m_uiState.selectedImage = -1;
            m_uiState.bodySelection = {};
            m_uiState.curveSelection = {};
        }
    }
    if (m_leftDrag == DragMode::None) { // hide, isolate (Alt+H, Alt+I), show all (Alt+Shift+H)
        std::string bodies; // the selected bodies, by name (as Delete)
        for (int b : selectedBodies(m_doc.scene, m_uiState.selectedBody, m_uiState.bodySelection))
            bodies += " \"" + m_doc.scene.bodies()[static_cast<std::size_t>(b)].name + "\"";
        if (keyActionPressed("edit.show-all")) runCommandLine("SHOW ALL");
        if (keyActionPressed("edit.hide")) {
            if (!bodies.empty()) runCommandLine("HIDE" + bodies);
            else setStatus(tr("Select the bodies to hide first"));
        }
        if (keyActionPressed("edit.isolate")) {
            if (!bodies.empty()) runCommandLine("ISOLATE" + bodies);
            else setStatus(tr("Select the bodies to isolate first"));
        }
    }
    if (keyActionPressed("edit.delete") && m_leftDrag == DragMode::None) {
        // By the ERASE command, so it is in the history and undoes.
        const auto& curves = m_doc.scene.sketch().curves();
        std::string name;
        if (m_uiState.selectedCurve >= 0 && m_uiState.selectedCurve < static_cast<int>(curves.size())) {
            const std::vector<int> picked = selectedCurves(m_doc.scene.sketch(), m_uiState.selectedCurve, m_uiState.curveSelection);
            for (int c : picked) name += (name.empty() ? "" : "\" \"") + curves[static_cast<std::size_t>(c)].name; // every selected curve
        } else if (m_uiState.selectedImage >= 0 && m_uiState.selectedImage < static_cast<int>(m_doc.scene.images().size()))
            name = m_doc.scene.images()[static_cast<std::size_t>(m_uiState.selectedImage)].name;
        std::string bodies; // every selected body, by name
        for (int b : selectedBodies(m_doc.scene, m_uiState.selectedBody, m_uiState.bodySelection))
            bodies += " \"" + m_doc.scene.bodies()[static_cast<std::size_t>(b)].name + "\"";
        if (m_uiState.selectedCurve < 0 && !bodies.empty()) runCommandLine("ERASE" + bodies);
        else if (!name.empty()) runCommandLine("ERASE \"" + name + "\"");
    }
    const bool selectAll = keyActionPressed("edit.select-all");
    if (selectAll && is2D && m_leftDrag == DragMode::None) { // 2D: every visible sketch curve
        std::vector<int> all;
        for (std::size_t i = 0; i < m_doc.scene.sketch().curves().size(); ++i)
            if (m_doc.scene.sketch().curves()[i].visible) all.push_back(static_cast<int>(i));
        m_uiState.selectedBody = m_uiState.selectedImage = -1;
        m_uiState.bodySelection = {};
        selectCurves(m_doc.scene.sketch(), m_uiState.selectedCurve, m_uiState.curveSelection, all);
        selectionChanged();
    }
    if (selectAll && !is2D && m_leftDrag == DragMode::None) { // every visible body
        std::vector<int> all;
        for (std::size_t i = 0; i < m_doc.scene.bodies().size(); ++i)
            if (m_doc.scene.bodies()[i].visible) all.push_back(static_cast<int>(i));
        m_uiState.selectedCurve = m_uiState.selectedImage = -1;
        selectBodies(m_doc.scene, m_uiState.selectedBody, m_uiState.bodySelection, all);
        selectionChanged();
    }
    handleCommands(cmd);
}

void Application::handleCommands(const UiCommands& cmd) {
    if (cmd.switchFile)
        if (const int i = fileIndex(*cmd.switchFile); i >= 0) activateFile(i);
    if (cmd.plane) setPlane(*cmd.plane);
    if (cmd.mode) setMode(*cmd.mode);
    const bool is2D = m_uiState.mode == ViewMode::Drawing2D;

    if (cmd.view) {
        if (is2D) {
            // In the drawing window the standard views pick the drawing plane.
            switch (*cmd.view) {
            case StandardView::Top: case StandardView::Bottom: setPlane(SketchPlane::Top); break;
            case StandardView::Front: case StandardView::Back: setPlane(SketchPlane::Front); break;
            case StandardView::Left: case StandardView::Right: setPlane(SketchPlane::Right); break;
            case StandardView::Isometric:
                setMode(ViewMode::Model3D);
                m_cam3D.setStandardView(StandardView::Isometric);
                break;
            }
        } else {
            m_cam3D.setStandardView(*cmd.view);
        }
    }
    if (cmd.viewDirection) m_cam3D.setViewDirection(*cmd.viewDirection);
    if (cmd.rotateZ != 0.0f) m_cam3D.rotateAroundZ(cmd.rotateZ);
    if (cmd.roll != 0.0f && !is2D) m_cam3D.rollView(cmd.roll);
    if (cmd.resetRoll) m_cam3D.resetRoll();
    if ((cmd.orbitDelta.x != 0.0f || cmd.orbitDelta.y != 0.0f) && !is2D) m_cam3D.orbit(cmd.orbitDelta.x, cmd.orbitDelta.y);
    if (cmd.home) {
        m_cam3D.setStandardView(StandardView::Isometric, false);
        m_cam3D.fitToBounds(viewBounds());
    }
    if (cmd.fit) camera().fitToBounds(viewBounds());
    if (cmd.section) {
        m_doc.state.section = *cmd.section;
        m_framesToRender = std::max(m_framesToRender, 2);
    }
    if (cmd.toggleProjection && !is2D)
        m_cam3D.setProjection(m_cam3D.projection() == Projection::Perspective ? Projection::Orthographic
                                                                               : Projection::Perspective);
    if (cmd.quit) {
        glfwSetWindowShouldClose(m_window, GLFW_TRUE);
        if (anyFileDirty()) { // same question as closing the window
            glfwSetWindowShouldClose(m_window, GLFW_FALSE);
            m_closeRequested = true;
        }
    }
    if (cmd.toggleMcp) toggleMcp();
    if (cmd.applyExtrude) applyExtrude();
    if (cmd.cancelExtrude) cancelExtrude();
    for (const std::string& line : cmd.commands) runCommandLine(line);
    if (!cmd.commands.empty()) m_framesToRender = std::max(m_framesToRender, 3); // show what they did now, not at the next event
    if (cmd.closeFile) {
        if (const int i = fileIndex(*cmd.closeFile); i >= 0 && !closeFileTab(i, cmd.closeDiscard))
            setStatus(fileDoc(i).displayName() + " was not saved, so it stays open");
    }
}

void Application::frame(float dt) {
    // Typed lengths and every readout follow the active file's units.
    setActiveUnits(m_doc.state.units);
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // AI agents (MCP over HTTP) run on the main thread, between frames.
    if (m_mcpHttp.poll([this](const std::string& body) { return handleMcp(body); }) > 0)
        m_framesToRender = std::max(m_framesToRender, 3);

    advanceSimulation(dt);
    advanceExplode(dt);
    penInputPoll();
    handleViewportInput();
    updatePrompt();
    refreshFileTabs();
    updateSheet();
    m_uiState.assembly = &m_doc.state.assembly;
    m_uiState.project = &m_doc.state;
    m_uiState.cam = &m_doc.state.cam;
    m_uiState.lathe = &m_doc.state.lathe;
    m_uiState.activeFrame = activePlaneFrame(m_doc.state);
    m_uiState.planeLabel = activePlaneName(m_doc.state);
    m_uiState.datums.clear();
    for (const DatumPlane& d : m_doc.state.datums) m_uiState.datums.push_back(d.name);
    m_uiState.study = m_doc.study;
    m_uiState.studyCommand = m_doc.studyCommand;
    const UiCommands cmd = m_ui.draw(m_doc.scene, camera(), viewport(), m_uiState);
    handleCommands(cmd);
    pollRender();
    pollVideo();

    // Unsaved changes: compare what would be saved with the last save.
    m_doc.dirty = contentSignature() != m_savedSignature;
    m_uiState.dirty = m_doc.dirty;
    updateTitle();
    refreshFileTabs();
    autosave();
    watchComponents();
    // Files the system asked us to open (a project double-clicked in the macOS Finder).
    for (const std::string& path : takeOpenFileRequests()) {
        runCommandLine((fileKindOf(path) == FileKind::Project ? "OPEN \"" : "IMPORT \"") + path + "\"");
        m_framesToRender = std::max(m_framesToRender, 3);
    }
    if (m_updateCheck.valid() && m_updateCheck.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        const UpdateResult r = m_updateCheck.get();
        // Only news is shown; a failed check at start is a quiet log line.
        if (r.ok && r.newer) setStatus(r.describe(SICDM_VERSION));
        log(r.describe(SICDM_VERSION), r.ok ? UiState::LogLine::Output : UiState::LogLine::Error);
        m_framesToRender = std::max(m_framesToRender, 3);
    }

    if (m_offerRecovery) {
        ImGui::OpenPopup("Recover Unsaved Work");
        m_offerRecovery = false;
    }
    if (ImGui::BeginPopupModal("Recover Unsaved Work", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text("%s", tr("SI CDM didn't close normally last time. This unsaved work was kept:"));
        for (const RecoveryFile& f : m_recoverable)
            ImGui::BulletText("%s%s", f.name.c_str(), f.original.empty() ? "  (never saved)" : ("  (" + f.original + ")").c_str());
        ImGui::Spacing();
        if (ImGui::Button(tr("Restore All"), {120, 0})) {
            // Each RECOVER 1 opens the newest left and takes it off the list.
            for (std::size_t k = 0; k < m_recoverable.size(); ++k) runCommandLine("RECOVER 1");
            m_recoverable.clear();
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button(tr("Discard"), {90, 0})) {
            runCommandLine("RECOVER DISCARD ALL");
            m_recoverable.clear();
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button(tr("Later"), {90, 0}) || ImGui::IsKeyPressed(ImGuiKey_Escape)) ImGui::CloseCurrentPopup();
        ImGui::TextDisabled("%s", tr("Restored files open in their own tabs, still unsaved. Later: type RECOVER."));
        ImGui::EndPopup();
    }

    if (m_offerCrashes && !m_offerRecovery && !ImGui::IsPopupOpen("Recover Unsaved Work")) {
        ImGui::OpenPopup("SI CDM Closed Unexpectedly");
        m_offerCrashes = false;
    }
    if (ImGui::BeginPopupModal("SI CDM Closed Unexpectedly", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text(m_newCrashes.size() == 1 ? "SI CDM stopped because of a fault last time. A report was kept on this computer:"
                                             : "SI CDM stopped because of a fault. Reports were kept on this computer:");
        for (const CrashReportFile& f : m_newCrashes) ImGui::BulletText("%s", f.path.c_str());
        ImGui::TextDisabled("%s", tr("Nothing has been sent. To report the problem, attach the report to an issue or an e-mail."));
        ImGui::Spacing();
        if (ImGui::Button(m_newCrashes.size() == 1 ? "Copy the Report" : "Copy the Newest Report", {180, 0})) {
            std::ifstream f(m_newCrashes.front().path, std::ios::binary);
            std::ostringstream text;
            text << f.rdbuf();
            ImGui::SetClipboardText(text.str().c_str());
        }
        ImGui::SameLine();
        if (ImGui::Button(tr("OK"), {90, 0}) || ImGui::IsKeyPressed(ImGuiKey_Escape)) {
            acknowledgeCrashReports();
            m_newCrashes.clear();
            ImGui::CloseCurrentPopup();
        }
        ImGui::TextDisabled("%s", tr("CRASHES lists them; CRASHES CLEAR deletes them."));
        ImGui::EndPopup();
    }

    if (m_closeRequested) {
        ImGui::OpenPopup("Quit SI CDM?");
        m_closeRequested = false;
    }
    if (ImGui::BeginPopupModal("Quit SI CDM?", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        bool canSave = true;
        ImGui::Text("%s", tr("These files have unsaved changes:"));
        for (int i = 0; i < static_cast<int>(m_files.size()); ++i) {
            const Document& d = fileDoc(i);
            if (!d.dirty) continue;
            ImGui::BulletText("%s", d.displayName().c_str());
            canSave = canSave && !d.path.empty();
        }
        ImGui::Spacing();
        if (!canSave) ImGui::BeginDisabled();
        if (ImGui::Button(tr("Save All and Quit"), {150, 0})) {
            saveAll();
            if (!anyFileDirty()) glfwSetWindowShouldClose(m_window, GLFW_TRUE);
            ImGui::CloseCurrentPopup();
        }
        if (!canSave) ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button(tr("Quit Without Saving"), {160, 0})) {
            glfwSetWindowShouldClose(m_window, GLFW_TRUE);
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button(tr("Cancel"), {90, 0}) || ImGui::IsKeyPressed(ImGuiKey_Escape)) ImGui::CloseCurrentPopup();
        if (!canSave) ImGui::TextDisabled("%s", tr("Some files were never saved: use File > Save As on them first."));
        ImGui::EndPopup();
    }
    const bool anim3D = m_cam3D.update(dt);
    const bool anim2D = m_cam2D.update(dt);
    if (anim3D || anim2D) m_framesToRender = std::max(m_framesToRender, 2);
    m_viewMoving = anim3D || anim2D;

    // Sketch preview and crosshair, drawn over the canvas.
    const ImGuiIO& io = ImGui::GetIO();
    // Select: the selected curve's grips, and the one under the cursor lit up.
    if (m_ui.toolbox().active() == ToolId::Select && !simulating() &&
        (m_leftDrag == DragMode::SketchGrip || (!io.WantCaptureMouse && m_leftDrag == DragMode::None))) {
        ImDrawList* fg = ImGui::GetForegroundDrawList();
        const auto& curves = m_doc.scene.sketch().curves();
        const std::optional<SketchGripHit> hot = m_leftDrag == DragMode::SketchGrip ? std::optional<SketchGripHit>(m_sketchDrag)
                                                                                     : pickSketchGrip(io.MousePos.x, io.MousePos.y);
        auto mark = [&](const SketchCurve& c, Vec2 q, bool lit) {
            const auto sp = projectToScreen(camera(), curveFrame(c).toWorld(q), io.DisplaySize);
            if (!sp) return;
            const float h = lit ? 5.0f : 3.5f;
            fg->AddRectFilled({sp->x - h, sp->y - h}, {sp->x + h, sp->y + h}, lit ? IM_COL32(255, 196, 64, 255) : IM_COL32(20, 24, 32, 230));
            fg->AddRect({sp->x - h, sp->y - h}, {sp->x + h, sp->y + h}, lit ? IM_COL32(255, 255, 255, 255) : IM_COL32(120, 200, 255, 255));
        };
        const int sel = m_uiState.selectedCurve;
        if (sel >= 0 && sel < static_cast<int>(curves.size()) && curves[static_cast<std::size_t>(sel)].spatial()) {
            // A 3D curve: its points where they are in space, numbered (PATH3D POINT n).
            const SketchCurve& c = curves[static_cast<std::size_t>(sel)];
            bool closed = false;
            const std::vector<Vec3> pts = spacePoints(c, &closed);
            for (std::size_t k = 0; k < pts.size(); ++k) {
                const auto sp = projectToScreen(camera(), pts[k], io.DisplaySize);
                if (!sp) continue;
                fg->AddCircleFilled({sp->x, sp->y}, 4.0f, IM_COL32(20, 24, 32, 230));
                fg->AddCircle({sp->x, sp->y}, 4.0f, IM_COL32(120, 200, 255, 255));
                fg->AddText({sp->x + 6, sp->y - 16}, IM_COL32(200, 230, 255, 230), std::to_string(k + 1).c_str());
            }
        } else if (sel >= 0 && sel < static_cast<int>(curves.size()))
            for (const auto& [k, q] : gripPoints(curves[static_cast<std::size_t>(sel)])) mark(curves[static_cast<std::size_t>(sel)], q, false);
        if (hot && hot->curve >= 0 && hot->curve < static_cast<int>(curves.size())) {
            const SketchCurve& c = curves[static_cast<std::size_t>(hot->curve)];
            if (hot->grip.ref.point >= 0 && c.kind != CurveKind::Point) {
                for (const auto& [k, q] : gripPoints(c))
                    if (k == hot->grip.ref.point) mark(c, q, true);
            } else if (c.kind == CurveKind::Point)
                mark(c, c.pts[0], true);
            ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll);
        }
    }
    // The 3D Path window's points so far, joined in order.
    if (m_uiState.showPath3d && !m_uiState.pathPoints.empty() && m_uiState.mode != ViewMode::Drawing2D) {
        ImDrawList* fg = ImGui::GetForegroundDrawList();
        std::optional<ImVec2> prev;
        const auto& pts = m_uiState.pathPoints;
        for (std::size_t k = 0; k <= pts.size(); ++k) {
            if (k == pts.size() && !(m_uiState.pathClosed && pts.size() > 2)) break;
            const auto sp = projectToScreen(camera(), pts[k % pts.size()], io.DisplaySize);
            if (sp && prev) fg->AddLine(*prev, *sp, IM_COL32(255, 196, 64, 220), 2.0f);
            if (sp && k < pts.size()) {
                fg->AddCircleFilled(*sp, 4.5f, IM_COL32(255, 196, 64, 255));
                fg->AddText({sp->x + 6, sp->y - 16}, IM_COL32(255, 230, 170, 240), std::to_string(k + 1).c_str());
            }
            prev = sp;
        }
    }
    if (isSketchTool(m_ui.toolbox().active()) && !io.WantCaptureMouse && !simulating()) {
        ImDrawList* fg = ImGui::GetForegroundDrawList();
        m_sketcher.drawOverlay(fg, camera(), io.DisplaySize, m_ui.toolbox().active());
        if (m_uiState.cursorPlane) {
            ImGui::SetMouseCursor(ImGuiMouseCursor_None);
            const ImVec2 m = io.MousePos;
            const ImU32 c = IM_COL32(235, 238, 245, 230);
            fg->AddLine({m.x - 12, m.y}, {m.x - 3, m.y}, c);
            fg->AddLine({m.x + 3, m.y}, {m.x + 12, m.y}, c);
            fg->AddLine({m.x, m.y - 12}, {m.x, m.y - 3}, c);
            fg->AddLine({m.x, m.y + 3}, {m.x, m.y + 12}, c);
        }
    }
    if (m_extrudeCurve >= 0) drawExtrudePreview();
    if (m_uiState.mode == ViewMode::Model3D && (m_doc.state.cam.showPaths || m_doc.state.lathe.showPaths)) drawToolpathOverlay();
    if (m_uiState.mode == ViewMode::Drawing2D && !m_doc.scene.sketch().constraints().empty()) drawConstraintOverlay();
    if (m_uiState.mode == ViewMode::Model3D && (!m_doc.state.datums.empty() || !m_doc.state.refAxes.empty() || !m_doc.state.refPoints.empty()))
        drawDatumOverlay();
    if (m_uiState.mode == ViewMode::Model3D) drawFieldLegend();
    if (m_explodeT > 0 && m_doc.state.assembly.explode.trails) drawExplodeOverlay();
    if (m_leftDrag == DragMode::PushPull) drawPushPullOverlay();
    drawGizmo();
    drawSnapMarker();
    if (!simulating() && (isSculptTool(m_ui.toolbox().active()) || isCutTool(m_ui.toolbox().active()) ||
                          isWearableTool(m_ui.toolbox().active()) || m_leftDrag == DragMode::BoxSelect))
        drawSculptOverlay(); // (and the selection box)
    if (simulating()) {
        m_simTransforms = simulationTransforms(m_doc.scene, m_uiState.simTime); // after scrubbing / panel edits
        updateLoopShapes();
        drawSimulationOverlay();
    }

    // Held widgets (a long press on a tool slot) need frames to time out, and a
    // newly opened popup needs a second frame to size itself before it shows.
    if (ImGui::IsAnyItemActive() || ImGui::IsPopupOpen(nullptr, ImGuiPopupFlags_AnyPopupId))
        m_framesToRender = std::max(m_framesToRender, 2);
    if (!m_uiState.statusMessage.empty() && glfwGetTime() > m_statusUntil) m_uiState.statusMessage.clear();
    m_renderer->sync(m_doc.scene);
    updateSketchColors();

    int fbw = 0, fbh = 0;
    glfwGetFramebufferSize(m_window, &fbw, &fbh);
    if (fbw > 0 && fbh > 0) {
        m_aspect = static_cast<float>(fbw) / static_cast<float>(fbh);
        m_cam3D.setAspect(m_aspect);
        m_cam2D.setAspect(m_aspect);
        {
            // The section view (SECTION) of the 3D window.
            const SectionView& sv = m_doc.state.section;
            ViewportSettings::Section& v = m_vp3D.section;
            v.on = sv.on, v.normal = sv.normal, v.offset = static_cast<float>(sv.offset), v.cap = sv.cap;
            v.cut.clear();
            if (!sv.only.empty())
                for (const Body& b : m_doc.scene.bodies()) v.cut.push_back(sv.cuts(b) ? 1 : 0);
        }
        m_renderer->setAlsoSelected(selectedBodies(m_doc.scene, m_uiState.selectedBody, m_uiState.bodySelection));
        m_renderer->setAlsoSelectedCurves(selectedCurves(m_doc.scene.sketch(), m_uiState.selectedCurve, m_uiState.curveSelection));
        m_renderer->render(m_doc.scene, camera(), viewport(), fbw, fbh, m_uiState.selectedBody, m_uiState.selectedCurve,
                           m_uiState.selectedImage, bodyTransforms(),
                           simulating() && !m_loopOverrides.empty() ? &m_loopOverrides
                           : (m_leftDrag == DragMode::Sculpt || m_leftDrag == DragMode::Grab) && sculptTargetValid()
                               ? &m_sculptOverrides
                                                                       : nullptr);
        if (m_renderer->wantsRefine()) m_framesToRender = std::max(m_framesToRender, 2); // fine again once the view stops
    }

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

// ---------------------------------------------------------------------------
// Command line, MCP and document plumbing

int Application::pickImage(float x, float y, Vec2* planePoint) const {
    const ImGuiIO& io = ImGui::GetIO();
    const Ray ray = camera().screenRay(x, y, io.DisplaySize.x, io.DisplaySize.y);
    int best = -1;
    float bestT = 1e30f;
    const auto& images = m_doc.scene.images();
    for (std::size_t i = 0; i < images.size(); ++i) {
        const ImageRef& img = images[i];
        if (!img.visible) continue;
        const PlaneFrame f = planeFrame(img.plane);
        const float d = dot(ray.dir, f.normal);
        if (std::fabs(d) < 1e-5f) continue;
        const float t = dot(f.origin - ray.origin, f.normal) / d; // planes are two-sided
        const Vec2 q = f.toPlane(ray.at(t));
        if (std::fabs(q.x - img.center.x) <= img.width * 0.5f && std::fabs(q.y - img.center.y) <= img.height() * 0.5f &&
            t < bestT) {
            bestT = t;
            best = static_cast<int>(i);
            if (planePoint) *planePoint = q;
        }
    }
    return best;
}

void Application::log(const std::string& text, int kind) {
    auto& l = m_uiState.commandLog;
    l.push_back({text, static_cast<UiState::LogLine::Kind>(kind)});
    if (l.size() > 500) l.erase(l.begin(), l.begin() + 100);
}

void Application::syncToDoc() {
    m_doc.selectedBody = m_uiState.selectedBody;
    m_doc.selection = m_uiState.bodySelection;
    m_doc.selectedCurve = m_uiState.selectedCurve;
    m_doc.curveSelection = m_uiState.curveSelection;
    m_doc.selectedImage = m_uiState.selectedImage;
    m_doc.state.plane = m_uiState.plane;
    m_doc.snapGrid = m_uiState.snapGrid;
    m_doc.snapPoints = m_uiState.snapPoints;
}

void Application::syncFromDoc() {
    m_uiState.selectedBody = m_doc.selectedBody;
    m_uiState.bodySelection = m_doc.selection;
    m_uiState.selectedCurve = m_doc.selectedCurve;
    m_uiState.curveSelection = m_doc.curveSelection;
    m_uiState.selectedImage = m_doc.selectedImage;
    m_uiState.snapGrid = m_doc.snapGrid;
    m_uiState.snapPoints = m_doc.snapPoints;
    if (m_doc.state.plane != m_uiState.plane ||
        (m_doc.state.plane == SketchPlane::Custom && !sameFrame(m_vp2D.gridFrame, m_doc.state.customFrame)))
        setPlane(m_doc.state.plane);
}

void Application::afterDocumentReplaced() {
    // New or opened document: reframe both windows and forget stale state.
    m_sketcher.cancel();
    cancelExtrude();
    // The stroke belonged to the old document: forget it without touching this one.
    m_sculptBody = -1;
    m_sculptOverrides.clear();
    m_sculptMesh = SculptMesh();
    m_stampBase = SculptMesh();
    m_stamping = false;
    cancelInteraction(false);
    m_cam3D.resetRoll(false);
    m_cam3D.setStandardView(StandardView::Isometric, false);
    if (m_doc.scene.bounds().valid()) m_cam3D.fitToBounds(m_doc.scene.bounds(), false);
    m_2DReady = false;
    m_vp2D.gridPlane = m_doc.state.plane;
    m_vp2D.gridFrame = m_doc.state.customFrame;
    m_uiState.plane = m_doc.state.plane;
    if (m_uiState.mode == ViewMode::Drawing2D) { // the 2D view square to the new plane
        m_uiState.mode = ViewMode::Model3D;
        setMode(ViewMode::Drawing2D);
    }
}

void Application::runCommandLine(const std::string& raw) {
    std::string text = raw;
    while (!text.empty() && (text.back() == ' ' || text.back() == '\t')) text.pop_back();
    const ToolId tool = m_ui.toolbox().active();

    // While a sketch tool is active, coordinates feed the drawing (AutoCAD style).
    if (isSketchTool(tool)) {
        if (text.empty()) {
            if (auto c = m_sketcher.finish()) addCurve(std::move(*c));
            return;
        }
        std::optional<Vec2> p;
        const std::string upper = toUpper(text);
        if (tool == ToolId::Polyline && m_sketcher.stage() >= 2 && (upper == "C" || upper == "CLOSE"))
            p = m_sketcher.firstPoint(); // close the loop, like clicking the first point
        else if (const auto in = parsePointInput(text)) p = in->resolve(m_sketcher.lastPoint().value_or(Vec2{}));
        else if (const auto r = parseLength(text); r && tool == ToolId::Circle && m_sketcher.stage() == 1 && *r > 0)
            p = Vec2{m_sketcher.lastPoint()->x + static_cast<float>(*r), m_sketcher.lastPoint()->y}; // typed radius
        if (p) {
            log(text, UiState::LogLine::Input);
            if (auto c = m_sketcher.typedPoint(tool, m_uiState.plane, activePlaneFrame(m_doc.state), *p)) addCurve(std::move(*c));
            return;
        }
    }
    // Extrude waiting for a depth: a number applies it.
    if (tool == ToolId::Extrude && m_extrudeCurve >= 0) {
        if (text.empty()) {
            applyExtrude();
            return;
        }
        if (const auto d = parseLength(text)) {
            log(text, UiState::LogLine::Input);
            m_uiState.extrudeDistance = static_cast<float>(*d);
            applyExtrude();
            return;
        }
    }
    if (text.empty()) return;

    log(text, UiState::LogLine::Input);
    syncToDoc();
    const CommandOutput out = executeCommand(m_doc, text, this);
    syncFromDoc();
    if (out.clearHistory) m_uiState.commandLog.clear();
    for (const std::string& l : out.lines) log(l, out.ok ? UiState::LogLine::Output : UiState::LogLine::Error);
    if (!out.ok) {
        ++m_commandFailures;
        if (!m_options.screenshotPath.empty()) // scripted runs: say what failed
            std::fprintf(stderr, "Command failed: %s\n  %s\n", text.c_str(), out.lines.empty() ? "" : out.lines.back().c_str());
    }
    if (!out.lines.empty()) setStatus(out.lines.back());
    if (out.documentReplaced) afterDocumentReplaced();
    if (!m_doc.dirty) m_savedSignature = contentSignature(); // SAVE / OPEN / NEW
    m_framesToRender = std::max(m_framesToRender, 3);
}

std::string Application::handleMcp(const std::string& body) {
    syncToDoc();
    std::string reply = m_mcp->handle(body);
    syncFromDoc();
    if (m_mcp->takeDocumentReplaced()) afterDocumentReplaced();
    if (m_mcp->takeDocumentChanged()) log("AI agent updated the model", UiState::LogLine::Output);
    if (!m_doc.dirty) m_savedSignature = contentSignature(); // SAVE / OPEN / NEW, or nothing changed
    return reply;
}

void Application::toggleMcp() {
    if (m_mcpHttp.running()) {
        m_mcpHttp.stop();
        m_uiState.mcpRunning = false;
        log("MCP server stopped", UiState::LogLine::Output);
        return;
    }
    std::string err;
    if (m_mcpHttp.start(m_uiState.mcpPort, [] { glfwPostEmptyEvent(); }, &err)) {
        m_uiState.mcpRunning = true;
        m_uiState.mcpError.clear();
        log("MCP server for AI agents: http://127.0.0.1:" + std::to_string(m_uiState.mcpPort) + "/mcp",
            UiState::LogLine::Output);
    } else {
        m_uiState.mcpRunning = false;
        m_uiState.mcpError = err;
        log(err, UiState::LogLine::Error);
    }
}

std::uint64_t Application::signatureOf(const Document& doc) {
    // FNV-1a over everything a project file stores.
    std::uint64_t h = 1469598103934665603ull;
    auto mix = [&h](const void* data, std::size_t n) {
        const auto* p = static_cast<const unsigned char*>(data);
        for (std::size_t i = 0; i < n; ++i) h = (h ^ p[i]) * 1099511628211ull;
    };
    auto mixStr = [&](const std::string& s) { mix(s.data(), s.size()); };
    const Scene& s = doc.scene;
    const std::size_t counts[3] = {s.bodies().size(), s.sketch().curves().size(), s.images().size()};
    mix(counts, sizeof(counts));
    // Mesh edits in place (sculpting, cuts, undo) can keep every count: the scene's
    // revision changes with each of them.
    const std::uint64_t revision = s.revision();
    mix(&revision, sizeof(revision));
    const UnitSettings& u = doc.state.units;
    const int units[4] = {static_cast<int>(u.unit), u.precision, static_cast<int>(u.style), u.denominator};
    mix(units, sizeof(units));
    for (const Body& b : s.bodies()) {
        mixStr(b.name);
        mixStr(b.kind);
        mix(b.params.data(), b.params.size() * sizeof(float));
        mix(&b.offset, sizeof(b.offset));
        mix(&b.color, sizeof(b.color));
        mix(&b.visible, sizeof(b.visible));
        mix(&b.scale, sizeof(b.scale));
        mix(&b.rotation, sizeof(b.rotation));
        mixStr(b.component);
        const std::size_t tris = b.mesh.indices.size();
        mix(&tris, sizeof(tris));
    }
    for (const SketchCurve& c : s.sketch().curves()) {
        mixStr(c.name);
        mix(c.pts.data(), c.pts.size() * sizeof(Vec2));
        mix(&c.radius, sizeof(c.radius));
        mix(&c.visible, sizeof(c.visible));
        mix(&c.plane, sizeof(c.plane));
        mix(&c.frame, sizeof(c.frame));
    }
    for (const ImageRef& i : s.images()) {
        mixStr(i.name);
        mixStr(i.path);
        mix(&i.center, sizeof(i.center));
        mix(&i.width, sizeof(i.width));
        mix(&i.opacity, sizeof(i.opacity));
        mix(&i.visible, sizeof(i.visible));
    }
    for (const Motion& m : s.motions()) {
        mixStr(m.name);
        for (const std::string& b : m.bodies) mixStr(b);
        mixStr(m.follow);
        const float f[] = {m.axis.x, m.axis.y, m.axis.z, m.origin.x, m.origin.y, m.origin.z, m.speed, m.range, m.start,
                           m.eachCentre ? 1.0f : 0.0f, m.radius, m.ratio, m.step};
        mix(f, sizeof(f));
        const int e[] = {static_cast<int>(m.kind), static_cast<int>(m.mode), m.enabled ? 1 : 0, m.alongLoop ? 1 : 0};
        mix(e, sizeof(e));
    }
    mix(&doc.state.plane, sizeof(doc.state.plane));
    mix(&doc.state.customFrame, sizeof(doc.state.customFrame));
    for (const DatumPlane& d : doc.state.datums) {
        mixStr(d.name);
        mix(&d.frame, sizeof(d.frame));
    }
    mix(&doc.state.simDuration, sizeof(doc.state.simDuration));
    return h;
}

void Application::updateTitle() {
    m_uiState.projectPath = m_doc.path;
    m_uiState.projectTitle = m_doc.title();
    m_uiState.projectName = m_doc.path.empty() ? m_doc.name : std::filesystem::path(m_doc.path).stem().string();
    const std::string title = m_uiState.projectTitle + "  —  SI CDM";
    if (title != m_shownTitle) {
        glfwSetWindowTitle(m_window, title.c_str());
        m_shownTitle = title;
    }
}

void Application::updatePrompt() {
    const ToolId tool = m_ui.toolbox().active();
    const int stage = m_sketcher.stage();
    std::string p = tr("Command:");
    switch (tool) {
    case ToolId::Point: p = "POINT  location (x,y):"; break;
    case ToolId::Line: p = stage == 0 ? "LINE  first point (x,y):" : "LINE  end point (x,y | @dx,dy | @len<deg):"; break;
    case ToolId::Polyline:
        p = stage == 0 ? "PLINE  first point (x,y):" : "PLINE  next point, or Enter to finish:";
        break;
    case ToolId::Rectangle: p = stage == 0 ? "RECT  first corner (x,y):" : "RECT  opposite corner (x,y | @w,h):"; break;
    case ToolId::Circle: p = stage == 0 ? "CIRCLE  centre (x,y):" : "CIRCLE  radius or point:"; break;
    case ToolId::Extrude:
        p = m_extrudeCurve < 0 ? "EXTRUDE  pick a closed profile:"
                               : std::string("EXTRUDE  depth (") + unitInfo(activeUnits().unit).symbol + "), drag or type; click / Enter:";
        break;
    default: break;
    }
    m_uiState.commandPrompt = p;
    m_uiState.commandKeepsFocus = isSketchTool(tool) || (tool == ToolId::Extrude && m_extrudeCurve >= 0);
}

// Extrude ---------------------------------------------------------------------

namespace {

// Representative point and outline of a profile, in world space at depth w.
Vec2 profileCentre(const Profile& p) {
    if (p.kind == Profile::Kind::Circle) return p.center;
    Vec2 c{};
    for (const Vec2& q : p.points) c = {c.x + q.x, c.y + q.y};
    return {c.x / static_cast<float>(p.points.size()), c.y / static_cast<float>(p.points.size())};
}

std::vector<Vec2> profileOutline(const Profile& p) {
    if (p.kind != Profile::Kind::Circle) return p.points;
    std::vector<Vec2> pts;
    for (int i = 0; i < 64; ++i) {
        const float a = 2.0f * kPi * static_cast<float>(i) / 64.0f;
        pts.push_back({p.center.x + p.radius * std::cos(a), p.center.y + p.radius * std::sin(a)});
    }
    return pts;
}

} // namespace

int Application::pickProfile(float x, float y) const {
    if (const int c = pickCurve(x, y); c >= 0) return c; // on the outline
    // Inside a closed profile: the smallest one containing the cursor wins (holes before outlines).
    const ImGuiIO& io = ImGui::GetIO();
    const Ray ray = camera().screenRay(x, y, io.DisplaySize.x, io.DisplaySize.y);
    const auto& curves = m_doc.scene.sketch().curves();
    int best = -1;
    float bestArea = 1e30f;
    for (std::size_t i = 0; i < curves.size(); ++i) {
        if (!curves[i].visible) continue;
        const auto p = profileFromSketch(m_doc.scene.sketch(), i);
        if (!p) continue;
        const PlaneFrame f = curveFrame(curves[i]);
        const float d = dot(ray.dir, f.normal);
        if (std::fabs(d) < 1e-5f) continue;
        const Vec2 q = f.toPlane(ray.at(dot(f.origin - ray.origin, f.normal) / d));
        bool inside = false;
        float area = 0.0f;
        if (p->kind == Profile::Kind::Circle) {
            inside = std::hypot(q.x - p->center.x, q.y - p->center.y) <= p->radius;
            area = kPi * p->radius * p->radius;
        } else {
            const auto& v = p->points;
            for (std::size_t a = 0, b = v.size() - 1; a < v.size(); b = a++) // even-odd rule
                if ((v[a].y > q.y) != (v[b].y > q.y) &&
                    q.x < (v[b].x - v[a].x) * (q.y - v[a].y) / (v[b].y - v[a].y) + v[a].x)
                    inside = !inside;
            area = std::fabs(signedArea(v));
        }
        if (inside && area < bestArea) {
            bestArea = area;
            best = static_cast<int>(i);
        }
    }
    return best;
}

void Application::beginExtrude(int curve) {
    const SketchCurve& c = m_doc.scene.sketch().curves()[static_cast<std::size_t>(curve)];
    std::string why;
    if (!profileFromSketch(m_doc.scene.sketch(), static_cast<std::size_t>(curve), &why)) {
        setStatus("Cannot extrude: " + why);
        log("Cannot extrude: " + why, UiState::LogLine::Error);
        return;
    }
    m_extrudeCurve = curve;
    m_uiState.selectedCurve = curve;
    m_uiState.selectedBody = m_uiState.selectedImage = -1;
    m_uiState.extrudeProfileName = c.name;
    m_uiState.extrudeLive = true;
    const ImGuiIO& io = ImGui::GetIO();
    m_extrudeAnchorX = io.MousePos.x;
    m_extrudeAnchorY = io.MousePos.y;
    m_extrudeAnchorDist = m_uiState.extrudeDistance;
    setStatus("Extruding " + c.name + ": drag or type the depth, click or Enter to finish");
}

void Application::updateExtrudeFromMouse() {
    if (!m_uiState.extrudeLive) return;
    const auto& curves = m_doc.scene.sketch().curves();
    const SketchCurve& c = curves[static_cast<std::size_t>(m_extrudeCurve)];
    const auto profile = profileFromSketch(m_doc.scene.sketch(), static_cast<std::size_t>(m_extrudeCurve));
    if (!profile) return;
    const ImGuiIO& io = ImGui::GetIO();
    const PlaneFrame f = curveFrame(c);
    const Vec3 base = f.toWorld(profileCentre(*profile));
    // Screen direction of the plane normal; if it points at the viewer (2D window),
    // vertical mouse motion sets the depth instead.
    const float probe = std::max(camera().distance() * 0.1f, 1.0f);
    const auto s0 = projectToScreen(camera(), base, io.DisplaySize);
    const auto s1 = projectToScreen(camera(), base + f.normal * probe, io.DisplaySize);
    const float mx = io.MousePos.x - m_extrudeAnchorX, my = io.MousePos.y - m_extrudeAnchorY;
    float d = m_extrudeAnchorDist;
    if (s0 && s1 && std::hypot(s1->x - s0->x, s1->y - s0->y) > 12.0f) {
        const float dx = s1->x - s0->x, dy = s1->y - s0->y, len = std::hypot(dx, dy);
        d += (mx * dx + my * dy) / len * (probe / len);
    } else {
        d += -my * camera().worldPerPixel(io.DisplaySize.y);
    }
    m_uiState.extrudeDistance = std::round(d * 2.0f) * 0.5f; // 0.5 mm steps
}

void Application::applyExtrude() {
    if (m_extrudeCurve < 0) return;
    const auto& curves = m_doc.scene.sketch().curves();
    if (m_extrudeCurve >= static_cast<int>(curves.size())) {
        cancelExtrude();
        return;
    }
    if (m_uiState.extrudeDistance == 0.0f) {
        setStatus("Depth must not be zero");
        return;
    }
    char depth[32];
    std::snprintf(depth, sizeof(depth), "%.9gmm", static_cast<double>(m_uiState.extrudeDistance)); // mm, whatever the units
    std::string line = "EXTRUDE \"" + curves[static_cast<std::size_t>(m_extrudeCurve)].name + "\" " + depth;
    if (m_uiState.extrudeSymmetric) line += " SYMMETRIC";
    if (m_uiState.extrudeKeepSketch) line += " KEEP";
    m_uiState.selectedCurve = m_extrudeCurve;
    cancelExtrude();
    runCommandLine(line);
}

void Application::cancelExtrude() {
    m_extrudeCurve = -1;
    m_uiState.extrudeProfileName.clear();
}

void Application::drawExtrudePreview() const {
    const auto& curves = m_doc.scene.sketch().curves();
    if (m_extrudeCurve < 0 || m_extrudeCurve >= static_cast<int>(curves.size())) return;
    const SketchCurve& c = curves[static_cast<std::size_t>(m_extrudeCurve)];
    const auto profile = profileFromSketch(m_doc.scene.sketch(), static_cast<std::size_t>(m_extrudeCurve));
    if (!profile) return;
    const PlaneFrame f = curveFrame(c);
    const float d = m_uiState.extrudeDistance;
    const float w0 = m_uiState.extrudeSymmetric ? -std::fabs(d) * 0.5f : std::min(0.0f, d);
    const float w1 = m_uiState.extrudeSymmetric ? std::fabs(d) * 0.5f : std::max(0.0f, d);
    const ImGuiIO& io = ImGui::GetIO();
    ImDrawList* dl = ImGui::GetForegroundDrawList();
    const ImU32 col = IM_COL32(255, 190, 90, 230), faint = IM_COL32(255, 190, 90, 120);
    auto scr = [&](Vec2 q, float w) { return projectToScreen(camera(), f.toWorld(q) + f.normal * w, io.DisplaySize); };
    const std::vector<Vec2> outline = profileOutline(*profile);
    const std::size_t n = outline.size();
    for (float w : {w0, w1})
        for (std::size_t i = 0; i < n; ++i) {
            const auto a = scr(outline[i], w), b = scr(outline[(i + 1) % n], w);
            if (a && b) dl->AddLine(*a, *b, w == w1 ? col : faint, 2.0f);
        }
    const std::size_t step = std::max<std::size_t>(1, n / 8);
    for (std::size_t i = 0; i < n; i += step) {
        const auto a = scr(outline[i], w0), b = scr(outline[i], w1);
        if (a && b) dl->AddLine(*a, *b, faint, 1.5f);
    }
    // Depth arrow from the profile centre, with the value.
    const Vec2 ctr = profileCentre(*profile);
    const auto a = scr(ctr, 0.0f), b = scr(ctr, m_uiState.extrudeSymmetric ? w1 : d);
    if (a && b) {
        dl->AddLine(*a, *b, col, 2.0f);
        dl->AddCircleFilled(*b, 4.0f, col);
    }
    char label[96];
    std::snprintf(label, sizeof(label), "D %s%s", lengthText(d).c_str(), m_uiState.extrudeSymmetric ? " (symmetric)" : "");
    const ImVec2 at{io.MousePos.x + 16, io.MousePos.y + 14};
    const ImVec2 ts = ImGui::CalcTextSize(label);
    dl->AddRectFilled({at.x - 4, at.y - 2}, {at.x + ts.x + 4, at.y + ts.y + 2}, IM_COL32(20, 22, 28, 220), 3.0f);
    dl->AddText(at, IM_COL32(240, 240, 240, 255), label);
}

// File tabs -------------------------------------------------------------------

void Application::parkActiveFile() {
    syncToDoc();
    FileTab& t = m_files[static_cast<std::size_t>(m_activeFile)];
    t.doc = std::move(m_doc);
    m_doc = Document{};
    t.cam3D = m_cam3D;
    t.cam2D = m_cam2D;
    t.vp3D = m_vp3D;
    t.vp2D = m_vp2D;
    t.ready2D = m_2DReady;
    t.savedSignature = m_savedSignature;
    t.mode = m_uiState.mode;
    t.simTime = m_uiState.simTime;
    m_uiState.simPlaying = false;
}

void Application::loadFile(int index) {
    FileTab& t = m_files[static_cast<std::size_t>(index)];
    m_activeFile = index;
    m_doc = std::move(t.doc);
    t.doc = Document{};
    m_cam3D = t.cam3D;
    m_cam2D = t.cam2D;
    m_vp3D = t.vp3D;
    m_vp2D = t.vp2D;
    m_2DReady = t.ready2D;
    m_savedSignature = t.savedSignature;
    m_uiState.mode = t.mode;
    m_uiState.simTime = t.simTime;
    m_uiState.plane = m_doc.state.plane;
    m_uiState.selectedBody = m_doc.selectedBody;
    m_uiState.bodySelection = m_doc.selection;
    m_uiState.selectedCurve = m_doc.selectedCurve;
    m_uiState.curveSelection = m_doc.curveSelection;
    m_uiState.selectedImage = m_doc.selectedImage;
    m_uiState.snapGrid = m_doc.snapGrid;
    m_uiState.snapPoints = m_doc.snapPoints;
}

void Application::activateFile(int index) {
    if (index == m_activeFile || index < 0 || index >= static_cast<int>(m_files.size())) return;
    m_sketcher.cancel();
    cancelExtrude();
    cancelInteraction(true); // keep the stroke in the file it was made in
    parkActiveFile();
    loadFile(index);
    updateTitle();
    setStatus(m_doc.displayName() + (m_uiState.mode == ViewMode::Drawing2D   ? "  -  2D Drawing"
                                     : m_uiState.mode == ViewMode::Simulation ? "  -  Simulation"
                                                                              : "  -  3D Model"));
    m_framesToRender = std::max(m_framesToRender, 3);
}

void Application::newFileTab() {
    // A fresh untitled file: same window and display settings, framed on the origin.
    m_sketcher.cancel();
    cancelExtrude();
    cancelInteraction(true); // keep the stroke in the file it was made in
    const ViewportSettings vp3D = m_vp3D, vp2D = m_vp2D;
    const ViewMode mode = m_uiState.mode;
    parkActiveFile();
    FileTab t;
    t.id = m_nextFileId++;
    t.vp3D = vp3D;
    t.vp2D = vp2D;
    t.vp2D.gridPlane = SketchPlane::Top;
    t.mode = mode;
    t.doc.name = "Untitled " + std::to_string(++m_untitledCount);
    t.cam3D.setAspect(m_aspect);
    t.cam3D.setStandardView(StandardView::Isometric, false);
    AABB box;
    box.expand(Vec3{-60, -60, 0});
    box.expand(Vec3{60, 60, 40});
    t.cam3D.fitToBounds(box, false);
    t.savedSignature = signatureOf(t.doc);
    m_files.push_back(std::move(t));
    loadFile(static_cast<int>(m_files.size()) - 1);
    if (mode == ViewMode::Drawing2D) {
        init2DCamera();
        AABB plane;
        plane.expand(Vec3{-60, -60, -60});
        plane.expand(Vec3{60, 60, 60});
        m_cam2D.fitToBounds(plane, false);
    }
    updateTitle();
}

bool Application::closeFileTab(int index, bool discard) {
    if (index < 0 || index >= static_cast<int>(m_files.size())) return false;
    if (!discard && fileDoc(index).dirty) return false;
    const std::string name = fileDoc(index).displayName();
    if (m_files.size() == 1) newFileTab(); // never leave the canvas without a file
    if (index == m_activeFile) activateFile(index + 1 < static_cast<int>(m_files.size()) ? index + 1 : index - 1);
    m_autosaver.forget(m_files[static_cast<std::size_t>(index)].id);
    m_files.erase(m_files.begin() + index);
    if (m_activeFile > index) --m_activeFile;
    updateTitle();
    log("Closed " + name, UiState::LogLine::Output);
    setStatus("Closed " + name);
    m_framesToRender = std::max(m_framesToRender, 3);
    return true;
}

void Application::watchComponents() {
    // Every two seconds, between strokes and drags: a part file saved since (in another tab,
    // by a colleague, from the vault) comes in. Each change is tried once (a released
    // assembly refuses it, and says so once).
    const double now = glfwGetTime();
    if (now < m_componentCheck || m_leftDrag != DragMode::None) return;
    m_componentCheck = now + 2.0;
    std::string key;
    for (const ComponentInfo& c : listComponents(m_doc.scene))
        if (c.changed) key += c.name + "|" + c.file + "|" + std::to_string(fileStamp(c.file)) + ";";
    if (key.empty() || key == m_componentTried) return;
    m_componentTried = key;
    runCommandLine("COMPONENT RELOAD");
    m_framesToRender = std::max(m_framesToRender, 3);
}

void Application::autosave() {
    const double now = glfwGetTime();
    if (now < m_autosaveCheck) return;
    m_autosaveCheck = now + 1.0;
    // Preferences: the interval, or off (this session's copies go).
    const int seconds = preferences().autosaveSeconds;
    if (seconds <= 0) {
        if (m_autosaveOn) m_autosaver.clear();
        m_autosaveOn = false;
        return;
    }
    m_autosaveOn = true;
    if (m_autosaver.interval() != seconds) m_autosaver.setInterval(seconds);
    for (int i = 0; i < static_cast<int>(m_files.size()); ++i) {
        const Document& d = fileDoc(i);
        std::string err;
        const std::uint64_t sig = i == m_activeFile ? contentSignature() : signatureOf(d);
        if (!m_autosaver.update(d, m_files[static_cast<std::size_t>(i)].id, sig, now, &err) && !err.empty())
            log("Autosave of " + d.displayName() + " failed: " + err, UiState::LogLine::Error);
    }
}

int Application::fileIndex(int id) const {
    for (std::size_t i = 0; i < m_files.size(); ++i)
        if (m_files[i].id == id) return static_cast<int>(i);
    return -1;
}

const Document& Application::fileDoc(int index) const {
    return index == m_activeFile ? m_doc : m_files[static_cast<std::size_t>(index)].doc;
}

bool Application::anyFileDirty() const {
    for (int i = 0; i < static_cast<int>(m_files.size()); ++i)
        if (fileDoc(i).dirty) return true;
    return false;
}

void Application::refreshFileTabs() {
    auto& files = m_uiState.files;
    files.resize(m_files.size());
    for (std::size_t i = 0; i < m_files.size(); ++i) {
        const Document& d = fileDoc(static_cast<int>(i));
        files[i].id = m_files[i].id;
        files[i].name = d.displayName();
        files[i].path = d.path;
        files[i].dirty = d.dirty;
        files[i].mode = static_cast<int>(i) == m_activeFile ? m_uiState.mode : m_files[i].mode;
    }
    m_uiState.activeFile = m_activeFile;
}

void Application::saveAll() {
    const int back = m_activeFile;
    for (int i = 0; i < static_cast<int>(m_files.size()); ++i) {
        if (!fileDoc(i).dirty || fileDoc(i).path.empty()) continue;
        activateFile(i);
        runCommandLine("SAVE");
        m_doc.dirty = contentSignature() != m_savedSignature;
    }
    activateFile(back);
}

bool Application::prepareDocument(const std::string& path) {
    if (!path.empty()) {
        // Already open: show that tab instead of opening a second copy.
        const std::filesystem::path want = std::filesystem::path(path).lexically_normal();
        for (int i = 0; i < static_cast<int>(m_files.size()); ++i) {
            const std::string& p = fileDoc(i).path;
            if (!p.empty() && std::filesystem::path(p).lexically_normal() == want) {
                activateFile(i);
                return true;
            }
        }
        // An empty (or the demo) untouched untitled file is replaced rather than kept beside it.
        const Scene& s = m_doc.scene;
        const bool blank = s.bodies().empty() && s.sketch().curves().empty() && s.images().empty();
        if (m_doc.path.empty() && !m_doc.dirty && (blank || m_files[static_cast<std::size_t>(m_activeFile)].demo)) {
            m_files[static_cast<std::size_t>(m_activeFile)].demo = false;
            return false;
        }
    }
    newFileTab();
    return false;
}

std::vector<std::string> Application::fileTabs() const {
    std::vector<std::string> out;
    for (int i = 0; i < static_cast<int>(m_files.size()); ++i) {
        const Document& d = fileDoc(i);
        out.push_back((i == m_activeFile ? "> " : "  ") + std::to_string(i + 1) + "  " + d.title() +
                      (d.path.empty() ? std::string() : "   " + d.path));
    }
    return out;
}

bool Application::switchFile(const std::string& which, std::string* error) {
    int index = -1;
    if (const auto n = parseNumber(which); n && *n >= 1 && *n <= static_cast<float>(m_files.size()) &&
                                           std::floor(*n) == *n)
        index = static_cast<int>(*n) - 1;
    const std::string want = toUpper(which);
    for (int i = 0; i < static_cast<int>(m_files.size()) && index < 0; ++i) {
        const std::string name = toUpper(fileDoc(i).displayName());
        if (name == want || std::filesystem::path(name).stem().string() == want) index = i;
    }
    if (index < 0) {
        *error = "No open file " + which + " (FILES lists them)";
        return false;
    }
    syncToDoc();
    activateFile(index);
    syncToDoc(); // the command engine re-reads the selection from the activated file
    return true;
}

bool Application::closeFile(bool discard, std::string* error) {
    if (!discard && m_doc.dirty) {
        *error = m_doc.displayName() + " has unsaved changes: SAVE it first, or CLOSE DISCARD";
        return false;
    }
    syncToDoc();
    closeFileTab(m_activeFile, true);
    syncToDoc();
    return true;
}

// ViewHost --------------------------------------------------------------------

void Application::openWindow(bool drawing2D) { setMode(drawing2D ? ViewMode::Drawing2D : ViewMode::Model3D); }

// Simulation ------------------------------------------------------------------

const std::vector<Mat4>* Application::bodyTransforms() const {
    if (simulating()) return m_simTransforms.empty() ? nullptr : &m_simTransforms;
    return m_explodeTransforms.empty() ? nullptr : &m_explodeTransforms;
}

void Application::advanceExplode(float dt) {
    const ExplodeState& ex = m_doc.state.assembly.explode;
    const bool want = ex.shown && !ex.steps.empty() && m_uiState.mode == ViewMode::Model3D && !isSculptTool(m_ui.toolbox().active());
    const float target = want ? 1.0f : 0.0f;
    if (ex.steps.empty()) m_explodeT = 0.0f;
    if (m_explodeT != target) {
        // The steps one after another: about a third of a second each, a second and a half at most.
        const float span = std::clamp(0.3f * static_cast<float>(ex.steps.size()), 0.4f, 1.5f);
        const float step = std::min(dt, 1.0f / 30.0f) / span;
        m_explodeT = target > m_explodeT ? std::min(target, m_explodeT + step) : std::max(target, m_explodeT - step);
        m_framesToRender = std::max(m_framesToRender, 2);
    }
    m_explodeMoving = m_explodeT != target;
    m_explodeTransforms.clear();
    m_explodeShifts.clear();
    if (m_explodeT <= 0.0f) return;
    // Eased: parts start and stop gently.
    const double t = static_cast<double>(m_explodeT);
    m_explodeShifts = explodeShifts(m_doc.scene, ex, t * t * (3 - 2 * t));
    m_explodeTransforms.reserve(m_explodeShifts.size());
    for (const DVec3& v : m_explodeShifts) m_explodeTransforms.push_back(translate(v.f()));
}

// Trail lines: from where each part sits to where it is drawn, step by step.
void Application::drawExplodeOverlay() const {
    const ImGuiIO& io = ImGui::GetIO();
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    dl->PushClipRect(m_ui.canvasMin(), m_ui.canvasMax(), true);
    const auto& bodies = m_doc.scene.bodies();
    const double t = static_cast<double>(m_explodeT), te = t * t * (3 - 2 * t);
    const ImU32 col = IM_COL32(150, 165, 190, 170);
    for (const ExplodeTrail& tr : explodeTrails(m_doc.scene, m_doc.state.assembly.explode, te, [&](std::size_t i) { return bodies[i].visible; })) {
        const auto a = projectToScreen(camera(), tr.from.f(), io.DisplaySize);
        const auto b = projectToScreen(camera(), tr.to.f(), io.DisplaySize);
        if (!a || !b) continue;
        const float len = std::hypot(b->x - a->x, b->y - a->y);
        const int dashes = std::max(1, static_cast<int>(len / 8.0f));
        for (int d = 0; d < dashes; d += 2) {
            const float t0 = static_cast<float>(d) / static_cast<float>(dashes), t1 = static_cast<float>(d + 1) / static_cast<float>(dashes);
            dl->AddLine(ImVec2(a->x + (b->x - a->x) * t0, a->y + (b->y - a->y) * t0), ImVec2(a->x + (b->x - a->x) * t1, a->y + (b->y - a->y) * t1), col, 1.2f);
        }
    }
    dl->PopClipRect();
}

// Push / pull ------------------------------------------------------------------

bool Application::beginPushPull(float x, float y) {
    const ImGuiIO& io = ImGui::GetIO();
    const auto hit = pick(x, y);
    if (!hit) {
        setStatus("Press on a face of a body and drag");
        return false;
    }
    const Body& body = m_doc.scene.bodies()[static_cast<std::size_t>(hit->body)];
    DirectFaceInfo f;
    std::string why;
    if (!directFaceAt(body, hit->point, f, &why)) {
        setStatus(why);
        return false;
    }
    PushPullDrag p;
    p.body = hit->body;
    p.at = hit->point;
    const DVec3 at(hit->point);
    switch (f.kind) {
    case DirectFaceKind::Plane: p.dir = f.normal.f(), p.what = "flat face"; break;
    case DirectFaceKind::Cylinder:
    case DirectFaceKind::Cone: {
        const DVec3 q = at - f.point;
        const DVec3 radial = dnormalize(q - f.axis * ddot(q, f.axis));
        p.dir = (radial * (f.hole ? -1.0 : 1.0)).f();
        p.what = f.hole ? "hole" : "round face";
        break;
    }
    case DirectFaceKind::Sphere: p.dir = dnormalize(at - f.point).f(), p.what = "round face"; break;
    default: setStatus("Only flat faces, and round faces of exact solids, can be pushed or pulled"); return false;
    }
    // The normal on screen, and how many pixels a millimetre along it is.
    const auto s0 = projectToScreen(camera(), p.at, io.DisplaySize), s1 = projectToScreen(camera(), p.at + p.dir * 10.0f, io.DisplaySize);
    if (!s0 || !s1) return false;
    float dx = s1->x - s0->x, dy = s1->y - s0->y;
    float len = std::hypot(dx, dy);
    if (len < 4.0f) { // looking straight along it: drag up to pull
        const auto s2 = projectToScreen(camera(), p.at + camera().up() * 10.0f, io.DisplaySize);
        dx = 0, dy = -1;
        len = s2 ? std::max(4.0f, std::hypot(s2->x - s0->x, s2->y - s0->y)) : 40.0f;
        p.screenDir = {dx, dy};
    } else {
        p.screenDir = {dx / len, dy / len};
    }
    p.pxPerMm = len / 10.0f;
    p.start = {x, y};
    m_push = p;
    setStatus("Drag along the " + p.what + "'s normal (Shift: whole millimetres)");
    return true;
}

void Application::updatePushPull(float x, float y) {
    const float along = (x - m_push.start.x) * m_push.screenDir.x + (y - m_push.start.y) * m_push.screenDir.y;
    float d = along / std::max(m_push.pxPerMm, 1e-3f);
    d = ImGui::GetIO().KeyShift ? std::round(d) : std::round(d * 10.0f) / 10.0f;
    m_push.distance = d;
    m_framesToRender = std::max(m_framesToRender, 2);
}

void Application::finishPushPull() {
    if (m_push.body < 0 || m_push.body >= static_cast<int>(m_doc.scene.bodies().size()) || std::fabs(m_push.distance) < 0.05f) {
        setStatus("Drag further to push or pull the face");
        m_push = PushPullDrag{};
        return;
    }
    runCommandLine("FACE OFFSET " + quotedName(m_doc.scene.bodies()[static_cast<std::size_t>(m_push.body)].name) + " AT " + mmVec(m_push.at) + " DISTANCE " +
                   mmText(m_push.distance));
    m_push = PushPullDrag{};
}

// The arrow from where the face was pressed to where it goes, and the distance.
bool Application::gizmoShown(Vec3& pivot, float& size) const {
    const int sel = m_leftDrag == DragMode::Gizmo ? m_gizmoBody : m_uiState.selectedBody;
    if (sel < 0 || sel >= static_cast<int>(m_doc.scene.bodies().size())) return false;
    const Body& b = m_doc.scene.bodies()[static_cast<std::size_t>(sel)];
    if (!b.visible) return false;
    if (m_leftDrag == DragMode::Gizmo) { // it stays where the drag began
        pivot = m_gizmoPivot, size = m_gizmoSize;
        return true;
    }
    AABB box; // the middle of every selected body (they move together)
    for (int i : selectedBodies(m_doc.scene, sel, m_uiState.bodySelection)) box.expand(m_doc.scene.bodies()[static_cast<std::size_t>(i)].bounds());
    if (!box.valid()) return false;
    pivot = box.center();
    // About 110 pixels long whatever the zoom.
    const ImGuiIO& io = ImGui::GetIO();
    const auto p = projectToScreen(camera(), pivot, io.DisplaySize), q = projectToScreen(camera(), pivot + camera().right(), io.DisplaySize);
    if (!p || !q) return false;
    const float perMm = std::hypot(q->x - p->x, q->y - p->y);
    if (!(perMm > 1e-6f)) return false;
    size = 110.0f / perMm;
    return true;
}

GizmoProject Application::gizmoProjector() const {
    const ImVec2 view = ImGui::GetIO().DisplaySize;
    const OrbitCamera* cam = &camera();
    return [cam, view](const Vec3& p) -> std::optional<Vec2> {
        const auto s = projectToScreen(*cam, p, view);
        if (!s) return std::nullopt;
        return Vec2{s->x, s->y};
    };
}

void Application::startSnap(const std::vector<int>& moving) {
    // What moves: the points of the bodies dragged; what it can land on: every other visible
    // body's (a few thousand of each at most, so a frame stays quick).
    m_snapMoving.clear();
    m_snapTargets.clear();
    m_snapHit.reset();
    m_snapMovingBodies = moving;
    m_snapSeen.clear();
    const auto& bodies = m_doc.scene.bodies();
    for (int b : moving)
        if (b >= 0 && b < static_cast<int>(bodies.size())) {
            const std::vector<Vec3> p = snapPoints(bodies[static_cast<std::size_t>(b)], 4000 / std::max<std::size_t>(1, moving.size()));
            m_snapMoving.insert(m_snapMoving.end(), p.begin(), p.end());
        }
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        if (!bodies[i].visible || std::find(moving.begin(), moving.end(), static_cast<int>(i)) != moving.end()) continue;
        const std::vector<Vec3> p = snapPoints(bodies[i], 20000);
        m_snapTargets.insert(m_snapTargets.end(), p.begin(), p.end());
        if (m_snapTargets.size() > 200000) break;
    }
}

std::optional<GizmoSnapHit> Application::freeSnap(const Vec3& move) {
    if (m_snapSeen.size() != m_snapTargets.size()) m_snapSeen.assign(m_snapTargets.size(), -1);
    const GizmoProject project = gizmoProjector();
    // In sight: nothing but what is being moved between the camera and the point (asked once
    // per point a drag: the camera and the other bodies hold still meanwhile).
    auto seen = [&](std::size_t i) {
        if (m_snapSeen[i] < 0) {
            bool visible = false;
            if (const auto at = project(m_snapTargets[i])) {
                const ImGuiIO& io = ImGui::GetIO();
                const Ray ray = camera().screenRay(at->x, at->y, io.DisplaySize.x, io.DisplaySize.y);
                const auto hit = pick(at->x, at->y);
                const float want = length(m_snapTargets[i] - ray.origin);
                visible = !hit || std::find(m_snapMovingBodies.begin(), m_snapMovingBodies.end(), hit->body) != m_snapMovingBodies.end() ||
                          length(hit->point - ray.origin) >= want - std::max(0.01f, want * 1e-4f);
            }
            m_snapSeen[i] = visible ? 1 : 0;
        }
        return m_snapSeen[i] == 1;
    };
    if (auto hit = gizmoSnap(m_snapMoving, m_snapTargets, move, project, 12.0f, {}, {}, seen)) return hit;
    return gizmoSnap(m_snapMoving, m_snapTargets, move, project, 12.0f, {}, camera().forward(), [&](std::size_t i) { return !seen(i); });
}

void Application::drawSnapMarker() {
    if (!m_snapHit || (m_leftDrag != DragMode::Gizmo && m_leftDrag != DragMode::MoveBody)) return;
    const auto at = gizmoProjector()(m_snapHit->to);
    if (!at) return;
    ImDrawList* fg = ImGui::GetForegroundDrawList();
    const ImU32 col = IM_COL32(80, 230, 255, 255);
    fg->AddRect({at->x - 6, at->y - 6}, {at->x + 6, at->y + 6}, col, 0.0f, 0, 2.0f);
    fg->AddCircleFilled({at->x, at->y}, 2.5f, col);
    fg->AddText({at->x + 9, at->y - 18}, col, tr("snap"));
}

void Application::drawGizmo() {
    if (m_ui.toolbox().active() != ToolId::Move || m_uiState.mode == ViewMode::Drawing2D || simulating()) return;
    Vec3 pivot{};
    float size = 0;
    if (!gizmoShown(pivot, size)) return;
    const GizmoMode mode = static_cast<GizmoMode>(std::clamp(m_uiState.gizmoMode, 0, 2));
    ImDrawList* fg = ImGui::GetForegroundDrawList();
    static const ImU32 kAxisColour[3] = {IM_COL32(230, 80, 80, 255), IM_COL32(110, 200, 90, 255), IM_COL32(80, 140, 240, 255)};
    const GizmoHandle hot = m_leftDrag == DragMode::Gizmo ? m_gizmoHandle : m_gizmoHover;
    for (const GizmoStroke& s : gizmoStrokes(mode, pivot, size, camera().forward(), gizmoProjector())) {
        if (s.points.size() < 2) continue;
        ImU32 col = s.handle.kind == GizmoHandle::Center ? IM_COL32(235, 235, 235, 230)
                    : s.handle.kind == GizmoHandle::ViewRing ? IM_COL32(200, 200, 200, 200)
                                                              : kAxisColour[std::clamp(s.handle.axis, 0, 2)];
        if (hot.kind != GizmoHandle::None && s.handle == hot) col = IM_COL32(255, 214, 60, 255);
        std::vector<ImVec2> pts;
        for (const Vec2& p : s.points) pts.push_back({p.x, p.y});
        if (s.fill) {
            const ImU32 fill = s.handle.kind == GizmoHandle::Plane ? (col & 0x00FFFFFF) | 0x80000000 : col;
            fg->AddConvexPolyFilled(pts.data(), static_cast<int>(pts.size()), fill);
        } else {
            fg->AddPolyline(pts.data(), static_cast<int>(pts.size()), col, s.closed ? ImDrawFlags_Closed : ImDrawFlags_None, s.handle == hot ? 3.5f : 2.2f);
        }
    }
    if (m_leftDrag != DragMode::Gizmo) return;
    // What the drag does so far, by the mouse.
    char label[96];
    if (mode == GizmoMode::Move)
        std::snprintf(label, sizeof label, "%+.2f, %+.2f, %+.2f mm", static_cast<double>(m_gizmoDelta.move.x), static_cast<double>(m_gizmoDelta.move.y),
                      static_cast<double>(m_gizmoDelta.move.z));
    else if (mode == GizmoMode::Rotate)
        std::snprintf(label, sizeof label, "%+.1f deg", static_cast<double>(m_gizmoDelta.angle));
    else if (m_gizmoDelta.uniform)
        std::snprintf(label, sizeof label, "x%.3f", static_cast<double>(m_gizmoDelta.scale.x));
    else
        std::snprintf(label, sizeof label, "x%.3f, %.3f, %.3f", static_cast<double>(m_gizmoDelta.scale.x), static_cast<double>(m_gizmoDelta.scale.y),
                      static_cast<double>(m_gizmoDelta.scale.z));
    const ImVec2 m = ImGui::GetIO().MousePos;
    fg->AddText(ImVec2(m.x + 16, m.y + 12), IM_COL32(255, 214, 60, 255), label);
}

void Application::drawPushPullOverlay() const {
    const ImGuiIO& io = ImGui::GetIO();
    if (m_push.body < 0) return;
    ImDrawList* fg = ImGui::GetForegroundDrawList();
    const auto a = projectToScreen(camera(), m_push.at, io.DisplaySize), b = projectToScreen(camera(), m_push.at + m_push.dir * m_push.distance, io.DisplaySize);
    if (!a || !b) return;
    const ImU32 col = m_push.distance >= 0 ? IM_COL32(110, 200, 140, 240) : IM_COL32(240, 130, 110, 240);
    fg->AddCircleFilled(*a, 3.0f, col);
    fg->AddLine(*a, *b, col, 2.0f);
    const float dx = b->x - a->x, dy = b->y - a->y, l = std::hypot(dx, dy);
    if (l > 6) {
        const float ux = dx / l, uy = dy / l;
        fg->AddTriangleFilled(*b, ImVec2(b->x - ux * 10 - uy * 5, b->y - uy * 10 + ux * 5), ImVec2(b->x - ux * 10 + uy * 5, b->y - uy * 10 - ux * 5), col);
    }
    char label[64];
    std::snprintf(label, sizeof label, "%s %.1f mm", m_push.distance >= 0 ? "+" : "", static_cast<double>(m_push.distance));
    fg->AddText(ImVec2(b->x + 10, b->y - 8), col, label);
}

void Application::advanceSimulation(float dt) {
    UiState& st = m_uiState;
    st.simDurationSetting = m_doc.state.simDuration;
    st.simDuration = m_doc.state.simDuration > 0.0f ? m_doc.state.simDuration : suggestedDuration(m_doc.scene);
    st.simTime = std::max(st.simTime, 0.0f);
    if (!simulating()) {
        m_simTransforms.clear();
        return;
    }
    if (st.simPlaying) {
        st.simTime += dt * st.simSpeed;
        if (st.simTime >= st.simDuration) {
            if (st.simLoop) st.simTime = std::fmod(st.simTime, st.simDuration);
            else {
                st.simTime = st.simDuration;
                st.simPlaying = false;
            }
        }
        m_framesToRender = std::max(m_framesToRender, 2); // keep animating
    }
    m_simTransforms = simulationTransforms(m_doc.scene, st.simTime);
}

namespace {

// Point on a circle around `axis` through `c`, radius r, at angle a.
Vec3 ringPoint(const Vec3& c, const Vec3& axis, float r, float a) {
    const Vec3 n = normalize(axis);
    const Vec3 u = normalize(std::fabs(n.z) < 0.9f ? cross(n, Vec3{0, 0, 1}) : cross(n, Vec3{1, 0, 0}));
    const Vec3 v = cross(n, u);
    return c + (u * std::cos(a) + v * std::sin(a)) * r;
}

ImU32 toU32(ImVec4 c, float alpha = 1.0f) { return ImGui::ColorConvertFloat4ToU32({c.x, c.y, c.z, c.w * alpha}); }

} // namespace

void Application::updateLoopShapes() {
    // Chains and belts moved along their loop: rebuild their shape when the travel changes.
    const auto& bodies = m_doc.scene.bodies();
    m_loopShapes.resize(bodies.size());
    m_loopOverrides.assign(bodies.size(), nullptr);
    bool any = false;
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        LoopShape& shape = m_loopShapes[i];
        if (!isLoopKind(bodies[i].kind)) {
            shape = LoopShape{};
            continue;
        }
        const float travel = loopTravel(m_doc.scene, i, m_uiState.simTime);
        if (travel == 0.0f) continue; // the stored mesh is the shape at rest
        if (!shape.valid || shape.travel != travel || shape.revision != m_doc.scene.revision()) {
            shape.mesh = loopMeshAt(bodies[i], travel);
            shape.travel = travel;
            shape.revision = m_doc.scene.revision();
            shape.valid = true;
        }
        m_loopOverrides[i] = &shape.mesh;
        any = true;
    }
    if (!any) m_loopOverrides.clear();
}

// Sketch constraints in the 2D view: each dimension's value by its geometry, and a
// short tag (H, V, //, perpendicular, T, =, ...) for the geometric ones. Unmet ones red.
// The colour scale of a result plot (FEA PLOT) shown in the window: its legend in the corner.
void Application::drawFieldLegend() const {
    const Body* shown = nullptr;
    for (const Body& b : m_doc.scene.bodies())
        if (b.visible && !b.field.empty()) shown = &b;
    if (!shown) return;
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    const ImVec2 lo = m_ui.canvasMin();
    const float x = lo.x + 16, y = m_ui.canvasMax().y - 120, w = 220, hgt = 14;
    if (shown->field.rfind("draft", 0) == 0) { // a draft analysis: its four colours, named
        static const DraftClass kinds[4] = {DraftClass::Positive, DraftClass::Negative, DraftClass::Shallow, DraftClass::Undercut};
        static const char* const names[4] = {SICDM_TR_NOOP("cavity side"), SICDM_TR_NOOP("core side"), SICDM_TR_NOOP("too little draft"), SICDM_TR_NOOP("undercut")};
        float at = x;
        for (int k = 0; k < 4; ++k) {
            const Vec3 c = draftColour(kinds[k]);
            const ImU32 col = IM_COL32(static_cast<int>(c.x * 255), static_cast<int>(c.y * 255), static_cast<int>(c.z * 255), 255);
            dl->AddRectFilled(ImVec2(at, y), ImVec2(at + hgt, y + hgt), col);
            const char* label = tr(names[k]);
            dl->AddText(ImVec2(at + hgt + 5, y - 1), IM_COL32(220, 225, 235, 230), label);
            at += hgt + 14 + ImGui::CalcTextSize(label).x;
        }
        dl->AddText(ImVec2(x, y - 18), IM_COL32(220, 225, 235, 230), (shown->name + ": " + shown->field).c_str());
        return;
    }
    static const ImU32 stops[5] = {IM_COL32(26, 64, 242, 255), IM_COL32(26, 217, 242, 255), IM_COL32(51, 230, 64, 255), IM_COL32(250, 230, 38, 255),
                                   IM_COL32(242, 38, 26, 255)};
    for (int k = 0; k < 4; ++k)
        dl->AddRectFilledMultiColor(ImVec2(x + w * static_cast<float>(k) / 4, y), ImVec2(x + w * static_cast<float>(k + 1) / 4, y + hgt), stops[k], stops[k + 1],
                                    stops[k + 1], stops[k]);
    dl->AddRect(ImVec2(x, y), ImVec2(x + w, y + hgt), IM_COL32(200, 205, 215, 160));
    dl->AddText(ImVec2(x, y - 18), IM_COL32(220, 225, 235, 230), (shown->name + ": " + shown->field).c_str());
}

// Datum planes as framed squares (the sketch plane brighter), reference axes as long dashed
// lines and reference points as crosses, each with its name.
void Application::drawDatumOverlay() const {
    const ImGuiIO& io = ImGui::GetIO();
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    dl->PushClipRect(m_ui.canvasMin(), m_ui.canvasMax(), true);
    const AABB box = m_doc.scene.bounds();
    const float size = box.valid() ? std::max(10.0f, 0.35f * length(box.max - box.min)) : 50.0f;
    auto scr = [&](const Vec3& p) { return projectToScreen(camera(), p, io.DisplaySize); };
    const ImU32 plane = IM_COL32(120, 170, 255, 110), active = IM_COL32(255, 190, 90, 200), text = IM_COL32(200, 215, 240, 220);
    for (const DatumPlane& d : m_doc.state.datums) {
        const bool on = m_doc.state.plane == SketchPlane::Custom && d.name == m_doc.state.customPlane;
        const PlaneFrame& f = d.frame;
        const Vec3 c[4] = {f.toWorld({-size, -size}), f.toWorld({size, -size}), f.toWorld({size, size}), f.toWorld({-size, size})};
        std::optional<ImVec2> s[4];
        bool all = true;
        for (int k = 0; k < 4; ++k) all = (s[k] = scr(c[k])).has_value() && all;
        if (!all) continue;
        const ImU32 col = on ? active : plane;
        for (int k = 0; k < 4; ++k) dl->AddLine(*s[k], *s[(k + 1) % 4], col, on ? 2.0f : 1.2f);
        dl->AddQuadFilled(*s[0], *s[1], *s[2], *s[3], (col & 0x00FFFFFF) | (on ? 0x22000000u : 0x14000000u));
        dl->AddText(ImVec2(s[3]->x + 4, s[3]->y + 2), on ? active : text, d.name.c_str());
    }
    for (const DatumAxis& x : m_doc.state.refAxes) {
        const auto a = scr(x.origin - x.dir * (size * 1.4f)), b = scr(x.origin + x.dir * (size * 1.4f));
        if (!a || !b) continue;
        const float len = std::hypot(b->x - a->x, b->y - a->y);
        const int dashes = std::max(1, static_cast<int>(len / 12.0f));
        for (int k = 0; k < dashes; k += 2) { // dash, gap, dash...
            const float t0 = static_cast<float>(k) / static_cast<float>(dashes), t1 = static_cast<float>(k + 1) / static_cast<float>(dashes);
            dl->AddLine(ImVec2(a->x + (b->x - a->x) * t0, a->y + (b->y - a->y) * t0), ImVec2(a->x + (b->x - a->x) * t1, a->y + (b->y - a->y) * t1),
                        IM_COL32(120, 230, 200, 210), 1.5f);
        }
        dl->AddText(ImVec2(b->x + 4, b->y - 14), text, x.name.c_str());
    }
    for (const DatumPoint& p : m_doc.state.refPoints) {
        const auto s = scr(p.at);
        if (!s) continue;
        const ImU32 col = IM_COL32(255, 220, 120, 230);
        dl->AddLine(ImVec2(s->x - 5, s->y - 5), ImVec2(s->x + 5, s->y + 5), col, 1.6f);
        dl->AddLine(ImVec2(s->x - 5, s->y + 5), ImVec2(s->x + 5, s->y - 5), col, 1.6f);
        dl->AddText(ImVec2(s->x + 7, s->y - 7), text, p.name.c_str());
    }
    dl->PopClipRect();
}

void Application::drawConstraintOverlay() const {
    const ImGuiIO& io = ImGui::GetIO();
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    dl->PushClipRect(m_ui.canvasMin(), m_ui.canvasMax(), true);
    const Sketch& sk = m_doc.scene.sketch();
    auto find = [&](const std::string& n) -> const SketchCurve* {
        for (const SketchCurve& c : sk.curves())
            if (c.name == n) return &c;
        return nullptr;
    };
    // Where a reference sits on the plane: a point, a side's middle, a circle's rim.
    auto anchor = [&](const SketchRef& r, Vec2* out) {
        const SketchCurve* c = find(r.curve);
        if (!c || c->pts.empty()) return false;
        auto corner = [&](int k) {
            if (c->kind != CurveKind::Rectangle) return c->pts[static_cast<std::size_t>(std::clamp(k, 0, static_cast<int>(c->pts.size()) - 1))];
            const Vec2 a = c->pts[0], b = c->pts[1];
            k = ((k % 4) + 4) % 4;
            return k == 0 ? a : k == 1 ? Vec2{b.x, a.y} : k == 2 ? b : Vec2{a.x, b.y};
        };
        if (r.segment >= 0) {
            const Vec2 a = corner(r.segment), b = corner(c->kind == CurveKind::Rectangle ? r.segment + 1 : r.segment + 1);
            *out = {(a.x + b.x) * 0.5f, (a.y + b.y) * 0.5f};
        } else if (r.point >= 0) *out = corner(r.point);
        else if (c->kind == CurveKind::Circle) *out = {c->pts[0].x + c->radius * 0.707f, c->pts[0].y + c->radius * 0.707f};
        else if (c->kind == CurveKind::Line) *out = {(c->pts[0].x + c->pts[1].x) * 0.5f, (c->pts[0].y + c->pts[1].y) * 0.5f};
        else *out = c->pts[0];
        return c->visible;
    };
    for (const SketchConstraint& k : sk.constraints()) {
        if (k.refs.empty()) continue;
        const SketchCurve* first = find(k.refs[0].curve);
        if (!first || !onPlane(*first, m_uiState.plane, m_doc.state.customFrame)) continue;
        Vec2 a{}, b{};
        if (!anchor(k.refs[0], &a)) continue;
        if (k.refs.size() > 1 && anchor(k.refs[1], &b)) a = {(a.x + b.x) * 0.5f, (a.y + b.y) * 0.5f};
        const auto s = projectToScreen(camera(), curveFrame(*first).toWorld(a), io.DisplaySize);
        if (!s) continue;
        std::string label;
        switch (k.kind) {
        case ConstraintKind::Horizontal: label = "H"; break;
        case ConstraintKind::Vertical: label = "V"; break;
        case ConstraintKind::Parallel: label = "//"; break;
        case ConstraintKind::Perpendicular: label = "_|_"; break;
        case ConstraintKind::Tangent: label = "T"; break;
        case ConstraintKind::Equal: label = "="; break;
        case ConstraintKind::Concentric: label = "(o)"; break;
        case ConstraintKind::Coincident: label = "o"; break;
        case ConstraintKind::PointOn: label = "on"; break;
        case ConstraintKind::Midpoint: label = "mid"; break;
        case ConstraintKind::Fix: label = "fix"; break;
        default: {
            char t[64];
            std::snprintf(t, sizeof(t), k.kind == ConstraintKind::Angle ? "%s %g\xC2\xB0" : k.kind == ConstraintKind::Radius ? "%s R%g"
                                         : k.kind == ConstraintKind::Diameter ? "%s \xC3\x98%g" : "%s %g",
                          k.name.c_str(), k.value);
            label = t;
        }
        }
        const double err = constraintError(sk, k);
        const ImU32 col = std::isnan(err) || err > 1e-4 ? IM_COL32(255, 110, 100, 255) : IM_COL32(120, 200, 255, 255);
        const ImVec2 ts = ImGui::CalcTextSize(label.c_str());
        dl->AddRectFilled({s->x + 4, s->y - 2}, {s->x + 10 + ts.x, s->y + ts.y + 2}, IM_COL32(16, 18, 24, 200), 3.0f);
        dl->AddText({s->x + 7, s->y}, col, label.c_str());
    }
    dl->PopClipRect();
}

// CAM toolpaths over the 3D view: cuts in each operation's colour, plunges yellow,
// rapids thin red. Lathe paths are drawn in the plane through the turning axis.
void Application::drawToolpathOverlay() const {
    const CamJob& job = m_doc.state.cam;
    const LatheJob& lathe = m_doc.state.lathe;
    std::size_t total = 0, latheMoves = 0;
    if (job.showPaths)
        for (const CamOperation& op : job.ops)
            if (op.generated && !op.suppressed) total += op.moves.size();
    if (lathe.showPaths)
        for (const LatheOperation& op : lathe.ops)
            if (op.generated && !op.suppressed) latheMoves += op.moves.size();
    if (total == 0 && latheMoves == 0) return;
    const ImGuiIO& io = ImGui::GetIO();
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    dl->PushClipRect(m_ui.canvasMin(), m_ui.canvasMax(), true);
    static const ImU32 kColours[] = {IM_COL32(80, 200, 255, 220), IM_COL32(120, 230, 140, 220), IM_COL32(255, 170, 70, 220),
                                     IM_COL32(200, 140, 255, 220), IM_COL32(255, 110, 170, 220), IM_COL32(90, 230, 220, 220)};
    if (latheMoves > 0) {
        for (std::size_t o = 0; o < lathe.ops.size(); ++o) {
            const LatheOperation& op = lathe.ops[o];
            if (!op.generated || op.suppressed || op.moves.empty()) continue;
            const ImU32 cut = kColours[o % std::size(kColours)];
            std::optional<ImVec2> prev;
            for (const LatheMove& m : op.moves) {
                const auto q = projectToScreen(m_cam3D, lathe.frame.toModel(m.x, m.z), io.DisplaySize);
                if (q && prev) dl->AddLine(*prev, *q, m.kind == LatheMove::Rapid ? IM_COL32(235, 80, 70, 130) : cut, m.kind == LatheMove::Rapid ? 1.0f : 1.4f);
                prev = q;
            }
        }
    }
    if (total == 0) {
        dl->PopClipRect();
        return;
    }
    // Very long programs are thinned to keep the view responsive.
    const std::size_t stride = std::max<std::size_t>(1, total / 150000);
    const ImU32 rapid = IM_COL32(235, 80, 70, 130), plunge = IM_COL32(250, 220, 80, 220);
    std::size_t k = 0;
    for (std::size_t o = 0; o < job.ops.size(); ++o) {
        const CamOperation& op = job.ops[o];
        if (!op.generated || op.suppressed || op.moves.empty()) continue;
        const ImU32 cut = kColours[o % std::size(kColours)];
        // Multi-axis moves are in the turned machine frame: drawn where they fall on the part.
        const bool turned = job.multiAxis();
        auto onPart = [&](const CamMove& m) { return turned ? camToPart(job, m.to, m.a, m.c) : m.to; };
        std::optional<ImVec2> prev = projectToScreen(m_cam3D, onPart(op.moves[0]), io.DisplaySize);
        for (std::size_t i = 1; i < op.moves.size(); ++i) {
            const CamMove& m = op.moves[i];
            if (++k % stride != 0 && m.kind == CamMove::Feed && i + 1 < op.moves.size() && op.moves[i + 1].kind == CamMove::Feed) continue;
            const auto q = projectToScreen(m_cam3D, onPart(m), io.DisplaySize);
            if (q && prev) dl->AddLine(*prev, *q, m.kind == CamMove::Rapid ? rapid : m.kind == CamMove::Plunge ? plunge : cut,
                                       m.kind == CamMove::Rapid ? 1.0f : 1.4f);
            prev = q;
        }
    }
    dl->PopClipRect();
}

void Application::drawSimulationOverlay() const {
    const ImGuiIO& io = ImGui::GetIO();
    const OrbitCamera& cam = m_cam3D;
    const Scene& scene = m_doc.scene;
    const UiState& st = m_uiState;
    ImDrawList* dl = ImGui::GetBackgroundDrawList(); // under the panels, over the 3D view
    dl->PushClipRect(m_ui.canvasMin(), m_ui.canvasMax(), true);
    auto scr = [&](const Vec3& p) { return projectToScreen(cam, p, io.DisplaySize); };
    auto line = [&](const Vec3& a, const Vec3& b, ImU32 c, float w) {
        const auto sa = scr(a), sb = scr(b);
        if (sa && sb) dl->AddLine(*sa, *sb, c, w);
    };
    auto label = [&](const Vec3& at, ImU32 c, const char* text) {
        if (const auto s = scr(at)) {
            const ImVec2 ts = ImGui::CalcTextSize(text);
            dl->AddRectFilled({s->x + 6, s->y - 2}, {s->x + 12 + ts.x, s->y + ts.y + 2}, IM_COL32(16, 18, 24, 190), 3.0f);
            dl->AddText({s->x + 9, s->y}, c, text);
        }
    };
    const float wpp = cam.worldPerPixel(io.DisplaySize.y);
    const auto& bodies = scene.bodies();
    const auto& motions = scene.motions();
    const float t = st.simTime;

    // Bodies a motion moves: a representative body and the centre of the group.
    struct Group {
        int body = -1;
        Vec3 centre;
        float radius = 0;
    };
    // Matched once per frame (a humanoid has hundreds of bodies and motions): the bodies
    // each motion moves, each body's motions, and the bounds of the moving bodies.
    const auto moved = motionBodies(scene);
    std::vector<std::vector<std::size_t>> movers(bodies.size());
    std::vector<AABB> boxes(bodies.size());
    for (std::size_t k = 0; k < moved.size(); ++k)
        for (std::size_t i : moved[k]) {
            if (movers[i].empty()) boxes[i] = bodies[i].bounds();
            movers[i].push_back(k);
        }
    auto placed = [&](std::size_t body, float time, std::size_t before = static_cast<std::size_t>(-1)) {
        return bodyMotionTransform(scene, body, time, before, movers[body]);
    };
    std::vector<Group> groups(motions.size());
    for (std::size_t k = 0; k < motions.size(); ++k) {
        AABB box;
        for (std::size_t i : moved[k]) {
            if (groups[k].body < 0) groups[k].body = static_cast<int>(i);
            box.expand(boxes[i]);
        }
        if (box.valid()) {
            groups[k].centre = box.center();
            groups[k].radius = box.radius();
        }
    }

    // Paths: where each motion's group centre (and the selected body) has been.
    if (st.simPaths) {
        auto trail = [&](std::size_t body, const Vec3& p, ImU32 c) {
            constexpr int kSamples = 96;
            const float t0 = std::max(0.0f, t - std::min(t, st.simDuration));
            std::optional<ImVec2> prev;
            for (int s = 0; s <= kSamples; ++s) {
                const float ts = t0 + (t - t0) * static_cast<float>(s) / kSamples;
                const auto q = scr(placed(body, ts).transformPoint(p));
                if (q && prev) dl->AddLine(*prev, *q, c, 1.5f);
                prev = q;
            }
        };
        for (std::size_t k = 0; k < motions.size(); ++k)
            if (motions[k].enabled && groups[k].body >= 0)
                trail(static_cast<std::size_t>(groups[k].body), groups[k].centre, toU32(motionColor(k), 0.75f));
        if (st.selectedBody >= 0 && static_cast<std::size_t>(st.selectedBody) < bodies.size())
            trail(static_cast<std::size_t>(st.selectedBody), bodies[static_cast<std::size_t>(st.selectedBody)].bounds().center(),
                  IM_COL32(255, 255, 255, 170));
    }

    // Motion axes: rotation axes with a turning arrow, travel arrows.
    if (st.simMotionAxes) {
        for (std::size_t k = 0; k < motions.size(); ++k) {
            const Motion& m = motions[k];
            if (!m.enabled || groups[k].body < 0 || length(m.axis) < 1e-6f) continue;
            if (m.alongLoop) { // chains / belts: no straight travel arrow, just the live value
                char travel[96];
                std::snprintf(travel, sizeof(travel), "%s  %s  %s/s", m.name.c_str(),
                              lengthText(motionValue(scene, m, t)).c_str(), lengthText(motionRate(scene, m, t)).c_str());
                label(groups[k].centre, toU32(motionColor(k)), travel);
                continue;
            }
            const ImU32 c = toU32(motionColor(k));
            const Mat4 outer = placed(static_cast<std::size_t>(groups[k].body), t, k);
            const Vec3 dir = normalize(outer.transformDir(m.axis));
            const float r = std::max(groups[k].radius * 0.6f, 24.0f * wpp);
            char text[96];
            if (m.kind == MotionKind::Circular)
                std::snprintf(text, sizeof(text), "%s  %.1f deg  %.0f deg/s", m.name.c_str(),
                              static_cast<double>(motionValue(scene, m, t)), static_cast<double>(motionRate(scene, m, t)));
            else
                std::snprintf(text, sizeof(text), "%s  %s  %s/s", m.name.c_str(), lengthText(motionValue(scene, m, t)).c_str(),
                              lengthText(motionRate(scene, m, t)).c_str());
            if (m.kind == MotionKind::Circular) {
                const Vec3 pivot = m.eachCentre ? boxes[static_cast<std::size_t>(groups[k].body)].center() : m.origin;
                const Vec3 o = outer.transformPoint(pivot);
                const float len = r * 1.6f;
                for (int d = -6; d < 6; d += 2) // dashed axis line
                    line(o + dir * (len * static_cast<float>(d) / 6.0f), o + dir * (len * static_cast<float>(d + 1) / 6.0f), c, 2.0f);
                // Arc arrow turning with the motion.
                const float base = radians(motionValue(m, t));
                const float sign = motionRate(m, t) < 0.0f ? -1.0f : 1.0f;
                std::optional<ImVec2> prev;
                constexpr int kArc = 28;
                for (int s = 0; s <= kArc; ++s) {
                    const float a = base + sign * 1.5f * kPi * static_cast<float>(s) / kArc;
                    const auto q = scr(ringPoint(o, dir, r, a));
                    if (q && prev) dl->AddLine(*prev, *q, c, 2.0f);
                    prev = q;
                }
                const float aEnd = base + sign * 1.5f * kPi;
                const auto tip = scr(ringPoint(o, dir, r, aEnd));
                const auto back = scr(ringPoint(o, dir, r, aEnd - sign * 0.25f));
                if (tip && back) {
                    const ImVec2 d{tip->x - back->x, tip->y - back->y};
                    const float l = std::max(std::hypot(d.x, d.y), 1e-3f);
                    const ImVec2 u{d.x / l, d.y / l}, n{-u.y, u.x};
                    dl->AddTriangleFilled({tip->x + u.x * 4, tip->y + u.y * 4},
                                          {tip->x - u.x * 8 + n.x * 5, tip->y - u.y * 8 + n.y * 5},
                                          {tip->x - u.x * 8 - n.x * 5, tip->y - u.y * 8 - n.y * 5}, c);
                }
                label(o + dir * len, c, text);
            } else {
                const Mat4 full = placed(static_cast<std::size_t>(groups[k].body), t);
                const Vec3 p = full.transformPoint(groups[k].centre);
                const float rate = motionRate(scene, m, t);
                const float sign = (rate != 0.0f ? rate : m.speed) < 0.0f ? -1.0f : 1.0f; // current direction
                const Vec3 end = p + dir * (sign * r * 1.4f);
                line(p, end, c, 2.5f);
                const auto se = scr(end), sp = scr(p);
                if (se && sp) {
                    const ImVec2 d{se->x - sp->x, se->y - sp->y};
                    const float l = std::max(std::hypot(d.x, d.y), 1e-3f);
                    const ImVec2 u{d.x / l, d.y / l}, n{-u.y, u.x};
                    dl->AddTriangleFilled({se->x + u.x * 6, se->y + u.y * 6},
                                          {se->x - u.x * 7 + n.x * 6, se->y - u.y * 7 + n.y * 6},
                                          {se->x - u.x * 7 - n.x * 6, se->y - u.y * 7 - n.y * 6}, c);
                }
                label(end, c, text);
            }
        }
    }

    // Real-time axes: the moving X / Y / Z of each motion group and of the selection.
    if (st.simAxes) {
        auto triad = [&](std::size_t body, const Vec3& p, float size, bool strong) {
            const Mat4 tf = placed(body, t);
            const Vec3 o = tf.transformPoint(p);
            const ImU32 cols[] = {IM_COL32(240, 80, 80, 255), IM_COL32(90, 210, 90, 255), IM_COL32(90, 140, 255, 255)};
            const char* names[] = {"X", "Y", "Z"};
            const Vec3 axes[] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
            for (int a = 0; a < 3; ++a) {
                const Vec3 e = o + normalize(tf.transformDir(axes[a])) * size;
                line(o, e, cols[a], strong ? 3.0f : 2.0f);
                if (const auto s = scr(e)) dl->AddText({s->x + 3, s->y - 7}, cols[a], names[a]);
            }
            if (const auto s = scr(o)) dl->AddCircleFilled(*s, strong ? 4.0f : 3.0f, IM_COL32(255, 255, 255, 230));
        };
        for (std::size_t k = 0; k < motions.size(); ++k)
            if (motions[k].enabled && groups[k].body >= 0)
                triad(static_cast<std::size_t>(groups[k].body), groups[k].centre,
                      std::max(groups[k].radius * 0.45f, 30.0f * wpp), false);
        if (st.selectedBody >= 0 && static_cast<std::size_t>(st.selectedBody) < bodies.size()) {
            const Body& b = bodies[static_cast<std::size_t>(st.selectedBody)];
            triad(static_cast<std::size_t>(st.selectedBody), b.bounds().center(), std::max(b.bounds().radius() * 0.8f, 36.0f * wpp),
                  true);
        }
    }

    // Physics: wavefronts, field arrows, velocity and force vectors, temperature halos,
    // trails, and the energy balance.
    if (const auto run = physicsRun(scene); run && !run->frames.empty()) {
        const PhysicsSettings& ps = run->settings;
        const PhysicsFrame f = run->at(t);
        auto arrow = [&](const Vec3& from, const Vec3& to, ImU32 c, float w) {
            const auto a = scr(from), b = scr(to);
            if (!a || !b) return;
            dl->AddLine(*a, *b, c, w);
            const ImVec2 d{b->x - a->x, b->y - a->y};
            const float l = std::hypot(d.x, d.y);
            if (l < 4.0f) return;
            const ImVec2 u{d.x / l, d.y / l}, n{-u.y, u.x};
            dl->AddTriangleFilled({b->x + u.x * 5, b->y + u.y * 5}, {b->x - u.x * 7 + n.x * 5, b->y - u.y * 7 + n.y * 5},
                                  {b->x - u.x * 7 - n.x * 5, b->y - u.y * 7 - n.y * 5}, c);
        };
        // Temperature colour: blue (0 C) - white (20 C) - red (100 C and hotter).
        auto heat = [](float kelvin, float alpha) {
            const float c = kelvin - 273.15f;
            float r = 1, g = 1, b = 1;
            if (c < 20.0f) {
                const float k = std::clamp((20.0f - c) / 20.0f, 0.0f, 1.0f);
                r = 1 - 0.7f * k, g = 1 - 0.4f * k;
            } else {
                const float k = std::clamp((c - 20.0f) / 80.0f, 0.0f, 1.0f);
                g = 1 - 0.75f * k, b = 1 - 0.85f * k;
            }
            return IM_COL32(static_cast<int>(r * 255), static_cast<int>(g * 255), static_cast<int>(b * 255), static_cast<int>(alpha * 255));
        };
        // Waves: crests emitted every period from where the source was then (Doppler).
        for (std::size_t i = 0; i < run->bodies.size(); ++i) {
            const PhysicsBodyInfo& b = run->bodies[i];
            if (b.props.wave == WaveKind::Off) continue;
            const double speed = b.props.wave == WaveKind::Sound ? kSoundSpeed * 1000.0
                                 : b.props.wave == WaveKind::Light ? (ps.relativity ? ps.lightSpeed : kLightSpeed) * 1000.0
                                                                   : static_cast<double>(b.props.waveSpeed);
            const double period = 1.0 / std::max(b.props.waveFrequency, 1e-6f);
            const double reach = std::max(4000.0, static_cast<double>(scene.bounds().radius()) * 3.0); // mm
            const ImU32 c = IM_COL32(120, 200, 255, 255);
            int drawn = 0;
            for (double te = std::floor(t / period) * period; te >= 0 && drawn < 40; te -= period, ++drawn) {
                const double r = speed * (t - te);
                if (r > reach) break;
                const Vec3 centre = run->at(static_cast<float>(te)).bodies[i].position;
                const float fade = 1.0f - static_cast<float>(r / reach);
                std::optional<ImVec2> prev;
                for (int k = 0; k <= 72; ++k) {
                    const auto q = scr(ringPoint(centre, {0, 0, 1}, static_cast<float>(r), 2.0f * kPi * static_cast<float>(k) / 72.0f));
                    if (q && prev) dl->AddLine(*prev, *q, (c & 0x00FFFFFF) | (static_cast<ImU32>(40 + 180 * fade) << 24), 1.5f);
                    prev = q;
                }
            }
        }
        // Uniform fields: a few arrows around the scene.
        const AABB sb = scene.bounds();
        const Vec3 mid = sb.valid() ? sb.center() : Vec3{};
        const float span = sb.valid() ? std::max(sb.radius(), 200.0f) : 500.0f;
        auto field = [&](const Vec3& v, ImU32 c, const char* name, const char* unit) {
            if (length(v) <= 0.0f) return;
            const Vec3 u = normalize(v);
            const float len = 80.0f * wpp;
            for (int x = -1; x <= 1; ++x)
                for (int y = -1; y <= 1; ++y) {
                    const Vec3 o = mid + Vec3{static_cast<float>(x) * span * 0.7f, static_cast<float>(y) * span * 0.7f, 0};
                    arrow(o - u * (len * 0.5f), o + u * (len * 0.5f), c, 1.5f);
                }
            char text[64];
            std::snprintf(text, sizeof(text), "%s %.3g %s", name, static_cast<double>(length(v)), unit);
            label(mid + Vec3{span * 0.7f, span * 0.7f, 0} + u * (len * 0.5f), c, text);
        };
        field(ps.electricField, IM_COL32(250, 210, 80, 200), "E", "V/m");
        field(ps.magneticField, IM_COL32(90, 220, 230, 200), "B", "T");
        // Per body: trail, temperature halo, velocity (green) and net force (orange).
        float maxForce = 1e-9f;
        for (const PhysicsBodyState& b : f.bodies) maxForce = std::max(maxForce, length(b.force));
        for (std::size_t i = 0; i < run->bodies.size(); ++i) {
            const PhysicsBodyInfo& info = run->bodies[i];
            const PhysicsBodyState& b = f.bodies[i];
            if (st.simPaths && !info.props.fixed) {
                std::optional<ImVec2> prev;
                const int samples = 120;
                for (int k = 0; k <= samples; ++k) {
                    const auto q = scr(run->at(t * static_cast<float>(k) / samples).bodies[i].position);
                    if (q && prev) dl->AddLine(*prev, *q, IM_COL32(255, 255, 255, 110), 1.5f);
                    prev = q;
                }
            }
            const float radius = static_cast<float>(info.radius * 1000.0);
            if (const auto c = scr(b.position)) {
                const float px = std::max(radius / wpp, 6.0f);
                dl->AddCircle(*c, px + 3.0f, heat(b.temperature, 0.85f), 48, 2.0f);
            }
            if (!info.props.fixed && length(b.velocity) > 1e-3f) {
                // 0.25 s of travel, at least 30 px.
                Vec3 d = b.velocity * 0.25f;
                if (length(d) < 30.0f * wpp) d = normalize(d) * (30.0f * wpp);
                arrow(b.position, b.position + d, IM_COL32(110, 230, 120, 255), 2.5f);
            }
            if (length(b.force) > 1e-9f)
                arrow(b.position, b.position + normalize(b.force) * (wpp * (20.0f + 70.0f * length(b.force) / maxForce)),
                      IM_COL32(255, 160, 60, 255), 2.0f);
            if (st.simReadout) {
                char text[160];
                const double v = length(b.velocity) / 1000.0;
                int at = std::snprintf(text, sizeof(text), "%s  %.3g m/s  %.1f C", info.name.c_str(), v,
                                       static_cast<double>(b.temperature - 273.15f));
                if (ps.relativity && at > 0 && at < static_cast<int>(sizeof(text)))
                    std::snprintf(text + at, sizeof(text) - static_cast<std::size_t>(at), "  gamma %.3f",
                                  1.0 / std::sqrt(std::max(1.0 - v * v / (ps.lightSpeed * ps.lightSpeed), 1e-12)));
                label(b.position + Vec3{radius, 0, radius}, heat(b.temperature, 1.0f), text);
            }
        }
        // Energy balance, bottom left: where the energy is, and that its total holds.
        if (st.simReadout) {
            const PhysicsEnergy& e = f.energy;
            const double start = run->frames.front().energy.balance();
            struct Bar {
                const char* name;
                double value;
                ImU32 colour;
            } bars[] = {{"Kinetic", e.kinetic, IM_COL32(110, 230, 120, 255)},
                        {"Potential", e.gravity + e.electric, IM_COL32(110, 160, 255, 255)},
                        {"Heat", e.thermal, IM_COL32(255, 120, 90, 255)},
                        {"To air", e.toAir, IM_COL32(200, 200, 210, 255)},
                        {"Work in", -e.externalWork, IM_COL32(250, 210, 80, 255)}};
            double top = 1e-12;
            for (const Bar& b : bars) top = std::max(top, std::fabs(b.value));
            top = std::max(top, std::fabs(start));
            const ImVec2 o{m_ui.canvasMin().x + 14, m_ui.canvasMin().y + 78}; // under the clock
            dl->AddRectFilled({o.x - 8, o.y - 26}, {o.x + 290, o.y + 128}, IM_COL32(16, 18, 24, 200), 6.0f);
            char head[96];
            std::snprintf(head, sizeof(head), "Energy  total %.4g J  drift %.3f %%", e.balance(),
                          100.0 * (e.balance() - start) / std::max(std::fabs(start), top * 1e-3));
            dl->AddText({o.x, o.y - 20}, IM_COL32(230, 232, 238, 255), head);
            for (int k = 0; k < 5; ++k) {
                const float y = o.y + static_cast<float>(k) * 24.0f;
                dl->AddText({o.x, y}, IM_COL32(190, 195, 205, 255), bars[k].name);
                const float w = static_cast<float>(150.0 * std::fabs(bars[k].value) / top);
                dl->AddRectFilled({o.x + 72, y + 2}, {o.x + 72 + w, y + 14}, bars[k].colour, 2.0f);
                char v[32];
                std::snprintf(v, sizeof(v), "%.3g J", bars[k].value);
                dl->AddText({o.x + 228, y}, IM_COL32(190, 195, 205, 255), v);
            }
            char mom[96];
            std::snprintf(mom, sizeof(mom), "Momentum %.4g kg m/s   %d collisions in the run", static_cast<double>(length(f.momentum)),
                          run->collisions);
            dl->AddText({o.x, o.y + 120 - 2}, IM_COL32(160, 165, 175, 255), mom);
        }
    }

    // Clock in the corner of the canvas.
    if (st.simReadout) {
        char clock[96];
        std::snprintf(clock, sizeof(clock), "%s  t = %.2f s   %gx%s", st.simPlaying ? "PLAYING" : "PAUSED",
                      static_cast<double>(t), static_cast<double>(st.simSpeed), st.simLoop ? "   loop" : "");
        const ImVec2 at{m_ui.canvasMin().x + 14, m_ui.canvasMin().y + 12};
        const ImVec2 ts = ImGui::CalcTextSize(clock);
        dl->AddRectFilled({at.x - 8, at.y - 6}, {at.x + ts.x + 8, at.y + ts.y + 6}, IM_COL32(16, 18, 24, 200), 5.0f);
        dl->AddText(at, st.simPlaying ? IM_COL32(120, 220, 140, 255) : IM_COL32(230, 200, 120, 255), clock);
        // Progress bar under the clock.
        const float frac = st.simDuration > 0 ? std::clamp(t / st.simDuration, 0.0f, 1.0f) : 0.0f;
        dl->AddRectFilled({at.x - 8, at.y + ts.y + 6}, {at.x - 8 + (ts.x + 16) * frac, at.y + ts.y + 8},
                          IM_COL32(90, 160, 255, 255));
        if (motions.empty() && !physicsRun(scene))
            dl->AddText({at.x, at.y + ts.y + 16}, IM_COL32(200, 205, 215, 220),
                        "No motions: add a Circular or Linear motion in the Motion Study panel");
    }
    dl->PopClipRect();
}

std::string Application::simStatus() const {
    char buf[128];
    std::snprintf(buf, sizeof(buf), "Simulation %s at %gx, t = %.2f / %.2f s%s", m_uiState.simPlaying ? "playing" : "paused",
                  static_cast<double>(m_uiState.simSpeed), static_cast<double>(m_uiState.simTime),
                  static_cast<double>(m_uiState.simDuration), m_uiState.simLoop ? ", looping" : "");
    return buf;
}

bool Application::simulation(const std::string& action, float value, std::string* reply) {
    UiState& st = m_uiState;
    if (action == "open" || action == "play" || action == "toggle") setMode(ViewMode::Simulation);
    if (action == "play") {
        if (st.simTime >= st.simDuration - 1e-4f) st.simTime = 0.0f;
        st.simPlaying = true;
    } else if (action == "pause") {
        st.simPlaying = false;
    } else if (action == "toggle") {
        st.simPlaying = !st.simPlaying;
    } else if (action == "stop") {
        st.simPlaying = false;
        st.simTime = 0.0f;
    } else if (action == "time") {
        st.simTime = std::max(value, 0.0f);
    } else if (action == "speed") {
        if (value <= 0.0f) {
            *reply = "Speed must be > 0";
            return false;
        }
        st.simSpeed = value;
    } else if (action == "axes" || action == "paths") {
        bool& flag = action == "axes" ? st.simAxes : st.simPaths;
        flag = value < 0.0f ? !flag : value != 0.0f;
        *reply = std::string(action == "axes" ? "Real-time axes " : "Paths ") + (flag ? "on" : "off");
        return true;
    } else if (action != "open") {
        *reply = "Unknown simulation action " + action;
        return false;
    }
    advanceSimulation(0.0f);
    *reply = simStatus();
    m_framesToRender = std::max(m_framesToRender, 3);
    return true;
}

void Application::standardView(StandardView v) {
    UiCommands c;
    c.view = v;
    handleCommands(c);
}

void Application::planeChanged(SketchPlane p) { setPlane(p); }

void Application::fit() { camera().fitToBounds(viewBounds()); }

AABB Application::viewBounds() const {
    AABB box = m_doc.scene.bounds();
    // Simulating physics: everywhere the bodies go during the run.
    if (simulating())
        if (const auto run = physicsRun(m_doc.scene))
            for (std::size_t i = 0; i < run->bodies.size(); ++i) {
                const float r = static_cast<float>(run->bodies[i].radius * 1000.0);
                for (const PhysicsFrame& f : run->frames) {
                    const Vec3 p = f.bodies[i].position;
                    box.expand(AABB{p - Vec3{r, r, r}, p + Vec3{r, r, r}});
                }
            }
    // Exploded: the parts where they are drawn (where they end up, while it opens).
    const ExplodeState& ex = m_doc.state.assembly.explode;
    if (!simulating() && ex.shown && !ex.steps.empty() && m_uiState.mode == ViewMode::Model3D) {
        const std::vector<DVec3> shifts = explodeShifts(m_doc.scene, ex, 1);
        const auto& bodies = m_doc.scene.bodies();
        for (std::size_t i = 0; i < bodies.size() && i < shifts.size(); ++i) {
            if (!bodies[i].visible) continue;
            AABB b = bodies[i].bounds();
            if (!b.valid()) continue;
            b.min += shifts[i].f(), b.max += shifts[i].f();
            box.expand(b);
        }
    }
    return box;
}

void Application::zoomStep(float steps) { camera().zoom(steps); }

void Application::home() {
    UiCommands c;
    c.home = true;
    if (m_uiState.mode == ViewMode::Drawing2D) c.mode = ViewMode::Model3D; // Simulation keeps its window
    handleCommands(c);
}

void Application::roll(float degrees, bool absolute) {
    if (absolute) m_cam3D.resetRoll();
    else m_cam3D.rollView(radians(degrees));
}

void Application::setProjection(Projection p) { m_cam3D.setProjection(p); }

void Application::setDisplay(int mode) { m_vp3D.displayMode = static_cast<DisplayMode>(std::clamp(mode, 0, 3)); }

void Application::setGrid(std::optional<bool> on) { viewport().showGrid = on.value_or(!viewport().showGrid); }

bool Application::setPolygroups(std::optional<bool> on) {
    m_vp3D.showPolygroups = on.value_or(!m_vp3D.showPolygroups);
    return true;
}

bool Application::setBodyTexture(const std::string& body, bool checker, const std::string& colorPath, const std::string& normalPath, std::string* error) {
    ViewportSettings::BodyTexture t;
    t.body = body;
    t.checker = checker;
    auto load = [&](const std::string& path, int& w, int& h, std::vector<unsigned char>& px) {
        if (path.empty()) return true;
        ImageRef img;
        img.path = path;
        if (!loadImagePixels(img, error)) return false;
        w = img.pixelWidth, h = img.pixelHeight, px = std::move(img.pixels);
        return true;
    };
    if (!load(colorPath, t.colorWidth, t.colorHeight, t.color) || !load(normalPath, t.normalWidth, t.normalHeight, t.normal)) return false;
    auto& list = m_vp3D.textures;
    list.erase(std::remove_if(list.begin(), list.end(), [&](const ViewportSettings::BodyTexture& x) { return x.body == body; }), list.end());
    if (checker || !t.color.empty() || !t.normal.empty()) list.push_back(std::move(t));
    ++m_vp3D.textureRevision;
    return true;
}

void Application::pollRender() {
    UiState& st = m_uiState;
    if (st.renderStart && !m_renderDone.valid()) {
        st.renderStart = false;
        // A copy of the scene as it is, the 3D view's camera at the picture's shape.
        auto job = std::make_shared<RenderJob>();
        const RenderSettings rs = m_doc.state.render;
        job->width = rs.width, job->height = rs.height, job->samples = rs.samples;
        OrbitCamera cam = m_cam3D;
        cam.setAspect(static_cast<float>(rs.width) / static_cast<float>(rs.height));
        const RenderCamera rc = renderCamera(cam);
        auto scene = std::make_shared<Scene>(m_doc.scene);
        m_renderJob = job;
        m_renderDone = std::async(std::launch::async, [job, scene, rs, rc] {
            return renderImage(*scene, rs, rc, job->pixels, &job->stats, &job->why, [job](float p) {
                job->progress = p;
                return !job->stop.load();
            });
        });
        st.renderRunning = true;
        st.renderProgress = 0;
        st.renderInfo.clear();
    }
    if (m_renderDone.valid()) {
        st.renderProgress = m_renderJob->progress;
        if (st.renderCancel) m_renderJob->stop = true;
        m_framesToRender = std::max(m_framesToRender, 2); // keep the progress moving
        if (m_renderDone.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            const bool ok = m_renderDone.get();
            st.renderRunning = false;
            if (!ok) {
                st.renderInfo = m_renderJob->why;
            } else {
                m_renderPixels = std::move(m_renderJob->pixels);
                m_renderWidth = m_renderJob->width, m_renderHeight = m_renderJob->height;
                if (!m_renderTexture) glGenTextures(1, &m_renderTexture);
                glBindTexture(GL_TEXTURE_2D, m_renderTexture);
                glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, m_renderWidth, m_renderHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, m_renderPixels.data());
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                glBindTexture(GL_TEXTURE_2D, 0);
                char info[160];
                std::snprintf(info, sizeof(info), "%d x %d, %d samples, %.1f s%s", m_renderWidth, m_renderHeight, m_renderJob->samples,
                              m_renderJob->stats.seconds, st.renderCancel ? " (stopped)" : "");
                st.renderInfo = info;
                st.renderTexture = m_renderTexture, st.renderWidth = m_renderWidth, st.renderHeight = m_renderHeight;
            }
            st.renderCancel = false;
            m_renderJob.reset();
        }
    }
    if (!st.renderSavePath.empty()) {
        std::string err;
        if (m_renderPixels.empty()) setStatus("Render a picture first");
        else if (writeFile(st.renderSavePath, encodePngCompressed(m_renderWidth, m_renderHeight, 3, m_renderPixels), &err))
            setStatus("Picture saved to " + st.renderSavePath);
        else setStatus("Cannot save the picture: " + err);
        st.renderSavePath.clear();
    }
}

void Application::selectionChanged() {
    if (const std::vector<int> curves = selectedCurves(m_doc.scene.sketch(), m_uiState.selectedCurve, m_uiState.curveSelection); !curves.empty()) {
        const std::string& first = m_doc.scene.sketch().curves()[static_cast<std::size_t>(curves[0])].name;
        setStatus(curves.size() == 1 ? "Selected " + first : std::to_string(curves.size()) + " sketch curves selected (" + first + " active)");
        return;
    }
    const std::vector<int> sel = selectedBodies(m_doc.scene, m_uiState.selectedBody, m_uiState.bodySelection);
    if (sel.empty()) setStatus("Nothing selected");
    else if (sel.size() == 1) setStatus("Selected " + m_doc.scene.bodies()[static_cast<std::size_t>(sel[0])].name);
    else setStatus(std::to_string(sel.size()) + " bodies selected (" + m_doc.scene.bodies()[static_cast<std::size_t>(sel[0])].name + " active)");
}

void Application::finishBoxSelect(float x, float y) {
    const ImGuiIO& io = ImGui::GetIO();
    const float x0 = std::min(m_boxStart.x, x), x1 = std::max(m_boxStart.x, x), y0 = std::min(m_boxStart.y, y), y1 = std::max(m_boxStart.y, y);
    if (x1 - x0 < 2 || y1 - y0 < 2) return;
    const bool window = x >= m_boxStart.x; // left to right: wholly inside; right to left: touching
    auto inside = [&](const ImVec2& p) { return p.x >= x0 && p.x <= x1 && p.y >= y0 && p.y <= y1; };
    if (m_uiState.mode == ViewMode::Drawing2D) { // the 2D window: sketch curves
        std::vector<int> list = selectedCurves(m_doc.scene.sketch(), m_uiState.selectedCurve, m_uiState.curveSelection);
        const auto& curves = m_doc.scene.sketch().curves();
        for (std::size_t i = 0; i < curves.size(); ++i) {
            if (!curves[i].visible) continue;
            bool all = true, any = false;
            for (const Vec3& w : curves[i].worldStrip(48)) {
                const auto p = projectToScreen(camera(), w, io.DisplaySize);
                const bool in = p && inside(*p);
                all = all && in, any = any || in;
            }
            if ((window ? all : any) && std::find(list.begin(), list.end(), static_cast<int>(i)) == list.end()) list.push_back(static_cast<int>(i));
        }
        m_uiState.selectedBody = m_uiState.selectedImage = -1;
        m_uiState.bodySelection = {};
        selectCurves(m_doc.scene.sketch(), m_uiState.selectedCurve, m_uiState.curveSelection, list);
        selectionChanged();
        return;
    }
    std::vector<int> list = selectedBodies(m_doc.scene, m_uiState.selectedBody, m_uiState.bodySelection);
    const auto& bodies = m_doc.scene.bodies();
    const auto centre = pick(0.5f * (x0 + x1), 0.5f * (y0 + y1));
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        const Body& b = bodies[i];
        if (!b.visible || b.mesh.vertices.empty()) continue;
        bool in = false;
        if (window) { // its box's corners all inside
            const AABB bb = b.bounds();
            in = true;
            for (int k = 0; k < 8 && in; ++k) {
                const auto p = projectToScreen(camera(), {k & 1 ? bb.max.x : bb.min.x, k & 2 ? bb.max.y : bb.min.y, k & 4 ? bb.max.z : bb.min.z}, io.DisplaySize);
                in = p && inside(*p);
            }
        } else { // a vertex of it inside, or the box's middle on it
            const std::size_t n = b.mesh.vertices.size(), step = std::max<std::size_t>(1, n / 4000);
            for (std::size_t v = 0; v < n && !in; v += step)
                if (const auto p = projectToScreen(camera(), b.mesh.vertices[v].position + b.offset, io.DisplaySize)) in = inside(*p);
            in = in || (centre && centre->body == static_cast<int>(i));
        }
        if (in && std::find(list.begin(), list.end(), static_cast<int>(i)) == list.end()) list.push_back(static_cast<int>(i));
    }
    m_uiState.selectedCurve = m_uiState.selectedImage = -1;
    m_uiState.curveSelection = {};
    selectBodies(m_doc.scene, m_uiState.selectedBody, m_uiState.bodySelection, list);
    selectionChanged();
}

void Application::pollVideo() {
    UiState& st = m_uiState;
    if (!st.videoPath.empty() && !m_videoDone.valid()) {
        // A copy of the scene as it is; the camera from the 3D view's direction.
        static const int kSizes[][2] = {{480, 270}, {640, 360}, {1280, 720}, {1920, 1080}};
        VideoRequest rq;
        rq.path = st.videoPath;
        rq.render = m_doc.state.render;
        const int size = std::clamp(st.videoSize, 0, 3);
        rq.render.width = kSizes[size][0], rq.render.height = kSizes[size][1];
        rq.render.samples = std::max(1, st.videoSamples);
        rq.fps = std::max(1, st.videoFps);
        rq.seconds = st.videoSeconds;
        rq.animate = st.videoKind != 0;
        rq.turns = st.videoKind == 1 ? 0.0f : st.videoTurns;
        OrbitCamera cam = m_cam3D;
        cam.setAspect(static_cast<float>(rq.render.width) / static_cast<float>(rq.render.height));
        rq.camera = renderCamera(cam);
        auto job = std::make_shared<VideoJob>();
        job->path = rq.path;
        auto scene = std::make_shared<Scene>(m_doc.scene);
        m_videoJob = job;
        m_videoDone = std::async(std::launch::async, [job, scene, rq] {
            return makeVideo(*scene, rq, &job->report, &job->why, [job](int done, int total) {
                job->progress = static_cast<float>(done) / static_cast<float>(std::max(total, 1));
                return !job->stop.load();
            });
        });
        st.videoPath.clear();
        st.videoRunning = true;
        st.videoProgress = 0;
        st.videoInfo.clear();
    }
    if (m_videoDone.valid()) {
        st.videoProgress = m_videoJob->progress;
        if (st.videoCancel) m_videoJob->stop = true;
        m_framesToRender = std::max(m_framesToRender, 2); // keep the progress moving
        if (m_videoDone.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            const bool ok = m_videoDone.get();
            st.videoRunning = false;
            st.videoCancel = false;
            st.videoInfo = ok ? videoSummary(m_videoJob->report, m_videoJob->path) : "Video not made: " + m_videoJob->why;
            setStatus(st.videoInfo);
            m_videoJob.reset();
        }
    } else {
        st.videoCancel = false;
    }
}

void Application::setPanel(std::optional<bool> on) { m_uiState.showModelTree = on.value_or(!m_uiState.showModelTree); }

bool Application::setGizmo(int mode) {
    if (mode < 0) {
        m_ui.toolbox().setActive(ToolId::Select);
        return true;
    }
    m_ui.toolbox().setActive(ToolId::Move);
    m_uiState.gizmoMode = std::clamp(mode, 0, 2);
    if (m_uiState.mode == ViewMode::Drawing2D) m_uiState.mode = ViewMode::Model3D;
    return true;
}

void Application::setPanelPage(int page) {
    if (page == 8 || page == 9) m_uiState.showRender = true, m_uiState.renderStart = m_uiState.renderStart || page == 9;
    else if (page == 10) m_uiState.showProduct = true;
    else if (page == 11) m_uiState.showAppearance = true;
    else if (page == 12) m_uiState.showSection = true;
    else if (page == 13) m_uiState.showPath3d = true;
    else if (page == 14) m_uiState.showStudy = true;
    else if (page == 15) m_uiState.showStack = true;
    else if (page == 16) m_uiState.showArmature = true;
    else if (page == 7) m_uiState.showLicence = true;
    else if (page == 6) m_uiState.showPreferences = true;
    else if (page == 5) m_uiState.showHelp = true;
    else if (page == 3) m_uiState.showCamTools = true;
    else if (page == 4) m_uiState.showCamMachine = true;
    else m_uiState.showModelTree = true, m_uiState.dockTab = page;
}

void Application::updateSketchColors() {
    // Each curve's colour from how the constraints hold it: construction grey, fully
    // constrained green, a constraint that can't be met red, free the usual colour.
    const Sketch& sk = m_doc.scene.sketch();
    // Not while dragging (the colours can't change: the constraints all hold throughout).
    if (m_leftDrag == DragMode::SketchGrip && m_vp3D.sketchColors.size() == sk.curves().size()) return;
    const std::uint64_t key = sk.revision() * 1000003u + sk.constraints().size();
    if (key == m_sketchColorKey && m_vp3D.sketchColors.size() == sk.curves().size()) return;
    m_sketchColorKey = key;
    const std::vector<int> freedom = curveFreedom(sk);
    std::vector<Vec3> colors;
    int full = 0, free = 0, conflict = 0;
    for (std::size_t i = 0; i < sk.curves().size(); ++i) {
        const SketchCurve& c = sk.curves()[i];
        const int f = i < freedom.size() ? freedom[i] : -1;
        Vec3 col = m_vp3D.sketchColor;
        if (c.construction) col = {0.62f, 0.66f, 0.74f};
        if (f == 0) col = {0.42f, 0.86f, 0.50f}, ++full;
        else if (f == -2) col = {1.0f, 0.36f, 0.32f}, ++conflict;
        else if (f > 0) ++free;
        colors.push_back(col);
    }
    m_vp3D.sketchColors = colors;
    m_vp2D.sketchColors = colors;
    m_uiState.sketchConstraintStatus.clear();
    if (!sk.constraints().empty()) {
        m_uiState.sketchConstraintStatus = std::to_string(full) + " fully constrained";
        if (free) m_uiState.sketchConstraintStatus += ", " + std::to_string(free) + " free";
        if (conflict) m_uiState.sketchConstraintStatus += ", " + std::to_string(conflict) + " in conflict";
    }
}

bool Application::showSheet(bool on) {
    m_uiState.showSheet = on;
    return true;
}

void Application::updateSheet() {
    if (!m_uiState.showSheet || m_doc.state.drawing.empty()) {
        m_uiState.sheet = nullptr;
        return;
    }
    const Drawing shown = drawingWithProduct(m_doc.state.drawing, m_doc.state.product); // PART and REV from the project
    std::string text = drawingToJson(shown).dump() + assemblyToJson(m_doc.state.assembly).dump();
    if (shown.partsList.shown || !shown.balloons.empty()) // the parts' properties (they don't change the scene's revision)
        for (const auto& row : drawingPartsRows(shown, m_doc.scene))
            for (const std::string& cell : row) text += cell + "|";
    const std::uint64_t key = std::hash<std::string>{}(text) * 1000003u ^ m_doc.scene.revision();
    if (key != m_sheetKey || m_uiState.sheet == nullptr) {
        m_sheet = renderDrawing(shown, m_doc.scene, 1, &m_doc.state.assembly.explode);
        m_sheetKey = key;
    }
    m_uiState.sheet = &m_sheet;
}

std::string Application::describe() const {
    const OrbitCamera& c = camera();
    static const char* kDisplay[] = {"shaded", "shaded+edges", "wireframe", "hidden line"};
    char buf[320];
    std::snprintf(buf, sizeof(buf),
                  "Window: %s   plane: %s   camera target (%.1f, %.1f, %.1f) distance %.1f yaw %.1f pitch %.1f "
                  "roll %.1f   %s   display: %s   grid %s",
                  m_uiState.mode == ViewMode::Drawing2D   ? tr("2D Drawing")
                  : m_uiState.mode == ViewMode::Simulation ? tr("Simulation")
                                                           : tr("3D Model"),
                  activePlaneName(m_doc.state).c_str(),
                  static_cast<double>(c.target().x), static_cast<double>(c.target().y), static_cast<double>(c.target().z),
                  static_cast<double>(c.distance()), static_cast<double>(degrees(c.yaw())),
                  static_cast<double>(degrees(c.pitch())), static_cast<double>(degrees(c.roll())),
                  c.projection() == Projection::Perspective ? "perspective" : "orthographic",
                  kDisplay[static_cast<int>(m_vp3D.displayMode)], viewport().showGrid ? "on" : "off");
    return buf;
}

bool Application::saveScreenshot(const std::string& path) const {
    int w = 0, h = 0;
    glfwGetFramebufferSize(m_window, &w, &h);
    std::vector<unsigned char> px(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 3);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadBuffer(GL_BACK);
    glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, px.data());

    std::ofstream out(path, std::ios::binary);
    if (!out) return false;
    out << "P6\n" << w << ' ' << h << "\n255\n";
    const std::size_t row = static_cast<std::size_t>(w) * 3;
    for (int y = h - 1; y >= 0; --y) // flip: GL origin is bottom-left
        out.write(reinterpret_cast<const char*>(px.data() + static_cast<std::size_t>(y) * row),
                  static_cast<std::streamsize>(row));
    return static_cast<bool>(out);
}

int Application::run() {
    if (!init()) return 1;
    if (m_splash) {
        m_splash->finish();
        m_splash.reset();
        glfwMakeContextCurrent(m_window);
        glfwShowWindow(m_window);
        glfwFocusWindow(m_window);
        m_framesToRender = std::max(m_framesToRender, 3);
    }

    using clock = std::chrono::steady_clock;
    auto last = clock::now();
    int screenshotCountdown = 5;

    while (!glfwWindowShouldClose(m_window)) {
        if (m_framesToRender > 0 || !m_options.screenshotPath.empty()) glfwPollEvents();
        else {
            // Idle: zero CPU/GPU until the user does something, a status message expires,
            // or an autosave falls due.
            double wake = m_uiState.statusMessage.empty() ? -1.0 : m_statusUntil;
            if (const double due = m_autosaver.nextDue(); due >= 0) wake = wake < 0 ? std::max(due, m_autosaveCheck) : std::min(wake, std::max(due, m_autosaveCheck));
            if (m_renderer && m_renderer->pendingWork()) wake = wake < 0 ? glfwGetTime() + 0.25 : std::min(wake, glfwGetTime() + 0.25); // coarser copies on the way
            if (wake >= 0) glfwWaitEventsTimeout(std::max(wake - glfwGetTime(), 0.0) + 0.05);
            else glfwWaitEvents();
        }

        const auto now = clock::now();
        const float dt = std::min(std::chrono::duration<float>(now - last).count(), 0.1f);
        last = now;

        const auto t0 = clock::now();
        frame(dt);
        glFinish();
        m_uiState.frameMs = std::chrono::duration<float, std::milli>(clock::now() - t0).count();

        if (m_uiState.renderRunning || m_uiState.renderStart) screenshotCountdown = std::max(screenshotCountdown, 5); // the picture first (and its window grown)
        if (m_explodeMoving) screenshotCountdown = std::max(screenshotCountdown, 2);                                  // the exploded view opened (or shut)
        if (m_viewMoving) screenshotCountdown = std::max(screenshotCountdown, 2);                                     // the view arrived (Fit, a standard view)
        if (!m_options.screenshotPath.empty() && --screenshotCountdown == 0) {
            const bool ok = saveScreenshot(m_options.screenshotPath);
            std::printf("%s screenshot: %s\n", ok ? "Saved" : "Failed to save", m_options.screenshotPath.c_str());
            glfwSwapBuffers(m_window);
            if (ok && m_commandFailures > 0) return 3; // a --command failed
            return ok ? 0 : 1;
        }

        glfwSwapBuffers(m_window);
        if (m_framesToRender > 0) --m_framesToRender;
    }
    return 0;
}

} // namespace sicdm

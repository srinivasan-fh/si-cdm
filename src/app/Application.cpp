#include "app/Application.hpp"

#include "render/GL.hpp"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include "core/Command.hpp"
#include "core/Library.hpp"
#include "core/Physics.hpp"
#include "core/Units.hpp"
#include "ui/Theme.hpp"
#include "core/Extrude.hpp"
#include "core/Json.hpp"

#include <algorithm>
#include <filesystem>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iterator>
#include <cstdio>
#include <fstream>
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
#if defined(__APPLE__)
    glfwWindowHint(GLFW_COCOA_RETINA_FRAMEBUFFER, GLFW_TRUE);
#endif
    if (m_splash) glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE); // shown when the splash closes

    m_window = glfwCreateWindow(m_options.width, m_options.height, "SI CDM — CAD / CAM Workbench", nullptr, nullptr);
    glfwDefaultWindowHints();
    if (!m_window) return false;
    applyWindowIcon(m_window);
    glfwMakeContextCurrent(m_window);
    glfwSwapInterval(1);

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

    log("SI CDM ready. Type HELP for commands, or pick a tool on the left.", UiState::LogLine::Output);
    m_savedSignature = contentSignature(); // the demo part counts as unmodified
    if (!m_options.openPath.empty()) {
        progress(0.88f, "Opening " + std::filesystem::path(m_options.openPath).filename().string(), "Project");
        // A project opens in its own tab next to the demo; other files import into the demo.
        const FileKind kind = fileKindOf(m_options.openPath);
        runCommandLine((kind == FileKind::Project ? "OPEN \"" : "IMPORT \"") + m_options.openPath + "\"");
        if (kind != FileKind::Project) m_savedSignature = contentSignature();
    }
    refreshFileTabs();
    updateTitle();
    // Unsaved work from a session that ended without closing: offered back in frame().
    m_recoverable = findRecoveryFiles();
    m_offerRecovery = !m_recoverable.empty() && m_options.screenshotPath.empty();
    progress(0.96f, "Restoring interface", "Workspace");
    return true;
}

void Application::shutdown() {
    m_autosaver.clear(); // a clean exit: the recovery copies aren't needed
    m_splash.reset();
    m_mcpHttp.stop();
    m_renderer.reset();
    if (m_imguiReady) {
        ImGui_ImplOpenGL3_Shutdown();
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

float snap(float v, float step) { return std::round(v / step) * step; }

StandardView planeView(SketchPlane p) {
    switch (p) {
    case SketchPlane::Top: return StandardView::Top;
    case SketchPlane::Front: return StandardView::Front;
    case SketchPlane::Right: return StandardView::Right;
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
        setStatus(mode == ViewMode::Drawing2D ? std::string("2D Drawing window: ") + planeName(m_uiState.plane)
                                              : std::string("3D Model window"));
}

void Application::init2DCamera() {
    // First visit: plan view of the active plane, framed on the model.
    m_cam2D.setAspect(m_aspect);
    m_cam2D.setProjection(Projection::Orthographic);
    m_cam2D.setStandardView(planeView(m_uiState.plane), false);
    if (m_doc.scene.bounds().valid()) m_cam2D.fitToBounds(m_doc.scene.bounds(), false);
    m_2DReady = true;
}

void Application::setPlane(SketchPlane plane) {
    m_sketcher.cancel();
    const bool changed = plane != m_uiState.plane;
    m_uiState.plane = plane;
    m_doc.state.plane = plane;
    m_vp2D.gridPlane = plane;
    if (m_2DReady && changed) {
        m_cam2D.setStandardView(planeView(plane), true);
        m_cam2D.fitToBounds(m_doc.scene.bounds(), true);
    }
    if (changed) setStatus(std::string("Sketch plane: ") + planeName(plane));
}

std::optional<RayHit> Application::pick(float x, float y) const {
    const ImGuiIO& io = ImGui::GetIO();
    return m_doc.scene.raycast(camera().screenRay(x, y, io.DisplaySize.x, io.DisplaySize.y),
                               simulating() && !m_simTransforms.empty() ? &m_simTransforms : nullptr);
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

void Application::addCurve(SketchCurve curve) {
    const SketchCurve& c = m_doc.scene.sketch().add(std::move(curve));
    m_uiState.selectedCurve = static_cast<int>(m_doc.scene.sketch().curves().size()) - 1;
    m_uiState.selectedBody = m_uiState.selectedImage = -1;
    setStatus("Added " + c.name + "  " + c.measure());
    log("Added " + c.name + " on " + planeName(c.plane) + "  " + c.measure(), UiState::LogLine::Output);
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
        char at[96];
        std::snprintf(at, sizeof at, "%.4f,%.4f,%.4f", hit->point.x, hit->point.y, hit->point.z);
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
        char first[96];
        std::snprintf(first, sizeof first, "%.4f,%.4f,%.4f", m_uiState.mateFirstPoint.x, m_uiState.mateFirstPoint.y, m_uiState.mateFirstPoint.z);
        std::string line = "MATE " + m_uiState.mateTool;
        if (m_uiState.mateTool == "DISTANCE" || m_uiState.mateTool == "ANGLE") {
            char v[32];
            std::snprintf(v, sizeof v, " %g", static_cast<double>(m_uiState.mateValue));
            line += v;
        }
        line += " \"" + m_uiState.mateFirst + "\" AT " + first + " \"" + b.name + "\" AT " + at;
        m_uiState.mateFirst.clear(); // ready for the next pair
        runCommandLine(line);
        return;
    }

    if (isSketchTool(tool)) {
        if (auto c = m_sketcher.click(tool, doubleClick)) addCurve(std::move(*c));
        return;
    }

    if (isCutTool(tool)) {
        if (tool != ToolId::CutChannel && tool != ToolId::CutKnife) applyCutTool(tool, x, y);
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

    if (tool == ToolId::Select || tool == ToolId::Move) {
        // Sketch curves are thin, so they win over the bodies behind them.
        const int curve = tool == ToolId::Select ? pickCurve(x, y) : -1;
        if (curve >= 0) {
            m_uiState.selectedCurve = curve;
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
            const PlaneFrame f = planeFrame(is2D ? m_uiState.plane : SketchPlane::Top);
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
        m_doc.scene.add(std::move(body));
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
    s.refine = m_uiState.sculptRefine;
    s.detail = std::max(m_uiState.sculptDetail, 0.0f);
    m_sculptSettings = s;
    m_sculptBody = hit->body;
    m_sculptName = body.name;
    m_dabs = 0;
    const ImGuiIO& io = ImGui::GetIO();
    const Ray ray = camera().screenRay(x, y, io.DisplaySize.x, io.DisplaySize.y);
    const Vec3 p = m_sculptMesh.raycast(ray).value_or(hit->point);
    if (s.brush == SculptBrush::Grab) {
        m_sculptMesh.beginGrab(p, s);
        m_grabPoint = p;
    } else {
        m_sculptMesh.beginStroke();
        m_sculptMesh.dab(p, s);
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
    } else {
        const auto hit = m_sculptMesh.raycast(ray);
        if (!hit) return;
        // Dabs every quarter radius along the stroke, each on the surface.
        const float spacing = std::max(m_sculptSettings.radius * 0.25f, 1e-3f);
        const Vec3 from = m_lastDab;
        const float len = length(*hit - from);
        if (len < spacing) return;
        const int steps = std::min(static_cast<int>(len / spacing), 64);
        for (int i = 1; i <= steps; ++i) {
            const Vec3 q = m_sculptMesh.closestPoint(from + (*hit - from) * (static_cast<float>(i) * spacing / len));
            m_sculptMesh.dab(q, m_sculptSettings, q - m_lastDab);
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
}

void Application::cancelInteraction(bool commit) {
    if (m_sculptBody >= 0) {
        if (commit) endSculpt();
        else abortSculpt();
    }
    m_cutPath.clear();
    m_cutBody = -1;
    m_uiState.booleanFirst.clear();
    m_leftDrag = m_otherDrag = DragMode::None;
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
    case ToolId::AddBoss: {
        static const char* const kShapes[] = {"CYLINDER", "BOX", "BALL"};
        line = "BOSS " + body + at + " " + kShapes[std::clamp(u.bossShape, 0, 2)] + " DIAMETER " + mmText(u.bossDiameter) +
               " HEIGHT " + mmText(u.bossHeight);
        break;
    }
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

void Application::drawSculptOverlay() {
    const ImGuiIO& io = ImGui::GetIO();
    ImDrawList* fg = ImGui::GetForegroundDrawList();
    const ToolId tool = m_ui.toolbox().active();
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
    if (!m_brushHit || tool == ToolId::CutKnife || tool == ToolId::BooleanBodies) return;
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

    // Sketch cursor (also feeds the rulers and status bar).
    m_uiState.cursorPlane.reset();
    if (onCanvas && io.MousePos.x > -1e30f) {
        const SnapOptions so{m_uiState.snapGrid, m_uiState.snapPoints};
        if (isSketchTool(tool)) {
            if (m_sketcher.hover(m_doc.scene, cam, m_uiState.plane, io.MousePos, io.DisplaySize, so, io.KeyShift))
                m_uiState.cursorPlane = m_sketcher.cursor();
        } else {
            const PlaneFrame f = planeFrame(m_uiState.plane);
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

    // Right drag orbits (3D) / pans (2D); middle drag pans; any tool.
    if (onCanvas && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) m_otherDrag = orbitOrPan;
    if (onCanvas && ImGui::IsMouseClicked(ImGuiMouseButton_Middle)) m_otherDrag = DragMode::Pan;
    if (!ImGui::IsMouseDown(ImGuiMouseButton_Right) && !ImGui::IsMouseDown(ImGuiMouseButton_Middle))
        m_otherDrag = DragMode::None;
    if (m_otherDrag == DragMode::Orbit) cam.orbit(dx, dy);
    if (m_otherDrag == DragMode::Pan) cam.pan(dx, dy, wh);

    // Left button behaviour depends on the active tool.
    if (onCanvas && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        m_pressX = mx;
        m_pressY = my;
        m_leftDrag = DragMode::Pending;
        if (isSculptTool(tool) && beginSculpt(mx, my)) {
            m_leftDrag = m_sculptSettings.brush == SculptBrush::Grab ? DragMode::Grab : DragMode::Sculpt;
        } else if (tool == ToolId::CutChannel && pick(mx, my)) {
            const auto hit = pick(mx, my);
            m_leftDrag = DragMode::CutPath;
            m_cutBody = hit->body;
            m_cutPath = {hit->point};
        } else if (tool == ToolId::CutKnife) {
            m_leftDrag = DragMode::Knife;
            m_knifeStart = {mx, my};
        } else if (io.KeyShift && !isSketchTool(tool)) {
            m_leftDrag = DragMode::Pan;
        } else if (tool == ToolId::Orbit) {
            m_leftDrag = orbitOrPan;
        } else if (tool == ToolId::Pan) {
            m_leftDrag = DragMode::Pan;
        } else if (tool == ToolId::Zoom) {
            m_leftDrag = DragMode::Zoom;
        } else if (tool == ToolId::Move) {
            Vec2 grab{};
            if (const auto hit = pick(mx, my)) {
                m_leftDrag = DragMode::MoveBody;
                m_uiState.selectedBody = hit->body;
                m_uiState.selectedCurve = m_uiState.selectedImage = -1;
                m_movePlanePoint = hit->point;
                m_moveStartOffset = m_doc.scene.bodies()[static_cast<std::size_t>(hit->body)].offset;
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
        if (m_leftDrag == DragMode::Pending && std::hypot(mx - m_pressX, my - m_pressY) > kDragThreshold)
            m_leftDrag = isSketchTool(tool) ? DragMode::None : orbitOrPan; // dragging empty space navigates
        switch (m_leftDrag) {
        case DragMode::Orbit: cam.orbit(dx, dy); break;
        case DragMode::Pan: cam.pan(dx, dy, wh); break;
        case DragMode::Zoom: cam.zoomAt(-dy * 0.02f, m_pressX, m_pressY, io.DisplaySize.x, wh); break;
        case DragMode::MoveBody: {
            // Slide in the plane through the grab point, facing the camera.
            const int sel = m_uiState.selectedBody;
            if (sel < 0 || sel >= static_cast<int>(m_doc.scene.bodies().size())) break;
            const Ray ray = cam.screenRay(mx, my, io.DisplaySize.x, wh);
            float t = 0.0f;
            if (intersectPlane(ray, m_movePlanePoint, cam.forward(), t))
                m_doc.scene.bodies()[static_cast<std::size_t>(sel)].offset =
                    m_moveStartOffset + (ray.at(t) - m_movePlanePoint);
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
        case DragMode::CutPath: {
            const auto hit = pick(mx, my);
            const float step = std::max(m_uiState.channelWidth * 0.5f, 0.5f);
            if (hit && hit->body == m_cutBody && length(hit->point - m_cutPath.back()) >= step) m_cutPath.push_back(hit->point);
            break;
        }
        default: break;
        }
    }

    if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        if (m_leftDrag == DragMode::Pending) onCanvasClick(mx, my, false);
        else if (m_leftDrag == DragMode::Sculpt || m_leftDrag == DragMode::Grab) endSculpt();
        else if (m_leftDrag == DragMode::CutPath) finishCutDrag(ToolId::CutChannel, mx, my);
        else if (m_leftDrag == DragMode::Knife) finishCutDrag(ToolId::CutKnife, mx, my);
        m_leftDrag = DragMode::None;
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
    if (m_leftDrag == DragMode::None) m_ui.toolbox().handleShortcuts(); // not mid-stroke
    if (m_ui.toolbox().active() != before) m_sketcher.cancel();

    const bool ctrl = commandModifierHeld();
    UiCommands cmd;
    if (ImGui::IsKeyPressed(ImGuiKey_Tab, false) && !ctrl) {
        if (io.KeyShift) cmd.mode = simulating() ? ViewMode::Model3D : ViewMode::Simulation;
        else cmd.mode = is2D ? ViewMode::Model3D : simulating() ? ViewMode::Model3D : ViewMode::Drawing2D;
    }
    if (simulating() && !ctrl) { // playback keys
        if (ImGui::IsKeyPressed(ImGuiKey_Space, false)) {
            if (!m_uiState.simPlaying && m_uiState.simTime >= m_uiState.simDuration - 1e-4f) m_uiState.simTime = 0.0f;
            m_uiState.simPlaying = !m_uiState.simPlaying;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Home, false)) m_uiState.simTime = 0.0f;
        if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow)) m_uiState.simTime = std::max(m_uiState.simTime - 1.0f / 30.0f, 0.0f);
        if (ImGui::IsKeyPressed(ImGuiKey_RightArrow)) m_uiState.simTime += 1.0f / 30.0f;
    }
    auto viewKey = [&](ImGuiKey a, ImGuiKey b) { return ImGui::IsKeyPressed(a) || ImGui::IsKeyPressed(b); };
    if (viewKey(ImGuiKey_1, ImGuiKey_Keypad1)) cmd.view = ctrl ? StandardView::Back : StandardView::Front;
    if (viewKey(ImGuiKey_3, ImGuiKey_Keypad3)) cmd.view = ctrl ? StandardView::Left : StandardView::Right;
    if (viewKey(ImGuiKey_7, ImGuiKey_Keypad7)) cmd.view = ctrl ? StandardView::Bottom : StandardView::Top;
    if (viewKey(ImGuiKey_0, ImGuiKey_Keypad0)) cmd.view = StandardView::Isometric;
    if (ImGui::IsKeyPressed(ImGuiKey_F)) cmd.fit = true;
    auto togglePage = [&](int page) { // M: Model page, K: Library page of the docked panel
        m_uiState.showModelTree = !(m_uiState.showModelTree && m_uiState.dockTab == page);
        m_uiState.dockTab = page;
    };
    if (ImGui::IsKeyPressed(ImGuiKey_M, false) && !ctrl) togglePage(0);
    if (ImGui::IsKeyPressed(ImGuiKey_K, false) && !ctrl) togglePage(1);
    if (ctrl) { // Ctrl/Cmd + key belongs to the menus, not to single-key shortcuts
        if (ImGui::IsKeyPressed(ImGuiKey_Q)) cmd.quit = true;
        handleCommands(cmd);
        return;
    }
    if (isSculptTool(m_ui.toolbox().active())) { // brush size, like Photoshop and ZBrush
        if (ImGui::IsKeyPressed(ImGuiKey_LeftBracket)) m_uiState.sculptRadius = std::max(m_uiState.sculptRadius / 1.2f, 0.1f);
        if (ImGui::IsKeyPressed(ImGuiKey_RightBracket)) m_uiState.sculptRadius *= 1.2f;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_P) && !is2D) cmd.toggleProjection = true;
    if (ImGui::IsKeyPressed(ImGuiKey_G)) viewport().showGrid = !viewport().showGrid;
    if (ImGui::IsKeyPressed(ImGuiKey_W) && !is2D)
        m_vp3D.displayMode = static_cast<DisplayMode>((static_cast<int>(m_vp3D.displayMode) + 1) % 4);
    if (ImGui::IsKeyPressed(ImGuiKey_Enter) || ImGui::IsKeyPressed(ImGuiKey_KeypadEnter)) {
        if (m_extrudeCurve >= 0) applyExtrude();
        else if (auto c = m_sketcher.finish()) addCurve(std::move(*c));
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
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
        }
    }
    if ((ImGui::IsKeyPressed(ImGuiKey_Delete) || ImGui::IsKeyPressed(ImGuiKey_Backspace)) && m_leftDrag == DragMode::None) {
        auto& curves = m_doc.scene.sketch().curves();
        if (m_uiState.selectedCurve >= 0 && m_uiState.selectedCurve < static_cast<int>(curves.size())) {
            setStatus("Deleted " + curves[static_cast<std::size_t>(m_uiState.selectedCurve)].name);
            m_doc.scene.sketch().remove(static_cast<std::size_t>(m_uiState.selectedCurve));
            m_uiState.selectedCurve = -1;
        } else if (m_uiState.selectedBody >= 0 && m_uiState.selectedBody < static_cast<int>(m_doc.scene.bodies().size())) {
            setStatus("Deleted " + m_doc.scene.bodies()[static_cast<std::size_t>(m_uiState.selectedBody)].name);
            m_doc.scene.remove(static_cast<std::size_t>(m_uiState.selectedBody));
            m_uiState.selectedBody = -1;
        } else if (m_uiState.selectedImage >= 0 && m_uiState.selectedImage < static_cast<int>(m_doc.scene.images().size())) {
            setStatus("Deleted " + m_doc.scene.images()[static_cast<std::size_t>(m_uiState.selectedImage)].name);
            m_doc.scene.removeImage(static_cast<std::size_t>(m_uiState.selectedImage));
            m_uiState.selectedImage = -1;
        }
    }
    if (ctrl && ImGui::IsKeyPressed(ImGuiKey_Q)) cmd.quit = true;
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
    handleViewportInput();
    updatePrompt();
    refreshFileTabs();
    updateSheet();
    m_uiState.assembly = &m_doc.state.assembly;
    const UiCommands cmd = m_ui.draw(m_doc.scene, camera(), viewport(), m_uiState);
    handleCommands(cmd);

    // Unsaved changes: compare what would be saved with the last save.
    m_doc.dirty = contentSignature() != m_savedSignature;
    m_uiState.dirty = m_doc.dirty;
    updateTitle();
    refreshFileTabs();
    autosave();

    if (m_offerRecovery) {
        ImGui::OpenPopup("Recover Unsaved Work");
        m_offerRecovery = false;
    }
    if (ImGui::BeginPopupModal("Recover Unsaved Work", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text("SI CDM didn't close normally last time. This unsaved work was kept:");
        for (const RecoveryFile& f : m_recoverable)
            ImGui::BulletText("%s%s", f.name.c_str(), f.original.empty() ? "  (never saved)" : ("  (" + f.original + ")").c_str());
        ImGui::Spacing();
        if (ImGui::Button("Restore All", {120, 0})) {
            // Each RECOVER 1 opens the newest left and takes it off the list.
            for (std::size_t k = 0; k < m_recoverable.size(); ++k) runCommandLine("RECOVER 1");
            m_recoverable.clear();
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Discard", {90, 0})) {
            runCommandLine("RECOVER DISCARD ALL");
            m_recoverable.clear();
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Later", {90, 0}) || ImGui::IsKeyPressed(ImGuiKey_Escape)) ImGui::CloseCurrentPopup();
        ImGui::TextDisabled("Restored files open in their own tabs, still unsaved. Later: type RECOVER.");
        ImGui::EndPopup();
    }

    if (m_closeRequested) {
        ImGui::OpenPopup("Quit SI CDM?");
        m_closeRequested = false;
    }
    if (ImGui::BeginPopupModal("Quit SI CDM?", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        bool canSave = true;
        ImGui::Text("These files have unsaved changes:");
        for (int i = 0; i < static_cast<int>(m_files.size()); ++i) {
            const Document& d = fileDoc(i);
            if (!d.dirty) continue;
            ImGui::BulletText("%s", d.displayName().c_str());
            canSave = canSave && !d.path.empty();
        }
        ImGui::Spacing();
        if (!canSave) ImGui::BeginDisabled();
        if (ImGui::Button("Save All and Quit", {150, 0})) {
            saveAll();
            if (!anyFileDirty()) glfwSetWindowShouldClose(m_window, GLFW_TRUE);
            ImGui::CloseCurrentPopup();
        }
        if (!canSave) ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button("Quit Without Saving", {160, 0})) {
            glfwSetWindowShouldClose(m_window, GLFW_TRUE);
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", {90, 0}) || ImGui::IsKeyPressed(ImGuiKey_Escape)) ImGui::CloseCurrentPopup();
        if (!canSave) ImGui::TextDisabled("Some files were never saved: use File > Save As on them first.");
        ImGui::EndPopup();
    }
    const bool anim3D = m_cam3D.update(dt);
    const bool anim2D = m_cam2D.update(dt);
    if (anim3D || anim2D) m_framesToRender = std::max(m_framesToRender, 2);

    // Sketch preview and crosshair, drawn over the canvas.
    const ImGuiIO& io = ImGui::GetIO();
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
    if (m_uiState.mode == ViewMode::Model3D && m_doc.state.cam.showPaths) drawToolpathOverlay();
    if (m_uiState.mode == ViewMode::Drawing2D && !m_doc.scene.sketch().constraints().empty()) drawConstraintOverlay();
    if (!simulating() && (isSculptTool(m_ui.toolbox().active()) || isCutTool(m_ui.toolbox().active()) ||
                          isWearableTool(m_ui.toolbox().active())))
        drawSculptOverlay();
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

    int fbw = 0, fbh = 0;
    glfwGetFramebufferSize(m_window, &fbw, &fbh);
    if (fbw > 0 && fbh > 0) {
        m_aspect = static_cast<float>(fbw) / static_cast<float>(fbh);
        m_cam3D.setAspect(m_aspect);
        m_cam2D.setAspect(m_aspect);
        m_renderer->render(m_doc.scene, camera(), viewport(), fbw, fbh, m_uiState.selectedBody, m_uiState.selectedCurve,
                           m_uiState.selectedImage, simulating() ? &m_simTransforms : nullptr,
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
    m_doc.selectedCurve = m_uiState.selectedCurve;
    m_doc.selectedImage = m_uiState.selectedImage;
    m_doc.state.plane = m_uiState.plane;
    m_doc.snapGrid = m_uiState.snapGrid;
    m_doc.snapPoints = m_uiState.snapPoints;
}

void Application::syncFromDoc() {
    m_uiState.selectedBody = m_doc.selectedBody;
    m_uiState.selectedCurve = m_doc.selectedCurve;
    m_uiState.selectedImage = m_doc.selectedImage;
    m_uiState.snapGrid = m_doc.snapGrid;
    m_uiState.snapPoints = m_doc.snapPoints;
    if (m_doc.state.plane != m_uiState.plane) setPlane(m_doc.state.plane);
}

void Application::afterDocumentReplaced() {
    // New or opened document: reframe both windows and forget stale state.
    m_sketcher.cancel();
    cancelExtrude();
    // The stroke belonged to the old document: forget it without touching this one.
    m_sculptBody = -1;
    m_sculptOverrides.clear();
    m_sculptMesh = SculptMesh();
    cancelInteraction(false);
    m_cam3D.resetRoll(false);
    m_cam3D.setStandardView(StandardView::Isometric, false);
    if (m_doc.scene.bounds().valid()) m_cam3D.fitToBounds(m_doc.scene.bounds(), false);
    m_2DReady = false;
    if (m_uiState.mode == ViewMode::Drawing2D) {
        m_uiState.mode = ViewMode::Model3D;
        setMode(ViewMode::Drawing2D);
    }
    m_vp2D.gridPlane = m_doc.state.plane;
    m_uiState.plane = m_doc.state.plane;
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
            if (auto c = m_sketcher.typedPoint(tool, m_uiState.plane, *p)) addCurve(std::move(*c));
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
    std::string p = "Command:";
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
        const auto p = profileFromCurve(curves[i]);
        if (!p) continue;
        const PlaneFrame f = planeFrame(curves[i].plane);
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
    if (!profileFromCurve(c, &why)) {
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
    const auto profile = profileFromCurve(c);
    if (!profile) return;
    const ImGuiIO& io = ImGui::GetIO();
    const PlaneFrame f = planeFrame(c.plane);
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
    const auto profile = profileFromCurve(c);
    if (!profile) return;
    const PlaneFrame f = planeFrame(c.plane);
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
    m_uiState.selectedCurve = m_doc.selectedCurve;
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

void Application::autosave() {
    const double now = glfwGetTime();
    if (now < m_autosaveCheck) return;
    m_autosaveCheck = now + 1.0;
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
        if (!first || first->plane != m_uiState.plane) continue;
        Vec2 a{}, b{};
        if (!anchor(k.refs[0], &a)) continue;
        if (k.refs.size() > 1 && anchor(k.refs[1], &b)) a = {(a.x + b.x) * 0.5f, (a.y + b.y) * 0.5f};
        const auto s = projectToScreen(camera(), planeFrame(first->plane).toWorld(a), io.DisplaySize);
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
// rapids thin red.
void Application::drawToolpathOverlay() const {
    const CamJob& job = m_doc.state.cam;
    std::size_t total = 0;
    for (const CamOperation& op : job.ops)
        if (op.generated) total += op.moves.size();
    if (total == 0) return;
    const ImGuiIO& io = ImGui::GetIO();
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    dl->PushClipRect(m_ui.canvasMin(), m_ui.canvasMax(), true);
    static const ImU32 kColours[] = {IM_COL32(80, 200, 255, 220), IM_COL32(120, 230, 140, 220), IM_COL32(255, 170, 70, 220),
                                     IM_COL32(200, 140, 255, 220), IM_COL32(255, 110, 170, 220), IM_COL32(90, 230, 220, 220)};
    // Very long programs are thinned to keep the view responsive.
    const std::size_t stride = std::max<std::size_t>(1, total / 150000);
    const ImU32 rapid = IM_COL32(235, 80, 70, 130), plunge = IM_COL32(250, 220, 80, 220);
    std::size_t k = 0;
    for (std::size_t o = 0; o < job.ops.size(); ++o) {
        const CamOperation& op = job.ops[o];
        if (!op.generated || op.moves.empty()) continue;
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

void Application::setPanel(std::optional<bool> on) { m_uiState.showModelTree = on.value_or(!m_uiState.showModelTree); }

bool Application::showSheet(bool on) {
    m_uiState.showSheet = on;
    return true;
}

void Application::updateSheet() {
    if (!m_uiState.showSheet || m_doc.state.drawing.empty()) {
        m_uiState.sheet = nullptr;
        return;
    }
    const std::string text = drawingToJson(m_doc.state.drawing).dump();
    const std::uint64_t key = std::hash<std::string>{}(text) * 1000003u ^ m_doc.scene.revision();
    if (key != m_sheetKey || m_uiState.sheet == nullptr) {
        m_sheet = renderDrawing(m_doc.state.drawing, m_doc.scene, 1);
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
                  m_uiState.mode == ViewMode::Drawing2D   ? "2D Drawing"
                  : m_uiState.mode == ViewMode::Simulation ? "Simulation"
                                                           : "3D Model",
                  planeName(m_uiState.plane),
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

        if (!m_options.screenshotPath.empty() && --screenshotCountdown == 0) {
            const bool ok = saveScreenshot(m_options.screenshotPath);
            std::printf("%s screenshot: %s\n", ok ? "Saved" : "Failed to save", m_options.screenshotPath.c_str());
            glfwSwapBuffers(m_window);
            return ok ? 0 : 1;
        }

        glfwSwapBuffers(m_window);
        if (m_framesToRender > 0) --m_framesToRender;
    }
    return 0;
}

} // namespace sicdm

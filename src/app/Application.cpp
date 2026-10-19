#include "app/Application.hpp"

#include "render/GL.hpp"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include "core/Command.hpp"
#include "ui/Theme.hpp"
#include "core/Extrude.hpp"

#include <algorithm>
#include <filesystem>
#include <chrono>
#include <cmath>
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

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE); // required on macOS
    glfwWindowHint(GLFW_SAMPLES, 4);
#if defined(__APPLE__)
    glfwWindowHint(GLFW_COCOA_RETINA_FRAMEBUFFER, GLFW_TRUE);
#endif

    m_window = glfwCreateWindow(m_options.width, m_options.height, "SI CDM — CAD / CAM Workbench", nullptr, nullptr);
    if (!m_window) return false;
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

    m_renderer = std::make_unique<Renderer>();
    if (!m_renderer->valid()) {
        std::fprintf(stderr, "Failed to build viewport shaders\n");
        return false;
    }

    m_uiState.gpuName = reinterpret_cast<const char*>(glGetString(GL_RENDERER));
    m_uiState.glVersion = std::string("OpenGL ") + reinterpret_cast<const char*>(glGetString(GL_VERSION));

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
        // A project opens in its own tab next to the demo; other files import into the demo.
        const FileKind kind = fileKindOf(m_options.openPath);
        runCommandLine((kind == FileKind::Project ? "OPEN \"" : "IMPORT \"") + m_options.openPath + "\"");
        if (kind != FileKind::Project) m_savedSignature = contentSignature();
    }
    refreshFileTabs();
    updateTitle();
    return true;
}

void Application::shutdown() {
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
    m_leftDrag = m_otherDrag = DragMode::None;
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

    if (isSketchTool(tool)) {
        if (auto c = m_sketcher.click(tool, doubleClick)) addCurve(std::move(*c));
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
        std::snprintf(msg, sizeof(msg), "Placed %s at (%.0f, %.0f, %.0f) mm", body.name.c_str(), at.x, at.y, at.z);
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
        if (io.KeyShift && !isSketchTool(tool)) {
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

    if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
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
        default: break;
        }
    }

    if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        if (m_leftDrag == DragMode::Pending) onCanvasClick(mx, my, false);
        m_leftDrag = DragMode::None;
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
    m_ui.toolbox().handleShortcuts();
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
    if (ImGui::IsKeyPressed(ImGuiKey_P) && !is2D) cmd.toggleProjection = true;
    if (ImGui::IsKeyPressed(ImGuiKey_G)) viewport().showGrid = !viewport().showGrid;
    if (ImGui::IsKeyPressed(ImGuiKey_W) && !is2D)
        m_vp3D.displayMode = static_cast<DisplayMode>((static_cast<int>(m_vp3D.displayMode) + 1) % 4);
    if (ImGui::IsKeyPressed(ImGuiKey_Enter) || ImGui::IsKeyPressed(ImGuiKey_KeypadEnter)) {
        if (m_extrudeCurve >= 0) applyExtrude();
        else if (auto c = m_sketcher.finish()) addCurve(std::move(*c));
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
        if (m_extrudeCurve >= 0) {
            cancelExtrude();
            setStatus("Extrude cancelled");
        } else if (m_sketcher.busy()) {
            m_sketcher.cancel();
            setStatus("Drawing cancelled");
        } else {
            m_uiState.selectedBody = m_uiState.selectedCurve = m_uiState.selectedImage = -1;
        }
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Delete) || ImGui::IsKeyPressed(ImGuiKey_Backspace)) {
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
        m_cam3D.fitToBounds(m_doc.scene.bounds());
    }
    if (cmd.fit) camera().fitToBounds(m_doc.scene.bounds());
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
    const UiCommands cmd = m_ui.draw(m_doc.scene, camera(), viewport(), m_uiState);
    handleCommands(cmd);

    // Unsaved changes: compare what would be saved with the last save.
    m_doc.dirty = contentSignature() != m_savedSignature;
    m_uiState.dirty = m_doc.dirty;
    updateTitle();
    refreshFileTabs();

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
    if (simulating()) {
        m_simTransforms = motionTransforms(m_doc.scene, m_uiState.simTime); // after scrubbing / panel edits
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
                           m_uiState.selectedImage, simulating() ? &m_simTransforms : nullptr);
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
    m_leftDrag = m_otherDrag = DragMode::None;
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
        else if (const auto r = parseNumber(text); r && tool == ToolId::Circle && m_sketcher.stage() == 1 && *r > 0)
            p = Vec2{m_sketcher.lastPoint()->x + *r, m_sketcher.lastPoint()->y}; // typed radius
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
        if (const auto d = parseNumber(text)) {
            log(text, UiState::LogLine::Input);
            m_uiState.extrudeDistance = *d;
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
    for (const Body& b : s.bodies()) {
        mixStr(b.name);
        mixStr(b.kind);
        mix(b.params.data(), b.params.size() * sizeof(float));
        mix(&b.offset, sizeof(b.offset));
        mix(&b.color, sizeof(b.color));
        mix(&b.visible, sizeof(b.visible));
        mix(&b.scale, sizeof(b.scale));
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
        const int e[] = {static_cast<int>(m.kind), static_cast<int>(m.mode), m.enabled ? 1 : 0};
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
        p = m_extrudeCurve < 0 ? "EXTRUDE  pick a closed profile:" : "EXTRUDE  depth (mm), drag or type; click / Enter:";
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
    std::snprintf(depth, sizeof(depth), "%g", static_cast<double>(m_uiState.extrudeDistance));
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
    char label[64];
    std::snprintf(label, sizeof(label), m_uiState.extrudeSymmetric ? "D %.1f mm (symmetric)" : "D %.1f mm",
                  static_cast<double>(d));
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
    m_leftDrag = m_otherDrag = DragMode::None;
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
    m_leftDrag = m_otherDrag = DragMode::None;
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
    m_files.erase(m_files.begin() + index);
    if (m_activeFile > index) --m_activeFile;
    updateTitle();
    log("Closed " + name, UiState::LogLine::Output);
    setStatus("Closed " + name);
    m_framesToRender = std::max(m_framesToRender, 3);
    return true;
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
    m_simTransforms = motionTransforms(m_doc.scene, st.simTime);
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
    std::vector<Group> groups(motions.size());
    for (std::size_t k = 0; k < motions.size(); ++k) {
        AABB box;
        for (std::size_t i = 0; i < bodies.size(); ++i)
            if (motions[k].moves(bodies[i].name)) {
                if (groups[k].body < 0) groups[k].body = static_cast<int>(i);
                box.expand(bodies[i].bounds());
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
                const auto q = scr(bodyMotionTransform(scene, body, ts).transformPoint(p));
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
            const ImU32 c = toU32(motionColor(k));
            const Mat4 outer = bodyMotionTransform(scene, static_cast<std::size_t>(groups[k].body), t, k);
            const Vec3 dir = normalize(outer.transformDir(m.axis));
            const float r = std::max(groups[k].radius * 0.6f, 24.0f * wpp);
            char text[96];
            std::snprintf(text, sizeof(text), "%s  %.1f %s  %.0f %s", m.name.c_str(), static_cast<double>(motionValue(scene, m, t)),
                          m.unit(), static_cast<double>(motionRate(scene, m, t)), m.rateUnit());
            if (m.kind == MotionKind::Circular) {
                const Vec3 pivot = m.eachCentre ? bodies[static_cast<std::size_t>(groups[k].body)].bounds().center() : m.origin;
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
                const Mat4 full = bodyMotionTransform(scene, static_cast<std::size_t>(groups[k].body), t);
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
            const Mat4 tf = bodyMotionTransform(scene, body, t);
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
        if (motions.empty())
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

void Application::fit() { camera().fitToBounds(m_doc.scene.bounds()); }

void Application::zoomStep(float steps) { camera().zoom(steps); }

void Application::home() {
    UiCommands c;
    c.home = true;
    c.mode = ViewMode::Model3D;
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

    using clock = std::chrono::steady_clock;
    auto last = clock::now();
    int screenshotCountdown = 5;

    while (!glfwWindowShouldClose(m_window)) {
        if (m_framesToRender > 0 || !m_options.screenshotPath.empty()) glfwPollEvents();
        else if (!m_uiState.statusMessage.empty()) glfwWaitEventsTimeout(std::max(m_statusUntil - glfwGetTime(), 0.0) + 0.05);
        else glfwWaitEvents(); // idle: zero CPU/GPU until the user does something

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

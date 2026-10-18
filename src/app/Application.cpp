#include "app/Application.hpp"

#include "render/GL.hpp"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <algorithm>
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

void applyTheme() {
    ImGui::StyleColorsDark();
    ImGuiStyle& s = ImGui::GetStyle();
    s.WindowRounding = 6.0f;
    s.FrameRounding = 4.0f;
    s.GrabRounding = 4.0f;
    s.PopupRounding = 4.0f;
    s.WindowBorderSize = 1.0f;
    s.FramePadding = {8, 4};
    s.ItemSpacing = {8, 6};

    ImVec4* c = s.Colors;
    c[ImGuiCol_WindowBg] = {0.11f, 0.12f, 0.14f, 1.00f};
    c[ImGuiCol_MenuBarBg] = {0.09f, 0.10f, 0.12f, 1.00f};
    c[ImGuiCol_PopupBg] = {0.10f, 0.11f, 0.13f, 0.98f};
    c[ImGuiCol_Border] = {0.24f, 0.26f, 0.30f, 0.60f};
    c[ImGuiCol_TitleBg] = {0.10f, 0.11f, 0.13f, 1.00f};
    c[ImGuiCol_TitleBgActive] = {0.13f, 0.15f, 0.18f, 1.00f};
    c[ImGuiCol_FrameBg] = {0.17f, 0.18f, 0.21f, 1.00f};
    c[ImGuiCol_FrameBgHovered] = {0.22f, 0.24f, 0.28f, 1.00f};
    c[ImGuiCol_Button] = {0.18f, 0.20f, 0.24f, 1.00f};
    c[ImGuiCol_ButtonHovered] = {0.20f, 0.42f, 0.72f, 1.00f};
    c[ImGuiCol_ButtonActive] = {0.16f, 0.36f, 0.64f, 1.00f};
    c[ImGuiCol_Header] = {0.20f, 0.38f, 0.64f, 0.55f};
    c[ImGuiCol_HeaderHovered] = {0.22f, 0.44f, 0.74f, 0.70f};
    c[ImGuiCol_HeaderActive] = {0.22f, 0.44f, 0.74f, 1.00f};
    c[ImGuiCol_CheckMark] = {0.40f, 0.68f, 1.00f, 1.00f};
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
    applyTheme();
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

    m_scene = Scene::makeDemo();
    m_renderer->upload(m_scene);

    int fbw = 0, fbh = 0;
    glfwGetFramebufferSize(m_window, &fbw, &fbh);
    m_aspect = fbh > 0 ? static_cast<float>(fbw) / static_cast<float>(fbh) : 1.0f;
    m_cam3D.setAspect(m_aspect);
    m_cam3D.setStandardView(StandardView::Isometric, false);
    m_cam3D.fitToBounds(m_scene.bounds(), false);
    m_cam3D.zoom(-1.5f); // leave some breathing room on the landing page

    // 2D drawing window: flat drafting background, hidden-line bodies,
    // sketches drawn over everything on the active plane.
    m_vp2D.displayMode = DisplayMode::HiddenLine;
    m_vp2D.backgroundTop = m_vp2D.backgroundBottom = {0.105f, 0.115f, 0.135f};
    m_vp2D.edgeColor = {0.78f, 0.81f, 0.86f};
    m_vp2D.gridStrength = 0.8f;
    m_vp2D.sketchOnTop = true;

    if (m_options.start2D) {
        m_uiState.showWelcome = false;
        setMode(ViewMode::Drawing2D);
    }
    return true;
}

void Application::shutdown() {
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

const Vec3 kPalette[] = {{0.72f, 0.75f, 0.80f}, {0.82f, 0.62f, 0.32f}, {0.45f, 0.60f, 0.78f},
                         {0.38f, 0.70f, 0.52f}, {0.78f, 0.45f, 0.45f}, {0.65f, 0.55f, 0.80f}};

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

// Builds a default-sized primitive around the origin; Body::offset positions it.
Body makePrimitive(ToolId tool, int serial) {
    using namespace primitives;
    Body b;
    switch (tool) {
    case ToolId::Block: b.name = "Block"; b.mesh = box({-20, -20, 0}, {20, 20, 40}); break;
    case ToolId::Wedge: b.name = "Wedge"; b.mesh = wedge({-20, -20, 0}, {20, 20, 30}); break;
    case ToolId::Pyramid: b.name = "Pyramid"; b.mesh = pyramid({0, 0, 0}, 25, 35, 4); break;
    case ToolId::Prism: b.name = "Prism"; b.mesh = prism({0, 0, 0}, 20, 40, 6); break;
    case ToolId::Cylinder: b.name = "Cylinder"; b.mesh = cylinder({0, 0, 0}, 15, 40); break;
    case ToolId::Cone: b.name = "Cone"; b.mesh = cone({0, 0, 0}, 20, 0, 40); break;
    case ToolId::Sphere: b.name = "Sphere"; b.mesh = sphere({0, 0, 20}, 20); break;
    case ToolId::Torus: b.name = "Torus"; b.mesh = torus({0, 0, 7}, 25, 7); break;
    default: break;
    }
    b.name += " " + std::to_string(serial);
    b.color = kPalette[static_cast<std::size_t>(serial) % std::size(kPalette)];
    return b;
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
    if (mode == ViewMode::Drawing2D && !m_2DReady) {
        // First visit: plan view of the active plane, framed on the model.
        m_cam2D.setAspect(m_aspect);
        m_cam2D.setProjection(Projection::Orthographic);
        m_cam2D.setStandardView(planeView(m_uiState.plane), false);
        m_cam2D.fitToBounds(m_scene.bounds(), false);
        m_2DReady = true;
    }
    setStatus(mode == ViewMode::Drawing2D ? std::string("2D Drawing window: ") + planeName(m_uiState.plane)
                                          : std::string("3D Model window"));
}

void Application::setPlane(SketchPlane plane) {
    m_sketcher.cancel();
    const bool changed = plane != m_uiState.plane;
    m_uiState.plane = plane;
    m_vp2D.gridPlane = plane;
    if (m_2DReady && changed) {
        m_cam2D.setStandardView(planeView(plane), true);
        m_cam2D.fitToBounds(m_scene.bounds(), true);
    }
    if (changed) setStatus(std::string("Sketch plane: ") + planeName(plane));
}

std::optional<RayHit> Application::pick(float x, float y) const {
    const ImGuiIO& io = ImGui::GetIO();
    return m_scene.raycast(camera().screenRay(x, y, io.DisplaySize.x, io.DisplaySize.y));
}

int Application::pickCurve(float x, float y) const {
    const ImGuiIO& io = ImGui::GetIO();
    const auto& curves = m_scene.sketch().curves();
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
    const SketchCurve& c = m_scene.sketch().add(std::move(curve));
    m_uiState.selectedCurve = static_cast<int>(m_scene.sketch().curves().size()) - 1;
    m_uiState.selectedBody = -1;
    setStatus("Added " + c.name + "  " + c.measure());
}

void Application::onCanvasClick(float x, float y, bool doubleClick) {
    const ToolId tool = m_ui.toolbox().active();
    const ToolInfo& info = toolInfo(tool);
    const bool is2D = m_uiState.mode == ViewMode::Drawing2D;

    if (isSketchTool(tool)) {
        if (auto c = m_sketcher.click(tool, doubleClick)) addCurve(std::move(*c));
        return;
    }

    if (tool == ToolId::Select || tool == ToolId::Move) {
        // Sketch curves are thin, so they win over the bodies behind them.
        const int curve = tool == ToolId::Select ? pickCurve(x, y) : -1;
        if (curve >= 0) {
            m_uiState.selectedCurve = curve;
            m_uiState.selectedBody = -1;
            setStatus("Selected " + m_scene.sketch().curves()[static_cast<std::size_t>(curve)].name);
            return;
        }
        const auto hit = pick(x, y);
        m_uiState.selectedBody = hit ? hit->body : -1;
        m_uiState.selectedCurve = -1;
        if (hit) setStatus("Selected " + m_scene.bodies()[static_cast<std::size_t>(hit->body)].name);
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
        Body body = makePrimitive(tool, ++m_primitiveSerial);
        body.offset = at;
        char msg[160];
        std::snprintf(msg, sizeof(msg), "Placed %s at (%.0f, %.0f, %.0f) mm", body.name.c_str(), at.x, at.y, at.z);
        m_scene.add(std::move(body));
        m_uiState.selectedBody = static_cast<int>(m_scene.bodies().size()) - 1;
        m_uiState.selectedCurve = -1;
        setStatus(msg);
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
    const ToolId tool = m_ui.toolbox().active();
    OrbitCamera& cam = camera();
    // In the 2D window the view never tilts: every "orbit" gesture pans.
    const DragMode orbitOrPan = is2D ? DragMode::Pan : DragMode::Orbit;

    // Sketch cursor (also feeds the rulers and status bar).
    m_uiState.cursorPlane.reset();
    if (onCanvas && io.MousePos.x > -1e30f) {
        const SnapOptions so{m_uiState.snapGrid, m_uiState.snapPoints};
        if (isSketchTool(tool)) {
            if (m_sketcher.hover(m_scene, cam, m_uiState.plane, io.MousePos, io.DisplaySize, so, io.KeyShift))
                m_uiState.cursorPlane = m_sketcher.cursor();
        } else {
            const PlaneFrame f = planeFrame(m_uiState.plane);
            const Ray r = cam.screenRay(mx, my, io.DisplaySize.x, wh);
            const float d = dot(r.dir, f.normal);
            if (std::fabs(d) > 0.05f) m_uiState.cursorPlane = f.toPlane(r.at(dot(f.origin - r.origin, f.normal) / d));
        }
    }

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
            if (const auto hit = pick(mx, my)) {
                m_leftDrag = DragMode::MoveBody;
                m_uiState.selectedBody = hit->body;
                m_uiState.selectedCurve = -1;
                m_movePlanePoint = hit->point;
                m_moveStartOffset = m_scene.bodies()[static_cast<std::size_t>(hit->body)].offset;
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
            if (sel < 0 || sel >= static_cast<int>(m_scene.bodies().size())) break;
            const Ray ray = cam.screenRay(mx, my, io.DisplaySize.x, wh);
            float t = 0.0f;
            if (intersectPlane(ray, m_movePlanePoint, cam.forward(), t))
                m_scene.bodies()[static_cast<std::size_t>(sel)].offset =
                    m_moveStartOffset + (ray.at(t) - m_movePlanePoint);
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

    const bool ctrl = io.KeyCtrl || io.KeySuper;
    UiCommands cmd;
    if (ImGui::IsKeyPressed(ImGuiKey_Tab, false))
        cmd.mode = is2D ? ViewMode::Model3D : ViewMode::Drawing2D;
    auto viewKey = [&](ImGuiKey a, ImGuiKey b) { return ImGui::IsKeyPressed(a) || ImGui::IsKeyPressed(b); };
    if (viewKey(ImGuiKey_1, ImGuiKey_Keypad1)) cmd.view = ctrl ? StandardView::Back : StandardView::Front;
    if (viewKey(ImGuiKey_3, ImGuiKey_Keypad3)) cmd.view = ctrl ? StandardView::Left : StandardView::Right;
    if (viewKey(ImGuiKey_7, ImGuiKey_Keypad7)) cmd.view = ctrl ? StandardView::Bottom : StandardView::Top;
    if (viewKey(ImGuiKey_0, ImGuiKey_Keypad0)) cmd.view = StandardView::Isometric;
    if (ImGui::IsKeyPressed(ImGuiKey_F)) cmd.fit = true;
    if (ImGui::IsKeyPressed(ImGuiKey_P) && !is2D) cmd.toggleProjection = true;
    if (ImGui::IsKeyPressed(ImGuiKey_G)) viewport().showGrid = !viewport().showGrid;
    if (ImGui::IsKeyPressed(ImGuiKey_W) && !is2D)
        m_vp3D.displayMode = static_cast<DisplayMode>((static_cast<int>(m_vp3D.displayMode) + 1) % 4);
    if (ImGui::IsKeyPressed(ImGuiKey_Enter) || ImGui::IsKeyPressed(ImGuiKey_KeypadEnter))
        if (auto c = m_sketcher.finish()) addCurve(std::move(*c));
    if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
        if (m_sketcher.busy()) {
            m_sketcher.cancel();
            setStatus("Drawing cancelled");
        } else {
            m_uiState.selectedBody = m_uiState.selectedCurve = -1;
        }
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Delete) || ImGui::IsKeyPressed(ImGuiKey_Backspace)) {
        auto& curves = m_scene.sketch().curves();
        if (m_uiState.selectedCurve >= 0 && m_uiState.selectedCurve < static_cast<int>(curves.size())) {
            setStatus("Deleted " + curves[static_cast<std::size_t>(m_uiState.selectedCurve)].name);
            m_scene.sketch().remove(static_cast<std::size_t>(m_uiState.selectedCurve));
            m_uiState.selectedCurve = -1;
        } else if (m_uiState.selectedBody >= 0 && m_uiState.selectedBody < static_cast<int>(m_scene.bodies().size())) {
            setStatus("Deleted " + m_scene.bodies()[static_cast<std::size_t>(m_uiState.selectedBody)].name);
            m_scene.remove(static_cast<std::size_t>(m_uiState.selectedBody));
            m_uiState.selectedBody = -1;
        }
    }
    if (ctrl && ImGui::IsKeyPressed(ImGuiKey_Q)) cmd.quit = true;
    handleCommands(cmd);
}

void Application::handleCommands(const UiCommands& cmd) {
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
        m_cam3D.fitToBounds(m_scene.bounds());
    }
    if (cmd.fit) camera().fitToBounds(m_scene.bounds());
    if (cmd.toggleProjection && !is2D)
        m_cam3D.setProjection(m_cam3D.projection() == Projection::Perspective ? Projection::Orthographic
                                                                               : Projection::Perspective);
    if (cmd.quit) glfwSetWindowShouldClose(m_window, GLFW_TRUE);
}

void Application::frame(float dt) {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    handleViewportInput();
    const UiCommands cmd = m_ui.draw(m_scene, camera(), viewport(), m_uiState);
    handleCommands(cmd);
    const bool anim3D = m_cam3D.update(dt);
    const bool anim2D = m_cam2D.update(dt);
    if (anim3D || anim2D) m_framesToRender = std::max(m_framesToRender, 2);

    // Sketch preview and crosshair, drawn over the canvas.
    const ImGuiIO& io = ImGui::GetIO();
    if (isSketchTool(m_ui.toolbox().active()) && !io.WantCaptureMouse) {
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
    // Held widgets (a long press on a tool slot) need frames to time out, and a
    // newly opened popup needs a second frame to size itself before it shows.
    if (ImGui::IsAnyItemActive() || ImGui::IsPopupOpen(nullptr, ImGuiPopupFlags_AnyPopupId))
        m_framesToRender = std::max(m_framesToRender, 2);
    if (!m_uiState.statusMessage.empty() && glfwGetTime() > m_statusUntil) m_uiState.statusMessage.clear();
    m_renderer->sync(m_scene);

    int fbw = 0, fbh = 0;
    glfwGetFramebufferSize(m_window, &fbw, &fbh);
    if (fbw > 0 && fbh > 0) {
        m_aspect = static_cast<float>(fbw) / static_cast<float>(fbh);
        m_cam3D.setAspect(m_aspect);
        m_cam2D.setAspect(m_aspect);
        m_renderer->render(m_scene, camera(), viewport(), fbw, fbh, m_uiState.selectedBody, m_uiState.selectedCurve);
    }

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
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

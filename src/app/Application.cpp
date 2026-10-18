#include "app/Application.hpp"

#include "render/GL.hpp"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

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
    m_camera.setAspect(fbh > 0 ? static_cast<float>(fbw) / static_cast<float>(fbh) : 1.0f);
    m_camera.setStandardView(StandardView::Isometric, false);
    m_camera.fitToBounds(m_scene.bounds(), false);
    m_camera.zoom(-1.5f); // leave some breathing room on the landing page
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
constexpr float kPlaceSnap = 5.0f;     // mm grid snap when placing on the ground plane

const Vec3 kPalette[] = {{0.72f, 0.75f, 0.80f}, {0.82f, 0.62f, 0.32f}, {0.45f, 0.60f, 0.78f},
                         {0.38f, 0.70f, 0.52f}, {0.78f, 0.45f, 0.45f}, {0.65f, 0.55f, 0.80f}};

float snap(float v, float step) { return std::round(v / step) * step; }

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

std::optional<RayHit> Application::pick(float x, float y) const {
    const ImGuiIO& io = ImGui::GetIO();
    return m_scene.raycast(m_camera.screenRay(x, y, io.DisplaySize.x, io.DisplaySize.y));
}

void Application::onCanvasClick(float x, float y) {
    const ToolId tool = m_ui.toolbox().active();
    const ToolInfo& info = toolInfo(tool);
    const auto hit = pick(x, y);

    if (tool == ToolId::Select || tool == ToolId::Move) {
        m_uiState.selectedBody = hit ? hit->body : -1;
        if (hit) setStatus("Selected " + m_scene.bodies()[static_cast<std::size_t>(hit->body)].name);
        return;
    }

    if (isPrimitiveTool(tool)) {
        Vec3 at;
        if (hit) {
            at = hit->point; // stack on the surface that was clicked
        } else {
            const ImGuiIO& io = ImGui::GetIO();
            const Ray ray = m_camera.screenRay(x, y, io.DisplaySize.x, io.DisplaySize.y);
            float t = 0.0f;
            if (!intersectPlane(ray, {0, 0, 0}, {0, 0, 1}, t)) {
                setStatus("Click on the grid or a body to place it");
                return;
            }
            const Vec3 p = ray.at(t);
            at = {snap(p.x, kPlaceSnap), snap(p.y, kPlaceSnap), 0.0f};
        }
        Body body = makePrimitive(tool, ++m_primitiveSerial);
        body.offset = at;
        char msg[160];
        std::snprintf(msg, sizeof(msg), "Placed %s at (%.0f, %.0f, %.0f) mm", body.name.c_str(), at.x, at.y, at.z);
        m_scene.add(std::move(body));
        m_uiState.selectedBody = static_cast<int>(m_scene.bodies().size()) - 1;
        setStatus(msg);
        return;
    }

    if (!info.available) setStatus(std::string(info.name) + " is planned for a later milestone");
}

void Application::handleViewportInput() {
    ImGuiIO& io = ImGui::GetIO();
    const float mx = io.MousePos.x, my = io.MousePos.y;
    const float dx = io.MouseDelta.x, dy = io.MouseDelta.y;
    const float wh = io.DisplaySize.y;
    const bool onCanvas = !io.WantCaptureMouse;
    const ToolId tool = m_ui.toolbox().active();

    // Right drag orbits and middle drag pans whatever tool is active.
    if (onCanvas && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) m_otherDrag = DragMode::Orbit;
    if (onCanvas && ImGui::IsMouseClicked(ImGuiMouseButton_Middle)) m_otherDrag = DragMode::Pan;
    if (!ImGui::IsMouseDown(ImGuiMouseButton_Right) && !ImGui::IsMouseDown(ImGuiMouseButton_Middle))
        m_otherDrag = DragMode::None;
    if (m_otherDrag == DragMode::Orbit) m_camera.orbit(dx, dy);
    if (m_otherDrag == DragMode::Pan) m_camera.pan(dx, dy, wh);

    // Left button behaviour depends on the active tool.
    if (onCanvas && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        m_pressX = mx;
        m_pressY = my;
        m_leftDrag = DragMode::Pending;
        if (io.KeyShift) {
            m_leftDrag = DragMode::Pan;
        } else if (tool == ToolId::Orbit) {
            m_leftDrag = DragMode::Orbit;
        } else if (tool == ToolId::Pan) {
            m_leftDrag = DragMode::Pan;
        } else if (tool == ToolId::Zoom) {
            m_leftDrag = DragMode::Zoom;
        } else if (tool == ToolId::Move) {
            if (const auto hit = pick(mx, my)) {
                m_leftDrag = DragMode::MoveBody;
                m_uiState.selectedBody = hit->body;
                m_movePlanePoint = hit->point;
                m_moveStartOffset = m_scene.bodies()[static_cast<std::size_t>(hit->body)].offset;
            }
        }
    }

    if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        if (m_leftDrag == DragMode::Pending &&
            std::hypot(mx - m_pressX, my - m_pressY) > kDragThreshold)
            m_leftDrag = DragMode::Orbit; // dragging empty space always navigates
        switch (m_leftDrag) {
        case DragMode::Orbit: m_camera.orbit(dx, dy); break;
        case DragMode::Pan: m_camera.pan(dx, dy, wh); break;
        case DragMode::Zoom: m_camera.zoom(-dy * 0.02f); break;
        case DragMode::MoveBody: {
            // Slide in the plane through the grab point, facing the camera.
            const int sel = m_uiState.selectedBody;
            if (sel < 0 || sel >= static_cast<int>(m_scene.bodies().size())) break;
            const Ray ray = m_camera.screenRay(mx, my, io.DisplaySize.x, wh);
            float t = 0.0f;
            if (intersectPlane(ray, m_movePlanePoint, m_camera.forward(), t))
                m_scene.bodies()[static_cast<std::size_t>(sel)].offset =
                    m_moveStartOffset + (ray.at(t) - m_movePlanePoint);
            break;
        }
        default: break;
        }
    }

    if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        if (m_leftDrag == DragMode::Pending) onCanvasClick(mx, my);
        m_leftDrag = DragMode::None;
    }

    if (m_pendingScroll != 0.0f) {
        m_camera.zoom(m_pendingScroll);
        m_pendingScroll = 0.0f;
    }

    if (io.WantCaptureKeyboard) return;
    m_ui.toolbox().handleShortcuts();
    const bool ctrl = io.KeyCtrl || io.KeySuper;
    UiCommands cmd;
    if (ImGui::IsKeyPressed(ImGuiKey_1) || ImGui::IsKeyPressed(ImGuiKey_Keypad1))
        cmd.view = ctrl ? StandardView::Back : StandardView::Front;
    if (ImGui::IsKeyPressed(ImGuiKey_3) || ImGui::IsKeyPressed(ImGuiKey_Keypad3))
        cmd.view = ctrl ? StandardView::Left : StandardView::Right;
    if (ImGui::IsKeyPressed(ImGuiKey_7) || ImGui::IsKeyPressed(ImGuiKey_Keypad7))
        cmd.view = ctrl ? StandardView::Bottom : StandardView::Top;
    if (ImGui::IsKeyPressed(ImGuiKey_0) || ImGui::IsKeyPressed(ImGuiKey_Keypad0))
        cmd.view = StandardView::Isometric;
    if (ImGui::IsKeyPressed(ImGuiKey_F)) cmd.fit = true;
    if (ImGui::IsKeyPressed(ImGuiKey_P)) cmd.toggleProjection = true;
    if (ImGui::IsKeyPressed(ImGuiKey_G)) m_viewport.showGrid = !m_viewport.showGrid;
    if (ImGui::IsKeyPressed(ImGuiKey_W))
        m_viewport.displayMode = static_cast<DisplayMode>((static_cast<int>(m_viewport.displayMode) + 1) % 3);
    if (ImGui::IsKeyPressed(ImGuiKey_Escape)) m_uiState.selectedBody = -1;
    if ((ImGui::IsKeyPressed(ImGuiKey_Delete) || ImGui::IsKeyPressed(ImGuiKey_Backspace)) &&
        m_uiState.selectedBody >= 0 && m_uiState.selectedBody < static_cast<int>(m_scene.bodies().size())) {
        setStatus("Deleted " + m_scene.bodies()[static_cast<std::size_t>(m_uiState.selectedBody)].name);
        m_scene.remove(static_cast<std::size_t>(m_uiState.selectedBody));
        m_uiState.selectedBody = -1;
    }
    if (ctrl && ImGui::IsKeyPressed(ImGuiKey_Q)) cmd.quit = true;
    handleCommands(cmd);
}

void Application::handleCommands(const UiCommands& cmd) {
    if (cmd.view) m_camera.setStandardView(*cmd.view);
    if (cmd.fit) m_camera.fitToBounds(m_scene.bounds());
    if (cmd.toggleProjection)
        m_camera.setProjection(m_camera.projection() == Projection::Perspective ? Projection::Orthographic
                                                                                 : Projection::Perspective);
    if (cmd.quit) glfwSetWindowShouldClose(m_window, GLFW_TRUE);
}

void Application::frame(float dt) {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    handleViewportInput();
    const UiCommands cmd = m_ui.draw(m_scene, m_camera, m_viewport, m_uiState);
    handleCommands(cmd);
    if (m_camera.update(dt)) m_framesToRender = std::max(m_framesToRender, 2);
    // Held widgets (a long press on a tool slot) need frames to time out, and a
    // newly opened popup needs a second frame to size itself before it shows.
    if (ImGui::IsAnyItemActive() || ImGui::IsPopupOpen(nullptr, ImGuiPopupFlags_AnyPopupId))
        m_framesToRender = std::max(m_framesToRender, 2);
    if (!m_uiState.statusMessage.empty() && glfwGetTime() > m_statusUntil) m_uiState.statusMessage.clear();
    m_renderer->sync(m_scene);

    int fbw = 0, fbh = 0;
    glfwGetFramebufferSize(m_window, &fbw, &fbh);
    if (fbw > 0 && fbh > 0) {
        m_camera.setAspect(static_cast<float>(fbw) / static_cast<float>(fbh));
        m_renderer->render(m_scene, m_camera, m_viewport, fbw, fbh, m_uiState.selectedBody);
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

#include "app/Application.hpp"

#include "render/GL.hpp"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <chrono>
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

void Application::handleViewportInput() {
    ImGuiIO& io = ImGui::GetIO();
    double x = 0, y = 0;
    glfwGetCursorPos(m_window, &x, &y);
    const float dx = static_cast<float>(x - m_lastX);
    const float dy = static_cast<float>(y - m_lastY);
    m_lastX = x;
    m_lastY = y;

    const bool left = glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    const bool middle = glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    const bool right = glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    const bool shift = glfwGetKey(m_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                       glfwGetKey(m_window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;

    // A drag that starts on the canvas keeps ownership until release,
    // even if the cursor passes over a panel.
    if (!left && !middle && !right) m_dragging = false;
    else if (!m_dragging && !io.WantCaptureMouse) m_dragging = true;

    if (m_dragging && (dx != 0.0f || dy != 0.0f)) {
        int wh = 0;
        glfwGetWindowSize(m_window, nullptr, &wh);
        if (middle || right || (left && shift)) m_camera.pan(dx, dy, static_cast<float>(wh));
        else if (left) m_camera.orbit(dx, dy);
    }

    if (m_pendingScroll != 0.0f) {
        m_camera.zoom(m_pendingScroll);
        m_pendingScroll = 0.0f;
    }

    if (io.WantCaptureKeyboard) return;
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

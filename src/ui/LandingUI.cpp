#include "ui/LandingUI.hpp"

#include <imgui.h>

#include <algorithm>
#include <array>
#include <cstdio>

namespace sicdm {

namespace {

constexpr ImU32 kAxisColors[3] = {IM_COL32(230, 80, 80, 255), IM_COL32(110, 205, 95, 255),
                                  IM_COL32(80, 140, 240, 255)};

const char* projectionLabel(Projection p) {
    return p == Projection::Perspective ? "Perspective" : "Orthographic";
}

const char* displayModeLabel(DisplayMode m) {
    switch (m) {
    case DisplayMode::Shaded: return "Shaded";
    case DisplayMode::ShadedWithEdges: return "Shaded + Edges";
    case DisplayMode::Wireframe: return "Wireframe";
    }
    return "";
}

struct ViewEntry {
    const char* label;
    const char* shortcut;
    StandardView view;
};

constexpr std::array<ViewEntry, 7> kViews = {{
    {"Front", "1", StandardView::Front},
    {"Back", "Ctrl+1", StandardView::Back},
    {"Right", "3", StandardView::Right},
    {"Left", "Ctrl+3", StandardView::Left},
    {"Top", "7", StandardView::Top},
    {"Bottom", "Ctrl+7", StandardView::Bottom},
    {"Isometric", "0", StandardView::Isometric},
}};

} // namespace

UiCommands LandingUI::draw(Scene& scene, const OrbitCamera& cam, ViewportSettings& vp, UiState& st) {
    UiCommands cmd;
    drawMenuBar(cmd, cam, vp, st);
    drawToolbar(cmd, cam, vp);
    drawStatusBar(scene, cam, st);
    if (st.showModelTree) drawModelTree(scene, st);
    drawTriad(cmd, cam);
    if (st.showWelcome) drawWelcome(st);
    if (st.showControls) drawControlsWindow(st);
    if (st.showAbout) drawAboutWindow(st);
    return cmd;
}

void LandingUI::drawMenuBar(UiCommands& cmd, const OrbitCamera& cam, ViewportSettings& vp, UiState& st) {
    if (!ImGui::BeginMainMenuBar()) return;
    m_menuHeight = ImGui::GetWindowSize().y;

    if (ImGui::BeginMenu("File")) {
        ImGui::MenuItem("New Part", "Cmd+N", false, false);
        ImGui::MenuItem("New Assembly", nullptr, false, false);
        ImGui::MenuItem("Open...", "Cmd+O", false, false);
        ImGui::Separator();
        if (ImGui::BeginMenu("Import")) {
            ImGui::MenuItem("STEP (.step, .stp)", nullptr, false, false);
            ImGui::MenuItem("IGES (.iges, .igs)", nullptr, false, false);
            ImGui::MenuItem("STL (.stl)", nullptr, false, false);
            ImGui::EndMenu();
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Quit", "Cmd+Q")) cmd.quit = true;
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("View")) {
        for (const ViewEntry& v : kViews)
            if (ImGui::MenuItem(v.label, v.shortcut)) cmd.view = v.view;
        ImGui::Separator();
        if (ImGui::MenuItem("Zoom to Fit", "F")) cmd.fit = true;
        if (ImGui::MenuItem("Orthographic", "P", cam.projection() == Projection::Orthographic))
            cmd.toggleProjection = true;
        ImGui::Separator();
        for (DisplayMode m : {DisplayMode::Shaded, DisplayMode::ShadedWithEdges, DisplayMode::Wireframe})
            if (ImGui::MenuItem(displayModeLabel(m), nullptr, vp.displayMode == m)) vp.displayMode = m;
        ImGui::Separator();
        ImGui::MenuItem("Grid", "G", &vp.showGrid);
        ImGui::MenuItem("Model Tree", nullptr, &st.showModelTree);
        ImGui::MenuItem("Welcome", nullptr, &st.showWelcome);
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Design")) {
        ImGui::MenuItem("Sketch", nullptr, false, false);
        ImGui::MenuItem("Extrude", nullptr, false, false);
        ImGui::MenuItem("Revolve", nullptr, false, false);
        ImGui::MenuItem("Fillet / Chamfer", nullptr, false, false);
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Manufacture")) {
        ImGui::MenuItem("New CAM Setup", nullptr, false, false);
        ImGui::MenuItem("2D Contour", nullptr, false, false);
        ImGui::MenuItem("3D Adaptive Clearing", nullptr, false, false);
        ImGui::MenuItem("Simulate Toolpath", nullptr, false, false);
        ImGui::MenuItem("Post Process (G-code)", nullptr, false, false);
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Help")) {
        ImGui::MenuItem("Viewport Controls", nullptr, &st.showControls);
        ImGui::MenuItem("About SI CDM", nullptr, &st.showAbout);
        ImGui::EndMenu();
    }

    // Right-aligned GPU info.
    const char* gpu = st.gpuName.c_str();
    const float w = ImGui::CalcTextSize(gpu).x;
    ImGui::SameLine(ImGui::GetWindowWidth() - w - ImGui::GetStyle().ItemSpacing.x * 2.0f);
    ImGui::TextDisabled("%s", gpu);
    ImGui::EndMainMenuBar();
}

void LandingUI::drawToolbar(UiCommands& cmd, const OrbitCamera& cam, ViewportSettings& vp) {
    const ImGuiViewport* mv = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos({mv->WorkPos.x, mv->WorkPos.y});
    ImGui::SetNextWindowSize({mv->WorkSize.x, 0.0f});
    constexpr ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                                       ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus |
                                       ImGuiWindowFlags_AlwaysAutoResize;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {8, 5});
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::Begin("##toolbar", nullptr, flags);

    ImGui::TextDisabled("VIEW");
    ImGui::SameLine();
    for (const ViewEntry& v : kViews) {
        if (v.view == StandardView::Back || v.view == StandardView::Left || v.view == StandardView::Bottom)
            continue;
        if (ImGui::Button(v.label)) cmd.view = v.view;
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s view  [%s]", v.label, v.shortcut);
        ImGui::SameLine();
    }
    if (ImGui::Button("Fit")) cmd.fit = true;
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Zoom to fit  [F]");

    ImGui::SameLine();
    ImGui::TextDisabled("|");
    ImGui::SameLine();
    if (ImGui::Button(projectionLabel(cam.projection()))) cmd.toggleProjection = true;
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Toggle projection  [P]");

    ImGui::SameLine();
    ImGui::SetNextItemWidth(150.0f);
    int mode = static_cast<int>(vp.displayMode);
    if (ImGui::Combo("##display", &mode, "Shaded\0Shaded + Edges\0Wireframe\0"))
        vp.displayMode = static_cast<DisplayMode>(mode);

    ImGui::SameLine();
    ImGui::Checkbox("Grid", &vp.showGrid);

    m_toolbarHeight = ImGui::GetWindowSize().y;
    ImGui::End();
    ImGui::PopStyleVar(2);
}

void LandingUI::drawModelTree(Scene& scene, UiState& st) {
    const ImGuiViewport* mv = ImGui::GetMainViewport();
    const float top = mv->WorkPos.y + m_toolbarHeight + 8.0f;
    const float height = mv->WorkSize.y - m_toolbarHeight - m_statusHeight - 16.0f;
    ImGui::SetNextWindowPos({mv->WorkPos.x + 8.0f, top}, ImGuiCond_Always);
    ImGui::SetNextWindowSizeConstraints({200, 200}, {600, height});
    ImGui::SetNextWindowSize({260, std::min(height, 520.0f)}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowBgAlpha(0.88f);

    if (ImGui::Begin("Model", &st.showModelTree, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse)) {
        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        if (ImGui::TreeNode("Demo Bracket  (Part)")) {
            ImGui::SetNextItemOpen(true, ImGuiCond_Once);
            if (ImGui::TreeNode("Bodies")) {
                auto& bodies = scene.bodies();
                for (int i = 0; i < static_cast<int>(bodies.size()); ++i) {
                    Body& b = bodies[static_cast<std::size_t>(i)];
                    ImGui::PushID(i);
                    ImGui::Checkbox("##vis", &b.visible);
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip(b.visible ? "Hide" : "Show");
                    ImGui::SameLine();
                    ImGui::ColorButton("##col", {b.color.x, b.color.y, b.color.z, 1.0f},
                                       ImGuiColorEditFlags_NoTooltip, {12, 12});
                    ImGui::SameLine();
                    if (ImGui::Selectable(b.name.c_str(), st.selectedBody == i))
                        st.selectedBody = st.selectedBody == i ? -1 : i;
                    ImGui::PopID();
                }
                ImGui::TreePop();
            }
            ImGui::BeginDisabled();
            ImGui::BulletText("Sketches");
            ImGui::BulletText("Construction");
            ImGui::BulletText("CAM Setups");
            ImGui::EndDisabled();
            ImGui::TreePop();
        }

        if (st.selectedBody >= 0 && st.selectedBody < static_cast<int>(scene.bodies().size())) {
            Body& b = scene.bodies()[static_cast<std::size_t>(st.selectedBody)];
            const AABB bb = b.mesh.bounds();
            ImGui::Spacing();
            ImGui::SeparatorText("Properties");
            ImGui::Text("Name       %s", b.name.c_str());
            ImGui::Text("Triangles  %zu", b.mesh.indices.size() / 3);
            ImGui::Text("Size (mm)  %.1f x %.1f x %.1f", bb.max.x - bb.min.x, bb.max.y - bb.min.y,
                        bb.max.z - bb.min.z);
            ImGui::ColorEdit3("Colour", &b.color.x, ImGuiColorEditFlags_NoInputs);
        }
    }
    ImGui::End();
}

void LandingUI::drawWelcome(UiState& st) {
    const ImGuiViewport* mv = ImGui::GetMainViewport();
    const ImVec2 size{460, 0};
    ImGui::SetNextWindowPos({mv->WorkPos.x + mv->WorkSize.x - size.x - 16.0f,
                             mv->WorkPos.y + m_toolbarHeight + 16.0f},
                            ImGuiCond_Always);
    ImGui::SetNextWindowSize(size);
    ImGui::SetNextWindowBgAlpha(0.92f);
    constexpr ImGuiWindowFlags flags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                                       ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings |
                                       ImGuiWindowFlags_AlwaysAutoResize;
    if (ImGui::Begin("Welcome to SI CDM", &st.showWelcome, flags)) {
        ImGui::SetWindowFontScale(1.35f);
        ImGui::TextUnformatted("CAD / CAM Workbench");
        ImGui::SetWindowFontScale(1.0f);
        ImGui::TextDisabled("High-performance C++ design and manufacturing");
        ImGui::Spacing();
        ImGui::SeparatorText("Start");

        const ImVec2 btn{(size.x - ImGui::GetStyle().WindowPadding.x * 2 - ImGui::GetStyle().ItemSpacing.x) * 0.5f, 40};
        ImGui::BeginDisabled();
        ImGui::Button("New Part", btn);
        ImGui::SameLine();
        ImGui::Button("New Assembly", btn);
        ImGui::Button("Open File...", btn);
        ImGui::SameLine();
        ImGui::Button("New CAM Setup", btn);
        ImGui::EndDisabled();
        ImGui::TextDisabled("Modelling and CAM commands arrive in upcoming milestones.");

        ImGui::SeparatorText("Recent");
        ImGui::TextDisabled("No recent documents");

        ImGui::SeparatorText("Navigate the canvas");
        ImGui::BulletText("Orbit: left drag    Pan: middle or Shift + left drag");
        ImGui::BulletText("Zoom: scroll wheel  Fit: F    Views: 1 / 3 / 7 / 0");
        ImGui::Spacing();
        if (ImGui::Button("Explore the demo part")) st.showWelcome = false;
    }
    ImGui::End();
}

void LandingUI::drawStatusBar(const Scene& scene, const OrbitCamera& cam, const UiState& st) {
    const ImGuiViewport* mv = ImGui::GetMainViewport();
    const float h = ImGui::GetFrameHeight() + 4.0f;
    ImGui::SetNextWindowPos({mv->WorkPos.x, mv->WorkPos.y + mv->WorkSize.y - h});
    ImGui::SetNextWindowSize({mv->WorkSize.x, h});
    constexpr ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                                       ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {10, 3});
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::Begin("##status", nullptr, flags);
    const Vec3 t = cam.target();
    ImGui::Text("Ready");
    ImGui::SameLine(120);
    ImGui::TextDisabled("Units: mm   |   Bodies: %zu   Triangles: %zu   |   Target (%.1f, %.1f, %.1f)   Dist %.1f",
                        scene.bodies().size(), scene.triangleCount(), t.x, t.y, t.z, cam.distance());
    char perf[64];
    std::snprintf(perf, sizeof(perf), "%.2f ms  |  %s", st.frameMs, st.glVersion.c_str());
    ImGui::SameLine(ImGui::GetWindowWidth() - ImGui::CalcTextSize(perf).x - 12.0f);
    ImGui::TextDisabled("%s", perf);
    m_statusHeight = h;
    ImGui::End();
    ImGui::PopStyleVar(2);
}

void LandingUI::drawTriad(UiCommands& cmd, const OrbitCamera& cam) {
    // Navigation triad in the bottom-right of the canvas. Clicking an axis
    // tip snaps to the view looking down that axis.
    const ImGuiViewport* mv = ImGui::GetMainViewport();
    const float radius = 46.0f;
    const ImVec2 center{mv->WorkPos.x + mv->WorkSize.x - radius - 30.0f,
                        mv->WorkPos.y + mv->WorkSize.y - m_statusHeight - radius - 24.0f};
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    dl->AddCircleFilled(center, radius + 14.0f, IM_COL32(20, 22, 28, 110), 48);

    const Mat4 v = cam.view();
    struct Tip {
        int axis;
        float sign;
        ImVec2 pos;
        float depth;
    };
    std::array<Tip, 6> tips{};
    for (int a = 0; a < 3; ++a)
        for (int s = 0; s < 2; ++s) {
            Vec3 d{};
            (&d.x)[a] = s == 0 ? 1.0f : -1.0f;
            const Vec3 e = v.transformDir(d); // view space, +Y up, -Z forward
            tips[static_cast<std::size_t>(a * 2 + s)] = {a, s == 0 ? 1.0f : -1.0f,
                                                         {center.x + e.x * radius, center.y - e.y * radius}, e.z};
        }
    // Painter's order: farthest first.
    std::sort(tips.begin(), tips.end(), [](const Tip& l, const Tip& r) { return l.depth < r.depth; });

    const ImVec2 mouse = ImGui::GetIO().MousePos;
    const bool overUi = ImGui::GetIO().WantCaptureMouse;
    static constexpr char kNames[3] = {'X', 'Y', 'Z'};
    for (const Tip& t : tips) {
        const ImU32 col = kAxisColors[t.axis];
        const float dx = mouse.x - t.pos.x, dy = mouse.y - t.pos.y;
        const bool hover = !overUi && dx * dx + dy * dy < 11.0f * 11.0f;
        if (t.sign > 0) {
            dl->AddLine(center, t.pos, col, 2.5f);
            dl->AddCircleFilled(t.pos, hover ? 11.0f : 9.0f, col, 24);
            const char label[2] = {kNames[t.axis], 0};
            const ImVec2 ts = ImGui::CalcTextSize(label);
            dl->AddText({t.pos.x - ts.x * 0.5f, t.pos.y - ts.y * 0.5f}, IM_COL32(15, 15, 18, 255), label);
        } else {
            dl->AddCircleFilled(t.pos, hover ? 8.0f : 6.0f, (col & 0x00FFFFFF) | 0x70000000, 24);
            dl->AddCircle(t.pos, hover ? 8.0f : 6.0f, col, 24, 1.5f);
        }
        if (hover && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
            static constexpr StandardView pos[3] = {StandardView::Right, StandardView::Back, StandardView::Top};
            static constexpr StandardView neg[3] = {StandardView::Left, StandardView::Front, StandardView::Bottom};
            cmd.view = t.sign > 0 ? pos[t.axis] : neg[t.axis];
        }
    }
}

void LandingUI::drawControlsWindow(UiState& st) {
    ImGui::SetNextWindowSize({380, 0}, ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Viewport Controls", &st.showControls, ImGuiWindowFlags_AlwaysAutoResize)) {
        if (ImGui::BeginTable("keys", 2, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV)) {
            const char* rows[][2] = {
                {"Left drag", "Orbit"},
                {"Middle drag / Shift + left drag", "Pan"},
                {"Right drag", "Pan"},
                {"Scroll", "Zoom"},
                {"F", "Zoom to fit"},
                {"1 / 3 / 7 / 0", "Front / Right / Top / Iso"},
                {"Ctrl + 1 / 3 / 7", "Back / Left / Bottom"},
                {"P", "Toggle perspective / orthographic"},
                {"W", "Cycle display mode"},
                {"G", "Toggle grid"},
                {"Esc", "Clear selection"},
            };
            for (const auto& r : rows) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(r[0]);
                ImGui::TableSetColumnIndex(1);
                ImGui::TextDisabled("%s", r[1]);
            }
            ImGui::EndTable();
        }
    }
    ImGui::End();
}

void LandingUI::drawAboutWindow(UiState& st) {
    if (ImGui::Begin("About SI CDM", &st.showAbout, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text("SI CDM  %s", SICDM_VERSION);
        ImGui::TextDisabled("CAD / CAM workbench written in modern C++");
        ImGui::Separator();
        ImGui::Text("Renderer : %s", st.gpuName.c_str());
        ImGui::Text("OpenGL   : %s", st.glVersion.c_str());
        ImGui::Text("Dear ImGui %s", IMGUI_VERSION);
    }
    ImGui::End();
}

} // namespace sicdm

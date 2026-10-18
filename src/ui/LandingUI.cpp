#include "ui/LandingUI.hpp"

#include "core/Command.hpp"
#include "ui/ToolIcons.hpp"

#include <filesystem>

#include <imgui.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>

namespace sicdm {

namespace {

constexpr float kRulerSize = 20.0f;
constexpr ImU32 kAccent = IM_COL32(52, 92, 150, 255);

const char* displayModeLabel(DisplayMode m) {
    switch (m) {
    case DisplayMode::Shaded: return "Shaded";
    case DisplayMode::ShadedWithEdges: return "Shaded + Edges";
    case DisplayMode::Wireframe: return "Wireframe";
    case DisplayMode::HiddenLine: return "Hidden Line";
    }
    return "";
}

constexpr DisplayMode kDisplayModes[] = {DisplayMode::Shaded, DisplayMode::ShadedWithEdges, DisplayMode::Wireframe,
                                         DisplayMode::HiddenLine};

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

const char* shortPlaneName(SketchPlane p) {
    switch (p) {
    case SketchPlane::Top: return "Top";
    case SketchPlane::Front: return "Front";
    case SketchPlane::Right: return "Right";
    }
    return "";
}

// One half of the 2D / 3D segmented switch. Returns true when clicked.
bool segment(const char* label, bool active, float width, ImDrawFlags corners, void (*icon)(ImDrawList*, ImVec2, ImU32)) {
    const float h = ImGui::GetFrameHeight();
    ImGui::InvisibleButton(label, {width, h});
    const ImVec2 lo = ImGui::GetItemRectMin(), hi = ImGui::GetItemRectMax();
    const bool hovered = ImGui::IsItemHovered();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImU32 bg = active ? kAccent : (hovered ? IM_COL32(58, 64, 76, 255) : IM_COL32(40, 44, 52, 255));
    dl->AddRectFilled(lo, hi, bg, 5.0f, corners);
    const ImU32 fg = active ? IM_COL32(255, 255, 255, 255) : IM_COL32(190, 195, 204, 255);
    icon(dl, {lo.x + 16, (lo.y + hi.y) * 0.5f}, fg);
    dl->AddText({lo.x + 30, lo.y + (h - ImGui::GetFontSize()) * 0.5f}, fg, label);
    return ImGui::IsItemClicked();
}

void icon2D(ImDrawList* dl, ImVec2 c, ImU32 col) {
    // Sheet with a drawn profile.
    dl->AddRect({c.x - 7, c.y - 6}, {c.x + 7, c.y + 6}, col, 1.0f, 0, 1.2f);
    dl->AddLine({c.x - 4, c.y + 3}, {c.x - 1, c.y - 3}, col, 1.2f);
    dl->AddLine({c.x - 1, c.y - 3}, {c.x + 4, c.y + 3}, col, 1.2f);
}

void icon3D(ImDrawList* dl, ImVec2 c, ImU32 col) {
    // Isometric cube.
    const ImVec2 t{c.x, c.y - 7}, l{c.x - 6, c.y - 3.5f}, r{c.x + 6, c.y - 3.5f}, m{c.x, c.y};
    const ImVec2 bl{c.x - 6, c.y + 3.5f}, br{c.x + 6, c.y + 3.5f}, b{c.x, c.y + 7};
    dl->AddQuad(t, r, m, l, col, 1.2f);
    dl->AddQuad(l, m, b, bl, col, 1.2f);
    dl->AddQuad(m, r, br, b, col, 1.2f);
}

// "Nice" ruler step (1, 2, 5 x 10^n) of at least `minWorld`.
float niceStep(float minWorld) {
    const float p = std::pow(10.0f, std::floor(std::log10(std::max(minWorld, 1e-6f))));
    for (float m : {1.0f, 2.0f, 5.0f, 10.0f})
        if (p * m >= minWorld) return p * m;
    return p * 10.0f;
}

} // namespace

UiCommands LandingUI::draw(Scene& scene, const OrbitCamera& cam, ViewportSettings& vp, UiState& st) {
    UiCommands cmd;
    const ImGuiViewport* mv = ImGui::GetMainViewport();
    drawMenuBar(cmd, cam, vp, st);
    drawOptionsBar(cmd, cam, vp, st);
    drawStatusBar(scene, cam, st);
    drawCommandBar(cmd, st);
    const float railTop = mv->WorkPos.y + m_optionsHeight;
    const float bottom = mv->WorkPos.y + mv->WorkSize.y - m_statusHeight - m_commandHeight;
    m_toolboxWidth = m_toolbox.draw(railTop, bottom, st.mode == ViewMode::Drawing2D, st.showModelTree);
    // The Model panel docks beside the rail; tabs, rulers and canvas start after it.
    m_leftDockWidth = m_toolboxWidth;
    if (st.showModelTree) m_leftDockWidth += drawModelTree(scene, st, mv->WorkPos.x + m_toolboxWidth, railTop, bottom);
    drawDocumentTabs(cmd, st);

    const bool is2D = st.mode == ViewMode::Drawing2D;
    m_canvasMin = {mv->WorkPos.x + m_leftDockWidth + (is2D ? kRulerSize : 0.0f), m_tabsBottom + (is2D ? kRulerSize : 0.0f)};
    m_canvasMax = {mv->WorkPos.x + mv->WorkSize.x, bottom};
    if (is2D) drawRulers(cam, st);

    const ViewCubeResult vc = m_viewCube.draw(cam, {m_canvasMax.x - 10.0f, m_canvasMax.y - 10.0f}, is2D);
    if (vc.viewDirection) cmd.viewDirection = vc.viewDirection;
    if (vc.drawingPlane) {
        cmd.plane = vc.drawingPlane;
        cmd.mode = ViewMode::Drawing2D;
    }
    if (vc.open3D) cmd.mode = ViewMode::Model3D;
    if (vc.home) {
        cmd.home = true;
        cmd.mode = ViewMode::Model3D;
    }
    cmd.fit = cmd.fit || vc.fit;
    cmd.toggleProjection = cmd.toggleProjection || vc.toggleProjection;
    cmd.rotateZ += vc.rotateZ;
    cmd.roll += vc.roll;
    cmd.resetRoll = cmd.resetRoll || vc.resetRoll;
    cmd.orbitDelta = vc.orbitDelta;

    drawFileDialogs(cmd, st);
    if (m_showMcp) drawMcpWindow(st);
    if (st.showControls) drawControlsWindow(st);
    if (st.showAbout) drawAboutWindow(st);
    return cmd;
}

void LandingUI::drawMenuBar(UiCommands& cmd, const OrbitCamera& cam, ViewportSettings& vp, UiState& st) {
    if (!ImGui::BeginMainMenuBar()) return;
    m_menuHeight = ImGui::GetWindowSize().y;
    const bool is2D = st.mode == ViewMode::Drawing2D;

    if (ImGui::BeginMenu("File")) {
        if (ImGui::MenuItem("New Project", "Ctrl+N")) requestFile(-1, st);
        if (ImGui::MenuItem("Open Project...", "Ctrl+O")) requestFile(FileOpen, st);
        ImGui::Separator();
        if (ImGui::MenuItem("Save", "Ctrl+S")) {
            if (st.projectPath.empty()) requestFile(FileSaveAs, st);
            else cmd.commands.push_back("SAVE");
        }
        if (ImGui::MenuItem("Save As...", "Ctrl+Shift+S")) requestFile(FileSaveAs, st);
        ImGui::Separator();
        if (ImGui::MenuItem("Import...", "Ctrl+I")) requestFile(FileImport, st);
        if (ImGui::MenuItem("Import Image...")) requestFile(FileImportImage, st);
        if (ImGui::MenuItem("Export...", "Ctrl+E")) requestFile(FileExport, st);
        ImGui::Separator();
        ImGui::BeginDisabled();
        ImGui::MenuItem("Import STEP / IGES (planned)");
        ImGui::EndDisabled();
        ImGui::Separator();
        if (ImGui::MenuItem("Quit", "Ctrl+Q")) cmd.quit = true;
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("View")) {
        if (ImGui::MenuItem("3D Model Window", "Tab", !is2D)) cmd.mode = ViewMode::Model3D;
        if (ImGui::BeginMenu("2D Drawing Window")) {
            for (SketchPlane p : {SketchPlane::Top, SketchPlane::Front, SketchPlane::Right})
                if (ImGui::MenuItem(planeName(p), nullptr, is2D && st.plane == p)) {
                    cmd.mode = ViewMode::Drawing2D;
                    cmd.plane = p;
                }
            ImGui::EndMenu();
        }
        ImGui::Separator();
        if (!is2D) {
            for (const ViewEntry& v : kViews)
                if (ImGui::MenuItem(v.label, v.shortcut)) cmd.view = v.view;
            ImGui::Separator();
        }
        if (ImGui::MenuItem("Zoom to Fit", "F")) cmd.fit = true;
        if (!is2D) {
            if (ImGui::MenuItem("Orthographic", "P", cam.projection() == Projection::Orthographic))
                cmd.toggleProjection = true;
            ImGui::Separator();
            for (DisplayMode m : kDisplayModes)
                if (ImGui::MenuItem(displayModeLabel(m), nullptr, vp.displayMode == m)) vp.displayMode = m;
        }
        ImGui::Separator();
        ImGui::MenuItem("Grid", "G", &vp.showGrid);
        ImGui::MenuItem("Model Panel", "M", &st.showModelTree);
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
    if (ImGui::BeginMenu("AI")) {
        bool running = st.mcpRunning;
        char label[64];
        std::snprintf(label, sizeof(label), "MCP Server (port %d)", st.mcpPort);
        if (ImGui::MenuItem(label, nullptr, &running)) cmd.toggleMcp = true;
        if (ImGui::MenuItem("Connect an AI Agent...")) m_showMcp = true;
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Help")) {
        ImGui::MenuItem("Viewport Controls", nullptr, &st.showControls);
        ImGui::MenuItem("About SI CDM", nullptr, &st.showAbout);
        ImGui::EndMenu();
    }

    const char* gpu = st.gpuName.c_str();
    const float w = ImGui::CalcTextSize(gpu).x;
    ImGui::SameLine(ImGui::GetWindowWidth() - w - ImGui::GetStyle().ItemSpacing.x * 2.0f);
    ImGui::TextDisabled("%s", gpu);
    ImGui::EndMainMenuBar();
}

void LandingUI::drawOptionsBar(UiCommands& cmd, const OrbitCamera& cam, ViewportSettings& vp, UiState& st) {
    // Photoshop-style options bar: home, active tool + its options on the
    // left; the 2D / 3D window switch and view options on the right.
    const ImGuiViewport* mv = ImGui::GetMainViewport();
    const bool is2D = st.mode == ViewMode::Drawing2D;
    ImGui::SetNextWindowPos(mv->WorkPos);
    ImGui::SetNextWindowSize({mv->WorkSize.x, 0.0f});
    constexpr ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                                       ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus |
                                       ImGuiWindowFlags_AlwaysAutoResize;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {8, 6});
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::Begin("##options", nullptr, flags);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const float fh = ImGui::GetFrameHeight();

    // Home: isometric 3D view of the whole model.
    {
        ImGui::InvisibleButton("home", {fh, fh});
        const ImVec2 lo = ImGui::GetItemRectMin();
        const ImVec2 c{lo.x + fh * 0.5f, lo.y + fh * 0.5f + 1};
        if (ImGui::IsItemHovered()) {
            dl->AddRectFilled(lo, {lo.x + fh, lo.y + fh}, IM_COL32(58, 64, 76, 255), 4.0f);
            ImGui::SetTooltip("Home view (3D isometric, fit all)");
        }
        if (ImGui::IsItemClicked()) {
            cmd.home = true;
            cmd.mode = ViewMode::Model3D;
        }
        const ImU32 ic = IM_COL32(210, 214, 222, 255);
        dl->AddTriangleFilled({c.x - 8, c.y - 1}, {c.x, c.y - 8}, {c.x + 8, c.y - 1}, ic);
        dl->AddRectFilled({c.x - 6, c.y - 1}, {c.x + 6, c.y + 7}, ic);
        dl->AddRectFilled({c.x - 2, c.y + 2}, {c.x + 2, c.y + 7}, IM_COL32(28, 30, 36, 255));
    }
    auto divider = [&] {
        ImGui::SameLine();
        const ImVec2 p = ImGui::GetCursorScreenPos();
        dl->AddLine({p.x + 2, p.y + 3}, {p.x + 2, p.y + fh - 3}, IM_COL32(70, 76, 88, 255));
        ImGui::Dummy({6, fh});
        ImGui::SameLine();
    };
    divider();

    // Active tool.
    const ToolInfo& tool = toolInfo(m_toolbox.active());
    {
        ImGui::Dummy({fh, fh});
        const ImVec2 lo = ImGui::GetItemRectMin();
        drawToolIcon(dl, tool.id, {lo.x + fh * 0.5f, lo.y + fh * 0.5f}, fh * 0.62f, IM_COL32(230, 232, 238, 255));
        ImGui::SameLine();
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted(tool.name);
    }
    divider();

    // Tool options.
    ImGui::AlignTextToFramePadding();
    if (isSketchTool(tool.id)) {
        ImGui::TextDisabled("Plane");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(110);
        if (ImGui::BeginCombo("##plane", planeName(st.plane))) {
            for (SketchPlane p : {SketchPlane::Top, SketchPlane::Front, SketchPlane::Right})
                if (ImGui::Selectable(planeName(p), st.plane == p)) cmd.plane = p;
            ImGui::EndCombo();
        }
        ImGui::SameLine();
        ImGui::Checkbox("Snap to grid", &st.snapGrid);
        ImGui::SameLine();
        ImGui::Checkbox("Snap to points", &st.snapPoints);
        ImGui::SameLine();
        ImGui::TextDisabled(tool.id == ToolId::Polyline ? "Enter / double-click to finish  ·  Esc cancels"
                                                        : "Shift: horizontal / vertical  ·  Esc cancels");
    } else if (tool.id == ToolId::Extrude) {
        ImGui::TextDisabled("Distance");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(100);
        if (ImGui::DragFloat("##extdist", &st.extrudeDistance, 0.5f, -10000.0f, 10000.0f, "%.1f mm"))
            st.extrudeLive = false; // typed / dragged in the field: stop following the mouse
        ImGui::SameLine();
        ImGui::Checkbox("Symmetric", &st.extrudeSymmetric);
        ImGui::SameLine();
        ImGui::Checkbox("Keep sketch", &st.extrudeKeepSketch);
        ImGui::SameLine();
        if (st.extrudeProfileName.empty()) {
            ImGui::TextDisabled("Click a closed profile: rectangle, circle or closed polyline");
        } else {
            ImGui::TextColored({0.55f, 0.78f, 1.0f, 1.0f}, "%s", st.extrudeProfileName.c_str());
            ImGui::SameLine();
            if (ImGui::Button("Apply")) cmd.applyExtrude = true;
            ImGui::SameLine();
            if (ImGui::Button("Cancel")) cmd.cancelExtrude = true;
            ImGui::SameLine();
            ImGui::TextDisabled("drag, type the depth, click or Enter");
        }
    } else if (isPrimitiveTool(tool.id)) {
        ImGui::TextDisabled(is2D ? "Click to place on the %s plane (5 mm snap)" : "Click the grid or a face to place (5 mm snap)",
                            shortPlaneName(st.plane));
    } else if (tool.id == ToolId::Move || tool.id == ToolId::Select) {
        ImGui::TextDisabled("Auto-select:");
        ImGui::SameLine();
        ImGui::TextUnformatted(tool.id == ToolId::Select ? "Bodies & sketch curves" : "Bodies");
    } else if (tool.id == ToolId::Orbit && is2D) {
        ImGui::TextColored({0.95f, 0.70f, 0.35f, 1.0f}, "Orbit is a 3D tool; drag pans in the 2D window");
    } else if (!tool.available) {
        ImGui::TextColored({0.95f, 0.70f, 0.35f, 1.0f}, "Planned for a later milestone");
    } else {
        ImGui::TextDisabled("%s", tool.description);
    }

    // Right-aligned group: window switch + view options.
    // Width measured on the previous frame keeps the group flush right.
    const float right = ImGui::GetWindowWidth() - ImGui::GetStyle().WindowPadding.x;
    ImGui::SameLine(std::max(ImGui::GetCursorPosX() + 16.0f, right - m_rightGroupWidth));
    const float groupStart = ImGui::GetCursorScreenPos().x;
    if (segment("2D Drawing", is2D, 128.0f, ImDrawFlags_RoundCornersLeft, icon2D) && !is2D) cmd.mode = ViewMode::Drawing2D;
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("2D drawing window: plan view of the %s plane  [Tab]", shortPlaneName(st.plane));
    ImGui::SameLine(0, 2);
    if (segment("3D Model", !is2D, 128.0f, ImDrawFlags_RoundCornersRight, icon3D) && is2D) cmd.mode = ViewMode::Model3D;
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("3D model window  [Tab]");
    divider();

    if (is2D) {
        ImGui::AlignTextToFramePadding();
        ImGui::TextDisabled("Plane");
        ImGui::SameLine();
        for (SketchPlane p : {SketchPlane::Top, SketchPlane::Front, SketchPlane::Right}) {
            const bool on = st.plane == p;
            if (on) ImGui::PushStyleColor(ImGuiCol_Button, ImGui::ColorConvertU32ToFloat4(kAccent));
            if (ImGui::Button(shortPlaneName(p))) cmd.plane = p;
            if (on) ImGui::PopStyleColor();
            ImGui::SameLine();
        }
    } else {
        ImGui::SetNextItemWidth(140.0f);
        int mode = static_cast<int>(vp.displayMode);
        if (ImGui::Combo("##display", &mode, "Shaded\0Shaded + Edges\0Wireframe\0Hidden Line\0"))
            vp.displayMode = static_cast<DisplayMode>(mode);
        ImGui::SameLine();
        if (ImGui::Button(cam.projection() == Projection::Perspective ? "Persp" : "Ortho")) cmd.toggleProjection = true;
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Toggle projection  [P]");
        ImGui::SameLine();
    }
    if (ImGui::Button("Fit")) cmd.fit = true;
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Zoom to fit  [F]");
    ImGui::SameLine();
    ImGui::Checkbox("Grid", &vp.showGrid);
    m_rightGroupWidth = ImGui::GetItemRectMax().x - groupStart;

    m_optionsHeight = ImGui::GetWindowSize().y;
    ImGui::End();
    ImGui::PopStyleVar(2);
}

void LandingUI::drawDocumentTabs(UiCommands& cmd, const UiState& st) {
    // Each view window is a document tab, so switching feels like Photoshop.
    const ImGuiViewport* mv = ImGui::GetMainViewport();
    const float x = mv->WorkPos.x + m_leftDockWidth;
    const float y = mv->WorkPos.y + m_optionsHeight;
    ImGui::SetNextWindowPos({x, y});
    ImGui::SetNextWindowSize({mv->WorkSize.x - m_leftDockWidth, 0.0f});
    constexpr ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                                       ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus |
                                       ImGuiWindowFlags_AlwaysAutoResize;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {6, 4});
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::Begin("##tabs", nullptr, flags);

    // Tabs change the mode only when clicked; otherwise they follow it.
    auto flagsFor = [&](ViewMode m) {
        return st.mode == m && m_tabMode != m ? ImGuiTabItemFlags_SetSelected : ImGuiTabItemFlags_None;
    };
    char label2D[96];
    std::snprintf(label2D, sizeof(label2D), "Demo Bracket @ 2D Drawing (%s)###tab2d", shortPlaneName(st.plane));
    if (ImGui::BeginTabBar("##docs")) {
        const bool open3D = ImGui::BeginTabItem("Demo Bracket @ 3D Model###tab3d", nullptr, flagsFor(ViewMode::Model3D));
        if (ImGui::IsItemClicked()) cmd.mode = ViewMode::Model3D;
        if (open3D) {
            m_tabMode = ViewMode::Model3D;
            ImGui::EndTabItem();
        }
        const bool open2D = ImGui::BeginTabItem(label2D, nullptr, flagsFor(ViewMode::Drawing2D));
        if (ImGui::IsItemClicked()) cmd.mode = ViewMode::Drawing2D;
        if (open2D) {
            m_tabMode = ViewMode::Drawing2D;
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    m_tabsBottom = y + ImGui::GetWindowSize().y;
    ImGui::End();
    ImGui::PopStyleVar(2);
}

void LandingUI::drawRulers(const OrbitCamera& cam, const UiState& st) {
    // Rulers in mm along the drawing plane's u (top) and v (left) axes.
    const ImGuiIO& io = ImGui::GetIO();
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    const ImVec2 vp = io.DisplaySize;
    const float wpp = cam.worldPerPixel(vp.y);
    if (wpp <= 0.0f) return;
    const PlaneFrame f = planeFrame(st.plane);
    const Vec2 c = f.toPlane(cam.target());
    auto uAt = [&](float x) { return c.x + (x - vp.x * 0.5f) * wpp; };
    auto vAt = [&](float y) { return c.y - (y - vp.y * 0.5f) * wpp; };
    auto xOf = [&](float u) { return vp.x * 0.5f + (u - c.x) / wpp; };
    auto yOf = [&](float v) { return vp.y * 0.5f - (v - c.y) / wpp; };

    const float x0 = m_canvasMin.x - kRulerSize, y0 = m_canvasMin.y - kRulerSize;
    const ImU32 bg = IM_COL32(30, 33, 39, 255), tick = IM_COL32(120, 126, 138, 255), text = IM_COL32(170, 175, 186, 255);
    dl->AddRectFilled({x0, y0}, {m_canvasMax.x, m_canvasMin.y}, bg);
    dl->AddRectFilled({x0, y0}, {m_canvasMin.x, m_canvasMax.y}, bg);
    dl->AddLine({m_canvasMin.x, m_canvasMin.y}, {m_canvasMax.x, m_canvasMin.y}, IM_COL32(60, 64, 74, 255));
    dl->AddLine({m_canvasMin.x, m_canvasMin.y}, {m_canvasMin.x, m_canvasMax.y}, IM_COL32(60, 64, 74, 255));

    const float major = niceStep(wpp * 70.0f);
    const float minor = major / 10.0f;
    ImFont* font = ImGui::GetFont();
    const float fs = ImGui::GetFontSize() * 0.82f;
    char buf[32];
    auto fmt = [&](float v) {
        if (std::fabs(v) < major * 1e-3f) v = 0.0f;
        std::snprintf(buf, sizeof(buf), major >= 1.0f ? "%.0f" : (major >= 0.1f ? "%.1f" : "%.2f"), v);
        return buf;
    };

    // Top ruler (u).
    dl->PushClipRect({m_canvasMin.x, y0}, {m_canvasMax.x, m_canvasMin.y}, true);
    for (float u = std::floor(uAt(m_canvasMin.x) / minor) * minor; u <= uAt(m_canvasMax.x); u += minor) {
        const float x = xOf(u);
        const long k = std::lround(u / minor);
        const float len = k % 10 == 0 ? kRulerSize : (k % 5 == 0 ? 8.0f : 4.0f);
        dl->AddLine({x, m_canvasMin.y - len}, {x, m_canvasMin.y}, tick);
        if (k % 10 == 0) dl->AddText(font, fs, {x + 3, y0 + 2}, text, fmt(u));
    }
    dl->PopClipRect();

    // Left ruler (v), labels written top-to-bottom like Photoshop.
    dl->PushClipRect({x0, m_canvasMin.y}, {m_canvasMin.x, m_canvasMax.y}, true);
    for (float v = std::floor(vAt(m_canvasMax.y) / minor) * minor; v <= vAt(m_canvasMin.y); v += minor) {
        const float y = yOf(v);
        const long k = std::lround(v / minor);
        const float len = k % 10 == 0 ? kRulerSize : (k % 5 == 0 ? 8.0f : 4.0f);
        dl->AddLine({m_canvasMin.x - len, y}, {m_canvasMin.x, y}, tick);
        if (k % 10 == 0) {
            const char* s = fmt(v);
            float yy = y + 2;
            for (const char* ch = s; *ch; ++ch) {
                const char one[2] = {*ch, 0};
                dl->AddText(font, fs, {x0 + 3, yy}, text, one);
                yy += fs * 0.8f;
            }
        }
    }
    dl->PopClipRect();

    // Cursor markers.
    if (st.cursorPlane) {
        const ImU32 mk = IM_COL32(120, 220, 255, 255);
        const float cx = xOf(st.cursorPlane->x), cy = yOf(st.cursorPlane->y);
        if (cx > m_canvasMin.x && cx < m_canvasMax.x) dl->AddLine({cx, y0}, {cx, m_canvasMin.y}, mk);
        if (cy > m_canvasMin.y && cy < m_canvasMax.y) dl->AddLine({x0, cy}, {m_canvasMin.x, cy}, mk);
    }
    // Corner box with units and plane axes.
    dl->AddRectFilled({x0, y0}, {m_canvasMin.x, m_canvasMin.y}, IM_COL32(24, 26, 31, 255));
    dl->AddText(font, fs, {x0 + 2, y0 + 3}, text, "mm");
}

float LandingUI::drawModelTree(Scene& scene, UiState& st, float x, float top, float bottom) {
    // Docked full-height panel; only its right edge can be dragged to resize.
    const float height = std::max(bottom - top, 100.0f);
    ImGui::SetNextWindowPos({x, top}, ImGuiCond_Always);
    ImGui::SetNextWindowSizeConstraints({200, height}, {440, height});
    ImGui::SetNextWindowSize({260, height}, ImGuiCond_FirstUseEver);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    constexpr ImGuiWindowFlags flags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
                                       ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus;
    const bool visible = ImGui::Begin("Model", &st.showModelTree, flags);
    ImGui::PopStyleVar();
    const float width = ImGui::GetWindowWidth();
    if (visible) {
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
                    if (ImGui::Selectable(b.name.c_str(), st.selectedBody == i)) {
                        st.selectedBody = st.selectedBody == i ? -1 : i;
                        st.selectedCurve = st.selectedImage = -1;
                    }
                    ImGui::PopID();
                }
                ImGui::TreePop();
            }
            auto& curves = scene.sketch().curves();
            char sketchLabel[48];
            std::snprintf(sketchLabel, sizeof(sketchLabel), "Sketches (%zu)###sketches", curves.size());
            ImGui::SetNextItemOpen(true, ImGuiCond_Once);
            if (ImGui::TreeNode(sketchLabel)) {
                if (curves.empty()) ImGui::TextDisabled("Use the 2D tools to draw");
                for (int i = 0; i < static_cast<int>(curves.size()); ++i) {
                    SketchCurve& c = curves[static_cast<std::size_t>(i)];
                    ImGui::PushID(1000 + i);
                    ImGui::Checkbox("##vis", &c.visible);
                    ImGui::SameLine();
                    if (ImGui::Selectable(c.name.c_str(), st.selectedCurve == i)) {
                        st.selectedCurve = st.selectedCurve == i ? -1 : i;
                        st.selectedBody = st.selectedImage = -1;
                    }
                    ImGui::SameLine(190);
                    ImGui::TextDisabled("%s", shortPlaneName(c.plane));
                    ImGui::PopID();
                }
                ImGui::TreePop();
            }
            auto& images = scene.images();
            char imageLabel[48];
            std::snprintf(imageLabel, sizeof(imageLabel), "Images (%zu)###images", images.size());
            ImGui::SetNextItemOpen(true, ImGuiCond_Once);
            if (ImGui::TreeNode(imageLabel)) {
                if (images.empty()) ImGui::TextDisabled("File > Import Image...");
                for (int i = 0; i < static_cast<int>(images.size()); ++i) {
                    ImageRef& img = images[static_cast<std::size_t>(i)];
                    ImGui::PushID(2000 + i);
                    ImGui::Checkbox("##vis", &img.visible);
                    ImGui::SameLine();
                    if (ImGui::Selectable(img.name.c_str(), st.selectedImage == i)) {
                        st.selectedImage = st.selectedImage == i ? -1 : i;
                        st.selectedBody = st.selectedCurve = -1;
                    }
                    ImGui::SameLine(190);
                    ImGui::TextDisabled("%s", img.pixels.empty() ? "missing" : shortPlaneName(img.plane));
                    ImGui::PopID();
                }
                ImGui::TreePop();
            }
            ImGui::BeginDisabled();
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
            ImGui::Text("Position   %.1f, %.1f, %.1f", b.offset.x, b.offset.y, b.offset.z);
            ImGui::ColorEdit3("Colour", &b.color.x, ImGuiColorEditFlags_NoInputs);
        } else if (st.selectedCurve >= 0 && st.selectedCurve < static_cast<int>(scene.sketch().curves().size())) {
            const SketchCurve& c = scene.sketch().curves()[static_cast<std::size_t>(st.selectedCurve)];
            ImGui::Spacing();
            ImGui::SeparatorText("Properties");
            ImGui::Text("Name       %s", c.name.c_str());
            ImGui::Text("Type       %s", curveKindName(c.kind));
            ImGui::Text("Plane      %s", planeName(c.plane));
            ImGui::Text("Size (mm)  %s", c.measure().c_str());
        } else if (st.selectedImage >= 0 && st.selectedImage < static_cast<int>(scene.images().size())) {
            ImageRef& img = scene.images()[static_cast<std::size_t>(st.selectedImage)];
            ImGui::Spacing();
            ImGui::SeparatorText("Properties");
            ImGui::Text("Name       %s", img.name.c_str());
            ImGui::Text("Plane      %s", planeName(img.plane));
            ImGui::Text("Pixels     %d x %d", img.pixelWidth, img.pixelHeight);
            ImGui::TextWrapped("File       %s", img.path.c_str());
            ImGui::SetNextItemWidth(-90);
            ImGui::DragFloat("Width mm", &img.width, 0.5f, 0.1f, 100000.0f, "%.1f");
            ImGui::TextDisabled("Height     %.1f mm", static_cast<double>(img.height()));
            ImGui::SetNextItemWidth(-90);
            ImGui::DragFloat2("Centre", &img.center.x, 0.5f, -100000.0f, 100000.0f, "%.1f");
            ImGui::SetNextItemWidth(-90);
            ImGui::SliderFloat("Opacity", &img.opacity, 0.0f, 1.0f, "%.2f");
            img.width = std::max(img.width, 0.1f);
        }
    }
    ImGui::End();
    return width;
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
    const ToolInfo& tool = toolInfo(m_toolbox.active());
    ImGui::Text("%s", tool.name);
    ImGui::SameLine();
    // Long messages (file paths) are clipped so they never run into the coordinates.
    const float msgEnd = ImGui::GetWindowPos().x + 588.0f;
    const ImVec2 msgPos = ImGui::GetCursorScreenPos();
    ImGui::PushClipRect(msgPos, {msgEnd, msgPos.y + ImGui::GetFrameHeight()}, true);
    if (!st.statusMessage.empty()) ImGui::TextColored({0.55f, 0.78f, 1.0f, 1.0f}, "%s", st.statusMessage.c_str());
    else if (tool.available) ImGui::TextDisabled("%s", tool.description);
    else ImGui::TextColored({0.95f, 0.70f, 0.35f, 1.0f}, "planned for a later milestone");
    ImGui::PopClipRect();

    ImGui::SameLine(600.0f);
    const char* uName = st.plane == SketchPlane::Right ? "Y" : "X";
    const char* vName = st.plane == SketchPlane::Top ? "Y" : "Z";
    auto clean = [](float v) { return std::fabs(v) < 0.005f ? 0.0f : v; }; // no "-0.00"
    if (st.cursorPlane)
        ImGui::Text("%s %8.2f   %s %8.2f mm", uName, clean(st.cursorPlane->x), vName, clean(st.cursorPlane->y));
    else
        ImGui::TextDisabled("%s     --      %s     --   mm", uName, vName);
    ImGui::SameLine();
    ImGui::TextDisabled("|  %s  |  Bodies %zu  Curves %zu  Images %zu", st.mode == ViewMode::Drawing2D ? "2D Drawing" : "3D Model",
                        scene.bodies().size(), scene.sketch().curves().size(), scene.images().size());
    if (st.mcpRunning) {
        ImGui::SameLine();
        ImGui::TextColored({0.45f, 0.85f, 0.55f, 1.0f}, "|  MCP :%d", st.mcpPort);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("AI agents can connect at http://127.0.0.1:%d/mcp", st.mcpPort);
    }
    (void)cam;
    char perf[64];
    std::snprintf(perf, sizeof(perf), "%.1f ms", static_cast<double>(st.frameMs));
    ImGui::SameLine(ImGui::GetWindowWidth() - ImGui::CalcTextSize(perf).x - 12.0f);
    ImGui::TextDisabled("%s", perf);
    m_statusHeight = h;
    ImGui::End();
    ImGui::PopStyleVar(2);
}

namespace {

constexpr const char* kImageExts[] = {".png", ".jpg", ".jpeg", ".bmp", ".tga", ".gif", ".psd", ".hdr", ".pnm", ".ppm", ".pgm"};

std::string quoted(const std::string& path) { return "\"" + path + "\""; }

std::string startDirFor(const UiState& st) {
    if (!st.projectPath.empty()) return std::filesystem::path(st.projectPath).parent_path().string();
    std::error_code ec;
    return std::filesystem::current_path(ec).string();
}

std::string projectStem(const UiState& st) {
    return st.projectPath.empty() ? std::string("Untitled") : std::filesystem::path(st.projectPath).stem().string();
}

} // namespace

int LandingUI::commandInputCallback(ImGuiInputTextCallbackData* data) {
    auto* self = static_cast<LandingUI*>(data->UserData);
    if (data->EventFlag == ImGuiInputTextFlags_CallbackCompletion) {
        // Tab: complete the command name.
        const std::string text(data->Buf, static_cast<std::size_t>(data->BufTextLen));
        if (text.find(' ') == std::string::npos) {
            const auto matches = completeCommand(text);
            if (!matches.empty()) {
                data->DeleteChars(0, data->BufTextLen);
                data->InsertChars(0, (std::string(matches.front()->name) + " ").c_str());
            }
        }
    } else if (data->EventFlag == ImGuiInputTextFlags_CallbackHistory && !self->m_recall.empty()) {
        // Up / Down: recall earlier inputs.
        const int n = static_cast<int>(self->m_recall.size());
        if (data->EventKey == ImGuiKey_UpArrow) self->m_recallPos = self->m_recallPos < 0 ? n - 1 : std::max(0, self->m_recallPos - 1);
        else if (data->EventKey == ImGuiKey_DownArrow) self->m_recallPos = self->m_recallPos < 0 || self->m_recallPos + 1 >= n ? -1 : self->m_recallPos + 1;
        data->DeleteChars(0, data->BufTextLen);
        if (self->m_recallPos >= 0) data->InsertChars(0, self->m_recall[static_cast<std::size_t>(self->m_recallPos)].c_str());
    }
    return 0;
}

void LandingUI::drawCommandBar(UiCommands& cmd, UiState& st) {
    // Full-width command line directly above the status bar (AutoCAD style).
    const ImGuiViewport* mv = ImGui::GetMainViewport();
    const ImGuiIO& io = ImGui::GetIO();
    const float h = ImGui::GetFrameHeight() + 10.0f;
    const float y = mv->WorkPos.y + mv->WorkSize.y - m_statusHeight - h;
    m_commandHeight = h;

    // Shortcuts: Space jumps to the command line; Ctrl/Cmd + N O S I E for files
    // (these also work while typing in the command line, but not in dialogs).
    const bool mod = commandModifierHeld();
    if (!io.WantCaptureKeyboard && !mod && ImGui::IsKeyPressed(ImGuiKey_Space, false)) m_focusCommand = true;
    if (mod && !m_fileDialog.isOpen() && !ImGui::IsPopupOpen(nullptr, ImGuiPopupFlags_AnyPopupId)) {
        if (ImGui::IsKeyPressed(ImGuiKey_N, false)) requestFile(-1, st);
        if (mod && ImGui::IsKeyPressed(ImGuiKey_O, false)) requestFile(FileOpen, st);
        if (mod && ImGui::IsKeyPressed(ImGuiKey_I, false)) requestFile(FileImport, st);
        if (mod && ImGui::IsKeyPressed(ImGuiKey_E, false)) requestFile(FileExport, st);
        if (mod && ImGui::IsKeyPressed(ImGuiKey_S, false)) {
            if (io.KeyShift || st.projectPath.empty()) requestFile(FileSaveAs, st);
            else cmd.commands.push_back("SAVE");
        }
    }

    // New output arrived: show the last lines briefly above the bar.
    if (st.commandLog.size() != m_seenLog) {
        m_seenLog = st.commandLog.size();
        m_showRecentUntil = ImGui::GetTime() + 5.0;
    }
    const bool recent = ImGui::GetTime() < m_showRecentUntil;

    ImGui::SetNextWindowPos({mv->WorkPos.x, y});
    ImGui::SetNextWindowSize({mv->WorkSize.x, h});
    constexpr ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                                       ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {8, 5});
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4{0.085f, 0.09f, 0.105f, 1.0f});
    ImGui::Begin("##commandbar", nullptr, flags);

    // History toggle.
    if (ImGui::ArrowButton("##hist", m_showHistory ? ImGuiDir_Down : ImGuiDir_Up)) m_showHistory = !m_showHistory;
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Command history");
    ImGui::SameLine();
    ImGui::AlignTextToFramePadding();
    ImGui::TextColored({0.45f, 0.72f, 1.0f, 1.0f}, "%s", st.commandPrompt.c_str());
    ImGui::SameLine();

    ImGui::SetNextItemWidth(-1);
    if (m_focusCommand) {
        ImGui::SetKeyboardFocusHere();
        m_focusCommand = false;
    }
    ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4{0.13f, 0.14f, 0.17f, 1.0f});
    const bool submitted = ImGui::InputTextWithHint(
        "##command", "Type a command (HELP for the list, Tab completes, Space to focus) or x,y coordinates",
        m_commandBuf, sizeof(m_commandBuf),
        ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_CallbackCompletion |
            ImGuiInputTextFlags_CallbackHistory,
        commandInputCallback, this);
    ImGui::PopStyleColor();
    const bool active = ImGui::IsItemActive();
    const ImVec2 inputMin = ImGui::GetItemRectMin();
    if (submitted) {
        const std::string text = m_commandBuf;
        cmd.commands.push_back(text); // empty = Enter (finishes a polyline)
        if (!text.empty() && (m_recall.empty() || m_recall.back() != text)) m_recall.push_back(text);
        m_recallPos = -1;
        m_commandBuf[0] = '\0';
        // Stay in the command line while a sketch tool takes more points; otherwise
        // hand the keyboard back so single-key shortcuts work again.
        m_focusCommand = st.commandKeepsFocus;
    }
    ImGui::End();
    ImGui::PopStyleColor();
    ImGui::PopStyleVar(2);

    const float lineH = ImGui::GetTextLineHeightWithSpacing();
    auto colorOf = [](UiState::LogLine::Kind k) {
        switch (k) {
        case UiState::LogLine::Input: return ImVec4{0.62f, 0.66f, 0.74f, 1.0f};
        case UiState::LogLine::Error: return ImVec4{1.0f, 0.48f, 0.42f, 1.0f};
        default: return ImVec4{0.86f, 0.89f, 0.94f, 1.0f};
        }
    };

    // Completion list above the input while typing a command name.
    const std::string typed = m_commandBuf;
    if (active && !typed.empty() && typed.find(' ') == std::string::npos) {
        const auto matches = completeCommand(typed);
        if (!matches.empty()) {
            const std::size_t n = std::min<std::size_t>(matches.size(), 8);
            ImGui::SetNextWindowPos({inputMin.x, y - 6.0f}, ImGuiCond_Always, {0.0f, 1.0f});
            ImGui::SetNextWindowBgAlpha(0.96f);
            ImGui::Begin("##complete", nullptr,
                         ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav |
                             ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings |
                             ImGuiWindowFlags_NoInputs);
            for (std::size_t i = 0; i < n; ++i) {
                ImGui::TextColored(i == 0 ? ImVec4{0.45f, 0.72f, 1.0f, 1} : ImVec4{0.85f, 0.87f, 0.92f, 1}, "%-10s",
                                   matches[i]->name);
                ImGui::SameLine();
                ImGui::TextDisabled("%-38s %s", matches[i]->usage, matches[i]->help);
            }
            ImGui::TextDisabled("Tab completes %s", matches.front()->name);
            ImGui::End();
        }
    } else if ((m_showHistory || recent) && !st.commandLog.empty()) {
        // History panel (toggle) or the last few lines after a command.
        const int lines = m_showHistory ? 12 : 3;
        const float ph = lineH * static_cast<float>(lines) + 12.0f;
        ImGui::SetNextWindowPos({mv->WorkPos.x, y - ph});
        ImGui::SetNextWindowSize({mv->WorkSize.x, ph});
        ImGui::SetNextWindowBgAlpha(m_showHistory ? 0.96f : 0.75f);
        ImGuiWindowFlags hf = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoSavedSettings |
                              ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav;
        if (!m_showHistory) hf |= ImGuiWindowFlags_NoInputs;
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
        ImGui::Begin("##history", nullptr, hf);
        const std::size_t first =
            m_showHistory ? 0 : (st.commandLog.size() > 3 ? st.commandLog.size() - 3 : 0);
        for (std::size_t i = first; i < st.commandLog.size(); ++i) {
            const auto& l = st.commandLog[i];
            ImGui::TextColored(colorOf(l.kind), "%s%s", l.kind == UiState::LogLine::Input ? "> " : "  ", l.text.c_str());
        }
        if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 1.0f || !m_showHistory) ImGui::SetScrollHereY(1.0f);
        ImGui::End();
        ImGui::PopStyleVar();
    }
}

void LandingUI::requestFile(int action, const UiState& st) {
    // New / Open would discard unsaved work: ask first.
    if ((action == -1 || action == FileOpen) && st.dirty) {
        m_confirmAction = action;
        m_confirmOpen = true;
        return;
    }
    m_fileAction = action;
    const std::string dir = startDirFor(st);
    switch (action) {
    case -1: m_fileAction = -1; break; // handled in drawFileDialogs
    case FileOpen: m_fileDialog.open(FileDialog::Mode::Open, "Open Project", {".sicdm"}, dir); break;
    case FileSaveAs:
        m_fileDialog.open(FileDialog::Mode::Save, "Save Project As", {".sicdm"}, dir, projectStem(st) + ".sicdm");
        break;
    case FileImport: {
        std::vector<std::string> ext = {".stl", ".obj", ".dxf"};
        ext.insert(ext.end(), std::begin(kImageExts), std::end(kImageExts));
        m_fileDialog.open(FileDialog::Mode::Open, "Import (STL, OBJ, DXF, images)", ext, dir);
        break;
    }
    case FileImportImage:
        m_fileDialog.open(FileDialog::Mode::Open, "Import Image", {std::begin(kImageExts), std::end(kImageExts)}, dir);
        break;
    case FileExport:
        m_fileDialog.open(FileDialog::Mode::Save, "Export (STL / OBJ model, DXF / SVG sketch plane)",
                          {".stl", ".obj", ".dxf", ".svg"}, dir, projectStem(st) + ".stl");
        break;
    default: m_fileAction = FileNone;
    }
}

void LandingUI::drawFileDialogs(UiCommands& cmd, UiState& st) {
    if (m_fileAction == -1) { // New confirmed or nothing to lose
        cmd.commands.push_back("NEW");
        m_fileAction = FileNone;
    }
    if (const auto path = m_fileDialog.draw()) {
        switch (m_fileAction) {
        case FileOpen: cmd.commands.push_back("OPEN " + quoted(*path)); break;
        case FileSaveAs: cmd.commands.push_back("SAVE " + quoted(*path)); break;
        case FileImport:
        case FileImportImage: cmd.commands.push_back("IMPORT " + quoted(*path)); break;
        case FileExport: cmd.commands.push_back("EXPORT " + quoted(*path)); break;
        default: break;
        }
        m_fileAction = FileNone;
    }

    if (m_confirmOpen) {
        ImGui::OpenPopup("Unsaved changes");
        m_confirmOpen = false;
    }
    if (ImGui::BeginPopupModal("Unsaved changes", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text("%s has unsaved changes.", st.projectTitle.c_str());
        ImGui::TextDisabled(m_confirmAction == -1 ? "Start a new project anyway?" : "Open another project anyway?");
        ImGui::Spacing();
        if (ImGui::Button("Save First", {110, 0})) {
            if (st.projectPath.empty()) {
                ImGui::CloseCurrentPopup();
                requestFile(FileSaveAs, st);
            } else {
                cmd.commands.push_back("SAVE");
                st.dirty = false; // so the follow-up action does not ask again
                const int next = m_confirmAction;
                ImGui::CloseCurrentPopup();
                requestFile(next, st);
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Discard", {110, 0})) {
            const int next = m_confirmAction;
            ImGui::CloseCurrentPopup();
            UiState clean = st;
            clean.dirty = false;
            requestFile(next, clean);
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", {110, 0}) || ImGui::IsKeyPressed(ImGuiKey_Escape)) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
}

void LandingUI::drawMcpWindow(UiState& st) {
    ImGui::SetNextWindowSize({620, 0}, ImGuiCond_Appearing);
    if (!ImGui::Begin("Connect an AI Agent (MCP)", &m_showMcp, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::End();
        return;
    }
    ImGui::TextWrapped("SI CDM speaks the Model Context Protocol, so AI agents can read and edit the 2D and 3D "
                       "model: sketches, solids, images, views and files.");
    ImGui::Spacing();
    ImGui::SeparatorText("Live app (HTTP)");
    if (st.mcpRunning)
        ImGui::TextColored({0.45f, 0.85f, 0.55f, 1}, "Running on http://127.0.0.1:%d/mcp", st.mcpPort);
    else if (!st.mcpError.empty())
        ImGui::TextColored({1.0f, 0.48f, 0.42f, 1}, "%s", st.mcpError.c_str());
    else
        ImGui::TextDisabled("Stopped. Enable it in AI > MCP Server, or start the app with --mcp-port %d.", st.mcpPort);
    char url[96];
    std::snprintf(url, sizeof(url), "claude mcp add --transport http si-cdm http://127.0.0.1:%d/mcp", st.mcpPort);
    ImGui::TextDisabled("Claude Code:");
    ImGui::TextUnformatted(url);
    ImGui::SameLine();
    if (ImGui::SmallButton("Copy##http")) ImGui::SetClipboardText(url);

    ImGui::Spacing();
    ImGui::SeparatorText("Headless (stdio)");
    ImGui::TextWrapped("sicdm-mcp edits project files without a window. Point your agent at it:");
    const std::string exe = st.mcpExecutable.empty() ? std::string("/path/to/sicdm-mcp") : st.mcpExecutable;
    const std::string cli = "claude mcp add si-cdm-files -- \"" + exe + "\"";
    ImGui::TextUnformatted(cli.c_str());
    ImGui::SameLine();
    if (ImGui::SmallButton("Copy##cli")) ImGui::SetClipboardText(cli.c_str());
    const std::string json = "{ \"mcpServers\": { \"si-cdm\": { \"command\": \"" + exe + "\" } } }";
    ImGui::TextDisabled("Claude Desktop / other clients (JSON config):");
    ImGui::TextUnformatted(json.c_str());
    ImGui::SameLine();
    if (ImGui::SmallButton("Copy##json")) ImGui::SetClipboardText(json.c_str());
    ImGui::Spacing();
    ImGui::TextDisabled("Tools: get_scene, create_sketch, create_solid, extrude, modify, delete, set_view, run_command,");
    ImGui::TextDisabled("       open/save/new_project, import_file, export_file, list_commands");
    ImGui::End();
}

void LandingUI::drawControlsWindow(UiState& st) {
    ImGui::SetNextWindowSize({380, 0}, ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Viewport Controls", &st.showControls, ImGuiWindowFlags_AlwaysAutoResize)) {
        if (ImGui::BeginTable("keys", 2, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV)) {
            const char* rows[][2] = {
                {"Tab", "Switch 2D Drawing / 3D Model window"},
                {"ViewCube click", "Face / edge / corner view (26 views)"},
                {"ViewCube drag", "Orbit (3D)"},
                {"ViewCube right-click", "More views & windows"},
                {"Left drag (empty space)", "Orbit in 3D, pan in 2D"},
                {"Right drag", "Orbit in 3D, pan in 2D"},
                {"Middle drag / Shift + left drag", "Pan"},
                {"Scroll", "Zoom at cursor"},
                {"F", "Zoom to fit"},
                {"1 / 3 / 7 / 0", "3D: Front / Right / Top / Iso"},
                {"1 / 3 / 7 (2D)", "Front / Right / Top drawing plane"},
                {"Ctrl + 1 / 3 / 7", "Back / Left / Bottom"},
                {"P", "Toggle perspective / orthographic"},
                {"W", "Cycle display mode"},
                {"G", "Toggle grid"},
                {"Enter", "Finish polyline"},
                {"Esc", "Cancel drawing / clear selection"},
                {"Delete / Backspace", "Delete selected body or curve"},
                {"M", "Show / hide the Model panel"},
                {"V  S  O  H  Z", "Move / Select / Orbit / Pan / Zoom tools"},
                {"L  C  R", "Line / Circle / Rectangle sketch tools"},
                {"B  Y", "Block / Cylinder tools (Shift cycles the slot)"},
                {"E", "Extrude: click a closed profile, drag or type the depth"},
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

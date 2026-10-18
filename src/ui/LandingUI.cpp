#include "ui/LandingUI.hpp"

#include "ui/ToolIcons.hpp"

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
    const float railTop = mv->WorkPos.y + m_optionsHeight;
    const float bottom = mv->WorkPos.y + mv->WorkSize.y - m_statusHeight;
    m_toolboxWidth = m_toolbox.draw(railTop, bottom, st.mode == ViewMode::Drawing2D);
    drawDocumentTabs(cmd, st);

    const bool is2D = st.mode == ViewMode::Drawing2D;
    m_canvasMin = {mv->WorkPos.x + m_toolboxWidth + (is2D ? kRulerSize : 0.0f), m_tabsBottom + (is2D ? kRulerSize : 0.0f)};
    m_canvasMax = {mv->WorkPos.x + mv->WorkSize.x, bottom};
    if (is2D) drawRulers(cam, st);

    if (st.showModelTree) drawModelTree(scene, st);

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

    if (st.showControls) drawControlsWindow(st);
    if (st.showAbout) drawAboutWindow(st);
    return cmd;
}

void LandingUI::drawMenuBar(UiCommands& cmd, const OrbitCamera& cam, ViewportSettings& vp, UiState& st) {
    if (!ImGui::BeginMainMenuBar()) return;
    m_menuHeight = ImGui::GetWindowSize().y;
    const bool is2D = st.mode == ViewMode::Drawing2D;

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
        ImGui::MenuItem("Model Tree", nullptr, &st.showModelTree);
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
    const float x = mv->WorkPos.x + m_toolboxWidth;
    const float y = mv->WorkPos.y + m_optionsHeight;
    ImGui::SetNextWindowPos({x, y});
    ImGui::SetNextWindowSize({mv->WorkSize.x - m_toolboxWidth, 0.0f});
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

void LandingUI::drawModelTree(Scene& scene, UiState& st) {
    const float top = m_canvasMin.y + 8.0f;
    const float height = m_canvasMax.y - m_canvasMin.y - 16.0f;
    ImGui::SetNextWindowPos({m_canvasMin.x + 8.0f, top}, ImGuiCond_Always);
    ImGui::SetNextWindowSizeConstraints({200, 200}, {600, std::max(height, 200.0f)});
    ImGui::SetNextWindowSize({270, std::min(height, 520.0f)}, ImGuiCond_FirstUseEver);
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
                    if (ImGui::Selectable(b.name.c_str(), st.selectedBody == i)) {
                        st.selectedBody = st.selectedBody == i ? -1 : i;
                        st.selectedCurve = -1;
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
                        st.selectedBody = -1;
                    }
                    ImGui::SameLine(190);
                    ImGui::TextDisabled("%s", shortPlaneName(c.plane));
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
        }
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
    const ToolInfo& tool = toolInfo(m_toolbox.active());
    ImGui::Text("%s", tool.name);
    ImGui::SameLine();
    if (!st.statusMessage.empty()) ImGui::TextColored({0.55f, 0.78f, 1.0f, 1.0f}, "%s", st.statusMessage.c_str());
    else if (tool.available) ImGui::TextDisabled("%s", tool.description);
    else ImGui::TextColored({0.95f, 0.70f, 0.35f, 1.0f}, "planned for a later milestone");

    ImGui::SameLine(std::max(ImGui::GetCursorPosX() + 12.0f, 600.0f));
    const char* uName = st.plane == SketchPlane::Right ? "Y" : "X";
    const char* vName = st.plane == SketchPlane::Top ? "Y" : "Z";
    auto clean = [](float v) { return std::fabs(v) < 0.005f ? 0.0f : v; }; // no "-0.00"
    if (st.cursorPlane)
        ImGui::Text("%s %8.2f   %s %8.2f mm", uName, clean(st.cursorPlane->x), vName, clean(st.cursorPlane->y));
    else
        ImGui::TextDisabled("%s     --      %s     --   mm", uName, vName);
    ImGui::SameLine();
    ImGui::TextDisabled("|  %s  |  Bodies %zu  Curves %zu", st.mode == ViewMode::Drawing2D ? "2D Drawing" : "3D Model",
                        scene.bodies().size(), scene.sketch().curves().size());
    (void)cam;
    char perf[64];
    std::snprintf(perf, sizeof(perf), "%.2f ms  |  %s", st.frameMs, st.glVersion.c_str());
    ImGui::SameLine(ImGui::GetWindowWidth() - ImGui::CalcTextSize(perf).x - 12.0f);
    ImGui::TextDisabled("%s", perf);
    m_statusHeight = h;
    ImGui::End();
    ImGui::PopStyleVar(2);
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
                {"V  S  O  H  Z", "Move / Select / Orbit / Pan / Zoom tools"},
                {"L  C  R", "Line / Circle / Rectangle sketch tools"},
                {"B  Y", "Block / Cylinder tools (Shift cycles the slot)"},
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

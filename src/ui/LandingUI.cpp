#include "ui/LandingUI.hpp"

#include "core/Command.hpp"
#include "core/Library.hpp"
#include "core/Physics.hpp"
#include "core/Pose.hpp"
#include "core/Units.hpp"
#include "ui/Theme.hpp"
#include "ui/ToolIcons.hpp"

#include <filesystem>

#include <imgui.h>

#include <algorithm>
#include <functional>
#include <array>
#include <map>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace sicdm {

namespace {

constexpr float kRulerSize = 20.0f;
constexpr ImU32 kAccent = theme::kAccent;

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

// Visibility toggle drawn as an eye (open / closed), like the layer lists of DCC tools.
bool eyeToggle(const char* id, bool& visible) {
    const float h = ImGui::GetTextLineHeight();
    ImGui::InvisibleButton(id, {h + 4, h});
    const bool clicked = ImGui::IsItemClicked();
    if (clicked) visible = !visible;
    const ImVec2 lo = ImGui::GetItemRectMin(), hi = ImGui::GetItemRectMax();
    const ImVec2 c{(lo.x + hi.x) * 0.5f, (lo.y + hi.y) * 0.5f};
    const bool hov = ImGui::IsItemHovered();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImU32 col = visible ? (hov ? theme::kText : theme::kTextDim) : (hov ? theme::kTextDim : theme::kTextFaint);
    if (visible) {
        dl->PathArcTo({c.x, c.y + 4.5f}, 7.5f, 3.6f, 5.82f, 10); // upper lid
        dl->PathArcTo({c.x, c.y - 4.5f}, 7.5f, 0.46f, 2.68f, 10); // lower lid
        dl->PathStroke(col, ImDrawFlags_Closed, 1.3f);
        dl->AddCircleFilled(c, 2.3f, col);
    } else {
        dl->PathArcTo({c.x, c.y - 4.5f}, 7.5f, 0.46f, 2.68f, 10); // closed lid
        dl->PathStroke(col, 0, 1.3f);
    }
    if (hov) ImGui::SetTooltip(visible ? "Hide" : "Show");
    return clicked;
}

// Label / value row of a property grid (label column in the dim colour).
void propertyRow(const char* label, const char* fmt, ...) IM_FMTARGS(2);
void propertyRow(const char* label, const char* fmt, ...) {
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextColored(theme::vec(theme::kTextDim), "%s", label);
    ImGui::TableNextColumn();
    va_list args;
    va_start(args, fmt);
    ImGui::TextV(fmt, args);
    va_end(args);
}

// Editor for one library parameter: a list for choices, a drag field otherwise.
// A length field: shows `mm` in the document units and takes any typed length
// ("12.5", "3in", "5'6-1/2\"", "1/8\"", "10pt", "1m+25mm"), committed on Enter or
// when the field loses focus. `suffix` follows the unit ("/s" for speeds).
bool lengthField(const char* id, float* mm, float minMm = -1e7f, float maxMm = 1e7f, const char* suffix = "",
                 const char* emptyText = nullptr) {
    // Text being typed, per field: moving straight to another field (even one drawn
    // earlier in the frame) must not lose this one's edit before it commits.
    using Text = std::array<char, 96>;
    static std::map<ImGuiID, Text> s_editing;
    static ImGuiID s_errorId = 0;
    static double s_errorUntil = 0.0;
    const ImGuiID wid = ImGui::GetID(id);
    Text local{};
    const auto editing = s_editing.find(wid);
    char* buf = editing != s_editing.end() ? editing->second.data() : local.data();
    if (editing == s_editing.end()) {
        const std::string text = emptyText && *mm <= 0.0f ? std::string(emptyText) : lengthText(*mm) + suffix;
        std::snprintf(local.data(), local.size(), "%s", text.c_str());
    }
    const bool error = s_errorId == wid && ImGui::GetTime() < s_errorUntil;
    if (error) ImGui::PushStyleColor(ImGuiCol_FrameBg, theme::vec(theme::rgb(92, 38, 40)));
    ImGui::InputText(id, buf, local.size(), ImGuiInputTextFlags_AutoSelectAll);
    if (error) ImGui::PopStyleColor();
    if (ImGui::IsItemActivated()) s_editing[wid] = editing != s_editing.end() ? editing->second : local;
    bool changed = false;
    if (ImGui::IsItemDeactivated() && s_editing.count(wid)) {
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            std::string text = s_editing[wid].data();
            const std::size_t sl = std::strlen(suffix);
            if (sl && text.size() >= sl && text.compare(text.size() - sl, sl, suffix) == 0) text.resize(text.size() - sl);
            const auto v = emptyText && text == emptyText ? std::optional<double>(0.0) : parseLength(text);
            if (v && std::isfinite(*v)) {
                *mm = static_cast<float>(std::clamp(*v, static_cast<double>(minMm), static_cast<double>(maxMm)));
                changed = true;
            } else {
                s_errorId = wid;
                s_errorUntil = ImGui::GetTime() + 1.2;
            }
        }
        s_editing.erase(wid);
    }
    if (ImGui::IsItemHovered() && !s_editing.count(wid))
        ImGui::SetTooltip("%s\nType a length in any unit: 12.5  3in  5'6-1/2\"  1/8\"  10pt  1m+25mm",
                          error ? "Not a length" : lengthText(*mm, {true, false}).c_str());
    return changed;
}

// Unit choices, shared by View > Units and the status bar chip. Each choice runs
// a UNITS command, so it is logged, marks the file changed and saves with it.
void unitsMenu(UiCommands& cmd) {
    const UnitSettings& u = activeUnits();
    ImGui::TextDisabled("Lengths stay exact in mm; this sets typing and display");
    ImGui::Separator();
    struct Choice {
        LengthUnit unit;
        const char* label;
        const char* word;
    };
    static constexpr Choice kChoices[] = {
        {LengthUnit::Millimeter, "Millimeters", "MM"}, {LengthUnit::Centimeter, "Centimeters", "CM"},
        {LengthUnit::Meter, "Meters", "M"},            {LengthUnit::Inch, "Inches", "IN"},
        {LengthUnit::Foot, "Feet", "FT"},              {LengthUnit::Point, "Points (1/72 in)", "PT"},
    };
    for (const Choice& c : kChoices)
        if (ImGui::MenuItem(c.label, unitInfo(c.unit).symbol, u.unit == c.unit && !(u.fractional())))
            cmd.commands.push_back(std::string("UNITS ") + c.word + " FRACTION OFF");
    if (ImGui::MenuItem("Feet-Inches (5'-6 1/2\")", nullptr, u.unit == LengthUnit::Foot && u.fractional()))
        cmd.commands.push_back("UNITS FT FRACTION 1/" + std::to_string(u.denominator));
    if (ImGui::MenuItem("Fractional Inches (3-1/2\")", nullptr, u.unit == LengthUnit::Inch && u.fractional()))
        cmd.commands.push_back("UNITS IN FRACTION 1/" + std::to_string(u.denominator));
    ImGui::Separator();
    if (ImGui::BeginMenu("Precision", !u.fractional())) {
        if (ImGui::MenuItem("Auto", nullptr, u.precision < 0)) cmd.commands.push_back("UNITS PRECISION AUTO");
        for (int d = 0; d <= 8; ++d) {
            char label[32];
            std::snprintf(label, sizeof(label), "%s", formatDecimal(0.0, d, false).c_str());
            if (ImGui::MenuItem(label, nullptr, u.precision == d)) cmd.commands.push_back("UNITS PRECISION " + std::to_string(d));
        }
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Fraction Precision", u.unit == LengthUnit::Inch || u.unit == LengthUnit::Foot)) {
        for (int den = 2; den <= 256; den *= 2) {
            const std::string label = "1/" + std::to_string(den) + " in";
            if (ImGui::MenuItem(label.c_str(), nullptr, u.fractional() && u.denominator == den))
                cmd.commands.push_back("UNITS FRACTION 1/" + std::to_string(den));
        }
        ImGui::EndMenu();
    }
    ImGui::Separator();
    ImGui::TextDisabled("Resolution  %s", formatLength(u.resolutionMm(), UnitSettings{LengthUnit::Millimeter, 6}).c_str());
}

bool paramEditor(const LibraryParam& p, float& value) {
    ImGui::SetNextItemWidth(-1);
    ImGui::PushID(p.key);
    bool changed = false;
    if (p.choices) {
        std::vector<std::string> names;
        std::string list = std::string(p.choices) + "|";
        for (std::size_t at = 0, bar; (bar = list.find('|', at)) != std::string::npos; at = bar + 1)
            names.push_back(list.substr(at, bar - at));
        const int cur = std::clamp(static_cast<int>(value), 0, static_cast<int>(names.size()) - 1);
        if (ImGui::BeginCombo("##v", names[static_cast<std::size_t>(cur)].c_str())) {
            for (int i = 0; i < static_cast<int>(names.size()); ++i)
                if (ImGui::Selectable(names[static_cast<std::size_t>(i)].c_str(), i == cur)) {
                    value = static_cast<float>(i);
                    changed = true;
                }
            ImGui::EndCombo();
        }
    } else if (p.unit && std::string_view(p.unit) == "mm" && !p.integer) {
        changed = lengthField("##v", &value, p.min, p.max);
    } else {
        char fmt[32];
        std::snprintf(fmt, sizeof(fmt), p.integer ? "%%.0f %s" : "%%.2f %s", p.unit);
        const float speed = p.integer ? 0.1f : std::max((p.max - p.min) / 400.0f, 0.01f);
        changed = ImGui::DragFloat("##v", &value, speed, p.min, p.max, fmt, ImGuiSliderFlags_AlwaysClamp);
        if (p.integer) value = std::round(value);
    }
    ImGui::PopID();
    return changed;
}

bool beginPropertyGrid(const char* id) {
    if (!ImGui::BeginTable(id, 2, ImGuiTableFlags_SizingFixedFit)) return false;
    ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthFixed, 82.0f);
    ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthStretch);
    return true;
}

// One part of the 2D / 3D / Simulation segmented switch. Returns true when clicked.
bool segment(const char* label, bool active, ImDrawFlags corners, void (*icon)(ImDrawList*, ImVec2, ImU32)) {
    const float h = ImGui::GetFrameHeight();
    const float width = 36.0f + ImGui::CalcTextSize(label).x + 14.0f; // icon gutter + label + padding
    ImGui::InvisibleButton(label, {width, h});
    const ImVec2 lo = ImGui::GetItemRectMin(), hi = ImGui::GetItemRectMax();
    const bool hovered = ImGui::IsItemHovered();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImU32 bg = active ? theme::kAccent : (hovered ? theme::kHover : theme::kPanelRaised);
    dl->AddRectFilled(lo, hi, bg, 4.0f, corners);
    const ImU32 fg = active ? IM_COL32(255, 255, 255, 255) : theme::kTextDim;
    icon(dl, {lo.x + 20, (lo.y + hi.y) * 0.5f}, fg);
    ImFont* f = active ? theme::font(theme::Font::Strong) : ImGui::GetFont();
    dl->AddText(f, ImGui::GetFontSize(), {lo.x + 36, lo.y + (h - ImGui::GetFontSize()) * 0.5f}, active ? fg : theme::kText,
                label);
    return ImGui::IsItemClicked();
}

// Thin vertical divider between groups of a bar.
void barDivider(float height) {
    ImGui::SameLine(0, 10);
    const ImVec2 p = ImGui::GetCursorScreenPos();
    ImGui::GetWindowDrawList()->AddLine({p.x, p.y + 4}, {p.x, p.y + height - 4}, theme::kLine);
    ImGui::Dummy({1, height});
    ImGui::SameLine(0, 10);
}

void icon2D(ImDrawList* dl, ImVec2 c, ImU32 col) {
    // Sheet with a drawn profile.
    dl->AddRect({c.x - 7, c.y - 6}, {c.x + 7, c.y + 6}, col, 1.0f, 0, 1.2f);
    dl->AddLine({c.x - 4, c.y + 3}, {c.x - 1, c.y - 3}, col, 1.2f);
    dl->AddLine({c.x - 1, c.y - 3}, {c.x + 4, c.y + 3}, col, 1.2f);
}

void iconSim(ImDrawList* dl, ImVec2 c, ImU32 col) {
    // Orbiting part around an axis with a motion arrow.
    dl->AddLine({c.x, c.y - 8}, {c.x, c.y + 8}, col, 1.2f);
    dl->PathArcTo({c.x, c.y}, 6.5f, kPi * 0.15f, kPi * 1.65f, 16);
    dl->PathStroke(col, 0, 1.4f);
    const ImVec2 tip{c.x + 6.5f * std::cos(kPi * 1.65f), c.y + 6.5f * std::sin(kPi * 1.65f)};
    dl->AddTriangleFilled({tip.x + 3.5f, tip.y + 0.5f}, {tip.x - 1.0f, tip.y - 3.0f}, {tip.x - 0.5f, tip.y + 3.0f}, col);
    dl->AddCircleFilled({c.x + 6.2f, c.y + 2.0f}, 2.0f, col);
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
    drawStatusBar(cmd, scene, cam, st);
    drawCommandBar(cmd, st);
    const float railTop = mv->WorkPos.y + m_optionsHeight;
    const float bottom = mv->WorkPos.y + mv->WorkSize.y - m_statusHeight - m_commandHeight;
    m_toolboxWidth = m_toolbox.draw(railTop, bottom, st.mode == ViewMode::Drawing2D, st.showModelTree, st.dockTab);
    // The Model panel docks beside the rail; tabs, rulers and canvas start after it.
    m_leftDockWidth = m_toolboxWidth;
    if (st.showModelTree) m_leftDockWidth += drawModelTree(cmd, scene, st, mv->WorkPos.x + m_toolboxWidth, railTop, bottom);
    drawFileTabs(cmd, st);
    // The Simulation window docks the motion study on the right.
    const float rightDock = st.mode == ViewMode::Simulation ? drawMotionPanel(scene, cmd, st, m_tabsBottom, bottom) : 0.0f;

    const bool is2D = st.mode == ViewMode::Drawing2D;
    m_canvasMin = {mv->WorkPos.x + m_leftDockWidth + (is2D ? kRulerSize : 0.0f), m_tabsBottom + (is2D ? kRulerSize : 0.0f)};
    m_canvasMax = {mv->WorkPos.x + mv->WorkSize.x - rightDock, bottom};
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
        if (is2D) cmd.mode = ViewMode::Model3D; // the Simulation window keeps its mode: it shares the 3D camera
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
    // 26 pt bar, menus with 24 pt rows (AutoCAD / Photoshop proportions).
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, {10, (theme::kMenuBarHeight - ImGui::GetFontSize()) * 0.5f});
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, {12, 8});
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {8, 8});
    ImGui::PushStyleVar(ImGuiStyleVar_PopupRounding, 6.0f);
    const bool open = ImGui::BeginMainMenuBar();
    if (!open) {
        ImGui::PopStyleVar(4);
        return;
    }
    m_menuHeight = ImGui::GetWindowSize().y;
    const bool is2D = st.mode == ViewMode::Drawing2D;

    if (ImGui::BeginMenu("File")) {
        if (ImGui::MenuItem("New Project", "Ctrl+N")) requestFile(-1, st);
        if (ImGui::MenuItem("Open Project...", "Ctrl+O")) requestFile(FileOpen, st);
        if (ImGui::MenuItem("Close", "Ctrl+W")) requestClose(cmd, st, st.activeFile);
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
        if (ImGui::MenuItem("3D Model Window", "Tab", st.mode == ViewMode::Model3D)) cmd.mode = ViewMode::Model3D;
        if (ImGui::MenuItem("Simulation Window", "Shift+Tab", st.mode == ViewMode::Simulation)) cmd.mode = ViewMode::Simulation;
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
        if (ImGui::BeginMenu("Units")) {
            unitsMenu(cmd);
            ImGui::EndMenu();
        }
        ImGui::Separator();
        ImGui::MenuItem("Grid", "G", &vp.showGrid);
        if (ImGui::MenuItem("Model Panel", "M", st.showModelTree && st.dockTab == 0)) {
            st.showModelTree = !(st.showModelTree && st.dockTab == 0);
            st.dockTab = 0;
        }
        if (ImGui::MenuItem("Library", "K", st.showModelTree && st.dockTab == 1)) {
            st.showModelTree = !(st.showModelTree && st.dockTab == 1);
            st.dockTab = 1;
        }
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

    {
        theme::ScopedFont small(theme::Font::Small);
        const char* gpu = st.gpuName.c_str();
        const float w = ImGui::CalcTextSize(gpu).x;
        ImGui::SameLine(ImGui::GetWindowWidth() - w - 14.0f);
        ImGui::TextColored(theme::vec(theme::kTextFaint), "%s", gpu);
    }
    ImGui::EndMainMenuBar();
    ImGui::PopStyleVar(4);
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
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {10, (theme::kOptionsBarHeight - ImGui::GetFrameHeight()) * 0.5f});
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
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
            if (st.mode == ViewMode::Drawing2D) cmd.mode = ViewMode::Model3D; // Simulation stays Simulation
        }
        const ImU32 ic = IM_COL32(210, 214, 222, 255);
        dl->AddTriangleFilled({c.x - 8, c.y - 1}, {c.x, c.y - 8}, {c.x + 8, c.y - 1}, ic);
        dl->AddRectFilled({c.x - 6, c.y - 1}, {c.x + 6, c.y + 7}, ic);
        dl->AddRectFilled({c.x - 2, c.y + 2}, {c.x + 2, c.y + 7}, IM_COL32(28, 30, 36, 255));
    }
    auto divider = [&] { barDivider(fh); };
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

    // Tool options (the Simulation window shows playback instead).
    ImGui::AlignTextToFramePadding();
    if (st.mode == ViewMode::Simulation) {
        drawSimTransport(st);
    } else if (isSketchTool(tool.id)) {
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
        if (lengthField("##extdist", &st.extrudeDistance, -1e6f, 1e6f))
            st.extrudeLive = false; // typed in the field: stop following the mouse
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
    } else if (isSculptTool(tool.id)) {
        ImGui::TextDisabled("Radius");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(70);
        lengthField("##sradius", &st.sculptRadius, 0.1f, 1e5f);
        ImGui::SameLine();
        ImGui::TextDisabled("Strength");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80);
        ImGui::SliderFloat("##sstrength", &st.sculptStrength, 0.0f, 1.0f, "%.2f", ImGuiSliderFlags_AlwaysClamp);
        ImGui::SameLine();
        ImGui::TextDisabled("Symmetry");
        ImGui::SameLine();
        ImGui::Checkbox("X##symx", &st.sculptSymmetry);
        ImGui::SameLine();
        ImGui::Checkbox("Y##symy", &st.sculptSymmetryY);
        ImGui::SameLine();
        ImGui::Checkbox("Z##symz", &st.sculptSymmetryZ);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(60);
        ImGui::InputInt("##radial", &st.sculptRadial, 1, 4);
        st.sculptRadial = std::clamp(st.sculptRadial, 1, 64);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Radial symmetry: copies of each dab around the body's vertical axis (1: none)");
        ImGui::SameLine();
        {
            static const char* kFalloffs[] = {"Smooth", "Sphere", "Sharp", "Linear", "Flat"};
            ImGui::SetNextItemWidth(78);
            ImGui::Combo("##falloff", &st.sculptFalloff, kFalloffs, 5);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Brush falloff from its centre to its rim");
        }
        if (tool.id == ToolId::SculptMask) {
            ImGui::SameLine();
            if (ImGui::Button("Clear mask")) cmd.commands.push_back("MASK CLEAR");
            ImGui::SameLine();
            if (ImGui::Button("Invert")) cmd.commands.push_back("MASK INVERT");
            ImGui::SameLine();
            if (ImGui::Button("Blur")) cmd.commands.push_back("MASK SMOOTH");
        }
        ImGui::SameLine();
        ImGui::Checkbox("Dyntopo", &st.sculptRefine);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Dynamic topology: split long edges under the brush so detail can be added anywhere");
        if (st.sculptRefine) {
            ImGui::SameLine();
            ImGui::TextDisabled("Detail");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(60);
            lengthField("##sdetail", &st.sculptDetail, 0.0f, 1e5f);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Longest edge left under the brush (0: a quarter of the radius)");
        }
        ImGui::SameLine();
        if (ImGui::Button("Undo")) cmd.commands.push_back("UNDO");
        ImGui::SameLine();
        if (ImGui::Button("Redo")) cmd.commands.push_back("REDO");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Drag over a body. Ctrl inverts the brush, Shift smooths, [ and ] change the size.%s%s",
                              st.sculptInfo.empty() ? "" : "\n", st.sculptInfo.c_str());
    } else if (isCutTool(tool.id)) {
        auto shapeCombo = [](const char* id, int* v, std::initializer_list<const char*> names) {
            ImGui::SetNextItemWidth(90);
            const char* items[4];
            int n = 0;
            for (const char* nm : names) items[n++] = nm;
            ImGui::Combo(id, v, items, n);
        };
        auto field = [&](const char* label, const char* id, float* v, float lo) {
            ImGui::TextDisabled("%s", label);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(80);
            lengthField(id, v, lo, 1e5f);
            ImGui::SameLine();
        };
        switch (tool.id) {
        case ToolId::CutHole:
            field("Diameter", "##hd", &st.holeDiameter, 0.01f);
            ImGui::Checkbox("Through", &st.holeThrough);
            ImGui::SameLine();
            if (!st.holeThrough) field("Depth", "##hdepth", &st.holeDepth, 0.01f);
            ImGui::TextDisabled("Click a surface");
            break;
        case ToolId::CutPocket:
            shapeCombo("##pshape", &st.pocketShape, {"Box", "Round", "Ball end"});
            ImGui::SameLine();
            field("Width", "##pw", &st.pocketWidth, 0.01f);
            if (st.pocketShape == 0) field("Length", "##pl", &st.pocketLength, 0.01f);
            field("Depth", "##pd", &st.pocketDepth, 0.01f);
            if (st.pocketShape == 0) {
                ImGui::TextDisabled("Angle");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(70);
                ImGui::DragFloat("##pa", &st.pocketAngle, 1.0f, -180.0f, 180.0f, "%.0f deg");
                ImGui::SameLine();
            }
            ImGui::TextDisabled("Click a surface");
            break;
        case ToolId::CutChannel:
            field("Width", "##cw", &st.channelWidth, 0.01f);
            field("Depth", "##cd", &st.channelDepth, 0.0f);
            ImGui::TextDisabled("Drag along a surface (depth 0: a round groove)");
            break;
        case ToolId::CutKnife:
            ImGui::TextDisabled("Drag a line across a body: the smaller side is cut away and the face capped");
            break;
        case ToolId::AddBoss:
            shapeCombo("##bshape", &st.bossShape, {"Cylinder", "Box", "Dome"});
            ImGui::SameLine();
            field("Diameter", "##bd", &st.bossDiameter, 0.01f);
            field("Height", "##bh", &st.bossHeight, 0.01f);
            ImGui::TextDisabled("Click a surface");
            break;
        default: // BooleanBodies
            shapeCombo("##bop", &st.booleanOp, {"Union", "Subtract", "Intersect"});
            ImGui::SameLine();
            ImGui::Checkbox("Keep tool body", &st.booleanKeep);
            ImGui::SameLine();
            if (st.booleanFirst.empty()) ImGui::TextDisabled("Click the body to keep");
            else ImGui::TextColored({0.55f, 0.78f, 1.0f, 1.0f}, "%s  -  now click the tool body", st.booleanFirst.c_str());
            break;
        }
        ImGui::SameLine();
        if (ImGui::Button("Undo")) cmd.commands.push_back("UNDO");
        ImGui::SameLine();
        if (ImGui::Button("Redo")) cmd.commands.push_back("REDO");
    } else if (isWearableTool(tool.id)) {
        auto field = [&](const char* label, const char* id, float* v, float lo) {
            ImGui::TextDisabled("%s", label);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(70);
            lengthField(id, v, lo, 1e5f);
            ImGui::SameLine();
        };
        switch (tool.id) {
        case ToolId::ShellBody:
            field("Wall", "##sw", &st.shellThickness, 0.05f);
            ImGui::Checkbox("Outward", &st.shellOutward);
            ImGui::SameLine();
            ImGui::Checkbox("Open the clicked side", &st.shellOpen);
            ImGui::SameLine();
            if (st.shellOpen) field("Cut depth", "##sod", &st.shellOpenDepth, 0.0f);
            ImGui::TextDisabled("Click a body");
            break;
        case ToolId::ArmorBody:
            field("Clearance", "##ac", &st.armorClearance, 0.0f);
            field("Wall", "##at", &st.armorThickness, 0.05f);
            ImGui::TextDisabled("Click a body part");
            break;
        case ToolId::JointBody: {
            ImGui::SetNextItemWidth(80);
            const char* axes[] = {"Axis X", "Axis Y", "Axis Z", "View axis"};
            ImGui::Combo("##jaxis", &st.jointAxis, axes, 4);
            ImGui::SameLine();
            ImGui::Checkbox("Ball", &st.jointBall);
            ImGui::SameLine();
            ImGui::TextDisabled("Range");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(120);
            ImGui::DragFloatRange2("##jrange", &st.jointLo, &st.jointHi, 1.0f, -180.0f, 180.0f, "%.0f", "%.0f deg",
                                   ImGuiSliderFlags_AlwaysClamp);
            ImGui::SameLine();
            ImGui::TextDisabled("Period");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(60);
            ImGui::DragFloat("##jper", &st.jointPeriod, 0.1f, 0.2f, 60.0f, "%.1f s", ImGuiSliderFlags_AlwaysClamp);
            ImGui::SameLine();
            ImGui::TextDisabled("Click the moving part at its pivot");
            break;
        }
        case ToolId::InterferenceCheck:
            field("Clearance", "##ic", &st.interferenceClearance, 0.0f);
            ImGui::TextDisabled("Click to sweep the motions");
            break;
        default: // Lighten / TPMS lattice
            field("Skin", "##ls", &st.latticeSkin, 0.1f);
            ImGui::Checkbox("Hollow only", &st.latticeHollow);
            ImGui::SameLine();
            if (!st.latticeHollow) {
                field("Cell", "##lc", &st.latticeCell, 1.0f);
                field("Wall", "##lw", &st.latticeWall, 0.1f);
            }
            ImGui::TextDisabled("Click a solid body");
            break;
        }
        ImGui::SameLine();
        if (ImGui::Button("Undo")) cmd.commands.push_back("UNDO");
        ImGui::SameLine();
        if (ImGui::Button("Redo")) cmd.commands.push_back("REDO");
    } else if (isPrimitiveTool(tool.id)) {
        const std::string snap = lengthText(5.0);
        if (is2D) ImGui::TextDisabled("Click to place on the %s plane (%s snap)", shortPlaneName(st.plane), snap.c_str());
        else ImGui::TextDisabled("Click the grid or a face to place (%s snap)", snap.c_str());
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
    const bool isSim = st.mode == ViewMode::Simulation;
    if (segment("2D Drawing", is2D, ImDrawFlags_RoundCornersLeft, icon2D) && !is2D) cmd.mode = ViewMode::Drawing2D;
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("2D drawing window: plan view of the %s plane  [Tab]", shortPlaneName(st.plane));
    ImGui::SameLine(0, 2);
    if (segment("3D Model", st.mode == ViewMode::Model3D, ImDrawFlags_RoundCornersNone, icon3D) &&
        st.mode != ViewMode::Model3D)
        cmd.mode = ViewMode::Model3D;
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("3D model window  [Tab]");
    ImGui::SameLine(0, 2);
    if (segment("Simulation", isSim, ImDrawFlags_RoundCornersRight, iconSim) && !isSim)
        cmd.mode = ViewMode::Simulation;
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Simulation window: circular and linear motion, live axes  [Shift+Tab]");
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
    {
        const ImVec2 wp = ImGui::GetWindowPos(), ws = ImGui::GetWindowSize();
        dl->AddLine({wp.x, wp.y + ws.y - 1}, {wp.x + ws.x, wp.y + ws.y - 1}, theme::kBorder);
    }
    ImGui::End();
    ImGui::PopStyleVar(3);
}

void LandingUI::drawFileTabs(UiCommands& cmd, const UiState& st) {
    // One tab per open file at the top of the canvas (AutoCAD style): the file
    // menu, the files (with * when unsaved, x to close) and + for a new file.
    const ImGuiViewport* mv = ImGui::GetMainViewport();
    const float x = mv->WorkPos.x + m_leftDockWidth;
    const float y = mv->WorkPos.y + m_optionsHeight;
    ImGui::SetNextWindowPos({x, y});
    ImGui::SetNextWindowSize({mv->WorkSize.x - m_leftDockWidth, 0.0f});
    constexpr ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                                       ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus |
                                       ImGuiWindowFlags_AlwaysAutoResize;
    // Document tabs sit on the chrome colour; the active one takes the panel colour
    // with an accent overline and reads as part of the canvas below.
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {6, 4});
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, {14, 6});
    ImGui::PushStyleVar(ImGuiStyleVar_ItemInnerSpacing, {8, 4});
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, {2, 0});
    ImGui::PushStyleColor(ImGuiCol_WindowBg, theme::vec(theme::kChrome));
    ImGui::Begin("##tabs", nullptr, flags);

    const int activeId = st.activeFile >= 0 && st.activeFile < static_cast<int>(st.files.size())
                             ? st.files[static_cast<std::size_t>(st.activeFile)].id
                             : -1;
    if (ImGui::BeginTabBar("##files", ImGuiTabBarFlags_FittingPolicyScroll | ImGuiTabBarFlags_NoTooltip |
                                                ImGuiTabBarFlags_DrawSelectedOverline)) {
        // File menu: three bars drawn over a blank leading button.
        if (ImGui::TabItemButton("    ##filemenu", ImGuiTabItemFlags_Leading | ImGuiTabItemFlags_NoTooltip))
            ImGui::OpenPopup("##filesMenu");
        {
            const ImVec2 a = ImGui::GetItemRectMin(), b = ImGui::GetItemRectMax();
            const float cx = (a.x + b.x) * 0.5f, cy = (a.y + b.y) * 0.5f;
            const ImU32 c = ImGui::GetColorU32(ImGuiCol_Text);
            for (float dy : {-4.0f, 0.0f, 4.0f}) ImGui::GetWindowDrawList()->AddLine({cx - 6, cy + dy}, {cx + 6, cy + dy}, c, 1.5f);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Files");
        }
        if (ImGui::BeginPopup("##filesMenu")) {
            if (ImGui::MenuItem("New", "Ctrl+N")) cmd.commands.push_back("NEW");
            if (ImGui::MenuItem("Open...", "Ctrl+O")) requestFile(FileOpen, st);
            if (ImGui::MenuItem("Save", "Ctrl+S")) {
                if (st.projectPath.empty()) requestFile(FileSaveAs, st);
                else cmd.commands.push_back("SAVE");
            }
            if (ImGui::MenuItem("Save As...", "Ctrl+Shift+S")) requestFile(FileSaveAs, st);
            if (ImGui::MenuItem("Close", "Ctrl+W")) requestClose(cmd, st, st.activeFile);
            ImGui::Separator();
            for (const auto& f : st.files) {
                const std::string label = f.name + (f.dirty ? "*" : "") + "###menufile" + std::to_string(f.id);
                if (ImGui::MenuItem(label.c_str(), nullptr, f.id == activeId)) cmd.switchFile = f.id;
            }
            ImGui::EndPopup();
        }

        for (std::size_t i = 0; i < st.files.size(); ++i) {
            const auto& f = st.files[i];
            const std::string label = f.name + (f.dirty ? "*" : "") + "###file" + std::to_string(f.id);
            bool open = true;
            // Tabs switch files only when clicked; otherwise they follow the active file.
            const ImGuiTabItemFlags tf =
                f.id == activeId && m_shownFile != activeId ? ImGuiTabItemFlags_SetSelected : ImGuiTabItemFlags_None;
            if (f.id == activeId) ImGui::PushFont(theme::font(theme::Font::Strong));
            const bool selected = ImGui::BeginTabItem(label.c_str(), &open, tf);
            if (f.id == activeId) ImGui::PopFont();
            if (ImGui::IsItemClicked() && f.id != activeId) cmd.switchFile = f.id;
            if (ImGui::IsItemClicked(ImGuiMouseButton_Middle)) requestClose(cmd, st, static_cast<int>(i));
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayNormal)) {
                ImGui::SetTooltip("%s\n%s window%s", f.path.empty() ? "Not saved yet" : f.path.c_str(),
                                  f.mode == ViewMode::Drawing2D   ? "2D Drawing"
                                  : f.mode == ViewMode::Simulation ? "Simulation"
                                                                   : "3D Model",
                                  f.dirty ? "\nUnsaved changes" : "");
            }
            if (selected) {
                if (f.id == activeId) m_shownFile = activeId;
                ImGui::EndTabItem();
            }
            if (!open) requestClose(cmd, st, static_cast<int>(i));
        }
        if (ImGui::TabItemButton("+", ImGuiTabItemFlags_Trailing | ImGuiTabItemFlags_NoTooltip))
            cmd.commands.push_back("NEW");
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("New file (Ctrl+N)");
        ImGui::EndTabBar();
    }
    m_tabsBottom = y + ImGui::GetWindowSize().y;
    ImGui::End();
    ImGui::PopStyleColor();
    ImGui::PopStyleVar(6);
}

void LandingUI::requestClose(UiCommands& cmd, const UiState& st, int index) {
    if (index < 0 || index >= static_cast<int>(st.files.size())) return;
    const auto& f = st.files[static_cast<std::size_t>(index)];
    if (!f.dirty) {
        cmd.closeFile = f.id;
        return;
    }
    // Unsaved: show the file, then ask whether to save it.
    cmd.switchFile = f.id;
    m_closeAsk = f.id;
    m_closeAskOpen = true;
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

    // Ticks and labels in the document units.
    const UnitSettings& units = activeUnits();
    // Feet rulers read in inches, as on a drafting scale.
    const LengthUnit rulerUnit = units.unit == LengthUnit::Foot ? LengthUnit::Inch : units.unit;
    const float perUnit = static_cast<float>(toMillimeters(1.0, rulerUnit)); // mm per ruler unit
    const float major = niceStep(wpp / perUnit * 70.0f);
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
    for (float u = std::floor(uAt(m_canvasMin.x) / perUnit / minor) * minor; u <= uAt(m_canvasMax.x) / perUnit; u += minor) {
        const float x = xOf(u * perUnit);
        const long k = std::lround(u / minor);
        const float len = k % 10 == 0 ? kRulerSize : (k % 5 == 0 ? 8.0f : 4.0f);
        dl->AddLine({x, m_canvasMin.y - len}, {x, m_canvasMin.y}, tick);
        if (k % 10 == 0) dl->AddText(font, fs, {x + 3, y0 + 2}, text, fmt(u));
    }
    dl->PopClipRect();

    // Left ruler (v), labels written top-to-bottom like Photoshop.
    dl->PushClipRect({x0, m_canvasMin.y}, {m_canvasMin.x, m_canvasMax.y}, true);
    for (float v = std::floor(vAt(m_canvasMax.y) / perUnit / minor) * minor; v <= vAt(m_canvasMin.y) / perUnit; v += minor) {
        const float y = yOf(v * perUnit);
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
    dl->AddText(font, fs, {x0 + 2, y0 + 3}, text, unitInfo(rulerUnit).symbol);
}

float LandingUI::drawModelTree(UiCommands& cmd, Scene& scene, UiState& st, float x, float top, float bottom) {
    // Docked full-height panel; only its right edge can be dragged to resize.
    const float height = std::max(bottom - top, 100.0f);
    ImGui::SetNextWindowPos({x, top}, ImGuiCond_Always);
    ImGui::SetNextWindowSizeConstraints({200, height}, {440, height});
    ImGui::SetNextWindowSize({260, height}, ImGuiCond_FirstUseEver);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    constexpr ImGuiWindowFlags flags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                                       ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus;
    const bool visible = ImGui::Begin("##leftdock", nullptr, flags);
    ImGui::PopStyleVar();
    const float width = ImGui::GetWindowWidth();
    if (visible) {
        // Page tabs (Model | Library) with a close button, like a docked DCC panel.
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, {12, 5});
        if (ImGui::BeginTabBar("##dockpages", ImGuiTabBarFlags_DrawSelectedOverline)) {
            const char* pages[] = {"Model", "Library"};
            for (int p = 0; p < 2; ++p) {
                const ImGuiTabItemFlags f = st.dockTab == p && m_shownDockTab != p ? ImGuiTabItemFlags_SetSelected : 0;
                const bool selected = ImGui::BeginTabItem(pages[p], nullptr, f);
                if (ImGui::IsItemClicked()) st.dockTab = p; // tabs switch pages only when clicked
                if (selected) {
                    m_shownDockTab = p;
                    ImGui::EndTabItem();
                }
            }
            if (ImGui::TabItemButton("x", ImGuiTabItemFlags_Trailing | ImGuiTabItemFlags_NoTooltip)) st.showModelTree = false;
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Close panel");
            ImGui::EndTabBar();
        }
        ImGui::PopStyleVar();
    }
    if (visible && st.dockTab == 1) {
        drawLibrary(cmd, scene, st);
    } else if (visible) {
        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        if (ImGui::TreeNode("##root", "%s  (Part)", st.projectName.c_str())) {
            ImGui::SetNextItemOpen(true, ImGuiCond_Once);
            if (ImGui::TreeNode("Bodies")) {
                auto& bodies = scene.bodies();
                for (int i = 0; i < static_cast<int>(bodies.size()); ++i) {
                    Body& b = bodies[static_cast<std::size_t>(i)];
                    ImGui::PushID(i);
                    eyeToggle("##vis", b.visible);
                    ImGui::SameLine(0, 4);
                    ImGui::ColorButton("##col", {b.color.x, b.color.y, b.color.z, 1.0f},
                                       ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoBorder, {10, 10});
                    ImGui::SameLine(0, 6);
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
                    if (eyeToggle("##vis", c.visible)) scene.sketch().touch();
                    ImGui::SameLine(0, 6);
                    if (ImGui::Selectable(c.name.c_str(), st.selectedCurve == i)) {
                        st.selectedCurve = st.selectedCurve == i ? -1 : i;
                        st.selectedBody = st.selectedImage = -1;
                    }
                    ImGui::SameLine(ImGui::GetWindowWidth() - 52);
                    ImGui::TextColored(theme::vec(theme::kTextFaint), "%s", shortPlaneName(c.plane));
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
                    eyeToggle("##vis", img.visible);
                    ImGui::SameLine(0, 6);
                    if (ImGui::Selectable(img.name.c_str(), st.selectedImage == i)) {
                        st.selectedImage = st.selectedImage == i ? -1 : i;
                        st.selectedBody = st.selectedCurve = -1;
                    }
                    ImGui::SameLine(ImGui::GetWindowWidth() - 52);
                    ImGui::TextColored(theme::vec(img.pixels.empty() ? theme::kWarning : theme::kTextFaint), "%s",
                                       img.pixels.empty() ? "missing" : shortPlaneName(img.plane));
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
            theme::caption("Properties");
            if (beginPropertyGrid("##bodyprops")) {
                propertyRow("Name", "%s", b.name.c_str());
                propertyRow("Type", "%s", b.kind.c_str());
                if (!b.component.empty()) propertyRow("Component", "%s", b.component.c_str());
                // Size factor: the whole library component scales with its motions.
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::AlignTextToFramePadding();
                ImGui::TextColored(theme::vec(theme::kTextDim), "Scale");
                ImGui::TableNextColumn();
                float scale = b.scale;
                ImGui::SetNextItemWidth(-1);
                if (ImGui::InputFloat("##scale", &scale, 0.1f, 0.5f, "x %.2f", ImGuiInputTextFlags_EnterReturnsTrue) &&
                    scale > 0.01f && scale != b.scale) {
                    char line[160];
                    std::snprintf(line, sizeof(line), "SCALE \"%s\" TO %g", b.name.c_str(), static_cast<double>(scale));
                    cmd.commands.push_back(line);
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(b.component.empty() ? "Size factor (Enter or +/-)" : "Size factor of the whole component, motions included");
                propertyRow("Extents", "%s", lengthsText({bb.max.x - bb.min.x, bb.max.y - bb.min.y, bb.max.z - bb.min.z}, " x ").c_str());
                propertyRow("Position", "%s", lengthsText({b.offset.x, b.offset.y, b.offset.z}, ", ").c_str());
                propertyRow("Triangles", "%zu", b.mesh.indices.size() / 3);
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::AlignTextToFramePadding();
                ImGui::TextColored(theme::vec(theme::kTextDim), "Colour");
                ImGui::TableNextColumn();
                ImGui::ColorEdit3("##colour", &b.color.x, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_NoLabel);
                ImGui::EndTable();
            }
            // Library part parameters (a motor housing shows its rotor's shafts), rebuilt live.
            Body* part = &b;
            if (partParams(b.kind, b.params).empty() && !b.component.empty())
                for (Body& other : scene.bodies())
                    if (other.component == b.component && other.kind == "motor_rotor") part = &other;
            const std::vector<LibraryParam> schema = partParams(part->kind, part->params);
            if (!schema.empty()) {
                theme::caption(part->kind == "motor_rotor" ? (!part->params.empty() && std::lround(part->params[0]) == 2 ? "Horn" : "Shaft")
                                                           : "Parameters");
                std::vector<float> values = withDefaults(part->kind, part->params);
                bool changed = false;
                if (beginPropertyGrid("##partparams")) {
                    std::vector<std::size_t> order; // choice lists (Shafts, Teeth style) first
                    for (std::size_t i = 0; i < schema.size(); ++i)
                        if (schema[i].choices) order.push_back(i);
                    for (std::size_t i = 0; i < schema.size(); ++i)
                        if (!schema[i].choices) order.push_back(i);
                    for (std::size_t i : order) {
                        if (!(schema[i].min < schema[i].max)) continue; // fixed (e.g. motor style)
                        ImGui::TableNextRow();
                        ImGui::TableNextColumn();
                        ImGui::AlignTextToFramePadding();
                        ImGui::TextColored(theme::vec(theme::kTextDim), "%s", schema[i].label);
                        ImGui::TableNextColumn();
                        const bool off = std::string(schema[i].key) == "rear_length" && values.size() > 2 && values[2] < 1.5f;
                        if (off) ImGui::BeginDisabled();
                        changed = paramEditor(schema[i], values[i]) || changed;
                        if (off) ImGui::EndDisabled();
                    }
                    ImGui::EndTable();
                }
                if (changed) {
                    const std::vector<float> before = part->params;
                    part->params = values;
                    part->rebuild();
                    if (part->mesh.indices.empty()) { // keep the last valid shape
                        part->params = before;
                        part->rebuild();
                    }
                    scene.touchBodies();
                }
            }
        } else if (st.selectedCurve >= 0 && st.selectedCurve < static_cast<int>(scene.sketch().curves().size())) {
            const SketchCurve& c = scene.sketch().curves()[static_cast<std::size_t>(st.selectedCurve)];
            ImGui::Spacing();
            theme::caption("Properties");
            if (beginPropertyGrid("##curveprops")) {
                propertyRow("Name", "%s", c.name.c_str());
                propertyRow("Type", "%s", curveKindName(c.kind));
                propertyRow("Plane", "%s", planeName(c.plane));
                propertyRow("Size", "%s", c.measure().c_str());
                ImGui::EndTable();
            }
        } else if (st.selectedImage >= 0 && st.selectedImage < static_cast<int>(scene.images().size())) {
            ImageRef& img = scene.images()[static_cast<std::size_t>(st.selectedImage)];
            ImGui::Spacing();
            theme::caption("Properties");
            if (beginPropertyGrid("##imageprops")) {
                propertyRow("Name", "%s", img.name.c_str());
                propertyRow("Plane", "%s", planeName(img.plane));
                propertyRow("Pixels", "%d x %d", img.pixelWidth, img.pixelHeight);
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextColored(theme::vec(theme::kTextDim), "File");
                ImGui::TableNextColumn();
                ImGui::TextWrapped("%s", img.path.c_str());
                ImGui::EndTable();
            }
            ImGui::SetNextItemWidth(-90);
            lengthField("Width", &img.width, 0.1f, 1e6f);
            ImGui::TextDisabled("Height     %s", lengthText(img.height()).c_str());
            const float half = (ImGui::GetContentRegionAvail().x - 90.0f - ImGui::GetStyle().ItemInnerSpacing.x) * 0.5f;
            ImGui::SetNextItemWidth(half);
            lengthField("##cu", &img.center.x);
            ImGui::SameLine(0, ImGui::GetStyle().ItemInnerSpacing.x);
            ImGui::SetNextItemWidth(half);
            lengthField("Centre", &img.center.y);
            ImGui::SetNextItemWidth(-90);
            ImGui::SliderFloat("Opacity", &img.opacity, 0.0f, 1.0f, "%.2f");
            img.width = std::max(img.width, 0.1f);
        }
    }
    ImGui::End();
    return width;
}

void LandingUI::drawStatusBar(UiCommands& cmd, const Scene& scene, const OrbitCamera& cam, const UiState& st) {
    const ImGuiViewport* mv = ImGui::GetMainViewport();
    const float h = theme::kStatusBarHeight;
    ImGui::SetNextWindowPos({mv->WorkPos.x, mv->WorkPos.y + mv->WorkSize.y - h});
    ImGui::SetNextWindowSize({mv->WorkSize.x, h});
    constexpr ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                                       ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus;
    ImGui::PushFont(theme::font(theme::Font::Small));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {12, (h - ImGui::GetFontSize()) * 0.5f});
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, {8, 0});
    ImGui::PushStyleColor(ImGuiCol_WindowBg, theme::vec(theme::kChrome));
    ImGui::Begin("##status", nullptr, flags);
    {
        const ImVec2 wp = ImGui::GetWindowPos();
        ImGui::GetWindowDrawList()->AddLine(wp, {wp.x + ImGui::GetWindowWidth(), wp.y}, theme::kBorder);
    }
    auto sep = [&] {
        ImGui::SameLine(0, 12);
        const ImVec2 p = ImGui::GetCursorScreenPos();
        ImGui::GetWindowDrawList()->AddLine({p.x, p.y - 1}, {p.x, p.y + ImGui::GetFontSize() + 1}, theme::kLine);
        ImGui::Dummy({1, ImGui::GetFontSize()});
        ImGui::SameLine(0, 12);
    };
    const ToolInfo& tool = toolInfo(m_toolbox.active());
    ImGui::PushFont(theme::font(theme::Font::Strong));
    ImGui::TextUnformatted(tool.name);
    ImGui::PopFont();
    ImGui::SameLine(0, 10);
    // Long messages (file paths) are clipped so they never run into the coordinates.
    const float msgEnd = ImGui::GetWindowPos().x + 588.0f;
    const ImVec2 msgPos = ImGui::GetCursorScreenPos();
    ImGui::PushClipRect(msgPos, {msgEnd, msgPos.y + ImGui::GetFrameHeight()}, true);
    if (!st.statusMessage.empty()) ImGui::TextColored(theme::vec(theme::kAccentText), "%s", st.statusMessage.c_str());
    else if (tool.available) ImGui::TextColored(theme::vec(theme::kTextDim), "%s", tool.description);
    else ImGui::TextColored(theme::vec(theme::kWarning), "planned for a later milestone");
    ImGui::PopClipRect();

    ImGui::SameLine(600.0f);
    ImGui::PushFont(theme::font(theme::Font::Mono));
    const char* uName = st.plane == SketchPlane::Right ? "Y" : "X";
    const char* vName = st.plane == SketchPlane::Top ? "Y" : "Z";
    // Fixed-width coordinate readout in the document units, so the digits do not jump.
    const UnitSettings& units = activeUnits();
    const bool each = units.fractional();
    const LengthFormat readout{each, false};
    const char* symbol = each ? "" : unitInfo(units.unit).symbol;
    if (st.cursorPlane)
        ImGui::Text("%s %11s  %s %11s %s", uName, lengthText(st.cursorPlane->x, readout).c_str(), vName,
                    lengthText(st.cursorPlane->y, readout).c_str(), symbol);
    else
        ImGui::TextColored(theme::vec(theme::kTextFaint), "%s %11s  %s %11s %s", uName, "--", vName, "--", symbol);
    ImGui::PopFont();
    // Units chip: click to change the document units.
    ImGui::SameLine(0, 10);
    const std::string chip = !each ? std::string(unitInfo(units.unit).symbol)
                             : (units.unit == LengthUnit::Foot ? "ft-in" : "in") + std::string(" 1/") +
                                   std::to_string(units.denominator);
    ImGui::PushStyleColor(ImGuiCol_Button, theme::vec(theme::kPanelRaised));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, {6, 1});
    if (ImGui::SmallButton(chip.c_str())) ImGui::OpenPopup("##unitspopup");
    ImGui::PopStyleVar();
    ImGui::PopStyleColor();
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Units: %s\nLengths are stored exactly in mm", describeUnits(units).c_str());
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {10, 8});
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, {8, 6});
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, {8, 4});
    if (ImGui::BeginPopup("##unitspopup")) {
        unitsMenu(cmd);
        ImGui::EndPopup();
    }
    ImGui::PopStyleVar(3);
    sep();
    ImGui::TextColored(theme::vec(theme::kTextDim), "%s",
                       st.mode == ViewMode::Drawing2D   ? "2D Drawing"
                       : st.mode == ViewMode::Simulation ? "Simulation"
                                                          : "3D Model");
    sep();
    ImGui::TextColored(theme::vec(theme::kTextDim), "Bodies %zu   Curves %zu   Images %zu", scene.bodies().size(),
                       scene.sketch().curves().size(), scene.images().size());
    if (st.mcpRunning) {
        sep();
        ImGui::TextColored(theme::vec(theme::kSuccess), "MCP  :%d", st.mcpPort);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("AI agents can connect at http://127.0.0.1:%d/mcp", st.mcpPort);
    }
    (void)cam;
    char perf[64];
    std::snprintf(perf, sizeof(perf), "%.1f ms", static_cast<double>(st.frameMs));
    ImGui::SameLine(ImGui::GetWindowWidth() - ImGui::CalcTextSize(perf).x - 12.0f);
    ImGui::TextColored(theme::vec(theme::kTextFaint), "%s", perf);
    m_statusHeight = h;
    ImGui::End();
    ImGui::PopStyleColor();
    ImGui::PopStyleVar(4);
    ImGui::PopFont();
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
    return st.projectPath.empty() ? st.projectName : std::filesystem::path(st.projectPath).stem().string();
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
    const float h = theme::kCommandBarHeight;
    const float y = mv->WorkPos.y + mv->WorkSize.y - m_statusHeight - h;
    m_commandHeight = h;

    // Shortcuts: Space jumps to the command line; Ctrl/Cmd + N O S I E for files
    // (these also work while typing in the command line, but not in dialogs).
    const bool mod = commandModifierHeld();
    if (!io.WantCaptureKeyboard && !mod && ImGui::IsKeyPressed(ImGuiKey_Space, false) && st.mode != ViewMode::Simulation)
        m_focusCommand = true; // (Space plays / pauses in the Simulation window)
    if (mod && !m_fileDialog.isOpen() && !ImGui::IsPopupOpen(nullptr, ImGuiPopupFlags_AnyPopupId)) {
        if (ImGui::IsKeyPressed(ImGuiKey_N, false)) requestFile(-1, st);
        if (mod && ImGui::IsKeyPressed(ImGuiKey_O, false)) requestFile(FileOpen, st);
        if (mod && ImGui::IsKeyPressed(ImGuiKey_I, false)) requestFile(FileImport, st);
        if (mod && ImGui::IsKeyPressed(ImGuiKey_E, false)) requestFile(FileExport, st);
        if (mod && ImGui::IsKeyPressed(ImGuiKey_W, false)) requestClose(cmd, st, st.activeFile);
        if (mod && ImGui::IsKeyPressed(ImGuiKey_Tab, false) && st.files.size() > 1) { // next / previous file
            const int n = static_cast<int>(st.files.size());
            cmd.switchFile = st.files[static_cast<std::size_t>((st.activeFile + (io.KeyShift ? n - 1 : 1)) % n)].id;
        }
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
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {8, (h - ImGui::GetFrameHeight()) * 0.5f});
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, theme::vec(theme::kPanel));
    ImGui::Begin("##commandbar", nullptr, flags);
    {
        const ImVec2 wp = ImGui::GetWindowPos();
        ImGui::GetWindowDrawList()->AddLine(wp, {wp.x + ImGui::GetWindowWidth(), wp.y}, theme::kBorder);
    }

    // History toggle.
    if (ImGui::ArrowButton("##hist", m_showHistory ? ImGuiDir_Down : ImGuiDir_Up)) m_showHistory = !m_showHistory;
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Command history");
    ImGui::SameLine();
    ImGui::AlignTextToFramePadding();
    ImGui::PushFont(theme::font(theme::Font::Strong));
    ImGui::TextColored(theme::vec(theme::kAccentText), "%s", st.commandPrompt.c_str());
    ImGui::PopFont();
    ImGui::SameLine(0, 10);

    ImGui::SetNextItemWidth(-1);
    if (m_focusCommand) {
        ImGui::SetKeyboardFocusHere();
        m_focusCommand = false;
    }
    ImGui::PushStyleColor(ImGuiCol_FrameBg, theme::vec(theme::kField));
    ImGui::PushFont(theme::font(theme::Font::Mono));
    // Mono text, but the frame keeps the height of the other controls.
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding,
                        {10, (ImGui::GetFrameHeight() - ImGui::GetFontSize()) * 0.5f + 0.0f});
    const bool submitted = ImGui::InputTextWithHint(
        "##command",
        st.mode == ViewMode::Simulation
            ? "Type a command (SIM PLAY, MOTION ..., HELP; click here to type; Space plays / pauses the simulation)"
            : "Type a command (HELP for the list, Tab completes, Space to focus) or x,y coordinates",
        m_commandBuf, sizeof(m_commandBuf),
        ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_CallbackCompletion |
            ImGuiInputTextFlags_CallbackHistory,
        commandInputCallback, this);
    ImGui::PopStyleVar();
    ImGui::PopFont();
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
    ImGui::PopStyleVar(3);

    ImGui::PushFont(theme::font(theme::Font::Mono));
    const float lineH = ImGui::GetTextLineHeightWithSpacing();
    auto colorOf = [](UiState::LogLine::Kind k) {
        switch (k) {
        case UiState::LogLine::Input: return theme::vec(theme::kTextDim);
        case UiState::LogLine::Error: return theme::vec(theme::kError);
        default: return theme::vec(theme::kText);
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
        const float ph = lineH * static_cast<float>(lines) + 16.0f;
        // Over the canvas: clear of the tool rail and of the ViewCube in the right corner.
        const float left = mv->WorkPos.x + m_leftDockWidth;
        const float right = std::max(left + 320.0f, m_canvasMax.x - (m_showHistory ? 0.0f : 190.0f));
        ImGui::SetNextWindowPos({left, y - ph});
        ImGui::SetNextWindowSize({right - left, ph});
        ImGui::SetNextWindowBgAlpha(m_showHistory ? 0.97f : 0.82f);
        ImGuiWindowFlags hf = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoSavedSettings |
                              ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav;
        if (!m_showHistory) hf |= ImGuiWindowFlags_NoInputs;
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {12, 8});
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        ImGui::PushStyleColor(ImGuiCol_WindowBg, theme::vec(theme::kChrome));
        ImGui::Begin("##history", nullptr, hf);
        const std::size_t first =
            m_showHistory ? 0 : (st.commandLog.size() > 3 ? st.commandLog.size() - 3 : 0);
        for (std::size_t i = first; i < st.commandLog.size(); ++i) {
            const auto& l = st.commandLog[i];
            ImGui::TextColored(colorOf(l.kind), "%s%s", l.kind == UiState::LogLine::Input ? "> " : "  ", l.text.c_str());
        }
        if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 1.0f || !m_showHistory) ImGui::SetScrollHereY(1.0f);
        ImGui::End();
        ImGui::PopStyleColor();
        ImGui::PopStyleVar(3);
    }
    ImGui::PopFont();
}

void LandingUI::requestFile(int action, const UiState& st) {
    // New and Open use their own file tab, so nothing is discarded.
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
        case FileSaveAs:
            cmd.commands.push_back("SAVE " + quoted(*path));
            if (m_closeAfterSave >= 0) cmd.closeFile = m_closeAfterSave; // closes only if the save worked
            m_closeAfterSave = -1;
            break;
        case FileImport:
        case FileImportImage: cmd.commands.push_back("IMPORT " + quoted(*path)); break;
        case FileExport: cmd.commands.push_back("EXPORT " + quoted(*path)); break;
        default: break;
        }
        m_fileAction = FileNone;
    }

    // Close Save As'd or cancelled: forget the pending close.
    if (m_closeAfterSave >= 0 && m_fileAction != FileSaveAs) m_closeAfterSave = -1;

    if (m_closeAskOpen) {
        ImGui::OpenPopup("Close File###closefile");
        m_closeAskOpen = false;
    }
    if (ImGui::BeginPopupModal("Close File###closefile", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        std::string name = "This file";
        for (const auto& f : st.files)
            if (f.id == m_closeAsk) name = f.name;
        ImGui::Text("Save changes to %s before closing?", name.c_str());
        ImGui::Spacing();
        if (ImGui::Button("Save", {110, 0})) {
            if (st.projectPath.empty()) {
                requestFile(FileSaveAs, st);
                m_closeAfterSave = m_closeAsk;
            } else {
                cmd.commands.push_back("SAVE");
                cmd.closeFile = m_closeAsk; // closes only if the save worked
            }
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Don't Save", {110, 0})) {
            cmd.closeFile = m_closeAsk;
            cmd.closeDiscard = true;
            ImGui::CloseCurrentPopup();
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
                {"Shift + Tab", "Simulation window"},
                {"Space (Simulation)", "Play / pause"},
                {"Home / Left / Right (Simulation)", "Rewind / step back / step forward"},
                {"Ctrl + Tab / Shift", "Next / previous file tab"},
                {"Ctrl + W", "Close the file tab"},
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


// ---------------------------------------------------------------------------
// Library

namespace {

// Small line-art thumbnail for a library item, centred on c with radius r.
void drawLibraryIcon(ImDrawList* dl, const std::string& key, ImVec2 c, float r, ImU32 col) {
    const float t = 1.4f;
    auto gear = [&](ImVec2 at, float rr, int teeth, float phase = 0.0f) {
        for (int i = 0; i < teeth; ++i) {
            const float a = phase + 2.0f * kPi * static_cast<float>(i) / static_cast<float>(teeth);
            const ImVec2 d{std::cos(a), std::sin(a)};
            dl->AddLine({at.x + d.x * rr * 0.78f, at.y + d.y * rr * 0.78f}, {at.x + d.x * rr, at.y + d.y * rr}, col, rr * 0.28f);
        }
        dl->AddCircle(at, rr * 0.78f, col, 24, t);
        dl->AddCircle(at, rr * 0.25f, col, 12, t);
    };
    auto comb = [&](float x0, float x1, float y, int n) {
        dl->AddRect({x0, y}, {x1, y + r * 0.35f}, col, 1.0f, 0, t);
        for (int i = 0; i < n; ++i) {
            const float x = x0 + (x1 - x0) * (static_cast<float>(i) + 0.5f) / static_cast<float>(n);
            dl->AddLine({x, y}, {x, y - r * 0.3f}, col, r * 0.16f);
        }
    };
    auto screw = [&](float x0, float x1, float y, float h) {
        dl->AddRect({x0, y - h}, {x1, y + h}, col, 2.0f, 0, t);
        for (float x = x0 + h * 0.6f; x < x1 - h * 0.3f; x += h * 0.9f) dl->AddLine({x, y + h}, {x + h * 0.7f, y - h}, col, t);
    };
    auto tyreIcon = [&](ImVec2 at, float rr, bool lugs) {
        dl->AddCircle(at, rr, col, 32, 2.6f);
        dl->AddCircle(at, rr * 0.62f, col, 24, t);
        for (int i = 0; i < (lugs ? 10 : 0); ++i) {
            const float a = 2.0f * kPi * static_cast<float>(i) / 10.0f;
            dl->AddLine({at.x + std::cos(a) * rr, at.y + std::sin(a) * rr}, {at.x + std::cos(a) * rr * 1.15f, at.y + std::sin(a) * rr * 1.15f},
                        col, 2.4f);
        }
    };
    auto loop = [&](ImVec2 a, float ra, ImVec2 b, float rb, bool links) {
        dl->AddLine({a.x, a.y - ra}, {b.x, b.y - rb}, col, links ? 2.2f : 1.8f);
        dl->AddLine({a.x, a.y + ra}, {b.x, b.y + rb}, col, links ? 2.2f : 1.8f);
        dl->PathArcTo(a, ra, kPi * 0.5f, kPi * 1.5f, 12);
        dl->PathStroke(col, 0, links ? 2.2f : 1.8f);
        dl->PathArcTo(b, rb, -kPi * 0.5f, kPi * 0.5f, 12);
        dl->PathStroke(col, 0, links ? 2.2f : 1.8f);
    };
    // Robot arm & hand: links as bars with a joint ring at their base.
    auto link = [&](ImVec2 a, ImVec2 b, float w) {
        dl->AddLine(a, b, col, w);
        dl->AddCircle(a, w * 0.75f, col, 12, t);
    };
    auto digit = [&](ImVec2 base, float angle, float len, float w, int links) {
        const ImVec2 d{std::sin(angle), -std::cos(angle)};
        for (int i = 0; i < links; ++i) {
            const ImVec2 b{base.x + d.x * len, base.y + d.y * len};
            link(base, b, w);
            base = b;
            len *= 0.72f;
        }
    };
    auto hand = [&](ImVec2 at, float s) { // palm up the page, thumb to the left
        dl->AddRect({at.x - s * 0.5f, at.y - s * 0.1f}, {at.x + s * 0.5f, at.y + s * 0.65f}, col, s * 0.12f, 0, t);
        const float len[] = {0.38f, 0.44f, 0.4f, 0.3f};
        for (int i = 0; i < 4; ++i)
            digit({at.x + s * (-0.36f + 0.24f * static_cast<float>(i)), at.y - s * 0.16f}, 0.0f, s * len[i], s * 0.13f, 3);
        digit({at.x - s * 0.56f, at.y + s * 0.4f}, -0.9f, s * 0.34f, s * 0.14f, 2);
    };
    if (key == "robot_hand") return hand({c.x, c.y + r * 0.2f}, r * 1.05f);
    // Humanoid body: rounded shells with a joint ring at each pivot.
    auto rrect = [&](float x0, float y0, float x1, float y1, float round) {
        dl->AddRect({c.x + r * x0, c.y + r * y0}, {c.x + r * x1, c.y + r * y1}, col, r * round, 0, t);
    };
    auto joint = [&](float x, float y, float rad) { dl->AddCircle({c.x + r * x, c.y + r * y}, r * rad, col, 16, t); };
    auto bar = [&](float x0, float y0, float x1, float y1, float w) {
        dl->AddLine({c.x + r * x0, c.y + r * y0}, {c.x + r * x1, c.y + r * y1}, col, r * w);
    };
    auto head = [&](float x, float y, float s) { // rounded skull with a visor line
        rrect(x - 0.42f * s, y - 0.5f * s, x + 0.42f * s, y + 0.5f * s, 0.36f * s);
        bar(x - 0.26f * s, y - 0.04f * s, x + 0.26f * s, y - 0.04f * s, 0.12f * s);
    };
    auto bellows = [&](float x0, float x1, float y0, float y1, int folds) {
        for (int i = 0; i <= folds; ++i) {
            const float y = y0 + (y1 - y0) * static_cast<float>(i) / static_cast<float>(folds);
            const float in = i % 2 ? 0.0f : 0.06f;
            bar(x0 + in, y, x1 - in, y, 0.05f);
        }
    };
    auto heart = [&](float x, float y, float rad) {
        joint(x, y, rad);
        for (int i = 0; i < 6; ++i) {
            const float a = kPi / 3.0f * static_cast<float>(i);
            bar(x, y, x + std::cos(a) * rad * 0.8f, y + std::sin(a) * rad * 0.8f, 0.04f);
        }
    };
    auto leg = [&](float x, float y, float h) { // hip at (x, y), foot on y + h
        joint(x, y, 0.1f * h);
        bar(x, y + 0.08f * h, x, y + 0.48f * h, 0.16f * h);
        joint(x, y + 0.5f * h, 0.08f * h);
        bar(x, y + 0.56f * h, x, y + 0.92f * h, 0.13f * h);
        bar(x - 0.06f * h, y + 0.98f * h, x + 0.16f * h, y + 0.98f * h, 0.07f * h);
    };
    if (key == "humanoid") { // broad, muscular V-taper
        head(0, -0.82f, 0.3f);
        bar(0, -0.66f, 0, -0.56f, 0.09f);
        rrect(-0.38f, -0.58f, 0.38f, -0.32f, 0.12f);
        rrect(-0.26f, -0.34f, 0.26f, -0.08f, 0.08f);
        heart(0, -0.44f, 0.08f);
        rrect(-0.24f, -0.02f, 0.24f, 0.14f, 0.06f);
        for (float x : {-0.5f, 0.5f}) bar(x, -0.5f, x * 1.06f, 0.2f, 0.14f);
        for (float x : {-0.14f, 0.14f}) leg(x, 0.16f, 0.8f);
        return;
    }
    if (key == "skeleton_robot") { // skull, spine, ribs, pelvis and limb bones
        head(0, -0.82f, 0.26f);
        bar(0, -0.68f, 0, 0.06f, 0.035f);                                             // spine
        for (int i = 0; i < 5; ++i) {                                                 // ribs
            const float y = -0.54f + 0.09f * static_cast<float>(i), w = 0.2f - 0.012f * static_cast<float>(i * i);
            bar(-w, y + 0.02f, w, y + 0.02f, 0.025f);
        }
        bar(-0.26f, -0.6f, 0.26f, -0.6f, 0.03f);                                      // collarbones
        rrect(-0.2f, 0.04f, 0.2f, 0.14f, 0.05f);                                      // pelvis
        for (float x : {-0.3f, 0.3f}) bar(x, -0.58f, x * 1.1f, 0.2f, 0.035f);         // arms
        for (float x : {-0.11f, 0.11f}) {
            bar(x, 0.14f, x, 0.56f, 0.04f);
            bar(x, 0.58f, x, 0.92f, 0.035f);
            bar(x - 0.04f, 0.97f, x + 0.12f, 0.97f, 0.035f);
        }
        return;
    }
    if (key == "humanoid_female") { // slim: narrow shoulders and waist, curved hips
        head(0, -0.8f, 0.27f);
        bar(0, -0.66f, 0, -0.56f, 0.05f);
        rrect(-0.22f, -0.58f, 0.22f, -0.3f, 0.1f);
        rrect(-0.14f, -0.32f, 0.14f, -0.1f, 0.06f);
        heart(0, -0.45f, 0.06f);
        rrect(-0.22f, -0.1f, 0.22f, 0.14f, 0.1f);
        for (float x : {-0.3f, 0.3f}) bar(x, -0.52f, x * 1.08f, 0.16f, 0.07f);
        for (float x : {-0.1f, 0.1f}) leg(x, 0.16f, 0.8f);
        return;
    }
    if (key == "robot_head_neck") {
        head(0, -0.38f, 0.8f);
        bar(0, 0.04f, 0, 0.42f, 0.18f);
        joint(0, 0.42f, 0.1f);
        bar(-0.55f, 0.85f, 0.55f, 0.85f, 0.08f);
        rrect(-0.14f, 0.5f, 0.14f, 0.82f, 0.02f);
        return;
    }
    if (key == "robot_torso") {
        rrect(-0.6f, -0.9f, 0.6f, -0.12f, 0.2f);
        heart(0, -0.5f, 0.14f);
        bellows(-0.42f, 0.42f, -0.08f, 0.12f, 3);
        rrect(-0.45f, 0.16f, 0.45f, 0.46f, 0.1f);
        rrect(-0.5f, 0.52f, 0.5f, 0.86f, 0.18f);
        return;
    }
    if (key == "robot_leg") {
        leg(0, -0.8f, 1.7f);
        return;
    }
    if (key == "robot_head") return head(0, 0, 1.6f);
    if (key == "robot_neck") {
        rrect(-0.24f, -0.8f, 0.24f, 0.8f, 0.05f);
        rrect(-0.38f, 0.55f, 0.38f, 0.8f, 0.04f);
        rrect(-0.32f, -0.8f, 0.32f, -0.6f, 0.04f);
        return;
    }
    if (key == "robot_chest") {
        rrect(-0.6f, -0.7f, 0.6f, 0.8f, 0.3f);
        joint(-0.68f, -0.5f, 0.2f);
        joint(0.68f, -0.5f, 0.2f);
        return;
    }
    if (key == "robot_heart") {
        heart(0, 0, 0.75f);
        joint(0, 0, 0.9f);
        return;
    }
    if (key == "robot_gut") {
        for (int i = 0; i < 3; ++i) rrect(-0.75f, -0.75f + 0.52f * static_cast<float>(i), 0.75f, -0.33f + 0.52f * static_cast<float>(i), 0.12f);
        return;
    }
    if (key == "robot_midriff") return bellows(-0.75f, 0.75f, -0.6f, 0.6f, 6);
    if (key == "robot_pelvis") {
        rrect(-0.75f, -0.6f, 0.75f, 0.4f, 0.3f);
        joint(-0.45f, 0.4f, 0.22f);
        joint(0.45f, 0.4f, 0.22f);
        return;
    }
    if (key == "robot_thigh" || key == "robot_shin") {
        const bool thigh = key == "robot_thigh";
        joint(0, -0.75f, thigh ? 0.24f : 0.18f);
        bar(0, -0.55f, 0, 0.7f, thigh ? 0.4f : 0.32f);
        if (!thigh) joint(0, 0.85f, 0.12f);
        return;
    }
    if (key == "robot_foot") {
        const ImVec2 pts[] = {{c.x - r * 0.8f, c.y + r * 0.45f}, {c.x - r * 0.8f, c.y - r * 0.1f}, {c.x - r * 0.45f, c.y - r * 0.45f},
                              {c.x - r * 0.1f, c.y - r * 0.4f}, {c.x + r * 0.45f, c.y + r * 0.05f}, {c.x + r * 0.45f, c.y + r * 0.45f}};
        dl->AddPolyline(pts, 6, col, ImDrawFlags_Closed, t);
        rrect(0.5f, 0.1f, 0.9f, 0.45f, 0.15f); // toe
        joint(-0.45f, -0.15f, 0.14f);         // ankle
        return;
    }
    if (key == "robot_finger" || key == "robot_thumb") {
        const bool thumb = key == "robot_thumb";
        dl->AddLine({c.x - r * 0.7f, c.y + r * 0.85f}, {c.x + r * 0.7f, c.y + r * 0.85f}, col, 2.4f); // motor base
        digit({c.x - (thumb ? r * 0.3f : 0.0f), c.y + r * 0.7f}, thumb ? 0.5f : 0.0f, r * (thumb ? 0.75f : 0.72f), r * 0.24f, 3);
        return;
    }
    if (key == "finger_link") {
        link({c.x - r * 0.6f, c.y + r * 0.45f}, {c.x + r * 0.55f, c.y - r * 0.4f}, r * 0.38f);
        return;
    }
    if (key == "robot_palm") {
        const ImVec2 pts[] = {{c.x - r * 0.45f, c.y + r * 0.85f}, {c.x + r * 0.45f, c.y + r * 0.85f}, {c.x + r * 0.8f, c.y + r * 0.2f},
                              {c.x + r * 0.6f, c.y - r * 0.6f}, {c.x - r * 0.1f, c.y - r * 0.8f}, {c.x - r * 0.7f, c.y - r * 0.6f},
                              {c.x - r * 0.8f, c.y + r * 0.2f}};
        dl->AddPolyline(pts, 7, col, ImDrawFlags_Closed, t);
        for (int i = 0; i < 4; ++i) dl->AddCircle({c.x + r * (-0.52f + 0.36f * static_cast<float>(i)), c.y - r * 0.55f}, r * 0.12f, col, 10, t);
        return;
    }
    if (key == "robot_forearm") {
        dl->AddRect({c.x - r * 0.95f, c.y - r * 0.3f}, {c.x + r * 0.55f, c.y + r * 0.3f}, col, 2.0f, 0, t);
        for (int i = 0; i < 3; ++i) dl->AddCircle({c.x + r * (-0.65f + 0.4f * static_cast<float>(i)), c.y}, r * 0.14f, col, 12, t);
        dl->AddLine({c.x + r * 0.55f, c.y - r * 0.22f}, {c.x + r * 0.9f, c.y - r * 0.22f}, col, 2.4f); // clevis
        dl->AddLine({c.x + r * 0.55f, c.y + r * 0.22f}, {c.x + r * 0.9f, c.y + r * 0.22f}, col, 2.4f);
        return;
    }
    if (key == "upper_arm") {
        dl->AddCircle({c.x - r * 0.65f, c.y + r * 0.45f}, r * 0.3f, col, 20, t);
        dl->AddLine({c.x - r * 0.42f, c.y + r * 0.25f}, {c.x + r * 0.5f, c.y - r * 0.45f}, col, r * 0.3f);
        dl->AddCircle({c.x + r * 0.62f, c.y - r * 0.55f}, r * 0.24f, col, 16, t);
        return;
    }
    if (key == "robot_shoulder" || key == "robot_arm") {
        const ImVec2 ball{c.x - r * 0.55f, c.y - r * 0.05f};
        dl->AddLine({c.x - r * 0.95f, c.y + r * 0.9f}, {c.x - r * 0.15f, c.y + r * 0.9f}, col, 2.6f); // base plate
        dl->AddRect({ball.x - r * 0.14f, ball.y + r * 0.25f}, {ball.x + r * 0.14f, c.y + r * 0.88f}, col, 1.0f, 0, t);
        dl->AddCircle(ball, r * 0.24f, col, 20, t);
        if (key == "robot_shoulder") return;
        const ImVec2 elbow{c.x + r * 0.05f, c.y - r * 0.7f}, wrist{c.x + r * 0.55f, c.y - r * 0.05f};
        dl->AddLine({ball.x + r * 0.16f, ball.y - r * 0.18f}, elbow, col, r * 0.2f);
        link(elbow, wrist, r * 0.16f);
        hand({wrist.x + r * 0.2f, wrist.y + r * 0.08f}, r * 0.42f);
        return;
    }
    if (key == "car_wheel" || key == "robot_wheel" || key == "tyre") {
        tyreIcon(c, r * 0.85f, true);
        if (key == "car_wheel")
            for (int i = 0; i < 5; ++i) {
                const float a = -kPi * 0.5f + 2.0f * kPi * static_cast<float>(i) / 5.0f;
                dl->AddLine(c, {c.x + std::cos(a) * r * 0.5f, c.y + std::sin(a) * r * 0.5f}, col, t);
            }
        else if (key == "robot_wheel") dl->AddCircleFilled(c, r * 0.15f, col);
        return;
    }
    if (key == "caster_wheel") {
        dl->AddLine({c.x - r * 0.8f, c.y - r * 0.85f}, {c.x + r * 0.8f, c.y - r * 0.85f}, col, 2.4f);
        dl->AddLine({c.x - r * 0.45f, c.y - r * 0.8f}, {c.x - r * 0.45f, c.y + r * 0.2f}, col, t);
        dl->AddLine({c.x + r * 0.45f, c.y - r * 0.8f}, {c.x + r * 0.45f, c.y + r * 0.2f}, col, t);
        tyreIcon({c.x, c.y + r * 0.3f}, r * 0.55f, false);
        return;
    }
    if (key == "omni_wheel" || key == "mecanum_wheel") {
        dl->AddCircle(c, r * 0.55f, col, 24, t);
        for (int i = 0; i < 8; ++i) {
            const float a = 2.0f * kPi * static_cast<float>(i) / 8.0f;
            const ImVec2 p{c.x + std::cos(a) * r * 0.8f, c.y + std::sin(a) * r * 0.8f};
            const float tilt = key == "mecanum_wheel" ? 0.6f : 0.0f;
            const ImVec2 d{-std::sin(a + tilt) * r * 0.2f, std::cos(a + tilt) * r * 0.2f};
            dl->AddLine({p.x - d.x, p.y - d.y}, {p.x + d.x, p.y + d.y}, col, 3.2f);
        }
        return;
    }
    if (key == "sprocket") {
        for (int i = 0; i < 14; ++i) {
            const float a = 2.0f * kPi * static_cast<float>(i) / 14.0f;
            dl->AddCircleFilled({c.x + std::cos(a) * r * 0.85f, c.y + std::sin(a) * r * 0.85f}, r * 0.11f, col);
        }
        dl->AddCircle(c, r * 0.72f, col, 28, t);
        dl->AddCircle(c, r * 0.22f, col, 12, t);
        return;
    }
    if (key == "roller_chain") {
        for (int i = 0; i < 4; ++i) {
            const float x0 = c.x - r + static_cast<float>(i) * r * 0.5f;
            dl->AddRect({x0, c.y - r * 0.22f}, {x0 + r * 0.62f, c.y + r * 0.22f}, col, r * 0.22f, 0, t);
            dl->AddCircleFilled({x0 + r * 0.18f, c.y}, r * 0.08f, col);
        }
        return;
    }
    if (key == "chain_drive" || key == "belt_drive" || key == "rubber_belt") {
        const ImVec2 a{c.x - r * 0.5f, c.y}, b{c.x + r * 0.55f, c.y};
        const float ra = key == "rubber_belt" ? r * 0.38f : r * 0.3f, rb = key == "rubber_belt" ? r * 0.38f : r * 0.42f;
        loop(a, ra, b, rb, key == "chain_drive");
        if (key != "rubber_belt") {
            dl->AddCircle(a, ra * 0.7f, col, 16, t);
            dl->AddCircle(b, rb * 0.75f, col, 16, t);
        }
        return;
    }
    if (key == "pulley") {
        dl->AddCircle(c, r * 0.9f, col, 32, t);
        dl->AddCircle(c, r * 0.7f, col, 28, t);
        dl->AddCircle(c, r * 0.2f, col, 12, t);
        return;
    }
    if (key == "o_ring") {
        dl->AddCircle(c, r * 0.75f, col, 32, 3.0f);
        return;
    }
    if (key == "ev_flange_motor") { // ribbed drum with a flange face and splined shaft, side view
        dl->AddRect({c.x - r * 0.95f, c.y - r * 0.55f}, {c.x + r * 0.35f, c.y + r * 0.6f}, col, 3.0f, 0, t);
        for (float k : {-0.2f, 0.15f}) dl->AddLine({c.x - r * 0.85f, c.y + r * k}, {c.x + r * 0.25f, c.y + r * k}, col, 1.0f);
        dl->AddRect({c.x + r * 0.35f, c.y - r * 0.68f}, {c.x + r * 0.48f, c.y + r * 0.72f}, col, 1.0f, 0, t);
        dl->AddLine({c.x + r * 0.48f, c.y}, {c.x + r, c.y}, col, 3.4f);
        dl->AddCircle({c.x - r * 0.65f, c.y - r * 0.72f}, r * 0.13f, col, 10, t);
        dl->AddCircle({c.x + r * 0.1f, c.y - r * 0.72f}, r * 0.13f, col, 10, t);
    } else if (key == "ev_motor") { // round frame on feet, terminal box, shaft to the right
        dl->AddCircle({c.x - r * 0.2f, c.y + r * 0.05f}, r * 0.62f, col, 28, t);
        dl->AddCircle({c.x - r * 0.2f, c.y + r * 0.05f}, r * 0.25f, col, 16, t);
        dl->AddRect({c.x - r * 0.55f, c.y - r * 0.95f}, {c.x + r * 0.1f, c.y - r * 0.6f}, col, 1.0f, 0, t);
        dl->AddLine({c.x - r * 0.75f, c.y + r * 0.8f}, {c.x + r * 0.35f, c.y + r * 0.8f}, col, 2.0f);
        dl->AddLine({c.x + r * 0.05f, c.y + r * 0.05f}, {c.x + r, c.y + r * 0.05f}, col, 3.0f);
    } else if (key == "dc_motor") {
        dl->AddRect({c.x - r * 0.75f, c.y - r * 0.55f}, {c.x + r * 0.45f, c.y + r * 0.55f}, col, r * 0.5f, 0, t);
        dl->AddLine({c.x + r * 0.45f, c.y}, {c.x + r, c.y}, col, 2.2f);
        dl->AddLine({c.x - r * 0.75f, c.y - r * 0.2f}, {c.x - r * 0.95f, c.y - r * 0.2f}, col, t);
        dl->AddLine({c.x - r * 0.75f, c.y + r * 0.2f}, {c.x - r * 0.95f, c.y + r * 0.2f}, col, t);
    } else if (key == "stepper_motor") {
        const float h = r * 0.62f, k = r * 0.2f;
        const ImVec2 o{c.x - r * 0.2f, c.y};
        const ImVec2 pts[] = {{o.x - h + k, o.y - h}, {o.x + h - k, o.y - h}, {o.x + h, o.y - h + k}, {o.x + h, o.y + h - k},
                              {o.x + h - k, o.y + h}, {o.x - h + k, o.y + h}, {o.x - h, o.y + h - k}, {o.x - h, o.y - h + k}};
        dl->AddPolyline(pts, 8, col, ImDrawFlags_Closed, t);
        dl->AddCircle(o, h * 0.45f, col, 16, t);
        dl->AddLine({o.x + h, o.y}, {c.x + r, o.y}, col, 2.2f);
    } else if (key == "servo_motor") {
        dl->AddRect({c.x - r * 0.9f, c.y - r * 0.2f}, {c.x + r * 0.5f, c.y + r * 0.7f}, col, 2.0f, 0, t);
        dl->AddLine({c.x - r, c.y + r * 0.05f}, {c.x + r * 0.6f, c.y + r * 0.05f}, col, t);
        dl->AddLine({c.x - r * 0.1f, c.y - r * 0.2f}, {c.x - r * 0.1f, c.y - r * 0.45f}, col, 2.0f);
        dl->AddLine({c.x - r * 0.75f, c.y - r * 0.55f}, {c.x + r * 0.55f, c.y - r * 0.55f}, col, 3.0f);
    } else if (key == "spur_gear" || key == "worm_wheel") {
        gear(c, r * 0.95f, 12);
    } else if (key == "helical_gear" || key == "herringbone_gear") {
        dl->AddRect({c.x - r * 0.9f, c.y - r * 0.6f}, {c.x + r * 0.9f, c.y + r * 0.6f}, col, 2.0f, 0, t);
        for (int i = -3; i <= 3; ++i) {
            const float x = c.x + static_cast<float>(i) * r * 0.26f;
            if (key == "helical_gear") dl->AddLine({x - r * 0.2f, c.y + r * 0.6f}, {x + r * 0.2f, c.y - r * 0.6f}, col, t);
            else {
                dl->AddLine({x - r * 0.15f, c.y - r * 0.6f}, {x + r * 0.15f, c.y}, col, t);
                dl->AddLine({x + r * 0.15f, c.y}, {x - r * 0.15f, c.y + r * 0.6f}, col, t);
            }
        }
    } else if (key == "bevel_gear") {
        const ImVec2 pts[] = {{c.x - r * 0.95f, c.y + r * 0.5f}, {c.x + r * 0.95f, c.y + r * 0.5f},
                              {c.x + r * 0.45f, c.y - r * 0.5f}, {c.x - r * 0.45f, c.y - r * 0.5f}};
        dl->AddPolyline(pts, 4, col, ImDrawFlags_Closed, t);
        for (int i = -2; i <= 2; ++i)
            dl->AddLine({c.x + static_cast<float>(i) * r * 0.36f, c.y + r * 0.5f},
                        {c.x + static_cast<float>(i) * r * 0.18f, c.y - r * 0.5f}, col, t);
    } else if (key == "ring_gear") {
        dl->AddCircle(c, r * 0.95f, col, 32, t);
        for (int i = 0; i < 16; ++i) {
            const float a = 2.0f * kPi * static_cast<float>(i) / 16.0f;
            const ImVec2 d{std::cos(a), std::sin(a)};
            dl->AddLine({c.x + d.x * r * 0.75f, c.y + d.y * r * 0.75f}, {c.x + d.x * r * 0.55f, c.y + d.y * r * 0.55f}, col, 2.2f);
        }
        dl->AddCircle(c, r * 0.75f, col, 32, t);
    } else if (key == "worm") {
        screw(c.x - r, c.x + r, c.y, r * 0.4f);
    } else if (key == "rack") {
        comb(c.x - r, c.x + r, c.y, 6);
    } else if (key == "gear_pair") {
        gear({c.x - r * 0.45f, c.y + r * 0.1f}, r * 0.5f, 8);
        gear({c.x + r * 0.42f, c.y - r * 0.05f}, r * 0.62f, 10, 0.3f);
    } else if (key == "rack_pinion") {
        gear({c.x, c.y - r * 0.3f}, r * 0.55f, 8);
        comb(c.x - r, c.x + r, c.y + r * 0.55f, 6);
    } else if (key == "planetary") {
        dl->AddCircle(c, r * 0.95f, col, 32, t);
        dl->AddCircle(c, r * 0.28f, col, 16, t);
        for (int i = 0; i < 3; ++i) {
            const float a = -kPi * 0.5f + 2.0f * kPi * static_cast<float>(i) / 3.0f;
            dl->AddCircle({c.x + std::cos(a) * r * 0.6f, c.y + std::sin(a) * r * 0.6f}, r * 0.3f, col, 16, t);
        }
    } else if (key == "worm_drive") {
        gear({c.x, c.y - r * 0.25f}, r * 0.62f, 10);
        screw(c.x - r, c.x + r, c.y + r * 0.62f, r * 0.22f);
    }
}

bool containsNoCase(const std::string& hay, const char* needle) {
    if (!needle[0]) return true;
    std::string h = hay, n = needle;
    for (char& ch : h) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    for (char& ch : n) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    return h.find(n) != std::string::npos;
}

} // namespace

void LandingUI::drawLibrary(UiCommands& cmd, const Scene& scene, UiState& st) {
    (void)scene;
    (void)st;
    ImGui::SetNextItemWidth(-1);
    ImGui::InputTextWithHint("##libsearch", "Search motors, gears...", m_libSearch, sizeof(m_libSearch));

    const auto& items = libraryItems();
    const LibraryItem* selected = m_libParamsOpen ? findLibraryItem(m_libSelected) : nullptr;
    // List on top, parameters of the selected part below (closed with x, reopened by picking a part).
    const float paramsH = selected ? 96.0f + static_cast<float>(selected->params.size()) * (ImGui::GetFrameHeightWithSpacing()) : 0.0f;
    ImGui::BeginChild("##liblist", {0, -paramsH});
    constexpr float kRow = 44.0f;
    const char* lastCategory = "";
    for (const LibraryItem& it : items) {
        if (!containsNoCase(std::string(it.name) + " " + it.category + " " + it.summary + " " + it.aliases, m_libSearch))
            continue;
        if (std::string(lastCategory) != it.category) {
            theme::caption(it.category);
            lastCategory = it.category;
        }
        ImGui::PushID(it.key);
        const bool isSel = m_libSelected == it.key;
        if (ImGui::Selectable("##item", isSel, ImGuiSelectableFlags_AllowDoubleClick, {0, kRow})) {
            m_libSelected = it.key;
            m_libParamsOpen = true;
            if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) cmd.commands.push_back(std::string("PART ") + it.key);
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s\n%s\nDouble-click to insert", it.name, it.summary);
        const ImVec2 lo = ImGui::GetItemRectMin(), hi = ImGui::GetItemRectMax();
        ImDrawList* dl = ImGui::GetWindowDrawList();
        // Thumbnail tile.
        const ImVec2 t0{lo.x + 4, lo.y + 4}, t1{lo.x + 4 + kRow - 8, hi.y - 4};
        dl->AddRectFilled(t0, t1, theme::kField, 4.0f);
        drawLibraryIcon(dl, it.key, {(t0.x + t1.x) * 0.5f, (t0.y + t1.y) * 0.5f}, (t1.x - t0.x) * 0.36f,
                        isSel ? theme::kAccentText : theme::kTextDim);
        const float tx = t1.x + 10;
        dl->AddText(theme::font(theme::Font::Strong), ImGui::GetFontSize(), {tx, lo.y + 6}, theme::kText, it.name);
        {
            ImFont* small = theme::font(theme::Font::Small);
            const float fs = small->FontSize * ImGui::GetIO().FontGlobalScale;
            dl->PushClipRect({tx, lo.y}, {hi.x - 4, hi.y}, true);
            dl->AddText(small, fs, {tx, lo.y + 24}, theme::kTextDim, it.summary);
            dl->PopClipRect();
        }
        ImGui::PopID();
    }
    ImGui::EndChild();

    if (!selected) return;
    // Parameters of the selected part.
    auto& values = m_libValues[selected->key];
    const float right = ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x;
    theme::caption(selected->name);
    ImGui::SameLine(right - ImGui::CalcTextSize("x").x - ImGui::GetStyle().FramePadding.x * 2.0f);
    if (ImGui::SmallButton("x##libclose")) m_libParamsOpen = false;
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Hide the parameters");
    if (beginPropertyGrid("##libparams")) {
        for (const LibraryParam& p : selected->params) {
            auto it = values.find(p.key);
            if (it == values.end()) it = values.emplace(p.key, p.value).first;
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::AlignTextToFramePadding();
            ImGui::TextColored(theme::vec(theme::kTextDim), "%s", p.label);
            ImGui::TableNextColumn();
            // Rear shaft only matters for a dual-shaft motor.
            const auto shafts = values.find("shafts");
            const bool off = std::string(p.key) == "rear_length" && shafts != values.end() && shafts->second < 1.5f;
            if (off) ImGui::BeginDisabled();
            paramEditor(p, it->second);
            if (off) ImGui::EndDisabled();
        }
        ImGui::EndTable();
    }
    const float half = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
    if (ImGui::Button("Defaults", {half * 0.7f, 0})) values.clear();
    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button, theme::vec(theme::kAccent));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, theme::vec(theme::kAccentHover));
    if (ImGui::Button("Insert", {-1, 0})) {
        std::string line = std::string("PART ") + selected->key;
        for (const LibraryParam& p : selected->params) {
            const auto it = values.find(p.key);
            if (it == values.end() || it->second == p.value) continue;
            // Lengths carry their unit: the command reads bare numbers in the document's units.
            const bool mm = p.unit && std::string_view(p.unit) == "mm" && !p.integer && !p.choices;
            char num[64];
            std::snprintf(num, sizeof(num), " %s %.9g%s", p.key, static_cast<double>(it->second), mm ? "mm" : "");
            line += num;
        }
        cmd.commands.push_back(line);
    }
    ImGui::PopStyleColor(2);
    {
        theme::ScopedFont small(theme::Font::Small);
        ImGui::PushStyleColor(ImGuiCol_Text, theme::vec(theme::kTextFaint));
        ImGui::TextWrapped("Placed beside the model; double-click a part to insert it with defaults. Motors spin in the "
                           "Simulation window.");
        ImGui::PopStyleColor();
    }
}

// ---------------------------------------------------------------------------
// Simulation window

namespace {

// Rotation of a rigid transform as A (about X), B (about Y), C (about Z) in degrees, ZYX order.
Vec3 eulerABC(const Mat4& t) {
    const float r20 = std::clamp(t.m[0][2], -1.0f, 1.0f);
    return {degrees(std::atan2(t.m[1][2], t.m[2][2])), degrees(-std::asin(r20)), degrees(std::atan2(t.m[0][1], t.m[0][0]))};
}

std::string quotedName(const std::string& s) { return "\"" + s + "\""; }

const ImVec4 kMotionColors[] = {{1.00f, 0.66f, 0.28f, 1}, {0.45f, 0.80f, 1.00f, 1}, {0.60f, 0.90f, 0.45f, 1},
                                {0.95f, 0.50f, 0.75f, 1}, {0.85f, 0.80f, 0.40f, 1}, {0.70f, 0.60f, 1.00f, 1}};

} // namespace

ImVec4 motionColor(std::size_t i) { return kMotionColors[i % (sizeof(kMotionColors) / sizeof(kMotionColors[0]))]; }

void LandingUI::drawSimTransport(UiState& st) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const float fh = ImGui::GetFrameHeight();
    auto iconButton = [&](const char* id, const char* tip, auto&& icon, bool on = false) {
        ImGui::InvisibleButton(id, {fh + 6, fh});
        const ImVec2 lo = ImGui::GetItemRectMin(), hi = ImGui::GetItemRectMax();
        const bool hov = ImGui::IsItemHovered();
        dl->AddRectFilled(lo, hi, on ? kAccent : hov ? IM_COL32(58, 64, 76, 255) : IM_COL32(40, 44, 52, 255), 4.0f);
        icon(ImVec2{(lo.x + hi.x) * 0.5f, (lo.y + hi.y) * 0.5f}, IM_COL32(230, 232, 238, 255));
        if (hov) ImGui::SetTooltip("%s", tip);
        ImGui::SameLine(0, 4);
        return ImGui::IsItemClicked();
    };
    if (iconButton("##rewind", "Rewind to 0 s  [Home]", [&](ImVec2 c, ImU32 col) {
            dl->AddRectFilled({c.x - 6, c.y - 5}, {c.x - 4, c.y + 5}, col);
            dl->AddTriangleFilled({c.x + 5, c.y - 5}, {c.x + 5, c.y + 5}, {c.x - 3, c.y}, col);
        }))
        st.simTime = 0.0f;
    if (iconButton("##play", st.simPlaying ? "Pause  [Space]" : "Play  [Space]",
                   [&](ImVec2 c, ImU32 col) {
                       if (st.simPlaying) {
                           dl->AddRectFilled({c.x - 5, c.y - 5}, {c.x - 1.5f, c.y + 5}, col);
                           dl->AddRectFilled({c.x + 1.5f, c.y - 5}, {c.x + 5, c.y + 5}, col);
                       } else {
                           dl->AddTriangleFilled({c.x - 4, c.y - 6}, {c.x - 4, c.y + 6}, {c.x + 6, c.y}, col);
                       }
                   },
                   st.simPlaying)) {
        if (!st.simPlaying && st.simTime >= st.simDuration - 1e-4f) st.simTime = 0.0f; // replay from the start
        st.simPlaying = !st.simPlaying;
    }
    if (iconButton("##stop", "Stop: back to the modelled positions", [&](ImVec2 c, ImU32 col) {
            dl->AddRectFilled({c.x - 5, c.y - 5}, {c.x + 5, c.y + 5}, col, 1.0f);
        })) {
        st.simPlaying = false;
        st.simTime = 0.0f;
    }
    ImGui::SetNextItemWidth(230);
    char label[48];
    std::snprintf(label, sizeof(label), "t = %.2f / %.1f s", static_cast<double>(st.simTime),
                  static_cast<double>(st.simDuration));
    ImGui::SliderFloat("##simtime", &st.simTime, 0.0f, st.simDuration, label, ImGuiSliderFlags_NoRoundToFormat);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Scrub the timeline  [Left / Right step 1/30 s]");
    ImGui::SameLine();
    static const float kSpeeds[] = {0.1f, 0.25f, 0.5f, 1.0f, 2.0f, 4.0f, 10.0f};
    char speed[16];
    std::snprintf(speed, sizeof(speed), "%gx", static_cast<double>(st.simSpeed));
    ImGui::SetNextItemWidth(70);
    if (ImGui::BeginCombo("##simspeed", speed)) {
        for (float s : kSpeeds) {
            std::snprintf(speed, sizeof(speed), "%gx", static_cast<double>(s));
            if (ImGui::Selectable(speed, s == st.simSpeed)) st.simSpeed = s;
        }
        ImGui::EndCombo();
    }
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Playback speed (1x = real time)");
    ImGui::SameLine();
    ImGui::Checkbox("Loop", &st.simLoop);
}

float LandingUI::drawMotionPanel(Scene& scene, UiCommands& cmd, UiState& st, float top, float bottom) {
    const ImGuiViewport* mv = ImGui::GetMainViewport();
    constexpr float kWidth = 340.0f;
    ImGui::SetNextWindowPos({mv->WorkPos.x + mv->WorkSize.x - kWidth, top});
    ImGui::SetNextWindowSize({kWidth, bottom - top});
    constexpr ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                                       ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoCollapse |
                                       ImGuiWindowFlags_NoBringToFrontOnFocus;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::Begin("##motionstudy", nullptr, flags);
    {
        theme::ScopedFont title(theme::Font::Strong);
        ImGui::TextUnformatted("Motion Study");
    }
    ImGui::SameLine();
    {
        theme::ScopedFont small(theme::Font::Small);
        ImGui::TextColored(theme::vec(theme::kTextDim), "%zu motion%s", scene.motions().size(),
                           scene.motions().size() == 1 ? "" : "s");
    }
    ImGui::Spacing();

    const auto& bodies = scene.bodies();
    const bool hasSel = st.selectedBody >= 0 && st.selectedBody < static_cast<int>(bodies.size());
    const std::string targets = hasSel ? std::string() : " BODIES ALL";
    const float half = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
    if (ImGui::Button("+ Circular", {half, 0})) cmd.commands.push_back("MOTION CIRCULAR" + targets);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Rotate %s about an axis (deg/s)", hasSel ? bodies[static_cast<std::size_t>(st.selectedBody)].name.c_str() : "all bodies");
    ImGui::SameLine();
    if (ImGui::Button("+ Linear", {half, 0})) cmd.commands.push_back("MOTION LINEAR" + targets);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Move %s along a direction (mm/s)", hasSel ? bodies[static_cast<std::size_t>(st.selectedBody)].name.c_str() : "all bodies");
    {
        theme::ScopedFont small(theme::Font::Small);
        ImGui::TextColored(theme::vec(theme::kTextDim), hasSel ? "Adds a motion for the selected body"
                                                              : "Adds a motion for all bodies (select one to target it)");
    }

    theme::caption("Display");
    ImGui::Checkbox("Real-time axes", &st.simAxes);
    ImGui::SameLine(170);
    ImGui::Checkbox("Motion axes", &st.simMotionAxes);
    ImGui::Checkbox("Paths", &st.simPaths);
    ImGui::SameLine(170);
    ImGui::Checkbox("Readout", &st.simReadout);
    {
        ImGui::AlignTextToFramePadding();
        ImGui::TextDisabled("Length");
        ImGui::SameLine();
        bool automatic = st.simDurationSetting <= 0.0f;
        if (ImGui::Checkbox("Auto", &automatic))
            cmd.commands.push_back(automatic ? "SIM DURATION 0" : "SIM DURATION " + std::to_string(st.simDuration));
        ImGui::SameLine();
        float d = st.simDuration;
        ImGui::SetNextItemWidth(110);
        if (automatic) ImGui::BeginDisabled();
        if (ImGui::DragFloat("##simlen", &d, 0.1f, 0.1f, 3600.0f, "%.1f s") && d > 0.05f) {
            char line[48];
            std::snprintf(line, sizeof(line), "SIM DURATION %.3f", static_cast<double>(d));
            cmd.commands.push_back(line);
        }
        if (automatic) ImGui::EndDisabled();
    }

    // Physical environment: gravity, fields, air, heat, relativity; the selected body's
    // mass, velocity, charge, temperature... Edits run PHYSICS commands when finished.
    {
        PhysicsSettings& ps = scene.physics();
        const ImGuiTreeNodeFlags open = ps.enabled ? ImGuiTreeNodeFlags_DefaultOpen : 0;
        if (ImGui::CollapsingHeader("Physics environment", open)) {
            ImGui::PushID("physics");
            auto run = [&](const std::string& line) { cmd.commands.push_back(line); };
            const float labelW = 92.0f;
            const float fieldW = ImGui::GetContentRegionAvail().x - labelW;
            auto label = [&](const char* text) {
                ImGui::AlignTextToFramePadding();
                ImGui::TextDisabled("%s", text);
                ImGui::SameLine(labelW);
                ImGui::SetNextItemWidth(fieldW);
            };
            auto number = [&](const char* text, const char* id, float value, const char* format,
                              const std::function<std::string(float)>& line, const char* tip = nullptr) {
                label(text);
                float v = value;
                ImGui::InputFloat(id, &v, 0, 0, format);
                if (tip && ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tip);
                if (ImGui::IsItemDeactivatedAfterEdit()) run(line(v));
            };
            auto vector = [&](const char* text, const char* id, Vec3 value, const char* format,
                              const std::function<std::string(const Vec3&)>& line, const char* tip = nullptr) {
                label(text);
                float v[3] = {value.x, value.y, value.z};
                ImGui::InputFloat3(id, v, format);
                if (tip && ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tip);
                if (ImGui::IsItemDeactivatedAfterEdit()) run(line({v[0], v[1], v[2]}));
            };
            auto num = [](double v) {
                char b[32];
                std::snprintf(b, sizeof(b), "%.6g", v);
                return std::string(b);
            };
            auto triple = [&](const Vec3& v, const char* unit) {
                return num(v.x) + unit + "," + num(v.y) + unit + "," + num(v.z) + unit;
            };
            bool on = ps.enabled;
            if (ImGui::Checkbox("Physics on", &on)) run(on ? "PHYSICS ON" : "PHYSICS OFF");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Bodies in the physics move by forces: gravity, collisions, charges, fields.\n"
                                  "Turning it on adds every visible body when none was chosen.");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
            if (ImGui::BeginCombo("##demo", "Experiment...")) {
                for (const PhysicsDemo& d : physicsDemos()) {
                    if (ImGui::Selectable(d.title)) {
                        run(std::string("PHYSICS DEMO ") + d.key);
                        run("SIM STOP");
                        run("SIM PLAY");
                    }
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s\n(replaces the scene)", d.summary);
                }
                ImGui::EndCombo();
            }
            // World.
            label("Gravity");
            static const char* kModes[] = {"Off", "Uniform (g)", "Newton (G m1 m2 / r^2)", "Both"};
            int mode = static_cast<int>(ps.gravity);
            if (ImGui::Combo("##gravity", &mode, kModes, 4))
                run(std::string("PHYSICS GRAVITY ") + gravityModeKey(static_cast<GravityMode>(mode)));
            if (ps.gravity == GravityMode::Uniform || ps.gravity == GravityMode::Both)
                vector("g  m/s^2", "##g", ps.g, "%.4g", [&](const Vec3& v) {
                    return std::string("PHYSICS GRAVITY ") + gravityModeKey(ps.gravity) + " " + num(v.x) + "," + num(v.y) + "," + num(v.z);
                }, "Earth 9.80665, Moon 1.62, Mars 3.71 (downwards: 0,0,-g)");
            if (ps.gravity == GravityMode::Newton || ps.gravity == GravityMode::Both)
                number("G", "##bigG", static_cast<float>(ps.bigG), "%.4e", [&](float v) {
                    return std::string("PHYSICS GRAVITY ") + gravityModeKey(ps.gravity) + " G " + num(v);
                }, "Newton's gravitational constant, 6.6743e-11 m^3/(kg s^2)");
            bool floor = ps.floor, air = ps.air, rel = ps.relativity;
            if (ImGui::Checkbox("Floor", &floor)) run(floor ? "PHYSICS FLOOR ON" : "PHYSICS FLOOR OFF");
            ImGui::SameLine(110);
            if (ImGui::Checkbox("Air drag", &air)) run(air ? "PHYSICS AIR ON" : "PHYSICS AIR OFF");
            ImGui::SameLine(220);
            if (ImGui::Checkbox("Relativity", &rel)) run(rel ? "PHYSICS RELATIVITY ON" : "PHYSICS RELATIVITY OFF");
            if (ps.floor)
                number("Friction", "##mu", ps.friction, "%.3g", [&](float v) { return "PHYSICS FLOOR ON FRICTION " + num(v); });
            if (ps.relativity)
                number("c  m/s", "##c", static_cast<float>(ps.lightSpeed), "%.6g",
                       [&](float v) { return "PHYSICS RELATIVITY ON C " + num(v); }, "Lower it (e.g. 2 m/s) to see relativity");
            vector("E  V/m", "##E", ps.electricField, "%.4g", [&](const Vec3& v) { return "PHYSICS FIELD E " + triple(v, ""); });
            vector("B  T", "##B", ps.magneticField, "%.4g", [&](const Vec3& v) { return "PHYSICS FIELD B " + triple(v, ""); });
            number("Ambient C", "##amb", ps.ambient - 273.15f, "%.1f", [&](float v) { return "PHYSICS AMBIENT " + num(v); });
            number("Length s", "##dur", ps.duration, "%.1f", [&](float v) { return "PHYSICS DURATION " + num(v); },
                   "Seconds simulated (the run is computed once, then plays and scrubs)");

            // The selected body.
            if (hasSel) {
                const Body& b = bodies[static_cast<std::size_t>(st.selectedBody)];
                const PhysicsProps& p = b.physics;
                const std::string q = "PHYSICS BODY \"" + b.name + "\"";
                theme::caption(b.name.c_str());
                bool in = p.enabled, fixed = p.fixed;
                if (ImGui::Checkbox("In the physics", &in)) run(q + (in ? (p.fixed ? " FIXED" : " DYNAMIC") : " OFF"));
                if (p.enabled) {
                    ImGui::SameLine(160);
                    if (ImGui::Checkbox("Fixed", &fixed)) run(q + (fixed ? " FIXED" : " DYNAMIC"));
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Does not move (an anchor, a planet, a charged plate): still attracts and collides");
                    const MassProperties mp = massProperties(b);
                    char massFmt[48];
                    std::snprintf(massFmt, sizeof(massFmt), p.mass > 0 ? "%%.4g kg" : "auto %.4g kg", mp.mass);
                    number("Mass", "##mass", p.mass, massFmt, [&](float v) { return q + " MASS " + num(v); },
                           "0 = volume x density");
                    number("Density", "##rho", p.density, "%.4g kg/m^3", [&](float v) { return q + " DENSITY " + num(v); },
                           "Steel 7850, aluminium 2700, water 1000, wood 600");
                    vector("Velocity m/s", "##vel", p.velocity * 0.001f, "%.4g",
                           [&](const Vec3& v) { return q + " VELOCITY " + triple(v, "m"); }, "At t = 0");
                    vector("Spin deg/s", "##spin", p.spin, "%.4g", [&](const Vec3& v) { return q + " SPIN " + triple(v, ""); });
                    vector("Force N", "##force", p.force, "%.4g", [&](const Vec3& v) { return q + " FORCE " + triple(v, ""); },
                           "Applied all the time (a thruster, a push)");
                    number("Charge uC", "##q", static_cast<float>(p.charge * 1e6), "%.4g",
                           [&](float v) { return q + " CHARGE " + num(v * 1e-6); }, "Coulomb's law between charges, q(E + v x B) in fields");
                    number("Temp C", "##temp", p.temperature - 273.15f, "%.1f", [&](float v) { return q + " TEMP " + num(v); });
                    number("Specific heat", "##cp", p.specificHeat, "%.4g J/kgK", [&](float v) { return q + " HEAT " + num(v); });
                    number("Bounce", "##e", p.restitution, "%.2f", [&](float v) { return q + " BOUNCE " + num(v); },
                           "Coefficient of restitution: 1 elastic, 0 sticks");
                    label("Waves");
                    static const char* kWaves[] = {"Off", "Sound (343 m/s)", "Light (c)", "Custom speed"};
                    int wave = static_cast<int>(p.wave);
                    if (ImGui::Combo("##wave", &wave, kWaves, 4))
                        run(q + " WAVE " + waveKindKey(static_cast<WaveKind>(wave)));
                    if (p.wave != WaveKind::Off) {
                        number("Frequency", "##f", p.waveFrequency, "%.4g Hz", [&](float v) { return q + " FREQ " + num(v); });
                        if (p.wave == WaveKind::Custom)
                            number("Wave m/s", "##ws", p.waveSpeed * 0.001f, "%.4g",
                                   [&](float v) { return q + " WAVESPEED " + num(v) + "m"; }, "Ripples, slow sound...");
                    }
                }
            } else {
                theme::caption("Body");
                ImGui::TextDisabled("Select a body to give it mass, velocity, charge...");
            }
            // Live readout at the playhead.
            if (ps.enabled && physicsRun(scene)) {
                theme::caption("Readout");
                theme::ScopedFont small(theme::Font::Small);
                const auto lines = physicsReport(scene, st.simTime,
                                                 hasSel ? std::optional<std::size_t>(static_cast<std::size_t>(st.selectedBody)) : std::nullopt);
                for (std::size_t i = 2; i < lines.size() && i < (hasSel ? lines.size() : 5); ++i)
                    ImGui::TextWrapped("%s", lines[i].c_str() + std::min<std::size_t>(lines[i].find_first_not_of(' '), lines[i].size()));
            }
            ImGui::PopID();
        }
    }

    // Postures and gestures of the library robots (humanoid, arm, hand, leg, neck...).
    if (const std::vector<std::string> robots = robotNames(scene); !robots.empty()) {
        theme::caption("Robot pose");
        static std::string robot;
        static int posture = 0, gesture = 0;
        static bool loopGesture = false, mirrorGesture = false;
        if (std::find(robots.begin(), robots.end(), robot) == robots.end()) robot = robots.front();
        if (hasSel && !bodies[static_cast<std::size_t>(st.selectedBody)].component.empty()) {
            const std::string& c = bodies[static_cast<std::size_t>(st.selectedBody)].component;
            if (std::find(robots.begin(), robots.end(), c) != robots.end()) robot = c;
        }
        const auto& postures = namedPostures();
        const auto& gestures = namedGestures();
        const float labelW = 64.0f, buttonW = 64.0f;
        const float fieldW = ImGui::GetContentRegionAvail().x - labelW - buttonW - 2 * ImGui::GetStyle().ItemSpacing.x;
        auto label = [&](const char* text) {
            ImGui::AlignTextToFramePadding();
            ImGui::TextDisabled("%s", text);
            ImGui::SameLine(labelW);
        };
        const std::string quoted = "\"" + robot + "\"";
        label("Robot");
        ImGui::SetNextItemWidth(fieldW + buttonW + ImGui::GetStyle().ItemSpacing.x);
        if (ImGui::BeginCombo("##robot", robot.c_str())) {
            for (const std::string& r : robots)
                if (ImGui::Selectable(r.c_str(), r == robot)) robot = r;
            ImGui::EndCombo();
        }
        label("Posture");
        ImGui::SetNextItemWidth(fieldW);
        posture = std::clamp(posture, 0, static_cast<int>(postures.size()) - 1);
        if (ImGui::BeginCombo("##posture", postures[static_cast<std::size_t>(posture)].name)) {
            for (int i = 0; i < static_cast<int>(postures.size()); ++i) {
                if (ImGui::Selectable(postures[static_cast<std::size_t>(i)].name, i == posture)) posture = i;
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", postures[static_cast<std::size_t>(i)].summary);
            }
            ImGui::EndCombo();
        }
        ImGui::SameLine();
        if (ImGui::Button("Pose", {buttonW, 0})) {
            cmd.commands.push_back("POSE " + quoted + " " + postures[static_cast<std::size_t>(posture)].name);
            cmd.commands.push_back("SIM STOP");
            cmd.commands.push_back("SIM PLAY");
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Move into the posture from the one it holds: joints clamped to their hard stops,\n"
                              "minimum-jerk paths within the motors' rated speed, feet kept on the floor");
        label("Gesture");
        ImGui::SetNextItemWidth(fieldW);
        gesture = std::clamp(gesture, 0, static_cast<int>(gestures.size()) - 1);
        if (ImGui::BeginCombo("##gesture", gestures[static_cast<std::size_t>(gesture)].name)) {
            for (int i = 0; i < static_cast<int>(gestures.size()); ++i) {
                if (ImGui::Selectable(gestures[static_cast<std::size_t>(i)].name, i == gesture)) {
                    gesture = i;
                    loopGesture = gestures[static_cast<std::size_t>(i)].loop;
                }
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", gestures[static_cast<std::size_t>(i)].summary);
            }
            ImGui::EndCombo();
        }
        ImGui::SameLine();
        if (ImGui::Button("Play", {buttonW, 0})) {
            cmd.commands.push_back("GESTURE " + quoted + " " + gestures[static_cast<std::size_t>(gesture)].name +
                                   (loopGesture ? " LOOP" : " ONCE") + (mirrorGesture ? " MIRROR" : ""));
            cmd.commands.push_back("SIM STOP");
            cmd.commands.push_back("SIM PLAY");
        }
        ImGui::Dummy({labelW - ImGui::GetStyle().ItemSpacing.x, 0});
        ImGui::SameLine(labelW);
        ImGui::Checkbox("Loop##gesture", &loopGesture);
        ImGui::SameLine();
        ImGui::Checkbox("Other side", &mirrorGesture);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Play it with the left arm / hand");
    }

    theme::caption("Motions");
    auto& motions = scene.motions();
    const float readoutH = st.simReadout ? 190.0f : 0.0f;
    ImGui::BeginChild("##motions", {0, -readoutH}, ImGuiChildFlags_None);
    if (motions.empty()) {
        ImGui::TextWrapped("No motions yet. Select a body, add a Circular (rotation) or Linear (travel) motion, "
                           "then press Play.");
        ImGui::Spacing();
        ImGui::TextDisabled("Command line:");
        ImGui::TextWrapped("MOTION LINEAR Drive DIR X SPEED 20 BODIES ALL");
        ImGui::TextWrapped("MOTION CIRCLE Spin AXIS Y RPM 30 BODIES \"Wheel 1\"");
    }
    int remove = -1;
    for (std::size_t i = 0; i < motions.size(); ++i) {
        Motion& m = motions[i];
        ImGui::PushID(static_cast<int>(i));
        const ImVec4 col = motionColor(i);
        // Neutral section header with the motion's colour as a swatch.
        ImGui::PushStyleColor(ImGuiCol_Header, theme::vec(theme::kPanelRaised));
        ImGui::PushStyleColor(ImGuiCol_HeaderHovered, theme::vec(theme::kHover));
        ImGui::PushStyleColor(ImGuiCol_HeaderActive, theme::vec(theme::kHover));
        ImGui::PushStyleColor(ImGuiCol_Text, m.enabled ? theme::vec(theme::kText) : theme::vec(theme::kTextFaint));
        const std::string header = std::string("      ") + m.name + "###hdr";
        const bool open = ImGui::CollapsingHeader(header.c_str(), ImGuiTreeNodeFlags_DefaultOpen);
        ImGui::PopStyleColor(4);
        {
            const ImVec2 lo = ImGui::GetItemRectMin(), hi = ImGui::GetItemRectMax();
            const float cy = (lo.y + hi.y) * 0.5f, x = lo.x + 30.0f;
            ImDrawList* hdl = ImGui::GetWindowDrawList();
            hdl->AddCircleFilled({x, cy}, 4.0f, ImGui::ColorConvertFloat4ToU32(m.enabled ? col : theme::vec(theme::kTextFaint)));
            theme::ScopedFont small(theme::Font::Small);
            const char* kind = m.kind == MotionKind::Circular ? "Circular" : "Linear";
            const float kw = ImGui::CalcTextSize(kind).x;
            hdl->AddText({hi.x - kw - 10.0f, cy - ImGui::GetFontSize() * 0.5f}, theme::kTextDim, kind);
        }
        if (open) {
            ImGui::Checkbox("##on", &m.enabled);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Enabled");
            ImGui::SameLine();
            char name[96];
            std::snprintf(name, sizeof(name), "%s", m.name.c_str());
            ImGui::SetNextItemWidth(180);
            if (ImGui::InputText("##name", name, sizeof(name)) && name[0]) m.name = name;
            ImGui::SameLine();
            if (ImGui::SmallButton("Delete")) remove = static_cast<int>(i);

            int kind = m.kind == MotionKind::Circular ? 0 : 1;
            if (ImGui::RadioButton("Circular", &kind, 0)) m.kind = MotionKind::Circular;
            ImGui::SameLine();
            if (ImGui::RadioButton("Linear", &kind, 1)) m.kind = MotionKind::Linear;

            // Moved bodies.
            ImGui::TextDisabled("Bodies");
            for (std::size_t b = 0; b < m.bodies.size(); ++b) {
                ImGui::PushID(static_cast<int>(b));
                ImGui::BulletText("%s", m.bodies[b] == "*" ? "All bodies" : m.bodies[b].c_str());
                ImGui::SameLine();
                if (ImGui::SmallButton("x") && m.bodies.size() > 1) {
                    m.bodies.erase(m.bodies.begin() + static_cast<std::ptrdiff_t>(b));
                    ImGui::PopID();
                    break;
                }
                ImGui::PopID();
            }
            {
                const bool all = m.bodies.size() == 1 && m.bodies[0] == "*";
                const std::string sel = hasSel ? bodies[static_cast<std::size_t>(st.selectedBody)].name : std::string();
                const bool canAdd = hasSel && !all && !m.moves(sel);
                if (canAdd && ImGui::SmallButton(("+ " + sel).c_str())) {
                    if (all) m.bodies.clear();
                    m.bodies.push_back(sel);
                }
                if (!all) {
                    if (canAdd) ImGui::SameLine();
                    if (ImGui::SmallButton("All bodies")) m.bodies = {"*"};
                }
            }

            // Axis / direction.
            ImGui::TextDisabled(m.kind == MotionKind::Circular ? "Axis" : "Direction");
            ImGui::SameLine(90);
            const Vec3 presets[] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
            const char* names[] = {"X", "Y", "Z"};
            for (int k = 0; k < 3; ++k) {
                const bool on = std::fabs(m.axis.x - presets[k].x) < 1e-4f && std::fabs(m.axis.y - presets[k].y) < 1e-4f &&
                                std::fabs(m.axis.z - presets[k].z) < 1e-4f;
                if (on) ImGui::PushStyleColor(ImGuiCol_Button, ImGui::ColorConvertU32ToFloat4(kAccent));
                if (ImGui::SmallButton(names[k])) m.axis = presets[k];
                if (on) ImGui::PopStyleColor();
                ImGui::SameLine();
            }
            if (ImGui::SmallButton("Flip")) m.axis = m.axis * -1.0f;
            ImGui::SetNextItemWidth(-1);
            ImGui::DragFloat3("##axis", &m.axis.x, 0.01f, -1.0f, 1.0f, "%.2f");
            if (m.kind == MotionKind::Circular) {
                ImGui::Checkbox("Each body about its own centre", &m.eachCentre);
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("One motion spins every wheel about its own hub");
            }
            if (m.kind == MotionKind::Circular) {
                // Gear trains: turn with another circular motion at a ratio.
                ImGui::TextDisabled("Driven by");
                ImGui::SameLine(90);
                ImGui::SetNextItemWidth(-1);
                if (ImGui::BeginCombo("##drivenby", m.follow.empty() ? "Own speed" : m.follow.c_str())) {
                    if (ImGui::Selectable("Own speed", m.follow.empty())) m.follow.clear();
                    for (const Motion& d : motions)
                        if (&d != &m && d.kind == MotionKind::Circular && ImGui::Selectable(d.name.c_str(), d.name == m.follow))
                            m.follow = d.name;
                    ImGui::EndCombo();
                }
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("Turns with another motion: angle = ratio x its angle");
                if (!m.follow.empty()) {
                    ImGui::TextDisabled("Ratio");
                    ImGui::SameLine(90);
                    ImGui::SetNextItemWidth(130);
                    ImGui::DragFloat("##ratio", &m.ratio, 0.01f, -1000.0f, 1000.0f, "%.3f");
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("External mesh: -teeth(driver) / teeth(driven)");
                } else {
                    ImGui::TextDisabled("Step");
                    ImGui::SameLine(90);
                    ImGui::SetNextItemWidth(130);
                    ImGui::DragFloat("##step", &m.step, 0.05f, 0.0f, 360.0f, m.step <= 0.0f ? "smooth" : "%.2f deg");
                    m.step = std::max(m.step, 0.0f);
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Stepper motors: moves in steps of this angle (1.8 deg = 200 steps/rev)");
                }
            }
            if (m.kind == MotionKind::Linear) {
                ImGui::Checkbox("Travel along the chain / belt loop", &m.alongLoop);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Moves a chain or belt body around its own loop instead of sliding it");
                ImGui::TextDisabled("Rolls with");
                ImGui::SameLine(90);
                ImGui::SetNextItemWidth(-1);
                if (ImGui::BeginCombo("##follow", m.follow.empty() ? "Own speed" : m.follow.c_str())) {
                    if (ImGui::Selectable("Own speed", m.follow.empty())) m.follow.clear();
                    for (const Motion& d : motions)
                        if (d.kind == MotionKind::Circular && ImGui::Selectable(d.name.c_str(), d.name == m.follow))
                            m.follow = d.name;
                    ImGui::EndCombo();
                }
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("Travel follows a wheel spin: distance = wheel angle x radius");
                if (!m.follow.empty()) {
                    ImGui::TextDisabled("Radius");
                    ImGui::SameLine(90);
                    ImGui::SetNextItemWidth(130);
                    lengthField("##radius", &m.radius, 0.0f, 1e6f, "", "wheel size");
                    m.radius = std::max(m.radius, 0.0f);
                }
            }
            if (m.kind == MotionKind::Circular && !m.eachCentre) {
                ImGui::TextDisabled("Through");
                ImGui::SameLine(90);
                if (ImGui::SmallButton("Centre of bodies")) {
                    AABB box;
                    for (const Body& b : bodies)
                        if (m.moves(b.name)) box.expand(b.bounds());
                    if (box.valid()) m.origin = box.center();
                }
                const float third = (ImGui::GetContentRegionAvail().x - 2.0f * ImGui::GetStyle().ItemInnerSpacing.x) / 3.0f;
                for (int k = 0; k < 3; ++k) {
                    if (k) ImGui::SameLine(0, ImGui::GetStyle().ItemInnerSpacing.x);
                    ImGui::SetNextItemWidth(third);
                    const char* ids[] = {"##ox", "##oy", "##oz"};
                    lengthField(ids[k], k == 0 ? &m.origin.x : k == 1 ? &m.origin.y : &m.origin.z);
                }
            }

            // Speed, range, mode, delay.
            ImGui::TextDisabled("Speed");
            ImGui::SameLine(90);
            ImGui::SetNextItemWidth(130);
            if (m.kind == MotionKind::Circular) ImGui::DragFloat("##speed", &m.speed, 1.0f, -100000.0f, 100000.0f, "%.1f deg/s");
            else lengthField("##speed", &m.speed, -1e6f, 1e6f, "/s");
            if (m.kind == MotionKind::Circular) {
                ImGui::SameLine();
                ImGui::TextDisabled("%.1f rpm", static_cast<double>(m.speed / 6.0f));
            }
            ImGui::TextDisabled(m.kind == MotionKind::Circular ? "Sweep" : "Stroke");
            ImGui::SameLine(90);
            ImGui::SetNextItemWidth(110);
            if (m.kind == MotionKind::Circular)
                ImGui::DragFloat("##range", &m.range, 1.0f, 0.0f, 1e6f, m.range <= 0.0f ? "unlimited" : "%.1f deg");
            else lengthField("##range", &m.range, 0.0f, 1e7f, "", "unlimited");
            m.range = std::max(m.range, 0.0f);
            ImGui::SameLine();
            int mode = static_cast<int>(m.mode);
            ImGui::SetNextItemWidth(-1);
            if (m.range <= 0.0f) ImGui::BeginDisabled();
            if (ImGui::Combo("##mode", &mode, "Continuous\0Loop\0Oscillate\0Once\0")) m.mode = static_cast<MotionMode>(mode);
            if (m.range <= 0.0f) ImGui::EndDisabled();
            ImGui::TextDisabled("Start");
            ImGui::SameLine(90);
            ImGui::SetNextItemWidth(110);
            ImGui::DragFloat("##start", &m.start, 0.05f, 0.0f, 3600.0f, "%.2f s");
            m.start = std::max(m.start, 0.0f);

            // Live value.
            ImGui::TextDisabled("Now");
            ImGui::SameLine(90);
            if (m.kind == MotionKind::Circular)
                ImGui::TextColored(col, "%.2f deg   %.1f deg/s", static_cast<double>(motionValue(scene, m, st.simTime)),
                                   static_cast<double>(motionRate(scene, m, st.simTime)));
            else
                ImGui::TextColored(col, "%s   %s/s", lengthText(motionValue(scene, m, st.simTime)).c_str(),
                                   lengthText(motionRate(scene, m, st.simTime)).c_str());
            ImGui::Spacing();
        }
        ImGui::PopID();
    }
    if (remove >= 0) cmd.commands.push_back("MOTION DELETE " + quotedName(motions[static_cast<std::size_t>(remove)].name));
    ImGui::EndChild();

    // Real-time axes: digital readout of the selected (or first moving) body.
    if (st.simReadout) {
        theme::caption("Real-time axes");
        int body = hasSel ? st.selectedBody : -1;
        for (std::size_t i = 0; i < bodies.size() && body < 0; ++i)
            if (isMoving(scene, i)) body = static_cast<int>(i);
        if (body < 0) {
            ImGui::TextDisabled("Add a motion to read the moving axes.");
        } else {
            const Body& b = bodies[static_cast<std::size_t>(body)];
            const float t = st.simTime, h = 1.0f / 120.0f;
            const Mat4 now = motionTransforms(scene, t)[static_cast<std::size_t>(body)];
            const Mat4 next = motionTransforms(scene, t + h)[static_cast<std::size_t>(body)];
            const Vec3 c0 = b.bounds().center();
            const Vec3 p = now.transformPoint(c0), v = (next.transformPoint(c0) - p) * (1.0f / h);
            const Vec3 r = eulerABC(now), r1 = eulerABC(next);
            auto wrap = [](float d) { return d > 180.0f ? d - 360.0f : d < -180.0f ? d + 360.0f : d; };
            const Vec3 w{wrap(r1.x - r.x) / h, wrap(r1.y - r.y) / h, wrap(r1.z - r.z) / h};
            ImGui::Text("%s%s", b.name.c_str(), isMoving(scene, static_cast<std::size_t>(body)) ? "" : "  (static)");
            if (ImGui::BeginTable("##dro", 3, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg)) {
                const ImVec4 axisCol[] = {{0.95f, 0.40f, 0.40f, 1}, {0.45f, 0.85f, 0.45f, 1}, {0.45f, 0.65f, 1.00f, 1}};
                const char* lin[] = {"X", "Y", "Z"};
                const char* rot[] = {"A", "B", "C"};
                const float pv[] = {p.x, p.y, p.z}, vv[] = {v.x, v.y, v.z}, rv[] = {r.x, r.y, r.z}, wv[] = {w.x, w.y, w.z};
                for (int k = 0; k < 3; ++k) {
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();
                    ImGui::TextColored(axisCol[k], "%s %9.2f", lin[k], static_cast<double>(pv[k]));
                    ImGui::TableNextColumn();
                    ImGui::TextColored(axisCol[k], "%s %8.2f", rot[k], static_cast<double>(rv[k]));
                    ImGui::TableNextColumn();
                    ImGui::TextDisabled("%7.1f  %6.1f", static_cast<double>(vv[k]), static_cast<double>(wv[k]));
                }
                ImGui::EndTable();
            }
            ImGui::TextDisabled("lengths in mm  ·  deg  ·  mm/s, deg/s   |v| %s/s", lengthText(length(v)).c_str());
        }
    }
    ImGui::End();
    ImGui::PopStyleVar();
    return kWidth;
}

} // namespace sicdm

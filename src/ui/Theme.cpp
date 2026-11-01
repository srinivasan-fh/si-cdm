#include "ui/Theme.hpp"

#include "ui/EmbeddedFonts.hpp"

#include <algorithm>
#include <cctype>
#include <string>

namespace sicdm::theme {

namespace {

ImFont* g_fonts[5] = {};

ImFont* load(const fonts::Blob& blob, float sizePx, float rasterScale, const ImWchar* ranges) {
    ImFontConfig cfg;
    cfg.FontDataOwnedByAtlas = false; // static data
    cfg.OversampleH = 2;
    cfg.OversampleV = 1;
    cfg.RasterizerMultiply = 1.08f; // a touch more weight: light text on dark panels
    ImGuiIO& io = ImGui::GetIO();
    return io.Fonts->AddFontFromMemoryTTF(const_cast<unsigned char*>(blob.data), static_cast<int>(blob.size),
                                          sizePx * rasterScale, &cfg, ranges);
}

} // namespace

ImFont* font(Font f) { return g_fonts[static_cast<int>(f)] ? g_fonts[static_cast<int>(f)] : ImGui::GetFont(); }

void apply(float uiScale, float framebufferScale) {
    uiScale = std::max(uiScale, 1.0f);
    framebufferScale = std::max(framebufferScale, 1.0f);
    ImGuiIO& io = ImGui::GetIO();

    // Fonts: rasterised at the framebuffer resolution, drawn at window size.
    static const ImWchar kRanges[] = {0x0020, 0x017F, 0x2013, 0x2026, 0x2190, 0x21FF, 0x2212, 0x2265,
                                      0x2300, 0x2325, 0x25B2, 0x25C0, 0x2713, 0x2715, 0};
    const float r = uiScale * framebufferScale;
    io.Fonts->Clear();
    g_fonts[static_cast<int>(Font::Body)] = load(fonts::Inter_Regular, 13.0f, r, kRanges);
    g_fonts[static_cast<int>(Font::Strong)] = load(fonts::Inter_SemiBold, 13.0f, r, kRanges);
    g_fonts[static_cast<int>(Font::Small)] = load(fonts::Inter_Regular, 11.5f, r, kRanges);
    g_fonts[static_cast<int>(Font::Caption)] = load(fonts::Inter_SemiBold, 10.5f, r, kRanges);
    g_fonts[static_cast<int>(Font::Mono)] = load(fonts::JetBrainsMono_Regular, 12.5f, r, kRanges);
    io.FontDefault = g_fonts[static_cast<int>(Font::Body)];
    io.FontGlobalScale = 1.0f / framebufferScale;

    ImGuiStyle& s = ImGui::GetStyle();
    s = ImGuiStyle();
    // Spacing: compact like CAD / DCC tools, 22 pt controls, 8 pt rhythm.
    s.WindowPadding = {kPanelPadding, kPanelPadding};
    s.FramePadding = {8, 4.5f};
    s.CellPadding = {6, 3};
    s.ItemSpacing = {8, 6};
    s.ItemInnerSpacing = {6, 4};
    s.IndentSpacing = 16;
    s.ScrollbarSize = 10;
    s.GrabMinSize = 9;
    s.WindowBorderSize = 1;
    s.ChildBorderSize = 1;
    s.PopupBorderSize = 1;
    s.FrameBorderSize = 1;
    s.TabBorderSize = 0;
    s.TabBarBorderSize = 1;
    s.TabBarOverlineSize = 2;
    s.WindowRounding = 6;
    s.ChildRounding = 4;
    s.FrameRounding = 3;
    s.PopupRounding = 5;
    s.ScrollbarRounding = 5;
    s.GrabRounding = 3;
    s.TabRounding = 4;
    s.WindowTitleAlign = {0.0f, 0.5f};
    s.WindowMenuButtonPosition = ImGuiDir_None;
    s.SeparatorTextBorderSize = 1;
    s.SeparatorTextAlign = {0.0f, 0.5f};
    s.SeparatorTextPadding = {0, 4};
    s.SelectableTextAlign = {0.0f, 0.5f};
    s.DisplaySafeAreaPadding = {4, 4};
    s.ScaleAllSizes(uiScale);

    applyColors();
}

namespace {

std::string g_palette = "dark";

struct Palette {
    ImU32 chrome, panel, raised, hover, field, border, line, text, textDim, textFaint, accent, accentHover, accentText, warning, success,
        error, icon, iconActive, iconOff, tile, tileHover, rulerBg, rulerCorner, rulerTick, rulerText, errorField;
};

Palette palette(const std::string& name) {
    if (name == "light")
        return {rgb(232, 235, 240), rgb(245, 246, 248), rgb(226, 229, 235), rgb(212, 217, 225), rgb(255, 255, 255), rgb(184, 190, 200),
                rgb(204, 209, 217), rgb(28, 31, 36),    rgb(88, 95, 106),   rgb(128, 135, 146), rgb(37, 110, 214),  rgb(56, 128, 232),
                rgb(22, 94, 196),   rgb(170, 104, 0),   rgb(28, 135, 58),   rgb(196, 48, 42),   rgb(58, 64, 75),    rgb(255, 255, 255),
                rgb(160, 166, 176), rgb(226, 229, 235), rgb(208, 214, 223), rgb(236, 238, 242), rgb(226, 229, 234), rgb(130, 136, 148),
                rgb(80, 86, 98),    rgb(255, 218, 216)};
    if (name == "high-contrast")
        return {rgb(0, 0, 0),       rgb(0, 0, 0),       rgb(24, 24, 24),    rgb(70, 70, 0),     rgb(0, 0, 0),       rgb(255, 255, 255),
                rgb(220, 220, 220), rgb(255, 255, 255), rgb(240, 240, 240), rgb(210, 210, 210), rgb(0, 84, 255),    rgb(40, 120, 255),
                rgb(255, 230, 0),   rgb(255, 200, 0),   rgb(0, 255, 120),   rgb(255, 80, 80),   rgb(255, 255, 255), rgb(255, 255, 255),
                rgb(170, 170, 170), rgb(0, 0, 0),       rgb(70, 70, 0),     rgb(0, 0, 0),       rgb(0, 0, 0),       rgb(255, 255, 255),
                rgb(255, 255, 255), rgb(120, 0, 0)};
    return {rgb(28, 30, 34),    rgb(37, 39, 44),    rgb(45, 48, 54),    rgb(56, 60, 68),    rgb(25, 27, 31),    rgb(14, 15, 18),
            rgb(58, 62, 70),    rgb(220, 223, 228), rgb(142, 148, 158), rgb(104, 110, 120), rgb(47, 123, 224),  rgb(66, 140, 238),
            rgb(110, 172, 255), rgb(240, 178, 90),  rgb(110, 205, 130), rgb(240, 110, 105), rgb(205, 210, 218), rgb(255, 255, 255),
            rgb(130, 135, 145), rgb(40, 44, 52),    rgb(58, 64, 76),    rgb(30, 33, 39),    rgb(24, 26, 31),    rgb(120, 126, 138),
            rgb(170, 175, 186), rgb(92, 38, 40)};
}

} // namespace

void setPalette(const std::string& name) {
    g_palette = name == "light" || name == "high-contrast" ? name : "dark";
    const Palette p = palette(g_palette);
    kChrome = p.chrome, kPanel = p.panel, kPanelRaised = p.raised, kHover = p.hover, kField = p.field, kBorder = p.border, kLine = p.line;
    kText = p.text, kTextDim = p.textDim, kTextFaint = p.textFaint, kAccent = p.accent, kAccentHover = p.accentHover, kAccentText = p.accentText;
    kWarning = p.warning, kSuccess = p.success, kError = p.error, kIcon = p.icon, kIconActive = p.iconActive, kIconOff = p.iconOff;
    kTile = p.tile, kTileHover = p.tileHover, kRulerBg = p.rulerBg, kRulerCorner = p.rulerCorner, kRulerTick = p.rulerTick, kRulerText = p.rulerText;
    kErrorField = p.errorField;
}

const std::string& paletteName() { return g_palette; }

void applyColors() {
    ImVec4* c = ImGui::GetStyle().Colors;
    c[ImGuiCol_Text] = vec(kText);
    c[ImGuiCol_TextDisabled] = vec(kTextFaint);
    c[ImGuiCol_WindowBg] = vec(kPanel);
    c[ImGuiCol_ChildBg] = {0, 0, 0, 0};
    c[ImGuiCol_PopupBg] = vec((kPanelRaised & 0x00FFFFFFu) | (252u << 24));
    c[ImGuiCol_Border] = vec((kLine & 0x00FFFFFFu) | (200u << 24));
    c[ImGuiCol_BorderShadow] = {0, 0, 0, 0};
    c[ImGuiCol_FrameBg] = vec(kField);
    c[ImGuiCol_FrameBgHovered] = vec(kHover);
    c[ImGuiCol_FrameBgActive] = vec(kHover);
    c[ImGuiCol_TitleBg] = vec(kChrome);
    c[ImGuiCol_TitleBgActive] = vec(kPanelRaised);
    c[ImGuiCol_TitleBgCollapsed] = vec(kChrome);
    c[ImGuiCol_MenuBarBg] = vec(kChrome);
    c[ImGuiCol_ScrollbarBg] = {0, 0, 0, 0};
    c[ImGuiCol_ScrollbarGrab] = vec(kLine);
    c[ImGuiCol_ScrollbarGrabHovered] = vec(kTextFaint);
    c[ImGuiCol_ScrollbarGrabActive] = vec(kTextDim);
    c[ImGuiCol_CheckMark] = vec(kAccentText);
    c[ImGuiCol_SliderGrab] = vec(kAccent);
    c[ImGuiCol_SliderGrabActive] = vec(kAccentHover);
    c[ImGuiCol_Button] = vec(kPanelRaised);
    c[ImGuiCol_ButtonHovered] = vec(kHover);
    c[ImGuiCol_ButtonActive] = vec(kAccent);
    c[ImGuiCol_Header] = vec((kAccent & 0x00FFFFFFu) | (70u << 24));
    c[ImGuiCol_HeaderHovered] = vec((kText & 0x00FFFFFFu) | (22u << 24));
    c[ImGuiCol_HeaderActive] = vec((kAccent & 0x00FFFFFFu) | (110u << 24));
    c[ImGuiCol_Separator] = vec(kLine);
    c[ImGuiCol_SeparatorHovered] = vec(kAccent);
    c[ImGuiCol_SeparatorActive] = vec(kAccentHover);
    c[ImGuiCol_ResizeGrip] = {0, 0, 0, 0};
    c[ImGuiCol_ResizeGripHovered] = vec(kAccent);
    c[ImGuiCol_ResizeGripActive] = vec(kAccentHover);
    c[ImGuiCol_Tab] = vec(kChrome);
    c[ImGuiCol_TabHovered] = vec(kHover);
    c[ImGuiCol_TabSelected] = vec(kPanel);
    c[ImGuiCol_TabSelectedOverline] = vec(kAccent);
    c[ImGuiCol_TabDimmed] = vec(kChrome);
    c[ImGuiCol_TabDimmedSelected] = vec(kPanel);
    c[ImGuiCol_TabDimmedSelectedOverline] = vec(kLine);
    c[ImGuiCol_TableHeaderBg] = vec(kPanelRaised);
    c[ImGuiCol_TableBorderStrong] = vec(kLine);
    c[ImGuiCol_TableBorderLight] = vec(kLine);
    c[ImGuiCol_TableRowBg] = {0, 0, 0, 0};
    c[ImGuiCol_TableRowBgAlt] = vec((kText & 0x00FFFFFFu) | (10u << 24));
    c[ImGuiCol_TextSelectedBg] = vec((kAccent & 0x00FFFFFFu) | (110u << 24));
    c[ImGuiCol_DragDropTarget] = vec(kAccentHover);
    c[ImGuiCol_NavCursor] = vec(kAccent);
    c[ImGuiCol_ModalWindowDimBg] = vec(rgb(0, 0, 0, 110));
}

void caption(const char* text) {
    std::string upper(text);
    for (char& ch : upper) ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    ImGui::Dummy({0, 2});
    const float right = ImGui::GetCursorScreenPos().x + ImGui::GetContentRegionAvail().x;
    ImGui::PushFont(font(Font::Caption));
    ImGui::PushStyleColor(ImGuiCol_Text, vec(kTextDim));
    ImGui::TextUnformatted(upper.c_str());
    ImGui::PopStyleColor();
    ImGui::PopFont();
    const ImVec2 lo = ImGui::GetItemRectMax();
    const float y = (ImGui::GetItemRectMin().y + lo.y) * 0.5f;
    ImGui::GetWindowDrawList()->AddLine({lo.x + 8, y}, {right, y}, kLine);
}

} // namespace sicdm::theme

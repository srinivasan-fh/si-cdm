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

    ImVec4* c = s.Colors;
    c[ImGuiCol_Text] = vec(kText);
    c[ImGuiCol_TextDisabled] = vec(kTextFaint);
    c[ImGuiCol_WindowBg] = vec(kPanel);
    c[ImGuiCol_ChildBg] = {0, 0, 0, 0};
    c[ImGuiCol_PopupBg] = vec(rgb(43, 46, 52, 252));
    c[ImGuiCol_Border] = vec(rgb(62, 66, 75, 200));
    c[ImGuiCol_BorderShadow] = {0, 0, 0, 0};
    c[ImGuiCol_FrameBg] = vec(kField);
    c[ImGuiCol_FrameBgHovered] = vec(rgb(31, 34, 39));
    c[ImGuiCol_FrameBgActive] = vec(rgb(34, 37, 43));
    c[ImGuiCol_TitleBg] = vec(kChrome);
    c[ImGuiCol_TitleBgActive] = vec(rgb(33, 35, 40));
    c[ImGuiCol_TitleBgCollapsed] = vec(kChrome);
    c[ImGuiCol_MenuBarBg] = vec(kChrome);
    c[ImGuiCol_ScrollbarBg] = {0, 0, 0, 0};
    c[ImGuiCol_ScrollbarGrab] = vec(rgb(70, 75, 84));
    c[ImGuiCol_ScrollbarGrabHovered] = vec(rgb(88, 94, 104));
    c[ImGuiCol_ScrollbarGrabActive] = vec(rgb(104, 110, 120));
    c[ImGuiCol_CheckMark] = vec(kAccentText);
    c[ImGuiCol_SliderGrab] = vec(kAccent);
    c[ImGuiCol_SliderGrabActive] = vec(kAccentHover);
    c[ImGuiCol_Button] = vec(kPanelRaised);
    c[ImGuiCol_ButtonHovered] = vec(kHover);
    c[ImGuiCol_ButtonActive] = vec(kAccent);
    c[ImGuiCol_Header] = vec(rgb(47, 123, 224, 70));
    c[ImGuiCol_HeaderHovered] = vec(rgb(255, 255, 255, 18));
    c[ImGuiCol_HeaderActive] = vec(rgb(47, 123, 224, 110));
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
    c[ImGuiCol_TableBorderLight] = vec(rgb(52, 56, 63));
    c[ImGuiCol_TableRowBg] = {0, 0, 0, 0};
    c[ImGuiCol_TableRowBgAlt] = vec(rgb(255, 255, 255, 8));
    c[ImGuiCol_TextSelectedBg] = vec(rgb(47, 123, 224, 110));
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

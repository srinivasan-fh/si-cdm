#include "ui/Toolbox.hpp"

#include "core/Preferences.hpp"
#include "ui/KeyInput.hpp"
#include "core/Translate.hpp"

#include "ui/Theme.hpp"

#include "ui/ToolIcons.hpp"

#include <imgui.h>

#include <cmath>

namespace sicdm {

namespace {

constexpr float kSlot = 32.0f;
constexpr float kGap = 2.0f;
constexpr float kPad = 6.0f;
constexpr double kLongPress = 0.4; // seconds before a held slot opens its flyout

ImU32 iconColor(ToolId id, bool active) {
    if (active) return IM_COL32(255, 255, 255, 255);
    return toolInfo(id).available ? IM_COL32(205, 210, 218, 255) : IM_COL32(130, 135, 145, 255);
}

} // namespace

bool commandModifierHeld() {
    const ImGuiIO& io = ImGui::GetIO();
    return io.KeyCtrl || io.KeySuper || ImGui::IsKeyDown(ImGuiKey_LeftCtrl) || ImGui::IsKeyDown(ImGuiKey_RightCtrl) ||
           ImGui::IsKeyDown(ImGuiKey_LeftSuper) || ImGui::IsKeyDown(ImGuiKey_RightSuper);
}

Toolbox::Toolbox() : m_current(toolGroups().size(), 0) {
    registerToolKeys(); // the tools' own keys, the keymap's defaults for them
    setActive(ToolId::Select);
}

int Toolbox::groupOf(ToolId id) const {
    const auto groups = toolGroups();
    for (std::size_t g = 0; g < groups.size(); ++g)
        for (ToolId t : groups[g].tools)
            if (t == id) return static_cast<int>(g);
    return -1;
}

void Toolbox::setActive(ToolId id) {
    m_active = id;
    const int g = groupOf(id);
    if (g < 0) return;
    const auto tools = toolGroups()[static_cast<std::size_t>(g)].tools;
    for (std::size_t i = 0; i < tools.size(); ++i)
        if (tools[i] == id) m_current[static_cast<std::size_t>(g)] = static_cast<int>(i);
}

void Toolbox::handleShortcuts() {
    const ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureKeyboard) return;
    for (int i = 0; i < static_cast<int>(ToolId::Count); ++i) {
        const ToolInfo& info = toolInfo(static_cast<ToolId>(i));
        // The tool's keys (the user's, else its own); with Shift too, they step through its slot.
        bool hit = false, cycle = false;
        for (KeyChord c : boundChords(preferences(), toolKeyId(info.name))) {
            hit = hit || chordPressed(c);
            if (!c.shift && (c.shift = true, chordPressed(c))) cycle = true;
        }
        if (!hit && !cycle) continue;
        const int g = groupOf(info.id);
        const auto tools = toolGroups()[static_cast<std::size_t>(g)].tools;
        int& cur = m_current[static_cast<std::size_t>(g)];
        if (cycle && groupOf(m_active) == g) cur = (cur + 1) % static_cast<int>(tools.size());
        setActive(tools[static_cast<std::size_t>(cur)]);
        return;
    }
}

std::string toolKeysText(ToolId id) {
    const std::vector<KeyChord> c = boundChords(preferences(), toolKeyId(toolInfo(id).name));
    return c.empty() ? std::string() : chordText(c.front());
}

float Toolbox::draw(float top, float bottom, bool drawing2D, bool& panelOpen, int& panelTab) {
    m_drawing2D = drawing2D;
    const int cols = m_twoColumns ? 2 : 1;
    const float width = kPad * 2.0f + kSlot * static_cast<float>(cols) + kGap * static_cast<float>(cols - 1);

    const ImGuiViewport* mv = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos({mv->WorkPos.x, top});
    ImGui::SetNextWindowSize({width, bottom - top});
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {kPad, kPad});
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, {kGap, kGap});
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    constexpr ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                                       ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus;
    ImGui::Begin("##toolbox", nullptr, flags);
    {
        // Hairline between the rail and the canvas / panel.
        const ImVec2 wp = ImGui::GetWindowPos(), ws = ImGui::GetWindowSize();
        ImGui::GetWindowDrawList()->AddLine({wp.x + ws.x - 1, wp.y}, {wp.x + ws.x - 1, wp.y + ws.y}, theme::kBorder);
    }

    // Header: expand / collapse between one and two columns (double chevron, like Photoshop).
    {
        ImGui::InvisibleButton("##columns", {width - kPad * 2.0f, 16.0f});
        const ImVec2 lo = ImGui::GetItemRectMin(), hi = ImGui::GetItemRectMax();
        const bool hov = ImGui::IsItemHovered();
        if (ImGui::IsItemClicked()) m_twoColumns = !m_twoColumns;
        ImDrawList* dl = ImGui::GetWindowDrawList();
        if (hov) dl->AddRectFilled(lo, hi, theme::kHover, 3.0f);
        const ImU32 c = hov ? theme::kText : theme::kTextDim;
        const float cx = (lo.x + hi.x) * 0.5f, cy = (lo.y + hi.y) * 0.5f;
        const float d = m_twoColumns ? 1.0f : -1.0f; // << collapses, >> expands
        for (float off : {-3.5f, 2.5f}) {
            const float x = cx + off * d;
            dl->AddLine({x + 2.5f * d, cy - 3.5f}, {x - 1.0f * d, cy}, c, 1.4f);
            dl->AddLine({x - 1.0f * d, cy}, {x + 2.5f * d, cy + 3.5f}, c, 1.4f);
        }
        if (hov) ImGui::SetTooltip(m_twoColumns ? "Single-column tools" : "Two-column tools");
    }

    // Panel toggles: Model (tree icon) and Library (gear icon) dock beside the rail.
    // Two columns: side by side; one column: stacked.
    auto panelButton = [&](const char* id, int tab, float w) {
        ImGui::InvisibleButton(id, {w, 26.0f});
        const ImVec2 lo = ImGui::GetItemRectMin(), hi = ImGui::GetItemRectMax();
        const bool hovered = ImGui::IsItemHovered();
        const bool on = panelOpen && panelTab == tab;
        if (ImGui::IsItemClicked()) {
            panelOpen = !on;
            panelTab = tab;
        }
        ImDrawList* dl = ImGui::GetWindowDrawList();
        if (on) dl->AddRectFilled(lo, hi, theme::kAccent, 4.0f);
        else if (hovered) dl->AddRectFilled(lo, hi, theme::kHover, 4.0f);
        const ImU32 c = on ? IM_COL32(255, 255, 255, 255) : IM_COL32(205, 210, 218, 255);
        const ImVec2 mid{(lo.x + hi.x) * 0.5f, (lo.y + hi.y) * 0.5f};
        if (tab == 0) { // root node + two indented children
            const ImVec2 o{mid.x - 7.0f, mid.y - 6.0f};
            dl->AddRectFilled({o.x, o.y}, {o.x + 5, o.y + 3}, c);
            dl->AddLine({o.x + 2, o.y + 3}, {o.x + 2, o.y + 11}, c);
            dl->AddLine({o.x + 2, o.y + 6}, {o.x + 6, o.y + 6}, c);
            dl->AddLine({o.x + 2, o.y + 11}, {o.x + 6, o.y + 11}, c);
            dl->AddRectFilled({o.x + 7, o.y + 5}, {o.x + 14, o.y + 7.5f}, c);
            dl->AddRectFilled({o.x + 7, o.y + 10}, {o.x + 14, o.y + 12.5f}, c);
        } else { // gear
            for (int t = 0; t < 8; ++t) {
                const float a = static_cast<float>(t) * 0.785398f;
                const ImVec2 d{std::cos(a), std::sin(a)};
                dl->AddLine({mid.x + d.x * 4.5f, mid.y + d.y * 4.5f}, {mid.x + d.x * 7.5f, mid.y + d.y * 7.5f}, c, 2.4f);
            }
            dl->AddCircle(mid, 5.0f, c, 16, 2.0f);
            dl->AddCircleFilled(mid, 1.6f, c);
        }
        if (hovered)
            ImGui::SetTooltip(tab == 0 ? (on ? "Hide Model panel  (M)" : "Show Model panel  (M)")
                                       : (on ? "Hide Library" : "Library: motors, gears, gear trains  (K)"));
    };
    {
        const float full = width - kPad * 2.0f;
        if (m_twoColumns) {
            const float half = (full - kGap) * 0.5f;
            panelButton("##modelpanel", 0, half);
            ImGui::SameLine();
            panelButton("##library", 1, half);
        } else {
            panelButton("##modelpanel", 0, full);
            panelButton("##library", 1, full);
        }
    }

    const auto groups = toolGroups();
    int col = 0;
    for (std::size_t g = 0; g < groups.size(); ++g) {
        if (g == 0 || groups[g].category != groups[g - 1].category) {
            // Section divider: always starts a new row.
            col = 0;
            ImGui::Dummy({width - kPad * 2.0f, 5.0f});
            const ImVec2 a = ImGui::GetItemRectMin(), b = ImGui::GetItemRectMax();
            const float y = (a.y + b.y) * 0.5f;
            ImGui::GetWindowDrawList()->AddLine({a.x + 4, y}, {b.x - 4, y}, theme::kLine, 1.0f);
        }
        if (col > 0) ImGui::SameLine();
        drawSlot(static_cast<int>(g), kSlot);
        col = (col + 1) % cols;
    }

    ImGui::End();
    ImGui::PopStyleVar(4);
    return width;
}

void Toolbox::drawSlot(int gi, float size) {
    const ToolGroup& group = toolGroups()[static_cast<std::size_t>(gi)];
    const ToolId shown = group.tools[static_cast<std::size_t>(m_current[static_cast<std::size_t>(gi)])];
    const bool isActive = groupOf(m_active) == gi;

    ImGui::PushID(gi);
    ImGui::InvisibleButton("slot", {size, size}, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
    const ImVec2 lo = ImGui::GetItemRectMin(), hi = ImGui::GetItemRectMax();
    const bool hovered = ImGui::IsItemHovered();

    ImDrawList* dl = ImGui::GetWindowDrawList();
    if (isActive) dl->AddRectFilled(lo, hi, theme::kAccent, 4.0f);
    else if (hovered) dl->AddRectFilled(lo, hi, theme::kHover, 4.0f);
    const bool dimmed = m_drawing2D && shown == ToolId::Orbit;
    drawToolIcon(dl, shown, {(lo.x + hi.x) * 0.5f, (lo.y + hi.y) * 0.5f}, size * 0.56f,
                 dimmed ? IM_COL32(95, 100, 110, 255) : iconColor(shown, isActive));

    const bool hasFlyout = group.tools.size() > 1;
    if (hasFlyout) // corner triangle, as in Photoshop
        dl->AddTriangleFilled({hi.x - 2, hi.y - 7}, {hi.x - 2, hi.y - 2}, {hi.x - 7, hi.y - 2},
                              IM_COL32(185, 190, 198, 255));

    // Click selects; right-click or a long press opens the flyout.
    bool openFlyout = false;
    if (ImGui::IsItemActivated() && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        m_pressedGroup = gi;
        m_pressTime = ImGui::GetTime();
    }
    if (m_pressedGroup == gi && ImGui::IsItemActive() && hasFlyout && ImGui::GetTime() - m_pressTime > kLongPress) {
        openFlyout = true;
        m_pressedGroup = -1;
    }
    if (ImGui::IsItemDeactivated() && m_pressedGroup == gi) {
        if (hovered) setActive(shown);
        m_pressedGroup = -1;
    }
    if (ImGui::IsItemClicked(ImGuiMouseButton_Right)) {
        if (hasFlyout) openFlyout = true;
        else setActive(shown);
    }
    if (openFlyout) ImGui::OpenPopup("flyout");

    if (hovered && !ImGui::IsPopupOpen("flyout")) {
        const ToolInfo& info = toolInfo(shown);
        ImGui::BeginTooltip();
        if (const std::string keys = toolKeysText(shown); !keys.empty()) ImGui::Text("%s  (%s)", tr(info.name), keys.c_str());
        else ImGui::TextUnformatted(tr(info.name));
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 22.0f);
        ImGui::TextDisabled("%s", tr(info.description));
        ImGui::PopTextWrapPos();
        if (!info.available) ImGui::TextColored(theme::vec(theme::kWarning), "%s", tr("Planned for a later milestone"));
        if (dimmed) ImGui::TextColored(theme::vec(theme::kWarning), "%s", tr("3D window only"));
        if (hasFlyout) ImGui::TextDisabled(tr("Right-click or hold for %zu more"), group.tools.size() - 1);
        ImGui::EndTooltip();
    }

    // Flyout opens beside the rail, aligned with the slot.
    ImGui::SetNextWindowPos({ImGui::GetWindowPos().x + ImGui::GetWindowWidth() + 4.0f, lo.y - 4.0f});
    drawFlyout(gi);
    ImGui::PopID();
}

void Toolbox::drawFlyout(int gi) {
    if (!ImGui::BeginPopup("flyout")) return;
    const ToolGroup& group = toolGroups()[static_cast<std::size_t>(gi)];
    theme::caption(categoryName(group.category));

    constexpr float rowH = 26.0f, rowW = 250.0f;
    for (std::size_t i = 0; i < group.tools.size(); ++i) {
        const ToolId id = group.tools[i];
        const ToolInfo& info = toolInfo(id);
        ImGui::PushID(static_cast<int>(i));
        if (ImGui::Selectable("##tool", m_active == id, 0, {rowW, rowH})) {
            setActive(id);
            ImGui::CloseCurrentPopup();
        }
        const ImVec2 lo = ImGui::GetItemRectMin();
        ImDrawList* dl = ImGui::GetWindowDrawList();
        const bool current = static_cast<int>(i) == m_current[static_cast<std::size_t>(gi)];
        if (current) dl->AddRectFilled({lo.x - 4, lo.y + 6}, {lo.x - 1, lo.y + rowH - 6}, theme::kAccentText);
        drawToolIcon(dl, id, {lo.x + 13, lo.y + rowH * 0.5f}, 18.0f, iconColor(id, false));
        const float ty = lo.y + (rowH - ImGui::GetFontSize()) * 0.5f;
        dl->AddText({lo.x + 32, ty}, info.available ? theme::kText : theme::kTextFaint,
                    tr(info.name));
        if (const std::string keys = toolKeysText(id); !keys.empty())
            dl->AddText({lo.x + rowW - 8 - ImGui::CalcTextSize(keys.c_str()).x, ty}, theme::kTextDim, keys.c_str());
        if (ImGui::IsItemHovered()) {
            ImGui::BeginTooltip();
            ImGui::PushTextWrapPos(ImGui::GetFontSize() * 22.0f);
            ImGui::TextUnformatted(tr(info.description));
            ImGui::PopTextWrapPos();
            if (!info.available) ImGui::TextColored(theme::vec(theme::kWarning), "%s", tr("Planned for a later milestone"));
            ImGui::EndTooltip();
        }
        ImGui::PopID();
    }
    ImGui::EndPopup();
}

} // namespace sicdm

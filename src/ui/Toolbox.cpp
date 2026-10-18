#include "ui/Toolbox.hpp"

#include "ui/ToolIcons.hpp"

#include <imgui.h>

namespace sicdm {

namespace {

constexpr float kSlot = 32.0f;
constexpr float kGap = 2.0f;
constexpr float kPad = 5.0f;
constexpr double kLongPress = 0.4; // seconds before a held slot opens its flyout

ImU32 iconColor(ToolId id, bool active) {
    if (active) return IM_COL32(255, 255, 255, 255);
    return toolInfo(id).available ? IM_COL32(205, 210, 218, 255) : IM_COL32(130, 135, 145, 255);
}

} // namespace

Toolbox::Toolbox() : m_current(toolGroups().size(), 0) { setActive(ToolId::Select); }

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
    if (io.WantCaptureKeyboard || io.KeyCtrl || io.KeySuper || io.KeyAlt) return;
    for (int i = 0; i < static_cast<int>(ToolId::Count); ++i) {
        const ToolInfo& info = toolInfo(static_cast<ToolId>(i));
        const char k = info.shortcut[0];
        if (k < 'A' || k > 'Z') continue;
        if (!ImGui::IsKeyPressed(static_cast<ImGuiKey>(ImGuiKey_A + (k - 'A')), false)) continue;

        const int g = groupOf(info.id);
        const auto tools = toolGroups()[static_cast<std::size_t>(g)].tools;
        int& cur = m_current[static_cast<std::size_t>(g)];
        if (io.KeyShift && groupOf(m_active) == g) cur = (cur + 1) % static_cast<int>(tools.size());
        setActive(tools[static_cast<std::size_t>(cur)]);
        return;
    }
}

float Toolbox::draw(float top, float bottom) {
    const int cols = m_twoColumns ? 2 : 1;
    const float width = kPad * 2.0f + kSlot * static_cast<float>(cols) + kGap * static_cast<float>(cols - 1);

    const ImGuiViewport* mv = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos({mv->WorkPos.x, top});
    ImGui::SetNextWindowSize({width, bottom - top});
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {kPad, kPad});
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, {kGap, kGap});
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    constexpr ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                                       ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus;
    ImGui::Begin("##toolbox", nullptr, flags);

    // Header: expand / collapse between one and two columns.
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, {0, 0});
    ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_WindowBg));
    if (ImGui::Button(m_twoColumns ? "<<" : ">>", {width - kPad * 2.0f, 14.0f})) m_twoColumns = !m_twoColumns;
    if (ImGui::IsItemHovered()) ImGui::SetTooltip(m_twoColumns ? "Single-column tools" : "Two-column tools");
    ImGui::PopStyleColor();
    ImGui::PopStyleVar();

    const auto groups = toolGroups();
    int col = 0;
    for (std::size_t g = 0; g < groups.size(); ++g) {
        if (g == 0 || groups[g].category != groups[g - 1].category) {
            // Section divider: always starts a new row.
            col = 0;
            ImGui::Dummy({width - kPad * 2.0f, 5.0f});
            const ImVec2 a = ImGui::GetItemRectMin(), b = ImGui::GetItemRectMax();
            const float y = (a.y + b.y) * 0.5f;
            ImGui::GetWindowDrawList()->AddLine({a.x + 2, y}, {b.x - 2, y}, IM_COL32(80, 86, 96, 255), 1.0f);
        }
        if (col > 0) ImGui::SameLine();
        drawSlot(static_cast<int>(g), kSlot);
        col = (col + 1) % cols;
    }

    ImGui::End();
    ImGui::PopStyleVar(3);
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
    if (isActive) dl->AddRectFilled(lo, hi, IM_COL32(52, 92, 150, 255), 4.0f);
    else if (hovered) dl->AddRectFilled(lo, hi, IM_COL32(62, 66, 76, 255), 4.0f);
    drawToolIcon(dl, shown, {(lo.x + hi.x) * 0.5f, (lo.y + hi.y) * 0.5f}, size * 0.56f, iconColor(shown, isActive));

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
        if (info.shortcut[0]) ImGui::Text("%s  (%s)", info.name, info.shortcut);
        else ImGui::TextUnformatted(info.name);
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 22.0f);
        ImGui::TextDisabled("%s", info.description);
        ImGui::PopTextWrapPos();
        if (!info.available) ImGui::TextColored({0.95f, 0.70f, 0.35f, 1.0f}, "Planned for a later milestone");
        if (hasFlyout) ImGui::TextDisabled("Right-click or hold for %zu more", group.tools.size() - 1);
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
    ImGui::TextDisabled("%s", categoryName(group.category));
    ImGui::Separator();

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
        if (current) dl->AddRectFilled({lo.x - 4, lo.y + 6}, {lo.x - 1, lo.y + rowH - 6}, IM_COL32(90, 160, 255, 255));
        drawToolIcon(dl, id, {lo.x + 13, lo.y + rowH * 0.5f}, 18.0f, iconColor(id, false));
        const float ty = lo.y + (rowH - ImGui::GetFontSize()) * 0.5f;
        dl->AddText({lo.x + 32, ty}, info.available ? IM_COL32(225, 228, 234, 255) : IM_COL32(150, 154, 162, 255),
                    info.name);
        if (info.shortcut[0]) dl->AddText({lo.x + rowW - 16, ty}, IM_COL32(140, 145, 155, 255), info.shortcut);
        if (ImGui::IsItemHovered()) {
            ImGui::BeginTooltip();
            ImGui::PushTextWrapPos(ImGui::GetFontSize() * 22.0f);
            ImGui::TextUnformatted(info.description);
            ImGui::PopTextWrapPos();
            if (!info.available) ImGui::TextColored({0.95f, 0.70f, 0.35f, 1.0f}, "Planned for a later milestone");
            ImGui::EndTooltip();
        }
        ImGui::PopID();
    }
    ImGui::EndPopup();
}

} // namespace sicdm

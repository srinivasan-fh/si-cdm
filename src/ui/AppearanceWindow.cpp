// The Appearance window (View menu): the selected body's material picked from swatches, like
// a sample card: each family of materials a row of chips shaded the way the material
// reflects, a finish, and a card of coloured metals (hues by shade). Every click is a
// MATERIAL command, so it is logged and undone like a typed one.

#include "core/Material.hpp"
#include "core/Scene.hpp"
#include "core/Translate.hpp"
#include "ui/LandingUI.hpp"

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace sicdm {

namespace {

ImU32 rgba(Vec3 c, float k = 1.0f) {
    auto ch = [&](float v) { return static_cast<int>(std::clamp(v * k, 0.0f, 1.0f) * 255.0f + 0.5f); };
    return IM_COL32(ch(c.x), ch(c.y), ch(c.z), 255);
}
Vec3 mixc(Vec3 a, Vec3 b, float t) { return a * (1 - t) + b * t; }

// A chip shaded like the material: metals as a polished bar (dark edges, a bright band, a
// coloured body), others as a soft gloss. Returns true when clicked.
bool chip(const char* id, Vec3 colour, float metallic, float roughness, bool selected, float size) {
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const bool clicked = ImGui::InvisibleButton(id, {size, size});
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const float x0 = p.x, x1 = p.x + size, y0 = p.y, y1 = p.y + size;
    if (metallic > 0.5f) {
        const float sharp = 1.0f - 0.7f * roughness;
        const Vec3 dark = colour * (0.30f + 0.25f * roughness), lit = mixc(colour, {1, 1, 1}, 0.25f * sharp);
        // Vertical bands across the chip: dark, light, the colour, a bright line, the colour, dark.
        const float xs[] = {x0, x0 + size * 0.18f, x0 + size * 0.38f, x0 + size * 0.52f, x0 + size * 0.62f, x0 + size * 0.84f, x1};
        const Vec3 cs[] = {dark, lit, colour * 0.85f, mixc(lit, {1, 1, 1}, 0.35f * sharp), colour * 0.85f, colour * 0.6f, dark};
        for (int k = 0; k < 6; ++k)
            dl->AddRectFilledMultiColor({xs[k], y0}, {xs[k + 1], y1}, rgba(cs[k]), rgba(cs[k + 1]), rgba(cs[k + 1]), rgba(cs[k]));
    } else {
        const Vec3 top = mixc(colour, {1, 1, 1}, 0.35f * (1.0f - roughness)), bottom = colour * 0.7f;
        dl->AddRectFilledMultiColor({x0, y0}, {x1, y1}, rgba(top), rgba(top), rgba(bottom), rgba(bottom));
    }
    dl->AddRect({x0, y0}, {x1, y1}, selected ? IM_COL32(90, 170, 255, 255) : IM_COL32(0, 0, 0, 160), 0.0f, 0, selected ? 2.5f : 1.0f);
    return clicked;
}

} // namespace

void LandingUI::drawAppearanceWindow(UiCommands& cmd, UiState& st) {
    ImGui::SetNextWindowSize({560, 0}, ImGuiCond_Appearing);
    if (!ImGui::Begin(tr("Appearance"), &st.showAppearance, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::End();
        return;
    }
    if (!m_menuScene || st.selectedBody < 0 || st.selectedBody >= static_cast<int>(m_menuScene->bodies().size())) {
        ImGui::TextDisabled("%s", tr("Select a body to give it a material"));
        ImGui::End();
        return;
    }
    const Body& b = m_menuScene->bodies()[static_cast<std::size_t>(st.selectedBody)];
    const std::string mq = "MATERIAL \"" + b.name + "\" ";
    ImGui::Text("%s: %s", b.name.c_str(), b.material.preset.empty() ? tr("plastic") : tr(b.material.preset.c_str()));
    const float size = 30.0f;
    const float perRow = 12;
    // Each family a row (or two) of chips.
    for (const char* family : materialFamilies()) {
        ImGui::SeparatorText(tr(family));
        int n = 0;
        for (const MaterialPreset& p : materialPresets()) {
            if (p.family != family) continue;
            if (n % static_cast<int>(perRow) != 0) ImGui::SameLine(0, 4);
            ++n;
            const Vec3 colour = p.setsColor ? p.color : b.color; // "in the body's colour" shows the body's
            if (chip(p.name, colour, p.metallic, p.roughness, b.material.preset == p.name, size)) cmd.commands.push_back(mq + p.name);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s\n%s", tr(p.name), tr(p.description));
        }
    }
    // Coloured metals by hue and shade: anodised aluminium in any colour.
    ImGui::SeparatorText(tr("Anodised colours"));
    static const unsigned kHues[] = {0x808080, 0xE0A020, 0xC06A2A, 0xA02828, 0xD8202A, 0xB01850, 0x9A2AB0, 0x4A2AC8,
                                     0x1E5BD6, 0x1F7AB0, 0x1F9A70, 0x4A9A20, 0x14903A, 0x5A7A5A, 0x7A6040};
    for (int shade = 0; shade < 4; ++shade) {
        for (int h = 0; h < 15; ++h) {
            const unsigned rgb = kHues[h];
            const Vec3 base{static_cast<float>((rgb >> 16) & 255) / 255.0f, static_cast<float>((rgb >> 8) & 255) / 255.0f, static_cast<float>(rgb & 255) / 255.0f};
            // Darker to lighter, as on a colour card.
            const float t = static_cast<float>(shade) / 3.0f;
            const Vec3 c = t < 0.34f ? base * (0.55f + 1.35f * t) : mixc(base, {1, 1, 1}, (t - 0.33f) * 0.75f);
            char hex[16];
            std::snprintf(hex, sizeof hex, "#%02X%02X%02X", static_cast<int>(c.x * 255 + 0.5f), static_cast<int>(c.y * 255 + 0.5f), static_cast<int>(c.z * 255 + 0.5f));
            if (h) ImGui::SameLine(0, 3);
            ImGui::PushID(shade * 16 + h);
            if (chip("##hue", c, 1.0f, 0.3f, false, 24.0f)) cmd.commands.push_back(mq + "anodised COLOR " + hex);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s %s", tr("Anodised"), hex);
            ImGui::PopID();
        }
    }
    // The finish, and the settings themselves.
    ImGui::SeparatorText(tr("Finish"));
    for (const MaterialFinish& f : materialFinishes()) {
        if (&f != &materialFinishes().front()) ImGui::SameLine();
        if (ImGui::RadioButton(tr(f.name), std::fabs(b.material.roughness - f.roughness) < 0.01f)) cmd.commands.push_back(mq + "FINISH " + f.name);
    }
    float rough = b.material.roughness, metal = b.material.metallic;
    ImGui::SetNextItemWidth(220);
    ImGui::SliderFloat(tr("Roughness"), &rough, 0.0f, 1.0f, "%.2f");
    if (ImGui::IsItemDeactivatedAfterEdit()) cmd.commands.push_back(mq + "ROUGHNESS " + std::to_string(rough));
    ImGui::SetNextItemWidth(220);
    ImGui::SliderFloat(tr("Metallic"), &metal, 0.0f, 1.0f, "%.2f");
    if (ImGui::IsItemDeactivatedAfterEdit()) cmd.commands.push_back(mq + "METALLIC " + std::to_string(metal));
    float col[3] = {b.color.x, b.color.y, b.color.z};
    ImGui::SetNextItemWidth(220);
    ImGui::ColorEdit3(tr("Colour"), col, ImGuiColorEditFlags_NoInputs);
    if (ImGui::IsItemDeactivatedAfterEdit()) {
        char hex[16];
        std::snprintf(hex, sizeof hex, "#%02X%02X%02X", static_cast<int>(col[0] * 255 + 0.5f), static_cast<int>(col[1] * 255 + 0.5f), static_cast<int>(col[2] * 255 + 0.5f));
        cmd.commands.push_back(mq + "COLOR " + hex);
    }
    if (ImGui::Button(tr("Default"))) cmd.commands.push_back(mq + "DEFAULT");
    ImGui::End();
}

} // namespace sicdm

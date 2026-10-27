// The Preferences window (File menu) and the Licence window (Help menu). Changes go
// through commands (PREFERENCES SET, LICENCE INSTALL), so they are logged like typed ones.

#include "core/Cam.hpp"
#include "core/Translate.hpp"
#include "core/Lathe.hpp"
#include "core/Licence.hpp"
#include "core/Preferences.hpp"
#include "core/Units.hpp"
#include "ui/LandingUI.hpp"
#include "ui/Theme.hpp"

#include <imgui.h>

#include <cstring>
#include <filesystem>
#include <string>

namespace sicdm {

namespace {

std::string quotedArg(const std::string& s) { return "\"" + s + "\""; }

} // namespace

void LandingUI::drawPreferencesWindow(UiCommands& cmd, UiState& st) {
    ImGui::SetNextWindowSize({520, 0}, ImGuiCond_Appearing);
    if (!ImGui::Begin(tr("Preferences"), &st.showPreferences, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::End();
        return;
    }
    const Preferences& p = preferences();
    auto set = [&](const char* key, const std::string& value) { cmd.commands.push_back(std::string("PREFERENCES SET ") + key + " " + quotedArg(value)); };

    ImGui::SeparatorText(tr("Language"));
    {
        std::string shown = p.language;
        const std::vector<LanguageInfo> langs = languages();
        for (const LanguageInfo& l : langs)
            if (l.code == p.language) shown = l.name;
        ImGui::SetNextItemWidth(260);
        if (ImGui::BeginCombo("##language", shown.c_str())) {
            for (const LanguageInfo& l : langs)
                if (ImGui::Selectable(l.name.c_str(), l.code == p.language)) set("language", l.code);
            ImGui::EndCombo();
        }
        ImGui::SameLine();
        ImGui::TextColored(theme::vec(theme::kTextDim), "%s", tr("More: a language file in the lang folder (see the User Guide)"));
    }

    ImGui::SeparatorText(tr("New projects"));
    {
        const std::string shown = p.units.empty() ? std::string("As the last project") : unitInfo(*parseUnitName(p.units)).name;
        ImGui::SetNextItemWidth(260);
        if (ImGui::BeginCombo(tr("Units"), shown.c_str())) {
            if (ImGui::Selectable(tr("As the last project"), p.units.empty())) set("units", "last");
            for (const LengthUnitInfo& u : lengthUnits())
                if (ImGui::Selectable(u.name, p.units == u.symbol)) set("units", u.symbol);
            ImGui::EndCombo();
        }
    }

    ImGui::SeparatorText(tr("Autosave"));
    {
        bool on = p.autosaveSeconds > 0;
        if (ImGui::Checkbox(tr("Keep a recovery copy of unsaved work"), &on)) set("autosave", on ? "60" : "off");
        ImGui::BeginDisabled(!on);
        static int seconds = 60;
        if (!ImGui::IsAnyItemActive() || !on) seconds = p.autosaveSeconds > 0 ? p.autosaveSeconds : 60;
        ImGui::SetNextItemWidth(260);
        ImGui::SliderInt(tr("After (seconds)"), &seconds, 5, 600);
        if (ImGui::IsItemDeactivatedAfterEdit()) set("autosave", std::to_string(seconds));
        ImGui::EndDisabled();
    }

    ImGui::SeparatorText(tr("CAM and lathe set-ups start with"));
    {
        const CamMaterial* mat = camMaterial(p.camMaterial);
        ImGui::SetNextItemWidth(260);
        if (ImGui::BeginCombo(tr("Material"), mat ? mat->name.c_str() : "Default (aluminium)")) {
            if (ImGui::Selectable(tr("Default (aluminium)"), !mat)) set("material", "default");
            for (const CamMaterial& m : camMaterials())
                if (ImGui::Selectable(m.name.c_str(), mat && m.key == mat->key)) set("material", m.key);
            ImGui::EndCombo();
        }
        ImGui::SetNextItemWidth(260);
        if (ImGui::BeginCombo(tr("Post processor"), p.camPost.empty() ? "Default (fanuc)" : p.camPost.c_str())) {
            if (ImGui::Selectable(tr("Default (fanuc)"), p.camPost.empty())) set("post", "default");
            for (const std::string& q : camPosts())
                if (ImGui::Selectable(q.c_str(), p.camPost == q)) set("post", q);
            ImGui::EndCombo();
        }
        ImGui::SetNextItemWidth(260);
        if (ImGui::BeginCombo(tr("Machine"), p.camMachine.empty() ? "Default (generic)" : p.camMachine.c_str())) {
            if (ImGui::Selectable(tr("Default (generic)"), p.camMachine.empty())) set("machine", "default");
            for (const CamMachine& m : camMachinePresets())
                if (ImGui::Selectable(m.name.c_str(), p.camMachine == m.name)) set("machine", m.name);
            ImGui::EndCombo();
        }
        ImGui::TextColored(theme::vec(theme::kTextDim), "%s", tr("A lathe set-up takes the material, and the post when the lathe has it."));
    }

    ImGui::SeparatorText(tr("Updates"));
    {
        bool on = p.checkUpdates;
        if (ImGui::Checkbox(tr("Check for a newer release when SI CDM starts"), &on)) set("updates", on ? "on" : "off");
        static char url[512] = "";
        if (!ImGui::IsAnyItemActive() || ImGui::IsWindowAppearing()) std::snprintf(url, sizeof(url), "%s", p.updateUrl.c_str());
        ImGui::SetNextItemWidth(360);
        ImGui::InputTextWithHint(tr("Release feed"), "https://.../latest.json", url, sizeof(url));
        if (ImGui::IsItemDeactivatedAfterEdit() && p.updateUrl != url) set("update-url", url[0] ? url : "none");
        ImGui::BeginDisabled(p.updateUrl.empty());
        if (ImGui::Button(tr("Check Now"))) cmd.commands.push_back("UPDATES CHECK");
        ImGui::EndDisabled();
        if (p.updateUrl.empty()) {
            ImGui::SameLine();
            ImGui::TextColored(theme::vec(theme::kTextDim), "%s", tr("Set the release feed your administrator gave you."));
        }
    }

    ImGui::Separator();
    ImGui::TextColored(theme::vec(theme::kTextDim), tr("Saved in %s"), preferencesPath().c_str());
    if (ImGui::Button(tr("Reset to Defaults"))) cmd.commands.push_back("PREFERENCES RESET");
    ImGui::SameLine();
    if (ImGui::Button(tr("Close"))) st.showPreferences = false;
    ImGui::End();
}

void LandingUI::drawLicenceWindow(UiCommands& cmd, UiState& st) {
    if (!ImGui::Begin(tr("Licence"), &st.showLicence, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::End();
        return;
    }
    // Read again every half second at most: it is a small file, and an install changes it.
    static LicenceStatus status;
    static double readAt = -1.0;
    if (ImGui::IsWindowAppearing() || ImGui::GetTime() - readAt > 0.5) {
        status = currentLicence();
        readAt = ImGui::GetTime();
    }
    const ImVec4 good{0.45f, 0.85f, 0.55f, 1}, bad{1.0f, 0.48f, 0.42f, 1};
    switch (status.state) {
    case LicenceState::Valid: ImGui::TextColored(good, "%s", status.describe().c_str()); break;
    case LicenceState::Expired:
    case LicenceState::BadSignature: ImGui::TextColored(bad, "%s", status.describe().c_str()); break;
    default: ImGui::TextUnformatted(status.describe().c_str()); break;
    }
    if (status.state == LicenceState::Valid || status.state == LicenceState::Expired) {
        const Licence& l = status.licence;
        ImGui::Text(tr("Licensee: %s"), l.licensee.c_str());
        if (!l.email.empty()) ImGui::Text(tr("Email:    %s"), l.email.c_str());
        ImGui::Text(tr("Edition:  %s, %d seat%s"), l.edition.c_str(), l.seats, l.seats == 1 ? "" : "s");
        ImGui::Text(tr("Issued:   %s"), l.issued.c_str());
        ImGui::Text(tr("Expires:  %s"), l.expires.empty() ? "never" : l.expires.c_str());
    }
    ImGui::Spacing();
    if (status.state == LicenceState::NoKey)
        ImGui::TextColored(theme::vec(theme::kTextDim), "%s", tr("This build checks no licences: it runs as the Community edition."));
    ImGui::TextColored(theme::vec(theme::kTextDim), tr("Licence file: %s"), licencePath().c_str());
    ImGui::Spacing();
    if (ImGui::Button(tr("Install a Licence File..."))) requestFile(FileLicence, st);
    std::error_code ec;
    const bool installed = std::filesystem::exists(licencePath(), ec);
    ImGui::SameLine();
    ImGui::BeginDisabled(!installed);
    if (ImGui::Button(tr("Remove Licence"))) cmd.commands.push_back("LICENCE REMOVE");
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button(tr("Close"))) st.showLicence = false;
    ImGui::End();
}

} // namespace sicdm

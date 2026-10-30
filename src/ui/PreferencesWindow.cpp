// The Preferences window (File menu) and the Licence window (Help menu). Changes go
// through commands (PREFERENCES SET, LICENCE INSTALL), so they are logged like typed ones.

#include "core/Cam.hpp"
#include "core/Translate.hpp"
#include "core/Lathe.hpp"
#include "core/Licence.hpp"
#include "core/Keymap.hpp"
#include "core/Preferences.hpp"
#include "core/Units.hpp"
#include "ui/LandingUI.hpp"
#include "ui/KeyInput.hpp"
#include "ui/Theme.hpp"
#include "ui/Tools.hpp"

#include <imgui.h>

#include <cctype>
#include <cstring>
#include <filesystem>
#include <string>

namespace sicdm {

namespace {

std::string quotedArg(const std::string& s) { return "\"" + s + "\""; }

} // namespace

// Every action and tool with its keys: click the keys and press new ones (KEYS SET).
void LandingUI::drawKeyboardShortcuts(UiCommands& cmd) {
    const Preferences& p = preferences();
    struct Row {
        std::string id, label, group;
    };
    std::vector<Row> rows;
    for (const KeyAction& a : keyActions()) rows.push_back({a.id, tr(a.label), a.group});
    for (int t = 0; t < static_cast<int>(ToolId::Count); ++t) {
        const ToolInfo& info = toolInfo(static_cast<ToolId>(t));
        if (info.available) rows.push_back({toolKeyId(info.name), std::string(tr(info.name)) + " " + tr("tool"), "Tools"});
    }
    ImGui::SetNextItemWidth(220);
    ImGui::InputTextWithHint("##keyfilter", tr("Find an action or a key"), m_keyFilter, sizeof m_keyFilter);
    ImGui::SameLine();
    if (ImGui::Button(tr("Reset All Shortcuts"))) cmd.commands.push_back("KEYS RESET ALL");
    const std::string filter = [&] {
        std::string f = m_keyFilter;
        for (char& c : f) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        return f;
    }();
    // Setting one: the next keys pressed, Esc and Enter too (a click on its button again: leave
    // it as it was).
    if (!m_keyCapture.empty() && !m_keyShared) {
        if (const auto c = pressedChord()) {
            if (actionsOnChord(p, *c, m_keyCapture).empty()) {
                cmd.commands.push_back("KEYS SET " + m_keyCapture + " " + quotedArg(chordText(*c)));
                m_keyCapture.clear();
            } else {
                m_keyShared = *c; // another action has it: ask first
            }
        }
    }
    setKeyCapture(!m_keyCapture.empty());
    if (ImGui::BeginChild("##keys", {560, 260}, ImGuiChildFlags_Borders)) {
        if (ImGui::BeginTable("##keytable", 3, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit)) {
            ImGui::TableSetupColumn(tr("Action"), ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn(tr("Keys"));
            ImGui::TableSetupColumn("");
            std::string group;
            for (const Row& r : rows) {
                const std::vector<KeyChord> bound = boundChords(p, r.id);
                const std::string keys = bound.empty() ? std::string() : chordsText(bound);
                if (!filter.empty()) {
                    std::string hay = r.label + " " + r.id + " " + keys;
                    for (char& c : hay) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
                    if (hay.find(filter) == std::string::npos) continue;
                }
                if (r.group != group) {
                    group = r.group;
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextColored(theme::vec(theme::kTextDim), "%s", tr(group.c_str()));
                }
                ImGui::PushID(r.id.c_str());
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(r.label.c_str());
                // Another action on the same keys: said here.
                std::string shared;
                for (const KeyChord& c : bound)
                    for (const std::string& o : actionsOnChord(p, c, r.id)) shared += (shared.empty() ? "" : ", ") + o;
                if (!shared.empty()) {
                    ImGui::SameLine();
                    ImGui::TextColored(theme::vec(theme::kError), "(%s %s)", tr("also"), shared.c_str());
                }
                ImGui::TableSetColumnIndex(1);
                const bool setting = m_keyCapture == r.id;
                const std::string label = setting ? std::string(tr("Press the keys... (click: stop)")) : keys.empty() ? std::string("-") : keys;
                if (ImGui::Button((label + "##set").c_str(), {170, 0})) m_keyCapture = setting ? std::string() : r.id, m_keyShared.reset();
                if (ImGui::IsItemHovered() && !setting) ImGui::SetTooltip("%s", tr("Click, then press the new keys (with Ctrl, Shift, Alt as you like)"));
                ImGui::TableSetColumnIndex(2);
                if (p.keys.count(r.id)) {
                    if (ImGui::SmallButton(tr("Default"))) cmd.commands.push_back("KEYS RESET " + r.id);
                } else if (!keys.empty()) {
                    if (ImGui::SmallButton(tr("Clear"))) cmd.commands.push_back("KEYS SET " + r.id + " none");
                }
                ImGui::PopID();
            }
            ImGui::EndTable();
        }
    }
    ImGui::EndChild();
    if (m_keyShared) {
        const std::vector<std::string> others = actionsOnChord(p, *m_keyShared, m_keyCapture);
        ImGui::TextColored(theme::vec(theme::kError), tr("%s is %s's already."), chordText(*m_keyShared).c_str(), others.empty() ? "?" : others[0].c_str());
        ImGui::SameLine();
        if (ImGui::SmallButton(tr("Use for Both"))) {
            cmd.commands.push_back("KEYS SET " + m_keyCapture + " " + quotedArg(chordText(*m_keyShared)) + " FORCE");
            m_keyCapture.clear(), m_keyShared.reset();
        }
        ImGui::SameLine();
        if (ImGui::SmallButton(tr("Cancel"))) m_keyCapture.clear(), m_keyShared.reset();
    }
}

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

    ImGui::SeparatorText(tr("Keyboard Shortcuts"));
    drawKeyboardShortcuts(cmd);

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

// The Product data window (File menu): the project's and the selected part's properties, the
// revision and its state, and the team's vault. Changes go through commands (PROPERTY,
// REVISION, VAULT), so they are logged and undone like typed ones.

#include "core/FileIO.hpp"
#include "core/Pdm.hpp"
#include "core/PdmCommand.hpp"
#include "core/Scene.hpp"
#include "core/Translate.hpp"
#include "ui/LandingUI.hpp"

#include <imgui.h>

#include <cstring>
#include <string>

namespace sicdm {

namespace {

std::string quoted(const std::string& s) { return "\"" + s + "\""; }

// One property per row: its value edited in place (Enter sets it), and a button to remove it.
void propertyRows(UiCommands& cmd, const std::map<std::string, std::string>& props, const std::string& target, bool locked, const char* id) {
    if (props.empty()) ImGui::TextDisabled("%s", tr("No properties"));
    if (!ImGui::BeginTable(id, 3, ImGuiTableFlags_SizingFixedFit)) return;
    for (const auto& [key, value] : props) {
        ImGui::PushID(key.c_str());
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TextUnformatted(key.c_str());
        ImGui::TableNextColumn();
        char buf[256];
        std::snprintf(buf, sizeof buf, "%s", value.c_str());
        ImGui::SetNextItemWidth(260);
        ImGui::BeginDisabled(locked);
        if (ImGui::InputText("##v", buf, sizeof buf, ImGuiInputTextFlags_EnterReturnsTrue) && std::string(buf) != value && buf[0])
            cmd.commands.push_back("PROPERTY " + target + key + " " + buf);
        ImGui::TableNextColumn();
        if (ImGui::SmallButton("x")) cmd.commands.push_back("PROPERTY " + target + "DELETE " + key);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tr("Remove this property"));
        ImGui::EndDisabled();
        ImGui::PopID();
    }
    ImGui::EndTable();
}

} // namespace

void LandingUI::drawProductWindow(UiCommands& cmd, UiState& st) {
    ImGui::SetNextWindowSize({560, 0}, ImGuiCond_Appearing);
    if (!ImGui::Begin(tr("Product Data"), &st.showProduct, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::End();
        return;
    }
    const ProductData pd = st.project ? st.project->product : ProductData{};
    const bool locked = pd.locked();

    // The revision and its state.
    ImGui::SeparatorText(tr("Revision"));
    ImGui::Text("%s %s  -  %s", tr("Revision"), pd.revision.c_str(), tr(releaseStateName(pd.state)));
    if (locked) {
        ImGui::SameLine();
        ImGui::TextDisabled("%s", tr("(can't be changed)"));
    }
    static char note[200] = "";
    ImGui::SetNextItemWidth(320);
    ImGui::InputTextWithHint("##note", tr("Note (why)"), note, sizeof note);
    const std::string withNote = note[0] ? std::string(" NOTE ") + note : std::string();
    auto revision = [&](const std::string& words) {
        cmd.commands.push_back("REVISION " + words + withNote);
        note[0] = 0;
    };
    if (pd.state == ReleaseState::InWork && ImGui::Button(tr("Submit for Review"))) revision("REVIEW");
    if (pd.state == ReleaseState::InReview) {
        if (ImGui::Button(tr("Back to Work"))) revision("STATE in work");
        ImGui::SameLine();
    }
    if ((pd.state == ReleaseState::InWork || pd.state == ReleaseState::InReview)) {
        if (pd.state == ReleaseState::InWork) ImGui::SameLine();
        if (ImGui::Button(tr("Release"))) revision("RELEASE");
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tr("A released revision can't be changed: the next revision can"));
    }
    if (pd.state == ReleaseState::Released) {
        if (ImGui::Button(tr("Make Obsolete"))) revision("STATE obsolete");
        ImGui::SameLine();
    }
    if (locked || pd.state != ReleaseState::InWork) {
        if (ImGui::Button(tr("Start Next Revision"))) revision("NEXT");
    }
    if (!pd.history.empty() && ImGui::TreeNode(tr("History"))) {
        for (const RevisionRecord& r : pd.history)
            ImGui::BulletText("%s  %s  %s  %s%s%s", r.revision.c_str(), tr(releaseStateName(r.state)), r.date.c_str(), r.author.c_str(), r.note.empty() ? "" : "  ",
                              r.note.c_str());
        ImGui::TreePop();
    }

    // Properties: the project's and the selected part's.
    ImGui::SeparatorText(tr("Project Properties"));
    propertyRows(cmd, pd.properties, "PROJECT ", locked, "##project");
    static char key[64] = "", value[256] = "";
    auto addRow = [&](const std::string& target) {
        ImGui::BeginDisabled(locked);
        ImGui::SetNextItemWidth(120);
        ImGui::InputTextWithHint("##k", tr("NUMBER"), key, sizeof key);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(220);
        ImGui::InputTextWithHint("##val", tr("Value"), value, sizeof value);
        ImGui::SameLine();
        if (ImGui::Button(tr("Add")) && key[0] && value[0] && !std::strpbrk(key, " =\"")) {
            cmd.commands.push_back("PROPERTY " + target + key + " " + value);
            key[0] = value[0] = 0;
        }
        ImGui::EndDisabled();
    };
    ImGui::PushID("projectAdd");
    addRow("PROJECT ");
    ImGui::PopID();
    if (m_menuScene && st.selectedBody >= 0 && st.selectedBody < static_cast<int>(m_menuScene->bodies().size())) {
        const Body& b = m_menuScene->bodies()[static_cast<std::size_t>(st.selectedBody)];
        ImGui::SeparatorText((std::string(tr("Part")) + ": " + b.name).c_str());
        propertyRows(cmd, b.properties, quoted(b.name) + " ", locked, "##part");
        ImGui::PushID("partAdd");
        addRow(quoted(b.name) + " ");
        ImGui::PopID();
    }

    // The vault.
    ImGui::SeparatorText(tr("Vault"));
    const std::string folder = vaultFolder();
    static char folderBuf[512] = "";
    if (ImGui::IsWindowAppearing()) std::snprintf(folderBuf, sizeof folderBuf, "%s", folder.c_str());
    ImGui::SetNextItemWidth(380);
    if (ImGui::InputTextWithHint("##vault", tr("Shared folder (a network drive, a synced folder)"), folderBuf, sizeof folderBuf,
                                 ImGuiInputTextFlags_EnterReturnsTrue) &&
        folderBuf[0])
        cmd.commands.push_back("VAULT FOLDER " + quoted(folderBuf));
    ImGui::SameLine();
    if (ImGui::Button(tr("Refresh")) || ImGui::IsWindowAppearing() || m_vaultDirty) {
        m_vaultItems = vaultItems(folder, &m_vaultWhy);
        m_vaultDirty = false;
    }
    if (folder.empty()) {
        ImGui::TextDisabled("%s", tr("No vault: set a shared folder to check projects in and out"));
    } else {
        if (!m_vaultWhy.empty() && m_vaultItems.empty()) ImGui::TextDisabled("%s", m_vaultWhy.c_str());
        if (ImGui::BeginTable("##items", 5, ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_ScrollY, {0, 180})) {
            ImGui::TableSetupColumn(tr("Item"));
            ImGui::TableSetupColumn(tr("Rev"));
            ImGui::TableSetupColumn(tr("State"));
            ImGui::TableSetupColumn(tr("Checked out by"));
            ImGui::TableSetupColumn("");
            ImGui::TableHeadersRow();
            for (const VaultItem& it : m_vaultItems) {
                ImGui::PushID(it.key.c_str());
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(it.key.c_str());
                if (!it.description.empty() && ImGui::IsItemHovered()) ImGui::SetTooltip("%s", it.description.c_str());
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(it.revisions.empty() ? "-" : it.revisions.back().revision.c_str());
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(it.revisions.empty() ? "" : tr(releaseStateName(it.revisions.back().state)));
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(it.lockedBy.c_str());
                ImGui::TableNextColumn();
                if (it.lockedBy.empty() && ImGui::SmallButton(tr("Check Out"))) {
                    cmd.commands.push_back("VAULT CHECKOUT " + it.key);
                    m_vaultDirty = true;
                }
                if (it.lockedBy == currentUser() && ImGui::SmallButton(tr("Undo Check Out"))) {
                    cmd.commands.push_back("VAULT UNDO " + it.key);
                    m_vaultDirty = true;
                }
                ImGui::SameLine();
                if (ImGui::SmallButton(tr("Get Copy"))) cmd.commands.push_back("VAULT GET " + it.key);
                ImGui::PopID();
            }
            ImGui::EndTable();
        }
        if (ImGui::Button(tr("Check In"))) {
            cmd.commands.push_back("VAULT CHECKIN" + withNote);
            note[0] = 0;
            m_vaultDirty = true;
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tr("Saves the project and puts this revision in the vault (the note above goes with it)"));
        if (!pd.vaultItem.empty()) {
            ImGui::SameLine();
            ImGui::TextDisabled("%s %s", tr("Vault item:"), pd.vaultItem.c_str());
        }
    }
    ImGui::End();
}

} // namespace sicdm

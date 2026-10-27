// The user guide window: the built-in guides (docs/*.md) listed and searchable on the left,
// the picked one on the right, with links between them; and the sample projects.

#include "core/HelpDocs.hpp"
#include "core/Translate.hpp"
#include "ui/LandingUI.hpp"
#include "ui/Theme.hpp"

#include <imgui.h>

#include <algorithm>
#include <cctype>
#include <string>

namespace sicdm {

namespace {

std::string lowered(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

// A heading's anchor, as GitHub makes them: "The CAM panel" -> "the-cam-panel".
std::string slug(const std::string& heading) {
    std::string out;
    for (char c : lowered(markdownPlain(heading))) {
        if (std::isalnum(static_cast<unsigned char>(c)) || c == '-' || c == '_') out += c;
        else if (c == ' ') out += '-';
    }
    return out;
}

std::size_t countOf(const std::string& hay, const std::string& needle) {
    if (needle.empty()) return 0;
    std::size_t n = 0;
    for (std::size_t p = hay.find(needle); p != std::string::npos; p = hay.find(needle, p + needle.size())) ++n;
    return n;
}

} // namespace

void LandingUI::runSample(UiCommands& cmd, const std::string& file) {
    for (const HelpExample& e : helpExamples()) {
        if (e.file != file) continue;
        // Each sample builds a project of its own.
        bool fresh = false;
        std::size_t start = 0;
        while (start < e.text.size()) {
            std::size_t end = e.text.find('\n', start);
            if (end == std::string::npos) end = e.text.size();
            std::string line = e.text.substr(start, end - start);
            start = end + 1;
            while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.pop_back();
            const std::size_t first = line.find_first_not_of(' ');
            if (first == std::string::npos || line[first] == '#') continue;
            line = line.substr(first);
            if (!fresh) {
                if (lowered(line) != "new") cmd.commands.push_back("NEW");
                fresh = true;
            }
            cmd.commands.push_back(line);
        }
    }
}

void LandingUI::drawHelpWindow(UiCommands& cmd, UiState& st) {
    ImGui::SetNextWindowSize({1000, 720}, ImGuiCond_FirstUseEver);
    if (!ImGui::Begin(tr("User Guide"), &st.showHelp)) {
        ImGui::End();
        return;
    }
    // The page asked for: a file and maybe a heading.
    {
        const std::size_t hash = st.helpPage.find('#');
        const std::string file = st.helpPage.substr(0, hash);
        if (file != m_helpShown && helpDoc(file)) {
            if (!m_helpShown.empty()) m_helpBack.push_back(m_helpShown);
            m_helpShown = file;
            m_helpAnchor = hash == std::string::npos ? "#top" : st.helpPage.substr(hash + 1);
        } else if (hash != std::string::npos && file == m_helpShown) {
            m_helpAnchor = st.helpPage.substr(hash + 1);
        }
        if (!helpDoc(m_helpShown) && !helpDocs().empty()) m_helpShown = helpDocs().front().file, m_helpAnchor = "#top";
        st.helpPage = m_helpShown;
    }
    auto go = [&](const std::string& target) { st.helpPage = target; };

    // Left: search and the guides, then the samples.
    ImGui::BeginChild("##helpnav", {240, 0}, ImGuiChildFlags_Borders);
    ImGui::SetNextItemWidth(-1);
    ImGui::InputTextWithHint("##helpsearch", "Search the guide", m_helpSearch, sizeof m_helpSearch);
    const std::string q = lowered(m_helpSearch);
    ImGui::BeginDisabled(m_helpBack.empty());
    if (ImGui::SmallButton(tr("< Back"))) {
        const std::string back = m_helpBack.back();
        m_helpBack.pop_back();
        m_helpShown = back;
        st.helpPage = back;
        m_helpAnchor = "#top";
    }
    ImGui::EndDisabled();
    ImGui::Spacing();
    for (const HelpDoc& d : helpDocs()) {
        std::size_t hits = 0;
        if (!q.empty()) {
            hits = countOf(lowered(d.text), q);
            if (hits == 0) continue;
        }
        const std::string label = q.empty() ? d.title : d.title + "  (" + std::to_string(hits) + ")";
        if (ImGui::Selectable((label + "##" + d.file).c_str(), d.file == m_helpShown)) go(d.file);
    }
    if (!q.empty()) ImGui::TextDisabled(tr("Pages with \"%s\""), m_helpSearch);
    ImGui::Spacing();
    ImGui::SeparatorText(tr("Sample projects"));
    for (const HelpExample& e : helpExamples()) {
        ImGui::PushID(e.file.c_str());
        if (ImGui::Selectable(e.title.c_str())) runSample(cmd, e.file);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip(tr("Builds it in a new tab (%s)"), e.file.c_str());
        ImGui::PopID();
    }
    ImGui::EndChild();
    ImGui::SameLine();

    // Right: the page.
    ImGui::BeginChild("##helppage", {0, 0}, ImGuiChildFlags_None, ImGuiWindowFlags_HorizontalScrollbar);
    const HelpDoc* doc = helpDoc(m_helpShown);
    if (!doc) {
        ImGui::TextDisabled("%s", tr("The guide isn't built into this copy."));
        ImGui::EndChild();
        ImGui::End();
        return;
    }
    if (m_helpAnchor == "#top") {
        ImGui::SetScrollY(0);
        m_helpAnchor.clear();
    }
    const float wrap = std::min(ImGui::GetContentRegionAvail().x, 760.0f);
    ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + wrap);
    // Links to other pages (or headings), as buttons after the block that has them.
    auto links = [&](const MdBlock& b) {
        bool any = false;
        for (const auto& [text, target] : b.links) {
            const bool web = target.rfind("http", 0) == 0;
            const std::string file = target.substr(0, target.find('#'));
            if (!web && !file.empty() && !helpDoc(file)) continue; // a file outside the guide
            if (any) ImGui::SameLine();
            any = true;
            ImGui::PushID(&text);
            if (web) {
                ImGui::TextColored(theme::vec(theme::kAccentText), "%s", target.c_str());
            } else if (ImGui::SmallButton(("-> " + text).c_str())) {
                go(file.empty() ? m_helpShown + target : target);
            }
            ImGui::PopID();
        }
    };
    int item = 0;
    for (const MdBlock& b : parseMarkdown(doc->text)) {
        ImGui::PushID(item++);
        switch (b.kind) {
        case MdBlock::Heading: {
            if (b.level > 1) ImGui::Spacing();
            theme::ScopedFont f(b.level <= 2 ? theme::Font::Strong : theme::Font::Body);
            if (b.level == 1) ImGui::SetWindowFontScale(1.35f);
            else if (b.level == 2) ImGui::SetWindowFontScale(1.15f);
            ImGui::TextUnformatted(markdownPlain(b.text).c_str());
            ImGui::SetWindowFontScale(1.0f);
            if (b.level <= 2) ImGui::Separator();
            if (!m_helpAnchor.empty() && slug(b.text) == m_helpAnchor) {
                ImGui::SetScrollHereY(0.0f);
                m_helpAnchor.clear();
            }
            break;
        }
        case MdBlock::Paragraph:
            ImGui::TextWrapped("%s", markdownPlain(b.text).c_str());
            links(b);
            ImGui::Spacing();
            break;
        case MdBlock::Bullet:
        case MdBlock::Numbered: {
            const float ind = 14.0f * static_cast<float>(b.level + 1);
            ImGui::Indent(ind);
            if (b.kind == MdBlock::Bullet) ImGui::Bullet();
            ImGui::TextWrapped("%s", markdownPlain(b.text).c_str());
            links(b);
            ImGui::Unindent(ind);
            break;
        }
        case MdBlock::Quote:
            ImGui::Indent(10);
            ImGui::PushStyleColor(ImGuiCol_Text, theme::vec(theme::kWarning));
            ImGui::TextWrapped("%s", markdownPlain(b.text).c_str());
            ImGui::PopStyleColor();
            ImGui::Unindent(10);
            ImGui::Spacing();
            break;
        case MdBlock::Code: {
            theme::ScopedFont f(theme::Font::Mono);
            ImGui::PushStyleColor(ImGuiCol_ChildBg, theme::vec(theme::kField));
            ImGui::BeginChild("##code", {wrap, 0}, ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_HorizontalScrollbar);
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextUnformatted(b.text.c_str());
            ImGui::PopTextWrapPos();
            ImGui::EndChild();
            ImGui::PopStyleColor();
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tr("Right-click to copy"));
            if (ImGui::BeginPopupContextItem("##copy")) {
                if (ImGui::MenuItem(tr("Copy"))) ImGui::SetClipboardText(b.text.c_str());
                ImGui::EndPopup();
            }
            ImGui::Spacing();
            break;
        }
        case MdBlock::Table: {
            const int cols = b.rows.empty() ? 0 : static_cast<int>(b.rows[0].size());
            if (cols > 0 && ImGui::BeginTable("##table", cols, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp, {wrap, 0})) {
                for (std::size_t r = 0; r < b.rows.size(); ++r) {
                    ImGui::TableNextRow(r == 0 ? ImGuiTableRowFlags_Headers : ImGuiTableRowFlags_None);
                    for (int c = 0; c < cols; ++c) {
                        ImGui::TableSetColumnIndex(c);
                        const std::string cell = c < static_cast<int>(b.rows[r].size()) ? markdownPlain(b.rows[r][static_cast<std::size_t>(c)]) : "";
                        if (r == 0) {
                            theme::ScopedFont f(theme::Font::Strong);
                            ImGui::TextWrapped("%s", cell.c_str());
                        } else ImGui::TextWrapped("%s", cell.c_str());
                    }
                }
                ImGui::EndTable();
            }
            links(b);
            ImGui::Spacing();
            break;
        }
        case MdBlock::Image:
            ImGui::TextColored(theme::vec(theme::kTextDim), tr("[Picture: %s]"), markdownPlain(b.text).c_str());
            ImGui::Spacing();
            break;
        case MdBlock::Rule: ImGui::Separator(); break;
        }
        ImGui::PopID();
    }
    ImGui::PopTextWrapPos();
    ImGui::EndChild();
    ImGui::End();
}

} // namespace sicdm

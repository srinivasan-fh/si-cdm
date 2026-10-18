#include "ui/FileDialog.hpp"

#include <imgui.h>

#include <algorithm>
#include <cctype>
#include <cstring>
#include <filesystem>

namespace sicdm {

namespace fs = std::filesystem;

namespace {

std::string lowerExt(const std::string& name) {
    std::string e = fs::path(name).extension().string();
    for (char& c : e) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return e;
}

void copyTo(char* dst, std::size_t n, const std::string& s) {
    std::strncpy(dst, s.c_str(), n - 1);
    dst[n - 1] = '\0';
}

} // namespace

void FileDialog::open(Mode mode, std::string title, std::vector<std::string> extensions, std::string startDir,
                      std::string defaultName) {
    m_mode = mode;
    m_title = std::move(title);
    m_extensions = std::move(extensions);
    std::error_code ec;
    if (startDir.empty() || !fs::is_directory(startDir, ec)) startDir = fs::current_path(ec).string();
    m_dir = fs::absolute(startDir, ec).lexically_normal().string();
    copyTo(m_name, sizeof(m_name), defaultName);
    m_error.clear();
    m_confirmOverwrite = false;
    m_open = m_justOpened = m_focusName = true;
    refresh();
}

bool FileDialog::accepts(const std::string& filename) const {
    if (m_extensions.empty()) return true;
    const std::string e = lowerExt(filename);
    return std::find(m_extensions.begin(), m_extensions.end(), e) != m_extensions.end();
}

void FileDialog::refresh() {
    m_dirs.clear();
    m_files.clear();
    copyTo(m_dirBuf, sizeof(m_dirBuf), m_dir);
    std::error_code ec;
    for (fs::directory_iterator it(m_dir, fs::directory_options::skip_permission_denied, ec), end; !ec && it != end;
         it.increment(ec)) {
        const std::string name = it->path().filename().string();
        if (name.empty() || name[0] == '.') continue; // hidden files
        std::error_code e2;
        if (it->is_directory(e2)) m_dirs.push_back(name);
        else if (accepts(name)) m_files.push_back(name);
    }
    if (ec) m_error = "Cannot read folder: " + ec.message();
    auto ci = [](const std::string& a, const std::string& b) {
        return std::lexicographical_compare(a.begin(), a.end(), b.begin(), b.end(), [](char x, char y) {
            return std::tolower(static_cast<unsigned char>(x)) < std::tolower(static_cast<unsigned char>(y));
        });
    };
    std::sort(m_dirs.begin(), m_dirs.end(), ci);
    std::sort(m_files.begin(), m_files.end(), ci);
}

std::optional<std::string> FileDialog::draw() {
    if (!m_open) return std::nullopt;
    // "###" makes the ID independent of the visible title, so OpenPopup and
    // BeginPopupModal below refer to the same popup.
    const char* id = "###filedialog";
    if (m_justOpened) {
        ImGui::OpenPopup(id);
        m_justOpened = false;
    }
    std::optional<std::string> result;
    ImGui::SetNextWindowSize({680, 460}, ImGuiCond_Appearing);
    const ImGuiViewport* mv = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos({mv->GetCenter().x, mv->GetCenter().y}, ImGuiCond_Appearing, {0.5f, 0.5f});
    bool keepOpen = true;
    const std::string label = m_title + id;
    if (!ImGui::BeginPopupModal(label.c_str(), &keepOpen, ImGuiWindowFlags_NoSavedSettings)) {
        m_open = ImGui::IsPopupOpen(id);
        if (!keepOpen) m_open = false;
        return std::nullopt;
    }

    // Folder bar.
    if (ImGui::Button("Up")) {
        m_dir = fs::path(m_dir).parent_path().string();
        refresh();
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(-1);
    if (ImGui::InputText("##dir", m_dirBuf, sizeof(m_dirBuf), ImGuiInputTextFlags_EnterReturnsTrue)) {
        std::error_code ec;
        if (fs::is_directory(m_dirBuf, ec)) {
            m_dir = m_dirBuf;
            m_error.clear();
            refresh();
        } else {
            m_error = "Not a folder";
        }
    }

    // Listing.
    const float footer = ImGui::GetFrameHeightWithSpacing() * 2.6f;
    if (ImGui::BeginChild("##list", {0, -footer}, ImGuiChildFlags_Borders)) {
        for (const std::string& d : m_dirs) {
            const std::string row = "[dir]  " + d;
            if (ImGui::Selectable(row.c_str(), false, ImGuiSelectableFlags_AllowDoubleClick) &&
                ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                m_dir = (fs::path(m_dir) / d).string();
                refresh();
                break;
            }
        }
        for (const std::string& f : m_files) {
            const bool sel = f == m_name;
            if (ImGui::Selectable(f.c_str(), sel, ImGuiSelectableFlags_AllowDoubleClick)) {
                copyTo(m_name, sizeof(m_name), f);
                m_confirmOverwrite = false;
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left) && m_mode == Mode::Open)
                    result = (fs::path(m_dir) / f).string();
            }
        }
        if (m_dirs.empty() && m_files.empty()) ImGui::TextDisabled("(empty)");
    }
    ImGui::EndChild();

    // File name + actions.
    std::string filter;
    for (const std::string& e : m_extensions) filter += (filter.empty() ? "" : " ") + e;
    ImGui::SetNextItemWidth(-240);
    if (m_focusName) {
        ImGui::SetKeyboardFocusHere();
        m_focusName = false;
    }
    const bool enter = ImGui::InputTextWithHint("##name", "file name", m_name, sizeof(m_name),
                                                ImGuiInputTextFlags_EnterReturnsTrue);
    ImGui::SameLine();
    ImGui::TextDisabled("%s", filter.empty() ? "All files" : filter.c_str());
    if (!m_error.empty()) ImGui::TextColored({0.95f, 0.45f, 0.4f, 1}, "%s", m_error.c_str());
    else if (m_confirmOverwrite) ImGui::TextColored({0.95f, 0.70f, 0.35f, 1}, "File exists. Click Save again to replace it.");
    else ImGui::TextDisabled("%s", m_dir.c_str());

    ImGui::SetCursorPosX(ImGui::GetWindowWidth() - 200);
    const bool ok = ImGui::Button(m_mode == Mode::Open ? "Open" : "Save", {90, 0});
    ImGui::SameLine();
    const bool cancel = ImGui::Button("Cancel", {90, 0}) || ImGui::IsKeyPressed(ImGuiKey_Escape);

    if ((ok || enter) && !result) {
        std::string name = m_name;
        if (name.empty()) {
            m_error = "Enter a file name";
        } else {
            fs::path p = fs::path(name).is_absolute() ? fs::path(name) : fs::path(m_dir) / name;
            if (m_mode == Mode::Save && !m_extensions.empty() && !accepts(p.filename().string()))
                p += m_extensions.front();
            std::error_code ec;
            if (m_mode == Mode::Open && !fs::is_regular_file(p, ec)) {
                m_error = "File not found";
            } else if (m_mode == Mode::Save && fs::exists(p, ec) && !m_confirmOverwrite) {
                m_confirmOverwrite = true;
                m_error.clear();
            } else {
                result = p.string();
            }
        }
    }
    if (result || cancel || !keepOpen) {
        m_open = false;
        ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
    return result;
}

} // namespace sicdm

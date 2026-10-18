#pragma once

#include <optional>
#include <string>
#include <vector>

namespace sicdm {

// Modal file browser (open or save) drawn with Dear ImGui.
class FileDialog {
public:
    enum class Mode { Open, Save };

    // `extensions` like {".sicdm"}; empty accepts every file.
    void open(Mode mode, std::string title, std::vector<std::string> extensions, std::string startDir,
              std::string defaultName = {});
    bool isOpen() const noexcept { return m_open; }

    // Call every frame; returns the chosen path once.
    std::optional<std::string> draw();

private:
    void refresh();
    bool accepts(const std::string& filename) const;

    Mode m_mode = Mode::Open;
    std::string m_title;
    std::vector<std::string> m_extensions;
    std::string m_dir;
    char m_name[512] = "";
    char m_dirBuf[1024] = "";
    std::vector<std::string> m_dirs, m_files;
    std::string m_error;
    bool m_open = false;
    bool m_justOpened = false;
    bool m_focusName = false;
    bool m_confirmOverwrite = false;
};

} // namespace sicdm

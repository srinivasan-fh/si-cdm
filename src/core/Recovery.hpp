#pragma once

// Autosave and crash recovery.
//
// While a document has unsaved changes, a copy of it is kept in the recovery folder,
// written once the changes are a little while old and again after further changes. A
// clean save, close or exit removes the copy. A copy left by a session that has ended
// (a crash, a power cut, a killed process) is offered back the next time: see RECOVER.

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace sicdm {

struct Document;

// $SICDM_RECOVERY_DIR, else the per-user state folder: %LOCALAPPDATA%\SI-CDM\Recovery on
// Windows, ~/Library/Application Support/SI-CDM/Recovery on macOS, and
// $XDG_STATE_HOME/sicdm/recovery (~/.local/state/sicdm/recovery) elsewhere.
std::string recoveryDirectory();

// A copy another session left behind.
struct RecoveryFile {
    std::string project;  // the copy (a project file)
    std::string meta;     // its description
    std::string original; // the file it was a copy of; empty if never saved
    std::string name;     // the document's name
    std::int64_t savedAt = 0; // seconds since 1970 (UTC)
};

// Copies whose sessions are over, newest first.
std::vector<RecoveryFile> findRecoveryFiles(const std::string& directory = recoveryDirectory());
// Removes a copy.
void discardRecoveryFile(const RecoveryFile& f);
// Takes a copy over for this session (it is no longer offered while this session runs,
// and goes when it exits cleanly; after another crash it is offered again).
void adoptRecoveryFile(const RecoveryFile& f, const std::string& directory = recoveryDirectory());

class Autosaver {
public:
    explicit Autosaver(std::string directory = recoveryDirectory(), double intervalSeconds = 60.0);

    // Call often. `id` tells documents apart; `signature` changes whenever the content
    // does; `now` is in seconds (any steady clock). Returns true when it wrote a copy.
    bool update(const Document& doc, int id, std::uint64_t signature, double now, std::string* error = nullptr);
    void forget(int id); // the document closed: remove its copy
    void clear();        // a clean exit: remove every copy this session made

    std::string projectPath(int id) const;
    // When the next copy falls due (in update()'s seconds), or -1 when none is waiting: an
    // idle app should wake by then.
    double nextDue() const;
    double interval() const { return m_interval; }
    void setInterval(double seconds) { m_interval = seconds; }

private:
    struct Entry {
        std::uint64_t written = 0; // signature of the copy on disk (0: none)
        double pendingSince = -1;  // when changes after it were first seen
    };
    std::string m_dir;
    double m_interval;
    std::map<int, Entry> m_entries;
};

} // namespace sicdm

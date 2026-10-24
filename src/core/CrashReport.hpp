#pragma once

// Crash reports, kept on this computer.
//
// When the program dies on a fault (a bad memory access, an abort, an uncaught exception,
// a floating-point trap), a short report is written to the crash folder: what happened,
// the version, the platform, the last command it ran and the call stack. Nothing is sent
// anywhere. The next session offers the reports it hasn't shown yet (CRASHES lists them).

#include <cstdint>
#include <string>
#include <vector>

namespace sicdm {

// $SICDM_CRASH_DIR, else beside the recovery folder: %LOCALAPPDATA%\SI-CDM\Crashes on
// Windows, ~/Library/Application Support/SI-CDM/Crashes on macOS, and
// $XDG_STATE_HOME/sicdm/crashes (~/.local/state/sicdm/crashes) elsewhere.
std::string crashDirectory();

// Installs the handlers (once; later calls only update the version). Reports go to `dir`
// (default: crashDirectory()).
void installCrashHandler(const std::string& version, const std::string& dir = std::string());

// What the program is doing, written into a report if it dies now (e.g. the command running).
void setCrashContext(const std::string& what);

struct CrashReportFile {
    std::string path;
    std::int64_t time = 0; // seconds since 1970
    bool seen = false;     // already offered to the user
};

// The reports in the folder, newest first.
std::vector<CrashReportFile> crashReports(const std::string& dir = crashDirectory());
// Marks the unseen ones seen (they stay in the folder, up to the newest 20).
void acknowledgeCrashReports(const std::string& dir = crashDirectory());
// Removes them all.
void clearCrashReports(const std::string& dir = crashDirectory());

} // namespace sicdm

#pragma once

// Preferences: what a user sets once for every project, kept in preferences.json in the
// per-user configuration folder. The recent files list lives here too.

#include <map>
#include <string>
#include <vector>

namespace sicdm {

class Json;

struct Preferences {
    std::vector<std::string> recent; // project files, newest first (at most kMaxRecent)
    std::string units;               // for new projects ("mm", "in", ...); empty: as the last project
    int autosaveSeconds = 60;        // how old unsaved changes get before a recovery copy; 0: off
    std::string camMaterial;         // CAM and lathe set-ups start with these; empty: the built-in default
    std::string camPost;
    std::string camMachine;          // a machine preset's name
    bool checkUpdates = false;       // look for a newer release when the app starts
    bool rulers = true;              // rulers along the canvas's top and left edges
    bool reopenLast = true;          // start with the projects open when the app last closed
    std::vector<std::string> session; // those projects (the work space), in tab order
    int sessionActive = 0;           // the tab that was in front, an index into `session`
    bool sessionOpening = false;     // set while they are being reopened: a crash then skips it next time
    std::string updateUrl;           // where the latest release is described (see UpdateCheck.hpp)
    std::string language = "en";     // the interface's language (see Translate.hpp)
    std::string vault;               // the team's vault folder (VAULT; Pdm.hpp); empty: $SICDM_VAULT, else none
    std::map<std::string, std::string> keys; // keyboard shortcuts changed from their defaults: action -> keys (Keymap.hpp)

    static constexpr std::size_t kMaxRecent = 10;
    static constexpr std::size_t kMaxSession = 32;
};

// What to open at start-up (with REOPEN on and no file given): the projects open when the
// app last closed that are still there, else the most recent project. When the last start
// closed while opening them, nothing (a broken file would close it every time).
struct StartupSession {
    std::vector<std::string> files;
    int active = 0;              // which of `files` goes in front
    bool crashedLastTime = false;
};
StartupSession startupSession(const Preferences& p);

Json preferencesToJson(const Preferences& p);
Preferences preferencesFromJson(const Json& j); // unknown or bad fields keep their defaults

// %APPDATA%\SI-CDM on Windows, ~/Library/Application Support/SI-CDM on macOS and
// $XDG_CONFIG_HOME/sicdm (~/.config/sicdm) elsewhere.
std::string configDirectory();
// $SICDM_PREFERENCES, else preferences.json in configDirectory().
std::string preferencesPath();

// The preferences of this process, read from preferencesPath() on first use.
Preferences& preferences();
// Writes preferences() back; false (and why) when it can't.
bool savePreferences(std::string* error = nullptr);
// Reads preferencesPath() again (after the path or the file changed).
void reloadPreferences();

// Puts a project file at the top of the recent list (and saves). Files that are gone drop
// off the list when it is shown, not here.
void noteRecentFile(const std::string& path);

// Sets one preference by its name in PREFERENCES SET: units, autosave, material, post,
// machine, updates, update-url, rulers. False (and why) for an unknown name or a bad value.
bool setPreference(Preferences& p, const std::string& key, const std::string& value, std::string* error);
// One "name: value" line per preference.
std::vector<std::string> describePreferences(const Preferences& p);

} // namespace sicdm

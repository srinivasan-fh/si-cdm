#pragma once

// Preferences: what a user sets once for every project, kept in preferences.json in the
// per-user configuration folder. The recent files list lives here too.

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
    std::string updateUrl;           // where the latest release is described (see UpdateCheck.hpp)
    std::string language = "en";     // the interface's language (see Translate.hpp)

    static constexpr std::size_t kMaxRecent = 10;
};

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
// machine, updates, update-url. False (and why) for an unknown name or a bad value.
bool setPreference(Preferences& p, const std::string& key, const std::string& value, std::string* error);
// One "name: value" line per preference.
std::vector<std::string> describePreferences(const Preferences& p);

} // namespace sicdm

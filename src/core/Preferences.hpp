#pragma once

// Preferences: what a user sets once for every project, kept in preferences.json in the
// per-user configuration folder. The recent files list lives here too.

#include "core/Units.hpp"

#include <array>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace sicdm {

class Json;

struct Preferences {
    std::vector<std::string> recent; // project files, newest first (at most kMaxRecent)
    std::string units;               // for new projects ("mm", "in", ...); empty: as the last project
    int decimals = -1;               // ...their decimals (-1: the unit's own)
    int fraction = 0;                // ...inches as fractions to 1/fraction (0: decimal)
    bool radians = false;            // ...angles shown in radians
    int angleDecimals = -1;          // ...and to these decimals (-1: 2 for degrees, 4 for radians)
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
    // Navigation.
    std::string mouse = "sicdm";
    std::string spaceMouse = "object"; // a 3D mouse: "object" (the model follows the cap), "camera", or "off"
    float spaceMouseSpeed = 1.0f;      // its speed (0.25 to 4)     // which buttons orbit, pan and zoom (MouseScheme.hpp)
    bool zoomInvert = false;         // the mouse wheel zooms the other way
    float zoomSpeed = 1.0f;          // wheel and zoom-drag speed (0.25 to 4)
    float orbitSpeed = 1.0f;         // orbit drag speed (0.25 to 4)
    bool zoomToCursor = true;        // zoom about the point under the mouse (off: the view's centre)
    bool animateViews = true;        // standard views, Fit and the view sphere glide (off: jump)
    // The 3D window as it starts (and as these change).
    std::string displayMode = "shaded-edges"; // shaded, shaded-edges, wireframe, hidden-line
    bool perspective = true;         // off: orthographic
    bool grid = true;
    std::string background = "dark"; // dark, grey, light, white, black
    bool viewSphere = true;          // the view sphere in the corner
    // Modelling.
    bool snapGrid = true;            // sketch points snap to the grid...
    bool snapPoints = true;          // ...and to other curves' points
    float moveStep = 1.0f;           // the Move gizmo's snap steps (mm, degrees)
    float angleStep = 15.0f;
    // Files.
    int recentCount = 10;            // how many recent projects File > Open Recent lists (1 to 30)
    int undoSteps = 16;              // how many steps undo keeps (1 to 500)
    std::string selectionColour = "blue"; // blue, orange, green, magenta, yellow, red, or #rrggbb
    std::string meshQuality = "normal";   // coarse, normal, fine, finest: exact shapes to triangles
    bool backup = false;             // SAVE keeps the file it replaces as name.sicdm.bak
    std::string projectFolder;       // where Open and Save As start with no project open; empty: the current folder
    std::string theme = "dark";      // the interface's colours: dark, light, high-contrast
    float uiScale = 1.0f;            // the whole interface larger or smaller (0.75 to 2)
    // Start-up and graphics (antialiasing on the next start).
    bool splash = true;
    int antialiasing = 4;            // multisamples: 0 (off), 2, 4 or 8
    bool vsync = true;               // draw in step with the screen (off: as fast as it can)
    int mcpPort = 0;                 // > 0: the AI agents' MCP server starts on this port
    // The main window and its side panel as they were left (kept on close, not settings).
    int windowX = 0, windowY = 0, windowWidth = 0, windowHeight = 0; // 0 size: not kept yet
    bool windowMaximized = false;
    bool modelPanel = false;         // the docked panel beside the tool rail...
    int panelPage = 0;               // ...and its page (0 Model, 1 Library, 2 CAM)
    float panelWidth = 260.0f;       // ...and its width (200 to 640)
    bool toolColumns2 = false;       // the left tool rail in two columns (default: one)
    std::string updateUrl;           // where the latest release is described (see UpdateCheck.hpp)
    std::string language = "en";     // the interface's language (see Translate.hpp)
    std::string vault;               // the team's vault folder (VAULT; Pdm.hpp); empty: $SICDM_VAULT, else none
    std::map<std::string, std::string> keys; // keyboard shortcuts changed from their defaults: action -> keys (Keymap.hpp)

    static constexpr std::size_t kMaxRecent = 30; // the most RECENT-COUNT allows
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

// A new project's display units from the preferences (units, decimals, fractions, angles).
void applyNewProjectUnits(const Preferences& p, UnitSettings& units);

// The background colours (top, bottom of the gradient) of a BACKGROUND preference.
struct BackgroundColours {
    float top[3], bottom[3];
};
BackgroundColours backgroundColours(const std::string& name);
const std::vector<std::string>& backgroundNames();
const std::vector<std::string>& displayModeNames();
// The selection colour's red, green, blue (0..1) for a SELECTION-COLOUR value; nullopt if not one.
std::optional<std::array<float, 3>> selectionRgb(const std::string& value);
// 0 coarse ... 3 finest, for a MESH-QUALITY value (-1: not one).
int meshQualityLevel(const std::string& value);

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

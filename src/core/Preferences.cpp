#include "core/Preferences.hpp"

#include "core/Keymap.hpp"
#include "core/MouseScheme.hpp"
#include "core/Translate.hpp"

#include "core/Cam.hpp"
#include "core/Json.hpp"
#include "core/Units.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <cmath>
#include <fstream>
#include <map>
#include <optional>
#include <sstream>

namespace sicdm {

namespace fs = std::filesystem;

namespace {

std::string env(const char* k) {
    const char* v = std::getenv(k);
    return v ? v : "";
}

std::string lower(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

bool samePath(const std::string& a, const std::string& b) {
#if defined(_WIN32)
    return lower(a) == lower(b);
#else
    return a == b;
#endif
}

Preferences readPreferences() {
    std::ifstream f(preferencesPath(), std::ios::binary);
    if (!f) return {};
    std::ostringstream s;
    s << f.rdbuf();
    const auto j = Json::parse(s.str());
    return j ? preferencesFromJson(*j) : Preferences{};
}

std::string upper(std::string s) {
    for (char& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

std::optional<double> number(const std::string& s) {
    char* end = nullptr;
    const double x = std::strtod(s.c_str(), &end);
    if (s.empty() || end == s.c_str() || *end != '\0' || !std::isfinite(x)) return std::nullopt;
    return x;
}

// The settings that are ON or OFF, by name.
bool* onOffKey(Preferences& p, const std::string& k) {
    if (k == "zoom-invert") return &p.zoomInvert;
    if (k == "zoom-to-cursor") return &p.zoomToCursor;
    if (k == "animate-views" || k == "animation") return &p.animateViews;
    if (k == "perspective") return &p.perspective;
    if (k == "grid") return &p.grid;
    if (k == "view-sphere") return &p.viewSphere;
    if (k == "snap-grid") return &p.snapGrid;
    if (k == "snap-points") return &p.snapPoints;
    if (k == "backup") return &p.backup;
    if (k == "splash") return &p.splash;
    if (k == "vsync") return &p.vsync;
    return nullptr;
}

bool g_loaded = false;
Preferences g_prefs;

} // namespace

Json preferencesToJson(const Preferences& p) {
    Json j = Json::object();
    Json recent = Json::array();
    for (const std::string& r : p.recent) recent.push(r);
    j.set("recent", recent);
    j.set("units", p.units);
    j.set("decimals", p.decimals);
    j.set("fraction", p.fraction);
    j.set("angles", p.radians ? "rad" : "deg");
    j.set("angle_decimals", p.angleDecimals);
    j.set("autosave_seconds", p.autosaveSeconds);
    j.set("cam_material", p.camMaterial);
    j.set("cam_post", p.camPost);
    j.set("cam_machine", p.camMachine);
    j.set("check_updates", p.checkUpdates);
    j.set("rulers", p.rulers);
    j.set("reopen_last", p.reopenLast);
    Json session = Json::array();
    for (const std::string& f : p.session) session.push(f);
    j.set("session", session);
    j.set("session_active", p.sessionActive);
    if (p.sessionOpening) j.set("session_opening", true);
    j.set("mouse", p.mouse);
    j.set("zoom_invert", p.zoomInvert);
    j.set("zoom_speed", p.zoomSpeed);
    j.set("orbit_speed", p.orbitSpeed);
    j.set("zoom_to_cursor", p.zoomToCursor);
    j.set("animate_views", p.animateViews);
    j.set("display", p.displayMode);
    j.set("perspective", p.perspective);
    j.set("grid", p.grid);
    j.set("background", p.background);
    j.set("view_sphere", p.viewSphere);
    j.set("snap_grid", p.snapGrid);
    j.set("snap_points", p.snapPoints);
    j.set("move_step", p.moveStep);
    j.set("angle_step", p.angleStep);
    j.set("backup", p.backup);
    j.set("recent_count", p.recentCount);
    j.set("undo_steps", p.undoSteps);
    j.set("selection_colour", p.selectionColour);
    j.set("mesh_quality", p.meshQuality);
    j.set("project_folder", p.projectFolder);
    j.set("ui_scale", p.uiScale);
    j.set("theme", p.theme);
    j.set("splash", p.splash);
    j.set("antialiasing", p.antialiasing);
    j.set("vsync", p.vsync);
    j.set("mcp_port", p.mcpPort);
    if (p.windowWidth > 0 && p.windowHeight > 0) {
        Json w = Json::object();
        w.set("x", p.windowX).set("y", p.windowY).set("width", p.windowWidth).set("height", p.windowHeight).set("maximized", p.windowMaximized);
        j.set("window", w);
    }
    j.set("model_panel", p.modelPanel);
    j.set("panel_page", p.panelPage);
    j.set("update_url", p.updateUrl);
    j.set("language", p.language);
    j.set("vault", p.vault);
    if (!p.keys.empty()) {
        Json keys = Json::object();
        for (const auto& [id, chords] : p.keys) keys.set(id, chords);
        j.set("keys", keys);
    }
    return j;
}

Preferences preferencesFromJson(const Json& j) {
    Preferences p;
    const Json& recent = j["recent"];
    if (j["recent_count"].isNumber()) p.recentCount = std::clamp(j["recent_count"].asInt(10), 1, static_cast<int>(Preferences::kMaxRecent));
    for (std::size_t i = 0; i < recent.size() && p.recent.size() < static_cast<std::size_t>(p.recentCount); ++i)
        if (recent[i].isString() && !recent[i].asString().empty()) p.recent.push_back(recent[i].asString());
    std::string err;
    // Each value goes through the same checks as PREFERENCES SET, so a hand-edited file
    // can't hold one the app would refuse.
    auto take = [&](const char* key, const char* name) {
        const Json& v = j[key];
        if (v.isString() && !v.asString().empty()) setPreference(p, name, v.asString(), &err);
    };
    take("units", "units");
    take("cam_material", "material");
    take("cam_post", "post");
    take("cam_machine", "machine");
    take("update_url", "update-url");
    take("language", "language");
    take("vault", "vault");
    if (j["autosave_seconds"].isNumber()) p.autosaveSeconds = std::clamp(j["autosave_seconds"].asInt(), 0, 3600);
    p.checkUpdates = j["check_updates"].asBool(false);
    p.rulers = j["rulers"].asBool(true);
    p.zoomInvert = j["zoom_invert"].asBool(false);
    p.zoomToCursor = j["zoom_to_cursor"].asBool(true);
    p.animateViews = j["animate_views"].asBool(true);
    p.perspective = j["perspective"].asBool(true);
    p.grid = j["grid"].asBool(true);
    p.viewSphere = j["view_sphere"].asBool(true);
    p.snapGrid = j["snap_grid"].asBool(true);
    p.snapPoints = j["snap_points"].asBool(true);
    p.backup = j["backup"].asBool(false);
    p.splash = j["splash"].asBool(true);
    p.vsync = j["vsync"].asBool(true);
    // Numbers and names go through PREFERENCES SET's checks.
    auto number = [&](const char* key, const char* name) {
        if (j[key].isNumber()) setPreference(p, name, std::to_string(j[key].asNumber()), &err);
    };
    number("decimals", "decimals");
    number("fraction", "fraction");
    take("angles", "angles");
    number("angle_decimals", "angle-decimals");
    number("ui_scale", "ui-scale");
    number("undo_steps", "undo-steps");
    take("selection_colour", "selection-colour");
    take("mesh_quality", "mesh-quality");
    number("zoom_speed", "zoom-speed");
    number("orbit_speed", "orbit-speed");
    number("move_step", "move-step");
    number("angle_step", "angle-step");
    number("antialiasing", "antialiasing");
    number("mcp_port", "mcp-port");
    take("display", "display");
    take("mouse", "mouse");
    take("theme", "theme");
    if (const Json& w = j["window"]; w.isObject()) {
        const int ww = w["width"].asInt(0), wh = w["height"].asInt(0);
        if (ww >= 400 && wh >= 300 && ww <= 16384 && wh <= 16384) { // a sane size, else not kept
            p.windowWidth = ww;
            p.windowHeight = wh;
            p.windowX = w["x"].asInt(0);
            p.windowY = w["y"].asInt(0);
            p.windowMaximized = w["maximized"].asBool(false);
        }
    }
    p.modelPanel = j["model_panel"].asBool(false);
    p.panelPage = std::clamp(j["panel_page"].asInt(0), 0, 2);
    take("background", "background");
    take("project_folder", "project-folder");
    p.reopenLast = j["reopen_last"].asBool(true);
    const Json& session = j["session"];
    for (std::size_t i = 0; i < session.size() && p.session.size() < Preferences::kMaxSession; ++i)
        if (session[i].isString() && !session[i].asString().empty()) p.session.push_back(session[i].asString());
    if (j["session_active"].isNumber())
        p.sessionActive = std::clamp(j["session_active"].asInt(), 0, std::max(0, static_cast<int>(p.session.size()) - 1));
    p.sessionOpening = j["session_opening"].asBool(false);
    // Changed shortcuts: only keys this version can read (a hand-edited file can't break them).
    if (j["keys"].isObject())
        for (const auto& [id, v] : j["keys"].members())
            if (v.isString() && (findKeyAction(id) || isToolKeyId(id)) && parseChords(v.asString())) p.keys[id] = v.asString();
    return p;
}

std::string configDirectory() {
#if defined(_WIN32)
    if (const std::string d = env("APPDATA"); !d.empty()) return (fs::path(d) / "SI-CDM").string();
#elif defined(__APPLE__)
    if (const std::string h = env("HOME"); !h.empty()) return (fs::path(h) / "Library" / "Application Support" / "SI-CDM").string();
#else
    if (const std::string d = env("XDG_CONFIG_HOME"); !d.empty()) return (fs::path(d) / "sicdm").string();
    if (const std::string h = env("HOME"); !h.empty()) return (fs::path(h) / ".config" / "sicdm").string();
#endif
    std::error_code ec;
    return (fs::temp_directory_path(ec) / "sicdm").string();
}

std::string preferencesPath() {
    if (const std::string p = env("SICDM_PREFERENCES"); !p.empty()) return p;
    return (fs::path(configDirectory()) / "preferences.json").string();
}

Preferences& preferences() {
    if (!g_loaded) {
        g_prefs = readPreferences();
        g_loaded = true;
    }
    return g_prefs;
}

void reloadPreferences() {
    g_prefs = readPreferences();
    g_loaded = true;
}

bool savePreferences(std::string* error) {
    const fs::path path = preferencesPath();
    std::error_code ec;
    if (path.has_parent_path()) fs::create_directories(path.parent_path(), ec);
    // Written beside and renamed over, so a crash mid-write leaves the old file.
    const fs::path tmp = path.string() + ".tmp";
    {
        std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
        if (!f) {
            if (error) *error = "Can't write " + tmp.string();
            return false;
        }
        f << preferencesToJson(preferences()).dump() << "\n";
        if (!f) {
            if (error) *error = "Can't write " + tmp.string();
            return false;
        }
    }
    fs::rename(tmp, path, ec);
    if (ec) {
        fs::remove(path, ec);
        fs::rename(tmp, path, ec);
    }
    if (ec) {
        if (error) *error = "Can't write " + path.string() + ": " + ec.message();
        return false;
    }
    return true;
}

std::optional<std::array<float, 3>> selectionRgb(const std::string& value) {
    static const std::map<std::string, std::array<float, 3>> named{
        {"blue", {0.30f, 0.65f, 1.00f}},   {"orange", {1.00f, 0.55f, 0.10f}}, {"green", {0.25f, 0.85f, 0.35f}},
        {"magenta", {0.95f, 0.30f, 0.85f}}, {"yellow", {1.00f, 0.88f, 0.15f}}, {"red", {1.00f, 0.30f, 0.25f}},
    };
    if (const auto it = named.find(lower(value)); it != named.end()) return it->second;
    if (value.size() == 7 && value[0] == '#') {
        std::array<float, 3> c{};
        for (int k = 0; k < 3; ++k) {
            const std::string h = value.substr(1 + 2 * static_cast<std::size_t>(k), 2);
            if (!std::isxdigit(static_cast<unsigned char>(h[0])) || !std::isxdigit(static_cast<unsigned char>(h[1]))) return std::nullopt;
            c[static_cast<std::size_t>(k)] = static_cast<float>(std::strtol(h.c_str(), nullptr, 16)) / 255.0f;
        }
        return c;
    }
    return std::nullopt;
}

int meshQualityLevel(const std::string& value) {
    const std::string v = lower(value);
    if (v == "coarse") return 0;
    if (v == "normal") return 1;
    if (v == "fine") return 2;
    if (v == "finest") return 3;
    return -1;
}

const std::vector<std::string>& backgroundNames() {
    static const std::vector<std::string> names{"dark", "grey", "light", "white", "black"};
    return names;
}

const std::vector<std::string>& displayModeNames() {
    static const std::vector<std::string> names{"shaded", "shaded-edges", "wireframe", "hidden-line"};
    return names;
}

BackgroundColours backgroundColours(const std::string& name) {
    if (name == "grey") return {{0.44f, 0.46f, 0.50f}, {0.22f, 0.23f, 0.26f}};
    if (name == "light") return {{0.90f, 0.92f, 0.95f}, {0.64f, 0.68f, 0.74f}};
    if (name == "white") return {{1.0f, 1.0f, 1.0f}, {0.93f, 0.94f, 0.96f}};
    if (name == "black") return {{0.03f, 0.03f, 0.04f}, {0.0f, 0.0f, 0.0f}};
    return {{0.20f, 0.23f, 0.29f}, {0.06f, 0.07f, 0.09f}}; // dark, the default
}

void applyNewProjectUnits(const Preferences& p, UnitSettings& units) {
    if (const auto u = parseUnitName(p.units)) units.unit = *u;
    if (p.decimals >= 0) units.precision = p.decimals;
    if (p.fraction > 0) {
        units.style = LengthStyle::Fractional;
        units.denominator = p.fraction;
    }
    units.radians = p.radians;
    units.angleDecimals = p.angleDecimals;
}

StartupSession startupSession(const Preferences& p) {
    StartupSession out;
    if (p.sessionOpening) {
        out.crashedLastTime = true;
        return out;
    }
    std::error_code ec;
    for (std::size_t i = 0; i < p.session.size(); ++i) {
        if (!fs::is_regular_file(p.session[i], ec)) continue; // moved or deleted since
        if (static_cast<int>(i) == p.sessionActive) out.active = static_cast<int>(out.files.size());
        out.files.push_back(p.session[i]);
    }
    if (out.files.empty())
        for (const std::string& r : p.recent)
            if (fs::is_regular_file(r, ec)) {
                out.files.push_back(r);
                break;
            }
    return out;
}

void noteRecentFile(const std::string& path) {
    if (path.empty()) return;
    std::error_code ec;
    const std::string abs = fs::absolute(path, ec).lexically_normal().string();
    std::vector<std::string>& r = preferences().recent;
    r.erase(std::remove_if(r.begin(), r.end(), [&](const std::string& q) { return samePath(q, abs); }), r.end());
    r.insert(r.begin(), abs);
    if (r.size() > static_cast<std::size_t>(preferences().recentCount)) r.resize(static_cast<std::size_t>(preferences().recentCount));
    savePreferences();
}

bool setPreference(Preferences& p, const std::string& key, const std::string& value, std::string* error) {
    auto fail = [&](const std::string& why) {
        if (error) *error = why;
        return false;
    };
    std::string k = lower(key);
    for (char& c : k)
        if (c == '_') c = '-';
    const std::string v = lower(value);
    const bool clear = v.empty() || v == "default" || v == "none" || v == "auto";
    if (k == "units") {
        if (clear || v == "last") {
            p.units.clear();
        } else {
            const auto u = parseUnitName(v);
            if (!u) return fail("Unknown units '" + value + "' (mm, cm, m, in, ft, pt)");
            p.units = unitInfo(*u).symbol;
        }
    } else if (k == "autosave") {
        if (v == "off" || v == "no" || v == "0") {
            p.autosaveSeconds = 0;
        } else {
            char* end = nullptr;
            const double s = std::strtod(v.c_str(), &end);
            if (end == v.c_str() || s < 5 || s > 3600) return fail("AUTOSAVE needs seconds (5 to 3600) or OFF");
            p.autosaveSeconds = static_cast<int>(s);
        }
    } else if (k == "material") {
        if (clear) p.camMaterial.clear();
        else if (const CamMaterial* m = camMaterial(v)) p.camMaterial = m->key;
        else return fail("Unknown material '" + value + "'");
    } else if (k == "post") {
        if (clear) p.camPost.clear();
        else if (std::find(camPosts().begin(), camPosts().end(), v) != camPosts().end()) p.camPost = v;
        else return fail("Unknown post '" + value + "'");
    } else if (k == "machine") {
        if (clear) p.camMachine.clear();
        else if (const CamMachine* m = camMachinePreset(value)) p.camMachine = m->name;
        else return fail("Unknown machine '" + value + "'");
    } else if (k == "updates") {
        if (v == "on" || v == "yes" || v == "true" || v == "1") p.checkUpdates = true;
        else if (v == "off" || v == "no" || v == "false" || v == "0") p.checkUpdates = false;
        else return fail("UPDATES is ON or OFF");
    } else if (k == "rulers" || k == "ruler") {
        if (v == "on" || v == "yes" || v == "true" || v == "1" || v == "show") p.rulers = true;
        else if (v == "off" || v == "no" || v == "false" || v == "0" || v == "hide") p.rulers = false;
        else return fail("RULERS is ON or OFF");
    } else if (k == "reopen" || k == "reopen-last" || k == "reopen_last") {
        if (v == "on" || v == "yes" || v == "true" || v == "1") p.reopenLast = true;
        else if (v == "off" || v == "no" || v == "false" || v == "0") p.reopenLast = false;
        else return fail("REOPEN is ON or OFF");
    } else if (bool* flag = onOffKey(p, k)) {
        if (v == "on" || v == "yes" || v == "true" || v == "1" || v == "show") *flag = true;
        else if (v == "off" || v == "no" || v == "false" || v == "0" || v == "hide") *flag = false;
        else return fail(upper(key) + " is ON or OFF");
    } else if (k == "zoom-speed" || k == "orbit-speed") {
        const auto x = number(v);
        if (!x || *x < 0.25 || *x > 4.0) return fail(upper(key) + " is a factor from 0.25 to 4 (1: normal)");
        (k == "zoom-speed" ? p.zoomSpeed : p.orbitSpeed) = static_cast<float>(*x);
    } else if (k == "ui-scale" || k == "scale") {
        const auto x = number(v.size() > 1 && v.back() == '%' ? std::to_string(std::atof(v.c_str()) / 100.0) : v);
        if (!x || *x < 0.75 || *x > 2.0) return fail("UI-SCALE is 0.75 to 2 (or 75% to 200%)");
        p.uiScale = static_cast<float>(std::round(*x * 100.0) / 100.0);
    } else if (k == "move-step") {
        const auto x = number(v);
        if (!x || *x <= 0.0 || *x > 1000.0) return fail("MOVE-STEP is a distance in mm, above 0 and up to 1000");
        p.moveStep = static_cast<float>(*x);
    } else if (k == "angle-step") {
        const auto x = number(v);
        if (!x || *x <= 0.0 || *x > 180.0) return fail("ANGLE-STEP is in degrees, above 0 and up to 180");
        p.angleStep = static_cast<float>(*x);
    } else if (k == "antialiasing" || k == "aa") {
        const auto x = number(v == "off" ? "0" : v);
        const int n = x ? static_cast<int>(*x) : -1;
        if (n != 0 && n != 2 && n != 4 && n != 8) return fail("ANTIALIASING is 0 (OFF), 2, 4 or 8 samples");
        p.antialiasing = n;
    } else if (k == "mcp-port") {
        const auto x = number(v == "off" ? "0" : v);
        const int n = x ? static_cast<int>(*x) : -1;
        if (!x || *x != n || (n != 0 && (n < 1024 || n > 65535))) return fail("MCP-PORT is a port from 1024 to 65535, or OFF");
        p.mcpPort = n;
    } else if (k == "decimals" || k == "precision" || k == "angle-decimals") {
        int& slot = k == "angle-decimals" ? p.angleDecimals : p.decimals;
        if (clear) {
            slot = -1;
        } else {
            const auto x = number(v);
            if (!x || *x < 0 || *x > 10 || *x != std::floor(*x)) return fail(upper(key) + " is 0 to 10, or AUTO");
            slot = static_cast<int>(*x);
        }
    } else if (k == "fraction" || k == "fractions") {
        std::string d = v.rfind("1/", 0) == 0 ? v.substr(2) : v;
        if (v == "off" || v == "decimal" || clear) d = "0";
        const auto x = number(d);
        const int n = x ? static_cast<int>(*x) : -1;
        if (!x || *x != n || (n != 0 && (n < 2 || n > 256 || (n & (n - 1)) != 0))) return fail("FRACTION is 1/2, 1/4 ... 1/256, or OFF");
        p.fraction = n;
    } else if (k == "angles" || k == "angle-units") {
        if (v == "deg" || v == "degrees") p.radians = false;
        else if (v == "rad" || v == "radians") p.radians = true;
        else return fail("ANGLES is DEG or RAD");
    } else if (k == "recent-count" || k == "recent") {
        const auto x = number(v);
        if (!x || *x < 1 || *x > static_cast<double>(Preferences::kMaxRecent) || *x != std::floor(*x)) return fail("RECENT-COUNT is 1 to 30");
        p.recentCount = static_cast<int>(*x);
        if (p.recent.size() > static_cast<std::size_t>(p.recentCount)) p.recent.resize(static_cast<std::size_t>(p.recentCount));
    } else if (k == "undo-steps" || k == "undo") {
        const auto x = number(v);
        if (!x || *x < 1 || *x > 500 || *x != std::floor(*x)) return fail("UNDO-STEPS is 1 to 500");
        p.undoSteps = static_cast<int>(*x);
    } else if (k == "selection-colour" || k == "selection-color" || k == "selection") {
        if (clear) p.selectionColour = "blue";
        else if (selectionRgb(v)) p.selectionColour = v;
        else return fail("SELECTION-COLOUR is blue, orange, green, magenta, yellow, red, or #rrggbb");
    } else if (k == "mesh-quality" || k == "tessellation") {
        if (clear) p.meshQuality = "normal";
        else if (meshQualityLevel(v) >= 0) p.meshQuality = v;
        else return fail("MESH-QUALITY is coarse, normal, fine or finest");
    } else if (k == "theme") {
        if (clear || v == "dark") p.theme = "dark";
        else if (v == "light") p.theme = "light";
        else if (v == "high-contrast" || v == "contrast" || v == "highcontrast") p.theme = "high-contrast";
        else return fail("THEME is dark, light or high-contrast");
    } else if (k == "mouse") {
        if (clear) p.mouse = "sicdm";
        else if (isMouseScheme(v)) p.mouse = v;
        else {
            std::string names;
            for (const MouseSchemeInfo& m : mouseSchemes()) names += (names.empty() ? "" : ", ") + m.key;
            return fail("MOUSE is one of " + names);
        }
    } else if (k == "display") {
        const auto& names = displayModeNames();
        if (clear) p.displayMode = "shaded-edges";
        else if (std::find(names.begin(), names.end(), v) != names.end()) p.displayMode = v;
        else return fail("DISPLAY is shaded, shaded-edges, wireframe or hidden-line");
    } else if (k == "background") {
        const auto& names = backgroundNames();
        if (clear) p.background = "dark";
        else if (std::find(names.begin(), names.end(), v) != names.end()) p.background = v;
        else return fail("BACKGROUND is dark, grey, light, white or black");
    } else if (k == "project-folder" || k == "folder") {
        if (clear) p.projectFolder.clear();
        else p.projectFolder = value; // it may not be reachable yet (a network drive): the dialogs fall back
    } else if (k == "update-url" || k == "updateurl" || k == "update_url") {
        if (clear) p.updateUrl.clear();
        else if (value.rfind("https://", 0) == 0 || value.rfind("http://", 0) == 0 || value.rfind("file://", 0) == 0) p.updateUrl = value;
        else return fail("UPDATE-URL needs an https://, http:// or file:// address");
    } else if (k == "language") {
        if (clear || v == "english") {
            p.language = "en";
        } else {
            std::string code;
            for (const LanguageInfo& l : languages())
                if (v == lower(l.code) || v == lower(l.name)) code = l.code;
            if (code.empty()) return fail("No translation for '" + value + "' (PREFERENCES LANGUAGES lists them)");
            p.language = code;
        }
    } else if (k == "vault") {
        if (clear) p.vault.clear();
        else p.vault = value; // it may not be reachable yet (a network drive); VAULT says so
    } else {
        return fail("Unknown preference '" + key + "' (PREFERENCES lists them)");
    }
    return true;
}

std::vector<std::string> describePreferences(const Preferences& p) {
    auto orDefault = [](const std::string& s, const char* d) { return s.empty() ? std::string(d) : s; };
    auto onOff = [](bool b) { return b ? "on" : "off"; };
    auto factor = [](float x) {
        char b[32];
        std::snprintf(b, sizeof b, "%g", static_cast<double>(x));
        return std::string(b);
    };
    return {
        "units: " + orDefault(p.units, "as the last project"),
        "decimals: " + (p.decimals >= 0 ? std::to_string(p.decimals) : std::string("the unit's own")),
        "fraction: " + (p.fraction > 0 ? "1/" + std::to_string(p.fraction) + " in" : std::string("off")),
        std::string("angles: ") + (p.radians ? "rad" : "deg") + (p.angleDecimals >= 0 ? ", " + std::to_string(p.angleDecimals) + " decimals" : std::string()),
        "autosave: " + (p.autosaveSeconds > 0 ? std::to_string(p.autosaveSeconds) + " s" : std::string("off")),
        "material: " + orDefault(p.camMaterial, "default (aluminium)"),
        "post: " + orDefault(p.camPost, "default (fanuc)"),
        "machine: " + orDefault(p.camMachine, "default (generic)"),
        std::string("updates: ") + (p.checkUpdates ? "on" : "off"),
        std::string("rulers: ") + (p.rulers ? "on" : "off"),
        std::string("reopen: ") + (p.reopenLast ? "on (the projects open at the last close)" : "off (start with the demo part)"),
        "mouse: " + p.mouse,
        std::string("zoom-invert: ") + onOff(p.zoomInvert),
        "zoom-speed: " + factor(p.zoomSpeed),
        "orbit-speed: " + factor(p.orbitSpeed),
        std::string("zoom-to-cursor: ") + onOff(p.zoomToCursor),
        std::string("animate-views: ") + onOff(p.animateViews),
        "display: " + p.displayMode,
        std::string("perspective: ") + (p.perspective ? "on" : "off (orthographic)"),
        std::string("grid: ") + onOff(p.grid),
        "background: " + p.background,
        std::string("view-sphere: ") + onOff(p.viewSphere),
        std::string("snap-grid: ") + onOff(p.snapGrid),
        std::string("snap-points: ") + onOff(p.snapPoints),
        "move-step: " + factor(p.moveStep) + " mm",
        "angle-step: " + factor(p.angleStep) + " deg",
        "recent-count: " + std::to_string(p.recentCount),
        "undo-steps: " + std::to_string(p.undoSteps),
        "selection-colour: " + p.selectionColour,
        "mesh-quality: " + p.meshQuality,
        std::string("backup: ") + (p.backup ? "on (name.sicdm.bak)" : "off"),
        "project-folder: " + orDefault(p.projectFolder, "the current folder"),
        "theme: " + p.theme,
        "ui-scale: " + factor(p.uiScale * 100.0f) + "%",
        std::string("splash: ") + onOff(p.splash),
        "antialiasing: " + (p.antialiasing > 0 ? std::to_string(p.antialiasing) + " samples" : std::string("off")) + " (from the next start)",
        std::string("vsync: ") + onOff(p.vsync),
        "mcp-port: " + (p.mcpPort > 0 ? std::to_string(p.mcpPort) : std::string("off")),
        "update-url: " + orDefault(p.updateUrl, "none"),
        "language: " + p.language,
        "vault: " + orDefault(p.vault, "none"),
    };
}

} // namespace sicdm

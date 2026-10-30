#include "core/Preferences.hpp"

#include "core/Keymap.hpp"
#include "core/Translate.hpp"

#include "core/Cam.hpp"
#include "core/Json.hpp"
#include "core/Units.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
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

bool g_loaded = false;
Preferences g_prefs;

} // namespace

Json preferencesToJson(const Preferences& p) {
    Json j = Json::object();
    Json recent = Json::array();
    for (const std::string& r : p.recent) recent.push(r);
    j.set("recent", recent);
    j.set("units", p.units);
    j.set("autosave_seconds", p.autosaveSeconds);
    j.set("cam_material", p.camMaterial);
    j.set("cam_post", p.camPost);
    j.set("cam_machine", p.camMachine);
    j.set("check_updates", p.checkUpdates);
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
    for (std::size_t i = 0; i < recent.size() && p.recent.size() < Preferences::kMaxRecent; ++i)
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

void noteRecentFile(const std::string& path) {
    if (path.empty()) return;
    std::error_code ec;
    const std::string abs = fs::absolute(path, ec).lexically_normal().string();
    std::vector<std::string>& r = preferences().recent;
    r.erase(std::remove_if(r.begin(), r.end(), [&](const std::string& q) { return samePath(q, abs); }), r.end());
    r.insert(r.begin(), abs);
    if (r.size() > Preferences::kMaxRecent) r.resize(Preferences::kMaxRecent);
    savePreferences();
}

bool setPreference(Preferences& p, const std::string& key, const std::string& value, std::string* error) {
    auto fail = [&](const std::string& why) {
        if (error) *error = why;
        return false;
    };
    const std::string k = lower(key), v = lower(value);
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
        return fail("Unknown preference '" + key + "' (units, autosave, material, post, machine, updates, update-url, language, vault)");
    }
    return true;
}

std::vector<std::string> describePreferences(const Preferences& p) {
    auto orDefault = [](const std::string& s, const char* d) { return s.empty() ? std::string(d) : s; };
    return {
        "units: " + orDefault(p.units, "as the last project"),
        "autosave: " + (p.autosaveSeconds > 0 ? std::to_string(p.autosaveSeconds) + " s" : std::string("off")),
        "material: " + orDefault(p.camMaterial, "default (aluminium)"),
        "post: " + orDefault(p.camPost, "default (fanuc)"),
        "machine: " + orDefault(p.camMachine, "default (generic)"),
        std::string("updates: ") + (p.checkUpdates ? "on" : "off"),
        "update-url: " + orDefault(p.updateUrl, "none"),
        "language: " + p.language,
        "vault: " + orDefault(p.vault, "none"),
    };
}

} // namespace sicdm

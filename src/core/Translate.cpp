#include "core/Translate.hpp"

#include "core/Json.hpp"
#include "core/Preferences.hpp"

#include <algorithm>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <mutex>
#include <set>
#include <unordered_map>

namespace sicdm {

namespace lang {
// Built-in translations (src/core/lang/*.cpp): pairs of English and translated words.
extern const char* const kGermanName;
extern const char* const kGerman[][2];
extern const std::size_t kGermanCount;
} // namespace lang

namespace {

struct Builtin {
    const char* code;
    const char* name;
    const char* const (*table)[2];
    const std::size_t* count;
};

const Builtin kBuiltins[] = {
    {"de", lang::kGermanName, lang::kGerman, &lang::kGermanCount},
};

struct State {
    std::mutex lock;
    std::string code = "en";
    std::unordered_map<std::string, std::string> table; // English -> translated
    std::unordered_map<std::string, const char*> cache; // a looked-up label -> what is shown
    std::deque<std::string> store;                      // the shown strings (stable addresses)
    std::set<std::string> missing;
};

State& state() {
    static State s;
    return s;
}

std::string& languageDirectoryOverride() {
    static std::string dir;
    return dir;
}

std::string installedPath(const std::string& code) { return (std::filesystem::path(languageDirectory()) / (code + ".json")).string(); }

// An installed language file: its name and words. False when there is none or it is broken.
bool readInstalled(const std::string& path, std::string& name, std::unordered_map<std::string, std::string>& words, std::string* why) {
    std::error_code ec;
    if (!std::filesystem::exists(path, ec)) return false;
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        if (why) *why = "Cannot read " + path;
        return false;
    }
    const std::optional<std::string> text = std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    std::string err;
    const auto j = Json::parse(*text, &err);
    if (!j || !j->isObject() || !(*j)["strings"].isObject()) {
        if (why) *why = path + ": not a language file (" + (err.empty() ? "no \"strings\"" : err) + ")";
        return false;
    }
    name = (*j)["name"].asString(name);
    for (const auto& [k, v] : (*j)["strings"].members())
        if (v.isString() && !v.asString().empty()) words[k] = v.asString();
    return true;
}

} // namespace

std::vector<LanguageInfo> languages() {
    std::vector<LanguageInfo> out{{"en", "English"}};
    for (const Builtin& b : kBuiltins) out.push_back({b.code, b.name});
    std::error_code ec;
    const std::filesystem::path dir = languageDirectory();
    if (std::filesystem::is_directory(dir, ec))
        for (const auto& e : std::filesystem::directory_iterator(dir, ec)) {
            if (e.path().extension() != ".json") continue;
            const std::string code = e.path().stem().string();
            if (code == "en" || std::any_of(out.begin(), out.end(), [&](const LanguageInfo& l) { return l.code == code; })) continue;
            std::string name = code;
            std::unordered_map<std::string, std::string> words;
            if (readInstalled(e.path().string(), name, words, nullptr)) out.push_back({code, name});
        }
    std::sort(out.begin() + 1, out.end(), [](const LanguageInfo& a, const LanguageInfo& b) { return a.code < b.code; });
    return out;
}

bool setLanguage(const std::string& code, std::string* why) {
    std::unordered_map<std::string, std::string> words;
    bool found = code == "en" || code.empty();
    if (!found) {
        for (const Builtin& b : kBuiltins)
            if (code == b.code) {
                for (std::size_t i = 0; i < *b.count; ++i) words[b.table[i][0]] = b.table[i][1];
                found = true;
            }
        std::string name = code, err;
        if (readInstalled(installedPath(code), name, words, &err)) found = true;
        else if (!err.empty() && why) *why = err;
    }
    if (!found) {
        if (why && why->empty()) *why = "No translation for \"" + code + "\" (installed ones go in " + installedPath(code) + ")";
        return false;
    }
    State& s = state();
    std::lock_guard<std::mutex> g(s.lock);
    s.code = code.empty() ? "en" : code;
    s.table = std::move(words);
    s.cache.clear();
    s.missing.clear();
    return true;
}

const std::string& language() { return state().code; }

std::string languageDirectory() {
    if (!languageDirectoryOverride().empty()) return languageDirectoryOverride();
    if (const char* e = std::getenv("SICDM_LANG_DIR"); e && *e) return e;
    return (std::filesystem::path(configDirectory()) / "lang").string();
}

void setLanguageDirectory(const std::string& dir) { languageDirectoryOverride() = dir; }

const char* tr(const char* english) {
    if (!english) return "";
    State& s = state();
    std::lock_guard<std::mutex> g(s.lock);
    if (s.code == "en") return english;
    const auto hit = s.cache.find(english);
    if (hit != s.cache.end()) return hit->second;
    // The words: before an ImGui "##id", without the spaces around them.
    const std::string label = english;
    const std::size_t id = label.find("##");
    const std::string shown = label.substr(0, id);
    std::size_t a = 0, b = shown.size();
    while (a < b && shown[a] == ' ') ++a;
    while (b > a && shown[b - 1] == ' ') --b;
    const std::string words = shown.substr(a, b - a);
    if (words.empty()) return english; // an id alone: nothing to show
    const char* out = english;
    {
        const auto t = s.table.find(words);
        if (t != s.table.end()) {
            s.store.push_back(shown.substr(0, a) + t->second + shown.substr(b) + (id == std::string::npos ? std::string() : label.substr(id)));
            out = s.store.back().c_str();
        } else {
            s.missing.insert(words);
            s.store.push_back(label); // a copy: the caller's string may not outlive the call
            out = s.store.back().c_str();
        }
    }
    s.cache.emplace(label, out);
    return out;
}

std::string tr(const std::string& english) { return tr(english.c_str()); }

std::vector<std::string> untranslated() {
    State& s = state();
    std::lock_guard<std::mutex> g(s.lock);
    return {s.missing.begin(), s.missing.end()};
}

std::vector<std::pair<std::string, std::string>> builtinTranslation(const std::string& code) {
    std::vector<std::pair<std::string, std::string>> out;
    for (const Builtin& b : kBuiltins)
        if (code == b.code)
            for (std::size_t i = 0; i < *b.count; ++i) out.emplace_back(b.table[i][0], b.table[i][1]);
    return out;
}

} // namespace sicdm

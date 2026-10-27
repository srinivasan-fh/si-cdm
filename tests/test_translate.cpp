#include "TestFramework.hpp"
#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Translate.hpp"

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>

using namespace sicdm;

#define REQUIRE(c)                  \
    do {                            \
        const bool required_ = (c); \
        CHECK(required_);           \
        if (!required_) return;     \
    } while (0)

TEST_CASE("translation: German built in, ImGui ids kept, an installed file adds and corrects words") {
    const auto dir = std::filesystem::temp_directory_path() / "sicdm_lang_test";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    setLanguageDirectory(dir.string());
    // English: as written, the very same pointer.
    REQUIRE(setLanguage("en"));
    const char* file = "File";
    CHECK(tr(file) == file);
    // German.
    REQUIRE(setLanguage("de"));
    CHECK(std::string(tr("File")) == "Datei");
    CHECK(std::string(tr("Save##toolbar")) == "Speichern##toolbar"); // the hidden id stays
    CHECK(std::string(tr("  Undo ")) == "  Rückgängig ");          // and the spaces around the words
    CHECK(std::string(tr("##only")) == "##only");
    CHECK(std::string(tr("Level %d/%d")) == "Stufe %d/%d");
    const char* once = tr("Preferences");
    CHECK(tr("Preferences") == once); // looked up once, the same string after
    // A word the table lacks: shown in English and listed for the translator.
    CHECK(std::string(tr("Frobnicate the widget")) == "Frobnicate the widget");
    const auto missing = untranslated();
    CHECK(std::find(missing.begin(), missing.end(), "Frobnicate the widget") != missing.end());
    // Every built-in entry keeps the format specifiers of its English.
    for (const auto& [en, de] : builtinTranslation("de")) {
        auto specs = [](const std::string& s) {
            std::string out;
            for (std::size_t i = 0; i + 1 < s.size(); ++i)
                if (s[i] == '%') {
                    std::size_t j = i + 1;
                    while (j < s.size() && std::strchr("-+ #0123456789.lzh", s[j])) ++j;
                    if (j < s.size()) out += s.substr(i, j - i + 1);
                    i = j;
                }
            return out;
        };
        CHECK(specs(en) == specs(de));
    }
    // An installed file: corrects a built-in word, adds one, and a new language.
    {
        std::ofstream(dir / "de.json") << R"({"name": "Deutsch", "strings": {"File": "Ablage", "Frobnicate the widget": "Das Ding frobnizieren"}})";
        std::ofstream(dir / "xx.json") << R"({"name": "Testisch", "strings": {"File": "Fiile"}})";
    }
    REQUIRE(setLanguage("de"));
    CHECK(std::string(tr("File")) == "Ablage");
    CHECK(std::string(tr("Frobnicate the widget")) == "Das Ding frobnizieren");
    CHECK(std::string(tr("Save")) == "Speichern");
    const auto langs = languages();
    CHECK(langs.front().code == "en");
    CHECK(std::any_of(langs.begin(), langs.end(), [](const LanguageInfo& l) { return l.code == "xx" && l.name == "Testisch"; }));
    REQUIRE(setLanguage("xx"));
    CHECK(std::string(tr("File")) == "Fiile");
    // No such language: refused, the language stays.
    std::string why;
    CHECK(!setLanguage("zz", &why));
    CHECK(!why.empty());
    CHECK(language() == "xx");
    REQUIRE(setLanguage("en"));
    setLanguageDirectory("");
    std::filesystem::remove_all(dir);
}

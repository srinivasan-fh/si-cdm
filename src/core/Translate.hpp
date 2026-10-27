#pragma once

// The language the interface is shown in. The words are written in English in the code
// and looked up here: in a built-in translation, or in one installed as
// configDirectory()/lang/<code>.json:
//   {"name": "Deutsch", "strings": {"File": "Datei", "Save As...": "Speichern unter..."}}
// An installed file adds to a built-in language of the same code (its words win), so a
// translation can be finished or corrected without a new build. tools/i18n_extract.py
// lists every word the interface looks up, for a translator to start from.

#include <string>
#include <vector>

// Marks a word for translation where tr() can't be called (a constant table); tr() is then
// called on it where it is shown. tools/i18n_extract.py finds both.
#define SICDM_TR_NOOP(s) s

namespace sicdm {

struct LanguageInfo {
    std::string code; // "en", "de"...
    std::string name; // in the language itself: "English", "Deutsch"
};

// English first, then the built-in and installed translations (by code).
std::vector<LanguageInfo> languages();

// Switches the language ("en": as written). False (why) for a code with no translation;
// the language then stays as it was.
bool setLanguage(const std::string& code, std::string* why = nullptr);
const std::string& language();

// The translation of a piece of the interface, else the English. In an ImGui label the part
// from "##" on (its hidden id) is kept as it is; spaces around the words too. The pointer
// stays valid for the life of the program, so it can be passed straight to ImGui.
const char* tr(const char* english);
std::string tr(const std::string& english);

// The words looked up since the language was set that it has no translation for, sorted:
// what a translator still has to do.
std::vector<std::string> untranslated();

// Where installed language files are: $SICDM_LANG_DIR, else configDirectory()/lang; or
// the folder given here ("" goes back to that).
std::string languageDirectory();
void setLanguageDirectory(const std::string& dir);

// For tests: the built-in table of a language (pairs of English and translated), or empty.
std::vector<std::pair<std::string, std::string>> builtinTranslation(const std::string& code);

} // namespace sicdm

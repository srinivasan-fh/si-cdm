#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Json.hpp"
#include "core/Keymap.hpp"
#include "core/Preferences.hpp"

#include <algorithm>

using namespace sicdm;

#define REQUIRE(c)                  \
    do {                            \
        const bool required_ = (c); \
        CHECK(required_);           \
        if (!required_) return;     \
    } while (0)

namespace {

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }

} // namespace

TEST_CASE("Keymap: chords read and written back, the defaults, refusals") {
    auto c = parseChords("ctrl+shift+z, Ctrl+Y");
    REQUIRE(c.has_value());
    REQUIRE(c->size() == 2);
    CHECK((*c)[0].key == "Z");
    CHECK((*c)[0].ctrl && (*c)[0].shift && !(*c)[0].alt);
    CHECK(chordsText(*c) == "Ctrl+Shift+Z, Ctrl+Y");
    CHECK(chordText(parseChords("Cmd+,")->front()) == "Ctrl+,"); // Cmd is Ctrl; a lone comma is the key
    CHECK(chordText(parseChords("option+num1")->front()) == "Alt+Num1");
    CHECK(chordText(parseChords("del")->front()) == "Delete");
    CHECK(parseChords("none")->empty());
    CHECK(parseChords("")->empty());
    CHECK(!parseChords("Ctrl+Nope"));
    CHECK(!parseChords("Hyper+A"));
    CHECK(!parseChords("Escape")); // kept for cancelling
    // Every default parses, and no two actions share one.
    Preferences p;
    for (const KeyAction& a : keyActions()) {
        CHECK(parseChords(a.chords).has_value());
        for (const KeyChord& k : boundChords(p, a.id)) CHECK(actionsOnChord(p, k, a.id).empty());
    }
    CHECK(toolKeyId("Block / Cuboid") == "tool.block-cuboid");
    CHECK(toolKeyId("Move") == "tool.move");
}

TEST_CASE("Keymap: setting, sharing and resetting a shortcut; saved with the preferences") {
    Preferences p;
    setToolKeyDefaults({{"tool.pan", "H"}, {"tool.move", "V"}});
    std::string why;
    // Hide on H: Pan has it, so it is refused unless shared on purpose.
    CHECK(!setKeyBinding(p, "edit.hide", "H", false, &why));
    CHECK(why.find("tool.pan") != std::string::npos);
    REQUIRE(setKeyBinding(p, "edit.hide", "Ctrl+H", false, &why));
    CHECK(chordsText(boundChords(p, "edit.hide")) == "Ctrl+H");
    REQUIRE(setKeyBinding(p, "edit.hide", "H", true, &why)); // FORCE: both on H
    CHECK(actionsOnChord(p, parseChords("H")->front(), "edit.hide") == std::vector<std::string>{"tool.pan"});
    // A tool moved to another key.
    REQUIRE(setKeyBinding(p, "tool.move", "M", true, &why));
    CHECK(chordsText(boundChords(p, "tool.move")) == "M");
    // Set back to the default: no longer kept as a change.
    REQUIRE(setKeyBinding(p, "edit.copy", "Ctrl+C", false, &why));
    CHECK(p.keys.count("edit.copy") == 0);
    // No keys at all.
    REQUIRE(setKeyBinding(p, "view.grid", "none", false, &why));
    CHECK(boundChords(p, "view.grid").empty());
    CHECK(!setKeyBinding(p, "no.such", "F5", false, &why));
    // Saved and read back; a hand-broken entry is left out.
    Json j = preferencesToJson(p);
    Json keys = j["keys"];
    keys.set("edit.paste", "Ctrl+Nonsense");
    j.set("keys", keys);
    const Preferences back = preferencesFromJson(j);
    CHECK(back.keys.at("edit.hide") == "H");
    CHECK(back.keys.at("tool.move") == "M");
    CHECK(back.keys.at("view.grid") == "none");
    CHECK(back.keys.count("edit.paste") == 0);
    resetKeyBinding(p, "edit.hide");
    CHECK(chordsText(boundChords(p, "edit.hide")) == "Alt+H");
    resetKeyBinding(p, "");
    CHECK(p.keys.empty());
    setToolKeyDefaults({});
}

TEST_CASE("KEYS: list, set, reset") {
    Document d;
    REQUIRE(run(d, "KEYS RESET ALL").ok);
    const CommandOutput list = run(d, "KEYS");
    REQUIRE(list.ok);
    CHECK(std::any_of(list.lines.begin(), list.lines.end(), [](const std::string& l) { return l.find("edit.copy  Ctrl+C") != std::string::npos; }));
    REQUIRE(run(d, "KEYS SET edit.redo \"Ctrl+Shift+Z, Ctrl+Y, F12\"").ok);
    CHECK(chordsText(boundChords(preferences(), "edit.redo")) == "Ctrl+Shift+Z, Ctrl+Y, F12");
    CHECK(!run(d, "KEYS SET edit.redo Ctrl+C").ok); // Copy's
    REQUIRE(run(d, "KEYS SET edit.redo Ctrl+C FORCE").ok);
    CHECK(!run(d, "KEYS SET edit.redo Ctrl+Blah").ok);
    REQUIRE(run(d, "KEYS RESET edit.redo").ok);
    CHECK(chordsText(boundChords(preferences(), "edit.redo")) == "Ctrl+Shift+Z, Ctrl+Y");
    CHECK(!run(d, "KEYS RESET no.such").ok);
    REQUIRE(run(d, "KEYS RESET ALL").ok);
    CHECK(preferences().keys.empty());
}

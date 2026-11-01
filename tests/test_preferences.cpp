#include "TestFramework.hpp"

#include "core/Camera.hpp"
#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Json.hpp"
#include "core/Licence.hpp"
#include "core/Plugin.hpp"
#include "core/Preferences.hpp"
#include "core/Translate.hpp"
#include "core/UpdateCheck.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

using namespace sicdm;
namespace fs = std::filesystem;

#define REQUIRE(c)                  \
    do {                            \
        const bool required_ = (c); \
        CHECK(required_);           \
        if (!required_) return;     \
    } while (0)

namespace {

fs::path freshDir(const char* name) {
    const fs::path d = fs::temp_directory_path() / "sicdm_tests" / name;
    std::error_code ec;
    fs::remove_all(d, ec);
    fs::create_directories(d);
    return d;
}

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }

std::string lastLine(const CommandOutput& o) { return o.lines.empty() ? std::string() : o.lines.back(); }

std::string fileUrl(const fs::path& p) {
    std::string s = fs::absolute(p).generic_string();
    return s[0] == '/' ? "file://" + s : "file:///" + s;
}

// Starts each case with empty preferences in the file the tests use.
void freshPreferences() {
    std::error_code ec;
    fs::remove(preferencesPath(), ec);
    reloadPreferences();
}

} // namespace

TEST_CASE("Preferences: values are checked, saved and read back; a bad file keeps the defaults") {
    freshPreferences();
    Preferences p;
    std::string err;
    CHECK(setPreference(p, "units", "inches", &err));
    CHECK(p.units == "in");
    CHECK(setPreference(p, "autosave", "120", &err));
    CHECK(p.autosaveSeconds == 120);
    CHECK(setPreference(p, "autosave", "off", &err));
    CHECK(p.autosaveSeconds == 0);
    CHECK(!setPreference(p, "autosave", "2", &err)); // too often
    CHECK(setPreference(p, "material", "steel", &err));
    CHECK(!setPreference(p, "material", "cheese", &err));
    CHECK(setPreference(p, "post", "HAAS", &err));
    CHECK(p.camPost == "haas");
    CHECK(setPreference(p, "machine", "VMC", &err));
    CHECK(p.camMachine == "vmc");
    CHECK(!setPreference(p, "update-url", "ftp://x", &err));
    CHECK(setPreference(p, "update-url", "https://example.com/sicdm/latest.json", &err));
    CHECK(setPreference(p, "updates", "on", &err));
    CHECK(!setPreference(p, "colour", "blue", &err));
    CHECK(p.rulers); // on unless turned off
    CHECK(setPreference(p, "rulers", "off", &err) && !p.rulers);
    CHECK(!setPreference(p, "rulers", "maybe", &err));
    CHECK(p.reopenLast); // on unless turned off
    CHECK(setPreference(p, "reopen", "off", &err) && !p.reopenLast);
    CHECK(!setPreference(p, "reopen", "later", &err));
    p.recent = {"/a/one.sicdm", "/b/two.sicdm"};
    p.session = {"/a/one.sicdm", "/c/three.sicdm"};
    p.sessionActive = 1;

    const Preferences q = preferencesFromJson(preferencesToJson(p));
    CHECK(q.recent == p.recent);
    CHECK(q.units == "in" && q.autosaveSeconds == 0 && q.camMaterial == p.camMaterial && q.camPost == "haas");
    CHECK(q.camMachine == "vmc" && q.checkUpdates && q.updateUrl == p.updateUrl);
    CHECK(!q.rulers); // saved off, read back off
    CHECK(!q.reopenLast && q.session == p.session && q.sessionActive == 1 && !q.sessionOpening);

    // A hand-edited file with values the app would refuse: those keep their defaults.
    const auto j = Json::parse(R"({"units":"furlongs","autosave_seconds":"soon","cam_post":"rm -rf","update_url":"javascript:x","recent":[1,"/ok.sicdm"]})");
    REQUIRE(j.has_value());
    const Preferences r = preferencesFromJson(*j);
    CHECK(r.units.empty() && r.autosaveSeconds == 60 && r.camPost.empty() && r.updateUrl.empty());
    CHECK(r.recent.size() == 1 && r.recent[0] == "/ok.sicdm");

    // PREFERENCES SET writes the file; another process reads it.
    Document d;
    CHECK(run(d, "PREFERENCES SET units mm").ok);
    CHECK(run(d, "PREFERENCES SET autosave 30").ok);
    CHECK(!run(d, "PREFERENCES SET autosave never").ok);
    CHECK(fs::exists(preferencesPath()));
    reloadPreferences();
    CHECK(preferences().units == "mm" && preferences().autosaveSeconds == 30);
    const CommandOutput shown = run(d, "PREFS");
    CHECK(shown.ok && shown.lines.size() == 32);
    // The language: checked, shown at once, kept, and back to English on RESET.
    CHECK(!run(d, "PREFERENCES SET language klingon").ok);
    CHECK(run(d, "PREFERENCES SET language Deutsch").ok);
    CHECK(language() == "de");
    reloadPreferences();
    CHECK(preferences().language == "de");
    CHECK(run(d, "PREFERENCES LANGUAGES").lines.size() >= 2);
    CHECK(run(d, "PREFERENCES RESET").ok);
    CHECK(language() == "en");
    CHECK(preferences().units.empty() && preferences().autosaveSeconds == 60);
    freshPreferences();
}

TEST_CASE("Preferences: units for new projects apply on NEW") {
    freshPreferences();
    Document d;
    CHECK(run(d, "NEW").ok);
    CHECK(d.state.units.unit == LengthUnit::Millimeter); // no preference: as it was
    CHECK(run(d, "UNITS in").ok);
    CHECK(run(d, "NEW").ok);
    CHECK(d.state.units.unit == LengthUnit::Inch);
    CHECK(run(d, "PREFERENCES SET units cm").ok);
    CHECK(run(d, "NEW").ok);
    CHECK(d.state.units.unit == LengthUnit::Centimeter);
    freshPreferences();
}

TEST_CASE("Preferences: projects opened and saved go to the top of the recent list") {
    freshPreferences();
    const fs::path dir = freshDir("recent");
    Document d;
    CHECK(run(d, "RECENT").ok);
    CHECK(lastLine(run(d, "RECENT")) == "No recent files");
    CHECK(!run(d, "RECENT 1").ok);
    REQUIRE(run(d, "BOX 10 10 10").ok);
    for (int i = 0; i < 12; ++i) REQUIRE(run(d, "SAVE \"" + (dir / ("p" + std::to_string(i) + ".sicdm")).string() + "\"").ok);
    std::vector<std::string> r = preferences().recent;
    REQUIRE(r.size() == Preferences::kMaxRecent);
    CHECK(fs::path(r[0]).filename() == "p11.sicdm");
    CHECK(fs::path(r[9]).filename() == "p2.sicdm");

    // Opening one moves it to the top, once.
    REQUIRE(run(d, "OPEN \"" + (dir / "p5.sicdm").string() + "\"").ok);
    r = preferences().recent;
    CHECK(fs::path(r[0]).filename() == "p5.sicdm" && fs::path(r[1]).filename() == "p11.sicdm" && r.size() == 10);

    // RECENT n opens; a file that is gone is marked and not opened.
    Document e;
    CHECK(run(e, "RECENT 2").ok);
    CHECK(fs::path(e.path).filename() == "p11.sicdm");
    CHECK(fs::path(preferences().recent[0]).filename() == "p11.sicdm"); // opened: on top again
    fs::remove(dir / "p11.sicdm");
    const CommandOutput listed = run(e, "RECENT");
    CHECK(listed.lines.size() == 10 && listed.lines[0].find("(missing)") != std::string::npos);
    CHECK(!run(e, "RECENT 1").ok);
    CHECK(!run(e, "RECENT 11").ok);

    // It is kept between sessions.
    reloadPreferences();
    CHECK(preferences().recent.size() == 10);
    CHECK(run(e, "RECENT CLEAR").ok);
    reloadPreferences();
    CHECK(preferences().recent.empty());
    freshPreferences();
}

TEST_CASE("Preferences: navigation, view, modelling, files, graphics and AI settings are checked and kept") {
    Preferences p;
    std::string err;
    // The defaults are how the app behaved before they were settings.
    CHECK(!p.zoomInvert && p.zoomSpeed == 1.0f && p.orbitSpeed == 1.0f && p.zoomToCursor && p.animateViews);
    CHECK(p.displayMode == "shaded-edges" && p.perspective && p.grid && p.background == "dark" && p.viewSphere);
    CHECK(p.snapGrid && p.snapPoints && p.moveStep == 1.0f && p.angleStep == 15.0f);
    CHECK(!p.backup && p.projectFolder.empty() && p.splash && p.antialiasing == 4 && p.vsync && p.mcpPort == 0);

    for (const char* key : {"zoom-invert", "animate-views", "perspective", "grid", "view-sphere", "snap-grid", "snap-points", "backup", "splash", "vsync"}) {
        CHECK(setPreference(p, key, "off", &err));
        CHECK(!setPreference(p, key, "sometimes", &err));
    }
    CHECK(setPreference(p, "zoom_invert", "on", &err) && p.zoomInvert); // _ or -
    CHECK(setPreference(p, "zoom-to-cursor", "off", &err) && !p.zoomToCursor);
    CHECK(setPreference(p, "zoom-speed", "2.5", &err) && p.zoomSpeed == 2.5f);
    CHECK(!setPreference(p, "zoom-speed", "10", &err) && !setPreference(p, "orbit-speed", "0.1", &err) && !setPreference(p, "orbit-speed", "fast", &err));
    CHECK(setPreference(p, "orbit-speed", "0.5", &err) && p.orbitSpeed == 0.5f);
    CHECK(setPreference(p, "display", "Wireframe", &err) && p.displayMode == "wireframe");
    CHECK(!setPreference(p, "display", "x-ray", &err));
    CHECK(setPreference(p, "background", "light", &err) && p.background == "light");
    CHECK(!setPreference(p, "background", "purple", &err));
    CHECK(setPreference(p, "move-step", "0.5", &err) && p.moveStep == 0.5f);
    CHECK(!setPreference(p, "move-step", "0", &err) && !setPreference(p, "angle-step", "200", &err));
    CHECK(setPreference(p, "angle-step", "5", &err) && p.angleStep == 5.0f);
    CHECK(setPreference(p, "antialiasing", "8", &err) && p.antialiasing == 8);
    CHECK(setPreference(p, "antialiasing", "off", &err) && p.antialiasing == 0);
    CHECK(!setPreference(p, "antialiasing", "3", &err));
    CHECK(setPreference(p, "mcp-port", "8765", &err) && p.mcpPort == 8765);
    CHECK(!setPreference(p, "mcp-port", "80", &err) && !setPreference(p, "mcp-port", "70000", &err) && !setPreference(p, "mcp-port", "8765.5", &err));
    CHECK(setPreference(p, "project-folder", "/work/parts", &err) && p.projectFolder == "/work/parts");

    const Preferences q = preferencesFromJson(preferencesToJson(p));
    CHECK(q.zoomInvert && q.zoomSpeed == 2.5f && q.orbitSpeed == 0.5f && !q.zoomToCursor && !q.animateViews);
    CHECK(q.displayMode == "wireframe" && !q.perspective && !q.grid && q.background == "light" && !q.viewSphere);
    CHECK(!q.snapGrid && !q.snapPoints && q.moveStep == 0.5f && q.angleStep == 5.0f);
    CHECK(!q.backup && q.projectFolder == "/work/parts" && !q.splash && q.antialiasing == 0 && !q.vsync && q.mcpPort == 8765);

    // The window's place and the side panel come back; a silly size isn't kept.
    p.windowX = 120, p.windowY = 80, p.windowWidth = 1400, p.windowHeight = 900, p.windowMaximized = true;
    p.modelPanel = true, p.panelPage = 2;
    const Preferences w = preferencesFromJson(preferencesToJson(p));
    CHECK(w.windowX == 120 && w.windowY == 80 && w.windowWidth == 1400 && w.windowHeight == 900 && w.windowMaximized);
    CHECK(w.modelPanel && w.panelPage == 2);
    const auto tiny = Json::parse(R"({"window":{"x":0,"y":0,"width":5,"height":5},"panel_page":9})");
    REQUIRE(tiny.has_value());
    CHECK(preferencesFromJson(*tiny).windowWidth == 0 && preferencesFromJson(*tiny).panelPage == 2);

    // A hand-edited file can't hold what PREFERENCES SET would refuse.
    const auto j = Json::parse(R"({"zoom_speed":99,"antialiasing":5,"mcp_port":22,"display":"x-ray","background":"plaid","move_step":-1})");
    REQUIRE(j.has_value());
    const Preferences r = preferencesFromJson(*j);
    CHECK(r.zoomSpeed == 1.0f && r.antialiasing == 4 && r.mcpPort == 0 && r.displayMode == "shaded-edges" && r.background == "dark" && r.moveStep == 1.0f);

    // Each background has its colours; the dark one is the window's own.
    for (const std::string& b : backgroundNames()) CHECK(backgroundColours(b).top[0] >= backgroundColours(b).bottom[0]);
    CHECK(backgroundColours("dark").top[2] == 0.29f);
}

TEST_CASE("Preferences: BACKUP keeps the file a save replaces; views jump with ANIMATE-VIEWS off") {
    freshPreferences();
    const fs::path dir = freshDir("backup");
    const std::string file = (dir / "part.sicdm").string();
    Document d;
    REQUIRE(run(d, "BOX 10 10 10").ok);
    REQUIRE(run(d, "SAVE \"" + file + "\"").ok);
    CHECK(!fs::exists(file + ".bak")); // off by default
    CHECK(run(d, "PREFERENCES SET backup on").ok);
    REQUIRE(run(d, "SAVE \"" + file + "\"").ok); // the first save's file is kept
    CHECK(fs::exists(file + ".bak"));
    REQUIRE(run(d, "BOX 5 5 5").ok);
    const std::uintmax_t before = fs::file_size(file);
    REQUIRE(run(d, "SAVE").ok);
    CHECK(fs::file_size(file + ".bak") == before); // the one just replaced
    CHECK(fs::file_size(file) > before);

    OrbitCamera cam;
    OrbitCamera::setAnimations(false);
    cam.setStandardView(StandardView::Top);
    CHECK(!cam.update(0.0f)); // already there
    CHECK(cam.pitch() > 1.5f);
    OrbitCamera::setAnimations(true);
    cam.setStandardView(StandardView::Front);
    CHECK(cam.update(0.001f)); // gliding
    freshPreferences();
}

TEST_CASE("Preferences: start-up reopens the projects open at the last close") {
    const fs::path dir = freshDir("session");
    for (const char* n : {"a.sicdm", "b.sicdm", "c.sicdm"}) std::ofstream(dir / n) << "{}";
    const std::string a = (dir / "a.sicdm").string(), b = (dir / "b.sicdm").string(), c = (dir / "c.sicdm").string();
    const std::string gone = (dir / "gone.sicdm").string();

    Preferences p;
    CHECK(startupSession(p).files.empty()); // nothing open, nothing recent: the demo part

    // The open projects, in order, with the one that was in front.
    p.session = {a, gone, c};
    p.sessionActive = 2;
    StartupSession s = startupSession(p);
    CHECK(s.files.size() == 2 && s.files[0] == a && s.files[1] == c); // a moved or deleted one is left out
    CHECK(s.active == 1 && !s.crashedLastTime);

    // None open (only untitled work): the most recent project that is still there.
    p.session.clear();
    p.recent = {gone, b, a};
    s = startupSession(p);
    CHECK(s.files.size() == 1 && s.files[0] == b && s.active == 0);

    // The app closed while reopening them last time: not tried again.
    p.session = {a};
    p.sessionOpening = true;
    s = startupSession(p);
    CHECK(s.crashedLastTime && s.files.empty());

    // Kept between sessions, and RESET keeps them (they aren't a setting).
    freshPreferences();
    preferences().session = {a, b};
    preferences().sessionActive = 1;
    CHECK(savePreferences());
    reloadPreferences();
    CHECK(preferences().session.size() == 2 && preferences().sessionActive == 1);
    Document d;
    CHECK(run(d, "PREFERENCES SET reopen off").ok);
    CHECK(!preferences().reopenLast);
    CHECK(run(d, "PREFERENCES RESET").ok);
    CHECK(preferences().reopenLast && preferences().session.size() == 2);
    freshPreferences();
}

TEST_CASE("Update check: versions compare as numbers, feeds are read, UPDATES reports") {
    CHECK(compareVersions("1.10.0", "1.9.9") > 0);
    CHECK(compareVersions("1.2", "1.2.0") == 0);
    CHECK(compareVersions("v2.0.0", "1.99") > 0);
    CHECK(compareVersions("1.3.0-rc1", "1.3.0") < 0);
    CHECK(compareVersions("1.3.0", "1.3.0-rc2") > 0);
    CHECK(compareVersions("0.9.0", "0.10.0") < 0);

    ReleaseInfo info;
    std::string err;
    CHECK(!parseReleaseFeed("not json", info, &err));
    CHECK(!parseReleaseFeed(R"({"notes":"no version"})", info, &err));
    CHECK(parseReleaseFeed(R"({"version":"9.1.0","url":"https://example.com/dl","downloads":{"windows":"https://example.com/w.exe","macos":"https://example.com/m.dmg","linux":"https://example.com/l.tgz"}})", info, &err));
    CHECK(info.version == "9.1.0");
#if defined(_WIN32)
    CHECK(info.download == "https://example.com/w.exe");
#elif defined(__APPLE__)
    CHECK(info.download == "https://example.com/m.dmg");
#else
    CHECK(info.download == "https://example.com/l.tgz");
#endif

    const fs::path dir = freshDir("updates");
    {
        std::ofstream(dir / "newer.json") << R"({"version":"999.0.0","date":"2030-01-01","notes":"Everything is faster","url":"https://example.com/sicdm"})";
        std::ofstream(dir / "older.json") << R"({"version":"0.0.1"})";
        std::ofstream(dir / "bad.json") << "<html>";
    }
    const UpdateResult newer = checkForUpdate(fileUrl(dir / "newer.json"), SICDM_VERSION);
    CHECK(newer.ok && newer.newer && newer.latest.version == "999.0.0");
    CHECK(newer.describe(SICDM_VERSION).find("999.0.0 is available") != std::string::npos);
    const UpdateResult older = checkForUpdate(fileUrl(dir / "older.json"), SICDM_VERSION);
    CHECK(older.ok && !older.newer);
    CHECK(!checkForUpdate(fileUrl(dir / "bad.json"), SICDM_VERSION).ok);
    CHECK(!checkForUpdate(fileUrl(dir / "missing.json"), SICDM_VERSION).ok);
    CHECK(!checkForUpdate("", SICDM_VERSION).ok);
    std::string body;
    CHECK(!fetchUrl("https://example.com/a b", body, &err)); // no spaces reach curl
    CHECK(!fetchUrl("gopher://x", body, &err));

    freshPreferences();
    Document d;
    CHECK(!run(d, "UPDATES").ok); // no feed set
    CHECK(run(d, "PREFERENCES SET update-url \"" + fileUrl(dir / "newer.json") + "\"").ok);
    const CommandOutput o = run(d, "UPDATES CHECK");
    CHECK(o.ok && o.lines.size() == 2 && o.lines[1].find("Everything is faster") != std::string::npos);
    CHECK(run(d, "UPDATES CHECK \"" + fileUrl(dir / "older.json") + "\"").ok);
    freshPreferences();
}

TEST_CASE("Preferences: programs run without a shell, and the licence sits in the configuration folder") {
#if !defined(_WIN32)
    std::string out, err;
    // The argument would run a second command if a shell read it.
    CHECK(runProgram({"echo", "a; echo injected"}, 10.0, out, &err) == 0);
    CHECK(out == "a; echo injected\n");
    CHECK(runProgram({"false"}, 10.0, out, &err) != 0);
    CHECK(runProgram({"sicdm-no-such-program"}, 10.0, out, &err) == -1);
    CHECK(runProgram({"sleep", "5"}, 0.3, out, &err) == -1);
#endif
    const char* had = std::getenv("SICDM_LICENCE");
    if (!had) CHECK(fs::path(licencePath()).parent_path() == fs::path(configDirectory()));
}

#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/FileIO.hpp"
#include "core/Plugin.hpp"
#include "core/Scene.hpp"

#include <chrono>
#include <cstdlib>
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

namespace {

bool has(const std::string& s, const std::string& what) { return s.find(what) != std::string::npos; }

std::string text(const CommandOutput& r) {
    std::string s;
    for (const std::string& l : r.lines) s += l + "\n";
    return s;
}

void setEnv(const char* name, const std::string& value) {
#if defined(_WIN32)
    _putenv_s(name, value.c_str());
#else
    setenv(name, value.c_str(), 1);
#endif
}

// A JSON string for a path (backslashes on Windows).
std::string jsonPath(const std::string& p) {
    std::string s;
    for (char c : p) s += c == '\\' ? std::string("\\\\") : std::string(1, c);
    return s;
}

// A plugin folder whose commands run the sample plugin.
std::filesystem::path makePlugins() {
    const std::filesystem::path root = std::filesystem::temp_directory_path() / "sicdm plugin test";
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(root / "sample");
    const std::string exe = jsonPath(SICDM_SAMPLE_PLUGIN);
    std::ofstream(root / "sample" / "plugin.json") << R"({
  "name": "sample", "version": "0.1", "description": "The test plugin",
  "commands": [
    {"name": "SampleBox", "usage": "SAMPLEBOX size", "help": "A box, through run_command", "run": [")" + exe + R"(", "box"]},
    {"name": "SAMPLEFAIL", "run": [")" + exe + R"(", "fail"]},
    {"name": "SAMPLEECHO", "run": [")" + exe + R"(", "echo"]},
    {"name": "BOX", "run": [")" + exe + R"(", "box"]}
  ],
  "menu": [{"label": "Sample Box", "command": "SAMPLEBOX 10"}]
})";
    std::filesystem::create_directories(root / "slow");
    std::ofstream(root / "slow" / "plugin.json") << R"({"name": "slow", "timeout": 1, "commands": [{"name": "SAMPLEHANG", "run": [")" + exe + R"(", "hang"]}]})";
    std::filesystem::create_directories(root / "broken");
    std::ofstream(root / "broken" / "plugin.json") << R"({"name": "broken", "commands": [{"name": "NOPROGRAM", "run": ["sicdm-no-such-program-anywhere"]}]})";
    setEnv("SICDM_PLUGIN_PATH", root.string());
    reloadPlugins();
    return root;
}

} // namespace

TEST_CASE("Plugins: found from their manifests, listed, their commands in HELP; built-in names refused") {
    const auto root = makePlugins();
    const Plugin* owner = nullptr;
    const PluginCommand* c = findPluginCommand("samplebox", &owner);
    REQUIRE(c != nullptr);
    CHECK(c->name == "SAMPLEBOX");
    CHECK(owner->name == "sample");
    CHECK(owner->version == "0.1");
    REQUIRE(owner->menu.size() == 1);
    CHECK(owner->menu[0].command == "SAMPLEBOX 10");
    CHECK(has(owner->error, "BOX is built in"));
    CHECK(findPluginCommand("BOX") == nullptr);
    Document d;
    const std::string list = text(executeCommand(d, "PLUGINS", nullptr));
    CHECK(has(list, "sample 0.1: The test plugin"));
    CHECK(has(list, "SAMPLEBOX size   A box, through run_command"));
    CHECK(has(list, "not loaded in full: the command BOX is built in"));
    CHECK(has(text(executeCommand(d, "HELP SAMPLEBOX", nullptr)), "(plugin sample)"));
    CHECK(has(text(executeCommand(d, "HELP", nullptr)), "SAMPLEBOX size"));
    std::filesystem::remove_all(root);
    reloadPlugins();
}

TEST_CASE("Plugins: a command runs the program, which drives the document over JSON-RPC") {
    const auto root = makePlugins();
    Document d;
    auto r = executeCommand(d, "SAMPLEBOX 12", nullptr);
    CHECK(r.ok);
    const std::string t = text(r);
    CHECK(has(t, "made a 12 mm box"));
    CHECK(has(t, "scene reply has the block"));
    CHECK(has(t, "unknown tools are errors"));
    REQUIRE(d.scene.bodies().size() == 1);
    const AABB b = d.scene.bodies()[0].bounds();
    CHECK_NEAR(b.max.x - b.min.x, 12.0, 1e-4);
    // The feature history holds what the plugin did, so REGEN rebuilds it without the plugin.
    bool boxStep = false, pluginStep = false;
    for (const HistoryStep& s : d.state.history) boxStep = boxStep || s.line.rfind("BOX 12", 0) == 0, pluginStep = pluginStep || s.line.rfind("SAMPLEBOX", 0) == 0;
    CHECK(boxStep);
    CHECK(!pluginStep);
    // Arguments, quoted ones whole; the plugin's folder in its environment.
    r = executeCommand(d, "SAMPLEECHO one \"two words\"", nullptr);
    CHECK(r.ok);
    CHECK(has(text(r), "arg: one\narg: two words\n"));
    CHECK(has(text(r), "dir: " + (root / "sample").string()));
    // Failures: the exit code and what it wrote to its standard error.
    r = executeCommand(d, "SAMPLEFAIL", nullptr);
    CHECK(!r.ok);
    CHECK(has(text(r), "failed (exit code 3)"));
    CHECK(has(text(r), "the sample plugin can't do that"));
    r = executeCommand(d, "NOPROGRAM", nullptr);
    CHECK(!r.ok);
    // Too slow: stopped at the manifest's timeout.
    const auto t0 = std::chrono::steady_clock::now();
    r = executeCommand(d, "SAMPLEHANG", nullptr);
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    CHECK(!r.ok);
    CHECK(has(text(r), "took longer than 1 s"));
    CHECK(secs < 8.0);
    std::filesystem::remove_all(root);
    reloadPlugins();
}

TEST_CASE("Plugins: the example gear plugin (Python) makes a spur gear") {
#if defined(_WIN32)
    const bool python = std::system("python --version > NUL 2>&1") == 0;
#else
    const bool python = std::system("python3 --version > /dev/null 2>&1") == 0;
#endif
    if (!python) return; // the example needs Python; the plugin host is tested above
    setEnv("SICDM_PLUGIN_PATH", std::string(SICDM_SOURCE_DIR) + "/docs/examples/plugins");
    reloadPlugins();
    Document d;
    const auto r = executeCommand(d, "SPURGEAR 12 2 5", nullptr);
    CHECK(r.ok);
    CHECK(has(text(r), "Spur gear: 12 teeth, module 2: pitch 24.00 mm, tips 28.00 mm"));
    REQUIRE(d.scene.bodies().size() == 1);
    const AABB b = d.scene.bodies()[0].bounds();
    CHECK(b.max.x - b.min.x > 27.0);
    CHECK(b.max.x - b.min.x < 28.01);
    CHECK_NEAR(b.max.z - b.min.z, 5.0, 1e-4);
    setEnv("SICDM_PLUGIN_PATH", "");
    reloadPlugins();
}

#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/FileIO.hpp"
#include "core/Json.hpp"
#include "core/Recovery.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
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

std::string slurp(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

std::size_t filesIn(const fs::path& d) {
    std::size_t n = 0;
    for (const auto& e : fs::directory_iterator(d)) n += e.is_regular_file();
    return n;
}

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }

void setEnv(const char* k, const std::string& v) {
#if defined(_WIN32)
    _putenv_s(k, v.c_str());
#else
    setenv(k, v.c_str(), 1);
#endif
}

} // namespace

TEST_CASE("Files are written whole or not at all, with nothing left beside them") {
    const fs::path d = freshDir("atomic");
    const std::string path = (d / "a.txt").string();
    std::string err;
    REQUIRE(writeFile(path, "first", &err));
    REQUIRE(writeFile(path, "second", &err));
    CHECK(slurp(path) == "second");
    CHECK(filesIn(d) == 1); // no temporary left over
    // A write that can't happen (the folder is a file) fails and leaves the old file.
    const std::string blocked = (d / "a.txt" / "b.txt").string();
    CHECK(!writeFile(blocked, "x", &err));
    CHECK(!err.empty());
    CHECK(slurp(path) == "second");
}

TEST_CASE("Project format versions: newer files open when they can, and say what was left out") {
    const fs::path d = freshDir("versions");
    Document doc;
    REQUIRE(run(doc, "BOX 10 10 10").ok);
    const std::string path = (d / "p.sicdm").string();
    std::string err, warn;
    REQUIRE(saveProject(doc.scene, doc.state, path, &err));
    auto j = Json::parse(slurp(path));
    REQUIRE(j.has_value());
    CHECK((*j)["version"].asInt() == kProjectVersion);
    CHECK((*j)["reader_version"].asInt() == 1);
    CHECK((*j)["saved_by"].asString("").rfind("SI CDM ", 0) == 0);
    // A later format that only adds things: opens, with a note.
    j->set("version", kProjectVersion + 1);
    j->set("saved_by", "SI CDM 9.0");
    j->set("something_new", 1);
    REQUIRE(writeFile(path, j->dump(), &err));
    Scene s;
    ProjectState st;
    REQUIRE(loadProject(s, st, path, &err, &warn));
    CHECK(s.bodies().size() == 1);
    CHECK(warn.find("SI CDM 9.0") != std::string::npos);
    // A later format an older reader can't follow: refused, saying why.
    j->set("reader_version", kProjectVersion + 1);
    REQUIRE(writeFile(path, j->dump(), &err));
    err.clear();
    CHECK(!loadProject(s, st, path, &err, &warn));
    CHECK(err.find("Update SI CDM") != std::string::npos);
    // A format-1 file (no reader_version) still opens.
    j = Json::parse(slurp(path));
    j->set("version", 1);
    j->set("reader_version", Json());
    REQUIRE(writeFile(path, j->dump(), &err));
    CHECK(loadProject(s, st, path, &err, &warn));
}

TEST_CASE("Autosave keeps a copy of unsaved work once it is a little old, and drops it on save") {
    const fs::path d = freshDir("autosave");
    Autosaver a(d.string(), 10.0);
    Document doc;
    REQUIRE(run(doc, "BOX 10 20 30").ok);
    doc.dirty = true;
    CHECK(!a.update(doc, 1, 111, 0.0));
    CHECK(!a.update(doc, 1, 111, 9.0)); // not old enough yet
    CHECK(a.update(doc, 1, 111, 10.0));
    CHECK(fs::exists(a.projectPath(1)));
    CHECK(!a.update(doc, 1, 111, 50.0)); // nothing new
    // More changes: written again an interval after they were first seen.
    CHECK(!a.update(doc, 1, 222, 60.0));
    CHECK(!a.update(doc, 1, 333, 65.0));
    CHECK(a.update(doc, 1, 333, 70.0));
    // A copy that loads back as the document.
    Scene s;
    ProjectState st;
    std::string err;
    REQUIRE(loadProject(s, st, a.projectPath(1), &err));
    CHECK(s.bodies().size() == 1);
    // This session is alive: its copies aren't offered for recovery.
    CHECK(findRecoveryFiles(d.string()).empty());
    // Saved: the copy goes.
    doc.dirty = false;
    a.update(doc, 1, 333, 80.0);
    CHECK(!fs::exists(a.projectPath(1)));
    CHECK(filesIn(d) == 0);
    // Closed, or a clean exit: gone too.
    doc.dirty = true;
    a.update(doc, 2, 5, 0.0);
    CHECK(a.update(doc, 2, 5, 10.0));
    a.update(doc, 3, 6, 0.0);
    CHECK(a.update(doc, 3, 6, 10.0));
    a.forget(2);
    CHECK(filesIn(d) == 2);
    a.clear();
    CHECK(filesIn(d) == 0);
}

TEST_CASE("RECOVER reopens the unsaved work a crashed session left, still unsaved, then discards") {
    const fs::path d = freshDir("recover");
    setEnv("SICDM_RECOVERY_DIR", d.string());
    {
        // A session that crashed: its copies stay, under a process id that isn't running.
        Autosaver a(d.string(), 0.0);
        Document doc;
        REQUIRE(run(doc, "BOX 10 20 30").ok);
        REQUIRE(run(doc, "SPHERE 5 @40,0,0").ok);
        doc.path = (d / "bracket.sicdm").string();
        doc.dirty = true;
        REQUIRE(a.update(doc, 7, 1, 0.0));
        Document other;
        REQUIRE(run(other, "CYLINDER 4 9").ok);
        other.name = "Untitled 3";
        other.dirty = true;
        REQUIRE(a.update(other, 8, 1, 0.0));
        for (int id : {7, 8}) {
            fs::path p(a.projectPath(id)), m = p;
            m.replace_extension(".json");
            const std::string dead = "2147483646-" + std::to_string(id);
            fs::rename(p, d / (dead + ".sicdm"));
            fs::rename(m, d / (dead + ".json"));
        }
    }
    REQUIRE(findRecoveryFiles(d.string()).size() == 2);
    Document doc;
    auto r = run(doc, "RECOVER");
    REQUIRE(r.ok);
    std::string all;
    for (const std::string& l : r.lines) all += l + "\n";
    CHECK(all.find("bracket.sicdm") != std::string::npos);
    CHECK(all.find("Untitled 3") != std::string::npos);
    // Reopen the bracket: its bodies, its file, and still unsaved.
    const auto files = findRecoveryFiles(d.string());
    const int which = files[0].name == "bracket.sicdm" ? 1 : 2;
    r = run(doc, "RECOVER " + std::to_string(which));
    REQUIRE(r.ok);
    CHECK(doc.scene.bodies().size() == 2);
    CHECK(doc.dirty);
    CHECK(fs::path(doc.path).filename() == "bracket.sicdm");
    // It belongs to this session now: no longer offered (and kept until this session ends).
    CHECK(findRecoveryFiles(d.string()).size() == 1);
    // With unsaved work open, the headless command line won't replace it.
    CHECK(!run(doc, "RECOVER 1").ok);
    CHECK(!run(doc, "RECOVER 5").ok);
    REQUIRE(run(doc, "RECOVER DISCARD ALL").ok);
    CHECK(findRecoveryFiles(d.string()).empty());
    CHECK(run(doc, "RECOVER").lines.front() == "No unsaved work to recover");
    Autosaver(d.string()).clear(); // this session's adopted copy
    CHECK(filesIn(d) == 0);
    setEnv("SICDM_RECOVERY_DIR", "");
}

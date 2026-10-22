#include "TestFramework.hpp"

#include "core/Cam.hpp"
#include "core/CamVerify.hpp"
#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Scene.hpp"

#include <cstdio>
#include <filesystem>
#include <functional>
#include <fstream>
#include <sstream>
#include <string>

using namespace sicdm;

#define REQUIRE(c)                  \
    do {                            \
        const bool required_ = (c); \
        CHECK(required_);           \
        if (!required_) return;     \
    } while (0)

namespace {

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }

std::string text(const CommandOutput& r) {
    std::string s;
    for (const std::string& l : r.lines) s += l + "\n";
    return s;
}

bool has(const std::string& s, const std::string& what) { return s.find(what) != std::string::npos; }

std::string tempPath(const char* name) { return (std::filesystem::temp_directory_path() / name).string(); }

// The plate of the CAM tests, with every 2.5D operation and the drilling cycles.
bool bracketJob(Document& d) {
    for (const char* l : {"BOX 60 40 15 @0,0,0", "HOLE \"Block 1\" AT -20,0,15 DIAMETER 6 THROUGH", "HOLE \"Block 1\" AT 20,0,15 DIAMETER 6 THROUGH",
                          "POCKET \"Block 1\" AT 0,0,15 BOX WIDTH 16 LENGTH 24 DEPTH 6", "CAM SETUP \"Block 1\" STOCK 3 TOP 1 MATERIAL aluminium",
                          "CAM TOOL 1 FLAT 6", "CAM TOOL 2 DRILL 6", "CAM TOOL 3 FACE 40", "CAM TOOL 5 SPOT 10", "CAM FACE", "CAM POCKET",
                          "CAM CONTOUR TABS 4 TABHEIGHT 1.5", "CAM DRILL TOOL 5 CYCLE SPOT", "CAM DRILL TOOL 2 CYCLE PECK", "CAM POCKET TOOL 1 ADAPTIVE"})
        if (!run(d, l).ok) return false;
    return true;
}

std::string posted(Document& d, const char* post) { return postToString(d.state.cam, d.scene.bodies()[0], post); }

bool anyIssue(const CamPostCheck& c, const std::string& what) {
    for (const GcodeIssue& i : c.issues)
        if (i.error && has(i.text, what)) return true;
    return false;
}

// The program with the first line starting `prefix` changed by `edit`.
std::string editLine(const std::string& g, const std::string& prefix, const std::function<std::string(const std::string&)>& edit) {
    std::istringstream in(g);
    std::string out, l;
    bool done = false;
    while (std::getline(in, l)) {
        if (!done && l.rfind(prefix, 0) == 0) {
            l = edit(l);
            done = true;
            if (l.empty()) continue;
        }
        out += l + "\n";
    }
    return out;
}

} // namespace

TEST_CASE("Posts read back clean: every dialect, 2.5D, cycles, helical bore, 3+2 and 4-axis, on the toolpath") {
    Document d;
    REQUIRE(bracketJob(d));
    for (const char* post : {"fanuc", "haas", "linuxcnc", "grbl"}) {
        const CamPostCheck c = checkPost(d.state.cam, post, posted(d, post));
        if (c.errors) {
            std::fprintf(stderr, "  %s: %s\n", post, c.summary().c_str());
            for (const GcodeIssue& i : c.issues) std::fprintf(stderr, "    line %d: %s\n", i.line, i.text.c_str());
        }
        CHECK(c.errors == 0);
        CHECK(c.warnings == 0);
        CHECK(c.deviation <= c.allowed);
        CHECK(c.plot.arcs > 10);
        CHECK(c.plot.cycles >= (std::string(post) == "grbl" ? 0u : 4u));
        CHECK(c.plot.tools.size() == (std::string(post) == "grbl" ? 0u : 5u)); // T3 T1 T5 T2 T1
    }
    // Tapping: Fanuc's rigid tap (M29 G84), Haas G84, LinuxCNC's G33.1.
    REQUIRE(run(d, "CAM TOOL 4 TAP 6").ok);
    REQUIRE(run(d, "CAM TAP TOOL 4").ok);
    for (const char* post : {"fanuc", "haas", "linuxcnc"}) CHECK(checkPost(d.state.cam, post, posted(d, post)).errors == 0);
    // A helical bore, a 3+2 side pocket and hole, a 4-axis wrap.
    Document b;
    for (const char* l : {"BOX 50 50 12 @0,0,0", "HOLE \"Block 1\" AT 0,0,12 DIAMETER 20 THROUGH", "CAM SETUP \"Block 1\" STOCK 0 TOP 0", "CAM TOOL 1 FLAT 8",
                          "CAM BORE"})
        REQUIRE(run(b, l).ok);
    CHECK(checkPost(b.state.cam, "grbl", posted(b, "grbl")).errors == 0);
    Document x;
    for (const char* l : {"BOX 60 40 30 @0,0,0", "POCKET \"Block 1\" AT 0,-20,15 DIR 0,1,0 ROUND WIDTH 16 DEPTH 8",
                          "HOLE \"Block 1\" AT 30,0,15 DIR -1,0,0 DIAMETER 6 DEPTH 10", "CAM SETUP \"Block 1\" STOCK 0 TOP 0", "CAM TOOL 1 FLAT 6 LENGTH 20",
                          "CAM TOOL 2 DRILL 6", "CAM POCKET TOOL 1 A -90", "CAM DRILL TOOL 2 INDEX -90 -90"})
        REQUIRE(run(x, l).ok);
    const CamPostCheck ix = checkPost(x.state.cam, "haas", posted(x, "haas"));
    CHECK(ix.errors == 0);
    CHECK(ix.plot.usesA && ix.plot.usesC);
    Document r;
    for (const char* l : {"CYLINDER 15 60 @0,0,0", "ROTATE \"Cylinder 1\" 0,90,0", "CAM SETUP \"Cylinder 1\" STOCK 2", "CAM TOOL 1 BALL 6 LENGTH 30",
                          "CAM ROTARY TOOL 1 STEPOVER 3 STEPDOWN 4"})
        REQUIRE(run(r, l).ok);
    const CamPostCheck rc = checkPost(r.state.cam, "linuxcnc", posted(r, "linuxcnc"));
    CHECK(rc.errors == 0);
    CHECK(rc.deviation < 0.01);
}

TEST_CASE("The read-back catches what would hurt a machine") {
    Document d;
    REQUIRE(bracketJob(d));
    const std::string fanuc = posted(d, "fanuc");
    auto check = [&](const std::string& g, const char* post = "fanuc") { return checkPost(d.state.cam, post, g); };
    // A length without its decimal point: microns on a Fanuc.
    CHECK(anyIssue(check(editLine(fanuc, "G00 G90 X", [](const std::string& l) {
                       const std::size_t p = l.find('.');
                       return l.substr(0, p) + l.substr(p + 1);
                   })),
                   "no decimal point"));
    // No length offset for the tool.
    CHECK(anyIssue(check(editLine(fanuc, "G43 H", [](const std::string&) { return std::string(); })), "without its length offset"));
    // The wrong one.
    CHECK(anyIssue(check(editLine(fanuc, "G43 H01", [](std::string l) { return l.replace(l.find("H01"), 3, "H02"); })), "while tool 1"));
    // An arc whose centre is off: the control alarms.
    CHECK(anyIssue(check(editLine(fanuc, "G03", [](std::string l) {
                       const std::size_t i = l.find(" I");
                       return l.substr(0, i) + " I" + std::to_string(std::stod(l.substr(i + 2)) + 5.0) + l.substr(l.find(' ', i + 1));
                   })),
                   "from its centre"));
    // A cut 0.3 mm off the path.
    CHECK(anyIssue(check(editLine(fanuc, "G01 X", [](std::string l) {
                       const std::size_t i = l.find('X'), e = l.find(' ', i);
                       return l.substr(0, i + 1) + std::to_string(std::stod(l.substr(i + 1, e - i - 1)) + 0.3) + l.substr(e);
                   })),
                   "off the toolpath"));
    // The spindle never started.
    CHECK(anyIssue(check(editLine(fanuc, "S", [](const std::string&) { return std::string(); })), "spindle off"));
    // G28 in absolute mode drives down to Z0 first.
    CHECK(anyIssue(check(editLine(fanuc, "G91 G28 Z0.", [](const std::string&) { return std::string("G90 G28 Z0."); })), "goes down"));
    // A cut short program: the toolpath isn't all there, and no end.
    const CamPostCheck half = check(fanuc.substr(0, fanuc.size() / 2));
    CHECK(anyIssue(half, "misses part of the toolpath"));
    CHECK(!half.issues.empty() && half.warnings >= 1);
    // A code GRBL doesn't have.
    const std::string grbl = posted(d, "grbl");
    CHECK(anyIssue(check(editLine(grbl, "G54", [](const std::string& l) { return l + "\nG83 X0 Y0 Z-5 R2 Q1 F100"; }), "grbl"), "isn't a grbl code"));
}

TEST_CASE("Machines: travel, rotary axes and tool pockets are checked; POST won't write a program that fails") {
    Document d;
    REQUIRE(bracketJob(d));
    const std::string path = tempPath("sicdm_verify.nc");
    std::filesystem::remove(path);
    // A small machine's span: the 66 mm stock doesn't fit 50 mm of travel.
    REQUIRE(run(d, "CAM MACHINE TRAVEL 50 50 200").ok);
    auto r = run(d, "CAM POST " + path);
    CHECK(!r.ok);
    CHECK(has(text(r), "X travel"));
    CHECK(!std::filesystem::exists(path));
    // FORCE writes it anyway.
    CHECK(run(d, "CAM POST " + path + " FORCE").ok);
    CHECK(std::filesystem::exists(path));
    // A real machine class, with G54 placed near the end of X: past the travel.
    REQUIRE(run(d, "CAM MACHINE PRESET vmc ZERO 740,200,-300").ok);
    r = run(d, "CAM CHECK");
    CHECK(!r.ok);
    CHECK(has(text(r), "X travel"));
    REQUIRE(run(d, "CAM MACHINE ZERO 300,200,-300").ok);
    r = run(d, "CAM CHECK");
    CHECK(r.ok);
    CHECK(has(text(r), "0 errors"));
    // Z: the spindle nose with the longest tool, too high for a work zero near the top.
    REQUIRE(run(d, "CAM MACHINE ZERO 300,200,-20").ok);
    r = run(d, "CAM CHECK");
    CHECK(has(text(r), "Z travel"));
    REQUIRE(run(d, "CAM MACHINE ZERO 300,200,-300").ok);
    // Tool pockets.
    REQUIRE(run(d, "CAM MACHINE POCKETS 3").ok);
    r = run(d, "CAM CHECK");
    CHECK(has(text(r), "T5: the tool changer has pockets 1 to 3"));
    // The spindle's limit: feeds and speeds follow it, so the program keeps to it.
    REQUIRE(run(d, "CAM MACHINE POCKETS 20 MAXRPM 3000").ok);
    r = run(d, "CAM CHECK");
    CHECK(r.ok);
    CHECK(checkPost(d.state.cam, "fanuc", posted(d, "fanuc")).plot.maxRpm <= 3000);
    // Rotary axes: a 3-axis machine has none; a trunnion's A stops at -35.
    Document x;
    for (const char* l : {"BOX 60 40 30 @0,0,0", "POCKET \"Block 1\" AT 0,-20,15 DIR 0,1,0 ROUND WIDTH 16 DEPTH 8", "CAM SETUP \"Block 1\" STOCK 0 TOP 0",
                          "CAM TOOL 1 FLAT 6 LENGTH 20", "CAM POCKET TOOL 1 A -90", "CAM MACHINE PRESET vmc"})
        REQUIRE(run(x, l).ok);
    CHECK(has(text(run(x, "CAM CHECK")), "no A axis"));
    REQUIRE(run(x, "CAM MACHINE PRESET trunnion").ok);
    CHECK(has(text(run(x, "CAM CHECK")), "A turns to -90"));
    REQUIRE(run(x, "CAM MACHINE A -120 120").ok);
    CHECK(run(x, "CAM CHECK").ok);
    // A program from a file, checked against the job.
    REQUIRE(run(d, "CAM MACHINE PRESET generic").ok);
    REQUIRE(run(d, "CAM POST " + path + " haas").ok);
    r = run(d, "CAM CHECK " + path + " haas");
    CHECK(r.ok);
    std::filesystem::remove(path);
}

TEST_CASE("The machine is saved with the project") {
    Document d;
    REQUIRE(bracketJob(d));
    REQUIRE(run(d, "CAM MACHINE PRESET trunnion ZERO 300,200,-250 A -30 110 POCKETS 24 NAME \"Cell 3\"").ok);
    const std::string proj = tempPath("sicdm_machine.sicdm");
    REQUIRE(run(d, "SAVE " + proj).ok);
    Document e;
    REQUIRE(run(e, "OPEN " + proj).ok);
    const CamMachine& m = e.state.cam.setup.machine;
    CHECK(m.name == "Cell 3");
    CHECK(m.hasWorkOffset && m.workOffset[0] == 300 && m.workOffset[2] == -250);
    CHECK(m.hasA && m.aMin == -30 && m.aMax == 110);
    CHECK(m.hasC && m.cMax >= 99999);
    CHECK(m.toolPockets == 24);
    CHECK(m.travelMax[0] == 610 && m.travelMin[2] == -394);
    CHECK(m.maxRpm == 12000);
    std::filesystem::remove(proj);
}

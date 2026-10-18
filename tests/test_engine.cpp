#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"

#include <filesystem>
#include <fstream>

using namespace sicdm;
namespace fs = std::filesystem;

namespace {

CommandOutput run(Document& d, const char* line) { return executeCommand(d, line, nullptr); }

std::string tmpPath(const char* name) {
    const fs::path dir = fs::temp_directory_path() / "sicdm_engine_tests";
    fs::create_directories(dir);
    return (dir / name).string();
}

} // namespace

TEST_CASE("sketch commands create curves on the active plane") {
    Document d;
    CHECK(run(d, "PLANE front").ok);
    CHECK(run(d, "line 0,0 @50,0 @0,30").ok); // two connected segments
    CHECK(d.scene.sketch().curves().size() == 2);
    CHECK_NEAR(d.scene.sketch().curves()[1].pts[1].y, 30, 1e-5);
    CHECK(d.scene.sketch().curves()[1].plane == SketchPlane::Front);
    CHECK(run(d, "rect 0,0 @40,20").ok);
    CHECK(run(d, "c 10,10 5").ok);
    CHECK(run(d, "circle 0,0 @3,4").ok); // radius from a point on the circle
    CHECK_NEAR(d.scene.sketch().curves().back().radius, 5, 1e-5);
    CHECK(run(d, "pline 0,0 10,0 10,10 close").ok);
    CHECK(d.scene.sketch().curves().back().pts.size() == 4);
    CHECK(run(d, "point 1,2").ok);
    CHECK(d.dirty);
    CHECK(!run(d, "line 0,0").ok);
    CHECK(!run(d, "circle 0,0 -1").ok);
    CHECK(!run(d, "rect 0,0 0,10").ok);
}

TEST_CASE("solid commands build parametric bodies") {
    Document d;
    auto o = run(d, "BOX 40 30 20 @10,0,5");
    CHECK(o.ok);
    CHECK(d.scene.bodies().size() == 1);
    const Body& b = d.scene.bodies()[0];
    CHECK(b.kind == "box");
    CHECK_NEAR(b.bounds().max.x - b.bounds().min.x, 40, 1e-4);
    CHECK_NEAR(b.bounds().min.z, 5, 1e-4);
    CHECK(run(d, "cyl 10 30").ok);
    CHECK(run(d, "cone 10 20 2").ok);
    CHECK(run(d, "sphere 8").ok);
    CHECK(run(d, "torus 20 4").ok);
    CHECK(run(d, "prism 10 20 8").ok);
    CHECK(d.scene.bodies().back().params[2] == 8.0f);
    CHECK(!run(d, "torus 4 20").ok); // tube larger than ring
    CHECK(!run(d, "box 1 2").ok);
    CHECK(d.selectedBody == static_cast<int>(d.scene.bodies().size()) - 1);
}

TEST_CASE("select, move and erase by name") {
    Document d;
    run(d, "box 10 10 10");
    run(d, "line 0,0 5,5");
    const std::string boxName = d.scene.bodies()[0].name;
    CHECK(run(d, ("select " + boxName).c_str()).ok);
    CHECK(d.selectedBody == 0);
    CHECK(run(d, "move 1,2,3").ok);
    CHECK_NEAR(d.scene.bodies()[0].offset.z, 3, 1e-6);
    CHECK(run(d, "erase Line 1").ok);
    CHECK(d.scene.sketch().curves().empty());
    CHECK(run(d, ("erase " + boxName).c_str()).ok);
    CHECK(d.scene.bodies().empty());
    CHECK(!run(d, "erase").ok);
    CHECK(!run(d, "select ghost").ok);
}

TEST_CASE("save, open, export and import through commands") {
    Document d;
    run(d, "box 20 20 20");
    run(d, "circle 0,0 15");
    const std::string proj = tmpPath("cmd_project"); // extension added
    CHECK(run(d, ("save " + proj).c_str()).ok);
    CHECK(!d.dirty);
    CHECK(fs::exists(proj + ".sicdm"));
    CHECK(d.title() == "cmd_project.sicdm");
    CHECK(run(d, ("export " + tmpPath("cmd.stl")).c_str()).ok);
    CHECK(run(d, ("export " + tmpPath("cmd.dxf")).c_str()).ok);
    CHECK(!run(d, ("export " + tmpPath("cmd.xyz")).c_str()).ok);

    Document e;
    auto o = run(e, ("open " + proj + ".sicdm").c_str());
    CHECK(o.ok && o.documentReplaced);
    CHECK(e.scene.bodies().size() == 1);
    CHECK(e.scene.sketch().curves().size() == 1);
    CHECK(run(e, ("import " + tmpPath("cmd.stl")).c_str()).ok);
    CHECK(e.scene.bodies().size() == 2);
    CHECK(run(e, "plane right").ok);
    CHECK(run(e, ("import " + tmpPath("cmd.dxf")).c_str()).ok);
    CHECK(e.scene.sketch().curves().back().plane == SketchPlane::Right);
    CHECK(run(e, "new").ok);
    CHECK(e.scene.bodies().empty() && e.path.empty());
    CHECK(!run(e, "open /definitely/not/here.sicdm").ok);
}

TEST_CASE("view commands need a window when headless; HELP and LIST work") {
    Document d;
    CHECK(!run(d, "view top").ok);
    CHECK(!run(d, "2d").ok);
    CHECK(run(d, "help").lines.size() == commandTable().size() + 1);
    CHECK(run(d, "help circle").ok);
    run(d, "box 1 1 1");
    const auto l = run(d, "list");
    CHECK(l.ok && l.lines.size() >= 4);
    CHECK(!run(d, "frobnicate").ok);
    CHECK(run(d, "").ok);
    CHECK(run(d, "clear").clearHistory);
}

TEST_CASE("EXTRUDE turns a selected or named profile into a solid") {
    Document d;
    CHECK(run(d, "rect 0,0 @40,20").ok);
    auto o = run(d, "extrude 15"); // selected curve
    CHECK(o.ok);
    CHECK(d.scene.bodies().size() == 1);
    const Body& b = d.scene.bodies()[0];
    CHECK(b.kind == "extrude");
    CHECK_NEAR(b.bounds().max.z, 15, 1e-4);
    CHECK(!d.scene.sketch().curves()[0].visible); // consumed
    CHECK(d.selectedBody == 0);

    CHECK(run(d, "plane front").ok);
    CHECK(run(d, "circle 0,10 5").ok);
    CHECK(run(d, "extrude Circle 2 20 symmetric keep").ok);
    const Body& c = d.scene.bodies()[1];
    CHECK_NEAR(c.bounds().min.y, -10, 1e-3);
    CHECK_NEAR(c.bounds().max.y, 10, 1e-3);
    CHECK(d.scene.sketch().curves()[1].visible);

    CHECK(run(d, "line 0,0 10,10").ok);
    CHECK(!run(d, "extrude 5").ok);       // open curve
    CHECK(!run(d, "extrude ghost 5").ok); // unknown name
    CHECK(!run(d, "extrude").ok);         // no distance
    CHECK(!run(d, "select none").lines.empty());
    CHECK(!run(d, "extrude 0").ok);
}

TEST_CASE("extruded bodies survive save and reopen") {
    Document d;
    run(d, "pline 0,0 30,0 30,10 10,10 10,30 0,30 close");
    CHECK(run(d, "extrude 8").ok);
    const std::string path = tmpPath("extrude_project.sicdm");
    CHECK(run(d, ("save " + path).c_str()).ok);
    Document e;
    CHECK(run(e, ("open " + path).c_str()).ok);
    CHECK(e.scene.bodies().size() == 1);
    CHECK(e.scene.bodies()[0].kind == "extrude");
    CHECK(e.scene.bodies()[0].mesh.indices.size() == d.scene.bodies()[0].mesh.indices.size());
}

namespace {

// Minimal multi-document host: like the app, it swaps the open files through `doc`.
struct TabsHost final : ViewHost {
    Document& doc;
    std::vector<Document> parked; // the inactive files
    int prepared = 0;
    explicit TabsHost(Document& d) : doc(d) {}

    void openWindow(bool) override {}
    void standardView(StandardView) override {}
    void planeChanged(SketchPlane) override {}
    void fit() override {}
    void zoomStep(float) override {}
    void home() override {}
    void roll(float, bool) override {}
    void setProjection(Projection) override {}
    void setDisplay(int) override {}
    void setGrid(std::optional<bool>) override {}
    void setPanel(std::optional<bool>) override {}
    std::string describe() const override { return {}; }

    bool prepareDocument(const std::string& path) override {
        ++prepared;
        for (Document& p : parked)
            if (!path.empty() && p.path == path) {
                std::swap(p, doc);
                return true;
            }
        parked.push_back(std::move(doc));
        doc = Document{};
        doc.name = "Untitled " + std::to_string(parked.size() + 1);
        return false;
    }
    std::vector<std::string> fileTabs() const override {
        std::vector<std::string> t{"> " + doc.title()};
        for (const Document& p : parked) t.push_back("  " + p.title());
        return t;
    }
    bool closeFile(bool discard, std::string* error) override {
        if (doc.dirty && !discard) {
            *error = "unsaved";
            return false;
        }
        if (parked.empty()) doc = Document{};
        else {
            doc = std::move(parked.back());
            parked.pop_back();
        }
        return true;
    }
};

} // namespace

TEST_CASE("NEW and OPEN ask a multi-document host for a tab; FILES and CLOSE") {
    Document d;
    TabsHost host(d);
    auto runIn = [&](const std::string& line) { return executeCommand(d, line, &host); };

    CHECK(runIn("BOX 10 10 10").ok);
    const std::string proj = tmpPath("tabs_a.sicdm");
    CHECK(runIn("SAVE " + proj).ok);
    const std::string savedPath = d.path;

    CHECK(runIn("NEW").ok); // the saved file stays open in its tab
    CHECK(host.prepared == 1);
    CHECK(host.parked.size() == 1);
    CHECK(host.parked[0].scene.bodies().size() == 1);
    CHECK(d.scene.bodies().empty());
    CHECK(d.title() == "Untitled 2");

    CHECK(!runIn("OPEN " + tmpPath("missing.sicdm")).ok); // nothing loads: no tab is made
    CHECK(host.prepared == 1);

    const CommandOutput again = runIn("OPEN " + proj); // already open: its tab is shown
    CHECK(again.ok);
    CHECK(!again.documentReplaced);
    CHECK(d.path == savedPath);
    CHECK(d.scene.bodies().size() == 1);
    CHECK(runIn("FILES").lines.size() == 2);

    CHECK(runIn("SPHERE 4").ok);
    d.dirty = true;
    CHECK(!runIn("CLOSE").ok); // unsaved work is not dropped silently
    CHECK(runIn("CLOSE DISCARD").ok);
    CHECK(d.title() == "Untitled 2");
    CHECK(!runIn("CLOSE NOW").ok);
}

TEST_CASE("file tab commands are harmless headless") {
    Document d;
    CHECK(run(d, "FILES").ok);
    CHECK(!run(d, "FILES 2").ok);
    CHECK(!run(d, "CLOSE").ok);
    CHECK(run(d, "NEW").ok);
}

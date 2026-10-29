#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Mcp.hpp"
#include "core/SectionView.hpp"

#include <cmath>
#include <filesystem>
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

std::string text(const CommandOutput& o) {
    std::string s;
    for (const std::string& l : o.lines) s += l + "\n";
    return s;
}

bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

const SectionCut* cutOf(const std::vector<SectionCut>& cuts, const std::string& body) {
    for (const SectionCut& c : cuts)
        if (c.body == body) return &c;
    return nullptr;
}

} // namespace

TEST_CASE("section view: a plane through the model, the section's area and centroid") {
    Document d;
    REQUIRE(run(d, "BOX 40 30 20 @0,0,0").ok);
    CHECK(!d.state.section.on);
    const CommandOutput o = run(d, "SECTION X"); // through the middle, keeping x < 0
    REQUIRE(o.ok);
    CHECK(d.state.section.on);
    CHECK(text(o).find("x = 0, keeping x < 0") != std::string::npos);
    CHECK(text(o).find("section area 600 mm2") != std::string::npos);
    auto cuts = sectionCuts(d.scene, d.state.section);
    REQUIRE(cuts.size() == 1);
    CHECK(near(cuts[0].area, 600, 1e-3));
    CHECK(near(cuts[0].centroid.x, 0, 1e-4) && near(cuts[0].centroid.y, 0, 1e-4) && near(cuts[0].centroid.z, 10, 1e-4));
    // Another axis, another place; then moved and flipped.
    REQUIRE(run(d, "SECTION Z AT 5").ok);
    cuts = sectionCuts(d.scene, d.state.section);
    REQUIRE(cuts.size() == 1);
    CHECK(near(cuts[0].area, 1200, 1e-3));
    CHECK(near(cuts[0].centroid.z, 5, 1e-4));
    REQUIRE(run(d, "SECTION MOVE 10").ok); // z = 15
    CHECK(near(sectionCuts(d.scene, d.state.section)[0].centroid.z, 15, 1e-4));
    REQUIRE(run(d, "SECTION FLIP").ok);
    CHECK(text(run(d, "SECTION")).find("z = 15, keeping z > 15") != std::string::npos);
    CHECK(near(d.state.section.distance({0, 0, 20}), -5, 1e-6)); // the top is kept now
    // -Y at a point; any plane.
    REQUIRE(run(d, "SECTION -Y AT 0,5,0").ok);
    CHECK(text(run(d, "SECTION")).find("y = 5, keeping y > 5") != std::string::npos);
    CHECK(near(sectionCuts(d.scene, d.state.section)[0].area, 800, 1e-3));
    REQUIRE(run(d, "SECTION PLANE 1,1,0 AT 0,0,0").ok);
    CHECK(near(sectionCuts(d.scene, d.state.section)[0].area, 30 * std::sqrt(2.0) * 20, 1e-2)); // the diagonal across 30 x 30
    // Off and on again: the same plane.
    REQUIRE(run(d, "SECTION OFF").ok);
    CHECK(!d.state.section.on);
    REQUIRE(run(d, "SECTION ON").ok);
    CHECK(d.state.section.on);
    CHECK(near(d.state.section.normal.x, std::sqrt(0.5), 1e-6));
    // A cylinder (its mesh has a seam): cut through its axis, a rectangle.
    REQUIRE(run(d, "CYLINDER 10 30 @200,0,0").ok);
    REQUIRE(run(d, "SECTION -Y AT 200,0,0").ok);
    cuts = sectionCuts(d.scene, d.state.section);
    const SectionCut* cyl = cutOf(cuts, d.scene.bodies().back().name);
    REQUIRE(cyl != nullptr);
    CHECK(near(cyl->area, 20 * 30, 1.0));
    CHECK(near(cyl->centroid.x, 200, 1e-3) && near(cyl->centroid.z, 15, 1e-3));
    // Nothing cut: said so.
    REQUIRE(run(d, "SECTION X AT 500").ok);
    CHECK(text(run(d, "SECTION")).find("cuts no solid") != std::string::npos);
}

TEST_CASE("section view: holes leave gaps, only the chosen bodies are cut, open surfaces aren't solids") {
    Document d;
    REQUIRE(run(d, "BOX 60 60 20 @0,0,0").ok);
    REQUIRE(run(d, "HOLE \"Block 1\" AT 0,0,20 DIR 0,0,-1 DIAMETER 10 THROUGH").ok);
    REQUIRE(run(d, "BOX 20 20 20 @100,0,0").ok);
    REQUIRE(run(d, "LINE 200,-10 220,10").ok); // a surface across the plane: no inside to fill
    REQUIRE(run(d, "SURFACE EXTRUDE \"Line 1\" DISTANCE 10").ok);
    REQUIRE(run(d, "SECTION Y AT 0").ok);
    auto cuts = sectionCuts(d.scene, d.state.section);
    REQUIRE(cuts.size() == 2);
    const SectionCut* a = cutOf(cuts, "Block 1");
    REQUIRE(a != nullptr);
    CHECK(a->loops == 2);                       // the two sides of the hole
    CHECK(near(a->area, 60 * 20 - 10 * 20, 15)); // the hole is faceted
    CHECK(cutOf(cuts, "Block 2") != nullptr);
    REQUIRE(run(d, "SECTION ONLY \"Block 2\"").ok);
    cuts = sectionCuts(d.scene, d.state.section);
    REQUIRE(cuts.size() == 1);
    CHECK(cuts[0].body == "Block 2");
    CHECK(!run(d, "SECTION ONLY Nothing").ok);
    REQUIRE(run(d, "SECTION ALL").ok);
    CHECK(d.state.section.only.empty());
    // Refused: nonsense.
    CHECK(!run(d, "SECTION PLANE 0,0,0").ok);
    CHECK(!run(d, "SECTION SIDEWAYS").ok);
    CHECK(!run(d, "SECTION MOVE").ok);
}

TEST_CASE("section view: picking skips what is taken away and stops on cut faces") {
    Document d;
    REQUIRE(run(d, "BOX 60 60 20 @0,0,0").ok);
    REQUIRE(run(d, "HOLE \"Block 1\" AT 0,0,20 DIR 0,0,-1 DIAMETER 10 THROUGH").ok);
    const Ray solid{{100, 20, 10}, {-1, 0, 0}}; // towards the block, through material
    const Ray hole{{100, 0, 10}, {-1, 0, 0}};   // ... through the hole's middle
    // No section: the near side.
    auto h = sectionRaycast(d.scene, d.state.section, solid);
    REQUIRE(h.has_value());
    CHECK(near(h->point.x, 30, 1e-3));
    REQUIRE(run(d, "SECTION X AT 0").ok); // keeping x < 0
    h = sectionRaycast(d.scene, d.state.section, solid);
    REQUIRE(h.has_value());
    CHECK(h->body == 0);
    CHECK(near(h->point.x, 0, 1e-3)); // the cut face
    CHECK(near(h->t, 100, 1e-3));
    h = sectionRaycast(d.scene, d.state.section, hole);
    REQUIRE(h.has_value());
    CHECK(h->point.x < -4.5f); // through the gap to the far wall of the hole
    // Cut faces not filled: through to the inside of the far wall.
    REQUIRE(run(d, "SECTION CAP OFF").ok);
    h = sectionRaycast(d.scene, d.state.section, solid);
    REQUIRE(h.has_value());
    CHECK(near(h->point.x, -30, 1e-3));
    REQUIRE(run(d, "SECTION CAP ON").ok);
    // Straight down onto the half taken away: nothing there.
    CHECK(!sectionRaycast(d.scene, d.state.section, Ray{{10, 20, 100}, {0, 0, -1}}).has_value());
    CHECK(sectionRaycast(d.scene, d.state.section, Ray{{-10, 20, 100}, {0, 0, -1}}).has_value());
    // Flipped: the near half is kept, the near face hit as usual.
    REQUIRE(run(d, "SECTION FLIP").ok);
    h = sectionRaycast(d.scene, d.state.section, solid);
    REQUIRE(h.has_value());
    CHECK(near(h->point.x, 30, 1e-3));
    // Seen from the other side: the kept half's cut face.
    h = sectionRaycast(d.scene, d.state.section, Ray{{-100, 20, 10}, {1, 0, 0}});
    REQUIRE(h.has_value());
    CHECK(near(h->point.x, 0, 1e-3));
}

TEST_CASE("section view: saved with the project, cleared by NEW, free on released revisions, over MCP") {
    Document d;
    REQUIRE(run(d, "BOX 40 30 20 @0,0,0").ok);
    REQUIRE(run(d, "SECTION -Z AT 12").ok);
    REQUIRE(run(d, "SECTION CAP OFF").ok);
    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_section.sicdm").string();
    REQUIRE(run(d, "SAVE \"" + path + "\"").ok);
    Document e;
    REQUIRE(run(e, "OPEN \"" + path + "\"").ok);
    CHECK(e.state.section == d.state.section);
    CHECK(e.state.section.on && !e.state.section.cap);
    CHECK(near(e.state.section.distance({0, 0, 12}), 0, 1e-6));
    // A released revision can still be looked into.
    REQUIRE(run(e, "REVISION RELEASE").ok);
    CHECK(run(e, "SECTION X").ok);
    CHECK(run(e, "SECTION OFF").ok);
    REQUIRE(run(e, "NEW").ok);
    CHECK(e.state.section == SectionView{});
    // Over MCP.
    Document m;
    McpServer s(m, nullptr);
    const auto tool = [&](const std::string& args) {
        const std::string msg = R"({"jsonrpc":"2.0","id":7,"method":"tools/call","params":{"name":"section","arguments":)" + args + "}}";
        const auto r = Json::parse(s.handle(msg));
        return r ? (*r)["result"] : Json();
    };
    REQUIRE(run(m, "BOX 40 30 20 @0,0,0").ok);
    Json r = tool(R"({"operation":"set","axis":"Y","at":5})");
    CHECK(!r["isError"].asBool());
    CHECK(r["content"][0]["text"].asString().find("section area 800 mm2") != std::string::npos);
    CHECK(!tool(R"({"operation":"move","distance":-5})")["isError"].asBool());
    CHECK(near(m.state.section.offset, 0, 1e-9));
    CHECK(!tool(R"({"operation":"set","normal":[0,0,1],"point":[0,0,4]})")["isError"].asBool());
    CHECK(near(m.state.section.offset, 4, 1e-6));
    CHECK(!tool(R"({"operation":"off"})")["isError"].asBool());
    CHECK(!m.state.section.on);
    CHECK(tool(R"({"operation":"move"})")["isError"].asBool());
    CHECK(tool(R"({"operation":"twist"})")["isError"].asBool());
}

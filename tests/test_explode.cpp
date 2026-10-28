#include "TestFramework.hpp"

#include "core/Assembly.hpp"
#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Drawing.hpp"
#include "core/FileIO.hpp"
#include "core/Json.hpp"
#include "core/Mcp.hpp"
#include "core/Scene.hpp"

#include <algorithm>
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

int named(const Document& d, const std::string& n) {
    for (std::size_t i = 0; i < d.scene.bodies().size(); ++i)
        if (d.scene.bodies()[i].name == n) return static_cast<int>(i);
    return -1;
}

// A housing, a shaft through it, a cover on it and four screws in the cover: the parts with
// their part numbers. The screws share one.
void buildGearbox(Document& d) {
    for (const char* l : {"BOX 80 80 40 @0,0,0", "CYLINDER 6 70 @0,0,0", "BOX 80 80 8 @0,0,40", "CYLINDER 3 14 @30,30,40", "CYLINDER 3 14 @-30,30,40",
                          "CYLINDER 3 14 @30,-30,40", "CYLINDER 3 14 @-30,-30,40"})
        CHECK(run(d, l).ok);
    CHECK(run(d, "PROPERTY \"Block 1\" NUMBER H-100").ok);
    CHECK(run(d, "PROPERTY \"Block 1\" DESCRIPTION Housing").ok);
    CHECK(run(d, "PROPERTY \"Block 1\" MATERIAL Aluminium 6061").ok);
    CHECK(run(d, "PROPERTY \"Cylinder 2\" NUMBER S-200").ok);
    CHECK(run(d, "PROPERTY \"Block 3\" NUMBER C-300").ok);
    for (int k = 4; k <= 7; ++k) CHECK(run(d, "PROPERTY \"Cylinder " + std::to_string(k) + "\" NUMBER M6x14").ok);
    CHECK(run(d, "PROPERTY \"Cylinder 7\" DESCRIPTION Cap screw").ok); // one of them says what they are
}

AABB shiftedBounds(const Document& d, const std::vector<DVec3>& s, int i) {
    AABB b = d.scene.bodies()[static_cast<std::size_t>(i)].bounds();
    b.min += s[static_cast<std::size_t>(i)].f(), b.max += s[static_cast<std::size_t>(i)].f();
    return b;
}

} // namespace

TEST_CASE("explode: automatic steps stack the parts apart along the axis, the largest staying put") {
    Document d;
    buildGearbox(d);
    const CommandOutput r = run(d, "EXPLODE AUTO");
    REQUIRE(r.ok);
    const ExplodeState& ex = d.state.assembly.explode;
    CHECK(ex.shown);
    CHECK(ex.steps.size() == 3); // the screws together, the cover, the shaft
    const auto s = explodeShifts(d.scene, ex, 1);
    const int housing = named(d, "Block 1"), shaft = named(d, "Cylinder 2"), cover = named(d, "Block 3"), screw = named(d, "Cylinder 4");
    CHECK(dlength(s[static_cast<std::size_t>(housing)]) == 0);
    // Each part clear above what it sat on (or in), a gap between them.
    const AABB h = shiftedBounds(d, s, housing), sh = shiftedBounds(d, s, shaft), c = shiftedBounds(d, s, cover), sc = shiftedBounds(d, s, screw);
    CHECK(sh.min.z > h.max.z + 1);
    CHECK(c.min.z > sh.max.z + 1);
    CHECK(sc.min.z > c.max.z + 1);
    // All four screws went up together, straight up.
    for (int k = 5; k <= 7; ++k) CHECK(std::fabs(s[static_cast<std::size_t>(named(d, "Cylinder " + std::to_string(k)))].z - s[static_cast<std::size_t>(screw)].z) < 1e-9);
    CHECK(s[static_cast<std::size_t>(screw)].x == 0);
    // Played part way: the steps take turns, the farthest first.
    const auto half = explodeShifts(d.scene, ex, 1.0 / 3);
    CHECK(std::fabs(half[static_cast<std::size_t>(screw)].z - s[static_cast<std::size_t>(screw)].z) < 1e-9);
    CHECK(half[static_cast<std::size_t>(shaft)].z == 0);
    // The parts themselves did not move.
    CHECK(std::fabs(d.scene.bodies()[static_cast<std::size_t>(cover)].bounds().min.z - 40) < 1e-4f);
    // Out from the middle instead: the parts away from the assembly's centre.
    REQUIRE(run(d, "EXPLODE AUTO RADIAL FACTOR 2").ok);
    const auto rs = explodeShifts(d.scene, d.state.assembly.explode, 1);
    CHECK(rs[static_cast<std::size_t>(screw)].x > 0);
    CHECK(rs[static_cast<std::size_t>(screw)].y > 0);
    CHECK(!run(d, "EXPLODE AUTO FACTOR 0").ok);
}

TEST_CASE("explode: steps by hand, instances and sub-assemblies, attached parts, undo, save and regen") {
    Document d;
    REQUIRE(run(d, "BOX 40 40 10 @0,0,0").ok);
    REQUIRE(run(d, "BOX 10 10 10 @0,0,10").ok);
    REQUIRE(run(d, "CYLINDER 2 20 @30,0,0").ok);
    REQUIRE(run(d, "ASSEMBLY GROUP \"Top\" \"Block 2\"").ok);
    d.scene.bodies()[2].attachedTo = "Block 2"; // a pin on the top block goes with it
    CHECK(!run(d, "EXPLODE ADD Nothing ALONG Z DISTANCE 5").ok);
    CHECK(!run(d, "EXPLODE ADD Top ALONG Z").ok);        // how far?
    CHECK(!run(d, "EXPLODE ADD Top ALONG W DISTANCE 5").ok);
    REQUIRE(run(d, "EXPLODE ADD Top ALONG Z DISTANCE 30").ok); // the sub-assembly by its name
    REQUIRE(run(d, "EXPLODE ADD \"Block 2\" DIR 1,0,0 DISTANCE 10").ok);
    auto s = explodeShifts(d.scene, d.state.assembly.explode, 1);
    CHECK(std::fabs(s[1].z - 30) < 1e-9);
    CHECK(std::fabs(s[1].x - 10) < 1e-9);
    CHECK(std::fabs(s[2].z - 30) < 1e-9); // the attached pin came along
    CHECK(dlength(s[0]) == 0);
    REQUIRE(run(d, "EXPLODE EDIT 1 DISTANCE -15").ok); // a negative distance goes the other way
    s = explodeShifts(d.scene, d.state.assembly.explode, 1);
    CHECK(std::fabs(s[1].z + 15) < 1e-9);
    CHECK(text(run(d, "EXPLODE")).find("2: Block 2 10 mm along +X") != std::string::npos);
    // Undo takes a step back.
    REQUIRE(run(d, "EXPLODE DELETE 2").ok);
    CHECK(d.state.assembly.explode.steps.size() == 1);
    REQUIRE(run(d, "UNDO").ok);
    CHECK(d.state.assembly.explode.steps.size() == 2);
    REQUIRE(run(d, "EXPLODE HIDE").ok);
    CHECK(!d.state.assembly.explode.shown);
    REQUIRE(run(d, "EXPLODE TRAILS OFF").ok);
    // Saved and opened again.
    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_explode.sicdm").string();
    REQUIRE(run(d, "SAVE \"" + path + "\"").ok);
    Document e;
    REQUIRE(run(e, "OPEN \"" + path + "\"").ok);
    REQUIRE(e.state.assembly.explode.steps.size() == 2);
    CHECK(e.state.assembly.explode.steps[0].parts[0] == "Top");
    CHECK(std::fabs(e.state.assembly.explode.steps[0].dir.z + 1) < 1e-9);
    CHECK(!e.state.assembly.explode.shown);
    CHECK(!e.state.assembly.explode.trails);
    // REGEN rebuilds the model; the exploded view stays.
    REQUIRE(run(e, "REGEN").ok);
    CHECK(e.state.assembly.explode.steps.size() == 2);
    // A part that is gone is listed as such.
    REQUIRE(run(e, "ERASE \"Block 2\"").ok);
    CHECK(text(run(e, "EXPLODE LIST")).find("[gone: Block 2]") != std::string::npos);
    REQUIRE(run(e, "EXPLODE CLEAR").ok);
    CHECK(e.state.assembly.explode.empty() == false); // trails off is kept
    CHECK(e.state.assembly.explode.steps.empty());
    CHECK(!run(e, "EXPLODE SHOW").ok);
}

TEST_CASE("explode: a released revision keeps its exploded view, which can still be shown") {
    Document d;
    buildGearbox(d);
    REQUIRE(run(d, "EXPLODE AUTO").ok);
    REQUIRE(run(d, "REVISION RELEASE").ok);
    CHECK(!run(d, "EXPLODE ADD \"Block 3\" ALONG Z DISTANCE 5").ok);
    CHECK(run(d, "EXPLODE HIDE").ok);
    CHECK(run(d, "EXPLODE SHOW").ok);
    CHECK(run(d, "EXPLODE").ok);
}

TEST_CASE("parts list: items for the drawn parts, bodies of one part number together, any of them describing it") {
    Document d;
    buildGearbox(d);
    std::vector<int> item;
    const auto list = partsList(d.scene, [](const Body& b) { return b.name != "Block 3"; }, &item);
    REQUIRE(list.size() == 3);
    CHECK(list[2].number == "M6x14");
    CHECK(list[2].quantity == 4);
    CHECK(list[2].description == "Cap screw");
    CHECK(item[static_cast<std::size_t>(named(d, "Block 3"))] == -1);
    CHECK(item[static_cast<std::size_t>(named(d, "Cylinder 6"))] == 2);
    CHECK(list[1].firstBody == static_cast<std::size_t>(named(d, "Cylinder 2")));
}

TEST_CASE("assembly drawing: the exploded view, balloons numbered as the parts list, trail lines") {
    Document d;
    buildGearbox(d);
    REQUIRE(run(d, "EXPLODE AUTO").ok);
    const CommandOutput r = run(d, "SHEET NEW ASSEMBLY");
    REQUIRE(r.ok);
    const Drawing& dw = d.state.drawing;
    REQUIRE(dw.views.size() == 1);
    CHECK(dw.views[0].exploded);
    CHECK(dw.partsList.shown);
    const auto rows = drawingPartsRows(dw, d.scene);
    REQUIRE(rows.size() == 4);
    CHECK(rows[0][2] == "H-100");
    CHECK(rows[0][3].empty() == false);
    CHECK(rows[0][4] == "Aluminium 6061");
    CHECK(rows[3][1] == "4");
    CHECK(rows[3][3] == "Cap screw");
    // A balloon for each item, round the view and clear of each other.
    REQUIRE(dw.balloons.size() == 4);
    std::vector<int> itemOf;
    drawingPartsRows(dw, d.scene, &itemOf);
    std::vector<int> items;
    for (const DrawingBalloon& b : dw.balloons) items.push_back(itemOf[static_cast<std::size_t>(named(d, b.body))]);
    std::sort(items.begin(), items.end());
    CHECK((items == std::vector<int>{1, 2, 3, 4}));
    for (std::size_t i = 0; i < dw.balloons.size(); ++i)
        for (std::size_t j = i + 1; j < dw.balloons.size(); ++j)
            CHECK(std::hypot(dw.balloons[i].x - dw.balloons[j].x, dw.balloons[i].y - dw.balloons[j].y) >= 9);
    const SheetGraphics g = renderDrawing(dw, d.scene, 1, &d.state.assembly.explode);
    CHECK(g.warnings.empty());
    auto hasText = [&](const std::string& t) { return std::any_of(g.texts.begin(), g.texts.end(), [&](const SheetText& x) { return x.text == t; }); };
    CHECK(hasText("ITEM"));
    CHECK(hasText("PART NUMBER"));
    CHECK(hasText("M6x14"));
    CHECK(hasText("4"));
    const auto chain = std::count_if(g.lines.begin(), g.lines.end(), [](const SheetLine& l) { return l.style == SheetLine::Center; });
    CHECK(chain >= 6); // the shaft, the cover and four screws on their way out
    // The balloons sit outside the picture, beside the parts list, on the sheet.
    for (const DrawingBalloon& b : dw.balloons) {
        CHECK(b.x > 10 && b.y > 10 && b.x < 410 && b.y < 287);
        CHECK(!(b.x > 230 && b.y < 85)); // not on the title block or the parts list
    }
    // Trail lines off: none drawn.
    REQUIRE(run(d, "EXPLODE TRAILS OFF").ok);
    const SheetGraphics g2 = renderDrawing(dw, d.scene, 1, &d.state.assembly.explode);
    CHECK(std::count_if(g2.lines.begin(), g2.lines.end(), [](const SheetLine& l) { return l.style == SheetLine::Center; }) < chain);
    // Exported, and saved with the project.
    const std::string svg = (std::filesystem::temp_directory_path() / "sicdm_assembly.svg").string();
    REQUIRE(run(d, "SHEET EXPORT " + svg).ok);
    CHECK(std::filesystem::file_size(svg) > 1000);
    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_assembly_drawing.sicdm").string();
    REQUIRE(run(d, "SAVE \"" + path + "\"").ok);
    Document e;
    REQUIRE(run(e, "OPEN \"" + path + "\"").ok);
    CHECK(e.state.drawing.views[0].exploded);
    CHECK(e.state.drawing.partsList.shown);
    REQUIRE(e.state.drawing.balloons.size() == 4);
    CHECK(e.state.drawing.balloons[2].body == dw.balloons[2].body);
    CHECK(std::fabs(e.state.drawing.balloons[2].x - dw.balloons[2].x) < 1e-5); // the file keeps 9 digits
}

TEST_CASE("balloons: only parts that can be seen, by hand, moved and removed; exploded views take no dimensions") {
    Document d;
    REQUIRE(run(d, "BOX 60 60 60 @0,0,0").ok);
    REQUIRE(run(d, "BOX 10 10 10 @0,0,20").ok); // inside the big one: never seen together
    REQUIRE(run(d, "BOX 20 20 20 @80,0,0").ok);
    REQUIRE(run(d, "PROPERTY \"Block 2\" NUMBER INNER").ok);
    REQUIRE(run(d, "SHEET NEW").ok);
    const CommandOutput r = run(d, "SHEET BALLOONS ISOMETRIC");
    REQUIRE(r.ok);
    CHECK(d.state.drawing.balloons.size() == 1); // "Block 1" and "Block 3" are one part (Block); the inner one is hidden
    CHECK(text(r).find("hidden in Isometric") != std::string::npos);
    CHECK(!run(d, "SHEET BALLOON \"Block 2\" IN Isometric").ok);
    // Exploded, the inner block comes out and can be seen.
    REQUIRE(run(d, "EXPLODE ADD \"Block 2\" ALONG Z DISTANCE 80").ok);
    REQUIRE(run(d, "SHEET VIEW EXPLODED Isometric ON").ok);
    CHECK(d.state.drawing.balloons.empty()); // their leaders pointed at the parts' old places
    REQUIRE(run(d, "SHEET BALLOON \"Block 2\"").ok);
    REQUIRE(d.state.drawing.balloons.size() == 1);
    REQUIRE(run(d, "SHEET BALLOON \"Block 3\" AT 300,250").ok);
    CHECK(d.state.drawing.balloons[1].x == 300);
    REQUIRE(run(d, "SHEET BALLOON MOVE 1 50,250").ok);
    CHECK(d.state.drawing.balloons[0].y == 250);
    CHECK(text(run(d, "SHEET")).find("balloon 2 in Isometric: item 1 (Block 3)") != std::string::npos);
    REQUIRE(run(d, "SHEET BALLOON DELETE 2").ok);
    CHECK(d.state.drawing.balloons.size() == 1);
    // Exploded views: no dimensions, sections or callouts on the parts' moved places.
    CHECK(!run(d, "SHEET DIM HORIZONTAL Isometric 0,0,0 10,0,0").ok);
    CHECK(run(d, "SHEET AUTODIM").ok);
    CHECK(std::none_of(d.state.drawing.dims.begin(), d.state.drawing.dims.end(), [](const DrawingDim& x) { return x.view == "Isometric"; }));
    CHECK(!run(d, "SHEET VIEW EXPLODED Front ON").ok); // it has dimensions now
    REQUIRE(run(d, "SHEET VIEW ADD ISO EXPLODED").ok);
    CHECK(d.state.drawing.view("Exploded Isometric") != nullptr);
    REQUIRE(run(d, "SHEET VIEW DELETE Isometric").ok);
    CHECK(d.state.drawing.balloons.empty());
    // The parts list on and off, and moved.
    REQUIRE(run(d, "SHEET PARTSLIST AT 400,200").ok);
    CHECK(d.state.drawing.partsList.shown);
    CHECK(d.state.drawing.partsList.x == 400);
    REQUIRE(run(d, "SHEET PARTSLIST OFF").ok);
    CHECK(!d.state.drawing.partsList.shown);
    REQUIRE(run(d, "SHEET BALLOON CLEAR").ok);
    REQUIRE(run(d, "EXPLODE CLEAR").ok);
    CHECK(!run(d, "SHEET VIEW ADD ISO EXPLODED").ok);
}

TEST_CASE("MCP: explode and the assembly drawing's balloons and parts list") {
    Document d;
    buildGearbox(d);
    McpServer s(d, nullptr);
    auto tool = [&](const std::string& name, const std::string& args) {
        const auto r = Json::parse(s.handle(R"({"jsonrpc":"2.0","id":7,"method":"tools/call","params":{"name":")" + name + R"(","arguments":)" + args + "}}"));
        return r ? (*r)["result"] : Json();
    };
    CHECK(!tool("explode", R"({"operation":"auto","along":"Z"})")["isError"].asBool());
    CHECK(d.state.assembly.explode.steps.size() == 3);
    CHECK(!tool("explode", R"({"operation":"add","parts":["Block 1"],"along":"-Z","distance":20})")["isError"].asBool());
    CHECK(!tool("explode", R"({"operation":"edit","step":4,"direction":[1,0,0],"distance":5})")["isError"].asBool());
    CHECK(std::fabs(d.state.assembly.explode.steps[3].dir.x - 1) < 1e-9);
    CHECK(tool("explode", R"({"operation":"add","along":"Z","distance":5})")["isError"].asBool()); // which parts?
    CHECK(!tool("explode", R"({"operation":"delete","step":4})")["isError"].asBool());
    CHECK(!tool("drawing", R"({"operation":"assembly","size":"A3"})")["isError"].asBool());
    CHECK(d.state.drawing.balloons.size() == 4);
    CHECK(!tool("drawing", R"({"operation":"balloon_move","index":1,"at":[40,250]})")["isError"].asBool());
    CHECK(d.state.drawing.balloons[0].x == 40);
    CHECK(!tool("drawing", R"({"operation":"parts_list","on":false})")["isError"].asBool());
    CHECK(!d.state.drawing.partsList.shown);
    CHECK(!tool("drawing", R"({"operation":"view_add","view":"front","exploded":true})")["isError"].asBool());
    CHECK(d.state.drawing.view("Exploded Front") != nullptr);
    CHECK(!tool("drawing", R"({"operation":"balloons","view":"Exploded Front"})")["isError"].asBool());
    CHECK(!tool("explode", R"({"operation":"hide"})")["isError"].asBool());
    CHECK(!d.state.assembly.explode.shown);
}

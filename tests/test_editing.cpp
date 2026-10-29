#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Json.hpp"
#include "core/Mcp.hpp"
#include "core/Scene.hpp"
#include "core/Selection.hpp"

#include <algorithm>
#include <cmath>

using namespace sicdm;

#define REQUIRE(c)                  \
    do {                            \
        const bool required_ = (c); \
        CHECK(required_);           \
        if (!required_) return;     \
    } while (0)

namespace {

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }

std::vector<std::string> selectedNames(const Document& d) {
    std::vector<std::string> out;
    for (int b : d.selectedBodies()) out.push_back(d.scene.bodies()[static_cast<std::size_t>(b)].name);
    return out;
}

int bodyNamed(const Document& d, const std::string& n) {
    for (std::size_t i = 0; i < d.scene.bodies().size(); ++i)
        if (d.scene.bodies()[i].name == n) return static_cast<int>(i);
    return -1;
}

// Four boxes in a row: Block 1 ... Block 4.
void fourBlocks(Document& d) {
    for (int i = 0; i < 4; ++i) run(d, "BOX 10 10 10 @" + std::to_string(30 * i) + ",0,0");
}

} // namespace

TEST_CASE("selection: toggling, the active body, and dropping when another is picked") {
    Document d;
    fourBlocks(d);
    REQUIRE(d.scene.bodies().size() == 4);
    int active = -1;
    BodySelection sel;
    toggleBody(d.scene, active, sel, 0);
    toggleBody(d.scene, active, sel, 2);
    toggleBody(d.scene, active, sel, 3);
    CHECK(active == 3); // the last added is active
    CHECK((selectedBodies(d.scene, active, sel) == std::vector<int>{3, 2, 0})); // newest first
    toggleBody(d.scene, active, sel, 0); // out again: the active one stays
    CHECK((selectedBodies(d.scene, active, sel) == std::vector<int>{3, 2}));
    toggleBody(d.scene, active, sel, 3); // the active one out: the most recent of the rest
    CHECK(active == 2);
    CHECK((selectedBodies(d.scene, active, sel) == std::vector<int>{2}));
    // Kept by name: a body deleted before them doesn't shift them.
    selectBodies(d.scene, active, sel, {1, 2, 3});
    d.scene.remove(0);
    CHECK((selectedBodies(d.scene, 0, sel) == std::vector<int>{0, 1, 2}));
    // Anything selecting another body on its own drops the others.
    CHECK((selectedBodies(d.scene, 2, sel) == std::vector<int>{2}));
    CHECK(selectedBodies(d.scene, -1, sel).empty());
}

TEST_CASE("SELECT: several names, ADD, REMOVE, ALL, INVERT, NONE") {
    Document d;
    fourBlocks(d);
    REQUIRE(run(d, "SELECT \"Block 1\" \"Block 3\"").ok);
    CHECK((selectedNames(d) == std::vector<std::string>{"Block 1", "Block 3"}));
    REQUIRE(run(d, "SELECT ADD \"Block 4\"").ok);
    CHECK((selectedNames(d) == std::vector<std::string>{"Block 4", "Block 1", "Block 3"}));
    REQUIRE(run(d, "SELECT REMOVE \"Block 1\"").ok);
    CHECK((selectedNames(d) == std::vector<std::string>{"Block 4", "Block 3"}));
    const auto told = run(d, "SELECT");
    REQUIRE(told.ok);
    CHECK(told.lines[0].find("2 bodies") != std::string::npos);
    REQUIRE(run(d, "SELECT INVERT").ok);
    CHECK((selectedNames(d) == std::vector<std::string>{"Block 1", "Block 2"}));
    d.scene.bodies()[3].visible = false;
    REQUIRE(run(d, "SELECT ALL").ok);
    CHECK(d.selectedBodies().size() == 3); // the visible ones
    REQUIRE(run(d, "SELECT NONE").ok);
    CHECK(d.selectedBodies().empty());
    REQUIRE(run(d, "SELECT Block 2").ok); // one unquoted name, as before
    CHECK((selectedNames(d) == std::vector<std::string>{"Block 2"}));
    CHECK(!run(d, "SELECT \"Block 2\" \"Nope\"").ok);
    CHECK(!run(d, "SELECT Nothing Here").ok);
    // A new body is selected on its own: the others go.
    REQUIRE(run(d, "SELECT \"Block 1\" \"Block 2\"").ok);
    REQUIRE(run(d, "SPHERE 5 @0,40,0").ok);
    CHECK(d.selectedBodies().size() == 1);
}

TEST_CASE("ERASE: every selected body in one step, undone together, replayed by name") {
    Document d;
    fourBlocks(d);
    REQUIRE(run(d, "SELECT \"Block 2\" \"Block 4\"").ok);
    const auto r = run(d, "ERASE");
    REQUIRE(r.ok);
    CHECK(r.lines[0] == "Deleted 2 bodies");
    CHECK(d.scene.bodies().size() == 2);
    CHECK(bodyNamed(d, "Block 2") < 0);
    CHECK(bodyNamed(d, "Block 4") < 0);
    CHECK(d.selectedBodies().empty());
    // The history keeps the names, not "whatever is selected".
    REQUIRE(!d.state.history.empty());
    CHECK(d.state.history.back().line == "ERASE \"Block 2\" \"Block 4\"");
    REQUIRE(run(d, "UNDO").ok);
    CHECK(d.scene.bodies().size() == 4);
    REQUIRE(run(d, "ERASE \"Block 1\" \"Block 3\"").ok);
    CHECK(d.scene.bodies().size() == 2);
    CHECK(!run(d, "ERASE \"Block 1\"").ok); // gone
    CHECK(!run(d, "ERASE").ok);              // nothing selected
}

TEST_CASE("MCP delete: one name or several") {
    Document d;
    fourBlocks(d);
    McpServer server(d, nullptr);
    auto call = [&](const std::string& args) {
        const auto r = Json::parse(server.handle(R"({"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"delete","arguments":)" + args + "}}"));
        return r ? (*r)["result"] : Json();
    };
    CHECK(!call(R"({"names":["Block 1","Block 2"]})")["isError"].asBool());
    CHECK(d.scene.bodies().size() == 2);
    CHECK(!call(R"({"name":"Block 3"})")["isError"].asBool());
    CHECK(d.scene.bodies().size() == 1);
    CHECK(call(R"({})")["isError"].asBool());
    const auto dup = Json::parse(server.handle(R"({"jsonrpc":"2.0","id":2,"method":"tools/call","params":{"name":"duplicate","arguments":{"names":["Block 4"],"offset":[0,0,20]}}})"));
    REQUIRE(dup.has_value());
    CHECK(!(*dup)["result"]["isError"].asBool());
    CHECK(bodyNamed(d, "Block 4 copy") >= 0);
}

TEST_CASE("DUPLICATE: copies beside the originals, named, selected, undone in one step, replayed") {
    Document d;
    fourBlocks(d);
    REQUIRE(run(d, "SELECT \"Block 1\" \"Block 2\"").ok);
    const auto r = run(d, "DUPLICATE");
    REQUIRE(r.ok);
    REQUIRE(d.scene.bodies().size() == 6);
    const int c1 = bodyNamed(d, "Block 1 copy"), c2 = bodyNamed(d, "Block 2 copy");
    REQUIRE(c1 >= 0);
    REQUIRE(c2 >= 0);
    // Beside them: the pair's width (40 mm) and a tenth more along +X.
    const Vec3 shift = d.scene.bodies()[static_cast<std::size_t>(c1)].bounds().min - d.scene.bodies()[0].bounds().min;
    CHECK(std::fabs(shift.x - 44.0f) < 1e-3f);
    CHECK(std::fabs(shift.y) < 1e-6f);
    CHECK((selectedNames(d) == std::vector<std::string>{"Block 1 copy", "Block 2 copy"}));
    // Again: new names, not the same ones.
    REQUIRE(run(d, "DUPLICATE \"Block 1\" OFFSET 0,0,50").ok);
    CHECK(bodyNamed(d, "Block 1 copy 2") >= 0);
    CHECK(copyName(d.scene, "Block 1 copy") == "Block 1 copy 3"); // a copy of a copy: numbered from the original
    CHECK(copyName(d.scene, "Bracket copy holder") == "Bracket copy holder copy");
    CHECK(std::fabs(d.scene.bodies().back().bounds().min.z - 50.0f) < 1e-3f);
    REQUIRE(run(d, "UNDO").ok);
    REQUIRE(run(d, "UNDO").ok);
    CHECK(d.scene.bodies().size() == 4);
    // The copies are separate: moving one leaves the original.
    REQUIRE(run(d, "DUPLICATE \"Block 3\"").ok);
    REQUIRE(run(d, "MOVE 0,100,0 \"Block 3 copy\"").ok);
    CHECK(std::fabs(d.scene.bodies()[2].offset.y) < 1e-6f);
    CHECK(std::fabs(d.scene.bodies().back().offset.y - 100.0f) < 1e-4f);
    // REGEN replays it by name.
    REQUIRE(run(d, "REGEN").ok);
    CHECK(bodyNamed(d, "Block 3 copy") >= 0);
    CHECK(!run(d, "DUPLICATE \"Nope\"").ok);
    REQUIRE(run(d, "SELECT NONE").ok);
    CHECK(!run(d, "DUPLICATE").ok); // nothing selected
}

TEST_CASE("COPY, CUT, PASTE: in place in another file, beside the originals in the same one") {
    Document a, b;
    fourBlocks(a);
    REQUIRE(run(a, "MATERIAL \"Block 2\" Gold").ok);
    REQUIRE(run(a, "COPY \"Block 2\" \"Block 3\"").ok);
    // Another file: same names, same place.
    REQUIRE(run(b, "PASTE").ok);
    REQUIRE(b.scene.bodies().size() == 2);
    CHECK(b.scene.bodies()[0].name == "Block 2");
    CHECK(b.scene.bodies()[0].material.preset == "gold");
    CHECK(length(b.scene.bodies()[0].bounds().center() - a.scene.bodies()[1].bounds().center()) < 1e-4f);
    CHECK(b.selectedBodies().size() == 2);
    // The same file: names of their own, beside them.
    REQUIRE(run(a, "PASTE").ok);
    CHECK(bodyNamed(a, "Block 2 copy") >= 0);
    CHECK(a.scene.bodies().back().bounds().min.x > a.scene.bodies()[2].bounds().max.x);
    REQUIRE(run(a, "PASTE OFFSET 0,0,100").ok);
    CHECK(std::fabs(a.scene.bodies().back().bounds().min.z - 100.0f) < 1e-3f);
    // CUT: taken out (one undo puts them back), and pasted back in place.
    Document c;
    fourBlocks(c);
    REQUIRE(run(c, "SELECT \"Block 1\" \"Block 4\"").ok);
    REQUIRE(run(c, "CUT").ok);
    CHECK(c.scene.bodies().size() == 2);
    CHECK(c.state.history.back().line == "ERASE \"Block 1\" \"Block 4\"");
    REQUIRE(run(c, "PASTE").ok);
    CHECK(c.scene.bodies().size() == 4);
    CHECK(bodyNamed(c, "Block 1") >= 0); // the name was free again
    REQUIRE(run(c, "UNDO").ok);
    REQUIRE(run(c, "UNDO").ok);
    CHECK(c.scene.bodies().size() == 4);
    CHECK(!run(c, "PASTE \"Block 1\"").ok);
    CHECK(!run(c, "DUPLICATE OFFSET 1,2").ok);
}

TEST_CASE("RENAME: the body and everything that refers to it, undone, replayed, refused when unclear") {
    Document d;
    fourBlocks(d);
    REQUIRE(run(d, "MATE DISTANCE 5 \"Block 1\" AT 5,5,10 \"Block 2\" AT 25,5,0").ok);
    REQUIRE(run(d, "STACK NEW Blocks GAP \"Block 1\" \"Block 2\" ALONG X").ok);
    REQUIRE(run(d, "STACK Blocks ADD BODY \"Block 1\" ALONG X MINUS").ok);
    REQUIRE(run(d, "CAM SETUP \"Block 1\" STOCK 3").ok);
    REQUIRE(run(d, "SELECT \"Block 3\" \"Block 1\"").ok);
    const auto r = run(d, "RENAME \"Block 1\" TO \"Base plate\"");
    REQUIRE(r.ok);
    CHECK(r.lines[0] == "Renamed Block 1 to Base plate");
    CHECK(d.scene.bodies()[0].name == "Base plate");
    REQUIRE(!d.state.assembly.mates.empty());
    CHECK(d.state.assembly.mates[0].a.body == "Base plate");
    REQUIRE(!d.state.stacks.empty());
    CHECK(d.state.stacks[0].gapFrom == "Base plate");
    REQUIRE(!d.state.stacks[0].items.empty());
    CHECK(d.state.stacks[0].items[0].body == "Base plate");
    CHECK(d.state.cam.setup.body == "Base plate");
    CHECK((selectedNames(d) == std::vector<std::string>{"Block 3", "Base plate"})); // still both selected
    CHECK(d.state.history.back().line == "RENAME \"Block 1\" TO \"Base plate\"");
    // The selected body, unquoted words.
    REQUIRE(run(d, "SELECT \"Block 2\"").ok);
    REQUIRE(run(d, "RENAME Side plate").ok);
    CHECK(d.scene.bodies()[1].name == "Side plate");
    CHECK(d.state.assembly.mates[0].b.body == "Side plate");
    CHECK(d.state.stacks[0].gapTo == "Side plate");
    // Refused: taken, a quote, empty, no such body.
    CHECK(!run(d, "RENAME \"Block 3\" TO \"Base plate\"").ok);
    CHECK(!run(d, "RENAME \"Block 3\" TO bad\"name").ok);
    CHECK(!run(d, "RENAME \"Block 3\" TO").ok);
    CHECK(!run(d, "RENAME \"Nope\" TO Fine").ok);
    CHECK(bodyNamed(d, "Block 3") == 2);
    // Undo puts the name and the references back together.
    REQUIRE(run(d, "UNDO").ok);
    CHECK(d.scene.bodies()[1].name == "Block 2");
    CHECK(d.state.assembly.mates[0].b.body == "Block 2");
    CHECK(d.state.stacks[0].gapTo == "Block 2");
    REQUIRE(run(d, "REDO").ok);
    CHECK(d.scene.bodies()[1].name == "Side plate");
    // REGEN replays both by name; later steps on the new name still work.
    const float zBefore = d.scene.bodies()[1].offset.z; // where the mate put it
    REQUIRE(run(d, "MOVE 0,0,30 \"Side plate\"").ok);
    REQUIRE(run(d, "REGEN").ok);
    CHECK(bodyNamed(d, "Base plate") == 0);
    REQUIRE(bodyNamed(d, "Side plate") == 1);
    CHECK(std::fabs(d.scene.bodies()[1].offset.z - zBefore - 30.0f) < 1e-4f);
    // The same name again (or with spaces round it) is fine and changes nothing.
    CHECK(run(d, "RENAME \"Side plate\" TO \" Side plate \"").ok);
    CHECK(d.scene.bodies()[1].name == "Side plate");
}

TEST_CASE("MCP modify rename: references follow, a taken name is an error") {
    Document d;
    fourBlocks(d);
    REQUIRE(run(d, "STACK NEW Blocks GAP \"Block 1\" \"Block 2\" ALONG X").ok);
    McpServer server(d, nullptr);
    auto call = [&](const std::string& args) {
        const auto r = Json::parse(server.handle(R"({"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"modify","arguments":)" + args + "}}"));
        return r ? (*r)["result"] : Json();
    };
    CHECK(!call(R"({"name":"Block 2","rename":"Lid"})")["isError"].asBool());
    CHECK(d.state.stacks[0].gapTo == "Lid");
    CHECK(call(R"({"name":"Block 3","rename":"Lid"})")["isError"].asBool());
    CHECK(bodyNamed(d, "Block 3") == 2);
}

TEST_CASE("HIDE, SHOW, ISOLATE: named or selected, one undo step, replayed, left out of SELECT") {
    Document d;
    fourBlocks(d);
    auto shown = [&] {
        std::vector<std::string> out;
        for (const Body& b : d.scene.bodies())
            if (b.visible) out.push_back(b.name);
        return out;
    };
    REQUIRE(run(d, "SELECT \"Block 1\" \"Block 2\"").ok);
    const auto r = run(d, "HIDE");
    REQUIRE(r.ok);
    CHECK(r.lines[0] == "Hid 2 bodies");
    CHECK((shown() == std::vector<std::string>{"Block 3", "Block 4"}));
    CHECK(d.selectedBodies().empty()); // hidden bodies don't stay selected
    CHECK(d.state.history.back().line == "HIDE \"Block 1\" \"Block 2\"");
    REQUIRE(run(d, "SELECT ALL").ok);
    CHECK(d.selectedBodies().size() == 2); // the visible ones
    REQUIRE(run(d, "UNDO").ok);
    CHECK(shown().size() == 4);
    REQUIRE(run(d, "REDO").ok);
    CHECK(shown().size() == 2);
    REQUIRE(run(d, "SHOW \"Block 2\"").ok);
    CHECK((shown() == std::vector<std::string>{"Block 2", "Block 3", "Block 4"}));
    CHECK(run(d, "SHOW \"Block 2\"").lines[0].find("no change") != std::string::npos);
    // ISOLATE: only these; SHOW ALL: everything again.
    const auto iso = run(d, "ISOLATE \"Block 4\"");
    REQUIRE(iso.ok);
    CHECK(iso.lines[0].find("3 other bodies hidden") != std::string::npos);
    CHECK((shown() == std::vector<std::string>{"Block 4"}));
    // REGEN keeps what was hidden (the steps are in the history).
    REQUIRE(run(d, "REGEN").ok);
    CHECK((shown() == std::vector<std::string>{"Block 4"}));
    REQUIRE(run(d, "SHOW ALL").ok);
    CHECK(shown().size() == 4);
    // Refused: nothing selected, an unknown name.
    REQUIRE(run(d, "SELECT NONE").ok);
    CHECK(!run(d, "HIDE").ok);
    CHECK(!run(d, "ISOLATE").ok);
    CHECK(!run(d, "HIDE \"Nope\"").ok);
    // Selecting the others takes only what can be seen.
    REQUIRE(run(d, "HIDE \"Block 1\" \"Block 2\" \"Block 3\"").ok);
    REQUIRE(run(d, "SELECT INVERT").ok);
    CHECK((selectedNames(d) == std::vector<std::string>{"Block 4"}));
}

TEST_CASE("MCP visibility: hide, isolate, show_all") {
    Document d;
    fourBlocks(d);
    McpServer server(d, nullptr);
    auto call = [&](const std::string& args) {
        const auto r = Json::parse(server.handle(R"({"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"visibility","arguments":)" + args + "}}"));
        return r ? (*r)["result"] : Json();
    };
    CHECK(!call(R"({"action":"hide","names":["Block 1"]})")["isError"].asBool());
    CHECK(!d.scene.bodies()[0].visible);
    CHECK(!call(R"({"action":"isolate","names":["Block 3"]})")["isError"].asBool());
    CHECK(d.scene.bodies()[2].visible);
    CHECK(!d.scene.bodies()[1].visible);
    CHECK(!call(R"({"action":"show_all"})")["isError"].asBool());
    for (const Body& b : d.scene.bodies()) CHECK(b.visible);
    CHECK(call(R"({"action":"hide"})")["isError"].asBool());
    CHECK(call(R"({"action":"blink","names":["Block 1"]})")["isError"].asBool());
}

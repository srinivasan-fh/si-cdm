#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Json.hpp"
#include "core/Mcp.hpp"
#include "core/Scene.hpp"
#include "core/Selection.hpp"

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
}

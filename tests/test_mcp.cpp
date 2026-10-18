#include "TestFramework.hpp"

#include "core/Mcp.hpp"

#include <filesystem>

using namespace sicdm;

namespace {

Json call(McpServer& s, int id, const std::string& method, const std::string& params = "{}") {
    const std::string msg = R"({"jsonrpc":"2.0","id":)" + std::to_string(id) + R"(,"method":")" + method +
                            R"(","params":)" + params + "}";
    const auto r = Json::parse(s.handle(msg));
    return r ? *r : Json();
}

Json tool(McpServer& s, const std::string& name, const std::string& args) {
    return call(s, 7, "tools/call", R"({"name":")" + name + R"(","arguments":)" + args + "}")["result"];
}

} // namespace

TEST_CASE("MCP initialize negotiates the protocol and lists tools") {
    Document d;
    McpServer s(d, nullptr);
    Json r = call(s, 1, "initialize", R"({"protocolVersion":"2025-03-26","capabilities":{},"clientInfo":{"name":"t","version":"1"}})");
    CHECK(r["result"]["protocolVersion"].asString() == "2025-03-26");
    CHECK(r["result"]["serverInfo"]["name"].asString() == "si-cdm");
    CHECK(r["result"]["capabilities"]["tools"].isObject());
    r = call(s, 2, "initialize", R"({"protocolVersion":"1999-01-01"})");
    CHECK(r["result"]["protocolVersion"].asString() == "2025-06-18"); // falls back to ours
    CHECK(s.handle(R"({"jsonrpc":"2.0","method":"notifications/initialized"})").empty());
    r = call(s, 3, "tools/list");
    CHECK(r["result"]["tools"].size() >= 12);
    for (const Json& t : r["result"]["tools"].items()) {
        CHECK(!t["name"].asString().empty());
        CHECK(t["inputSchema"]["type"].asString() == "object");
    }
    CHECK(call(s, 4, "ping")["result"].isObject());
}

TEST_CASE("MCP reports protocol errors") {
    Document d;
    McpServer s(d, nullptr);
    CHECK(Json::parse(s.handle("{not json"))->operator[]("error")["code"].asInt() == -32700);
    CHECK(call(s, 1, "nope")["error"]["code"].asInt() == -32601);
    CHECK(call(s, 2, "tools/call", R"({"name":"bogus","arguments":{}})")["error"]["code"].asInt() == -32602);
    CHECK(Json::parse(s.handle(R"({"id":1,"method":"ping"})"))->operator[]("error")["code"].asInt() == -32600);
}

TEST_CASE("MCP tools build 2D and 3D geometry") {
    Document d;
    McpServer s(d, nullptr);
    Json r = tool(s, "create_solid", R"({"kind":"cylinder","radius":10,"height":30,"position":[5,0,0],"name":"Shaft","color":[1,0,0]})");
    CHECK(!r["isError"].asBool());
    CHECK(d.scene.bodies().size() == 1 && d.scene.bodies()[0].name == "Shaft");
    CHECK_NEAR(d.scene.bodies()[0].offset.x, 5, 1e-6);
    CHECK(tool(s, "create_solid", R"({"kind":"box","width":10})")["isError"].asBool()); // missing dims

    r = tool(s, "create_sketch", R"({"kind":"rectangle","points":[[0,0],[40,20]],"plane":"front"})");
    CHECK(!r["isError"].asBool());
    CHECK(d.scene.sketch().curves().size() == 1);
    CHECK(d.scene.sketch().curves()[0].plane == SketchPlane::Front);
    CHECK(d.state.plane == SketchPlane::Top); // one-off plane does not stick
    CHECK(!tool(s, "create_sketch", R"({"kind":"circle","points":[[0,0]],"radius":5})")["isError"].asBool());
    CHECK(tool(s, "create_sketch", R"({"kind":"circle","points":[[0,0]]})")["isError"].asBool());
    CHECK(s.takeDocumentChanged());

    r = tool(s, "modify", R"({"name":"Shaft","move_by":[0,0,10],"rename":"Axle"})");
    CHECK(!r["isError"].asBool());
    CHECK(d.scene.bodies()[0].name == "Axle");
    CHECK_NEAR(d.scene.bodies()[0].offset.z, 10, 1e-6);

    r = tool(s, "get_scene", "{}");
    const Json& sc = r["structuredContent"];
    CHECK(sc["bodies"].size() == 1);
    CHECK(sc["bodies"][0]["kind"].asString() == "cylinder");
    CHECK(sc["sketch_curves"].size() == 2);
    CHECK(sc["sketch_curves"][1]["radius"].asNumber() == 5.0);

    CHECK(!tool(s, "run_command", R"({"command":"LINE 0,0 10,10"})")["isError"].asBool());
    CHECK(tool(s, "run_command", R"({"command":"FROB"})")["isError"].asBool());
    CHECK(!tool(s, "delete", R"({"name":"Axle"})")["isError"].asBool());
    CHECK(d.scene.bodies().empty());
    CHECK(tool(s, "set_view", R"({"view":"top"})")["isError"].asBool()); // headless
}

TEST_CASE("MCP file tools save, export and reopen") {
    Document d;
    McpServer s(d, nullptr);
    const auto dir = std::filesystem::temp_directory_path() / "sicdm_mcp_tests";
    std::filesystem::create_directories(dir);
    const std::string proj = (dir / "agent project.sicdm").string();
    tool(s, "create_solid", R"({"kind":"sphere","radius":4})");
    CHECK(!tool(s, "save_project", R"({"path":")" + proj + R"("})")["isError"].asBool());
    CHECK(!tool(s, "export_file", R"({"path":")" + (dir / "ball.stl").string() + R"("})")["isError"].asBool());
    CHECK(!tool(s, "new_project", "{}")["isError"].asBool());
    CHECK(d.scene.bodies().empty());
    CHECK(!tool(s, "open_project", R"({"path":")" + proj + R"("})")["isError"].asBool());
    CHECK(d.scene.bodies().size() == 1);
    CHECK(s.takeDocumentReplaced());
    CHECK(!tool(s, "import_file", R"({"path":")" + (dir / "ball.stl").string() + R"("})")["isError"].asBool());
    CHECK(d.scene.bodies().size() == 2);
}

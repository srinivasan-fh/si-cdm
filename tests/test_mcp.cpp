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

TEST_CASE("MCP extrude tool") {
    Document d;
    McpServer s(d, nullptr);
    tool(s, "create_sketch", R"({"kind":"circle","points":[[0,0]],"radius":6})");
    Json r = tool(s, "extrude", R"({"profile":"Circle 1","distance":25,"name":"Shaft"})");
    CHECK(!r["isError"].asBool());
    CHECK(d.scene.bodies().size() == 1 && d.scene.bodies()[0].name == "Shaft");
    CHECK(tool(s, "extrude", R"({"profile":"Circle 1"})")["isError"].asBool());
    bool listed = false;
    const Json list = call(s, 9, "tools/list");
    for (const Json& t : list["result"]["tools"].items()) listed = listed || t["name"].asString() == "extrude";
    CHECK(listed);
}

TEST_CASE("MCP motion tools add motions and report poses headless") {
    Document d;
    McpServer s(d, nullptr);
    CHECK(!tool(s, "create_solid", R"({"kind":"box","width":40,"depth":20,"height":10,"name":"Chassis"})")["isError"].asBool());
    CHECK(!tool(s, "create_solid", R"({"kind":"cylinder","radius":5,"height":4,"name":"Front wheel"})")["isError"].asBool());
    Json r = tool(s, "create_motion", R"({"kind":"linear","name":"Drive","bodies":["*"],"axis":[1,0,0],"speed":10})");
    CHECK(!r["isError"].asBool());
    r = tool(s, "create_motion", R"({"kind":"circular","name":"Wheel spin","bodies":["Front wheel"],"axis":[0,1,0],"rpm":60,"enabled":false})");
    CHECK(!r["isError"].asBool());
    CHECK(d.scene.motions().size() == 2);
    CHECK_NEAR(d.scene.motions()[1].speed, 360, 1e-4);
    CHECK(!d.scene.motions()[1].enabled);
    CHECK(tool(s, "create_motion", R"({"kind":"spiral"})")["isError"].asBool());
    CHECK(tool(s, "create_motion", R"({"kind":"linear","bodies":["Nobody"]})")["isError"].asBool());

    const Json sc = tool(s, "get_scene", "{}")["structuredContent"];
    CHECK(sc["motions"].size() == 2);
    CHECK(sc["motions"][0]["kind"].asString() == "linear");

    r = tool(s, "simulate", R"({"action":"poses","time":2})");
    CHECK(!r["isError"].asBool());
    CHECK(r["content"][0]["text"].asString().find("20.00 mm") != std::string::npos);
    CHECK(tool(s, "simulate", R"({"action":"play"})")["isError"].asBool()); // needs the app window
    CHECK(!tool(s, "delete_motion", R"({"name":"Wheel spin"})")["isError"].asBool());
    CHECK(d.scene.motions().size() == 1);
}

TEST_CASE("MCP library tools insert, list and scale parts") {
    Document d;
    McpServer s(d, nullptr);
    Json r = tool(s, "insert_part", R"({"part":"stepper_motor","name":"X Axis","position":[0,0,0],"params":{"size":2,"rpm":15}})");
    CHECK(!r["isError"].asBool());
    CHECK(d.scene.bodies().size() == 2);
    CHECK_NEAR(d.scene.bodies()[0].scale, 2, 1e-6);
    CHECK(d.scene.motions().size() == 1);
    CHECK_NEAR(d.scene.motions()[0].speed, 90, 1e-4);
    CHECK(tool(s, "insert_part", R"({"part":"flux_capacitor"})")["isError"].asBool());
    CHECK(tool(s, "insert_part", R"({"part":"spur_gear","params":{"teeth":"many"}})")["isError"].asBool());
    CHECK(tool(s, "list_parts", "{}")["content"][0]["text"].asString().find("planetary") != std::string::npos);
    CHECK(!tool(s, "modify", R"({"name":"X Axis","scale":1})")["isError"].asBool());
    CHECK_NEAR(d.scene.bodies()[1].scale, 1, 1e-6); // the rotor scales with its component
    const Json sc = tool(s, "get_scene", "{}")["structuredContent"];
    CHECK(sc["bodies"][1]["component"].asString() == "X Axis");
}

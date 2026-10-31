#include "TestFramework.hpp"

#include "core/Cam.hpp"
#include "core/Mcp.hpp"
#include "core/Motion.hpp"
#include "core/Physics.hpp"
#include "core/Sculpt.hpp"

#include <cmath>
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
    // The rounder shapes.
    for (const char* a : {R"({"kind":"frustum","radius":20,"top_radius":8,"height":30})", R"({"kind":"tube","radius":20,"inner_radius":15,"height":30})",
                          R"({"kind":"ellipsoid","semi_axes":[20,10,8]})", R"({"kind":"capsule","radius":5,"length":40})",
                          R"({"kind":"hemisphere","radius":12})", R"({"kind":"spring","radius":10,"tube_radius":1.5,"pitch":6,"turns":4,"left_hand":true})"})
        CHECK(!tool(s, "create_solid", a)["isError"].asBool());
    CHECK(d.scene.bodies().size() == 7 && d.scene.bodies().back().kind == "spring" && d.scene.bodies().back().params[4] == 1.0f);
    CHECK(d.scene.bodies()[1].kind == "cone" && d.scene.bodies()[1].params[1] == 8.0f);
    CHECK(tool(s, "create_solid", R"({"kind":"ellipsoid","semi_axes":[1,2]})")["isError"].asBool());
    while (d.scene.bodies().size() > 1) d.scene.remove(d.scene.bodies().size() - 1);

    // The shapes from a few points.
    {
        Document e;
        McpServer m(e, nullptr);
        for (const char* a : {R"({"kind":"centre_rectangle","points":[[0,0],[10,5]]})", R"({"kind":"rectangle_3p","points":[[0,0],[10,0],[0,4]]})",
                              R"({"kind":"circle_3p","points":[[0,0],[10,0],[0,10]]})", R"({"kind":"line","points":[[50,0],[60,0]]})",
                              R"({"kind":"elliptical_arc","points":[[0,50],[20,50],[30,50],[0,70]],"radius":10})",
                              R"({"kind":"conic","points":[[0,100],[40,100],[20,130]],"rho":0.6})", R"({"kind":"text","points":[[0,150]],"text":"Ok","height":8})"})
            CHECK(!tool(m, "create_sketch", a)["isError"].asBool());
        const std::string lineName = e.scene.sketch().curves()[3].name;
        CHECK(!tool(m, "create_sketch", R"({"kind":"tangent_arc","points":[[70,10]],"curve":")" + lineName + R"("})")["isError"].asBool());
        CHECK(e.scene.sketch().curves().back().kind == CurveKind::Arc);
        CHECK(tool(m, "create_sketch", R"({"kind":"circle_3p","points":[[0,0],[1,1]]})")["isError"].asBool());
    }
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
    // generic_string(): forward slashes, which JSON takes unescaped and Windows accepts.
    const auto dir = std::filesystem::temp_directory_path() / "sicdm_mcp_tests";
    std::filesystem::create_directories(dir);
    const std::string proj = (dir / "agent project.sicdm").generic_string();
    tool(s, "create_solid", R"({"kind":"sphere","radius":4})");
    CHECK(!tool(s, "save_project", R"({"path":")" + proj + R"("})")["isError"].asBool());
    CHECK(!tool(s, "export_file", R"({"path":")" + (dir / "ball.stl").generic_string() + R"("})")["isError"].asBool());
    CHECK(!tool(s, "new_project", "{}")["isError"].asBool());
    CHECK(d.scene.bodies().empty());
    CHECK(!tool(s, "open_project", R"({"path":")" + proj + R"("})")["isError"].asBool());
    CHECK(d.scene.bodies().size() == 1);
    CHECK(s.takeDocumentReplaced());
    CHECK(!tool(s, "import_file", R"({"path":")" + (dir / "ball.stl").generic_string() + R"("})")["isError"].asBool());
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

TEST_CASE("MCP modify edits library part parameters") {
    Document d;
    McpServer s(d, nullptr);
    CHECK(!tool(s, "insert_part", R"({"part":"dc_motor","name":"M"})")["isError"].asBool());
    Json r = tool(s, "modify", R"({"name":"M","params":{"shafts":"dual","rear_length":12,"shaft_length":20}})");
    CHECK(!r["isError"].asBool());
    CHECK_NEAR(d.scene.bodies()[1].params[2], 2, 1e-6);
    CHECK_NEAR(d.scene.bodies()[1].params[3], 12, 1e-6);
    CHECK(tool(s, "modify", R"({"name":"M","params":{"shafts":"many"}})")["isError"].asBool());
}

TEST_CASE("MCP tools stay in mm while run_command follows the document units") {
    Document d;
    McpServer s(d, nullptr);
    CHECK(!tool(s, "run_command", R"({"command":"UNITS IN"})")["isError"].asBool());
    Json r = tool(s, "create_solid", R"({"kind":"cylinder","radius":10,"height":30,"position":[5,0,0]})");
    CHECK(!r["isError"].asBool());
    CHECK_NEAR(d.scene.bodies().back().params[0], 10, 1e-5);   // mm, not inches
    CHECK_NEAR(d.scene.bodies().back().offset.x, 5, 1e-5);
    CHECK(!tool(s, "run_command", R"({"command":"CYLINDER 1 2"})")["isError"].asBool());
    CHECK_NEAR(d.scene.bodies().back().params[0], 25.4, 1e-4); // inches
    CHECK(!tool(s, "run_command", R"({"command":"BOX 20mm 1cm 1/2\""})")["isError"].asBool());
    CHECK_NEAR(d.scene.bodies().back().params[5], 12.7, 1e-4);
    const Json scene = tool(s, "get_scene", "{}")["structuredContent"];
    CHECK(scene["project"]["units"].asString() == "mm");
    CHECK(scene["project"]["display_units"]["unit"].asString() == "in");
    CHECK(!d.apiMillimeters);
}

TEST_CASE("MCP set_pose and play_gesture move a library robot") {
    Document d;
    McpServer s(d, nullptr);
    CHECK(!tool(s, "insert_part", R"({"part":"humanoid","name":"Atlas"})")["isError"].asBool());
    Json r = tool(s, "set_pose", R"({"robot":"Atlas","posture":"t_pose","joints":{"right elbow":200,"neck yaw":-30},"seconds":2})");
    CHECK(!r["isError"].asBool());
    CHECK(r["content"][0]["text"].asString().find("hard stop: 150") != std::string::npos);
    float elbow = 0, yaw = 0;
    for (const Motion& m : d.scene.motions()) {
        if (m.name == "Atlas Right Arm Elbow") elbow = motionValue(d.scene, m, 5.0f);
        if (m.name == "Atlas Neck Yaw") yaw = motionValue(d.scene, m, 5.0f);
    }
    CHECK_NEAR(elbow, 150, 1e-3);
    CHECK_NEAR(yaw, -30, 1e-3);
    CHECK(tool(s, "set_pose", R"({"robot":"Atlas"})")["isError"].asBool());
    CHECK(!tool(s, "play_gesture", R"({"gesture":"wave","mirror":true,"speed":0.5})")["isError"].asBool());
    CHECK(tool(s, "play_gesture", R"({"gesture":"moonwalk"})")["isError"].asBool());
}

TEST_CASE("MCP set_physics and physics_report run an experiment") {
    Document d;
    McpServer s(d, nullptr);
    CHECK(!tool(s, "create_solid", R"({"kind":"sphere","radius":25,"position":[0,0,500],"name":"Ball"})")["isError"].asBool());
    Json r = tool(s, "set_physics", R"({"gravity":"uniform","air":false,"body":{"name":"Ball","velocity":[1000,0,0],"temperature_c":40,"restitution":0.7}})");
    CHECK(!r["isError"].asBool());
    CHECK(d.scene.physics().enabled);
    CHECK(d.scene.bodies()[0].physics.enabled);
    CHECK_NEAR(d.scene.bodies()[0].physics.velocity.x, 1000, 1e-3);
    r = tool(s, "physics_report", R"({"time":0.2,"body":"Ball"})");
    CHECK(!r["isError"].asBool());
    const std::string text = r["content"][0]["text"].asString();
    CHECK(text.find("velocity (1, 0, -1.961) m/s") != std::string::npos); // v = (1, 0, -g t)
    CHECK(text.find("Conservation of energy") != std::string::npos);
    CHECK(!tool(s, "set_physics", R"({"demo":"orbit"})")["isError"].asBool());
    CHECK(d.scene.physics().gravity == GravityMode::Newton);
    CHECK(tool(s, "set_physics", R"({"body":{"mode":"fixed"}})")["isError"].asBool());
}

TEST_CASE("MCP print checks, slices and exports a part") {
    Document d;
    McpServer s(d, nullptr);
    CHECK(!tool(s, "create_solid", R"({"kind":"box","width":20,"depth":20,"height":10,"name":"Part"})")["isError"].asBool());
    Json r = tool(s, "print", R"({"operation":"check","body":"Part"})");
    CHECK(!r["isError"].asBool());
    CHECK(r["content"][0]["text"].asString().find("ready to print") != std::string::npos);
    const std::string gcode = (std::filesystem::temp_directory_path() / "sicdm mcp slice.gcode").generic_string();
    r = tool(s, "print", R"({"operation":"slice","body":"Part","path":")" + gcode + R"(","layer":0.25,"infill":15})");
    CHECK(!r["isError"].asBool());
    CHECK(r["content"][0]["text"].asString().find("40 layers") != std::string::npos);
    CHECK(std::filesystem::exists(gcode));
    std::filesystem::remove(gcode);
    CHECK(tool(s, "print", R"({"operation":"slice","body":"Part"})")["isError"].asBool()); // no path
    CHECK(tool(s, "print", R"({"operation":"melt"})")["isError"].asBool());
}

TEST_CASE("MCP cam sets up, adds tools and operations, simulates and posts") {
    Document d;
    McpServer s(d, nullptr);
    CHECK(!tool(s, "create_solid", R"({"kind":"box","width":40,"depth":30,"height":10,"name":"Plate"})")["isError"].asBool());
    CHECK(!tool(s, "cam", R"({"operation":"setup","body":"Plate","stock_margin":3,"material":"aluminium","post":"grbl"})")["isError"].asBool());
    CHECK(!tool(s, "cam", R"({"operation":"tool","number":1,"type":"flat","diameter":6})")["isError"].asBool());
    Json r = tool(s, "cam", R"({"operation":"contour","tabs":2,"stepdown":2.5})");
    CHECK(!r["isError"].asBool());
    CHECK(r["content"][0]["text"].asString().find("Added 1. Contour") != std::string::npos);
    r = tool(s, "cam", R"({"operation":"simulate"})");
    CHECK(!r["isError"].asBool());
    CHECK(r["content"][0]["text"].asString().find("no gouges") != std::string::npos);
    const std::string nc = (std::filesystem::temp_directory_path() / "sicdm mcp cam.nc").generic_string();
    r = tool(s, "cam", R"({"operation":"post","path":")" + nc + R"("})");
    CHECK(!r["isError"].asBool());
    CHECK(std::filesystem::exists(nc));
    std::filesystem::remove(nc);
    CHECK(tool(s, "cam", R"({"operation":"juggle"})")["isError"].asBool());
    CHECK(tool(s, "cam", R"({"operation":"tool","number":2})")["isError"].asBool());
    // Simultaneous 5-axis wants a ball end mill and a machine that turns the table both ways.
    CHECK(!tool(s, "cam", R"({"operation":"tool","number":3,"type":"ball","diameter":4})")["isError"].asBool());
    CHECK(!tool(s, "cam", R"({"operation":"machine","preset":"vmc"})")["isError"].asBool());
    CHECK(tool(s, "cam", R"({"operation":"5axis","tool":3})")["isError"].asBool());
    CHECK(!tool(s, "cam", R"({"operation":"machine","preset":"trunnion"})")["isError"].asBool());
    r = tool(s, "cam", R"({"operation":"5axis","tool":3,"scallop":0.05,"max_tilt":40,"lead_angle":5,"side_tilt":-3})");
    CHECK(!r["isError"].asBool());
    CHECK(r["content"][0]["text"].asString().find("lead 5 deg, tilt -3 deg, at most 40 deg from vertical") != std::string::npos);
    CHECK(d.state.cam.ops.size() == 2);
    CHECK(d.state.cam.ops.size() == 2 && d.state.cam.ops[1].kind == CamOpKind::Surface5);
}

TEST_CASE("MCP lathe sets up a bar, adds tools and operations, simulates and posts") {
    Document d;
    McpServer s(d, nullptr);
    CHECK(!tool(s, "create_solid", R"({"kind":"cylinder","radius":10,"height":30,"name":"Pin"})")["isError"].asBool());
    Json r = tool(s, "lathe", R"({"operation":"setup","body":"Pin","material":"brass","post":"linuxcnc","bar":22})");
    CHECK(!r["isError"].asBool());
    CHECK(r["content"][0]["text"].asString().find("22 mm bar") != std::string::npos);
    CHECK(!tool(s, "lathe", R"({"operation":"tool","number":1,"type":"turn","nose":0.4,"insert":"D"})")["isError"].asBool());
    CHECK(!tool(s, "lathe", R"({"operation":"tool","number":2,"type":"groove","size":2.5})")["isError"].asBool());
    CHECK(!tool(s, "lathe", R"({"operation":"face"})")["isError"].asBool());
    r = tool(s, "lathe", R"({"operation":"rough","leave":0.2,"stepdown":1})");
    CHECK(!r["isError"].asBool());
    CHECK(r["content"][0]["text"].asString().find("Rough turn") != std::string::npos);
    CHECK(!tool(s, "lathe", R"({"operation":"finish"})")["isError"].asBool());
    CHECK(!tool(s, "lathe", R"({"operation":"partoff","peck":2})")["isError"].asBool());
    r = tool(s, "lathe", R"({"operation":"simulate"})");
    CHECK(!r["isError"].asBool());
    CHECK(r["content"][0]["text"].asString().find("no gouges") != std::string::npos);
    const std::string nc = (std::filesystem::temp_directory_path() / "sicdm mcp lathe.ngc").generic_string();
    CHECK(!tool(s, "lathe", R"({"operation":"post","path":")" + nc + R"("})")["isError"].asBool());
    CHECK(std::filesystem::exists(nc));
    std::filesystem::remove(nc);
    CHECK(tool(s, "lathe", R"({"operation":"knurl"})")["isError"].asBool());
    CHECK(d.state.lathe.ops.size() == 4);
}

TEST_CASE("MCP drawing lays out views, dimensions them and exports the sheet") {
    Document d;
    McpServer s(d, nullptr);
    CHECK(!tool(s, "create_solid", R"({"kind":"box","width":60,"depth":40,"height":15,"name":"Plate"})")["isError"].asBool());
    Json r = tool(s, "drawing", R"({"operation":"new","size":"A3","third_angle":true})");
    CHECK(!r["isError"].asBool());
    CHECK(d.state.drawing.views.size() == 4);
    CHECK(d.state.drawing.thirdAngle);
    CHECK(!tool(s, "drawing", R"({"operation":"section","view":"Front"})")["isError"].asBool());
    CHECK(!tool(s, "drawing", R"({"operation":"autodim"})")["isError"].asBool());
    CHECK(!tool(s, "drawing", R"({"operation":"callouts"})")["isError"].asBool()); // no holes yet: nothing to add
    CHECK(d.state.drawing.callouts.empty());
    CHECK(tool(s, "drawing", R"({"operation":"flat"})")["isError"].asBool()); // no sheet metal
    CHECK(tool(s, "drawing", R"({"operation":"callout_delete","index":1})")["isError"].asBool());
    CHECK(!tool(s, "drawing", R"({"operation":"dim","kind":"horizontal","view":"Front","from":[-30,0,0],"to":[0,0,0],"offset":-20})")["isError"].asBool());
    CHECK(!tool(s, "drawing", R"({"operation":"title","field":"MATERIAL","text":"Steel"})")["isError"].asBool());
    CHECK(!tool(s, "drawing", R"({"operation":"note","text":"DEBURR","at":[30,40]})")["isError"].asBool());
    r = tool(s, "drawing", R"({"operation":"list"})");
    CHECK(r["content"][0]["text"].asString().find("= 30.0 mm") != std::string::npos);
    const std::string pdf = (std::filesystem::temp_directory_path() / "sicdm mcp drawing.pdf").generic_string();
    CHECK(!tool(s, "drawing", R"({"operation":"export","path":")" + pdf + R"("})")["isError"].asBool());
    CHECK(std::filesystem::exists(pdf));
    std::filesystem::remove(pdf);
    CHECK(tool(s, "drawing", R"({"operation":"dim","kind":"horizontal"})")["isError"].asBool());
    CHECK(tool(s, "drawing", R"({"operation":"scribble"})")["isError"].asBool());

    // Tolerances and GD&T.
    const std::size_t dims = d.state.drawing.dims.size();
    CHECK(!tool(s, "drawing", R"({"operation":"dim","kind":"vertical","view":"Front","from":[30,0,0],"to":[30,0,15],"tolerance":"+0.1 -0.05"})")["isError"].asBool());
    CHECK(d.state.drawing.dims.size() == dims + 1);
    if (d.state.drawing.dims.size() != dims + 1) return;
    CHECK(d.state.drawing.dims.back().tol.kind == DimTolerance::Deviation);
    CHECK(!tool(s, "drawing", R"({"operation":"tolerance","index":1,"tolerance":"FIT H7"})")["isError"].asBool());
    CHECK(d.state.drawing.dims[0].tol.fit == "H7");
    CHECK(!tool(s, "drawing", R"({"operation":"datum","letter":"A","view":"Front","point":[-30,0,0],"sheet_offset":[-12,0]})")["isError"].asBool());
    CHECK(!tool(s, "drawing", R"({"operation":"gdt","symbol":"perpendicularity","value":0.02,"datums":["A"],"view":"Front","point":[30,0,7.5]})")["isError"].asBool());
    CHECK(!tool(s, "drawing", R"({"operation":"gdt","symbol":"position","value":0.05,"diameter":true,"material":"MMC","datums":["A"],"index":1})")["isError"].asBool());
    CHECK(tool(s, "drawing", R"({"operation":"gdt","symbol":"flatness","value":0.02,"datums":["A"],"view":"Front","point":[0,0,15]})")["isError"].asBool());
    CHECK(!tool(s, "drawing", R"({"operation":"finish","value":3.2,"machined":true,"view":"Front","point":[0,0,15]})")["isError"].asBool());
    CHECK(!tool(s, "drawing", R"({"operation":"general_tolerance","tolerance":"ISO 2768-m"})")["isError"].asBool());
    CHECK(d.state.drawing.frames.size() == 2);
    CHECK(d.state.drawing.datums.size() == 1);
    CHECK(d.state.drawing.finishes.size() == 1);
    CHECK(d.state.drawing.generalTolerance == "ISO 2768-m");
    CHECK(!tool(s, "drawing", R"({"operation":"gdt_delete","index":1})")["isError"].asBool());
    CHECK(d.state.drawing.frames.size() == 1);
}

TEST_CASE("MCP cad revolves, patterns, fillets, weighs and edits the history") {
    Document d;
    McpServer s(d, nullptr);
    CHECK(!tool(s, "run_command", R"({"command":"PLANE FRONT"})")["isError"].asBool());
    CHECK(!tool(s, "run_command", R"({"command":"RECT 10,0 @10,30"})")["isError"].asBool());
    Json r = tool(s, "cad", R"({"operation":"revolve","curve":"Rectangle 1","angle":360})");
    CHECK(!r["isError"].asBool());
    CHECK(r["content"][0]["text"].asString().find("closed") != std::string::npos);
    r = tool(s, "cad", R"({"operation":"pattern","body":"Revolve 1","type":"linear","count":3,"spacing":50,"direction":[1,0,0]})");
    CHECK(!r["isError"].asBool());
    CHECK(d.scene.bodies().size() == 3);
    r = tool(s, "cad", R"({"operation":"fillet","body":"Revolve 1","radius":2})");
    CHECK(!r["isError"].asBool());
    r = tool(s, "cad", R"({"operation":"massprops","body":"Revolve 1","material":"aluminium"})");
    CHECK(r["content"][0]["text"].asString().find("centre of mass") != std::string::npos);
    r = tool(s, "cad", R"j({"operation":"measure","body":"Revolve 1","other":"Revolve 1 (2)"})j");
    CHECK(r["content"][0]["text"].asString().find("Closest distance") != std::string::npos);
    r = tool(s, "cad", R"({"operation":"history"})");
    CHECK(r["content"][0]["text"].asString().find("REVOLVE") != std::string::npos);
    CHECK(tool(s, "cad", R"({"operation":"fillet"})")["isError"].asBool()); // no radius
    CHECK(tool(s, "cad", R"({"operation":"bend"})")["isError"].asBool());
}

TEST_CASE("MCP sculpt paints and groups a body") {
    Document d;
    McpServer s(d, nullptr);
    CHECK(!tool(s, "create_solid", R"({"kind":"box","width":40,"depth":30,"height":20,"name":"Clay"})")["isError"].asBool());
    CHECK(!tool(s, "sculpt", R"({"operation":"refine","body":"Clay","detail":4})")["isError"].asBool());
    CHECK(!tool(s, "sculpt", R"({"operation":"polygroup_auto","body":"Clay","angle":30})")["isError"].asBool());
    CHECK(groupSizes(d.scene.bodies().back()).size() == 6);
    CHECK(!tool(s, "sculpt", R"({"operation":"paint_fill","body":"Clay","color":"#ff0000","points":[[0,0,20]]})")["isError"].asBool());
    CHECK(paintedFraction(d.scene.bodies().back()) > 0.05f);
    CHECK(!tool(s, "sculpt", R"({"operation":"paint","body":"Clay","points":[[20,0,10]],"radius":5,"strength":1,"color":"blue"})")["isError"].asBool());
    CHECK(!tool(s, "sculpt", R"({"operation":"mask_group","body":"Clay","group":0})")["isError"].asBool());
    CHECK(maskedFraction(d.scene.bodies().back()) > 0.0f);
    CHECK(tool(s, "sculpt", R"({"operation":"paint_fill","body":"Clay"})")["isError"].asBool()); // no colour
    CHECK(!tool(s, "sculpt", R"({"operation":"paint_clear","body":"Clay"})")["isError"].asBool());
    CHECK(paintedFraction(d.scene.bodies().back()) == 0.0f);
    CHECK(!tool(s, "sculpt", R"({"operation":"polygroup_clear","body":"Clay"})")["isError"].asBool());
    CHECK(!d.scene.bodies().back().mesh.hasGroups());
}

TEST_CASE("MCP sculpt records into layers and dials them") {
    Document d;
    McpServer s(d, nullptr);
    CHECK(!tool(s, "create_solid", R"({"kind":"sphere","radius":20,"name":"Head"})")["isError"].asBool());
    CHECK(!tool(s, "sculpt", R"({"operation":"refine","body":"Head","detail":3})")["isError"].asBool());
    CHECK(!tool(s, "sculpt", R"({"operation":"layer_new","body":"Head","layer":"Brow"})")["isError"].asBool());
    const float top0 = d.scene.bodies()[0].bounds().max.z;
    CHECK(!tool(s, "sculpt", R"({"operation":"draw","body":"Head","points":[[0,0,40]],"radius":8,"strength":1})")["isError"].asBool());
    const float top1 = d.scene.bodies()[0].bounds().max.z;
    CHECK(top1 > top0 + 0.5f);
    CHECK(!tool(s, "sculpt", R"({"operation":"layer_strength","body":"Head","layer":"Brow","value":0})")["isError"].asBool());
    CHECK(std::fabs(d.scene.bodies()[0].bounds().max.z - top0) < 0.1f);
    CHECK(tool(s, "sculpt", R"({"operation":"layer_strength","body":"Head","layer":"Brow"})")["isError"].asBool()); // no value
    CHECK(!tool(s, "sculpt", R"({"operation":"layer_on","body":"Head","layer":"Brow"})")["isError"].asBool());
    CHECK(!tool(s, "sculpt", R"({"operation":"layer_list","body":"Head"})")["isError"].asBool());
    CHECK(!tool(s, "sculpt", R"({"operation":"layer_bake","body":"Head"})")["isError"].asBool());
    CHECK(d.scene.bodies()[0].mesh.layers.empty());
}

#include "TestFramework.hpp"

#include "core/Assembly.hpp"
#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Mcp.hpp"
#include "core/Sculpt.hpp"
#include "core/Weldment.hpp"

#include <cmath>
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

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }

std::string text(const CommandOutput& o) {
    std::string s;
    for (const std::string& l : o.lines) s += l + "\n";
    return s;
}

bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

const Body* named(const Document& d, const std::string& n) {
    for (const Body& b : d.scene.bodies())
        if (b.name == n) return &b;
    return nullptr;
}

double polyArea(const std::string& profile) { return weldProfile(profile)->polygonArea; }

// The member's length tip to tip along its line (the most any of its points runs).
double tipToTip(const Body& b) {
    const Vec3 d = normalize(b.member.end - b.member.start);
    float lo = 1e30f, hi = -1e30f;
    for (const Vertex& v : b.mesh.vertices) lo = std::min(lo, dot(v.position, d)), hi = std::max(hi, dot(v.position, d));
    return hi - lo;
}

} // namespace

TEST_CASE("weldment profiles: the standard sections' areas, any size by its dimensions, nonsense refused") {
    // Areas as the standards' tables give them (cm^2).
    const std::pair<const char*, double> tables[] = {{"SHS 40x40x3", 4.21}, {"RHS 100x50x4", 10.95}, {"SHS 100x100x8", 27.2}, {"CHS 48.3x3.2", 4.53},
                                                     {"IPE 200", 28.48},   {"IPE 300", 53.81},     {"UPE 120", 15.4},     {"FLAT 40x6", 2.40},
                                                     {"ROUND 20", 3.1416}};
    for (const auto& [name, cm2] : tables) {
        std::string why;
        const auto p = weldProfile(name, &why);
        REQUIRE(p.has_value());
        CHECK(near(p->area / 100, cm2, cm2 * 0.006));
        CHECK(p->designation == name);
    }
    CHECK(weldProfile("L50x50x5")->designation == "ANGLE 50x50x5");
    CHECK(weldProfile("shs40x40x3")->designation == "SHS 40x40x3");
    CHECK(weldProfile("SHS 45x2.5")->designation == "SHS 45x45x2.5");
    CHECK(weldProfile("I 200x100x6x9")->hollow() == false);
    CHECK(weldProfile("CHS 60.3x4")->hollow());
    CHECK(weldProfile("CHS 60.3x4")->inner.size() == weldProfile("CHS 60.3x4")->outer.size());
    std::string why;
    CHECK(!weldProfile("SHS 40x40x30", &why).has_value());
    CHECK(!weldProfile("IPE 210", &why).has_value());
    CHECK(why.find("standard size") != std::string::npos);
    CHECK(!weldProfile("HEB 200", &why).has_value());
    CHECK(why.find("No profile family") != std::string::npos);
    CHECK(!weldProfileSizes("IPE").empty());
}

TEST_CASE("weldment: a rectangle of tube mitred at its corners, exact, in the cut list and the parts list") {
    Document d;
    REQUIRE(run(d, "RECT 0,0 1000,600").ok);
    const CommandOutput o = run(d, "WELDMENT MEMBER SHS 40x40x3 ALONG \"Rectangle 1\"");
    REQUIRE(o.ok);
    CHECK(text(o).find("Added 4 members") != std::string::npos);
    REQUIRE(d.scene.bodies().size() == 4);
    double vol = 0;
    for (const Body& b : d.scene.bodies()) {
        CHECK(b.isMember());
        CHECK(b.assembly == "Weldment 1");
        CHECK(b.exact());
        CHECK(openEdgeCount(b.mesh) == 0);
        vol += meshVolume(b.mesh);
    }
    CHECK(near(vol, polyArea("SHS 40x40x3") * 3200, 1)); // mitres through the line: the line's length
    CHECK(near(tipToTip(*named(d, "Member 1")), 1040, 1e-3));
    CHECK(near(tipToTip(*named(d, "Member 2")), 640, 1e-3));
    const auto items = cutList(d.scene);
    REQUIRE(items.size() == 2);
    CHECK(items[0].members.size() == 2);
    CHECK(near(items[0].length, 1040, 1e-3));
    CHECK(near(items[0].angle0, 45, 1e-3) && near(items[0].angle1, 45, 1e-3));
    CHECK(near(items[0].massEach, 4.21e-4 * 7.85 * 1000, 0.01)); // the line's 1000 mm (kg)
    // The parts list: two items, two of each, described.
    const auto bom = partsList(d.scene);
    REQUIRE(bom.size() == 2);
    CHECK(bom[0].quantity == 2);
    CHECK(bom[0].number == "Weldment 1-1");
    CHECK(bom[0].description.find("SHS 40x40x3, 1040 mm") != std::string::npos);
    const CommandOutput cut = run(d, "WELDMENT CUTLIST");
    REQUIRE(cut.ok);
    CHECK(text(cut).find("2 x 1040 mm, ends 45 and 45 deg off square") != std::string::npos);
    CHECK(text(cut).find("3.36 m in all") != std::string::npos);
    // Undo takes the four away together; redo brings them back.
    REQUIRE(run(d, "UNDO").ok);
    CHECK(d.scene.bodies().empty());
    REQUIRE(run(d, "REDO").ok);
    CHECK(d.scene.bodies().size() == 4);
    // Rebuilt from the history: the same frame.
    REQUIRE(run(d, "REGEN").ok);
    REQUIRE(d.scene.bodies().size() == 4);
    CHECK(near(tipToTip(*named(d, "Member 1")), 1040, 1e-3));
    CHECK(cutList(d.scene).size() == 2);
}

TEST_CASE("weldment: butt joints, a cross member cut to the sides it meets, a joint changed") {
    Document d;
    REQUIRE(run(d, "PLINE 0,0 500,0 500,300").ok);
    REQUIRE(run(d, "WELDMENT MEMBER SHS 40x40x3 ALONG \"Polyline 1\" CORNER BUTT").ok);
    // The first runs through to the second's far face; the second stops at the first.
    CHECK(near(tipToTip(*named(d, "Member 1")), 520, 1e-3));
    CHECK(near(tipToTip(*named(d, "Member 2")), 280, 1e-3));
    CHECK(near(meshVolume(named(d, "Member 2")->mesh), polyArea("SHS 40x40x3") * 280, 1));
    // The joint the other way round.
    REQUIRE(run(d, "WELDMENT JOINT 500,0,0 THROUGH \"Member 2\"").ok);
    CHECK(near(tipToTip(*named(d, "Member 1")), 480, 1e-3));
    CHECK(near(tipToTip(*named(d, "Member 2")), 320, 1e-3));
    REQUIRE(run(d, "WELDMENT JOINT 500,0,0 MITER").ok);
    CHECK(near(tipToTip(*named(d, "Member 1")), 520, 1e-3));
    CHECK(near(tipToTip(*named(d, "Member 2")), 320, 1e-3));
    CHECK(!run(d, "WELDMENT JOINT 900,0,0 MITER").ok); // no member ends there

    // A frame with a cross member between the middles of its long sides: a T at each end.
    Document f;
    REQUIRE(run(f, "RECT 0,0 1000,600").ok);
    REQUIRE(run(f, "WELDMENT MEMBER RHS 60x40x3 ALONG \"Rectangle 1\"").ok);
    REQUIRE(run(f, "LINE 500,0 500,600").ok);
    REQUIRE(run(f, "WELDMENT MEMBER SHS 40x40x3 ALONG \"" + f.scene.sketch().curves().back().name + "\"").ok);
    const Body* cross = named(f, "Member 5");
    REQUIRE(cross != nullptr);
    // RHS 60x40 on the Top plane: 60 wide in the plane, so the cross member stops 30 short each end.
    CHECK(near(tipToTip(*cross), 540, 1e-3));
    CHECK(near(meshVolume(cross->mesh), polyArea("SHS 40x40x3") * 540, 1));
    const auto items = cutList(f.scene);
    CHECK(items.size() == 3);
    // Legs up to the frame's corners: the frame keeps its mitres, the legs stop under it.
    REQUIRE(run(f, "WELDMENT MEMBER SHS 40x40x3 FROM 0,0,-700 TO 0,0,0").ok);
    const Body* leg = named(f, "Member 6");
    REQUIRE(leg != nullptr);
    CHECK(near(tipToTip(*leg), 680, 1e-3)); // up to the RHS's underside (40 high: 20 below its line)
    CHECK(near(tipToTip(*named(f, "Member 1")), 1060, 1e-3)); // still mitred
}

TEST_CASE("weldment: members by points, aligned, turned and mirrored; moved and erased members free their neighbours") {
    Document d;
    // A post up from the origin: round bar.
    REQUIRE(run(d, "WELDMENT MEMBER ROUND 20 FROM 0,0,0 TO 0,0,500").ok);
    CHECK(near(meshVolume(d.scene.bodies()[0].mesh), polyArea("ROUND 20") * 500, 2));
    // A flat bar lying on the line (its bottom on it).
    REQUIRE(run(d, "WELDMENT MEMBER FLAT 40x6 FROM 100,0,0 TO 300,0,0 ALIGN bottom GROUP Bars").ok);
    const AABB box = d.scene.bodies()[1].bounds();
    CHECK(near(box.min.z, 0, 1e-4) && near(box.max.z, 6, 1e-4));
    CHECK(d.scene.bodies()[1].assembly == "Bars");
    // An angle turned and mirrored: still a closed solid of the right volume.
    REQUIRE(run(d, "WELDMENT MEMBER ANGLE 50x50x5 FROM 0,200,0 TO 400,200,0 ANGLE 90 MIRROR GROUP Bars").ok);
    const Body& ang = d.scene.bodies()[2];
    CHECK(openEdgeCount(ang.mesh) == 0);
    CHECK(near(meshVolume(ang.mesh), polyArea("ANGLE 50x50x5") * 400, 1));
    CHECK(ang.exact());
    // A mitred corner of two; moving one apart squares the other's end, undo mitres it again.
    REQUIRE(run(d, "WELDMENT MEMBER SHS 30x30x2 FROM 0,500,0 TO 300,500,0 TO 300,800,0 GROUP Corner").ok);
    const std::size_t a = d.scene.bodies().size() - 2, b = a + 1;
    CHECK(near(tipToTip(d.scene.bodies()[a]), 315, 1e-3));
    REQUIRE(run(d, "MOVE 0,0,100 \"" + d.scene.bodies()[b].name + "\"").ok);
    CHECK(near(tipToTip(d.scene.bodies()[a]), 300, 1e-3));
    REQUIRE(run(d, "UNDO").ok);
    CHECK(near(tipToTip(d.scene.bodies()[a]), 315, 1e-3));
    const std::string bName = d.scene.bodies()[b].name;
    REQUIRE(run(d, "ERASE \"" + bName + "\"").ok);
    CHECK(near(tipToTip(d.scene.bodies()[a]), 300, 1e-3));
    REQUIRE(run(d, "UNDO").ok);
    CHECK(near(tipToTip(d.scene.bodies()[a]), 315, 1e-3));
    // A different profile: remade, the corner mitred to the new one.
    REQUIRE(run(d, "WELDMENT EDIT \"" + bName + "\" PROFILE \"SHS 50x50x3\"").ok);
    CHECK(named(d, bName)->member.profile == "SHS 50x50x3");
    CHECK(openEdgeCount(named(d, bName)->mesh) == 0);
    // Nonsense refused.
    CHECK(!run(d, "WELDMENT MEMBER SHS 40x40x3").ok); // which line?
    CHECK(!run(d, "WELDMENT MEMBER HEB 200 FROM 0,0,0 TO 1,0,0").ok);
    CHECK(!run(d, "WELDMENT MEMBER SHS 40x40x3 FROM 0,0,0 TO 100,0,0 ALIGN sideways").ok);
    REQUIRE(run(d, "CIRCLE 0,0 50").ok);
    CHECK(!run(d, "WELDMENT MEMBER SHS 40x40x3 ALONG \"Circle 1\"").ok);
}

TEST_CASE("weldment: a drilled member keeps its ends; saved and opened; the cut list to a file; released; over MCP") {
    const fs::path dir = fs::temp_directory_path() / "sicdm_weldment";
    std::error_code ec;
    fs::create_directories(dir, ec);
    Document d;
    REQUIRE(run(d, "RECT 0,0 800,400").ok);
    REQUIRE(run(d, "WELDMENT MEMBER SHS 40x40x3 ALONG \"Rectangle 1\"").ok);
    // A hole through the bottom member: it is changed since made.
    REQUIRE(run(d, "HOLE \"Member 1\" AT 400,0,20 DIR 0,0,-1 DIAMETER 8 THROUGH").ok);
    const double drilled = meshVolume(named(d, "Member 1")->mesh);
    // Moving a neighbour: the drilled one's end stays as it was (and says so).
    const CommandOutput moved = run(d, "WELDMENT JOINT 800,0,0 SQUARE");
    REQUIRE(moved.ok);
    CHECK(text(moved).find("changed since") != std::string::npos);
    CHECK(near(meshVolume(named(d, "Member 1")->mesh), drilled, 1e-6));
    CHECK(cutList(d.scene).size() >= 2); // still counted
    const std::string path = (dir / "frame.sicdm").string();
    REQUIRE(run(d, "SAVE \"" + path + "\"").ok);
    Document e;
    REQUIRE(run(e, "OPEN \"" + path + "\"").ok);
    REQUIRE(named(e, "Member 2") != nullptr);
    CHECK(named(e, "Member 2")->member.profile == "SHS 40x40x3");
    CHECK(named(e, "Member 2")->member.key == meshKey(named(e, "Member 2")->mesh)); // as made: still remade by its joints
    CHECK(named(e, "Member 1")->member.key != meshKey(named(e, "Member 1")->mesh)); // drilled: kept
    CHECK(cutList(e.scene).size() == cutList(d.scene).size());
    const std::string csv = (dir / "cut.csv").string();
    REQUIRE(run(e, "WELDMENT CUTLIST EXPORT \"" + csv + "\"").ok);
    std::ifstream in(csv);
    std::stringstream ss;
    ss << in.rdbuf();
    CHECK(ss.str().find("Item,Quantity,Profile,Length (mm)") == 0);
    CHECK(ss.str().find("\"SHS 40x40x3\"") != std::string::npos);
    // A released revision: looked at, not changed.
    REQUIRE(run(e, "REVISION RELEASE").ok);
    CHECK(run(e, "WELDMENT CUTLIST").ok);
    CHECK(run(e, "WELDMENT LIST").ok);
    CHECK(!run(e, "WELDMENT MEMBER FLAT 40x6 FROM 0,0,0 TO 100,0,0").ok);
    // Over MCP.
    Document m;
    McpServer s(m, nullptr);
    const auto tool = [&](const std::string& args) {
        const std::string msg = R"({"jsonrpc":"2.0","id":7,"method":"tools/call","params":{"name":"weldment","arguments":)" + args + "}}";
        const auto r = Json::parse(s.handle(msg));
        return r ? (*r)["result"] : Json();
    };
    Json r = tool(R"({"operation":"member","profile":"IPE 200","points":[[0,0,0],[2000,0,0]]})");
    CHECK(!r["isError"].asBool());
    REQUIRE(m.scene.bodies().size() == 1);
    CHECK(near(meshVolume(m.scene.bodies()[0].mesh), polyArea("IPE 200") * 2000, 5));
    r = tool(R"({"operation":"cutlist"})");
    CHECK(r["content"][0]["text"].asString().find("IPE 200") != std::string::npos);
    CHECK(!tool(R"({"operation":"profiles","profile":"UPE"})")["isError"].asBool());
    CHECK(tool(R"({"operation":"member","profile":"IPE 200"})")["isError"].asBool());
    CHECK(tool(R"({"operation":"weld"})")["isError"].asBool());
}

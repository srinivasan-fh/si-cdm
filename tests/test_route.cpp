#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Json.hpp"
#include "core/Mcp.hpp"
#include "core/Route.hpp"
#include "core/Sculpt.hpp"

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

const SketchCurve* curve(const Document& d, const std::string& n) {
    for (const SketchCurve& c : d.scene.sketch().curves())
        if (c.name == n) return &c;
    return nullptr;
}

const double kPiD = 3.14159265358979323846;
// A 48-sided section's share of the round one's area.
const double kFacets = 48.0 / (2 * kPiD) * std::sin(2 * kPiD / 48);

} // namespace

TEST_CASE("route sizes: EN 10255 and ASME B36.10M pipes, tubes, bars, cables, hoses; nonsense refused") {
    auto dn20 = routeSpec("pipe dn20");
    REQUIRE(dn20.has_value());
    CHECK(dn20->od == 26.9f && dn20->wall == 2.6f);
    CHECK(dn20->designation == "Pipe DN20 (26.9 x 2.6)");
    CHECK(routeSpec(dn20->spec)->designation == dn20->designation); // reads back
    CHECK(near(dn20->defaultBend(), 1.5 * 26.9, 1e-4));
    auto nps = routeSpec("PIPE NPS 3/4 SCH 80");
    REQUIRE(nps.has_value());
    CHECK(nps->od == 26.7f && nps->wall == 3.91f);
    CHECK(routeSpec("PIPE NPS 2")->wall == 3.91f); // schedule 40 when not given
    CHECK(routeSpec("PIPE NPS 1-1/2 SCH XS")->wall == 5.08f);
    CHECK(routeSpec("PIPE DN 150")->od == 165.1f);
    CHECK(routeSpec("pipe 30x2")->designation == "Pipe 30 x 2");
    CHECK(routeSpec("TUBE 12x1")->kind == "tube");
    CHECK(routeSpec("ROD 10")->wall == 0);
    CHECK(routeSpec("CABLE 8")->flexible);
    CHECK(near(routeSpec("CABLE 8")->defaultBend(), 48, 1e-4));
    CHECK(routeSpec("HOSE 19x3.5")->flexible);
    std::string why;
    CHECK(!routeSpec("PIPE DN200", &why).has_value());
    CHECK(why.find("DN150") != std::string::npos);
    CHECK(!routeSpec("PIPE NPS 3/4 SCH 160").has_value());
    CHECK(!routeSpec("TUBE 12x6").has_value()); // the wall fills it
    CHECK(!routeSpec("TUBE 12").has_value());
    CHECK(!routeSpec("ROPE 10").has_value());
    CHECK(!routeSpec("").has_value());
    CHECK(routeSizes("pipe").size() == 14 + 2 * 15);
    CHECK(!routeSizes("hose").empty());
}

TEST_CASE("3D sketch: a polyline bent at its corners, its points edited, undone; flat tools refuse it") {
    Document d;
    REQUIRE(run(d, "PATH3D 0,0,0 500,0,0 @0,400,0 @0,0,300 BEND 50").ok);
    const SketchCurve* c = curve(d, "3D Polyline 1");
    REQUIRE(c != nullptr);
    CHECK(c->spatial());
    CHECK(c->radius == 50.0f);
    bool closed = true;
    const std::vector<Vec3> p = spacePoints(*c, &closed);
    REQUIRE(p.size() == 4);
    CHECK(!closed);
    CHECK(near(p[3].z, 300, 1e-4) && near(p[3].y, 400, 1e-4));
    // The line it draws: straights and quarter circles, 1000 + 2 x (pi/2 x 50) long.
    const std::vector<Vec3> strip = c->worldStrip(96);
    double len = 0;
    for (std::size_t i = 1; i < strip.size(); ++i) len += length(strip[i] - strip[i - 1]);
    CHECK(near(len, 1000 + kPiD * 50, 0.05));
    // Not on any plane: no profile, no constraints, no flat edits.
    CHECK(!run(d, "EXTRUDE \"3D Polyline 1\" 10").ok);
    CHECK(!run(d, "CONSTRAIN HORIZONTAL \"3D Polyline 1\"").ok);
    CHECK(!run(d, "OFFSET \"3D Polyline 1\" 5").ok);
    CHECK(!run(d, "EXTEND \"3D Polyline 1\" END").ok);
    CHECK(!run(d, "SKETCHFILLET \"3D Polyline 1\" 5").ok);
    // Its points: moved, added, taken out; a change the bends can't take refused.
    REQUIRE(run(d, "PATH3D POINT \"3D Polyline 1\" 4 @0,0,100").ok);
    CHECK(near(curve(d, "3D Polyline 1")->point3(3).z, 400, 1e-4));
    REQUIRE(run(d, "PATH3D INSERT \"3D Polyline 1\" 5 500,700,400").ok);
    CHECK(spacePoints(*curve(d, "3D Polyline 1")).size() == 5);
    REQUIRE(run(d, "PATH3D DELETE \"3D Polyline 1\" 5").ok);
    const CommandOutput tight = run(d, "PATH3D BEND \"3D Polyline 1\" 500");
    CHECK(!tight.ok);
    CHECK(text(tight).find("need") != std::string::npos);
    CHECK(curve(d, "3D Polyline 1")->radius == 50.0f); // unchanged
    const CommandOutput info = run(d, "PATH3D INFO \"3D Polyline 1\"");
    REQUIRE(info.ok);
    CHECK(text(info).find("turns 90 deg") != std::string::npos);
    // Undo goes back point by point.
    REQUIRE(run(d, "UNDO").ok); // the delete
    CHECK(spacePoints(*curve(d, "3D Polyline 1")).size() == 5);
    REQUIRE(run(d, "UNDO").ok);
    REQUIRE(run(d, "UNDO").ok);
    CHECK(near(curve(d, "3D Polyline 1")->point3(3).z, 300, 1e-4));
    // A flat curve isn't PATH3D's to edit; bad paths refused.
    REQUIRE(run(d, "LINE 0,0 10,0").ok);
    CHECK(!run(d, "PATH3D POINT \"Line 2\" 1 0,0,5").ok);
    CHECK(!run(d, "PATH3D 0,0,0").ok);
    CHECK(!run(d, "PATH3D 0,0,0 100,0,0 0,0,0 BEND 5").ok); // turns back on itself
    CHECK(!run(d, "PATH3D 0,0,0 10,0,0 10,10,0 BEND 50").ok); // the bend doesn't fit
    CHECK(!run(d, "PATH3D 0,0,0 10,0,0 10,10,0 SPLINE BEND 5").ok);
    CHECK(!run(d, "PATH3D @1,1,1 10,0,0").ok);
    // A reference point names a point.
    REQUIRE(run(d, "REFPOINT Port AT 100,100,100").ok);
    REQUIRE(run(d, "PATH3D 0,0,0 port SPLINE").ok);
    CHECK(near(curve(d, "3D Spline 3")->point3(1).z, 100, 1e-4));
    CHECK(curve(d, "3D Spline 3")->kind == CurveKind::Spline);
}

TEST_CASE("route: a pipe along a 3D path, exact, its bend table and length; updated when the path moves; REGEN") {
    Document d;
    REQUIRE(run(d, "PATH3D 0,0,0 500,0,0 500,400,0 500,400,300 BEND 50").ok);
    const CommandOutput o = run(d, "ROUTE \"3D Polyline 1\" PIPE DN20");
    REQUIRE(o.ok);
    CHECK(text(o).find("Added Pipe 1: Pipe DN20 (26.9 x 2.6)") != std::string::npos);
    const Body* b = named(d, "Pipe 1");
    REQUIRE(b != nullptr);
    CHECK(b->isRoute());
    CHECK(b->exact());
    CHECK(openEdgeCount(b->mesh) == 0);
    const double L = 1000 + kPiD * 50; // straights and two quarter bends
    const double area = kPiD * (13.45 * 13.45 - 10.85 * 10.85);
    CHECK(near(meshVolume(b->mesh), area * L * kFacets, area * L * 0.003));
    // The bend table: feed, rotate, bend.
    const RouteReport rep = routeReport(b->route);
    REQUIRE(rep.bends.size() == 2);
    CHECK(near(rep.bends[0].feed, 450, 1e-3) && near(rep.bends[0].angle, 90, 1e-3) && near(rep.bends[0].rotation, 0, 1e-6));
    CHECK(near(rep.bends[1].feed, 300, 1e-3) && near(std::fabs(rep.bends[1].rotation), 90, 1e-3));
    CHECK(near(rep.lastFeed, 250, 1e-3));
    CHECK(near(rep.length, L, 1e-3));
    CHECK(near(rep.mass, area * L * 7.85e-6, 1e-6));
    CHECK(near(length(rep.bends[1].corner - Vec3{500, 400, 0}), 0, 1e-3));
    CHECK(b->properties.at("DESCRIPTION").find("Pipe DN20 (26.9 x 2.6), 1157.08 mm, 2 bends") == 0);
    const CommandOutput bends = run(d, "ROUTE BENDS \"Pipe 1\"");
    REQUIRE(bends.ok);
    CHECK(text(bends).find("1. feed 450 mm, rotate 0 deg, bend 90 deg") != std::string::npos);
    CHECK(text(bends).find("then feed 250 mm") != std::string::npos);
    // The path changed: the route says so, UPDATE refits it, UNDO takes that back.
    REQUIRE(run(d, "PATH3D POINT \"3D Polyline 1\" 4 500,400,600").ok);
    CHECK(text(run(d, "ROUTE LIST")).find("its path changed") != std::string::npos);
    REQUIRE(run(d, "ROUTE UPDATE \"Pipe 1\"").ok);
    CHECK(near(routeReport(named(d, "Pipe 1")->route).lastFeed, 550, 1e-3));
    CHECK(text(run(d, "ROUTE LIST")).find("its path changed") == std::string::npos);
    REQUIRE(run(d, "UNDO").ok);
    CHECK(near(routeReport(named(d, "Pipe 1")->route).lastFeed, 250, 1e-3));
    REQUIRE(run(d, "REDO").ok);
    // A bigger size, tighter bends: the table follows.
    REQUIRE(run(d, "ROUTE EDIT \"Pipe 1\" PIPE NPS 1 SCH 80 BEND 60 MATERIAL stainless").ok);
    b = named(d, "Pipe 1");
    CHECK(b->route.od == 33.4f && b->route.bend == 60.0f && b->route.material == "Stainless");
    CHECK(near(routeReport(b->route).bends[0].feed, 440, 1e-3));
    CHECK(!run(d, "ROUTE EDIT \"Pipe 1\" BEND 10").ok); // tighter than the pipe itself
    // Rebuilt from the history: the same route.
    REQUIRE(run(d, "REGEN").ok);
    b = named(d, "Pipe 1");
    REQUIRE(b != nullptr);
    CHECK(b->route.od == 33.4f);
    CHECK(near(routeReport(b->route).lastFeed, 540, 1e-3));
    // Nonsense refused.
    CHECK(!run(d, "ROUTE \"3D Polyline 1\"").ok); // no size
    CHECK(!run(d, "ROUTE \"No Such Path\" PIPE DN20").ok);
    CHECK(!run(d, "ROUTE \"3D Polyline 1\" PIPE DN20 BEND 5").ok);
    REQUIRE(run(d, "CIRCLE 0,0 50").ok);
    CHECK(!run(d, "ROUTE \"Circle 2\" ROD 5").ok);
    CHECK(!run(d, "ROUTE BENDS \"No Route\"").ok);
}

TEST_CASE("route: a cable along a 3D spline, a closed tube loop, a flat polyline; swept and framed along 3D paths") {
    Document d;
    REQUIRE(run(d, "PATH3D 0,0,0 100,0,50 200,100,0 300,0,80 SPLINE").ok);
    REQUIRE(run(d, "ROUTE \"3D Spline 1\" CABLE 8").ok);
    const Body* cable = named(d, "Cable 1");
    REQUIRE(cable != nullptr);
    CHECK(cable->route.spline);
    CHECK(cable->route.material == "Plastic");
    CHECK(openEdgeCount(cable->mesh) == 0);
    const RouteReport rc = routeReport(cable->route);
    CHECK(rc.bends.empty());
    CHECK(near(meshVolume(cable->mesh), kPiD * 16 * rc.length * kFacets, kPiD * 16 * rc.length * 0.004));
    CHECK(!run(d, "ROUTE BENDS \"Cable 1\"").ok); // a spline has no bend table
    // A closed loop: no ends, four bends in one plane (no rotation between them).
    REQUIRE(run(d, "PATH3D 0,0,0 200,0,0 200,200,100 0,200,100 CLOSE BEND 20").ok);
    REQUIRE(run(d, "ROUTE \"3D Polyline 2\" TUBE 10x1").ok);
    const Body* loop = named(d, "Tube 1");
    REQUIRE(loop != nullptr);
    CHECK(loop->route.closed);
    CHECK(openEdgeCount(loop->mesh) == 0);
    CHECK(loop->exact());
    const RouteReport rl = routeReport(loop->route);
    REQUIRE(rl.bends.size() == 4);
    for (const RouteBend& r : rl.bends) CHECK(near(r.rotation, 0, 1e-3) && near(r.angle, 90, 1e-3));
    const double side = std::sqrt(200.0 * 200 + 100 * 100);
    CHECK(near(rl.length, 2 * (200 - 40) + 2 * (side - 40) + 2 * kPiD * 20, 1e-3));
    // A flat polyline is a path too (bent at 2 x OD).
    REQUIRE(run(d, "PLINE 0,0 100,0 100,100").ok);
    REQUIRE(run(d, "ROUTE \"Polyline 3\" ROD 6").ok);
    CHECK(named(d, "Rod 1")->route.bend == 12.0f);
    // A sweep and a weldment frame follow a 3D path.
    REQUIRE(run(d, "PATH3D 0,0,0 300,0,0 300,0,200 BEND 30").ok);
    REQUIRE(run(d, "CIRCLE 0,0 5").ok);
    REQUIRE(run(d, "SWEEP \"Circle 5\" ALONG \"3D Polyline 4\" KEEP").ok);
    CHECK(openEdgeCount(d.scene.bodies().back().mesh) == 0);
    REQUIRE(run(d, "WELDMENT MEMBER \"SHS 20x20x2\" ALONG \"3D Polyline 4\"").ok);
    const Body* m2 = named(d, "Member 2");
    REQUIRE(m2 != nullptr);
    CHECK(near(length(m2->member.start - Vec3{300, 0, 0}), 0, 1e-3) && near(length(m2->member.end - Vec3{300, 0, 200}), 0, 1e-3));
    CHECK(text(run(d, "ROUTE LIST")).find("Mass:") != std::string::npos);
}

TEST_CASE("route: saved and opened (format 11), the bend table to a file, released, over MCP") {
    const fs::path dir = fs::temp_directory_path() / "sicdm_route";
    std::error_code ec;
    fs::create_directories(dir, ec);
    Document d;
    REQUIRE(run(d, "PATH3D 0,0,0 300,0,0 300,0,200 300,250,200 BEND 40").ok);
    REQUIRE(run(d, "ROUTE \"3D Polyline 1\" PIPE NPS 1 SCH 80").ok);
    REQUIRE(run(d, "PATH3D 0,0,0 50,50,50 100,0,0 SPLINE").ok);
    const std::string path = (dir / "routes.sicdm").string();
    REQUIRE(run(d, "SAVE \"" + path + "\"").ok);
    {
        std::ifstream in(path);
        std::stringstream ss;
        ss << in.rdbuf();
        const auto j = Json::parse(ss.str());
        REQUIRE(j.has_value());
        CHECK((*j)["reader_version"].asInt() == 11); // an older reader would lay the 3D curves flat
    }
    Document e;
    REQUIRE(run(e, "OPEN \"" + path + "\"").ok);
    const SketchCurve* c = curve(e, "3D Polyline 1");
    REQUIRE(c != nullptr);
    CHECK(c->spatial() && c->radius == 40.0f);
    CHECK(near(c->point3(3).z, 200, 1e-4));
    REQUIRE(curve(e, "3D Spline 2") != nullptr);
    CHECK(curve(e, "3D Spline 2")->spatial());
    const Body* b = named(e, "Pipe 1");
    REQUIRE(b != nullptr);
    CHECK(b->route.spec == "PIPE NPS 1 SCH 80");
    CHECK(b->route.key == meshKey(b->mesh));
    CHECK(near(routeReport(b->route).length, routeReport(named(d, "Pipe 1")->route).length, 1e-6));
    CHECK(text(run(e, "ROUTE LIST")).find("its path changed") == std::string::npos);
    const std::string csv = (dir / "bends.csv").string();
    REQUIRE(run(e, "ROUTE BENDS \"Pipe 1\" EXPORT \"" + csv + "\"").ok);
    std::ifstream in(csv);
    std::stringstream ss;
    ss << in.rdbuf();
    CHECK(ss.str().find("bend,feed_mm,rotation_deg,angle_deg") == 0);
    CHECK(ss.str().find("\n1,260,0,90,40,") != std::string::npos);
    CHECK(ss.str().find("\nend,210,") != std::string::npos);
    // A released revision: looked at, not changed.
    REQUIRE(run(e, "REVISION RELEASE").ok);
    CHECK(run(e, "ROUTE LIST").ok);
    CHECK(run(e, "ROUTE BENDS \"Pipe 1\"").ok);
    CHECK(run(e, "PATH3D INFO \"3D Polyline 1\"").ok);
    CHECK(!run(e, "ROUTE \"3D Polyline 1\" TUBE 12x1").ok);
    CHECK(!run(e, "PATH3D POINT \"3D Polyline 1\" 1 0,0,10").ok);
    // Over MCP.
    Document m;
    McpServer s(m, nullptr);
    const auto tool = [&](const std::string& args) {
        const std::string msg = R"({"jsonrpc":"2.0","id":7,"method":"tools/call","params":{"name":"route","arguments":)" + args + "}}";
        const auto r = Json::parse(s.handle(msg));
        return r ? (*r)["result"] : Json();
    };
    CHECK(!tool(R"({"operation":"path","points":[[0,0,0],[400,0,0],[400,300,0]],"bend":60})")["isError"].asBool());
    CHECK(!tool(R"({"operation":"path_point","curve":"3D Polyline 1","index":3,"point":[0,0,100],"relative":true})")["isError"].asBool());
    Json r = tool(R"({"operation":"route","curve":"3D Polyline 1","size":"TUBE 20x2"})");
    CHECK(!r["isError"].asBool());
    REQUIRE(m.scene.bodies().size() == 1);
    CHECK(m.scene.bodies()[0].route.points.back().z == 100.0f);
    r = tool(R"({"operation":"bends","name":"Tube 1"})");
    CHECK(r["content"][0]["text"].asString().find("bend 90 deg") != std::string::npos);
    CHECK(!tool(R"({"operation":"sizes","size":"PIPE"})")["isError"].asBool());
    CHECK(tool(R"({"operation":"route","curve":"3D Polyline 1"})")["isError"].asBool());
    CHECK(tool(R"({"operation":"bend"})")["isError"].asBool());
}

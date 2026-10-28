#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/DirectEdit.hpp"
#include "core/Document.hpp"
#include "core/Mcp.hpp"
#include "core/Sculpt.hpp"

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

const Body& last(const Document& d) { return d.scene.bodies().back(); }
double volume(const Document& d) { return meshVolume(last(d).mesh); }
bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

} // namespace

TEST_CASE("direct editing: a flat face pushed and pulled, its neighbours following at any angle") {
    Document d;
    REQUIRE(run(d, "BOX 40 30 20 @0,0,0").ok);
    DirectFaceInfo f;
    REQUIRE(directFaceAt(last(d), {0, 0, 20}, f));
    CHECK(f.kind == DirectFaceKind::Plane);
    CHECK(f.exact);
    CHECK(near(f.area, 1200, 1e-6));
    CHECK(near(f.normal.z, 1, 1e-9));
    REQUIRE(run(d, "FACE OFFSET AT 0,0,20 DISTANCE 10").ok); // the top up 10
    CHECK(near(volume(d), 40.0 * 30 * 30, 1e-3));
    REQUIRE(run(d, "FACE OFFSET AT 20,0,15 DISTANCE -5").ok); // a side in 5
    CHECK(near(volume(d), 35.0 * 30 * 30, 1e-3));
    CHECK(last(d).exact());
    // A wedge's base down 5: its slope runs on (a 50 x 25 triangle, 30 deep).
    REQUIRE(run(d, "WEDGE 40 30 20 @0,100,0").ok);
    REQUIRE(run(d, "FACE OFFSET AT 0,100,0 DISTANCE 5").ok);
    CHECK(near(volume(d), 0.5 * 50 * 25 * 30, 1e-2));
    CHECK(last(d).exact());
    // A cylinder's end pulled: its side runs on along its axis; then its radius.
    REQUIRE(run(d, "CYLINDER 10 30 @200,0,0").ok);
    const double v0 = volume(d);
    REQUIRE(run(d, "FACE OFFSET AT 200,0,30 DISTANCE 10").ok);
    CHECK(near(volume(d), v0 * 40 / 30, 1e-3 * v0));
    CHECK(last(d).exact());
    REQUIRE(run(d, "FACE OFFSET AT 210,0,15 DISTANCE 2").ok);
    REQUIRE(directFaceAt(last(d), {212, 0, 15}, f));
    CHECK(f.kind == DirectFaceKind::Cylinder);
    CHECK(near(f.radius, 12, 1e-9));
    CHECK(near(volume(d), v0 * 40 / 30 * 1.44, 3e-3 * v0));
    // A sphere bigger: still a sphere.
    REQUIRE(run(d, "SPHERE 10 @400,0,0").ok);
    REQUIRE(run(d, "FACE OFFSET AT 400,0,20 DISTANCE 5").ok);
    CHECK(last(d).exact());
    REQUIRE(directFaceAt(last(d), {400, 0, 25}, f));
    CHECK(near(f.radius, 15, 1e-9));
    // Too far: a face round it would turn inside out; nothing changes.
    REQUIRE(run(d, "WEDGE 40 30 20 @0,300,0").ok);
    const double w = volume(d);
    CHECK(!run(d, "FACE OFFSET AT 0,300,0 DISTANCE -30").ok);
    CHECK(near(volume(d), w, 1e-6));
}

TEST_CASE("direct editing: holes and bosses made larger, moved and taken away, exactly") {
    Document d;
    REQUIRE(run(d, "BOX 60 60 20 @0,0,0").ok);
    REQUIRE(run(d, "HOLE \"Block 1\" AT 0,0,20 DIR 0,0,-1 DIAMETER 10 THROUGH").ok);
    DirectFaceInfo f;
    REQUIRE(directFaceAt(last(d), {5, 0, 10}, f));
    CHECK(f.kind == DirectFaceKind::Cylinder);
    CHECK(f.hole);
    CHECK(near(f.radius, 5, 1e-9));
    REQUIRE(run(d, "FACE OFFSET AT 5,0,10 DISTANCE -2").ok); // out from the hole: larger
    REQUIRE(directFaceAt(last(d), {7, 0, 10}, f));
    CHECK(near(f.radius, 7, 1e-9));
    REQUIRE(run(d, "FACE MOVE AT 7,0,10 BY 15,0,0").ok);
    REQUIRE(directFaceAt(last(d), {22, 0, 10}, f));
    CHECK(near(f.point.x, 15, 1e-6));
    CHECK(near(f.radius, 7, 1e-9));
    CHECK(last(d).exact());
    CHECK(openEdgeCount(last(d).mesh) == 0);
    REQUIRE(run(d, "FACE DELETE AT 22,0,10").ok); // filled: the block again
    CHECK(near(volume(d), 72000, 1e-2));
    CHECK(last(d).exact());
    // A blind hole: its bottom goes with it.
    REQUIRE(run(d, "BOX 60 60 20 @0,200,0").ok);
    REQUIRE(run(d, "HOLE \"Block 2\" AT 0,200,20 DIR 0,0,-1 DIAMETER 10 DEPTH 8").ok);
    REQUIRE(run(d, "FACE DELETE AT 5,200,16").ok);
    CHECK(near(volume(d), 72000, 1e-2));
    // A boss: larger, moved, taken away.
    REQUIRE(run(d, "BOX 60 60 20 @200,0,0").ok);
    REQUIRE(run(d, "BOSS \"Block 3\" AT 200,0,20 DIR 0,0,1 DIAMETER 16 HEIGHT 10").ok);
    REQUIRE(directFaceAt(last(d), {208, 0, 22}, f));
    CHECK(!f.hole);
    REQUIRE(run(d, "FACE OFFSET AT 208,0,22 DISTANCE 2").ok);
    REQUIRE(run(d, "FACE MOVE AT 210,0,22 BY 0,15,0").ok);
    CHECK(last(d).exact());
    REQUIRE(run(d, "FACE DELETE AT 210,15,22").ok);
    CHECK(near(volume(d), 72000, 1e-2));
    // Undo takes each step back.
    REQUIRE(run(d, "UNDO").ok);
    REQUIRE(directFaceAt(last(d), {210, 15, 22}, f));
    CHECK(f.kind == DirectFaceKind::Cylinder);
}

TEST_CASE("direct editing: fillets and chamfers taken off, the faces meeting again") {
    Document d;
    REQUIRE(run(d, "BOX 40 40 20 @100,0,0").ok);
    REQUIRE(run(d, "FILLET \"Block 1\" RADIUS 5 EDGES NEAR 120,0,20").ok);
    DirectFaceInfo f;
    REQUIRE(directFaceAt(last(d), {118.5, 0, 18.5}, f));
    CHECK(f.kind == DirectFaceKind::Cylinder);
    CHECK(!f.hole);
    REQUIRE(run(d, "FACE DELETE AT 118.5,0,18.5").ok);
    CHECK(near(volume(d), 32000, 1e-2));
    CHECK(openEdgeCount(last(d).mesh) == 0);
    CHECK(last(d).exact());
    // A filleted block's top pulled up: the fillet goes with it, still a fillet.
    REQUIRE(run(d, "BOX 40 40 20 @100,100,0").ok);
    REQUIRE(run(d, "FILLET \"Block 2\" RADIUS 5 EDGES NEAR 120,100,20").ok);
    const double filleted = volume(d);
    REQUIRE(run(d, "FACE OFFSET AT 100,100,20 DISTANCE 8").ok);
    CHECK(near(volume(d), filleted + 40.0 * 40 * 8, 1e-2));
    REQUIRE(directFaceAt(last(d), {118.5, 100, 26.5}, f));
    CHECK(f.kind == DirectFaceKind::Cylinder);
    CHECK(near(f.radius, 5, 1e-9));
    CHECK(last(d).exact());
    REQUIRE(run(d, "FACE OFFSET AT 100,100,28 DISTANCE -12").ok); // and pushed down past where it was
    CHECK(near(volume(d), filleted - 40.0 * 40 * 4, 1e-2));
    CHECK(openEdgeCount(last(d).mesh) == 0);
    REQUIRE(run(d, "BOX 40 40 20 @200,200,0").ok);
    REQUIRE(run(d, "CHAMFER \"Block 3\" DISTANCE 4 EDGES NEAR 220,200,20").ok);
    REQUIRE(run(d, "FACE DELETE AT 218,200,18").ok);
    CHECK(near(volume(d), 32000, 1e-2));
    // A plain flat face can't be deleted (it can be offset).
    CHECK(!run(d, "FACE DELETE AT 200,200,20").ok);
    // Saved and opened: still the solid it was.
    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_direct.sicdm").string();
    REQUIRE(run(d, "SAVE \"" + path + "\"").ok);
    Document e;
    REQUIRE(run(e, "OPEN \"" + path + "\"").ok);
    CHECK(near(volume(e), 32000, 1e-2));
}

TEST_CASE("direct editing: what can't be done is refused and says so") {
    Document d;
    CHECK(!run(d, "FACE OFFSET AT 0,0,0 DISTANCE 5").ok); // no body
    REQUIRE(run(d, "BOX 20 20 20 @0,0,0").ok);
    CHECK(!run(d, "FACE OFFSET AT 0,0,20").ok);                // how far?
    CHECK(!run(d, "FACE OFFSET AT 0,0,80 DISTANCE 5").ok);     // nowhere near a face
    CHECK(!run(d, "FACE MOVE AT 0,0,20 BY 5,0,0").ok);         // along the face: nothing to do
    CHECK(!run(d, "FACE OFFSET \"Nothing\" AT 0,0,20 DISTANCE 5").ok);
    const CommandOutput info = run(d, "FACE INFO AT 0,0,20");
    REQUIRE(info.ok);
    CHECK(info.lines[0].find("flat face") != std::string::npos);
    // A flat face moved along its normal by the part of the move that way.
    REQUIRE(run(d, "FACE MOVE AT 0,0,20 BY 3,0,4").ok);
    CHECK(near(volume(d), 20.0 * 20 * 24, 1e-3));
    // A released revision can't be changed.
    REQUIRE(run(d, "REVISION RELEASE").ok);
    CHECK(!run(d, "FACE OFFSET AT 0,0,24 DISTANCE 1").ok);
    CHECK(run(d, "FACE INFO AT 0,0,24").ok);
}

TEST_CASE("MCP: face info, offset, move and delete") {
    Document d;
    McpServer s(d, nullptr);
    const auto tool = [&](const std::string& args) {
        const std::string msg = R"({"jsonrpc":"2.0","id":7,"method":"tools/call","params":{"name":"face","arguments":)" + args + "}}";
        const auto r = Json::parse(s.handle(msg));
        return r ? (*r)["result"] : Json();
    };
    REQUIRE(run(d, "BOX 60 60 20 @0,0,0").ok);
    REQUIRE(run(d, "HOLE \"Block 1\" AT 0,0,20 DIR 0,0,-1 DIAMETER 10 THROUGH").ok);
    Json r = tool(R"({"operation":"info","at":[5,0,10]})");
    CHECK(!r["isError"].asBool());
    CHECK(r["content"][0]["text"].asString().find("hole") != std::string::npos);
    CHECK(!tool(R"({"operation":"offset","body":"Block 1","at":[20,20,20],"distance":5})")["isError"].asBool());
    CHECK(near(volume(d), 60.0 * 60 * 25 - 3.14159265 * 25 * 25, 20.0)); // the hole is faceted
    CHECK(!tool(R"({"operation":"move","at":[5,0,10],"by":[10,0,0]})")["isError"].asBool());
    CHECK(!tool(R"({"operation":"delete","at":[15,0,10]})")["isError"].asBool());
    CHECK(near(volume(d), 60.0 * 60 * 25, 1e-2));
    CHECK(tool(R"({"operation":"offset","at":[0,0,25]})")["isError"].asBool()); // no distance
    CHECK(tool(R"({"operation":"bend","at":[0,0,25]})")["isError"].asBool());
    CHECK(tool(R"({"operation":"info"})")["isError"].asBool()); // no point
}

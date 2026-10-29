#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Json.hpp"
#include "core/Mcp.hpp"
#include "core/Mold.hpp"
#include "core/Sculpt.hpp"

#include <cmath>
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

// A mould that closes round the part: both halves solid, and with the part they fill the block.
bool closes(const MoldHalves& h) {
    const double vc = meshVolume(h.cavity), vk = meshVolume(h.core);
    return openEdgeCount(h.cavity) == 0 && openEdgeCount(h.core) == 0 && vc > 0 && vk > 0 && std::fabs(vc + vk + h.part - h.block) <= 1e-5 * h.block;
}

AABB boundsOf(const MeshData& m) { return m.bounds(); }

} // namespace

TEST_CASE("mold draft: faces by their draft along the pull, the least draft, undercuts found") {
    Document d;
    REQUIRE(run(d, "RECT 0,0 80,50").ok);
    REQUIRE(run(d, "EXTRUDE \"Rectangle 1\" 30 DRAFT 3").ok);
    const MeshData tapered = worldMesh(*named(d, "Extrude 1"));
    DraftReport rep;
    std::vector<DraftClass> k = draftClasses(tapered, {0, 0, 1}, 1.0f, &rep);
    CHECK(rep.area[static_cast<int>(DraftClass::Shallow)] == 0);
    CHECK(rep.area[static_cast<int>(DraftClass::Undercut)] == 0);
    CHECK(near(rep.minPositive, 3, 1e-3)); // the sides lean in by 3 degrees going up
    CHECK(near(rep.area[static_cast<int>(DraftClass::Negative)], 80 * 50, 1e-3)); // the bottom
    draftClasses(tapered, {0, 0, 1}, 5.0f, &rep); // asking for 5: the sides need more
    CHECK(rep.area[static_cast<int>(DraftClass::Shallow)] > 0);
    draftClasses(tapered, {0, 0, -1}, 1.0f, &rep); // pulled the other way: the sides go with the core
    CHECK(near(rep.area[static_cast<int>(DraftClass::Positive)], 80 * 50, 1e-3));
    CHECK(near(rep.minNegative, 3, 1e-3));
    // A block with a pocket under a roof: the pocket's floor faces up into the roof.
    REQUIRE(run(d, "BOX 60 60 30 @300,0,0").ok);
    REQUIRE(run(d, "BOX 30 30 10 @300,0,10").ok);
    REQUIRE(run(d, "BOOLEAN \"Block 2\" SUBTRACT \"Block 3\"").ok);
    k = draftClasses(worldMesh(*named(d, "Block 2")), {0, 0, 1}, 1.0f, &rep);
    CHECK(near(rep.area[static_cast<int>(DraftClass::Undercut)], 2 * 30 * 30, 1));
    // Shown in colours, in place of the part; MOLD OFF puts it back; undo too.
    const CommandOutput o = run(d, "MOLD DRAFT \"Extrude 1\" MIN 2");
    REQUIRE(o.ok);
    CHECK(text(o).find("least draft 3 deg") != std::string::npos);
    const Body* shown = named(d, "Extrude 1 draft 1");
    REQUIRE(shown != nullptr);
    CHECK(shown->field.rfind("draft", 0) == 0);
    CHECK(!named(d, "Extrude 1")->visible);
    CHECK(painted(shown->mesh.vertices.front().color));
    REQUIRE(run(d, "MOLD OFF").ok);
    CHECK(named(d, "Extrude 1 draft 1") == nullptr);
    CHECK(named(d, "Extrude 1")->visible);
    REQUIRE(run(d, "MOLD DRAFT \"Block 2\"").ok);
    CHECK(text(run(d, "MOLD DRAFT \"Block 2\" PULL -Z")).find("purple, undercut (can't come away): 0%") == std::string::npos);
    REQUIRE(run(d, "ANALYZE OFF").ok); // the analyses off together
    CHECK(named(d, "Block 2")->visible);
    CHECK(!run(d, "MOLD DRAFT \"Block 2\" PULL 0,0,0").ok);
    CHECK(!run(d, "MOLD DRAFT \"Block 2\" MIN 60").ok);
    CHECK(!run(d, "MOLD DRAFT \"No Part\"").ok);
    CHECK(!run(d, "MOLD MELT").ok);
}

TEST_CASE("mold parting line: round a box's base, a sphere's equator, both rims of a hole; given a height; as 3D paths, undone") {
    Document d;
    REQUIRE(run(d, "BOX 100 60 40").ok);
    PartingLine pl;
    std::string why;
    REQUIRE(partingLine(worldMesh(*named(d, "Block 1")), {0, 0, 1}, std::nullopt, pl, &why));
    REQUIRE(pl.loops.size() == 1);
    CHECK(near(pl.length, 320, 1e-3));
    CHECK(near(pl.low, 0, 1e-4) && near(pl.high, 0, 1e-4)); // its upright sides go with the cavity
    REQUIRE(partingLine(worldMesh(*named(d, "Block 1")), {0, 0, 1}, 25.0, pl, &why));
    CHECK(near(pl.low, 25, 1e-4) && near(pl.high, 25, 1e-4) && near(pl.length, 320, 1e-3));
    // A sphere parts round its middle.
    REQUIRE(run(d, "SPHERE 30 @200,0,0").ok);
    const MeshData ball = worldMesh(*named(d, "Sphere 2"));
    const AABB bb = ball.bounds();
    REQUIRE(partingLine(ball, {0, 0, 1}, std::nullopt, pl, &why));
    REQUIRE(pl.loops.size() == 1);
    CHECK(near(pl.height, 0.5 * (bb.min.z + bb.max.z), 1e-3));
    CHECK(near(pl.length, 2 * 3.14159265 * 30, 0.01 * 2 * 3.14159265 * 30));
    // A plate with a hole through: round the outside and round the hole.
    REQUIRE(run(d, "BOX 60 60 20 @400,0,0").ok);
    REQUIRE(run(d, "HOLE \"Block 3\" AT 400,0,20 DIR 0,0,-1 DIAMETER 20 THROUGH").ok);
    const CommandOutput o = run(d, "MOLD PARTING \"Block 3\"");
    REQUIRE(o.ok);
    CHECK(text(o).find("2 loops") != std::string::npos);
    const SketchCurve* c = curve(d, "Parting Line 1");
    REQUIRE(c != nullptr);
    CHECK(c->spatial());
    CHECK(c->isClosed());
    REQUIRE(curve(d, "Parting Line 2") != nullptr);
    REQUIRE(run(d, "UNDO").ok); // one step takes both away
    CHECK(curve(d, "Parting Line 1") == nullptr && curve(d, "Parting Line 2") == nullptr);
    // An open surface has no parting line.
    REQUIRE(run(d, "RECT 0,0 10,10").ok);
    REQUIRE(run(d, "SURFACE PLANAR \"Rectangle 1\"").ok);
    const CommandOutput open = run(d, "MOLD PARTING");
    CHECK(!open.ok);
    CHECK(text(open).find("solid") != std::string::npos);
}

TEST_CASE("mold split: cavity and core close round the part and fill the block, for plain, round, cupped and stepped parts") {
    Document d;
    // A box parted at its base: the core is a plate, the cavity holds the whole box.
    REQUIRE(run(d, "BOX 100 60 40").ok);
    MoldHalves h;
    std::string why;
    MoldOptions o;
    REQUIRE(splitMold(worldMesh(*named(d, "Block 1")), o, h, &why));
    CHECK(closes(h));
    CHECK(near(boundsOf(h.core).max.z, 0, 1e-4));
    CHECK(near(boundsOf(h.cavity).min.z, 0, 1e-4));
    CHECK(near(h.block, (100 + 50) * (60 + 50) * (40 + 50), 1)); // a quarter of the largest size round it
    // Parted halfway up its sides, and above its top (shut-off walls down to the plane).
    o.at = 20;
    REQUIRE(splitMold(worldMesh(*named(d, "Block 1")), o, h, &why));
    CHECK(closes(h));
    CHECK(near(meshVolume(h.cavity), meshVolume(h.core), 1));
    o.at = 55;
    REQUIRE(splitMold(worldMesh(*named(d, "Block 1")), o, h, &why));
    CHECK(closes(h));
    o.at = 500; // outside the block
    CHECK(!splitMold(worldMesh(*named(d, "Block 1")), o, h, &why));
    // Round parts, any pull: a sphere's halves alike; a cylinder on its side parts at its axis.
    REQUIRE(run(d, "SPHERE 30 @300,0,0").ok);
    o = MoldOptions{};
    o.pull = normalize(Vec3{1, 1, 1});
    REQUIRE(splitMold(worldMesh(*named(d, "Sphere 2")), o, h, &why));
    CHECK(closes(h));
    CHECK(near(meshVolume(h.cavity), meshVolume(h.core), 1e-6 * h.block + 1));
    REQUIRE(run(d, "CYLINDER 20 50 @600,0,0").ok);
    o = MoldOptions{};
    o.pull = {1, 0, 0};
    REQUIRE(splitMold(worldMesh(*named(d, "Cylinder 3")), o, h, &why));
    CHECK(closes(h));
    CHECK(near(h.parting.height, 600, 1e-3));
    CHECK(near(h.parting.low, h.parting.high, 1e-3)); // flat: no shut-offs
    // A cup: the core rises into it.
    REQUIRE(run(d, "BOX 80 80 40 @0,300,0").ok);
    REQUIRE(run(d, "HOLE \"Block 4\" AT 0,300,40 DIR 0,0,-1 DIAMETER 50 DEPTH 30").ok);
    o = MoldOptions{};
    o.pull = {0, 0, -1}; // the open side down: the cavity comes off underneath
    REQUIRE(splitMold(worldMesh(*named(d, "Block 4")), o, h, &why));
    CHECK(closes(h));
    // A T: along its stem, and across it both ways (a flat parting line at the stem's face).
    REQUIRE(run(d, "BOX 100 20 10 @300,300,40").ok);
    REQUIRE(run(d, "BOX 20 20 40 @300,300,0").ok);
    REQUIRE(run(d, "BOOLEAN \"Block 5\" UNION \"Block 6\"").ok);
    for (const Vec3 pull : {Vec3{0, 0, 1}, Vec3{1, 0, 0}, Vec3{-1, 0, 0}, Vec3{0, 1, 0}}) {
        o = MoldOptions{};
        o.pull = pull;
        REQUIRE(splitMold(worldMesh(*named(d, "Block 5")), o, h, &why));
        CHECK(closes(h));
        CHECK(near(h.parting.low, h.parting.high, 1e-3));
    }
    // Booleans and roundings leave their faces a hair off (walls not quite upright, slivers,
    // points a hair apart): still parted cleanly. A chamfered block along every axis; a
    // rounded one along the pull its base is square to.
    REQUIRE(run(d, "BOX 80 60 30 @0,900,0").ok);
    REQUIRE(run(d, "CHAMFER \"Block 7\" DISTANCE 4").ok);
    for (const Vec3 pull : {Vec3{0, 0, 1}, Vec3{1, 0, 0}, Vec3{0, 1, 0}, normalize(Vec3{1, 1, 1})}) {
        o = MoldOptions{};
        o.pull = pull;
        REQUIRE(splitMold(worldMesh(*named(d, "Block 7")), o, h, &why));
        CHECK(closes(h));
    }
    REQUIRE(run(d, "BOX 80 60 30 @300,900,0").ok);
    REQUIRE(run(d, "FILLET \"Block 8\" RADIUS 5 EDGES ALL").ok);
    o = MoldOptions{};
    REQUIRE(splitMold(worldMesh(*named(d, "Block 8")), o, h, &why));
    CHECK(closes(h));
    // Where the bottom rounding meets the sides, or up them where that line runs through the
    // rounded corners' crumbs.
    CHECK(h.parting.height > 5 - 1e-3 && h.parting.height < 25 + 1e-3);
    o.at = 15;
    REQUIRE(splitMold(worldMesh(*named(d, "Block 8")), o, h, &why));
    CHECK(closes(h));
    CHECK(near(meshVolume(h.cavity), meshVolume(h.core), 1e-5 * h.block));
    // Undercuts refused, with where to look.
    REQUIRE(run(d, "BOX 60 60 30 @0,600,0").ok);
    REQUIRE(run(d, "BOX 30 30 10 @0,600,10").ok);
    REQUIRE(run(d, "BOOLEAN \"Block 9\" SUBTRACT \"Block 10\"").ok);
    o = MoldOptions{};
    CHECK(!splitMold(worldMesh(*named(d, "Block 9")), o, h, &why));
    CHECK(why.find("undercuts") != std::string::npos);
    CHECK(h.undercut > 0);
}

TEST_CASE("mold split command: shrinkage, a margin, the cavity shown open, one undo step, REGEN, over MCP") {
    Document d;
    REQUIRE(run(d, "SPHERE 25").ok);
    const double part = meshVolume(named(d, "Sphere 1")->mesh);
    const CommandOutput o = run(d, "MOLD SPLIT \"Sphere 1\" SHRINK 2 MARGIN 10 GAP 40");
    REQUIRE(o.ok);
    CHECK(text(o).find("Cavity 1 and Core 1 in Mold 1") != std::string::npos);
    CHECK(text(o).find("grown 2% for shrinkage") != std::string::npos);
    const Body* cav = named(d, "Cavity 1");
    const Body* core = named(d, "Core 1");
    REQUIRE(cav != nullptr && core != nullptr);
    CHECK(cav->assembly == "Mold 1" && core->assembly == "Mold 1");
    CHECK(near(cav->offset.z, 40, 1e-6)); // shown open
    CHECK(openEdgeCount(cav->mesh) == 0 && openEdgeCount(core->mesh) == 0);
    const AABB box = core->mesh.bounds(), top = cav->mesh.bounds();
    const AABB ball = named(d, "Sphere 1")->mesh.bounds();
    CHECK(near(box.max.x - box.min.x, (ball.max.x - ball.min.x) * 1.02 + 20, 1e-3)); // the grown sphere and a 10 mm wall
    const double block = static_cast<double>(box.max.x - box.min.x) * (box.max.y - box.min.y) * (top.max.z - box.min.z);
    CHECK(near(meshVolume(cav->mesh) + meshVolume(core->mesh) + part * std::pow(1.02, 3), block, 1e-5 * block));
    CHECK(core->properties.at("MATERIAL") == "Tool steel");
    // A second mould goes in its own group; undo takes both halves away at once.
    REQUIRE(run(d, "MOLD SPLIT \"Sphere 1\"").ok);
    CHECK(named(d, "Cavity 2")->assembly == "Mold 2");
    REQUIRE(run(d, "UNDO").ok);
    CHECK(named(d, "Cavity 2") == nullptr && named(d, "Core 2") == nullptr);
    CHECK(named(d, "Cavity 1") != nullptr);
    // Rebuilt from the history.
    REQUIRE(run(d, "REGEN").ok);
    REQUIRE(named(d, "Cavity 1") != nullptr);
    CHECK(near(named(d, "Cavity 1")->offset.z, 40, 1e-6));
    // Refused: no part, nonsense numbers.
    CHECK(!run(d, "MOLD SPLIT \"Sphere 1\" SHRINK 50").ok);
    CHECK(!run(d, "MOLD SPLIT \"Sphere 1\" MARGIN -1").ok);
    CHECK(!run(d, "MOLD SPLIT \"Sphere 1\" AT").ok);
    // Over MCP.
    Document m;
    McpServer s(m, nullptr);
    const auto tool = [&](const std::string& args) {
        const std::string msg = R"({"jsonrpc":"2.0","id":7,"method":"tools/call","params":{"name":"mold","arguments":)" + args + "}}";
        const auto r = Json::parse(s.handle(msg));
        return r ? (*r)["result"] : Json();
    };
    REQUIRE(run(m, "BOX 40 40 20").ok);
    Json r = tool(R"({"operation":"draft","body":"Block 1","pull":"Z"})");
    CHECK(r["content"][0]["text"].asString().find("yellow") != std::string::npos);
    CHECK(!tool(R"({"operation":"off"})")["isError"].asBool());
    r = tool(R"({"operation":"split","body":"Block 1","at":10,"gap":20})");
    CHECK(!r["isError"].asBool());
    CHECK(named(m, "Core 1") != nullptr);
    CHECK(!tool(R"({"operation":"parting","body":"Block 1","pull":"X"})")["isError"].asBool());
    CHECK(tool(R"({"operation":"cast"})")["isError"].asBool());
}

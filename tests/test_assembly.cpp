#include "TestFramework.hpp"

#include "core/Assembly.hpp"
#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/FileIO.hpp"
#include "core/Mcp.hpp"
#include "core/Scene.hpp"
#include "core/Step.hpp"

#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

using namespace sicdm;

#define REQUIRE(c)                  \
    do {                            \
        const bool required_ = (c); \
        CHECK(required_);           \
        if (!required_) return;     \
    } while (0)

namespace {

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }
std::string tempPath(const char* name) { return (std::filesystem::temp_directory_path() / name).string(); }
std::string stepPath(const std::string& name) { return std::string(SICDM_SOURCE_DIR) + "/tests/data/step/" + name + ".step"; }

const Body* body(const Document& d, const std::string& name) {
    for (const Body& b : d.scene.bodies())
        if (b.name == name) return &b;
    return nullptr;
}

bool near(float a, double b, double tol = 1e-3) { return std::fabs(static_cast<double>(a) - b) < tol; }

bool says(const CommandOutput& r, const std::string& text) {
    for (const std::string& l : r.lines)
        if (l.find(text) != std::string::npos) return true;
    return false;
}

// A 60 x 40 x 10 plate with an 8 mm hole through it at x = 15, and a pin lying on its side
// somewhere else.
void plateAndPin(Document& d) {
    run(d, "BOX 60 40 10 @0,0,0");                          // Block 1
    run(d, "HOLE \"Block 1\" AT 15,0,10 DIAMETER 8 THROUGH"); // a mesh body now: the hole is fitted
    run(d, "CYLINDER 4 30 @100,50,20");                      // Cylinder 2
    run(d, "ROTATE \"Cylinder 2\" 0,90,0");                  // lying along x
}

} // namespace

TEST_CASE("Assembly: a concentric mate turns and moves a pin into a hole, a flush mate seats it") {
    Document d;
    plateAndPin(d);
    // The pin's round face (it lies along x from its origin) and the hole's wall.
    auto r = run(d, "MATE CONCENTRIC \"Block 1\" AT 19,0,5 \"Cylinder 2\" AT 115,50,24");
    REQUIRE(r.ok);
    CHECK(says(r, "2 degrees of freedom left")); // it can still slide and spin
    const Body* pin = body(d, "Cylinder 2");
    REQUIRE(pin);
    AABB b = pin->bounds();
    CHECK(near((b.min.x + b.max.x) / 2, 15, 0.05)); // on the hole's axis
    CHECK(near((b.min.y + b.max.y) / 2, 0, 0.05));
    CHECK(near(b.max.z - b.min.z, 30, 0.05)); // standing up
    CHECK(near(b.max.x - b.min.x, 8, 0.05));
    // Its end face flush with the plate's underside.
    const float lowZ = b.min.z;
    r = run(d, std::string("MATE FLUSH \"Block 1\" AT 0,0,0 \"Cylinder 2\" AT 15,0,") + std::to_string(lowZ));
    REQUIRE(r.ok);
    CHECK(says(r, "1 degree of freedom left")); // the spin about its own axis
    b = body(d, "Cylinder 2")->bounds();
    CHECK(near(b.min.z, 0, 1e-3));
    CHECK(near(b.max.z, 30, 1e-3));
    for (const Mate& m : d.state.assembly.mates) CHECK(mateError(d.scene, m) < 1e-3);
    // The plate held still.
    CHECK(near(body(d, "Block 1")->offset.x, 0) && near(body(d, "Block 1")->offset.z, 0));
}

TEST_CASE("Assembly: three plane mates place a turned block exactly, with no freedom left") {
    Document d;
    run(d, "BOX 60 40 10 @0,0,0");   // Block 1
    run(d, "BOX 20 20 5 @-80,30,0"); // Block 2, turned 30 degrees and lifted
    run(d, "ROTATE \"Block 2\" 0,0,30");
    run(d, "MOVE 0,0,7 \"Block 2\"");
    // Its underside on the plate's top.
    auto r = run(d, "MATE COINCIDENT \"Block 1\" AT 0,0,10 \"Block 2\" AT -80,30,7");
    REQUIRE(r.ok);
    CHECK(says(r, "5 degrees of freedom left") || says(r, "3 degrees of freedom left"));
    // Its +x side flush with the plate's +x side, its +y side with the plate's +y side.
    const Body* b2 = body(d, "Block 2");
    REQUIRE(b2);
    const Vec3 o = b2->offset;
    // The side faces of the turned block: centre plus 10 mm along its turned x and y.
    const double c = std::cos(30 * 3.14159265358979 / 180), s = std::sin(30 * 3.14159265358979 / 180);
    char px[96], py[96];
    std::snprintf(px, sizeof px, "%.4f,%.4f,%.4f", o.x + 10 * c, o.y + 10 * s, o.z + 2.5);
    std::snprintf(py, sizeof py, "%.4f,%.4f,%.4f", o.x - 10 * s, o.y + 10 * c, o.z + 2.5);
    REQUIRE(run(d, std::string("MATE FLUSH \"Block 1\" AT 30,0,5 \"Block 2\" AT ") + px).ok);
    // Where its +y side is now (it moved and turned square).
    const AABB now = body(d, "Block 2")->bounds();
    std::snprintf(py, sizeof py, "%.4f,%.4f,%.4f", (now.min.x + now.max.x) / 2, now.max.y, (now.min.z + now.max.z) / 2);
    r = run(d, std::string("MATE FLUSH \"Block 1\" AT 0,20,5 \"Block 2\" AT ") + py);
    REQUIRE(r.ok);
    CHECK(says(r, "Fully placed"));
    b2 = body(d, "Block 2");
    const AABB bb = b2->bounds();
    CHECK(near(bb.min.x, 10, 1e-3) && near(bb.max.x, 30, 1e-3));
    CHECK(near(bb.min.y, 0, 1e-3) && near(bb.max.y, 20, 1e-3));
    CHECK(near(bb.min.z, 10, 1e-3) && near(bb.max.z, 15, 1e-3));
    CHECK(std::fabs(std::remainder(static_cast<double>(b2->rotation.z), 90.0)) < 1e-4); // square to the plate again
    // Listed as met.
    r = run(d, "MATE");
    REQUIRE(r.lines.size() == 3);
    for (const std::string& l : r.lines) CHECK(l.find(": met") != std::string::npos);
}

TEST_CASE("Assembly: a mate that conflicts is refused and nothing moves; distance and angle values edit") {
    Document d;
    run(d, "BOX 20 20 20 @0,0,0");  // Block 1
    run(d, "BOX 10 10 10 @50,0,0"); // Block 2
    REQUIRE(run(d, "MATE DISTANCE 5 \"Block 1\" AT 0,0,20 \"Block 2\" AT 50,0,0").ok);
    AABB b = body(d, "Block 2")->bounds();
    CHECK(near(b.min.z, 25, 1e-3)); // 5 mm above the top
    REQUIRE(run(d, "MATE EDIT M1 12").ok);
    b = body(d, "Block 2")->bounds();
    CHECK(near(b.min.z, 32, 1e-3));
    // The same faces flush as well: impossible together.
    const Vec3 before = body(d, "Block 2")->offset;
    char at[64];
    std::snprintf(at, sizeof at, "%.3f,%.3f,%.3f", b.min.x + 5, b.min.y + 5, b.min.z);
    auto r = run(d, std::string("MATE COINCIDENT \"Block 1\" AT 0,0,20 \"Block 2\" AT ") + at);
    CHECK(!r.ok);
    CHECK(says(r, "conflicts"));
    CHECK(d.state.assembly.mates.size() == 1);
    CHECK(near(body(d, "Block 2")->offset.z, before.z, 1e-6));
    // Wrong kinds of face.
    CHECK(!run(d, "MATE CONCENTRIC \"Block 1\" AT 0,0,20 \"Block 2\" AT 50,0,32").ok);
    CHECK(!run(d, "MATE COINCIDENT \"Block 1\" AT 0,0,20 \"Block 1\" AT 0,0,0").ok);
    CHECK(!run(d, "MATE COINCIDENT \"Block 1\" AT 0,0,20 \"Nothing\" AT 0,0,0").ok);
    // An angle between side faces.
    Document e;
    run(e, "BOX 20 20 20 @0,0,0");
    run(e, "BOX 10 10 10 @50,0,0");
    REQUIRE(run(e, "MATE ANGLE 30 \"Block 1\" AT 10,0,10 \"Block 2\" AT 55,0,5").ok);
    CHECK(mateError(e.scene, e.state.assembly.mates[0]) < 1e-3);
    REQUIRE(run(e, "MATE EDIT 1 60").ok);
    CHECK(mateError(e.scene, e.state.assembly.mates[0]) < 1e-3);
    CHECK(!run(e, "MATE EDIT 1 abc").ok);
}

TEST_CASE("Assembly: a fixed part stays and the other moves; suppressed mates are not solved") {
    Document d;
    run(d, "BOX 20 20 20 @0,0,0");  // Block 1
    run(d, "BOX 10 10 10 @50,0,0"); // Block 2
    REQUIRE(run(d, "MATE FIX \"Block 2\"").ok);
    CHECK(!run(d, "MATE FIX \"Block 2\"").ok); // already
    REQUIRE(run(d, "MATE COINCIDENT \"Block 1\" AT 0,0,20 \"Block 2\" AT 50,0,0").ok);
    // Block 2 held: Block 1 moved under it.
    CHECK(near(body(d, "Block 2")->offset.x, 50) && near(body(d, "Block 2")->offset.z, 0));
    CHECK(near(body(d, "Block 1")->bounds().max.z, 0, 1e-3));
    REQUIRE(run(d, "MATE SUPPRESS M2").ok);
    run(d, "MOVE 0,0,-30 \"Block 1\"");
    REQUIRE(run(d, "MATE SOLVE").ok);
    CHECK(near(body(d, "Block 1")->bounds().max.z, -30, 1e-3)); // not pulled back
    REQUIRE(run(d, "MATE UNSUPPRESS M2").ok);
    CHECK(near(body(d, "Block 1")->bounds().max.z, 0, 1e-3));
    REQUIRE(run(d, "MATE DELETE M2").ok);
    REQUIRE(run(d, "MATE CLEAR").ok);
    CHECK(d.state.assembly.mates.empty());
}

TEST_CASE("Assembly: interference finds overlapping parts and their overlap, not parts that only touch") {
    Document d;
    run(d, "BOX 10 10 10 @0,0,0"); // Block 1: -5..5
    run(d, "BOX 10 10 10 @5,0,0"); // Block 2: 0..10, overlapping 5 x 10 x 10
    run(d, "BOX 10 10 10 @0,0,10"); // Block 3: sits on Block 1, touching
    const auto hits = findInterference(d.scene);
    REQUIRE(hits.size() == 1); // Block 3 only touches the other two
    CHECK(hits[0].a == "Block 1");
    CHECK(hits[0].b == "Block 2");
    CHECK(std::fabs(hits[0].volume - 500) < 0.5);
    CHECK(near(hits[0].centre.x, 2.5, 0.01));
    CHECK(near(hits[0].size.x, 5, 0.01));
    const auto r = run(d, "ASSEMBLY INTERFERENCE");
    CHECK(says(r, "1 pair overlaps"));
    CHECK(says(r, "Block 1 and Block 2: 500 mm3"));
    // Only the parts named.
    CHECK(says(run(d, "ASSEMBLY INTERFERENCE \"Block 1\" \"Block 3\""), "No interference"));
    CHECK(!run(d, "ASSEMBLY INTERFERENCE \"Block 1\"").ok);
}

TEST_CASE("Assembly: sub-assemblies, the tree and the parts list") {
    Document d;
    run(d, "BOX 60 40 10 @0,0,0");   // Block 1
    run(d, "CYLINDER 4 30 @0,0,0");  // Cylinder 2
    run(d, "CYLINDER 4 30 @20,0,0"); // Cylinder 3
    REQUIRE(run(d, "ASSEMBLY GROUP \"Base\" \"Block 1\"").ok);
    REQUIRE(run(d, "ASSEMBLY GROUP \"Base/Pins\" \"Cylinder 2\" \"Cylinder 3\"").ok);
    CHECK(body(d, "Cylinder 3")->assembly == "Base/Pins");
    const auto tree = run(d, "ASSEMBLY").lines;
    REQUIRE(tree.size() == 5);
    CHECK(tree[0] == "[Base]");
    CHECK(tree[1] == "  Block 1");
    CHECK(tree[2] == "  [Pins]");
    CHECK(tree[3] == "    Cylinder 2");
    const auto bom = billOfMaterials(d.scene);
    REQUIRE(bom.size() == 2);
    CHECK(bom[0].first == "Block" && bom[0].second == 1);
    CHECK(bom[1].first == "Cylinder" && bom[1].second == 2);
    REQUIRE(run(d, "ASSEMBLY UNGROUP \"Block 1\"").ok);
    CHECK(body(d, "Block 1")->assembly.empty());
    CHECK(!run(d, "ASSEMBLY GROUP \"\" \"Block 1\"").ok);
    CHECK(!run(d, "ASSEMBLY GROUP \"X\" \"Nothing\"").ok);
}

TEST_CASE("Assembly: STEP files keep the tree both ways") {
    // Read: an assembly written by another system.
    {
        Document d;
        REQUIRE(run(d, "IMPORT \"" + stepPath("assembly") + "\"").ok);
        int sub = 0, top = 0;
        for (const Body& b : d.scene.bodies()) {
            if (b.assembly == "Top/SubAssembly") ++sub;
            if (b.assembly == "Top") ++top;
        }
        CHECK(sub == 2); // the two brackets
        CHECK(top == 1); // the pin
    }
    // Written and read back.
    Document d;
    run(d, "BOX 60 40 10 @0,0,0");
    run(d, "CYLINDER 4 30 @0,0,0");
    run(d, "BOX 5 5 5 @50,50,0");
    run(d, "ASSEMBLY GROUP \"Base\" \"Block 1\"");
    run(d, "ASSEMBLY GROUP \"Base/Pins\" \"Cylinder 2\"");
    const std::string path = tempPath("sicdm_assembly.step");
    REQUIRE(run(d, "EXPORT \"" + path + "\"").ok);
    StepReadReport rep;
    std::string err;
    const auto parts = readStepFile(path, &rep, &err);
    REQUIRE(parts.size() == 3);
    for (const StepPart& p : parts) {
        if (p.name == "Block 1") CHECK(p.assembly == "Base");
        else if (p.name == "Cylinder 2") CHECK(p.assembly == "Base/Pins");
        else CHECK(p.assembly.empty());
    }
    std::filesystem::remove(path);
}

TEST_CASE("Assembly: mates and the tree are saved, and the history replays the mates") {
    Document d;
    run(d, "BOX 20 20 20 @0,0,0");
    run(d, "BOX 10 10 10 @50,0,0");
    run(d, "MATE DISTANCE 5 \"Block 1\" AT 0,0,20 \"Block 2\" AT 50,0,0");
    run(d, "ASSEMBLY GROUP \"Stack\" \"Block 2\"");
    const std::string path = tempPath("sicdm_assembly.sicdm");
    std::string err;
    REQUIRE(saveProject(d.scene, d.state, path, &err));
    Scene scene;
    ProjectState st;
    REQUIRE(loadProject(scene, st, path, &err));
    REQUIRE(st.assembly.mates.size() == 1);
    CHECK(st.assembly.mates[0].type == MateType::Distance);
    CHECK(st.assembly.mates[0].value == 5);
    CHECK(mateError(scene, st.assembly.mates[0]) < 1e-3);
    bool grouped = false;
    for (const Body& b : scene.bodies()) grouped = grouped || (b.name == "Block 2" && b.assembly == "Stack");
    CHECK(grouped);
    std::filesystem::remove(path);
    // REGEN rebuilds the parts and puts the mate back once.
    REQUIRE(run(d, "REGEN").ok);
    CHECK(d.state.assembly.mates.size() == 1);
    CHECK(near(body(d, "Block 2")->bounds().min.z, 25, 1e-3));
    // NEW starts without them.
    run(d, "NEW");
    CHECK(d.state.assembly.mates.empty());
}

TEST_CASE("MCP assembly mates parts, lists the tree and checks interference") {
    Document d;
    McpServer s(d, nullptr);
    auto tool = [&](const std::string& name, const std::string& args) {
        const auto r = Json::parse(s.handle(R"({"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":")" + name + R"(","arguments":)" + args + "}}"));
        return r ? (*r)["result"] : Json();
    };
    run(d, "BOX 20 20 20 @0,0,0");
    run(d, "BOX 10 10 10 @50,0,0");
    Json r = tool("assembly", R"({"operation":"mate","type":"coincident","part":"Block 1","at":[0,0,20],"moving":"Block 2","moving_at":[50,0,0]})");
    CHECK(!r["isError"].asBool());
    CHECK(near(body(d, "Block 2")->bounds().min.z, 20, 1e-3));
    r = tool("assembly", R"({"operation":"list"})");
    CHECK(r["content"][0]["text"].asString().find("M1: coincident") != std::string::npos);
    CHECK(!tool("assembly", R"({"operation":"group","name":"Top","parts":["Block 1","Block 2"]})")["isError"].asBool());
    r = tool("assembly", R"({"operation":"tree"})");
    CHECK(r["content"][0]["text"].asString().find("[Top]") != std::string::npos);
    r = tool("assembly", R"({"operation":"interference"})");
    CHECK(r["content"][0]["text"].asString().find("No interference") != std::string::npos);
    CHECK(tool("assembly", R"({"operation":"mate","type":"coincident"})")["isError"].asBool());
    CHECK(tool("assembly", R"({"operation":"wobble"})")["isError"].asBool());
}

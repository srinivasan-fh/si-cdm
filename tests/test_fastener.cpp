#include "TestFramework.hpp"

#include "core/Assembly.hpp"
#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/ExactMesh.hpp"
#include "core/Fastener.hpp"
#include "core/HoleWizard.hpp"
#include "core/Json.hpp"
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

std::string prop(const Body* b, const char* key) {
    if (!b) return {};
    const auto it = b->properties.find(key);
    return it == b->properties.end() ? std::string() : it->second;
}

int sizeOf(FastenerType t, const char* name) { return findFastenerSize(t, name); }

std::vector<float> params(FastenerType t, const char* size, float length = 0) {
    return {static_cast<float>(t), static_cast<float>(sizeOf(t, size)), length};
}

constexpr double kPolygon = 0.99714; // area of a 48-gon over its circle's

} // namespace

TEST_CASE("fasteners: every size of every type is a closed, exact solid") {
    for (int t = 0; t < static_cast<int>(FastenerType::Count); ++t) {
        const auto type = static_cast<FastenerType>(t);
        const auto& sizes = fastenerSizes(type);
        CHECK(!sizes.empty());
        for (std::size_t i = 0; i < sizes.size(); ++i) {
            CHECK(sizes[i].d > 0);
            const float lo = minFastenerLength(type, static_cast<int>(i));
            CHECK(!fastenerHasLength(type) || lo >= sizes[i].d - 1e-4f);
            for (float len : {lo, pickFastenerLength(type, static_cast<int>(i), 3.0f * sizes[i].d)}) {
                const std::vector<float> p{static_cast<float>(t), static_cast<float>(i), len};
                std::string why;
                const MeshData m = buildFastener(p, &why);
                CHECK(!m.indices.empty());
                if (m.indices.empty()) {
                    std::printf("    %s %s x %g: %s\n", fastenerTypeName(type), sizes[i].name.c_str(), static_cast<double>(len), why.c_str());
                    continue;
                }
                CHECK(openEdgeCount(m) == 0);
                CHECK(meshVolume(m) > 0);
                ExactReport rep;
                CHECK(exactBrepFromMesh(m, fastenerSurfaces(p), &rep) != nullptr);
                CHECK(!fastenerDesignation(p).empty());
            }
        }
    }
    // Too short, an unknown size or type: no part.
    std::string why;
    CHECK(buildFastener(params(FastenerType::SocketCap, "M6", 0.2f), &why).indices.empty());
    CHECK(why.find("short") != std::string::npos);
    CHECK(buildFastener({1, 99, 20}).indices.empty());
    CHECK(buildFastener({42, 0, 20}).indices.empty());
}

TEST_CASE("fasteners: the dimensions that matter follow the standards") {
    // ISO 4762 M6x20: head 10 across, 6 high on the bearing face (z = 0), the shank 20 down.
    MeshData m = buildFastener(params(FastenerType::SocketCap, "M6", 20));
    AABB b = m.bounds();
    CHECK(near(b.max.z, 6, 1e-5) && near(b.min.z, -20, 1e-5));
    CHECK(near(b.max.x - b.min.x, 10, 1e-4));
    // ISO 4017 M8: 13 across flats, so 13 / cos 30 across the corners; head 5.3.
    m = buildFastener(params(FastenerType::HexBolt, "M8", 30));
    b = m.bounds();
    CHECK(near(b.max.x - b.min.x, 13 / std::cos(kPi / 6), 1e-3));
    CHECK(near(b.max.y - b.min.y, 13, 1e-3));
    CHECK(near(b.max.z, 5.3, 1e-5) && near(b.min.z, -30, 1e-5));
    // ISO 4032 M10 nut 8.4 high, ISO 7089 M6 washer 6.4 x 12 x 1.6 (its volume, as a 48-gon).
    m = buildFastener(params(FastenerType::HexNut, "M10"));
    CHECK(near(m.bounds().max.z - m.bounds().min.z, 8.4, 1e-5));
    m = buildFastener(params(FastenerType::Washer, "M6"));
    CHECK(near(meshVolume(m), kPolygon * kPi / 4 * (12.0 * 12 - 6.4 * 6.4) * 1.6, 0.01));
    // A countersunk head is the hole wizard's countersink, flush at z = 0, its length overall.
    const HoleSize* m5 = findHoleSize("M5");
    REQUIRE(m5 != nullptr);
    m = buildFastener(params(FastenerType::Countersunk, "M5", 16));
    b = m.bounds();
    CHECK(near(b.max.x - b.min.x, m5->csinkDia, 1e-4));
    CHECK(near(b.max.z, 0, 1e-6) && near(b.min.z, -16, 1e-5));
    const FastenerSize& csk = fastenerSizes(FastenerType::Countersunk)[static_cast<std::size_t>(sizeOf(FastenerType::Countersunk, "M5"))];
    CHECK(near(csk.height, (m5->csinkDia - 5) / 2, 1e-5)); // 90 degrees
    const FastenerSize& cskInch = fastenerSizes(FastenerType::Countersunk)[static_cast<std::size_t>(sizeOf(FastenerType::Countersunk, "1/4-20"))];
    CHECK(near(cskInch.height, (cskInch.head - cskInch.d) / 2 / std::tan(41.0 * kPi / 180), 1e-4)); // 82 degrees
    // A dowel pin is centred on its origin.
    m = buildFastener(params(FastenerType::DowelPin, "6", 20));
    CHECK(near(m.bounds().min.z, -10, 1e-5) && near(m.bounds().max.z, 10, 1e-5));
    // Threads match the hole wizard's sizes.
    for (const FastenerSize& s : fastenerSizes(FastenerType::SocketCap)) {
        const HoleSize* h = findHoleSize(s.name);
        REQUIRE(h != nullptr);
        CHECK(near(s.d, h->nominal, 1e-6) && near(s.pitch, h->pitch, 1e-6));
    }
}

TEST_CASE("fasteners: sizes by name, designations, standard lengths") {
    // M1.6 and M16 are different sizes, however they're spelled.
    CHECK(sizeOf(FastenerType::SocketCap, "M1.6") == 0);
    CHECK(fastenerSizes(FastenerType::SocketCap)[static_cast<std::size_t>(sizeOf(FastenerType::SocketCap, "m16"))].d == 16.0f);
    CHECK(sizeOf(FastenerType::SocketCap, "10-24") == sizeOf(FastenerType::SocketCap, "#10-24"));
    CHECK(sizeOf(FastenerType::SocketCap, "M6x1") == sizeOf(FastenerType::SocketCap, "M6"));
    CHECK(sizeOf(FastenerType::Washer, "1/4-20") == sizeOf(FastenerType::Washer, "1/4"));
    CHECK(sizeOf(FastenerType::ButtonHead, "M20") < 0); // ISO 7380 stops at M16
    CHECK(sizeOf(FastenerType::Countersunk, "M2") < 0);
    // Pins: a bare number is mm; an inch size has a fraction or a unit.
    const auto& pins = fastenerSizes(FastenerType::DowelPin);
    CHECK(!pins[static_cast<std::size_t>(sizeOf(FastenerType::DowelPin, "1"))].inch);
    CHECK(pins[static_cast<std::size_t>(sizeOf(FastenerType::DowelPin, "1in"))].inch);
    CHECK(near(pins[static_cast<std::size_t>(sizeOf(FastenerType::DowelPin, "1/4"))].d, 6.35, 1e-5));
    CHECK(sizeOf(FastenerType::DowelPin, "7") < 0);

    CHECK(fastenerDesignation(params(FastenerType::SocketCap, "M6", 20)) == "ISO 4762 M6x20");
    CHECK(fastenerDescription(params(FastenerType::SocketCap, "M6", 20)) == "Socket head cap screw M6x20");
    CHECK(fastenerDesignation(params(FastenerType::SocketCap, "1/4-20", 19.05f)) == "ASME B18.3 1/4-20 x 3/4");
    CHECK(fastenerDesignation(params(FastenerType::HexBolt, "1/2-13", 38.1f)) == "ASME B18.2.1 1/2-13 x 1-1/2");
    CHECK(fastenerDesignation(params(FastenerType::HexNut, "M8")) == "ISO 4032 M8");
    CHECK(fastenerDesignation(params(FastenerType::HexNut, "#10-24")) == "ASME B18.6.3 #10-24");
    CHECK(fastenerDesignation(params(FastenerType::Washer, "M6")) == "ISO 7089 M6");
    CHECK(fastenerDesignation(params(FastenerType::DowelPin, "6", 20)) == "ISO 8734 6x20");
    CHECK(fastenerLengthText(20, true) == "0.787");

    const int m6 = sizeOf(FastenerType::SocketCap, "M6");
    CHECK(pickFastenerLength(FastenerType::SocketCap, m6, 22.6f) == 25.0f);
    CHECK(pickFastenerLength(FastenerType::SocketCap, m6, 25.0f) == 25.0f);
    CHECK(pickFastenerLength(FastenerType::SocketCap, m6, 13.5f, true) == 12.0f);
    CHECK(pickFastenerLength(FastenerType::SocketCap, m6, 1.0f, true) == 6.0f); // never below the shortest
    CHECK(pickFastenerLength(FastenerType::SocketCap, m6, 410.0f) == 420.0f);  // past the table: in its last step
    const int quarter = sizeOf(FastenerType::SocketCap, "1/4-20");
    CHECK(near(pickFastenerLength(FastenerType::SocketCap, quarter, 20.0f), 22.225, 1e-4)); // 7/8

    const auto len = parseFastenerLength("3/4", true);
    REQUIRE(len.has_value());
    CHECK(near(*len, 19.05, 1e-4));
    CHECK(near(*parseFastenerLength("20mm", true), 20, 1e-6));
    CHECK(near(*parseFastenerLength("20", false), 20, 1e-6));
    CHECK(!parseFastenerLength("-3", false));

    // The rotation turns the shank (-Z) onto the direction.
    for (const Vec3 dir : {Vec3{1, 0, 0}, Vec3{0, 1, 0}, Vec3{0, 0, 1}, Vec3{0, 0, -1}, normalize(Vec3{1, -2, 3})}) {
        const Vec3 got = eulerRotation(fastenerRotation(dir)).transformPoint({0, 0, -1});
        CHECK(length(got - dir) < 1e-5f);
    }
}

TEST_CASE("fasteners: FASTENER places one, PART and PARAM edit it") {
    Document d;
    CHECK(text(run(d, "FASTENER")).find("ISO 4762") != std::string::npos);
    CHECK(text(run(d, "FASTENER SIZES NUT")).find("across flats 16, height 8.4") != std::string::npos);
    CommandOutput o = run(d, "FASTENER SHCS M6x20 AT 0,0,10");
    REQUIRE(o.ok);
    const Body* s = named(d, "Socket Head Cap Screw 1");
    REQUIRE(s != nullptr);
    CHECK(s->kind == "fastener" && s->exact());
    CHECK(prop(s, "NUMBER") == "ISO 4762 M6x20" && prop(s, "MATERIAL") == "Steel 12.9" && prop(s, "THREAD") == "M6x1");
    CHECK(near(s->bounds().max.z, 16, 1e-4) && near(s->bounds().min.z, -10, 1e-4));
    // Lying along +X: the head behind the point, the shank ahead.
    REQUIRE(run(d, "FASTENER BOLT 1/4-20x3/4 AT 100,0,0 DIR X NAME Pivot").ok);
    const Body* p = named(d, "Pivot");
    REQUIRE(p != nullptr);
    CHECK(near(p->bounds().max.x, 100 + 19.05, 1e-3));
    CHECK(near(p->bounds().min.x, 100 - 0.156 * 25.4, 1e-3));
    CHECK(prop(p, "NUMBER") == "ASME B18.2.1 1/4-20 x 3/4");
    // No length given: picked from the size.
    REQUIRE(run(d, "FASTENER PIN 8").ok);
    CHECK(prop(named(d, "Dowel Pin 1"), "NUMBER") == "ISO 8734 8x24");
    // Refused.
    CHECK(!run(d, "FASTENER SHCS M7").ok);
    CHECK(!run(d, "FASTENER SHCS M6x2").ok);
    CHECK(!run(d, "FASTENER CSK M2").ok);
    CHECK(!run(d, "FASTENER WIDGET M6").ok);
    CHECK(!run(d, "FASTENER SHCS M6 NAME Pivot").ok);
    CHECK(!run(d, "FASTENER SHCS M6 DIR 0,0,0").ok);
    // The library: PART takes the size by name (M1.6 is not M16), PARAM changes it and the
    // part number follows.
    REQUIRE(run(d, "PART socket_head_cap_screw SIZE M16 LENGTH 40").ok);
    const Body* q = named(d, "Socket Head Cap Screw 2");
    REQUIRE(q != nullptr);
    CHECK(prop(q, "NUMBER") == "ISO 4762 M16x40");
    REQUIRE(run(d, "PARAM \"Socket Head Cap Screw 2\" SIZE M1.6 LENGTH 5").ok);
    q = named(d, "Socket Head Cap Screw 2");
    CHECK(prop(q, "NUMBER") == "ISO 4762 M1.6x5");
    CHECK(near(q->bounds().max.x - q->bounds().min.x, 3, 1e-4));
    CHECK(!run(d, "PARAM \"Socket Head Cap Screw 2\" LENGTH 0.5").ok); // too short: kept as it was
    CHECK(prop(named(d, "Socket Head Cap Screw 2"), "NUMBER") == "ISO 4762 M1.6x5");
    REQUIRE(run(d, "UNDO").ok);
    CHECK(prop(named(d, "Socket Head Cap Screw 2"), "NUMBER") == "ISO 4762 M16x40");
}

TEST_CASE("fasteners: INTO fills the hole wizard's holes") {
    Document d;
    REQUIRE(run(d, "BOX 120 60 20").ok); // -60..60, -30..30, 0..20
    REQUIRE(run(d, "HOLE \"Block 1\" AT -45,-15,20 SIZE M6 CBORE THROUGH").ok);
    REQUIRE(run(d, "HOLE \"Block 1\" AT -20,-15,20 SIZE M5 CSINK THROUGH").ok);
    REQUIRE(run(d, "HOLE \"Block 1\" AT 5,-15,20 SIZE M8 TAPPED DEPTH 16").ok);
    REQUIRE(run(d, "HOLE \"Block 1\" AT 30,-15,20 SIZE M10 THROUGH").ok);
    REQUIRE(run(d, "HOLE \"Block 1\" AT -45,15,0 SIZE 1/4-20 TAPPED THROUGH").ok);
    REQUIRE(run(d, "HOLE \"Block 1\" AT 0,15,20 SIZE M4 TAPPED THROUGH").ok);
    REQUIRE(run(d, "HOLE \"Block 1\" AT 40,15,20 DIAMETER 5 DEPTH 8").ok);
    const std::size_t before = d.scene.bodies().size();

    CommandOutput o = run(d, "FASTENER INTO \"Block 1\" HOLE 1,2,3");
    REQUIRE(o.ok);
    CHECK(text(o).find("3 holes") != std::string::npos);
    // Counterbored: a cap screw sunk in it, through the plate and 1.5 d on (20 - 6.4 + 9 -> 25).
    const Body* s1 = named(d, "Socket Head Cap Screw 1");
    REQUIRE(s1 != nullptr);
    CHECK(prop(s1, "NUMBER") == "ISO 4762 M6x25");
    CHECK(near(s1->bounds().max.z, 20 - 6.4 + 6, 1e-3) && near(s1->bounds().min.z, 20 - 6.4 - 25, 1e-3));
    CHECK(near(s1->bounds().center().x, -45, 1e-3) && near(s1->bounds().center().y, -15, 1e-3));
    // Countersunk: flush, 20 + 7.5 -> 30 overall.
    const Body* c1 = named(d, "Countersunk Screw 1");
    REQUIRE(c1 != nullptr);
    CHECK(prop(c1, "NUMBER") == "ISO 10642 M5x30");
    CHECK(near(c1->bounds().max.z, 20, 1e-3));
    // Tapped blind (13.5 of thread): 1.5 d of engagement, 12.
    const Body* s2 = named(d, "Socket Head Cap Screw 2");
    REQUIRE(s2 != nullptr);
    CHECK(prop(s2, "NUMBER") == "ISO 4762 M8x12");
    CHECK(near(s2->bounds().min.z, 8, 1e-3));

    // A plain hole: a hex bolt, washers both sides and a nut, two threads past it.
    o = run(d, "FASTENER INTO \"Block 1\" HOLE 4 WASHER NUT");
    REQUIRE(o.ok);
    const Body* bolt = named(d, "Hex Bolt 1");
    const Body* w1 = named(d, "Plain Washer 1");
    const Body* w2 = named(d, "Plain Washer 2");
    const Body* nut = named(d, "Hex Nut 1");
    REQUIRE(bolt && w1 && w2 && nut);
    CHECK(prop(bolt, "NUMBER") == "ISO 4017 M10x40");
    CHECK(near(w1->bounds().min.z, 20, 1e-3) && near(w1->bounds().max.z, 22, 1e-3));
    CHECK(near(bolt->bounds().max.z, 22 + 6.4, 1e-3));
    CHECK(near(w2->bounds().max.z, 0, 1e-3) && near(w2->bounds().min.z, -2, 1e-3));
    CHECK(near(nut->bounds().max.z, -2, 1e-3) && near(nut->bounds().min.z, -10.4, 1e-3));
    CHECK(bolt->bounds().min.z < nut->bounds().min.z - 2 * 1.5 + 1e-3);

    // Tapped from below (pointing up): the head under the plate, 1.5 d of thread.
    REQUIRE(run(d, "FASTENER INTO \"Block 1\" HOLE 5").ok);
    const Body* s3 = named(d, "Socket Head Cap Screw 3");
    REQUIRE(s3 != nullptr);
    CHECK(prop(s3, "NUMBER") == "ASME B18.3 1/4-20 x 3/8");
    CHECK(near(s3->bounds().max.z, 9.525, 1e-3) && near(s3->bounds().min.z, -6.35, 1e-3));
    // GRIP: room for a 5 mm part clamped under the washer and head.
    REQUIRE(run(d, "FASTENER INTO \"Block 1\" HOLE 6 GRIP 5 WASHER").ok);
    const Body* w3 = named(d, "Plain Washer 3");
    const Body* s4 = named(d, "Socket Head Cap Screw 4");
    REQUIRE(w3 && s4);
    CHECK(near(w3->bounds().min.z, 25, 1e-3) && near(s4->bounds().max.z, 25.8 + 4, 1e-3));
    CHECK(prop(s4, "NUMBER") == "ISO 4762 M4x12");
    // No washer under a head in a counterbore.
    o = run(d, "FASTENER INTO \"Block 1\" HOLE 1 WASHER");
    REQUIRE(o.ok);
    CHECK(text(o).find("no washer") != std::string::npos);
    CHECK(named(d, "Plain Washer 4") == nullptr);

    // Refused, and nothing added: a hole without a size, a size the type lacks, NUT in a
    // blind hole, a hole that isn't there, a nut as the fastener.
    const std::size_t n = d.scene.bodies().size();
    CHECK(!run(d, "FASTENER INTO \"Block 1\" HOLE 7").ok);
    CHECK(run(d, "FASTENER INTO \"Block 1\" HOLE 7 PIN SIZE 5").ok); // a pin goes in, half its length
    CHECK(prop(named(d, "Dowel Pin 1"), "NUMBER") == "ISO 8734 5x16");
    CHECK(!run(d, "FASTENER INTO \"Block 1\" HOLE 1 BHCS SIZE M20").ok);
    CHECK(!run(d, "FASTENER INTO \"Block 1\" HOLE 3 NUT").ok);
    CHECK(!run(d, "FASTENER INTO \"Block 1\" HOLE 9").ok);
    CHECK(!run(d, "FASTENER INTO \"Block 1\" NUT HOLE 4 WASHER HEXNUT").ok);
    CHECK(!run(d, "FASTENER INTO \"Socket Head Cap Screw 1\"").ok);
    CHECK(d.scene.bodies().size() == n + 1);

    // The parts list counts them by designation.
    REQUIRE(run(d, "FASTENER INTO \"Block 1\" HOLE 1,1").ok);
    int cap25 = 0;
    for (const BomLine& l : partsList(d.scene)) {
        if (l.number != "ISO 4762 M6x25") continue;
        cap25 = l.quantity;
        CHECK(l.description == "Socket head cap screw M6x25");
    }
    CHECK(cap25 == 4);

    // Saved and opened again, as they were; undone; rebuilt from the history.
    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_fastener_test.sicdm").string();
    REQUIRE(run(d, "SAVE \"" + path + "\"").ok);
    Document e;
    REQUIRE(run(e, "OPEN \"" + path + "\"").ok);
    std::filesystem::remove(path);
    REQUIRE(e.scene.bodies().size() == d.scene.bodies().size());
    for (std::size_t i = 0; i < e.scene.bodies().size(); ++i) {
        const Body &a = d.scene.bodies()[i], &b = e.scene.bodies()[i];
        CHECK(a.name == b.name && a.properties == b.properties);
        CHECK(length(a.bounds().min - b.bounds().min) < 1e-3f && length(a.bounds().max - b.bounds().max) < 1e-3f);
    }
    const std::size_t all = d.scene.bodies().size();
    REQUIRE(run(d, "UNDO").ok);
    CHECK(d.scene.bodies().size() == all - 2);
    REQUIRE(run(d, "REGEN").ok);
    CHECK(d.scene.bodies().size() == all - 2);
    CHECK(prop(named(d, "Hex Bolt 1"), "NUMBER") == "ISO 4017 M10x40");
    CHECK(d.scene.bodies().size() > before);
}

TEST_CASE("fasteners: over MCP") {
    Document m;
    McpServer s(m, nullptr);
    const auto tool = [&](const std::string& args) {
        const std::string msg = R"({"jsonrpc":"2.0","id":7,"method":"tools/call","params":{"name":"fastener","arguments":)" + args + "}}";
        const auto r = Json::parse(s.handle(msg));
        return r ? (*r)["result"] : Json();
    };
    CHECK(tool(R"({"operation":"list"})")["content"][0]["text"].asString().find("ISO 7089") != std::string::npos);
    CHECK(tool(R"({"operation":"sizes","type":"washer"})")["content"][0]["text"].asString().find("6.4 x 12 x 1.6") != std::string::npos);
    Json r = tool(R"({"operation":"place","type":"SHCS","size":"M8","length":"30","at":[0,0,0],"dir":[1,0,0],"name":"Axle"})");
    CHECK(!r["isError"].asBool());
    CHECK(prop(named(m, "Axle"), "NUMBER") == "ISO 4762 M8x30");
    REQUIRE(run(m, "BOX 60 40 10").ok);
    REQUIRE(run(m, "HOLE \"Block 1\" AT -15,0,10 AT 15,0,10 SIZE M6 THROUGH").ok);
    r = tool(R"({"operation":"into","body":"Block 1","holes":[1,2],"washer":true,"nut":true})");
    CHECK(!r["isError"].asBool());
    CHECK(named(m, "Hex Nut 2") != nullptr);
    CHECK(tool(R"({"operation":"place","type":"SHCS"})")["isError"].asBool());
    CHECK(tool(R"({"operation":"weld"})")["isError"].asBool());
}

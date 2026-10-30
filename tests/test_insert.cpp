#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/FileIO.hpp"
#include "core/InsertBrush.hpp"
#include "core/Json.hpp"
#include "core/Mcp.hpp"
#include "core/Scene.hpp"
#include "core/Sculpt.hpp"

#include <cmath>
#include <filesystem>

using namespace sicdm;

#define REQUIRE(c)                  \
    do {                            \
        const bool required_ = (c); \
        CHECK(required_);           \
        if (!required_) return;     \
    } while (0)

namespace {

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }

Body* named(Document& d, const std::string& n) {
    for (Body& b : d.scene.bodies())
        if (b.name == n) return &b;
    return nullptr;
}

AABB meshBox(const MeshData& m) { return m.bounds(); }

std::string joined(const CommandOutput& o) {
    std::string s;
    for (const std::string& l : o.lines) s += l + "\n";
    return s;
}

} // namespace

TEST_CASE("insert brush: each shape stands on the surface along its normal") {
    const InsertShape shapes[] = {InsertShape::Sphere, InsertShape::Cube, InsertShape::Cylinder, InsertShape::Cone, InsertShape::Torus};
    for (const InsertShape s : shapes) {
        // On a wall facing +X at x = 10: the shape reaches out along +X, `sink` of it inside
        // (a torus lies flat: 0.3 of its size tall).
        const InsertPlace place{{10, 0, 5}, {1, 0, 0}, {0, 1, 0}};
        const MeshData m = insertShapeMesh(s, 4, place, 0, 0.25f);
        const float tall = s == InsertShape::Torus ? 1.2f : 4.0f;
        REQUIRE(!m.indices.empty());
        CHECK(watertight(m));
        CHECK(meshVolume(m) > 0);
        const AABB b = meshBox(m);
        CHECK(std::fabs(b.min.x - (10 - 0.25f * tall)) < 0.05f); // a quarter sunk in
        CHECK(std::fabs(b.max.x - (10 + 0.75f * tall)) < 0.05f); // the rest standing out
        CHECK(std::fabs((b.min.y + b.max.y) * 0.5f) < 0.05f);
        CHECK(std::fabs((b.min.z + b.max.z) * 0.5f - 5) < 0.05f);
    }
    // A torus lies flat on the surface: as wide as SIZE, a ring's thickness tall.
    const MeshData t = insertShapeMesh(InsertShape::Torus, 6, {{0, 0, 0}, {0, 0, 1}, {1, 0, 0}}, 0, 0);
    const AABB tb = meshBox(t);
    CHECK(std::fabs(tb.max.x - tb.min.x - 6) < 0.1f);
    CHECK(tb.max.z - tb.min.z < 3);
    CHECK(insertShapeFromName("ball") == InsertShape::Sphere);
    CHECK(insertShapeFromName("spike") == InsertShape::Cone);
    CHECK(!insertShapeFromName("teapot"));
}

TEST_CASE("insert brush: an even stroke and a closed tube along it") {
    std::vector<Vec3> pts, ns;
    for (int i = 0; i <= 10; ++i) pts.push_back({static_cast<float>(i * i) * 0.2f, 0, 0}), ns.push_back({0, 0, 1});
    const auto stroke = evenStroke(pts, ns, 1.0f);
    REQUIRE(stroke.size() > 10);
    for (std::size_t i = 1; i + 1 < stroke.size(); ++i) CHECK(std::fabs(length(stroke[i].at - stroke[i - 1].at) - 1.0f) < 0.15f);
    const MeshData tube = curveTube(stroke, 1.0f, 0.5f);
    REQUIRE(!tube.indices.empty());
    CHECK(watertight(tube));
    // About a cylinder of 20 mm and two half balls.
    const double expect = 3.14159 * 20 + 4.18879;
    CHECK(std::fabs(meshVolume(tube) - expect) < expect * 0.06);
    // A row of copies every 4 mm along the 20 mm stroke.
    const MeshData row = curveCopies(stroke, InsertShape::Sphere, 2, 4);
    CHECK(watertight(row));
    const double one = meshVolume(insertShapeMesh(InsertShape::Sphere, 2, {}, 0));
    const double copies = meshVolume(row) / one;
    CHECK(copies > 4.5);
    CHECK(copies < 6.5);
}

TEST_CASE("INSERTMESH and CURVE: united, appended, new bodies, undo") {
    Document d;
    REQUIRE(run(d, "BOX 20 20 10").ok);
    Body* box = &d.scene.bodies().back();
    const std::string name = box->name;
    const double v0 = meshVolume(box->mesh);
    // A cone on the top face: one closed solid, larger.
    const auto cone = run(d, "INSERTMESH \"" + name + "\" CONE AT 0,0,10 SIZE 6");
    REQUIRE(cone.ok);
    box = named(d, name);
    REQUIRE(box);
    CHECK(openEdgeCount(box->mesh) == 0);
    const double v1 = meshVolume(box->mesh);
    CHECK(v1 > v0 + 20);
    CHECK(meshBox(box->mesh).max.z > 14);
    // A tube across the top, united.
    REQUIRE(run(d, "CURVE \"" + name + "\" TUBE PATH -8,-8,10 0,-6,10 8,-8,10 RADIUS 1").ok);
    box = named(d, name);
    CHECK(openEdgeCount(box->mesh) == 0);
    CHECK(meshVolume(box->mesh) > v1 + 10);
    // Appended: fused into one closed solid at once, what shows of it a polygroup of its own.
    const std::size_t tris = box->mesh.indices.size() / 3;
    const double v2 = meshVolume(box->mesh);
    const auto app = run(d, "INSERTMESH \"" + name + "\" TORUS AT 5,5,10 SIZE 4 APPEND");
    REQUIRE(app.ok);
    CHECK(joined(app).find("fused into one closed solid") != std::string::npos);
    box = named(d, name);
    CHECK(box->kind == "mesh");
    CHECK(box->mesh.hasGroups());
    CHECK(openEdgeCount(box->mesh) == 0);
    CHECK(box->mesh.indices.size() / 3 > tris);
    CHECK(meshVolume(box->mesh) > v2 + 5);
    {
        // The new group: on the torus (its triangles about it), the rest the box's.
        std::uint32_t top = 0;
        for (const std::uint32_t g : box->mesh.groups) top = std::max(top, g);
        std::size_t onTorus = 0, elsewhere = 0;
        for (std::size_t t = 0; t < box->mesh.groups.size(); ++t) {
            if (box->mesh.groups[t] != top) continue;
            Vec3 c{0, 0, 0};
            for (int k = 0; k < 3; ++k) c += box->mesh.vertices[box->mesh.indices[3 * t + static_cast<std::size_t>(k)]].position * (1.0f / 3);
            const Vec3 at = c + box->offset;
            (length(Vec3{at.x - 5, at.y - 5, 0}) < 3.5f && at.z > 9.9f ? onTorus : elsewhere) += 1;
        }
        CHECK(onTorus > 20);
        CHECK(elsewhere == 0);
    }
    // A row of spheres as a body of its own.
    const std::size_t bodies = d.scene.bodies().size();
    const auto row = run(d, "CURVE \"" + name + "\" INSERT SPHERE PATH -8,8,10 8,8,10 SIZE 2 SPACING 4 NEW");
    REQUIRE(row.ok);
    CHECK(d.scene.bodies().size() == bodies + 1);
    CHECK(d.scene.bodies().back().name == "Insert 1");
    // Another body's shape put on the box.
    REQUIRE(run(d, "SPHERE 3 @40,0,0").ok);
    const std::string ball = d.scene.bodies().back().name;
    REQUIRE(run(d, "INSERTMESH \"" + name + "\" BODY \"" + ball + "\" AT 10,0,5 SIZE 4").ok);
    CHECK(meshBox(named(d, name)->mesh).max.x > 12.5f);
    // Undo takes them back one by one.
    REQUIRE(run(d, "UNDO").ok);
    CHECK(meshBox(named(d, name)->mesh).max.x < 10.5f);
}

TEST_CASE("curve brush: tube profiles and taper") {
    std::vector<Vec3> pts, ns;
    for (int i = 0; i <= 20; ++i) pts.push_back({static_cast<float>(i), 0, 0}), ns.push_back({0, 0, 1});
    const auto stroke = evenStroke(pts, ns, 0.5f);
    // Each profile closed, its section about the shape it says (radius 1, 20 mm long).
    struct Want {
        TubeProfile p;
        double area; // of the section
    };
    const double pi = 3.14159265358979;
    for (const Want& w : {Want{TubeProfile::Square, 4.0}, Want{TubeProfile::Flat, pi * 0.35}, Want{TubeProfile::Triangle, 0.75 * std::sqrt(3.0)},
                          Want{TubeProfile::Star, 5 * 0.5 * std::sin(2 * pi / 10)}}) {
        const MeshData t = curveTube(stroke, 1.0f, 0.5f, w.p);
        REQUIRE(!t.indices.empty());
        CHECK(watertight(t));
        CHECK(std::fabs(meshVolume(t) / 20 - w.area) < 0.06 * w.area);
    }
    // A flat ribbon lies on the surface: wide across, thin up.
    const AABB flat = meshBox(curveTube(stroke, 1.0f, 0.0f, TubeProfile::Flat));
    CHECK(std::fabs((flat.max.y - flat.min.y) - 2.0f) < 0.05f);
    CHECK(std::fabs((flat.max.z - flat.min.z) - 0.7f) < 0.05f);
    CHECK(std::fabs(flat.min.z) < 0.05f); // resting on it (sink 0)
    // Tapered to a point: a cone 20 long of radius 1, so a third of the cylinder.
    const MeshData cone = curveTube(stroke, 1.0f, 0.5f, TubeProfile::Square, 1.0f, 0.0f);
    CHECK(watertight(cone));
    CHECK(std::fabs(meshVolume(cone) - 4.0 * 20 / 3) < 0.08 * 4.0 * 20 / 3);
    CHECK(tubeProfileFromName("ribbon") == TubeProfile::Flat);
    CHECK(!tubeProfileFromName("hexagon"));
    // From the command line.
    Document d;
    REQUIRE(run(d, "BOX 30 20 10").ok);
    const std::string name = d.scene.bodies().back().name;
    const auto o = run(d, "CURVE \"" + name + "\" TUBE PATH -10,0,10 10,0,10 RADIUS 2 PROFILE STAR TAPER 1 0.3");
    REQUIRE(o.ok);
    CHECK(joined(o).find("star tube") != std::string::npos);
    CHECK(openEdgeCount(named(d, name)->mesh) == 0);
    CHECK(!run(d, "CURVE \"" + name + "\" TUBE PATH -10,0,10 10,0,10 PROFILE HEXAGON").ok);
    CHECK(!run(d, "CURVE \"" + name + "\" TUBE PATH -10,0,10 10,0,10 TAPER 0 0").ok);
    CHECK(!run(d, "CURVE \"" + name + "\" TUBE PATH -10,0,10 10,0,10 TAPER 1").ok);
}

TEST_CASE("INSERTMESH and CURVE: what they refuse") {
    Document d;
    CHECK(!run(d, "INSERTMESH SPHERE AT 0,0,0").ok); // nothing selected
    REQUIRE(run(d, "BOX 20 20 10").ok);
    const std::string name = d.scene.bodies().back().name;
    const std::string q = "\"" + name + "\"";
    CHECK(!run(d, "INSERTMESH " + q + " SPHERE").ok);                       // no point
    CHECK(!run(d, "INSERTMESH " + q + " AT 0,0,10").ok);                    // no shape
    CHECK(!run(d, "INSERTMESH " + q + " SPHERE AT 0,0,100").ok);            // far off the body
    CHECK(!run(d, "INSERTMESH " + q + " SPHERE AT 0,0,10 SINK 2").ok);      // sink out of range
    CHECK(!run(d, "INSERTMESH " + q + " SPHERE AT 0,0,10 SIZE -1").ok);     // size
    CHECK(!run(d, "INSERTMESH " + q + " BODY " + q + " AT 0,0,10").ok);     // itself
    CHECK(!run(d, "INSERTMESH " + q + " BODY \"Nope\" AT 0,0,10").ok);      // no such body
    CHECK(!run(d, "INSERTMESH " + q + " SPHERE AT 0,0,10 0,1,10").ok);      // one point only
    CHECK(!run(d, "CURVE " + q + " TUBE PATH 0,0,10").ok);                  // one point
    CHECK(!run(d, "CURVE " + q + " INSERT PATH 0,0,10 5,0,10").ok);         // no shape
    CHECK(!run(d, "INSERTMESH \"Nope\" SPHERE AT 0,0,10").ok);              // no such body
    CHECK(!run(d, "INSERTMESH " + q + " SPHERE AT 0,0,10 WOBBLE").ok);      // unknown word
}

TEST_CASE("insert brushes: kept in a saved project, and over MCP") {
    Document d;
    REQUIRE(run(d, "BOX 20 20 10").ok);
    const std::string name = d.scene.bodies().back().name;
    REQUIRE(run(d, "INSERTMESH \"" + name + "\" SPHERE AT 0,0,10 SIZE 4 APPEND").ok);
    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_insert_test.sicdm").string();
    std::string err;
    REQUIRE(saveProject(d.scene, d.state, path, &err));
    Document e;
    REQUIRE(loadProject(e.scene, e.state, path, &err, nullptr));
    std::error_code ec;
    std::filesystem::remove(path, ec);
    const Body* a = named(d, name);
    const Body* b = named(e, name);
    REQUIRE(b);
    CHECK(b->mesh.indices.size() == a->mesh.indices.size());
    CHECK(b->mesh.groups == a->mesh.groups);

    Document m;
    McpServer server(m, nullptr);
    const auto tool = [&](const std::string& args) {
        const std::string msg = R"({"jsonrpc":"2.0","id":9,"method":"tools/call","params":{"name":"insert_mesh","arguments":)" + args + "}}";
        const auto r = Json::parse(server.handle(msg));
        return r ? (*r)["result"] : Json();
    };
    REQUIRE(run(m, "BOX 20 20 10").ok);
    const std::string box = m.scene.bodies().back().name;
    CHECK(!tool(R"({"body":")" + box + R"(","shape":"cylinder","at":[0,0,10],"size":5})")["isError"].asBool());
    CHECK(meshBox(named(m, box)->mesh).max.z > 13.5f);
    CHECK(!tool(R"({"shape":"tube","path":[[-8,-8,10],[8,-8,10]],"radius":1,"join":"new"})")["isError"].asBool());
    CHECK(named(m, "Tube 1") != nullptr);
    CHECK(!tool(R"({"body":")" + box + R"(","shape":"cone","path":[[-8,8,10],[8,8,10]],"size":2,"spacing":5})")["isError"].asBool());
    CHECK(tool(R"({"body":")" + box + R"(","shape":"tube","at":[0,0,10]})")["isError"].asBool());
    CHECK(tool(R"({"body":")" + box + R"(","shape":"sphere"})")["isError"].asBool());
}

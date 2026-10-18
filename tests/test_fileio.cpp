#include "TestFramework.hpp"

#include "core/FileIO.hpp"
#include "core/Json.hpp"

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>

using namespace sicdm;
namespace fs = std::filesystem;

namespace {

fs::path tempDir() {
    const fs::path d = fs::temp_directory_path() / "sicdm_tests";
    fs::create_directories(d);
    return d;
}

std::string tmp(const char* name) { return (tempDir() / name).string(); }

// 2x1 24-bit BMP: red, blue.
void writeBmp(const std::string& path) {
    const unsigned char px[] = {0, 0, 255, 255, 0, 0, 0, 0}; // BGR BGR + 2 pad bytes
    std::uint32_t fileSize = 54 + sizeof(px), offset = 54, hdr = 40, w = 2, h = 1, imgSize = sizeof(px);
    std::uint16_t planes = 1, bpp = 24;
    std::uint32_t zero = 0;
    std::ofstream f(path, std::ios::binary);
    f.write("BM", 2);
    f.write(reinterpret_cast<char*>(&fileSize), 4);
    f.write(reinterpret_cast<char*>(&zero), 4);
    f.write(reinterpret_cast<char*>(&offset), 4);
    f.write(reinterpret_cast<char*>(&hdr), 4);
    f.write(reinterpret_cast<char*>(&w), 4);
    f.write(reinterpret_cast<char*>(&h), 4);
    f.write(reinterpret_cast<char*>(&planes), 2);
    f.write(reinterpret_cast<char*>(&bpp), 2);
    f.write(reinterpret_cast<char*>(&zero), 4);
    f.write(reinterpret_cast<char*>(&imgSize), 4);
    for (int i = 0; i < 4; ++i) f.write(reinterpret_cast<char*>(&zero), 4);
    f.write(reinterpret_cast<const char*>(px), sizeof(px));
}

} // namespace

TEST_CASE("json parses, edits and round-trips") {
    std::string err;
    auto j = Json::parse(R"({"a":[1,2.5,-3e2],"b":{"c":"x\nyé"},"d":true,"e":null})", &err);
    CHECK(j.has_value());
    if (!j) return;
    CHECK((*j)["a"].size() == 3);
    CHECK_NEAR((*j)["a"][1].asNumber(), 2.5, 1e-12);
    CHECK((*j)["b"]["c"].asString() == "x\ny\xc3\xa9");
    CHECK((*j)["d"].asBool());
    CHECK((*j)["e"].isNull());
    CHECK((*j)["missing"]["deeper"].isNull());
    Json copy = *j;
    copy.set("d", false);
    CHECK((*j)["d"].asBool()); // copy-on-write
    const auto again = Json::parse(copy.dump());
    CHECK(again && (*again)["b"]["c"].asString() == (*j)["b"]["c"].asString());
    CHECK(!Json::parse("{\"a\":}"));
    CHECK(!Json::parse("[1,2"));
    CHECK(!Json::parse("1 2"));
}

TEST_CASE("project save and load round-trips bodies, sketches and images") {
    Scene s = Scene::makeDemo();
    s.bodies()[1].offset = {5, 6, 7};
    s.bodies()[2].visible = false;
    SketchCurve c;
    c.kind = CurveKind::Circle;
    c.plane = SketchPlane::Front;
    c.pts = {{3, 4}};
    c.radius = 12.5f;
    s.sketch().add(c);
    const std::string bmp = tmp("ref.bmp");
    writeBmp(bmp);
    std::string err;
    auto img = importImage(bmp, SketchPlane::Right, {10, 20}, &err);
    CHECK(img.has_value());
    if (img) {
        img->width = 80;
        s.addImage(*img);
    }

    const std::string path = tmp("demo.sicdm");
    CHECK(saveProject(s, {SketchPlane::Front}, path, &err));

    Scene loaded;
    ProjectState st;
    std::string warn;
    const auto before = loaded.revision();
    CHECK(loadProject(loaded, st, path, &err, &warn));
    CHECK(loaded.revision() != before);
    CHECK(st.plane == SketchPlane::Front);
    CHECK(loaded.bodies().size() == s.bodies().size());
    CHECK(loaded.bodies()[1].kind == "tube");
    CHECK_NEAR(loaded.bodies()[1].offset.z, 7, 1e-6);
    CHECK(!loaded.bodies()[2].visible);
    CHECK(loaded.bodies()[3].kind == "mesh"); // studs: stored as triangles
    CHECK(loaded.bodies()[3].mesh.indices.size() == s.bodies()[3].mesh.indices.size());
    CHECK(loaded.sketch().curves().size() == 1);
    CHECK_NEAR(loaded.sketch().curves()[0].radius, 12.5, 1e-5);
    CHECK(loaded.sketch().curves()[0].plane == SketchPlane::Front);
    CHECK(loaded.images().size() == 1);
    if (!loaded.images().empty()) {
        const ImageRef& li = loaded.images()[0];
        CHECK(li.pixelWidth == 2 && li.pixelHeight == 1);
        CHECK(li.plane == SketchPlane::Right);
        CHECK_NEAR(li.width, 80, 1e-5);
        CHECK(li.pixels.size() == 8);
        CHECK(li.pixels[0] == 255 && li.pixels[2] == 0); // first pixel red
    }
    CHECK(warn.empty());
}

TEST_CASE("loading rejects foreign or newer files") {
    const std::string bad = tmp("bad.sicdm");
    std::ofstream(bad) << R"({"format":"something-else"})";
    Scene s;
    ProjectState st;
    std::string err;
    CHECK(!loadProject(s, st, bad, &err));
    std::ofstream(bad) << R"({"format":"sicdm-project","version":99})";
    CHECK(!loadProject(s, st, bad, &err));
}

TEST_CASE("STL and OBJ export then import keep the triangles") {
    Scene s;
    s.add(makeBody("Cube", "box", {0, 0, 0, 10, 10, 10}, {100, 0, 0}));
    std::string err;
    const std::string stl = tmp("cube.stl"), obj = tmp("cube.obj");
    CHECK(exportStl(s, stl, &err));
    CHECK(fs::file_size(stl) == 84 + 50 * 12);
    CHECK(exportObj(s, obj, &err));
    for (const std::string& p : {stl, obj}) {
        auto b = importMesh(p, &err);
        CHECK(b.has_value());
        if (!b) continue;
        CHECK(b->mesh.indices.size() == 36);
        CHECK_NEAR(b->mesh.bounds().min.x, 100, 1e-4); // offset baked into world coordinates
        CHECK(b->mesh.edgeVertices.size() == 24);      // 12 cube edges, diagonals dropped
        CHECK(b->name == "cube");
    }
    // ASCII STL.
    const std::string ascii = tmp("tri.stl");
    std::ofstream(ascii) << "solid t\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\n"
                            "endloop\nendfacet\nendsolid t\n";
    auto t = importMesh(ascii, &err);
    CHECK(t && t->mesh.indices.size() == 3);
    Scene empty;
    CHECK(!exportStl(empty, stl, &err));
}

TEST_CASE("DXF export and import round-trip sketch curves") {
    Scene s;
    SketchCurve line;
    line.kind = CurveKind::Line;
    line.pts = {{0, 0}, {50, 0}};
    s.sketch().add(line);
    SketchCurve rect;
    rect.kind = CurveKind::Rectangle;
    rect.pts = {{0, 0}, {30, 20}};
    s.sketch().add(rect);
    SketchCurve circ;
    circ.kind = CurveKind::Circle;
    circ.pts = {{10, 10}};
    circ.radius = 4;
    s.sketch().add(circ);
    SketchCurve other; // different plane: not exported
    other.kind = CurveKind::Point;
    other.plane = SketchPlane::Front;
    other.pts = {{1, 1}};
    s.sketch().add(other);

    std::string err;
    const std::string dxf = tmp("drawing.dxf");
    CHECK(exportDxf(s, SketchPlane::Top, dxf, &err) == 3);
    Scene in;
    CHECK(importDxf(in, SketchPlane::Front, dxf, &err) == 3);
    const auto& c = in.sketch().curves();
    CHECK(c.size() == 3);
    if (c.size() == 3) {
        CHECK(c[0].kind == CurveKind::Line && c[0].plane == SketchPlane::Front);
        CHECK_NEAR(c[0].pts[1].x, 50, 1e-4);
        CHECK(c[1].kind == CurveKind::Polyline && c[1].pts.size() == 5); // closed rectangle
        CHECK(c[2].kind == CurveKind::Circle);
        CHECK_NEAR(c[2].radius, 4, 1e-4);
    }
    CHECK(exportSvg(s, SketchPlane::Top, tmp("drawing.svg"), &err) == 3);
    CHECK(exportDxf(s, SketchPlane::Right, dxf, &err) == -1); // nothing there
}

TEST_CASE("DXF import understands LWPOLYLINE and ARC") {
    const std::string path = tmp("lw.dxf");
    std::ofstream(path) << "0\nSECTION\n2\nENTITIES\n"
                           "0\nLWPOLYLINE\n8\n0\n90\n3\n70\n1\n10\n0\n20\n0\n10\n10\n20\n0\n10\n10\n20\n10\n"
                           "0\nARC\n8\n0\n10\n0\n20\n0\n40\n5\n50\n0\n51\n90\n"
                           "0\nENDSEC\n0\nEOF\n";
    Scene s;
    std::string err;
    CHECK(importDxf(s, SketchPlane::Top, path, &err) == 2);
    const auto& c = s.sketch().curves();
    CHECK(c.size() == 2);
    if (c.size() == 2) {
        CHECK(c[0].pts.size() == 4); // closed: start repeated
        CHECK_NEAR(c[1].pts.back().y, 5, 1e-4);
        CHECK_NEAR(c[1].pts.back().x, 0, 1e-4);
    }
}

TEST_CASE("file kinds come from the extension") {
    CHECK(fileKindOf("a/b.SICDM") == FileKind::Project);
    CHECK(fileKindOf("x.JPEG") == FileKind::Image);
    CHECK(fileKindOf("x.png") == FileKind::Image);
    CHECK(fileKindOf("x.stl") == FileKind::Stl);
    CHECK(fileKindOf("x.dxf") == FileKind::Dxf);
    CHECK(fileKindOf("x.doc") == FileKind::Unknown);
    std::string err;
    CHECK(!importImage(tmp("missing.png"), SketchPlane::Top, {}, &err));
    CHECK(!err.empty());
}

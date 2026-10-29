#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Geometry.hpp"
#include "core/MeshImport.hpp"
#include "core/Sculpt.hpp"
#include "core/Xml.hpp"
#include "core/Zip.hpp"

#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
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

#include "meshimport_fixtures.inc"

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }

std::string text(const CommandOutput& o) {
    std::string s;
    for (const std::string& l : o.lines) s += l + "\n";
    return s;
}

bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

fs::path freshDir(const char* name) {
    const fs::path p = fs::temp_directory_path() / name;
    std::error_code ec;
    fs::remove_all(p, ec);
    fs::create_directories(p, ec);
    return p;
}

void write(const fs::path& p, const std::string& s) {
    std::ofstream f(p, std::ios::binary);
    f.write(s.data(), static_cast<std::streamsize>(s.size()));
}

double volumeOf(const ImportedPart& p) {
    double v = 0;
    for (const auto& t : p.triangles) v += dot(p.points[t[0]], cross(p.points[t[1]], p.points[t[2]])) / 6.0;
    return v;
}

AABB boxOf(const ImportedPart& p) {
    AABB b;
    for (const Vec3& q : p.points) b.expand(q);
    return b;
}

bool nearV(const Vec3& a, const Vec3& b, float tol = 1e-3f) { return std::fabs(a.x - b.x) <= tol && std::fabs(a.y - b.y) <= tol && std::fabs(a.z - b.z) <= tol; }

// A unit cube's corners and outward triangles.
const char* const kCubeOff = R"(8 12 0
0 0 0
1 0 0
1 1 0
0 1 0
0 0 1
1 0 1
1 1 1
0 1 1
)";
const int kCubeTris[12][3] = {{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4}, {1, 2, 6}, {1, 6, 5}, {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7}};

std::string base64Of(const std::string& s) {
    static const char* a = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    std::size_t i = 0;
    for (; i + 2 < s.size(); i += 3) {
        const unsigned v = static_cast<unsigned char>(s[i]) << 16 | static_cast<unsigned char>(s[i + 1]) << 8 | static_cast<unsigned char>(s[i + 2]);
        out += {a[v >> 18], a[(v >> 12) & 63], a[(v >> 6) & 63], a[v & 63]};
    }
    if (i + 1 == s.size()) {
        const unsigned v = static_cast<unsigned char>(s[i]) << 16;
        out += {a[v >> 18], a[(v >> 12) & 63], '=', '='};
    } else if (i + 2 == s.size()) {
        const unsigned v = static_cast<unsigned char>(s[i]) << 16 | static_cast<unsigned char>(s[i + 1]) << 8;
        out += {a[v >> 18], a[(v >> 12) & 63], a[(v >> 6) & 63], '='};
    }
    return out;
}

// A glTF buffer: the unit cube's points (floats, metres) and its triangles (unsigned shorts).
std::string cubeBuffer() {
    std::string b;
    const float pts[8][3] = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}};
    b.append(reinterpret_cast<const char*>(pts), sizeof pts); // 96 bytes
    for (const auto& t : kCubeTris)
        for (int k = 0; k < 3; ++k) {
            const std::uint16_t v = static_cast<std::uint16_t>(t[k]);
            b.append(reinterpret_cast<const char*>(&v), 2);
        }
    return b; // 96 + 72
}

std::string cubeGltfJson(const std::string& bufferUri, bool glb) {
    return std::string(R"({"asset":{"version":"2.0"},"scene":0,"scenes":[{"nodes":[0]}],
 "nodes":[{"name":"Frame","rotation":[0,0,0.7071067811865476,0.7071067811865476],"children":[1]},
          {"name":"Block","mesh":0,"translation":[2,0,0],"scale":[0.01,0.02,0.03]}],
 "meshes":[{"name":"cube","primitives":[{"attributes":{"POSITION":0},"indices":1,"material":0}]}],
 "materials":[{"name":"Steel","pbrMetallicRoughness":{"baseColorFactor":[0.2140,0.2140,0.2140,1],"metallicFactor":1,"roughnessFactor":0.3}}],
 "accessors":[{"bufferView":0,"componentType":5126,"count":8,"type":"VEC3","min":[0,0,0],"max":[1,1,1]},
              {"bufferView":1,"componentType":5123,"count":36,"type":"SCALAR"}],
 "bufferViews":[{"buffer":0,"byteOffset":0,"byteLength":96},{"buffer":0,"byteOffset":96,"byteLength":72}],
 "buffers":[{"byteLength":168)") +
           (glb ? "" : R"(,"uri":")" + bufferUri + "\"") + "}]}";
}

// A zip package with its members stored as they are.
std::string storedZip(const std::vector<std::pair<std::string, std::string>>& members) {
    std::string out, dir;
    auto u16 = [](std::string& s, unsigned v) { s.push_back(static_cast<char>(v & 0xFF)), s.push_back(static_cast<char>((v >> 8) & 0xFF)); };
    auto u32 = [&](std::string& s, std::uint32_t v) { u16(s, v & 0xFFFF), u16(s, v >> 16); };
    for (const auto& [name, data] : members) {
        const std::uint32_t crc = crc32Of(data.data(), data.size()), at = static_cast<std::uint32_t>(out.size());
        u32(out, 0x04034b50u), u16(out, 20), u16(out, 0), u16(out, 0), u16(out, 0), u16(out, 0), u32(out, crc);
        u32(out, static_cast<std::uint32_t>(data.size())), u32(out, static_cast<std::uint32_t>(data.size())), u16(out, static_cast<unsigned>(name.size())), u16(out, 0);
        out += name + data;
        u32(dir, 0x02014b50u), u16(dir, 20), u16(dir, 20), u16(dir, 0), u16(dir, 0), u16(dir, 0), u16(dir, 0), u32(dir, crc);
        u32(dir, static_cast<std::uint32_t>(data.size())), u32(dir, static_cast<std::uint32_t>(data.size())), u16(dir, static_cast<unsigned>(name.size()));
        u16(dir, 0), u16(dir, 0), u16(dir, 0), u16(dir, 0), u32(dir, 0), u32(dir, at);
        dir += name;
    }
    const std::uint32_t cd = static_cast<std::uint32_t>(out.size());
    out += dir;
    u32(out, 0x06054b50u), u16(out, 0), u16(out, 0), u16(out, static_cast<unsigned>(members.size())), u16(out, static_cast<unsigned>(members.size()));
    u32(out, static_cast<std::uint32_t>(dir.size())), u32(out, cd), u16(out, 0);
    return out;
}

} // namespace

TEST_CASE("deflate: dynamic, fixed and stored blocks; damaged data refused") {
    std::string out, why;
    REQUIRE(inflateRaw(kDyn, sizeof kDyn, out, 1 << 20, &why));
    CHECK(out == kDynText);
    out.clear();
    REQUIRE(inflateRaw(kStored, sizeof kStored, out, 1 << 20, &why));
    CHECK(out == "stored block, as it is");
    out.clear();
    REQUIRE(inflateRaw(kFixed, sizeof kFixed, out, 1 << 20, &why));
    CHECK(out == "abcabcabcabcabc fixed codes fixed codes");
    // Cut short, too big for its limit: refused.
    out.clear();
    CHECK(!inflateRaw(kDyn, sizeof kDyn / 2, out, 1 << 20, &why));
    out.clear();
    CHECK(!inflateRaw(kDyn, sizeof kDyn, out, 100, &why));
    CHECK(why.find("more than") != std::string::npos);
    CHECK(crc32Of("123456789", 9) == 0xCBF43926u); // the check value
}

TEST_CASE("XML: elements, attributes, entities, CDATA and comments; broken XML refused") {
    std::string why;
    const auto x = parseXml("\xEF\xBB\xBF<?xml version=\"1.0\"?>\n<!-- c --><!DOCTYPE m [<!ENTITY a \"b\">]>\n"
                            "<m:model xmlns:m=\"u\" unit='inch'><a k=\"1 &lt; 2 &amp;&#65;&#x42;\">t<![CDATA[<raw>]]>x</a><b/><a k=\"2\"/></m:model>",
                            &why);
    REQUIRE(x.has_value());
    CHECK(x->name == "model");
    CHECK(x->attrOr("unit") == "inch");
    REQUIRE(x->childrenNamed("a").size() == 2);
    CHECK(x->child("a")->attrOr("k") == "1 < 2 &AB");
    CHECK(x->child("a")->text == "t<raw>x");
    CHECK(x->child("b") != nullptr);
    CHECK(!parseXml("<a><b></a>", &why).has_value());
    CHECK(why.find("closes") != std::string::npos);
    CHECK(!parseXml("<a k=1/>").has_value());
}

TEST_CASE("3MF: a deflated package with units, components, build transforms and colours") {
    const fs::path dir = freshDir("sicdm_meshimport");
    write(dir / "pair.3mf", std::string(reinterpret_cast<const char*>(k3mf), sizeof k3mf));
    MeshImportReport rep;
    std::string why;
    const auto parts = importMeshModel((dir / "pair.3mf").string(), &rep, &why);
    REQUIRE(parts.size() == 2); // the support left out
    CHECK(rep.format == "3MF");
    CHECK(near(rep.unitToMm, 25.4, 1e-9));
    CHECK(!rep.warnings.empty()); // the support
    for (const ImportedPart& p : parts) {
        CHECK(p.name == "Cube");
        CHECK(p.assembly == "Pair");
        CHECK(near(volumeOf(p), 25.4 * 25.4 * 25.4, 1e-2)); // an inch cube, wound outward
        CHECK(p.cornerColors.size() == 36);                    // red, one side blue
    }
    CHECK(nearV(boxOf(parts[0]).min, {0, 0, 50.8f}));
    CHECK(nearV(boxOf(parts[1]).min, {76.2f, 0, 50.8f}));
    const Vec3 red = unpackColor(parts[0].cornerColors[0]);
    CHECK(near(red.x, 1, 1e-3) && near(red.z, 0, 1e-3));
    CHECK(near(unpackColor(parts[0].cornerColors[33]).z, 1, 1e-3));
    // Through IMPORT: bodies named apart, in their assembly, a colour per corner.
    Document d;
    const CommandOutput o = run(d, "IMPORT \"" + (dir / "pair.3mf").string() + "\"");
    REQUIRE(o.ok);
    CHECK(text(o).find("Imported 2 parts") != std::string::npos);
    CHECK(text(o).find("1 file unit = 25.4 mm") != std::string::npos);
    REQUIRE(d.scene.bodies().size() == 2);
    CHECK(d.scene.bodies()[0].name == "Cube");
    CHECK(d.scene.bodies()[1].name == "Cube 2");
    CHECK(d.scene.bodies()[1].assembly == "Pair");
    CHECK(near(meshVolume(d.scene.bodies()[0].mesh), 25.4 * 25.4 * 25.4, 1e-1));
    REQUIRE(run(d, "UNDO").ok);
    CHECK(d.scene.bodies().empty());
    // Our own 3MF (stored, not deflated) back in.
    Document e;
    REQUIRE(run(e, "BOX 20 10 5 @0,0,0").ok);
    REQUIRE(run(e, "CYLINDER 4 10 @30,0,0").ok);
    REQUIRE(run(e, "EXPORT \"" + (dir / "own.3mf").string() + "\"").ok);
    const auto back = importMeshModel((dir / "own.3mf").string(), &rep, &why);
    REQUIRE(back.size() == 2);
    CHECK(back[0].name == "Block 1");
    CHECK(near(volumeOf(back[0]), 1000, 1e-2));
    // Not a package; a damaged one.
    write(dir / "bad.3mf", "PK\x03\x04 not really");
    CHECK(importMeshModel((dir / "bad.3mf").string(), &rep, &why).empty());
    std::string damaged(reinterpret_cast<const char*>(k3mf), sizeof k3mf);
    const std::size_t member = damaged.find("3D/pair.model"); // its local header's name: the data follows
    REQUIRE(member != std::string::npos);
    damaged[member + 13 + 60] = static_cast<char>(damaged[member + 13 + 60] ^ 0x55);
    write(dir / "damaged.3mf", damaged);
    CHECK(importMeshModel((dir / "damaged.3mf").string(), &rep, &why).empty());
}

TEST_CASE("AMF: objects, volume colours, materials and a constellation of instances") {
    const fs::path dir = freshDir("sicdm_meshimport_amf");
    std::string amf = R"(<?xml version="1.0"?>
<amf unit="millimeter">
 <material id="7"><color><r>0</r><g>0.5</g><b>0</b></color></material>
 <object id="1">
  <metadata type="name">Brick</metadata>
  <mesh>
   <vertices>)";
    const float pts[8][3] = {{0, 0, 0}, {10, 0, 0}, {10, 10, 0}, {0, 10, 0}, {0, 0, 10}, {10, 0, 10}, {10, 10, 10}, {0, 10, 10}};
    for (const auto& p : pts)
        amf += "<vertex><coordinates><x>" + std::to_string(p[0]) + "</x><y>" + std::to_string(p[1]) + "</y><z>" + std::to_string(p[2]) + "</z></coordinates></vertex>\n";
    amf += "</vertices>\n<volume materialid=\"7\">\n";
    for (const auto& t : kCubeTris) amf += "<triangle><v1>" + std::to_string(t[0]) + "</v1><v2>" + std::to_string(t[1]) + "</v2><v3>" + std::to_string(t[2]) + "</v3></triangle>\n";
    amf += R"(</volume>
  </mesh>
 </object>
 <constellation id="2">
  <instance objectid="1"><deltax>0</deltax><deltay>0</deltay><deltaz>0</deltaz></instance>
  <instance objectid="1"><deltax>50</deltax><deltay>0</deltay><deltaz>0</deltaz><rz>90</rz></instance>
 </constellation>
</amf>
)";
    write(dir / "bricks.amf", amf);
    MeshImportReport rep;
    std::string why;
    const auto parts = importMeshModel((dir / "bricks.amf").string(), &rep, &why);
    REQUIRE(parts.size() == 2);
    CHECK(rep.format == "AMF");
    CHECK(parts[0].name == "Brick");
    REQUIRE(parts[0].color.has_value());
    CHECK(near(parts[0].color->y, 0.5, 2e-3));
    CHECK(near(volumeOf(parts[0]), 1000, 1e-3));
    // The second turned a quarter about Z, then moved: x from 40 to 50.
    CHECK(nearV(boxOf(parts[1]).min, {40, 0, 0}, 1e-3f));
    CHECK(nearV(boxOf(parts[1]).max, {50, 10, 10}, 1e-3f));
    CHECK(near(volumeOf(parts[1]), 1000, 1e-3));
}

TEST_CASE("PLY (ASCII and binary) and OFF: points, faces, colours; units taken as mm unless given") {
    const fs::path dir = freshDir("sicdm_meshimport_ply");
    std::string ply = "ply\nformat ascii 1.0\ncomment made by hand\nelement vertex 8\nproperty float x\nproperty float y\nproperty float z\n"
                      "property uchar red\nproperty uchar green\nproperty uchar blue\nelement face 6\nproperty list uchar int vertex_indices\nend_header\n";
    const int pts[8][3] = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}};
    for (const auto& p : pts) ply += std::to_string(p[0]) + " " + std::to_string(p[1]) + " " + std::to_string(p[2]) + (p[2] ? " 255 0 0\n" : " 0 0 255\n");
    ply += "4 0 3 2 1\n4 4 5 6 7\n4 0 1 5 4\n4 1 2 6 5\n4 2 3 7 6\n4 3 0 4 7\n"; // quads, fanned
    write(dir / "cube.ply", ply);
    MeshImportReport rep;
    std::string why;
    auto parts = importMeshModel((dir / "cube.ply").string(), &rep, &why);
    REQUIRE(parts.size() == 1);
    CHECK(rep.format == "PLY (ASCII)");
    CHECK(rep.unitAssumed);
    CHECK(parts[0].name == "cube");
    CHECK(parts[0].triangles.size() == 12);
    CHECK(near(volumeOf(parts[0]), 1, 1e-6));
    CHECK(parts[0].cornerColors.size() == 36); // red on top, blue below
    // Binary, big endian, in inches (given).
    std::string bin = "ply\nformat binary_big_endian 1.0\nelement vertex 8\nproperty double x\nproperty double y\nproperty double z\n"
                      "element face 12\nproperty list uchar uint vertex_index\nend_header\n";
    auto be = [&](const void* v, int n) {
        const char* c = static_cast<const char*>(v);
        for (int k = n - 1; k >= 0; --k) bin.push_back(c[k]);
    };
    for (const auto& p : pts)
        for (int k = 0; k < 3; ++k) {
            const double v = p[k];
            be(&v, 8);
        }
    for (const auto& t : kCubeTris) {
        bin.push_back(3);
        for (int k = 0; k < 3; ++k) {
            const std::uint32_t v = static_cast<std::uint32_t>(t[k]);
            be(&v, 4);
        }
    }
    write(dir / "cube_be.ply", bin);
    parts = importMeshModel((dir / "cube_be.ply").string(), &rep, &why, 25.4);
    REQUIRE(parts.size() == 1);
    CHECK(rep.format == "PLY (binary)");
    CHECK(!rep.unitAssumed);
    CHECK(near(volumeOf(parts[0]), 25.4 * 25.4 * 25.4, 1e-1));
    // OFF with a colour on each face.
    std::string off = std::string("COFF\n# a cube\n") + kCubeOff;
    for (const auto& t : kCubeTris) off += "3 " + std::to_string(t[0]) + " " + std::to_string(t[1]) + " " + std::to_string(t[2]) + " 0 255 0\n";
    // (COFF: vertex colours; these vertices have none, the faces carry theirs.)
    write(dir / "cube.off", off);
    parts = importMeshModel((dir / "cube.off").string(), &rep, &why);
    REQUIRE(parts.size() == 1);
    CHECK(near(volumeOf(parts[0]), 1, 1e-6));
    REQUIRE(parts[0].color.has_value()); // one colour for all: the part's
    CHECK(near(parts[0].color->y, 1, 1e-3));
    // Nothing to import: said so.
    write(dir / "empty.ply", "ply\nformat ascii 1.0\nelement vertex 0\nelement face 0\nend_header\n");
    CHECK(importMeshModel((dir / "empty.ply").string(), &rep, &why).empty());
    CHECK(why.find("no faces") != std::string::npos);
}

TEST_CASE("glTF and GLB: the node tree's transforms, metres and +Y up to mm and +Z up, materials") {
    const fs::path dir = freshDir("sicdm_meshimport_gltf");
    const std::string buf = cubeBuffer();
    write(dir / "embedded.gltf", cubeGltfJson("data:application/octet-stream;base64," + base64Of(buf), false));
    write(dir / "cube.bin", buf);
    write(dir / "separate.gltf", cubeGltfJson("cube.bin", false));
    // GLB: the JSON chunk and the binary chunk, each padded to four bytes.
    std::string json = cubeGltfJson("", true);
    while (json.size() % 4) json += ' ';
    std::string binc = buf;
    while (binc.size() % 4) binc.push_back('\0');
    std::string glb = "glTF";
    auto u32 = [&](std::uint32_t v) { glb.append(reinterpret_cast<const char*>(&v), 4); };
    u32(2);
    u32(static_cast<std::uint32_t>(12 + 8 + json.size() + 8 + binc.size()));
    u32(static_cast<std::uint32_t>(json.size()));
    u32(0x4E4F534Au);
    glb += json;
    u32(static_cast<std::uint32_t>(binc.size()));
    u32(0x004E4942u);
    glb += binc;
    write(dir / "cube.glb", glb);
    for (const char* f : {"embedded.gltf", "separate.gltf", "cube.glb"}) {
        MeshImportReport rep;
        std::string why;
        const auto parts = importMeshModel((dir / f).string(), &rep, &why);
        REQUIRE(parts.size() == 1);
        const ImportedPart& p = parts[0];
        CHECK(p.name == "Block");
        CHECK(p.assembly == "Frame");
        // 10 x 20 x 30 mm, 2 m along glTF x; the frame turns it a quarter about glTF z (x to y,
        // y to -x); glTF (x, y, z) is CAD (x, -z, y): the 10 mm side runs up CAD z, 2 m up.
        const AABB b = boxOf(p);
        const Vec3 size = b.max - b.min;
        CHECK(nearV(size, {20, 30, 10}, 1e-2f));
        CHECK(nearV(b.min, {-20, -30, 2000}, 1e-2f));
        CHECK(near(volumeOf(p), 6000, 1e-1)); // wound outward still
        REQUIRE(p.color.has_value());
        CHECK(near(p.color->x, 0.5, 1e-2)); // linear 0.214 is sRGB 0.5
        REQUIRE(p.metallic.has_value());
        CHECK(near(*p.metallic, 1, 1e-6));
        CHECK(near(*p.roughness, 0.3, 1e-6));
        CHECK(near(rep.unitToMm, 1000, 1e-9));
    }
    // Compressed geometry it can't read: said so.
    write(dir / "draco.gltf", R"({"asset":{"version":"2.0"},"extensionsRequired":["KHR_draco_mesh_compression"]})");
    std::string why;
    CHECK(importMeshModel((dir / "draco.gltf").string(), nullptr, &why).empty());
    CHECK(why.find("KHR_draco_mesh_compression") != std::string::npos);
    write(dir / "old.gltf", R"({"asset":{"version":"1.0"}})");
    CHECK(importMeshModel((dir / "old.gltf").string(), nullptr, &why).empty());
}

TEST_CASE("VRML 97: transforms, shapes, materials, DEF and USE, units") {
    const fs::path dir = freshDir("sicdm_meshimport_wrl");
    std::string coords;
    const int pts[8][3] = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}};
    for (const auto& p : pts) coords += std::to_string(p[0]) + " " + std::to_string(p[1]) + " " + std::to_string(p[2]) + ",\n";
    std::string idx;
    for (const auto& t : kCubeTris) idx += std::to_string(t[0]) + ", " + std::to_string(t[1]) + ", " + std::to_string(t[2]) + ", -1,\n";
    const std::string wrl = "#VRML V2.0 utf8\n# two boxes, the second a copy\nDEF Board Transform {\n  translation 1 0 0\n  scale 2 2 2\n  children [\n"
                            "    DEF Part Shape {\n      appearance Appearance { material Material { diffuseColor 0 0 1 shininess 0.8 } }\n"
                            "      geometry IndexedFaceSet { coord Coordinate { point [ " + coords + " ] } coordIndex [ " + idx + " ] }\n    }\n"
                            "    Transform { translation 0 0 5 children [ USE Part ] }\n  ]\n}\nROUTE A.b TO C.d\n";
    write(dir / "boards.wrl", wrl);
    MeshImportReport rep;
    std::string why;
    auto parts = importMeshModel((dir / "boards.wrl").string(), &rep, &why);
    REQUIRE(parts.size() == 2);
    CHECK(rep.format == "VRML 97");
    CHECK(near(rep.unitToMm, 1000, 1e-9)); // metres
    CHECK(near(volumeOf(parts[0]), 8e9, 1e3)); // 2 m cube
    CHECK(nearV(boxOf(parts[0]).min, {1000, 0, 0}, 1e-1f));
    CHECK(nearV(boxOf(parts[1]).min, {1000, 0, 10000}, 1e-1f)); // (inside the scaled frame)
    REQUIRE(parts[0].color.has_value());
    CHECK(near(parts[0].color->z, 1, 1e-6));
    CHECK(near(*parts[0].roughness, 0.2, 1e-6));
    // Many CAD tools write VRML in other units: UNIT says which.
    Document d;
    const CommandOutput o = run(d, "IMPORT \"" + (dir / "boards.wrl").string() + "\" UNIT mm");
    REQUIRE(o.ok);
    CHECK(text(o).find("1 file unit = 1 mm") != std::string::npos);
    REQUIRE(d.scene.bodies().size() == 2);
    CHECK(near(meshVolume(d.scene.bodies()[0].mesh), 8, 1e-4));
    CHECK(d.scene.bodies()[0].assembly == "Board"); // (the DEF names the Transform's group)
    write(dir / "old.wrl", "#VRML V1.0 ascii\nSeparator {}\n");
    CHECK(importMeshModel((dir / "old.wrl").string(), nullptr, &why).empty());
    CHECK(why.find("1.0") != std::string::npos);
}

TEST_CASE("imported meshes keep sharp edges sharp and curves smooth, and close up as solids") {
    ImportedPart p;
    p.name = "cube";
    const float pts[8][3] = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}};
    for (const auto& q : pts) p.points.push_back({q[0], q[1], q[2]});
    for (const auto& t : kCubeTris) p.triangles.push_back({static_cast<std::uint32_t>(t[0]), static_cast<std::uint32_t>(t[1]), static_cast<std::uint32_t>(t[2])});
    const Body b = bodyFromPart(p);
    CHECK(b.mesh.vertices.size() == 24); // each corner three ways: flat faces
    for (const Vertex& v : b.mesh.vertices) CHECK(std::fabs(std::fabs(v.normal.x) + std::fabs(v.normal.y) + std::fabs(v.normal.z) - 1) < 1e-5f);
    CHECK(watertight(b.mesh));
    CHECK(near(meshVolume(b.mesh), 1, 1e-6));
    // A 32-sided prism: its sides one smooth surface, its caps flat. Each rim point: one side
    // normal and one cap normal; each cap centre one.
    ImportedPart prism;
    const int n = 32;
    for (int z = 0; z < 2; ++z)
        for (int k = 0; k < n; ++k) prism.points.push_back({std::cos(6.2831853f * static_cast<float>(k) / n), std::sin(6.2831853f * static_cast<float>(k) / n), static_cast<float>(z)});
    prism.points.push_back({0, 0, 0});
    prism.points.push_back({0, 0, 1});
    for (int k = 0; k < n; ++k) {
        const std::uint32_t a = static_cast<std::uint32_t>(k), b2 = static_cast<std::uint32_t>((k + 1) % n);
        prism.triangles.push_back({a, b2, b2 + n});
        prism.triangles.push_back({a, b2 + n, a + n});
        prism.triangles.push_back({2 * n, b2, a});
        prism.triangles.push_back({2 * n + 1, a + n, b2 + n});
    }
    const Body pb = bodyFromPart(prism);
    CHECK(pb.mesh.vertices.size() == static_cast<std::size_t>(4 * n + 2));
    CHECK(watertight(pb.mesh));
    CHECK(near(meshVolume(pb.mesh), 0.5 * n * std::sin(6.2831853 / n), 1e-4));
}

TEST_CASE("mesh imports refuse loops and stop files that multiply without end") {
    const fs::path dir = freshDir("sicdm_meshimport_loops");
    MeshImportReport rep;
    std::string why;
    // 3MF: an object that holds itself; the cube beside it still comes in.
    const std::string cube = R"(<object id="1" name="Cube"><mesh><vertices><vertex x="0" y="0" z="0"/><vertex x="1" y="0" z="0"/><vertex x="0" y="1" z="0"/><vertex x="0" y="0" z="1"/></vertices>
<triangles><triangle v1="0" v2="2" v3="1"/><triangle v1="0" v2="1" v3="3"/><triangle v1="0" v2="3" v3="2"/><triangle v1="1" v2="2" v3="3"/></triangles></mesh></object>)";
    const std::string model = R"(<?xml version="1.0"?><model unit="millimeter"><resources>)" + cube +
                              R"(<object id="2" name="Loop"><components><component objectid="1"/><component objectid="2"/></components></object>
</resources><build><item objectid="2"/></build></model>)";
    write(dir / "loop.3mf", storedZip({{"3D/3dmodel.model", model}}));
    auto parts = importMeshModel((dir / "loop.3mf").string(), &rep, &why);
    REQUIRE(parts.size() == 1);
    CHECK(near(volumeOf(parts[0]), 1.0 / 6, 1e-6));
    bool said = false;
    for (const std::string& w : rep.warnings) said = said || w.find("holds itself") != std::string::npos;
    CHECK(said);
    // glTF: a node that is its own child; and a tree that doubles forty times.
    const std::string buf = "data:application/octet-stream;base64," + base64Of(cubeBuffer());
    std::string nodes = R"([{"mesh":0,"children":[0]}])";
    auto gltf = [&](const std::string& n, const std::string& roots) {
        return R"({"asset":{"version":"2.0"},"scenes":[{"nodes":)" + roots + R"(}],"nodes":)" + n +
               R"(,"meshes":[{"primitives":[{"attributes":{"POSITION":0},"indices":1}]}],
 "accessors":[{"bufferView":0,"componentType":5126,"count":8,"type":"VEC3"},{"bufferView":1,"componentType":5123,"count":36,"type":"SCALAR"}],
 "bufferViews":[{"buffer":0,"byteOffset":0,"byteLength":96},{"buffer":0,"byteOffset":96,"byteLength":72}],"buffers":[{"byteLength":168,"uri":")" + buf + "\"}]}";
    };
    write(dir / "loop.gltf", gltf(nodes, "[0]"));
    parts = importMeshModel((dir / "loop.gltf").string(), &rep, &why);
    CHECK(parts.size() == 1);
    nodes = "[";
    for (int k = 0; k < 40; ++k) nodes += "{\"children\":[" + std::to_string(k + 1) + "," + std::to_string(k + 1) + "]},";
    nodes += R"({"mesh":0}])";
    write(dir / "bomb.gltf", gltf(nodes, "[0]"));
    parts = importMeshModel((dir / "bomb.gltf").string(), &rep, &why);
    CHECK(!parts.empty());
    CHECK(parts.size() <= 20000);
    said = false;
    for (const std::string& w : rep.warnings) said = said || w.find("stopped") != std::string::npos;
    CHECK(said);
    // AMF: a constellation that holds itself.
    write(dir / "loop.amf", R"(<amf><object id="1"><mesh><vertices>
<vertex><coordinates><x>0</x><y>0</y><z>0</z></coordinates></vertex><vertex><coordinates><x>1</x><y>0</y><z>0</z></coordinates></vertex>
<vertex><coordinates><x>0</x><y>1</y><z>0</z></coordinates></vertex><vertex><coordinates><x>0</x><y>0</y><z>1</z></coordinates></vertex></vertices>
<volume><triangle><v1>0</v1><v2>2</v2><v3>1</v3></triangle><triangle><v1>0</v1><v2>1</v2><v3>3</v3></triangle>
<triangle><v1>0</v1><v2>3</v2><v3>2</v3></triangle><triangle><v1>1</v1><v2>2</v2><v3>3</v3></triangle></volume></mesh></object>
<constellation id="5"><instance objectid="1"/><instance objectid="5"/></constellation></amf>)");
    parts = importMeshModel((dir / "loop.amf").string(), &rep, &why);
    CHECK(parts.size() == 1);
    // VRML: a node can't use itself; a chain of groups each using the last twice stops.
    write(dir / "self.wrl", "#VRML V2.0 utf8\nDEF A Group { children [ USE A ] }\n");
    CHECK(importMeshModel((dir / "self.wrl").string(), &rep, &why).empty());
    std::string wrl = "#VRML V2.0 utf8\nDEF G0 Shape { geometry IndexedFaceSet { coord Coordinate { point [0 0 0, 1 0 0, 0 1 0] } coordIndex [0 1 2 -1] } }\n";
    for (int k = 1; k <= 40; ++k) wrl += "DEF G" + std::to_string(k) + " Group { children [ USE G" + std::to_string(k - 1) + " USE G" + std::to_string(k - 1) + " ] }\n";
    write(dir / "bomb.wrl", wrl);
    parts = importMeshModel((dir / "bomb.wrl").string(), &rep, &why);
    CHECK(parts.size() <= 20000);
    said = false;
    for (const std::string& w : rep.warnings) said = said || w.find("stopped") != std::string::npos;
    CHECK(said);
}

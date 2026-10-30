#include "TestFramework.hpp"

#include "core/Armature.hpp"
#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/ImageWrite.hpp"
#include "core/Json.hpp"
#include "core/Mcp.hpp"
#include "core/MeshImport.hpp"
#include "core/ModelExport.hpp"
#include "core/Scene.hpp"
#include "core/Sculpt.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <sstream>

using namespace sicdm;

#define REQUIRE(c)                  \
    do {                            \
        const bool required_ = (c); \
        CHECK(required_);           \
        if (!required_) return;     \
    } while (0)

namespace {

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }

std::string tempPath(const std::string& name) { return (std::filesystem::temp_directory_path() / name).string(); }

std::string readAll(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    std::ostringstream s;
    s << f.rdbuf();
    return s.str();
}

void removeFile(const std::string& path) {
    std::error_code ec;
    std::filesystem::remove(path, ec);
}

template <class T> T get(const std::string& s, std::size_t at) {
    T v{};
    if (at + sizeof(T) <= s.size()) std::memcpy(&v, s.data() + at, sizeof(T));
    return v;
}

// A GLB's JSON and binary chunk.
struct Glb {
    Json json;
    std::string bin;

    // An accessor's numbers (floats, or unsigned integers as numbers).
    std::vector<double> numbers(int accessor) const {
        const Json& a = json["accessors"][accessor];
        const Json& v = json["bufferViews"][a["bufferView"].asInt()];
        static const std::map<std::string, int> kComps = {{"SCALAR", 1}, {"VEC2", 2}, {"VEC3", 3}, {"VEC4", 4}, {"MAT4", 16}};
        const std::size_t n = static_cast<std::size_t>(a["count"].asInt() * kComps.at(a["type"].asString()));
        const std::size_t at = static_cast<std::size_t>(v["byteOffset"].asInt());
        std::vector<double> out(n);
        for (std::size_t i = 0; i < n; ++i) {
            switch (a["componentType"].asInt()) {
            case 5126: out[i] = get<float>(bin, at + 4 * i); break;
            case 5123: out[i] = get<std::uint16_t>(bin, at + 2 * i); break;
            default: out[i] = get<std::uint32_t>(bin, at + 4 * i); break;
            }
        }
        return out;
    }
};

std::optional<Glb> readGlb(const std::string& path) {
    const std::string s = readAll(path);
    if (s.size() < 20 || s.compare(0, 4, "glTF") != 0 || get<std::uint32_t>(s, 4) != 2 || get<std::uint32_t>(s, 8) != s.size()) return std::nullopt;
    const std::uint32_t jl = get<std::uint32_t>(s, 12);
    if (s.compare(16, 4, "JSON") != 0 || jl % 4 || 20 + jl + 8 > s.size()) return std::nullopt;
    auto j = Json::parse(s.substr(20, jl));
    if (!j) return std::nullopt;
    const std::uint32_t bl = get<std::uint32_t>(s, 20 + jl);
    if (s.compare(20 + jl + 4, 4, std::string("BIN\0", 4)) != 0) return std::nullopt;
    return Glb{*j, s.substr(20 + jl + 8, bl)};
}

const Json* nodeNamed(const Json& g, const std::string& name) {
    for (std::size_t i = 0; i < g["nodes"].size(); ++i)
        if (g["nodes"][i]["name"].asString() == name) return &g["nodes"][i];
    return nullptr;
}

// A node of a binary FBX file, read back.
struct FbxRead {
    std::string name;
    std::vector<std::string> strings;
    std::vector<double> numbers;               // scalar properties
    std::vector<std::vector<double>> arrays;   // array properties
    std::size_t rawBytes = 0;                  // a raw ('R') property's size
    std::vector<FbxRead> kids;

    const FbxRead* kid(const std::string& n) const {
        for (const FbxRead& k : kids)
            if (k.name == n) return &k;
        return nullptr;
    }
    std::size_t count(const std::string& n) const {
        return static_cast<std::size_t>(std::count_if(kids.begin(), kids.end(), [&](const FbxRead& k) { return k.name == n; }));
    }
};

bool readFbxNode(const std::string& s, std::size_t& at, FbxRead& out, bool& end) {
    const std::uint32_t endAt = get<std::uint32_t>(s, at), props = get<std::uint32_t>(s, at + 4);
    const std::uint8_t nameLen = get<std::uint8_t>(s, at + 12);
    if (endAt == 0) {
        at += 13, end = true;
        return true;
    }
    if (endAt > s.size() || endAt <= at) return false;
    out.name = s.substr(at + 13, nameLen);
    std::size_t p = at + 13 + nameLen;
    for (std::uint32_t i = 0; i < props; ++i) {
        const char t = s[p++];
        switch (t) {
        case 'I': out.numbers.push_back(get<std::int32_t>(s, p)), p += 4; break;
        case 'L': out.numbers.push_back(static_cast<double>(get<std::int64_t>(s, p))), p += 8; break;
        case 'D': out.numbers.push_back(get<double>(s, p)), p += 8; break;
        case 'F': out.numbers.push_back(get<float>(s, p)), p += 4; break;
        case 'C': out.numbers.push_back(get<std::uint8_t>(s, p)), p += 1; break;
        case 'Y': out.numbers.push_back(get<std::int16_t>(s, p)), p += 2; break;
        case 'S':
        case 'R': {
            const std::uint32_t n = get<std::uint32_t>(s, p);
            if (t == 'S') out.strings.push_back(s.substr(p + 4, n));
            else out.rawBytes = n;
            p += 4 + n;
            break;
        }
        case 'd':
        case 'i':
        case 'f':
        case 'l': {
            const std::uint32_t n = get<std::uint32_t>(s, p), enc = get<std::uint32_t>(s, p + 4), bytes = get<std::uint32_t>(s, p + 8);
            if (enc != 0) return false;
            std::vector<double> a(n);
            const std::size_t w = t == 'd' || t == 'l' ? 8 : 4;
            if (bytes != n * w) return false;
            for (std::uint32_t k = 0; k < n; ++k)
                a[k] = t == 'd' ? get<double>(s, p + 12 + 8 * k)
                       : t == 'l' ? static_cast<double>(get<std::int64_t>(s, p + 12 + 8 * k))
                       : t == 'f' ? static_cast<double>(get<float>(s, p + 12 + 4 * k))
                                  : static_cast<double>(get<std::int32_t>(s, p + 12 + 4 * k));
            out.arrays.push_back(std::move(a));
            p += 12 + bytes;
            break;
        }
        default: return false;
        }
    }
    if (p != at + 13 + nameLen + get<std::uint32_t>(s, at + 8)) return false;
    while (p < endAt) {
        FbxRead k;
        bool kidEnd = false;
        if (!readFbxNode(s, p, k, kidEnd)) return false;
        if (kidEnd) break;
        out.kids.push_back(std::move(k));
    }
    if (p != endAt) return false;
    at = endAt;
    return true;
}

std::optional<FbxRead> readFbx(const std::string& path, std::uint32_t* version) {
    const std::string s = readAll(path);
    if (s.size() < 27 || s.compare(0, 23, std::string("Kaydara FBX Binary  \0\x1a\0", 23)) != 0) return std::nullopt;
    *version = get<std::uint32_t>(s, 23);
    FbxRead root;
    std::size_t at = 27;
    for (;;) {
        FbxRead n;
        bool end = false;
        if (!readFbxNode(s, at, n, end)) return std::nullopt;
        if (end) break;
        root.kids.push_back(std::move(n));
    }
    // The footer ends with its fixed 16 bytes.
    static const unsigned char kMagic[16] = {0xf8, 0x5a, 0x8c, 0x6a, 0xde, 0xf5, 0xd9, 0x7e, 0xec, 0xe9, 0x0c, 0xe3, 0x75, 0x8f, 0x29, 0x0b};
    if (s.size() < at + 16 || std::memcmp(s.data() + s.size() - 16, kMagic, 16) != 0) return std::nullopt;
    return root;
}

Vec3 toMetresYUp(const Vec3& p) { return Vec3{p.x, p.z, -p.y} * 0.001f; }

} // namespace

TEST_CASE("glTF export: bodies, assemblies, materials and vertex colours, read back") {
    Document d;
    REQUIRE(run(d, "BOX 40 30 10").ok);
    REQUIRE(run(d, "CYLINDER 8 30 @0,0,10").ok);
    const std::string block = d.scene.bodies()[0].name, post = d.scene.bodies()[1].name;
    REQUIRE(run(d, "ASSEMBLY GROUP \"Base/Post\" \"" + block + "\" \"" + post + "\"").ok);
    REQUIRE(run(d, "MATERIAL \"" + post + "\" Gold").ok);
    REQUIRE(run(d, "BOX 10 10 10 @0,60,0").ok);
    const std::string painted = d.scene.bodies().back().name;
    REQUIRE(run(d, "PAINT \"" + painted + "\" FILL #ff2020 AT 0,60,10").ok);
    REQUIRE(run(d, "SPHERE 5 @80,0,0").ok);
    d.scene.bodies().back().visible = false; // hidden: left out
    for (const char* ext : {".glb", ".gltf"}) {
        const std::string path = tempPath(std::string("sicdm_export_test") + ext);
        const auto r = run(d, "EXPORT " + path);
        REQUIRE(r.ok);
        CHECK(r.lines[0].find("3 bodies") != std::string::npos);
        MeshImportReport rep;
        std::string why;
        const auto parts = importMeshModel(path, &rep, &why);
        removeFile(path);
        REQUIRE(parts.size() == 3);
        for (const ImportedPart& p : parts) {
            const Body* b = nullptr;
            for (const Body& x : d.scene.bodies())
                if (x.name == p.name) b = &x;
            REQUIRE(b);
            // The same place and size, in millimetres and +Z up again.
            AABB box;
            for (const Vec3& q : p.points) box.expand(q);
            const AABB was = b->bounds();
            CHECK(length(box.min - was.min) < 1e-3f);
            CHECK(length(box.max - was.max) < 1e-3f);
            CHECK(p.triangles.size() == b->mesh.indices.size() / 3);
            if (p.name == post) {
                CHECK(p.assembly == "Base/Post");
                CHECK(p.metallic && *p.metallic > 0.99f);
                CHECK(p.color && length(*p.color - b->color) < 0.01f);
            }
            if (p.name == painted) // the paint, as vertex colours
                CHECK((!p.cornerColors.empty() || (p.color && length(*p.color - Vec3{1.0f, 0.125f, 0.125f}) < 0.02f)));
        }
    }
    // The binary's structure: the group nodes hold the bodies, accessors are bounded.
    const std::string path = tempPath("sicdm_export_test2.glb");
    ModelExportReport rep;
    std::string why;
    REQUIRE(exportModel(d.scene, path, &rep, &why));
    const auto g = readGlb(path);
    removeFile(path);
    REQUIRE(g.has_value());
    CHECK(g->json["asset"]["version"].asString() == "2.0");
    const Json* base = nodeNamed(g->json, "Base");
    REQUIRE(base);
    CHECK(base->has("children"));
    const Json* inner = &g->json["nodes"][(*base)["children"][0].asInt()];
    CHECK((*inner)["name"].asString() == "Post");
    CHECK((*inner)["children"].size() == 2);
    const Json& prim = g->json["meshes"][0]["primitives"][0];
    const Json& pos = g->json["accessors"][prim["attributes"]["POSITION"].asInt()];
    CHECK(pos["min"].size() == 3);
    CHECK(std::fabs(pos["max"][0].asNumber() - 0.020) < 1e-6); // the block: 40 mm wide, centred
    CHECK(std::fabs(pos["max"][1].asNumber() - 0.010) < 1e-6); // 10 mm tall, now along +Y
    CHECK(rep.bodies == 3);
    CHECK(rep.materials == 3);
}

TEST_CASE("glTF export: texture coordinates and embedded maps, glass and glow") {
    Document d;
    REQUIRE(run(d, "SPHERE 15").ok);
    const std::string ball = d.scene.bodies().back().name;
    REQUIRE(run(d, "UV \"" + ball + "\" UNWRAP").ok);
    const std::string png = tempPath("sicdm_export_map.png"), ppm = tempPath("sicdm_export_map.ppm");
    {
        std::vector<std::uint8_t> px(16 * 16 * 3, 200);
        REQUIRE(writePng8(png, 16, 16, 3, px));
        std::ofstream f(ppm, std::ios::binary);
        f << "P6\n2 2\n255\n";
        for (int i = 0; i < 4; ++i) f.put(static_cast<char>(128)), f.put(static_cast<char>(64)), f.put(static_cast<char>(32));
    }
    REQUIRE(run(d, "MATERIAL \"" + ball + "\" COLORMAP " + png + " NORMALMAP " + ppm).ok);
    REQUIRE(run(d, "SPHERE 10 @-60,0,0").ok);
    REQUIRE(run(d, "MATERIAL \"" + d.scene.bodies().back().name + "\" TRANSMISSION 1 IOR 1.45").ok);
    REQUIRE(run(d, "SPHERE 4 @60,0,0").ok);
    REQUIRE(run(d, "MATERIAL \"" + d.scene.bodies().back().name + "\" EMISSION 5").ok);
    REQUIRE(run(d, "BOX 10 10 10 @0,60,0").ok);
    REQUIRE(run(d, "MATERIAL \"" + d.scene.bodies().back().name + "\" COLORMAP " + png).ok); // no UVs: the map is left out
    const std::string path = tempPath("sicdm_export_maps.glb");
    ModelExportReport rep;
    std::string why;
    REQUIRE(exportModel(d.scene, path, &rep, &why));
    const auto g = readGlb(path);
    removeFile(path), removeFile(png), removeFile(ppm);
    REQUIRE(g.has_value());
    CHECK(rep.textures == 2);
    CHECK(std::any_of(rep.notes.begin(), rep.notes.end(), [](const std::string& n) { return n.find("no texture coordinates") != std::string::npos; }));
    const Json& j = g->json;
    REQUIRE(j["images"].size() == 2);
    for (std::size_t i = 0; i < 2; ++i) { // both PNG (the PPM written as one)
        CHECK(j["images"][i]["mimeType"].asString() == "image/png");
        const Json& v = j["bufferViews"][j["images"][i]["bufferView"].asInt()];
        CHECK(g->bin.compare(static_cast<std::size_t>(v["byteOffset"].asInt()), 4, "\x89PNG") == 0);
    }
    // The ball: split where its charts meet, the UVs flipped to glTF's (v down).
    const Json& prim = j["meshes"][0]["primitives"][0];
    REQUIRE(prim["attributes"].has("TEXCOORD_0"));
    CHECK(prim["attributes"].has("TANGENT")); // it has a normal map
    const auto uv = g->numbers(prim["attributes"]["TEXCOORD_0"].asInt());
    const auto idx = g->numbers(prim["indices"].asInt());
    const MeshData& m = d.scene.bodies()[0].mesh;
    REQUIRE(idx.size() == m.indices.size());
    double worst = 0;
    for (std::size_t c = 0; c < idx.size(); ++c) {
        const std::size_t v = static_cast<std::size_t>(idx[c]);
        worst = std::max({worst, std::fabs(uv[2 * v] - m.uvs[c].x), std::fabs(uv[2 * v + 1] - (1.0 - m.uvs[c].y))});
    }
    CHECK(worst < 1e-6);
    const Json& mat = j["materials"][prim["material"].asInt()];
    CHECK(mat["pbrMetallicRoughness"].has("baseColorTexture"));
    CHECK(mat.has("normalTexture"));
    bool glass = false, glow = false;
    for (std::size_t i = 0; i < j["materials"].size(); ++i) {
        const Json& e = j["materials"][i]["extensions"];
        glass = glass || (std::fabs(e["KHR_materials_transmission"]["transmissionFactor"].asNumber() - 1) < 1e-6 &&
                          std::fabs(e["KHR_materials_ior"]["ior"].asNumber() - 1.45) < 1e-6);
        glow = glow || std::fabs(e["KHR_materials_emissive_strength"]["emissiveStrength"].asNumber() - 5) < 1e-6;
    }
    CHECK(glass);
    CHECK(glow);
    CHECK(j["extensionsUsed"].size() == 3);
}

TEST_CASE("glTF export: an armature's skin, posed by its joints as in the model") {
    Document d;
    REQUIRE(run(d, "ARMATURE NEW Arm AT 0,-80,20 RADIUS 8").ok);
    REQUIRE(run(d, "ARMATURE Arm ADD AT 40,-80,20 RADIUS 6").ok);
    REQUIRE(run(d, "ARMATURE Arm ADD AT 70,-80,20 RADIUS 5").ok);
    REQUIRE(run(d, "ARMATURE Arm ADD TO 1 AT 0,-40,30 RADIUS 5").ok);
    REQUIRE(run(d, "ARMATURE Arm SKIN VOXEL 2 LINEAR").ok); // as glTF viewers pose it, so the two agree exactly
    REQUIRE(run(d, "ARMATURE Arm POSE 2 AXIS 0,0,1 ANGLE 45").ok);
    REQUIRE(run(d, "ARMATURE Arm POSE 1 AXIS 1,0,0 ANGLE 20").ok);
    REQUIRE(run(d, "MOVE 5,5,5 \"Arm skin\"").ok);
    const std::string path = tempPath("sicdm_export_skin.glb");
    ModelExportReport rep;
    std::string why;
    REQUIRE(exportModel(d.scene, path, &rep, &why));
    const auto g = readGlb(path);
    removeFile(path);
    REQUIRE(g.has_value());
    CHECK(rep.skins == 1);
    CHECK(rep.bodies == 1); // the armature itself is not a mesh
    const Json& j = g->json;
    const Json* node = nodeNamed(j, "Arm skin");
    REQUIRE(node && node->has("skin"));
    const Json& skin = j["skins"][(*node)["skin"].asInt()];
    REQUIRE(skin["joints"].size() == 4);
    // Each joint's place in the world: its own transform after its parents'.
    std::map<int, int> parent;
    for (std::size_t i = 0; i < j["nodes"].size(); ++i)
        for (std::size_t k = 0; k < j["nodes"][i]["children"].size(); ++k) parent[j["nodes"][i]["children"][k].asInt()] = static_cast<int>(i);
    auto local = [&](int n) {
        const Json& x = j["nodes"][n];
        std::array<float, 4> q{0, 0, 0, 1};
        if (x["rotation"].size() == 4)
            for (std::size_t c = 0; c < 4; ++c) q[c] = x["rotation"][c].asFloat();
        const Json& t = x["translation"];
        return translate(Vec3{t[0].asFloat(), t[1].asFloat(), t[2].asFloat()}) * quatMatrix(q);
    };
    auto world = [&](int n) {
        Mat4 m = local(n);
        for (auto it = parent.find(n); it != parent.end(); it = parent.find(it->second)) m = local(it->second) * m;
        return m;
    };
    const auto ibm = g->numbers(skin["inverseBindMatrices"].asInt());
    std::vector<Mat4> joint;
    for (std::size_t k = 0; k < skin["joints"].size(); ++k) {
        Mat4 inv;
        for (int c = 0; c < 4; ++c)
            for (int r = 0; r < 4; ++r) inv.m[static_cast<std::size_t>(c)][static_cast<std::size_t>(r)] = static_cast<float>(ibm[16 * k + static_cast<std::size_t>(4 * c + r)]);
        joint.push_back(world(skin["joints"][k].asInt()) * inv);
    }
    const Json& attr = j["meshes"][(*node)["mesh"].asInt()]["primitives"][0]["attributes"];
    const auto P = g->numbers(attr["POSITION"].asInt()), J = g->numbers(attr["JOINTS_0"].asInt()), W = g->numbers(attr["WEIGHTS_0"].asInt());
    const MeshData shown = worldMesh(d.scene.bodies().back());
    REQUIRE(P.size() == shown.vertices.size() * 3);
    double worst = 0, moved = 0, sumOff = 0;
    for (std::size_t v = 0; v < shown.vertices.size(); ++v) {
        const Vec3 rest{static_cast<float>(P[3 * v]), static_cast<float>(P[3 * v + 1]), static_cast<float>(P[3 * v + 2])};
        Vec3 at{};
        double sum = 0;
        for (std::size_t k = 0; k < 4; ++k) {
            const float w = static_cast<float>(W[4 * v + k]);
            at = at + joint[static_cast<std::size_t>(J[4 * v + k])].transformPoint(rest) * w;
            sum += w;
        }
        const Vec3 want = toMetresYUp(shown.vertices[v].position);
        worst = std::max(worst, static_cast<double>(length(at - want)));
        moved = std::max(moved, static_cast<double>(length(rest - want)));
        sumOff = std::max(sumOff, std::fabs(sum - 1));
    }
    CHECK(worst < 1e-6);  // under a thousandth of a millimetre
    CHECK(moved > 0.01);  // and the pose is not the rest pose
    CHECK(sumOff < 1e-5); // weights sum to one
}

TEST_CASE("FBX export: a well-formed binary file with geometry, quads, UVs, colours and materials") {
    Document d;
    REQUIRE(run(d, "BOX 40 30 10").ok);
    REQUIRE(run(d, "ASSEMBLY GROUP Base \"" + d.scene.bodies()[0].name + "\"").ok);
    REQUIRE(run(d, "SPHERE 15 @60,0,0").ok);
    const std::string ball = d.scene.bodies().back().name;
    REQUIRE(run(d, "RETOPO \"" + ball + "\" FACES 200 REPLACE").ok);
    REQUIRE(run(d, "UV \"" + ball + "\" UNWRAP").ok);
    const std::string png = tempPath("sicdm_export_fbxmap.png");
    REQUIRE(writePng8(png, 4, 4, 3, std::vector<std::uint8_t>(48, 90)));
    REQUIRE(run(d, "MATERIAL \"" + ball + "\" COLORMAP " + png).ok);
    REQUIRE(run(d, "BOX 10 10 10 @0,60,0").ok);
    REQUIRE(run(d, "PAINT \"" + d.scene.bodies().back().name + "\" FILL #20ff20 AT 0,60,10").ok);
    const std::string path = tempPath("sicdm_export_test.fbx");
    const auto r = run(d, "EXPORT " + path);
    removeFile(png);
    REQUIRE(r.ok);
    std::uint32_t version = 0;
    const auto f = readFbx(path, &version);
    removeFile(path);
    REQUIRE(f.has_value());
    CHECK(version == 7400);
    std::vector<std::string> names;
    for (const FbxRead& k : f->kids) names.push_back(k.name);
    CHECK((names == std::vector<std::string>{"FBXHeaderExtension", "GlobalSettings", "Documents", "References", "Definitions", "Objects", "Connections"}));
    const FbxRead* objects = f->kid("Objects");
    REQUIRE(objects);
    CHECK(objects->count("Geometry") == 3);
    CHECK(objects->count("Model") == 4); // three bodies and the Base group
    CHECK(objects->count("Material") == 3);
    CHECK(objects->count("Texture") == 1);
    CHECK(objects->count("Video") == 1);
    CHECK(f->kid("Connections")->count("C") == 4 + 3 + 3 + 2); // models up, geometry and material to models, map
    const MeshData& quadMesh = d.scene.bodies()[1].mesh;
    for (const FbxRead& g : objects->kids) {
        if (g.name != "Geometry") continue;
        REQUIRE(g.kid("Vertices") && g.kid("PolygonVertexIndex"));
        const auto& verts = g.kid("Vertices")->arrays[0];
        const auto& polys = g.kid("PolygonVertexIndex")->arrays[0];
        CHECK(verts.size() % 3 == 0);
        const std::size_t faces = static_cast<std::size_t>(std::count_if(polys.begin(), polys.end(), [](double v) { return v < 0; }));
        const std::string name = g.strings[0].substr(0, g.strings[0].find('\0'));
        if (name == ball) { // quads, with UVs per polygon corner
            CHECK(faces == quadMesh.quads.size());
            CHECK(polys.size() == 4 * faces);
            const FbxRead* uv = g.kid("LayerElementUV");
            REQUIRE(uv);
            CHECK(uv->kid("UV")->arrays[0].size() == 2 * polys.size());
        } else if (name == d.scene.bodies()[0].name) { // the block, in centimetres with +Y up
            CHECK(faces == 12);
            double maxX = 0, maxY = 0, maxZ = 0;
            for (std::size_t i = 0; i < verts.size(); i += 3) maxX = std::max(maxX, verts[i]), maxY = std::max(maxY, verts[i + 1]), maxZ = std::max(maxZ, verts[i + 2]);
            CHECK(std::fabs(maxX - 2.0) < 1e-6);
            CHECK(std::fabs(maxY - 1.0) < 1e-6);
            CHECK(std::fabs(maxZ - 1.5) < 1e-6);
        } else {
            CHECK(g.kid("LayerElementColor") != nullptr);
        }
        CHECK(g.kid("LayerElementNormal")->kid("Normals")->arrays[0].size() == 3 * polys.size());
    }
    const FbxRead* video = objects->kid("Video");
    CHECK(video->kid("Content")->rawBytes > 8);
}

TEST_CASE("model export: what it refuses, and over MCP") {
    Document d;
    std::string why;
    CHECK(!exportModel(d.scene, tempPath("sicdm_empty.glb"), nullptr, &why));
    CHECK(why.find("Nothing to export") != std::string::npos);
    REQUIRE(run(d, "BOX 10 10 10").ok);
    CHECK(!exportModel(d.scene, tempPath("sicdm_x.dae"), nullptr, &why));
    { // a file where its folder should be: it can't be written
        const std::string blocker = tempPath("sicdm_export_blocker");
        std::ofstream(blocker) << "x";
        CHECK(!run(d, "EXPORT " + blocker + "/x.glb").ok);
        removeFile(blocker);
    }
    Document m;
    REQUIRE(run(m, "BOX 10 10 10").ok);
    McpServer server(m, nullptr);
    const std::string path = tempPath("sicdm_mcp_export.glb");
    Json args = Json::object();
    args.set("path", path);
    const std::string msg = R"({"jsonrpc":"2.0","id":3,"method":"tools/call","params":{"name":"export_file","arguments":)" + args.dump() + "}}";
    const auto r = Json::parse(server.handle(msg));
    REQUIRE(r.has_value());
    CHECK(!(*r)["result"]["isError"].asBool());
    CHECK(readGlb(path).has_value());
    removeFile(path);
}

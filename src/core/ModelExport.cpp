// glTF 2.0 and FBX export: bodies, assemblies, UVs, vertex colours, PBR materials with their
// maps, and armature skins (glTF).

#include "core/ModelExport.hpp"

#include "core/Armature.hpp"
#include "core/Brep.hpp"
#include "core/FileIO.hpp"
#include "core/ImageWrite.hpp"
#include "core/Json.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <unordered_map>

namespace sicdm {

namespace {

std::string lowerOf(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

std::string extensionOf(const std::string& path) {
    const std::size_t dot = path.find_last_of('.');
    const std::size_t slash = path.find_last_of("/\\");
    if (dot == std::string::npos || (slash != std::string::npos && dot < slash)) return {};
    return lowerOf(path.substr(dot));
}

std::string baseName(const std::string& path) {
    const std::size_t slash = path.find_last_of("/\\");
    return slash == std::string::npos ? path : path.substr(slash + 1);
}

float srgbToLinear(float c) {
    c = std::clamp(c, 0.0f, 1.0f);
    return c <= 0.04045f ? c / 12.92f : std::pow((c + 0.055f) / 1.055f, 2.4f);
}

Vec3 linearColor(const Vec3& c) { return {srgbToLinear(c.x), srgbToLinear(c.y), srgbToLinear(c.z)}; }

// The scene (mm, +Z up) in metres with +Y up, as glTF has it (FBX: centimetres, the same axes).
Vec3 toYUp(const Vec3& p) { return {p.x, p.z, -p.y}; }
Vec3 toMetres(const Vec3& p) { return toYUp(p) * 0.001f; }
std::array<float, 4> quatToYUp(const std::array<float, 4>& q) { return {q[0], q[2], -q[1], q[3]}; }

// --- what goes out ----------------------------------------------------------------------

struct OutTexture {
    std::string name;
    std::string bytes; // PNG or JPEG
    std::string mime;
};

struct OutMaterial {
    std::string name;
    Vec3 color{1, 1, 1}; // sRGB
    MaterialProps props;
    bool vertexColors = false; // the colour comes from the vertices (the factor is white)
    int colorTexture = -1, normalTexture = -1;
};

struct OutJoint {
    std::string name;
    int parent = -1;                       // index in the skeleton
    Vec3 at{};                             // where it is at rest (metres, +Y up)
    std::array<float, 4> turn{0, 0, 0, 1}; // its pose (+Y up)
};

struct OutMesh {
    std::string name, assembly;
    // Vertices as the body has them (metres, +Y up) and its triangles; per corner UVs and
    // vertex colours (linear) when it has them.
    std::vector<Vec3> pos, nrm;
    std::vector<std::uint32_t> idx;
    std::vector<Vec2> uv;          // per triangle corner (v up), or empty
    std::vector<Vec3> color;       // per vertex, linear, or empty
    std::vector<std::array<std::uint32_t, 4>> quads; // FBX: the quads of a quad mesh (else empty)
    int material = 0;
    // Skin: per vertex up to four joints of `joints` and their weights (summing to one).
    std::vector<OutJoint> joints;
    std::vector<std::array<std::uint16_t, 4>> boneIndex;
    std::vector<std::array<float, 4>> boneWeight;
    bool skinned() const { return !joints.empty(); }
};

struct Prepared {
    std::vector<OutMesh> meshes;
    std::vector<OutMaterial> materials;
    std::vector<OutTexture> textures;
    std::vector<std::string> notes;
};

std::optional<std::string> readBytes(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return std::nullopt;
    std::ostringstream s;
    s << f.rdbuf();
    return s.str();
}

// A map image as PNG or JPEG bytes (other formats decoded and written as PNG).
std::optional<OutTexture> loadTextureFile(const std::string& path, std::string* why) {
    const auto bytes = readBytes(path);
    if (!bytes || bytes->empty()) {
        if (why) *why = "can't read " + path;
        return std::nullopt;
    }
    OutTexture t;
    t.name = baseName(path);
    if (bytes->size() > 8 && bytes->compare(0, 8, "\x89PNG\r\n\x1a\n") == 0) {
        t.bytes = *bytes, t.mime = "image/png";
    } else if (bytes->size() > 3 && static_cast<unsigned char>((*bytes)[0]) == 0xFF && static_cast<unsigned char>((*bytes)[1]) == 0xD8) {
        t.bytes = *bytes, t.mime = "image/jpeg";
    } else {
        ImageRef img;
        img.path = path;
        std::string err;
        if (!loadImagePixels(img, &err) || img.pixelWidth <= 0) {
            if (why) *why = "can't read the image " + path + (err.empty() ? std::string() : ": " + err);
            return std::nullopt;
        }
        std::vector<std::uint8_t> px(img.pixels.begin(), img.pixels.end());
        t.bytes = encodePngCompressed(img.pixelWidth, img.pixelHeight, 4, px);
        t.mime = "image/png";
        const std::size_t dot = t.name.find_last_of('.');
        t.name = (dot == std::string::npos ? t.name : t.name.substr(0, dot)) + ".png";
    }
    return t;
}

const Body* bodyNamed(const Scene& scene, const std::string& name) {
    for (const Body& b : scene.bodies())
        if (b.name == name) return &b;
    return nullptr;
}

// The skin's joints and weights, and its vertices at rest (mm, the armature's frame), when
// the armature is there and the skin still has the vertices it was made with.
bool skinOf(const Scene& scene, const Body& b, OutMesh& m, std::vector<Vec3>& rest, std::string& note) {
    const SkinData& s = *b.skin;
    const Body* ab = bodyNamed(scene, s.armature);
    Armature arm;
    if (!ab || ab->kind != "armature" || !armatureFromParams(ab->params, arm) || arm.balls.empty()) {
        note = b.name + ": its armature " + s.armature + " is gone, so it goes as it is posed, without a skin";
        return false;
    }
    if (s.rest.size() != b.mesh.vertices.size() || s.bone.size() != s.rest.size() || s.weight.size() != s.rest.size()) {
        note = b.name + " was remeshed since it was skinned, so it goes as it is, without a skin";
        return false;
    }
    if (meshKey(b.mesh) == s.key) {
        rest = s.rest;
    } else { // sculpted since it was posed: those edits taken back to the rest pose
        std::vector<Vec3> now(b.mesh.vertices.size());
        for (std::size_t i = 0; i < now.size(); ++i) now[i] = b.mesh.vertices[i].position;
        rest = unposeSkin(s, arm, now);
    }
    // A joint per ball, at rest where the ball is modelled, turned as it is posed.
    for (std::size_t i = 0; i < arm.balls.size(); ++i) {
        const ArmatureBall& ball = arm.balls[i];
        OutJoint j;
        j.name = s.armature + " ball " + std::to_string(i + 1);
        j.parent = ball.parent >= 0 && ball.parent < static_cast<int>(arm.balls.size()) ? ball.parent : -1;
        j.at = toMetres(ball.at + b.offset);
        j.turn = quatToYUp(ball.turn);
        m.joints.push_back(j);
    }
    // A link moves with the ball at its near end: its weight goes to that ball's joint
    // (two links from one ball are one joint).
    m.boneIndex.assign(rest.size(), {0, 0, 0, 0});
    m.boneWeight.assign(rest.size(), {0, 0, 0, 0});
    for (std::size_t v = 0; v < rest.size(); ++v) {
        std::array<int, 4> j{-1, -1, -1, -1};
        std::array<float, 4> w{0, 0, 0, 0};
        int n = 0;
        float total = 0;
        for (int k = 0; k < 4; ++k) {
            const int bone = s.bone[v][static_cast<std::size_t>(k)];
            const float wt = s.weight[v][static_cast<std::size_t>(k)];
            if (bone < 0 || bone >= static_cast<int>(arm.balls.size()) || !(wt > 0)) continue;
            const int p = arm.balls[static_cast<std::size_t>(bone)].parent;
            const int joint = p >= 0 ? p : bone;
            int slot = 0;
            while (slot < n && j[static_cast<std::size_t>(slot)] != joint) ++slot;
            if (slot == n) j[static_cast<std::size_t>(n++)] = joint;
            w[static_cast<std::size_t>(slot)] += wt;
            total += wt;
        }
        if (n == 0 || !(total > 0)) { // follows nothing: the root
            m.boneWeight[v] = {1, 0, 0, 0};
            continue;
        }
        for (int k = 0; k < n; ++k) {
            m.boneIndex[v][static_cast<std::size_t>(k)] = static_cast<std::uint16_t>(j[static_cast<std::size_t>(k)]);
            m.boneWeight[v][static_cast<std::size_t>(k)] = w[static_cast<std::size_t>(k)] / total;
        }
    }
    return true;
}

std::string materialKey(const OutMaterial& m) {
    char b[256];
    std::snprintf(b, sizeof b, "%.4f %.4f %.4f|%.4f %.4f %.4f %.4f %.4f|%d|", static_cast<double>(m.color.x), static_cast<double>(m.color.y),
                  static_cast<double>(m.color.z), static_cast<double>(m.props.metallic), static_cast<double>(m.props.roughness),
                  static_cast<double>(m.props.transmission), static_cast<double>(m.props.ior), static_cast<double>(m.props.emission),
                  m.vertexColors ? 1 : 0);
    return b + m.props.colorMap + "|" + m.props.normalMap;
}

// skins: armature skins with their joints and rest pose (else as they are posed).
Prepared prepare(const Scene& scene, bool skins) {
    Prepared out;
    std::map<std::string, int> materialOf, textureOf;
    std::size_t armatures = 0;
    auto texture = [&](const std::string& path) -> int {
        if (path.empty()) return -1;
        if (const auto it = textureOf.find(path); it != textureOf.end()) return it->second;
        std::string why;
        auto t = loadTextureFile(path, &why);
        if (!t) {
            out.notes.push_back("A map was left out: " + why);
            return textureOf[path] = -1;
        }
        out.textures.push_back(std::move(*t));
        return textureOf[path] = static_cast<int>(out.textures.size()) - 1;
    };
    for (const Body& b : scene.bodies()) {
        if (!b.visible || b.mesh.indices.empty()) continue;
        if (b.kind == "armature") {
            ++armatures;
            continue;
        }
        OutMesh m;
        m.name = b.name;
        m.assembly = b.assembly;
        const MeshData& mesh = b.mesh;
        // Where the vertices are: as placed, or (a skin) at rest with its joints.
        std::vector<Vec3> at(mesh.vertices.size());
        for (std::size_t i = 0; i < at.size(); ++i) at[i] = mesh.vertices[i].position;
        std::vector<Vec3> normals(at.size());
        for (std::size_t i = 0; i < at.size(); ++i) normals[i] = mesh.vertices[i].normal;
        if (b.skin && !skins) {
            out.notes.push_back(b.name + " goes as it is posed (glTF carries its skin and bones)");
        } else if (b.skin) {
            std::vector<Vec3> rest;
            std::string note;
            if (b.skin && !b.skin->options.linear)
                out.notes.push_back(b.name + " is posed with dual quaternions here; glTF viewers blend linearly, so a joint twisted far may thin "
                                             "there (ARMATURE name SKINNING LINEAR poses it the same way here)");
            if (skinOf(scene, b, m, rest, note)) {
                at = rest;
                MeshData r = mesh; // the rest pose's own normals
                for (std::size_t i = 0; i < at.size(); ++i) r.vertices[i].position = at[i];
                smoothVertexNormals(r);
                for (std::size_t i = 0; i < at.size(); ++i) normals[i] = r.vertices[i].normal;
            } else {
                out.notes.push_back(note);
            }
        }
        // Normals of unit length (a face's where a vertex has none).
        std::vector<Vec3> faceSum(at.size());
        for (std::size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
            const std::uint32_t a = mesh.indices[t], c = mesh.indices[t + 1], d = mesh.indices[t + 2];
            if (a >= at.size() || c >= at.size() || d >= at.size()) continue;
            const Vec3 n = cross(at[c] - at[a], at[d] - at[a]);
            faceSum[a] = faceSum[a] + n, faceSum[c] = faceSum[c] + n, faceSum[d] = faceSum[d] + n;
        }
        for (std::size_t i = 0; i < at.size(); ++i) {
            Vec3 n = normals[i];
            if (!(length(n) > 1e-6f)) n = faceSum[i];
            n = length(n) > 1e-12f ? normalize(n) : Vec3{0, 0, 1};
            m.nrm.push_back(toYUp(n));
            m.pos.push_back(toMetres(at[i] + b.offset));
        }
        for (std::size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
            const std::uint32_t a = mesh.indices[t], c = mesh.indices[t + 1], d = mesh.indices[t + 2];
            if (a >= at.size() || c >= at.size() || d >= at.size()) continue;
            m.idx.insert(m.idx.end(), {a, c, d});
        }
        if (m.idx.size() != mesh.indices.size()) {
            out.notes.push_back(b.name + ": triangles with a missing corner were left out");
        } else {
            if (mesh.hasUvs()) m.uv = mesh.uvs;
            if (mesh.hasQuads() && mesh.quads.size() * 6 == mesh.indices.size()) m.quads = mesh.quads;
        }
        const bool paint = std::any_of(mesh.vertices.begin(), mesh.vertices.end(), [](const Vertex& v) { return painted(v.color); });
        if (paint)
            for (const Vertex& v : mesh.vertices) m.color.push_back(linearColor(painted(v.color) ? unpackColor(v.color) : b.color));
        // Its material (shared by bodies that look the same).
        OutMaterial mat;
        mat.name = !b.material.preset.empty() ? b.material.preset : b.name;
        mat.color = b.color;
        mat.props = b.material;
        mat.vertexColors = paint;
        if (!m.uv.empty()) {
            mat.colorTexture = texture(b.material.colorMap);
            mat.normalTexture = texture(b.material.normalMap);
        } else if (!b.material.colorMap.empty() || !b.material.normalMap.empty()) {
            out.notes.push_back(b.name + " has maps but no texture coordinates (UV UNWRAP), so its maps were left out");
            mat.props.colorMap.clear(), mat.props.normalMap.clear();
        }
        const std::string key = materialKey(mat);
        auto it = materialOf.find(key);
        if (it == materialOf.end()) {
            out.materials.push_back(mat);
            it = materialOf.emplace(key, static_cast<int>(out.materials.size()) - 1).first;
        }
        m.material = it->second;
        out.meshes.push_back(std::move(m));
    }
    if (armatures) out.notes.push_back(std::to_string(armatures) + (armatures == 1 ? " armature" : " armatures") + " not written as meshes (their skins carry them)");
    return out;
}

// --- glTF ---------------------------------------------------------------------------------

class GltfWriter {
public:
    std::string bin;
    Json views = Json::array(), accessors = Json::array();

    int view(const void* data, std::size_t bytes, int target) {
        while (bin.size() % 4) bin.push_back('\0');
        Json v = Json::object();
        v.set("buffer", 0).set("byteOffset", bin.size()).set("byteLength", bytes);
        if (target) v.set("target", target);
        bin.append(static_cast<const char*>(data), bytes);
        views.push(v);
        return static_cast<int>(views.size()) - 1;
    }

    int floats(const std::vector<float>& f, int comps, const char* type, int target, bool bounds) {
        const int v = view(f.data(), f.size() * sizeof(float), target);
        Json a = Json::object();
        a.set("bufferView", v).set("componentType", 5126).set("count", f.size() / static_cast<std::size_t>(comps)).set("type", type);
        if (bounds && !f.empty()) {
            Json lo = Json::array(), hi = Json::array();
            for (int c = 0; c < comps; ++c) {
                float l = f[static_cast<std::size_t>(c)], h = l;
                for (std::size_t i = static_cast<std::size_t>(c); i < f.size(); i += static_cast<std::size_t>(comps)) l = std::min(l, f[i]), h = std::max(h, f[i]);
                lo.push(static_cast<double>(l)), hi.push(static_cast<double>(h));
            }
            a.set("min", lo).set("max", hi);
        }
        accessors.push(a);
        return static_cast<int>(accessors.size()) - 1;
    }

    int indices(const std::vector<std::uint32_t>& idx, std::size_t vertexCount) {
        Json a = Json::object();
        int v;
        if (vertexCount <= 65535) {
            std::vector<std::uint16_t> s(idx.begin(), idx.end());
            v = view(s.data(), s.size() * 2, 34963);
            a.set("componentType", 5123);
        } else {
            v = view(idx.data(), idx.size() * 4, 34963);
            a.set("componentType", 5125);
        }
        a.set("bufferView", v).set("count", idx.size()).set("type", "SCALAR");
        accessors.push(a);
        return static_cast<int>(accessors.size()) - 1;
    }

    int shorts4(const std::vector<std::uint16_t>& s) {
        const int v = view(s.data(), s.size() * 2, 34962);
        Json a = Json::object();
        a.set("bufferView", v).set("componentType", 5123).set("count", s.size() / 4).set("type", "VEC4");
        accessors.push(a);
        return static_cast<int>(accessors.size()) - 1;
    }
};

Json vec(const Vec3& v) {
    Json j = Json::array();
    j.push(static_cast<double>(v.x)), j.push(static_cast<double>(v.y)), j.push(static_cast<double>(v.z));
    return j;
}

std::string base64Of(const std::string& s) {
    static const char* const k = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string o;
    o.reserve((s.size() + 2) / 3 * 4);
    std::size_t i = 0;
    for (; i + 2 < s.size(); i += 3) {
        const unsigned v = static_cast<unsigned char>(s[i]) << 16 | static_cast<unsigned char>(s[i + 1]) << 8 | static_cast<unsigned char>(s[i + 2]);
        o += k[v >> 18], o += k[(v >> 12) & 63], o += k[(v >> 6) & 63], o += k[v & 63];
    }
    if (i < s.size()) {
        const unsigned v = static_cast<unsigned char>(s[i]) << 16 | (i + 1 < s.size() ? static_cast<unsigned char>(s[i + 1]) << 8 : 0u);
        o += k[v >> 18], o += k[(v >> 12) & 63];
        o += i + 1 < s.size() ? k[(v >> 6) & 63] : '=';
        o += '=';
    }
    return o;
}

bool writeGltf(const Prepared& p, const std::string& path, bool binary, ModelExportReport& rep, std::string* why) {
    GltfWriter w;
    std::vector<Json> nodes;
    std::map<int, std::vector<int>> kidsOf; // node -> its children
    std::vector<int> roots;
    std::map<std::string, int> groupNode; // assembly path -> its node
    std::function<int(const std::string&)> group = [&](const std::string& assembly) -> int {
        if (assembly.empty()) return -1;
        if (const auto it = groupNode.find(assembly); it != groupNode.end()) return it->second;
        const std::size_t slash = assembly.find_last_of('/');
        const int parent = group(slash == std::string::npos ? std::string() : assembly.substr(0, slash));
        nodes.push_back(Json::object().set("name", slash == std::string::npos ? assembly : assembly.substr(slash + 1)));
        const int n = static_cast<int>(nodes.size()) - 1;
        groupNode[assembly] = n;
        (parent >= 0 ? kidsOf[parent] : roots).push_back(n);
        return n;
    };
    Json meshes = Json::array(), skins = Json::array();
    for (const OutMesh& m : p.meshes) {
        // Vertices split where a corner's texture coordinates differ.
        std::vector<std::uint32_t> source; // new vertex -> the body's
        std::vector<Vec2> uv;              // new vertex's (glTF: v down)
        std::vector<std::uint32_t> idx(m.idx.size());
        if (!m.uv.empty()) {
            std::unordered_map<std::uint32_t, std::vector<std::uint32_t>> copiesOf; // body vertex -> its copies
            for (std::size_t c = 0; c < m.idx.size(); ++c) {
                const std::uint32_t v = m.idx[c];
                const Vec2 t{m.uv[c].x, 1.0f - m.uv[c].y};
                auto& copies = copiesOf[v];
                std::uint32_t found = 0xFFFFFFFFu;
                for (std::uint32_t k : copies)
                    if (uv[k].x == t.x && uv[k].y == t.y) found = k;
                if (found == 0xFFFFFFFFu) {
                    found = static_cast<std::uint32_t>(source.size());
                    source.push_back(v), uv.push_back(t), copies.push_back(found);
                }
                idx[c] = found;
            }
        } else {
            source.resize(m.pos.size());
            for (std::size_t i = 0; i < source.size(); ++i) source[i] = static_cast<std::uint32_t>(i);
            idx = m.idx;
        }
        std::vector<float> pos, nrm, tex, col, wts;
        std::vector<std::uint16_t> jts;
        for (std::uint32_t s : source) {
            pos.insert(pos.end(), {m.pos[s].x, m.pos[s].y, m.pos[s].z});
            nrm.insert(nrm.end(), {m.nrm[s].x, m.nrm[s].y, m.nrm[s].z});
            if (!m.color.empty()) col.insert(col.end(), {m.color[s].x, m.color[s].y, m.color[s].z});
            if (m.skinned()) {
                jts.insert(jts.end(), m.boneIndex[s].begin(), m.boneIndex[s].end());
                wts.insert(wts.end(), m.boneWeight[s].begin(), m.boneWeight[s].end());
            }
        }
        for (const Vec2& t : uv) tex.insert(tex.end(), {t.x, t.y});
        // A normal map needs the tangent frame it was baked in: along u, with w the side v
        // runs to (as BAKE and RENDER take it, from each face's texture coordinates).
        std::vector<float> tan;
        if (!uv.empty() && p.materials[static_cast<std::size_t>(m.material)].normalTexture >= 0) {
            std::vector<Vec3> tu(source.size()), tv(source.size());
            for (std::size_t c = 0; c + 2 < idx.size(); c += 3) {
                const Vec3 p0 = m.pos[source[idx[c]]], p1 = m.pos[source[idx[c + 1]]], p2 = m.pos[source[idx[c + 2]]];
                const Vec2 u0 = m.uv[c], u1 = m.uv[c + 1], u2 = m.uv[c + 2];
                const Vec3 e1 = p1 - p0, e2 = p2 - p0;
                const float du1 = u1.x - u0.x, dv1 = u1.y - u0.y, du2 = u2.x - u0.x, dv2 = u2.y - u0.y;
                const float r = du1 * dv2 - du2 * dv1;
                if (std::fabs(r) < 1e-20f) continue;
                const Vec3 a = (e1 * dv2 - e2 * dv1) / r, b = (e2 * du1 - e1 * du2) / r;
                for (std::size_t k = 0; k < 3; ++k) tu[idx[c + k]] = tu[idx[c + k]] + a, tv[idx[c + k]] = tv[idx[c + k]] + b;
            }
            for (std::size_t i = 0; i < source.size(); ++i) {
                const Vec3 n = m.nrm[source[i]];
                Vec3 t = tu[i] - n * dot(n, tu[i]);
                if (!(length(t) > 1e-12f)) t = cross(n, std::fabs(n.x) < 0.9f ? Vec3{1, 0, 0} : Vec3{0, 1, 0});
                t = normalize(t);
                const float hand = dot(cross(n, t), tv[i]) < 0 ? -1.0f : 1.0f;
                tan.insert(tan.end(), {t.x, t.y, t.z, hand});
            }
        }
        Json attr = Json::object();
        attr.set("POSITION", w.floats(pos, 3, "VEC3", 34962, true));
        attr.set("NORMAL", w.floats(nrm, 3, "VEC3", 34962, false));
        if (!tex.empty()) attr.set("TEXCOORD_0", w.floats(tex, 2, "VEC2", 34962, false));
        if (!tan.empty()) attr.set("TANGENT", w.floats(tan, 4, "VEC4", 34962, false));
        if (!col.empty()) attr.set("COLOR_0", w.floats(col, 3, "VEC3", 34962, false));
        if (m.skinned()) {
            attr.set("JOINTS_0", w.shorts4(jts));
            attr.set("WEIGHTS_0", w.floats(wts, 4, "VEC4", 34962, false));
        }
        Json prim = Json::object();
        prim.set("attributes", attr).set("indices", w.indices(idx, source.size())).set("material", m.material).set("mode", 4);
        meshes.push(Json::object().set("name", m.name).set("primitives", Json(Json::Array{prim})));
        Json node = Json::object();
        node.set("name", m.name).set("mesh", static_cast<int>(meshes.size()) - 1);
        if (m.skinned()) {
            // A node per joint under its parent, turned as posed; the mesh node at the top
            // (a skinned mesh is placed by its joints, not its node).
            const int first = static_cast<int>(nodes.size());
            Json jointList = Json::array();
            std::vector<float> ibm;
            int root = -1;
            for (std::size_t j = 0; j < m.joints.size(); ++j) {
                const OutJoint& J = m.joints[j];
                const Vec3 local = J.parent >= 0 ? J.at - m.joints[static_cast<std::size_t>(J.parent)].at : J.at;
                Json jn = Json::object();
                jn.set("name", J.name).set("translation", vec(local));
                if (J.turn[0] != 0 || J.turn[1] != 0 || J.turn[2] != 0) {
                    Json q = Json::array();
                    for (float c : J.turn) q.push(static_cast<double>(c));
                    jn.set("rotation", q);
                }
                nodes.push_back(jn);
                const int n = first + static_cast<int>(j);
                jointList.push(n);
                if (J.parent >= 0) {
                    kidsOf[first + J.parent].push_back(n);
                } else {
                    roots.push_back(n);
                    if (root < 0) root = n;
                }
                // Inverse bind matrix: back from where the joint is at rest (column major).
                const float m16[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, -J.at.x, -J.at.y, -J.at.z, 1};
                ibm.insert(ibm.end(), m16, m16 + 16);
            }
            Json skin = Json::object();
            skin.set("name", m.name).set("joints", jointList).set("inverseBindMatrices", w.floats(ibm, 16, "MAT4", 0, false));
            if (root >= 0) skin.set("skeleton", root);
            skins.push(skin);
            node.set("skin", static_cast<int>(skins.size()) - 1);
            nodes.push_back(node);
            roots.push_back(static_cast<int>(nodes.size()) - 1);
            ++rep.skins;
        } else {
            const int g = group(m.assembly);
            nodes.push_back(node);
            const int n = static_cast<int>(nodes.size()) - 1;
            (g >= 0 ? kidsOf[g] : roots).push_back(n);
        }
        rep.triangles += m.idx.size() / 3;
        ++rep.bodies;
    }
    for (const auto& [n, kids] : kidsOf) {
        Json k = Json::array();
        for (int c : kids) k.push(c);
        nodes[static_cast<std::size_t>(n)].set("children", k);
    }
    // Materials, their maps and images.
    std::set<std::string> extensions;
    Json materials = Json::array(), textures = Json::array(), images = Json::array();
    for (const OutTexture& t : p.textures) {
        const int v = w.view(t.bytes.data(), t.bytes.size(), 0);
        images.push(Json::object().set("name", t.name).set("mimeType", t.mime).set("bufferView", v));
        textures.push(Json::object().set("sampler", 0).set("source", static_cast<int>(images.size()) - 1));
    }
    for (const OutMaterial& mt : p.materials) {
        const MaterialProps& q = mt.props;
        const Vec3 c = mt.vertexColors ? Vec3{1, 1, 1} : linearColor(mt.color);
        Json pbr = Json::object();
        pbr.set("baseColorFactor", Json(Json::Array{c.x, c.y, c.z, 1.0f}));
        pbr.set("metallicFactor", std::clamp(q.metallic, 0.0f, 1.0f)).set("roughnessFactor", std::clamp(q.roughness, 0.0f, 1.0f));
        if (mt.colorTexture >= 0) pbr.set("baseColorTexture", Json::object().set("index", mt.colorTexture));
        Json jm = Json::object();
        jm.set("name", mt.name).set("pbrMetallicRoughness", pbr);
        if (mt.normalTexture >= 0) jm.set("normalTexture", Json::object().set("index", mt.normalTexture));
        Json ext = Json::object();
        if (q.emission > 0) {
            const Vec3 e = linearColor(mt.color);
            jm.set("emissiveFactor", vec(e));
            if (q.emission > 1) {
                ext.set("KHR_materials_emissive_strength", Json::object().set("emissiveStrength", q.emission));
                extensions.insert("KHR_materials_emissive_strength");
            }
        }
        if (q.transmission > 0) {
            ext.set("KHR_materials_transmission", Json::object().set("transmissionFactor", std::clamp(q.transmission, 0.0f, 1.0f)));
            extensions.insert("KHR_materials_transmission");
        }
        if (q.ior != 1.5f) {
            ext.set("KHR_materials_ior", Json::object().set("ior", std::clamp(q.ior, 1.0f, 4.0f)));
            extensions.insert("KHR_materials_ior");
        }
        if (!ext.members().empty()) jm.set("extensions", ext);
        materials.push(jm);
    }
    rep.materials = p.materials.size();
    rep.textures = p.textures.size();

    Json g = Json::object();
    g.set("asset", Json::object().set("version", "2.0").set("generator", "SI CDM"));
    Json rootList = Json::array();
    for (int r : roots) rootList.push(r);
    g.set("scene", 0).set("scenes", Json(Json::Array{Json::object().set("name", "Scene").set("nodes", rootList)}));
    g.set("nodes", Json(Json::Array(nodes.begin(), nodes.end()))).set("meshes", meshes).set("materials", materials);
    if (skins.size()) g.set("skins", skins);
    if (images.size()) {
        g.set("images", images).set("textures", textures);
        g.set("samplers", Json(Json::Array{Json::object().set("magFilter", 9729).set("minFilter", 9987).set("wrapS", 10497).set("wrapT", 10497)}));
    }
    if (!extensions.empty()) {
        Json used = Json::array();
        for (const std::string& e : extensions) used.push(e);
        g.set("extensionsUsed", used);
    }
    while (w.bin.size() % 4) w.bin.push_back('\0');
    Json buffer = Json::object();
    buffer.set("byteLength", w.bin.size());
    if (!binary) buffer.set("uri", "data:application/octet-stream;base64," + base64Of(w.bin));
    g.set("buffers", Json(Json::Array{buffer})).set("bufferViews", w.views).set("accessors", w.accessors);

    std::string json = g.dump();
    if (!binary) return writeFile(path, json, why);
    while (json.size() % 4) json.push_back(' ');
    auto u32 = [](std::string& s, std::uint32_t v) {
        char b[4];
        std::memcpy(b, &v, 4);
        s.append(b, 4);
    };
    std::string glb = "glTF";
    u32(glb, 2);
    u32(glb, static_cast<std::uint32_t>(12 + 8 + json.size() + 8 + w.bin.size()));
    u32(glb, static_cast<std::uint32_t>(json.size()));
    glb += "JSON";
    glb += json;
    u32(glb, static_cast<std::uint32_t>(w.bin.size()));
    glb.append("BIN\0", 4);
    glb += w.bin;
    return writeFile(path, glb, why);
}

// --- FBX (binary 7.4) ---------------------------------------------------------------------

// A node of an FBX file: its name, its properties (each a type code and its bytes) and the
// nodes inside it.
struct FbxNode {
    std::string name;
    std::string props; // the properties, encoded
    std::uint32_t count = 0;
    std::vector<FbxNode> kids;

    explicit FbxNode(std::string n) : name(std::move(n)) {}

    template <class T> void raw(char code, const T& v) {
        props += code;
        props.append(reinterpret_cast<const char*>(&v), sizeof(T));
        ++count;
    }
    FbxNode& I(std::int32_t v) { return raw('I', v), *this; }
    FbxNode& L(std::int64_t v) { return raw('L', v), *this; }
    FbxNode& D(double v) { return raw('D', v), *this; }
    FbxNode& C(bool v) { return raw('C', static_cast<std::uint8_t>(v ? 1 : 0)), *this; }
    FbxNode& S(const std::string& s, char code = 'S') {
        props += code;
        const std::uint32_t n = static_cast<std::uint32_t>(s.size());
        props.append(reinterpret_cast<const char*>(&n), 4);
        props += s;
        ++count;
        return *this;
    }
    FbxNode& R(const std::string& bytes) { return S(bytes, 'R'); }
    template <class T> FbxNode& array(char code, const std::vector<T>& v) {
        props += code;
        const std::uint32_t n = static_cast<std::uint32_t>(v.size()), enc = 0, bytes = static_cast<std::uint32_t>(v.size() * sizeof(T));
        props.append(reinterpret_cast<const char*>(&n), 4);
        props.append(reinterpret_cast<const char*>(&enc), 4);
        props.append(reinterpret_cast<const char*>(&bytes), 4);
        props.append(reinterpret_cast<const char*>(v.data()), bytes);
        ++count;
        return *this;
    }
    FbxNode& add(const std::string& n) { return kids.emplace_back(n); }
    // A Properties70 entry: P: name, type, label, flags, values...
    FbxNode& P(const std::string& n, const std::string& type, const std::string& label, const std::string& flags) {
        return add("P").S(n).S(type).S(label).S(flags);
    }
};

void writeFbxNode(const FbxNode& n, std::string& out) {
    const std::size_t start = out.size();
    out.append(12, '\0'); // end offset, property count, property bytes
    out += static_cast<char>(n.name.size());
    out += n.name;
    out += n.props;
    for (const FbxNode& k : n.kids) writeFbxNode(k, out);
    if (!n.kids.empty() || n.count == 0) out.append(13, '\0');
    const std::uint32_t end = static_cast<std::uint32_t>(out.size()), count = n.count, bytes = static_cast<std::uint32_t>(n.props.size());
    std::memcpy(&out[start], &end, 4);
    std::memcpy(&out[start + 4], &count, 4);
    std::memcpy(&out[start + 8], &bytes, 4);
}

std::string fbxName(const std::string& name, const char* cls) { return name + std::string("\0\1", 2) + cls; }

bool writeFbx(const Prepared& p, const std::string& path, ModelExportReport& rep, std::string* why) {
    FbxNode header("FBXHeaderExtension"), globals("GlobalSettings"), documents("Documents"), references("References"), defs("Definitions"),
        objects("Objects"), connections("Connections");
    header.add("FBXHeaderVersion").I(1003);
    header.add("FBXVersion").I(7400);
    header.add("Creator").S("SI CDM");
    globals.add("Version").I(1000);
    {
        FbxNode& p70 = globals.add("Properties70");
        p70.P("UpAxis", "int", "Integer", "").I(1);
        p70.P("UpAxisSign", "int", "Integer", "").I(1);
        p70.P("FrontAxis", "int", "Integer", "").I(2);
        p70.P("FrontAxisSign", "int", "Integer", "").I(1);
        p70.P("CoordAxis", "int", "Integer", "").I(0);
        p70.P("CoordAxisSign", "int", "Integer", "").I(1);
        p70.P("UnitScaleFactor", "double", "Number", "").D(1.0); // centimetres
        p70.P("OriginalUnitScaleFactor", "double", "Number", "").D(0.1);
    }
    documents.add("Count").I(0);
    std::int64_t nextId = 1000000;
    auto link = [&](const char* kind, std::int64_t child, std::int64_t parent, const std::string& prop = {}) {
        FbxNode& c = connections.add("C");
        c.S(kind).L(child).L(parent);
        if (!prop.empty()) c.S(prop);
    };
    std::size_t models = 0, geometries = 0;
    // Assembly groups as empty (null) models.
    std::map<std::string, std::int64_t> groupId;
    std::function<std::int64_t(const std::string&)> group = [&](const std::string& assembly) -> std::int64_t {
        if (assembly.empty()) return 0;
        if (const auto it = groupId.find(assembly); it != groupId.end()) return it->second;
        const std::size_t slash = assembly.find_last_of('/');
        const std::int64_t parent = group(slash == std::string::npos ? std::string() : assembly.substr(0, slash));
        const std::int64_t id = nextId++;
        FbxNode& m = objects.add("Model");
        m.L(id).S(fbxName(slash == std::string::npos ? assembly : assembly.substr(slash + 1), "Model")).S("Null");
        m.add("Version").I(232);
        m.add("Properties70");
        m.add("Culling").S("CullingOff");
        link("OO", id, parent);
        ++models;
        return groupId[assembly] = id;
    };
    // Materials and their maps.
    std::vector<std::int64_t> materialId, textureVideo;
    for (const OutTexture& t : p.textures) {
        const std::int64_t id = nextId++;
        FbxNode& v = objects.add("Video");
        v.L(id).S(fbxName(t.name, "Video")).S("Clip");
        v.add("Type").S("Clip");
        v.add("Properties70").P("Path", "KString", "XRefUrl", "").S(t.name);
        v.add("UseMipMap").I(0);
        v.add("Filename").S(t.name);
        v.add("RelativeFilename").S(t.name);
        v.add("Content").R(t.bytes);
        textureVideo.push_back(id);
    }
    auto texture = [&](int index, std::int64_t material, const char* slot) {
        if (index < 0) return;
        const OutTexture& t = p.textures[static_cast<std::size_t>(index)];
        const std::int64_t id = nextId++;
        FbxNode& x = objects.add("Texture");
        x.L(id).S(fbxName(t.name, "Texture")).S("");
        x.add("Type").S("TextureVideoClip");
        x.add("Version").I(202);
        x.add("TextureName").S(fbxName(t.name, "Texture"));
        x.add("Properties70");
        x.add("Media").S(fbxName(t.name, "Video"));
        x.add("FileName").S(t.name);
        x.add("RelativeFilename").S(t.name);
        x.add("ModelUVTranslation").D(0).D(0);
        x.add("ModelUVScaling").D(1).D(1);
        x.add("Texture_Alpha_Source").S("None");
        x.add("Cropping").I(0).I(0).I(0).I(0);
        link("OO", textureVideo[static_cast<std::size_t>(index)], id);
        link("OP", id, material, slot);
    };
    for (const OutMaterial& mt : p.materials) {
        const std::int64_t id = nextId++;
        const MaterialProps& q = mt.props;
        const Vec3 c = mt.vertexColors ? Vec3{1, 1, 1} : mt.color;
        FbxNode& m = objects.add("Material");
        m.L(id).S(fbxName(mt.name, "Material")).S("");
        m.add("Version").I(102);
        m.add("ShadingModel").S("phong");
        m.add("MultiLayer").I(0);
        FbxNode& p70 = m.add("Properties70");
        p70.P("DiffuseColor", "Color", "", "A").D(c.x).D(c.y).D(c.z);
        p70.P("DiffuseFactor", "Number", "", "A").D(1);
        const float shine = (1 - std::clamp(q.roughness, 0.0f, 1.0f));
        p70.P("SpecularColor", "Color", "", "A").D(0.5 + 0.5 * q.metallic * c.x).D(0.5 + 0.5 * q.metallic * c.y).D(0.5 + 0.5 * q.metallic * c.z);
        p70.P("SpecularFactor", "Number", "", "A").D(shine);
        p70.P("ShininessExponent", "Number", "", "A").D(2 + 98 * shine * shine);
        p70.P("ReflectionFactor", "Number", "", "A").D(std::clamp(q.metallic, 0.0f, 1.0f));
        if (q.emission > 0) {
            p70.P("EmissiveColor", "Color", "", "A").D(mt.color.x).D(mt.color.y).D(mt.color.z);
            p70.P("EmissiveFactor", "Number", "", "A").D(q.emission);
        }
        if (q.transmission > 0) {
            p70.P("TransparentColor", "Color", "", "A").D(1).D(1).D(1);
            p70.P("TransparencyFactor", "Number", "", "A").D(std::clamp(q.transmission, 0.0f, 1.0f));
            p70.P("Opacity", "Number", "", "A").D(1 - std::clamp(q.transmission, 0.0f, 1.0f));
        }
        materialId.push_back(id);
        texture(mt.colorTexture, id, "DiffuseColor");
        texture(mt.normalTexture, id, "NormalMap");
    }
    for (const OutMesh& m : p.meshes) {
        // Centimetres: the metres times 100.
        std::vector<double> verts;
        for (const Vec3& v : m.pos) verts.insert(verts.end(), {v.x * 100.0, v.y * 100.0, v.z * 100.0});
        // Polygons: quads where the mesh has them, else triangles; each one's last corner as -(i + 1).
        std::vector<std::int32_t> polys;
        std::vector<std::size_t> corners; // each polygon corner's triangle corner (for UVs)
        if (!m.quads.empty()) {
            for (std::size_t qi = 0; qi < m.quads.size(); ++qi)
                for (int k = 0; k < 4; ++k) {
                    const std::uint32_t v = m.quads[qi][static_cast<std::size_t>(k)];
                    std::size_t c = qi * 6;
                    while (c + 1 < qi * 6 + 6 && m.idx[c] != v) ++c;
                    corners.push_back(c);
                    polys.push_back(k == 3 ? -static_cast<std::int32_t>(v) - 1 : static_cast<std::int32_t>(v));
                }
        } else {
            for (std::size_t c = 0; c < m.idx.size(); ++c) {
                corners.push_back(c);
                polys.push_back(c % 3 == 2 ? -static_cast<std::int32_t>(m.idx[c]) - 1 : static_cast<std::int32_t>(m.idx[c]));
            }
        }
        auto vertexAt = [&](std::size_t k) { return static_cast<std::size_t>(polys[k] < 0 ? -polys[k] - 1 : polys[k]); };
        const std::int64_t gid = nextId++, mid = nextId++;
        FbxNode& g = objects.add("Geometry");
        g.L(gid).S(fbxName(m.name, "Geometry")).S("Mesh");
        g.add("Properties70");
        g.add("GeometryVersion").I(124);
        g.add("Vertices").array('d', verts);
        g.add("PolygonVertexIndex").array('i', polys);
        {
            std::vector<double> n;
            for (std::size_t k = 0; k < polys.size(); ++k) {
                const Vec3& v = m.nrm[vertexAt(k)];
                n.insert(n.end(), {v.x, v.y, v.z});
            }
            FbxNode& e = g.add("LayerElementNormal").I(0);
            e.add("Version").I(101);
            e.add("Name").S("");
            e.add("MappingInformationType").S("ByPolygonVertex");
            e.add("ReferenceInformationType").S("Direct");
            e.add("Normals").array('d', n);
        }
        if (!m.uv.empty()) {
            std::vector<double> uv;
            std::vector<std::int32_t> uvIndex;
            for (std::size_t k = 0; k < polys.size(); ++k) {
                const Vec2& t = m.uv[corners[k]];
                uv.insert(uv.end(), {t.x, t.y});
                uvIndex.push_back(static_cast<std::int32_t>(k));
            }
            FbxNode& e = g.add("LayerElementUV").I(0);
            e.add("Version").I(101);
            e.add("Name").S("UVMap");
            e.add("MappingInformationType").S("ByPolygonVertex");
            e.add("ReferenceInformationType").S("IndexToDirect");
            e.add("UV").array('d', uv);
            e.add("UVIndex").array('i', uvIndex);
        }
        if (!m.color.empty()) {
            std::vector<double> col;
            std::vector<std::int32_t> colIndex;
            for (const Vec3& c : m.color) col.insert(col.end(), {c.x, c.y, c.z, 1.0});
            for (std::size_t k = 0; k < polys.size(); ++k) colIndex.push_back(static_cast<std::int32_t>(vertexAt(k)));
            FbxNode& e = g.add("LayerElementColor").I(0);
            e.add("Version").I(101);
            e.add("Name").S("Col");
            e.add("MappingInformationType").S("ByPolygonVertex");
            e.add("ReferenceInformationType").S("IndexToDirect");
            e.add("Colors").array('d', col);
            e.add("ColorIndex").array('i', colIndex);
        }
        {
            FbxNode& e = g.add("LayerElementMaterial").I(0);
            e.add("Version").I(101);
            e.add("Name").S("");
            e.add("MappingInformationType").S("AllSame");
            e.add("ReferenceInformationType").S("IndexToDirect");
            e.add("Materials").array('i', std::vector<std::int32_t>{0});
        }
        {
            FbxNode& layer = g.add("Layer").I(0);
            layer.add("Version").I(100);
            auto element = [&](const char* type) {
                FbxNode& e = layer.add("LayerElement");
                e.add("Type").S(type);
                e.add("TypedIndex").I(0);
            };
            element("LayerElementNormal");
            if (!m.uv.empty()) element("LayerElementUV");
            if (!m.color.empty()) element("LayerElementColor");
            element("LayerElementMaterial");
        }
        FbxNode& model = objects.add("Model");
        model.L(mid).S(fbxName(m.name, "Model")).S("Mesh");
        model.add("Version").I(232);
        model.add("Properties70").P("DefaultAttributeIndex", "int", "Integer", "").I(0);
        model.add("Shading").C(true);
        model.add("Culling").S("CullingOff");
        link("OO", mid, group(m.assembly));
        link("OO", gid, mid);
        link("OO", materialId[static_cast<std::size_t>(m.material)], mid);
        ++models, ++geometries;
        rep.triangles += m.idx.size() / 3;
        ++rep.bodies;
    }
    rep.materials = p.materials.size();
    rep.textures = p.textures.size();
    // What kinds of objects there are.
    defs.add("Version").I(100);
    defs.add("Count").I(static_cast<std::int32_t>(1 + models + geometries + p.materials.size() + 2 * rep.textures));
    auto kind = [&](const char* type, std::size_t n) {
        if (n) defs.add("ObjectType").S(type).add("Count").I(static_cast<std::int32_t>(n));
    };
    kind("GlobalSettings", 1);
    kind("Model", models);
    kind("Geometry", geometries);
    kind("Material", p.materials.size());
    kind("Texture", rep.textures);
    kind("Video", rep.textures);
    const FbxNode* top[] = {&header, &globals, &documents, &references, &defs, &objects, &connections};

    std::string out("Kaydara FBX Binary  \0\x1a\0", 23);
    const std::uint32_t version = 7400;
    out.append(reinterpret_cast<const char*>(&version), 4);
    for (const FbxNode* n : top) writeFbxNode(*n, out);
    out.append(13, '\0');
    // The footer: an id, padding to 16 bytes, the version, zeros and the closing magic.
    static const unsigned char kFootId[16] = {0xfa, 0xbc, 0xab, 0x09, 0xd0, 0xc8, 0xd4, 0x66, 0xb1, 0x76, 0xfb, 0x83, 0x1c, 0xf7, 0x26, 0x7e};
    static const unsigned char kMagic[16] = {0xf8, 0x5a, 0x8c, 0x6a, 0xde, 0xf5, 0xd9, 0x7e, 0xec, 0xe9, 0x0c, 0xe3, 0x75, 0x8f, 0x29, 0x0b};
    out.append(reinterpret_cast<const char*>(kFootId), 16);
    std::size_t pad = ((out.size() + 15) & ~std::size_t(15)) - out.size();
    if (pad == 0) pad = 16;
    out.append(pad, '\0');
    out.append(reinterpret_cast<const char*>(&version), 4);
    out.append(120, '\0');
    out.append(reinterpret_cast<const char*>(kMagic), 16);
    return writeFile(path, out, why);
}

} // namespace

bool isModelExportExtension(const std::string& ext) {
    const std::string e = lowerOf(ext);
    return e == ".gltf" || e == ".glb" || e == ".fbx";
}

bool exportModel(const Scene& scene, const std::string& path, ModelExportReport* report, std::string* why) {
    ModelExportReport local;
    ModelExportReport& rep = report ? *report : local;
    rep = ModelExportReport{};
    const std::string ext = extensionOf(path);
    if (!isModelExportExtension(ext)) {
        if (why) *why = "Export models as .gltf, .glb or .fbx";
        return false;
    }
    Prepared p = prepare(scene, ext != ".fbx");
    if (p.meshes.empty()) {
        if (why) *why = "Nothing to export: no visible bodies";
        return false;
    }
    rep.notes = p.notes;
    if (ext == ".fbx") {
        rep.format = "FBX 7.4 (binary)";
        return writeFbx(p, path, rep, why);
    }
    rep.format = ext == ".glb" ? "glTF 2.0 (binary)" : "glTF 2.0";
    return writeGltf(p, path, ext == ".glb", rep, why);
}

std::string modelExportSummary(const ModelExportReport& r, const std::string& path) {
    auto count = [](std::size_t n, const char* one, const char* many) { return std::to_string(n) + " " + (n == 1 ? one : many); };
    std::string s = "Exported " + count(r.bodies, "body", "bodies") + " (" + count(r.triangles, "triangle", "triangles") + ", " +
                    count(r.materials, "material", "materials");
    if (r.textures) s += ", " + count(r.textures, "map", "maps");
    if (r.skins) s += ", " + count(r.skins, "skin", "skins");
    s += ") to " + path + " (" + r.format + (r.format.rfind("FBX", 0) == 0 ? ", centimetres, +Y up)" : ", metres, +Y up)");
    return s;
}

} // namespace sicdm

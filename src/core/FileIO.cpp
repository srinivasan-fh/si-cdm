#include "core/FileIO.hpp"
#include "core/Multires.hpp"

#include "core/Brep.hpp"
#include "core/Step.hpp"

#include "core/Json.hpp"

#include <stb_image.h>

#include <algorithm>
#include <cstdlib>
#include <map>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unordered_map>

#if defined(_WIN32)
#include <io.h>
#else
#include <unistd.h>
#endif

namespace sicdm {

namespace fs = std::filesystem;

namespace {

void setError(std::string* e, std::string msg) {
    if (e) *e = std::move(msg);
}

std::string lowerExt(const std::string& path) {
    std::string ext = fs::path(path).extension().string();
    for (char& c : ext) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return ext;
}

std::optional<std::string> readFile(const std::string& path, std::string* error) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        setError(error, "Cannot open " + path);
        return std::nullopt;
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

} // namespace

bool writeFile(const std::string& path, const std::string& data, std::string* error) {
    // Written beside the target, flushed to the disk, then renamed over it: a crash or a
    // full disk part way leaves the old file whole, never a half-written one.
    std::error_code ec;
    const fs::path target(path);
    const fs::path parent = target.parent_path();
    if (!parent.empty()) fs::create_directories(parent, ec);
    fs::path tmp = target;
    tmp += ".saving";
    std::FILE* f = std::fopen(tmp.string().c_str(), "wb");
    if (!f) {
        setError(error, "Cannot write " + path);
        return false;
    }
    bool ok = std::fwrite(data.data(), 1, data.size(), f) == data.size() && std::fflush(f) == 0;
#if defined(_WIN32)
    ok = ok && _commit(_fileno(f)) == 0;
#else
    ok = ok && fsync(fileno(f)) == 0;
#endif
    ok = std::fclose(f) == 0 && ok;
    if (ok) {
        fs::rename(tmp, target, ec);
        ok = !ec;
    }
    if (!ok) {
        fs::remove(tmp, ec);
        setError(error, "Write failed for " + path);
        return false;
    }
    return true;
}

namespace {

const char* curveKey(CurveKind k) {
    switch (k) {
    case CurveKind::Point: return "point";
    case CurveKind::Line: return "line";
    case CurveKind::Polyline: return "polyline";
    case CurveKind::Rectangle: return "rectangle";
    case CurveKind::Circle: return "circle";
    case CurveKind::Arc: return "arc";
    case CurveKind::Spline: return "spline";
    case CurveKind::Ellipse: return "ellipse";
    case CurveKind::Slot: return "slot";
    }
    return "line";
}

std::optional<CurveKind> curveFromKey(const std::string& k) {
    if (k == "point") return CurveKind::Point;
    if (k == "line") return CurveKind::Line;
    if (k == "polyline") return CurveKind::Polyline;
    if (k == "rectangle") return CurveKind::Rectangle;
    if (k == "circle") return CurveKind::Circle;
    if (k == "arc") return CurveKind::Arc;
    if (k == "spline") return CurveKind::Spline;
    if (k == "ellipse") return CurveKind::Ellipse;
    if (k == "slot") return CurveKind::Slot;
    return std::nullopt;
}

Json vec3Json(const Vec3& v) { return Json(Json::Array{v.x, v.y, v.z}); }
Vec3 jsonVec3(const Json& j, Vec3 fallback = {}) {
    if (!j.isArray() || j.size() < 3) return fallback;
    return {j[0].asFloat(), j[1].asFloat(), j[2].asFloat()};
}

std::string stemOf(const std::string& path) { return fs::path(path).stem().string(); }

// Flat-shaded triangle soup -> MeshData with feature edges.
MeshData meshFromTriangles(const std::vector<Vec3>& tri) {
    MeshData m;
    m.vertices.reserve(tri.size());
    m.indices.reserve(tri.size());
    for (std::size_t i = 0; i + 2 < tri.size(); i += 3) {
        const Vec3 n = normalize(cross(tri[i + 1] - tri[i], tri[i + 2] - tri[i]));
        for (int k = 0; k < 3; ++k) {
            m.indices.push_back(static_cast<std::uint32_t>(m.vertices.size()));
            m.vertices.push_back({tri[i + static_cast<std::size_t>(k)], n});
        }
    }
    m.computeFeatureEdges();
    return m;
}

template <typename F>
void forEachWorldTriangle(const Scene& scene, F&& f) {
    for (const Body& b : scene.bodies()) {
        if (!b.visible) continue;
        const auto& v = b.mesh.vertices;
        const auto& idx = b.mesh.indices;
        for (std::size_t i = 0; i + 2 < idx.size(); i += 3)
            f(b, v[idx[i]].position + b.offset, v[idx[i + 1]].position + b.offset, v[idx[i + 2]].position + b.offset);
    }
}

void appendF32(std::string& s, float v) {
    char b[4];
    std::memcpy(b, &v, 4);
    s.append(b, 4);
}

// ---------------------------------------------------------------------------
// DXF helpers

struct DxfPair {
    int code;
    std::string value;
};

std::string trimmed(std::string s) {
    while (!s.empty() && (s.back() == '\r' || s.back() == '\n' || s.back() == ' ' || s.back() == '\t')) s.pop_back();
    std::size_t i = 0;
    while (i < s.size() && (s[i] == ' ' || s[i] == '\t')) ++i;
    return s.substr(i);
}

void dxfPair(std::string& out, int code, const std::string& value) {
    out += std::to_string(code);
    out += '\n';
    out += value;
    out += '\n';
}

std::string num(float v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.6f", static_cast<double>(v));
    return buf;
}

void dxfPoint(std::string& out, int base, Vec2 p) {
    dxfPair(out, base, num(p.x));
    dxfPair(out, base + 10, num(p.y));
    dxfPair(out, base + 20, "0.0");
}

void dxfPolyline(std::string& out, const std::string& layer, const std::vector<Vec2>& pts, bool closed) {
    dxfPair(out, 0, "POLYLINE");
    dxfPair(out, 8, layer);
    dxfPair(out, 66, "1");
    dxfPair(out, 70, closed ? "1" : "0");
    dxfPoint(out, 10, {0, 0});
    for (const Vec2& p : pts) {
        dxfPair(out, 0, "VERTEX");
        dxfPair(out, 8, layer);
        dxfPoint(out, 10, p);
    }
    dxfPair(out, 0, "SEQEND");
    dxfPair(out, 8, layer);
}

// An ARC entity: centre, radius, start and end angles (counter-clockwise, degrees).
void dxfArc(std::string& out, const std::string& layer, Vec2 c, float r, float a0, float a1) {
    dxfPair(out, 0, "ARC");
    dxfPair(out, 8, layer);
    dxfPoint(out, 10, c);
    dxfPair(out, 40, num(r));
    dxfPair(out, 50, num(a0 * 57.29577951f));
    dxfPair(out, 51, num(a1 * 57.29577951f));
}


std::string lowerCopyOf(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

Json frameToJson(const PlaneFrame& f) {
    Json a = Json::array();
    for (const Vec3& v : {f.origin, f.u, f.v})
        for (float x : {v.x, v.y, v.z}) a.push(Json(x));
    return a;
}

// False (and `f` untouched) unless it holds an origin and two square unit axes.
bool frameFromJson(const Json& j, PlaneFrame& f) {
    const auto& n = j.items();
    if (n.size() != 9) return false;
    float x[9];
    for (std::size_t i = 0; i < 9; ++i) x[i] = n[i].asFloat();
    const Vec3 o{x[0], x[1], x[2]}, u{x[3], x[4], x[5]}, v{x[6], x[7], x[8]};
    if (std::fabs(length(u) - 1.0f) > 1e-3f || std::fabs(length(v) - 1.0f) > 1e-3f || std::fabs(dot(u, v)) > 1e-3f) return false;
    return makePlaneFrame(o, cross(u, v), u, f);
}

} // namespace

const DatumAxis* findRefAxis(const ProjectState& st, const std::string& name) {
    for (const DatumAxis& a : st.refAxes)
        if (lowerCopyOf(a.name) == lowerCopyOf(name)) return &a;
    return nullptr;
}

const DatumPoint* findRefPoint(const ProjectState& st, const std::string& name) {
    for (const DatumPoint& p : st.refPoints)
        if (lowerCopyOf(p.name) == lowerCopyOf(name)) return &p;
    return nullptr;
}

// The active sketch plane's frame.
PlaneFrame activePlaneFrame(const ProjectState& st) {
    return st.plane == SketchPlane::Custom ? st.customFrame : planeFrame(st.plane);
}

std::string activePlaneName(const ProjectState& st) {
    if (st.plane != SketchPlane::Custom) return planeName(st.plane);
    return st.customPlane.empty() ? std::string("Custom") : st.customPlane;
}

const char* planeKey(SketchPlane p) {
    switch (p) {
    case SketchPlane::Top: return "top";
    case SketchPlane::Front: return "front";
    case SketchPlane::Right: return "right";
    case SketchPlane::Custom: return "custom";
    }
    return "top";
}

std::optional<SketchPlane> planeFromKey(const std::string& keyIn) {
    std::string k = keyIn;
    for (char& c : k) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (k == "top" || k == "xy") return SketchPlane::Top;
    if (k == "front" || k == "xz") return SketchPlane::Front;
    if (k == "right" || k == "yz") return SketchPlane::Right;
    if (k == "custom") return SketchPlane::Custom;
    return std::nullopt;
}

FileKind fileKindOf(const std::string& path) {
    const std::string e = lowerExt(path);
    if (e == ".sicdm") return FileKind::Project;
    if (e == ".stl") return FileKind::Stl;
    if (e == ".obj") return FileKind::Obj;
    if (e == ".dxf") return FileKind::Dxf;
    if (e == ".svg") return FileKind::Svg;
    if (e == ".step" || e == ".stp" || e == ".p21") return FileKind::Step;
    if (e == ".igs" || e == ".iges") return FileKind::Iges;
    if (e == ".png" || e == ".jpg" || e == ".jpeg" || e == ".bmp" || e == ".tga" || e == ".gif" || e == ".psd" ||
        e == ".hdr" || e == ".pnm" || e == ".ppm" || e == ".pgm")
        return FileKind::Image;
    return FileKind::Unknown;
}

const char* fileKindName(FileKind k) {
    switch (k) {
    case FileKind::Project: return "SI CDM project";
    case FileKind::Stl: return "STL mesh";
    case FileKind::Obj: return "OBJ mesh";
    case FileKind::Dxf: return "DXF drawing";
    case FileKind::Svg: return "SVG drawing";
    case FileKind::Step: return "STEP model";
    case FileKind::Iges: return "IGES model";
    case FileKind::Image: return "image";
    case FileKind::Unknown: return "unknown";
    }
    return "unknown";
}

// ---------------------------------------------------------------------------
// Large meshes go in as little-endian binary arrays in base64 (a number per vertex
// coordinate as text would make a part of half a million triangles some 30 MB of JSON).
constexpr std::size_t kBinaryMeshVertices = 20000;

namespace {

const char kB64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

std::string base64(const std::vector<unsigned char>& in) {
    std::string out;
    out.reserve((in.size() + 2) / 3 * 4);
    std::size_t i = 0;
    for (; i + 2 < in.size(); i += 3) {
        const unsigned v = (unsigned{in[i]} << 16) | (unsigned{in[i + 1]} << 8) | in[i + 2];
        out += kB64[(v >> 18) & 63], out += kB64[(v >> 12) & 63], out += kB64[(v >> 6) & 63], out += kB64[v & 63];
    }
    if (i < in.size()) {
        const unsigned v = (unsigned{in[i]} << 16) | (i + 1 < in.size() ? unsigned{in[i + 1]} << 8 : 0u);
        out += kB64[(v >> 18) & 63], out += kB64[(v >> 12) & 63];
        out += i + 1 < in.size() ? kB64[(v >> 6) & 63] : '=';
        out += '=';
    }
    return out;
}

bool unbase64(const std::string& s, std::vector<unsigned char>& out) {
    static const auto table = [] {
        std::array<int, 256> t{};
        t.fill(-1);
        for (int k = 0; k < 64; ++k) t[static_cast<unsigned char>(kB64[k])] = k;
        return t;
    }();
    out.clear();
    out.reserve(s.size() / 4 * 3);
    unsigned v = 0;
    int bits = 0;
    for (char c : s) {
        if (c == '=') break;
        const int d = table[static_cast<unsigned char>(c)];
        if (d < 0) return false;
        v = (v << 6) | static_cast<unsigned>(d);
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            out.push_back(static_cast<unsigned char>((v >> bits) & 0xFF));
        }
    }
    return true;
}

void putU32(std::vector<unsigned char>& b, std::uint32_t u) {
    for (int k = 0; k < 4; ++k) b.push_back(static_cast<unsigned char>((u >> (8 * k)) & 0xFF));
}
void putF32(std::vector<unsigned char>& b, float f) {
    std::uint32_t u;
    std::memcpy(&u, &f, 4);
    putU32(b, u);
}
std::uint32_t getU32(const unsigned char* p) { return std::uint32_t{p[0]} | (std::uint32_t{p[1]} << 8) | (std::uint32_t{p[2]} << 16) | (std::uint32_t{p[3]} << 24); }
float getF32(const unsigned char* p) {
    const std::uint32_t u = getU32(p);
    float f;
    std::memcpy(&f, &u, 4);
    return f;
}

} // namespace

// Project

bool saveProject(const Scene& scene, const ProjectState& state, const std::string& path, std::string* error) {
    const fs::path projectDir = fs::absolute(fs::path(path)).parent_path();
    Json doc = Json::object();
    doc.set("format", "sicdm-project");
    doc.set("version", kProjectVersion);
    doc.set("reader_version", 1); // what 2 to 4 added is optional: a format-1 reader can open it (5: see below)
    bool binaryMeshes = false;
    std::unordered_map<std::uint64_t, std::pair<std::string, bool>> written; // shape key -> the body holding it, exact
    std::unordered_map<std::string, int> nameCount; // only a body with a name of its own can be referred to
    for (const Body& b : scene.bodies()) ++nameCount[b.name];
#ifdef SICDM_VERSION
    doc.set("saved_by", std::string("SI CDM ") + SICDM_VERSION);
#endif
    doc.set("units", "mm"); // storage unit of every length in the file
    Json du = Json::object();
    du.set("unit", unitInfo(state.units.unit).symbol)
        .set("precision", state.units.precision)
        .set("style", state.units.style == LengthStyle::Fractional ? "fractional" : "decimal")
        .set("denominator", state.units.denominator);
    doc.set("display_units", du);
    doc.set("plane", planeKey(state.plane));
    if (state.plane == SketchPlane::Custom) {
        Json cp = Json::object();
        cp.set("name", state.customPlane).set("frame", frameToJson(state.customFrame));
        doc.set("custom_plane", cp);
    }
    if (!state.variables.empty()) {
        Json vs = Json::array();
        for (const Variable& v : state.variables) {
            Json jv = Json::object();
            jv.set("name", v.name).set("expr", v.expr);
            vs.push(jv);
        }
        doc.set("variables", vs);
    }
    if (!state.configs.empty()) {
        Json cs = Json::array();
        for (const Configuration& c : state.configs) {
            Json jc = Json::object(), vals = Json::object();
            for (const auto& [k, e] : c.values) vals.set(k, e);
            jc.set("name", c.name).set("values", vals);
            cs.push(jc);
        }
        doc.set("configs", cs);
        if (!state.config.empty()) doc.set("config", state.config);
    }
    if (!state.refAxes.empty()) {
        Json as = Json::array();
        for (const DatumAxis& x : state.refAxes) {
            Json j = Json::object();
            j.set("name", x.name).set("origin", vec3Json(x.origin)).set("dir", vec3Json(x.dir));
            as.push(j);
        }
        doc.set("ref_axes", as);
    }
    if (!state.refPoints.empty()) {
        Json ps = Json::array();
        for (const DatumPoint& x : state.refPoints) {
            Json j = Json::object();
            j.set("name", x.name).set("at", vec3Json(x.at));
            ps.push(j);
        }
        doc.set("ref_points", ps);
    }
    if (!state.datums.empty()) {
        Json ds = Json::array();
        for (const DatumPlane& d : state.datums) {
            Json jd = Json::object();
            jd.set("name", d.name).set("frame", frameToJson(d.frame));
            ds.push(jd);
        }
        doc.set("datums", ds);
    }

    Json bodies = Json::array();
    for (const Body& b : scene.bodies()) {
        Json jb = Json::object();
        jb.set("name", b.name);
        jb.set("kind", b.kind);
        Json params = Json::array();
        for (float p : b.params) params.push(p);
        jb.set("params", params);
        jb.set("offset", vec3Json(b.offset));
        jb.set("color", vec3Json(b.color));
        jb.set("visible", b.visible);
        if (b.scale != 1.0f) jb.set("scale", b.scale);
        if (b.rotation.x != 0.0f || b.rotation.y != 0.0f || b.rotation.z != 0.0f) jb.set("rotation", vec3Json(b.rotation));
        if (!b.component.empty()) jb.set("component", b.component);
        if (!b.attachedTo.empty()) jb.set("attached_to", b.attachedTo);
        if (!b.assembly.empty()) jb.set("assembly", b.assembly);
        if (b.linked()) { // a part file's body: the file relative to this one (the pair moves together)
            // (Between the real folders, links followed: the same whichever way the project's folder
            // is spelled when it is opened.)
            std::error_code ec1, ec2;
            const fs::path realDir = fs::weakly_canonical(projectDir, ec1), realFile = fs::weakly_canonical(fs::path(b.link.file), ec2);
            const fs::path rel = ec1 || ec2 ? fs::path(b.link.file).lexically_relative(projectDir) : realFile.lexically_relative(realDir); // empty: another drive
            const std::string file = rel.empty() ? fs::path(b.link.file).generic_string() : rel.generic_string();
            jb.set("link", Json::object()
                               .set("file", file)
                               .set("body", b.link.body)
                               .set("part_offset", vec3Json(b.link.partOffset))
                               .set("part_rotation", vec3Json(b.link.partRotation))
                               .set("vertices", static_cast<double>(b.link.vertices))
                               .set("stamp", std::to_string(b.link.stamp)));
        }
        if (!b.field.empty()) jb.set("field", b.field);
        if (b.multires && !b.multires->levels.empty()) { // the cage's triangles and each level's positions (finer triangles follow)
            std::vector<unsigned char> tri;
            for (const auto& t : b.multires->levels[0].t) putU32(tri, t[0]), putU32(tri, t[1]), putU32(tri, t[2]);
            Json levels = Json::array();
            for (const MultiresLevel& l : b.multires->levels) {
                std::vector<unsigned char> pos;
                for (const Vec3& q : l.p) putF32(pos, q.x), putF32(pos, q.y), putF32(pos, q.z);
                levels.push(base64(pos));
            }
            jb.set("multires", Json::object().set("current", b.multires->current).set("cage_triangles_b64", base64(tri)).set("levels_b64", levels));
        }
        if (!b.holes.empty()) {
            Json hs = Json::array();
            for (const HoleNote& h : b.holes) {
                Json jh = Json::object();
                jh.set("at", vec3Json(h.at)).set("dir", vec3Json(h.dir)).set("callout", h.callout);
                if (h.threadDia > 0) {
                    jh.set("thread", h.threadDia).set("pitch", h.pitch).set("thread_depth", h.threadDepth);
                    if (h.threadModelled) jh.set("modelled", true);
                }
                if (h.diameter > 0) {
                    jh.set("diameter", h.diameter).set("depth", h.depth);
                    if (h.cboreDia > 0) jh.set("cbore", h.cboreDia).set("cbore_depth", h.cboreDepth);
                    if (h.csinkDia > 0) jh.set("csink", h.csinkDia).set("csink_angle", h.csinkAngle);
                    if (!h.size.empty()) jh.set("size", h.size);
                }
                hs.push(jh);
            }
            jb.set("holes", hs);
        }
        if (b.physics.enabled) {
            const PhysicsProps& p = b.physics;
            Json ph = Json::object();
            ph.set("fixed", p.fixed);
            if (p.mass > 0) ph.set("mass", p.mass);
            ph.set("density", p.density);
            ph.set("velocity", vec3Json(p.velocity));
            ph.set("spin", vec3Json(p.spin));
            ph.set("charge", p.charge);
            ph.set("temperature", p.temperature);
            ph.set("specific_heat", p.specificHeat);
            ph.set("restitution", p.restitution);
            ph.set("force", vec3Json(p.force));
            if (p.wave != WaveKind::Off) {
                ph.set("wave", waveKindKey(p.wave));
                ph.set("wave_frequency", p.waveFrequency);
                ph.set("wave_speed", p.waveSpeed);
            }
            jb.set("physics", ph);
        }
        if (!b.material.isDefault()) {
            const MaterialProps& m = b.material;
            Json jm = Json::object();
            if (!m.preset.empty()) jm.set("preset", m.preset);
            jm.set("metallic", m.metallic).set("roughness", m.roughness).set("transmission", m.transmission).set("ior", m.ior).set("emission", m.emission);
            if (!m.colorMap.empty()) jm.set("color_map", m.colorMap);
            if (!m.normalMap.empty()) jm.set("normal_map", m.normalMap);
            jb.set("material", jm);
        }
        if (!b.properties.empty()) {
            Json jp = Json::object();
            for (const auto& [k, v] : b.properties) jp.set(k, v);
            jb.set("properties", jp);
        }
        // A large shape already written for another body (pattern copies, repeated parts):
        // a reference to it.
        const std::uint64_t shapeKey = !isPrimitiveKind(b.kind) && b.mesh.vertices.size() >= kBinaryMeshVertices ? meshContentKey(b.mesh) : 0;
        const auto sameAs = shapeKey ? written.find(shapeKey) : written.end();
        if (sameAs != written.end() && sameAs->second.second == b.exact()) {
            jb.set("mesh_same_as", sameAs->second.first);
            binaryMeshes = true;
        } else if (!isPrimitiveKind(b.kind) && b.mesh.vertices.size() >= kBinaryMeshVertices) {
            if (nameCount[b.name] == 1) written.emplace(shapeKey, std::make_pair(b.name, b.exact()));
            std::vector<unsigned char> pos, nrm, idx, edges, mask;
            pos.reserve(b.mesh.vertices.size() * 12), nrm.reserve(b.mesh.vertices.size() * 12);
            bool masked = false;
            for (const Vertex& v : b.mesh.vertices) {
                putF32(pos, v.position.x), putF32(pos, v.position.y), putF32(pos, v.position.z);
                putF32(nrm, v.normal.x), putF32(nrm, v.normal.y), putF32(nrm, v.normal.z);
                masked = masked || v.mask > 0.0f;
            }
            idx.reserve(b.mesh.indices.size() * 4);
            for (std::uint32_t i : b.mesh.indices) putU32(idx, i);
            for (const Vec3& e : b.mesh.edgeVertices) putF32(edges, e.x), putF32(edges, e.y), putF32(edges, e.z);
            Json mesh = Json::object();
            mesh.set("encoding", "binary-le").set("positions_b64", base64(pos)).set("normals_b64", base64(nrm)).set("indices_b64", base64(idx));
            mesh.set("edges_b64", base64(edges));
            if (masked) {
                for (const Vertex& v : b.mesh.vertices) putF32(mask, v.mask);
                mesh.set("mask_b64", base64(mask));
            }
            if (std::any_of(b.mesh.vertices.begin(), b.mesh.vertices.end(), [](const Vertex& v) { return painted(v.color); })) {
                std::vector<unsigned char> col;
                for (const Vertex& v : b.mesh.vertices) putU32(col, v.color);
                mesh.set("colors_b64", base64(col));
            }
            if (b.mesh.hasGroups()) {
                std::vector<unsigned char> grp;
                for (std::uint32_t g : b.mesh.groups) putU32(grp, g);
                mesh.set("groups_b64", base64(grp));
            }
            if (b.mesh.hasQuads()) {
                std::vector<unsigned char> qd;
                for (const auto& q : b.mesh.quads) putU32(qd, q[0]), putU32(qd, q[1]), putU32(qd, q[2]), putU32(qd, q[3]);
                mesh.set("quads_b64", base64(qd));
            }
            if (b.mesh.hasUvs()) {
                std::vector<unsigned char> uv;
                for (const Vec2& t : b.mesh.uvs) putF32(uv, t.x), putF32(uv, t.y);
                mesh.set("uvs_b64", base64(uv));
            }
            if (b.mesh.hasLayers()) {
                Json layers = Json::array();
                for (const SculptLayer& l : b.mesh.layers) {
                    std::vector<unsigned char> off;
                    for (const Vec3& o : l.offset) putF32(off, o.x), putF32(off, o.y), putF32(off, o.z);
                    layers.push(Json::object().set("name", l.name).set("strength", l.strength).set("saved", l.saved).set("offsets_b64", base64(off)));
                }
                mesh.set("layers", layers).set("record_layer", b.mesh.recordLayer);
            }
            jb.set("mesh", mesh);
            binaryMeshes = true;
            if (b.exact()) jb.set("brep_step", writeStep({StepPart{b.name, std::const_pointer_cast<Brep>(b.brep), ""}}, b.name + ".step"));
        } else if (!isPrimitiveKind(b.kind)) {
            Json pos = Json::array(), nrm = Json::array(), idx = Json::array(), edges = Json::array();
            for (const Vertex& v : b.mesh.vertices) {
                pos.push(v.position.x).push(v.position.y).push(v.position.z);
                nrm.push(v.normal.x).push(v.normal.y).push(v.normal.z);
            }
            for (std::uint32_t i : b.mesh.indices) idx.push(static_cast<double>(i));
            for (const Vec3& e : b.mesh.edgeVertices) edges.push(e.x).push(e.y).push(e.z);
            Json mesh = Json::object();
            mesh.set("positions", pos).set("normals", nrm).set("indices", idx).set("edges", edges);
            if (std::any_of(b.mesh.vertices.begin(), b.mesh.vertices.end(), [](const Vertex& v) { return v.mask > 0.0f; })) {
                Json mask = Json::array(); // the sculpt mask, only when there is one
                for (const Vertex& v : b.mesh.vertices) mask.push(v.mask);
                mesh.set("mask", mask);
            }
            if (std::any_of(b.mesh.vertices.begin(), b.mesh.vertices.end(), [](const Vertex& v) { return painted(v.color); })) {
                Json col = Json::array(); // polypaint, 0xAABBGGRR per vertex
                for (const Vertex& v : b.mesh.vertices) col.push(static_cast<double>(v.color));
                mesh.set("colors", col);
            }
            if (b.mesh.hasGroups()) {
                Json grp = Json::array(); // polygroups, per triangle
                for (std::uint32_t g : b.mesh.groups) grp.push(static_cast<double>(g));
                mesh.set("groups", grp);
            }
            if (b.mesh.hasQuads()) {
                Json qd = Json::array(); // quad faces (their triangles are the indices, two each)
                for (const auto& q : b.mesh.quads) qd.push(static_cast<double>(q[0])).push(static_cast<double>(q[1])).push(static_cast<double>(q[2])).push(static_cast<double>(q[3]));
                mesh.set("quads", qd);
            }
            if (b.mesh.hasUvs()) {
                Json uv = Json::array(); // texture coordinates, a pair per triangle corner
                for (const Vec2& t : b.mesh.uvs) uv.push(t.x).push(t.y);
                mesh.set("uvs", uv);
            }
            if (b.mesh.hasLayers()) {
                Json layers = Json::array(); // sculpt layers: per vertex offsets
                for (const SculptLayer& l : b.mesh.layers) {
                    Json off = Json::array();
                    for (const Vec3& o : l.offset) off.push(o.x).push(o.y).push(o.z);
                    layers.push(Json::object().set("name", l.name).set("strength", l.strength).set("saved", l.saved).set("offsets", off));
                }
                mesh.set("layers", layers).set("record_layer", b.mesh.recordLayer);
            }
            jb.set("mesh", mesh);
            // Exact geometry kept with the mesh it belongs to (as a STEP record).
            if (b.exact()) jb.set("brep_step", writeStep({StepPart{b.name, std::const_pointer_cast<Brep>(b.brep), ""}}, b.name + ".step"));
        }
        bodies.push(jb);
    }
    doc.set("bodies", bodies);
    if (binaryMeshes) doc.set("reader_version", 5); // a reader before 5 can't read those meshes
    bool customPlanes = false; // a reader before 10 would put them on the Top plane
    for (const SketchCurve& c : scene.sketch().curves()) customPlanes = customPlanes || c.plane == SketchPlane::Custom;
    for (const Body& b : scene.bodies())
        customPlanes = customPlanes || (b.kind == "extrude" && !b.params.empty() && b.params[0] == static_cast<float>(SketchPlane::Custom));
    if (customPlanes) doc.set("reader_version", 10);

    Json curves = Json::array();
    for (const SketchCurve& c : scene.sketch().curves()) {
        Json jc = Json::object();
        jc.set("name", c.name).set("kind", curveKey(c.kind)).set("plane", planeKey(c.plane));
        if (c.plane == SketchPlane::Custom) jc.set("frame", frameToJson(c.frame));
        Json pts = Json::array();
        for (const Vec2& p : c.pts) pts.push(Json(Json::Array{p.x, p.y}));
        jc.set("points", pts).set("radius", c.radius).set("visible", c.visible);
        if (c.closed) jc.set("closed", true);
        if (c.construction) jc.set("construction", true);
        curves.push(jc);
    }
    doc.set("sketch", curves);
    if (!scene.sketch().constraints().empty()) doc.set("constraints", constraintsToJson(scene.sketch()));

    Json images = Json::array();
    for (const ImageRef& img : scene.images()) {
        std::error_code ec;
        std::string stored = img.path;
        const fs::path rel = fs::relative(fs::absolute(img.path), projectDir, ec);
        if (!ec && !rel.empty() && rel.string().rfind("..", 0) != 0) stored = rel.generic_string();
        Json ji = Json::object();
        ji.set("name", img.name).set("path", stored).set("plane", planeKey(img.plane));
        ji.set("center", Json(Json::Array{img.center.x, img.center.y}));
        ji.set("width", img.width).set("opacity", img.opacity).set("visible", img.visible);
        images.push(ji);
    }
    doc.set("images", images);

    Json motions = Json::array();
    for (const Motion& m : scene.motions()) {
        Json jm = Json::object();
        Json targets = Json::array();
        for (const std::string& b : m.bodies) targets.push(b);
        jm.set("name", m.name).set("kind", motionKindKey(m.kind)).set("bodies", targets);
        jm.set("axis", vec3Json(m.axis)).set("origin", vec3Json(m.origin)).set("each_centre", m.eachCentre);
        jm.set("speed", m.speed).set("range", m.range).set("mode", motionModeKey(m.mode));
        jm.set("start", m.start).set("enabled", m.enabled);
        if (!m.follow.empty()) jm.set("follow", m.follow).set("radius", m.radius).set("ratio", m.ratio);
        if (m.step > 0.0f) jm.set("step", m.step);
        if (m.alongLoop) jm.set("along_loop", true);
        if (!m.keys.empty()) {
            Json keys = Json::array();
            for (const MotionKey& k : m.keys) keys.push(Json(Json::Array{k.t, k.value}));
            jm.set("keys", keys);
            if (m.keySpline) jm.set("key_curve", "spline");
            if (m.keysAdvance) jm.set("keys_advance", true);
        }
        if (m.hasLimits) jm.set("limits", Json(Json::Array{m.lo, m.hi}));
        motions.push(jm);
    }
    doc.set("motions", motions);
    {
        const PhysicsSettings& p = scene.physics();
        Json ph = Json::object();
        ph.set("enabled", p.enabled);
        ph.set("gravity", gravityModeKey(p.gravity));
        ph.set("g", vec3Json(p.g));
        ph.set("big_g", p.bigG);
        ph.set("floor", p.floor);
        ph.set("friction", p.friction);
        ph.set("air", p.air);
        if (p.wind.x != 0 || p.wind.y != 0 || p.wind.z != 0) ph.set("wind", vec3Json(p.wind));
        ph.set("electric_field", vec3Json(p.electricField));
        ph.set("magnetic_field", vec3Json(p.magneticField));
        ph.set("ambient", p.ambient);
        ph.set("convection", p.convection);
        ph.set("conductance", p.conductance);
        ph.set("relativity", p.relativity);
        ph.set("light_speed", p.lightSpeed);
        ph.set("duration", p.duration);
        doc.set("physics", ph);
    }
    if (state.simDuration > 0.0f) doc.set("simulation_duration", state.simDuration);
    if (!state.cam.empty()) doc.set("cam", camToJson(state.cam));
    if (!state.lathe.empty()) doc.set("lathe", latheToJson(state.lathe));
    if (!state.assembly.empty()) doc.set("assembly", assemblyToJson(state.assembly));
    if (!(state.section == SectionView{})) doc.set("section", sectionToJson(state.section));
    {
        const RenderSettings& r = state.render;
        doc.set("render", Json::object()
                              .set("environment", r.environment)
                              .set("sun", Json::array().push(r.sunAzimuth).push(r.sunElevation))
                              .set("light", r.light)
                              .set("exposure", r.exposure)
                              .set("floor", r.floor)
                              .set("size", Json::array().push(r.width).push(r.height))
                              .set("samples", r.samples)
                              .set("bounces", r.bounces)
                              .set("denoise", r.denoise));
    }
    {
        const ProductData& pd = state.product;
        Json jp = Json::object().set("revision", pd.revision).set("state", releaseStateName(pd.state));
        if (!pd.properties.empty()) {
            Json props = Json::object();
            for (const auto& [k, v] : pd.properties) props.set(k, v);
            jp.set("properties", props);
        }
        if (!pd.history.empty()) {
            Json hist = Json::array();
            for (const RevisionRecord& r : pd.history)
                hist.push(Json::object().set("revision", r.revision).set("state", releaseStateName(r.state)).set("date", r.date).set("author", r.author).set("note", r.note));
            jp.set("history", hist);
        }
        if (!pd.vaultItem.empty()) jp.set("vault_item", pd.vaultItem);
        doc.set("product", jp);
    }
    if (!scene.rigs().empty()) doc.set("robots", rigStateToJson(scene.rigs()));
    if (scene.terrain().active()) doc.set("terrain", terrainToJson(scene.terrain()));
    if (!state.drawing.empty()) doc.set("drawing", drawingToJson(state.drawing));
    if (!state.history.empty()) {
        Json h = Json::array();
        for (const HistoryStep& step : state.history) {
            Json j = Json::object();
            j.set("line", step.line);
            if (!step.undoable) j.set("undoable", false);
            if (step.millimetres) j.set("mm", true);
            j.set("unit", unitInfo(step.unit).symbol);
            if (!step.selectedBody.empty()) j.set("body", step.selectedBody);
            if (!step.selectedCurve.empty()) j.set("curve", step.selectedCurve);
            if (step.suppressed) j.set("suppressed", true);
            h.push(j);
        }
        doc.set("history", h);
        doc.set("history_complete", state.historyComplete);
        if (state.rollback >= 0) doc.set("history_rollback", state.rollback);
    }

    return writeFile(path, doc.dump() + "\n", error);
}

bool loadProject(Scene& scene, ProjectState& state, const std::string& path, std::string* error, std::string* warning) {
    const auto text = readFile(path, error);
    if (!text) return false;
    std::string parseError;
    const auto doc = Json::parse(*text, &parseError);
    if (!doc || !doc->isObject()) {
        setError(error, "Not a valid project file: " + parseError);
        return false;
    }
    if ((*doc)["format"].asString() != "sicdm-project") {
        setError(error, "Not an SI CDM project file");
        return false;
    }
    // Format versions: refuse a file that needs a newer reader; open a newer file that
    // doesn't (and say its additions are left out).
    const int version = (*doc)["version"].asInt(1);
    const int reader = (*doc)["reader_version"].asInt(version);
    const std::string savedBy = (*doc)["saved_by"].asString("a newer version of SI CDM");
    if (reader > kProjectVersion) {
        setError(error, "This project was saved by " + savedBy + " in format " + std::to_string(version) +
                            ", which this version (format " + std::to_string(kProjectVersion) + ") can't read. Update SI CDM to open it");
        return false;
    }
    std::string versionNote;
    if (version > kProjectVersion)
        versionNote = "Saved by " + savedBy + " (format " + std::to_string(version) + "): anything newer than format " +
                      std::to_string(kProjectVersion) + " in it was left out, and saving writes format " + std::to_string(kProjectVersion);

    Scene loaded;
    ProjectState st;
    st.plane = planeFromKey((*doc)["plane"].asString()).value_or(SketchPlane::Top);
    if (st.plane == SketchPlane::Custom) {
        const Json& cp = (*doc)["custom_plane"];
        st.customPlane = cp["name"].asString();
        if (!frameFromJson(cp["frame"], st.customFrame)) st.plane = SketchPlane::Top, st.customPlane.clear();
    }
    for (const Json& jv : (*doc)["variables"].items())
        if (validVariableName(jv["name"].asString()) && !findVariable(st.variables, jv["name"].asString()))
            st.variables.push_back({jv["name"].asString(), jv["expr"].asString()});
    for (const Json& jc : (*doc)["configs"].items()) {
        Configuration c;
        c.name = jc["name"].asString();
        if (c.name.empty()) continue;
        for (const auto& [k, e] : jc["values"].members()) c.values.push_back({k, e.asString()});
        st.configs.push_back(std::move(c));
    }
    st.config = (*doc)["config"].asString();
    for (const Json& j : (*doc)["ref_axes"].items()) {
        const Vec3 d = jsonVec3(j["dir"], {0, 0, 1});
        if (!j["name"].asString().empty() && length(d) > 1e-6f) st.refAxes.push_back({j["name"].asString(), jsonVec3(j["origin"]), normalize(d)});
    }
    for (const Json& j : (*doc)["ref_points"].items())
        if (!j["name"].asString().empty()) st.refPoints.push_back({j["name"].asString(), jsonVec3(j["at"])});
    for (const Json& jd : (*doc)["datums"].items()) {
        DatumPlane d;
        d.name = jd["name"].asString();
        if (!d.name.empty() && frameFromJson(jd["frame"], d.frame)) st.datums.push_back(std::move(d));
    }

    for (const Json& jb : (*doc)["bodies"].items()) {
        std::vector<float> params;
        for (const Json& p : jb["params"].items()) params.push_back(p.asFloat());
        const std::string kind = jb["kind"].asString("mesh");
        Body b;
        if (isPrimitiveKind(kind)) {
            b = makeBody(jb["name"].asString("Body"), kind, params);
        } else if (jb["mesh_same_as"].isString()) { // the shape of a body before it
            const std::string other = jb["mesh_same_as"].asString();
            const Body* src = nullptr;
            for (const Body& o : loaded.bodies())
                if (o.name == other) src = &o;
            if (!src) {
                setError(error, "Body " + jb["name"].asString() + " has the shape of " + other + ", which isn't before it in the file");
                return false;
            }
            b.mesh = src->mesh;
            b.brep = src->brep;
            b.brepKey = src->brepKey;
            b.name = jb["name"].asString("Mesh");
            b.kind = "mesh";
        } else {
            const Json& m = jb["mesh"];
            if (m["positions_b64"].isString()) {
                std::vector<unsigned char> pos, nrm, idx, edges, mask;
                if (!unbase64(m["positions_b64"].asString(), pos) || !unbase64(m["normals_b64"].asString(), nrm) || !unbase64(m["indices_b64"].asString(), idx) ||
                    !unbase64(m["edges_b64"].asString(), edges) || !unbase64(m["mask_b64"].asString(), mask) || pos.size() % 12 || idx.size() % 12) {
                    setError(error, "Corrupt mesh in body " + jb["name"].asString());
                    return false;
                }
                const std::size_t nv = pos.size() / 12;
                b.mesh.vertices.resize(nv);
                for (std::size_t i = 0; i < nv; ++i) {
                    Vertex& v = b.mesh.vertices[i];
                    v.position = {getF32(&pos[12 * i]), getF32(&pos[12 * i + 4]), getF32(&pos[12 * i + 8])};
                    if (nrm.size() >= 12 * (i + 1)) v.normal = {getF32(&nrm[12 * i]), getF32(&nrm[12 * i + 4]), getF32(&nrm[12 * i + 8])};
                    if (mask.size() >= 4 * (i + 1)) v.mask = std::clamp(getF32(&mask[4 * i]), 0.0f, 1.0f);
                }
                b.mesh.indices.resize(idx.size() / 4);
                for (std::size_t i = 0; i < b.mesh.indices.size(); ++i) {
                    const std::uint32_t k = getU32(&idx[4 * i]);
                    if (k >= nv) {
                        setError(error, "Corrupt mesh in body " + jb["name"].asString());
                        return false;
                    }
                    b.mesh.indices[i] = k;
                }
                for (std::size_t i = 0; i + 12 <= edges.size(); i += 12) b.mesh.edgeVertices.push_back({getF32(&edges[i]), getF32(&edges[i + 4]), getF32(&edges[i + 8])});
            }
            const auto& pos = m["positions"].items();
            const auto& nrm = m["normals"].items();
            const auto& mask = m["mask"].items();
            for (std::size_t i = 0; i + 2 < pos.size(); i += 3) {
                Vertex v;
                v.position = {pos[i].asFloat(), pos[i + 1].asFloat(), pos[i + 2].asFloat()};
                if (i + 2 < nrm.size()) v.normal = {nrm[i].asFloat(), nrm[i + 1].asFloat(), nrm[i + 2].asFloat()};
                if (i / 3 < mask.size()) v.mask = std::clamp(mask[i / 3].asFloat(), 0.0f, 1.0f);
                b.mesh.vertices.push_back(v);
            }
            for (const Json& i : m["indices"].items()) {
                const int k = i.asInt(-1);
                if (k < 0 || static_cast<std::size_t>(k) >= b.mesh.vertices.size()) {
                    setError(error, "Corrupt mesh in body " + jb["name"].asString());
                    return false;
                }
                b.mesh.indices.push_back(static_cast<std::uint32_t>(k));
            }
            b.mesh.indices.resize(b.mesh.indices.size() / 3 * 3);
            const auto& e = m["edges"].items();
            for (std::size_t i = 0; i + 2 < e.size(); i += 3)
                b.mesh.edgeVertices.push_back({e[i].asFloat(), e[i + 1].asFloat(), e[i + 2].asFloat()});
            // Polypaint and polygroups (either encoding); ones that don't fit the mesh are dropped.
            std::vector<std::uint32_t> colors, groups;
            std::vector<unsigned char> raw;
            if (m["colors_b64"].isString() && unbase64(m["colors_b64"].asString(), raw))
                for (std::size_t i = 0; i + 4 <= raw.size(); i += 4) colors.push_back(getU32(&raw[i]));
            for (const Json& c : m["colors"].items()) colors.push_back(static_cast<std::uint32_t>(c.asNumber(0)));
            raw.clear();
            if (m["groups_b64"].isString() && unbase64(m["groups_b64"].asString(), raw))
                for (std::size_t i = 0; i + 4 <= raw.size(); i += 4) groups.push_back(getU32(&raw[i]));
            for (const Json& g : m["groups"].items()) groups.push_back(static_cast<std::uint32_t>(g.asNumber(0)));
            if (colors.size() == b.mesh.vertices.size())
                for (std::size_t i = 0; i < colors.size(); ++i) b.mesh.vertices[i].color = colors[i];
            if (groups.size() * 3 == b.mesh.indices.size()) b.mesh.groups = std::move(groups);
            {
                std::vector<std::uint32_t> qv;
                raw.clear();
                if (m["quads_b64"].isString() && unbase64(m["quads_b64"].asString(), raw))
                    for (std::size_t i = 0; i + 4 <= raw.size(); i += 4) qv.push_back(getU32(&raw[i]));
                for (const Json& x : m["quads"].items()) qv.push_back(static_cast<std::uint32_t>(x.asNumber(0)));
                bool fine = qv.size() % 4 == 0 && qv.size() / 4 * 6 == b.mesh.indices.size();
                for (std::uint32_t v : qv) fine = fine && v < b.mesh.vertices.size();
                if (fine)
                    for (std::size_t i = 0; i + 3 < qv.size(); i += 4) b.mesh.quads.push_back({qv[i], qv[i + 1], qv[i + 2], qv[i + 3]});
            }
            {
                std::vector<Vec2> uv;
                raw.clear();
                if (m["uvs_b64"].isString() && unbase64(m["uvs_b64"].asString(), raw))
                    for (std::size_t i = 0; i + 8 <= raw.size(); i += 8) uv.push_back({getF32(&raw[i]), getF32(&raw[i + 4])});
                const auto& ju = m["uvs"].items();
                for (std::size_t i = 0; i + 1 < ju.size(); i += 2) uv.push_back({ju[i].asFloat(0), ju[i + 1].asFloat(0)});
                if (uv.size() == b.mesh.indices.size()) b.mesh.uvs = std::move(uv);
            }
            for (const Json& jl : m["layers"].items()) {
                SculptLayer l;
                l.name = jl["name"].asString("Layer");
                l.strength = jl["strength"].asFloat(1.0f), l.saved = jl["saved"].asFloat(1.0f);
                raw.clear();
                if (jl["offsets_b64"].isString() && unbase64(jl["offsets_b64"].asString(), raw))
                    for (std::size_t i = 0; i + 12 <= raw.size(); i += 12) l.offset.push_back({getF32(&raw[i]), getF32(&raw[i + 4]), getF32(&raw[i + 8])});
                const auto& o = jl["offsets"].items();
                for (std::size_t i = 0; i + 2 < o.size(); i += 3) l.offset.push_back({o[i].asFloat(), o[i + 1].asFloat(), o[i + 2].asFloat()});
                if (l.offset.size() == b.mesh.vertices.size()) b.mesh.layers.push_back(std::move(l));
            }
            b.mesh.recordLayer = std::clamp(m["record_layer"].asInt(-1), -1, static_cast<int>(b.mesh.layers.size()) - 1);
            b.name = jb["name"].asString("Mesh");
            b.kind = "mesh";
            if (jb["brep_step"].isString()) {
                const std::vector<StepPart> parts = readStep(jb["brep_step"].asString());
                if (parts.size() == 1 && parts[0].brep) {
                    b.brep = parts[0].brep;
                    b.brepKey = meshKey(b.mesh);
                }
            }
        }
        const Vec3 rotation = jsonVec3(jb["rotation"]);
        if (isPrimitiveKind(kind)) { // meshes are stored already rotated and scaled
            if (rotation.x != 0.0f || rotation.y != 0.0f || rotation.z != 0.0f) {
                b.rotation = rotation;
                b.rebuild();
            }
            b.setScale(jb["scale"].asFloat(1.0f));
        } else {
            b.scale = std::max(jb["scale"].asFloat(1.0f), 1e-6f);
            b.rotation = rotation;
        }
        b.component = jb["component"].asString();
        b.attachedTo = jb["attached_to"].asString();
        b.assembly = jb["assembly"].asString();
        if (const Json& jl = jb["link"]; jl.isObject() && !jl["file"].asString().empty()) {
            fs::path f = jl["file"].asString();
            if (f.is_relative()) {
                // From the project's real folder (as links are written); a file written relative
                // to the folder as it was spelled, through a link, from that.
                const fs::path spelled = fs::absolute(fs::path(path)).parent_path();
                std::error_code ec;
                const fs::path real = fs::weakly_canonical(spelled, ec);
                fs::path resolved = ((ec ? spelled : real) / f).lexically_normal();
                if (std::error_code e2; !fs::exists(resolved, e2) && fs::exists((spelled / f).lexically_normal(), e2)) resolved = (spelled / f).lexically_normal();
                f = resolved;
            }
            b.link.file = f.lexically_normal().string();
            b.link.body = jl["body"].asString();
            b.link.partOffset = jsonVec3(jl["part_offset"]);
            b.link.partRotation = jsonVec3(jl["part_rotation"]);
            b.link.vertices = static_cast<std::size_t>(std::max(jl["vertices"].asNumber(), 0.0));
            b.link.stamp = std::strtoull(jl["stamp"].asString().c_str(), nullptr, 10);
        }
        b.field = jb["field"].asString();
        if (const Json& jm = jb["multires"]; jm.isObject()) { // levels that don't fit together are left out
            auto md = std::make_shared<MultiresData>();
            std::vector<unsigned char> raw;
            MultiresLevel cage;
            if (unbase64(jm["cage_triangles_b64"].asString(), raw))
                for (std::size_t i = 0; i + 12 <= raw.size(); i += 12) cage.t.push_back({getU32(&raw[i]), getU32(&raw[i + 4]), getU32(&raw[i + 8])});
            bool fits = !cage.t.empty();
            for (const Json& jl : jm["levels_b64"].items()) {
                raw.clear();
                if (!fits || !unbase64(jl.asString(), raw)) break;
                MultiresLevel l;
                if (md->levels.empty()) l.t = cage.t;
                else l = loopSubdivideLevel(md->levels.back());
                std::vector<Vec3> p;
                for (std::size_t i = 0; i + 12 <= raw.size(); i += 12) p.push_back({getF32(&raw[i]), getF32(&raw[i + 4]), getF32(&raw[i + 8])});
                if (md->levels.empty()) {
                    for (const auto& t : l.t)
                        for (std::uint32_t v : t) fits = fits && v < p.size();
                } else fits = fits && p.size() == l.p.size();
                if (!fits) break;
                l.p = std::move(p);
                md->levels.push_back(std::move(l));
            }
            md->current = jm["current"].asInt(0);
            if (fits && md->levels.size() >= 2 && md->current >= 0 && md->current < static_cast<int>(md->levels.size())) b.multires = std::move(md);
        }
        for (const Json& jh : jb["holes"].items()) {
            HoleNote& h = b.holes.emplace_back();
            h.at = jsonVec3(jh["at"]), h.dir = jsonVec3(jh["dir"], {0, 0, -1}), h.callout = jh["callout"].asString();
            h.threadDia = jh["thread"].asFloat(), h.pitch = jh["pitch"].asFloat(), h.threadDepth = jh["thread_depth"].asFloat();
            h.threadModelled = jh["modelled"].asBool(false);
            h.diameter = jh["diameter"].asFloat(), h.depth = jh["depth"].asFloat();
            h.cboreDia = jh["cbore"].asFloat(), h.cboreDepth = jh["cbore_depth"].asFloat();
            h.csinkDia = jh["csink"].asFloat(), h.csinkAngle = jh["csink_angle"].asFloat();
            h.size = jh["size"].asString();
        }
        b.offset = jsonVec3(jb["offset"]);
        b.color = jsonVec3(jb["color"], b.color);
        b.visible = jb["visible"].asBool(true);
        if (const Json& ph = jb["physics"]; ph.isObject()) {
            PhysicsProps& p = b.physics;
            p.enabled = true;
            p.fixed = ph["fixed"].asBool();
            p.mass = std::max(ph["mass"].asFloat(), 0.0f);
            p.density = std::max(ph["density"].asFloat(p.density), 1e-6f);
            p.velocity = jsonVec3(ph["velocity"]);
            p.spin = jsonVec3(ph["spin"]);
            p.charge = ph["charge"].asNumber();
            p.temperature = std::max(ph["temperature"].asFloat(p.temperature), 0.0f);
            p.specificHeat = std::max(ph["specific_heat"].asFloat(p.specificHeat), 1e-3f);
            p.restitution = std::clamp(ph["restitution"].asFloat(p.restitution), 0.0f, 1.0f);
            p.force = jsonVec3(ph["force"]);
            p.wave = waveKindFromKey(ph["wave"].asString("off")).value_or(WaveKind::Off);
            p.waveFrequency = std::max(ph["wave_frequency"].asFloat(p.waveFrequency), 1e-6f);
            p.waveSpeed = std::max(ph["wave_speed"].asFloat(p.waveSpeed), 1e-6f);
        }
        if (const Json& jm = jb["material"]; jm.isObject()) {
            MaterialProps& m = b.material;
            m.preset = jm["preset"].asString();
            m.metallic = std::clamp(jm["metallic"].asFloat(m.metallic), 0.0f, 1.0f);
            m.roughness = std::clamp(jm["roughness"].asFloat(m.roughness), 0.0f, 1.0f);
            m.transmission = std::clamp(jm["transmission"].asFloat(m.transmission), 0.0f, 1.0f);
            m.ior = std::clamp(jm["ior"].asFloat(m.ior), 1.0f, 4.0f);
            m.emission = std::clamp(jm["emission"].asFloat(m.emission), 0.0f, 1000.0f);
            m.colorMap = jm["color_map"].asString();
            m.normalMap = jm["normal_map"].asString();
        }
        if (const Json& jp = jb["properties"]; jp.isObject())
            for (const auto& [k, v] : jp.members())
                if (v.isString()) b.properties[k] = v.asString();
        loaded.add(std::move(b));
    }

    for (const Json& jc : (*doc)["sketch"].items()) {
        const auto kind = curveFromKey(jc["kind"].asString());
        if (!kind) continue;
        SketchCurve c;
        c.kind = *kind;
        c.plane = planeFromKey(jc["plane"].asString()).value_or(SketchPlane::Top);
        if (c.plane == SketchPlane::Custom && !frameFromJson(jc["frame"], c.frame)) continue; // nowhere to put it
        for (const Json& p : jc["points"].items()) c.pts.push_back({p[0].asFloat(), p[1].asFloat()});
        c.radius = jc["radius"].asFloat();
        c.name = jc["name"].asString();
        c.visible = jc["visible"].asBool(true);
        c.closed = jc["closed"].asBool(false);
        c.construction = jc["construction"].asBool(false);
        const std::size_t need = c.kind == CurveKind::Arc ? 3 : (c.kind == CurveKind::Ellipse || c.kind == CurveKind::Slot || c.kind == CurveKind::Spline) ? 2 : 1;
        if (c.pts.size() < need) continue;
        loaded.sketch().add(std::move(c));
    }
    constraintsFromJson((*doc)["constraints"], loaded.sketch());

    std::string missing;
    const fs::path projectDir = fs::absolute(fs::path(path)).parent_path();
    for (const Json& ji : (*doc)["images"].items()) {
        ImageRef img;
        img.name = ji["name"].asString("Image");
        img.path = ji["path"].asString();
        if (!img.path.empty() && fs::path(img.path).is_relative()) img.path = (projectDir / img.path).string();
        img.plane = planeFromKey(ji["plane"].asString()).value_or(SketchPlane::Top);
        img.center = {ji["center"][0].asFloat(), ji["center"][1].asFloat()};
        img.width = std::max(ji["width"].asFloat(100.0f), 0.01f);
        img.opacity = std::clamp(ji["opacity"].asFloat(0.6f), 0.0f, 1.0f);
        img.visible = ji["visible"].asBool(true);
        std::string imgErr;
        if (!loadImagePixels(img, &imgErr)) missing += (missing.empty() ? "" : ", ") + img.name;
        loaded.addImage(std::move(img));
    }
    if (warning) {
        *warning = missing.empty() ? "" : "Images not found: " + missing;
        if (!versionNote.empty()) *warning += (warning->empty() ? "" : "\n") + versionNote;
    }

    for (const Json& jm : (*doc)["motions"].items()) {
        Motion m;
        m.name = jm["name"].asString("Motion");
        m.kind = motionKindFromKey(jm["kind"].asString()).value_or(MotionKind::Circular);
        for (const Json& b : jm["bodies"].items()) m.bodies.push_back(b.asString());
        m.axis = jsonVec3(jm["axis"], {0, 0, 1});
        m.origin = jsonVec3(jm["origin"]);
        m.eachCentre = jm["each_centre"].asBool(false);
        m.speed = jm["speed"].asFloat(m.speed);
        m.range = std::max(jm["range"].asFloat(), 0.0f);
        m.mode = motionModeFromKey(jm["mode"].asString()).value_or(MotionMode::Continuous);
        m.start = std::max(jm["start"].asFloat(), 0.0f);
        m.enabled = jm["enabled"].asBool(true);
        m.follow = jm["follow"].asString();
        m.radius = std::max(jm["radius"].asFloat(), 0.0f);
        m.ratio = jm["ratio"].asFloat(1.0f);
        m.step = std::max(jm["step"].asFloat(), 0.0f);
        m.alongLoop = jm["along_loop"].asBool(false);
        for (const Json& k : jm["keys"].items())
            if (k.isArray() && k.size() >= 2) m.keys.push_back({std::max(k[0].asFloat(), 0.0f), k[1].asFloat()});
        m.sortKeys();
        m.keySpline = jm["key_curve"].asString() == "spline";
        m.keysAdvance = jm["keys_advance"].asBool();
        if (jm["limits"].isArray() && jm["limits"].size() >= 2) {
            m.hasLimits = true;
            m.lo = std::min(jm["limits"][0].asFloat(), jm["limits"][1].asFloat());
            m.hi = std::max(jm["limits"][0].asFloat(), jm["limits"][1].asFloat());
        }
        loaded.motions().push_back(std::move(m));
    }
    if (const Json& ph = (*doc)["physics"]; ph.isObject()) {
        PhysicsSettings& p = loaded.physics();
        p.enabled = ph["enabled"].asBool();
        p.gravity = gravityModeFromKey(ph["gravity"].asString("uniform")).value_or(GravityMode::Uniform);
        p.g = jsonVec3(ph["g"], p.g);
        p.bigG = std::max(ph["big_g"].asNumber(p.bigG), 0.0);
        p.floor = ph["floor"].asBool(true);
        p.friction = std::max(ph["friction"].asFloat(p.friction), 0.0f);
        p.air = ph["air"].asBool();
        p.wind = jsonVec3(ph["wind"]);
        p.electricField = jsonVec3(ph["electric_field"]);
        p.magneticField = jsonVec3(ph["magnetic_field"]);
        p.ambient = std::max(ph["ambient"].asFloat(p.ambient), 0.0f);
        p.convection = std::max(ph["convection"].asFloat(p.convection), 0.0f);
        p.conductance = std::max(ph["conductance"].asFloat(p.conductance), 0.0f);
        p.relativity = ph["relativity"].asBool();
        p.lightSpeed = std::max(ph["light_speed"].asNumber(p.lightSpeed), 1e-6);
        p.duration = std::clamp(ph["duration"].asFloat(p.duration), 0.01f, 600.0f);
    }
    st.simDuration = std::max((*doc)["simulation_duration"].asFloat(), 0.0f);
    if (const Json& cj = (*doc)["cam"]; cj.isObject()) camFromJson(cj, st.cam);
    if (const Json& lj = (*doc)["lathe"]; lj.isObject()) latheFromJson(lj, st.lathe);
    if (const Json& aj = (*doc)["assembly"]; aj.isObject()) assemblyFromJson(aj, st.assembly);
    if (const Json& sj = (*doc)["section"]; sj.isObject()) st.section = sectionFromJson(sj);
    if (const Json& rj = (*doc)["render"]; rj.isObject()) {
        RenderSettings& r = st.render;
        const std::string env = rj["environment"].asString(r.environment);
        if (std::find(renderEnvironments().begin(), renderEnvironments().end(), env) != renderEnvironments().end()) r.environment = env;
        if (rj["sun"].size() == 2) r.sunAzimuth = rj["sun"][0].asFloat(r.sunAzimuth), r.sunElevation = std::clamp(rj["sun"][1].asFloat(r.sunElevation), 1.0f, 90.0f);
        r.light = std::clamp(rj["light"].asFloat(r.light), 0.0f, 100.0f);
        r.exposure = std::clamp(rj["exposure"].asFloat(r.exposure), -10.0f, 10.0f);
        r.floor = rj["floor"].asBool(r.floor);
        if (rj["size"].size() == 2) r.width = std::clamp(rj["size"][0].asInt(r.width), 8, 8192), r.height = std::clamp(rj["size"][1].asInt(r.height), 8, 8192);
        r.samples = std::clamp(rj["samples"].asInt(r.samples), 1, 65536);
        r.bounces = std::clamp(rj["bounces"].asInt(r.bounces), 1, 64);
        r.denoise = rj["denoise"].asBool(r.denoise);
    }
    if (const Json& pj = (*doc)["product"]; pj.isObject()) {
        ProductData& pd = st.product;
        pd.revision = pj["revision"].asString(pd.revision);
        pd.state = parseReleaseState(pj["state"].asString()).value_or(ReleaseState::InWork);
        for (const auto& [k, v] : pj["properties"].members())
            if (v.isString()) pd.properties[k] = v.asString();
        for (const Json& h : pj["history"].items()) {
            RevisionRecord r;
            r.revision = h["revision"].asString();
            r.state = parseReleaseState(h["state"].asString()).value_or(ReleaseState::InWork);
            r.date = h["date"].asString();
            r.author = h["author"].asString();
            r.note = h["note"].asString();
            pd.history.push_back(std::move(r));
        }
        pd.vaultItem = pj["vault_item"].asString();
    }
    if (const Json& rj = (*doc)["robots"]; rj.isObject()) rigStateFromJson(rj, loaded.rigs());
    if (const Json& tj = (*doc)["terrain"]; tj.isObject()) terrainFromJson(tj, loaded.terrain());
    if (const Json& dj = (*doc)["drawing"]; dj.isObject()) drawingFromJson(dj, st.drawing);
    for (const Json& j : (*doc)["history"].items()) {
        HistoryStep step;
        step.line = j["line"].asString("");
        step.undoable = j["undoable"].asBool(true);
        step.millimetres = j["mm"].asBool(false);
        if (const auto u = parseUnitName(j["unit"].asString("mm"))) step.unit = *u;
        step.selectedBody = j["body"].asString("");
        step.selectedCurve = j["curve"].asString("");
        step.suppressed = j["suppressed"].asBool(false);
        if (!step.line.empty()) st.history.push_back(std::move(step));
    }
    st.historyComplete = (*doc)["history_complete"].asBool(true);
    st.rollback = std::clamp((*doc)["history_rollback"].asInt(-1), -1, static_cast<int>(st.history.size()));
    if (st.rollback == static_cast<int>(st.history.size())) st.rollback = -1;
    const Json& du = (*doc)["display_units"];
    st.units = UnitSettings{};
    if (const auto u = parseUnitName(du["unit"].asString("mm"))) st.units.unit = *u;
    st.units.precision = std::clamp(static_cast<int>(du["precision"].asNumber(-1)), -1, 10);
    st.units.style = du["style"].asString("decimal") == "fractional" ? LengthStyle::Fractional : LengthStyle::Decimal;
    st.units.denominator = std::clamp(static_cast<int>(du["denominator"].asNumber(64)), 2, 256);

    scene.assign(std::move(loaded));
    state = st;
    return true;
}

// ---------------------------------------------------------------------------
// Meshes

bool exportStl(const Scene& scene, const std::string& path, std::string* error) {
    std::string data(80, '\0');
    const char header[] = "SI CDM binary STL (mm)";
    std::memcpy(data.data(), header, sizeof(header) - 1);
    std::uint32_t count = 0;
    std::string tris;
    forEachWorldTriangle(scene, [&](const Body&, const Vec3& a, const Vec3& b, const Vec3& c) {
        const Vec3 n = normalize(cross(b - a, c - a));
        for (const Vec3& v : {n, a, b, c}) {
            appendF32(tris, v.x);
            appendF32(tris, v.y);
            appendF32(tris, v.z);
        }
        tris.append(2, '\0');
        ++count;
    });
    if (count == 0) {
        setError(error, "Nothing to export: no visible bodies");
        return false;
    }
    char cnt[4];
    std::memcpy(cnt, &count, 4);
    data.append(cnt, 4);
    data += tris;
    return writeFile(path, data, error);
}

bool exportObj(const Scene& scene, const std::string& path, std::string* error) {
    std::string out = "# SI CDM OBJ export (mm)\n";
    std::size_t base = 1, uvBase = 1, written = 0;
    char line[160];
    for (const Body& b : scene.bodies()) {
        if (!b.visible || b.mesh.indices.empty()) continue;
        std::string name = b.name;
        std::replace(name.begin(), name.end(), ' ', '_');
        out += "o " + name + "\n";
        // Polypaint as vertex colours after the position (read by ZBrush, Blender, MeshLab).
        const bool paint = std::any_of(b.mesh.vertices.begin(), b.mesh.vertices.end(), [](const Vertex& v) { return painted(v.color); });
        for (const Vertex& v : b.mesh.vertices) {
            const Vec3 p = v.position + b.offset;
            if (paint) {
                const Vec3 c = painted(v.color) ? unpackColor(v.color) : b.color;
                std::snprintf(line, sizeof(line), "v %.6f %.6f %.6f %.4f %.4f %.4f\n", static_cast<double>(p.x), static_cast<double>(p.y),
                              static_cast<double>(p.z), static_cast<double>(c.x), static_cast<double>(c.y), static_cast<double>(c.z));
            } else {
                std::snprintf(line, sizeof(line), "v %.6f %.6f %.6f\n", static_cast<double>(p.x), static_cast<double>(p.y),
                              static_cast<double>(p.z));
            }
            out += line;
        }
        for (const Vertex& v : b.mesh.vertices) {
            std::snprintf(line, sizeof(line), "vn %.6f %.6f %.6f\n", static_cast<double>(v.normal.x),
                          static_cast<double>(v.normal.y), static_cast<double>(v.normal.z));
            out += line;
        }
        const auto& idx = b.mesh.indices;
        // Texture coordinates: one "vt" per triangle corner, numbered from uvBase.
        const bool mapped = b.mesh.hasUvs();
        if (mapped)
            for (const Vec2& t : b.mesh.uvs) {
                std::snprintf(line, sizeof(line), "vt %.6f %.6f\n", static_cast<double>(t.x), static_cast<double>(t.y));
                out += line;
            }
        auto corner = [&](std::size_t v, std::size_t c) { // "v/vt/vn" of vertex v at triangle corner c
            if (mapped) std::snprintf(line, sizeof(line), " %zu/%zu/%zu", base + v, uvBase + c, base + v);
            else std::snprintf(line, sizeof(line), " %zu//%zu", base + v, base + v);
            return std::string(line);
        };
        if (b.mesh.hasQuads()) { // a quad mesh: its quads as they are
            for (std::size_t qi = 0; qi < b.mesh.quads.size(); ++qi) {
                const auto& q = b.mesh.quads[qi];
                std::string f = "f";
                for (std::uint32_t v : q) { // the corner of one of the quad's triangles at v
                    std::size_t c = qi * 6;
                    while (c + 1 < qi * 6 + 6 && idx[c] != v) ++c;
                    f += corner(v, c);
                }
                out += f + "\n";
            }
            base += b.mesh.vertices.size();
            if (mapped) uvBase += b.mesh.uvs.size();
            ++written;
            continue;
        }
        const bool grouped = b.mesh.hasGroups();
        std::uint32_t lastGroup = 0xFFFFFFFFu;
        for (std::size_t i = 0; i + 2 < idx.size(); i += 3) {
            if (grouped && b.mesh.groups[i / 3] != lastGroup) { // polygroups as OBJ groups (ZBrush reads them back as polygroups)
                lastGroup = b.mesh.groups[i / 3];
                out += "g " + name + "_" + std::to_string(lastGroup) + "\n";
            }
            out += "f" + corner(idx[i], i) + corner(idx[i + 1], i + 1) + corner(idx[i + 2], i + 2) + "\n";
        }
        base += b.mesh.vertices.size();
        if (mapped) uvBase += b.mesh.uvs.size();
        ++written;
    }
    if (written == 0) {
        setError(error, "Nothing to export: no visible bodies");
        return false;
    }
    return writeFile(path, out, error);
}

std::optional<Body> importMesh(const std::string& path, std::string* error) {
    const FileKind kind = fileKindOf(path);
    const auto data = readFile(path, error);
    if (!data) return std::nullopt;
    std::vector<Vec3> tri;
    std::vector<Vec3> triColor;          // OBJ: per corner (negative: none)
    std::vector<std::uint32_t> triGroup; // OBJ: per triangle
    std::vector<Vec2> triTex;            // OBJ: per corner, when every face has them
    bool allTextured = true;
    std::size_t groupCount = 0;

    if (kind == FileKind::Stl) {
        const std::string& d = *data;
        std::uint32_t n = 0;
        if (d.size() >= 84) std::memcpy(&n, d.data() + 80, 4);
        const bool binary = d.size() >= 84 && 84ull + 50ull * n == d.size();
        if (binary) {
            tri.reserve(static_cast<std::size_t>(n) * 3);
            for (std::uint32_t t = 0; t < n; ++t) {
                const char* rec = d.data() + 84 + 50ull * t + 12; // skip normal
                for (int k = 0; k < 3; ++k) {
                    float v[3];
                    std::memcpy(v, rec + 12 * k, 12);
                    tri.push_back({v[0], v[1], v[2]});
                }
            }
        } else {
            std::istringstream in(d);
            std::string word;
            while (in >> word) {
                if (word != "vertex") continue;
                Vec3 v;
                if (!(in >> v.x >> v.y >> v.z)) break;
                tri.push_back(v);
            }
            tri.resize(tri.size() / 3 * 3);
        }
    } else if (kind == FileKind::Obj) {
        std::vector<Vec3> pos, col;
        std::vector<Vec2> tex;
        std::istringstream in(*data);
        std::string line;
        std::map<std::string, std::uint32_t> groupIds;
        std::uint32_t group = 0;
        while (std::getline(in, line)) {
            std::istringstream ls(line);
            std::string tag;
            ls >> tag;
            if (tag == "v") {
                Vec3 v, c{-1, -1, -1};
                if (ls >> v.x >> v.y >> v.z) {
                    pos.push_back(v);
                    if (ls >> c.x >> c.y >> c.z) col.resize(pos.size(), Vec3{-1, -1, -1}), col.back() = c; // a vertex colour
                }
            } else if (tag == "vt") {
                Vec2 t;
                if (ls >> t.x >> t.y) tex.push_back(t);
            } else if (tag == "g") {
                std::string name;
                std::getline(ls, name);
                group = groupIds.try_emplace(name, static_cast<std::uint32_t>(groupIds.size())).first->second;
            } else if (tag == "f") {
                std::vector<Vec3> poly;
                std::vector<Vec3> polyCol;
                std::vector<Vec2> polyTex;
                std::string tok;
                bool textured = true;
                while (ls >> tok) {
                    char* end = nullptr;
                    const long k = std::strtol(tok.c_str(), &end, 10); // "a/b/c" -> a (b: the texture coordinate)
                    const long i = k < 0 ? static_cast<long>(pos.size()) + k : k - 1;
                    if (i < 0 || static_cast<std::size_t>(i) >= pos.size()) {
                        setError(error, "Bad face index in " + path);
                        return std::nullopt;
                    }
                    long ti = -1;
                    if (end && *end == '/' && end[1] != '/' && end[1] != '\0') {
                        const long kt = std::strtol(end + 1, nullptr, 10);
                        ti = kt < 0 ? static_cast<long>(tex.size()) + kt : kt - 1;
                    }
                    if (ti < 0 || static_cast<std::size_t>(ti) >= tex.size()) textured = false;
                    else polyTex.push_back(tex[static_cast<std::size_t>(ti)]);
                    poly.push_back(pos[static_cast<std::size_t>(i)]);
                    polyCol.push_back(static_cast<std::size_t>(i) < col.size() ? col[static_cast<std::size_t>(i)] : Vec3{-1, -1, -1});
                }
                if (!textured) allTextured = false;
                for (std::size_t j = 1; j + 1 < poly.size(); ++j) { // fan
                    tri.push_back(poly[0]);
                    tri.push_back(poly[j]);
                    tri.push_back(poly[j + 1]);
                    if (textured) triTex.insert(triTex.end(), {polyTex[0], polyTex[j], polyTex[j + 1]});
                    triColor.push_back(polyCol[0]);
                    triColor.push_back(polyCol[j]);
                    triColor.push_back(polyCol[j + 1]);
                    triGroup.push_back(group);
                }
            }
        }
        groupCount = groupIds.size();
    } else {
        setError(error, "Unsupported mesh format: " + lowerExt(path));
        return std::nullopt;
    }

    if (tri.empty()) {
        setError(error, "No triangles found in " + path);
        return std::nullopt;
    }
    MeshData mesh = meshFromTriangles(tri); // a vertex per corner, in order
    if (triColor.size() == mesh.vertices.size())
        for (std::size_t i = 0; i < triColor.size(); ++i)
            if (triColor[i].x >= 0) mesh.vertices[i].color = packColor(triColor[i]);
    if (groupCount > 1 && triGroup.size() * 3 == mesh.indices.size()) mesh.groups = std::move(triGroup);
    if (allTextured && triTex.size() == mesh.indices.size()) mesh.uvs = std::move(triTex);
    return meshBody(stemOf(path), std::move(mesh));
}

// ---------------------------------------------------------------------------
// 2D drawings

int exportDxf(const Scene& scene, SketchPlane plane, const std::string& path, std::string* error, const PlaneFrame& custom) {
    std::string out;
    dxfPair(out, 0, "SECTION");
    dxfPair(out, 2, "HEADER");
    dxfPair(out, 9, "$ACADVER");
    dxfPair(out, 1, "AC1009");
    dxfPair(out, 9, "$INSUNITS");
    dxfPair(out, 70, "4"); // millimetres
    dxfPair(out, 0, "ENDSEC");
    dxfPair(out, 0, "SECTION");
    dxfPair(out, 2, "ENTITIES");
    const std::string layer = std::string("SKETCH_") + planeKey(plane);
    int count = 0;
    for (const SketchCurve& c : scene.sketch().curves()) {
        if (!onPlane(c, plane, custom) || !c.visible || c.pts.empty()) continue;
        const std::string curveLayer = c.construction ? layer + "_CONSTRUCTION" : layer;
        switch (c.kind) {
        case CurveKind::Point:
            dxfPair(out, 0, "POINT");
            dxfPair(out, 8, curveLayer);
            dxfPoint(out, 10, c.pts[0]);
            break;
        case CurveKind::Line:
            if (c.pts.size() < 2) continue;
            dxfPair(out, 0, "LINE");
            dxfPair(out, 8, curveLayer);
            dxfPoint(out, 10, c.pts[0]);
            dxfPoint(out, 11, c.pts[1]);
            break;
        case CurveKind::Circle:
            dxfPair(out, 0, "CIRCLE");
            dxfPair(out, 8, curveLayer);
            dxfPoint(out, 10, c.pts[0]);
            dxfPair(out, 40, num(c.radius));
            break;
        case CurveKind::Polyline: dxfPolyline(out, curveLayer, c.pts, false); break;
        case CurveKind::Rectangle: {
            if (c.pts.size() < 2) continue;
            const Vec2 a = c.pts[0], b = c.pts[1];
            dxfPolyline(out, curveLayer, {a, {b.x, a.y}, b, {a.x, b.y}}, true);
            break;
        }
        case CurveKind::Arc: {
            if (c.pts.size() < 3) continue;
            float a0, a1;
            c.arcAngles(a0, a1);
            dxfArc(out, curveLayer, c.pts[0], std::hypot(c.pts[1].x - c.pts[0].x, c.pts[1].y - c.pts[0].y), a0, a1);
            break;
        }
        case CurveKind::Slot: { // two lines and two half circles
            if (c.pts.size() < 2) continue;
            const Vec2 a = c.pts[0], b = c.pts[1];
            const float ang = std::atan2(b.y - a.y, b.x - a.x), r = c.radius;
            const Vec2 n{-std::sin(ang) * r, std::cos(ang) * r};
            dxfArc(out, curveLayer, b, r, ang - 1.5707963f, ang + 1.5707963f);
            dxfArc(out, curveLayer, a, r, ang + 1.5707963f, ang + 4.7123890f);
            for (const auto& [p, q] : {std::pair{Vec2{b.x + n.x, b.y + n.y}, Vec2{a.x + n.x, a.y + n.y}}, std::pair{Vec2{a.x - n.x, a.y - n.y}, Vec2{b.x - n.x, b.y - n.y}}}) {
                dxfPair(out, 0, "LINE");
                dxfPair(out, 8, curveLayer);
                dxfPoint(out, 10, p);
                dxfPoint(out, 11, q);
            }
            break;
        }
        case CurveKind::Spline:
        case CurveKind::Ellipse: { // R12 has neither: a fine polyline
            std::vector<Vec2> s = c.planeStrip(0.01f);
            const bool loop = c.isClosed();
            if (loop && s.size() > 2) s.pop_back();
            dxfPolyline(out, curveLayer, s, loop);
            break;
        }
        }
        ++count;
    }
    dxfPair(out, 0, "ENDSEC");
    dxfPair(out, 0, "EOF");
    if (count == 0) {
        setError(error, std::string("No sketch curves on the ") + planeKey(plane) + " plane to export");
        return -1;
    }
    return writeFile(path, out, error) ? count : -1;
}

int exportSvg(const Scene& scene, SketchPlane plane, const std::string& path, std::string* error, const PlaneFrame& custom) {
    AABB b;
    int count = 0;
    for (const SketchCurve& c : scene.sketch().curves()) {
        if (!onPlane(c, plane, custom) || !c.visible || c.pts.empty()) continue;
        ++count;
        for (const Vec2& p : c.pts) b.expand(Vec3{p.x, p.y, 0});
        for (const Vec2& p : c.planeStrip(0.1f)) b.expand(Vec3{p.x, p.y, 0});
        if (c.kind == CurveKind::Circle) {
            b.expand(Vec3{c.pts[0].x - c.radius, c.pts[0].y - c.radius, 0});
            b.expand(Vec3{c.pts[0].x + c.radius, c.pts[0].y + c.radius, 0});
        }
    }
    if (count == 0) {
        setError(error, std::string("No sketch curves on the ") + planeKey(plane) + " plane to export");
        return -1;
    }
    const float m = 5.0f;
    const float x0 = b.min.x - m, y0 = b.min.y - m;
    const float w = b.max.x - b.min.x + 2 * m, h = b.max.y - b.min.y + 2 * m;
    char buf[256];
    std::string out = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    std::snprintf(buf, sizeof(buf),
                  "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"%.3fmm\" height=\"%.3fmm\" viewBox=\"%.3f %.3f %.3f %.3f\">\n",
                  static_cast<double>(w), static_cast<double>(h), static_cast<double>(x0), static_cast<double>(-y0 - h),
                  static_cast<double>(w), static_cast<double>(h));
    out += buf;
    // Sketch v points up; SVG y points down, so flip.
    out += "<g transform=\"scale(1,-1)\" fill=\"none\" stroke=\"#000\" stroke-width=\"0.25\" stroke-linecap=\"round\">\n";
    auto pts = [](const std::vector<Vec2>& v) {
        std::string s;
        char p[64];
        for (const Vec2& q : v) {
            std::snprintf(p, sizeof(p), "%.4f,%.4f ", static_cast<double>(q.x), static_cast<double>(q.y));
            s += p;
        }
        return s;
    };
    for (const SketchCurve& c : scene.sketch().curves()) {
        if (!onPlane(c, plane, custom) || !c.visible || c.pts.empty()) continue;
        switch (c.kind) {
        case CurveKind::Point:
            std::snprintf(buf, sizeof(buf), "<circle cx=\"%.4f\" cy=\"%.4f\" r=\"0.5\" fill=\"#000\"/>\n",
                          static_cast<double>(c.pts[0].x), static_cast<double>(c.pts[0].y));
            break;
        case CurveKind::Line:
            if (c.pts.size() < 2) continue;
            std::snprintf(buf, sizeof(buf), "<line x1=\"%.4f\" y1=\"%.4f\" x2=\"%.4f\" y2=\"%.4f\"/>\n",
                          static_cast<double>(c.pts[0].x), static_cast<double>(c.pts[0].y),
                          static_cast<double>(c.pts[1].x), static_cast<double>(c.pts[1].y));
            break;
        case CurveKind::Circle:
            std::snprintf(buf, sizeof(buf), "<circle cx=\"%.4f\" cy=\"%.4f\" r=\"%.4f\"/>\n",
                          static_cast<double>(c.pts[0].x), static_cast<double>(c.pts[0].y),
                          static_cast<double>(c.radius));
            break;
        case CurveKind::Polyline: out += "<polyline points=\"" + pts(c.pts) + "\"/>\n"; continue;
        case CurveKind::Rectangle: {
            if (c.pts.size() < 2) continue;
            const Vec2 a = c.pts[0], q = c.pts[1];
            out += "<polygon points=\"" + pts({a, {q.x, a.y}, q, {a.x, q.y}}) + "\"/>\n";
            continue;
        }
        case CurveKind::Arc:
        case CurveKind::Spline:
        case CurveKind::Ellipse:
        case CurveKind::Slot:
            out += "<polyline points=\"" + pts(c.planeStrip(0.01f)) + "\"" + (c.construction ? " stroke-dasharray=\"2,1\"" : "") + "/>\n";
            continue;
        }
        out += buf;
    }
    out += "</g>\n</svg>\n";
    return writeFile(path, out, error) ? count : -1;
}

int importDxf(Scene& scene, SketchPlane plane, const std::string& path, std::string* error, const PlaneFrame& custom) {
    const auto data = readFile(path, error);
    if (!data) return -1;
    std::vector<DxfPair> pairs;
    {
        std::istringstream in(*data);
        std::string codeLine, valueLine;
        while (std::getline(in, codeLine) && std::getline(in, valueLine)) {
            char* end = nullptr;
            const std::string c = trimmed(codeLine);
            const long code = std::strtol(c.c_str(), &end, 10);
            if (c.empty() || *end != '\0') {
                setError(error, "Not an ASCII DXF file: " + path);
                return -1;
            }
            pairs.push_back({static_cast<int>(code), trimmed(valueLine)});
        }
    }
    auto f = [](const std::string& s) { return static_cast<float>(std::strtod(s.c_str(), nullptr)); };

    int added = 0;
    auto addCurve = [&](CurveKind k, std::vector<Vec2> pts, float r = 0.0f) {
        if (pts.empty()) return;
        SketchCurve c;
        c.kind = k;
        c.plane = plane;
        c.frame = custom;
        c.pts = std::move(pts);
        c.radius = r;
        scene.sketch().add(std::move(c));
        ++added;
    };

    // Walk entities: each starts at a code-0 pair.
    std::size_t i = 0;
    bool inEntities = false;
    while (i < pairs.size()) {
        if (pairs[i].code != 0) {
            ++i;
            continue;
        }
        const std::string type = pairs[i].value;
        std::size_t j = i + 1;
        while (j < pairs.size() && pairs[j].code != 0) ++j;
        auto group = [&](int code, std::size_t nth = 0) -> const std::string* {
            for (std::size_t k = i + 1; k < j; ++k)
                if (pairs[k].code == code && nth-- == 0) return &pairs[k].value;
            return nullptr;
        };
        auto g = [&](int code) { const std::string* v = group(code); return v ? f(*v) : 0.0f; };
        if (type == "SECTION") {
            const std::string* name = group(2);
            inEntities = name && *name == "ENTITIES";
        } else if (type == "ENDSEC") {
            inEntities = false;
        } else if (inEntities) {
            if (type == "LINE") {
                addCurve(CurveKind::Line, {{g(10), g(20)}, {g(11), g(21)}});
            } else if (type == "CIRCLE") {
                if (g(40) > 0) addCurve(CurveKind::Circle, {{g(10), g(20)}}, g(40));
            } else if (type == "ARC") {
                if (g(40) > 0) {
                    const Vec2 c{g(10), g(20)};
                    const float r = g(40), a0 = radians(g(50)), a1 = radians(g(51));
                    addCurve(CurveKind::Arc, {c, {c.x + r * std::cos(a0), c.y + r * std::sin(a0)}, {c.x + r * std::cos(a1), c.y + r * std::sin(a1)}}, r);
                }
            } else if (type == "SPLINE") {
                // Its fit points when it has them, else its control points (a close match).
                std::vector<Vec2> pts;
                for (int code : {11, 10}) {
                    for (std::size_t n = 0;; ++n) {
                        const std::string* x = group(code, n);
                        const std::string* y = group(code + 10, n);
                        if (!x || !y) break;
                        pts.push_back({f(*x), f(*y)});
                    }
                    if (pts.size() >= 2) break;
                    pts.clear();
                }
                const bool closedFlag = (static_cast<int>(g(70)) & 1) != 0;
                if (closedFlag && pts.size() > 2 && std::hypot(pts.front().x - pts.back().x, pts.front().y - pts.back().y) < 1e-6f) pts.pop_back();
                if (pts.size() >= 2) {
                    SketchCurve c;
                    c.kind = CurveKind::Spline;
                    c.plane = plane;
                    c.frame = custom;
                    c.pts = std::move(pts);
                    c.closed = closedFlag;
                    scene.sketch().add(std::move(c));
                    ++added;
                }
            } else if (type == "ELLIPSE") {
                const Vec2 c{g(10), g(20)}, m{g(11), g(21)};
                const float ratio = g(40), t0 = g(41), t1 = group(42) ? g(42) : 6.2831853f;
                const float rx = std::hypot(m.x, m.y);
                if (rx > 0 && ratio > 0) {
                    if (std::fabs(t1 - t0 - 6.2831853f) < 1e-4f || (t0 == 0 && t1 == 0)) {
                        addCurve(CurveKind::Ellipse, {c, {c.x + m.x, c.y + m.y}}, rx * ratio);
                    } else { // part of one: a spline through it
                        std::vector<Vec2> pts;
                        float e1 = t1;
                        while (e1 <= t0) e1 += 6.2831853f;
                        const Vec2 n{-m.y * ratio, m.x * ratio};
                        for (int k = 0; k <= 24; ++k) {
                            const float t = t0 + (e1 - t0) * static_cast<float>(k) / 24.0f;
                            pts.push_back({c.x + m.x * std::cos(t) + n.x * std::sin(t), c.y + m.y * std::cos(t) + n.y * std::sin(t)});
                        }
                        addCurve(CurveKind::Spline, std::move(pts));
                    }
                }
            } else if (type == "POINT") {
                addCurve(CurveKind::Point, {{g(10), g(20)}});
            } else if (type == "LWPOLYLINE") {
                std::vector<Vec2> pts;
                for (std::size_t n = 0;; ++n) {
                    const std::string* x = group(10, n);
                    const std::string* y = group(20, n);
                    if (!x || !y) break;
                    pts.push_back({f(*x), f(*y)});
                }
                if ((static_cast<int>(g(70)) & 1) && !pts.empty()) pts.push_back(pts.front());
                if (pts.size() >= 2) addCurve(CurveKind::Polyline, std::move(pts));
            } else if (type == "POLYLINE") {
                const bool closed = static_cast<int>(g(70)) & 1;
                std::vector<Vec2> pts;
                std::size_t k = j;
                while (k < pairs.size() && pairs[k].code == 0 && pairs[k].value == "VERTEX") {
                    std::size_t e = k + 1;
                    Vec2 p{};
                    while (e < pairs.size() && pairs[e].code != 0) {
                        if (pairs[e].code == 10) p.x = f(pairs[e].value);
                        if (pairs[e].code == 20) p.y = f(pairs[e].value);
                        ++e;
                    }
                    pts.push_back(p);
                    k = e;
                }
                if (closed && !pts.empty()) pts.push_back(pts.front());
                if (pts.size() >= 2) addCurve(CurveKind::Polyline, std::move(pts));
                j = k;
            }
        }
        i = j;
    }
    if (added == 0) setError(error, "No supported entities (LINE, CIRCLE, ARC, ELLIPSE, SPLINE, POINT, POLYLINE) in " + path);
    return added;
}

// ---------------------------------------------------------------------------
// Images

bool loadImagePixels(ImageRef& img, std::string* error) {
    int w = 0, h = 0, n = 0;
    unsigned char* px = stbi_load(img.path.c_str(), &w, &h, &n, 4);
    if (!px) {
        const char* why = stbi_failure_reason();
        setError(error, "Cannot read image " + img.path + (why ? std::string(": ") + why : ""));
        img.pixels.clear();
        img.pixelWidth = img.pixelHeight = 0;
        return false;
    }
    img.pixelWidth = w;
    img.pixelHeight = h;
    img.pixels.assign(px, px + static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 4);
    stbi_image_free(px);
    return true;
}

std::optional<ImageRef> importImage(const std::string& path, SketchPlane plane, Vec2 center, std::string* error) {
    ImageRef img;
    img.path = fs::absolute(path).string();
    img.name = fs::path(path).filename().string();
    img.plane = plane;
    img.center = center;
    if (!loadImagePixels(img, error)) return std::nullopt;
    img.width = std::min(static_cast<float>(img.pixelWidth) * 0.25f, 400.0f);
    return img;
}

} // namespace sicdm

// Mesh exchange formats in (MeshImport.hpp).

#include "core/MeshImport.hpp"

#include "core/Geometry.hpp"
#include "core/Json.hpp"
#include "core/Xml.hpp"
#include "core/Zip.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <new>
#include <stdexcept>
#include <set>
#include <sstream>
#include <string_view>

namespace sicdm {

namespace fs = std::filesystem;

namespace {

using Tri = std::array<std::uint32_t, 3>;

bool fail(std::string* why, const std::string& w) {
    if (why) *why = w;
    return false;
}

std::string lowerOf(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

std::optional<std::string> slurp(const std::string& path, std::string* why) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        fail(why, "Cannot open " + path);
        return std::nullopt;
    }
    std::ostringstream s;
    s << f.rdbuf();
    return s.str();
}

// A gzip stream's contents (VRML files are often gzipped); the bytes as they are otherwise.
bool gunzipIfZipped(std::string& bytes, std::string* why) {
    if (bytes.size() < 18 || static_cast<unsigned char>(bytes[0]) != 0x1f || static_cast<unsigned char>(bytes[1]) != 0x8b) return true;
    if (bytes[2] != 8) return fail(why, "a gzip stream packed a way this reader doesn't know");
    const unsigned flags = static_cast<unsigned char>(bytes[3]);
    std::size_t at = 10;
    if (flags & 4) { // extra field
        if (at + 2 > bytes.size()) return fail(why, "a damaged gzip stream");
        at += 2 + (static_cast<unsigned char>(bytes[at]) | static_cast<unsigned char>(bytes[at + 1]) << 8);
    }
    for (unsigned f : {8u, 16u}) // file name, comment
        if (flags & f) {
            while (at < bytes.size() && bytes[at]) ++at;
            ++at;
        }
    if (flags & 2) at += 2; // header CRC
    if (at >= bytes.size()) return fail(why, "a damaged gzip stream");
    const std::size_t size = static_cast<unsigned char>(bytes[bytes.size() - 4]) | static_cast<std::size_t>(static_cast<unsigned char>(bytes[bytes.size() - 3])) << 8 |
                             static_cast<std::size_t>(static_cast<unsigned char>(bytes[bytes.size() - 2])) << 16 |
                             static_cast<std::size_t>(static_cast<unsigned char>(bytes[bytes.size() - 1])) << 24;
    std::string out;
    out.reserve(std::min(size, bytes.size() * 1032 + 4096)); // (deflate packs at most ~1032:1)
    std::string w;
    if (!inflateRaw(reinterpret_cast<const std::uint8_t*>(bytes.data()) + at, bytes.size() - at, out, std::size_t{1} << 31, &w))
        return fail(why, "a damaged gzip stream: " + w);
    bytes = std::move(out);
    return true;
}

// An affine map: x' = A x + t (millimetres or file units, as the caller keeps them).
struct Affine {
    double a[3][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}};
    Vec3 apply(const Vec3& p) const {
        const double x = p.x, y = p.y, z = p.z;
        return {static_cast<float>(a[0][0] * x + a[0][1] * y + a[0][2] * z + a[0][3]), static_cast<float>(a[1][0] * x + a[1][1] * y + a[1][2] * z + a[1][3]),
                static_cast<float>(a[2][0] * x + a[2][1] * y + a[2][2] * z + a[2][3])};
    }
    // This first, then `next`.
    Affine then(const Affine& next) const {
        Affine r;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 4; ++j) {
                double v = j == 3 ? next.a[i][3] : 0.0;
                for (int k = 0; k < 3; ++k) v += next.a[i][k] * a[k][j];
                r.a[i][j] = v;
            }
        }
        return r;
    }
    double det() const {
        return a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    }
    static Affine scale(double s) {
        Affine m;
        for (int i = 0; i < 3; ++i) m.a[i][i] = s;
        return m;
    }
};

// Turned about a unit axis by an angle (radians).
Affine rotation(double x, double y, double z, double ang) {
    const double l = std::sqrt(x * x + y * y + z * z);
    Affine m;
    if (l < 1e-12) return m;
    x /= l, y /= l, z /= l;
    const double c = std::cos(ang), s = std::sin(ang), t = 1 - c;
    const double r[3][3] = {{t * x * x + c, t * x * y - s * z, t * x * z + s * y}, {t * x * y + s * z, t * y * y + c, t * y * z - s * x}, {t * x * z - s * y, t * y * z + s * x, t * z * z + c}};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) m.a[i][j] = r[i][j];
    return m;
}

// A part's triangles the right way round when its placement mirrors it.
void fixWinding(ImportedPart& p, const Affine& m) {
    if (m.det() >= 0) return;
    for (Tri& t : p.triangles) std::swap(t[1], t[2]);
    for (std::size_t k = 0; k + 2 < p.cornerColors.size(); k += 3) std::swap(p.cornerColors[k + 1], p.cornerColors[k + 2]);
}

// "#RRGGBB" or "#RRGGBBAA" (sRGB).
std::optional<Vec3> hexColor(const std::string& s) {
    if (s.size() < 7 || s[0] != '#') return std::nullopt;
    auto byte = [&](std::size_t i) { return static_cast<float>(std::strtol(s.substr(i, 2).c_str(), nullptr, 16)) / 255.0f; };
    return Vec3{byte(1), byte(3), byte(5)};
}

float linearToSrgb(float c) {
    c = std::clamp(c, 0.0f, 1.0f);
    return c <= 0.0031308f ? 12.92f * c : 1.055f * std::pow(c, 1.0f / 2.4f) - 0.055f;
}

// A part with one colour on every corner gets it as its own colour instead.
void settleColors(ImportedPart& p) {
    if (p.cornerColors.empty()) return;
    const std::uint32_t c0 = p.cornerColors[0];
    if (std::all_of(p.cornerColors.begin(), p.cornerColors.end(), [&](std::uint32_t c) { return c == c0; })) {
        if (c0) p.color = unpackColor(c0);
        p.cornerColors.clear();
    }
}

// How much a file may make: node trees can share sub-trees (DEF / USE, components used many
// times), which multiply; past these the import stops with a warning instead of running away.
struct Budget {
    std::size_t visits = 0, triangles = 0;
    bool hit = false;
    static constexpr std::size_t kVisits = 2000000, kParts = 20000, kTriangles = 50000000;
    // False (and noted) once the file has made too much.
    bool more(const std::vector<ImportedPart>& parts) {
        if (++visits > kVisits || parts.size() >= kParts || triangles > kTriangles) hit = true;
        return !hit;
    }
    void count(const ImportedPart& p) { triangles += p.triangles.size(); }
    void report(MeshImportReport& rep) const {
        if (hit) rep.warnings.push_back("the file places more than this reader takes (parts used over and over): the import stopped there");
    }
};

double unitFromName(const std::string& u) {
    const std::string s = lowerOf(u);
    if (s == "micron" || s == "micrometer" || s == "micrometre") return 0.001;
    if (s == "centimeter" || s == "centimetre" || s == "cm") return 10;
    if (s == "meter" || s == "metre" || s == "m") return 1000;
    if (s == "inch" || s == "in") return 25.4;
    if (s == "foot" || s == "feet" || s == "ft") return 304.8;
    return 1; // millimetres (also the default)
}

// --- fast scanning of XML element runs (vertex and triangle lists) --------------------------

struct Attrs {
    std::array<std::pair<std::string_view, std::string_view>, 12> kv;
    int n = 0;
    std::string_view get(std::string_view key) const {
        for (int i = 0; i < n; ++i) {
            std::string_view k = kv[static_cast<std::size_t>(i)].first;
            if (const std::size_t c = k.rfind(':'); c != std::string_view::npos) k = k.substr(c + 1);
            if (k == key) return kv[static_cast<std::size_t>(i)].second;
        }
        return {};
    }
};

// Each element named `local` in the raw XML, with its attributes (values not decoded:
// numbers and ids).
template <typename F>
void scanElements(std::string_view raw, std::string_view local, F&& f) {
    std::size_t i = 0;
    while ((i = raw.find('<', i)) != std::string_view::npos) {
        ++i;
        if (i >= raw.size() || raw[i] == '/' || raw[i] == '!' || raw[i] == '?') continue;
        const std::size_t nb = i;
        while (i < raw.size() && !std::isspace(static_cast<unsigned char>(raw[i])) && raw[i] != '/' && raw[i] != '>') ++i;
        std::string_view name = raw.substr(nb, i - nb);
        if (const std::size_t c = name.rfind(':'); c != std::string_view::npos) name = name.substr(c + 1);
        if (name != local) continue;
        Attrs at;
        for (;;) {
            while (i < raw.size() && std::isspace(static_cast<unsigned char>(raw[i]))) ++i;
            if (i >= raw.size() || raw[i] == '/' || raw[i] == '>') break;
            const std::size_t kb = i;
            while (i < raw.size() && raw[i] != '=' && !std::isspace(static_cast<unsigned char>(raw[i]))) ++i;
            const std::string_view key = raw.substr(kb, i - kb);
            while (i < raw.size() && raw[i] != '"' && raw[i] != '\'') ++i;
            if (i >= raw.size()) break;
            const char q = raw[i++];
            const std::size_t e = raw.find(q, i);
            if (e == std::string_view::npos) break;
            if (at.n < 12) at.kv[static_cast<std::size_t>(at.n++)] = {key, raw.substr(i, e - i)};
            i = e + 1;
        }
        f(at);
    }
}

double num(std::string_view s, double fallback = 0) {
    if (s.empty()) return fallback;
    char buf[64];
    const std::size_t n = std::min(s.size(), sizeof buf - 1);
    std::memcpy(buf, s.data(), n);
    buf[n] = 0;
    char* end = nullptr;
    const double v = std::strtod(buf, &end);
    return end == buf ? fallback : v;
}

long inum(std::string_view s, long fallback = -1) {
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) s.remove_prefix(1);
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.remove_suffix(1);
    if (s.empty()) return fallback;
    long v = 0;
    std::size_t i = 0;
    bool neg = false;
    if (s[0] == '-') neg = true, ++i;
    if (i >= s.size()) return fallback;
    for (; i < s.size(); ++i) {
        if (s[i] < '0' || s[i] > '9') return fallback;
        v = v * 10 + (s[i] - '0');
    }
    return neg ? -v : v;
}

// --- big lists cut out of the XML tree ---------------------------------------------------------

// The XML with the contents of every element named in `cut` taken out (kept as raw text, the
// element marked sicdm-raw="index"), so millions of vertices don't each become a tree node.
std::string slimXml(std::string_view text, const std::set<std::string>& cut, std::vector<std::string_view>& raws) {
    std::string out;
    out.reserve(text.size() / 8 + 1024);
    std::size_t i = 0;
    while (i < text.size()) {
        const std::size_t lt = text.find('<', i);
        if (lt == std::string_view::npos) {
            out.append(text.substr(i));
            break;
        }
        out.append(text.substr(i, lt - i));
        if (text.compare(lt, 4, "<!--") == 0) { // comments as they are (they may hold '>')
            const std::size_t e = text.find("-->", lt + 4);
            const std::size_t end = e == std::string_view::npos ? text.size() : e + 3;
            out.append(text.substr(lt, end - lt));
            i = end;
            continue;
        }
        std::size_t ne = lt + 1;
        while (ne < text.size() && !std::isspace(static_cast<unsigned char>(text[ne])) && text[ne] != '>' && text[ne] != '/') ++ne;
        const std::string_view q = text.substr(lt + 1, ne - lt - 1);
        std::string_view local = q;
        if (const std::size_t c = local.rfind(':'); c != std::string_view::npos) local = local.substr(c + 1);
        const std::size_t gt = text.find('>', ne);
        if (gt == std::string_view::npos) {
            out.append(text.substr(lt));
            break;
        }
        if (cut.count(std::string(local)) && text[gt - 1] != '/') {
            const std::string end = "</" + std::string(q);
            const std::size_t close = text.find(end, gt + 1);
            if (close != std::string_view::npos) {
                out.append(text.substr(lt, gt - lt));
                out += " sicdm-raw=\"" + std::to_string(raws.size()) + "\">";
                raws.push_back(text.substr(gt + 1, close - gt - 1));
                i = close;
                continue;
            }
        }
        out.append(text.substr(lt, gt + 1 - lt));
        i = gt + 1;
    }
    return out;
}

// The raw contents cut out of an element (empty if none).
std::string_view rawOf(const XmlNode* n, const std::vector<std::string_view>& raws) {
    if (!n) return {};
    const std::string* r = n->attr("sicdm-raw");
    if (!r) return {};
    const std::size_t k = static_cast<std::size_t>(std::strtoul(r->c_str(), nullptr, 10));
    return k < raws.size() ? raws[k] : std::string_view{};
}

// --- 3MF ------------------------------------------------------------------------------------

struct Model3mf {
    std::string text;
    std::vector<std::string_view> raws; // vertex and triangle lists (in text)
    std::optional<XmlNode> root;        // the rest
    double scale = 1;
    std::map<std::string, const XmlNode*> objects;
    std::map<std::string, std::vector<Vec3>> colors; // property groups: base materials, colour groups
};

Affine transform3mf(const std::string& s) {
    Affine m;
    if (s.empty()) return m;
    std::istringstream in(s);
    double v[12];
    for (double& x : v)
        if (!(in >> x)) return m;
    // Row vectors: p' = p M, the last row the translation.
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) m.a[c][r] = v[r * 3 + c];
        m.a[r][3] = v[9 + r];
    }
    return m;
}

std::vector<ImportedPart> import3mf(const std::string& bytes, MeshImportReport& rep, std::string* why) {
    ZipReader zip;
    std::string w;
    if (!zip.open(bytes, &w)) {
        fail(why, "Not a 3MF package: " + w);
        return {};
    }
    // The model the package's relationships point to (else the first .model part).
    std::string rootPath;
    if (auto rels = zip.read("_rels/.rels")) {
        if (auto x = parseXml(*rels))
            for (const XmlNode* r : x->childrenNamed("Relationship"))
                if (const std::string* type = r->attr("Type"); type && type->size() >= 8 && type->compare(type->size() - 8, 8, "3dmodel") == 0)
                    rootPath = r->attrOr("Target");
    }
    if (rootPath.empty() || !zip.find(rootPath))
        for (const ZipReader::Entry& e : zip.entries())
            if (lowerOf(e.name).size() > 6 && lowerOf(e.name).rfind(".model") == lowerOf(e.name).size() - 6) {
                rootPath = e.name;
                break;
            }
    if (rootPath.empty()) {
        fail(why, "The 3MF package has no 3D model in it");
        return {};
    }
    std::map<std::string, Model3mf> models;
    std::function<Model3mf*(const std::string&)> modelAt = [&](const std::string& path) -> Model3mf* {
        std::string key = lowerOf(path);
        if (!key.empty() && key[0] == '/') key.erase(0, 1);
        if (auto it = models.find(key); it != models.end()) return it->second.root ? &it->second : nullptr;
        Model3mf& m = models[key];
        auto text = zip.read(path, &w);
        if (!text) {
            rep.warnings.push_back("can't read " + path + ": " + w);
            return nullptr;
        }
        m.text = std::move(*text);
        const std::string slim = slimXml(m.text, {"vertices", "triangles"}, m.raws);
        m.root = parseXml(slim, &w);
        if (!m.root) {
            rep.warnings.push_back(path + ": " + w);
            return nullptr;
        }
        m.scale = unitFromName(m.root->attrOr("unit", "millimeter"));
        if (const XmlNode* res = m.root->child("resources")) {
            for (const XmlNode& c : res->children) {
                if (c.name == "object") m.objects[c.attrOr("id")] = &c;
                else if (c.name == "basematerials") {
                    auto& v = m.colors[c.attrOr("id")];
                    for (const XmlNode* b : c.childrenNamed("base")) v.push_back(hexColor(b->attrOr("displaycolor")).value_or(Vec3{0.72f, 0.75f, 0.8f}));
                } else if (c.name == "colorgroup") {
                    auto& v = m.colors[c.attrOr("id")];
                    for (const XmlNode* b : c.childrenNamed("color")) v.push_back(hexColor(b->attrOr("color")).value_or(Vec3{0.72f, 0.75f, 0.8f}));
                }
            }
        }
        return &m;
    };
    Model3mf* root = modelAt(rootPath);
    if (!root) {
        fail(why, "Can't read the 3MF model: " + w);
        return {};
    }
    rep.format = "3MF";
    rep.unitToMm = root->scale;
    std::vector<ImportedPart> parts;
    int skipped = 0;
    Budget budget;
    std::set<std::string> active; // the objects being placed (one that holds itself is refused)
    std::function<void(const std::string&, const std::string&, const Affine&, const std::string&, int)> emit =
        [&](const std::string& path, const std::string& id, const Affine& T, const std::string& assembly, int depth) {
            Model3mf* m = modelAt(path);
            if (!m || depth > 32 || !budget.more(parts)) return;
            const std::string key = lowerOf(path) + "#" + id;
            if (active.count(key)) {
                rep.warnings.push_back("object " + id + " holds itself (through its components): left out there");
                return;
            }
            struct Leave {
                std::set<std::string>& s;
                std::string k;
                ~Leave() { s.erase(k); }
            } leave{active, key};
            active.insert(key);
            const auto it = m->objects.find(id);
            if (it == m->objects.end()) {
                rep.warnings.push_back("no object " + id + " in " + path);
                return;
            }
            const XmlNode& obj = *it->second;
            if (obj.attrOr("type") == "support") {
                ++skipped;
                return;
            }
            const std::string name = obj.attrOr("name", "Object " + id);
            if (const XmlNode* comps = obj.child("components")) {
                const std::string sub = assembly.empty() ? name : assembly + "/" + name;
                for (const XmlNode* c : comps->childrenNamed("component")) {
                    const std::string cpath = c->attrOr("path", path); // (production extension: another model part)
                    emit(cpath, c->attrOr("objectid"), transform3mf(c->attrOr("transform")).then(T), sub, depth + 1);
                }
                return;
            }
            const XmlNode* mesh = obj.child("mesh");
            if (!mesh) return;
            ImportedPart p;
            p.name = name;
            p.assembly = assembly;
            const Affine toMm = T.then(Affine::scale(root->scale));
            scanElements(rawOf(mesh->child("vertices"), m->raws), "vertex", [&](const Attrs& a) {
                p.points.push_back(toMm.apply(Vec3{static_cast<float>(num(a.get("x"))), static_cast<float>(num(a.get("y"))), static_cast<float>(num(a.get("z")))}));
            });
            // Colours: the object's property, or each triangle's (a property group and an index a corner).
            const std::string opid = obj.attrOr("pid");
            const long oindex = std::strtol(obj.attrOr("pindex", "0").c_str(), nullptr, 10);
            auto colorOf = [&](const std::string& pid, long index) -> std::uint32_t {
                const auto g = m->colors.find(pid);
                if (g == m->colors.end() || index < 0 || static_cast<std::size_t>(index) >= g->second.size()) return 0;
                return packColor(g->second[static_cast<std::size_t>(index)]);
            };
            bool bad = false, anyColor = false;
            const std::uint32_t objectColor = opid.empty() ? 0 : colorOf(opid, oindex);
            scanElements(rawOf(mesh->child("triangles"), m->raws), "triangle", [&](const Attrs& a) {
                const long v1 = inum(a.get("v1")), v2 = inum(a.get("v2")), v3 = inum(a.get("v3"));
                const long n = static_cast<long>(p.points.size());
                if (v1 < 0 || v2 < 0 || v3 < 0 || v1 >= n || v2 >= n || v3 >= n) {
                    bad = true;
                    return;
                }
                p.triangles.push_back({static_cast<std::uint32_t>(v1), static_cast<std::uint32_t>(v2), static_cast<std::uint32_t>(v3)});
                const std::string_view tpid = a.get("pid");
                const std::string pid = tpid.empty() ? opid : std::string(tpid);
                const long p1 = inum(a.get("p1"), tpid.empty() ? oindex : 0), p2 = inum(a.get("p2"), p1), p3 = inum(a.get("p3"), p1);
                const std::uint32_t c1 = pid.empty() ? objectColor : colorOf(pid, p1), c2 = pid.empty() ? objectColor : colorOf(pid, p2),
                                    c3 = pid.empty() ? objectColor : colorOf(pid, p3);
                anyColor = anyColor || c1 || c2 || c3;
                p.cornerColors.insert(p.cornerColors.end(), {c1, c2, c3});
            });
            if (bad) rep.warnings.push_back(name + ": triangles with corners that aren't in its vertex list were left out");
            if (!anyColor) p.cornerColors.clear();
            fixWinding(p, toMm);
            settleColors(p);
            budget.count(p);
            if (!p.triangles.empty()) parts.push_back(std::move(p));
        };
    const XmlNode* build = root->root->child("build");
    std::vector<const XmlNode*> items = build ? build->childrenNamed("item") : std::vector<const XmlNode*>{};
    if (!items.empty()) {
        for (const XmlNode* item : items) emit(item->attrOr("path", rootPath), item->attrOr("objectid"), transform3mf(item->attrOr("transform")), "", 0);
    } else { // no build: every object nothing else uses
        std::set<std::string> used;
        for (const auto& [id, obj] : root->objects)
            if (const XmlNode* comps = obj->child("components"))
                for (const XmlNode* c : comps->childrenNamed("component")) used.insert(c->attrOr("objectid"));
        for (const auto& [id, obj] : root->objects)
            if (!used.count(id)) emit(rootPath, id, Affine{}, "", 0);
    }
    if (skipped) rep.warnings.push_back(std::to_string(skipped) + " support objects left out");
    budget.report(rep);
    if (parts.empty()) fail(why, "The 3MF model has no triangles to import");
    return parts;
}

// --- AMF ------------------------------------------------------------------------------------

// Leaf elements' text in document order, with the element path's last two names.
template <typename F>
void scanLeaves(std::string_view raw, F&& f) {
    std::vector<std::string_view> stack;
    std::size_t i = 0;
    while ((i = raw.find('<', i)) != std::string_view::npos) {
        if (raw.compare(i, 4, "<!--") == 0) {
            const std::size_t e = raw.find("-->", i);
            if (e == std::string_view::npos) return;
            i = e + 3;
            continue;
        }
        const bool close = i + 1 < raw.size() && raw[i + 1] == '/';
        std::size_t nb = i + (close ? 2 : 1), ne = nb;
        while (ne < raw.size() && !std::isspace(static_cast<unsigned char>(raw[ne])) && raw[ne] != '>' && raw[ne] != '/') ++ne;
        std::string_view name = raw.substr(nb, ne - nb);
        const std::size_t gt = raw.find('>', ne);
        if (gt == std::string_view::npos) return;
        if (close) {
            if (!stack.empty()) stack.pop_back();
        } else if (raw[gt - 1] != '/') {
            stack.push_back(name);
            const std::size_t next = raw.find('<', gt + 1);
            if (next != std::string_view::npos && raw.compare(next, 2, "</") == 0) // a leaf: its text
                f(stack.size() >= 2 ? stack[stack.size() - 2] : std::string_view{}, name, raw.substr(gt + 1, next - gt - 1));
        }
        i = gt + 1;
    }
}

std::vector<ImportedPart> importAmf(std::string bytes, MeshImportReport& rep, std::string* why) {
    if (looksLikeZip(bytes)) { // a zipped AMF: its .amf member
        ZipReader zip;
        std::string w;
        if (!zip.open(bytes, &w)) {
            fail(why, "Not an AMF file: " + w);
            return {};
        }
        std::optional<std::string> inner;
        for (const ZipReader::Entry& e : zip.entries())
            if (lowerOf(fs::path(e.name).extension().string()) == ".amf" || zip.entries().size() == 1) {
                inner = zip.read(e, &w);
                break;
            }
        if (!inner) {
            fail(why, "The zipped AMF file has no .amf in it" + (w.empty() ? std::string() : ": " + w));
            return {};
        }
        bytes = std::move(*inner);
        rep.format = "AMF (zipped)";
    } else {
        rep.format = "AMF";
    }
    std::string w;
    std::vector<std::string_view> raws;
    const std::string slim = slimXml(bytes, {"vertices", "volume"}, raws);
    const auto doc = parseXml(slim, &w);
    if (!doc || doc->name != "amf") {
        fail(why, doc ? "Not an AMF file (its root is <" + doc->name + ">)" : "Not an AMF file: " + w);
        return {};
    }
    rep.unitToMm = unitFromName(doc->attrOr("unit", "millimeter"));
    auto colorOf = [](const XmlNode* c) -> std::optional<Vec3> {
        if (!c) return std::nullopt;
        auto ch = [&](const char* n) {
            const XmlNode* x = c->child(n);
            return x ? static_cast<float>(std::atof(x->text.c_str())) : 0.0f;
        };
        return Vec3{ch("r"), ch("g"), ch("b")};
    };
    auto metaName = [](const XmlNode& n) -> std::string {
        for (const XmlNode* m : n.childrenNamed("metadata"))
            if (m->attrOr("type") == "name") return m->text;
        return {};
    };
    std::map<std::string, Vec3> materialColor;
    for (const XmlNode* m : doc->childrenNamed("material"))
        if (auto c = colorOf(m->child("color"))) materialColor[m->attrOr("id")] = *c;
    // Each object's mesh in file units, each volume its own colour.
    struct Obj {
        std::string name;
        ImportedPart part;
    };
    std::map<std::string, Obj> objects;
    std::vector<std::string> order;
    for (const XmlNode* o : doc->childrenNamed("object")) {
        const std::string id = o->attrOr("id");
        Obj ob;
        ob.name = metaName(*o);
        if (ob.name.empty()) ob.name = "Object " + id;
        ob.part.name = ob.name;
        const std::optional<Vec3> objColor = colorOf(o->child("color"));
        const XmlNode* mesh = o->child("mesh");
        if (!mesh) continue;
        // The vertices: <x> <y> <z> leaves in order.
        {
            Vec3 q{};
            scanLeaves(rawOf(mesh->child("vertices"), raws), [&](std::string_view parent, std::string_view leaf, std::string_view text) {
                if (parent != "coordinates") return;
                const float v = static_cast<float>(num(text));
                if (leaf == "x") q.x = v;
                else if (leaf == "y") q.y = v;
                else if (leaf == "z") {
                    q.z = v;
                    ob.part.points.push_back(q);
                }
            });
        }
        bool anyColor = false;
        for (const XmlNode* vol : mesh->childrenNamed("volume")) {
            // Its colour (a <color> before its triangles), else its material's, else the object's.
            std::optional<Vec3> vc;
            Vec3 c{};
            bool seenTriangle = false, gotColor = false;
            long v[3] = {-1, -1, -1};
            std::vector<std::array<long, 3>> tris;
            scanLeaves(rawOf(vol, raws), [&](std::string_view parent, std::string_view leaf, std::string_view text) {
                if (parent == "triangle") {
                    seenTriangle = true;
                    const int k = leaf == "v1" ? 0 : leaf == "v2" ? 1 : leaf == "v3" ? 2 : -1;
                    if (k < 0) return;
                    v[k] = inum(text, -1);
                    if (k == 2) tris.push_back({v[0], v[1], v[2]}), v[0] = v[1] = v[2] = -1;
                } else if (parent == "color" && !seenTriangle) {
                    const float x = static_cast<float>(num(text));
                    if (leaf == "r") c.x = x, gotColor = true;
                    else if (leaf == "g") c.y = x;
                    else if (leaf == "b") c.z = x;
                }
            });
            if (gotColor) vc = c;
            if (!vc) {
                const auto mc = materialColor.find(vol->attrOr("materialid"));
                if (mc != materialColor.end()) vc = mc->second;
            }
            if (!vc) vc = objColor;
            const std::uint32_t packed = vc ? packColor(*vc) : 0;
            anyColor = anyColor || packed;
            const long n = static_cast<long>(ob.part.points.size());
            for (const auto& t : tris) {
                if (t[0] < 0 || t[1] < 0 || t[2] < 0 || t[0] >= n || t[1] >= n || t[2] >= n) continue;
                ob.part.triangles.push_back({static_cast<std::uint32_t>(t[0]), static_cast<std::uint32_t>(t[1]), static_cast<std::uint32_t>(t[2])});
                ob.part.cornerColors.insert(ob.part.cornerColors.end(), {packed, packed, packed});
            }
        }
        if (!anyColor) ob.part.cornerColors.clear();
        settleColors(ob.part);
        if (ob.part.triangles.empty()) continue;
        order.push_back(id);
        objects[id] = std::move(ob);
    }
    std::vector<ImportedPart> parts;
    Budget budget;
    const Affine units = Affine::scale(rep.unitToMm);
    auto place = [&](const ImportedPart& src, const Affine& T, const std::string& assembly) {
        if (!budget.more(parts)) return;
        ImportedPart p = src;
        p.assembly = assembly;
        const Affine m = T.then(units);
        for (Vec3& q : p.points) q = m.apply(q);
        fixWinding(p, m);
        budget.count(p);
        parts.push_back(std::move(p));
    };
    // Constellations: instances of objects (or of other constellations) placed and turned.
    std::map<std::string, const XmlNode*> constellations;
    for (const XmlNode* c : doc->childrenNamed("constellation")) constellations[c->attrOr("id")] = c;
    std::set<std::string> placedInside; // by another constellation (one placing itself doesn't count)
    for (const auto& [id, c] : constellations)
        for (const XmlNode* in : c->childrenNamed("instance"))
            if (in->attrOr("objectid") != id) placedInside.insert(in->attrOr("objectid"));
    std::set<const XmlNode*> expanding; // a constellation that holds itself is refused
    std::function<void(const XmlNode*, const Affine&, const std::string&, int)> expand = [&](const XmlNode* c, const Affine& T, const std::string& assembly, int depth) {
        if (depth > 16 || expanding.count(c) || !budget.more(parts)) return;
        expanding.insert(c);
        for (const XmlNode* in : c->childrenNamed("instance")) {
            auto val = [&](const char* n) {
                const XmlNode* x = in->child(n);
                return x ? std::atof(x->text.c_str()) : 0.0;
            };
            const double deg = 3.14159265358979323846 / 180.0;
            Affine local = rotation(1, 0, 0, val("rx") * deg).then(rotation(0, 1, 0, val("ry") * deg)).then(rotation(0, 0, 1, val("rz") * deg));
            local.a[0][3] = val("deltax"), local.a[1][3] = val("deltay"), local.a[2][3] = val("deltaz");
            const std::string oid = in->attrOr("objectid");
            if (const auto o = objects.find(oid); o != objects.end()) place(o->second.part, local.then(T), assembly);
            else if (const auto cc = constellations.find(oid); cc != constellations.end()) {
                const std::string n = metaName(*cc->second);
                expand(cc->second, local.then(T), assembly.empty() ? (n.empty() ? "Constellation " + oid : n) : assembly + "/" + (n.empty() ? "Constellation " + oid : n), depth + 1);
            }
        }
        expanding.erase(c);
    };
    bool anyRoot = false;
    for (const auto& [id, c] : constellations)
        if (!placedInside.count(id)) expand(c, Affine{}, "", 0), anyRoot = true;
    if (!anyRoot && !constellations.empty()) expand(constellations.begin()->second, Affine{}, "", 0); // each holds another: the first
    for (const std::string& id : order) // objects no constellation places: where they are
        if (!placedInside.count(id)) place(objects[id].part, Affine{}, "");
    budget.report(rep);
    if (parts.empty()) fail(why, "The AMF file has no triangles to import");
    return parts;
}

// --- PLY ------------------------------------------------------------------------------------

std::vector<ImportedPart> importPly(const std::string& d, const std::string& stem, MeshImportReport& rep, std::string* why) {
    if (d.compare(0, 3, "ply") != 0) {
        fail(why, "Not a PLY file");
        return {};
    }
    const std::size_t endHeader = d.find("end_header");
    if (endHeader == std::string::npos) {
        fail(why, "The PLY header has no end");
        return {};
    }
    std::size_t body = d.find('\n', endHeader);
    if (body == std::string::npos) {
        fail(why, "The PLY file has nothing after its header");
        return {};
    }
    ++body;
    struct Prop {
        std::string name, type, countType; // countType: a list
    };
    struct Element {
        std::string name;
        std::size_t count = 0;
        std::vector<Prop> props;
    };
    std::vector<Element> elements;
    int format = 0; // 0 ascii, 1 little endian, 2 big endian
    {
        std::istringstream h(d.substr(0, endHeader));
        std::string line;
        while (std::getline(h, line)) {
            std::istringstream ls(line);
            std::string w;
            ls >> w;
            if (w == "format") {
                std::string f;
                ls >> f;
                format = f == "ascii" ? 0 : f == "binary_little_endian" ? 1 : f == "binary_big_endian" ? 2 : -1;
            } else if (w == "element") {
                Element e;
                ls >> e.name >> e.count;
                elements.push_back(e);
            } else if (w == "property" && !elements.empty()) {
                Prop p;
                std::string t;
                ls >> t;
                if (t == "list") ls >> p.countType >> p.type >> p.name;
                else p.type = t, ls >> p.name;
                elements.back().props.push_back(p);
            }
        }
    }
    if (format < 0) {
        fail(why, "A PLY format this reader doesn't know");
        return {};
    }
    auto size = [](const std::string& t) -> int {
        if (t == "char" || t == "uchar" || t == "int8" || t == "uint8") return 1;
        if (t == "short" || t == "ushort" || t == "int16" || t == "uint16") return 2;
        if (t == "int" || t == "uint" || t == "int32" || t == "uint32" || t == "float" || t == "float32") return 4;
        if (t == "double" || t == "float64") return 8;
        return 0;
    };
    // A value of a type at a position (binary) or the next token (ASCII).
    std::size_t at = body;
    bool cut = false;
    std::istringstream ascii(format == 0 ? d.substr(body) : std::string());
    auto read = [&](const std::string& t) -> double {
        if (format == 0) {
            double v = 0;
            if (!(ascii >> v)) cut = true;
            return v;
        }
        const int n = size(t);
        if (n == 0 || at + static_cast<std::size_t>(n) > d.size()) {
            cut = true;
            return 0;
        }
        unsigned char b[8];
        std::memcpy(b, d.data() + at, static_cast<std::size_t>(n));
        at += static_cast<std::size_t>(n);
        if (format == 2) std::reverse(b, b + n);
        if (t == "char" || t == "int8") return static_cast<std::int8_t>(b[0]);
        if (t == "uchar" || t == "uint8") return b[0];
        if (t == "short" || t == "int16") {
            std::int16_t v;
            std::memcpy(&v, b, 2);
            return v;
        }
        if (t == "ushort" || t == "uint16") {
            std::uint16_t v;
            std::memcpy(&v, b, 2);
            return v;
        }
        if (t == "int" || t == "int32") {
            std::int32_t v;
            std::memcpy(&v, b, 4);
            return v;
        }
        if (t == "uint" || t == "uint32") {
            std::uint32_t v;
            std::memcpy(&v, b, 4);
            return v;
        }
        if (t == "float" || t == "float32") {
            float v;
            std::memcpy(&v, b, 4);
            return v;
        }
        double v;
        std::memcpy(&v, b, 8);
        return v;
    };
    ImportedPart p;
    p.name = stem;
    std::vector<std::uint32_t> vertexColor;
    bool colored = false;
    for (const Element& e : elements) {
        const bool vertex = e.name == "vertex", face = e.name == "face";
        for (std::size_t k = 0; k < e.count && !cut; ++k) {
            Vec3 q{};
            Vec3 c{-1, -1, -1};
            for (const Prop& pr : e.props) {
                if (!pr.countType.empty()) {
                    const std::size_t n = static_cast<std::size_t>(read(pr.countType));
                    if (n > 1024) {
                        cut = true;
                        break;
                    }
                    std::vector<std::uint32_t> idx(n);
                    for (std::size_t j = 0; j < n; ++j) idx[j] = static_cast<std::uint32_t>(read(pr.type));
                    if (face && (pr.name == "vertex_indices" || pr.name == "vertex_index"))
                        for (std::size_t j = 1; j + 1 < n; ++j) p.triangles.push_back({idx[0], idx[j], idx[j + 1]});
                    continue;
                }
                const double v = read(pr.type);
                if (!vertex) continue;
                const bool unit = pr.type == "float" || pr.type == "float32" || pr.type == "double" || pr.type == "float64";
                if (pr.name == "x") q.x = static_cast<float>(v);
                else if (pr.name == "y") q.y = static_cast<float>(v);
                else if (pr.name == "z") q.z = static_cast<float>(v);
                else if (pr.name == "red" || pr.name == "r" || pr.name == "diffuse_red") c.x = static_cast<float>(unit ? v : v / 255.0);
                else if (pr.name == "green" || pr.name == "g" || pr.name == "diffuse_green") c.y = static_cast<float>(unit ? v : v / 255.0);
                else if (pr.name == "blue" || pr.name == "b" || pr.name == "diffuse_blue") c.z = static_cast<float>(unit ? v : v / 255.0);
            }
            if (vertex) {
                p.points.push_back(q);
                vertexColor.push_back(c.x >= 0 && c.y >= 0 && c.z >= 0 ? packColor(c) : 0);
                colored = colored || vertexColor.back();
            }
        }
    }
    if (cut) rep.warnings.push_back("the file ends before its header says it should: what was there is imported");
    const std::size_t n = p.points.size();
    std::erase_if(p.triangles, [&](const Tri& t) { return t[0] >= n || t[1] >= n || t[2] >= n; });
    if (colored)
        for (const Tri& t : p.triangles) p.cornerColors.insert(p.cornerColors.end(), {vertexColor[t[0]], vertexColor[t[1]], vertexColor[t[2]]});
    settleColors(p);
    rep.format = format == 0 ? "PLY (ASCII)" : "PLY (binary)";
    rep.unitAssumed = true;
    if (p.triangles.empty()) {
        fail(why, "The PLY file has no faces to import");
        return {};
    }
    return {std::move(p)};
}

// --- OFF ------------------------------------------------------------------------------------

std::vector<ImportedPart> importOff(const std::string& d, const std::string& stem, MeshImportReport& rep, std::string* why) {
    // Lines without comments, split into numbers.
    std::istringstream in(d);
    std::string line, header;
    std::vector<std::vector<double>> rows;
    while (std::getline(in, line)) {
        if (const std::size_t h = line.find('#'); h != std::string::npos) line.erase(h);
        std::istringstream ls(line);
        std::string first;
        if (!(ls >> first)) continue;
        if (header.empty() && !first.empty() && std::isalpha(static_cast<unsigned char>(first[0]))) {
            header = first;
            std::vector<double> rest;
            double v;
            while (ls >> v) rest.push_back(v);
            if (!rest.empty()) rows.push_back(rest);
            continue;
        }
        std::vector<double> r;
        r.push_back(std::atof(first.c_str()));
        double v;
        while (ls >> v) r.push_back(v);
        rows.push_back(r);
    }
    if (header.find("OFF") == std::string::npos) {
        fail(why, "Not an OFF file");
        return {};
    }
    if (rows.empty() || rows[0].size() < 2) {
        fail(why, "The OFF file has no counts");
        return {};
    }
    const std::size_t nv = static_cast<std::size_t>(rows[0][0]), nf = static_cast<std::size_t>(rows[0][1]);
    if (rows.size() < 1 + nv + nf) rep.warnings.push_back("the file ends before its counts say it should");
    const bool colors = header.find('C') != std::string::npos, normals = header.find('N') != std::string::npos && header.find("NOFF") != std::string::npos;
    ImportedPart p;
    p.name = stem;
    std::vector<std::uint32_t> vc;
    bool colored = false;
    for (std::size_t k = 0; k < nv && 1 + k < rows.size(); ++k) {
        const auto& r = rows[1 + k];
        if (r.size() < 3) continue;
        p.points.push_back({static_cast<float>(r[0]), static_cast<float>(r[1]), static_cast<float>(r[2])});
        std::uint32_t c = 0;
        const std::size_t ci = normals ? 6 : 3;
        if (colors && r.size() >= ci + 3) {
            const bool bytes = r[ci] > 1 || r[ci + 1] > 1 || r[ci + 2] > 1;
            const float s = bytes ? 1.0f / 255.0f : 1.0f;
            c = packColor({static_cast<float>(r[ci]) * s, static_cast<float>(r[ci + 1]) * s, static_cast<float>(r[ci + 2]) * s});
            colored = true;
        }
        vc.push_back(c);
    }
    for (std::size_t k = 0; k < nf && 1 + nv + k < rows.size(); ++k) {
        const auto& r = rows[1 + nv + k];
        if (r.empty()) continue;
        const std::size_t n = static_cast<std::size_t>(r[0]);
        if (r.size() < 1 + n) continue;
        // A colour after the corners: the face's.
        std::uint32_t fc = 0;
        if (r.size() >= 1 + n + 3) {
            const bool bytes = r[1 + n] > 1 || r[2 + n] > 1 || r[3 + n] > 1;
            const float s = bytes ? 1.0f / 255.0f : 1.0f;
            fc = packColor({static_cast<float>(r[1 + n]) * s, static_cast<float>(r[2 + n]) * s, static_cast<float>(r[3 + n]) * s});
        }
        for (std::size_t j = 2; j < n; ++j) {
            const Tri t{static_cast<std::uint32_t>(r[1]), static_cast<std::uint32_t>(r[j]), static_cast<std::uint32_t>(r[j + 1])};
            if (t[0] >= p.points.size() || t[1] >= p.points.size() || t[2] >= p.points.size()) continue;
            p.triangles.push_back(t);
            if (fc) colored = true;
            for (std::uint32_t v : t) p.cornerColors.push_back(fc ? fc : vc[v]);
        }
    }
    if (!colored) p.cornerColors.clear();
    settleColors(p);
    rep.format = colors ? "OFF (with colours)" : "OFF";
    rep.unitAssumed = true;
    if (p.triangles.empty()) {
        fail(why, "The OFF file has no faces to import");
        return {};
    }
    return {std::move(p)};
}

// --- glTF 2.0 -------------------------------------------------------------------------------

std::optional<std::string> base64(std::string_view s) {
    static const std::array<int, 256> table = [] {
        std::array<int, 256> t{};
        t.fill(-1);
        const char* a = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        for (int i = 0; i < 64; ++i) t[static_cast<unsigned char>(a[i])] = i;
        t['-'] = 62, t['_'] = 63; // the URL-safe alphabet too
        return t;
    }();
    std::string out;
    out.reserve(s.size() * 3 / 4);
    unsigned acc = 0;
    int bits = 0;
    for (char ch : s) {
        if (ch == '=' || std::isspace(static_cast<unsigned char>(ch))) continue;
        const int v = table[static_cast<unsigned char>(ch)];
        if (v < 0) return std::nullopt;
        acc = acc << 6 | static_cast<unsigned>(v);
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            out.push_back(static_cast<char>((acc >> bits) & 0xFF));
        }
    }
    return out;
}

std::vector<ImportedPart> importGltf(const std::string& bytes, const std::string& path, MeshImportReport& rep, std::string* why) {
    std::string jsonText, bin;
    if (bytes.size() >= 12 && bytes.compare(0, 4, "glTF") == 0) { // GLB: a JSON chunk and a binary chunk
        auto u32 = [&](std::size_t at) {
            std::uint32_t v;
            std::memcpy(&v, bytes.data() + at, 4);
            return v;
        };
        std::size_t at = 12;
        while (at + 8 <= bytes.size()) {
            const std::uint32_t len = u32(at), type = u32(at + 4);
            if (at + 8 + len > bytes.size()) break;
            if (type == 0x4E4F534Au) jsonText = bytes.substr(at + 8, len);
            else if (type == 0x004E4942u && bin.empty()) bin = bytes.substr(at + 8, len);
            at += 8 + ((len + 3) & ~3u);
        }
        rep.format = "glTF 2.0 (binary)";
    } else {
        jsonText = bytes;
        rep.format = "glTF 2.0";
    }
    std::string err;
    const auto doc = Json::parse(jsonText, &err);
    if (!doc || !doc->isObject()) {
        fail(why, "Not a glTF file: " + err);
        return {};
    }
    const Json& g = *doc;
    if (g["asset"]["version"].asString().rfind("2", 0) != 0) {
        fail(why, "Only glTF 2.0 can be read (this file is version " + g["asset"]["version"].asString() + ")");
        return {};
    }
    for (std::size_t i = 0; i < g["extensionsRequired"].size(); ++i) {
        const std::string ext = g["extensionsRequired"][i].asString();
        if (ext != "KHR_materials_emissive_strength" && ext != "KHR_mesh_quantization" && ext != "KHR_texture_transform") {
            fail(why, "The glTF file needs the " + ext + " extension, which this reader doesn't have (save it without compression)");
            return {};
        }
    }
    // Buffers: the GLB's own, embedded data, or files beside it.
    std::vector<std::string> buffers;
    for (std::size_t i = 0; i < g["buffers"].size(); ++i) {
        const Json& b = g["buffers"][i];
        const std::string uri = b["uri"].asString();
        if (uri.empty()) buffers.push_back(i == 0 ? bin : std::string());
        else if (uri.rfind("data:", 0) == 0) {
            const std::size_t comma = uri.find(',');
            auto data = comma == std::string::npos ? std::nullopt : base64(std::string_view(uri).substr(comma + 1));
            if (!data) {
                fail(why, "Buffer " + std::to_string(i) + " has embedded data that isn't base64");
                return {};
            }
            buffers.push_back(std::move(*data));
        } else {
            std::string file;
            for (std::size_t k = 0; k < uri.size(); ++k) // %20 and the like
                if (uri[k] == '%' && k + 2 < uri.size()) file.push_back(static_cast<char>(std::strtol(uri.substr(k + 1, 2).c_str(), nullptr, 16))), k += 2;
                else file.push_back(uri[k]);
            std::string w;
            auto data = slurp((fs::path(path).parent_path() / fs::path(reinterpret_cast<const char8_t*>(file.c_str()))).string(), &w);
            if (!data) {
                fail(why, "Buffer " + uri + " isn't beside the glTF file");
                return {};
            }
            buffers.push_back(std::move(*data));
        }
    }
    // An accessor's values as doubles, `comps` a value (normalised integers scaled to 0..1 or -1..1).
    auto accessor = [&](int index, int& comps, std::string* aw) -> std::optional<std::vector<double>> {
        const Json& a = g["accessors"][index];
        if (!a.isObject()) {
            if (aw) *aw = "no accessor " + std::to_string(index);
            return std::nullopt;
        }
        const std::string type = a["type"].asString();
        comps = type == "SCALAR" ? 1 : type == "VEC2" ? 2 : type == "VEC3" ? 3 : type == "VEC4" ? 4 : type == "MAT4" ? 16 : 0;
        const int ct = a["componentType"].asInt();
        const int csize = ct == 5120 || ct == 5121 ? 1 : ct == 5122 || ct == 5123 ? 2 : ct == 5125 || ct == 5126 ? 4 : 0;
        const std::size_t count = static_cast<std::size_t>(a["count"].asNumber());
        if (!comps || !csize || count > (std::size_t{1} << 28)) {
            if (aw) *aw = "accessor " + std::to_string(index) + " has a type this reader doesn't know";
            return std::nullopt;
        }
        const bool normalized = a["normalized"].asBool(false);
        // The data must be there before room is made for it (an accessor without a buffer view
        // is zeros: kept small).
        if (a.has("bufferView")) {
            const Json& bv = g["bufferViews"][a["bufferView"].asInt()];
            const int bi = bv["buffer"].asInt();
            const std::size_t need = count ? static_cast<std::size_t>(bv["byteOffset"].asNumber() + a["byteOffset"].asNumber()) +
                                                 (bv["byteStride"].asNumber() > 0 ? static_cast<std::size_t>(bv["byteStride"].asNumber()) : static_cast<std::size_t>(csize * comps)) * (count - 1) +
                                                 static_cast<std::size_t>(csize * comps)
                                           : 0;
            if (bi < 0 || static_cast<std::size_t>(bi) >= buffers.size() || need > buffers[static_cast<std::size_t>(bi)].size()) {
                if (aw) *aw = "accessor " + std::to_string(index) + " runs past the end of its buffer";
                return std::nullopt;
            }
        } else if (count > (std::size_t{1} << 22)) {
            if (aw) *aw = "accessor " + std::to_string(index) + " is too large";
            return std::nullopt;
        }
        std::vector<double> out(count * static_cast<std::size_t>(comps), 0.0);
        auto decode = [&](const char* p) -> double {
            switch (ct) {
            case 5120: return normalized ? std::max(-1.0, static_cast<std::int8_t>(*p) / 127.0) : static_cast<std::int8_t>(*p);
            case 5121: return normalized ? static_cast<std::uint8_t>(*p) / 255.0 : static_cast<std::uint8_t>(*p);
            case 5122: {
                std::int16_t v;
                std::memcpy(&v, p, 2);
                return normalized ? std::max(-1.0, v / 32767.0) : v;
            }
            case 5123: {
                std::uint16_t v;
                std::memcpy(&v, p, 2);
                return normalized ? v / 65535.0 : v;
            }
            case 5125: {
                std::uint32_t v;
                std::memcpy(&v, p, 4);
                return v;
            }
            default: {
                float v;
                std::memcpy(&v, p, 4);
                return v;
            }
            }
        };
        if (a.has("bufferView")) {
            const Json& bv = g["bufferViews"][a["bufferView"].asInt()];
            const int bi = bv["buffer"].asInt();
            if (bi < 0 || static_cast<std::size_t>(bi) >= buffers.size()) {
                if (aw) *aw = "accessor " + std::to_string(index) + " points at a buffer that isn't there";
                return std::nullopt;
            }
            const std::string& buf = buffers[static_cast<std::size_t>(bi)];
            const std::size_t base = static_cast<std::size_t>(bv["byteOffset"].asNumber()) + static_cast<std::size_t>(a["byteOffset"].asNumber());
            const std::size_t elem = static_cast<std::size_t>(csize * comps);
            const std::size_t stride = bv["byteStride"].asNumber() > 0 ? static_cast<std::size_t>(bv["byteStride"].asNumber()) : elem;
            if (count && base + stride * (count - 1) + elem > buf.size()) {
                if (aw) *aw = "accessor " + std::to_string(index) + " runs past the end of its buffer";
                return std::nullopt;
            }
            for (std::size_t k = 0; k < count; ++k)
                for (int c = 0; c < comps; ++c) out[k * static_cast<std::size_t>(comps) + static_cast<std::size_t>(c)] = decode(buf.data() + base + stride * k + static_cast<std::size_t>(c * csize));
        }
        // Sparse: some elements replaced.
        if (const Json& sp = a["sparse"]; sp.isObject()) {
            const std::size_t n = static_cast<std::size_t>(sp["count"].asNumber());
            const Json& si = sp["indices"];
            const Json& sv = sp["values"];
            const Json& ibv = g["bufferViews"][si["bufferView"].asInt()];
            const Json& vbv = g["bufferViews"][sv["bufferView"].asInt()];
            const int ib = ibv["buffer"].asInt(), vb = vbv["buffer"].asInt();
            if (ib < 0 || vb < 0 || static_cast<std::size_t>(ib) >= buffers.size() || static_cast<std::size_t>(vb) >= buffers.size()) return out;
            const std::string& ibuf = buffers[static_cast<std::size_t>(ib)];
            const std::string& vbuf = buffers[static_cast<std::size_t>(vb)];
            const int ict = si["componentType"].asInt();
            const std::size_t isz = ict == 5121 ? 1 : ict == 5123 ? 2 : 4;
            const std::size_t ibase = static_cast<std::size_t>(ibv["byteOffset"].asNumber() + si["byteOffset"].asNumber()),
                              vbase = static_cast<std::size_t>(vbv["byteOffset"].asNumber() + sv["byteOffset"].asNumber());
            if (ibase + isz * n > ibuf.size() || vbase + static_cast<std::size_t>(csize * comps) * n > vbuf.size()) return out;
            for (std::size_t k = 0; k < n; ++k) {
                std::uint32_t idx = 0;
                std::memcpy(&idx, ibuf.data() + ibase + isz * k, isz);
                if (idx >= count) continue;
                for (int c = 0; c < comps; ++c)
                    out[idx * static_cast<std::size_t>(comps) + static_cast<std::size_t>(c)] = decode(vbuf.data() + vbase + (k * static_cast<std::size_t>(comps) + static_cast<std::size_t>(c)) * static_cast<std::size_t>(csize));
            }
        }
        return out;
    };
    // +Y up in metres to +Z up in millimetres.
    Affine toCad;
    toCad.a[0][0] = 1000, toCad.a[0][1] = 0, toCad.a[0][2] = 0;
    toCad.a[1][0] = 0, toCad.a[1][1] = 0, toCad.a[1][2] = -1000;
    toCad.a[2][0] = 0, toCad.a[2][1] = 1000, toCad.a[2][2] = 0;
    rep.unitToMm = 1000;
    auto nodeLocal = [&](const Json& n) {
        Affine m;
        if (n["matrix"].size() == 16) { // column major
            for (int c = 0; c < 4; ++c)
                for (int r = 0; r < 3; ++r) m.a[r][c] = n["matrix"][c * 4 + r].asNumber();
            return m;
        }
        const Json& t = n["translation"];
        const Json& r = n["rotation"];
        const Json& s = n["scale"];
        double qx = r.size() == 4 ? r[0].asNumber() : 0, qy = r.size() == 4 ? r[1].asNumber() : 0, qz = r.size() == 4 ? r[2].asNumber() : 0, qw = r.size() == 4 ? r[3].asNumber() : 1;
        const double ql = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
        if (ql > 0) qx /= ql, qy /= ql, qz /= ql, qw /= ql;
        const double R[3][3] = {{1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)},
                                {2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)},
                                {2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)}};
        const double S[3] = {s.size() == 3 ? s[0].asNumber() : 1, s.size() == 3 ? s[1].asNumber() : 1, s.size() == 3 ? s[2].asNumber() : 1};
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) m.a[i][j] = R[i][j] * S[j];
            m.a[i][3] = t.size() == 3 ? t[i].asNumber() : 0;
        }
        return m;
    };
    std::vector<ImportedPart> parts;
    int skippedPrims = 0;
    Budget budget;
    std::set<int> visiting; // a node that is its own ancestor is refused
    std::function<void(int, const Affine&, const std::string&, int)> visit = [&](int ni, const Affine& parent, const std::string& assembly, int depth) {
        const Json& n = g["nodes"][ni];
        if (!n.isObject() || depth > 64 || visiting.count(ni) || !budget.more(parts)) return;
        visiting.insert(ni);
        struct Leave {
            std::set<int>& s;
            int k;
            ~Leave() { s.erase(k); }
        } leave{visiting, ni};
        const Affine world = nodeLocal(n).then(parent);
        const std::string nodeName = n["name"].asString();
        if (n.has("mesh")) {
            const Json& mesh = g["meshes"][n["mesh"].asInt()];
            const Json& prims = mesh["primitives"];
            std::string base = !nodeName.empty() ? nodeName : mesh["name"].asString();
            if (base.empty()) base = "Mesh " + std::to_string(n["mesh"].asInt());
            for (std::size_t pi = 0; pi < prims.size(); ++pi) {
                const Json& pr = prims[pi];
                const int mode = pr.has("mode") ? pr["mode"].asInt() : 4;
                if (mode < 4 || mode > 6 || !pr["attributes"].has("POSITION")) {
                    ++skippedPrims;
                    continue;
                }
                int comps = 0;
                std::string aw;
                auto pos = accessor(pr["attributes"]["POSITION"].asInt(), comps, &aw);
                if (!pos || comps != 3) {
                    rep.warnings.push_back(base + ": " + (aw.empty() ? "positions that aren't 3D" : aw));
                    continue;
                }
                ImportedPart p;
                const Affine m = world.then(toCad);
                for (std::size_t k = 0; k + 2 < pos->size(); k += 3)
                    p.points.push_back(m.apply(Vec3{static_cast<float>((*pos)[k]), static_cast<float>((*pos)[k + 1]), static_cast<float>((*pos)[k + 2])}));
                std::vector<std::uint32_t> idx;
                if (pr.has("indices")) {
                    int ic = 0;
                    auto ind = accessor(pr["indices"].asInt(), ic, &aw);
                    if (!ind) {
                        rep.warnings.push_back(base + ": " + aw);
                        continue;
                    }
                    for (double v : *ind) idx.push_back(static_cast<std::uint32_t>(v));
                } else {
                    for (std::uint32_t k = 0; k < p.points.size(); ++k) idx.push_back(k);
                }
                const std::size_t np = p.points.size();
                auto add = [&](std::uint32_t a, std::uint32_t b, std::uint32_t c) {
                    if (a < np && b < np && c < np && a != b && b != c && a != c) p.triangles.push_back({a, b, c});
                };
                if (mode == 4)
                    for (std::size_t k = 0; k + 2 < idx.size(); k += 3) add(idx[k], idx[k + 1], idx[k + 2]);
                else if (mode == 5)
                    for (std::size_t k = 0; k + 2 < idx.size(); ++k) k % 2 ? add(idx[k + 1], idx[k], idx[k + 2]) : add(idx[k], idx[k + 1], idx[k + 2]);
                else
                    for (std::size_t k = 1; k + 1 < idx.size(); ++k) add(idx[0], idx[k], idx[k + 1]);
                // Vertex colours (linear): to sRGB.
                if (pr["attributes"].has("COLOR_0")) {
                    int cc = 0;
                    if (auto col = accessor(pr["attributes"]["COLOR_0"].asInt(), cc, &aw); col && (cc == 3 || cc == 4) && col->size() / static_cast<std::size_t>(cc) == np) {
                        std::vector<std::uint32_t> vc(np);
                        for (std::size_t k = 0; k < np; ++k)
                            vc[k] = packColor({linearToSrgb(static_cast<float>((*col)[k * static_cast<std::size_t>(cc)])), linearToSrgb(static_cast<float>((*col)[k * static_cast<std::size_t>(cc) + 1])),
                                               linearToSrgb(static_cast<float>((*col)[k * static_cast<std::size_t>(cc) + 2]))});
                        for (const Tri& t : p.triangles) p.cornerColors.insert(p.cornerColors.end(), {vc[t[0]], vc[t[1]], vc[t[2]]});
                    }
                }
                std::string matName;
                if (pr.has("material")) {
                    const Json& mat = g["materials"][pr["material"].asInt()];
                    matName = mat["name"].asString();
                    const Json& pbr = mat["pbrMetallicRoughness"];
                    const Json& bc = pbr["baseColorFactor"];
                    if (bc.size() >= 3) p.color = Vec3{linearToSrgb(bc[0].asFloat()), linearToSrgb(bc[1].asFloat()), linearToSrgb(bc[2].asFloat())};
                    else p.color = Vec3{1, 1, 1};
                    p.metallic = std::clamp(pbr.has("metallicFactor") ? pbr["metallicFactor"].asFloat() : 1.0f, 0.0f, 1.0f);
                    p.roughness = std::clamp(pbr.has("roughnessFactor") ? pbr["roughnessFactor"].asFloat() : 1.0f, 0.0f, 1.0f);
                }
                p.name = prims.size() > 1 ? base + " " + (matName.empty() ? std::to_string(pi + 1) : matName) : base;
                p.assembly = assembly;
                fixWinding(p, m);
                settleColors(p);
                budget.count(p);
                if (!p.triangles.empty() && budget.more(parts)) parts.push_back(std::move(p));
            }
        }
        const Json& kids = n["children"];
        const std::string sub = kids.size() && !nodeName.empty() ? (assembly.empty() ? nodeName : assembly + "/" + nodeName) : assembly;
        for (std::size_t k = 0; k < kids.size(); ++k) visit(kids[k].asInt(), world, sub, depth + 1);
    };
    // The scene shown (else every node no other node has as a child).
    std::vector<int> roots;
    const int sceneIndex = g.has("scene") ? g["scene"].asInt() : 0;
    if (g["scenes"].size() > 0) {
        const Json& sc = g["scenes"][sceneIndex];
        for (std::size_t k = 0; k < sc["nodes"].size(); ++k) roots.push_back(sc["nodes"][k].asInt());
    } else {
        std::set<int> kids;
        for (std::size_t i = 0; i < g["nodes"].size(); ++i)
            for (std::size_t k = 0; k < g["nodes"][i]["children"].size(); ++k) kids.insert(g["nodes"][i]["children"][k].asInt());
        for (std::size_t i = 0; i < g["nodes"].size(); ++i)
            if (!kids.count(static_cast<int>(i))) roots.push_back(static_cast<int>(i));
    }
    for (int r : roots) visit(r, Affine{}, "", 0);
    if (skippedPrims) rep.warnings.push_back(std::to_string(skippedPrims) + " point or line primitives left out (only triangles are bodies)");
    budget.report(rep);
    if (parts.empty()) fail(why, "The glTF file has no triangles to import");
    return parts;
}

// --- VRML 97 --------------------------------------------------------------------------------

struct VrmlLexer {
    std::string_view s;
    std::size_t i = 0;
    // The next token: a word, a number, a string, or one of { } [ ].
    std::string_view next() {
        for (;;) {
            while (i < s.size() && (std::isspace(static_cast<unsigned char>(s[i])) || s[i] == ',')) ++i;
            if (i < s.size() && s[i] == '#') {
                while (i < s.size() && s[i] != '\n') ++i;
                continue;
            }
            break;
        }
        if (i >= s.size()) return {};
        const std::size_t b = i;
        if (s[i] == '{' || s[i] == '}' || s[i] == '[' || s[i] == ']') return s.substr(i++, 1);
        if (s[i] == '"') {
            ++i;
            while (i < s.size() && s[i] != '"') i += s[i] == '\\' ? 2 : 1;
            ++i;
            return s.substr(b, std::min(i, s.size()) - b);
        }
        while (i < s.size() && !std::isspace(static_cast<unsigned char>(s[i])) && s[i] != ',' && s[i] != '{' && s[i] != '}' && s[i] != '[' && s[i] != ']' && s[i] != '#') ++i;
        return s.substr(b, i - b);
    }
    std::string_view peek() {
        const std::size_t save = i;
        const std::string_view t = next();
        i = save;
        return t;
    }
};

struct VrmlNode {
    std::string type, def; // def: its DEF name
    std::map<std::string, std::vector<double>> numbers;
    std::map<std::string, std::vector<std::shared_ptr<VrmlNode>>> nodes;
    std::map<std::string, std::string> words; // a single word or string value (TRUE, FALSE, "name")
};

bool isNumberToken(std::string_view t) {
    return !t.empty() && (std::isdigit(static_cast<unsigned char>(t[0])) || ((t[0] == '-' || t[0] == '+' || t[0] == '.') && t.size() > 1));
}

struct VrmlParser {
    VrmlLexer lex;
    std::map<std::string, std::shared_ptr<VrmlNode>> defs;
    int depth = 0;

    void skipBalanced(std::string_view open, std::string_view close) {
        int d = 1;
        while (d > 0) {
            const std::string_view t = lex.next();
            if (t.empty()) return;
            if (t == open) ++d;
            else if (t == close) --d;
        }
    }
    // A node after its type word (or DEF / USE), or nothing.
    std::shared_ptr<VrmlNode> node(std::string_view first) {
        if (first == "USE") {
            const auto it = defs.find(std::string(lex.next()));
            return it == defs.end() ? nullptr : it->second;
        }
        std::string def;
        if (first == "DEF") {
            def = std::string(lex.next());
            first = lex.next();
        }
        if (lex.peek() != "{") return nullptr;
        lex.next();
        auto n = std::make_shared<VrmlNode>();
        n->type = std::string(first);
        n->def = def;
        if (++depth > 200) return nullptr;
        for (;;) {
            const std::string_view field = lex.next();
            if (field.empty() || field == "}") break;
            if (field == "ROUTE") {
                lex.next(), lex.next(), lex.next();
                continue;
            }
            if (field == "PROTO" || field == "EXTERNPROTO") { // definitions inside: skipped
                lex.next();
                if (lex.next() == "[") skipBalanced("[", "]");
                if (field == "PROTO" && lex.next() == "{") skipBalanced("{", "}");
                continue;
            }
            const std::string f(field);
            const std::string_view v = lex.peek();
            if (v == "[") {
                lex.next();
                for (;;) {
                    const std::string_view t = lex.next();
                    if (t.empty() || t == "]") break;
                    if (isNumberToken(t)) n->numbers[f].push_back(num(t));
                    else if (t[0] == '"') n->words[f] = std::string(t.substr(1, t.size() > 1 ? t.size() - 2 : 0));
                    else if (auto c = node(t)) n->nodes[f].push_back(c);
                }
            } else if (isNumberToken(v)) {
                while (isNumberToken(lex.peek())) n->numbers[f].push_back(num(lex.next()));
            } else if (!v.empty() && v[0] == '"') {
                lex.next();
                n->words[f] = std::string(v.substr(1, v.size() > 1 ? v.size() - 2 : 0));
            } else if (v == "TRUE" || v == "FALSE" || v == "NULL") {
                n->words[f] = std::string(lex.next());
            } else if (!v.empty() && v != "}") {
                lex.next();
                if (auto c = node(v)) n->nodes[f].push_back(c);
            }
        }
        --depth;
        if (!def.empty()) defs[def] = n; // (only now: a USE inside it can't make a loop)
        return n;
    }
};

std::vector<ImportedPart> importVrml(std::string bytes, const std::string& stem, MeshImportReport& rep, std::string* why) {
    if (!gunzipIfZipped(bytes, why)) return {};
    if (bytes.compare(0, 10, "#VRML V2.0") != 0) {
        fail(why, bytes.compare(0, 10, "#VRML V1.0") == 0 ? "VRML 1.0 can't be read (only VRML 97, version 2.0)" : "Not a VRML 97 file");
        return {};
    }
    rep.format = "VRML 97";
    VrmlParser vp;
    vp.lex.s = bytes;
    std::vector<std::shared_ptr<VrmlNode>> roots;
    for (;;) {
        const std::string_view t = vp.lex.next();
        if (t.empty()) break;
        if (t == "ROUTE") {
            vp.lex.next(), vp.lex.next(), vp.lex.next();
            continue;
        }
        if (t == "PROTO" || t == "EXTERNPROTO") {
            vp.lex.next();
            if (vp.lex.next() == "[") vp.skipBalanced("[", "]");
            if (t == "PROTO" && vp.lex.next() == "{") vp.skipBalanced("{", "}");
            continue;
        }
        if (auto n = vp.node(t)) roots.push_back(n);
    }
    std::vector<ImportedPart> parts;
    int shapes = 0, primitives = 0;
    Budget budget;
    std::function<void(const VrmlNode&, const Affine&, const std::string&, int)> walk = [&](const VrmlNode& n, const Affine& T, const std::string& assembly, int depth) {
        if (depth > 64 || !budget.more(parts)) return;
        auto vec = [&](const char* f, std::vector<double> d) {
            const auto it = n.numbers.find(f);
            return it != n.numbers.end() && it->second.size() >= d.size() ? std::vector<double>(it->second.begin(), it->second.begin() + static_cast<std::ptrdiff_t>(d.size())) : d;
        };
        Affine local;
        if (n.type == "Transform") {
            // T x C x R x SR x S x -SR x -C
            const auto t = vec("translation", {0, 0, 0}), c = vec("center", {0, 0, 0}), r = vec("rotation", {0, 0, 1, 0}), s = vec("scale", {1, 1, 1}),
                       so = vec("scaleOrientation", {0, 0, 1, 0});
            Affine S;
            S.a[0][0] = s[0], S.a[1][1] = s[1], S.a[2][2] = s[2];
            const Affine SR = rotation(so[0], so[1], so[2], so[3]), SRi = rotation(so[0], so[1], so[2], -so[3]);
            Affine C, Ci;
            for (int k = 0; k < 3; ++k) C.a[k][3] = c[static_cast<std::size_t>(k)], Ci.a[k][3] = -c[static_cast<std::size_t>(k)];
            Affine Tr;
            for (int k = 0; k < 3; ++k) Tr.a[k][3] = t[static_cast<std::size_t>(k)];
            local = Ci.then(SRi).then(S).then(SR).then(rotation(r[0], r[1], r[2], r[3])).then(C).then(Tr);
        }
        const Affine world = local.then(T);
        const std::string name = !n.def.empty() ? n.def : n.words.count("name") ? n.words.at("name") : std::string();
        if (n.type == "Shape") {
            ++shapes;
            const auto g = n.nodes.find("geometry");
            if (g == n.nodes.end() || g->second.empty()) return;
            const VrmlNode& geo = *g->second[0];
            if (geo.type != "IndexedFaceSet") {
                if (geo.type == "Box" || geo.type == "Sphere" || geo.type == "Cylinder" || geo.type == "Cone") ++primitives;
                return;
            }
            const auto coordNode = geo.nodes.find("coord");
            if (coordNode == geo.nodes.end() || coordNode->second.empty()) return;
            const auto pts = coordNode->second[0]->numbers.find("point");
            if (pts == coordNode->second[0]->numbers.end()) return;
            ImportedPart p;
            p.name = name.empty() ? stem + " " + std::to_string(parts.size() + 1) : name;
            p.assembly = assembly;
            const Affine m = world.then(Affine::scale(rep.unitToMm));
            for (std::size_t k = 0; k + 2 < pts->second.size(); k += 3)
                p.points.push_back(m.apply(Vec3{static_cast<float>(pts->second[k]), static_cast<float>(pts->second[k + 1]), static_cast<float>(pts->second[k + 2])}));
            const auto ci = geo.numbers.find("coordIndex");
            if (ci == geo.numbers.end()) return;
            const bool ccw = !geo.words.count("ccw") || geo.words.at("ccw") != "FALSE";
            std::vector<std::uint32_t> face;
            auto flush = [&]() {
                for (std::size_t j = 1; j + 1 < face.size(); ++j) {
                    const Tri t = ccw ? Tri{face[0], face[j], face[j + 1]} : Tri{face[0], face[j + 1], face[j]};
                    if (t[0] < p.points.size() && t[1] < p.points.size() && t[2] < p.points.size()) p.triangles.push_back(t);
                }
                face.clear();
            };
            for (double v : ci->second) {
                if (v < 0) flush();
                else face.push_back(static_cast<std::uint32_t>(v));
            }
            flush();
            if (const auto ap = n.nodes.find("appearance"); ap != n.nodes.end() && !ap->second.empty())
                if (const auto mat = ap->second[0]->nodes.find("material"); mat != ap->second[0]->nodes.end() && !mat->second.empty()) {
                    const auto dc = mat->second[0]->numbers.find("diffuseColor");
                    if (dc != mat->second[0]->numbers.end() && dc->second.size() >= 3)
                        p.color = Vec3{static_cast<float>(dc->second[0]), static_cast<float>(dc->second[1]), static_cast<float>(dc->second[2])};
                    const auto sh = mat->second[0]->numbers.find("shininess");
                    if (sh != mat->second[0]->numbers.end() && !sh->second.empty()) p.roughness = static_cast<float>(std::clamp(1.0 - sh->second[0], 0.0, 1.0));
                }
            fixWinding(p, m);
            budget.count(p);
            if (!p.triangles.empty()) parts.push_back(std::move(p));
            return;
        }
        const std::string sub = !name.empty() ? (assembly.empty() ? name : assembly + "/" + name) : assembly;
        for (const char* f : {"children", "level", "choice"}) {
            const auto it = n.nodes.find(f);
            if (it == n.nodes.end()) continue;
            if (std::string(f) == "choice") { // a Switch shows one
                const auto wc = n.numbers.find("whichChoice");
                const long k = wc != n.numbers.end() && !wc->second.empty() ? static_cast<long>(wc->second[0]) : -1;
                if (k >= 0 && static_cast<std::size_t>(k) < it->second.size()) walk(*it->second[static_cast<std::size_t>(k)], world, sub, depth + 1);
                continue;
            }
            if (std::string(f) == "level") { // an LOD: its finest
                if (!it->second.empty()) walk(*it->second[0], world, sub, depth + 1);
                continue;
            }
            for (const auto& c : it->second) walk(*c, world, sub, depth + 1);
        }
    };
    for (const auto& r : roots) walk(*r, Affine{}, "", 0);
    if (primitives) rep.warnings.push_back(std::to_string(primitives) + " Box, Sphere, Cylinder or Cone shapes left out (only face sets are read)");
    budget.report(rep);
    if (parts.empty()) fail(why, shapes ? "The VRML file has no face sets to import" : "The VRML file has no shapes");
    return parts;
}

} // namespace

bool isMeshModelExtension(const std::string& ext) {
    const std::string e = lowerOf(ext);
    return e == ".3mf" || e == ".amf" || e == ".ply" || e == ".off" || e == ".gltf" || e == ".glb" || e == ".wrl" || e == ".vrml" || e == ".wrz";
}

namespace {
std::vector<ImportedPart> importMeshModelImpl(const std::string& path, MeshImportReport* report, std::string* why, std::optional<double> unitToMm);
} // namespace

std::vector<ImportedPart> importMeshModel(const std::string& path, MeshImportReport* report, std::string* why, std::optional<double> unitToMm) {
    try {
        return importMeshModelImpl(path, report, why, unitToMm);
    } catch (const std::bad_alloc&) {
        fail(why, "The file is too large to import");
    } catch (const std::length_error&) {
        fail(why, "The file is too large to import");
    }
    return {};
}

namespace {
std::vector<ImportedPart> importMeshModelImpl(const std::string& path, MeshImportReport* report, std::string* why, std::optional<double> unitToMm) {
    MeshImportReport local;
    MeshImportReport& rep = report ? *report : local;
    rep = MeshImportReport{};
    auto bytes = slurp(path, why);
    if (!bytes) return {};
    const std::string ext = lowerOf(fs::path(path).extension().string()), stem = fs::path(path).stem().string();
    std::vector<ImportedPart> parts;
    if (ext == ".3mf") parts = import3mf(*bytes, rep, why);
    else if (ext == ".amf") parts = importAmf(std::move(*bytes), rep, why);
    else if (ext == ".ply") parts = importPly(*bytes, stem, rep, why);
    else if (ext == ".off") parts = importOff(*bytes, stem, rep, why);
    else if (ext == ".gltf" || ext == ".glb") parts = importGltf(*bytes, path, rep, why);
    else if (ext == ".wrl" || ext == ".vrml" || ext == ".wrz") {
        rep.unitToMm = 1000; // VRML is in metres
        parts = importVrml(std::move(*bytes), stem, rep, why);
    } else {
        fail(why, "Not a mesh model format this reader knows: " + ext);
        return {};
    }
    // A unit given for the file: in place of what it says (or what was taken).
    if (unitToMm && *unitToMm > 0 && !parts.empty()) {
        const float k = static_cast<float>(*unitToMm / rep.unitToMm);
        for (ImportedPart& p : parts)
            for (Vec3& q : p.points) q = q * k;
        rep.unitToMm = *unitToMm;
        rep.unitAssumed = false;
    }
    return parts;
}
} // namespace

Body bodyFromPart(const ImportedPart& part, float creaseDeg) {
    // Face normals and areas; for each corner, the normal of the faces round its point that
    // turn less than the crease from its own face.
    const std::size_t nt = part.triangles.size();
    std::vector<Vec3> fn(nt);
    for (std::size_t t = 0; t < nt; ++t) {
        const Tri& tr = part.triangles[t];
        fn[t] = cross(part.points[tr[1]] - part.points[tr[0]], part.points[tr[2]] - part.points[tr[0]]); // length: twice the area
    }
    std::vector<std::vector<std::uint32_t>> around(part.points.size());
    for (std::size_t t = 0; t < nt; ++t)
        for (std::uint32_t v : part.triangles[t]) around[v].push_back(static_cast<std::uint32_t>(t));
    const float cosCrease = std::cos(creaseDeg * 3.14159265f / 180.0f);
    MeshData m;
    m.vertices.reserve(part.points.size());
    m.indices.reserve(nt * 3);
    // Shared vertices: one per point, normal and colour.
    std::vector<std::vector<std::uint32_t>> made(part.points.size());
    const bool colors = part.cornerColors.size() == nt * 3;
    for (std::size_t t = 0; t < nt; ++t) {
        const Vec3 own = length(fn[t]) > 0 ? normalize(fn[t]) : Vec3{0, 0, 1};
        for (int k = 0; k < 3; ++k) {
            const std::uint32_t v = part.triangles[t][static_cast<std::size_t>(k)];
            Vec3 n{};
            for (std::uint32_t o : around[v]) {
                const float l = length(fn[o]);
                if (l > 0 && dot(fn[o] / l, own) >= cosCrease) n += fn[o];
            }
            n = length(n) > 0 ? normalize(n) : own;
            const std::uint32_t col = colors ? part.cornerColors[t * 3 + static_cast<std::size_t>(k)] : 0;
            std::uint32_t id = ~0u;
            for (std::uint32_t c : made[v])
                if (dot(m.vertices[c].normal, n) > 0.9999f && m.vertices[c].color == col) {
                    id = c;
                    break;
                }
            if (id == ~0u) {
                Vertex vx{part.points[v], n};
                vx.color = col;
                id = static_cast<std::uint32_t>(m.vertices.size());
                m.vertices.push_back(vx);
                made[v].push_back(id);
            }
            m.indices.push_back(id);
        }
    }
    m.computeFeatureEdges();
    Body b = meshBody(part.name, std::move(m), part.color.value_or(Vec3{0.72f, 0.75f, 0.80f}));
    b.assembly = part.assembly;
    if (part.metallic) b.material.metallic = *part.metallic;
    if (part.roughness) b.material.roughness = *part.roughness;
    return b;
}

} // namespace sicdm

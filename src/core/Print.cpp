#include "core/Print.hpp"

#include "core/Scene.hpp"
#include "core/Sculpt.hpp"
#include "core/Section.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <numeric>
#include <unordered_map>
#include <unordered_set>

namespace sicdm {

namespace {

using Tri = std::array<std::uint32_t, 3>;

std::uint64_t edgeOf(std::uint32_t a, std::uint32_t b) {
    return a < b ? (static_cast<std::uint64_t>(a) << 32) | b : (static_cast<std::uint64_t>(b) << 32) | a;
}
std::uint64_t directed(std::uint32_t a, std::uint32_t b) { return (static_cast<std::uint64_t>(a) << 32) | b; }

Vec3 faceNormal(const std::vector<Vec3>& p, const Tri& t) { return cross(p[t[1]] - p[t[0]], p[t[2]] - p[t[0]]); }

// Shells: triangles joined through shared corners (union-find on the corners).
std::vector<std::uint32_t> shellOf(std::size_t vertices, const std::vector<Tri>& tris, std::size_t* count) {
    std::vector<std::uint32_t> parent(vertices);
    std::iota(parent.begin(), parent.end(), 0u);
    auto find = [&](std::uint32_t v) {
        while (parent[v] != v) v = parent[v] = parent[parent[v]];
        return v;
    };
    for (const Tri& t : tris) {
        const std::uint32_t a = find(t[0]);
        parent[find(t[1])] = a;
        parent[find(t[2])] = a;
    }
    std::unordered_map<std::uint32_t, std::uint32_t> id;
    std::vector<std::uint32_t> out(tris.size());
    for (std::size_t i = 0; i < tris.size(); ++i) out[i] = id.try_emplace(find(tris[i][0]), static_cast<std::uint32_t>(id.size())).first->second;
    if (count) *count = id.size();
    return out;
}

double signedVolume(const std::vector<Vec3>& p, const Tri& t) {
    const Vec3 &a = p[t[0]], &b = p[t[1]], &c = p[t[2]];
    return static_cast<double>(dot(a, cross(b, c))) / 6.0;
}

// The area that needs support with every point and face normal turned by `m`.
double supportArea(const std::vector<Vec3>& p, const std::vector<Tri>& tris, const Mat4& m, float overhangAngle, float* height) {
    std::vector<float> z(p.size());
    float lo = 1e30f, hi = -1e30f;
    for (std::size_t i = 0; i < p.size(); ++i) {
        z[i] = m.transformPoint(p[i]).z;
        lo = std::min(lo, z[i]);
        hi = std::max(hi, z[i]);
    }
    if (height) *height = hi - lo;
    const float limit = -std::sin(radians(overhangAngle));
    double area = 0;
    for (const Tri& t : tris) {
        const Vec3 n = m.transformDir(faceNormal(p, t));
        const float len = length(n);
        if (!(len > 0) || n.z / len >= limit) continue;
        if (std::max({z[t[0]], z[t[1]], z[t[2]]}) <= lo + 0.2f) continue; // on the bed
        area += 0.5 * static_cast<double>(len);
    }
    return area;
}

// A rotation taking direction d to straight down.
Mat4 downward(const Vec3& d) {
    const Vec3 down{0, 0, -1};
    const Vec3 axis = cross(d, down);
    const float c = std::clamp(dot(d, down), -1.0f, 1.0f);
    if (length(axis) < 1e-6f) return c > 0 ? Mat4::identity() : rotateAxis({1, 0, 0}, kPi);
    return rotateAxis(normalize(axis), std::acos(c));
}

std::string xmlEscape(const std::string& s) {
    std::string out;
    for (const char ch : s) {
        switch (ch) {
        case '&': out += "&amp;"; break;
        case '<': out += "&lt;"; break;
        case '>': out += "&gt;"; break;
        case '"': out += "&quot;"; break;
        default: out += ch;
        }
    }
    return out;
}

// A zip archive of stored (uncompressed) files, as a 3MF package is.
class ZipWriter {
public:
    void add(const std::string& name, const std::string& data) {
        Entry e{name, static_cast<std::uint32_t>(m_out.size()), crc32(data), static_cast<std::uint32_t>(data.size())};
        u32(0x04034b50);
        u16(20); // version needed
        u16(0);  // flags
        u16(0);  // stored
        u16(0);  // time
        u16(0x21); // date: 1980-01-01
        u32(e.crc);
        u32(e.size);
        u32(e.size);
        u16(static_cast<std::uint16_t>(name.size()));
        u16(0);
        m_out += name;
        m_out += data;
        m_entries.push_back(e);
    }
    std::string finish() {
        const auto cd = static_cast<std::uint32_t>(m_out.size());
        for (const Entry& e : m_entries) {
            u32(0x02014b50);
            u16(20);
            u16(20);
            u16(0);
            u16(0);
            u16(0);
            u16(0x21);
            u32(e.crc);
            u32(e.size);
            u32(e.size);
            u16(static_cast<std::uint16_t>(e.name.size()));
            u16(0);
            u16(0);
            u16(0);
            u16(0);
            u32(0);
            u32(e.offset);
            m_out += e.name;
        }
        const auto cdSize = static_cast<std::uint32_t>(m_out.size()) - cd;
        u32(0x06054b50);
        u16(0);
        u16(0);
        u16(static_cast<std::uint16_t>(m_entries.size()));
        u16(static_cast<std::uint16_t>(m_entries.size()));
        u32(cdSize);
        u32(cd);
        u16(0);
        return std::move(m_out);
    }
    static std::uint32_t crc32(const std::string& data) {
        static const auto table = [] {
            std::array<std::uint32_t, 256> t{};
            for (std::uint32_t i = 0; i < 256; ++i) {
                std::uint32_t c = i;
                for (int k = 0; k < 8; ++k) c = c & 1 ? 0xEDB88320u ^ (c >> 1) : c >> 1;
                t[i] = c;
            }
            return t;
        }();
        std::uint32_t c = 0xFFFFFFFFu;
        for (const char ch : data) c = table[(c ^ static_cast<unsigned char>(ch)) & 0xFFu] ^ (c >> 8);
        return c ^ 0xFFFFFFFFu;
    }

private:
    struct Entry {
        std::string name;
        std::uint32_t offset, crc, size;
    };
    void u16(std::uint16_t v) {
        m_out += static_cast<char>(v & 0xFF);
        m_out += static_cast<char>(v >> 8);
    }
    void u32(std::uint32_t v) {
        u16(static_cast<std::uint16_t>(v & 0xFFFF));
        u16(static_cast<std::uint16_t>(v >> 16));
    }
    std::string m_out;
    std::vector<Entry> m_entries;
};

// --- slicing: the sections, walls and infill come from core/Section ---------

using section::Loop;
using section::P2;
using section::contours;
using section::hatch;
using section::loopArea;
using section::offsetLeft;

class Gcode {
public:
    Gcode(const PrintSettings& s) : m_s(s) {
        const double r = s.filament * 0.5;
        m_ePerMm = s.lineWidth * s.layerHeight / (kPi * r * r);
    }
    void line(const std::string& l) { m_text += l + "\n"; }
    void layer(int k, double z) {
        line(";LAYER:" + std::to_string(k));
        if (k == 1) line("M106 S255");
        char b[64];
        std::snprintf(b, sizeof(b), "G0 Z%.3f F%.0f", z, m_s.travelSpeed * 60.0);
        line(b);
        m_first = k == 0;
    }
    void travel(const P2& q) {
        const double d = std::hypot(q.x - m_at.x, q.y - m_at.y);
        if (d < 1e-6) return;
        const bool retract = d > 2.0 && m_s.retract > 0;
        char b[96];
        if (retract) {
            m_e -= m_s.retract;
            std::snprintf(b, sizeof(b), "G1 E%.5f F2700", m_e);
            line(b);
        }
        std::snprintf(b, sizeof(b), "G0 X%.3f Y%.3f F%.0f", q.x, q.y, m_s.travelSpeed * 60.0);
        line(b);
        if (retract) {
            m_e += m_s.retract;
            std::snprintf(b, sizeof(b), "G1 E%.5f F2700", m_e);
            line(b);
        }
        m_seconds += d / m_s.travelSpeed;
        m_at = q;
        ++m_moves;
    }
    void extrude(const P2& q) {
        const double d = std::hypot(q.x - m_at.x, q.y - m_at.y);
        if (d < 1e-6) return;
        m_e += d * m_ePerMm;
        m_filament += d * m_ePerMm;
        const double v = m_first ? m_s.firstLayerSpeed : m_s.speed;
        char b[96];
        std::snprintf(b, sizeof(b), "G1 X%.3f Y%.3f E%.5f F%.0f", q.x, q.y, m_e, v * 60.0);
        line(b);
        m_seconds += d / v;
        m_at = q;
        ++m_moves;
    }
    std::string m_text;
    double m_seconds = 0, m_filament = 0;
    std::size_t m_moves = 0;

private:
    const PrintSettings& m_s;
    double m_ePerMm = 0, m_e = 0;
    P2 m_at{0, 0};
    bool m_first = true;
};

} // namespace

// ---------------------------------------------------------------------------

PrintReport analyzePrint(const Body& body, const PrintSettings& s) {
    PrintReport r;
    const SculptMesh sm(body);
    const auto& p = sm.positions();
    const auto& tris = sm.triangles();
    r.triangles = tris.size();
    if (tris.empty()) return r;
    std::unordered_map<std::uint64_t, int> uses;
    uses.reserve(tris.size() * 2);
    for (const Tri& t : tris)
        for (int e = 0; e < 3; ++e) ++uses[edgeOf(t[static_cast<std::size_t>(e)], t[static_cast<std::size_t>((e + 1) % 3)])];
    for (const auto& [k, n] : uses) {
        if (n == 1) ++r.openEdges;
        if (n > 2) ++r.nonManifoldEdges;
    }
    shellOf(p.size(), tris, &r.shells);
    for (const Tri& t : tris) {
        r.volume += signedVolume(p, t);
        r.area += 0.5 * static_cast<double>(length(faceNormal(p, t)));
    }
    r.insideOut = r.volume < 0;
    r.volume = std::fabs(r.volume);
    for (const Vec3& q : p) r.box.expand(q);
    const Vec3 size = r.box.max - r.box.min;
    r.fits = size.x <= s.buildVolume.x && size.y <= s.buildVolume.y && size.z <= s.buildVolume.z;
    if (r.insideOut) { // the faces point in: measure as if they pointed out
        std::vector<Tri> flipped = tris;
        for (Tri& t : flipped) std::swap(t[1], t[2]);
        r.overhangArea = supportArea(p, flipped, Mat4::identity(), s.overhangAngle, nullptr);
    } else {
        r.overhangArea = supportArea(p, tris, Mat4::identity(), s.overhangAngle, nullptr);
    }
    // Wall thickness: from points spread over the surface, straight in to the far side.
    std::vector<Vec3> vn(p.size(), Vec3{});
    for (const Tri& t : tris) {
        const Vec3 n = faceNormal(p, t) * (r.insideOut ? -1.0f : 1.0f);
        for (const std::uint32_t v : t) vn[v] += n;
    }
    const std::size_t samples = std::min<std::size_t>(p.size(), 600);
    const std::size_t stride = std::max<std::size_t>(1, p.size() / std::max<std::size_t>(samples, 1));
    const float eps = std::max(length(size), 1e-3f) * 1e-5f;
    for (std::size_t i = 0; i < p.size(); i += stride) {
        const Vec3 n = normalize(vn[i]);
        if (!(length(n) > 0.5f)) continue;
        const auto hit = sm.raycast({p[i] - n * eps, -n});
        if (!hit) continue;
        const float d = length(*hit - p[i]);
        if (d < eps * 4) continue;
        ++r.wallSamples;
        if (r.thinnest == 0.0f || d < r.thinnest) r.thinnest = d;
        if (d < s.minWall) ++r.thinSamples;
    }
    // Material and time: the walls (at most the whole part) and the infill inside them.
    const double shell = std::min(r.volume, r.area * s.walls * s.lineWidth);
    const double printed = shell + std::max(0.0, r.volume - shell) * std::clamp(s.infill, 0.0f, 1.0f);
    const double fr = s.filament * 0.5;
    r.grams = printed / 1000.0 * s.density;
    r.filamentMetres = printed / (kPi * fr * fr) / 1000.0;
    r.minutes = printed / (static_cast<double>(s.lineWidth) * s.layerHeight * s.speed) / 60.0 * 1.25;
    return r;
}

bool repairMesh(Body& body, RepairReport* report, std::string* error) {
    RepairReport rep;
    const SculptMesh sm(body); // welds seams and near-coincident corners
    std::vector<Vec3> p = sm.positions();
    std::vector<Tri> tris = sm.triangles();
    if (tris.empty()) {
        if (error) *error = body.name + " has no surface";
        return false;
    }
    AABB box;
    for (const Vec3& q : p) box.expand(q);
    const float size = std::max(length(box.max - box.min), 1e-3f);
    // Degenerate faces (no area) and duplicates (the same three corners).
    {
        std::vector<Tri> kept;
        std::unordered_set<std::string> seen;
        for (const Tri& t : tris) {
            if (length(faceNormal(p, t)) <= size * size * 1e-12f) {
                ++rep.degenerate;
                continue;
            }
            Tri k = t;
            std::sort(k.begin(), k.end());
            if (!seen.insert(std::to_string(k[0]) + ":" + std::to_string(k[1]) + ":" + std::to_string(k[2])).second) {
                ++rep.duplicates;
                continue;
            }
            kept.push_back(t);
        }
        tris = std::move(kept);
    }
    // Consistent winding: across every edge with two faces, the faces run it in
    // opposite directions (a walk over each shell flips the ones that do not).
    const std::vector<Tri> original = tris;
    {
        std::unordered_map<std::uint64_t, std::vector<std::uint32_t>> byEdge;
        for (std::size_t i = 0; i < tris.size(); ++i)
            for (int e = 0; e < 3; ++e) byEdge[edgeOf(tris[i][static_cast<std::size_t>(e)], tris[i][static_cast<std::size_t>((e + 1) % 3)])].push_back(static_cast<std::uint32_t>(i));
        std::vector<int> state(tris.size(), -1); // -1 unvisited, 0 kept, 1 flipped
        std::vector<std::uint32_t> stack;
        auto runs = [&](std::uint32_t ti, std::uint32_t a, std::uint32_t b) {
            const Tri& t = tris[ti];
            for (int e = 0; e < 3; ++e)
                if (t[static_cast<std::size_t>(e)] == a && t[static_cast<std::size_t>((e + 1) % 3)] == b) return true;
            return false;
        };
        for (std::size_t seed = 0; seed < tris.size(); ++seed) {
            if (state[seed] >= 0) continue;
            state[seed] = 0;
            stack.push_back(static_cast<std::uint32_t>(seed));
            while (!stack.empty()) {
                const std::uint32_t i = stack.back();
                stack.pop_back();
                Tri t = tris[i];
                if (state[i] == 1) std::swap(t[1], t[2]);
                for (int e = 0; e < 3; ++e) {
                    const std::uint32_t a = t[static_cast<std::size_t>(e)], b = t[static_cast<std::size_t>((e + 1) % 3)];
                    const auto& across = byEdge[edgeOf(a, b)];
                    if (across.size() != 2) continue;
                    const std::uint32_t j = across[0] == i ? across[1] : across[0];
                    if (state[j] >= 0) continue;
                    state[j] = runs(j, a, b) ? 1 : 0; // the same way round: flip it
                    stack.push_back(j);
                }
            }
        }
        for (std::size_t i = 0; i < tris.size(); ++i)
            if (state[i] == 1) std::swap(tris[i][1], tris[i][2]);
    }
    // Specks: shells far smaller than the rest.
    {
        std::size_t shells = 0;
        const std::vector<std::uint32_t> shell = shellOf(p.size(), tris, &shells);
        std::vector<double> area(shells, 0.0), volume(shells, 0.0);
        double total = 0;
        for (std::size_t i = 0; i < tris.size(); ++i) {
            const double a = 0.5 * static_cast<double>(length(faceNormal(p, tris[i])));
            area[shell[i]] += a;
            volume[shell[i]] += signedVolume(p, tris[i]);
            total += a;
        }
        std::vector<Tri> kept;
        std::vector<Tri> was; // each kept face as it came in
        std::vector<char> gone(shells, 0);
        for (std::size_t k = 0; k < shells; ++k)
            if (shells > 1 && area[k] < total * 1e-4) {
                gone[k] = 1;
                ++rep.specks;
            }
        for (std::size_t i = 0; i < tris.size(); ++i) {
            if (gone[shell[i]]) continue;
            Tri t = tris[i];
            if (volume[shell[i]] < 0) std::swap(t[1], t[2]); // an inside-out shell: turned the right way
            kept.push_back(t);
            was.push_back(original[i]);
        }
        tris = std::move(kept);
        // Faces that end up the other way round from how they came.
        for (std::size_t i = 0; i < tris.size(); ++i) {
            const Tri& t = tris[i];
            const Tri& o = was[i];
            int k = 0;
            while (k < 3 && o[static_cast<std::size_t>(k)] != t[0]) ++k;
            if (k < 3 && o[static_cast<std::size_t>((k + 1) % 3)] != t[1]) ++rep.flipped;
        }
    }
    // Holes: each loop of open edges is closed with a fan round its centre.
    {
        std::unordered_set<std::uint64_t> runs;
        for (const Tri& t : tris)
            for (int e = 0; e < 3; ++e) runs.insert(directed(t[static_cast<std::size_t>(e)], t[static_cast<std::size_t>((e + 1) % 3)]));
        std::unordered_multimap<std::uint32_t, std::uint32_t> next; // along the open edges
        for (const Tri& t : tris)
            for (int e = 0; e < 3; ++e) {
                const std::uint32_t a = t[static_cast<std::size_t>(e)], b = t[static_cast<std::size_t>((e + 1) % 3)];
                if (!runs.count(directed(b, a))) next.emplace(a, b);
            }
        while (!next.empty()) {
            const auto first = next.begin();
            const std::uint32_t start = first->first;
            std::vector<std::uint32_t> loop{start};
            std::uint32_t at = first->second;
            next.erase(first);
            bool closed = false;
            for (std::size_t guard = 0; guard < 1000000; ++guard) {
                if (at == start) {
                    closed = true;
                    break;
                }
                loop.push_back(at);
                const auto it = next.find(at);
                if (it == next.end()) break;
                at = it->second;
                next.erase(it);
            }
            if (!closed || loop.size() < 3) continue;
            ++rep.holesFilled;
            if (loop.size() == 3) {
                tris.push_back({loop[2], loop[1], loop[0]});
                continue;
            }
            Vec3 c{};
            for (const std::uint32_t v : loop) c += p[v];
            c = c / static_cast<float>(loop.size());
            const auto ci = static_cast<std::uint32_t>(p.size());
            p.push_back(c);
            for (std::size_t i = 0; i < loop.size(); ++i) tris.push_back({loop[(i + 1) % loop.size()], loop[i], ci});
        }
    }
    MeshData m;
    m.vertices.reserve(p.size());
    for (const Vec3& q : p) m.vertices.push_back({q - body.offset, {0, 0, 1}});
    for (const Tri& t : tris) m.indices.insert(m.indices.end(), t.begin(), t.end());
    body.mesh = std::move(m);
    body.kind = "mesh";
    body.params.clear();
    SculptMesh(body).writeTo(body, true); // shading normals and feature edges
    if (report) *report = rep;
    return true;
}

double orientForPrint(Body& body, const PrintSettings& s, double* before) {
    const SculptMesh sm(body);
    const auto& p = sm.positions();
    const auto& tris = sm.triangles();
    if (tris.empty()) return 0.0;
    // Which way down: the six axis directions and the normals of the largest faces
    // (a flat face set on the bed needs nothing under it).
    std::vector<Vec3> downs{{0, 0, -1}, {0, 0, 1}, {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}};
    std::vector<std::pair<float, Vec3>> faces;
    for (const Tri& t : tris) {
        const Vec3 n = faceNormal(p, t);
        faces.emplace_back(length(n), n);
    }
    std::partial_sort(faces.begin(), faces.begin() + static_cast<std::ptrdiff_t>(std::min<std::size_t>(faces.size(), 40)), faces.end(),
                      [](const auto& a, const auto& b) { return a.first > b.first; });
    for (std::size_t i = 0; i < std::min<std::size_t>(faces.size(), 40); ++i) {
        const Vec3 n = normalize(faces[i].second);
        bool near = false;
        for (const Vec3& d : downs) near = near || dot(d, n) > 0.996f;
        if (!near && downs.size() < 30) downs.push_back(n);
    }
    float h0 = 0;
    const double area0 = supportArea(p, tris, Mat4::identity(), s.overhangAngle, &h0);
    if (before) *before = area0;
    Mat4 best = Mat4::identity();
    double bestArea = area0;
    float bestHeight = h0;
    for (const Vec3& d : downs) {
        const Mat4 m = downward(d);
        float h = 0;
        const double a = supportArea(p, tris, m, s.overhangAngle, &h);
        if (a < bestArea - std::max(1.0, bestArea * 0.01) || (a <= bestArea + std::max(1.0, bestArea * 0.01) && h < bestHeight - 0.5f)) {
            best = m;
            bestArea = a;
            bestHeight = h;
        }
    }
    body.mesh.transform(best); // about the body's origin
    body.kind = "mesh";
    body.params.clear();
    const AABB b = body.bounds();
    if (b.valid()) body.offset.z -= b.min.z; // down on the bed
    return bestArea;
}

bool export3mf(const Scene& scene, const std::string& path, std::string* error) {
    std::string model =
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        "<model unit=\"millimeter\" xml:lang=\"en-US\" xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\">\n"
        " <metadata name=\"Application\">SI-CDM</metadata>\n <resources>\n";
    std::string items;
    int id = 0;
    char b[160];
    for (const Body& body : scene.bodies()) {
        if (!body.visible || body.mesh.indices.empty()) continue;
        const SculptMesh sm(body);
        if (sm.empty()) continue;
        ++id;
        model += "  <object id=\"" + std::to_string(id) + "\" type=\"model\" name=\"" + xmlEscape(body.name) + "\">\n   <mesh>\n    <vertices>\n";
        for (const Vec3& q : sm.positions()) {
            std::snprintf(b, sizeof(b), "     <vertex x=\"%.6g\" y=\"%.6g\" z=\"%.6g\"/>\n", static_cast<double>(q.x), static_cast<double>(q.y),
                          static_cast<double>(q.z));
            model += b;
        }
        model += "    </vertices>\n    <triangles>\n";
        for (const Tri& t : sm.triangles()) {
            std::snprintf(b, sizeof(b), "     <triangle v1=\"%u\" v2=\"%u\" v3=\"%u\"/>\n", t[0], t[1], t[2]);
            model += b;
        }
        model += "    </triangles>\n   </mesh>\n  </object>\n";
        items += "  <item objectid=\"" + std::to_string(id) + "\"/>\n";
    }
    if (id == 0) {
        if (error) *error = "Nothing visible to export";
        return false;
    }
    model += " </resources>\n <build>\n" + items + " </build>\n</model>\n";
    ZipWriter zip;
    zip.add("[Content_Types].xml",
            "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
            "<Types xmlns=\"http://schemas.openxmlformats.org/package/2006/content-types\">"
            "<Default Extension=\"rels\" ContentType=\"application/vnd.openxmlformats-package.relationships+xml\"/>"
            "<Default Extension=\"model\" ContentType=\"application/vnd.ms-package.3dmanufacturing-3dmodel+xml\"/></Types>\n");
    zip.add("_rels/.rels", "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
                           "<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">"
                           "<Relationship Target=\"/3D/3dmodel.model\" Id=\"rel0\" "
                           "Type=\"http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel\"/></Relationships>\n");
    zip.add("3D/3dmodel.model", model);
    std::ofstream f(path, std::ios::binary);
    if (!f) {
        if (error) *error = "Cannot write " + path;
        return false;
    }
    const std::string data = zip.finish();
    f.write(data.data(), static_cast<std::streamsize>(data.size()));
    return static_cast<bool>(f);
}

bool sliceToGcode(const Body& body, const PrintSettings& s, const std::string& path, SliceReport* report, std::string* error) {
    if (!(s.layerHeight > 0.01f) || !(s.lineWidth > 0.05f) || !(s.filament > 0.1f)) {
        if (error) *error = "Layer height, line width and filament need sensible sizes";
        return false;
    }
    const SculptMesh sm(body);
    std::vector<Vec3> p = sm.positions();
    const auto& tris = sm.triangles();
    if (tris.empty()) {
        if (error) *error = body.name + " has no surface";
        return false;
    }
    // On the bed, centred.
    AABB box;
    for (const Vec3& q : p) box.expand(q);
    const Vec3 shift{s.buildVolume.x * 0.5f - box.center().x, s.buildVolume.y * 0.5f - box.center().y, -box.min.z};
    for (Vec3& q : p) q += shift;
    const double height = static_cast<double>(box.max.z - box.min.z);
    // A sliver under 1% of a layer is float noise (from ORIENT's turn, say), not a layer.
    const int layers = static_cast<int>(std::ceil(height / s.layerHeight - 0.01));
    if (layers <= 0 || layers > 200000) {
        if (error) *error = body.name + " is too flat or too tall to slice";
        return false;
    }
    // The triangles that cross each layer.
    std::vector<std::vector<std::uint32_t>> bucket(static_cast<std::size_t>(layers));
    for (std::size_t i = 0; i < tris.size(); ++i) {
        const Tri& t = tris[i];
        const double lo = std::min({p[t[0]].z, p[t[1]].z, p[t[2]].z}), hi = std::max({p[t[0]].z, p[t[1]].z, p[t[2]].z});
        const int k0 = std::max(0, static_cast<int>(std::floor(lo / s.layerHeight - 0.5))),
                  k1 = std::min(layers - 1, static_cast<int>(std::ceil(hi / s.layerHeight - 0.5)));
        for (int k = k0; k <= k1; ++k) bucket[static_cast<std::size_t>(k)].push_back(static_cast<std::uint32_t>(i));
    }
    Gcode g(s);
    char b[160];
    g.line("; Generated by SI-CDM: " + body.name);
    std::snprintf(b, sizeof(b), "; layer %.3f mm, line %.3f mm, %d walls, infill %.0f%%, filament %.2f mm", static_cast<double>(s.layerHeight),
                  static_cast<double>(s.lineWidth), s.walls, static_cast<double>(s.infill) * 100.0, static_cast<double>(s.filament));
    g.line(b);
    g.line("M107");
    std::snprintf(b, sizeof(b), "M140 S%.0f", static_cast<double>(s.bedTemp));
    g.line(b);
    std::snprintf(b, sizeof(b), "M104 S%.0f", static_cast<double>(s.nozzleTemp));
    g.line(b);
    std::snprintf(b, sizeof(b), "M190 S%.0f", static_cast<double>(s.bedTemp));
    g.line(b);
    std::snprintf(b, sizeof(b), "M109 S%.0f", static_cast<double>(s.nozzleTemp));
    g.line(b);
    g.line("G28 ; home");
    g.line("G90 ; absolute positions");
    g.line("M82 ; absolute extrusion");
    g.line("G92 E0");
    const double w = s.lineWidth;
    for (int k = 0; k < layers; ++k) {
        double z = (k + 0.5) * s.layerHeight;
        // Keep the plane off the corners (a corner exactly on it would split a contour).
        for (int guard = 0; guard < 8; ++guard) {
            bool onCorner = false;
            for (const std::uint32_t ti : bucket[static_cast<std::size_t>(k)])
                for (const std::uint32_t v : tris[ti]) onCorner = onCorner || std::fabs(p[v].z - z) < 1e-7;
            if (!onCorner) break;
            z += 1e-5;
        }
        const std::vector<Loop> outline = contours(p, tris, bucket[static_cast<std::size_t>(k)], z);
        g.layer(k, (k + 1) * static_cast<double>(s.layerHeight));
        if (outline.empty()) continue;
        // Walls from the outside in, each half a line inside the last.
        std::vector<Loop> inner = outline;
        for (int wall = 0; wall < std::max(s.walls, 0); ++wall) {
            const std::vector<Loop> ring = offsetLeft(outline, (wall + 0.5) * w);
            g.line(wall == 0 ? ";TYPE:WALL-OUTER" : ";TYPE:WALL-INNER");
            for (const Loop& l : ring) {
                g.travel(l.front());
                for (std::size_t i = 1; i <= l.size(); ++i) g.extrude(l[i % l.size()]);
            }
            inner = offsetLeft(outline, (wall + 1.0) * w);
        }
        // Infill inside the walls: solid at the top and bottom, sparse in between.
        const bool solid = k < s.bottomLayers || k >= layers - s.topLayers;
        const double fill = solid ? 1.0 : std::clamp(static_cast<double>(s.infill), 0.0, 1.0);
        if (fill > 0.0) {
            g.line(solid ? ";TYPE:SOLID-FILL" : ";TYPE:FILL");
            const double angle = (k % 2 == 0 ? 1.0 : -1.0) * kPi * 0.25;
            for (const auto& [a, c] : hatch(inner, w / fill, angle)) {
                g.travel(a);
                g.extrude(c);
            }
        }
    }
    g.line("; end");
    g.line("M107");
    g.line("M104 S0");
    g.line("M140 S0");
    g.line("G91");
    g.line("G1 Z10 F600");
    g.line("G90");
    g.line("M84");
    std::ofstream f(path);
    if (!f) {
        if (error) *error = "Cannot write " + path;
        return false;
    }
    f << g.m_text;
    if (report) {
        report->layers = layers;
        report->filamentMm = g.m_filament;
        report->seconds = g.m_seconds;
        report->moves = g.m_moves;
    }
    return static_cast<bool>(f);
}

} // namespace sicdm

#include "core/Mold.hpp"
#include "core/Brep.hpp"
#include "core/Bvh.hpp"
#include "core/Sculpt.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <functional>
#include <map>
#include <tuple>
#include <unordered_map>

namespace sicdm {

namespace {

constexpr double kRad = 3.14159265358979323846 / 180.0;
constexpr double kUpright = 1e-2; // |n . pull| below this (0.57 deg, less than moulding asks): an upright face, no draft to speak of (booleans and roundings leave walls a hair off)

bool fail(std::string* why, const std::string& w) {
    if (why) *why = w;
    return false;
}

// A closed part as welded triangles, in double.
struct Part {
    std::vector<DVec3> p;
    std::vector<std::array<int, 3>> t;
    double size = 1;
};

std::uint64_t edgeKey(int a, int b) { return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(a)) << 32) | static_cast<std::uint32_t>(b); }

// The same points made one (exactly, then those on open edges within a hair: a
// primitive's seams); slivers dropped. False when what is left isn't closed: every edge
// shared by two triangles running opposite ways.
bool weld(const MeshData& m, Part& out, std::string* why) {
    out = Part{};
    AABB box;
    for (const Vertex& v : m.vertices) box.expand(v.position);
    if (m.indices.size() < 12 || !box.valid()) return fail(why, "The part has no solid to mould");
    out.size = std::max(static_cast<double>(length(box.max - box.min)), 1e-9);
    // Exactly the same points.
    struct Key {
        float x, y, z;
        bool operator==(const Key& o) const { return x == o.x && y == o.y && z == o.z; }
    };
    struct KeyHash {
        std::size_t operator()(const Key& k) const {
            std::uint32_t a, b, c;
            std::memcpy(&a, &k.x, 4), std::memcpy(&b, &k.y, 4), std::memcpy(&c, &k.z, 4);
            return (static_cast<std::size_t>(a) * 73856093u) ^ (static_cast<std::size_t>(b) * 19349663u) ^ (static_cast<std::size_t>(c) * 83492791u);
        }
    };
    std::unordered_map<Key, int, KeyHash> same;
    std::vector<int> id(m.vertices.size());
    for (std::size_t i = 0; i < m.vertices.size(); ++i) {
        const Vec3& q = m.vertices[i].position;
        const auto [it, added] = same.try_emplace(Key{q.x + 0.0f, q.y + 0.0f, q.z + 0.0f}, static_cast<int>(out.p.size()));
        if (added) out.p.push_back(DVec3(q));
        id[i] = it->second;
    }
    std::vector<std::array<int, 3>> tris;
    for (std::size_t k = 0; k + 2 < m.indices.size(); k += 3) tris.push_back({id[m.indices[k]], id[m.indices[k + 1]], id[m.indices[k + 2]]});
    auto counts = [](const std::vector<std::array<int, 3>>& t) {
        std::unordered_map<std::uint64_t, int> c;
        for (const auto& f : t)
            if (f[0] != f[1] && f[1] != f[2] && f[0] != f[2])
                for (int e = 0; e < 3; ++e) ++c[edgeKey(f[static_cast<std::size_t>(e)], f[static_cast<std::size_t>((e + 1) % 3)])];
        return c;
    };
    // Points on open edges a hair apart (seams): one.
    {
        const auto c = counts(tris);
        std::vector<char> open(out.p.size(), 0);
        for (const auto& [k, n] : c)
            if (!c.count(edgeKey(static_cast<int>(k & 0xFFFFFFFFu), static_cast<int>(k >> 32)))) open[k >> 32] = open[k & 0xFFFFFFFFu] = 1;
        const double tol = 1e-5 * out.size;
        std::vector<int> to(out.p.size());
        std::unordered_map<std::uint64_t, std::vector<int>> grid;
        auto cell = [&](double x) { return static_cast<std::int64_t>(std::floor(x / tol)); };
        auto key = [](std::int64_t a, std::int64_t b, std::int64_t c3) {
            return (static_cast<std::uint64_t>(a & 0x1FFFFF) << 42) | (static_cast<std::uint64_t>(b & 0x1FFFFF) << 21) | static_cast<std::uint64_t>(c3 & 0x1FFFFF);
        };
        for (std::size_t i = 0; i < out.p.size(); ++i) {
            to[i] = static_cast<int>(i);
            if (!open[i]) continue;
            const DVec3 q = out.p[i];
            const std::int64_t cx = cell(q.x), cy = cell(q.y), cz = cell(q.z);
            int found = -1;
            for (int dx = -1; dx <= 1 && found < 0; ++dx)
                for (int dy = -1; dy <= 1 && found < 0; ++dy)
                    for (int dz = -1; dz <= 1 && found < 0; ++dz) {
                        const auto it = grid.find(key(cx + dx, cy + dy, cz + dz));
                        if (it == grid.end()) continue;
                        for (int j : it->second)
                            if (dlength(out.p[static_cast<std::size_t>(j)] - q) <= tol) {
                                found = j;
                                break;
                            }
                    }
            if (found >= 0) to[i] = found;
            else grid[key(cx, cy, cz)].push_back(static_cast<int>(i));
        }
        for (auto& f : tris)
            for (int& v : f) v = to[static_cast<std::size_t>(v)];
    }
    for (const auto& f : tris)
        if (f[0] != f[1] && f[1] != f[2] && f[0] != f[2]) out.t.push_back(f);
    const auto c = counts(out.t);
    for (const auto& [k, n] : c) {
        const auto twin = c.find(edgeKey(static_cast<int>(k & 0xFFFFFFFFu), static_cast<int>(k >> 32)));
        if (n != 1 || twin == c.end() || twin->second != 1)
            return fail(why, "The part isn't a closed solid (it has open or shared edges): a mould needs one (REPAIR may close it)");
    }
    return true;
}

DVec3 normalOf(const Part& m, const std::array<int, 3>& t, double* area = nullptr) {
    const DVec3 n = dcross(m.p[static_cast<std::size_t>(t[1])] - m.p[static_cast<std::size_t>(t[0])], m.p[static_cast<std::size_t>(t[2])] - m.p[static_cast<std::size_t>(t[0])]);
    if (area) *area = 0.5 * dlength(n);
    return dnormalize(n);
}

// The part cut at height h along d: triangles crossing it split, points within a hair
// of it put on it.
Part splitAt(const Part& in, const DVec3& d, double h) {
    Part m = in;
    const double tol = 1e-5 * m.size; // as welded: closer points are one
    for (DVec3& q : m.p) {
        const double z = ddot(q, d);
        if (std::fabs(z - h) <= tol) q = q + d * (h - z);
    }
    auto side = [&](int i) {
        const double z = ddot(m.p[static_cast<std::size_t>(i)], d) - h;
        return z > tol ? 1 : z < -tol ? -1 : 0;
    };
    std::unordered_map<std::uint64_t, int> made;
    auto cut = [&](int a, int b) {
        const std::uint64_t k = edgeKey(std::min(a, b), std::max(a, b));
        if (const auto it = made.find(k); it != made.end()) return it->second;
        const DVec3 pa = m.p[static_cast<std::size_t>(a)], pb = m.p[static_cast<std::size_t>(b)];
        const double za = ddot(pa, d) - h, zb = ddot(pb, d) - h;
        DVec3 q = pa + (pb - pa) * (za / (za - zb));
        q = q + d * (h - ddot(q, d));
        m.p.push_back(q);
        return made[k] = static_cast<int>(m.p.size()) - 1;
    };
    std::vector<std::array<int, 3>> out;
    for (const auto& t : m.t) {
        const int s[3] = {side(t[0]), side(t[1]), side(t[2])};
        const bool crosses = (s[0] > 0 || s[1] > 0 || s[2] > 0) && (s[0] < 0 || s[1] < 0 || s[2] < 0);
        if (!crosses) {
            out.push_back(t);
            continue;
        }
        // First the corner on the plane (the other two then lie either side), else the one
        // alone on its side.
        int k = 0;
        if (s[0] == 0 || s[1] == 0 || s[2] == 0) {
            while (s[k] != 0) ++k;
        } else {
            while (s[k] == s[(k + 1) % 3] || s[k] == s[(k + 2) % 3]) ++k;
        }
        const int a = t[static_cast<std::size_t>(k)], b = t[static_cast<std::size_t>((k + 1) % 3)], c = t[static_cast<std::size_t>((k + 2) % 3)];
        if (s[k] == 0) { // through the corner: two triangles
            const int q = cut(b, c);
            out.push_back({a, b, q}), out.push_back({a, q, c});
        } else { // a alone: three
            const int qab = cut(a, b), qac = cut(a, c);
            out.push_back({a, qab, qac}), out.push_back({qab, b, c}), out.push_back({qab, c, qac});
        }
    }
    m.t = std::move(out);
    return m;
}

// Faces inside the solid (a crack's two walls, as roundings leave where they meet): just
// off them on their outward side is still inside, by the parity of a ray's crossings.
std::vector<char> insideFaces(const MeshData& m, const MeshBvh& bvh, float eps) {
    const std::size_t n = m.indices.size() / 3;
    std::vector<char> out(n, 0);
    const Vec3 dir = normalize(Vec3{0.5773f, 0.5774f, 0.5772f} + Vec3{0.0123f, -0.0071f, 0.0049f});
    for (std::size_t i = 0; i < n; ++i) {
        const Vec3 a = m.vertices[m.indices[3 * i]].position, b = m.vertices[m.indices[3 * i + 1]].position, c = m.vertices[m.indices[3 * i + 2]].position;
        const Vec3 nn = cross(b - a, c - a);
        if (!(length(nn) > 0)) continue;
        Ray r;
        r.origin = (a + b + c) / 3.0f + normalize(nn) * eps;
        r.dir = dir;
        int crossings = 0;
        for (int guard = 0; guard < 64; ++guard) {
            float t = 1e30f;
            if (!bvh.raycast(r, t)) break;
            ++crossings;
            r.origin = r.origin + dir * (t + eps);
        }
        out[i] = crossings % 2 == 1;
    }
    return out;
}

// The cavity side (1) or the core side (0) of each triangle, and the parting loops
// (point ids, round the cavity's faces).
struct Partition {
    Part part;
    std::vector<char> cavity;
    std::vector<std::vector<int>> loops;
    std::unordered_map<int, char> fold; // a loop edge (by its first point) between faces turned to and away from the pull
};

bool partition(const Part& welded, const DVec3& d, double h, bool split, Partition& out, std::string* why) {
    out.part = split ? splitAt(welded, d, h) : welded;
    const Part& m = out.part;
    const std::size_t n = m.t.size();
    std::unordered_map<std::uint64_t, int> owner;
    for (std::size_t i = 0; i < n; ++i)
        for (int e = 0; e < 3; ++e) owner[edgeKey(m.t[i][static_cast<std::size_t>(e)], m.t[i][static_cast<std::size_t>((e + 1) % 3)])] = static_cast<int>(i);
    auto across = [&](std::size_t i, int e) {
        const auto it = owner.find(edgeKey(m.t[i][static_cast<std::size_t>((e + 1) % 3)], m.t[i][static_cast<std::size_t>(e)]));
        return it == owner.end() ? -1 : it->second;
    };
    std::vector<double> s(n);
    std::vector<DVec3> nrm(n), mid(n);
    for (std::size_t i = 0; i < n; ++i) {
        nrm[i] = normalOf(m, m.t[i]);
        s[i] = ddot(nrm[i], d);
        mid[i] = (m.p[static_cast<std::size_t>(m.t[i][0])] + m.p[static_cast<std::size_t>(m.t[i][1])] + m.p[static_cast<std::size_t>(m.t[i][2])]) / 3.0;
    }
    // An upright face goes with what lies outside it: just outside, with part behind along
    // the pull (a floor), it is the cavity's; with part ahead (a ceiling), the core's; with
    // neither (the part's side), it parts at h. Looked at from its middle and near each corner.
    MeshData probe;
    for (const DVec3& q : m.p) {
        Vertex vx;
        vx.position = q.f();
        probe.vertices.push_back(vx);
    }
    for (const auto& t : m.t)
        for (int c : t) probe.indices.push_back(static_cast<std::uint32_t>(c));
    const MeshBvh bvh(probe);
    const float eps = static_cast<float>(1e-4 * m.size);
    // Seen from just off the face and from further out (past the crumbs a rounding leaves
    // where it meets a wall, a hundredth or two proud of it): only part both see counts.
    auto looks = [&](const DVec3& at, const DVec3& nn) {
        int seen = 3;
        for (const double off : {static_cast<double>(eps), 2e-3 * m.size}) {
            const Vec3 o = (at + nn * off).f();
            int here = 0;
            for (int dir = 0; dir < 2; ++dir) {
                Ray r;
                r.origin = o;
                r.dir = (dir == 0 ? -d : d).f();
                float t = 1e30f;
                // 1: part behind (a floor), 2: ahead (a ceiling); grazing a curve close by is neither.
                if (bvh.raycast(r, t) && t > 2e-3f * static_cast<float>(m.size)) here |= dir == 0 ? 1 : 2;
            }
            seen &= here;
        }
        return seen;
    };
    // Slivers (a boolean's leftovers, all but flat) have no direction of their own: they go
    // with most of their neighbours.
    std::vector<char> sliver(n, 0);
    for (std::size_t i = 0; i < n; ++i) {
        double area = 0, longest = 0;
        normalOf(m, m.t[i], &area);
        for (int e = 0; e < 3; ++e)
            longest = std::max(longest, dlength(m.p[static_cast<std::size_t>(m.t[i][static_cast<std::size_t>(e)])] - m.p[static_cast<std::size_t>(m.t[i][static_cast<std::size_t>((e + 1) % 3)])]));
        sliver[i] = 2 * area < 1e-5 * longest * longest;
        // Points are good to about a hundred-thousandth of the part: that tips a face's normal
        // by as much over its narrowest width. A thin face leaning less than that is upright.
        const double width = longest > 0 ? 2 * area / longest : 0;
        if (std::fabs(s[i]) > kUpright && width > 0 && std::fabs(s[i]) * width < 2e-5 * m.size) s[i] = 0;
    }
    {
        const std::vector<char> inside = insideFaces(probe, bvh, eps);
        for (std::size_t i = 0; i < n; ++i) sliver[i] = sliver[i] || inside[i];
    }
    out.cavity.assign(n, 0);
    for (std::size_t i = 0; i < n; ++i) {
        if (sliver[i]) continue;
        if (s[i] > kUpright) {
            out.cavity[i] = 1;
            continue;
        }
        if (s[i] < -kUpright) continue;
        const DVec3 c[3] = {m.p[static_cast<std::size_t>(m.t[i][0])], m.p[static_cast<std::size_t>(m.t[i][1])], m.p[static_cast<std::size_t>(m.t[i][2])]};
        const int seen = looks(mid[i], nrm[i]);
        bool clear = true;
        for (int k = 0; k < 3 && clear; ++k) clear = looks(c[k] * 0.6 + c[(k + 1) % 3] * 0.2 + c[(k + 2) % 3] * 0.2, nrm[i]) == seen;
        if (!clear) { // partly over the part and partly beside it: as its neighbours
            sliver[i] = 1;
            continue;
        }
        out.cavity[i] = seen == 1 || (seen == 0 && ddot(mid[i], d) > h) ? 1 : 0;
    }
    // By the length they share: a sliver follows the faces along its long sides, not one
    // touching its tip (and waits while those are slivers still to be settled).
    constexpr int kPasses = 64;
    for (int pass = 0; pass < kPasses; ++pass) {
        bool left = false;
        for (std::size_t i = 0; i < n; ++i) {
            if (sliver[i] != 1) continue;
            double votes[2] = {0, 0}, round = 0;
            for (int e = 0; e < 3; ++e) {
                const double len = dlength(m.p[static_cast<std::size_t>(m.t[i][static_cast<std::size_t>(e)])] - m.p[static_cast<std::size_t>(m.t[i][static_cast<std::size_t>((e + 1) % 3)])]);
                round += len;
                if (const int j = across(i, e); j >= 0 && !sliver[static_cast<std::size_t>(j)]) votes[out.cavity[static_cast<std::size_t>(j)] ? 1 : 0] += len;
            }
            if (votes[0] + votes[1] == 0 || (votes[0] + votes[1] < 0.5 * round && pass + 1 < kPasses)) {
                left = true;
                continue;
            }
            out.cavity[i] = votes[1] > votes[0] ? 1 : 0;
            sliver[i] = 2; // settled: its neighbours may go by it
        }
        if (!left) break;
        for (char& f : sliver)
            if (f == 2) f = 0;
    }
    for (std::size_t i = 0; i < n; ++i) // slivers among slivers only: by their own facing
        if (sliver[i] == 1) out.cavity[i] = s[i] > 0 ? 1 : 0;
    // Specks: a patch of either side smaller than a hundred-thousandth of the part's size
    // squared (a mesh's crumbs at a crease: steps of a few hundredths where roundings meet), or
    // one of faces all within a degree of upright smaller than a hundredth (a rounding's facets
    // wavering about the pull where it turns), goes with what is round it.
    {
        std::vector<int> comp(n, -1);
        const double speck = 1e-5 * m.size * m.size;
        for (std::size_t i = 0; i < n; ++i) {
            if (comp[i] >= 0) continue;
            std::vector<std::size_t> members{i}, stack{i};
            comp[i] = static_cast<int>(i);
            double area = 0, steepest = 0;
            int votes[2] = {0, 0};
            while (!stack.empty()) {
                const std::size_t k = stack.back();
                stack.pop_back();
                double a = 0;
                normalOf(m, m.t[k], &a);
                area += a;
                steepest = std::max(steepest, std::fabs(s[k]));
                for (int e = 0; e < 3; ++e) {
                    const int j = across(k, e);
                    if (j < 0) continue;
                    const std::size_t uj = static_cast<std::size_t>(j);
                    if (out.cavity[uj] != out.cavity[i]) {
                        ++votes[out.cavity[uj] ? 1 : 0];
                        continue;
                    }
                    if (comp[uj] < 0) comp[uj] = static_cast<int>(i), stack.push_back(uj), members.push_back(uj);
                }
            }
            if ((area < speck || (steepest < 0.02 && area < 100 * speck)) && votes[0] + votes[1] > 0)
                for (std::size_t k : members) out.cavity[k] = out.cavity[k] ? 0 : 1;
        }
    }
    // The loops: the cavity's edges with the core across them, each followed by the next one
    // round the cavity's faces at its end (where the line passes through a point twice, the
    // two passes stay apart).
    auto isCavity = [&](int t) { return t >= 0 && out.cavity[static_cast<std::size_t>(t)]; };
    std::unordered_map<std::uint64_t, char> used;
    out.loops.clear();
    out.fold.clear();
    for (std::size_t i0 = 0; i0 < n; ++i0) {
        if (!out.cavity[i0]) continue;
        for (int e0 = 0; e0 < 3; ++e0) {
            if (isCavity(across(i0, e0))) continue;
            const int a0 = m.t[i0][static_cast<std::size_t>(e0)], b0 = m.t[i0][static_cast<std::size_t>((e0 + 1) % 3)];
            if (used.count(edgeKey(a0, b0))) continue;
            std::vector<int> loop;
            std::size_t t = i0;
            int e = e0;
            for (std::size_t guard = 0;; ++guard) {
                const int a = m.t[t][static_cast<std::size_t>(e)], b = m.t[t][static_cast<std::size_t>((e + 1) % 3)];
                if (used.count(edgeKey(a, b))) {
                    if (a != a0 || b != b0) return fail(why, "The parting line doesn't close round the part");
                    break;
                }
                if (guard > 4 * n) return fail(why, "The parting line doesn't close round the part");
                used[edgeKey(a, b)] = 1;
                loop.push_back(a);
                const int j = across(t, e);
                out.fold[a] = j >= 0 && std::fabs(s[t]) > kUpright && std::fabs(s[static_cast<std::size_t>(j)]) > kUpright;
                // Round b through the cavity's faces to the next edge with the core across.
                int ne = (e + 1) % 3; // b -> c in this face
                for (int spin = 0;; ++spin) {
                    const int k = across(t, ne);
                    if (!isCavity(k)) break;
                    if (spin > 64) return fail(why, "The parting line doesn't close round the part");
                    // In the face across, the edge leaving b.
                    const std::size_t tk = static_cast<std::size_t>(k);
                    int f = 0;
                    while (m.t[tk][static_cast<std::size_t>(f)] != b) ++f;
                    t = tk, ne = f;
                }
                e = ne;
            }
            if (loop.size() < 3) return fail(why, "The parting line doesn't close round the part");
            out.loops.push_back(std::move(loop));
        }
    }
    if (out.loops.empty()) return fail(why, "The part has no parting line along this pull (every face on one side)");
    return true;
}

// The height whose shut-off walls are least: the median, by length, of the loops' folds
// (where faces turn from the pull to away from it: the true parting line), or of the
// whole loops when they run only round upright walls.
double medianHeight(const Partition& pt, const DVec3& d) {
    std::vector<std::pair<double, double>> zw; // height, weight
    double total = 0;
    bool folds = false;
    for (const auto& [a, f] : pt.fold) folds = folds || f;
    for (const auto& loop : pt.loops)
        for (std::size_t k = 0; k < loop.size(); ++k) {
            if (folds && !pt.fold.at(loop[k])) continue;
            const DVec3 a = pt.part.p[static_cast<std::size_t>(loop[k])], b = pt.part.p[static_cast<std::size_t>(loop[(k + 1) % loop.size()])];
            const double w = dlength(b - a);
            zw.push_back({ddot(a, d), 0.5 * w}), zw.push_back({ddot(b, d), 0.5 * w});
            total += w;
        }
    std::sort(zw.begin(), zw.end());
    double acc = 0;
    for (const auto& [z, w] : zw)
        if ((acc += w) >= 0.5 * total) return z;
    return zw.empty() ? 0 : zw.back().first;
}

PartingLine describeLoops(const Partition& pt, const DVec3& d, double h) {
    PartingLine pl;
    pl.height = h;
    bool first = true;
    for (const auto& loop : pt.loops) {
        std::vector<Vec3> l;
        for (std::size_t k = 0; k < loop.size(); ++k) {
            const DVec3 q = pt.part.p[static_cast<std::size_t>(loop[k])];
            l.push_back(q.f());
            const double z = ddot(q, d);
            pl.low = first ? z : std::min(pl.low, z), pl.high = first ? z : std::max(pl.high, z);
            first = false;
            pl.length += dlength(pt.part.p[static_cast<std::size_t>(loop[(k + 1) % loop.size()])] - q);
        }
        pl.loops.push_back(std::move(l));
    }
    return pl;
}

DVec3 unitPull(Vec3 pull) {
    const DVec3 d = dnormalize(DVec3(pull));
    return dlength(d) > 0 ? d : DVec3{0, 0, 1};
}

// --- 2D: the parting plane's faces ------------------------------------------------------

struct P2 {
    double x, y;
};

double orient(P2 a, P2 b, P2 c) { return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x); }

double polyArea(const std::vector<P2>& q, const std::vector<int>& l) {
    double a = 0;
    for (std::size_t i = 0; i < l.size(); ++i) {
        const P2 p = q[static_cast<std::size_t>(l[i])], r = q[static_cast<std::size_t>(l[(i + 1) % l.size()])];
        a += p.x * r.y - r.x * p.y;
    }
    return 0.5 * a;
}

bool insidePoly(const std::vector<P2>& q, const std::vector<int>& l, P2 p) {
    bool in = false;
    for (std::size_t i = 0, j = l.size() - 1; i < l.size(); j = i++) {
        const P2 a = q[static_cast<std::size_t>(l[i])], b = q[static_cast<std::size_t>(l[j])];
        if ((a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) in = !in;
    }
    return in;
}

bool segmentsCross(P2 a, P2 b, P2 c, P2 d, double eps) {
    const double d1 = orient(c, d, a), d2 = orient(c, d, b), d3 = orient(a, b, c), d4 = orient(a, b, d);
    if (((d1 > eps && d2 < -eps) || (d1 < -eps && d2 > eps)) && ((d3 > eps && d4 < -eps) || (d3 < -eps && d4 > eps))) return true;
    auto on = [&](P2 p, P2 q, P2 r, double o) { // r on segment pq
        return std::fabs(o) <= eps && r.x >= std::min(p.x, q.x) - 1e-12 && r.x <= std::max(p.x, q.x) + 1e-12 && r.y >= std::min(p.y, q.y) - 1e-12 &&
               r.y <= std::max(p.y, q.y) + 1e-12;
    };
    return on(c, d, a, d1) || on(c, d, b, d2) || on(a, b, c, d3) || on(a, b, d, d4);
}

// A polygon with holes (outer counter-clockwise, holes clockwise; ids into q) cut into
// triangles (counter-clockwise) using only its own points: each hole joined to the outside
// by a cut to a point it can see, then ears clipped.
bool triangulateRegion(const std::vector<P2>& q, std::vector<int> outer, std::vector<std::vector<int>> holes, double eps, std::vector<std::array<int, 3>>& tris) {
    auto P = [&](int i) { return q[static_cast<std::size_t>(i)]; };
    std::sort(holes.begin(), holes.end(), [&](const std::vector<int>& a, const std::vector<int>& b) {
        double ax = -1e300, bx = -1e300;
        for (int i : a) ax = std::max(ax, P(i).x);
        for (int i : b) bx = std::max(bx, P(i).x);
        return ax > bx;
    });
    for (const std::vector<int>& hole : holes) {
        std::size_t mi = 0;
        for (std::size_t i = 1; i < hole.size(); ++i)
            if (P(hole[i]).x > P(hole[mi]).x) mi = i;
        const P2 M = P(hole[mi]);
        // The nearest crossing of the ray to +x with the outside's edges.
        double bestX = 1e300;
        std::size_t edge = outer.size();
        for (std::size_t i = 0; i < outer.size(); ++i) {
            const P2 a = P(outer[i]), b = P(outer[(i + 1) % outer.size()]);
            if ((a.y > M.y) == (b.y > M.y)) continue;
            const double x = a.x + (M.y - a.y) * (b.x - a.x) / (b.y - a.y);
            if (x >= M.x && x < bestX) bestX = x, edge = i;
        }
        if (edge == outer.size()) return false;
        const P2 I{bestX, M.y};
        std::size_t pick = P(outer[edge]).x > P(outer[(edge + 1) % outer.size()]).x ? edge : (edge + 1) % outer.size();
        // A point of the outside in the triangle M, I, pick (a reflex one) hides it: the one
        // nearest the ray's direction instead.
        const P2 Pp = P(outer[pick]);
        double bestCos = -2, bestD = 1e300;
        for (std::size_t i = 0; i < outer.size(); ++i) {
            if (i == pick) continue;
            const P2 r = P(outer[i]);
            if (r.x == Pp.x && r.y == Pp.y) continue;
            const double o1 = orient(M, I, r), o2 = orient(I, Pp, r), o3 = orient(Pp, M, r);
            const bool in = (o1 >= -eps && o2 >= -eps && o3 >= -eps) || (o1 <= eps && o2 <= eps && o3 <= eps);
            if (!in) continue;
            const P2 pr = P(outer[(i + outer.size() - 1) % outer.size()]), nx = P(outer[(i + 1) % outer.size()]);
            if (orient(pr, r, nx) > eps) continue; // convex: can't hide
            const double dx = r.x - M.x, dy = r.y - M.y, dl = std::hypot(dx, dy);
            if (dl <= 0) continue;
            const double c = dx / dl;
            if (c > bestCos + 1e-12 || (std::fabs(c - bestCos) <= 1e-12 && dl < bestD)) bestCos = c, bestD = dl, pick = i;
        }
        std::vector<int> spliced(outer.begin(), outer.begin() + static_cast<std::ptrdiff_t>(pick) + 1);
        for (std::size_t k = 0; k <= hole.size(); ++k) spliced.push_back(hole[(mi + k) % hole.size()]);
        spliced.push_back(outer[pick]);
        spliced.insert(spliced.end(), outer.begin() + static_cast<std::ptrdiff_t>(pick) + 1, outer.end());
        outer = std::move(spliced);
    }
    std::vector<int> l = std::move(outer);
    auto inTri = [&](P2 a, P2 b, P2 c, P2 p) { return orient(a, b, p) >= -eps && orient(b, c, p) >= -eps && orient(c, a, p) >= -eps; };
    auto same = [](P2 a, P2 b) { return a.x == b.x && a.y == b.y; };
    // Each round clips one ear: no more rounds than points (the bound from the start: the
    // polygon shrinks as it goes).
    const std::size_t rounds = l.size() + 16;
    std::size_t guard = 0;
    while (l.size() > 3) {
        if (++guard > rounds) return false;
        const std::size_t n = l.size();
        bool clipped = false;
        for (int pass = 0; pass < 2 && !clipped; ++pass)
            for (std::size_t i = 0; i < n && !clipped; ++i) {
                const std::size_t ip = (i + n - 1) % n, in = (i + 1) % n;
                const P2 a = P(l[ip]), b = P(l[i]), c = P(l[in]);
                const double o = orient(a, b, c);
                if (pass == 0 ? !(o > eps) : !(std::fabs(o) <= eps)) continue; // first convex ears, then straight runs
                bool ear = true;
                if (pass == 0)
                    for (std::size_t j = 0; j < n && ear; ++j) {
                        if (j == ip || j == i || j == in) continue;
                        const P2 r = P(l[j]);
                        if (same(r, a) || same(r, b) || same(r, c)) continue;
                        if (inTri(a, b, c, r)) ear = false;
                    }
                if (!ear) continue;
                tris.push_back({l[ip], l[i], l[in]});
                l.erase(l.begin() + static_cast<std::ptrdiff_t>(i));
                clipped = true;
            }
        if (!clipped) return false;
    }
    if (l.size() == 3) tris.push_back({l[0], l[1], l[2]});
    return true;
}

} // namespace

Vec3 draftColour(DraftClass c) noexcept {
    switch (c) {
    case DraftClass::Positive: return {0.22f, 0.72f, 0.30f};
    case DraftClass::Negative: return {0.86f, 0.26f, 0.20f};
    case DraftClass::Shallow: return {0.96f, 0.80f, 0.16f};
    case DraftClass::Undercut: return {0.72f, 0.24f, 0.86f};
    }
    return {1, 1, 1};
}

std::vector<DraftClass> draftClasses(const MeshData& world, Vec3 pull, float minDeg, DraftReport* report) {
    const DVec3 d = unitPull(pull);
    const Vec3 df = d.f();
    const std::size_t n = world.indices.size() / 3;
    std::vector<DraftClass> out(n, DraftClass::Shallow);
    DraftReport rep;
    rep.triangles = n;
    const AABB box = world.bounds();
    const float size = box.valid() ? length(box.max - box.min) : 1.0f;
    const float eps = 2e-4f * size;
    const MeshBvh bvh(world);
    const std::vector<char> inside = insideFaces(world, bvh, eps);
    for (std::size_t i = 0; i < n; ++i) {
        const Vec3 a = world.vertices[world.indices[3 * i]].position, b = world.vertices[world.indices[3 * i + 1]].position, c = world.vertices[world.indices[3 * i + 2]].position;
        const DVec3 nn = dcross(DVec3(b) - DVec3(a), DVec3(c) - DVec3(a));
        const double area = 0.5 * dlength(nn);
        if (!(area > 0)) continue;
        const DVec3 un = dnormalize(nn);
        const double s = ddot(un, d);
        const double deg = std::asin(std::clamp(s, -1.0, 1.0)) / kRad;
        DraftClass k = deg >= minDeg ? DraftClass::Positive : deg <= -minDeg ? DraftClass::Negative : DraftClass::Shallow;
        if (std::fabs(s) > kUpright && !inside[i]) { // would anything of the part stop it coming away?
            const Vec3 centre = (a + b + c) / 3.0f;
            const Vec3 dir = s > 0 ? df : -df;
            Ray r;
            r.origin = centre + un.f() * eps + dir * eps;
            r.dir = dir;
            float t = 1e30f;
            // (A hit closer than a thousandth of the part is a mesh's crease, not a wall in the way.)
            if (bvh.raycast(r, t) && t > 1e-3f * size) k = DraftClass::Undercut;
        }
        out[i] = k;
        rep.area[static_cast<int>(k)] += area;
        if (deg > 1e-3) rep.minPositive = std::min(rep.minPositive, deg);
        if (deg < -1e-3) rep.minNegative = std::min(rep.minNegative, -deg);
    }
    if (report) *report = rep;
    return out;
}

MeshData draftColoured(const MeshData& world, const std::vector<DraftClass>& classes) {
    MeshData m;
    const std::size_t n = std::min(world.indices.size() / 3, classes.size());
    m.vertices.reserve(3 * n);
    m.indices.reserve(3 * n);
    for (std::size_t i = 0; i < n; ++i) {
        const std::uint32_t colour = packColor(draftColour(classes[i]));
        const Vec3 a = world.vertices[world.indices[3 * i]].position, b = world.vertices[world.indices[3 * i + 1]].position, c = world.vertices[world.indices[3 * i + 2]].position;
        const Vec3 nn = normalize(cross(b - a, c - a));
        for (const Vec3& p : {a, b, c}) {
            Vertex v;
            v.position = p, v.normal = nn, v.color = colour;
            m.indices.push_back(static_cast<std::uint32_t>(m.vertices.size()));
            m.vertices.push_back(v);
        }
    }
    return m;
}

bool partingLine(const MeshData& world, Vec3 pull, std::optional<double> at, PartingLine& out, std::string* why) {
    Part part;
    if (!weld(world, part, why)) return false;
    const DVec3 d = unitPull(pull);
    double h = 0;
    Partition pt;
    if (at) {
        h = *at;
    } else {
        if (!partition(part, d, -1e300, false, pt, why)) return false;
        h = medianHeight(pt, d);
    }
    if (!partition(part, d, h, true, pt, why)) return false;
    out = describeLoops(pt, d, h);
    return true;
}

bool moldSealed(const MeshData& m) {
    std::map<std::tuple<float, float, float>, std::uint32_t> at; // the same position, one point
    std::vector<std::uint32_t> r(m.vertices.size());
    for (std::size_t i = 0; i < m.vertices.size(); ++i) {
        const Vec3& p = m.vertices[i].position;
        r[i] = at.try_emplace({p.x + 0.0f, p.y + 0.0f, p.z + 0.0f}, static_cast<std::uint32_t>(at.size())).first->second;
    }
    std::unordered_map<std::uint64_t, int> balance; // +1 for each use one way, -1 the other
    for (std::size_t k = 0; k + 2 < m.indices.size(); k += 3)
        for (std::size_t e = 0; e < 3; ++e) {
            const std::uint32_t a = r[m.indices[k + e]], b = r[m.indices[k + (e + 1) % 3]];
            if (a == b) continue;
            balance[a < b ? (static_cast<std::uint64_t>(a) << 32 | b) : (static_cast<std::uint64_t>(b) << 32 | a)] += a < b ? 1 : -1;
        }
    for (const auto& [k, n] : balance)
        if (n != 0) return false;
    return !m.indices.empty();
}

bool splitMold(const MeshData& worldIn, const MoldOptions& o, MoldHalves& out, std::string* why) {
    out = MoldHalves{};
    if (o.shrink < 0 || o.shrink > 20) return fail(why, "SHRINK is a percentage from 0 to 20");
    MeshData world = worldIn;
    const AABB box0 = world.bounds();
    if (!box0.valid()) return fail(why, "The part has no solid to mould");
    if (o.shrink > 0) { // grown about its middle by its shrinkage
        const Vec3 c = (box0.min + box0.max) * 0.5f;
        const float f = 1.0f + o.shrink / 100.0f;
        for (Vertex& v : world.vertices) v.position = c + (v.position - c) * f;
    }
    Part part;
    if (!weld(world, part, why)) return false;
    const DVec3 d = unitPull(o.pull);
    // Undercuts: no two-part mould takes the part out. (Specks of them, a mesh's crumbs at a
    // crease, don't count.)
    {
        MeshData probe;
        for (const DVec3& q : part.p) {
            Vertex vx;
            vx.position = q.f();
            probe.vertices.push_back(vx);
        }
        for (const auto& t : part.t)
            for (int c : t) probe.indices.push_back(static_cast<std::uint32_t>(c));
        const std::vector<DraftClass> cls = draftClasses(probe, o.pull, 0.0f);
        std::unordered_map<std::uint64_t, int> owner;
        for (std::size_t i = 0; i < part.t.size(); ++i)
            for (int e = 0; e < 3; ++e) owner[edgeKey(part.t[i][static_cast<std::size_t>(e)], part.t[i][static_cast<std::size_t>((e + 1) % 3)])] = static_cast<int>(i);
        std::vector<char> seen(part.t.size(), 0);
        for (std::size_t i = 0; i < part.t.size(); ++i) {
            if (seen[i] || cls[i] != DraftClass::Undercut) continue;
            double area = 0, rim = 0;
            std::vector<std::size_t> stack{i};
            seen[i] = 1;
            while (!stack.empty()) {
                const std::size_t k = stack.back();
                stack.pop_back();
                double a = 0;
                normalOf(part, part.t[k], &a);
                area += a;
                for (int e = 0; e < 3; ++e) {
                    const int pa = part.t[k][static_cast<std::size_t>(e)], pb = part.t[k][static_cast<std::size_t>((e + 1) % 3)];
                    const auto it = owner.find(edgeKey(pb, pa));
                    const bool undercutThere = it != owner.end() && cls[static_cast<std::size_t>(it->second)] == DraftClass::Undercut;
                    if (!undercutThere) rim += dlength(part.p[static_cast<std::size_t>(pa)] - part.p[static_cast<std::size_t>(pb)]);
                    if (it == owner.end()) continue;
                    const std::size_t j = static_cast<std::size_t>(it->second);
                    if (!seen[j] && undercutThere) seen[j] = 1, stack.push_back(j);
                }
            }
            // A strip a hundredth wide (crumbs along a rounding's edge) isn't one either.
            const double width = rim > 0 ? 2 * area / rim : 0;
            if (area >= 1e-5 * part.size * part.size && width >= 2e-4 * part.size) out.undercut += area;
        }
    }
    if (out.undercut > 0) {
        char b[64];
        std::snprintf(b, sizeof b, "%.4g", out.undercut);
        return fail(why, std::string("The part has undercuts along this pull (") + b + " mm2 of faces that can't come away): MOLD DRAFT shows them in purple; turn the pull or change the part");
    }
    // One parting height: the halves made there, closing round the part.
    auto attempt = [&](double h, std::string* whyNot) -> bool {
    Partition pt;
    if (!partition(part, d, h, true, pt, whyNot)) return false;
    out.parting = describeLoops(pt, d, h);
    const Part& m = pt.part;
    // The pull's frame: u, v across it (u x v = d), heights along it.
    const DVec3 u = dnormalize(dcross(std::fabs(d.z) < 0.9 ? DVec3{0, 0, 1} : DVec3{1, 0, 0}, d));
    const DVec3 v = dcross(d, u);
    double lo[3] = {1e300, 1e300, 1e300}, hi[3] = {-1e300, -1e300, -1e300};
    for (const DVec3& q : m.p) {
        const double c[3] = {ddot(q, u), ddot(q, v), ddot(q, d)};
        for (int k = 0; k < 3; ++k) lo[k] = std::min(lo[k], c[k]), hi[k] = std::max(hi[k], c[k]);
    }
    const double extent = std::max({hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]});
    const double margin = o.margin > 0 ? o.margin : std::max(5.0, 0.25 * extent);
    for (int k = 0; k < 3; ++k) lo[k] -= margin, hi[k] += margin;
    if (!(h > lo[2] && h < hi[2])) return fail(whyNot, "The parting height is outside the block round the part");
    // Points: the part's, the parting line's on the plane, the block's corners.
    std::vector<DVec3> P = m.p;
    const double onTol = 1e-9 * part.size;
    std::unordered_map<int, int> proj;
    const double sameTol = 1e-5 * part.size; // projections closer than this are one point
    // The points on the plane, one for each place (a step straight along the pull, or a point
    // landing on one of the part's own there, is the same point).
    std::unordered_map<std::uint64_t, std::vector<int>> onPlane;
    auto cellKey = [&](const DVec3& q, int dx, int dy, int dz) {
        auto c = [&](double x, int off) { return static_cast<std::uint64_t>((static_cast<std::int64_t>(std::floor(x / sameTol)) + off) & 0x1FFFFF); };
        return (c(q.x, dx) << 42) | (c(q.y, dy) << 21) | c(q.z, dz);
    };
    auto placed = [&](const DVec3& q) -> int {
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
                for (int dz = -1; dz <= 1; ++dz)
                    if (const auto it = onPlane.find(cellKey(q, dx, dy, dz)); it != onPlane.end())
                        for (int j : it->second)
                            if (dlength(P[static_cast<std::size_t>(j)] - q) <= sameTol) return j;
        return -1;
    };
    auto place = [&](int id) { onPlane[cellKey(P[static_cast<std::size_t>(id)], 0, 0, 0)].push_back(id); };
    for (const auto& loop : pt.loops) // the part's own points on the plane first
        for (int a : loop)
            if (std::fabs(ddot(m.p[static_cast<std::size_t>(a)], d) - h) <= onTol && !proj.count(a)) {
                const int at = placed(m.p[static_cast<std::size_t>(a)]);
                proj[a] = at >= 0 ? at : a;
                if (at < 0) place(a);
            }
    for (const auto& loop : pt.loops)
        for (int a : loop) {
            if (proj.count(a)) continue;
            const DVec3 q = m.p[static_cast<std::size_t>(a)];
            const DVec3 on = q + d * (h - ddot(q, d));
            const int at = placed(on);
            if (at >= 0) {
                proj[a] = at;
            } else {
                proj[a] = static_cast<int>(P.size());
                P.push_back(on);
                place(proj[a]);
            }
        }
    auto corner = [&](double a, double b, double c) {
        P.push_back(u * a + v * b + d * c);
        return static_cast<int>(P.size()) - 1;
    };
    int ch[4], ct[4], cb[4]; // at h, top, bottom; counter-clockwise seen down the pull
    const double cu[4] = {lo[0], hi[0], hi[0], lo[0]}, cv[4] = {lo[1], lo[1], hi[1], hi[1]};
    for (int k = 0; k < 4; ++k) ch[k] = corner(cu[k], cv[k], h), ct[k] = corner(cu[k], cv[k], hi[2]), cb[k] = corner(cu[k], cv[k], lo[2]);
    // The plane: the block's outline less the part's outline (its holes back in).
    std::vector<P2> q2(P.size());
    for (std::size_t i = 0; i < P.size(); ++i) q2[i] = {ddot(P[i], u), ddot(P[i], v)};
    std::vector<std::vector<int>> loops2;
    for (const auto& loop : pt.loops) {
        std::vector<int> l;
        for (int a : loop)
            if (l.empty() || l.back() != proj[a]) l.push_back(proj[a]);
        while (l.size() > 1 && l.back() == l.front()) l.pop_back();
        if (l.size() < 3) return fail(whyNot, "The parting line has a loop with no width seen along the pull");
        loops2.push_back(std::move(l));
    }
    const double eps2 = 1e-12 * extent * extent;
    for (std::size_t i = 0; i < loops2.size(); ++i) // no loop folds over itself or another, seen down the pull
        for (std::size_t j = i; j < loops2.size(); ++j)
            for (std::size_t a = 0; a < loops2[i].size(); ++a)
                for (std::size_t b = (i == j ? a + 1 : 0); b < loops2[j].size(); ++b) {
                    const int a0 = loops2[i][a], a1 = loops2[i][(a + 1) % loops2[i].size()];
                    const int b0 = loops2[j][b], b1 = loops2[j][(b + 1) % loops2[j].size()];
                    if (a0 == b0 || a0 == b1 || a1 == b0 || a1 == b1) continue;
                    if (segmentsCross(q2[static_cast<std::size_t>(a0)], q2[static_cast<std::size_t>(a1)], q2[static_cast<std::size_t>(b0)], q2[static_cast<std::size_t>(b1)], eps2))
                        return fail(whyNot, "The parting line folds over itself seen along the pull: the part can't open along it (try another pull or parting height)");
                }
    // Nesting: the block holds the loops; a loop inside an odd number of others is an island.
    const std::size_t L = loops2.size();
    std::vector<int> depth(L, 1), parent(L, -1);
    for (std::size_t i = 0; i < L; ++i)
        for (std::size_t j = 0; j < L; ++j)
            if (i != j && insidePoly(q2, loops2[j], q2[static_cast<std::size_t>(loops2[i][0])])) ++depth[i];
    for (std::size_t i = 0; i < L; ++i)
        for (std::size_t j = 0; j < L; ++j)
            if (i != j && depth[j] == depth[i] - 1 && insidePoly(q2, loops2[j], q2[static_cast<std::size_t>(loops2[i][0])])) parent[i] = static_cast<int>(j);
    std::vector<std::array<int, 3>> plane; // counter-clockwise seen down the pull (facing +d)
    auto ccw = [&](std::vector<int> l, bool want) {
        if ((polyArea(q2, l) > 0) != want) std::reverse(l.begin(), l.end());
        return l;
    };
    {
        std::vector<std::vector<int>> holes;
        for (std::size_t i = 0; i < L; ++i)
            if (depth[i] == 1) holes.push_back(ccw(loops2[i], false));
        if (!triangulateRegion(q2, {ch[0], ch[1], ch[2], ch[3]}, holes, eps2, plane)) return fail(whyNot, "Can't lay out the parting plane round the part");
    }
    for (std::size_t i = 0; i < L; ++i) {
        if (depth[i] % 2 != 0) continue;
        std::vector<std::vector<int>> holes;
        for (std::size_t j = 0; j < L; ++j)
            if (parent[j] == static_cast<int>(i)) holes.push_back(ccw(loops2[j], false));
        if (!triangulateRegion(q2, ccw(loops2[i], true), holes, eps2, plane)) return fail(whyNot, "Can't lay out the parting plane inside the part's opening");
    }
    // The halves: the part's faces on each side (turned inside out: the mould's faces), the
    // shut-off walls from the parting line to the plane, the plane, the block.
    auto build = [&](bool cavity) {
        std::vector<std::array<int, 3>> t;
        for (std::size_t i = 0; i < m.t.size(); ++i)
            if ((pt.cavity[i] != 0) == cavity) t.push_back({m.t[i][0], m.t[i][2], m.t[i][1]});
        for (const auto& loop : pt.loops)
            for (std::size_t k = 0; k < loop.size(); ++k) {
                // The patch's edge a -> b round its border (the cavity's way; the core's the other).
                int a = loop[k], b = loop[(k + 1) % loop.size()];
                if (!cavity) std::swap(a, b);
                const int ap = proj[a], bp = proj[b];
                auto add = [&](int x, int y, int z) {
                    if (x != y && y != z && x != z) t.push_back({x, y, z});
                };
                add(a, b, bp);
                add(a, bp, ap);
            }
        for (const auto& f : plane) t.push_back(cavity ? std::array<int, 3>{f[0], f[2], f[1]} : f);
        for (int k = 0; k < 4; ++k) {
            const int k1 = (k + 1) % 4;
            if (cavity) t.push_back({ch[k], ch[k1], ct[k1]}), t.push_back({ch[k], ct[k1], ct[k]});
            else t.push_back({cb[k], cb[k1], ch[k1]}), t.push_back({cb[k], ch[k1], ch[k]});
        }
        if (cavity) t.push_back({ct[0], ct[1], ct[2]}), t.push_back({ct[0], ct[2], ct[3]});
        else t.push_back({cb[0], cb[2], cb[1]}), t.push_back({cb[0], cb[3], cb[2]});
        MeshData mesh;
        mesh.vertices.reserve(3 * t.size());
        mesh.indices.reserve(3 * t.size());
        for (const auto& f : t) {
            const Vec3 a = P[static_cast<std::size_t>(f[0])].f(), b = P[static_cast<std::size_t>(f[1])].f(), c = P[static_cast<std::size_t>(f[2])].f();
            // Two corners at the very same place: no face (its other two sides cancel out).
            if (a.x == b.x && a.y == b.y && a.z == b.z) continue;
            if (b.x == c.x && b.y == c.y && b.z == c.z) continue;
            if (a.x == c.x && a.y == c.y && a.z == c.z) continue;
            const Vec3 nn = normalize(cross(b - a, c - a));
            for (const Vec3& p : {a, b, c}) {
                Vertex vx;
                vx.position = p, vx.normal = nn;
                mesh.indices.push_back(static_cast<std::uint32_t>(mesh.vertices.size()));
                mesh.vertices.push_back(vx);
            }
        }
        return mesh;
    };
    out.cavity = build(true);
    out.core = build(false);
    out.block = (hi[0] - lo[0]) * (hi[1] - lo[1]) * (hi[2] - lo[2]);
    out.part = meshVolume(world);
    const double vc = meshVolume(out.cavity), vk = meshVolume(out.core);
    if (!moldSealed(out.cavity) || !moldSealed(out.core) || vc <= 0 || vk <= 0 ||
        std::fabs(vc + vk + out.part - out.block) > 1e-4 * out.block)
        return fail(whyNot, "The mould halves don't close round the part (its parting line may be too complex for one plane)");
    return true;
    };
    // The height given; else the part's own (where its faces turn from the pull), and if the
    // halves won't close there (a tangent line through rounded corners, say), its middle.
    std::vector<double> heights;
    if (o.at) {
        heights.push_back(*o.at);
    } else {
        Partition natural;
        if (!partition(part, d, -1e300, false, natural, why)) return false;
        heights.push_back(medianHeight(natural, d));
        double lo = 1e300, hi = -1e300;
        for (const DVec3& q : part.p) lo = std::min(lo, ddot(q, d)), hi = std::max(hi, ddot(q, d));
        for (const double t : {0.5, 0.5 * (heights.front() - lo) / std::max(hi - lo, 1e-12) + 0.25})
            if (std::fabs(lo + t * (hi - lo) - heights.front()) > 1e-6 * part.size) heights.push_back(lo + t * (hi - lo));
    }
    std::string first;
    for (std::size_t k = 0; k < heights.size(); ++k) {
        std::string w;
        if (attempt(heights[k], &w)) return true;
        if (k == 0) first = w;
    }
    return fail(why, first);
}

} // namespace sicdm

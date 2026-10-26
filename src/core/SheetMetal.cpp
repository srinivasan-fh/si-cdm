#include "core/SheetMetal.hpp"

#include "core/Extrude.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <tuple>

namespace sicdm {

namespace {

constexpr float kPiF = 3.14159265358979f;

// Where a flange sits in the flat: its edge from a to b (outward normal n, along e).
struct Layout {
    Vec2 a, b, e, n;
    float len = 0;     // the edge's length
    float ba = 0;      // bend allowance (flat width of the bend)
    float theta = 0;   // radians
    float r = 0;       // inside radius
    bool down = false;
    bool ok = false;
};

Vec2 sub(Vec2 p, Vec2 q) { return {p.x - q.x, p.y - q.y}; }
Vec2 add(Vec2 p, Vec2 q) { return {p.x + q.x, p.y + q.y}; }
Vec2 mul(Vec2 p, float s) { return {p.x * s, p.y * s}; }
float dot2(Vec2 p, Vec2 q) { return p.x * q.x + p.y * q.y; }

// The flat layout of every flange, parents first (a flange's parent comes before it).
std::vector<Layout> layout(const SheetPart& part, std::string* why) {
    std::vector<Layout> out(part.flanges.size());
    const std::size_t nb = part.base.size();
    for (std::size_t i = 0; i < part.flanges.size(); ++i) {
        const SheetFlange& f = part.flanges[i];
        Layout& L = out[i];
        Vec2 a, b, nrm;
        if (f.parent < 0) {
            if (f.edge < 0 || static_cast<std::size_t>(f.edge) >= nb) {
                if (why) *why = "A flange's edge isn't on the base";
                return {};
            }
            a = part.base[static_cast<std::size_t>(f.edge)], b = part.base[(static_cast<std::size_t>(f.edge) + 1) % nb];
            const Vec2 d = sub(b, a);
            const float l = std::hypot(d.x, d.y);
            if (!(l > 1e-6f)) {
                if (why) *why = "A flange's edge has no length";
                return {};
            }
            nrm = {d.y / l, -d.x / l}; // right of a counter-clockwise outline: outward
        } else {
            if (f.parent >= static_cast<int>(i) || !out[static_cast<std::size_t>(f.parent)].ok) {
                if (why) *why = "A flange is on a flange that comes after it";
                return {};
            }
            const Layout& p = out[static_cast<std::size_t>(f.parent)];
            const float reach = p.ba + part.flanges[static_cast<std::size_t>(f.parent)].length;
            // The parent's far edge, run the same way as its bend line (the outline's way round).
            a = add(p.a, mul(p.n, reach)), b = add(p.b, mul(p.n, reach));
            nrm = p.n;
        }
        L.a = a, L.b = b;
        L.len = std::hypot(b.x - a.x, b.y - a.y);
        L.e = {(b.x - a.x) / L.len, (b.y - a.y) / L.len};
        L.n = nrm;
        L.r = f.radius >= 0 ? f.radius : part.radius;
        L.theta = std::clamp(f.angle, 0.0f, 180.0f) * kPiF / 180.0f;
        L.ba = L.theta * (L.r + part.k * part.thickness);
        L.down = f.down;
        L.ok = true;
    }
    return out;
}

// Flange i's fold, as a map of a point given in its parent's flat frame (x, y in the plane,
// z through the sheet, 0 to t) to that frame folded.
Vec3 foldLocal(const SheetPart& part, const Layout& L, Vec3 p) {
    const float t = part.thickness;
    const Vec2 q{p.x, p.y};
    const float s = dot2(sub(q, L.a), L.n), w = dot2(sub(q, L.a), L.e);
    const float z = L.down ? t - p.z : p.z; // bending down: the same, mirrored through the sheet
    const float rho = t + L.r - z;          // distance from the bend's centre
    float u, h;                             // along n, along z
    if (s <= 0 || L.theta <= 0) {
        u = s, h = z;
    } else if (s < L.ba) {
        const float phi = L.theta * s / L.ba;
        u = rho * std::sin(phi), h = t + L.r - rho * std::cos(phi);
    } else {
        const float extra = s - L.ba;
        u = rho * std::sin(L.theta) + extra * std::cos(L.theta);
        h = t + L.r - rho * std::cos(L.theta) + extra * std::sin(L.theta);
    }
    if (L.down) h = t - h;
    const Vec2 at = add(add(L.a, mul(L.e, w)), mul(L.n, u));
    return {at.x, at.y, h};
}

// A point of flange i's region in the flat, folded all the way up to the base's frame.
Vec3 foldAll(const SheetPart& part, const std::vector<Layout>& lay, int i, Vec3 p) {
    while (i >= 0) {
        p = foldLocal(part, lay[static_cast<std::size_t>(i)], p);
        i = part.flanges[static_cast<std::size_t>(i)].parent;
    }
    return p;
}

} // namespace

float bendAllowance(float angleDeg, float radius, float k, float thickness) { return angleDeg * kPiF / 180.0f * (radius + k * thickness); }

std::vector<float> sheetParams(const SheetPart& part) {
    std::vector<float> p = {part.thickness, part.radius, part.k};
    for (const Vec3& v : {part.frame.origin, part.frame.u, part.frame.v}) p.insert(p.end(), {v.x, v.y, v.z});
    p.push_back(static_cast<float>(part.base.size()));
    for (const Vec2& q : part.base) p.insert(p.end(), {q.x, q.y});
    p.push_back(static_cast<float>(part.flanges.size()));
    for (const SheetFlange& f : part.flanges)
        p.insert(p.end(), {static_cast<float>(f.parent), static_cast<float>(f.edge), f.length, f.angle, f.down ? 1.0f : 0.0f, f.radius});
    return p;
}

bool sheetFromParams(const std::vector<float>& p, SheetPart& out) {
    if (p.size() < 13) return false;
    SheetPart s;
    s.thickness = p[0], s.radius = p[1], s.k = p[2];
    const Vec3 o{p[3], p[4], p[5]}, u{p[6], p[7], p[8]}, v{p[9], p[10], p[11]};
    if (!makePlaneFrame(o, cross(u, v), u, s.frame)) return false;
    std::size_t i = 12;
    const auto nb = static_cast<std::size_t>(p[i++]);
    if (nb < 3 || nb > 100000 || p.size() < i + 2 * nb + 1) return false;
    for (std::size_t k = 0; k < nb; ++k, i += 2) s.base.push_back({p[i], p[i + 1]});
    const auto nf = static_cast<std::size_t>(p[i++]);
    if (nf > 1000 || p.size() < i + 6 * nf) return false;
    for (std::size_t k = 0; k < nf; ++k, i += 6)
        s.flanges.push_back({static_cast<int>(p[i]), static_cast<int>(p[i + 1]), p[i + 2], p[i + 3], p[i + 4] != 0.0f, p[i + 5]});
    out = std::move(s);
    return true;
}

MeshData buildSheet(const SheetPart& part, bool flat, std::string* why) {
    auto fail = [&](const char* w) {
        if (why) *why = w;
        return MeshData{};
    };
    const float t = part.thickness;
    if (!(t > 0.0f) || !(part.radius >= 0.0f) || !(part.k >= 0.0f && part.k <= 1.0f)) return fail("The thickness, bend radius or K factor is out of range");
    if (part.base.size() < 3 || !isSimplePolygon(part.base) || signedArea(part.base) <= 0) return fail("The base outline must be a simple loop");
    const std::vector<Layout> lay = layout(part, why);
    if (lay.size() != part.flanges.size()) return MeshData{};
    // Each edge takes one flange.
    std::map<std::pair<int, int>, int> used;
    for (std::size_t i = 0; i < part.flanges.size(); ++i) {
        const SheetFlange& f = part.flanges[i];
        if (!(f.length >= 0) || !(f.angle >= 0 && f.angle <= 180)) return fail("A flange's length or angle is out of range");
        if (++used[{f.parent, f.parent < 0 ? f.edge : 0}] > 1) return fail("Two flanges on one edge");
    }
    // The flat pieces, counter-clockwise, each with the flange (-1: base) whose fold it takes.
    struct Piece {
        std::vector<Vec2> pts;
        int region;
    };
    std::vector<Piece> pieces;
    pieces.push_back({part.base, -1});
    for (std::size_t i = 0; i < lay.size(); ++i) {
        const Layout& L = lay[i];
        const int strips = L.theta > 0 ? std::max(2, static_cast<int>(std::ceil(L.theta / (7.5f * kPiF / 180.0f)))) : 0;
        float s0 = 0;
        // The bend zone in strips (smooth arcs), then the straight flat.
        for (int k = 0; k < strips; ++k) {
            const float s1 = L.ba * static_cast<float>(k + 1) / static_cast<float>(strips);
            pieces.push_back({{add(L.a, mul(L.n, s0)), add(L.a, mul(L.n, s1)), add(L.b, mul(L.n, s1)), add(L.b, mul(L.n, s0))}, static_cast<int>(i)});
            s0 = s1;
        }
        const float s1 = L.ba + part.flanges[i].length;
        if (s1 > s0 + 1e-6f) pieces.push_back({{add(L.a, mul(L.n, s0)), add(L.a, mul(L.n, s1)), add(L.b, mul(L.n, s1)), add(L.b, mul(L.n, s0))}, static_cast<int>(i)});
    }
    // Those quads are counter-clockwise when the edge runs b -> a... make sure.
    for (Piece& pc : pieces)
        if (signedArea(pc.pts) < 0) std::reverse(pc.pts.begin(), pc.pts.end());
    // Which flat edges lie on the outline: used by one piece only.
    auto key = [](Vec2 q) { return std::make_pair(std::llround(q.x * 1e4), std::llround(q.y * 1e4)); };
    std::map<std::pair<std::pair<long long, long long>, std::pair<long long, long long>>, int> edgeCount;
    for (const Piece& pc : pieces)
        for (std::size_t k = 0; k < pc.pts.size(); ++k) {
            auto ka = key(pc.pts[k]), kb = key(pc.pts[(k + 1) % pc.pts.size()]);
            if (kb < ka) std::swap(ka, kb);
            ++edgeCount[{ka, kb}];
        }
    MeshData m;
    const PlaneFrame& f = part.frame;
    auto world = [&](const Piece& pc, Vec2 q, float z) {
        const Vec3 p = flat ? Vec3{q.x, q.y, z} : foldAll(part, lay, pc.region, {q.x, q.y, z});
        return f.toWorld({p.x, p.y}) + f.normal * p.z;
    };
    auto tri = [&](Vec3 a, Vec3 b, Vec3 c) {
        const Vec3 n = normalize(cross(b - a, c - a));
        const auto base = static_cast<std::uint32_t>(m.vertices.size());
        m.vertices.push_back({a, n}), m.vertices.push_back({b, n}), m.vertices.push_back({c, n});
        m.indices.insert(m.indices.end(), {base, base + 1, base + 2});
    };
    for (const Piece& pc : pieces) {
        const std::vector<std::uint32_t> ids = pc.pts.size() == 4 ? std::vector<std::uint32_t>{0, 1, 2, 0, 2, 3} : triangulatePolygon(pc.pts);
        if (ids.empty()) return fail("The base outline could not be filled");
        for (std::size_t k = 0; k + 2 < ids.size(); k += 3) {
            const Vec2 a = pc.pts[ids[k]], b = pc.pts[ids[k + 1]], c = pc.pts[ids[k + 2]];
            tri(world(pc, a, t), world(pc, b, t), world(pc, c, t)); // top
            tri(world(pc, a, 0), world(pc, c, 0), world(pc, b, 0)); // bottom
        }
        for (std::size_t k = 0; k < pc.pts.size(); ++k) { // walls along the outline
            const Vec2 a = pc.pts[k], b = pc.pts[(k + 1) % pc.pts.size()];
            auto ka = key(a), kb = key(b);
            if (kb < ka) std::swap(ka, kb);
            if (edgeCount[{ka, kb}] != 1) continue;
            const Vec3 a0 = world(pc, a, 0), b0 = world(pc, b, 0), a1 = world(pc, a, t), b1 = world(pc, b, t);
            tri(a0, b0, b1);
            tri(a0, b1, a1);
        }
    }
    m.computeFeatureEdges(20.0f);
    return m;
}

MeshData buildSheetParams(const std::vector<float>& params) {
    SheetPart s;
    if (!sheetFromParams(params, s)) return {};
    return buildSheet(s, false);
}

std::vector<SheetEdge> sheetFreeEdges(const SheetPart& part) {
    std::vector<SheetEdge> out;
    const std::vector<Layout> lay = layout(part, nullptr);
    if (lay.size() != part.flanges.size()) return out;
    auto taken = [&](int parent, int edge) {
        return std::any_of(part.flanges.begin(), part.flanges.end(), [&](const SheetFlange& f) { return f.parent == parent && (parent >= 0 || f.edge == edge); });
    };
    const PlaneFrame& f = part.frame;
    const float t = part.thickness;
    auto lift = [&](int region, Vec2 q) {
        const Vec3 p = foldAll(part, lay, region, {q.x, q.y, t * 0.5f});
        return f.toWorld({p.x, p.y}) + f.normal * p.z;
    };
    for (std::size_t k = 0; k < part.base.size(); ++k) {
        if (taken(-1, static_cast<int>(k))) continue;
        SheetEdge e{-1, static_cast<int>(k), part.base[k], part.base[(k + 1) % part.base.size()], {}, {}, 0};
        e.a3 = lift(-1, e.a), e.b3 = lift(-1, e.b), e.length = std::hypot(e.b.x - e.a.x, e.b.y - e.a.y);
        if (e.length > 1e-6f) out.push_back(e);
    }
    for (std::size_t i = 0; i < lay.size(); ++i) {
        if (taken(static_cast<int>(i), 0)) continue;
        const Layout& L = lay[i];
        const float reach = L.ba + part.flanges[i].length;
        SheetEdge e{static_cast<int>(i), 0, add(L.a, mul(L.n, reach)), add(L.b, mul(L.n, reach)), {}, {}, L.len};
        e.a3 = lift(static_cast<int>(i), e.a), e.b3 = lift(static_cast<int>(i), e.b);
        out.push_back(e);
    }
    return out;
}

void sheetFlatOutline(const SheetPart& part, std::vector<std::vector<Vec2>>& outline, std::vector<std::pair<Vec2, Vec2>>& bendLines) {
    outline.clear(), bendLines.clear();
    const std::vector<Layout> lay = layout(part, nullptr);
    if (lay.size() != part.flanges.size()) return;
    // The base and each flange's whole strip, counter-clockwise; edges used once are outline.
    std::vector<std::vector<Vec2>> pieces{part.base};
    for (std::size_t i = 0; i < lay.size(); ++i) {
        const Layout& L = lay[i];
        const float reach = L.ba + part.flanges[i].length;
        std::vector<Vec2> q{L.a, add(L.a, mul(L.n, reach)), add(L.b, mul(L.n, reach)), L.b};
        if (signedArea(q) < 0) std::reverse(q.begin(), q.end());
        pieces.push_back(std::move(q));
        if (L.theta > 0) bendLines.push_back({add(L.a, mul(L.n, L.ba * 0.5f)), add(L.b, mul(L.n, L.ba * 0.5f))});
    }
    using Key = std::pair<long long, long long>;
    auto key = [](Vec2 q) { return Key{std::llround(q.x * 1e4), std::llround(q.y * 1e4)}; };
    std::map<std::pair<Key, Key>, int> count;
    for (const auto& pc : pieces)
        for (std::size_t k = 0; k < pc.size(); ++k) {
            auto a = key(pc[k]), b = key(pc[(k + 1) % pc.size()]);
            if (b < a) std::swap(a, b);
            ++count[{a, b}];
        }
    std::multimap<Key, std::pair<Vec2, Vec2>> from; // outline edges by their start
    for (const auto& pc : pieces)
        for (std::size_t k = 0; k < pc.size(); ++k) {
            const Vec2 p = pc[k], q = pc[(k + 1) % pc.size()];
            auto a = key(p), b = key(q);
            if (b < a) std::swap(a, b);
            if (count[{a, b}] == 1) from.insert({key(p), {p, q}});
        }
    while (!from.empty()) { // follow edges end to start round each loop
        auto it = from.begin();
        std::vector<Vec2> loop{it->second.first};
        Vec2 at = it->second.second;
        from.erase(it);
        for (std::size_t guard = 0; guard < 100000; ++guard) {
            auto nx = from.find(key(at));
            if (nx == from.end()) break;
            loop.push_back(at);
            at = nx->second.second;
            from.erase(nx);
        }
        if (loop.size() >= 3) outline.push_back(std::move(loop));
    }
}

} // namespace sicdm

#include "core/Airfoil.hpp"

#include "core/Armature.hpp"
#include "core/Brep.hpp"
#include "core/Extrude.hpp"
#include "core/Surfacing.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace sicdm {

namespace {

constexpr float kPiF = 3.14159265358979f;
constexpr int kSide = 40; // points along each side of a section

bool failWhy(std::string* why, const std::string& w) {
    if (why) *why = w;
    return false;
}

// Stations up the span: enough for the twist and taper, and a dozen a tubercle.
int stationsOf(const std::vector<float>& p) {
    int n = 24;
    if (p[6] > 0 && p[7] > 0) n = std::max(n, static_cast<int>(std::ceil(p[1] / p[7] * 12.0f)));
    return std::min(n, 240);
}

bool validBlade(const std::vector<float>& p) {
    if (p.size() < 8 || !(p[1] > 0) || !(p[2] > 0) || !(p[3] > 0) || p[6] < 0 || (p[6] > 0 && !(p[7] > 0))) return false;
    // Tubercles no deeper than a third of the shortest chord.
    if (p[6] * 3.0f > std::min(p[2], p[3])) return false;
    std::vector<Vec2> s;
    return nacaSection(bladeCode(p), 8, s);
}

} // namespace

bool nacaSection(const std::string& code, int n, std::vector<Vec2>& out, std::string* why) {
    out.clear();
    if (code.size() != 4 || !std::all_of(code.begin(), code.end(), [](char c) { return c >= '0' && c <= '9'; }))
        return failWhy(why, "a NACA four-digit code, e.g. 0012 or 2412");
    const float m = static_cast<float>(code[0] - '0') / 100.0f, p = static_cast<float>(code[1] - '0') / 10.0f,
                t = static_cast<float>((code[2] - '0') * 10 + (code[3] - '0')) / 100.0f;
    if (!(t > 0)) return failWhy(why, "the last two digits are the thickness (%): more than 00");
    if (m > 0 && !(p > 0)) return failWhy(why, "a cambered section needs the camber's place (the second digit)");
    n = std::max(n, 4);
    std::vector<Vec2> upper, lower;
    for (int i = 0; i < n; ++i) {
        const float b = kPiF * static_cast<float>(i) / static_cast<float>(n - 1), x = 0.5f * (1.0f - std::cos(b)); // 0 .. 1, close at the ends
        const float yt = 5.0f * t * (0.2969f * std::sqrt(x) - 0.1260f * x - 0.3516f * x * x + 0.2843f * x * x * x - 0.1036f * x * x * x * x); // closed edge
        float yc = 0, dyc = 0;
        if (m > 0) {
            if (x < p) yc = m / (p * p) * (2 * p * x - x * x), dyc = 2 * m / (p * p) * (p - x);
            else yc = m / ((1 - p) * (1 - p)) * ((1 - 2 * p) + 2 * p * x - x * x), dyc = 2 * m / ((1 - p) * (1 - p)) * (p - x);
        }
        const float th = std::atan(dyc);
        upper.push_back({x - yt * std::sin(th), yc + yt * std::cos(th)});
        lower.push_back({x + yt * std::sin(th), yc - yt * std::cos(th)});
    }
    // Trailing edge, over the top to the leading edge, under it back (the edge once).
    for (int i = n - 1; i >= 0; --i) out.push_back(upper[static_cast<std::size_t>(i)]);
    for (int i = 1; i < n - 1; ++i) out.push_back(lower[static_cast<std::size_t>(i)]);
    out.front() = {1, 0};
    return true;
}

std::string bladeCode(const std::vector<float>& p) {
    if (p.empty()) return {};
    char b[16];
    std::snprintf(b, sizeof b, "%04d", std::clamp(static_cast<int>(std::lround(p[0])), 0, 9999));
    return b;
}

std::vector<Vec3> bladeSection(const std::vector<float>& p, float z, int n) {
    std::vector<Vec2> unit;
    std::vector<Vec3> out;
    if (p.size() < 8 || !nacaSection(bladeCode(p), n, unit)) return out;
    const float s = std::clamp(z / p[1], 0.0f, 1.0f);
    const float chord = p[2] + (p[3] - p[2]) * s;
    // Tubercles move the leading edge forward and back; the trailing edge stays.
    const float bump = p[6] > 0 && p[7] > 0 ? p[6] * std::sin(2.0f * kPiF * z / p[7]) : 0.0f;
    const float le = p[5] * s - bump, c = chord + bump;
    const float twist = -p[4] * s * kPiF / 180.0f; // nose down for a positive twist (washout)
    const float qx = le + 0.25f * c, ct = std::cos(twist), st = std::sin(twist);
    for (const Vec2& u : unit) {
        const float x = le + u.x * c - qx, y = u.y * c;
        out.push_back({qx + x * ct - y * st, x * st + y * ct, z});
    }
    return out;
}

MeshData buildBlade(const std::vector<float>& p) {
    MeshData m;
    if (!validBlade(p)) return m;
    const int rows = stationsOf(p);
    std::vector<std::vector<Vec3>> secs;
    for (int k = 0; k <= rows; ++k) secs.push_back(bladeSection(p, p[1] * static_cast<float>(k) / static_cast<float>(rows), kSide));
    const auto ring = static_cast<std::uint32_t>(secs[0].size());
    for (const auto& s : secs)
        for (const Vec3& q : s) m.vertices.push_back({q, {}});
    for (std::uint32_t k = 0; k < static_cast<std::uint32_t>(rows); ++k)
        for (std::uint32_t j = 0; j < ring; ++j) {
            const std::uint32_t a = k * ring + j, a1 = k * ring + (j + 1) % ring, b = a + ring, b1 = a1 + ring;
            m.indices.insert(m.indices.end(), {a, a1, b1, a, b1, b});
        }
    smoothVertexNormals(m);
    // Flat ends, with their own vertices (a crisp edge).
    for (int end = 0; end < 2; ++end) {
        const std::vector<Vec3>& s = end == 0 ? secs.front() : secs.back();
        std::vector<Vec2> poly;
        for (const Vec3& q : s) poly.push_back({q.x, q.y});
        const std::vector<std::uint32_t> tri = triangulatePolygon(poly);
        const auto base = static_cast<std::uint32_t>(m.vertices.size());
        for (const Vec3& q : s) m.vertices.push_back({q, {0, 0, end == 0 ? -1.0f : 1.0f}});
        for (std::size_t t = 0; t + 2 < tri.size(); t += 3) {
            if (end == 0) m.indices.insert(m.indices.end(), {base + tri[t], base + tri[t + 2], base + tri[t + 1]}); // the root faces down
            else m.indices.insert(m.indices.end(), {base + tri[t], base + tri[t + 1], base + tri[t + 2]});
        }
    }
    m.computeFeatureEdges(30.0f);
    return m;
}

std::shared_ptr<Brep> bladeBrep(const std::vector<float>& p) {
    if (!validBlade(p)) return nullptr;
    const int rows = stationsOf(p);
    std::vector<std::vector<DVec3>> grid;
    for (int k = 0; k <= rows; ++k) {
        std::vector<DVec3> r;
        for (const Vec3& q : bladeSection(p, p[1] * static_cast<float>(k) / static_cast<float>(rows), kSide)) r.push_back(DVec3(q));
        r.push_back(r.front()); // round to the trailing edge again: the seam
        grid.push_back(r);
    }
    BSplineSurface skin;
    if (!interpolateSurface(grid, skin)) return nullptr;
    auto b = surfaceBrep(skin);
    if (b->edges.size() != 3) return nullptr; // the trailing edge (the seam), the root and the tip
    BShell& sh = b->shells[0];
    sh.faces[0].sameSense = false; // (span x around) points into the blade
    for (int end = 0; end < 2; ++end) {
        BSurface pl;
        pl.kind = BSurfaceKind::Plane;
        pl.f = Frame::make({0, 0, end == 0 ? 0.0 : static_cast<double>(p[1])}, {0, 0, end == 0 ? -1.0 : 1.0}, {1, 0, 0});
        b->surfaces.push_back(pl);
        BFace f;
        f.surface = static_cast<int>(b->surfaces.size()) - 1;
        BLoop l;
        l.outer = true;
        l.edges.push_back({end == 0 ? 2 : 1, end == 0});
        f.loops.push_back(l);
        sh.faces.push_back(f);
    }
    sh.closed = true;
    sh.name = "blade";
    return b;
}

} // namespace sicdm

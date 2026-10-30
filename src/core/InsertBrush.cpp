// Insert-mesh and curve brushes: shapes and tubes laid on a surface, joined to the body.

#include "core/InsertBrush.hpp"

#include "core/Features.hpp"
#include "core/Scene.hpp"
#include "core/Sculpt.hpp"
#include "core/Armature.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <map>

namespace sicdm {

namespace {

// The frame a placed shape stands in: z up the normal, x along `along`.
Mat4 frameAt(const InsertPlace& p, float spinDeg) {
    const Vec3 z = normalize(p.normal);
    Vec3 x = p.along - z * dot(p.along, z);
    if (length(x) < 1e-6f) x = cross(z, std::fabs(z.z) < 0.9f ? Vec3{0, 0, 1} : Vec3{1, 0, 0});
    x = normalize(x);
    const Vec3 y = cross(z, x);
    Mat4 m = Mat4::identity();
    m.m[0][0] = x.x, m.m[0][1] = x.y, m.m[0][2] = x.z;
    m.m[1][0] = y.x, m.m[1][1] = y.y, m.m[1][2] = y.z;
    m.m[2][0] = z.x, m.m[2][1] = z.y, m.m[2][2] = z.z;
    m.m[3][0] = p.at.x, m.m[3][1] = p.at.y, m.m[3][2] = p.at.z;
    return m * rotateAxis({0, 0, 1}, radians(spinDeg));
}

std::string lowerOf(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

} // namespace

const char* insertShapeName(InsertShape s) {
    switch (s) {
    case InsertShape::Cube: return "cube";
    case InsertShape::Cylinder: return "cylinder";
    case InsertShape::Cone: return "cone";
    case InsertShape::Torus: return "torus";
    case InsertShape::Mesh: return "mesh";
    default: return "sphere";
    }
}

std::optional<InsertShape> insertShapeFromName(const std::string& word) {
    const std::string w = lowerOf(word);
    if (w == "sphere" || w == "ball") return InsertShape::Sphere;
    if (w == "cube" || w == "box") return InsertShape::Cube;
    if (w == "cylinder" || w == "peg") return InsertShape::Cylinder;
    if (w == "cone" || w == "spike") return InsertShape::Cone;
    if (w == "torus" || w == "ring") return InsertShape::Torus;
    return std::nullopt;
}

MeshData insertShapeMesh(InsertShape shape, float size, const InsertPlace& place, float spin, float sink, const MeshData* custom) {
    using namespace primitives;
    const float s = std::max(size, 1e-3f), h = s;
    MeshData m;
    float tall = h; // its height along the normal
    switch (shape) {
    case InsertShape::Cube: m = box({-s / 2, -s / 2, 0}, {s / 2, s / 2, s}); break;
    case InsertShape::Cylinder: m = cylinder({0, 0, 0}, s / 2, s, 40); break;
    case InsertShape::Cone: m = cone({0, 0, 0}, s / 2, 0, s, 40); break;
    case InsertShape::Torus:
        m = torus({0, 0, s * 0.15f}, s * 0.35f, s * 0.15f, 40, 16);
        tall = s * 0.3f;
        break;
    case InsertShape::Mesh:
        if (custom && !custom->indices.empty()) {
            // Fitted: centred over the spot, standing on it, its largest side `size`.
            m = *custom;
            const AABB b = m.bounds();
            const Vec3 ext = b.max - b.min;
            const float k = s / std::max({ext.x, ext.y, ext.z, 1e-6f});
            m.transform(scale(Vec3{k, k, k}) * translate(Vec3{-(b.min.x + b.max.x) * 0.5f, -(b.min.y + b.max.y) * 0.5f, -b.min.z}));
            tall = ext.z * k;
            break;
        }
        [[fallthrough]];
    default: m = sphere({0, 0, s / 2}, s / 2, 32, 16); break;
    }
    m.transform(frameAt(place, spin) * translate(Vec3{0, 0, -std::clamp(sink, 0.0f, 1.0f) * tall}));
    return m;
}

std::vector<InsertPlace> evenStroke(const std::vector<Vec3>& points, const std::vector<Vec3>& normals, float step) {
    std::vector<InsertPlace> out;
    if (points.empty() || !(step > 0)) return out;
    // Arc length along the stroke.
    std::vector<float> s(points.size(), 0);
    for (std::size_t i = 1; i < points.size(); ++i) s[i] = s[i - 1] + length(points[i] - points[i - 1]);
    const float total = s.back();
    const int n = std::max(2, static_cast<int>(std::ceil(total / step)) + 1);
    auto sample = [&](float t, Vec3& p, Vec3& nn) {
        std::size_t k = static_cast<std::size_t>(std::upper_bound(s.begin(), s.end(), t) - s.begin());
        k = std::clamp<std::size_t>(k, 1, points.size() - 1);
        const float span = s[k] - s[k - 1], f = span > 0 ? (t - s[k - 1]) / span : 0;
        p = points[k - 1] + (points[k] - points[k - 1]) * f;
        const Vec3 na = k - 1 < normals.size() ? normals[k - 1] : Vec3{0, 0, 1}, nb = k < normals.size() ? normals[k] : na;
        nn = normalize(na + (nb - na) * f);
    };
    std::vector<Vec3> ps(static_cast<std::size_t>(n)), ns(static_cast<std::size_t>(n));
    if (points.size() == 1 || total <= 0) {
        out.push_back({points[0], normals.empty() ? Vec3{0, 0, 1} : normalize(normals[0]), {1, 0, 0}});
        return out;
    }
    for (int i = 0; i < n; ++i) sample(total * static_cast<float>(i) / static_cast<float>(n - 1), ps[static_cast<std::size_t>(i)], ns[static_cast<std::size_t>(i)]);
    // A hand's stroke wobbles: a few passes of smoothing (the ends stay).
    for (int pass = 0; pass < 3; ++pass) {
        std::vector<Vec3> q = ps;
        for (std::size_t i = 1; i + 1 < ps.size(); ++i) q[i] = ps[i] * 0.5f + (ps[i - 1] + ps[i + 1]) * 0.25f;
        ps = q;
    }
    for (std::size_t i = 0; i < ps.size(); ++i) {
        const Vec3 d = i + 1 < ps.size() ? ps[i + 1] - ps[i] : ps[i] - ps[i - 1];
        out.push_back({ps[i], ns[i], length(d) > 0 ? normalize(d) : Vec3{1, 0, 0}});
    }
    return out;
}

const char* tubeProfileName(TubeProfile p) {
    switch (p) {
    case TubeProfile::Square: return "square";
    case TubeProfile::Flat: return "flat";
    case TubeProfile::Triangle: return "triangle";
    case TubeProfile::Star: return "star";
    default: return "round";
    }
}

std::optional<TubeProfile> tubeProfileFromName(const std::string& word) {
    std::string w;
    for (const char ch : word) w += static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    if (w == "round" || w == "circle") return TubeProfile::Round;
    if (w == "square" || w == "box") return TubeProfile::Square;
    if (w == "flat" || w == "ribbon") return TubeProfile::Flat;
    if (w == "triangle") return TubeProfile::Triangle;
    if (w == "star") return TubeProfile::Star;
    return std::nullopt;
}

namespace {

// The profile's edge at angle ph, as (across, up) for radius 1: up is out from the surface.
std::array<float, 2> profilePoint(TubeProfile p, float ph) {
    const float c = std::cos(ph), s = std::sin(ph);
    switch (p) {
    case TubeProfile::Square: {
        const float k = 1.0f / std::max(std::fabs(c), std::fabs(s));
        return {c * k, s * k};
    }
    case TubeProfile::Flat: return {c, 0.35f * s}; // a ribbon a third as thick as it is wide
    case TubeProfile::Triangle: {
        // An equilateral triangle, a point up: the nearer of its three sides' lines.
        float best = 1e30f;
        for (int k = 0; k < 3; ++k) {
            const float a = kPi * 0.5f + kPi + 2 * kPi * static_cast<float>(k) / 3.0f; // its sides' outward normals: down, then round
            const float d = c * std::cos(a) + s * std::sin(a);
            if (d > 1e-6f) best = std::min(best, 0.5f / d);
        }
        return {c * best, s * best};
    }
    case TubeProfile::Star: {
        // Five points: the radius between 1 and 0.5, straight between them.
        const float seg = 2 * kPi / 10;
        const float a = ph - kPi * 0.5f;
        const int k = static_cast<int>(std::floor(a / seg));
        const float t = (a - static_cast<float>(k) * seg) / seg;
        const float r0 = (k % 2 == 0) ? 1.0f : 0.5f, r1 = (k % 2 == 0) ? 0.5f : 1.0f;
        const float a0 = static_cast<float>(k) * seg + kPi * 0.5f, a1 = a0 + seg;
        const float x = (1 - t) * r0 * std::cos(a0) + t * r1 * std::cos(a1), y = (1 - t) * r0 * std::sin(a0) + t * r1 * std::sin(a1);
        return {x, y};
    }
    default: return {c, s};
    }
}

} // namespace

MeshData curveTube(const std::vector<InsertPlace>& stroke, float r, float sink, TubeProfile profile, float taperStart, float taperEnd) {
    MeshData m;
    if (stroke.size() < 2 || !(r > 0)) return m;
    const bool round = profile == TubeProfile::Round;
    const int seg = round ? 20 : profile == TubeProfile::Star ? 40 : 32;
    std::vector<Vec3> c(stroke.size()), t(stroke.size());
    // How far along each point is (0...1), for the taper.
    std::vector<float> along(stroke.size(), 0.0f);
    for (std::size_t i = 1; i < stroke.size(); ++i) along[i] = along[i - 1] + length(stroke[i].at - stroke[i - 1].at);
    const float total = along.back() > 0 ? along.back() : 1.0f;
    std::vector<float> rad(stroke.size());
    for (std::size_t i = 0; i < stroke.size(); ++i) {
        const float f = along[i] / total;
        rad[i] = r * std::max(0.02f, taperStart + (taperEnd - taperStart) * f);
        // The centre line above the surface: `sink` of the profile's height in, at its own size there.
        const float half = rad[i] * (profile == TubeProfile::Flat ? 0.35f : 1.0f);
        c[i] = stroke[i].at + normalize(stroke[i].normal) * (half - 2 * half * std::clamp(sink, 0.0f, 1.0f));
    }
    for (std::size_t i = 0; i < c.size(); ++i) {
        const Vec3 d = i + 1 < c.size() ? c[i + 1] - (i > 0 ? c[i - 1] : c[i]) : c[i] - c[i - 1];
        t[i] = length(d) > 0 ? normalize(d) : Vec3{1, 0, 0};
    }
    // Frames along the centre line: `up` out from the surface (so a flat or square profile lies
    // on it), `across` square to it and the line.
    std::vector<std::array<Vec3, 2>> frames(c.size());
    for (std::size_t i = 0; i < c.size(); ++i) {
        Vec3 up = stroke[i].normal - t[i] * dot(stroke[i].normal, t[i]);
        if (length(up) < 1e-5f) up = i > 0 ? frames[i - 1][1] : cross(t[i], std::fabs(t[i].z) < 0.9f ? Vec3{0, 0, 1} : Vec3{1, 0, 0});
        up = normalize(up);
        frames[i] = {normalize(cross(up, t[i])), up};
    }
    auto ring = [&](const Vec3& centre, const std::array<Vec3, 2>& f, float rr, const Vec3& axisDir, float axial) {
        for (int j = 0; j < seg; ++j) {
            const float ph = 2 * kPi * static_cast<float>(j) / static_cast<float>(seg);
            const auto q = profilePoint(profile, ph);
            const Vec3 radial = f[0] * q[0] + f[1] * q[1];
            m.vertices.push_back({centre + radial * rr + axisDir * axial, normalize(radial * rr + axisDir * axial)});
        }
    };
    auto band = [&](std::uint32_t r0, std::uint32_t r1) {
        for (int j = 0; j < seg; ++j) {
            const std::uint32_t a = r0 + static_cast<std::uint32_t>(j), b = r0 + static_cast<std::uint32_t>((j + 1) % seg),
                                d = r1 + static_cast<std::uint32_t>(j), e = r1 + static_cast<std::uint32_t>((j + 1) % seg);
            m.indices.insert(m.indices.end(), {a, b, d, b, e, d});
        }
    };
    // Each end: a round tube's a half ball, another profile's flat, to the end's own size.
    const int capRings = round ? 6 : 1;
    const Vec3 back = t.front() * -1.0f;
    const float r0 = rad.front(), r1 = rad.back();
    m.vertices.push_back({c.front() + back * (round ? r0 : 0.0f), back});
    const std::uint32_t pole0 = 0;
    std::uint32_t prev = 1;
    for (int k = 1; k <= capRings; ++k) {
        const float a = kPi * 0.5f * static_cast<float>(k) / static_cast<float>(capRings);
        if (round) ring(c.front(), frames.front(), r0 * std::sin(a), back, r0 * std::cos(a));
        else ring(c.front(), frames.front(), r0, back, 0);
        if (k == 1)
            for (int j = 0; j < seg; ++j)
                m.indices.insert(m.indices.end(), {pole0, prev + static_cast<std::uint32_t>((j + 1) % seg), prev + static_cast<std::uint32_t>(j)});
        else
            band(prev, prev + static_cast<std::uint32_t>(seg)), prev += static_cast<std::uint32_t>(seg);
    }
    for (std::size_t i = 1; i < c.size(); ++i) {
        ring(c[i], frames[i], rad[i], t[i], 0);
        band(prev, prev + static_cast<std::uint32_t>(seg));
        prev += static_cast<std::uint32_t>(seg);
    }
    const Vec3 fwd = t.back();
    for (int k = capRings - 1; k >= 1; --k) {
        const float a = kPi * 0.5f * static_cast<float>(k) / static_cast<float>(capRings);
        ring(c.back(), frames.back(), r1 * std::sin(a), fwd, r1 * std::cos(a));
        band(prev, prev + static_cast<std::uint32_t>(seg));
        prev += static_cast<std::uint32_t>(seg);
    }
    m.vertices.push_back({c.back() + fwd * (round ? r1 : 0.0f), fwd});
    const std::uint32_t pole1 = static_cast<std::uint32_t>(m.vertices.size() - 1);
    for (int j = 0; j < seg; ++j)
        m.indices.insert(m.indices.end(), {prev + static_cast<std::uint32_t>(j), prev + static_cast<std::uint32_t>((j + 1) % seg), pole1});
    return m;
}

MeshData curveCopies(const std::vector<InsertPlace>& stroke, InsertShape shape, float size, float spacing, float sink, const MeshData* custom) {
    MeshData m;
    if (stroke.empty() || !(spacing > 0)) return m;
    float next = 0, walked = 0;
    for (std::size_t i = 0; i < stroke.size(); ++i) {
        if (i > 0) walked += length(stroke[i].at - stroke[i - 1].at);
        if (walked + 1e-4f < next) continue;
        m.append(insertShapeMesh(shape, size, stroke[i], 0, sink, custom));
        next = walked + spacing;
    }
    return m;
}

bool joinInsert(Body& body, const MeshData& piece, InsertJoin join, std::string* why) {
    if (piece.indices.empty()) {
        if (why) *why = "Nothing to put in";
        return false;
    }
    if (join == InsertJoin::Union) {
        const Body tool = featureBody("insert", piece, body.color);
        return bodyBoolean(body, tool, BooleanOp::Union, why);
    }
    // Appended: fused exactly too (one closed solid straight away), and what of it shows a
    // polygroup of its own, to sculpt or mask apart; REMESH still blends the joins.
    {
        Body fused = body;
        std::vector<std::uint32_t> before = body.mesh.hasGroups() ? body.mesh.groups : std::vector<std::uint32_t>(body.mesh.indices.size() / 3, 0);
        const std::uint32_t g = before.empty() ? 1 : *std::max_element(before.begin(), before.end()) + 1;
        const Body tool = featureBody("insert", piece, body.color);
        std::string unionWhy;
        if (bodyBoolean(fused, tool, BooleanOp::Union, &unionWhy)) {
            // Each triangle of the result: the piece's where it lies on the piece's surface (a
            // grid over the piece's triangles finds the few near it), else the body's own group
            // of the body triangle nearest.
            MeshData local = piece;
            local.transform(translate(fused.offset * -1.0f));
            AABB box;
            for (const Vertex& v : local.vertices) box.expand(v.position);
            const Vec3 ext = box.max - box.min;
            const float size = std::max({ext.x, ext.y, ext.z, 1e-3f});
            const float cell = size / 24.0f, eps = std::max(1e-4f, size * 1e-4f);
            std::map<std::array<int, 3>, std::vector<std::uint32_t>> grid;
            auto key = [&](const Vec3& p) {
                return std::array<int, 3>{static_cast<int>(std::floor((p.x - box.min.x) / cell)), static_cast<int>(std::floor((p.y - box.min.y) / cell)),
                                          static_cast<int>(std::floor((p.z - box.min.z) / cell))};
            };
            for (std::uint32_t t3 = 0; t3 + 2 < local.indices.size(); t3 += 3) {
                const Vec3 a = local.vertices[local.indices[t3]].position, b = local.vertices[local.indices[t3 + 1]].position,
                           cc = local.vertices[local.indices[t3 + 2]].position;
                AABB tb;
                tb.expand(a), tb.expand(b), tb.expand(cc);
                const auto lo = key(tb.min - Vec3{eps, eps, eps}), hi = key(tb.max + Vec3{eps, eps, eps});
                for (int x = lo[0]; x <= hi[0]; ++x)
                    for (int y = lo[1]; y <= hi[1]; ++y)
                        for (int z = lo[2]; z <= hi[2]; ++z) grid[{x, y, z}].push_back(t3);
            }
            auto onPiece = [&](const Vec3& p) {
                const auto it = grid.find(key(p));
                if (it == grid.end()) return false;
                for (const std::uint32_t t3 : it->second) {
                    const Vec3 q = closestPointOnTriangle(p, local.vertices[local.indices[t3]].position, local.vertices[local.indices[t3 + 1]].position,
                                                          local.vertices[local.indices[t3 + 2]].position);
                    if (length(q - p) <= eps) return true;
                }
                return false;
            };
            std::vector<Vec3> mids;
            for (std::size_t t3 = 0; t3 + 2 < body.mesh.indices.size(); t3 += 3)
                mids.push_back((body.mesh.vertices[body.mesh.indices[t3]].position + body.mesh.vertices[body.mesh.indices[t3 + 1]].position +
                                body.mesh.vertices[body.mesh.indices[t3 + 2]].position) *
                               (1.0f / 3.0f));
            const NearestPoints nearMid(mids);
            MeshData& out = fused.mesh;
            std::vector<std::uint32_t> groups(out.indices.size() / 3, 0);
            for (std::size_t t3 = 0; t3 + 2 < out.indices.size(); t3 += 3) {
                const Vec3 mid = (out.vertices[out.indices[t3]].position + out.vertices[out.indices[t3 + 1]].position + out.vertices[out.indices[t3 + 2]].position) *
                                 (1.0f / 3.0f);
                if (onPiece(mid)) {
                    groups[t3 / 3] = g;
                } else if (const std::int64_t j = nearMid.find(mid, 1e30f); j >= 0 && static_cast<std::size_t>(j) < before.size()) {
                    groups[t3 / 3] = before[static_cast<std::size_t>(j)];
                }
            }
            out.groups = std::move(groups);
            if (fused.kind != "mesh") fused.kind = "mesh", fused.params.clear();
            body = std::move(fused);
            body.mesh.computeFeatureEdges(35.0f);
            return true;
        }
    }
    // The union failed (an open body, say): appended as it is, to fuse later with REMESH.
    MeshData local = piece;
    local.transform(translate(body.offset * -1.0f));
    if (body.kind != "mesh") { // its triangles are its own from now on
        body.kind = "mesh";
        body.params.clear();
    }
    const std::size_t had = body.mesh.indices.size() / 3;
    std::vector<std::uint32_t> groups = body.mesh.hasGroups() ? body.mesh.groups : std::vector<std::uint32_t>(had, 0);
    const std::uint32_t g = groups.empty() ? 1 : *std::max_element(groups.begin(), groups.end()) + 1;
    body.mesh.append(local);
    groups.resize(body.mesh.indices.size() / 3, g);
    body.mesh.groups = std::move(groups);
    body.mesh.computeFeatureEdges(35.0f);
    body.brep.reset();
    return true;
}

} // namespace sicdm

// Insert-mesh and curve brushes: shapes and tubes laid on a surface, joined to the body.

#include "core/InsertBrush.hpp"

#include "core/Features.hpp"
#include "core/Scene.hpp"
#include "core/Sculpt.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>

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

MeshData curveTube(const std::vector<InsertPlace>& stroke, float r, float sink) {
    MeshData m;
    if (stroke.size() < 2 || !(r > 0)) return m;
    const int seg = 20;
    const float lift = r - 2 * r * std::clamp(sink, 0.0f, 1.0f); // the centre line above the surface
    std::vector<Vec3> c(stroke.size()), t(stroke.size());
    for (std::size_t i = 0; i < stroke.size(); ++i) c[i] = stroke[i].at + normalize(stroke[i].normal) * lift;
    for (std::size_t i = 0; i < c.size(); ++i) {
        const Vec3 d = i + 1 < c.size() ? c[i + 1] - (i > 0 ? c[i - 1] : c[i]) : c[i] - c[i - 1];
        t[i] = length(d) > 0 ? normalize(d) : Vec3{1, 0, 0};
    }
    // Rings round the centre line, turned along it without twisting (each frame carried on).
    Vec3 u = cross(t[0], stroke[0].normal);
    if (length(u) < 1e-5f) u = cross(t[0], std::fabs(t[0].z) < 0.9f ? Vec3{0, 0, 1} : Vec3{1, 0, 0});
    u = normalize(u);
    std::vector<std::array<Vec3, 2>> frames(c.size());
    for (std::size_t i = 0; i < c.size(); ++i) {
        if (i > 0) {
            u = u - t[i] * dot(u, t[i]);
            if (length(u) < 1e-6f) u = cross(t[i], std::fabs(t[i].z) < 0.9f ? Vec3{0, 0, 1} : Vec3{1, 0, 0});
            u = normalize(u);
        }
        frames[i] = {u, cross(t[i], u)};
    }
    auto ring = [&](const Vec3& centre, const Vec3& a, const Vec3& b, float rr, const Vec3& axisDir, float axial) {
        for (int j = 0; j < seg; ++j) {
            const float ph = 2 * kPi * static_cast<float>(j) / static_cast<float>(seg);
            const Vec3 radial = a * std::cos(ph) + b * std::sin(ph);
            const Vec3 nrm = normalize(radial * rr + axisDir * axial);
            m.vertices.push_back({centre + radial * rr + axisDir * axial, nrm});
        }
    };
    auto band = [&](std::uint32_t r0, std::uint32_t r1) {
        for (int j = 0; j < seg; ++j) {
            const std::uint32_t a = r0 + static_cast<std::uint32_t>(j), b = r0 + static_cast<std::uint32_t>((j + 1) % seg),
                                d = r1 + static_cast<std::uint32_t>(j), e = r1 + static_cast<std::uint32_t>((j + 1) % seg);
            m.indices.insert(m.indices.end(), {a, b, d, b, e, d});
        }
    };
    // Start cap: a half ball, pole first.
    const int capRings = 6;
    const Vec3 back = t.front() * -1.0f;
    m.vertices.push_back({c.front() + back * r, back});
    const std::uint32_t pole0 = 0;
    std::uint32_t prev = 1;
    for (int k = 1; k <= capRings; ++k) {
        const float a = kPi * 0.5f * static_cast<float>(k) / static_cast<float>(capRings);
        ring(c.front(), frames.front()[0], frames.front()[1], r * std::sin(a), back, r * std::cos(a));
        if (k == 1)
            for (int j = 0; j < seg; ++j)
                m.indices.insert(m.indices.end(), {pole0, prev + static_cast<std::uint32_t>((j + 1) % seg), prev + static_cast<std::uint32_t>(j)});
        else
            band(prev, prev + static_cast<std::uint32_t>(seg)), prev += static_cast<std::uint32_t>(seg);
    }
    // Along the stroke.
    for (std::size_t i = 1; i < c.size(); ++i) {
        ring(c[i], frames[i][0], frames[i][1], r, t[i], 0);
        band(prev, prev + static_cast<std::uint32_t>(seg));
        prev += static_cast<std::uint32_t>(seg);
    }
    // End cap.
    const Vec3 fwd = t.back();
    for (int k = capRings - 1; k >= 1; --k) {
        const float a = kPi * 0.5f * static_cast<float>(k) / static_cast<float>(capRings);
        ring(c.back(), frames.back()[0], frames.back()[1], r * std::sin(a), fwd, r * std::cos(a));
        band(prev, prev + static_cast<std::uint32_t>(seg));
        prev += static_cast<std::uint32_t>(seg);
    }
    m.vertices.push_back({c.back() + fwd * r, fwd});
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
    // Appended as it is, in the body's frame, a polygroup of its own (REMESH fuses it later).
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

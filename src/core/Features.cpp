#include "core/Features.hpp"

#include "core/Print.hpp"
#include "core/Scene.hpp"
#include "core/Sculpt.hpp"
#include "core/Wearable.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <unordered_map>

namespace sicdm {

namespace {

constexpr double kTau = 6.283185307179586;

// Rodrigues: v turned by `angle` about the unit axis k.
Vec3 rotateAbout(const Vec3& v, const Vec3& k, float angle) {
    const float c = std::cos(angle), s = std::sin(angle);
    return v * c + cross(k, v) * s + k * (dot(k, v) * (1.0f - c));
}

// The rotation taking unit a onto unit b, applied to v.
Vec3 turnLike(const Vec3& v, const Vec3& a, const Vec3& b) {
    const Vec3 ax = cross(a, b);
    const float s = length(ax), c = dot(a, b);
    if (s < 1e-7f) {
        if (c > 0) return v;
        // Opposite: half a turn about any axis square to a.
        Vec3 p = std::fabs(a.x) < 0.9f ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
        p = normalize(p - a * dot(p, a));
        return rotateAbout(v, p, static_cast<float>(kTau / 2));
    }
    return rotateAbout(v, ax / s, std::atan2(s, c));
}

struct RawMesh {
    std::vector<Vec3> p;
    std::vector<std::uint32_t> t;
    void tri(std::uint32_t a, std::uint32_t b, std::uint32_t c) {
        if (a == b || b == c || a == c) return;
        t.insert(t.end(), {a, b, c});
    }
    MeshData data() const {
        MeshData m;
        for (const Vec3& q : p) m.vertices.push_back({q, {}});
        m.indices = t;
        return m;
    }
};

// Triangles of a planar loop (world points), ear-clipped in its own plane.
std::vector<std::uint32_t> capTriangles(const std::vector<Vec3>& loop) {
    Vec3 n{};
    for (std::size_t i = 0; i < loop.size(); ++i) n += cross(loop[i], loop[(i + 1) % loop.size()]); // Newell
    if (length(n) < 1e-12f) return {};
    n = normalize(n);
    Vec3 u = std::fabs(n.x) < 0.9f ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
    u = normalize(u - n * dot(u, n));
    const Vec3 v = cross(n, u);
    std::vector<Vec2> poly;
    for (const Vec3& q : loop) poly.push_back({dot(q, u), dot(q, v)});
    return triangulatePolygon(poly); // counter-clockwise about n, as Newell makes it
}

// Resamples a closed loop to n points evenly by length.
std::vector<Vec3> resampleLoop(const std::vector<Vec3>& l, std::size_t n) {
    std::vector<double> acc{0.0};
    for (std::size_t i = 0; i < l.size(); ++i) acc.push_back(acc.back() + length(l[(i + 1) % l.size()] - l[i]));
    const double total = acc.back();
    std::vector<Vec3> out;
    std::size_t k = 0;
    for (std::size_t i = 0; i < n; ++i) {
        const double s = total * static_cast<double>(i) / static_cast<double>(n);
        while (k + 1 < acc.size() - 1 && acc[k + 1] < s) ++k;
        const double seg = acc[k + 1] - acc[k];
        const float t = seg > 0 ? static_cast<float>((s - acc[k]) / seg) : 0.0f;
        out.push_back(l[k] + (l[(k + 1) % l.size()] - l[k]) * t);
    }
    return out;
}

Vec3 centroidOf(const std::vector<Vec3>& l) {
    Vec3 c{};
    for (const Vec3& q : l) c += q;
    return l.empty() ? c : c / static_cast<float>(l.size());
}

} // namespace

std::vector<Vec2> profilePolygon(const Profile& p, int circleSegments) {
    if (p.kind == Profile::Kind::Polygon) return p.points;
    std::vector<Vec2> out;
    for (int i = 0; i < circleSegments; ++i) {
        const float a = static_cast<float>(kTau * i / circleSegments);
        out.push_back({p.center.x + p.radius * std::cos(a), p.center.y + p.radius * std::sin(a)});
    }
    return out;
}

Body featureBody(std::string name, MeshData world, Vec3 color) {
    Body b = meshBody(std::move(name), std::move(world), color);
    repairMesh(b);                // weld, wind outward
    SculptMesh(b).writeTo(b, true, 40.0f); // normals smooth across gentle bends, crisp at edges
    return b;
}

std::optional<MeshData> revolveProfile(const Profile& profile, SketchPlane plane, Vec2 axisPoint, Vec2 axisDir, float degrees, std::string* error) {
    auto fail = [&](const char* why) -> std::optional<MeshData> {
        if (error) *error = why;
        return std::nullopt;
    };
    const float dl = std::hypot(axisDir.x, axisDir.y);
    if (dl < 1e-9f) return fail("The axis needs a direction");
    if (!(std::fabs(degrees) > 0.01f)) return fail("The angle must not be zero");
    const Vec2 d{axisDir.x / dl, axisDir.y / dl}, perp{-d.y, d.x};
    const std::vector<Vec2> pts = profilePolygon(profile);
    std::vector<float> along, radius;
    float lo = 1e30f, hi = -1e30f;
    for (const Vec2& q : pts) {
        const Vec2 r{q.x - axisPoint.x, q.y - axisPoint.y};
        along.push_back(r.x * d.x + r.y * d.y);
        radius.push_back(r.x * perp.x + r.y * perp.y);
        lo = std::min(lo, radius.back()), hi = std::max(hi, radius.back());
    }
    const float eps = 1e-4f * std::max(1.0f, hi - lo);
    if (lo < -eps && hi > eps) return fail("The profile crosses the axis: keep it on one side");
    if (hi - lo < eps) return fail("The profile lies on the axis");
    const PlaneFrame f = planeFrame(plane);
    const Vec3 A = f.toWorld(axisPoint), D = normalize(f.u * d.x + f.v * d.y);
    Vec3 P = normalize(f.u * perp.x + f.v * perp.y);
    if (hi <= eps) { // on the negative side: measure the radius the other way
        P = -P;
        for (float& r : radius) r = -r;
    }
    const Vec3 B = cross(D, P);
    const bool full = std::fabs(degrees) >= 359.999f;
    const float sweep = static_cast<float>(std::clamp(static_cast<double>(degrees), -360.0, 360.0) * kTau / 360.0);
    const int segments = std::max(3, static_cast<int>(std::ceil(std::fabs(degrees) / 360.0f * 96.0f)));
    const int rings = full ? segments : segments + 1;
    RawMesh m;
    const std::size_t n = pts.size();
    std::vector<std::uint32_t> onAxis(n, UINT32_MAX);
    std::vector<std::uint32_t> idx(static_cast<std::size_t>(rings) * n);
    for (int k = 0; k < rings; ++k) {
        const float a = sweep * static_cast<float>(k) / static_cast<float>(segments);
        const Vec3 dir = P * std::cos(a) + B * std::sin(a);
        for (std::size_t i = 0; i < n; ++i) {
            if (radius[i] < eps) { // on the axis: one point for every ring
                if (onAxis[i] == UINT32_MAX) {
                    onAxis[i] = static_cast<std::uint32_t>(m.p.size());
                    m.p.push_back(A + D * along[i]);
                }
                idx[static_cast<std::size_t>(k) * n + i] = onAxis[i];
                continue;
            }
            idx[static_cast<std::size_t>(k) * n + i] = static_cast<std::uint32_t>(m.p.size());
            m.p.push_back(A + D * along[i] + dir * radius[i]);
        }
    }
    auto at = [&](int k, std::size_t i) { return idx[static_cast<std::size_t>(k % rings) * n + i % n]; };
    for (int k = 0; k < segments; ++k)
        for (std::size_t i = 0; i < n; ++i) {
            const std::uint32_t a = at(k, i), b = at(k, i + 1), c = at(k + 1, i), e = at(k + 1, i + 1);
            m.tri(a, c, e);
            m.tri(a, e, b);
        }
    if (!full) { // caps at both ends
        const std::vector<std::uint32_t> t = triangulatePolygon(pts);
        if (t.empty()) return fail("The profile can't be capped (it crosses itself)");
        for (std::size_t q = 0; q + 2 < t.size(); q += 3) {
            m.tri(at(0, t[q]), at(0, t[q + 2]), at(0, t[q + 1]));
            m.tri(at(segments, t[q]), at(segments, t[q + 1]), at(segments, t[q + 2]));
        }
    }
    return m.data();
}

std::optional<MeshData> sweepProfile(const Profile& profile, SketchPlane profilePlane, const std::vector<Vec3>& pathIn, bool closedPath,
                                     std::string* error) {
    auto fail = [&](const char* why) -> std::optional<MeshData> {
        if (error) *error = why;
        return std::nullopt;
    };
    std::vector<Vec3> path;
    for (const Vec3& q : pathIn)
        if (path.empty() || length(q - path.back()) > 1e-5f) path.push_back(q);
    if (closedPath && path.size() > 2 && length(path.front() - path.back()) < 1e-5f) path.pop_back();
    if (path.size() < 2 || (closedPath && path.size() < 3)) return fail("The path needs at least two points");
    const std::size_t m = path.size(), segs = closedPath ? m : m - 1;
    std::vector<Vec3> dir(segs);
    for (std::size_t i = 0; i < segs; ++i) dir[i] = normalize(path[(i + 1) % m] - path[i]);
    // The profile, centred, in its plane's axes turned square to the path's start.
    std::vector<Vec2> poly = profilePolygon(profile);
    Vec2 c{};
    for (const Vec2& q : poly) c = {c.x + q.x, c.y + q.y};
    c = {c.x / static_cast<float>(poly.size()), c.y / static_cast<float>(poly.size())};
    for (Vec2& q : poly) q = {q.x - c.x, q.y - c.y};
    const PlaneFrame pf = planeFrame(profilePlane);
    std::vector<Vec3> nAxis(segs), bAxis(segs);
    nAxis[0] = turnLike(pf.u, pf.normal, dir[0]);
    bAxis[0] = turnLike(pf.v, pf.normal, dir[0]);
    // Carried along: turned at each corner by the corner's own rotation (parallel transport).
    for (std::size_t i = 1; i < segs; ++i) {
        nAxis[i] = turnLike(nAxis[i - 1], dir[i - 1], dir[i]);
        bAxis[i] = turnLike(bAxis[i - 1], dir[i - 1], dir[i]);
    }
    if (closedPath) { // the twist left after going round is shared out along the way
        const Vec3 back = turnLike(nAxis[segs - 1], dir[segs - 1], dir[0]);
        const float twist = std::atan2(dot(cross(back, nAxis[0]), dir[0]), dot(back, nAxis[0]));
        for (std::size_t i = 1; i < segs; ++i) {
            const float a = twist * static_cast<float>(i) / static_cast<float>(segs);
            nAxis[i] = rotateAbout(nAxis[i], dir[i], a);
            bAxis[i] = rotateAbout(bAxis[i], dir[i], a);
        }
    }
    // A section at each path point: square at open ends, on the mitre plane at corners.
    RawMesh mesh;
    const std::size_t n = poly.size();
    for (std::size_t i = 0; i < m; ++i) {
        const bool end = !closedPath && (i == 0 || i == m - 1);
        const std::size_t in = closedPath ? (i + segs - 1) % segs : (i == 0 ? 0 : i - 1); // the segment arriving
        const Vec3 di = dir[in];
        const Vec3 ni = nAxis[in], bi = bAxis[in];
        const Vec3 tNext = end ? di : dir[closedPath ? i % segs : i];
        const Vec3 miter = normalize(di + tNext);
        if (!end && dot(di, tNext) < -0.95f) return fail("The path doubles back on itself");
        for (const Vec2& q : poly) {
            Vec3 p = path[i] + ni * q.x + bi * q.y;
            if (!end) p += di * (dot(path[i] - p, miter) / dot(di, miter));
            mesh.p.push_back(p);
        }
    }
    const std::size_t rings = m;
    auto at = [&](std::size_t r, std::size_t j) { return static_cast<std::uint32_t>((r % rings) * n + j % n); };
    for (std::size_t r = 0; r < (closedPath ? rings : rings - 1); ++r)
        for (std::size_t j = 0; j < n; ++j) {
            mesh.tri(at(r, j), at(r, j + 1), at(r + 1, j + 1));
            mesh.tri(at(r, j), at(r + 1, j + 1), at(r + 1, j));
        }
    if (!closedPath) {
        const std::vector<std::uint32_t> t = triangulatePolygon(poly);
        if (t.empty()) return fail("The profile can't be capped (it crosses itself)");
        for (std::size_t q = 0; q + 2 < t.size(); q += 3) {
            mesh.tri(at(0, t[q]), at(0, t[q + 2]), at(0, t[q + 1]));
            mesh.tri(at(rings - 1, t[q]), at(rings - 1, t[q + 1]), at(rings - 1, t[q + 2]));
        }
    }
    return mesh.data();
}

std::optional<MeshData> loftSections(const std::vector<std::vector<Vec3>>& sectionsIn, bool smooth, std::string* error) {
    auto fail = [&](const char* why) -> std::optional<MeshData> {
        if (error) *error = why;
        return std::nullopt;
    };
    if (sectionsIn.size() < 2) return fail("A loft needs two sections or more");
    std::size_t n = 64;
    for (const auto& s : sectionsIn) {
        if (s.size() < 3) return fail("Each section must be a closed curve");
        n = std::max(n, std::min<std::size_t>(s.size(), 256));
    }
    std::vector<std::vector<Vec3>> sec;
    Vec3 n0{};
    for (std::size_t k = 0; k < sectionsIn.size(); ++k) {
        std::vector<Vec3> s = resampleLoop(sectionsIn[k], n);
        Vec3 nk{};
        for (std::size_t i = 0; i < s.size(); ++i) nk += cross(s[i], s[(i + 1) % s.size()]);
        if (k == 0) n0 = nk;
        else if (dot(nk, n0) < 0) std::reverse(s.begin(), s.end()); // all wound the same way
        if (k > 0) { // start where it lines up best with the section before
            const std::vector<Vec3>& prev = sec.back();
            const Vec3 cp = centroidOf(prev), cs = centroidOf(s);
            std::size_t best = 0;
            double bd = 1e300;
            for (std::size_t sh = 0; sh < n; ++sh) {
                double d = 0;
                for (std::size_t i = 0; i < n; i += 4) {
                    const Vec3 e = (s[(i + sh) % n] - cs) - (prev[i] - cp);
                    d += dot(e, e);
                }
                if (d < bd) bd = d, best = sh;
            }
            std::rotate(s.begin(), s.begin() + static_cast<std::ptrdiff_t>(best), s.end());
        }
        sec.push_back(std::move(s));
    }
    // Rings between the sections: straight, or a Catmull-Rom curve through them.
    const int sub = smooth ? 8 : 1;
    std::vector<std::vector<Vec3>> rings;
    const std::size_t ns = sec.size();
    for (std::size_t k = 0; k + 1 < ns; ++k)
        for (int s = 0; s < sub; ++s) {
            const float t = static_cast<float>(s) / static_cast<float>(sub);
            std::vector<Vec3> ring(n);
            for (std::size_t i = 0; i < n; ++i) {
                const Vec3& p1 = sec[k][i];
                const Vec3& p2 = sec[k + 1][i];
                if (!smooth) {
                    ring[i] = p1 + (p2 - p1) * t;
                    continue;
                }
                const Vec3 p0 = k > 0 ? sec[k - 1][i] : p1 * 2.0f - p2;
                const Vec3 p3 = k + 2 < ns ? sec[k + 2][i] : p2 * 2.0f - p1;
                const float t2 = t * t, t3 = t2 * t;
                ring[i] = (p1 * 2.0f + (p2 - p0) * t + (p0 * 2.0f - p1 * 5.0f + p2 * 4.0f - p3) * t2 + (p1 * 3.0f - p0 - p2 * 3.0f + p3) * t3) * 0.5f;
            }
            rings.push_back(std::move(ring));
        }
    rings.push_back(sec.back());
    RawMesh mesh;
    for (const auto& r : rings) mesh.p.insert(mesh.p.end(), r.begin(), r.end());
    auto at = [&](std::size_t r, std::size_t j) { return static_cast<std::uint32_t>(r * n + j % n); };
    for (std::size_t r = 0; r + 1 < rings.size(); ++r)
        for (std::size_t j = 0; j < n; ++j) {
            mesh.tri(at(r, j), at(r, j + 1), at(r + 1, j + 1));
            mesh.tri(at(r, j), at(r + 1, j + 1), at(r + 1, j));
        }
    const std::vector<std::uint32_t> c0 = capTriangles(rings.front()), c1 = capTriangles(rings.back());
    if (c0.empty() || c1.empty()) return fail("An end section can't be capped (it isn't flat, or it crosses itself)");
    for (std::size_t q = 0; q + 2 < c0.size(); q += 3) mesh.tri(at(0, c0[q]), at(0, c0[q + 2]), at(0, c0[q + 1]));
    const std::size_t last = rings.size() - 1;
    for (std::size_t q = 0; q + 2 < c1.size(); q += 3) mesh.tri(at(last, c1[q]), at(last, c1[q + 1]), at(last, c1[q + 2]));
    return mesh.data();
}

MeshData mirrorMesh(const MeshData& src, const Vec3& point, const Vec3& normalIn) {
    MeshData m = src;
    const Vec3 n = normalize(normalIn);
    for (Vertex& v : m.vertices) {
        v.position = v.position - n * (2.0f * dot(v.position - point, n));
        v.normal = v.normal - n * (2.0f * dot(v.normal, n));
    }
    for (std::size_t k = 0; k + 2 < m.indices.size(); k += 3) std::swap(m.indices[k + 1], m.indices[k + 2]);
    for (Vec3& e : m.edgeVertices) e = e - n * (2.0f * dot(e - point, n));
    return m;
}

// --- mass properties -----------------------------------------------------------------

SolidMass solidMass(const MeshData& m, double density) {
    // Volume integrals of 1, x, y, z, x^2, y^2, z^2, xy, yz, zx by the divergence
    // theorem, triangle by triangle (the polyhedral mass properties of Eberly).
    double in[10] = {};
    auto sub = [](double w0, double w1, double w2, double& f1, double& f2, double& f3, double& g0, double& g1, double& g2) {
        const double t0 = w0 + w1, t1 = w0 * w0, t2 = t1 + w1 * t0;
        f1 = t0 + w2;
        f2 = t2 + w2 * f1;
        f3 = w0 * t1 + w1 * t2 + w2 * f2;
        g0 = f2 + w0 * (f1 + w0);
        g1 = f2 + w1 * (f1 + w1);
        g2 = f2 + w2 * (f1 + w2);
    };
    SolidMass r;
    for (std::size_t k = 0; k + 2 < m.indices.size(); k += 3) {
        const Vec3& p0 = m.vertices[m.indices[k]].position;
        const Vec3& p1 = m.vertices[m.indices[k + 1]].position;
        const Vec3& p2 = m.vertices[m.indices[k + 2]].position;
        r.area += 0.5 * length(cross(p1 - p0, p2 - p0));
        r.box.expand(p0), r.box.expand(p1), r.box.expand(p2);
        const double x0 = p0.x, y0 = p0.y, z0 = p0.z, x1 = p1.x, y1 = p1.y, z1 = p1.z, x2 = p2.x, y2 = p2.y, z2 = p2.z;
        const double a1 = x1 - x0, b1 = y1 - y0, c1 = z1 - z0, a2 = x2 - x0, b2 = y2 - y0, c2 = z2 - z0;
        const double d0 = b1 * c2 - b2 * c1, d1 = a2 * c1 - a1 * c2, d2 = a1 * b2 - a2 * b1;
        double f1x, f2x, f3x, g0x, g1x, g2x, f1y, f2y, f3y, g0y, g1y, g2y, f1z, f2z, f3z, g0z, g1z, g2z;
        sub(x0, x1, x2, f1x, f2x, f3x, g0x, g1x, g2x);
        sub(y0, y1, y2, f1y, f2y, f3y, g0y, g1y, g2y);
        sub(z0, z1, z2, f1z, f2z, f3z, g0z, g1z, g2z);
        in[0] += d0 * f1x;
        in[1] += d0 * f2x, in[2] += d1 * f2y, in[3] += d2 * f2z;
        in[4] += d0 * f3x, in[5] += d1 * f3y, in[6] += d2 * f3z;
        in[7] += d0 * (y0 * g0x + y1 * g1x + y2 * g2x);
        in[8] += d1 * (z0 * g0y + z1 * g1y + z2 * g2y);
        in[9] += d2 * (x0 * g0z + x1 * g1z + x2 * g2z);
    }
    static const double kMult[10] = {1.0 / 6, 1.0 / 24, 1.0 / 24, 1.0 / 24, 1.0 / 60, 1.0 / 60, 1.0 / 60, 1.0 / 120, 1.0 / 120, 1.0 / 120};
    for (int i = 0; i < 10; ++i) in[i] *= kMult[i];
    r.volume = in[0];
    if (std::fabs(r.volume) < 1e-12) return r;
    const double cx = in[1] / in[0], cy = in[2] / in[0], cz = in[3] / in[0];
    r.centroid = {static_cast<float>(cx), static_cast<float>(cy), static_cast<float>(cz)};
    const double rho = density / 1000.0; // g/mm^3
    r.mass = r.volume * rho;
    const double v = in[0];
    double I[3][3];
    I[0][0] = (in[5] + in[6] - v * (cy * cy + cz * cz)) * rho;
    I[1][1] = (in[4] + in[6] - v * (cz * cz + cx * cx)) * rho;
    I[2][2] = (in[4] + in[5] - v * (cx * cx + cy * cy)) * rho;
    I[0][1] = I[1][0] = -(in[7] - v * cx * cy) * rho;
    I[1][2] = I[2][1] = -(in[8] - v * cy * cz) * rho;
    I[0][2] = I[2][0] = -(in[9] - v * cz * cx) * rho;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) r.inertia[i][j] = I[i][j];
    // Principal moments: Jacobi rotations on the symmetric tensor.
    double a[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) a[i][j] = I[i][j];
    for (int sweep = 0; sweep < 50; ++sweep) {
        const double off = a[0][1] * a[0][1] + a[0][2] * a[0][2] + a[1][2] * a[1][2];
        if (off < 1e-24 * (a[0][0] * a[0][0] + a[1][1] * a[1][1] + a[2][2] * a[2][2] + 1e-300)) break;
        for (int p = 0; p < 2; ++p)
            for (int q = p + 1; q < 3; ++q) {
                if (std::fabs(a[p][q]) < 1e-300) continue;
                const double th = (a[q][q] - a[p][p]) / (2 * a[p][q]);
                const double t = (th >= 0 ? 1.0 : -1.0) / (std::fabs(th) + std::sqrt(th * th + 1));
                const double c = 1 / std::sqrt(t * t + 1), s = t * c;
                for (int k = 0; k < 3; ++k) { // columns
                    const double akp = a[k][p], akq = a[k][q];
                    a[k][p] = c * akp - s * akq;
                    a[k][q] = s * akp + c * akq;
                }
                for (int k = 0; k < 3; ++k) { // rows
                    const double apk = a[p][k], aqk = a[q][k];
                    a[p][k] = c * apk - s * aqk;
                    a[q][k] = s * apk + c * aqk;
                }
            }
    }
    r.principal[0] = a[0][0], r.principal[1] = a[1][1], r.principal[2] = a[2][2];
    std::sort(r.principal, r.principal + 3);
    return r;
}

std::optional<double> materialDensity(const std::string& name) {
    std::string k;
    for (char ch : name) k += static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    static const std::pair<const char*, double> kTable[] = {
        {"steel", 7.85},    {"stainless", 8.0},  {"aluminium", 2.70}, {"aluminum", 2.70}, {"brass", 8.50},     {"copper", 8.96},
        {"castiron", 7.20}, {"iron", 7.87},      {"titanium", 4.43},  {"plastic", 1.20},  {"abs", 1.04},       {"pla", 1.24},
        {"petg", 1.27},     {"nylon", 1.14},     {"pom", 1.41},       {"acrylic", 1.18},  {"wood", 0.70},      {"water", 1.0},
        {"gold", 19.32},    {"silver", 10.49},   {"magnesium", 1.74}, {"carbon", 1.60},   {"concrete", 2.40},  {"glass", 2.50}};
    for (const auto& [key, rho] : kTable)
        if (k == key) return rho;
    return std::nullopt;
}

// --- distance --------------------------------------------------------------------------

namespace {

Vec3 closestOnTriangle(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c) {
    const Vec3 ab = b - a, ac = c - a, ap = p - a;
    const float d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0 && d2 <= 0) return a;
    const Vec3 bp = p - b;
    const float d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0 && d4 <= d3) return b;
    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) return a + ab * (d1 / (d1 - d3));
    const Vec3 cp = p - c;
    const float d5 = dot(ab, cp), d6 = dot(ac, cp);
    if (d6 >= 0 && d5 <= d6) return c;
    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) return a + ac * (d2 / (d2 - d6));
    const float va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    const float den = 1.0f / (va + vb + vc);
    return a + ab * (vb * den) + ac * (vc * den);
}

// Nearest point of mesh m to each point of `pts`: the overall closest pair.
double nearestPair(const std::vector<Vec3>& pts, const MeshData& m, Vec3* from, Vec3* to) {
    double best = 1e300;
    // Triangles binned on a coarse grid; points search outward ring by ring.
    const AABB box = m.bounds();
    const Vec3 ext = box.max - box.min;
    const std::size_t tris = m.indices.size() / 3;
    const float cell = std::max(std::cbrt(std::max(ext.x * ext.y * ext.z, 1e-6f) / static_cast<float>(std::max<std::size_t>(tris, 1))) * 2.0f,
                                std::max({ext.x, ext.y, ext.z}) / 64.0f + 1e-6f);
    const int nx = static_cast<int>(ext.x / cell) + 1, ny = static_cast<int>(ext.y / cell) + 1, nz = static_cast<int>(ext.z / cell) + 1;
    std::unordered_map<std::uint64_t, std::vector<std::uint32_t>> bins;
    auto key = [&](int i, int j, int k) { return (static_cast<std::uint64_t>(i) * 4096u + static_cast<std::uint64_t>(j)) * 4096u + static_cast<std::uint64_t>(k); };
    auto cellOf = [&](float v, float lo, int n) { return std::clamp(static_cast<int>((v - lo) / cell), 0, n - 1); };
    for (std::uint32_t t = 0; t < tris; ++t) {
        AABB tb;
        for (int c = 0; c < 3; ++c) tb.expand(m.vertices[m.indices[t * 3 + static_cast<std::uint32_t>(c)]].position);
        for (int i = cellOf(tb.min.x, box.min.x, nx); i <= cellOf(tb.max.x, box.min.x, nx); ++i)
            for (int j = cellOf(tb.min.y, box.min.y, ny); j <= cellOf(tb.max.y, box.min.y, ny); ++j)
                for (int k = cellOf(tb.min.z, box.min.z, nz); k <= cellOf(tb.max.z, box.min.z, nz); ++k) bins[key(i, j, k)].push_back(t);
    }
    for (const Vec3& p : pts) {
        // Outside the box the search starts at the nearest cell.
        const int ci = cellOf(p.x, box.min.x, nx), cj = cellOf(p.y, box.min.y, ny), ck = cellOf(p.z, box.min.z, nz);
        const Vec3 q{std::clamp(p.x, box.min.x, box.max.x), std::clamp(p.y, box.min.y, box.max.y), std::clamp(p.z, box.min.z, box.max.z)};
        const double outside = length(p - q);
        if (outside >= best) continue;
        double mine = 1e300;
        Vec3 mineTo{};
        const int maxRing = std::max({nx, ny, nz});
        for (int ring = 0; ring <= maxRing; ++ring) {
            if (ring > 0 && outside + (ring - 1) * static_cast<double>(cell) > std::min(mine, best)) break;
            for (int i = ci - ring; i <= ci + ring; ++i)
                for (int j = cj - ring; j <= cj + ring; ++j)
                    for (int k = ck - ring; k <= ck + ring; ++k) {
                        if (std::max({std::abs(i - ci), std::abs(j - cj), std::abs(k - ck)}) != ring) continue;
                        if (i < 0 || j < 0 || k < 0 || i >= nx || j >= ny || k >= nz) continue;
                        const auto it = bins.find(key(i, j, k));
                        if (it == bins.end()) continue;
                        for (std::uint32_t t : it->second) {
                            const Vec3 c = closestOnTriangle(p, m.vertices[m.indices[t * 3]].position, m.vertices[m.indices[t * 3 + 1]].position,
                                                             m.vertices[m.indices[t * 3 + 2]].position);
                            const double d = length(c - p);
                            if (d < mine) mine = d, mineTo = c;
                        }
                    }
        }
        if (mine < best) {
            best = mine;
            if (from) *from = p;
            if (to) *to = mineTo;
        }
    }
    return best;
}

} // namespace

double meshDistance(const MeshData& a, const MeshData& b, Vec3* onA, Vec3* onB) {
    if (a.indices.empty() || b.indices.empty()) return -1.0;
    std::vector<Vec3> pa, pb;
    for (const Vertex& v : a.vertices) pa.push_back(v.position);
    for (const Vertex& v : b.vertices) pb.push_back(v.position);
    Vec3 f1, t1, f2, t2;
    const double d1 = nearestPair(pa, b, &f1, &t1), d2 = nearestPair(pb, a, &f2, &t2);
    if (d1 <= d2) {
        if (onA) *onA = f1;
        if (onB) *onB = t1;
        return d1;
    }
    if (onA) *onA = t2;
    if (onB) *onB = f2;
    return d2;
}

// --- rounds (rolling-ball fillets) --------------------------------------------------------

namespace {

// A signed distance on a voxel grid with, at each voxel, the closest point of the
// surface it measures to. Layout (x * ny + y) * nz + z, as marchTets reads it.
struct DistanceGrid {
    int nx = 0, ny = 0, nz = 0;
    Vec3 origin;
    float h = 1;
    std::vector<float> d;  // signed: negative inside
    std::vector<Vec3> cp;  // closest surface point
    std::vector<char> has; // cp known
    std::size_t idx(int x, int y, int z) const {
        return (static_cast<std::size_t>(x) * static_cast<std::size_t>(ny) + static_cast<std::size_t>(y)) * static_cast<std::size_t>(nz) +
               static_cast<std::size_t>(z);
    }
    Vec3 at(int x, int y, int z) const {
        return origin + Vec3{(static_cast<float>(x) + 0.5f) * h, (static_cast<float>(y) + 0.5f) * h, (static_cast<float>(z) + 0.5f) * h};
    }
};

// Spreads closest points to every voxel: sweeps over the grid in all eight diagonal
// orders, each voxel taking a neighbour's closest point when it is nearer
// (fast-sweeping closest points; eight orders carry them every way).
void propagate(DistanceGrid& g) {
    for (int order = 0; order < 8; ++order) {
        const int sx = order & 1 ? -1 : 1, sy = order & 2 ? -1 : 1, sz = order & 4 ? -1 : 1;
        const int x0 = sx > 0 ? 0 : g.nx - 1, x1 = sx > 0 ? g.nx : -1;
        const int y0 = sy > 0 ? 0 : g.ny - 1, y1 = sy > 0 ? g.ny : -1;
        const int z0 = sz > 0 ? 0 : g.nz - 1, z1 = sz > 0 ? g.nz : -1;
        for (int x = x0; x != x1; x += sx)
            for (int y = y0; y != y1; y += sy)
                for (int z = z0; z != z1; z += sz) {
                    const std::size_t i = g.idx(x, y, z);
                    const Vec3 p = g.at(x, y, z);
                    float best = g.has[i] ? length(p - g.cp[i]) : 1e30f;
                    // The neighbours already swept in this order.
                    for (int dx = -1; dx <= 0; ++dx)
                        for (int dy = -1; dy <= 0; ++dy)
                            for (int dz = -1; dz <= 0; ++dz) {
                                if ((dx | dy | dz) == 0) continue;
                                const int xx = x + dx * sx, yy = y + dy * sy, zz = z + dz * sz;
                                if (xx < 0 || yy < 0 || zz < 0 || xx >= g.nx || yy >= g.ny || zz >= g.nz) continue;
                                const std::size_t j = g.idx(xx, yy, zz);
                                if (!g.has[j]) continue;
                                const float dd = length(p - g.cp[j]);
                                if (dd < best) {
                                    best = dd;
                                    g.cp[i] = g.cp[j];
                                    g.has[i] = 1;
                                }
                            }
                }
    }
}

// The solid's signed distance (exact near its surface), on a grid over `box`.
bool solidDistance(const MeshData& m, const AABB& box, float h, DistanceGrid& g) {
    g.h = h;
    g.origin = box.min;
    g.nx = static_cast<int>(std::ceil((box.max.x - box.min.x) / h));
    g.ny = static_cast<int>(std::ceil((box.max.y - box.min.y) / h));
    g.nz = static_cast<int>(std::ceil((box.max.z - box.min.z) / h));
    const std::size_t cells = static_cast<std::size_t>(g.nx) * static_cast<std::size_t>(g.ny) * static_cast<std::size_t>(g.nz);
    if (cells > 12'000'000 || g.nx < 2 || g.ny < 2 || g.nz < 2) return false;
    g.d.assign(cells, 0.0f);
    g.cp.assign(cells, Vec3{});
    g.has.assign(cells, 0);
    // Closest points next to each triangle.
    const std::size_t tris = m.indices.size() / 3;
    for (std::size_t t = 0; t < tris; ++t) {
        const Vec3& a = m.vertices[m.indices[t * 3]].position;
        const Vec3& b = m.vertices[m.indices[t * 3 + 1]].position;
        const Vec3& c = m.vertices[m.indices[t * 3 + 2]].position;
        AABB tb;
        tb.expand(a), tb.expand(b), tb.expand(c);
        const float pad = 1.5f * h;
        const int x0 = std::max(0, static_cast<int>((tb.min.x - pad - g.origin.x) / h)), x1 = std::min(g.nx - 1, static_cast<int>((tb.max.x + pad - g.origin.x) / h));
        const int y0 = std::max(0, static_cast<int>((tb.min.y - pad - g.origin.y) / h)), y1 = std::min(g.ny - 1, static_cast<int>((tb.max.y + pad - g.origin.y) / h));
        const int z0 = std::max(0, static_cast<int>((tb.min.z - pad - g.origin.z) / h)), z1 = std::min(g.nz - 1, static_cast<int>((tb.max.z + pad - g.origin.z) / h));
        for (int x = x0; x <= x1; ++x)
            for (int y = y0; y <= y1; ++y)
                for (int z = z0; z <= z1; ++z) {
                    const std::size_t i = g.idx(x, y, z);
                    const Vec3 p = g.at(x, y, z);
                    const Vec3 q = closestOnTriangle(p, a, b, c);
                    if (!g.has[i] || length(p - q) < length(p - g.cp[i])) {
                        g.cp[i] = q;
                        g.has[i] = 1;
                    }
                }
    }
    propagate(g);
    // Inside or out: crossings of a ray up each column (nudged off the grid's lines).
    std::vector<char> inside(cells, 0);
    std::vector<std::vector<std::uint32_t>> colTris(static_cast<std::size_t>(g.nx) * static_cast<std::size_t>(g.ny));
    for (std::uint32_t t = 0; t < tris; ++t) {
        AABB tb;
        for (int c = 0; c < 3; ++c) tb.expand(m.vertices[m.indices[t * 3 + static_cast<std::uint32_t>(c)]].position);
        const int x0 = std::max(0, static_cast<int>(std::floor((tb.min.x - g.origin.x) / h - 0.5f))), x1 = std::min(g.nx - 1, static_cast<int>(std::ceil((tb.max.x - g.origin.x) / h - 0.5f)));
        const int y0 = std::max(0, static_cast<int>(std::floor((tb.min.y - g.origin.y) / h - 0.5f))), y1 = std::min(g.ny - 1, static_cast<int>(std::ceil((tb.max.y - g.origin.y) / h - 0.5f)));
        for (int x = x0; x <= x1; ++x)
            for (int y = y0; y <= y1; ++y) colTris[static_cast<std::size_t>(x * g.ny + y)].push_back(t);
    }
    std::vector<float> hits;
    for (int x = 0; x < g.nx; ++x)
        for (int y = 0; y < g.ny; ++y) {
            const double px = g.origin.x + (x + 0.5) * h + h * 1.3e-4, py = g.origin.y + (y + 0.5) * h + h * 0.7e-4;
            hits.clear();
            for (std::uint32_t t : colTris[static_cast<std::size_t>(x * g.ny + y)]) {
                const Vec3& a = m.vertices[m.indices[t * 3]].position;
                const Vec3& b = m.vertices[m.indices[t * 3 + 1]].position;
                const Vec3& c = m.vertices[m.indices[t * 3 + 2]].position;
                const double den = (b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y);
                if (std::fabs(den) < 1e-15) continue;
                const double l1 = ((b.y - c.y) * (px - c.x) + (c.x - b.x) * (py - c.y)) / den;
                const double l2 = ((c.y - a.y) * (px - c.x) + (a.x - c.x) * (py - c.y)) / den;
                if (l1 < 0 || l2 < 0 || l1 + l2 > 1) continue;
                hits.push_back(static_cast<float>(l1 * a.z + l2 * b.z + (1 - l1 - l2) * c.z));
            }
            std::sort(hits.begin(), hits.end());
            std::size_t k = 0;
            bool in = false;
            for (int z = 0; z < g.nz; ++z) {
                const float pz = g.origin.z + (static_cast<float>(z) + 0.5f) * h;
                while (k < hits.size() && hits[k] < pz) in = !in, ++k;
                inside[g.idx(x, y, z)] = in ? 1 : 0;
            }
        }
    for (std::size_t i = 0; i < cells; ++i) {
        const int x = static_cast<int>(i / (static_cast<std::size_t>(g.ny) * static_cast<std::size_t>(g.nz)));
        const int y = static_cast<int>((i / static_cast<std::size_t>(g.nz)) % static_cast<std::size_t>(g.ny));
        const int z = static_cast<int>(i % static_cast<std::size_t>(g.nz));
        const float dist = g.has[i] ? length(g.at(x, y, z) - g.cp[i]) : 1e30f;
        g.d[i] = inside[i] ? -dist : dist;
    }
    return true;
}

// Triangles binned in space, for the exact closest point of the mesh to a point.
class TriBins {
public:
    TriBins(const MeshData& m, float cell) : m_m(m), m_cell(cell) {
        const AABB box = m.bounds();
        m_lo = box.min;
        for (std::uint32_t t = 0; t * 3 + 2 < m.indices.size(); ++t) {
            AABB b;
            for (int c = 0; c < 3; ++c) b.expand(m.vertices[m.indices[t * 3 + static_cast<std::uint32_t>(c)]].position);
            const auto lo = cellOf(b.min), hi = cellOf(b.max);
            for (int i = lo[0]; i <= hi[0]; ++i)
                for (int j = lo[1]; j <= hi[1]; ++j)
                    for (int k = lo[2]; k <= hi[2]; ++k) m_bins[key(i, j, k)].push_back(t);
        }
    }
    // The closest point of the mesh to p, searching within `reach`.
    Vec3 closest(const Vec3& p, float reach) const {
        const auto lo = cellOf(p - Vec3{reach, reach, reach}), hi = cellOf(p + Vec3{reach, reach, reach});
        thread_local std::vector<std::uint32_t> seen; // a big triangle sits in many bins: once each
        seen.clear();
        for (int i = lo[0]; i <= hi[0]; ++i)
            for (int j = lo[1]; j <= hi[1]; ++j)
                for (int k = lo[2]; k <= hi[2]; ++k) {
                    const auto it = m_bins.find(key(i, j, k));
                    if (it != m_bins.end()) seen.insert(seen.end(), it->second.begin(), it->second.end());
                }
        std::sort(seen.begin(), seen.end());
        seen.erase(std::unique(seen.begin(), seen.end()), seen.end());
        float best = 1e30f;
        Vec3 out = p;
        for (std::uint32_t t : seen) {
            const Vec3 q = closestOnTriangle(p, m_m.vertices[m_m.indices[t * 3]].position, m_m.vertices[m_m.indices[t * 3 + 1]].position,
                                             m_m.vertices[m_m.indices[t * 3 + 2]].position);
            const float d = length(q - p);
            if (d < best) best = d, out = q;
        }
        return out;
    }

private:
    std::array<int, 3> cellOf(const Vec3& p) const {
        return {static_cast<int>(std::floor((p.x - m_lo.x) / m_cell)), static_cast<int>(std::floor((p.y - m_lo.y) / m_cell)),
                static_cast<int>(std::floor((p.z - m_lo.z) / m_cell))};
    }
    static std::uint64_t key(int i, int j, int k) {
        return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(i + 100000)) << 42) ^
               (static_cast<std::uint64_t>(static_cast<std::uint32_t>(j + 100000)) << 21) ^ static_cast<std::uint64_t>(static_cast<std::uint32_t>(k + 100000));
    }
    const MeshData& m_m;
    float m_cell;
    Vec3 m_lo;
    std::unordered_map<std::uint64_t, std::vector<std::uint32_t>> m_bins;
};

// Morphological opening by a ball: the union of every ball of radius r inside the solid.
// Its centres are where d <= -r; the result is their distance less r. Closest points
// on that set's boundary start where the level d = -r crosses each voxel edge (found by
// interpolation, so exact where the field is) and spread out from there.
void openBy(DistanceGrid& g, float r, const TriBins& mesh) {
    DistanceGrid e = g;
    std::fill(e.has.begin(), e.has.end(), 0);
    auto seed = [&](std::size_t i, const Vec3& p, const Vec3& y) {
        if (!e.has[i] || length(p - y) < length(p - e.cp[i])) {
            e.cp[i] = y;
            e.has[i] = 1;
        }
    };
    for (int x = 0; x < g.nx; ++x)
        for (int y = 0; y < g.ny; ++y)
            for (int z = 0; z < g.nz; ++z) {
                const std::size_t i = g.idx(x, y, z);
                const float di = g.d[i] + r;
                const int nb[3][3] = {{x + 1, y, z}, {x, y + 1, z}, {x, y, z + 1}};
                for (const auto& q : nb) {
                    if (q[0] >= g.nx || q[1] >= g.ny || q[2] >= g.nz) continue;
                    const std::size_t j = g.idx(q[0], q[1], q[2]);
                    const float dj = g.d[j] + r;
                    if ((di <= 0) == (dj <= 0)) continue;
                    const Vec3 pi = g.at(x, y, z), pj = g.at(q[0], q[1], q[2]);
                    const Vec3 c = pi + (pj - pi) * (di / (di - dj));
                    seed(i, pi, c);
                    seed(j, pj, c);
                }
            }
    propagate(e);
    // Near the result's surface the closest point is made exact: projected steps toward
    // the voxel that stay r from the mesh, so it can settle on the centre set's edges
    // and corners (the seeds lie on its faces only). Steps short of r / 2 stay on the
    // centres' side of the surface.
    for (int x = 0; x < g.nx; ++x)
        for (int y = 0; y < g.ny; ++y)
            for (int z = 0; z < g.nz; ++z) {
                const std::size_t i = g.idx(x, y, z);
                if (!e.has[i] || g.d[i] <= -r) continue;
                const Vec3 p = g.at(x, y, z);
                if (std::fabs(length(p - e.cp[i]) - r) > 2.5f * g.h) continue;
                Vec3 c = e.cp[i];
                for (int it = 0; it < 16; ++it) {
                    const Vec3 step = p - c;
                    const float len = length(step);
                    if (len < 1e-6f) break;
                    Vec3 next = c + step * std::min(0.5f, 0.5f * r / len);
                    // Back to r from every face near it (alternating projections).
                    bool feasible = false;
                    for (int k = 0; k < 12; ++k) {
                        const Vec3 q = mesh.closest(next, r + g.h);
                        const float dq = length(next - q);
                        if (dq >= r - 1e-4f * r || dq < 1e-9f) {
                            feasible = dq >= r - 1e-3f * r;
                            break;
                        }
                        next = q + (next - q) * (r / dq);
                    }
                    if (!feasible || length(p - next) >= length(p - c) - 1e-6f) break;
                    c = next;
                }
                e.cp[i] = c;
            }
    for (int x = 0; x < g.nx; ++x)
        for (int y = 0; y < g.ny; ++y)
            for (int z = 0; z < g.nz; ++z) {
                const std::size_t i = g.idx(x, y, z);
                const Vec3 p = g.at(x, y, z);
                if (!e.has[i]) {
                    g.d[i] = 1e30f;
                    g.has[i] = 0;
                    continue;
                }
                const Vec3 c = e.cp[i];
                const float de = length(p - c);
                const bool inE = g.d[i] <= -r;
                g.d[i] = (inE ? -de : de) - r;
                // The new surface's closest point: r from the centre set, on the line through p.
                const Vec3 away = de > 1e-9f ? (p - c) / de : Vec3{0, 0, 1};
                g.cp[i] = inE ? c - away * r : c + away * r;
                g.has[i] = 1;
            }
}

void negate(DistanceGrid& g) {
    for (float& v : g.d) v = -v;
}

} // namespace

bool roundBody(Body& body, const RoundOptions& o, std::string* error) {
    auto fail = [&](const std::string& why) {
        if (error) *error = why;
        return false;
    };
    if (!(o.radius > 0)) return fail("The radius must be positive");
    if (!o.convex && !o.concave) return fail("Round the convex edges, the concave ones, or both");
    const MeshData world = worldMesh(body);
    if (world.indices.empty()) return fail(body.name + " has no surface");
    if (openEdgeCount(world) > 0) return fail(body.name + " isn't closed (REPAIR it first)");
    AABB box = world.bounds();
    const Vec3 ext = box.max - box.min;
    float h = o.voxel > 0 ? o.voxel : o.radius / 4.0f;
    const float pad = o.radius * (o.concave ? 2.0f : 1.0f) + 3.0f * h;
    auto cellsFor = [&](float hh) {
        return static_cast<double>((ext.x + 2 * pad) / hh) * ((ext.y + 2 * pad) / hh) * ((ext.z + 2 * pad) / hh);
    };
    while (cellsFor(h) > 8e6) h *= 1.15f;
    if (o.radius < h * 2.0f) return fail("The radius is too small for the part's size here (under two voxels of " + std::to_string(h) + " mm)");
    box.min = box.min - Vec3{pad, pad, pad};
    box.max = box.max + Vec3{pad, pad, pad};
    DistanceGrid g;
    if (!solidDistance(world, box, h, g)) return fail("The part is too big to round at this radius");
    const double before = meshVolume(world);
    const TriBins bins(world, o.radius + h); // a query reaches r + h: 27 bins at most
    if (o.convex) openBy(g, o.radius, bins);
    if (o.concave) {
        // After a convex round the surface is the rounded one: its closest points are no
        // longer on the mesh, so the concave pass works from a mesh of that surface.
        negate(g);
        if (o.convex) {
            MeshData mid = marchTets(g.d, g.nx, g.ny, g.nz, g.origin, h, false);
            const TriBins midBins(mid, o.radius + h);
            openBy(g, o.radius, midBins);
        } else
            openBy(g, o.radius, bins);
        negate(g);
    }
    MeshData out = marchTets(g.d, g.nx, g.ny, g.nz, g.origin, h, true);
    if (out.indices.empty()) return fail("Nothing is left: every part of " + body.name + " is thinner than the ball");
    finishMarched(out, 1);
    simplifyMesh(out, h * 0.8f);
    const Vec3 color = body.color;
    Body b = featureBody(body.name, std::move(out), color);
    if (openEdgeCount(b.mesh) > 0) return fail("Rounding left holes in " + body.name);
    const double after = meshVolume(b.mesh);
    if (after < before * 0.2) return fail("Rounding would remove most of " + body.name + ": the ball is too big for it");
    b.visible = body.visible;
    b.component = body.component;
    b.attachedTo = body.attachedTo;
    b.physics = body.physics;
    body = std::move(b);
    return true;
}

} // namespace sicdm

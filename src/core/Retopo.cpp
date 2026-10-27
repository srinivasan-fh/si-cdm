#include "core/Retopo.hpp"

#include "core/Multires.hpp"
#include "core/Scene.hpp"
#include "core/Sculpt.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <unordered_map>

namespace sicdm {

namespace {

constexpr float kPiF = 3.14159265358979f;

std::uint64_t keyOf(std::uint32_t a, std::uint32_t b);

std::uint64_t keyOf(std::uint32_t a, std::uint32_t b) {
    if (a > b) std::swap(a, b);
    return (static_cast<std::uint64_t>(a) << 32) | b;
}

// A unit tangent of a face and the one across it (a frame on the face's plane).
struct FaceFrame {
    Vec3 n, t1, t2;
};

// The original's sharp edges (faces meeting at more than 40 degrees) and corners (where
// three or more meet, or one ends), for the new mesh to keep.
struct Features {
    std::vector<std::pair<Vec3, Vec3>> lines;
    std::vector<Vec3> corners;
    // The nearest corner within `cornerReach`, else the nearest point on a line within
    // `lineReach` (with the line's direction): 2 corner, 1 line, 0 neither.
    int snap(const Vec3& p, float cornerReach, float lineReach, Vec3& at, Vec3* dir = nullptr) const {
        float best = cornerReach * cornerReach;
        int kind = 0;
        for (const Vec3& c : corners)
            if (dot(c - p, c - p) < best) best = dot(c - p, c - p), at = c, kind = 2;
        if (kind) return kind;
        best = lineReach * lineReach;
        for (const auto& [a, b] : lines) {
            const Vec3 ab = b - a;
            const float l2 = dot(ab, ab);
            const float t = l2 > 0 ? std::clamp(dot(p - a, ab) / l2, 0.0f, 1.0f) : 0.0f;
            const Vec3 q = a + ab * t;
            if (dot(q - p, q - p) < best) {
                best = dot(q - p, q - p), at = q, kind = 1;
                if (dir) *dir = normalize(ab);
            }
        }
        return kind;
    }
};

Features featuresOf(const SculptMesh& m) {
    Features f;
    const auto& P = m.positions();
    const auto& T = m.triangles();
    std::unordered_map<std::uint64_t, std::vector<Vec3>> normals;
    for (const auto& t : T) {
        const Vec3 n = normalize(cross(P[t[1]] - P[t[0]], P[t[2]] - P[t[0]]));
        for (int e = 0; e < 3; ++e) normals[keyOf(t[static_cast<std::size_t>(e)], t[static_cast<std::size_t>((e + 1) % 3)])].push_back(n);
    }
    const float cosLimit = std::cos(40.0f * kPiF / 180.0f);
    std::unordered_map<std::uint32_t, int> degree;
    for (const auto& [k, ns] : normals) {
        if (ns.size() != 2 || dot(ns[0], ns[1]) >= cosLimit) continue;
        const auto a = static_cast<std::uint32_t>(k >> 32), b = static_cast<std::uint32_t>(k & 0xFFFFFFFFu);
        f.lines.push_back({P[a], P[b]});
        ++degree[a], ++degree[b];
    }
    for (const auto& [v, d] : degree)
        if (d != 2) f.corners.push_back(P[v]);
    if (f.lines.size() > 20000) f = {}; // a dense scan's creases: no features to keep
    return f;
}

// Angles of a four-way direction field are kept as the unit complex number e^(4i angle):
// the four directions that are the same field direction become one number.
struct C2 {
    float re = 0, im = 0;
};

} // namespace

bool retopologize(const Body& source, const RetopoOptions& opt, MeshData& out, RetopoReport* report, std::string* why) {
    const SculptMesh src(source);
    if (src.empty()) {
        if (why) *why = source.name + " has no surface";
        return false;
    }
    if (src.openEdges() != 0) {
        if (why) *why = source.name + " isn't closed (REPAIR it, or REMESH it first)";
        return false;
    }
    // 1. Even triangles, about half as many as the quads wanted (each later pair of
    // triangles becomes one quad, split in four).
    double area = 0;
    for (const auto& t : src.triangles())
        area += 0.5 * static_cast<double>(length(cross(src.positions()[t[1]] - src.positions()[t[0]], src.positions()[t[2]] - src.positions()[t[0]])));
    const double tris = std::max(40.0, static_cast<double>(std::max(opt.faces, 20)) / 1.55); // pairs, then each face in quads
    const float L = static_cast<float>(std::sqrt(4.0 * area / (std::sqrt(3.0) * tris)));
    SculptMesh m = src;
    m.remeshIsotropic(L, std::max(opt.iterations, 1), src, 40.0f);
    std::vector<Vec3> P = m.positions();
    const std::vector<std::array<std::uint32_t, 3>> T = m.triangles();
    // The original's sharp edges and corners: vertices near them go onto them.
    const Features feat = featuresOf(src);
    std::vector<int> onFeature(P.size(), 0);
    if (!feat.lines.empty())
        for (std::size_t v = 0; v < P.size(); ++v) {
            Vec3 at;
            if (const int k = feat.snap(P[v], 0.6f * L, 0.45f * L, at)) P[v] = at, onFeature[v] = k;
        }
    if (T.size() < 4) {
        if (why) *why = "The surface is too small for that many faces";
        return false;
    }
    // 2. A four-way direction field over the faces: smooth, and along the principal
    // curvature where the surface is clearly more curved one way than the other.
    const std::size_t F = T.size();
    std::vector<FaceFrame> frame(F);
    std::vector<Vec3> vn(P.size(), Vec3{});
    for (std::size_t f = 0; f < F; ++f) {
        const auto& t = T[f];
        const Vec3 c = cross(P[t[1]] - P[t[0]], P[t[2]] - P[t[0]]);
        for (std::uint32_t v : t) vn[v] += c;
        frame[f].n = normalize(c);
        frame[f].t1 = normalize(P[t[1]] - P[t[0]]);
        frame[f].t2 = cross(frame[f].n, frame[f].t1);
    }
    for (Vec3& n : vn) n = normalize(n);
    std::vector<C2> prior(F), field(F);
    std::vector<float> weight(F, 0.0f);
    for (std::size_t f = 0; f < F; ++f) {
        // The shape operator from how the normals turn along the edges (least squares on
        // the face's plane): S e = dn for each edge e.
        const auto& t = T[f];
        const FaceFrame& fr = frame[f];
        // Least squares for S = [s11 s12; s12 s22]: the normal equations' matrix and right side.
        double b1 = 0, b2 = 0, b3 = 0, m11 = 0, m12 = 0, m22 = 0, m33 = 0;
        for (int e = 0; e < 3; ++e) {
            const std::uint32_t i = t[static_cast<std::size_t>(e)], j = t[static_cast<std::size_t>((e + 1) % 3)];
            const Vec3 ed = P[j] - P[i], dn = vn[j] - vn[i];
            const double ex = dot(ed, fr.t1), ey = dot(ed, fr.t2), nx = dot(dn, fr.t1), ny = dot(dn, fr.t2);
            // nx = s11 ex + s12 ey, ny = s12 ex + s22 ey: unknowns (s11, s12, s22).
            m11 += ex * ex, m12 += ex * ey, m22 += ey * ey + ex * ex, m33 += ey * ey;
            b1 += nx * ex, b2 += nx * ey + ny * ex, b3 += ny * ey;
        }
        // [[sum ex2, sum exey, 0], [sum exey, sum ex2 + ey2, sum exey], [0, sum exey, sum ey2]] x = b
        const double A[3][3] = {{m11, m12, 0}, {m12, m22, m12}, {0, m12, m33}};
        const double B[3] = {b1, b2, b3};
        double M[3][4];
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) M[r][c] = A[r][c];
            M[r][3] = B[r];
        }
        bool ok = true;
        for (int c = 0; c < 3 && ok; ++c) {
            int piv = c;
            for (int r = c + 1; r < 3; ++r)
                if (std::fabs(M[r][c]) > std::fabs(M[piv][c])) piv = r;
            if (std::fabs(M[piv][c]) < 1e-18) {
                ok = false;
                break;
            }
            for (int k = 0; k < 4; ++k) std::swap(M[c][k], M[piv][k]);
            for (int r = 0; r < 3; ++r) {
                if (r == c) continue;
                const double fct = M[r][c] / M[c][c];
                for (int k = c; k < 4; ++k) M[r][k] -= fct * M[c][k];
            }
        }
        if (!ok) continue;
        const double s11 = M[0][3] / M[0][0], s12 = M[1][3] / M[1][1], s22 = M[2][3] / M[2][2];
        // Principal directions: the eigenvectors of S; the field follows them where the two
        // curvatures differ clearly.
        const double tr = s11 + s22, det = s11 * s22 - s12 * s12;
        const double disc = std::sqrt(std::max(0.0, tr * tr / 4 - det));
        const double k1 = tr / 2 + disc, k2 = tr / 2 - disc;
        const double aniso = (std::fabs(k1) + std::fabs(k2)) > 1e-12 ? std::fabs(k1 - k2) / (std::fabs(k1) + std::fabs(k2)) : 0.0;
        const double angle = 0.5 * std::atan2(2 * s12, s11 - s22); // of the first principal direction
        const double strength = std::fabs(k1 - k2) * static_cast<double>(L); // turning across one edge
        if (opt.curvature && aniso > 0.35 && strength > 0.05) {
            weight[f] = static_cast<float>(std::min(1.0, aniso * std::min(1.0, strength * 4)));
            prior[f] = {static_cast<float>(std::cos(4 * angle)), static_cast<float>(std::sin(4 * angle))};
        }
    }
    // Faces with an edge along a sharp edge of the original take that edge's direction.
    for (std::size_t f = 0; f < F && !feat.lines.empty(); ++f)
        for (int e = 0; e < 3; ++e) {
            const std::uint32_t a = T[f][static_cast<std::size_t>(e)], b = T[f][static_cast<std::size_t>((e + 1) % 3)];
            if (!onFeature[a] || !onFeature[b]) continue;
            Vec3 at, dir;
            if (feat.snap((P[a] + P[b]) * 0.5f, 0.0f, 0.05f * L, at, &dir) != 1) continue;
            const Vec3 ed = normalize(P[b] - P[a]);
            if (std::fabs(dot(ed, dir)) < 0.95f) continue;
            const float ang = std::atan2(dot(ed, frame[f].t2), dot(ed, frame[f].t1));
            prior[f] = {std::cos(4 * ang), std::sin(4 * ang)};
            weight[f] = 2.0f;
        }
    // Faces across each edge.
    std::unordered_map<std::uint64_t, std::vector<std::uint32_t>> edgeFaces;
    for (std::size_t f = 0; f < F; ++f)
        for (int e = 0; e < 3; ++e) edgeFaces[keyOf(T[f][static_cast<std::size_t>(e)], T[f][static_cast<std::size_t>((e + 1) % 3)])].push_back(static_cast<std::uint32_t>(f));
    std::vector<std::array<int, 3>> nb(F, {-1, -1, -1});
    for (std::size_t f = 0; f < F; ++f)
        for (int e = 0; e < 3; ++e) {
            const auto& fs = edgeFaces[keyOf(T[f][static_cast<std::size_t>(e)], T[f][static_cast<std::size_t>((e + 1) % 3)])];
            for (std::uint32_t g : fs)
                if (g != f) nb[f][static_cast<std::size_t>(e)] = static_cast<int>(g);
        }
    // A direction of face g as an angle on face f's frame.
    auto direction = [&](std::size_t f, const C2& u) { // one of the four directions, in 3D
        const float a = std::atan2(u.im, u.re) / 4;
        return frame[f].t1 * std::cos(a) + frame[f].t2 * std::sin(a);
    };
    auto onFace = [&](std::size_t f, const Vec3& d) -> C2 {
        const float a = std::atan2(dot(d, frame[f].t2), dot(d, frame[f].t1));
        return {std::cos(4 * a), std::sin(4 * a)};
    };
    for (std::size_t f = 0; f < F; ++f) field[f] = weight[f] > 0 ? prior[f] : onFace(f, Vec3{1, 0.31f, 0.17f}); // a start off the axes
    for (int it = 0; it < 60; ++it) {
        std::vector<C2> next(F);
        for (std::size_t f = 0; f < F; ++f) {
            C2 s{field[f].re * 0.5f, field[f].im * 0.5f};
            for (int k : nb[f]) {
                if (k < 0) continue;
                const C2 g = onFace(f, direction(static_cast<std::size_t>(k), field[static_cast<std::size_t>(k)]));
                s.re += g.re, s.im += g.im;
            }
            const float w = weight[f] * 6.0f;
            s.re += prior[f].re * w, s.im += prior[f].im * w;
            const float l = std::hypot(s.re, s.im);
            next[f] = l > 1e-9f ? C2{s.re / l, s.im / l} : field[f];
        }
        field = std::move(next);
    }
    // 3. Triangles paired into quads: square corners, edges along the field, greedily best first.
    struct Pair {
        float score;
        std::uint32_t f, g;
        std::array<std::uint32_t, 4> q;
    };
    std::vector<Pair> pairs;
    for (const auto& [k, fs] : edgeFaces) {
        if (fs.size() != 2) continue;
        const std::uint32_t f = fs[0], g = fs[1];
        const auto a0 = static_cast<std::uint32_t>(k >> 32), b0 = static_cast<std::uint32_t>(k & 0xFFFFFFFFu);
        // f = (a, b, c) in its order with the shared edge a -> b; g then has b -> a, and d.
        const auto& tf = T[f];
        int i = 0;
        while (!((tf[static_cast<std::size_t>(i)] == a0 && tf[static_cast<std::size_t>((i + 1) % 3)] == b0) ||
                 (tf[static_cast<std::size_t>(i)] == b0 && tf[static_cast<std::size_t>((i + 1) % 3)] == a0)) && i < 3)
            ++i;
        if (i == 3) continue;
        const std::uint32_t a = tf[static_cast<std::size_t>(i)], b = tf[static_cast<std::size_t>((i + 1) % 3)], c = tf[static_cast<std::size_t>((i + 2) % 3)];
        std::uint32_t d = a;
        for (std::uint32_t v : T[g])
            if (v != a && v != b) d = v;
        if (d == a || d == c) continue;
        const std::array<std::uint32_t, 4> q{a, d, b, c};
        float worst = 0;
        bool convex = true;
        for (int j = 0; j < 4; ++j) {
            const Vec3 p0 = P[q[static_cast<std::size_t>((j + 3) % 4)]], p1 = P[q[static_cast<std::size_t>(j)]], p2 = P[q[static_cast<std::size_t>((j + 1) % 4)]];
            const Vec3 u = normalize(p0 - p1), v = normalize(p2 - p1);
            const float ang = std::acos(std::clamp(dot(u, v), -1.0f, 1.0f)) * 180.0f / kPiF;
            if (ang > 165.0f) convex = false;
            worst = std::max(worst, std::fabs(ang - 90.0f));
        }
        if (!convex) continue;
        const float square = 1.0f - worst / 90.0f;
        float align = 0;
        const Vec3 df = direction(f, field[f]);
        for (int j = 0; j < 4; ++j) {
            Vec3 e = P[q[static_cast<std::size_t>((j + 1) % 4)]] - P[q[static_cast<std::size_t>(j)]];
            e -= frame[f].n * dot(e, frame[f].n);
            const float le = length(e);
            if (le < 1e-12f) continue;
            const float cs = dot(e, df) / le;
            const float c4 = 8 * cs * cs * cs * cs - 8 * cs * cs + 1; // cos 4 angle
            align += 0.5f * (1 + c4) / 4;
        }
        pairs.push_back({square * (0.35f + 0.65f * align), f, g, q});
    }
    std::sort(pairs.begin(), pairs.end(), [](const Pair& x, const Pair& y) { return x.score > y.score; });
    std::vector<char> paired(F, 0);
    std::vector<std::vector<std::uint32_t>> polys;
    for (const Pair& p : pairs) {
        if (paired[p.f] || paired[p.g] || p.score < 0.3f) continue;
        paired[p.f] = paired[p.g] = 1;
        polys.push_back({p.q[0], p.q[1], p.q[2], p.q[3]});
    }
    for (std::size_t f = 0; f < F; ++f)
        if (!paired[f]) polys.push_back({T[f][0], T[f][1], T[f][2]});
    // 4. Every face split into quads: a point mid-edge and one in the middle.
    const SurfaceIndex index(src);
    std::vector<Vec3> Q = P;
    std::unordered_map<std::uint64_t, std::uint32_t> mid;
    std::vector<int> kindQ = onFeature; // 2 corner, 1 on a sharp edge, 0 on the surface
    auto midpoint = [&](std::uint32_t a, std::uint32_t b) {
        const auto [it, fresh] = mid.try_emplace(keyOf(a, b), static_cast<std::uint32_t>(Q.size()));
        if (fresh) {
            const Vec3 c = (P[a] + P[b]) * 0.5f;
            Vec3 at;
            if (onFeature[a] && onFeature[b] && feat.snap(c, 0.0f, 0.05f * L, at) == 1) Q.push_back(at), kindQ.push_back(1); // on the same sharp edge
            else Q.push_back(index.closestPoint(c)), kindQ.push_back(0);
        }
        return it->second;
    };
    std::vector<std::array<std::uint32_t, 4>> quads;
    quads.reserve(polys.size() * 4);
    for (const auto& poly : polys) {
        Vec3 c{};
        for (std::uint32_t v : poly) c += P[v];
        const auto centre = static_cast<std::uint32_t>(Q.size());
        Q.push_back(index.closestPoint(c * (1.0f / static_cast<float>(poly.size()))));
        kindQ.push_back(0);
        const std::size_t n = poly.size();
        for (std::size_t j = 0; j < n; ++j) {
            const std::uint32_t v = poly[j], prev = poly[(j + n - 1) % n], next = poly[(j + 1) % n];
            quads.push_back({v, midpoint(v, next), centre, midpoint(prev, v)});
        }
    }
    // 5. Relaxed over the original surface (each point towards its neighbours' middle,
    // along the surface), then onto it.
    std::vector<std::vector<std::uint32_t>> ring(Q.size());
    std::map<std::uint64_t, int> qedges;
    for (const auto& q : quads)
        for (int j = 0; j < 4; ++j) {
            const std::uint32_t a = q[static_cast<std::size_t>(j)], b = q[static_cast<std::size_t>((j + 1) % 4)];
            if (qedges[keyOf(a, b)]++ == 0) ring[a].push_back(b), ring[b].push_back(a);
        }
    for (int it = 0; it < 100; ++it) {
        std::vector<Vec3> n(Q.size(), Vec3{});
        for (const auto& q : quads) {
            const Vec3 c4 = cross(Q[q[2]] - Q[q[0]], Q[q[3]] - Q[q[1]]);
            for (std::uint32_t v : q) n[v] += c4;
        }
        std::vector<Vec3> next = Q;
        for (std::size_t v = 0; v < Q.size(); ++v) {
            if (ring[v].empty() || kindQ[v] == 2) continue; // corners stay
            Vec3 avg{};
            for (std::uint32_t w : ring[v]) avg += Q[w];
            avg = avg * (1.0f / static_cast<float>(ring[v].size()));
            const Vec3 nn = normalize(n[v]);
            Vec3 d = avg - Q[v];
            if (kindQ[v] == 1) { // along its sharp edge only
                Vec3 at;
                if (feat.snap(Q[v] + d * 0.6f, 0.0f, 0.5f * L, at) == 1) next[v] = at;
                continue;
            }
            d -= nn * dot(d, nn);
            next[v] = index.closestPoint(Q[v] + d * 0.6f);
        }
        Q = std::move(next);
    }
    // The mesh: two triangles a quad (split on the shorter diagonal), smooth normals,
    // the quads' edges as its lines.
    out = MeshData{};
    out.vertices.resize(Q.size());
    std::vector<Vec3> n(Q.size(), Vec3{});
    for (const auto& q : quads) {
        const Vec3 c = cross(Q[q[2]] - Q[q[0]], Q[q[3]] - Q[q[1]]);
        for (std::uint32_t v : q) n[v] += c;
    }
    for (std::size_t v = 0; v < Q.size(); ++v) out.vertices[v].position = Q[v] - source.offset, out.vertices[v].normal = normalize(n[v]);
    out.quads = quads;
    out.indices.reserve(quads.size() * 6);
    for (const auto& q : quads) {
        if (length(Q[q[0]] - Q[q[2]]) <= length(Q[q[1]] - Q[q[3]])) out.indices.insert(out.indices.end(), {q[0], q[1], q[2], q[0], q[2], q[3]});
        else out.indices.insert(out.indices.end(), {q[0], q[1], q[3], q[1], q[2], q[3]});
    }
    for (const auto& [k, c] : qedges) {
        (void)c;
        out.edgeVertices.push_back(out.vertices[static_cast<std::size_t>(k >> 32)].position);
        out.edgeVertices.push_back(out.vertices[static_cast<std::size_t>(k & 0xFFFFFFFFu)].position);
    }
    if (report) {
        *report = {};
        report->quads = quads.size();
        std::vector<int> val(Q.size(), 0);
        for (const auto& [k, c] : qedges) {
            (void)c;
            ++val[static_cast<std::size_t>(k >> 32)], ++val[static_cast<std::size_t>(k & 0xFFFFFFFFu)];
        }
        for (int v : val) report->irregular += v != 4 && v > 0 ? 1 : 0;
        double edgeSum = 0, sq = 0;
        for (const auto& [k, c] : qedges) {
            (void)c;
            edgeSum += static_cast<double>(length(Q[static_cast<std::size_t>(k >> 32)] - Q[static_cast<std::size_t>(k & 0xFFFFFFFFu)]));
        }
        report->edge = qedges.empty() ? 0.0f : static_cast<float>(edgeSum / static_cast<double>(qedges.size()));
        for (const auto& q : quads) {
            float worst = 0;
            for (int j = 0; j < 4; ++j) {
                const Vec3 u = normalize(Q[q[static_cast<std::size_t>((j + 3) % 4)]] - Q[q[static_cast<std::size_t>(j)]]);
                const Vec3 v = normalize(Q[q[static_cast<std::size_t>((j + 1) % 4)]] - Q[q[static_cast<std::size_t>(j)]]);
                worst = std::max(worst, std::fabs(std::acos(std::clamp(dot(u, v), -1.0f, 1.0f)) * 180.0f / kPiF - 90.0f));
            }
            sq += 1.0 - static_cast<double>(worst) / 90.0;
        }
        report->quality = quads.empty() ? 0.0f : static_cast<float>(sq / static_cast<double>(quads.size()));
    }
    return true;
}

void projectLevelsOnto(Body& body, const Body& source) {
    if (!body.multires) return;
    const SculptMesh src(source);
    if (src.empty()) return;
    const SurfaceIndex index(src);
    auto md = std::make_shared<MultiresData>(*body.multires);
    for (MultiresLevel& l : md->levels)
        for (Vec3& p : l.p) p = index.closestPoint(p + body.offset) - body.offset;
    body.mesh = multiresMesh(md->levels[static_cast<std::size_t>(md->current)]);
    body.multires = std::move(md);
}

} // namespace sicdm

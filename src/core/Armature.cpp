// Armatures: a tree of balls joined by tapered links, posed by turning balls, skinned into
// one smooth mesh whose vertices follow the links near them.

#include "core/Armature.hpp"
#include "core/UvMap.hpp"

#include "core/Wearable.hpp"

#include <algorithm>
#include <map>
#include <cmath>
#include <functional>

namespace sicdm {

namespace {

using Quat = std::array<float, 4>;

// The distance from p to the solid two balls and the cone tangent to both make (their
// convex hull), negative inside. In the plane through the axis, with `a` at the origin and
// the axis along +t: the cone's side touches ball a at angle (sin s, cos c) from the axis.
float linkDistance(const Vec3& p, const Vec3& a, float ra, const Vec3& b, float rb) {
    const Vec3 ab = b - a;
    const float L = length(ab);
    if (!(L > 1e-9f) || std::fabs(ra - rb) >= L) // one ball holds the other
        return std::min(length(p - a) - ra, length(p - b) - rb);
    const Vec3 u = ab * (1.0f / L);
    const float t = dot(p - a, u);
    const float q = length((p - a) - u * t);
    const float s = (ra - rb) / L, c = std::sqrt(std::max(0.0f, 1.0f - s * s));
    // Along the side, from where it touches ball a (0) to where it touches ball b (c L).
    const float w = (t - ra * s) * c - (q - ra * c) * s;
    if (w < 0) return length(p - a) - ra;
    if (w > c * L) return length(p - b) - rb;
    return t * s + q * c - ra;
}

// A rounded minimum (the two surfaces blended over about k).
float smoothMin(float a, float b, float k) {
    if (!(k > 0)) return std::min(a, b);
    const float h = std::max(k - std::fabs(a - b), 0.0f) / k;
    return std::min(a, b) - h * h * k * 0.25f;
}

// A ring of `n` points round `centre`, square to `axis`.
void frameFor(const Vec3& axis, Vec3& e1, Vec3& e2) {
    const Vec3 any = std::fabs(axis.z) < 0.9f ? Vec3{0, 0, 1} : Vec3{1, 0, 0};
    e1 = normalize(cross(axis, any));
    e2 = cross(axis, e1);
}

void addSphere(MeshData& m, const Vec3& c, float r, int seg = 16, int rings = 10) {
    const auto base = static_cast<std::uint32_t>(m.vertices.size());
    for (int i = 0; i <= rings; ++i) {
        const float th = kPi * static_cast<float>(i) / static_cast<float>(rings);
        for (int j = 0; j <= seg; ++j) {
            const float ph = 2 * kPi * static_cast<float>(j) / static_cast<float>(seg);
            const Vec3 n{std::sin(th) * std::cos(ph), std::sin(th) * std::sin(ph), std::cos(th)};
            m.vertices.push_back({c + n * r, n});
        }
    }
    for (int i = 0; i < rings; ++i)
        for (int j = 0; j < seg; ++j) {
            const std::uint32_t a = base + static_cast<std::uint32_t>(i * (seg + 1) + j), b = a + 1, d = a + static_cast<std::uint32_t>(seg + 1), e = d + 1;
            m.indices.insert(m.indices.end(), {a, d, b, b, d, e});
        }
}

// The cone tangent to two balls (just its side).
void addLink(MeshData& m, const Vec3& a, float ra, const Vec3& b, float rb, int seg = 16) {
    const Vec3 ab = b - a;
    const float L = length(ab);
    if (!(L > 1e-6f) || std::fabs(ra - rb) >= L) return;
    const Vec3 u = ab * (1.0f / L);
    const float s = (ra - rb) / L, c = std::sqrt(std::max(0.0f, 1.0f - s * s));
    Vec3 e1, e2;
    frameFor(u, e1, e2);
    const auto base = static_cast<std::uint32_t>(m.vertices.size());
    for (int end = 0; end < 2; ++end) {
        const Vec3 centre = end == 0 ? a : b;
        const float r = end == 0 ? ra : rb;
        for (int j = 0; j <= seg; ++j) {
            const float ph = 2 * kPi * static_cast<float>(j) / static_cast<float>(seg);
            const Vec3 radial = e1 * std::cos(ph) + e2 * std::sin(ph);
            const Vec3 n = radial * c + u * s; // the side's normal there
            m.vertices.push_back({centre + n * r, n});
        }
    }
    for (int j = 0; j < seg; ++j) {
        const std::uint32_t p0 = base + static_cast<std::uint32_t>(j), p1 = p0 + 1, q0 = p0 + static_cast<std::uint32_t>(seg + 1), q1 = q0 + 1;
        m.indices.insert(m.indices.end(), {p0, p1, q0, p1, q1, q0});
    }
}

// The order to visit the balls in: each after its parent.
std::vector<int> parentsFirst(const Armature& a) {
    const int n = static_cast<int>(a.balls.size());
    std::vector<int> order, state(static_cast<std::size_t>(n), 0);
    std::function<void(int)> visit = [&](int i) {
        if (state[static_cast<std::size_t>(i)] != 0) return;
        state[static_cast<std::size_t>(i)] = 1;
        const int p = a.balls[static_cast<std::size_t>(i)].parent;
        if (p >= 0 && p < n && state[static_cast<std::size_t>(p)] != 1) visit(p);
        state[static_cast<std::size_t>(i)] = 2;
        order.push_back(i);
    };
    for (int i = 0; i < n; ++i) visit(i);
    return order;
}

} // namespace

// ---------------------------------------------------------------------------
// Quaternions

Quat quatFromAxisAngle(const Vec3& axis, float ang) {
    const Vec3 u = normalize(axis);
    const float s = std::sin(ang * 0.5f);
    return {u.x * s, u.y * s, u.z * s, std::cos(ang * 0.5f)};
}

Quat quatMultiply(const Quat& a, const Quat& b) {
    return {a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1], a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
            a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3], a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]};
}

Quat quatFromEuler(const Vec3& d) {
    // Rz * Ry * Rx, as eulerRotation().
    return quatMultiply(quatMultiply(quatFromAxisAngle({0, 0, 1}, radians(d.z)), quatFromAxisAngle({0, 1, 0}, radians(d.y))),
                        quatFromAxisAngle({1, 0, 0}, radians(d.x)));
}

Mat4 quatMatrix(const Quat& q0) {
    const float n = std::sqrt(q0[0] * q0[0] + q0[1] * q0[1] + q0[2] * q0[2] + q0[3] * q0[3]);
    const Quat q = n > 0 ? Quat{q0[0] / n, q0[1] / n, q0[2] / n, q0[3] / n} : Quat{0, 0, 0, 1};
    const float x = q[0], y = q[1], z = q[2], w = q[3];
    Mat4 m = Mat4::identity();
    m.m[0][0] = 1 - 2 * (y * y + z * z), m.m[0][1] = 2 * (x * y + z * w), m.m[0][2] = 2 * (x * z - y * w);
    m.m[1][0] = 2 * (x * y - z * w), m.m[1][1] = 1 - 2 * (x * x + z * z), m.m[1][2] = 2 * (y * z + x * w);
    m.m[2][0] = 2 * (x * z + y * w), m.m[2][1] = 2 * (y * z - x * w), m.m[2][2] = 1 - 2 * (x * x + y * y);
    return m;
}

Vec3 quatToEuler(const Quat& q) {
    // R = Rz Ry Rx: R[2][0] (row 2, column 0) = -sin(y).
    const Mat4 r = quatMatrix(q);
    const float r20 = r.m[0][2], r21 = r.m[1][2], r22 = r.m[2][2], r10 = r.m[0][1], r00 = r.m[0][0];
    const float y = std::asin(std::clamp(-r20, -1.0f, 1.0f));
    float x = 0, z = 0;
    if (std::fabs(r20) < 0.99999f) {
        x = std::atan2(r21, r22);
        z = std::atan2(r10, r00);
    } else { // straight up or down: all of it about Z
        z = std::atan2(-r.m[1][0], r.m[1][1]);
    }
    auto clean = [](float rad) { // no -0 (nor a hair off 0) to show
        const float d = degrees(rad);
        return std::fabs(d) < 1e-4f ? 0.0f : d;
    };
    return {clean(x), clean(y), clean(z)};
}

// ---------------------------------------------------------------------------
// Params

std::vector<float> armatureParams(const Armature& a) {
    std::vector<float> p;
    p.reserve(a.balls.size() * kArmatureFloats);
    for (const ArmatureBall& b : a.balls)
        p.insert(p.end(), {b.at.x, b.at.y, b.at.z, b.radius, static_cast<float>(b.parent), b.turn[0], b.turn[1], b.turn[2], b.turn[3]});
    return p;
}

bool armatureFromParams(const std::vector<float>& p, Armature& out) {
    out.balls.clear();
    if (p.empty() || p.size() % kArmatureFloats != 0) return false;
    const int n = static_cast<int>(p.size() / kArmatureFloats);
    for (int i = 0; i < n; ++i) {
        const float* f = &p[static_cast<std::size_t>(i) * kArmatureFloats];
        ArmatureBall b;
        b.at = {f[0], f[1], f[2]};
        b.radius = f[3];
        b.parent = static_cast<int>(std::lround(f[4]));
        b.turn = {f[5], f[6], f[7], f[8]};
        if (!(b.radius > 0) || b.parent < -1 || b.parent >= n || b.parent == i) return false;
        out.balls.push_back(b);
    }
    // One root, and no loops.
    int roots = 0;
    for (const ArmatureBall& b : out.balls) roots += b.parent < 0;
    if (roots != 1) return false;
    for (int i = 0; i < n; ++i) {
        int k = i, steps = 0;
        while (k >= 0 && steps <= n) k = out.balls[static_cast<std::size_t>(k)].parent, ++steps;
        if (steps > n) return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Pose

std::vector<Mat4> armaturePose(const Armature& a) {
    std::vector<Mat4> M(a.balls.size(), Mat4::identity());
    for (const int i : parentsFirst(a)) {
        const ArmatureBall& b = a.balls[static_cast<std::size_t>(i)];
        const Mat4 own = translate(b.at) * quatMatrix(b.turn) * translate(b.at * -1.0f);
        M[static_cast<std::size_t>(i)] = b.parent >= 0 ? M[static_cast<std::size_t>(b.parent)] * own : own;
    }
    return M;
}

std::vector<Vec3> posedBalls(const Armature& a) {
    const std::vector<Mat4> M = armaturePose(a);
    std::vector<Vec3> out(a.balls.size());
    for (std::size_t i = 0; i < a.balls.size(); ++i) {
        const int p = a.balls[i].parent;
        out[i] = p >= 0 ? M[static_cast<std::size_t>(p)].transformPoint(a.balls[i].at) : a.balls[i].at;
    }
    return out;
}

MeshData armaturePreview(const Armature& a) {
    MeshData m;
    const std::vector<Vec3> at = posedBalls(a);
    for (std::size_t i = 0; i < a.balls.size(); ++i) {
        addSphere(m, at[i], a.balls[i].radius);
        const int p = a.balls[i].parent;
        if (p >= 0) addLink(m, at[static_cast<std::size_t>(p)], a.balls[static_cast<std::size_t>(p)].radius * 0.5f, at[i], a.balls[i].radius * 0.5f);
    }
    return m;
}

// ---------------------------------------------------------------------------
// Skin

void skinWeights(const Armature& a, float soft, const std::vector<Vec3>& pts, SkinData& skin) {
    skin.rest = pts, skin.bone.clear(), skin.weight.clear();
    std::vector<int> links; // the balls at the far end of each link
    for (std::size_t i = 0; i < a.balls.size(); ++i)
        if (a.balls[i].parent >= 0) links.push_back(static_cast<int>(i));
    for (const Vec3& p : pts) {
        std::array<std::int32_t, 4> bone{-1, -1, -1, -1};
        std::array<float, 4> weight{0, 0, 0, 0};
        if (links.empty()) {
            skin.bone.push_back(bone), skin.weight.push_back(weight);
            continue;
        }
        std::vector<std::pair<float, int>> d;
        d.reserve(links.size());
        float best = 1e30f;
        for (const int c : links) {
            const ArmatureBall& cb = a.balls[static_cast<std::size_t>(c)];
            const ArmatureBall& pb = a.balls[static_cast<std::size_t>(cb.parent)];
            const float dist = std::max(0.0f, linkDistance(p, pb.at, pb.radius, cb.at, cb.radius));
            d.push_back({dist, c});
            best = std::min(best, dist);
        }
        // Links much further than the nearest don't pull at all.
        std::sort(d.begin(), d.end());
        float sum = 0;
        int k = 0;
        for (const auto& [dist, c] : d) {
            if (k == 4 || dist > best + 2 * soft) break;
            const float x = (dist + soft) / soft;
            const float w = 1.0f / (x * x * x * x);
            bone[static_cast<std::size_t>(k)] = c, weight[static_cast<std::size_t>(k)] = w;
            sum += w;
            ++k;
        }
        for (int j = 0; j < k; ++j) weight[static_cast<std::size_t>(j)] /= sum;
        skin.bone.push_back(bone), skin.weight.push_back(weight);
    }
}

float skinSoftness(const Armature& a, const SkinOptions& o) {
    float rmin = 1e30f;
    AABB box;
    for (const ArmatureBall& b : a.balls) {
        rmin = std::min(rmin, b.radius);
        box.expand(b.at - Vec3{b.radius, b.radius, b.radius});
        box.expand(b.at + Vec3{b.radius, b.radius, b.radius});
    }
    if (a.balls.empty()) return 1.0f;
    const float blend = o.blend >= 0 ? o.blend : rmin * 0.25f;
    const Vec3 ext = box.max - box.min;
    float h = o.voxel > 0 ? o.voxel : rmin / 4.0f;
    h = std::max(h, (std::max({ext.x, ext.y, ext.z}) + 2 * blend) / 250.0f);
    return std::max(blend, rmin * 0.25f) + h;
}

bool armatureSkin(const Armature& a, const SkinOptions& o, MeshData& out, SkinData& skin, std::string* why) {
    auto fail = [&](const std::string& w) {
        if (why) *why = w;
        return false;
    };
    if (a.balls.empty()) return fail("The armature has no balls");
    float rmin = 1e30f;
    AABB box;
    for (const ArmatureBall& b : a.balls) {
        rmin = std::min(rmin, b.radius);
        box.expand(b.at - Vec3{b.radius, b.radius, b.radius});
        box.expand(b.at + Vec3{b.radius, b.radius, b.radius});
    }
    const float blend = o.blend >= 0 ? o.blend : rmin * 0.25f;
    const Vec3 ext = box.max - box.min;
    float h = o.voxel > 0 ? o.voxel : rmin / 4.0f;
    const float span = std::max({ext.x, ext.y, ext.z}) + 2 * blend;
    h = std::max(h, span / 250.0f); // at most about 256 voxels a side
    const float pad = blend + 3 * h;
    const Vec3 origin = box.min - Vec3{pad, pad, pad};
    const int nx = static_cast<int>(std::ceil((ext.x + 2 * pad) / h)) + 1, ny = static_cast<int>(std::ceil((ext.y + 2 * pad) / h)) + 1,
              nz = static_cast<int>(std::ceil((ext.z + 2 * pad) / h)) + 1;
    const float far = 1e9f;
    std::vector<float> f(static_cast<std::size_t>(nx) * ny * nz, far);
    // Each link (and a lone root ball) fills the voxels near it.
    struct Piece {
        Vec3 a, b;
        float ra, rb;
    };
    std::vector<Piece> pieces;
    for (std::size_t i = 0; i < a.balls.size(); ++i) {
        const ArmatureBall& c = a.balls[i];
        if (c.parent >= 0) {
            const ArmatureBall& p = a.balls[static_cast<std::size_t>(c.parent)];
            pieces.push_back({p.at, c.at, p.radius, c.radius});
        } else {
            pieces.push_back({c.at, c.at, c.radius, c.radius});
        }
    }
    for (const Piece& pc : pieces) {
        const float r = std::max(pc.ra, pc.rb) + blend + 2 * h;
        const Vec3 lo = vmin(pc.a, pc.b) - Vec3{r, r, r}, hi = vmax(pc.a, pc.b) + Vec3{r, r, r};
        const int x0 = std::max(0, static_cast<int>((lo.x - origin.x) / h)), x1 = std::min(nx - 1, static_cast<int>((hi.x - origin.x) / h) + 1);
        const int y0 = std::max(0, static_cast<int>((lo.y - origin.y) / h)), y1 = std::min(ny - 1, static_cast<int>((hi.y - origin.y) / h) + 1);
        const int z0 = std::max(0, static_cast<int>((lo.z - origin.z) / h)), z1 = std::min(nz - 1, static_cast<int>((hi.z - origin.z) / h) + 1);
        for (int x = x0; x <= x1; ++x)
            for (int y = y0; y <= y1; ++y)
                for (int z = z0; z <= z1; ++z) {
                    const Vec3 q = origin + Vec3{(static_cast<float>(x) + 0.5f) * h, (static_cast<float>(y) + 0.5f) * h, (static_cast<float>(z) + 0.5f) * h};
                    const float d = linkDistance(q, pc.a, pc.ra, pc.b, pc.rb);
                    float& cell = f[(static_cast<std::size_t>(x) * ny + y) * nz + z];
                    cell = cell >= far ? d : smoothMin(cell, d, blend);
                }
    }
    MeshData m = marchTets(f, nx, ny, nz, origin, h, true);
    if (m.indices.empty()) return fail("The skin came out empty");
    finishMarched(m, std::max(0, o.smooth));
    // Onto the surface itself: the field is known exactly, so each vertex steps along its
    // gradient to where it is zero (the voxels and the smoothing only placed it near), and the
    // gradient there is the normal.
    auto field = [&](const Vec3& q) {
        float v = far;
        for (const Piece& pc : pieces) {
            const float d = linkDistance(q, pc.a, pc.ra, pc.b, pc.rb);
            v = v >= far ? d : smoothMin(v, d, blend);
        }
        return v;
    };
    const float e = h * 0.05f;
    auto gradient = [&](const Vec3& q) {
        return Vec3{field(q + Vec3{e, 0, 0}) - field(q - Vec3{e, 0, 0}), field(q + Vec3{0, e, 0}) - field(q - Vec3{0, e, 0}),
                    field(q + Vec3{0, 0, e}) - field(q - Vec3{0, 0, e})} *
               (0.5f / e);
    };
    auto project = [&](Vertex& v) {
        for (int it = 0; it < 4; ++it) {
            const float fv = field(v.position);
            const Vec3 g = gradient(v.position);
            const float gg = dot(g, g);
            if (!(gg > 1e-12f)) break;
            const Vec3 step = g * (fv / gg);
            if (length(step) > h) break; // not near the surface after all: leave it
            v.position -= step;
            if (std::fabs(fv) < 1e-5f * h) break;
        }
        const Vec3 g = gradient(v.position);
        if (length(g) > 0) v.normal = normalize(g);
    };
    for (Vertex& v : m.vertices) project(v);
    // The smoothing can fold a sliver where the surface turns tightly: a triangle facing
    // against the field gets its corners eased to their neighbours' middle, and projected again.
    std::vector<std::vector<std::uint32_t>> around(m.vertices.size());
    for (std::size_t k = 0; k + 2 < m.indices.size(); k += 3)
        for (int corner = 0; corner < 3; ++corner) {
            const std::uint32_t p = m.indices[k + static_cast<std::size_t>(corner)], q = m.indices[k + static_cast<std::size_t>((corner + 1) % 3)];
            around[p].push_back(q), around[q].push_back(p);
        }
    for (int pass = 0; pass < 10; ++pass) {
        std::vector<char> bad(m.vertices.size(), 0);
        bool any = false;
        for (std::size_t k = 0; k + 2 < m.indices.size(); k += 3) {
            const std::uint32_t i0 = m.indices[k], i1 = m.indices[k + 1], i2 = m.indices[k + 2];
            const Vec3 n = cross(m.vertices[i1].position - m.vertices[i0].position, m.vertices[i2].position - m.vertices[i0].position);
            const Vec3 g = m.vertices[i0].normal + m.vertices[i1].normal + m.vertices[i2].normal;
            if (dot(n, g) <= 0) bad[i0] = bad[i1] = bad[i2] = 1, any = true;
        }
        if (!any) break;
        for (std::size_t i = 0; i < m.vertices.size(); ++i) {
            if (!bad[i] || around[i].empty()) continue;
            Vec3 mid{0, 0, 0};
            for (const std::uint32_t j : around[i]) mid += m.vertices[j].position;
            m.vertices[i].position = mid * (1.0f / static_cast<float>(around[i].size()));
            project(m.vertices[i]);
        }
    }
    m.computeFeatureEdges(60.0f);

    // Bone weights: each vertex follows the links nearest its surface, by inverse distance.
    std::vector<Vec3> pts;
    pts.reserve(m.vertices.size());
    for (const Vertex& v : m.vertices) pts.push_back(v.position);
    skinWeights(a, std::max(blend, rmin * 0.25f) + h, pts, skin);
    skin.base = skin.rest;
    out = std::move(m);
    return true;
}

void smoothVertexNormals(MeshData& m) {
    std::vector<Vec3> n(m.vertices.size(), Vec3{0, 0, 0});
    for (std::size_t k = 0; k + 2 < m.indices.size(); k += 3) {
        const std::uint32_t a = m.indices[k], b = m.indices[k + 1], c = m.indices[k + 2];
        const Vec3 f = cross(m.vertices[b].position - m.vertices[a].position, m.vertices[c].position - m.vertices[a].position);
        n[a] += f, n[b] += f, n[c] += f;
    }
    for (std::size_t i = 0; i < m.vertices.size(); ++i)
        if (length(n[i]) > 0) m.vertices[i].normal = normalize(n[i]);
}

namespace {

// The blended transform of one vertex: the weighted sum of its links' transforms (each link
// moves with the ball at its near end).
Mat4 blended(const SkinData& s, std::size_t i, const std::vector<Mat4>& pose, const Armature& a) {
    Mat4 sum{};
    float total = 0;
    for (int k = 0; k < 4; ++k) {
        const int b = s.bone[i][static_cast<std::size_t>(k)];
        const float w = s.weight[i][static_cast<std::size_t>(k)];
        if (b < 0 || !(w > 0) || b >= static_cast<int>(a.balls.size())) continue;
        const int p = a.balls[static_cast<std::size_t>(b)].parent;
        const Mat4& M = pose[static_cast<std::size_t>(p >= 0 ? p : b)];
        for (int c = 0; c < 4; ++c)
            for (int r = 0; r < 4; ++r) sum.m[c][r] += w * M.m[c][r];
        total += w;
    }
    return total > 0 ? sum : Mat4::identity();
}

} // namespace

namespace {

// A point near link a0-b0 (radii ra0, rb0) taken to the same place by link a1-b1 (ra1, rb1):
// as far along it, turned with it, out from it in proportion to its radius there.
Vec3 alongLink(const Vec3& x, const Vec3& a0, const Vec3& b0, float ra0, float rb0, const Vec3& a1, const Vec3& b1, float ra1, float rb1) {
    const Vec3 d0 = b0 - a0, d1 = b1 - a1;
    const float l0 = length(d0), l1 = length(d1);
    if (!(l0 > 1e-6f) || !(l1 > 1e-6f)) { // a lone ball: about its centre
        const float k = ra0 > 0 ? ra1 / ra0 : 1.0f;
        return a1 + (x - a0) * k;
    }
    const float t = std::clamp(dot(x - a0, d0) / (l0 * l0), 0.0f, 1.0f);
    const Vec3 out = x - (a0 + d0 * t); // from the link out to the point (past the ends: round them)
    const Vec3 u0 = d0 * (1.0f / l0), u1 = d1 * (1.0f / l1);
    Vec3 turned = out;
    const Vec3 ax = cross(u0, u1);
    const float s = length(ax), c = dot(u0, u1);
    if (s > 1e-7f) {
        turned = quatMatrix(quatFromAxisAngle(ax * (1.0f / s), std::atan2(s, c))).transformDir(out);
    } else if (c < 0) { // turned right round: about any line square to it
        const Vec3 any = std::fabs(u0.x) < 0.9f ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
        turned = quatMatrix(quatFromAxisAngle(normalize(cross(u0, any)), 3.14159265358979f)).transformDir(out);
    }
    const float r0 = ra0 + (rb0 - ra0) * t, r1 = ra1 + (rb1 - ra1) * t;
    return a1 + d1 * t + turned * (r0 > 0 ? r1 / r0 : 1.0f);
}

} // namespace

std::vector<Vec3> refitSkinRest(const SkinData& s, const Armature& o, const Armature& n) {
    std::vector<Vec3> out = s.rest;
    if (o.balls.size() != n.balls.size()) return out;
    auto link = [&](const Armature& a, int b, Vec3& from, Vec3& to, float& rf, float& rt) {
        const ArmatureBall& c = a.balls[static_cast<std::size_t>(b)];
        const ArmatureBall& p = c.parent >= 0 ? a.balls[static_cast<std::size_t>(c.parent)] : c;
        from = p.at, to = c.at, rf = p.radius, rt = c.radius;
    };
    int root = 0;
    for (std::size_t k = 0; k < o.balls.size(); ++k)
        if (o.balls[k].parent < 0) root = static_cast<int>(k);
    for (std::size_t i = 0; i < s.rest.size(); ++i) {
        Vec3 sum{0, 0, 0};
        float total = 0;
        for (std::size_t k = 0; k < 4 && i < s.bone.size(); ++k) {
            const int b = s.bone[i][k];
            const float w = s.weight[i][k];
            if (b < 0 || b >= static_cast<int>(o.balls.size()) || !(w > 0)) continue;
            Vec3 a0, b0, a1, b1;
            float ra0, rb0, ra1, rb1;
            link(o, b, a0, b0, ra0, rb0);
            link(n, b, a1, b1, ra1, rb1);
            sum += alongLink(s.rest[i], a0, b0, ra0, rb0, a1, b1, ra1, rb1) * w;
            total += w;
        }
        if (total > 0) {
            out[i] = sum * (1.0f / total);
        } else { // follows no link (a lone root ball): goes with the root
            const ArmatureBall &r0 = o.balls[static_cast<std::size_t>(root)], &r1 = n.balls[static_cast<std::size_t>(root)];
            out[i] = alongLink(s.rest[i], r0.at, r0.at, r0.radius, r0.radius, r1.at, r1.at, r1.radius, r1.radius);
        }
    }
    return out;
}

bool copySkinDetail(const SkinData& s, const Armature& rest, int from, int to, int mirrorAxis, std::vector<Vec3>& out, std::string* why) {
    auto fail = [&](const std::string& w) {
        if (why) *why = w;
        return false;
    };
    const int n = static_cast<int>(rest.balls.size());
    auto linked = [&](int b) { return b >= 0 && b < n && rest.balls[static_cast<std::size_t>(b)].parent >= 0; };
    if (!linked(from) || !linked(to)) return fail("Both balls need a link (the root has none)");
    if (from == to && mirrorAxis < 0) return fail("The detail would come from the ball it goes to");
    if (s.base.size() != s.rest.size() || s.bone.size() != s.rest.size() || s.weight.size() != s.rest.size())
        return fail("The skin doesn't keep its unsculpted shape (skin it again)");
    auto weightOn = [&](std::size_t i, int bone) {
        float w = 0;
        for (std::size_t q = 0; q < 4; ++q)
            if (s.bone[i][q] == bone) w += s.weight[i][q];
        return w;
    };
    // Where the detail comes from: the vertices `from`'s link weights most, as unsculpted.
    std::vector<Vec3> source;
    std::vector<std::size_t> sourceIndex;
    for (std::size_t i = 0; i < s.rest.size(); ++i)
        if (weightOn(i, from) >= 0.5f) source.push_back(s.base[i]), sourceIndex.push_back(i);
    if (source.empty()) return fail("Nothing of the skin follows the ball the detail comes from");
    const NearestPoints near(source);
    const ArmatureBall& f = rest.balls[static_cast<std::size_t>(from)];
    const ArmatureBall& fp = rest.balls[static_cast<std::size_t>(f.parent)];
    const ArmatureBall& t = rest.balls[static_cast<std::size_t>(to)];
    const ArmatureBall& tp = rest.balls[static_cast<std::size_t>(t.parent)];
    Vec3 root = rest.balls[0].at;
    for (const ArmatureBall& b : rest.balls)
        if (b.parent < 0) root = b.at;
    auto flip = [&](Vec3 v, bool point) {
        float& c = mirrorAxis == 0 ? v.x : mirrorAxis == 1 ? v.y : v.z;
        const float r = mirrorAxis == 0 ? root.x : mirrorAxis == 1 ? root.y : root.z;
        c = point ? 2 * r - c : -c;
        return v;
    };
    out = s.rest;
    std::size_t touched = 0;
    for (std::size_t i = 0; i < s.rest.size(); ++i) {
        const float w = std::min(1.0f, weightOn(i, to));
        if (!(w > 0)) continue;
        const Vec3 at = mirrorAxis >= 0 ? flip(s.base[i], true) : alongLink(s.base[i], tp.at, t.at, tp.radius, t.radius, fp.at, f.at, fp.radius, f.radius);
        const std::int64_t j = near.find(at, 1e30f);
        if (j < 0) continue;
        const std::size_t k = sourceIndex[static_cast<std::size_t>(j)];
        const Vec3 detail = s.rest[k] - s.base[k];
        Vec3 carried;
        if (mirrorAxis >= 0) {
            carried = flip(detail, false);
        } else { // turned and sized as the link is from there to here
            carried = alongLink(s.base[k] + detail, fp.at, f.at, fp.radius, f.radius, tp.at, t.at, tp.radius, t.radius) -
                      alongLink(s.base[k], fp.at, f.at, fp.radius, f.radius, tp.at, t.at, tp.radius, t.radius);
        }
        const Vec3 own = s.rest[i] - s.base[i];
        out[i] = s.base[i] + own * (1 - w) + carried * w;
        ++touched;
    }
    if (touched == 0) return fail("Nothing of the skin follows the ball the detail goes to");
    return true;
}

NearestPoints::NearestPoints(const std::vector<Vec3>& pts, float cellSize) : m_pts(pts) {
    for (const Vec3& p : pts) m_box.expand(p);
    const Vec3 ext = m_box.max - m_box.min;
    m_cell = cellSize > 0 ? cellSize : std::max({ext.x, ext.y, ext.z, 1e-3f}) / 64.0f;
    for (std::size_t i = 0; i < pts.size(); ++i) m_cells[key(pts[i])].push_back(static_cast<std::uint32_t>(i));
}

std::array<int, 3> NearestPoints::key(const Vec3& p) const {
    return {static_cast<int>(std::floor((p.x - m_box.min.x) / m_cell)), static_cast<int>(std::floor((p.y - m_box.min.y) / m_cell)),
            static_cast<int>(std::floor((p.z - m_box.min.z) / m_cell))};
}

std::int64_t NearestPoints::find(const Vec3& p, float within, float* distance) const {
    if (m_pts.empty()) return -1;
    const auto k = key(p);
    const int rings = std::max(1, static_cast<int>(std::ceil(within / m_cell)));
    float best = within;
    std::int64_t pick = -1;
    // Ring by ring outwards; done once the nearest found can't be beaten further out.
    for (int r = 0; r <= rings; ++r) {
        if (pick >= 0 && best <= (static_cast<float>(r) - 1.0f) * m_cell) break;
        for (int dx = -r; dx <= r; ++dx)
            for (int dy = -r; dy <= r; ++dy)
                for (int dz = -r; dz <= r; ++dz) {
                    if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != r) continue; // this ring's shell only
                    const auto it = m_cells.find({k[0] + dx, k[1] + dy, k[2] + dz});
                    if (it == m_cells.end()) continue;
                    for (const std::uint32_t j : it->second)
                        if (const float d = length(m_pts[j] - p); d < best) best = d, pick = j;
                }
    }
    if (distance) *distance = best;
    return pick;
}

void NearestPoints::around(const Vec3& p, float radius, std::vector<std::uint32_t>& out) const {
    out.clear();
    if (m_pts.empty() || !(radius >= 0)) return;
    const auto lo = key(p - Vec3{radius, radius, radius}), hi = key(p + Vec3{radius, radius, radius});
    for (int x = lo[0]; x <= hi[0]; ++x)
        for (int y = lo[1]; y <= hi[1]; ++y)
            for (int z = lo[2]; z <= hi[2]; ++z) {
                const auto it = m_cells.find({x, y, z});
                if (it == m_cells.end()) continue;
                for (const std::uint32_t j : it->second)
                    if (length(m_pts[j] - p) <= radius) out.push_back(j);
            }
}

namespace {

// Twice a UV triangle's signed area.
float uvCross(const Vec2& a, const Vec2& b, const Vec2& c) { return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x); }

// A new ball's skin (`fresh`: the triangles `own` marks, where nothing of the old skin is near)
// given UVs of its own: unwrapped as a mesh of its own, at the old skin's texel density (or
// smaller, to fit), in the largest square the old UV layout leaves empty, so it isn't stretched
// over the old triangles and doesn't overlap them. False (the UVs left as they were) when the
// layout leaves no room.
bool ownUvIsland(MeshData& out, const std::vector<Vec3>& fresh, const std::vector<char>& own, const SkinCarry& c) {
    // The old layout: where it is in the unit square (each triangle widened by a cell, so the
    // island keeps a gap), and its texel density.
    constexpr int G = 256;
    const float h = 1.0f / G;
    std::vector<char> used(static_cast<std::size_t>(G) * G, 0);
    double uvArea = 0, area = 0;
    for (std::size_t t = 0; t + 2 < c.indices.size(); t += 3) {
        const Vec2 a = c.uvs[t], b = c.uvs[t + 1], d = c.uvs[t + 2];
        uvArea += 0.5 * std::fabs(static_cast<double>(uvCross(a, b, d)));
        area += 0.5 * static_cast<double>(length(cross(c.bare[c.indices[t + 1]] - c.bare[c.indices[t]], c.bare[c.indices[t + 2]] - c.bare[c.indices[t]])));
        const float x0 = std::min({a.x, b.x, d.x}) - h, x1 = std::max({a.x, b.x, d.x}) + h;
        const float y0 = std::min({a.y, b.y, d.y}) - h, y1 = std::max({a.y, b.y, d.y}) + h;
        const int i0 = std::max(0, static_cast<int>(std::floor(x0 / h))), i1 = std::min(G - 1, static_cast<int>(std::floor(x1 / h)));
        const int j0 = std::max(0, static_cast<int>(std::floor(y0 / h))), j1 = std::min(G - 1, static_cast<int>(std::floor(y1 / h)));
        const float sgn = uvCross(a, b, d) < 0 ? -1.0f : 1.0f;
        for (int j = j0; j <= j1; ++j)
            for (int i = i0; i <= i1; ++i) {
                char& u = used[static_cast<std::size_t>(j) * G + static_cast<std::size_t>(i)];
                if (u) continue;
                const Vec2 p{(static_cast<float>(i) + 0.5f) * h, (static_cast<float>(j) + 0.5f) * h};
                bool hit = sgn * uvCross(a, b, p) >= 0 && sgn * uvCross(b, d, p) >= 0 && sgn * uvCross(d, a, p) >= 0;
                for (int e = 0; e < 3 && !hit; ++e) { // or within a cell of an edge
                    const Vec2 q0 = e == 0 ? a : e == 1 ? b : d, q1 = e == 0 ? b : e == 1 ? d : a;
                    const float ex = q1.x - q0.x, ey = q1.y - q0.y, len2 = ex * ex + ey * ey;
                    const float k = len2 > 0 ? std::clamp(((p.x - q0.x) * ex + (p.y - q0.y) * ey) / len2, 0.0f, 1.0f) : 0.0f;
                    const float dx = p.x - (q0.x + ex * k), dy = p.y - (q0.y + ey * k);
                    hit = dx * dx + dy * dy <= (1.5f * h) * (1.5f * h);
                }
                u = hit ? 1 : 0;
            }
    }
    // The largest empty square (each cell: the side of the empty square ending there).
    std::vector<int> side(used.size(), 0);
    int best = 0, bi = 0, bj = 0;
    for (int j = 0; j < G; ++j)
        for (int i = 0; i < G; ++i) {
            const std::size_t k = static_cast<std::size_t>(j) * G + static_cast<std::size_t>(i);
            if (used[k]) continue;
            side[k] = i == 0 || j == 0 ? 1 : 1 + std::min({side[k - 1], side[k - G], side[k - G - 1]});
            if (side[k] > best) best = side[k], bi = i, bj = j;
        }
    if (best < 6) return false;
    // The new ball's triangles as a mesh of their own, unwrapped.
    MeshData sub;
    std::vector<std::int64_t> into(fresh.size(), -1);
    std::vector<std::size_t> which;
    for (std::size_t t = 0; t < own.size(); ++t) {
        if (!own[t]) continue;
        which.push_back(t);
        for (int k = 0; k < 3; ++k) {
            const std::uint32_t v = out.indices[3 * t + static_cast<std::size_t>(k)];
            if (into[v] < 0) {
                into[v] = static_cast<std::int64_t>(sub.vertices.size());
                Vertex nv;
                nv.position = fresh[v];
                sub.vertices.push_back(nv);
            }
            sub.indices.push_back(static_cast<std::uint32_t>(into[v]));
        }
    }
    if (which.empty() || !unwrapUv(sub, UvOptions{}) || sub.uvs.size() != sub.indices.size()) return false;
    // At the old density, smaller if it must be to fit the square (less a cell each side).
    double subUv = 0, subArea = 0;
    float u0 = 1e30f, u1 = -1e30f, v0 = 1e30f, v1 = -1e30f;
    for (std::size_t t = 0; t + 2 < sub.indices.size(); t += 3) {
        subUv += 0.5 * std::fabs(static_cast<double>(uvCross(sub.uvs[t], sub.uvs[t + 1], sub.uvs[t + 2])));
        subArea += 0.5 * static_cast<double>(length(cross(sub.vertices[sub.indices[t + 1]].position - sub.vertices[sub.indices[t]].position,
                                                          sub.vertices[sub.indices[t + 2]].position - sub.vertices[sub.indices[t]].position)));
    }
    for (const Vec2& uv : sub.uvs) u0 = std::min(u0, uv.x), u1 = std::max(u1, uv.x), v0 = std::min(v0, uv.y), v1 = std::max(v1, uv.y);
    const float room = static_cast<float>(best - 2) * h, size = std::max(u1 - u0, v1 - v0);
    if (!(size > 0) || !(subUv > 0)) return false;
    float k = uvArea > 0 && area > 0 && subArea > 0 ? static_cast<float>(std::sqrt((uvArea / area) / (subUv / subArea))) : room / size;
    k = std::min(k, room / size);
    const float ox = static_cast<float>(bi - best + 2) * h, oy = static_cast<float>(bj - best + 2) * h; // the square's corner, a cell in
    for (std::size_t n = 0; n < which.size(); ++n)
        for (std::size_t q = 0; q < 3; ++q) {
            const Vec2 uv = sub.uvs[3 * n + q];
            out.uvs[3 * which[n] + q] = {ox + (uv.x - u0) * k, oy + (uv.y - v0) * k};
        }
    return true;
}

} // namespace

bool weightsAbout(const NearestPoints& near, const std::vector<Vec3>& pts, const std::vector<std::array<std::int32_t, 4>>& bone,
                  const std::vector<std::array<float, 4>>& weight, const Vec3& p, const std::vector<int>& map, std::array<std::int32_t, 4>& outBone,
                  std::array<float, 4>& outWeight) {
    float d = 0;
    const std::int64_t j = near.find(p, 1e30f, &d);
    if (j < 0) return false;
    // Within twice the nearest: on an old vertex, that one; between two, both alike.
    const float reach = 2 * d + 1e-5f;
    std::vector<std::uint32_t> about;
    if (reach < 4 * near.cell()) near.around(p, reach, about);
    else about.assign(1, static_cast<std::uint32_t>(j));
    std::map<int, float> sum;
    for (const std::uint32_t k : about) {
        if (k >= bone.size() || k >= weight.size()) continue;
        const float f = 1 - length(pts[k] - p) / reach;
        for (std::size_t q = 0; q < 4; ++q) {
            int b = bone[k][q];
            if (b < 0 || !(weight[k][q] > 0)) continue;
            if (!map.empty()) b = b < static_cast<int>(map.size()) ? map[static_cast<std::size_t>(b)] : -1;
            if (b >= 0) sum[b] += f * f * weight[k][q];
        }
    }
    std::vector<std::pair<float, int>> by;
    for (const auto& [b, w] : sum) by.push_back({w, b});
    std::stable_sort(by.begin(), by.end(), [](const auto& x, const auto& y) { return x.first > y.first; });
    if (by.size() > 4) by.resize(4);
    float total = 0;
    for (const auto& e : by) total += e.first;
    if (!(total > 0)) return false;
    outBone = {-1, -1, -1, -1}, outWeight = {0, 0, 0, 0};
    for (std::size_t q = 0; q < by.size(); ++q) outBone[q] = by[q].second, outWeight[q] = by[q].first / total;
    return true;
}

bool reskinWithDetail(const Armature& n, const SkinOptions& o, const SkinCarry& c, MeshData& out, SkinData& skin, std::string* why) {
    if (!armatureSkin(n, o, out, skin, why)) return false;
    skin.options = o;
    if (c.bare.empty() || c.detail.size() != c.bare.size()) return true;
    const NearestPoints near(c.bare);
    const float cell = near.cell();
    // Each new vertex takes its nearest old point's detail: all of it within a cell, fading to
    // none by three (a new ball's skin starts plain, without a step where the old ends), and
    // its paint.
    const std::vector<Vec3> fresh = skin.rest;
    std::vector<char> painted(out.vertices.size(), 0), apart(out.vertices.size(), 0); // apart: nothing old within a cell
    std::vector<float> fade(out.vertices.size(), 0.0f); // how much of the old skin is about it: 1 within a cell, none by three
    for (std::size_t v = 0; v < out.vertices.size(); ++v) {
        float d = 0;
        const std::int64_t j = near.find(fresh[v], 3.0f * cell, &d);
        apart[v] = j < 0 || d > cell ? 1 : 0;
        if (j < 0) continue;
        const float t = std::clamp((3.0f * cell - d) / (2.0f * cell), 0.0f, 1.0f);
        const float w = t * t * (3 - 2 * t);
        fade[v] = w;
        out.vertices[v].position += c.detail[static_cast<std::size_t>(j)] * w;
        skin.rest[v] = out.vertices[v].position;
        if (static_cast<std::size_t>(j) < c.colors.size()) out.vertices[v].color = c.colors[static_cast<std::size_t>(j)], painted[v] = 1;
    }
    // The weights it had (set by hand, perhaps), where the old skin was: a new ball's link
    // takes what the new skin's own weights give it, the rest shared as the old skin's were;
    // fading to the new skin's own where nothing of the old is near.
    if (c.bone.size() == c.bare.size() && c.weight.size() == c.bare.size()) {
        std::vector<char> added(n.balls.size(), 1);
        for (const int b : c.ballMap)
            if (b >= 0 && b < static_cast<int>(added.size())) added[static_cast<std::size_t>(b)] = 0;
        for (std::size_t v = 0; v < out.vertices.size() && v < skin.bone.size(); ++v) {
            if (!(fade[v] > 0)) continue;
            std::array<std::int32_t, 4> ob;
            std::array<float, 4> ow;
            if (!weightsAbout(near, c.bare, c.bone, c.weight, fresh[v], c.ballMap, ob, ow)) continue;
            float toNew = 0;
            for (std::size_t q = 0; q < 4; ++q)
                if (skin.bone[v][q] >= 0 && added[static_cast<std::size_t>(skin.bone[v][q])]) toNew += skin.weight[v][q];
            std::map<int, float> sum;
            for (std::size_t q = 0; q < 4; ++q) {
                if (ob[q] >= 0) sum[ob[q]] += fade[v] * (1 - toNew) * ow[q];
                const int b = skin.bone[v][q];
                if (b < 0) continue;
                sum[b] += (added[static_cast<std::size_t>(b)] ? 1.0f : 1 - fade[v]) * skin.weight[v][q];
            }
            std::vector<std::pair<float, int>> by;
            for (const auto& [b, w] : sum) by.push_back({w, b});
            std::stable_sort(by.begin(), by.end(), [](const auto& x, const auto& y) { return x.first > y.first; });
            if (by.size() > 4) by.resize(4);
            float total = 0;
            for (const auto& e : by) total += e.first;
            if (!(total > 0)) continue;
            skin.bone[v] = {-1, -1, -1, -1}, skin.weight[v] = {0, 0, 0, 0};
            for (std::size_t q = 0; q < by.size(); ++q) skin.bone[v][q] = by[q].second, skin.weight[v][q] = by[q].first / total;
        }
    }
    // Where nothing of the old skin is near (a new ball), the paint flows on from the painted
    // vertices next to it, along the skin: a new ball takes the colour of where it joins.
    if (!c.colors.empty()) {
        std::vector<std::vector<std::uint32_t>> around(out.vertices.size());
        for (std::size_t k = 0; k + 2 < out.indices.size(); k += 3)
            for (int e = 0; e < 3; ++e) {
                const std::uint32_t p = out.indices[k + static_cast<std::size_t>(e)], q = out.indices[k + static_cast<std::size_t>((e + 1) % 3)];
                around[p].push_back(q), around[q].push_back(p);
            }
        std::vector<std::uint32_t> front;
        for (std::size_t v = 0; v < painted.size(); ++v)
            if (painted[v]) front.push_back(static_cast<std::uint32_t>(v));
        while (!front.empty()) {
            std::vector<std::uint32_t> next;
            for (const std::uint32_t v : front)
                for (const std::uint32_t w : around[v])
                    if (!painted[w]) painted[w] = 1, out.vertices[w].color = out.vertices[v].color, next.push_back(w);
            front = std::move(next);
        }
    }
    // Polygroups and UVs, triangle by triangle: from the old triangle nearest the new one's
    // middle; its UVs by where each corner falls on that one triangle (so a new triangle never
    // straddles a seam).
    const bool groups = c.groups.size() * 3 == c.indices.size() && !c.groups.empty();
    const bool uvs = c.uvs.size() == c.indices.size() && !c.uvs.empty();
    if (groups || uvs) {
        std::vector<Vec3> mids;
        for (std::size_t t = 0; t + 2 < c.indices.size(); t += 3)
            mids.push_back((c.bare[c.indices[t]] + c.bare[c.indices[t + 1]] + c.bare[c.indices[t + 2]]) * (1.0f / 3.0f));
        const NearestPoints nearTri(mids, cell);
        const std::size_t tris = out.indices.size() / 3;
        if (groups) out.groups.assign(tris, 0);
        if (uvs) out.uvs.assign(out.indices.size(), Vec2{0, 0});
        for (std::size_t t = 0; t < tris; ++t) {
            const Vec3 a = fresh[out.indices[3 * t]], b = fresh[out.indices[3 * t + 1]], cc = fresh[out.indices[3 * t + 2]];
            const std::int64_t o3 = nearTri.find((a + b + cc) * (1.0f / 3.0f), 64.0f * cell);
            if (o3 < 0) continue;
            const std::size_t ot = static_cast<std::size_t>(o3);
            if (groups) out.groups[t] = c.groups[ot];
            if (uvs) {
                const Vec3 p0 = c.bare[c.indices[3 * ot]], p1 = c.bare[c.indices[3 * ot + 1]], p2 = c.bare[c.indices[3 * ot + 2]];
                const Vec3 e0 = p1 - p0, e1 = p2 - p0;
                const float d00 = dot(e0, e0), d01 = dot(e0, e1), d11 = dot(e1, e1), den = d00 * d11 - d01 * d01;
                for (int k = 0; k < 3; ++k) {
                    const Vec3 q = fresh[out.indices[3 * t + static_cast<std::size_t>(k)]] - p0;
                    const float d20 = dot(q, e0), d21 = dot(q, e1);
                    const float bv = std::fabs(den) > 1e-20f ? (d11 * d20 - d01 * d21) / den : 0.0f;
                    const float bw = std::fabs(den) > 1e-20f ? (d00 * d21 - d01 * d20) / den : 0.0f;
                    const Vec2 u0 = c.uvs[3 * ot], u1 = c.uvs[3 * ot + 1], u2 = c.uvs[3 * ot + 2];
                    const float bu = 1 - bv - bw;
                    out.uvs[3 * t + static_cast<std::size_t>(k)] = {bu * u0.x + bv * u1.x + bw * u2.x, bu * u0.y + bv * u1.y + bw * u2.y};
                }
            }
        }
        // A new ball's skin, away from the old: an island of its own rather than stretched
        // over the old triangle nearest it.
        if (uvs) {
            std::vector<char> own(tris, 0);
            bool any = false;
            for (std::size_t t = 0; t < tris; ++t)
                for (int k = 0; k < 3; ++k)
                    if (apart[out.indices[3 * t + static_cast<std::size_t>(k)]]) own[t] = 1, any = true;
            if (any) ownUvIsland(out, fresh, own, c);
        }
    }
    smoothVertexNormals(out);
    out.computeFeatureEdges(60.0f);
    return true;
}

std::vector<Vec3> poseSkin(const SkinData& s, const Armature& a) {
    const std::vector<Mat4> pose = armaturePose(a);
    std::vector<Vec3> out(s.rest.size());
    for (std::size_t i = 0; i < s.rest.size(); ++i)
        out[i] = i < s.bone.size() ? blended(s, i, pose, a).transformPoint(s.rest[i]) : s.rest[i];
    return out;
}

std::vector<Vec3> unposeSkin(const SkinData& s, const Armature& a, const std::vector<Vec3>& posed) {
    const std::vector<Mat4> pose = armaturePose(a);
    std::vector<Vec3> out(posed.size());
    for (std::size_t i = 0; i < posed.size(); ++i)
        out[i] = i < s.bone.size() ? inverse(blended(s, i, pose, a)).transformPoint(posed[i]) : posed[i];
    return out;
}

} // namespace sicdm

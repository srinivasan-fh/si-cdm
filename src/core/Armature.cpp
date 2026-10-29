// Armatures: a tree of balls joined by tapered links, posed by turning balls, skinned into
// one smooth mesh whose vertices follow the links near them.

#include "core/Armature.hpp"

#include "core/Wearable.hpp"

#include <algorithm>
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
        for (int e = 0; e < 3; ++e) {
            const std::uint32_t a = m.indices[k + static_cast<std::size_t>(e)], b = m.indices[k + static_cast<std::size_t>((e + 1) % 3)];
            around[a].push_back(b), around[b].push_back(a);
        }
    for (int pass = 0; pass < 10; ++pass) {
        std::vector<char> bad(m.vertices.size(), 0);
        bool any = false;
        for (std::size_t k = 0; k + 2 < m.indices.size(); k += 3) {
            const std::uint32_t a = m.indices[k], b = m.indices[k + 1], c = m.indices[k + 2];
            const Vec3 n = cross(m.vertices[b].position - m.vertices[a].position, m.vertices[c].position - m.vertices[a].position);
            const Vec3 g = m.vertices[a].normal + m.vertices[b].normal + m.vertices[c].normal;
            if (dot(n, g) <= 0) bad[a] = bad[b] = bad[c] = 1, any = true;
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
    skin.rest.clear(), skin.bone.clear(), skin.weight.clear();
    skin.rest.reserve(m.vertices.size());
    std::vector<int> links; // the balls at the far end of each link
    for (std::size_t i = 0; i < a.balls.size(); ++i)
        if (a.balls[i].parent >= 0) links.push_back(static_cast<int>(i));
    const float soft = std::max(blend, rmin * 0.25f) + h;
    for (const Vertex& v : m.vertices) {
        skin.rest.push_back(v.position);
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
            const float dist = std::max(0.0f, linkDistance(v.position, pb.at, pb.radius, cb.at, cb.radius));
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

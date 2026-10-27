// Arm kinematics, collisions, motion planning and trajectory optimisation on a robot rig
// (see Kinematics.hpp). Everything is in the world frame at rest, in metres and radians.

#include "core/Kinematics.hpp"

#include "core/Spatial.hpp"

#include "core/Motion.hpp"
#include "core/Physics.hpp"
#include "core/Scene.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>

namespace sicdm {

namespace {

constexpr double kPiD = 3.14159265358979323846;

using namespace spatial;

// Deterministic random numbers (splitmix64), the same on every platform.
struct Rng {
    std::uint64_t s;
    explicit Rng(std::uint64_t seed) : s(seed * 0x9E3779B97F4A7C15ull + 0x632BE59BD9B4E019ull) {}
    double next() {
        std::uint64_t z = (s += 0x9E3779B97F4A7C15ull);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        z ^= z >> 31;
        return static_cast<double>(z >> 11) * (1.0 / 9007199254740992.0);
    }
    double between(double lo, double hi) { return lo + (hi - lo) * next(); }
};

// Each joint's world axis and a point on it, and every link's pose, at joint values q
// (mimics already applied).
struct Frames {
    std::vector<LinkPose> link;
    std::vector<V3d> axis, origin;
};

Frames framesAt(const RobotRig& rig, const std::vector<double>& q) {
    Frames f;
    f.link.assign(rig.links.size(), LinkPose{});
    f.axis.assign(rig.joints.size(), {0, 0, 1});
    f.origin.assign(rig.joints.size(), {0, 0, 0});
    for (std::size_t j = 0; j < rig.joints.size(); ++j) {
        const RigJoint& J = rig.joints[j];
        const LinkPose& P = f.link[static_cast<std::size_t>(J.parent)];
        const V3d axis = mv(P.R, J.axis);
        const V3d origin = add(mv(P.R, J.origin), P.t);
        f.axis[j] = axis, f.origin[j] = origin;
        LinkPose& C = f.link[static_cast<std::size_t>(J.child)];
        const double v = j < q.size() ? q[j] : 0.0;
        if (J.type == RigJointType::Prismatic) {
            C.R = P.R;
            C.t = add(P.t, mul(axis, v));
        } else {
            const M3 rot = rotation(axis, v);
            C.R = mm(rot, P.R);
            C.t = add(mv(rot, sub(P.t, origin)), origin);
        }
    }
    return f;
}

// Joints between the base and a link (all of them, mimics included), base first.
std::vector<std::size_t> pathTo(const RobotRig& rig, int link) {
    std::vector<std::size_t> out;
    int l = link;
    while (l > 0 && static_cast<std::size_t>(l) < rig.links.size()) {
        const int j = rig.links[static_cast<std::size_t>(l)].parentJoint;
        if (j < 0) break;
        out.push_back(static_cast<std::size_t>(j));
        l = rig.joints[static_cast<std::size_t>(j)].parent;
    }
    std::reverse(out.begin(), out.end());
    return out;
}

// Solves the small dense system A x = b (n x n, row major) in place; false when singular.
bool solveDense(std::vector<double> A, std::vector<double>& b, std::size_t n) {
    for (std::size_t c = 0; c < n; ++c) {
        std::size_t p = c;
        for (std::size_t r = c + 1; r < n; ++r)
            if (std::fabs(A[r * n + c]) > std::fabs(A[p * n + c])) p = r;
        if (std::fabs(A[p * n + c]) < 1e-14) return false;
        if (p != c) {
            for (std::size_t k = 0; k < n; ++k) std::swap(A[p * n + k], A[c * n + k]);
            std::swap(b[p], b[c]);
        }
        for (std::size_t r = c + 1; r < n; ++r) {
            const double f = A[r * n + c] / A[c * n + c];
            if (f == 0) continue;
            for (std::size_t k = c; k < n; ++k) A[r * n + k] -= f * A[c * n + k];
            b[r] -= f * b[c];
        }
    }
    for (std::size_t i = n; i-- > 0;) {
        double s = b[i];
        for (std::size_t k = i + 1; k < n; ++k) s -= A[i * n + k] * b[k];
        b[i] = s / A[i * n + i];
    }
    return true;
}

// Separating-axis test of two oriented boxes (Collide.hpp).
bool boxesOverlap(const V3d& ca, const M3& Ra, const V3d& ha, const V3d& cb, const M3& Rb, const V3d& hb) { return obbOverlap(ca, Ra, ha, cb, Rb, hb); }

const M3 kIdentity = identity3();

double jointScale(const RobotRig& rig, std::size_t j) {
    // Distances in joint space: a metre of travel counts as 4 rad.
    return rig.joints[j].type == RigJointType::Prismatic ? 4.0 : 1.0;
}

// Minimum-jerk profile s(u) on [0, 1] and its first two derivatives.
void minJerk(double u, double& s, double& ds, double& dds) {
    const double u2 = u * u, u3 = u2 * u;
    s = u3 * (10 - 15 * u + 6 * u2);
    ds = 30 * u2 - 60 * u3 + 30 * u2 * u2;
    dds = 60 * u - 180 * u2 + 120 * u3;
}

} // namespace

// --- kinematics --------------------------------------------------------------------------------

RobotKinematics::RobotKinematics(const RobotRig& rig) : m_rig(rig) {
    m_leader.assign(m_rig.joints.size(), -1);
    for (std::size_t j = 0; j < m_rig.joints.size(); ++j) {
        const RigJoint& J = m_rig.joints[j];
        if (J.mimic.empty()) continue;
        for (std::size_t k = 0; k < m_rig.joints.size(); ++k)
            if (k != j && m_rig.joints[k].name == J.mimic) m_leader[j] = static_cast<int>(k);
    }
}

std::vector<double> RobotKinematics::withMimics(std::vector<double> q) const {
    q.resize(m_rig.joints.size(), 0.0);
    // Twice, so a mimic of a mimic listed before its leader settles too.
    for (int pass = 0; pass < 2; ++pass)
        for (std::size_t j = 0; j < q.size(); ++j)
            if (m_leader[j] >= 0) q[j] = m_rig.joints[j].mimicRatio * q[static_cast<std::size_t>(m_leader[j])];
    return q;
}

std::vector<LinkPose> RobotKinematics::forward(const std::vector<double>& q) const { return framesAt(m_rig, withMimics(q)).link; }

std::vector<std::size_t> RobotKinematics::chain(int link) const {
    std::vector<std::size_t> out;
    for (std::size_t j : pathTo(m_rig, link)) {
        std::size_t v = j;
        // A coupled joint on the way moves with its leader: the leader is the variable.
        for (int hop = 0; hop < 4 && m_leader[v] >= 0; ++hop) v = static_cast<std::size_t>(m_leader[v]);
        if (m_leader[v] >= 0) continue;
        if (std::find(out.begin(), out.end(), v) == out.end()) out.push_back(v);
    }
    return out;
}

double RobotKinematics::lower(std::size_t joint) const {
    const RigJoint& J = m_rig.joints[joint];
    return J.type == RigJointType::Continuous ? -2 * kPiD : J.lo;
}

double RobotKinematics::upper(std::size_t joint) const {
    const RigJoint& J = m_rig.joints[joint];
    return J.type == RigJointType::Continuous ? 2 * kPiD : J.hi;
}

// --- collisions --------------------------------------------------------------------------------

CollisionModel::CollisionModel(const Scene& scene, const RobotRig& rig, float t) {
    const auto& bodies = scene.bodies();
    const std::size_t nl = rig.links.size();
    m_names.resize(nl);
    m_links.assign(nl, {});
    std::vector<char> inRobot(bodies.size(), 0);
    // A part's box, a millimetre in from its faces so parts that only touch don't collide.
    auto boxOf = [](const AABB& b) {
        Box x;
        for (std::size_t k = 0; k < 3; ++k) {
            const double lo = (k == 0 ? b.min.x : k == 1 ? b.min.y : b.min.z) * 1e-3, hi = (k == 0 ? b.max.x : k == 1 ? b.max.y : b.max.z) * 1e-3;
            x.centre[k] = 0.5 * (lo + hi);
            x.half[k] = std::max(0.5 * (hi - lo) - 0.001, 0.0005);
        }
        return x;
    };
    for (std::size_t l = 0; l < nl; ++l) {
        m_names[l] = rig.links[l].name;
        for (std::size_t b : rig.links[l].bodies) {
            if (b >= bodies.size()) continue;
            inRobot[b] = 1;
            if (!bodies[b].visible || bodies[b].mesh.indices.empty()) continue;
            const AABB bb = bodies[b].bounds();
            if (!bb.valid()) continue;
            Box x = boxOf(bb);
            x.shape = std::make_shared<const CollisionShape>(bodies[b].mesh, bodies[b].offset);
            m_links[l].push_back(std::move(x));
        }
    }
    // Touching at rest: the boxes overlap and so do the parts (an L-shaped link's box may
    // reach over a part it never touches; that pair stays checked).
    auto overlapAtRest = [&](const Box& a, const Box& b, const Rigid& bAt = Rigid{}) {
        return boxesOverlap(a.centre, kIdentity, a.half, b.centre, kIdentity, b.half) && (!a.shape || !b.shape || shapesCollide(*a.shape, Rigid{}, *b.shape, bAt));
    };
    // Self collisions: not between a link and the next one with parts up or down the tree,
    // nor between links that already touch at rest (they meet at a joint).
    auto partParent = [&](std::size_t l) -> int {
        int p = static_cast<int>(l);
        while (p > 0) {
            const int j = rig.links[static_cast<std::size_t>(p)].parentJoint;
            if (j < 0) return 0;
            p = rig.joints[static_cast<std::size_t>(j)].parent;
            if (!m_links[static_cast<std::size_t>(p)].empty()) return p;
        }
        return p;
    };
    m_skip.assign(nl, std::vector<char>(nl, 0));
    for (std::size_t a = 0; a < nl; ++a)
        for (std::size_t b = a + 1; b < nl; ++b) {
            bool skip = m_links[a].empty() || m_links[b].empty() || partParent(a) == static_cast<int>(b) || partParent(b) == static_cast<int>(a);
            for (const Box& x : m_links[a])
                for (const Box& y : m_links[b]) skip = skip || overlapAtRest(x, y);
            m_skip[a][b] = m_skip[b][a] = skip ? 1 : 0;
        }
    // The scene around the robot, where it is at time t.
    const std::vector<Mat4> tf = simulationTransforms(scene, std::max(t, 0.0f));
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        const Body& body = bodies[i];
        if (inRobot[i] || !body.visible || body.mesh.indices.empty() || body.name == kTerrainBody) continue;
        const AABB bb = body.bounds();
        if (!bb.valid()) continue;
        AABB w;
        for (int c = 0; c < 8; ++c) {
            const Vec3 p{(c & 1) ? bb.max.x : bb.min.x, (c & 2) ? bb.max.y : bb.min.y, (c & 4) ? bb.max.z : bb.min.z};
            w.expand(i < tf.size() ? tf[i].transformPoint(p) : p);
        }
        Obstacle o;
        o.lo = {w.min.x * 1e-3 + 0.001, w.min.y * 1e-3 + 0.001, w.min.z * 1e-3 + 0.001};
        o.hi = {w.max.x * 1e-3 - 0.001, w.max.y * 1e-3 - 0.001, w.max.z * 1e-3 - 0.001};
        for (std::size_t k = 0; k < 3; ++k)
            if (o.hi[k] < o.lo[k]) o.lo[k] = o.hi[k] = 0.5 * (o.lo[k] + o.hi[k]);
        o.name = body.name;
        o.shape = std::make_shared<const CollisionShape>(body.mesh, body.offset);
        if (i < tf.size()) o.at = rigidOf(tf[i]);
        m_env.push_back(std::move(o));
    }
    // Obstacles a link already touches at rest (the table under the arm) stay out of its checks,
    // and so does the ground for links on it; the base never moves.
    m_skipEnv.assign(nl, std::vector<char>(m_env.size(), 0));
    m_skipFloor.assign(nl, 0);
    bool terrainDrawn = false; // as the robot engines see the ground
    for (const Body& b : bodies) terrainDrawn = terrainDrawn || b.name == kTerrainBody;
    if (scene.terrain().active() && terrainDrawn) m_terrain = std::make_shared<const TerrainSettings>(scene.terrain());
    for (std::size_t l = 0; l < nl; ++l) {
        for (std::size_t o = 0; o < m_env.size(); ++o) {
            bool skip = l == 0;
            Box ob;
            for (std::size_t k = 0; k < 3; ++k) ob.centre[k] = 0.5 * (m_env[o].lo[k] + m_env[o].hi[k]), ob.half[k] = 0.5 * (m_env[o].hi[k] - m_env[o].lo[k]);
            ob.shape = m_env[o].shape;
            for (const Box& x : m_links[l]) skip = skip || overlapAtRest(x, ob, m_env[o].at);
            m_skipEnv[l][o] = skip ? 1 : 0;
        }
        bool onGround = l == 0;
        for (const Box& x : m_links[l]) {
            const double ground = m_terrain ? terrainHeight(*m_terrain, x.centre[0] * 1e3, x.centre[1] * 1e3) * 1e-3 : 0.0;
            onGround = onGround || x.centre[2] - x.half[2] < ground + 0.002;
        }
        m_skipFloor[l] = onGround ? 1 : 0;
    }
}

void CollisionModel::onlyMoving(const RobotKinematics& kin, const std::vector<std::size_t>& active) {
    const RobotRig& rig = kin.rig();
    const std::size_t nl = std::min(rig.links.size(), m_links.size());
    // The joints that move: the active ones and those coupled to them.
    std::vector<char> moving(rig.joints.size(), 0);
    for (std::size_t j : active)
        if (j < moving.size()) moving[j] = 1;
    std::vector<double> probe(rig.joints.size(), 0.0);
    for (std::size_t j : active)
        if (j < probe.size()) probe[j] = 1.0;
    probe = kin.withMimics(probe);
    for (std::size_t j = 0; j < probe.size(); ++j)
        if (probe[j] != 0.0) moving[j] = 1;
    // Per link, the joints from the base out to it.
    std::vector<std::vector<std::size_t>> path(nl);
    for (std::size_t l = 0; l < nl; ++l) path[l] = pathTo(rig, static_cast<int>(l));
    auto movesBetween = [&](std::size_t a, std::size_t b) {
        // Joints on one path and not the other lie between the two links.
        for (std::size_t j : path[a])
            if (moving[j] && std::find(path[b].begin(), path[b].end(), j) == path[b].end()) return true;
        for (std::size_t j : path[b])
            if (moving[j] && std::find(path[a].begin(), path[a].end(), j) == path[a].end()) return true;
        return false;
    };
    for (std::size_t a = 0; a < nl; ++a) {
        for (std::size_t b = a + 1; b < nl; ++b)
            if (!movesBetween(a, b)) m_skip[a][b] = m_skip[b][a] = 1;
        if (!movesBetween(a, 0)) {
            std::fill(m_skipEnv[a].begin(), m_skipEnv[a].end(), 1);
            m_skipFloor[a] = 1;
        }
    }
}

int CollisionModel::allowTouching(const std::vector<LinkPose>& poses) {
    int allowed = 0;
    std::string what;
    // One pair at a time: each collides() call names the first it finds.
    for (int guard = 0; guard < 10000 && collides(poses, &what); ++guard) {
        const std::size_t nl = std::min(poses.size(), m_links.size());
        bool found = false;
        for (std::size_t a = 0; a < nl && !found; ++a) {
            if (what.rfind(m_names[a] + " hits ", 0) != 0) continue;
            const std::string other = what.substr(m_names[a].size() + 6);
            for (std::size_t b = 0; b < nl && !found; ++b)
                if (b != a && other == m_names[b] && !m_skip[a][b]) m_skip[a][b] = m_skip[b][a] = 1, found = true;
            for (std::size_t o = 0; o < m_env.size() && !found; ++o)
                if (other == m_env[o].name && !m_skipEnv[a][o]) m_skipEnv[a][o] = 1, found = true;
            if (!found && other == "the ground" && !m_skipFloor[a]) m_skipFloor[a] = 1, found = true;
        }
        if (!found) break;
        ++allowed;
    }
    return allowed;
}

bool CollisionModel::collides(const std::vector<LinkPose>& poses, std::string* what) const {
    const std::size_t nl = std::min(poses.size(), m_links.size());
    // Every part's box where the link has taken it.
    std::vector<std::vector<Box>> placed(nl);
    for (std::size_t l = 0; l < nl; ++l)
        for (const Box& b : m_links[l]) placed[l].push_back({poses[l].apply(b.centre), b.half, b.shape});
    // Where a link has taken its parts' exact shapes (mm).
    auto rigid = [&](std::size_t l) {
        Rigid r;
        r.R = poses[l].R;
        for (std::size_t k = 0; k < 3; ++k) r.t[k] = poses[l].t[k] * 1e3;
        return r;
    };
    // The boxes overlap: do the parts themselves?
    auto parts = [&](const Box& x, std::size_t la, const Box& y, std::size_t lb) {
        return !x.shape || !y.shape || shapesCollide(*x.shape, rigid(la), *y.shape, rigid(lb));
    };
    for (std::size_t a = 0; a < nl; ++a)
        for (std::size_t b = a + 1; b < nl; ++b) {
            if (m_skip[a][b]) continue;
            for (const Box& x : placed[a])
                for (const Box& y : placed[b])
                    if (boxesOverlap(x.centre, poses[a].R, x.half, y.centre, poses[b].R, y.half) && parts(x, a, y, b)) {
                        if (what) *what = m_names[a] + " hits " + m_names[b];
                        return true;
                    }
        }
    for (std::size_t l = 1; l < nl; ++l) {
        for (std::size_t o = 0; o < m_env.size(); ++o) {
            if (m_skipEnv[l][o]) continue;
            const Obstacle& ob = m_env[o];
            const V3d c{0.5 * (ob.lo[0] + ob.hi[0]), 0.5 * (ob.lo[1] + ob.hi[1]), 0.5 * (ob.lo[2] + ob.hi[2])};
            const V3d h{0.5 * (ob.hi[0] - ob.lo[0]), 0.5 * (ob.hi[1] - ob.lo[1]), 0.5 * (ob.hi[2] - ob.lo[2])};
            for (const Box& x : placed[l])
                if (boxesOverlap(x.centre, poses[l].R, x.half, c, kIdentity, h) && (!x.shape || !ob.shape || shapesCollide(*x.shape, rigid(l), *ob.shape, ob.at))) {
                    if (what) *what = m_names[l] + " hits " + ob.name;
                    return true;
                }
        }
        if (m_skipFloor[l]) continue;
        const M3& R = poses[l].R;
        for (const Box& x : placed[l]) {
            bool boxBelow = false;
            for (int c = 0; c < 8 && !boxBelow; ++c) {
                const double sx = (c & 1) ? 1 : -1, sy = (c & 2) ? 1 : -1, sz = (c & 4) ? 1 : -1;
                const V3d local{sx * x.half[0], sy * x.half[1], sz * x.half[2]};
                const V3d p = add(x.centre, mv(R, local));
                const double ground = m_terrain ? terrainHeight(*m_terrain, p[0] * 1e3, p[1] * 1e3) * 1e-3 : 0.0;
                boxBelow = p[2] < ground;
            }
            if (!boxBelow) continue;
            // The part itself: some point more than a millimetre into the ground.
            const auto ground = [&](double gx, double gy) { return m_terrain ? static_cast<double>(terrainHeight(*m_terrain, gx, gy)) : 0.0; };
            if (!x.shape || x.shape->below(rigid(l), ground, 1.0)) {
                if (what) *what = m_names[l] + " hits the ground";
                return true;
            }
        }
    }
    return false;
}

// --- inverse kinematics ------------------------------------------------------------------------

IkResult solveIk(const RobotKinematics& kin, const std::vector<double>& start, const IkTarget& target, const CollisionModel* collisions, int restarts) {
    const RobotRig& rig = kin.rig();
    IkResult res;
    int link = target.link;
    if (link < 0) {
        const std::vector<int> ee = rig.endEffectors();
        link = ee.empty() ? static_cast<int>(rig.links.size()) - 1 : ee.front();
    }
    if (link <= 0 || static_cast<std::size_t>(link) >= rig.links.size()) {
        res.why = "the tool must be on a moving link";
        return res;
    }
    const std::vector<std::size_t> vars = kin.chain(link);
    if (vars.empty()) {
        res.why = "no joint moves " + rig.links[static_cast<std::size_t>(link)].name;
        return res;
    }
    const std::vector<std::size_t> path = pathTo(rig, link);
    const std::size_t n = vars.size();
    const bool orient = target.approach.has_value();
    const std::size_t m = orient ? 6 : 3;
    constexpr double kOrientWeight = 0.1; // m per unit of direction error
    std::vector<int> varOf(rig.joints.size(), -1);
    for (std::size_t i = 0; i < n; ++i) varOf[vars[i]] = static_cast<int>(i);
    std::vector<int> leaderOf(rig.joints.size(), -1);
    for (std::size_t j = 0; j < rig.joints.size(); ++j)
        if (!rig.joints[j].mimic.empty())
            for (std::size_t k = 0; k < rig.joints.size(); ++k)
                if (k != j && rig.joints[k].name == rig.joints[j].mimic) leaderOf[j] = static_cast<int>(k);
    V3d dir{0, 0, 1}, axisAtRest{0, 0, 1};
    if (orient) {
        dir = *target.approach;
        axisAtRest = target.approachAt;
        const double a = norm(dir), b = norm(axisAtRest);
        if (a < 1e-9 || b < 1e-9) {
            res.why = "the approach direction is zero";
            return res;
        }
        dir = mul(dir, 1 / a), axisAtRest = mul(axisAtRest, 1 / b);
    }

    struct Attempt {
        std::vector<double> q;
        double pos = 1e300, ang = 1e300;
    };
    // Damped least squares from q, within the limits.
    auto descend = [&](std::vector<double> q) {
        Attempt best;
        std::vector<double> J(m * n), e(m), A(m * m);
        for (int it = 0; it < 300; ++it) {
            const std::vector<double> qm = kin.withMimics(q);
            const Frames f = framesAt(rig, qm);
            const LinkPose& P = f.link[static_cast<std::size_t>(link)];
            const V3d p = P.apply(target.point);
            const V3d ep = sub(target.target, p);
            V3d a{0, 0, 0}, ea{0, 0, 0};
            if (orient) {
                a = mv(P.R, axisAtRest);
                ea = sub(dir, a);
            }
            const double pos = norm(ep), ang = orient ? norm(ea) : 0.0;
            if (pos + kOrientWeight * ang < best.pos + kOrientWeight * best.ang) best = {q, pos, ang};
            if (pos <= target.tolerance && ang <= 0.01) break;
            std::fill(J.begin(), J.end(), 0.0);
            for (std::size_t j : path) {
                // The variable this joint moves with, and how much.
                int v = varOf[j];
                double ratio = 1;
                if (v < 0 && leaderOf[j] >= 0) {
                    ratio = rig.joints[j].mimicRatio;
                    std::size_t l = static_cast<std::size_t>(leaderOf[j]);
                    for (int hop = 0; hop < 4 && varOf[l] < 0 && leaderOf[l] >= 0; ++hop) {
                        ratio *= rig.joints[l].mimicRatio;
                        l = static_cast<std::size_t>(leaderOf[l]);
                    }
                    v = varOf[l];
                }
                if (v < 0) continue;
                const std::size_t c = static_cast<std::size_t>(v);
                V3d dp, da{0, 0, 0};
                if (rig.joints[j].type == RigJointType::Prismatic) {
                    dp = f.axis[j];
                } else {
                    dp = cross(f.axis[j], sub(p, f.origin[j]));
                    da = cross(f.axis[j], a);
                }
                for (std::size_t r = 0; r < 3; ++r) {
                    J[r * n + c] += ratio * dp[r];
                    if (orient) J[(3 + r) * n + c] += ratio * kOrientWeight * da[r];
                }
            }
            for (std::size_t r = 0; r < 3; ++r) {
                e[r] = ep[r];
                if (orient) e[3 + r] = kOrientWeight * ea[r];
            }
            // (J J^T + lambda^2) y = e, dq = J^T y; less damping as the target nears.
            const double err = std::sqrt(pos * pos + kOrientWeight * kOrientWeight * ang * ang);
            const double lambda = std::clamp(0.5 * err, 0.005, 0.05);
            for (std::size_t r = 0; r < m; ++r)
                for (std::size_t s = 0; s < m; ++s) {
                    double sum = r == s ? lambda * lambda : 0.0;
                    for (std::size_t c = 0; c < n; ++c) sum += J[r * n + c] * J[s * n + c];
                    A[r * m + s] = sum;
                }
            std::vector<double> y = e;
            if (!solveDense(A, y, m)) break;
            std::vector<double> dq(n, 0.0);
            double scale = 1;
            for (std::size_t c = 0; c < n; ++c) {
                for (std::size_t r = 0; r < m; ++r) dq[c] += J[r * n + c] * y[r];
                const double cap = rig.joints[vars[c]].type == RigJointType::Prismatic ? 0.05 : 0.2;
                if (std::fabs(dq[c]) * scale > cap) scale = cap / std::fabs(dq[c]);
            }
            for (std::size_t c = 0; c < n; ++c) {
                const std::size_t j = vars[c];
                q[j] = std::clamp(q[j] + dq[c] * scale, kin.lower(j), kin.upper(j));
            }
        }
        return best;
    };

    std::vector<double> q0 = start;
    q0.resize(rig.joints.size(), 0.0);
    Rng rng(0x1C0FFEEu);
    Attempt closest;          // the least error
    Attempt chosen;           // the collision-free solution nearest the start
    double chosenDist = 1e300;
    int found = 0;
    bool anyReached = false;
    std::string blocked;
    for (int k = 0; k <= std::max(restarts, 0); ++k) {
        std::vector<double> guess = q0;
        if (k > 0)
            for (std::size_t j : vars) {
                const double lo = std::max(kin.lower(j), -kPiD), hi = std::min(kin.upper(j), kPiD);
                guess[j] = hi > lo ? rng.between(lo, hi) : lo;
            }
        Attempt at = descend(guess);
        if (at.pos + kOrientWeight * at.ang < closest.pos + kOrientWeight * closest.ang) closest = at;
        if (at.pos > target.tolerance || at.ang > 0.01) continue;
        anyReached = true;
        std::string why;
        if (collisions && collisions->collides(kin.forward(at.q), &why)) {
            if (blocked.empty()) blocked = why;
            continue;
        }
        double d = 0;
        for (std::size_t j : vars) {
            const double s = (at.q[j] - q0[j]) * jointScale(rig, j);
            d += s * s;
        }
        if (d < chosenDist) chosenDist = d, chosen = at;
        // The start's own solution is the nearest there is; otherwise a few to choose from.
        if (k == 0 || ++found >= 4) break;
    }
    if (chosenDist < 1e300) {
        res.ok = true;
        res.q = kin.withMimics(chosen.q);
        res.error = chosen.pos;
        return res;
    }
    res.q = kin.withMimics(closest.q.empty() ? q0 : closest.q);
    res.error = closest.pos;
    if (anyReached) {
        res.collisionFree = false;
        res.why = "every way to reach it collides (" + blocked + ")";
    } else {
        char buf[160];
        std::snprintf(buf, sizeof buf, "out of reach: the closest the tool gets is %.1f mm away", closest.pos * 1e3);
        res.why = buf;
        if (orient && closest.pos <= target.tolerance) res.why = "can't point the tool that way there";
    }
    return res;
}

// --- planning ----------------------------------------------------------------------------------

PlanResult planPath(const RobotKinematics& kin, const CollisionModel& collisions, const std::vector<double>& start, const std::vector<double>& goal,
                    const std::vector<std::size_t>& active, int maxIterations, unsigned seed) {
    const RobotRig& rig = kin.rig();
    PlanResult res;
    const std::size_t n = active.size();
    std::vector<double> base = start;
    base.resize(rig.joints.size(), 0.0);
    using Node = std::vector<double>;
    auto full = [&](const Node& x) {
        std::vector<double> q = base;
        for (std::size_t i = 0; i < n; ++i) q[active[i]] = x[i];
        return kin.withMimics(q);
    };
    auto pick = [&](const std::vector<double>& q) {
        Node x(n);
        for (std::size_t i = 0; i < n; ++i) x[i] = active[i] < q.size() ? q[active[i]] : 0.0;
        return x;
    };
    std::vector<double> scale(n);
    for (std::size_t i = 0; i < n; ++i) scale[i] = jointScale(rig, active[i]);
    auto dist = [&](const Node& a, const Node& b) {
        double s = 0;
        for (std::size_t i = 0; i < n; ++i) {
            const double d = (a[i] - b[i]) * scale[i];
            s += d * d;
        }
        return std::sqrt(s);
    };
    std::string why;
    auto clearAt = [&](const Node& x) { return !collisions.collides(kin.forward(full(x)), &why); };
    // Checked about every degree along the way (the end included, not the start).
    auto edgeFree = [&](const Node& a, const Node& b) {
        double span = 0;
        for (std::size_t i = 0; i < n; ++i) span = std::max(span, std::fabs(b[i] - a[i]) * scale[i]);
        const int steps = std::max(1, static_cast<int>(std::ceil(span / 0.0175)));
        Node x(n);
        for (int s = 1; s <= steps; ++s) {
            const double u = static_cast<double>(s) / steps;
            for (std::size_t i = 0; i < n; ++i) x[i] = a[i] + (b[i] - a[i]) * u;
            if (!clearAt(x)) return false;
        }
        return true;
    };
    const Node qs = pick(start), qg = pick(goal);
    if (!clearAt(qs)) {
        res.why = "the arm collides where it starts (" + why + ")";
        return res;
    }
    if (!clearAt(qg)) {
        res.why = "the arm would collide at the goal (" + why + ")";
        return res;
    }
    if (n == 0 || edgeFree(qs, qg)) {
        res.ok = true;
        res.path = {full(qs), full(qg)};
        return res;
    }
    struct Tree {
        std::vector<Node> nodes;
        std::vector<int> parent;
    };
    Tree ta{{qs}, {-1}}, tb{{qg}, {-1}};
    bool swapped = false;
    Rng rng(seed);
    constexpr double kStep = 0.1;
    auto nearest = [&](const Tree& t, const Node& x) {
        std::size_t best = 0;
        double bd = 1e300;
        for (std::size_t i = 0; i < t.nodes.size(); ++i) {
            const double d = dist(t.nodes[i], x);
            if (d < bd) bd = d, best = i;
        }
        return best;
    };
    // One step of a tree toward x: 0 blocked, 1 advanced, 2 reached x.
    auto extend = [&](Tree& t, const Node& x) {
        const std::size_t ni = nearest(t, x);
        const Node near = t.nodes[ni];
        const double d = dist(near, x);
        Node y = x;
        const bool reach = d <= kStep;
        if (!reach)
            for (std::size_t i = 0; i < n; ++i) y[i] = near[i] + (x[i] - near[i]) * (kStep / d);
        if (!edgeFree(near, y)) return 0;
        t.nodes.push_back(y);
        t.parent.push_back(static_cast<int>(ni));
        return reach ? 2 : 1;
    };
    auto trace = [](const Tree& t, int i) {
        std::vector<Node> out;
        for (; i >= 0; i = t.parent[static_cast<std::size_t>(i)]) out.push_back(t.nodes[static_cast<std::size_t>(i)]);
        return out; // from i back to the root
    };
    std::vector<double> lo(n), hi(n);
    for (std::size_t i = 0; i < n; ++i) {
        lo[i] = std::max(kin.lower(active[i]), qs[i] - kPiD);
        hi[i] = std::min(kin.upper(active[i]), qs[i] + kPiD);
    }
    std::vector<Node> path;
    for (int it = 0; it < maxIterations; ++it) {
        res.iterations = it + 1;
        Node r(n);
        for (std::size_t i = 0; i < n; ++i) r[i] = rng.between(lo[i], hi[i]);
        if (extend(ta, r) == 0) {
            std::swap(ta, tb), swapped = !swapped;
            continue;
        }
        const Node added = ta.nodes.back();
        int st = 1;
        while (st == 1) st = extend(tb, added);
        if (st == 2) {
            // ta's branch root..added, then tb's from added back to its root.
            std::vector<Node> a = trace(ta, static_cast<int>(ta.nodes.size()) - 1);
            std::reverse(a.begin(), a.end());
            const std::vector<Node> b = trace(tb, static_cast<int>(tb.nodes.size()) - 1);
            path = a;
            path.insert(path.end(), b.begin() + 1, b.end());
            if (swapped) std::reverse(path.begin(), path.end());
            break;
        }
        std::swap(ta, tb), swapped = !swapped;
    }
    if (path.empty()) {
        res.why = "no collision-free way found in " + std::to_string(maxIterations) + " tries";
        return res;
    }
    // Shortcuts: straight lines between waypoints wherever they are clear.
    for (int k = 0; k < 100 && path.size() > 2; ++k) {
        std::size_t i = static_cast<std::size_t>(rng.next() * static_cast<double>(path.size()));
        std::size_t j = static_cast<std::size_t>(rng.next() * static_cast<double>(path.size()));
        if (i > j) std::swap(i, j);
        if (j >= path.size() || j < i + 2) continue;
        if (edgeFree(path[i], path[j])) path.erase(path.begin() + static_cast<std::ptrdiff_t>(i + 1), path.begin() + static_cast<std::ptrdiff_t>(j));
    }
    res.ok = true;
    for (const Node& x : path) res.path.push_back(full(x));
    return res;
}

// --- dynamics and timing -----------------------------------------------------------------------

std::vector<double> inverseDynamics(const RobotKinematics& kin, const std::vector<double>& q, const std::vector<double>& qd, const std::vector<double>& qdd,
                                    const V3d& gravity) {
    const RobotRig& rig = kin.rig();
    const std::size_t nl = rig.links.size(), nj = rig.joints.size();
    const std::vector<double> Q = kin.withMimics(q), Qd = kin.withMimics(qd), Qdd = kin.withMimics(qdd);
    const Frames f = framesAt(rig, Q);
    std::vector<SV> v(nl), a(nl), S(nj), F(nl);
    a[0] = {{0, 0, 0}, mul(gravity, -1.0)}; // gravity as the base accelerating up
    for (std::size_t j = 0; j < nj; ++j) {
        const RigJoint& J = rig.joints[j];
        const std::size_t p = static_cast<std::size_t>(J.parent), c = static_cast<std::size_t>(J.child);
        S[j] = J.type == RigJointType::Prismatic ? SV{{0, 0, 0}, f.axis[j]} : SV{f.axis[j], cross(f.origin[j], f.axis[j])};
        const SV vj = S[j] * Qd[j];
        v[c] = v[p] + vj;
        a[c] = a[p] + S[j] * Qdd[j] + crm(v[c], vj);
    }
    for (std::size_t l = 0; l < nl; ++l) {
        const RigLink& L = rig.links[l];
        const LinkPose& P = f.link[l];
        const M3 Ic = mm(mm(P.R, L.inertia), transpose(P.R));
        const M6 I = spatialInertia(std::max(L.mass, 1e-4), P.apply(L.com), Ic);
        F[l] = I * a[l] + crf(v[l], I * v[l]);
    }
    std::vector<double> tau(nj, 0.0);
    for (std::size_t j = nj; j-- > 0;) {
        const RigJoint& J = rig.joints[j];
        const std::size_t p = static_cast<std::size_t>(J.parent), c = static_cast<std::size_t>(J.child);
        tau[j] = sdot(S[j], F[c]);
        F[p] = F[p] + F[c];
    }
    return tau;
}


namespace {

// Per joint, the joint it is coupled to (-1: none).
std::vector<int> leadersOf(const RobotRig& rig) {
    std::vector<int> leader(rig.joints.size(), -1);
    for (std::size_t j = 0; j < rig.joints.size(); ++j)
        if (!rig.joints[j].mimic.empty())
            for (std::size_t k = 0; k < rig.joints.size(); ++k)
                if (k != j && rig.joints[k].name == rig.joints[j].mimic) leader[j] = static_cast<int>(k);
    return leader;
}

// Sum of the squared motor torques (a coupled joint's load is its leader's motor's).
double squaredTorques(const RobotKinematics& kin, const std::vector<int>& leader, const std::vector<double>& q, const std::vector<double>& qd,
                      const std::vector<double>& qdd) {
    std::vector<double> tau = inverseDynamics(kin, q, qd, qdd);
    for (std::size_t j = 0; j < tau.size(); ++j)
        if (leader[j] >= 0) tau[static_cast<std::size_t>(leader[j])] += kin.rig().joints[j].mimicRatio * tau[j], tau[j] = 0;
    double s = 0;
    for (double t : tau) s += t * t;
    return s;
}

// Joint values, rates and accelerations of a timed path at time t, the way a keyframed
// motion moves through its keys: minimum-jerk segments that stop at every waypoint, or
// (smooth) a cubic Hermite spline with tangents from the neighbouring keys.
void pathState(const TimedPath& p, double t, std::vector<double>& q, std::vector<double>& qd, std::vector<double>& qdd) {
    const std::size_t n = p.q.empty() ? 0 : p.q.front().size(), last = p.q.size() - 1;
    q.assign(n, 0.0), qd.assign(n, 0.0), qdd.assign(n, 0.0);
    if (p.q.empty()) return;
    if (t <= p.times.front() || last == 0) {
        q = p.q.front();
        return;
    }
    if (t >= p.times.back()) {
        q = p.q.back();
        return;
    }
    std::size_t i = 0;
    while (i + 1 < last && t >= p.times[i + 1]) ++i;
    const double dt = p.times[i + 1] - p.times[i];
    if (dt <= 0) {
        q = p.q[i + 1];
        return;
    }
    const double s = (t - p.times[i]) / dt;
    if (!p.smooth) {
        double x, dx, ddx;
        minJerk(s, x, dx, ddx);
        for (std::size_t j = 0; j < n; ++j) {
            const double d = p.q[i + 1][j] - p.q[i][j];
            q[j] = p.q[i][j] + d * x;
            qd[j] = d * dx / dt;
            qdd[j] = d * ddx / (dt * dt);
        }
        return;
    }
    auto slope = [&](std::size_t k, std::size_t j) {
        if (k == 0 || k >= last) return 0.0;
        const double span = p.times[k + 1] - p.times[k - 1];
        return span > 0 ? (p.q[k + 1][j] - p.q[k - 1][j]) / span : 0.0;
    };
    const double s2 = s * s, s3 = s2 * s;
    for (std::size_t j = 0; j < n; ++j) {
        const double p0 = p.q[i][j], p1 = p.q[i + 1][j], m0 = slope(i, j) * dt, m1 = slope(i + 1, j) * dt;
        q[j] = (2 * s3 - 3 * s2 + 1) * p0 + (s3 - 2 * s2 + s) * m0 + (-2 * s3 + 3 * s2) * p1 + (s3 - s2) * m1;
        qd[j] = ((6 * s2 - 6 * s) * p0 + (3 * s2 - 4 * s + 1) * m0 + (-6 * s2 + 6 * s) * p1 + (3 * s2 - 2 * s) * m1) / dt;
        qdd[j] = ((12 * s - 6) * p0 + (6 * s - 4) * m0 + (-12 * s + 6) * p1 + (6 * s - 2) * m1) / (dt * dt);
    }
}

// The k-th shape a segment can bend by (k >= 1): 16 u^2 (1 - u)^2 sin(k pi u), zero with
// its first two derivatives at both ends, so the waypoints and their stops stay put.
void bendShape(int k, double u, double& b, double& db, double& ddb) {
    const double w = 16 * u * u * (1 - u) * (1 - u), dw = 32 * u * (1 - u) * (1 - 2 * u), ddw = 32 * (1 - 6 * u + 6 * u * u);
    const double f = k * kPiD, S = std::sin(f * u), dS = f * std::cos(f * u), ddS = -f * f * S;
    b = w * S;
    db = dw * S + w * dS;
    ddb = ddw * S + 2 * dw * dS + w * ddS;
}

// The path sampled every 40 ms or less (each segment's ends included), for smooth keys.
TimedPath densePath(const TimedPath& p) {
    if (p.smooth || p.q.size() < 2) return p;
    TimedPath d;
    d.smooth = true;
    d.effort = p.effort;
    d.times.push_back(p.times.front());
    d.q.push_back(p.q.front());
    std::vector<double> q, qd, qdd;
    for (std::size_t s = 0; s + 1 < p.q.size(); ++s) {
        const double T = p.times[s + 1] - p.times[s];
        const int m = std::max(2, static_cast<int>(std::ceil(T / 0.04)));
        for (int k = 1; k <= m; ++k) {
            const double t = p.times[s] + T * k / m;
            if (k == m) {
                q = p.q[s + 1];
            } else {
                pathState(p, t, q, qd, qdd);
            }
            d.times.push_back(t);
            d.q.push_back(q);
        }
    }
    return d;
}

} // namespace

double pathEffort(const RobotKinematics& kin, const TimedPath& path) {
    if (path.q.size() < 2 || path.times.size() != path.q.size()) return 0;
    const std::vector<int> leader = leadersOf(kin.rig());
    const double total = path.times.back() - path.times.front();
    if (total <= 0) return 0;
    const int steps = std::max(2, static_cast<int>(std::ceil(total * 100)));
    const double h = total / steps;
    double effort = 0;
    std::vector<double> q, qd, qdd;
    for (int k = 0; k < steps; ++k) {
        pathState(path, path.times.front() + (k + 0.5) * h, q, qd, qdd);
        effort += squaredTorques(kin, leader, q, qd, qdd) * h;
    }
    return effort;
}

TimedPath timePath(const RobotKinematics& kin, const std::vector<std::vector<double>>& path, double speedScale) {
    const RobotRig& rig = kin.rig();
    TimedPath out;
    out.q = path;
    if (path.empty()) return out;
    out.times.push_back(0.0);
    const double s = std::clamp(speedScale, 0.01, 1.0);
    for (std::size_t k = 1; k < path.size(); ++k) {
        double slowest = 0;
        for (std::size_t j = 0; j < rig.joints.size() && j < path[k].size() && j < path[k - 1].size(); ++j) {
            if (!rig.joints[j].mimic.empty()) continue;
            const double vmax = std::max(rigJointVelocity(rig, j), 1e-3);
            slowest = std::max(slowest, std::fabs(path[k][j] - path[k - 1][j]) / vmax);
        }
        // A minimum-jerk move peaks at 1.875 x its mean speed.
        out.times.push_back(out.times.back() + std::max(1.875 * slowest / s, 0.05));
    }
    out.effort = pathEffort(kin, out);
    return out;
}

TimedPath optimisePath(const RobotKinematics& kin, const CollisionModel& collisions, const TimedPath& initial, const std::vector<std::size_t>& active,
                       int iterations) {
    const RobotRig& rig = kin.rig();
    TimedPath plain = initial;
    plain.effort = pathEffort(kin, plain);
    if (plain.smooth || plain.q.size() < 2 || plain.times.size() != plain.q.size() || active.empty()) return plain;
    const std::size_t segs = plain.q.size() - 1, n = active.size();
    constexpr int kShapes = 2;
    const std::vector<int> leader = leadersOf(rig);
    // Each segment of the planner's path, between the same waypoints in the same time, bent by
    // kShapes smooth shapes per joint: c[(s * kShapes + k) * n + i].
    auto state = [&](const std::vector<double>& c, std::size_t s, double u, std::vector<double>& q, std::vector<double>& qd, std::vector<double>& qdd) {
        const double T = plain.times[s + 1] - plain.times[s];
        double x, dx, ddx;
        minJerk(u, x, dx, ddx);
        const std::vector<double>&a = plain.q[s], &b = plain.q[s + 1];
        q.resize(a.size()), qd.resize(a.size()), qdd.resize(a.size());
        for (std::size_t j = 0; j < a.size(); ++j) {
            q[j] = a[j] + (b[j] - a[j]) * x;
            qd[j] = (b[j] - a[j]) * dx / T;
            qdd[j] = (b[j] - a[j]) * ddx / (T * T);
        }
        for (int k = 0; k < kShapes; ++k) {
            double B, dB, ddB;
            bendShape(k + 1, u, B, dB, ddB);
            for (std::size_t i = 0; i < n; ++i) {
                const double w = c[(s * kShapes + static_cast<std::size_t>(k)) * n + i];
                const std::size_t j = active[i];
                q[j] += w * B, qd[j] += w * dB / T, qdd[j] += w * ddB / (T * T);
            }
        }
        q = kin.withMimics(q), qd = kin.withMimics(qd), qdd = kin.withMimics(qdd);
    };
    // Effort at 50 Hz, or infinity where a joint would leave its range or go faster than it can.
    auto cost = [&](const std::vector<double>& c) {
        double effort = 0;
        std::vector<double> q, qd, qdd;
        for (std::size_t s = 0; s < segs; ++s) {
            const double T = plain.times[s + 1] - plain.times[s];
            const int steps = std::max(2, static_cast<int>(std::ceil(T * 50)));
            for (int k = 0; k < steps; ++k) {
                state(c, s, (k + 0.5) / steps, q, qd, qdd);
                for (std::size_t j : active)
                    if (std::fabs(qd[j]) > 0.95 * rigJointVelocity(rig, j) || q[j] < kin.lower(j) || q[j] > kin.upper(j))
                        return std::numeric_limits<double>::infinity();
                effort += squaredTorques(kin, leader, q, qd, qdd) * T / steps;
            }
        }
        return effort;
    };
    // Clear of collisions, checked about every degree along the bent segments.
    auto clear = [&](const std::vector<double>& c) {
        std::vector<double> q, qd, qdd;
        for (std::size_t s = 0; s < segs; ++s) {
            double span = 0;
            for (std::size_t i = 0; i < n; ++i) {
                double bend = 0;
                for (int k = 0; k < kShapes; ++k) bend += std::fabs(c[(s * kShapes + static_cast<std::size_t>(k)) * n + i]);
                span = std::max(span, (std::fabs(plain.q[s + 1][active[i]] - plain.q[s][active[i]]) + 4 * bend) * jointScale(rig, active[i]));
            }
            const int steps = std::max(2, static_cast<int>(std::ceil(span / 0.0175)));
            for (int k = 1; k < steps; ++k) {
                state(c, s, static_cast<double>(k) / steps, q, qd, qdd);
                if (collisions.collides(kin.forward(q))) return false;
            }
        }
        return true;
    };
    std::vector<double> c(segs * kShapes * n, 0.0), g(c.size());
    double f = cost(c);
    if (!std::isfinite(f)) return plain;
    double stepLen = 0.05;
    for (int it = 0; it < iterations && stepLen > 1e-4; ++it) {
        double gn = 0;
        for (std::size_t k = 0; k < c.size(); ++k) {
            const double h = 1e-4;
            std::vector<double> cp = c;
            cp[k] += h;
            const double fp = cost(cp);
            if (std::isfinite(fp)) {
                g[k] = (fp - f) / h;
            } else {
                cp[k] = c[k] - h;
                const double fm = cost(cp);
                g[k] = std::isfinite(fm) ? (f - fm) / h : 0.0;
            }
            gn += g[k] * g[k];
        }
        gn = std::sqrt(gn);
        if (gn < 1e-12) break;
        // Down the slope, the step shortened until it pays and stays clear.
        bool accepted = false;
        while (stepLen > 1e-4) {
            std::vector<double> y = c;
            for (std::size_t k = 0; k < c.size(); ++k) y[k] -= stepLen * g[k] / gn;
            const double fy = cost(y);
            if (fy < f && clear(y)) {
                c = y, f = fy;
                stepLen *= 1.5;
                accepted = true;
                break;
            }
            stepLen *= 0.5;
        }
        if (!accepted) break;
    }
    // The bent path, sampled finely enough for smooth keys to follow it.
    TimedPath out;
    out.smooth = true;
    out.times.push_back(plain.times.front());
    out.q.push_back(plain.q.front());
    std::vector<double> q, qd, qdd;
    for (std::size_t s = 0; s < segs; ++s) {
        const double T = plain.times[s + 1] - plain.times[s];
        const int m = std::max(2, static_cast<int>(std::ceil(T / 0.04)));
        for (int k = 1; k <= m; ++k) {
            if (k == m) q = plain.q[s + 1];
            else state(c, s, static_cast<double>(k) / m, q, qd, qdd);
            out.times.push_back(plain.times[s] + T * k / m);
            out.q.push_back(q);
        }
    }
    out.effort = pathEffort(kin, out);
    return out.effort < plain.effort ? out : plain;
}

// --- writing it into the scene -----------------------------------------------------------------

std::vector<std::string> writeJointPath(Scene& scene, const RobotRig& rig, const TimedPath& path, float t0, const std::vector<std::size_t>& active) {
    std::vector<std::string> changed;
    if (path.q.empty() || path.times.size() != path.q.size()) return changed;
    auto& motions = scene.motions();
    auto motionNamed = [&](const std::string& name) -> int {
        for (std::size_t i = 0; i < motions.size(); ++i)
            if (motions[i].name == name) return static_cast<int>(i);
        return -1;
    };
    std::optional<TimedPath> dense;
    for (std::size_t j : active) {
        if (j >= rig.joints.size()) continue;
        const RigJoint& J = rig.joints[j];
        if (J.motion < 0 || static_cast<std::size_t>(J.motion) >= motions.size()) continue;
        int target = J.motion;
        double factor = J.type == RigJointType::Prismatic ? 1e3 : 180.0 / kPiD; // to mm / deg
        // A joint geared to a drive motor: key the motor, so everything on it moves together.
        if (!J.drive.empty()) {
            const Motion& own = motions[static_cast<std::size_t>(J.motion)];
            const int d = motionNamed(J.drive);
            if (d >= 0 && own.kind == MotionKind::Circular && motions[static_cast<std::size_t>(d)].kind == MotionKind::Circular && own.ratio != 0.0f) {
                target = d;
                factor /= own.ratio;
            }
        }
        Motion& m = motions[static_cast<std::size_t>(target)];
        if (std::find(changed.begin(), changed.end(), m.name) != changed.end()) continue;
        if (m.start > t0) {
            m.start = t0;
            m.keys.clear();
        }
        const float tk = t0 - m.start;
        // Smooth keys (a spline through them) for a smooth path, or where the motion already
        // has them; then every move on it is sampled finely so it keeps its shape.
        const bool spline = path.smooth || (m.keySpline && m.keys.size() > 1);
        if (spline && !m.keySpline && m.keys.size() > 1) {
            // Its earlier keys as they moved (stopping at each), sampled for the spline.
            const Motion old = m;
            std::vector<MotionKey> keys;
            for (std::size_t k = 0; k + 1 < old.keys.size() && old.keys[k].t < tk; ++k) {
                const float a = old.keys[k].t, b = std::min(old.keys[k + 1].t, tk);
                const int steps = std::max(1, static_cast<int>(std::ceil((b - a) / 0.04f)));
                for (int s = 0; s < steps; ++s) {
                    const float t = a + (b - a) * static_cast<float>(s) / static_cast<float>(steps);
                    keys.push_back({t, keyedValue(old, old.start + t)});
                }
            }
            m.keys = keys;
        }
        if (spline && !dense) dense = densePath(path);
        const TimedPath& src = spline ? *dense : path;
        auto value = [&](std::size_t k) { return static_cast<float>(src.q[k][j] * factor); };
        if (m.keys.empty()) {
            if (tk > 1e-4f) m.keys.push_back({0.0f, value(0)});
        } else {
            m.keys.erase(std::remove_if(m.keys.begin(), m.keys.end(), [&](const MotionKey& k) { return k.t >= tk - 1e-4f; }), m.keys.end());
        }
        m.keys.push_back({tk, value(0)});
        for (std::size_t k = 1; k < src.q.size(); ++k) m.keys.push_back({tk + static_cast<float>(src.times[k]), value(k)});
        m.sortKeys();
        m.mode = MotionMode::Once;
        m.keySpline = spline;
        m.keysAdvance = false;
        m.enabled = true;
        changed.push_back(m.name);
    }
    return changed;
}

} // namespace sicdm

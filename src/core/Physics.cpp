#include "core/Physics.hpp"

#include "core/Multibody.hpp"
#include "core/Scene.hpp"
#include "core/Units.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>

namespace sicdm {

namespace {

constexpr double kStep = 0.0005;       // s, integration step
constexpr int kStepsPerFrame = 10;     // frames every 5 ms
constexpr double kMm = 1e-3;           // mm -> m
constexpr double kRestSpeed = 0.05;    // m/s: slower than this against the floor = resting
constexpr int kSupportPoints = 2048;   // mesh points kept for floor contact
constexpr double kRollingResistance = 0.01; // a ball rolling on the floor

struct V3 {
    double x = 0, y = 0, z = 0;
};
V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
V3 operator*(V3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
V3& operator+=(V3& a, V3 b) { return a = a + b; }
V3& operator-=(V3& a, V3 b) { return a = a - b; }
double dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
double norm(V3 a) { return std::sqrt(dot(a, a)); }
V3 v3(const Vec3& v) { return {v.x, v.y, v.z}; }
Vec3 vec(V3 v) { return {static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)}; }

// 3x3 rotation (row-major) for the spin.
struct Rot {
    double m[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    V3 apply(V3 v) const {
        return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z, m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
    }
};
Rot axisAngle(V3 axis, double angle) {
    Rot r;
    const double len = norm(axis);
    if (len < 1e-15 || angle == 0.0) return r;
    const V3 a = axis * (1.0 / len);
    const double c = std::cos(angle), s = std::sin(angle), k = 1 - c;
    r.m[0][0] = c + a.x * a.x * k, r.m[0][1] = a.x * a.y * k - a.z * s, r.m[0][2] = a.x * a.z * k + a.y * s;
    r.m[1][0] = a.y * a.x * k + a.z * s, r.m[1][1] = c + a.y * a.y * k, r.m[1][2] = a.y * a.z * k - a.x * s;
    r.m[2][0] = a.z * a.x * k - a.y * s, r.m[2][1] = a.z * a.y * k + a.x * s, r.m[2][2] = c + a.z * a.z * k;
    return r;
}
Rot operator*(const Rot& a, const Rot& b) {
    Rot r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) r.m[i][j] = a.m[i][0] * b.m[0][j] + a.m[i][1] * b.m[1][j] + a.m[i][2] * b.m[2][j];
    return r;
}
Mat4 toMat4(const Rot& r) {
    Mat4 m = Mat4::identity();
    // Mat4 is column-major: m.m[col][row].
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) m.m[j][i] = static_cast<float>(r.m[i][j]);
    return m;
}

// The live state of one body during integration (SI).
struct Live {
    V3 x, v, p;            // m, m/s, kg m/s
    V3 force;              // N, field and applied forces of the last step
    V3 accel;              // m/s^2
    Rot rot;
    V3 omega;              // rad/s
    double temperature = 0, properTime = 0;
    bool floorContact = false, touching = false;
    double invMass = 0;    // 0: fixed
};

double gammaOf(V3 v, const PhysicsSettings& s) {
    if (!s.relativity) return 1.0;
    const double b2 = std::min(dot(v, v) / (s.lightSpeed * s.lightSpeed), 1.0 - 1e-15);
    return 1.0 / std::sqrt(1.0 - b2);
}
// Velocity from momentum: v = p / (gamma m), so |v| < c however large p grows.
V3 velocityOf(V3 p, double mass, const PhysicsSettings& s) {
    if (mass <= 0) return {};
    if (!s.relativity) return p * (1.0 / mass);
    const double c = s.lightSpeed;
    return p * (1.0 / std::sqrt(mass * mass + dot(p, p) / (c * c)));
}
double kineticEnergy(V3 v, double mass, const PhysicsSettings& s) {
    if (!s.relativity) return 0.5 * mass * dot(v, v);
    return (gammaOf(v, s) - 1.0) * mass * s.lightSpeed * s.lightSpeed;
}

std::uint64_t fnv(std::uint64_t h, const void* data, std::size_t n) {
    const auto* b = static_cast<const unsigned char*>(data);
    for (std::size_t i = 0; i < n; ++i) h = (h ^ b[i]) * 1099511628211ull;
    return h;
}
template <class T>
std::uint64_t mix(std::uint64_t h, const T& v) {
    return fnv(h, &v, sizeof(v));
}
std::uint64_t mixProps(std::uint64_t h, const PhysicsProps& p) {
    h = mix(h, p.enabled), h = mix(h, p.fixed), h = mix(h, p.mass), h = mix(h, p.density);
    h = mix(h, p.velocity), h = mix(h, p.spin), h = mix(h, p.charge), h = mix(h, p.temperature);
    h = mix(h, p.specificHeat), h = mix(h, p.restitution), h = mix(h, p.force), h = mix(h, p.wave);
    h = mix(h, p.waveFrequency);
    return mix(h, p.waveSpeed);
}

// Everything a run depends on.
std::uint64_t signature(const Scene& scene) {
    const PhysicsSettings& s = scene.physics();
    std::uint64_t h = 1469598103934665603ull;
    h = mix(h, s.enabled), h = mix(h, s.gravity), h = mix(h, s.g), h = mix(h, s.bigG), h = mix(h, s.floor);
    h = mix(h, s.friction), h = mix(h, s.air), h = mix(h, s.electricField), h = mix(h, s.magneticField);
    h = mix(h, s.ambient), h = mix(h, s.convection), h = mix(h, s.conductance), h = mix(h, s.relativity);
    h = mix(h, s.lightSpeed), h = mix(h, s.duration);
    bool anyFixed = false;
    for (std::size_t i = 0; i < scene.bodies().size(); ++i) {
        const Body& b = scene.bodies()[i];
        if (!b.physics.enabled) continue;
        anyFixed = anyFixed || b.physics.fixed;
        h = mix(h, i), h = mixProps(h, b.physics), h = mix(h, b.offset), h = mix(h, b.scale), h = mix(h, b.rotation);
        h = mix(h, b.mesh.vertices.size());
        const AABB bb = b.mesh.bounds();
        h = mix(h, bb.min), h = mix(h, bb.max);
        h = fnv(h, b.name.data(), b.name.size());
    }
    if (anyFixed) // fixed bodies follow their motions
        for (const Motion& m : scene.motions()) {
            h = mix(h, m.enabled), h = mix(h, m.kind), h = mix(h, m.speed), h = mix(h, m.range), h = mix(h, m.mode);
            h = mix(h, m.start), h = mix(h, m.axis), h = mix(h, m.origin), h = mix(h, m.ratio), h = mix(h, m.keys.size());
            for (const MotionKey& k : m.keys) h = mix(h, k);
            h = fnv(h, m.follow.data(), m.follow.size());
            for (const std::string& n : m.bodies) h = fnv(h, n.data(), n.size());
        }
    return h;
}

// Floor contact points of a body, relative to its centre of mass (m).
std::vector<V3> supportPoints(const Body& b, const Vec3& centre) {
    std::vector<V3> pts;
    pts.reserve(b.mesh.vertices.size());
    for (const Vertex& v : b.mesh.vertices) pts.push_back(v3((v.position + b.offset) - centre) * kMm);
    if (pts.size() > static_cast<std::size_t>(kSupportPoints)) { // the outermost points carry the contacts
        std::nth_element(pts.begin(), pts.begin() + kSupportPoints, pts.end(),
                         [](V3 a, V3 c) { return dot(a, a) > dot(c, c); });
        pts.resize(kSupportPoints);
    }
    return pts;
}

// Collision shape: an exact ball, or the body's box (its bounds about the centre of
// mass, turned with it when it spins).
struct Collider {
    bool sphere = false;
    double radius = 0;
    V3 lo, hi; // box corners relative to the centre (m), current orientation
};

struct Contact {
    V3 normal;          // from the first body to the second
    double depth = 0;   // overlap (m); <= 0: apart by -depth
    double area = 0;    // m^2, for heat conduction
};

Contact contactOf(const Collider& a, V3 ca, const Collider& b, V3 cb) {
    Contact c;
    if (a.sphere && b.sphere) {
        const V3 d = cb - ca;
        const double dist = norm(d);
        c.normal = dist > 1e-12 ? d * (1.0 / dist) : V3{1, 0, 0};
        c.depth = a.radius + b.radius - dist;
        const double r = std::min(a.radius, b.radius) * 0.5;
        c.area = kPi * r * r;
        return c;
    }
    if (a.sphere != b.sphere) { // ball against box: the box's closest point
        const bool flip = !a.sphere;
        const Collider& ball = flip ? b : a;
        const Collider& box = flip ? a : b;
        const V3 cs = flip ? cb : ca, cx = flip ? ca : cb;
        const V3 lo = cx + box.lo, hi = cx + box.hi;
        const V3 q{std::clamp(cs.x, lo.x, hi.x), std::clamp(cs.y, lo.y, hi.y), std::clamp(cs.z, lo.z, hi.z)};
        V3 d = q - cs; // ball centre -> box
        double dist = norm(d);
        if (dist < 1e-12) { // centre inside the box: out through the nearest face
            const double gaps[6] = {cs.x - lo.x, hi.x - cs.x, cs.y - lo.y, hi.y - cs.y, cs.z - lo.z, hi.z - cs.z};
            const int k = static_cast<int>(std::min_element(gaps, gaps + 6) - gaps);
            const V3 axes[6] = {{-1, 0, 0}, {1, 0, 0}, {0, -1, 0}, {0, 1, 0}, {0, 0, -1}, {0, 0, 1}};
            d = axes[k] * -1.0; // towards the box, i.e. away from the nearest face
            dist = -gaps[k];
        }
        c.normal = dist > 0 ? d * (1.0 / dist) : d; // inside: d is already the unit push direction
        c.depth = ball.radius - dist;
        if (flip) c.normal = c.normal * -1.0;
        const double r = ball.radius * 0.5;
        c.area = kPi * r * r;
        return c;
    }
    // Box against box: the axis of least overlap.
    const V3 alo = ca + a.lo, ahi = ca + a.hi, blo = cb + b.lo, bhi = cb + b.hi;
    const double ox = std::min(ahi.x, bhi.x) - std::max(alo.x, blo.x), oy = std::min(ahi.y, bhi.y) - std::max(alo.y, blo.y),
                 oz = std::min(ahi.z, bhi.z) - std::max(alo.z, blo.z);
    const V3 d = cb - ca;
    if (ox <= oy && ox <= oz) c.normal = {d.x >= 0 ? 1.0 : -1.0, 0, 0}, c.depth = ox, c.area = std::max(oy, 0.0) * std::max(oz, 0.0);
    else if (oy <= oz) c.normal = {0, d.y >= 0 ? 1.0 : -1.0, 0}, c.depth = oy, c.area = std::max(ox, 0.0) * std::max(oz, 0.0);
    else c.normal = {0, 0, d.z >= 0 ? 1.0 : -1.0}, c.depth = oz, c.area = std::max(ox, 0.0) * std::max(oy, 0.0);
    if (ox < 0 || oy < 0 || oz < 0) c.depth = std::min({ox, oy, oz}); // apart
    return c;
}

std::shared_ptr<PhysicsRun> simulate(const Scene& scene) {
    auto run = std::make_shared<PhysicsRun>();
    const PhysicsSettings& s = scene.physics();
    run->settings = s;
    run->frameStep = static_cast<float>(kStep * kStepsPerFrame);
    std::vector<std::vector<V3>> support;
    std::vector<Collider> colliders;
    for (std::size_t i = 0; i < scene.bodies().size(); ++i) {
        const Body& b = scene.bodies()[i];
        if (!b.physics.enabled || b.mesh.vertices.empty()) continue;
        const MassProperties mp = massProperties(b);
        PhysicsBodyInfo info;
        info.body = i;
        info.name = b.name;
        info.props = b.physics;
        info.mass = mp.mass > 0 ? mp.mass : 1e-6;
        info.area = mp.area;
        info.restCentre = mp.centre;
        // Collision sphere: exact for balls, the same volume otherwise.
        info.radius = b.kind == "sphere" && !b.params.empty() ? b.params[0] * b.scale * kMm
                                                              : std::cbrt(3.0 * std::max(mp.volume, 1e-12) / (4.0 * kPi));
        run->bodies.push_back(info);
        support.push_back(supportPoints(b, mp.centre));
        Collider col;
        col.sphere = b.kind == "sphere";
        col.radius = info.radius;
        const AABB bb = b.bounds();
        col.lo = v3(bb.min - mp.centre) * kMm;
        col.hi = v3(bb.max - mp.centre) * kMm;
        colliders.push_back(col);
    }
    const std::size_t n = run->bodies.size();
    if (n == 0) return run;

    // Fixed bodies follow their motions (moving anchors, a paddle...).
    bool anyFixedMoving = false;
    for (const PhysicsBodyInfo& b : run->bodies)
        if (b.props.fixed && isMoving(scene, b.body)) anyFixedMoving = true;

    std::vector<Live> L(n);
    for (std::size_t i = 0; i < n; ++i) {
        const PhysicsBodyInfo& b = run->bodies[i];
        Live& l = L[i];
        l.x = v3(b.restCentre) * kMm;
        l.v = b.props.fixed ? V3{} : v3(b.props.velocity) * kMm;
        l.p = l.v * (b.mass * gammaOf(l.v, s));
        if (s.relativity) l.v = velocityOf(l.p, b.mass, s); // a start speed above c is held below it
        l.omega = b.props.fixed ? V3{} : v3(b.props.spin) * (kPi / 180.0);
        l.temperature = b.props.temperature;
        l.invMass = b.props.fixed ? 0.0 : 1.0 / b.mass;
    }
    auto fixedPlacement = [&](float t) {
        if (!anyFixedMoving) return;
        const std::vector<Mat4> tf = motionTransforms(scene, t);
        for (std::size_t i = 0; i < n; ++i) {
            if (!run->bodies[i].props.fixed) continue;
            const V3 x = v3(tf[run->bodies[i].body].transformPoint(run->bodies[i].restCentre)) * kMm;
            L[i].v = (x - L[i].x) * (1.0 / (kStep * kStepsPerFrame));
            L[i].x = x;
        }
    };

    const bool uniform = s.gravity == GravityMode::Uniform || s.gravity == GravityMode::Both;
    const bool newton = s.gravity == GravityMode::Newton || s.gravity == GravityMode::Both;
    const V3 g = uniform ? v3(s.g) : V3{};
    const V3 E = v3(s.electricField), B = v3(s.magneticField);

    PhysicsEnergy acc; // running heat / work totals
    auto lowestPoint = [&](std::size_t i) {
        double low = 1e300;
        for (const V3& q : support[i]) low = std::min(low, L[i].rot.apply(q).z);
        return L[i].x.z + low;
    };
    // Mechanical energy of body i (kinetic + potential in the uniform fields).
    auto mechanical = [&](std::size_t i) {
        const PhysicsBodyInfo& b = run->bodies[i];
        return kineticEnergy(L[i].v, b.mass, s) - b.mass * dot(g, L[i].x) - b.props.charge * dot(E, L[i].x);
    };
    auto pairRadius = [&](std::size_t i, std::size_t j) { return run->bodies[i].radius + run->bodies[j].radius; };
    auto addHeat = [&](std::size_t i, double joules) {
        const PhysicsBodyInfo& b = run->bodies[i];
        L[i].temperature += joules / (b.mass * std::max(b.props.specificHeat, 1.0f));
        acc.thermal += joules;
    };

    auto record = [&](float t) {
        PhysicsFrame f;
        f.t = t;
        f.energy = acc;
        V3 momentum;
        for (std::size_t i = 0; i < n; ++i) {
            const PhysicsBodyInfo& b = run->bodies[i];
            const Live& l = L[i];
            PhysicsBodyState st;
            st.position = vec(l.x * 1000.0);
            st.velocity = vec(l.v * 1000.0);
            st.acceleration = vec(l.accel * 1000.0);
            st.rotation = toMat4(l.rot);
            st.force = vec(l.force);
            st.temperature = static_cast<float>(l.temperature);
            st.properTime = l.properTime;
            st.touching = l.touching;
            f.bodies.push_back(st);
            if (!b.props.fixed) {
                f.energy.kinetic += kineticEnergy(l.v, b.mass, s);
                momentum += l.p;
            }
            f.energy.gravity -= b.mass * dot(g, l.x);
            f.energy.electric -= b.props.charge * dot(E, l.x);
            for (std::size_t j = i + 1; j < n; ++j) {
                const PhysicsBodyInfo& c = run->bodies[j];
                const double r = std::max(norm(L[j].x - l.x), 0.5 * pairRadius(i, j));
                if (newton) f.energy.gravity -= s.bigG * b.mass * c.mass / r;
                f.energy.electric += kCoulombConstant * b.props.charge * c.props.charge / r;
            }
        }
        f.momentum = vec(momentum);
        run->frames.push_back(std::move(f));
    };

    const int steps = static_cast<int>(std::ceil(s.duration / kStep));
    fixedPlacement(0.0f);
    std::vector<V3> F(n), drag(n);
    // Field, gravity, drag and applied forces (the magnetic force is applied as an exact
    // rotation of the velocity, below).
    auto computeForces = [&]() {
        for (std::size_t i = 0; i < n; ++i) {
            const PhysicsBodyInfo& b = run->bodies[i];
            V3 f = g * b.mass + v3(b.props.force) + E * b.props.charge;
            drag[i] = {};
            if (s.air && !b.props.fixed) { // drag 1/2 rho Cd A v^2, against the motion
                const double speed = norm(L[i].v);
                drag[i] = L[i].v * (-0.5 * 1.225 * 0.47 * kPi * b.radius * b.radius * speed);
                f += drag[i];
            }
            F[i] = f;
        }
        for (std::size_t i = 0; i < n; ++i)
            for (std::size_t j = i + 1; j < n; ++j) {
                const V3 d = L[j].x - L[i].x;
                const double r = std::max(norm(d), 0.5 * pairRadius(i, j)); // no singularity inside the bodies
                const V3 u = d * (1.0 / std::max(norm(d), 1e-12));
                double attract = 0;
                if (newton) attract += s.bigG * run->bodies[i].mass * run->bodies[j].mass / (r * r); // Newton
                attract -= kCoulombConstant * run->bodies[i].props.charge * run->bodies[j].props.charge / (r * r); // Coulomb
                F[i] += u * attract;
                F[j] -= u * attract;
            }
    };
    // The floor's normal force: whatever pushes a body resting on it into the floor is held
    // by an equal and opposite reaction (Newton's third law); friction opposes sliding
    // (and turns the motion into heat). Bodies touching each other meet with impulses.
    std::vector<V3> held(n); // contact force on each body
    auto contactForces = [&](double h, bool heat) {
        for (std::size_t i = 0; i < n; ++i) held[i] = {};
        for (std::size_t i = 0; i < n; ++i) {
            Live& l = L[i];
            if (l.invMass == 0 || !(s.floor && l.floorContact)) continue;
            V3 f = F[i] + held[i];
            if (f.z >= 0 || std::fabs(l.v.z) >= kRestSpeed) continue;
            const double normal = -f.z;
            held[i].z += normal;
            const V3 slide{l.v.x, l.v.y, 0};
            const double speed = norm(slide);
            if (speed > 1e-12) {
                // A ball rolls (rolling resistance); anything else slides (Coulomb friction).
                const double mu = colliders[i].sphere ? kRollingResistance : s.friction;
                const double friction = std::min(mu * normal, speed * run->bodies[i].mass / h);
                held[i] -= slide * (friction / speed);
                if (heat) addHeat(i, friction * speed * h);
            }
        }
        for (std::size_t i = 0; i < n; ++i) F[i] += held[i];
    };
    // Total mechanical energy: kinetic and every potential.
    auto mechanicalTotal = [&]() {
        double e = 0;
        for (std::size_t i = 0; i < n; ++i) {
            e += mechanical(i);
            for (std::size_t j = i + 1; j < n; ++j) {
                const double r = std::max(norm(L[j].x - L[i].x), 0.5 * pairRadius(i, j));
                if (newton) e -= s.bigG * run->bodies[i].mass * run->bodies[j].mass / r;
                e += kCoulombConstant * run->bodies[i].props.charge * run->bodies[j].props.charge / r;
            }
        }
        return e;
    };
    auto kick = [&](std::size_t i, double h) { // p += F h (Newton's second law, dp/dt = F)
        Live& l = L[i];
        l.p += F[i] * h;
        l.v = velocityOf(l.p, run->bodies[i].mass, s);
        acc.toAir -= dot(drag[i], l.v) * h;
    };
    // Magnetic force q v x B as an exact rotation of the momentum (Boris): it bends the path
    // but never changes the speed, so it does no work.
    auto magnetic = [&](std::size_t i, double h) {
        const PhysicsBodyInfo& b = run->bodies[i];
        if (b.props.charge == 0 || norm(B) == 0) return;
        Live& l = L[i];
        const double gamma = gammaOf(l.v, s);
        const V3 t = B * (b.props.charge * h / (2 * gamma * b.mass));
        const V3 u = l.p * (1.0 / b.mass);
        const V3 u1 = u + cross(u, t);
        const V3 u2 = u + cross(u1, t * (2.0 / (1.0 + dot(t, t))));
        l.p = u2 * b.mass;
        l.v = velocityOf(l.p, b.mass, s);
    };

    computeForces();
    contactForces(kStep, false);
    for (std::size_t i = 0; i < n; ++i) L[i].force = F[i], L[i].accel = F[i] * L[i].invMass;
    record(0.0f);

    for (int step = 1; step <= steps; ++step) {
        const double h = kStep;
        // Velocity Verlet: half kick, drift, half kick with the forces at the new place.
        std::vector<V3> v0(n), x0(n);
        for (std::size_t i = 0; i < n; ++i) v0[i] = L[i].v, x0[i] = L[i].x;
        computeForces();
        contactForces(h * 0.5, true);
        for (std::size_t i = 0; i < n; ++i) {
            if (L[i].invMass == 0) continue;
            kick(i, h * 0.5);
            magnetic(i, h);
            L[i].x += L[i].v * h;
            L[i].rot = axisAngle(L[i].omega, norm(L[i].omega) * h) * L[i].rot;
        }
        computeForces();
        contactForces(h * 0.5, true);
        for (std::size_t i = 0; i < n; ++i) {
            Live& l = L[i];
            l.force = F[i] - held[i];
            if (l.invMass == 0) continue;
            kick(i, h * 0.5);
            l.accel = (l.v - v0[i]) * (1.0 / h);
            acc.externalWork += dot(v3(run->bodies[i].props.force), l.x - x0[i]); // W = F . dx
            l.properTime += 0.5 * h * (1.0 / gammaOf(v0[i], s) + 1.0 / gammaOf(l.v, s)); // d tau = dt / gamma
        }
        if (step % kStepsPerFrame == 0) fixedPlacement(static_cast<float>(step * h));

        // Contacts: the floor, then body pairs (momentum-conserving impulses). The
        // mechanical energy a contact takes away becomes heat in the bodies touching.
        const double before = mechanicalTotal();
        for (std::size_t i = 0; i < n; ++i) L[i].touching = L[i].floorContact = false;
        auto floorContacts = [&](bool count) {
            if (!s.floor) return;
            for (std::size_t i = 0; i < n; ++i) {
                Live& l = L[i];
                const PhysicsBodyInfo& b = run->bodies[i];
                if (b.props.fixed) continue;
                const double low = lowestPoint(i);
                l.floorContact = low <= 1e-4;
                l.touching = l.touching || l.floorContact;
                if (low >= 0) continue;
                l.x.z -= low;
                if (l.v.z < 0) {
                    const double vn = -l.v.z;
                    const double bounce = vn > kRestSpeed ? b.props.restitution * vn : 0.0;
                    // Friction during the impact: at most mu x the normal impulse. A solid ball
                    // starts rolling instead, keeping 5/7 of its slip.
                    const V3 slide{l.v.x, l.v.y, 0};
                    const double speed = norm(slide);
                    const double cut = std::min(colliders[i].sphere ? speed * 2.0 / 7.0 : speed, s.friction * (vn + bounce));
                    if (speed > 1e-12) l.v -= slide * (cut / speed);
                    l.v.z = bounce;
                    l.p = l.v * (b.mass * gammaOf(l.v, s));
                    if (count && vn > kRestSpeed) ++run->collisions;
                }
            }
        };
        floorContacts(true);
        // A spinning box turns its collision box with it.
        for (std::size_t i = 0; i < n; ++i) {
            if (colliders[i].sphere || norm(L[i].omega) == 0) continue;
            V3 lo{1e300, 1e300, 1e300}, hi{-1e300, -1e300, -1e300};
            for (const V3& q : support[i]) {
                const V3 r = L[i].rot.apply(q);
                lo = {std::min(lo.x, r.x), std::min(lo.y, r.y), std::min(lo.z, r.z)};
                hi = {std::max(hi.x, r.x), std::max(hi.y, r.y), std::max(hi.z, r.z)};
            }
            colliders[i].lo = lo, colliders[i].hi = hi;
        }
        for (std::size_t i = 0; i < n; ++i)
            for (std::size_t j = i + 1; j < n; ++j) {
                Live &a = L[i], &c = L[j];
                const Contact ct = contactOf(colliders[i], a.x, colliders[j], c.x);
                // Heat flows through the touching faces (Fourier's law, contact conductance).
                if (ct.depth > -1e-4) {
                    const double q = s.conductance * ct.area * (c.temperature - a.temperature) * h;
                    addHeat(i, q);
                    addHeat(j, -q);
                    a.touching = c.touching = true;
                }
                if (ct.depth <= 0) continue;
                const double wsum = a.invMass + c.invMass;
                if (wsum <= 0) continue;
                const V3 u = ct.normal;
                // Separate them, the lighter one moving more.
                a.x -= u * (ct.depth * a.invMass / wsum);
                c.x += u * (ct.depth * c.invMass / wsum);
                const double closing = dot(a.v - c.v, u);
                // Below this the bodies are pressed together, not colliding: what one step of the
                // force pressing them gives them (strong attraction) plus a small margin.
                const double rest = kRestSpeed + 2.0 * h * std::fabs(dot(a.accel - c.accel, u));
                if (closing > 0) {
                    const double e = closing > rest
                                         ? 0.5 * (run->bodies[i].props.restitution + run->bodies[j].props.restitution)
                                         : 0.0; // resting contacts settle instead of jittering
                    const double J = (1 + e) * closing / wsum; // impulse along the normal
                    a.v -= u * (J * a.invMass);
                    c.v += u * (J * c.invMass);
                    if (a.invMass > 0) a.p = a.v * (run->bodies[i].mass * gammaOf(a.v, s));
                    if (c.invMass > 0) c.p = c.v * (run->bodies[j].mass * gammaOf(c.v, s));
                    if (closing > rest) ++run->collisions;
                }
            }
        floorContacts(false); // bodies pushed into the floor by the ones above them
        // What the contacts took from the motion is heat, shared by the bodies touching.
        const double lost = before - mechanicalTotal();
        if (std::fabs(lost) > 0) {
            std::vector<std::size_t> warmed;
            for (std::size_t i = 0; i < n; ++i)
                if (L[i].touching && L[i].invMass > 0) warmed.push_back(i);
            for (std::size_t i : warmed) addHeat(i, lost / static_cast<double>(warmed.size()));
        }
        // The air cools the bodies (Newton's law of cooling).
        for (std::size_t i = 0; i < n; ++i) {
            const double q = s.convection * run->bodies[i].area * (L[i].temperature - s.ambient) * h;
            addHeat(i, -q);
            acc.toAir += q;
        }
        if (step % kStepsPerFrame == 0) record(static_cast<float>(step * h));
    }
    return run;
}

PhysicsBodyState lerpState(const PhysicsBodyState& a, const PhysicsBodyState& b, float u) {
    PhysicsBodyState s = u < 0.5f ? a : b;
    s.position = a.position + (b.position - a.position) * u;
    s.velocity = a.velocity + (b.velocity - a.velocity) * u;
    s.acceleration = a.acceleration + (b.acceleration - a.acceleration) * u;
    s.force = a.force + (b.force - a.force) * u;
    s.temperature = a.temperature + (b.temperature - a.temperature) * u;
    s.properTime = a.properTime + (b.properTime - a.properTime) * u;
    return s;
}

std::string num(double v, const char* unit) {
    char b[64];
    const double a = std::fabs(v);
    if (a != 0 && (a < 1e-3 || a >= 1e6)) std::snprintf(b, sizeof(b), "%.3e %s", v, unit);
    else std::snprintf(b, sizeof(b), "%.4g %s", v, unit);
    return b;
}
std::string vecNum(const Vec3& v, const char* unit) {
    char b[128];
    std::snprintf(b, sizeof(b), "(%.4g, %.4g, %.4g) %s", static_cast<double>(v.x), static_cast<double>(v.y),
                  static_cast<double>(v.z), unit);
    return b;
}

} // namespace

const char* gravityModeKey(GravityMode m) noexcept {
    switch (m) {
    case GravityMode::Off: return "off";
    case GravityMode::Uniform: return "uniform";
    case GravityMode::Newton: return "newton";
    case GravityMode::Both: return "both";
    }
    return "uniform";
}
const char* waveKindKey(WaveKind k) noexcept {
    switch (k) {
    case WaveKind::Off: return "off";
    case WaveKind::Sound: return "sound";
    case WaveKind::Light: return "light";
    case WaveKind::Custom: return "custom";
    }
    return "off";
}
std::optional<GravityMode> gravityModeFromKey(const std::string& key) noexcept {
    for (GravityMode m : {GravityMode::Off, GravityMode::Uniform, GravityMode::Newton, GravityMode::Both})
        if (key == gravityModeKey(m)) return m;
    return std::nullopt;
}
std::optional<WaveKind> waveKindFromKey(const std::string& key) noexcept {
    for (WaveKind k : {WaveKind::Off, WaveKind::Sound, WaveKind::Light, WaveKind::Custom})
        if (key == waveKindKey(k)) return k;
    return std::nullopt;
}

MassProperties massProperties(const Body& body) {
    MassProperties mp;
    const auto& v = body.mesh.vertices;
    const auto& idx = body.mesh.indices;
    double vol = 0, area = 0;
    V3 c;
    for (std::size_t t = 0; t + 2 < idx.size(); t += 3) {
        const V3 a = v3(v[idx[t]].position + body.offset), b = v3(v[idx[t + 1]].position + body.offset),
                 d = v3(v[idx[t + 2]].position + body.offset);
        const double sv = dot(a, cross(b, d)) / 6.0; // signed tetrahedron with the origin
        vol += sv;
        c += (a + b + d) * (sv / 4.0);
        area += 0.5 * norm(cross(b - a, d - a));
    }
    if (std::fabs(vol) > 1e-9) {
        mp.centre = vec(c * (1.0 / vol));
    } else { // open or flat mesh: the bounds' centre
        mp.centre = body.bounds().center();
    }
    mp.volume = std::fabs(vol) * 1e-9; // mm^3 -> m^3
    mp.area = area * 1e-6;
    mp.mass = body.physics.mass > 0 ? body.physics.mass : mp.volume * body.physics.density;
    return mp;
}

PhysicsFrame PhysicsRun::at(float t) const {
    if (frames.empty()) return {};
    if (t <= 0.0f) return frames.front();
    const float f = t / frameStep;
    const std::size_t i = static_cast<std::size_t>(f);
    if (i + 1 >= frames.size()) return frames.back();
    const float u = f - static_cast<float>(i);
    PhysicsFrame out = u < 0.5f ? frames[i] : frames[i + 1];
    out.t = t;
    for (std::size_t b = 0; b < out.bodies.size(); ++b) out.bodies[b] = lerpState(frames[i].bodies[b], frames[i + 1].bodies[b], u);
    return out;
}

int PhysicsRun::indexOf(std::size_t sceneBody) const noexcept {
    for (std::size_t i = 0; i < bodies.size(); ++i)
        if (bodies[i].body == sceneBody) return static_cast<int>(i);
    return -1;
}

std::shared_ptr<const PhysicsRun> physicsRun(const Scene& scene) {
    if (!scene.physics().enabled) return nullptr;
    bool any = false;
    for (const Body& b : scene.bodies()) any = any || b.physics.enabled;
    if (!any) return nullptr;
    // A few recent runs (several open documents), newest first.
    static std::vector<std::pair<std::uint64_t, std::shared_ptr<const PhysicsRun>>> cache;
    const std::uint64_t key = signature(scene);
    for (std::size_t i = 0; i < cache.size(); ++i)
        if (cache[i].first == key) {
            auto hit = cache[i];
            cache.erase(cache.begin() + static_cast<std::ptrdiff_t>(i));
            cache.insert(cache.begin(), hit);
            return hit.second;
        }
    std::shared_ptr<const PhysicsRun> run = simulate(scene);
    cache.insert(cache.begin(), {key, run});
    if (cache.size() > 4) cache.pop_back();
    return run;
}

std::vector<Mat4> simulationTransforms(const Scene& scene, float t) {
    std::vector<Mat4> tf = motionTransforms(scene, t);
    applyRobotRuns(scene, t, tf);
    const auto run = physicsRun(scene);
    if (!run || run->frames.empty()) {
        applyAttachments(scene, tf);
        return tf;
    }
    const PhysicsFrame f = run->at(t);
    for (std::size_t i = 0; i < run->bodies.size(); ++i) {
        const PhysicsBodyInfo& b = run->bodies[i];
        if (b.props.fixed || b.body >= tf.size()) continue;
        const PhysicsBodyState& st = f.bodies[i];
        Mat4 m = translate(st.position) * st.rotation * translate(b.restCentre * -1.0f);
        if (run->settings.relativity) { // length contraction along the motion, 1/gamma
            const double v = length(st.velocity) * kMm;
            if (v > 1e-9) {
                const double beta2 = std::min(v * v / (run->settings.lightSpeed * run->settings.lightSpeed), 1.0 - 1e-12);
                const float k = static_cast<float>(std::sqrt(1.0 - beta2)) - 1.0f;
                const Vec3 u = normalize(st.velocity);
                Mat4 squash = Mat4::identity(); // I + (1/gamma - 1) u u^T
                const float uu[3] = {u.x, u.y, u.z};
                for (int c = 0; c < 3; ++c)
                    for (int r = 0; r < 3; ++r) squash.m[c][r] += k * uu[r] * uu[c];
                m = translate(st.position) * squash * translate(st.position * -1.0f) * m;
            }
        }
        tf[b.body] = m;
    }
    applyAttachments(scene, tf);
    return tf;
}

std::vector<std::string> physicsReport(const Scene& scene, float t, std::optional<std::size_t> only) {
    std::vector<std::string> out;
    const PhysicsSettings& s = scene.physics();
    if (!s.enabled) {
        out.push_back("Physics is off (PHYSICS ON, or PHYSICS DEMO to load an experiment)");
        return out;
    }
    const auto run = physicsRun(scene);
    if (!run) {
        out.push_back("Physics is on, but no body takes part (PHYSICS BODY name DYNAMIC)");
        return out;
    }
    t = std::min(t, run->frames.back().t); // the run holds its last state
    const PhysicsFrame f = run->at(t), f0 = run->frames.front();
    char line[256];
    std::snprintf(line, sizeof(line), "Physics at t = %.3f s of %.2f s  (%zu bodies, %d collisions in the whole run)",
                  static_cast<double>(t), static_cast<double>(s.duration), run->bodies.size(), run->collisions);
    out.push_back(line);
    std::string world = "  World: gravity " + std::string(gravityModeKey(s.gravity));
    if (s.gravity == GravityMode::Uniform || s.gravity == GravityMode::Both) world += " g " + vecNum(s.g, "m/s^2");
    if (s.gravity == GravityMode::Newton || s.gravity == GravityMode::Both) world += "  G " + num(s.bigG, "m^3/(kg s^2)");
    world += s.floor ? "  floor (mu " + num(s.friction, ")") : "  no floor";
    if (s.air) world += "  air drag";
    if (length(s.electricField) > 0) world += "  E " + vecNum(s.electricField, "V/m");
    if (length(s.magneticField) > 0) world += "  B " + vecNum(s.magneticField, "T");
    world += "  ambient " + num(s.ambient - 273.15, "C");
    if (s.relativity) world += "  relativity c = " + num(s.lightSpeed, "m/s");
    out.push_back(world);
    const PhysicsEnergy& e = f.energy;
    out.push_back("  Momentum (moving bodies): " + vecNum(f.momentum, "kg m/s") + "  |p| " + num(length(f.momentum), "kg m/s") +
                  "  (start " + num(length(f0.momentum), "kg m/s") + ")");
    out.push_back("  Energy: kinetic " + num(e.kinetic, "J") + ", gravity " + num(e.gravity, "J") + ", electric " +
                  num(e.electric, "J") + ", heat in bodies " + num(e.thermal, "J") + ", to the air " + num(e.toAir, "J") +
                  ", work done " + num(e.externalWork, "J"));
    const double b0 = f0.energy.balance(), b1 = e.balance();
    const double scale = std::max({std::fabs(f0.energy.kinetic) + std::fabs(f0.energy.gravity) + std::fabs(f0.energy.electric),
                                   std::fabs(e.kinetic), 1e-12});
    std::snprintf(line, sizeof(line), "  Conservation of energy: total %s (start %s), drift %.4f %%", num(b1, "J").c_str(),
                  num(b0, "J").c_str(), 100.0 * (b1 - b0) / scale);
    out.push_back(line);
    for (std::size_t i = 0; i < run->bodies.size(); ++i) {
        const PhysicsBodyInfo& b = run->bodies[i];
        if (only && *only != b.body) continue;
        const PhysicsBodyState& st = f.bodies[i];
        const double v = length(st.velocity) * kMm, m = b.mass;
        const double gamma = s.relativity ? 1.0 / std::sqrt(std::max(1.0 - v * v / (s.lightSpeed * s.lightSpeed), 1e-15)) : 1.0;
        const double p = gamma * m * v;
        out.push_back("  " + b.name + (b.props.fixed ? "  [fixed]" : "") + "  mass " + num(m, "kg") +
                      (b.props.charge != 0 ? "  charge " + num(b.props.charge, "C") : std::string()));
        out.push_back("    position " + lengthsText({st.position.x, st.position.y, st.position.z}, ", ", "(", ")") +
                      "  velocity " + vecNum(st.velocity * 0.001f, "m/s") + "  speed " + num(v, "m/s"));
        out.push_back("    acceleration " + vecNum(st.acceleration * 0.001f, "m/s^2") + "  net force " + vecNum(st.force, "N") +
                      (st.touching ? "  (in contact)" : ""));
        out.push_back("    momentum " + num(p, "kg m/s") + "  kinetic energy " +
                      num(s.relativity ? (gamma - 1) * m * s.lightSpeed * s.lightSpeed : 0.5 * m * v * v, "J") +
                      "  temperature " + num(st.temperature - 273.15, "C"));
        if (s.relativity)
            out.push_back("    relativity: v/c " + num(v / s.lightSpeed, "") + " gamma " + num(gamma, "") + " proper time " +
                          num(st.properTime, "s") + " (clock " + num(t - st.properTime, "s") + " behind), length x " +
                          num(1.0 / gamma, "") + ", rest energy " + num(m * s.lightSpeed * s.lightSpeed, "J"));
        // Quantum: matter wavelength and the smallest velocity spread its size allows.
        const double size = 2 * b.radius;
        out.push_back("    quantum: de Broglie wavelength " + (p > 0 ? num(kPlanck / p, "m") : std::string("infinite (at rest)")) +
                      ", uncertainty dv >= " + num(kPlanck / (4 * kPi * m * size), "m/s") + " within its own size");
        if (b.props.wave != WaveKind::Off) {
            const double c = b.props.wave == WaveKind::Sound ? kSoundSpeed
                             : b.props.wave == WaveKind::Light ? (s.relativity ? s.lightSpeed : kLightSpeed)
                                                               : b.props.waveSpeed * kMm;
            out.push_back("    wave source (" + std::string(waveKindKey(b.props.wave)) + "): frequency " +
                          num(b.props.waveFrequency, "Hz") + ", speed " + num(c, "m/s") + ", wavelength " +
                          num(c / std::max(b.props.waveFrequency, 1e-6f), "m"));
            for (std::size_t j = 0; j < run->bodies.size(); ++j) {
                if (j == i) continue;
                const PhysicsBodyState& o = f.bodies[j];
                const Vec3 d = o.position - st.position;
                if (length(d) < 1e-6f) continue;
                const Vec3 u = normalize(d);
                const double vs = dot(st.velocity, u) * kMm, vo = dot(o.velocity, u) * kMm;
                double heard;
                if (b.props.wave == WaveKind::Light) { // relativistic Doppler
                    const double cc = c, gs = 1 / std::sqrt(std::max(1 - std::pow(length(st.velocity) * kMm / cc, 2), 1e-15)),
                                 go = 1 / std::sqrt(std::max(1 - std::pow(length(o.velocity) * kMm / cc, 2), 1e-15));
                    heard = b.props.waveFrequency * go * (1 - vo / cc) / (gs * (1 - vs / cc));
                } else {
                    heard = b.props.waveFrequency * (c - vo) / std::max(c - vs, 1e-9);
                }
                out.push_back("      at " + run->bodies[j].name + ": " + num(heard, "Hz") + " (Doppler), distance " +
                              num(length(d) * kMm, "m"));
            }
        }
    }
    return out;
}

// ---------------------------------------------------------------- demos

namespace {

Body ball(const std::string& name, float r, Vec3 at, Vec3 color) {
    Body b = makeBody(name, "sphere", {r}, {at.x, at.y, at.z - r}, color); // sphere rests on z: centre at `at`
    b.physics.enabled = true;
    return b;
}
Body block(const std::string& name, Vec3 lo, Vec3 hi, Vec3 color) {
    Body b = makeBody(name, "box", {lo.x, lo.y, lo.z, hi.x, hi.y, hi.z}, {}, color);
    b.physics.enabled = true;
    return b;
}
constexpr Vec3 kRed{0.86f, 0.30f, 0.26f}, kBlue{0.28f, 0.50f, 0.90f}, kGreen{0.30f, 0.72f, 0.42f}, kGold{0.92f, 0.70f, 0.25f},
    kGrey{0.62f, 0.66f, 0.72f};

} // namespace

const std::vector<PhysicsDemo>& physicsDemos() {
    static const std::vector<PhysicsDemo> d = {
        {"drop", "Drop and bounce", "Three balls fall 1 m: gravity, restitution 0.9 / 0.6 / 0.3, kinetic energy turned into heat"},
        {"projectile", "Projectile", "A ball thrown at 45 deg, 4.5 m/s: a parabola, horizontal speed constant, vertical a = -g"},
        {"collision", "Momentum", "Newton's cradle in space: elastic collisions pass the momentum along, total p conserved"},
        {"orbit", "Orbit", "A moon around a 1e10 kg planet by Newton's gravitation: circular orbit, energy conserved"},
        {"charges", "Electric charges", "Opposite charges attract, like charges repel (Coulomb's law)"},
        {"cyclotron", "Magnetic field", "A charge in a 1 T field circles at r = mv/qB (Lorentz force; no work done)"},
        {"heat", "Heat transfer", "A 100 C block on a 0 C block: conduction to equal temperature, then cooling in the air"},
        {"doppler", "Waves and Doppler", "A moving ripple source: wavefronts bunch up ahead, frequency heard higher in front"},
        {"relativity", "Relativity", "c = 2 m/s: a pushed ball approaches but never reaches c; its clock runs slow"},
    };
    return d;
}

bool loadPhysicsDemo(Scene& scene, const std::string& key) {
    const auto& list = physicsDemos();
    if (std::none_of(list.begin(), list.end(), [&](const PhysicsDemo& d) { return key == d.key; })) return false;
    scene.clear();
    PhysicsSettings& s = scene.physics();
    s = PhysicsSettings{};
    s.enabled = true;
    if (key == "drop") {
        const float e[] = {0.9f, 0.6f, 0.3f};
        const Vec3 c[] = {kGreen, kBlue, kRed};
        for (int i = 0; i < 3; ++i) {
            Body b = ball("Ball e" + std::to_string(static_cast<int>(e[i] * 10)), 50, {-200.0f + 200.0f * static_cast<float>(i), 0, 1050}, c[i]);
            b.physics.restitution = e[i];
            b.physics.density = 1200;
            scene.add(std::move(b));
        }
        s.duration = 6;
    } else if (key == "projectile") {
        Body b = ball("Ball", 40, {-1200, 0, 40}, kGold);
        b.physics.velocity = {3182, 0, 3182}; // 4.5 m/s at 45 deg
        b.physics.restitution = 0.5f;
        scene.add(std::move(b));
        s.duration = 3;
    } else if (key == "collision") {
        s.gravity = GravityMode::Off;
        s.floor = false;
        Body striker = ball("Striker", 50, {-600, 0, 300}, kRed);
        striker.physics.velocity = {800, 0, 0};
        striker.physics.restitution = 1;
        scene.add(std::move(striker));
        for (int i = 0; i < 4; ++i) {
            Body b = ball("Ball " + std::to_string(i + 1), 50, {static_cast<float>(i) * 100.0f, 0, 300}, kGrey);
            b.physics.restitution = 1;
            scene.add(std::move(b));
        }
        s.duration = 4;
    } else if (key == "orbit") {
        s.gravity = GravityMode::Newton;
        s.floor = false;
        Body planet = ball("Planet", 150, {0, 0, 600}, kBlue);
        planet.physics.mass = 1e10f;
        planet.physics.fixed = true;
        scene.add(std::move(planet));
        // Circular orbit: v = sqrt(G M / r).
        const float r = 0.6f, v = static_cast<float>(std::sqrt(kGravitationalConstant * 1e10 / r));
        Body moon = ball("Moon", 30, {r * 1000.0f, 0, 600}, kGrey);
        moon.physics.mass = 1;
        moon.physics.velocity = {0, v * 1000.0f, 0};
        scene.add(std::move(moon));
        s.duration = 12;
    } else if (key == "charges") {
        s.gravity = GravityMode::Off;
        s.floor = false;
        const struct {
            const char* name;
            float x;
            double q;
            Vec3 c;
        } q[] = {{"Plus", -300, 5e-6, kRed}, {"Minus", 300, -5e-6, kBlue}, {"Plus 2", 0, 5e-6, kRed}};
        for (int i = 0; i < 3; ++i) {
            Body b = ball(q[i].name, 40, {q[i].x, i == 2 ? 500.0f : 0.0f, 400}, q[i].c);
            b.physics.mass = 0.05f;
            b.physics.charge = q[i].q;
            b.physics.restitution = 0.2f;
            scene.add(std::move(b));
        }
        s.duration = 5;
    } else if (key == "cyclotron") {
        s.gravity = GravityMode::Off;
        s.floor = false;
        s.magneticField = {0, 0, 1};
        Body b = ball("Charge", 25, {0, -200, 400}, kGold);
        b.physics.mass = 0.01f;
        b.physics.charge = 0.05;
        b.physics.velocity = {1000, 0, 0}; // r = mv/qB = 0.2 m
        scene.add(std::move(b));
        s.duration = 2.6f; // a little over two turns (T = 2 pi m / qB = 1.26 s)
    } else if (key == "heat") {
        Body cold = block("Cold block", {-150, -150, 0}, {150, 150, 100}, kBlue);
        cold.physics.temperature = 273.15f;
        cold.physics.density = 2700;
        Body hot = block("Hot block", {-150, -150, 100}, {150, 150, 200}, kRed);
        hot.physics.temperature = 373.15f;
        hot.physics.density = 2700;
        scene.add(std::move(cold));
        scene.add(std::move(hot));
        s.conductance = 20000;
        s.duration = 60;
    } else if (key == "doppler") {
        s.gravity = GravityMode::Off;
        s.floor = false;
        Body src = ball("Source", 40, {-1000, 0, 300}, kGold);
        src.physics.velocity = {400, 0, 0};
        src.physics.wave = WaveKind::Custom;
        src.physics.waveFrequency = 3;
        src.physics.waveSpeed = 800;
        scene.add(std::move(src));
        Body front = ball("Listener ahead", 40, {1200, 250, 300}, kGreen);
        front.physics.fixed = true;
        Body back = ball("Listener behind", 40, {-1400, 250, 300}, kBlue);
        back.physics.fixed = true;
        scene.add(std::move(front));
        scene.add(std::move(back));
        s.duration = 5;
    } else if (key == "relativity") {
        s.gravity = GravityMode::Off;
        s.floor = false;
        s.relativity = true;
        s.lightSpeed = 2.0;
        Body b = ball("Rocket", 60, {-1500, 0, 300}, kRed);
        b.physics.mass = 1;
        b.physics.force = {2, 0, 0}; // would reach 2 m/s in 1 s without relativity
        scene.add(std::move(b));
        Body clock = ball("Clock at rest", 60, {-1500, 400, 300}, kGrey);
        clock.physics.mass = 1;
        scene.add(std::move(clock));
        s.duration = 5;
    }
    return true;
}

} // namespace sicdm

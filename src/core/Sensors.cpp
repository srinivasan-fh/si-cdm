#include "core/Sensors.hpp"

#include "core/Bvh.hpp"
#include "core/FileIO.hpp"
#include "core/ImageWrite.hpp"
#include "core/Multibody.hpp"
#include "core/Physics.hpp"
#include "core/Scene.hpp"
#include "core/Terrain.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>

namespace sicdm {

namespace {

constexpr double kPiS = 3.14159265358979323846;
constexpr double kEarthRadius = 6378137.0; // m (WGS 84 equatorial)

// --- deterministic noise ------------------------------------------------------------------

std::uint64_t mixBits(std::uint64_t h) {
    h ^= h >> 33;
    h *= 0xff51afd7ed558ccdull;
    h ^= h >> 33;
    h *= 0xc4ceb9fe1a85ec53ull;
    return h ^ (h >> 33);
}

// A standard normal draw from (seed, channel, sample).
double gaussian(int seed, int channel, std::int64_t sample) {
    const std::uint64_t h1 = mixBits(static_cast<std::uint64_t>(seed) * 0x9E3779B97F4A7C15ull ^ static_cast<std::uint64_t>(channel) * 0xBF58476D1CE4E5B9ull ^
                                     static_cast<std::uint64_t>(sample));
    const std::uint64_t h2 = mixBits(h1 + 0x632BE59BD9B4E019ull);
    const double u1 = (static_cast<double>(h1 >> 11) + 1.0) / 9007199254740993.0; // (0, 1]
    const double u2 = static_cast<double>(h2 >> 11) / 9007199254740992.0;
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * kPiS * u2);
}

// The value of a bias random walk (sigma per root second) at sample k of a sensor sampled at `rate`.
double biasWalk(int seed, int channel, std::int64_t k, double rate, double sigma) {
    if (sigma <= 0 || k <= 0) return 0;
    double b = 0;
    const double step = sigma / std::sqrt(rate);
    for (std::int64_t i = 0; i < k; ++i) b += step * gaussian(seed + 7919, channel, i);
    return b;
}

// --- sensor poses -------------------------------------------------------------------------

struct Frame {
    V3d p{};                       // m, world
    std::array<double, 9> R{};     // columns: the sensor's x (look), y, z (up), in world
};

int bodyIndex(const Scene& scene, const std::string& name) {
    for (std::size_t i = 0; i < scene.bodies().size(); ++i)
        if (scene.bodies()[i].name == name) return static_cast<int>(i);
    return -1;
}

// The sensor's frame at rest, from its look and up directions.
std::array<Vec3, 3> restAxes(const SensorSpec& s) {
    const Vec3 x = normalize(length(s.look) > 0 ? s.look : Vec3{1, 0, 0});
    Vec3 z = s.up - x * dot(s.up, x);
    if (length(z) < 1e-6f) z = std::fabs(x.z) < 0.9f ? Vec3{0, 0, 1} - x * x.z : Vec3{0, 1, 0} - x * x.y;
    z = normalize(z);
    return {x, cross(z, x), z};
}

Frame sensorFrame(const SensorSpec& s, const Mat4& T) {
    Frame f;
    const Vec3 p = T.transformPoint(s.at);
    f.p = {p.x * 1e-3, p.y * 1e-3, p.z * 1e-3};
    const auto ax = restAxes(s);
    for (int c = 0; c < 3; ++c) {
        const Vec3 d = normalize(T.transformDir(ax[static_cast<std::size_t>(c)]));
        f.R[static_cast<std::size_t>(c)] = d.x, f.R[static_cast<std::size_t>(3 + c)] = d.y, f.R[static_cast<std::size_t>(6 + c)] = d.z;
    }
    return f;
}

bool frameAt(const Scene& scene, const SensorSpec& s, double t, Frame& f, std::vector<Mat4>* transforms = nullptr) {
    const int b = bodyIndex(scene, s.body);
    if (b < 0) return false;
    std::vector<Mat4> tf = simulationTransforms(scene, static_cast<float>(std::max(t, 0.0)));
    f = sensorFrame(s, tf[static_cast<std::size_t>(b)]);
    if (transforms) *transforms = std::move(tf);
    return true;
}

V3d toSensor(const Frame& f, const V3d& w) { // R^T w
    return {f.R[0] * w[0] + f.R[3] * w[1] + f.R[6] * w[2], f.R[1] * w[0] + f.R[4] * w[1] + f.R[7] * w[2], f.R[2] * w[0] + f.R[5] * w[1] + f.R[8] * w[2]};
}
V3d fromSensor(const Frame& f, const V3d& s) { // R s
    return {f.R[0] * s[0] + f.R[1] * s[1] + f.R[2] * s[2], f.R[3] * s[0] + f.R[4] * s[1] + f.R[5] * s[2], f.R[6] * s[0] + f.R[7] * s[1] + f.R[8] * s[2]};
}

// Angular velocity (world) taking orientation A to B in time dt.
V3d angularVelocity(const std::array<double, 9>& A, const std::array<double, 9>& B, double dt) {
    // D = B A^T; its axis-angle over dt.
    double D[9];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            double s = 0;
            for (int k = 0; k < 3; ++k) s += B[static_cast<std::size_t>(i * 3 + k)] * A[static_cast<std::size_t>(j * 3 + k)];
            D[i * 3 + j] = s;
        }
    const double c = std::clamp((D[0] + D[4] + D[8] - 1.0) * 0.5, -1.0, 1.0);
    const double angle = std::acos(c);
    const V3d axis{D[7] - D[5], D[2] - D[6], D[3] - D[1]};
    const double n = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    if (n < 1e-12 || angle < 1e-12) return {0, 0, 0};
    const double k = angle / (n * dt);
    return {axis[0] * k, axis[1] * k, axis[2] * k};
}

// --- raycasting the posed scene -------------------------------------------------------------

struct Hit {
    int body = -1; // -2 the floor
    double t = 0;
    Vec3 normal{0, 0, 1};
};

std::shared_ptr<const MeshBvh> bvhOf(const MeshData& m) {
    static std::mutex lock;
    static std::map<std::uint64_t, std::shared_ptr<const MeshBvh>> cache;
    const std::uint64_t key = meshContentKey(m);
    std::lock_guard<std::mutex> g(lock);
    if (auto it = cache.find(key); it != cache.end()) return it->second;
    auto b = std::make_shared<const MeshBvh>(m);
    if (cache.size() > 512) cache.clear();
    cache[key] = b;
    return b;
}

struct RayWorld {
    const Scene& scene;
    const std::vector<Mat4>& tf;
    int skip = -1; // the sensor's own part: it looks out of it, not at its inside
    std::vector<Mat4> inv;
    std::vector<AABB> boxes;
    std::vector<std::shared_ptr<const MeshBvh>> bvh;
    bool terrain = false;

    RayWorld(const Scene& s, const std::vector<Mat4>& transforms) : scene(s), tf(transforms) {
        const auto& bodies = scene.bodies();
        inv.resize(bodies.size());
        boxes.resize(bodies.size());
        bvh.resize(bodies.size());
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            if (!bodies[i].visible || bodies[i].mesh.indices.empty()) continue;
            inv[i] = inverse(tf[i]);
            bvh[i] = bvhOf(bodies[i].mesh);
            AABB b = bvh[i]->bounds();
            b.min += bodies[i].offset, b.max += bodies[i].offset;
            boxes[i] = b;
            terrain = terrain || bodies[i].name == kTerrainBody;
        }
    }

    // Nearest hit within maxT (mm along a unit direction).
    Hit cast(const Vec3& origin, const Vec3& dir, double maxT) const {
        Hit best;
        double bestT = maxT;
        const auto& bodies = scene.bodies();
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            if (!bvh[i] || static_cast<int>(i) == skip) continue;
            const Ray r{inv[i].transformPoint(origin), inv[i].transformDir(dir)};
            float tBox = 0;
            if (!intersectAABB(r, boxes[i], tBox) || tBox > bestT) continue;
            const Ray local{r.origin - bodies[i].offset, r.dir};
            float t = static_cast<float>(bestT);
            std::uint32_t tri = 0;
            if (bvh[i]->raycast(local, t, &tri) && t < bestT) {
                bestT = t;
                best.body = static_cast<int>(i);
                best.t = t;
                const auto& m = bodies[i].mesh;
                const Vec3 a = m.vertices[m.indices[tri * 3]].position, b = m.vertices[m.indices[tri * 3 + 1]].position,
                           c = m.vertices[m.indices[tri * 3 + 2]].position;
                Vec3 n = normalize(tf[i].transformDir(cross(b - a, c - a)));
                if (dot(n, dir) > 0) n = n * -1.0f; // seen from the ray's side
                best.normal = n;
            }
        }
        // The floor at z = 0 (under any terrain's patch too).
        if (dir.z < -1e-9f) {
            const double t = -origin.z / dir.z;
            if (t > 0 && t < bestT) {
                best = Hit{-2, t, {0, 0, 1}};
            }
        }
        return best;
    }
};

std::uint8_t clampByte(double v) { return static_cast<std::uint8_t>(std::clamp(v, 0.0, 255.0)); }

// --- the sensors ------------------------------------------------------------------------------

bool readImu(const Scene& scene, const SensorSpec& s, double t, SensorReading& out) {
    const double h = 0.01;
    const double t0 = std::max(0.0, t - h), t1 = t0 + h, t2 = t0 + 2 * h; // centred, or forward at 0
    Frame a, b, c;
    if (!frameAt(scene, s, t0, a) || !frameAt(scene, s, t1, b) || !frameAt(scene, s, t2, c)) return false;
    V3d acc{(c.p[0] - 2 * b.p[0] + a.p[0]) / (h * h), (c.p[1] - 2 * b.p[1] + a.p[1]) / (h * h), (c.p[2] - 2 * b.p[2] + a.p[2]) / (h * h)};
    const Vec3 g = scene.physics().gravity == GravityMode::Off ? Vec3{} : scene.physics().g;
    const V3d specific{acc[0] - g.x, acc[1] - g.y, acc[2] - g.z};
    const V3d w = angularVelocity(a.R, c.R, 2 * h);
    const V3d fs = toSensor(b, specific), ws = toSensor(b, w);
    const std::int64_t k = static_cast<std::int64_t>(std::llround(t * s.rate));
    out.labels = {"ax", "ay", "az", "wx", "wy", "wz"};
    out.values.clear();
    for (int i = 0; i < 6; ++i) {
        const SensorNoise n = typicalNoise(SensorKind::Imu, i);
        const double truth = i < 3 ? fs[static_cast<std::size_t>(i)] : ws[static_cast<std::size_t>(i - 3)];
        out.values.push_back(truth + s.noise * n.white * gaussian(s.seed, i, k) + biasWalk(s.seed, i, k, s.rate, s.bias * n.walk));
    }
    return true;
}

bool readGps(const Scene& scene, const SensorSpec& s, double t, SensorReading& out) {
    const double h = 0.01;
    Frame a, b;
    if (!frameAt(scene, s, std::max(0.0, t - h), a) || !frameAt(scene, s, std::max(0.0, t - h) + h, b)) return false;
    Frame now;
    frameAt(scene, s, t, now);
    const std::int64_t k = static_cast<std::int64_t>(std::llround(t * s.rate));
    double v[6] = {now.p[0], now.p[1], now.p[2], (b.p[0] - a.p[0]) / h, (b.p[1] - a.p[1]) / h, (b.p[2] - a.p[2]) / h};
    for (int i = 0; i < 6; ++i) v[i] += s.noise * typicalNoise(SensorKind::Gps, i).white * gaussian(s.seed, i, k);
    const double lat = s.lat + v[1] / kEarthRadius * 180.0 / kPiS;
    const double lon = s.lon + v[0] / (kEarthRadius * std::cos(s.lat * kPiS / 180.0)) * 180.0 / kPiS;
    out.labels = {"x", "y", "z", "vx", "vy", "vz", "lat", "lon", "alt"};
    out.values = {v[0], v[1], v[2], v[3], v[4], v[5], lat, lon, s.alt + v[2]};
    return true;
}

bool readForceTorque(const Scene& scene, const RigSettings& robot, const SensorSpec& s, double t, SensorReading& out, std::string* error) {
    const float duration = scene.physics().duration > 0 ? scene.physics().duration : 10.0f;
    const auto run = robot.simulate ? robotRun(scene, robot, std::max(duration, static_cast<float>(t))) : nullptr;
    if (!run || run->frames.empty()) {
        if (error) *error = "A force-torque sensor reads the robot's simulation: RIG SIMULATE ON (and an articulated or vehicle engine)";
        return false;
    }
    const int b = bodyIndex(scene, s.body);
    const int link = b < 0 ? -1 : run->rig.linkOf(static_cast<std::size_t>(b));
    if (link <= 0) {
        if (error) *error = "A force-torque sensor goes on a part below a joint (its joint carries the load); " + s.body + " is on the base";
        return false;
    }
    const std::size_t j = static_cast<std::size_t>(run->rig.links[static_cast<std::size_t>(link)].parentJoint);
    const RobotRunFrame f = run->at(static_cast<float>(t));
    if (j >= f.wrench.size()) return false;
    const auto& w = f.wrench[j];
    const V3d F{w[0], w[1], w[2]}, tauO{w[3], w[4], w[5]};
    // The joint's axis point now, and the sensor's.
    const RigJoint& J = run->rig.joints[j];
    const V3d o = f.links[static_cast<std::size_t>(J.parent)].apply(J.origin);
    Frame fr;
    frameAt(scene, s, t, fr);
    const V3d d{o[0] - fr.p[0], o[1] - fr.p[1], o[2] - fr.p[2]};
    const V3d tauS{tauO[0] + d[1] * F[2] - d[2] * F[1], tauO[1] + d[2] * F[0] - d[0] * F[2], tauO[2] + d[0] * F[1] - d[1] * F[0]};
    const V3d Fs = toSensor(fr, F), Ts = toSensor(fr, tauS);
    const std::int64_t k = static_cast<std::int64_t>(std::llround(t * s.rate));
    out.labels = {"fx", "fy", "fz", "tx", "ty", "tz"};
    out.values.clear();
    for (int i = 0; i < 6; ++i) {
        const double truth = i < 3 ? Fs[static_cast<std::size_t>(i)] : Ts[static_cast<std::size_t>(i - 3)];
        out.values.push_back(truth + s.noise * typicalNoise(SensorKind::ForceTorque, i).white * gaussian(s.seed, i, k));
    }
    return true;
}

bool readCamera(const Scene& scene, const RigSettings& robot, const SensorSpec& s, double t, SensorReading& out) {
    Frame f;
    std::vector<Mat4> tf;
    if (!frameAt(scene, s, t, f, &tf)) return false;
    RayWorld world(scene, tf);
    world.skip = bodyIndex(scene, s.body);
    // Parts of the robot (for the semantic classes).
    std::vector<char> isRobot(scene.bodies().size(), 0);
    RobotRig rig;
    // A named robot is all its parts; the scene as a robot ("") only its jointed links (its
    // base is whatever stands around).
    if (buildRig(scene, robot, rig))
        for (std::size_t l = robot.name.empty() ? 1 : 0; l < rig.links.size(); ++l)
            for (std::size_t b : rig.links[l].bodies) isRobot[b] = 1;
    const int W = std::clamp(s.width, 8, 4096), H = std::clamp(s.height, 8, 4096);
    out.width = W, out.height = H;
    out.rgb.assign(static_cast<std::size_t>(W * H * 3), 0);
    out.depth.assign(static_cast<std::size_t>(W * H), 0.0f);
    out.instance.assign(static_cast<std::size_t>(W * H), -1);
    out.semantic.assign(static_cast<std::size_t>(W * H), 0);
    const double focal = (W * 0.5) / std::tan(std::clamp(static_cast<double>(s.fov), 1.0, 170.0) * kPiS / 360.0);
    const V3d fwd = fromSensor(f, {1, 0, 0}), left = fromSensor(f, {0, 1, 0}), up = fromSensor(f, {0, 0, 1});
    const Vec3 origin{static_cast<float>(f.p[0] * 1e3), static_cast<float>(f.p[1] * 1e3), static_cast<float>(f.p[2] * 1e3)};
    const Vec3 light = normalize(Vec3{0.3f, 0.4f, 0.85f});
    const double maxRange = std::max(0.1f, s.range) * 1e3;
    const std::int64_t k = static_cast<std::int64_t>(std::llround(t * s.rate));
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) {
            const double u = (x + 0.5 - W * 0.5), v = (y + 0.5 - H * 0.5);
            V3d d{fwd[0] * focal - left[0] * u - up[0] * v, fwd[1] * focal - left[1] * u - up[1] * v, fwd[2] * focal - left[2] * u - up[2] * v};
            const double n = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
            const Vec3 dir{static_cast<float>(d[0] / n), static_cast<float>(d[1] / n), static_cast<float>(d[2] / n)};
            const Hit h = world.cast(origin, dir, maxRange);
            const std::size_t px = static_cast<std::size_t>(y * W + x);
            double r, g, b;
            if (h.body == -1) { // sky: lighter toward the horizon
                const double up01 = std::clamp(0.5 + 0.5 * dir.z, 0.0, 1.0);
                r = 150 + 60 * (1 - up01), g = 185 + 40 * (1 - up01), b = 235;
                out.instance[px] = -1;
                out.semantic[px] = 0;
            } else {
                Vec3 base;
                if (h.body == -2) {
                    const Vec3 p = origin + dir * static_cast<float>(h.t);
                    const bool check = (static_cast<long>(std::floor(p.x / 500.0)) + static_cast<long>(std::floor(p.y / 500.0))) % 2 == 0;
                    base = check ? Vec3{0.55f, 0.56f, 0.58f} : Vec3{0.47f, 0.48f, 0.50f};
                    out.instance[px] = -2;
                    out.semantic[px] = 1;
                } else {
                    const Body& body = scene.bodies()[static_cast<std::size_t>(h.body)];
                    base = body.color;
                    out.instance[px] = h.body;
                    out.semantic[px] = body.name == kTerrainBody ? 1 : isRobot[static_cast<std::size_t>(h.body)] ? 2 : 3;
                }
                // Key light from above, plus some from the camera so faces turned to it read.
                const double shade = 0.25 + 0.5 * std::max(0.0f, dot(h.normal, light)) + 0.3 * std::fabs(dot(h.normal, dir));
                r = base.x * 255 * shade, g = base.y * 255 * shade, b = base.z * 255 * shade;
                // Depth along the view axis, with a structured-light-like error growing as z^2.
                double z = h.t * 1e-3 * (dir.x * fwd[0] + dir.y * fwd[1] + dir.z * fwd[2]);
                z += s.noise * 0.001 * z * z * gaussian(s.seed, 100 + static_cast<int>(px % 997), k * 1000003 + static_cast<std::int64_t>(px));
                out.depth[px] = static_cast<float>(std::max(z, 0.0));
            }
            out.rgb[px * 3] = clampByte(r), out.rgb[px * 3 + 1] = clampByte(g), out.rgb[px * 3 + 2] = clampByte(b);
        }
    return true;
}

bool readLidar(const Scene& scene, const SensorSpec& s, double t, SensorReading& out) {
    Frame f;
    std::vector<Mat4> tf;
    if (!frameAt(scene, s, t, f, &tf)) return false;
    RayWorld world(scene, tf);
    world.skip = bodyIndex(scene, s.body);
    const Vec3 origin{static_cast<float>(f.p[0] * 1e3), static_cast<float>(f.p[1] * 1e3), static_cast<float>(f.p[2] * 1e3)};
    const int rings = std::clamp(s.channels, 1, 256), samples = std::clamp(s.samples, 1, 8192);
    const double maxRange = std::max(0.1f, s.range) * 1e3;
    const std::int64_t k = static_cast<std::int64_t>(std::llround(t * s.rate));
    out.points.clear();
    out.ranges.clear();
    for (int ring = 0; ring < rings; ++ring) {
        const double el = (rings == 1 ? 0.0 : (-0.5 + ring / static_cast<double>(rings - 1)) * s.vfov) * kPiS / 180.0;
        for (int i = 0; i < samples; ++i) {
            const double az = 2 * kPiS * i / samples;
            const V3d ds{std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el)};
            const V3d dw = fromSensor(f, ds);
            const Vec3 dir{static_cast<float>(dw[0]), static_cast<float>(dw[1]), static_cast<float>(dw[2])};
            const Hit h = world.cast(origin, dir, maxRange);
            if (h.body == -1) continue; // nothing in range
            double range = h.t * 1e-3 + s.noise * typicalNoise(SensorKind::Lidar, 0).white * gaussian(s.seed, ring, k * 100003 + i);
            range = std::max(range, 0.0);
            out.ranges.push_back(static_cast<float>(range));
            out.points.push_back({static_cast<float>(ds[0] * range), static_cast<float>(ds[1] * range), static_cast<float>(ds[2] * range)});
        }
    }
    return true;
}

} // namespace

SensorNoise typicalNoise(SensorKind kind, int value) {
    switch (kind) {
    case SensorKind::Imu: return value < 3 ? SensorNoise{0.02, 0.001} : SensorNoise{0.001, 0.0001}; // m/s^2, rad/s
    case SensorKind::Gps: return value < 2 ? SensorNoise{0.5, 0} : value == 2 ? SensorNoise{1.0, 0} : SensorNoise{0.05, 0};
    case SensorKind::ForceTorque: return value < 3 ? SensorNoise{0.1, 0} : SensorNoise{0.005, 0};
    case SensorKind::Camera: return {0.001, 0};
    case SensorKind::Lidar: return {0.01, 0};
    }
    return {};
}

bool readSensor(const Scene& scene, const RigSettings& robot, const SensorSpec& s, float t, SensorReading& out, std::string* error) {
    out = {};
    out.kind = s.kind;
    out.t = t;
    if (bodyIndex(scene, s.body) < 0) {
        if (error) *error = "The sensor " + s.name + " is on " + s.body + ", which is gone";
        return false;
    }
    bool ok = false;
    switch (s.kind) {
    case SensorKind::Imu: ok = readImu(scene, s, t, out); break;
    case SensorKind::Gps: ok = readGps(scene, s, t, out); break;
    case SensorKind::ForceTorque: return readForceTorque(scene, robot, s, t, out, error);
    case SensorKind::Camera: ok = readCamera(scene, robot, s, t, out); break;
    case SensorKind::Lidar: ok = readLidar(scene, s, t, out); break;
    }
    if (!ok && error && error->empty()) *error = "Can't read " + s.name;
    return ok;
}

std::string describeReading(const SensorReading& r) {
    char buf[512];
    if (r.kind == SensorKind::Camera) {
        int hits = 0;
        float nearest = 1e9f;
        for (float d : r.depth)
            if (d > 0) ++hits, nearest = std::min(nearest, d);
        std::snprintf(buf, sizeof buf, "%d x %d image at %.3f s: %d%% of it sees something%s", r.width, r.height, r.t,
                      r.width * r.height > 0 ? 100 * hits / (r.width * r.height) : 0, hits ? (", nearest " + std::to_string(nearest).substr(0, 5) + " m").c_str() : "");
        return buf;
    }
    if (r.kind == SensorKind::Lidar) {
        float nearest = 1e9f;
        for (float d : r.ranges) nearest = std::min(nearest, d);
        std::snprintf(buf, sizeof buf, "%zu points at %.3f s%s", r.points.size(), r.t, r.ranges.empty() ? "" : (", nearest " + std::to_string(nearest).substr(0, 5) + " m").c_str());
        return buf;
    }
    std::string s;
    std::snprintf(buf, sizeof buf, "%.3f s:", r.t);
    s = buf;
    for (std::size_t i = 0; i < r.values.size(); ++i) {
        const bool deg = r.labels[i] == "lat" || r.labels[i] == "lon";
        std::snprintf(buf, sizeof buf, deg ? " %s %.7f" : " %s %.4g", r.labels[i].c_str(), r.values[i]);
        s += buf;
    }
    return s;
}

std::vector<std::string> captureSensor(const Scene& scene, const RigSettings& robot, const SensorSpec& s, float t, const std::string& path, std::string* error) {
    std::vector<std::string> files;
    SensorReading r;
    if (!readSensor(scene, robot, s, t, r, error)) return files;
    namespace fs = std::filesystem;
    if (s.kind == SensorKind::Camera) {
        fs::path p(path);
        const std::string stem = (p.parent_path() / p.stem()).string();
        const std::string rgb = stem + "_rgb.png", depth = stem + "_depth.png", seg = stem + "_segmentation.png";
        if (!writePng8(rgb, r.width, r.height, 3, r.rgb, error)) return files;
        files.push_back(rgb);
        std::vector<std::uint16_t> mm(r.depth.size());
        for (std::size_t i = 0; i < mm.size(); ++i) mm[i] = static_cast<std::uint16_t>(std::clamp(r.depth[i] * 1000.0f + 0.5f, 0.0f, 65535.0f));
        if (!writePng16(depth, r.width, r.height, 1, mm, error)) return files;
        files.push_back(depth);
        // Segmentation: a colour per part (the floor grey, the sky black).
        std::vector<std::uint8_t> col(r.instance.size() * 3, 0);
        for (std::size_t i = 0; i < r.instance.size(); ++i) {
            const int id = r.instance[i];
            if (id == -1) continue;
            if (id == -2) {
                col[i * 3] = col[i * 3 + 1] = col[i * 3 + 2] = 90;
                continue;
            }
            const std::uint64_t h = mixBits(static_cast<std::uint64_t>(id) + 1);
            col[i * 3] = static_cast<std::uint8_t>(64 + (h & 0xBF)), col[i * 3 + 1] = static_cast<std::uint8_t>(64 + ((h >> 8) & 0xBF)),
                    col[i * 3 + 2] = static_cast<std::uint8_t>(64 + ((h >> 16) & 0xBF));
        }
        if (!writePng8(seg, r.width, r.height, 3, col, error)) return files;
        files.push_back(seg);
        return files;
    }
    if (s.kind == SensorKind::Lidar) {
        std::string ext = fs::path(path).extension().string();
        for (char& c : ext) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        std::string data;
        char line[128];
        if (ext == ".pcd") {
            std::snprintf(line, sizeof line, "# .PCD v0.7 - written by SI CDM\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH %zu\nHEIGHT 1\n",
                          r.points.size());
            data = line;
            std::snprintf(line, sizeof line, "VIEWPOINT 0 0 0 1 0 0 0\nPOINTS %zu\nDATA ascii\n", r.points.size());
            data += line;
            for (const auto& p : r.points) {
                std::snprintf(line, sizeof line, "%.4f %.4f %.4f\n", p[0], p[1], p[2]);
                data += line;
            }
        } else {
            data = "x,y,z,range\n";
            for (std::size_t i = 0; i < r.points.size(); ++i) {
                std::snprintf(line, sizeof line, "%.4f,%.4f,%.4f,%.4f\n", r.points[i][0], r.points[i][1], r.points[i][2], r.ranges[i]);
                data += line;
            }
        }
        if (writeFile(path, data, error)) files.push_back(path);
        return files;
    }
    if (error) *error = "CAPTURE is for cameras and LiDARs; LOG writes the readings of " + std::string(sensorKindKey(s.kind)) + " sensors";
    return files;
}

bool logSensor(const Scene& scene, const RigSettings& robot, const SensorSpec& s, float from, float to, const std::string& path, std::string* error) {
    if (s.kind == SensorKind::Camera || s.kind == SensorKind::Lidar) {
        if (error) *error = "LOG writes IMU, GPS and force-torque readings; CAPTURE a camera or LiDAR";
        return false;
    }
    const double rate = std::max(0.1f, s.rate);
    std::string data;
    bool header = false;
    char cell[64];
    for (double t = from; t <= to + 1e-9; t += 1.0 / rate) {
        SensorReading r;
        if (!readSensor(scene, robot, s, static_cast<float>(t), r, error)) return false;
        if (!header) {
            data = "t";
            for (const std::string& l : r.labels) data += "," + l;
            data += "\n";
            header = true;
        }
        std::snprintf(cell, sizeof cell, "%.4f", t);
        data += cell;
        for (double v : r.values) {
            std::snprintf(cell, sizeof cell, ",%.9g", v);
            data += cell;
        }
        data += "\n";
    }
    return writeFile(path, data, error);
}

} // namespace sicdm

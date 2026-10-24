// Lathe CAM: the part's turned profile, turning / boring / grooving / drilling /
// threading / parting paths, the turned-stock simulation and lathe posts.

#include "core/Lathe.hpp"

#include "core/Cam.hpp"
#include "core/Geometry.hpp"
#include "core/Json.hpp"
#include "core/Scene.hpp"
#include "core/Sculpt.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <functional>
#include <mutex>
#include <sstream>
#include <thread>

namespace sicdm {

namespace {

constexpr double kPiL = 3.14159265358979323846;
constexpr double kInfL = std::numeric_limits<double>::infinity();

// Runs f(i) for i in [0, n) on a few threads.
void parallelFor(int n, const std::function<void(int)>& f) {
    const int threads = std::clamp(static_cast<int>(std::thread::hardware_concurrency()), 1, 8);
    if (n < 256 || threads == 1) {
        for (int i = 0; i < n; ++i) f(i);
        return;
    }
    std::vector<std::thread> pool;
    for (int t = 0; t < threads; ++t)
        pool.emplace_back([&, t] {
            for (int i = t; i < n; i += threads) f(i);
        });
    for (auto& th : pool) th.join();
}

std::string fmt(const char* f, double v) {
    char b[64];
    std::snprintf(b, sizeof b, f, v);
    return b;
}

} // namespace

// --- tools ------------------------------------------------------------------------------

const char* latheToolTypeKey(LatheToolType t) {
    switch (t) {
    case LatheToolType::Turn: return "turn";
    case LatheToolType::Groove: return "groove";
    case LatheToolType::Thread: return "thread";
    case LatheToolType::Drill: return "drill";
    case LatheToolType::Bore: return "bore";
    }
    return "turn";
}

bool latheToolTypeFromKey(const std::string& key, LatheToolType* out) {
    static const std::pair<const char*, LatheToolType> kKeys[] = {
        {"turn", LatheToolType::Turn},     {"turning", LatheToolType::Turn}, {"groove", LatheToolType::Groove},
        {"grooving", LatheToolType::Groove}, {"parting", LatheToolType::Groove}, {"part", LatheToolType::Groove},
        {"thread", LatheToolType::Thread}, {"threading", LatheToolType::Thread}, {"drill", LatheToolType::Drill},
        {"bore", LatheToolType::Bore},     {"boring", LatheToolType::Bore},  {"bar", LatheToolType::Bore}};
    for (const auto& [k, t] : kKeys)
        if (key == k) {
            *out = t;
            return true;
        }
    return false;
}

double LatheTool::relief() const noexcept {
    // A turning insert leads at 93 degrees (95 for the strong 80 degree ones): what is left of
    // 180 behind its trailing edge, less 2 degrees of clearance, it can go down at.
    if (type != LatheToolType::Turn && type != LatheToolType::Bore) return 0.0;
    const double lead = insertAngle >= 75.0 ? 95.0 : 93.0;
    return std::clamp(180.0 - lead - insertAngle - 2.0, 0.0, 60.0);
}

std::string LatheTool::describe() const {
    std::string s = "T" + std::to_string(number) + "  ";
    auto mm = [](double v) { return fmt("%g", v) + " mm"; };
    switch (type) {
    case LatheToolType::Turn:
    case LatheToolType::Bore: {
        const char* shape = insertAngle >= 79 && insertAngle <= 81 ? " (C)" : insertAngle >= 54 && insertAngle <= 56 ? " (D)"
                            : insertAngle >= 34 && insertAngle <= 36 ? " (V)" : insertAngle >= 59 && insertAngle <= 61 ? " (T)" : "";
        s += type == LatheToolType::Turn ? "turning insert, " : "boring bar for bores from " + mm(diameter) + ", ";
        s += mm(noseRadius) + " nose, " + fmt("%g", insertAngle) + " deg" + shape;
        break;
    }
    case LatheToolType::Groove: s += mm(width) + " grooving / parting blade, " + mm(reach) + " deep"; break;
    case LatheToolType::Thread: s += "60 deg threading insert"; break;
    case LatheToolType::Drill: s += mm(diameter) + " drill, " + fmt("%g", tipAngle) + " deg point"; break;
    }
    s += hss ? ", HSS" : ", carbide";
    if (!name.empty()) s += " \"" + name + "\"";
    return s;
}

const char* latheOpKey(LatheOpKind k) {
    switch (k) {
    case LatheOpKind::Face: return "face";
    case LatheOpKind::Rough: return "rough";
    case LatheOpKind::Finish: return "finish";
    case LatheOpKind::Groove: return "groove";
    case LatheOpKind::Drill: return "drill";
    case LatheOpKind::Thread: return "thread";
    case LatheOpKind::PartOff: return "partoff";
    }
    return "rough";
}

const LatheTool* LatheJob::tool(int number) const {
    for (const LatheTool& t : tools)
        if (t.number == number) return &t;
    return nullptr;
}

// --- the part's profile -----------------------------------------------------------------

Vec3 LatheFrame::toModel(double x, double z) const {
    return origin + axis * static_cast<float>(z) + radial * static_cast<float>(x);
}

namespace {

struct D2 {
    double a, b;
};

// The part's vertices in lathe coordinates: z along the axis (0 at the front face), (a, b) across.
struct LatheMesh {
    std::vector<double> z, a, b;
    std::vector<std::uint32_t> idx;
    Vec3 axisPoint, u, v, w;
    double front = 0.0;
};

bool latheMesh(const LatheJob& job, const Body& part, LatheMesh& m) {
    const MeshData mesh = worldMesh(part);
    if (mesh.indices.size() < 3 || mesh.vertices.empty()) return false;
    const int ax = std::clamp(job.setup.axis, 0, 2);
    Vec3 u{0, 0, 0};
    (ax == 0 ? u.x : ax == 1 ? u.y : u.z) = job.setup.flip ? -1.0f : 1.0f;
    // Across: the other two model axes, in order.
    Vec3 v{0, 0, 0}, w{0, 0, 0};
    const int a1 = (ax + 1) % 3, a2 = (ax + 2) % 3;
    (a1 == 0 ? v.x : a1 == 1 ? v.y : v.z) = 1.0f;
    (a2 == 0 ? w.x : a2 == 1 ? w.y : w.z) = 1.0f;
    AABB box = mesh.bounds();
    const Vec3 c = (box.min + box.max) * 0.5f;
    m.u = u, m.v = v, m.w = w;
    double front = -kInfL;
    for (const Vertex& p : mesh.vertices) front = std::max(front, static_cast<double>(dot(p.position - c, u)));
    m.front = front;
    m.axisPoint = c + u * static_cast<float>(front);
    m.z.reserve(mesh.vertices.size()), m.a.reserve(mesh.vertices.size()), m.b.reserve(mesh.vertices.size());
    for (const Vertex& p : mesh.vertices) {
        const Vec3 d = p.position - m.axisPoint;
        m.z.push_back(dot(d, u)), m.a.push_back(dot(d, v)), m.b.push_back(dot(d, w));
    }
    m.idx = mesh.indices;
    return true;
}

} // namespace

bool latheFrame(const LatheJob& job, const Body& part, LatheFrame& out) {
    LatheMesh m;
    if (!latheMesh(job, part, m)) return false;
    out.origin = m.axisPoint, out.axis = m.u;
    // Drawn in the plane that faces up: the model's Z across, unless Z is the axis.
    out.radial = std::fabs(m.u.z) > 0.5f ? Vec3{0, -1, 0} : Vec3{0, 0, 1};
    return true;
}

namespace {

bool computeProfile(const LatheJob& job, const Body& part, LatheProfile& out, std::string* error);

} // namespace

bool latheProfile(const LatheJob& job, const Body& part, LatheProfile& out, std::string* error) {
    // The last few, by the part's shape and place and the axis: every operation, simulation
    // and post asks again for the same.
    struct Entry {
        double key;
        int axis;
        bool flip;
        LatheProfile p;
    };
    static std::mutex lock;
    static std::vector<Entry> cache;
    const double key = camPartKey(part);
    {
        std::lock_guard<std::mutex> g(lock);
        for (const Entry& e : cache)
            if (e.key == key && e.axis == job.setup.axis && e.flip == job.setup.flip) {
                out = e.p;
                return true;
            }
    }
    if (!computeProfile(job, part, out, error)) return false;
    std::lock_guard<std::mutex> g(lock);
    if (cache.size() >= 4) cache.erase(cache.begin());
    cache.push_back({key, job.setup.axis, job.setup.flip, out});
    return true;
}

namespace {

bool computeProfile(const LatheJob& job, const Body& part, LatheProfile& out, std::string* error) {
    LatheMesh m;
    if (!latheMesh(job, part, m)) {
        if (error) *error = part.name + " has no surface";
        return false;
    }
    double zmin = kInfL;
    for (double z : m.z) zmin = std::min(zmin, z);
    const double len = -zmin;
    if (!(len > 1e-6)) {
        if (error) *error = part.name + " has no length along the axis";
        return false;
    }
    out.dz = std::clamp(len / 20000.0, 0.005, 0.05);
    out.z0 = -std::ceil(len / out.dz - 1e-9) * out.dz;
    const std::size_t n = static_cast<std::size_t>(std::llround(-out.z0 / out.dz)) + 1;
    out.outer.assign(n, 0.0), out.inner.assign(n, kInfL), out.has.assign(n, 0);
    // Each triangle on the stations it spans.
    std::vector<std::vector<std::uint32_t>> at(n);
    const std::size_t tris = m.idx.size() / 3;
    for (std::uint32_t t = 0; t < tris; ++t) {
        double lo = kInfL, hi = -kInfL;
        for (int k = 0; k < 3; ++k) lo = std::min(lo, m.z[m.idx[3 * t + k]]), hi = std::max(hi, m.z[m.idx[3 * t + k]]);
        const long i0 = std::max(0L, static_cast<long>(std::floor((lo - out.z0) / out.dz - 1e-7)));
        const long i1 = std::min(static_cast<long>(n) - 1, static_cast<long>(std::ceil((hi - out.z0) / out.dz + 1e-7)));
        for (long i = i0; i <= i1; ++i) at[static_cast<std::size_t>(i)].push_back(t);
    }
    // At a station: the plane's cut through each triangle. A face exactly on the station is
    // seen from both sides (a hair either way), and the part there is the more of the two.
    const double ra = std::cos(0.7071), rb = std::sin(0.7071); // a ray off any vertex
    parallelFor(static_cast<int>(n), [&](int si) {
        const std::size_t i = static_cast<std::size_t>(si);
        double outer = 0.0, inner = kInfL;
        bool any = false, solid = false;
        for (const double eps : {-1e-6, 1e-6}) {
            const double z = out.zAt(i) + eps;
            int crossings = 0;
            for (const std::uint32_t t : at[i]) {
                D2 p[2];
                int k = 0;
                for (int e = 0; e < 3 && k < 2; ++e) {
                    const std::uint32_t i0 = m.idx[3 * t + e], i1 = m.idx[3 * t + (e + 1) % 3];
                    const double z0 = m.z[i0], z1 = m.z[i1];
                    if ((z0 - z) * (z1 - z) > 0 || z0 == z1) continue;
                    const double s = (z - z0) / (z1 - z0);
                    if (s < 0 || s > 1) continue;
                    p[k++] = {m.a[i0] + (m.a[i1] - m.a[i0]) * s, m.b[i0] + (m.b[i1] - m.b[i0]) * s};
                }
                if (k < 2) continue;
                any = true;
                outer = std::max({outer, std::hypot(p[0].a, p[0].b), std::hypot(p[1].a, p[1].b)});
                // Nearest the axis on the segment.
                const double da = p[1].a - p[0].a, db = p[1].b - p[0].b, l2 = da * da + db * db;
                const double s = l2 > 0 ? std::clamp(-(p[0].a * da + p[0].b * db) / l2, 0.0, 1.0) : 0.0;
                inner = std::min(inner, std::hypot(p[0].a + da * s, p[0].b + db * s));
                // The ray from the axis: does the segment cross it?
                const double c0 = p[0].a * rb - p[0].b * ra, c1 = p[1].a * rb - p[1].b * ra;
                if ((c0 > 0) != (c1 > 0)) {
                    const double f = c0 / (c0 - c1);
                    const double along = (p[0].a + da * f) * ra + (p[0].b + db * f) * rb;
                    if (along > 0) ++crossings;
                }
            }
            if (crossings % 2 == 1) solid = true;
        }
        out.has[i] = any ? 1 : 0;
        out.outer[i] = outer;
        out.inner[i] = !any ? 0.0 : solid ? 0.0 : inner;
    });
    out.maxRadius = 0;
    for (double r : out.outer) out.maxRadius = std::max(out.maxRadius, r);
    return true;
}

} // namespace

double latheStockDiameter(const LatheJob& job, const LatheProfile& p) {
    if (job.setup.stockDiameter > 0) return job.setup.stockDiameter;
    return std::ceil(2.0 * p.maxRadius + 2.0 - 1e-3);
}

// --- feeds and speeds -------------------------------------------------------------------

namespace {

struct LatheCut {
    double vc = 0, f = 0, rpm = 0; // m/min, mm/turn, constant rpm (0: constant surface speed)
};

LatheCut latheCut(const LatheJob& job, const LatheOperation& op, const LatheTool& t) {
    const CamMaterial* mat = camMaterial(job.setup.material);
    const CamMaterial& m = mat ? *mat : camMaterials().front();
    LatheCut c;
    c.vc = op.speed > 0 ? op.speed : t.vc > 0 ? t.vc : (t.hss ? m.cuttingSpeedHss : m.cuttingSpeedCarbide) * 0.9;
    const double scale = std::clamp(std::sqrt(m.chipLoadPerDiameter / 0.008), 0.5, 1.2);
    double f = 0;
    switch (op.kind) {
    case LatheOpKind::Face:
    case LatheOpKind::Rough: f = std::clamp(0.6 * t.noseRadius, 0.1, 0.35) * scale; break;
    case LatheOpKind::Finish: f = std::clamp(std::sqrt(32.0 * t.noseRadius * 0.0016), 0.05, 0.2); break;
    case LatheOpKind::Groove: f = 0.05 * scale; break;
    case LatheOpKind::PartOff: f = 0.04 * scale; break;
    case LatheOpKind::Drill: f = std::clamp(m.drillFeedPerDiameter * t.diameter, 0.03, 0.4); break;
    case LatheOpKind::Thread: f = op.pitch; break;
    }
    c.f = op.feedPerRev > 0 && op.kind != LatheOpKind::Thread ? op.feedPerRev : t.feedPerRev > 0 && op.kind != LatheOpKind::Thread ? t.feedPerRev : f;
    if (op.rpm > 0) c.rpm = std::min(op.rpm, job.setup.maxRpm);
    else if (op.kind == LatheOpKind::Drill) c.rpm = std::min(1000.0 * c.vc / (kPiL * std::max(t.diameter, 0.5)), job.setup.maxRpm);
    else if (op.kind == LatheOpKind::Thread) {
        const double d = std::max(op.threadDiameter, 1.0);
        c.rpm = std::min({1000.0 * c.vc * 0.5 / (kPiL * d), job.setup.maxRpm, 2000.0});
    }
    return c;
}

// What an operation's moves were made with (or would be).
LatheCut cutOf(const LatheJob& job, const LatheOperation& op, const LatheTool& t) {
    if (op.generated) return {op.usedSpeed, op.usedFeed, op.usedRpm};
    return latheCut(job, op, t);
}

double rpmAt(const LatheJob& job, const LatheCut& c, double x) {
    if (c.rpm > 0) return c.rpm;
    return std::clamp(1000.0 * c.vc / (2.0 * kPiL * std::max(std::fabs(x), 1e-3)), 50.0, job.setup.maxRpm);
}

// --- the grid the operations work on -------------------------------------------------

// Stations along Z from behind the part (the bar going to the chuck) to in front of the
// bar's end; values on one side of the axis in "y": the radius outside, minus the radius
// in a bore, so that both sides cut down towards y = -infinity... and keep above.
struct Grid {
    double z0 = 0, dz = 0.01;
    std::size_t n = 0;
    double zAt(std::size_t i) const noexcept { return z0 + dz * static_cast<double>(i); }
    long index(double z) const noexcept { return static_cast<long>(std::floor((z - z0) / dz + 0.5)); }
};

struct Plan {
    const LatheJob& job;
    const LatheProfile& prof;
    Grid g;
    double stockR = 0;   // the bar
    double front = 0;    // the bar's end (z)
    double clear = 2.0;
    // What must stay: outside (radius), and in the bore (radius; 0 solid).
    std::vector<double> keepOut, keepIn;
    explicit Plan(const LatheJob& j, const LatheProfile& p) : job(j), prof(p) {
        g.dz = p.dz;
        stockR = latheStockDiameter(j, p) * 0.5;
        front = std::max(j.setup.frontStock, 0.0);
        clear = std::max(j.setup.clearance, 0.5);
        const double back = 12.0; // the bar behind the part: room for a blade
        g.z0 = p.z0 - std::ceil(back / g.dz) * g.dz;
        const double top = front + clear + 6.0;
        g.n = static_cast<std::size_t>(std::ceil((top - g.z0) / g.dz)) + 1;
        keepOut.assign(g.n, -kInfL), keepIn.assign(g.n, kInfL);
        // Behind the part the outside is turned on as far as a parting blade takes, so the
        // part's back corner comes off sharp; then the bar holding it.
        double partOff = 0;
        for (const LatheTool& t : j.tools)
            if (t.type == LatheToolType::Groove) partOff = std::max(partOff, t.width + 0.5);
        double backR = 0;
        for (std::size_t q = 0; q < std::min<std::size_t>(p.size(), 3); ++q) backR = std::max(backR, p.has[q] ? p.outer[q] : 0.0);
        for (std::size_t i = 0; i < g.n; ++i) {
            const double z = g.zAt(i);
            if (z < p.z0 - 1e-9) keepOut[i] = z >= p.z0 - partOff ? backR : stockR, keepIn[i] = 0.0; // the bar holding the part
            else if (z <= 1e-9) {
                const std::size_t k = std::min(p.size() - 1, static_cast<std::size_t>(std::llround((z - p.z0) / p.dz)));
                // Conservative: a station takes the most of the part either side of it.
                double o = -kInfL, in = kInfL;
                for (std::size_t q = k > 0 ? k - 1 : 0; q <= std::min(p.size() - 1, k + 1); ++q) {
                    if (!p.has[q]) continue;
                    o = std::max(o, p.outer[q]);
                    in = std::min(in, p.inner[q]);
                }
                keepOut[i] = o;
                keepIn[i] = in;
            }
        }
    }
};

// The "keep" in y for a side: outside y = radius; in the bore y = -radius.
std::vector<double> keepY(const Plan& pl, bool inside, double leave, double leaveZ) {
    const std::size_t n = pl.g.n;
    std::vector<double> k(n);
    for (std::size_t i = 0; i < n; ++i) k[i] = inside ? (pl.keepIn[i] < kInfL ? -pl.keepIn[i] : -kInfL) : pl.keepOut[i];
    if (leaveZ > 0) { // faces too
        const long m = static_cast<long>(std::ceil(leaveZ / pl.g.dz));
        std::vector<double> o(n, -kInfL);
        for (long i = 0; i < static_cast<long>(n); ++i)
            for (long j = std::max(0L, i - m); j <= std::min(static_cast<long>(n) - 1, i + m); ++j) o[static_cast<std::size_t>(i)] = std::max(o[static_cast<std::size_t>(i)], k[static_cast<std::size_t>(j)]);
        k.swap(o);
    }
    if (leave != 0)
        for (double& v : k)
            if (v > -kInfL) v += leave;
    return k;
}

// The nose's centre over the keep: the keep grown by the nose radius (Minkowski).
std::vector<double> noseCentre(const Grid& g, const std::vector<double>& k, double rn) {
    const long m = static_cast<long>(std::floor(rn / g.dz + 1e-9));
    std::vector<double> o(g.n, -kInfL);
    std::vector<double> lift(static_cast<std::size_t>(m) + 1);
    for (long j = 0; j <= m; ++j) lift[static_cast<std::size_t>(j)] = std::sqrt(std::max(0.0, rn * rn - (static_cast<double>(j) * g.dz) * (static_cast<double>(j) * g.dz)));
    parallelFor(static_cast<int>(g.n), [&](int ii) {
        const long i = ii;
        double best = -kInfL;
        for (long j = -m; j <= m; ++j) {
            const long q = i + j;
            if (q < 0 || q >= static_cast<long>(g.n)) continue;
            const double v = k[static_cast<std::size_t>(q)];
            if (v > -kInfL) best = std::max(best, v + lift[static_cast<std::size_t>(std::labs(j))]);
        }
        o[static_cast<std::size_t>(i)] = best;
    });
    return o;
}

// A blade's edge (its middle) over the keep: the most the keep reaches across its width,
// less its corners' rounding.
std::vector<double> bladeEdge(const Grid& g, const std::vector<double>& k, double width, double rc) {
    const double half = width * 0.5;
    rc = std::clamp(rc, 0.0, half);
    const long m = static_cast<long>(std::floor(half / g.dz - 1e-9));
    std::vector<double> o(g.n, -kInfL);
    parallelFor(static_cast<int>(g.n), [&](int ii) {
        const long i = ii;
        double best = -kInfL;
        for (long j = -m; j <= m; ++j) {
            const long q = i + j;
            if (q < 0 || q >= static_cast<long>(g.n)) continue;
            const double v = k[static_cast<std::size_t>(q)];
            if (!(v > -kInfL)) continue;
            const double d = std::fabs(static_cast<double>(j) * g.dz), e = d - (half - rc);
            const double h = e > 0 ? rc - std::sqrt(std::max(0.0, rc * rc - e * e)) : 0.0;
            best = std::max(best, v - h);
        }
        o[static_cast<std::size_t>(i)] = best;
    });
    return o;
}

// Where the tool can get going towards the chuck, going down no steeper than `relief`.
std::vector<double> reachable(const Grid& g, const std::vector<double>& o, double relief) {
    std::vector<double> r(o);
    const double drop = std::tan(relief * kPiL / 180.0) * g.dz;
    for (std::size_t i = g.n - 1; i-- > 0;) r[i] = std::max(o[i], r[i + 1] - drop);
    return r;
}

// A path along stations [i1 down to i0] at y(i), as few points as the tolerance allows.
std::vector<std::pair<double, double>> simplified(const Grid& g, const std::vector<double>& y, long i1, long i0, double tol) {
    std::vector<std::pair<double, double>> pts; // (z, y)
    for (long i = i1; i >= i0; --i) pts.push_back({g.zAt(static_cast<std::size_t>(i)), y[static_cast<std::size_t>(i)]});
    if (pts.size() < 3) return pts;
    std::vector<char> keep(pts.size(), 0);
    keep.front() = keep.back() = 1;
    std::vector<std::pair<std::size_t, std::size_t>> stack{{0, pts.size() - 1}};
    while (!stack.empty()) {
        const auto [a, b] = stack.back();
        stack.pop_back();
        if (b <= a + 1) continue;
        const double dzv = pts[b].first - pts[a].first, dyv = pts[b].second - pts[a].second, l = std::hypot(dzv, dyv);
        double worst = 0;
        std::size_t at = a;
        for (std::size_t q = a + 1; q < b; ++q) {
            const double d = l > 0 ? std::fabs((pts[q].first - pts[a].first) * dyv - (pts[q].second - pts[a].second) * dzv) / l
                                   : std::hypot(pts[q].first - pts[a].first, pts[q].second - pts[a].second);
            if (d > worst) worst = d, at = q;
        }
        if (worst > tol) {
            keep[at] = 1;
            stack.push_back({a, at}), stack.push_back({at, b});
        }
    }
    std::vector<std::pair<double, double>> out;
    for (std::size_t q = 0; q < pts.size(); ++q)
        if (keep[q]) out.push_back(pts[q]);
    return out;
}

struct Emit {
    std::vector<LatheMove>& moves;
    bool inside;
    double sgn() const { return inside ? -1.0 : 1.0; }
    void rapid(double z, double y) { moves.push_back({LatheMove::Rapid, y * sgn(), z, 0}); }
    void feed(double z, double y, double f) { moves.push_back({LatheMove::Feed, y * sgn(), z, f}); }
    double x() const { return moves.empty() ? 0 : moves.back().x; }
    double z() const { return moves.empty() ? 0 : moves.back().z; }
};

// The bore's radius the drills before `op` leave (0: none).
double drilledRadius(const LatheJob& job, const LatheOperation& op) {
    double r = 0;
    for (const LatheOperation& o : job.ops) {
        if (&o == &op) break;
        if (o.kind != LatheOpKind::Drill) continue;
        if (const LatheTool* t = job.tool(o.tool)) r = std::max(r, t->diameter * 0.5);
    }
    return r;
}

// Where a boring bar's nose goes to travel in and out of a bore of radius r: near the
// middle, with the bar's far side clear of the wall opposite.
double boreHome(const LatheTool& t, double r) {
    return std::clamp(t.diameter - r + 0.2, t.noseRadius + 0.1, std::max(t.noseRadius + 0.1, r - t.noseRadius - 0.05));
}

double stepdownFor(const LatheJob& job, const LatheOperation& op) {
    if (op.stepdown > 0) return op.stepdown;
    const CamMaterial* m = camMaterial(job.setup.material);
    return std::clamp(4.0 * (m ? m->stepdownFactor : 0.5), 0.5, 2.5);
}

// Roughing in passes along Z, each a depth of cut further down, following the part
// (grown by the stock to leave) where it is higher; lifting over what the passes before
// already took.
bool genRough(const Plan& pl, const LatheOperation& op, const LatheTool& t, double feed, std::vector<LatheMove>& out, std::string* err) {
    const Grid& g = pl.g;
    const bool inside = op.inside;
    const double rn = t.noseRadius;
    const std::vector<double> k = keepY(pl, inside, std::max(op.leave, 0.0), std::max(op.leaveZ, 0.0));
    const std::vector<double> reach = reachable(g, noseCentre(g, k, rn), t.relief());
    double top; // the stock's top in y, at the nose's centre
    if (inside) {
        const double r = drilledRadius(pl.job, op);
        if (r <= 0) {
            if (err) *err = "Drill the bore first (LATHE DRILL): the boring bar needs a hole to go in";
            return false;
        }
        if (t.diameter > 2 * r + 1e-9) {
            if (err) *err = "The boring bar needs a " + fmt("%g", t.diameter) + " mm bore; the drill leaves " + fmt("%g", 2 * r);
            return false;
        }
        top = -(r - rn); // the nose against the drilled wall
    } else top = pl.stockR + rn;
    // The Z range: from in front of the bar to the operation's end.
    const double zHi = std::isnan(op.zFrom) ? pl.front + pl.clear + rn : op.zFrom + rn;
    const double zLo = std::isnan(op.zTo) ? g.z0 : op.zTo;
    const long i1 = std::min(static_cast<long>(g.n) - 1, g.index(zHi)), i0 = std::max(0L, g.index(zLo));
    double lowest = kInfL; // the deepest the part lets the tool go (air in front doesn't count)
    for (long i = i0; i <= i1; ++i)
        if (reach[static_cast<std::size_t>(i)] > -kInfL) lowest = std::min(lowest, reach[static_cast<std::size_t>(i)]);
    if (!inside) lowest = std::max(lowest, rn); // not past the middle
    if (!(lowest < top - 1e-6)) {
        if (err) *err = "Nothing to rough there: the part already fills the stock";
        return false;
    }
    const double doc = stepdownFor(pl.job, op);
    Emit e{out, inside};
    const double tol = std::max(pl.job.setup.tolerance * 0.5, 0.002);
    std::vector<double> prev(g.n, top); // the stock's top before this pass
    const double zStart = g.zAt(static_cast<std::size_t>(i1));
    long from = i1; // where the tool is (a station)
    for (double level = top - doc;; level -= doc) {
        const double y = std::max(level, lowest);
        std::vector<double> path(g.n);
        for (std::size_t i = 0; i < g.n; ++i) path[i] = std::max(reach[i], y);
        // Runs of stations where this pass takes something.
        long i = i1;
        while (i >= i0) {
            while (i >= i0 && !(path[static_cast<std::size_t>(i)] < prev[static_cast<std::size_t>(i)] - tol)) --i;
            if (i < i0) break;
            long j = i;
            while (j >= i0 && path[static_cast<std::size_t>(j)] < prev[static_cast<std::size_t>(j)] - tol) --j;
            // Stations i down to j+1 cut; the path goes on to where it meets the stock before.
            const long a = std::min(i + 1, i1), b = std::max(j, i0);
            double over = -kInfL; // safe height for the way there and back
            for (long q = std::min(b, from); q <= i1; ++q) over = std::max(over, prev[static_cast<std::size_t>(q)]);
            const double safe = std::min(over, top) + 0.5;
            const auto pts = simplified(g, path, a, b, tol);
            e.rapid(pts.front().first, std::max(safe, pts.front().second + 0.5));
            e.rapid(pts.front().first, pts.front().second + 0.5);
            for (const auto& [z, yy] : pts) e.feed(z, yy, feed);
            // Off the cut at 45 degrees, then back out.
            e.feed(pts.back().first + 0.5, pts.back().second + 0.5, feed);
            e.rapid(pts.back().first + 0.5, safe);
            for (long q = b; q <= a; ++q) prev[static_cast<std::size_t>(q)] = std::min(prev[static_cast<std::size_t>(q)], path[static_cast<std::size_t>(q)]);
            from = b;
            i = j;
        }
        if (y <= lowest + 1e-12) break;
    }
    // Back to in front of the part, clear of the stock (out of a bore along its middle).
    if (inside) {
        e.rapid(e.z(), -boreHome(t, -top + rn));
        e.rapid(zStart, -boreHome(t, -top + rn));
    } else e.rapid(zStart, top + pl.clear);
    return true;
}

// The finish: once along the part, as far as the insert reaches.
bool genFinish(const Plan& pl, const LatheOperation& op, const LatheTool& t, double feed, std::vector<LatheMove>& out, std::string* err) {
    const Grid& g = pl.g;
    const bool inside = op.inside;
    const double rn = t.noseRadius;
    const std::vector<double> k = keepY(pl, inside, op.leave, op.leaveZ);
    const std::vector<double> reach = reachable(g, noseCentre(g, k, rn), t.relief());
    double top;
    if (inside) {
        const double r = drilledRadius(pl.job, op);
        if (r <= 0) {
            if (err) *err = "Drill the bore first (LATHE DRILL)";
            return false;
        }
        top = -(r - rn);
    } else top = pl.stockR + rn;
    const double zHi = std::isnan(op.zFrom) ? rn : op.zFrom + rn;
    const double zLo = std::isnan(op.zTo) ? g.z0 : op.zTo;
    long i1 = std::min(static_cast<long>(g.n) - 1, g.index(zHi));
    const long i0 = std::max(0L, g.index(zLo));
    while (i1 > i0 && !(reach[static_cast<std::size_t>(i1)] > -kInfL)) --i1; // the part's front
    // Down to where the path climbs out of the stock.
    long end = i1;
    while (end > i0 && reach[static_cast<std::size_t>(end - 1)] < top - 1e-9) --end;
    if (end >= i1) {
        if (err) *err = "Nothing to finish there";
        return false;
    }
    std::vector<double> path(reach);
    for (double& v : path) v = std::min(v, top);
    const double tol = std::max(pl.job.setup.tolerance * 0.5, 0.002);
    const auto pts = simplified(g, path, i1, end, tol);
    Emit e{out, inside};
    const double z0 = pts.front().first + pl.clear;
    const double outside = inside ? -boreHome(t, -top + rn) : top + pl.clear;
    e.rapid(z0, outside);
    e.rapid(z0, pts.front().second);
    for (const auto& [z, y] : pts) e.feed(z, y, feed);
    e.feed(pts.back().first + 0.3, pts.back().second + 0.3, feed);
    e.rapid(pts.back().first + 0.3, inside ? std::max(pts.back().second + 0.3, outside) : std::max(top + pl.clear, pts.back().second + 0.3));
    if (inside) e.rapid(z0, outside); // out of the bore along its middle
    else e.rapid(z0, top + pl.clear);
    return true;
}

// Facing the bar's end down to the front face, passes towards the chuck.
bool genFace(const Plan& pl, const LatheOperation& op, const LatheTool& t, double feed, std::vector<LatheMove>& out, std::string* err) {
    const double rn = t.noseRadius;
    const double faceZ = std::max(op.leaveZ, 0.0);
    if (pl.front <= faceZ + 1e-9) {
        if (err) *err = "No stock in front of the part to face (LATHE SETUP FRONT mm)";
        return false;
    }
    const double doc = stepdownFor(pl.job, op);
    Emit e{out, false};
    const double outside = pl.stockR + rn + pl.clear;
    for (double z = pl.front - doc;; z -= doc) {
        const double zz = std::max(z, faceZ);
        e.rapid(zz + rn + doc + 0.5, outside);
        e.rapid(zz + rn, outside);
        e.feed(zz + rn, -rn, feed); // past the middle
        e.feed(zz + rn + 0.5, -rn + 0.5, feed);
        e.rapid(zz + rn + pl.clear, -rn + 0.5);
        e.rapid(zz + rn + pl.clear, outside);
        if (zz <= faceZ + 1e-12) break;
    }
    return true;
}

// Grooving the recesses a turning insert leaves (it can't go down behind a shoulder
// steeper than its relief): plunges side by side with a blade.
bool genGroove(const Plan& pl, const LatheOperation& op, const LatheTool& t, double feed, std::vector<LatheMove>& out, std::string* err) {
    const Grid& g = pl.g;
    const bool inside = op.inside;
    // The insert the recesses are left by: the job's first turning insert (in a bore, its
    // first boring bar).
    const LatheTool* turn = nullptr;
    for (const LatheTool& q : pl.job.tools)
        if (q.type == (inside ? LatheToolType::Bore : LatheToolType::Turn) && !turn) turn = &q;
    const std::vector<double> k = keepY(pl, inside, std::max(op.leave, 0.0), 0.0);
    const std::vector<double> edge = bladeEdge(g, k, t.width, t.noseRadius);
    std::vector<double> left(g.n, -kInfL); // the material the turning leaves over the keep
    if (turn) {
        const double rn = turn->noseRadius;
        const std::vector<double> r = reachable(g, noseCentre(g, k, rn), turn->relief());
        // The nose's lowest edge along its path (the path less the nose, Minkowski).
        const long m = static_cast<long>(std::floor(rn / g.dz + 1e-9));
        const double tol = std::max(pl.job.setup.tolerance, 0.01) + 2 * g.dz;
        for (long i = 0; i < static_cast<long>(g.n); ++i) {
            double low = kInfL;
            for (long jj = -m; jj <= m; ++jj) {
                const long q = i + jj;
                if (q < 0 || q >= static_cast<long>(g.n) || !(r[static_cast<std::size_t>(q)] > -kInfL)) continue;
                const double d = static_cast<double>(jj) * g.dz;
                low = std::min(low, r[static_cast<std::size_t>(q)] - std::sqrt(std::max(0.0, rn * rn - d * d)));
            }
            const std::size_t ui = static_cast<std::size_t>(i);
            if (low < kInfL && k[ui] > -kInfL && low > k[ui] + tol) left[ui] = low;
        }
    }
    const double zHi = std::isnan(op.zFrom) ? 0.0 : op.zFrom, zLo = std::isnan(op.zTo) ? pl.prof.z0 : op.zTo;
    const long i1 = std::min(static_cast<long>(g.n) - 1, g.index(zHi)), i0 = std::max(0L, g.index(zLo));
    Emit e{out, inside};
    // Outside the blade comes in from beyond the bar; in a bore, along its middle.
    double outside = pl.stockR + pl.clear, entry = pl.front + pl.clear;
    if (inside) {
        double bore = kInfL; // the narrowest the bore is over the grooves
        for (long q = i0; q <= i1; ++q)
            if (left[static_cast<std::size_t>(q)] > -kInfL) bore = std::min(bore, -left[static_cast<std::size_t>(q)]);
        if (bore < kInfL && 2 * bore < t.diameter) {
            if (err) *err = "The grooving bar needs a " + fmt("%g", t.diameter) + " mm bore; the bore is " + fmt("%.3g", 2 * bore) + " mm there";
            return false;
        }
        outside = bore < kInfL ? -boreHome(t, bore) : -t.noseRadius;
    }
    std::size_t grooves = 0;
    const double step = 0.8 * t.width;
    long i = i1;
    while (i >= i0) {
        while (i >= i0 && !(left[static_cast<std::size_t>(i)] > -kInfL)) --i;
        if (i < i0) break;
        long j = i;
        while (j >= i0 && left[static_cast<std::size_t>(j)] > -kInfL) --j;
        // The recess spans stations j+1..i (and the walls either side); plunge across it.
        const double za = g.zAt(static_cast<std::size_t>(j + 1)), zb = g.zAt(static_cast<std::size_t>(i));
        double topY = -kInfL;
        for (long q = j + 1; q <= i; ++q) topY = std::max(topY, left[static_cast<std::size_t>(q)]);
        std::vector<double> centres;
        // The blade's sides half a station off the walls (the walls' stations stay).
        const double first = zb + 0.5 * g.dz - t.width * 0.5, last = za - 0.5 * g.dz + t.width * 0.5;
        if (first < last) centres.push_back(0.5 * (za + zb)); // narrower than the blade: one plunge in the middle
        else {
            const int n = static_cast<int>(std::ceil((first - last) / step - 1e-9));
            for (int q = 0; q <= n; ++q) centres.push_back(first - (first - last) * (n ? static_cast<double>(q) / n : 0.0));
        }
        for (const double zc : centres) {
            const long ci = std::clamp(g.index(zc), 0L, static_cast<long>(g.n) - 1);
            const double bottom = edge[static_cast<std::size_t>(ci)];
            if (!(bottom > -kInfL) || bottom > topY + 1e-9) continue;
            if (topY - bottom > t.reach) {
                if (err) *err = "The blade reaches " + fmt("%g", t.reach) + " mm; the recess at Z" + fmt("%.3f", zc) + " is deeper";
                return false;
            }
            if (inside && out.empty()) e.rapid(entry, outside);
            e.rapid(zc, outside);
            e.rapid(zc, topY + 0.5);
            e.feed(zc, bottom, feed);
            e.rapid(zc, outside);
        }
        ++grooves;
        i = j;
    }
    if (grooves == 0) {
        if (err)
            *err = turn ? std::string("No recesses: the ") + (inside ? "boring bar reaches all of the bore" : "turning insert reaches all of the outside")
                        : std::string("Add a ") + (inside ? "boring bar" : "turning tool") + " first: grooving takes what it can't reach";
        return false;
    }
    if (inside) e.rapid(entry, outside); // out of the bore along its middle
    return true;
}

// Parting the part off the bar at its back, the blade's front side on the back face.
bool genPartOff(const Plan& pl, const LatheOperation& op, const LatheTool& t, double feed, std::vector<LatheMove>& out, std::string* err) {
    const double zc = pl.prof.z0 - t.width * 0.5;
    if (pl.stockR > t.reach + 1e-9) {
        if (err) *err = "The blade reaches " + fmt("%g", t.reach) + " mm: too short to part a " + fmt("%g", 2 * pl.stockR) + " mm bar";
        return false;
    }
    double bore = 0; // a bore through the back: part off just into it
    if (!pl.prof.inner.empty() && pl.prof.has[0]) bore = pl.prof.inner[0];
    const double bottom = bore > 0.5 ? bore - 0.3 : -0.2;
    Emit e{out, false};
    const double outside = pl.stockR + pl.clear;
    e.rapid(zc, outside);
    e.rapid(zc, pl.stockR + 0.5);
    if (op.peck > 0) {
        for (double y = pl.stockR - op.peck;; y -= op.peck) {
            const double yy = std::max(y, bottom);
            e.feed(zc, yy, feed);
            if (yy <= bottom + 1e-12) break;
            e.rapid(zc, yy + 0.3);
            e.rapid(zc, yy + 0.05);
        }
    } else e.feed(zc, bottom, feed);
    e.rapid(zc, outside);
    return true;
}

bool genDrill(const Plan& pl, const LatheOperation& op, const LatheTool& t, double feed, std::vector<LatheMove>& out, std::string* err) {
    const LatheProfile& p = pl.prof;
    const double r = t.diameter * 0.5;
    const double tipLen = r / std::tan(std::clamp(t.tipAngle, 60.0, 180.0) * 0.5 * kPiL / 180.0);
    double depth = op.depth;
    bool through = false;
    if (depth <= 0) { // the bore from the front
        std::size_t i = p.size();
        while (i > 0 && p.has[i - 1] && p.inner[i - 1] > 1e-6) --i;
        if (i == p.size()) {
            if (err) *err = "The part has no bore on its axis: give the drill a DEPTH";
            return false;
        }
        through = i == 0;
        depth = through ? p.length() + tipLen + 1.0 : -p.zAt(i);
    }
    const double tip = -depth;
    // The drill mustn't be wider than the bore it goes down (its point narrower near the tip).
    for (std::size_t i = 0; i < p.size(); ++i) {
        const double z = p.zAt(i);
        if (z < tip || !p.has[i]) continue;
        const double need = std::min(r, (z - tip) * std::tan(std::clamp(t.tipAngle, 60.0, 180.0) * 0.5 * kPiL / 180.0));
        if (p.inner[i] + std::max(pl.job.setup.tolerance, 0.01) < need) {
            if (err) *err = "The " + fmt("%g", t.diameter) + " mm drill is wider than the bore at Z" + fmt("%.3f", z) + " (" + fmt("%g", 2 * p.inner[i]) + " mm)";
            return false;
        }
    }
    if (depth > t.reach) {
        if (err) *err = "The drill reaches " + fmt("%g", t.reach) + " mm; the hole is " + fmt("%.2f", depth) + " deep";
        return false;
    }
    Emit e{out, false};
    const double start = pl.front + pl.clear;
    e.rapid(start, 0);
    if (op.peck > 0) { // full retracts, back down to just above the last depth
        double at = pl.front + 0.5;
        for (double z = pl.front - op.peck;; z -= op.peck) {
            const double zz = std::max(z, tip);
            e.rapid(at, 0);
            e.feed(zz, 0, feed);
            if (zz <= tip + 1e-12) break;
            e.rapid(start, 0);
            at = zz + 0.3;
        }
    } else {
        e.rapid(pl.front + 0.5, 0);
        e.feed(tip, 0, feed);
    }
    e.rapid(start, 0);
    return true;
}

// Single-point threading: passes of falling depth, the insert going in along the thread's
// flank (29.5 degrees), each pass synchronised to the spindle.
// ISO metric thread depths on the radius: outside to the minor diameter, in a nut out to the major.
double threadDepth(const LatheOperation& op) { return (op.inside ? 0.5413 : 0.6134) * op.pitch; }

// The thread's major radius: as given, else outside the most of the part over its length,
// in a bore its least (the minor diameter) plus the depth (0: none).
double threadMajor(const Plan& pl, const LatheOperation& op) {
    if (op.threadDiameter > 0) return op.threadDiameter * 0.5;
    const LatheProfile& p = pl.prof;
    const double zA = std::isnan(op.zFrom) ? 0.0 : op.zFrom, zB = std::isnan(op.zTo) ? p.z0 : op.zTo;
    double major = 0, minor = kInfL;
    for (std::size_t i = 0; i < p.size(); ++i)
        if (p.zAt(i) >= std::min(zA, zB) && p.zAt(i) <= std::max(zA, zB) && p.has[i])
            major = std::max(major, p.outer[i]), minor = std::min(minor, p.inner[i]);
    if (op.inside) return minor > 0 && minor < kInfL ? minor + threadDepth(op) : 0.0;
    return major;
}

bool genThread(const Plan& pl, const LatheOperation& op, const LatheTool& t, std::vector<LatheMove>& out, std::string* err) {
    if (op.pitch <= 0) {
        if (err) *err = "THREAD needs a PITCH (mm)";
        return false;
    }
    const LatheProfile& p = pl.prof;
    const double zA = std::isnan(op.zFrom) ? 0.0 : op.zFrom, zB = std::isnan(op.zTo) ? p.z0 : op.zTo;
    const double hi = std::max(zA, zB), lo = std::min(zA, zB);
    if (hi - lo < op.pitch) {
        if (err) *err = "The thread is shorter than its pitch: give FROM and TO";
        return false;
    }
    const double major = threadMajor(pl, op);
    if (major <= 0) {
        if (err) *err = op.inside ? "No bore there to thread (or give its DIAMETER)" : "No part there to thread";
        return false;
    }
    const double h = threadDepth(op);
    const int passes = std::max(4, static_cast<int>(std::ceil(h / 0.12)));
    const double runIn = 2.0 * op.pitch, flank = std::tan(29.5 * kPiL / 180.0);
    // Outside the insert goes in from beyond the bar towards the axis; in a bore it starts at
    // the minor diameter and goes out, coming and going along the bore's middle.
    const double start = op.inside ? major - h : major;
    double away = std::max(major, pl.stockR) + pl.clear;
    if (op.inside) {
        if (2 * start < t.diameter) {
            if (err) *err = "The threading bar needs a " + fmt("%g", t.diameter) + " mm bore; the thread's is " + fmt("%.3g", 2 * start) + " mm";
            return false;
        }
        away = boreHome(t, start);
        out.push_back({LatheMove::Rapid, away, pl.front + pl.clear, 0});
    }
    for (int q = 1; q <= passes + 1; ++q) { // the last again: a spring pass
        const double d = h * std::sqrt(static_cast<double>(std::min(q, passes)) / passes);
        const double shift = d * flank, x = op.inside ? start + d : start - d;
        out.push_back({LatheMove::Rapid, away, hi + runIn + shift, 0});
        out.push_back({LatheMove::Rapid, x, hi + runIn + shift, 0});
        out.push_back({LatheMove::Thread, x, lo + shift, op.pitch});
        out.push_back({LatheMove::Rapid, away, lo + shift, 0});
    }
    out.push_back({LatheMove::Rapid, away, hi + runIn, 0});
    if (op.inside) out.push_back({LatheMove::Rapid, away, pl.front + pl.clear, 0});
    return true;
}

} // namespace

bool generateLatheOperation(const LatheJob& job, const Body& part, LatheOperation& op, std::string* error) {
    op.moves.clear();
    op.generated = false;
    const LatheTool* t = job.tool(op.tool);
    if (!t) {
        if (error) *error = "No tool T" + std::to_string(op.tool);
        return false;
    }
    auto needs = [&](std::initializer_list<LatheToolType> types, const char* what) {
        for (LatheToolType k : types)
            if (t->type == k) return true;
        if (error) *error = std::string(latheOpKey(op.kind)) + " needs " + what + "; T" + std::to_string(t->number) + " is a " + latheToolTypeKey(t->type) + " tool";
        return false;
    };
    switch (op.kind) {
    case LatheOpKind::Face: if (!needs({LatheToolType::Turn}, "a turning insert")) return false; break;
    case LatheOpKind::Rough:
    case LatheOpKind::Finish:
        if (op.inside ? !needs({LatheToolType::Bore}, "a boring bar") : !needs({LatheToolType::Turn}, "a turning insert")) return false;
        break;
    case LatheOpKind::Groove:
    case LatheOpKind::PartOff: if (!needs({LatheToolType::Groove}, "a grooving / parting blade")) return false; break;
    case LatheOpKind::Drill: if (!needs({LatheToolType::Drill}, "a drill")) return false; break;
    case LatheOpKind::Thread: if (!needs({LatheToolType::Thread}, "a threading insert")) return false; break;
    }
    LatheProfile prof;
    if (!latheProfile(job, part, prof, error)) return false;
    const Plan pl(job, prof);
    LatheOperation sized = op; // a thread's speed is from its diameter
    if (op.kind == LatheOpKind::Thread) sized.threadDiameter = 2 * threadMajor(pl, op);
    const LatheCut c = latheCut(job, sized, *t);
    bool ok = false;
    switch (op.kind) {
    case LatheOpKind::Face: ok = genFace(pl, op, *t, c.f, op.moves, error); break;
    case LatheOpKind::Rough: ok = genRough(pl, op, *t, c.f, op.moves, error); break;
    case LatheOpKind::Finish: ok = genFinish(pl, op, *t, c.f, op.moves, error); break;
    case LatheOpKind::Groove: ok = genGroove(pl, op, *t, c.f, op.moves, error); break;
    case LatheOpKind::PartOff: ok = genPartOff(pl, op, *t, c.f, op.moves, error); break;
    case LatheOpKind::Drill: ok = genDrill(pl, op, *t, c.f, op.moves, error); break;
    case LatheOpKind::Thread: ok = genThread(pl, op, *t, op.moves, error); break;
    }
    if (!ok) {
        op.moves.clear();
        return false;
    }
    op.usedSpeed = c.vc, op.usedFeed = c.f, op.usedRpm = c.rpm;
    op.generated = true;
    return true;
}

double latheSeconds(const LatheJob& job, const LatheOperation& op) {
    const LatheTool* t = job.tool(op.tool);
    if (!t) return 0;
    const LatheCut c = cutOf(job, op, *t);
    double s = 0, x = job.setup.stockDiameter * 0.5 + 10, z = job.setup.frontStock + 10;
    for (const LatheMove& m : op.moves) {
        const double l = std::hypot(m.x - x, m.z - z);
        if (m.kind == LatheMove::Rapid) s += l / std::max(job.setup.rapid, 1.0) * 60.0;
        else {
            const int n = std::max(1, static_cast<int>(std::ceil(l / 2.0)));
            for (int q = 0; q < n; ++q) {
                const double xm = x + (m.x - x) * (q + 0.5) / n;
                const double f = m.kind == LatheMove::Thread ? m.feed : std::max(m.feed, 1e-3);
                s += (l / n) / (f * rpmAt(job, c, xm)) * 60.0;
            }
        }
        x = m.x, z = m.z;
    }
    return s;
}

// --- simulation -----------------------------------------------------------------------

namespace {

// The lowest (outside) / highest (bore) radius a tool's shape reaches at station z over a
// move from a to b. Returns false where the move doesn't reach z.
struct Shape {
    enum Kind { Nose, Blade, Point, Drill } kind = Nose;
    double r = 0;     // nose radius, blade corner, drill radius
    double beta = 0;  // nose: the trailing edge's rise towards the front (radians)
    double edge = 8;  // nose: how far the trailing edge goes
    double half = 0;  // blade half width
    double cone = 0;  // drill: radius gained per mm up from the tip
    double span() const { return kind == Blade ? half : kind == Point ? r : kind == Drill ? 0.0 : std::max(r, edge); }
    // The edge's height above its reference at distance d along Z (inf: not there).
    double h(double d) const {
        if (kind == Nose) { // from the centre: the round nose, and in front of it the trailing edge
            const double t = r * std::sin(beta);
            if (d < -r || d > edge) return kInfL;
            if (d <= t) return -std::sqrt(std::max(0.0, r * r - d * d));
            return -r * std::cos(beta) + (d - t) * std::tan(beta);
        }
        d = std::fabs(d);
        switch (kind) {
        case Nose: break;
        case Blade: {
            if (d >= half) return kInfL;
            const double e = d - (half - r);
            return e > 0 ? r - std::sqrt(std::max(0.0, r * r - e * e)) : 0.0;
        }
        case Point: return d > r ? kInfL : d * std::sqrt(3.0); // a 60 degree V
        case Drill: return kInfL;
        }
        return kInfL;
    }
};

} // namespace

bool simulateLathe(LatheJob& job, const Body& part, LatheSimulation& out, std::string* error) {
    out = LatheSimulation{};
    LatheProfile prof;
    if (!latheProfile(job, part, prof, error)) return false;
    const Plan pl(job, prof);
    const Grid& g = pl.g;
    out.z0 = g.z0, out.dz = g.dz;
    out.outer.assign(g.n, 0.0), out.inner.assign(g.n, 0.0);
    for (std::size_t i = 0; i < g.n; ++i)
        if (g.zAt(i) <= pl.front + 1e-9) out.outer[i] = pl.stockR;
    auto volume = [&]() {
        double v = 0;
        for (std::size_t i = 0; i < g.n; ++i) v += kPiL * std::max(0.0, out.outer[i] * out.outer[i] - out.inner[i] * out.inner[i]) * g.dz;
        return v;
    };
    const double v0 = volume();
    std::vector<unsigned char> threaded(g.n, 0), threadedIn(g.n, 0); // a thread's cut is meant
    for (LatheOperation& op : job.ops) {
        if (!op.generated && !generateLatheOperation(job, part, op, error)) return false;
        const LatheTool* t = job.tool(op.tool);
        if (!t) continue;
        // Each tool comes from the tool change, off the bar's end and outside it.
        double x = pl.stockR + 10, z = pl.front + 10;
        out.seconds += latheSeconds(job, op) + job.setup.toolChange;
        Shape sh;
        bool bore = false;
        switch (t->type) {
        case LatheToolType::Turn: sh.kind = Shape::Nose, sh.r = t->noseRadius, sh.beta = (t->relief() + 2.0) * kPiL / 180.0; break;
        case LatheToolType::Bore: sh.kind = Shape::Nose, sh.r = t->noseRadius, sh.beta = (t->relief() + 2.0) * kPiL / 180.0, bore = true; break;
        case LatheToolType::Groove: sh.kind = Shape::Blade, sh.r = std::min(t->noseRadius, t->width * 0.5), sh.half = t->width * 0.5, bore = op.inside; break;
        case LatheToolType::Thread: sh.kind = Shape::Point, sh.r = 2.0, bore = op.inside; break;
        case LatheToolType::Drill:
            sh.kind = Shape::Drill, sh.r = t->diameter * 0.5;
            sh.cone = std::tan(std::clamp(t->tipAngle, 60.0, 180.0) * 0.5 * kPiL / 180.0);
            bore = true;
            break;
        }
        for (const LatheMove& m : op.moves) {
            const double ax = x, az = z, bx = m.x, bz = m.z;
            const double l = std::hypot(bx - ax, bz - az);
            (m.kind == LatheMove::Rapid ? out.rapidLength : out.cutLength) += l;
            x = bx, z = bz;
            if (l < 1e-12) continue;
            bool crashed = false;
            const double span = sh.kind == Shape::Drill ? sh.r / std::max(sh.cone, 1e-6) + 1.0 : sh.span();
            const long i0 = std::max(0L, static_cast<long>(std::floor((std::min(az, bz) - span - g.z0) / g.dz)));
            const long i1 = std::min(static_cast<long>(g.n) - 1, static_cast<long>(std::ceil((std::max(az, bz) + span - g.z0) / g.dz)));
            for (long ii = i0; ii <= i1; ++ii) {
                const std::size_t i = static_cast<std::size_t>(ii);
                const double zs = g.zAt(i);
                if (sh.kind == Shape::Drill) { // along the axis: the drill's radius at zs, deepest along the move
                    if (std::fabs(ax) > 1e-6 || std::fabs(bx) > 1e-6) break; // off the axis it is moving in the air
                    const double tipZ = std::min(az, bz);
                    if (zs < tipZ) continue;
                    const double r = std::min(sh.r, (zs - tipZ) * sh.cone);
                    if (r > out.inner[i] + 1e-9 && out.outer[i] > out.inner[i]) {
                        if (m.kind == LatheMove::Rapid && std::min(r, out.outer[i]) > out.inner[i] + 1e-3) crashed = true;
                        out.inner[i] = r;
                    }
                    continue;
                }
                // The tool's reference along the move: the radius its edge reaches at zs is
                // x(t) + h(zs - z(t)) (outside: the least; a bore: the most), convex in t.
                auto at = [&](double tt) {
                    const double zt = az + (bz - az) * tt, xt = ax + (bx - ax) * tt, hh = sh.h(zs - zt);
                    return hh >= kInfL ? kInfL : bore ? -(xt - hh) : xt + hh;
                };
                double lo = 0, hi = 1;
                if (std::fabs(bz - az) > 1e-12) { // only where the shape spans zs
                    const double s = sh.span();
                    double t0 = (zs - s - az) / (bz - az), t1 = (zs + s - az) / (bz - az);
                    if (t0 > t1) std::swap(t0, t1);
                    lo = std::max(0.0, t0), hi = std::min(1.0, t1);
                    if (lo > hi) continue;
                }
                double best = std::min(at(lo), at(hi));
                if (hi - lo > 1e-12) {
                    double u = lo, v = hi;
                    for (int it = 0; it < 40; ++it) {
                        const double p1 = u + (v - u) / 3, p2 = v - (v - u) / 3;
                        if (at(p1) < at(p2)) v = p2;
                        else u = p1;
                    }
                    best = std::min(best, at(0.5 * (u + v)));
                }
                if (!(best < kInfL)) continue;
                if (bore) {
                    const double r = -best;
                    if (r > out.inner[i] + 1e-9 && out.outer[i] > out.inner[i]) {
                        if (m.kind == LatheMove::Rapid && std::min(r, out.outer[i]) > out.inner[i] + 1e-3) crashed = true;
                        out.inner[i] = r;
                        if (t->type == LatheToolType::Thread) threadedIn[i] = 1;
                    }
                } else if (best < out.outer[i] - 1e-9 && out.outer[i] > out.inner[i]) {
                    if (m.kind == LatheMove::Rapid && out.outer[i] - std::max(best, out.inner[i]) > 1e-3) crashed = true;
                    out.outer[i] = std::max(best, 0.0);
                    if (t->type == LatheToolType::Thread) threaded[i] = 1;
                }
            }
            if (crashed) ++out.rapidCrashes;
        }
        if (op.kind == LatheOpKind::PartOff) out.partedOff = true;
    }
    out.removed = v0 - volume();
    // Against the part: cut inside it is a gouge; what is left over it, stock left.
    const double tolG = std::max(2.0 * job.setup.tolerance, 0.02);
    double sum = 0;
    std::size_t count = 0;
    for (std::size_t i = 0; i < g.n; ++i) {
        const double zs = g.zAt(i);
        if (zs < prof.z0 + 1e-9 || zs > -1e-9) continue; // the part's inside stations (its end faces: the facing and parting)
        const std::size_t k = std::min(prof.size() - 1, static_cast<std::size_t>(std::llround((zs - prof.z0) / prof.dz)));
        if (!prof.has[k]) continue;
        // Beside a shoulder a station may be either side of it: a gouge is cut below the
        // part on both sides, stock left is over the part on both.
        double oLo = kInfL, oHi = 0, inLo = kInfL, inHi = 0;
        for (std::size_t q = k > 0 ? k - 1 : 0; q <= std::min(prof.size() - 1, k + 1); ++q) {
            if (!prof.has[q]) continue;
            oLo = std::min(oLo, prof.outer[q]), oHi = std::max(oHi, prof.outer[q]);
            inLo = std::min(inLo, prof.inner[q]), inHi = std::max(inHi, prof.inner[q]);
        }
        double cut = 0;
        if (!threaded[i] && out.outer[i] < oLo - tolG) cut = std::max(cut, oLo - out.outer[i]);
        if (!threadedIn[i] && out.inner[i] > inHi + tolG) cut = std::max(cut, out.inner[i] - inHi);
        if (cut > 0) {
            ++out.gouges;
            out.deepestGouge = std::max(out.deepestGouge, cut);
        }
        const double left = std::max(0.0, out.outer[i] - oHi) + std::max(0.0, inLo - out.inner[i]);
        out.maxLeft = std::max(out.maxLeft, left);
        sum += left, ++count;
    }
    out.meanLeft = count ? sum / static_cast<double>(count) : 0.0;
    return true;
}

Body turnedStock(const LatheJob& job, const Body& part, const LatheSimulation& sim) {
    MeshData m;
    LatheFrame f;
    if (!latheFrame(job, part, f) || sim.outer.empty()) return meshBody("Stock (turned)", m);
    const Vec3 u = f.axis;
    // Two directions across the axis.
    Vec3 a = std::fabs(u.x) < 0.9f ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
    a = normalize(a - u * dot(a, u));
    const Vec3 b = cross(u, a);
    const std::size_t n = sim.outer.size();
    const std::size_t stride = std::max<std::size_t>(1, n / 800);
    constexpr int kSeg = 72;
    auto ring = [&](double z, double r, int j) {
        const double th = 2.0 * kPiL * j / kSeg;
        return f.origin + u * static_cast<float>(z) + a * static_cast<float>(r * std::cos(th)) + b * static_cast<float>(r * std::sin(th));
    };
    // Runs of stations with material; each a closed outline in (z, r): out along the
    // outside, back along the bore, revolved.
    std::size_t i = 0;
    while (i < n) {
        while (i < n && !(sim.outer[i] > sim.inner[i] + 1e-6)) ++i;
        if (i >= n) break;
        std::size_t j = i;
        while (j < n && sim.outer[j] > sim.inner[j] + 1e-6) ++j;
        std::vector<std::pair<double, double>> loop; // (z, r)
        auto zAt = [&](std::size_t q) { return sim.z0 + sim.dz * static_cast<double>(q); };
        loop.push_back({zAt(i), sim.inner[i]});
        for (std::size_t q = i; q < j; q += stride) loop.push_back({zAt(q), sim.outer[q]});
        loop.push_back({zAt(j - 1), sim.outer[j - 1]});
        loop.push_back({zAt(j - 1), sim.inner[j - 1]});
        for (std::size_t q = j - 1; q > i; q = q >= i + stride ? q - stride : i) loop.push_back({zAt(q), sim.inner[q]});
        for (std::size_t e = 0; e < loop.size(); ++e) {
            const auto [z0, r0] = loop[e];
            const auto [z1, r1] = loop[(e + 1) % loop.size()];
            if (std::fabs(z1 - z0) < 1e-12 && std::fabs(r1 - r0) < 1e-12) continue;
            const double nz = r1 - r0, nr = -(z1 - z0); // outward in (z, r) for this winding
            const double ln = std::hypot(nz, nr);
            for (int s = 0; s < kSeg; ++s) {
                const double th0 = 2.0 * kPiL * s / kSeg, th1 = 2.0 * kPiL * (s + 1) / kSeg, thm = 0.5 * (th0 + th1);
                const Vec3 nrm = normalize(u * static_cast<float>(nz / ln) + (a * static_cast<float>(std::cos(thm)) + b * static_cast<float>(std::sin(thm))) * static_cast<float>(nr / ln));
                const auto base = static_cast<std::uint32_t>(m.vertices.size());
                m.vertices.push_back({ring(z0, r0, s), nrm});
                m.vertices.push_back({ring(z1, r1, s), nrm});
                m.vertices.push_back({ring(z1, r1, s + 1), nrm});
                m.vertices.push_back({ring(z0, r0, s + 1), nrm});
                m.indices.insert(m.indices.end(), {base, base + 2, base + 1, base, base + 3, base + 2});
            }
        }
        i = j;
    }
    return meshBody("Stock (turned)", std::move(m), {0.55f, 0.70f, 0.86f});
}

// --- posts ----------------------------------------------------------------------------

const std::vector<std::string>& lathePosts() {
    static const std::vector<std::string> kPosts = {"fanuc", "haas", "linuxcnc"};
    return kPosts;
}

namespace {

// The programmed point for a move: the insert's imaginary tip, a blade's corner towards
// the chuck, a drill's point.
// Arcs through runs of points (z, x) where the points lie on a circle within `tol` and so
// does the path between them: for each point after the first, an arc to it (r > 0; ccw:
// counterclockwise with Z across and X up) or a straight move (r = 0).
struct ArcStep {
    double z = 0, x = 0, r = 0;
    bool ccw = false;
};

std::vector<ArcStep> fitArcs(const std::vector<std::pair<double, double>>& p, double tol) {
    std::vector<ArcStep> out;
    const std::size_t n = p.size();
    std::size_t i = 0;
    while (i + 1 < n) {
        std::size_t best = 0;
        ArcStep arc;
        for (std::size_t j = i + 3; j < n; ++j) {
            // The circle through the first, middle and last points.
            const std::size_t m = (i + j) / 2;
            const double ax = p[i].first, ay = p[i].second, bx = p[m].first, by = p[m].second, cx = p[j].first, cy = p[j].second;
            const double d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
            if (std::fabs(d) < 1e-12) break; // straight
            const double a2 = ax * ax + ay * ay, b2 = bx * bx + by * by, c2 = cx * cx + cy * cy;
            const double ux = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d, uy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d;
            const double r = std::hypot(ax - ux, ay - uy);
            if (r < 0.05 || r > 2000) break;
            const double turn = (bx - ax) * (cy - by) - (by - ay) * (cx - bx);
            bool ok = true;
            double swept = 0;
            for (std::size_t k = i; k <= j && ok; ++k) {
                if (std::fabs(std::hypot(p[k].first - ux, p[k].second - uy) - r) > tol) ok = false;
                if (k == i) continue;
                // Each step turns the same way, and its chord stays within the tolerance of the arc.
                const double a0 = std::atan2(p[k - 1].second - uy, p[k - 1].first - ux), a1 = std::atan2(p[k].second - uy, p[k].first - ux);
                double da = a1 - a0;
                while (da > kPiL) da -= 2 * kPiL;
                while (da < -kPiL) da += 2 * kPiL;
                if ((da > 0) != (turn > 0) || std::fabs(da) < 1e-12) ok = false;
                swept += std::fabs(da);
                const double chord = std::hypot(p[k].first - p[k - 1].first, p[k].second - p[k - 1].second);
                if (r - std::sqrt(std::max(0.0, r * r - chord * chord / 4)) > tol) ok = false;
            }
            if (!ok || swept > kPiL * 0.95) break; // R arcs stay under half a turn
            best = j;
            arc = {cx, cy, r, turn > 0};
        }
        if (best) {
            out.push_back(arc);
            i = best;
        } else {
            out.push_back({p[i + 1].first, p[i + 1].second, 0, false});
            ++i;
        }
    }
    return out;
}

void programmed(const LatheTool& t, const LatheMove& m, double& x, double& z) {
    x = m.x, z = m.z;
    switch (t.type) {
    case LatheToolType::Turn: x = m.x - t.noseRadius, z = m.z - t.noseRadius; break;
    case LatheToolType::Bore: x = m.x + t.noseRadius, z = m.z - t.noseRadius; break;
    case LatheToolType::Groove: z = m.z - t.width * 0.5; break;
    case LatheToolType::Thread:
    case LatheToolType::Drill: break;
    }
}

} // namespace

std::string postLathe(LatheJob& job, const Body& part, const std::string& post, std::string* error) {
    const bool lcnc = post == "linuxcnc";
    if (std::find(lathePosts().begin(), lathePosts().end(), post) == lathePosts().end()) {
        if (error) *error = "Lathe posts: fanuc, haas, linuxcnc (" + post + " doesn't turn)";
        return {};
    }
    for (LatheOperation& op : job.ops)
        if (!op.generated && !generateLatheOperation(job, part, op, error)) return {};
    if (job.ops.empty()) {
        if (error) *error = "No lathe operations";
        return {};
    }
    std::ostringstream o;
    const char* nl = "\n";
    auto n = [&](double v) {
        // Fanuc-style controls read X10 as 10 microns: always a decimal point.
        char b[48];
        std::snprintf(b, sizeof b, "%.4f", std::fabs(v) < 5e-5 ? 0.0 : v);
        std::string s = b;
        while (!s.empty() && s.back() == '0') s.pop_back();
        if (lcnc && !s.empty() && s.back() == '.') s.pop_back();
        return s;
    };
    LatheProfile prof;
    if (!latheProfile(job, part, prof, error)) return {};
    o << "%" << nl;
    if (!lcnc) o << (post == "haas" ? "O01000" : "O1000") << " (" << job.setup.body << ")" << nl;
    else o << "(" << job.setup.body << ")" << nl;
    o << "(TURNED FROM A " << n(latheStockDiameter(job, prof)) << " MM BAR, " << job.setup.material << ")" << nl;
    o << "(Z0 AT THE FRONT FACE, X DIAMETER)" << nl;
    for (const LatheTool& t : job.tools) {
        std::string d = t.describe();
        for (char& c : d)
            if (c == '(' || c == ')') c = ' ';
        o << "(" << d << ")" << nl;
    }
    o << (lcnc ? "G7 G18 G21 G40 G90 G95" : "G21 G18 G40 G80 G99") << nl;
    for (std::size_t k = 0; k < job.ops.size(); ++k) {
        const LatheOperation& op = job.ops[k];
        const LatheTool* t = job.tool(op.tool);
        if (!t || op.moves.empty()) continue;
        const LatheCut c = cutOf(job, op, *t);
        o << "(" << (k + 1) << " " << (op.name.empty() ? std::string(latheOpKey(op.kind)) : op.name) << ")" << nl;
        if (lcnc) o << "G28" << nl << "T" << t->number << " M6 G43" << nl;
        else {
            char tt[16];
            std::snprintf(tt, sizeof tt, "T%02d%02d", t->number, t->number);
            o << "G28 U0. W0." << nl << tt << nl;
        }
        const bool css = c.rpm <= 0;
        if (css) {
            if (lcnc) o << "G96 D" << n(job.setup.maxRpm) << " S" << n(std::round(c.vc)) << " M3" << nl;
            else o << "G50 S" << static_cast<long>(std::lround(job.setup.maxRpm)) << nl << "G96 S" << static_cast<long>(std::lround(c.vc)) << " M03" << nl;
        } else o << (lcnc ? "G97 S" : "G97 S") << static_cast<long>(std::lround(c.rpm)) << (lcnc ? " M3" : " M03") << nl;
        if (op.kind != LatheOpKind::Thread) o << (lcnc ? "M8" : "M08") << nl;
        double lastF = -1, px = 0, pz = 0;
        int lastG = -1;
        for (std::size_t mi = 0; mi < op.moves.size();) {
            const LatheMove& m = op.moves[mi];
            double x = 0, z = 0;
            programmed(*t, m, x, z);
            std::string w;
            if (m.kind == LatheMove::Feed && mi > 0) {
                // A run of cuts at one feed: arcs where they lie on circles (the round corners
                // and curves of a profile), straight moves elsewhere. X stays a diameter; R is
                // the arc's radius; G2 turns clockwise with Z across and X up (seen from +Y).
                std::vector<std::pair<double, double>> pts{{pz, px}};
                std::size_t e = mi;
                for (; e < op.moves.size() && op.moves[e].kind == LatheMove::Feed && std::fabs(op.moves[e].feed - m.feed) < 1e-12; ++e) {
                    double qx = 0, qz = 0;
                    programmed(*t, op.moves[e], qx, qz);
                    pts.push_back({qz, qx});
                }
                const std::vector<ArcStep> steps = fitArcs(pts, std::max(job.setup.tolerance, 0.001));
                for (const ArcStep& st : steps) {
                    const int g = st.r > 0 ? (st.ccw ? 3 : 2) : 1;
                    std::string line;
                    if (g != lastG) line += lcnc ? "G" + std::to_string(g) + " " : "G0" + std::to_string(g) + " ";
                    lastG = g;
                    line += "X" + n(2 * st.x) + " Z" + n(st.z);
                    if (st.r > 0) line += " R" + n(st.r);
                    if (std::fabs(m.feed - lastF) > 1e-9) line += " F" + n(m.feed), lastF = m.feed;
                    o << line << nl;
                }
                px = pts.back().second, pz = pts.back().first;
                mi = e;
                continue;
            }
            if (m.kind == LatheMove::Rapid) {
                if (lastG != 0) w += lcnc ? "G0 " : "G00 ";
                lastG = 0;
                w += "X" + n(2 * x) + " Z" + n(z);
            } else if (m.kind == LatheMove::Thread) {
                w += (lcnc ? "G33 X" : "G32 X") + n(2 * x) + " Z" + n(z) + (lcnc ? " K" : " F") + n(m.feed);
                lastG = 32, lastF = -1;
            } else {
                if (lastG != 1) w += lcnc ? "G1 " : "G01 ";
                lastG = 1;
                w += "X" + n(2 * x) + " Z" + n(z);
                if (std::fabs(m.feed - lastF) > 1e-9) w += " F" + n(m.feed), lastF = m.feed;
            }
            o << w << nl;
            px = x, pz = z;
            ++mi;
        }
        o << (lcnc ? "M9" : "M09") << nl << (lcnc ? "M5" : "M05") << nl;
        if (css && !lcnc) o << "G97" << nl;
    }
    o << (lcnc ? "G28" : "G28 U0. W0.") << nl << (lcnc ? "M2" : "M30") << nl << "%" << nl;
    return o.str();
}

// --- JSON -----------------------------------------------------------------------------

Json latheToJson(const LatheJob& job) {
    Json j = Json::object();
    const LatheSetup& su = job.setup;
    Json s = Json::object();
    s.set("body", su.body).set("axis", su.axis).set("flip", su.flip).set("stock_diameter", su.stockDiameter).set("front_stock", su.frontStock);
    s.set("material", su.material).set("post", su.post).set("max_rpm", su.maxRpm).set("rapid", su.rapid).set("clearance", su.clearance);
    s.set("tolerance", su.tolerance).set("tool_change", su.toolChange);
    j.set("setup", s);
    Json tools = Json::array();
    for (const LatheTool& t : job.tools) {
        Json o = Json::object();
        o.set("number", t.number).set("type", latheToolTypeKey(t.type)).set("nose_radius", t.noseRadius).set("insert_angle", t.insertAngle);
        o.set("width", t.width).set("diameter", t.diameter).set("tip_angle", t.tipAngle).set("reach", t.reach).set("hss", t.hss);
        if (!t.name.empty()) o.set("name", t.name);
        if (t.vc > 0) o.set("vc", t.vc);
        if (t.feedPerRev > 0) o.set("feed_per_rev", t.feedPerRev);
        tools.push(o);
    }
    j.set("tools", tools);
    Json ops = Json::array();
    for (const LatheOperation& op : job.ops) {
        Json o = Json::object();
        o.set("kind", latheOpKey(op.kind)).set("tool", op.tool);
        if (!op.name.empty()) o.set("name", op.name);
        if (op.inside) o.set("inside", true);
        if (op.stepdown > 0) o.set("stepdown", op.stepdown);
        if (op.leave != 0) o.set("leave", op.leave);
        if (op.leaveZ != 0) o.set("leave_z", op.leaveZ);
        if (!std::isnan(op.zFrom)) o.set("z_from", op.zFrom);
        if (!std::isnan(op.zTo)) o.set("z_to", op.zTo);
        if (op.depth > 0) o.set("depth", op.depth);
        if (op.peck > 0) o.set("peck", op.peck);
        if (op.pitch > 0) o.set("pitch", op.pitch);
        if (op.threadDiameter > 0) o.set("thread_diameter", op.threadDiameter);
        if (op.speed > 0) o.set("speed", op.speed);
        if (op.feedPerRev > 0) o.set("feed_per_rev", op.feedPerRev);
        if (op.rpm > 0) o.set("rpm", op.rpm);
        ops.push(o);
    }
    j.set("ops", ops);
    j.set("show_paths", job.showPaths);
    return j;
}

bool latheFromJson(const Json& j, LatheJob& job) {
    if (!j.isObject()) return false;
    job = LatheJob{};
    const Json& s = j["setup"];
    LatheSetup& su = job.setup;
    su.body = s["body"].asString(std::string());
    su.axis = std::clamp(s["axis"].asInt(0), 0, 2);
    su.flip = s["flip"].asBool(false);
    su.stockDiameter = s["stock_diameter"].asNumber(0);
    su.frontStock = s["front_stock"].asNumber(1);
    su.material = s["material"].asString(std::string("aluminium"));
    su.post = s["post"].asString(std::string("fanuc"));
    su.maxRpm = s["max_rpm"].asNumber(4000);
    su.rapid = s["rapid"].asNumber(10000);
    su.clearance = s["clearance"].asNumber(2);
    su.tolerance = s["tolerance"].asNumber(0.01);
    su.toolChange = s["tool_change"].asNumber(2);
    for (const Json& o : j["tools"].items()) {
        LatheTool t;
        t.number = o["number"].asInt(1);
        if (!latheToolTypeFromKey(o["type"].asString(std::string("turn")), &t.type)) t.type = LatheToolType::Turn;
        t.noseRadius = o["nose_radius"].asNumber(0.4);
        t.insertAngle = o["insert_angle"].asNumber(55);
        t.width = o["width"].asNumber(3);
        t.diameter = o["diameter"].asNumber(10);
        t.tipAngle = o["tip_angle"].asNumber(118);
        t.reach = o["reach"].asNumber(50);
        t.hss = o["hss"].asBool(false);
        t.name = o["name"].asString(std::string());
        t.vc = o["vc"].asNumber(0);
        t.feedPerRev = o["feed_per_rev"].asNumber(0);
        job.tools.push_back(t);
    }
    for (const Json& o : j["ops"].items()) {
        LatheOperation op;
        const std::string k = o["kind"].asString(std::string("rough"));
        for (LatheOpKind q : {LatheOpKind::Face, LatheOpKind::Rough, LatheOpKind::Finish, LatheOpKind::Groove, LatheOpKind::Drill, LatheOpKind::Thread, LatheOpKind::PartOff})
            if (k == latheOpKey(q)) op.kind = q;
        op.tool = o["tool"].asInt(1);
        op.name = o["name"].asString(std::string());
        op.inside = o["inside"].asBool(false);
        op.stepdown = o["stepdown"].asNumber(0);
        op.leave = o["leave"].asNumber(0);
        op.leaveZ = o["leave_z"].asNumber(0);
        if (o["z_from"].isNumber()) op.zFrom = o["z_from"].asNumber();
        if (o["z_to"].isNumber()) op.zTo = o["z_to"].asNumber();
        op.depth = o["depth"].asNumber(0);
        op.peck = o["peck"].asNumber(0);
        op.pitch = o["pitch"].asNumber(0);
        op.threadDiameter = o["thread_diameter"].asNumber(0);
        op.speed = o["speed"].asNumber(0);
        op.feedPerRev = o["feed_per_rev"].asNumber(0);
        op.rpm = o["rpm"].asNumber(0);
        job.ops.push_back(op);
    }
    job.showPaths = j["show_paths"].asBool(true);
    return true;
}

} // namespace sicdm

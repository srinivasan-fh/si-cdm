#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Physics.hpp"
#include "core/Scene.hpp"

#include <cmath>
#include <cstdio>
#include <filesystem>

using namespace sicdm;

#define REQUIRE(c)               \
    do {                         \
        const bool required_ = (c); \
        CHECK(required_);        \
        if (!required_) return;  \
    } while (0)

namespace {

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }

int bodyIndex(const PhysicsRun& r, const Scene& s, const std::string& name) {
    for (std::size_t i = 0; i < s.bodies().size(); ++i)
        if (s.bodies()[i].name == name) return r.indexOf(i);
    return -1;
}

// Largest change of the conserved total over the run, against the energies involved.
double balanceDrift(const PhysicsRun& r) {
    const double start = r.frames.front().energy.balance();
    double scale = 1e-12, worst = 0;
    for (const PhysicsFrame& f : r.frames) {
        const PhysicsEnergy& e = f.energy;
        scale = std::max(scale, std::fabs(e.kinetic) + std::fabs(e.gravity) + std::fabs(e.electric) + std::fabs(e.externalWork));
        worst = std::max(worst, std::fabs(e.balance() - start));
    }
    return worst / scale;
}

} // namespace

TEST_CASE("free fall and projectiles follow the kinematic equations exactly") {
    Document d;
    REQUIRE(run(d, "SPHERE 20 @0,0,1000").ok);
    REQUIRE(run(d, "PHYSICS BODY \"Sphere 1\" DYNAMIC VELOCITY 500,0,2000").ok);
    REQUIRE(d.scene.physics().enabled);
    const auto r = physicsRun(d.scene);
    REQUIRE(r != nullptr);
    const int b = bodyIndex(*r, d.scene, "Sphere 1");
    REQUIRE(b >= 0);
    const Vec3 x0 = r->frames.front().bodies[static_cast<std::size_t>(b)].position;
    const double g = kStandardGravity;
    for (float t : {0.1f, 0.3f, 0.5f}) { // before it lands
        const PhysicsBodyState s = r->at(t).bodies[static_cast<std::size_t>(b)];
        CHECK_NEAR(s.position.x, x0.x + 500 * t, 1e-2);                                       // constant horizontal speed
        CHECK_NEAR(s.position.z, x0.z + 2000 * t - 0.5 * g * 1000 * t * t, 1e-2);              // z = z0 + v t - g t^2 / 2
        CHECK_NEAR(s.velocity.z, 2000 - g * 1000 * t, 1e-2);                                  // v = v0 - g t
        CHECK_NEAR(s.acceleration.z, -g * 1000, 1.0);                                         // a = -g
    }
    // Mass from volume x density: a 20 mm ball of water.
    CHECK_NEAR(r->bodies[static_cast<std::size_t>(b)].mass, 4.0 / 3.0 * kPi * 0.02 * 0.02 * 0.02 * 1000, 1e-3);
}

TEST_CASE("collisions conserve momentum; elastic ones kinetic energy too") {
    Document d;
    REQUIRE(run(d, "PHYSICS DEMO collision").ok);
    const auto r = physicsRun(d.scene);
    REQUIRE(r != nullptr);
    const Vec3 p0 = r->frames.front().momentum;
    for (const PhysicsFrame& f : r->frames) CHECK(length(f.momentum - p0) < 1e-6f * length(p0) + 1e-9f);
    CHECK(r->collisions >= 4);
    CHECK(balanceDrift(*r) < 1e-6);
    // Newton's cradle: the striker stops and the last ball leaves with its speed.
    const PhysicsFrame end = r->frames.back();
    CHECK(length(end.bodies[static_cast<std::size_t>(bodyIndex(*r, d.scene, "Striker"))].velocity) < 1.0f);
    CHECK_NEAR(end.bodies[static_cast<std::size_t>(bodyIndex(*r, d.scene, "Ball 4"))].velocity.x, 800, 0.5);
}

TEST_CASE("Newton's gravitation holds a circular orbit and conserves energy") {
    Document d;
    REQUIRE(run(d, "PHYSICS DEMO orbit").ok);
    const auto r = physicsRun(d.scene);
    REQUIRE(r != nullptr);
    const std::size_t moon = static_cast<std::size_t>(bodyIndex(*r, d.scene, "Moon"));
    const std::size_t planet = static_cast<std::size_t>(bodyIndex(*r, d.scene, "Planet"));
    const double radius = 600, v = std::sqrt(kGravitationalConstant * 1e10 / 0.6) * 1000;
    for (const PhysicsFrame& f : r->frames) {
        CHECK(std::fabs(length(f.bodies[moon].position - f.bodies[planet].position) - radius) < 0.005 * radius);
        CHECK(std::fabs(length(f.bodies[moon].velocity) - v) < 0.005 * v);
    }
    CHECK(balanceDrift(*r) < 1e-5);
    // Period 2 pi r / v: back where it started.
    const float period = static_cast<float>(2 * kPi * radius / v);
    CHECK(length(r->at(period).bodies[moon].position - r->frames.front().bodies[moon].position) < 0.01f * static_cast<float>(radius));
}

TEST_CASE("Coulomb's law and the Lorentz force") {
    Document d;
    REQUIRE(run(d, "PHYSICS DEMO charges").ok);
    auto r = physicsRun(d.scene);
    REQUIRE(r != nullptr);
    // Plus and Minus 600 mm apart, Plus 2 at (0, 500): the force on Minus at t = 0.
    const std::size_t minus = static_cast<std::size_t>(bodyIndex(*r, d.scene, "Minus"));
    const double q = 5e-6, k = kCoulombConstant;
    const Vec3 dPlus{-0.6f, 0, 0}, dPlus2{-0.3f, 0.5f, 0};
    const Vec3 expect = (normalize(dPlus) * static_cast<float>(k * q * q / 0.36) +
                         normalize(dPlus2) * static_cast<float>(k * q * q / (0.09 + 0.25)));
    CHECK(length(r->frames.front().bodies[minus].force - expect) < 1e-3f * length(expect));
    // Momentum stays zero (internal forces only).
    for (const PhysicsFrame& f : r->frames) CHECK(length(f.momentum) < 1e-9f);

    // A charge in a magnetic field circles at r = m v / (q B) at constant speed.
    Document c;
    REQUIRE(run(c, "PHYSICS DEMO cyclotron").ok);
    r = physicsRun(c.scene);
    REQUIRE(r != nullptr);
    const Vec3 centre{0, -400, 400}; // the start (0, -200) turned towards -y by q v x B
    for (const PhysicsFrame& f : r->frames) {
        CHECK_NEAR(length(f.bodies[0].velocity), 1000, 1e-2);
        CHECK_NEAR(length(f.bodies[0].position - centre), 200, 0.5); // 0.25 mm: the half-step phase
    }
    CHECK(balanceDrift(*r) < 1e-6); // the magnetic force does no work
}

TEST_CASE("bounces turn kinetic energy into heat and the total is conserved") {
    Document d;
    REQUIRE(run(d, "PHYSICS DEMO drop").ok);
    const auto r = physicsRun(d.scene);
    REQUIRE(r != nullptr);
    CHECK(balanceDrift(*r) < 1e-3);
    const PhysicsEnergy& e = r->frames.back().energy;
    CHECK(e.thermal > 15.0); // most of the 19 J fall became heat
    CHECK(r->frames.back().bodies[0].temperature > r->frames.front().bodies[0].temperature);
    // The more elastic ball rebounds higher: its first rebound peak after the first impact.
    auto peak = [&](std::size_t b) {
        float best = 0, lowSeen = 1e9f;
        for (const PhysicsFrame& f : r->frames) {
            const float z = f.bodies[b].position.z;
            lowSeen = std::min(lowSeen, z);
            if (lowSeen < 60.0f) best = std::max(best, z);
            if (lowSeen < 60.0f && z < best - 1.0f) break;
        }
        return best;
    };
    CHECK(peak(0) > peak(1));
    CHECK(peak(1) > peak(2));
    // h' = e^2 h for a ball from 1 m (centre 50 mm above the floor at rest).
    CHECK_NEAR(peak(0) - 50, 0.81 * 1000, 25);
}

TEST_CASE("touching bodies reach the same temperature and the air cools them") {
    Document d;
    REQUIRE(run(d, "PHYSICS DEMO heat").ok);
    const auto r = physicsRun(d.scene);
    REQUIRE(r != nullptr);
    const PhysicsFrame& f = r->frames.back();
    CHECK(std::fabs(f.bodies[0].temperature - f.bodies[1].temperature) < 0.5f); // equilibrium
    CHECK(f.bodies[0].temperature < 323.15f && f.bodies[0].temperature > 320.0f); // just below the 50 C mean: cooled
    CHECK(f.energy.toAir > 0.0);
    CHECK(balanceDrift(*r) < 1e-3);
    // The stack stays put.
    CHECK(std::fabs(f.bodies[1].position.z - 150) < 0.1f);
}

TEST_CASE("special relativity: nothing reaches c and moving clocks run slow") {
    Document d;
    REQUIRE(run(d, "PHYSICS DEMO relativity").ok);
    const auto r = physicsRun(d.scene);
    REQUIRE(r != nullptr);
    const double c = 2.0, m = 1.0, F = 2.0;
    for (const PhysicsFrame& f : r->frames) {
        const double t = f.t, v = length(f.bodies[0].velocity) / 1000.0;
        CHECK(v < c);
        CHECK_NEAR(f.momentum.x, F * t, 1e-6);                       // dp/dt = F
        const double u = F * t / (m * c);
        CHECK_NEAR(v, c * u / std::sqrt(1 + u * u), 1e-6);           // v = p / sqrt(m^2 + p^2/c^2)
        CHECK_NEAR(f.bodies[0].properTime, m * c / F * std::asinh(u), 1e-4); // tau = (mc/F) asinh(Ft/mc)
        CHECK_NEAR(f.bodies[1].properTime, t, 1e-6);                 // the clock at rest
    }
    CHECK(balanceDrift(*r) < 1e-6);
    // Length contraction: the moving body is 1/gamma as long along its motion.
    const auto tf = simulationTransforms(d.scene, 5.0f);
    const Body& rocket = d.scene.bodies()[0];
    AABB box;
    for (const Vertex& v : rocket.mesh.vertices) box.expand(tf[0].transformPoint(v.position + rocket.offset));
    const double u = F * 5 / (m * c), gamma = std::sqrt(1 + u * u);
    CHECK_NEAR((box.max.x - box.min.x) / (box.max.y - box.min.y), 1.0 / gamma, 0.01);
}

TEST_CASE("waves: wavelength and Doppler shift, and the report") {
    Document d;
    REQUIRE(run(d, "PHYSICS DEMO doppler").ok);
    const CommandOutput o = run(d, "PHYSICS AT 1");
    REQUIRE(o.ok);
    auto said = [&](const std::string& text) {
        for (const std::string& l : o.lines)
            if (l.find(text) != std::string::npos) return true;
        return false;
    };
    CHECK(said("wavelength 0.2667 m"));
    CHECK(said("Conservation of energy"));
    CHECK(said("de Broglie wavelength"));
    // Source at (-600, 0) mm moving 0.4 m/s along x, waves at 0.8 m/s, 3 Hz: f c / (c - v.n),
    // n towards each listener 250 mm off the path.
    const double ahead = 3 * 0.8 / (0.8 - 0.4 * 1800 / std::hypot(1800.0, 250.0));
    const double behind = 3 * 0.8 / (0.8 + 0.4 * 800 / std::hypot(800.0, 250.0));
    char text[64];
    std::snprintf(text, sizeof(text), "at Listener ahead: %.4g Hz", ahead);
    CHECK(said(text));
    std::snprintf(text, sizeof(text), "at Listener behind: %.4g Hz", behind);
    CHECK(said(text));
    CHECK(ahead > 3 && behind < 3);
}

TEST_CASE("physics bodies move in the simulation, fixed ones stay, and it all saves") {
    Document d;
    REQUIRE(run(d, "SPHERE 20 @0,0,500").ok);
    REQUIRE(run(d, "BOX 100 100 10 @300,0,0").ok);
    REQUIRE(run(d, "PHYSICS BODY \"Sphere 1\" DYNAMIC CHARGE 1e-6 TEMP 80 BOUNCE 0.8 FORCE 0,0,0.01").ok);
    REQUIRE(run(d, "PHYSICS BODY \"Block 2\" FIXED MASS 5").ok);
    REQUIRE(run(d, "PHYSICS GRAVITY BOTH 3.7 G 1e-9").ok);
    REQUIRE(run(d, "PHYSICS FIELD E 0,0,100").ok);
    REQUIRE(run(d, "PHYSICS AIR ON").ok);
    REQUIRE(run(d, "PHYSICS RELATIVITY ON C 1000").ok);
    REQUIRE(run(d, "PHYSICS DURATION 2").ok);
    const auto tf = simulationTransforms(d.scene, 0.3f);
    CHECK(tf[0].transformPoint({0, 0, 0}).z < -100.0f); // falling (3.7 m/s^2 and a little E)
    CHECK(length(tf[1].transformPoint({1, 2, 3}) - Vec3{1, 2, 3}) < 1e-4f);
    const auto path = std::filesystem::temp_directory_path() / "sicdm_physics_test.sicdm";
    REQUIRE(run(d, "SAVE " + path.string()).ok);
    Document e;
    REQUIRE(run(e, "OPEN " + path.string()).ok);
    std::filesystem::remove(path);
    const PhysicsSettings& s = e.scene.physics();
    CHECK(s.enabled && s.gravity == GravityMode::Both && s.air && s.relativity);
    CHECK_NEAR(s.g.z, -3.7, 1e-5);
    CHECK_NEAR(s.bigG, 1e-9, 1e-15);
    CHECK_NEAR(s.electricField.z, 100, 1e-4);
    CHECK_NEAR(s.lightSpeed, 1000, 1e-3);
    const PhysicsProps& p = e.scene.bodies()[0].physics;
    CHECK(p.enabled && !p.fixed);
    CHECK_NEAR(p.charge, 1e-6, 1e-12);
    CHECK_NEAR(p.temperature, 353.15, 1e-3);
    CHECK_NEAR(p.restitution, 0.8, 1e-5);
    CHECK(e.scene.bodies()[1].physics.fixed);
    CHECK_NEAR(e.scene.bodies()[1].physics.mass, 5, 1e-5);
    const auto tf2 = simulationTransforms(e.scene, 0.3f);
    CHECK(length(tf2[0].transformPoint({0, 0, 0}) - tf[0].transformPoint({0, 0, 0})) < 1e-3f);
    // Turning physics off leaves the bodies where they are.
    REQUIRE(run(e, "PHYSICS OFF").ok);
    CHECK(physicsRun(e.scene) == nullptr);
    CHECK(length(simulationTransforms(e.scene, 0.3f)[0].transformPoint({0, 0, 0})) < 1e-6f);
    CHECK(!run(e, "PHYSICS BODY nothing DYNAMIC").ok);
    CHECK(!run(e, "PHYSICS DEMO warp").ok);
    CHECK(run(e, "PHYSICS DEMO").ok);
}

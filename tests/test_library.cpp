#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/FileIO.hpp"
#include "core/Library.hpp"
#include "core/Scene.hpp"

#include <cmath>
#include <filesystem>
#include <map>
#include <tuple>

using namespace sicdm;

#define REQUIRE(c)               \
    do {                         \
        const bool required_ = (c); \
        CHECK(required_);        \
        if (!required_) return;  \
    } while (0)

namespace {

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }

// Closed solid: after welding, every edge is shared by exactly two triangles.
bool watertight(const MeshData& m) {
    std::map<std::tuple<long, long, long>, std::uint32_t> weld;
    std::vector<std::uint32_t> id(m.vertices.size());
    for (std::size_t i = 0; i < m.vertices.size(); ++i) {
        const Vec3& p = m.vertices[i].position;
        const auto k = std::make_tuple(std::lround(p.x * 1e3f), std::lround(p.y * 1e3f), std::lround(p.z * 1e3f));
        id[i] = weld.emplace(k, static_cast<std::uint32_t>(weld.size())).first->second;
    }
    std::map<std::pair<std::uint32_t, std::uint32_t>, int> edges;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3)
        for (int e = 0; e < 3; ++e) {
            auto a = id[m.indices[t + e]], b = id[m.indices[t + (e + 1) % 3]];
            if (a == b) continue;
            if (a > b) std::swap(a, b);
            ++edges[{a, b}];
        }
    for (const auto& [e, n] : edges)
        if (n != 2) return false;
    return !edges.empty();
}

const Body* body(const Scene& s, const std::string& name) {
    for (const Body& b : s.bodies())
        if (b.name == name) return &b;
    return nullptr;
}
const Motion* motion(const Scene& s, const std::string& name) {
    for (const Motion& m : s.motions())
        if (m.name == name) return &m;
    return nullptr;
}

} // namespace

TEST_CASE("involute gear outline is star-shaped between root and tip circles") {
    const float m = 2.0f;
    const int z = 20;
    const auto pts = gearOutline(m, z, 0.0f);
    REQUIRE(pts.size() > static_cast<std::size_t>(z * 10));
    const float rp = m * z * 0.5f, ra = rp + m, rf = rp - 1.25f * m;
    float prev = -1e9f, unwrap = 0.0f;
    bool monotone = true;
    float maxR = 0.0f, minR = 1e9f;
    for (const Vec2& p : pts) {
        float a = std::atan2(p.y, p.x) + unwrap;
        if (a < prev - kPi) {
            unwrap += 2.0f * kPi;
            a += 2.0f * kPi;
        }
        if (a < prev - 1e-5f) monotone = false;
        prev = a;
        const float r = std::hypot(p.x, p.y);
        maxR = std::max(maxR, r);
        minR = std::min(minR, r);
    }
    CHECK(monotone);
    CHECK_NEAR(maxR, ra, 1e-3);
    CHECK_NEAR(minR, rf, 1e-3);
    // Tooth 0 is centred on +X: the tip there is at the addendum radius.
    CHECK_NEAR(std::hypot(pts[pts.size() / static_cast<std::size_t>(z) / 2].x, pts[pts.size() / static_cast<std::size_t>(z) / 2].y), ra, 0.2);
}

TEST_CASE("library gears and motors build closed solids") {
    const std::vector<std::pair<std::string, std::vector<float>>> parts = {
        {"gear", {2, 20, 10, 4, 0, 0, 0}},  {"gear", {2, 24, 10, 4, 20, 1, 0}}, {"gear", {1.5f, 30, 14, 0, 25, 2, 0}},
        {"ring_gear", {2, 48, 10, 6, 0}},   {"bevel_gear", {2, 20, 8, 45, 4, 0}}, {"rack", {2, 12, 10, 8}},
    };
    for (const auto& [kind, params] : parts) {
        const MeshData m = buildLibraryPart(kind, params);
        CHECK(!m.indices.empty());
        if (!watertight(m)) std::fprintf(stderr, "  not watertight: %s %g\n", kind.c_str(), static_cast<double>(params[1]));
        CHECK(watertight(m));
    }
    const MeshData spur = buildLibraryPart("gear", {2, 20, 10, 4, 0, 0, 0});
    const AABB b = spur.bounds();
    CHECK_NEAR(b.max.x - b.min.x, 2.0f * (20 + 2), 0.2); // tip diameter m (z + 2)
    CHECK_NEAR(b.max.z, 10, 1e-4);
    for (const char* k : {"dc_motor", "stepper_motor", "servo_motor", "worm"})
        CHECK(!buildLibraryPart(k, {2, 1, 40, 0}).indices.empty());
    for (float style : {0.0f, 1.0f, 2.0f}) CHECK(!buildLibraryPart("motor_rotor", {style}).indices.empty());
    CHECK(buildLibraryPart("gear", {2, 2, 10}).indices.empty()); // too few teeth
    CHECK(isPrimitiveKind("gear") && isPrimitiveKind("servo_motor"));
}

TEST_CASE("motors come with a 360 degree drive on the rotor; steppers step") {
    Document d;
    REQUIRE(run(d, "PART DCMOTOR").ok);
    REQUIRE(d.scene.bodies().size() == 2);
    CHECK(d.scene.bodies()[0].name == "DC Motor 1");
    CHECK(d.scene.bodies()[1].name == "DC Motor 1 Rotor");
    CHECK(d.scene.bodies()[1].component == "DC Motor 1");
    const Motion* drive = motion(d.scene, "DC Motor 1 Drive");
    REQUIRE(drive);
    CHECK(drive->kind == MotionKind::Circular);
    CHECK(drive->mode == MotionMode::Loop);
    CHECK_NEAR(drive->range, 360, 1e-6);
    CHECK_NEAR(drive->speed, 720, 1e-4); // 120 rpm
    CHECK(drive->moves("DC Motor 1 Rotor") && !drive->moves("DC Motor 1"));

    REQUIRE(run(d, "PART STEPPER RPM 10 STEP 1.8").ok);
    const Motion* step = motion(d.scene, "Stepper Motor 1 Drive");
    REQUIRE(step);
    CHECK_NEAR(step->step, 1.8, 1e-6);
    CHECK_NEAR(motionValue(*step, 0.1f), 5.4, 1e-3); // 6 deg after 0.1 s at 60 deg/s -> 3 whole steps
    // Placed beside the first motor, not on top of it.
    CHECK(body(d.scene, "Stepper Motor 1")->bounds().min.x > body(d.scene, "DC Motor 1")->bounds().max.x);

    REQUIRE(run(d, "PART SERVO NAME Pan SWEEP 180 @0,200,0").ok);
    const Motion* pan = motion(d.scene, "Pan Drive");
    REQUIRE(pan);
    CHECK(pan->mode == MotionMode::Oscillate);
    CHECK_NEAR(pan->range, 180, 1e-6);
    CHECK_NEAR(body(d.scene, "Pan")->offset.y, 200, 1e-6);

    CHECK(!run(d, "PART WARPDRIVE").ok);
    CHECK(!run(d, "PART SPUR TEETH 2").ok);   // below the minimum
    CHECK(!run(d, "PART SPUR COLOUR 3 X").ok);  // unknown parameter
    CHECK(!run(d, "PART DCMOTOR NAME Pan").ok); // name taken
    CHECK(run(d, "PART").lines.size() > 10);    // the catalogue
}

TEST_CASE("gear trains link their motions: pairs, rack and pinion, planetary") {
    Document d;
    REQUIRE(run(d, "PART GEARPAIR TEETH1 15 TEETH2 45 RPM 10").ok);
    const Motion* drive = motion(d.scene, "Gear Pair 1 Driver Drive");
    const Motion* mesh = motion(d.scene, "Gear Pair 1 Driven Mesh");
    REQUIRE(drive && mesh);
    CHECK_NEAR(motionValue(d.scene, *mesh, 1.0f), -motionValue(*drive, 1.0f) / 3.0f, 1e-3);
    const Body* driven = body(d.scene, "Gear Pair 1 Driven");
    REQUIRE(driven);
    CHECK_NEAR(driven->offset.x - body(d.scene, "Gear Pair 1 Driver")->offset.x, 2.0f * (15 + 45) / 2.0f, 1e-3);

    Document r;
    REQUIRE(run(r, "PART RACKPINION TEETH 16 MODULE 2").ok);
    const Motion* slide = motion(r.scene, "Rack and Pinion 1 Rack Slide");
    const Motion* pin = motion(r.scene, "Rack and Pinion 1 Pinion Drive");
    REQUIRE(slide && pin);
    const float rp = 16.0f; // m z / 2
    CHECK_NEAR(motionValue(r.scene, *slide, 0.5f), radians(motionValue(*pin, 0.5f)) * rp, 1e-3);

    Document p;
    REQUIRE(run(p, "PART PLANETARY SUN_TEETH 18 PLANET_TEETH 18").ok);
    CHECK(p.scene.bodies().size() == 5); // sun, 3 planets, ring
    const Motion* carrier = motion(p.scene, "Planetary Gear Set 1 Carrier");
    REQUIRE(carrier);
    CHECK_NEAR(carrier->ratio, 18.0f / (18.0f + 54.0f), 1e-5);
    // A planet's centre orbits with the carrier.
    const auto tf = motionTransforms(p.scene, 1.5f);
    const Body* planet = body(p.scene, "Planetary Gear Set 1 Planet 1");
    REQUIRE(planet);
    std::size_t pi = 0;
    while (&p.scene.bodies()[pi] != planet) ++pi;
    const Vec3 c = tf[pi].transformPoint(planet->bounds().center());
    const float orbit = std::atan2(c.y, c.x);
    const float expected = radians(motionValue(p.scene, *carrier, 1.5f));
    CHECK_NEAR(std::remainder(orbit - expected, 2.0f * kPi), 0, 1e-3);
    CHECK_NEAR(std::hypot(c.x, c.y), 1.5f * (18 + 18) / 2.0f, 1e-2);

    CHECK(run(p, "PART WORMDRIVE").ok);
    CHECK(!run(p, "MOTION CIRCLE \"Planetary Gear Set 1 Carrier\" FOLLOW \"Planetary Gear Set 1 Carrier\"").ok);
}

TEST_CASE("SCALE resizes a component with its motions") {
    Document d;
    REQUIRE(run(d, "PART GEARPAIR @0,0,0").ok);
    const Body* driven = body(d.scene, "Gear Pair 1 Driven");
    REQUIRE(driven);
    const float gap = driven->offset.x;
    const float width = driven->bounds().max.x - driven->bounds().min.x;
    REQUIRE(run(d, "SCALE \"Gear Pair 1 Driven\" 2").ok);
    driven = body(d.scene, "Gear Pair 1 Driven");
    CHECK_NEAR(driven->offset.x, 2 * gap, 1e-3);                                  // centre distance doubles
    CHECK_NEAR(driven->bounds().max.x - driven->bounds().min.x, 2 * width, 1e-2); // and the gear itself
    CHECK_NEAR(body(d.scene, "Gear Pair 1 Driver")->scale, 2, 1e-6);
    CHECK_NEAR(motion(d.scene, "Gear Pair 1 Driven Mesh")->origin.x, 2 * gap, 1e-3);
    REQUIRE(run(d, "SCALE \"Gear Pair 1 Driver\" TO 1").ok);
    CHECK_NEAR(body(d.scene, "Gear Pair 1 Driven")->offset.x, gap, 1e-3);
    CHECK(!run(d, "SCALE Nobody 2").ok);
    CHECK(!run(d, "SCALE 0").ok);
    // Plain bodies scale too.
    REQUIRE(run(d, "BOX 10 10 10").ok);
    REQUIRE(run(d, "SCALE 3").ok); // the selection
    const Body& box = d.scene.bodies().back();
    CHECK_NEAR(box.bounds().max.z - box.bounds().min.z, 30, 1e-3);
}

TEST_CASE("library components survive save and reopen") {
    Document d;
    REQUIRE(run(d, "PART STEPPER SIZE 1.5").ok);
    REQUIRE(run(d, "PART HELICAL TEETH 24 HELIX 15").ok);
    REQUIRE(run(d, "PART GEARPAIR").ok);
    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_library_test.sicdm").string();
    std::string err;
    REQUIRE(saveProject(d.scene, d.state, path, &err));
    Scene s;
    ProjectState st;
    REQUIRE(loadProject(s, st, path, &err, nullptr));
    REQUIRE(s.bodies().size() == d.scene.bodies().size());
    for (std::size_t i = 0; i < s.bodies().size(); ++i) {
        const Body &a = d.scene.bodies()[i], &b = s.bodies()[i];
        CHECK(a.kind == b.kind);
        CHECK(a.component == b.component);
        CHECK_NEAR(a.scale, b.scale, 1e-6);
        CHECK(a.mesh.indices.size() == b.mesh.indices.size());
        CHECK_NEAR(a.bounds().max.z, b.bounds().max.z, 1e-3);
    }
    const Motion* mesh = motion(s, "Gear Pair 1 Driven Mesh");
    REQUIRE(mesh);
    CHECK(mesh->follow == "Gear Pair 1 Driver Drive");
    CHECK_NEAR(mesh->ratio, -0.5, 1e-6);
    CHECK_NEAR(motion(s, "Stepper Motor 1 Drive")->step, 1.8, 1e-6);
}

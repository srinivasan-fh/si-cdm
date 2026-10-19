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

TEST_CASE("motor shafts: none, single or dual, with configurable lengths") {
    auto rotor = [](std::vector<float> p) { return buildLibraryPart("motor_rotor", p).bounds(); };
    // Stepper: boss face at z = 42, rear face at z = 0.
    const AABB single = rotor({1, 30, 1, 15, 5, 1});
    CHECK_NEAR(single.max.z, 72, 1e-3);
    CHECK_NEAR(single.min.z, 42, 1e-3);
    const AABB dual = rotor({1, 30, 2, 20, 5, 0});
    CHECK_NEAR(dual.min.z, -20, 1e-3);
    CHECK_NEAR(dual.max.z, 72, 1e-3);
    CHECK_NEAR(dual.max.y - dual.min.y, 5, 0.05); // plain shaft (no coupler), 5 mm; the D-flat is on +X
    const AABB none = rotor({1, 30, 0, 20, 5, 1});
    CHECK(none.max.z < 44.0f);
    // Old projects stored only the style: defaults fill in (DC: 14 mm shaft from z = 32).
    CHECK_NEAR(rotor({0}).max.z, 46, 1e-3);
    for (float horn : {0.0f, 1.0f, 2.0f, 3.0f}) CHECK(!buildLibraryPart("motor_rotor", {2, horn}).indices.empty());
    CHECK(withDefaults("motor_rotor", {1}).size() == 6);
    CHECK(partParams("motor_rotor", {2}).size() == 2);
}

TEST_CASE("PART takes shaft options; PARAM edits them after placing") {
    Document d;
    REQUIRE(run(d, "PART STEPPER SHAFTS DUAL SHAFT_LENGTH 40 REAR_LENGTH 25 SHAFT_DIA 8 FITTING PLAIN").ok);
    const Body* r = body(d.scene, "Stepper Motor 1 Rotor");
    REQUIRE(r && r->params.size() == 6);
    CHECK_NEAR(r->params[1], 40, 1e-6);
    CHECK_NEAR(r->params[2], 2, 1e-6);
    CHECK_NEAR(r->params[4], 8, 1e-6);
    CHECK_NEAR(r->bounds().min.z, -25, 1e-3);
    CHECK(!run(d, "PART DCMOTOR SHAFTS TRIPLE").ok);
    CHECK(!run(d, "PART DCMOTOR SHAFT_LENGTH -3").ok);
    REQUIRE(run(d, "PART SERVO HORN CROSS").ok);
    CHECK(run(d, "PART SERVO HORN SINGLE_ARM").ok);
    CHECK_NEAR(body(d.scene, "Servo Motor 1 Rotor")->params[1], 2, 1e-6);

    // Edit through the housing name: the rotor changes.
    REQUIRE(run(d, "PARAM \"Stepper Motor 1\" SHAFTS SINGLE SHAFT_LENGTH 10").ok);
    r = body(d.scene, "Stepper Motor 1 Rotor");
    CHECK_NEAR(r->params[2], 1, 1e-6);
    CHECK_NEAR(r->bounds().min.z, 42, 1e-3);
    CHECK_NEAR(r->bounds().max.z, 52, 1e-3);
    CHECK(run(d, "PARAM \"Stepper Motor 1\"").lines.size() == 6); // header + 5 parameters
    CHECK(!run(d, "PARAM \"Stepper Motor 1\" COLOUR 3").ok);
    CHECK(!run(d, "PARAM \"Stepper Motor 1\" SHAFT_DIA 0").ok);
    // Gears too, and scaled parts keep their scale.
    REQUIRE(run(d, "PART SPUR NAME G TEETH 20").ok);
    REQUIRE(run(d, "SCALE G 2").ok);
    REQUIRE(run(d, "PARAM G TEETH 30").ok);
    const Body* g = body(d.scene, "G");
    CHECK_NEAR(g->bounds().max.x - g->bounds().min.x, 2 * 2 * (30 + 2), 0.3);
    REQUIRE(run(d, "BOX 5 5 5").ok);
    CHECK(!run(d, "PARAM TEETH 3").ok); // plain bodies have no parameters

    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_shaft_test.sicdm").string();
    std::string err;
    REQUIRE(saveProject(d.scene, d.state, path, &err));
    Scene s;
    ProjectState st;
    REQUIRE(loadProject(s, st, path, &err, nullptr));
    const Body* lr = body(s, "Stepper Motor 1 Rotor");
    REQUIRE(lr);
    CHECK_NEAR(lr->bounds().max.z, 52, 1e-3);
}

TEST_CASE("electric car motors: horizontal shaft axis, shaft options, scaling") {
    Document d;
    REQUIRE(run(d, "PART EVMOTOR NAME Foot @0,0,0").ok);
    REQUIRE(run(d, "PART EVFLANGE NAME Flange @0,500,0 SHAFTS DUAL REAR_LENGTH 50").ok);
    const Motion* foot = motion(d.scene, "Foot Drive");
    const Motion* flange = motion(d.scene, "Flange Drive");
    REQUIRE(foot && flange);
    CHECK_NEAR(foot->axis.x, 1, 1e-6);   // turns about +X
    CHECK_NEAR(foot->origin.z, 170, 1e-3); // at the shaft height
    CHECK_NEAR(flange->origin.z, 120, 1e-3);
    CHECK(foot->mode == MotionMode::Loop && foot->range == 360.0f);
    const Body* fh = body(d.scene, "Foot");
    REQUIRE(fh);
    CHECK_NEAR(fh->bounds().min.z, 0, 1e-3); // stands on its feet
    const Body* fr = body(d.scene, "Foot Rotor");
    REQUIRE(fr);
    CHECK_NEAR(fr->bounds().max.x, 132 + 120, 1e-2); // 120 mm shaft from the seal face
    const Body* sr = body(d.scene, "Flange Rotor");
    REQUIRE(sr);
    CHECK_NEAR(sr->bounds().min.x, -197 - 50, 1e-2); // rear shaft
    CHECK_NEAR(sr->bounds().max.x, 140 + 70, 1e-2);
    // Splined vs plain end, editable afterwards.
    const std::size_t splined = sr->mesh.indices.size();
    REQUIRE(run(d, "PARAM Flange FITTING PLAIN SHAFT_LENGTH 90").ok);
    sr = body(d.scene, "Flange Rotor");
    CHECK(sr->mesh.indices.size() < splined);
    CHECK_NEAR(sr->bounds().max.x, 140 + 90, 1e-2);
    // Scaling keeps the drive on the shaft axis.
    REQUIRE(run(d, "SCALE Foot 0.5").ok);
    CHECK_NEAR(motion(d.scene, "Foot Drive")->origin.z, 85, 1e-3);
    CHECK_NEAR(body(d.scene, "Foot Rotor")->bounds().max.x, 0.5 * (132 + 120), 1e-2);
}

TEST_CASE("wheels and tyres stand on the ground and roll along +X") {
    Document d;
    REQUIRE(run(d, "PART CARWHEEL NAME W @0,0,0 DIAMETER 600 WIDTH 200 RIM_DIA 400").ok);
    const Body* tyreBody = body(d.scene, "W Tyre");
    const Body* rim = body(d.scene, "W Rim");
    REQUIRE(tyreBody && rim);
    CHECK_NEAR(tyreBody->bounds().min.z, 0, 0.05);   // on the ground
    CHECK_NEAR(tyreBody->bounds().max.z, 600, 0.05);
    CHECK(rim->bounds().max.z < 600.0f);            // rim inside the tyre
    const Motion* spinM = motion(d.scene, "W Spin");
    REQUIRE(spinM);
    CHECK_NEAR(spinM->axis.y, 1, 1e-6);
    CHECK_NEAR(spinM->origin.z, 300, 1e-3);
    // Positive rotation rolls the top of the wheel toward +X.
    const Vec3 top = motionMatrix(*spinM, 0.1f).transformPoint({0, 0, 600});
    CHECK(top.x > 0.0f);
    for (const char* part : {"ROBOTWHEEL", "CASTER", "OMNI", "MECANUM", "TYRE TREAD OFF", "TYRE TREAD SLICK"}) {
        Document e;
        CHECK(run(e, std::string("PART ") + part + " @0,0,0").ok);
        AABB box;
        for (const Body& b : e.scene.bodies()) {
            CHECK(!b.mesh.indices.empty());
            box.expand(b.bounds());
        }
        if (box.min.z < -0.01f) std::fprintf(stderr, "  %s dips to z = %g\n", part, static_cast<double>(box.min.z));
        CHECK(box.min.z > -0.01f);
    }
    CHECK(run(d, "PART ROBOTWHEEL RPM 0 NAME Free").ok);
    CHECK(motion(d.scene, "Free Spin") == nullptr); // rpm 0: no motion
}

TEST_CASE("sprockets, steel chain and belts") {
    // Sprocket outline between root and tip; teeth stand proud of the pitch circle.
    const MeshData sp = buildLibraryPart("sprocket", {12.7f, 18, 7, 6});
    REQUIRE(!sp.indices.empty());
    CHECK(watertight(buildLibraryPart("sprocket", {12.7f, 18, 7, 0})));
    const float R = 12.7f / (2.0f * std::sin(kPi / 18.0f));
    CHECK(sp.bounds().max.x > R);
    CHECK(sp.bounds().max.x < 12.7f * (0.6f + 1.0f / std::tan(kPi / 18.0f)) * 0.5f + 0.01f);

    const MeshData chain = buildLibraryPart("chain", {12.7f, 10});
    CHECK_NEAR(chain.bounds().max.x - chain.bounds().min.x, 12.7f * 10 + 2 * 12.7f * 0.47f, 0.05);
    CHECK(chain.bounds().min.z > -0.01f);

    Document d;
    REQUIRE(run(d, "PART CHAINDRIVE TEETH1 12 TEETH2 36 RPM 10").ok);
    const Motion* driven = motion(d.scene, "Chain Drive 1 Driven Chain");
    REQUIRE(driven);
    CHECK_NEAR(driven->ratio, 12.0f / 36.0f, 1e-6); // same direction, slower
    CHECK(body(d.scene, "Chain Drive 1 Chain") != nullptr);

    for (const char* t : {"V", "TIMING", "FLAT"}) {
        Document b;
        REQUIRE(run(b, std::string("PART BELTDRIVE TYPE ") + t + " DIA1 40 DIA2 120").ok);
        const Motion* m = motion(b.scene, "Belt Drive 1 Driven Belt");
        REQUIRE(m);
        CHECK(m->ratio > 0.25f && m->ratio < 0.45f); // about 40 / 120
        const Body* belt = body(b.scene, "Belt Drive 1 Belt");
        REQUIRE(belt);
        CHECK(!belt->mesh.indices.empty());
    }
    CHECK(run(d, "PART PULLEY TYPE TIMING").ok);
    CHECK(run(d, "PART BELT TYPE V-BELT DIA1 50 DIA2 80").ok);
    CHECK(run(d, "PART ORING INNER_DIA 30 SECTION 3").ok);
    CHECK(!run(d, "PART PULLEY TYPE ROUND").ok);
    REQUIRE(run(d, "PARAM \"Pulley 1\" TYPE FLAT DIAMETER 80").ok);
    CHECK_NEAR(body(d.scene, "Pulley 1")->bounds().max.x - body(d.scene, "Pulley 1")->bounds().min.x, 80, 0.5);
}

TEST_CASE("chains and belts travel along their loop with the driver") {
    Document d;
    REQUIRE(run(d, "PART CHAINDRIVE NAME CD TEETH1 12 TEETH2 24 RPM 10 @0,0,0").ok);
    const Motion* travel = motion(d.scene, "CD Chain Travel");
    const Motion* drive = motion(d.scene, "CD Driver Drive");
    REQUIRE(travel && drive);
    CHECK(travel->alongLoop);
    std::size_t ci = 0;
    while (d.scene.bodies()[ci].name != "CD Chain") ++ci;
    const float r1 = 12.7f / (2.0f * std::sin(kPi / 12.0f));
    // Travel = driver angle (rad) x pitch radius; the chain is not moved rigidly.
    CHECK_NEAR(loopTravel(d.scene, ci, 0.7f), radians(motionValue(*drive, 0.7f)) * r1, 1e-3);
    const Vec3 p = motionTransforms(d.scene, 0.7f)[ci].transformPoint({10, 20, 0});
    CHECK_NEAR(p.x, 10, 1e-5);
    CHECK_NEAR(p.y, 20, 1e-5);

    // The shape changes with travel and comes back after a whole loop.
    const Body& chain = d.scene.bodies()[ci];
    const MeshData rest = loopMeshAt(chain, 0.0f);
    const MeshData moved = loopMeshAt(chain, 5.0f);
    REQUIRE(rest.vertices.size() == moved.vertices.size());
    CHECK(length(rest.vertices[0].position - moved.vertices[0].position) > 1.0f);
    float loopLen = 0.0f; // the chain's own loop: perimeter of the pitch-circle belt path
    {
        const auto& prm = chain.params;
        const float r2 = prm[2], a = prm[3];
        const float beta = std::asin((r1 - r2) / a);
        loopLen = 2.0f * a * std::cos(beta) + r1 * (kPi + 2.0f * beta) + r2 * (kPi - 2.0f * beta);
    }
    const MeshData around = loopMeshAt(chain, loopLen);
    CHECK(length(rest.vertices[0].position - around.vertices[0].position) < 0.5f);

    Document b;
    REQUIRE(run(b, "PART BELTDRIVE NAME BD TYPE TIMING").ok);
    REQUIRE(motion(b.scene, "BD Belt Travel"));
    CHECK(motion(b.scene, "BD Belt Travel")->alongLoop);
    REQUIRE(run(b, "PART BELT NAME Loose").ok);
    REQUIRE(run(b, "MOTION LINEAR Spin PATH SPEED 50 BODIES Loose").ok);
    std::size_t li = 0;
    while (b.scene.bodies()[li].name != "Loose") ++li;
    CHECK_NEAR(loopTravel(b.scene, li, 2.0f), 100, 1e-3);
    CHECK(!run(b, "MOTION CIRCLE Bad PATH BODIES Loose").ok);

    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_loop_test.sicdm").string();
    std::string err;
    REQUIRE(saveProject(b.scene, b.state, path, &err));
    Scene s;
    ProjectState st;
    REQUIRE(loadProject(s, st, path, &err, nullptr));
    REQUIRE(motion(s, "Spin"));
    CHECK(motion(s, "Spin")->alongLoop);
}

TEST_CASE("robot hand: links, DC motors in the forearm, joints within their limits") {
    for (const char* k : {"phalanx", "robot_palm", "robot_forearm"})
        CHECK(!buildLibraryPart(k, withDefaults(k, {})).indices.empty());

    Document d;
    REQUIRE(run(d, "PART ROBOTHAND").ok);
    // Palm + grip pad, forearm, 15 links + grip pads, 6 hand + 2 wrist motors (housing + rotor each).
    CHECK(d.scene.bodies().size() == 3 + 2 * 15 + 2 * 8);
    CHECK(motion(d.scene, "Robot Hand 1 Index DIP")->moves("Robot Hand 1 Index Distal Pad")); // pads move with their link
    REQUIRE(body(d.scene, "Robot Hand 1 Motor Index Flex") && body(d.scene, "Robot Hand 1 Motor Thumb CMC"));
    CHECK(body(d.scene, "Robot Hand 1 Motor Index Flex")->kind == "dc_motor");
    CHECK(d.scene.motions().front().name == "Robot Hand 1 Motor Wrist Flex Drive"); // wrist outermost

    const Motion* drive = motion(d.scene, "Robot Hand 1 Motor Index Flex Drive");
    REQUIRE(drive);
    CHECK(drive->mode == MotionMode::Oscillate);
    CHECK_NEAR(drive->range, 4 * 90, 1e-4); // gear x MCP limit
    const float closed = drive->range / drive->speed;
    const std::pair<const char*, float> limits[] = {{"Index MCP", 90}, {"Index PIP", 100}, {"Index DIP", 80},
                                                    {"Thumb MCP", 50}, {"Thumb IP", 80},   {"Wrist Flex", 80}};
    for (const auto& [joint, limit] : limits) {
        const Motion* j = motion(d.scene, std::string("Robot Hand 1 ") + joint);
        REQUIRE(j);
        const Motion* lead = motion(d.scene, j->follow);
        REQUIRE(lead);
        CHECK_NEAR(motionValue(d.scene, *j, lead->range / lead->speed), limit, 1e-2);
        for (float t = 0; t < 12; t += 0.07f) {
            const float a = motionValue(d.scene, *j, t);
            CHECK(a >= -1e-3f && a <= limit + 1e-2f);
        }
    }

    // The chain holds together: the PIP point moves the same with the proximal and middle links.
    const auto tf = motionTransforms(d.scene, closed * 0.6f);
    std::size_t prox = 0, mid = 0, tip = 0;
    for (std::size_t i = 0; i < d.scene.bodies().size(); ++i) {
        const std::string& n = d.scene.bodies()[i].name;
        if (n == "Robot Hand 1 Index Proximal") prox = i;
        if (n == "Robot Hand 1 Index Middle") mid = i;
        if (n == "Robot Hand 1 Index Distal") tip = i;
    }
    const Vec3 pipPoint = d.scene.bodies()[mid].offset;
    const Vec3 a = tf[prox].transformPoint(pipPoint), b = tf[mid].transformPoint(pipPoint);
    CHECK_NEAR(length(a - b), 0, 1e-2);
    // Flexion curls the fingertip toward the palm side (-Z).
    const Vec3 tipPoint = d.scene.bodies()[tip].bounds().max;
    CHECK(tf[tip].transformPoint(tipPoint).z < tipPoint.z - 20.0f);

    REQUIRE(run(d, "PART ROBOTHAND DRIVE 1 WRIST 0 HAND 1 @400,0,0").ok);
    int motors = 0;
    for (const Body& m : d.scene.bodies())
        if (m.component == "Robot Hand 2" && m.kind == "dc_motor") ++motors;
    CHECK(motors == 15);
    // Left hand: the thumb is on the +X side of the palm.
    CHECK(body(d.scene, "Robot Hand 2 Thumb Distal")->bounds().center().x > 400 + 40);
    CHECK(body(d.scene, "Robot Hand 1 Thumb Distal")->bounds().center().x < -40);
}

TEST_CASE("robot finger joints can follow a separately placed DC motor") {
    Document d;
    REQUIRE(run(d, "PART ROBOTFINGER FINGER 2 PIP 90").ok);
    REQUIRE(motion(d.scene, "Robot Finger 1 Ring PIP"));
    CHECK(body(d.scene, "Robot Finger 1 Ring Distal"));
    REQUIRE(run(d, "PART DCMOTOR @0,200,0").ok);
    REQUIRE(run(d, "MOTION CIRCLE \"DC Motor 1 Drive\" RANGE 180 OSC").ok);
    REQUIRE(run(d, "MOTION CIRCLE \"Robot Finger 1 Ring PIP\" FOLLOW \"DC Motor 1 Drive\" RATIO 0.5").ok);
    const Motion* pip = motion(d.scene, "Robot Finger 1 Ring PIP");
    const Motion* dc = motion(d.scene, "DC Motor 1 Drive");
    REQUIRE(pip && dc);
    CHECK_NEAR(motionValue(d.scene, *pip, 180.0f / dc->speed), 90, 1e-2);
    REQUIRE(run(d, "PART ROBOTTHUMB DRIVE 1").ok);
    CHECK(motion(d.scene, "Robot Thumb 1 Thumb IP"));
}

TEST_CASE("robot arm: shoulder to hand, every joint on a DC motor within its limit") {
    Document d;
    REQUIRE(run(d, "PART ROBOTARM").ok);
    REQUIRE(body(d.scene, "Robot Arm 1 Upper Arm") && body(d.scene, "Robot Arm 1 Shoulder"));
    CHECK(body(d.scene, "Robot Arm 1 Motor Elbow")->kind == "dc_motor");
    // Shoulder joints are outermost, then elbow, forearm rotation, wrist, fingers.
    std::vector<std::string> joints;
    for (const Motion& m : d.scene.motions())
        if (!m.follow.empty()) joints.push_back(m.name);
    REQUIRE(joints.size() > 7);
    const char* order[] = {"Shoulder Flex", "Shoulder Abduction", "Shoulder Rotation", "Elbow", "Forearm Rotation", "Wrist Flex"};
    for (std::size_t i = 0; i < 6; ++i) CHECK(joints[i] == std::string("Robot Arm 1 ") + order[i]);

    const std::pair<const char*, float> limits[] = {
        {"Shoulder Flex", 90}, {"Shoulder Abduction", 90}, {"Shoulder Rotation", 70}, {"Elbow", 145}, {"Forearm Rotation", 160}};
    for (const auto& [joint, limit] : limits) {
        const Motion* j = motion(d.scene, std::string("Robot Arm 1 ") + joint);
        REQUIRE(j);
        const Motion* lead = motion(d.scene, j->follow);
        REQUIRE(lead);
        CHECK_NEAR(motionValue(d.scene, *j, lead->range / lead->speed), limit, 1e-2);
    }
    // Shoulder 300 mm + forearm 250 mm: the wrist is 550 mm ahead of the shoulder joint.
    const Body* palm = body(d.scene, "Robot Arm 1 Palm");
    const Body* upper = body(d.scene, "Robot Arm 1 Upper Arm");
    REQUIRE(palm && upper);
    CHECK_NEAR(palm->offset.y - upper->offset.y, 550, 1e-3);
    // Raising the shoulder lifts the hand; the pedestal stays put.
    const Motion* raise = motion(d.scene, "Robot Arm 1 Shoulder Flex");
    const float t = motion(d.scene, raise->follow)->range / motion(d.scene, raise->follow)->speed;
    for (Motion& m : d.scene.motions()) m.enabled = m.name == raise->name || m.name == raise->follow;
    const auto tf = motionTransforms(d.scene, t);
    std::size_t pi = 0, si = 0;
    for (std::size_t i = 0; i < d.scene.bodies().size(); ++i) {
        if (d.scene.bodies()[i].name == "Robot Arm 1 Palm") pi = i;
        if (d.scene.bodies()[i].name == "Robot Arm 1 Shoulder") si = i;
    }
    CHECK_NEAR(tf[pi].transformPoint(palm->offset).z - palm->offset.z, 550, 1e-2); // arm straight up
    CHECK_NEAR(length(tf[si].transformPoint(Vec3{}) - Vec3{}), 0, 1e-4);

    REQUIRE(run(d, "PART ROBOTARM HAND LEFT DRIVE 1 @800,0,0").ok);
    CHECK(run(d, "PART UPPERARM @0,900,0").ok);
    CHECK(run(d, "PART SHOULDER @300,900,0").ok);
}

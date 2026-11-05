#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Print.hpp"
#include "core/Sculpt.hpp"

#include <cmath>
#include <cstdio>
#include <string>

using namespace sicdm;

#define REQUIRE(c)                  \
    do {                            \
        const bool required_ = (c); \
        CHECK(required_);           \
        if (!required_) return;     \
    } while (0)

namespace {

constexpr double kPiS = 3.14159265358979323846;

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }

// A closed, outward-facing solid of about this volume (within `share` of it).
bool solidOf(const Body& b, double volume, double share) {
    const PrintReport r = analyzePrint(b);
    // Measured away from the origin: a face turned the wrong way changes it there even where
    // it adds nothing about the origin (a base on z = 0).
    MeshData moved = b.mesh;
    for (Vertex& p : moved.vertices) p.position = p.position + Vec3{7, -11, 13};
    const double v = meshVolume(moved);
    const bool ok = r.openEdges == 0 && r.nonManifoldEdges == 0 && !r.insideOut && std::fabs(v - volume) <= share * volume;
    if (!ok)
        std::fprintf(stderr, "%s: open %zu, non-manifold %zu, inside out %d, volume %.6g (want %.6g)\n", b.name.c_str(), r.openEdges,
                     r.nonManifoldEdges, r.insideOut ? 1 : 0, v, volume);
    return ok;
}

} // namespace

TEST_CASE("Shapes: frustum, tube, ellipsoid, capsule, hemisphere, spring") {
    Document d;
    REQUIRE(run(d, "FRUSTUM 20 10 30").ok);
    CHECK(solidOf(d.scene.bodies().back(), kPiS * 30 / 3 * (400 + 200 + 100), 0.01));
    CHECK(d.scene.bodies().back().exact());
    REQUIRE(run(d, "CONE 20 30 10").ok); // the cone's own top radius: the same frustum
    CHECK(solidOf(d.scene.bodies().back(), kPiS * 30 / 3 * (400 + 200 + 100), 0.01));

    REQUIRE(run(d, "TUBE 20 14 40").ok);
    CHECK(solidOf(d.scene.bodies().back(), kPiS * (400 - 196) * 40, 0.01));
    CHECK(d.scene.bodies().back().exact());
    CHECK(!run(d, "TUBE 10 12 40").ok); // the bore is wider than the tube

    REQUIRE(run(d, "ELLIPSOID 25 15 12").ok);
    const Body& e = d.scene.bodies().back();
    CHECK(solidOf(e, 4.0 / 3 * kPiS * 25 * 15 * 12, 0.01));
    CHECK(std::fabs(e.mesh.bounds().min.z) < 1e-4f);
    CHECK(std::fabs(e.mesh.bounds().max.x - 25) < 1e-3f && std::fabs(e.mesh.bounds().max.y - 15) < 1e-3f &&
          std::fabs(e.mesh.bounds().max.z - 24) < 1e-3f);
    REQUIRE(run(d, "ELLIPSOID 10 10 10").ok); // round: a sphere, exact
    CHECK(d.scene.bodies().back().exact());

    REQUIRE(run(d, "CAPSULE 10 50").ok);
    const Body& c = d.scene.bodies().back();
    CHECK(solidOf(c, kPiS * 100 * 30 + 4.0 / 3 * kPiS * 1000, 0.01));
    CHECK(std::fabs(c.mesh.bounds().max.z - 50) < 1e-3f && std::fabs(c.mesh.bounds().min.z) < 1e-3f);
    CHECK(c.exact());
    CHECK(!run(d, "CAPSULE 10 15").ok); // shorter than its two ends

    REQUIRE(run(d, "HEMISPHERE 20").ok);
    const Body& h = d.scene.bodies().back();
    CHECK(solidOf(h, 2.0 / 3 * kPiS * 8000, 0.01));
    CHECK(std::fabs(h.mesh.bounds().max.z - 20) < 1e-3f && std::fabs(h.mesh.bounds().min.z) < 1e-4f);
    CHECK(h.exact());

    // A spring's volume: the wire's section along the helix (Pappus), less a sliver at the bends.
    REQUIRE(run(d, "SPRING 15 2 8 5").ok);
    const double helix = 5 * std::sqrt(std::pow(2 * kPiS * 15, 2) + 64.0);
    CHECK(solidOf(d.scene.bodies().back(), kPiS * 4 * helix, 0.025));
    CHECK(d.scene.bodies().back().mesh.bounds().min.z > -0.05f);
    CHECK(d.scene.bodies().back().mesh.bounds().max.z < 2 * 2 + 8 * 5 + 0.05f);
    REQUIRE(run(d, "SPRING 15 2 8 5 LEFT").ok);
    CHECK(solidOf(d.scene.bodies().back(), kPiS * 4 * helix, 0.025));
    CHECK(!run(d, "SPRING 15 2 3 5").ok);  // coils run into each other
    CHECK(!run(d, "SPRING 15 16 40 5").ok); // the wire crosses the axis
}

TEST_CASE("Shapes: sizes edited with PARAM") {
    Document d;
    REQUIRE(run(d, "SPRING 15 2 8 5").ok);
    const std::string name = d.scene.bodies().back().name;
    REQUIRE(run(d, "PARAM \"" + name + "\" TURNS 10 HAND Left").ok);
    const Body& s = d.scene.bodies().back();
    CHECK(s.params.size() == 5 && s.params[3] == 10.0f && s.params[4] == 1.0f);
    CHECK(s.mesh.bounds().max.z > 80.0f);
    CHECK(!run(d, "PARAM \"" + name + "\" PITCH 3").ok); // would collide: refused, kept
    CHECK(d.scene.bodies().back().params[2] == 8.0f);

    REQUIRE(run(d, "CONE 20 40").ok);
    REQUIRE(run(d, "PARAM \"" + d.scene.bodies().back().name + "\" TOP_RADIUS 8").ok); // a cone into a frustum
    CHECK(d.scene.bodies().back().params[1] == 8.0f);
    CHECK(d.scene.bodies().back().exact());
    REQUIRE(run(d, "TUBE 20 14 40").ok);
    CHECK(!run(d, "PARAM \"" + d.scene.bodies().back().name + "\" INNER_RADIUS 25").ok);
}

TEST_CASE("Every primitive is a closed solid (sphere poles, torus seams and disk centres join), so fillets and booleans take it") {
    for (const char* line : {"BOX 20 20 20", "CYLINDER 10 30", "SPHERE 10", "CONE 10 20", "CONE 10 20 4", "TORUS 20 5", "TUBE 10 6 20",
                             "ELLIPSOID 10 8 6", "CAPSULE 5 20", "HEMISPHERE 10", "FRUSTUM 10 5 20", "WEDGE 20 10 10", "PYRAMID 20 20 15",
                             "PRISM 6 10 20", "SPRING 10 2 5 4"}) {
        Document d;
        const bool made = executeCommand(d, line, nullptr).ok;
        CHECK(made);
        if (!made || d.scene.bodies().empty()) continue;
        const Body& b = d.scene.bodies().back();
        CHECK(openEdgeCount(b.mesh) == 0);
        CHECK(meshVolume(b.mesh) > 0);
    }
    Document c;
    CHECK(executeCommand(c, "CYLINDER 10 20", nullptr).ok);
    CHECK(executeCommand(c, "FILLET RADIUS 1", nullptr).ok); // refused before: "isn't closed"
    CHECK(openEdgeCount(c.scene.bodies().back().mesh) == 0);
}

TEST_CASE("The EV sedan sample builds to its sizes: closed body, glasshouse, wheels on the ground") {
    Document d;
    const auto r = executeCommand(d, std::string("RUN ") + SICDM_SOURCE_DIR + "/docs/examples/ev-sedan.txt", nullptr);
    CHECK(r.ok);
    auto box = [&](const std::string& name, AABB& b) {
        for (const Body& body : d.scene.bodies())
            if (body.name == name) {
                const MeshData m = worldMesh(body);
                for (const Vertex& v : m.vertices) b.expand(v.position);
                CHECK(openEdgeCount(m) == 0);
                CHECK(meshVolume(m) > 0);
                return true;
            }
        return false;
    };
    AABB body, glass, front, rear;
    AABB rim;
    if (!box("Body", body) || !box("Glasshouse", glass) || !box("TyreFL", front) || !box("TyreRL", rear) || !box("RimFL", rim)) {
        CHECK(false); // a part is missing
        return;
    }
    CHECK_NEAR(body.max.x - body.min.x, 4720.0f, 1.0f);   // length
    CHECK_NEAR(body.max.y - body.min.y, 1850.0f, 5.0f);   // width
    CHECK_NEAR(glass.max.z, 1440.0f, 3.0f);               // height (the roof)
    CHECK_NEAR(body.min.z, 140.0f, 3.0f);                 // ground clearance
    CHECK_NEAR(front.min.z, 0.0f, 0.5f);                  // tyres on the ground
    CHECK_NEAR(front.max.z - front.min.z, 670.0f, 1.0f);  // 235/45 R18
    CHECK_NEAR(std::fabs(0.5f * (rear.min.x + rear.max.x) - 0.5f * (front.min.x + front.max.x)), 2875.0f, 1.0f); // wheelbase
}

TEST_CASE("The SUV sample builds to its sizes: closed body, glasshouse, 22-inch wheels on the ground") {
    Document d;
    const auto r = executeCommand(d, std::string("RUN ") + SICDM_SOURCE_DIR + "/docs/examples/suv.txt", nullptr);
    CHECK(r.ok);
    auto box = [&](const std::string& name, AABB& b) {
        for (const Body& body : d.scene.bodies())
            if (body.name == name) {
                const MeshData m = worldMesh(body);
                for (const Vertex& v : m.vertices) b.expand(v.position);
                CHECK(openEdgeCount(m) == 0);
                CHECK(meshVolume(m) > 0);
                return true;
            }
        return false;
    };
    AABB body, glass, front, rear, rim;
    if (!box("Body", body) || !box("Glasshouse", glass) || !box("TyreFL", front) || !box("TyreRL", rear) || !box("RimFL", rim)) {
        CHECK(false); // a part is missing
        return;
    }
    CHECK_NEAR(body.max.x - body.min.x, 5052.0f, 1.0f);   // length
    CHECK_NEAR(body.max.y - body.min.y, 2047.0f, 5.0f);   // width
    CHECK_NEAR(glass.max.z, 1870.0f, 3.0f);               // height (the roof)
    CHECK_NEAR(body.min.z, 230.0f, 3.0f);                 // ground clearance
    CHECK_NEAR(front.min.z, 0.0f, 0.5f);                  // tyres on the ground
    CHECK_NEAR(front.max.z - front.min.z, 816.0f, 2.0f);  // 285/45 R22
    CHECK_NEAR(std::fabs(0.5f * (rear.min.x + rear.max.x) - 0.5f * (front.min.x + front.max.x)), 2997.0f, 1.0f); // wheelbase
    CHECK(body.max.x > front.max.x); // the nose points +X
}

TEST_CASE("The sports coupe sample builds to its sizes: staggered wheels, the bonnet whole between the fenders") {
    Document d;
    const auto r = executeCommand(d, std::string("RUN ") + SICDM_SOURCE_DIR + "/docs/examples/sportscar.txt", nullptr);
    CHECK(r.ok);
    auto box = [&](const std::string& name, AABB& b) {
        for (const Body& body : d.scene.bodies())
            if (body.name == name) {
                const MeshData m = worldMesh(body);
                for (const Vertex& v : m.vertices) b.expand(v.position);
                CHECK(openEdgeCount(m) == 0);
                CHECK(meshVolume(m) > 0);
                return true;
            }
        return false;
    };
    AABB body, glass, front, rear;
    if (!box("Body", body) || !box("Glasshouse", glass) || !box("TyreFL", front) || !box("TyreRL", rear)) {
        CHECK(false); // a part is missing
        return;
    }
    CHECK_NEAR(body.max.x - body.min.x, 4519.0f, 1.0f);   // length
    CHECK_NEAR(body.max.y - body.min.y, 1852.0f, 5.0f);   // width
    CHECK_NEAR(glass.max.z, 1298.0f, 3.0f);               // height (the roof)
    CHECK_NEAR(body.min.z, 110.0f, 3.0f);                 // ground clearance
    CHECK_NEAR(front.max.z - front.min.z, 696.0f, 1.0f);  // 235/40 R20
    CHECK_NEAR(rear.max.z - rear.min.z, 716.0f, 1.0f);    // 305/30 R21
    CHECK(rear.max.y - rear.min.y > front.max.y - front.min.y + 60.0f); // the rears are wider
    CHECK_NEAR(std::fabs(0.5f * (rear.min.x + rear.max.x) - 0.5f * (front.min.x + front.max.x)), 2450.0f, 1.0f); // wheelbase
    // The bonnet between the fenders is whole: a ray straight down at the car's centre line,
    // over the front axle, meets the body above the wheels' tops (the arches didn't cut it).
    for (const Body& b : d.scene.bodies())
        if (b.name == "Body") {
            const MeshData m = worldMesh(b);
            const float xa = 0.5f * (front.min.x + front.max.x);
            float best = -1e9f;
            for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3) {
                const Vec3 &p0 = m.vertices[m.indices[i]].position, &p1 = m.vertices[m.indices[i + 1]].position, &p2 = m.vertices[m.indices[i + 2]].position;
                // Barycentric test of the point (xa, 0) in the triangle seen from above.
                const float den = (p1.y - p2.y) * (p0.x - p2.x) + (p2.x - p1.x) * (p0.y - p2.y);
                if (std::fabs(den) < 1e-9f) continue;
                const float u = ((p1.y - p2.y) * (xa - p2.x) + (p2.x - p1.x) * (0 - p2.y)) / den;
                const float v = ((p2.y - p0.y) * (xa - p2.x) + (p0.x - p2.x) * (0 - p2.y)) / den;
                if (u < 0 || v < 0 || u + v > 1) continue;
                best = std::max(best, u * p0.z + v * p1.z + (1 - u - v) * p2.z);
            }
            CHECK(best > front.max.z); // the bonnet's top is above the front tyres
        }
}

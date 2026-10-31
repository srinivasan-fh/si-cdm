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

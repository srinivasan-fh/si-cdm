#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Fea.hpp"
#include "core/Print.hpp"
#include "core/Scene.hpp"
#include "core/Sculpt.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>

using namespace sicdm;

#define REQUIRE(c)                                    \
    do {                                              \
        const bool required_ = static_cast<bool>(c); \
        CHECK(required_);                             \
        if (!required_) return;                       \
    } while (0)

namespace {
CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }
} // namespace

TEST_CASE("Topology optimisation: a cantilever") {
    // 80 x 20 x 40, fixed at x = -40, pushed down at the free end's middle.
    Document d;
    REQUIRE(run(d, "BOX 80 20 40 @0,0,0").ok);
    const MeshData world = worldMesh(d.scene.bodies().back());
    TopologySetup ts;
    ts.fea.material = *feaMaterial("aluminium");
    FeaSelection fix;
    fix.kind = FeaSelection::Kind::Face, fix.point = {-40, 0, 20};
    ts.fea.fixed.push_back(fix);
    FeaLoad load;
    load.where.kind = FeaSelection::Kind::Near, load.where.point = {40, 0, 20}, load.where.radius = 3;
    load.force = {0, 0, -1000};
    ts.fea.loads.push_back(load);
    ts.volume = 0.35;
    ts.iterations = 30;
    ts.maxCells = 2500;
    TopologyResult r;
    std::string why;
    const auto t0 = std::chrono::steady_clock::now();
    REQUIRE(runTopology(world, ts, r, &why));
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::fprintf(stderr, "topology: %d cubes, %d steps, compliance %g -> %g, volume %.3f, %ld solver steps, %.1f s\n", r.cells, r.iterations, r.startCompliance, r.compliance, r.volume,
                 r.solverIterations, secs);
    CHECK(std::fabs(r.volume - 0.35) < 0.02);
    CHECK(r.compliance < 0.5 * r.startCompliance); // far stiffer than the even grey start
    for (std::size_t k = 1; k + 1 < r.history.size(); ++k) CHECK(r.history[k] < r.history.front()); // better than where it began
    Body b = meshBody("opt", r.mesh);
    const PrintReport pr = analyzePrint(b);
    CHECK(pr.openEdges == 0 && pr.nonManifoldEdges == 0 && !pr.insideOut);
    const AABB bb = r.mesh.bounds();
    CHECK(bb.min.x < -38 && bb.max.x > 38); // from the wall to the load
    const float h = r.cell; // the cubes may reach a cube past the block
    CHECK(bb.min.x > -40 - h && bb.max.x < 40 + h && bb.min.z > -h && bb.max.z < 40 + h);
    // Its material is under half the block's.
    CHECK(pr.volume < 0.5 * 80 * 20 * 40);
}

TEST_CASE("Topology optimisation: the command") {
    Document d;
    REQUIRE(run(d, "BOX 60 10 30 @0,0,0").ok);
    const std::string name = d.scene.bodies().back().name;
    const CommandOutput o =
        run(d, "TOPOLOGY \"" + name + "\" FIX FACE -30,0,15 LOAD NEAR 30,0,15 3 FORCE 0,0,-500 VOLUME 40 ITERATIONS 15 CELLS 1500 MATERIAL steel");
    if (!o.ok) std::fprintf(stderr, "%s\n", o.lines.back().c_str());
    REQUIRE(o.ok);
    CHECK(d.scene.bodies().back().name == name + " Optimised");
    CHECK(!d.scene.bodies().front().visible);
    CHECK(!run(d, "TOPOLOGY \"" + name + "\" LOAD NEAR 30,0,15 3 FORCE 0,0,-500").ok); // not fixed
    CHECK(!run(d, "TOPOLOGY \"" + name + "\" FIX FACE -30,0,15").ok);                   // not loaded
    CHECK(!run(d, "TOPOLOGY \"" + name + "\" FIX FACE -30,0,15 LOAD NEAR 30,0,15 3 FORCE 0,0,-1 VOLUME 0.99").ok);
}

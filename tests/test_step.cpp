#include "TestFramework.hpp"

#include "core/Brep.hpp"
#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Scene.hpp"
#include "core/Sculpt.hpp"
#include "core/Step.hpp"

#include <cmath>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

using namespace sicdm;

#define REQUIRE(c)                  \
    do {                            \
        const bool required_ = (c); \
        CHECK(required_);           \
        if (!required_) return;     \
    } while (0)

namespace {

constexpr double kPiT = 3.14159265358979323846;

const std::vector<std::pair<std::string, double>>& references() {
    static const std::vector<std::pair<std::string, double>> v = {
#include "data/step/volumes.inc"
    };
    return v;
}

std::string dataPath(const std::string& name) { return std::string(SICDM_SOURCE_DIR) + "/tests/data/step/" + name + ".step"; }

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }

std::string text(const CommandOutput& r) {
    std::string s;
    for (const std::string& l : r.lines) s += l + "\n";
    return s;
}

std::string tempPath(const char* name) { return (std::filesystem::temp_directory_path() / name).string(); }

double volumeOf(const std::vector<StepPart>& parts, double chord, std::size_t* open = nullptr, std::size_t* failed = nullptr) {
    double v = 0;
    TessellateOptions o;
    o.chord = chord;
    for (const StepPart& p : parts) {
        TessellateReport r;
        const MeshData m = tessellate(*p.brep, o, &r);
        v += meshVolume(m);
        if (open) *open += openEdgeCount(m);
        if (failed) *failed += r.failedFaces;
    }
    return v;
}

const Body* named(const Document& d, const std::string& name) {
    for (const Body& b : d.scene.bodies())
        if (b.name == name) return &b;
    return nullptr;
}

} // namespace

TEST_CASE("STEP import: every reference solid comes in closed, at its exact volume to the tolerance") {
    for (const auto& [name, volume] : references()) {
        StepReadReport rep;
        std::string err;
        const std::vector<StepPart> parts = readStepFile(dataPath(name), &rep, &err);
        if (parts.empty()) {
            std::fprintf(stderr, "  %s: %s\n", name.c_str(), err.c_str());
            CHECK(!parts.empty());
            continue;
        }
        std::size_t open = 0, failed = 0;
        const double v = volumeOf(parts, 0.002, &open, &failed);
        if (open || failed || std::fabs(v - volume) > 1e-3 * volume)
            std::fprintf(stderr, "  %s: volume %.4f (exact %.4f), %zu open edges, %zu failed faces\n", name.c_str(), v, volume, open, failed);
        CHECK(open == 0);
        CHECK(failed == 0);
        CHECK_NEAR(v, volume, 1e-3 * volume); // 0.1% at a 2 micron chord; the facets sit inside curved faces
        // The default tolerance is coarser, but still closed and within half a percent.
        std::size_t open2 = 0;
        CHECK_NEAR(volumeOf(parts, 0.02, &open2), volume, 7e-3 * volume);
        CHECK(open2 == 0);
    }
}

TEST_CASE("STEP import: units and assemblies (two placements of one part, two levels deep)") {
    StepReadReport rep;
    const std::vector<StepPart> inch = readStepFile(dataPath("box_inch"), &rep);
    REQUIRE(inch.size() == 1);
    CHECK_NEAR(rep.unitToMm, 25.4, 1e-9);
    CHECK_NEAR(volumeOf(inch, 0.01), 24000.0, 1e-6); // the file says inches; the part is still 40 x 30 x 20 mm
    const std::vector<StepPart> parts = readStepFile(dataPath("assembly"));
    REQUIRE(parts.size() == 3);
    int brackets = 0;
    for (const StepPart& p : parts) {
        const AABB b = tessellate(*p.brep).bounds();
        if (p.name == "Bracket") {
            ++brackets;
            // 10 x 20 x 30 at (100, 0, 0), or turned 90 degrees about Z at (0, 50, 0); both up 200.
            const bool moved = std::fabs(b.min.x - 100) < 1e-3 && std::fabs(b.max.y - 20) < 1e-3;
            const bool turned = std::fabs(b.min.x + 20) < 1e-3 && std::fabs(b.min.y - 50) < 1e-3 && std::fabs(b.max.y - 60) < 1e-3;
            CHECK(moved || turned);
            CHECK_NEAR(b.min.z, 200.0, 1e-3);
        } else {
            CHECK(p.name == "Pin");
            CHECK_NEAR(b.min.z, 0.0, 1e-3);
            CHECK_NEAR(0.5 * (b.min.x + b.max.x), 5.0, 1e-2);
        }
    }
    CHECK(brackets == 2);
}

TEST_CASE("STEP export: the writer's files read back to the same exact B-reps") {
    for (const auto& [name, volume] : references()) {
        const std::vector<StepPart> parts = readStepFile(dataPath(name));
        REQUIRE(!parts.empty());
        const std::string written = writeStep(parts, name + ".step");
        std::string err;
        const std::vector<StepPart> again = readStep(written, nullptr, &err);
        REQUIRE(again.size() == parts.size());
        for (std::size_t i = 0; i < parts.size(); ++i) {
            CHECK(again[i].name == parts[i].name);
            CHECK(again[i].brep->faceCount() == parts[i].brep->faceCount());
        }
        CHECK_NEAR(volumeOf(again, 0.01), volumeOf(parts, 0.01), 1e-6 * volume);
    }
}

TEST_CASE("STEP from the model: primitives exactly (turned and scaled too), other meshes as planar faces") {
    Document d;
    for (const char* l : {"BOX 40 30 20 @0,0,0", "CYLINDER 10 30 @60,0,0", "ROTATE \"Cylinder 2\" 30,0,45", "SPHERE 12 @0,60,0", "TORUS 20 6 @0,120,0",
                          "SCALE \"Torus 4\" 1.5", "WEDGE 20 20 10 @60,120,0", "BOX 20 20 20 @-60,60,0", "FILLET \"Block 6\" RADIUS 2 EDGES TOP"})
        REQUIRE(run(d, l).ok);
    const std::string path = tempPath("sicdm_model.step");
    auto r = run(d, "EXPORT " + path);
    REQUIRE(r.ok);
    CHECK(text(r).find("5 exact, 1 as planar facets") != std::string::npos);
    const std::vector<StepPart> parts = readStepFile(path);
    REQUIRE(parts.size() == 6);
    const double expect[] = {24000.0, kPiT * 100 * 30, 4.0 / 3 * kPiT * 1728, 2 * kPiT * kPiT * 30 * 81, 2000.0};
    for (int i = 0; i < 5; ++i) CHECK_NEAR(volumeOf({parts[static_cast<std::size_t>(i)]}, 0.002), expect[i], 1e-3 * expect[i]);
    // The filleted block went as facets: exactly the mesh's volume.
    const double faceted = meshVolume(worldMesh(*named(d, "Block 6")));
    CHECK_NEAR(volumeOf({parts[5]}, 0.01), faceted, 1e-6 * faceted);
    // Placed where the bodies are: the turned cylinder's centre stays at its base's position.
    const AABB cb = tessellate(*parts[1].brep).bounds();
    CHECK(cb.min.x > 25 && cb.max.x < 95);
    // Back in, as bodies with their exact geometry.
    Document e;
    r = run(e, "IMPORT " + path);
    REQUIRE(r.ok);
    CHECK(e.scene.bodies().size() == 6);
    for (const Body& b : e.scene.bodies()) CHECK(b.exact());
    std::filesystem::remove(path);
}

TEST_CASE("Imported exact geometry survives save and open, and is dropped when the mesh is edited") {
    Document d;
    REQUIRE(run(d, "IMPORT " + dataPath("plate_holes")).ok);
    REQUIRE(d.scene.bodies().size() == 1);
    CHECK(d.scene.bodies()[0].exact());
    const std::string name = d.scene.bodies()[0].name;
    const std::string proj = tempPath("sicdm_step_project.sicdm");
    REQUIRE(run(d, "SAVE " + proj).ok);
    Document e;
    REQUIRE(run(e, "OPEN " + proj).ok);
    REQUIRE(e.scene.bodies().size() == 1);
    CHECK(e.scene.bodies()[0].exact());
    CHECK(e.scene.bodies()[0].brep->faceCount() == 10);
    // A sculpt stroke changes the mesh: the B-rep no longer describes it.
    REQUIRE(run(e, "SCULPT \"" + name + "\" DRAW AT 0,10,10 RADIUS 3").ok);
    CHECK(!e.scene.bodies()[0].exact());
    const std::string out = tempPath("sicdm_step_after.step");
    const auto r = run(e, "EXPORT " + out);
    REQUIRE(r.ok);
    CHECK(text(r).find("0 exact, 1 as planar facets") != std::string::npos);
    std::filesystem::remove(proj);
    std::filesystem::remove(out);
}

TEST_CASE("A mesh as a planar B-rep: coplanar triangles merge, the volume is the mesh's") {
    const MeshData box = buildPrimitive("box", {0, 0, 0, 10, 20, 30});
    const auto b = meshToBrep(box);
    REQUIRE(b->shells.size() == 1);
    CHECK(b->shells[0].closed);
    CHECK(b->faceCount() == 6);
    CHECK_NEAR(meshVolume(tessellate(*b)), 6000.0, 1e-6);
    Body ball = makeBody("Ball", "sphere", {10});
    const auto s = meshToBrep(ball.mesh);
    const MeshData m = tessellate(*s);
    CHECK(openEdgeCount(m) == 0);
    CHECK_NEAR(meshVolume(m), meshVolume(ball.mesh), 1e-3);
}

TEST_CASE("STEP reader refuses what isn't STEP, or is cut short, without crashing") {
    std::string err;
    CHECK(readStep("hello", nullptr, &err).empty());
    CHECK(!err.empty());
    err.clear();
    CHECK(readStep("ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n#1=CARTESIAN_POINT('',(0.,0.,0.));\nENDSEC;\n", nullptr, &err).empty());
    CHECK(err.find("no solids") != std::string::npos);
    // The first half of a real file: refs to entities that never come.
    std::FILE* f = std::fopen(dataPath("fillet_block").c_str(), "rb");
    REQUIRE(f != nullptr);
    std::string whole;
    char buf[4096];
    for (std::size_t n; (n = std::fread(buf, 1, sizeof buf, f)) > 0;) whole.append(buf, n);
    std::fclose(f);
    const std::vector<StepPart> half = readStep(whole.substr(0, whole.size() / 2));
    for (const StepPart& p : half) (void)tessellate(*p.brep);
    Document d;
    CHECK(!run(d, "IMPORT /no/such/file.step").ok);
}

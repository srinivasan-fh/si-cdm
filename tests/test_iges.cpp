#include "TestFramework.hpp"

#include "core/Brep.hpp"
#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Iges.hpp"
#include "core/Scene.hpp"
#include "core/Sculpt.hpp"
#include "core/Step.hpp"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <set>
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

constexpr double kPiI = 3.14159265358979323846;

const std::vector<std::pair<std::string, double>>& references() {
    static const std::vector<std::pair<std::string, double>> v = {
#include "data/step/volumes.inc"
    };
    return v;
}

std::string igesPath(const std::string& name) { return std::string(SICDM_SOURCE_DIR) + "/tests/data/iges/" + name + ".igs"; }
std::string stepPath(const std::string& name) { return std::string(SICDM_SOURCE_DIR) + "/tests/data/step/" + name + ".step"; }
std::string tempPath(const char* name) { return (std::filesystem::temp_directory_path() / name).string(); }
CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }

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

} // namespace

TEST_CASE("IGES import: trimmed surfaces sewn into closed solids, and B-rep solids, at their exact volumes") {
    for (const auto& [name, volume] : references())
        for (const char* form : {"_faces", "_brep"}) {
            StepReadReport rep;
            std::string err;
            const std::vector<StepPart> parts = readIgesFile(igesPath(name + form), &rep, &err);
            if (parts.size() != 1) std::fprintf(stderr, "  %s%s: %zu parts %s\n", name.c_str(), form, parts.size(), err.c_str());
            REQUIRE(parts.size() == 1);
            CHECK(rep.schema == "IGES");
            std::size_t open = 0, failed = 0;
            const double v = volumeOf(parts, 0.002, &open, &failed);
            if (open || failed || std::fabs(v - volume) > 1e-3 * volume)
                std::fprintf(stderr, "  %s%s: volume %.4f (exact %.4f), %zu open edges, %zu failed faces\n", name.c_str(), form, v, volume, open, failed);
            CHECK(open == 0);
            CHECK(failed == 0);
            CHECK_NEAR(v, volume, 1e-3 * volume);
            CHECK(parts[0].brep->shells[0].closed);
        }
}

TEST_CASE("IGES import: units and several solids in one file, where the file puts them") {
    StepReadReport rep;
    for (const char* form : {"box_inch_faces", "box_inch_brep"}) {
        const std::vector<StepPart> inch = readIgesFile(igesPath(form), &rep);
        REQUIRE(inch.size() == 1);
        CHECK_NEAR(rep.unitToMm, 25.4, 1e-9);
        CHECK_NEAR(volumeOf(inch, 0.01), 24000.0, 1e-6); // written in inches: still 40 x 30 x 20 mm
    }
    for (const char* form : {"assembly_faces", "assembly_brep"}) {
        const std::vector<StepPart> parts = readIgesFile(igesPath(form));
        REQUIRE(parts.size() == 3);
        int brackets = 0, pins = 0;
        for (const StepPart& p : parts) {
            const AABB b = tessellate(*p.brep).bounds();
            const double v = volumeOf({p}, 0.01);
            if (std::fabs(v - 6000.0) < 1.0) {
                ++brackets;
                CHECK_NEAR(b.min.z, 200.0, 1e-3);
            } else {
                ++pins;
                CHECK_NEAR(b.min.z, 0.0, 1e-3);
                CHECK_NEAR(0.5 * (b.min.x + b.max.x), 5.0, 1e-2);
            }
        }
        CHECK(brackets == 2);
        CHECK(pins == 1);
    }
}

TEST_CASE("IGES export: every reference written as trimmed surfaces reads back at its volume") {
    for (const auto& [name, volume] : references()) {
        const std::vector<StepPart> parts = readStepFile(stepPath(name));
        REQUIRE(!parts.empty());
        const std::string text = writeIges(parts, name + ".igs");
        // Fixed 80-column records, sections in order, ending in T.
        CHECK(text.size() % 81 == 0);
        CHECK(text.compare(text.size() - 9, 1, "T") == 0);
        std::string err;
        const std::vector<StepPart> again = readIges(text, nullptr, &err);
        REQUIRE(again.size() == parts.size());
        std::size_t open = 0;
        CHECK_NEAR(volumeOf(again, 0.002, &open), volume, 1e-3 * volume);
        CHECK(open == 0);
    }
}

TEST_CASE("IGES from the model: primitives, revolves and extrusions exact, other meshes as facets; and back in") {
    Document d;
    for (const char* l : {"BOX 40 30 20 @0,0,0", "CYLINDER 10 30 @60,0,0", "ROTATE \"Cylinder 2\" 30,0,45", "SPHERE 12 @0,60,0", "TORUS 20 6 @0,120,0",
                          "BOX 20 20 20 @-60,60,0", "FILLET \"Block 5\" RADIUS 2 EDGES TOP", "PLANE FRONT", "CIRCLE 160,40 3", "LINE 150,0 150,10",
                          "REVOLVE \"Circle 1\" AXIS \"Line 2\" ANGLE 200"})
        REQUIRE(run(d, l).ok);
    const std::string path = tempPath("sicdm_model.igs");
    auto r = run(d, "EXPORT " + path);
    REQUIRE(r.ok);
    CHECK(r.lines.back().find("6 exact, 0 as planar facets") != std::string::npos); // the rounded block too (see ExactMesh)
    Document e;
    r = run(e, "IMPORT " + path);
    REQUIRE(r.ok);
    REQUIRE(e.scene.bodies().size() == 6);
    // Sewn back into six closed solids; their volumes add up to the model's.
    double want = 24000.0 + kPiI * 100 * 30 + 4.0 / 3 * kPiI * 1728 + 2 * kPiI * kPiI * 20 * 36 + 2 * kPiI * kPiI * 10 * 9 * 200 / 360;
    for (const Body& b : d.scene.bodies())
        if (b.name == "Block 5") want += meshVolume(worldMesh(b));
    double got = 0;
    for (const Body& b : e.scene.bodies()) {
        CHECK(b.exact());
        CHECK(openEdgeCount(b.mesh) == 0);
        got += meshVolume(tessellate(*b.brep, TessellateOptions{0.002, 0.26, 0}));
    }
    CHECK_NEAR(got, want, 1e-3 * want);
    // Each part comes back as itself, by name (the export groups and names them).
    std::set<std::string> before, after;
    for (const Body& b : d.scene.bodies()) before.insert(b.name);
    for (const Body& b : e.scene.bodies()) after.insert(b.name);
    CHECK(before == after);
    std::filesystem::remove(path);
}

TEST_CASE("IGES reader refuses what isn't IGES, or is cut short, without crashing") {
    std::string err;
    CHECK(readIges("hello", nullptr, &err).empty());
    CHECK(!err.empty());
    err.clear();
    std::string compressed(72, ' ');
    compressed += "C0000001\n";
    CHECK(readIges(compressed, nullptr, &err).empty());
    CHECK(err.find("Compressed") != std::string::npos);
    // The first half of a real file: directory entries pointing past the end.
    const std::vector<StepPart> parts = readStepFile(stepPath("fillet_block"));
    const std::string whole = writeIges(parts);
    for (std::size_t cut : {whole.size() / 3, whole.size() / 2, whole.size() * 9 / 10}) {
        const std::vector<StepPart> some = readIges(whole.substr(0, cut / 81 * 81));
        for (const StepPart& p : some) (void)tessellate(*p.brep);
    }
    Document d;
    CHECK(!run(d, "IMPORT /no/such/file.igs").ok);
}

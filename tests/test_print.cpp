#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Print.hpp"
#include "core/Scene.hpp"
#include "core/Sculpt.hpp"
#include "core/Wearable.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

using namespace sicdm;

#define REQUIRE(c)               \
    do {                         \
        const bool required_ = (c); \
        CHECK(required_);        \
        if (!required_) return;  \
    } while (0)

namespace {

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }

std::string slurp(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    std::ostringstream s;
    s << f.rdbuf();
    return s.str();
}

std::string tempPath(const char* name) { return (std::filesystem::temp_directory_path() / name).string(); }

} // namespace

TEST_CASE("printability: a clean block is ready, its size, volume and material are right") {
    const Body box = makeBody("Block", "box", {0, 0, 0, 20, 20, 20});
    const PrintReport r = analyzePrint(box);
    CHECK(r.printable());
    CHECK(r.openEdges == 0);
    CHECK(r.nonManifoldEdges == 0);
    CHECK(r.shells == 1);
    CHECK(!r.insideOut);
    CHECK_NEAR(r.volume, 8000.0, 1e-3);
    CHECK_NEAR(r.area, 2400.0, 1e-3);
    CHECK(r.overhangArea < 1.0); // its bottom is on the bed
    CHECK(r.thinnest > 19.0f);
    CHECK(r.grams > 1.0);
    CHECK(r.grams < 8000.0 / 1000.0 * 1.24);
    PrintSettings tiny;
    tiny.buildVolume = {10, 10, 10};
    CHECK(!analyzePrint(box, tiny).fits);
}

TEST_CASE("printability finds holes, inside-out faces, overhangs and thin walls") {
    Body open = makeBody("Block", "box", {0, 0, 0, 20, 20, 20});
    open.kind = "mesh";
    open.mesh.indices.resize(open.mesh.indices.size() - 6); // two faces gone: a square hole
    PrintReport r = analyzePrint(open);
    CHECK(r.openEdges == 4);
    CHECK(!r.printable());

    Body flipped = makeBody("Block", "box", {0, 0, 0, 20, 20, 20});
    for (std::size_t k = 0; k + 2 < flipped.mesh.indices.size(); k += 3) std::swap(flipped.mesh.indices[k + 1], flipped.mesh.indices[k + 2]);
    r = analyzePrint(flipped);
    CHECK(r.insideOut);
    CHECK(!r.printable());

    // A T standing on its stem: the arms' undersides need support.
    Document d;
    REQUIRE(run(d, "BOX 10 10 30 @0,0,0").ok);
    REQUIRE(run(d, "BOX 50 10 6 @0,0,28").ok); // into the stem a little (faces that only touch are a boolean's hard case)
    REQUIRE(run(d, "BOOLEAN \"Block 1\" UNION \"Block 2\"").ok);
    r = analyzePrint(d.scene.bodies()[0]);
    CHECK_NEAR(r.overhangArea, 50.0 * 10.0 - 10.0 * 10.0, 1.0);

    // A 0.4 mm wall is thinner than the nozzle prints.
    Body shell = makeBody("Cup", "box", {0, 0, 0, 30, 30, 30});
    ShellOptions so;
    so.thickness = 0.4f;
    REQUIRE(shellBody(shell, so));
    r = analyzePrint(shell);
    CHECK(r.thinSamples > 0);
    CHECK(r.thinnest < 0.6f);
}

TEST_CASE("REPAIR closes holes, turns faces the right way and drops duplicates and specks") {
    Body b = makeBody("Block", "box", {0, 0, 0, 20, 20, 20});
    b.kind = "mesh";
    auto& idx = b.mesh.indices;
    idx.resize(idx.size() - 6);                       // a hole
    std::swap(idx[1], idx[2]);                         // one face the wrong way round
    idx.insert(idx.end(), {idx[3], idx[4], idx[5]});   // a duplicate face
    // A speck: a tiny separate triangle pair far away.
    const auto base = static_cast<std::uint32_t>(b.mesh.vertices.size());
    for (const Vec3& q : {Vec3{100, 0, 0}, Vec3{100.01f, 0, 0}, Vec3{100, 0.01f, 0}}) b.mesh.vertices.push_back({q, {0, 0, 1}});
    idx.insert(idx.end(), {base, base + 1, base + 2, base, base + 2, base + 1});
    RepairReport rep;
    REQUIRE(repairMesh(b, &rep));
    CHECK(rep.holesFilled == 1);
    CHECK(rep.flipped == 1);
    CHECK(rep.duplicates >= 1); // the copied face (and the speck's back-to-back pair)
    CHECK(rep.specks == 1);
    const PrintReport r = analyzePrint(b);
    CHECK(r.printable());
    CHECK(r.shells == 1);
    CHECK_NEAR(r.volume, 8000.0, 1.0);

    // Inside out as a whole: turned out.
    Body f = makeBody("Ball", "sphere", {10});
    for (std::size_t k = 0; k + 2 < f.mesh.indices.size(); k += 3) std::swap(f.mesh.indices[k + 1], f.mesh.indices[k + 2]);
    REQUIRE(repairMesh(f));
    CHECK(!analyzePrint(f).insideOut);
    CHECK(meshVolume(worldMesh(f)) > 0);
}

TEST_CASE("ORIENT turns a part for the least support and sets it on the bed") {
    Document d;
    REQUIRE(run(d, "BOX 10 10 30 @0,0,0").ok);
    REQUIRE(run(d, "BOX 50 10 6 @0,0,28").ok);
    REQUIRE(run(d, "BOOLEAN \"Block 1\" UNION \"Block 2\"").ok); // a T: the arms overhang
    REQUIRE(run(d, "MOVE 0,0,25 \"Block 1\"").ok);
    const auto r = run(d, "ORIENT \"Block 1\"");
    REQUIRE(r.ok);
    const Body& t = d.scene.bodies()[0];
    CHECK(analyzePrint(t).overhangArea < 1.0); // on its side or upside down: nothing hangs out
    CHECK_NEAR(t.bounds().min.z, 0.0, 1e-3);
    CHECK_NEAR(meshVolume(worldMesh(t)), 10.0 * 10 * 28 + 50.0 * 10 * 6, 1.0);
    // The turn leaves float noise in the height; it must not add a sliver layer.
    const double h = t.bounds().max.z - t.bounds().min.z;
    const std::string path = tempPath("sicdm_orient_test.gcode");
    SliceReport sr;
    REQUIRE(sliceToGcode(t, PrintSettings{}, path, &sr));
    CHECK(sr.layers == static_cast<int>(std::lround(h / 0.2)));
    std::filesystem::remove(path);
}

TEST_CASE("SLICE writes G-code: heats, homes, layer by layer, inside the part's outline") {
    Document d;
    REQUIRE(run(d, "BOX 20 20 10 @0,0,0").ok);
    const std::string path = tempPath("sicdm_slice_test.gcode");
    const auto r = run(d, "SLICE \"Block 1\" TO " + path + " LAYER 0.2 INFILL 20 WALLS 2");
    REQUIRE(r.ok);
    CHECK(r.lines[0].find("50 layers") != std::string::npos);
    const std::string g = slurp(path);
    CHECK(g.find("M109 S210") != std::string::npos);
    CHECK(g.find("G28") != std::string::npos);
    CHECK(g.find(";LAYER:0\n") != std::string::npos);
    CHECK(g.find(";LAYER:49\n") != std::string::npos);
    CHECK(g.find(";LAYER:50\n") == std::string::npos);
    CHECK(g.find(";TYPE:WALL-OUTER") != std::string::npos);
    CHECK(g.find(";TYPE:SOLID-FILL") != std::string::npos);
    CHECK(g.find(";TYPE:FILL\n") != std::string::npos);
    // Every extruding move stays inside the 20 x 20 outline, centred on the bed.
    std::istringstream lines(g);
    std::string l;
    double lastE = 0, lo = 1e9, hi = -1e9;
    bool rising = true;
    while (std::getline(lines, l)) {
        if (l.rfind("G1 X", 0) != 0) continue;
        double x = 0, y = 0, e = 0;
        if (std::sscanf(l.c_str(), "G1 X%lf Y%lf E%lf", &x, &y, &e) != 3) continue;
        lo = std::min({lo, x, y});
        hi = std::max({hi, x, y});
        rising = rising && e >= lastE;
        lastE = e;
    }
    CHECK(rising);
    CHECK(lo >= 128.0 - 10.0);
    CHECK(hi <= 128.0 + 10.0);
    CHECK(lo < 128.0 - 9.0); // the outer wall is half a line inside the edge
    std::filesystem::remove(path);
    CHECK(!run(d, "SLICE \"Block 1\"").ok); // no TO
}

TEST_CASE("EXPORT .3mf writes a 3MF package the slicers read") {
    Document d;
    REQUIRE(run(d, "BOX 10 10 10 @0,0,0").ok);
    REQUIRE(run(d, "SPHERE 5 @30,0,0").ok);
    const std::string path = tempPath("sicdm_test.3mf");
    REQUIRE(run(d, "EXPORT " + path).ok);
    const std::string z = slurp(path);
    REQUIRE(z.size() > 200);
    CHECK(z.compare(0, 4, "PK\x03\x04") == 0);
    CHECK(z.find("[Content_Types].xml") != std::string::npos);
    CHECK(z.find("_rels/.rels") != std::string::npos);
    CHECK(z.find("3D/3dmodel.model") != std::string::npos);
    CHECK(z.find("unit=\"millimeter\"") != std::string::npos);
    CHECK(z.find("name=\"Block 1\"") != std::string::npos);
    CHECK(z.find("name=\"Sphere 2\"") != std::string::npos);
    CHECK(z.find("<item objectid=\"2\"/>") != std::string::npos);
    CHECK(z.find("PK\x05\x06") != std::string::npos); // the central directory's end
    std::filesystem::remove(path);
}

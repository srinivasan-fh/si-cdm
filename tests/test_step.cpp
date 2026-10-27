#include "TestFramework.hpp"

#include "core/Brep.hpp"
#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/ExactMesh.hpp"
#include "core/Scene.hpp"
#include "core/Sculpt.hpp"
#include "core/Iges.hpp"
#include "core/Step.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <map>
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
    CHECK(text(r).find("6 exact, 0 as planar facets") != std::string::npos); // the rounded block too (see ExactMesh)
    const std::vector<StepPart> parts = readStepFile(path);
    REQUIRE(parts.size() == 6);
    const double expect[] = {24000.0, kPiT * 100 * 30, 4.0 / 3 * kPiT * 1728, 2 * kPiT * kPiT * 30 * 81, 2000.0};
    for (int i = 0; i < 5; ++i) CHECK_NEAR(volumeOf({parts[static_cast<std::size_t>(i)]}, 0.002), expect[i], 1e-3 * expect[i]);
    // The rounded block went exact: planes and the rounds' cylinders. Four R2 rounds along
    // 20 mm, less what two of them both take at each corner (0.76691 mm3, numerically).
    const double rounded = 8000 - 4 * (1 - kPiT / 4) * 4 * 20 + 4 * 0.76691;
    CHECK_NEAR(volumeOf({parts[5]}, 0.0005), rounded, 0.5);
    int rounds = 0;
    for (const BSurface& su : parts[5].brep->surfaces) rounds += su.kind == BSurfaceKind::Cylinder;
    CHECK(rounds == 4);
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

TEST_CASE("REVOLVE and EXTRUDE bodies carry exact B-reps: planes, cylinders, cones, tori, partial turns") {
    Document d;
    for (const char* l : {"PLANE FRONT", "RECT 10,0 @10,30", "REVOLVE", "RECT 10,40 @10,10", "REVOLVE ANGLE 90",
                          "PLINE 0,60 15,60 25,75 25,90 0,100 CLOSE", "REVOLVE", "PLINE 0,110 15,110 25,125 0,140 CLOSE", "REVOLVE ANGLE -120",
                          "LINE 50,0 50,10", "CIRCLE 60,20 3", "REVOLVE AXIS \"Line 5\"", "CIRCLE 60,40 3", "REVOLVE AXIS \"Line 5\" ANGLE 200",
                          "PLINE 5,150 30,150 20,170 10,165 CLOSE", "REVOLVE ANGLE 270", "CIRCLE 0,-30 6", "EXTRUDE 12",
                          "PLINE 40,-40 60,-40 50,-25 CLOSE", "EXTRUDE 7", "PLANE RIGHT", "CIRCLE 5,5 4", "EXTRUDE -9"})
        REQUIRE(run(d, l).ok);
    REQUIRE(d.scene.bodies().size() == 10);
    // The exact volumes (Pappus for the turned polygons), as OpenCascade measures the
    // exported solids too.
    const double expect[] = {kPiT * 300 * 30, kPiT * 300 * 10 / 4, 55239.670825620524, 9686.577348568528, 2 * kPiT * kPiT * 10 * 9,
                             2 * kPiT * kPiT * 10 * 9 * 200 / 360, 25034.566458293662, kPiT * 36 * 12, 1050.0, kPiT * 16 * 9};
    for (std::size_t i = 0; i < 10; ++i) {
        const Body& b = d.scene.bodies()[i];
        REQUIRE(b.exact());
        const MeshData m = tessellate(*b.brep, TessellateOptions{0.002, 0.26, 0});
        CHECK(openEdgeCount(m) == 0);
        CHECK_NEAR(meshVolume(m), expect[i], 1e-3 * expect[i]);
        // Where the mesh is.
        const AABB eb = m.bounds(), mb = worldMesh(b).bounds();
        CHECK(length(eb.min - mb.min) < 0.05f && length(eb.max - mb.max) < 0.05f);
    }
    const std::string path = tempPath("sicdm_revolve.step");
    const auto r = run(d, "EXPORT " + path);
    REQUIRE(r.ok);
    CHECK(text(r).find("10 exact, 0 as planar facets") != std::string::npos);
    const std::vector<StepPart> parts = readStepFile(path);
    REQUIRE(parts.size() == 10);
    for (std::size_t i = 0; i < 10; ++i) CHECK_NEAR(volumeOf({parts[i]}, 0.002), expect[i], 1e-3 * expect[i]);
    std::filesystem::remove(path);
}

TEST_CASE("Exact geometry keeps typed dimensions and turns in double precision") {
    CHECK(decimal(0.1f) == 0.1);
    CHECK(decimal(12.7f) == 12.7);
    CHECK(decimal(-3.0e-5f) == -3.0e-5);
    CHECK(static_cast<float>(decimal(1.0f / 3.0f)) == 1.0f / 3.0f); // always reads back as the float
    // Quarter turns are exact; a turn and its inverse come back to the bit.
    const Placement q = eulerPlacement({90, 0, 0});
    CHECK(q.r[0] == 1 && q.r[4] == 0 && q.r[5] == -1 && q.r[7] == 1 && q.r[8] == 0);
    const Placement t = eulerPlacement({30, 20, 10});
    const DVec3 p{12.5, -7.25, 3.0};
    const DVec3 back = t.inverse().point(t.point(p));
    CHECK(dlength(back - p) < 1e-13);
    Document d;
    REQUIRE(run(d, "BOX 0.1 0.2 0.3 @0,0,0").ok);
    REQUIRE(run(d, "PLANE FRONT").ok);
    REQUIRE(run(d, "RECT 0.1,0 @0.7,1.3").ok);
    REQUIRE(run(d, "REVOLVE").ok);
    // A STEP file carries the dimensions as typed: 0.1, not 0.100000001490116.
    const std::string path = tempPath("sicdm_decimal.step");
    REQUIRE(run(d, "EXPORT " + path).ok);
    std::FILE* f = std::fopen(path.c_str(), "rb");
    REQUIRE(f != nullptr);
    std::string whole;
    char buf[4096];
    for (std::size_t k; (k = std::fread(buf, 1, sizeof buf, f)) > 0;) whole.append(buf, k);
    std::fclose(f);
    std::filesystem::remove(path);
    CHECK(whole.find("00000001") == std::string::npos);
    CHECK(whole.find("9999999") == std::string::npos);
    CHECK(whole.find("0.8") != std::string::npos); // the revolve's outer radius
    // Turning a revolved (mesh) body away and back leaves its exact geometry where it was.
    Body& r = d.scene.bodies()[1];
    REQUIRE(r.exact());
    const Brep before = *r.brep;
    r.setRotation({30, 20, 10});
    r.setRotation({0, 0, 0});
    REQUIRE(r.exact());
    double worst = 0;
    for (std::size_t i = 0; i < before.vertices.size(); ++i) worst = std::max(worst, dlength(r.brep->vertices[i].p - before.vertices[i].p));
    CHECK(worst < 1e-12);
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

namespace {

// A flat face in z = 0 bounded by a conic arc from (x0, -y0) to (x0, y0) and the straight line back.
std::string conicFace(const std::string& conic, double x0, double y0) {
    char pts[160];
    std::snprintf(pts, sizeof pts, "#7=CARTESIAN_POINT('',(%.15g,%.15g,0.));\n#8=CARTESIAN_POINT('',(%.15g,%.15g,0.));\n", x0, -y0, x0, y0);
    return std::string("ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION((''),'2;1');\nFILE_NAME('conic','',(''),(''),'','','');\n"
                       "FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n"
                       "#1=CARTESIAN_POINT('',(0.,0.,0.));\n#2=DIRECTION('',(0.,0.,1.));\n#3=DIRECTION('',(1.,0.,0.));\n"
                       "#4=AXIS2_PLACEMENT_3D('',#1,#2,#3);\n#5=PLANE('',#4);\n#6=") +
           conic + ";\n" + pts +
           "#9=VERTEX_POINT('',#7);\n#10=VERTEX_POINT('',#8);\n#11=EDGE_CURVE('',#9,#10,#6,.T.);\n"
           "#12=DIRECTION('',(0.,-1.,0.));\n#13=VECTOR('',#12,1.);\n#14=LINE('',#8,#13);\n#15=EDGE_CURVE('',#10,#9,#14,.T.);\n"
           "#16=ORIENTED_EDGE('',*,*,#11,.T.);\n#17=ORIENTED_EDGE('',*,*,#15,.T.);\n#18=EDGE_LOOP('',(#16,#17));\n"
           "#19=FACE_OUTER_BOUND('',#18,.T.);\n#20=ADVANCED_FACE('',(#19),#5,.T.);\n#21=OPEN_SHELL('',(#20));\n"
           "#22=SHELL_BASED_SURFACE_MODEL('',(#21));\nENDSEC;\nEND-ISO-10303-21;\n";
}

double meshArea(const MeshData& m) {
    double a = 0;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const Vec3 p = m.vertices[m.indices[t]].position, q = m.vertices[m.indices[t + 1]].position, r = m.vertices[m.indices[t + 2]].position;
        a += 0.5 * static_cast<double>(length(cross(q - p, r - p)));
    }
    return a;
}

} // namespace

TEST_CASE("STEP and IGES: parabolas and hyperbolas are read and written exactly") {
    const double c = std::cosh(1.0), s = std::sinh(1.0);
    // Between the hyperbola x^2 - y^2 = 1 and x = cosh 1: 2 s c - (s c + asinh(s)) = s c - 1.
    // Between the parabola y^2 = 4x and x = 1: 4 - 16/12.
    const struct {
        std::string entity;
        double x0, y0, area;
    } cases[] = {{"HYPERBOLA('',#4,1.,1.)", c, s, s * c - 1.0}, {"PARABOLA('',#4,1.)", 1.0, 2.0, 4.0 - 16.0 / 12.0}};
    for (const auto& k : cases) {
        StepReadReport rep;
        std::string err;
        const std::vector<StepPart> parts = readStep(conicFace(k.entity, k.x0, k.y0), &rep, &err);
        REQUIRE(parts.size() == 1);
        CHECK(rep.warnings.empty());
        const TessellateOptions fine{0.0005, 0.05, 0};
        CHECK_NEAR(meshArea(tessellate(*parts[0].brep, fine)), k.area, 1e-3 * k.area);
        // Out and back in, both ways, still exact.
        const std::vector<StepPart> again = readStep(writeStep(parts, "conic.step"));
        REQUIRE(again.size() == 1);
        CHECK_NEAR(meshArea(tessellate(*again[0].brep, fine)), k.area, 1e-3 * k.area);
        StepReadReport irep;
        const std::vector<StepPart> viaIges = readIges(writeIges(parts, "conic.igs"), &irep);
        REQUIRE(viaIges.size() == 1);
        CHECK(irep.warnings.empty());
        CHECK_NEAR(meshArea(tessellate(*viaIges[0].brep, fine)), k.area, 1e-3 * k.area);
    }
}

TEST_CASE("STEP import: a solid two representations list is one part; translator names give way to the file's") {
    std::FILE* f = std::fopen(dataPath("box").c_str(), "rb");
    REQUIRE(f != nullptr);
    std::string text;
    for (int ch; (ch = std::fgetc(f)) != EOF;) text += static_cast<char>(ch);
    std::fclose(f);
    // A second representation of the same solid (as some writers add for the part's product).
    const std::size_t brep = text.find("MANIFOLD_SOLID_BREP");
    REQUIRE(brep != std::string::npos);
    const std::size_t hash = text.rfind('#', brep);
    const std::string id = text.substr(hash, text.find('=', hash) - hash);
    const std::size_t rep = text.find("ADVANCED_BREP_SHAPE_REPRESENTATION");
    REQUIRE(rep != std::string::npos);
    const std::size_t close = text.find(";", rep);
    const std::size_t ctx = text.rfind('#', close);
    const std::string context = text.substr(ctx, text.find(')', ctx) - ctx);
    const std::size_t end = text.rfind("ENDSEC;");
    text.insert(end, "#999999=SHAPE_REPRESENTATION(''," + std::string("(") + id + ")," + context + ");\n");
    const std::vector<StepPart> parts = readStep(text);
    CHECK(parts.size() == 1);

    // A part named by its translator takes the file's name.
    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_translator_named.step").string();
    std::string renamed = readStep(text).empty() ? std::string() : text;
    std::size_t prod = renamed.find("PRODUCT(");
    REQUIRE(prod != std::string::npos);
    const std::size_t q1 = renamed.find('\'', prod), q2 = renamed.find('\'', q1 + 1);
    renamed.replace(q1 + 1, q2 - q1 - 1, "Open CASCADE STEP translator 7.5 1");
    const std::size_t q3 = renamed.find('\'', q2 + 1), q4 = renamed.find('\'', q3 + 1);
    renamed.replace(q3 + 1, q4 - q3 - 1, "Open CASCADE STEP translator 7.5 1");
    {
        std::FILE* o = std::fopen(path.c_str(), "wb");
        REQUIRE(o != nullptr);
        std::fwrite(renamed.data(), 1, renamed.size(), o);
        std::fclose(o);
    }
    const std::vector<StepPart> named = readStepFile(path);
    REQUIRE(named.size() == 1);
    CHECK(named[0].name == "sicdm_translator_named");
    std::remove(path.c_str());
}

TEST_CASE("IGES: touching parts stay separate parts, by name") {
    Document d;
    REQUIRE(run(d, "BOX 20 20 20 @0,0,0").ok);
    REQUIRE(run(d, "BOX 20 20 20 @20,0,0").ok);  // face to face with the first
    REQUIRE(run(d, "BOX 10 10 10 @0,0,20").ok);  // standing on it
    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_touching.igs").string();
    REQUIRE(run(d, "EXPORT \"" + path + "\"").ok);
    Document e;
    REQUIRE(run(e, "IMPORT \"" + path + "\"").ok);
    REQUIRE(e.scene.bodies().size() == 3);
    std::vector<std::string> names;
    for (const Body& b : e.scene.bodies()) names.push_back(b.name);
    std::sort(names.begin(), names.end());
    CHECK(names == (std::vector<std::string>{"Block 1", "Block 2", "Block 3"}));
    std::remove(path.c_str());
}

namespace {

// Faces of each kind in a body's exact geometry.
std::map<BSurfaceKind, int> kinds(const Body& b) {
    std::map<BSurfaceKind, int> k;
    if (!b.brep) return k;
    for (const BShell& sh : b.brep->shells)
        for (const BFace& f : sh.faces) ++k[b.brep->surfaces[static_cast<std::size_t>(f.surface)].kind];
    return k;
}

int circles(const Body& b) {
    int n = 0;
    for (const BrepEdge& e : b.brep->edges) n += e.curve >= 0 && b.brep->curves[static_cast<std::size_t>(e.curve)].kind == BCurveKind::Circle;
    return n;
}

} // namespace

TEST_CASE("Exact geometry through holes, pockets, bosses and rounds") {
    Document d;
    REQUIRE(run(d, "BOX 60 40 15 @0,0,0").ok);
    REQUIRE(run(d, "HOLE \"Block 1\" AT -20,0,15 DIAMETER 6 THROUGH").ok);
    const Body& b = d.scene.bodies()[0];
    // A hole through a box: its six planes, the hole's cylinder, its two rims circles.
    REQUIRE(b.exact());
    CHECK(kinds(b)[BSurfaceKind::Plane] == 6);
    CHECK(kinds(b)[BSurfaceKind::Cylinder] == 1);
    CHECK(circles(b) == 2);
    for (const BSurface& s : b.brep->surfaces)
        if (s.kind == BSurfaceKind::Cylinder) CHECK_NEAR(s.r1, 3.0, 1e-6);
    // A blind drilled hole (its cone point closed at the apex), a pocket, a round on an edge and
    // on the hole's rim: still exact, every face on its own surface.
    REQUIRE(run(d, "HOLE \"Block 1\" AT 20,0,15 DIAMETER 8 DEPTH 6").ok);
    REQUIRE(run(d, "POCKET \"Block 1\" AT 0,10,15 WIDTH 10 LENGTH 14 DEPTH 4").ok);
    REQUIRE(run(d, "FILLET \"Block 1\" RADIUS 1 EDGES NEAR -17,0,15").ok);
    REQUIRE(run(d, "FILLET \"Block 1\" RADIUS 2 EDGES NEAR 0,-20,15").ok);
    const Body& c = d.scene.bodies()[0];
    REQUIRE(c.exact());
    auto k = kinds(c);
    CHECK(k[BSurfaceKind::Cylinder] == 3); // two holes and the edge round
    CHECK(k[BSurfaceKind::Cone] == 1);     // the drill point
    CHECK(k[BSurfaceKind::Torus] == 1);    // the round on the hole's rim
    CHECK(k[BSurfaceKind::Plane] == 11);   // the box's six and the pocket's five
    // Its tessellation is closed and holds the mesh's volume.
    TessellateReport tr;
    const MeshData m = tessellate(*c.brep, {}, &tr);
    CHECK(tr.failedFaces == 0);
    CHECK(openEdgeCount(m) == 0);
    CHECK_NEAR(meshVolume(m), meshVolume(c.mesh), 2e-3 * meshVolume(c.mesh));

    // To STEP as exact faces, and back in closed with the same faces.
    const std::string path = tempPath("sicdm_exact_features.step");
    const auto r = run(d, "EXPORT \"" + path + "\"");
    REQUIRE(r.ok);
    CHECK(text(r).find("1 exact, 0 as planar facets") != std::string::npos);
    const std::vector<StepPart> parts = readStepFile(path);
    REQUIRE(parts.size() == 1);
    CHECK(parts[0].brep->faceCount() == c.brep->faceCount());
    std::size_t open = 0, failed = 0;
    const double v = volumeOf(parts, 0.01, &open, &failed);
    CHECK(open == 0);
    CHECK(failed == 0);
    CHECK_NEAR(v, meshVolume(c.mesh), 2e-3 * meshVolume(c.mesh));
    std::filesystem::remove(path);

    // Saved with the project and opened exact again.
    const std::string proj = tempPath("sicdm_exact_features.sicdm");
    REQUIRE(run(d, "SAVE \"" + proj + "\"").ok);
    Document e;
    REQUIRE(run(e, "OPEN \"" + proj + "\"").ok);
    REQUIRE(!e.scene.bodies().empty());
    CHECK(e.scene.bodies()[0].exact());
    CHECK(e.scene.bodies()[0].brep->faceCount() == c.brep->faceCount());
    std::filesystem::remove(proj);
}

TEST_CASE("Exact geometry through booleans of exact bodies, and not for meshes") {
    Document d;
    REQUIRE(run(d, "BOX 40 40 10 @0,0,0").ok);
    REQUIRE(run(d, "CYLINDER 8 20 @0,0,5").ok);
    REQUIRE(run(d, "BOOLEAN \"Block 1\" UNION \"Cylinder 2\"").ok);
    const Body& u = d.scene.bodies()[0];
    REQUIRE(u.exact());
    CHECK(kinds(u)[BSurfaceKind::Cylinder] == 1);
    CHECK(kinds(u)[BSurfaceKind::Plane] == 7); // the box's six and the cylinder's top
    // The boss's foot and top: circles.
    CHECK(circles(u) == 2);
    // A round on the boss's rim: a torus, meeting the cylinder and the top in circles.
    REQUIRE(run(d, "FILLET \"Block 1\" RADIUS 1.5 EDGES NEAR 8,0,25").ok);
    const Body& f = d.scene.bodies()[0];
    REQUIRE(f.exact());
    CHECK(kinds(f)[BSurfaceKind::Torus] == 1);
    CHECK(circles(f) == 3);
    // A sculpted (mesh) body cut by a hole gains no exact geometry of its own.
    Document s;
    REQUIRE(run(s, "SPHERE 20 @0,0,0").ok);
    REQUIRE(run(s, "SCULPT \"Sphere 1\" INFLATE AT 0,0,40 RADIUS 10 STRENGTH 0.5").ok);
    CHECK(!s.scene.bodies()[0].exact());
    REQUIRE(run(s, "HOLE \"Sphere 1\" AT 0,0,40 DIAMETER 4 DEPTH 5").ok);
    CHECK(!s.scene.bodies()[0].exact());
    // The surfaces a profile of revolution makes.
    const auto rs = revolveSurfaces({0, 0, 0}, {0, 0, 1}, {{0, -5}, {2, -4}, {2, 3}, {0, 3}});
    REQUIRE(rs.size() == 3);
    CHECK(rs[0].kind == BSurfaceKind::Cone);
    CHECK(rs[1].kind == BSurfaceKind::Cylinder);
    CHECK(rs[2].kind == BSurfaceKind::Plane);
    double dist;
    DVec3 n;
    REQUIRE(surfaceDistance(rs[0], {1, 0, -4.5}, dist, n));
    CHECK_NEAR(dist, 0.0, 1e-9); // on the cone
}

#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/FileIO.hpp"
#include "core/Json.hpp"
#include "core/Lathe.hpp"
#include "core/Scene.hpp"

#include <cmath>
#include <filesystem>
#include <algorithm>
#include <sstream>
#include <string>

using namespace sicdm;

#define REQUIRE(c)                  \
    do {                            \
        const bool required_ = (c); \
        CHECK(required_);           \
        if (!required_) return;     \
    } while (0)

namespace {

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }

std::string text(const CommandOutput& r) {
    std::string s;
    for (const std::string& l : r.lines) s += l + "\n";
    return s;
}

bool has(const std::string& s, const std::string& what) { return s.find(what) != std::string::npos; }

// A shaft along Z, 60 long: 30 across for 20 at the back, a 4 mm neck 24 across, 30 across
// again for 16, then 20 across for the front 20, with an 8 mm bore 15 deep in its front.
bool shaft(Document& d) {
    for (const char* l : {"CYLINDER 15 20 @0,0,0", "CYLINDER 12 4 @0,0,20", "CYLINDER 15 16 @0,0,24", "CYLINDER 10 20 @0,0,40",
                          "BOOLEAN \"Cylinder 1\" UNION \"Cylinder 2\"", "BOOLEAN \"Cylinder 1\" UNION \"Cylinder 3\"",
                          "BOOLEAN \"Cylinder 1\" UNION \"Cylinder 4\"", "CYLINDER 4 16 @0,0,45", "BOOLEAN \"Cylinder 1\" SUBTRACT \"Cylinder 5\"",
                          "LATHE SETUP \"Cylinder 1\"", "LATHE TOOL 1 TURN NOSE 0.8 INSERT C", "LATHE TOOL 2 TURN NOSE 0.4 INSERT D",
                          "LATHE TOOL 3 GROOVE 3", "LATHE TOOL 4 DRILL 6", "LATHE TOOL 5 BORE 5 NOSE 0.2", "LATHE TOOL 6 THREAD"})
        if (!run(d, l).ok) return false;
    return true;
}

const Body& partOf(const Document& d) { return d.scene.bodies()[0]; }

// The simulated outside radius at lathe Z.
double outerAt(const LatheSimulation& s, double z) {
    const long i = static_cast<long>(std::llround((z - s.z0) / s.dz));
    return i >= 0 && i < static_cast<long>(s.outer.size()) ? s.outer[static_cast<std::size_t>(i)] : -1.0;
}

} // namespace

TEST_CASE("Lathe profile: a stepped shaft with a neck and a bore, as the lathe sees it") {
    Document d;
    REQUIRE(shaft(d));
    const LatheJob& job = d.state.lathe;
    CHECK(job.setup.axis == 2); // its longest side
    LatheProfile p;
    REQUIRE(latheProfile(job, partOf(d), p));
    CHECK_NEAR(p.length(), 60.0, 0.01);
    CHECK_NEAR(p.maxRadius, 15.0, 0.01);
    CHECK_NEAR(latheStockDiameter(job, p), 32.0, 1e-9);
    auto at = [&](double z, bool inner) {
        const std::size_t i = static_cast<std::size_t>(std::llround((z - p.z0) / p.dz));
        return inner ? p.inner[i] : p.outer[i];
    };
    CHECK_NEAR(at(-10, false), 10.0, 0.05); // the front 20 across
    CHECK_NEAR(at(-30, false), 15.0, 0.05);
    CHECK_NEAR(at(-38, false), 12.0, 0.05); // the neck
    CHECK_NEAR(at(-50, false), 15.0, 0.05);
    CHECK(at(-5, true) > 3.9);              // the bore
    CHECK(at(-5, true) < 4.0 + 1e-6);
    CHECK(at(-20, true) == 0.0);            // solid behind it
    const std::string t = text(run(d, "LATHE PROFILE"));
    CHECK(has(t, "a bore from the front"));
    // Inserts: how steeply each goes down behind a shoulder.
    LatheTool c, dd, v;
    c.insertAngle = 80, dd.insertAngle = 55, v.insertAngle = 35;
    CHECK_NEAR(c.relief(), 3.0, 1e-9);
    CHECK_NEAR(dd.relief(), 30.0, 1e-9);
    CHECK_NEAR(v.relief(), 50.0, 1e-9);
}

TEST_CASE("Lathe job: face, turn, groove, drill, bore, thread and part off a shaft, simulated clean and posted") {
    Document d;
    REQUIRE(shaft(d));
    for (const char* l : {"LATHE FACE", "LATHE ROUGH TOOL 1 LEAVE 0.2", "LATHE FINISH TOOL 2"}) REQUIRE(run(d, l).ok);
    LatheJob& job = d.state.lathe;
    // The turning inserts can't get down into the neck.
    LatheSimulation s;
    REQUIRE(simulateLathe(job, partOf(d), s));
    CHECK(s.gouges == 0);
    CHECK(s.rapidCrashes == 0);
    CHECK(outerAt(s, -38) > 12.5);
    CHECK_NEAR(outerAt(s, -10), 10.0, 0.02); // the front finished
    CHECK_NEAR(outerAt(s, -30), 15.0, 0.02);
    for (const char* l : {"LATHE GROOVE", "LATHE DRILL", "LATHE BORE LEAVE 0.1", "LATHE FINISH INSIDE", "LATHE THREAD PITCH 1.5 FROM 0 TO -14", "LATHE PARTOFF"}) {
        const auto r = run(d, l);
        CHECK(r.ok);
        if (!r.ok) std::printf("%s: %s\n", l, text(r).c_str());
    }
    REQUIRE(job.ops.size() == 9);
    REQUIRE(simulateLathe(job, partOf(d), s));
    CHECK(s.gouges == 0);
    CHECK(s.rapidCrashes == 0);
    CHECK(s.partedOff);
    CHECK(s.meanLeft < 0.01);
    CHECK(s.maxLeft < 0.45);                 // the finishing nose's radius in the inside corners
    CHECK(outerAt(s, -38) < 12.2);           // the neck grooved
    CHECK(outerAt(s, -7) < 10.0 - 0.8);      // the thread cut
    CHECK(s.seconds > 10);
    const std::string sim = text(run(d, "LATHE SIMULATE"));
    CHECK(has(sim, "no gouges"));
    CHECK(has(sim, "parted off"));
    // Fanuc: tools with their offsets, constant surface speed under a spindle limit, feed per
    // turn, diameters, threading passes.
    const std::string g = postLathe(job, partOf(d), "fanuc");
    for (const char* w : {"G99", "T0101", "T0606", "G50 S4000", "G96 S", "G97", "G32 X", " F1.5", "X20. ", "G28 U0. W0.", "M30"}) {
        CHECK(has(g, w));
        if (!has(g, w)) std::printf("fanuc lacks %s\n", w);
    }
    std::size_t threads = 0;
    std::istringstream in(g);
    for (std::string l; std::getline(in, l);) threads += l.rfind("G32", 0) == 0 ? 1 : 0;
    CHECK(threads == 9); // 8 passes to 0.92 deep, and a spring pass
    const std::string lc = postLathe(job, partOf(d), "linuxcnc");
    for (const char* w : {"G7 G18", "G95", "G96 D4000 S", "G33 X", " K1.5", "T1 M6", "M2"}) CHECK(has(lc, w));
    CHECK(has(postLathe(job, partOf(d), "haas"), "O01000"));
    std::string err;
    CHECK(postLathe(job, partOf(d), "grbl", &err).empty());
    CHECK(has(err, "doesn't turn"));
    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_lathe.nc").string();
    const auto r = run(d, "LATHE POST " + path + " LINUXCNC");
    CHECK(r.ok);
    CHECK(std::filesystem::exists(path));
    std::filesystem::remove(path);
    // The turned stock as a body.
    CHECK(run(d, "LATHE SIMULATE SHOW").ok);
    bool shown = false;
    for (const Body& b : d.scene.bodies()) shown = shown || (b.name == "Stock (turned)" && b.mesh.indices.size() > 1000);
    CHECK(shown);
}

TEST_CASE("Lathe refuses what it can't do") {
    Document d;
    REQUIRE(shaft(d));
    auto fails = [&](const std::string& l, const char* why) {
        const auto r = run(d, l);
        CHECK(!r.ok);
        CHECK(has(text(r), why));
        if (r.ok || !has(text(r), why)) std::printf("%s: %s\n", l.c_str(), text(r).c_str());
    };
    fails("LATHE BORE TOOL 5", "Drill the bore first");
    fails("LATHE FACE TOOL 3", "turning insert");
    fails("LATHE THREAD", "PITCH");
    fails("LATHE SETUP BAR 20", "thinner than the part");
    REQUIRE(run(d, "LATHE TOOL 7 DRILL 10").ok);
    fails("LATHE DRILL TOOL 7", "wider than the bore");
    REQUIRE(run(d, "LATHE TOOL 8 GROOVE 3 REACH 10").ok);
    fails("LATHE PARTOFF TOOL 8", "too short");
    fails("LATHE FINISH TOOL 4", "turning insert");
    CHECK(d.state.lathe.ops.empty());
    // A turning insert alone leaves no recess for a blade to find on a plain bar.
    Document e;
    for (const char* l : {"CYLINDER 10 30", "LATHE SETUP \"Cylinder 1\"", "LATHE TOOL 1 TURN", "LATHE TOOL 2 GROOVE 3"}) REQUIRE(run(e, l).ok);
    const auto g = run(e, "LATHE GROOVE");
    CHECK(!g.ok);
    CHECK(has(text(g), "No recesses"));
}

TEST_CASE("Lathe job saves with the project; a part along X turns about X, faced from either end") {
    Document d;
    REQUIRE(shaft(d));
    for (const char* l : {"LATHE FACE", "LATHE ROUGH TOOL 1 LEAVE 0.2 STEPDOWN 1.5", "LATHE THREAD PITCH 1.25 FROM -2 TO -12 NAME \"M20 thread\""})
        REQUIRE(run(d, l).ok);
    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_lathe.sicdm").string();
    REQUIRE(run(d, "SAVE " + path).ok);
    Document e;
    REQUIRE(run(e, "OPEN " + path).ok);
    const LatheJob& j = e.state.lathe;
    CHECK(j.setup.body == "Cylinder 1");
    REQUIRE(j.tools.size() == 6);
    CHECK(j.tools[0].insertAngle == 80);
    CHECK_NEAR(j.tools[0].noseRadius, 0.8, 1e-9);
    CHECK(j.tools[2].type == LatheToolType::Groove && j.tools[2].width == 3);
    REQUIRE(j.ops.size() == 3);
    CHECK(j.ops[1].kind == LatheOpKind::Rough && j.ops[1].leave == 0.2 && j.ops[1].stepdown == 1.5);
    CHECK(j.ops[2].pitch == 1.25 && j.ops[2].zFrom == -2 && j.ops[2].zTo == -12 && j.ops[2].name == "M20 thread");
    CHECK(run(e, "LATHE SIMULATE").ok); // the paths are remade
    std::filesystem::remove(path);
    // Along X.
    Document x;
    for (const char* l : {"CYLINDER 10 40 @0,0,0", "ROTATE \"Cylinder 1\" 0,90,0", "LATHE SETUP \"Cylinder 1\""}) REQUIRE(run(x, l).ok);
    CHECK(x.state.lathe.setup.axis == 0);
    LatheFrame f;
    REQUIRE(latheFrame(x.state.lathe, partOf(x), f));
    CHECK_NEAR(f.axis.x, 1.0, 1e-6);
    const AABB box = partOf(x).bounds();
    CHECK_NEAR(f.origin.x, box.max.x, 1e-3); // Z0 at the +X end
    REQUIRE(run(x, "LATHE SETUP FLIP").ok);
    REQUIRE(latheFrame(x.state.lathe, partOf(x), f));
    CHECK_NEAR(f.axis.x, -1.0, 1e-6);
    CHECK_NEAR(f.origin.x, box.min.x, 1e-3);
}

TEST_CASE("Lathe in a bore: drilled, bored, grooved and threaded inside, simulated clean") {
    Document d;
    for (const char* l : {"CYLINDER 20 40 @0,0,0", "CYLINDER 8 30 @0,0,10", "BOOLEAN \"Cylinder 1\" SUBTRACT \"Cylinder 2\"", "CYLINDER 10 3 @0,0,20",
                          "BOOLEAN \"Cylinder 1\" SUBTRACT \"Cylinder 3\"", "LATHE SETUP \"Cylinder 1\"", "LATHE TOOL 1 TURN NOSE 0.8 INSERT C",
                          "LATHE TOOL 4 DRILL 12", "LATHE TOOL 5 BORE 10 NOSE 0.2", "LATHE TOOL 7 GROOVE 2 DIAMETER 12 REACH 6",
                          "LATHE TOOL 8 THREAD DIAMETER 12"})
        REQUIRE(run(d, l).ok);
    CHECK(d.state.lathe.setup.axis == 2); // 40 by 40 by 40: a tie goes to Z
    for (const char* l : {"LATHE FACE", "LATHE ROUGH TOOL 1 LEAVE 0.2", "LATHE FINISH TOOL 1", "LATHE DRILL", "LATHE BORE LEAVE 0.1", "LATHE FINISH INSIDE"})
        REQUIRE(run(d, l).ok);
    // The bore's groove is past the boring bar's reach (a recess); the grooving bar takes it.
    LatheJob& job = d.state.lathe;
    LatheSimulation s;
    REQUIRE(simulateLathe(job, partOf(d), s));
    auto innerAt = [&](double z) {
        const long i = static_cast<long>(std::llround((z - s.z0) / s.dz));
        return s.inner[static_cast<std::size_t>(i)];
    };
    CHECK(innerAt(-18.5) < 9.0); // the groove (radius 10, Z -20 to -17) not yet cut
    auto r = run(d, "LATHE GROOVE INSIDE TOOL 7");
    REQUIRE(r.ok);
    CHECK(has(text(r), "Groove the bore"));
    r = run(d, "LATHE THREAD INSIDE TOOL 8 PITCH 1.5 FROM 0 TO -8");
    REQUIRE(r.ok);
    REQUIRE(simulateLathe(job, partOf(d), s));
    CHECK(s.gouges == 0);
    CHECK(s.rapidCrashes == 0);
    CHECK(s.maxLeft < 0.25);
    CHECK(innerAt(-18.5) > 9.98); // grooved
    CHECK_NEAR(innerAt(-4), 8.0 + 0.5413 * 1.5, 0.02); // threaded out to the major diameter
    const std::string g = postLathe(job, partOf(d), "fanuc");
    CHECK(has(g, "T0808"));
    CHECK(has(g, "G32 X"));
    // A bar too big for the bore is refused.
    REQUIRE(run(d, "LATHE TOOL 9 THREAD DIAMETER 30").ok);
    r = run(d, "LATHE THREAD INSIDE TOOL 9 PITCH 1.5 FROM 0 TO -8");
    CHECK(!r.ok);
    CHECK(has(text(r), "needs a 30 mm bore"));
}

TEST_CASE("Lathe posts fit arcs (R, under half a turn) to the round parts of a profile") {
    Document d;
    REQUIRE(shaft(d));
    for (const char* l : {"LATHE FACE", "LATHE ROUGH TOOL 1 LEAVE 0.2", "LATHE FINISH TOOL 2"}) REQUIRE(run(d, l).ok);
    LatheJob& job = d.state.lathe;
    const std::string g = postLathe(job, partOf(d), "linuxcnc");
    // The finish's programmed path (the imaginary tip), as straight segments.
    const LatheOperation& fin = job.ops[2];
    const LatheTool* t2 = job.tool(2);
    REQUIRE(t2 != nullptr);
    std::vector<std::pair<double, double>> path; // (z, x radius)
    for (const LatheMove& m : fin.moves) path.push_back({m.z - t2->noseRadius, m.x - t2->noseRadius});
    auto nearPath = [&](double z, double x) {
        double best = 1e9;
        for (std::size_t i = 1; i < path.size(); ++i) {
            const double az = path[i - 1].first, ax = path[i - 1].second, bz = path[i].first, bx = path[i].second;
            const double l2 = (bz - az) * (bz - az) + (bx - ax) * (bx - ax);
            const double u = l2 > 0 ? std::clamp(((z - az) * (bz - az) + (x - ax) * (bx - ax)) / l2, 0.0, 1.0) : 0.0;
            best = std::min(best, std::hypot(z - az - u * (bz - az), x - ax - u * (bx - ax)));
        }
        return best;
    };
    // Read the finish back: each arc from where the last move ended, its middle on the path.
    std::istringstream in(g.substr(g.find("(3 ")));
    double z = 0, x = 0;
    int g0 = 0, arcs = 0, bad = 0;
    auto word = [](const std::string& l, char w, double& v) {
        for (std::size_t i = 0; i < l.size(); ++i)
            if (l[i] == w && (i == 0 || l[i - 1] == ' ')) {
                v = std::atof(l.c_str() + i + 1);
                return true;
            }
        return false;
    };
    for (std::string l; std::getline(in, l);) {
        if (l.rfind("G2 ", 0) == 0) g0 = 2;
        else if (l.rfind("G3 ", 0) == 0) g0 = 3;
        else if (l.rfind("G0 ", 0) == 0 || l.rfind("G1 ", 0) == 0) g0 = l[1] - '0';
        double nx = x * 2, nz = z, r = 0;
        const bool hasX = word(l, 'X', nx), hasZ = word(l, 'Z', nz);
        if (!hasX && !hasZ) continue;
        nx *= 0.5; // diameters
        if ((g0 == 2 || g0 == 3) && word(l, 'R', r)) {
            ++arcs;
            // The centre: on the chord's bisector, to the left (G3, counterclockwise) or right.
            const double mz = 0.5 * (z + nz), mx = 0.5 * (x + nx), cz = nz - z, cx = nx - x, c = std::hypot(cz, cx);
            const double h = std::sqrt(std::max(0.0, r * r - c * c / 4));
            const double s = g0 == 3 ? 1.0 : -1.0;
            const double oz = mz - s * cx / c * h, ox = mx + s * cz / c * h;
            // The arc's middle: from the centre through the chord's middle, a radius out.
            const double dz = mz - oz, dx = mx - ox, dl = std::hypot(dz, dx);
            const double az = oz + dz / dl * r, ax = ox + dx / dl * r;
            if (nearPath(az, ax) > job.setup.tolerance * 1.5) ++bad;
        }
        z = nz, x = nx;
        if (l.rfind("M9", 0) == 0) break;
    }
    CHECK(arcs >= 2); // the nose's corners round the shoulders
    CHECK(bad == 0);
}

TEST_CASE("Lathe posts with CYCLES: G71 rough turning and boring, G72 facing; written out where type I can't") {
    Document d;
    for (const char* l : {"CYLINDER 15 25 @0,0,0", "CYLINDER 10 20 @0,0,25", "BOOLEAN \"Cylinder 1\" UNION \"Cylinder 2\"", "CYLINDER 6 15 @0,0,35",
                          "BOOLEAN \"Cylinder 1\" SUBTRACT \"Cylinder 3\"", "LATHE SETUP \"Cylinder 1\"", "LATHE TOOL 1 TURN NOSE 0.8 INSERT C",
                          "LATHE TOOL 4 DRILL 8", "LATHE TOOL 5 BORE 8 NOSE 0.2", "LATHE FACE", "LATHE ROUGH TOOL 1 LEAVE 0.2 LEAVEZ 0.1", "LATHE DRILL",
                          "LATHE BORE LEAVE 0.1"})
        REQUIRE(run(d, l).ok);
    LatheJob& job = d.state.lathe;
    const std::string g = postLathe(job, partOf(d), "fanuc", nullptr, true);
    for (const char* w : {"G72 W2. R0.5", "G72 P1000 Q1001 U0. W0. F0.35", "N1000 G00 Z0.", "G71 U2. R0.5", "G71 P2000 Q2001 U0.4 W0.1 F0.35",
                          "N2000 G00 X", "G71 P4000 Q4001 U-0.2 W0.", "N4001 X8. Z"}) {
        CHECK(has(g, w));
        if (!has(g, w)) std::printf("lacks %s\n", w);
    }
    // The outside outline between N2000 and N2001: X never falls going towards the chuck, and
    // it ends on the bar.
    std::istringstream in(g.substr(g.find("N2000"), g.find("M09", g.find("N2000")) - g.find("N2000")));
    double lastX = -1, x = 0;
    bool mono = true;
    for (std::string l; std::getline(in, l);) {
        const auto at = l.find(" X") != std::string::npos ? l.find(" X") + 2 : l.rfind("X", 0) == 0 ? 1 : std::string::npos;
        if (at == std::string::npos) continue;
        x = std::atof(l.c_str() + at);
        if (x < lastX - 1e-6) mono = false;
        lastX = x;
    }
    CHECK(mono);
    CHECK_NEAR(x, 32.0, 1e-3);
    CHECK(has(postLathe(job, partOf(d), "haas", nullptr, true), "G71 P2000 Q2001 U0.4 W0.1 D2. F0.35"));
    CHECK(!has(postLathe(job, partOf(d), "linuxcnc", nullptr, true), "G71"));
    CHECK(!has(postLathe(job, partOf(d), "fanuc"), "G71")); // only when asked
    // A neck the turning insert can't follow: that operation is written out pass by pass.
    Document e;
    REQUIRE(shaft(e));
    REQUIRE(run(e, "LATHE ROUGH TOOL 1 LEAVE 0.2").ok);
    const std::string g2 = postLathe(e.state.lathe, partOf(e), "fanuc", nullptr, true);
    CHECK(has(g2, "(PASSES WRITTEN OUT: THE OUTLINE GOES DOWN BEHIND A SHOULDER"));
    CHECK(!has(g2, "G71 P"));
    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_cycles.nc").string();
    const auto r = run(d, "LATHE POST " + path + " CYCLES");
    CHECK(r.ok);
    CHECK(has(text(r), "2 G71 and 1 G72 cycles"));
    std::filesystem::remove(path);
}

TEST_CASE("Lathe operations edited in place, moved, suppressed and duplicated") {
    Document d;
    REQUIRE(shaft(d));
    for (const char* l : {"LATHE FACE", "LATHE ROUGH TOOL 1 LEAVE 0.2", "LATHE FINISH TOOL 2"}) REQUIRE(run(d, l).ok);
    LatheJob& job = d.state.lathe;
    REQUIRE(run(d, "LATHE EDIT 2 LEAVE 0.4 STEPDOWN 1.5").ok);
    CHECK(job.ops[1].leave == 0.4);
    CHECK(job.ops[1].stepdown == 1.5);
    CHECK(job.ops[1].kind == LatheOpKind::Rough);
    CHECK(job.ops[1].generated);
    CHECK(!job.ops[2].generated);
    REQUIRE(run(d, "LATHE EDIT 2 STEPDOWN AUTO FROM -5").ok);
    CHECK(job.ops[1].stepdown == 0);
    CHECK(job.ops[1].zFrom == -5);
    CHECK(!run(d, "LATHE EDIT 2 TOOL 9").ok);
    CHECK(job.ops[1].tool == 1);
    REQUIRE(run(d, "LATHE MOVE 3 UP").ok);
    CHECK(job.ops[1].kind == LatheOpKind::Finish);
    REQUIRE(run(d, "LATHE MOVE 2 DOWN").ok);
    REQUIRE(run(d, "LATHE DUPLICATE 3").ok);
    REQUIRE(job.ops.size() == 4);
    REQUIRE(run(d, "LATHE SUPPRESS 4").ok);
    LatheSimulation s;
    REQUIRE(simulateLathe(job, partOf(d), s));
    CHECK(s.gouges == 0);
    CHECK(!job.ops[3].generated);
    std::string err;
    const std::string g = postLathe(job, partOf(d), "fanuc", &err);
    REQUIRE(!g.empty());
    CHECK(has(text(run(d, "LATHE LIST")), "(suppressed)"));
    Json j = latheToJson(job);
    LatheJob back;
    REQUIRE(latheFromJson(j, back));
    CHECK(back.ops[3].suppressed);
}

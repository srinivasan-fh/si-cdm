#include "TestFramework.hpp"

#include "core/Cam.hpp"
#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Scene.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
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

std::string slurp(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    std::ostringstream s;
    s << f.rdbuf();
    return s.str();
}

std::string tempPath(const char* name) { return (std::filesystem::temp_directory_path() / name).string(); }

// A 60 x 40 x 15 plate with two 6 mm through holes and a 16 x 24 x 6 pocket.
void bracket(Document& d) {
    REQUIRE(run(d, "BOX 60 40 15 @0,0,0").ok);
    REQUIRE(run(d, "HOLE \"Block 1\" AT -20,0,15 DIAMETER 6 THROUGH").ok);
    REQUIRE(run(d, "HOLE \"Block 1\" AT 20,0,15 DIAMETER 6 THROUGH").ok);
    REQUIRE(run(d, "POCKET \"Block 1\" AT 0,0,15 BOX WIDTH 16 LENGTH 24 DEPTH 6").ok);
}

// Every G2 / G3 in the program ends on its circle (start and end the same distance from the centre).
int badArcs(const std::string& g, int* arcs) {
    std::istringstream in(g);
    std::string l;
    double x = 0, y = 0;
    int bad = 0;
    *arcs = 0;
    while (std::getline(in, l)) {
        if (l.empty() || l[0] == '(' || l[0] == '%' || l[0] == 'O' || l[0] == 'N') continue;
        auto word = [&](char w, double* v) {
            for (std::size_t i = 0; i < l.size(); ++i)
                if (l[i] == w && (i == 0 || l[i - 1] == ' ')) {
                    *v = std::atof(l.c_str() + i + 1);
                    return true;
                }
            return false;
        };
        double nx = x, ny = y, i = 0, j = 0;
        word('X', &nx);
        word('Y', &ny);
        const bool arc = l.rfind("G02", 0) == 0 || l.rfind("G03", 0) == 0 || l.rfind("G2 ", 0) == 0 || l.rfind("G3 ", 0) == 0;
        if (arc && word('I', &i) && word('J', &j)) {
            ++*arcs;
            const double cx = x + i, cy = y + j;
            if (std::fabs(std::hypot(i, j) - std::hypot(nx - cx, ny - cy)) > 0.01) ++bad;
        }
        x = nx, y = ny;
    }
    return bad;
}

} // namespace

TEST_CASE("CAM feeds and speeds: cutting speed, chip load, chip thinning, machine limit, taps") {
    const CamMaterial* al = camMaterial("aluminum");
    REQUIRE(al != nullptr);
    CHECK(al->key == "aluminium");
    CamTool t;
    t.diameter = 6;
    t.flutes = 3;
    CamMachine m;
    CamFeeds f = camFeeds(t, *al, m);
    CHECK_NEAR(f.rpm, 12000, 1e-6);              // 300 m/min wants 15 900 rpm: the machine's limit
    CHECK_NEAR(f.chipLoad, 0.048, 1e-9);         // 0.008 mm per mm of diameter
    CHECK_NEAR(f.feed, 12000 * 3 * 0.048, 1.0);  // rpm x flutes x chip load
    const CamFeeds thin = camFeeds(t, *al, m, 0.6); // 10% stepover: the chip thins, the feed rises
    CHECK_NEAR(thin.chipLoad, 0.048 * 6.0 / (2.0 * std::sqrt(0.6 * 5.4)), 1e-6);
    CHECK(thin.feed > f.feed * 1.6);
    m.maxRpm = 30000;
    CHECK_NEAR(camFeeds(t, *al, m).rpm, 15910, 10.0);
    const CamMaterial* steel = camMaterial("steel");
    REQUIRE(steel != nullptr);
    CHECK(camFeeds(t, *steel, m).rpm < 7000);
    CamTool tap;
    tap.type = CamToolType::Tap;
    tap.diameter = 6;
    tap.pitch = 1.0;
    const CamFeeds tf = camFeeds(tap, *al, m);
    CHECK_NEAR(tf.feed, tf.rpm * 1.0, 1e-6);     // one pitch a turn
    CHECK(camMaterial("unobtainium") == nullptr);
}

TEST_CASE("CAM drop-cutter: flat, ball and bull-nose tips sit exactly on a block's edge") {
    Document d;
    REQUIRE(run(d, "BOX 60 40 15 @0,0,0").ok);
    const Body& b = d.scene.bodies()[0];
    CamTool flat;
    flat.diameter = 6;
    CHECK_NEAR(cutterHeight(b, flat, 0, 0, -10), 15.0, 1e-9);
    CHECK_NEAR(cutterHeight(b, flat, 0, 22.9, -10), 15.0, 1e-9); // the rim still over the edge
    CHECK_NEAR(cutterHeight(b, flat, 0, 23.1, -10), -10.0, 1e-9);
    CHECK_NEAR(cutterHeight(b, flat, 32.9, 0, -10), 15.0, 1e-9);
    CamTool ball;
    ball.type = CamToolType::Ball;
    ball.diameter = 6;
    for (double x : {31.0, 32.0, 32.9}) CHECK_NEAR(cutterHeight(b, ball, x, 0, -10), 12.0 + std::sqrt(9.0 - (x - 30) * (x - 30)), 1e-6);
    CamTool bull;
    bull.type = CamToolType::BullNose;
    bull.diameter = 8;
    bull.cornerRadius = 1;
    CHECK_NEAR(cutterHeight(b, bull, 33.0, 0, -10), 15.0, 1e-6);                        // the flat of its tip
    CHECK_NEAR(cutterHeight(b, bull, 33.5, 0, -10), 15.0 - (1.0 - std::sqrt(0.75)), 1e-6); // on its corner
    CHECK_NEAR(cutterHeight(b, bull, 34.1, 0, -10), -10.0, 1e-9);
}

TEST_CASE("CAM finds a part's round holes: size, place, depth, through or blind") {
    Document d;
    bracket(d);
    REQUIRE(run(d, "HOLE \"Block 1\" AT 0,15,15 DIAMETER 4 DEPTH 5").ok);
    const auto holes = findHoles(d.scene.bodies()[0]);
    REQUIRE(holes.size() == 3);
    int through = 0;
    for (const CamHole& h : holes) {
        if (h.through) {
            ++through;
            CHECK_NEAR(h.diameter, 6.0, 0.01);
            CHECK_NEAR(std::fabs(h.x), 20.0, 1e-3);
            CHECK_NEAR(h.y, 0.0, 1e-3);
            CHECK_NEAR(h.top, 15.0, 1e-3);
            CHECK_NEAR(h.bottom, 0.0, 1e-3);
        } else {
            CHECK_NEAR(h.diameter, 4.0, 0.01);
            CHECK_NEAR(h.top, 15.0, 1e-3);
            CHECK(h.bottom > 9.5); // about 5 deep (the drill point below that)
        }
    }
    CHECK(through == 2);
}

TEST_CASE("CAM job: face, pocket, contour with tabs and peck drilling cut the part without gouges") {
    Document d;
    bracket(d);
    REQUIRE(run(d, "CAM SETUP \"Block 1\" STOCK 3 TOP 1 MATERIAL aluminium").ok);
    REQUIRE(run(d, "CAM TOOL 1 FLAT 6").ok);
    REQUIRE(run(d, "CAM TOOL 2 DRILL 6").ok);
    REQUIRE(run(d, "CAM TOOL 3 FACE 40").ok);
    auto r = run(d, "CAM FACE");
    REQUIRE(r.ok);
    CHECK(has(text(r), "with T3"));
    REQUIRE(run(d, "CAM POCKET").ok);
    REQUIRE(run(d, "CAM CONTOUR TABS 4 TABHEIGHT 1.5").ok);
    r = run(d, "CAM DRILL CYCLE PECK");
    REQUIRE(r.ok);
    CHECK(has(text(r), "2 holes"));
    const CamJob& job = d.state.cam;
    REQUIRE(job.ops.size() == 4);
    for (const CamOperation& op : job.ops) CHECK(op.generated);
    // The contour's last depths rise over four tabs 1.5 mm high.
    int tabMoves = 0;
    for (const CamMove& m : job.ops[2].moves)
        if (m.kind == CamMove::Feed && std::fabs(m.to.z - 1.5f) < 1e-3f) ++tabMoves;
    CHECK(tabMoves >= 8);
    // No cut inside the part's outline below its top, except in the pocket and the holes.
    for (const CamOperation& op : job.ops)
        for (const CamMove& m : op.moves)
            if (op.kind != CamOpKind::Drill && m.kind != CamMove::Rapid && m.to.z < 14.999f) {
                const bool inPocket = std::fabs(m.to.x) <= 8.0f - 3.0f + 1e-3f && std::fabs(m.to.y) <= 12.0f - 3.0f + 1e-3f;
                // The tool's centre at least its radius from the plate's outline.
                const double ox = std::max(std::fabs(m.to.x) - 30.0, 0.0), oy = std::max(std::fabs(m.to.y) - 20.0, 0.0);
                const bool overPart = std::hypot(ox, oy) < 3.0 - 1e-3;
                CHECK(inPocket || !overPart);
            }
    r = run(d, "CAM SIMULATE");
    REQUIRE(r.ok);
    const std::string s = text(r);
    CHECK(has(s, "no gouges"));
    CHECK(has(s, "no rapid moves through stock"));
    CamSimulation sim;
    REQUIRE(simulateJob(d.state.cam, d.scene.bodies()[0], sim));
    // Stock 66 x 46 x 16 less the part (plate, two holes, the pocket) less what a round
    // tool can't reach (the pocket's corners) and the tabs.
    const double part = 60.0 * 40 * 15 - 2 * 3.14159265 * 9 * 15 - 16.0 * 24 * 6;
    const double cut = 66.0 * 46 * 16 - part;
    CHECK(sim.removed < cut + 60);
    CHECK(sim.removed > cut - 700);
    CHECK(sim.gouges == 0);
    CHECK(sim.rapidCrashes == 0);
    CHECK(sim.seconds > 30);
    REQUIRE(run(d, "CAM SIMULATE SHOW").ok);
    CHECK(d.scene.bodies().back().name == "Stock (simulated)");
    CHECK(!d.scene.bodies()[0].visible);
}

TEST_CASE("CAM post: Fanuc, Haas, LinuxCNC and GRBL programs with arcs, cycles and safe tool changes") {
    Document d;
    bracket(d);
    REQUIRE(run(d, "CAM SETUP \"Block 1\" STOCK 3").ok);
    REQUIRE(run(d, "CAM TOOL 1 FLAT 6").ok);
    REQUIRE(run(d, "CAM TOOL 2 DRILL 6").ok);
    REQUIRE(run(d, "CAM CONTOUR").ok);
    REQUIRE(run(d, "CAM DRILL PECK 3").ok);
    const std::string path = tempPath("sicdm_cam_test.nc");
    for (const char* post : {"fanuc", "haas", "linuxcnc", "grbl"}) {
        const auto r = run(d, std::string("CAM POST ") + path + " " + post);
        REQUIRE(r.ok);
        const std::string g = slurp(path);
        int arcs = 0;
        CHECK(badArcs(g, &arcs) == 0);
        CHECK(arcs >= 4); // the profile's rounded corners
        const bool grbl = std::string(post) == "grbl";
        CHECK(has(g, "G21"));
        CHECK(has(g, "G54"));
        CHECK(has(g, " M3") || has(g, " M03"));
        if (grbl) {
            CHECK(!has(g, "G83"));
            CHECK(!has(g, "M6"));
            CHECK(!has(g, "G43"));
            CHECK(has(g, "M0 (Change to T2"));
            CHECK(has(g, "M30"));
        } else {
            CHECK(has(g, "G83"));
            CHECK(has(g, " Q3"));
            CHECK(has(g, "G80"));
            CHECK(has(g, "G43 H"));
            CHECK(has(g, std::string(post) == "linuxcnc" ? "T2 M6" : "T2 M06"));
            // After each tool change, no Z move before the length offset is on.
            std::istringstream in(g);
            std::string l;
            bool changed = false, bad = false;
            while (std::getline(in, l)) {
                if (has(l, " M06") || has(l, " M6")) changed = true;
                else if (changed && has(l, "G43")) changed = false;
                else if (changed && l[0] != '(' && has(l, "Z")) bad = true;
            }
            CHECK(!bad);
        }
        if (std::string(post) == "fanuc") {
            CHECK(g.rfind("%\nO1000", 0) == 0);
            CHECK(has(g, "G91 G28 Z0."));
            CHECK(has(g, "M30\n%"));
        }
        if (std::string(post) == "haas") CHECK(has(g, "G53"));
        if (std::string(post) == "linuxcnc") CHECK(has(g, "G64 P"));
    }
    std::filesystem::remove(path);
    REQUIRE(run(d, "CAM TOOL 3 TAP 6").ok);
    REQUIRE(run(d, "CAM TAP TOOL 3 AT -20,0 DEPTH 8").ok);
    CHECK(!run(d, "CAM POST " + path + " grbl").ok); // GRBL can't tap
    const auto fanuc = run(d, "CAM POST " + path + " fanuc");
    REQUIRE(fanuc.ok);
    const std::string g = slurp(path);
    CHECK(has(g, "M29 S"));
    CHECK(has(g, "G84"));
    std::filesystem::remove(path);
}

TEST_CASE("CAM 3D: roughing, parallel and waterline finishing over a dome leave little and gouge nothing") {
    Document d;
    REQUIRE(run(d, "BOX 60 40 10 @0,0,0").ok);
    REQUIRE(run(d, "SPHERE 15 @0,0,-5").ok); // a dome over the plate's top
    REQUIRE(run(d, "BOOLEAN \"Block 1\" UNION \"Sphere 2\"").ok);
    REQUIRE(run(d, "CAM SETUP \"Block 1\" STOCK 2").ok);
    REQUIRE(run(d, "CAM TOOL 1 FLAT 6 LENGTH 30").ok);
    REQUIRE(run(d, "CAM TOOL 3 BALL 6 LENGTH 30").ok);
    REQUIRE(run(d, "CAM ROUGH TOOL 1 LEAVE 0.3 BOTTOM 0").ok);
    REQUIRE(run(d, "CAM PARALLEL TOOL 3 SCALLOP 0.01 SLOPE 0 60").ok);
    REQUIRE(run(d, "CAM WATERLINE TOOL 3 STEEP STEPDOWN 0.25").ok);
    CamSimulation sim;
    REQUIRE(simulateJob(d.state.cam, d.scene.bodies()[0], sim));
    CHECK(sim.gouges == 0);
    CHECK(sim.rapidCrashes == 0);
    CHECK(sim.meanLeft < 0.1);
    // The roughing alone leaves at least its 0.3 mm on the dome.
    CamJob rough = d.state.cam;
    rough.ops.resize(1);
    CamSimulation rs;
    REQUIRE(simulateJob(rough, d.scene.bodies()[0], rs));
    CHECK(rs.gouges == 0);
    CHECK(rs.meanLeft > 0.3);
}

TEST_CASE("CAM bore: helical interpolation opens a hole wider than the end mill") {
    Document d;
    REQUIRE(run(d, "BOX 50 50 12 @0,0,0").ok);
    REQUIRE(run(d, "HOLE \"Block 1\" AT 0,0,12 DIAMETER 20 THROUGH").ok);
    REQUIRE(run(d, "CAM SETUP \"Block 1\" STOCK 0 TOP 0").ok);
    REQUIRE(run(d, "CAM TOOL 1 FLAT 8").ok);
    const auto r = run(d, "CAM BORE");
    REQUIRE(r.ok);
    const CamOperation& op = d.state.cam.ops.back();
    double rmax = 0, zmin = 1e9;
    for (const CamMove& m : op.moves)
        if (m.kind == CamMove::Feed) rmax = std::max(rmax, std::hypot(static_cast<double>(m.to.x), static_cast<double>(m.to.y))), zmin = std::min(zmin, static_cast<double>(m.to.z));
    CHECK(rmax > 10.0 - 4.0 - 0.06); // the tool's edge on the hole's wall (inside its facets)
    CHECK(rmax <= 10.0 - 4.0 + 1e-6);
    CHECK(zmin < 0.0);                  // through
    CamSimulation sim;
    REQUIRE(simulateJob(d.state.cam, d.scene.bodies()[0], sim));
    CHECK(sim.gouges == 0);
    CHECK_NEAR(sim.removed, 3.14159265 * 100 * 12, 3.14159265 * 100 * 12 * 0.03);
}

TEST_CASE("CAM job saves with the project, and stale paths are remade after the part moves") {
    Document d;
    bracket(d);
    REQUIRE(run(d, "CAM SETUP \"Block 1\" STOCK 3 MATERIAL steel ORIGIN CENTRE POST haas").ok);
    REQUIRE(run(d, "CAM TOOL 1 FLAT 6 FLUTES 4 NAME \"Rougher\"").ok);
    REQUIRE(run(d, "CAM ROUGH STEPOVER 30% LEAVE 0.2 NAME \"Rough it\"").ok);
    const std::string path = tempPath("sicdm_cam_test.sicdm");
    REQUIRE(run(d, "SAVE " + path).ok);
    Document e;
    REQUIRE(run(e, "OPEN " + path).ok);
    const CamJob& j = e.state.cam;
    CHECK(j.setup.body == "Block 1");
    CHECK(j.setup.material == "steel");
    CHECK(j.setup.post == "haas");
    CHECK(j.setup.origin == CamSetup::Origin::StockTopCentre);
    REQUIRE(j.tools.size() == 1);
    CHECK(j.tools[0].flutes == 4);
    CHECK(j.tools[0].name == "Rougher");
    REQUIRE(j.ops.size() == 1);
    CHECK(j.ops[0].name == "Rough it");
    CHECK_NEAR(j.ops[0].stepover, 1.8, 1e-9);
    CHECK_NEAR(j.ops[0].leave, 0.2, 1e-9);
    CHECK(!j.ops[0].generated);
    REQUIRE(run(e, "CAM GENERATE").ok);
    CHECK(e.state.cam.ops[0].generated);
    std::filesystem::remove(path);
    // Moving the part makes its paths stale: the next command remakes them where it is.
    auto lowestX = [&] {
        float x = 1e9f;
        for (const CamMove& m : d.state.cam.ops[0].moves)
            if (m.kind != CamMove::Rapid) x = std::min(x, m.to.x);
        return x;
    };
    const float x0 = lowestX();
    REQUIRE(run(d, "MOVE 10,0,0 \"Block 1\"").ok);
    REQUIRE(run(d, "CAM SETUP \"Block 1\" STOCK 3").ok);
    REQUIRE(run(d, "CAM GENERATE").ok);
    CHECK_NEAR(lowestX(), x0 + 10.0f, 0.01);
}

TEST_CASE("CAM commands refuse what they can't do") {
    Document d;
    REQUIRE(run(d, "BOX 20 20 10 @0,0,0").ok);
    CHECK(!run(d, "CAM ROUGH").ok);                  // no setup
    CHECK(!run(d, "CAM SETUP \"Nothing\"").ok);
    REQUIRE(run(d, "CAM SETUP \"Block 1\"").ok);
    CHECK(!run(d, "CAM ROUGH").ok);                  // no tool
    CHECK(!run(d, "CAM TOOL 1 SPOON 6").ok);
    CHECK(!run(d, "CAM TOOL 1 FLAT -2").ok);
    REQUIRE(run(d, "CAM TOOL 1 FLAT 6").ok);
    CHECK(!run(d, "CAM DRILL").ok);                  // no drill
    CHECK(!run(d, "CAM ROUGH TOOL 9").ok);
    CHECK(!run(d, "CAM ROUGH WOBBLE").ok);
    CHECK(!run(d, "CAM SETUP \"Block 1\" MATERIAL cheese").ok);
    CHECK(!run(d, "CAM POST").ok);
    CHECK(!run(d, "CAM DELETE 4").ok);
    REQUIRE(run(d, "CAM ROUGH").ok);
    CHECK(has(text(run(d, "CAM LIST")), "1. Rough"));
    REQUIRE(run(d, "CAM DELETE 1").ok);
    CHECK(d.state.cam.ops.empty());
    REQUIRE(run(d, "CAM CLEAR").ok);
    CHECK(d.state.cam.empty());
}

TEST_CASE("CAM simulation: wide stepover links don't gouge edges; tiny and huge tools in one job stay quick and exact") {
    Document d;
    REQUIRE(run(d, "BOX 10 10 10 @0,0,0").ok);
    REQUIRE(run(d, "CAM SETUP \"Block 1\" STOCK 0 TOP 0").ok);
    REQUIRE(run(d, "CAM TOOL 1 FLAT 30").ok);
    REQUIRE(run(d, "CAM TOOL 2 FLAT 0.1").ok);
    REQUIRE(run(d, "CAM TOOL 3 BALL 2").ok);
    REQUIRE(run(d, "CAM CONTOUR TOOL 1").ok);
    REQUIRE(run(d, "CAM CONTOUR TOOL 2 STEPDOWN 5").ok);
    REQUIRE(run(d, "CAM PARALLEL TOOL 3 STEPOVER 3").ok); // passes 3 mm apart, linked over the block's edges
    const auto t0 = std::chrono::steady_clock::now();
    CamSimulation sim;
    REQUIRE(simulateJob(d.state.cam, d.scene.bodies()[0], sim));
    CHECK(std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < 20.0);
    CHECK(sim.gouges == 0);
    CHECK(sim.rapidCrashes == 0);
}

namespace {

double feedLength(const CamOperation& op) {
    double len = 0;
    Vec3 at = op.moves.empty() ? Vec3{} : op.moves[0].to;
    for (const CamMove& m : op.moves) {
        if (m.kind != CamMove::Rapid) len += length(m.to - at);
        at = m.to;
    }
    return len;
}

void pocketPart(Document& d) {
    REQUIRE(run(d, "BOX 80 60 20 @0,0,0").ok);
    REQUIRE(run(d, "POCKET \"Block 1\" AT 0,0,20 BOX WIDTH 40 LENGTH 50 DEPTH 12").ok);
    REQUIRE(run(d, "CAM SETUP \"Block 1\" STOCK 0 TOP 0 MATERIAL aluminium").ok);
}

} // namespace

TEST_CASE("CAM rest machining cuts only what the bigger tool left") {
    Document d;
    pocketPart(d);
    REQUIRE(run(d, "CAM TOOL 1 FLAT 12 LENGTH 30").ok);
    REQUIRE(run(d, "CAM TOOL 2 FLAT 4 LENGTH 16").ok);
    CHECK(!run(d, "CAM POCKET TOOL 2 REST").ok); // nothing before it
    REQUIRE(run(d, "CAM POCKET TOOL 1 LEAVE 0.2").ok);
    CamSimulation before;
    REQUIRE(simulateJob(d.state.cam, d.scene.bodies()[0], before));
    auto r = run(d, "CAM POCKET TOOL 2 REST LEAVE 0.2");
    REQUIRE(r.ok);
    CHECK(has(text(r), "rest"));
    // The same pocket with the small tool from scratch, for comparison.
    Document e;
    pocketPart(e);
    REQUIRE(run(e, "CAM TOOL 2 FLAT 4 LENGTH 16").ok);
    REQUIRE(run(e, "CAM POCKET TOOL 2 LEAVE 0.2").ok);
    const double rest = feedLength(d.state.cam.ops[1]), full = feedLength(e.state.cam.ops[0]);
    CHECK(rest > 0);
    CHECK(rest < 0.25 * full);
    // Every level's cuts stay in the corners, where the 12 mm tool couldn't reach.
    for (const CamMove& m : d.state.cam.ops[1].moves)
        if (m.kind == CamMove::Feed && m.to.z < 19.9f) CHECK(std::fabs(m.to.x) > 12.0f && std::fabs(m.to.y) > 17.0f);
    CamSimulation after;
    REQUIRE(simulateJob(d.state.cam, d.scene.bodies()[0], after));
    CHECK(after.gouges == 0);
    CHECK(after.removed > before.removed + 50.0); // the corners: about 4 x (6^2 - 2^2)(1 - pi/4) x 12
}

TEST_CASE("CAM adaptive clearing: one deep level, light engagement, the same pocket, no gouges") {
    Document d;
    pocketPart(d);
    REQUIRE(run(d, "CAM TOOL 1 FLAT 10 LENGTH 30").ok);
    auto r = run(d, "CAM POCKET TOOL 1 ADAPTIVE LEAVE 0.2");
    REQUIRE(r.ok);
    CHECK(has(text(r), "adaptive"));
    const CamOperation& op = d.state.cam.ops[0];
    // 2 diameters a level: the 11.8 mm deep pocket is one level.
    Vec3 prev = op.moves[0].to;
    for (const CamMove& m : op.moves) {
        if (m.kind == CamMove::Feed && std::fabs(m.to.z - prev.z) < 1e-6f && m.to.z < 19.0f) CHECK_NEAR(m.to.z, 8.2, 1e-4); // ramps aside
        prev = m.to;
    }
    // Engagement along the path, measured on a fine grid of the pocket's material.
    const double R = 5, c = 0.1, X0 = -25;
    const int N = 500;
    std::vector<char> mat(static_cast<std::size_t>(N * N));
    for (int j = 0; j < N; ++j)
        for (int i = 0; i < N; ++i) mat[static_cast<std::size_t>(j * N + i)] = std::fabs(X0 + i * c) < 19.8 && std::fabs(X0 + j * c) < 24.8;
    auto clearAt = [&](double x, double y) {
        for (int j = static_cast<int>((y - R - X0) / c); j <= static_cast<int>((y + R - X0) / c) + 1; ++j)
            for (int i = static_cast<int>((x - R - X0) / c); i <= static_cast<int>((x + R - X0) / c) + 1; ++i)
                if (i >= 0 && j >= 0 && i < N && j < N && std::hypot(X0 + i * c - x, X0 + j * c - y) <= R) mat[static_cast<std::size_t>(j * N + i)] = 0;
    };
    std::size_t samples = 0, heavy = 0;
    Vec3 at = op.moves[0].to;
    for (const CamMove& m : op.moves) {
        const double L = std::hypot(m.to.x - at.x, m.to.y - at.y);
        const int n = std::max(1, static_cast<int>(L / 0.05));
        for (int k = 1; k <= n; ++k) {
            const double x = at.x + (m.to.x - at.x) * static_cast<double>(k) / n, y = at.y + (m.to.y - at.y) * static_cast<double>(k) / n;
            if (m.to.z < 8.3f && at.z < 8.3f && L > 1e-9) {
                int deg = 0;
                for (int a = 0; a < 360; ++a) {
                    const double ux = std::cos(a * 3.14159265358979 / 180), uy = std::sin(a * 3.14159265358979 / 180);
                    if (ux * (m.to.x - at.x) + uy * (m.to.y - at.y) < 0) continue;
                    const int i = static_cast<int>(std::lround((x + ux * R - X0) / c)), j = static_cast<int>(std::lround((y + uy * R - X0) / c));
                    if (i >= 0 && j >= 0 && i < N && j < N && mat[static_cast<std::size_t>(j * N + i)]) ++deg;
                }
                ++samples;
                if (deg > 90) ++heavy;
            }
            if (m.to.z < 8.3f || at.z < 8.3f) clearAt(x, y);
        }
        at = m.to;
    }
    CHECK(samples > 10000);
    CHECK(static_cast<double>(heavy) < 0.02 * static_cast<double>(samples)); // a slot is 180 degrees; offset rings at 40% reach 100+
    CamSimulation sim;
    REQUIRE(simulateJob(d.state.cam, d.scene.bodies()[0], sim));
    CHECK(sim.gouges == 0);
    CHECK(sim.rapidCrashes == 0);
    CHECK_NEAR(sim.removed, 40.0 * 50 * 12 - 4 * 25 * (1 - kPi / 4) * 12 - 0.2 * (40 * 50 + 180 * 12), 400.0);
}

TEST_CASE("CAM simulation finds the holder hitting the stock, and the stickout that clears it; saved with the job") {
    Document d;
    pocketPart(d);
    REQUIRE(run(d, "CAM TOOL 1 FLAT 10 LENGTH 8 STICKOUT 9 HOLDER 30").ok);
    REQUIRE(run(d, "CAM POCKET TOOL 1 ADAPTIVE").ok);
    REQUIRE(run(d, "CAM TOOL 2 FLAT 4").ok);
    REQUIRE(run(d, "CAM POCKET TOOL 2 REST").ok);
    auto r = run(d, "CAM SIMULATE");
    REQUIRE(r.ok);
    CHECK(has(text(r), "HOLDER"));
    CHECK(has(text(r), "stickout of 14 mm")); // 12 deep, plus a millimetre, rounded up
    REQUIRE(run(d, "CAM TOOL 1 FLAT 10 LENGTH 15 STICKOUT 14 HOLDER 30").ok);
    r = run(d, "CAM SIMULATE");
    CHECK(has(text(r), "no holder collisions"));
    const std::string path = tempPath("sicdm_cam_holder.sicdm");
    REQUIRE(run(d, "SAVE " + path).ok);
    Document e;
    REQUIRE(run(e, "OPEN " + path).ok);
    REQUIRE(e.state.cam.tools.size() == 2);
    CHECK_NEAR(e.state.cam.tools[0].stickout, 14.0, 1e-9);
    CHECK_NEAR(e.state.cam.tools[0].holderDiameter, 30.0, 1e-9);
    REQUIRE(e.state.cam.ops.size() == 2);
    CHECK(e.state.cam.ops[0].adaptive);
    CHECK(e.state.cam.ops[1].rest);
    std::filesystem::remove(path);
}

TEST_CASE("CAM 3+2: a side pocket and a side hole, cut with the table turned, land on their faces") {
    Document d;
    REQUIRE(run(d, "BOX 60 40 30 @0,0,0").ok);
    REQUIRE(run(d, "POCKET \"Block 1\" AT 0,-20,15 DIR 0,1,0 ROUND WIDTH 16 DEPTH 8").ok);
    REQUIRE(run(d, "HOLE \"Block 1\" AT 30,0,15 DIR -1,0,0 DIAMETER 6 DEPTH 10").ok);
    REQUIRE(run(d, "CAM SETUP \"Block 1\" STOCK 0 TOP 0 MATERIAL aluminium").ok);
    REQUIRE(run(d, "CAM TOOL 1 FLAT 6 LENGTH 20").ok);
    REQUIRE(run(d, "CAM TOOL 2 DRILL 6").ok);
    CHECK(!run(d, "CAM POCKET TOOL 1").ok); // from the top there is no pocket
    auto r = run(d, "CAM POCKET TOOL 1 A -90");
    REQUIRE(r.ok);
    CHECK(has(text(r), "table at A-90 C0"));
    REQUIRE(run(d, "CAM DRILL TOOL 2 INDEX -90 -90").ok);
    const CamJob& job = d.state.cam;
    CHECK(job.multiAxis());
    // The pocket's cuts, back on the part: inside the round pocket on the -Y face, down to its floor.
    double ymin = 1e9, ymax = -1e9, rmax = 0;
    for (const CamMove& m : job.ops[0].moves) {
        CHECK_NEAR(m.a, -90.0, 1e-6);
        if (m.kind == CamMove::Rapid) continue;
        const Vec3 p = camToPart(job, m.to, m.a, m.c);
        ymin = std::min(ymin, static_cast<double>(p.y)), ymax = std::max(ymax, static_cast<double>(p.y));
        rmax = std::max(rmax, std::hypot(p.x, p.z - 15.0));
    }
    CHECK_NEAR(ymax, -12.0, 1e-3); // the floor, 8 deep
    CHECK(ymin < -20.0);           // from outside the face
    CHECK(rmax < 5.0 + 1e-3);      // the tool's centre stays a radius inside the 16 mm pocket
    const Vec3 axis = camToolAxis(-90, 0);
    CHECK_NEAR(axis.y, -1.0, 1e-6); // the tool comes in from -Y
    const Vec3 drillAxis = camToolAxis(-90, -90);
    CHECK_NEAR(drillAxis.x, 1.0, 1e-6); // and the drill from +X
    REQUIRE(job.ops[1].holes.size() == 1);
    // The voxel simulation: both features cut, nothing gouged.
    CamSimulation sim;
    REQUIRE(simulateJob(d.state.cam, d.scene.bodies()[0], sim));
    CHECK(sim.voxels);
    CHECK(sim.gouges == 0);
    CHECK(sim.rapidCrashes == 0);
    CHECK_NEAR(sim.removed, kPi * 64 * 8 + kPi * 9 * 10 + kPi * 9 * 3 / std::tan(59.0 * kPi / 180) / 3, 250.0);
    // The post turns the table at clearance before each operation; GRBL refuses.
    const std::string g = postToString(d.state.cam, d.scene.bodies()[0], "fanuc");
    CHECK(has(g, "G00 A-90. C0."));
    CHECK(has(g, "G00 A-90. C-90."));
    CHECK(has(g, "ROTARY CENTRE"));
    std::string e;
    CHECK(postToString(d.state.cam, d.scene.bodies()[0], "grbl", nullptr, &e).empty());
    CHECK(has(e, "3 axes"));
}

TEST_CASE("CAM rotary: 4-axis wrapping turns a bar under the tool, roughing in levels, finishing on the surface") {
    Document d;
    REQUIRE(run(d, "CYLINDER 15 60 @0,0,0").ok);
    REQUIRE(run(d, "ROTATE \"Cylinder 1\" 0,90,0").ok);
    REQUIRE(run(d, "POCKET \"Cylinder 1\" AT 0,0,15 DIR 0,0,-1 BOX WIDTH 20 LENGTH 30 DEPTH 3").ok);
    REQUIRE(run(d, "CAM SETUP \"Cylinder 1\" STOCK 2 MATERIAL aluminium").ok);
    REQUIRE(run(d, "CAM TOOL 1 BALL 6 LENGTH 30").ok);
    auto r = run(d, "CAM ROTARY TOOL 1 STEPOVER 1 STEPDOWN 4");
    REQUIRE(r.ok);
    CHECK(has(text(r), "helical"));
    const CamJob& job = d.state.cam;
    const Vec3 c = job.rotaryCentre();
    // Every move sits over the axis (Y at the centre); A turns many times; the finish is at
    // the part's radius plus the ball's (the ball's centre is a radius above its tip).
    double maxA = 0;
    std::size_t onBar = 0, feeds = 0;
    for (const CamMove& m : job.ops[0].moves) {
        CHECK_NEAR(m.to.y, c.y, 1e-4);
        maxA = std::max(maxA, static_cast<double>(m.a));
        if (m.kind == CamMove::Feed) {
            ++feeds;
            if (std::fabs(m.to.z - c.z - 15.0) < 0.05) ++onBar;
        }
    }
    CHECK(maxA > 360.0 * 50);
    CHECK(onBar > feeds / 6); // two roughing levels, then the finish
    CamSimulation sim;
    REQUIRE(simulateJob(d.state.cam, d.scene.bodies()[0], sim));
    CHECK(sim.gouges == 0);
    CHECK(sim.rapidCrashes == 0);
    // Stock left on the turned length: almost all within a voxel of the round.
    std::size_t near = 0, far = 0;
    for (int k = 0; k < sim.vz; ++k)
        for (int j = 0; j < sim.ny; ++j)
            for (int i = 0; i < sim.nx; ++i) {
                if (sim.voxel[(static_cast<std::size_t>(k) * static_cast<std::size_t>(sim.ny) + static_cast<std::size_t>(j)) * static_cast<std::size_t>(sim.nx) +
                              static_cast<std::size_t>(i)] != 1)
                    continue;
                const double x = sim.voxelMin.x + (i + 0.5) * sim.cell, y = sim.voxelMin.y + (j + 0.5) * sim.cell, z = sim.voxelMin.z + (k + 0.5) * sim.cell;
                if (std::fabs(x - c.x) > 26 || (std::fabs(x - c.x) < 17 && std::fabs(y - c.y) < 12 && z > c.z)) continue; // the ends and the flat
                (std::hypot(y - c.y, z - c.z) - 15.0 < sim.cell ? near : far)++;
            }
    CHECK(near < 2000); // a skin of voxels whose centres sit just off the faceted round
    CHECK(far < 10);
    // Posted with A words and inverse-time feed.
    const std::string g = postToString(d.state.cam, d.scene.bodies()[0], "linuxcnc");
    CHECK(has(g, "G93"));
    CHECK(has(g, "G94"));
    CHECK(has(g, " A360"));
    // Along the axis instead, and the job saved and opened again.
    REQUIRE(run(d, "CAM ROTARY TOOL 1 ALONG STEPOVER 1.5").ok);
    CHECK(d.state.cam.ops[1].along);
    const std::string path = tempPath("sicdm_cam_rotary.sicdm");
    REQUIRE(run(d, "SAVE " + path).ok);
    Document e;
    REQUIRE(run(e, "OPEN " + path).ok);
    REQUIRE(e.state.cam.ops.size() == 2);
    CHECK(e.state.cam.ops[0].kind == CamOpKind::Rotary);
    CHECK(e.state.cam.ops[1].along);
    std::filesystem::remove(path);
}

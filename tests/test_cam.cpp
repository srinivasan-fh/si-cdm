#include "TestFramework.hpp"

#include "core/Cam.hpp"
#include "core/CamEdit.hpp"
#include "core/CamLibrary.hpp"
#include "core/CamVerify.hpp"
#include "core/Json.hpp"
#include "core/Mcp.hpp"
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

TEST_CASE("CAM 5-axis keeps clear of the stock roughing couldn't reach (under an overhang)") {
    // A ball sitting on a plate: roughing from above leaves stock under the ball, round its
    // base. The 5-axis finish simulates the roughing and keeps its shank and holder out of
    // that stock, and moves in and out beside it at feed.
    Document d;
    for (const char* l : {"BOX 40 40 8 @0,0,0", "SPHERE 12 @0,0,8", "BOOLEAN \"Block 1\" UNION \"Sphere 2\"", "CAM SETUP \"Block 1\" STOCK 1 MATERIAL aluminium",
                          "CAM MACHINE PRESET trunnion", "CAM TOOL 1 FLAT 6 LENGTH 30", "CAM TOOL 2 BALL 4 STICKOUT 40", "CAM ROUGH TOOL 1 LEAVE 0.3",
                          "CAM 5AXIS TOOL 2 SCALLOP 0.1"})
        REQUIRE(run(d, l).ok);
    CamSimulation sim;
    REQUIRE(simulateJob(d.state.cam, d.scene.bodies()[0], sim));
    CHECK(sim.gouges == 0);
    CHECK(sim.rapidCrashes == 0); // 19 when only the part was checked
    CHECK(sim.shankHits == 0);    // 119
    CHECK(sim.meanLeft < 0.8);
}

TEST_CASE("CAM 5-axis: a ball end mill leans along the cut over a dome, turning the table as it goes") {
    Document d;
    REQUIRE(run(d, "BOX 40 40 8 @0,0,0").ok);
    REQUIRE(run(d, "SPHERE 15 @0,0,-7").ok); // centred on the plate's top
    REQUIRE(run(d, "BOOLEAN \"Block 1\" UNION \"Sphere 2\"").ok);
    REQUIRE(run(d, "CAM SETUP \"Block 1\" STOCK 1 MATERIAL aluminium").ok);
    REQUIRE(run(d, "CAM TOOL 1 FLAT 6 LENGTH 30").ok);
    REQUIRE(run(d, "CAM TOOL 2 BALL 4 STICKOUT 40").ok);
    REQUIRE(run(d, "CAM ROUGH TOOL 1 LEAVE 0.3").ok);
    // A 3-axis machine can't, a flat end mill can't, and the table isn't indexed by hand.
    REQUIRE(run(d, "CAM MACHINE PRESET vmc").ok);
    auto r = run(d, "CAM 5AXIS TOOL 2");
    CHECK(!r.ok);
    CHECK(has(text(r), "A and C"));
    REQUIRE(run(d, "CAM MACHINE PRESET trunnion").ok);
    CHECK(!run(d, "CAM 5AXIS TOOL 1").ok);
    CHECK(!run(d, "CAM 5AXIS TOOL 2 A 30").ok);
    CHECK(!run(d, "CAM 5AXIS TOOL 2 MAXTILT 150").ok);
    r = run(d, "CAM 5AXIS TOOL 2 SCALLOP 0.05");
    REQUIRE(r.ok);
    CHECK(has(text(r), "lead 10 deg"));
    const CamJob& job = d.state.cam;
    REQUIRE(job.ops.size() == 2);
    const CamOperation& op = job.ops[1];
    CHECK(op.kind == CamOpKind::Surface5);
    CHECK(job.multiAxis());
    // The ball's centre never inside the dome grown by its radius; on the dome the axis
    // leans from the surface normal by about the lead; on its flanks the table tilts far.
    const Vec3 centre{0, 0, 8};
    double nearest = 1e9, maxA = 0, leanSum = 0, cLo = 1e9, cHi = -1e9;
    std::size_t onDome = 0;
    for (const CamMove& m : op.moves) {
        if (m.kind == CamMove::Rapid) continue;
        const Vec3 axis = camToolAxis(m.a, m.c);
        const Vec3 ball = camToPart(job, m.to, m.a, m.c) + axis * 2.0f;
        const double rr = length(ball - centre);
        nearest = std::min(nearest, rr);
        maxA = std::max(maxA, std::fabs(static_cast<double>(m.a)));
        cLo = std::min(cLo, static_cast<double>(m.c)), cHi = std::max(cHi, static_cast<double>(m.c));
        if (std::fabs(rr - 17.0) < 0.1 && ball.z > 12.0 && std::fabs(m.a) < 45.0) {
            const Vec3 n = (ball - centre) * static_cast<float>(1.0 / rr);
            leanSum += std::acos(std::clamp(static_cast<double>(dot(n, axis)), -1.0, 1.0)) * 180.0 / kPi;
            ++onDome;
        }
    }
    CHECK(nearest > 17.0 - 0.1); // the sphere's facets sit up to 0.07 inside it
    CHECK(maxA > 40.0);
    CHECK(maxA < 60.0 + 1e-3);
    CHECK(cHi - cLo > 180.0);
    REQUIRE(onDome > 500);
    CHECK_NEAR(leanSum / static_cast<double>(onDome), 10.0, 2.5);
    // Cut in the voxels: nothing gouged, no rapid through the stock.
    CamSimulation sim;
    REQUIRE(simulateJob(d.state.cam, d.scene.bodies()[0], sim));
    CHECK(sim.voxels);
    CHECK(sim.gouges == 0);
    CHECK(sim.rapidCrashes == 0);
    // Posted with A and C at inverse-time feed, and read back on the toolpath.
    const std::string g = postToString(d.state.cam, d.scene.bodies()[0], "linuxcnc");
    CHECK(has(g, "G93"));
    CHECK(has(g, "G94"));
    CHECK(has(g, " C"));
    const CamPostCheck pc = checkPost(d.state.cam, "linuxcnc", g);
    CHECK(pc.errors == 0);
    CHECK(pc.deviation <= pc.allowed);
    // Less lean, and the options saved with the job.
    REQUIRE(run(d, "CAM DELETE 2").ok);
    REQUIRE(run(d, "CAM 5AXIS TOOL 2 SCALLOP 0.1 MAXTILT 25 LEADANGLE 0 SIDETILT 5").ok);
    for (const CamMove& m : d.state.cam.ops[1].moves) CHECK(std::fabs(m.a) <= 25.0 + 1e-3);
    const std::string path = tempPath("sicdm_cam_5axis.sicdm");
    REQUIRE(run(d, "SAVE " + path).ok);
    Document e;
    REQUIRE(run(e, "OPEN " + path).ok);
    REQUIRE(e.state.cam.ops.size() == 2);
    const CamOperation& o = e.state.cam.ops[1];
    CHECK(o.kind == CamOpKind::Surface5);
    CHECK(o.maxTilt == 25 && o.leadAngle == 0 && o.sideTilt == 5);
    std::filesystem::remove(path);
}

TEST_CASE("CAM 5-axis on a head-table machine: B tilts the tool, C turns the part, posted with tool centre point") {
    Document d;
    for (const char* l : {"BOX 40 40 8 @0,0,0", "SPHERE 15 @0,0,-7", "BOOLEAN \"Block 1\" UNION \"Sphere 2\"", "CAM SETUP \"Block 1\" STOCK 1 MATERIAL aluminium",
                          "CAM MACHINE PRESET head-table", "CAM TOOL 1 FLAT 6 LENGTH 30", "CAM TOOL 2 BALL 4 STICKOUT 40", "CAM ROUGH TOOL 1 LEAVE 0.3"})
        REQUIRE(run(d, l).ok);
    CHECK(d.state.cam.setup.machine.headTilts());
    // No A to turn the part on.
    CHECK(!run(d, "CAM ROTARY TOOL 1").ok);
    REQUIRE(run(d, "CAM 5AXIS TOOL 2 SCALLOP 0.1").ok);
    const CamJob& job = d.state.cam;
    const CamOperation& op = job.ops[1];
    // The table never tilts; the head does, and the ball stays off the dome.
    const Vec3 centre{0, 0, 8};
    double nearest = 1e9, maxB = 0, leanSum = 0;
    std::size_t onDome = 0;
    for (const CamMove& m : op.moves) {
        CHECK(m.a == 0.0f);
        if (m.kind == CamMove::Rapid) continue;
        const Vec3 axis = camToolAxis(m.a, m.c, m.b);
        const Vec3 ball = camToPart(job, m.to, m.a, m.c) + axis * 2.0f;
        const double rr = length(ball - centre);
        nearest = std::min(nearest, rr);
        maxB = std::max(maxB, std::fabs(static_cast<double>(m.b)));
        if (std::fabs(rr - 17.0) < 0.1 && ball.z > 12.0 && std::fabs(m.b) < 45.0) {
            const Vec3 n = (ball - centre) * static_cast<float>(1.0 / rr);
            leanSum += std::acos(std::clamp(static_cast<double>(dot(n, axis)), -1.0, 1.0)) * 180.0 / kPi;
            ++onDome;
        }
    }
    CHECK(nearest > 17.0 - 0.1);
    CHECK(maxB > 40.0);
    CHECK(maxB < 60.0 + 1e-3);
    REQUIRE(onDome > 500);
    CHECK_NEAR(leanSum / static_cast<double>(onDome), 10.0, 2.5);
    CamSimulation sim;
    REQUIRE(simulateJob(d.state.cam, d.scene.bodies()[0], sim));
    CHECK(sim.gouges == 0);
    CHECK(sim.rapidCrashes == 0);
    CHECK(sim.shankHits == 0);
    // Fanuc: G43.4 on for the 5-axis moves, X Y Z on the part, B and C; G43 again after.
    std::string g = postToString(d.state.cam, d.scene.bodies()[0], "fanuc");
    CHECK(has(g, "G43.4 H02"));
    CHECK(has(g, " B"));
    bool aWord = false;
    for (std::size_t q = g.find(" A"); q != std::string::npos; q = g.find(" A", q + 1))
        aWord = aWord || (q + 2 < g.size() && (std::isdigit(static_cast<unsigned char>(g[q + 2])) || g[q + 2] == '-'));
    CHECK(!aWord);
    CHECK(has(g, "G00 B0.\nG43 H02"));
    CamPostCheck pc = checkPost(d.state.cam, "fanuc", g);
    for (const GcodeIssue& i : pc.issues)
        if (i.error) std::printf("  line %d: %s\n", i.line, i.text.c_str());
    CHECK(pc.errors == 0);
    CHECK(pc.deviation <= pc.allowed);
    CHECK(pc.plot.usesB);
    // Haas: G234. LinuxCNC has no standard tool centre point mode: refused.
    g = postToString(d.state.cam, d.scene.bodies()[0], "haas");
    CHECK(has(g, "G234 H02"));
    pc = checkPost(d.state.cam, "haas", g);
    CHECK(pc.errors == 0);
    std::string err;
    CHECK(postToString(d.state.cam, d.scene.bodies()[0], "linuxcnc", nullptr, &err).empty());
    CHECK(has(err, "tool centre point"));
    // A head that tilts less: the axis is held within its travel.
    REQUIRE(run(d, "CAM MACHINE B -30 30").ok);
    REQUIRE(run(d, "CAM GENERATE").ok);
    for (const CamMove& m : d.state.cam.ops[1].moves) CHECK(std::fabs(m.b) <= 30.0 + 1e-3);
    // Saved with the job.
    const std::string path = tempPath("sicdm_cam_head.sicdm");
    REQUIRE(run(d, "SAVE " + path).ok);
    Document e;
    REQUIRE(run(e, "OPEN " + path).ok);
    CHECK(e.state.cam.setup.machine.headTilts());
    CHECK(e.state.cam.setup.machine.bMax == 30);
    std::filesystem::remove(path);
}

TEST_CASE("CAM 5-axis on a head-head machine: the part stays put, B tilts and C swivels the head") {
    Document d;
    for (const char* l : {"BOX 40 40 8 @0,0,0", "SPHERE 15 @0,0,-7", "BOOLEAN \"Block 1\" UNION \"Sphere 2\"", "CAM SETUP \"Block 1\" STOCK 1 MATERIAL aluminium",
                          "CAM MACHINE PRESET head-head", "CAM TOOL 1 FLAT 6 LENGTH 30", "CAM TOOL 2 BALL 4 STICKOUT 40", "CAM ROUGH TOOL 1 LEAVE 0.3"})
        REQUIRE(run(d, l).ok);
    CHECK(d.state.cam.setup.machine.headHead());
    // No table to turn the part on.
    CHECK(!run(d, "CAM ROTARY TOOL 1").ok);
    auto r = run(d, "CAM PARALLEL TOOL 2 INDEX 0 90");
    CHECK(!r.ok);
    CHECK(has(text(r), "swivels the head"));
    REQUIRE(run(d, "CAM 5AXIS TOOL 2 SCALLOP 0.1").ok);
    const CamJob& job = d.state.cam;
    const CamOperation& op = job.ops[1];
    const Vec3 centre{0, 0, 8};
    double nearest = 1e9, maxB = 0, leanSum = 0, cLo = 1e9, cHi = -1e9;
    std::size_t onDome = 0;
    for (const CamMove& m : op.moves) {
        CHECK(m.a == 0.0f);
        CHECK(length(camToPart(job, m.to, m.a, m.c) - m.to) < 1e-5f); // the part never turns
        if (m.kind == CamMove::Rapid) continue;
        const Vec3 axis = camToolAxis(m.a, m.c, m.b, true);
        const Vec3 ball = m.to + axis * 2.0f;
        const double rr = length(ball - centre);
        nearest = std::min(nearest, rr);
        maxB = std::max(maxB, std::fabs(static_cast<double>(m.b)));
        cLo = std::min(cLo, static_cast<double>(m.c)), cHi = std::max(cHi, static_cast<double>(m.c));
        if (std::fabs(rr - 17.0) < 0.1 && ball.z > 12.0 && std::fabs(m.b) < 45.0) {
            const Vec3 n = (ball - centre) * static_cast<float>(1.0 / rr);
            leanSum += std::acos(std::clamp(static_cast<double>(dot(n, axis)), -1.0, 1.0)) * 180.0 / kPi;
            ++onDome;
        }
    }
    CHECK(nearest > 17.0 - 0.1);
    CHECK(maxB > 40.0);
    CHECK(cHi - cLo > 180.0);
    CHECK(cLo >= -360.0 - 1e-3);
    CHECK(cHi <= 360.0 + 1e-3);
    REQUIRE(onDome > 500);
    CHECK_NEAR(leanSum / static_cast<double>(onDome), 10.0, 2.5);
    CamSimulation sim;
    REQUIRE(simulateJob(d.state.cam, d.scene.bodies()[0], sim));
    CHECK(sim.gouges == 0);
    CHECK(sim.rapidCrashes == 0);
    CHECK(sim.shankHits == 0);
    // Posted in tool centre point mode and read back on the toolpath, the travel checked with
    // the head swivelled.
    const std::string g = postToString(d.state.cam, d.scene.bodies()[0], "fanuc");
    CHECK(has(g, "G43.4 H02"));
    CHECK(has(g, "B TILTS THE HEAD AND C SWIVELS IT"));
    const CamPostCheck pc = checkPost(d.state.cam, "fanuc", g);
    for (const GcodeIssue& i : pc.issues)
        if (i.error) std::printf("  line %d: %s\n", i.line, i.text.c_str());
    CHECK(pc.errors == 0);
    CHECK(pc.deviation <= pc.allowed);
    // Kinematics set by hand, and saved with the job.
    REQUIRE(run(d, "CAM MACHINE PRESET trunnion KINEMATICS head-head B -90 90").ok);
    CHECK(d.state.cam.setup.machine.headHead());
    const std::string path = tempPath("sicdm_cam_headhead.sicdm");
    REQUIRE(run(d, "SAVE " + path).ok);
    Document e;
    REQUIRE(run(e, "OPEN " + path).ok);
    CHECK(e.state.cam.setup.machine.headHead());
    CHECK(e.state.cam.setup.machine.bMax == 90);
    std::filesystem::remove(path);
}

TEST_CASE("CAM swarf: the side of a flat end mill along leaning walls, its axis on the walls' rules") {
    // A cone frustum (40 mm across at the foot, 20 at the top, 20 high: the wall leans in
    // 26.6 degrees) on a plate.
    Document d;
    for (const char* l : {"CONE 20 40 @0,0,0", "BOX 60 60 20 @0,0,0", "BOOLEAN \"Cone 1\" INTERSECT \"Block 2\"", "BOX 60 60 6 @0,0,-6",
                          "BOOLEAN \"Cone 1\" UNION \"Block 3\"", "CAM SETUP \"Cone 1\" STOCK 1 MATERIAL aluminium", "CAM TOOL 1 FLAT 8 LENGTH 30 STICKOUT 45",
                          "CAM TOOL 2 FLAT 6 LENGTH 20", "CAM TOOL 3 BALL 6", "CAM ROUGH TOOL 2 LEAVE 0.3 BOTTOM 0"})
        REQUIRE(run(d, l).ok);
    // A 3-axis machine can't, nor can a ball end mill.
    REQUIRE(run(d, "CAM MACHINE PRESET vmc").ok);
    CHECK(!run(d, "CAM SWARF TOOL 1 BOTTOM 0").ok);
    REQUIRE(run(d, "CAM MACHINE PRESET trunnion").ok);
    CHECK(!run(d, "CAM SWARF TOOL 3 BOTTOM 0").ok);
    REQUIRE(run(d, "CAM SWARF TOOL 1 BOTTOM 0").ok);
    const CamJob& job = d.state.cam;
    REQUIRE(job.ops.size() == 2);
    CHECK(job.ops[1].kind == CamOpKind::Swarf);
    // All round, the side runs the tool's radius off the wall and the axis lies along it:
    // in the plane through the cone's axis, leaning with the wall.
    const double s5 = std::sqrt(5.0);
    std::size_t cuts = 0;
    double worst = 0, offPlane = 0, offWall = 0, turned = 0;
    const std::vector<CamMove>& mv = job.ops[1].moves;
    for (std::size_t i = 0; i + 1 < mv.size(); ++i) {
        const CamMove& m = mv[i];
        if (m.kind != CamMove::Feed || mv[i + 1].kind == CamMove::Rapid) continue; // the lead out ends off the wall
        const Vec3 axis = camToolAxis(m.a, m.c), tip = camToPart(job, m.to, m.a, m.c);
        const double rr = std::hypot(tip.x, tip.y);
        ++cuts;
        const double ux = tip.x / rr, uy = tip.y / rr;
        const double ar = axis.x * ux + axis.y * uy, at = -axis.x * uy + axis.y * ux;
        worst = std::max(worst, std::fabs(((rr - 20.0) * 2.0 + tip.z) / s5 - 4.0));
        offPlane = std::max(offPlane, std::fabs(at));
        offWall = std::max(offWall, std::fabs((2.0 * ar + axis.z) / s5));
        turned = std::max(turned, std::fabs(static_cast<double>(m.c)));
    }
    CHECK(cuts > 100);
    CHECK(worst < 0.1); // the cone's facets lie inside it by their sag
    CHECK(offPlane < 0.03);
    CHECK(offWall < 0.01);
    CHECK(turned > 170); // the table turns the part round under the tool
    // Cut in the voxels after the roughing: nothing gouged, no rapid through the stock, the
    // shank and holder clear.
    CamSimulation sim;
    REQUIRE(simulateJob(d.state.cam, d.scene.bodies()[0], sim));
    CHECK(sim.gouges == 0);
    CHECK(sim.rapidCrashes == 0);
    CHECK(sim.shankHits == 0);
    std::string g = postToString(d.state.cam, d.scene.bodies()[0], "fanuc");
    CamPostCheck pc = checkPost(d.state.cam, "fanuc", g);
    CHECK(pc.errors == 0);
    CHECK(pc.deviation <= pc.allowed);
    // The same walls on a head-table machine, in tool centre point mode.
    REQUIRE(run(d, "CAM MACHINE PRESET head-table").ok);
    REQUIRE(run(d, "CAM GENERATE").ok);
    for (const CamMove& m : d.state.cam.ops[1].moves) CHECK(m.a == 0.0f);
    REQUIRE(simulateJob(d.state.cam, d.scene.bodies()[0], sim));
    CHECK(sim.gouges == 0);
    CHECK(sim.rapidCrashes == 0);
    g = postToString(d.state.cam, d.scene.bodies()[0], "haas");
    CHECK(has(g, "G234 H01"));
    pc = checkPost(d.state.cam, "haas", g);
    CHECK(pc.errors == 0);
    CHECK(pc.deviation <= pc.allowed);
    // Saved with the job.
    const std::string path = tempPath("sicdm_cam_swarf.sicdm");
    REQUIRE(run(d, "SAVE " + path).ok);
    Document e;
    REQUIRE(run(e, "OPEN " + path).ok);
    REQUIRE(e.state.cam.ops.size() == 2);
    CHECK(e.state.cam.ops[1].kind == CamOpKind::Swarf);
    std::filesystem::remove(path);
}

TEST_CASE("CAM swarf in bands: a wall longer than the flutes, or than STEPDOWN, is cut from the top down") {
    Document d;
    for (const char* l : {"CONE 20 40 @0,0,0", "BOX 60 60 20 @0,0,0", "BOOLEAN \"Cone 1\" INTERSECT \"Block 2\"", "BOX 60 60 6 @0,0,-6",
                          "BOOLEAN \"Cone 1\" UNION \"Block 3\"", "CAM SETUP \"Cone 1\" STOCK 1 MATERIAL aluminium", "CAM MACHINE PRESET trunnion",
                          "CAM TOOL 1 FLAT 8 LENGTH 30 STICKOUT 45", "CAM TOOL 2 FLAT 6 LENGTH 20", "CAM TOOL 4 FLAT 8 LENGTH 12 STICKOUT 45",
                          "CAM ROUGH TOOL 2 LEAVE 0.3 BOTTOM 0"})
        REQUIRE(run(d, l).ok);
    auto plunges = [](const CamOperation& op) {
        std::size_t n = 0;
        for (const CamMove& m : op.moves) n += m.kind == CamMove::Plunge;
        return n;
    };
    // The wall runs 22 mm along the tool: one pass with 30 mm of flutes, three with 12 (each
    // band overlapping the next, the top one past the wall's top).
    REQUIRE(run(d, "CAM SWARF TOOL 1 BOTTOM 0").ok);
    CHECK(plunges(d.state.cam.ops[1]) == 1);
    REQUIRE(run(d, "CAM DELETE 2").ok);
    REQUIRE(run(d, "CAM SWARF TOOL 4 BOTTOM 0").ok);
    const CamOperation& op = d.state.cam.ops[1];
    CHECK(plunges(op) == 3);
    // The bands step down the tool's axis, the last at the foot.
    double lowest = 1e9, highest = -1e9;
    for (const CamMove& m : op.moves)
        if (m.kind == CamMove::Feed) {
            const double z = camToPart(d.state.cam, m.to, m.a, m.c).z;
            lowest = std::min(lowest, z), highest = std::max(highest, z);
        }
    CHECK(lowest < 3.0);
    CHECK(highest > 10.0);
    CamSimulation sim;
    REQUIRE(simulateJob(d.state.cam, d.scene.bodies()[0], sim));
    CHECK(sim.gouges == 0);
    CHECK(sim.rapidCrashes == 0);
    CHECK(sim.shankHits == 0); // the shank of the bands below goes over what the band above cut
    // STEPDOWN makes the bands shorter.
    REQUIRE(run(d, "CAM DELETE 2").ok);
    REQUIRE(run(d, "CAM SWARF TOOL 1 BOTTOM 0 STEPDOWN 8").ok);
    CHECK(plunges(d.state.cam.ops[1]) == 4);
    REQUIRE(run(d, "CAM DELETE 2").ok);
    REQUIRE(run(d, "CAM SWARF TOOL 1 BOTTOM 0 STEPDOWN 5").ok);
    CHECK(plunges(d.state.cam.ops[1]) == 5);
}

TEST_CASE("CAM swarf on a twisted wall: rules corner to corner, the side touching the wall at its foot and top") {
    // A ruled loft from a square (corners at radius 21) to a smaller one (radius 11) turned
    // 8 degrees: each wall twists along its height.
    Document d;
    for (const char* l : {"POLYGON 0,0 21 4", "POLYGON 0,0 11 4 ANGLE 8", "LOFT \"Polyline 1\" \"Polyline 2\" AT 20 RULED", "BOX 50 50 6 @0,0,-6",
                          "BOOLEAN \"Loft 1\" UNION \"Block 2\"", "CAM SETUP \"Loft 1\" STOCK 1 MATERIAL aluminium", "CAM MACHINE PRESET trunnion",
                          "CAM TOOL 1 FLAT 8 LENGTH 30 STICKOUT 45", "CAM TOOL 2 FLAT 6 LENGTH 20", "CAM ROUGH TOOL 2 LEAVE 0.3 BOTTOM 0",
                          "CAM SWARF TOOL 1 BOTTOM 0"})
        REQUIRE(run(d, l).ok);
    const CamJob& job = d.state.cam;
    // The wall between the corners at 0 and 90 degrees rises from the middle of its foot to
    // the middle of its top (turned 8 degrees). The station passing the foot's middle has its
    // axis a tool's radius off both.
    const double c8 = std::cos(8.0 * 3.14159265358979323846 / 180.0), s8 = std::sin(8.0 * 3.14159265358979323846 / 180.0);
    const Vec3 B{10.5f, 10.5f, 0.0f};
    const Vec3 T{static_cast<float>(11 * (c8 - s8) / 2), static_cast<float>(11 * (s8 + c8) / 2), 20.0f};
    auto off = [](const Vec3& p, const Vec3& tip, const Vec3& axis) { return static_cast<double>(length((p - tip) - axis * dot(p - tip, axis))); };
    double bestB = 1e9, atT = 0;
    for (const CamMove& m : job.ops[1].moves) {
        if (m.kind != CamMove::Feed) continue;
        const Vec3 axis = camToolAxis(m.a, m.c), tip = camToPart(job, m.to, m.a, m.c);
        const double db = off(B, tip, axis);
        if (db < bestB) bestB = db, atT = off(T, tip, axis);
    }
    CHECK(std::fabs(bestB - 4.0) < 0.05);
    CHECK(std::fabs(atT - 4.0) < 0.1);
    CamSimulation sim;
    REQUIRE(simulateJob(d.state.cam, d.scene.bodies()[0], sim));
    CHECK(sim.rapidCrashes == 0);
    CHECK(sim.shankHits == 0);
}

TEST_CASE("CAM swarf INSIDE: a drafted pocket's wall; one that closes over the tool is refused") {
    // A conical pocket through a block, 40 mm across at the top and 20 at the bottom.
    Document d;
    for (const char* l : {"BOX 70 70 20 @0,0,0", "CONE 25 50 @0,0,0", "ROTATE \"Cone 2\" 180,0,0", "MOVE 0,0,30 \"Cone 2\"",
                          "BOOLEAN \"Block 1\" SUBTRACT \"Cone 2\"", "CAM SETUP \"Block 1\" STOCK 1 MATERIAL aluminium", "CAM MACHINE PRESET trunnion",
                          "CAM TOOL 1 FLAT 8 LENGTH 30 STICKOUT 45", "CAM TOOL 2 FLAT 6 LENGTH 25", "CAM ROUGH TOOL 2 LEAVE 0.3", "CAM SWARF TOOL 1 INSIDE"})
        REQUIRE(run(d, l).ok);
    CamSimulation sim;
    REQUIRE(simulateJob(d.state.cam, d.scene.bodies()[0], sim));
    CHECK(sim.gouges == 0);
    CHECK(sim.rapidCrashes == 0);
    CHECK(sim.shankHits == 0);
    // The same pocket the other way up: narrower at the top, the wall over the tool.
    Document u;
    for (const char* l : {"BOX 70 70 20 @0,0,0", "CONE 25 50 @0,0,-30", "BOOLEAN \"Block 1\" SUBTRACT \"Cone 2\"", "CAM SETUP \"Block 1\" STOCK 1 MATERIAL aluminium",
                          "CAM MACHINE PRESET trunnion", "CAM TOOL 1 FLAT 8 LENGTH 30 STICKOUT 45"})
        REQUIRE(run(u, l).ok);
    const auto r = run(u, "CAM SWARF TOOL 1 INSIDE");
    CHECK(!r.ok);
    CHECK(has(text(r), "overhang"));
}

namespace {

void setEnv(const char* name, const std::string& value) {
#if defined(_WIN32)
    _putenv_s(name, value.c_str());
#else
    setenv(name, value.c_str(), 1);
#endif
}

void writeText(const std::string& path, const std::string& text) {
    std::ofstream f(path, std::ios::binary);
    f << text;
}

} // namespace

TEST_CASE("CAM tool library: vendor catalogues as CSV, Fusion 360 libraries, and back") {
    // A catalogue with its own column names and order, a quoted name, SFM and an unknown type.
    const std::string csv = "EDP,Type,Dia,Flutes,LOC,Corner Radius,Material,Description,Manufacturer,SFM,Chipload\n"
                            "33006,End Mill,6,3,18,,Carbide,\"6mm 3FL, AlTiN\",Acme,1000,0.03\n"
                            "33020,Bull Nose,12,4,30,1.5,Carbide,12mm R1.5,Acme,,\n"
                            "44005,Spot Drill,8,,,,HSS,spot,Acme,,\n"
                            "55001,Thread Mill,6,3,12,,Carbide,threadmill,Acme,,\n";
    std::vector<CamTool> tools;
    std::vector<std::string> notes;
    std::string err;
    REQUIRE(toolsFromCsv(csv, tools, &err, &notes));
    REQUIRE(tools.size() == 3);
    CHECK(notes.size() == 1); // the thread mill
    CHECK(tools[0].type == CamToolType::Flat && tools[0].diameter == 6 && tools[0].flutes == 3 && tools[0].fluteLength == 18);
    CHECK(tools[0].name == "6mm 3FL, AlTiN");
    CHECK(tools[0].vendor == "Acme" && tools[0].productId == "33006");
    CHECK(std::fabs(tools[0].vc - 304.8) < 1e-9); // 1000 SFM
    CHECK(tools[0].fz == 0.03);
    CHECK(tools[1].type == CamToolType::BullNose && tools[1].cornerRadius == 1.5);
    CHECK(tools[2].type == CamToolType::Spot && tools[2].hss && tools[2].flutes == 2); // the type's defaults fill in
    CHECK(tools[0].number == 1 && tools[1].number == 2 && tools[2].number == 3);

    // Fusion 360: millimetre and inch tools, presets, holders and unknown types left out.
    const std::string fusion = R"({"data":[
      {"type":"flat end mill","unit":"millimeters","description":"8mm","vendor":"Harvey","product-id":"H-8","BMC":"carbide",
       "geometry":{"DC":8,"NOF":3,"LCF":20,"RE":0},"post-process":{"number":4},"start-values":{"presets":[{"v_c":350,"f_z":0.04}]}},
      {"type":"chamfer mill","unit":"inches","geometry":{"DC":0.25,"NOF":4,"LCF":0.3,"TA":45},"post-process":{"number":7},
       "start-values":{"presets":[{"v_c":800,"f_z":0.001}]}},
      {"type":"holder","geometry":{}},
      {"type":"slot mill","unit":"millimeters","geometry":{"DC":20}},
      {"type":"drill","unit":"millimeters","geometry":{"DC":5,"SIG":140},"post-process":{"number":9},"start-values":{"presets":[{"v_c":90,"f_n":0.12}]}}]})";
    std::vector<CamTool> fu;
    notes.clear();
    REQUIRE(toolsFromFusion(fusion, fu, &err, &notes));
    REQUIRE(fu.size() == 3);
    CHECK(notes.size() == 1); // the slot mill; the holder silently
    CHECK(fu[0].number == 4 && fu[0].diameter == 8 && fu[0].vc == 350 && fu[0].fz == 0.04 && fu[0].productId == "H-8");
    CHECK(fu[1].type == CamToolType::Chamfer && std::fabs(fu[1].diameter - 6.35) < 1e-9 && fu[1].tipAngle == 90);
    CHECK(std::fabs(fu[1].vc - 243.84) < 1e-9 && std::fabs(fu[1].fz - 0.0254) < 1e-12);
    CHECK(fu[2].type == CamToolType::Drill && fu[2].tipAngle == 140 && fu[2].fz == 0.12);

    // Every format reads back what it wrote.
    std::vector<CamTool> all = tools;
    all.insert(all.end(), fu.begin(), fu.end());
    for (ToolFileFormat f : {ToolFileFormat::Native, ToolFileFormat::Csv, ToolFileFormat::Fusion}) {
        const std::string path = (std::filesystem::temp_directory_path() / (f == ToolFileFormat::Csv ? "sicdm tools.csv" : "sicdm tools.json")).string();
        REQUIRE(saveToolFile(path, all, &err, f));
        std::vector<CamTool> back;
        REQUIRE(loadToolFile(path, back, &err));
        REQUIRE(back.size() == all.size());
        for (std::size_t i = 0; i < all.size(); ++i) {
            CHECK(back[i].type == all[i].type);
            CHECK(std::fabs(back[i].diameter - all[i].diameter) < 1e-6);
            CHECK(back[i].flutes == all[i].flutes);
            CHECK(back[i].number == all[i].number);
            CHECK(std::fabs(back[i].vc - all[i].vc) < 1e-6);
            CHECK(std::fabs(back[i].fz - all[i].fz) < 1e-9);
            CHECK(back[i].productId == all[i].productId);
        }
        std::filesystem::remove(path);
    }
    // The vendor's cutting data sets the feeds and speeds.
    const CamMaterial* alu = camMaterial("aluminium");
    REQUIRE(alu);
    CamMachine mach;
    mach.maxRpm = 30000;
    const CamFeeds f = camFeeds(fu[2], *alu, mach);
    CHECK(std::fabs(f.rpm - 5720) < 1e-9); // 90 m/min on 5 mm, to the 10 below
    CHECK(std::fabs(f.chipLoad - 0.12) < 1e-12);
}

TEST_CASE("CAM LIBRARY: import into the user's library, use in a job, store and save") {
    const std::string lib = (std::filesystem::temp_directory_path() / "sicdm test tool library.json").string();
    std::filesystem::remove(lib);
    setEnv("SICDM_TOOL_LIBRARY", lib);
    const std::string cat = (std::filesystem::temp_directory_path() / "sicdm catalogue.csv").string();
    writeText(cat, "type,diameter,flutes,product_id,vc,fz\nflat,6,3,F6,250,0.03\nball,10,2,B10,,\ndrill,6.8,2,D68,,\n");
    Document d;
    REQUIRE(run(d, "CAM LIBRARY").ok);
    REQUIRE(run(d, "CAM LIBRARY IMPORT \"" + cat + "\"").ok);
    CHECK(text(run(d, "CAM LIBRARY IMPORT \"" + cat + "\"")).find("3 already there") != std::string::npos);
    CHECK(text(run(d, "CAM LIBRARY")).find("#3") != std::string::npos);
    REQUIRE(run(d, "BOX 60 40 20").ok);
    REQUIRE(run(d, "CAM SETUP STOCK 2 MATERIAL aluminium").ok);
    REQUIRE(run(d, "CAM LIBRARY USE F6").ok);
    REQUIRE(run(d, "CAM LIBRARY USE F6").ok); // the next free number, not a replacement
    REQUIRE(run(d, "CAM LIBRARY USE #2 AS 12").ok);
    REQUIRE(d.state.cam.tools.size() == 3);
    CHECK(d.state.cam.tools[0].number == 1 && d.state.cam.tools[1].number == 2 && d.state.cam.tools[2].number == 12);
    CHECK(d.state.cam.tools[0].vc == 250);
    CHECK(!run(d, "CAM LIBRARY USE nothing").ok);
    // Operations use the library tools like any other.
    REQUIRE(run(d, "CAM FACE TOOL 1").ok);
    // Job tools into the library, once; a job's tools to a file and into another job.
    REQUIRE(run(d, "CAM TOOL 5 BULL 8 CORNER 1").ok);
    CHECK(text(run(d, "CAM LIBRARY STORE ALL")).find("Stored 1 tool") != std::string::npos);
    const std::string jobFile = (std::filesystem::temp_directory_path() / "sicdm job tools.csv").string();
    REQUIRE(run(d, "CAM LIBRARY SAVE \"" + jobFile + "\"").ok);
    Document e;
    REQUIRE(run(e, "CAM LIBRARY LOAD \"" + jobFile + "\"").ok);
    CHECK(e.state.cam.tools.size() == 4);
    REQUIRE(run(d, "CAM LIBRARY DELETE #1").ok);
    std::vector<CamTool> stored;
    REQUIRE(loadToolFile(lib, stored));
    CHECK(stored.size() == 3);
    // Saved with the project: the vendor fields too.
    const Json j = camToJson(d.state.cam);
    CamJob back;
    REQUIRE(camFromJson(j, back));
    CHECK(back.tools[0].productId == "F6" && back.tools[0].vc == 250 && back.tools[0].fz == 0.03);
    // Over MCP.
    McpServer s(d, nullptr);
    const auto r = Json::parse(s.handle(R"({"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"cam","arguments":{"operation":"library","library":"list"}}})"));
    REQUIRE(r.has_value());
    CHECK(!(*r)["result"]["isError"].asBool());
    CHECK((*r)["result"]["content"][0]["text"].asString().find("Tool library") != std::string::npos);
    std::filesystem::remove(lib);
    std::filesystem::remove(cat);
    std::filesystem::remove(jobFile);
    setEnv("SICDM_TOOL_LIBRARY", "");
}

TEST_CASE("CAM operations edited in place, moved, suppressed and duplicated (commands and MCP)") {
    Document d;
    bracket(d);
    for (const char* l : {"CAM SETUP \"Block 1\" STOCK 3 TOP 1 MATERIAL aluminium", "CAM TOOL 1 FLAT 6", "CAM TOOL 2 FLAT 10", "CAM TOOL 3 BALL 6",
                          "CAM ROUGH TOOL 2 LEAVE 0.3", "CAM PARALLEL TOOL 3"})
        REQUIRE(run(d, l).ok);
    CamJob& job = d.state.cam;
    REQUIRE(job.ops.size() == 2);
    // Edit: the options given change, the rest stay; it is made again and the finish after it
    // waits to be made again from what it now leaves.
    auto r = run(d, "CAM EDIT 1 LEAVE 0.5 STEPDOWN 2 TOOL 1 NAME \"Rough it\"");
    REQUIRE(r.ok);
    CHECK(has(text(r), "Changed 1. Rough it"));
    CHECK(job.ops[0].leave == 0.5);
    CHECK(job.ops[0].stepdown == 2);
    CHECK(job.ops[0].tool == 1);
    CHECK(job.ops[0].kind == CamOpKind::Rough);
    CHECK(job.ops[0].generated);
    CHECK(!job.ops[1].generated);
    // AUTO puts an option back; a bad edit leaves the operation as it was.
    REQUIRE(run(d, "CAM EDIT 1 STEPDOWN AUTO TOP 10").ok);
    CHECK(job.ops[0].stepdown == 0);
    CHECK(job.ops[0].top == 10);
    REQUIRE(run(d, "CAM EDIT 1 TOP AUTO").ok);
    CHECK(std::isnan(job.ops[0].top));
    CHECK(!run(d, "CAM EDIT 1 TOOL 9").ok);
    CHECK(!run(d, "CAM EDIT 3 LEAVE 1").ok);
    CHECK(!run(d, "CAM EDIT 1 WOBBLE 2").ok);
    CHECK(job.ops[0].tool == 1);
    // Move, duplicate, suppress.
    REQUIRE(run(d, "CAM MOVE 2 UP").ok);
    CHECK(job.ops[0].kind == CamOpKind::Parallel);
    CHECK(!run(d, "CAM MOVE 1 UP").ok);
    REQUIRE(run(d, "CAM MOVE 1 TO 2").ok);
    CHECK(job.ops[0].kind == CamOpKind::Rough);
    REQUIRE(run(d, "CAM DUPLICATE 2").ok);
    REQUIRE(job.ops.size() == 3);
    CHECK(job.ops[2].kind == CamOpKind::Parallel);
    REQUIRE(run(d, "CAM SUPPRESS 3").ok);
    CHECK(job.ops[2].suppressed);
    CHECK(has(text(run(d, "CAM LIST")), "(suppressed)"));
    // A suppressed operation isn't simulated or posted, and is kept in the saved job.
    CamSimulation sim;
    REQUIRE(simulateJob(job, d.scene.bodies()[0], sim));
    CHECK(sim.gouges == 0);
    CHECK(!job.ops[2].generated);
    const std::string g = postToString(job, d.scene.bodies()[0], "fanuc");
    REQUIRE(!g.empty());
    CHECK(checkPost(job, "fanuc", g).errors == 0);
    std::size_t opsPosted = 0;
    for (std::size_t q = g.find("(ROUGH IT"); q != std::string::npos; q = g.find("(ROUGH IT", q + 1)) ++opsPosted;
    CHECK(opsPosted == 1);
    std::size_t parallels = 0;
    for (std::size_t q = g.find("(PARALLEL"); q != std::string::npos; q = g.find("(PARALLEL", q + 1)) ++parallels;
    CHECK(parallels == 1); // the copy is suppressed
    CamJob back;
    REQUIRE(camFromJson(camToJson(job), back));
    CHECK(back.ops[2].suppressed);
    REQUIRE(run(d, "CAM UNSUPPRESS 3").ok);
    CHECK(!job.ops[2].suppressed);
    REQUIRE(run(d, "CAM SUPPRESS 1").ok);
    REQUIRE(run(d, "CAM SUPPRESS 2").ok);
    REQUIRE(run(d, "CAM SUPPRESS 3").ok);
    std::string e;
    CHECK(postToString(job, d.scene.bodies()[0], "fanuc", nullptr, &e).empty());
    CHECK(has(e, "suppressed"));
    REQUIRE(run(d, "CAM UNSUPPRESS 1").ok);
    // Over MCP.
    McpServer s(d, nullptr);
    auto call = [&](const std::string& args) {
        const auto j = Json::parse(s.handle(R"({"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"cam","arguments":)" + args + "}}"));
        return j.has_value() && !(*j)["result"]["isError"].asBool();
    };
    CHECK(call(R"({"operation":"edit","index":1,"leave":0.4})"));
    CHECK(job.ops[0].leave == 0.4);
    CHECK(call(R"({"operation":"move","index":1,"direction":"down"})"));
    CHECK(job.ops[1].kind == CamOpKind::Rough);
    CHECK(call(R"({"operation":"move","index":2,"to":1})"));
    CHECK(job.ops[0].kind == CamOpKind::Rough);
    CHECK(call(R"({"operation":"duplicate","index":1})"));
    CHECK(job.ops.size() == 4);
    CHECK(!call(R"({"operation":"edit","leave":0.4})"));
}

TEST_CASE("CAM panel edits: the fields changed become CAM EDIT options that reproduce them") {
    Document d;
    bracket(d);
    for (const char* l : {"CAM SETUP \"Block 1\" STOCK 3 TOP 1 MATERIAL aluminium", "CAM TOOL 1 FLAT 6", "CAM TOOL 2 FLAT 10", "CAM TOOL 3 DRILL 6",
                          "CAM ROUGH TOOL 2", "CAM CONTOUR TOOL 1", "CAM DRILL TOOL 3"})
        REQUIRE(run(d, l).ok);
    CamJob& job = d.state.cam;
    auto same = [](const CamOperation& a, const CamOperation& b) {
        auto eq = [](double x, double y) { return (std::isnan(x) && std::isnan(y)) || std::fabs(x - y) < 1e-6 * std::max(1.0, std::fabs(x)); };
        return a.name == b.name && a.tool == b.tool && eq(a.stepdown, b.stepdown) && eq(a.stepover, b.stepover) && eq(a.leave, b.leave) && eq(a.top, b.top) &&
               eq(a.bottom, b.bottom) && eq(a.angle, b.angle) && a.climb == b.climb && a.raster == b.raster && a.adaptive == b.adaptive && a.rest == b.rest &&
               a.side == b.side && a.tabs == b.tabs && eq(a.tabWidth, b.tabWidth) && eq(a.tabHeight, b.tabHeight) && eq(a.lead, b.lead) && a.cycle == b.cycle &&
               eq(a.peck, b.peck) && eq(a.dwell, b.dwell) && eq(a.setRpm, b.setRpm) && eq(a.setFeed, b.setFeed) && eq(a.setPlunge, b.setPlunge) &&
               eq(a.indexA, b.indexA) && eq(a.indexC, b.indexC);
    };
    CHECK(camEditArgs(job.ops[0], job.ops[0]).empty());
    // Rough: many fields at once, then back to automatic.
    CamOperation e = job.ops[0];
    e.name = "Big \"rough\"", e.tool = 1, e.stepdown = 2.5, e.stepover = 2, e.leave = 0.4, e.top = 14, e.bottom = 3, e.adaptive = true, e.climb = false;
    e.setRpm = 9000, e.setFeed = 1500, e.setPlunge = 400;
    std::string args = camEditArgs(job.ops[0], e);
    auto r = run(d, "CAM EDIT 1" + args);
    REQUIRE(r.ok);
    e.name = "Big 'rough'"; // quotes inside a name become single quotes
    CHECK(same(job.ops[0], e));
    e = job.ops[0];
    e.stepdown = 0, e.stepover = 0, e.top = std::numeric_limits<double>::quiet_NaN(), e.setRpm = 0, e.setFeed = 0, e.setPlunge = 0, e.adaptive = false;
    REQUIRE(run(d, "CAM EDIT 1" + camEditArgs(job.ops[0], e)).ok);
    CHECK(same(job.ops[0], e));
    // Contour: walls, tabs, lead.
    e = job.ops[1];
    e.side = 2, e.tabs = 3, e.tabWidth = 5, e.tabHeight = 2, e.lead = 3, e.leave = 0.1;
    REQUIRE(run(d, "CAM EDIT 2" + camEditArgs(job.ops[1], e)).ok);
    CHECK(same(job.ops[1], e));
    e.lead = -1;
    REQUIRE(run(d, "CAM EDIT 2" + camEditArgs(job.ops[1], e)).ok);
    CHECK(same(job.ops[1], e));
    // Drill: a peck depth keeps the cycle chosen; a 3+2 turn.
    e = job.ops[2];
    e.cycle = DrillCycle::ChipBreak, e.peck = 1.5, e.dwell = 0.2;
    REQUIRE(run(d, "CAM EDIT 3" + camEditArgs(job.ops[2], e)).ok);
    CHECK(same(job.ops[2], e));
    e.cycle = DrillCycle::Drill, e.peck = 0;
    REQUIRE(run(d, "CAM EDIT 3" + camEditArgs(job.ops[2], e)).ok);
    CHECK(same(job.ops[2], e));
}

TEST_CASE("UNDO and REDO take back CAM and lathe changes, in turn with model edits") {
    Document d;
    bracket(d);
    for (const char* l : {"CAM SETUP \"Block 1\" STOCK 3 TOP 1 MATERIAL aluminium", "CAM TOOL 1 FLAT 6", "CAM ROUGH TOOL 1 LEAVE 0.3", "CAM CONTOUR TOOL 1"})
        REQUIRE(run(d, l).ok);
    CamJob& job = d.state.cam;
    REQUIRE(run(d, "CAM EDIT 1 LEAVE 0.7").ok);
    REQUIRE(run(d, "CAM SUPPRESS 2").ok);
    auto r = run(d, "UNDO");
    REQUIRE(r.ok);
    CHECK(has(text(r), "CAM SUPPRESS 2"));
    CHECK(!d.state.cam.ops[1].suppressed);
    REQUIRE(run(d, "UNDO").ok);
    CHECK(std::fabs(d.state.cam.ops[0].leave - 0.3) < 1e-6);
    REQUIRE(run(d, "REDO").ok);
    CHECK(std::fabs(d.state.cam.ops[0].leave - 0.7) < 1e-6);
    // A model edit after a CAM edit is undone first; then the CAM edit.
    REQUIRE(run(d, "CAM DELETE 2").ok);
    REQUIRE(job.ops.size() == 1);
    REQUIRE(run(d, "SCULPT \"Block 1\" DRAW AT 0,15,15 RADIUS 6").ok);
    const std::size_t verts = d.scene.bodies()[0].mesh.vertices.size();
    REQUIRE(run(d, "UNDO").ok); // the sculpt
    CHECK(d.scene.bodies()[0].mesh.vertices.size() != verts);
    CHECK(d.state.cam.ops.size() == 1);
    REQUIRE(run(d, "UNDO").ok); // the delete
    CHECK(d.state.cam.ops.size() == 2);
    // A new CAM edit drops what could be redone.
    REQUIRE(run(d, "CAM EDIT 1 LEAVE 0.2").ok);
    CHECK(!run(d, "REDO").ok);
    // Lists and simulations change nothing to undo.
    const std::size_t depth = d.jobsUndo.size();
    REQUIRE(run(d, "CAM LIST").ok);
    REQUIRE(run(d, "CAM GENERATE").ok);
    CHECK(d.jobsUndo.size() == depth);
    // A new project forgets them.
    REQUIRE(run(d, "NEW").ok);
    CHECK(d.jobsUndo.empty());
}

TEST_CASE("CAM panel commands carry millimetres in an inch document; a tool written out comes back the same") {
    Document d;
    bracket(d);
    for (const char* l : {"CAM SETUP \"Block 1\" STOCK 3 TOP 1 MATERIAL aluminium", "CAM TOOL 1 FLAT 6", "CAM ROUGH TOOL 1 LEAVE 0.3"}) REQUIRE(run(d, l).ok);
    REQUIRE(run(d, "UNITS IN").ok);
    CamOperation e = d.state.cam.ops[0];
    e.leave = 0.5, e.stepdown = 2.0;
    REQUIRE(run(d, "CAM EDIT 1" + camEditArgs(d.state.cam.ops[0], e)).ok);
    CHECK(std::fabs(d.state.cam.ops[0].leave - 0.5) < 1e-6); // not 0.5 inch
    CHECK(std::fabs(d.state.cam.ops[0].stepdown - 2.0) < 1e-6);
    // Every option of a tool, through its command (the units don't matter either).
    CamTool t;
    t.number = 7, t.type = CamToolType::BullNose, t.diameter = 12, t.cornerRadius = 1.5, t.flutes = 4, t.fluteLength = 30, t.stickout = 45;
    t.holderDiameter = 32, t.holderLength = 50, t.hss = false, t.vc = 250, t.fz = 0.06, t.name = "Bull 12", t.vendor = "Acme", t.productId = "B12-4";
    REQUIRE(run(d, camToolCommand(t)).ok);
    const CamTool* b = d.state.cam.tool(7);
    REQUIRE(b != nullptr);
    CHECK(b->type == CamToolType::BullNose);
    CHECK(std::fabs(b->diameter - 12) < 1e-6);
    CHECK(std::fabs(b->cornerRadius - 1.5) < 1e-6);
    CHECK(b->flutes == 4);
    CHECK(std::fabs(b->fluteLength - 30) < 1e-6);
    CHECK(std::fabs(b->stickout - 45) < 1e-6);
    CHECK(std::fabs(b->holderDiameter - 32) < 1e-6);
    CHECK(std::fabs(b->holderLength - 50) < 1e-6);
    CHECK(std::fabs(b->vc - 250) < 1e-9);
    CHECK(std::fabs(b->fz - 0.06) < 1e-9);
    CHECK(b->name == "Bull 12");
    CHECK(b->vendor == "Acme");
    CHECK(b->productId == "B12-4");
    CamTool tap;
    tap.number = 8, tap.type = CamToolType::Tap, tap.diameter = 6, tap.pitch = 1.0, tap.flutes = 3, tap.fluteLength = 15, tap.hss = true;
    REQUIRE(run(d, camToolCommand(tap)).ok);
    CHECK(d.state.cam.tool(8)->hss);
    CHECK(std::fabs(d.state.cam.tool(8)->pitch - 1.0) < 1e-6);
    REQUIRE(run(d, "UNITS MM").ok);
}

TEST_CASE("CAM machine window: a machine written out as CAM MACHINE comes back the same") {
    Document d;
    bracket(d);
    REQUIRE(run(d, "CAM SETUP \"Block 1\"").ok);
    REQUIRE(run(d, "UNITS IN").ok);
    for (const char* preset : {"trunnion", "head-table", "head-head", "vmc", "generic"}) {
        CamMachine m = *camMachinePreset(preset);
        m.name = std::string("My ") + preset;
        m.hasWorkOffset = true, m.workOffset[0] = 100, m.workOffset[1] = 50.5, m.workOffset[2] = -300;
        m.toolPockets = 12, m.maxRpm = 9000, m.toolChange = 2.5;
        if (m.hasC && !m.cHead) m.cMin = -200, m.cMax = 200;
        const auto r = run(d, camMachineCommand(m));
        REQUIRE(r.ok);
        const CamMachine& b = d.state.cam.setup.machine;
        CHECK(b.name == m.name);
        CHECK(std::string(b.kinematics()) == m.kinematics());
        CHECK(b.hasA == m.hasA);
        CHECK(b.hasB == m.hasB);
        CHECK(b.hasC == m.hasC);
        CHECK(b.cHead == m.cHead);
        for (int i = 0; i < 3; ++i) {
            CHECK(std::fabs(b.travelMin[i] - m.travelMin[i]) < 1e-3);
            CHECK(std::fabs(b.travelMax[i] - m.travelMax[i]) < 1e-3);
            CHECK(std::fabs(b.workOffset[i] - m.workOffset[i]) < 1e-3);
        }
        CHECK(b.hasWorkOffset);
        if (m.hasA) CHECK((b.aMin == m.aMin && b.aMax == m.aMax));
        if (m.hasB) CHECK((b.bMin == m.bMin && b.bMax == m.bMax));
        if (m.hasC) CHECK((b.cMin == m.cMin && b.cMax == m.cMax));
        CHECK(b.toolPockets == 12);
        CHECK(b.maxRpm == 9000);
        CHECK(std::fabs(b.toolChange - 2.5) < 1e-6);
    }
    REQUIRE(run(d, "UNITS MM").ok);
}

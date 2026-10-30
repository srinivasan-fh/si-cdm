#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/FileIO.hpp"
#include "core/Json.hpp"
#include "core/Mcp.hpp"
#include "core/Stackup.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>

using namespace sicdm;

#define REQUIRE(c)                  \
    do {                            \
        const bool required_ = (c); \
        CHECK(required_);           \
        if (!required_) return;     \
    } while (0)

namespace {

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }

std::string joined(const CommandOutput& o) {
    std::string s;
    for (const std::string& l : o.lines) s += l + "\n";
    return s;
}

StackItem item(const char* name, double nominal, double tol, int sign = 1, StackDist dist = StackDist::Normal) {
    StackItem it;
    it.name = name;
    it.nominal = nominal;
    it.upper = tol, it.lower = -tol;
    it.sign = sign;
    it.dist = dist;
    return it;
}

// End play: a 50 +-0.1 housing less two parts, 20 and 29.5 +-0.05; spec 0.35 to 0.65.
ToleranceStack endPlay() {
    ToleranceStack s;
    s.name = "End play";
    s.items = {item("Housing", 50, 0.1), item("Part A", 20, 0.05, -1), item("Part B", 29.5, 0.05, -1)};
    s.hasLow = s.hasHigh = true, s.low = 0.35, s.high = 0.65;
    return s;
}

const ToleranceStack* stackNamed(const Document& d, const std::string& name) {
    for (const ToleranceStack& s : d.state.stacks)
        if (s.name == name) return &s;
    return nullptr;
}

} // namespace

TEST_CASE("stack-up: worst case, RSS and modified RSS, the spec, and each dimension's share") {
    const ToleranceStack s = endPlay();
    const StackResult r = analyzeStack(s);
    CHECK_NEAR(r.nominal, 0.5, 1e-12);
    CHECK_NEAR(r.mean, 0.5, 1e-12);
    CHECK_NEAR(r.wcLow, 0.3, 1e-12);
    CHECK_NEAR(r.wcHigh, 0.7, 1e-12);
    // sqrt(0.1^2 + 0.05^2 + 0.05^2) = 0.12247 at 3 sigma.
    CHECK_NEAR(3 * r.sigma, std::sqrt(0.015), 1e-12);
    CHECK_NEAR(r.rssLow, 0.5 - std::sqrt(0.015), 1e-12);
    CHECK_NEAR(r.mrssHigh, 0.5 + 1.5 * std::sqrt(0.015), 1e-12);
    CHECK_NEAR(r.variance[0], 100.0 * 0.01 / 0.015, 1e-9);
    CHECK_NEAR(r.wcShare[1], 25.0, 1e-9);
    CHECK(!r.wcPass);
    CHECK(r.rssPass);
    // 0.15 from the middle at sigma 0.040825: z 3.674, both tails 239 ppm; Cpk 1.2247.
    CHECK_NEAR(r.cpk, 0.15 / std::sqrt(0.015), 1e-9);
    CHECK_NEAR(r.ppm, 238.6, 1.0);
    // The Monte Carlo run agrees, and is the same every time.
    CHECK_NEAR(r.mcMean, 0.5, 0.001);
    CHECK_NEAR(r.mcSigma, r.sigma, 0.02 * r.sigma);
    CHECK(r.mcPpm > 100 && r.mcPpm < 450);
    CHECK_NEAR(r.mcP1 - r.mcP0, 6 * r.sigma, 0.06 * 6 * r.sigma);
    const StackResult again = analyzeStack(s);
    CHECK(again.mcMean == r.mcMean && again.mcPpm == r.mcPpm);
    int counted = 0;
    for (int c : r.histogram) counted += c;
    CHECK(counted == r.samples);
    // What to change: every tolerance to 75% fits the worst case; the housing is the big one.
    std::string notes;
    for (const std::string& n : r.notes) notes += n + "\n";
    CHECK(notes.find("at 75% of what it is") != std::string::npos);
    CHECK(notes.find("Housing gives 66.7% of the variance") != std::string::npos);
}

TEST_CASE("stack-up: uniform and triangular spreads, Cp, one-sided tolerances, sensitivity") {
    ToleranceStack s;
    s.name = "Spread";
    s.items = {item("U", 10, 0.3, 1, StackDist::Uniform)};
    StackResult r = analyzeStack(s);
    CHECK_NEAR(r.sigma, 0.3 / std::sqrt(3.0), 1e-12);
    CHECK_NEAR(r.mcSigma, r.sigma, 0.01 * r.sigma);
    CHECK(r.mcLow >= 9.7 && r.mcHigh <= 10.3); // never outside its band
    // One uniform dimension: 3 sigma (0.52) is past its band, so RSS stops at the worst case.
    CHECK_NEAR(r.rssHigh, 10.3, 1e-12);
    CHECK_NEAR(r.rssLow, 9.7, 1e-12);
    CHECK(r.notes.back().find("3 sigma reaches past the worst case") != std::string::npos);
    s.items = {item("T", 10, 0.3, 1, StackDist::Triangular)};
    r = analyzeStack(s);
    CHECK_NEAR(r.sigma, 0.3 / std::sqrt(6.0), 1e-12);
    CHECK_NEAR(r.mcSigma, r.sigma, 0.01 * r.sigma);
    s.items = {item("N", 10, 0.3)};
    s.items[0].cp = 1.5;
    r = analyzeStack(s);
    CHECK_NEAR(r.sigma, 0.3 / 4.5, 1e-12);
    // 20 +0.1/0 less 5 +0/-0.04 through a 2:1 lever (x0.5): middle 20.05 - 0.5 x 4.98.
    StackItem a = item("Bore", 20, 0), b = item("Pin", 5, 0, -1);
    a.upper = 0.1, a.lower = 0;
    b.upper = 0, b.lower = -0.04, b.sens = 0.5;
    s.items = {a, b};
    r = analyzeStack(s);
    CHECK_NEAR(r.nominal, 17.5, 1e-12);
    CHECK_NEAR(r.mean, 20.05 - 0.5 * 4.98, 1e-12);
    CHECK_NEAR(r.wcHigh, 20.1 - 0.5 * 4.96, 1e-12);
    CHECK_NEAR(r.wcLow, 20.0 - 0.5 * 5.0, 1e-12);
    // A middle outside the spec: tolerances can't fix it.
    s.hasLow = true, s.low = 18;
    r = analyzeStack(s);
    CHECK(r.notes[0].find("change a nominal size") != std::string::npos);
}

TEST_CASE("STACK: dimensions typed, from fits and general tolerances, from bodies and the drawing") {
    Document d;
    REQUIRE(run(d, "STACK NEW \"End play\" SPEC 0.35 0.65 NOTE \"the shaft's end play\"").ok);
    CHECK(!run(d, "STACK NEW \"End play\"").ok); // taken
    REQUIRE(run(d, "STACK \"End play\" ADD Housing 50 TOL 0.1").ok);
    REQUIRE(run(d, "STACK \"End play\" ADD \"Part A\" 20 TOL +0.05 -0.05 MINUS").ok);
    const CommandOutput o = run(d, "STACK \"End play\" ADD \"Part B\" -29.5 TOL 0.05");
    REQUIRE(o.ok);
    CHECK(joined(o).find("worst case 0.3 to 0.7, RSS 0.37753 to 0.62247 mm; fits the spec by RSS only") != std::string::npos);
    const ToleranceStack* s = stackNamed(d, "End play");
    REQUIRE(s && s->items.size() == 3);
    CHECK(s->items[2].sign == -1 && s->items[2].nominal == 29.5); // a negative size takes away
    CHECK(s->note == "the shaft's end play");
    const std::string report = joined(run(d, "STACK \"End play\""));
    CHECK(report.find("Worst case: 0.3 to 0.7 mm") != std::string::npos);
    CHECK(report.find("RSS fits, Cpk 1.22, 239 ppm outside") != std::string::npos);
    // Fits and general tolerances follow the nominal.
    REQUIRE(run(d, "STACK \"End play\" ADD Pin 10 FIT g6").ok);
    CHECK_NEAR(s->items[3].upper, -0.005, 1e-12);
    CHECK_NEAR(s->items[3].lower, -0.014, 1e-12);
    REQUIRE(run(d, "STACK \"End play\" EDIT 4 NOMINAL 20").ok); // g6 on 20: -0.007 / -0.020
    CHECK_NEAR(s->items[3].lower, -0.020, 1e-12);
    REQUIRE(run(d, "STACK \"End play\" ADD Bush 10 GENERAL m UNIFORM").ok);
    CHECK_NEAR(s->items[4].upper, 0.2, 1e-12);
    CHECK(s->items[4].dist == StackDist::Uniform);
    REQUIRE(run(d, "STACK \"End play\" EDIT 5 LIMITS 10.1 9.95 NAME Bushing SENS 0.5 TRIANGULAR").ok);
    CHECK_NEAR(s->items[4].upper, 0.1, 1e-12);
    CHECK_NEAR(s->items[4].lower, -0.05, 1e-12);
    CHECK(s->items[4].name == "Bushing" && s->items[4].sens == 0.5 && s->items[4].general == 0);
    REQUIRE(run(d, "STACK \"End play\" REMOVE 5").ok);
    REQUIRE(run(d, "STACK \"End play\" REMOVE 4").ok);
    CHECK(s->items.size() == 3);
    // Refusals.
    CHECK(!run(d, "STACK \"End play\" ADD Thing").ok);           // no size
    CHECK(!run(d, "STACK \"End play\" ADD Thing 5 FIT Q7").ok);  // no such fit
    CHECK(!run(d, "STACK \"End play\" ADD Thing 5 GENERAL x").ok);
    CHECK(!run(d, "STACK \"End play\" ADD Thing 5 TOL -0.1").ok); // a +- tolerance is positive
    CHECK(!run(d, "STACK \"End play\" EDIT 9 TOL 0.1").ok);
    CHECK(!run(d, "STACK Nothing ADD x 1").ok);
    CHECK(!run(d, "STACK \"End play\" SPEC 1").ok);
    CHECK(s->items.size() == 3);

    // From the model: two blocks' lengths along X, and the gap between them checked.
    REQUIRE(run(d, "BOX 20 30 30 @0,0,15").ok);
    REQUIRE(run(d, "BOX 29.5 30 30 @25.25,0,15").ok); // 10.5 to 40: a 0.5 gap after Block 1
    REQUIRE(run(d, "STACK NEW Blocks GAP \"Block 1\" \"Block 2\" ALONG X").ok);
    REQUIRE(run(d, "STACK Blocks ADD BODY \"Block 1\" ALONG X MINUS").ok);
    const ToleranceStack* b = stackNamed(d, "Blocks");
    REQUIRE(b && b->items.size() == 1);
    CHECK_NEAR(b->items[0].nominal, 20, 1e-4);
    CHECK(b->items[0].general == 'm');
    CHECK_NEAR(b->items[0].upper, 0.2, 1e-12); // ISO 2768-m until given one
    REQUIRE(run(d, "STACK Blocks ADD BODY \"Block 2\" ALONG -X TOL 0.05 MINUS").ok);
    CHECK_NEAR(b->items[1].nominal, 29.5, 1e-4);
    REQUIRE(run(d, "STACK Blocks ADD Space 50 TOL 0.1").ok);
    std::string rep = joined(run(d, "STACK Blocks REPORT SAMPLES 0"));
    CHECK(rep.find("The model's gap from Block 1 to Block 2: 0.5 mm, as the stack's nominal") != std::string::npos);
    CHECK(rep.find("Monte Carlo") == std::string::npos);
    // The model changes: UPDATE measures again (the general tolerance follows the size).
    REQUIRE(run(d, "SCALE \"Block 1\" 2").ok);
    const CommandOutput up = run(d, "STACK Blocks UPDATE");
    REQUIRE(up.ok);
    CHECK(joined(up).find("Block 1 (X): 20 +0.2/-0.2 is now 40 +0.3/-0.3") != std::string::npos);
    CHECK(joined(run(d, "STACK Blocks")).find("but the stack's nominal is") != std::string::npos); // Block 1 grew both ways
    CHECK(!run(d, "STACK Blocks ADD BODY Nothing ALONG X").ok);
    CHECK(!run(d, "STACK Blocks ADD BODY \"Block 1\" ALONG W").ok);

    // From the drawing: its tolerance, or the title block's general one.
    REQUIRE(run(d, "BOX 80 50 12 @300,0,0").ok);
    REQUIRE(run(d, "SHEET NEW SIZE A3").ok);
    REQUIRE(run(d, "SHEET DIM VERTICAL Front 340,0,0 340,0,12 TOL +0.05 -0.1").ok);
    REQUIRE(run(d, "SHEET DIM HORIZONTAL Top 260,25,12 340,25,12").ok);
    REQUIRE(run(d, "STACK NEW Sheet").ok);
    REQUIRE(run(d, "STACK Sheet ADD DIM 1").ok);
    REQUIRE(run(d, "STACK Sheet ADD DIM 2 MINUS").ok);
    const ToleranceStack* sh = stackNamed(d, "Sheet");
    REQUIRE(sh && sh->items.size() == 2);
    CHECK_NEAR(sh->items[0].nominal, 12, 1e-4);
    CHECK_NEAR(sh->items[0].upper, 0.05, 1e-6);
    CHECK_NEAR(sh->items[0].lower, -0.1, 1e-6);
    CHECK_NEAR(sh->items[1].nominal, 80, 1e-4);
    CHECK_NEAR(sh->items[1].upper, 0.3, 1e-12); // ISO 2768-m on 80
    CHECK(!run(d, "STACK Sheet ADD DIM 7").ok);

    // List, rename, delete.
    CHECK(run(d, "STACK").lines.size() == 3);
    REQUIRE(run(d, "STACK Sheet RENAME Drawing").ok);
    CHECK(stackNamed(d, "Drawing") != nullptr);
    CHECK(!run(d, "STACK Drawing RENAME Blocks").ok);
    REQUIRE(run(d, "STACK Drawing DELETE").ok);
    CHECK(d.state.stacks.size() == 2);
}

TEST_CASE("STACK: undo, saved with the project, CSV and SVG, over MCP") {
    Document d;
    REQUIRE(run(d, "STACK NEW Gap MIN 0.1").ok);
    REQUIRE(run(d, "STACK Gap ADD Housing 50 TOL 0.1 UNIFORM").ok);
    REQUIRE(run(d, "STACK Gap ADD Shaft 49.7 TOL +0 -0.05 MINUS CP 1.33").ok);
    REQUIRE(run(d, "STACK Gap ADD Pin 10 FIT g6 SENS 0.25").ok);
    REQUIRE(run(d, "UNDO").ok);
    REQUIRE(stackNamed(d, "Gap") != nullptr);
    CHECK(stackNamed(d, "Gap")->items.size() == 2);
    REQUIRE(run(d, "REDO").ok);
    CHECK(stackNamed(d, "Gap")->items.size() == 3);

    const auto dir = std::filesystem::temp_directory_path();
    const std::string path = (dir / "sicdm_stack_test.sicdm").string();
    std::string err;
    REQUIRE(saveProject(d.scene, d.state, path, &err));
    Scene sc;
    ProjectState st;
    REQUIRE(loadProject(sc, st, path, &err, nullptr));
    REQUIRE(st.stacks.size() == 1);
    const ToleranceStack& s = st.stacks[0];
    CHECK(s.name == "Gap" && s.hasLow && !s.hasHigh && s.low == 0.1);
    REQUIRE(s.items.size() == 3);
    CHECK(s.items[0].dist == StackDist::Uniform);
    CHECK(s.items[1].sign == -1 && s.items[1].cp == 1.33 && s.items[1].upper == 0 && s.items[1].lower == -0.05);
    CHECK(s.items[2].fit == "g6" && s.items[2].sens == 0.25);
    std::error_code ec;
    std::filesystem::remove(path, ec);

    const std::string csv = (dir / "sicdm_stack_test.csv").string(), svg = (dir / "sicdm_stack_test.svg").string();
    REQUIRE(run(d, "STACK Gap REPORT SAMPLES 5000 CSV \"" + csv + "\" SVG \"" + svg + "\"").ok);
    std::stringstream c, g;
    c << std::ifstream(csv).rdbuf();
    g << std::ifstream(svg).rdbuf();
    CHECK(c.str().rfind("Item,Sign,Name,Nominal (mm)", 0) == 0);
    CHECK(c.str().find("2,-,Shaft,49.7,0,-0.05") != std::string::npos);
    CHECK(c.str().find("Worst case,") != std::string::npos);
    CHECK(g.str().find("<svg") != std::string::npos && g.str().find("LSL 0.1") != std::string::npos && g.str().find("</svg>") != std::string::npos);
    std::filesystem::remove(csv, ec);
    std::filesystem::remove(svg, ec);

    Document m;
    McpServer server(m, nullptr);
    const auto tool = [&](const std::string& args) {
        const std::string msg = R"({"jsonrpc":"2.0","id":5,"method":"tools/call","params":{"name":"tolerance_stack","arguments":)" + args + "}}";
        const auto r = Json::parse(server.handle(msg));
        return r ? (*r)["result"] : Json();
    };
    CHECK(!tool(R"({"operation":"new","stack":"Lid","low":0.35,"high":0.65})")["isError"].asBool());
    CHECK(!tool(R"({"operation":"add","stack":"Lid","name":"Housing","nominal":50,"tol":0.1})")["isError"].asBool());
    CHECK(!tool(R"({"operation":"add","stack":"Lid","name":"Part A","nominal":20,"upper":0.05,"lower":-0.05,"minus":true})")["isError"].asBool());
    CHECK(!tool(R"({"operation":"add","stack":"Lid","name":"Part B","nominal":29.5,"tol":0.05,"minus":true,"dist":"uniform"})")["isError"].asBool());
    CHECK(!tool(R"({"operation":"edit","stack":"Lid","index":3,"dist":"normal"})")["isError"].asBool());
    const Json r = tool(R"({"operation":"report","stack":"Lid","samples":1000})");
    CHECK(!r["isError"].asBool());
    CHECK(r["content"][0]["text"].asString().find("Worst case: 0.3 to 0.7 mm") != std::string::npos);
    CHECK(tool(R"({"operation":"add","stack":"Lid"})")["isError"].asBool());
    CHECK(tool(R"({"operation":"weld","stack":"Lid"})")["isError"].asBool());
    CHECK(!tool(R"({"operation":"delete","stack":"Lid"})")["isError"].asBool());
    CHECK(m.state.stacks.empty());
}

TEST_CASE("Vector stacks: a 2D loop with an angle, by its sensitivities and the loop itself") {
    // 100 along X, turn 90 deg, 50 along (the turned) X: the far end is at (100, 50).
    ToleranceStack st;
    st.name = "L";
    st.vector = true;
    StackItem base;
    base.name = "Base", base.nominal = 100, base.upper = 0.1, base.lower = -0.1, base.dir = {1, 0, 0};
    StackItem turn;
    turn.name = "Turn", turn.nominal = 90, turn.upper = 0.5, turn.lower = -0.5, turn.angle = true, turn.axis = {0, 0, 1};
    StackItem arm;
    arm.name = "Arm", arm.nominal = 50, arm.upper = 0.1, arm.lower = -0.1, arm.dir = {1, 0, 0};
    st.items = {base, turn, arm};
    st.samples = 200000;
    StackResult r = analyzeStack(st);
    const double R = std::sqrt(100.0 * 100 + 50.0 * 50);
    CHECK(std::fabs(r.nominal - R) < 1e-6);
    const std::vector<double> s = stackSensitivities(st);
    CHECK(std::fabs(s[0] - 100 / R) < 1e-5);
    CHECK(std::fabs(s[2] - 50 / R) < 1e-5);
    CHECK(std::fabs(s[1] + 100 * 50 * (3.14159265358979 / 180) / R) < 1e-4); // mm per degree
    const double wc = 0.1 * 100 / R + 0.1 * 50 / R + 0.5 * 100 * 50 * (3.14159265358979 / 180) / R;
    CHECK(std::fabs(r.wcHigh - (R + wc)) < 1e-4);
    CHECK(std::fabs(r.wcLow - (R - wc)) < 1e-4);
    CHECK(std::fabs(r.mcMean - R) < 0.01); // nearly straight here: the loop agrees
    CHECK(r.mcP0 > r.wcLow); // the 99.73% points inside the worst case (normal tails reach past it)
    CHECK(r.mcP1 < r.wcHigh);
    // Nearly straight here: each one's share (its total effect on the variance) is the linear
    // one, (sensitivity x sigma)^2 of the sum.
    {
        double sum = 0;
        std::vector<double> lin(3);
        for (std::size_t i = 0; i < 3; ++i) lin[i] = std::pow(s[i] * stackItemSigma(st.items[i]), 2), sum += lin[i];
        for (std::size_t i = 0; i < 3; ++i) CHECK(std::fabs(r.variance[i] - 100 * lin[i] / sum) < 0.5);
    }
    // Along X only: the arm, square to X, doesn't count; the angle swings its end.
    st.measure = {1, 0, 0};
    r = analyzeStack(st);
    CHECK(std::fabs(r.nominal - 100) < 1e-6);
    CHECK(std::fabs(stackSensitivities(st)[2]) < 1e-6);
    // The loop's own extreme: the arm longest and turned least (the linear figure, 100.5363,
    // misses that the two tolerances work together).
    CHECK(std::fabs(r.wcHigh - (100.1 + 50.1 * std::cos(89.5 * 3.14159265358979 / 180))) < 1e-6);
    // Folded back on itself (out and back, 180 deg): the straight-line view says it closes to
    // nothing and doesn't move; the loop itself never goes below zero and opens up.
    ToleranceStack fold = st;
    fold.measure = {0, 0, 0};
    fold.items[1].nominal = 180, fold.items[1].upper = 2, fold.items[1].lower = -2;
    fold.items[2].nominal = 100;
    r = analyzeStack(fold);
    CHECK(std::fabs(r.nominal) < 1e-6);
    CHECK(r.mcLow >= 0);
    CHECK(r.mcMean > 0.5);
    // Worst case: the loop can close exactly (least 0), and opens most with the angle at its
    // limit and the lengths apart: the greatest of the box's corners.
    CHECK(std::fabs(r.wcLow) < 1e-6);
    double most = 0;
    for (double a : {99.9, 100.1})
        for (double b : {99.9, 100.1})
            for (double t : {178.0, 182.0}) {
                const double q = t * 3.14159265358979 / 180;
                most = std::max(most, std::hypot(a + b * std::cos(q), b * std::sin(q)));
            }
    CHECK(std::fabs(r.wcHigh - most) < 1e-6);
    // RSS at the kink: the loop's own mean and spread, integrated over every dimension, agree
    // with the sampled loop (the straight-line view says 0 and 0).
    CHECK(std::fabs(r.mean - r.mcMean) < 0.02 * r.mcMean);
    CHECK(std::fabs(r.sigma - r.mcSigma) < 0.03 * r.mcSigma);
    // Its own shape: it leans high (it can't go below 0) and RSS's limits are its own 0.135%
    // and 99.865% points (the high one capped by the worst case: the angle's normal tails reach
    // past its +-2 deg box).
    CHECK(r.skew > 0.8);
    CHECK(std::fabs(r.rssLow - r.mcP0) < 0.005);
    CHECK(std::fabs(r.rssHigh - r.wcHigh) < 1e-9);
    CHECK(r.variance[1] > 99.9); // the turn makes all of it
    // The share outside a spec: from the loop's own distribution, near or far in its tail
    // (references: 20 million samples of the same loop; a normal fit would say 12500 ppm for
    // the first).
    {
        ToleranceStack sp = fold;
        sp.hasHigh = true, sp.high = 2.5;
        const StackResult q = analyzeStack(sp);
        CHECK(std::fabs(q.ppm - 31681) < 0.02 * 31681);
        sp.items[2].nominal = 70.1, sp.high = 30.15; // out and most of the way back
        const StackResult far = analyzeStack(sp);
        CHECK(std::fabs(far.ppm - 250.6) < 0.05 * 250.6);
        CHECK(far.skew > 0.15);
        sp.items[2].nominal = 50, sp.high = 50.12;
        CHECK(std::fabs(analyzeStack(sp).ppm - 9809) < 0.02 * 9809);
    }
    bool noted = false;
    for (const std::string& n : r.notes) noted = noted || n.find("vector stack") != std::string::npos;
    CHECK(noted);
    // Where the bend moves the mean: along Y the arm's end is 50 sin(turn), on average
    // 50 exp(-s^2 / 2) with the turn's sigma s: second order finds it, as the loop does.
    ToleranceStack wide = st;
    wide.measure = {0, 1, 0};
    wide.items[1].upper = 10, wide.items[1].lower = -10; // +-10 deg: sigma 3.33 deg
    r = analyzeStack(wide);
    const double sTurn = 10.0 / 3 * 3.14159265358979 / 180;
    CHECK(std::fabs(r.mean - 50 * std::exp(-sTurn * sTurn / 2)) < 2e-3);
    CHECK(std::fabs(r.mcMean - r.mean) < 0.01);
    {
        // Leaning low (a turn either way shortens it): 49.6 is 2.8 sigma below the mean, yet
        // about 3% are below it (20 million samples: 30413 ppm).
        ToleranceStack sp = wide;
        sp.hasLow = true, sp.low = 49.6;
        const StackResult q = analyzeStack(sp);
        CHECK(q.skew < -2);
        CHECK(std::fabs(q.ppm - q.mcPpm) < 0.05 * q.mcPpm);
        CHECK(std::fabs(q.rssLow - q.mcP0) < 0.05);
    }
    CHECK(std::fabs(r.mean - 50) > 0.08); // the straight-line view's 50 is off by more than that
    CHECK(std::fabs(r.wcHigh - 50.1) < 1e-6); // the arm at its longest, square to X
    // A straight stack is as it was: the value is the signed sum.
    ToleranceStack lin;
    lin.items = {base, arm};
    lin.items[1].sign = -1;
    CHECK(std::fabs(stackValue(lin, {100, 50}) - 50) < 1e-12);
    // Saved and read back.
    const std::vector<ToleranceStack> back = stacksFromJson(stacksToJson({st}));
    REQUIRE(back.size() == 1);
    CHECK(back[0].vector);
    CHECK(back[0].items[1].angle);
    CHECK(std::fabs(back[0].measure.x - 1) < 1e-6f);
    CHECK(std::fabs(analyzeStack(back[0]).nominal - 100) < 1e-6);
}

TEST_CASE("STACK VECTOR: lengths along directions and angles, in any units") {
    Document d;
    REQUIRE(run(d, "UNITS IN").ok); // lengths typed in inches; angles stay degrees
    REQUIRE(run(d, "STACK NEW Pin VECTOR DISTANCE SPEC 4.38 4.42").ok);
    REQUIRE(run(d, "STACK Pin ADD Base 4 TOL 0.004 DIR X").ok);
    REQUIRE(run(d, "STACK Pin ADD Turn 90 TOL 0.5 ANGLE AXIS Z").ok);
    REQUIRE(run(d, "STACK Pin ADD Arm 2 TOL 0.004 DIR X").ok);
    const ToleranceStack& st = d.state.stacks[0];
    CHECK(st.vector);
    CHECK(std::fabs(st.items[0].nominal - 101.6) < 1e-6);
    CHECK(std::fabs(st.items[1].nominal - 90) < 1e-9); // not 90 inches
    CHECK(std::fabs(st.items[1].upper - 0.5) < 1e-9);
    const CommandOutput rep = run(d, "STACK Pin REPORT SAMPLES 20000");
    REQUIRE(rep.ok);
    const std::string text = joined(rep);
    CHECK(text.find("vector loop") != std::string::npos);
    CHECK(text.find("per deg") != std::string::npos);
    CHECK(std::fabs(analyzeStack(st).nominal - std::sqrt(101.6 * 101.6 + 50.8 * 50.8)) < 1e-6);
    REQUIRE(run(d, "STACK Pin MEASURE Y").ok);
    CHECK(std::fabs(analyzeStack(d.state.stacks[0]).nominal - 50.8) < 1e-5);
    REQUIRE(run(d, "STACK Pin LINEAR").ok);
    CHECK(!d.state.stacks[0].vector);
    CHECK(!run(d, "STACK Pin ADD Bad 1 DIR W").ok);
}
TEST_CASE("Vector stacks: the share outside a spec far out in the tails, against exact answers") {
    // Normal dimensions of sigma 0.1 along X, Y, Z: the distance from the start is normal (one,
    // measured along X), Rayleigh (two) or Maxwell (three), whose tails are known exactly.
    auto mk = [](int dims) {
        ToleranceStack st;
        st.vector = true;
        const Vec3 d[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        for (int i = 0; i < dims; ++i) {
            StackItem it;
            it.name = "D" + std::to_string(i);
            it.nominal = 0, it.upper = 0.3, it.lower = -0.3, it.dir = d[i];
            st.items.push_back(it);
        }
        st.samples = 1000;
        st.hasHigh = true;
        return st;
    };
    const double pi = 3.14159265358979;
    for (const double k : {4.0, 5.0, 6.0, 7.0, 8.0}) { // sigmas out: from 3e-5 to 6e-16 past it
        ToleranceStack lin = mk(1);
        lin.items[0].nominal = 10, lin.measure = {1, 0, 0};
        lin.high = 10 + k * 0.1;
        const double e1 = 0.5 * std::erfc(k / std::sqrt(2.0));
        CHECK(std::fabs(analyzeStack(lin).ppm / 1e6 / e1 - 1) < 0.02);
        ToleranceStack ray = mk(2); // every direction is a way out: a whole ring of most likely points
        ray.high = k * 0.1;
        const double e2 = std::exp(-k * k / 2);
        CHECK(std::fabs(analyzeStack(ray).ppm / 1e6 / e2 - 1) < 0.04);
        ToleranceStack chi = mk(3);
        chi.high = k * 0.1;
        const double e3 = std::erfc(k / std::sqrt(2.0)) + std::sqrt(2 / pi) * k * std::exp(-k * k / 2);
        CHECK(std::fabs(analyzeStack(chi).ppm / 1e6 / e3 - 1) < 0.04);
    }
    // Two uniform dimensions summed: a triangle, whose last eps of its reach holds eps^2 / 2;
    // past its reach, nothing at all.
    ToleranceStack u = mk(2);
    u.items[1].dir = {1, 0, 0};
    u.items[0].nominal = u.items[1].nominal = 5;
    u.items[0].dist = u.items[1].dist = StackDist::Uniform;
    u.measure = {1, 0, 0};
    for (const double eps : {0.1, 0.01, 0.001}) {
        u.high = 10 + 0.6 * (1 - eps);
        CHECK(std::fabs(analyzeStack(u).ppm / 1e6 / (eps * eps / 2) - 1) < 0.03);
    }
    u.high = 10.61;
    CHECK(analyzeStack(u).ppm == 0);
}

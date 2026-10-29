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

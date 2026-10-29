#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/FileIO.hpp"
#include "core/Json.hpp"
#include "core/Mcp.hpp"
#include "core/Motion.hpp"
#include "core/Scene.hpp"
#include "core/Study.hpp"

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

constexpr double kPiD = 3.14159265358979323846;

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }

const Motion* motionNamed(const Document& d, const std::string& name) {
    for (const Motion& m : d.scene.motions())
        if (m.name == name) return &m;
    return nullptr;
}

const StudySeries* seriesNamed(const StudyResult& r, const std::string& name) {
    for (const StudySeries& s : r.series)
        if (s.name == name) return &s;
    return nullptr;
}

std::string joined(const CommandOutput& o) {
    std::string s;
    for (const std::string& l : o.lines) s += l + "\n";
    return s;
}

// An eccentric cam: a 20 mm round turned about a point 5 mm off its centre, and a flat
// follower on top sliding along Y.
void eccentricCam(Document& d, int speed) {
    run(d, "CYLINDER 20 10 @5,0,0");
    run(d, "BOX 20 10 10 @0,35,0");
    run(d, "MOTION CIRCLE \"Cam spin\" AXIS Z AT 0,0,0 SPEED " + std::to_string(speed) + " BODIES \"Cylinder 1\"");
}

} // namespace

TEST_CASE("contact motions: a flat follower rides an eccentric cam, a rocker rests on a turning block") {
    Document d;
    eccentricCam(d, 90);
    REQUIRE(run(d, "MOTION LINEAR \"Follower\" AXIS Y CONTACT \"Cylinder 1\" BODIES \"Block 2\"").ok);
    const Motion* f = motionNamed(d, "Follower");
    REQUIRE(f != nullptr);
    CHECK(f->contact == "Cylinder 1");
    // The follower's underside (y 30) sits on the cam's top, 20 + 5 sin(turn) up.
    double worst = 0;
    for (int k = 0; k <= 40; ++k) {
        const float t = 0.1f * static_cast<float>(k);
        bool touching = false;
        const float v = contactValue(d.scene, *f, t, &touching);
        CHECK(touching);
        worst = std::max(worst, std::fabs(v - (5 * std::sin(kPiD / 2 * t) - 10)));
    }
    CHECK(worst < 0.03); // the cam's facets
    // Its rate: 5 x pi/2 cos(turn) mm/s.
    CHECK_NEAR(motionRate(d.scene, *f, 0.0f), 5 * kPiD / 2, 0.1);
    CHECK_NEAR(motionRate(d.scene, *f, 1.0f), 0.0, 0.05);
    // The follower's block really moves with it.
    std::size_t block = 0;
    for (std::size_t i = 0; i < d.scene.bodies().size(); ++i)
        if (d.scene.bodies()[i].name == "Block 2") block = i;
    CHECK_NEAR(bodyMotionTransform(d.scene, block, 1.0f).transformPoint({0, 35, 0}).y, 35 - 5.0, 0.03);

    // A bar resting on a square cam, turning about its far end (its angle: down is negative),
    // against a brute-force search for where the two squares first touch.
    REQUIRE(run(d, "BOX 10 10 10 @60,0,0").ok);
    REQUIRE(run(d, "BOX 50 6 6 @75,25,0").ok);
    REQUIRE(run(d, "MOTION CIRCLE \"Cam 2 spin\" AXIS Z AT 60,0,0 SPEED 90 BODIES \"Block 3\"").ok);
    REQUIRE(run(d, "MOTION CIRCLE \"Rocker\" AXIS 0,0,-1 AT 100,25,3 CONTACT \"Block 3\" BODIES \"Block 4\"").ok);
    const Motion* r = motionNamed(d, "Rocker");
    REQUIRE(r != nullptr);
    CHECK_NEAR(contactValue(d.scene, *r, 0.0f), -20.4695, 2e-3);
    CHECK_NEAR(contactValue(d.scene, *r, 0.25f), -22.2095, 2e-3);
    CHECK_NEAR(contactValue(d.scene, *r, 0.5f), -20.2185, 2e-3);
    CHECK(joined(run(d, "MOTION")).find("rides on Block 3") != std::string::npos);
    // Renaming the cam keeps the follower on it; NONE frees it.
    for (std::size_t i = 0; i < d.scene.bodies().size(); ++i)
        if (d.scene.bodies()[i].name == "Block 3") d.scene.renameBody(i, "Square cam");
    CHECK(motionNamed(d, "Rocker")->contact == "Square cam");
    REQUIRE(run(d, "MOTION CIRCLE \"Rocker\" CONTACT NONE").ok);
    CHECK(motionNamed(d, "Rocker")->contact.empty());
    CHECK(!run(d, "MOTION LINEAR \"Follower\" CONTACT \"No such body\"").ok);
}

TEST_CASE("STUDY: travel, velocity and acceleration of a cam follower against the analytic ones") {
    Document d;
    eccentricCam(d, 360);
    REQUIRE(run(d, "MOTION LINEAR \"Follower\" AXIS Y CONTACT \"Cylinder 1\" BODIES \"Block 2\"").ok);
    const CommandOutput o = run(d, "STUDY Follower FROM 0 TO 2 SAMPLES 801");
    REQUIRE(o.ok);
    REQUIRE(d.study != nullptr);
    const StudyResult& r = *d.study;
    CHECK(r.t.size() == 801);
    CHECK_NEAR(r.t.back(), 2.0, 1e-9);
    const StudySeries* x = seriesNamed(r, "Follower travel");
    const StudySeries* v = seriesNamed(r, "Follower velocity");
    const StudySeries* a = seriesNamed(r, "Follower acceleration");
    REQUIRE(x && v && a);
    CHECK(x->unit == "mm" && v->unit == "mm/s" && a->unit == "mm/s2");
    // x = 5 sin(2 pi t) - 10: velocity peak 10 pi, acceleration peak 20 pi^2.
    double vMax = 0, aMax = 0, vErr = 0;
    for (std::size_t i = 0; i < r.t.size(); ++i) {
        vMax = std::max(vMax, std::fabs(v->v[i]));
        aMax = std::max(aMax, std::fabs(a->v[i]));
        vErr = std::max(vErr, std::fabs(v->v[i] - 10 * kPiD * std::cos(2 * kPiD * r.t[i])));
    }
    CHECK_NEAR(vMax, 10 * kPiD, 0.05 * 10 * kPiD);
    CHECK_NEAR(aMax, 20 * kPiD * kPiD, 0.06 * 20 * kPiD * kPiD);
    CHECK(vErr < 0.06 * 10 * kPiD);
    // A motion of its own: the rate is exact, and a steady turn has no acceleration.
    REQUIRE(run(d, "STUDY \"Cam spin\" TO 1").ok);
    const StudySeries* w = seriesNamed(*d.study, "Cam spin velocity");
    const StudySeries* wa = seriesNamed(*d.study, "Cam spin acceleration");
    REQUIRE(w && wa);
    for (std::size_t i = 0; i < w->v.size(); ++i) {
        CHECK_NEAR(w->v[i], 360.0, 1e-6);
        CHECK_NEAR(wa->v[i], 0.0, 1e-9);
    }
    // A point on the cam's rim goes round a 15 mm circle (its centre 5 off the axis + 10).
    REQUIRE(run(d, "STUDY POINT \"Cylinder 1\" 15,0,5 TO 1 SAMPLES 401").ok);
    const std::vector<std::string> sum = studySummary(*d.study);
    CHECK(!sum.empty());
    bool path = false;
    for (const std::string& n : d.study->notes)
        if (n.find("path ") != std::string::npos) {
            const double got = std::stod(n.substr(n.find("path ") + 5));
            CHECK_NEAR(got, 2 * kPiD * 15, 0.5);
            path = true;
        }
    CHECK(path);
    const StudySeries* sp = seriesNamed(*d.study, "Cylinder 1 (15, 0, 5) speed");
    REQUIRE(sp != nullptr);
    CHECK_NEAR(sp->v[200], 2 * kPiD * 15, 0.5);
    // Refusals.
    CHECK(!run(d, "STUDY \"No such motion\"").ok);
    CHECK(!run(d, "STUDY POINT \"No body\"").ok);
    CHECK(!run(d, "STUDY SAMPLES 2").ok);
    Document empty;
    CHECK(!run(empty, "STUDY").ok);
}

TEST_CASE("STUDY: a cam follower's contact force on its spring, where it lifts off and the spring it needs") {
    Document d;
    eccentricCam(d, 360);
    // 50 kg on a 2 N preload: at the top of its travel it decelerates at 20 pi^2 mm/s2, wanting
    // 50 x 0.197 = 9.87 N the preload can't give; the spring must make up 7.87 N over 10 mm.
    REQUIRE(run(d, "MOTION LINEAR \"Follower\" AXIS Y CONTACT \"Cylinder 1\" PRELOAD 2 MASS 50 BODIES \"Block 2\"").ok);
    REQUIRE(run(d, "STUDY Follower TO 1 SAMPLES 801").ok);
    const StudySeries* f = seriesNamed(*d.study, "Follower contact force");
    REQUIRE(f != nullptr);
    CHECK(f->unit == "N");
    std::string lift;
    for (const std::string& n : d.study->notes)
        if (n.find("lifts off") != std::string::npos) lift = n;
    REQUIRE(!lift.empty());
    // Off where 9.87 sin(turn) > 2: one span a turn, from 0.0325 to 0.4675 s.
    const std::size_t between = lift.find("between ");
    REQUIRE(between != std::string::npos);
    std::istringstream span(lift.substr(between + 8));
    double t0 = 0, t1 = 0;
    std::string word;
    span >> t0 >> word >> t1;
    CHECK_NEAR(t0, 0.0325, 0.006);
    CHECK_NEAR(t1, 0.4675, 0.006);
    CHECK(lift.find("more time") == std::string::npos); // one turn studied: once
    const std::size_t at = lift.find("at least ");
    REQUIRE(at != std::string::npos);
    CHECK_NEAR(std::stod(lift.substr(at + 9)), 7.87 / 10, 0.06);
    // At the bottom (t 0.75) it presses with preload + m a = 2 + 9.87 N.
    CHECK_NEAR(f->v[600], 2 + 50 * 20 * kPiD * kPiD * 1e-3, 0.6);
    // With that spring (and a little over) it stays on.
    REQUIRE(run(d, "MOTION LINEAR \"Follower\" SPRING 0.9").ok);
    CHECK_NEAR(motionNamed(d, "Follower")->spring, 0.9, 1e-6);
    REQUIRE(run(d, "STUDY Follower TO 1 SAMPLES 801").ok);
    bool stays = false;
    for (const std::string& n : d.study->notes) stays = stays || n.find("stays on Cylinder 1") != std::string::npos;
    CHECK(stays);
    // Its mass by default: the follower block's steel (2000 mm3 x 7.85 g/cm3).
    REQUIRE(run(d, "MOTION LINEAR \"Follower\" MASS 0").ok);
    REQUIRE(run(d, "STUDY Follower TO 1").ok);
    bool steel = false;
    for (const std::string& n : d.study->notes) steel = steel || n.find("0.0157 kg moving") != std::string::npos;
    CHECK(steel);
}

TEST_CASE("STUDY writes a CSV table and an SVG chart; contact motions are saved with the project") {
    Document d;
    eccentricCam(d, 90);
    REQUIRE(run(d, "MOTION LINEAR \"Follower\" AXIS Y CONTACT \"Cylinder 1\" SPRING 1.5 PRELOAD 4 MASS 0.2 BODIES \"Block 2\"").ok);
    const auto dir = std::filesystem::temp_directory_path();
    const std::string csv = (dir / "sicdm_study_test.csv").string(), svg = (dir / "sicdm_study_test.svg").string();
    REQUIRE(run(d, "STUDY Follower TO 4 STEP 0.01 CSV \"" + csv + "\" SVG \"" + svg + "\" TITLE \"Cam & follower\"").ok);
    CHECK(d.studyCommand.rfind("STUDY Follower", 0) == 0);
    {
        std::ifstream c(csv); // closed before the file is removed (Windows won't remove an open file)
        std::string header, line;
        std::getline(c, header);
        CHECK(header == "t (s),Follower travel (mm),Follower velocity (mm/s),Follower acceleration (mm/s2),Follower contact force (N)");
        std::size_t rows = 0;
        while (std::getline(c, line)) ++rows;
        CHECK(rows == 401);
    }
    std::stringstream s;
    s << std::ifstream(svg).rdbuf();
    const std::string chart = s.str();
    CHECK(chart.rfind("<svg", 0) == 0 || chart.find("<svg") != std::string::npos);
    CHECK(chart.find("Cam &amp; follower") != std::string::npos);
    CHECK(chart.find("</svg>") != std::string::npos);
    std::error_code ec;
    std::filesystem::remove(csv, ec);
    std::filesystem::remove(svg, ec);

    const std::string path = (dir / "sicdm_study_test.sicdm").string();
    std::string err;
    REQUIRE(saveProject(d.scene, d.state, path, &err));
    Scene sc;
    ProjectState st;
    REQUIRE(loadProject(sc, st, path, &err, nullptr));
    const Motion* m = nullptr;
    for (const Motion& x : sc.motions())
        if (x.name == "Follower") m = &x;
    REQUIRE(m != nullptr);
    CHECK(m->contact == "Cylinder 1");
    CHECK_NEAR(m->spring, 1.5, 1e-6);
    CHECK_NEAR(m->preload, 4, 1e-6);
    CHECK_NEAR(m->mass, 0.2, 1e-6);
    std::filesystem::remove(path, ec);
}

TEST_CASE("STUDY over MCP, and a contact motion made over MCP") {
    Document m;
    McpServer s(m, nullptr);
    const auto tool = [&](const std::string& name, const std::string& args) {
        const std::string msg = R"({"jsonrpc":"2.0","id":3,"method":"tools/call","params":{"name":")" + name + R"(","arguments":)" + args + "}}";
        const auto r = Json::parse(s.handle(msg));
        return r ? (*r)["result"] : Json();
    };
    eccentricCam(m, 360);
    Json r = tool("create_motion", R"({"name":"Follower","kind":"linear","axis":[0,1,0],"contact":"Cylinder 1","spring":0.9,"preload":2,"mass":50,"bodies":["Block 2"]})");
    CHECK(!r["isError"].asBool());
    const Motion* f = motionNamed(m, "Follower");
    REQUIRE(f != nullptr);
    CHECK(f->contact == "Cylinder 1");
    CHECK_NEAR(f->mass, 50, 1e-6);
    r = tool("study", R"({"motions":["Follower"],"to":1,"samples":401})");
    CHECK(!r["isError"].asBool());
    CHECK(r["content"][0]["text"].asString().find("stays on Cylinder 1") != std::string::npos);
    CHECK(tool("study", R"({"motions":["Nothing"]})")["isError"].asBool());
}

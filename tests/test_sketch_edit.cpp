#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Constraints.hpp"
#include "core/Document.hpp"
#include "core/Extrude.hpp"
#include "core/FileIO.hpp"
#include "core/Json.hpp"
#include "core/Mcp.hpp"
#include "core/Sculpt.hpp"
#include "core/SketchEdit.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace sicdm;

#define REQUIRE(c)                  \
    do {                            \
        const bool required_ = (c); \
        CHECK(required_);           \
        if (!required_) return;     \
    } while (0)

namespace {

constexpr double kPiS = 3.14159265358979323846;

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }
std::string tempPath(const char* name) { return (std::filesystem::temp_directory_path() / name).string(); }

const SketchCurve* curve(const Document& d, const std::string& name) {
    for (const SketchCurve& c : d.scene.sketch().curves())
        if (c.name == name) return &c;
    return nullptr;
}
const SketchCurve& last(const Document& d) { return d.scene.sketch().curves().back(); }

bool near(double a, double b, double tol = 1e-3) { return std::fabs(a - b) < tol; }

double stripLength(const SketchCurve& c) {
    const std::vector<Vec2> s = c.planeStrip(0.001f);
    double l = 0;
    for (std::size_t i = 1; i < s.size(); ++i) l += std::hypot(s[i].x - s[i - 1].x, s[i].y - s[i - 1].y);
    return l;
}

double bodyVolume(const Document& d) { return meshVolume(d.scene.bodies().back().mesh); }

} // namespace

TEST_CASE("Sketch: arcs through three points or about a centre, splines through their points") {
    Document d;
    REQUIRE(run(d, "ARC 10,0 0,10 -10,0").ok); // the top half of a circle of radius 10
    const SketchCurve& a = last(d);
    CHECK(a.kind == CurveKind::Arc);
    CHECK(near(a.pts[0].x, 0) && near(a.pts[0].y, 0));
    CHECK(near(a.radius, 10));
    float a0, a1;
    a.arcAngles(a0, a1);
    CHECK(near(a1 - a0, kPiS, 1e-5));
    CHECK(near(stripLength(a), 10 * kPiS, 0.01));
    // The same points the other way round: the same arc.
    REQUIRE(run(d, "ARC -10,0 0,10 10,0").ok);
    CHECK(near(stripLength(last(d)), 10 * kPiS, 0.01));
    CHECK(!run(d, "ARC 0,0 1,1 2,2").ok); // in line
    REQUIRE(run(d, "ARC CENTER 0,0 5,0 0,5").ok); // a quarter, counter-clockwise
    CHECK(near(stripLength(last(d)), 2.5 * kPiS, 0.01));
    // A spline passes through every point it was given.
    REQUIRE(run(d, "SPLINE 0,0 10,8 20,-3 30,5").ok);
    const SketchCurve& s = last(d);
    CHECK(s.kind == CurveKind::Spline);
    const std::vector<Vec2> strip = s.planeStrip(0.01f);
    for (const Vec2& p : s.pts) {
        double best = 1e9;
        for (const Vec2& q : strip) best = std::min(best, static_cast<double>(std::hypot(p.x - q.x, p.y - q.y)));
        CHECK(best < 1e-4);
    }
    CHECK(!s.isClosed());
    REQUIRE(run(d, "SPLINE 0,0 10,0 10,10 0,10 CLOSE").ok);
    CHECK(last(d).isClosed());
    CHECK(!run(d, "SPLINE 0,0").ok);
}

TEST_CASE("Sketch: ellipses, slots and polygons are closed profiles that extrude to the right volume") {
    Document d;
    REQUIRE(run(d, "SKETCHSLOT 0,0 30,0 5").ok);
    REQUIRE(run(d, "EXTRUDE 10").ok);
    CHECK(near(bodyVolume(d), (30 * 10 + kPiS * 25) * 10, 5.0)); // a 30 x 10 rectangle and two half circles
    REQUIRE(run(d, "ELLIPSE 0,50 20,50 8").ok);
    CHECK(near(last(d).radius, 8));
    REQUIRE(run(d, "EXTRUDE 5").ok);
    CHECK(near(bodyVolume(d), kPiS * 20 * 8 * 5, 3.0));
    // A hexagon across flats 20 (circumscribed about a circle of radius 10).
    REQUIRE(run(d, "POLYGON 0,100 10 6 CIRCUMSCRIBED").ok);
    const SketchCurve& h = last(d);
    CHECK(h.kind == CurveKind::Polyline);
    CHECK(h.isClosed());
    float ymin = 1e9f, ymax = -1e9f;
    for (const Vec2& p : h.pts) ymin = std::min(ymin, p.y), ymax = std::max(ymax, p.y);
    CHECK(near(ymax - ymin, 20, 1e-3)); // flats at top and bottom
    REQUIRE(run(d, "EXTRUDE 1").ok);
    CHECK(near(bodyVolume(d), 6 * 10 * 10 * std::tan(kPiS / 6), 0.1)); // 2 sqrt(3) r^2
    CHECK(!run(d, "POLYGON 0,0 10 2").ok);
    CHECK(!run(d, "SKETCHSLOT 0,0 0,0 3").ok);
}

TEST_CASE("Sketch: construction curves are no profiles, and are kept in the file") {
    Document d;
    REQUIRE(run(d, "RECT 0,0 10,10 CONSTRUCTION").ok);
    CHECK(last(d).construction);
    auto r = run(d, "EXTRUDE 5");
    CHECK(!r.ok);
    REQUIRE(run(d, "CONSTRUCTION \"Rectangle 1\" OFF").ok);
    CHECK(!last(d).construction);
    REQUIRE(run(d, "EXTRUDE 5").ok);
    REQUIRE(run(d, "CIRCLE 50,0 5").ok);
    REQUIRE(run(d, "CONSTRUCTION \"Circle 2\"").ok);
    REQUIRE(run(d, "SPLINE 0,0 5,5 10,0 CLOSE").ok);
    REQUIRE(run(d, "ARC 0,0 5,5 10,0").ok);
    const std::string path = tempPath("sicdm_sketch_kinds.sicdm");
    std::string err;
    REQUIRE(saveProject(d.scene, d.state, path, &err));
    Scene s;
    ProjectState st;
    REQUIRE(loadProject(s, st, path, &err));
    const auto& cs = s.sketch().curves();
    REQUIRE(cs.size() == 4);
    CHECK(cs[1].construction);
    CHECK(cs[2].kind == CurveKind::Spline && cs[2].closed);
    CHECK(cs[3].kind == CurveKind::Arc && cs[3].pts.size() == 3);
    std::filesystem::remove(path);
}

TEST_CASE("Sketch: trim cuts back to the crossing curves, a circle to an arc, a lone curve away") {
    Document d;
    run(d, "LINE -20,0 20,0");  // Line 1
    run(d, "LINE 0,-10 0,10");  // Line 2, crossing it at 0,0
    REQUIRE(run(d, "TRIM \"Line 1\" AT 10,0").ok);
    // Line 1 goes, Line 3 (its left half) is left.
    CHECK(curve(d, "Line 1") == nullptr);
    const SketchCurve& left = last(d);
    CHECK(left.kind == CurveKind::Line);
    CHECK(near(stripLength(left), 20));
    CHECK(near(std::max(left.pts[0].x, left.pts[1].x), 0));
    // A circle cut by a line through its middle: an arc of half of it.
    run(d, "CIRCLE 0,50 10");
    run(d, "LINE -20,50 20,50");
    const std::string circle = d.scene.sketch().curves()[d.scene.sketch().curves().size() - 2].name;
    REQUIRE(run(d, "TRIM \"" + circle + "\" AT 0,60").ok);
    const SketchCurve& arc = last(d);
    CHECK(arc.kind == CurveKind::Arc);
    CHECK(near(stripLength(arc), 10 * kPiS, 0.01));
    for (const Vec2& p : arc.planeStrip(0.1f)) CHECK(p.y <= 50.0001f); // the bottom half stays
    // A rectangle cut by a line across it: three sides left.
    run(d, "RECT 100,0 140,20");
    run(d, "LINE 120,-10 120,30");
    const std::string rect = d.scene.sketch().curves()[d.scene.sketch().curves().size() - 2].name;
    REQUIRE(run(d, "TRIM \"" + rect + "\" AT 140,10").ok);
    CHECK(last(d).kind == CurveKind::Polyline);
    CHECK(near(stripLength(last(d)), 60));
    // Nothing crossing: the whole curve goes.
    run(d, "LINE 0,-100 10,-100");
    const std::size_t before = d.scene.sketch().curves().size();
    REQUIRE(run(d, "TRIM \"" + last(d).name + "\" AT 5,-100").ok);
    CHECK(d.scene.sketch().curves().size() == before - 1);
    // An ellipse halved: a spline the length of half of it.
    Document e;
    run(e, "ELLIPSE 0,0 20,0 8");
    run(e, "LINE -30,0 30,0");
    REQUIRE(run(e, "TRIM \"Ellipse 1\" AT 0,8").ok);
    CHECK(last(e).kind == CurveKind::Spline);
    CHECK(near(stripLength(last(e)), 46.0, 0.5)); // half of 92.2
    // A 3D body trim still works as before.
    run(e, "BOX 10 10 10");
    CHECK(run(e, "TRIM \"" + e.scene.bodies().back().name + "\" 0,0,5 NORMAL 0,0,1").ok);
}

TEST_CASE("Sketch: extend carries lines and arcs on to the next curve") {
    Document d;
    run(d, "LINE 30,-30 30,30"); // a wall at x = 30
    run(d, "LINE 0,0 10,0");
    REQUIRE(run(d, "EXTEND \"Line 2\" END").ok);
    CHECK(near(curve(d, "Line 2")->pts[1].x, 30));
    CHECK(near(curve(d, "Line 2")->pts[1].y, 0));
    CHECK(!run(d, "EXTEND \"Line 2\" START").ok); // nothing behind it
    // An arc of radius 10 about the origin, a quarter from 0 to 90 degrees: carried on
    // round to the line y = -5.
    Document e;
    run(e, "LINE -30,-5 30,-5");
    run(e, "ARC CENTER 0,0 10,0 0,10");
    REQUIRE(run(e, "EXTEND \"Arc 2\" START").ok);
    const SketchCurve& a = *curve(e, "Arc 2");
    CHECK(near(a.pts[1].y, -5, 1e-3));
    CHECK(near(std::hypot(a.pts[1].x, a.pts[1].y), 10, 1e-3));
    CHECK(!run(e, "EXTEND \"Line 1\" END").ok);
}

TEST_CASE("Sketch: offset makes parallel lines, concentric circles and arcs, and grown or shrunk loops") {
    Document d;
    run(d, "LINE 0,0 40,0");
    REQUIRE(run(d, "OFFSET \"Line 1\" 5").ok); // left of it: up
    CHECK(near(last(d).pts[0].y, 5) && near(last(d).pts[1].y, 5));
    REQUIRE(run(d, "OFFSET \"Line 1\" 5 SIDE 20,-1").ok);
    CHECK(near(last(d).pts[0].y, -5));
    run(d, "CIRCLE 100,0 10");
    const std::string c = last(d).name;
    REQUIRE(run(d, "OFFSET \"" + c + "\" 3").ok); // outside
    CHECK(near(last(d).radius, 13));
    REQUIRE(run(d, "OFFSET \"" + c + "\" -3").ok); // inside
    CHECK(near(last(d).radius, 7));
    CHECK(!run(d, "OFFSET \"" + c + "\" -12").ok); // would close it up
    run(d, "POLYGON 0,100 10 6");
    const std::string hex = last(d).name;
    REQUIRE(run(d, "OFFSET \"" + hex + "\" 2").ok);
    CHECK(near(stripLength(last(d)), 6 * (10 + 4 * std::tan(kPiS / 6)), 1e-3));
    REQUIRE(run(d, "OFFSET \"" + hex + "\" -2").ok);
    CHECK(near(stripLength(last(d)), 6 * (10 - 4 * std::tan(kPiS / 6)), 1e-3));
    run(d, "RECT 200,0 240,20");
    REQUIRE(run(d, "OFFSET \"" + last(d).name + "\" 1").ok);
    CHECK(near(stripLength(last(d)), 2 * (42 + 22)));
    run(d, "SKETCHSLOT 0,200 30,200 5");
    REQUIRE(run(d, "OFFSET \"" + last(d).name + "\" 1").ok);
    CHECK(near(last(d).radius, 6));
    run(d, "SPLINE 0,300 10,305 20,300 30,305");
    REQUIRE(run(d, "OFFSET \"" + last(d).name + "\" 1").ok);
    CHECK(last(d).kind == CurveKind::Spline);
    // Every point of the offset spline is 1 mm from the original.
    const SketchCurve& orig = d.scene.sketch().curves()[d.scene.sketch().curves().size() - 2];
    const std::vector<Vec2> o = orig.planeStrip(0.005f);
    for (const Vec2& p : last(d).planeStrip(0.05f)) {
        double best = 1e9;
        for (const Vec2& q : o) best = std::min(best, static_cast<double>(std::hypot(p.x - q.x, p.y - q.y)));
        CHECK(best > 0.9 && best < 1.1);
    }
    CHECK(!run(d, "OFFSET \"Nothing\" 2").ok);
}

TEST_CASE("Sketch: lines and arcs joined end to end extrude as one profile") {
    Document d;
    // A slot drawn as two lines and two arcs.
    run(d, "LINE 0,5 30,5");
    run(d, "ARC CENTER 30,0 30,-5 30,5");
    run(d, "LINE 30,-5 0,-5");
    run(d, "ARC CENTER 0,0 0,5 0,-5");
    std::vector<std::size_t> members;
    CHECK(chainLoop(d.scene.sketch(), 0, &members).size() > 4);
    CHECK(members.size() == 4);
    REQUIRE(run(d, "EXTRUDE \"Line 1\" 10").ok);
    CHECK(near(bodyVolume(d), (30 * 10 + kPiS * 25) * 10, 5.0));
    for (const SketchCurve& c : d.scene.sketch().curves()) CHECK(!c.visible); // the loop was used up
    // An open L of lines: no loop.
    Document e;
    run(e, "LINE 0,0 10,0");
    run(e, "LINE 10,0 10,10");
    const auto r = run(e, "EXTRUDE \"Line 1\" 5");
    CHECK(!r.ok);
    CHECK(r.lines.back().find("closed loop") != std::string::npos);
}

TEST_CASE("Sketch: constraints hold arcs, and say which curves are fully constrained") {
    Document d;
    run(d, "ARC CENTER 0,0 10,0 0,10");
    run(d, "LINE 0,10 -20,10");
    REQUIRE(run(d, "DIM \"Arc 1\" 15").ok); // its radius
    CHECK(near(curve(d, "Arc 1")->radius, 15, 1e-3));
    REQUIRE(run(d, "CONSTRAIN COINCIDENT \"Arc 1\" END \"Line 2\" START").ok);
    const SketchCurve& a = *curve(d, "Arc 1");
    const SketchCurve& l = *curve(d, "Line 2");
    CHECK(near(std::hypot(a.pts[2].x - l.pts[0].x, a.pts[2].y - l.pts[0].y), 0, 1e-3));
    // Freedom: the arc and line are still free; a fixed and dimensioned rectangle is not.
    run(d, "RECT 100,0 140,20");
    run(d, "CONSTRAIN FIX \"Rectangle 3\" CORNER 1");
    run(d, "DIM \"Rectangle 3\" SIDE 1 40");
    run(d, "DIM \"Rectangle 3\" SIDE 2 20");
    run(d, "CIRCLE 0,-50 5");
    const std::vector<int> f = curveFreedom(d.scene.sketch());
    REQUIRE(f.size() == 4);
    CHECK(f[0] > 0);
    CHECK(f[1] > 0);
    CHECK(f[2] == 0);
    CHECK(f[3] == -1); // no constraints on it
}

TEST_CASE("Sketch: arcs, slots, splines and ellipses go out to DXF, and arcs, splines and ellipses come back") {
    Document d;
    run(d, "ARC CENTER 0,0 10,0 0,10");
    run(d, "SKETCHSLOT 0,40 30,40 5");
    run(d, "SPLINE 0,80 10,85 20,80 30,85");
    run(d, "ELLIPSE 60,0 80,0 8");
    run(d, "LINE 0,-20 10,-20 CONSTRUCTION");
    const std::string path = tempPath("sicdm_sketch_kinds.dxf");
    REQUIRE(run(d, "EXPORT \"" + path + "\"").ok);
    Document e;
    REQUIRE(run(e, "IMPORT \"" + path + "\"").ok);
    int arcs = 0, lines = 0, polys = 0;
    for (const SketchCurve& c : e.scene.sketch().curves()) {
        arcs += c.kind == CurveKind::Arc;
        lines += c.kind == CurveKind::Line;
        polys += c.kind == CurveKind::Polyline;
    }
    CHECK(arcs == 3);  // the arc and the slot's two ends
    CHECK(lines == 3); // the slot's sides and the construction line
    CHECK(polys == 2); // the spline and the ellipse, as polylines (DXF R12)
    std::filesystem::remove(path);
    // SPLINE and ELLIPSE entities (from newer DXF writers) read as a spline and an ellipse.
    const std::string dxf = tempPath("sicdm_spline_ellipse.dxf");
    {
        std::ofstream f(dxf);
        f << "0\nSECTION\n2\nENTITIES\n"
             "0\nSPLINE\n8\n0\n70\n8\n11\n0\n21\n0\n11\n10\n21\n5\n11\n20\n21\n0\n"
             "0\nELLIPSE\n8\n0\n10\n50\n20\n0\n11\n20\n21\n0\n40\n0.5\n41\n0\n42\n6.283185307\n"
             "0\nENDSEC\n0\nEOF\n";
    }
    Document g;
    REQUIRE(run(g, "IMPORT \"" + dxf + "\"").ok);
    REQUIRE(g.scene.sketch().curves().size() == 2);
    CHECK(g.scene.sketch().curves()[0].kind == CurveKind::Spline);
    CHECK(g.scene.sketch().curves()[0].pts.size() == 3);
    CHECK(g.scene.sketch().curves()[1].kind == CurveKind::Ellipse);
    CHECK(near(g.scene.sketch().curves()[1].radius, 10));
    std::filesystem::remove(dxf);
}

TEST_CASE("MCP create_sketch draws arcs, splines, polygons, slots and construction curves") {
    Document d;
    McpServer s(d, nullptr);
    auto tool = [&](const std::string& args) {
        const auto r = Json::parse(s.handle(R"({"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"create_sketch","arguments":)" + args + "}}"));
        return r ? (*r)["result"] : Json();
    };
    CHECK(!tool(R"({"kind":"arc","points":[[10,0],[0,10],[-10,0]]})")["isError"].asBool());
    CHECK(last(d).kind == CurveKind::Arc);
    CHECK(!tool(R"({"kind":"spline","points":[[0,0],[10,5],[20,0]],"closed":true})")["isError"].asBool());
    CHECK(last(d).closed);
    CHECK(!tool(R"({"kind":"polygon","points":[[0,0]],"radius":10,"sides":8})")["isError"].asBool());
    CHECK(last(d).pts.size() == 9);
    CHECK(!tool(R"({"kind":"slot","points":[[0,0],[20,0]],"radius":3,"construction":true})")["isError"].asBool());
    CHECK(last(d).kind == CurveKind::Slot && last(d).construction);
    CHECK(!tool(R"({"kind":"ellipse","points":[[0,0],[20,0]],"radius":5})")["isError"].asBool());
    CHECK(tool(R"({"kind":"ellipse","points":[[0,0],[20,0]]})")["isError"].asBool()); // no minor radius
}

TEST_CASE("Sketch drag: a point goes where it is dragged, as far as the constraints let it") {
    Document d;
    // A rectangle drawn as a closed polyline: square corners, one corner fixed, its width 40.
    REQUIRE(run(d, "PLINE 0,0 40,0 40,30 0,30 CLOSE").ok);
    for (const char* c : {"HORIZONTAL \"Polyline 1\" SIDE 1", "HORIZONTAL \"Polyline 1\" SIDE 3", "VERTICAL \"Polyline 1\" SIDE 2",
                          "VERTICAL \"Polyline 1\" SIDE 4", "FIX \"Polyline 1\" POINT 1"})
        REQUIRE(run(d, std::string("CONSTRAIN ") + c).ok);
    REQUIRE(run(d, "DIM \"Polyline 1\" SIDE 1 40").ok);
    // The far corner can only rise: the height follows, the width and corner 1 stay.
    const auto r = run(d, "DRAG \"Polyline 1\" POINT 3 TO 60,50");
    REQUIRE(r.ok);
    CHECK(r.lines[0].find("20 mm short") != std::string::npos);
    const SketchCurve& p = *curve(d, "Polyline 1");
    CHECK(near(p.pts[0].x, 0) && near(p.pts[0].y, 0));
    CHECK(near(p.pts[1].x, 40) && near(p.pts[1].y, 0));
    CHECK(near(p.pts[2].x, 40) && near(p.pts[2].y, 50));
    CHECK(near(p.pts[3].x, 0) && near(p.pts[3].y, 50));
    CHECK(near(p.pts[4].x, 0) && near(p.pts[4].y, 0)); // still closed
    for (const SketchConstraint& c : d.scene.sketch().constraints()) CHECK(constraintError(d.scene.sketch(), c) < 1e-5);
    // The fixed corner doesn't move at all.
    REQUIRE(run(d, "DRAG \"Polyline 1\" POINT 1 TO 5,5").ok);
    CHECK(near(curve(d, "Polyline 1")->pts[0].x, 0) && near(curve(d, "Polyline 1")->pts[0].y, 0));
    // A free point lands exactly where it is dragged.
    REQUIRE(run(d, "LINE 100,0 120,0").ok);
    REQUIRE(run(d, "DRAG \"Line 2\" END TO 130.25,10.5").ok);
    CHECK(near(curve(d, "Line 2")->pts[1].x, 130.25, 1e-4) && near(curve(d, "Line 2")->pts[1].y, 10.5, 1e-4));
    CHECK(near(curve(d, "Line 2")->pts[0].x, 100, 1e-4)); // the other end stays
}

TEST_CASE("Sketch drag: joined curves follow, a whole curve moves rigidly, a rim drag sets the radius") {
    Document d;
    REQUIRE(run(d, "LINE 0,0 20,0").ok);
    REQUIRE(run(d, "LINE 20,0 20,15").ok);
    REQUIRE(run(d, "CONSTRAIN COINCIDENT \"Line 1\" END \"Line 2\" START").ok);
    REQUIRE(run(d, "CONSTRAIN PERPENDICULAR \"Line 1\" \"Line 2\"").ok);
    REQUIRE(run(d, "DIM \"Line 2\" 15").ok);
    // Dragging the corner carries the other line along: still joined, square and 15 long.
    REQUIRE(run(d, "DRAG \"Line 1\" END TO 30,5").ok);
    const SketchCurve& a = *curve(d, "Line 1");
    const SketchCurve& b = *curve(d, "Line 2");
    CHECK(near(a.pts[1].x, 30, 1e-3) && near(a.pts[1].y, 5, 1e-3));
    CHECK(near(b.pts[0].x, 30, 1e-3) && near(b.pts[0].y, 5, 1e-3));
    CHECK(near(std::hypot(b.pts[1].x - b.pts[0].x, b.pts[1].y - b.pts[0].y), 15, 1e-3));
    const double dotp = (a.pts[1].x - a.pts[0].x) * (b.pts[1].x - b.pts[0].x) + (a.pts[1].y - a.pts[0].y) * (b.pts[1].y - b.pts[0].y);
    CHECK(near(dotp, 0, 1e-2));
    // A curve nothing joins moves as a whole, keeping its shape, and nothing else moves.
    REQUIRE(run(d, "CIRCLE 0,100 10").ok);
    const Vec2 lineEnd = curve(d, "Line 1")->pts[0];
    REQUIRE(run(d, "DRAG \"Circle 3\" BY 5,-7").ok);
    CHECK(near(curve(d, "Circle 3")->pts[0].x, 5, 1e-4) && near(curve(d, "Circle 3")->pts[0].y, 93, 1e-4));
    CHECK(near(curve(d, "Circle 3")->radius, 10, 1e-4));
    CHECK(near(curve(d, "Line 1")->pts[0].x, lineEnd.x, 1e-6));
    // The rim sets the radius; the centre stays.
    REQUIRE(run(d, "DRAG \"Circle 3\" RIM TO 30,93").ok);
    CHECK(near(curve(d, "Circle 3")->radius, 25, 1e-3));
    CHECK(near(curve(d, "Circle 3")->pts[0].x, 5, 1e-3));
    // A dimensioned radius holds against a rim drag.
    REQUIRE(run(d, "DIM \"Circle 3\" 12").ok);
    REQUIRE(run(d, "DRAG \"Circle 3\" RIM TO 60,93").ok);
    CHECK(near(curve(d, "Circle 3")->radius, 12, 1e-3));
    // A whole constrained curve drags with what it is joined to.
    REQUIRE(run(d, "DRAG \"Line 2\" BY 10,0").ok);
    CHECK(near(curve(d, "Line 2")->pts[0].x, curve(d, "Line 1")->pts[1].x, 1e-3));
    // Refused: no such point, BY on a point, RIM on a line.
    CHECK(!run(d, "DRAG \"Line 1\" POINT 7 TO 1,1").ok);
    CHECK(!run(d, "DRAG \"Line 1\" END BY 1,1").ok);
    CHECK(!run(d, "DRAG \"Line 1\" RIM TO 1,1").ok);
    CHECK(!run(d, "DRAG \"Nope\" TO 1,1").ok);
}

TEST_CASE("Sketch edits undo and redo: new curves, trims, constraints and drags") {
    Document d;
    REQUIRE(run(d, "LINE 0,0 10,0").ok);
    REQUIRE(run(d, "LINE 0,0 0,10").ok);
    REQUIRE(run(d, "DRAG \"Line 1\" END TO 20,5").ok);
    REQUIRE(run(d, "UNDO").ok); // the drag
    CHECK(near(curve(d, "Line 1")->pts[1].x, 10) && near(curve(d, "Line 1")->pts[1].y, 0));
    REQUIRE(run(d, "REDO").ok);
    CHECK(near(curve(d, "Line 1")->pts[1].x, 20) && near(curve(d, "Line 1")->pts[1].y, 5));
    REQUIRE(run(d, "UNDO").ok);
    REQUIRE(run(d, "UNDO").ok); // Line 2
    CHECK(d.scene.sketch().curves().size() == 1);
    REQUIRE(run(d, "CONSTRAIN HORIZONTAL \"Line 1\"").ok);
    CHECK(d.scene.sketch().constraints().size() == 1);
    REQUIRE(run(d, "UNDO").ok);
    CHECK(d.scene.sketch().constraints().empty());
    // A failed sketch command leaves nothing to undo in its name, and a body edit's undo still works.
    CHECK(!run(d, "DRAG \"Line 9\" TO 1,1").ok);
    REQUIRE(run(d, "BOX 10 10 10 @0,50,0").ok);
    REQUIRE(run(d, "HOLE \"Block 1\" AT 0,50,10 DIAMETER 3").ok);
    REQUIRE(run(d, "UNDO").ok);
    REQUIRE(run(d, "UNDO").ok); // the box
    CHECK(d.scene.bodies().empty());
    CHECK(run(d, "UNDO").ok); // Line 1
    CHECK(d.scene.sketch().curves().empty());
    // Erasing a curve undoes, with the constraints on it.
    REQUIRE(run(d, "RECT 0,0 10,5").ok);
    const std::string rect = "\"" + last(d).name + "\"";
    REQUIRE(run(d, "DIM " + rect + " SIDE 1 10").ok);
    REQUIRE(run(d, "ERASE " + rect).ok);
    CHECK(d.scene.sketch().curves().empty());
    REQUIRE(run(d, "UNDO").ok);
    CHECK(d.scene.sketch().curves().size() == 1);
    CHECK(d.scene.sketch().constraints().size() == 1);
}

TEST_CASE("MCP cad drag moves sketch geometry with its constraints held") {
    Document d;
    McpServer s(d, nullptr);
    auto call = [&](const std::string& name, const std::string& args) {
        const auto r = Json::parse(s.handle(R"({"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":")" + name + R"(","arguments":)" + args + "}}"));
        return r ? (*r)["result"] : Json();
    };
    REQUIRE(run(d, "LINE 0,0 10,0").ok);
    REQUIRE(!call("cad", R"({"operation":"constrain","kind":"horizontal","refs":[{"curve":"Line 1"}]})")["isError"].asBool());
    REQUIRE(!call("cad", R"({"operation":"drag","curve":"Line 1","grip":"end","to":[30,8]})")["isError"].asBool());
    CHECK(near(curve(d, "Line 1")->pts[1].x, 30, 1e-3));
    CHECK(near(curve(d, "Line 1")->pts[1].y, curve(d, "Line 1")->pts[0].y, 1e-4)); // still horizontal
    REQUIRE(!call("cad", R"({"operation":"drag","curve":"Line 1","by":[0,5]})")["isError"].asBool());
    CHECK(near(curve(d, "Line 1")->pts[0].x, 0, 1e-3));
    CHECK(call("cad", R"({"operation":"drag","curve":"Line 1"})")["isError"].asBool());
}

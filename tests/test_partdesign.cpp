#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Extrude.hpp"
#include "core/FileIO.hpp"
#include "core/HoleWizard.hpp"
#include "core/Variables.hpp"
#include "core/SheetMetal.hpp"
#include "core/Fea.hpp"
#include "core/Json.hpp"
#include "core/Brep.hpp"
#include "core/Scene.hpp"
#include "core/Sculpt.hpp"
#include "core/TextOutline.hpp"

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

const Body& last(const Document& d) { return d.scene.bodies().back(); }
double volumeOf(const Body& b) { return meshVolume(worldMesh(b)); }
AABB boundsOf(const Body& b) { return worldMesh(b).bounds(); }

} // namespace

TEST_CASE("Datum planes: offset, angled, through three points, by point and normal, on a face") {
    Document d;
    REQUIRE(run(d, "PLANE OFFSET 20 FROM TOP").ok);
    REQUIRE(d.state.datums.size() == 1);
    CHECK(d.state.datums[0].name == "Plane 1");
    CHECK(d.state.plane == SketchPlane::Custom);
    CHECK_NEAR(d.state.customFrame.origin.z, 20.0f, 1e-5);

    // Tipped 90 degrees about its u axis, Top becomes Front.
    REQUIRE(run(d, "PLANE ANGLE 90 ABOUT U FROM TOP NAME Tipped").ok);
    CHECK(sameFrame(d.state.customFrame, planeFrame(SketchPlane::Front)));
    CHECK(d.state.customPlane == "Tipped");

    // A face's plane: level faces come out like Top, upright ones with v up.
    REQUIRE(run(d, "PLANE AT 0,0,0 NORMAL 0,-1,0").ok);
    CHECK(sameFrame(d.state.customFrame, planeFrame(SketchPlane::Front)));
    REQUIRE(run(d, "PLANE AT 5,5,5 NORMAL X").ok);
    PlaneFrame right = planeFrame(SketchPlane::Right);
    right.origin = {5, 5, 5};
    CHECK(sameFrame(d.state.customFrame, right));

    REQUIRE(run(d, "PLANE THROUGH 0,0,0 10,0,10 0,10,0").ok);
    CHECK_NEAR(d.state.customFrame.normal.x, -std::sqrt(0.5f), 1e-5);
    CHECK_NEAR(d.state.customFrame.normal.z, std::sqrt(0.5f), 1e-5);

    REQUIRE(run(d, "BOX 60 40 15 @0,0,0").ok);
    const AABB box = boundsOf(last(d));
    REQUIRE(run(d, "PLANE FACE 10,10," + std::to_string(box.max.z + 7) + " NAME Lid").ok);
    CHECK_NEAR(d.state.customFrame.normal.z, 1.0f, 1e-5);
    CHECK_NEAR(d.state.customFrame.origin.z, box.max.z, 1e-4);
    CHECK_NEAR(d.state.customFrame.origin.x, 10.0f, 1e-4);

    // Back to a datum by name, to a principal plane, and the list.
    REQUIRE(run(d, "PLANE \"Plane 1\"").ok);
    CHECK_NEAR(d.state.customFrame.origin.z, 20.0f, 1e-5);
    REQUIRE(run(d, "PLANE TOP").ok);
    CHECK(d.state.plane == SketchPlane::Top);
    const CommandOutput list = run(d, "PLANE LIST");
    REQUIRE(list.ok);
    CHECK(list.lines.size() == 2 + d.state.datums.size() - 1);

    // Refused: points in line, a name taken, unknown planes.
    CHECK(!run(d, "PLANE THROUGH 0,0,0 1,1,1 2,2,2").ok);
    CHECK(!run(d, "PLANE OFFSET 5 NAME Lid").ok);
    CHECK(!run(d, "PLANE OFFSET 5 NAME front").ok);
    CHECK(!run(d, "PLANE OFFSET 5 FROM Nowhere").ok);
    CHECK(!run(d, "PLANE Nowhere").ok);
    CHECK(!run(d, "PLANE AT 0,0,0").ok);
    REQUIRE(run(d, "PLANE DELETE Lid").ok);
    CHECK(!run(d, "PLANE Lid").ok);
}

TEST_CASE("Sketches on datum planes extrude and revolve there, exactly") {
    Document d;
    REQUIRE(run(d, "PLANE OFFSET 20 FROM TOP").ok);
    REQUIRE(run(d, "RECT 0,0 10,10").ok);
    const SketchCurve& rect = d.scene.sketch().curves().back();
    CHECK(rect.plane == SketchPlane::Custom);
    CHECK_NEAR(rect.frame.origin.z, 20.0f, 1e-6);
    REQUIRE(run(d, "EXTRUDE 5").ok);
    AABB b = boundsOf(last(d));
    CHECK_NEAR(b.min.z, 20.0f, 1e-4);
    CHECK_NEAR(b.max.z, 25.0f, 1e-4);
    CHECK_NEAR(volumeOf(last(d)), 500.0, 1e-2);

    // A circle on a tilted plane: an exact cylinder along the plane's normal.
    REQUIRE(run(d, "PLANE THROUGH 0,0,0 10,0,10 0,10,0").ok);
    REQUIRE(run(d, "CIRCLE 0,0 5").ok);
    REQUIRE(run(d, "EXTRUDE 10").ok);
    REQUIRE(last(d).brep != nullptr);
    CHECK_NEAR(volumeOf(last(d)), 3.14159265 * 25 * 10, 2.0);
    const AABB e = tessellate(*last(d).brep).bounds();
    const AABB m = boundsOf(last(d));
    CHECK_NEAR(e.max.z, m.max.z, 0.05);
    CHECK_NEAR(e.min.x, m.min.x, 0.05);
    // Its axis runs along the normal: the top cap's centre is 10 mm out.
    const Vec3 n = d.state.customFrame.normal;
    const Vec3 mid = (m.min + m.max) * 0.5f;
    CHECK_NEAR(mid.x, n.x * 5, 0.05);
    CHECK_NEAR(mid.z, n.z * 5, 0.05);

    // Revolved on a plane 50 mm up, about its v axis.
    REQUIRE(run(d, "PLANE AT 0,0,50 NORMAL Z NAME High").ok);
    REQUIRE(run(d, "RECT 10,0 20,10").ok);
    REQUIRE(run(d, "REVOLVE \"" + d.scene.sketch().curves().back().name + "\"").ok);
    b = boundsOf(last(d));
    CHECK_NEAR(volumeOf(last(d)), 3.14159265 * (400 - 100) * 10, 15.0);
    CHECK_NEAR(b.min.z, 30.0f, 0.05);
    CHECK_NEAR(b.max.z, 70.0f, 0.05);
    CHECK_NEAR(b.min.y, 0.0f, 0.05);
    CHECK_NEAR(b.max.y, 10.0f, 0.05);
}

TEST_CASE("Datum planes and their sketches are saved, and REGEN rebuilds them") {
    Document d;
    REQUIRE(run(d, "PLANE ANGLE 30 ABOUT V FROM FRONT NAME Slope").ok);
    REQUIRE(run(d, "CIRCLE 5,5 4").ok);
    REQUIRE(run(d, "EXTRUDE 6").ok);
    const double vol = volumeOf(last(d));
    const PlaneFrame slope = d.state.customFrame;
    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_datum_test.sicdm").string();
    REQUIRE(run(d, "SAVE \"" + path + "\"").ok);
    {
        std::ifstream f(path);
        std::stringstream ss;
        ss << f.rdbuf();
        const auto j = Json::parse(ss.str());
        REQUIRE(j.has_value());
        CHECK((*j)["reader_version"].asInt() == 10);
        CHECK((*j)["version"].asInt() == kProjectVersion);
    }
    Document e;
    REQUIRE(run(e, "OPEN \"" + path + "\"").ok);
    REQUIRE(e.state.datums.size() == 1);
    CHECK(e.state.datums[0].name == "Slope");
    CHECK(sameFrame(e.state.datums[0].frame, slope));
    CHECK(e.state.plane == SketchPlane::Custom);
    CHECK(e.state.customPlane == "Slope");
    REQUIRE(e.scene.sketch().curves().size() == 1);
    CHECK(sameFrame(e.scene.sketch().curves()[0].frame, slope));
    REQUIRE(e.scene.bodies().size() == 1);
    CHECK_NEAR(volumeOf(e.scene.bodies()[0]), vol, 1e-3);
    // The history makes the plane again.
    REQUIRE(run(e, "REGEN").ok);
    REQUIRE(e.state.datums.size() == 1);
    CHECK(sameFrame(e.state.datums[0].frame, slope));
    CHECK_NEAR(volumeOf(e.scene.bodies()[0]), vol, 1e-3);
    std::filesystem::remove(path);

    // A history begun on an unnamed custom plane starts with that plane, exactly.
    Document h;
    REQUIRE(makePlaneFrame({1, 2, 3}, {0.3f, -0.4f, 0.8f}, {1, 0, 0}, h.state.customFrame));
    h.state.plane = SketchPlane::Custom;
    REQUIRE(run(h, "RECT 0,0 4,3").ok);
    REQUIRE(h.state.history.size() == 2);
    CHECK(h.state.history[0].line.rfind("PLANE AT ", 0) == 0);
    const PlaneFrame first = h.state.customFrame;
    REQUIRE(run(h, "REGEN").ok);
    REQUIRE(h.scene.sketch().curves().size() == 1);
    CHECK(sameFrame(h.scene.sketch().curves()[0].frame, first));
}

TEST_CASE("Extrusion parameters on a custom plane carry its frame") {
    Profile p;
    p.kind = Profile::Kind::Circle;
    p.radius = 2;
    PlaneFrame f;
    REQUIRE(makePlaneFrame({0, 0, 10}, {0, 1, 1}, {1, 0, 0}, f));
    const std::vector<float> prm = extrudeParams(p, SketchPlane::Custom, 3, false, f);
    CHECK(prm.size() == 16);
    PlaneFrame g;
    std::size_t at = 0;
    REQUIRE(extrusionPlane(prm, g, at));
    CHECK(at == 13);
    CHECK(sameFrame(f, g));
    CHECK(!buildExtrusion(prm).indices.empty());
    std::vector<float> bad = prm;
    bad[0] = 4;
    CHECK(buildExtrusion(bad).indices.empty());
    bad = prm;
    bad.resize(12); // the frame cut short
    CHECK(buildExtrusion(bad).indices.empty());
}

TEST_CASE("EXTRUDE cuts, joins, goes both ways, through all and up to the next face") {
    Document d;
    REQUIRE(run(d, "BOX 60 40 20 @0,0,0").ok);
    const AABB box = boundsOf(last(d));
    const double boxVol = volumeOf(last(d));
    const float cx = (box.min.x + box.max.x) * 0.5f, cy = (box.min.y + box.max.y) * 0.5f;
    // A hole through all from a plane on the top face, downwards.
    REQUIRE(run(d, "PLANE FACE " + std::to_string(cx) + "," + std::to_string(cy) + "," + std::to_string(box.max.z + 1) + " NAME \"Top face\"").ok);
    REQUIRE(run(d, "CIRCLE 0,0 5").ok);
    REQUIRE(run(d, "EXTRUDE THROUGH REVERSE CUT").ok);
    REQUIRE(d.scene.bodies().size() == 1);
    CHECK(openEdgeCount(last(d).mesh) == 0);
    CHECK_NEAR(volumeOf(last(d)), boxVol - 3.14159265 * 25 * (box.max.z - box.min.z), 6.0);
    // A boss joined on the top: one body, more volume.
    REQUIRE(run(d, "RECT -25,-15 -15,-5").ok);
    const double before = volumeOf(last(d));
    REQUIRE(run(d, "EXTRUDE 8 JOIN").ok);
    REQUIRE(d.scene.bodies().size() == 1);
    CHECK_NEAR(volumeOf(last(d)), before + 10 * 10 * 8, 0.5);
    CHECK_NEAR(boundsOf(last(d)).max.z, box.max.z + 8, 1e-3);
    // Nothing to cut: refused, nothing changed.
    REQUIRE(run(d, "RECT 200,200 210,210").ok);
    CHECK(!run(d, "EXTRUDE 5 CUT").ok);
    CHECK(d.scene.bodies().size() == 1);

    // Two directions, and up to a point.
    Document e;
    REQUIRE(run(e, "RECT 0,0 10,10").ok);
    REQUIRE(run(e, "EXTRUDE 6 TWO 4").ok);
    CHECK_NEAR(boundsOf(last(e)).min.z, -4.0f, 1e-4);
    CHECK_NEAR(boundsOf(last(e)).max.z, 6.0f, 1e-4);
    REQUIRE(run(e, "PLANE OFFSET 50 FROM TOP").ok);
    REQUIRE(run(e, "RECT 0,0 10,10").ok);
    REQUIRE(run(e, "EXTRUDE UPTO 0,0,20").ok); // down to z = 20
    CHECK_NEAR(boundsOf(last(e)).min.z, 20.0f, 1e-4);
    CHECK_NEAR(boundsOf(last(e)).max.z, 50.0f, 1e-4);
    // Up to the next face: from z = 50 down onto the first block's top at z = 6.
    REQUIRE(run(e, "PLANE OFFSET 80 FROM TOP").ok);
    REQUIRE(run(e, "RECT 2,2 8,8").ok);
    REQUIRE(run(e, "EXTRUDE UPTO NEXT REVERSE").ok);
    CHECK_NEAR(boundsOf(last(e)).min.z, 50.0f, 1e-3);
    REQUIRE(run(e, "RECT 100,100 110,110").ok);
    CHECK(!run(e, "EXTRUDE UPTO NEXT REVERSE").ok); // nothing below it
}

TEST_CASE("EXTRUDE with a draft, as a thin wall, and from an open curve; REVOLVE cuts a groove") {
    Document d;
    REQUIRE(run(d, "RECT -10,-10 10,10").ok);
    REQUIRE(run(d, "EXTRUDE 10 DRAFT 10").ok);
    const AABB b = boundsOf(last(d));
    const float top = 10 - 10 * std::tan(10 * 3.14159265f / 180);
    CHECK_NEAR(b.max.z, 10.0f, 1e-4);
    CHECK_NEAR(b.max.x, 10.0f, 1e-4);
    // A frustum: (h/3)(A1 + A2 + sqrt(A1 A2)).
    const double a1 = 400, a2 = 4.0 * top * top;
    CHECK_NEAR(volumeOf(last(d)), 10.0 / 3 * (a1 + a2 + std::sqrt(a1 * a2)), 0.05);
    CHECK(last(d).brep != nullptr); // flat faces: exact
    CHECK(openEdgeCount(last(d).mesh) == 0);
    // A draft too steep for the depth.
    REQUIRE(run(d, "RECT 50,0 52,2").ok);
    CHECK(!run(d, "EXTRUDE 20 DRAFT 30").ok);

    // A wall 2 thick inside a 20 x 20 outline, 10 high: (400 - 256) x 10.
    REQUIRE(run(d, "RECT 100,0 120,20").ok);
    REQUIRE(run(d, "EXTRUDE 10 THIN 2").ok);
    CHECK_NEAR(volumeOf(last(d)), 1440.0, 0.05);
    CHECK(openEdgeCount(last(d).mesh) == 0);
    // An open polyline thickened into a rib.
    REQUIRE(run(d, "PLINE 200,0 230,0 230,20").ok);
    REQUIRE(run(d, "EXTRUDE 5 THIN 2").ok);
    CHECK_NEAR(volumeOf(last(d)), (30 * 2 + 20 * 2) * 5.0, 0.1);
    CHECK(openEdgeCount(last(d).mesh) == 0);
    REQUIRE(run(d, "LINE 300,0 310,0").ok);
    CHECK(!run(d, "EXTRUDE 5").ok); // open without THIN

    // A groove turned into a cylinder (revolved cut on the Front plane, about its V axis = Z).
    Document g;
    REQUIRE(run(g, "CYLINDER 20 40").ok);
    const double cyl = volumeOf(last(g));
    REQUIRE(run(g, "PLANE FRONT").ok);
    REQUIRE(run(g, "RECT 17,15 21,20").ok);
    REQUIRE(run(g, "REVOLVE CUT").ok);
    REQUIRE(g.scene.bodies().size() == 1);
    CHECK(openEdgeCount(last(g).mesh) == 0);
    CHECK_NEAR(volumeOf(last(g)), cyl - 3.14159265 * (400 - 289) * 5, 25.0);
}

TEST_CASE("Hole wizard: standard sizes, counterbores, countersinks, tapped holes and their callouts") {
    // The tables.
    const HoleSize* m6 = findHoleSize("m6");
    REQUIRE(m6 != nullptr);
    CHECK_NEAR(m6->tapDrill, 5.0f, 1e-6);
    CHECK_NEAR(m6->clearance[1], 6.6f, 1e-6);
    CHECK(findHoleSize("M6x1") == m6);
    CHECK(findHoleSize("M6x0.75") == nullptr); // fine threads aren't in the table
    const HoleSize* q = findHoleSize("1/4-20");
    REQUIRE(q != nullptr);
    CHECK(q->inch);
    CHECK_NEAR(q->pitch, 1.27f, 1e-5);
    CHECK(findHoleSize("10-24") == findHoleSize("#10-24"));
    CHECK(findHoleSize("M7") == nullptr);

    // The shape: a counterbore's step, a countersink's cone, a drill point.
    std::vector<std::pair<float, float>> prof;
    HoleSpec h;
    h.diameter = 6.6f, h.cboreDia = 11, h.cboreDepth = 6.4f;
    REQUIRE(holeProfile(h, 100, 1, prof));
    CHECK(prof.size() == 6);
    CHECK_NEAR(prof[2].second, -6.4f, 1e-6);
    CHECK_NEAR(prof[3].first, 5.5f, 1e-6);
    h.csinkDia = 13;
    CHECK(!holeProfile(h, 100, 1, prof)); // not both
    h.cboreDia = 0, h.cboreDepth = 0, h.depth = 2;
    CHECK(!holeProfile(h, 100, 1, prof)); // shallower than its countersink

    Document d;
    REQUIRE(run(d, "BOX 80 60 20 @0,0,0").ok);
    const double v0 = volumeOf(last(d));
    REQUIRE(run(d, "HOLE \"Block 1\" AT -25,-15,20 SIZE M6 CBORE THROUGH").ok);
    const double cb = 3.14159265 * (3.3 * 3.3 * 20 + (5.5 * 5.5 - 3.3 * 3.3) * 6.4);
    CHECK_NEAR(v0 - volumeOf(last(d)), cb, cb * 0.01);
    REQUIRE(run(d, "HOLE \"Block 1\" AT -25,15,20 25,15,20 SIZE M8 TAPPED DEPTH 15").ok);
    REQUIRE(last(d).holes.size() == 3);
    CHECK(last(d).holes[1].callout == "M8 tapped 12.5 deep (drill 6.8 x 15)");
    CHECK_NEAR(last(d).holes[2].at.x, 25.0f, 1e-3);
    CHECK_NEAR(last(d).holes[2].dir.z, -1.0f, 1e-4);
    CHECK(openEdgeCount(last(d).mesh) == 0);
    const CommandOutput list = run(d, "HOLE LIST");
    REQUIRE(list.ok);
    CHECK(list.lines.size() == 3);
    // Refused: unknown sizes, tapping without a size, a thread deeper than the hole.
    CHECK(!run(d, "HOLE \"Block 1\" AT 0,0,20 SIZE M7").ok);
    CHECK(!run(d, "HOLE \"Block 1\" AT 0,0,20 DIAMETER 5 TAPPED").ok);
    CHECK(!run(d, "HOLE \"Block 1\" AT 0,0,20 SIZE M5 TAPPED DEPTH 8 THREAD 10").ok);
    CHECK(last(d).holes.size() == 3);
    // The callouts are saved with the body.
    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_holes_test.sicdm").string();
    REQUIRE(run(d, "SAVE \"" + path + "\"").ok);
    Document e;
    REQUIRE(run(e, "OPEN \"" + path + "\"").ok);
    REQUIRE(e.scene.bodies().size() == 1);
    REQUIRE(e.scene.bodies()[0].holes.size() == 3);
    CHECK(e.scene.bodies()[0].holes[0].callout == "dia 6.6 thru (M6 clearance), counterbore 11 x 6.4");
    std::filesystem::remove(path);
}

namespace {
const SketchCurve* curve(const Document& d, const std::string& name) {
    for (const SketchCurve& c : d.scene.sketch().curves())
        if (c.name == name) return &c;
    return nullptr;
}
} // namespace

TEST_CASE("Sketch fillets and chamfers: every corner, chosen corners, two lines") {
    Document d;
    REQUIRE(run(d, "RECT 0,0 40,20").ok);
    REQUIRE(run(d, "SKETCHFILLET \"Rectangle 1\" 5").ok);
    CHECK(curve(d, "Rectangle 1") == nullptr);
    int arcs = 0, lines = 0;
    for (const SketchCurve& c : d.scene.sketch().curves()) arcs += c.kind == CurveKind::Arc, lines += c.kind == CurveKind::Line;
    CHECK(arcs == 4);
    CHECK(lines == 4);
    // The pieces close into a loop: a rounded slab, 40 x 20 less the four corners.
    REQUIRE(run(d, "EXTRUDE \"Line 2\" 10").ok);
    CHECK_NEAR(volumeOf(last(d)), (800 - (4 - 3.14159265) * 25) * 10, 8.0); // arcs as facets

    // Two corners of a rectangle bevelled.
    REQUIRE(run(d, "RECT 100,0 140,20").ok);
    REQUIRE(run(d, "SKETCHCHAMFER \"Rectangle 10\" CORNER 1 CORNER 3 4").ok);
    const std::size_t n = d.scene.sketch().curves().size();
    REQUIRE(run(d, "EXTRUDE \"" + d.scene.sketch().curves()[n - 1].name + "\" 1").ok);
    CHECK_NEAR(volumeOf(last(d)), 800 - 2 * 8, 1e-3);
    // Two lines meeting at a corner: both trimmed back 6 mm, an arc between.
    REQUIRE(run(d, "LINE 200,0 240,0").ok);
    REQUIRE(run(d, "LINE 200,30 200,0").ok);
    const std::string l1 = d.scene.sketch().curves()[d.scene.sketch().curves().size() - 2].name, l2 = d.scene.sketch().curves().back().name;
    REQUIRE(run(d, "SKETCHFILLET \"" + l1 + "\" \"" + l2 + "\" 6").ok);
    const SketchCurve& arc = d.scene.sketch().curves().back();
    CHECK(arc.kind == CurveKind::Arc);
    CHECK_NEAR(arc.pts[0].x, 206.0f, 1e-4);
    CHECK_NEAR(arc.pts[0].y, 6.0f, 1e-4);
    CHECK_NEAR(curve(d, l1)->pts[0].x, 206.0f, 1e-4);
    CHECK_NEAR(curve(d, l2)->pts[1].y, 6.0f, 1e-4);
    // Too big for the sides.
    REQUIRE(run(d, "RECT 300,0 310,10").ok);
    CHECK(!run(d, "SKETCHFILLET \"" + d.scene.sketch().curves().back().name + "\" 6").ok);
    CHECK(!run(d, "SKETCHFILLET \"Circle 99\" 2").ok);
}

TEST_CASE("Sketch mirror and patterns, and projecting a body onto the sketch plane") {
    Document d;
    REQUIRE(run(d, "LINE 0,-10 0,50 CONSTRUCTION").ok);
    REQUIRE(run(d, "ARC CENTER 10,10 14,10 10,14").ok); // a quarter, counter-clockwise
    REQUIRE(run(d, "SKETCHMIRROR \"Arc 2\" ABOUT \"Line 1\"").ok);
    const SketchCurve& m = d.scene.sketch().curves().back();
    REQUIRE(m.kind == CurveKind::Arc);
    CHECK_NEAR(m.pts[0].x, -10.0f, 1e-5); // centre across the line
    // Still counter-clockwise: from (-10, 14) round to (-14, 10).
    CHECK_NEAR(m.pts[1].x, -10.0f, 1e-5);
    CHECK_NEAR(m.pts[1].y, 14.0f, 1e-5);
    CHECK_NEAR(m.pts[2].x, -14.0f, 1e-5);
    // A tilted rectangle comes out as a closed polyline.
    REQUIRE(run(d, "RECT 20,0 30,5").ok);
    REQUIRE(run(d, "SKETCHMIRROR \"Rectangle 4\" ABOUT 0,0 10,5").ok);
    CHECK(d.scene.sketch().curves().back().kind == CurveKind::Polyline);

    REQUIRE(run(d, "CIRCLE 50,0 2").ok);
    const std::size_t before = d.scene.sketch().curves().size();
    REQUIRE(run(d, "SKETCHPATTERN \"Circle 6\" CIRCULAR 6 ABOUT 40,0").ok);
    REQUIRE(d.scene.sketch().curves().size() == before + 5);
    const SketchCurve& third = d.scene.sketch().curves()[before + 2]; // 180 degrees round
    CHECK_NEAR(third.pts[0].x, 30.0f, 1e-4);
    CHECK_NEAR(third.pts[0].y, 0.0f, 1e-4);
    REQUIRE(run(d, "SKETCHPATTERN \"Circle 6\" LINEAR 3 0,12 ROWS 2 20,0").ok);
    CHECK(d.scene.sketch().curves().size() == before + 10);
    CHECK_NEAR(d.scene.sketch().curves().back().pts[0].x, 70.0f, 1e-4);
    CHECK_NEAR(d.scene.sketch().curves().back().pts[0].y, 24.0f, 1e-4);
    CHECK(!run(d, "SKETCHPATTERN \"Circle 6\" LINEAR 1 5,0").ok);

    // The Front plane cuts a block in a square; its edges seen from the front are 4 lines.
    REQUIRE(run(d, "BOX 30 30 30 @500,0,0").ok);
    REQUIRE(run(d, "PLANE FRONT").ok);
    const std::size_t c0 = d.scene.sketch().curves().size();
    REQUIRE(run(d, "PROJECT \"Block 1\" SECTION").ok);
    REQUIRE(d.scene.sketch().curves().size() == c0 + 1);
    const SketchCurve& sec = d.scene.sketch().curves().back();
    CHECK(sec.plane == SketchPlane::Front);
    CHECK(sec.isClosed());
    std::string why;
    const auto prof = profileFromCurve(sec, &why);
    REQUIRE(prof.has_value());
    CHECK_NEAR(std::fabs(signedArea(prof->points)), 900.0f, 1e-2);
    REQUIRE(run(d, "PROJECT \"Block 1\" EDGES CONSTRUCTION").ok);
    CHECK(d.scene.sketch().curves().size() == c0 + 5);
    CHECK(d.scene.sketch().curves().back().construction);
    REQUIRE(run(d, "PLANE OFFSET 40 FROM FRONT").ok);
    CHECK(!run(d, "PROJECT \"Block 1\" SECTION").ok); // misses it
}

TEST_CASE("DRAFT tilts a body's side faces about the neutral plane") {
    Document d;
    REQUIRE(run(d, "BOX 40 40 20 @0,0,0").ok);
    REQUIRE(run(d, "DRAFT \"Block 1\" ANGLE 5").ok);
    const double t = 40 - 2 * 20 * std::tan(5 * 3.14159265 / 180);
    CHECK_NEAR(volumeOf(last(d)), 20.0 / 3 * (1600 + t * t + 40 * t), 0.05);
    CHECK(openEdgeCount(last(d).mesh) == 0);
    const AABB b = boundsOf(last(d));
    CHECK_NEAR(b.max.x, 20.0f, 1e-3); // the bottom keeps its size
    // About a neutral plane in the middle: wider below it, narrower above.
    REQUIRE(run(d, "BOX 40 40 20 @100,0,0").ok);
    REQUIRE(run(d, "DRAFT \"Block 2\" ANGLE 5 NEUTRAL 0,0,10").ok);
    CHECK_NEAR(boundsOf(last(d)).max.x, 100 + 20 + 10 * std::tan(5 * 3.14159265f / 180), 1e-3);
    // Along X: the faces across X stay, the others tilt.
    REQUIRE(run(d, "BOX 20 20 20 @200,0,0").ok);
    REQUIRE(run(d, "DRAFT \"Block 3\" ANGLE 4 PULL X").ok);
    CHECK(openEdgeCount(last(d).mesh) == 0);
    CHECK(volumeOf(last(d)) < 8000);
    // Refused: folding over, nothing to draft, angle out of range.
    REQUIRE(run(d, "BOX 10 10 50 @300,0,0").ok);
    CHECK(!run(d, "DRAFT \"Block 4\" ANGLE 10").ok);
    REQUIRE(run(d, "SPHERE 10 @400,0,0").ok);
    CHECK(!run(d, "DRAFT \"Sphere 5\" ANGLE 3").ok);
    CHECK(!run(d, "DRAFT \"Block 1\" ANGLE 60").ok);
}

TEST_CASE("Helical sweeps (springs) and modelled threads, outside and in tapped holes") {
    Document d;
    REQUIRE(run(d, "PLANE FRONT").ok);
    REQUIRE(run(d, "CIRCLE 20,0 2").ok);
    REQUIRE(run(d, "SWEEP \"Circle 1\" HELIX PITCH 8 TURNS 5").ok);
    // Pappus: the section's area times the helix's length.
    const double helixLen = 5 * std::sqrt(std::pow(2 * 3.14159265 * 20, 2) + 64);
    CHECK_NEAR(volumeOf(last(d)), 3.14159265 * 4 * helixLen, 3.14159265 * 4 * helixLen * 0.01);
    CHECK(openEdgeCount(last(d).mesh) == 0);
    const AABB s = boundsOf(last(d));
    CHECK_NEAR(s.max.z - s.min.z, 5 * 8 + 4.0f, 0.05);
    REQUIRE(run(d, "CIRCLE 2,0 3").ok);
    CHECK(!run(d, "SWEEP \"Circle 2\" HELIX PITCH 8 TURNS 2").ok); // crosses the axis
    REQUIRE(run(d, "CIRCLE 40,0 3").ok);
    CHECK(!run(d, "SWEEP \"Circle 3\" HELIX PITCH 4 TURNS 2").ok); // coils run into each other

    // An M10 thread 20 long on a rod: about the groove's area round its centre, turn by turn.
    REQUIRE(run(d, "CYLINDER 5 30 @100,0,0").ok);
    const double rod = volumeOf(last(d));
    const std::string cyl = "\"" + last(d).name + "\"";
    REQUIRE(run(d, "THREAD " + cyl + " AT 100,0,30 AXIS 0,0,-1 SIZE M10 LENGTH 20").ok);
    const double P = 1.5, groove = (7.0 / 8 + 1.0 / 4) / 2 * 0.6134 * P * P;
    CHECK_NEAR(rod - volumeOf(last(d)), groove * 2 * 3.14159265 * (5 - 0.6134 * P / 2) * (20 / P), 25.0);
    CHECK(openEdgeCount(last(d).mesh) == 0);
    CHECK(!run(d, "THREAD " + cyl + " AT 100,0,30 AXIS Z DIAMETER 2 PITCH 3 LENGTH 5").ok); // too coarse

    // The wizard's tapped holes, threaded in place; a second time there's nothing left to do.
    REQUIRE(run(d, "BOX 40 40 20 @200,0,0").ok);
    const std::string blk = "\"" + last(d).name + "\"";
    REQUIRE(run(d, "HOLE " + blk + " AT 200,0,20 SIZE M8 TAPPED DEPTH 16").ok);
    const double drilled = volumeOf(last(d));
    REQUIRE(run(d, "THREAD " + blk + " HOLES").ok);
    CHECK(volumeOf(last(d)) < drilled - 50);
    CHECK(openEdgeCount(last(d).mesh) == 0);
    CHECK(last(d).holes[0].threadModelled);
    CHECK(!run(d, "THREAD " + blk + " HOLES").ok);
    CHECK(!run(d, "THREAD " + blk + " HOLE 2").ok);
}

TEST_CASE("Variables and equations drive the model; configurations and design tables switch it") {
    // The expressions.
    std::vector<Variable> vars = {{"w", "60"}, {"d", "w / 2"}, {"h", "max(10, d / 3)"}, {"a", "2 ^ 3 ^ 2"}};
    CHECK_NEAR(*evalExpression("d + 1", vars), 31.0, 1e-12);
    CHECK_NEAR(*evalExpression("h", vars), 10.0, 1e-12);
    CHECK_NEAR(*evalExpression("a", vars), 512.0, 1e-9); // right to left
    CHECK_NEAR(*evalExpression("sin(30) + -(-2) * pi", vars), 0.5 + 2 * 3.14159265358979, 1e-9);
    CHECK(!evalExpression("1 / (w - 60)", vars));
    CHECK(!evalExpression("w +", vars));
    CHECK(!evalExpression("nosuch * 2", vars));
    vars.push_back({"p", "q"}), vars.push_back({"q", "p + 1"});
    CHECK(!evalExpression("p", vars)); // a circle
    CHECK(!validVariableName("sin"));
    CHECK(!validVariableName("2x"));
    std::string ex;
    REQUIRE(expandVariables("BOX {w} {d} 10 NAME \"{not this}\"", vars, ex));
    CHECK(ex == "BOX 60 30 10 NAME \"{not this}\"");
    CHECK(!expandVariables("BOX {w 1 1", vars, ex));

    Document doc;
    REQUIRE(run(doc, "VAR w = 60").ok);
    REQUIRE(run(doc, "VAR d = w / 2").ok);
    REQUIRE(run(doc, "VAR r = 4").ok);
    REQUIRE(run(doc, "BOX {w} {d} 10 @0,0,0").ok);
    REQUIRE(run(doc, "HOLE \"Block 1\" AT 0,0,10 DIAMETER {r*2} THROUGH").ok);
    CHECK(doc.state.history[0].line == "BOX {w} {d} 10 @0,0,0"); // the history keeps the braces
    CHECK_NEAR(volumeOf(last(doc)), 60 * 30 * 10 - 3.14159265 * 16 * 10, 5.0);
    // Changing one rebuilds the model.
    REQUIRE(run(doc, "VAR w = 100").ok);
    REQUIRE(doc.scene.bodies().size() == 1);
    CHECK_NEAR(volumeOf(last(doc)), 100 * 50 * 10 - 3.14159265 * 16 * 10, 5.0);
    // A change that would break the model is refused, the model and variables kept.
    CHECK(!run(doc, "VAR r = 80").ok); // a hole wider than the block
    CHECK(findVariable(doc.state.variables, "r")->expr == "4");
    CHECK_NEAR(volumeOf(last(doc)), 100 * 50 * 10 - 3.14159265 * 16 * 10, 5.0);
    CHECK(!run(doc, "VAR w = d").ok); // a circle
    CHECK(!run(doc, "VAR DELETE r").ok); // the history uses it

    // Configurations and a design table.
    REQUIRE(run(doc, "CONFIG SAVE base").ok);
    const std::string csv = (std::filesystem::temp_directory_path() / "sicdm_design_table.csv").string();
    {
        std::ofstream f(csv);
        f << "name,w,d,r\nsmall,40,20,3\nlarge,80,40,6\n";
    }
    REQUIRE(run(doc, "CONFIG TABLE \"" + csv + "\"").ok);
    std::filesystem::remove(csv);
    REQUIRE(doc.state.configs.size() == 3);
    REQUIRE(run(doc, "CONFIG small").ok);
    CHECK_NEAR(volumeOf(last(doc)), 40 * 20 * 10 - 3.14159265 * 9 * 10, 5.0);
    CHECK(doc.state.config == "small");
    REQUIRE(run(doc, "CONFIG base").ok);
    CHECK_NEAR(volumeOf(last(doc)), 100 * 50 * 10 - 3.14159265 * 16 * 10, 5.0);
    CHECK(!run(doc, "CONFIG nosuch").ok);

    // Saved with the project.
    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_vars_test.sicdm").string();
    REQUIRE(run(doc, "SAVE \"" + path + "\"").ok);
    Document e;
    REQUIRE(run(e, "OPEN \"" + path + "\"").ok);
    std::filesystem::remove(path);
    CHECK(e.state.variables.size() == 3);
    CHECK(e.state.configs.size() == 3);
    CHECK(e.state.config == "base");
    REQUIRE(run(e, "CONFIG large").ok);
    CHECK_NEAR(volumeOf(last(e)), 80 * 40 * 10 - 3.14159265 * 36 * 10, 8.0);
}

TEST_CASE("PATTERN along a curve, at sketch points, and filling a region") {
    Document d;
    REQUIRE(run(d, "CYLINDER 2 5 @0,0,0").ok);
    REQUIRE(run(d, "ARC CENTER 0,40 0,0 40,40").ok); // a quarter circle from (0,0) to (40,40)
    REQUIRE(run(d, "PATTERN \"Cylinder 1\" ALONG \"Arc 1\" 5").ok);
    REQUIRE(d.scene.bodies().size() == 5);
    CHECK_NEAR(d.scene.bodies()[4].offset.x, 40.0f, 0.05);
    CHECK_NEAR(d.scene.bodies()[4].offset.y, 40.0f, 0.05);
    // The middle one halfway round: 45 degrees.
    CHECK_NEAR(d.scene.bodies()[2].offset.x, 40 * std::sin(3.14159265f / 4), 0.05);
    CHECK(!run(d, "PATTERN \"Cylinder 1\" ALONG \"Arc 1\"").ok); // how many?

    Document p;
    for (const char* q : {"POINT 100,0", "POINT 110,10", "POINT 120,0"}) REQUIRE(run(p, q).ok);
    REQUIRE(run(p, "CYLINDER 2 5 @100,0,0").ok);
    REQUIRE(run(p, "PATTERN \"Cylinder 1\" POINTS").ok);
    REQUIRE(p.scene.bodies().size() == 3); // the original already sits on the first
    CHECK_NEAR(p.scene.bodies()[1].offset.x, 110.0f, 1e-4);
    CHECK_NEAR(p.scene.bodies()[1].offset.y, 10.0f, 1e-4);
    CHECK_NEAR(p.scene.bodies()[1].offset.z, 0.0f, 1e-4);

    // 24 holes filling a 60 x 40 rectangle 10 apart, 3 in from its edge, cut through a plate.
    Document f;
    REQUIRE(run(f, "RECT 200,0 260,40").ok);
    REQUIRE(run(f, "BOX 70 50 5 @230,20,-5").ok);
    REQUIRE(run(f, "CYLINDER 2 10 @205,5,-5").ok);
    REQUIRE(run(f, "PATTERN \"Cylinder 2\" FILL \"Rectangle 1\" SPACING 10 MARGIN 3 SUBTRACT \"Block 1\"").ok);
    REQUIRE(f.scene.bodies().size() == 1);
    CHECK_NEAR(volumeOf(last(f)), 70 * 50 * 5 - 24 * 3.14159265 * 4 * 5, 15.0);
    CHECK(openEdgeCount(last(f).mesh) == 0);
    // Staggered rows fit differently.
    Document g;
    REQUIRE(run(g, "RECT 200,0 260,40").ok);
    REQUIRE(run(g, "CYLINDER 2 5 @205,5,0").ok);
    REQUIRE(run(g, "PATTERN \"Cylinder 1\" FILL \"Rectangle 1\" SPACING 10 STAGGER MARGIN 3").ok);
    CHECK(g.scene.bodies().size() == 22);
    CHECK(!run(g, "PATTERN \"Cylinder 1\" FILL \"Rectangle 1\"").ok); // SPACING
}

TEST_CASE("Reference axes and points, used by planes, patterns and revolves; SPLIT") {
    Document d;
    REQUIRE(run(d, "CYLINDER 10 40 @0,0,0").ok);
    REQUIRE(run(d, "ROTATE \"Cylinder 1\" 0,90,0").ok); // lying along X
    REQUIRE(run(d, "REFAXIS A1 OF \"Cylinder 1\"").ok);
    const DatumAxis* a1 = findRefAxis(d.state, "a1");
    REQUIRE(a1 != nullptr);
    CHECK_NEAR(std::fabs(a1->dir.x), 1.0f, 1e-4);
    REQUIRE(run(d, "REFPOINT P1 CENTRE \"Cylinder 1\"").ok);
    CHECK_NEAR(findRefPoint(d.state, "P1")->at.x, 20.0f, 1e-3);
    REQUIRE(run(d, "REFAXIS Z0 PLANES FRONT RIGHT").ok); // the Z axis
    CHECK_NEAR(std::fabs(findRefAxis(d.state, "Z0")->dir.z), 1.0f, 1e-6);
    CHECK(!run(d, "REFAXIS Bad PLANES TOP TOP").ok);
    CHECK(!run(d, "REFAXIS A1 THROUGH 0,0,0 1,0,0").ok); // name taken

    // A plane through the point square to the axis, and a split there.
    REQUIRE(run(d, "PLANE AT P1 NORMAL A1 NAME Mid").ok);
    CHECK_NEAR(std::fabs(d.state.customFrame.normal.x), 1.0f, 1e-4);
    const double whole = volumeOf(last(d));
    REQUIRE(run(d, "SPLIT \"Cylinder 1\" BY Mid").ok);
    REQUIRE(d.scene.bodies().size() == 2);
    CHECK_NEAR(volumeOf(d.scene.bodies()[0]) + volumeOf(d.scene.bodies()[1]), whole, whole * 1e-3);
    CHECK_NEAR(volumeOf(d.scene.bodies()[0]), whole / 2, whole * 1e-2);
    CHECK(d.scene.bodies()[1].name == "Cylinder 1 (2)");
    CHECK(!run(d, "SPLIT \"Cylinder 1\" 500,0,0 NORMAL X").ok); // misses it

    // A circular pattern about the reference axis, and a revolve about one in the sketch plane.
    REQUIRE(run(d, "BOX 5 5 5 @30,0,100").ok);
    REQUIRE(run(d, "PATTERN \"Block 2\" CIRCULAR 4 AXIS Z0").ok);
    CHECK_NEAR(d.scene.bodies().back().offset.y, -30.0f, 1e-3);
    REQUIRE(run(d, "PLANE FRONT").ok);
    REQUIRE(run(d, "REFAXIS V0 AT 0,0,0 DIR Z").ok);
    REQUIRE(run(d, "RECT 50,0 60,10").ok);
    REQUIRE(run(d, "REVOLVE \"" + d.scene.sketch().curves().back().name + "\" AXIS V0").ok);
    CHECK_NEAR(volumeOf(last(d)), 3.14159265 * (3600 - 2500) * 10, 40.0);
    REQUIRE(run(d, "REFAXIS Off AT 0,5,0 DIR Z").ok);
    REQUIRE(run(d, "RECT 50,20 60,30").ok);
    CHECK(!run(d, "REVOLVE \"" + d.scene.sketch().curves().back().name + "\" AXIS Off").ok); // not in the plane

    // Saved, and REGEN makes them again.
    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_refs_test.sicdm").string();
    REQUIRE(run(d, "SAVE \"" + path + "\"").ok);
    Document e;
    REQUIRE(run(e, "OPEN \"" + path + "\"").ok);
    std::filesystem::remove(path);
    CHECK(e.state.refAxes.size() == 4);
    CHECK(e.state.refPoints.size() == 1);
    REQUIRE(run(e, "REGEN").ok);
    CHECK(e.state.refAxes.size() == 4);
    CHECK(e.scene.bodies().size() == d.scene.bodies().size());
}

TEST_CASE("Sheet metal: base, flanges, bend allowance and the flat pattern") {
    CHECK_NEAR(bendAllowance(90, 1, 0.44f, 2), 3.14159265f / 2 * (1 + 0.88f), 1e-5);
    Document d;
    REQUIRE(run(d, "RECT 0,0 100,60").ok);
    REQUIRE(run(d, "SHEETMETAL BASE \"Rectangle 1\" THICKNESS 2 RADIUS 1 K 0.44").ok);
    CHECK(last(d).kind == "sheet");
    CHECK_NEAR(volumeOf(last(d)), 100 * 60 * 2, 1e-2);
    REQUIRE(run(d, "SHEETMETAL FLANGE \"Sheet 1\" EDGE 1 LENGTH 20").ok); // along y = 0, up
    const AABB b = boundsOf(last(d));
    // The bend is added outside the base's edge: the flange's outside face at y = -(r + t).
    CHECK_NEAR(b.min.y, -3.0f, 1e-3);
    CHECK_NEAR(b.max.z, 1 + 2 + 20.0f, 1e-3); // r + t over the base, then the flat 20
    CHECK(openEdgeCount(last(d).mesh) == 0);
    REQUIRE(run(d, "SHEETMETAL FLANGE \"Sheet 1\" NEAR 0,30,0 LENGTH 10 DOWN").ok);
    SheetPart part;
    REQUIRE(sheetFromParams(last(d).params, part));
    REQUIRE(part.flanges.size() == 2);
    CHECK(part.flanges[1].edge == 3); // the base's left edge
    CHECK(part.flanges[1].down);
    CHECK_NEAR(boundsOf(last(d)).min.z, -(1 + 2 + 10.0f) + 2.0f, 1e-3);
    // A flange on a flange's far edge.
    const auto edges = sheetFreeEdges(part);
    int far = -1;
    for (std::size_t e = 0; e < edges.size(); ++e)
        if (edges[e].parent == 0) far = static_cast<int>(e);
    REQUIRE(far >= 0);
    REQUIRE(run(d, "SHEETMETAL FLANGE \"Sheet 1\" EDGE " + std::to_string(far + 1) + " LENGTH 5 ANGLE 90").ok);
    CHECK(openEdgeCount(last(d).mesh) == 0);
    CHECK(!run(d, "SHEETMETAL FLANGE \"Sheet 1\" EDGE 99 LENGTH 5").ok);

    // The flat pattern: the base plus each bend's allowance and flat.
    const float ba = bendAllowance(90, 1, 0.44f, 2);
    REQUIRE(run(d, "SHEETMETAL FLAT \"Sheet 1\" SKETCH").ok);
    const Body& flat = last(d);
    const AABB fb = boundsOf(flat);
    CHECK_NEAR(fb.max.x - fb.min.x, 100 + ba + 10, 1e-3);
    CHECK_NEAR(fb.max.y - fb.min.y, 60 + ba + 20 + ba + 5, 1e-3);
    CHECK_NEAR(fb.max.z - fb.min.z, 2.0f, 1e-4);
    CHECK_NEAR(volumeOf(flat), 2.0 * ((100 * 60) + (ba + 20) * 100 + (ba + 10) * 60 + (ba + 5) * 100), 0.5);
    // Its outline is one loop, with a bend line for each bend.
    int loops = 0, bends = 0;
    for (const SketchCurve& c : d.scene.sketch().curves()) loops += c.kind == CurveKind::Polyline, bends += c.kind == CurveKind::Line && c.construction;
    CHECK(loops == 1);
    CHECK(bends == 3);
    CHECK(run(d, "SHEETMETAL INFO \"Sheet 1\"").ok);
    CHECK(!run(d, "SHEETMETAL FLANGE \"Sheet 1 flat\" EDGE 1 LENGTH 5").ok); // not a sheet metal body

    // Saved and reopened, it rebuilds the same.
    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_sheet_test.sicdm").string();
    REQUIRE(run(d, "SAVE \"" + path + "\"").ok);
    Document e;
    REQUIRE(run(e, "OPEN \"" + path + "\"").ok);
    std::filesystem::remove(path);
    CHECK_NEAR(volumeOf(e.scene.bodies()[0]), volumeOf(d.scene.bodies()[0]), 1e-2);
}

TEST_CASE("FEA: a cantilever's deflection and bending stress match beam theory") {
    // 100 x 10 x 10 steel, held at one end, 100 N down at the other:
    // deflection P L^3 / (3 E I) = 0.2 mm, root stress M c / I = 60 MPa.
    Document d;
    REQUIRE(run(d, "BOX 100 10 10 @50,0,0").ok);
    FeaSetup setup;
    setup.material = *feaMaterial("steel");
    setup.fixed.push_back({FeaSelection::Kind::Face, {0, 0, 5}, 0});
    setup.loads.push_back({{FeaSelection::Kind::Face, {100, 0, 5}, 0}, {0, 0, -100}});
    setup.cell = 1;
    FeaResult r;
    std::string why;
    REQUIRE(runFea(worldMesh(last(d)), setup, r, &why));
    CHECK(r.converged);
    CHECK(r.cells == 10000);
    CHECK_NEAR(r.maxDisplacement, 0.2, 0.2 * 0.05);
    CHECK_NEAR(r.maxStress, 60.0, 60.0 * 0.08);
    CHECK_NEAR(r.maxDisplacementAt.x, 100.0f, 1e-3);
    CHECK(r.maxStressAt.x < 5.0f); // at the root
    CHECK_NEAR(r.safetyFactor, 250.0 / r.maxStress, 1e-9);
    // Aluminium bends E_steel / E_al times as much.
    setup.material = *feaMaterial("aluminum");
    FeaResult al;
    REQUIRE(runFea(worldMesh(last(d)), setup, al, &why));
    CHECK_NEAR(al.maxDisplacement / r.maxDisplacement, 200000.0 / 69000.0, 0.01);
    // Refused: nothing held, nothing loaded.
    setup.fixed.clear();
    CHECK(!runFea(worldMesh(last(d)), setup, al, &why));

    // The command: a report, and with PLOT a coloured copy (kept in the history).
    const CommandOutput o = run(d, "FEA \"Block 1\" FIX FACE 0,0,5 LOAD FACE 100,0,5 FORCE 0,0,-100 CELL 2 PLOT");
    REQUIRE(o.ok);
    REQUIRE(d.scene.bodies().size() == 2);
    CHECK(!d.scene.bodies()[0].visible);
    CHECK(!last(d).field.empty());
    float hi = 0;
    for (const Vertex& v : last(d).mesh.vertices) hi = std::max(hi, v.mask);
    CHECK_NEAR(hi, 1.0f, 0.05);
    CHECK(d.state.history.back().line.find("FEA") == 0);
    CHECK(run(d, "FEA MATERIALS").ok);
    CHECK(!run(d, "FEA \"Block 1\" LOAD FACE 100,0,5 FORCE 0,0,-100").ok);
    CHECK(!run(d, "FEA \"Block 1\" FIX FACE 0,0,5 LOAD FACE 100,0,5 FORCE 0,0,-100 MATERIAL unobtainium").ok);
}

TEST_CASE("Variable-radius fillets and full rounds") {
    // R2 to R8 along a 60 mm edge: the corner removed is (1 - pi/4) r^2, integrated along it.
    Document d;
    REQUIRE(run(d, "BOX 60 40 20 @0,0,0").ok);
    REQUIRE(run(d, "FILLET \"Block 1\" RADIUS 2 TO 8 EDGES NEAR 0,-20,20 START -30,-20,20").ok);
    const double removed = (1 - 3.14159265 / 4) * 60 * (4 + 16 + 64) / 3.0;
    CHECK_NEAR(volumeOf(last(d)), 48000 - removed, 6.0);
    CHECK(openEdgeCount(last(d).mesh) == 0);
    // The small end where START is: the face at x = -30 keeps nearly all its corner.
    float zAtMinX = -1e9f;
    for (const Vertex& v : worldMesh(last(d)).vertices)
        if (std::fabs(v.position.x + 30) < 1e-3f && std::fabs(v.position.y + 20) < 1e-3f) zAtMinX = std::max(zAtMinX, v.position.z);
    CHECK_NEAR(zAtMinX, 18.0f, 0.05);
    CHECK(!run(d, "CHAMFER \"Block 1\" DISTANCE 1 TO 3 EDGES 5").ok); // a chamfer has one size

    // A full round on a 6 mm wall: both top edges at R3, meeting in a half circle.
    Document w;
    REQUIRE(run(w, "BOX 60 6 30 @0,0,0").ok);
    REQUIRE(run(w, "FILLET \"Block 1\" FULLROUND 0,0,30").ok);
    CHECK_NEAR(volumeOf(last(w)), 60 * 6 * 30 - 2 * (1 - 3.14159265 / 4) * 9 * 60, 4.0);
    CHECK(openEdgeCount(last(w).mesh) == 0);
    CHECK_NEAR(boundsOf(last(w)).max.z, 30.0f, 0.02);
    CHECK(!run(w, "FILLET \"Block 1\" FULLROUND 500,0,0").ok);
}

TEST_CASE("Shell with a chosen open face, domes and ribs") {
    // A 60 x 40 x 30 box, 2 mm walls, open at the top: 60*40*30 - 56*36*28.
    Document d;
    REQUIRE(run(d, "BOX 60 40 30 @0,0,0").ok);
    REQUIRE(run(d, "SHELL \"Block 1\" THICKNESS 2 FACE 0,0,30").ok);
    CHECK_NEAR(volumeOf(last(d)), 60 * 40 * 30 - 56 * 36 * 28, 20.0);
    CHECK(openEdgeCount(last(d).mesh) == 0);

    // A dome on a disc reaching in by R(1 - cos a) as it rises h sin a: on a circle of radius
    // R that is half an ellipsoid's cap of the same height, 2/3 pi R^2 h.
    Document c;
    REQUIRE(run(c, "CYLINDER 10 5 @0,0,0").ok);
    const double disc = volumeOf(last(c));
    REQUIRE(run(c, "DOME \"Cylinder 1\" FACE 0,0,5 HEIGHT 6").ok);
    CHECK(openEdgeCount(last(c).mesh) == 0);
    CHECK_NEAR(volumeOf(last(c)) - disc, 1266.0, 20.0); // the outline's reach is 98% of R (numerically 1265.7)
    CHECK_NEAR(boundsOf(last(c)).max.z, 11.0f, 0.05);
    // Dished in: the same shape taken out.
    Document b;
    REQUIRE(run(b, "BOX 40 40 10 @0,0,0").ok);
    REQUIRE(run(b, "DOME \"Block 1\" FACE 0,0,10 HEIGHT 4 CONCAVE").ok);
    CHECK_NEAR(volumeOf(last(b)), 16000 - 4297.3, 25.0);
    CHECK(openEdgeCount(last(b).mesh) == 0);
    CHECK(!run(b, "DOME \"Block 1\" FACE 0,0,10").ok); // needs a height

    // A rib in the corner of an L: a 40 x 20 triangle, 4 mm thick.
    Document l;
    REQUIRE(run(l, "BOX 60 40 5 @0,0,0").ok);
    REQUIRE(run(l, "BOX 5 40 40 @-27.5,0,0").ok);
    REQUIRE(run(l, "BOOLEAN \"Block 1\" UNION \"Block 2\"").ok);
    const double ell = volumeOf(last(l));
    REQUIRE(run(l, "PLANE FRONT").ok);
    REQUIRE(run(l, "LINE -25,25 15,5").ok);
    REQUIRE(run(l, "RIB THICKNESS 4").ok);
    const Body& ribbed = l.scene.bodies()[0];
    CHECK_NEAR(volumeOf(ribbed) - ell, 40 * 20 / 2 * 4, 15.0);
    CHECK(openEdgeCount(ribbed.mesh) == 0);
    CHECK_NEAR(boundsOf(ribbed).max.x, 30.0f, 0.01); // nothing pokes out of the L
    CHECK_NEAR(boundsOf(ribbed).min.x, -30.0f, 0.01);
    CHECK_NEAR(boundsOf(ribbed).max.z, 40.0f, 0.01);
    REQUIRE(run(l, "LINE -100,100 -90,100").ok);
    CHECK(!run(l, "RIB \"Line 2\" THICKNESS 4").ok); // over nothing
    REQUIRE(run(l, "RECT 0,10 10,20").ok);
    CHECK(!run(l, "RIB THICKNESS 2").ok); // a closed curve
}

TEST_CASE("Text outlines and embossed or engraved lettering") {
    // The built-in fonts' glyphs: 'o' is a ring (an outer contour and a hole), capitals as tall
    // as asked.
    float width = 0;
    const auto o = textOutline("o", 10, TextFont::SansBold, &width);
    REQUIRE(o.size() == 2);
    CHECK(signedArea(o[0]) * signedArea(o[1]) < 0);
    CHECK(width > 4.0f);
    const auto h = textOutline("H", 10);
    REQUIRE(h.size() == 1);
    float top = 0, bottom = 1;
    for (const Vec2& p : h[0]) top = std::max(top, p.y), bottom = std::min(bottom, p.y);
    CHECK_NEAR(top, 10.0f, 1e-3);
    CHECK_NEAR(bottom, 0.0f, 1e-3);
    CHECK(textOutline("B8%", 5, TextFont::Mono).size() >= 6);
    CHECK(textOutline("SI CDM", 5, TextFont::Sans).size() >= 6);
    std::string why;
    CHECK(textOutline("\xe4\xb8\xad", 5, TextFont::SansBold, nullptr, &why).empty()); // no CJK in the font
    CHECK(!why.empty());

    // Raised by the letters' area times the depth; cut likewise.
    double area = 0;
    for (const auto& l : textOutline("SI CDM", 8)) area += signedArea(l);
    Document d;
    REQUIRE(run(d, "BOX 80 30 10 @0,0,0").ok);
    REQUIRE(run(d, "EMBOSS \"Block 1\" TEXT \"SI CDM\" FACE 0,0,10 HEIGHT 8 DEPTH 1").ok);
    CHECK_NEAR(volumeOf(last(d)), 24000 + area, 1.0);
    CHECK(openEdgeCount(last(d).mesh) == 0);
    CHECK_NEAR(boundsOf(last(d)).max.z, 11.0f, 1e-3);
    double cutArea = 0;
    for (const auto& l : textOutline("Part 42-B", 5)) cutArea += signedArea(l);
    REQUIRE(run(d, "ENGRAVE \"Block 1\" TEXT \"Part 42-B\" FACE 0,-15,5 HEIGHT 5 DEPTH 0.5").ok);
    CHECK_NEAR(volumeOf(last(d)), 24000 + area - 0.5 * cutArea, 1.0);
    CHECK(openEdgeCount(last(d).mesh) == 0);
    CHECK_NEAR(boundsOf(last(d)).min.y, -15.0f, 1e-3); // cut in, nothing sticking out
    // On a cylinder the letters follow the curve: all of them 1 mm proud of a 20 mm radius.
    Document c;
    REQUIRE(run(c, "CYLINDER 20 40 @0,0,0").ok);
    REQUIRE(run(c, "EMBOSS \"Cylinder 1\" TEXT \"OK\" FACE 0,-20,20 HEIGHT 8 DEPTH 1").ok);
    CHECK(openEdgeCount(last(c).mesh) == 0);
    float rMax = 0;
    for (const Vertex& v : worldMesh(last(c)).vertices) rMax = std::max(rMax, std::hypot(v.position.x, v.position.y));
    CHECK_NEAR(rMax, 21.0f, 0.05);
    CHECK(!run(c, "EMBOSS \"Cylinder 1\" TEXT \"MUCH TOO LONG FOR THIS\" FACE 0,-20,20 HEIGHT 30 DEPTH 1").ok);
    CHECK(!run(c, "ENGRAVE \"Cylinder 1\" TEXT \"x\" FACE 0,-20,20 HEIGHT 5").ok); // no depth
}

TEST_CASE("Feature tree: suppress, roll back, reorder and change parameters") {
    Document d;
    REQUIRE(run(d, "BOX 60 40 15 @0,0,0").ok);
    REQUIRE(run(d, "HOLE \"Block 1\" AT -20,0,15 DIAMETER 6 THROUGH").ok);
    REQUIRE(run(d, "HOLE \"Block 1\" AT 20,0,15 DIAMETER 8 THROUGH").ok);
    const double pi = 3.14159265358979;
    const double box = 60 * 40 * 15, h6 = pi * 9 * 15, h8 = pi * 16 * 15, h10 = pi * 25 * 15;
    CHECK_NEAR(volumeOf(last(d)), box - h6 - h8, 6.0);
    // A parameter changed in place.
    REQUIRE(run(d, "HISTORY SET 2 DIAMETER 10").ok);
    CHECK(d.state.history[1].line == "HOLE \"Block 1\" AT -20,0,15 DIAMETER 10 THROUGH");
    CHECK_NEAR(volumeOf(last(d)), box - h10 - h8, 6.0);
    CHECK(!run(d, "HISTORY SET 2 RADIUS 3").ok); // a hole has no RADIUS
    CHECK(!run(d, "HISTORY SET 9 DIAMETER 3").ok);
    // Suppressed: kept, skipped; then back.
    REQUIRE(run(d, "HISTORY SUPPRESS 2").ok);
    CHECK(d.state.history[1].suppressed);
    CHECK_NEAR(volumeOf(last(d)), box - h8, 6.0);
    CHECK(run(d, "HISTORY").lines[2].find("(suppressed)") != std::string::npos);
    REQUIRE(run(d, "HISTORY UNSUPPRESS 2-3").ok);
    CHECK_NEAR(volumeOf(last(d)), box - h10 - h8, 6.0);
    // Suppressing the step that makes the body leaves later steps nothing to work on:
    // refused, and nothing changes.
    CHECK(!run(d, "HISTORY SUPPRESS 1").ok);
    CHECK(!d.state.history[0].suppressed);
    CHECK_NEAR(volumeOf(last(d)), box - h10 - h8, 6.0);

    // Rolled back to step 1: only the box; a new step goes in there.
    REQUIRE(run(d, "HISTORY ROLLBACK 1").ok);
    CHECK(d.state.rollback == 1);
    CHECK_NEAR(volumeOf(last(d)), box, 1.0);
    REQUIRE(run(d, "FILLET \"Block 1\" RADIUS 2 EDGES NEAR 0,20,15").ok);
    REQUIRE(d.state.history.size() == 4);
    CHECK(d.state.history[1].line.find("FILLET") == 0);
    CHECK(d.state.rollback == 2);
    // Saved and opened rolled back, with the flags.
    REQUIRE(run(d, "HISTORY SUPPRESS 4").ok);
    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_tree_test.sicdm").string();
    REQUIRE(run(d, "SAVE \"" + path + "\"").ok);
    Document e;
    REQUIRE(run(e, "OPEN \"" + path + "\"").ok);
    CHECK(e.state.rollback == 2);
    REQUIRE(e.state.history.size() == 4);
    CHECK(e.state.history[3].suppressed);
    std::filesystem::remove(path);
    // UNDO takes back the step before the bar.
    REQUIRE(run(d, "UNDO").ok);
    CHECK(d.state.history.size() == 3);
    CHECK(d.state.rollback == 1);
    REQUIRE(run(d, "REDO").ok);
    CHECK(d.state.rollback == 2);
    // Forward again: everything built (the suppressed hole left out).
    REQUIRE(run(d, "HISTORY ROLLFORWARD").ok);
    CHECK(d.state.rollback == -1);
    const double rounded = volumeOf(last(d));
    CHECK_NEAR(rounded, box - h10 - (1 - pi / 4) * 4 * 60, 6.0); // the R2 round along the 60 mm edge

    // Reordered: the second hole before the fillet gives the same part; a step moved before
    // the body it needs is refused.
    REQUIRE(run(d, "HISTORY MOVE 3 TO 2").ok);
    CHECK(d.state.history[1].line.find("HOLE") == 0);
    CHECK_NEAR(volumeOf(last(d)), rounded, 0.5);
    CHECK(!run(d, "HISTORY MOVE 2 TO 1").ok);
    CHECK(d.state.history[0].line.find("BOX") == 0);
    // Deleting a step before the bar moves the bar with it.
    REQUIRE(run(d, "HISTORY ROLLBACK 3").ok);
    REQUIRE(run(d, "HISTORY DELETE 2").ok);
    CHECK(d.state.rollback == 2);
}

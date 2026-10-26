#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Extrude.hpp"
#include "core/FileIO.hpp"
#include "core/Json.hpp"
#include "core/Brep.hpp"
#include "core/Scene.hpp"
#include "core/Sculpt.hpp"

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

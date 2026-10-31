#include "TestFramework.hpp"

#include "core/Brep.hpp"
#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Scene.hpp"
#include "core/Sculpt.hpp"
#include "core/Step.hpp"
#include "core/Surfacing.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
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

const Body* named(const Document& d, const std::string& name) {
    for (const Body& b : d.scene.bodies())
        if (b.name == name) return &b;
    return nullptr;
}

} // namespace

TEST_CASE("B-spline interpolation passes through its points") {
    std::vector<DVec3> pts;
    for (int k = 0; k <= 8; ++k) pts.push_back({static_cast<double>(k) * 3, std::sin(k * 0.7) * 5, k * k * 0.1});
    const BSplineCurve c = interpolateCurve(pts, 3);
    REQUIRE(c.cp.size() == pts.size());
    CHECK(c.degree == 3);
    std::vector<double> params, knots;
    interpolationParams({pts}, 3, params, knots);
    for (std::size_t k = 0; k < pts.size(); ++k) CHECK(dlength(c.eval(params[k]) - pts[k]) < 1e-9);
    // A surface through a grid: through every point of it.
    std::vector<std::vector<DVec3>> rows;
    for (int i = 0; i < 5; ++i) {
        std::vector<DVec3> r;
        for (int j = 0; j < 7; ++j) r.push_back({static_cast<double>(i) * 10, static_cast<double>(j) * 5, std::sin(i * 0.5) * std::cos(j * 0.4) * 4});
        rows.push_back(r);
    }
    BSplineSurface s;
    REQUIRE(interpolateSurface(rows, s));
    CHECK(s.du == 3);
    CHECK(s.dv == 3);
    std::vector<double> pu, ku, pv, kv;
    std::vector<std::vector<DVec3>> cols(7, std::vector<DVec3>(5));
    for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 7; ++j) cols[static_cast<std::size_t>(j)][static_cast<std::size_t>(i)] = rows[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)];
    interpolationParams(rows, 3, pv, kv);
    interpolationParams(cols, 3, pu, ku);
    double worst = 0;
    for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 7; ++j)
            worst = std::max(worst, dlength(s.eval(pu[static_cast<std::size_t>(i)], pv[static_cast<std::size_t>(j)]) - rows[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]));
    CHECK(worst < 1e-9);
    // Two rows: ruled (degree 1 across).
    BSplineSurface r2;
    REQUIRE(interpolateSurface({rows[0], rows[1]}, r2));
    CHECK(r2.du == 1);
}

TEST_CASE("Surfaces: loft, boundary, extrude, revolve, offset, blend") {
    // Lofted through two circles of radius 10, 20 apart: a cylinder.
    Document d;
    REQUIRE(run(d, "CIRCLE 0,0 10").ok);
    REQUIRE(run(d, "PLANE OFFSET 20").ok);
    REQUIRE(run(d, "CIRCLE 0,0 10").ok);
    REQUIRE(run(d, "SURFACE LOFT \"Circle 1\" \"Circle 2\"").ok);
    const Body* loft = named(d, "Surface 1");
    REQUIRE(loft);
    REQUIRE(loft->exact());
    CHECK(loft->brep->faceCount() == 1);
    CHECK(!loft->brep->shells[0].closed);
    CHECK(loft->brep->surfaces[0].kind == BSurfaceKind::BSpline);
    double rMin = 1e9, rMax = 0;
    for (const Vertex& v : worldMesh(*loft).vertices) {
        const double r = std::hypot(v.position.x, v.position.y);
        rMin = std::min(rMin, r), rMax = std::max(rMax, r);
    }
    CHECK_NEAR(rMin, 10.0, 0.02);
    CHECK_NEAR(rMax, 10.0, 0.02);
    // To STEP as a surface model, and back.
    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_surface.step").string();
    REQUIRE(run(d, "EXPORT \"" + path + "\"").ok);
    const std::vector<StepPart> parts = readStepFile(path);
    REQUIRE(parts.size() == 1);
    CHECK(parts[0].brep->surfaces[0].kind == BSurfaceKind::BSpline);
    std::filesystem::remove(path);
    CHECK(!run(d, "SURFACE LOFT \"Circle 1\"").ok);

    // Four lines round a square: the flat patch inside, 40 x 40.
    Document b;
    for (const char* l : {"LINE -20,-20 20,-20", "LINE 20,-20 20,20", "LINE 20,20 -20,20", "LINE -20,20 -20,-20"}) REQUIRE(run(b, l).ok);
    REQUIRE(run(b, "SURFACE BOUNDARY \"Line 1\" \"Line 3\" \"Line 2\" \"Line 4\"").ok); // in any order
    const Body* patch = named(b, "Surface 1");
    REQUIRE(patch);
    double area = 0;
    const MeshData pm = worldMesh(*patch);
    for (std::size_t k = 0; k + 2 < pm.indices.size(); k += 3)
        area += 0.5 * length(cross(pm.vertices[pm.indices[k + 1]].position - pm.vertices[pm.indices[k]].position,
                                   pm.vertices[pm.indices[k + 2]].position - pm.vertices[pm.indices[k]].position));
    CHECK_NEAR(area, 1600.0, 0.5);
    REQUIRE(run(b, "LINE 50,0 60,0").ok);
    CHECK(!run(b, "SURFACE BOUNDARY \"Line 1\" \"Line 2\" \"Line 3\" \"Line 5\"").ok); // no loop

    // Offset 2 from the patch: 2 above (or below) everywhere.
    REQUIRE(run(b, "SURFACE OFFSET \"Surface 1\" DISTANCE 2").ok);
    const Body* off = named(b, "Surface 2");
    REQUIRE(off);
    for (const Vertex& v : worldMesh(*off).vertices) CHECK_NEAR(std::fabs(v.position.z), 2.0f, 1e-3);
    // Thickened 3: a 40 x 40 x 3 slab.
    REQUIRE(run(b, "THICKEN \"Surface 1\" THICKNESS 3").ok);
    const Body& slab = b.scene.bodies().back();
    CHECK(openEdgeCount(slab.mesh) == 0);
    CHECK_NEAR(meshVolume(worldMesh(slab)), 4800.0, 1.0);
    CHECK(!named(b, "Surface 1")->visible);
    CHECK(!run(b, "THICKEN \"" + slab.name + "\" THICKNESS 1").ok); // already a solid

    // Extruded and revolved from open curves.
    Document e;
    REQUIRE(run(e, "LINE 0,0 10,0").ok);
    REQUIRE(run(e, "SURFACE EXTRUDE \"Line 1\" DISTANCE 5").ok);
    const AABB eb = worldMesh(*named(e, "Surface 1")).bounds();
    CHECK_NEAR(eb.max.z - eb.min.z, 5.0f, 1e-4);
    REQUIRE(run(e, "PLANE FRONT").ok);
    REQUIRE(run(e, "LINE 10,0 10,20").ok);
    REQUIRE(run(e, "SURFACE REVOLVE \"Line 2\" AXIS Z AT 0,0,0").ok);
    for (const Vertex& v : worldMesh(*named(e, "Surface 2")).vertices) CHECK_NEAR(std::hypot(v.position.x, v.position.y), 10.0f, 0.01);
    CHECK(!run(e, "SURFACE REVOLVE \"Line 2\"").ok); // no axis

    // A blend between two strips meets both along their facing edges.
    Document g;
    REQUIRE(run(g, "LINE 0,0 20,0").ok);
    REQUIRE(run(g, "SURFACE EXTRUDE \"Line 1\" DISTANCE 10").ok); // z 0..10 at y 0
    REQUIRE(run(g, "PLANE OFFSET 30").ok);
    REQUIRE(run(g, "LINE 0,10 20,10").ok);
    REQUIRE(run(g, "SURFACE EXTRUDE \"Line 2\" DISTANCE 10").ok); // z 30..40 at y 10
    REQUIRE(run(g, "SURFACE BLEND \"Surface 1\" \"Surface 2\"").ok);
    const AABB bb = worldMesh(*named(g, "Surface 3")).bounds();
    CHECK_NEAR(bb.min.z, 10.0f, 0.01);
    CHECK_NEAR(bb.max.z, 30.0f, 0.01);
    CHECK_NEAR(bb.min.y, 0.0f, 0.05);
    CHECK_NEAR(bb.max.y, 10.0f, 0.05);
}

TEST_CASE("Knitting surfaces into a solid, and surface analysis") {
    // A tube and its two caps (tessellated differently) knit into a closed cylinder.
    Document d;
    REQUIRE(run(d, "CIRCLE 0,0 10").ok);
    REQUIRE(run(d, "SURFACE EXTRUDE \"Circle 1\" DISTANCE 20").ok);
    REQUIRE(run(d, "CIRCLE 0,0 10").ok);
    REQUIRE(run(d, "SURFACE PLANAR \"Circle 2\"").ok);
    REQUIRE(run(d, "PLANE OFFSET 20").ok);
    REQUIRE(run(d, "CIRCLE 0,0 10").ok);
    REQUIRE(run(d, "SURFACE PLANAR \"Circle 3\"").ok);
    const auto r = run(d, "KNIT \"Surface 1\" \"Surface 2\" \"Surface 3\"");
    REQUIRE(r.ok);
    REQUIRE(d.scene.bodies().size() == 1);
    const Body& solid = d.scene.bodies()[0];
    CHECK(openEdgeCount(solid.mesh) == 0);
    CHECK_NEAR(meshVolume(solid.mesh), kPiS * 100 * 20, 0.005 * kPiS * 100 * 20);
    // Two of them alone stay open.
    Document o;
    REQUIRE(run(o, "CIRCLE 0,0 10").ok);
    REQUIRE(run(o, "SURFACE EXTRUDE \"Circle 1\" DISTANCE 20").ok);
    REQUIRE(run(o, "CIRCLE 0,0 10").ok);
    REQUIRE(run(o, "SURFACE PLANAR \"Circle 2\"").ok);
    REQUIRE(run(o, "KNIT \"Surface 1\" \"Surface 2\"").ok);
    CHECK(openEdgeCount(o.scene.bodies()[0].mesh) > 0);

    // Curvature of a sphere of radius 20: mean 1/20, Gaussian 1/400.
    Document s;
    REQUIRE(run(s, "SPHERE 20 @0,0,0").ok);
    std::vector<float> H, K;
    meshCurvature(worldMesh(s.scene.bodies()[0]), H, K);
    std::vector<float> hs(H), ks(K);
    std::sort(hs.begin(), hs.end());
    std::sort(ks.begin(), ks.end());
    CHECK_NEAR(std::fabs(hs[hs.size() / 2]), 0.05f, 0.005f);
    CHECK_NEAR(ks[ks.size() / 2], 1.0f / 400, 0.0004f);
    // Zebra and curvature plots stand in for the body, and go away again.
    REQUIRE(run(s, "ANALYZE ZEBRA \"Sphere 1\"").ok);
    CHECK(s.scene.bodies().back().field.rfind("zebra", 0) == 0);
    CHECK(!s.scene.bodies()[0].visible);
    REQUIRE(run(s, "ANALYZE CURVATURE \"Sphere 1\" GAUSS").ok);
    CHECK(s.scene.bodies().back().field.find("Gaussian") != std::string::npos);
    REQUIRE(run(s, "ANALYZE OFF").ok);
    CHECK(s.scene.bodies().size() == 1);
    CHECK(s.scene.bodies()[0].visible);
}

TEST_CASE("Surfaces: a Gordon network through crossing curves") {
    // Curves cut from z = f(x, y): three profiles along x, three guides along y. The network
    // must come back to f between them as well as on them.
    auto f = [](double x, double y) { return 0.002 * (x * x - y * y) + 0.001 * x * y; };
    auto path = [&](bool alongX, double c) {
        std::string line = "PATH3D";
        for (int k = 0; k <= 8; ++k) {
            const double v = 5.0 * k, x = alongX ? v : c, y = alongX ? c : v;
            line += " " + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(f(x, y));
        }
        return line + " SPLINE";
    };
    Document d;
    std::vector<std::string> prof, guide;
    for (double c : {0.0, 20.0, 40.0}) {
        REQUIRE(run(d, path(true, c)).ok);
        prof.push_back(d.scene.sketch().curves().back().name);
    }
    for (double c : {40.0, 0.0, 20.0}) { // any order; one reversed below
        REQUIRE(run(d, path(false, c)).ok);
        guide.push_back(d.scene.sketch().curves().back().name);
    }
    std::string cmd = "SURFACE NETWORK";
    for (const std::string& n : prof) cmd += " \"" + n + "\"";
    cmd += " ACROSS";
    for (const std::string& n : guide) cmd += " \"" + n + "\"";
    const CommandOutput o = run(d, cmd);
    if (!o.ok) std::fprintf(stderr, "%s\n", o.lines.back().c_str());
    REQUIRE(o.ok);
    const Body& s = d.scene.bodies().back();
    double worst = 0;
    for (const Vertex& v : s.mesh.vertices) {
        const Vec3 p = v.position + s.offset;
        worst = std::max(worst, std::fabs(p.z - f(p.x, p.y)));
    }
    CHECK(worst < 0.02); // f spans 6.4 over the patch
    const AABB bb = s.mesh.bounds();
    CHECK(std::fabs(bb.min.x) < 0.05f && std::fabs(bb.max.x - 40) < 0.05f && std::fabs(bb.min.y) < 0.05f && std::fabs(bb.max.y - 40) < 0.05f);

    // Curves that miss each other are refused.
    REQUIRE(run(d, "PATH3D 0,100,50 40,100,50").ok);
    CHECK(!run(d, "SURFACE NETWORK \"" + prof[0] + "\" \"" + d.scene.sketch().curves().back().name + "\" ACROSS \"" + guide[0] + "\" \"" + guide[1] + "\"").ok);
}

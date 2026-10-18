#include "TestFramework.hpp"

#include "core/Extrude.hpp"
#include "core/Scene.hpp"

using namespace sicdm;

namespace {

bool windsOutward(const MeshData& m) {
    for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3) {
        const Vertex& a = m.vertices[m.indices[i]];
        const Vertex& b = m.vertices[m.indices[i + 1]];
        const Vertex& c = m.vertices[m.indices[i + 2]];
        const Vec3 n = cross(b.position - a.position, c.position - a.position);
        if (length(n) < 1e-9f) continue;
        if (dot(n, a.normal + b.normal + c.normal) <= 0.0f) return false;
    }
    return true;
}

// Signed volume via the divergence theorem: positive for a closed, outward mesh.
double volume(const MeshData& m) {
    double v = 0.0;
    for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3) {
        const Vec3 a = m.vertices[m.indices[i]].position, b = m.vertices[m.indices[i + 1]].position,
                   c = m.vertices[m.indices[i + 2]].position;
        v += dot(a, cross(b, c)) / 6.0;
    }
    return v;
}

SketchCurve curve(CurveKind k, std::vector<Vec2> pts, float r = 0, SketchPlane plane = SketchPlane::Top) {
    SketchCurve c;
    c.kind = k;
    c.pts = std::move(pts);
    c.radius = r;
    c.plane = plane;
    c.name = "c";
    return c;
}

} // namespace

TEST_CASE("ear clipping triangulates convex and concave polygons") {
    const std::vector<Vec2> square = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
    CHECK(triangulatePolygon(square).size() == 6);
    const std::vector<Vec2> ell = {{0, 0}, {20, 0}, {20, 5}, {5, 5}, {5, 20}, {0, 20}}; // L shape
    const auto t = triangulatePolygon(ell);
    CHECK(t.size() == 12);
    double area = 0;
    for (std::size_t i = 0; i + 2 < t.size(); i += 3) {
        const Vec2 a = ell[t[i]], b = ell[t[i + 1]], c = ell[t[i + 2]];
        area += 0.5 * ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y));
    }
    CHECK_NEAR(area, 20 * 5 + 5 * 15, 1e-3);
    CHECK(isSimplePolygon(ell));
    CHECK(!isSimplePolygon({{0, 0}, {10, 10}, {10, 0}, {0, 10}})); // bow-tie
}

TEST_CASE("rectangle extrusion is a closed outward box of the right size") {
    const auto p = profileFromCurve(curve(CurveKind::Rectangle, {{0, 0}, {30, 20}}));
    CHECK(p.has_value());
    if (!p) return;
    const MeshData m = buildExtrusion(extrudeParams(*p, SketchPlane::Top, 10, false));
    CHECK(windsOutward(m));
    CHECK_NEAR(volume(m), 30 * 20 * 10, 1e-2);
    CHECK_NEAR(m.bounds().max.z, 10, 1e-5);
    CHECK(m.edgeVertices.size() == 24); // 4 + 4 outline edges, 4 verticals
}

TEST_CASE("concave closed polyline, circle, symmetric and negative depths") {
    SketchCurve ell = curve(CurveKind::Polyline, {{0, 0}, {20, 0}, {20, 5}, {5, 5}, {5, 20}, {0, 20}, {0, 0}});
    auto p = profileFromCurve(ell);
    CHECK(p.has_value());
    if (p) {
        const MeshData m = buildExtrusion(extrudeParams(*p, SketchPlane::Top, 4, false));
        CHECK(windsOutward(m));
        CHECK_NEAR(volume(m), 175 * 4, 1e-2);
    }
    // Clockwise input is reoriented.
    p = profileFromCurve(curve(CurveKind::Polyline, {{0, 0}, {0, 10}, {10, 10}, {10, 0}, {0, 0}}));
    CHECK(p && signedArea(p->points) > 0);

    p = profileFromCurve(curve(CurveKind::Circle, {{5, 5}}, 10));
    CHECK(p && p->kind == Profile::Kind::Circle);
    if (p) {
        const MeshData m = buildExtrusion(extrudeParams(*p, SketchPlane::Top, 20, true));
        CHECK(windsOutward(m));
        CHECK_NEAR(volume(m), kPi * 100 * 20, 2 * kPi * 100 * 20 * 0.002); // polygonal circle
        CHECK_NEAR(m.bounds().min.z, -10, 1e-4);
        CHECK_NEAR(m.bounds().max.z, 10, 1e-4);
    }
    const auto down = extrudeParams(*profileFromCurve(curve(CurveKind::Rectangle, {{0, 0}, {1, 1}})),
                                    SketchPlane::Top, -5, false);
    CHECK(down[1] == -5.0f && down[2] == 0.0f);
}

TEST_CASE("extrusion follows the sketch-plane normal") {
    // Front plane: u = X, v = Z, normal = -Y (towards the viewer of the front view).
    const auto p = profileFromCurve(curve(CurveKind::Rectangle, {{0, 0}, {10, 10}}, 0, SketchPlane::Front));
    const MeshData m = buildExtrusion(extrudeParams(*p, SketchPlane::Front, 6, false));
    CHECK(windsOutward(m));
    CHECK_NEAR(m.bounds().min.y, -6, 1e-5);
    CHECK_NEAR(m.bounds().max.z, 10, 1e-5);
    CHECK_NEAR(volume(m), 600, 1e-2);
    const auto r = profileFromCurve(curve(CurveKind::Circle, {{0, 0}}, 2, SketchPlane::Right));
    const MeshData mr = buildExtrusion(extrudeParams(*r, SketchPlane::Right, 3, false));
    CHECK(windsOutward(mr));
    CHECK_NEAR(mr.bounds().max.x, 3, 1e-5);
}

TEST_CASE("open, self-intersecting and degenerate curves are rejected") {
    std::string why;
    CHECK(!profileFromCurve(curve(CurveKind::Line, {{0, 0}, {5, 5}}), &why));
    CHECK(!why.empty());
    CHECK(!profileFromCurve(curve(CurveKind::Polyline, {{0, 0}, {5, 0}, {5, 5}}), &why)); // open
    CHECK(!profileFromCurve(curve(CurveKind::Polyline, {{0, 0}, {10, 10}, {10, 0}, {0, 10}, {0, 0}}), &why));
    CHECK(!profileFromCurve(curve(CurveKind::Point, {{0, 0}}), &why));
    CHECK(!profileFromCurve(curve(CurveKind::Polyline, {{0, 0}, {5, 0}, {10, 0}, {0, 0}}), &why)); // no area
    CHECK(buildExtrusion({0, 0, 0, 0, 0, 0, 1, 0, 1, 1}).indices.empty()); // zero depth
}

TEST_CASE("extruded bodies are parametric and rebuild from their params") {
    const auto p = profileFromCurve(curve(CurveKind::Rectangle, {{0, 0}, {4, 4}}));
    const Body b = makeBody("Extrude 1", "extrude", extrudeParams(*p, SketchPlane::Top, 2, false));
    CHECK(isPrimitiveKind("extrude"));
    CHECK(b.mesh.indices.size() == 36);
}

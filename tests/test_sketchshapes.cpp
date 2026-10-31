#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/SketchShapes.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace sicdm;

#define REQUIRE(c)                  \
    do {                            \
        const bool required_ = static_cast<bool>(c); \
        CHECK(required_);           \
        if (!required_) return;     \
    } while (0)

namespace {

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }
const SketchCurve& last(const Document& d) { return d.scene.sketch().curves().back(); }
bool near(Vec2 a, Vec2 b, float tol = 1e-3f) { return std::hypot(a.x - b.x, a.y - b.y) <= tol; }
float area(std::vector<Vec2> p) {
    float s = 0;
    for (std::size_t i = 0; i + 1 < p.size(); ++i) s += p[i].x * p[i + 1].y - p[i + 1].x * p[i].y;
    return 0.5f * s;
}

} // namespace

TEST_CASE("Sketch shapes: rectangles from the centre and from three points") {
    const auto c = centreRectangle({10, 5}, {16, 9});
    REQUIRE(c);
    CHECK(c->kind == CurveKind::Rectangle && near(c->pts[0], {4, 1}) && near(c->pts[1], {16, 9}));
    CHECK(!centreRectangle({0, 0}, {5, 0}));

    // A 30-degree rectangle, 20 long and 8 wide, on either side of its first side.
    const float a = 30.0f * kPi / 180.0f;
    const Vec2 p0{0, 0}, p1{20 * std::cos(a), 20 * std::sin(a)};
    for (float side : {1.0f, -1.0f}) {
        const Vec2 off{-std::sin(a) * 8 * side, std::cos(a) * 8 * side};
        const auto r = rectangle3Points(p0, p1, {p1.x + off.x + 3 * std::cos(a), p1.y + off.y + 3 * std::sin(a)});
        REQUIRE(r);
        CHECK(r->isClosed() && r->pts.size() == 5);
        CHECK(std::fabs(area(r->pts) - 160.0f) < 1e-2f); // counter-clockwise, 20 x 8
    }
    CHECK(!rectangle3Points(p0, p1, {p1.x * 2, p1.y * 2})); // in line: no width
}

TEST_CASE("Sketch shapes: circles through three points and touching two lines") {
    const auto c = circle3Points({0, 0}, {10, 0}, {0, 10});
    REQUIRE(c);
    CHECK(near(c->pts[0], {5, 5}) && std::fabs(c->radius - std::sqrt(50.0f)) < 1e-4f);
    CHECK(!circle3Points({0, 0}, {1, 1}, {2, 2}));
    // Lines along x and y: radius 5, in the quarter nearest (10, 10).
    const auto t = circleTangentToLines({0, 0}, {50, 0}, {0, 0}, {0, 50}, 5, {10, 10});
    REQUIRE(t);
    CHECK(near(t->pts[0], {5, 5}) && t->radius == 5.0f);
    const auto t2 = circleTangentToLines({0, 0}, {50, 0}, {0, 0}, {0, 50}, 5, {-10, 10});
    REQUIRE(t2);
    CHECK(near(t2->pts[0], {-5, 5}));
    CHECK(!circleTangentToLines({0, 0}, {1, 0}, {0, 2}, {1, 2}, 1, {}));
}

TEST_CASE("Sketch shapes: tangent arc, elliptical arc, conic, text") {
    // On from (10,0) heading +x to (20,10): a quarter circle about (10,10), turning left.
    const auto t = tangentArc({10, 0}, {1, 0}, {20, 10});
    REQUIRE(t);
    CHECK(near(t->pts[0], {10, 10}) && near(t->pts[1], {10, 0}) && near(t->pts[2], {20, 10}) && std::fabs(t->radius - 10) < 1e-4f);
    const auto tr = tangentArc({10, 0}, {1, 0}, {20, -10}); // turning right: clockwise, stored the other way round
    REQUIRE(tr);
    CHECK(near(tr->pts[0], {10, -10}) && near(tr->pts[1], {20, -10}) && near(tr->pts[2], {10, 0}));
    CHECK(!tangentArc({0, 0}, {1, 0}, {5, 0}));
    // The arc carries a line on without a kink: its tangent at the start is the line's way.
    SketchCurve line;
    line.kind = CurveKind::Line;
    line.pts = {{0, 0}, {10, 0}};
    Vec2 p, dir;
    REQUIRE(curveEndDirection(line, false, p, dir));
    CHECK(near(p, {10, 0}) && dir.x > 0 && std::fabs(dir.y) < 1e-6f);
    REQUIRE(curveEndDirection(*t, false, p, dir)); // and an arc carries on from its end
    CHECK(near(p, {20, 10}) && std::fabs(dir.x) < 1e-5f && dir.y > 0);

    // A quarter of a 20 x 10 ellipse: from the major axis end to the minor one.
    const auto e = ellipticalArc({0, 0}, {20, 0}, 10, {25, 0}, {0, 7});
    REQUIRE(e);
    CHECK(!e->isClosed() && isEllipticalArc(*e));
    const std::vector<Vec2> s = e->planeStrip(0.001f);
    CHECK(near(s.front(), {20, 0}) && near(s.back(), {0, 10}));
    float len = 0;
    for (std::size_t i = 1; i < s.size(); ++i) len += std::hypot(s[i].x - s[i - 1].x, s[i].y - s[i - 1].y);
    CHECK(std::fabs(len - 24.2211f) < 0.01f); // a quarter of the perimeter (elliptic integral)

    // Conics: the spline stays on the exact curve.
    for (float rho : {0.3f, 0.5f, 0.8f}) {
        const Vec2 a{0, 0}, b{40, 0}, apex{20, 30};
        const auto c = conicCurve(a, b, apex, rho);
        REQUIRE(c);
        std::vector<Vec2> exact;
        for (int i = 0; i <= 4000; ++i) exact.push_back(conicPoint(a, b, apex, rho, static_cast<float>(i) / 4000.0f));
        float worst = 0;
        for (Vec2 q : c->planeStrip(0.005f)) {
            float d = 1e9f; // to the exact curve's chords (fine ones)
            for (std::size_t i = 1; i < exact.size(); ++i) {
                const Vec2 u = exact[i - 1], v = exact[i];
                const float vx = v.x - u.x, vy = v.y - u.y, l2 = vx * vx + vy * vy;
                const float k = l2 > 0 ? std::clamp(((q.x - u.x) * vx + (q.y - u.y) * vy) / l2, 0.0f, 1.0f) : 0.0f;
                d = std::min(d, std::hypot(q.x - u.x - vx * k, q.y - u.y - vy * k));
            }
            worst = std::max(worst, d);
        }
        if (worst >= 0.02f) std::fprintf(stderr, "rho %g worst %g\n", rho, worst);
        CHECK(worst < 0.02f); // 0.05% of its 40 mm span
    }
    CHECK(!conicCurve({0, 0}, {10, 0}, {5, 5}, 1.0f));
    CHECK(!conicCurve({0, 0}, {10, 0}, {5, 0}, 0.5f));
    // A parabola (rho 0.5) through its vertex: y = 15 at the middle.
    CHECK(near(conicPoint({0, 0}, {40, 0}, {20, 30}, 0.5f, 0.5f), {20, 15}));

    // Text: "AB" is two outlines and three holes, 10 tall.
    const std::vector<SketchCurve> text = sketchText("AB", {5, 5}, 10, TextFont::SansBold, 0);
    CHECK(text.size() == 5);
    float top = -1e9f, bottom = 1e9f;
    for (const SketchCurve& c : text) {
        CHECK(c.isClosed());
        for (Vec2 q : c.pts) top = std::max(top, q.y), bottom = std::min(bottom, q.y);
    }
    CHECK(std::fabs(bottom - 5) < 0.2f && std::fabs(top - 15) < 0.5f);
}

TEST_CASE("Sketch shape commands") {
    Document d;
    REQUIRE(run(d, "RECT CENTER 0,0 10,5").ok);
    CHECK(last(d).kind == CurveKind::Rectangle && near(last(d).pts[0], {-10, -5}));
    REQUIRE(run(d, "RECT 3P 0,0 20,0 5,8").ok);
    CHECK(last(d).kind == CurveKind::Polyline && last(d).isClosed());
    REQUIRE(run(d, "EXTRUDE 5").ok); // a profile like any other
    CHECK(std::fabs(d.scene.bodies().back().bounds().max.y - 8) < 1e-3f);
    REQUIRE(run(d, "CIRCLE 3P 0,0 10,0 0,10").ok);
    CHECK(last(d).kind == CurveKind::Circle && near(last(d).pts[0], {5, 5}));

    REQUIRE(run(d, "LINE 100,0 150,0").ok);
    const std::string l1 = last(d).name;
    REQUIRE(run(d, "LINE 100,0 100,50").ok);
    const std::string l2 = last(d).name;
    REQUIRE(run(d, "CIRCLE TTR \"" + l1 + "\" \"" + l2 + "\" 5").ok);
    CHECK(near(last(d).pts[0], {105, 5}));
    REQUIRE(run(d, "CIRCLE TTR \"" + l1 + "\" \"" + l2 + "\" 5 NEAR 90,-10").ok);
    CHECK(near(last(d).pts[0], {95, -5}));
    CHECK(!run(d, "CIRCLE TTR \"" + l1 + "\" Nothing 5").ok);

    { const auto o = run(d, "ARC TANGENT \"" + l1 + "\" 160,10"); if (!o.ok) std::fprintf(stderr, "%s\n", o.lines.back().c_str()); REQUIRE(o.ok); }
    CHECK(last(d).kind == CurveKind::Arc && near(last(d).pts[0], {150, 10}));
    REQUIRE(run(d, "ARC TANGENT 150,20").ok); // on from the arc just drawn (selected), the same circle on
    CHECK(last(d).kind == CurveKind::Arc && near(last(d).keyPoints()[1], {160, 10}, 1e-2f));

    REQUIRE(run(d, "ELLIPSE 0,100 20,100 10 FROM 30,100 TO 0,120").ok);
    CHECK(isEllipticalArc(last(d)) && !last(d).isClosed());
    REQUIRE(run(d, "ELLIPSE 0,100 20,100 10").ok);
    CHECK(last(d).isClosed());

    REQUIRE(run(d, "CONIC 0,200 40,200 20,230 RHO 0.7").ok);
    CHECK(last(d).kind == CurveKind::Spline);
    CHECK(!run(d, "CONIC 0,200 40,200 20,230 RHO 1.5").ok);

    const std::size_t before = d.scene.sketch().curves().size();
    REQUIRE(run(d, "SKETCHTEXT \"Hi\" AT 0mm,300mm HEIGHT 12mm FONT mono").ok);
    CHECK(d.scene.sketch().curves().size() > before + 1);
    CHECK(!run(d, "SKETCHTEXT \"Hi\" FONT gothic").ok);
}

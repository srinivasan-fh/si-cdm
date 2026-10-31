#include "core/SketchShapes.hpp"

#include <cmath>

namespace sicdm {

namespace {

float dist(Vec2 a, Vec2 b) { return std::hypot(b.x - a.x, b.y - a.y); }
float cross2(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }

template <class T> std::optional<T> fail(std::string* why, const char* w) {
    if (why) *why = w;
    return std::nullopt;
}

SketchCurve curve(CurveKind k, std::vector<Vec2> pts, float r = 0) {
    SketchCurve c;
    c.kind = k;
    c.pts = std::move(pts);
    c.radius = r;
    return c;
}

} // namespace

std::optional<SketchCurve> centreRectangle(Vec2 c, Vec2 q, std::string* why) {
    const float hx = std::fabs(q.x - c.x), hy = std::fabs(q.y - c.y);
    if (!(hx > 0) || !(hy > 0)) return fail<SketchCurve>(why, "The corner must be off both axes through the centre");
    return curve(CurveKind::Rectangle, {{c.x - hx, c.y - hy}, {c.x + hx, c.y + hy}});
}

std::optional<SketchCurve> rectangle3Points(Vec2 a, Vec2 b, Vec2 c, std::string* why) {
    const float l = dist(a, b);
    if (!(l > 0)) return fail<SketchCurve>(why, "The first side has no length");
    const Vec2 u{(b.x - a.x) / l, (b.y - a.y) / l}, n{-u.y, u.x};
    const float w = (c.x - a.x) * n.x + (c.y - a.y) * n.y; // signed: c's side
    if (std::fabs(w) < 1e-6f * std::max(1.0f, l)) return fail<SketchCurve>(why, "The third point is on the first side: the rectangle has no width");
    const Vec2 d{n.x * w, n.y * w};
    // Counter-clockwise either way round.
    std::vector<Vec2> p = {a, b, {b.x + d.x, b.y + d.y}, {a.x + d.x, a.y + d.y}};
    if (w < 0) p = {a, {a.x + d.x, a.y + d.y}, {b.x + d.x, b.y + d.y}, b};
    p.push_back(p.front());
    return curve(CurveKind::Polyline, p);
}

std::optional<SketchCurve> circle3Points(Vec2 a, Vec2 b, Vec2 c, std::string* why) {
    Vec2 centre;
    float r;
    bool ccw;
    if (!arcThrough(a, b, c, centre, r, ccw)) return fail<SketchCurve>(why, "The three points are in line: no circle passes through them");
    return curve(CurveKind::Circle, {centre}, r);
}

std::optional<SketchCurve> circleTangentToLines(Vec2 a0, Vec2 a1, Vec2 b0, Vec2 b1, float r, Vec2 near, std::string* why) {
    if (!(r > 0)) return fail<SketchCurve>(why, "The radius must be positive");
    const float la = dist(a0, a1), lb = dist(b0, b1);
    if (!(la > 0) || !(lb > 0)) return fail<SketchCurve>(why, "A line has no length");
    const Vec2 ua{(a1.x - a0.x) / la, (a1.y - a0.y) / la}, ub{(b1.x - b0.x) / lb, (b1.y - b0.y) / lb};
    const float den = cross2(ua, ub);
    if (std::fabs(den) < 1e-6f) return fail<SketchCurve>(why, "The lines are parallel: give two that cross");
    const Vec2 na{-ua.y, ua.x}, nb{-ub.y, ub.x};
    std::optional<Vec2> best;
    for (int sa : {-1, 1})
        for (int sb : {-1, 1}) { // the lines moved r to one side each meet at a centre
            const Vec2 p{a0.x + na.x * r * static_cast<float>(sa), a0.y + na.y * r * static_cast<float>(sa)};
            const Vec2 q{b0.x + nb.x * r * static_cast<float>(sb), b0.y + nb.y * r * static_cast<float>(sb)};
            const float t = cross2({q.x - p.x, q.y - p.y}, ub) / den;
            const Vec2 c{p.x + ua.x * t, p.y + ua.y * t};
            if (!best || dist(c, near) < dist(*best, near)) best = c;
        }
    return curve(CurveKind::Circle, {*best}, r);
}

std::optional<SketchCurve> tangentArc(Vec2 s, Vec2 dir, Vec2 e, std::string* why) {
    const float l = std::hypot(dir.x, dir.y);
    if (!(l > 0)) return fail<SketchCurve>(why, "No direction to carry on");
    const Vec2 u{dir.x / l, dir.y / l}, n{-u.y, u.x}; // n: to the left of the way it goes
    const Vec2 w{s.x - e.x, s.y - e.y};
    const float nw = n.x * w.x + n.y * w.y, ww = w.x * w.x + w.y * w.y;
    if (!(ww > 0)) return fail<SketchCurve>(why, "The end is the start");
    if (std::fabs(nw) < 1e-6f * std::sqrt(ww)) return fail<SketchCurve>(why, "The end is straight ahead: a line carries on there, not an arc");
    const float k = -ww / (2.0f * nw); // the centre is k along n from the start
    const Vec2 c{s.x + n.x * k, s.y + n.y * k};
    const float r = std::fabs(k);
    // Centre on the left: it turns counter-clockwise from the start; on the right, clockwise.
    return k > 0 ? curve(CurveKind::Arc, {c, s, e}, r) : curve(CurveKind::Arc, {c, e, s}, r);
}

bool curveEndDirection(const SketchCurve& c, bool atStart, Vec2& point, Vec2& dir) {
    if (c.kind == CurveKind::Arc && c.pts.size() >= 3) {
        // Counter-clockwise from pts[1] to pts[2]: at the end it goes on that way; at the start, back.
        const Vec2 p = atStart ? c.pts[1] : c.pts[2];
        const Vec2 r{p.x - c.pts[0].x, p.y - c.pts[0].y};
        point = p;
        dir = atStart ? Vec2{r.y, -r.x} : Vec2{-r.y, r.x};
        return true;
    }
    const std::vector<Vec2> s = c.planeStrip(0.01f);
    if (s.size() < 2 || c.isClosed()) return false;
    if (atStart) point = s.front(), dir = {s[0].x - s[1].x, s[0].y - s[1].y};
    else point = s.back(), dir = {s.back().x - s[s.size() - 2].x, s.back().y - s[s.size() - 2].y};
    return std::hypot(dir.x, dir.y) > 0;
}

std::optional<SketchCurve> ellipticalArc(Vec2 centre, Vec2 majorEnd, float ry, Vec2 from, Vec2 to, std::string* why) {
    if (!(dist(centre, majorEnd) > 0) || !(ry > 0)) return fail<SketchCurve>(why, "An ellipse needs a major axis and a minor radius");
    SketchCurve c = curve(CurveKind::Ellipse, {centre, majorEnd}, ry);
    const float t0 = ellipseParam(c, from), t1 = ellipseParam(c, to);
    if (std::fabs(std::remainder(t1 - t0, 2.0f * kPi)) < 1e-4f) return fail<SketchCurve>(why, "The arc starts where it ends: draw a whole ellipse instead");
    c.pts.push_back(ellipsePointAt(c, t0));
    c.pts.push_back(ellipsePointAt(c, t1));
    return c;
}

Vec2 conicPoint(Vec2 a, Vec2 b, Vec2 apex, float rho, float t) {
    // A rational quadratic: the apex weighted rho / (1 - rho).
    const float w = rho / (1.0f - rho), s = 1.0f - t;
    const float k0 = s * s, k1 = 2.0f * s * t * w, k2 = t * t, d = k0 + k1 + k2;
    return {(a.x * k0 + apex.x * k1 + b.x * k2) / d, (a.y * k0 + apex.y * k1 + b.y * k2) / d};
}

std::optional<SketchCurve> conicCurve(Vec2 a, Vec2 b, Vec2 apex, float rho, std::string* why) {
    if (!(rho > 0.0f) || !(rho < 1.0f)) return fail<SketchCurve>(why, "Rho is between 0 and 1 (0.5: a parabola)");
    if (std::fabs(cross2({apex.x - a.x, apex.y - a.y}, {b.x - a.x, b.y - a.y})) < 1e-6f * std::max(1.0f, dist(a, b) * dist(a, b)))
        return fail<SketchCurve>(why, "The apex is in line with the ends: no conic bends through it");
    std::vector<Vec2> pts;
    // Points evenly along the curve (a heavy apex weight bunches them there by parameter).
    constexpr int kFine = 2048, kN = 32;
    std::vector<Vec2> fine;
    std::vector<float> run{0.0f};
    for (int i = 0; i <= kFine; ++i) {
        fine.push_back(conicPoint(a, b, apex, rho, static_cast<float>(i) / kFine));
        if (i > 0) run.push_back(run.back() + dist(fine[static_cast<std::size_t>(i) - 1], fine.back()));
    }
    std::size_t j = 0;
    for (int i = 0; i <= kN; ++i) {
        const float want = run.back() * static_cast<float>(i) / kN;
        while (j + 1 < run.size() && run[j + 1] < want) ++j;
        pts.push_back(i == kN ? fine.back() : fine[j]);
    }
    return curve(CurveKind::Spline, pts);
}

std::vector<SketchCurve> sketchText(const std::string& text, Vec2 at, float height, TextFont font, float angleDeg, std::string* why) {
    std::vector<SketchCurve> out;
    if (!(height > 0)) {
        if (why) *why = "The height must be positive";
        return out;
    }
    const auto loops = textOutline(text, height, font, nullptr, why);
    const float a = angleDeg * kPi / 180.0f, ca = std::cos(a), sa = std::sin(a);
    for (const std::vector<Vec2>& loop : loops) {
        if (loop.size() < 3) continue;
        SketchCurve c = curve(CurveKind::Polyline, {});
        for (Vec2 p : loop) c.pts.push_back({at.x + p.x * ca - p.y * sa, at.y + p.x * sa + p.y * ca});
        c.pts.push_back(c.pts.front());
        out.push_back(std::move(c));
    }
    if (out.empty() && why && why->empty()) *why = "No letters to draw";
    return out;
}

} // namespace sicdm

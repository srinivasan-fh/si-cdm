#include "core/SketchEdit.hpp"

#include "core/Extrude.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

namespace sicdm {

namespace {

constexpr double kTwoPi = 6.28318530717958647692;

struct P {
    double x = 0, y = 0;
};
P operator+(P a, P b) { return {a.x + b.x, a.y + b.y}; }
P operator-(P a, P b) { return {a.x - b.x, a.y - b.y}; }
P operator*(P a, double s) { return {a.x * s, a.y * s}; }
double dotp(P a, P b) { return a.x * b.x + a.y * b.y; }
double crossp(P a, P b) { return a.x * b.y - a.y * b.x; }
double lenp(P a) { return std::hypot(a.x, a.y); }
P toP(Vec2 v) { return {v.x, v.y}; }
Vec2 toV(P p) { return {static_cast<float>(p.x), static_cast<float>(p.y)}; }

// A piece of a curve: a straight segment, or an arc (counter-clockwise from a0 to a1).
struct Seg {
    bool arc = false;
    P a, b;              // the ends
    P c;                 // arc: centre
    double r = 0, a0 = 0, a1 = 0;
    P at(double t) const {
        if (!arc) return a + (b - a) * t;
        const double ang = a0 + (a1 - a0) * t;
        return {c.x + r * std::cos(ang), c.y + r * std::sin(ang)};
    }
    // The parameter of the point nearest q, and the distance to it.
    double nearest(P q, double* dist) const {
        double t;
        if (!arc) {
            const P d = b - a;
            const double l2 = dotp(d, d);
            t = l2 > 0 ? std::clamp(dotp(q - a, d) / l2, 0.0, 1.0) : 0.0;
        } else {
            double ang = std::atan2(q.y - c.y, q.x - c.x) - a0;
            ang = std::fmod(ang, kTwoPi);
            if (ang < 0) ang += kTwoPi;
            const double sweep = a1 - a0;
            if (ang <= sweep) t = ang / sweep;
            else t = (ang - sweep) < (kTwoPi - ang) ? 1.0 : 0.0; // past the end: the nearer end
        }
        if (dist) *dist = lenp(at(t) - q);
        return t;
    }
    // The parameter of an angle on the arc, or -1 off it.
    double angleParam(double ang) const {
        double d = std::fmod(ang - a0, kTwoPi);
        if (d < 0) d += kTwoPi;
        const double sweep = a1 - a0;
        if (d <= sweep + 1e-9) return std::min(d / sweep, 1.0);
        if (kTwoPi - d < 1e-9) return 0.0;
        return -1;
    }
};

Seg lineSeg(P a, P b) {
    Seg s;
    s.a = a, s.b = b;
    return s;
}
Seg arcSeg(P c, double r, double a0, double a1) {
    Seg s;
    s.arc = true;
    s.c = c, s.r = r, s.a0 = a0, s.a1 = a1;
    s.a = s.at(0), s.b = s.at(1);
    return s;
}

// A curve as pieces, in order along it; `closed` when the last ends where the first starts.
std::vector<Seg> segmentsOf(const SketchCurve& c, bool* closed) {
    std::vector<Seg> out;
    bool loop = false;
    auto poly = [&](const std::vector<Vec2>& pts) {
        for (std::size_t i = 1; i < pts.size(); ++i)
            if (std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y) > 1e-9f) out.push_back(lineSeg(toP(pts[i - 1]), toP(pts[i])));
    };
    switch (c.kind) {
    case CurveKind::Point: break;
    case CurveKind::Line:
    case CurveKind::Polyline:
        poly(c.pts);
        loop = c.isClosed();
        break;
    case CurveKind::Rectangle:
        if (c.pts.size() >= 2) {
            const Vec2 a = c.pts[0], b = c.pts[1];
            poly({a, {b.x, a.y}, b, {a.x, b.y}, a});
            loop = true;
        }
        break;
    case CurveKind::Circle:
        if (!c.pts.empty()) out.push_back(arcSeg(toP(c.pts[0]), c.radius, 0, kTwoPi));
        loop = true;
        break;
    case CurveKind::Arc:
        if (c.pts.size() >= 3) {
            float a0, a1;
            c.arcAngles(a0, a1);
            out.push_back(arcSeg(toP(c.pts[0]), std::hypot(c.pts[1].x - c.pts[0].x, c.pts[1].y - c.pts[0].y), a0, a1));
        }
        break;
    case CurveKind::Slot:
        if (c.pts.size() >= 2) {
            const P a = toP(c.pts[0]), b = toP(c.pts[1]);
            const double ang = std::atan2(b.y - a.y, b.x - a.x), r = c.radius;
            const P n{-std::sin(ang) * r, std::cos(ang) * r};
            out.push_back(arcSeg(b, r, ang - kTwoPi / 4, ang + kTwoPi / 4));
            out.push_back(lineSeg(b + n, a + n));
            out.push_back(arcSeg(a, r, ang + kTwoPi / 4, ang + 3 * kTwoPi / 4));
            out.push_back(lineSeg(a - n, b - n));
            loop = true;
        }
        break;
    case CurveKind::Ellipse:
    case CurveKind::Spline:
        poly(c.planeStrip(0.005f));
        loop = c.isClosed();
        break;
    }
    if (closed) *closed = loop;
    return out;
}

// Where two pieces cross: (t on a, t on b) pairs.
void intersect(const Seg& p, const Seg& q, std::vector<std::pair<double, double>>& out) {
    const double eps = 1e-9;
    if (!p.arc && !q.arc) {
        const P d1 = p.b - p.a, d2 = q.b - q.a;
        const double den = crossp(d1, d2);
        if (std::fabs(den) < 1e-14 * std::max(dotp(d1, d1) * dotp(d2, d2), 1e-30)) return; // parallel
        const double t = crossp(q.a - p.a, d2) / den, u = crossp(q.a - p.a, d1) / den;
        if (t >= -eps && t <= 1 + eps && u >= -eps && u <= 1 + eps) out.push_back({std::clamp(t, 0.0, 1.0), std::clamp(u, 0.0, 1.0)});
        return;
    }
    if (p.arc && !q.arc) {
        std::vector<std::pair<double, double>> sw;
        intersect(q, p, sw);
        for (auto [a, b] : sw) out.push_back({b, a});
        return;
    }
    if (!p.arc && q.arc) { // segment p against circle q
        const P d = p.b - p.a, f = p.a - q.c;
        const double A = dotp(d, d), B = 2 * dotp(f, d), C = dotp(f, f) - q.r * q.r;
        const double disc = B * B - 4 * A * C;
        if (A <= 0 || disc < -1e-12 * A * q.r * q.r) return;
        const double sq = std::sqrt(std::max(disc, 0.0));
        for (double t : {(-B - sq) / (2 * A), (-B + sq) / (2 * A)}) {
            if (t < -eps || t > 1 + eps) continue;
            const P x = p.at(std::clamp(t, 0.0, 1.0));
            const double u = q.angleParam(std::atan2(x.y - q.c.y, x.x - q.c.x));
            if (u >= 0) out.push_back({std::clamp(t, 0.0, 1.0), u});
            if (disc <= 0) break; // touching: once
        }
        return;
    }
    // Two circles.
    const P d = q.c - p.c;
    const double dist = lenp(d);
    if (dist < 1e-12 || dist > p.r + q.r + 1e-9 || dist < std::fabs(p.r - q.r) - 1e-9) return;
    const double a = (p.r * p.r - q.r * q.r + dist * dist) / (2 * dist);
    const double h = std::sqrt(std::max(p.r * p.r - a * a, 0.0));
    const P m = p.c + d * (a / dist), n{-d.y / dist, d.x / dist};
    for (int s : {-1, 1}) {
        const P x = m + n * (h * s);
        const double t = p.angleParam(std::atan2(x.y - p.c.y, x.x - p.c.x)), u = q.angleParam(std::atan2(x.y - q.c.y, x.x - q.c.x));
        if (t >= 0 && u >= 0) out.push_back({t, u});
        if (h <= 0) break;
    }
}

// A run of a curve's pieces, from s0 to s1 (piece index plus the parameter along it;
// past the end of a closed curve it carries on round).
std::vector<Seg> piecesBetween(const std::vector<Seg>& segs, double s0, double s1) {
    std::vector<Seg> out;
    const double L = static_cast<double>(segs.size());
    for (double s = s0; s < s1 - 1e-9;) {
        const double k = std::floor(s + 1e-12);
        const double next = std::min(k + 1, s1);
        const Seg& g = segs[static_cast<std::size_t>(std::fmod(k, L) + (std::fmod(k, L) < 0 ? L : 0))];
        const double t0 = s - k, t1 = next - k;
        if (t1 - t0 > 1e-9) {
            if (g.arc) out.push_back(arcSeg(g.c, g.r, g.a0 + (g.a1 - g.a0) * t0, g.a0 + (g.a1 - g.a0) * t1));
            else out.push_back(lineSeg(g.at(t0), g.at(t1)));
        }
        s = next;
    }
    return out;
}

SketchCurve like(const SketchCurve& src) {
    SketchCurve c;
    c.plane = src.plane;
    c.visible = src.visible;
    c.construction = src.construction;
    return c;
}

// Curves for a run of pieces cut from `src`: lines joined into polylines, arcs as arcs,
// and the sampled curves (ellipses, splines) as splines through their points.
void emitPieces(const SketchCurve& src, const std::vector<Seg>& pieces, std::vector<SketchCurve>& out) {
    if (pieces.empty()) return;
    if (src.kind == CurveKind::Ellipse || src.kind == CurveKind::Spline) {
        SketchCurve c = like(src);
        c.kind = CurveKind::Spline;
        // Every few sample points: enough to follow the shape, few enough to edit.
        const std::size_t stride = std::max<std::size_t>(1, pieces.size() / 24);
        c.pts.push_back(toV(pieces.front().a));
        for (std::size_t i = stride; i < pieces.size(); i += stride) c.pts.push_back(toV(pieces[i].a));
        c.pts.push_back(toV(pieces.back().b));
        if (c.pts.size() >= 2) out.push_back(c);
        return;
    }
    std::vector<Vec2> run; // consecutive straight pieces
    auto flush = [&] {
        if (run.size() >= 2) {
            SketchCurve c = like(src);
            c.kind = run.size() == 2 ? CurveKind::Line : CurveKind::Polyline;
            c.pts = run;
            out.push_back(c);
        }
        run.clear();
    };
    for (const Seg& g : pieces) {
        if (g.arc) {
            flush();
            SketchCurve c = like(src);
            c.kind = CurveKind::Arc;
            c.pts = {toV(g.c), toV(g.at(0)), toV(g.at(1))};
            c.radius = static_cast<float>(g.r);
            out.push_back(c);
        } else {
            if (run.empty()) run.push_back(toV(g.a));
            run.push_back(toV(g.b));
        }
    }
    flush();
}

// The parameter (piece + t) of the point on the curve nearest q.
double nearestParam(const std::vector<Seg>& segs, P q) {
    double best = 1e300, s = 0;
    for (std::size_t i = 0; i < segs.size(); ++i) {
        double d;
        const double t = segs[i].nearest(q, &d);
        if (d < best) best = d, s = static_cast<double>(i) + t;
    }
    return s;
}

bool fail(std::string* why, const std::string& text) {
    if (why) *why = text;
    return false;
}

} // namespace

std::vector<Vec2> curveIntersections(const SketchCurve& a, const SketchCurve& b) {
    std::vector<Vec2> out;
    if (a.plane != b.plane) return out;
    const std::vector<Seg> sa = segmentsOf(a, nullptr), sb = segmentsOf(b, nullptr);
    for (const Seg& p : sa)
        for (const Seg& q : sb) {
            std::vector<std::pair<double, double>> hits;
            intersect(p, q, hits);
            for (auto [t, u] : hits) {
                const Vec2 x = toV(p.at(t));
                if (std::none_of(out.begin(), out.end(), [&](Vec2 o) { return std::hypot(o.x - x.x, o.y - x.y) < 1e-6f; })) out.push_back(x);
            }
        }
    return out;
}

bool trimCurve(const Sketch& sketch, std::size_t index, Vec2 pick, std::vector<SketchCurve>& out, std::string* why) {
    out.clear();
    const auto& curves = sketch.curves();
    if (index >= curves.size()) return fail(why, "No such curve");
    const SketchCurve& c = curves[index];
    bool closed = false;
    const std::vector<Seg> segs = segmentsOf(c, &closed);
    if (segs.empty()) return fail(why, c.name + " has nothing to trim");
    const double L = static_cast<double>(segs.size());
    // Where the other curves cross it.
    std::vector<double> cuts;
    for (std::size_t j = 0; j < curves.size(); ++j) {
        if (j == index || curves[j].plane != c.plane || !curves[j].visible) continue;
        const std::vector<Seg> other = segmentsOf(curves[j], nullptr);
        for (std::size_t i = 0; i < segs.size(); ++i)
            for (const Seg& q : other) {
                std::vector<std::pair<double, double>> hits;
                intersect(segs[i], q, hits);
                for (auto [t, u] : hits) cuts.push_back(static_cast<double>(i) + t);
            }
    }
    std::sort(cuts.begin(), cuts.end());
    cuts.erase(std::unique(cuts.begin(), cuts.end(), [](double a, double b) { return b - a < 1e-7; }), cuts.end());
    if (closed && !cuts.empty() && cuts.back() > L - 1e-7 && cuts.front() < 1e-7) cuts.pop_back(); // the start, met twice
    const double sp = nearestParam(segs, toP(pick));
    if (!closed) {
        // Cuts at the very ends don't split anything.
        std::erase_if(cuts, [&](double s) { return s < 1e-7 || s > L - 1e-7; });
        double lo = 0, hi = L;
        for (double s : cuts) {
            if (s < sp) lo = s;
            else if (s > sp) {
                hi = s;
                break;
            }
        }
        if (lo > 1e-7) emitPieces(c, piecesBetween(segs, 0, lo), out);
        if (hi < L - 1e-7) emitPieces(c, piecesBetween(segs, hi, L), out);
        return true;
    }
    if (cuts.size() < 2) return true; // a loop crossed once or not at all: all of it goes
    // The cuts either side of the pick (round the loop if need be).
    double lo = cuts.back() - L, hi = cuts.front() + L;
    for (double s : cuts) {
        if (s < sp) lo = s;
        else if (s > sp) {
            hi = s;
            break;
        }
    }
    // What stays: from the cut after the pick round to the one before it.
    emitPieces(c, piecesBetween(segs, hi, lo + L), out);
    return true;
}

bool extendCurve(const Sketch& sketch, std::size_t index, Vec2 pick, SketchCurve& out, std::string* why) {
    const auto& curves = sketch.curves();
    if (index >= curves.size()) return fail(why, "No such curve");
    const SketchCurve& c = curves[index];
    const P q = toP(pick);
    // Every piece of the other curves on the plane.
    std::vector<Seg> others;
    for (std::size_t j = 0; j < curves.size(); ++j)
        if (j != index && curves[j].plane == c.plane && curves[j].visible)
            for (const Seg& g : segmentsOf(curves[j], nullptr)) others.push_back(g);
    out = c;
    if (c.kind == CurveKind::Line || (c.kind == CurveKind::Polyline && !c.isClosed() && c.pts.size() >= 2)) {
        const std::size_t n = c.pts.size();
        const bool atEnd = lenp(toP(c.pts[n - 1]) - q) < lenp(toP(c.pts[0]) - q);
        const P end = toP(atEnd ? c.pts[n - 1] : c.pts[0]), prev = toP(atEnd ? c.pts[n - 2] : c.pts[1]);
        const P d = end - prev;
        const double l = lenp(d);
        if (l <= 0) return fail(why, c.name + " has no direction to go on in");
        // A long ray on from the end.
        const Seg ray = lineSeg(end, end + d * (1e6 / l));
        double best = 1e300;
        for (const Seg& g : others) {
            std::vector<std::pair<double, double>> hits;
            intersect(ray, g, hits);
            for (auto [t, u] : hits)
                if (t * 1e6 > 1e-6) best = std::min(best, t);
        }
        if (best >= 1e300) return fail(why, "Nothing beyond that end of " + c.name + " to reach");
        out.pts[atEnd ? n - 1 : 0] = toV(ray.at(best));
        return true;
    }
    if (c.kind == CurveKind::Arc) {
        float a0, a1;
        c.arcAngles(a0, a1);
        const P centre = toP(c.pts[0]);
        const double r = std::hypot(c.pts[1].x - c.pts[0].x, c.pts[1].y - c.pts[0].y);
        const bool atEnd = lenp(toP(c.pts[2]) - q) < lenp(toP(c.pts[1]) - q);
        // Round the circle from the end, away from the arc.
        const Seg rest = atEnd ? arcSeg(centre, r, a1, a0 + kTwoPi) : arcSeg(centre, r, a1 - kTwoPi, a0);
        double best = atEnd ? 1e300 : -1e300;
        for (const Seg& g : others) {
            std::vector<std::pair<double, double>> hits;
            intersect(rest, g, hits);
            for (auto [t, u] : hits) {
                if (atEnd && t > 1e-9) best = std::min(best, t);
                if (!atEnd && t < 1 - 1e-9) best = std::max(best, t);
            }
        }
        if (std::fabs(best) >= 1e300) return fail(why, "Nothing round the circle from that end of " + c.name + " to reach");
        const P x = rest.at(best);
        out.pts[atEnd ? 2 : 1] = toV(x);
        return true;
    }
    return fail(why, "EXTEND carries on lines, arcs and open polylines");
}

bool offsetCurve(const SketchCurve& c, float distanceF, Vec2 sideV, SketchCurve& out, std::string* why) {
    const double d = std::fabs(static_cast<double>(distanceF));
    if (!(d > 0)) return fail(why, "The offset distance must be more than 0");
    const P side = toP(sideV);
    out = c;
    out.name.clear();
    auto shrinkOrGrow = [&](double r, bool inside) -> double { return inside ? r - d : r + d; };
    switch (c.kind) {
    case CurveKind::Point: return fail(why, "A point has no offset");
    case CurveKind::Circle:
    case CurveKind::Arc: {
        const P centre = toP(c.pts[0]);
        const double r = c.kind == CurveKind::Circle ? c.radius : std::hypot(c.pts[1].x - c.pts[0].x, c.pts[1].y - c.pts[0].y);
        const double nr = shrinkOrGrow(r, lenp(side - centre) < r);
        if (!(nr > 1e-6)) return fail(why, "That offset would close up the curve: the distance is more than its radius");
        if (c.kind == CurveKind::Circle) out.radius = static_cast<float>(nr);
        else {
            for (std::size_t k = 1; k <= 2; ++k) {
                const P v = toP(c.pts[k]) - centre;
                out.pts[k] = toV(centre + v * (nr / std::max(lenp(v), 1e-12)));
            }
            out.radius = static_cast<float>(nr);
        }
        return true;
    }
    case CurveKind::Slot: {
        const P a = toP(c.pts[0]), b = toP(c.pts[1]), ab = b - a;
        const double t = std::clamp(dotp(side - a, ab) / std::max(dotp(ab, ab), 1e-12), 0.0, 1.0);
        const double nr = shrinkOrGrow(c.radius, lenp(side - (a + ab * t)) < c.radius);
        if (!(nr > 1e-6)) return fail(why, "That offset would close up the slot");
        out.radius = static_cast<float>(nr);
        return true;
    }
    case CurveKind::Rectangle: {
        const float x0 = std::min(c.pts[0].x, c.pts[1].x), x1 = std::max(c.pts[0].x, c.pts[1].x);
        const float y0 = std::min(c.pts[0].y, c.pts[1].y), y1 = std::max(c.pts[0].y, c.pts[1].y);
        const bool inside = side.x > x0 && side.x < x1 && side.y > y0 && side.y < y1;
        const float k = static_cast<float>(inside ? -d : d);
        if (x1 - x0 + 2 * k <= 0 || y1 - y0 + 2 * k <= 0) return fail(why, "That offset would close up the rectangle");
        out.pts = {{x0 - k, y0 - k}, {x1 + k, y1 + k}};
        return true;
    }
    case CurveKind::Line:
    case CurveKind::Polyline: {
        std::vector<Vec2> pts = c.pts;
        const bool closedLoop = c.isClosed();
        if (closedLoop) pts.pop_back();
        const std::size_t n = pts.size();
        if (n < 2) return fail(why, c.name + " is too short to offset");
        // Which way: left of the nearest piece, or into a closed loop's inside.
        double sign = 1;
        if (closedLoop) {
            const bool inside = [&] {
                bool in = false;
                for (std::size_t i = 0, j = n - 1; i < n; j = i++)
                    if ((pts[i].y > side.y) != (pts[j].y > side.y) &&
                        side.x < (pts[j].x - pts[i].x) * (side.y - pts[i].y) / (pts[j].y - pts[i].y) + pts[i].x)
                        in = !in;
                return in;
            }();
            const bool ccw = signedArea(pts) > 0;
            sign = inside == ccw ? 1 : -1; // inside of a counter-clockwise loop is on its left
        } else {
            const std::vector<Seg> segs = segmentsOf(c, nullptr);
            const double sp = nearestParam(segs, side);
            const Seg& g = segs[std::min(static_cast<std::size_t>(sp), segs.size() - 1)];
            sign = crossp(g.b - g.a, side - g.a) >= 0 ? 1 : -1;
        }
        // Each piece moved to the side, the corners where the moved pieces meet.
        const std::size_t m = closedLoop ? n : n - 1;
        std::vector<std::pair<P, P>> lines;
        for (std::size_t i = 0; i < m; ++i) {
            const P a = toP(pts[i]), b = toP(pts[(i + 1) % n]), dir = b - a;
            const double l = lenp(dir);
            if (l <= 0) continue;
            const P nrm{-dir.y / l * d * sign, dir.x / l * d * sign};
            lines.push_back({a + nrm, b + nrm});
        }
        auto meet = [](std::pair<P, P> l1, std::pair<P, P> l2) {
            const P d1 = l1.second - l1.first, d2 = l2.second - l2.first;
            const double den = crossp(d1, d2);
            if (std::fabs(den) < 1e-12 * lenp(d1) * lenp(d2)) return l1.second; // in line
            return l1.first + d1 * (crossp(l2.first - l1.first, d2) / den);
        };
        std::vector<Vec2> o;
        const std::size_t k = lines.size();
        for (std::size_t i = 0; i < k; ++i) {
            if (!closedLoop && i == 0) o.push_back(toV(lines[0].first));
            if (closedLoop || i > 0) o.push_back(toV(meet(lines[(i + k - 1) % k], lines[i])));
        }
        if (!closedLoop) o.push_back(toV(lines.back().second));
        else o.push_back(o.front());
        out.pts = o;
        if (out.kind == CurveKind::Line && o.size() != 2) out.kind = CurveKind::Polyline;
        return true;
    }
    case CurveKind::Ellipse:
    case CurveKind::Spline: {
        // Sampled and moved along the normal: a spline through the moved points.
        std::vector<Vec2> s = c.planeStrip(0.01f);
        const bool loop = c.isClosed();
        if (loop && s.size() > 2) s.pop_back();
        const std::size_t n = s.size();
        if (n < 2) return fail(why, c.name + " is too short to offset");
        double sign = 1;
        {
            double best = 1e300;
            for (std::size_t i = 0; i + 1 < n; ++i) {
                const double dd = lenp(toP(s[i]) - side);
                if (dd < best) best = dd, sign = crossp(toP(s[i + 1]) - toP(s[i]), side - toP(s[i])) >= 0 ? 1 : -1;
            }
        }
        std::vector<Vec2> moved;
        for (std::size_t i = 0; i < n; ++i) {
            const P prev = toP(s[loop ? (i + n - 1) % n : (i == 0 ? 0 : i - 1)]), next = toP(s[loop ? (i + 1) % n : std::min(i + 1, n - 1)]);
            const P t = next - prev;
            const double l = lenp(t);
            if (l <= 0) continue;
            moved.push_back(toV(toP(s[i]) + P{-t.y / l, t.x / l} * (d * sign)));
        }
        out.kind = CurveKind::Spline;
        out.closed = loop;
        out.radius = 0;
        const std::size_t stride = std::max<std::size_t>(1, moved.size() / 48);
        out.pts.clear();
        for (std::size_t i = 0; i < moved.size(); i += stride) out.pts.push_back(moved[i]);
        if (!loop && std::hypot(out.pts.back().x - moved.back().x, out.pts.back().y - moved.back().y) > 1e-6f) out.pts.push_back(moved.back());
        return out.pts.size() >= 2;
    }
    }
    return fail(why, "Can't offset that");
}

std::vector<Vec2> chainLoop(const Sketch& sketch, std::size_t index, std::vector<std::size_t>* members, float tol, float chord) {
    const auto& curves = sketch.curves();
    if (index >= curves.size()) return {};
    auto usable = [&](const SketchCurve& c) {
        return c.visible && !c.construction && c.plane == curves[index].plane && !c.isClosed() &&
               (c.kind == CurveKind::Line || c.kind == CurveKind::Arc || c.kind == CurveKind::Polyline || c.kind == CurveKind::Spline);
    };
    if (!usable(curves[index])) return {};
    // Each usable curve's points along it, start to end.
    std::vector<std::vector<Vec2>> strips(curves.size());
    for (std::size_t i = 0; i < curves.size(); ++i)
        if (usable(curves[i])) strips[i] = curves[i].planeStrip(chord);
    auto near = [&](Vec2 a, Vec2 b) { return std::hypot(a.x - b.x, a.y - b.y) <= tol; };
    // Depth-first round the joins until back at the start.
    std::vector<std::pair<std::size_t, bool>> path{{index, false}}; // curve, reversed
    std::vector<char> used(curves.size(), 0);
    used[index] = 1;
    const Vec2 start = strips[index].front();
    std::function<bool(Vec2)> walk = [&](Vec2 at) -> bool {
        if (path.size() > 1 && near(at, start)) return true;
        if (path.size() > 256) return false;
        for (std::size_t j = 0; j < curves.size(); ++j) {
            if (used[j] || strips[j].size() < 2) continue;
            for (bool rev : {false, true}) {
                const Vec2 from = rev ? strips[j].back() : strips[j].front(), to = rev ? strips[j].front() : strips[j].back();
                if (!near(from, at)) continue;
                used[j] = 1;
                path.push_back({j, rev});
                if (walk(to)) return true;
                path.pop_back();
                used[j] = 0;
            }
        }
        return false;
    };
    if (!walk(strips[index].back())) return {};
    std::vector<Vec2> poly;
    for (const auto& [j, rev] : path) {
        std::vector<Vec2> s = strips[j];
        if (rev) std::reverse(s.begin(), s.end());
        for (std::size_t k = poly.empty() ? 0 : 1; k < s.size(); ++k) poly.push_back(s[k]);
        if (members) members->push_back(j);
    }
    if (poly.size() > 1 && near(poly.front(), poly.back())) poly.pop_back();
    if (poly.size() < 3) return {};
    if (signedArea(poly) < 0) std::reverse(poly.begin(), poly.end());
    return poly;
}

} // namespace sicdm

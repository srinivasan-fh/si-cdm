#include "core/Drawing.hpp"
#include "core/Tolerances.hpp"

#include "core/Json.hpp"
#include "core/Scene.hpp"
#include "core/Sculpt.hpp"
#include "core/Section.hpp"
#include "core/Step.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <limits>
#include <map>
#include <set>
#include <unordered_map>

namespace sicdm {

namespace {

constexpr double kPiD2 = 3.14159265358979323846;

struct ViewFrame {
    DVec3 r, u, d; // right, up, and into the scene (away from the viewer)
};

ViewFrame frameOf(ViewKind k) {
    switch (k) {
    case ViewKind::Front: return {{1, 0, 0}, {0, 0, 1}, {0, 1, 0}};
    case ViewKind::Back: return {{-1, 0, 0}, {0, 0, 1}, {0, -1, 0}};
    case ViewKind::Top: return {{1, 0, 0}, {0, 1, 0}, {0, 0, -1}};
    case ViewKind::Bottom: return {{1, 0, 0}, {0, -1, 0}, {0, 0, 1}};
    case ViewKind::Right: return {{0, 1, 0}, {0, 0, 1}, {-1, 0, 0}};
    case ViewKind::Left: return {{0, -1, 0}, {0, 0, 1}, {1, 0, 0}};
    case ViewKind::Iso: {
        const DVec3 d = dnormalize({-1, 1, -1});
        const DVec3 r = dnormalize(dcross(d, {0, 0, 1}));
        return {r, dcross(r, d), d};
    }
    case ViewKind::Section: break;
    }
    return {{1, 0, 0}, {0, 0, 1}, {0, 1, 0}};
}

std::string upper(std::string s) {
    for (char& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

std::string num(double v) {
    char b[32];
    std::snprintf(b, sizeof b, "%.2f", v);
    std::string s = b;
    while (!s.empty() && s.back() == '0') s.pop_back();
    if (!s.empty() && s.back() == '.') s.pop_back();
    if (s == "-0") s = "0";
    return s;
}

// --- the model, as triangles with their faces -----------------------------------------------

struct Model {
    std::vector<DVec3> p;                 // welded positions
    std::vector<std::array<int, 3>> tris;
    std::vector<int> face;                // exact face id, or -1 (a mesh body)
    std::vector<DVec3> normal;            // per triangle
    struct Edge {
        int a, b, t0, t1; // t1 = -1: an open edge
    };
    std::vector<Edge> edges;
    struct Round { // a cylinder face of an exact body
        DVec3 o, axis;
        double r, h0, h1;
        bool full;
    };
    std::vector<Round> rounds;
    DVec3 lo{1e300, 1e300, 1e300}, hi{-1e300, -1e300, -1e300};
    bool valid() const { return !tris.empty(); }
    DVec3 centre() const { return (lo + hi) * 0.5; }
    double diag() const { return dlength(hi - lo); }
};

std::vector<const Body*> drawnBodies(const Drawing& d, const Scene& scene) {
    std::vector<const Body*> out;
    for (const Body& b : scene.bodies()) {
        if (b.mesh.indices.empty()) continue;
        if (d.bodies.empty() ? !b.visible : std::find(d.bodies.begin(), d.bodies.end(), b.name) == d.bodies.end()) continue;
        out.push_back(&b);
    }
    return out;
}

Model buildModel(const std::vector<const Body*>& bodies, double chord) {
    Model m;
    int faceBase = 0;
    std::map<std::tuple<float, float, float>, int> weld;
    auto vid = [&](const Vec3& q) {
        const auto [it, added] = weld.try_emplace({q.x, q.y, q.z}, static_cast<int>(m.p.size()));
        if (added) m.p.push_back(DVec3(q));
        return it->second;
    };
    for (const Body* b : bodies) {
        const auto ex = exactBrep(*b);
        MeshData mesh;
        std::vector<int> triFace;
        if (ex) {
            Brep br = *ex;
            Placement t;
            t.t = decimal(b->offset);
            br.transform(t);
            TessellateOptions o;
            o.chord = chord;
            o.angle = 0.2;
            TessellateReport rep;
            mesh = tessellate(br, o, &rep);
            for (std::size_t f = 0; f < rep.faceTriangles.size(); ++f) triFace.insert(triFace.end(), rep.faceTriangles[f], faceBase + static_cast<int>(f));
            // Cylinders: centre lines and marks.
            std::size_t fi = 0;
            std::size_t tri0 = 0;
            for (const BShell& sh : br.shells)
                for (const BFace& f : sh.faces) {
                    const std::size_t nt = fi < rep.faceTriangles.size() ? rep.faceTriangles[fi] : 0;
                    const BSurface& S = br.surfaces[static_cast<std::size_t>(f.surface)];
                    if (S.kind == BSurfaceKind::Cylinder && nt > 0) {
                        Model::Round rd{S.f.o, S.f.z, S.r1, 1e300, -1e300, false};
                        bool quad[4] = {false, false, false, false};
                        for (std::size_t ti = tri0; ti < tri0 + nt; ++ti)
                            for (int k = 0; k < 3; ++k) {
                                const DVec3 q(mesh.vertices[mesh.indices[ti * 3 + static_cast<std::size_t>(k)]].position);
                                const double h = ddot(q - S.f.o, S.f.z);
                                rd.h0 = std::min(rd.h0, h), rd.h1 = std::max(rd.h1, h);
                                const double a = std::atan2(ddot(q - S.f.o, S.f.y), ddot(q - S.f.o, S.f.x));
                                quad[static_cast<int>(std::floor((a + kPiD2) / (kPiD2 / 2))) & 3] = true;
                            }
                        rd.full = quad[0] && quad[1] && quad[2] && quad[3];
                        if (rd.full) m.rounds.push_back(rd);
                    }
                    tri0 += nt;
                    ++fi;
                }
            faceBase += static_cast<int>(rep.faceTriangles.size());
        } else {
            mesh = worldMesh(*b);
        }
        for (std::size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
            const std::array<int, 3> tr{vid(mesh.vertices[mesh.indices[t]].position), vid(mesh.vertices[mesh.indices[t + 1]].position),
                                        vid(mesh.vertices[mesh.indices[t + 2]].position)};
            if (tr[0] == tr[1] || tr[1] == tr[2] || tr[2] == tr[0]) continue;
            m.tris.push_back(tr);
            m.face.push_back(ex && t / 3 < triFace.size() ? triFace[t / 3] : -1);
        }
    }
    for (const DVec3& q : m.p) {
        m.lo = {std::min(m.lo.x, q.x), std::min(m.lo.y, q.y), std::min(m.lo.z, q.z)};
        m.hi = {std::max(m.hi.x, q.x), std::max(m.hi.y, q.y), std::max(m.hi.z, q.z)};
    }
    m.normal.resize(m.tris.size());
    std::unordered_map<std::uint64_t, std::size_t> edgeAt;
    for (std::size_t t = 0; t < m.tris.size(); ++t) {
        const auto& tr = m.tris[t];
        m.normal[t] = dnormalize(dcross(m.p[static_cast<std::size_t>(tr[1])] - m.p[static_cast<std::size_t>(tr[0])], m.p[static_cast<std::size_t>(tr[2])] - m.p[static_cast<std::size_t>(tr[0])]));
        for (int k = 0; k < 3; ++k) {
            int a = tr[static_cast<std::size_t>(k)], c = tr[static_cast<std::size_t>((k + 1) % 3)];
            if (a > c) std::swap(a, c);
            const std::uint64_t key = (static_cast<std::uint64_t>(static_cast<std::uint32_t>(a)) << 32) | static_cast<std::uint32_t>(c);
            const auto it = edgeAt.find(key);
            if (it == edgeAt.end()) {
                edgeAt[key] = m.edges.size();
                m.edges.push_back({a, c, static_cast<int>(t), -1});
            } else if (m.edges[it->second].t1 < 0) {
                m.edges[it->second].t1 = static_cast<int>(t);
            }
        }
    }
    return m;
}

// --- hidden lines: a depth buffer --------------------------------------------------------

class DepthBuffer {
public:
    DepthBuffer(double x0, double y0, double x1, double y1, double px) : m_x0(x0), m_y0(y0), m_px(px) {
        m_w = std::max(1, static_cast<int>(std::ceil((x1 - x0) / px)) + 3);
        m_h = std::max(1, static_cast<int>(std::ceil((y1 - y0) / px)) + 3);
        m_z.assign(static_cast<std::size_t>(m_w) * static_cast<std::size_t>(m_h), std::numeric_limits<float>::infinity());
    }
    void triangle(const DVec3& a, const DVec3& b, const DVec3& c) { // (x, y, depth)
        const double ax = (a.x - m_x0) / m_px, ay = (a.y - m_y0) / m_px, bx = (b.x - m_x0) / m_px, by = (b.y - m_y0) / m_px, cx = (c.x - m_x0) / m_px,
                     cy = (c.y - m_y0) / m_px;
        const double area = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
        if (std::fabs(area) < 1e-12) return;
        const int i0 = std::max(0, static_cast<int>(std::floor(std::min({ax, bx, cx})))), i1 = std::min(m_w - 1, static_cast<int>(std::ceil(std::max({ax, bx, cx}))));
        const int j0 = std::max(0, static_cast<int>(std::floor(std::min({ay, by, cy})))), j1 = std::min(m_h - 1, static_cast<int>(std::ceil(std::max({ay, by, cy}))));
        for (int j = j0; j <= j1; ++j)
            for (int i = i0; i <= i1; ++i) {
                const double x = i + 0.5, y = j + 0.5;
                double w0 = ((bx - x) * (cy - y) - (by - y) * (cx - x)) / area;
                double w1 = ((cx - x) * (ay - y) - (cy - y) * (ax - x)) / area;
                double w2 = 1 - w0 - w1;
                if (w0 < -1e-9 || w1 < -1e-9 || w2 < -1e-9) continue;
                const float z = static_cast<float>(w0 * a.z + w1 * b.z + w2 * c.z);
                float& cell = m_z[static_cast<std::size_t>(j) * static_cast<std::size_t>(m_w) + static_cast<std::size_t>(i)];
                if (z < cell) cell = z;
            }
    }
    // A flat region (even-odd loops) at one depth: a section's cut face.
    void region(const std::vector<std::vector<std::pair<double, double>>>& loops, double depth) {
        for (int j = 0; j < m_h; ++j) {
            const double y = m_y0 + (j + 0.5) * m_px;
            std::vector<double> xs;
            for (const auto& l : loops)
                for (std::size_t k = 0; k < l.size(); ++k) {
                    const auto& p = l[k];
                    const auto& q = l[(k + 1) % l.size()];
                    if ((p.second > y) != (q.second > y)) xs.push_back(p.first + (y - p.second) / (q.second - p.second) * (q.first - p.first));
                }
            std::sort(xs.begin(), xs.end());
            for (std::size_t k = 0; k + 1 < xs.size(); k += 2) {
                const int i0 = std::max(0, static_cast<int>(std::ceil((xs[k] - m_x0) / m_px - 0.5)));
                const int i1 = std::min(m_w - 1, static_cast<int>(std::floor((xs[k + 1] - m_x0) / m_px - 0.5)));
                for (int i = i0; i <= i1; ++i) {
                    float& cell = m_z[static_cast<std::size_t>(j) * static_cast<std::size_t>(m_w) + static_cast<std::size_t>(i)];
                    cell = std::min(cell, static_cast<float>(depth));
                }
            }
        }
    }
    // Something clearly in front of (x, y, depth), all round it.
    bool hidden(double x, double y, double depth, double eps) const {
        const int ci = static_cast<int>(std::floor((x - m_x0) / m_px)), cj = static_cast<int>(std::floor((y - m_y0) / m_px));
        for (int j = cj - 1; j <= cj + 1; ++j)
            for (int i = ci - 1; i <= ci + 1; ++i) {
                if (i < 0 || j < 0 || i >= m_w || j >= m_h) return false;
                if (!(m_z[static_cast<std::size_t>(j) * static_cast<std::size_t>(m_w) + static_cast<std::size_t>(i)] < depth - eps)) return false;
            }
        return true;
    }

private:
    double m_x0, m_y0, m_px;
    int m_w = 1, m_h = 1;
    std::vector<float> m_z;
};

// --- one view -----------------------------------------------------------------------------

struct ViewSetup {
    ViewFrame f;
    DVec3 keepN{0, 0, 0}; // sections: keep dot(p, keepN) <= keepC
    double keepC = 0;
    bool section = false;
};

bool setupView(const Drawing& d, const DrawingView& v, ViewSetup& s) {
    if (v.kind != ViewKind::Section) {
        s.f = frameOf(v.kind);
        return true;
    }
    const DrawingView* p = d.view(v.parent);
    if (!p || p->kind == ViewKind::Section || p->kind == ViewKind::Iso) return false;
    const ViewFrame pf = frameOf(p->kind);
    const double sg = v.flip ? -1.0 : 1.0;
    const DVec3 axis = v.vertical ? pf.r : pf.u;
    // The viewer on the cut-away side, looking at what is left.
    s.keepN = axis * sg;
    s.keepC = v.cut * sg;
    s.f.d = axis * -sg;
    const DVec3 n = s.keepN;
    if (v.vertical) {
        s.f.u = pf.u;
        s.f.r = dcross(s.f.u, n);
    } else {
        s.f.r = pf.r;
        s.f.u = dcross(n, s.f.r);
        // Cut from a top or bottom view: the part the right way up (turned half round, not mirrored).
        if (s.f.u.z < -0.5) s.f.u = s.f.u * -1.0, s.f.r = s.f.r * -1.0;
    }
    s.section = true;
    return true;
}

// Projects a model point into a view on the sheet.
struct Projector {
    ViewFrame f;
    double cx, cy, sx, sy, scale;
    std::pair<double, double> operator()(const DVec3& p) const { return {sx + scale * (ddot(p, f.r) - cx), sy + scale * (ddot(p, f.u) - cy)}; }
};

Projector projectorFor(const Drawing& d, const DrawingView& v, const ViewFrame& f, const DVec3& centre) {
    const double s = v.scale > 0 ? v.scale : d.scale;
    return {f, ddot(centre, f.r), ddot(centre, f.u), v.x, v.y, s};
}

void addArrow(SheetGraphics& g, double tx, double ty, double dx, double dy, double len = 3.0, double half = 0.6) {
    const double l = std::hypot(dx, dy);
    if (!(l > 0)) return;
    dx /= l, dy /= l;
    const double bx = tx - dx * len, by = ty - dy * len;
    g.fills.push_back({{tx, ty}, {bx - dy * half, by + dx * half}, {bx + dy * half, by - dx * half}});
}

// Clips triangle (a, b, c) to dot(p, n) <= c0: the pieces as triangles.
void clipTriangle(const DVec3& a, const DVec3& b, const DVec3& c, const DVec3& n, double c0, std::vector<std::array<DVec3, 3>>& out) {
    std::vector<DVec3> poly{a, b, c}, res;
    for (std::size_t i = 0; i < poly.size(); ++i) {
        const DVec3 &p = poly[i], &q = poly[(i + 1) % poly.size()];
        const double dp = ddot(p, n) - c0, dq = ddot(q, n) - c0;
        if (dp <= 0) res.push_back(p);
        if ((dp < 0 && dq > 0) || (dp > 0 && dq < 0)) res.push_back(p + (q - p) * (dp / (dp - dq)));
    }
    for (std::size_t i = 1; i + 1 < res.size(); ++i) out.push_back({res[0], res[i], res[i + 1]});
}

void drawView(const Drawing& d, const DrawingView& v, const Model& m, SheetGraphics& g, int quality, std::map<std::string, double>* extentOut = nullptr) {
    ViewSetup vs;
    if (!setupView(d, v, vs)) {
        g.warnings.push_back("View " + v.name + ": its parent view " + v.parent + " can't be cut");
        return;
    }
    const ViewFrame& F = vs.f;
    const Projector P = projectorFor(d, v, F, m.centre());
    const double scale = P.scale;
    // In view coordinates: x right, y up, z into the scene.
    auto V = [&](const DVec3& p) { return DVec3{ddot(p, F.r), ddot(p, F.u), ddot(p, F.d)}; };
    double x0 = 1e300, y0 = 1e300, x1 = -1e300, y1 = -1e300;
    for (const DVec3& q : m.p) {
        const DVec3 w = V(q);
        x0 = std::min(x0, w.x), x1 = std::max(x1, w.x), y0 = std::min(y0, w.y), y1 = std::max(y1, w.y);
    }
    if (!(x0 <= x1)) return;
    // A pixel of about 0.08 sheet mm (finer for exports), within reason: on screen a large
    // view gets coarser cells so the sheet redraws quickly.
    const double extent = std::max(x1 - x0, y1 - y0);
    double px = 0.08 / quality / scale;
    px = std::max(px, extent / (quality == 1 ? 1200.0 : 1500.0 * quality));
    px = std::max(px, extent / 6000.0);
    DepthBuffer zb(x0 - 2 * px, y0 - 2 * px, x1 + 2 * px, y1 + 2 * px, px);
    const double eps = 1.5 * px + 1e-4 * m.diag();
    // Triangles (cut back for a section).
    std::vector<std::array<DVec3, 3>> kept;
    for (std::size_t t = 0; t < m.tris.size(); ++t) {
        const DVec3 &a = m.p[static_cast<std::size_t>(m.tris[t][0])], &b = m.p[static_cast<std::size_t>(m.tris[t][1])], &c = m.p[static_cast<std::size_t>(m.tris[t][2])];
        if (vs.section) {
            kept.clear();
            clipTriangle(a, b, c, vs.keepN, vs.keepC, kept);
            for (const auto& k : kept) zb.triangle(V(k[0]), V(k[1]), V(k[2]));
        } else {
            zb.triangle(V(a), V(b), V(c));
        }
    }
    // The cut face: its outline, hatching, and a lid in the depth buffer.
    if (vs.section) {
        std::vector<Vec3> pp;
        pp.reserve(m.p.size());
        const DVec3 n = vs.keepN;
        for (const DVec3& q : m.p) pp.push_back(Vec3{static_cast<float>(ddot(q, F.r)), static_cast<float>(ddot(q, F.u)), static_cast<float>(ddot(q, n) - vs.keepC)});
        std::vector<section::Tri> tris;
        std::vector<std::uint32_t> cand;
        for (std::size_t t = 0; t < m.tris.size(); ++t) {
            tris.push_back({static_cast<std::uint32_t>(m.tris[t][0]), static_cast<std::uint32_t>(m.tris[t][1]), static_cast<std::uint32_t>(m.tris[t][2])});
            cand.push_back(static_cast<std::uint32_t>(t));
        }
        // Just inside the kept side when the cut runs through vertices (a hole's axis, a seam),
        // where the contour would otherwise have gaps.
        double level = 0;
        for (const Vec3& q : pp)
            if (std::fabs(q.z) < 1e-5 * m.diag()) {
                level = -2e-5 * m.diag();
                break;
            }
        const std::vector<section::Loop> loops = section::contours(pp, tris, cand, level);
        std::vector<std::vector<std::pair<double, double>>> lid;
        for (const section::Loop& l : loops) {
            std::vector<std::pair<double, double>> q;
            for (const section::P2& p2 : l) q.push_back({p2.x, p2.y});
            lid.push_back(q);
            for (std::size_t k = 0; k < l.size(); ++k) {
                const section::P2 &a = l[k], &b = l[(k + 1) % l.size()];
                g.lines.push_back({P.sx + scale * (a.x - P.cx), P.sy + scale * (a.y - P.cy), P.sx + scale * (b.x - P.cx), P.sy + scale * (b.y - P.cy), SheetLine::Visible});
            }
        }
        zb.region(lid, -vs.keepC);
        for (const auto& [a, b] : section::hatch(loops, 3.0 / scale, kPiD2 / 4))
            g.lines.push_back({P.sx + scale * (a.x - P.cx), P.sy + scale * (a.y - P.cy), P.sx + scale * (b.x - P.cx), P.sy + scale * (b.y - P.cy), SheetLine::Thin});
    }
    // Edges: the model's (faces meeting at a crease, open edges) and silhouettes.
    const double creaseExact = std::cos(10.0 * kPiD2 / 180), creaseMesh = std::cos(30.0 * kPiD2 / 180);
    std::set<std::pair<long long, long long>> snapsSeen;
    auto snap = [&](const DVec3& p) {
        const auto s2 = P(p);
        const std::pair<long long, long long> key{std::llround(s2.first * 20), std::llround(s2.second * 20)};
        if (snapsSeen.insert(key).second) g.snaps.push_back({s2.first, s2.second, v.name, p});
    };
    auto emit = [&](const DVec3& a, const DVec3& b, SheetLine::Style visibleStyle) {
        DVec3 pa = a, pb = b;
        if (vs.section) { // clip to what is kept
            const double da = ddot(pa, vs.keepN) - vs.keepC, db = ddot(pb, vs.keepN) - vs.keepC;
            if (da > 0 && db > 0) return;
            if (da > 0) pa = pa + (pb - pa) * (da / (da - db));
            else if (db > 0) pb = pa + (pb - pa) * (da / (da - db));
        }
        const DVec3 va = V(pa), vb = V(pb);
        const double len = std::hypot(vb.x - va.x, vb.y - va.y);
        const int n = std::max(1, static_cast<int>(std::ceil(len / (2 * px))));
        int state = -1;
        double start = 0;
        auto flush = [&](double tEnd) {
            if (state < 0 || (state == 1 && !v.hidden)) return;
            const DVec3 s0 = pa + (pb - pa) * start, s1 = pa + (pb - pa) * tEnd;
            const auto q0 = P(s0), q1 = P(s1);
            if (std::hypot(q1.first - q0.first, q1.second - q0.second) < 1e-6) return;
            if (state == 1 && visibleStyle == SheetLine::Thin) return; // hidden tangent edges: left out
            g.lines.push_back({q0.first, q0.second, q1.first, q1.second, state == 1 ? SheetLine::Hidden : visibleStyle});
        };
        for (int i = 0; i < n; ++i) {
            const double tm = (i + 0.5) / n;
            const DVec3 w = va + (vb - va) * tm;
            const int st = zb.hidden(w.x, w.y, w.z, eps) ? 1 : 0;
            if (st != state) {
                flush(static_cast<double>(i) / n);
                state = st;
                start = static_cast<double>(i) / n;
            }
        }
        flush(1.0);
        if (!zb.hidden(va.x, va.y, va.z, eps)) snap(pa);
        if (!zb.hidden(vb.x, vb.y, vb.z, eps)) snap(pb);
    };
    const bool pictorial = v.kind == ViewKind::Iso;
    for (const Model::Edge& e : m.edges) {
        const DVec3 &a = m.p[static_cast<std::size_t>(e.a)], &b = m.p[static_cast<std::size_t>(e.b)];
        bool draw = false, tangent = false;
        if (e.t1 < 0) draw = true;
        else {
            const DVec3 &n0 = m.normal[static_cast<std::size_t>(e.t0)], &n1 = m.normal[static_cast<std::size_t>(e.t1)];
            const int f0 = m.face[static_cast<std::size_t>(e.t0)], f1 = m.face[static_cast<std::size_t>(e.t1)];
            const double dd = ddot(n0, n1);
            if (f0 >= 0 && f1 >= 0 && f0 != f1) { // a face boundary; a tangent one only in pictorial views, thin
                draw = dd < creaseExact;
                tangent = !draw && pictorial;
            } else if (f0 < 0 || f1 < 0) draw = dd < creaseMesh; // a mesh's crease
            if (!draw) { // a silhouette: one side faces the viewer, the other away
                const double s0 = ddot(n0, F.d), s1 = ddot(n1, F.d);
                draw = (s0 < 0) != (s1 < 0) && std::fabs(s0 - s1) > 1e-9;
                if (draw) tangent = false;
            }
        }
        if (draw) emit(a, b, SheetLine::Visible);
        else if (tangent) emit(a, b, SheetLine::Thin);
    }
    // Centre marks (a hole or boss seen end on) and centre lines (seen from the side).
    for (const Model::Round& rd : m.rounds) {
        const double along = std::fabs(ddot(rd.axis, F.d));
        const DVec3 c = rd.o + rd.axis * (0.5 * (rd.h0 + rd.h1));
        if (vs.section && ddot(c, vs.keepN) > vs.keepC + 1e-9) continue; // cut away
        const auto cc = P(c);
        if (along > 0.999) {
            const double arm = rd.r * scale + 2.0;
            g.lines.push_back({cc.first - arm, cc.second, cc.first + arm, cc.second, SheetLine::Center});
            g.lines.push_back({cc.first, cc.second - arm, cc.first, cc.second + arm, SheetLine::Center});
            g.snaps.push_back({cc.first, cc.second, v.name, c});
        } else if (along < 0.001) {
            const auto a = P(rd.o + rd.axis * rd.h0), b = P(rd.o + rd.axis * rd.h1);
            const double dx = b.first - a.first, dy = b.second - a.second, l = std::hypot(dx, dy);
            if (l < 1e-6) continue;
            const double ox = dx / l * 2.0, oy = dy / l * 2.0;
            g.lines.push_back({a.first - ox, a.second - oy, b.first + ox, b.second + oy, SheetLine::Center});
        }
    }
    if (extentOut) {
        (*extentOut)["x0"] = P.sx + scale * (x0 - P.cx);
        (*extentOut)["x1"] = P.sx + scale * (x1 - P.cx);
        (*extentOut)["y0"] = P.sy + scale * (y0 - P.cy);
        (*extentOut)["y1"] = P.sy + scale * (y1 - P.cy);
    }
}

// --- annotation -----------------------------------------------------------------------------

// Rough text widths (Helvetica-like), for laying text out against lines and boxes. A text
// `h` high is set in a font h / 0.72 big (h is the capitals' height), as the exports do.
double textWidth(const std::string& s, double h) {
    double w = 0;
    for (std::size_t i = 0; i < s.size(); ++i) {
        const unsigned char c = static_cast<unsigned char>(s[i]);
        if ((c & 0xC0) == 0x80) continue; // UTF-8 continuation
        w += c == ' ' || c == '.' || c == ',' || c == '(' || c == ')' || c == '/' || c == '|' ? 0.29 : c >= 0x80 ? 0.78 : (c == 'M' || c == 'W') ? 0.83 : 0.56;
    }
    return w * h / 0.72;
}

// Where a dimension's text went: its centre on the baseline, angle and width.
struct TextPlace {
    double x = 0, y = 0, angle = 0, width = 0;
    bool placed = false;
};

// A dimension's text and tolerance along a baseline: the value, then a stacked pair
// (deviations or limits) in smaller text after it, or the value boxed (basic). (x, y) is
// the start (align 0) or the middle (align 1) of the whole on the baseline.
TextPlace dimText(SheetGraphics& g, double x, double y, double angle, int align, double th, const std::string& main, const std::string& up,
                  const std::string& lo, bool box) {
    const double ar = angle * kPiD2 / 180, ux = std::cos(ar), uy = std::sin(ar), nx = -uy, ny = ux;
    const double sh = th * 0.7, wm = textWidth(main, th), ws = std::max(textWidth(up, sh), textWidth(lo, sh));
    const bool stack = !up.empty() || !lo.empty();
    const double total = wm + (stack ? 0.8 + ws : 0);
    const double s0 = align == 1 ? -total / 2 : 0; // along the baseline from (x, y)
    auto at = [&](double along, double across) { return std::pair<double, double>{x + ux * along + nx * across, y + uy * along + ny * across}; };
    const auto m0 = at(s0, 0);
    if (!main.empty()) g.texts.push_back({m0.first, m0.second, th, angle, 0, main});
    if (stack) {
        const auto u0 = at(s0 + wm + 0.8, th * 0.45), l0 = at(s0 + wm + 0.8, -th * 0.25);
        if (!up.empty()) g.texts.push_back({u0.first, u0.second, sh, angle, 0, up});
        if (!lo.empty()) g.texts.push_back({l0.first, l0.second, sh, angle, 0, lo});
    }
    if (box) { // a basic (theoretically exact) dimension
        const auto a = at(s0 - 0.8, -0.8), b = at(s0 + wm + 0.8, -0.8), c = at(s0 + wm + 0.8, th + 0.8), e = at(s0 - 0.8, th + 0.8);
        g.lines.push_back({a.first, a.second, b.first, b.second, SheetLine::Thin});
        g.lines.push_back({b.first, b.second, c.first, c.second, SheetLine::Thin});
        g.lines.push_back({c.first, c.second, e.first, e.second, SheetLine::Thin});
        g.lines.push_back({e.first, e.second, a.first, a.second, SheetLine::Thin});
    }
    const auto mid = at(s0 + total / 2, 0);
    return {mid.first, mid.second, angle, total, true};
}

void drawDimension(const Drawing& d, const DrawingDim& dim, const Model& m, SheetGraphics& g, TextPlace* where = nullptr) {
    const DrawingView* v = d.view(dim.view);
    if (!v) return;
    ViewSetup vs;
    if (!setupView(d, *v, vs)) return;
    const Projector P = projectorFor(d, *v, vs.f, m.centre());
    const auto A = P(dim.a), B = P(dim.b);
    std::string up, lo;
    const std::string text = dimensionText(d, dim, &up, &lo);
    const bool basic = dim.tol.kind == DimTolerance::Basic;
    const double th = 3.5, gap = 1.0, over = 2.0;
    TextPlace placed;
    auto line = [&](double ax, double ay, double bx, double by, SheetLine::Style s = SheetLine::Thin) { g.lines.push_back({ax, ay, bx, by, s}); };
    if (dim.kind == DrawingDim::Diameter || dim.kind == DrawingDim::Radius) {
        // A leader from the circle at 45 degrees, out to the text.
        const double r = std::hypot(B.first - A.first, B.second - A.second);
        const double c45 = std::sqrt(0.5);
        const double ex = A.first + r * c45, ey = A.second + r * c45;
        const double off = std::max(dim.offset, 4.0);
        const double kx = ex + off * c45, ky = ey + off * c45;
        line(ex, ey, kx, ky);
        placed = dimText(g, kx + 1, ky + 0.8, 0, 0, th, text, up, lo, basic);
        line(kx, ky, kx + placed.width + 2, ky);
        addArrow(g, ex, ey, -c45, -c45);
        if (dim.kind == DrawingDim::Radius) line(A.first, A.second, ex, ey);
        if (where) *where = placed;
        return;
    }
    double ux = 1, uy = 0; // along the dimension line
    if (dim.kind == DrawingDim::Vertical) ux = 0, uy = 1;
    if (dim.kind == DrawingDim::Aligned) {
        const double l = std::hypot(B.first - A.first, B.second - A.second);
        if (l < 1e-9) return;
        ux = (B.first - A.first) / l, uy = (B.second - A.second) / l;
    }
    // The side the dimension line stands off to: up for horizontal, right for vertical
    // (negative offsets go below and to the left), left of a to b for aligned.
    double nx = -uy, ny = ux;
    if (dim.kind == DrawingDim::Vertical) nx = 1, ny = 0;
    // The dimension line through the farther point, `offset` beyond it.
    const double sa = A.first * nx + A.second * ny, sb = B.first * nx + B.second * ny;
    const double base = dim.offset >= 0 ? std::max(sa, sb) + dim.offset : std::min(sa, sb) + dim.offset;
    auto onLine = [&](double px, double py) { // foot of p on the dimension line
        const double s = px * nx + py * ny;
        return std::pair<double, double>{px + nx * (base - s), py + ny * (base - s)};
    };
    const auto a2 = onLine(A.first, A.second), b2 = onLine(B.first, B.second);
    const double dir = dim.offset >= 0 ? 1.0 : -1.0;
    auto extension = [&](std::pair<double, double> from, std::pair<double, double> to) {
        const double ex = to.first - from.first, ey = to.second - from.second, l = std::hypot(ex, ey);
        if (l < gap) return;
        line(from.first + ex / l * gap, from.second + ey / l * gap, to.first + nx * over * dir, to.second + ny * over * dir);
    };
    extension(A, a2);
    extension(B, b2);
    line(a2.first, a2.second, b2.first, b2.second);
    const double len = std::hypot(b2.first - a2.first, b2.second - a2.second);
    if (len > 1e-9) {
        const double dx = (b2.first - a2.first) / len, dy = (b2.second - a2.second) / len;
        if (len > 8) { // arrows inside, else outside
            addArrow(g, a2.first, a2.second, -dx, -dy);
            addArrow(g, b2.first, b2.second, dx, dy);
        } else {
            addArrow(g, a2.first, a2.second, dx, dy);
            addArrow(g, b2.first, b2.second, -dx, -dy);
            line(a2.first - dx * 5, a2.second - dy * 5, a2.first, a2.second);
            line(b2.first, b2.second, b2.first + dx * 5, b2.second + dy * 5);
        }
    }
    // The text above the line, reading from the bottom or the right.
    double angle = std::atan2(uy, ux) * 180 / kPiD2;
    if (angle > 90.01) angle -= 180;
    if (angle < -89.99) angle += 180;
    const double tx = 0.5 * (a2.first + b2.first), ty = 0.5 * (a2.second + b2.second);
    const double ar = angle * kPiD2 / 180;
    placed = dimText(g, tx - std::sin(ar) * 1.0, ty + std::cos(ar) * 1.0, angle, 1, th, text, up, lo, basic);
    if (where) *where = placed;
}

// --- geometric tolerancing -------------------------------------------------------------------

void polyline(SheetGraphics& g, const std::vector<std::pair<double, double>>& pts, bool closed = false, SheetLine::Style st = SheetLine::Thin) {
    for (std::size_t i = 0; i + 1 < pts.size() + (closed ? 1 : 0); ++i) {
        const auto& a = pts[i];
        const auto& b = pts[(i + 1) % pts.size()];
        g.lines.push_back({a.first, a.second, b.first, b.second, st});
    }
}
void circle(SheetGraphics& g, double cx, double cy, double r, double a0 = 0, double a1 = 2 * kPiD2, SheetLine::Style st = SheetLine::Thin) {
    const int n = std::max(6, static_cast<int>(24 * (a1 - a0) / (2 * kPiD2)));
    std::vector<std::pair<double, double>> pts;
    for (int k = 0; k <= n; ++k) {
        const double t = a0 + (a1 - a0) * k / n;
        pts.push_back({cx + r * std::cos(t), cy + r * std::sin(t)});
    }
    polyline(g, pts, false, st);
}

// The characteristic's symbol in a box `h` high centred on (cx, cy).
void gdtGlyph(SheetGraphics& g, GdtSymbol s, double cx, double cy, double h) {
    const double r = h * 0.3; // half the glyph's size
    auto L = [&](double x0, double y0, double x1, double y1) { g.lines.push_back({cx + x0 * r, cy + y0 * r, cx + x1 * r, cy + y1 * r, SheetLine::Thin}); };
    auto arrowHead = [&](double tx, double ty, double dx, double dy) { addArrow(g, cx + tx * r, cy + ty * r, dx, dy, 0.45 * r * 2, 0.22 * r * 2); };
    switch (s) {
    case GdtSymbol::Straightness: L(-1, 0, 1, 0); break;
    case GdtSymbol::Flatness: polyline(g, {{cx - r, cy - 0.5 * r}, {cx + 0.5 * r, cy - 0.5 * r}, {cx + r, cy + 0.5 * r}, {cx - 0.5 * r, cy + 0.5 * r}}, true); break;
    case GdtSymbol::Circularity: circle(g, cx, cy, r * 0.85); break;
    case GdtSymbol::Cylindricity:
        circle(g, cx, cy, r * 0.6);
        L(-0.95, -1, -0.15, 1);
        L(0.15, -1, 0.95, 1);
        break;
    case GdtSymbol::LineProfile: circle(g, cx, cy - 0.5 * r, r * 0.95, 0, kPiD2); break;
    case GdtSymbol::SurfaceProfile:
        circle(g, cx, cy - 0.5 * r, r * 0.95, 0, kPiD2);
        L(-0.95, -0.5, 0.95, -0.5);
        break;
    case GdtSymbol::Angularity:
        L(-1, -0.8, 1, -0.8);
        L(-1, -0.8, 0.9, 0.9);
        break;
    case GdtSymbol::Perpendicularity:
        L(-1, -0.9, 1, -0.9);
        L(0, -0.9, 0, 1);
        break;
    case GdtSymbol::Parallelism:
        L(-0.9, -1, -0.1, 1);
        L(0.1, -1, 0.9, 1);
        break;
    case GdtSymbol::Position:
        circle(g, cx, cy, r * 0.65);
        L(-1, 0, 1, 0);
        L(0, -1, 0, 1);
        break;
    case GdtSymbol::Concentricity:
        circle(g, cx, cy, r * 0.45);
        circle(g, cx, cy, r * 0.95);
        break;
    case GdtSymbol::Symmetry:
        L(-1, 0, 1, 0);
        L(-0.6, 0.55, 0.6, 0.55);
        L(-0.6, -0.55, 0.6, -0.55);
        break;
    case GdtSymbol::CircularRunout:
        L(-0.6, -1, 0.5, 0.9);
        arrowHead(0.5, 0.9, 1.1, 1.9);
        break;
    case GdtSymbol::TotalRunout:
        L(-0.9, -0.9, 0.9, -0.9);
        L(-0.9, -0.9, -0.2, 0.9);
        L(0.2, -0.9, 0.9, 0.9);
        arrowHead(-0.2, 0.9, 0.7, 1.8);
        arrowHead(0.9, 0.9, 0.7, 1.8);
        break;
    }
}

// A material condition modifier: the letter in a circle.
void circledLetter(SheetGraphics& g, double cx, double cy, double h, char letter) {
    circle(g, cx, cy, h * 0.42);
    g.texts.push_back({cx, cy - h * 0.25, h * 0.55, 0, 1, std::string(1, letter)});
}

// A feature control frame with its top-left corner at (x, y); returns its width.
double drawFrameBoxes(SheetGraphics& g, const DrawingFrame& f, double x, double y, double h) {
    const double th = h * 0.5;
    std::vector<double> cells{h}; // the symbol
    std::string value = num(f.tolerance);
    if (f.tolerance < 0.01 && f.tolerance > 0) { // three or four decimals
        char b[32];
        std::snprintf(b, sizeof b, "%.4f", f.tolerance);
        value = b;
        while (value.back() == '0') value.pop_back();
    }
    const std::string zone = (f.diameter ? "\xC3\x98" : "") + value;
    const double tw = textWidth(zone, th) + (f.material ? h * 1.2 : 0) + 3;
    cells.push_back(std::max(tw, h * 1.6));
    std::vector<std::string> datum;
    for (const std::string& dt : f.datums) {
        datum.push_back(dt);
        cells.push_back(std::max(h, textWidth(dt, th) + 3));
    }
    double w = 0;
    for (double c : cells) w += c;
    const double y0 = y - h;
    polyline(g, {{x, y0}, {x + w, y0}, {x + w, y}, {x, y}}, true);
    double cx = x;
    for (std::size_t i = 0; i < cells.size(); ++i) {
        if (i > 0) g.lines.push_back({cx, y0, cx, y, SheetLine::Thin});
        if (i == 0) gdtGlyph(g, f.symbol, cx + h / 2, y0 + h / 2, h);
        else if (i == 1) {
            g.texts.push_back({cx + 1.5, y0 + (h - th) / 2, th, 0, 0, zone});
            if (f.material) circledLetter(g, cx + 1.5 + textWidth(zone, th) + h * 0.65, y0 + h / 2, h * 0.9, f.material);
        } else {
            const std::string& dt = datum[i - 2];
            g.texts.push_back({cx + cells[i] / 2, y0 + (h - th) / 2, th, 0, 1, dt});
        }
        cx += cells[i];
    }
    return w;
}

// A leader from the box at (x, y) to the point (px, py), the arrow on the point.
void leader(SheetGraphics& g, double x, double y, double px, double py) {
    g.lines.push_back({x, y, px, py, SheetLine::Thin});
    addArrow(g, px, py, px - x, py - y);
}

bool viewPoint(const Drawing& d, const Model& m, const std::string& view, const DVec3& at, double& px, double& py) {
    const DrawingView* v = d.view(view);
    if (!v) return false;
    ViewSetup vs;
    if (!setupView(d, *v, vs)) return false;
    const auto p = projectorFor(d, *v, vs.f, m.centre())(at);
    px = p.first, py = p.second;
    return true;
}

void drawFrame(const Drawing& d, const DrawingFrame& f, const Model& m, const std::vector<TextPlace>& dimPlaces, SheetGraphics& g) {
    const double h = 7.0;
    if (f.dim >= 0) { // under the dimension's value
        if (f.dim >= static_cast<int>(dimPlaces.size()) || !dimPlaces[static_cast<std::size_t>(f.dim)].placed) return;
        const TextPlace& t = dimPlaces[static_cast<std::size_t>(f.dim)];
        // Below the text (to its right when the text runs up the sheet).
        DrawingFrame probe = f;
        SheetGraphics scratch;
        const double w = drawFrameBoxes(scratch, probe, 0, 0, h);
        // Under the value: below horizontal text, below the lower end of text running up
        // the sheet (so it stays outside the part, along the dimension line).
        double cx = t.x, cy = t.y - 2.0 - h / 2;
        if (std::fabs(std::sin(t.angle * kPiD2 / 180)) > 0.7) cy = t.y - t.width / 2 - 2.0 - h / 2, cx = t.x - 1.0 - w / 2; // clear of the line
        // A diameter or radius: from the start of its text, clear of the leader's elbow.
        const DrawingDim& dm = d.dims[static_cast<std::size_t>(f.dim)];
        if (dm.kind == DrawingDim::Diameter || dm.kind == DrawingDim::Radius) cx = t.x - t.width / 2 + w / 2;
        drawFrameBoxes(g, f, cx - w / 2, cy + h / 2, h);
        return;
    }
    double px, py;
    if (!viewPoint(d, m, f.view, f.at, px, py)) return;
    const double x = px + f.dx, y = py + f.dy + h / 2; // the frame's left end, middle, at (px + dx, py + dy)
    const bool left = f.dx < 0;
    SheetGraphics scratch;
    const double w = drawFrameBoxes(scratch, f, 0, 0, h);
    const double fx = left ? x - w : x;
    drawFrameBoxes(g, f, fx, y, h);
    const double ex = left ? x : fx, ey = y - h / 2;
    // The leader leaves the frame's near end horizontally, then runs to the point.
    const double kx = ex + (left ? 3 : -3);
    g.lines.push_back({ex, ey, kx, ey, SheetLine::Thin});
    leader(g, kx, ey, px, py);
}

void drawDatum(const Drawing& d, const DrawingDatum& dt, const Model& m, SheetGraphics& g) {
    double px, py;
    if (!viewPoint(d, m, dt.view, dt.at, px, py)) return;
    const double h = 7.0, bx = px + dt.dx, by = py + dt.dy; // the box's centre
    polyline(g, {{bx - h / 2, by - h / 2}, {bx + h / 2, by - h / 2}, {bx + h / 2, by + h / 2}, {bx - h / 2, by + h / 2}}, true);
    g.texts.push_back({bx, by - h * 0.25, h * 0.5, 0, 1, dt.letter});
    // From the box's nearest side to the feature, ending in a filled triangle on it.
    double dx = px - bx, dy = py - by;
    const double l = std::hypot(dx, dy);
    if (l < h * 0.6) return;
    dx /= l, dy /= l;
    const double sx = bx + dx * h / 2 / std::max(std::fabs(dx), std::fabs(dy)), sy = by + dy * h / 2 / std::max(std::fabs(dx), std::fabs(dy));
    const double tri = 3.0, half = 1.7;
    const double qx = px - dx * tri, qy = py - dy * tri; // the triangle's apex, its base on the feature
    g.lines.push_back({sx, sy, qx, qy, SheetLine::Thin});
    g.fills.push_back({{qx, qy}, {px - dy * half, py + dx * half}, {px + dy * half, py - dx * half}});
}

void drawFinish(const Drawing& d, const DrawingFinish& f, const Model& m, SheetGraphics& g) {
    double px, py;
    if (!viewPoint(d, m, f.view, f.at, px, py)) return;
    const double x = px + f.dx, y = py + f.dy, s = 3.5;
    // The check mark: a short leg up-left and a long one up-right from the foot.
    const double c60 = 0.5, s60 = std::sqrt(3.0) / 2;
    const double lx = x - s * c60 * 1.2, ly = y + s * s60 * 1.2;   // short leg
    const double rx = x + s * c60 * 2.6, ry = y + s * s60 * 2.6;   // long leg
    g.lines.push_back({x, y, lx, ly, SheetLine::Thin});
    g.lines.push_back({x, y, rx, ry, SheetLine::Thin});
    if (f.process == 1) g.lines.push_back({lx, ly, 2 * x - lx, ly, SheetLine::Thin}); // the bar closing a triangle: removal required
    if (f.process == 2) circle(g, x, y + s * 0.62, s * 0.42);                                               // the circle: no removal
    // The Ra value over a line running on from the long leg.
    const std::string text = "Ra " + num(f.ra);
    const double w = textWidth(text, 2.5) + 1.5;
    g.lines.push_back({rx, ry, rx + w, ry, SheetLine::Thin});
    g.texts.push_back({rx + 0.8, ry + 0.8, 2.5, 0, 0, text});
}

void drawSectionLine(const Drawing& d, const DrawingView& sec, const Model& m, SheetGraphics& g) {
    const DrawingView* p = d.view(sec.parent);
    if (!p) return;
    ViewSetup ps, ss;
    if (!setupView(d, *p, ps) || !setupView(d, sec, ss)) return;
    const Projector P = projectorFor(d, *p, ps.f, m.centre());
    // The cut line across the parent view's extent, a little beyond.
    double lo = 1e300, hi = -1e300;
    const DVec3 along = sec.vertical ? ps.f.u : ps.f.r;
    for (const DVec3& q : m.p) lo = std::min(lo, ddot(q, along)), hi = std::max(hi, ddot(q, along));
    const DVec3 across = sec.vertical ? ps.f.r : ps.f.u;
    const DVec3 a = across * sec.cut + along * (lo - 6 / P.scale), b = across * sec.cut + along * (hi + 6 / P.scale);
    // Points on the parent's other axis are whatever the centre gives: the projector only uses r and u.
    const auto A = P(a), B = P(b);
    g.lines.push_back({A.first, A.second, B.first, B.second, SheetLine::Cut});
    // Arrows at both ends, pointing the way the section looks.
    const double vx = ddot(ss.f.d, ps.f.r), vy = ddot(ss.f.d, ps.f.u);
    const std::string label = sec.name.size() > 8 && sec.name.rfind("Section ", 0) == 0 ? sec.name.substr(8, 1) : sec.name.substr(0, 1);
    for (const auto& e : {A, B}) {
        const double bx = e.first - vx * 8, by = e.second - vy * 8;
        g.lines.push_back({bx, by, e.first, e.second, SheetLine::Visible});
        addArrow(g, e.first, e.second, vx, vy, 3.5, 1.0);
        g.texts.push_back({bx - vx * 2 - 1.5, by - vy * 2 - 2, 5.0, 0, 0, label});
    }
}

void drawSheetFrame(const Drawing& d, SheetGraphics& g) {
    const double W = g.width, H = g.height, b = 10;
    auto L = [&](double x0, double y0, double x1, double y1, SheetLine::Style s = SheetLine::Thin) { g.lines.push_back({x0, y0, x1, y1, s}); };
    L(b, b, W - b, b, SheetLine::Border);
    L(W - b, b, W - b, H - b, SheetLine::Border);
    L(W - b, H - b, b, H - b, SheetLine::Border);
    L(b, H - b, b, b, SheetLine::Border);
    // Title block, bottom right: 180 x 40.
    const double tw = std::min(180.0, W - 2 * b), th = 40, x0 = W - b - tw, y0 = b;
    L(x0, y0, x0, y0 + th, SheetLine::Border);
    L(x0, y0 + th, W - b, y0 + th, SheetLine::Border);
    const double r1 = y0 + 10, r2 = y0 + 20, r3 = y0 + 30;
    for (double y : {r1, r2, r3}) L(x0, y, W - b, y);
    auto field = [&](const char* key, const std::string& fallback = "") {
        const auto it = d.title.find(key);
        return it != d.title.end() && !it->second.empty() ? it->second : fallback;
    };
    auto cell = [&](double cx0, double cy0, double cw, const std::string& label, const std::string& value, double vh = 3.5) {
        g.texts.push_back({cx0 + 1.5, cy0 + 6.8, 1.8, 0, 0, label});
        g.texts.push_back({cx0 + 1.5, cy0 + 1.8, vh, 0, 0, value});
        (void)cw;
    };
    // Row 4 (top): the title across.
    cell(x0, r3, tw, "TITLE", field("TITLE", "Part"), 4.5);
    // Row 3: part number, material.
    const double c1 = x0 + tw * 0.5;
    L(c1, r2, c1, r3);
    cell(x0, r2, tw * 0.5, "PART NO.", field("PART"));
    cell(c1, r2, tw * 0.5, "MATERIAL", field("MATERIAL"));
    // Row 2: drawn, checked, date.
    const double c2 = x0 + tw / 3, c3 = x0 + 2 * tw / 3;
    L(c2, r1, c2, r2);
    L(c3, r1, c3, r2);
    std::string date = field("DATE");
    if (date.empty()) {
        char buf[16];
        const std::time_t t = std::time(nullptr);
        std::tm tm{};
#if defined(_WIN32)
        localtime_s(&tm, &t);
#else
        localtime_r(&t, &tm);
#endif
        std::strftime(buf, sizeof buf, "%Y-%m-%d", &tm);
        date = buf;
    }
    cell(x0, r1, tw / 3, "DRAWN", field("DRAWN"));
    cell(c2, r1, tw / 3, "CHECKED", field("CHECKED"));
    cell(c3, r1, tw / 3, "DATE", date);
    // Row 1: scale, units, projection, sheet, revision, company.
    const double w6 = tw / 6;
    for (int k = 1; k < 6; ++k) L(x0 + w6 * k, y0, x0 + w6 * k, r1);
    cell(x0, y0, w6, "SCALE", scaleText(d.scale));
    cell(x0 + w6, y0, w6, "UNITS", "mm");
    g.texts.push_back({x0 + 2 * w6 + 1.5, y0 + 6.8, 1.8, 0, 0, d.thirdAngle ? "3RD ANGLE" : "1ST ANGLE"});
    // The projection symbol: a truncated cone's two views.
    {
        const double sx = x0 + 2 * w6 + 4, sy = y0 + 1.2, hgt = 4.6;
        const double circX = d.thirdAngle ? sx + 15 : sx + 3, coneX = d.thirdAngle ? sx + 4 : sx + 9.5;
        // Circles (polygons) for the end view.
        for (double rr : {hgt / 2, hgt / 4}) {
            const double cy = sy + hgt / 2;
            for (int k = 0; k < 24; ++k) {
                const double a0 = k * kPiD2 / 12, a1 = (k + 1) * kPiD2 / 12;
                L(circX + rr * std::cos(a0), cy + rr * std::sin(a0), circX + rr * std::cos(a1), cy + rr * std::sin(a1), SheetLine::Visible);
            }
        }
        // The side view: a trapezoid, wide at the circle's side.
        const bool wideLeft = !d.thirdAngle;
        const double xl = coneX, xr = coneX + 7, big = hgt / 2, small = hgt / 4, cy = sy + hgt / 2;
        const double ll = wideLeft ? small : big, rr2 = wideLeft ? big : small;
        L(xl, cy - ll, xr, cy - rr2, SheetLine::Visible);
        L(xr, cy - rr2, xr, cy + rr2, SheetLine::Visible);
        L(xr, cy + rr2, xl, cy + ll, SheetLine::Visible);
        L(xl, cy + ll, xl, cy - ll, SheetLine::Visible);
    }
    cell(x0 + 3 * w6, y0, w6, "SHEET", field("SHEET", "1 / 1"));
    cell(x0 + 4 * w6, y0, w6, "REV", field("REV", "A"));
    cell(x0 + 5 * w6, y0, w6, "SIZE", upper(d.sheet));
    if (!field("COMPANY").empty()) g.texts.push_back({x0 + tw - 1.5, r3 + 1.8, 3.0, 0, 2, field("COMPANY")});
    if (!d.generalTolerance.empty())
        g.texts.push_back({W - b - 1.5, y0 + th + 2.0, 2.5, 0, 2, "UNLESS OTHERWISE STATED: GENERAL TOLERANCES " + upper(d.generalTolerance)});
}

} // namespace

// --- names, sizes, scales -----------------------------------------------------------------

void viewAxes(ViewKind k, DVec3& right, DVec3& up, DVec3& into) {
    const ViewFrame f = frameOf(k);
    right = f.r, up = f.u, into = f.d;
}

const char* viewKindName(ViewKind k) {
    switch (k) {
    case ViewKind::Front: return "Front";
    case ViewKind::Back: return "Back";
    case ViewKind::Top: return "Top";
    case ViewKind::Bottom: return "Bottom";
    case ViewKind::Left: return "Left";
    case ViewKind::Right: return "Right";
    case ViewKind::Iso: return "Isometric";
    case ViewKind::Section: return "Section";
    }
    return "Front";
}

bool viewKindFromName(const std::string& name, ViewKind* out) {
    const std::string n = upper(name);
    static const std::pair<const char*, ViewKind> kNames[] = {{"FRONT", ViewKind::Front},   {"BACK", ViewKind::Back},  {"REAR", ViewKind::Back},
                                                              {"TOP", ViewKind::Top},       {"PLAN", ViewKind::Top},   {"BOTTOM", ViewKind::Bottom},
                                                              {"LEFT", ViewKind::Left},     {"RIGHT", ViewKind::Right}, {"ISO", ViewKind::Iso},
                                                              {"ISOMETRIC", ViewKind::Iso}, {"SECTION", ViewKind::Section}};
    for (const auto& [s, k] : kNames)
        if (n == s) {
            *out = k;
            return true;
        }
    return false;
}

const DrawingView* Drawing::view(const std::string& name) const {
    for (const DrawingView& v : views)
        if (upper(v.name) == upper(name)) return &v;
    return nullptr;
}

bool sheetSize(const std::string& name, double& w, double& h) {
    const std::string n = upper(name);
    static const std::map<std::string, std::pair<double, double>> kSizes = {
        {"A4", {297, 210}},          {"A3", {420, 297}},        {"A2", {594, 420}},        {"A1", {841, 594}},       {"A0", {1189, 841}},
        {"LETTER", {279.4, 215.9}},  {"ANSI A", {279.4, 215.9}}, {"TABLOID", {431.8, 279.4}}, {"ANSI B", {431.8, 279.4}}, {"ANSI C", {558.8, 431.8}},
        {"ANSI D", {863.6, 558.8}}, {"ANSIA", {279.4, 215.9}}, {"ANSIB", {431.8, 279.4}}, {"ANSIC", {558.8, 431.8}}, {"ANSID", {863.6, 558.8}}};
    const auto it = kSizes.find(n);
    if (it == kSizes.end()) return false;
    w = it->second.first, h = it->second.second;
    return true;
}

bool parseScale(const std::string& s, double& scale) {
    const std::size_t c = s.find_first_of(":/");
    char* end = nullptr;
    if (c == std::string::npos) {
        scale = std::strtod(s.c_str(), &end);
        return end && *end == '\0' && scale > 0;
    }
    const double a = std::strtod(s.substr(0, c).c_str(), nullptr), b = std::strtod(s.substr(c + 1).c_str(), nullptr);
    if (!(a > 0 && b > 0)) return false;
    scale = a / b;
    return true;
}

std::string scaleText(double s) {
    if (!(s > 0)) return "1:1";
    if (s >= 1) return num(s) + ":1";
    return "1:" + num(1 / s);
}

double standardScale(double needed) {
    static const double kScales[] = {50, 20, 10, 5, 2, 1, 0.5, 0.2, 0.1, 0.05, 0.02, 0.01, 0.005, 0.002, 0.001};
    for (double s : kScales)
        if (s <= needed * (1 + 1e-9)) return s;
    return 0.001;
}

// --- layout -------------------------------------------------------------------------------

namespace {

// A view's size on the sheet, model mm (before scale): its projected extent.
std::pair<double, double> viewExtent(const Drawing& d, const Model& m, const DrawingView& v) {
    ViewSetup vs;
    if (!setupView(d, v, vs)) return {0, 0};
    double x0 = 1e300, x1 = -1e300, y0 = 1e300, y1 = -1e300;
    for (const DVec3& q : m.p) {
        if (vs.section && ddot(q, vs.keepN) > vs.keepC) continue;
        x0 = std::min(x0, ddot(q, vs.f.r)), x1 = std::max(x1, ddot(q, vs.f.r));
        y0 = std::min(y0, ddot(q, vs.f.u)), y1 = std::max(y1, ddot(q, vs.f.u));
    }
    return x0 <= x1 ? std::pair<double, double>{x1 - x0, y1 - y0} : std::pair<double, double>{0, 0};
}

Model modelFor(const Drawing& d, const Scene& scene, double chord) {
    return buildModel(drawnBodies(d, scene), chord);
}

double chordFor(const Drawing& d, const std::vector<const Body*>& bodies) {
    AABB box;
    for (const Body* b : bodies) {
        const AABB bb = worldMesh(*b).bounds();
        if (bb.valid()) box.expand(bb.min), box.expand(bb.max);
    }
    const double diag = box.valid() ? static_cast<double>(length(box.max - box.min)) : 100.0;
    return std::clamp(0.05 / std::max(d.scale, 1e-6), std::max(diag * 2e-5, 0.002), std::max(diag / 300, 0.002));
}

} // namespace

void placeView(Drawing& d, const Scene& scene, DrawingView& v) {
    double W = 0, H = 0;
    if (!sheetSize(d.sheet, W, H)) W = 420, H = 297;
    if (!d.landscape) std::swap(W, H);
    const Model m = modelFor(d, scene, 1e9);
    const DrawingView* front = d.view("Front");
    const double s = v.scale > 0 ? v.scale : d.scale, gap = 18;
    const auto ext = viewExtent(d, m, v);
    if (!front) {
        v.x = W * 0.35, v.y = H * 0.55;
        return;
    }
    const auto fe = viewExtent(d, m, *front);
    const double fs = front->scale > 0 ? front->scale : d.scale;
    const double dx = fs * fe.first / 2 + s * ext.first / 2 + gap, dy = fs * fe.second / 2 + s * ext.second / 2 + gap;
    const double sg = d.thirdAngle ? 1.0 : -1.0; // third angle: views where they are seen from
    v.x = front->x, v.y = front->y;
    switch (v.kind) {
    case ViewKind::Top: v.y = front->y + sg * dy; break;
    case ViewKind::Bottom: v.y = front->y - sg * dy; break;
    case ViewKind::Right: v.x = front->x + sg * dx; break;
    case ViewKind::Left: v.x = front->x - sg * dx; break;
    case ViewKind::Back: {
        const DrawingView side{"", d.thirdAngle ? ViewKind::Right : ViewKind::Left, 0, 0, 0, true, "", true, 0, false};
        const auto se = viewExtent(d, m, side);
        v.x = front->x + dx + s * se.first + gap;
        break;
    }
    case ViewKind::Iso:
        v.x = W - 10 - 20 - s * ext.first / 2;
        v.y = H - 10 - 20 - s * ext.second / 2;
        break;
    case ViewKind::Section: {
        // The free place nearest its parent: clear of the other views and the title block.
        const DrawingView* p = d.view(v.parent);
        const DrawingView& base = p ? *p : *front;
        const double hw = s * ext.first / 2 + gap / 2, hh = s * ext.second / 2 + gap / 2 + 4; // + its label below
        // It reads best on the side the viewer of the cut stands, level with the parent.
        double px = 1, py = 0;
        ViewSetup ps, ss;
        if (p && setupView(d, *p, ps) && setupView(d, v, ss)) px = -ddot(ss.f.d, ps.f.r), py = -ddot(ss.f.d, ps.f.u);
        std::vector<std::array<double, 4>> taken; // x0 y0 x1 y1
        for (const DrawingView& o : d.views) {
            if (&o == &v) continue;
            const auto oe = viewExtent(d, m, o);
            const double os = o.scale > 0 ? o.scale : d.scale;
            // Room for dimensions around the orthographic views.
            const double m2 = o.kind == ViewKind::Iso ? gap / 2 : gap;
            taken.push_back({o.x - os * oe.first / 2 - m2, o.y - os * oe.second / 2 - m2, o.x + os * oe.first / 2 + m2, o.y + os * oe.second / 2 + m2});
        }
        taken.push_back({W - 10 - 180, 0, W, 10 + 40 + 6}); // the title block
        auto freeAt = [&](double x, double y) {
            if (x - hw < 10 || x + hw > W - 10 || y - hh < 10 || y + hh > H - 10) return false;
            for (const auto& t : taken)
                if (x - hw < t[2] && x + hw > t[0] && y - hh < t[3] && y + hh > t[1]) return false;
            return true;
        };
        double best = 1e300;
        v.x = base.x, v.y = base.y;
        for (double y = 10 + hh; y <= H - 10 - hh; y += 2.5)
            for (double x = 10 + hw; x <= W - 10 - hw; x += 2.5) {
                const double ox = x - base.x, oy = y - base.y;
                const double along = ox * px + oy * py, off = std::fabs(oy * px - ox * py);
                const double cost = off * 4 + std::fabs(along) + (along < 0 ? 400 : 0);
                if (cost < best && freeAt(x, y)) best = cost, v.x = x, v.y = y;
            }
        if (best >= 1e300) v.x = base.x + s * ext.first + gap; // nowhere free: beside it
        break;
    }
    default: break;
    }
}

Drawing defaultDrawing(const Scene& scene, const std::vector<std::string>& bodies, const std::string& sheet, bool landscape, bool thirdAngle,
                       double scale) {
    Drawing d;
    d.sheet = sheet;
    d.landscape = landscape;
    d.thirdAngle = thirdAngle;
    d.bodies = bodies;
    double W = 0, H = 0;
    if (!sheetSize(sheet, W, H)) W = 420, H = 297, d.sheet = "A3";
    if (!landscape) std::swap(W, H);
    const Model m = modelFor(d, scene, 1e9);
    if (!m.valid()) return d;
    auto extentOf = [&](ViewKind k) { return viewExtent(d, m, DrawingView{"", k, 0, 0, 0, true, "", true, 0, false}); };
    const auto F = extentOf(ViewKind::Front), T = extentOf(ViewKind::Top), R = extentOf(ViewKind::Right), I = extentOf(ViewKind::Iso);
    // Front and right side by side, the top above or below the front, the isometric in
    // the corner above the title block: the largest standard scale that fits.
    const double g = 18, border = 10, block = 50;
    const double fitW = (W - 2 * border - 4 * g) / std::max(R.first + F.first + I.first, 1e-9);
    const double fitH = (H - 2 * border - 3 * g) / std::max(F.second + T.second, 1e-9);
    const double fitI = (H - 2 * border - block - 2 * g) / std::max(I.second, 1e-9);
    d.scale = scale > 0 ? scale : standardScale(std::min({fitW, fitH, fitI}));
    const double s = d.scale;
    // Columns left to right (first angle: right view, front; third: front, right), centred
    // in the room left of the isometric.
    const double room = W - 2 * border - g - s * I.first - g;
    const double used = s * (R.first + F.first) + g;
    const double left = border + g + std::max(0.0, (room - used - g) / 2);
    const double topY = H - border - g;
    DrawingView front{"Front", ViewKind::Front, 0, 0, 0, true, "", true, 0, false};
    DrawingView right{"Right", ViewKind::Right, 0, 0, 0, true, "", true, 0, false};
    DrawingView top{"Top", ViewKind::Top, 0, 0, 0, true, "", true, 0, false};
    DrawingView iso{"Isometric", ViewKind::Iso, 0, 0, 0, false, "", true, 0, false};
    const double rowsH = s * (F.second + T.second) + g;
    const double y0 = std::min(topY, border + g + rowsH + std::max(0.0, (H - 2 * border - 2 * g - rowsH) / 2));
    if (thirdAngle) {
        top.y = y0 - s * T.second / 2;
        front.y = top.y - s * T.second / 2 - g - s * F.second / 2;
        front.x = left + s * F.first / 2;
        right.x = front.x + s * F.first / 2 + g + s * R.first / 2;
    } else {
        front.y = y0 - s * F.second / 2;
        top.y = front.y - s * F.second / 2 - g - s * T.second / 2;
        right.x = left + s * R.first / 2;
        front.x = right.x + s * R.first / 2 + g + s * F.first / 2;
    }
    top.x = front.x;
    right.y = front.y;
    // Up off the title block when the lowest view reaches into it, as far as the sheet allows.
    {
        const double bx0 = W - border - std::min(180.0, W - 2 * border), by1 = border + 40 + g / 2;
        double low = 1e300, high = -1e300;
        bool onBlock = false;
        for (const auto& [v, e] : {std::pair{&front, F}, std::pair{&top, T}, std::pair{&right, R}}) {
            low = std::min(low, v->y - s * e.second / 2);
            high = std::max(high, v->y + s * e.second / 2);
            onBlock = onBlock || (v->x + s * e.first / 2 > bx0 && v->y - s * e.second / 2 < by1);
        }
        if (onBlock) {
            const double up = std::max(0.0, std::min(by1 - low, topY - high));
            front.y += up, top.y += up, right.y += up;
        }
    }
    iso.x = W - border - g - s * I.first / 2;
    iso.y = topY - s * I.second / 2;
    d.views = {front, top, right, iso};
    d.title["TITLE"] = bodies.size() == 1 ? bodies[0] : "";
    return d;
}

int autoDimension(Drawing& d, const Scene& scene, const std::string& onlyView) {
    const Model m = modelFor(d, scene, chordFor(d, drawnBodies(d, scene)));
    if (!m.valid()) return 0;
    int added = 0;
    auto exists = [&](const DrawingDim& n) {
        for (const DrawingDim& o : d.dims)
            if (o.kind == n.kind && std::fabs(dimensionValue(d, o) - dimensionValue(d, n)) < 1e-6 && (o.view == n.view || n.kind == DrawingDim::Diameter)) return true;
        return false;
    };
    auto add = [&](const DrawingDim& n) {
        if (exists(n)) return;
        d.dims.push_back(n);
        ++added;
    };
    bool depthDone = false;
    for (const DrawingView& v : d.views) {
        if (!onlyView.empty() && upper(v.name) != upper(onlyView)) continue;
        if (v.kind == ViewKind::Iso || v.kind == ViewKind::Section) continue;
        const ViewFrame F = frameOf(v.kind);
        // The model points farthest along the view's axes.
        std::size_t xl = 0, xh = 0, yl = 0, yh = 0;
        for (std::size_t i = 0; i < m.p.size(); ++i) {
            const double x = ddot(m.p[i], F.r), y = ddot(m.p[i], F.u);
            if (x < ddot(m.p[xl], F.r)) xl = i;
            if (x > ddot(m.p[xh], F.r)) xh = i;
            if (y < ddot(m.p[yl], F.u)) yl = i;
            if (y > ddot(m.p[yh], F.u)) yh = i;
        }
        const bool front = v.kind == ViewKind::Front || v.kind == ViewKind::Back;
        const bool plan = v.kind == ViewKind::Top || v.kind == ViewKind::Bottom;
        if (front || (!plan && !depthDone)) {
            DrawingDim h;
            h.kind = DrawingDim::Horizontal, h.view = v.name, h.a = m.p[xl], h.b = m.p[xh], h.offset = -12;
            add(h);
            if (!front) depthDone = true;
        }
        if (front || (plan && !depthDone)) {
            DrawingDim vd;
            vd.kind = DrawingDim::Vertical, vd.view = v.name, vd.a = m.p[yl], vd.b = m.p[yh], vd.offset = -12;
            add(vd);
            if (plan) depthDone = true;
        }
    }
    // Round holes and bosses, dimensioned once where they are seen end on.
    for (const Model::Round& rd : m.rounds) {
        for (const DrawingView& v : d.views) {
            if (!onlyView.empty() && upper(v.name) != upper(onlyView)) continue;
            if (v.kind == ViewKind::Iso || v.kind == ViewKind::Section) continue;
            if (std::fabs(ddot(rd.axis, frameOf(v.kind).d)) < 0.999) continue;
            DrawingDim n;
            n.kind = DrawingDim::Diameter, n.view = v.name;
            n.a = rd.o + rd.axis * rd.h1;
            DVec3 x = dcross(rd.axis, {0, 0, 1});
            if (dlength(x) < 1e-6) x = dcross(rd.axis, {1, 0, 0});
            n.b = n.a + dnormalize(x) * rd.r;
            n.offset = 6;
            add(n);
            break;
        }
    }
    return added;
}

double dimensionValue(const Drawing& d, const DrawingDim& dim) {
    const DrawingView* v = d.view(dim.view);
    ViewSetup vs;
    if (!v || !setupView(d, *v, vs)) return dlength(dim.b - dim.a);
    const DVec3 e = dim.b - dim.a;
    switch (dim.kind) {
    case DrawingDim::Horizontal: return std::fabs(ddot(e, vs.f.r));
    case DrawingDim::Vertical: return std::fabs(ddot(e, vs.f.u));
    case DrawingDim::Aligned: return std::hypot(ddot(e, vs.f.r), ddot(e, vs.f.u));
    case DrawingDim::Diameter: return 2 * dlength(e);
    case DrawingDim::Radius: return dlength(e);
    }
    return 0;
}

namespace {

struct GdtInfo {
    GdtSymbol s;
    const char* name;
    int datums;    // 0 none, 1 optional, 2 required
    bool diameter; // a cylindrical zone allowed
};
constexpr GdtInfo kGdt[] = {
    {GdtSymbol::Straightness, "straightness", 0, true},      {GdtSymbol::Flatness, "flatness", 0, false},
    {GdtSymbol::Circularity, "circularity", 0, false},       {GdtSymbol::Cylindricity, "cylindricity", 0, false},
    {GdtSymbol::LineProfile, "profile-line", 1, false},      {GdtSymbol::SurfaceProfile, "profile-surface", 1, false},
    {GdtSymbol::Angularity, "angularity", 2, true},          {GdtSymbol::Perpendicularity, "perpendicularity", 2, true},
    {GdtSymbol::Parallelism, "parallelism", 2, true},        {GdtSymbol::Position, "position", 2, true},
    {GdtSymbol::Concentricity, "concentricity", 2, true},    {GdtSymbol::Symmetry, "symmetry", 2, false},
    {GdtSymbol::CircularRunout, "runout", 2, false},         {GdtSymbol::TotalRunout, "total-runout", 2, false}};

const GdtInfo& gdtInfo(GdtSymbol s) {
    for (const GdtInfo& g : kGdt)
        if (g.s == s) return g;
    return kGdt[0];
}

// A deviation: "+0.021", "-0.007", "0".
std::string deviationText(double v) {
    if (std::fabs(v) < 5e-7) return "0";
    char b[32];
    std::snprintf(b, sizeof b, "%+.3f", v);
    std::string t = b;
    while (t.back() == '0' && t.find('.') != std::string::npos && t.size() > t.find('.') + 2) t.pop_back();
    return t;
}

std::string limitText(double v) {
    char b[32];
    std::snprintf(b, sizeof b, "%.3f", v);
    std::string t = b;
    while (t.back() == '0' && t.size() > t.find('.') + 2) t.pop_back();
    return t;
}

} // namespace

const char* gdtName(GdtSymbol s) { return gdtInfo(s).name; }

bool gdtFromName(const std::string& name, GdtSymbol* out) {
    std::string n;
    for (char c : name) n += c == '_' || c == ' ' ? '-' : static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    for (const GdtInfo& g : kGdt)
        if (n == g.name) return *out = g.s, true;
    static const std::pair<const char*, GdtSymbol> aliases[] = {
        {"flat", GdtSymbol::Flatness},          {"straight", GdtSymbol::Straightness},     {"roundness", GdtSymbol::Circularity},
        {"round", GdtSymbol::Circularity},      {"cylinder", GdtSymbol::Cylindricity},     {"lineprofile", GdtSymbol::LineProfile},
        {"profile", GdtSymbol::SurfaceProfile}, {"surfaceprofile", GdtSymbol::SurfaceProfile}, {"angle", GdtSymbol::Angularity},
        {"perp", GdtSymbol::Perpendicularity},  {"square", GdtSymbol::Perpendicularity},   {"parallel", GdtSymbol::Parallelism},
        {"pos", GdtSymbol::Position},           {"true-position", GdtSymbol::Position},    {"coaxiality", GdtSymbol::Concentricity},
        {"concentric", GdtSymbol::Concentricity}, {"circular-runout", GdtSymbol::CircularRunout}, {"totalrunout", GdtSymbol::TotalRunout}};
    for (const auto& [a, sym] : aliases)
        if (n == a) return *out = sym, true;
    return false;
}

bool gdtTakesDatums(GdtSymbol s) { return gdtInfo(s).datums > 0; }
bool gdtNeedsDatum(GdtSymbol s) { return gdtInfo(s).datums == 2; }
bool gdtTakesDiameter(GdtSymbol s) { return gdtInfo(s).diameter; }

std::string dimensionText(const Drawing& d, const DrawingDim& dim, std::string* upper, std::string* lower) {
    if (upper) upper->clear();
    if (lower) lower->clear();
    const double value = dimensionValue(d, dim);
    const std::string prefix = dim.kind == DrawingDim::Diameter ? "\xC3\x98" : dim.kind == DrawingDim::Radius ? "R" : "";
    const std::string base = !dim.text.empty() ? dim.text : prefix + num(value);
    const DimTolerance& t = dim.tol;
    switch (t.kind) {
    case DimTolerance::None:
    case DimTolerance::Basic: return base;
    case DimTolerance::Symmetric: return base + " \xC2\xB1" + deviationText(std::fabs(t.upper)).substr(1);
    case DimTolerance::Deviation:
        if (upper) *upper = deviationText(t.upper);
        if (lower) *lower = deviationText(t.lower);
        return base;
    case DimTolerance::Limits:
        if (upper) *upper = limitText(value + t.upper);
        if (lower) *lower = limitText(value + t.lower);
        return prefix;
    case DimTolerance::Fit: {
        std::string s = base + " " + t.fit;
        FitDeviations f;
        if (t.values && isoFit(t.fit, value, f)) s += " (" + deviationText(f.upper) + "/" + deviationText(f.lower) + ")";
        return s;
    }
    }
    return base;
}

bool dimensionLimits(const Drawing& d, const DrawingDim& dim, double& high, double& low) {
    const double value = dimensionValue(d, dim);
    const DimTolerance& t = dim.tol;
    switch (t.kind) {
    case DimTolerance::Symmetric: high = value + std::fabs(t.upper), low = value - std::fabs(t.upper); return true;
    case DimTolerance::Deviation:
    case DimTolerance::Limits: high = value + t.upper, low = value + t.lower; return true;
    case DimTolerance::Fit: {
        FitDeviations f;
        if (!isoFit(t.fit, value, f)) return false;
        high = value + f.upper, low = value + f.lower;
        return true;
    }
    default: return false;
    }
}

SheetGraphics renderDrawing(const Drawing& d, const Scene& scene, int quality) {
    SheetGraphics g;
    if (!sheetSize(d.sheet, g.width, g.height)) g.width = 420, g.height = 297;
    if (!d.landscape) std::swap(g.width, g.height);
    drawSheetFrame(d, g);
    const std::vector<const Body*> bodies = drawnBodies(d, scene);
    if (bodies.empty()) {
        g.warnings.push_back("No bodies to draw");
        return g;
    }
    const Model m = buildModel(bodies, chordFor(d, bodies));
    std::vector<std::pair<std::string, std::array<double, 4>>> rects; // where each view landed
    for (const DrawingView& v : d.views) {
        std::map<std::string, double> ext;
        drawView(d, v, m, g, std::max(1, quality), &ext);
        if (!ext.empty()) rects.push_back({v.name, {ext["x0"], ext["y0"], ext["x1"], ext["y1"]}});
        // The view's name under it (sections and the isometric; standard views go unlabelled in their places).
        if (v.kind == ViewKind::Section || v.kind == ViewKind::Iso || (v.scale > 0 && std::fabs(v.scale - d.scale) > 1e-12)) {
            std::string label = v.kind == ViewKind::Section ? upper(v.name) : upper(v.name);
            if (v.kind == ViewKind::Section) {
                const std::string l = v.name.rfind("Section ", 0) == 0 && v.name.size() > 8 ? v.name.substr(8, 1) : v.name.substr(0, 1);
                label = "SECTION " + upper(l) + "-" + upper(l);
            }
            if (v.scale > 0 && std::fabs(v.scale - d.scale) > 1e-12) label += "  SCALE " + scaleText(v.scale);
            if (!ext.empty()) g.texts.push_back({0.5 * (ext["x0"] + ext["x1"]), ext["y0"] - 9, 3.5, 0, 1, label});
        }
        if (v.kind == ViewKind::Section) drawSectionLine(d, v, m, g);
    }
    std::vector<TextPlace> dimPlaces(d.dims.size());
    for (std::size_t i = 0; i < d.dims.size(); ++i) drawDimension(d, d.dims[i], m, g, &dimPlaces[i]);
    for (const DrawingFrame& f : d.frames) drawFrame(d, f, m, dimPlaces, g);
    for (const DrawingDatum& dt : d.datums) drawDatum(d, dt, m, g);
    for (const DrawingFinish& f : d.finishes) drawFinish(d, f, m, g);
    for (const DrawingNote& n : d.notes) g.texts.push_back({n.x, n.y, n.height, 0, 0, n.text});
    // Views that do not fit: off the sheet, on the title block or on each other.
    const double W = g.width, H = g.height, tol = 0.5;
    const std::array<double, 4> block{W - 10 - std::min(180.0, W - 20), 10, W - 10, 50};
    auto meets = [&](const std::array<double, 4>& a, const std::array<double, 4>& b) {
        return a[0] < b[2] - tol && a[2] > b[0] + tol && a[1] < b[3] - tol && a[3] > b[1] + tol;
    };
    for (std::size_t i = 0; i < rects.size(); ++i) {
        const auto& r = rects[i].second;
        if (r[0] < 10 - tol || r[1] < 10 - tol || r[2] > W - 10 + tol || r[3] > H - 10 + tol)
            g.warnings.push_back(rects[i].first + " runs off the sheet: a smaller scale (SHEET SCALE) or a larger sheet (SHEET SIZE)");
        else if (meets(r, block)) g.warnings.push_back(rects[i].first + " is on the title block: move it (SHEET VIEW MOVE) or use a smaller scale");
        for (std::size_t j = i + 1; j < rects.size(); ++j)
            if (meets(r, rects[j].second)) g.warnings.push_back(rects[i].first + " and " + rects[j].first + " overlap: move one (SHEET VIEW MOVE) or use a smaller scale");
    }
    return g;
}

// --- export -------------------------------------------------------------------------------

namespace {

std::string xmlEscape(const std::string& s) {
    std::string o;
    for (char c : s) {
        if (c == '<') o += "&lt;";
        else if (c == '>') o += "&gt;";
        else if (c == '&') o += "&amp;";
        else if (c == '"') o += "&quot;";
        else o += c;
    }
    return o;
}

struct StyleInfo {
    double width;
    const char* dash; // SVG dash array (mm), or ""
};
StyleInfo styleOf(SheetLine::Style s) {
    switch (s) {
    case SheetLine::Visible: return {0.5, ""};
    case SheetLine::Hidden: return {0.25, "3 1.5"};
    case SheetLine::Center: return {0.25, "8 1.5 1 1.5"};
    case SheetLine::Thin: return {0.18, ""};
    case SheetLine::Border: return {0.7, ""};
    case SheetLine::Cut: return {0.35, "10 1.5 1.5 1.5"};
    }
    return {0.25, ""};
}

} // namespace

std::string sheetToSvg(const SheetGraphics& g) {
    std::string o;
    char b[512];
    std::snprintf(b, sizeof b, "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"%gmm\" height=\"%gmm\" viewBox=\"0 0 %g %g\">\n", g.width, g.height, g.width, g.height);
    o += b;
    o += "<rect x=\"0\" y=\"0\" width=\"100%\" height=\"100%\" fill=\"white\"/>\n";
    for (int st = 0; st <= SheetLine::Cut; ++st) {
        const StyleInfo si = styleOf(static_cast<SheetLine::Style>(st));
        std::string path;
        for (const SheetLine& l : g.lines) {
            if (l.style != st) continue;
            std::snprintf(b, sizeof b, "M%.3f %.3fL%.3f %.3f", l.x0, g.height - l.y0, l.x1, g.height - l.y1);
            path += b;
        }
        if (path.empty()) continue;
        std::snprintf(b, sizeof b, "<path fill=\"none\" stroke=\"black\" stroke-width=\"%g\" stroke-linecap=\"round\"%s%s%s d=\"", si.width, *si.dash ? " stroke-dasharray=\"" : "",
                      si.dash, *si.dash ? "\"" : "");
        o += b + path + "\"/>\n";
    }
    for (const auto& f : g.fills) {
        o += "<polygon fill=\"black\" points=\"";
        for (const auto& [x, y] : f) {
            std::snprintf(b, sizeof b, "%.3f,%.3f ", x, g.height - y);
            o += b;
        }
        o += "\"/>\n";
    }
    for (const SheetText& t : g.texts) {
        const char* anchor = t.align == 1 ? "middle" : t.align == 2 ? "end" : "start";
        std::snprintf(b, sizeof b, "<text x=\"%.3f\" y=\"%.3f\" font-family=\"Helvetica, Arial, sans-serif\" font-size=\"%.3f\" text-anchor=\"%s\" transform=\"rotate(%.3f %.3f %.3f)\">",
                      t.x, g.height - t.y, t.height / 0.72, anchor, -t.angle, t.x, g.height - t.y);
        o += b + xmlEscape(t.text) + "</text>\n";
    }
    o += "</svg>\n";
    return o;
}

namespace {

// Helvetica widths (per 1000) for WinAnsi text: enough to centre text.
double helveticaWidth(unsigned char c) {
    if (c == ' ') return 278;
    if (c >= '0' && c <= '9') return 556;
    if (c == '.' || c == ',' || c == ':' || c == ';' || c == '!' || c == 'i' || c == 'l' || c == 'j' || c == '\'' || c == '|') return 250;
    if (c == '-' || c == '(' || c == ')' || c == '/' || c == 'r' || c == 'f' || c == 't') return 333;
    if (c == 'm' || c == 'M' || c == 'W') return 833;
    if (c == 'w') return 722;
    if (c >= 'A' && c <= 'Z') return c == 'I' ? 278 : 680;
    if (c >= 'a' && c <= 'z') return 520;
    if (c == 216) return 778; // Oslash
    return 556;
}

// UTF-8 to WinAnsi (the few characters drawings use), PDF-escaped.
std::string pdfString(const std::string& s, double* width1000) {
    std::string o;
    double w = 0;
    for (std::size_t i = 0; i < s.size(); ++i) {
        unsigned char c = static_cast<unsigned char>(s[i]);
        if (c >= 0x80 && i + 1 < s.size()) {
            const unsigned cp = ((c & 0x1Fu) << 6) | (static_cast<unsigned char>(s[i + 1]) & 0x3Fu);
            ++i;
            c = cp < 256 ? static_cast<unsigned char>(cp) : '?';
        }
        w += helveticaWidth(c);
        if (c == '(' || c == ')' || c == '\\') o += '\\';
        if (c >= 0x80) {
            char b[8];
            std::snprintf(b, sizeof b, "\\%03o", c);
            o += b;
        } else o += static_cast<char>(c);
    }
    if (width1000) *width1000 = w;
    return o;
}

} // namespace

std::string sheetToPdf(const SheetGraphics& g, const std::string& title) {
    const double k = 72.0 / 25.4; // points per mm
    std::string content;
    char b[512];
    content += "1 J 1 j\n";
    for (int st = 0; st <= SheetLine::Cut; ++st) {
        const StyleInfo si = styleOf(static_cast<SheetLine::Style>(st));
        bool any = false;
        for (const SheetLine& l : g.lines)
            if (l.style == st) {
                if (!any) {
                    std::string dash = "[";
                    for (const char* p = si.dash; *p;) {
                        char* e = nullptr;
                        const double v = std::strtod(p, &e);
                        if (e == p) break;
                        std::snprintf(b, sizeof b, "%s%.2f", dash.size() > 1 ? " " : "", v * k);
                        dash += b;
                        p = e;
                    }
                    std::snprintf(b, sizeof b, "%.3f w %s] 0 d\n", si.width * k, dash.c_str());
                    content += b;
                    any = true;
                }
                std::snprintf(b, sizeof b, "%.2f %.2f m %.2f %.2f l S\n", l.x0 * k, l.y0 * k, l.x1 * k, l.y1 * k);
                content += b;
            }
    }
    content += "[] 0 d\n";
    for (const auto& f : g.fills) {
        for (std::size_t i = 0; i < f.size(); ++i) {
            std::snprintf(b, sizeof b, "%.2f %.2f %s\n", f[i].first * k, f[i].second * k, i ? "l" : "m");
            content += b;
        }
        content += "f\n";
    }
    for (const SheetText& t : g.texts) {
        double w1000 = 0;
        const std::string s = pdfString(t.text, &w1000);
        const double size = t.height / 0.72 * k;
        const double width = w1000 / 1000.0 * size;
        const double a = t.angle * kPiD2 / 180, ca = std::cos(a), sa = std::sin(a);
        const double shift = t.align == 1 ? width / 2 : t.align == 2 ? width : 0;
        const double x = t.x * k - ca * shift, y = t.y * k - sa * shift;
        std::snprintf(b, sizeof b, "BT /F1 %.2f Tf %.5f %.5f %.5f %.5f %.2f %.2f Tm (", size, ca, sa, -sa, ca, x, y);
        content += b + s + ") Tj ET\n";
    }
    std::vector<std::string> objs;
    objs.push_back("<< /Type /Catalog /Pages 2 0 R >>");
    objs.push_back("<< /Type /Pages /Kids [3 0 R] /Count 1 >>");
    std::snprintf(b, sizeof b, "<< /Type /Page /Parent 2 0 R /MediaBox [0 0 %.2f %.2f] /Resources << /Font << /F1 5 0 R >> >> /Contents 4 0 R >>", g.width * k,
                  g.height * k);
    objs.push_back(b);
    objs.push_back("<< /Length " + std::to_string(content.size()) + " >>\nstream\n" + content + "endstream");
    objs.push_back("<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica /Encoding /WinAnsiEncoding >>");
    objs.push_back("<< /Title (" + pdfString(title, nullptr) + ") /Producer (SI-CDM) >>");
    std::string out = "%PDF-1.4\n%\xE2\xE3\xCF\xD3\n";
    std::vector<std::size_t> offs;
    for (std::size_t i = 0; i < objs.size(); ++i) {
        offs.push_back(out.size());
        out += std::to_string(i + 1) + " 0 obj\n" + objs[i] + "\nendobj\n";
    }
    const std::size_t xref = out.size();
    out += "xref\n0 " + std::to_string(objs.size() + 1) + "\n0000000000 65535 f \n";
    for (std::size_t o : offs) {
        std::snprintf(b, sizeof b, "%010zu 00000 n \n", o);
        out += b;
    }
    out += "trailer\n<< /Size " + std::to_string(objs.size() + 1) + " /Root 1 0 R /Info " + std::to_string(objs.size()) + " 0 R >>\nstartxref\n" + std::to_string(xref) +
           "\n%%EOF\n";
    return out;
}

std::string sheetToDxf(const SheetGraphics& g) {
    std::string o;
    char b[256];
    auto pair = [&](int code, const std::string& v) {
        o += std::to_string(code) + "\n" + v + "\n";
    };
    auto real = [&](int code, double v) {
        std::snprintf(b, sizeof b, "%.4f", v);
        pair(code, b);
    };
    static const char* kLayers[] = {"VISIBLE", "HIDDEN", "CENTER", "THIN", "BORDER", "CUT"};
    static const char* kTypes[] = {"CONTINUOUS", "HIDDEN", "CENTER", "CONTINUOUS", "CONTINUOUS", "PHANTOM"};
    pair(0, "SECTION");
    pair(2, "HEADER");
    pair(9, "$ACADVER");
    pair(1, "AC1009");
    pair(9, "$INSUNITS");
    pair(70, "4");
    pair(9, "$EXTMIN");
    real(10, 0);
    real(20, 0);
    pair(9, "$EXTMAX");
    real(10, g.width);
    real(20, g.height);
    pair(0, "ENDSEC");
    pair(0, "SECTION");
    pair(2, "TABLES");
    pair(0, "TABLE");
    pair(2, "LTYPE");
    pair(70, "4");
    struct LT {
        const char* name;
        const char* desc;
        std::vector<double> pat;
    };
    const LT lts[] = {{"CONTINUOUS", "Solid line", {}}, {"HIDDEN", "Hidden __ __ __", {3, -1.5}}, {"CENTER", "Center ____ _ ____", {8, -1.5, 1, -1.5}},
                      {"PHANTOM", "Cut ____ _ _ ____", {10, -1.5, 1.5, -1.5}}};
    for (const LT& lt : lts) {
        pair(0, "LTYPE");
        pair(2, lt.name);
        pair(70, "0");
        pair(3, lt.desc);
        pair(72, "65");
        pair(73, std::to_string(lt.pat.size()));
        double total = 0;
        for (double v : lt.pat) total += std::fabs(v);
        real(40, total);
        for (double v : lt.pat) real(49, v);
    }
    pair(0, "ENDTAB");
    pair(0, "TABLE");
    pair(2, "LAYER");
    pair(70, "6");
    for (int i = 0; i < 6; ++i) {
        pair(0, "LAYER");
        pair(2, kLayers[i]);
        pair(70, "0");
        pair(62, i == 1 ? "8" : i == 2 ? "1" : i == 3 ? "3" : "7");
        pair(6, kTypes[i]);
    }
    pair(0, "ENDTAB");
    pair(0, "ENDSEC");
    pair(0, "SECTION");
    pair(2, "ENTITIES");
    for (const SheetLine& l : g.lines) {
        pair(0, "LINE");
        pair(8, kLayers[l.style]);
        real(10, l.x0);
        real(20, l.y0);
        real(30, 0);
        real(11, l.x1);
        real(21, l.y1);
        real(31, 0);
    }
    for (const auto& f : g.fills) {
        if (f.size() < 3) continue;
        pair(0, "SOLID");
        pair(8, "THIN");
        const std::size_t idx[4] = {0, 1, 2, 2};
        for (int k = 0; k < 4; ++k) {
            real(10 + k, f[idx[k]].first);
            real(20 + k, f[idx[k]].second);
            real(30 + k, 0);
        }
    }
    for (const SheetText& t : g.texts) {
        std::string s;
        for (std::size_t i = 0; i < t.text.size(); ++i) {
            const unsigned char c0 = static_cast<unsigned char>(t.text[i]);
            const unsigned char c1 = i + 1 < t.text.size() ? static_cast<unsigned char>(t.text[i + 1]) : 0;
            if (c0 == 0xC3 && c1 == 0x98) s += "%%c", ++i;      // the diameter sign
            else if (c0 == 0xC2 && c1 == 0xB1) s += "%%p", ++i; // plus-minus
            else if (c0 == 0xC2 && c1 == 0xB0) s += "%%d", ++i; // degrees
            else s += t.text[i];
        }
        pair(0, "TEXT");
        pair(8, "THIN");
        real(10, t.x);
        real(20, t.y);
        real(30, 0);
        real(40, t.height);
        pair(1, s);
        real(50, t.angle);
        if (t.align) {
            pair(72, std::to_string(t.align));
            real(11, t.x);
            real(21, t.y);
            real(31, 0);
        }
    }
    pair(0, "ENDSEC");
    pair(0, "EOF");
    return o;
}

// --- save and load -------------------------------------------------------------------------

Json drawingToJson(const Drawing& d) {
    Json j = Json::object();
    j.set("sheet", d.sheet).set("landscape", d.landscape).set("third_angle", d.thirdAngle).set("scale", d.scale);
    Json bodies = Json::array();
    for (const std::string& b : d.bodies) bodies.push(b);
    j.set("bodies", bodies);
    Json views = Json::array();
    for (const DrawingView& v : d.views) {
        Json o = Json::object();
        o.set("name", v.name).set("kind", viewKindName(v.kind)).set("x", v.x).set("y", v.y).set("scale", v.scale).set("hidden", v.hidden);
        if (v.kind == ViewKind::Section) o.set("parent", v.parent).set("vertical", v.vertical).set("cut", v.cut).set("flip", v.flip);
        views.push(o);
    }
    j.set("views", views);
    static const char* kDims[] = {"horizontal", "vertical", "aligned", "diameter", "radius"};
    Json dims = Json::array();
    for (const DrawingDim& dm : d.dims) {
        Json o = Json::object();
        o.set("kind", kDims[dm.kind]).set("view", dm.view).set("offset", dm.offset);
        o.set("a", Json(Json::Array{Json(dm.a.x), Json(dm.a.y), Json(dm.a.z)})).set("b", Json(Json::Array{Json(dm.b.x), Json(dm.b.y), Json(dm.b.z)}));
        if (!dm.text.empty()) o.set("text", dm.text);
        if (dm.tol.kind != DimTolerance::None) {
            static const char* kTol[] = {"none", "symmetric", "deviation", "limits", "fit", "basic"};
            Json t = Json::object();
            t.set("kind", kTol[dm.tol.kind]).set("upper", dm.tol.upper).set("lower", dm.tol.lower);
            if (!dm.tol.fit.empty()) t.set("fit", dm.tol.fit).set("values", dm.tol.values);
            o.set("tol", t);
        }
        dims.push(o);
    }
    j.set("dims", dims);
    auto point = [](const DVec3& p) { return Json(Json::Array{Json(p.x), Json(p.y), Json(p.z)}); };
    Json frames = Json::array();
    for (const DrawingFrame& f : d.frames) {
        Json o = Json::object();
        o.set("symbol", gdtName(f.symbol)).set("tolerance", f.tolerance).set("diameter", f.diameter);
        if (f.material) o.set("material", std::string(1, f.material));
        Json dl = Json::array();
        for (const std::string& x : f.datums) dl.push(x);
        o.set("datums", dl);
        if (f.dim >= 0) o.set("dim", f.dim);
        else o.set("view", f.view).set("at", point(f.at)).set("dx", f.dx).set("dy", f.dy);
        frames.push(o);
    }
    j.set("frames", frames);
    Json datums = Json::array();
    for (const DrawingDatum& dt : d.datums)
        datums.push(Json::object().set("letter", dt.letter).set("view", dt.view).set("at", point(dt.at)).set("dx", dt.dx).set("dy", dt.dy));
    j.set("datums", datums);
    Json finishes = Json::array();
    for (const DrawingFinish& f : d.finishes)
        finishes.push(Json::object().set("ra", f.ra).set("process", f.process).set("view", f.view).set("at", point(f.at)).set("dx", f.dx).set("dy", f.dy));
    j.set("finishes", finishes);
    if (!d.generalTolerance.empty()) j.set("general_tolerance", d.generalTolerance);
    Json notes = Json::array();
    for (const DrawingNote& n : d.notes) notes.push(Json::object().set("x", n.x).set("y", n.y).set("height", n.height).set("text", n.text));
    j.set("notes", notes);
    Json title = Json::object();
    for (const auto& [k, v] : d.title) title.set(k, v);
    j.set("title", title);
    return j;
}

bool drawingFromJson(const Json& j, Drawing& d) {
    if (!j.isObject()) return false;
    d = Drawing{};
    d.sheet = j["sheet"].asString("A3");
    d.landscape = j["landscape"].asBool(true);
    d.thirdAngle = j["third_angle"].asBool(false);
    d.scale = j["scale"].asNumber(1);
    for (const Json& b : j["bodies"].items()) d.bodies.push_back(b.asString(""));
    for (const Json& o : j["views"].items()) {
        DrawingView v;
        v.name = o["name"].asString("View");
        if (!viewKindFromName(o["kind"].asString("Front"), &v.kind)) v.kind = ViewKind::Front;
        v.x = o["x"].asNumber(0), v.y = o["y"].asNumber(0), v.scale = o["scale"].asNumber(0), v.hidden = o["hidden"].asBool(true);
        v.parent = o["parent"].asString(""), v.vertical = o["vertical"].asBool(true), v.cut = o["cut"].asNumber(0), v.flip = o["flip"].asBool(false);
        d.views.push_back(v);
    }
    static const char* kDims[] = {"horizontal", "vertical", "aligned", "diameter", "radius"};
    for (const Json& o : j["dims"].items()) {
        DrawingDim dm;
        const std::string k = o["kind"].asString("horizontal");
        for (int i = 0; i < 5; ++i)
            if (k == kDims[i]) dm.kind = static_cast<DrawingDim::Kind>(i);
        dm.view = o["view"].asString("");
        dm.offset = o["offset"].asNumber(10);
        const auto& a = o["a"].items();
        const auto& b = o["b"].items();
        if (a.size() == 3) dm.a = {a[0].asNumber(0), a[1].asNumber(0), a[2].asNumber(0)};
        if (b.size() == 3) dm.b = {b[0].asNumber(0), b[1].asNumber(0), b[2].asNumber(0)};
        dm.text = o["text"].asString("");
        if (o["tol"].isObject()) {
            static const char* kTol[] = {"none", "symmetric", "deviation", "limits", "fit", "basic"};
            const std::string tk = o["tol"]["kind"].asString("none");
            for (int i = 0; i < 6; ++i)
                if (tk == kTol[i]) dm.tol.kind = static_cast<DimTolerance::Kind>(i);
            dm.tol.upper = o["tol"]["upper"].asNumber(0), dm.tol.lower = o["tol"]["lower"].asNumber(0);
            dm.tol.fit = o["tol"]["fit"].asString(""), dm.tol.values = o["tol"]["values"].asBool(false);
        }
        d.dims.push_back(dm);
    }
    auto point = [](const Json& a) {
        const auto& v = a.items();
        return v.size() == 3 ? DVec3{v[0].asNumber(0), v[1].asNumber(0), v[2].asNumber(0)} : DVec3{};
    };
    for (const Json& o : j["frames"].items()) {
        DrawingFrame f;
        if (!gdtFromName(o["symbol"].asString(""), &f.symbol)) continue;
        f.tolerance = o["tolerance"].asNumber(0.1), f.diameter = o["diameter"].asBool(false);
        const std::string mc = o["material"].asString("");
        f.material = mc.empty() ? 0 : mc[0];
        for (const Json& x : o["datums"].items()) f.datums.push_back(x.asString(""));
        f.dim = o["dim"].asInt(-1);
        f.view = o["view"].asString(""), f.at = point(o["at"]), f.dx = o["dx"].asNumber(14), f.dy = o["dy"].asNumber(10);
        d.frames.push_back(f);
    }
    for (const Json& o : j["datums"].items())
        d.datums.push_back({o["letter"].asString("A"), o["view"].asString(""), point(o["at"]), o["dx"].asNumber(0), o["dy"].asNumber(-12)});
    for (const Json& o : j["finishes"].items())
        d.finishes.push_back({o["ra"].asNumber(1.6), o["process"].asInt(0), o["view"].asString(""), point(o["at"]), o["dx"].asNumber(0), o["dy"].asNumber(0)});
    d.generalTolerance = j["general_tolerance"].asString("");
    for (const Json& o : j["notes"].items()) d.notes.push_back({o["x"].asNumber(0), o["y"].asNumber(0), o["height"].asNumber(3.5), o["text"].asString("")});
    if (j["title"].isObject())
        for (const auto& [k, v] : j["title"].members()) d.title[k] = v.asString("");
    return true;
}

} // namespace sicdm

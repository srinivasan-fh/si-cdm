#include "core/Route.hpp"
#include "core/Features.hpp"
#include "core/Sculpt.hpp"
#include "core/Sketch.hpp"
#include "core/Units.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <sstream>

namespace sicdm {

namespace {

constexpr double kPiD = 3.14159265358979323846;

// EN 10255 medium series: nominal size, outside diameter and wall (mm).
struct DnSize {
    const char* name;
    float od, wall;
};
constexpr DnSize kDn[] = {{"DN6", 10.2f, 2.0f},   {"DN8", 13.5f, 2.3f},   {"DN10", 17.2f, 2.3f},  {"DN15", 21.3f, 2.6f},  {"DN20", 26.9f, 2.6f},
                          {"DN25", 33.7f, 3.2f},  {"DN32", 42.4f, 3.2f},  {"DN40", 48.3f, 3.2f},  {"DN50", 60.3f, 3.6f},  {"DN65", 76.1f, 3.6f},
                          {"DN80", 88.9f, 4.0f},  {"DN100", 114.3f, 4.5f}, {"DN125", 139.7f, 5.0f}, {"DN150", 165.1f, 5.0f}};

// ASME B36.10M: nominal pipe size, outside diameter, schedule 40 and 80 walls (mm).
struct NpsSize {
    const char* name;
    float od, sch40, sch80;
};
constexpr NpsSize kNps[] = {{"1/8", 10.3f, 1.73f, 2.41f},   {"1/4", 13.7f, 2.24f, 3.02f},   {"3/8", 17.1f, 2.31f, 3.20f},  {"1/2", 21.3f, 2.77f, 3.73f},
                            {"3/4", 26.7f, 2.87f, 3.91f},   {"1", 33.4f, 3.38f, 4.55f},     {"1-1/4", 42.2f, 3.56f, 4.85f}, {"1-1/2", 48.3f, 3.68f, 5.08f},
                            {"2", 60.3f, 3.91f, 5.54f},     {"2-1/2", 73.0f, 5.16f, 7.01f}, {"3", 88.9f, 5.49f, 7.62f},    {"4", 114.3f, 6.02f, 8.56f},
                            {"5", 141.3f, 6.55f, 9.53f},    {"6", 168.3f, 7.11f, 10.97f},   {"8", 219.1f, 8.18f, 12.70f}};

std::string upper(std::string s) {
    for (char& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

std::vector<std::string> words(const std::string& s) {
    std::istringstream in(s);
    std::vector<std::string> w;
    for (std::string t; in >> t;) w.push_back(t);
    return w;
}

bool number(const std::string& w, float& v) {
    if (w.empty()) return false;
    char* end = nullptr;
    const double d = std::strtod(w.c_str(), &end);
    if (end != w.c_str() + w.size() || !std::isfinite(d)) return false;
    v = static_cast<float>(d);
    return true;
}

// "12x1" (or 12X1): two numbers.
bool pairOf(const std::string& w, float& a, float& b) {
    const std::size_t x = w.find_first_of("xX*");
    return x != std::string::npos && number(w.substr(0, x), a) && number(w.substr(x + 1), b);
}

std::string g(double v) {
    char b[32];
    std::snprintf(b, sizeof b, "%g", v);
    return b;
}

bool fail(std::string* why, const std::string& w) {
    if (why) *why = w;
    return false;
}

// A piece of a polyline route's centre line: a straight from a to b, or a bend.
struct Piece {
    bool arc = false;
    Vec3 a, b;
    PathBend bend;
};

// A polyline route's pieces in order (straights through corners that don't turn are one).
bool pieces(const Body::Route& r, std::vector<Piece>& out, std::string* why) {
    out.clear();
    const std::vector<Vec3>& p = r.points;
    const std::size_t n = p.size();
    std::vector<PathBend> bends;
    std::string w;
    if (!pathBends(p, r.closed, r.bend, bends, &w)) return fail(why, w);
    std::vector<const PathBend*> at(n, nullptr);
    for (const PathBend& b : bends) at[b.corner] = &b;
    auto straight = [&](Vec3 a, Vec3 b) {
        if (!(length(b - a) > 1e-6f)) return;
        if (!out.empty() && !out.back().arc) {
            out.back().b = b;
            return;
        }
        Piece s;
        s.a = a, s.b = b;
        out.push_back(s);
    };
    Vec3 cur = p[0];
    auto corner = [&](std::size_t k) {
        if (const PathBend* b = at[k]) {
            straight(cur, b->from);
            Piece s;
            s.arc = true;
            s.a = b->from, s.b = b->to, s.bend = *b;
            out.push_back(s);
            cur = b->to;
        }
    };
    if (!r.closed) {
        for (std::size_t k = 1; k + 1 < n; ++k) corner(k);
        straight(cur, p[n - 1]);
    } else {
        if (at[0]) cur = at[0]->to;
        const Vec3 start = cur;
        for (std::size_t k = 1; k < n; ++k) corner(k);
        if (at[0]) corner(0);
        else straight(cur, start);
        if (out.size() > 1 && !out.front().arc && !out.back().arc) { // the first straight carries on from the last
            out.front().a = out.back().a;
            out.pop_back();
        }
    }
    if (out.empty()) return fail(why, "The path has no length");
    return true;
}

struct Station {
    Vec3 p, t;
    double s = 0; // along the centre line
};

// The centre line as points with their directions: the bends in steps of at most 7.5
// degrees, a spline as finely as `chord`.
bool stations(const Body::Route& r, float chord, std::vector<Station>& st, std::vector<Piece>* outPieces, std::string* why) {
    st.clear();
    auto add = [&](Vec3 p, Vec3 t) {
        Station s;
        s.p = p, s.t = normalize(t);
        s.s = st.empty() ? 0.0 : st.back().s + length(p - st.back().p);
        st.push_back(s);
    };
    if (r.spline) {
        const std::vector<Vec3>& p = r.points;
        const std::size_t n = p.size();
        if (n < 2 || (r.closed && n < 3)) return fail(why, "The path needs more points");
        std::vector<Vec3> q{p[0]};
        const std::size_t spans = r.closed ? n : n - 1;
        for (std::size_t sp = 0; sp < spans; ++sp) {
            const float len = length(p[(sp + 1) % n] - p[sp]);
            const int steps = std::clamp(static_cast<int>(std::ceil(len / std::max(std::sqrt(8.0f * std::max(chord, 1e-3f) * len), 1e-3f) * 2.0f)), 8, 256);
            for (int i = 1; i <= steps; ++i) q.push_back(splinePoint3(p, r.closed, sp, static_cast<float>(i) / static_cast<float>(steps)));
        }
        std::vector<Vec3> clean{q[0]};
        for (std::size_t i = 1; i < q.size(); ++i)
            if (length(q[i] - clean.back()) > 1e-5f) clean.push_back(q[i]);
        if (r.closed) clean.back() = clean.front();
        const std::size_t m = clean.size();
        if (m < 2) return fail(why, "The path has no length");
        for (std::size_t i = 0; i < m; ++i) {
            Vec3 t;
            if (r.closed) {
                const Vec3 prev = clean[i == 0 ? m - 2 : i - 1], next = clean[i + 1 == m ? 1 : i + 1];
                t = next - prev;
            } else {
                t = clean[std::min(i + 1, m - 1)] - clean[i == 0 ? 0 : i - 1];
            }
            if (!(length(t) > 1e-9f)) return fail(why, "The path turns back on itself");
            add(clean[i], t);
        }
        return true;
    }
    std::vector<Piece> ps;
    if (!pieces(r, ps, why)) return false;
    for (const Piece& pc : ps) {
        if (!pc.arc) {
            const Vec3 d = pc.b - pc.a;
            if (st.empty()) add(pc.a, d);
            else st.back().t = normalize(d); // already there: the bend's end, along the straight
            add(pc.b, d);
            continue;
        }
        const PathBend& b = pc.bend;
        const Vec3 v = b.from - b.centre;
        const int segs = std::max(2, static_cast<int>(std::ceil(b.angle / (kPiD / 24))));
        auto tangentAt = [&](Vec3 q) { return cross(b.axis, q - b.centre); };
        if (st.empty()) add(b.from, tangentAt(b.from));
        for (int i = 1; i <= segs; ++i) {
            const float phi = b.angle * static_cast<float>(i) / static_cast<float>(segs);
            const float cs = std::cos(phi), sn = std::sin(phi);
            const Vec3 q = i == segs ? b.to : b.centre + v * cs + cross(b.axis, v) * sn + b.axis * (dot(b.axis, v) * (1 - cs));
            add(q, tangentAt(q));
        }
    }
    if (r.closed && !st.empty()) st.back().p = st.front().p; // round to where it began
    if (outPieces) *outPieces = std::move(ps);
    return true;
}

// v turned about the unit axis k by phi.
Vec3 turn(Vec3 v, Vec3 k, float phi) {
    const float c = std::cos(phi), s = std::sin(phi);
    return v * c + cross(k, v) * s + k * (dot(k, v) * (1 - c));
}

} // namespace

float RouteSpec::defaultBend() const noexcept {
    if (flexible) return 6.0f * od;
    return kind == "pipe" ? 1.5f * od : 2.0f * od;
}

std::optional<RouteSpec> routeSpec(const std::string& text, std::string* why) {
    auto bad = [&](const std::string& w) -> std::optional<RouteSpec> {
        if (why) *why = w;
        return std::nullopt;
    };
    const std::vector<std::string> w = words(upper(text));
    const char* usage = "A size is PIPE DN20, PIPE NPS 3/4 [SCH 40|80], PIPE od x wall, TUBE od x wall, ROD d, CABLE d or HOSE od x wall";
    if (w.empty()) return bad(usage);
    RouteSpec s;
    std::string k = w[0];
    if (k == "BAR") k = "ROD";
    if (k == "WIRE") k = "CABLE";
    if (k == "PIPE") {
        if (w.size() < 2) return bad(usage);
        std::string size = w[1];
        if (size == "DN" && w.size() > 2) size += w[2];
        if (size.rfind("DN", 0) == 0) {
            for (const DnSize& d : kDn)
                if (size == d.name) {
                    s.od = d.od, s.wall = d.wall;
                    s.spec = std::string("PIPE ") + d.name;
                    s.designation = std::string("Pipe ") + d.name + " (" + g(d.od) + " x " + g(d.wall) + ")";
                }
            if (s.spec.empty()) return bad("No pipe " + size + " (EN 10255: DN6 to DN150)");
        } else if (size == "NPS") {
            if (w.size() < 3) return bad("NPS needs a size: PIPE NPS 3/4 SCH 40");
            int sch = 40;
            if (w.size() > 3) {
                if (w[3] != "SCH" && w[3] != "SCHEDULE") return bad(usage);
                if (w.size() < 5) return bad("SCH needs 40 or 80 (STD, XS)");
                const std::string sc = w[4];
                sch = sc == "40" || sc == "STD" ? 40 : sc == "80" || sc == "XS" ? 80 : 0;
                if (!sch) return bad("Schedule 40 (STD) or 80 (XS)");
            }
            for (const NpsSize& d : kNps)
                if (w[2] == d.name) {
                    s.od = d.od, s.wall = sch == 40 ? d.sch40 : d.sch80;
                    s.spec = std::string("PIPE NPS ") + d.name + " SCH " + std::to_string(sch);
                    s.designation = std::string("Pipe NPS ") + d.name + " Sch " + std::to_string(sch) + " (" + g(s.od) + " x " + g(s.wall) + ")";
                }
            if (s.spec.empty()) return bad("No pipe NPS " + w[2] + " (ASME B36.10M: 1/8 to 8)");
        } else if (pairOf(size, s.od, s.wall)) {
            s.spec = "PIPE " + g(s.od) + "x" + g(s.wall);
            s.designation = "Pipe " + g(s.od) + " x " + g(s.wall);
        } else {
            return bad(usage);
        }
        s.kind = "pipe";
    } else if (k == "TUBE" || k == "HOSE") {
        if (w.size() < 2 || !pairOf(w[1], s.od, s.wall)) return bad(k == "TUBE" ? "TUBE od x wall, as TUBE 12x1" : "HOSE od x wall, as HOSE 19x3.5");
        s.kind = k == "TUBE" ? "tube" : "hose";
        s.flexible = k == "HOSE";
        s.spec = k + " " + g(s.od) + "x" + g(s.wall);
        s.designation = (k == "TUBE" ? "Tube " : "Hose ") + g(s.od) + " x " + g(s.wall);
    } else if (k == "ROD" || k == "CABLE") {
        if (w.size() < 2 || !number(w[1], s.od)) return bad(k == "ROD" ? "ROD d, as ROD 10" : "CABLE d, as CABLE 8");
        s.kind = k == "ROD" ? "rod" : "cable";
        s.flexible = k == "CABLE";
        s.wall = 0;
        s.spec = k + " " + g(s.od);
        s.designation = (k == "ROD" ? "Rod " : "Cable ") + g(s.od);
    } else {
        return bad(usage);
    }
    if (!(s.od > 0) || s.od > 5000) return bad("The outside diameter is more than 0 and at most 5 m");
    if (s.wall < 0 || (s.wall > 0 && !(2 * s.wall < s.od))) return bad("The wall must be less than half the outside diameter (a solid one: ROD or CABLE)");
    return s;
}

std::vector<std::string> routeSizes(const std::string& kind) {
    std::vector<std::string> out;
    const std::string k = upper(kind);
    if (k == "PIPE") {
        for (const DnSize& d : kDn) out.push_back(d.name);
        for (const NpsSize& d : kNps) out.push_back(std::string("NPS ") + d.name + " SCH 40"), out.push_back(std::string("NPS ") + d.name + " SCH 80");
    } else if (k == "TUBE") {
        out = {"4x1", "6x1", "8x1", "10x1", "12x1", "12x1.5", "15x1", "16x1.5", "18x1", "20x2", "22x1", "25x2", "28x1.5", "30x2", "35x1.5", "38x3", "42x1.5"};
    } else if (k == "ROD") {
        out = {"4", "5", "6", "8", "10", "12", "16", "20", "25", "30"};
    } else if (k == "CABLE") {
        out = {"3", "4", "5", "6", "8", "10", "12", "16", "20"};
    } else if (k == "HOSE") {
        out = {"10x2.5", "13x3", "16x3.5", "19x3.5", "25x4", "32x4.5", "38x5"};
    }
    return out;
}

std::vector<Vec3> routeCentreline(const Body::Route& r, float chord) {
    std::vector<Station> st;
    if (!stations(r, chord, st, nullptr, nullptr)) return r.points;
    std::vector<Vec3> out;
    for (const Station& s : st) out.push_back(s.p);
    return out;
}

bool buildRoute(const Body::Route& r, MeshData& mesh, std::vector<BSurface>& surfaces, std::string* why) {
    mesh = MeshData{};
    surfaces.clear();
    const float R = 0.5f * r.od, Ri = r.wall > 0 ? R - r.wall : 0.0f;
    if (!(R > 0)) return fail(why, "No size");
    if (!r.spline && !(r.bend > R)) return fail(why, "The bends' radius (" + lengthText(r.bend) + ") must be more than half the outside diameter (" + lengthText(R) + ")");
    std::vector<Station> st;
    std::vector<Piece> ps;
    if (!stations(r, std::max(0.02f * r.od, 0.01f), st, &ps, why)) return false;
    if (st.size() < 2) return fail(why, "The path has no length");
    // A spline tighter than the route: the tube would fold over itself.
    if (r.spline)
        for (std::size_t i = 1; i + 1 < st.size(); ++i) {
            const Vec3 a = st[i].p - st[i - 1].p, b = st[i + 1].p - st[i].p;
            const float la = length(a), lb = length(b), turnA = std::acos(std::clamp(dot(a, b) / std::max(la * lb, 1e-12f), -1.0f, 1.0f));
            if (turnA > 1e-4f && 0.5f * (la + lb) / turnA < R) return fail(why, "The path curves tighter than the route's radius (" + lengthText(R) + ") near " + lengthsText({st[i].p.x, st[i].p.y, st[i].p.z}, ", ", "(", ")"));
        }
    // Frames carried along without twisting; a closed route's left-over twist spread along it.
    std::vector<Vec3> N(st.size());
    {
        const Vec3 t0 = st[0].t;
        N[0] = normalize(cross(t0, std::fabs(t0.z) < 0.9f ? Vec3{0, 0, 1} : Vec3{1, 0, 0}));
        for (std::size_t k = 1; k < st.size(); ++k) {
            const Vec3 a = st[k - 1].t, b = st[k].t, ax = cross(a, b);
            Vec3 n = N[k - 1];
            const float sn = length(ax);
            if (sn > 1e-9f) n = turn(n, ax / sn, std::atan2(sn, dot(a, b)));
            N[k] = normalize(n - b * dot(n, b));
        }
        if (r.closed) {
            const double L = st.back().s;
            const Vec3 t = st.back().t;
            const float alpha = std::atan2(dot(cross(N.back(), N[0]), t), dot(N.back(), N[0]));
            for (std::size_t k = 1; k < st.size(); ++k) N[k] = turn(N[k], st[k].t, static_cast<float>(alpha * st[k].s / L));
            st.pop_back(), N.pop_back();
        }
    }
    const std::size_t S = st.size();
    constexpr int M = 48;
    const bool hollow = Ri > 0;
    auto ring = [&](std::size_t k, int j, float rad) {
        const float phi = 2.0f * static_cast<float>(kPiD) * static_cast<float>(j % M) / M;
        const Vec3 B = cross(st[k].t, N[k]);
        const Vec3 dir = N[k] * std::cos(phi) + B * std::sin(phi);
        return std::make_pair(st[k].p + dir * rad, dir);
    };
    auto vertex = [&](Vec3 p, Vec3 n) {
        Vertex v;
        v.position = p, v.normal = n;
        mesh.vertices.push_back(v);
        return static_cast<std::uint32_t>(mesh.vertices.size() - 1);
    };
    // The sides: outside (and inside), ring by ring.
    std::vector<std::uint32_t> outer(S * M), inner(hollow ? S * M : 0);
    for (std::size_t k = 0; k < S; ++k)
        for (int j = 0; j < M; ++j) {
            const auto [p, d] = ring(k, j, R);
            outer[k * M + static_cast<std::size_t>(j)] = vertex(p, d);
            if (hollow) {
                const auto [q, e] = ring(k, j, Ri);
                inner[k * M + static_cast<std::size_t>(j)] = vertex(q, -e);
            }
        }
    auto tri = [&](std::uint32_t a, std::uint32_t b, std::uint32_t c) { mesh.indices.insert(mesh.indices.end(), {a, b, c}); };
    const std::size_t spans = r.closed ? S : S - 1;
    for (std::size_t k = 0; k < spans; ++k) {
        const std::size_t k1 = (k + 1) % S;
        for (int j = 0; j < M; ++j) {
            const std::size_t j0 = static_cast<std::size_t>(j), j1 = static_cast<std::size_t>((j + 1) % M);
            const std::uint32_t a = outer[k * M + j0], b = outer[k1 * M + j0], c = outer[k1 * M + j1], d = outer[k * M + j1];
            tri(a, d, c), tri(a, c, b);
            if (hollow) {
                const std::uint32_t ia = inner[k * M + j0], ib = inner[k1 * M + j0], ic = inner[k1 * M + j1], id = inner[k * M + j1];
                tri(ia, ic, id), tri(ia, ib, ic);
            }
        }
    }
    // The ends of an open route: rings (or discs) square to it.
    if (!r.closed)
        for (int e = 0; e < 2; ++e) {
            const std::size_t k = e ? S - 1 : 0;
            const Vec3 n = e ? st[k].t : -st[k].t;
            std::vector<std::uint32_t> o(M), in(M);
            for (int j = 0; j < M; ++j) {
                o[static_cast<std::size_t>(j)] = vertex(mesh.vertices[outer[k * M + static_cast<std::size_t>(j)]].position, n);
                if (hollow) in[static_cast<std::size_t>(j)] = vertex(mesh.vertices[inner[k * M + static_cast<std::size_t>(j)]].position, n);
            }
            const std::uint32_t centre = hollow ? 0 : vertex(st[k].p, n);
            for (int j = 0; j < M; ++j) {
                const std::size_t j0 = static_cast<std::size_t>(j), j1 = static_cast<std::size_t>((j + 1) % M);
                if (hollow) {
                    if (e == 0) tri(o[j0], in[j0], in[j1]), tri(o[j0], in[j1], o[j1]);
                    else tri(o[j0], in[j1], in[j0]), tri(o[j0], o[j1], in[j1]);
                } else {
                    if (e == 0) tri(centre, o[j1], o[j0]);
                    else tri(centre, o[j0], o[j1]);
                }
            }
            BSurface pl;
            pl.kind = BSurfaceKind::Plane;
            pl.f = Frame::make(DVec3(st[k].p), DVec3(n), DVec3(N[k]));
            surfaces.push_back(pl);
        }
    // The exact surfaces: a polyline route's straights are cylinders, its bends tori.
    for (const Piece& pc : ps) {
        for (const float rad : {R, Ri}) {
            if (!(rad > 0)) continue;
            BSurface s;
            if (!pc.arc) {
                const Vec3 d = normalize(pc.b - pc.a);
                s.kind = BSurfaceKind::Cylinder;
                s.f = Frame::make(DVec3(pc.a), DVec3(d), DVec3(std::fabs(d.z) < 0.9f ? cross(d, Vec3{0, 0, 1}) : cross(d, Vec3{1, 0, 0})));
                s.r1 = rad;
            } else {
                s.kind = BSurfaceKind::Torus;
                s.f = Frame::make(DVec3(pc.bend.centre), DVec3(pc.bend.axis), DVec3(pc.bend.from - pc.bend.centre));
                s.r1 = pc.bend.radius;
                s.r2 = rad;
            }
            surfaces.push_back(s);
        }
    }
    if (openEdgeCount(mesh) != 0) return fail(why, "The route's surface doesn't close (a bend folds over itself?)");
    return true;
}

RouteReport routeReport(const Body::Route& r) {
    RouteReport rep;
    const double R = 0.5 * r.od, Ri = r.wall > 0 ? R - r.wall : 0.0;
    rep.area = kPiD * (R * R - Ri * Ri);
    if (r.spline) {
        const std::vector<Vec3> c = routeCentreline(r, std::max(0.005f * r.od, 0.005f));
        for (std::size_t i = 1; i < c.size(); ++i) rep.length += length(c[i] - c[i - 1]);
    } else {
        std::vector<Piece> ps;
        if (!pieces(r, ps, &rep.why)) return rep;
        double feed = 0;
        Vec3 prevAxis{};
        bool first = true;
        for (const Piece& pc : ps) {
            if (!pc.arc) {
                const double l = length(pc.b - pc.a);
                feed += l, rep.length += l;
                continue;
            }
            const PathBend& b = pc.bend;
            RouteBend row;
            const Vec3 din = normalize(cross(b.axis, b.from - b.centre)); // the way it was going
            row.corner = b.from + din * b.tangent;
            row.feed = feed;
            row.angle = b.angle * 180.0 / kPiD;
            row.radius = b.radius;
            row.arc = static_cast<double>(b.radius) * b.angle;
            if (!first) row.rotation = std::atan2(static_cast<double>(dot(cross(prevAxis, b.axis), din)), static_cast<double>(dot(prevAxis, b.axis))) * 180.0 / kPiD;
            if (std::fabs(row.rotation) < 5e-7) row.rotation = 0;
            rep.bends.push_back(row);
            rep.length += row.arc;
            prevAxis = b.axis, first = false, feed = 0;
        }
        rep.lastFeed = feed;
    }
    const double rho = materialDensity(r.material).value_or(7.85); // g/cm3
    rep.mass = rep.area * rep.length * rho * 1e-6;
    return rep;
}

} // namespace sicdm

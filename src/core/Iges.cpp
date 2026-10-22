#include "core/Iges.hpp"

#include "core/FileIO.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <set>
#include <sstream>
#include <tuple>

namespace sicdm {

namespace {

constexpr double kPiI = 3.14159265358979323846;

// --- the file -----------------------------------------------------------------------------

struct Param {
    double v = 0;
    std::string s;
    bool isString = false;
    bool empty = true;
};

struct Entity {
    int type = 0;
    int paramLine = 0, paramLines = 0;
    int transform = -1; // entity index of its matrix (124), or -1
    int form = 0;
    int status = 0;     // BBSSUUHH
    std::string label;
    int subscript = 0;
    std::vector<Param> p; // p[0] is the type again
    int subordinate() const { return (status / 10000) % 100; }
};

// Splits parameter text at the delimiters; Hollerith strings (nH...) kept whole.
std::vector<Param> splitParams(const std::string& t, char pd, char rd) {
    std::vector<Param> out;
    std::size_t i = 0;
    Param cur;
    std::string tok;
    auto finish = [&] {
        std::string s;
        for (char ch : tok)
            if (!std::isspace(static_cast<unsigned char>(ch))) s += ch;
        if (!cur.isString) {
            cur.empty = s.empty();
            for (char& ch : s)
                if (ch == 'D' || ch == 'd') ch = 'E';
            cur.v = s.empty() ? 0.0 : std::strtod(s.c_str(), nullptr);
        }
        out.push_back(cur);
        cur = Param{};
        tok.clear();
    };
    while (i < t.size()) {
        const char ch = t[i];
        if (ch == pd) {
            finish();
            ++i;
            continue;
        }
        if (ch == rd) {
            finish();
            break;
        }
        // nH...: a string of n characters.
        if (tok.find_first_not_of(" ") == std::string::npos || tok.empty()) {
            std::size_t j = i;
            while (j < t.size() && t[j] == ' ') ++j;
            std::size_t k = j;
            while (k < t.size() && std::isdigit(static_cast<unsigned char>(t[k]))) ++k;
            if (k > j && k < t.size() && (t[k] == 'H' || t[k] == 'h')) {
                const std::size_t n = static_cast<std::size_t>(std::atoi(t.substr(j, k - j).c_str()));
                cur.s = t.substr(k + 1, n);
                cur.isString = true;
                cur.empty = false;
                i = k + 1 + n;
                continue;
            }
        }
        tok += ch;
        ++i;
    }
    if (!tok.empty() || cur.isString) finish();
    return out;
}

struct File {
    std::vector<Entity> e;
    double unitToMm = 1.0;
    double resolution = 0.0; // in file units (0: not given)
    std::string error;
    std::vector<std::string> warnings;
};

File parseFile(const std::string& text) {
    File f;
    std::vector<std::string> S, G, D, P;
    std::size_t pos = 0;
    while (pos < text.size()) {
        std::size_t eol = text.find('\n', pos);
        if (eol == std::string::npos) eol = text.size();
        std::string l = text.substr(pos, eol - pos);
        pos = eol + 1;
        if (!l.empty() && l.back() == '\r') l.pop_back();
        if (l.size() < 73) {
            if (l.find_first_not_of(" ") == std::string::npos) continue;
            l.resize(80, ' ');
        }
        switch (l[72]) {
        case 'S': S.push_back(l); break;
        case 'G': G.push_back(l); break;
        case 'D': D.push_back(l); break;
        case 'P': P.push_back(l); break;
        case 'T': break;
        case 'C': f.error = "Compressed IGES files aren't supported"; return f;
        default: break;
        }
    }
    if (D.empty() || P.empty()) {
        f.error = "Not an IGES file (no directory or parameter section)";
        return f;
    }
    // Global section: the delimiters, then the unit and resolution.
    std::string g;
    for (const std::string& l : G) g += l.substr(0, 72);
    char pd = ',', rd = ';';
    {
        std::size_t i = 0;
        while (i < g.size() && g[i] == ' ') ++i;
        if (g.compare(i, 2, "1H") == 0 && i + 2 < g.size()) {
            pd = g[i + 2];
            i += 3;
            if (i < g.size() && g[i] == pd) ++i;
            while (i < g.size() && g[i] == ' ') ++i;
            if (g.compare(i, 2, "1H") == 0 && i + 2 < g.size()) rd = g[i + 2];
        } else if (i < g.size() && g[i] != pd) {
            // An empty first field means the defaults.
        }
    }
    const std::vector<Param> gp = splitParams(g, pd, rd);
    auto gnum = [&](std::size_t k, double def) { return k < gp.size() && !gp[k].empty && !gp[k].isString ? gp[k].v : def; };
    auto gstr = [&](std::size_t k) { return k < gp.size() && gp[k].isString ? gp[k].s : std::string(); };
    const int unitFlag = static_cast<int>(gnum(13, 2));
    std::string unitName = gstr(14);
    for (char& ch : unitName) ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    static const double kUnits[] = {1, 25.4, 1, 1, 304.8, 1609344, 1000, 1e6, 0.0254, 0.001, 10, 2.54e-5};
    if (unitFlag >= 1 && unitFlag <= 11 && unitFlag != 3) f.unitToMm = kUnits[unitFlag];
    else {
        static const std::map<std::string, double> kNames = {{"IN", 25.4},  {"INCH", 25.4}, {"MM", 1},       {"FT", 304.8}, {"MI", 1609344}, {"M", 1000},
                                                             {"KM", 1e6},   {"MIL", 0.0254}, {"UM", 0.001}, {"CM", 10},    {"UIN", 2.54e-5}};
        const auto it = kNames.find(unitName);
        f.unitToMm = it != kNames.end() ? it->second : 1.0;
        if (it == kNames.end() && !unitName.empty()) f.warnings.push_back("Unknown unit '" + unitName + "': read as millimetres");
    }
    f.resolution = gnum(18, 0.0);

    // Directory: two lines per entity.
    auto field = [](const std::string& l, int k) { return l.substr(static_cast<std::size_t>(k) * 8, 8); };
    auto ifield = [&](const std::string& l, int k) {
        const std::string s = field(l, k);
        return s.find_first_not_of(' ') == std::string::npos ? 0 : std::atoi(s.c_str());
    };
    f.e.resize(D.size() / 2);
    for (std::size_t i = 0; i + 1 < D.size(); i += 2) {
        Entity& e = f.e[i / 2];
        e.type = ifield(D[i], 0);
        e.paramLine = ifield(D[i], 1);
        const int t = ifield(D[i], 6);
        e.transform = t > 0 ? (t - 1) / 2 : -1;
        e.status = ifield(D[i], 8);
        e.paramLines = ifield(D[i + 1], 3);
        e.form = ifield(D[i + 1], 4);
        e.label = field(D[i + 1], 7);
        while (!e.label.empty() && e.label.back() == ' ') e.label.pop_back();
        while (!e.label.empty() && e.label.front() == ' ') e.label.erase(0, 1);
        e.subscript = ifield(D[i + 1], 8);
    }
    // Parameters: each entity's lines, up to the record delimiter.
    for (Entity& e : f.e) {
        if (e.paramLine < 1 || e.paramLine > static_cast<int>(P.size())) continue;
        std::string t;
        const int n = std::max(1, e.paramLines);
        for (int k = 0; k < n && e.paramLine - 1 + k < static_cast<int>(P.size()); ++k) t += P[static_cast<std::size_t>(e.paramLine - 1 + k)].substr(0, 64);
        e.p = splitParams(t, pd, rd);
    }
    return f;
}

// --- geometry -----------------------------------------------------------------------------

// x' = R x + T.
struct Affine {
    double r[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    double t[3] = {0, 0, 0};
    DVec3 point(const DVec3& p) const { return {r[0] * p.x + r[1] * p.y + r[2] * p.z + t[0], r[3] * p.x + r[4] * p.y + r[5] * p.z + t[1], r[6] * p.x + r[7] * p.y + r[8] * p.z + t[2]}; }
    DVec3 dir(const DVec3& d) const { return {r[0] * d.x + r[1] * d.y + r[2] * d.z, r[3] * d.x + r[4] * d.y + r[5] * d.z, r[6] * d.x + r[7] * d.y + r[8] * d.z}; }
    double scale() const { return dlength(dir({1, 0, 0})); }
    // This, then `outer`.
    Affine then(const Affine& o) const {
        Affine a;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) a.r[i * 3 + j] = o.r[i * 3] * r[j] + o.r[i * 3 + 1] * r[3 + j] + o.r[i * 3 + 2] * r[6 + j];
            a.t[i] = o.r[i * 3] * t[0] + o.r[i * 3 + 1] * t[1] + o.r[i * 3 + 2] * t[2] + o.t[i];
        }
        return a;
    }
};

// A piece of a curve, from a to b.
struct Piece {
    BCurve c;
    bool straight = false;
    DVec3 a, b;
    double t0 = 0, t1 = 1; // the curve's parameters at a and b (circles and ellipses: angles from a)
    DVec3 at(double f) const {
        if (straight) return a + (b - a) * f;
        if (c.kind == BCurveKind::Polyline) {
            const double x = f * static_cast<double>(c.pts.size() - 1);
            const std::size_t i = std::min(static_cast<std::size_t>(x), c.pts.size() - 2);
            return c.pts[i] + (c.pts[i + 1] - c.pts[i]) * (x - static_cast<double>(i));
        }
        return c.eval(t0 + (t1 - t0) * f);
    }
};

class Reader {
public:
    explicit Reader(File& f) : m_f(f) {}

    const Entity* entity(int i) const { return i >= 0 && i < static_cast<int>(m_f.e.size()) ? &m_f.e[static_cast<std::size_t>(i)] : nullptr; }
    static double num(const Entity& e, std::size_t k) { return k < e.p.size() ? e.p[k].v : 0.0; }
    static int ptr(const Entity& e, std::size_t k) {
        const int v = static_cast<int>(std::lround(std::fabs(num(e, k))));
        return v > 0 ? (v - 1) / 2 : -1;
    }
    void warn(const std::string& w) {
        if (m_warned.insert(w).second) m_f.warnings.push_back(w);
    }

    // The entity's own matrix chain.
    Affine own(int i) const {
        Affine a;
        std::set<int> seen;
        const Entity* e = entity(i);
        int t = e ? e->transform : -1;
        while (t >= 0 && seen.insert(t).second) {
            const Entity* m = entity(t);
            if (!m || m->type != 124) break;
            Affine x;
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) x.r[r * 3 + c] = num(*m, static_cast<std::size_t>(1 + r * 4 + c));
                x.t[r] = num(*m, static_cast<std::size_t>(1 + r * 4 + 3));
            }
            a = a.then(x);
            t = m->transform;
        }
        return a;
    }
    Affine placed(int i, const Affine& parent) const { return own(i).then(parent); }

    DVec3 pointOf(int i, const Affine& parent) const {
        const Entity* e = entity(i);
        if (!e) return {0, 0, 0};
        return placed(i, parent).point({num(*e, 1), num(*e, 2), num(*e, 3)});
    }
    DVec3 dirOf(int i, const Affine& parent) const {
        const Entity* e = entity(i);
        if (!e) return {0, 0, 1};
        return dnormalize(placed(i, parent).dir({num(*e, 1), num(*e, 2), num(*e, 3)}));
    }

    // A curve entity as pieces (composites flattened). False if it can't be read.
    bool pieces(int i, const Affine& parent, std::vector<Piece>& out, int depth = 0) {
        const Entity* e = entity(i);
        if (!e || depth > 20) return false;
        const Affine T = placed(i, parent);
        switch (e->type) {
        case 110: {
            Piece p;
            p.straight = true;
            p.a = T.point({num(*e, 1), num(*e, 2), num(*e, 3)});
            p.b = T.point({num(*e, 4), num(*e, 5), num(*e, 6)});
            if (dlength(p.b - p.a) > 0) out.push_back(p);
            return true;
        }
        case 100: {
            const double z = num(*e, 1);
            const DVec3 c{num(*e, 2), num(*e, 3), z}, s{num(*e, 4), num(*e, 5), z}, en{num(*e, 6), num(*e, 7), z};
            const double r = std::hypot(s.x - c.x, s.y - c.y);
            if (!(r > 0)) return false;
            const double a0 = std::atan2(s.y - c.y, s.x - c.x), a1 = std::atan2(en.y - c.y, en.x - c.x);
            double sweep = a1 - a0;
            while (sweep <= 1e-12) sweep += 2 * kPiI;
            if (std::hypot(en.x - s.x, en.y - s.y) < 1e-9 * std::max(1.0, r)) sweep = 2 * kPiI;
            Piece p;
            p.c.kind = BCurveKind::Circle;
            const DVec3 x = dnormalize(T.dir({(s.x - c.x) / r, (s.y - c.y) / r, 0})), y = dnormalize(T.dir({-(s.y - c.y) / r, (s.x - c.x) / r, 0}));
            p.c.f.o = T.point(c);
            p.c.f.x = x;
            p.c.f.y = y;
            p.c.f.z = dcross(x, y);
            p.c.r1 = r * T.scale();
            p.t0 = 0, p.t1 = sweep;
            p.a = T.point(s);
            p.b = sweep >= 2 * kPiI - 1e-12 ? p.a : T.point(en);
            out.push_back(p);
            return true;
        }
        case 104: {
            const double A = num(*e, 1), B = num(*e, 2), C = num(*e, 3), D = num(*e, 4), E = num(*e, 5), F = num(*e, 6), z = num(*e, 7);
            if (e->form != 1 && !(A * C > 0 && F * A < 0)) {
                warn("Hyperbolic and parabolic arcs (IGES 104) aren't read");
                return false;
            }
            if (std::fabs(B) + std::fabs(D) + std::fabs(E) > 1e-9 * (std::fabs(A) + std::fabs(C))) {
                warn("Ellipses not centred on their own axes (IGES 104) aren't read");
                return false;
            }
            const double ra = std::sqrt(-F / A), rb = std::sqrt(-F / C);
            const DVec3 s{num(*e, 8), num(*e, 9), z}, en{num(*e, 10), num(*e, 11), z};
            const double a0 = std::atan2(s.y / rb, s.x / ra), a1 = std::atan2(en.y / rb, en.x / ra);
            double sweep = a1 - a0;
            while (sweep <= 1e-12) sweep += 2 * kPiI;
            if (std::hypot(en.x - s.x, en.y - s.y) < 1e-9 * std::max(1.0, ra)) sweep = 2 * kPiI;
            Piece p;
            p.c.kind = BCurveKind::Ellipse;
            const DVec3 x = dnormalize(T.dir({1, 0, 0})), y = dnormalize(T.dir({0, 1, 0}));
            p.c.f.o = T.point({0, 0, z});
            p.c.f.x = x;
            p.c.f.y = y;
            p.c.f.z = dcross(x, y);
            p.c.r1 = ra * T.scale();
            p.c.r2 = rb * T.scale();
            p.t0 = a0, p.t1 = a0 + sweep;
            p.a = T.point(s);
            p.b = sweep >= 2 * kPiI - 1e-12 ? p.a : T.point(en);
            out.push_back(p);
            return true;
        }
        case 106: {
            const int ip = static_cast<int>(num(*e, 1)), n = static_cast<int>(num(*e, 2));
            if (e->form != 11 && e->form != 12 && e->form != 13 && e->form != 63) return true; // points, not a curve
            Piece p;
            p.c.kind = BCurveKind::Polyline;
            std::size_t k = 3;
            const double z = ip == 1 ? num(*e, k++) : 0.0;
            for (int j = 0; j < n; ++j) {
                if (ip == 1) p.c.pts.push_back(T.point({num(*e, k), num(*e, k + 1), z})), k += 2;
                else {
                    p.c.pts.push_back(T.point({num(*e, k), num(*e, k + 1), num(*e, k + 2)}));
                    k += ip == 3 ? 6 : 3;
                }
            }
            if (e->form == 63 && !p.c.pts.empty()) p.c.pts.push_back(p.c.pts.front());
            if (p.c.pts.size() < 2) return false;
            p.a = p.c.pts.front(), p.b = p.c.pts.back();
            out.push_back(p);
            return true;
        }
        case 126: {
            const int K = static_cast<int>(num(*e, 1)), M = static_cast<int>(num(*e, 2));
            if (K < 0 || M < 1 || K > 100000) return false;
            const bool polynomial = num(*e, 5) == 1;
            std::size_t k = 7;
            Piece p;
            p.c.kind = BCurveKind::BSpline;
            BSplineCurve& bs = p.c.bs;
            bs.degree = M;
            for (int j = 0; j < K + M + 2; ++j) bs.knots.push_back(num(*e, k++));
            std::vector<double> w;
            for (int j = 0; j <= K; ++j) w.push_back(num(*e, k++));
            for (int j = 0; j <= K; ++j, k += 3) bs.cp.push_back(T.point({num(*e, k), num(*e, k + 1), num(*e, k + 2)}));
            double v0 = num(*e, k), v1 = num(*e, k + 1);
            bool equal = true;
            for (double x : w) equal = equal && std::fabs(x - w[0]) < 1e-12 * std::max(1.0, std::fabs(w[0]));
            if (!polynomial && !equal) bs.w = w;
            if (bs.cp.size() < static_cast<std::size_t>(M + 1)) return false;
            if (!(v1 > v0)) v0 = bs.t0(), v1 = bs.t1();
            v0 = std::max(v0, bs.t0()), v1 = std::min(v1, bs.t1());
            p.t0 = v0, p.t1 = v1;
            p.a = bs.eval(v0), p.b = bs.eval(v1);
            out.push_back(p);
            return true;
        }
        case 102: {
            const int n = static_cast<int>(num(*e, 1));
            for (int j = 0; j < n; ++j)
                if (!pieces(ptr(*e, static_cast<std::size_t>(2 + j)), T, out, depth + 1)) return false;
            return true;
        }
        default:
            warn("IGES curves of type " + std::to_string(e->type) + " aren't read");
            return false;
        }
    }

    // A curve's piece as a B-rep curve (-1 for a straight one).
    static int addCurve(Brep& b, const Piece& p) {
        if (p.straight) return -1;
        b.curves.push_back(p.c);
        return static_cast<int>(b.curves.size()) - 1;
    }

    // A surface entity in `b`; -1 if it can't be read.
    int surface(int i, const Affine& parent, Brep& b) {
        const Entity* e = entity(i);
        if (!e) return -1;
        const Affine T = placed(i, parent);
        BSurface s;
        auto frameFrom = [&](const DVec3& o, const DVec3& z, const DVec3* ref) {
            DVec3 x = ref ? *ref : (std::fabs(z.z) < 0.9 ? DVec3{0, 0, 1} : DVec3{1, 0, 0});
            if (!ref) x = dcross(x, z);
            return Frame::make(o, z, x);
        };
        switch (e->type) {
        case 108: {
            const DVec3 n{num(*e, 1), num(*e, 2), num(*e, 3)};
            const double l2 = ddot(n, n);
            if (!(l2 > 0)) return -1;
            s.kind = BSurfaceKind::Plane;
            s.f = frameFrom(T.point(n * (num(*e, 4) / l2)), dnormalize(T.dir(n)), nullptr);
            break;
        }
        case 190: {
            s.kind = BSurfaceKind::Plane;
            const DVec3 z = dirOf(ptr(*e, 2), T);
            const DVec3 ref = e->form == 1 ? dirOf(ptr(*e, 3), T) : DVec3{};
            s.f = frameFrom(pointOf(ptr(*e, 1), T), z, e->form == 1 ? &ref : nullptr);
            break;
        }
        case 192:
        case 194: {
            s.kind = e->type == 192 ? BSurfaceKind::Cylinder : BSurfaceKind::Cone;
            const DVec3 z = dirOf(ptr(*e, 2), T);
            const std::size_t refAt = e->type == 192 ? 4 : 5;
            const DVec3 ref = e->form == 1 ? dirOf(ptr(*e, refAt), T) : DVec3{};
            s.f = frameFrom(pointOf(ptr(*e, 1), T), z, e->form == 1 ? &ref : nullptr);
            s.r1 = num(*e, 3) * T.scale();
            if (e->type == 194) s.r2 = num(*e, 4) * kPiI / 180.0; // the semi-angle, in degrees
            break;
        }
        case 196: {
            s.kind = BSurfaceKind::Sphere;
            const DVec3 z = e->form == 1 ? dirOf(ptr(*e, 3), T) : dnormalize(T.dir({0, 0, 1}));
            const DVec3 ref = e->form == 1 ? dirOf(ptr(*e, 4), T) : DVec3{};
            s.f = frameFrom(pointOf(ptr(*e, 1), T), z, e->form == 1 ? &ref : nullptr);
            s.r1 = num(*e, 2) * T.scale();
            break;
        }
        case 198: {
            s.kind = BSurfaceKind::Torus;
            const DVec3 z = dirOf(ptr(*e, 2), T);
            const DVec3 ref = e->form == 1 ? dirOf(ptr(*e, 5), T) : DVec3{};
            s.f = frameFrom(pointOf(ptr(*e, 1), T), z, e->form == 1 ? &ref : nullptr);
            s.r1 = num(*e, 3) * T.scale();
            s.r2 = num(*e, 4) * T.scale();
            break;
        }
        case 128: {
            const int K1 = static_cast<int>(num(*e, 1)), K2 = static_cast<int>(num(*e, 2)), M1 = static_cast<int>(num(*e, 3)), M2 = static_cast<int>(num(*e, 4));
            if (K1 < 0 || K2 < 0 || M1 < 1 || M2 < 1 || static_cast<long long>(K1 + 1) * (K2 + 1) > 4000000) return -1;
            const bool polynomial = num(*e, 7) == 1;
            s.kind = BSurfaceKind::BSpline;
            BSplineSurface& bs = s.bs;
            bs.du = M1, bs.dv = M2, bs.nu = K1 + 1, bs.nv = K2 + 1;
            std::size_t k = 10;
            for (int j = 0; j < K1 + M1 + 2; ++j) bs.ku.push_back(num(*e, k++));
            for (int j = 0; j < K2 + M2 + 2; ++j) bs.kv.push_back(num(*e, k++));
            const std::size_t n = static_cast<std::size_t>(bs.nu) * static_cast<std::size_t>(bs.nv);
            std::vector<double> w(n);
            bs.cp.resize(n);
            // IGES lists them with the first index fastest; here cp[i * nv + j], i along u.
            for (int j = 0; j <= K2; ++j)
                for (int i2 = 0; i2 <= K1; ++i2) w[static_cast<std::size_t>(i2 * bs.nv + j)] = num(*e, k++);
            for (int j = 0; j <= K2; ++j)
                for (int i2 = 0; i2 <= K1; ++i2, k += 3) bs.cp[static_cast<std::size_t>(i2 * bs.nv + j)] = T.point({num(*e, k), num(*e, k + 1), num(*e, k + 2)});
            bool equal = true;
            for (double x : w) equal = equal && std::fabs(x - w[0]) < 1e-12 * std::max(1.0, std::fabs(w[0]));
            if (!polynomial && !equal) bs.w = w;
            break;
        }
        case 120: {
            std::vector<Piece> axis, gen;
            if (!pieces(ptr(*e, 1), T, axis) || axis.size() != 1 || !pieces(ptr(*e, 2), T, gen)) return -1;
            if (gen.size() != 1) {
                warn("Surfaces of revolution with composite generatrices aren't read");
                return -1;
            }
            const DVec3 o = axis[0].a, z = dnormalize(axis[0].b - axis[0].a);
            if (analyticRevolution(o, z, gen[0], s)) break;
            s.kind = BSurfaceKind::Revolution;
            s.f = frameFrom(o, z, nullptr);
            s.curve = curveOrLine(b, gen[0]);
            break;
        }
        case 122: {
            std::vector<Piece> dir;
            if (!pieces(ptr(*e, 1), T, dir) || dir.size() != 1) {
                warn("Tabulated cylinders on composite curves aren't read");
                return -1;
            }
            s.kind = BSurfaceKind::Extrusion;
            s.curve = curveOrLine(b, dir[0]);
            s.dir = T.point({num(*e, 2), num(*e, 3), num(*e, 4)}) - dir[0].a;
            break;
        }
        default:
            warn("IGES surfaces of type " + std::to_string(e->type) + " aren't read");
            return -1;
        }
        b.surfaces.push_back(s);
        return static_cast<int>(b.surfaces.size()) - 1;
    }

    // A surface of revolution that is a plane, cylinder, cone, sphere or torus: a line or
    // circular arc turned about an axis in its plane. Those keep their exact, simpler form
    // (and their poles and apexes are known for what they are).
    static bool analyticRevolution(const DVec3& o, const DVec3& z, const Piece& g, BSurface& s) {
        auto foot = [&](const DVec3& p) { return o + z * ddot(p - o, z); };
        auto radial = [&](const DVec3& p) { return dlength(p - foot(p)); };
        const double size = std::max({1.0, dlength(g.a - o), dlength(g.b - o)});
        const double tol = 1e-9 * size;
        if (g.straight) {
            const DVec3 d = g.b - g.a;
            const double len = dlength(d);
            if (!(len > 0)) return false;
            // In a plane through the axis.
            if (std::fabs(ddot(dcross(g.a - o, g.b - o), z)) > tol * len) return false;
            // Radii measured one way across the axis (a line through it has one end on each side).
            const DVec3 far = radial(g.a) >= radial(g.b) ? g.a : g.b;
            if (radial(far) <= tol) return false; // along the axis itself
            const DVec3 ref = dnormalize(far - foot(far));
            const double ha = ddot(g.a - o, z), hb = ddot(g.b - o, z), ra = ddot(g.a - foot(g.a), ref), rb = ddot(g.b - foot(g.b), ref);
            if (std::fabs(hb - ha) <= tol) { // square to the axis: a plane
                s.kind = BSurfaceKind::Plane;
                s.f = Frame::make(foot(g.a), z, ref);
                return true;
            }
            if (std::fabs(rb - ra) <= tol) {
                s.kind = BSurfaceKind::Cylinder;
                s.f = Frame::make(foot(g.a), z, ref);
                s.r1 = ra;
                return true;
            }
            // A cone opening the way its radius grows, from the radius at the far end.
            const double k = (rb - ra) / (hb - ha);
            s.kind = BSurfaceKind::Cone;
            s.f = Frame::make(foot(far), k > 0 ? z : -z, ref);
            s.r1 = radial(far);
            s.r2 = std::atan(std::fabs(k));
            return true;
        }
        if (g.c.kind != BCurveKind::Circle) return false;
        const DVec3 c = g.c.f.o;
        // The arc's plane holds the axis.
        if (std::fabs(ddot(g.c.f.z, z)) > 1e-9 || std::fabs(ddot(c - o, g.c.f.z)) > tol) return false;
        const double R = radial(c);
        const DVec3 ref = R > tol ? dnormalize(c - foot(c)) : dnormalize(dcross(g.c.f.z, z));
        if (R <= tol) {
            s.kind = BSurfaceKind::Sphere;
            s.f = Frame::make(foot(c), z, ref);
            s.r1 = g.c.r1;
            return true;
        }
        s.kind = BSurfaceKind::Torus;
        s.f = Frame::make(foot(c), z, ref);
        s.r1 = R;
        s.r2 = g.c.r1;
        return true;
    }

    // A swept curve: the piece's curve, a line for a straight one.
    static int curveOrLine(Brep& b, const Piece& p) {
        if (!p.straight) {
            b.curves.push_back(p.c);
            return static_cast<int>(b.curves.size()) - 1;
        }
        BCurve l;
        l.kind = BCurveKind::Line;
        l.f.o = p.a;
        l.f.x = dnormalize(p.b - p.a);
        b.curves.push_back(l);
        return static_cast<int>(b.curves.size()) - 1;
    }

    File& m_f;
    std::set<std::string> m_warned;
};

// --- sewing ---------------------------------------------------------------------------------

// Vertices within `tol` become one; pieces between the same vertices along the same
// curve become one edge, used by both faces.
class Sewer {
public:
    Sewer(Brep& b, double tol) : m_b(b), m_tol(tol) {}

    int vertex(const DVec3& p) {
        const auto [cx, cy, cz] = cellOf(p);
        for (long long dx = -1; dx <= 1; ++dx)
            for (long long dy = -1; dy <= 1; ++dy)
                for (long long dz = -1; dz <= 1; ++dz) {
                    const auto it = m_grid.find({cx + dx, cy + dy, cz + dz});
                    if (it == m_grid.end()) continue;
                    for (int v : it->second)
                        if (dlength(m_b.vertices[static_cast<std::size_t>(v)].p - p) <= m_tol) return v;
                }
        m_b.vertices.push_back({p});
        const int v = static_cast<int>(m_b.vertices.size()) - 1;
        m_grid[{cx, cy, cz}].push_back(v);
        return v;
    }

    // The piece as an edge of a loop (false: it has no length). It is the edge already
    // made between the same vertices when it lies along that edge's curve (two
    // descriptions of one ellipse need not share their parameters).
    bool edge(const Piece& pc, OrientedEdge& out) {
        const int va = vertex(pc.a), vb = vertex(pc.b);
        if (va == vb && dlength(pc.at(0.5) - pc.a) <= m_tol && dlength(pc.at(0.25) - pc.a) <= m_tol) return false; // collapsed (a pole)
        const std::pair<int, int> key{std::min(va, vb), std::max(va, vb)};
        for (const Rec& r : m_edges[key]) {
            bool along = true;
            for (double fr : {0.13, 0.31, 0.5, 0.69, 0.87}) along = along && distanceTo(r.piece, pc.at(fr)) <= 10 * m_tol;
            if (!along) continue;
            const BrepEdge& E = m_b.edges[static_cast<std::size_t>(r.edge)];
            if (va != vb) out = {r.edge, E.v0 == va};
            else out = {r.edge, fractionOn(r.piece, pc.at(0.2)) > fractionOn(r.piece, pc.at(0.1))};
            return true;
        }
        m_b.edges.push_back({va, vb, Reader::addCurve(m_b, pc), true});
        const int id = static_cast<int>(m_b.edges.size()) - 1;
        m_edges[key].push_back({id, pc});
        out = {id, true};
        return true;
    }

private:
    std::tuple<long long, long long, long long> cellOf(const DVec3& p) const {
        const double c = std::max(m_tol * 2, 1e-12);
        return {static_cast<long long>(std::floor(p.x / c)), static_cast<long long>(std::floor(p.y / c)), static_cast<long long>(std::floor(p.z / c))};
    }
    struct Rec {
        int edge;
        Piece piece;
    };
    // How far p is from the piece's curve.
    static double distanceTo(const Piece& pc, const DVec3& p) {
        if (pc.straight) {
            const DVec3 d = pc.b - pc.a;
            const double l2 = ddot(d, d);
            const double t = l2 > 0 ? std::clamp(ddot(p - pc.a, d) / l2, 0.0, 1.0) : 0.0;
            return dlength(pc.a + d * t - p);
        }
        if (pc.c.kind == BCurveKind::Polyline) return dlength(pc.at(fractionOn(pc, p)) - p);
        // The nearest point of the curve, if it is on this piece of it; else an end.
        double t = pc.c.param(p);
        if (pc.c.periodic()) t = pc.t0 + std::fmod(std::fmod(t - pc.t0, 2 * kPiI) + 2 * kPiI, 2 * kPiI);
        if (t >= std::min(pc.t0, pc.t1) - 1e-12 && t <= std::max(pc.t0, pc.t1) + 1e-12) return dlength(pc.c.eval(t) - p);
        return std::min(dlength(pc.a - p), dlength(pc.b - p));
    }
    // Where along the piece (0 to 1) p lies: the nearest of its samples, refined.
    static double fractionOn(const Piece& pc, const DVec3& p) {
        constexpr int n = 256;
        int best = 0;
        double bd = 1e300;
        for (int i = 0; i <= n; ++i) {
            const double d = dlength(pc.at(static_cast<double>(i) / n) - p);
            if (d < bd) bd = d, best = i;
        }
        return static_cast<double>(best) / n;
    }
    Brep& m_b;
    double m_tol;
    std::map<std::tuple<long long, long long, long long>, std::vector<int>> m_grid;
    std::map<std::pair<int, int>, std::vector<Rec>> m_edges;
};

// The faces listed, in a B-rep of their own (vertices, edges, curves and surfaces copied).
std::shared_ptr<Brep> extract(const Brep& src, const std::vector<std::vector<int>>& shellFaces, const std::vector<bool>& closed) {
    auto out = std::make_shared<Brep>();
    std::map<int, int> vmap, emap, cmap, smap;
    auto curve = [&](int c) -> int {
        if (c < 0) return -1;
        const auto [it, added] = cmap.try_emplace(c, static_cast<int>(out->curves.size()));
        if (added) out->curves.push_back(src.curves[static_cast<std::size_t>(c)]);
        return it->second;
    };
    auto vertex = [&](int v) -> int {
        if (v < 0) return -1;
        const auto [it, added] = vmap.try_emplace(v, static_cast<int>(out->vertices.size()));
        if (added) out->vertices.push_back(src.vertices[static_cast<std::size_t>(v)]);
        return it->second;
    };
    auto edge = [&](int e) -> int {
        const auto it = emap.find(e);
        if (it != emap.end()) return it->second;
        BrepEdge E = src.edges[static_cast<std::size_t>(e)];
        E.v0 = vertex(E.v0), E.v1 = vertex(E.v1), E.curve = curve(E.curve);
        out->edges.push_back(E);
        return emap[e] = static_cast<int>(out->edges.size()) - 1;
    };
    auto surface = [&](int s) -> int {
        const auto [it, added] = smap.try_emplace(s, static_cast<int>(out->surfaces.size()));
        if (added) {
            BSurface S = src.surfaces[static_cast<std::size_t>(s)];
            if (S.curve >= 0) S.curve = curve(S.curve);
            out->surfaces.push_back(S);
        }
        return it->second;
    };
    std::size_t k = 0;
    for (const std::vector<int>& faces : shellFaces) {
        BShell sh;
        sh.closed = closed[k++];
        for (int fi : faces) {
            // fi indexes the flattened face list of src.
            int n = fi;
            for (const BShell& s : src.shells) {
                if (n < static_cast<int>(s.faces.size())) {
                    BFace f = s.faces[static_cast<std::size_t>(n)];
                    f.surface = surface(f.surface);
                    for (BLoop& l : f.loops) {
                        for (OrientedEdge& oe : l.edges) oe.edge = edge(oe.edge);
                        if (l.vertex >= 0) l.vertex = vertex(l.vertex);
                    }
                    sh.faces.push_back(std::move(f));
                    break;
                }
                n -= static_cast<int>(s.faces.size());
            }
        }
        out->shells.push_back(std::move(sh));
    }
    return out;
}

// Faces that share edges, in groups; and whether each group closes (every edge used twice).
void components(const Brep& b, std::vector<std::vector<int>>& groups, std::vector<bool>& closed) {
    std::vector<const BFace*> faces;
    for (const BShell& s : b.shells)
        for (const BFace& f : s.faces) faces.push_back(&f);
    std::vector<int> parent(faces.size());
    for (std::size_t i = 0; i < parent.size(); ++i) parent[i] = static_cast<int>(i);
    std::function<int(int)> find = [&](int x) { return parent[static_cast<std::size_t>(x)] == x ? x : parent[static_cast<std::size_t>(x)] = find(parent[static_cast<std::size_t>(x)]); };
    std::map<int, int> firstFace;
    std::map<int, int> uses;
    for (std::size_t i = 0; i < faces.size(); ++i)
        for (const BLoop& l : faces[i]->loops)
            for (const OrientedEdge& oe : l.edges) {
                ++uses[oe.edge];
                const auto [it, added] = firstFace.try_emplace(oe.edge, static_cast<int>(i));
                if (!added) parent[static_cast<std::size_t>(find(static_cast<int>(i)))] = find(it->second);
            }
    std::map<int, int> groupOf;
    for (std::size_t i = 0; i < faces.size(); ++i) {
        const int r = find(static_cast<int>(i));
        const auto [it, added] = groupOf.try_emplace(r, static_cast<int>(groups.size()));
        if (added) groups.emplace_back(), closed.push_back(true);
        groups[static_cast<std::size_t>(it->second)].push_back(static_cast<int>(i));
    }
    for (const auto& [e, n] : uses)
        if (n != 2) closed[static_cast<std::size_t>(groupOf[find(firstFace[e])])] = false;
    // A face with no edges (a whole sphere or torus) closes on itself.
    for (std::size_t g = 0; g < groups.size(); ++g)
        if (groups[g].size() == 1) {
            const BFace& f = *faces[static_cast<std::size_t>(groups[g][0])];
            bool any = false;
            for (const BLoop& l : f.loops) any = any || !l.edges.empty();
            if (!any) closed[g] = true;
        }
}

// The B-rep's size: its vertices, curves and surfaces (a torus has one vertex, or none).
double boxDiagonal(const Brep& b) {
    DVec3 lo{1e300, 1e300, 1e300}, hi{-1e300, -1e300, -1e300};
    auto add = [&](const DVec3& p, double r) {
        lo = {std::min(lo.x, p.x - r), std::min(lo.y, p.y - r), std::min(lo.z, p.z - r)};
        hi = {std::max(hi.x, p.x + r), std::max(hi.y, p.y + r), std::max(hi.z, p.z + r)};
    };
    for (const BrepVertex& v : b.vertices) add(v.p, 0);
    for (const BCurve& c : b.curves) {
        if (c.kind == BCurveKind::Circle || c.kind == BCurveKind::Ellipse) add(c.f.o, std::max(c.r1, c.r2));
        for (const DVec3& q : c.bs.cp) add(q, 0);
        for (const DVec3& q : c.pts) add(q, 0);
    }
    for (const BSurface& s : b.surfaces) {
        if (s.kind == BSurfaceKind::Sphere || s.kind == BSurfaceKind::Torus || s.kind == BSurfaceKind::Cylinder) add(s.f.o, s.r1 + (s.kind == BSurfaceKind::Torus ? s.r2 : 0));
        for (const DVec3& q : s.bs.cp) add(q, 0);
    }
    return lo.x <= hi.x ? std::max(dlength(hi - lo), 1e-6) : 1.0;
}

// A loop along one edge and straight back (a sphere's seam, down and up again) bounds
// nothing: the face is its whole surface. (Two seams each way round a torus-like
// surface do bound it, and stay.)
bool cancelsOut(const BLoop& l) {
    if (l.edges.empty()) return false;
    std::map<int, int> net;
    for (const OrientedEdge& oe : l.edges) net[oe.edge] += oe.forward ? 1 : -1;
    if (net.size() != 1) return false;
    return net.begin()->second == 0;
}

// Whether a loop bounds nothing worth writing: every edge it uses, it uses back.
bool boundsNothing(const BLoop& l) {
    if (l.edges.empty()) return false;
    std::map<int, int> net;
    for (const OrientedEdge& oe : l.edges) net[oe.edge] += oe.forward ? 1 : -1;
    for (const auto& [e, n] : net)
        if (n != 0) return false;
    return true;
}

// A face on a surface closed both ways (a torus, an ellipse turned about an axis) whose
// edges are all closed curves round it is the whole surface. Its loops then give way to
// the plain boundary: none for a torus or B-spline, the curve and one circle for a
// surface of revolution (STEP needs those to bound it).
void wholeSurfaceFace(Brep& b, BFace& f) {
    const BSurface& S = b.surfaces[static_cast<std::size_t>(f.surface)];
    const BCurve* gen = S.kind == BSurfaceKind::Revolution && S.curve >= 0 ? &b.curves[static_cast<std::size_t>(S.curve)] : nullptr;
    const bool closedBoth = S.kind == BSurfaceKind::Torus || (S.kind == BSurfaceKind::BSpline && S.bs.closedU() && S.bs.closedV()) ||
                            (gen && (gen->periodic() || (gen->kind == BCurveKind::BSpline && gen->bs.closed())));
    if (!closedBoth || f.loops.empty()) return;
    for (const BLoop& l : f.loops)
        for (const OrientedEdge& oe : l.edges)
            if (b.edges[static_cast<std::size_t>(oe.edge)].v0 != b.edges[static_cast<std::size_t>(oe.edge)].v1) return;
    f.loops.clear();
    if (!gen) return;
    const double t0 = gen->kind == BCurveKind::BSpline ? gen->bs.t0() : 0.0;
    const DVec3 P = gen->eval(t0), o = S.f.o + S.f.z * ddot(P - S.f.o, S.f.z);
    if (!(dlength(P - o) > 0)) return;
    b.vertices.push_back({P});
    const int v = static_cast<int>(b.vertices.size()) - 1;
    BCurve ring;
    ring.kind = BCurveKind::Circle;
    ring.f = Frame::make(o, S.f.z, P - o);
    ring.r1 = dlength(P - o);
    b.curves.push_back(ring);
    b.edges.push_back({v, v, S.curve, true});
    const int seam = static_cast<int>(b.edges.size()) - 1;
    b.edges.push_back({v, v, static_cast<int>(b.curves.size()) - 1, true});
    const int round = static_cast<int>(b.edges.size()) - 1;
    BLoop l;
    l.outer = true;
    l.edges = {{seam, true}, {round, true}, {seam, false}, {round, false}};
    f.loops.push_back(std::move(l));
}

} // namespace

void orientShells(Brep& b) {
    TessellateOptions o;
    o.chord = boxDiagonal(b) * 2e-3;
    o.angle = 0.5;
    TessellateReport rep;
    const MeshData m = tessellate(b, o, &rep);
    std::vector<BFace*> faces;
    for (BShell& s : b.shells)
        for (BFace& f : s.faces) faces.push_back(&f);
    if (rep.faceTriangles.size() != faces.size()) return;
    std::vector<int> triFace;
    for (std::size_t f = 0; f < faces.size(); ++f) triFace.insert(triFace.end(), rep.faceTriangles[f], static_cast<int>(f));
    // Welded by position: faces meet on the same samples along their shared edges.
    std::map<std::tuple<float, float, float>, int> weld;
    std::vector<int> vid(m.vertices.size());
    for (std::size_t i = 0; i < m.vertices.size(); ++i) {
        const Vec3& p = m.vertices[i].position;
        vid[i] = weld.try_emplace({p.x, p.y, p.z}, static_cast<int>(weld.size())).first->second;
    }
    std::map<std::pair<int, int>, std::vector<std::pair<int, bool>>> uses;
    const std::size_t nt = std::min(triFace.size(), m.indices.size() / 3);
    for (std::size_t t = 0; t < nt; ++t)
        for (int e = 0; e < 3; ++e) {
            const int a = vid[m.indices[t * 3 + static_cast<std::size_t>(e)]], c = vid[m.indices[t * 3 + static_cast<std::size_t>((e + 1) % 3)]];
            if (a != c) uses[{std::min(a, c), std::max(a, c)}].push_back({triFace[t], a < c});
        }
    // Votes between neighbouring faces: the same sense (a shared edge run both ways) or not.
    std::map<std::pair<int, int>, int> vote;
    for (const auto& [e, u] : uses)
        if (u.size() == 2 && u[0].first != u[1].first) vote[{std::min(u[0].first, u[1].first), std::max(u[0].first, u[1].first)}] += u[0].second != u[1].second ? 1 : -1;
    std::vector<std::vector<std::pair<int, bool>>> adj(faces.size());
    for (const auto& [fg, v] : vote) {
        if (v == 0) continue;
        adj[static_cast<std::size_t>(fg.first)].push_back({fg.second, v < 0});
        adj[static_cast<std::size_t>(fg.second)].push_back({fg.first, v < 0});
    }
    std::vector<int> flip(faces.size(), -1);
    for (std::size_t s = 0; s < faces.size(); ++s) {
        if (flip[s] >= 0) continue;
        std::vector<int> comp{static_cast<int>(s)}, stack{static_cast<int>(s)};
        flip[s] = 0;
        while (!stack.empty()) {
            const int f = stack.back();
            stack.pop_back();
            for (const auto& [g, differ] : adj[static_cast<std::size_t>(f)]) {
                if (flip[static_cast<std::size_t>(g)] >= 0) continue;
                flip[static_cast<std::size_t>(g)] = flip[static_cast<std::size_t>(f)] ^ (differ ? 1 : 0);
                comp.push_back(g);
                stack.push_back(g);
            }
        }
        // Outward: the enclosed volume positive.
        std::vector<char> in(faces.size(), 0);
        for (int f : comp) in[static_cast<std::size_t>(f)] = 1;
        double vol = 0;
        for (std::size_t t = 0; t < nt; ++t) {
            const int f = triFace[t];
            if (!in[static_cast<std::size_t>(f)]) continue;
            const Vec3 &p0 = m.vertices[m.indices[t * 3]].position, &p1 = m.vertices[m.indices[t * 3 + 1]].position, &p2 = m.vertices[m.indices[t * 3 + 2]].position;
            const double v = ddot(DVec3(p0), dcross(DVec3(p1), DVec3(p2))) / 6.0;
            vol += flip[static_cast<std::size_t>(f)] ? -v : v;
        }
        if (vol < 0)
            for (int f : comp) flip[static_cast<std::size_t>(f)] ^= 1;
    }
    for (std::size_t f = 0; f < faces.size(); ++f)
        if (flip[f] == 1) faces[f]->sameSense = !faces[f]->sameSense;
}

std::vector<StepPart> readIges(const std::string& text, StepReadReport* report, std::string* error) {
    File file = parseFile(text);
    if (!file.error.empty()) {
        if (error) *error = file.error;
        return {};
    }
    Reader rd(file);
    std::vector<StepPart> parts;
    auto nameOf = [&](const Entity& e, const char* fallback, std::size_t n) {
        if (!e.label.empty()) return e.label + (e.subscript ? " " + std::to_string(e.subscript) : std::string());
        return std::string(fallback) + " " + std::to_string(n);
    };

    // Solids: manifold solid B-reps (186).
    std::size_t solids = 0;
    for (std::size_t i = 0; i < file.e.size(); ++i) {
        const Entity& ms = file.e[i];
        if (ms.type != 186) continue;
        const Affine T = rd.own(static_cast<int>(i));
        auto b = std::make_shared<Brep>();
        std::map<std::pair<int, int>, int> vmap;
        std::map<std::pair<int, int>, std::vector<OrientedEdge>> emap;
        auto vertex = [&](int list, int idx) {
            const auto [it, added] = vmap.try_emplace({list, idx}, static_cast<int>(b->vertices.size()));
            if (added) {
                const Entity* l = rd.entity(list);
                const std::size_t k = static_cast<std::size_t>(2 + (idx - 1) * 3);
                b->vertices.push_back({l ? rd.placed(list, T).point({Reader::num(*l, k), Reader::num(*l, k + 1), Reader::num(*l, k + 2)}) : DVec3{}});
            }
            return it->second;
        };
        auto edgeChain = [&](int list, int idx) -> const std::vector<OrientedEdge>& {
            const auto [it, added] = emap.try_emplace({list, idx});
            if (!added) return it->second;
            const Entity* l = rd.entity(list);
            if (!l) return it->second;
            const std::size_t k = static_cast<std::size_t>(2 + (idx - 1) * 5);
            const int vs = vertex(Reader::ptr(*l, k + 1), static_cast<int>(Reader::num(*l, k + 2)));
            const int vt = vertex(Reader::ptr(*l, k + 3), static_cast<int>(Reader::num(*l, k + 4)));
            std::vector<Piece> ps;
            if (!rd.pieces(Reader::ptr(*l, k), rd.placed(list, T), ps) || ps.empty()) return it->second;
            const DVec3 ps0 = b->vertices[static_cast<std::size_t>(vs)].p, pt0 = b->vertices[static_cast<std::size_t>(vt)].p;
            // The pieces run from the start vertex to the end one, or the other way.
            const bool along = dlength(ps.front().a - ps0) <= dlength(ps.front().a - pt0) || vs == vt;
            if (!along) std::reverse(ps.begin(), ps.end());
            int from = vs;
            for (std::size_t q = 0; q < ps.size(); ++q) {
                const Piece& pc = ps[q];
                const int to = q + 1 == ps.size() ? vt : static_cast<int>((b->vertices.push_back({along ? pc.b : pc.a}), b->vertices.size() - 1));
                b->edges.push_back({from, to, Reader::addCurve(*b, pc), along || vs == vt});
                it->second.push_back({static_cast<int>(b->edges.size()) - 1, true});
                from = to;
            }
            return it->second;
        };
        auto shell = [&](int si, bool isVoid) {
            const Entity* s = rd.entity(si);
            if (!s || s->type != 514) return;
            BShell sh;
            sh.closed = s->form != 2;
            sh.name = isVoid ? "void" : "outer";
            const int n = static_cast<int>(Reader::num(*s, 1));
            for (int j = 0; j < n; ++j) {
                const int fi = Reader::ptr(*s, static_cast<std::size_t>(2 + j * 2));
                const bool of = Reader::num(*s, static_cast<std::size_t>(3 + j * 2)) != 0;
                const Entity* fe = rd.entity(fi);
                if (!fe || fe->type != 510) continue;
                BFace f;
                f.surface = rd.surface(Reader::ptr(*fe, 1), rd.placed(fi, T), *b);
                if (f.surface < 0) continue;
                f.sameSense = of;
                const int nl = static_cast<int>(Reader::num(*fe, 2));
                const bool outerFirst = Reader::num(*fe, 3) != 0;
                for (int q = 0; q < nl; ++q) {
                    const int li = Reader::ptr(*fe, static_cast<std::size_t>(4 + q));
                    const Entity* le = rd.entity(li);
                    if (!le || le->type != 508) continue;
                    BLoop loop;
                    loop.outer = q == 0 && outerFirst;
                    const int ne = static_cast<int>(Reader::num(*le, 1));
                    std::size_t k = 2;
                    int poleVertex = -1;
                    for (int m = 0; m < ne; ++m) {
                        const int type = static_cast<int>(Reader::num(*le, k));
                        const int list = Reader::ptr(*le, k + 1), idx = static_cast<int>(Reader::num(*le, k + 2));
                        const bool fwd = Reader::num(*le, k + 3) != 0;
                        const int npc = static_cast<int>(Reader::num(*le, k + 4));
                        k += 5 + static_cast<std::size_t>(std::max(0, npc)) * 2;
                        if (type == 1) {
                            poleVertex = vertex(list, idx);
                            continue;
                        }
                        std::vector<OrientedEdge> chain = edgeChain(list, idx);
                        if (!fwd) {
                            std::reverse(chain.begin(), chain.end());
                            for (OrientedEdge& oe : chain) oe.forward = !oe.forward;
                        }
                        loop.edges.insert(loop.edges.end(), chain.begin(), chain.end());
                    }
                    if (cancelsOut(loop)) continue; // a seam and poles: the whole surface
                    if (loop.edges.empty() && poleVertex >= 0) loop.vertex = poleVertex;
                    if (!loop.edges.empty() || loop.vertex >= 0) f.loops.push_back(std::move(loop));
                }
                wholeSurfaceFace(*b, f);
                sh.faces.push_back(std::move(f));
            }
            if (!sh.faces.empty()) b->shells.push_back(std::move(sh));
        };
        shell(Reader::ptr(ms, 1), false);
        const int voids = static_cast<int>(Reader::num(ms, 3));
        for (int v = 0; v < voids; ++v) shell(Reader::ptr(ms, static_cast<std::size_t>(4 + v * 2)), true);
        if (b->shells.empty()) continue;
        orientShells(*b);
        for (BShell& sh : b->shells)
            if (sh.name == "void")
                for (BFace& f : sh.faces) f.sameSense = !f.sameSense; // voids face into the hollow
        orientLoops(*b);
        b->transform([&] {
            Placement p;
            p.s = file.unitToMm;
            return p;
        }());
        StepPart part;
        part.name = nameOf(ms, "Solid", ++solids);
        b->name = part.name;
        part.brep = std::move(b);
        parts.push_back(std::move(part));
    }

    // Trimmed surfaces (144) and bare surfaces, sewn into shells.
    {
        struct FaceSpec {
            int surface = -1;
            std::vector<std::vector<Piece>> loops; // the outer first
            bool outerGiven = false;
        };
        auto all = std::make_shared<Brep>();
        std::vector<FaceSpec> specs;
        std::set<int> trimmedSurfaces;
        for (const Entity& e : file.e)
            if (e.type == 144) trimmedSurfaces.insert(Reader::ptr(e, 1));
        DVec3 lo{1e300, 1e300, 1e300}, hi{-1e300, -1e300, -1e300};
        auto extend = [&](const DVec3& p) {
            lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
            hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
        };
        for (std::size_t i = 0; i < file.e.size(); ++i) {
            const Entity& e = file.e[i];
            if (e.type == 144) {
                const Affine T = rd.own(static_cast<int>(i));
                FaceSpec fs;
                fs.surface = rd.surface(Reader::ptr(e, 1), T, *all);
                if (fs.surface < 0) continue;
                const bool n1 = Reader::num(e, 2) != 0;
                const int n2 = static_cast<int>(Reader::num(e, 3));
                std::vector<int> bounds;
                if (n1) bounds.push_back(Reader::ptr(e, 4));
                for (int k = 0; k < n2; ++k) bounds.push_back(Reader::ptr(e, static_cast<std::size_t>(5 + k)));
                fs.outerGiven = n1;
                bool ok = true;
                for (int bi : bounds) {
                    const Entity* c = rd.entity(bi);
                    if (!c || c->type != 142) {
                        ok = false;
                        break;
                    }
                    std::vector<Piece> ps;
                    const int model = Reader::ptr(*c, 4);
                    if (model < 0 || !rd.pieces(model, rd.placed(bi, T), ps) || ps.empty()) {
                        rd.warn("Trimmed surfaces bounded only in their parameters aren't read");
                        ok = false;
                        break;
                    }
                    for (const Piece& p : ps) extend(p.a), extend(p.b);
                    fs.loops.push_back(std::move(ps));
                }
                if (ok) specs.push_back(std::move(fs));
            } else if ((e.type == 128 || e.type == 108 || (e.type >= 190 && e.type <= 198 && e.type % 2 == 0) || e.type == 120 || e.type == 122) &&
                       e.subordinate() == 0 && !trimmedSurfaces.count(static_cast<int>(i)) && (e.type != 108 || e.form == 0)) {
                // A surface on its own: its whole domain (bounded ones only).
                FaceSpec fs;
                fs.surface = rd.surface(static_cast<int>(i), Affine{}, *all);
                if (fs.surface >= 0 && e.type == 128) specs.push_back(std::move(fs));
            }
        }
        if (!specs.empty()) {
            const double diag = lo.x <= hi.x ? dlength(hi - lo) : 1.0;
            double tol = file.resolution > 0 ? file.resolution : 1e-5 * diag;
            tol = std::clamp(tol, 1e-7 * diag, 1e-3 * diag);
            Sewer sew(*all, tol);
            BShell sh;
            for (const FaceSpec& fs : specs) {
                BFace f;
                f.surface = fs.surface;
                for (std::size_t li = 0; li < fs.loops.size(); ++li) {
                    BLoop l;
                    l.outer = li == 0 && fs.outerGiven;
                    for (const Piece& p : fs.loops[li]) {
                        OrientedEdge oe;
                        if (sew.edge(p, oe)) l.edges.push_back(oe);
                    }
                    if (!l.edges.empty() && !cancelsOut(l)) f.loops.push_back(std::move(l));
                }
                wholeSurfaceFace(*all, f);
                sh.faces.push_back(std::move(f));
            }
            all->shells.push_back(std::move(sh));
            std::vector<std::vector<int>> groups;
            std::vector<bool> closed;
            components(*all, groups, closed);
            // Each closed group a solid; the open ones together, a surface model.
            std::vector<std::vector<int>> openGroups;
            std::vector<bool> openFlags;
            std::size_t bodies = 0;
            for (std::size_t g = 0; g < groups.size(); ++g) {
                if (!closed[g]) {
                    openGroups.push_back(groups[g]);
                    openFlags.push_back(false);
                    continue;
                }
                auto b = extract(*all, {groups[g]}, {true});
                orientShells(*b);
                orientLoops(*b);
                Placement p;
                p.s = file.unitToMm;
                b->transform(p);
                StepPart part;
                part.name = "Body " + std::to_string(++bodies);
                b->name = part.name;
                part.brep = std::move(b);
                parts.push_back(std::move(part));
            }
            if (!openGroups.empty()) {
                auto b = extract(*all, openGroups, openFlags);
                orientShells(*b);
                orientLoops(*b);
                Placement p;
                p.s = file.unitToMm;
                b->transform(p);
                StepPart part;
                part.name = "Surfaces";
                b->name = part.name;
                part.brep = std::move(b);
                parts.push_back(std::move(part));
            }
        }
    }
    if (report) {
        report->entities = file.e.size();
        report->unitToMm = file.unitToMm;
        report->schema = "IGES";
        report->warnings = file.warnings;
    }
    if (parts.empty() && error) *error = file.warnings.empty() ? "No solids or surfaces in the file" : "No solids or surfaces read: " + file.warnings.front();
    return parts;
}

// --- writing ------------------------------------------------------------------------------

namespace {

std::string real(double v) {
    if (!std::isfinite(v)) v = 0;
    if (std::fabs(v) < 1e-300) v = 0;
    char b[40];
    std::snprintf(b, sizeof b, "%.15G", v);
    std::string s = b;
    const std::size_t e = s.find('E');
    if (s.find('.') == std::string::npos) s.insert(e == std::string::npos ? s.size() : e, ".");
    return s;
}

std::string hollerith(const std::string& t) { return std::to_string(t.size()) + "H" + t; }

class IgesOut {
public:
    struct Ent {
        int type = 0, form = 0, transform = 0;
        bool dependent = true;
        std::vector<std::string> params;
    };
    // Adds an entity; returns its directory pointer.
    int add(int type, std::vector<std::string> params, int form = 0, int transform = 0, bool dependent = true) {
        Ent e;
        e.type = type, e.form = form, e.transform = transform, e.dependent = dependent;
        e.params = std::move(params);
        m_e.push_back(std::move(e));
        return static_cast<int>(m_e.size()) * 2 - 1;
    }
    // x' = R x + t with R's columns x, y, z.
    int matrix(const DVec3& o, const DVec3& x, const DVec3& y, const DVec3& z) {
        return add(124, {real(x.x), real(y.x), real(z.x), real(o.x), real(x.y), real(y.y), real(z.y), real(o.y), real(x.z), real(y.z), real(z.z), real(o.z)});
    }

    std::string text(const std::string& fileName, double maxCoord) const {
        std::string out;
        auto pad = [](std::string l, char sec, int seq) {
            l.resize(72, ' ');
            char b[16];
            std::snprintf(b, sizeof b, "%c%07d", sec, seq);
            return l + b + "\n";
        };
        out += pad("SI-CDM IGES export: trimmed surfaces in millimetres", 'S', 1);
        // Global section.
        char date[32];
        {
            const std::time_t t = std::time(nullptr);
            std::tm tm{};
#if defined(_WIN32)
            gmtime_s(&tm, &t);
#else
            gmtime_r(&t, &tm);
#endif
            std::strftime(date, sizeof date, "%Y%m%d.%H%M%S", &tm);
        }
        const std::vector<std::string> g = {"1H,", "1H;", hollerith("SI-CDM"), hollerith(fileName), hollerith("SI-CDM"), hollerith("SI-CDM IGES 5.3"), "32", "38", "6", "308", "15",
                                            hollerith("SI-CDM"), "1.", "2", "2HMM", "1", "1.", hollerith(date), "1.E-06", real(std::max(maxCoord, 1.0)), "", "", "11", "0",
                                            hollerith(date)};
        std::string gs;
        for (std::size_t i = 0; i < g.size(); ++i) gs += g[i] + (i + 1 < g.size() ? "," : ";");
        int gl = 0;
        for (std::size_t i = 0; i < gs.size(); i += 72) out += pad(gs.substr(i, 72), 'G', ++gl);
        // Parameters, then the directory pointing at them.
        std::vector<std::string> plines;
        std::vector<int> pstart(m_e.size()), pcount(m_e.size());
        for (std::size_t i = 0; i < m_e.size(); ++i) {
            const Ent& e = m_e[i];
            pstart[i] = static_cast<int>(plines.size()) + 1;
            std::string line;
            std::vector<std::string> toks{std::to_string(e.type)};
            toks.insert(toks.end(), e.params.begin(), e.params.end());
            for (std::size_t k = 0; k < toks.size(); ++k) {
                const std::string t = toks[k] + (k + 1 < toks.size() ? "," : ";");
                if (!line.empty() && line.size() + t.size() > 64) {
                    plines.push_back(line);
                    line.clear();
                }
                line += t;
            }
            plines.push_back(line);
            pcount[i] = static_cast<int>(plines.size()) + 1 - pstart[i];
        }
        int pl = 0;
        std::string pblock;
        for (std::size_t i = 0; i < m_e.size(); ++i)
            for (int k = 0; k < pcount[i]; ++k) {
                std::string l = plines[static_cast<std::size_t>(pstart[i] - 1 + k)];
                l.resize(64, ' ');
                char b[16];
                std::snprintf(b, sizeof b, " %7d", static_cast<int>(i) * 2 + 1);
                pblock += pad(l + b, 'P', ++pl);
            }
        int dl = 0;
        for (std::size_t i = 0; i < m_e.size(); ++i) {
            const Ent& e = m_e[i];
            char b1[80], b2[80];
            std::snprintf(b1, sizeof b1, "%8d%8d%8d%8d%8d%8d%8d%8d%08d", e.type, pstart[i], 0, 0, 0, 0, e.transform, 0, e.dependent ? 10000 : 0);
            std::snprintf(b2, sizeof b2, "%8d%8d%8d%8d%8d%8s%8s%8s%8d", e.type, 0, 0, pcount[i], e.form, "", "", "", 0);
            out += pad(b1, 'D', ++dl);
            out += pad(b2, 'D', ++dl);
        }
        out += pblock;
        char t[80];
        std::snprintf(t, sizeof t, "S%07dG%07dD%07dP%07d", 1, gl, dl, pl);
        out += pad(t, 'T', 1);
        return out;
    }

private:
    std::vector<Ent> m_e;
};

// The curve parameters an edge runs between (as the tessellator samples it).
void edgeRange(const Brep& b, const BrepEdge& E, double& ta, double& tb) {
    const BCurve& c = b.curves[static_cast<std::size_t>(E.curve)];
    const DVec3 p0 = b.vertices[static_cast<std::size_t>(E.v0)].p, p1 = b.vertices[static_cast<std::size_t>(E.v1)].p;
    const bool closed = E.v0 == E.v1;
    ta = c.param(p0), tb = c.param(p1);
    if (c.kind == BCurveKind::Polyline && closed) ta = 0, tb = static_cast<double>(c.pts.size() - 1);
    if (c.periodic()) {
        if (E.sameSense) {
            while (tb <= ta + 1e-12) tb += 2 * kPiI;
            if (closed) tb = ta + 2 * kPiI;
        } else {
            while (tb >= ta - 1e-12) tb -= 2 * kPiI;
            if (closed) tb = ta - 2 * kPiI;
        }
    } else if (c.kind == BCurveKind::BSpline && closed) {
        ta = E.sameSense ? c.bs.t0() : c.bs.t1();
        tb = E.sameSense ? c.bs.t1() : c.bs.t0();
    }
}

// Points along an oriented edge, from its start to its end.
std::vector<DVec3> edgePoints(const Brep& b, const OrientedEdge& oe, int n) {
    const BrepEdge& E = b.edges[static_cast<std::size_t>(oe.edge)];
    std::vector<DVec3> pts;
    const DVec3 p0 = b.vertices[static_cast<std::size_t>(E.v0)].p, p1 = b.vertices[static_cast<std::size_t>(E.v1)].p;
    if (E.curve < 0 || b.curves[static_cast<std::size_t>(E.curve)].kind == BCurveKind::Line) pts = {p0, p1};
    else {
        double ta = 0, tb = 0;
        edgeRange(b, E, ta, tb);
        const BCurve& c = b.curves[static_cast<std::size_t>(E.curve)];
        for (int i = 0; i <= n; ++i) {
            const double t = ta + (tb - ta) * i / n;
            if (c.kind == BCurveKind::Polyline) {
                const double x = std::clamp(t, 0.0, static_cast<double>(c.pts.size() - 1));
                const std::size_t k = std::min(static_cast<std::size_t>(x), c.pts.size() - 2);
                pts.push_back(c.pts[k] + (c.pts[k + 1] - c.pts[k]) * (x - static_cast<double>(k)));
            } else pts.push_back(c.eval(t));
        }
        pts.front() = p0, pts.back() = p1;
    }
    if (!oe.forward) std::reverse(pts.begin(), pts.end());
    return pts;
}

// One oriented edge as an IGES curve running the way the loop does.
int writeEdge(IgesOut& w, const Brep& b, const OrientedEdge& oe, bool split = true) {
    const BrepEdge& E = b.edges[static_cast<std::size_t>(oe.edge)];
    DVec3 s = b.vertices[static_cast<std::size_t>(E.v0)].p, e = b.vertices[static_cast<std::size_t>(E.v1)].p;
    if (!oe.forward) std::swap(s, e);
    auto line = [&](const DVec3& a, const DVec3& c) { return w.add(110, {real(a.x), real(a.y), real(a.z), real(c.x), real(c.y), real(c.z)}); };
    if (E.curve < 0) return line(s, e);
    const BCurve& c = b.curves[static_cast<std::size_t>(E.curve)];
    if (c.kind == BCurveKind::Line) return line(s, e);
    double ta = 0, tb = 0;
    edgeRange(b, E, ta, tb);
    if (!oe.forward) std::swap(ta, tb);
    const bool up = tb > ta; // the curve's own direction along the loop
    if (c.kind == BCurveKind::Circle || c.kind == BCurveKind::Ellipse) {
        // In the curve's plane, turned over when the loop runs against it: counter-clockwise.
        const double sg = up ? 1.0 : -1.0;
        const int m = w.matrix(c.f.o, c.f.x, c.f.y * sg, c.f.z * sg);
        const double a0 = ta * sg, a1 = tb * sg;
        const bool full = std::fabs(tb - ta) >= 2 * kPiI - 1e-9;
        if (c.kind == BCurveKind::Circle) {
            const double r = c.r1;
            const double ex = full ? r * std::cos(a0) : r * std::cos(a1), ey = full ? r * std::sin(a0) : r * std::sin(a1);
            return w.add(100, {"0.", "0.", "0.", real(r * std::cos(a0)), real(r * std::sin(a0)), real(ex), real(ey)}, 0, m);
        }
        // An ellipse goes as one arc, or a closed one as two halves (some readers start a
        // closed conic at its own axis, not at the point given).
        const double ra = c.r1, rb = c.r2;
        auto arc = [&](double from, double to) {
            return w.add(104, {real(1 / (ra * ra)), "0.", real(1 / (rb * rb)), "0.", "0.", "-1.", "0.", real(ra * std::cos(from)), real(rb * std::sin(from)), real(ra * std::cos(to)),
                               real(rb * std::sin(to))},
                         1, m);
        };
        if (!full || !split) return arc(a0, full ? a0 : a1);
        const int h1 = arc(a0, a0 + kPiI), h2 = arc(a0 + kPiI, a0);
        return w.add(102, {"2", std::to_string(h1), std::to_string(h2)});
    }
    if (c.kind == BCurveKind::BSpline) {
        const BSplineCurve& bs = c.bs;
        std::vector<DVec3> cp = bs.cp;
        std::vector<double> wts = bs.w, kn = bs.knots;
        double v0 = ta, v1 = tb;
        if (!up) { // reversed: the same curve run backwards
            std::reverse(cp.begin(), cp.end());
            std::reverse(wts.begin(), wts.end());
            const double sum = kn.front() + kn.back();
            std::reverse(kn.begin(), kn.end());
            for (double& k : kn) k = sum - k;
            v0 = sum - ta, v1 = sum - tb;
        }
        const int K = static_cast<int>(cp.size()) - 1;
        std::vector<std::string> p = {std::to_string(K), std::to_string(bs.degree), "0", c.bs.closed() ? "1" : "0", wts.empty() ? "1" : "0", "0"};
        for (double k : kn) p.push_back(real(k));
        for (std::size_t i = 0; i < cp.size(); ++i) p.push_back(wts.empty() ? "1." : real(wts[i]));
        for (const DVec3& q : cp) p.push_back(real(q.x)), p.push_back(real(q.y)), p.push_back(real(q.z));
        p.push_back(real(v0));
        p.push_back(real(v1));
        p.insert(p.end(), {"0.", "0.", "0."});
        return w.add(126, p);
    }
    // A polyline (or anything else): its points.
    const std::vector<DVec3> pts = edgePoints(b, oe, 32);
    std::vector<std::string> p = {"2", std::to_string(pts.size())};
    for (const DVec3& q : pts) p.push_back(real(q.x)), p.push_back(real(q.y)), p.push_back(real(q.z));
    return w.add(106, p, 12);
}

// A curve swept by a surface, restricted to [t0, t1], as an IGES curve.
int writeSwept(IgesOut& w, const BCurve& c, double t0, double t1, const DVec3& shift, const Placement* turn = nullptr) {
    Brep tmp;
    tmp.curves.push_back(c);
    if (turn) tmp.transform(*turn);
    BCurve& cc = tmp.curves[0];
    cc.f.o = cc.f.o + shift;
    for (DVec3& q : cc.bs.cp) q = q + shift;
    for (DVec3& q : cc.pts) q = q + shift;
    tmp.vertices.push_back({cc.eval(t0)});
    tmp.vertices.push_back({cc.eval(t1)});
    if (cc.kind == BCurveKind::Line) return w.add(110, {real(tmp.vertices[0].p.x), real(tmp.vertices[0].p.y), real(tmp.vertices[0].p.z), real(tmp.vertices[1].p.x),
                                                       real(tmp.vertices[1].p.y), real(tmp.vertices[1].p.z)});
    tmp.edges.push_back({0, 1, 0, true});
    if (std::fabs(t1 - t0) >= 2 * kPiI - 1e-9 && cc.periodic()) tmp.edges[0].v1 = 0;
    return writeEdge(w, tmp, {0, true}, false); // one curve for the surface to sweep
}

// A face's surface, covering the face; 0 when it can't be written. `flip`: whether the
// face's loops must be written backwards, so they run counter-clockwise about the
// written surface's own parameter normal (IGES readers trim to the left of them).
int writeSurface(IgesOut& w, const Brep& b, const BFace& f, bool& flip) {
    const BSurface& S = b.surfaces[static_cast<std::size_t>(f.surface)];
    // The boundary, edge by edge (runs of points along each).
    std::vector<std::vector<DVec3>> runs;
    std::vector<DVec3> pts;
    for (const BLoop& l : f.loops)
        for (const OrientedEdge& oe : l.edges) {
            runs.push_back(edgePoints(b, oe, 48));
            pts.insert(pts.end(), runs.back().begin(), runs.back().end());
        }
    const DVec3 o = S.f.o, z = S.f.z;
    // Where the face's own seams are (edges it uses twice): the surface written has its
    // seam there too, or readers see a slit. A seam along a meridian sets where the
    // generatrix lies; one round the axis (a partial torus's tube) where its circle starts.
    auto foot = [&](const DVec3& q) { return o + z * ddot(q - o, z); };
    auto around = [&](const DVec3& q) { return std::atan2(ddot(q - o, S.f.y), ddot(q - o, S.f.x)); };
    auto sameAngle = [](double x, double y) { return std::fabs(std::remainder(x - y, 2 * kPiI)) < 1e-7; };
    DVec3 ref = S.f.x;
    bool meridianSeam = false;
    DVec3 roundSeam{};
    bool hasRoundSeam = false;
    {
        std::map<int, int> uses;
        for (const BLoop& l : f.loops)
            for (const OrientedEdge& oe : l.edges) ++uses[oe.edge];
        for (const auto& [e, n] : uses) {
            if (n < 2) continue;
            const std::vector<DVec3> q = edgePoints(b, {e, true}, 2); // a straight edge gives its two ends only
            const DVec3 first = q.front(), last = q.back(), mid = q.size() > 2 ? q[1] : (first + last) * 0.5;
            if (dlength(mid - foot(mid)) < 1e-9) continue;
            if (sameAngle(around(first), around(mid)) && sameAngle(around(last), around(mid))) {
                ref = dnormalize(mid - foot(mid));
                meridianSeam = true;
            } else {
                roundSeam = mid;
                hasRoundSeam = true;
            }
        }
    }
    (void)meridianSeam;
    // The face's outward normal against the written surface's: sampled round the boundary.
    const double outward = f.sameSense ? 1.0 : -1.0;
    auto decide = [&](const std::function<DVec3(const DVec3&)>& written) {
        double sum = 0;
        for (const std::vector<DVec3>& r : runs) {
            if (r.size() < 2) continue;
            const DVec3 q = r[r.size() / 2];
            const DVec3 mine = dnormalize(surfaceNormalNear(b, S, q)) * outward;
            sum += ddot(dnormalize(written(q)), mine);
        }
        flip = sum < 0;
    };
    auto radialAt = [&](const DVec3& q) {
        const DVec3 r = q - foot(q);
        return dlength(r) > 0 ? dnormalize(r) : ref;
    };
    // The range of fn over the boundary, a little wider (2%) so the boundary lies well
    // inside the surface written; a periodic fn is followed along each edge.
    auto range = [&](const std::function<double(const DVec3&)>& fn, double& lo, double& hi, double period = 0) {
        lo = 1e300, hi = -1e300;
        for (const std::vector<DVec3>& r : runs) {
            double prev = 0;
            for (std::size_t i = 0; i < r.size(); ++i) {
                double x = fn(r[i]);
                if (period > 0 && i > 0) x += period * std::round((prev - x) / period);
                prev = x;
                lo = std::min(lo, x), hi = std::max(hi, x);
            }
        }
        if (period > 0 && hi - lo > period * 0.98) { // all the way round
            lo = 0, hi = period;
            return true;
        }
        const double m = std::max(1e-6, (hi - lo) * 0.02);
        lo -= m, hi += m;
        if (period > 0 && hi - lo > period) hi = lo + period;
        return lo <= hi;
    };
    auto axisLine = [&] { return w.add(110, {real(o.x), real(o.y), real(o.z), real(o.x + z.x), real(o.y + z.y), real(o.z + z.z)}); };
    auto revolve = [&](int gen) { return w.add(120, {std::to_string(axisLine()), std::to_string(gen), "0.", real(2 * kPiI)}); };
    switch (S.kind) {
    case BSurfaceKind::Plane: {
        decide([&](const DVec3&) { return z; });
        const double d = ddot(z, o);
        return w.add(108, {real(z.x), real(z.y), real(z.z), real(d), "0", "0.", "0.", "0.", "0."});
    }
    case BSurfaceKind::Cylinder:
    case BSurfaceKind::Cone: {
        double lo = 0, hi = 0;
        if (!range([&](const DVec3& q) { return ddot(q - o, z); }, lo, hi)) return 0;
        const double t = S.kind == BSurfaceKind::Cone ? std::tan(S.r2) : 0.0;
        if (t > 0) lo = std::max(lo, -S.r1 / t); // not past the apex
        const DVec3 a = o + z * lo + ref * (S.r1 + lo * t), c = o + z * hi + ref * (S.r1 + hi * t);
        // IGES turns the generatrix (parameter t) by the angle: its normal is dC/dt x dS/dangle.
        decide([&](const DVec3& q) {
            const DVec3 rh = radialAt(q), dt = z * ddot(c - a, z) + rh * ddot(c - a, ref);
            return dcross(dt, dcross(z, rh));
        });
        const int gen = w.add(110, {real(a.x), real(a.y), real(a.z), real(c.x), real(c.y), real(c.z)});
        return revolve(gen);
    }
    case BSurfaceKind::Sphere: { // a half circle from pole to pole, in the x-z plane
        decide([&](const DVec3& q) {
            const DVec3 rh = radialAt(q);
            const double ph = std::atan2(ddot(q - o, z), ddot(q - o, rh));
            return dcross(rh * -std::sin(ph) + z * std::cos(ph), dcross(z, rh));
        });
        const int m = w.matrix(o, ref, z, dcross(ref, z));
        const int gen = w.add(100, {"0.", "0.", "0.", "0.", real(-S.r1), "0.", real(S.r1)}, 0, m);
        return revolve(gen);
    }
    case BSurfaceKind::Torus: { // the tube's whole circle
        DVec3 rr = ref;
        double phi = 0; // where on the tube the circle starts
        if (hasRoundSeam) {
            rr = dnormalize(roundSeam - foot(roundSeam));
            const DVec3 cc0 = o + rr * S.r1; // the tube's centre in the seam's meridian
            phi = std::atan2(ddot(roundSeam - cc0, z), ddot(roundSeam - cc0, rr));
            if (!meridianSeam) rr = ref;
        }
        decide([&](const DVec3& q) {
            const DVec3 rh = radialAt(q), tc = o + rh * S.r1;
            const double ph = std::atan2(ddot(q - tc, z), ddot(q - tc, rh));
            return dcross(rh * -std::sin(ph) + z * std::cos(ph), dcross(z, rh));
        });
        const DVec3 cc = o + rr * S.r1;
        const int m = w.matrix(cc, rr, z, dcross(rr, z));
        const std::string px = real(S.r2 * std::cos(phi)), py = real(S.r2 * std::sin(phi));
        const int gen = w.add(100, {"0.", "0.", "0.", px, py, px, py}, 0, m);
        return revolve(gen);
    }
    case BSurfaceKind::BSpline: {
        decide([&](const DVec3& q) { return surfaceNormalNear(b, S, q); }); // the same parameters
        const BSplineSurface& bs = S.bs;
        std::vector<std::string> p = {std::to_string(bs.nu - 1), std::to_string(bs.nv - 1), std::to_string(bs.du), std::to_string(bs.dv), bs.closedU() ? "1" : "0",
                                      bs.closedV() ? "1" : "0", bs.w.empty() ? "1" : "0", "0", "0"};
        for (double k : bs.ku) p.push_back(real(k));
        for (double k : bs.kv) p.push_back(real(k));
        for (int j = 0; j < bs.nv; ++j)
            for (int i = 0; i < bs.nu; ++i) p.push_back(bs.w.empty() ? "1." : real(bs.w[static_cast<std::size_t>(i * bs.nv + j)]));
        for (int j = 0; j < bs.nv; ++j)
            for (int i = 0; i < bs.nu; ++i) {
                const DVec3& q = bs.cp[static_cast<std::size_t>(i * bs.nv + j)];
                p.push_back(real(q.x)), p.push_back(real(q.y)), p.push_back(real(q.z));
            }
        p.insert(p.end(), {real(bs.u0()), real(bs.u1()), real(bs.v0()), real(bs.v1())});
        return w.add(128, p);
    }
    case BSurfaceKind::Revolution: {
        if (S.curve < 0) return 0;
        decide([&](const DVec3& q) { return surfaceNormalNear(b, S, q) * -1.0; }); // (curve, angle), not (angle, curve)
        const BCurve& c = b.curves[static_cast<std::size_t>(S.curve)];
        // The curve's parameter at each point turned back onto the curve's own meridian.
        const DVec3 c0 = c.eval(c.kind == BCurveKind::BSpline ? 0.5 * (c.bs.t0() + c.bs.t1()) : 0.0);
        const DVec3 ref0 = dnormalize((c0 - o) - z * ddot(c0 - o, z)); // the curve's own meridian
        double lo = 0, hi = 0;
        if (pts.empty()) {
            if (c.kind == BCurveKind::BSpline) lo = c.bs.t0(), hi = c.bs.t1();
            else if (c.periodic()) lo = 0, hi = 2 * kPiI;
            else return 0;
        } else if (!range(
                       [&](const DVec3& q) {
                           const double h = ddot(q - o, z);
                           const double r = dlength((q - o) - z * h);
                           return c.param(o + z * h + ref0 * r);
                       },
                       lo, hi, c.periodic() ? 2 * kPiI : 0.0))
            return 0;
        if (c.kind == BCurveKind::BSpline) lo = std::max(lo, c.bs.t0()), hi = std::min(hi, c.bs.t1());
        if (meridianSeam) { // the curve turned about the axis onto the seam
            const double th = std::atan2(ddot(dcross(ref0, ref), z), ddot(ref0, ref));
            Placement turn;
            const double cs = std::cos(th), sn = std::sin(th);
            // Rotation about z by th: R = I cs + (1 - cs) z z^T + sn [z]x.
            const double zx = z.x, zy = z.y, zz = z.z;
            const double R[9] = {cs + (1 - cs) * zx * zx, (1 - cs) * zx * zy - sn * zz, (1 - cs) * zx * zz + sn * zy,
                                 (1 - cs) * zy * zx + sn * zz, cs + (1 - cs) * zy * zy, (1 - cs) * zy * zz - sn * zx,
                                 (1 - cs) * zz * zx - sn * zy, (1 - cs) * zz * zy + sn * zx, cs + (1 - cs) * zz * zz};
            for (int k = 0; k < 9; ++k) turn.r[k] = R[k];
            turn.t = o - turn.dir(o);
            return revolve(writeSwept(w, c, lo, hi, {0, 0, 0}, &turn));
        }
        return revolve(writeSwept(w, c, lo, hi, {0, 0, 0}));
    }
    case BSurfaceKind::Extrusion: {
        if (S.curve < 0 || pts.empty()) return 0;
        decide([&](const DVec3& q) { return surfaceNormalNear(b, S, q); }); // (curve, along) as here
        const BCurve& c = b.curves[static_cast<std::size_t>(S.curve)];
        // Points as curve + dir v: v along the curve plane's normal.
        DVec3 n_ = c.f.z;
        if (c.kind == BCurveKind::BSpline || c.kind == BCurveKind::Polyline) {
            const std::vector<DVec3>& q = c.kind == BCurveKind::BSpline ? c.bs.cp : c.pts;
            n_ = {0, 0, 0};
            for (std::size_t i = 0; i < q.size(); ++i) n_ += dcross(q[i], q[(i + 1) % q.size()]);
            n_ = dlength(n_) > 0 ? dnormalize(n_) : dnormalize(S.dir);
        }
        const double dn = ddot(S.dir, n_);
        const DVec3 c0 = c.eval(c.kind == BCurveKind::BSpline ? c.bs.t0() : 0.0);
        if (std::fabs(dn) < 1e-9 * dlength(S.dir)) return 0;
        double vlo = 0, vhi = 0, ulo = 0, uhi = 0;
        range([&](const DVec3& q) { return ddot(q - c0, n_) / dn; }, vlo, vhi);
        range([&](const DVec3& q) { return c.param(q - S.dir * (ddot(q - c0, n_) / dn)); }, ulo, uhi);
        if (c.periodic()) { // round from the seam (where the face's seam line runs), else from 0
            ulo = 0;
            for (const BLoop& l : f.loops)
                for (const OrientedEdge& oe : l.edges) {
                    int n = 0;
                    for (const BLoop& l2 : f.loops)
                        for (const OrientedEdge& o2 : l2.edges) n += o2.edge == oe.edge;
                    if (n >= 2) {
                        const DVec3 q = edgePoints(b, oe, 1)[0];
                        ulo = c.param(q - S.dir * (ddot(q - c0, n_) / dn));
                    }
                }
            uhi = ulo + 2 * kPiI;
        }
        if (c.kind == BCurveKind::BSpline) ulo = std::max(ulo, c.bs.t0()), uhi = std::min(uhi, c.bs.t1());
        const int dir = writeSwept(w, c, ulo, uhi, S.dir * vlo);
        const DVec3 end = c.eval(ulo) + S.dir * vhi;
        return w.add(122, {std::to_string(dir), real(end.x), real(end.y), real(end.z)});
    }
    }
    return 0;
}

} // namespace

std::string writeIges(const std::vector<StepPart>& parts, const std::string& fileName) {
    IgesOut w;
    double maxCoord = 1;
    for (const StepPart& part : parts) {
        if (!part.brep) continue;
        const Brep& b = *part.brep;
        for (const BrepVertex& v : b.vertices) maxCoord = std::max({maxCoord, std::fabs(v.p.x), std::fabs(v.p.y), std::fabs(v.p.z)});
        for (const BShell& sh : b.shells)
            for (const BFace& f : sh.faces) {
                if (f.surface < 0) continue;
                bool flip = false;
                const int surf = writeSurface(w, b, f, flip);
                if (!surf) continue;
                // Faceted B-reps: a polygon loop as lines.
                std::vector<std::vector<int>> loopCurves;
                int outer = -1;
                double best = -1;
                for (const BLoop& l : f.loops) {
                    std::vector<int> cs;
                    DVec3 lo{1e300, 1e300, 1e300}, hi{-1e300, -1e300, -1e300};
                    auto grow = [&](const DVec3& q) {
                        lo = {std::min(lo.x, q.x), std::min(lo.y, q.y), std::min(lo.z, q.z)};
                        hi = {std::max(hi.x, q.x), std::max(hi.y, q.y), std::max(hi.z, q.z)};
                    };
                    const bool backwards = !l.sense != flip;
                    if (!l.edges.empty()) {
                        if (boundsNothing(l)) continue; // the whole surface: no boundary written
                        std::vector<OrientedEdge> es = l.edges;
                        if (backwards) {
                            std::reverse(es.begin(), es.end());
                            for (OrientedEdge& oe : es) oe.forward = !oe.forward;
                        }
                        for (const OrientedEdge& oe : es) {
                            cs.push_back(writeEdge(w, b, oe));
                            for (const DVec3& q : edgePoints(b, oe, 8)) grow(q);
                        }
                    } else if (l.poly.size() >= 3) {
                        std::vector<DVec3> poly = l.poly;
                        if (backwards) std::reverse(poly.begin(), poly.end());
                        for (std::size_t i = 0; i < poly.size(); ++i) {
                            const DVec3 &a = poly[i], &c = poly[(i + 1) % poly.size()];
                            cs.push_back(w.add(110, {real(a.x), real(a.y), real(a.z), real(c.x), real(c.y), real(c.z)}));
                            grow(a);
                        }
                    } else continue; // a single vertex (a pole) bounds nothing
                    const double size = dlength(hi - lo);
                    if (l.outer && best < 1e300) best = 1e300, outer = static_cast<int>(loopCurves.size());
                    else if (size > best) best = size, outer = static_cast<int>(loopCurves.size());
                    loopCurves.push_back(std::move(cs));
                }
                std::vector<int> bounds;
                for (const std::vector<int>& cs : loopCurves) {
                    std::vector<std::string> p = {std::to_string(cs.size())};
                    for (int c : cs) p.push_back(std::to_string(c));
                    const int comp = w.add(102, p);
                    bounds.push_back(w.add(142, {"0", std::to_string(surf), "0", std::to_string(comp), "2"}));
                }
                std::vector<std::string> p = {std::to_string(surf), outer >= 0 ? "1" : "0", std::to_string(bounds.size() - (outer >= 0 ? 1 : 0)),
                                              outer >= 0 ? std::to_string(bounds[static_cast<std::size_t>(outer)]) : "0"};
                for (std::size_t i = 0; i < bounds.size(); ++i)
                    if (static_cast<int>(i) != outer) p.push_back(std::to_string(bounds[i]));
                w.add(144, p, 0, 0, false);
            }
    }
    return w.text(fileName, maxCoord);
}

bool writeIgesFile(const std::vector<StepPart>& parts, const std::string& path, std::string* error) {
    return writeFile(path, writeIges(parts, std::filesystem::path(path).filename().string()), error);
}

std::vector<StepPart> readIgesFile(const std::string& path, StepReadReport* report, std::string* error) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        if (error) *error = "Cannot open " + path;
        return {};
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return readIges(ss.str(), report, error);
}

} // namespace sicdm

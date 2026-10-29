// Weldments (Weldment.hpp).

#include "core/Weldment.hpp"

#include "core/ExactMesh.hpp"
#include "core/Extrude.hpp"
#include "core/Features.hpp"
#include "core/Sculpt.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <sstream>

namespace sicdm {

namespace {

constexpr float kHalfTurn = 3.14159265358979f;

std::string upperOf(std::string s) {
    for (char& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

std::string num(double v) {
    char b[32];
    std::snprintf(b, sizeof b, "%g", v);
    return b;
}

// A loop under construction: its points, and for each the arc of the edge that reaches it
// from the one before (-1 straight).
struct LoopBuilder {
    std::vector<Vec2> pts;
    std::vector<int> arcIn;
    std::vector<WeldProfile::Arc>* arcs;
    void point(Vec2 p, int arc = -1) {
        pts.push_back(p);
        arcIn.push_back(arc);
    }
    // Points on an arc from angle a0 to a1 (degrees, either way), n segments; the first
    // point is reached by a straight edge.
    void arc(Vec2 c, float r, float a0, float a1, int n) {
        const int index = static_cast<int>(arcs->size());
        arcs->push_back({c, r});
        for (int k = 0; k <= n; ++k) {
            const float a = (a0 + (a1 - a0) * static_cast<float>(k) / static_cast<float>(n)) * kHalfTurn / 180.0f;
            point({c.x + r * std::cos(a), c.y + r * std::sin(a)}, k == 0 ? -1 : index);
        }
    }
    // The loop closed: per edge (point k to k + 1) its arc.
    void finish(std::vector<Vec2>& out, std::vector<int>& edgeArc) const {
        out = pts;
        edgeArc.assign(pts.size(), -1);
        for (std::size_t k = 0; k < pts.size(); ++k) edgeArc[k] = arcIn[(k + 1) % pts.size()];
    }
};

double loopArea(const std::vector<Vec2>& l) {
    double a = 0;
    for (std::size_t k = 0, j = l.size() - 1; k < l.size(); j = k++) a += static_cast<double>(l[j].x) * l[k].y - static_cast<double>(l[k].x) * l[j].y;
    return a * 0.5;
}

// A rounded rectangle, counter-clockwise, n segments a corner (radius 0: a sharp corner).
void roundedRect(LoopBuilder& b, float w, float h, float r, int n) {
    const float x = w * 0.5f, y = h * 0.5f;
    const Vec2 c[4] = {{x - r, -y + r}, {x - r, y - r}, {-x + r, y - r}, {-x + r, -y + r}};
    for (int k = 0; k < 4; ++k) {
        if (r <= 0) b.point(Vec2{k == 0 || k == 1 ? x : -x, k == 1 || k == 2 ? y : -y});
        else b.arc(c[k], r, -90.0f + 90.0f * static_cast<float>(k), 90.0f * static_cast<float>(k), n);
    }
}

void circle(LoopBuilder& b, float r, int n) {
    const int index = static_cast<int>(b.arcs->size());
    b.arcs->push_back({{0, 0}, r});
    for (int k = 0; k < n; ++k) {
        const float a = 2 * kHalfTurn * static_cast<float>(k) / static_cast<float>(n);
        b.point({r * std::cos(a), r * std::sin(a)}, index);
    }
}

// The profile's points moved so its bounding box is centred on (0, 0).
void centre(WeldProfile& p) {
    AABB box;
    for (const Vec2& q : p.outer) box.expand(Vec3{q.x, q.y, 0});
    const Vec2 c{(box.min.x + box.max.x) * 0.5f, (box.min.y + box.max.y) * 0.5f};
    for (Vec2& q : p.outer) q = {q.x - c.x, q.y - c.y};
    for (Vec2& q : p.inner) q = {q.x - c.x, q.y - c.y};
    for (WeldProfile::Arc& a : p.arcs) a.centre = {a.centre.x - c.x, a.centre.y - c.y};
}

std::vector<double> dims(const std::string& s) {
    std::vector<double> out;
    std::string t;
    for (char c : s + "x") {
        if (c == 'x' || c == 'X' || c == '*' || c == '/') {
            if (!t.empty()) {
                char* end = nullptr;
                const double v = std::strtod(t.c_str(), &end);
                if (end == t.c_str() || *end) return {};
                out.push_back(v);
            }
            t.clear();
        } else if (!std::isspace(static_cast<unsigned char>(c))) {
            t += c == ',' ? '.' : c;
        }
    }
    return out;
}

struct Table {
    const char* size;
    double h, b, tw, tf, r;
};
// IPE beams (EN 10365): h, b, tw, tf, r (mm).
const Table kIpe[] = {{"80", 80, 46, 3.8, 5.2, 5},     {"100", 100, 55, 4.1, 5.7, 7},  {"120", 120, 64, 4.4, 6.3, 7},   {"140", 140, 73, 4.7, 6.9, 7},
                      {"160", 160, 82, 5.0, 7.4, 9},   {"180", 180, 91, 5.3, 8.0, 9},  {"200", 200, 100, 5.6, 8.5, 12}, {"220", 220, 110, 5.9, 9.2, 12},
                      {"240", 240, 120, 6.2, 9.8, 15}, {"270", 270, 135, 6.6, 10.2, 15}, {"300", 300, 150, 7.1, 10.7, 15}};
// UPE parallel flange channels (DIN 1026-2): h, b, tw, tf, r.
const Table kUpe[] = {{"80", 80, 50, 4.0, 7.0, 10},    {"100", 100, 55, 4.5, 7.5, 10}, {"120", 120, 60, 5.0, 8.0, 12}, {"140", 140, 65, 5.0, 9.0, 12},
                      {"160", 160, 70, 5.5, 9.5, 12},  {"180", 180, 75, 5.5, 10.5, 12}, {"200", 200, 80, 6.0, 11.0, 13}};
// Equal angles (EN 10056-1): the size and the root radius.
const std::pair<const char*, double> kAngleRoot[] = {{"20x20x3", 3.5}, {"25x25x3", 3.5}, {"30x30x3", 5}, {"40x40x4", 6},   {"45x45x5", 7},  {"50x50x5", 7},
                                                     {"60x60x6", 8},   {"70x70x7", 9},   {"80x80x8", 10}, {"90x90x9", 11}, {"100x100x10", 12}};

const char* const kShs[] = {"20x20x2", "25x25x2",  "25x25x3",  "30x30x2",  "30x30x3",  "40x40x2",   "40x40x3",   "40x40x4",   "50x50x3",
                            "50x50x4", "50x50x5",  "60x60x3",  "60x60x4",  "60x60x5",  "70x70x4",   "80x80x4",   "80x80x5",   "80x80x6",
                            "90x90x5", "100x100x4", "100x100x5", "100x100x6", "100x100x8", "120x120x6", "140x140x6", "150x150x8"};
const char* const kRhs[] = {"40x20x2", "40x20x3",  "50x25x2",  "50x30x3",   "60x30x3",   "60x40x3",   "60x40x4",  "70x40x4",   "80x40x3",  "80x40x4",
                            "80x60x4", "100x40x4", "100x50x4", "100x60x5",  "120x60x5",  "120x80x5",  "140x80x5", "150x100x6", "160x80x6", "200x100x6"};
const char* const kChs[] = {"21.3x2.6", "26.9x2.6", "33.7x3.2", "42.4x3.2", "48.3x3.2", "48.3x4", "60.3x4", "76.1x4", "88.9x5", "114.3x6.3", "139.7x6.3", "168.3x7.1"};
const char* const kFlat[] = {"20x3", "20x5", "25x5", "30x5", "40x5", "40x8", "50x6", "50x10", "60x8", "80x10", "100x10"};
const char* const kRound[] = {"8", "10", "12", "16", "20", "25", "30", "40", "50"};

// An I section: h, b, tw, tf and root radius r, symmetric, counter-clockwise.
void iSection(LoopBuilder& L, double h, double b, double tw, double tf, double r, int n) {
    const float H = static_cast<float>(h) * 0.5f, B = static_cast<float>(b) * 0.5f, W = static_cast<float>(tw) * 0.5f, F = static_cast<float>(tf), R = static_cast<float>(r);
    L.point({-B, -H});
    L.point({B, -H});
    L.point({B, -H + F});
    if (R > 0) L.arc({W + R, -H + F + R}, R, 270, 180, n);
    else L.point({W, -H + F});
    if (R > 0) L.arc({W + R, H - F - R}, R, 180, 90, n);
    else L.point({W, H - F});
    L.point({B, H - F});
    L.point({B, H});
    L.point({-B, H});
    L.point({-B, H - F});
    if (R > 0) L.arc({-W - R, H - F - R}, R, 90, 0, n);
    else L.point({-W, H - F});
    if (R > 0) L.arc({-W - R, -H + F + R}, R, 0, -90, n);
    else L.point({-W, -H + F});
    L.point({-B, -H + F});
}

// A channel (web on the left, flanges to the right): h, b, tw, tf, root radius r.
void cSection(LoopBuilder& L, double h, double b, double tw, double tf, double r, int n) {
    const float H = static_cast<float>(h), B = static_cast<float>(b), W = static_cast<float>(tw), F = static_cast<float>(tf), R = static_cast<float>(r);
    L.point({0, 0});
    L.point({B, 0});
    L.point({B, F});
    if (R > 0) L.arc({W + R, F + R}, R, 270, 180, n);
    else L.point({W, F});
    if (R > 0) L.arc({W + R, H - F - R}, R, 180, 90, n);
    else L.point({W, H - F});
    L.point({B, H - F});
    L.point({B, H});
    L.point({0, H});
}

// An angle: legs a (up) and b (across), thickness t, root radius r; the heel at (0, 0).
void lSection(LoopBuilder& L, double a, double b, double t, double r, int n) {
    const float A = static_cast<float>(a), B = static_cast<float>(b), T = static_cast<float>(t), R = static_cast<float>(r);
    L.point({0, 0});
    L.point({B, 0});
    L.point({B, T});
    if (R > 0) L.arc({T + R, T + R}, R, 270, 180, n);
    else L.point({T, T});
    L.point({T, A});
    L.point({0, A});
}

} // namespace

std::vector<std::string> weldProfileFamilies() { return {"SHS", "RHS", "CHS", "FLAT", "ROUND", "ANGLE", "UPE", "IPE", "I", "C"}; }

std::vector<std::string> weldProfileSizes(const std::string& family) {
    const std::string f = upperOf(family);
    std::vector<std::string> out;
    if (f == "SHS") out.assign(std::begin(kShs), std::end(kShs));
    else if (f == "RHS") out.assign(std::begin(kRhs), std::end(kRhs));
    else if (f == "CHS" || f == "PIPE") out.assign(std::begin(kChs), std::end(kChs));
    else if (f == "FLAT") out.assign(std::begin(kFlat), std::end(kFlat));
    else if (f == "ROUND") out.assign(std::begin(kRound), std::end(kRound));
    else if (f == "ANGLE" || f == "L")
        for (const auto& [s, r] : kAngleRoot) out.push_back(s);
    else if (f == "UPE")
        for (const Table& t : kUpe) out.push_back(t.size);
    else if (f == "IPE")
        for (const Table& t : kIpe) out.push_back(t.size);
    return out;
}

std::optional<WeldProfile> weldProfile(const std::string& designation, std::string* why) {
    auto fail = [&](const std::string& w) -> std::optional<WeldProfile> {
        if (why) *why = w;
        return std::nullopt;
    };
    // The family, then the size ("SHS 40x40x3", "SHS40x40x3", "L50x50x5", "IPE 200").
    std::string s;
    for (char c : designation)
        if (!std::isspace(static_cast<unsigned char>(c)) || (!s.empty() && s.back() != ' ')) s += c;
    while (!s.empty() && s.back() == ' ') s.pop_back();
    std::size_t k = 0;
    while (k < s.size() && std::isalpha(static_cast<unsigned char>(s[k]))) ++k;
    std::string family = upperOf(s.substr(0, k));
    std::string size = s.substr(k);
    while (!size.empty() && size[0] == ' ') size.erase(0, 1);
    if (family == "PIPE" || family == "TUBE") family = "CHS";
    if (family == "L") family = "ANGLE";
    if (family == "FB" || family == "BAR") family = "FLAT";
    if (family == "RB" || family == "ROD") family = "ROUND";
    const std::vector<double> d = dims(size);
    WeldProfile p;
    p.family = family;
    p.designation = family + " " + size;
    LoopBuilder outer{{}, {}, &p.arcs}, inner{{}, {}, &p.arcs};
    const int n = 6; // segments a quarter round
    auto positive = [&](std::initializer_list<double> v) { return std::all_of(v.begin(), v.end(), [](double x) { return x > 0 && x < 1e5; }); };
    if (family == "SHS" || family == "RHS") {
        double w = 0, h = 0, t = 0;
        if (family == "SHS" && d.size() == 2) w = h = d[0], t = d[1];
        else if (d.size() == 3) w = d[0], h = d[1], t = d[2];
        if (!positive({w, h, t}) || 2 * t >= std::min(w, h)) return fail(family + " is width x height x wall (" + family + " 40x40x3)");
        // EN 10219: outer corner radius 2t and inner t up to 6 mm walls; 2.5t and 1.5t above.
        const double ro = std::min(t <= 6 ? 2 * t : 2.5 * t, std::min(w, h) * 0.5 - 1e-3), ri = std::max(1e-3, ro - t);
        roundedRect(outer, static_cast<float>(w), static_cast<float>(h), static_cast<float>(ro), n);
        roundedRect(inner, static_cast<float>(w - 2 * t), static_cast<float>(h - 2 * t), static_cast<float>(ri), n);
        p.designation = family + " " + num(w) + "x" + num(h) + "x" + num(t);
        if (family == "SHS") p.designation = "SHS " + num(w) + "x" + num(w) + "x" + num(t);
    } else if (family == "CHS") {
        if (d.size() != 2 || !positive({d[0], d[1]}) || 2 * d[1] >= d[0]) return fail("CHS is diameter x wall (CHS 48.3x3.2)");
        circle(outer, static_cast<float>(d[0] * 0.5), 48);
        circle(inner, static_cast<float>(d[0] * 0.5 - d[1]), 48);
        p.designation = "CHS " + num(d[0]) + "x" + num(d[1]);
    } else if (family == "FLAT") {
        if (d.size() != 2 || !positive({d[0], d[1]})) return fail("FLAT is width x thickness (FLAT 40x6)");
        roundedRect(outer, static_cast<float>(d[0]), static_cast<float>(d[1]), 0, 0);
        p.designation = "FLAT " + num(d[0]) + "x" + num(d[1]);
    } else if (family == "ROUND") {
        if (d.size() != 1 || !positive({d[0]})) return fail("ROUND is the diameter (ROUND 20)");
        circle(outer, static_cast<float>(d[0] * 0.5), 48);
        p.designation = "ROUND " + num(d[0]);
    } else if (family == "ANGLE") {
        double a = 0, b = 0, t = 0;
        if (d.size() == 2) a = b = d[0], t = d[1];
        else if (d.size() == 3) a = d[0], b = d[1], t = d[2];
        if (!positive({a, b, t}) || t >= std::min(a, b)) return fail("ANGLE is leg x leg x thickness (ANGLE 50x50x5)");
        const std::string key = num(a) + "x" + num(b) + "x" + num(t);
        double r = std::min(t * 1.2, (std::min(a, b) - t) * 0.5);
        for (const auto& [sz, root] : kAngleRoot)
            if (key == sz) r = root;
        lSection(outer, a, b, t, r, n);
        p.designation = "ANGLE " + key;
    } else if (family == "IPE" || family == "UPE") {
        std::optional<Table> t;
        const Table* first = family == "IPE" ? std::begin(kIpe) : std::begin(kUpe);
        const Table* last = family == "IPE" ? std::end(kIpe) : std::end(kUpe);
        for (const Table* e = first; e != last; ++e)
            if (size == e->size) t = *e, p.designation = family + " " + e->size;
        if (!t) {
            std::string list;
            for (const std::string& z : weldProfileSizes(family)) list += (list.empty() ? "" : ", ") + z;
            return fail(family + " " + size + " isn't a standard size (" + list + "); custom sections are I or C h x b x tw x tf");
        }
        if (family == "IPE") iSection(outer, t->h, t->b, t->tw, t->tf, t->r, n);
        else cSection(outer, t->h, t->b, t->tw, t->tf, t->r, n);
    } else if (family == "I" || family == "C") {
        if (d.size() < 4 || d.size() > 5 || !positive({d[0], d[1], d[2], d[3]}) || 2 * d[3] >= d[0] || d[2] >= d[1])
            return fail(family + " is height x width x web x flange [x root radius] (" + family + " 200x100x6x9)");
        const double r = d.size() == 5 ? d[4] : d[3];
        if (r < 0 || 2 * (d[3] + r) >= d[0] || (family == "I" ? d[2] * 0.5 + r >= d[1] * 0.5 : d[2] + r >= d[1])) return fail("the root radius doesn't fit");
        if (family == "I") iSection(outer, d[0], d[1], d[2], d[3], r, n);
        else cSection(outer, d[0], d[1], d[2], d[3], r, n);
        p.designation = family + " " + num(d[0]) + "x" + num(d[1]) + "x" + num(d[2]) + "x" + num(d[3]) + (d.size() == 5 ? "x" + num(d[4]) : std::string());
    } else {
        std::string list;
        for (const std::string& f : weldProfileFamilies()) list += (list.empty() ? "" : ", ") + f;
        return fail("No profile family " + (family.empty() ? designation : family) + " (" + list + ")");
    }
    outer.finish(p.outer, p.outerArc);
    if (!inner.pts.empty()) inner.finish(p.inner, p.innerArc);
    centre(p);
    p.polygonArea = std::fabs(loopArea(p.outer)) - (p.inner.empty() ? 0.0 : std::fabs(loopArea(p.inner)));
    // Exact: each faceted arc's sliver between chord and arc added back (taken off where
    // the arc is hollow, or bounds the hole).
    double slivers = 0;
    for (int loop = 0; loop < 2; ++loop) {
        const std::vector<Vec2>& L = loop ? p.inner : p.outer;
        const std::vector<int>& A = loop ? p.innerArc : p.outerArc;
        for (std::size_t e = 0; e < L.size(); ++e) {
            if (A[e] < 0) continue;
            const WeldProfile::Arc& arc = p.arcs[static_cast<std::size_t>(A[e])];
            const Vec2 a = L[e], b = L[(e + 1) % L.size()];
            const double ax = a.x - arc.centre.x, ay = a.y - arc.centre.y, bx = b.x - arc.centre.x, by = b.y - arc.centre.y;
            const double turn = ax * by - ay * bx, th = std::atan2(std::fabs(turn), ax * bx + ay * by);
            const double seg = 0.5 * static_cast<double>(arc.radius) * arc.radius * (th - std::sin(th));
            slivers += (turn >= 0 ? seg : -seg) * (loop ? -1 : 1);
        }
    }
    p.area = p.polygonArea + slivers;
    if (loopArea(p.outer) <= 0 || p.area <= 0 || !isSimplePolygon(p.outer)) return fail("the profile " + p.designation + " doesn't close up");
    return p;
}

MemberFrame memberFrame(const Body::Member& m, const WeldProfile& p) {
    MemberFrame f;
    Vec3 d = m.end - m.start;
    d = length(d) > 0 ? normalize(d) : Vec3{1, 0, 0};
    Vec3 up = m.up - d * dot(m.up, d);
    if (length(up) < 1e-4f) { // along the line: another way up
        const Vec3 alt = std::fabs(d.z) < 0.9f ? Vec3{0, 0, 1} : Vec3{0, 1, 0};
        up = alt - d * dot(alt, d);
    }
    up = normalize(up);
    Vec3 across = cross(up, d);
    const float a = m.angle * kHalfTurn / 180.0f;
    const Vec3 x = across * std::cos(a) + up * std::sin(a), y = up * std::cos(a) - across * std::sin(a);
    f.across = x, f.up = y, f.along = d;
    // Where the line runs on the profile.
    AABB box;
    for (const Vec2& q : p.outer) box.expand(Vec3{m.mirror ? -q.x : q.x, q.y, 0});
    std::string al = m.align;
    for (char& c : al) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    float sx = 0, sy = 0;
    if (al.find("top") != std::string::npos) sy = -box.max.y;
    if (al.find("bottom") != std::string::npos) sy = -box.min.y;
    if (al.find("left") != std::string::npos) sx = -box.min.x;
    if (al.find("right") != std::string::npos) sx = -box.max.x;
    f.origin = m.start + x * sx + y * sy;
    return f;
}

namespace {

// A loop's 3D points at the line's start: profile (x, y) placed by the frame.
Vec3 placed(const MemberFrame& f, const Vec2& q, bool mirror) { return f.origin + f.across * (mirror ? -q.x : q.x) + f.up * q.y; }

// The end planes (outward normals): the cut found, or square at the line's ends.
void endPlanes(const Body::Member& m, const MemberFrame& f, Vec3 n[2], double w[2]) {
    for (int e = 0; e < 2; ++e) {
        const Vec4& c = m.cut[static_cast<std::size_t>(e)];
        const Vec3 cn{c.x, c.y, c.z};
        if (length(cn) > 0.5f) n[e] = normalize(cn), w[e] = c.w;
        else {
            n[e] = e == 0 ? f.along * -1.0f : f.along;
            w[e] = dot(n[e], e == 0 ? m.start : m.end);
        }
    }
}

// Where along the line from b the end plane is.
double tAt(const Vec3& b, const Vec3& d, const Vec3& n, double w) { return (w - dot(n, b)) / static_cast<double>(dot(n, d)); }

} // namespace

bool buildMember(const Body::Member& m, const WeldProfile& p0, MeshData& mesh, std::vector<BSurface>& surfaces, std::string* why) {
    auto fail = [&](const std::string& w) {
        if (why) *why = w;
        return false;
    };
    if (length(m.end - m.start) < 1e-3f) return fail("the member has no length");
    // Mirrored: the profile flipped left to right, its loops turned back round to run
    // counter-clockwise.
    WeldProfile p = p0;
    if (m.mirror) {
        for (auto* loop : {&p.outer, &p.inner}) {
            for (Vec2& q : *loop) q.x = -q.x;
            std::reverse(loop->begin(), loop->end());
        }
        for (auto* arcs : {&p.outerArc, &p.innerArc}) {
            const std::size_t nn = arcs->size();
            std::vector<int> r(nn);
            for (std::size_t k = 0; k < nn; ++k) r[k] = (*arcs)[(2 * nn - 2 - k) % nn];
            *arcs = r;
        }
        for (WeldProfile::Arc& a : p.arcs) a.centre.x = -a.centre.x;
    }
    MemberFrame f = memberFrame(m, p0);
    // (memberFrame placed the unmirrored profile's box; the points here are already flipped.)
    Vec3 n[2];
    double w[2];
    endPlanes(m, f, n, w);
    const Vec3 d = f.along;
    if (dot(n[0], d) > -0.1f || dot(n[1], d) < 0.1f) return fail("an end is cut too steeply (more than 84 degrees off square)");
    auto placeLoop = [&](const std::vector<Vec2>& loop, std::vector<Vec3>& s, std::vector<Vec3>& e) {
        for (const Vec2& q : loop) {
            const Vec3 b = f.origin + f.across * q.x + f.up * q.y;
            const double t0 = tAt(b, d, n[0], w[0]), t1 = tAt(b, d, n[1], w[1]);
            if (t1 - t0 < 1e-3) return false;
            s.push_back(b + d * static_cast<float>(t0));
            e.push_back(b + d * static_cast<float>(t1));
        }
        return true;
    };
    std::vector<Vec3> os, oe, is, ie;
    if (!placeLoop(p.outer, os, oe) || !placeLoop(p.inner, is, ie)) return fail("its ends cross: the cuts at its two ends meet inside it");
    mesh = MeshData{};
    auto vtx = [&](const Vec3& q, const Vec3& nn) {
        mesh.vertices.push_back({q, nn});
        return static_cast<std::uint32_t>(mesh.vertices.size() - 1);
    };
    // Side walls: flat edges a normal each; edges on an arc the arc's radial normals.
    auto walls = [&](const std::vector<Vec2>& loop, const std::vector<int>& arcOf, const std::vector<Vec3>& s, const std::vector<Vec3>& e, bool hole) {
        const std::size_t nn = loop.size();
        for (std::size_t k = 0; k < nn; ++k) {
            const std::size_t k1 = (k + 1) % nn;
            const Vec2 a = loop[k], b = loop[k1];
            Vec2 na, nb;
            if (arcOf[k] >= 0) {
                const WeldProfile::Arc& arc = p.arcs[static_cast<std::size_t>(arcOf[k])];
                // Round the arc's centre counter-clockwise: convex (normal out from the
                // centre), else the other way.
                const float turn = (a.x - arc.centre.x) * (b.y - arc.centre.y) - (a.y - arc.centre.y) * (b.x - arc.centre.x);
                const float sgn = turn >= 0 ? 1.0f : -1.0f;
                na = {(a.x - arc.centre.x) * sgn, (a.y - arc.centre.y) * sgn}, nb = {(b.x - arc.centre.x) * sgn, (b.y - arc.centre.y) * sgn};
            } else {
                na = nb = {b.y - a.y, a.x - b.x}; // right of a counter-clockwise edge: out
            }
            if (hole) na = {-na.x, -na.y}, nb = {-nb.x, -nb.y};
            const Vec3 Na = normalize(f.across * na.x + f.up * na.y), Nb = normalize(f.across * nb.x + f.up * nb.y);
            const std::uint32_t v0 = vtx(s[k], Na), v1 = vtx(s[k1], Nb), v2 = vtx(e[k1], Nb), v3 = vtx(e[k], Na);
            if (!hole) mesh.indices.insert(mesh.indices.end(), {v0, v1, v2, v0, v2, v3});
            else mesh.indices.insert(mesh.indices.end(), {v1, v0, v3, v1, v3, v2});
        }
    };
    walls(p.outer, p.outerArc, os, oe, false);
    if (p.hollow()) walls(p.inner, p.innerArc, is, ie, true);
    // End caps: rings between the loops, or the profile's own triangles.
    for (int end = 0; end < 2; ++end) {
        const std::vector<Vec3>& o = end ? oe : os;
        const std::vector<Vec3>& in = end ? ie : is;
        const Vec3 N = n[end];
        const std::uint32_t base = static_cast<std::uint32_t>(mesh.vertices.size());
        for (const Vec3& q : o) vtx(q, N);
        for (const Vec3& q : in) vtx(q, N);
        auto tri = [&](std::uint32_t a, std::uint32_t b, std::uint32_t c) {
            const Vec3 tn = cross(mesh.vertices[b].position - mesh.vertices[a].position, mesh.vertices[c].position - mesh.vertices[a].position);
            if (dot(tn, N) >= 0) mesh.indices.insert(mesh.indices.end(), {a, b, c});
            else mesh.indices.insert(mesh.indices.end(), {a, c, b});
        };
        const std::uint32_t no = static_cast<std::uint32_t>(o.size());
        if (p.hollow()) {
            for (std::uint32_t k = 0; k < no; ++k) {
                const std::uint32_t k1 = (k + 1) % no;
                tri(base + k, base + k1, base + no + k1);
                tri(base + k, base + no + k1, base + no + k);
            }
        } else {
            const std::vector<std::uint32_t> t = triangulatePolygon(p.outer);
            if (t.size() != (p.outer.size() - 2) * 3) return fail("the profile can't be capped");
            for (std::size_t k = 0; k + 2 < t.size(); k += 3) tri(base + t[k], base + t[k + 1], base + t[k + 2]);
        }
    }
    mesh.computeFeatureEdges();
    // Exact surfaces: a plane for each flat side, a cylinder for each round, the end planes.
    surfaces.clear();
    auto planeAt = [&](const Vec3& o, const Vec3& nn, const Vec3& x) {
        BSurface s;
        s.kind = BSurfaceKind::Plane;
        s.f = Frame::make(DVec3(o), DVec3(normalize(nn)), DVec3(x));
        surfaces.push_back(s);
    };
    for (int loop = 0; loop < 2; ++loop) {
        const std::vector<Vec2>& L = loop ? p.inner : p.outer;
        const std::vector<int>& A = loop ? p.innerArc : p.outerArc;
        for (std::size_t k = 0; k < L.size(); ++k) {
            if (A[k] >= 0) continue;
            const Vec2 a = L[k], b = L[(k + 1) % L.size()];
            if (std::hypot(b.x - a.x, b.y - a.y) < 1e-6f) continue;
            Vec2 nn{b.y - a.y, a.x - b.x};
            if (loop) nn = {-nn.x, -nn.y};
            planeAt(f.origin + f.across * a.x + f.up * a.y, f.across * nn.x + f.up * nn.y, d);
        }
    }
    for (const WeldProfile::Arc& arc : p.arcs) {
        BSurface s;
        s.kind = BSurfaceKind::Cylinder;
        s.f = Frame::make(DVec3(f.origin + f.across * arc.centre.x + f.up * arc.centre.y), DVec3(d), DVec3(f.across));
        s.r1 = arc.radius;
        surfaces.push_back(s);
    }
    for (int e = 0; e < 2; ++e) planeAt(n[e] * static_cast<float>(w[e]), n[e], std::fabs(n[e].z) < 0.9f ? cross(n[e], Vec3{0, 0, 1}) : Vec3{1, 0, 0});
    return true;
}

namespace {

struct Placed {
    std::size_t body;
    WeldProfile profile;
    MemberFrame frame;
    Vec3 s, e, d; // the line in the world
    Vec3 offset;  // the body's
    std::vector<Vec3> section; // the profile's points, from the line (world directions)
    bool editable;
};

// How far the section reaches from the line towards `dir`.
float reach(const Placed& m, const Vec3& dir) {
    float r = -1e30f;
    for (const Vec3& q : m.section) r = std::max(r, dot(q, dir));
    return r;
}

} // namespace

int resolveJoints(Scene& scene, const std::string& group, std::vector<std::string>* notes) {
    auto& bodies = scene.bodies();
    std::vector<Placed> ms;
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        const Body& b = bodies[i];
        if (!b.isMember() || (!group.empty() && b.member.group != group)) continue;
        auto p = weldProfile(b.member.profile);
        if (!p) continue;
        Placed m;
        m.body = i;
        m.profile = *p;
        m.frame = memberFrame(b.member, *p);
        m.offset = b.offset;
        m.s = b.member.start + b.offset, m.e = b.member.end + b.offset;
        m.d = normalize(m.e - m.s);
        for (const Vec2& q : p->outer) m.section.push_back(placed(m.frame, q, b.member.mirror) - b.member.start);
        m.editable = b.member.key == 0 || b.member.key == meshKey(b.mesh);
        ms.push_back(std::move(m));
    }
    const float tol = 0.01f;
    // An end's plane (world): outward normal and w.
    struct Plane {
        Vec3 n{};
        double w = 0;
        bool set = false;
    };
    std::vector<std::array<Plane, 2>> planes(ms.size());
    auto square = [&](std::size_t i, int e) {
        const Placed& m = ms[i];
        const Vec3 n = e ? m.d : m.d * -1.0f;
        return Plane{n, dot(n, e ? m.e : m.s), true};
    };
    // i's end e stops at j's side, j running along `bj`.
    auto butt = [&](std::size_t i, int e, std::size_t j) -> Plane {
        const Placed& mi = ms[i];
        const Vec3 a = e ? mi.d : mi.d * -1.0f; // into the joint
        const Vec3 P = e ? mi.e : mi.s;
        const Vec3 aPerp = a - ms[j].d * dot(a, ms[j].d);
        if (length(aPerp) < 1e-3f) return square(i, e);
        const Vec3 nF = normalize(aPerp);
        // The line of j nearest P, and how far j's section reaches back from it towards i.
        return Plane{nF, dot(nF, P) - reach(ms[j], nF * -1.0f), true};
    };
    auto styleOf = [&](std::size_t i, int e) { return bodies[ms[i].body].member.style[static_cast<std::size_t>(e)]; };
    std::vector<std::string> said;
    for (std::size_t i = 0; i < ms.size(); ++i)
        for (int e = 0; e < 2; ++e) {
            const Vec3 P = e ? ms[i].e : ms[i].s;
            // Ends that meet this one.
            std::vector<std::pair<std::size_t, int>> at;
            for (std::size_t j = 0; j < ms.size(); ++j)
                if (j != i)
                    for (int f = 0; f < 2; ++f)
                        if (length((f ? ms[j].e : ms[j].s) - P) < tol) at.push_back({j, f});
            // Three at a corner, one square to the plane of the other two (a leg under a frame's
            // corner): the two make their corner, the post stops under them.
            if (at.size() == 2) {
                const std::size_t trio[3] = {i, at[0].first, at[1].first};
                // The post: square to the other two's plane; of several (a cube's corner), the
                // one running along the other two's up (a leg under a frame laid flat).
                int post = -1, squareTo = 0;
                for (int k = 0; k < 3; ++k) {
                    const Placed& o1 = ms[trio[(k + 1) % 3]];
                    const Placed& o2 = ms[trio[(k + 2) % 3]];
                    const Vec3 n = cross(o1.d, o2.d);
                    if (length(n) < 0.1f || std::fabs(dot(normalize(n), ms[trio[k]].d)) < 0.99f) continue;
                    ++squareTo;
                    if (post < 0 || (std::fabs(dot(ms[trio[k]].d, o1.frame.up)) > 0.99f && std::fabs(dot(ms[trio[k]].d, o2.frame.up)) > 0.99f)) post = k;
                }
                if (squareTo > 1) { // only one of them along the others' up will do
                    post = -1;
                    for (int k = 0; k < 3; ++k)
                        if (std::fabs(dot(ms[trio[k]].d, ms[trio[(k + 1) % 3]].frame.up)) > 0.99f && std::fabs(dot(ms[trio[k]].d, ms[trio[(k + 2) % 3]].frame.up)) > 0.99f)
                            post = post < 0 ? k : 3; // (two such: none)
                    if (post == 3) post = -1;
                }
                if (post == 0) { // this one: up to the others' faces
                    const Vec3 a = e ? ms[i].d : ms[i].d * -1.0f;
                    const float r = std::max(reach(ms[at[0].first], a * -1.0f), reach(ms[at[1].first], a * -1.0f));
                    planes[i][static_cast<std::size_t>(e)] = Plane{a, dot(a, P) - r, true};
                    continue;
                }
                if (post > 0) at.erase(at.begin() + (post - 1)); // the corner without the post
            }
            if (at.size() > 1) {
                planes[i][static_cast<std::size_t>(e)] = square(i, e);
                if (notes && std::find(said.begin(), said.end(), "many") == said.end()) {
                    notes->push_back("Where more than two members meet at one point (and none is a post square to the others) their ends are left square");
                    said.push_back("many");
                }
                continue;
            }
            if (at.empty()) {
                // A T: the end on another member's side.
                for (std::size_t j = 0; j < ms.size() && !planes[i][static_cast<std::size_t>(e)].set; ++j) {
                    if (j == i) continue;
                    const Vec3 sj = ms[j].s, ej = ms[j].e;
                    const float L = length(ej - sj), t = dot(P - sj, ms[j].d);
                    if (t <= tol || t >= L - tol) continue;
                    if (length(sj + ms[j].d * t - P) > tol) continue;
                    planes[i][static_cast<std::size_t>(e)] = styleOf(i, e) == 3 ? square(i, e) : butt(i, e, j);
                }
                if (!planes[i][static_cast<std::size_t>(e)].set) planes[i][static_cast<std::size_t>(e)] = square(i, e);
                continue;
            }
            // A corner of two.
            const std::size_t j = at[0].first;
            const int f = at[0].second;
            int style = styleOf(i, e); // from i's side: 0 miter, 1 i stops, 2 i runs through, 3 square
            if (style < 0) {
                const int sj = styleOf(j, f);
                if (sj >= 0) style = sj == 1 ? 2 : sj == 2 ? 1 : sj;
                else { // the earlier member's corner: butt, the earlier one running through
                    const std::size_t first = std::min(i, j);
                    const int corner = bodies[ms[first].body].member.corner;
                    style = corner == 0 ? 0 : corner == 2 ? 3 : (first == i ? 2 : 1);
                }
            }
            const Vec3 a = e ? ms[i].d : ms[i].d * -1.0f;       // i into the joint
            const Vec3 b = f == 0 ? ms[j].d : ms[j].d * -1.0f;  // j away from it
            if (std::fabs(dot(a, b)) > 0.9999f) { // in line (or folded back): square
                planes[i][static_cast<std::size_t>(e)] = square(i, e);
                continue;
            }
            if (style == 0) {
                const Vec3 nm = normalize(a + b);
                planes[i][static_cast<std::size_t>(e)] = Plane{nm, dot(nm, P), true};
            } else if (style == 1) {
                planes[i][static_cast<std::size_t>(e)] = butt(i, e, j);
            } else if (style == 2) {
                // Through: on past the joint far enough to cover j's end, which stops at i.
                const Plane pj = butt(j, f, i);
                float far = 0;
                for (const Vec3& q : ms[j].section) {
                    const Vec3 base = P + q;
                    const float t = static_cast<float>((pj.w - dot(pj.n, base)) / dot(pj.n, ms[j].d));
                    far = std::max(far, dot(base + ms[j].d * t - P, a));
                }
                planes[i][static_cast<std::size_t>(e)] = Plane{a, dot(a, P) + far, true};
            } else {
                planes[i][static_cast<std::size_t>(e)] = square(i, e);
            }
        }
    // Remade where an end changed.
    int remade = 0;
    for (std::size_t i = 0; i < ms.size(); ++i) {
        Body& b = bodies[ms[i].body];
        Body::Member next = b.member;
        for (int e = 0; e < 2; ++e) {
            const Plane& pl = planes[i][static_cast<std::size_t>(e)];
            next.cut[static_cast<std::size_t>(e)] = Vec4{pl.n.x, pl.n.y, pl.n.z, static_cast<float>(pl.w - dot(pl.n, ms[i].offset))};
        }
        auto same = [](const Vec4& x, const Vec4& y) { return std::fabs(x.x - y.x) < 1e-6f && std::fabs(x.y - y.y) < 1e-6f && std::fabs(x.z - y.z) < 1e-6f && std::fabs(x.w - y.w) < 1e-4f; };
        if (b.member.key != 0 && same(next.cut[0], b.member.cut[0]) && same(next.cut[1], b.member.cut[1])) continue;
        if (!ms[i].editable) {
            if (notes) notes->push_back(b.name + " was changed since it was made (cut, drilled...): its ends are left as they are");
            continue;
        }
        MeshData mesh;
        std::vector<BSurface> surfaces;
        std::string why;
        if (!buildMember(next, ms[i].profile, mesh, surfaces, &why)) {
            if (notes) notes->push_back(b.name + ": " + why + " (left as it was)");
            continue;
        }
        b.member = next;
        b.mesh = std::move(mesh);
        b.kind = "mesh";
        b.params.clear();
        keepExactGeometry(b, surfaces);
        b.member.key = meshKey(b.mesh);
        ++remade;
    }
    if (remade) scene.touchBodies();
    return remade;
}

std::vector<CutItem> cutList(const Scene& scene, const std::string& group) {
    std::vector<CutItem> items;
    for (const Body& b : scene.bodies()) {
        if (!b.isMember() || (!group.empty() && b.member.group != group)) continue;
        const auto p = weldProfile(b.member.profile);
        if (!p) continue;
        const MemberFrame f = memberFrame(b.member, *p);
        Vec3 n[2];
        double w[2];
        endPlanes(b.member, f, n, w);
        // Tip to tip: the longest the profile's points run between the cuts.
        double len = 0;
        for (const Vec2& q : p->outer) {
            const Vec3 base = placed(f, q, b.member.mirror);
            len = std::max(len, tAt(base, f.along, n[1], w[1]) - tAt(base, f.along, n[0], w[0]));
        }
        double ang[2];
        for (int e = 0; e < 2; ++e) ang[e] = std::acos(std::clamp(std::fabs(static_cast<double>(dot(n[e], f.along))), 0.0, 1.0)) * 180.0 / 3.14159265358979;
        if (ang[0] > ang[1]) std::swap(ang[0], ang[1]);
        const double rho = materialDensity(b.member.material).value_or(7.85); // g/cm3
        // The mesh's volume, its faceted rounds made round again (mm3 -> kg).
        const double mass = meshVolume(b.mesh) * (p->polygonArea > 0 ? p->area / p->polygonArea : 1.0) * rho * 1e-6;
        auto r1 = [](double v) { return std::round(v * 10) / 10; };
        auto it = std::find_if(items.begin(), items.end(), [&](const CutItem& c) {
            return c.profile == p->designation && c.material == b.member.material && r1(c.length) == r1(len) && r1(c.angle0) == r1(ang[0]) && r1(c.angle1) == r1(ang[1]);
        });
        if (it == items.end()) {
            CutItem c;
            c.profile = p->designation, c.material = b.member.material, c.length = len, c.angle0 = ang[0], c.angle1 = ang[1], c.massEach = mass;
            items.push_back(c);
            it = items.end() - 1;
        }
        it->members.push_back(b.name);
    }
    std::stable_sort(items.begin(), items.end(), [](const CutItem& a, const CutItem& b) { return a.profile != b.profile ? a.profile < b.profile : a.length > b.length; });
    return items;
}

} // namespace sicdm

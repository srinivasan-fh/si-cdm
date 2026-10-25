#include "core/CamVerify.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <map>
#include <unordered_map>
#include <utility>

namespace sicdm {

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kPiV = 3.14159265358979323846;

struct Dialect {
    std::string key;
    bool fanuc = false; // Fanuc-style number format (Fanuc, Haas)
    bool haas = false;
    bool grbl = false;
    bool linuxcnc = false;
};

bool dialectOf(const std::string& name, Dialect* d) {
    std::string k;
    for (char ch : name) k += static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    if (k == "fanuc") *d = {"fanuc", true, false, false, false};
    else if (k == "haas") *d = {"haas", true, true, false, false};
    else if (k == "grbl") *d = {"grbl", false, false, true, false};
    else if (k == "linuxcnc" || k == "emc" || k == "mach3" || k == "mach") *d = {"linuxcnc", false, false, false, true};
    else return false;
    return true;
}

// G codes as tenths (G33.1 is 331).
bool knownG(const Dialect& d, int g10) {
    static const int common[] = {0, 10, 20, 30, 40, 170, 180, 190, 200, 210, 280, 400, 490, 530, 540, 800, 900, 910, 930, 940};
    for (int g : common)
        if (g == g10) return true;
    if (!d.grbl)
        for (int g : {430, 730, 810, 820, 830, 840, 850, 980, 990})
            if (g == g10) return true;
    if (d.linuxcnc && (g10 == 640 || g10 == 331)) return true;
    if (d.fanuc && (d.haas ? g10 == 2340 : g10 == 434)) return true; // tool centre point control
    return false;
}

bool knownM(const Dialect& d, int m) {
    for (int c : {0, 1, 2, 3, 4, 5, 8, 9, 30})
        if (c == m) return true;
    if (!d.grbl && m == 6) return true;
    if (d.fanuc && !d.haas && m == 29) return true; // rigid tapping
    return false;
}

bool knownLetter(const Dialect& d, char c) {
    const std::string base = "GMXYZIJKFSTPRN";
    if (base.find(c) != std::string::npos) return true;
    if (d.grbl) return false;
    if (c == 'H' || c == 'Q' || c == 'A' || c == 'B' || c == 'C') return true;
    return c == 'O' && d.fanuc;
}

struct Word {
    char letter;
    double value;
    bool dot;
};

std::string fmt(double v) {
    char b[48];
    std::snprintf(b, sizeof b, "%.4g", v);
    return b;
}

struct P3 {
    double x, y, z;
};

double segDist(const P3& p, const P3& a, const P3& b) {
    const double dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
    const double l2 = dx * dx + dy * dy + dz * dz;
    double t = l2 > 0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy + (p.z - a.z) * dz) / l2 : 0.0;
    t = std::clamp(t, 0.0, 1.0);
    const double ex = a.x + dx * t - p.x, ey = a.y + dy * t - p.y, ez = a.z + dz * t - p.z;
    return std::sqrt(ex * ex + ey * ey + ez * ez);
}

// Segments in a grid of cells, each listed in the cells within `pad` of it.
class SegmentGrid {
public:
    SegmentGrid(const std::vector<std::pair<P3, P3>>& segs, double pad) : m_segs(segs), m_cell(std::max(2.0 * pad, 0.5)) {
        for (int i = 0; i < static_cast<int>(segs.size()); ++i) {
            const P3 &a = segs[static_cast<std::size_t>(i)].first, &b = segs[static_cast<std::size_t>(i)].second;
            const double len = std::sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y) + (b.z - a.z) * (b.z - a.z));
            const int n = std::max(1, static_cast<int>(std::ceil(len / (m_cell * 0.5))));
            for (int k = 0; k <= n; ++k) {
                const double t = static_cast<double>(k) / n;
                const long long cx = cellOf(a.x + (b.x - a.x) * t), cy = cellOf(a.y + (b.y - a.y) * t), cz = cellOf(a.z + (b.z - a.z) * t);
                for (long long dx = -1; dx <= 1; ++dx)
                    for (long long dy = -1; dy <= 1; ++dy)
                        for (long long dz = -1; dz <= 1; ++dz) {
                            std::vector<int>& v = m_grid[key(cx + dx, cy + dy, cz + dz)];
                            if (v.empty() || v.back() != i) v.push_back(i);
                        }
            }
        }
    }
    // The distance to the nearest segment, if it is within the pad (else +inf).
    double distance(const P3& p, int* nearest = nullptr) const {
        const auto it = m_grid.find(key(cellOf(p.x), cellOf(p.y), cellOf(p.z)));
        double best = std::numeric_limits<double>::infinity();
        if (it == m_grid.end()) return best;
        for (int i : it->second) {
            const double d = segDist(p, m_segs[static_cast<std::size_t>(i)].first, m_segs[static_cast<std::size_t>(i)].second);
            if (d < best) {
                best = d;
                if (nearest) *nearest = i;
            }
        }
        return best;
    }

private:
    long long cellOf(double v) const { return static_cast<long long>(std::floor(v / m_cell)); }
    static long long key(long long x, long long y, long long z) { return (x * 73856093LL) ^ (y * 19349663LL) ^ (z * 83492791LL); }
    const std::vector<std::pair<P3, P3>>& m_segs;
    double m_cell;
    std::unordered_map<long long, std::vector<int>> m_grid;
};

} // namespace

std::size_t GcodeBackplot::errors() const {
    return static_cast<std::size_t>(std::count_if(issues.begin(), issues.end(), [](const GcodeIssue& i) { return i.error; }));
}
std::size_t GcodeBackplot::warnings() const { return issues.size() - errors(); }

GcodeBackplot backplotGcode(const std::string& program, const std::string& dialect, const CamMachine& mc) {
    GcodeBackplot out;
    Dialect d;
    if (!dialectOf(dialect, &d)) {
        out.issues.push_back({0, true, "Unknown dialect '" + dialect + "' (fanuc, haas, grbl, linuxcnc)"});
        return out;
    }
    // Each kind of fault is listed for its first few lines, then counted.
    std::map<std::string, int> seen;
    std::map<std::string, bool> kindError;
    auto report = [&](int line, bool error, const std::string& kind, const std::string& text) {
        kindError[kind] = error;
        if (++seen[kind] <= 5) out.issues.push_back({line, error, text});
    };

    // The control's state.
    double x = kNaN, y = kNaN, z = kNaN, a = 0, c = 0, b = 0; // work coordinates; NaN: unknown (at machine home)
    bool tcp = false; // tool centre point control: X Y Z are the tip on the part
    int motion = -1, plane = 17, units = 0;
    double scale = 1.0;
    bool absolute = true, inverseTime = false, feedSet = false, wcs = false;
    double feed = 0, rpm = 0;
    int spindle = 0; // 0 off, 3 / 4 on
    int selected = -1, loaded = 0;
    bool lengthComp = false, needLengthComp = false, ended = false, unitsReported = false;
    // Canned cycles.
    int cycle = 0; // 0: none, else 73 / 81..85
    double cR = kNaN, cZ = kNaN, cQ = 0, cP = 0, cInit = kNaN;
    bool g98 = true, cycleFresh = true;

    auto segment = [&](double x1, double y1, double z1, bool rapid, int line) {
        if (std::isnan(x) || std::isnan(y) || std::isnan(z) || std::isnan(x1) || std::isnan(y1) || std::isnan(z1)) return;
        GcodeBackplot::Segment s;
        s.x0 = x, s.y0 = y, s.z0 = z, s.x1 = x1, s.y1 = y1, s.z1 = z1;
        s.a = a, s.c = c, s.b = b;
        s.tcp = tcp;
        s.rapid = rapid;
        s.line = line;
        s.tool = loaded;
        const double len = std::sqrt((x1 - x) * (x1 - x) + (y1 - y) * (y1 - y) + (z1 - z) * (z1 - z));
        if (rapid) out.seconds += len / std::max(mc.rapid, 1.0) * 60.0;
        else if (!inverseTime && feed > 0) out.seconds += len / (feed * scale) * 60.0;
        out.segments.push_back(s);
    };
    auto moveTo = [&](double x1, double y1, double z1, bool rapid, int line) {
        segment(x1, y1, z1, rapid, line);
        x = x1, y = y1, z = z1;
    };
    // What a cutting move needs: a place to start, the spindle turning and a feed.
    auto cutChecks = [&](int line, bool feedOnLine) {
        if (std::isnan(x) || std::isnan(y) || std::isnan(z))
            report(line, true, "cutunknown", "Cutting from an unknown position (the tool was last sent to machine home)");
        if (spindle == 0 || rpm <= 0) report(line, true, "spindleoff", "Cutting with the spindle off");
        if (inverseTime && !feedOnLine) report(line, true, "g93f", "G93 (inverse time) needs an F on every cutting move");
        if (!inverseTime && (!feedSet || feed <= 0)) report(line, true, "nofeed", "Cutting with no feed rate set");
    };

    int lineNo = 0;
    std::size_t pos = 0;
    while (pos <= program.size()) {
        std::size_t eol = program.find('\n', pos);
        if (eol == std::string::npos) eol = program.size();
        std::string text = program.substr(pos, eol - pos);
        pos = eol + 1;
        ++lineNo;
        if (!text.empty() && text.back() == '\r') text.pop_back();
        // Words, without comments.
        std::vector<Word> words;
        bool bad = false;
        for (std::size_t i = 0; i < text.size();) {
            const char ch = text[i];
            if (std::isspace(static_cast<unsigned char>(ch))) {
                ++i;
                continue;
            }
            if (ch == '(') {
                const std::size_t close = text.find(')', i);
                if (close == std::string::npos) {
                    report(lineNo, true, "comment", "A comment that isn't closed");
                    break;
                }
                i = close + 1;
                continue;
            }
            if (ch == ';') break;
            if (ch == '%' && words.empty()) {
                ++i;
                continue;
            }
            const char letter = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
            if (!std::isalpha(static_cast<unsigned char>(letter))) {
                report(lineNo, true, "char", std::string("Unexpected '") + ch + "'");
                bad = true;
                break;
            }
            ++i;
            while (i < text.size() && text[i] == ' ') ++i;
            const std::size_t start = i;
            if (i < text.size() && (text[i] == '+' || text[i] == '-')) ++i;
            bool digits = false, dot = false;
            while (i < text.size() && (std::isdigit(static_cast<unsigned char>(text[i])) || text[i] == '.')) {
                if (text[i] == '.') {
                    if (dot) break;
                    dot = true;
                } else digits = true;
                ++i;
            }
            if (!digits) {
                report(lineNo, true, "novalue", std::string(1, letter) + " without a value");
                bad = true;
                break;
            }
            words.push_back({letter, std::strtod(text.substr(start, i - start).c_str(), nullptr), dot});
        }
        if (bad || words.empty()) continue;
        ++out.lines;
        if (ended) {
            report(lineNo, false, "afterend", "Words after the program's end (M30 / M2) are never run");
            continue;
        }
        // Letters the dialect knows; one of each (G and M may repeat); lengths with a point.
        std::map<char, double> w;
        std::vector<int> gs, ms;
        for (const Word& wd : words) {
            if (!knownLetter(d, wd.letter)) {
                report(lineNo, true, std::string("letter") + wd.letter, std::string(1, wd.letter) + " words aren't understood by " + d.key);
                continue;
            }
            if (wd.letter == 'G') {
                const int g10 = static_cast<int>(std::lround(wd.value * 10));
                if (!knownG(d, g10)) report(lineNo, true, "g" + std::to_string(g10), "G" + fmt(wd.value) + " isn't a " + d.key + " code");
                else gs.push_back(g10);
                continue;
            }
            if (wd.letter == 'M') {
                const int m = static_cast<int>(std::lround(wd.value));
                if (!knownM(d, m)) report(lineNo, true, "m" + std::to_string(m), "M" + fmt(wd.value) + " isn't a " + d.key + " code");
                else ms.push_back(m);
                continue;
            }
            if (w.count(wd.letter)) report(lineNo, true, "twice", std::string("Two ") + wd.letter + " words on one line");
            if (d.fanuc && !wd.dot && std::string("XYZIJKRQABC").find(wd.letter) != std::string::npos && wd.value != 0)
                report(lineNo, true, "nodot",
                       std::string(1, wd.letter) + fmt(wd.value) + " has no decimal point: a Fanuc-style control reads it in thousandths of a mm");
            w[wd.letter] = wd.value;
        }
        auto has = [&](char l) { return w.count(l) > 0; };
        auto hasG = [&](int g10) { return std::find(gs.begin(), gs.end(), g10) != gs.end(); };
        auto hasM = [&](int m) { return std::find(ms.begin(), ms.end(), m) != ms.end(); };

        // Modes, in the order a control applies them.
        if (hasG(930)) inverseTime = true;
        if (hasG(940)) inverseTime = false;
        if (has('F')) {
            feed = w['F'];
            feedSet = true;
            if (!inverseTime && feed * scale > mc.maxFeed + 0.5)
                report(lineNo, true, "maxfeed", "F" + fmt(feed) + " is over the machine's " + fmt(mc.maxFeed) + " mm/min");
            if (!inverseTime) out.maxFeed = std::max(out.maxFeed, feed * scale);
        }
        if (has('S')) {
            rpm = w['S'];
            out.maxRpm = std::max(out.maxRpm, rpm);
            if (rpm > mc.maxRpm + 0.5) report(lineNo, true, "maxrpm", "S" + fmt(rpm) + " is over the machine's " + fmt(mc.maxRpm) + " rpm");
        }
        if (has('T')) {
            selected = static_cast<int>(std::lround(w['T']));
            if (mc.toolPockets > 0 && (selected < 1 || selected > mc.toolPockets))
                report(lineNo, true, "pocket", "T" + std::to_string(selected) + ": the tool changer has pockets 1 to " + std::to_string(mc.toolPockets));
        }
        if (hasM(6)) {
            if (selected < 0) report(lineNo, true, "m6", "M6 with no tool selected (T)");
            else {
                loaded = selected;
                out.tools.push_back(loaded);
            }
            spindle = 0; // the changer stops it
            lengthComp = false, tcp = false;
            needLengthComp = true;
        }
        if (hasM(3) || hasM(4)) {
            spindle = hasM(3) ? 3 : 4;
            if (rpm <= 0) report(lineNo, true, "nospeed", "The spindle started with no speed (S)");
        }
        if (hasM(5)) spindle = 0;
        if (hasG(40)) {
            const double p = has('P') ? w['P'] : 0;
            out.seconds += d.fanuc && !d.haas ? p / 1000.0 : p;
        }
        for (int g : {170, 180, 190})
            if (hasG(g)) plane = g / 10;
        if (hasG(200) || hasG(210)) {
            units = hasG(200) ? 20 : 21;
            scale = units == 20 ? 25.4 : 1.0;
        }
        if (hasG(490)) lengthComp = false, tcp = false;
        if (hasG(430) || hasG(434) || hasG(2340)) {
            tcp = !hasG(430);
            const int h = has('H') ? static_cast<int>(std::lround(w['H'])) : -1;
            if (h < 0) report(lineNo, true, "g43h", "G43 without an H (the length offset to use)");
            else if (h != loaded) report(lineNo, true, "g43t", "G43 H" + std::to_string(h) + " while tool " + std::to_string(loaded) + " is in the spindle");
            lengthComp = true;
            needLengthComp = false;
        }
        if (hasG(540)) wcs = true;
        if (hasG(900)) absolute = true;
        if (hasG(910)) absolute = false;
        if (hasG(980)) g98 = true;
        if (hasG(990)) g98 = false;

        // Non-modal returns to machine home.
        const bool axisWords = has('X') || has('Y') || has('Z') || has('A') || has('B') || has('C');
        if (hasG(280)) {
            // Through the point given (absolute or incremental), then home on those axes.
            double ix = x, iy = y, iz = z;
            if (has('X')) ix = absolute ? w['X'] * scale : x + w['X'] * scale;
            if (has('Y')) iy = absolute ? w['Y'] * scale : y + w['Y'] * scale;
            if (has('Z')) iz = absolute ? w['Z'] * scale : z + w['Z'] * scale;
            if (has('Z') && !std::isnan(z) && iz < z - 1e-6)
                report(lineNo, true, "g28down", "G28 goes down to Z" + fmt(iz) + " on its way home (use G91 G28 Z0)");
            moveTo(ix, iy, iz, true, lineNo);
            if (has('X')) x = kNaN;
            if (has('Y')) y = kNaN;
            if (has('Z')) z = kNaN;
            continue;
        }
        if (hasG(530)) { // a move in machine coordinates: where that is in the work frame isn't known here
            if (has('X')) x = kNaN;
            if (has('Y')) y = kNaN;
            if (has('Z')) z = kNaN;
            if (has('A')) a = w['A'];
            if (has('B')) b = w['B'];
            if (has('C')) c = w['C'];
            continue;
        }

        // The motion mode.
        bool newCycle = false;
        for (int g : gs) {
            if (g == 0 || g == 10 || g == 20 || g == 30 || g == 331) motion = g, cycle = 0;
            else if (g == 730 || (g >= 810 && g <= 850)) {
                if (cycle != g / 10) cycleFresh = true;
                cycle = g / 10;
                motion = g;
                newCycle = true;
            } else if (g == 800) {
                motion = -1;
                cycle = 0;
                cycleFresh = true;
            }
        }
        if (cycle && (has('R') || newCycle)) {
            if (has('R')) cR = w['R'] * scale;
            if (has('Z')) cZ = w['Z'] * scale;
            if (has('Q')) cQ = w['Q'] * scale;
            if (has('P')) cP = w['P'];
        } else if (cycle && has('Z')) cZ = w['Z'] * scale;

        // Rotary axes.
        auto rotary = [&](char axis) {
            if (!has(axis)) return;
            const double v = w[axis];
            const bool fitted = axis == 'A' ? mc.hasA : axis == 'B' ? mc.hasB : mc.hasC;
            if (!fitted) report(lineNo, true, std::string("no") + axis, std::string("The machine has no ") + axis + " axis");
            if (axis == 'A') {
                out.aLo = out.usesA ? std::min(out.aLo, v) : v;
                out.aHi = out.usesA ? std::max(out.aHi, v) : v;
                out.usesA = true;
            } else if (axis == 'B') {
                out.bLo = out.usesB ? std::min(out.bLo, v) : v;
                out.bHi = out.usesB ? std::max(out.bHi, v) : v;
                out.usesB = true;
            } else {
                out.cLo = out.usesC ? std::min(out.cLo, v) : v;
                out.cHi = out.usesC ? std::max(out.cHi, v) : v;
                out.usesC = true;
            }
        };
        rotary('A');
        rotary('B');
        rotary('C');

        if (axisWords || (cycle && newCycle)) {
            if (units == 0 && !unitsReported) {
                report(lineNo, true, "units", "Moves before the units are set (G21 for mm)");
                unitsReported = true;
            }
            if (!d.grbl && needLengthComp && !lengthComp && has('Z'))
                report(lineNo, true, "nolength", "Tool " + std::to_string(loaded) + " moves in Z without its length offset (G43 H" + std::to_string(loaded) + ")");
            double tx = x, ty = y, tz = z;
            if (has('X')) tx = absolute ? w['X'] * scale : x + w['X'] * scale;
            if (has('Y')) ty = absolute ? w['Y'] * scale : y + w['Y'] * scale;
            if (!cycle && has('Z')) tz = absolute ? w['Z'] * scale : z + w['Z'] * scale;
            if (has('A')) a = w['A'];
            if (has('B')) b = w['B'];
            if (has('C')) c = w['C'];
            if (motion == -1) {
                report(lineNo, true, "nomotion", "Axis words with no motion mode (G0 / G1)");
            } else if (motion == 0) {
                moveTo(tx, ty, tz, true, lineNo);
            } else if (motion == 10) {
                cutChecks(lineNo, has('F'));
                moveTo(tx, ty, tz, false, lineNo);
                if (inverseTime && has('F') && w['F'] > 0) out.seconds += 60.0 / w['F'];
            } else if (motion == 20 || motion == 30) {
                cutChecks(lineNo, has('F'));
                if (plane != 17) {
                    report(lineNo, true, "plane", "Only XY-plane (G17) arcs are checked");
                    x = tx, y = ty, z = tz;
                    continue;
                }
                double cx = kNaN, cy = kNaN;
                if (has('R')) {
                    const double r = w['R'] * scale, dx = tx - x, dy = ty - y, chord = std::hypot(dx, dy);
                    if (chord < 1e-9 || std::fabs(r) < chord / 2 - 1e-4) {
                        report(lineNo, true, "rarc", "R" + fmt(r) + " is too small for the arc's ends");
                        x = tx, y = ty, z = tz;
                        continue;
                    }
                    const double h = std::sqrt(std::max(0.0, r * r - chord * chord / 4));
                    // The centre on the side that makes the arc short (R > 0) or long (R < 0).
                    const double side = (motion == 20 ? -1.0 : 1.0) * (r > 0 ? 1.0 : -1.0);
                    cx = x + dx / 2 - dy / chord * h * side;
                    cy = y + dy / 2 + dx / chord * h * side;
                } else {
                    cx = x + (has('I') ? w['I'] * scale : 0.0);
                    cy = y + (has('J') ? w['J'] * scale : 0.0);
                }
                if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
                    x = tx, y = ty, z = tz;
                    continue;
                }
                const double r0 = std::hypot(x - cx, y - cy), r1 = std::hypot(tx - cx, ty - cy);
                if (std::fabs(r0 - r1) > 0.01)
                    report(lineNo, true, "radius", "The arc ends " + fmt(r1) + " mm from its centre but starts " + fmt(r0) + " mm from it (the control stops)");
                const double a0 = std::atan2(y - cy, x - cx), a1 = std::atan2(ty - cy, tx - cx);
                double sweep = a1 - a0;
                if (motion == 20) {
                    while (sweep >= 0) sweep -= 2 * kPiV;
                    if (std::hypot(tx - x, ty - y) < 1e-9) sweep = -2 * kPiV;
                } else {
                    while (sweep <= 0) sweep += 2 * kPiV;
                    if (std::hypot(tx - x, ty - y) < 1e-9) sweep = 2 * kPiV;
                    else if (sweep >= 2 * kPiV) sweep -= 2 * kPiV;
                }
                const double r = 0.5 * (r0 + r1);
                const double step = r > 0.002 ? 2 * std::acos(std::max(-1.0, 1 - 0.001 / r)) : kPiV / 8;
                const int n = std::clamp(static_cast<int>(std::ceil(std::fabs(sweep) / std::max(step, 1e-4))), 2, 4000);
                const double z0 = z;
                for (int k = 1; k <= n; ++k) {
                    const double t = static_cast<double>(k) / n, ang = a0 + sweep * t, rr = r0 + (r1 - r0) * t;
                    if (k == n) moveTo(tx, ty, tz, false, lineNo);
                    else moveTo(cx + rr * std::cos(ang), cy + rr * std::sin(ang), z0 + (tz - z0) * t, false, lineNo);
                }
                ++out.arcs;
            } else if (motion == 331) { // LinuxCNC rigid tap: down at the pitch, back out
                cutChecks(lineNo, true);
                const double z0 = z;
                if (has('Z')) moveTo(tx, ty, w['Z'] * scale, false, lineNo);
                moveTo(tx, ty, z0, false, lineNo);
                ++out.cycles;
            } else if (cycle) {
                // A hole: over at the current height, down to R, the cycle, back up.
                if (absolute == false) report(lineNo, true, "cycleinc", "Canned cycles are only checked in G90");
                if (cycleFresh || std::isnan(cInit)) {
                    cInit = z;
                    cycleFresh = false;
                }
                cutChecks(lineNo, true);
                if (std::isnan(cR) || std::isnan(cZ)) {
                    report(lineNo, true, "cyclerz", "A canned cycle without R or Z");
                    continue;
                }
                if (!(cR > cZ)) report(lineNo, true, "cycler", "The cycle's R" + fmt(cR) + " isn't above its bottom Z" + fmt(cZ));
                if (cInit < cR - 1e-6) report(lineNo, false, "cycleinit", "The cycle starts below its R level");
                if ((cycle == 83 || cycle == 73) && !(cQ > 0)) report(lineNo, true, "cycleq", "A peck cycle needs Q > 0");
                const double back = g98 ? std::max(cInit, cR) : cR;
                moveTo(tx, ty, z, true, lineNo);
                moveTo(tx, ty, cR, true, lineNo);
                switch (cycle) {
                case 83: { // peck: out to R after each, back down to just above the last depth
                    double depth = cR;
                    for (int guard = 0; depth > cZ + 1e-9 && guard < 100000; ++guard) {
                        const double next = std::max(depth - std::max(cQ, 1e-3), cZ);
                        if (depth < cR) moveTo(tx, ty, std::min(cR, depth + 0.5), true, lineNo);
                        moveTo(tx, ty, next, false, lineNo);
                        depth = next;
                        if (depth > cZ + 1e-9) moveTo(tx, ty, cR, true, lineNo);
                    }
                    break;
                }
                case 73: { // chip-break: a little back after each peck
                    double depth = cR;
                    for (int guard = 0; depth > cZ + 1e-9 && guard < 100000; ++guard) {
                        const double next = std::max(depth - std::max(cQ, 1e-3), cZ);
                        if (depth < cR) moveTo(tx, ty, depth + 0.1, true, lineNo);
                        moveTo(tx, ty, next, false, lineNo);
                        depth = next;
                    }
                    break;
                }
                case 84: // tap: in, reverse, out at the feed
                case 85: // bore: in and out at the feed
                    moveTo(tx, ty, cZ, false, lineNo);
                    moveTo(tx, ty, cR, false, lineNo);
                    break;
                default: // 81 drill, 82 drill and dwell
                    moveTo(tx, ty, cZ, false, lineNo);
                    if (cycle == 82) out.seconds += d.fanuc && !d.haas ? cP / 1000.0 : cP;
                }
                moveTo(tx, ty, back, true, lineNo);
                ++out.cycles;
            }
        }
        if (hasM(2) || hasM(30)) ended = true;
    }
    if (!ended) report(0, false, "noend", "The program has no end (M30 or M2)");
    if (!wcs) report(0, false, "nowcs", "No work offset is selected (G54): the control uses whichever was active");
    for (const auto& [kind, n] : seen)
        if (n > 5) out.issues.push_back({0, kindError[kind], "... and " + std::to_string(n - 5) + " more lines like that"});
    return out;
}

std::string CamPostCheck::summary() const {
    char b[200];
    std::snprintf(b, sizeof b, "checked: %zu error%s, %zu warning%s; the program's path is within %.4f mm of the toolpath (allowed %.4f)", errors,
                  errors == 1 ? "" : "s", warnings, warnings == 1 ? "" : "s", deviation, allowed);
    return b;
}

CamPostCheck checkPost(const CamJob& job, const std::string& post, const std::string& program) {
    CamPostCheck r;
    r.plot = backplotGcode(program, post, job.setup.machine);
    r.issues = r.plot.issues;
    const CamMachine& m = job.setup.machine;
    auto fail = [&](bool error, const std::string& text) { r.issues.push_back({0, error, text}); };

    // Travel: the tool's tip, and the spindle nose a tool's length above it.
    std::map<int, double> toolLength;
    double longest = 0;
    for (const CamTool& t : job.tools) {
        toolLength[t.number] = t.gauge() + t.holderLength;
        longest = std::max(longest, toolLength[t.number]);
    }
    double lo[3] = {kNaN, kNaN, kNaN}, hi[3] = {kNaN, kNaN, kNaN};
    auto extend = [&](int i, double v) {
        if (std::isnan(lo[i]) || v < lo[i]) lo[i] = v;
        if (std::isnan(hi[i]) || v > hi[i]) hi[i] = v;
    };
    for (const GcodeBackplot::Segment& s : r.plot.segments) {
        const auto it = toolLength.find(s.tool);
        const double len = it == toolLength.end() ? longest : it->second;
        if (s.tcp) {
            // The tip on the part: in the machine's frame the table's turn (about the work
            // zero, the rotary centre) moves it, and the nose is a tool's length up the
            // tilted axis.
            // With C in the head the part stays put and C swivels the tilted tool instead.
            const double cr = s.c * 3.14159265358979323846 / 180.0, br = s.b * 3.14159265358979323846 / 180.0;
            for (int e = 0; e < 2; ++e) {
                const double px = e ? s.x1 : s.x0, py = e ? s.y1 : s.y0, pz = e ? s.z1 : s.z0;
                if (m.cHead) {
                    extend(0, px + len * std::sin(br) * std::cos(cr));
                    extend(1, py + len * std::sin(br) * std::sin(cr));
                } else {
                    extend(0, px * std::cos(cr) - py * std::sin(cr) + len * std::sin(br));
                    extend(1, px * std::sin(cr) + py * std::cos(cr));
                }
                extend(2, pz + len * std::cos(br));
            }
            continue;
        }
        for (const double* p : {&s.x0, &s.x1}) extend(0, *p);
        for (const double* p : {&s.y0, &s.y1}) extend(1, *p);
        for (const double* p : {&s.z0, &s.z1}) extend(2, *p + len);
    }
    const char* axes = "XYZ";
    for (int i = 0; i < 3; ++i) {
        if (!m.hasTravel(i) || std::isnan(lo[i])) continue;
        const std::string what = i == 2 ? "the spindle nose (tip plus the tool's length)" : std::string("the tool");
        if (m.hasWorkOffset) {
            const double a = lo[i] + m.workOffset[i], b = hi[i] + m.workOffset[i];
            if (a < m.travelMin[i] - 1e-6 || b > m.travelMax[i] + 1e-6)
                fail(true, std::string(1, axes[i]) + " travel: " + what + " goes to " + fmt(a) + ".." + fmt(b) + " in machine coordinates, past the machine's " +
                               fmt(m.travelMin[i]) + ".." + fmt(m.travelMax[i]));
        } else if (hi[i] - lo[i] > m.travelMax[i] - m.travelMin[i] + 1e-6) {
            fail(true, std::string(1, axes[i]) + " travel: the program spans " + fmt(hi[i] - lo[i]) + " mm, more than the machine's " +
                           fmt(m.travelMax[i] - m.travelMin[i]) + " mm");
        }
    }
    if (r.plot.usesA && m.hasA && (r.plot.aLo < m.aMin - 1e-6 || r.plot.aHi > m.aMax + 1e-6))
        fail(true, "A turns to " + fmt(r.plot.aLo) + ".." + fmt(r.plot.aHi) + " deg, past the machine's " + fmt(m.aMin) + ".." + fmt(m.aMax));
    if (r.plot.usesB && m.hasB && (r.plot.bLo < m.bMin - 1e-6 || r.plot.bHi > m.bMax + 1e-6))
        fail(true, "B tilts to " + fmt(r.plot.bLo) + ".." + fmt(r.plot.bHi) + " deg, past the machine's " + fmt(m.bMin) + ".." + fmt(m.bMax));
    if (r.plot.usesC && m.hasC && (r.plot.cLo < m.cMin - 1e-6 || r.plot.cHi > m.cMax + 1e-6))
        fail(true, "C turns to " + fmt(r.plot.cLo) + ".." + fmt(r.plot.cHi) + " deg, past the machine's " + fmt(m.cMin) + ".." + fmt(m.cMax));

    // The program's cutting path against the toolpath, both ways.
    const Vec3 o = job.origin();
    std::vector<std::pair<P3, P3>> expected, programmed;
    for (const CamOperation& op : job.ops) {
        // Tool centre point moves are posted as the tip on the part.
        const bool onPart = (op.kind == CamOpKind::Surface5 || op.kind == CamOpKind::Swarf) && m.headTilts();
        auto at = [&](const CamMove& mv) { return onPart ? camToPart(job, mv.to, mv.a, mv.c) : mv.to; };
        for (std::size_t i = 1; i < op.moves.size(); ++i) {
            if (op.moves[i].kind == CamMove::Rapid) continue;
            const Vec3 p = at(op.moves[i - 1]), q = at(op.moves[i]);
            expected.push_back({{p.x - o.x + 0.0, p.y - o.y + 0.0, p.z - o.z + 0.0}, {q.x - o.x + 0.0, q.y - o.y + 0.0, q.z - o.z + 0.0}});
        }
    }
    for (const GcodeBackplot::Segment& s : r.plot.segments)
        if (!s.rapid) programmed.push_back({{s.x0, s.y0, s.z0}, {s.x1, s.y1, s.z1}});
    const bool fanucNumbers = post == "fanuc" || post == "haas";
    // Fitted arcs keep the points within the tolerance and bulge past the chords by up to
    // four times it; then the rounding of the numbers.
    r.allowed = 5.0 * std::max(job.setup.tolerance, 0.001) + (fanucNumbers ? 0.0005 : 0.002);
    const double pad = 2.0 * r.allowed;
    int worstLine = 0;
    bool missed = false;
    auto sweepAgainst = [&](const std::vector<std::pair<P3, P3>>& from, const std::vector<std::pair<P3, P3>>& to, bool fromProgram) {
        if (from.empty() || to.empty()) return;
        const SegmentGrid grid(to, pad);
        const double step = std::max(r.allowed, 0.25);
        for (std::size_t si = 0; si < from.size(); ++si) {
            const P3 &a = from[si].first, &b = from[si].second;
            const double len = std::sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y) + (b.z - a.z) * (b.z - a.z));
            const int n = std::max(1, static_cast<int>(std::ceil(len / step)));
            for (int k = 0; k <= n; ++k) {
                const double t = static_cast<double>(k) / n;
                const P3 p{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t};
                int nearest = -1;
                double dd = grid.distance(p, &nearest);
                if (std::isinf(dd)) dd = pad;
                if (dd > r.deviation) {
                    r.deviation = dd;
                    missed = !fromProgram;
                    if (fromProgram) worstLine = r.plot.segments.empty() ? 0 : [&] {
                        int seen = -1;
                        for (const GcodeBackplot::Segment& s : r.plot.segments)
                            if (!s.rapid && ++seen == static_cast<int>(si)) return s.line;
                        return 0;
                    }();
                    else if (nearest >= 0) {
                        int seen = -1;
                        worstLine = 0;
                        for (const GcodeBackplot::Segment& s : r.plot.segments)
                            if (!s.rapid && ++seen == nearest) worstLine = s.line;
                    }
                }
            }
        }
    };
    {
        sweepAgainst(programmed, expected, true);
        sweepAgainst(expected, programmed, false);
        if (expected.size() && programmed.empty()) r.deviation = pad, missed = true;
        if (r.deviation > r.allowed + 1e-9)
            r.issues.push_back({worstLine, true,
                                missed ? "The program misses part of the toolpath: " + std::string(r.deviation >= pad ? "more than " : "") + fmt(r.deviation) +
                                             " mm from the nearest cut"
                                       : "The program cuts " + std::string(r.deviation >= pad ? "more than " : "") + fmt(r.deviation) +
                                             " mm off the toolpath"});
    }

    // Rapids across the stock below its top that the post added (3-axis jobs: the stock
    // stays put). The toolpath's own rapids, inside what it has already cut, are the
    // simulation's to judge.
    if (job.setup.hasStock && !job.multiAxis()) {
        std::vector<std::pair<P3, P3>> ownRapids;
        for (const CamOperation& op : job.ops)
            for (std::size_t i = 1; i < op.moves.size(); ++i)
                if (op.moves[i].kind == CamMove::Rapid) {
                    const Vec3 &p = op.moves[i - 1].to, &q = op.moves[i].to;
                    ownRapids.push_back({{p.x - o.x + 0.0, p.y - o.y + 0.0, p.z - o.z + 0.0}, {q.x - o.x + 0.0, q.y - o.y + 0.0, q.z - o.z + 0.0}});
                }
        const SegmentGrid own(ownRapids, pad);
        auto fromToolpath = [&](const GcodeBackplot::Segment& s) {
            const double len = std::sqrt((s.x1 - s.x0) * (s.x1 - s.x0) + (s.y1 - s.y0) * (s.y1 - s.y0) + (s.z1 - s.z0) * (s.z1 - s.z0));
            const int n = std::max(1, static_cast<int>(std::ceil(len / 0.5)));
            for (int k = 0; k <= n; ++k) {
                const double t = static_cast<double>(k) / n;
                if (own.distance({s.x0 + (s.x1 - s.x0) * t, s.y0 + (s.y1 - s.y0) * t, s.z0 + (s.z1 - s.z0) * t}) > r.allowed) return false;
            }
            return true;
        };
        double radius = 0;
        for (const CamTool& t : job.tools) radius = std::max(radius, t.diameter * 0.5);
        const double x0 = job.setup.stockMin.x - o.x - radius, x1 = job.setup.stockMax.x - o.x + radius;
        const double y0 = job.setup.stockMin.y - o.y - radius, y1 = job.setup.stockMax.y - o.y + radius;
        const double top = job.setup.stockMax.z - o.z;
        std::size_t crashes = 0;
        int first = 0;
        for (const GcodeBackplot::Segment& s : r.plot.segments) {
            if (!s.rapid || std::hypot(s.x1 - s.x0, s.y1 - s.y0) < 0.01 || std::min(s.z0, s.z1) >= top - 0.01) continue;
            // Does the move's XY line cross the stock's outline (grown by the tool)?
            double t0 = 0, t1 = 1;
            const double dx = s.x1 - s.x0, dy = s.y1 - s.y0;
            auto clip = [&](double p, double q) {
                if (std::fabs(p) < 1e-12) return q >= 0;
                const double t = q / p;
                if (p < 0) t0 = std::max(t0, t);
                else t1 = std::min(t1, t);
                return t0 <= t1;
            };
            if (clip(-dx, s.x0 - x0) && clip(dx, x1 - s.x0) && clip(-dy, s.y0 - y0) && clip(dy, y1 - s.y0) && !fromToolpath(s)) {
                if (!crashes++) first = s.line;
            }
        }
        if (crashes)
            r.issues.push_back({first, true, std::to_string(crashes) + " rapid move" + (crashes == 1 ? "" : "s") + " across the stock below its top"});
    }
    for (const GcodeIssue& i : r.issues) (i.error ? r.errors : r.warnings) += 1;
    return r;
}

} // namespace sicdm

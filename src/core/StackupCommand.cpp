// STACK: tolerance stack-ups: dimensions by hand, from the model's bodies or the drawing,
// analysed by worst case, RSS and Monte Carlo against the spec.

#include "core/StackupCommand.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Stackup.hpp"
#include "core/Units.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <optional>

namespace sicdm {

namespace {

std::string lowerOf(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

std::string num(double v, int digits = 5) {
    char b[48];
    std::snprintf(b, sizeof b, "%.*g", digits, std::fabs(v) < 5e-12 ? 0.0 : v);
    return b;
}

std::string dev(double v) { return std::fabs(v) < 5e-12 ? "0" : (v > 0 ? "+" : "") + num(v); }

bool writeText(const std::string& path, const std::string& text) {
    std::ofstream f(path, std::ios::binary);
    if (!f) return false;
    f << text;
    return static_cast<bool>(f);
}

ToleranceStack* findStack(Document& doc, const std::string& name) {
    for (ToleranceStack& s : doc.state.stacks)
        if (lowerOf(s.name) == lowerOf(name)) return &s;
    return nullptr;
}

// X, Y, Z, -X ... or x,y,z.
std::optional<Vec3> parseDir(const std::string& w) {
    const std::string u = toUpper(w);
    const bool neg = !u.empty() && u[0] == '-';
    const std::string a = neg ? u.substr(1) : u;
    Vec3 d{0, 0, 0};
    if (a == "X") d = {1, 0, 0};
    else if (a == "Y") d = {0, 1, 0};
    else if (a == "Z") d = {0, 0, 1};
    else if (const auto v = parseVec3(w)) return length(*v) > 0 ? std::optional<Vec3>(*v) : std::nullopt;
    else return std::nullopt;
    return neg ? d * -1.0f : d;
}

std::string dirText(const Vec3& d) {
    if (d.y == 0 && d.z == 0 && d.x != 0) return d.x > 0 ? "X" : "-X";
    if (d.x == 0 && d.z == 0 && d.y != 0) return d.y > 0 ? "Y" : "-Y";
    if (d.x == 0 && d.y == 0 && d.z != 0) return d.z > 0 ? "Z" : "-Z";
    return num(d.x) + "," + num(d.y) + "," + num(d.z);
}

// A plain number, read as a double (a factor: SENS, CP).
std::optional<double> parseFactor(const std::string& w) {
    if (w.empty()) return std::nullopt;
    char* end = nullptr;
    const double v = std::strtod(w.c_str(), &end);
    if (end != w.c_str() + w.size() || !std::isfinite(v)) return std::nullopt;
    return v;
}

// A deviation typed with its sign: +0.1, -0.05 (a bare number reads as +).
bool signedWord(const std::string& w) { return !w.empty() && (w[0] == '+' || w[0] == '-'); }

// The options an item takes after ADD or EDIT, from args[i] on. What isn't an option goes to
// `loose` (the label and the nominal).
struct ItemOptions {
    std::optional<std::string> name;
    std::optional<double> nominal;
    std::optional<std::pair<double, double>> tol; // upper, lower
    std::optional<std::pair<double, double>> limits; // high, low
    std::optional<std::string> fit;
    std::optional<char> general;
    std::optional<int> sign;
    std::optional<double> sens;
    std::optional<StackDist> dist;
    std::optional<double> cp;
    std::optional<Vec3> dir;     // a vector stack's length: its direction
    std::optional<bool> angle;   // an angle (degrees)
    std::optional<Vec3> axis;    // ... about this
    std::optional<double> zone;  // a geometric tolerance's zone ...
    std::optional<double> bonus; // ... the feature's size range a material condition adds
    std::optional<char> material; // 'M', 'L', or 0 (RFS)
};

bool readItemOptions(const std::vector<std::string>& a, const std::vector<bool>& quoted, std::size_t i, ItemOptions& o, std::vector<std::string>& loose,
                     std::string& why, bool angleItem = false) {
    auto unq = [&](std::size_t k) { return !(k < quoted.size() && quoted[k]); };
    // An ANGLE's size and tolerance are degrees: plain numbers, not lengths in the units.
    bool degrees = angleItem;
    for (std::size_t k = i; k < a.size(); ++k) degrees = degrees || (unq(k) && lowerOf(a[k]) == "angle");
    auto len = [&](std::size_t k) -> std::optional<double> {
        if (k >= a.size()) return std::nullopt;
        std::string w = a[k];
        if (w.rfind("+-", 0) == 0) w = w.substr(2);
        else if (w.rfind("\xC2\xB1", 0) == 0) w = w.substr(2); // the plus-minus sign
        else if (!w.empty() && w[0] == '+') w = w.substr(1);
        if (degrees) {
            const auto v = parseNumber(w);
            return v ? std::optional<double>(*v) : std::nullopt;
        }
        return parseLength(w);
    };
    for (; i < a.size(); ++i) {
        const std::string k = unq(i) ? lowerOf(a[i]) : std::string();
        if (k == "tol" || k == "tolerance") {
            const auto t = len(i + 1);
            if (!t) return why = "TOL needs a tolerance: TOL 0.1 (+-), or TOL +0.1 -0.05 (upper, lower)", false;
            ++i;
            const bool plusMinus = a[i].rfind("+-", 0) == 0 || a[i].rfind("\xC2\xB1", 0) == 0;
            if (!plusMinus && i + 1 < a.size() && unq(i + 1) && signedWord(a[i + 1]) && len(i + 1)) {
                const double up = *t, lo = *len(i + 1);
                ++i;
                o.tol = {std::max(up, lo), std::min(up, lo)};
            } else {
                if (*t < 0) return why = "A +- tolerance is positive (TOL 0.1; TOL +0 -0.1 for one side)", false;
                o.tol = {*t, -*t};
            }
            o.fit.reset(), o.general.reset(), o.limits.reset();
        } else if (k == "limits") {
            const auto hi = len(i + 1), lo = len(i + 2);
            if (!hi || !lo) return why = "LIMITS needs the greatest and least size: LIMITS 20.05 19.95", false;
            i += 2;
            o.limits = {std::max(*hi, *lo), std::min(*hi, *lo)};
            o.tol.reset(), o.fit.reset(), o.general.reset();
        } else if (k == "fit") {
            if (i + 1 >= a.size()) return why = "FIT needs an ISO 286 code: FIT H7, FIT g6", false;
            o.fit = a[++i];
            o.tol.reset(), o.general.reset(), o.limits.reset();
        } else if (k == "general") {
            if (i + 1 >= a.size() || a[i + 1].size() != 1) return why = "GENERAL needs an ISO 2768-1 class: f, m, c or v", false;
            o.general = static_cast<char>(std::tolower(static_cast<unsigned char>(a[++i][0])));
            o.tol.reset(), o.fit.reset(), o.limits.reset();
        } else if (k == "minus" || k == "subtract") {
            o.sign = -1;
        } else if (k == "plus" || k == "add") {
            o.sign = 1;
        } else if (k == "sens" || k == "sensitivity") {
            const auto s = i + 1 < a.size() ? parseFactor(a[i + 1]) : std::nullopt;
            if (!s || *s == 0) return why = "SENS needs a factor (not 0): SENS 0.5 for a 2:1 lever", false;
            o.sens = *s, ++i;
        } else if (k == "normal") {
            o.dist = StackDist::Normal;
        } else if (k == "uniform") {
            o.dist = StackDist::Uniform;
        } else if (k == "triangular") {
            o.dist = StackDist::Triangular;
        } else if (k == "cp") {
            const auto c = i + 1 < a.size() ? parseFactor(a[i + 1]) : std::nullopt;
            if (!c || !(*c > 0)) return why = "CP needs the process capability (> 0): CP 1.33", false;
            o.cp = *c, ++i, o.dist = StackDist::Normal;
        } else if (k == "dir" || k == "direction") {
            const auto d = i + 1 < a.size() ? parseDir(a[i + 1]) : std::nullopt;
            if (!d) return why = "DIR needs X, Y, Z (-X...) or x,y,z: the length's direction in the loop", false;
            o.dir = *d, o.angle = false, ++i;
        } else if (k == "angle") {
            o.angle = true;
        } else if (k == "axis") {
            const auto d = i + 1 < a.size() ? parseDir(a[i + 1]) : std::nullopt;
            if (!d) return why = "AXIS needs X, Y, Z or x,y,z: what the angle turns about", false;
            o.axis = *d, ++i;
        } else if (k == "name" || k == "label") {
            if (i + 1 >= a.size()) return why = "NAME needs the dimension's name", false;
            o.name = a[++i];
        } else if (k == "nominal") {
            const auto v = len(i + 1);
            if (!v) return why = "NOMINAL needs a size", false;
            o.nominal = *v, ++i;
        } else if (k == "zone") {
            const auto v = len(i + 1);
            if (!v || !(*v > 0)) return why = "ZONE needs the geometric tolerance (above 0)", false;
            o.zone = *v, ++i;
        } else if (k == "bonus") {
            const auto v = len(i + 1);
            if (!v || *v < 0) return why = "BONUS needs the feature's size range (LMC - MMC)", false;
            o.bonus = *v, ++i;
        } else if (k == "mmc" || k == "lmc" || k == "rfs") {
            o.material = k == "mmc" ? 'M' : k == "lmc" ? 'L' : 0;
        } else {
            loose.push_back(a[i]);
        }
    }
    return true;
}

// Applies options to an item (its nominal set first, as the tolerances may follow it). A
// new typed item without a size takes the middle of its LIMITS.
bool applyItemOptions(StackItem& it, const ItemOptions& o, std::string& why, bool adding) {
    if (o.name) it.name = *o.name;
    if (o.nominal) {
        it.nominal = *o.nominal;
        if (it.nominal < 0) it.nominal = -it.nominal, it.sign = -it.sign; // a negative size takes from the result
    }
    if (o.sign) it.sign = *o.sign;
    if (o.sens) it.sens = *o.sens;
    if (o.dist) it.dist = *o.dist;
    if (o.cp) it.cp = *o.cp;
    if (o.dir) it.dir = *o.dir;
    if (o.angle) it.angle = *o.angle;
    if (o.axis) it.axis = *o.axis;
    if (o.zone) it.zone = *o.zone;
    if (o.bonus) it.bonus = *o.bonus;
    if (o.material) it.material = *o.material;
    if (!it.gdt.empty() && (o.zone || o.bonus || o.material || adding)) {
        if (o.material && *o.material && !o.bonus && !adding && it.bonus == 0)
            return why = "With MMC or LMC, give the feature's size range too: BONUS 0.1", false;
        if (!stackItemFromGdt(it, &why)) return false;
        it.frame = o.zone || o.bonus || o.material ? -1 : it.frame; // typed over: no longer the drawing's
    }
    if ((o.tol || o.limits || o.fit || o.general || o.nominal) && !it.gdt.empty())
        it.gdt.clear(), it.zone = 0, it.bonus = 0, it.material = 0, it.diametral = false, it.frame = -1; // a size again
    if (o.tol) it.upper = o.tol->first, it.lower = o.tol->second, it.fit.clear(), it.general = 0;
    if (o.limits) {
        if (adding && !o.nominal && it.body.empty() && it.dim < 0) it.nominal = 0.5 * (o.limits->first + o.limits->second);
        it.upper = o.limits->first - it.nominal, it.lower = o.limits->second - it.nominal;
        it.fit.clear(), it.general = 0;
    }
    if (o.fit) it.fit = *o.fit, it.general = 0;
    if (o.general) it.general = *o.general, it.fit.clear();
    if (!it.fit.empty() && !stackItemFromFit(it, &why)) return false;
    if (it.general && !stackItemFromGeneral(it, &why)) return false;
    return true;
}

std::string itemLine(const StackItem& it, std::size_t index, bool vector = false) {
    std::string l = std::to_string(index + 1) + ". " + (it.sign < 0 ? "- " : "+ ") + it.name + " " + num(it.nominal) + " " + dev(it.upper) + "/" + dev(it.lower);
    if (!it.gdt.empty()) {
        l += " (" + it.gdt + " " + (it.diametral ? "\xC3\x98" : "") + num(it.zone);
        if (it.material) l += std::string(" ") + it.material + ", bonus " + num(it.bonus);
        l += it.frame >= 0 ? ", frame " + std::to_string(it.frame + 1) + ")" : ")";
    } else if (!it.fit.empty()) l += " (" + it.fit + ")";
    else if (it.general) l += std::string(" (ISO 2768-") + it.general + ")";
    if (it.angle) l += " deg, turning about " + dirText(it.axis);
    else if (vector) l += " along " + dirText(it.dir);
    return l;
}

// The quick result after a change: worst case and RSS (no Monte Carlo).
std::string summary(const ToleranceStack& st) {
    ToleranceStack quick = st;
    quick.samples = 0;
    const StackResult r = analyzeStack(quick);
    std::string s = st.name + ": nominal " + num(r.nominal) + ", worst case " + num(r.wcLow) + " to " + num(r.wcHigh) + ", RSS " + num(r.rssLow) + " to " +
                    num(r.rssHigh) + " mm";
    if (st.hasLow || st.hasHigh) s += std::string(r.wcPass ? "; fits the spec" : r.rssPass ? "; fits the spec by RSS only" : "; outside the spec");
    return s;
}

bool readSpec(const std::vector<std::string>& a, std::size_t& i, ToleranceStack& st, std::string& why) {
    const std::string k = lowerOf(a[i]);
    auto len = [&](std::size_t j) { return j < a.size() ? parseLength(a[j]) : std::nullopt; };
    if (k == "spec") {
        if (i + 1 < a.size() && lowerOf(a[i + 1]) == "none") {
            st.hasLow = st.hasHigh = false, ++i;
            return true;
        }
        const auto lo = len(i + 1), hi = len(i + 2);
        if (!lo || !hi) return why = "SPEC needs the least and greatest result: SPEC 0.1 0.5 (or MIN, MAX, NONE)", false;
        st.hasLow = st.hasHigh = true, st.low = std::min(*lo, *hi), st.high = std::max(*lo, *hi), i += 2;
        return true;
    }
    const auto v = len(i + 1);
    if (!v) return why = toUpper(k) + " needs a size", false;
    (k == "min" ? st.hasLow : st.hasHigh) = true;
    (k == "min" ? st.low : st.high) = *v, ++i;
    if (st.hasLow && st.hasHigh && st.low > st.high) return why = "The spec's MIN is above its MAX", false;
    return true;
}

bool readGap(const Document& doc, const std::vector<std::string>& a, std::size_t& i, ToleranceStack& st, std::string& why) {
    if (i + 1 < a.size() && lowerOf(a[i + 1]) == "none") {
        st.gapFrom.clear(), st.gapTo.clear(), ++i;
        return true;
    }
    if (i + 4 >= a.size() || lowerOf(a[i + 3]) != "along") return why = "GAP needs two bodies and a direction: GAP Housing Lid ALONG Z (or GAP NONE)", false;
    const auto d = parseDir(a[i + 4]);
    if (!d) return why = "ALONG needs X, Y, Z (-X...) or x,y,z", false;
    double g = 0;
    if (!bodyGapAlong(doc.scene, a[i + 1], a[i + 2], *d, g, &why)) return false;
    st.gapFrom = a[i + 1], st.gapTo = a[i + 2], st.gapAlong = *d, i += 4;
    return true;
}

} // namespace

// VECTOR / LINEAR, MEASURE dir, DISTANCE, at args[i] (moved on past what it reads).
bool readMode(const std::vector<std::string>& a, std::size_t& i, ToleranceStack& st, std::string& why) {
    const std::string k = lowerOf(a[i]);
    if (k == "vector") {
        st.vector = true;
    } else if (k == "linear") {
        st.vector = false;
    } else if (k == "distance") {
        st.vector = true, st.measure = {0, 0, 0};
    } else { // measure
        const auto d = i + 1 < a.size() ? parseDir(a[i + 1]) : std::nullopt;
        if (!d) return why = "MEASURE needs the direction the result is taken along: X, Y, Z or x,y,z (DISTANCE: the straight distance)", false;
        st.vector = true, st.measure = *d, ++i;
    }
    return true;
}

CommandOutput runStackCommand(Document& doc, const std::vector<std::string>& a, const std::vector<bool>& quoted) {
    CommandOutput out;
    auto unq = [&](std::size_t k) { return !(k < quoted.size() && quoted[k]); };
    std::vector<ToleranceStack>& stacks = doc.state.stacks;
    if (a.empty() || (a.size() == 1 && unq(0) && lowerOf(a[0]) == "list")) {
        if (stacks.empty()) {
            out.say("No tolerance stacks (STACK NEW name, then STACK name ADD ...)");
            return out;
        }
        for (const ToleranceStack& st : stacks) out.say(summary(st) + " (" + std::to_string(st.items.size()) + " dimensions)");
        return out;
    }
    std::string why;
    // NEW
    if (unq(0) && lowerOf(a[0]) == "new") {
        if (a.size() < 2) return out.fail("STACK NEW needs the stack's name");
        if (findStack(doc, a[1])) return out.fail("There is already a stack named " + a[1]);
        ToleranceStack st;
        st.name = a[1];
        for (std::size_t i = 2; i < a.size(); ++i) {
            const std::string k = unq(i) ? lowerOf(a[i]) : std::string();
            if (k == "note") {
                if (i + 1 >= a.size()) return out.fail("NOTE needs a text");
                st.note = a[++i];
            } else if (k == "spec" || k == "min" || k == "max") {
                if (!readSpec(a, i, st, why)) return out.fail(why);
            } else if (k == "gap") {
                if (!readGap(doc, a, i, st, why)) return out.fail(why);
            } else if (k == "samples") {
                const auto n = i + 1 < a.size() ? parseNumber(a[i + 1]) : std::nullopt;
                if (!n || *n < 0 || *n > 2000000) return out.fail("SAMPLES is from 0 (no Monte Carlo) to 2000000");
                st.samples = static_cast<int>(std::lround(*n)), ++i;
            } else if (k == "vector" || k == "linear" || k == "measure" || k == "distance") {
                if (!readMode(a, i, st, why)) return out.fail(why);
            } else {
                return out.fail("STACK NEW doesn't take " + a[i] + " (NOTE, SPEC, MIN, MAX, GAP, SAMPLES, VECTOR, MEASURE)");
            }
        }
        stacks.push_back(st);
        out.say("Added the stack " + st.name + (st.hasLow || st.hasHigh ? "" : " (no spec yet: SPEC lo hi, MIN or MAX)") + ". Add its dimensions: STACK \"" + st.name +
                "\" ADD ...");
        return out;
    }
    ToleranceStack* st = findStack(doc, a[0]);
    if (!st) return out.fail("No stack named " + a[0] + (stacks.empty() ? " (STACK NEW name makes one)" : ""));
    const std::string sub = a.size() > 1 && unq(1) ? lowerOf(a[1]) : std::string("report");

    if (sub == "add") {
        StackItem it;
        std::size_t i = 2;
        std::string from;
        if (i < a.size() && unq(i) && lowerOf(a[i]) == "body") {
            if (i + 3 >= a.size() || lowerOf(a[i + 2]) != "along") return out.fail("ADD BODY needs the body and a direction: ADD BODY Spacer ALONG X");
            const auto d = parseDir(a[i + 3]);
            if (!d) return out.fail("ALONG needs X, Y, Z (-X...) or x,y,z");
            double s = 0;
            if (!bodySizeAlong(doc.scene, a[i + 1], *d, s, &why)) return out.fail(why);
            it.body = a[i + 1], it.along = *d, it.nominal = s;
            for (const Body& b : doc.scene.bodies())
                if (lowerOf(b.name) == lowerOf(it.body)) it.body = b.name;
            it.name = it.body + " (" + dirText(*d) + ")";
            it.general = 'm'; // until a tolerance is given: ISO 2768-m
            i += 4;
        } else if (i < a.size() && unq(i) && lowerOf(a[i]) == "dim") {
            const auto n = i + 1 < a.size() ? parseNumber(a[i + 1]) : std::nullopt;
            if (!n || *n < 1) return out.fail("ADD DIM needs the drawing dimension's number (1, 2 ...)");
            if (!stackItemFromDrawing(it, doc.state.drawing, static_cast<int>(std::lround(*n)) - 1, &why)) return out.fail(why);
            it.name = "Dimension " + std::to_string(static_cast<int>(std::lround(*n)));
            i += 2;
        } else if (i < a.size() && unq(i) && lowerOf(a[i]) == "frame") {
            const auto n = i + 1 < a.size() ? parseNumber(a[i + 1]) : std::nullopt;
            if (!n || *n < 1) return out.fail("ADD FRAME needs the drawing's feature control frame number (1, 2 ...)");
            if (!stackItemFromFrame(it, doc.state.drawing, static_cast<int>(std::lround(*n)) - 1, &why)) return out.fail(why);
            it.name = "Frame " + std::to_string(static_cast<int>(std::lround(*n))) + " (" + it.gdt + ")";
            i += 2;
        } else if (i < a.size() && unq(i) && lowerOf(a[i]) == "gdt") {
            // ADD GDT characteristic zone [DIA] [MMC|LMC BONUS b]: a geometric tolerance.
            if (i + 2 >= a.size()) return out.fail("ADD GDT needs the characteristic and its zone: ADD GDT POSITION 0.1 [DIA] [MMC BONUS 0.1]");
            it.gdt = a[i + 1];
            std::string w = a[i + 2];
            if (w.rfind("\xC3\x98", 0) == 0) w = w.substr(2), it.diametral = true; // a leading diameter sign
            const auto z = parseLength(w);
            if (!z || !(*z > 0)) return out.fail("ADD GDT needs the zone after the characteristic (above 0): ADD GDT POSITION 0.1");
            it.zone = *z;
            i += 3;
            if (i < a.size() && unq(i) && (lowerOf(a[i]) == "dia" || lowerOf(a[i]) == "diameter")) it.diametral = true, ++i;
            if (!stackItemFromGdt(it, &why)) return out.fail(why);
            it.name = std::string(1, static_cast<char>(std::toupper(static_cast<unsigned char>(it.gdt[0])))) + it.gdt.substr(1);
        }
        ItemOptions o;
        std::vector<std::string> loose;
        if (!readItemOptions(a, quoted, i, o, loose, why)) return out.fail(why);
        // What's left: a name and (by hand) the nominal size.
        for (const std::string& w : loose) {
            // An angle's size is degrees (a plain number), any other a length in the units.
            const auto v = it.body.empty() && it.dim < 0 && it.gdt.empty() && !o.nominal
                               ? (o.angle.value_or(false) ? (parseNumber(w) ? std::optional<double>(*parseNumber(w)) : std::nullopt) : parseLength(w))
                               : std::nullopt;
            if (v) o.nominal = *v;
            else if (!o.name) o.name = w;
            else return out.fail("STACK ADD doesn't know " + w + " (the name, then the size: ADD Spacer 10 TOL 0.05)");
        }
        const bool geometric = !it.gdt.empty();
        if (it.body.empty() && it.dim < 0 && !geometric && !o.nominal && !o.limits)
            return out.fail("STACK ADD needs the size: ADD [name] 10 TOL 0.05 (or BODY name ALONG X, DIM n, GDT ..., FRAME n)");
        if (geometric && (o.nominal || o.tol || o.limits || o.fit || o.general))
            return out.fail("A geometric tolerance has no size or tolerance of its own: its ZONE (and BONUS) set its band");
        if (o.material && *o.material && !o.bonus && it.frame < 0)
            return out.fail("With MMC or LMC, give the feature's size range too (its bonus): ADD GDT POSITION 0.1 MMC BONUS 0.2");
        if (o.nominal && (!it.body.empty() || it.dim >= 0))
            return out.fail("A measured dimension takes its size from the model or the drawing (EDIT it to type one)");
        if (it.name.empty()) it.name = "Dimension " + std::to_string(st->items.size() + 1);
        if (!applyItemOptions(it, o, why, true)) return out.fail(why);
        st->items.push_back(it);
        out.say("Added " + itemLine(it, st->items.size() - 1, st->vector) + " to " + st->name);
        out.say(summary(*st));
        return out;
    }
    if (sub == "edit") {
        const auto n = a.size() > 2 ? parseNumber(a[2]) : std::nullopt;
        if (!n || *n < 1 || *n > static_cast<double>(st->items.size())) return out.fail("STACK EDIT needs the dimension's number (1 to " + std::to_string(st->items.size()) + ")");
        StackItem it = st->items[static_cast<std::size_t>(std::lround(*n)) - 1];
        ItemOptions o;
        std::vector<std::string> loose;
        if (!readItemOptions(a, quoted, 3, o, loose, why, it.angle)) return out.fail(why);
        if (!loose.empty())
            return out.fail("STACK EDIT doesn't know " + loose[0] + " (NAME, NOMINAL, TOL, LIMITS, FIT, GENERAL, MINUS, PLUS, SENS, NORMAL, UNIFORM, TRIANGULAR, CP, DIR, ANGLE, AXIS, ZONE, BONUS, MMC, LMC, RFS)");
        if (o.nominal) it.body.clear(), it.dim = -1; // typed: no longer measured
        if (!applyItemOptions(it, o, why, false)) return out.fail(why);
        const std::size_t k = static_cast<std::size_t>(std::lround(*n)) - 1;
        st->items[k] = it;
        out.say("Changed " + itemLine(it, k, st->vector));
        out.say(summary(*st));
        return out;
    }
    if (sub == "remove") {
        const auto n = a.size() > 2 ? parseNumber(a[2]) : std::nullopt;
        if (!n || *n < 1 || *n > static_cast<double>(st->items.size())) return out.fail("STACK REMOVE needs the dimension's number (1 to " + std::to_string(st->items.size()) + ")");
        const std::size_t k = static_cast<std::size_t>(std::lround(*n)) - 1;
        out.say("Removed " + itemLine(st->items[k], k, st->vector));
        st->items.erase(st->items.begin() + static_cast<std::ptrdiff_t>(k));
        out.say(summary(*st));
        return out;
    }
    if (sub == "delete") {
        const std::string name = st->name;
        stacks.erase(stacks.begin() + (st - stacks.data()));
        out.say("Deleted the stack " + name);
        return out;
    }
    if (sub == "rename") {
        if (a.size() < 3) return out.fail("STACK RENAME needs the new name");
        if (ToleranceStack* other = findStack(doc, a[2]); other && other != st) return out.fail("There is already a stack named " + a[2]);
        out.say("Renamed the stack " + st->name + " to " + a[2]);
        st->name = a[2];
        return out;
    }
    if (sub == "spec" || sub == "min" || sub == "max" || sub == "gap" || sub == "note" || sub == "samples" || sub == "vector" || sub == "linear" ||
        sub == "measure" || sub == "distance") {
        ToleranceStack copy = *st;
        for (std::size_t i = 1; i < a.size(); ++i) {
            const std::string k = unq(i) ? lowerOf(a[i]) : std::string();
            if (k == "spec" || k == "min" || k == "max") {
                if (!readSpec(a, i, copy, why)) return out.fail(why);
            } else if (k == "gap") {
                if (!readGap(doc, a, i, copy, why)) return out.fail(why);
            } else if (k == "note") {
                if (i + 1 >= a.size()) return out.fail("NOTE needs a text");
                copy.note = a[++i];
            } else if (k == "samples") {
                const auto n = i + 1 < a.size() ? parseNumber(a[i + 1]) : std::nullopt;
                if (!n || *n < 0 || *n > 2000000) return out.fail("SAMPLES is from 0 (no Monte Carlo) to 2000000");
                copy.samples = static_cast<int>(std::lround(*n)), ++i;
            } else if (k == "vector" || k == "linear" || k == "measure" || k == "distance") {
                if (!readMode(a, i, copy, why)) return out.fail(why);
            } else {
                return out.fail("STACK " + st->name + " doesn't take " + a[i] + " here (SPEC, MIN, MAX, GAP, NOTE, SAMPLES, VECTOR, LINEAR, MEASURE)");
            }
        }
        *st = copy;
        std::string spec = st->hasLow || st->hasHigh ? (st->hasLow ? num(st->low) : std::string("-")) + " to " + (st->hasHigh ? num(st->high) : std::string("-")) + " mm"
                                                     : std::string("none");
        out.say("Stack " + st->name + ": spec " + spec + (st->gapFrom.empty() ? "" : "; checked against the gap from " + st->gapFrom + " to " + st->gapTo) +
                (st->vector ? "; a vector loop, measured " + (length(st->measure) > 0 ? "along " + dirText(st->measure) : std::string("as the distance across it")) : ""));
        out.say(summary(*st));
        return out;
    }
    if (sub == "update") {
        std::vector<std::string> report;
        ToleranceStack copy = *st;
        if (!updateStack(copy, doc.scene, doc.state.drawing, &report, &why)) return out.fail(why);
        *st = copy;
        if (report.empty()) out.say("Stack " + st->name + ": every measured dimension is as it was");
        for (const std::string& l : report) out.say(l);
        out.say(summary(*st));
        return out;
    }
    if (sub == "list") {
        out.say("Stack " + st->name + (st->note.empty() ? "" : " (" + st->note + ")") + ":");
        for (std::size_t k = 0; k < st->items.size(); ++k) {
            std::string l = "  " + itemLine(st->items[k], k, st->vector);
            if (!st->items[k].body.empty()) l += "  [size of " + st->items[k].body + " along " + dirText(st->items[k].along) + "]";
            if (st->items[k].dim >= 0) l += "  [drawing dimension " + std::to_string(st->items[k].dim + 1) + "]";
            out.say(l);
        }
        return out;
    }
    if (sub != "report" && sub != "csv" && sub != "svg" && sub != "samples")
        return out.fail("STACK " + st->name + " doesn't know " + a[1] + " (ADD, EDIT, REMOVE, SPEC, MIN, MAX, GAP, NOTE, UPDATE, LIST, REPORT, RENAME, DELETE)");
    // REPORT [SAMPLES n] [CSV file] [SVG file]
    ToleranceStack run = *st;
    std::string csv, svg;
    for (std::size_t i = sub == "report" && a.size() > 1 && unq(1) && lowerOf(a[1]) == "report" ? 2 : 1; i < a.size(); ++i) {
        const std::string k = unq(i) ? lowerOf(a[i]) : std::string();
        if (k == "samples") {
            const auto n = i + 1 < a.size() ? parseNumber(a[i + 1]) : std::nullopt;
            if (!n || *n < 0 || *n > 2000000) return out.fail("SAMPLES is from 0 (no Monte Carlo) to 2000000");
            run.samples = static_cast<int>(std::lround(*n)), ++i;
        } else if (k == "csv" || k == "svg") {
            if (i + 1 >= a.size()) return out.fail(toUpper(k) + " needs a file name");
            (k == "csv" ? csv : svg) = a[++i];
        } else {
            return out.fail("STACK REPORT doesn't take " + a[i] + " (SAMPLES, CSV, SVG)");
        }
    }
    StackResult r = analyzeStack(run);
    if (!run.gapFrom.empty() && bodyGapAlong(doc.scene, run.gapFrom, run.gapTo, run.gapAlong, r.modelGap, nullptr)) r.measured = true;
    for (const std::string& l : stackReport(run, r)) out.say(l);
    if (!csv.empty()) {
        if (!writeText(csv, stackCsv(run, r))) return out.fail("Can't write " + csv);
        out.say("Wrote " + csv);
    }
    if (!svg.empty()) {
        if (!writeText(svg, stackSvg(run, r))) return out.fail("Can't write " + svg);
        out.say("Wrote the chart " + svg);
    }
    return out;
}

} // namespace sicdm

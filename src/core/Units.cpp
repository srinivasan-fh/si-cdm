#include "core/Units.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <numeric>
#include <string>

namespace sicdm {

namespace {

constexpr LengthUnitInfo kUnits[] = {
    {LengthUnit::Millimeter, "mm", "millimeter", 1, 1, 3},
    {LengthUnit::Centimeter, "cm", "centimeter", 10, 1, 4},
    {LengthUnit::Meter, "m", "meter", 1000, 1, 6},
    {LengthUnit::Inch, "in", "inch", 127, 5, 4},    // 25.4 mm
    {LengthUnit::Foot, "ft", "foot", 1524, 5, 5},   // 304.8 mm
    {LengthUnit::Point, "pt", "point", 127, 360, 3} // 1/72 in = 0.352777... mm
};

thread_local UnitSettings t_units;

// An exact fraction n/d, falling back to long double when it would overflow.
struct Exact {
    std::int64_t n = 0, d = 1;
    bool exact = true;
    long double f = 0.0L;

    static Exact of(std::int64_t num, std::int64_t den) {
        Exact e;
        e.n = num;
        e.d = den;
        e.f = static_cast<long double>(num) / static_cast<long double>(den);
        e.reduce();
        return e;
    }
    static Exact approx(long double v) {
        Exact e;
        e.exact = false;
        e.f = v;
        return e;
    }
    void reduce() {
        if (!exact) return;
        if (d < 0) n = -n, d = -d;
        const std::int64_t g = std::gcd(n < 0 ? -n : n, d);
        if (g > 1) n /= g, d /= g;
    }
    double value() const {
        return static_cast<double>(exact ? static_cast<long double>(n) / static_cast<long double>(d) : f);
    }
};

constexpr std::int64_t kLimit = std::numeric_limits<std::int64_t>::max() / 4;

bool mul(std::int64_t a, std::int64_t b, std::int64_t& out) {
    if (a == 0 || b == 0) {
        out = 0;
        return true;
    }
    const std::int64_t aa = a < 0 ? -a : a, bb = b < 0 ? -b : b;
    if (aa > kLimit / bb) return false;
    out = a * b;
    return true;
}

Exact operator*(const Exact& a, const Exact& b) {
    if (a.exact && b.exact) {
        // Cross-reduce first to keep the numbers small.
        const std::int64_t g1 = std::gcd(a.n < 0 ? -a.n : a.n, b.d), g2 = std::gcd(b.n < 0 ? -b.n : b.n, a.d);
        const std::int64_t an = g1 ? a.n / g1 : a.n, bd = g1 ? b.d / g1 : b.d;
        const std::int64_t bn = g2 ? b.n / g2 : b.n, ad = g2 ? a.d / g2 : a.d;
        std::int64_t n = 0, d = 0;
        if (mul(an, bn, n) && mul(ad, bd, d)) return Exact::of(n, d);
    }
    return Exact::approx(a.f * b.f);
}

Exact operator+(const Exact& a, const Exact& b) {
    if (a.exact && b.exact) {
        const std::int64_t g = std::gcd(a.d, b.d);
        std::int64_t x = 0, y = 0, d = 0;
        if (mul(a.n, b.d / g, x) && mul(b.n, a.d / g, y) && mul(a.d / g, b.d, d) && (x < kLimit && x > -kLimit) &&
            (y < kLimit && y > -kLimit))
            return Exact::of(x + y, d);
    }
    return Exact::approx(a.f + b.f);
}

Exact negate(Exact a) {
    a.n = -a.n;
    a.f = -a.f;
    return a;
}

Exact pow10(int k) {
    std::int64_t p = 1;
    for (int i = 0; i < std::abs(k); ++i)
        if (!mul(p, 10, p)) return Exact::approx(std::pow(10.0L, static_cast<long double>(k)));
    return k >= 0 ? Exact::of(p, 1) : Exact::of(1, p);
}

Exact unitFactor(LengthUnit u) {
    const LengthUnitInfo& i = unitInfo(u);
    return Exact::of(i.num, i.den);
}

std::string lowerAscii(std::string_view s) {
    std::string r(s);
    for (char& c : r) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return r;
}

// Recursive-descent reader over the typed text.
class Reader {
public:
    explicit Reader(std::string_view s) : m_s(s) {}

    bool done() const { return m_i >= m_s.size(); }
    char peek(std::size_t k = 0) const { return m_i + k < m_s.size() ? m_s[m_i + k] : '\0'; }
    void skipSpaces() {
        while (!done() && std::isspace(static_cast<unsigned char>(peek()))) ++m_i;
    }
    std::size_t pos() const { return m_i; }
    std::string_view text() const { return m_s; }
    void seek(std::size_t i) { m_i = i; }
    void advance(std::size_t k = 1) { m_i += k; }
    bool digit(std::size_t k = 0) const { return std::isdigit(static_cast<unsigned char>(peek(k))) != 0; }

    // Unsigned decimal or "a/b". `integer` reports a plain whole number (a mixed
    // fraction may follow it).
    std::optional<Exact> number(bool& integer) {
        const std::size_t start = m_i;
        std::int64_t mant = 0;
        int digits = 0, fracDigits = 0;
        bool overflow = false, dot = false;
        while (digit() || (peek() == '.' && !dot)) {
            if (peek() == '.') {
                dot = true;
                ++m_i;
                continue;
            }
            const int dgt = peek() - '0';
            if (!overflow && mant <= (kLimit - dgt) / 10) {
                mant = mant * 10 + dgt;
                if (dot) ++fracDigits;
            } else if (!dot) {
                overflow = true; // keep counting magnitude below
            } else {
                // extra fraction digits beyond int64: ignored by the exact path
                overflow = true;
            }
            ++digits;
            ++m_i;
        }
        if (digits == 0) {
            m_i = start;
            return std::nullopt;
        }
        int exponent = 0;
        bool hasExp = false;
        if ((peek() == 'e' || peek() == 'E') && (digit(1) || ((peek(1) == '+' || peek(1) == '-') && digit(2)))) {
            hasExp = true;
            ++m_i;
            int sign = 1;
            if (peek() == '+' || peek() == '-') sign = peek() == '-' ? -1 : 1, ++m_i;
            int e = 0;
            while (digit()) {
                e = std::min(e * 10 + (peek() - '0'), 4000);
                ++m_i;
            }
            exponent = sign * e;
        }
        Exact v;
        if (overflow) {
            v = Exact::approx(std::strtold(std::string(m_s.substr(start, m_i - start)).c_str(), nullptr));
        } else {
            v = Exact::of(mant, 1) * pow10(exponent - fracDigits);
        }
        integer = !dot && !hasExp;
        // "a/b"
        if (integer && peek() == '/' && digit(1)) {
            ++m_i;
            bool denInt = false;
            const auto den = number(denInt);
            if (!den || !denInt || (den->exact ? den->n == 0 : den->f == 0.0L)) return std::nullopt;
            v = den->exact && v.exact ? Exact::of(v.n, den->n) : Exact::approx(v.f / den->f);
            integer = false;
        }
        return v;
    }

    // A unit word or mark right here (after optional spaces). Restores the position if none.
    std::optional<LengthUnit> unit(bool& bad) {
        const std::size_t start = m_i;
        skipSpaces();
        bad = false;
        if (peek() == '\'') return ++m_i, LengthUnit::Foot;
        if (peek() == '"') return ++m_i, LengthUnit::Inch;
        // Prime / double prime marks (U+2032, U+2033).
        if (peek() == '\xE2' && peek(1) == '\x80' && (peek(2) == '\xB2' || peek(2) == '\xB3')) {
            const bool feet = peek(2) == '\xB2';
            m_i += 3;
            return feet ? LengthUnit::Foot : LengthUnit::Inch;
        }
        std::size_t e = m_i;
        while (e < m_s.size() && std::isalpha(static_cast<unsigned char>(m_s[e]))) ++e;
        if (e == m_i) {
            m_i = start;
            return std::nullopt;
        }
        const auto u = parseUnitName(m_s.substr(m_i, e - m_i));
        if (!u) {
            bad = true;
            m_i = start;
            return std::nullopt;
        }
        m_i = e;
        return u;
    }

private:
    std::string_view m_s;
    std::size_t m_i = 0;
};

struct Term {
    Exact value;                   // in its unit
    std::optional<LengthUnit> unit; // explicit unit
};

// number [mixed fraction] [unit]
std::optional<Term> readTerm(Reader& r, bool& bad) {
    bad = false;
    r.skipSpaces();
    bool integer = false;
    auto v = r.number(integer);
    if (!v) return std::nullopt;
    if (integer) { // mixed number: 3-1/2  or  3 1/2
        Reader probe = r;
        if (probe.peek() == '-') probe.advance();
        else probe.skipSpaces();
        if (probe.pos() != r.pos() && probe.digit()) {
            const std::size_t at = probe.pos();
            bool fi = false;
            const auto f = probe.number(fi);
            const std::string_view typed = probe.text().substr(at, probe.pos() - at);
            if (f && typed.find('/') != std::string_view::npos) { // only a true fraction a/b
                v = *v + *f;
                r = probe;
            }
        }
    }
    Term t{*v, std::nullopt};
    bool badUnit = false;
    t.unit = r.unit(badUnit);
    if (badUnit) {
        bad = true;
        return std::nullopt;
    }
    return t;
}

} // namespace

std::span<const LengthUnitInfo> lengthUnits() { return kUnits; }

const LengthUnitInfo& unitInfo(LengthUnit unit) {
    for (const LengthUnitInfo& i : kUnits)
        if (i.unit == unit) return i;
    return kUnits[0];
}

std::optional<LengthUnit> parseUnitName(std::string_view word) {
    const std::string w = lowerAscii(word);
    struct Name {
        const char* text;
        LengthUnit unit;
    };
    static constexpr Name kNames[] = {
        {"mm", LengthUnit::Millimeter},         {"millimeter", LengthUnit::Millimeter},
        {"millimeters", LengthUnit::Millimeter}, {"millimetre", LengthUnit::Millimeter},
        {"millimetres", LengthUnit::Millimeter}, {"cm", LengthUnit::Centimeter},
        {"centimeter", LengthUnit::Centimeter},  {"centimeters", LengthUnit::Centimeter},
        {"centimetre", LengthUnit::Centimeter},  {"centimetres", LengthUnit::Centimeter},
        {"m", LengthUnit::Meter},                {"meter", LengthUnit::Meter},
        {"meters", LengthUnit::Meter},           {"metre", LengthUnit::Meter},
        {"metres", LengthUnit::Meter},           {"in", LengthUnit::Inch},
        {"inch", LengthUnit::Inch},              {"inches", LengthUnit::Inch},
        {"\"", LengthUnit::Inch},                {"ft", LengthUnit::Foot},
        {"foot", LengthUnit::Foot},              {"feet", LengthUnit::Foot},
        {"'", LengthUnit::Foot},                 {"pt", LengthUnit::Point},
        {"pts", LengthUnit::Point},              {"point", LengthUnit::Point},
        {"points", LengthUnit::Point},
    };
    for (const Name& n : kNames)
        if (w == n.text) return n.unit;
    return std::nullopt;
}

double toMillimeters(double value, LengthUnit unit) {
    const LengthUnitInfo& i = unitInfo(unit);
    return static_cast<double>(static_cast<long double>(value) * i.num / i.den);
}

double fromMillimeters(double mm, LengthUnit unit) {
    const LengthUnitInfo& i = unitInfo(unit);
    return static_cast<double>(static_cast<long double>(mm) * i.den / i.num);
}

double convertLength(double value, LengthUnit from, LengthUnit to) {
    const LengthUnitInfo& a = unitInfo(from);
    const LengthUnitInfo& b = unitInfo(to);
    return static_cast<double>(static_cast<long double>(value) * a.num * b.den / (static_cast<long double>(a.den) * b.num));
}

int UnitSettings::decimals() const { return precision >= 0 ? std::min(precision, 10) : unitInfo(unit).decimals; }

bool UnitSettings::fractional() const {
    return style == LengthStyle::Fractional && (unit == LengthUnit::Inch || unit == LengthUnit::Foot);
}

double UnitSettings::resolutionMm() const {
    if (fractional()) return 25.4 / std::max(denominator, 1);
    return toMillimeters(std::pow(10.0, -decimals()), unit);
}

std::optional<double> parseLength(std::string_view text, LengthUnit defaultUnit) {
    Reader r(text);
    r.skipSpaces();
    if (r.done()) return std::nullopt;
    bool negative = false;
    if (r.peek() == '+' || r.peek() == '-') {
        negative = r.peek() == '-';
        r.advance();
    }
    Exact total = Exact::of(0, 1);
    Exact group = Exact::of(0, 1);
    bool groupNegative = negative;
    std::optional<LengthUnit> lastExplicit; // explicit unit of the previous term in this group
    bool implicitJoin = false;              // this term follows the previous one without + / -
    auto closeGroup = [&] {
        total = total + (groupNegative ? negate(group) : group);
        group = Exact::of(0, 1);
        lastExplicit.reset();
    };
    while (true) {
        bool bad = false;
        const auto t = readTerm(r, bad);
        if (!t) return std::nullopt;
        // Juxtaposed terms need units ("1m 25mm"); only inches may stay bare after feet ("5' 6").
        if (implicitJoin && !t->unit && lastExplicit != LengthUnit::Foot) return std::nullopt;
        implicitJoin = false;
        // A bare number after feet is inches (5'6"); otherwise the default unit.
        const LengthUnit u = t->unit ? *t->unit : (lastExplicit == LengthUnit::Foot ? LengthUnit::Inch : defaultUnit);
        group = group + t->value * unitFactor(u);
        lastExplicit = t->unit;
        r.skipSpaces();
        if (r.done()) break;
        const char c = r.peek();
        if (c == '+' || c == '-') {
            // Architectural "5'-6"": a dash after feet joins the inches.
            if (c == '-' && lastExplicit == LengthUnit::Foot) {
                Reader probe = r;
                probe.advance();
                bool b2 = false;
                const auto next = readTerm(probe, b2);
                if (next && (!next->unit || *next->unit == LengthUnit::Inch)) {
                    r.advance();
                    continue;
                }
            }
            r.advance();
            closeGroup();
            groupNegative = c == '-';
            continue;
        }
        if (!lastExplicit) return std::nullopt; // an implicit join needs a unit before it: 5' 6"
        implicitJoin = true;
    }
    closeGroup();
    const double mm = total.value();
    if (!std::isfinite(mm)) return std::nullopt;
    return mm;
}

std::string formatDecimal(double value, int decimals, bool trim) {
    decimals = std::clamp(decimals, 0, 12);
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.*f", decimals, value);
    std::string s = buf;
    if (trim && s.find('.') != std::string::npos) {
        while (!s.empty() && s.back() == '0') s.pop_back();
        if (!s.empty() && s.back() == '.') s.pop_back();
    }
    // No "-0" / "-0.00".
    if (!s.empty() && s[0] == '-' && s.find_first_not_of("-0.") == std::string::npos) s.erase(0, 1);
    return s;
}

std::string formatLength(double mm, const UnitSettings& settings, LengthFormat format) {
    const LengthUnit unit = settings.unit;
    if (settings.fractional()) {
        long long den = 2;
        while (den < settings.denominator && den < 256) den *= 2;
        const double inches = std::abs(fromMillimeters(mm, LengthUnit::Inch));
        const long long n = std::llround(inches * static_cast<double>(den));
        const char* sign = n != 0 && mm < 0 ? "-" : "";
        auto fraction = [&](long long rem) {
            const long long g = std::gcd(rem, den);
            return std::to_string(rem / g) + "/" + std::to_string(den / g);
        };
        std::string s = sign;
        if (unit == LengthUnit::Foot) {
            const long long perFoot = 12 * den;
            const long long feet = n / perFoot, rest = n % perFoot;
            const long long whole = rest / den, rem = rest % den;
            if (feet > 0) s += std::to_string(feet) + "'-";
            if (whole > 0 || rem == 0 || feet > 0) s += std::to_string(whole);
            if (rem) s += (s.empty() || s == "-" ? "" : " ") + fraction(rem);
        } else {
            const long long whole = n / den, rem = n % den;
            if (whole > 0 || rem == 0) s += std::to_string(whole);
            if (rem) s += (whole > 0 ? "-" : "") + fraction(rem);
        }
        if (format.symbol) s += "\"";
        return s;
    }
    std::string s = formatDecimal(fromMillimeters(mm, unit), settings.decimals(), format.trim);
    if (format.symbol) s += std::string(" ") + unitInfo(unit).symbol;
    return s;
}

const UnitSettings& activeUnits() { return t_units; }
void setActiveUnits(const UnitSettings& units) { t_units = units; }
LengthUnit inputUnit() { return t_units.unit; }
std::string lengthText(double mm, LengthFormat format) { return formatLength(mm, t_units, format); }
std::optional<double> parseLength(std::string_view text) { return parseLength(text, t_units.unit); }

std::string lengthsText(std::initializer_list<double> mm, const char* separator, const char* open, const char* close) {
    const bool each = t_units.fractional();
    std::string s = open;
    bool first = true;
    for (double v : mm) {
        if (!first) s += separator;
        first = false;
        s += formatLength(v, t_units, {each, true});
    }
    s += close;
    if (!each) s += std::string(" ") + unitInfo(t_units.unit).symbol;
    return s;
}

ScopedUnits::ScopedUnits(const UnitSettings& units) : m_previous(t_units) { t_units = units; }
ScopedUnits::~ScopedUnits() { t_units = m_previous; }

std::string describeUnits(const UnitSettings& units) {
    const LengthUnitInfo& i = unitInfo(units.unit);
    std::string s = std::string(i.symbol) + " (" + i.name + ")";
    if (units.fractional()) {
        int den = 2;
        while (den < units.denominator && den < 256) den *= 2;
        s += units.unit == LengthUnit::Foot ? ", feet-inches to 1/" : ", fractional to 1/";
        s += std::to_string(den) + " in";
    } else {
        s += ", " + std::to_string(units.decimals()) + " decimals";
    }
    return s;
}

} // namespace sicdm

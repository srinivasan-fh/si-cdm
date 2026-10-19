#pragma once

// Length units. The model is always stored in millimetres; units only change
// how lengths are typed and shown. Conversion factors are exact rationals
// (1 in = 127/5 mm by the 1959 international definition, 1 ft = 12 in,
// 1 pt = 1/72 in, the DTP / PostScript point), and typed values are evaluated
// as exact fractions, so "5'6-1/2\"", "1/64in" or "10pt" turn into millimetres
// with a single final rounding.

#include <cstdint>
#include <initializer_list>
#include <optional>
#include <span>
#include <string>
#include <string_view>

namespace sicdm {

enum class LengthUnit { Millimeter, Centimeter, Meter, Inch, Foot, Point };

struct LengthUnitInfo {
    LengthUnit unit;
    const char* symbol; // "mm", "cm", "m", "in", "ft", "pt"
    const char* name;   // "millimeter"
    std::int64_t num;   // 1 unit = num / den mm, exactly
    std::int64_t den;
    int decimals;       // default display precision
};

std::span<const LengthUnitInfo> lengthUnits();
const LengthUnitInfo& unitInfo(LengthUnit unit);
// "mm", "millimetre", "inches", "\"", "'", "feet", "pt", "points" ... (case-insensitive).
std::optional<LengthUnit> parseUnitName(std::string_view word);

double toMillimeters(double value, LengthUnit unit);
double fromMillimeters(double mm, LengthUnit unit);
double convertLength(double value, LengthUnit from, LengthUnit to);

enum class LengthStyle {
    Decimal,    // 12.7 mm, 0.5 in, 1.25 ft
    Fractional, // inch: 3-1/2"   foot: 5'-6 1/2"   (other units stay decimal)
};

// How a document shows and reads lengths.
struct UnitSettings {
    LengthUnit unit = LengthUnit::Millimeter;
    int precision = -1;          // decimals; < 0: the unit's default
    LengthStyle style = LengthStyle::Decimal;
    int denominator = 64;        // finest fraction for Fractional (2..256, a power of two)

    int decimals() const;        // effective decimals
    bool fractional() const;     // Fractional style on a unit that has it (in, ft)
    // Smallest step the display resolves, in millimetres (e.g. 0.001 mm, 1/64 in).
    double resolutionMm() const;
};

// Parses a length typed by the user and returns millimetres:
//   12.5   (in `defaultUnit`)     12.5mm  1.2 cm  0.5m  3in  3"  2ft  2'  10pt  1.5e3mm
//   1/8in  3-1/2"  3 1/2in        5'6"  5' 6-1/2"  5ft 6in  (a bare number after feet is inches)
//   1m+25mm  2ft-3in              explicit + / - between terms;  -5'6" negates the whole length
std::optional<double> parseLength(std::string_view text, LengthUnit defaultUnit);

struct LengthFormat {
    bool symbol = true; // append the unit ("12.5 mm", "3-1/2\"")
    bool trim = true;   // drop trailing zeros ("12.5" rather than "12.500")
};

// `mm` shown in the settings' unit and precision.
std::string formatLength(double mm, const UnitSettings& settings, LengthFormat format = {});
// Plain decimal number in a unit, rounded to `decimals` (trailing zeros trimmed if asked).
std::string formatDecimal(double value, int decimals, bool trim);

// The units in effect on this thread: bare numbers typed in commands are read in
// activeUnits().unit, and messages show lengths with lengthText(). The app keeps
// them in step with the active document; executeCommand() scopes them to the
// document it runs against.
const UnitSettings& activeUnits();
void setActiveUnits(const UnitSettings& units);
LengthUnit inputUnit(); // activeUnits().unit
// formatLength(mm, activeUnits()).
std::string lengthText(double mm, LengthFormat format = {});
// Several lengths sharing one unit label: "30 x 20 mm", "(1, 2, 0) in"; fractional
// styles label each value ("1-1/4\" x 3/4\"").
std::string lengthsText(std::initializer_list<double> mm, const char* separator, const char* open = "",
                        const char* close = "");
// Parses with the active unit as the default.
std::optional<double> parseLength(std::string_view text);

// Sets the active units for a scope (one command line) and restores them after.
class ScopedUnits {
public:
    explicit ScopedUnits(const UnitSettings& units);
    ~ScopedUnits();
    ScopedUnits(const ScopedUnits&) = delete;
    ScopedUnits& operator=(const ScopedUnits&) = delete;

private:
    UnitSettings m_previous;
};

// "mm", "cm", "m", "in", "ft", "pt" with the style: "in (fractional 1/64)".
std::string describeUnits(const UnitSettings& units);

} // namespace sicdm

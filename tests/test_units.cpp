#include "TestFramework.hpp"

#include "core/Units.hpp"

#include <cmath>
#include <string>

using namespace sicdm;

namespace {

double mm(const char* text, LengthUnit def = LengthUnit::Millimeter) {
    const auto v = parseLength(text, def);
    CHECK(v.has_value());
    return v.value_or(std::nan(""));
}

bool rejects(const char* text) { return !parseLength(text, LengthUnit::Millimeter).has_value(); }

UnitSettings in(LengthUnit u, int precision = -1, LengthStyle style = LengthStyle::Decimal, int den = 64) {
    UnitSettings s;
    s.unit = u;
    s.precision = precision;
    s.style = style;
    s.denominator = den;
    return s;
}

} // namespace

TEST_CASE("unit factors are the exact international definitions") {
    CHECK(toMillimeters(1, LengthUnit::Millimeter) == 1.0);
    CHECK(toMillimeters(1, LengthUnit::Centimeter) == 10.0);
    CHECK(toMillimeters(1, LengthUnit::Meter) == 1000.0);
    CHECK(toMillimeters(1, LengthUnit::Inch) == 25.4);
    CHECK(toMillimeters(1, LengthUnit::Foot) == 304.8);
    CHECK(toMillimeters(72, LengthUnit::Point) == 25.4);  // 72 pt = 1 in
    CHECK(convertLength(12, LengthUnit::Inch, LengthUnit::Foot) == 1.0);
    CHECK(convertLength(1, LengthUnit::Foot, LengthUnit::Point) == 864.0);
    CHECK(convertLength(1, LengthUnit::Meter, LengthUnit::Centimeter) == 100.0);
    // Round trips are exact to the last bit for every unit.
    for (const LengthUnitInfo& u : lengthUnits())
        for (double v : {0.1, 1.0, 3.3, 123.456, 1e-6, 98765.4321})
            CHECK_NEAR(fromMillimeters(toMillimeters(v, u.unit), u.unit), v, std::abs(v) * 1e-15);
}

TEST_CASE("lengths parse with units, fractions and mixed numbers") {
    CHECK(mm("12.5") == 12.5);
    CHECK(mm("12.5mm") == 12.5);
    CHECK(mm("1.2 cm") == 12.0);
    CHECK(mm("0.5m") == 500.0);
    CHECK(mm("3in") == 76.2);
    CHECK(mm("3\"") == 76.2);
    CHECK(mm("2ft") == 609.6);
    CHECK(mm("2'") == 609.6);
    CHECK(mm("1 foot") == 304.8);
    CHECK(mm("4 inches") == 101.6);
    CHECK(mm("72pt") == 25.4);
    CHECK(mm("10 points") == 127.0 / 36.0);
    CHECK(mm("1.5e3mm") == 1500.0);
    CHECK(mm("2.5E-1cm") == 2.5);
    CHECK(mm("1/8in") == 3.175);
    CHECK(mm("1/64\"") == 25.4 / 64.0);
    CHECK(mm("3-1/2\"") == 88.9);
    CHECK(mm("3 1/2in") == 88.9);
    CHECK(mm("12", LengthUnit::Inch) == 304.8);  // bare numbers take the default unit
    CHECK(mm("1/3", LengthUnit::Foot) == 101.6);
    CHECK(mm("+5") == 5.0);
    CHECK(mm("-5") == -5.0);
    CHECK(rejects("MM"));
}

TEST_CASE("feet and inches combine the architectural way") {
    CHECK(mm("5'6\"") == 1676.4);
    CHECK(mm("5' 6\"") == 1676.4);
    CHECK(mm("5'6") == 1676.4);           // bare number after feet is inches
    CHECK(mm("5'-6\"") == 1676.4);        // dash after feet joins
    CHECK(mm("5'-6 1/2\"") == 1689.1);
    CHECK(mm("5ft 6-1/2in") == 1689.1);
    CHECK(mm("-5'6\"") == -1676.4);       // the sign covers the whole length
    CHECK(mm("1\xE2\x80\xB2 3\xE2\x80\xB3") == 381.0); // prime marks
}

TEST_CASE("explicit plus and minus combine terms in any units") {
    CHECK(mm("1m+25mm") == 1025.0);
    CHECK(mm("1m - 25mm") == 975.0);
    CHECK(mm("-1m+20mm") == -980.0);
    CHECK(mm("2ft-3in", LengthUnit::Millimeter) == 685.8); // architectural: 2'-3" = 27 in
    CHECK(mm("2ft - 1cm") == 599.6);
    CHECK_NEAR(mm("10pt+1in"), 25.4 + 127.0 / 36.0, 1e-13);
}

TEST_CASE("bad lengths are rejected") {
    CHECK(rejects(""));
    CHECK(rejects("abc"));
    CHECK(rejects("12 furlongs"));
    CHECK(rejects("3 4"));
    CHECK(rejects("1/0in"));
    CHECK(rejects("1m+"));
    CHECK(rejects("--3"));
    CHECK(rejects("3mm4"));
    CHECK(rejects("1.2.3"));
}

TEST_CASE("exact arithmetic beats float accumulation") {
    // 0.1 + 0.2 in floating point is 0.30000000000000004; parsed exactly it is the nearest double to 0.3.
    CHECK(mm("0.1mm+0.2mm") == 0.3);
    CHECK(mm("1/3in+2/3in") == 25.4);
    CHECK(mm("0.1in+0.2in") == 7.62);
    CHECK_NEAR(mm("1/7in"), 25.4 / 7.0, 1e-14);
}

TEST_CASE("lengths format in the chosen unit and precision") {
    CHECK(formatLength(12.5, in(LengthUnit::Millimeter)) == "12.5 mm");
    CHECK(formatLength(12.5, in(LengthUnit::Centimeter)) == "1.25 cm");
    CHECK(formatLength(1500, in(LengthUnit::Meter)) == "1.5 m");
    CHECK(formatLength(25.4, in(LengthUnit::Inch)) == "1 in");
    CHECK(formatLength(25.4f, in(LengthUnit::Inch)) == "1 in"); // float storage still shows exactly
    CHECK(formatLength(304.8f, in(LengthUnit::Foot)) == "1 ft");
    CHECK(formatLength(25.4, in(LengthUnit::Point)) == "72 pt");
    CHECK(formatLength(10, in(LengthUnit::Point)) == "28.346 pt");
    CHECK(formatLength(1.0 / 3.0, in(LengthUnit::Millimeter, 2)) == "0.33 mm");
    CHECK(formatLength(2, in(LengthUnit::Millimeter, 2), {true, false}) == "2.00 mm");
    CHECK(formatLength(-0.0001, in(LengthUnit::Millimeter, 2)) == "0 mm"); // no "-0"
    CHECK(formatLength(12.5, in(LengthUnit::Millimeter), {false, true}) == "12.5");
}

TEST_CASE("fractional inches and feet-inches") {
    const UnitSettings inch = in(LengthUnit::Inch, -1, LengthStyle::Fractional);
    CHECK(formatLength(88.9, inch) == "3-1/2\"");
    CHECK(formatLength(25.4 / 64.0, inch) == "1/64\"");
    CHECK(formatLength(76.2, inch) == "3\"");
    CHECK(formatLength(0, inch) == "0\"");
    CHECK(formatLength(-88.9, inch) == "-3-1/2\"");
    CHECK(formatLength(25.4 * 0.3, in(LengthUnit::Inch, -1, LengthStyle::Fractional, 8)) == "1/4\""); // nearest 1/8
    const UnitSettings ft = in(LengthUnit::Foot, -1, LengthStyle::Fractional);
    CHECK(formatLength(1689.1, ft) == "5'-6 1/2\"");
    CHECK(formatLength(1524, ft) == "5'-0\"");
    CHECK(formatLength(152.4, ft) == "6\"");
    CHECK(formatLength(19.05, ft) == "3/4\"");
    CHECK(formatLength(-19.05, ft) == "-3/4\"");
    CHECK(formatLength(1543.05, ft) == "5'-0 3/4\"");
    // What is shown parses back to the same length.
    for (double v : {1689.1, 88.9, 3.175, 1234.5678})
        for (const UnitSettings& s : {inch, ft}) {
            const auto back = parseLength(formatLength(v, s), LengthUnit::Millimeter);
            CHECK(back.has_value());
            CHECK_NEAR(back.value_or(0), v, s.resolutionMm() / 2 + 1e-9);
        }
    // Fractional only applies to inch and foot.
    CHECK(formatLength(12.5, in(LengthUnit::Millimeter, -1, LengthStyle::Fractional)) == "12.5 mm");
}

TEST_CASE("decimal display round-trips within its resolution in every unit") {
    for (const LengthUnitInfo& u : lengthUnits()) {
        const UnitSettings s = in(u.unit);
        for (double v : {0.0, 0.001, 1.0, 12.7, 25.4, 304.8, 999.999, 12345.678}) {
            const auto back = parseLength(formatLength(v, s), LengthUnit::Millimeter);
            CHECK(back.has_value());
            CHECK_NEAR(back.value_or(0), v, s.resolutionMm() / 2 + 1e-9);
        }
    }
}

TEST_CASE("unit names and the scoped input unit") {
    CHECK(parseUnitName("Millimetres") == LengthUnit::Millimeter);
    CHECK(parseUnitName("FEET") == LengthUnit::Foot);
    CHECK(parseUnitName("\"") == LengthUnit::Inch);
    CHECK(parseUnitName("pts") == LengthUnit::Point);
    CHECK(!parseUnitName("yd").has_value());
    CHECK(inputUnit() == LengthUnit::Millimeter);
    {
        ScopedUnits scope(in(LengthUnit::Inch, -1, LengthStyle::Fractional));
        CHECK(inputUnit() == LengthUnit::Inch);
        CHECK(parseLength("2").value_or(0) == 50.8);
        CHECK(lengthText(88.9) == "3-1/2\"");
        {
            ScopedUnits inner(in(LengthUnit::Point));
            CHECK(inputUnit() == LengthUnit::Point);
        }
        CHECK(inputUnit() == LengthUnit::Inch);
    }
    CHECK(inputUnit() == LengthUnit::Millimeter);
    CHECK(describeUnits(in(LengthUnit::Foot, -1, LengthStyle::Fractional, 16)) == "ft (foot), feet-inches to 1/16 in");
    CHECK(describeUnits(in(LengthUnit::Centimeter, 2)) == "cm (centimeter), 2 decimals");
}

// ---- commands and files -----------------------------------------------------

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/FileIO.hpp"

#include <filesystem>

namespace {

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }

std::string joined(const CommandOutput& o) {
    std::string s;
    for (const std::string& l : o.lines) s += l + "\n";
    return s;
}

} // namespace

TEST_CASE("UNITS switches the document units and commands follow them") {
    Document d;
    CHECK(joined(run(d, "UNITS")).find("mm (millimeter)") != std::string::npos);
    CHECK(run(d, "UNITS IN").ok);
    CHECK(d.state.units.unit == LengthUnit::Inch);
    CHECK(run(d, "BOX 2 1 1/2").ok); // inches
    const Body& b = d.scene.bodies().back();
    CHECK_NEAR(b.params[3], 25.4f, 1e-4);   // half of 2 in
    CHECK_NEAR(b.params[5], 12.7f, 1e-4);   // 1/2 in high
    CHECK(run(d, "PYRAMID 1 2 6").ok);      // the side count is not a length
    CHECK(d.scene.bodies().back().params[2] == 6.0f);
    CHECK(run(d, "CYLINDER 10mm 1ft @1,2,0").ok); // explicit units mix freely
    CHECK_NEAR(d.scene.bodies().back().params[0], 10.0f, 1e-5);
    CHECK_NEAR(d.scene.bodies().back().params[1], 304.8f, 1e-3);
    CHECK_NEAR(d.scene.bodies().back().offset.y, 50.8f, 1e-4);
    const auto c = run(d, "CIRCLE 0,0 3/4");
    CHECK(c.ok);
    CHECK_NEAR(d.scene.sketch().curves().back().radius, 19.05f, 1e-4);
    CHECK(joined(c).find("R 0.75 in") != std::string::npos);
    CHECK(run(d, "UNITS FT FRACTION 1/16").ok);
    CHECK(joined(run(d, "LIST")).find("R 3/4\"") != std::string::npos);
    CHECK(run(d, "LINE 0,0 5'6-1/2\",0").ok);
    CHECK_NEAR(d.scene.sketch().curves().back().pts[1].x, 1689.1f, 1e-3);
    CHECK(joined(run(d, "LIST")).find("L 5'-6 1/2\"") != std::string::npos);
    CHECK(run(d, "UNITS PT PRECISION 1 FRACTION OFF").ok);
    CHECK(d.state.units.precision == 1);
    CHECK(d.state.units.style == LengthStyle::Decimal);
    CHECK(!run(d, "UNITS PRECISION 11").ok);
    CHECK(!run(d, "UNITS FRACTION 1/3").ok);
    CHECK(!run(d, "UNITS PARSECS").ok);
    // The command's scope does not leak into the thread.
    CHECK(inputUnit() == LengthUnit::Millimeter);
}

TEST_CASE("CONVERT shows a length in every unit") {
    Document d;
    const auto o = run(d, "CONVERT 5'6-1/2\" TO mm");
    CHECK(o.ok);
    CHECK(joined(o).find("=  1689.1 mm") != std::string::npos);
    const std::string all = joined(run(d, "CONVERT 72pt"));
    CHECK(all.find("25.4 mm") != std::string::npos);
    CHECK(all.find("2.54 cm") != std::string::npos);
    CHECK(all.find("1 in") != std::string::npos);
    CHECK(all.find("72 pt") != std::string::npos);
    CHECK(all.find("1\"") != std::string::npos);
    CHECK(!run(d, "CONVERT 3 bananas").ok);
    CHECK(!run(d, "CONVERT 3in TO yards").ok);
}

TEST_CASE("unit settings are saved with the project") {
    const auto path = (std::filesystem::temp_directory_path() / "sicdm_units_test.sicdm").string();
    Document d;
    CHECK(run(d, "UNITS FT FRACTION 1/32").ok);
    CHECK(run(d, "BOX 1 1 1").ok);
    std::string err;
    CHECK(saveProject(d.scene, d.state, path, &err));
    Scene scene;
    ProjectState st;
    CHECK(loadProject(scene, st, path, &err));
    CHECK(st.units.unit == LengthUnit::Foot);
    CHECK(st.units.style == LengthStyle::Fractional);
    CHECK(st.units.denominator == 32);
    CHECK_NEAR(scene.bodies().back().params[5], 304.8f, 1e-3); // geometry stays in mm
    std::filesystem::remove(path);
}

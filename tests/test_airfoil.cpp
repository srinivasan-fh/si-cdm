#include "TestFramework.hpp"

#include "core/Airfoil.hpp"
#include "core/Brep.hpp"
#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Print.hpp"
#include "core/Sculpt.hpp"
#include "core/Step.hpp"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

using namespace sicdm;

#define REQUIRE(c)                                    \
    do {                                              \
        const bool required_ = static_cast<bool>(c); \
        CHECK(required_);                             \
        if (!required_) return;                       \
    } while (0)

namespace {

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }

double polyArea(const std::vector<Vec2>& p) {
    double s = 0;
    for (std::size_t i = 0; i < p.size(); ++i) {
        const Vec2 a = p[i], b = p[(i + 1) % p.size()];
        s += static_cast<double>(a.x) * b.y - static_cast<double>(b.x) * a.y;
    }
    return 0.5 * s;
}

} // namespace

TEST_CASE("Airfoils: NACA four-digit sections") {
    std::vector<Vec2> s;
    REQUIRE(nacaSection("0012", 200, s));
    // Counter-clockwise, its area the thickness integral 0.68085 t (closed trailing edge).
    CHECK(std::fabs(polyArea(s) - 0.68085 * 0.12) < 0.002 * 0.68085 * 0.12);
    float top = 0, at = 0;
    for (const Vec2& p : s)
        if (p.y > top) top = p.y, at = p.x;
    CHECK(std::fabs(2 * top - 0.12f) < 1e-3f && std::fabs(at - 0.3f) < 0.02f); // 12% thick at 30% of the chord
    CHECK(s.front().x == 1.0f && s.front().y == 0.0f);
    // 2412: its camber line 2% high at 40% of the chord.
    REQUIRE(nacaSection("2412", 200, s));
    float up = -1, low = 1;
    for (const Vec2& p : s) {
        if (std::fabs(p.x - 0.4f) > 0.004f) continue;
        up = std::max(up, p.y), low = std::min(low, p.y);
    }
    CHECK(std::fabs(0.5f * (up + low) - 0.02f) < 1.5e-3f);
    CHECK(!nacaSection("123", 10, s));
    CHECK(!nacaSection("0000", 10, s));
    CHECK(!nacaSection("2012", 10, s)); // camber with no place for it
}

TEST_CASE("Airfoils: blades with taper, twist, sweep and tubercles") {
    // Straight: NACA 0012, chord 40, span 100.
    const std::vector<float> straight{12, 100, 40, 40, 0, 0, 0, 20};
    const MeshData m = buildBlade(straight);
    REQUIRE(!m.indices.empty());
    Body b = meshBody("b", m);
    const PrintReport pr = analyzePrint(b);
    CHECK(pr.openEdges == 0 && pr.nonManifoldEdges == 0 && !pr.insideOut);
    const double want = 0.68085 * 0.12 * 1600 * 100;
    CHECK(std::fabs(meshVolume(m) - want) < 0.01 * want);
    // Twisted 10 degrees, nose down at the tip; swept 30 mm.
    const std::vector<float> twisted{2412, 100, 40, 20, 10, 30, 0, 20};
    const std::vector<Vec3> tip = bladeSection(twisted, 100, 60);
    const Vec3 te = tip.front(), le = tip[59];
    CHECK(std::fabs(std::atan2(te.y - le.y, te.x - le.x) * 180.0 / 3.14159265 + 10.0) < 0.2);
    CHECK(std::fabs(length(te - le) - 20.0f) < 0.05f);
    // Tubercles: the leading edge goes forward and back by their height.
    const std::vector<float> bumpy{21, 150, 50, 30, 0, 0, 2.5f, 20};
    float lo = 1e9f, hi = -1e9f;
    for (int k = 0; k <= 60; ++k) {
        const std::vector<Vec3> sec = bladeSection(bumpy, 5.0f + 0.25f * k, 60);
        lo = std::min(lo, sec[59].x), hi = std::max(hi, sec[59].x);
    }
    CHECK(hi - lo > 4.5f && hi - lo < 5.6f);
    CHECK(buildBlade({21, 150, 50, 30, 0, 0, 20, 20}).indices.empty()); // too deep for the chord
}

TEST_CASE("Airfoils: commands, exact geometry and edits") {
    Document d;
    REQUIRE(run(d, "AIRFOIL NACA 2412 CHORD 80 AT 10,5 ANGLE 4").ok);
    const SketchCurve& c = d.scene.sketch().curves().back();
    CHECK(c.isClosed());
    REQUIRE(run(d, "EXTRUDE 10").ok);
    CHECK(!run(d, "AIRFOIL NACA 12345").ok);

    REQUIRE(run(d, "BLADE NACA 0012 SPAN 100 ROOT 40 TIP 40 TWIST 0 SWEEP 0").ok);
    const Body& b = d.scene.bodies().back();
    CHECK(b.kind == "blade" && b.exact());
    const double want = 0.68085 * 0.12 * 1600 * 100;
    {
        const MeshData skin = tessellate(*b.brep);
        CHECK(std::fabs(meshVolume(skin) - want) < 0.01 * want);
    }
    const std::string name = b.name;
    REQUIRE(run(d, "PARAM \"" + name + "\" TWIST 12 TUBERCLES 2 WAVELENGTH 25").ok);
    CHECK(d.scene.bodies().back().params[4] == 12.0f && d.scene.bodies().back().exact());
    REQUIRE(run(d, "WING TUBERCLES 2.5 20 @0,80,0").ok);
    CHECK(d.scene.bodies().back().name.rfind("Wing", 0) == 0);
    CHECK(!run(d, "BLADE TUBERCLES 30 20").ok);
    // Out to STEP and back.
    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_blade.step").string();
    REQUIRE(run(d, "EXPORT " + path).ok);
    const std::vector<StepPart> parts = readStepFile(path);
    CHECK(parts.size() == 3);
}

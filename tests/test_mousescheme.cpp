#include "TestFramework.hpp"

#include "core/Json.hpp"
#include "core/MouseScheme.hpp"
#include "core/Preferences.hpp"

using namespace sicdm;

TEST_CASE("Mouse schemes: each preset's buttons orbit, pan and zoom as in the program it follows") {
    using B = MouseButton;
    using D = ViewDrag;
    // SI CDM: right orbits, middle pans; in 2D the right pans too.
    CHECK(mouseViewDrag("sicdm", B::Right, false, false, false) == D::Orbit);
    CHECK(mouseViewDrag("sicdm", B::Middle, false, false, false) == D::Pan);
    CHECK(mouseViewDrag("sicdm", B::Right, false, false, true) == D::Pan);
    // SolidWorks: middle orbits, Ctrl+middle pans, Shift+middle zooms; right is the menu.
    CHECK(mouseViewDrag("solidworks", B::Middle, false, false, false) == D::Orbit);
    CHECK(mouseViewDrag("solidworks", B::Middle, false, true, false) == D::Pan);
    CHECK(mouseViewDrag("solidworks", B::Middle, true, false, false) == D::Zoom);
    CHECK(mouseViewDrag("solidworks", B::Right, false, false, false) == D::None);
    // Creo: Shift+middle pans, Ctrl+middle zooms.
    CHECK(mouseViewDrag("creo", B::Middle, true, false, false) == D::Pan);
    CHECK(mouseViewDrag("creo", B::Middle, false, true, false) == D::Zoom);
    // Fusion: middle pans, Shift+middle orbits (pans in 2D).
    CHECK(mouseViewDrag("fusion", B::Middle, false, false, false) == D::Pan);
    CHECK(mouseViewDrag("fusion", B::Middle, true, false, false) == D::Orbit);
    CHECK(mouseViewDrag("fusion", B::Middle, true, false, true) == D::Pan);
    // The left button is always the tool's.
    for (const MouseSchemeInfo& m : mouseSchemes()) {
        CHECK(mouseViewDrag(m.key, B::Left, false, false, false) == D::None);
        CHECK(!m.name.empty() && !m.description.empty());
    }

    Preferences p;
    std::string err;
    CHECK(p.mouse == "sicdm");
    CHECK(setPreference(p, "mouse", "SolidWorks", &err) && p.mouse == "solidworks");
    CHECK(!setPreference(p, "mouse", "trackball", &err));
    CHECK(err.find("creo") != std::string::npos); // the error lists them
    CHECK(preferencesFromJson(preferencesToJson(p)).mouse == "solidworks");
}

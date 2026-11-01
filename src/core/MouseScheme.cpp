#include "core/MouseScheme.hpp"

#include "core/Translate.hpp"

#include <algorithm>

namespace sicdm {

const std::vector<MouseSchemeInfo>& mouseSchemes() {
    static const std::vector<MouseSchemeInfo> schemes{
        {"sicdm", "SI CDM", SICDM_TR_NOOP("Right drag orbits, middle drag pans")},
        {"solidworks", "SolidWorks", SICDM_TR_NOOP("Middle drag orbits, Ctrl+middle pans, Shift+middle zooms")},
        {"creo", "Creo / NX", SICDM_TR_NOOP("Middle drag orbits, Shift+middle pans, Ctrl+middle zooms")},
        {"fusion", "Fusion / Inventor", SICDM_TR_NOOP("Middle drag pans, Shift+middle orbits")},
        {"catia", "CATIA", SICDM_TR_NOOP("Middle drag pans, middle with Ctrl orbits, Shift+middle zooms")},
        {"blender", "Blender", SICDM_TR_NOOP("Middle drag orbits, Shift+middle pans, Ctrl+middle zooms")},
    };
    return schemes;
}

bool isMouseScheme(const std::string& key) {
    const auto& s = mouseSchemes();
    return std::any_of(s.begin(), s.end(), [&](const MouseSchemeInfo& m) { return m.key == key; });
}

ViewDrag mouseViewDrag(const std::string& scheme, MouseButton button, bool shift, bool ctrl, bool flat) {
    auto result = [&](ViewDrag d) { return flat && d == ViewDrag::Orbit ? ViewDrag::Pan : d; };
    if (button == MouseButton::Left) return ViewDrag::None;
    const bool middle = button == MouseButton::Middle;
    if (scheme == "solidworks") {
        if (!middle) return ViewDrag::None;
        return result(ctrl ? ViewDrag::Pan : shift ? ViewDrag::Zoom : ViewDrag::Orbit);
    }
    if (scheme == "creo" || scheme == "blender") {
        if (!middle) return ViewDrag::None;
        return result(shift ? ViewDrag::Pan : ctrl ? ViewDrag::Zoom : ViewDrag::Orbit);
    }
    if (scheme == "fusion") {
        if (!middle) return ViewDrag::None;
        return result(shift ? ViewDrag::Orbit : ViewDrag::Pan);
    }
    if (scheme == "catia") {
        if (!middle) return ViewDrag::None;
        return result(ctrl ? ViewDrag::Orbit : shift ? ViewDrag::Zoom : ViewDrag::Pan);
    }
    // SI CDM: right orbits (pans in 2D), middle pans.
    return result(middle ? ViewDrag::Pan : ViewDrag::Orbit);
}

} // namespace sicdm

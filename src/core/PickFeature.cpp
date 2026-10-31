#include "core/PickFeature.hpp"

#include "core/Translate.hpp"

#include <cmath>
#include <cstdio>

namespace sicdm {

namespace {

std::string mm(float v) {
    char b[48];
    std::snprintf(b, sizeof b, "%.6gmm", static_cast<double>(v));
    return b;
}

std::string num(float v) {
    char b[48];
    std::snprintf(b, sizeof b, "%.6g", static_cast<double>(v));
    return b;
}

const char* commandName(ToolId t) {
    switch (t) {
    case ToolId::Revolve: return "REVOLVE";
    case ToolId::Sweep: return "SWEEP";
    case ToolId::Helix: return "SWEEP";
    case ToolId::Loft: return "LOFT";
    case ToolId::Rib: return "RIB";
    default: return "SURFACE";
    }
}

} // namespace

std::string quoteWord(const std::string& name) {
    std::string s = "\"";
    for (char c : name)
        if (c != '"') s += c;
    return s + "\"";
}

bool isPickFeatureTool(ToolId t) noexcept {
    switch (t) {
    case ToolId::Revolve:
    case ToolId::Sweep:
    case ToolId::Helix:
    case ToolId::Loft:
    case ToolId::Rib:
    case ToolId::PlaneSurface:
    case ToolId::TabulatedCylinder:
    case ToolId::RuledSurface:
    case ToolId::BSplinePatch:
    case ToolId::CoonsPatch:
    case ToolId::GordonSurface:
    case ToolId::OffsetSurface: return true;
    default: return false;
    }
}

PickFeatureSpec pickFeatureSpec(ToolId t, const PickFeatureOptions& o) noexcept {
    switch (t) {
    case ToolId::Revolve: return o.revolveAxis == 2 ? PickFeatureSpec{2, 2, false} : PickFeatureSpec{1, 1, false};
    case ToolId::Sweep: return {2, 2, false};
    case ToolId::Helix: return {1, 1, false};
    case ToolId::Loft: return {2, -1, false};
    case ToolId::Rib: return {1, 1, false};
    case ToolId::PlaneSurface: return {1, 1, false};
    case ToolId::TabulatedCylinder: return {1, 1, false};
    case ToolId::RuledSurface: return {2, 2, false};
    case ToolId::BSplinePatch: return {2, -1, false};
    case ToolId::CoonsPatch: return {4, 4, false};
    case ToolId::GordonSurface: return {4, -1, false};
    case ToolId::OffsetSurface: return {0, 0, true};
    default: return {};
    }
}

bool pickWantsClosed(ToolId t, int index) noexcept {
    switch (t) {
    case ToolId::Revolve:
    case ToolId::Sweep: return index == 0; // the profile; then an axis line or a path
    case ToolId::Helix:
    case ToolId::Loft:
    case ToolId::PlaneSurface: return true;
    default: return false;
    }
}

std::string pickFeaturePrompt(ToolId t, const PickFeatureOptions& o, int picked) {
    switch (t) {
    case ToolId::Revolve:
        if (picked == 0) return SICDM_TR_NOOP("Click a closed profile to revolve");
        return SICDM_TR_NOOP("Click a sketch line on its plane: the axis");
    case ToolId::Sweep: return picked == 0 ? SICDM_TR_NOOP("Click a closed profile to sweep") : SICDM_TR_NOOP("Click the path: a line, arc, polyline or spline");
    case ToolId::Helix: return SICDM_TR_NOOP("Click a closed profile to sweep round a helix (a coil spring)");
    case ToolId::Loft:
        return picked < 2 ? SICDM_TR_NOOP("Click the sections in order (closed curves)") : SICDM_TR_NOOP("Click more sections, or Apply / Enter");
    case ToolId::Rib: return SICDM_TR_NOOP("Click an open sketch curve above a body: the rib's free edge");
    case ToolId::PlaneSurface: return SICDM_TR_NOOP("Click a closed curve to fill with a flat surface");
    case ToolId::TabulatedCylinder: return SICDM_TR_NOOP("Click a curve to extrude into a surface");
    case ToolId::RuledSurface: return picked == 0 ? SICDM_TR_NOOP("Click the first of two curves") : SICDM_TR_NOOP("Click the second curve");
    case ToolId::BSplinePatch:
        return picked < 2 ? SICDM_TR_NOOP("Click the curves the patch passes through, in order") : SICDM_TR_NOOP("Click more curves, or Apply / Enter");
    case ToolId::CoonsPatch: return SICDM_TR_NOOP("Click four curves meeting end to end");
    case ToolId::GordonSurface:
        if (o.networkSplit < 0) return picked < 2 ? SICDM_TR_NOOP("Click the profiles (curves one way), then Now the Guides") : SICDM_TR_NOOP("More profiles, or Now the Guides");
        return picked - o.networkSplit < 2 ? SICDM_TR_NOOP("Click the guides (the curves crossing them)") : SICDM_TR_NOOP("More guides, or Apply / Enter");
    case ToolId::OffsetSurface: return SICDM_TR_NOOP("Click a surface body to offset");
    default: return {};
    }
}

std::string pickFeatureCommand(ToolId t, const Sketch& sketch, const std::vector<int>& curves, const std::string& body,
                               const PickFeatureOptions& o, std::string* why) {
    auto fail = [&](const std::string& w) {
        if (why) *why = w;
        return std::string{};
    };
    const PickFeatureSpec spec = pickFeatureSpec(t, o);
    if (!isPickFeatureTool(t)) return fail("Not a pick tool");
    if (spec.body) {
        if (body.empty()) return fail(pickFeaturePrompt(t, o, 0));
        return "SURFACE OFFSET " + quoteWord(body) + " DISTANCE " + mm(o.surfaceDistance);
    }
    if (static_cast<int>(curves.size()) < spec.minPicks) return fail(pickFeaturePrompt(t, o, static_cast<int>(curves.size())));
    const auto& all = sketch.curves();
    std::vector<std::string> names;
    for (int c : curves) {
        if (c < 0 || c >= static_cast<int>(all.size())) return fail("A picked curve is gone");
        names.push_back(quoteWord(all[static_cast<std::size_t>(c)].name));
    }
    const std::string keep = o.keepSketch ? " KEEP" : "";
    std::string s = commandName(t);
    switch (t) {
    case ToolId::Revolve:
        s += " " + names[0] + " ANGLE " + num(o.revolveAngle) + " AXIS " + (o.revolveAxis == 2 ? names[1] : o.revolveAxis == 1 ? "U" : "V") + keep;
        break;
    case ToolId::Sweep: s += " " + names[0] + " ALONG " + names[1] + keep; break;
    case ToolId::Helix:
        s += " " + names[0] + " HELIX PITCH " + mm(o.helixPitch) + " TURNS " + num(o.helixTurns) + (o.helixLeft ? " LEFT" : "") + keep;
        break;
    case ToolId::Loft: {
        // Sections drawn on one plane are spaced out along its normal.
        bool onePlane = true;
        for (std::size_t k = 1; k < curves.size(); ++k)
            onePlane = onePlane && samePlane(all[static_cast<std::size_t>(curves[k])], all[static_cast<std::size_t>(curves[0])]);
        for (std::size_t k = 0; k < names.size(); ++k) {
            s += " " + names[k];
            if (onePlane && k > 0) s += " AT " + mm(o.loftSpacing * static_cast<float>(k));
        }
        s += o.loftSmooth ? " SMOOTH" : " RULED";
        s += keep;
        break;
    }
    case ToolId::Rib: s += " " + names[0] + " THICKNESS " + mm(o.ribThickness) + (o.ribFlip ? " FLIP" : "") + keep; break;
    case ToolId::PlaneSurface: s += " PLANAR " + names[0]; break;
    case ToolId::TabulatedCylinder: s += " EXTRUDE " + names[0] + " DISTANCE " + mm(o.surfaceDistance); break;
    case ToolId::RuledSurface:
    case ToolId::BSplinePatch:
        s += " LOFT";
        for (const std::string& n : names) s += " " + n;
        break;
    case ToolId::CoonsPatch:
        s += " BOUNDARY";
        for (const std::string& n : names) s += " " + n;
        break;
    case ToolId::GordonSurface: {
        const int split = o.networkSplit;
        if (split < 2 || static_cast<int>(names.size()) - split < 2) return fail(pickFeaturePrompt(t, o, static_cast<int>(curves.size())));
        s += " NETWORK";
        for (int k = 0; k < static_cast<int>(names.size()); ++k) s += (k == split ? " ACROSS " : " ") + names[static_cast<std::size_t>(k)];
        break;
    }
    default: break;
    }
    return s;
}

} // namespace sicdm

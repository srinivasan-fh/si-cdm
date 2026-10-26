#include "core/HoleWizard.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>

namespace sicdm {

namespace {

constexpr float kIn = 25.4f;

std::string lowerNoSpace(const std::string& s) {
    std::string o;
    for (char c : s)
        if (!std::isspace(static_cast<unsigned char>(c))) o += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return o;
}

std::string num(float v) {
    char b[32];
    std::snprintf(b, sizeof b, "%.4g", static_cast<double>(v));
    return b;
}

} // namespace

const std::vector<HoleSize>& holeSizes() {
    // name, nominal, pitch, tap drill, clearance close/normal/loose, cbore dia, cbore depth, csink dia, csink angle
    static const std::vector<HoleSize> kSizes = [] {
        std::vector<HoleSize> s = {
            {"M1.6", 1.6f, 0.35f, 1.25f, {1.7f, 1.8f, 2.0f}, 3.3f, 1.8f, 3.6f, 90, false},
            {"M2", 2.0f, 0.4f, 1.6f, {2.2f, 2.4f, 2.6f}, 4.3f, 2.4f, 4.4f, 90, false},
            {"M2.5", 2.5f, 0.45f, 2.05f, {2.7f, 2.9f, 3.1f}, 5.0f, 2.9f, 5.5f, 90, false},
            {"M3", 3.0f, 0.5f, 2.5f, {3.2f, 3.4f, 3.6f}, 6.0f, 3.4f, 6.72f, 90, false},
            {"M4", 4.0f, 0.7f, 3.3f, {4.3f, 4.5f, 4.8f}, 8.0f, 4.4f, 8.96f, 90, false},
            {"M5", 5.0f, 0.8f, 4.2f, {5.3f, 5.5f, 5.8f}, 10.0f, 5.4f, 11.2f, 90, false},
            {"M6", 6.0f, 1.0f, 5.0f, {6.4f, 6.6f, 7.0f}, 11.0f, 6.4f, 13.44f, 90, false},
            {"M8", 8.0f, 1.25f, 6.8f, {8.4f, 9.0f, 10.0f}, 15.0f, 8.6f, 17.92f, 90, false},
            {"M10", 10.0f, 1.5f, 8.5f, {10.5f, 11.0f, 12.0f}, 18.0f, 10.6f, 22.4f, 90, false},
            {"M12", 12.0f, 1.75f, 10.2f, {13.0f, 13.5f, 14.5f}, 20.0f, 12.6f, 26.88f, 90, false},
            {"M14", 14.0f, 2.0f, 12.0f, {15.0f, 15.5f, 16.5f}, 24.0f, 14.6f, 30.8f, 90, false},
            {"M16", 16.0f, 2.0f, 14.0f, {17.0f, 17.5f, 18.5f}, 26.0f, 16.6f, 33.6f, 90, false},
            {"M20", 20.0f, 2.5f, 17.5f, {21.0f, 22.0f, 24.0f}, 33.0f, 20.6f, 40.32f, 90, false},
            {"M24", 24.0f, 3.0f, 21.0f, {25.0f, 26.0f, 28.0f}, 40.0f, 24.8f, 48.0f, 90, false},
        };
        // UNC, in inches: major dia, threads per inch, tap drill, clearance close/normal/loose,
        // cbore dia, cbore depth, 82 degree countersink dia.
        struct Inch {
            const char* name;
            float d, tpi, tap, c0, c1, c2, cb, cbd, cs;
        };
        static const Inch kUnc[] = {
            {"#2-56", 0.086f, 56, 0.0700f, 0.0890f, 0.0960f, 0.1040f, 0.141f, 0.086f, 0.172f},
            {"#4-40", 0.112f, 40, 0.0890f, 0.1160f, 0.1285f, 0.1440f, 0.219f, 0.112f, 0.225f},
            {"#6-32", 0.138f, 32, 0.1065f, 0.1440f, 0.1495f, 0.1695f, 0.281f, 0.138f, 0.279f},
            {"#8-32", 0.164f, 32, 0.1360f, 0.1695f, 0.1770f, 0.1910f, 0.3125f, 0.164f, 0.332f},
            {"#10-24", 0.190f, 24, 0.1495f, 0.1960f, 0.2010f, 0.2210f, 0.375f, 0.190f, 0.385f},
            {"1/4-20", 0.250f, 20, 0.2010f, 0.2570f, 0.2660f, 0.2810f, 0.4375f, 0.250f, 0.507f},
            {"5/16-18", 0.3125f, 18, 0.2570f, 0.3230f, 0.3320f, 0.3440f, 0.531f, 0.3125f, 0.635f},
            {"3/8-16", 0.375f, 16, 0.3125f, 0.3860f, 0.3970f, 0.4060f, 0.625f, 0.375f, 0.762f},
            {"1/2-13", 0.500f, 13, 0.4219f, 0.5156f, 0.5312f, 0.5625f, 0.8125f, 0.500f, 1.014f},
            {"5/8-11", 0.625f, 11, 0.5312f, 0.6406f, 0.6562f, 0.6875f, 1.000f, 0.625f, 1.265f},
            {"3/4-10", 0.750f, 10, 0.6562f, 0.7656f, 0.7812f, 0.8125f, 1.1875f, 0.750f, 1.517f},
            {"1-8", 1.000f, 8, 0.8750f, 1.0156f, 1.0312f, 1.0625f, 1.625f, 1.000f, 2.000f},
        };
        for (const Inch& u : kUnc)
            s.push_back({u.name, u.d * kIn, kIn / u.tpi, u.tap * kIn, {u.c0 * kIn, u.c1 * kIn, u.c2 * kIn}, u.cb * kIn, u.cbd * kIn, u.cs * kIn, 82, true});
        return s;
    }();
    return kSizes;
}

const HoleSize* findHoleSize(const std::string& nameIn) {
    std::string n = lowerNoSpace(nameIn);
    if (n.empty()) return nullptr;
    for (const HoleSize& s : holeSizes()) {
        const std::string k = lowerNoSpace(s.name);
        if (n == k) return &s;
        if (!s.inch && n.rfind(k + "x", 0) == 0) { // M6x1: the coarse pitch only
            const float p = std::strtof(n.c_str() + k.size() + 1, nullptr);
            if (std::fabs(p - s.pitch) < 1e-3f) return &s;
        }
        if (s.inch && k[0] == '#' && n == k.substr(1)) return &s; // 10-24 for #10-24
    }
    return nullptr;
}

bool holeProfile(const HoleSpec& h, float through, float margin, std::vector<std::pair<float, float>>& out, std::string* why) {
    auto fail = [&](const char* w) {
        if (why) *why = w;
        return false;
    };
    out.clear();
    const float r = h.diameter * 0.5f;
    if (!(r > 0.0f)) return fail("The hole needs a diameter");
    if (h.depth < 0.0f || !std::isfinite(h.depth)) return fail("DEPTH must not be negative");
    if (h.cboreDia > 0.0f && h.csinkDia > 0.0f) return fail("A hole is counterbored or countersunk, not both");
    if (h.cboreDia > 0.0f && !(h.cboreDia > h.diameter && h.cboreDepth > 0.0f)) return fail("The counterbore must be wider than the hole, and have a depth");
    if (h.csinkDia > 0.0f && !(h.csinkDia > h.diameter && h.csinkAngle > 10.0f && h.csinkAngle < 170.0f))
        return fail("The countersink must be wider than the hole, at 10 to 170 degrees");
    const float csinkDepth = h.csinkDia > 0.0f ? (h.csinkDia - h.diameter) * 0.5f / std::tan(radians(h.csinkAngle * 0.5f)) : 0.0f;
    if (h.depth > 0.0f && (h.cboreDepth >= h.depth || csinkDepth >= h.depth)) return fail("The hole must go deeper than its counterbore or countersink");
    // Bottom: a drill point, a flat bottom, or past the far side.
    if (h.depth > 0.0f) {
        const float tip = h.tipped ? r / std::tan(radians(59.0f)) : 0.0f;
        out.push_back({0.0f, -h.depth - tip});
        if (tip == 0.0f) out.back() = {0.0f, -h.depth};
        out.push_back({r, -h.depth});
    } else {
        out.push_back({0.0f, -through});
        out.push_back({r, -through});
    }
    // Up the wall to the top: the counterbore's step or the countersink's cone.
    if (h.cboreDia > 0.0f) {
        out.push_back({r, -h.cboreDepth});
        out.push_back({h.cboreDia * 0.5f, -h.cboreDepth});
        out.push_back({h.cboreDia * 0.5f, margin});
    } else if (h.csinkDia > 0.0f) {
        out.push_back({r, -csinkDepth});
        out.push_back({h.csinkDia * 0.5f, 0.0f});
        out.push_back({h.csinkDia * 0.5f, margin});
    } else {
        out.push_back({r, margin});
    }
    out.push_back({0.0f, margin});
    return true;
}

std::string holeCallout(const HoleSpec& h) {
    std::string s;
    if (h.threadDia > 0.0f) {
        s = (h.size.empty() ? "thread " + num(h.threadDia) + " x " + num(h.threadPitch) : h.size) + " tapped";
        const float td = h.threadDepth > 0.0f ? h.threadDepth : h.depth;
        s += td > 0.0f ? " " + num(td) + " deep" : " thru";
        s += " (drill " + num(h.diameter) + (h.depth > 0.0f ? " x " + num(h.depth) : std::string(" thru")) + ")";
    } else {
        s = "dia " + num(h.diameter) + (h.depth > 0.0f ? " x " + num(h.depth) + " deep" : std::string(" thru"));
        if (!h.size.empty()) s += " (" + h.size + " clearance)";
    }
    if (h.cboreDia > 0.0f) s += ", counterbore " + num(h.cboreDia) + " x " + num(h.cboreDepth);
    if (h.csinkDia > 0.0f) s += ", countersink " + num(h.csinkDia) + " x " + num(h.csinkAngle) + " deg";
    return s;
}

} // namespace sicdm

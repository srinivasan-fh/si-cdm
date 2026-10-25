#include "core/CamEdit.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace sicdm {

namespace {

std::string num(double v) {
    char b[32];
    std::snprintf(b, sizeof b, "%.6g", v);
    return b;
}

// Options are read to single precision: closer than that is the same value.
bool differ(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return std::isnan(a) != std::isnan(b);
    return std::fabs(a - b) > 1e-6 * std::max({1.0, std::fabs(a), std::fabs(b)});
}

std::string quote(const std::string& s) {
    std::string q = "\"";
    for (char c : s) q += c == '"' ? '\'' : c;
    return q + "\"";
}

// A word with its value, or AUTO when the value is the default (`isAuto`).
struct Args {
    std::string s;
    void add(const std::string& w) { s += " " + w; }
    void value(const char* word, double from, double to, bool isAuto = false) {
        if (!differ(from, to)) return;
        add(std::string(word) + " " + (isAuto ? std::string("AUTO") : num(to)));
    }
};

} // namespace

std::string camEditArgs(const CamOperation& f, const CamOperation& t) {
    Args a;
    if (f.name != t.name) a.add("NAME " + quote(t.name));
    if (f.tool != t.tool) a.add("TOOL " + std::to_string(t.tool));
    a.value("STEPDOWN", f.stepdown, t.stepdown, t.stepdown <= 0);
    a.value("STEPOVER", f.stepover, t.stepover, t.stepover <= 0);
    a.value("LEAVE", f.leave, t.leave);
    a.value("TOP", f.top, t.top, std::isnan(t.top));
    a.value("BOTTOM", f.bottom, t.bottom, std::isnan(t.bottom));
    a.value("ANGLE", f.angle, t.angle);
    if (f.climb != t.climb) a.add(t.climb ? "CLIMB" : "CONVENTIONAL");
    if (f.raster != t.raster) a.add(t.raster ? "RASTER" : "OFFSET");
    if (f.adaptive != t.adaptive) a.add(t.adaptive ? "ADAPTIVE" : "NOADAPTIVE");
    if (f.rest != t.rest) a.add(t.rest ? "REST" : "NOREST");
    if (f.along != t.along) a.add(t.along ? "ALONG" : "AROUND");
    a.value("A", f.indexA, t.indexA);
    a.value("C", f.indexC, t.indexC);
    if (f.side != t.side) a.add(t.side == 0 ? "OUTSIDE" : t.side == 1 ? "INSIDE" : "BOTH");
    if (f.tabs != t.tabs) a.add("TABS " + std::to_string(t.tabs));
    a.value("TABWIDTH", f.tabWidth, t.tabWidth);
    a.value("TABHEIGHT", f.tabHeight, t.tabHeight);
    a.value("LEAD", f.lead, t.lead, t.lead < 0);
    a.value("SCALLOP", f.scallop, t.scallop);
    if (differ(f.minSlope, t.minSlope) || differ(f.maxSlope, t.maxSlope)) a.add("SLOPE " + num(t.minSlope) + " " + num(t.maxSlope));
    a.value("PECK", f.peck, t.peck, t.peck <= 0);
    if (f.cycle != t.cycle || (differ(f.peck, t.peck) && t.peck > 0)) a.add(std::string("CYCLE ") + drillCycleKey(t.cycle)); // after PECK, which sets peck cycles
    a.value("DWELL", f.dwell, t.dwell);
    a.value("MAXTILT", f.maxTilt, t.maxTilt);
    a.value("LEADANGLE", f.leadAngle, t.leadAngle);
    a.value("SIDETILT", f.sideTilt, t.sideTilt);
    a.value("RPM", f.setRpm, t.setRpm, t.setRpm <= 0);
    a.value("FEED", f.setFeed, t.setFeed, t.setFeed <= 0);
    a.value("PLUNGE", f.setPlunge, t.setPlunge, t.setPlunge <= 0);
    if (f.autoHoles != t.autoHoles && t.autoHoles) a.add("HOLES AUTO");
    return a.s;
}

std::string latheEditArgs(const LatheOperation& f, const LatheOperation& t) {
    Args a;
    if (f.name != t.name) a.add("NAME " + quote(t.name));
    if (f.tool != t.tool) a.add("TOOL " + std::to_string(t.tool));
    if (f.inside != t.inside) a.add(t.inside ? "INSIDE" : "OUTSIDE");
    a.value("STEPDOWN", f.stepdown, t.stepdown, t.stepdown <= 0);
    a.value("LEAVE", f.leave, t.leave);
    a.value("LEAVEZ", f.leaveZ, t.leaveZ);
    a.value("FROM", f.zFrom, t.zFrom, std::isnan(t.zFrom));
    a.value("TO", f.zTo, t.zTo, std::isnan(t.zTo));
    a.value("DEPTH", f.depth, t.depth, t.depth <= 0);
    a.value("PECK", f.peck, t.peck);
    a.value("PITCH", f.pitch, t.pitch);
    a.value("DIAMETER", f.threadDiameter, t.threadDiameter, t.threadDiameter <= 0);
    a.value("SPEED", f.speed, t.speed, t.speed <= 0);
    a.value("FEED", f.feedPerRev, t.feedPerRev, t.feedPerRev <= 0);
    a.value("RPM", f.rpm, t.rpm, t.rpm <= 0);
    return a.s;
}

} // namespace sicdm

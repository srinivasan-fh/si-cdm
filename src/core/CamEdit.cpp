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

// A length in millimetres whatever the document's display units are.
std::string mm(double v) { return num(v) + "mm"; }

// A word with its value, or AUTO when the value is the default (`isAuto`). Lengths are
// written in millimetres (a plain number would be read in the document's units).
struct Args {
    std::string s;
    void add(const std::string& w) { s += " " + w; }
    void value(const char* word, double from, double to, bool isAuto = false) {
        if (!differ(from, to)) return;
        add(std::string(word) + " " + (isAuto ? std::string("AUTO") : num(to)));
    }
    void length(const char* word, double from, double to, bool isAuto = false) {
        if (!differ(from, to)) return;
        add(std::string(word) + " " + (isAuto ? std::string("AUTO") : mm(to)));
    }
};

} // namespace

std::string camEditArgs(const CamOperation& f, const CamOperation& t) {
    Args a;
    if (f.name != t.name) a.add("NAME " + quote(t.name));
    if (f.tool != t.tool) a.add("TOOL " + std::to_string(t.tool));
    a.length("STEPDOWN", f.stepdown, t.stepdown, t.stepdown <= 0);
    a.length("STEPOVER", f.stepover, t.stepover, t.stepover <= 0);
    a.length("LEAVE", f.leave, t.leave);
    a.length("TOP", f.top, t.top, std::isnan(t.top));
    a.length("BOTTOM", f.bottom, t.bottom, std::isnan(t.bottom));
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
    a.length("TABWIDTH", f.tabWidth, t.tabWidth);
    a.length("TABHEIGHT", f.tabHeight, t.tabHeight);
    a.length("LEAD", f.lead, t.lead, t.lead < 0);
    a.length("SCALLOP", f.scallop, t.scallop);
    if (differ(f.minSlope, t.minSlope) || differ(f.maxSlope, t.maxSlope)) a.add("SLOPE " + num(t.minSlope) + " " + num(t.maxSlope));
    a.length("PECK", f.peck, t.peck, t.peck <= 0);
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

std::string camToolCommand(const CamTool& t) {
    std::string s = "CAM TOOL " + std::to_string(t.number) + " " + camToolTypeKey(t.type) + " " + mm(t.diameter);
    s += " FLUTES " + std::to_string(t.flutes) + " LENGTH " + mm(t.fluteLength);
    if (t.type == CamToolType::BullNose) s += " CORNER " + mm(t.cornerRadius);
    if (t.type == CamToolType::Drill || t.type == CamToolType::Spot || t.type == CamToolType::Chamfer) s += " ANGLE " + num(t.tipAngle);
    if (t.type == CamToolType::Tap) s += " PITCH " + mm(t.pitch);
    if (t.stickout > 0) s += " STICKOUT " + mm(t.stickout);
    if (t.holderDiameter > 0) s += " HOLDER " + mm(t.holderDiameter);
    s += " HOLDERLENGTH " + mm(t.holderLength);
    s += t.hss ? " HSS" : " CARBIDE";
    s += " VC " + (t.vc > 0 ? num(t.vc) : std::string("AUTO"));
    s += " FZ " + (t.fz > 0 ? num(t.fz) : std::string("AUTO"));
    if (!t.name.empty()) s += " NAME " + quote(t.name);
    if (!t.vendor.empty()) s += " VENDOR " + quote(t.vendor);
    if (!t.productId.empty()) s += " PRODUCT " + quote(t.productId);
    return s;
}

std::string camMachineCommand(const CamMachine& m) {
    std::string s = "CAM MACHINE NAME " + quote(m.name.empty() ? std::string("machine") : m.name);
    s += " LIMITS " + mm(m.travelMin[0]) + "," + mm(m.travelMin[1]) + "," + mm(m.travelMin[2]) + " " + mm(m.travelMax[0]) + "," + mm(m.travelMax[1]) + "," +
         mm(m.travelMax[2]);
    s += m.hasWorkOffset ? " ZERO " + mm(m.workOffset[0]) + "," + mm(m.workOffset[1]) + "," + mm(m.workOffset[2]) : std::string(" ZERO OFF");
    s += std::string(" KINEMATICS ") + m.kinematics();
    auto rotary = [&](const char* axis, bool fitted, double lo, double hi) {
        s += std::string(" ") + axis + " ";
        if (!fitted) s += "OFF";
        else if (lo <= -99999 && hi >= 99999) s += "CONTINUOUS";
        else s += num(lo) + " " + num(hi);
    };
    rotary("A", m.hasA, m.aMin, m.aMax);
    rotary("B", m.hasB, m.bMin, m.bMax);
    rotary("C", m.hasC, m.cMin, m.cMax);
    s += " POCKETS " + std::to_string(m.toolPockets) + " MAXRPM " + num(m.maxRpm) + " MAXFEED " + num(m.maxFeed) + " RAPID " + num(m.rapid) + " TOOLCHANGE " +
         num(m.toolChange);
    return s;
}

std::string latheEditArgs(const LatheOperation& f, const LatheOperation& t) {
    Args a;
    if (f.name != t.name) a.add("NAME " + quote(t.name));
    if (f.tool != t.tool) a.add("TOOL " + std::to_string(t.tool));
    if (f.inside != t.inside) a.add(t.inside ? "INSIDE" : "OUTSIDE");
    a.length("STEPDOWN", f.stepdown, t.stepdown, t.stepdown <= 0);
    a.length("LEAVE", f.leave, t.leave);
    a.length("LEAVEZ", f.leaveZ, t.leaveZ);
    a.length("FROM", f.zFrom, t.zFrom, std::isnan(t.zFrom));
    a.length("TO", f.zTo, t.zTo, std::isnan(t.zTo));
    a.length("DEPTH", f.depth, t.depth, t.depth <= 0);
    a.length("PECK", f.peck, t.peck);
    a.length("PITCH", f.pitch, t.pitch);
    a.length("DIAMETER", f.threadDiameter, t.threadDiameter, t.threadDiameter <= 0);
    a.value("SPEED", f.speed, t.speed, t.speed <= 0);
    a.value("FEED", f.feedPerRev, t.feedPerRev, t.feedPerRev <= 0);
    a.value("RPM", f.rpm, t.rpm, t.rpm <= 0);
    return a.s;
}

} // namespace sicdm

// The CAM command: setup, tool library, operations, simulation and post.

#include "core/CamCommand.hpp"

#include "core/Cam.hpp"
#include "core/CamVerify.hpp"
#include "core/FileIO.hpp"
#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Units.hpp"

#include <algorithm>
#include <sstream>
#include <optional>
#include <fstream>
#include <cctype>
#include <cmath>
#include <cstdio>

namespace sicdm {

namespace {

std::string lower(const std::string& s) {
    std::string o;
    for (char c : s) o += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return o;
}

std::string num(double v, const char* f = "%g") {
    char b[64];
    std::snprintf(b, sizeof(b), f, v);
    return b;
}

std::string minutes(double s) {
    const int t = static_cast<int>(std::lround(s));
    char b[48];
    if (t >= 3600) std::snprintf(b, sizeof(b), "%d h %02d min", t / 3600, (t / 60) % 60);
    else std::snprintf(b, sizeof(b), "%d min %02d s", t / 60, t % 60);
    return b;
}

int findBodyNamed(const Document& d, const std::string& name) {
    for (std::size_t i = 0; i < d.scene.bodies().size(); ++i)
        if (lower(d.scene.bodies()[i].name) == lower(name)) return static_cast<int>(i);
    return -1;
}

// ISO metric coarse pitch for a tap's size.
double coarsePitch(double d) {
    static const double kTable[][2] = {{1.6, 0.35}, {2, 0.4},   {2.5, 0.45}, {3, 0.5},  {4, 0.7},  {5, 0.8},  {6, 1.0},
                                       {8, 1.25},   {10, 1.5},  {12, 1.75},  {14, 2.0}, {16, 2.0}, {20, 2.5}, {24, 3.0}};
    double best = 1.0, bd = 1e9;
    for (const auto& r : kTable)
        if (std::fabs(r[0] - d) < bd) bd = std::fabs(r[0] - d), best = r[1];
    return best;
}

const char* opLabel(CamOpKind k) {
    switch (k) {
    case CamOpKind::Face: return "Face";
    case CamOpKind::Contour: return "Contour";
    case CamOpKind::Pocket: return "Pocket";
    case CamOpKind::Rough: return "Rough";
    case CamOpKind::Parallel: return "Parallel finish";
    case CamOpKind::Waterline: return "Waterline finish";
    case CamOpKind::Drill: return "Drill";
    case CamOpKind::Bore: return "Helical bore";
    case CamOpKind::Rotary: return "Rotary (4-axis)";
    }
    return "Op";
}

// The tool an operation uses when none is named: the first in the library that suits it.
int defaultTool(const CamJob& job, CamOpKind k, DrillCycle cycle) {
    auto first = [&](std::initializer_list<CamToolType> types) {
        for (CamToolType t : types)
            for (const CamTool& tool : job.tools)
                if (tool.type == t) return tool.number;
        return -1;
    };
    switch (k) {
    case CamOpKind::Face: {
        const int f = first({CamToolType::FaceMill});
        if (f >= 0) return f;
        const CamTool* big = nullptr;
        for (const CamTool& t : job.tools)
            if (t.type == CamToolType::Flat && (!big || t.diameter > big->diameter)) big = &t;
        return big ? big->number : -1;
    }
    case CamOpKind::Rough:
    case CamOpKind::Pocket:
    case CamOpKind::Contour:
    case CamOpKind::Bore: return first({CamToolType::Flat, CamToolType::BullNose});
    case CamOpKind::Parallel:
    case CamOpKind::Waterline:
    case CamOpKind::Rotary: return first({CamToolType::Ball, CamToolType::BullNose, CamToolType::Flat});
    case CamOpKind::Drill:
        if (cycle == DrillCycle::Spot) return first({CamToolType::Spot, CamToolType::Chamfer, CamToolType::Drill});
        if (cycle == DrillCycle::Tap) return first({CamToolType::Tap});
        return first({CamToolType::Drill});
    }
    return -1;
}

std::string setupText(const CamJob& job) {
    const CamSetup& su = job.setup;
    const CamMaterial* m = camMaterial(su.material);
    const Vec3 s = su.stockMax - su.stockMin;
    std::string t = "Setup: " + su.body + " in " + (m ? m->name : su.material) + " stock " + lengthsText({s.x, s.y, s.z}, " x ", "", "");
    t += ", work zero G54 at the ";
    t += job.multiAxis() ? "rotary centre (the stock's centre; the job turns the part)"
         : su.origin == CamSetup::Origin::Model ? "model origin" : su.origin == CamSetup::Origin::StockTopCentre ? "stock top centre" : "stock top front left corner";
    t += ", post " + su.post;
    return t;
}

std::string feedsText(const CamTool& t, const CamOperation* op, const CamJob& job) {
    if (op && op->generated)
        return "S" + num(std::round(op->rpm)) + " F" + num(std::round(op->feed)) + " (plunge " + num(std::round(op->plunge)) + ") mm/min";
    const CamMaterial* m = camMaterial(job.setup.material);
    if (!m) return "";
    const CamFeeds f = camFeeds(t, *m, job.setup.machine);
    return "S" + num(std::round(f.rpm)) + " F" + num(std::round(f.feed)) + " (plunge " + num(std::round(f.plunge)) + ") mm/min, " +
           num(f.chipLoad, "%.3f") + (t.type == CamToolType::Drill || t.type == CamToolType::Spot || t.type == CamToolType::Tap ? " mm/rev" : " mm/tooth");
}

std::string opText(const CamJob& job, std::size_t i) {
    const CamOperation& op = job.ops[i];
    const CamTool* t = job.tool(op.tool);
    std::string s = std::to_string(i + 1) + ". " + (op.name.empty() ? std::string(opLabel(op.kind)) : op.name) + " with " +
                    (t ? t->describe().substr(0, t->describe().find(',')) : "T" + std::to_string(op.tool) + " (missing)");
    if (op.indexA != 0 || op.indexC != 0) s += ", table at A" + num(op.indexA) + " C" + num(op.indexC);
    if (op.kind == CamOpKind::Rotary) s += op.along ? ", along the axis" : ", helical";
    if (op.adaptive) s += ", adaptive";
    if (op.rest) s += ", rest";
    if (op.kind == CamOpKind::Drill) s += std::string(", ") + drillCycleKey(op.cycle) + ", " + std::to_string(op.holes.size()) + " holes";
    if (op.generated) s += ": " + op.summary + ", " + feedsText(*t, &op, job);
    else s += ": not generated";
    return s;
}

} // namespace

CommandOutput runCamCommand(Document& doc, const std::vector<std::string>& a, const std::vector<bool>& quoted) {
    CommandOutput out;
    CamJob& job = doc.state.cam;
    auto quotedArg = [&](std::size_t k) { return k < quoted.size() && quoted[k]; };
    const std::string sub = a.empty() ? "list" : lower(a[0]);
    auto part = [&]() -> Body* {
        const int i = findBodyNamed(doc, job.setup.body);
        return i < 0 ? nullptr : &doc.scene.bodies()[static_cast<std::size_t>(i)];
    };
    // Paths made for an earlier shape of the part are remade.
    auto freshen = [&](Body& p) {
        const double key = camPartKey(p);
        if (key != job.partKey)
            for (CamOperation& op : job.ops) op.generated = false;
        job.partKey = key;
    };
    auto needPart = [&]() -> Body* {
        if (!job.setup.hasStock) {
            out.fail("Set up the part and stock first: CAM SETUP [body] [STOCK margin]");
            return nullptr;
        }
        Body* p = part();
        if (!p) out.fail("The CAM part '" + job.setup.body + "' is gone: CAM SETUP again");
        else freshen(*p);
        return p;
    };
    auto changed = [&] { doc.dirty = true; };

    if (sub == "list" || sub == "status") {
        if (job.empty()) {
            out.say("No CAM job. Start with CAM SETUP [body] [STOCK margin] [MATERIAL aluminium], then CAM TOOL 1 FLAT 6");
            return out;
        }
        if (job.setup.hasStock) out.say(setupText(job));
        out.say(std::to_string(job.tools.size()) + " tools, " + std::to_string(job.ops.size()) + " operations");
        for (const CamTool& t : job.tools) out.say("  " + t.describe());
        for (std::size_t i = 0; i < job.ops.size(); ++i) out.say("  " + opText(job, i));
        return out;
    }

    if (sub == "setup") {
        static const char* const kKeys[] = {"stock", "top",       "from",   "material", "origin", "clearance", "retract",
                                            "tolerance", "maxrpm", "maxfeed", "rapid",  "post",   "to"};
        auto isKey = [&](std::size_t k) {
            if (quotedArg(k)) return false;
            const std::string w = lower(a[k]);
            for (const char* q : kKeys)
                if (w == q) return true;
            return false;
        };
        std::size_t i = 1;
        std::string name;
        if (i < a.size() && quotedArg(i)) name = a[i++];
        else
            while (i < a.size() && !isKey(i)) name += (name.empty() ? "" : " ") + a[i++];
        // No name: the selection, else the part already set up.
        int bi = !name.empty() ? findBodyNamed(doc, name)
                 : doc.selectedBody >= 0 ? doc.selectedBody
                 : job.setup.hasStock ? findBodyNamed(doc, job.setup.body)
                                      : -1;
        if (bi < 0 || bi >= static_cast<int>(doc.scene.bodies().size()))
            return out.fail(name.empty() ? "Select the part, or name it: CAM SETUP \"Part\"" : "No body named " + name);
        const Body& p = doc.scene.bodies()[static_cast<std::size_t>(bi)];
        if (p.mesh.indices.empty()) return out.fail(p.name + " has no surface");
        CamSetup su = job.setup;
        su.body = p.name;
        double margin = 2.0, top = -1.0;
        bool stockGiven = false;
        std::optional<Vec3> size, from, to;
        for (; i < a.size(); ++i) {
            const std::string k = lower(a[i]);
            auto next = [&]() -> std::string { return i + 1 < a.size() ? a[++i] : std::string(); };
            if (k == "stock" || k == "top" || k == "from" || k == "to") stockGiven = true;
            if (k == "stock") {
                const std::string v = next();
                if (const auto s = parseVec3(v)) size = s;
                else if (const auto m = parseLength(v); m && *m >= 0) margin = *m;
                else return out.fail("STOCK needs a margin or a size x,y,z");
            } else if (k == "top") {
                const auto v = parseLength(next());
                if (!v || *v < 0) return out.fail("TOP needs the stock above the part");
                top = *v;
            } else if (k == "from") {
                from = parseVec3(next());
                if (!from) return out.fail("FROM needs x,y,z");
            } else if (k == "to") {
                to = parseVec3(next());
                if (!to) return out.fail("TO needs x,y,z");
            } else if (k == "material") {
                const std::string v = next();
                if (!camMaterial(v)) {
                    std::string list;
                    for (const CamMaterial& m : camMaterials()) list += (list.empty() ? "" : ", ") + m.key;
                    return out.fail("Unknown material '" + v + "' (" + list + ")");
                }
                su.material = camMaterial(v)->key;
            } else if (k == "origin") {
                const std::string v = lower(next());
                if (v == "corner") su.origin = CamSetup::Origin::StockTopCorner;
                else if (v == "centre" || v == "center") su.origin = CamSetup::Origin::StockTopCentre;
                else if (v == "model") su.origin = CamSetup::Origin::Model;
                else return out.fail("ORIGIN is CORNER, CENTRE or MODEL");
            } else if (k == "post") {
                const std::string v = lower(next());
                if (std::find(camPosts().begin(), camPosts().end(), v) == camPosts().end()) return out.fail("POST is fanuc, haas, grbl or linuxcnc");
                su.post = v;
            } else {
                const std::string v = next();
                const bool isLen = k == "clearance" || k == "retract" || k == "tolerance";
                std::optional<double> n;
                if (isLen) n = parseLength(v);
                else if (const auto f = parseNumber(v)) n = *f;
                if (!n || !(*n > 0)) return out.fail(toUpper(k) + " needs a positive value");
                if (k == "clearance") su.clearance = *n;
                else if (k == "retract") su.retract = *n;
                else if (k == "tolerance") su.tolerance = std::clamp(*n, 0.0005, 0.5);
                else if (k == "maxrpm") su.machine.maxRpm = *n;
                else if (k == "maxfeed") su.machine.maxFeed = *n;
                else if (k == "rapid") su.machine.rapid = *n;
            }
        }
        const AABB b = p.bounds();
        // The same part with no stock given (only options changed): the stock stays.
        const bool keepStock = !stockGiven && job.setup.hasStock && job.setup.body == p.name;
        if (!keepStock) {
            if (from || to) {
                if (!from || !to) return out.fail("Give the stock as FROM x,y,z TO x,y,z");
                su.stockMin = {std::min(from->x, to->x), std::min(from->y, to->y), std::min(from->z, to->z)};
                su.stockMax = {std::max(from->x, to->x), std::max(from->y, to->y), std::max(from->z, to->z)};
            } else if (size) {
                if (!(size->x > 0 && size->y > 0 && size->z > 0)) return out.fail("The stock size must be positive");
                const Vec3 c = b.center();
                su.stockMin = {c.x - size->x * 0.5f, c.y - size->y * 0.5f, b.min.z};
                su.stockMax = {c.x + size->x * 0.5f, c.y + size->y * 0.5f, b.min.z + size->z};
            } else {
                const float m = static_cast<float>(margin), t = static_cast<float>(top >= 0 ? top : std::min(margin, 1.0));
                su.stockMin = {b.min.x - m, b.min.y - m, b.min.z};
                su.stockMax = {b.max.x + m, b.max.y + m, b.max.z + t};
            }
        }
        if (!(su.stockMax.x > su.stockMin.x && su.stockMax.y > su.stockMin.y && su.stockMax.z > su.stockMin.z)) return out.fail("The stock is empty");
        su.hasStock = true;
        job.setup = su;
        for (CamOperation& op : job.ops) op.generated = false;
        changed();
        out.say(setupText(job));
        if (su.stockMax.z < b.max.z - 1e-3 || su.stockMin.x > b.min.x + 1e-3 || su.stockMax.x < b.max.x - 1e-3 || su.stockMin.y > b.min.y + 1e-3 ||
            su.stockMax.y < b.max.y - 1e-3)
            out.say("  note: the part sticks out of the stock");
        return out;
    }

    if (sub == "tool") {
        if (a.size() >= 3 && (lower(a[1]) == "delete" || lower(a[1]) == "remove")) {
            const auto n = parseNumber(lower(a[2]).substr(lower(a[2])[0] == 't' ? 1 : 0));
            const auto it = std::find_if(job.tools.begin(), job.tools.end(), [&](const CamTool& t) { return n && t.number == static_cast<int>(*n); });
            if (it == job.tools.end()) return out.fail("No tool " + a[2]);
            out.say("Removed " + it->describe());
            job.tools.erase(it);
            changed();
            return out;
        }
        if (a.size() < 4) return out.fail("Usage: CAM TOOL n FLAT|BALL|BULL|FACE|DRILL|SPOT|CHAMFER|TAP diameter [CORNER r] [FLUTES n] [LENGTH l] [ANGLE deg] [PITCH p] [STICKOUT l] [HOLDER d] [HOLDERLENGTH l] [HSS] [NAME \"x\"]");
        CamTool t;
        const std::string ns = lower(a[1]);
        const auto n = parseNumber(ns.substr(!ns.empty() && ns[0] == 't' ? 1 : 0));
        if (!n || *n < 1 || *n > 999) return out.fail("The tool number is 1 to 999 (T1, T2...)");
        t.number = static_cast<int>(*n);
        if (!camToolTypeFromKey(lower(a[2]), &t.type)) return out.fail("Tool type is flat, ball, bull, face, drill, spot, chamfer or tap");
        const auto d = parseLength(a[3]);
        if (!d || !(*d > 0.05) || *d > 300) return out.fail("The diameter must be 0.05 to 300 mm");
        t.diameter = *d;
        // Defaults by type.
        t.flutes = t.type == CamToolType::FaceMill ? 5 : t.type == CamToolType::Flat || t.type == CamToolType::BullNose || t.type == CamToolType::Tap ? 3 : 2;
        t.fluteLength = t.type == CamToolType::FaceMill ? 6.0 : t.type == CamToolType::Drill ? t.diameter * 5 : t.diameter * 3;
        t.tipAngle = t.type == CamToolType::Drill ? 118.0 : 90.0;
        if (t.type == CamToolType::Tap) t.pitch = coarsePitch(t.diameter);
        if (t.type == CamToolType::BullNose) t.cornerRadius = std::min(1.0, t.diameter * 0.25);
        for (std::size_t i = 4; i < a.size(); ++i) {
            const std::string k = lower(a[i]);
            auto next = [&]() -> std::string { return i + 1 < a.size() ? a[++i] : std::string(); };
            if (k == "hss") t.hss = true;
            else if (k == "carbide") t.hss = false;
            else if (k == "name") t.name = next();
            else if (k == "flutes") {
                const auto v = parseNumber(next());
                if (!v || *v < 1 || *v > 20) return out.fail("FLUTES is 1 to 20");
                t.flutes = static_cast<int>(*v);
            } else if (k == "angle") {
                const auto v = parseNumber(next());
                if (!v || *v < 20 || *v > 179) return out.fail("ANGLE is 20 to 179 degrees");
                t.tipAngle = *v;
            } else if (k == "stickout" || k == "holder" || k == "holderlength") {
                const auto v = parseLength(next());
                if (!v || *v <= 0) return out.fail(toUpper(k) + " needs a length");
                if (k == "stickout") t.stickout = *v;
                else if (k == "holder") t.holderDiameter = *v;
                else t.holderLength = *v;
            } else if (k == "corner" || k == "length" || k == "pitch") {
                const auto v = parseLength(next());
                if (!v || *v < 0) return out.fail(toUpper(k) + " needs a length");
                if (k == "corner") t.cornerRadius = std::min(*v, t.radius());
                else if (k == "length") t.fluteLength = std::max(*v, 0.1);
                else t.pitch = std::max(*v, 0.05);
            } else return out.fail("Unknown tool option " + a[i]);
        }
        const auto it = std::find_if(job.tools.begin(), job.tools.end(), [&](const CamTool& x) { return x.number == t.number; });
        if (it != job.tools.end()) *it = t;
        else job.tools.push_back(t);
        std::sort(job.tools.begin(), job.tools.end(), [](const CamTool& x, const CamTool& y) { return x.number < y.number; });
        for (CamOperation& op : job.ops)
            if (op.tool == t.number) op.generated = false;
        changed();
        out.say(t.describe());
        if (const CamMaterial* m = camMaterial(job.setup.material)) out.say("  in " + m->name + ": " + feedsText(t, nullptr, job));
        return out;
    }

    if (sub == "tools" || sub == "feeds") {
        if (job.tools.empty()) return out.fail("No tools: CAM TOOL 1 FLAT 6");
        const CamMaterial* m = camMaterial(job.setup.material);
        out.say("Tools, feeds and speeds in " + (m ? m->name : job.setup.material) + " (machine max " + num(job.setup.machine.maxRpm) + " rpm):");
        for (const CamTool& t : job.tools) out.say("  " + t.describe() + ": " + feedsText(t, nullptr, job));
        return out;
    }

    if (sub == "holes") {
        Body* p = job.setup.hasStock ? part() : nullptr;
        if (!p) {
            if (doc.selectedBody < 0) return out.fail("CAM SETUP first, or select a body");
            p = &doc.scene.bodies()[static_cast<std::size_t>(doc.selectedBody)];
        }
        const auto holes = findHoles(*p);
        out.say(p->name + ": " + std::to_string(holes.size()) + " round holes");
        for (const CamHole& h : holes)
            out.say("  " + lengthText(h.diameter) + " at " + lengthsText({h.x, h.y}, ", ", "(", ")") + ", " + (h.through ? "through" : "blind") +
                    ", from z " + lengthText(h.top) + " to " + lengthText(h.bottom));
        return out;
    }

    if (sub == "delete" || sub == "remove") {
        const auto n = a.size() > 1 ? parseNumber(a[1]) : std::nullopt;
        if (!n || *n < 1 || *n > static_cast<double>(job.ops.size())) return out.fail("CAM DELETE n: an operation's number (CAM LIST)");
        const std::size_t k = static_cast<std::size_t>(*n) - 1;
        out.say("Removed " + std::to_string(k + 1) + ". " + (job.ops[k].name.empty() ? std::string(opLabel(job.ops[k].kind)) : job.ops[k].name));
        job.ops.erase(job.ops.begin() + static_cast<std::ptrdiff_t>(k));
        changed();
        return out;
    }

    if (sub == "clear") {
        job = CamJob{};
        changed();
        out.say("CAM job cleared");
        return out;
    }

    if (sub == "show" || sub == "hide") {
        job.showPaths = sub == "show";
        out.say(job.showPaths ? "Toolpaths shown" : "Toolpaths hidden");
        return out;
    }

    if (sub == "generate" || sub == "regenerate") {
        Body* p = needPart();
        if (!p) return out;
        for (std::size_t i = 0; i < job.ops.size(); ++i) {
            std::string e;
            if (!generateOperation(job, *p, job.ops[i], &e)) return out.fail(std::to_string(i + 1) + ". " + e);
            out.say("  " + opText(job, i));
        }
        out.say("Generated " + std::to_string(job.ops.size()) + " operations");
        return out;
    }

    if (sub == "simulate" || sub == "sim" || sub == "verify") {
        Body* p = needPart();
        if (!p) return out;
        CamSimulation sim;
        std::string e;
        if (!simulateJob(job, *p, sim, &e)) return out.fail(e);
        const bool show = a.size() > 1 && (lower(a[1]) == "show" || lower(a[1]) == "stock");
        out.say("Simulated " + std::to_string(job.ops.size()) + " operations on a " + lengthText(sim.cell) + " grid:");
        out.say("  removed " + num(sim.removed / 1000.0, "%.1f") + " cm3; stock left on the part: up to " + lengthText(sim.maxLeft) + ", " +
                lengthText(sim.meanLeft) + " on average");
        out.say(sim.gouges ? "  GOUGES: " + std::to_string(sim.gouges) + " points cut into the part, the deepest " + lengthText(sim.deepestGouge)
                           : std::string("  no gouges"));
        out.say(sim.rapidCrashes ? "  RAPID CRASHES: " + std::to_string(sim.rapidCrashes) + " rapid moves through stock" : std::string("  no rapid moves through stock"));
        if (sim.shankHits) out.say("  SHANK: " + std::to_string(sim.shankHits) + " moves cut deeper than a tool's flute length");
        out.say(sim.holderHits ? "  HOLDER: " + std::to_string(sim.holderHits) + " moves bring " + sim.holderTool +
                                     "'s holder into the stock or part; a stickout of " + lengthText(std::ceil(sim.neededStickout)) + " would clear them"
                               : std::string("  no holder collisions"));
        out.say("  cycle time " + minutes(sim.seconds) + ", " + num(sim.cutLength / 1000.0, "%.2f") + " m cutting, " + num(sim.rapidLength / 1000.0, "%.2f") +
                " m rapid");
        if (show) {
            Body stock = simulatedStock(job, sim);
            const int existing = findBodyNamed(doc, stock.name);
            doc.scene.pushUndo("CAM SIMULATE");
            if (existing >= 0) doc.scene.bodies()[static_cast<std::size_t>(existing)] = std::move(stock);
            else doc.scene.bodies().push_back(std::move(stock));
            if (Body* pp = part()) pp->visible = false;
            doc.scene.touchBodies();
            changed();
            out.say("  the cut stock is the body \"Stock (simulated)\" (" + job.setup.body + " hidden)");
        }
        return out;
    }

    // A check's findings, the worst first, at most `limit` of them.
    auto sayIssues = [&](const CamPostCheck& c, std::size_t limit) {
        std::vector<GcodeIssue> list = c.issues;
        std::stable_sort(list.begin(), list.end(), [](const GcodeIssue& x, const GcodeIssue& y) { return x.error && !y.error; });
        for (std::size_t k = 0; k < list.size() && k < limit; ++k)
            out.say(std::string("  ") + (list[k].error ? "ERROR" : "warning") + (list[k].line > 0 ? " line " + std::to_string(list[k].line) : std::string()) +
                    ": " + list[k].text);
        if (list.size() > limit) out.say("  ... " + std::to_string(list.size() - limit) + " more");
    };

    if (sub == "post") {
        // CAM POST path [post] [FORCE]: posted, read back and checked; not written while
        // the check finds errors, unless forced.
        Body* p = needPart();
        if (!p) return out;
        std::string path, post = job.setup.post;
        bool force = false;
        for (std::size_t i = 1; i < a.size(); ++i) {
            const std::string w = lower(a[i]);
            if (!quotedArg(i) && std::find(camPosts().begin(), camPosts().end(), w) != camPosts().end()) post = w;
            else if (!quotedArg(i) && w == "to") continue;
            else if (!quotedArg(i) && w == "force") force = true;
            else path += (path.empty() ? "" : " ") + a[i];
        }
        if (path.empty()) return out.fail("Usage: CAM POST path.nc [FANUC|HAAS|GRBL|LINUXCNC] [FORCE]");
        CamPostReport r;
        std::string e;
        const std::string g = postToString(job, *p, post, &r, &e);
        if (g.empty()) return out.fail(e);
        const CamPostCheck c = checkPost(job, post, g);
        if (c.errors > 0 && !force) {
            out.fail("Not written: reading the " + post + " program back found " + std::to_string(c.errors) + " error" + (c.errors == 1 ? "" : "s") +
                     " (CAM POST ... FORCE writes it anyway)");
            sayIssues(c, 8);
            return out;
        }
        if (!writeFile(path, g, &e)) return out.fail(e);
        out.say("Posted " + std::to_string(job.ops.size()) + " operations for " + post + " to " + path + ": " + std::to_string(r.lines) + " lines, " +
                std::to_string(r.arcs) + " arcs, " + std::to_string(r.cycles) + " drilling cycles, about " + minutes(r.seconds));
        out.say("  " + c.summary() + " on " + job.setup.machine.name);
        sayIssues(c, 8);
        return out;
    }

    if (sub == "check" || sub == "backplot") {
        // CAM CHECK [post] | CAM CHECK file.nc [post]: the program read back as the control
        // would run it, against the job and the machine.
        Body* p = needPart();
        if (!p) return out;
        std::string path, post = job.setup.post;
        for (std::size_t i = 1; i < a.size(); ++i) {
            const std::string w = lower(a[i]);
            if (!quotedArg(i) && std::find(camPosts().begin(), camPosts().end(), w) != camPosts().end()) post = w;
            else path += (path.empty() ? "" : " ") + a[i];
        }
        std::string g, e;
        if (path.empty()) {
            g = postToString(job, *p, post, nullptr, &e);
            if (g.empty()) return out.fail(e);
        } else {
            std::ifstream f(path, std::ios::binary);
            if (!f) return out.fail("Can't read " + path);
            std::ostringstream ss;
            ss << f.rdbuf();
            g = ss.str();
            for (CamOperation& op : job.ops) // the toolpath to hold the program against
                if (!op.generated && !generateOperation(job, *p, op, &e)) return out.fail(e);
        }
        const CamPostCheck c = checkPost(job, post, g);
        out.say((path.empty() ? "The " + post + " program" : path + " (as " + post + ")") + ": " + std::to_string(c.plot.lines) + " lines, " +
                std::to_string(c.plot.arcs) + " arcs, " + std::to_string(c.plot.cycles) + " cycles, " + minutes(c.plot.seconds) + " at the programmed feeds");
        out.say("  " + c.summary() + " on " + job.setup.machine.name);
        sayIssues(c, 20);
        if (c.errors) out.ok = false;
        return out;
    }

    if (sub == "machine") {
        // CAM MACHINE [PRESET name] [TRAVEL x y z] [LIMITS x0,y0,z0 x1,y1,z1] [ZERO x,y,z|OFF]
        //   [A min max|OFF|CONTINUOUS] [C ...] [POCKETS n] [MAXRPM n] [MAXFEED n] [RAPID n] [TOOLCHANGE s] [NAME text]
        CamMachine m = job.setup.machine;
        for (std::size_t i = 1; i < a.size(); ++i) {
            const std::string k = lower(a[i]);
            auto next = [&]() -> std::string { return i + 1 < a.size() ? a[++i] : std::string(); };
            auto number = [&](const char* what) -> std::optional<double> {
                const auto v = parseNumber(next());
                if (!v) out.fail(std::string(what) + " needs a number");
                return v;
            };
            if (k == "preset") {
                const std::string v = next();
                const CamMachine* pm = camMachinePreset(v);
                if (!pm) {
                    std::string list;
                    for (const CamMachine& q : camMachinePresets()) list += (list.empty() ? "" : ", ") + q.name;
                    return out.fail("Unknown machine '" + v + "' (" + list + ")");
                }
                m = *pm;
            } else if (k == "name") {
                m.name = next();
            } else if (k == "travel") {
                for (int ax = 0; ax < 3; ++ax) {
                    const auto v = parseLength(next());
                    if (!v || !(*v > 0)) return out.fail("TRAVEL needs three positive lengths: x y z");
                    m.travelMin[ax] = ax == 2 ? -*v : 0.0;
                    m.travelMax[ax] = ax == 2 ? 0.0 : *v;
                }
            } else if (k == "limits") {
                const auto lo = parseVec3(next()), hi = parseVec3(next());
                if (!lo || !hi) return out.fail("LIMITS needs x0,y0,z0 x1,y1,z1 in machine coordinates");
                const double l[3] = {lo->x, lo->y, lo->z}, h[3] = {hi->x, hi->y, hi->z};
                for (int ax = 0; ax < 3; ++ax) m.travelMin[ax] = std::min(l[ax], h[ax]), m.travelMax[ax] = std::max(l[ax], h[ax]);
            } else if (k == "zero" || k == "g54") {
                const std::string v = next();
                if (lower(v) == "off") m.hasWorkOffset = false;
                else {
                    const auto z = parseVec3(v);
                    if (!z) return out.fail("ZERO needs x,y,z: where G54 is in machine coordinates (or OFF)");
                    m.hasWorkOffset = true;
                    m.workOffset[0] = z->x, m.workOffset[1] = z->y, m.workOffset[2] = z->z;
                }
            } else if (k == "a" || k == "c") {
                const std::string v = lower(next());
                bool& fitted = k == "a" ? m.hasA : m.hasC;
                double& lo = k == "a" ? m.aMin : m.cMin;
                double& hi = k == "a" ? m.aMax : m.cMax;
                if (v == "off" || v == "none") fitted = false;
                else if (v == "continuous" || v == "on") fitted = true, lo = -99999, hi = 99999;
                else {
                    const auto l = parseNumber(v), h = parseNumber(next());
                    if (!l || !h || !(*h > *l)) return out.fail(toUpper(k) + " needs min max (degrees), CONTINUOUS or OFF");
                    fitted = true, lo = *l, hi = *h;
                }
            } else if (k == "pockets") {
                const auto v = number("POCKETS");
                if (!v) return out;
                m.toolPockets = std::max(0, static_cast<int>(*v));
            } else if (k == "maxrpm" || k == "maxfeed" || k == "rapid" || k == "toolchange") {
                const auto v = number(toUpper(k).c_str());
                if (!v) return out;
                if (!(*v > 0)) return out.fail(toUpper(k) + " needs a positive value");
                (k == "maxrpm" ? m.maxRpm : k == "maxfeed" ? m.maxFeed : k == "rapid" ? m.rapid : m.toolChange) = *v;
            } else {
                return out.fail("CAM MACHINE [PRESET name] [TRAVEL x y z] [LIMITS x0,y0,z0 x1,y1,z1] [ZERO x,y,z|OFF] [A min max|CONTINUOUS|OFF] "
                                "[C ...] [POCKETS n] [MAXRPM n] [MAXFEED n] [RAPID n] [TOOLCHANGE s] [NAME text]");
            }
        }
        if (a.size() > 1) {
            job.setup.machine = m;
            for (CamOperation& op : job.ops) op.generated = false; // feeds follow the machine's limits
            changed();
        }
        out.say("Machine " + job.setup.machine.describe());
        if (a.size() <= 1) {
            std::string list;
            for (const CamMachine& q : camMachinePresets()) list += (list.empty() ? "" : ", ") + q.name;
            out.say("  presets: " + list + " (CAM MACHINE PRESET name)");
        }
        return out;
    }

    // Operations.
    CamOpKind kind;
    if (sub == "face") kind = CamOpKind::Face;
    else if (sub == "rough" || sub == "adaptive" || sub == "clearing") kind = CamOpKind::Rough;
    else if (sub == "pocket") kind = CamOpKind::Pocket;
    else if (sub == "contour" || sub == "profile") kind = CamOpKind::Contour;
    else if (sub == "parallel" || sub == "raster" || sub == "finish") kind = CamOpKind::Parallel;
    else if (sub == "waterline" || sub == "zlevel") kind = CamOpKind::Waterline;
    else if (sub == "drill" || sub == "spot" || sub == "tap" || sub == "peck") kind = CamOpKind::Drill;
    else if (sub == "bore" || sub == "helix") kind = CamOpKind::Bore;
    else if (sub == "rotary" || sub == "wrap" || sub == "4axis") kind = CamOpKind::Rotary;
    else
        return out.fail("CAM SETUP | TOOL | TOOLS | FACE | ROUGH | POCKET | CONTOUR | PARALLEL | WATERLINE | DRILL | BORE | ROTARY | HOLES | LIST | "
                        "GENERATE | SIMULATE [SHOW] | POST path [post] | DELETE n | CLEAR | SHOW | HIDE");
    Body* p = needPart();
    if (!p) return out;
    CamOperation op;
    op.kind = kind;
    if (sub == "spot") op.cycle = DrillCycle::Spot;
    if (sub == "tap") op.cycle = DrillCycle::Tap;
    if (sub == "peck") op.cycle = DrillCycle::Peck;
    int tool = -1;
    std::vector<std::pair<double, double>> at;
    double holeD = 0, holeDepth = -1;
    bool through = false;
    for (std::size_t i = 1; i < a.size(); ++i) {
        const std::string k = lower(a[i]);
        auto next = [&]() -> std::string { return i + 1 < a.size() ? a[++i] : std::string(); };
        auto len = [&](const char* what, bool allowNegative = false) -> std::optional<double> {
            const auto v = parseLength(next());
            if (!v || (!allowNegative && *v < 0)) {
                out.fail(std::string(what) + " needs a length");
                return std::nullopt;
            }
            return v;
        };
        auto number = [&](const char* what) -> std::optional<double> {
            const auto v = parseNumber(next());
            if (!v) {
                out.fail(std::string(what) + " needs a number");
                return std::nullopt;
            }
            return static_cast<double>(*v);
        };
        if (k == "tool") {
            std::string v = lower(next());
            if (!v.empty() && v[0] == 't') v = v.substr(1);
            const auto n = parseNumber(v);
            if (!n || !job.tool(static_cast<int>(*n))) return out.fail("No such tool (CAM TOOLS)");
            tool = static_cast<int>(*n);
        } else if (k == "name") op.name = next();
        else if (k == "stepdown" || k == "step") {
            const auto v = len("STEPDOWN");
            if (!v) return out;
            op.stepdown = *v;
        } else if (k == "stepover") {
            const std::string v = next();
            if (!v.empty() && v.back() == '%') {
                const auto pc = parseNumber(v.substr(0, v.size() - 1));
                if (!pc || *pc <= 0 || *pc > 100) return out.fail("STEPOVER % is 1 to 100");
                op.stepover = -*pc; // a share of the tool's diameter, set below
            } else {
                const auto mm = parseLength(v);
                if (!mm || *mm <= 0) return out.fail("STEPOVER needs a length or a percent");
                op.stepover = *mm;
            }
        } else if (k == "leave" || k == "allowance" || k == "stock") {
            const auto v = len("LEAVE");
            if (!v) return out;
            op.leave = *v;
        } else if (k == "top") {
            const auto v = len("TOP", true);
            if (!v) return out;
            op.top = *v;
        } else if (k == "bottom") {
            const auto v = len("BOTTOM", true);
            if (!v) return out;
            op.bottom = *v;
        } else if (k == "depth") {
            const auto v = len("DEPTH");
            if (!v) return out;
            holeDepth = *v;
            if (kind != CamOpKind::Drill && kind != CamOpKind::Bore) op.bottom = job.setup.stockMax.z - *v;
        } else if (k == "angle") {
            const auto v = number("ANGLE");
            if (!v) return out;
            op.angle = *v;
        } else if (k == "conventional") op.climb = false;
        else if (k == "climb") op.climb = true;
        else if (k == "raster" || k == "zigzag") op.raster = true;
        else if (k == "adaptive" || k == "hsm" || k == "trochoidal") op.adaptive = true;
        else if (k == "rest") op.rest = true;
        else if (k == "along") op.along = true;
        else if (k == "around" || k == "helical") op.along = false;
        else if (k == "a" || k == "c" || k == "index" || k == "tilt") {
            // INDEX a [c] | A deg | C deg: the table's turn for a 3+2 operation.
            const auto v = number(k == "c" ? "C" : "A");
            if (!v || std::fabs(*v) > 360) return out.fail(toUpper(k) + " needs an angle, -360 to 360 degrees");
            if (k == "c") op.indexC = *v;
            else op.indexA = *v;
            if (k == "index" && i + 1 < a.size())
                if (const auto c2 = parseNumber(a[i + 1]); c2 && std::fabs(*c2) <= 360) op.indexC = *c2, ++i;
        }
        else if (k == "offset" || k == "spiral") op.raster = false;
        else if (k == "outside") op.side = 0;
        else if (k == "inside") op.side = 1;
        else if (k == "both" || k == "all") op.side = 2;
        else if (k == "tabs") {
            const auto v = number("TABS");
            if (!v || *v < 0 || *v > 32) return out.fail("TABS is 0 to 32");
            op.tabs = static_cast<int>(*v);
        } else if (k == "tabwidth") {
            const auto v = len("TABWIDTH");
            if (!v) return out;
            op.tabWidth = *v;
        } else if (k == "tabheight") {
            const auto v = len("TABHEIGHT");
            if (!v) return out;
            op.tabHeight = *v;
        } else if (k == "lead") {
            const auto v = len("LEAD");
            if (!v) return out;
            op.lead = *v;
        } else if (k == "scallop") {
            const auto v = len("SCALLOP");
            if (!v || *v <= 0) return out.fail("SCALLOP needs a height");
            op.scallop = *v;
        } else if (k == "slope") {
            const auto lo = number("SLOPE"), hi = lo ? number("SLOPE") : std::nullopt;
            if (!lo || !hi) return out.fail("SLOPE needs from and to, in degrees");
            op.minSlope = std::clamp(std::min(*lo, *hi), 0.0, 90.0), op.maxSlope = std::clamp(std::max(*lo, *hi), 0.0, 90.0);
        } else if (k == "steep") op.minSlope = 45, op.maxSlope = 90;
        else if (k == "shallow") op.minSlope = 0, op.maxSlope = 50;
        else if (k == "cycle") {
            if (!drillCycleFromKey(lower(next()), &op.cycle)) return out.fail("CYCLE is spot, drill, peck, chipbreak, tap or bore");
        } else if (k == "peck") {
            const auto v = len("PECK");
            if (!v || *v <= 0) return out.fail("PECK needs a depth");
            op.peck = *v;
            if (op.cycle == DrillCycle::Drill) op.cycle = DrillCycle::Peck;
        } else if (k == "dwell") {
            const auto v = number("DWELL");
            if (!v || *v < 0) return out.fail("DWELL needs seconds");
            op.dwell = *v;
        } else if (k == "at") {
            while (i + 1 < a.size()) {
                const std::string& w = a[i + 1];
                const auto c = w.find(',');
                if (c == std::string::npos) break;
                const auto x = parseLength(w.substr(0, c)), y = parseLength(w.substr(c + 1));
                if (!x || !y) break;
                at.push_back({*x, *y});
                ++i;
            }
            if (at.empty()) return out.fail("AT needs x,y points");
        } else if (k == "diameter") {
            const auto v = len("DIAMETER");
            if (!v || *v <= 0) return out.fail("DIAMETER needs a size");
            holeD = *v;
        } else if (k == "through") through = true;
        else if (k == "rpm") {
            const auto v = number("RPM");
            if (!v || *v <= 0) return out.fail("RPM needs a speed");
            op.setRpm = *v;
        } else if (k == "feed") {
            const auto v = number("FEED");
            if (!v || *v <= 0) return out.fail("FEED needs mm/min");
            op.setFeed = *v;
        } else if (k == "plunge") {
            const auto v = number("PLUNGE");
            if (!v || *v <= 0) return out.fail("PLUNGE needs mm/min");
            op.setPlunge = *v;
        } else return out.fail("Unknown option " + a[i] + " for CAM " + toUpper(sub));
        if (!out.ok) return out;
    }
    if (tool < 0) tool = defaultTool(job, kind, op.cycle);
    if (tool < 0) return out.fail(std::string("No tool for ") + opLabel(kind) + " in the library: add one with CAM TOOL");
    op.tool = tool;
    const CamTool* t = job.tool(tool);
    if (op.stepover < 0) op.stepover = t->diameter * -op.stepover / 100.0;
    if (!at.empty()) { // holes given
        op.autoHoles = false;
        for (const auto& [x, y] : at) {
            CamTool probe; // as wide as the hole: its rim is the top (the middle may be open already)
            probe.diameter = std::max(holeD > 0 ? holeD : t->diameter, 0.1) * 1.1;
            const double topZ = cutterHeight(*p, probe, x, y, -1e9);
            if (topZ < -1e8) return out.fail("No part at " + lengthsText({x, y}, ", ", "(", ")"));
            CamHole h;
            h.x = x, h.y = y, h.top = topZ;
            h.diameter = holeD > 0 ? holeD : t->diameter;
            h.through = through || holeDepth < 0;
            h.bottom = holeDepth >= 0 && !through ? topZ - holeDepth : p->bounds().min.z;
            op.holes.push_back(h);
        }
    } else if (holeDepth >= 0 && (kind == CamOpKind::Drill || kind == CamOpKind::Bore)) {
        // Found holes, to a depth.
        for (const CamHole& h : findHoles(*p)) {
            CamHole c = h;
            c.bottom = h.top - holeDepth;
            c.through = false;
            op.holes.push_back(c);
        }
        op.autoHoles = false;
        CamOperation probe = op;
        probe.autoHoles = true;
        std::string e;
        if (generateOperation(job, *p, probe, &e)) { // keep only the holes the tool fits
            std::vector<CamHole> keep;
            for (const CamHole& h : op.holes)
                for (const CamHole& f : probe.holes)
                    if (std::hypot(h.x - f.x, h.y - f.y) < 1e-3) keep.push_back(h);
            op.holes = keep;
        }
    }
    std::string e;
    if (!generateOperation(job, *p, op, &e)) return out.fail(std::string(opLabel(kind)) + ": " + e);
    job.ops.push_back(op);
    changed();
    out.say("Added " + opText(job, job.ops.size() - 1));
    return out;
}

} // namespace sicdm

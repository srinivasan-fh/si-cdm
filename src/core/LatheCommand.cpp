// The LATHE command: turning setup, tools, operations, simulation and post.

#include "core/LatheCommand.hpp"

#include "core/Cam.hpp"
#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/FileIO.hpp"
#include "core/Lathe.hpp"
#include "core/Units.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <cstdio>
#include <fstream>

namespace sicdm {

namespace {

std::string lower(const std::string& s) {
    std::string o;
    for (char c : s) o += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return o;
}

std::string upper(const std::string& s) {
    std::string o;
    for (char c : s) o += static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return o;
}

std::string num(double v, const char* f = "%g") {
    char b[64];
    std::snprintf(b, sizeof b, f, v);
    return b;
}

std::string minutes(double s) {
    const int t = static_cast<int>(std::lround(s));
    char b[48];
    if (t >= 3600) std::snprintf(b, sizeof b, "%d h %02d min", t / 3600, (t / 60) % 60);
    else std::snprintf(b, sizeof b, "%d min %02d s", t / 60, t % 60);
    return b;
}

int findBodyNamed(const Document& d, const std::string& name) {
    for (std::size_t i = 0; i < d.scene.bodies().size(); ++i)
        if (lower(d.scene.bodies()[i].name) == lower(name)) return static_cast<int>(i);
    return -1;
}

const char* opLabel(const LatheOperation& op) {
    switch (op.kind) {
    case LatheOpKind::Face: return "Face";
    case LatheOpKind::Rough: return op.inside ? "Rough bore" : "Rough turn";
    case LatheOpKind::Finish: return op.inside ? "Finish bore" : "Finish turn";
    case LatheOpKind::Groove: return op.inside ? "Groove the bore" : "Groove";
    case LatheOpKind::Drill: return "Drill";
    case LatheOpKind::Thread: return op.inside ? "Thread the bore" : "Thread";
    case LatheOpKind::PartOff: return "Part off";
    }
    return "Op";
}

int defaultTool(const LatheJob& job, const LatheOperation& op) {
    LatheToolType want = LatheToolType::Turn;
    switch (op.kind) {
    case LatheOpKind::Face: want = LatheToolType::Turn; break;
    case LatheOpKind::Rough:
    case LatheOpKind::Finish: want = op.inside ? LatheToolType::Bore : LatheToolType::Turn; break;
    case LatheOpKind::Groove:
    case LatheOpKind::PartOff: want = LatheToolType::Groove; break;
    case LatheOpKind::Drill: want = LatheToolType::Drill; break;
    case LatheOpKind::Thread: want = LatheToolType::Thread; break;
    }
    // Finishing: the sharpest-nosed insert; else the first.
    const LatheTool* best = nullptr;
    for (const LatheTool& t : job.tools)
        if (t.type == want && (!best || (op.kind == LatheOpKind::Finish && t.noseRadius < best->noseRadius))) best = &t;
    return best ? best->number : -1;
}

std::string opText(const LatheJob& job, std::size_t i) {
    const LatheOperation& op = job.ops[i];
    const LatheTool* t = job.tool(op.tool);
    std::string s = std::to_string(i + 1) + ". " + (op.name.empty() ? opLabel(op) : op.name) + " with " + (t ? t->describe() : "T" + std::to_string(op.tool) + " (missing)");
    if (op.kind == LatheOpKind::Thread && op.pitch > 0) s += ", pitch " + num(op.pitch) + " mm";
    if (op.leave > 0 || op.leaveZ > 0) s += ", leaving " + lengthText(op.leave) + (op.leaveZ > 0 ? " / " + lengthText(op.leaveZ) + " on faces" : "");
    if (op.suppressed) return s + " (suppressed)";
    if (!op.generated) return s;
    double cut = 0, x = 0, z = 0;
    bool first = true;
    for (const LatheMove& m : op.moves) {
        if (!first && m.kind != LatheMove::Rapid) cut += std::hypot(m.x - x, m.z - z);
        x = m.x, z = m.z, first = false;
    }
    s += ": " + std::to_string(op.moves.size()) + " moves, " + num(cut / 1000.0, "%.2f") + " m cutting, " + minutes(latheSeconds(job, op)) + ", ";
    if (op.usedRpm > 0) s += num(std::round(op.usedRpm)) + " rpm";
    else s += num(std::round(op.usedSpeed)) + " m/min (to " + num(job.setup.maxRpm) + " rpm)";
    s += op.kind == LatheOpKind::Thread ? "" : ", " + num(op.usedFeed, "%.3g") + " mm/rev";
    return s;
}

std::string setupText(const LatheJob& job, const LatheProfile* p) {
    const LatheSetup& su = job.setup;
    const CamMaterial* m = camMaterial(su.material);
    std::string s = "Lathe setup: " + su.body + " turning about the model's " + std::string(1, "XYZ"[std::clamp(su.axis, 0, 2)]) + " axis, faced at its " +
                    (su.flip ? "-" : "+") + std::string(1, "XYZ"[std::clamp(su.axis, 0, 2)]) + " end, in " + (m ? m->name : su.material);
    if (p) s += ", from a " + lengthText(latheStockDiameter(job, *p)) + " bar (the part is " + lengthText(2 * p->maxRadius) + " across and " + lengthText(p->length()) + " long)";
    s += ", " + lengthText(su.frontStock) + " to face, post " + su.post;
    return s;
}

} // namespace

CommandOutput runLatheCommand(Document& doc, const std::vector<std::string>& a, const std::vector<bool>& quoted) {
    CommandOutput out;
    LatheJob& job = doc.state.lathe;
    auto changed = [&] { doc.dirty = true; };
    auto quotedArg = [&](std::size_t k) { return k < quoted.size() && quoted[k]; };
    const std::string sub = a.empty() ? "list" : lower(a[0]);
    auto part = [&]() -> Body* {
        const int i = findBodyNamed(doc, job.setup.body);
        return i >= 0 ? &doc.scene.bodies()[static_cast<std::size_t>(i)] : nullptr;
    };
    auto needPart = [&]() -> Body* {
        Body* p = job.setup.body.empty() ? nullptr : part();
        if (!p) out.fail(job.setup.body.empty() ? "Set up the part first: LATHE SETUP [body] [AXIS X|Y|Z]" : "The part " + job.setup.body + " is gone");
        return p;
    };

    if (sub == "list" || sub == "status") {
        if (job.empty()) {
            out.say("No lathe job. Start with LATHE SETUP [body] [AXIS X|Y|Z], then LATHE TOOL 1 TURN NOSE 0.4");
            return out;
        }
        LatheProfile p;
        const Body* b = part();
        if (b && latheProfile(job, *b, p)) out.say(setupText(job, &p));
        else out.say(setupText(job, nullptr));
        out.say(std::to_string(job.tools.size()) + " tools, " + std::to_string(job.ops.size()) + " operations");
        for (const LatheTool& t : job.tools) out.say("  " + t.describe());
        for (std::size_t i = 0; i < job.ops.size(); ++i) out.say("  " + opText(job, i));
        return out;
    }

    if (sub == "setup") {
        static const char* const kKeys[] = {"axis", "flip", "bar", "stock", "front", "material", "post", "maxrpm", "rapid", "clearance", "tolerance"};
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
        const int bi = !name.empty() ? findBodyNamed(doc, name) : doc.selectedBody >= 0 ? doc.selectedBody : !job.setup.body.empty() ? findBodyNamed(doc, job.setup.body) : -1;
        if (bi < 0 || bi >= static_cast<int>(doc.scene.bodies().size()))
            return out.fail(name.empty() ? "Select the part, or name it: LATHE SETUP \"Part\" [AXIS X|Y|Z]" : "No body named " + name);
        const Body& b = doc.scene.bodies()[static_cast<std::size_t>(bi)];
        LatheJob next = job;
        LatheSetup& su = next.setup;
        if (lower(su.body) != lower(b.name)) { // a new part: its axis is its longest side, unless told
            su = LatheSetup{};
            const AABB box = b.bounds();
            const Vec3 s = box.max - box.min;
            // The longest side; on a tie Z, then Y (cylinders are made along Z).
            const double tie = 1e-3 * std::max({s.x, s.y, s.z});
            su.axis = s.z + tie >= s.x && s.z + tie >= s.y ? 2 : s.y + tie >= s.x ? 1 : 0;
        }
        su.body = b.name;
        for (; i < a.size(); ++i) {
            const std::string k = lower(a[i]);
            auto next1 = [&]() -> std::string { return i + 1 < a.size() ? a[++i] : std::string(); };
            if (k == "axis") {
                const std::string v = lower(next1());
                if (v == "x") su.axis = 0;
                else if (v == "y") su.axis = 1;
                else if (v == "z") su.axis = 2;
                else return out.fail("AXIS is X, Y or Z: the model's axis the part turns about");
            } else if (k == "flip") su.flip = !su.flip;
            else if (k == "bar" || k == "stock") {
                const auto v = parseLength(next1());
                if (!v || *v < 0) return out.fail("BAR needs the bar's diameter");
                su.stockDiameter = *v;
            } else if (k == "front") {
                const auto v = parseLength(next1());
                if (!v || *v < 0) return out.fail("FRONT needs the bar in front of the part to face off");
                su.frontStock = *v;
            } else if (k == "material") {
                const std::string v = lower(next1());
                if (!camMaterial(v)) {
                    std::string list;
                    for (const CamMaterial& m : camMaterials()) list += (list.empty() ? "" : ", ") + m.key;
                    return out.fail("Unknown material " + v + " (" + list + ")");
                }
                su.material = v;
            } else if (k == "post") {
                const std::string v = lower(next1());
                if (std::find(lathePosts().begin(), lathePosts().end(), v) == lathePosts().end()) return out.fail("Lathe posts: fanuc, haas, linuxcnc");
                su.post = v;
            } else if (k == "maxrpm") {
                const auto v = parseNumber(next1());
                if (!v || *v <= 0) return out.fail("MAXRPM needs the spindle's limit");
                su.maxRpm = *v;
            } else if (k == "rapid") {
                const auto v = parseNumber(next1());
                if (!v || *v <= 0) return out.fail("RAPID needs mm/min");
                su.rapid = *v;
            } else if (k == "clearance") {
                const auto v = parseLength(next1());
                if (!v || *v < 0.2) return out.fail("CLEARANCE needs a distance (at least 0.2 mm)");
                su.clearance = *v;
            } else if (k == "tolerance") {
                const auto v = parseLength(next1());
                if (!v || *v <= 0) return out.fail("TOLERANCE needs a distance");
                su.tolerance = *v;
            } else return out.fail("Unknown option " + a[i] + " for LATHE SETUP");
        }
        LatheProfile p;
        std::string err;
        if (!latheProfile(next, b, p, &err)) return out.fail(err);
        if (su.stockDiameter > 0 && su.stockDiameter < 2 * p.maxRadius - 1e-6)
            return out.fail("A " + lengthText(su.stockDiameter) + " bar is thinner than the part (" + lengthText(2 * p.maxRadius) + ")");
        job = std::move(next);
        for (LatheOperation& o : job.ops) o.generated = false;
        changed();
        out.say(setupText(job, &p));
        return out;
    }

    if (sub == "tool") {
        // LATHE TOOL n TURN|BORE|GROOVE|THREAD|DRILL [size] [options]
        if (a.size() < 3) return out.fail("LATHE TOOL n TURN [NOSE r] [INSERT C|D|V] | BORE d | GROOVE w | THREAD | DRILL d");
        const auto n = parseNumber(a[1]);
        if (!n || *n < 1 || *n > 99 || std::floor(*n) != *n) return out.fail("The tool number is 1 to 99");
        LatheTool t;
        if (const LatheTool* old = job.tool(static_cast<int>(*n))) t = *old;
        t.number = static_cast<int>(*n);
        LatheToolType type;
        if (!latheToolTypeFromKey(lower(a[2]), &type)) return out.fail("Lathe tools: TURN, BORE, GROOVE, THREAD, DRILL");
        if (type != t.type || !job.tool(t.number)) { // fresh defaults for the type
            const int num0 = t.number;
            t = LatheTool{};
            t.number = num0;
            t.type = type;
            if (type == LatheToolType::Thread) t.noseRadius = 0.1;
            if (type == LatheToolType::Groove) t.noseRadius = 0.2, t.reach = 20;
            if (type == LatheToolType::Bore) t.diameter = 8, t.insertAngle = 55, t.reach = 40;
        }
        std::size_t i = 3;
        if ((type == LatheToolType::Drill || type == LatheToolType::Bore || type == LatheToolType::Groove) && i < a.size()) {
            if (const auto v = parseLength(a[i]); v && *v > 0) {
                (type == LatheToolType::Groove ? t.width : t.diameter) = *v;
                ++i;
            }
        }
        for (; i < a.size(); ++i) {
            const std::string k = lower(a[i]);
            auto next1 = [&]() -> std::string { return i + 1 < a.size() ? a[++i] : std::string(); };
            if (k == "nose" || k == "corner") {
                const auto v = parseLength(next1());
                if (!v || *v < 0 || *v > 5) return out.fail("NOSE is the insert's nose radius, 0 to 5 mm");
                t.noseRadius = *v;
            } else if (k == "insert") {
                const std::string v = lower(next1());
                static const std::pair<const char*, double> kShapes[] = {{"c", 80}, {"w", 80}, {"d", 55}, {"v", 35}, {"t", 60}, {"s", 90}};
                bool found = false;
                for (const auto& [key, ang] : kShapes)
                    if (v == key) t.insertAngle = ang, found = true;
                if (!found) {
                    const auto ang = parseNumber(v);
                    if (!ang || *ang < 20 || *ang > 100) return out.fail("INSERT is C, W (80 deg), D (55), V (35), T (60), S (90) or an angle");
                    t.insertAngle = *ang;
                }
            } else if (k == "width") {
                const auto v = parseLength(next1());
                if (!v || *v <= 0) return out.fail("WIDTH needs the blade's width");
                t.width = *v;
            } else if (k == "diameter") {
                const auto v = parseLength(next1());
                if (!v || *v <= 0) return out.fail("DIAMETER needs a size");
                t.diameter = *v;
            } else if (k == "angle") {
                const auto v = parseNumber(next1());
                if (!v || *v < 60 || *v > 180) return out.fail("ANGLE is the drill's point, 60 to 180 degrees");
                t.tipAngle = *v;
            } else if (k == "reach" || k == "length") {
                const auto v = parseLength(next1());
                if (!v || *v <= 0) return out.fail("REACH needs how deep the tool goes");
                t.reach = *v;
            } else if (k == "hss") t.hss = true;
            else if (k == "carbide") t.hss = false;
            else if (k == "vc" || k == "speed") {
                const auto v = parseNumber(next1());
                if (!v || *v <= 0) return out.fail("VC needs m/min");
                t.vc = *v;
            } else if (k == "feed" || k == "feedrev") {
                const auto v = parseNumber(next1());
                if (!v || *v <= 0 || *v > 2) return out.fail("FEED is mm per turn");
                t.feedPerRev = *v;
            } else if (k == "name") t.name = next1();
            else return out.fail("Unknown option " + a[i] + " for LATHE TOOL");
        }
        if (t.type == LatheToolType::Groove && t.noseRadius > t.width * 0.5) t.noseRadius = t.width * 0.5;
        bool replaced = false;
        for (LatheTool& o : job.tools)
            if (o.number == t.number) o = t, replaced = true;
        if (!replaced) job.tools.push_back(t);
        std::sort(job.tools.begin(), job.tools.end(), [](const LatheTool& x, const LatheTool& y) { return x.number < y.number; });
        for (LatheOperation& o : job.ops)
            if (o.tool == t.number) o.generated = false;
        changed();
        out.say(t.describe());
        if (t.type == LatheToolType::Turn || t.type == LatheToolType::Bore) out.say("  goes down behind shoulders at up to " + num(t.relief()) + " deg");
        return out;
    }

    if (sub == "tools") {
        if (job.tools.empty()) out.say("No lathe tools: LATHE TOOL 1 TURN NOSE 0.8, LATHE TOOL 2 TURN NOSE 0.4, LATHE TOOL 3 GROOVE 3 ...");
        for (const LatheTool& t : job.tools) out.say(t.describe());
        return out;
    }

    if (sub == "profile") {
        Body* p = needPart();
        if (!p) return out;
        LatheProfile pr;
        std::string err;
        if (!latheProfile(job, *p, pr, &err)) return out.fail(err);
        out.say(setupText(job, &pr));
        // The bore, from the front.
        std::size_t i = pr.size();
        double bore = 1e30;
        while (i > 0 && pr.has[i - 1] && pr.inner[i - 1] > 1e-6) bore = std::min(bore, pr.inner[--i]);
        if (i == pr.size()) out.say("  no bore on the axis from the front");
        else out.say("  a bore from the front, " + lengthText(2 * bore) + " across at its narrowest, " + (i == 0 ? std::string("through") : lengthText(-pr.zAt(i)) + " deep"));
        return out;
    }

    if (sub == "clear") {
        job = LatheJob{};
        changed();
        out.say("Lathe job cleared");
        return out;
    }

    if (sub == "delete") {
        const auto n = a.size() > 1 ? parseNumber(a[1]) : std::nullopt;
        if (!n || *n < 1 || *n > static_cast<double>(job.ops.size())) return out.fail("LATHE DELETE n (an operation's number)");
        const std::size_t k = static_cast<std::size_t>(*n) - 1;
        out.say("Deleted " + std::string(opLabel(job.ops[k])));
        job.ops.erase(job.ops.begin() + static_cast<long>(k));
        for (LatheOperation& o : job.ops) o.generated = false; // a drill before a bore may have gone
        changed();
        return out;
    }

    if (sub == "paths") {
        const std::string v = a.size() > 1 ? lower(a[1]) : std::string();
        job.showPaths = v == "on" ? true : v == "off" ? false : !job.showPaths;
        out.say(std::string("Lathe toolpaths ") + (job.showPaths ? "shown" : "hidden"));
        return out;
    }

    if (sub == "generate" || sub == "simulate" || sub == "sim" || sub == "post") {
        Body* p = needPart();
        if (!p) return out;
        latheFrame(job, *p, job.frame);
        if (job.ops.empty()) return out.fail("No lathe operations: LATHE FACE, LATHE ROUGH, LATHE FINISH ...");
        std::string err;
        for (std::size_t i = 0; i < job.ops.size(); ++i) {
            job.ops[i].generated = false;
            if (job.ops[i].suppressed) continue;
            if (!generateLatheOperation(job, *p, job.ops[i], &err)) return out.fail(std::to_string(i + 1) + ". " + opLabel(job.ops[i]) + ": " + err);
        }
        if (sub == "generate") {
            for (std::size_t i = 0; i < job.ops.size(); ++i) out.say(opText(job, i));
            return out;
        }
        if (sub == "post") {
            std::string path, post = job.setup.post;
            bool cycles = false;
            for (std::size_t i = 1; i < a.size(); ++i) {
                const std::string w = lower(a[i]);
                if (!quotedArg(i) && std::find(lathePosts().begin(), lathePosts().end(), w) != lathePosts().end()) post = w;
                else if (!quotedArg(i) && (w == "cycles" || w == "cycle")) cycles = true;
                else if (path.empty()) path = a[i];
                else return out.fail("LATHE POST file.nc [FANUC|HAAS|LINUXCNC] [CYCLES]");
            }
            if (path.empty()) return out.fail("LATHE POST file.nc [FANUC|HAAS|LINUXCNC] [CYCLES]");
            const std::string g = postLathe(job, *p, post, &err, cycles);
            if (g.empty()) return out.fail(err);
            if (!writeFile(path, g, &err)) return out.fail(err);
            std::size_t lines = static_cast<std::size_t>(std::count(g.begin(), g.end(), '\n'));
            double secs = 0;
            for (const LatheOperation& o : job.ops) secs += latheSeconds(job, o) + job.setup.toolChange;
            out.say("Wrote " + path + ": " + post + " lathe program, " + std::to_string(lines) + " lines, " + minutes(secs));
            if (cycles) {
                const auto count = [&](const char* w) {
                    std::size_t c = 0;
                    for (std::size_t at = g.find(w); at != std::string::npos; at = g.find(w, at + 1)) ++c;
                    return c;
                };
                if (post == "linuxcnc") out.say("  LinuxCNC: every pass written out (its G71 takes the outline as a subroutine)");
                else
                    out.say("  " + std::to_string(count("\nG71 P")) + " G71 and " + std::to_string(count("\nG72 P")) + " G72 cycles; " +
                            std::to_string(count("(PASSES WRITTEN OUT")) + " operations written out pass by pass");
            }
            return out;
        }
        const bool show = a.size() > 1 && lower(a[1]) == "show";
        LatheSimulation sim;
        if (!simulateLathe(job, *p, sim, &err)) return out.fail(err);
        out.say("Simulated " + std::to_string(job.ops.size()) + " lathe operations on " + lengthText(sim.dz) + " stations:");
        out.say("  removed " + num(sim.removed / 1000.0, "%.1f") + " cm3; stock left on the part: up to " + lengthText(sim.maxLeft) + ", " + lengthText(sim.meanLeft) +
                " on average");
        out.say(sim.gouges ? "  GOUGES: " + std::to_string(sim.gouges) + " stations cut into the part, up to " + lengthText(sim.deepestGouge) : std::string("  no gouges"));
        out.say(sim.rapidCrashes ? "  RAPID CRASHES: " + std::to_string(sim.rapidCrashes) + " rapid moves through stock" : std::string("  no rapid moves through stock"));
        if (sim.partedOff) out.say("  parted off the bar");
        out.say("  cycle time " + minutes(sim.seconds) + ", " + num(sim.cutLength / 1000.0, "%.2f") + " m cutting, " + num(sim.rapidLength / 1000.0, "%.2f") + " m rapid");
        if (show) {
            Body stock = turnedStock(job, *p, sim);
            const int existing = findBodyNamed(doc, stock.name);
            doc.scene.pushUndo("LATHE SIMULATE");
            if (existing >= 0) doc.scene.bodies()[static_cast<std::size_t>(existing)] = std::move(stock);
            else doc.scene.bodies().push_back(std::move(stock));
            if (Body* pp = part()) pp->visible = false;
            doc.scene.touchBodies();
            changed();
            out.say("  the turned stock is the body \"Stock (turned)\" (" + job.setup.body + " hidden)");
        }
        return out;
    }

    // An operation's number (1-based) from a[1], or -1 after failing.
    auto opIndex = [&](const char* usage) -> int {
        const auto n = a.size() > 1 ? parseNumber(a[1]) : std::nullopt;
        if (!n || *n < 1 || *n > static_cast<double>(job.ops.size()) || *n != std::floor(*n)) {
            out.fail(std::string(usage) + ": an operation's number (LATHE LIST)");
            return -1;
        }
        return static_cast<int>(*n) - 1;
    };
    // Drilling before boring and grooving: what comes after works from what the ones before left.
    auto stale = [&]() {
        for (LatheOperation& o : job.ops) o.generated = false;
    };
    if (sub == "move") {
        const int k = opIndex("LATHE MOVE n UP|DOWN|TO m");
        if (k < 0) return out;
        const std::string how = a.size() > 2 ? lower(a[2]) : std::string();
        int to = how == "up" ? k - 1 : how == "down" ? k + 1 : -1;
        if (how == "to") {
            const auto m = a.size() > 3 ? parseNumber(a[3]) : std::nullopt;
            if (!m) return out.fail("LATHE MOVE n TO m: the place to move it to");
            to = static_cast<int>(*m) - 1;
        } else if (how != "up" && how != "down") return out.fail("LATHE MOVE n UP|DOWN|TO m");
        if (to < 0 || to >= static_cast<int>(job.ops.size())) return out.fail("It can't move further that way");
        LatheOperation moved = std::move(job.ops[static_cast<std::size_t>(k)]);
        job.ops.erase(job.ops.begin() + k);
        job.ops.insert(job.ops.begin() + to, std::move(moved));
        stale();
        changed();
        out.say("Moved to " + opText(job, static_cast<std::size_t>(to)));
        return out;
    }
    if (sub == "suppress" || sub == "unsuppress") {
        const int k = opIndex(sub == "suppress" ? "LATHE SUPPRESS n" : "LATHE UNSUPPRESS n");
        if (k < 0) return out;
        job.ops[static_cast<std::size_t>(k)].suppressed = sub == "suppress";
        stale();
        changed();
        out.say(std::string(sub == "suppress" ? "Suppressed " : "Unsuppressed ") + opText(job, static_cast<std::size_t>(k)));
        return out;
    }
    if (sub == "duplicate" || sub == "copy") {
        const int k = opIndex("LATHE DUPLICATE n");
        if (k < 0) return out;
        LatheOperation c = job.ops[static_cast<std::size_t>(k)];
        if (!c.name.empty()) c.name += " (copy)";
        job.ops.insert(job.ops.begin() + k + 1, std::move(c));
        stale();
        changed();
        out.say("Added " + opText(job, static_cast<std::size_t>(k) + 1));
        return out;
    }

    // Operations: a new one, or LATHE EDIT n with options to change in an existing one.
    int editing = -1;
    LatheOperation op;
    if (sub == "edit" || sub == "set") {
        editing = opIndex("LATHE EDIT n options");
        if (editing < 0) return out;
        op = job.ops[static_cast<std::size_t>(editing)];
    } else if (sub == "face") op.kind = LatheOpKind::Face;
    else if (sub == "rough" || sub == "turn") op.kind = LatheOpKind::Rough;
    else if (sub == "finish") op.kind = LatheOpKind::Finish;
    else if (sub == "bore") op.kind = LatheOpKind::Rough, op.inside = true;
    else if (sub == "groove") op.kind = LatheOpKind::Groove;
    else if (sub == "drill") op.kind = LatheOpKind::Drill;
    else if (sub == "thread") op.kind = LatheOpKind::Thread;
    else if (sub == "partoff" || sub == "part" || sub == "cutoff") op.kind = LatheOpKind::PartOff;
    else
        return out.fail("LATHE SETUP|TOOL|TOOLS|PROFILE|FACE|ROUGH|FINISH|BORE|GROOVE|DRILL|THREAD|PARTOFF|LIST|GENERATE|SIMULATE|POST|EDIT n|MOVE n|SUPPRESS n|"
                        "UNSUPPRESS n|DUPLICATE n|DELETE|CLEAR|PATHS");
    Body* p = needPart();
    if (!p) return out;
    int tool = editing >= 0 ? op.tool : -1;
    for (std::size_t i = editing >= 0 ? 2 : 1; i < a.size(); ++i) {
        const std::string k = lower(a[i]);
        // AUTO puts an option back to its default (from the part, the tool or the material).
        if (i + 1 < a.size() && lower(a[i + 1]) == "auto") {
            const double nan = std::numeric_limits<double>::quiet_NaN();
            if (k == "from") op.zFrom = nan;
            else if (k == "to") op.zTo = nan;
            else if (k == "stepdown" || k == "doc") op.stepdown = 0;
            else if (k == "depth") op.depth = 0;
            else if (k == "diameter") op.threadDiameter = 0;
            else if (k == "speed" || k == "vc") op.speed = 0;
            else if (k == "feed") op.feedPerRev = 0;
            else if (k == "rpm") op.rpm = 0;
            else return out.fail(upper(k) + " has no AUTO");
            ++i;
            continue;
        }
        auto next1 = [&]() -> std::string { return i + 1 < a.size() ? a[++i] : std::string(); };
        auto len = [&](const char* what, bool positive = true) -> std::optional<double> {
            const auto v = parseLength(next1());
            if (!v || (positive && *v < 0)) {
                out.fail(std::string(what) + " needs a length");
                return std::nullopt;
            }
            return v;
        };
        auto number = [&](const char* what) -> std::optional<double> {
            const auto v = parseNumber(next1());
            if (!v || *v <= 0) {
                out.fail(std::string(what) + " needs a positive number");
                return std::nullopt;
            }
            return static_cast<double>(*v);
        };
        if (k == "tool") {
            const auto v = number("TOOL");
            if (!v) return out;
            tool = static_cast<int>(*v);
        } else if (k == "inside" || k == "bore" || k == "id") op.inside = true;
        else if (k == "outside" || k == "od") op.inside = false;
        else if (k == "stepdown" || k == "doc") {
            const auto v = len("STEPDOWN");
            if (!v) return out;
            op.stepdown = *v;
        } else if (k == "leave") {
            const auto v = len("LEAVE");
            if (!v) return out;
            op.leave = *v;
        } else if (k == "leavez") {
            const auto v = len("LEAVEZ");
            if (!v) return out;
            op.leaveZ = *v;
        } else if (k == "from") {
            const auto v = len("FROM", false);
            if (!v) return out;
            op.zFrom = *v;
        } else if (k == "to") {
            const auto v = len("TO", false);
            if (!v) return out;
            op.zTo = *v;
        } else if (k == "depth") {
            const auto v = len("DEPTH");
            if (!v) return out;
            op.depth = *v;
        } else if (k == "peck") {
            const auto v = len("PECK");
            if (!v) return out;
            op.peck = *v;
        } else if (k == "pitch") {
            const auto v = len("PITCH");
            if (!v || *v <= 0) return out.fail("PITCH needs the thread's pitch");
            op.pitch = *v;
        } else if (k == "diameter") {
            const auto v = len("DIAMETER");
            if (!v) return out;
            op.threadDiameter = *v;
        } else if (k == "speed" || k == "vc") {
            const auto v = number("SPEED");
            if (!v) return out;
            op.speed = *v;
        } else if (k == "feed") {
            const auto v = number("FEED");
            if (!v) return out;
            if (*v > 2) return out.fail("FEED is mm per turn");
            op.feedPerRev = *v;
        } else if (k == "rpm") {
            const auto v = number("RPM");
            if (!v) return out;
            op.rpm = *v;
        } else if (k == "name") op.name = next1();
        else return out.fail("Unknown option " + a[i] + " for LATHE " + upper(editing >= 0 ? std::string("edit") : sub));
    }
    if (op.kind == LatheOpKind::Thread && op.pitch <= 0) return out.fail("LATHE THREAD needs a PITCH, e.g. LATHE THREAD PITCH 1.5 FROM 0 TO -20");
    if (op.inside && op.kind != LatheOpKind::Rough && op.kind != LatheOpKind::Finish && op.kind != LatheOpKind::Groove && op.kind != LatheOpKind::Thread)
        return out.fail("INSIDE is for ROUGH, FINISH, GROOVE and THREAD (in the bore)");
    if (tool < 0) tool = defaultTool(job, op);
    if (tool < 0) return out.fail(std::string("No tool for ") + opLabel(op) + ": add one with LATHE TOOL");
    if (!job.tool(tool)) return out.fail("No tool T" + std::to_string(tool));
    op.tool = tool;
    latheFrame(job, *p, job.frame);
    if (editing >= 0) {
        // In its place, so what the operations before it leave is what it works from.
        const std::size_t k = static_cast<std::size_t>(editing);
        LatheOperation old = std::move(job.ops[k]);
        job.ops[k] = op;
        job.ops[k].generated = false;
        std::string err;
        if (!op.suppressed && !generateLatheOperation(job, *p, job.ops[k], &err)) {
            job.ops[k] = std::move(old);
            return out.fail(std::to_string(k + 1) + ". " + opLabel(op) + ": " + err + " (left as it was)");
        }
        for (std::size_t i = k + 1; i < job.ops.size(); ++i) job.ops[i].generated = false;
        changed();
        out.say("Changed " + opText(job, k));
        return out;
    }
    job.ops.push_back(op);
    std::string err;
    if (!generateLatheOperation(job, *p, job.ops.back(), &err)) {
        job.ops.pop_back();
        return out.fail(std::string(opLabel(op)) + ": " + err);
    }
    changed();
    out.say("Added " + opText(job, job.ops.size() - 1));
    return out;
}

} // namespace sicdm

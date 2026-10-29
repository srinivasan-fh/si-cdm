// MOLD: draft and undercut analysis, the parting line, the cavity and the core.

#include "core/MoldCommand.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Mold.hpp"
#include "core/Sculpt.hpp"
#include "core/Units.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <optional>

namespace sicdm {

namespace {

std::string lowerOf(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

std::string num(double v, int digits = 4) {
    char b[32];
    std::snprintf(b, sizeof b, "%.*g", digits, std::fabs(v) < 5e-10 ? 0.0 : v);
    return b;
}

int bodyIndex(const Document& d, const std::string& n) {
    const auto& b = d.scene.bodies();
    for (std::size_t i = 0; i < b.size(); ++i)
        if (b[i].name == n) return static_cast<int>(i);
    for (std::size_t i = 0; i < b.size(); ++i)
        if (lowerOf(b[i].name) == lowerOf(n)) return static_cast<int>(i);
    return -1;
}

std::string freeBodyName(const Document& d, const std::string& base) {
    for (int k = 1;; ++k)
        if (bodyIndex(d, base + " " + std::to_string(k)) < 0) return base + " " + std::to_string(k);
}

std::string freeCurveName(const Document& d, const std::string& base) {
    for (int k = 1;; ++k) {
        const std::string n = base + " " + std::to_string(k);
        const auto& c = d.scene.sketch().curves();
        if (std::none_of(c.begin(), c.end(), [&](const SketchCurve& s) { return lowerOf(s.name) == lowerOf(n); })) return n;
    }
}

// The pull: X, Y, Z (or -X...) or a direction x,y,z.
std::optional<Vec3> pullOf(const std::string& w) {
    const std::string k = lowerOf(w);
    const float sign = !k.empty() && k[0] == '-' ? -1.0f : 1.0f;
    const std::string axis = !k.empty() && (k[0] == '-' || k[0] == '+') ? k.substr(1) : k;
    if (axis == "x") return Vec3{sign, 0, 0};
    if (axis == "y") return Vec3{0, sign, 0};
    if (axis == "z") return Vec3{0, 0, sign};
    const auto v = parseVec3(w);
    if (!v || !(length(*v) > 1e-9f)) return std::nullopt;
    return normalize(*v);
}

std::string pullText(Vec3 d) {
    for (int a = 0; a < 3; ++a) {
        const float c = a == 0 ? d.x : a == 1 ? d.y : d.z;
        if (std::fabs(std::fabs(c) - 1) < 1e-6f) return std::string(c < 0 ? "-" : "+") + static_cast<char>('X' + a);
    }
    return "(" + num(d.x) + ", " + num(d.y) + ", " + num(d.z) + ")";
}

} // namespace

CommandOutput runMoldCommand(Document& doc, const std::vector<std::string>& a, const std::vector<bool>& quoted) {
    CommandOutput out;
    const std::string sub = a.empty() ? std::string() : lowerOf(a[0]);
    const char* usage = "MOLD DRAFT [part] [PULL Z|x,y,z] [MIN deg] | PARTING [part] [PULL ...] [AT h] | SPLIT [part] [PULL ...] [AT h] [MARGIN m] [SHRINK %] [GAP g] | OFF";
    auto& bodies = doc.scene.bodies();
    if (sub == "off") {
        int removed = 0;
        for (std::size_t k = bodies.size(); k-- > 0;) {
            if (bodies[k].field.rfind("draft", 0) != 0) continue;
            const std::string src = bodies[k].attachedTo;
            bodies.erase(bodies.begin() + static_cast<std::ptrdiff_t>(k));
            if (const int s = bodyIndex(doc, src); s >= 0) bodies[static_cast<std::size_t>(s)].visible = true;
            ++removed;
        }
        doc.scene.touchBodies();
        out.say(removed ? "Draft analysis off" : "No draft analysis shown");
        return out;
    }
    if (sub != "draft" && sub != "parting" && sub != "split") return out.fail(std::string("Usage: ") + usage);
    // The part (leading words naming a body, or the selected one), then the options.
    static const char* const kKeys[] = {"pull", "min", "at", "margin", "shrink", "gap"};
    auto isKey = [&](const std::string& w) { return std::any_of(std::begin(kKeys), std::end(kKeys), [&](const char* k) { return lowerOf(w) == k; }); };
    std::size_t i = 1;
    std::string name;
    if (i < a.size() && i < quoted.size() && quoted[i]) {
        name = a[i++];
    } else {
        std::string run, best;
        std::size_t end = i;
        for (std::size_t j = i; j < a.size() && !isKey(a[j]); ++j) {
            run += (run.empty() ? "" : " ") + a[j];
            if (bodyIndex(doc, run) >= 0) best = run, end = j + 1;
        }
        if (!best.empty()) name = best, i = end;
        else if (i < a.size() && !isKey(a[i])) return out.fail("No body named " + a[i]);
    }
    const int bi = name.empty() ? doc.selectedBody : bodyIndex(doc, name);
    if (bi < 0 || bi >= static_cast<int>(bodies.size())) return out.fail(name.empty() ? "Which part? Name it, or select it" : "No body named " + name);
    Vec3 pull{0, 0, 1};
    float minDeg = 1.0f, margin = 0, shrink = 0, gap = 0;
    std::optional<double> at;
    for (; i < a.size(); ++i) {
        const std::string k = lowerOf(a[i]);
        if (i + 1 >= a.size()) return out.fail(std::string("Usage: ") + usage);
        const std::string& w = a[++i];
        if (k == "pull") {
            const auto p = pullOf(w);
            if (!p) return out.fail("PULL is X, Y, Z (-X...) or a direction x,y,z");
            pull = *p;
        } else if (k == "min") {
            const auto v = parseNumber(w);
            if (!v || *v < 0 || *v >= 45) return out.fail("MIN is the least draft in degrees, from 0 to 45");
            minDeg = static_cast<float>(*v);
        } else if (k == "at") {
            const auto v = parseLength(w);
            if (!v) return out.fail("AT is the parting height along the pull");
            at = *v;
        } else if (k == "margin" || k == "gap") {
            const auto v = parseLength(w);
            if (!v || *v < 0) return out.fail(toUpper(k) + " is a length");
            (k == "margin" ? margin : gap) = static_cast<float>(*v);
        } else if (k == "shrink") {
            const auto v = parseNumber(w);
            if (!v || *v < 0 || *v > 20) return out.fail("SHRINK is the part's shrinkage in percent, 0 to 20");
            shrink = static_cast<float>(*v);
        } else {
            return out.fail(std::string("Usage: ") + usage);
        }
    }
    const Body src = bodies[static_cast<std::size_t>(bi)];
    if (!src.field.empty()) return out.fail(src.name + " is an analysis shown on " + src.attachedTo + ": name the part");
    const MeshData world = worldMesh(src);
    std::string why;

    if (sub == "draft") {
        DraftReport rep;
        const std::vector<DraftClass> classes = draftClasses(world, pull, minDeg, &rep);
        const double total = rep.area[0] + rep.area[1] + rep.area[2] + rep.area[3];
        auto share = [&](DraftClass c) { return total > 0 ? num(100.0 * rep.area[static_cast<int>(c)] / total, 3) + "%" : std::string("0%"); };
        // Shown in place of the part (MOLD OFF puts it back).
        for (std::size_t k = bodies.size(); k-- > 0;)
            if (bodies[k].field.rfind("draft", 0) == 0 && bodies[k].attachedTo == src.name) bodies.erase(bodies.begin() + static_cast<std::ptrdiff_t>(k));
        const int si = bodyIndex(doc, src.name);
        Body res = meshBody(freeBodyName(doc, src.name + " draft"), draftColoured(world, classes), src.color);
        res.field = "draft along " + pullText(pull) + ", at least " + num(minDeg) + " deg";
        res.attachedTo = src.name;
        if (si >= 0) bodies[static_cast<std::size_t>(si)].visible = false;
        out.say("Draft of " + src.name + " along " + pullText(pull) + " (at least " + num(minDeg) + " deg), shown on " + res.name + " (MOLD OFF puts " + src.name + " back):");
        out.say("  green, comes away with the cavity: " + share(DraftClass::Positive) + (rep.minPositive < 90 ? ", least draft " + num(rep.minPositive, 3) + " deg" : ""));
        out.say("  red, comes away with the core: " + share(DraftClass::Negative) + (rep.minNegative < 90 ? ", least draft " + num(rep.minNegative, 3) + " deg" : ""));
        out.say("  yellow, less draft than " + num(minDeg) + " deg: " + share(DraftClass::Shallow) + " (" + num(rep.area[static_cast<int>(DraftClass::Shallow)], 4) + " mm2)");
        out.say("  purple, undercut (can't come away): " + share(DraftClass::Undercut) + " (" + num(rep.area[static_cast<int>(DraftClass::Undercut)], 4) + " mm2)");
        doc.scene.add(std::move(res));
        doc.selectedBody = static_cast<int>(bodies.size()) - 1;
        doc.scene.touchBodies();
        doc.dirty = true;
        return out;
    }
    if (sub == "parting") {
        PartingLine pl;
        if (!partingLine(world, pull, at, pl, &why)) return out.fail("No parting line for " + src.name + ": " + why);
        // The loops as closed 3D paths (an undo step of the sketch).
        if (!doc.replaying) {
            doc.scene.pushSketchUndo("MOLD PARTING " + src.name);
            doc.scene.keepSketchUndo();
        }
        std::vector<std::string> names;
        for (const auto& loop : pl.loops) {
            SketchCurve c;
            c.kind = CurveKind::Polyline;
            c.plane = SketchPlane::Top;
            c.name = freeCurveName(doc, "Parting Line");
            for (const Vec3& q : loop) c.pts.push_back({q.x, q.y}), c.rise.push_back(q.z);
            c.pts.push_back(c.pts.front()), c.rise.push_back(c.rise.front());
            names.push_back(c.name);
            doc.scene.sketch().add(std::move(c));
        }
        doc.selectedCurve = static_cast<int>(doc.scene.sketch().curves().size()) - 1;
        doc.selectedBody = -1;
        doc.dirty = true;
        const bool flat = pl.high - pl.low <= 1e-6 * std::max(1.0, std::fabs(pl.high));
        std::string list;
        for (const std::string& n : names) list += (list.empty() ? "" : ", ") + n;
        out.say("Parting line of " + src.name + " along " + pullText(pull) + ": " + std::to_string(pl.loops.size()) + (pl.loops.size() == 1 ? " loop, " : " loops, ") +
                lengthText(pl.length) + (flat ? ", flat at " + lengthText(pl.height) : ", from " + lengthText(pl.low) + " to " + lengthText(pl.high) + " along the pull (shut-offs up and down to " + lengthText(pl.height) + ")") +
                " -> " + list);
        return out;
    }
    // SPLIT
    MoldOptions o;
    o.pull = pull, o.at = at, o.margin = margin, o.shrink = shrink;
    MoldHalves h;
    if (!splitMold(world, o, h, &why)) return out.fail("Cannot make the mould of " + src.name + ": " + why);
    std::string group;
    for (int k = 1; group.empty(); ++k) // a mould group no body is in yet
        if (std::none_of(bodies.begin(), bodies.end(), [&](const Body& b) { return b.assembly == "Mold " + std::to_string(k); })) group = "Mold " + std::to_string(k);
    Body cavity = meshBody(freeBodyName(doc, "Cavity"), std::move(h.cavity), Vec3{0.55f, 0.62f, 0.72f});
    Body core = meshBody(freeBodyName(doc, "Core"), std::move(h.core), Vec3{0.70f, 0.70f, 0.66f});
    if (core.name == cavity.name) core.name += "b";
    for (Body* b : {&cavity, &core}) {
        b->assembly = group;
        b->material.metallic = 0.7f, b->material.roughness = 0.35f;
        b->properties["MATERIAL"] = "Tool steel";
        b->properties["DESCRIPTION"] = std::string(b == &cavity ? "Cavity" : "Core") + " for " + src.name;
    }
    cavity.offset = pull * gap;
    out.say("Mould of " + src.name + " along " + pullText(pull) + ": " + cavity.name + " and " + core.name + " in " + group + ", parted at " + lengthText(h.parting.height) +
            (h.parting.high - h.parting.low > 1e-6 ? " with shut-offs (the parting line runs from " + lengthText(h.parting.low) + " to " + lengthText(h.parting.high) + ")" : ""));
    out.say("  block " + num(h.block / 1000.0, 5) + " cm3: cavity " + num(meshVolume(cavity.mesh) / 1000.0, 5) + " cm3, core " + num(meshVolume(core.mesh) / 1000.0, 5) + " cm3, part " +
            num(h.part / 1000.0, 5) + " cm3" + (shrink > 0 ? " (grown " + num(shrink) + "% for shrinkage)" : ""));
    doc.scene.add(std::move(cavity));
    doc.scene.add(std::move(core));
    doc.selectedBody = static_cast<int>(bodies.size()) - 1;
    doc.selectedCurve = doc.selectedImage = -1;
    doc.scene.touchBodies();
    doc.dirty = true;
    return out;
}

} // namespace sicdm

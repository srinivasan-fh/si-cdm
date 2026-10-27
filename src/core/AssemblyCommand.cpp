// MATE and ASSEMBLY: parts placed by mates, the assembly tree, interference and the parts list.

#include "core/AssemblyCommand.hpp"

#include "core/Assembly.hpp"
#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/FileIO.hpp"
#include "core/Units.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>

namespace sicdm {

namespace {

std::string lowerOf(const std::string& s) {
    std::string o;
    for (char c : s) o += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return o;
}

std::string fmt3(double v) {
    char b[32];
    std::snprintf(b, sizeof b, "%.3g", v);
    return b;
}

int bodyIndex(const Document& doc, const std::string& name) {
    const auto& bs = doc.scene.bodies();
    for (std::size_t i = 0; i < bs.size(); ++i)
        if (bs[i].name == name) return static_cast<int>(i);
    for (std::size_t i = 0; i < bs.size(); ++i)
        if (lowerOf(bs[i].name) == lowerOf(name)) return static_cast<int>(i);
    return -1;
}

std::string mateStatus(const Document& doc, const Mate& m) {
    if (m.suppressed) return "suppressed";
    const double e = mateError(doc.scene, m);
    if (e < 0) return "a body is missing";
    return e < 1e-3 ? "met" : "off by " + fmt3(e) + " mm";
}

// Solves and says what moved, what is left free and what can't be met.
void reportSolve(CommandOutput& out, const MateSolveReport& r) {
    if (!r.moved.empty()) {
        std::string s = "Moved";
        for (std::size_t i = 0; i < r.moved.size() && i < 8; ++i) s += (i ? ", " : " ") + r.moved[i];
        if (r.moved.size() > 8) s += " and " + std::to_string(r.moved.size() - 8) + " more";
        out.say(s);
    }
    if (!r.failing.empty()) {
        std::string s = "Can't meet";
        for (std::size_t i = 0; i < r.failing.size(); ++i) s += (i ? ", " : " ") + r.failing[i];
        out.say(s + ": they conflict with the other mates (off by " + fmt3(r.residual) + " mm)");
    }
    for (const std::string& m : r.missing) out.say(m + " names a body that is gone");
    if (r.failing.empty())
        out.say(r.freedom == 0 ? "Fully placed: no freedom left"
                               : std::to_string(r.freedom) + (r.freedom == 1 ? " degree" : " degrees") + " of freedom left");
}

Mate* findMate(AssemblyState& st, const std::string& key) {
    for (Mate& m : st.mates)
        if (lowerOf(m.name) == lowerOf(key)) return &m;
    char* end = nullptr;
    const long n = std::strtol(key.c_str(), &end, 10);
    if (end && *end == 0 && n >= 1 && n <= static_cast<long>(st.mates.size())) return &st.mates[static_cast<std::size_t>(n - 1)];
    return nullptr;
}

} // namespace

CommandOutput runAssemblyCommand(Document& doc, const std::string& cmd, const std::vector<std::string>& a, const std::vector<bool>& quoted) {
    CommandOutput out;
    AssemblyState& st = doc.state.assembly;
    auto isWord = [&](std::size_t k, const char* w) { return k < a.size() && !(k < quoted.size() && quoted[k]) && lowerOf(a[k]) == w; };

    if (cmd == "ASSEMBLY") {
        const std::string sub = a.empty() ? "tree" : lowerOf(a[0]);
        if (sub == "tree" || sub == "list") {
            if (doc.scene.bodies().empty()) {
                out.say("No parts");
                return out;
            }
            for (const std::string& l : assemblyTree(doc.scene)) out.say(l);
            if (!st.mates.empty()) out.say(std::to_string(st.mates.size()) + (st.mates.size() == 1 ? " mate (MATE lists it)" : " mates (MATE lists them)"));
            return out;
        }
        if (sub == "group" || sub == "ungroup") {
            // ASSEMBLY GROUP "path/to/sub" bodies... | ASSEMBLY UNGROUP bodies...
            const bool group = sub == "group";
            if (a.size() < (group ? 3u : 2u)) return out.fail(group ? "ASSEMBLY GROUP \"name\" body..." : "ASSEMBLY UNGROUP body...");
            std::string path = group ? a[1] : "";
            while (!path.empty() && path.back() == '/') path.pop_back();
            while (!path.empty() && path.front() == '/') path.erase(path.begin());
            if (group && path.empty()) return out.fail("Give the sub-assembly a name");
            std::vector<std::size_t> idx;
            for (std::size_t k = group ? 2 : 1; k < a.size(); ++k) {
                const int i = bodyIndex(doc, a[k]);
                if (i < 0) return out.fail("No body named " + a[k]);
                idx.push_back(static_cast<std::size_t>(i));
            }
            for (std::size_t i : idx) doc.scene.bodies()[i].assembly = path;
            doc.dirty = true;
            out.say(group ? "Put " + std::to_string(idx.size()) + (idx.size() == 1 ? " part in " : " parts in ") + path
                          : "Moved " + std::to_string(idx.size()) + (idx.size() == 1 ? " part" : " parts") + " to the top level");
            return out;
        }
        if (sub == "interference" || sub == "clash" || sub == "check") {
            std::vector<std::size_t> idx;
            for (std::size_t k = 1; k < a.size(); ++k) {
                const int i = bodyIndex(doc, a[k]);
                if (i < 0) return out.fail("No body named " + a[k]);
                idx.push_back(static_cast<std::size_t>(i));
            }
            if (idx.size() == 1) return out.fail("Name two or more bodies, or none for every visible one");
            const auto hits = findInterference(doc.scene, idx);
            if (hits.empty()) {
                out.say("No interference: no parts overlap");
                return out;
            }
            out.say(std::to_string(hits.size()) + (hits.size() == 1 ? " pair overlaps:" : " pairs overlap:"));
            for (const InterferenceVolume& h : hits) {
                char b[200];
                std::snprintf(b, sizeof b, "  %s and %s: %.4g mm3, around (%.2f, %.2f, %.2f), %.2f x %.2f x %.2f mm", h.a.c_str(), h.b.c_str(), h.volume, h.centre.x,
                              h.centre.y, h.centre.z, h.size.x, h.size.y, h.size.z);
                out.say(b);
            }
            return out;
        }
        if (sub == "bom" || sub == "parts") {
            const auto bom = partsList(doc.scene);
            if (bom.empty()) {
                out.say("No parts");
                return out;
            }
            const bool numbers = std::any_of(bom.begin(), bom.end(), [](const BomLine& l) { return !l.number.empty() || !l.description.empty(); });
            out.say(numbers ? "Item  Qty  Part number   Part  Description" : "Item  Qty  Part");
            int i = 1;
            for (const BomLine& l : bom) {
                char b[400];
                if (numbers)
                    std::snprintf(b, sizeof b, "%4d  %3d  %-12s  %s%s%s", i++, l.quantity, l.number.empty() ? "-" : l.number.c_str(), l.part.c_str(),
                                  l.description.empty() ? "" : "  ", l.description.c_str());
                else std::snprintf(b, sizeof b, "%4d  %3d  %s", i++, l.quantity, l.part.c_str());
                out.say(b);
            }
            return out;
        }
        return out.fail("ASSEMBLY [TREE] | GROUP \"name\" body... | UNGROUP body... | INTERFERENCE [body...] | BOM");
    }

    // --- MATE ---
    const std::string sub = a.empty() ? "list" : lowerOf(a[0]);
    if (sub == "list") {
        if (st.mates.empty()) {
            out.say("No mates. MATE COINCIDENT|FLUSH|CONCENTRIC|DISTANCE d|ANGLE deg|PARALLEL|PERPENDICULAR \"A\" AT x,y,z \"B\" AT x,y,z places B against A");
            return out;
        }
        for (const Mate& m : st.mates) out.say(describeMate(m) + ": " + mateStatus(doc, m));
        return out;
    }
    if (sub == "solve") {
        if (st.mates.empty()) return out.fail("No mates to solve");
        const MateSolveReport r = solveMates(doc.scene, st);
        if (!r.moved.empty()) doc.dirty = true;
        reportSolve(out, r);
        if (!r.solved) out.ok = false;
        return out;
    }
    if (sub == "clear") {
        st.mates.clear();
        doc.dirty = true;
        out.say("All mates removed (the parts stay where they are)");
        return out;
    }
    if (sub == "delete" || sub == "suppress" || sub == "unsuppress" || sub == "flip" || sub == "edit") {
        if (a.size() < 2) return out.fail("MATE " + toUpper(sub) + " name|n" + (sub == "edit" ? " value" : ""));
        Mate* m = findMate(st, a[1]);
        if (!m) return out.fail("No mate " + a[1] + " (MATE lists them)");
        const std::string name = m->name;
        if (sub == "delete") {
            st.mates.erase(st.mates.begin() + (m - st.mates.data()));
            doc.dirty = true;
            out.say("Deleted " + name + " (the parts stay where they are)");
            return out;
        }
        if (sub == "suppress" || sub == "unsuppress") m->suppressed = sub == "suppress";
        else if (sub == "flip") {
            if (m->a.geom != MateGeom::Plane || m->b.geom != MateGeom::Plane) return out.fail(name + " is not between two planes: nothing to flip");
            m->aligned = !m->aligned;
        } else {
            if (m->type != MateType::Distance && m->type != MateType::Angle) return out.fail(name + " has no value to edit");
            const auto v = a.size() > 2 ? (m->type == MateType::Distance ? parseLength(a[2]) : std::optional<double>(parseNumber(a[2]))) : std::nullopt;
            if (!v) return out.fail("MATE EDIT " + name + (m->type == MateType::Distance ? " distance" : " degrees"));
            m->value = *v;
        }
        doc.dirty = true;
        out.say(describeMate(*m));
        reportSolve(out, solveMates(doc.scene, st));
        return out;
    }

    // MATE type [value] "A" [AT x,y,z] ["B" AT x,y,z] [FLIP]
    Mate m;
    bool aligned = false;
    if (!mateTypeFromName(a[0], &m.type, &aligned))
        return out.fail("MATE COINCIDENT|FLUSH|CONCENTRIC|DISTANCE|ANGLE|PARALLEL|PERPENDICULAR|FIX ... | LIST | SOLVE | EDIT | FLIP | SUPPRESS | DELETE | CLEAR");
    m.aligned = aligned;
    std::size_t k = 1;
    if (m.type == MateType::Distance || m.type == MateType::Angle) {
        const auto v = k < a.size() ? (m.type == MateType::Distance ? parseLength(a[k]) : std::optional<double>(parseNumber(a[k]))) : std::nullopt;
        if (!v) return out.fail(m.type == MateType::Distance ? "MATE DISTANCE mm \"A\" AT x,y,z \"B\" AT x,y,z" : "MATE ANGLE degrees \"A\" AT x,y,z \"B\" AT x,y,z");
        m.value = *v;
        ++k;
    }
    // The references: a body name (words up to AT), then the point on its face.
    std::vector<MateRef> refs;
    while (k < a.size()) {
        if (isWord(k, "flip")) {
            m.aligned = !m.aligned;
            ++k;
            continue;
        }
        std::string name = a[k++];
        while (k < a.size() && !isWord(k, "at") && !isWord(k, "flip") && !(k - 1 < quoted.size() && quoted[k - 1]) && !(k < quoted.size() && quoted[k]))
            name += " " + a[k++];
        const int bi = bodyIndex(doc, name);
        if (bi < 0) return out.fail("No body named " + name);
        const Body& b = doc.scene.bodies()[static_cast<std::size_t>(bi)];
        MateRef r;
        r.body = b.name;
        if (isWord(k, "at")) {
            const auto p = k + 1 < a.size() ? parseVec3(a[k + 1]) : std::nullopt;
            if (!p) return out.fail("AT takes the point x,y,z on the face (mm)");
            std::string err;
            if (!mateRefAt(b, DVec3(*p), r, &err)) return out.fail(err);
            k += 2;
        } else if (m.type != MateType::Fix) return out.fail("Pick the face of " + b.name + " with AT x,y,z (a point on it)");
        refs.push_back(r);
    }
    if (m.type == MateType::Fix) {
        if (refs.size() != 1) return out.fail("MATE FIX \"body\"");
        for (const Mate& o : st.mates)
            if (o.type == MateType::Fix && o.a.body == refs[0].body) return out.fail(refs[0].body + " is already fixed (" + o.name + ")");
        m.a = refs[0];
        m.name = st.nextName();
        st.mates.push_back(m);
        doc.dirty = true;
        out.say("Fixed " + m.a.body + " where it is (" + m.name + ")");
        return out;
    }
    if (refs.size() != 2) return out.fail("A mate takes two faces: \"A\" AT x,y,z \"B\" AT x,y,z");
    if (refs[0].body == refs[1].body) return out.fail("Both faces are on " + refs[0].body + ": mate two parts");
    m.a = refs[0], m.b = refs[1];
    // What each kind can take.
    const bool dirs = m.a.geom != MateGeom::Point && m.b.geom != MateGeom::Point;
    if ((m.type == MateType::Parallel || m.type == MateType::Perpendicular || m.type == MateType::Angle) && !dirs)
        return out.fail(std::string("A ") + mateTypeName(m.type) + " mate takes planes or axes, not points");
    if (m.type == MateType::Concentric && m.a.geom != MateGeom::Axis && m.b.geom != MateGeom::Axis)
        return out.fail("A concentric mate takes round faces (cylinders, cones): the faces picked are " + std::string(mateGeomName(m.a.geom)) + " and " +
                        mateGeomName(m.b.geom));
    if (m.type == MateType::Concentric && (m.a.geom == MateGeom::Plane || m.b.geom == MateGeom::Plane))
        return out.fail("A concentric mate takes two round faces (or one and a sphere): one face picked is flat");
    m.name = st.nextName();
    // Keep the new mate only if it can be met with the others.
    const std::vector<Body> before = doc.scene.bodies();
    const bool wasMet = [&] {
        for (const Mate& o : st.mates)
            if (!o.suppressed && mateError(doc.scene, o) > 1e-3) return false;
        return true;
    }();
    st.mates.push_back(m);
    const MateSolveReport r = solveMates(doc.scene, st);
    if (!r.solved && wasMet) {
        st.mates.pop_back();
        doc.scene.bodies() = before;
        doc.scene.touchBodies();
        std::string others;
        for (const std::string& f : r.failing)
            if (f != m.name) others += (others.empty() ? " (" : ", ") + f;
        return out.fail("Can't add the " + std::string(mateTypeName(m.type)) + " mate: it conflicts with the mates already there" +
                        (others.empty() ? std::string() : others + ")") + ". Check the faces picked: a point picks the face nearest it");
    }
    doc.dirty = true;
    out.say("Added " + describeMate(m));
    reportSolve(out, r);
    return out;
}

} // namespace sicdm

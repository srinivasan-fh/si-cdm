// WELDMENT: structural members along sketch segments, their joints, the cut list.

#include "core/WeldmentCommand.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Units.hpp"
#include "core/Weldment.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <optional>

namespace sicdm {

namespace {

// The corners a member line runs through: a 3D polyline's own points (not its bends).
std::vector<Vec3> straightPath(const SketchCurve& c) {
    if (!c.spatial()) return c.worldStrip();
    bool closed = false;
    std::vector<Vec3> p = spacePoints(c, &closed);
    if (closed && !p.empty()) p.push_back(p.front());
    return p;
}

std::string lowerOf(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

std::string num(double v, int digits = 4) {
    char b[32];
    std::snprintf(b, sizeof b, "%.*g", digits, std::fabs(v) < 5e-10 ? 0.0 : v);
    return b;
}

bool isWord(const std::vector<std::string>& a, const std::vector<bool>& q, std::size_t i, const char* w) {
    return i < a.size() && !(i < q.size() && q[i]) && lowerOf(a[i]) == w;
}

const char* const kKeys[] = {"along", "from", "to", "angle", "align", "mirror", "up", "corner", "group", "material", "profile", "name"};
bool isKey(const std::vector<std::string>& a, const std::vector<bool>& q, std::size_t i) {
    if (i >= a.size() || (i < q.size() && q[i])) return false;
    const std::string w = lowerOf(a[i]);
    return std::any_of(std::begin(kKeys), std::end(kKeys), [&](const char* k) { return w == k; });
}

int findCurveNamed(const Document& d, const std::string& n) {
    const auto& c = d.scene.sketch().curves();
    for (std::size_t i = 0; i < c.size(); ++i)
        if (c[i].name == n) return static_cast<int>(i);
    for (std::size_t i = 0; i < c.size(); ++i)
        if (lowerOf(c[i].name) == lowerOf(n)) return static_cast<int>(i);
    return -1;
}

int findBodyNamed(const Document& d, const std::string& n) {
    const auto& b = d.scene.bodies();
    for (std::size_t i = 0; i < b.size(); ++i)
        if (b[i].name == n) return static_cast<int>(i);
    return -1;
}

std::string newMemberName(const Document& d) {
    for (int k = 1;; ++k)
        if (findBodyNamed(d, "Member " + std::to_string(k)) < 0) return "Member " + std::to_string(k);
}

double memberLength(const Body& b) { return length(b.member.end - b.member.start); }

// Descriptions, materials and part numbers from the cut list: members alike share a number
// (the weldment's name and the item), so the drawing's parts list counts them together. A
// number someone typed stays.
void labelMembers(Scene& scene) {
    std::map<std::string, std::vector<CutItem>> byGroup;
    for (const Body& b : scene.bodies())
        if (b.isMember()) byGroup[b.member.group];
    for (auto& [g, items] : byGroup) items = cutList(scene, g);
    for (Body& b : scene.bodies()) {
        if (!b.isMember()) continue;
        const auto& items = byGroup[b.member.group];
        for (std::size_t k = 0; k < items.size(); ++k) {
            if (std::find(items[k].members.begin(), items[k].members.end(), b.name) == items[k].members.end()) continue;
            b.properties["DESCRIPTION"] = items[k].profile + ", " + num(items[k].length, 6) + " mm";
            b.properties["MATERIAL"] = items[k].material;
            const std::string autoPrefix = b.member.group + "-";
            auto it = b.properties.find("NUMBER");
            const bool typed = it != b.properties.end() && !it->second.empty() &&
                               !(it->second.rfind(autoPrefix, 0) == 0 && it->second.find_first_not_of("0123456789", autoPrefix.size()) == std::string::npos);
            if (!typed) b.properties["NUMBER"] = autoPrefix + std::to_string(k + 1);
        }
    }
}

std::string cutLine(const CutItem& c, std::size_t k) {
    std::string angles;
    if (c.angle0 > 0.05 || c.angle1 > 0.05) angles = ", ends " + num(c.angle0, 3) + " and " + num(c.angle1, 3) + " deg off square";
    return "  " + std::to_string(k + 1) + ". " + c.profile + "  " + std::to_string(c.members.size()) + " x " + num(c.length, 6) + " mm" + angles + ", " +
           num(c.massEach, 4) + " kg each (" + c.material + ")";
}

} // namespace

void refreshWeldments(Scene& scene) {
    if (std::none_of(scene.bodies().begin(), scene.bodies().end(), [](const Body& b) { return b.isMember(); })) return;
    resolveJoints(scene, "");
    labelMembers(scene);
}

CommandOutput runWeldmentCommand(Document& doc, const std::vector<std::string>& a, const std::vector<bool>& quoted) {
    CommandOutput out;
    const char* usage = "WELDMENT PROFILES [family] | MEMBER profile ALONG curve... | MEMBER profile FROM x,y,z TO x,y,z [TO ...] [ANGLE deg] [ALIGN center|top|bottom|left|right] "
                        "[MIRROR] [UP x,y,z] [CORNER MITER|BUTT|NONE] [GROUP name] [MATERIAL m] | JOINT x,y,z MITER|BUTT|THROUGH|SQUARE [member] | "
                        "EDIT member [PROFILE p] [ANGLE deg] [ALIGN where] [MIRROR ON|OFF] [MATERIAL m] | LIST [group] | CUTLIST [group] [EXPORT file.csv]";
    if (a.empty()) return out.fail(usage);
    const std::string sub = lowerOf(a[0]);
    auto& bodies = doc.scene.bodies();

    if (sub == "profiles") {
        const std::vector<std::string> fams = a.size() > 1 ? std::vector<std::string>{a[1]} : weldProfileFamilies();
        for (const std::string& f : fams) {
            const auto sizes = weldProfileSizes(f);
            std::string line;
            for (const std::string& s : sizes) line += (line.empty() ? "" : "  ") + s;
            out.say(f + ": " + (sizes.empty() ? std::string("any size by its dimensions") : line));
        }
        out.say("Any size of SHS, RHS, CHS, FLAT, ROUND and ANGLE by its dimensions; I and C sections as h x b x web x flange [x root radius]");
        return out;
    }

    if (sub == "member") {
        if (a.size() < 2) return out.fail("WELDMENT MEMBER profile ALONG curve...  (WELDMENT PROFILES lists them)");
        // The profile: quoted, or a family word and its size.
        std::size_t i = 1;
        std::string profile = a[i++];
        if (!(1 < quoted.size() && quoted[1]) && i < a.size() && !isKey(a, quoted, i) && std::isdigit(static_cast<unsigned char>(a[i][0]))) profile += " " + a[i++];
        std::string why;
        const auto prof = weldProfile(profile, &why);
        if (!prof) return out.fail(why);
        Body::Member proto;
        proto.profile = prof->designation;
        std::vector<std::pair<std::vector<Vec3>, Vec3>> paths; // points, up
        std::optional<Vec3> up;
        std::string group;
        for (; i < a.size(); ++i) {
            if (isWord(a, quoted, i, "along")) {
                // Curves, named (quoted, or words joined until they name one).
                for (++i; i < a.size() && !isKey(a, quoted, i);) {
                    std::string n = a[i];
                    int ci = findCurveNamed(doc, n);
                    std::size_t j = i + 1;
                    while (ci < 0 && !(i < quoted.size() && quoted[i]) && j < a.size() && !isKey(a, quoted, j)) n += " " + a[j++], ci = findCurveNamed(doc, n);
                    if (ci < 0) return out.fail("No sketch curve named " + a[i]);
                    const SketchCurve& c = doc.scene.sketch().curves()[static_cast<std::size_t>(ci)];
                    if (c.kind != CurveKind::Line && c.kind != CurveKind::Polyline && c.kind != CurveKind::Rectangle)
                        return out.fail(c.name + " isn't straight: members follow lines, polylines and rectangles");
                    paths.push_back({straightPath(c), curveFrame(c).normal});
                    i = j;
                }
                --i;
            } else if (isWord(a, quoted, i, "from") && i + 1 < a.size()) {
                std::vector<Vec3> pts;
                const auto p0 = parseVec3(a[++i]);
                if (!p0) return out.fail("FROM needs a point x,y,z");
                pts.push_back(*p0);
                while (isWord(a, quoted, i + 1, "to") && i + 2 < a.size()) {
                    const auto p = parseVec3(a[i + 2]);
                    if (!p) return out.fail("TO needs a point x,y,z");
                    pts.push_back(*p);
                    i += 2;
                }
                if (pts.size() < 2) return out.fail("FROM x,y,z needs TO x,y,z");
                paths.push_back({pts, Vec3{0, 0, 1}});
            } else if (isWord(a, quoted, i, "angle") && i + 1 < a.size()) {
                proto.angle = static_cast<float>(std::atof(a[++i].c_str()));
            } else if (isWord(a, quoted, i, "align") && i + 1 < a.size()) {
                proto.align = lowerOf(a[++i]);
                const std::string& al = proto.align;
                const bool ok = al == "center" || al == "centre" || al == "top" || al == "bottom" || al == "left" || al == "right" || al == "top-left" ||
                                al == "top-right" || al == "bottom-left" || al == "bottom-right";
                if (!ok) return out.fail("ALIGN is center, top, bottom, left, right, top-left, top-right, bottom-left or bottom-right");
            } else if (isWord(a, quoted, i, "mirror")) {
                proto.mirror = true;
            } else if (isWord(a, quoted, i, "up") && i + 1 < a.size()) {
                up = parseVec3(a[++i]);
                if (!up || length(*up) < 1e-6f) return out.fail("UP needs a direction x,y,z");
            } else if (isWord(a, quoted, i, "corner") && i + 1 < a.size()) {
                const std::string c = lowerOf(a[++i]);
                proto.corner = c == "miter" || c == "mitre" ? 0 : c == "butt" ? 1 : c == "none" || c == "square" ? 2 : -1;
                if (proto.corner < 0) return out.fail("CORNER is MITER, BUTT or NONE");
            } else if (isWord(a, quoted, i, "group") && i + 1 < a.size()) {
                group = a[++i];
            } else if (isWord(a, quoted, i, "material") && i + 1 < a.size()) {
                proto.material = a[++i];
                proto.material[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(proto.material[0])));
            } else {
                return out.fail(usage);
            }
        }
        if (paths.empty()) { // the selected curve
            if (doc.selectedCurve < 0 || doc.selectedCurve >= static_cast<int>(doc.scene.sketch().curves().size()))
                return out.fail("WELDMENT MEMBER " + prof->designation + " ALONG curve (or FROM x,y,z TO x,y,z): which line?");
            const SketchCurve& c = doc.scene.sketch().curves()[static_cast<std::size_t>(doc.selectedCurve)];
            if (c.kind != CurveKind::Line && c.kind != CurveKind::Polyline && c.kind != CurveKind::Rectangle)
                return out.fail(c.name + " isn't straight: members follow lines, polylines and rectangles");
            paths.push_back({straightPath(c), curveFrame(c).normal});
        }
        if (group.empty()) { // the weldment the last member went in, else a new one
            for (auto it = bodies.rbegin(); it != bodies.rend() && group.empty(); ++it)
                if (it->isMember()) group = it->member.group;
            if (group.empty()) group = "Weldment 1";
        }
        proto.group = group;
        std::vector<std::string> made;
        for (const auto& [pts, normal] : paths) {
            for (std::size_t k = 0; k + 1 < pts.size(); ++k) {
                if (length(pts[k + 1] - pts[k]) < 1e-3f) continue;
                Body::Member m = proto;
                m.start = pts[k], m.end = pts[k + 1];
                m.up = up.value_or(normal);
                Body b = meshBody(newMemberName(doc), MeshData{}, Vec3{0.62f, 0.64f, 0.68f});
                b.material.metallic = 0.8f;
                b.material.roughness = 0.42f;
                b.assembly = group;
                b.member = m;
                MeshData mesh;
                std::vector<BSurface> surfaces;
                if (!buildMember(m, *prof, mesh, surfaces, &why)) return out.fail(why);
                b.mesh = std::move(mesh);
                made.push_back(b.name);
                doc.scene.add(std::move(b));
            }
        }
        if (made.empty()) return out.fail("No segment long enough for a member");
        std::vector<std::string> notes;
        resolveJoints(doc.scene, group, &notes);
        labelMembers(doc.scene);
        doc.selectedBody = static_cast<int>(bodies.size()) - 1;
        doc.selectedCurve = doc.selectedImage = -1;
        doc.scene.touchBodies();
        out.say("Added " + std::to_string(made.size()) + (made.size() == 1 ? " member" : " members") + " of " + prof->designation + " to " + group + ": " + made.front() +
                (made.size() > 1 ? " to " + made.back() : std::string()));
        for (const std::string& n : notes) out.say("  " + n);
        return out;
    }

    if (sub == "joint") {
        if (a.size() < 3) return out.fail("WELDMENT JOINT x,y,z MITER|BUTT|THROUGH|SQUARE [member]");
        const auto at = parseVec3(a[1]);
        if (!at) return out.fail("JOINT needs the point x,y,z where members meet");
        const std::string kind = lowerOf(a[2]);
        const int style = kind == "miter" || kind == "mitre" ? 0 : kind == "butt" ? 1 : kind == "through" ? 2 : kind == "square" || kind == "none" ? 3 : -1;
        if (style < 0) return out.fail("A joint is MITER, BUTT (the member named stops at the other), THROUGH (it runs past) or SQUARE");
        const std::string who = a.size() > 3 ? a[3] : std::string();
        // The member ends at that point.
        std::vector<std::pair<std::size_t, int>> ends;
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            if (!bodies[i].isMember()) continue;
            for (int e = 0; e < 2; ++e)
                if (length((e ? bodies[i].member.end : bodies[i].member.start) + bodies[i].offset - *at) < 0.5f) ends.push_back({i, e});
        }
        if (ends.empty()) return out.fail("No member ends at " + a[1]);
        if (!who.empty() && std::none_of(ends.begin(), ends.end(), [&](const auto& p) { return bodies[p.first].name == who; }))
            return out.fail(who + " doesn't end at " + a[1]);
        const std::string first = who.empty() ? bodies[ends[0].first].name : who;
        for (const auto& [i, e] : ends) {
            int s = style;
            if (style == 1 || style == 2) s = bodies[i].name == first ? style : (style == 1 ? 2 : 1);
            bodies[i].member.style[static_cast<std::size_t>(e)] = s;
        }
        std::vector<std::string> notes;
        // (Forced to be remade: the planes compare against what they were.)
        for (const auto& [i, e] : ends) bodies[i].member.cut[static_cast<std::size_t>(e)] = Vec4{};
        resolveJoints(doc.scene, "", &notes);
        labelMembers(doc.scene);
        out.say("Joint at " + a[1] + ": " + (style == 0 ? "mitred" : style == 3 ? "square" : first + (style == 1 ? " stops at the other" : " runs through")));
        for (const std::string& n : notes) out.say("  " + n);
        return out;
    }

    if (sub == "edit") {
        if (a.size() < 3) return out.fail("WELDMENT EDIT member [PROFILE p] [ANGLE deg] [ALIGN where] [MIRROR ON|OFF] [MATERIAL m]");
        const int bi = findBodyNamed(doc, a[1]);
        if (bi < 0 || !bodies[static_cast<std::size_t>(bi)].isMember()) return out.fail("No member named " + a[1]);
        Body& b = bodies[static_cast<std::size_t>(bi)];
        Body::Member m = b.member;
        for (std::size_t i = 2; i < a.size(); ++i) {
            if (isWord(a, quoted, i, "profile") && i + 1 < a.size()) {
                std::string p = a[++i];
                if (i + 1 < a.size() && !isKey(a, quoted, i + 1) && std::isdigit(static_cast<unsigned char>(a[i + 1][0]))) p += " " + a[++i];
                std::string why;
                const auto prof = weldProfile(p, &why);
                if (!prof) return out.fail(why);
                m.profile = prof->designation;
            } else if (isWord(a, quoted, i, "angle") && i + 1 < a.size()) {
                m.angle = static_cast<float>(std::atof(a[++i].c_str()));
            } else if (isWord(a, quoted, i, "align") && i + 1 < a.size()) {
                m.align = lowerOf(a[++i]);
            } else if (isWord(a, quoted, i, "mirror")) {
                m.mirror = !(isWord(a, quoted, i + 1, "off"));
                if (isWord(a, quoted, i + 1, "on") || isWord(a, quoted, i + 1, "off")) ++i;
            } else if (isWord(a, quoted, i, "material") && i + 1 < a.size()) {
                m.material = a[++i];
                m.material[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(m.material[0])));
            } else {
                return out.fail("WELDMENT EDIT member [PROFILE p] [ANGLE deg] [ALIGN where] [MIRROR ON|OFF] [MATERIAL m]");
            }
        }
        if (b.member.key && b.member.key != meshKey(b.mesh)) return out.fail(b.name + " was changed since it was made (cut, drilled...): it can't be remade");
        b.member = m;
        b.member.cut = {}; // found again
        b.member.key = 0;
        std::vector<std::string> notes;
        resolveJoints(doc.scene, "", &notes);
        labelMembers(doc.scene);
        out.say("Changed " + b.name + ": " + m.profile);
        for (const std::string& n : notes) out.say("  " + n);
        return out;
    }

    if (sub == "list") {
        const std::string group = a.size() > 1 ? a[1] : std::string();
        int n = 0;
        for (const Body& b : bodies) {
            if (!b.isMember() || (!group.empty() && b.member.group != group)) continue;
            ++n;
            out.say("  " + b.name + "  " + b.member.profile + "  " + num(memberLength(b), 6) + " mm on the line  (" + b.member.group + ")");
        }
        if (!n) out.say("No members (WELDMENT MEMBER SHS 40x40x3 ALONG \"Polyline 1\" adds some)");
        return out;
    }

    if (sub == "cutlist") {
        std::string group, file;
        for (std::size_t i = 1; i < a.size(); ++i) {
            if (isWord(a, quoted, i, "export") && i + 1 < a.size()) file = a[++i];
            else group = a[i];
        }
        const auto items = cutList(doc.scene, group);
        if (items.empty()) return out.fail(group.empty() ? "No members to cut" : "No members in " + group);
        double total = 0;
        std::map<std::string, double> metres;
        for (std::size_t k = 0; k < items.size(); ++k) {
            out.say(cutLine(items[k], k));
            total += items[k].massEach * static_cast<double>(items[k].members.size());
            metres[items[k].profile] += items[k].length * static_cast<double>(items[k].members.size()) / 1000.0;
        }
        for (const auto& [p, m] : metres) out.say("  " + p + ": " + num(m, 4) + " m in all");
        out.say("  Mass: " + num(total, 4) + " kg");
        if (!file.empty()) {
            std::ofstream f(file);
            if (!f) return out.fail("Cannot write " + file);
            f << "Item,Quantity,Profile,Length (mm),End angle 1 (deg),End angle 2 (deg),Material,Mass each (kg),Members\n";
            for (std::size_t k = 0; k < items.size(); ++k) {
                const CutItem& c = items[k];
                std::string members;
                for (const std::string& m : c.members) members += (members.empty() ? "" : "; ") + m;
                f << (k + 1) << "," << c.members.size() << ",\"" << c.profile << "\"," << num(c.length, 7) << "," << num(c.angle0, 4) << "," << num(c.angle1, 4) << ",\"" << c.material
                  << "\"," << num(c.massEach, 5) << ",\"" << members << "\"\n";
            }
            out.say("  Written to " + file);
        }
        return out;
    }
    return out.fail(usage);
}

} // namespace sicdm

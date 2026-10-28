// FACE: direct editing (DirectEdit.hpp).

#include "core/FaceCommand.hpp"

#include "core/Command.hpp"
#include "core/DirectEdit.hpp"
#include "core/Document.hpp"
#include "core/Features.hpp"
#include "core/Units.hpp"

#include <cctype>
#include <cmath>
#include <cstdio>

namespace sicdm {

namespace {

std::string lowerOf(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

std::string num(double v) {
    char b[32];
    std::snprintf(b, sizeof b, "%.4g", v);
    return b;
}

std::string vecText(const DVec3& v) { return "(" + num(v.x) + ", " + num(v.y) + ", " + num(v.z) + ")"; }

// The visible body whose surface is nearest the point.
int nearestBody(const Document& doc, const Vec3& p) {
    int best = -1;
    float bd = 1e30f;
    const auto& bodies = doc.scene.bodies();
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        const Body& b = bodies[i];
        if (!b.visible || b.mesh.indices.empty()) continue;
        const AABB box = b.bounds();
        const Vec3 c = vmax(box.min, vmin(p, box.max));
        if (length(c - p) > bd) continue; // its box is already farther than the best surface
        const Vec3 q = p - b.offset;
        for (std::size_t k = 0; k + 2 < b.mesh.indices.size(); k += 3) {
            const Vec3 x = closestPointOnTriangle(q, b.mesh.vertices[b.mesh.indices[k]].position, b.mesh.vertices[b.mesh.indices[k + 1]].position,
                                                  b.mesh.vertices[b.mesh.indices[k + 2]].position);
            const float d = length(x - q);
            if (d < bd) bd = d, best = static_cast<int>(i);
        }
    }
    return best;
}

} // namespace

CommandOutput runFaceCommand(Document& doc, const std::vector<std::string>& a, const std::vector<bool>& quoted) {
    CommandOutput out;
    const std::string usage = "FACE INFO|OFFSET|DELETE|MOVE [body] AT x,y,z [DISTANCE d] [BY dx,dy,dz]";
    if (a.empty()) return out.fail(usage);
    const std::string sub = lowerOf(a[0]);
    auto word = [&](std::size_t i, const char* w) { return i < a.size() && !(i < quoted.size() && quoted[i]) && lowerOf(a[i]) == w; };
    std::string bodyName;
    std::optional<Vec3> at;
    std::optional<double> distance;
    std::optional<Vec3> by;
    for (std::size_t i = 1; i < a.size(); ++i) {
        if (word(i, "at") && i + 1 < a.size()) {
            at = parseVec3(a[++i]);
            if (!at) return out.fail("AT needs a point x,y,z on the face");
        } else if ((word(i, "distance") || word(i, "by")) && i + 1 < a.size() && a[i + 1].find(',') == std::string::npos) {
            const auto d = parseLength(a[++i]);
            if (!d) return out.fail("DISTANCE needs a length");
            distance = *d;
        } else if (word(i, "by") && i + 1 < a.size()) {
            by = parseVec3(a[++i]);
            if (!by) return out.fail("BY needs dx,dy,dz");
        } else if (!(i < quoted.size() && quoted[i]) && a[i].find(',') != std::string::npos && !at) {
            at = parseVec3(a[i]);
            if (!at) return out.fail("A point is x,y,z");
        } else if (!(i < quoted.size() && quoted[i]) && !distance && sub == "offset" && parseLength(a[i])) {
            distance = *parseLength(a[i]);
        } else {
            bodyName += (bodyName.empty() ? "" : " ") + a[i];
        }
    }
    if (!at) return out.fail(usage + " (the point picks the face)");
    int bi = -1;
    if (!bodyName.empty()) {
        for (std::size_t i = 0; i < doc.scene.bodies().size() && bi < 0; ++i)
            if (lowerOf(doc.scene.bodies()[i].name) == lowerOf(bodyName)) bi = static_cast<int>(i);
        if (bi < 0) return out.fail("No body named " + bodyName);
    } else {
        bi = nearestBody(doc, *at);
        if (bi < 0) return out.fail("No body there");
    }
    Body& body = doc.scene.bodies()[static_cast<std::size_t>(bi)];
    std::string why, how;
    if (sub == "info") {
        DirectFaceInfo f;
        if (!directFaceAt(body, *at, f, &why)) return out.fail(why);
        std::string line = body.name + ": a " + directFaceKindName(f.kind) + " face, " + num(f.area) + " mm2" + (f.exact ? ", exact" : "");
        if (f.kind == DirectFaceKind::Plane) line += ", facing " + vecText(f.normal);
        else if (f.kind != DirectFaceKind::Freeform)
            line += std::string(f.hole ? ", a hole" : ", round outwards") + ", radius " + num(f.radius) + " mm about " + vecText(f.axis) + " through " + vecText(f.point);
        out.say(line);
        return out;
    }
    if (sub == "offset" || sub == "pushpull" || sub == "pull" || sub == "push") {
        if (!distance) return out.fail("FACE OFFSET [body] AT x,y,z DISTANCE d (negative: in)");
        const double d = sub == "push" ? -std::fabs(*distance) : *distance;
        if (!directOffsetFace(body, *at, d, &why, &how)) return out.fail(why);
    } else if (sub == "delete" || sub == "remove") {
        if (!directDeleteFace(body, *at, &why, &how)) return out.fail(why);
    } else if (sub == "move") {
        if (!by) return out.fail("FACE MOVE [body] AT x,y,z BY dx,dy,dz");
        if (!directMoveFace(body, *at, DVec3(*by), &why, &how)) return out.fail(why);
    } else {
        return out.fail(usage);
    }
    doc.scene.touchBodies(); // its shape changed in place: undo, the view and picking see it
    doc.dirty = true;
    doc.selectedBody = bi;
    out.say(body.name + ": " + how + (body.exact() ? "" : " (as a mesh: it has no exact geometry)"));
    return out;
}

} // namespace sicdm

// INSERTMESH and CURVE: shapes, tubes and rows of shapes laid on a body's surface.

#include "core/InsertCommand.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Features.hpp"
#include "core/InsertBrush.hpp"
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

std::string num(double v) {
    char b[32];
    std::snprintf(b, sizeof b, "%.4g", std::fabs(v) < 5e-6 ? 0.0 : v);
    return b;
}

std::string pointText(const Vec3& v) { return "(" + num(v.x) + ", " + num(v.y) + ", " + num(v.z) + ")"; }

int bodyIndex(const Document& d, const std::string& n) {
    const auto& b = d.scene.bodies();
    for (std::size_t i = 0; i < b.size(); ++i)
        if (b[i].name == n) return static_cast<int>(i);
    for (std::size_t i = 0; i < b.size(); ++i)
        if (lowerOf(b[i].name) == lowerOf(n)) return static_cast<int>(i);
    return -1;
}

std::string freeName(const Document& d, const std::string& base) {
    for (int k = 1;; ++k)
        if (bodyIndex(d, base + " " + std::to_string(k)) < 0) return base + " " + std::to_string(k);
}

bool keyword(const std::string& w) {
    static const char* const kKeys[] = {"sphere", "ball", "cube", "box", "cylinder", "peg", "cone", "spike", "torus", "ring", "body", "at", "size", "dir",
                                        "spin", "sink", "union", "append", "new", "tube", "insert", "path", "radius", "spacing"};
    const std::string k = lowerOf(w);
    for (const char* q : kKeys)
        if (k == q) return true;
    return false;
}

} // namespace

CommandOutput runInsertCommand(Document& doc, const std::string& cmd, const std::vector<std::string>& a, const std::vector<bool>& quoted) {
    CommandOutput out;
    auto unq = [&](std::size_t k) { return !(k < quoted.size() && quoted[k]); };
    const bool curve = cmd == "CURVE";
    const char* usage = curve ? "CURVE [body] TUBE PATH x,y,z x,y,z ... [RADIUS r] [SINK f] [UNION|APPEND|NEW]  |  CURVE [body] INSERT shape|BODY \"name\" PATH ... "
                                "[SIZE s] [SPACING d] [SINK f] [UNION|APPEND|NEW]"
                              : "INSERTMESH [body] SPHERE|CUBE|CYLINDER|CONE|TORUS|BODY \"name\" AT x,y,z [SIZE s] [DIR x,y,z] [SPIN deg] [SINK f] "
                                "[UNION|APPEND|NEW]";
    // The body: leading words up to the first keyword (default the selection).
    std::size_t i = 0;
    std::string name;
    if (!a.empty() && !unq(0)) name = a[i++];
    else
        while (i < a.size() && !keyword(a[i])) name += (name.empty() ? "" : " ") + a[i++];
    const int bi = name.empty() ? doc.selectedBody : bodyIndex(doc, name);
    if (bi < 0 || bi >= static_cast<int>(doc.scene.bodies().size())) return out.fail(name.empty() ? "Select a body first, or give its name" : "No body named " + name);

    bool tube = false;
    std::optional<InsertShape> shape;
    std::string custom;
    std::vector<Vec3> points;
    std::optional<float> size, radius, spacing, spin, sink;
    std::optional<Vec3> dir;
    InsertJoin join = InsertJoin::Union;
    auto length = [&](std::size_t k) -> std::optional<float> {
        if (k >= a.size()) return std::nullopt;
        const auto v = parseLength(a[k]);
        return v ? std::optional<float>(static_cast<float>(*v)) : std::nullopt;
    };
    for (; i < a.size(); ++i) {
        const std::string k = unq(i) ? lowerOf(a[i]) : std::string();
        if (const auto s = insertShapeFromName(k); s && !k.empty()) {
            shape = *s;
        } else if (k == "body") {
            if (i + 1 >= a.size()) return out.fail("BODY needs the body whose shape to put in");
            custom = a[++i], shape = InsertShape::Mesh;
        } else if (k == "tube" && curve) {
            tube = true;
        } else if (k == "insert" && curve) {
            tube = false;
        } else if (k == "at" || k == "path") {
            while (i + 1 < a.size() && unq(i + 1) && !keyword(a[i + 1])) {
                const auto p = parseVec3(a[i + 1]);
                if (!p) return out.fail(a[i + 1] + " isn't a point (x,y,z)");
                points.push_back(*p), ++i;
            }
        } else if (k == "size" || k == "radius" || k == "spacing") {
            const auto v = length(i + 1);
            if (!v || !(*v > 0)) return out.fail(toUpper(k) + " needs a size above 0");
            (k == "size" ? size : k == "radius" ? radius : spacing) = *v, ++i;
        } else if (k == "spin") {
            const auto v = i + 1 < a.size() ? parseNumber(a[i + 1]) : std::nullopt;
            if (!v) return out.fail("SPIN needs degrees");
            spin = *v, ++i;
        } else if (k == "sink") {
            const auto v = i + 1 < a.size() ? parseNumber(a[i + 1]) : std::nullopt;
            if (!v || *v < 0 || *v > 1) return out.fail("SINK is from 0 (resting on the surface) to 1 (sunk in)");
            sink = *v, ++i;
        } else if (k == "dir") {
            const auto v = i + 1 < a.size() ? parseVec3(a[i + 1]) : std::nullopt;
            if (!v || !(sicdm::length(*v) > 0)) return out.fail("DIR needs a direction x,y,z");
            dir = *v, ++i;
        } else if (k == "union") {
            join = InsertJoin::Union;
        } else if (k == "append") {
            join = InsertJoin::Append;
        } else if (k == "new") {
            join = InsertJoin::NewBody;
        } else {
            return out.fail(usage);
        }
    }
    Body& body = doc.scene.bodies()[static_cast<std::size_t>(bi)];
    if (body.mesh.indices.empty()) return out.fail(body.name + " has no surface to work on");
    if (points.empty()) return out.fail(curve ? "CURVE needs the stroke: PATH x,y,z x,y,z ..." : "INSERTMESH needs where: AT x,y,z");
    if (curve && points.size() < 2) return out.fail("A curve needs at least two points");
    if (!curve && points.size() != 1) return out.fail("INSERTMESH puts one shape at one point (CURVE INSERT lays a row)");
    if (!tube && !shape) return out.fail(std::string("Name the shape: SPHERE, CUBE, CYLINDER, CONE, TORUS or BODY \"name\".  ") + usage);
    MeshData customMesh;
    if (shape == InsertShape::Mesh) {
        const int ci = bodyIndex(doc, custom);
        if (ci < 0) return out.fail("No body named " + custom);
        if (ci == bi) return out.fail("A body can't be put into itself");
        customMesh = worldMesh(doc.scene.bodies()[static_cast<std::size_t>(ci)]);
    }
    const AABB box = body.bounds();
    const float bodySize = sicdm::length(box.max - box.min);
    const float s = size.value_or(std::max(0.1f * bodySize, 0.5f));

    // Each point onto the surface, with its outward normal (or against DIR).
    SculptMesh surface(body);
    SurfaceIndex index(surface);
    const float reach = std::max({s, radius.value_or(0.0f), 0.05f * bodySize});
    std::vector<Vec3> ps, ns;
    for (const Vec3& p : points) {
        Vec3 n;
        const Vec3 q = index.closestPoint(p, &n);
        if (sicdm::length(q - p) > reach) return out.fail(pointText(p) + " is " + num(sicdm::length(q - p)) + " mm away from " + body.name);
        ps.push_back(q);
        ns.push_back(dir ? normalize(*dir) * -1.0f : n);
    }
    MeshData piece;
    std::string what;
    if (!curve) {
        InsertPlace place{ps[0], ns[0], {1, 0, 0}};
        piece = insertShapeMesh(*shape, s, place, spin.value_or(0), sink.value_or(0.15f), shape == InsertShape::Mesh ? &customMesh : nullptr);
        what = std::string(shape == InsertShape::Mesh ? custom : insertShapeName(*shape)) + " " + num(s) + " mm at " + pointText(ps[0]);
    } else if (tube) {
        const float r = radius.value_or(std::max(0.02f * bodySize, 0.25f));
        piece = curveTube(evenStroke(ps, ns, r * 0.5f), r, sink.value_or(0.5f));
        what = "tube " + num(2 * r) + " mm across";
    } else {
        const float d = spacing.value_or(s);
        piece = curveCopies(evenStroke(ps, ns, std::min(d, s) * 0.25f), *shape, s, d, sink.value_or(0.15f), shape == InsertShape::Mesh ? &customMesh : nullptr);
        what = "row of " + std::string(shape == InsertShape::Mesh ? custom : insertShapeName(*shape)) + " every " + num(d) + " mm";
    }
    if (piece.indices.empty()) return out.fail("Nothing came out to put in");
    if (join == InsertJoin::NewBody) {
        const std::string n = freeName(doc, curve && tube ? "Tube" : "Insert");
        Body b = featureBody(n, std::move(piece), body.color);
        doc.scene.add(std::move(b));
        doc.selectedBody = static_cast<int>(doc.scene.bodies().size()) - 1;
        doc.dirty = true;
        out.say("Added " + n + ": " + what);
        return out;
    }
    std::string why;
    const std::size_t before = body.mesh.indices.size() / 3;
    if (!joinInsert(body, piece, join, &why)) return out.fail(why.empty() ? std::string("The shape couldn't be joined to ") + body.name : why);
    doc.scene.touchBodies();
    doc.dirty = true;
    const std::size_t open = openEdgeCount(body.mesh);
    out.say(body.name + ": " + what + (join == InsertJoin::Append ? " appended (a polygroup of its own: REMESH fuses it)" : " united") + ", " +
            std::to_string(body.mesh.indices.size() / 3) + " triangles (was " + std::to_string(before) + ")" +
            (join == InsertJoin::Append ? std::string() : open == 0 ? ", closed" : ", " + std::to_string(open) + " open edges"));
    return out;
}

} // namespace sicdm

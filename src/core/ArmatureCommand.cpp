// ARMATURE: balls joined by links, sketched like ZSpheres; skinned into one smooth mesh that
// stays bound to them; posed by turning balls.

#include "core/ArmatureCommand.hpp"

#include "core/Armature.hpp"
#include "core/Brep.hpp"
#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Units.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <optional>

namespace sicdm {

namespace {

std::string lowerOf(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

std::string num(double v, int digits = 4) {
    char b[48];
    std::snprintf(b, sizeof b, "%.*g", digits, std::fabs(v) < 5e-10 ? 0.0 : v);
    return b;
}

std::string vecText(const Vec3& v) { return "(" + num(v.x) + ", " + num(v.y) + ", " + num(v.z) + ")"; }

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

// Ball n as typed (1...), checked against the armature.
std::optional<int> ballNumber(const std::vector<std::string>& a, std::size_t i, const Armature& arm) {
    if (i >= a.size()) return std::nullopt;
    const auto v = parseNumber(a[i]);
    if (!v || *v < 1 || *v > static_cast<float>(arm.balls.size()) || std::fabs(*v - std::round(*v)) > 1e-6f) return std::nullopt;
    return static_cast<int>(std::lround(*v)) - 1;
}

// The plane a MIRROR names: through the root ball, square to X, Y or Z.
std::optional<int> mirrorAxis(const std::string& w) {
    const std::string u = toUpper(w);
    if (u == "X") return 0;
    if (u == "Y") return 1;
    if (u == "Z") return 2;
    return std::nullopt;
}

float& comp(Vec3& v, int axis) { return axis == 0 ? v.x : axis == 1 ? v.y : v.z; }
float comp(const Vec3& v, int axis) { return axis == 0 ? v.x : axis == 1 ? v.y : v.z; }

Vec3 mirrored(Vec3 p, int axis, const Vec3& root) {
    comp(p, axis) = 2 * comp(root, axis) - comp(p, axis);
    return p;
}

int rootOf(const Armature& arm) {
    for (std::size_t k = 0; k < arm.balls.size(); ++k)
        if (arm.balls[k].parent < 0) return static_cast<int>(k);
    return 0;
}

// The ball mirroring ball i across the plane (itself when it is on the plane); -1 when none.
int twinOf(const Armature& arm, int i, int axis) {
    const Vec3 root = arm.balls[static_cast<std::size_t>(rootOf(arm))].at;
    float size = 0;
    for (const ArmatureBall& b : arm.balls) size = std::max(size, b.radius);
    const float tol = std::max(1e-4f, size * 1e-3f);
    const Vec3 want = mirrored(arm.balls[static_cast<std::size_t>(i)].at, axis, root);
    for (std::size_t k = 0; k < arm.balls.size(); ++k)
        if (length(arm.balls[k].at - want) <= tol) return static_cast<int>(k);
    return -1;
}

// A turn mirrored across the plane square to `axis`.
std::array<float, 4> mirroredTurn(std::array<float, 4> q, int axis) {
    for (int k = 0; k < 3; ++k)
        if (k != axis) q[static_cast<std::size_t>(k)] = -q[static_cast<std::size_t>(k)];
    return q;
}

// The old skin's weights brought over to the points `now` (both where they stand in the
// armature's pose): each takes those of the old vertices about it, blended by how near they are
// (the nearest's alone where one sits on it). Weights the armature has no link for, or points
// with nothing old near, keep what they have (`s`'s, by distance).
void carryWeights(const SkinData& old, const Armature& arm, const std::vector<Vec3>& now, SkinData& s) {
    if (old.bone.size() != old.rest.size() || old.weight.size() != old.rest.size() || old.rest.empty()) return;
    const std::vector<Vec3> was = poseSkin(old, arm);
    const NearestPoints near(was);
    std::vector<int> usable(arm.balls.size(), -1); // links the armature has
    for (std::size_t b = 0; b < arm.balls.size(); ++b)
        if (arm.balls[b].parent >= 0) usable[b] = static_cast<int>(b);
    for (std::size_t i = 0; i < now.size() && i < s.bone.size(); ++i) {
        std::array<std::int32_t, 4> bone;
        std::array<float, 4> weight;
        if (weightsAbout(near, was, old.bone, old.weight, now[i], usable, bone, weight)) s.bone[i] = bone, s.weight[i] = weight;
    }
}

// A skin whose mesh was remade since (Dynamesh, retopology...) bound to the armature again as it
// stands: each vertex weighted as the old skin was about it (or, where it gave none, by the links
// nearest where it is now, the armature posed as it is), its rest pose taken back from that, and
// its unsculpted shape the nearest points of a fresh skin of the armature, so what was sculpted
// on it is kept on later edits.
std::shared_ptr<SkinData> rebind(const Body& b, const Armature& arm) {
    auto s = std::make_shared<SkinData>();
    s->armature = b.skin ? b.skin->armature : std::string();
    s->options = b.skin ? b.skin->options : SkinOptions{};
    std::vector<Vec3> now(b.mesh.vertices.size());
    for (std::size_t i = 0; i < now.size(); ++i) now[i] = b.mesh.vertices[i].position;
    Armature posed = arm;
    const std::vector<Vec3> at = posedBalls(arm);
    for (std::size_t i = 0; i < posed.balls.size(); ++i) posed.balls[i].at = at[i];
    skinWeights(posed, skinSoftness(arm, s->options), now, *s);
    if (b.skin) carryWeights(*b.skin, arm, now, *s);
    s->rest = unposeSkin(*s, arm, now);
    Armature rest = arm;
    for (ArmatureBall& ball : rest.balls) ball.turn = {0, 0, 0, 1};
    MeshData fresh;
    SkinData freshSkin;
    s->base = s->rest;
    if (armatureSkin(rest, s->options, fresh, freshSkin)) {
        const NearestPoints near(freshSkin.rest);
        for (std::size_t i = 0; i < s->rest.size(); ++i)
            if (const std::int64_t j = near.find(s->rest[i], 1e30f); j >= 0) s->base[i] = freshSkin.rest[static_cast<std::size_t>(j)];
    }
    s->key = meshKey(b.mesh);
    return s;
}

// Puts the armature back into its body, and bends the skins bound to it into its pose
// (edits made to a skin since are kept: its rest pose is taken back from it first).
void store(Document& doc, int index, const Armature& before, const Armature& arm, CommandOutput& out, bool poseSkins) {
    Body& body = doc.scene.bodies()[static_cast<std::size_t>(index)];
    body.params = armatureParams(arm);
    body.rebuild();
    if (poseSkins)
        for (Body& b : doc.scene.bodies()) {
            if (!b.skin || b.skin->armature != body.name) continue;
            std::shared_ptr<SkinData> s;
            if (b.mesh.vertices.size() != b.skin->rest.size()) { // remeshed since: bound again as it is now
                s = rebind(b, before);
                out.say(b.name + " was remeshed since it was skinned: bound to " + body.name + " again as it is");
            } else {
                s = std::make_shared<SkinData>(*b.skin);
            }
            if (meshKey(b.mesh) != s->key) { // sculpted, smoothed...: keep that
                std::vector<Vec3> now(b.mesh.vertices.size());
                for (std::size_t i = 0; i < now.size(); ++i) now[i] = b.mesh.vertices[i].position;
                s->rest = unposeSkin(*s, before, now);
            }
            const std::vector<Vec3> posed = poseSkin(*s, arm);
            for (std::size_t i = 0; i < posed.size(); ++i) b.mesh.vertices[i].position = posed[i];
            smoothVertexNormals(b.mesh);
            b.mesh.computeFeatureEdges(60.0f);
            b.brep.reset();
            s->key = meshKey(b.mesh);
            b.skin = std::move(s);
        }
    doc.scene.touchBodies();
    doc.dirty = true;
}

// The armature as it rests (no ball turned).
Armature atRest(Armature a) {
    for (ArmatureBall& b : a.balls) b.turn = {0, 0, 0, 1};
    return a;
}

// How an edit to an armature changed it, for the skins bound to it.
enum class Refit {
    Moved,    // balls moved or resized: the skin follows its links
    Inserted, // a ball put on a link: the link's vertices shared between its two halves
    Rebuilt   // balls added or taken away: skinned again, what was sculpted or painted kept
};

// A skin's data as its mesh stands now: bound again if it was remeshed since, and its rest pose
// taken back from the mesh if it was sculpted or smoothed since (`before`: the armature it is
// posed by).
std::shared_ptr<SkinData> currentSkin(const Body& b, const Armature& before, const std::string& armature, CommandOutput& out) {
    std::shared_ptr<SkinData> s;
    if (b.mesh.vertices.size() != b.skin->rest.size()) { // remeshed since: bound again as it is now
        s = rebind(b, before);
        out.say(b.name + " was remeshed since it was skinned: bound to " + armature + " again as it is");
    } else {
        s = std::make_shared<SkinData>(*b.skin);
    }
    if (meshKey(b.mesh) != s->key) {
        std::vector<Vec3> now(b.mesh.vertices.size());
        for (std::size_t i = 0; i < now.size(); ++i) now[i] = b.mesh.vertices[i].position;
        s->rest = unposeSkin(*s, before, now);
    }
    return s;
}

// Fits the skins bound to the armature to it after an edit, keeping what was sculpted on them.
// `ballMap`: each old ball's number now (-1: taken away), when they were renumbered.
void refitSkins(Document& doc, const std::string& armature, const Armature& before, const Armature& arm, CommandOutput& out, Refit how, int mid = -1,
                int child = -1, const std::vector<int>& ballMap = {}) {
    const Armature oldRest = atRest(before), newRest = atRest(arm);
    for (Body& b : doc.scene.bodies()) {
        if (!b.skin || b.skin->armature != armature) continue;
        std::shared_ptr<SkinData> s = currentSkin(b, before, armature, out);
        if (how == Refit::Moved) {
            if (s->base.size() == s->rest.size()) { // the unsculpted shape goes the same way
                SkinData bare = *s;
                bare.rest = s->base;
                s->base = refitSkinRest(bare, oldRest, newRest);
            }
            s->rest = refitSkinRest(*s, oldRest, newRest);
        } else if (how == Refit::Inserted && mid >= 0 && child >= 0) {
            const int parent = before.balls[static_cast<std::size_t>(child)].parent;
            const Vec3 a0 = oldRest.balls[static_cast<std::size_t>(parent)].at, b0 = oldRest.balls[static_cast<std::size_t>(child)].at;
            const Vec3 d = b0 - a0;
            for (std::size_t i = 0; i < s->rest.size(); ++i)
                for (std::int32_t& bone : s->bone[i])
                    if (bone == child && dot(s->rest[i] - a0, d) < 0.5f * dot(d, d)) bone = mid; // the near half
        } else if (how == Refit::Rebuilt) {
            // What was sculpted: how far each vertex is from where a fresh skin of the old
            // armature puts it (the same skin, vertex for vertex, unless it was remeshed).
            std::vector<Vec3> bare = s->base; // kept with the skin; else a fresh skin of the old armature
            if (bare.size() != s->rest.size()) {
                MeshData fresh;
                SkinData freshSkin;
                bare = armatureSkin(oldRest, s->options, fresh, freshSkin) ? freshSkin.rest : std::vector<Vec3>{};
            }
            SkinCarry carry;
            if (bare.size() == s->rest.size()) {
                carry.bare = std::move(bare);
                carry.detail.resize(s->rest.size());
                for (std::size_t i = 0; i < carry.detail.size(); ++i) carry.detail[i] = s->rest[i] - carry.bare[i];
                for (const Vertex& v : b.mesh.vertices) carry.colors.push_back(v.color);
                carry.indices = b.mesh.indices;
                if (b.mesh.hasGroups()) carry.groups = b.mesh.groups;
                if (b.mesh.hasUvs()) carry.uvs = b.mesh.uvs;
                carry.bone = s->bone, carry.weight = s->weight;
                carry.ballMap = ballMap;
                if (carry.ballMap.empty())
                    for (std::size_t k = 0; k < before.balls.size(); ++k) carry.ballMap.push_back(static_cast<int>(k));
            }
            const bool detail = !carry.detail.empty();
            MeshData mesh;
            auto ns = std::make_shared<SkinData>();
            std::string why;
            if (!reskinWithDetail(newRest, s->options, carry, mesh, *ns, &why)) {
                out.say(b.name + " couldn't be skinned again: " + why);
                continue;
            }
            ns->armature = armature;
            b.mesh = std::move(mesh);
            b.multires.reset();
            s = std::move(ns);
            if (!detail) out.say(b.name + " was skinned again (what was sculpted on it couldn't be carried over)");
        }
        const std::vector<Vec3> posed = poseSkin(*s, arm);
        for (std::size_t i = 0; i < posed.size(); ++i) b.mesh.vertices[i].position = posed[i];
        smoothVertexNormals(b.mesh);
        b.mesh.computeFeatureEdges(60.0f);
        b.brep.reset();
        s->key = meshKey(b.mesh);
        b.skin = std::move(s);
        out.say(b.name + (how == Refit::Rebuilt ? " skinned again to fit" : " fitted to it"));
    }
    doc.scene.touchBodies();
}

std::string ballLine(const Armature& arm, std::size_t i, const std::vector<Vec3>& posed) {
    const ArmatureBall& b = arm.balls[i];
    std::string l = std::to_string(i + 1) + ". at " + vecText(b.at) + " radius " + num(b.radius);
    l += b.parent < 0 ? std::string(", the root") : ", on " + std::to_string(b.parent + 1);
    const bool turned = std::fabs(b.turn[3]) < 1 - 1e-6f;
    if (turned) l += ", turned " + vecText(quatToEuler(b.turn)) + " deg";
    if (length(posed[i] - b.at) > 1e-4f) l += ", posed at " + vecText(posed[i]);
    return l;
}

} // namespace

CommandOutput runArmatureCommand(Document& doc, const std::vector<std::string>& a, const std::vector<bool>& quoted) {
    CommandOutput out;
    auto unq = [&](std::size_t k) { return !(k < quoted.size() && quoted[k]); };
    auto& bodies = doc.scene.bodies();
    if (a.empty() || (a.size() == 1 && unq(0) && lowerOf(a[0]) == "list")) {
        bool any = false;
        for (const Body& b : bodies) {
            if (b.kind != "armature") continue;
            Armature arm;
            armatureFromParams(b.params, arm);
            std::string skins;
            for (const Body& s : bodies)
                if (s.skin && s.skin->armature == b.name) skins += (skins.empty() ? "" : ", ") + s.name;
            out.say(b.name + ": " + std::to_string(arm.balls.size()) + (arm.balls.size() == 1 ? " ball" : " balls") +
                    (skins.empty() ? std::string() : "; skins: " + skins));
            any = true;
        }
        if (!any) out.say("No armatures (ARMATURE NEW makes one, then ARMATURE name ADD ... AT x,y,z)");
        return out;
    }
    std::string why;
    // NEW
    if (unq(0) && lowerOf(a[0]) == "new") {
        std::string name;
        Vec3 at{0, 0, 0};
        float radius = 10;
        for (std::size_t i = 1; i < a.size(); ++i) {
            const std::string k = unq(i) ? lowerOf(a[i]) : std::string();
            if (k == "at") {
                const auto v = i + 1 < a.size() ? parseVec3(a[i + 1]) : std::nullopt;
                if (!v) return out.fail("AT needs x,y,z");
                at = *v, ++i;
            } else if (k == "radius") {
                const auto r = i + 1 < a.size() ? parseLength(a[i + 1]) : std::nullopt;
                if (!r || !(*r > 0)) return out.fail("RADIUS needs a size above 0");
                radius = static_cast<float>(*r), ++i;
            } else if (name.empty()) {
                name = a[i];
            } else {
                return out.fail("ARMATURE NEW doesn't take " + a[i] + " (a name, AT, RADIUS)");
            }
        }
        if (name.empty()) name = freeName(doc, "Armature");
        else if (bodyIndex(doc, name) >= 0) return out.fail("There is already a body named " + name);
        Armature arm;
        arm.balls.push_back({at, radius, -1, {0, 0, 0, 1}});
        Body b = makeBody(name, "armature", armatureParams(arm), {}, Vec3{0.93f, 0.62f, 0.32f});
        doc.scene.add(std::move(b));
        doc.selectedBody = static_cast<int>(bodies.size()) - 1;
        doc.selectedCurve = doc.selectedImage = -1;
        doc.dirty = true;
        out.say("Added the armature " + name + ": its root ball at " + vecText(at) + ", radius " + num(radius) + ". Add balls: ARMATURE \"" + name +
                "\" ADD AT x,y,z");
        return out;
    }
    const int index = bodyIndex(doc, a[0]);
    if (index < 0 || bodies[static_cast<std::size_t>(index)].kind != "armature") return out.fail("No armature named " + a[0]);
    const std::string name = bodies[static_cast<std::size_t>(index)].name;
    Armature arm;
    if (!armatureFromParams(bodies[static_cast<std::size_t>(index)].params, arm)) return out.fail(name + " isn't a sound armature");
    const Armature before = arm;
    const std::string sub = a.size() > 1 && unq(1) ? lowerOf(a[1]) : std::string("list");
    const int root = rootOf(arm);
    // MIRROR x / y / z, wherever it is.
    std::optional<int> mirror;
    for (std::size_t i = 2; i + 1 < a.size(); ++i)
        if (unq(i) && lowerOf(a[i]) == "mirror") {
            mirror = mirrorAxis(a[i + 1]);
            if (!mirror) return out.fail("MIRROR needs X, Y or Z (the plane through the root ball square to it)");
        }
    auto skipMirror = [&](std::size_t& i) {
        if (unq(i) && lowerOf(a[i]) == "mirror") {
            ++i;
            return true;
        }
        return false;
    };

    if (sub == "show" || sub == "hide") {
        bodies[static_cast<std::size_t>(index)].visible = sub == "show";
        doc.scene.touchBodies();
        out.say(name + (sub == "show" ? " shown" : " hidden"));
        return out;
    }
    if (sub == "list") {
        const std::vector<Vec3> posed = posedBalls(arm);
        out.say(name + ":");
        for (std::size_t i = 0; i < arm.balls.size(); ++i) out.say("  " + ballLine(arm, i, posed));
        return out;
    }
    if (sub == "add") {
        int to = static_cast<int>(arm.balls.size()) - 1;
        std::optional<Vec3> at;
        std::optional<float> radius;
        for (std::size_t i = 2; i < a.size(); ++i) {
            const std::string k = unq(i) ? lowerOf(a[i]) : std::string();
            if (skipMirror(i)) continue;
            if (k == "to" || k == "on") {
                const auto n = ballNumber(a, i + 1, arm);
                if (!n) return out.fail("TO needs a ball's number (1 to " + std::to_string(arm.balls.size()) + ")");
                to = *n, ++i;
            } else if (k == "at") {
                const auto v = i + 1 < a.size() ? parseVec3(a[i + 1]) : std::nullopt;
                if (!v) return out.fail("AT needs x,y,z");
                at = *v, ++i;
            } else if (k == "radius") {
                const auto r = i + 1 < a.size() ? parseLength(a[i + 1]) : std::nullopt;
                if (!r || !(*r > 0)) return out.fail("RADIUS needs a size above 0");
                radius = static_cast<float>(*r), ++i;
            } else {
                return out.fail("ARMATURE ADD doesn't take " + a[i] + " (TO n, AT x,y,z, RADIUS r, MIRROR X|Y|Z)");
            }
        }
        if (!at) return out.fail("ARMATURE ADD needs where the ball goes: AT x,y,z");
        const ArmatureBall& parent = arm.balls[static_cast<std::size_t>(to)];
        const float r = radius.value_or(parent.radius * 0.8f);
        if (length(*at - parent.at) < 1e-6f) return out.fail("The new ball sits on ball " + std::to_string(to + 1) + ": put it somewhere else");
        arm.balls.push_back({*at, r, to, {0, 0, 0, 1}});
        std::string said = "Added ball " + std::to_string(arm.balls.size()) + " on " + std::to_string(to + 1) + " at " + vecText(*at) + ", radius " + num(r);
        if (mirror) {
            const Vec3 twinAt = mirrored(*at, *mirror, arm.balls[static_cast<std::size_t>(root)].at);
            if (length(twinAt - *at) > 1e-4f) {
                const int twinParent = twinOf(arm, to, *mirror);
                arm.balls.push_back({twinAt, r, twinParent >= 0 ? twinParent : to, {0, 0, 0, 1}});
                said += ", and its mirror " + std::to_string(arm.balls.size()) + " at " + vecText(twinAt);
            }
        }
        store(doc, index, before, arm, out, false);
        out.say(said);
        refitSkins(doc, name, before, arm, out, Refit::Rebuilt);
        return out;
    }
    if (sub == "insert") {
        const auto n = ballNumber(a, 2, arm);
        if (!n || arm.balls[static_cast<std::size_t>(*n)].parent < 0) return out.fail("ARMATURE INSERT needs a ball's number (not the root's): the ball goes halfway to it");
        ArmatureBall& child = arm.balls[static_cast<std::size_t>(*n)];
        const ArmatureBall parent = arm.balls[static_cast<std::size_t>(child.parent)];
        ArmatureBall mid{(parent.at + child.at) * 0.5f, 0.5f * (parent.radius + child.radius), child.parent, {0, 0, 0, 1}};
        arm.balls.push_back(mid);
        child.parent = static_cast<int>(arm.balls.size()) - 1;
        store(doc, index, before, arm, out, false);
        out.say("Inserted ball " + std::to_string(arm.balls.size()) + " at " + vecText(mid.at) + ", between " + std::to_string(mid.parent + 1) + " and " +
                std::to_string(*n + 1));
        refitSkins(doc, name, before, arm, out, Refit::Inserted, static_cast<int>(arm.balls.size()) - 1, *n);
        return out;
    }
    if (sub == "move" || sub == "radius") {
        const auto n = ballNumber(a, 2, arm);
        if (!n) return out.fail("ARMATURE " + toUpper(sub) + " needs a ball's number (1 to " + std::to_string(arm.balls.size()) + ")");
        std::vector<int> which{*n};
        if (mirror) {
            const int t = twinOf(arm, *n, *mirror);
            if (t >= 0 && t != *n) which.push_back(t);
        }
        if (sub == "radius") {
            const auto r = a.size() > 3 ? parseLength(a[3]) : std::nullopt;
            if (!r || !(*r > 0)) return out.fail("ARMATURE RADIUS n needs a size above 0");
            for (int w : which) arm.balls[static_cast<std::size_t>(w)].radius = static_cast<float>(*r);
            store(doc, index, before, arm, out, false);
            out.say("Ball " + std::to_string(*n + 1) + (which.size() > 1 ? " and its mirror" : "") + ": radius " + num(*r));
            refitSkins(doc, name, before, arm, out, Refit::Moved);
            return out;
        }
        std::optional<Vec3> to, by;
        bool chain = false;
        for (std::size_t i = 3; i < a.size(); ++i) {
            const std::string k = unq(i) ? lowerOf(a[i]) : std::string();
            if (skipMirror(i)) continue;
            if (k == "to" || k == "by") {
                const auto v = i + 1 < a.size() ? parseVec3(a[i + 1]) : std::nullopt;
                if (!v) return out.fail(toUpper(k) + " needs x,y,z");
                (k == "to" ? to : by) = *v, ++i;
            } else if (k == "chain") {
                chain = true;
            } else {
                return out.fail("ARMATURE MOVE doesn't take " + a[i] + " (TO x,y,z or BY dx,dy,dz, CHAIN, MIRROR X|Y|Z)");
            }
        }
        if (!to && !by) return out.fail("ARMATURE MOVE n needs TO x,y,z or BY dx,dy,dz");
        const Vec3 d = to ? *to - arm.balls[static_cast<std::size_t>(*n)].at : *by;
        for (std::size_t w = 0; w < which.size(); ++w) {
            Vec3 dw = d;
            if (w > 0) comp(dw, *mirror) = -comp(dw, *mirror);
            // CHAIN: everything hanging from it too.
            for (std::size_t k = 0; k < arm.balls.size(); ++k) {
                int j = static_cast<int>(k);
                bool below = j == which[w];
                if (chain)
                    while (!below && j >= 0) j = arm.balls[static_cast<std::size_t>(j)].parent, below = j == which[w];
                if (below) arm.balls[k].at += dw;
            }
        }
        store(doc, index, before, arm, out, false);
        out.say("Moved ball " + std::to_string(*n + 1) + (chain ? " and those on it" : "") + (which.size() > 1 ? ", and its mirror," : "") + " to " +
                vecText(arm.balls[static_cast<std::size_t>(*n)].at));
        refitSkins(doc, name, before, arm, out, Refit::Moved);
        return out;
    }
    if (sub == "remove" || sub == "delete") {
        const auto n = ballNumber(a, 2, arm);
        if (!n || arm.balls[static_cast<std::size_t>(*n)].parent < 0) return out.fail("ARMATURE REMOVE needs a ball's number (not the root's)");
        std::vector<char> gone(arm.balls.size(), 0);
        for (std::size_t k = 0; k < arm.balls.size(); ++k) {
            int j = static_cast<int>(k);
            while (j >= 0 && j != *n) j = arm.balls[static_cast<std::size_t>(j)].parent;
            gone[k] = j == *n;
        }
        std::vector<int> renumber(arm.balls.size(), -1);
        Armature kept;
        for (std::size_t k = 0; k < arm.balls.size(); ++k)
            if (!gone[k]) renumber[k] = static_cast<int>(kept.balls.size()), kept.balls.push_back(arm.balls[k]);
        for (ArmatureBall& b : kept.balls)
            if (b.parent >= 0) b.parent = renumber[static_cast<std::size_t>(b.parent)];
        const std::size_t removed = arm.balls.size() - kept.balls.size();
        store(doc, index, before, kept, out, false);
        out.say("Removed " + std::to_string(removed) + (removed == 1 ? " ball" : " balls") + " from " + name);
        refitSkins(doc, name, before, kept, out, Refit::Rebuilt, -1, -1, renumber);
        return out;
    }
    if (sub == "pose") {
        if (a.size() > 2 && unq(2) && lowerOf(a[2]) == "reset") {
            for (ArmatureBall& b : arm.balls) b.turn = {0, 0, 0, 1};
            store(doc, index, before, arm, out, true);
            out.say(name + " is back in its rest pose");
            return out;
        }
        const auto n = ballNumber(a, 2, arm);
        if (!n) return out.fail("ARMATURE POSE needs a ball's number (1 to " + std::to_string(arm.balls.size()) + "), or RESET");
        std::optional<std::array<float, 4>> set, by;
        std::optional<Vec3> axis;
        for (std::size_t i = 3; i < a.size(); ++i) {
            const std::string k = unq(i) ? lowerOf(a[i]) : std::string();
            if (skipMirror(i)) continue;
            if (k == "turn") {
                const auto v = i + 1 < a.size() ? parseVec3(a[i + 1]) : std::nullopt;
                if (!v) return out.fail("TURN needs degrees about X, Y and Z: TURN 0,0,45");
                set = quatFromEuler(*v), ++i;
            } else if (k == "axis") {
                const auto v = i + 1 < a.size() ? parseVec3(a[i + 1]) : std::nullopt;
                if (!v || !(length(*v) > 0)) return out.fail("AXIS needs a direction: AXIS 0,0,1");
                axis = *v, ++i;
            } else if (k == "angle") {
                const auto v = i + 1 < a.size() ? parseNumber(a[i + 1]) : std::nullopt;
                if (!v) return out.fail("ANGLE needs degrees");
                by = quatFromAxisAngle(axis.value_or(Vec3{0, 0, 1}), radians(*v)), ++i;
            } else if (k == "reset") {
                set = std::array<float, 4>{0, 0, 0, 1};
            } else {
                return out.fail("ARMATURE POSE doesn't take " + a[i] + " (TURN ax,ay,az, AXIS x,y,z ANGLE a, RESET, MIRROR X|Y|Z)");
            }
        }
        if (!set && !by) return out.fail("ARMATURE POSE n needs TURN ax,ay,az, AXIS x,y,z ANGLE a, or RESET");
        std::vector<int> which{*n};
        if (mirror) {
            const int t = twinOf(arm, *n, *mirror);
            if (t >= 0 && t != *n) which.push_back(t);
        }
        for (std::size_t w = 0; w < which.size(); ++w) {
            ArmatureBall& b = arm.balls[static_cast<std::size_t>(which[w])];
            std::array<float, 4> q = set ? *set : quatMultiply(*by, b.turn); // AXIS ANGLE turns on from where it is
            if (w > 0) q = mirroredTurn(q, *mirror);
            b.turn = q;
        }
        store(doc, index, before, arm, out, true);
        out.say("Ball " + std::to_string(*n + 1) + (which.size() > 1 ? " and its mirror" : "") + " turned " +
                vecText(quatToEuler(arm.balls[static_cast<std::size_t>(*n)].turn)) + " deg");
        return out;
    }
    if (sub == "detail") {
        // DETAIL [FROM n] TO m [MIRROR X|Y|Z]: what was sculpted round n copied onto m's skin.
        std::optional<int> from, to;
        for (std::size_t i = 2; i < a.size(); ++i) {
            const std::string k = unq(i) ? lowerOf(a[i]) : std::string();
            if (skipMirror(i)) continue;
            if (k == "from" || k == "to") {
                const auto n = ballNumber(a, i + 1, arm);
                if (!n) return out.fail(toUpper(k) + " needs a ball's number (1 to " + std::to_string(arm.balls.size()) + ")");
                (k == "from" ? from : to) = *n, ++i;
            } else {
                return out.fail("ARMATURE DETAIL doesn't take " + a[i] + " (FROM n, TO m, MIRROR X|Y|Z)");
            }
        }
        if (!to) return out.fail("ARMATURE DETAIL needs the ball the detail goes to: TO m");
        if (!from && mirror) from = twinOf(arm, *to, *mirror);
        if (!from || *from < 0) return out.fail(mirror ? "Ball " + std::to_string(*to + 1) + " has no mirror twin: name the ball the detail comes from (FROM n)"
                                                       : "ARMATURE DETAIL needs the ball the detail comes from: FROM n (or MIRROR X|Y|Z)");
        const Armature rest = atRest(arm);
        bool any = false;
        for (Body& b : bodies) {
            if (!b.skin || b.skin->armature != name) continue;
            std::shared_ptr<SkinData> sk = currentSkin(b, arm, name, out);
            std::vector<Vec3> moved;
            if (!copySkinDetail(*sk, rest, *from, *to, mirror.value_or(-1), moved, &why)) {
                out.say(b.name + ": " + why);
                continue;
            }
            sk->rest = std::move(moved);
            const std::vector<Vec3> posed = poseSkin(*sk, arm);
            for (std::size_t i = 0; i < posed.size(); ++i) b.mesh.vertices[i].position = posed[i];
            smoothVertexNormals(b.mesh);
            b.mesh.computeFeatureEdges(60.0f);
            b.brep.reset();
            sk->key = meshKey(b.mesh);
            b.skin = std::move(sk);
            out.say(b.name + ": the detail round ball " + std::to_string(*from + 1) + (mirror ? ", mirrored," : "") + " copied onto ball " +
                    std::to_string(*to + 1) + "'s skin");
            any = true;
        }
        if (!any) return out.fail("No skin of " + name + " took the detail" + (why.empty() ? std::string(" (ARMATURE \"" + name + "\" SKIN makes one)") : ""));
        doc.scene.touchBodies();
        doc.dirty = true;
        return out;
    }
    if (sub == "skin") {
        SkinOptions o;
        std::string skinName;
        for (std::size_t i = 2; i < a.size(); ++i) {
            const std::string k = unq(i) ? lowerOf(a[i]) : std::string();
            if (k == "name") {
                if (i + 1 >= a.size()) return out.fail("NAME needs the skin's name");
                skinName = a[++i];
            } else if (k == "voxel" || k == "blend") {
                const auto v = i + 1 < a.size() ? parseLength(a[i + 1]) : std::nullopt;
                if (!v || *v < 0) return out.fail(toUpper(k) + " needs a size");
                (k == "voxel" ? o.voxel : o.blend) = static_cast<float>(*v), ++i;
            } else if (k == "smooth") {
                const auto v = i + 1 < a.size() ? parseNumber(a[i + 1]) : std::nullopt;
                if (!v || *v < 0 || *v > 100) return out.fail("SMOOTH needs passes, 0 to 100");
                o.smooth = static_cast<int>(std::lround(*v)), ++i;
            } else {
                return out.fail("ARMATURE SKIN doesn't take " + a[i] + " (NAME, VOXEL, BLEND, SMOOTH)");
            }
        }
        if (skinName.empty()) skinName = bodyIndex(doc, name + " skin") < 0 ? name + " skin" : freeName(doc, name + " skin");
        else if (bodyIndex(doc, skinName) >= 0) return out.fail("There is already a body named " + skinName);
        // Skinned in the rest pose, then bent into the armature's pose.
        Armature rest = arm;
        for (ArmatureBall& b : rest.balls) b.turn = {0, 0, 0, 1};
        MeshData mesh;
        auto skin = std::make_shared<SkinData>();
        if (!armatureSkin(rest, o, mesh, *skin, &why)) return out.fail(why);
        skin->armature = name;
        skin->options = o;
        const std::vector<Vec3> posed = poseSkin(*skin, arm);
        for (std::size_t i = 0; i < posed.size(); ++i) mesh.vertices[i].position = posed[i];
        smoothVertexNormals(mesh);
        mesh.computeFeatureEdges(60.0f);
        skin->key = meshKey(mesh);
        Body b = meshBody(skinName, std::move(mesh), Vec3{0.80f, 0.66f, 0.55f});
        b.offset = bodies[static_cast<std::size_t>(index)].offset;
        b.skin = std::move(skin);
        const std::size_t tris = b.mesh.indices.size() / 3;
        doc.scene.add(std::move(b));
        // The armature steps aside: the skin is what is seen.
        bodies[static_cast<std::size_t>(index)].visible = false;
        doc.selectedBody = static_cast<int>(bodies.size()) - 1;
        doc.selectedCurve = doc.selectedImage = -1;
        doc.dirty = true;
        out.say("Skinned " + name + " into " + skinName + ": " + std::to_string(tris) + " triangles, bound to its " + std::to_string(arm.balls.size() - 1) +
                (arm.balls.size() == 2 ? " link" : " links") + " (POSE bends it; the armature is hidden: ARMATURE \"" + name + "\" SHOW to edit it)");
        return out;
    }
    return out.fail("ARMATURE " + name + " doesn't know " + a[1] + " (ADD, INSERT, MOVE, RADIUS, REMOVE, LIST, SKIN, DETAIL, POSE, SHOW, HIDE)");
}

} // namespace sicdm

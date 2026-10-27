#include "core/Scene.hpp"

#include <unordered_map>

#include "core/Brep.hpp"
#include "core/Step.hpp"

namespace sicdm {

Body meshBody(std::string name, MeshData mesh, Vec3 color) {
    Body b;
    b.name = std::move(name);
    b.mesh = std::move(mesh);
    b.color = color;
    return b;
}

Body makeBody(std::string name, std::string kind, std::vector<float> params, Vec3 offset, Vec3 color) {
    Body b;
    b.name = std::move(name);
    b.mesh = buildPrimitive(kind, params);
    b.kind = std::move(kind);
    b.params = std::move(params);
    b.offset = offset;
    b.color = color;
    attachPrimitiveBrep(b);
    return b;
}

bool Body::exact() const { return brep && !brep->empty() && brepKey == meshKey(mesh); }

Mat4 eulerRotation(const Vec3& d) noexcept {
    return rotateAxis({0, 0, 1}, radians(d.z)) * rotateAxis({0, 1, 0}, radians(d.y)) * rotateAxis({1, 0, 0}, radians(d.x));
}

void Body::rebuild() {
    if (kind == "mesh") return;
    mesh = buildPrimitive(kind, params);
    if (rotation.x != 0.0f || rotation.y != 0.0f || rotation.z != 0.0f) mesh.transform(eulerRotation(rotation));
    if (scale != 1.0f) mesh.transform(sicdm::scale(Vec3{scale, scale, scale}));
    attachPrimitiveBrep(*this);
}

void Body::setRotation(const Vec3& degrees) {
    if (kind == "mesh") { // undo the old orientation, apply the new one
        const Mat4 turn = eulerRotation(degrees) * inverse(eulerRotation(rotation));
        const bool wasExact = exact();
        mesh.transform(turn);
        if (wasExact) { // the exact geometry turns with it (in double: back from the old angles, on to the new)
            auto b = std::make_shared<Brep>(*brep);
            b->transform(eulerPlacement(decimal(rotation)).inverse().then(eulerPlacement(decimal(degrees))));
            brep = std::move(b);
            brepKey = meshKey(mesh);
        }
        rotation = degrees;
        return;
    }
    rotation = degrees;
    rebuild();
}

void Body::setScale(float s) {
    if (!(s > 0.0f) || s == scale) return;
    if (kind == "mesh") {
        const float k = s / scale;
        const bool wasExact = exact();
        mesh.transform(sicdm::scale(Vec3{k, k, k}));
        if (wasExact) {
            auto b = std::make_shared<Brep>(*brep);
            Placement t;
            t.s = decimal(s) / decimal(scale);
            b->transform(t);
            brep = std::move(b);
            brepKey = meshKey(mesh);
        }
        scale = s;
    } else {
        scale = s;
        rebuild();
    }
}

std::array<Vec3, 4> ImageRef::corners() const noexcept {
    const PlaneFrame f = planeFrame(plane);
    const float hw = width * 0.5f, hh = height() * 0.5f;
    return {f.toWorld({center.x - hw, center.y - hh}), f.toWorld({center.x + hw, center.y - hh}),
            f.toWorld({center.x + hw, center.y + hh}), f.toWorld({center.x - hw, center.y + hh})};
}

void Scene::removeImage(std::size_t index) {
    if (index >= m_images.size()) return;
    m_images.erase(m_images.begin() + static_cast<std::ptrdiff_t>(index));
    m_imageRevision = nextRevision();
}

namespace {

// The motions as far as an undo cares: their names and what they move.
std::string motionsKey(const std::vector<Motion>& motions) {
    std::string k;
    for (const Motion& m : motions) {
        k += m.name;
        k += '\x1f';
        for (const std::string& b : m.bodies) {
            k += b;
            k += '\x1e';
        }
        k += '\x1d';
    }
    return k;
}

// What the motions do (programs, speeds, ranges), beyond what motionsKey() names: an edit
// that only rekeys them (ARM, POSE) is still an edit of the motions to undo.
bool sameMotionPrograms(const std::vector<Motion>& a, const std::vector<Motion>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        const Motion &x = a[i], &y = b[i];
        if (x.speed != y.speed || x.range != y.range || x.mode != y.mode || x.start != y.start || x.enabled != y.enabled || x.keySpline != y.keySpline ||
            x.keys.size() != y.keys.size())
            return false;
        for (std::size_t k = 0; k < x.keys.size(); ++k)
            if (x.keys[k].t != y.keys[k].t || x.keys[k].value != y.keys[k].value) return false;
    }
    return true;
}

} // namespace

void Scene::pushUndo(std::string label) {
    ++m_undoPushes;
    Snapshot s;
    s.id = m_undoPushes;
    s.label = std::move(label);
    s.bodies = m_bodies;
    s.motions = m_motions;
    s.before = m_revision;
    m_undo.push_back(std::move(s));
    m_redo.clear(); // a new edit: what was undone is gone for good
}

void Scene::pushSketchUndo(std::string label) {
    ++m_undoPushes;
    Snapshot s;
    s.id = m_undoPushes;
    s.label = std::move(label);
    s.sketchOnly = true;
    s.sketch = m_sketch;
    s.before = m_sketch.revision();
    m_undo.push_back(std::move(s));
}

void Scene::beginBodyEdit(std::string label) {
    PendingEdit p;
    p.revision = m_revision;
    for (const Body& b : m_bodies) p.bodies.push_back({b.name, b.offset, b.color, b.visible}), p.physics.push_back(b.physics), p.materials.push_back(b.material), p.properties.push_back(b.properties);
    p.motions = motionsKey(m_motions);
    p.programs = m_motions;
    p.redo = m_redo; // pushUndo drops it; put back if nothing comes of this
    pushUndo(std::move(label));
    p.id = m_undoPushes;
    m_pending.push_back(std::move(p));
}

namespace {

bool sameVec(const Vec3& a, const Vec3& b) { return a.x == b.x && a.y == b.y && a.z == b.z; }

bool samePhysics(const PhysicsProps& a, const PhysicsProps& b) {
    return a.enabled == b.enabled && a.fixed == b.fixed && a.mass == b.mass && a.density == b.density && sameVec(a.velocity, b.velocity) &&
           sameVec(a.spin, b.spin) && a.charge == b.charge && a.temperature == b.temperature && a.specificHeat == b.specificHeat &&
           a.restitution == b.restitution && sameVec(a.force, b.force) && a.wave == b.wave && a.waveFrequency == b.waveFrequency &&
           a.waveSpeed == b.waveSpeed;
}

bool sameMaterial(const MaterialProps& a, const MaterialProps& b) {
    return a.preset == b.preset && a.metallic == b.metallic && a.roughness == b.roughness && a.transmission == b.transmission && a.ior == b.ior &&
           a.emission == b.emission && a.colorMap == b.colorMap && a.normalMap == b.normalMap;
}

} // namespace

void Scene::endBodyEdit() {
    if (m_pending.empty()) return;
    PendingEdit p = std::move(m_pending.back());
    m_pending.pop_back();
    // The edit may have kept a snapshot of its own (then that is the undo step), or taken one
    // and put it back because it failed (then the bodies are as they were, at a new revision).
    bool innerKept = false;
    for (const Snapshot& u : m_undo) innerKept = innerKept || u.id > p.id;
    const bool innerUndone = m_undoPushes != p.id && !innerKept;
    bool changed = (m_revision != p.revision && !innerUndone) || m_bodies.size() != p.bodies.size() || motionsKey(m_motions) != p.motions ||
                   !sameMotionPrograms(m_motions, p.programs);
    for (std::size_t i = 0; !changed && i < m_bodies.size(); ++i) {
        const Body& b = m_bodies[i];
        const BodyState& w = p.bodies[i];
        changed = b.name != w.name || b.visible != w.visible || b.offset.x != w.offset.x || b.offset.y != w.offset.y || b.offset.z != w.offset.z ||
                  b.color.x != w.color.x || b.color.y != w.color.y || b.color.z != w.color.z || !samePhysics(b.physics, p.physics[i]) ||
                  !sameMaterial(b.material, p.materials[i]) || b.properties != p.properties[i];
    }
    if (changed && !innerKept) return; // an undo step: sealed by the caller
    for (std::size_t i = m_undo.size(); i-- > 0;)
        if (m_undo[i].id == p.id) {
            m_undo.erase(m_undo.begin() + static_cast<std::ptrdiff_t>(i));
            break;
        }
    if (innerKept) return;
    // Nothing came of it: as though no snapshot was taken. The edit before can still be
    // undone (the state it left was only restored), and what could be redone still can.
    if (!m_undo.empty() && m_undo.back().sealed && !m_undo.back().sketchOnly && m_undo.back().after == p.revision) m_undo.back().after = m_revision;
    if (!innerUndone) --m_undoPushes;
    m_redo = std::move(p.redo);
}

void Scene::dropSketchUndo() {
    // The newest unsealed sketch snapshot (a body edit may have pushed one after it).
    for (std::size_t i = m_undo.size(); i-- > 0;)
        if (m_undo[i].sketchOnly && !m_undo[i].sealed) {
            m_undo.erase(m_undo.begin() + static_cast<std::ptrdiff_t>(i));
            --m_undoPushes;
            return;
        }
}

void Scene::sealUndo() {
    if (m_undo.empty() || m_undo.back().sealed) return;
    Snapshot& s = m_undo.back();
    s.sealed = true;
    if (s.sketchOnly) {
        s.sketchAfter = m_sketch.revision();
        if (m_undo.size() > 16) m_undo.erase(m_undo.begin());
        return;
    }
    s.after = m_revision;
    s.afterBodies.clear();
    for (const Body& b : m_bodies) s.afterBodies.push_back({b.name, b.offset, b.color, b.visible});
    s.afterMotions = motionsKey(m_motions);
    s.motionsChanged = s.afterMotions != motionsKey(s.motions) || !sameMotionPrograms(s.motions, m_motions);
    // At most 16 steps and about 2 million vertices of history (trimmed only once an
    // edit has succeeded, so a failed one costs no history).
    std::size_t vertices = 0;
    for (std::size_t i = m_undo.size(); i-- > 0;) {
        for (const Body& b : m_undo[i].bodies) vertices += b.mesh.vertices.size();
        if (m_undo.size() - i > 16 || (vertices > 2000000 && i + 1 < m_undo.size())) {
            m_undo.erase(m_undo.begin(), m_undo.begin() + static_cast<std::ptrdiff_t>(i + 1));
            break;
        }
    }
}

std::string Scene::undo(std::string* why) {
    if (m_undo.empty()) return "";
    Snapshot s = std::move(m_undo.back());
    m_undo.pop_back();
    if (s.sketchOnly) {
        if (s.sealed && m_sketch.revision() != s.sketchAfter) {
            if (why) *why = "Cannot undo " + s.label + ": the sketch changed since (undo history cleared)";
            m_undo.clear();
            return "";
        }
        Redo r;
        r.label = s.label;
        r.sketchOnly = true;
        r.sketch = m_sketch;
        m_sketch = std::move(s.sketch);
        m_sketch.touch();
        r.sketchAt = m_sketch.revision();
        if (s.sealed) m_redo.push_back(std::move(r));
        // The sketch edit before this one left the state now restored: it can be undone next.
        if (!m_undo.empty() && m_undo.back().sketchOnly && m_undo.back().sealed && m_undo.back().sketchAfter == s.before)
            m_undo.back().sketchAfter = m_sketch.revision();
        return s.label;
    }
    if (s.sealed) {
        const char* changed = m_revision != s.after || s.afterBodies.size() != m_bodies.size() ? "bodies were added, removed or rebuilt"
                              : s.motionsChanged && motionsKey(m_motions) != s.afterMotions ? "the motions changed"
                                                                                            : nullptr;
        if (changed) {
            if (why) *why = std::string("Cannot undo ") + s.label + ": " + changed + " since (undo history cleared)";
            m_undo.clear();
            return "";
        }
        // Keep what was done to the bodies since the edit: moves, colours, renames.
        keepLaterChanges(s.bodies, s.afterBodies);
        Redo r;
        r.label = s.label;
        r.bodies = m_bodies;
        r.motions = m_motions;
        r.from = m_revision;
        if (s.motionsChanged) m_motions = std::move(s.motions);
        m_bodies = std::move(s.bodies);
        m_revision = nextRevision();
        r.at = m_revision;
        for (const Body& b : m_bodies) r.atBodies.push_back({b.name, b.offset, b.color, b.visible});
        m_redo.push_back(std::move(r));
    } else {
        m_motions = std::move(s.motions); // a failed edit: everything as it was
        m_bodies = std::move(s.bodies);
        m_revision = nextRevision();
    }
    // Back to the state before this edit, which the edit before it left (unless
    // something changed in between): that one can be undone next.
    if (!m_undo.empty() && m_undo.back().sealed && m_undo.back().after == s.before) m_undo.back().after = m_revision;
    return s.label;
}

void Scene::keepLaterChanges(std::vector<Body>& bodies, const std::vector<BodyState>& was) const {
    for (std::size_t i = 0; i < m_bodies.size() && i < was.size(); ++i) {
        const BodyState& w = was[i];
        const Body& now = m_bodies[i];
        for (Body& b : bodies) {
            if (b.name != w.name) continue;
            b.offset += now.offset - w.offset;
            if (now.color.x != w.color.x || now.color.y != w.color.y || now.color.z != w.color.z) b.color = now.color;
            if (now.visible != w.visible) b.visible = now.visible;
            b.name = now.name;
            break;
        }
    }
}

std::string Scene::redo(std::string* why) {
    if (m_redo.empty()) return "";
    Redo r = std::move(m_redo.back());
    m_redo.pop_back();
    if (r.sketchOnly) {
        if (m_sketch.revision() != r.sketchAt) {
            if (why) *why = "Cannot redo " + r.label + ": the sketch changed since (redo history cleared)";
            m_redo.clear();
            return "";
        }
        Snapshot s;
        s.label = r.label;
        s.sketchOnly = true;
        s.sketch = m_sketch;
        const std::uint64_t from = m_sketch.revision();
        s.before = from;
        m_sketch = std::move(r.sketch);
        m_sketch.touch();
        m_undo.push_back(std::move(s));
        sealUndo();
        // Back to the state the undo before this one left: that one can be redone next.
        if (!m_redo.empty() && m_redo.back().sketchOnly && m_redo.back().sketchAt == from) m_redo.back().sketchAt = m_sketch.revision();
        return r.label;
    }
    if (m_revision != r.at || r.atBodies.size() != m_bodies.size()) {
        if (why) *why = "Cannot redo " + r.label + ": bodies were added, removed or rebuilt since (redo history cleared)";
        m_redo.clear();
        return "";
    }
    keepLaterChanges(r.bodies, r.atBodies);
    // Undoable again: the state before the redo is the snapshot.
    Snapshot s;
    s.label = r.label;
    s.bodies = m_bodies;
    s.motions = m_motions;
    s.before = m_revision;
    m_bodies = std::move(r.bodies);
    m_motions = std::move(r.motions);
    m_revision = nextRevision();
    m_undo.push_back(std::move(s));
    sealUndo();
    // Back to the state the undo before this one left: that one can be redone next.
    if (!m_redo.empty() && m_redo.back().at == r.from) m_redo.back().at = m_revision;
    return r.label;
}

void Scene::clear() {
    m_bodies.clear();
    m_undo.clear();
    m_redo.clear();
    m_images.clear();
    m_motions.clear();
    m_physics = PhysicsSettings{};
    m_rigs = RigState{};
    m_terrain = TerrainSettings{};
    m_sketch = Sketch{};
    m_revision = nextRevision();
    m_imageRevision = nextRevision();
    m_sketch.touch();
}

void Scene::assign(Scene&& other) {
    m_bodies = std::move(other.m_bodies);
    m_undo = std::move(other.m_undo);
    m_redo = std::move(other.m_redo);
    m_images = std::move(other.m_images);
    m_motions = std::move(other.m_motions);
    m_physics = other.m_physics;
    m_rigs = other.m_rigs;
    m_terrain = other.m_terrain;
    m_sketch = std::move(other.m_sketch);
    m_revision = nextRevision();
    m_imageRevision = nextRevision();
    m_sketch.touch();
}

void Scene::restore(Scene&& other) {
    m_bodies = std::move(other.m_bodies);
    m_undo = std::move(other.m_undo);
    m_redo = std::move(other.m_redo);
    m_images = std::move(other.m_images);
    m_motions = std::move(other.m_motions);
    m_physics = other.m_physics;
    m_rigs = other.m_rigs;
    m_terrain = other.m_terrain;
    m_sketch = std::move(other.m_sketch);
    m_revision = other.m_revision;
    m_imageRevision = other.m_imageRevision;
}

AABB Body::bounds() const noexcept {
    AABB b = mesh.bounds();
    if (b.valid()) {
        b.min += offset;
        b.max += offset;
    }
    return b;
}

void Scene::remove(std::size_t index) {
    if (index >= m_bodies.size()) return;
    const std::string name = m_bodies[index].name;
    m_bodies.erase(m_bodies.begin() + static_cast<std::ptrdiff_t>(index));
    // Nothing stays attached to a body that is gone (a later one of that name would
    // otherwise pick up its armour and pins).
    bool another = false;
    for (const Body& b : m_bodies) another = another || b.name == name;
    if (!another)
        for (Body& b : m_bodies)
            if (b.attachedTo == name) b.attachedTo.clear();
    m_revision = nextRevision();
}

void Scene::renameBody(std::size_t index, const std::string& name) {
    if (index >= m_bodies.size() || name.empty()) return;
    const std::string old = m_bodies[index].name;
    if (old == name) return;
    m_bodies[index].name = name;
    for (Body& b : m_bodies)
        if (b.attachedTo == old) b.attachedTo = name;
    for (Motion& m : m_motions)
        for (std::string& b : m.bodies)
            if (b == old) b = name;
}

std::optional<RayHit> Scene::raycast(const Ray& ray, const std::vector<Mat4>* transforms) const noexcept {
    std::optional<RayHit> best;
    std::vector<std::shared_ptr<const MeshBvh>> bvh; // this revision's, or none yet
    try {
        std::lock_guard<std::mutex> guard(m_pick.lock);
        if (m_pick.built != m_revision || m_pick.bvh.size() != m_bodies.size()) {
            if (m_pick.seen == m_revision) {
                // Built now; bodies whose shape is the same keep theirs.
                std::unordered_map<std::uint64_t, std::shared_ptr<const MeshBvh>> old;
                for (std::size_t i = 0; i < m_pick.bvh.size() && i < m_pick.keys.size(); ++i)
                    if (m_pick.bvh[i]) old[m_pick.keys[i]] = m_pick.bvh[i];
                m_pick.bvh.assign(m_bodies.size(), nullptr);
                m_pick.keys.assign(m_bodies.size(), 0);
                for (std::size_t i = 0; i < m_bodies.size(); ++i) {
                    const MeshData& m = m_bodies[i].mesh;
                    if (m.indices.size() / 3 < 2048) continue; // quick enough as it is
                    const std::uint64_t key = meshContentKey(m);
                    m_pick.keys[i] = key;
                    const auto it = old.find(key);
                    m_pick.bvh[i] = it != old.end() ? it->second : std::make_shared<const MeshBvh>(m);
                }
                m_pick.built = m_revision;
            } else m_pick.seen = m_revision;
        }
        if (m_pick.built == m_revision) bvh = m_pick.bvh;
    } catch (...) {
        bvh.clear(); // test every triangle
    }
    for (std::size_t i = 0; i < m_bodies.size(); ++i) {
        const Body& b = m_bodies[i];
        if (!b.visible) continue;
        // Moved body: bring the ray into its modelled position (rigid, so t is unchanged).
        Ray r = ray;
        if (transforms && i < transforms->size()) {
            const Mat4 inv = inverse((*transforms)[i]);
            r = Ray{inv.transformPoint(ray.origin), inv.transformDir(ray.dir)};
        }
        const MeshBvh* tree = i < bvh.size() ? bvh[i].get() : nullptr;
        AABB box = tree ? tree->bounds() : b.mesh.bounds();
        if (!box.valid()) continue;
        box.min += b.offset;
        box.max += b.offset;
        float tBox = 0.0f;
        if (!intersectAABB(r, box, tBox)) continue;
        if (best && tBox > best->t) continue;

        // Test in the body's local frame.
        const Ray local{r.origin - b.offset, r.dir};
        if (tree) {
            float t = best ? best->t : 1e30f;
            if (tree->raycast(local, t)) best = RayHit{static_cast<int>(i), t, ray.at(t)};
            continue;
        }
        const auto& v = b.mesh.vertices;
        const auto& idx = b.mesh.indices;
        for (std::size_t k = 0; k + 2 < idx.size(); k += 3) {
            float t = 0.0f;
            if (intersectTriangle(local, v[idx[k]].position, v[idx[k + 1]].position, v[idx[k + 2]].position, t) &&
                (!best || t < best->t))
                best = RayHit{static_cast<int>(i), t, ray.at(t)};
        }
    }
    return best;
}

AABB Scene::bounds() const noexcept {
    AABB visible, all;
    for (const Body& b : m_bodies) {
        const AABB bb = b.bounds();
        all.expand(bb);
        if (b.visible) visible.expand(bb);
    }
    visible.expand(m_sketch.bounds());
    for (const ImageRef& img : m_images)
        if (img.visible)
            for (const Vec3& c : img.corners()) visible.expand(c);
    return visible.valid() ? visible : all;
}

std::size_t Scene::triangleCount() const noexcept {
    std::size_t n = 0;
    for (const Body& b : m_bodies)
        if (b.visible) n += b.mesh.indices.size() / 3;
    return n;
}

Scene Scene::makeDemo() {
    using namespace primitives;
    Scene s;

    // Base plate 120 x 80 x 12 mm, sitting on the XY plane.
    s.add(makeBody("Base Plate", "box", {-60, -40, 0, 60, 40, 12}, {}, {0.70f, 0.73f, 0.78f}));

    // Central boss with bore.
    s.add(makeBody("Bearing Boss", "tube", {22, 12, 30}, {0, 0, 12}, {0.82f, 0.62f, 0.32f}));

    // Upright web behind the boss.
    s.add(makeBody("Support Web", "box", {-8, 22, 12, 8, 40, 42}, {}, {0.45f, 0.60f, 0.78f}));

    // Four mounting studs.
    const float sx = 46, sy = 28;
    const Vec3 studs[] = {{-sx, -sy, 12}, {sx, -sy, 12}, {sx, sy, 12}, {-sx, sy, 12}};
    MeshData studMesh;
    for (const Vec3& p : studs) studMesh.append(cylinder(p, 5, 8, 40));
    s.add(meshBody("Mounting Studs", std::move(studMesh), {0.55f, 0.57f, 0.60f}));

    // Datum sphere as a probe target.
    s.add(makeBody("Probe Datum", "sphere", {8}, {-40, 0, 12}, {0.38f, 0.70f, 0.52f}));

    return s;
}

} // namespace sicdm

#pragma once

#include "core/Bvh.hpp"
#include "core/Geometry.hpp"
#include "core/Motion.hpp"
#include "core/Physics.hpp"
#include "core/Rig.hpp"
#include "core/Terrain.hpp"
#include "core/Sketch.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace sicdm {

struct Brep;

// A hole the hole wizard made: where, which way it goes in, and its callout ("M6 tapped
// 12 deep (drill 5 x 15)"), relative to the body's offset.
struct HoleNote {
    Vec3 at{}, dir{0, 0, -1};
    std::string callout;
    float threadDia = 0, pitch = 0; // a tapped hole's thread (0: none)
    float threadDepth = 0;          // how deep it goes (0: through)
    bool threadModelled = false;    // THREAD cut it into the wall
};

struct Body {
    std::string name;
    MeshData mesh;
    Vec3 color{0.72f, 0.75f, 0.80f};
    bool visible = true;
    Vec3 offset{}; // translation applied on top of the mesh
    std::string kind = "mesh"; // see buildPrimitive(); "mesh" stores triangles as-is
    std::vector<float> params;
    float scale = 1.0f;     // uniform size factor about the body origin (library parts, SCALE)
    Vec3 rotation{};        // degrees about X, then Y, then Z, about the body origin (mesh bodies keep it baked in)
    std::string component;  // library component this body belongs to ("" = none)
    std::string attachedTo; // moves rigidly with this body (armour on its part, a hinge pin on its parent)
    std::string assembly;   // the sub-assembly it sits in ("Gearbox/Input shaft"; "" = the top level)
    PhysicsProps physics;   // mass, velocity, charge, temperature... (Simulation physics)
    std::vector<HoleNote> holes; // hole wizard callouts (tapped holes carry a cosmetic thread)
    std::string field;      // a result plot ("von Mises stress, MPa: 0 to 120"): the mesh's mask is the value, 0..1, drawn in colours
    // Exact geometry (STEP imports and the primitives), in the mesh's own frame: good
    // while the mesh is still the one made with it (a sculpt, boolean or remesh leaves it
    // behind, and exports fall back to the mesh's facets).
    std::shared_ptr<const Brep> brep;
    std::uint64_t brepKey = 0; // meshKey() of that mesh
    bool exact() const;

    AABB bounds() const noexcept;
    // Rebuilds a parametric body's mesh from kind, params and scale.
    void rebuild();
    // Sets the size factor (meshes are scaled in place).
    void setScale(float s);
    // Sets the orientation (degrees about X, then Y, then Z).
    void setRotation(const Vec3& degrees);
};

// Rotation by `degrees` about X, then Y, then Z (Rz * Ry * Rx).
Mat4 eulerRotation(const Vec3& degrees) noexcept;

// Body made from an explicit mesh (kind "mesh").
Body meshBody(std::string name, MeshData mesh, Vec3 color = {0.72f, 0.75f, 0.80f});

// Parametric body built by buildPrimitive(); empty mesh if the spec is invalid.
Body makeBody(std::string name, std::string kind, std::vector<float> params, Vec3 offset = {},
              Vec3 color = {0.72f, 0.75f, 0.80f});

// Reference image (scanned drawing, photo, blueprint) laid on a sketch plane.
// Pixels are loaded from `path`; the project file stores the path only.
struct ImageRef {
    std::string name;
    std::string path;
    SketchPlane plane = SketchPlane::Top;
    Vec2 center{};        // plane coordinates (mm)
    float width = 100.0f; // mm; height follows the pixel aspect ratio
    float opacity = 0.6f;
    bool visible = true;
    // Decoded RGBA8 pixels (not saved).
    int pixelWidth = 0, pixelHeight = 0;
    std::vector<unsigned char> pixels;

    float height() const noexcept {
        return pixelWidth > 0 ? width * static_cast<float>(pixelHeight) / static_cast<float>(pixelWidth) : width;
    }
    // Corners in world space: bottom-left, bottom-right, top-right, top-left.
    std::array<Vec3, 4> corners() const noexcept;
};

struct RayHit {
    int body = -1;
    float t = 0.0f;
    Vec3 point;
};

class Scene {
public:
    std::vector<Body>& bodies() noexcept { return m_bodies; }
    const std::vector<Body>& bodies() const noexcept { return m_bodies; }

    std::vector<ImageRef>& images() noexcept { return m_images; }
    const std::vector<ImageRef>& images() const noexcept { return m_images; }
    ImageRef& addImage(ImageRef img) {
        m_imageRevision = nextRevision();
        return m_images.emplace_back(std::move(img));
    }
    void removeImage(std::size_t index);
    // Bumped when images are added/removed (textures must be rebuilt).
    std::uint64_t imageRevision() const noexcept { return m_imageRevision; }

    // Removes everything (new document).
    void clear();
    // Takes over another scene's content with fresh change stamps.
    void assign(Scene&& other);
    // Puts back a copy taken earlier in the same command, change stamps and all (nothing
    // was drawn in between, so the views still hold what it describes).
    void restore(Scene&& other);

    // Motions played by the Simulation window (saved with the project).
    std::vector<Motion>& motions() noexcept { return m_motions; }
    const std::vector<Motion>& motions() const noexcept { return m_motions; }

    // The physical environment (gravity, fields, air...) of the Simulation.
    PhysicsSettings& physics() noexcept { return m_physics; }
    const PhysicsSettings& physics() const noexcept { return m_physics; }
    // Robots: their settings and auto-rigging tags (the tree is read from the parts, see Rig.hpp).
    RigState& rigs() noexcept { return m_rigs; }
    const RigState& rigs() const noexcept { return m_rigs; }
    // The ground robots stand on (drawn as the body "Terrain"; see Terrain.hpp).
    TerrainSettings& terrain() noexcept { return m_terrain; }
    const TerrainSettings& terrain() const noexcept { return m_terrain; }

    Sketch& sketch() noexcept { return m_sketch; }
    const Sketch& sketch() const noexcept { return m_sketch; }

    Body& add(Body body) {
        m_revision = nextRevision();
        return m_bodies.emplace_back(std::move(body));
    }
    void remove(std::size_t index); // bodies attached to it are let go
    // Renames a body; what refers to it by name (attachments, motions) follows.
    void renameBody(std::size_t index, const std::string& name);
    // Call after changing a body's mesh in place (rebuild, setScale).
    void touchBodies() noexcept { m_revision = nextRevision(); }

    // Shape edits (sculpting, booleans, armour, joints) can be undone: a snapshot of
    // the bodies and motions is kept before each one (the last few, within a memory
    // budget). Once the edit has succeeded, sealUndo() records the state it left.
    void pushUndo(std::string label);
    // The same for an edit of the sketch alone (curves, constraints, a drag): a copy of
    // the sketch, restored only while the sketch is as that edit left it.
    // It doesn't drop the redo list until keepSketchUndo() says the edit happened;
    // dropSketchUndo() forgets it (the command failed or changed nothing).
    void pushSketchUndo(std::string label);
    void keepSketchUndo() { m_redo.clear(); }
    void dropSketchUndo();
    // A command or tool that may add, remove, rebuild or move bodies (a primitive, an
    // extrude, an import, a move): beginBodyEdit() takes a snapshot, and endBodyEdit() keeps
    // it as an undo step only when the bodies or motions did change and the edit didn't keep
    // its own snapshot meanwhile. Otherwise nothing is recorded (and the redo list stays).
    // They pair up, and may nest. Seal the step with sealUndo() as for pushUndo().
    void beginBodyEdit(std::string label);
    void endBodyEdit();
    // How many snapshots were ever taken (the stack itself is capped).
    std::uint64_t undoPushes() const noexcept { return m_undoPushes; }
    void sealUndo();
    // Restores the last snapshot; returns its label, or "" when there is none. A sealed
    // snapshot is restored only while no body was added, removed or rebuilt since its
    // edit (else `why` says so and the history is dropped); moves, colours and renames
    // made since are kept. An unsealed one (a failed edit) is restored as it is.
    std::string undo(std::string* why = nullptr);
    std::size_t undoDepth() const noexcept { return m_undo.size(); }
    // Puts back what the last undo took away; returns its label, or "". Refused (and
    // the redo history dropped) once bodies were added, removed or rebuilt since; a
    // new edit drops it too. Moves, colours and renames since the undo are kept.
    std::string redo(std::string* why = nullptr);
    std::size_t redoDepth() const noexcept { return m_redo.size(); }
    // An edit made elsewhere (the CAM jobs): what was undone here can't come back after it.
    void clearRedo() { m_redo.clear(); }

    // Bumped whenever bodies are added or removed (GPU buffers must be rebuilt).
    std::uint64_t revision() const noexcept { return m_revision; }

    // Closest visible body hit by the ray, if any. `transforms` (one rigid
    // transform per body, e.g. from motionTransforms()) moves bodies first.
    std::optional<RayHit> raycast(const Ray& ray, const std::vector<Mat4>* transforms = nullptr) const noexcept;

    // Bounds of visible bodies and sketch curves (or everything if none are visible).
    AABB bounds() const noexcept;

    std::size_t triangleCount() const noexcept;

    // A small machined part shown on the landing canvas.
    static Scene makeDemo();

private:
    std::vector<Body> m_bodies;
    std::uint64_t m_revision = nextRevision();
    // Picking: a BVH per body, for the revision it was built at. It is built the second
    // time a ray is cast at one revision, so a shape changing every frame (a sculpt
    // stroke) doesn't rebuild it each time. Copies of the scene start without one.
    struct PickCache {
        std::mutex lock;
        std::uint64_t built = ~std::uint64_t{0}, seen = ~std::uint64_t{0};
        std::vector<std::shared_ptr<const MeshBvh>> bvh; // per body (null: small enough to test directly)
        std::vector<std::uint64_t> keys;                 // meshContentKey() of each
        PickCache() = default;
        PickCache(const PickCache&) {}
        PickCache& operator=(const PickCache&) { return *this; }
    };
    mutable PickCache m_pick;
    Sketch m_sketch;
    std::vector<ImageRef> m_images;
    std::vector<Motion> m_motions;
    PhysicsSettings m_physics;
    RigState m_rigs;
    TerrainSettings m_terrain;
    std::uint64_t m_imageRevision = nextRevision();
    struct BodyState { // what a later move or rename may change
        std::string name;
        Vec3 offset;
        Vec3 color;
        bool visible = true;
    };
    struct Snapshot {
        std::string label;
        std::vector<Body> bodies;
        std::vector<Motion> motions;
        bool sealed = false;
        std::uint64_t before = 0;        // revision when it was taken
        std::uint64_t after = 0;         // revision the edit left
        std::vector<BodyState> afterBodies;
        std::string afterMotions;        // motionsKey() the edit left
        bool motionsChanged = false;
        bool sketchOnly = false;         // pushSketchUndo: the sketch below, no bodies
        Sketch sketch;
        std::uint64_t sketchAfter = 0;   // the sketch's revision the edit left
        std::uint64_t id = 0;            // m_undoPushes when it was taken
    };
    std::vector<Snapshot> m_undo;
    std::uint64_t m_undoPushes = 0;
    struct Redo {
        std::string label;
        std::vector<Body> bodies;   // as the edit had left them
        std::vector<Motion> motions;
        std::uint64_t at = 0;       // revision the undo left
        std::uint64_t from = 0;     // revision before the undo (what the redo brings back)
        std::vector<BodyState> atBodies;
        bool sketchOnly = false;
        Sketch sketch;              // as the edit had left it
        std::uint64_t sketchAt = 0; // the sketch's revision the undo left
    };
    std::vector<Redo> m_redo;
    struct PendingEdit { // beginBodyEdit() waiting for its endBodyEdit()
        std::uint64_t id = 0, revision = 0;
        std::vector<BodyState> bodies;
        std::string motions;
        std::vector<Motion> programs; // the motions as they were (their keys too)
        std::vector<Redo> redo;
    };
    std::vector<PendingEdit> m_pending;
    // Carries moves, colours, visibility and renames made since `was` onto `bodies`.
    void keepLaterChanges(std::vector<Body>& bodies, const std::vector<BodyState>& was) const;
};

} // namespace sicdm

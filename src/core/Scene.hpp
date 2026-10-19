#pragma once

#include "core/Geometry.hpp"
#include "core/Motion.hpp"
#include "core/Sketch.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace sicdm {

struct Body {
    std::string name;
    MeshData mesh;
    Vec3 color{0.72f, 0.75f, 0.80f};
    bool visible = true;
    Vec3 offset{}; // translation applied on top of the mesh
    std::string kind = "mesh"; // see buildPrimitive(); "mesh" stores triangles as-is
    std::vector<float> params;
    float scale = 1.0f;     // uniform size factor about the body origin (library parts, SCALE)
    std::string component;  // library component this body belongs to ("" = none)

    AABB bounds() const noexcept;
    // Rebuilds a parametric body's mesh from kind, params and scale.
    void rebuild();
    // Sets the size factor (meshes are scaled in place).
    void setScale(float s);
};

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

    // Motions played by the Simulation window (saved with the project).
    std::vector<Motion>& motions() noexcept { return m_motions; }
    const std::vector<Motion>& motions() const noexcept { return m_motions; }

    Sketch& sketch() noexcept { return m_sketch; }
    const Sketch& sketch() const noexcept { return m_sketch; }

    Body& add(Body body) {
        m_revision = nextRevision();
        return m_bodies.emplace_back(std::move(body));
    }
    void remove(std::size_t index);
    // Call after changing a body's mesh in place (rebuild, setScale).
    void touchBodies() noexcept { m_revision = nextRevision(); }

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
    Sketch m_sketch;
    std::vector<ImageRef> m_images;
    std::vector<Motion> m_motions;
    std::uint64_t m_imageRevision = nextRevision();
};

} // namespace sicdm

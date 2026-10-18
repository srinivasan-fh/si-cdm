#pragma once

#include "core/Camera.hpp"
#include "core/Scene.hpp"
#include "ui/Tools.hpp"

#include <imgui.h>

#include <optional>
#include <vector>

namespace sicdm {

struct SnapOptions {
    bool grid = true;
    bool points = true;
};

// Interactive creation of sketch curves on a sketch plane. Works in both the
// 2D drawing window and the 3D window (clicks are projected onto the plane).
class SketchController {
public:
    // Updates the hover point from the cursor. Returns false if the cursor
    // ray misses the plane (e.g. looking edge-on in 3D).
    bool hover(const Scene& scene, const OrbitCamera& cam, SketchPlane plane, ImVec2 mouse, ImVec2 viewport,
               const SnapOptions& snap, bool constrain);

    // Handles a click with a sketch tool; returns a finished curve if any.
    std::optional<SketchCurve> click(ToolId tool, bool doubleClick);

    // Enter: completes an open polyline.
    std::optional<SketchCurve> finish();

    // Exact point typed on the command line (plane coordinates, mm).
    std::optional<SketchCurve> typedPoint(ToolId tool, SketchPlane plane, Vec2 p);

    // Points placed so far for the curve in progress, and the last of them.
    int stage() const noexcept { return static_cast<int>(m_points.size()); }
    std::optional<Vec2> lastPoint() const noexcept {
        return m_points.empty() ? std::nullopt : std::optional<Vec2>(m_points.back());
    }

    void cancel() { m_points.clear(); }
    bool busy() const noexcept { return !m_points.empty(); }

    // Preview, snap marker and live dimension, in screen space.
    void drawOverlay(ImDrawList* dl, const OrbitCamera& cam, ImVec2 viewport, ToolId tool) const;

    std::optional<Vec2> cursor() const noexcept { return m_hasHover ? std::optional<Vec2>(m_hover) : std::nullopt; }

private:
    SketchCurve makeCurve(CurveKind kind, std::vector<Vec2> pts, float radius = 0.0f) const;

    SketchPlane m_plane = SketchPlane::Top;
    std::vector<Vec2> m_points; // clicked so far (plane coordinates)
    Vec2 m_hover{};
    bool m_hasHover = false;
    bool m_snappedToPoint = false;
};

// World point to window coordinates; nullopt when behind the camera.
std::optional<ImVec2> projectToScreen(const OrbitCamera& cam, const Vec3& p, ImVec2 viewport);

// Grid spacing shown by the renderer for this camera (minor lines).
float gridStep(const OrbitCamera& cam);

} // namespace sicdm

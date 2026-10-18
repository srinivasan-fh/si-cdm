#pragma once

#include "core/Camera.hpp"
#include "core/Sketch.hpp"

#include <imgui.h>

#include <optional>

namespace sicdm {

// What the user asked the ViewCube for this frame.
struct ViewCubeResult {
    std::optional<Vec3> viewDirection;       // face / edge / corner clicked (towards the eye)
    std::optional<SketchPlane> drawingPlane; // 2D window: face picked
    bool home = false;
    bool fit = false;
    bool toggleProjection = false;
    bool open3D = false;
    float rotateZ = 0.0f;   // radians, from the rotate arrows
    Vec2 orbitDelta{};      // pixels, from dragging the cube
};

// Autodesk-style navigation cube. Faces, edges and corners are all
// clickable (26 views); hovering highlights the region under the cursor.
class ViewCube {
public:
    // `anchor` is the bottom-right corner the widget sits against.
    ViewCubeResult draw(const OrbitCamera& camera, ImVec2 anchor, bool drawing2D);

private:
    bool m_dragging = false;
    ImVec2 m_pressPos{};
};

} // namespace sicdm

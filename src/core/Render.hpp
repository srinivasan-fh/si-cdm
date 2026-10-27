#pragma once

// Photoreal still images of the scene: a path tracer (clean-room). Rays from the camera
// bounce around the scene picking up light: diffuse and glossy reflection (GGX microfacets),
// metals, clear glass (refraction, Fresnel), glowing materials, soft shadows. The light
// comes from an environment: a photo studio's soft boxes, or a sky with its sun. Those
// lights are sampled directly as well as by the bounces (multiple importance sampling), so
// a few dozen samples a pixel give a clean picture; an edge-aware filter guided by the
// surfaces' colours, normals and depths takes out the rest of the noise. Colour maps and
// normal maps (UV UNWRAP, BAKE) are used on bodies that have them.

#include "core/Camera.hpp"
#include "core/Material.hpp"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace sicdm {

class Scene;

struct RenderSettings {
    std::string environment = "studio"; // studio, sky, white, dark
    float sunAzimuth = 135.0f;           // sky: degrees from +X towards +Y
    float sunElevation = 45.0f;          // sky: degrees above the horizon
    float light = 1.0f;                  // the environment's brightness
    float exposure = 0.0f;               // stops (+1 twice as bright)
    bool floor = true;                   // a floor under the bodies (contact shadows, reflections)
    int width = 1280, height = 800;
    int samples = 64;                    // a pixel
    int bounces = 6;
    bool denoise = true;
};

// What the camera sees: from the app's view, or a default isometric view of everything.
struct RenderCamera {
    Vec3 position, forward, right, up;
    float fovY = 0.785f;   // radians (perspective)
    bool ortho = false;
    float orthoHeight = 1; // world units the image is high (orthographic)
};
RenderCamera renderCamera(const OrbitCamera& camera);
RenderCamera defaultRenderCamera(const Scene& scene, float aspect);

struct RenderStats {
    std::size_t triangles = 0;
    double seconds = 0;
    double raysPerSecond = 0;
};

// Renders into `rgb` (width x height, rows from the top, 8-bit sRGB). `progress` (0..1)
// may return false to stop early (the image is then what was done). False (why) when
// there is nothing visible to render.
bool renderImage(const Scene& scene, const RenderSettings& settings, const RenderCamera& camera, std::vector<std::uint8_t>& rgb,
                 RenderStats* stats = nullptr, std::string* why = nullptr, const std::function<bool(float)>& progress = {});

const std::vector<std::string>& renderEnvironments();

} // namespace sicdm

#pragma once

// Videos of the model, rendered by the path tracer (Render.hpp) frame by frame:
//
//   a turntable   the camera goes round the model (TURNS times), the model still;
//   an animation  the motions, robots and physics played from FROM to TO (as the
//                 simulation shows them), the camera still, or going round as well.
//
// Written as an MJPEG AVI (.avi: plays in VLC, Windows and editors), an animated PNG (.png:
// lossless, plays in web browsers), an animated GIF (.gif: 256 colours, plays anywhere) or
// a folder of numbered PNG frames (a path ending in / or with no extension).

#include "core/Math.hpp"
#include "core/Render.hpp"

#include <cstddef>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace sicdm {

class Scene;

struct VideoRequest {
    std::string path;
    bool animate = false;    // play the motions (else the model as it is at FROM)
    double from = 0;         // seconds into the motions
    double seconds = 0;      // how long the video is (0: an animation's own length, a turntable 6 s)
    int fps = 24;
    float turns = 0;         // times the camera goes round (a turntable: 1 when 0)
    float elevation = 20;    // degrees the camera looks down from (turntable)
    std::optional<StandardView> view; // the still camera's view (animation; default isometric)
    std::optional<RenderCamera> camera; // a camera given (the 3D window's), else fitted
    RenderSettings render;   // size, samples, environment...
    int quality = 90;        // AVI's JPEG quality
};

struct VideoReport {
    std::string format;      // "MJPEG AVI"
    int frames = 0, width = 0, height = 0;
    double fps = 0, seconds = 0, renderSeconds = 0;
    std::size_t bytes = 0;
    bool stopped = false;    // stopped early (what was made is written)
};

// The formats by the path's extension: ".avi", ".png", ".gif", or "" (a frame folder).
// False (why) for another extension.
bool videoFormatOf(const std::string& path, std::string& format, std::string* why = nullptr);

// Renders and writes the video. `progress(done, total)` after each frame may return false
// to stop (the frames so far are written). False (why) when nothing is visible, the
// request doesn't make sense, or the file can't be written.
bool makeVideo(const Scene& scene, const VideoRequest& request, VideoReport* report = nullptr, std::string* why = nullptr,
               const std::function<bool(int, int)>& progress = {});

std::string videoSummary(const VideoReport& r, const std::string& path);

// The encoders (for tests): frames are RGB, rows top to bottom.
std::string encodeAviMjpeg(const std::vector<std::string>& jpegFrames, int width, int height, int fps);
std::string encodeApng(const std::vector<std::vector<std::uint8_t>>& frames, int width, int height, int fps);
std::string encodeGif(const std::vector<std::vector<std::uint8_t>>& frames, int width, int height, int fps);

} // namespace sicdm

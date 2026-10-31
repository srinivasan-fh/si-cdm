#pragma once

// Feature tools that work by picking sketch curves (or a body) in the window, then run
// the matching command: Revolve, Sweep, Helix, Loft, Rib and the surface tools.

#include "core/Sketch.hpp"
#include "ui/Tools.hpp"

#include <string>
#include <vector>

namespace sicdm {

// Settings from the options bar.
struct PickFeatureOptions {
    float revolveAngle = 360.0f; // degrees
    int revolveAxis = 0;         // 0: the plane's vertical axis, 1: horizontal, 2: a picked sketch line
    float helixPitch = 10.0f;    // rise per turn
    float helixTurns = 5.0f;
    bool helixLeft = false;
    bool loftSmooth = true;
    float loftSpacing = 20.0f;   // between sections drawn on one plane
    float ribThickness = 2.0f;
    bool ribFlip = false;
    float surfaceDistance = 20.0f; // tabulated cylinder height, offset surface distance
    bool keepSketch = false;
};

// What a pick tool wants: how many curves (max < 0: any number, finished with Apply or
// Enter), whether a pick is a closed profile, or a body instead of curves.
struct PickFeatureSpec {
    int minPicks = 0;
    int maxPicks = 0;
    bool body = false;
};

bool isPickFeatureTool(ToolId tool) noexcept;
PickFeatureSpec pickFeatureSpec(ToolId tool, const PickFeatureOptions& o) noexcept;
// Pick number `index` (0 based) wants a closed profile (else any curve).
bool pickWantsClosed(ToolId tool, int index) noexcept;
// What to click next, for the options bar and command prompt.
std::string pickFeaturePrompt(ToolId tool, const PickFeatureOptions& o, int picked);

// The command for the picked curves (sketch indices) or body name; empty when more picks
// are needed. `why` says what is missing.
std::string pickFeatureCommand(ToolId tool, const Sketch& sketch, const std::vector<int>& curves, const std::string& body,
                               const PickFeatureOptions& o, std::string* why = nullptr);

// A name as a command word: quoted, inner quotes dropped.
std::string quoteWord(const std::string& name);

} // namespace sicdm

#pragma once

// Wearable armour and exoskeleton tools: turn sculpted or CAD shapes into hollow,
// custom-fit parts over a human (or robot) base.
//
//   Shell        solidify a surface or hollow a solid to a wall thickness, optionally
//                opening it on one side (a helmet, a gauntlet, a chest plate)
//   Armour       a closed shell offset from a body by a clearance gap (padding,
//                wiring, movement) with a wall thickness; it moves with the body
//   Interference sweeps the motion timeline (gestures, walk, physics) and reports
//                every pair of parts that collide or come closer than the clearance
//   Joints       hinge or ball joints that articulate one part against another
//   Lighten      hollows a thick part to a skin and fills it with a gyroid lattice
//                (generative lightweighting for printed parts)

#include "core/Geometry.hpp"
#include "core/Math.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace sicdm {

struct Body;
class Scene;

struct ShellOptions {
    float thickness = 3.0f;  // mm
    bool outward = false;    // grow outwards instead of hollowing inwards
    bool open = false;       // cut an opening first: the side `openNormal` points to goes
    Vec3 openPoint{};
    Vec3 openNormal{0, 0, 1};
};
// Solidifies an open surface into a plate, or hollows a closed solid into a shell.
bool shellBody(Body& body, const ShellOptions& options, std::string* error = nullptr);

// A closed armour shell around `source`: its inner face `clearance` mm off the
// surface, its outer face `thickness` further. Named "Armor <source>".
std::optional<Body> armorFor(const Body& source, float clearance, float thickness, std::string* error = nullptr);

struct InterferencePair {
    std::string a, b;
    float time = 0.0f;     // s, when they come closest
    float distance = 0.0f; // mm; negative: interpenetration depth
    Vec3 at{};             // world point of the closest approach
    float restDistance = 1e30f; // the same in the rest pose (negative: overlapping at a joint)
};
struct InterferenceReport {
    std::vector<InterferencePair> pairs; // closer than the clearance, worst first
    int samples = 0;
    float duration = 0.0f;
    std::size_t checkedPairs = 0;
    std::size_t restOverlaps = 0; // pairs overlapping at rest (joints) that the motion does not push deeper
};
// Checks the given bodies (default: armour parts against everything, or all visible
// bodies when there is no armour) over [0, duration] every `step` seconds. Pairs that
// already overlap in the rest pose (robot links at their joints) are reported only
// when the motion pushes them deeper.
InterferenceReport checkInterference(const Scene& scene, const std::vector<std::size_t>& bodies, float clearance,
                                     float step, float duration);

struct LightenOptions {
    float skin = 2.0f;   // mm of solid wall kept under the surface
    float cell = 12.0f;  // mm, gyroid period
    float wall = 1.2f;   // mm, lattice sheet thickness
    bool hollow = false; // only hollow, no lattice
};
// Replaces the inside of a closed body with a gyroid lattice (or a void).
bool lightenBody(Body& body, const LightenOptions& options, std::string* error = nullptr, double* volumeBefore = nullptr,
                 double* volumeAfter = nullptr);

} // namespace sicdm

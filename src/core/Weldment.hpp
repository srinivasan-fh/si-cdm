#pragma once

// Weldments: frames of structural members (tubes, pipes, bars, angles, channels, beams)
// laid along the straight segments of sketch curves, their ends cut where they meet, and
// the cut list that saws them.
//
//   Profiles   SHS / RHS square and rectangular hollow sections (EN 10219: outer corner
//              radius 2t, inner t; 2.5t and 1.5t over 6 mm), CHS pipe, FLAT and ROUND bar,
//              ANGLE (EN 10056-1 root radii), UPE parallel flange channels (DIN 1026-2),
//              IPE beams (EN 10365); any size of the parametric families, and custom
//              I and C sections, by their dimensions
//   Joints     where two members' ends meet: mitred (default), butt (one stops at the
//              other's face, the other runs through) or left square; where a member's end
//              meets another's side: cut to that side (a T)
//   Cut list   members alike (profile, length, end angles) counted together, with their
//              lengths, angles and masses

#include "core/Brep.hpp"
#include "core/Scene.hpp"

#include <optional>
#include <string>
#include <vector>

namespace sicdm {

struct WeldProfile {
    std::string designation; // "SHS 40x40x3"
    std::string family;      // "SHS"
    // Closed loops in the profile's plane (x across, y up, mm), counter-clockwise. A hollow
    // section's inner loop has as many points as its outer, point k across from point k.
    std::vector<Vec2> outer, inner;
    // Per edge (point k to k + 1, last to first): the arc it lies on (index into arcs), or -1.
    std::vector<int> outerArc, innerArc;
    struct Arc {
        Vec2 centre;
        float radius;
    };
    std::vector<Arc> arcs;
    double area = 0;        // mm^2, exact (round edges as arcs)
    double polygonArea = 0; // the faceted loops' (the mesh's)
    bool hollow() const noexcept { return !inner.empty(); }
};

// A profile from its designation: "SHS 40x40x3", "RHS 60x40x3", "CHS 48.3x3.2", "FLAT 40x6",
// "ROUND 20", "ANGLE 50x50x5", "UPE 120", "IPE 200", "I 200x100x6x9", "C 100x50x5x8". Nothing
// (with why) when it can't be made.
std::optional<WeldProfile> weldProfile(const std::string& designation, std::string* why = nullptr);
// The families, and a family's standard sizes.
std::vector<std::string> weldProfileFamilies();
std::vector<std::string> weldProfileSizes(const std::string& family);

// A member's frame: across (the profile's x), up (its y) and along the line.
struct MemberFrame {
    Vec3 across, up, along;
    Vec3 origin; // where the profile's (0, 0) is at the start of the line
};
MemberFrame memberFrame(const Body::Member& m, const WeldProfile& p);

// A member's solid between its end planes, with its exact surfaces (mesh frame). False
// (with why) when its ends cross or lie along it.
bool buildMember(const Body::Member& m, const WeldProfile& p, MeshData& mesh, std::vector<BSurface>& surfaces, std::string* why = nullptr);

// Every member of `group` cut where it meets the others, and remade where that changed
// ("" every group). Notes what it could not do. Returns how many were remade.
int resolveJoints(Scene& scene, const std::string& group, std::vector<std::string>* notes = nullptr);

struct CutItem {
    std::string profile, material;
    double length = 0;            // mm, tip to tip
    double angle0 = 0, angle1 = 0; // the end cuts off square (degrees)
    double massEach = 0;          // kg
    std::vector<std::string> members;
};
// Members alike counted together ("" every group), longest first within a profile.
std::vector<CutItem> cutList(const Scene& scene, const std::string& group = {});

} // namespace sicdm

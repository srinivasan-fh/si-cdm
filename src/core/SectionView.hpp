#pragma once

// The live section view: a plane through the model, the material on one side of it taken
// away in the 3D window so the inside shows, the cut faces filled and hatched. Nothing in
// the model changes. It reports what the plane cuts: each body's section area and centroid.

#include "core/Json.hpp"
#include "core/Math.hpp"
#include "core/Scene.hpp"

#include <optional>
#include <string>
#include <vector>

namespace sicdm {

struct CommandOutput;
struct Document;

struct SectionView {
    bool on = false;
    Vec3 normal{1, 0, 0};          // unit; points to the side taken away
    double offset = 0;             // the plane: dot(normal, p) == offset (mm)
    bool cap = true;               // the cut faces filled and hatched
    std::vector<std::string> only; // the bodies cut (empty: every body)

    // Positive on the side taken away.
    double distance(const Vec3& p) const noexcept {
        return static_cast<double>(normal.x) * p.x + static_cast<double>(normal.y) * p.y + static_cast<double>(normal.z) * p.z - offset;
    }
    bool cuts(const Body& b) const;
    bool operator==(const SectionView& o) const {
        return on == o.on && normal.x == o.normal.x && normal.y == o.normal.y && normal.z == o.normal.z && offset == o.offset && cap == o.cap && only == o.only;
    }
};

Json sectionToJson(const SectionView& s);
SectionView sectionFromJson(const Json& j);

// What the plane cuts of one body: the section's area (mm^2), its centroid (world) and
// how many closed loops bound it (outlines and holes).
struct SectionCut {
    std::string body;
    double area = 0;
    Vec3 centroid{};
    int loops = 0;
};
// For each visible body the plane cuts (closed solids only: an open surface has no inside).
std::vector<SectionCut> sectionCuts(const Scene& scene, const SectionView& s);

// A pick in the sectioned view: what is taken away can't be hit; a ray through a cut face
// stops on it (the point on the plane, in the body it cuts).
std::optional<RayHit> sectionRaycast(const Scene& scene, const SectionView& s, const Ray& ray, const std::vector<Mat4>* transforms = nullptr);

// The plane, in words ("x = 10, keeping x < 10").
std::string describeSection(const SectionView& s);

// SECTION [X|Y|Z|PLANE nx,ny,nz] [AT d | AT x,y,z] | OFFSET d | MOVE d | FLIP | ON | OFF |
// CAP ON|OFF | ONLY body... | ALL. `args` follow the word SECTION.
CommandOutput runSectionCommand(Document& doc, const std::vector<std::string>& args, const std::vector<bool>& quoted);

} // namespace sicdm

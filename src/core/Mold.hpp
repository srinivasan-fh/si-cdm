#pragma once

// Mold tools: how a part comes out of a mould along a pull direction, where the mould
// opens (the parting line), and the mould's two halves.
//
//   Draft     each face's draft, its angle off the pull: positive faces come away with the
//             cavity (the half that pulls along the pull), negative ones with the core; faces
//             under the minimum draft need more; faces hidden from both directions are
//             undercuts (they would lock the part in)
//   Parting   the line round the part where cavity and core meet: where faces turn from
//             facing the pull to facing away, and on straight walls at the parting height
//   Split     the cavity and the core: a block round the part (with a margin) cut along the
//             part's faces, flat at the parting height outside it, with upright shut-off
//             walls where the parting line steps off that height; the part grown for its
//             shrinkage first

#include "core/Geometry.hpp"

#include <optional>
#include <string>
#include <vector>

namespace sicdm {

enum class DraftClass { Positive, Negative, Shallow, Undercut };

struct DraftReport {
    double area[4] = {0, 0, 0, 0}; // mm^2 by DraftClass
    double minPositive = 90, minNegative = 90; // the least draft (degrees) of the faces that have it
    std::size_t triangles = 0;
};

// Per triangle of `world` (closed or not): its class for pulling along `pull` (unit) with at
// least `minDeg` of draft.
std::vector<DraftClass> draftClasses(const MeshData& world, Vec3 pull, float minDeg, DraftReport* report = nullptr);
// The same mesh coloured by class (each triangle on its own, painted: green positive, red
// negative, yellow too little draft, purple undercut).
MeshData draftColoured(const MeshData& world, const std::vector<DraftClass>& classes);
// The colours, for legends.
Vec3 draftColour(DraftClass c) noexcept;

struct PartingLine {
    std::vector<std::vector<Vec3>> loops; // closed, the first point not repeated
    double height = 0;                    // the parting plane, along the pull
    double low = 0, high = 0;             // the loops' heights (equal: flat)
    double length = 0;
};

// The parting line of a closed part along `pull` at the parting height `at` (along the
// pull), or the one that keeps the shut-off walls least. False (with why) when the part
// isn't closed or its parting line can't be found.
bool partingLine(const MeshData& world, Vec3 pull, std::optional<double> at, PartingLine& out, std::string* why = nullptr);

struct MoldOptions {
    Vec3 pull{0, 0, 1};
    std::optional<double> at;  // the parting height (along the pull)
    float margin = 0;          // the block round the part (mm; 0: a quarter of its size, at least 5)
    float shrink = 0;          // percent the part is grown by before (its shrinkage)
};
struct MoldHalves {
    MeshData cavity, core;     // closed, outward
    PartingLine parting;
    double block = 0;          // the block's volume (mm^3)
    double part = 0;           // the part's, grown
    double undercut = 0;       // mm^2 of undercut faces (none made then)
};
// The cavity and core of a closed part. False (with why) for an open part, one with
// undercuts along the pull, or a parting line that folds over itself.
bool splitMold(const MeshData& world, const MoldOptions& o, MoldHalves& out, std::string* why = nullptr);

} // namespace sicdm

#pragma once

// Hole wizard: standard hole sizes (ISO metric coarse M1.6 to M24 and UNC #2 to 1"), with
// the clearance, tap drill, counterbore and countersink sizes that go with them, and the
// shape of a drilled, counterbored, countersunk or tapped hole.
//
// Sizes come from the usual tables: ISO 273 clearance holes (close / normal / loose),
// ISO 261/262 coarse threads and their tap drills, counterbores for ISO 4762 socket head
// cap screws, countersinks (90 degrees) for ISO 10642 flat heads; for UNC the ASME
// B18.3 / B18.6 counterpart sizes (82 degree countersinks). All lengths in mm.

#include "core/Math.hpp"

#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace sicdm {

struct HoleSize {
    const char* name;  // "M6", "1/4-20"
    float nominal;     // the thread's major diameter
    float pitch;       // mm (UNC: 25.4 / threads per inch)
    float tapDrill;
    float clearance[3]; // close, normal, loose fit
    float cboreDia, cboreDepth;
    float csinkDia;    // the countersink's diameter at the surface
    float csinkAngle;  // its included angle
    bool inch;
};

const std::vector<HoleSize>& holeSizes();
// A size by name: "M6", "m6", "M6x1", "1/4-20", "#10-24", "10-24" (nullptr when unknown).
const HoleSize* findHoleSize(const std::string& name);

enum class HoleFit { Close, Normal, Loose };

// One hole's shape.
struct HoleSpec {
    float diameter = 0;      // the drilled hole
    float depth = 0;         // from the surface; 0: through
    float cboreDia = 0, cboreDepth = 0;
    float csinkDia = 0, csinkAngle = 90;
    bool tipped = true;      // a blind hole ends in a 118 degree drill point
    // Tapped holes: the thread (cosmetic: the hole is the tap drill).
    float threadDia = 0, threadPitch = 0, threadDepth = 0; // threadDepth 0: as deep as the hole
    std::string size;        // the standard size's name, if any
};

// The hole's outline turned about its axis: (radius, height along the outward normal) from
// the bottom of the axis round to the top, `through` deep when it goes through, rising
// `margin` above the surface. False (with why) when it doesn't make sense.
bool holeProfile(const HoleSpec& h, float through, float margin, std::vector<std::pair<float, float>>& out, std::string* why = nullptr);

// "M6 tapped 12 deep (drill 5 x 15)" / "6.6 thru, counterbore 11 x 6.4" style callout.
std::string holeCallout(const HoleSpec& h);

} // namespace sicdm

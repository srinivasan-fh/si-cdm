#pragma once

// Standard fasteners: hex bolts, socket head cap screws, countersunk and button head
// screws, set screws, hex nuts, plain washers and dowel pins, in ISO metric and ASME inch
// sizes. A fastener is a library body of kind "fastener" with params
//   type size length
// (type: FastenerType; size: an index into fastenerSizes(type); length in mm, unused by
// nuts and washers). Its origin is the bearing face (a countersunk screw's: the top of its
// head, flush with the part) and its shank runs down -Z; a dowel pin is centred on the
// origin. Threads are cosmetic: the shank is modelled at the nominal diameter.
//
// Sizes follow ISO 4017, 4762, 10642, 7380, 4026, 4032, 7089 and 8734, and ASME B18.2.1,
// B18.2.2, B18.3, B18.6.3, B18.22.1 and B18.8.2 (the dimensions that matter for fit: head
// and nut sizes, heights, socket sizes). Chamfers and the socket's bottom are simplified.

#include "core/Brep.hpp"
#include "core/Geometry.hpp"

#include <optional>
#include <string>
#include <vector>

namespace sicdm {

struct Body;

enum class FastenerType : int { HexBolt, SocketCap, Countersunk, ButtonHead, SetScrew, HexNut, Washer, DowelPin, Count };

struct FastenerSize {
    std::string name;  // "M6", "1/4-20", "6" (a pin's diameter), "1/4"
    bool inch = false;
    float d = 0;       // nominal (thread major) diameter, a pin's diameter (mm)
    float pitch = 0;   // thread pitch (0: a pin)
    float head = 0;    // head diameter, across flats (hex), or a washer's outside diameter
    float height = 0;  // head height, nut height, washer thickness
    float socket = 0;  // hex socket across flats (0: none)
    float depth = 0;   // socket depth
    float hole = 0;    // a washer's inside diameter
};

const std::vector<FastenerSize>& fastenerSizes(FastenerType type);
// "Socket head cap screw"
const char* fastenerTypeName(FastenerType type);
// The library item's key: "socket_head_cap_screw"
const char* fastenerKey(FastenerType type);
// The standard a size is made to: "ISO 4762", "ASME B18.3"
const char* fastenerStandard(FastenerType type, const FastenerSize& size);
// The sizes as a choice list for the library ("M1.6|M2|...").
const char* fastenerChoices(FastenerType type);
// A size by name ("M6", "m6", "1/4-20", "#10-24", "10-24"; pins "6", "6mm", "1/4"): its
// index, or -1.
int findFastenerSize(FastenerType type, const std::string& name);
// Nuts and washers have no length.
bool fastenerHasLength(FastenerType type);
// The shortest length modelled (a sound shape, and at least the diameter; mm), and the
// standard lengths of a type in a size's units (mm), shortest first.
float minFastenerLength(FastenerType type, int size);
std::vector<float> standardLengths(FastenerType type, bool inch);
// The first standard length at least `needed` (or, `atMost`, the longest no more than it;
// the shortest sound one when none is). Never below minFastenerLength().
float pickFastenerLength(FastenerType type, int size, float needed, bool atMost = false);

// The part (mesh empty when the params don't make a sound one, why says so) and the
// analytic surfaces its faces lie on (for its exact geometry).
MeshData buildFastener(const std::vector<float>& params, std::string* why = nullptr);
std::vector<BSurface> fastenerSurfaces(const std::vector<float>& params);

// "ISO 4762 M6x20", "ASME B18.3 1/4-20 x 3/4", "ISO 4032 M6"
std::string fastenerDesignation(const std::vector<float>& params);
// "Socket head cap screw M6x20"
std::string fastenerDescription(const std::vector<float>& params);
// "Steel 12.9"
std::string fastenerMaterial(FastenerType type, bool inch);
// A length in a size's units: "20" (mm) or "3/4", "1-1/2", "0.787" (inches).
std::string fastenerLengthText(float mm, bool inch);
// A length typed for a size: mm for metric sizes; inches for inch sizes unless it has units
// ("3/4", "1-1/2", "0.75", "20mm"). Nullopt when it isn't one.
std::optional<float> parseFastenerLength(const std::string& text, bool inch);

// Sets a fastener body's NUMBER, DESCRIPTION and THREAD properties from its params (and
// MATERIAL if it has none). Nothing for other bodies.
void applyFastenerProperties(Body& body);

// Black oxide for socket screws, else bright steel.
Vec3 fastenerColour(FastenerType type);

// The rotation (degrees about X, then Y: Body::rotation) that turns -Z onto `dir`.
Vec3 fastenerRotation(const Vec3& dir);

} // namespace sicdm
